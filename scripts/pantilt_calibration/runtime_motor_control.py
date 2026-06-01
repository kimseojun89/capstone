"""
runtime_motor_control.py

캘리브레이션 pose table을 이용한 절대 좌표 모터 제어.

FPGA에 A:pan_steps,tilt_steps 명령을 전송해 pose table 기반으로
안테나 평면 좌표 또는 픽셀 좌표에 정확히 조준한다.

사용법:
    # 안테나 평면 좌표로 직접 이동 (mm)
    python runtime_motor_control.py --goto-antenna 50.0,30.0

    # 픽셀 좌표가 가리키는 안테나 평면 교차점으로 이동
    python runtime_motor_control.py --goto-pixel 640,360 --pan-deg 5.0 --tilt-deg -2.0

    # 인터랙티브: 카메라 창에서 클릭하면 해당 지점으로 모터 이동
    python runtime_motor_control.py --interactive

    # 홈(0,0) 복귀
    python runtime_motor_control.py --home

    # 시리얼 없이 드라이런 (명령 계산만)
    python runtime_motor_control.py --no-serial --goto-antenna 50.0,30.0
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pantilt_calibration_utils import (
    load_intrinsics,
    load_plane,
    load_yaml,
    matrix_from_yaml,
    nearest_pose_entry,
    normalize,
    plane_xy,
    ray_plane_intersection,
    repo_path,
)

STEPS_PER_DEGREE: float = 4096.0 / 360.0  # ≈ 11.378 (CLAUDE.md 기준)


# ──────────────────────────────────────────────────────────────
#  시리얼 유틸
# ──────────────────────────────────────────────────────────────

def _open_serial(port: str, baud: int):
    try:
        import serial
    except ImportError:
        print("[-] pyserial 미설치: pip install pyserial", file=sys.stderr)
        sys.exit(1)
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.1)
    return ser


def _send(ser: Any, msg: str) -> None:
    if ser is not None:
        ser.write(msg.encode())


def _send_goto_abs(ser: Any, pan_steps: int, tilt_steps: int) -> None:
    """A:±pan,±tilt 절대 스텝 명령 전송."""
    cmd = f"A:{pan_steps:+d},{tilt_steps:+d}\n"
    _send(ser, cmd)
    return cmd


# ──────────────────────────────────────────────────────────────
#  데이터 로드
# ──────────────────────────────────────────────────────────────

def _load_pose_entries(path: Path) -> list[dict]:
    data = load_yaml(path)
    entries = data.get("poses")
    if not isinstance(entries, list):
        raise ValueError(f"Pose table에 poses 목록이 없습니다: {path}")
    return entries


# ──────────────────────────────────────────────────────────────
#  핵심 기하 연산
# ──────────────────────────────────────────────────────────────

def precompute_center_rays(
    entries: list[dict],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    cam_width: int,
    cam_height: int,
) -> list[dict]:
    """각 pose entry에서 카메라 중심 픽셀 ray의 world 방향을 사전 계산."""
    cu, cv = cam_width / 2.0, cam_height / 2.0
    pixel = np.array([[[cu, cv]]], dtype=np.float64)
    undist = cv2.undistortPoints(pixel, camera_matrix, dist_coeffs)
    xn, yn = undist.reshape(2)
    ray_cam = normalize([xn, yn, 1.0])

    result: list[dict] = []
    for e in entries:
        R = matrix_from_yaml(e["R_camera_to_world"], (3, 3))
        t = matrix_from_yaml(e["t_camera_in_world_mm"], (3,))
        ray_world = normalize(R @ ray_cam)
        result.append({**e, "_ray_world": ray_world, "_cam_origin": t})
    return result


def find_target_pose(
    entries_with_rays: list[dict],
    plane: dict,
    target_x_mm: float,
    target_y_mm: float,
) -> tuple[dict, float]:
    """
    pose table에서 카메라 중심 ray가 안테나 평면의
    (target_x_mm, target_y_mm)에 가장 근접하는 entry를 반환.
    반환값: (entry, 안테나 평면상 거리_mm)
    """
    best_entry: dict | None = None
    best_dist = float("inf")
    for e in entries_with_rays:
        try:
            pt, scale = ray_plane_intersection(
                e["_cam_origin"], e["_ray_world"],
                plane["origin_mm"], plane["normal"],
            )
            if scale < 0:
                continue
            x_mm, y_mm = plane_xy(pt, plane)
            d = math.sqrt((x_mm - target_x_mm) ** 2 + (y_mm - target_y_mm) ** 2)
            if d < best_dist:
                best_dist = d
                best_entry = e
        except ValueError:
            continue
    if best_entry is None:
        raise RuntimeError("유효한 pose entry를 찾을 수 없습니다 — plane 법선과 모든 ray가 평행합니다.")
    return best_entry, best_dist


def localize_pixel(
    u: float,
    v: float,
    pan_deg: float,
    tilt_deg: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    pose_entries: list[dict],
    plane: dict,
    max_angle_error_deg: float = 1.0,
) -> tuple[float, float]:
    """픽셀 (u,v) + 현재 pan/tilt_deg → 안테나 평면 좌표 (x_mm, y_mm)."""
    entry, angle_err = nearest_pose_entry(
        pose_entries, pan_deg, tilt_deg, max_angle_error_deg
    )
    pixel = np.array([[[u, v]]], dtype=np.float64)
    undist = cv2.undistortPoints(pixel, camera_matrix, dist_coeffs)
    xn, yn = undist.reshape(2)
    ray_cam = normalize([xn, yn, 1.0])

    R = matrix_from_yaml(entry["R_camera_to_world"], (3, 3))
    t = matrix_from_yaml(entry["t_camera_in_world_mm"], (3,))
    ray_world = normalize(R @ ray_cam)
    pt, scale = ray_plane_intersection(t, ray_world, plane["origin_mm"], plane["normal"])
    if scale < 0:
        raise RuntimeError(
            f"Ray-plane 교차점이 카메라 뒤에 있습니다 (scale={scale:.3f}). "
            "pose table / antenna plane 좌표계를 확인하세요."
        )
    return plane_xy(pt, plane)


def deg_to_steps(deg: float, steps_per_deg: float = STEPS_PER_DEGREE) -> int:
    return int(round(deg * steps_per_deg))


# ──────────────────────────────────────────────────────────────
#  동작 모드
# ──────────────────────────────────────────────────────────────

def run_home(ser: Any) -> None:
    cmd = _send_goto_abs(ser, 0, 0)
    print(f"[home] 명령 전송: {cmd.strip()}")


def run_goto_antenna(
    ser: Any,
    x_mm: float,
    y_mm: float,
    entries_with_rays: list[dict],
    plane: dict,
    steps_per_deg: float,
) -> None:
    entry, dist_mm = find_target_pose(entries_with_rays, plane, x_mm, y_mm)
    pan_steps  = deg_to_steps(float(entry["pan_deg"]),  steps_per_deg)
    tilt_steps = deg_to_steps(float(entry["tilt_deg"]), steps_per_deg)
    cmd = _send_goto_abs(ser, pan_steps, tilt_steps)
    print(
        f"[goto-antenna] 목표=({x_mm:.2f},{y_mm:.2f})mm\n"
        f"  → 선택 pose: pan={entry['pan_deg']:.3f}° tilt={entry['tilt_deg']:.3f}°\n"
        f"  → 안테나 평면 miss={dist_mm:.2f}mm\n"
        f"  → 명령: {cmd.strip()}"
    )


def run_goto_pixel(
    ser: Any,
    u: float,
    v: float,
    pan_deg: float,
    tilt_deg: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    pose_entries: list[dict],
    entries_with_rays: list[dict],
    plane: dict,
    max_angle_error_deg: float,
    steps_per_deg: float,
) -> None:
    x_mm, y_mm = localize_pixel(
        u, v, pan_deg, tilt_deg,
        camera_matrix, dist_coeffs,
        pose_entries, plane, max_angle_error_deg,
    )
    entry, dist_mm = find_target_pose(entries_with_rays, plane, x_mm, y_mm)
    pan_steps  = deg_to_steps(float(entry["pan_deg"]),  steps_per_deg)
    tilt_steps = deg_to_steps(float(entry["tilt_deg"]), steps_per_deg)
    cmd = _send_goto_abs(ser, pan_steps, tilt_steps)
    print(
        f"[goto-pixel] pixel=({u:.0f},{v:.0f}) "
        f"@ pan={pan_deg:.2f}° tilt={tilt_deg:.2f}°\n"
        f"  → 안테나=({x_mm:.2f},{y_mm:.2f})mm\n"
        f"  → 선택 pose: pan={entry['pan_deg']:.3f}° tilt={entry['tilt_deg']:.3f}°\n"
        f"  → 안테나 평면 miss={dist_mm:.2f}mm\n"
        f"  → 명령: {cmd.strip()}"
    )


def run_interactive(
    args: argparse.Namespace,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    entries_with_rays: list[dict],
    pose_entries: list[dict],
    plane: dict,
    ser: Any,
    steps_per_deg: float,
) -> None:
    """
    카메라 라이브 피드에서 마우스로 클릭하면 해당 픽셀이
    안테나 평면과 만나는 좌표로 모터를 이동한다.
    스크립트가 현재 pan/tilt steps를 누적 추적한다.
    """
    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        print(f"[-] 카메라 인덱스 {args.camera_index} 열기 실패", file=sys.stderr)
        return

    cur_pan_steps  = 0
    cur_tilt_steps = 0
    status_msg     = "클릭하면 해당 지점으로 이동합니다 | h=홈 | q/ESC=종료"

    click: dict[str, float | None] = {"u": None, "v": None}

    def on_mouse(event: int, x: int, y: int, flags: int, param: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            click["u"] = float(x)
            click["v"] = float(y)

    cv2.namedWindow("runtime_motor_control", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("runtime_motor_control", on_mouse)
    print("[interactive] 카메라 창에서 클릭하면 해당 지점으로 모터가 이동합니다.")
    print("  h: 홈(0,0) 복귀  |  q/ESC: 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[-] 프레임 읽기 실패", file=sys.stderr)
            break

        cur_pan_deg  = cur_pan_steps  / steps_per_deg
        cur_tilt_deg = cur_tilt_steps / steps_per_deg

        # 현재 위치 표시
        info = (
            f"pan={cur_pan_deg:+.2f}deg ({cur_pan_steps:+d}steps)  "
            f"tilt={cur_tilt_deg:+.2f}deg ({cur_tilt_steps:+d}steps)"
        )
        cv2.putText(frame, info, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, status_msg, (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        if click["u"] is not None:
            u, v = float(click["u"]), float(click["v"])
            click["u"] = None
            try:
                x_mm, y_mm = localize_pixel(
                    u, v, cur_pan_deg, cur_tilt_deg,
                    camera_matrix, dist_coeffs,
                    pose_entries, plane,
                    args.max_angle_error_deg,
                )
                entry, dist_mm = find_target_pose(entries_with_rays, plane, x_mm, y_mm)
                tpan  = deg_to_steps(float(entry["pan_deg"]),  steps_per_deg)
                ttilt = deg_to_steps(float(entry["tilt_deg"]), steps_per_deg)
                cmd = _send_goto_abs(ser, tpan, ttilt)
                cur_pan_steps  = tpan
                cur_tilt_steps = ttilt
                status_msg = (
                    f"antenna=({x_mm:.1f},{y_mm:.1f})mm  "
                    f"pose pan={entry['pan_deg']:.2f}° tilt={entry['tilt_deg']:.2f}°  "
                    f"miss={dist_mm:.1f}mm"
                )
                print(
                    f"[click] ({u:.0f},{v:.0f}) → antenna=({x_mm:.2f},{y_mm:.2f})mm "
                    f"→ {cmd.strip()} (miss={dist_mm:.1f}mm)"
                )
                cv2.drawMarker(
                    frame, (int(u), int(v)), (0, 0, 255),
                    cv2.MARKER_CROSS, 24, 2,
                )
                label = f"({x_mm:.1f},{y_mm:.1f})mm"
                cv2.putText(
                    frame, label, (int(u) + 12, int(v) - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA,
                )
            except Exception as exc:
                status_msg = f"오류: {exc}"
                print(f"[-] {exc}", file=sys.stderr)

        cv2.imshow("runtime_motor_control", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("h"):
            cmd = _send_goto_abs(ser, 0, 0)
            cur_pan_steps  = 0
            cur_tilt_steps = 0
            status_msg = "홈(0,0) 복귀 완료"
            print(f"[home] {cmd.strip()}")

    cap.release()
    cv2.destroyAllWindows()


# ──────────────────────────────────────────────────────────────
#  CLI
# ──────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # 캘리브레이션 파일
    p.add_argument("--intrinsics",  type=Path, default=repo_path("data", "camera_intrinsics.yaml"))
    p.add_argument("--pose-table",  type=Path, default=repo_path("data", "pantilt_pose_table.yaml"))
    p.add_argument("--plane",       type=Path, default=repo_path("data", "antenna_plane.yaml"))
    # 카메라 해상도 (center ray 계산용, 실제 캡처 해상도와 일치해야 함)
    p.add_argument("--cam-width",   type=int,   default=1920)
    p.add_argument("--cam-height",  type=int,   default=1080)
    # 시리얼
    p.add_argument("--serial-port", default="COM4")
    p.add_argument("--baud",        type=int,   default=256000)
    p.add_argument("--no-serial",   action="store_true", help="시리얼 없이 계산 결과만 출력")
    # 모터 파라미터
    p.add_argument("--steps-per-degree", type=float, default=STEPS_PER_DEGREE,
                   help=f"스텝/도 변환 계수 (기본: {STEPS_PER_DEGREE:.4f})")
    p.add_argument("--max-angle-error-deg", type=float, default=1.0,
                   help="pose table 근처 각도 허용 오차 (기본: 1.0deg)")

    # 동작 모드 (상호 배타)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--interactive",    action="store_true",
                   help="카메라 클릭 인터랙티브 모드")
    g.add_argument("--goto-antenna",   metavar="X,Y",
                   help="안테나 평면 좌표(mm)로 이동. 예: 50.0,30.0")
    g.add_argument("--goto-pixel",     metavar="U,V",
                   help="픽셀 좌표의 안테나 교차점으로 이동. --pan-deg/--tilt-deg 필요")
    g.add_argument("--home",           action="store_true",
                   help="홈(0,0) 복귀")

    # --goto-pixel 전용
    p.add_argument("--pan-deg",  type=float, default=0.0,
                   help="현재 pan 각도 (--goto-pixel 전용)")
    p.add_argument("--tilt-deg", type=float, default=0.0,
                   help="현재 tilt 각도 (--goto-pixel 전용)")
    # --interactive 전용
    p.add_argument("--camera-index", type=int, default=1,
                   help="카메라 인덱스 (--interactive 전용, 기본: 1)")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    # 캘리브레이션 데이터 로드
    print(f"[load] intrinsics: {args.intrinsics}")
    camera_matrix, dist_coeffs, _ = load_intrinsics(args.intrinsics)
    print(f"[load] pose-table: {args.pose_table}")
    pose_entries = _load_pose_entries(args.pose_table)
    print(f"[load] antenna-plane: {args.plane}")
    plane = load_plane(args.plane)

    spd = args.steps_per_degree
    print(f"[config] steps_per_degree={spd:.4f}, "
          f"cam={args.cam_width}x{args.cam_height}, "
          f"pose_entries={len(pose_entries)}")

    # 중심 ray 사전 계산 (pose table 역방향 탐색용)
    entries_with_rays = precompute_center_rays(
        pose_entries, camera_matrix, dist_coeffs,
        args.cam_width, args.cam_height,
    )

    # 시리얼 연결
    ser = None
    if not args.no_serial:
        print(f"[serial] {args.serial_port} @ {args.baud}bps 연결 중...", end=" ", flush=True)
        ser = _open_serial(args.serial_port, args.baud)
        print("OK")
    else:
        print("[serial] no-serial 모드 — 명령 계산만 수행")

    try:
        if args.home:
            run_home(ser)

        elif args.goto_antenna:
            parts = args.goto_antenna.split(",")
            if len(parts) != 2:
                print("[-] --goto-antenna 형식: X,Y (예: 50.0,30.0)", file=sys.stderr)
                sys.exit(1)
            run_goto_antenna(
                ser,
                float(parts[0]), float(parts[1]),
                entries_with_rays, plane, spd,
            )

        elif args.goto_pixel:
            parts = args.goto_pixel.split(",")
            if len(parts) != 2:
                print("[-] --goto-pixel 형식: U,V (예: 640,360)", file=sys.stderr)
                sys.exit(1)
            run_goto_pixel(
                ser,
                float(parts[0]), float(parts[1]),
                args.pan_deg, args.tilt_deg,
                camera_matrix, dist_coeffs,
                pose_entries, entries_with_rays,
                plane,
                args.max_angle_error_deg,
                spd,
            )

        elif args.interactive:
            run_interactive(
                args,
                camera_matrix, dist_coeffs,
                entries_with_rays, pose_entries, plane,
                ser, spd,
            )

    finally:
        if ser is not None:
            ser.close()
            print("[serial] 포트 닫힘")


if __name__ == "__main__":
    main()
