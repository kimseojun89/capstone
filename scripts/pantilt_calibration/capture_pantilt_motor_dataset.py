from __future__ import annotations

"""capture_pantilt_motor_dataset.py
-----------------------------------------
Pan/tilt ChArUco 캘리브레이션용 데이터셋 캡처 스크립트.
FPGA UART0 프로토콜에 맞춰 pan/tilt 모터를 제어하면서
각 pan/tilt 각도에서 ChArUco 보드 이미지를 저장한다.

FPGA 시리얼 프로토콜 (256000 baud, 8N1, DTR/RTS off):
    P:+N\n   pan을 N step (상대 이동)
    T:+N\n   tilt를 N step (상대 이동)

    N은 부호 포함 10진수 정수. 예: "P:+30\n", "T:-12\n"
    HOME 명령 없음 → 스크립트 시작 시 현재 위치를 0으로 간주.

조작 키 (OpenCV 창 포커스 필요):
    Left / A  : pan  -step
    Right / D : pan  +step
    Up    / W : tilt +step
    Down  / S : tilt -step
    Space     : 현재 프레임을 pan/tilt 메타데이터와 함께 저장
    R         : 현재 commanded 위치를 pan=0, tilt=0 으로 재설정
    +/-       : angle_step_deg 증가/감소 (0.5 deg 단위)
    Q / ESC   : 종료
"""

import argparse
import time
from pathlib import Path

import cv2

from pantilt_calibration_utils import (
    DEFAULT_DICTIONARY_NAME,
    DEFAULT_MARKER_LENGTH_MM,
    DEFAULT_SQUARE_LENGTH_MM,
    DEFAULT_SQUARES_X,
    DEFAULT_SQUARES_Y,
    configure_capture,
    count_ids,
    create_charuco_board,
    detect_charuco,
    draw_detection,
    ensure_dir,
    open_capture,
    repo_path,
    save_json,
    save_yaml,
    sleep_for_ui,
    timestamp_name,
    utc_now_iso,
)

# ---------------------------------------------------------------------------
# 키 코드 (cv2.waitKeyEx on Windows)
# ---------------------------------------------------------------------------
KEY_ESC = 27
KEY_SPACE = 32
# Windows DirectInput 확장 코드 + Qt/X11 코드 모두 포함
KEY_LEFT  = {81, 65361, 2424832}  # Left arrow
KEY_UP    = {82, 65362, 2490368}  # Up arrow
KEY_RIGHT = {83, 65363, 2555904}  # Right arrow
KEY_DOWN  = {84, 65364, 2621440}  # Down arrow
KEY_PLUS  = {ord("+"), ord("=")}
KEY_MINUS = {ord("-"), ord("_")}


# ---------------------------------------------------------------------------
# 인수 파싱
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture pan/tilt ChArUco dataset while controlling motors via arrow keys. "
            "FPGA serial protocol: P:±N\\n  T:±N\\n  (relative steps, 256000 baud)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # 카메라
    parser.add_argument("--camera-index", type=int, default=1, help="OpenCV camera index")
    parser.add_argument(
        "--camera-api",
        choices=["default", "any", "dshow", "msmf"],
        default="dshow",
        help="Camera backend (dshow recommended on Windows)",
    )
    # 실제 운용 해상도 1920×1080 (settings.hpp cameraWidth/cameraHeight 기본값 일치)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument(
        "--flush-frames",
        type=int,
        default=4,
        help="Number of frames to discard after motor move to clear camera buffer lag.",
    )
    # 시리얼
    parser.add_argument("--serial-port", default="COM4", help="FPGA UART port")
    parser.add_argument("--baud", type=int, default=256000)
    parser.add_argument(
        "--no-serial",
        action="store_true",
        help="Dry-run without serial. Keys still update displayed angles.",
    )
    parser.add_argument(
        "--settle-sec",
        type=float,
        default=0.4,
        help="Seconds to wait after sending motor command before reading camera.",
    )
    # 초기 상대 이동 (선택)
    parser.add_argument(
        "--init-pan-steps",
        type=int,
        default=0,
        help="Relative pan steps sent once at startup (e.g. to return to approximate home).",
    )
    parser.add_argument(
        "--init-tilt-steps",
        type=int,
        default=0,
        help="Relative tilt steps sent once at startup.",
    )
    parser.add_argument("--init-settle-sec", type=float, default=0.8)
    # 각도 설정
    parser.add_argument(
        "--angle-step-deg", type=float, default=1.0, help="Degrees moved per key press."
    )
    parser.add_argument(
        "--steps-per-degree",
        type=float,
        default=4096.0 / 360.0,
        help="Motor steps per degree (FPGA stepper resolution).",
    )
    # 저장
    parser.add_argument(
        "--output-dir", type=Path, default=repo_path("data", "pantilt")
    )
    parser.add_argument("--prefix", default="pantilt_motor")
    parser.add_argument("--ext", choices=["png", "jpg"], default="png")
    # ChArUco 보드 설정
    parser.add_argument("--squares-x", type=int, default=DEFAULT_SQUARES_X)
    parser.add_argument("--squares-y", type=int, default=DEFAULT_SQUARES_Y)
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_LENGTH_MM)
    parser.add_argument("--marker-mm", type=float, default=DEFAULT_MARKER_LENGTH_MM)
    parser.add_argument("--dictionary", default=DEFAULT_DICTIONARY_NAME)
    # 저장 조건
    parser.add_argument(
        "--require-charuco",
        action="store_true",
        help="Skip save if fewer than --min-corners ChArUco corners detected.",
    )
    parser.add_argument("--min-corners", type=int, default=4)
    parser.add_argument("--show-detections", action="store_true")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# 시리얼 모터 제어
# ---------------------------------------------------------------------------
class MotorSerial:
    """FPGA UART0 pan/tilt 모터 제어 (상대 step 명령).

    프로토콜: "P:+30\\n", "T:-12\\n"  (부호 필수, LF 종단)
    """

    def __init__(self, port: str, baud: int) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required. Install with: python -m pip install pyserial"
            ) from exc

        # DTR/RTS 비활성화 (FPGA와 맞춤, unified_gui.py 참고)
        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0,
            write_timeout=1.0,
            dsrdtr=False,
            rtscts=False,
        )
        self.port = port
        self.baud = baud
        print(f"[Motor] Opened serial {port} @ {baud} bps")

    def close(self) -> None:
        self._serial.close()

    def send_pan(self, delta_steps: int) -> None:
        """Pan 상대 이동 명령 전송: P:+N\\n"""
        cmd = f"P:{delta_steps:+d}\n".encode("ascii")
        self._serial.write(cmd)
        self._serial.flush()

    def send_tilt(self, delta_steps: int) -> None:
        """Tilt 상대 이동 명령 전송: T:+N\\n"""
        cmd = f"T:{delta_steps:+d}\n".encode("ascii")
        self._serial.write(cmd)
        self._serial.flush()

    def send_manual_mode(self, enable: bool) -> None:
        """FPGA 수동 모드 전환: M:1\\n (minStep 제한 해제) / M:0\\n (자동 모드 복귀)"""
        cmd = f"M:{1 if enable else 0}\n".encode("ascii")
        self._serial.write(cmd)
        self._serial.flush()


class DummyMotor:
    """--no-serial 모드용 더미 모터 (시리얼 없이 각도만 추적)."""

    port = "(no-serial)"
    baud = 0

    def close(self) -> None:
        pass

    def send_pan(self, delta_steps: int) -> None:  # noqa: ARG002
        pass

    def send_tilt(self, delta_steps: int) -> None:  # noqa: ARG002
        pass

    def send_manual_mode(self, enable: bool) -> None:  # noqa: ARG002
        pass


# ---------------------------------------------------------------------------
# 모터 각도/스텝 상태
# ---------------------------------------------------------------------------
class MotorState:
    """누적 각도와 스텝을 추적해 매 키 입력마다 delta step을 계산한다."""

    def __init__(self, steps_per_degree: float, angle_step_deg: float) -> None:
        self.steps_per_degree = float(steps_per_degree)
        self.angle_step_deg = float(angle_step_deg)
        self.pan_deg = 0.0
        self.tilt_deg = 0.0
        self.pan_steps = 0   # 지금까지 보낸 누적 step (반올림 오차 보정용)
        self.tilt_steps = 0

    def zero(self) -> None:
        """현재 물리적 위치를 pan=0, tilt=0 으로 재정의."""
        self.pan_deg = 0.0
        self.tilt_deg = 0.0
        self.pan_steps = 0
        self.tilt_steps = 0

    def move_pan(self, delta_deg: float) -> int:
        """Pan을 delta_deg 만큼 이동. 반환값은 FPGA에 보낼 delta steps."""
        self.pan_deg += delta_deg
        target = round(self.pan_deg * self.steps_per_degree)
        delta = int(target - self.pan_steps)
        self.pan_steps = int(target)
        return delta

    def move_tilt(self, delta_deg: float) -> int:
        """Tilt를 delta_deg 만큼 이동. 반환값은 FPGA에 보낼 delta steps."""
        self.tilt_deg += delta_deg
        target = round(self.tilt_deg * self.steps_per_degree)
        delta = int(target - self.tilt_steps)
        self.tilt_steps = int(target)
        return delta


# ---------------------------------------------------------------------------
# 화면 HUD
# ---------------------------------------------------------------------------
def put_hud(
    image,
    pan_deg: float,
    tilt_deg: float,
    corner_count: int,
    sample_count: int,
    angle_step_deg: float,
    serial_ok: bool,
) -> None:
    lines = [
        f"pan={pan_deg:+.2f}  tilt={tilt_deg:+.2f} deg   corners={corner_count}",
        f"samples={sample_count}  step={angle_step_deg:.2f} deg",
        "Arrows/WASD: move | Space: save | R: zero | +/-: step | Q/ESC: quit",
    ]
    if not serial_ok:
        lines.append("  [DRY-RUN: no serial]")
    y = 28
    for i, line in enumerate(lines):
        color = (0, 255, 0) if i < 3 else (0, 140, 255)
        cv2.putText(image, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)
        y += 26


# ---------------------------------------------------------------------------
# 샘플 저장
# ---------------------------------------------------------------------------
def save_sample(
    output_dir: Path,
    prefix: str,
    session_id: str,
    sample_index: int,
    ext: str,
    frame,
    args: argparse.Namespace,
    state: MotorState,
    corner_count: int,
) -> dict[str, object]:
    sample_id = f"{prefix}_{session_id}_{sample_index:03d}"
    image_path = output_dir / f"{sample_id}.{ext}"
    metadata_path = output_dir / f"{sample_id}.json"

    if not cv2.imwrite(str(image_path), frame):
        raise RuntimeError(f"Failed to write image: {image_path}")

    metadata = {
        "sample_id": sample_id,
        "captured_utc": utc_now_iso(),
        "image_file": image_path.name,
        "metadata_file": metadata_path.name,
        "pan_deg": state.pan_deg,
        "tilt_deg": state.tilt_deg,
        "pan_steps_total": state.pan_steps,
        "tilt_steps_total": state.tilt_steps,
        "steps_per_degree": state.steps_per_degree,
        "angle_step_deg": state.angle_step_deg,
        "charuco_corners_detected": corner_count,
        "width_px": int(frame.shape[1]),
        "height_px": int(frame.shape[0]),
        "camera_index": args.camera_index,
        "camera_api": args.camera_api,
        "serial_port": args.serial_port,
        "baud": args.baud,
        "board": {
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_mm": args.square_mm,
            "marker_length_mm": args.marker_mm,
            "dictionary": args.dictionary,
        },
        "units": "mm",
    }
    save_json(metadata_path, metadata)
    return metadata


# ---------------------------------------------------------------------------
# 이동 후 카메라 버퍼 플러시
# ---------------------------------------------------------------------------
def flush_camera(cap: cv2.VideoCapture, n_frames: int, settle_sec: float) -> None:
    """모터 이동 후 settle_sec 대기 + 버퍼 프레임 n_frames 소비."""
    time.sleep(settle_sec)
    for _ in range(n_frames):
        cap.grab()  # grab()은 retrieve() 없이 버퍼만 비움


# ---------------------------------------------------------------------------
# 메인
# ---------------------------------------------------------------------------
def main() -> None:
    args = parse_args()
    output_dir = ensure_dir(args.output_dir)

    # ChArUco 보드 생성
    board, dictionary = create_charuco_board(
        args.squares_x, args.squares_y, args.square_mm, args.marker_mm, args.dictionary
    )

    # 카메라 열기
    cap = open_capture(args.camera_index, args.camera_api)
    configure_capture(cap, args.width, args.height, args.fps)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera_index}.")
    print(f"[Camera] index={args.camera_index}  api={args.camera_api}")

    # 시리얼 열기
    if args.no_serial:
        motor: MotorSerial | DummyMotor = DummyMotor()
        serial_ok = False
        print("[Motor] --no-serial: running in dry-run mode (no serial commands sent).")
    else:
        motor = MotorSerial(args.serial_port, args.baud)
        serial_ok = True

    state = MotorState(args.steps_per_degree, args.angle_step_deg)
    session_id = timestamp_name()
    samples: list[dict[str, object]] = []
    index_yaml = output_dir / "pantilt_motor_dataset_index.yaml"
    window = "Pan/Tilt Motor ChArUco Capture"

    # 초기 상대 이동 (선택)
    if args.init_pan_steps:
        motor.send_pan(args.init_pan_steps)
    if args.init_tilt_steps:
        motor.send_tilt(args.init_tilt_steps)
    if args.init_pan_steps or args.init_tilt_steps:
        print(f"[Motor] Init move: pan {args.init_pan_steps:+d} steps, tilt {args.init_tilt_steps:+d} steps")
        flush_camera(cap, args.flush_frames, args.init_settle_sec)

    # 시작 위치를 0 으로 정의
    state.zero()

    # FPGA 수동 모드 진입 (minStep 제한 해제 → 1스텝/0.011° 정밀 제어)
    if serial_ok:
        motor.send_manual_mode(True)
        print("[Motor] FPGA manual mode enabled (minStep limit removed)")

    print(
        "[Motor] Current position defined as pan=0, tilt=0.\n"
        "        Controls: Arrows/WASD=move  Space=save  R=zero  +/-=step  Q/ESC=quit"
    )

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] Failed to read frame, retrying...")
                time.sleep(0.05)
                continue

            detection = detect_charuco(frame, board, dictionary)
            corner_count = count_ids(detection["charuco_ids"])

            if args.show_detections:
                preview = draw_detection(frame, detection)
            else:
                preview = frame.copy()

            put_hud(
                preview,
                state.pan_deg,
                state.tilt_deg,
                corner_count,
                len(samples),
                state.angle_step_deg,
                serial_ok,
            )
            cv2.imshow(window, preview)

            key = cv2.waitKeyEx(20)
            if key < 0:
                continue

            # ── 종료 ──
            if key in (KEY_ESC, ord("q"), ord("Q")):
                break

            # ── 이동 키 ──
            # 펌웨어(ps_main.cpp) 구조:
            #   g_host_pan_steps != 0 일 때만 motor_update_full_host() 진입.
            #   → Tilt 단독 이동 시에도 P:+1\n을 함께 보내 트리거.
            #     1 step < MOTOR_PAN_MIN_STEP(5) → 실제 pan 이동 없음.
            moved = False
            delta_steps = 0
            axis = ""

            if key in KEY_LEFT or key in (ord("a"), ord("A")):
                delta_steps = state.move_pan(-state.angle_step_deg)
                motor.send_pan(delta_steps)
                motor.send_tilt(0)  # tilt 현재 상태 명시 (선택)
                axis = "pan"
                moved = True
            elif key in KEY_RIGHT or key in (ord("d"), ord("D")):
                delta_steps = state.move_pan(+state.angle_step_deg)
                motor.send_pan(delta_steps)
                motor.send_tilt(0)
                axis = "pan"
                moved = True
            elif key in KEY_UP or key in (ord("w"), ord("W")):
                delta_steps = state.move_tilt(+state.angle_step_deg)
                motor.send_tilt(delta_steps)
                axis = "tilt"
                moved = True
            elif key in KEY_DOWN or key in (ord("s"), ord("S")):
                delta_steps = state.move_tilt(-state.angle_step_deg)
                motor.send_tilt(delta_steps)
                axis = "tilt"
                moved = True

            # ── 이동 처리 ──
            if moved:
                print(
                    f"[Move] {axis}  delta={delta_steps:+d} steps  "
                    f"→ pan={state.pan_deg:+.3f} deg  tilt={state.tilt_deg:+.3f} deg"
                )
                flush_camera(cap, args.flush_frames, args.settle_sec)
                continue  # 이동 후 즉시 다음 프레임 읽기

            # ── 기타 키 ──
            if key in (ord("r"), ord("R")):
                state.zero()
                print("[Zero] Current commanded position set to pan=0, tilt=0.")

            elif key in KEY_PLUS:
                state.angle_step_deg = round(state.angle_step_deg + 0.5, 2)
                print(f"[Step] angle_step_deg = {state.angle_step_deg:.2f}")

            elif key in KEY_MINUS:
                state.angle_step_deg = max(0.5, round(state.angle_step_deg - 0.5, 2))
                print(f"[Step] angle_step_deg = {state.angle_step_deg:.2f}")

            elif key == KEY_SPACE:
                # 저장 조건 확인
                if args.require_charuco and corner_count < args.min_corners:
                    print(
                        f"[Skip] Only {corner_count} ChArUco corners detected "
                        f"(need {args.min_corners}). Move board into view."
                    )
                    continue

                # 현재 프레임 저장 (detect 된 원본 frame 사용)
                metadata = save_sample(
                    output_dir,
                    args.prefix,
                    session_id,
                    len(samples),
                    args.ext,
                    frame,
                    args,
                    state,
                    corner_count,
                )
                samples.append(metadata)

                # 인덱스 YAML 업데이트
                save_yaml(
                    index_yaml,
                    {
                        "updated_utc": utc_now_iso(),
                        "output_dir": str(output_dir),
                        "sample_count": len(samples),
                        "samples": samples,
                        "units": "mm",
                    },
                )
                print(
                    f"[Saved #{len(samples):03d}] {metadata['image_file']}  "
                    f"pan={state.pan_deg:+.3f}  tilt={state.tilt_deg:+.3f}  corners={corner_count}"
                )
                sleep_for_ui()

    finally:
        # FPGA 자동 모드 복귀
        if serial_ok:
            motor.send_manual_mode(False)
        cap.release()
        motor.close()
        cv2.destroyAllWindows()

    print(f"\n[Done] Saved {len(samples)} samples in {output_dir}")
    print(f"       Index YAML: {index_yaml}")


if __name__ == "__main__":
    main()
