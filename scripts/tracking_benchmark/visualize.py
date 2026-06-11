# -*- coding: utf-8 -*-
"""
추적 파이프라인 벤치마크 CSV 시각화.

benchmark.exe --csv 로 생성한 프레임별 로그를 읽어 다음을 그린다:
  1) dashboard.png  - 모드 타임라인 + 점유율 막대 + trackId + conf + bbox 면적 + 요약
  2) trajectory.png - 타깃 센터 궤적(화면 좌표, 상태별 색)

pandas 불필요. 표준 csv + numpy + matplotlib만 사용.

사용:
  python visualize.py [csv_path]
  (기본: 같은 폴더의 bench_result.csv)
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")  # 헤드리스 저장 전용
import matplotlib.pyplot as plt
import numpy as np

# 한글 폰트(Windows 맑은 고딕) — 없으면 기본 폰트로 폴백
try:
    matplotlib.rcParams["font.family"] = "Malgun Gothic"
except Exception:
    pass
matplotlib.rcParams["axes.unicode_minus"] = False

# 상태별 색상/순서
STATE_ORDER = ["bytetrack", "csrt", "kalman_hold", "none"]
STATE_LABEL = {
    "bytetrack": "ByteTrack",
    "csrt": "CSRT",
    "kalman_hold": "Kalman 홀드",
    "none": "손실",
}
STATE_COLOR = {
    "bytetrack": "#2ca02c",
    "csrt": "#ff7f0e",
    "kalman_hold": "#d62728",
    "none": "#cccccc",
}
STATE_Y = {"bytetrack": 3, "csrt": 2, "kalman_hold": 1, "none": 0}


def load(csv_path):
    rows = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    if not rows:
        raise SystemExit("CSV가 비어 있습니다: " + csv_path)

    def fnum(r, key):
        v = r.get(key, "")
        return float(v) if v not in ("", None) else np.nan

    n = len(rows)
    data = {
        "frame": np.array([int(r["frame"]) for r in rows]),
        "t": np.array([float(r["t"]) for r in rows]),
        "state": [r["state"] for r in rows],
        "track_id": np.array([fnum(r, "track_id") for r in rows]),
        "cx": np.array([fnum(r, "cx") for r in rows]),
        "cy": np.array([fnum(r, "cy") for r in rows]),
        "bw": np.array([fnum(r, "box_w") for r in rows]),
        "bh": np.array([fnum(r, "box_h") for r in rows]),
        "conf": np.array([fnum(r, "conf") for r in rows]),
        "num_det": np.array([fnum(r, "num_det") for r in rows]),
    }
    data["n"] = n
    return data


def compute_metrics(d):
    n = d["n"]
    states = d["state"]
    counts = {s: states.count(s) for s in STATE_ORDER}
    target_frames = n - counts["none"]

    id_switches = 0
    reacq = 0
    hold_entries = 0
    frag = 0
    prev_state = None
    prev_has = False
    prev_id = None
    for i in range(n):
        s = states[i]
        has = s != "none"
        tid = d["track_id"][i]
        if has and prev_has and prev_id is not None and not np.isnan(tid) \
                and not np.isnan(prev_id) and tid != prev_id:
            id_switches += 1
        if has and not prev_has and i > 0:
            reacq += 1
        if s == "kalman_hold" and prev_state != "kalman_hold":
            hold_entries += 1
        if has and not prev_has:
            frag += 1
        prev_state, prev_has, prev_id = s, has, (tid if has else None)

    return {
        "n": n,
        "counts": counts,
        "target_frames": target_frames,
        "coverage": 100.0 * target_frames / n if n else 0.0,
        "id_switches": id_switches,
        "reacq": reacq,
        "hold_entries": hold_entries,
        "fragments": frag,
        "avg_conf": float(np.nanmean(d["conf"])) if np.any(~np.isnan(d["conf"])) else 0.0,
    }


def make_dashboard(d, m, out_path):
    fig = plt.figure(figsize=(15, 11))
    gs = fig.add_gridspec(4, 2, height_ratios=[1.1, 1, 1, 1.0],
                          hspace=0.45, wspace=0.22)
    fr = d["frame"]

    # --- (1) 모드 타임라인 ---
    ax0 = fig.add_subplot(gs[0, :])
    ys = np.array([STATE_Y[s] for s in d["state"]])
    for s in STATE_ORDER:
        mask = ys == STATE_Y[s]
        ax0.scatter(fr[mask], ys[mask], c=STATE_COLOR[s], s=14,
                    label=STATE_LABEL[s], marker="s")
    ax0.set_yticks([0, 1, 2, 3])
    ax0.set_yticklabels([STATE_LABEL[s] for s in ["none", "kalman_hold", "csrt", "bytetrack"]])
    ax0.set_xlabel("프레임")
    ax0.set_title("프레임별 타깃 출처(모드) 타임라인", fontweight="bold")
    ax0.grid(True, axis="x", alpha=0.3)
    ax0.set_ylim(-0.5, 3.5)

    # --- (2) 점유율 막대 ---
    ax1 = fig.add_subplot(gs[1, 0])
    labels = [STATE_LABEL[s] for s in STATE_ORDER]
    vals = [m["counts"][s] for s in STATE_ORDER]
    cols = [STATE_COLOR[s] for s in STATE_ORDER]
    bars = ax1.bar(labels, vals, color=cols)
    for b, v in zip(bars, vals):
        pctv = 100.0 * v / m["n"] if m["n"] else 0
        ax1.text(b.get_x() + b.get_width() / 2, v, f"{v}\n({pctv:.1f}%)",
                 ha="center", va="bottom", fontsize=9)
    ax1.set_ylabel("프레임 수")
    ax1.set_title("모드 점유율", fontweight="bold")
    ax1.set_ylim(0, max(vals) * 1.25 if max(vals) > 0 else 1)

    # --- (3) trackId 추이 ---
    ax2 = fig.add_subplot(gs[1, 1])
    tid = d["track_id"].copy()
    ax2.step(fr, tid, where="post", color="#1f77b4", lw=1.2)
    sw_idx = []
    for i in range(1, d["n"]):
        a, b = tid[i - 1], tid[i]
        if not np.isnan(a) and not np.isnan(b) and a != b:
            sw_idx.append(i)
    if sw_idx:
        ax2.scatter(fr[sw_idx], tid[sw_idx], color="red", zorder=5, s=40,
                    label=f"ID 전환 {len(sw_idx)}회")
        ax2.legend(fontsize=9)
    ax2.set_xlabel("프레임")
    ax2.set_ylabel("track ID")
    ax2.set_title("선택 타깃 track ID 추이", fontweight="bold")
    ax2.grid(True, alpha=0.3)

    # --- (4) 신뢰도 ---
    ax3 = fig.add_subplot(gs[2, 0])
    ax3.plot(fr, d["conf"], color="#2ca02c", lw=1.0)
    ax3.axhline(np.nanmean(d["conf"]), color="gray", ls="--", lw=1,
                label=f"평균 {m['avg_conf']:.3f}")
    ax3.set_xlabel("프레임")
    ax3.set_ylabel("confidence")
    ax3.set_title("탐지 신뢰도(ByteTrack 프레임)", fontweight="bold")
    ax3.set_ylim(0, 1.05)
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)

    # --- (5) bbox 면적 ---
    ax4 = fig.add_subplot(gs[2, 1])
    area = d["bw"] * d["bh"]
    ax4.plot(fr, area, color="#9467bd", lw=1.0)
    ax4.set_xlabel("프레임")
    ax4.set_ylabel("bbox 면적 (px²)")
    ax4.set_title("바운딩박스 크기", fontweight="bold")
    ax4.grid(True, alpha=0.3)

    # --- (6) 요약 텍스트 ---
    ax5 = fig.add_subplot(gs[3, :])
    ax5.axis("off")
    txt = (
        f"전체 프레임: {m['n']}      "
        f"커버리지(타깃 존재): {m['target_frames']} ({m['coverage']:.1f}%)\n\n"
        f"모드 점유율 →  "
        f"ByteTrack {m['counts']['bytetrack']} ({100*m['counts']['bytetrack']/m['n']:.1f}%)   |   "
        f"CSRT {m['counts']['csrt']} ({100*m['counts']['csrt']/m['n']:.1f}%)   |   "
        f"Kalman홀드 {m['counts']['kalman_hold']} ({100*m['counts']['kalman_hold']/m['n']:.1f}%)   |   "
        f"손실 {m['counts']['none']} ({100*m['counts']['none']/m['n']:.1f}%)\n\n"
        f"ID 전환: {m['id_switches']}회      "
        f"재확보: {m['reacq']}회      "
        f"Kalman 홀드 진입: {m['hold_entries']}회      "
        f"추적 구간 수: {m['fragments']}      "
        f"ByteTrack 평균 conf: {m['avg_conf']:.3f}"
    )
    ax5.text(0.01, 0.7, txt, fontsize=12, va="top",
             bbox=dict(boxstyle="round", fc="#f5f5f5", ec="#bbbbbb"))

    handles = [plt.Line2D([0], [0], marker="s", ls="", color=STATE_COLOR[s],
                          label=STATE_LABEL[s]) for s in STATE_ORDER]
    ax0.legend(handles=handles, loc="upper right", ncol=4, fontsize=9)

    fig.suptitle("드론 추적 파이프라인 정량화 — 테스트 영상", fontsize=15, fontweight="bold")
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    print("저장:", out_path)


def make_trajectory(d, out_path):
    fig, ax = plt.subplots(figsize=(10, 6.5))
    has = np.array([s != "none" for s in d["state"]])
    for s in ["bytetrack", "csrt", "kalman_hold"]:
        mask = np.array([st == s for st in d["state"]])
        if mask.any():
            ax.scatter(d["cx"][mask], d["cy"][mask], c=STATE_COLOR[s],
                       s=18, label=STATE_LABEL[s], alpha=0.8)
    # 궤적 선
    cx, cy = d["cx"][has], d["cy"][has]
    ax.plot(cx, cy, color="#888888", lw=0.6, alpha=0.5, zorder=0)
    if len(cx):
        ax.scatter(cx[0], cy[0], marker="*", s=220, c="black", zorder=6, label="시작")
    ax.set_xlabel("화면 X (px)")
    ax.set_ylabel("화면 Y (px)")
    ax.set_title("타깃 센터 궤적 (상태별 색)", fontweight="bold")
    ax.invert_yaxis()  # 영상 좌표계
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    print("저장:", out_path)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    csv_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "bench_result.csv")
    if not os.path.exists(csv_path):
        raise SystemExit("CSV 없음: " + csv_path + "\n먼저 benchmark.exe --csv 로 생성하세요.")

    d = load(csv_path)
    m = compute_metrics(d)
    out_dir = os.path.dirname(csv_path)
    make_dashboard(d, m, os.path.join(out_dir, "dashboard.png"))
    make_trajectory(d, os.path.join(out_dir, "trajectory.png"))
    print("\n요약:", m)


if __name__ == "__main__":
    main()
