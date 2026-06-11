# -*- coding: utf-8 -*-
"""
실제 드론 영상에서 신뢰할 수 있는 측정값만 플롯.

저장 데이터 (raw_frames.csv):
  frame_idx, timestamp_s, detected, conf, bbox_w, bbox_h, bbox_area_px, infer_ms

생성 그래프 (4종):
  1) 프레임별 탐지 성공/실패 타임라인
  2) 프레임별 confidence 추이
  3) 프레임별 bbox 면적 추이
  4) 처리 속도 (ms/frame) 분포 히스토그램
  + 5초 구간별 탐지율 bar chart
"""
import argparse
import csv
import time
from pathlib import Path
from collections import defaultdict

import cv2
import numpy as np
import torch  # ORT CUDA DLL 로드용
import onnxruntime as ort
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.font_manager as fm

# Windows 한글 폰트 설정
plt.rcParams["font.family"] = "Malgun Gothic"
plt.rcParams["axes.unicode_minus"] = False

MODEL = r"C:\Users\kimse\capstone\antidrone\models\drone_yolov8x\best.onnx"
SIZE  = 640
CONF  = 0.25
IOU   = 0.70


def letterbox(img, size=SIZE):
    h, w = img.shape[:2]
    scale = min(size / max(w, 1), size / max(h, 1))
    rw, rh = max(round(w * scale), 1), max(round(h * scale), 1)
    resized = cv2.resize(img, (rw, rh), interpolation=cv2.INTER_LINEAR)
    px, py = (size - rw) // 2, (size - rh) // 2
    out = cv2.copyMakeBorder(resized, py, size - rh - py,
                             px, size - rw - px,
                             cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return out, scale, px, py


def preprocess(img):
    lb, scale, px, py = letterbox(img)
    blob = cv2.dnn.blobFromImage(lb, 1.0 / 255.0, (SIZE, SIZE),
                                 swapRB=True, crop=False).astype(np.float32)
    return blob, scale, px, py


def postprocess(out, scale, px, py, orig_w, orig_h):
    o = out[0][0]
    if o.shape[0] <= o.shape[1]:
        o = o.transpose(1, 0)
    boxes, scores = [], []
    for row in o:
        conf = float(row[4:].max())
        if conf < CONF:
            continue
        cx, cy, bw, bh = row[0], row[1], row[2], row[3]
        x1 = (cx - bw / 2 - px) / scale
        y1 = (cy - bh / 2 - py) / scale
        boxes.append([float(x1), float(y1), float(bw / scale), float(bh / scale)])
        scores.append(conf)
    if not boxes:
        return []
    indices = cv2.dnn.NMSBoxes(boxes, scores, CONF, IOU)
    result = []
    for i in (indices.flatten() if len(indices) else []):
        x, y, w, h = boxes[i]
        result.append((scores[i], w, h))
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--model", default=MODEL, help="best.onnx 경로")
    ap.add_argument("--warmup", type=int, default=10)
    ap.add_argument("--provider", default="CUDA", choices=["CUDA", "CPU"])
    args = ap.parse_args()

    video_path = Path(args.video)
    out_dir = video_path.parent / f"benchmark_plot_{video_path.stem}"
    out_dir.mkdir(exist_ok=True)

    # ORT 세션
    providers = (["CUDAExecutionProvider", "CPUExecutionProvider"]
                 if args.provider == "CUDA" else ["CPUExecutionProvider"])
    sess = ort.InferenceSession(args.model, providers=providers)
    in_name = sess.get_inputs()[0].name
    print(f"[ORT] {sess.get_providers()}")

    cap = cv2.VideoCapture(str(video_path))
    vid_fps = cap.get(cv2.CAP_PROP_FPS)
    total_f = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    orig_w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_h  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[영상] {video_path.name}  {orig_w}x{orig_h}  {vid_fps:.1f}fps  {total_f}프레임")

    # warmup
    ret, frame = cap.read()
    blob, *_ = preprocess(frame)
    for _ in range(args.warmup):
        sess.run(None, {in_name: blob})
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    # 측정
    rows = []
    print("측정 중...")
    for frame_idx in range(total_f):
        ret, frame = cap.read()
        if not ret:
            break
        t0 = time.perf_counter()
        blob, scale, px, py = preprocess(frame)
        out = sess.run(None, {in_name: blob})
        dets = postprocess(out, scale, px, py, orig_w, orig_h)
        infer_ms = (time.perf_counter() - t0) * 1000

        ts = frame_idx / vid_fps
        if dets:
            # 가장 높은 conf bbox 선택
            best = max(dets, key=lambda x: x[0])
            conf, bw, bh = best
            area = bw * bh
            rows.append((frame_idx, ts, 1, conf, bw, bh, area, infer_ms))
        else:
            rows.append((frame_idx, ts, 0, 0.0, 0.0, 0.0, 0.0, infer_ms))

        if frame_idx % 100 == 0:
            print(f"  {frame_idx}/{total_f} 프레임...")

    cap.release()
    print(f"측정 완료 ({len(rows)}프레임)\n")

    # CSV 저장
    csv_path = out_dir / "raw_frames.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame_idx","timestamp_s","detected","conf",
                    "bbox_w","bbox_h","bbox_area_px","infer_ms"])
        w.writerows(rows)
    print(f"[저장] {csv_path}")

    # numpy 변환
    arr        = np.array(rows)
    frames     = arr[:, 0].astype(int)
    timestamps = arr[:, 1]
    detected   = arr[:, 2].astype(int)
    confs      = arr[:, 3]
    areas      = arr[:, 6]
    infer_ms   = arr[:, 7]

    det_mask   = detected == 1
    det_ts     = timestamps[det_mask]
    det_confs  = confs[det_mask]
    det_areas  = areas[det_mask]

    # ── 플롯 ─────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle(f"드론 탐지 벤치마크 — {video_path.name}\n"
                 f"({orig_w}x{orig_h}, {vid_fps:.0f}fps, {len(rows)}프레임 | "
                 f"ONNXRuntime CUDA, RTX 3050)",
                 fontsize=12, y=0.98)

    # 1) 탐지 타임라인
    ax = axes[0, 0]
    ax.scatter(timestamps[det_mask],  np.ones(det_mask.sum()),
               c="green", s=4, label="탐지 성공", alpha=0.7)
    ax.scatter(timestamps[~det_mask], np.zeros((~det_mask).sum()),
               c="red", s=4, label="미탐지", alpha=0.3)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["미탐지", "탐지 성공"])
    ax.set_xlabel("시간 (초)")
    ax.set_title(f"프레임별 탐지 성공/실패  (탐지율 {det_mask.sum()/len(rows)*100:.1f}%)")
    ax.legend(loc="upper right", markerscale=3)
    ax.set_xlim(0, timestamps[-1])
    ax.grid(axis="x", alpha=0.3)

    # 2) Confidence 추이
    ax = axes[0, 1]
    ax.scatter(det_ts, det_confs, s=4, c="steelblue", alpha=0.6)
    ax.axhline(det_confs.mean(), color="red", linestyle="--", linewidth=1,
               label=f"평균 {det_confs.mean():.3f}")
    ax.set_xlabel("시간 (초)")
    ax.set_ylabel("Confidence")
    ax.set_title("탐지 프레임 Confidence 추이")
    ax.set_ylim(0, 1.05)
    ax.set_xlim(0, timestamps[-1])
    ax.legend()
    ax.grid(alpha=0.3)

    # 3) Confidence 히스토그램
    ax = axes[1, 0]
    bins = np.arange(0.25, 1.05, 0.05)
    ax.hist(det_confs, bins=bins, color="steelblue", edgecolor="white", alpha=0.85)
    ax.axvline(det_confs.mean(), color="red", linestyle="--",
               label=f"평균 {det_confs.mean():.3f}")
    ax.set_xlabel("Confidence")
    ax.set_ylabel("프레임 수")
    ax.set_title("Confidence 분포")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    # 4) bbox 면적 추이
    ax = axes[1, 1]
    ax.scatter(det_ts, det_areas, s=4, c="darkorange", alpha=0.6)
    ax.axhline(det_areas.mean(), color="red", linestyle="--", linewidth=1,
               label=f"평균 {det_areas.mean():.0f} px²")
    ax.set_xlabel("시간 (초)")
    ax.set_ylabel("면적 (px²)")
    ax.set_title("탐지 bbox 면적 추이  (드론 거리 변화 반영)")
    ax.set_xlim(0, timestamps[-1])
    ax.legend()
    ax.grid(alpha=0.3)

    # 5) 5초 구간 탐지율
    ax = axes[2, 0]
    seg_stats = defaultdict(lambda: {"total": 0, "det": 0})
    for ts, d in zip(timestamps, detected):
        seg = int(ts // 5) * 5
        seg_stats[seg]["total"] += 1
        seg_stats[seg]["det"] += int(d)
    segs = sorted(seg_stats)
    rates = [seg_stats[s]["det"] / seg_stats[s]["total"] * 100 for s in segs]
    labels = [f"{s}~{s+5}s" for s in segs]
    bars = ax.bar(range(len(segs)), rates,
                  color=["green" if r >= 70 else "orange" if r >= 40 else "red"
                         for r in rates], edgecolor="white", alpha=0.85)
    ax.set_xticks(range(len(segs)))
    ax.set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("탐지율 (%)")
    ax.set_ylim(0, 110)
    ax.set_title("5초 구간별 탐지율")
    for bar, rate in zip(bars, rates):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1,
                f"{rate:.0f}%", ha="center", va="bottom", fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    patches = [mpatches.Patch(color="green", label=">=70%"),
               mpatches.Patch(color="orange", label="40~70%"),
               mpatches.Patch(color="red", label="<40%")]
    ax.legend(handles=patches, fontsize=8)

    # 6) 처리 속도 히스토그램
    ax = axes[2, 1]
    ax.hist(infer_ms, bins=40, color="slateblue", edgecolor="white", alpha=0.85)
    ax.axvline(np.mean(infer_ms), color="red", linestyle="--",
               label=f"평균 {np.mean(infer_ms):.1f} ms ({1000/np.mean(infer_ms):.1f} FPS)")
    ax.axvline(np.percentile(infer_ms, 95), color="orange", linestyle=":",
               label=f"P95 {np.percentile(infer_ms, 95):.1f} ms")
    ax.set_xlabel("처리 시간 (ms/frame)")
    ax.set_ylabel("프레임 수")
    ax.set_title("프레임 처리 속도 분포")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plot_path = out_dir / "benchmark_plots.png"
    plt.savefig(str(plot_path), dpi=150, bbox_inches="tight")
    plt.close()
    print(f"[저장] {plot_path}")
    print("완료!")


if __name__ == "__main__":
    main()
