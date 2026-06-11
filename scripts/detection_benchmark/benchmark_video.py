# -*- coding: utf-8 -*-
"""
실제 드론 영상 기반 벤치마크.

측정 항목:
  1) 실제 영상 FPS  : 카메라 해상도 그대로 letterbox→추론 (실전 처리량)
  2) conf 분포      : 탐지된 bbox의 신뢰도 구간별 분포
  3) 구간별 탐지율  : 5초 단위로 탐지 성공/실패 비율

하드웨어 불필요 — 영상 파일만 있으면 됨.
"""
import argparse
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch  # ORT CUDA DLL 로드용
import onnxruntime as ort

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


def postprocess(out, scale, px, py):
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
    return [scores[i] for i in (indices.flatten() if len(indices) else [])]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--model", default=MODEL, help="best.onnx 경로")
    ap.add_argument("--warmup", type=int, default=10)
    ap.add_argument("--provider", default="CUDA", choices=["CUDA", "CPU"])
    args = ap.parse_args()

    providers = (["CUDAExecutionProvider", "CPUExecutionProvider"]
                 if args.provider == "CUDA" else ["CPUExecutionProvider"])
    sess = ort.InferenceSession(args.model, providers=providers)
    in_name = sess.get_inputs()[0].name
    print(f"[ORT] {sess.get_providers()}")

    cap = cv2.VideoCapture(args.video)
    vid_fps  = cap.get(cv2.CAP_PROP_FPS)
    total_f  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    orig_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    duration = total_f / vid_fps
    print(f"[영상] {Path(args.video).name}  {orig_w}x{orig_h}  "
          f"{vid_fps:.1f}fps  {duration:.1f}s  {total_f}프레임\n")

    # warmup
    ret, frame = cap.read()
    if not ret:
        raise SystemExit("영상 읽기 실패")
    blob, *_ = preprocess(frame)
    for _ in range(args.warmup):
        sess.run(None, {in_name: blob})
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    # 측정
    frame_times = []       # 프레임별 처리 시간
    conf_scores = []       # 탐지된 conf 전체
    segment_stats = defaultdict(lambda: {"total": 0, "detected": 0})  # 5초 구간
    SEGMENT = 5.0

    frame_idx = 0
    print("측정 중...", end="", flush=True)
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t0 = time.perf_counter()
        blob, scale, px, py = preprocess(frame)
        out = sess.run(None, {in_name: blob})
        dets = postprocess(out, scale, px, py)
        t1 = time.perf_counter()

        elapsed_ms = (t1 - t0) * 1000
        frame_times.append(elapsed_ms)

        timestamp = frame_idx / vid_fps
        seg_key = int(timestamp // SEGMENT) * int(SEGMENT)
        segment_stats[seg_key]["total"] += 1
        if dets:
            segment_stats[seg_key]["detected"] += 1
            conf_scores.extend(dets)

        frame_idx += 1
        if frame_idx % 50 == 0:
            print(f"\r측정 중... {frame_idx}/{total_f}프레임", end="", flush=True)

    cap.release()
    print(f"\r측정 완료! {frame_idx}프레임 처리\n")

    # ── 결과 출력 ────────────────────────────────────────────────
    avg_ms   = np.mean(frame_times)
    p50_ms   = np.percentile(frame_times, 50)
    p95_ms   = np.percentile(frame_times, 95)
    avg_fps  = 1000 / avg_ms
    det_frames = sum(1 for s in segment_stats.values() for _ in range(s["detected"]))
    total_det_frames = sum(1 for c in [True] * len([f for f in frame_times]))

    print("=" * 60)
    print(f"  실제 영상 FPS 벤치마크  ({args.provider}, RTX 3050)")
    print(f"  영상: {Path(args.video).name}  ({orig_w}x{orig_h})")
    print("=" * 60)

    print("\n[1] 처리 속도")
    print(f"  평균 ms/frame : {avg_ms:.2f} ms  →  {avg_fps:.2f} FPS")
    print(f"  중간값 (P50)  : {p50_ms:.2f} ms")
    print(f"  최악 (P95)    : {p95_ms:.2f} ms")
    realtime = "가능" if avg_fps >= vid_fps else f"불가 (영상 {vid_fps:.0f}fps 기준)"
    print(f"  실시간 가능?  : {realtime}")

    print("\n[2] 탐지 통계")
    total_frames = len(frame_times)
    detected_frames = sum(1 for s in conf_scores) > 0 and len([1 for s in segment_stats.values() if s["detected"] > 0])
    n_detected = sum(s["detected"] for s in segment_stats.values())
    print(f"  전체 프레임   : {total_frames}장")
    print(f"  탐지 성공     : {n_detected}장 ({n_detected/total_frames*100:.1f}%)")
    print(f"  미탐지        : {total_frames - n_detected}장 ({(total_frames-n_detected)/total_frames*100:.1f}%)")

    if conf_scores:
        print(f"\n  conf 분포 (탐지된 {len(conf_scores)}개 bbox)")
        bins = [(0.25, 0.40), (0.40, 0.60), (0.60, 0.80), (0.80, 1.01)]
        for lo, hi in bins:
            cnt = sum(1 for c in conf_scores if lo <= c < hi)
            bar = "#" * int(cnt / len(conf_scores) * 30)
            print(f"  {lo:.2f}~{hi:.2f} : {bar:<30} {cnt:4d} ({cnt/len(conf_scores)*100:.1f}%)")
        print(f"  평균 conf     : {np.mean(conf_scores):.3f}")
        print(f"  최소 conf     : {np.min(conf_scores):.3f}")

    print("\n[3] 구간별 탐지율 (5초 단위)")
    for seg in sorted(segment_stats):
        s = segment_stats[seg]
        rate = s["detected"] / s["total"] * 100 if s["total"] else 0
        bar = "#" * int(rate / 100 * 20)
        print(f"  {seg:3d}~{seg+5:3d}s : {bar:<20} {rate:5.1f}%  ({s['detected']}/{s['total']})")

    print("=" * 60)


if __name__ == "__main__":
    main()
