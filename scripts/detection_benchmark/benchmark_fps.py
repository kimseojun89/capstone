# -*- coding: utf-8 -*-
"""
best.onnx 의 실제 배포 추론 FPS 측정 (하드웨어 불필요).

C++ 트래커(detector.cpp)의 추론 경로를 그대로 복제한다:
  letterbox(640, pad=114) -> blobFromImage(BGR->RGB, /255, NCHW)
    -> ONNXRuntime CUDAExecutionProvider session.run -> (NMS)

ultralytics 래퍼 오버헤드를 제외하고 onnxruntime 를 직접 호출하므로
C++ ONNXRuntime-GPU 배포 속도에 가장 근접한 수치를 준다.

측정:
  1) pure inference  : session.run 만 (GPU 엔진 순수 처리량)
  2) full pipeline   : 전처리 + 추론 + NMS 후처리 (실제 프레임당 비용)
"""
import time
import glob
import argparse
from pathlib import Path

import cv2
import numpy as np
import torch  # noqa: F401  torch 번들 CUDA12/cuDNN9 DLL을 프로세스에 로드(ORT GPU용)
import onnxruntime as ort

MODEL = r"C:\Users\kimse\capstone\antidrone\models\drone_yolov8x\best.onnx"
VALID = r"C:\Users\kimse\capstone\scripts\Drone-Detection-YOLOv8x-main\drone_dataset\valid\images"  # 데이터셋은 Drone-Detection-YOLOv8x-main 안에 위치
SIZE = 640
CONF = 0.25
IOU = 0.70


def letterbox(img, size=SIZE):
    h, w = img.shape[:2]
    scale = min(size / max(w, 1), size / max(h, 1))
    rw, rh = max(round(w * scale), 1), max(round(h * scale), 1)
    resized = cv2.resize(img, (rw, rh), interpolation=cv2.INTER_LINEAR)
    px, py = (size - rw) // 2, (size - rh) // 2
    out = cv2.copyMakeBorder(resized, py, size - rh - py, px, size - rw - px,
                             cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return out, scale, px, py


def preprocess(img):
    lb, scale, px, py = letterbox(img)
    blob = cv2.dnn.blobFromImage(lb, 1.0 / 255.0, (SIZE, SIZE),
                                 swapRB=True, crop=False)
    return blob.astype(np.float32), scale, px, py


def postprocess(out, scale, px, py):
    # out[0]: [1, C, N] or [1, N, C]; C=5 (cx,cy,w,h,score) for single class
    o = out[0][0]  # 배치 차원 제거 -> [C, N] 또는 [N, C]
    if o.shape[0] <= o.shape[1]:
        o = o.transpose(1, 0)  # -> [N, C]
    boxes, scores = [], []
    for row in o:
        score = float(row[4:].max())
        if score < CONF:
            continue
        cx, cy, w, h = row[0], row[1], row[2], row[3]
        x1 = (cx - w / 2 - px) / scale
        y1 = (cy - h / 2 - py) / scale
        boxes.append([x1, y1, w / scale, h / scale])
        scores.append(score)
    if boxes:
        cv2.dnn.NMSBoxes(boxes, scores, CONF, IOU)
    return len(boxes)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=MODEL, help="best.onnx 경로")
    ap.add_argument("--valid", default=VALID, help="검증 이미지 폴더 경로")
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--provider", default="CUDA", choices=["CUDA", "CPU"])
    args = ap.parse_args()

    providers = (["CUDAExecutionProvider", "CPUExecutionProvider"]
                 if args.provider == "CUDA" else ["CPUExecutionProvider"])
    so = ort.SessionOptions()
    sess = ort.InferenceSession(args.model, sess_options=so, providers=providers)
    in_name = sess.get_inputs()[0].name
    print(f"[FPS] providers in use: {sess.get_providers()}")
    print(f"[FPS] input: {sess.get_inputs()[0].shape}  warmup={args.warmup} "
          f"iters={args.iters}")

    # 실제 검증 이미지를 순환 입력 (캐시/메모리 패턴 현실화)
    files = sorted(glob.glob(str(Path(args.valid) / "*.jpg")))[:args.iters]
    if not files:
        raise SystemExit(f"이미지 없음: {args.valid}")
    imgs = [cv2.imread(f) for f in files]
    blobs = [preprocess(im) for im in imgs]  # 미리 전처리(pure inference용)

    # ---- warmup ----
    for i in range(args.warmup):
        b = blobs[i % len(blobs)][0]
        sess.run(None, {in_name: b})

    # ---- 1) pure inference (session.run only) ----
    n = len(blobs)
    t0 = time.perf_counter()
    for i in range(args.iters):
        b = blobs[i % n][0]
        sess.run(None, {in_name: b})
    t1 = time.perf_counter()
    pure_ms = (t1 - t0) / args.iters * 1000.0

    # ---- 2) full pipeline (preprocess + infer + postprocess) ----
    t0 = time.perf_counter()
    for i in range(args.iters):
        im = imgs[i % n]
        blob, scale, px, py = preprocess(im)
        out = sess.run(None, {in_name: blob})
        postprocess(out, scale, px, py)
    t1 = time.perf_counter()
    full_ms = (t1 - t0) / args.iters * 1000.0

    print("\n" + "=" * 60)
    print(f"  best.onnx 실측 FPS  ({args.provider}, RTX 3050)")
    print("=" * 60)
    print(f"  pure inference : {pure_ms:7.2f} ms/frame  ->  {1000/pure_ms:6.2f} FPS")
    print(f"  full pipeline  : {full_ms:7.2f} ms/frame  ->  {1000/full_ms:6.2f} FPS")
    print("=" * 60)
    print("  pure = GPU 추론 엔진 순수 처리량 (C++ session.Run 대응)")
    print("  full = 전처리+추론+NMS, 카메라 캡처/표시/시리얼 제외")


if __name__ == "__main__":
    main()
