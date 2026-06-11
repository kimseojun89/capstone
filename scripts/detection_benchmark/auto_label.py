# -*- coding: utf-8 -*-
"""
영상 → 프레임 추출 → best.onnx 자동 탐지 → YOLO 라벨 생성

출력 구조:
  output_dir/
    frames/   *.jpg  (샘플링된 프레임 이미지)
    labels/   *.txt  (YOLO 형식 라벨: class cx cy w h, 정규화)
    preview/  *.jpg  (bbox 시각화 이미지, 검토용)
    auto_label_report.txt  (탐지 요약)
"""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import torch  # ORT CUDA DLL 로드용
import onnxruntime as ort

MODEL   = r"C:\Users\kimse\capstone\antidrone\models\drone_yolov8x\best.onnx"
SIZE    = 640
CONF    = 0.25
IOU     = 0.70
CLASS_ID = 0  # drone


# ── 전처리 ──────────────────────────────────────────────────────────────────
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


# ── 후처리 ──────────────────────────────────────────────────────────────────
def postprocess(out, scale, px, py, orig_w, orig_h):
    o = out[0][0]
    if o.shape[0] <= o.shape[1]:
        o = o.transpose(1, 0)  # [N, C]

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
    results = []
    for i in (indices.flatten() if len(indices) else []):
        x1, y1, w, h = boxes[i]
        x2, y2 = x1 + w, y1 + h
        # 이미지 경계 클램프
        x1, y1 = max(0.0, x1), max(0.0, y1)
        x2, y2 = min(float(orig_w), x2), min(float(orig_h), y2)
        # YOLO 정규화
        cx_n = (x1 + x2) / 2 / orig_w
        cy_n = (y1 + y2) / 2 / orig_h
        w_n  = (x2 - x1) / orig_w
        h_n  = (y2 - y1) / orig_h
        results.append((scores[i], cx_n, cy_n, w_n, h_n, x1, y1, x2, y2))
    return results


# ── 메인 ────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--model", default=MODEL, help="best.onnx 경로")
    ap.add_argument("--sample_fps", type=float, default=10.0,
                    help="샘플링 FPS (기본 10)")
    ap.add_argument("--out", default=None,
                    help="출력 폴더 (기본: 영상 옆 auto_label_<이름>)")
    ap.add_argument("--provider", default="CUDA", choices=["CUDA", "CPU"])
    args = ap.parse_args()

    video_path = Path(args.video)
    out_dir    = Path(args.out) if args.out else \
                 video_path.parent / f"auto_label_{video_path.stem}"

    frames_dir  = out_dir / "frames"
    labels_dir  = out_dir / "labels"
    preview_dir = out_dir / "preview"
    for d in (frames_dir, labels_dir, preview_dir):
        d.mkdir(parents=True, exist_ok=True)

    # ORT 세션
    providers = (["CUDAExecutionProvider", "CPUExecutionProvider"]
                 if args.provider == "CUDA" else ["CPUExecutionProvider"])
    sess = ort.InferenceSession(args.model, providers=providers)
    in_name = sess.get_inputs()[0].name
    print(f"[ORT] {sess.get_providers()}")

    # 영상 열기
    cap = cv2.VideoCapture(str(video_path))
    vid_fps   = cap.get(cv2.CAP_PROP_FPS)
    total_f   = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    orig_w    = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_h    = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    duration  = total_f / vid_fps
    step      = max(1, round(vid_fps / args.sample_fps))  # 프레임 간격

    print(f"[영상] {video_path.name}  {orig_w}x{orig_h}  {vid_fps:.1f}fps  "
          f"{duration:.1f}s  총 {total_f}프레임")
    print(f"[샘플링] {args.sample_fps}fps → step={step}, "
          f"예상 {total_f // step}장 추출")
    print(f"[출력] {out_dir}\n")

    saved = 0
    detected = 0
    t_start = time.perf_counter()

    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx % step == 0:
            blob, scale, px, py = preprocess(frame)
            out = sess.run(None, {in_name: blob})
            dets = postprocess(out, scale, px, py, orig_w, orig_h)

            stem = f"frame_{frame_idx:06d}"

            # 이미지 저장
            cv2.imwrite(str(frames_dir / f"{stem}.jpg"), frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 95])

            # 라벨 저장 (YOLO 형식)
            label_path = labels_dir / f"{stem}.txt"
            with open(label_path, "w") as f:
                for det in dets:
                    _, cx_n, cy_n, w_n, h_n, *_ = det
                    f.write(f"{CLASS_ID} {cx_n:.6f} {cy_n:.6f} "
                            f"{w_n:.6f} {h_n:.6f}\n")

            # 프리뷰 저장
            preview = frame.copy()
            for det in dets:
                conf, _, _, _, _, x1, y1, x2, y2 = det
                cv2.rectangle(preview, (int(x1), int(y1)),
                              (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(preview, f"drone {conf:.2f}",
                            (int(x1), int(y1) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imwrite(str(preview_dir / f"{stem}.jpg"), preview,
                        [cv2.IMWRITE_JPEG_QUALITY, 85])

            if dets:
                detected += 1
            saved += 1

            elapsed = time.perf_counter() - t_start
            print(f"\r  {saved}장 처리 | 탐지 {detected}장 | "
                  f"{elapsed:.1f}s", end="", flush=True)

        frame_idx += 1

    cap.release()
    elapsed = time.perf_counter() - t_start

    # 리포트
    report = (
        f"=== 자동 라벨링 리포트 ===\n"
        f"영상:       {video_path.name}\n"
        f"해상도:     {orig_w}x{orig_h}, {vid_fps:.1f}fps, {duration:.1f}s\n"
        f"샘플링:     {args.sample_fps}fps (step={step})\n"
        f"추출 프레임: {saved}장\n"
        f"탐지 성공:  {detected}장 ({detected/saved*100:.1f}%)\n"
        f"미탐지:     {saved - detected}장 ({(saved-detected)/saved*100:.1f}%)\n"
        f"처리 시간:  {elapsed:.1f}s\n"
        f"\n출력 폴더:\n"
        f"  frames/   → CVAT 업로드용 이미지\n"
        f"  labels/   → YOLO 형식 자동 라벨\n"
        f"  preview/  → bbox 시각화 (검토용)\n"
        f"\n[다음 단계]\n"
        f"1. preview/ 이미지 확인 → 오탐/미탐 파악\n"
        f"2. CVAT(https://cvat.ai)에 frames/ + labels/ 업로드\n"
        f"3. 틀린 라벨만 수정 후 export → mAP 측정\n"
    )
    (out_dir / "auto_label_report.txt").write_text(report, encoding="utf-8")
    print(f"\n\n{report}")


if __name__ == "__main__":
    main()
