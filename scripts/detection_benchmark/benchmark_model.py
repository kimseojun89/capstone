# -*- coding: utf-8 -*-
"""
antidrone/cpp 가 사용하는 드론 탐지 모델(best.pt / best.onnx)을
검증 데이터셋으로 벤치마킹한다.

측정 항목: precision, recall, mAP50, mAP50-95, inference(ms/img)
- best.pt   : 원본 PyTorch 가중치 (이론상 최대 정확도)
- best.onnx : C++ 트래커가 실제로 배포에 쓰는 ONNX 산출물

C++ 추론 설정과 동일하게 맞춤 (settings.hpp):
  imgsz=640, conf=0.25, iou(NMS)=0.70
"""
import argparse
from pathlib import Path
from ultralytics import YOLO

MODELS_DIR = Path(r"C:\Users\kimse\capstone\antidrone\models\drone_yolov8x")  # --model-dir 로 덮어쓸 수 있음
DATA_YAML = Path(__file__).with_name("data_local.yaml")  # --data 로 덮어쓸 수 있음


def run(model_path: Path, data: Path, imgsz: int, conf: float, iou: float,
        device: str):
    print(f"\n{'='*70}\n[VAL] {model_path.name}\n{'='*70}")
    model = YOLO(str(model_path))
    m = model.val(
        data=str(data),
        imgsz=imgsz,
        conf=conf,
        iou=iou,
        device=device,
        split="val",
        plots=False,
        verbose=True,
    )
    speed = m.speed  # ms: preprocess / inference / postprocess
    return {
        "model": model_path.name,
        "precision": m.box.mp,
        "recall": m.box.mr,
        "mAP50": m.box.map50,
        "mAP50-95": m.box.map,
        "inference_ms": speed.get("inference"),
        "preprocess_ms": speed.get("preprocess"),
        "postprocess_ms": speed.get("postprocess"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-dir", default=str(MODELS_DIR),
                    help="best.pt / best.onnx 가 있는 폴더 경로")
    ap.add_argument("--data", default=str(DATA_YAML),
                    help="data yaml 경로")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.70)
    ap.add_argument("--device", default="0", help="'0'=GPU0, 'cpu'")
    ap.add_argument("--only", choices=["pt", "onnx"], default=None,
                    help="한 가지만 평가")
    args = ap.parse_args()

    model_dir = Path(args.model_dir)
    data_yaml = Path(args.data)

    targets = []
    if args.only in (None, "pt"):
        targets.append(model_dir / "best.pt")
    if args.only in (None, "onnx"):
        targets.append(model_dir / "best.onnx")

    rows = []
    for mp in targets:
        if not mp.exists():
            print(f"[SKIP] 파일 없음: {mp}")
            continue
        rows.append(run(mp, data_yaml, args.imgsz, args.conf, args.iou,
                        args.device))

    print(f"\n{'='*70}\n  벤치마크 요약 (imgsz={args.imgsz} conf={args.conf} "
          f"iou={args.iou} device={args.device})\n{'='*70}")
    hdr = f"{'model':<12}{'P':>8}{'R':>8}{'mAP50':>9}{'mAP50-95':>10}{'infer(ms)':>11}"
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['model']:<12}{r['precision']:>8.4f}{r['recall']:>8.4f}"
              f"{r['mAP50']:>9.4f}{r['mAP50-95']:>10.4f}"
              f"{(r['inference_ms'] or 0):>11.2f}")
    print("\n주의: infer(ms)는 Python(ultralytics) 환경 기준이며 "
          "C++ ONNXRuntime-GPU 배포 FPS와 다르다.")


if __name__ == "__main__":
    main()
