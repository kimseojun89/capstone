# Drone YOLOv8x OpenVINO Model

This folder is for the `doguilmak/Drone-Detection-YOLOv8x` model.

Expected files after export:

```text
models/drone_yolov8x/
  best.pt
  best_openvino_model/
    best.xml
    best.bin
    metadata.yaml
```

The original training config uses `imgsz=640`, so export with 640 unless you
retrain or re-export a dynamic model.

Create the OpenVINO model:

```bash
python export_drone_yolov8x_openvino.py
```

Then set `model_path` in `tapo_tracker.py`:

```python
model_path: str = str(DRONE_YOLOV8X_OPENVINO_MODEL)
```
