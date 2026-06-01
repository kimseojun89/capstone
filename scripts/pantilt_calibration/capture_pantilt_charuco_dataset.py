from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from pantilt_calibration_utils import (
    DEFAULT_DICTIONARY_NAME,
    DEFAULT_MARKER_LENGTH_MM,
    DEFAULT_SQUARES_X,
    DEFAULT_SQUARES_Y,
    DEFAULT_SQUARE_LENGTH_MM,
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture pan/tilt webcam images and per-sample metadata.")
    parser.add_argument("--camera-index", type=int, default=1)
    parser.add_argument("--camera-api", choices=["default", "any", "dshow", "msmf"], default="dshow")
    # 실제 운용 해상도 1920×1080 (settings.hpp cameraWidth/cameraHeight 기본값 일치)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--output-dir", type=Path, default=repo_path("data", "pantilt"))
    parser.add_argument("--prefix", default="pantilt")
    parser.add_argument("--ext", choices=["png", "jpg"], default="png")
    parser.add_argument("--pan-deg", type=float, default=None, help="Fixed pan angle for all captured samples.")
    parser.add_argument("--tilt-deg", type=float, default=None, help="Fixed tilt angle for all captured samples.")
    parser.add_argument("--require-charuco", action="store_true", help="Do not save frames with fewer than min corners.")
    parser.add_argument("--min-corners", type=int, default=4)
    parser.add_argument("--show-detections", action="store_true", help="Overlay detected ChArUco corners in preview.")
    parser.add_argument("--squares-x", type=int, default=DEFAULT_SQUARES_X)
    parser.add_argument("--squares-y", type=int, default=DEFAULT_SQUARES_Y)
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_LENGTH_MM)
    parser.add_argument("--marker-mm", type=float, default=DEFAULT_MARKER_LENGTH_MM)
    parser.add_argument("--dictionary", default=DEFAULT_DICTIONARY_NAME)
    return parser.parse_args()


def prompt_angle(name: str, default: float | None) -> float:
    if default is not None:
        return float(default)
    while True:
        raw = input(f"Enter {name} angle in degrees: ").strip()
        try:
            return float(raw)
        except ValueError:
            print("Invalid number; try again.")


def main() -> None:
    args = parse_args()
    output_dir = ensure_dir(args.output_dir)
    board, dictionary = create_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_mm,
        args.marker_mm,
        args.dictionary,
    )

    cap = open_capture(args.camera_index, args.camera_api)
    configure_capture(cap, args.width, args.height, args.fps)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open webcam index {args.camera_index}.")

    session_id = timestamp_name()
    samples: list[dict[str, object]] = []
    index_yaml = output_dir / "pantilt_dataset_index.yaml"
    window = "Capture pan/tilt dataset - SPACE save, q/ESC quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print("Press SPACE to save a sample. Press q or ESC to quit.")
    if args.pan_deg is None or args.tilt_deg is None:
        print("When SPACE is pressed, pan/tilt angles will be requested in this terminal.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Failed to read frame from webcam.")

            detection = detect_charuco(frame, board, dictionary)
            corner_count = count_ids(detection["charuco_ids"])
            preview = draw_detection(frame, detection) if args.show_detections else frame.copy()
            cv2.putText(
                preview,
                f"corners={corner_count} samples={len(samples)}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(window, preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key != ord(" "):
                continue

            if args.require_charuco and corner_count < args.min_corners:
                print(f"Skipped frame: only {corner_count} ChArUco corners detected.")
                continue

            cv2.imshow(window, preview)
            cv2.waitKey(1)
            pan_deg = prompt_angle("pan", args.pan_deg)
            tilt_deg = prompt_angle("tilt", args.tilt_deg)

            sample_id = f"{args.prefix}_{session_id}_{len(samples):03d}"
            image_path = output_dir / f"{sample_id}.{args.ext}"
            metadata_path = output_dir / f"{sample_id}.json"
            if not cv2.imwrite(str(image_path), frame):
                raise RuntimeError(f"Failed to write image: {image_path}")

            metadata = {
                "sample_id": sample_id,
                "captured_utc": utc_now_iso(),
                "image_file": image_path.name,
                "metadata_file": metadata_path.name,
                "pan_deg": pan_deg,
                "tilt_deg": tilt_deg,
                "charuco_corners_detected": corner_count,
                "width_px": int(frame.shape[1]),
                "height_px": int(frame.shape[0]),
                "camera_index": args.camera_index,
                "camera_api": args.camera_api,
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
            samples.append(metadata)
            save_yaml(
                index_yaml,
                {
                    "updated_utc": utc_now_iso(),
                    "output_dir": output_dir,
                    "sample_count": len(samples),
                    "samples": samples,
                    "units": "mm",
                },
            )
            print(f"Saved {image_path.name}, pan={pan_deg:.4f}, tilt={tilt_deg:.4f}, corners={corner_count}")
            sleep_for_ui()
    finally:
        cap.release()
        cv2.destroyAllWindows()

    print(f"Saved {len(samples)} samples in {output_dir}")
    print(f"Wrote YAML index: {index_yaml}")


if __name__ == "__main__":
    main()
