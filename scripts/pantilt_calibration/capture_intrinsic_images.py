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
    save_yaml,
    sleep_for_ui,
    timestamp_name,
    utc_now_iso,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture webcam images for ChArUco intrinsic calibration.")
    parser.add_argument("--camera-index", type=int, default=1)
    parser.add_argument("--camera-api", choices=["default", "any", "dshow", "msmf"], default="dshow")
    # 실제 운용 해상도 1920×1080 (settings.hpp cameraWidth/cameraHeight 기본값 일치)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--output-dir", type=Path, default=repo_path("data", "intrinsics"))
    parser.add_argument("--prefix", default="intrinsic")
    parser.add_argument("--ext", choices=["png", "jpg"], default="png")
    parser.add_argument("--show-detections", action="store_true", help="Overlay detected ChArUco corners in preview.")
    parser.add_argument("--squares-x", type=int, default=DEFAULT_SQUARES_X)
    parser.add_argument("--squares-y", type=int, default=DEFAULT_SQUARES_Y)
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_LENGTH_MM)
    parser.add_argument("--marker-mm", type=float, default=DEFAULT_MARKER_LENGTH_MM)
    parser.add_argument("--dictionary", default=DEFAULT_DICTIONARY_NAME)
    return parser.parse_args()


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
    saved: list[dict[str, object]] = []
    index_yaml = output_dir / "intrinsic_capture_index.yaml"
    window = "Capture intrinsics - SPACE save, q/ESC quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print("Press SPACE to save a frame. Press q or ESC to quit.")

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
                f"corners={corner_count} saved={len(saved)}",
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
            if key == ord(" "):
                filename = f"{args.prefix}_{session_id}_{len(saved):03d}.{args.ext}"
                image_path = output_dir / filename
                if not cv2.imwrite(str(image_path), frame):
                    raise RuntimeError(f"Failed to write image: {image_path}")
                record = {
                    "image_file": image_path.name,
                    "captured_utc": utc_now_iso(),
                    "charuco_corners_detected": corner_count,
                    "width_px": int(frame.shape[1]),
                    "height_px": int(frame.shape[0]),
                }
                saved.append(record)
                save_yaml(
                    index_yaml,
                    {
                        "updated_utc": utc_now_iso(),
                        "camera_index": args.camera_index,
                        "camera_api": args.camera_api,
                        "output_dir": output_dir,
                        "board": {
                            "squares_x": args.squares_x,
                            "squares_y": args.squares_y,
                            "square_length_mm": args.square_mm,
                            "marker_length_mm": args.marker_mm,
                            "dictionary": args.dictionary,
                        },
                        "images": saved,
                        "units": "mm",
                    },
                )
                print(f"Saved {image_path} ({corner_count} corners)")
                sleep_for_ui()
    finally:
        cap.release()
        cv2.destroyAllWindows()

    print(f"Saved {len(saved)} images in {output_dir}")
    print(f"Wrote YAML index: {index_yaml}")


if __name__ == "__main__":
    main()
