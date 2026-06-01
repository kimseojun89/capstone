from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from pantilt_calibration_utils import (
    DEFAULT_DICTIONARY_NAME,
    DEFAULT_MARKER_LENGTH_MM,
    DEFAULT_SQUARES_X,
    DEFAULT_SQUARES_Y,
    DEFAULT_SQUARE_LENGTH_MM,
    charuco_object_image_points,
    count_ids,
    create_charuco_board,
    detect_charuco,
    draw_detection,
    ensure_dir,
    repo_path,
    reprojection_error_px,
    save_yaml,
    utc_now_iso,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate camera intrinsics from saved ChArUco images.")
    parser.add_argument("--input-dir", type=Path, default=repo_path("data", "intrinsics"))
    parser.add_argument("--output", type=Path, default=repo_path("data", "camera_intrinsics.yaml"))
    parser.add_argument("--patterns", nargs="+", default=["*.png", "*.jpg", "*.jpeg", "*.bmp"])
    parser.add_argument("--min-corners", type=int, default=6)
    parser.add_argument("--min-images", type=int, default=3)
    parser.add_argument("--debug-dir", type=Path, default=None, help="Optional directory for detection overlay images.")
    parser.add_argument("--squares-x", type=int, default=DEFAULT_SQUARES_X)
    parser.add_argument("--squares-y", type=int, default=DEFAULT_SQUARES_Y)
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_LENGTH_MM)
    parser.add_argument("--marker-mm", type=float, default=DEFAULT_MARKER_LENGTH_MM)
    parser.add_argument("--dictionary", default=DEFAULT_DICTIONARY_NAME)
    return parser.parse_args()


def find_images(input_dir: Path, patterns: list[str]) -> list[Path]:
    paths: list[Path] = []
    for pattern in patterns:
        paths.extend(input_dir.glob(pattern))
    return sorted(set(paths))


def main() -> None:
    args = parse_args()
    board, dictionary = create_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_mm,
        args.marker_mm,
        args.dictionary,
    )

    image_paths = find_images(args.input_dir, args.patterns)
    if not image_paths:
        raise RuntimeError(f"No calibration images found in {args.input_dir}")

    if args.debug_dir:
        ensure_dir(args.debug_dir)

    object_points_all: list[np.ndarray] = []
    image_points_all: list[np.ndarray] = []
    used_frames: list[dict[str, object]] = []
    rejected_frames: list[dict[str, object]] = []
    image_size: tuple[int, int] | None = None

    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            rejected_frames.append({"image_file": image_path.name, "reason": "Could not read image."})
            continue

        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            rejected_frames.append(
                {
                    "image_file": image_path.name,
                    "reason": f"Image size {current_size} does not match first image size {image_size}.",
                }
            )
            continue

        detection = detect_charuco(image, board, dictionary)
        corner_count = count_ids(detection["charuco_ids"])
        if args.debug_dir:
            debug = draw_detection(image, detection)
            cv2.imwrite(str(args.debug_dir / f"{image_path.stem}_corners.jpg"), debug)

        if corner_count < args.min_corners:
            rejected_frames.append(
                {
                    "image_file": image_path.name,
                    "reason": f"Only {corner_count} ChArUco corners detected.",
                }
            )
            continue

        obj_points, img_points = charuco_object_image_points(
            board,
            detection["charuco_corners"],
            detection["charuco_ids"],
        )
        object_points_all.append(obj_points)
        image_points_all.append(img_points)
        used_frames.append(
            {
                "image_file": image_path.name,
                "charuco_corners": corner_count,
            }
        )
        print(f"Using {image_path.name}: {corner_count} corners")

    if image_size is None:
        raise RuntimeError("No readable calibration images were found.")
    if len(object_points_all) < args.min_images:
        raise RuntimeError(
            f"Need at least {args.min_images} valid images, got {len(object_points_all)}. "
            f"Rejected {len(rejected_frames)} images."
        )

    # The current OpenCV 4.x Python wheels may omit calibrateCameraCharuco.
    # Matching ChArUco IDs to board points and using calibrateCamera keeps the
    # calibration path compatible across those builds.
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points_all,
        image_points_all,
        image_size,
        None,
        None,
    )

    per_view_errors = []
    for frame, obj_points, img_points, rvec, tvec in zip(
        used_frames,
        object_points_all,
        image_points_all,
        rvecs,
        tvecs,
    ):
        err = reprojection_error_px(obj_points, img_points, rvec, tvec, camera_matrix, dist_coeffs)
        frame["reprojection_error_px"] = err
        per_view_errors.append(err)

    output = {
        "created_utc": utc_now_iso(),
        "opencv_version": cv2.__version__,
        "calibration_model": "ChArUco corners matched to board points, solved with cv2.calibrateCamera",
        "image_size": {
            "width_px": image_size[0],
            "height_px": image_size[1],
        },
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": camera_matrix,
        },
        "distortion_coefficients": {
            "rows": int(dist_coeffs.size),
            "cols": 1,
            "data": dist_coeffs.reshape(-1),
        },
        "rms_reprojection_error_px": float(rms),
        "mean_per_view_reprojection_error_px": float(np.mean(per_view_errors)),
        "board": {
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_mm": args.square_mm,
            "marker_length_mm": args.marker_mm,
            "dictionary": args.dictionary,
        },
        "valid_image_count": len(used_frames),
        "rejected_image_count": len(rejected_frames),
        "used_frames": used_frames,
        "rejected_frames": rejected_frames,
        "units": "mm",
    }
    save_yaml(args.output, output)

    print(f"RMS reprojection error: {rms:.4f} px")
    print(f"Mean per-view reprojection error: {np.mean(per_view_errors):.4f} px")
    print(f"Wrote intrinsics: {args.output}")


if __name__ == "__main__":
    main()
