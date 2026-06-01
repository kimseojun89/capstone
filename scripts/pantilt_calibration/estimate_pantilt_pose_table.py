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
    create_charuco_board,
    draw_detection,
    ensure_dir,
    estimate_charuco_pose,
    load_intrinsics,
    load_json,
    repo_path,
    resolve_image_path,
    save_yaml,
    utc_now_iso,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estimate ChArUco pose for each pan/tilt dataset sample.")
    parser.add_argument("--input-dir", type=Path, default=repo_path("data", "pantilt"))
    parser.add_argument("--intrinsics", type=Path, default=repo_path("data", "camera_intrinsics.yaml"))
    parser.add_argument("--output", type=Path, default=repo_path("data", "pantilt_pose_table.yaml"))
    parser.add_argument("--metadata-pattern", default="*.json")
    parser.add_argument("--min-corners", type=int, default=4)
    parser.add_argument("--debug-dir", type=Path, default=None, help="Optional directory for pose overlay images.")
    parser.add_argument("--squares-x", type=int, default=DEFAULT_SQUARES_X)
    parser.add_argument("--squares-y", type=int, default=DEFAULT_SQUARES_Y)
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_LENGTH_MM)
    parser.add_argument("--marker-mm", type=float, default=DEFAULT_MARKER_LENGTH_MM)
    parser.add_argument("--dictionary", default=DEFAULT_DICTIONARY_NAME)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    camera_matrix, dist_coeffs, intrinsics_yaml = load_intrinsics(args.intrinsics)
    board, dictionary = create_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_mm,
        args.marker_mm,
        args.dictionary,
    )

    metadata_paths = sorted(args.input_dir.glob(args.metadata_pattern))
    if not metadata_paths:
        raise RuntimeError(f"No metadata JSON files found in {args.input_dir}")
    if args.debug_dir:
        ensure_dir(args.debug_dir)

    poses: list[dict[str, object]] = []
    rejected: list[dict[str, object]] = []

    for metadata_path in metadata_paths:
        metadata = load_json(metadata_path)
        if "image_file" not in metadata or "pan_deg" not in metadata or "tilt_deg" not in metadata:
            rejected.append({"metadata_file": metadata_path.name, "reason": "Missing image_file, pan_deg, or tilt_deg."})
            continue

        image_path = resolve_image_path(metadata_path, str(metadata["image_file"]))
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            rejected.append({"metadata_file": metadata_path.name, "reason": f"Could not read image {image_path}."})
            continue

        pose = estimate_charuco_pose(
            image,
            board,
            dictionary,
            camera_matrix,
            dist_coeffs,
            min_corners=args.min_corners,
        )
        if not pose["valid"]:
            rejected.append({"metadata_file": metadata_path.name, "image_file": str(image_path), "reason": pose["reason"]})
            continue

        entry = {
            "sample_id": metadata.get("sample_id", metadata_path.stem),
            "metadata_file": metadata_path.name,
            "image_file": str(image_path),
            "pan_deg": float(metadata["pan_deg"]),
            "tilt_deg": float(metadata["tilt_deg"]),
            "charuco_corner_count": int(pose["charuco_corner_count"]),
            "reprojection_error_px": float(pose["reprojection_error_px"]),
            "rvec_board_to_camera": pose["rvec_board_to_camera"],
            "tvec_board_to_camera_mm": pose["tvec_board_to_camera_mm"],
            "R_board_to_camera": pose["R_board_to_camera"],
            "R_camera_to_world": pose["R_camera_to_world"],
            "t_camera_in_world_mm": pose["t_camera_in_world_mm"],
        }
        poses.append(entry)
        print(
            f"Pose {metadata_path.name}: pan={entry['pan_deg']:.4f}, tilt={entry['tilt_deg']:.4f}, "
            f"corners={entry['charuco_corner_count']}, error={entry['reprojection_error_px']:.3f}px"
        )

        if args.debug_dir:
            debug = draw_detection(
                image,
                pose["detection"],
                camera_matrix,
                dist_coeffs,
                np.asarray(pose["rvec_board_to_camera"], dtype=np.float64),
                np.asarray(pose["tvec_board_to_camera_mm"], dtype=np.float64),
                axis_length_mm=args.square_mm * 2.5,
            )
            cv2.imwrite(str(args.debug_dir / f"{metadata_path.stem}_pose.jpg"), debug)

    if not poses:
        raise RuntimeError(f"No valid poses estimated. Rejected {len(rejected)} samples.")

    reproj_errors = [float(p["reprojection_error_px"]) for p in poses]
    output = {
        "created_utc": utc_now_iso(),
        "opencv_version": cv2.__version__,
        "pose_convention": {
            "world_frame": "ChArUco board coordinate frame in millimeters.",
            "board_to_camera": "X_camera = R_board_to_camera * X_world + tvec_board_to_camera_mm",
            "camera_to_world": "X_world = R_camera_to_world * X_camera + t_camera_in_world_mm",
            "rotation_format": "3x3 row-major matrices; rvec is OpenCV Rodrigues vector.",
        },
        "intrinsics_file": args.intrinsics,
        "intrinsics_image_size": intrinsics_yaml.get("image_size"),
        "board": {
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_mm": args.square_mm,
            "marker_length_mm": args.marker_mm,
            "dictionary": args.dictionary,
        },
        "pose_count": len(poses),
        "mean_reprojection_error_px": float(np.mean(reproj_errors)),
        "max_reprojection_error_px": float(np.max(reproj_errors)),
        "poses": poses,
        "rejected": rejected,
        "units": "mm",
    }
    save_yaml(args.output, output)
    print(f"Wrote pose table: {args.output}")
    print(f"Valid poses: {len(poses)}, rejected samples: {len(rejected)}")


if __name__ == "__main__":
    main()
