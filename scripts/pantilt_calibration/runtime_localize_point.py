from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from pantilt_calibration_utils import (
    load_intrinsics,
    load_plane,
    load_yaml,
    matrix_from_yaml,
    nearest_pose_entry,
    normalize,
    plane_xy,
    ray_plane_intersection,
    repo_path,
    save_yaml,
    utc_now_iso,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Project an image pixel onto the calibrated antenna plane.")
    parser.add_argument("--image", type=Path, required=True, help="Image associated with this detection; used for bounds checks.")
    parser.add_argument("--pan-deg", type=float, required=True)
    parser.add_argument("--tilt-deg", type=float, required=True)
    parser.add_argument("--u", type=float, required=True, help="Target pixel u coordinate in pixels.")
    parser.add_argument("--v", type=float, required=True, help="Target pixel v coordinate in pixels.")
    parser.add_argument("--intrinsics", type=Path, default=repo_path("data", "camera_intrinsics.yaml"))
    parser.add_argument("--pose-table", type=Path, default=repo_path("data", "pantilt_pose_table.yaml"))
    parser.add_argument("--plane", type=Path, default=repo_path("data", "antenna_plane.yaml"))
    parser.add_argument(
        "--max-angle-error-deg",
        type=float,
        default=0.25,
        help="Reject nearest pose-table entry if pan/tilt distance is larger. Use a negative value to disable.",
    )
    parser.add_argument("--allow-behind-camera", action="store_true")
    parser.add_argument("--output-yaml", type=Path, default=repo_path("data", "runtime", "localization_result.yaml"))
    parser.add_argument("--overlay", type=Path, default=None, help="Optional output image with the input pixel marked.")
    return parser.parse_args()


def load_pose_entries(path: Path) -> list[dict[str, object]]:
    data = load_yaml(path)
    entries = data.get("poses")
    if not isinstance(entries, list):
        raise ValueError(f"Pose table has no poses list: {path}")
    return entries


def main() -> None:
    args = parse_args()
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Could not read image: {args.image}")

    height, width = image.shape[:2]
    if not (0.0 <= args.u < width and 0.0 <= args.v < height):
        print(f"Warning: pixel ({args.u}, {args.v}) is outside image bounds {width}x{height}.")

    camera_matrix, dist_coeffs, _ = load_intrinsics(args.intrinsics)
    pose_entries = load_pose_entries(args.pose_table)
    plane = load_plane(args.plane)

    max_angle_error = None if args.max_angle_error_deg < 0 else args.max_angle_error_deg
    pose_entry, pose_angle_error = nearest_pose_entry(
        pose_entries,
        args.pan_deg,
        args.tilt_deg,
        max_angle_error_deg=max_angle_error,
    )

    # Convert the distorted pixel to a normalized camera ray [x, y, 1].
    pixel = np.asarray([[[args.u, args.v]]], dtype=np.float64)
    undistorted = cv2.undistortPoints(pixel, camera_matrix, dist_coeffs)
    x_norm, y_norm = undistorted.reshape(2)
    ray_dir_camera = normalize([x_norm, y_norm, 1.0])

    # Pose table entries define the camera pose in the ChArUco/world frame.
    rotation_camera_to_world = matrix_from_yaml(pose_entry["R_camera_to_world"], (3, 3))
    camera_origin_world = matrix_from_yaml(pose_entry["t_camera_in_world_mm"], (3,))
    ray_dir_world = normalize(rotation_camera_to_world @ ray_dir_camera)

    intersection_world, ray_scale_mm = ray_plane_intersection(
        camera_origin_world,
        ray_dir_world,
        plane["origin_mm"],
        plane["normal"],
    )
    if ray_scale_mm < 0.0 and not args.allow_behind_camera:
        raise RuntimeError(
            f"Ray-plane intersection is behind the camera (ray scale {ray_scale_mm:.3f} mm). "
            "Check pose/plane conventions or pass --allow-behind-camera to record it anyway."
        )

    x_mm, y_mm = plane_xy(intersection_world, plane)
    result = {
        "created_utc": utc_now_iso(),
        "input": {
            "image": args.image,
            "pan_deg": args.pan_deg,
            "tilt_deg": args.tilt_deg,
            "pixel": {
                "u": args.u,
                "v": args.v,
            },
        },
        "selected_pose": {
            "sample_id": pose_entry.get("sample_id"),
            "pan_deg": float(pose_entry["pan_deg"]),
            "tilt_deg": float(pose_entry["tilt_deg"]),
            "angle_error_deg": pose_angle_error,
            "reprojection_error_px": pose_entry.get("reprojection_error_px"),
        },
        "ray": {
            "origin_world_mm": camera_origin_world,
            "direction_camera": ray_dir_camera,
            "direction_world": ray_dir_world,
            "ray_scale_mm": ray_scale_mm,
        },
        "intersection_world_mm": intersection_world,
        "antenna_plane_xy_mm": {
            "x": x_mm,
            "y": y_mm,
        },
        "units": "mm",
    }
    save_yaml(args.output_yaml, result)

    if args.overlay:
        overlay = image.copy()
        cv2.drawMarker(
            overlay,
            (int(round(args.u)), int(round(args.v))),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
        )
        cv2.putText(
            overlay,
            f"x={x_mm:.1f}mm y={y_mm:.1f}mm",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        args.overlay.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.overlay), overlay)

    print(f"antenna_x_mm: {x_mm:.3f}")
    print(f"antenna_y_mm: {y_mm:.3f}")
    print(f"world_point_mm: {intersection_world.tolist()}")
    print(f"selected_pose_sample: {pose_entry.get('sample_id')} (angle error {pose_angle_error:.4f} deg)")
    print(f"Wrote YAML: {args.output_yaml}")


if __name__ == "__main__":
    main()
