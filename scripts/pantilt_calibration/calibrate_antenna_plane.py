from __future__ import annotations

import argparse
from pathlib import Path

from pantilt_calibration_utils import (
    fit_plane,
    load_points_file,
    parse_point_triplet,
    parse_points_text,
    repo_path,
    save_yaml,
    utc_now_iso,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit the antenna plane from at least three reference points. "
            "Input points must be in the same world frame as pantilt_pose_table.yaml."
        )
    )
    parser.add_argument(
        "--points",
        default=None,
        help='Semicolon separated x,y,z points in mm. Example: "0,0,0;100,0,0;0,100,0"',
    )
    parser.add_argument("--points-file", type=Path, default=None, help="CSV or JSON file containing x,y,z points in mm.")
    parser.add_argument("--origin", choices=["first", "centroid"], default="first")
    parser.add_argument("--flip-normal", action="store_true")
    parser.add_argument("--output", type=Path, default=repo_path("data", "antenna_plane.yaml"))
    return parser.parse_args()


def prompt_points() -> list[list[float]]:
    print("Enter antenna reference points as x,y,z in mm. Submit an empty line when done.")
    print("Use coordinates in the same world frame as pantilt_pose_table.yaml.")
    points: list[list[float]] = []
    while True:
        raw = input(f"point {len(points)}: ").strip()
        if not raw:
            break
        points.append(parse_point_triplet(raw))
    return points


def main() -> None:
    args = parse_args()
    if args.points and args.points_file:
        raise ValueError("Use either --points or --points-file, not both.")

    if args.points:
        points = parse_points_text(args.points)
    elif args.points_file:
        points = load_points_file(args.points_file)
    else:
        points = prompt_points()

    plane = fit_plane(points, origin_mode=args.origin, flip_normal=args.flip_normal)
    output = {
        "created_utc": utc_now_iso(),
        "coordinate_convention": {
            "input_points": "3D points in the same world frame as pantilt_pose_table.yaml.",
            "plane_xy": "x = dot(P - origin_mm, x_axis), y = dot(P - origin_mm, y_axis)",
            "units": "mm",
        },
        **plane,
        "units": "mm",
    }
    save_yaml(args.output, output)

    print(f"Wrote antenna plane: {args.output}")
    print(f"Normal: {plane['normal']}")
    print(f"Origin: {plane['origin_mm']}")
    print(f"Fit RMS: {plane['fit_error']['rms_mm']:.6f} mm")


if __name__ == "__main__":
    main()
