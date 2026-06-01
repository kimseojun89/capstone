from __future__ import annotations

import csv
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

try:
    import yaml
except ImportError as exc:  # pragma: no cover - handled at runtime.
    yaml = None
    _YAML_IMPORT_ERROR = exc
else:
    _YAML_IMPORT_ERROR = None


SCRIPT_DIR = Path(__file__).resolve().parent


def find_repo_root(start: Path) -> Path:
    """Find the repository root so defaults keep writing under capstone/data."""
    for candidate in (start, *start.parents):
        if (candidate / ".git").exists():
            return candidate
    # Fallback for copied scripts: scripts/pantilt_calibration -> repo root.
    return start.parents[1]


REPO_ROOT = find_repo_root(SCRIPT_DIR)

DEFAULT_SQUARES_X = 7
DEFAULT_SQUARES_Y = 5
DEFAULT_SQUARE_LENGTH_MM = 30.0
DEFAULT_MARKER_LENGTH_MM = 22.0
DEFAULT_DICTIONARY_NAME = "DICT_4X4_50"


def repo_path(*parts: str) -> Path:
    return REPO_ROOT.joinpath(*parts)


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def timestamp_name() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def to_builtin(value: Any) -> Any:
    """Convert numpy/path values so PyYAML and json can serialize them."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): to_builtin(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_builtin(v) for v in value]
    return value


def save_yaml(path: Path, data: dict[str, Any]) -> None:
    if yaml is None:
        raise RuntimeError("PyYAML is required to write YAML files. Install with: pip install pyyaml") from _YAML_IMPORT_ERROR
    ensure_dir(path.parent)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(to_builtin(data), f, sort_keys=False, allow_unicode=False)


def load_yaml(path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError("PyYAML is required to read YAML files. Install with: pip install pyyaml") from _YAML_IMPORT_ERROR
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"YAML file does not contain a mapping: {path}")
    return data


def save_json(path: Path, data: dict[str, Any]) -> None:
    ensure_dir(path.parent)
    with path.open("w", encoding="utf-8") as f:
        json.dump(to_builtin(data), f, indent=2)
        f.write("\n")


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"JSON file does not contain an object: {path}")
    return data


def require_aruco() -> Any:
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "This OpenCV build has no cv2.aruco module. Install opencv-contrib-python 4.x."
        )
    return cv2.aruco


def get_aruco_dictionary(dictionary_name: str = DEFAULT_DICTIONARY_NAME) -> Any:
    aruco = require_aruco()
    if not hasattr(aruco, dictionary_name):
        raise ValueError(f"Unknown aruco dictionary: {dictionary_name}")
    dictionary_id = getattr(aruco, dictionary_name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(aruco, "Dictionary_get"):
        return aruco.Dictionary_get(dictionary_id)
    raise RuntimeError("OpenCV aruco dictionary API is unavailable.")


def create_charuco_board(
    squares_x: int = DEFAULT_SQUARES_X,
    squares_y: int = DEFAULT_SQUARES_Y,
    square_length_mm: float = DEFAULT_SQUARE_LENGTH_MM,
    marker_length_mm: float = DEFAULT_MARKER_LENGTH_MM,
    dictionary_name: str = DEFAULT_DICTIONARY_NAME,
) -> tuple[Any, Any]:
    """Create a ChArUco board while tolerating OpenCV 4.x API differences."""
    aruco = require_aruco()
    dictionary = get_aruco_dictionary(dictionary_name)

    if hasattr(aruco, "CharucoBoard"):
        try:
            board = aruco.CharucoBoard(
                (int(squares_x), int(squares_y)),
                float(square_length_mm),
                float(marker_length_mm),
                dictionary,
            )
        except TypeError:
            board = aruco.CharucoBoard(
                [int(squares_x), int(squares_y)],
                float(square_length_mm),
                float(marker_length_mm),
                dictionary,
            )
        return board, dictionary

    if hasattr(aruco, "CharucoBoard_create"):
        return (
            aruco.CharucoBoard_create(
                int(squares_x),
                int(squares_y),
                float(square_length_mm),
                float(marker_length_mm),
                dictionary,
            ),
            dictionary,
        )

    raise RuntimeError("OpenCV aruco CharucoBoard API is unavailable.")


def draw_charuco_board_image(board: Any, out_size_px: tuple[int, int], margin_px: int, border_bits: int = 1) -> np.ndarray:
    """Render a printable board using either generateImage() or the older draw()."""
    width_px, height_px = int(out_size_px[0]), int(out_size_px[1])
    method = None
    if hasattr(board, "generateImage"):
        method = board.generateImage
    elif hasattr(board, "draw"):
        method = board.draw
    else:
        raise RuntimeError("This OpenCV CharucoBoard has no image generation method.")

    last_error: Exception | None = None
    for call in (
        lambda: method((width_px, height_px), marginSize=int(margin_px), borderBits=int(border_bits)),
        lambda: method((width_px, height_px), None, int(margin_px), int(border_bits)),
        lambda: method((width_px, height_px), int(margin_px), int(border_bits)),
    ):
        try:
            image = call()
            break
        except Exception as exc:  # API signatures vary across 4.x wheels.
            last_error = exc
    else:
        raise RuntimeError(f"Could not render ChArUco board image: {last_error}")

    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image


def detector_parameters() -> Any:
    aruco = require_aruco()
    if hasattr(aruco, "DetectorParameters"):
        return aruco.DetectorParameters()
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return None


def _charuco_parameters(camera_matrix: np.ndarray | None, dist_coeffs: np.ndarray | None) -> Any | None:
    aruco = require_aruco()
    if not hasattr(aruco, "CharucoParameters"):
        return None
    params = aruco.CharucoParameters()
    if camera_matrix is not None:
        params.cameraMatrix = np.asarray(camera_matrix, dtype=np.float64)
    if dist_coeffs is not None:
        params.distCoeffs = np.asarray(dist_coeffs, dtype=np.float64)
    return params


def detect_charuco(
    image: np.ndarray,
    board: Any,
    dictionary: Any,
    camera_matrix: np.ndarray | None = None,
    dist_coeffs: np.ndarray | None = None,
) -> dict[str, Any]:
    """Detect ChArUco corners with new CharucoDetector or older aruco functions."""
    aruco = require_aruco()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    det_params = detector_parameters()

    if hasattr(aruco, "CharucoDetector"):
        charuco_params = _charuco_parameters(camera_matrix, dist_coeffs)
        detector = None
        constructor_errors: list[Exception] = []
        for args in ((board, charuco_params, det_params), (board,),):
            try:
                detector = aruco.CharucoDetector(*args)
                break
            except Exception as exc:  # API signatures vary across 4.x wheels.
                constructor_errors.append(exc)
        if detector is None:
            raise RuntimeError(f"Could not construct CharucoDetector: {constructor_errors[-1]}")
        result = detector.detectBoard(gray)
        if len(result) != 4:
            raise RuntimeError("Unexpected CharucoDetector.detectBoard return shape.")
        charuco_corners, charuco_ids, marker_corners, marker_ids = result
        return {
            "charuco_corners": charuco_corners,
            "charuco_ids": charuco_ids,
            "marker_corners": marker_corners,
            "marker_ids": marker_ids,
        }

    marker_corners = ()
    marker_ids = None
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, det_params)
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)
    elif hasattr(aruco, "detectMarkers"):
        marker_corners, marker_ids, _ = aruco.detectMarkers(gray, dictionary, parameters=det_params)

    if marker_ids is None or len(marker_ids) == 0 or not hasattr(aruco, "interpolateCornersCharuco"):
        return {
            "charuco_corners": None,
            "charuco_ids": None,
            "marker_corners": marker_corners,
            "marker_ids": marker_ids,
        }

    kwargs: dict[str, Any] = {}
    if camera_matrix is not None:
        kwargs["cameraMatrix"] = camera_matrix
    if dist_coeffs is not None:
        kwargs["distCoeffs"] = dist_coeffs
    try:
        _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            marker_corners,
            marker_ids,
            gray,
            board,
            **kwargs,
        )
    except TypeError:
        if camera_matrix is not None and dist_coeffs is not None:
            _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                gray,
                board,
                camera_matrix,
                dist_coeffs,
            )
        else:
            _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                gray,
                board,
            )
    return {
        "charuco_corners": charuco_corners,
        "charuco_ids": charuco_ids,
        "marker_corners": marker_corners,
        "marker_ids": marker_ids,
    }


def count_ids(ids: np.ndarray | None) -> int:
    if ids is None:
        return 0
    return int(np.asarray(ids).reshape(-1).shape[0])


def charuco_object_image_points(
    board: Any,
    charuco_corners: np.ndarray | None,
    charuco_ids: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray]:
    """Return matching 3D board points and 2D image points for ChArUco IDs."""
    if charuco_corners is None or charuco_ids is None or count_ids(charuco_ids) == 0:
        raise ValueError("No ChArUco corners were provided.")

    corners = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 1, 2)
    ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1, 1)

    if hasattr(board, "matchImagePoints"):
        obj_points, img_points = board.matchImagePoints(corners, ids)
        return np.asarray(obj_points, dtype=np.float32), np.asarray(img_points, dtype=np.float32)

    if hasattr(board, "getChessboardCorners"):
        all_obj = np.asarray(board.getChessboardCorners(), dtype=np.float32)
    elif hasattr(board, "chessboardCorners"):
        all_obj = np.asarray(board.chessboardCorners, dtype=np.float32)
    else:
        raise RuntimeError("This OpenCV CharucoBoard cannot expose chessboard object points.")

    flat_ids = ids.reshape(-1)
    obj = all_obj[flat_ids].reshape(-1, 1, 3)
    return obj.astype(np.float32), corners.astype(np.float32)


def draw_detection(
    image: np.ndarray,
    detection: dict[str, Any],
    camera_matrix: np.ndarray | None = None,
    dist_coeffs: np.ndarray | None = None,
    rvec: np.ndarray | None = None,
    tvec: np.ndarray | None = None,
    axis_length_mm: float = 75.0,
) -> np.ndarray:
    """Draw marker/corner detections and an optional pose axis for debugging."""
    aruco = require_aruco()
    out = image.copy()
    marker_ids = detection.get("marker_ids")
    marker_corners = detection.get("marker_corners")
    charuco_ids = detection.get("charuco_ids")
    charuco_corners = detection.get("charuco_corners")

    if marker_ids is not None and count_ids(marker_ids) > 0 and hasattr(aruco, "drawDetectedMarkers"):
        aruco.drawDetectedMarkers(out, marker_corners, marker_ids)
    if charuco_ids is not None and count_ids(charuco_ids) > 0 and hasattr(aruco, "drawDetectedCornersCharuco"):
        aruco.drawDetectedCornersCharuco(out, charuco_corners, charuco_ids, (255, 0, 0))
    if camera_matrix is not None and dist_coeffs is not None and rvec is not None and tvec is not None:
        cv2.drawFrameAxes(out, camera_matrix, dist_coeffs, rvec, tvec, float(axis_length_mm))
    return out


def matrix_from_yaml(value: Any, shape: tuple[int, ...] | None = None) -> np.ndarray:
    if isinstance(value, dict) and "data" in value:
        value = value["data"]
    arr = np.asarray(value, dtype=np.float64)
    if shape is not None:
        arr = arr.reshape(shape)
    return arr


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    data = load_yaml(path)
    if "camera_matrix" not in data or "distortion_coefficients" not in data:
        raise ValueError(f"Missing camera_matrix or distortion_coefficients in {path}")
    camera_matrix = matrix_from_yaml(data["camera_matrix"], (3, 3))
    dist_coeffs = matrix_from_yaml(data["distortion_coefficients"]).reshape(-1, 1)
    return camera_matrix, dist_coeffs, data


def rotation_matrix_from_rvec(rvec: np.ndarray) -> np.ndarray:
    rot, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return rot


def invert_transform(rotation: np.ndarray, translation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    translation = np.asarray(translation, dtype=np.float64).reshape(3, 1)
    inv_rotation = rotation.T
    inv_translation = -inv_rotation @ translation
    return inv_rotation, inv_translation.reshape(3)


def estimate_charuco_pose(
    image: np.ndarray,
    board: Any,
    dictionary: Any,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    min_corners: int = 4,
) -> dict[str, Any]:
    detection = detect_charuco(image, board, dictionary, camera_matrix, dist_coeffs)
    n_corners = count_ids(detection["charuco_ids"])
    if n_corners < min_corners:
        return {"valid": False, "reason": f"Only {n_corners} ChArUco corners detected.", "detection": detection}

    obj_points, img_points = charuco_object_image_points(
        board,
        detection["charuco_corners"],
        detection["charuco_ids"],
    )
    ok, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return {"valid": False, "reason": "cv2.solvePnP returned false.", "detection": detection}

    error_px = reprojection_error_px(obj_points, img_points, rvec, tvec, camera_matrix, dist_coeffs)
    rotation = rotation_matrix_from_rvec(rvec)
    inv_rotation, inv_translation = invert_transform(rotation, tvec)
    return {
        "valid": True,
        "detection": detection,
        "object_points": obj_points,
        "image_points": img_points,
        "rvec_board_to_camera": rvec.reshape(3),
        "tvec_board_to_camera_mm": tvec.reshape(3),
        "R_board_to_camera": rotation,
        "R_camera_to_world": inv_rotation,
        "t_camera_in_world_mm": inv_translation,
        "reprojection_error_px": error_px,
        "charuco_corner_count": n_corners,
    }


def reprojection_error_px(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    err = np.linalg.norm(projected.reshape(-1, 2) - image_points.reshape(-1, 2), axis=1)
    return float(np.mean(err))


def normalize(vector: Iterable[float]) -> np.ndarray:
    arr = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-12:
        raise ValueError("Cannot normalize a near-zero vector.")
    return arr / norm


def parse_point_triplet(text: str) -> list[float]:
    parts = [p.strip() for p in text.replace(" ", ",").split(",") if p.strip()]
    if len(parts) != 3:
        raise ValueError(f"Expected x,y,z triplet, got: {text}")
    return [float(p) for p in parts]


def parse_points_text(text: str) -> np.ndarray:
    rows = [row.strip() for row in text.split(";") if row.strip()]
    return np.asarray([parse_point_triplet(row) for row in rows], dtype=np.float64)


def load_points_file(path: Path) -> np.ndarray:
    if path.suffix.lower() == ".json":
        data = load_json(path)
        points = data.get("points", data.get("reference_points_mm"))
        if points is None:
            raise ValueError(f"JSON points file must contain points or reference_points_mm: {path}")
        return np.asarray(points, dtype=np.float64)

    points: list[list[float]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            points.append([float(row[0]), float(row[1]), float(row[2])])
    return np.asarray(points, dtype=np.float64)


def fit_plane(points: np.ndarray, origin_mode: str = "first", flip_normal: bool = False) -> dict[str, Any]:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < 3:
        raise ValueError("At least three 3D reference points are required.")

    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = normalize(vh[-1])

    # Align the SVD normal with the first non-collinear input triplet so the
    # axis orientation is controlled by the user's point order.
    reference_normal = None
    for i in range(1, points.shape[0] - 1):
        candidate = np.cross(points[i] - points[0], points[i + 1] - points[0])
        if np.linalg.norm(candidate) > 1e-9:
            reference_normal = normalize(candidate)
            break
    if reference_normal is None:
        raise ValueError("Reference points are collinear; cannot fit a plane.")
    if float(np.dot(normal, reference_normal)) < 0:
        normal = -normal
    if flip_normal:
        normal = -normal

    if origin_mode == "centroid":
        origin_source = centroid
    elif origin_mode == "first":
        origin_source = points[0]
    else:
        raise ValueError("origin_mode must be 'first' or 'centroid'.")
    # Keep the 2D coordinate origin on the best-fit plane even when the input
    # reference points contain small measurement noise.
    origin = origin_source - normal * float(np.dot(origin_source - centroid, normal))

    x_axis = points[1] - origin
    x_axis = x_axis - normal * float(np.dot(x_axis, normal))
    if np.linalg.norm(x_axis) < 1e-9:
        x_axis = points[2] - origin
        x_axis = x_axis - normal * float(np.dot(x_axis, normal))
    x_axis = normalize(x_axis)
    y_axis = normalize(np.cross(normal, x_axis))

    signed_distances = centered @ normal
    rms_mm = float(np.sqrt(np.mean(np.square(signed_distances))))
    max_abs_mm = float(np.max(np.abs(signed_distances)))
    d = -float(np.dot(normal, centroid))

    return {
        "reference_points_mm": points,
        "centroid_mm": centroid,
        "origin_source_mm": origin_source,
        "origin_mm": origin,
        "normal": normal,
        "x_axis": x_axis,
        "y_axis": y_axis,
        "plane_equation": {
            "a": float(normal[0]),
            "b": float(normal[1]),
            "c": float(normal[2]),
            "d": d,
            "form": "a*x + b*y + c*z + d = 0",
        },
        "fit_error": {
            "rms_mm": rms_mm,
            "max_abs_mm": max_abs_mm,
        },
    }


def load_plane(path: Path) -> dict[str, Any]:
    data = load_yaml(path)
    for key in ("origin_mm", "normal", "x_axis", "y_axis"):
        if key not in data:
            raise ValueError(f"Missing {key} in antenna plane YAML: {path}")
    data["origin_mm"] = matrix_from_yaml(data["origin_mm"], (3,))
    data["normal"] = normalize(data["normal"])
    data["x_axis"] = normalize(data["x_axis"])
    data["y_axis"] = normalize(data["y_axis"])
    return data


def ray_plane_intersection(
    ray_origin: np.ndarray,
    ray_direction: np.ndarray,
    plane_origin: np.ndarray,
    plane_normal: np.ndarray,
) -> tuple[np.ndarray, float]:
    ray_origin = np.asarray(ray_origin, dtype=np.float64).reshape(3)
    ray_direction = normalize(ray_direction)
    plane_origin = np.asarray(plane_origin, dtype=np.float64).reshape(3)
    plane_normal = normalize(plane_normal)
    denom = float(np.dot(plane_normal, ray_direction))
    if abs(denom) < 1e-9:
        raise ValueError("Ray is parallel to the antenna plane.")
    ray_scale = float(np.dot(plane_normal, plane_origin - ray_origin) / denom)
    point = ray_origin + ray_scale * ray_direction
    return point, ray_scale


def plane_xy(point_world: np.ndarray, plane: dict[str, Any]) -> tuple[float, float]:
    delta = np.asarray(point_world, dtype=np.float64).reshape(3) - np.asarray(plane["origin_mm"], dtype=np.float64).reshape(3)
    return float(np.dot(delta, plane["x_axis"])), float(np.dot(delta, plane["y_axis"]))


def open_capture(camera_index: int, api: str = "default") -> cv2.VideoCapture:
    api_map = {
        "default": 0,
        "any": cv2.CAP_ANY,
        "dshow": getattr(cv2, "CAP_DSHOW", 700),
        "msmf": getattr(cv2, "CAP_MSMF", 1400),
    }
    if api not in api_map:
        raise ValueError(f"Unknown camera API: {api}")
    if api == "default":
        return cv2.VideoCapture(int(camera_index))
    return cv2.VideoCapture(int(camera_index), api_map[api])


def configure_capture(cap: cv2.VideoCapture, width: int | None, height: int | None, fps: float | None) -> None:
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    if fps:
        cap.set(cv2.CAP_PROP_FPS, float(fps))


def resolve_image_path(metadata_path: Path, image_file: str) -> Path:
    image_path = Path(image_file)
    if image_path.is_absolute():
        return image_path
    candidate = metadata_path.parent / image_path
    if candidate.exists():
        return candidate
    return REPO_ROOT / image_path


def nearest_pose_entry(
    entries: list[dict[str, Any]],
    pan_deg: float,
    tilt_deg: float,
    max_angle_error_deg: float | None = None,
) -> tuple[dict[str, Any], float]:
    if not entries:
        raise ValueError("Pose table contains no entries.")

    def err(entry: dict[str, Any]) -> float:
        dp = float(entry["pan_deg"]) - float(pan_deg)
        dt = float(entry["tilt_deg"]) - float(tilt_deg)
        return math.sqrt(dp * dp + dt * dt)

    best = min(entries, key=err)
    best_err = err(best)
    if max_angle_error_deg is not None and best_err > max_angle_error_deg:
        raise ValueError(
            f"No pose table entry within {max_angle_error_deg} deg of pan={pan_deg}, tilt={tilt_deg}. "
            f"Nearest error is {best_err:.4f} deg."
        )
    return best, float(best_err)


def sleep_for_ui() -> None:
    # Tiny delay keeps OpenCV windows responsive after console input on Windows.
    time.sleep(0.05)
