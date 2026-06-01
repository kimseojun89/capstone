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
    draw_charuco_board_image,
    ensure_dir,
    repo_path,
    save_yaml,
    utc_now_iso,
)


PAGE_SIZES_MM = {
    "auto": None,
    "a4": (297.0, 210.0),
    "a4-landscape": (297.0, 210.0),
    "a4-portrait": (210.0, 297.0),
    "a2": (594.0, 420.0),
    "a2-landscape": (594.0, 420.0),
    "a2-portrait": (420.0, 594.0),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a printable ChArUco board PNG/PDF for pan/tilt calibration."
    )
    parser.add_argument("--squares-x", type=int, default=DEFAULT_SQUARES_X)
    parser.add_argument("--squares-y", type=int, default=DEFAULT_SQUARES_Y)
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_LENGTH_MM)
    parser.add_argument("--marker-mm", type=float, default=DEFAULT_MARKER_LENGTH_MM)
    parser.add_argument("--dictionary", default=DEFAULT_DICTIONARY_NAME)
    parser.add_argument("--dpi", type=int, default=600)
    parser.add_argument("--margin-mm", type=float, default=20.0)
    parser.add_argument(
        "--page-size",
        choices=sorted(PAGE_SIZES_MM.keys()),
        default="a4",
        help="'a4' is A4 landscape. 'auto' uses board size plus margins.",
    )
    parser.add_argument("--page-width-mm", type=float, default=None, help="Override output page width in mm.")
    parser.add_argument("--page-height-mm", type=float, default=None, help="Override output page height in mm.")
    parser.add_argument("--border-bits", type=int, default=1)
    parser.add_argument("--output-dir", type=Path, default=repo_path("data", "charuco", "a4"))
    parser.add_argument("--name", default="charuco_a4_7x5_30mm_600dpi")
    return parser.parse_args()


def save_pdf_with_pillow(image: np.ndarray, pdf_path: Path, dpi: int) -> None:
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("Pillow is required to write PDF output. Install with: pip install pillow") from exc

    Image.MAX_IMAGE_PIXELS = None
    pil_image = Image.fromarray(image)
    if pil_image.mode != "L":
        pil_image = pil_image.convert("L")
    pil_image.save(pdf_path, "PDF", resolution=float(dpi))


def save_png_with_dpi(image: np.ndarray, png_path: Path, dpi: int) -> None:
    try:
        from PIL import Image
    except ImportError:
        ok = cv2.imwrite(str(png_path), image)
        if not ok:
            raise RuntimeError(f"Failed to write PNG: {png_path}")
        return

    Image.MAX_IMAGE_PIXELS = None
    pil_image = Image.fromarray(image)
    if pil_image.mode != "L":
        pil_image = pil_image.convert("L")
    pil_image.save(png_path, "PNG", dpi=(float(dpi), float(dpi)))


def make_print_canvas(
    board_image: np.ndarray,
    page_width_px: int,
    page_height_px: int,
) -> tuple[np.ndarray, int, int]:
    if board_image.shape[1] > page_width_px or board_image.shape[0] > page_height_px:
        raise ValueError(
            "The requested page is smaller than the exact-size board plus margins. "
            "Increase page size or reduce square/margin dimensions."
        )
    canvas = np.full((page_height_px, page_width_px), 255, dtype=np.uint8)
    offset_x = (page_width_px - board_image.shape[1]) // 2
    offset_y = (page_height_px - board_image.shape[0]) // 2
    canvas[
        offset_y : offset_y + board_image.shape[0],
        offset_x : offset_x + board_image.shape[1],
    ] = board_image
    return canvas, offset_x, offset_y


def main() -> None:
    args = parse_args()
    output_dir = ensure_dir(args.output_dir)

    board, _ = create_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_mm,
        args.marker_mm,
        args.dictionary,
    )

    px_per_mm = float(args.dpi) / 25.4
    board_width_mm = args.squares_x * args.square_mm
    board_height_mm = args.squares_y * args.square_mm
    exact_width_mm = board_width_mm + 2.0 * args.margin_mm
    exact_height_mm = board_height_mm + 2.0 * args.margin_mm

    page_size = PAGE_SIZES_MM[args.page_size]
    if args.page_width_mm is not None or args.page_height_mm is not None:
        if args.page_width_mm is None or args.page_height_mm is None:
            raise ValueError("--page-width-mm and --page-height-mm must be provided together.")
        page_width_mm = float(args.page_width_mm)
        page_height_mm = float(args.page_height_mm)
        page_size_name = "custom"
    elif page_size is not None:
        page_width_mm, page_height_mm = page_size
        page_size_name = args.page_size
    else:
        page_width_mm = exact_width_mm
        page_height_mm = exact_height_mm
        page_size_name = "auto"

    page_width_px = int(round(page_width_mm * px_per_mm))
    page_height_px = int(round(page_height_mm * px_per_mm))
    exact_width_px = int(round(exact_width_mm * px_per_mm))
    exact_height_px = int(round(exact_height_mm * px_per_mm))
    margin_px = int(round(args.margin_mm * px_per_mm))

    # Render the board at its exact physical size first. The final page
    # page may be larger, but square_length_mm must stay true for calibration.
    exact_board_image = draw_charuco_board_image(
        board,
        (exact_width_px, exact_height_px),
        margin_px,
        args.border_bits,
    )
    image, offset_x_px, offset_y_px = make_print_canvas(
        exact_board_image,
        page_width_px,
        page_height_px,
    )

    png_path = output_dir / f"{args.name}.png"
    pdf_path = output_dir / f"{args.name}.pdf"
    yaml_path = output_dir / f"{args.name}.yaml"

    # PNG/PDF DPI metadata is useful for print dialogs that honor image DPI.
    save_png_with_dpi(image, png_path, args.dpi)
    save_pdf_with_pillow(image, pdf_path, args.dpi)

    save_yaml(
        yaml_path,
        {
            "created_utc": utc_now_iso(),
            "opencv_version": cv2.__version__,
            "board": {
                "squares_x": args.squares_x,
                "squares_y": args.squares_y,
                "square_length_mm": args.square_mm,
                "marker_length_mm": args.marker_mm,
                "dictionary": args.dictionary,
                "border_bits": args.border_bits,
            },
            "print": {
                "dpi": args.dpi,
                "page_size": page_size_name,
                "margin_mm": args.margin_mm,
                "board_width_mm": board_width_mm,
                "board_height_mm": board_height_mm,
                "exact_board_canvas_width_mm": exact_width_mm,
                "exact_board_canvas_height_mm": exact_height_mm,
                "page_width_mm": page_width_mm,
                "page_height_mm": page_height_mm,
                "exact_board_canvas_width_px": exact_width_px,
                "exact_board_canvas_height_px": exact_height_px,
                "page_width_px": page_width_px,
                "page_height_px": page_height_px,
                "margin_px": margin_px,
                "offset_x_px": offset_x_px,
                "offset_y_px": offset_y_px,
                "preserves_square_length_mm": True,
            },
            "files": {
                "png": png_path,
                "pdf": pdf_path,
            },
            "units": "mm",
        },
    )

    print(f"Wrote PNG: {png_path}")
    print(f"Wrote PDF: {pdf_path}")
    print(f"Wrote YAML: {yaml_path}")


if __name__ == "__main__":
    main()
