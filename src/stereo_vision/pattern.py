"""Generate the chessboard calibration target, as a printable PDF or a raster.

Both outputs come from the same square layout, so the raster used in the tests
is the pattern that ends up on paper.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .config import BoardSpec

MM_TO_PT = 72.0 / 25.4

# Landscape page sizes in millimetres.
PAPER_SIZES_MM = {
    "a4": (297.0, 210.0),
    "letter": (279.4, 215.9),
}

# Most printers cannot print right up to the edge, and the corner detector
# wants a white quiet zone around the outer squares.
MIN_MARGIN_MM = 10.0


def black_squares(board: BoardSpec) -> list[tuple[int, int]]:
    """(column, row) of every black square, counted from the top-left.

    A board with N inner corners across has N + 1 squares across. The top-left
    square is black.
    """
    columns = board.columns + 1
    rows = board.rows + 1
    return [(c, r) for r in range(rows) for c in range(columns) if (c + r) % 2 == 0]


def render_chessboard(
    board: BoardSpec, pixels_per_square: int = 40, margin_squares: float = 1.0
) -> np.ndarray:
    """Draw the board as a grayscale image with a white border."""
    margin = int(round(margin_squares * pixels_per_square))
    width = (board.columns + 1) * pixels_per_square + 2 * margin
    height = (board.rows + 1) * pixels_per_square + 2 * margin
    image = np.full((height, width), 255, np.uint8)
    for column, row in black_squares(board):
        x = margin + column * pixels_per_square
        y = margin + row * pixels_per_square
        image[y : y + pixels_per_square, x : x + pixels_per_square] = 0
    return image


def _pdf_text(value: str) -> str:
    return value.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def _page_content(board: BoardSpec, page_w: float, page_h: float) -> str:
    """PDF drawing operators for one page, in millimetres scaled to points."""
    square = board.square_size
    board_w = (board.columns + 1) * square
    board_h = (board.rows + 1) * square
    left = (page_w - board_w) / 2
    top = (page_h - board_h) / 2 + board_h  # PDF y runs upward

    def pt(mm: float) -> str:
        return f"{mm * MM_TO_PT:.3f}"

    ops = ["0 g"]
    for column, row in black_squares(board):
        x = left + column * square
        y = top - (row + 1) * square
        ops.append(f"{pt(x)} {pt(y)} {pt(square)} {pt(square)} re")
    ops.append("f")

    title = (
        f"Ster-Vis calibration target - {board.columns} x {board.rows} inner corners, "
        f"{square:g} mm squares"
    )
    note = "Print at 100% / Actual size. Measure a square; pass the real size to --square-size."
    text_y = top + (page_h - top) / 2 - 1.5
    ops.append(f"BT /F1 9 Tf {pt(left)} {pt(text_y)} Td ({_pdf_text(title)}) Tj ET")

    # A 100 mm scale bar below the board, to check the printer did not rescale.
    bar_y = (top - board_h) / 2
    bar_left = left
    ops.append("0.4 w 0 G")
    ops.append(f"{pt(bar_left)} {pt(bar_y)} m {pt(bar_left + 100)} {pt(bar_y)} l S")
    for tick in range(0, 101, 10):
        height = 3.0 if tick % 50 == 0 else 1.5
        x = bar_left + tick
        ops.append(f"{pt(x)} {pt(bar_y)} m {pt(x)} {pt(bar_y + height)} l S")
    ops.append(
        f"BT /F1 7 Tf {pt(bar_left + 104)} {pt(bar_y - 0.8)} Td "
        f"({_pdf_text('100 mm - check with a ruler')}) Tj ET"
    )
    ops.append(f"BT /F1 7 Tf {pt(bar_left + 150)} {pt(bar_y - 0.8)} Td ({_pdf_text(note)}) Tj ET")
    return "\n".join(ops) + "\n"


def write_chessboard_pdf(
    path: str | Path, board: BoardSpec | None = None, paper: str = "a4"
) -> Path:
    """Write a one-page, true-scale PDF of the board.

    Squares are vector rectangles at exact physical size, so the print is only
    as accurate as the printer's scaling setting. Raises ValueError if the
    board does not fit on the page with a printable margin.
    """
    board = board or BoardSpec()
    if paper not in PAPER_SIZES_MM:
        raise ValueError(f"paper must be one of {sorted(PAPER_SIZES_MM)}")
    page_w, page_h = PAPER_SIZES_MM[paper]

    board_w = (board.columns + 1) * board.square_size
    board_h = (board.rows + 1) * board.square_size
    margin_x = (page_w - board_w) / 2
    margin_y = (page_h - board_h) / 2
    if min(margin_x, margin_y) < MIN_MARGIN_MM:
        raise ValueError(
            f"a {board_w:g} x {board_h:g} mm board leaves less than {MIN_MARGIN_MM:g} mm "
            f"of margin on {paper.upper()}; use a smaller --square-mm"
        )

    content = _page_content(board, page_w, page_h).encode("ascii")
    objects = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        (
            f"<< /Type /Page /Parent 2 0 R "
            f"/MediaBox [0 0 {page_w * MM_TO_PT:.3f} {page_h * MM_TO_PT:.3f}] "
            f"/Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R >>"
        ).encode("ascii"),
        b"<< /Length %d >>\nstream\n" % len(content) + content + b"endstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
    ]

    output = bytearray(b"%PDF-1.4\n")
    offsets = []
    for number, body in enumerate(objects, start=1):
        offsets.append(len(output))
        output += b"%d 0 obj\n" % number + body + b"\nendobj\n"

    xref_at = len(output)
    output += b"xref\n0 %d\n" % (len(objects) + 1)
    output += b"0000000000 65535 f \n"
    for offset in offsets:
        output += b"%010d 00000 n \n" % offset
    output += b"trailer\n<< /Size %d /Root 1 0 R >>\n" % (len(objects) + 1)
    output += b"startxref\n%d\n%%%%EOF\n" % xref_at

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(bytes(output))
    return path
