"""Write a printable chessboard calibration target as a true-scale PDF.

The defaults match the defaults in capture_pairs.py and calibrate.py, so the
printed board works with no extra flags.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stereo_vision.config import BoardSpec  # noqa: E402
from stereo_vision.pattern import PAPER_SIZES_MM, write_chessboard_pdf  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--columns", type=int, default=9, help="inner corners across")
    parser.add_argument("--rows", type=int, default=6, help="inner corners down")
    parser.add_argument("--square-mm", type=float, default=25.0)
    parser.add_argument("--paper", choices=sorted(PAPER_SIZES_MM), default="a4")
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    board = BoardSpec(columns=args.columns, rows=args.rows, square_size=args.square_mm)
    output = args.output or Path(
        f"docs/chessboard_{args.columns}x{args.rows}_{args.square_mm:g}mm_{args.paper}.pdf"
    )
    try:
        path = write_chessboard_pdf(output, board, args.paper)
    except ValueError as error:
        print(error)
        return 1
    print(f"wrote {path}")
    print(
        f"use with: --columns {args.columns} --rows {args.rows} "
        f"--square-size {args.square_mm:g}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
