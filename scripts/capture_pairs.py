"""Capture calibration image pairs from two cameras.

Shows a live side-by-side preview. Press SPACE to save a pair, Q or ESC to quit.
Aim for 15-20 pairs with the board tilted and at different distances, filling
the frame corners as well as the centre.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stereo_vision.calibration import find_corners  # noqa: E402
from stereo_vision.capture import read_pair, save_pair, stereo_cameras  # noqa: E402
from stereo_vision.config import BoardSpec  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-index", type=int, default=0)
    parser.add_argument("--right-index", type=int, default=1)
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--output", type=Path, default=Path("data/pairs"))
    parser.add_argument("--columns", type=int, default=9, help="inner corners across")
    parser.add_argument("--rows", type=int, default=6, help="inner corners down")
    parser.add_argument(
        "--no-detect",
        action="store_true",
        help="skip the live board overlay, which is slow on large frames",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    board = BoardSpec(columns=args.columns, rows=args.rows)

    existing = sorted((args.output / "left").glob("*.png"))
    index = len(existing)
    if index:
        print(f"found {index} existing pairs, continuing from {index:03d}")

    with stereo_cameras(
        args.left_index, args.right_index, args.width, args.height
    ) as (left_camera, right_camera):
        print("SPACE saves a pair, Q or ESC quits")
        while True:
            left, right = read_pair(left_camera, right_camera)

            preview_left, preview_right = left.copy(), right.copy()
            both_found = True
            if not args.no_detect:
                for preview in (preview_left, preview_right):
                    corners = find_corners(preview, board, refine=False)
                    if corners is None:
                        both_found = False
                    else:
                        cv2.drawChessboardCorners(
                            preview, board.pattern_size, corners, True
                        )

            side_by_side = cv2.hconcat([preview_left, preview_right])
            color = (0, 200, 0) if both_found else (0, 0, 220)
            cv2.putText(
                side_by_side,
                f"saved: {index}",
                (12, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                color,
                2,
            )
            cv2.imshow("stereo capture (left | right)", side_by_side)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                left_path, _ = save_pair(args.output, index, left, right)
                print(f"saved pair {index:03d} -> {left_path.parent.parent}")
                index += 1

    cv2.destroyAllWindows()
    print(f"{index} pairs in {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
