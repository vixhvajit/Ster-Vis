"""Capture calibration image pairs from two cameras.

Shows a live side-by-side preview. Press SPACE to save a pair, Q or ESC to quit.

A grid over each view turns green where saved pairs have already put board
corners. Keep going until nearly all of it is green in both views: corners and
edges matter most, because the lens model is only trustworthy where the board
has been. Aim for about 25 pairs, tilted and at several distances.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stereo_vision.calibration import (  # noqa: E402
    COVERAGE_GRID,
    COVERAGE_WARN_PCT,
    find_corners,
    frame_coverage,
)
from stereo_vision.capture import load_pairs, read_pair, save_pair, stereo_cameras  # noqa: E402
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


def draw_coverage(image: np.ndarray, covered: np.ndarray) -> None:
    """Tint covered grid cells green and outline the rest, in place."""
    rows, columns = covered.shape
    height, width = image.shape[:2]
    tint = image.copy()
    for row in range(rows):
        for column in range(columns):
            x0, x1 = column * width // columns, (column + 1) * width // columns
            y0, y1 = row * height // rows, (row + 1) * height // rows
            if covered[row, column]:
                cv2.rectangle(tint, (x0, y0), (x1, y1), (0, 180, 0), -1)
            cv2.rectangle(image, (x0, y0), (x1, y1), (160, 160, 160), 1)
    cv2.addWeighted(tint, 0.25, image, 0.75, 0, dst=image)


def main() -> int:
    args = parse_args()
    board = BoardSpec(columns=args.columns, rows=args.rows)

    existing = sorted((args.output / "left").glob("*.png"))
    index = len(existing)
    corner_sets: dict[str, list[np.ndarray]] = {"left": [], "right": []}
    if index:
        print(f"found {index} existing pairs, continuing from {index:03d}")
        for side, images in zip(("left", "right"), load_pairs(args.output)[:2]):
            for image in images:
                corners = find_corners(image, board, refine=False)
                if corners is not None:
                    corner_sets[side].append(corners)
    image_size = None

    with stereo_cameras(
        args.left_index, args.right_index, args.width, args.height
    ) as (left_camera, right_camera):
        print("SPACE saves a pair, Q or ESC quits")
        while True:
            left, right = read_pair(left_camera, right_camera)
            image_size = (left.shape[1], left.shape[0])

            preview_left, preview_right = left.copy(), right.copy()
            coverage = {}
            for side, preview in (("left", preview_left), ("right", preview_right)):
                coverage[side], covered = frame_coverage(
                    corner_sets[side], image_size, COVERAGE_GRID
                )
                draw_coverage(preview, covered)

            both_found = True
            live = {"left": None, "right": None}
            if not args.no_detect:
                for side, frame, preview in (
                    ("left", left, preview_left),
                    ("right", right, preview_right),
                ):
                    live[side] = find_corners(frame, board, refine=False)
                    if live[side] is None:
                        both_found = False
                    else:
                        cv2.drawChessboardCorners(
                            preview, board.pattern_size, live[side], True
                        )

            side_by_side = cv2.hconcat([preview_left, preview_right])
            color = (0, 200, 0) if both_found else (0, 0, 220)
            worst = min(coverage.values())
            cv2.putText(
                side_by_side,
                f"saved: {index}   coverage: {worst:.0f}%",
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
                for side, frame in (("left", left), ("right", right)):
                    corners = live[side] if live[side] is not None else find_corners(frame, board, refine=False)
                    if corners is not None:
                        corner_sets[side].append(corners)
                print(f"saved pair {index:03d} -> {left_path.parent.parent}")
                index += 1

    cv2.destroyAllWindows()
    print(f"{index} pairs in {args.output}")
    if image_size is not None:
        worst = min(
            frame_coverage(corner_sets[side], image_size)[0] for side in ("left", "right")
        )
        print(f"frame coverage: {worst:.0f}%")
        if worst < COVERAGE_WARN_PCT:
            print("coverage is low: capture more pairs near the corners and edges")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
