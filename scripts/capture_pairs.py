"""Capture calibration image pairs from two cameras.

Shows a live side-by-side preview. Press SPACE to save a pair, Q or ESC to quit.

A grid over each view turns green where saved pairs have already put board
corners. Keep going until nearly all of it is green in both views: corners and
edges matter most, because the lens model is only trustworthy where the board
has been. Aim for about 25 pairs, tilted and at several distances.

On a Raspberry Pi without a display, use --auto --headless --stream 8080 and
watch from a browser: a pair is saved by itself whenever the board is held
still somewhere new, and capture stops once coverage and count are reached.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stereo_vision.autocapture import AutoTrigger, quick_corners  # noqa: E402
from stereo_vision.calibration import (  # noqa: E402
    COVERAGE_GRID,
    COVERAGE_WARN_PCT,
    frame_coverage,
)
from stereo_vision.capture import load_pairs, save_pair  # noqa: E402
from stereo_vision.config import BoardSpec  # noqa: E402
from stereo_vision.sources import open_source  # noqa: E402

TARGET_COVERAGE_PCT = 75.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--backend", choices=["auto", "opencv", "picamera2"], default="auto",
                        help="auto uses Pi camera modules when two are attached, else USB")
    parser.add_argument("--left-index", type=int, default=0, help="left camera number")
    parser.add_argument("--right-index", type=int, default=1, help="right camera number")
    parser.add_argument("--width", type=int, default=None, help="capture width; calibrate at the size you will run at")
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default=None, help="USB camera format, e.g. MJPG")
    parser.add_argument("--focus", type=float, default=1.0, metavar="DIOPTRES",
                        help="Pi cameras with autofocus: fixed focus, 1/metres (default 1.0 = 1 m); "
                             "measure with the same value later")
    parser.add_argument("--output", type=Path, default=Path("data/pairs"))
    parser.add_argument("--columns", type=int, default=9, help="inner corners across")
    parser.add_argument("--rows", type=int, default=6, help="inner corners down")
    parser.add_argument("--no-detect", action="store_true",
                        help="skip the live board overlay (not with --auto)")
    parser.add_argument("--auto", action="store_true",
                        help="save pairs automatically when the board is held still somewhere new")
    parser.add_argument("--target", type=int, default=25,
                        help=f"with --auto, stop after this many pairs once coverage reaches {TARGET_COVERAGE_PCT:.0f}%%")
    parser.add_argument("--headless", action="store_true", help="no window (needs --auto)")
    parser.add_argument("--stream", type=int, default=None, metavar="PORT",
                        help="serve the preview to a browser on this port")
    args = parser.parse_args()
    if args.headless and not args.auto:
        parser.error("--headless needs --auto: without a window there is no SPACE key to press")
    if args.auto and args.no_detect:
        parser.error("--auto needs board detection")
    return args


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


def to_bgr(image: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy()


def main() -> int:
    args = parse_args()
    board = BoardSpec(columns=args.columns, rows=args.rows)

    existing = sorted((args.output / "left").glob("*.png"))
    index = len(existing)
    corner_sets: dict[str, list[np.ndarray]] = {"left": [], "right": []}
    previous = []
    if index:
        print(f"found {index} existing pairs, continuing from {index:03d}")
        lefts, rights, _ = load_pairs(args.output)
        for left, right in zip(lefts, rights):
            found = (quick_corners(left, board), quick_corners(right, board))
            for side, corners in zip(("left", "right"), found):
                if corners is not None:
                    corner_sets[side].append(corners)
            if None not in found:
                previous.append(found)

    server = None
    if args.stream is not None:
        from stereo_vision.stream import MjpegServer

        server = MjpegServer(args.stream, max_fps=8, title="calibration capture")
        print("preview at " + "  ".join(server.urls()))
        print("note: anyone on this network can open that address")

    source = open_source(args.backend, args.left_index, args.right_index, args.width, args.height,
                         args.fps, grey=True, threaded=True, fourcc=args.fourcc,
                         focus_dioptres=args.focus)
    image_size = source.size
    trigger = AutoTrigger(image_size, COVERAGE_GRID) if args.auto else None
    if trigger is not None:
        for left, right in previous:
            trigger.remember(left, right)

    print(f"cameras at {image_size[0]}x{image_size[1]}")
    if args.auto:
        print(f"auto capture: hold the board still in a new spot; stops at {args.target} pairs "
              f"and {TARGET_COVERAGE_PCT:.0f}% coverage" + ("" if args.headless else "; Q quits"))
    else:
        print("SPACE saves a pair, Q or ESC quits")

    message, last_print = "", 0.0
    try:
        with source:
            while True:
                frame = source.read()
                left, right = frame.left, frame.right

                coverage = {}
                previews = {}
                for side, image in (("left", left), ("right", right)):
                    coverage[side], covered = frame_coverage(corner_sets[side], image_size, COVERAGE_GRID)
                    previews[side] = to_bgr(image)
                    draw_coverage(previews[side], covered)

                live = {"left": None, "right": None}
                if not args.no_detect:
                    for side, image in (("left", left), ("right", right)):
                        live[side] = quick_corners(image, board)
                        if live[side] is not None:
                            cv2.drawChessboardCorners(previews[side], board.pattern_size,
                                                      live[side].astype(np.float32), True)
                both_found = live["left"] is not None and live["right"] is not None

                save = False
                if trigger is not None:
                    save, message = trigger.update(live["left"], live["right"])

                worst = min(coverage.values())
                view = cv2.hconcat([previews["left"], previews["right"]])
                color = (0, 200, 0) if both_found else (0, 0, 220)
                status = f"saved: {index}   coverage: {worst:.0f}%" + (f"   {message}" if message else "")
                cv2.putText(view, status, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

                if server is not None and server.wants_frame():
                    server.publish(view)
                key = -1
                if not args.headless:
                    cv2.imshow("stereo capture (left | right)", view)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                if key == ord(" ") and not args.auto:
                    save = True

                if save:
                    left_path, _ = save_pair(args.output, index, left, right)
                    for side, image in (("left", left), ("right", right)):
                        corners = live[side] if live[side] is not None else quick_corners(image, board)
                        if corners is not None:
                            corner_sets[side].append(corners)
                    print(f"saved pair {index:03d} -> {left_path.parent.parent}  ({message or 'manual'})")
                    index += 1

                now = time.monotonic()
                if args.headless and now - last_print > 3.0:
                    print(f"  {status}", flush=True)
                    last_print = now
                if args.auto and index >= args.target and worst >= TARGET_COVERAGE_PCT:
                    print(f"reached {index} pairs and {worst:.0f}% coverage")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if not args.headless:
            cv2.destroyAllWindows()
        if server is not None:
            server.close()

    print(f"{index} pairs in {args.output}")
    worst = min(frame_coverage(corner_sets[side], image_size)[0] for side in ("left", "right"))
    print(f"frame coverage: {worst:.0f}%")
    if worst < COVERAGE_WARN_PCT:
        print("coverage is low: capture more pairs near the corners and edges")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
