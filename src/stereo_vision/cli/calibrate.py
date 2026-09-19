"""Calibrate a stereo rig from captured board pairs and save the result.

Reads data/pairs/{left,right}, solves both cameras and the rig, and writes an
.npz holding the intrinsics, extrinsics and rectification matrices.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


from stereo_vision.calibration import (
    COVERAGE_WARN_PCT,
    calibrate_stereo,
    rectify_pair,
)
from stereo_vision.capture import load_pairs
from stereo_vision.config import BoardSpec


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ster-vis calibrate", description=__doc__)
    parser.add_argument("--pairs", type=Path, default=Path("data/pairs"))
    parser.add_argument("--output", type=Path, default=Path("calib/stereo.npz"))
    parser.add_argument("--columns", type=int, default=9, help="inner corners across")
    parser.add_argument("--rows", type=int, default=6, help="inner corners down")
    parser.add_argument(
        "--square-size",
        type=float,
        default=25.0,
        help="square edge length; sets the unit of baseline and depth (default mm)",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.0,
        help="rectified field of view: 0 crops to valid pixels, 1 keeps all",
    )
    parser.add_argument(
        "--preview",
        type=Path,
        default=None,
        help="write a rectified pair with epipolar lines drawn, to eyeball the result",
    )
    return parser.parse_args(argv)


def write_preview(path: Path, left, right, step: int = 40) -> None:
    """Save a rectified pair with horizontal rules across it.

    After a good calibration the same scene feature sits on the same rule in
    both halves; if it does not, the rig moved or the board coverage was poor.
    """
    side_by_side = cv2.hconcat([left, right])
    for y in range(0, side_by_side.shape[0], step):
        cv2.line(side_by_side, (0, y), (side_by_side.shape[1], y), (0, 220, 0), 1)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), side_by_side)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    board = BoardSpec(
        columns=args.columns, rows=args.rows, square_size=args.square_size
    )

    left_images, right_images, names = load_pairs(args.pairs)
    if not left_images:
        print(f"no pairs found in {args.pairs}; run ster-vis capture first")
        return 1
    print(f"loaded {len(left_images)} pairs from {args.pairs}")

    calibration, used = calibrate_stereo(left_images, right_images, board, args.alpha)

    skipped = [names[i] for i in range(len(names)) if i not in set(used)]
    if skipped:
        print(f"skipped {len(skipped)} pairs without a full board: {', '.join(skipped)}")

    print(f"used {len(used)} pairs")
    print(f"reprojection RMS: {calibration.rms:.4f} px")
    print(f"baseline: {calibration.baseline:.2f} (same unit as --square-size)")
    print(f"rectified focal length: {calibration.focal_length_px:.2f} px")
    print(f"frame coverage: {calibration.coverage_pct:.0f}% (aim for 75% or more)")
    if calibration.rms > 1.0:
        print("RMS above 1.0 px: recapture with sharper, better-spread board views")
    if calibration.coverage_pct < COVERAGE_WARN_PCT:
        print(
            f"WARNING: the boards reached only {calibration.coverage_pct:.0f}% of the frame.\n"
            "  A low RMS does not mean a good calibration: it only measures the fit\n"
            "  where the boards were. Depth near the uncovered edges can be badly\n"
            "  wrong. Recapture with the board in the corners and along the edges;\n"
            "  capture_pairs.py shows a live coverage grid to help."
        )

    saved = calibration.save(args.output)
    print(f"wrote {saved}")

    if args.preview:
        maps = calibration.rectification_maps()
        left, right = rectify_pair(left_images[used[0]], right_images[used[0]], maps)
        write_preview(args.preview, left, right)
        print(f"wrote {args.preview}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
