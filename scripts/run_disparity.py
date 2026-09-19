"""Rectify a stereo pair, compute disparity, and optionally export a point cloud.

Works on two image files, or on a live camera pair with --live.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stereo_vision.calibration import load_calibration, rectify_pair  # noqa: E402
from stereo_vision.capture import read_pair, stereo_cameras  # noqa: E402
from stereo_vision.config import SGBMParams  # noqa: E402
from stereo_vision.depth import (  # noqa: E402
    colorize_depth,
    disparity_to_depth,
    point_cloud,
    save_depth,
    write_ply,
)
from stereo_vision.disparity import (  # noqa: E402
    build_matcher,
    colorize,
    compute_disparity,
    compute_disparity_wls,
    valid_mask,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, default=Path("calib/stereo.npz"))
    parser.add_argument("--left", type=Path, default=None, help="left image file")
    parser.add_argument("--right", type=Path, default=None, help="right image file")
    parser.add_argument("--live", action="store_true", help="read from two cameras")
    parser.add_argument("--left-index", type=int, default=0)
    parser.add_argument("--right-index", type=int, default=1)
    parser.add_argument("--num-disparities", type=int, default=128)
    parser.add_argument("--block-size", type=int, default=5)
    parser.add_argument("--wls", action="store_true", help="apply the WLS filter")
    parser.add_argument("--save", type=Path, default=None, help="write the color disparity image")
    parser.add_argument(
        "--depth-out",
        type=Path,
        default=None,
        help="write depth in mm: .png for 16-bit PNG, .npy for float32 "
        "(open with scripts/view_depth.py)",
    )
    parser.add_argument("--ply", type=Path, default=None, help="write a point cloud")
    parser.add_argument(
        "--max-depth",
        type=float,
        default=None,
        help="drop points beyond this depth from the point cloud",
    )
    return parser.parse_args()


def report(disparity: np.ndarray, calibration, params: SGBMParams) -> None:
    mask = valid_mask(disparity, params)
    coverage = 100.0 * mask.mean()
    print(f"matched {coverage:.1f}% of pixels")
    if not mask.any():
        print("nothing matched: check rectification and --num-disparities")
        return
    depth = disparity_to_depth(
        disparity, calibration.focal_length_px, calibration.baseline
    )
    finite = depth[np.isfinite(depth) & mask]
    if finite.size:
        print(
            f"depth range: {finite.min():.1f} to {finite.max():.1f} "
            f"(median {np.median(finite):.1f})"
        )


def main() -> int:
    args = parse_args()

    if not args.calibration.exists():
        print(f"no calibration at {args.calibration}; run scripts/calibrate.py first")
        return 1
    calibration = load_calibration(args.calibration)
    maps = calibration.rectification_maps()

    params = SGBMParams(
        num_disparities=args.num_disparities, block_size=args.block_size
    )

    if args.live:
        matcher = build_matcher(params)
        with stereo_cameras(args.left_index, args.right_index) as (left_cam, right_cam):
            print("Q or ESC quits")
            while True:
                left_raw, right_raw = read_pair(left_cam, right_cam)
                left, right = rectify_pair(left_raw, right_raw, maps)
                disparity = compute_disparity(left, right, params, matcher)
                cv2.imshow("disparity", colorize(disparity, params))
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
        cv2.destroyAllWindows()
        return 0

    if args.left is None or args.right is None:
        print("pass --left and --right, or use --live")
        return 2

    left_raw = cv2.imread(str(args.left), cv2.IMREAD_COLOR)
    right_raw = cv2.imread(str(args.right), cv2.IMREAD_COLOR)
    if left_raw is None or right_raw is None:
        print("could not read one of the input images")
        return 1

    left, right = rectify_pair(left_raw, right_raw, maps)
    if args.wls:
        disparity = compute_disparity_wls(left, right, params)
    else:
        disparity = compute_disparity(left, right, params)

    report(disparity, calibration, params)

    colored = colorize(disparity, params)
    if args.depth_out:
        depth = disparity_to_depth(
            np.where(valid_mask(disparity, params), disparity, 0),
            calibration.focal_length_px,
            calibration.baseline,
        )
        save_depth(args.depth_out, depth)
        print(f"wrote {args.depth_out}")
        preview = args.depth_out.with_name(args.depth_out.stem + "_color.png")
        cv2.imwrite(str(preview), colorize_depth(depth))
        print(f"wrote {preview}")

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.save), colored)
        print(f"wrote {args.save}")

    if args.ply:
        points, colors = point_cloud(
            disparity,
            calibration.Q,
            left,
            min_disparity=float(params.min_disparity),
            max_depth=args.max_depth,
        )
        write_ply(args.ply, points, colors)
        print(f"wrote {args.ply} with {len(points)} points")

    if not args.save and not args.ply and not args.depth_out:
        cv2.imshow("disparity", colored)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
