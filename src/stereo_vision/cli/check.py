"""End-to-end calibration check against a virtual rig with known geometry.

Renders chessboard pairs through a simulated 60 mm stereo rig, writes them to
disk, runs the real calibrate.py on them exactly as you would on camera
captures, then compares what it recovered against the true rig. Finally it
checks rectification and depth on fresh pairs the calibration never saw.

Exits non-zero if any metric is out of tolerance, so it can gate CI.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import shutil
from pathlib import Path


from stereo_vision.calibration import load_calibration
from stereo_vision.cli import calibrate
from stereo_vision.capture import save_pair
from stereo_vision.config import BoardSpec
from stereo_vision.synthetic import TOLERANCES, default_rig, evaluate, render_pairs

LABELS = {
    "rms_px": "reprojection RMS",
    "left_focal_error_pct": "left focal length error",
    "right_focal_error_pct": "right focal length error",
    "left_principal_point_error_px": "left principal point error",
    "right_principal_point_error_px": "right principal point error",
    "baseline_error_pct": "baseline error",
    "rotation_error_deg": "rig rotation error",
    "rectified_row_error_mean_px": "rectified row mismatch (mean)",
    "depth_error_mean_pct": "depth error on unseen pairs (mean)",
    "depth_error_max_pct": "depth error on unseen pairs (max)",
}


def unit(key: str) -> str:
    if key.endswith("_pct"):
        return "%"
    if key.endswith("_deg"):
        return "deg"
    return "px"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ster-vis check", description=__doc__)
    parser.add_argument("--pairs", type=int, default=25, help="25 or more; fewer can fail the checks")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--workdir", type=Path, default=Path("output/synthetic"))
    parser.add_argument(
        "--keep", action="store_true", help="keep the rendered pairs for inspection"
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    board = BoardSpec()
    rig = default_rig()

    if args.workdir.exists():
        shutil.rmtree(args.workdir)
    pairs_dir = args.workdir / "pairs"
    calib_path = args.workdir / "stereo.npz"

    print(f"rendering {args.pairs} pairs through a virtual {rig.baseline:.1f} mm rig ...")
    lefts, rights, _ = render_pairs(rig, board, count=args.pairs, seed=args.seed)
    for index, (left, right) in enumerate(zip(lefts, rights)):
        save_pair(pairs_dir, index, left, right)

    print("running ster-vis calibrate on them ...\n")
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured):
        code = calibrate.main([
            "--pairs", str(pairs_dir),
            "--output", str(calib_path),
            "--square-size", str(board.square_size),
        ])
    print("\n".join("  | " + line for line in captured.getvalue().strip().splitlines()))
    if code != 0:
        print("\ncalibration failed")
        return 1

    metrics = evaluate(load_calibration(calib_path), rig, board)

    print(
        f"\n  true baseline {metrics['baseline_true_mm']:.3f} mm, "
        f"recovered {metrics['baseline_estimated_mm']:.3f} mm\n"
    )
    print(f"  {'check':<38}{'value':>10}  {'limit':>8}  result")
    print(f"  {'-' * 38}{'-' * 10}  {'-' * 8}  ------")
    failures = 0
    for key, limit in TOLERANCES.items():
        value = metrics[key]
        passed = value <= limit
        failures += not passed
        print(
            f"  {LABELS[key]:<38}{value:>8.4f} {unit(key):<3}"
            f"{limit:>6g} {unit(key):<3} {'PASS' if passed else 'FAIL'}"
        )

    if not args.keep:
        shutil.rmtree(args.workdir)

    print(f"\n{len(TOLERANCES) - failures}/{len(TOLERANCES)} checks passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
