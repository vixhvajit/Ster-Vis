"""Produce a real depth map from a synthetic 3D scene and score it against truth.

The full pipeline runs exactly as it would on hardware:

  1. calibrate the virtual rig from rendered chessboard pairs
  2. render a textured room scene through the same rig, with lens distortion
  3. rectify with the recovered calibration, match with SGBM, triangulate
  4. compare every pixel against the ray-traced true depth

Nothing downstream is handed the true geometry, so calibration error, matching
error and triangulation error all show up in the score.

Outputs go to output/scene/ (see --output). --samples also refreshes the small
committed copies in docs/samples/ that the README links to.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


from stereo_vision.calibration import calibrate_stereo, rectify_pair
from stereo_vision.config import BoardSpec, SGBMParams
from stereo_vision.depth import (
    colorize_depth,
    disparity_to_depth,
    point_cloud,
    save_depth,
    write_ply,
)
from stereo_vision.disparity import (
    colorize,
    compute_disparity,
    compute_disparity_wls,
    valid_mask,
)
from stereo_vision.scene import default_scene, render_view, true_rectified_depth
from stereo_vision.synthetic import default_rig, render_pairs

NEAR_MM, FAR_MM = 600.0, 2400.0
ERROR_SCALE_PCT = 5.0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ster-vis scene", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, default=Path("output/scene"))
    parser.add_argument("--supersample", type=int, default=2, help="rays per pixel per axis")
    parser.add_argument("--num-disparities", type=int, default=96)
    parser.add_argument("--block-size", type=int, default=5)
    parser.add_argument("--wls", action="store_true", help="also score the WLS-filtered map")
    parser.add_argument(
        "--samples", action="store_true", help="refresh the committed samples in docs/samples/"
    )
    return parser.parse_args(argv)


def score(estimate: np.ndarray, truth: np.ndarray, ids: np.ndarray, visible: np.ndarray, names: list[str]) -> dict:
    """Accuracy and coverage of an estimated depth map against the truth."""
    matched = np.isfinite(estimate) & (estimate > 0)
    scored = matched & visible
    rel = np.abs(estimate - truth) / truth * 100.0
    errors = rel[scored]

    result = {
        "coverage_pct": float(100.0 * scored.sum() / max(visible.sum(), 1)),
        "median_error_pct": float(np.median(errors)),
        "mean_error_pct": float(errors.mean()),
        "p95_error_pct": float(np.percentile(errors, 95)),
        "within_1pct": float(100.0 * (errors <= 1.0).mean()),
        "within_5pct": float(100.0 * (errors <= 5.0).mean()),
        # Depth reported where the right camera cannot see: a wrong match,
        # since no correct one exists.
        "false_matches_in_occlusion_pct": float(
            100.0 * (matched & ~visible & np.isfinite(truth)).sum() / max((~visible & np.isfinite(truth)).sum(), 1)
        ),
        "objects": {},
    }
    for number, name in enumerate(names, start=1):
        on_object = ids == number
        seen = on_object & visible
        if seen.sum() < 50:
            continue
        mine = on_object & scored
        result["objects"][name] = {
            "coverage_pct": float(100.0 * mine.sum() / seen.sum()),
            "median_error_pct": float(np.median(rel[mine])) if mine.any() else float("nan"),
            "true_depth_mm": float(np.median(truth[seen])),
        }
    return result


def error_map(estimate: np.ndarray, truth: np.ndarray, visible: np.ndarray) -> np.ndarray:
    """Relative error heatmap: dark = accurate, bright = wrong.

    Grey marks pixels the right camera cannot see, where no stereo method can
    work. Black marks visible pixels the matcher left empty.
    """
    rel = np.abs(estimate - truth) / truth * 100.0
    matched = np.isfinite(estimate) & (estimate > 0)
    scaled = np.clip(rel / ERROR_SCALE_PCT * 255.0, 0, 255)
    image = cv2.applyColorMap(np.nan_to_num(scaled).astype(np.uint8), cv2.COLORMAP_INFERNO)
    image[visible & ~matched] = 0
    image[~visible] = (90, 90, 90)
    return image


def label(image: np.ndarray, text: str) -> np.ndarray:
    out = image.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(out, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def legend(width: int, height: int, colormap: int, left: str, right: str, title: str) -> np.ndarray:
    ramp = np.tile(np.linspace(0, 255, width - 40).astype(np.uint8), (18, 1))
    bar = cv2.applyColorMap(ramp, colormap)
    panel = np.full((height, width, 3), 24, np.uint8)
    panel[34:52, 20 : width - 20] = bar
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(panel, title, (20, 24), font, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(panel, left, (20, 74), font, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
    size = cv2.getTextSize(right, font, 0.55, 1)[0]
    cv2.putText(panel, right, (width - 20 - size[0], 74), font, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
    return panel


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    board = BoardSpec()
    rig = default_rig()
    scene = default_scene()
    names = [p.name for p in scene.primitives]
    started = time.perf_counter()

    print("1/4 calibrating the virtual rig from 25 chessboard pairs ...")
    lefts, rights, _ = render_pairs(rig, board, count=25, seed=0)
    calibration, _ = calibrate_stereo(lefts, rights, board)
    print(f"    RMS {calibration.rms:.3f} px, baseline {calibration.baseline:.2f} mm (true {rig.baseline:.2f})")

    print(f"2/4 ray tracing the scene ({args.supersample}x{args.supersample} rays per pixel) ...")
    left_raw = render_view(scene, rig, "left", args.supersample, np.random.default_rng(10))
    right_raw = render_view(scene, rig, "right", args.supersample, np.random.default_rng(11))

    print("3/4 rectifying, matching and triangulating ...")
    maps = calibration.rectification_maps()
    left, right = rectify_pair(left_raw, right_raw, maps)
    params = SGBMParams(num_disparities=args.num_disparities, block_size=args.block_size)
    t0 = time.perf_counter()
    disparity = compute_disparity(left, right, params)
    match_ms = (time.perf_counter() - t0) * 1000
    disparity[~valid_mask(disparity, params)] = 0
    depth = disparity_to_depth(disparity, calibration.focal_length_px, calibration.baseline)
    depth[~np.isfinite(depth)] = np.nan

    print("4/4 scoring against the ray-traced truth ...")
    truth, ids, visible = true_rectified_depth(scene, rig, calibration)
    results = {"sgbm": score(depth, truth, ids, visible, names)}
    results["sgbm"]["match_time_ms"] = match_ms

    if args.wls:
        wls_disp = compute_disparity_wls(left, right, params)
        wls_disp[~valid_mask(wls_disp, params)] = 0
        wls_depth = disparity_to_depth(wls_disp, calibration.focal_length_px, calibration.baseline)
        wls_depth[~np.isfinite(wls_depth)] = np.nan
        results["sgbm_wls"] = score(wls_depth, truth, ids, visible, names)

    median_depth = float(np.nanmedian(truth[visible]))
    results["theory"] = {
        "median_true_depth_mm": median_depth,
        "expected_error_at_median_pct_quarter_px": 100.0 * median_depth * 0.25
        / (calibration.focal_length_px * calibration.baseline),
    }
    results["calibration"] = {
        "rms_px": calibration.rms,
        "baseline_mm": calibration.baseline,
        "true_baseline_mm": rig.baseline,
    }

    # Files a person, or a viewer, can open.
    calibration.save(out / "stereo.npz")
    cv2.imwrite(str(out / "left_raw.png"), left_raw)
    cv2.imwrite(str(out / "right_raw.png"), right_raw)
    rectified = cv2.hconcat([left, right])
    for y in range(0, rectified.shape[0], 40):
        cv2.line(rectified, (0, y), (rectified.shape[1], y), (0, 200, 0), 1)
    cv2.imwrite(str(out / "rectified_pair.png"), rectified)
    cv2.imwrite(str(out / "disparity.png"), colorize(disparity, params))
    save_depth(out / "depth_mm.png", depth)
    save_depth(out / "depth_truth_mm.png", truth)
    depth_color = colorize_depth(depth, NEAR_MM, FAR_MM)
    truth_color = colorize_depth(truth, NEAR_MM, FAR_MM)
    errors = error_map(depth, truth, visible)
    cv2.imwrite(str(out / "depth_color.png"), depth_color)
    cv2.imwrite(str(out / "depth_truth_color.png"), truth_color)
    cv2.imwrite(str(out / "depth_error.png"), errors)

    points, colors = point_cloud(disparity, calibration.Q, left, min_disparity=0.5, max_depth=FAR_MM + 400)
    write_ply(out / "cloud.ply", points, colors)

    width = left.shape[1]
    top = cv2.hconcat([label(left, "rectified left camera"), label(truth_color, "true depth (ray traced)")])
    middle = cv2.hconcat([label(depth_color, "measured depth (SGBM)"), label(errors, "error vs truth")])
    legends = cv2.hconcat([
        legend(width, 90, cv2.COLORMAP_TURBO, f"{FAR_MM / 1000:.1f} m (far)", f"{NEAR_MM / 1000:.1f} m (near)", "depth"),
        legend(width, 90, cv2.COLORMAP_INFERNO, "0%", f"{ERROR_SCALE_PCT:g}%+   grey = hidden from right camera", "relative error"),
    ])
    summary = cv2.vconcat([top, middle, legends])
    cv2.imwrite(str(out / "summary.png"), summary)
    (out / "metrics.json").write_text(json.dumps(results, indent=2), encoding="utf-8")

    if args.samples:
        samples = Path("docs/samples")
        samples.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(samples / "scene_summary.png"), cv2.resize(summary, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
        save_depth(samples / "scene_depth_mm.png", depth)
        step = max(1, len(points) // 60000)
        write_ply(samples / "scene_cloud.ply", points[::step], colors[::step])
        print(f"    refreshed {samples}")

    s = results["sgbm"]
    print(f"\n  calibration: baseline {calibration.baseline:.2f} mm (true {rig.baseline:.2f}), RMS {calibration.rms:.3f} px")
    print(f"  SGBM on {width}x{left.shape[0]}: {match_ms:.0f} ms")
    print(f"  coverage of pixels both cameras see: {s['coverage_pct']:.1f}%")
    print(f"  depth error: median {s['median_error_pct']:.2f}%, mean {s['mean_error_pct']:.2f}%, 95th pct {s['p95_error_pct']:.2f}%")
    print(f"  within 1%: {s['within_1pct']:.1f}% of pixels, within 5%: {s['within_5pct']:.1f}%")
    print(f"  false depth in occluded areas: {s['false_matches_in_occlusion_pct']:.1f}% of them")
    print(f"  theory at median depth {median_depth:.0f} mm, quarter-pixel matching: "
          f"{results['theory']['expected_error_at_median_pct_quarter_px']:.2f}%")
    print(f"\n  {'object':<14}{'true depth':>12}{'coverage':>10}{'median err':>12}")
    for name, o in s["objects"].items():
        print(f"  {name:<14}{o['true_depth_mm']:>9.0f} mm{o['coverage_pct']:>9.1f}%{o['median_error_pct']:>11.2f}%")
    if "sgbm_wls" in results:
        w = results["sgbm_wls"]
        print(f"\n  with WLS: coverage {w['coverage_pct']:.1f}%, median {w['median_error_pct']:.2f}%, "
              f"95th pct {w['p95_error_pct']:.2f}%, false in occlusion {w['false_matches_in_occlusion_pct']:.1f}%")
    print(f"\n  wrote {out} in {time.perf_counter() - started:.0f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
