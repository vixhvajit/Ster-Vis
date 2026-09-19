"""Benchmark the presets: frame rate on this machine, and accuracy against truth.

  python scripts/benchmark.py                  timing, synthetic frame (about a minute)
  python scripts/benchmark.py --threads 1,2,4  same, at several thread counts
  python scripts/benchmark.py --accuracy       also score depth over a 0.6-3 m sweep
  python scripts/benchmark.py --live           time real cameras with your calibration

Timing is only meaningful on the machine you will run on, so run this on the
Raspberry Pi itself. Accuracy does not depend on the machine, and is slow to
compute (the sweep is ray traced); the rendered sweep is cached in
output/benchmark/ so later runs are quick.

The table prints as Markdown, ready to paste into an issue or notes.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from stereo_vision.benchmark import (  # noqa: E402
    SWEEP_MM,
    evaluate_preset,
    machine_description,
    raspberry_pi_health,
    render_sweep,
    time_preset,
)
from stereo_vision.presets import DEFAULT_MIN_DISTANCE_MM, PRESETS, get_preset  # noqa: E402
from stereo_vision.synthetic import default_rig, true_calibration  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--presets", default=",".join(PRESETS), help="comma-separated preset names")
    parser.add_argument("--threads", default=None, help="comma-separated OpenCV thread counts to try")
    parser.add_argument("--min-distance", type=float, default=DEFAULT_MIN_DISTANCE_MM)
    parser.add_argument("--accuracy", action="store_true", help="score depth over a distance sweep (slow)")
    parser.add_argument("--live", action="store_true", help="time real cameras instead of a synthetic frame")
    parser.add_argument("--calibration", type=Path, default=ROOT / "calib" / "stereo.npz",
                        help="calibration for --live")
    parser.add_argument("--backend", choices=["auto", "opencv", "picamera2"], default="auto")
    parser.add_argument("--seconds", type=float, default=10.0, help="per preset, with --live")
    parser.add_argument("--json", type=Path, default=None, help="also write results as JSON")
    return parser.parse_args()


def synthetic_rows(presets, threads, args) -> list[dict]:
    rig = default_rig()
    calibration = true_calibration(rig)
    print("rendering a test frame ...", flush=True)
    frames = render_sweep(rig, distances_mm=(1500,), cache=ROOT / "output" / "benchmark" / "frame_1500.npz")
    frame = frames[0]
    sweep = None
    if args.accuracy:
        print(f"rendering the accuracy sweep ({len(SWEEP_MM)} distances; cached after the first run) ...", flush=True)
        sweep = render_sweep(rig, cache=ROOT / "output" / "benchmark" / "sweep.npz")

    rows = []
    for count in threads:
        if count:
            cv2.setNumThreads(count)
        for preset in presets:
            if sweep is not None:
                result = evaluate_preset(preset, calibration, rig, sweep, args.min_distance, timing_frame=frame)
                timing = result.timing
                accuracy = {"coverage_pct": result.coverage_pct, **{f"{k}_error_pct": v for k, v in result.band_error_pct.items()}}
                size, disparities, closest = result.size, result.num_disparities, result.closest_mm
            else:
                timing, scaled, params = time_preset(preset, calibration, frame.left, frame.right, args.min_distance)
                accuracy = {}
                size, disparities = scaled.output_size, params.num_disparities
                closest = scaled.focal_length_px * scaled.baseline / disparities
            rows.append({
                "preset": preset.name, "threads": cv2.getNumThreads(), "size": f"{size[0]}x{size[1]}",
                "mode": preset.mode, "disparities": disparities, "closest_mm": round(closest),
                "rectify_ms": timing.rectify_ms, "match_ms": timing.match_ms, "depth_ms": timing.depth_ms,
                "fps": timing.fps, **accuracy,
            })
            print(f"  {preset.name:<9} {cv2.getNumThreads()} threads: {timing.total_ms:6.1f} ms  ({timing.fps:5.1f} fps)", flush=True)
    return rows


def live_rows(presets, threads, args) -> list[dict]:
    from stereo_vision.calibration import load_calibration
    from stereo_vision.live import DepthPipeline, RateMeter
    from stereo_vision.sources import open_source

    calibration = load_calibration(args.calibration)
    width, height = calibration.image_size
    rows = []
    with open_source(args.backend, width=width, height=height, grey=True, threaded=True) as source:
        for count in threads:
            if count:
                cv2.setNumThreads(count)
            for preset in presets:
                pipeline = DepthPipeline(calibration, preset, args.min_distance)
                meter = RateMeter(smoothing=0.05)
                skews = []
                pipeline.check_size(source.read())
                end = time.monotonic() + args.seconds
                while time.monotonic() < end:
                    frame = source.read()
                    meter.tick(pipeline.process(frame).stage_ms)
                    if frame.skew_ms is not None:
                        skews.append(abs(frame.skew_ms))
                size = pipeline.calibration.output_size
                row = {
                    "preset": preset.name, "threads": cv2.getNumThreads(), "size": f"{size[0]}x{size[1]}",
                    "mode": preset.mode, "disparities": pipeline.params.num_disparities,
                    "closest_mm": round(pipeline.closest_mm),
                    **{f"{k}_ms": v for k, v in meter.stage_ms.items()}, "fps": meter.fps,
                }
                if skews:
                    row["skew_ms_p95"] = float(np.percentile(skews, 95))
                rows.append(row)
                print(f"  {preset.name:<9} {cv2.getNumThreads()} threads: {meter.fps:5.1f} fps", flush=True)
                health = raspberry_pi_health()
                if health:
                    print(f"    {health}", flush=True)
    return rows


def markdown(rows: list[dict]) -> str:
    columns = [
        ("preset", "Preset", "{}"), ("threads", "Threads", "{}"), ("size", "Size", "{}"),
        ("mode", "Matcher", "{}"), ("closest_mm", "Closest (mm)", "{}"),
        ("rectify_ms", "Rectify (ms)", "{:.1f}"), ("match_ms", "Match (ms)", "{:.1f}"),
        ("depth_ms", "Depth (ms)", "{:.1f}"), ("fps", "FPS", "{:.1f}"),
        ("skew_ms_p95", "Skew p95 (ms)", "{:.1f}"), ("coverage_pct", "Filled", "{:.1f}%"),
        ("near_error_pct", "Error 0.6-1 m", "{:.2f}%"), ("mid_error_pct", "Error 1.2-2 m", "{:.2f}%"),
        ("far_error_pct", "Error 2.2-3 m", "{:.2f}%"),
    ]
    present = [c for c in columns if any(c[0] in r for r in rows)]
    lines = ["| " + " | ".join(c[1] for c in present) + " |", "|" + "---|" * len(present)]
    for r in rows:
        lines.append("| " + " | ".join(c[2].format(r[c[0]]) if c[0] in r else "" for c in present) + " |")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    presets = [get_preset(name.strip()) for name in args.presets.split(",") if name.strip()]
    threads = [int(t) for t in args.threads.split(",")] if args.threads else [0]

    print(machine_description())
    health = raspberry_pi_health()
    if health:
        print(health)

    rows = live_rows(presets, threads, args) if args.live else synthetic_rows(presets, threads, args)

    print("\n" + machine_description() + ("  (live cameras)" if args.live else "  (synthetic frame)"))
    print(markdown(rows))
    health = raspberry_pi_health()
    if health:
        print("\nafter the run: " + health)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps({"machine": machine_description(), "rows": rows}, indent=2), encoding="utf-8")
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
