"""Measure presets for speed and for accuracy against ray-traced ground truth.

Accuracy is scored on a flat textured target swept through many distances.
Scoring one scene is misleading: SGBM's sub-pixel estimate is pulled towards
whole pixels, so the error on a given surface depends on where its disparity
happens to fall between two integers, which is luck of the exact distance.
Averaging over a sweep removes that luck and leaves the real trend.

Accuracy does not depend on the computer, only timing does, so the same code
gives meaningful accuracy anywhere and meaningful timing on the target board.
"""

from __future__ import annotations

import os
import platform
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .calibration import StereoCalibration, rectify_pair
from .depth import disparity_to_depth
from .disparity import build_matcher, valid_mask
from .presets import Preset
from .scene import Plane, Scene, render_view, true_rectified_depth
from .synthetic import VirtualRig

SWEEP_MM = tuple(range(600, 3001, 200))
BANDS_MM = {"near": (600, 1000), "mid": (1200, 2000), "far": (2200, 3000)}


@dataclass
class SweepFrame:
    distance_mm: float
    left: np.ndarray   # raw greyscale, as a camera delivers it
    right: np.ndarray


def render_sweep(
    rig: VirtualRig,
    distances_mm=SWEEP_MM,
    supersample: int = 1,
    cache: Path | None = None,
) -> list[SweepFrame]:
    """Render the target at each distance, reusing a cache file if present."""
    if cache is not None and cache.exists():
        with np.load(cache) as data:
            if list(data["distances"]) == list(distances_mm):
                return [
                    SweepFrame(float(z), data["left"][i], data["right"][i])
                    for i, z in enumerate(data["distances"])
                ]

    frames = []
    for index, z in enumerate(distances_mm):
        scene = Scene([Plane("target", (0.0, 0.0, float(z)), (0.0, 0.0, -1.0))])
        rng = np.random.default_rng(100 + index)
        left = cv2.cvtColor(render_view(scene, rig, "left", supersample, rng), cv2.COLOR_BGR2GRAY)
        right = cv2.cvtColor(render_view(scene, rig, "right", supersample, rng), cv2.COLOR_BGR2GRAY)
        frames.append(SweepFrame(float(z), left, right))

    if cache is not None:
        cache.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            cache,
            distances=np.array(distances_mm, float),
            left=np.stack([f.left for f in frames]),
            right=np.stack([f.right for f in frames]),
        )
    return frames


@dataclass
class Timing:
    rectify_ms: float
    match_ms: float
    depth_ms: float

    @property
    def total_ms(self) -> float:
        return self.rectify_ms + self.match_ms + self.depth_ms

    @property
    def fps(self) -> float:
        return 1000.0 / self.total_ms if self.total_ms else float("inf")


@dataclass
class PresetResult:
    preset: Preset
    size: tuple[int, int]
    num_disparities: int
    closest_mm: float
    timing: Timing
    coverage_pct: float = float("nan")
    band_error_pct: dict[str, float] = field(default_factory=dict)
    per_distance: list[tuple[float, float, float]] = field(default_factory=list)


def _median_ms(fn, repeats: int) -> float:
    fn()  # warm up: first calls allocate buffers
    samples = []
    for _ in range(repeats):
        start = time.perf_counter()
        fn()
        samples.append(time.perf_counter() - start)
    return 1000.0 * float(np.median(samples))


def time_preset(
    preset: Preset,
    calibration: StereoCalibration,
    left: np.ndarray,
    right: np.ndarray,
    min_distance_mm: float,
    repeats: int = 15,
) -> tuple[Timing, StereoCalibration, object]:
    """Median time of each pipeline stage on one greyscale raw pair."""
    scaled = preset.calibration(calibration)
    params = preset.params(scaled, min_distance_mm)
    maps = scaled.rectification_maps()
    matcher = build_matcher(params)
    left_rect, right_rect = rectify_pair(left, right, maps)
    raw = matcher.compute(left_rect, right_rect)

    def to_depth():
        disparity = raw.astype(np.float32) / 16.0
        disparity[disparity <= params.min_disparity] = 0
        return disparity_to_depth(disparity, scaled.focal_length_px, scaled.baseline)

    timing = Timing(
        rectify_ms=_median_ms(lambda: rectify_pair(left, right, maps), repeats),
        match_ms=_median_ms(lambda: matcher.compute(left_rect, right_rect), repeats),
        depth_ms=_median_ms(to_depth, repeats),
    )
    return timing, scaled, params


def evaluate_preset(
    preset: Preset,
    calibration: StereoCalibration,
    rig: VirtualRig,
    frames: list[SweepFrame],
    min_distance_mm: float,
    timing_frame: SweepFrame | None = None,
) -> PresetResult:
    """Time a preset, then score its depth on every frame of the sweep."""
    frame = timing_frame or frames[len(frames) // 2]
    timing, scaled, params = time_preset(preset, calibration, frame.left, frame.right, min_distance_mm)
    matcher = build_matcher(params)
    maps = scaled.rectification_maps()

    per_distance = []
    covered = seen = 0
    for f in frames:
        left, right = rectify_pair(f.left, f.right, maps)
        disparity = matcher.compute(left, right).astype(np.float32) / 16.0
        disparity[~valid_mask(disparity, params)] = 0
        depth = disparity_to_depth(disparity, scaled.focal_length_px, scaled.baseline)
        scene = Scene([Plane("target", (0.0, 0.0, f.distance_mm), (0.0, 0.0, -1.0))])
        truth, _, visible = true_rectified_depth(scene, rig, scaled)
        scored = np.isfinite(depth) & visible
        error = np.abs(depth[scored] - truth[scored]) / truth[scored] * 100.0
        covered += int(scored.sum())
        seen += int(visible.sum())
        per_distance.append((f.distance_mm, float(np.median(error)) if error.size else float("nan"),
                             100.0 * scored.sum() / max(visible.sum(), 1)))

    bands = {}
    for name, (lo, hi) in BANDS_MM.items():
        errors = [e for z, e, _ in per_distance if lo <= z <= hi and np.isfinite(e)]
        bands[name] = float(np.mean(errors)) if errors else float("nan")

    return PresetResult(
        preset=preset,
        size=scaled.output_size,
        num_disparities=params.num_disparities,
        closest_mm=scaled.focal_length_px * scaled.baseline / params.num_disparities,
        timing=timing,
        coverage_pct=100.0 * covered / max(seen, 1),
        band_error_pct=bands,
        per_distance=per_distance,
    )


def machine_description() -> str:
    """CPU, core count and OpenCV build, for labelling results."""
    model = ""
    try:
        model = Path("/proc/device-tree/model").read_text(errors="ignore").strip("\x00\n ")
    except OSError:
        pass
    cpu = model or platform.processor() or platform.machine()
    return (
        f"{cpu} | {platform.system()} {platform.machine()} | {os.cpu_count()} cores | "
        f"OpenCV {cv2.__version__}, {cv2.getNumThreads()} threads"
    )


def raspberry_pi_health() -> str | None:
    """Temperature and throttling state on a Raspberry Pi, or None elsewhere.

    A Pi 5 without active cooling throttles under sustained stereo matching,
    and a throttled run reports a frame rate that the board cannot hold.
    """
    try:
        temp = subprocess.run(["vcgencmd", "measure_temp"], capture_output=True, text=True, timeout=2)
        throttled = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2)
    except (OSError, subprocess.SubprocessError):
        return None
    if temp.returncode != 0:
        return None
    value = throttled.stdout.strip().split("=")[-1]
    flags = int(value, 16) if value.startswith("0x") else 0
    # Bit meanings from the Raspberry Pi documentation for vcgencmd get_throttled.
    notes = []
    if flags & 0x4:
        notes.append("THROTTLED NOW")
    if flags & 0x2:
        notes.append("frequency capped now")
    if flags & 0x8:
        notes.append("soft temperature limit active")
    if flags & 0x40000:
        notes.append("has throttled since boot")
    if flags & 0x1:
        notes.append("UNDER-VOLTAGE now: use the official 27 W supply")
    state = ", ".join(notes) if notes else "not throttled"
    return f"{temp.stdout.strip()} | {state} (get_throttled={value})"
