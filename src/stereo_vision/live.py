"""The per-frame depth pipeline, with stage timings, for live use.

Everything that can be prepared once is prepared once: the scaled
calibration, the remap tables and the matcher. Per frame, the only work is
rectifying two greyscale images, matching them and converting to depth.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, replace

import cv2
import numpy as np

from .calibration import StereoCalibration, rectify_pair
from .config import SGBMParams
from .depth import colorize_depth, disparity_to_depth
from .disparity import build_matcher, valid_mask
from .presets import Preset
from .sources import StereoFrame


@dataclass
class DepthResult:
    left: np.ndarray        # rectified left image, as matched
    disparity: np.ndarray   # pixels, 0 where unmatched
    depth_mm: np.ndarray    # NaN where unmatched
    stage_ms: dict[str, float]


class DepthPipeline:
    def __init__(
        self,
        calibration: StereoCalibration,
        preset: Preset,
        min_distance_mm: float,
        num_disparities: int | None = None,
        block_size: int | None = None,
    ) -> None:
        self.raw_size = calibration.image_size
        self.calibration = preset.calibration(calibration)
        params = preset.params(self.calibration, min_distance_mm)
        if num_disparities is not None:
            params = replace(params, num_disparities=num_disparities)
        if block_size is not None:
            params = replace(params, block_size=block_size)
        self.params: SGBMParams = params
        self.maps = self.calibration.rectification_maps()
        self.matcher = build_matcher(params)
        self.closest_mm = (
            self.calibration.focal_length_px * self.calibration.baseline / params.num_disparities
        )

    def check_size(self, frame: StereoFrame) -> None:
        height, width = frame.left.shape[:2]
        if (width, height) != tuple(self.raw_size):
            raise ValueError(
                f"cameras deliver {width}x{height} but the calibration is for "
                f"{self.raw_size[0]}x{self.raw_size[1]}; capture at the calibrated size "
                "(use --width/--height), or recalibrate at this one"
            )

    def process(self, frame: StereoFrame) -> DepthResult:
        timings: dict[str, float] = {}
        start = time.perf_counter()
        left, right = frame.left, frame.right
        if left.ndim == 3:
            left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        left, right = rectify_pair(left, right, self.maps)
        timings["rectify"] = (time.perf_counter() - start) * 1000

        start = time.perf_counter()
        raw = self.matcher.compute(left, right)
        timings["match"] = (time.perf_counter() - start) * 1000

        start = time.perf_counter()
        disparity = raw.astype(np.float32) / 16.0
        disparity[~valid_mask(disparity, self.params)] = 0
        depth = disparity_to_depth(disparity, self.calibration.focal_length_px, self.calibration.baseline)
        depth[~np.isfinite(depth)] = np.nan
        timings["depth"] = (time.perf_counter() - start) * 1000
        return DepthResult(left, disparity, depth, timings)


@dataclass
class RateMeter:
    """Smoothed frames per second and stage times, for a status line."""

    smoothing: float = 0.1
    fps: float = 0.0
    stage_ms: dict[str, float] = field(default_factory=dict)
    frames: int = 0
    _last: float | None = None

    def tick(self, stage_ms: dict[str, float] | None = None) -> None:
        now = time.perf_counter()
        if self._last is not None:
            instant = 1.0 / max(now - self._last, 1e-9)
            self.fps = instant if self.frames <= 1 else self.fps + self.smoothing * (instant - self.fps)
        self._last = now
        self.frames += 1
        for name, value in (stage_ms or {}).items():
            previous = self.stage_ms.get(name, value)
            self.stage_ms[name] = previous + self.smoothing * (value - previous)

    def line(self) -> str:
        stages = "  ".join(f"{k} {v:5.1f} ms" for k, v in self.stage_ms.items())
        return f"{self.fps:5.1f} fps  {stages}"


def overlay(result: DepthResult, near_mm: float, far_mm: float, text: str) -> np.ndarray:
    """Rectified left view beside coloured depth, with a status line, for display."""
    depth = colorize_depth(result.depth_mm, near_mm, far_mm)
    left = cv2.cvtColor(result.left, cv2.COLOR_GRAY2BGR) if result.left.ndim == 2 else result.left
    view = cv2.hconcat([left, depth])
    scale = max(0.4, view.shape[1] / 1600)
    thickness = max(1, int(round(scale * 2)))
    cv2.rectangle(view, (0, 0), (view.shape[1], int(30 * scale) + 8), (0, 0, 0), -1)
    cv2.putText(view, text, (8, int(24 * scale) + 2), cv2.FONT_HERSHEY_SIMPLEX, scale * 0.8,
                (255, 255, 255), thickness, cv2.LINE_AA)
    return view
