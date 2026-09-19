"""Named speed/accuracy trade-offs, chosen from measurements on synthetic scenes.

A preset fixes three things: how far to scale the rectified image down, which
matcher to run, and its block size. The disparity range is not fixed; it is
derived from the closest distance you need to see, because that is the number
that matters physically, and it keeps the same meaning at every scale.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .calibration import StereoCalibration
from .config import SGBMParams


@dataclass(frozen=True)
class Preset:
    name: str
    scale: float
    mode: str
    block_size: int
    summary: str

    def calibration(self, calibration: StereoCalibration) -> StereoCalibration:
        """The calibration rectifying straight to this preset's resolution."""
        return calibration if self.scale == 1.0 else calibration.scaled(self.scale)

    def params(self, calibration: StereoCalibration, min_distance_mm: float) -> SGBMParams:
        """Matcher settings for a calibration already scaled by :meth:`calibration`."""
        return SGBMParams(
            num_disparities=disparities_for(calibration, min_distance_mm),
            block_size=self.block_size,
            mode=self.mode,
        )


def disparities_for(calibration: StereoCalibration, min_distance_mm: float) -> int:
    """Smallest valid disparity range that still sees down to ``min_distance_mm``.

    A point at distance Z has disparity f * B / Z, so the range must reach that
    at the closest distance. OpenCV needs a multiple of 16. A smaller range is
    faster and loses fewer columns at the left edge, where the matcher has
    nothing to compare against.
    """
    if min_distance_mm <= 0:
        raise ValueError("min_distance_mm must be positive")
    needed = calibration.focal_length_px * calibration.baseline / min_distance_mm
    return int(max(16, np.ceil(needed / 16.0) * 16))


# Chosen from a distance sweep against ray-traced truth (ster-vis benchmark
# --accuracy). sgbm_3way is kept throughout: it matched the other SGBM modes'
# accuracy at a third of their time. Below 0.375 scale, near-range error
# rose by about 70%, so no preset goes lower. Block matching was as accurate on
# the synthetic target but left 2-8% more pixels empty, and that target is
# richly textured, which flatters it; pi5-bm is there to try on real scenes.
PRESETS: dict[str, Preset] = {
    p.name: p
    for p in (
        Preset("quality", 1.0, "sgbm_3way", 5, "full resolution; for offline processing"),
        Preset("balanced", 0.75, "sgbm_3way", 5, "three-quarter resolution; a fast desktop in real time"),
        Preset("pi5", 0.5, "sgbm_3way", 5, "half resolution; the Raspberry Pi 5 default"),
        Preset("pi5-fast", 0.375, "sgbm_3way", 5, "three-eighths resolution; higher frame rate on a Pi 5"),
        Preset("pi5-bm", 0.5, "bm", 9, "half resolution, block matching; fewer filled pixels, try on your scene"),
    )
}

DEFAULT_MIN_DISTANCE_MM = 500.0


def get_preset(name: str) -> Preset:
    try:
        return PRESETS[name]
    except KeyError:
        raise ValueError(f"unknown preset {name!r}; choose from {', '.join(PRESETS)}") from None
