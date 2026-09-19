"""Tunable parameters for the calibration and matching stages."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BoardSpec:
    """Geometry of the calibration target.

    ``columns`` and ``rows`` count *inner* corners, not squares: a board with
    10x7 squares has 9x6 inner corners. ``square_size`` sets the unit of every
    downstream measurement, so passing millimetres yields depth in millimetres.
    """

    columns: int = 9
    rows: int = 6
    square_size: float = 25.0

    @property
    def pattern_size(self) -> tuple[int, int]:
        return (self.columns, self.rows)

    @property
    def corner_count(self) -> int:
        return self.columns * self.rows


MATCHER_MODES = ("sgbm_3way", "sgbm", "hh4", "bm")


@dataclass(frozen=True)
class SGBMParams:
    """Stereo matcher settings.

    ``num_disparities`` must be a positive multiple of 16 and bounds the closest
    measurable distance; ``block_size`` must be odd. The penalty defaults follow
    the OpenCV guidance of 8 and 32 times the channel count times block area.

    ``mode`` picks the algorithm:

    - ``sgbm_3way``: semi-global matching, OpenCV's fastest SGBM variant (default)
    - ``sgbm``: the original five-direction SGBM, slower, marginally smoother
    - ``hh4``: SGBM over four directions
    - ``bm``: plain block matching, several times faster than any SGBM but with
      more holes and noise; the penalties and ``channels`` do not apply to it
    """

    min_disparity: int = 0
    num_disparities: int = 128
    block_size: int = 5
    uniqueness_ratio: int = 10
    speckle_window_size: int = 100
    speckle_range: int = 2
    disp12_max_diff: int = 1
    pre_filter_cap: int = 31
    channels: int = 1
    mode: str = "sgbm_3way"

    def __post_init__(self) -> None:
        if self.num_disparities <= 0 or self.num_disparities % 16 != 0:
            raise ValueError("num_disparities must be a positive multiple of 16")
        if self.block_size % 2 == 0:
            raise ValueError("block_size must be odd")
        if self.mode not in MATCHER_MODES:
            raise ValueError(f"mode must be one of {', '.join(MATCHER_MODES)}")
        if self.mode == "bm" and self.block_size < 5:
            raise ValueError("block matching needs block_size of at least 5")

    @property
    def p1(self) -> int:
        return 8 * self.channels * self.block_size**2

    @property
    def p2(self) -> int:
        return 32 * self.channels * self.block_size**2
