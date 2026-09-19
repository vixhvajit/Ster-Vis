"""Decide when to save a calibration pair without anyone pressing a key.

On a headless Raspberry Pi there is no window to press SPACE in, so capture
has to decide for itself. A pair is saved when the board is seen in both
views, has been held still (a moving board blurs, and two unsynchronised
cameras see it at different moments), and either reaches part of the frame
not yet covered or sits clearly apart from every pose already saved.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from .config import BoardSpec


def quick_corners(image: np.ndarray, board: BoardSpec, scale: float = 0.5) -> np.ndarray | None:
    """Fast, approximate corners for previews and triggering.

    Runs the detector on a downscaled image with OpenCV's fast check, which
    rejects frames without a board quickly instead of searching them fully.
    That matters on a Pi, where a full-resolution search of an empty frame
    can take longer than a frame period. Corners come back in full-resolution
    pixels but unrefined; calibration detects them again, precisely, from the
    saved images.
    """
    grey = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(grey, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale != 1 else grey
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(small, board.pattern_size, flags)
    if not found:
        return None
    return corners / scale


def _cells(corners: np.ndarray, size: tuple[int, int], grid: tuple[int, int]) -> set[tuple[int, int]]:
    columns, rows = grid
    width, height = size
    xy = corners.reshape(-1, 2)
    col = np.clip((xy[:, 0] / width * columns).astype(int), 0, columns - 1)
    row = np.clip((xy[:, 1] / height * rows).astype(int), 0, rows - 1)
    return set(zip(row.tolist(), col.tolist()))


@dataclass
class AutoTrigger:
    image_size: tuple[int, int]
    grid: tuple[int, int] = (8, 6)
    still_px: float = 2.0          # corner movement between frames that counts as still
    still_frames: int = 5          # consecutive still frames needed
    novel_fraction: float = 0.08   # of image width: how far from every saved pose
    cooldown_s: float = 1.0
    saved_poses: list[np.ndarray] = field(default_factory=list)
    covered: set[tuple[int, int]] = field(default_factory=set)
    _previous: np.ndarray | None = None
    _still_count: int = 0
    _last_save: float = float("-inf")

    def remember(self, left_corners: np.ndarray, right_corners: np.ndarray) -> None:
        """Record a saved pair (also used for pairs already on disk)."""
        self.saved_poses.append(left_corners.reshape(-1, 2).copy())
        self.covered |= _cells(left_corners, self.image_size, self.grid)
        self.covered |= {(r, c + 1000) for r, c in _cells(right_corners, self.image_size, self.grid)}

    def update(
        self, left: np.ndarray | None, right: np.ndarray | None, now: float | None = None
    ) -> tuple[bool, str]:
        """Feed this frame's corners; returns (save now?, reason or what is missing)."""
        now = time.monotonic() if now is None else now
        if left is None or right is None:
            self._previous, self._still_count = None, 0
            return False, "board not in both views"

        points = left.reshape(-1, 2)
        if self._previous is not None and np.linalg.norm(points - self._previous, axis=1).mean() < self.still_px:
            self._still_count += 1
        else:
            self._still_count = 0
        self._previous = points.copy()

        if self._still_count < self.still_frames:
            return False, "hold still"
        if now - self._last_save < self.cooldown_s:
            return False, "just saved"

        new_cells = (_cells(left, self.image_size, self.grid)
                     | {(r, c + 1000) for r, c in _cells(right, self.image_size, self.grid)}) - self.covered
        distance = min(
            (np.linalg.norm(points - pose, axis=1).mean() for pose in self.saved_poses), default=np.inf
        )
        if not new_cells and distance < self.novel_fraction * self.image_size[0]:
            return False, "move to a new spot or angle"

        self.remember(left, right)
        self._last_save = now
        self._still_count = 0
        return True, f"saved ({len(new_cells)} new cells)" if new_cells else "saved (new pose)"
