"""Grab synchronized frame pairs from two cameras."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np


@contextmanager
def stereo_cameras(
    left_index: int = 0,
    right_index: int = 1,
    width: int | None = None,
    height: int | None = None,
    backend: int = cv2.CAP_ANY,
) -> Iterator[tuple[cv2.VideoCapture, cv2.VideoCapture]]:
    """Open both cameras, and release them even if the caller raises.

    Two independent USB cameras are only loosely synchronized: frames are read
    back to back, which is fine for a static scene but will smear on fast
    motion. A hardware-triggered rig is the fix if that matters.
    """
    left = cv2.VideoCapture(left_index, backend)
    right = cv2.VideoCapture(right_index, backend)
    try:
        for capture, index in ((left, left_index), (right, right_index)):
            if not capture.isOpened():
                raise RuntimeError(f"could not open camera index {index}")
            if width is not None:
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height is not None:
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        yield left, right
    finally:
        left.release()
        right.release()


def read_pair(
    left: cv2.VideoCapture, right: cv2.VideoCapture
) -> tuple[np.ndarray, np.ndarray]:
    """Read one frame from each camera, raising if either read fails.

    ``grab`` is issued on both before either ``retrieve`` so the two exposures
    start as close together as the driver allows.
    """
    left.grab()
    right.grab()
    left_ok, left_frame = left.retrieve()
    right_ok, right_frame = right.retrieve()
    if not left_ok or not right_ok:
        raise RuntimeError("failed to read a frame from one of the cameras")
    return left_frame, right_frame


def save_pair(
    directory: str | Path, index: int, left: np.ndarray, right: np.ndarray
) -> tuple[Path, Path]:
    """Write one pair as ``left/NNN.png`` and ``right/NNN.png``."""
    directory = Path(directory)
    left_path = directory / "left" / f"{index:03d}.png"
    right_path = directory / "right" / f"{index:03d}.png"
    left_path.parent.mkdir(parents=True, exist_ok=True)
    right_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(left_path), left)
    cv2.imwrite(str(right_path), right)
    return left_path, right_path


def load_pairs(
    directory: str | Path, pattern: str = "*.png"
) -> tuple[list[np.ndarray], list[np.ndarray], list[str]]:
    """Load every pair from a directory holding ``left/`` and ``right/``.

    Files are matched by name, so ``left/007.png`` pairs with ``right/007.png``.
    Names present on only one side are skipped.
    """
    directory = Path(directory)
    left_dir = directory / "left"
    right_dir = directory / "right"

    left_files = {path.name: path for path in sorted(left_dir.glob(pattern))}
    right_files = {path.name: path for path in sorted(right_dir.glob(pattern))}
    shared = sorted(set(left_files) & set(right_files))

    left_images: list[np.ndarray] = []
    right_images: list[np.ndarray] = []
    names: list[str] = []
    for name in shared:
        left_image = cv2.imread(str(left_files[name]), cv2.IMREAD_COLOR)
        right_image = cv2.imread(str(right_files[name]), cv2.IMREAD_COLOR)
        if left_image is None or right_image is None:
            continue
        left_images.append(left_image)
        right_images.append(right_image)
        names.append(name)

    return left_images, right_images, names
