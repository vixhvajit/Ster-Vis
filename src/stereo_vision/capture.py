"""Save and load calibration image pairs. Live capture lives in sources.py."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


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
