"""Turn disparity into metric depth and into point clouds."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

PLY_HEADER = """ply
format ascii 1.0
element vertex {count}
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
"""


def disparity_to_depth(
    disparity: np.ndarray, focal_length_px: float, baseline: float
) -> np.ndarray:
    """Convert disparity in pixels to depth, via ``Z = f * B / d``.

    Depth comes back in whatever unit the baseline uses, which for a calibration
    run is the unit of ``BoardSpec.square_size``. Pixels with zero or negative
    disparity are infinitely far away and are returned as ``inf``.
    """
    depth = np.full(disparity.shape, np.inf, np.float32)
    valid = disparity > 0
    depth[valid] = (focal_length_px * baseline) / disparity[valid]
    return depth


def reproject_to_3d(disparity: np.ndarray, Q: np.ndarray) -> np.ndarray:
    """Project disparity into an HxWx3 array of camera-space coordinates."""
    return cv2.reprojectImageTo3D(disparity.astype(np.float32), Q)


def point_cloud(
    disparity: np.ndarray,
    Q: np.ndarray,
    color_image: np.ndarray,
    min_disparity: float = 0.0,
    max_depth: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Build an (N, 3) point array and its (N, 3) BGR colors.

    Points behind the camera, beyond ``max_depth``, or without a match are
    dropped, so N is smaller than the pixel count.
    """
    points = reproject_to_3d(disparity, Q)
    colors = color_image if color_image.ndim == 3 else cv2.cvtColor(color_image, cv2.COLOR_GRAY2BGR)

    mask = (disparity > min_disparity) & np.isfinite(points).all(axis=2)
    if max_depth is not None:
        mask &= points[:, :, 2] < max_depth

    return points[mask], colors[mask]


def write_ply(
    path: str | Path, points: np.ndarray, colors: np.ndarray | None = None
) -> Path:
    """Write an ASCII PLY file that MeshLab or CloudCompare can open.

    ``colors`` is BGR, as OpenCV supplies it, and is converted to the RGB order
    the PLY format expects.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    points = np.asarray(points, np.float32).reshape(-1, 3)
    if colors is None:
        rgb = np.full((len(points), 3), 200, np.uint8)
    else:
        bgr = np.asarray(colors, np.uint8).reshape(-1, 3)
        if len(bgr) != len(points):
            raise ValueError("points and colors must have the same length")
        rgb = bgr[:, ::-1]

    rows = np.hstack([points, rgb.astype(np.float32)])
    with path.open("w", encoding="utf-8") as handle:
        handle.write(PLY_HEADER.format(count=len(points)))
        np.savetxt(handle, rows, fmt="%.4f %.4f %.4f %d %d %d")
    return path
