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


# 16-bit PNG in millimetres is the common interchange format for depth maps
# (RealSense, Kinect and most RGB-D datasets use it): lossless, viewable, and
# good to 1 mm up to 65.5 m. Zero marks pixels with no depth.
DEPTH_PNG_MAX_MM = 65535


def save_depth(path: str | Path, depth_mm: np.ndarray) -> Path:
    """Save depth in millimetres as a 16-bit PNG, or as float32 .npy.

    Non-finite and non-positive values are stored as 0 in the PNG, and as NaN
    in the .npy. Values beyond 65.5 m are clipped in the PNG.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    depth_mm = np.asarray(depth_mm, np.float32)
    valid = np.isfinite(depth_mm) & (depth_mm > 0)
    if path.suffix.lower() == ".npy":
        np.save(path, np.where(valid, depth_mm, np.nan).astype(np.float32))
        return path
    encoded = np.zeros(depth_mm.shape, np.uint16)
    encoded[valid] = np.clip(np.rint(depth_mm[valid]), 1, DEPTH_PNG_MAX_MM).astype(np.uint16)
    if not cv2.imwrite(str(path), encoded):
        raise OSError(f"could not write {path}")
    return path


def load_depth(path: str | Path) -> np.ndarray:
    """Load a depth map saved by :func:`save_depth`, as float32 mm with NaN for no depth."""
    path = Path(path)
    if path.suffix.lower() == ".npy":
        return np.load(path).astype(np.float32)
    encoded = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if encoded is None:
        raise OSError(f"could not read {path}")
    if encoded.dtype != np.uint16 or encoded.ndim != 2:
        raise ValueError(f"{path} is not a single-channel 16-bit depth PNG")
    depth = encoded.astype(np.float32)
    depth[encoded == 0] = np.nan
    return depth


def colorize_depth(
    depth_mm: np.ndarray, near: float | None = None, far: float | None = None
) -> np.ndarray:
    """Render depth as BGR with near = red and far = blue; no-depth pixels are black.

    Pass the same ``near`` and ``far`` to compare two maps on one scale.
    """
    valid = np.isfinite(depth_mm) & (depth_mm > 0)
    if near is None or far is None:
        values = depth_mm[valid]
        near = float(np.percentile(values, 1)) if values.size else 0.0
        far = float(np.percentile(values, 99)) if values.size else 1.0
    span = max(far - near, 1e-6)
    scaled = np.zeros(depth_mm.shape, np.uint8)
    scaled[valid] = np.clip((far - depth_mm[valid]) / span * 255.0, 0, 255).astype(np.uint8)
    colored = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    return colored
