"""Disparity estimation with StereoSGBM, plus the optional WLS refinement."""

from __future__ import annotations

import cv2
import numpy as np

from .config import SGBMParams

# StereoSGBM and StereoBM both return fixed-point disparity with 4 fractional
# bits, so raw values have to be divided by 16 to become pixels.
DISPARITY_SCALE = 16.0


SGBM_MODES = {
    "sgbm": cv2.STEREO_SGBM_MODE_SGBM,
    "sgbm_3way": cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    "hh4": cv2.STEREO_SGBM_MODE_HH4,
}


def build_matcher(params: SGBMParams | None = None) -> cv2.StereoMatcher:
    """Create a left-view matcher (SGBM or block matching) from the parameters."""
    params = params or SGBMParams()
    if params.mode == "bm":
        matcher = cv2.StereoBM_create(
            numDisparities=params.num_disparities, blockSize=params.block_size
        )
        matcher.setMinDisparity(params.min_disparity)
        matcher.setPreFilterCap(min(params.pre_filter_cap, 63))
        matcher.setUniquenessRatio(params.uniqueness_ratio)
        matcher.setSpeckleWindowSize(params.speckle_window_size)
        matcher.setSpeckleRange(params.speckle_range)
        matcher.setDisp12MaxDiff(params.disp12_max_diff)
        # Reject blocks with too little texture to match reliably.
        matcher.setTextureThreshold(10)
        return matcher
    return cv2.StereoSGBM_create(
        minDisparity=params.min_disparity,
        numDisparities=params.num_disparities,
        blockSize=params.block_size,
        P1=params.p1,
        P2=params.p2,
        disp12MaxDiff=params.disp12_max_diff,
        preFilterCap=params.pre_filter_cap,
        uniquenessRatio=params.uniqueness_ratio,
        speckleWindowSize=params.speckle_window_size,
        speckleRange=params.speckle_range,
        mode=SGBM_MODES[params.mode],
    )


def _to_gray(image: np.ndarray) -> np.ndarray:
    return image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def compute_disparity(
    left: np.ndarray,
    right: np.ndarray,
    params: SGBMParams | None = None,
    matcher: cv2.StereoMatcher | None = None,
) -> np.ndarray:
    """Compute a float32 disparity map in pixels.

    Both inputs must already be rectified. Unmatched pixels come back below
    ``min_disparity``; callers usually mask them with :func:`valid_mask`.
    """
    params = params or SGBMParams()
    matcher = matcher or build_matcher(params)
    raw = matcher.compute(_to_gray(left), _to_gray(right))
    return raw.astype(np.float32) / DISPARITY_SCALE


def compute_disparity_wls(
    left: np.ndarray,
    right: np.ndarray,
    params: SGBMParams | None = None,
    lambda_value: float = 8000.0,
    sigma: float = 1.5,
) -> np.ndarray:
    """Compute disparity and smooth it with the WLS filter.

    The filter fills small holes and snaps edges to image boundaries, at the
    cost of a second matching pass over the right view. Needs
    ``opencv-contrib-python``; raises ``RuntimeError`` on a plain build.
    """
    if not hasattr(cv2, "ximgproc"):
        raise RuntimeError(
            "WLS filtering needs opencv-contrib-python; install it or use compute_disparity"
        )

    params = params or SGBMParams()
    left_matcher = build_matcher(params)
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

    left_gray = _to_gray(left)
    right_gray = _to_gray(right)
    left_raw = left_matcher.compute(left_gray, right_gray)
    right_raw = right_matcher.compute(right_gray, left_gray)

    wls = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    wls.setLambda(lambda_value)
    wls.setSigmaColor(sigma)
    filtered = wls.filter(left_raw, left, disparity_map_right=right_raw)
    return filtered.astype(np.float32) / DISPARITY_SCALE


def valid_mask(disparity: np.ndarray, params: SGBMParams | None = None) -> np.ndarray:
    """Boolean mask of pixels that actually matched."""
    params = params or SGBMParams()
    return disparity > float(params.min_disparity)


def colorize(disparity: np.ndarray, params: SGBMParams | None = None) -> np.ndarray:
    """Render disparity as a BGR image for display, with invalid pixels black."""
    params = params or SGBMParams()
    mask = valid_mask(disparity, params)
    normalized = np.zeros(disparity.shape, np.uint8)
    if mask.any():
        low = float(disparity[mask].min())
        high = float(disparity[mask].max())
        span = max(high - low, 1e-6)
        scaled = (disparity - low) / span * 255.0
        normalized[mask] = np.clip(scaled[mask], 0, 255).astype(np.uint8)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    colored[~mask] = 0
    return colored
