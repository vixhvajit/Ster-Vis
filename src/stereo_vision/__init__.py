"""Stereo vision toolkit: calibration, rectification, disparity and depth."""

__version__ = "0.1.0"

from .config import BoardSpec, SGBMParams
from .calibration import StereoCalibration, calibrate_stereo, load_calibration
from .disparity import build_matcher, compute_disparity
from .depth import disparity_to_depth, reproject_to_3d, write_ply

__all__ = [
    "BoardSpec",
    "SGBMParams",
    "StereoCalibration",
    "calibrate_stereo",
    "load_calibration",
    "build_matcher",
    "compute_disparity",
    "disparity_to_depth",
    "reproject_to_3d",
    "write_ply",
]
