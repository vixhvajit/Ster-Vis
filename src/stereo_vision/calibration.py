"""Stereo calibration: detect the board, solve the rig, build rectification maps."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .config import BoardSpec

CORNER_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
STEREO_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)


@dataclass
class StereoCalibration:
    """Intrinsics, extrinsics and rectification output for one camera pair."""

    image_size: tuple[int, int]
    camera_matrix_left: np.ndarray
    dist_coeffs_left: np.ndarray
    camera_matrix_right: np.ndarray
    dist_coeffs_right: np.ndarray
    R: np.ndarray
    T: np.ndarray
    R1: np.ndarray
    R2: np.ndarray
    P1: np.ndarray
    P2: np.ndarray
    Q: np.ndarray
    rms: float

    @property
    def baseline(self) -> float:
        """Distance between the camera centres, in the unit of square_size."""
        return float(np.linalg.norm(self.T))

    @property
    def focal_length_px(self) -> float:
        """Rectified focal length in pixels, taken from the projection matrix."""
        return float(self.P1[0, 0])

    def rectification_maps(
        self, map_type: int = cv2.CV_16SC2
    ) -> tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]]:
        """Build the remap tables that turn raw frames into rectified ones."""
        left = cv2.initUndistortRectifyMap(
            self.camera_matrix_left,
            self.dist_coeffs_left,
            self.R1,
            self.P1,
            self.image_size,
            map_type,
        )
        right = cv2.initUndistortRectifyMap(
            self.camera_matrix_right,
            self.dist_coeffs_right,
            self.R2,
            self.P2,
            self.image_size,
            map_type,
        )
        return left, right

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            path,
            image_size=np.asarray(self.image_size),
            camera_matrix_left=self.camera_matrix_left,
            dist_coeffs_left=self.dist_coeffs_left,
            camera_matrix_right=self.camera_matrix_right,
            dist_coeffs_right=self.dist_coeffs_right,
            R=self.R,
            T=self.T,
            R1=self.R1,
            R2=self.R2,
            P1=self.P1,
            P2=self.P2,
            Q=self.Q,
            rms=np.asarray(self.rms),
        )
        return path


def load_calibration(path: str | Path) -> StereoCalibration:
    """Read a calibration written by StereoCalibration.save."""
    with np.load(Path(path)) as data:
        return StereoCalibration(
            image_size=tuple(int(v) for v in data["image_size"]),
            camera_matrix_left=data["camera_matrix_left"],
            dist_coeffs_left=data["dist_coeffs_left"],
            camera_matrix_right=data["camera_matrix_right"],
            dist_coeffs_right=data["dist_coeffs_right"],
            R=data["R"],
            T=data["T"],
            R1=data["R1"],
            R2=data["R2"],
            P1=data["P1"],
            P2=data["P2"],
            Q=data["Q"],
            rms=float(data["rms"]),
        )


def object_points(board: BoardSpec) -> np.ndarray:
    """Board corner coordinates in board space, with Z fixed at zero."""
    grid = np.zeros((board.corner_count, 3), np.float32)
    grid[:, :2] = np.mgrid[0 : board.columns, 0 : board.rows].T.reshape(-1, 2)
    return grid * board.square_size


def find_corners(
    image: np.ndarray, board: BoardSpec, refine: bool = True
) -> np.ndarray | None:
    """Locate inner chessboard corners, or return None if the board is absent."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board.pattern_size, flags)
    if not found:
        return None
    if refine:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CORNER_CRITERIA)
    return corners


def collect_correspondences(
    left_images: list[np.ndarray],
    right_images: list[np.ndarray],
    board: BoardSpec,
) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray], list[int]]:
    """Detect the board in both views, keeping only pairs where both succeed.

    Returns the object points, the two sets of image points, and the indices of
    the pairs that were used, so the caller can report which ones were dropped.
    """
    if len(left_images) != len(right_images):
        raise ValueError("left and right image lists must be the same length")

    template = object_points(board)
    obj_points: list[np.ndarray] = []
    left_points: list[np.ndarray] = []
    right_points: list[np.ndarray] = []
    used: list[int] = []

    for index, (left, right) in enumerate(zip(left_images, right_images)):
        left_corners = find_corners(left, board)
        right_corners = find_corners(right, board)
        if left_corners is None or right_corners is None:
            continue
        obj_points.append(template.copy())
        left_points.append(left_corners)
        right_points.append(right_corners)
        used.append(index)

    return obj_points, left_points, right_points, used


def calibrate_stereo(
    left_images: list[np.ndarray],
    right_images: list[np.ndarray],
    board: BoardSpec,
    alpha: float = 0.0,
) -> tuple[StereoCalibration, list[int]]:
    """Calibrate both cameras and the rig, then rectify.

    Each camera is solved on its own first, and those intrinsics are then held
    fixed while the relative pose is solved. That is more stable than letting
    stereoCalibrate move every parameter at once. alpha controls the rectified
    field of view: 0 crops to valid pixels only, 1 keeps all of them and leaves
    black borders.
    """
    obj_points, left_points, right_points, used = collect_correspondences(
        left_images, right_images, board
    )
    if len(obj_points) < 5:
        raise ValueError(
            "need at least 5 pairs with the board visible in both views, "
            f"got {len(obj_points)}"
        )

    height, width = left_images[used[0]].shape[:2]
    image_size = (width, height)

    _, matrix_left, dist_left, _, _ = cv2.calibrateCamera(
        obj_points, left_points, image_size, None, None
    )
    _, matrix_right, dist_right, _, _ = cv2.calibrateCamera(
        obj_points, right_points, image_size, None, None
    )

    result = cv2.stereoCalibrate(
        obj_points,
        left_points,
        right_points,
        matrix_left,
        dist_left,
        matrix_right,
        dist_right,
        image_size,
        criteria=STEREO_CRITERIA,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    rms, matrix_left, dist_left, matrix_right, dist_right, R, T = result[:7]

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        matrix_left,
        dist_left,
        matrix_right,
        dist_right,
        image_size,
        R,
        T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=alpha,
    )

    calibration = StereoCalibration(
        image_size=image_size,
        camera_matrix_left=matrix_left,
        dist_coeffs_left=dist_left,
        camera_matrix_right=matrix_right,
        dist_coeffs_right=dist_right,
        R=R,
        T=T,
        R1=R1,
        R2=R2,
        P1=P1,
        P2=P2,
        Q=Q,
        rms=float(rms),
    )
    return calibration, used


def rectify_pair(
    left: np.ndarray,
    right: np.ndarray,
    maps: tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]],
) -> tuple[np.ndarray, np.ndarray]:
    """Apply precomputed rectification maps to one frame pair."""
    (left_x, left_y), (right_x, right_y) = maps
    return (
        cv2.remap(left, left_x, left_y, cv2.INTER_LINEAR),
        cv2.remap(right, right_x, right_y, cv2.INTER_LINEAR),
    )
