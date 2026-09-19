"""Stereo calibration: detect the board, solve the rig, build rectification maps."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np

from .config import BoardSpec

CORNER_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
STEREO_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

# The frame is split into this many columns and rows to measure how much of it
# the calibration boards reached.
COVERAGE_GRID = (8, 6)
# Below this share of cells, rectification near the uncovered edges can be off
# by many pixels even when the RMS looks excellent.
COVERAGE_WARN_PCT = 65.0


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
    # Share of the frame, in percent, that board corners reached in the worse
    # of the two cameras. NaN for calibrations saved before it was recorded.
    coverage_pct: float = float("nan")
    # Size of the rectified output, when it differs from the raw frames; see
    # scaled(). None means the same size as image_size.
    rectified_size: tuple[int, int] | None = None

    @property
    def baseline(self) -> float:
        """Distance between the camera centres, in the unit of square_size."""
        return float(np.linalg.norm(self.T))

    @property
    def focal_length_px(self) -> float:
        """Rectified focal length in pixels, taken from the projection matrix."""
        return float(self.P1[0, 0])

    @property
    def output_size(self) -> tuple[int, int]:
        """Width and height of rectified images."""
        return self.rectified_size or self.image_size

    def scaled(self, scale: float) -> "StereoCalibration":
        """The same rig, rectified straight to a smaller (or larger) image.

        Raw frames keep their size; only the rectified output changes, inside
        the same remap that rectification already does, so it costs nothing
        extra. Matching time falls roughly with the square of the scale, while
        disparity, and with it depth precision, falls linearly: at 0.5 the
        matcher does about a quarter of the work and depth error at a given
        distance roughly doubles.

        This is the right way to trade accuracy for speed on a slow board:
        calibrate once at full resolution, then run scaled. Capturing at a
        lower resolution instead only works if that camera mode sees exactly
        the same field of view, which many webcam modes do not (they crop).
        """
        if scale <= 0:
            raise ValueError("scale must be positive")
        pixels = np.diag([scale, scale, 1.0])
        Q = self.Q.copy()
        # Q maps (u, v, d, 1) to 3D. Scaling u, v and d by s leaves the 3D
        # point unchanged once cx, cy, f and (cx - cx') scale by s too.
        Q[0, 3] *= scale
        Q[1, 3] *= scale
        Q[2, 3] *= scale
        Q[3, 3] *= scale
        width, height = self.output_size
        return replace(
            self,
            P1=pixels @ self.P1,
            P2=pixels @ self.P2,
            Q=Q,
            rectified_size=(max(1, round(width * scale)), max(1, round(height * scale))),
        )

    def rectification_maps(
        self, map_type: int = cv2.CV_16SC2
    ) -> tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]]:
        """Build the remap tables that turn raw frames into rectified ones."""
        left = cv2.initUndistortRectifyMap(
            self.camera_matrix_left,
            self.dist_coeffs_left,
            self.R1,
            self.P1,
            self.output_size,
            map_type,
        )
        right = cv2.initUndistortRectifyMap(
            self.camera_matrix_right,
            self.dist_coeffs_right,
            self.R2,
            self.P2,
            self.output_size,
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
            coverage_pct=np.asarray(self.coverage_pct),
            rectified_size=np.asarray(self.rectified_size or (0, 0)),
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
            coverage_pct=float(data["coverage_pct"]) if "coverage_pct" in data else float("nan"),
            rectified_size=_optional_size(data),
        )


def _optional_size(data) -> tuple[int, int] | None:
    if "rectified_size" not in data:
        return None
    width, height = (int(v) for v in data["rectified_size"])
    return (width, height) if width and height else None


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
        half = refinement_half_window(corners, board)
        corners = cv2.cornerSubPix(gray, corners, (half, half), (-1, -1), CORNER_CRITERIA)
    return corners


def refinement_half_window(
    corners: np.ndarray, board: BoardSpec, fraction: float = 0.4, largest: int = 11
) -> int:
    """Half-size of the cornerSubPix search window, scaled to the board in view.

    A fixed window breaks on small or steeply tilted boards: once the window
    is wider than a square it reaches the neighbouring corner, and refinement
    can converge there instead, moving the point by a whole square. Keeping
    the window under half the shortest corner spacing rules that out.
    """
    grid = corners.reshape(board.rows, board.columns, 2)
    across = np.linalg.norm(np.diff(grid, axis=1), axis=2).min()
    down = np.linalg.norm(np.diff(grid, axis=0), axis=2).min()
    return int(np.clip(min(across, down) * fraction, 2, largest))


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
        coverage_pct=min(
            frame_coverage(left_points, image_size)[0],
            frame_coverage(right_points, image_size)[0],
        ),
    )
    return calibration, used


def frame_coverage(
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
    grid: tuple[int, int] = COVERAGE_GRID,
) -> tuple[float, np.ndarray]:
    """Share of the frame reached by detected corners, and which cells were reached.

    The frame is divided into ``grid`` columns and rows; a cell counts once any
    corner from any view lands in it. RMS cannot reveal poor coverage, because
    it only measures the fit where the boards were. The distortion model is
    unconstrained elsewhere and can extrapolate badly, which is invisible in
    the RMS and ruinous for depth near the edges.
    """
    columns, rows = grid
    width, height = image_size
    covered = np.zeros((rows, columns), bool)
    for points in image_points:
        xy = points.reshape(-1, 2)
        col = np.clip((xy[:, 0] / width * columns).astype(int), 0, columns - 1)
        row = np.clip((xy[:, 1] / height * rows).astype(int), 0, rows - 1)
        covered[row, col] = True
    return float(100.0 * covered.mean()), covered


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
