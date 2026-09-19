"""A virtual stereo rig that photographs the chessboard with known geometry.

Calibration has no ground truth when run on real cameras: a plausible-looking
result can still be wrong. Rendering the board through a rig whose intrinsics,
distortion and pose are chosen up front lets the pipeline's answer be compared
against the numbers that actually produced the images.

Images are rendered by inverse mapping. Every output pixel is undistorted to
an ideal pinhole pixel, carried back onto the board plane by the inverse
homography, and sampled from a high-resolution raster of the board. That
reproduces lens distortion exactly, which a plain homography warp cannot.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from .calibration import object_points
from .config import BoardSpec
from .pattern import render_chessboard

TEXTURE_PX_PER_SQUARE = 80


@dataclass(frozen=True)
class VirtualCamera:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray


@dataclass
class VirtualRig:
    """Two cameras with a known relative pose.

    ``R`` and ``T`` follow the OpenCV stereoCalibrate convention: they carry a
    point from the left camera frame into the right camera frame,
    ``X_right = R @ X_left + T``.
    """

    left: VirtualCamera
    right: VirtualCamera
    R: np.ndarray
    T: np.ndarray
    image_size: tuple[int, int] = (1280, 720)
    _ideal_grids: dict = field(default_factory=dict, repr=False)

    @property
    def baseline(self) -> float:
        return float(np.linalg.norm(self.T))

    def _ideal_grid(self, side: str) -> np.ndarray:
        """Undistorted pixel position of every output pixel, computed once."""
        if side not in self._ideal_grids:
            camera = self.left if side == "left" else self.right
            width, height = self.image_size
            xs, ys = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
            pixels = np.stack([xs.ravel(), ys.ravel()], axis=1).reshape(-1, 1, 2)
            ideal = cv2.undistortPoints(
                pixels, camera.camera_matrix, camera.dist_coeffs, P=camera.camera_matrix
            )
            self._ideal_grids[side] = ideal.reshape(height, width, 2)
        return self._ideal_grids[side]

    def board_pose_in(self, side: str, rvec: np.ndarray, tvec: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Board rotation matrix and translation in the given camera's frame."""
        rotation, _ = cv2.Rodrigues(rvec)
        translation = tvec.reshape(3, 1)
        if side == "right":
            rotation = self.R @ rotation
            translation = self.R @ translation + self.T.reshape(3, 1)
        return rotation, translation

    def project_corners(self, side: str, board: BoardSpec, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        camera = self.left if side == "left" else self.right
        rotation, translation = self.board_pose_in(side, rvec, tvec)
        points, _ = cv2.projectPoints(
            object_points(board), cv2.Rodrigues(rotation)[0], translation,
            camera.camera_matrix, camera.dist_coeffs,
        )
        return points.reshape(-1, 2)

    def render(
        self,
        side: str,
        board: BoardSpec,
        rvec: np.ndarray,
        tvec: np.ndarray,
        texture: np.ndarray,
        rng: np.random.Generator | None = None,
    ) -> np.ndarray:
        """Photograph the board from one camera, as an 8-bit BGR image."""
        camera = self.left if side == "left" else self.right
        rotation, translation = self.board_pose_in(side, rvec, tvec)

        # Homography from board millimetres (X, Y, 1) to ideal pixels.
        homography = camera.camera_matrix @ np.hstack([rotation[:, :2], translation])
        inverse = np.linalg.inv(homography)

        ideal = self._ideal_grid(side)
        height, width = ideal.shape[:2]
        homogeneous = np.concatenate([ideal, np.ones((height, width, 1), np.float32)], axis=2)
        board_mm = homogeneous @ inverse.T
        board_mm = board_mm[:, :, :2] / board_mm[:, :, 2:3]

        # object_points puts the first inner corner at the origin, and the
        # texture has one square of margin plus one full square before it.
        # OpenCV puts pixel centres on integer coordinates, so the edge between
        # texture pixels 159 and 160 sits at 159.5, not 160.
        scale = TEXTURE_PX_PER_SQUARE / board.square_size
        offset = 2 * TEXTURE_PX_PER_SQUARE - 0.5
        map_x = (board_mm[:, :, 0] * scale + offset).astype(np.float32)
        map_y = (board_mm[:, :, 1] * scale + offset).astype(np.float32)

        # Off-board pixels see a mid-grey background, not the white quiet zone.
        image = cv2.remap(
            texture, map_x, map_y, cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=110,
        )

        # A little optical blur and sensor noise, so corner refinement has to
        # work for its accuracy rather than reading perfect edges.
        image = cv2.GaussianBlur(image, (0, 0), 0.8)
        if rng is not None:
            noise = rng.normal(0.0, 2.0, image.shape)
            image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def default_rig() -> VirtualRig:
    """A 60 mm webcam-like rig with mild barrel distortion and a slight misalignment.

    The two cameras differ a little in focal length, principal point and
    distortion, and the right one is yawed and rolled by a fraction of a
    degree, so rectification has real work to do.
    """
    left = VirtualCamera(
        camera_matrix=np.array([[700.0, 0, 643.0], [0, 700.0, 358.0], [0, 0, 1]]),
        dist_coeffs=np.array([-0.12, 0.05, 0.0005, -0.0003, 0.0]),
    )
    right = VirtualCamera(
        camera_matrix=np.array([[706.0, 0, 636.0], [0, 706.0, 363.0], [0, 0, 1]]),
        dist_coeffs=np.array([-0.10, 0.04, -0.0004, 0.0002, 0.0]),
    )
    R, _ = cv2.Rodrigues(np.deg2rad(np.array([0.3, -1.0, 0.5])))
    right_centre = np.array([[60.0], [0.0], [0.0]])
    T = -R @ right_centre
    return VirtualRig(left=left, right=right, R=R, T=T)


def random_poses(
    rig: VirtualRig,
    board: BoardSpec,
    count: int,
    rng: np.random.Generator,
    edge_margin: float = 30.0,
    max_attempts: int = 5000,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Board poses spread over depth, tilt and frame position.

    Each pose is kept only if every corner lands inside both images with a
    margin, which is what a person does when capturing real pairs.
    """
    centre = np.array(
        [board.columns - 1, board.rows - 1, 0.0]
    ) * board.square_size / 2
    width, height = rig.image_size
    poses: list[tuple[np.ndarray, np.ndarray]] = []

    for _ in range(max_attempts):
        if len(poses) == count:
            break
        tilt = np.deg2rad(rng.uniform([-35, -35, -20], [35, 35, 20]))
        rotation, _ = cv2.Rodrigues(tilt)
        depth = rng.uniform(420.0, 900.0)
        spread = depth * 0.35
        position = np.array([rng.uniform(-spread, spread) + 30.0, rng.uniform(-spread, spread) * 0.6, depth])
        tvec = (position - rotation @ centre).reshape(3, 1)
        rvec = cv2.Rodrigues(rotation)[0]

        visible = True
        for side in ("left", "right"):
            corners = rig.project_corners(side, board, rvec, tvec)
            inside = (
                (corners[:, 0] > edge_margin)
                & (corners[:, 0] < width - edge_margin)
                & (corners[:, 1] > edge_margin)
                & (corners[:, 1] < height - edge_margin)
            )
            if not inside.all():
                visible = False
                break
        if visible:
            poses.append((rvec, tvec))

    if len(poses) < count:
        raise RuntimeError(f"found only {len(poses)} visible poses out of {count}")
    return poses


def render_pairs(
    rig: VirtualRig, board: BoardSpec, count: int = 15, seed: int = 0
) -> tuple[list[np.ndarray], list[np.ndarray], list[tuple[np.ndarray, np.ndarray]]]:
    """Render ``count`` calibration pairs and return them with their true poses."""
    rng = np.random.default_rng(seed)
    texture = render_chessboard(board, TEXTURE_PX_PER_SQUARE, margin_squares=1.0)
    poses = random_poses(rig, board, count, rng)
    lefts = [rig.render("left", board, rvec, tvec, texture, rng) for rvec, tvec in poses]
    rights = [rig.render("right", board, rvec, tvec, texture, rng) for rvec, tvec in poses]
    return lefts, rights, poses


def _rotation_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle of the rotation that takes b to a."""
    cosine = (np.trace(a @ b.T) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def evaluate(calibration, rig: VirtualRig, board: BoardSpec, holdout_seed: int = 999, holdout_count: int = 3) -> dict:
    """Compare a calibration against the rig that rendered its images.

    Intrinsics, baseline and rotation are checked directly. Rectification and
    depth are checked on fresh pairs the calibration never saw: corners must
    land on the same row in both rectified views, and depth triangulated from
    their disparity must match the true distance.
    """
    from .calibration import find_corners, rectify_pair
    from .depth import disparity_to_depth

    metrics: dict[str, float] = {"rms_px": calibration.rms}

    for side, truth, estimate in (
        ("left", rig.left, calibration.camera_matrix_left),
        ("right", rig.right, calibration.camera_matrix_right),
    ):
        true_matrix = truth.camera_matrix
        metrics[f"{side}_focal_error_pct"] = float(
            100.0 * abs(estimate[0, 0] - true_matrix[0, 0]) / true_matrix[0, 0]
        )
        metrics[f"{side}_principal_point_error_px"] = float(
            np.hypot(estimate[0, 2] - true_matrix[0, 2], estimate[1, 2] - true_matrix[1, 2])
        )

    metrics["baseline_true_mm"] = rig.baseline
    metrics["baseline_estimated_mm"] = calibration.baseline
    metrics["baseline_error_pct"] = 100.0 * abs(calibration.baseline - rig.baseline) / rig.baseline
    metrics["rotation_error_deg"] = _rotation_angle_deg(calibration.R, rig.R)

    lefts, rights, poses = render_pairs(rig, board, count=holdout_count, seed=holdout_seed)
    maps = calibration.rectification_maps()
    row_errors: list[np.ndarray] = []
    depth_errors: list[np.ndarray] = []

    for left, right, (rvec, tvec) in zip(lefts, rights, poses):
        left_rect, right_rect = rectify_pair(left, right, maps)
        left_corners = find_corners(left_rect, board)
        right_corners = find_corners(right_rect, board)
        if left_corners is None or right_corners is None:
            raise RuntimeError("board not found in a rectified holdout pair")
        left_corners = left_corners.reshape(-1, 2)
        right_corners = right_corners.reshape(-1, 2)

        row_errors.append(np.abs(left_corners[:, 1] - right_corners[:, 1]))

        disparity = (left_corners[:, 0] - right_corners[:, 0]).astype(np.float32)
        estimated_depth = disparity_to_depth(
            disparity, calibration.focal_length_px, calibration.baseline
        )

        # True corner positions in the left camera, expressed along the
        # rectified optical axis that the estimated depth is measured on.
        rotation, translation = rig.board_pose_in("left", rvec, tvec)
        corners_left = (rotation @ object_points(board).T + translation).T
        true_depth = (calibration.R1 @ corners_left.T).T[:, 2]

        depth_errors.append(100.0 * np.abs(estimated_depth - true_depth) / true_depth)

    rows = np.concatenate(row_errors)
    depths = np.concatenate(depth_errors)
    metrics["rectified_row_error_mean_px"] = float(rows.mean())
    metrics["rectified_row_error_max_px"] = float(rows.max())
    metrics["depth_error_mean_pct"] = float(depths.mean())
    metrics["depth_error_max_pct"] = float(depths.max())
    return metrics


# Pass thresholds. Loose enough to absorb the rendered noise, tight enough that
# a real bug in calibration, rectification or triangulation cannot hide under
# them.
#
# Rotation is the loosest on purpose. With 15 noisy views each principal point
# is only pinned to 2-3 px, and at f = 700 px that alone tilts the recovered
# rig by atan(3 / 700), about 0.25 degrees. The error is shared consistently
# by R and the intrinsics, so it cancels in rectification and depth, which is
# why those two are held much tighter. 0.5 degrees still catches a transposed
# or inverted R, which shows up as more than 2 degrees on this rig.
TOLERANCES = {
    "rms_px": 0.5,
    "left_focal_error_pct": 1.0,
    "right_focal_error_pct": 1.0,
    "left_principal_point_error_px": 5.0,
    "right_principal_point_error_px": 5.0,
    "baseline_error_pct": 1.0,
    "rotation_error_deg": 0.5,
    "rectified_row_error_mean_px": 0.3,
    "depth_error_mean_pct": 1.0,
    "depth_error_max_pct": 3.0,
}
