"""Tests that run without any camera attached.

The disparity test builds a synthetic pair by shifting a textured image by a
known number of pixels, which is exactly what a rectified pair of a fronto-
parallel plane looks like, so the matcher should recover that shift.
"""

from __future__ import annotations

import numpy as np
import pytest

from stereo_vision.config import BoardSpec, SGBMParams
from stereo_vision.depth import disparity_to_depth, write_ply
from stereo_vision.disparity import compute_disparity, valid_mask


def textured_image(height: int = 240, width: int = 320, seed: int = 0) -> np.ndarray:
    """Random texture, which gives the matcher unambiguous features to lock onto."""
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, size=(height, width), dtype=np.uint8)


class TestBoardSpec:
    def test_pattern_size_and_count(self):
        board = BoardSpec(columns=9, rows=6, square_size=25.0)
        assert board.pattern_size == (9, 6)
        assert board.corner_count == 54

    def test_object_points_scale_with_square_size(self):
        from stereo_vision.calibration import object_points

        grid = object_points(BoardSpec(columns=4, rows=3, square_size=10.0))
        assert grid.shape == (12, 3)
        assert np.allclose(grid[:, 2], 0.0)
        assert grid[:, :2].max() == pytest.approx(30.0)


class TestSGBMParams:
    def test_penalties_follow_block_size(self):
        params = SGBMParams(block_size=5, channels=1)
        assert params.p1 == 8 * 25
        assert params.p2 == 32 * 25

    def test_rejects_bad_disparity_range(self):
        with pytest.raises(ValueError, match="multiple of 16"):
            SGBMParams(num_disparities=100)

    def test_rejects_even_block_size(self):
        with pytest.raises(ValueError, match="odd"):
            SGBMParams(block_size=4)


class TestDisparity:
    @pytest.mark.parametrize("shift", [8, 16])
    def test_recovers_a_known_shift(self, shift):
        left = textured_image()
        # The right view of a plane sees everything moved left by the disparity.
        right = np.roll(left, -shift, axis=1)

        params = SGBMParams(num_disparities=32, block_size=5)
        disparity = compute_disparity(left, right, params)

        # Ignore the borders, where the roll wraps and the window runs off frame.
        interior = disparity[40:-40, 60:-60]
        matched = interior[interior > 0]
        assert matched.size > 0
        assert np.median(matched) == pytest.approx(shift, abs=1.0)

    def test_valid_mask_excludes_unmatched(self):
        disparity = np.array([[-1.0, 0.0], [4.0, 9.0]], np.float32)
        mask = valid_mask(disparity, SGBMParams(min_disparity=0))
        assert mask.tolist() == [[False, False], [True, True]]


class TestDepth:
    def test_depth_follows_the_disparity_formula(self):
        disparity = np.array([[10.0, 20.0]], np.float32)
        depth = disparity_to_depth(disparity, focal_length_px=500.0, baseline=60.0)
        assert depth[0, 0] == pytest.approx(3000.0)
        # Twice the disparity is half the distance.
        assert depth[0, 1] == pytest.approx(1500.0)

    def test_unmatched_pixels_are_infinite(self):
        disparity = np.array([[0.0, -3.0]], np.float32)
        depth = disparity_to_depth(disparity, 500.0, 60.0)
        assert np.isinf(depth).all()


class TestPly:
    def test_writes_header_and_rows(self, tmp_path):
        points = np.array([[0.0, 0.0, 1.0], [1.0, 2.0, 3.0]], np.float32)
        colors = np.array([[255, 0, 0], [0, 0, 255]], np.uint8)  # BGR blue, BGR red

        path = write_ply(tmp_path / "cloud.ply", points, colors)
        lines = path.read_text(encoding="utf-8").splitlines()

        assert lines[0] == "ply"
        assert "element vertex 2" in lines
        body = lines[lines.index("end_header") + 1 :]
        assert len(body) == 2
        # BGR blue (255, 0, 0) is written as RGB 0 0 255.
        assert body[0].split()[3:] == ["0", "0", "255"]
        assert body[1].split()[3:] == ["255", "0", "0"]

    def test_rejects_mismatched_colors(self, tmp_path):
        points = np.zeros((3, 3), np.float32)
        with pytest.raises(ValueError, match="same length"):
            write_ply(tmp_path / "bad.ply", points, np.zeros((2, 3), np.uint8))


class TestCalibrationRoundTrip:
    def test_save_and_load_preserves_matrices(self, tmp_path):
        from stereo_vision.calibration import StereoCalibration, load_calibration

        identity = np.eye(3)
        calibration = StereoCalibration(
            image_size=(640, 480),
            camera_matrix_left=identity * 500,
            dist_coeffs_left=np.zeros((1, 5)),
            camera_matrix_right=identity * 501,
            dist_coeffs_right=np.zeros((1, 5)),
            R=identity,
            T=np.array([[-60.0], [0.0], [0.0]]),
            R1=identity,
            R2=identity,
            P1=np.hstack([identity * 500, np.zeros((3, 1))]),
            P2=np.hstack([identity * 500, np.array([[-30000.0], [0.0], [0.0]])]),
            Q=np.eye(4),
            rms=0.42,
        )

        loaded = load_calibration(calibration.save(tmp_path / "stereo.npz"))

        assert loaded.image_size == (640, 480)
        assert loaded.rms == pytest.approx(0.42)
        assert loaded.baseline == pytest.approx(60.0)
        assert loaded.focal_length_px == pytest.approx(500.0)
        assert np.allclose(loaded.Q, np.eye(4))


class TestCalibrationTarget:
    def test_opencv_finds_every_corner_on_the_default_board(self):
        import cv2

        from stereo_vision.calibration import find_corners
        from stereo_vision.pattern import render_chessboard

        board = BoardSpec()
        image = cv2.cvtColor(render_chessboard(board), cv2.COLOR_GRAY2BGR)
        corners = find_corners(image, board)

        assert corners is not None
        assert len(corners) == board.corner_count

    def test_pdf_is_true_scale_on_a4(self, tmp_path):
        from stereo_vision.pattern import MM_TO_PT, write_chessboard_pdf

        data = write_chessboard_pdf(tmp_path / "board.pdf", BoardSpec()).read_bytes()

        assert data.startswith(b"%PDF-1.4")
        assert data.rstrip().endswith(b"%%EOF")
        assert b"/MediaBox [0 0 841.890 595.276]" in data
        # 10 x 7 squares at 25 mm, half of them black.
        square = f"{25.0 * MM_TO_PT:.3f} {25.0 * MM_TO_PT:.3f} re".encode()
        assert data.count(square) == 35

    def test_rejects_a_board_too_big_for_the_page(self, tmp_path):
        from stereo_vision.pattern import write_chessboard_pdf

        with pytest.raises(ValueError, match="margin"):
            write_chessboard_pdf(tmp_path / "big.pdf", BoardSpec(square_size=30.0))
