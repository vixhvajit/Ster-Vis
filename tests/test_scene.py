"""Dense depth against a ray-traced scene, plus the depth file and viewer helpers.

The scene tests run the real pipeline (calibrate, rectify, SGBM, triangulate)
on rendered images and compare every pixel with the exact ray-traced depth.
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

from stereo_vision.calibration import (
    calibrate_stereo,
    frame_coverage,
    load_calibration,
    rectify_pair,
)
from stereo_vision.config import BoardSpec, SGBMParams
from stereo_vision.depth import colorize_depth, disparity_to_depth, load_depth, save_depth
from stereo_vision.disparity import compute_disparity, valid_mask
from stereo_vision.scene import (
    Plane,
    _intersect_plane,
    default_scene,
    render_view,
    true_rectified_depth,
)
from stereo_vision.synthetic import default_rig, render_pairs, true_calibration

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))


@pytest.fixture(scope="module")
def world():
    rig, scene = default_rig(), default_scene()
    left = render_view(scene, rig, "left", supersample=1, rng=np.random.default_rng(10))
    right = render_view(scene, rig, "right", supersample=1, rng=np.random.default_rng(11))
    return rig, scene, left, right


@pytest.fixture(scope="module")
def estimated_calibration():
    rig, board = default_rig(), BoardSpec()
    lefts, rights, _ = render_pairs(rig, board, count=25, seed=0)
    calibration, _ = calibrate_stereo(lefts, rights, board)
    return calibration


def measure(world, calibration):
    rig, scene, left_raw, right_raw = world
    left, right = rectify_pair(left_raw, right_raw, calibration.rectification_maps())
    params = SGBMParams(num_disparities=96)
    disparity = compute_disparity(left, right, params)
    disparity[~valid_mask(disparity, params)] = 0
    depth = disparity_to_depth(disparity, calibration.focal_length_px, calibration.baseline)
    truth, ids, visible = true_rectified_depth(scene, rig, calibration)
    scored = np.isfinite(depth) & visible
    error = np.abs(depth - truth) / truth * 100.0
    per_object = {
        p.name: float(np.median(error[scored & (ids == n)]))
        for n, p in enumerate(scene.primitives, start=1)
        if (scored & (ids == n)).sum() > 50
    }
    return 100.0 * scored.sum() / visible.sum(), float(np.median(error[scored])), per_object


def test_matcher_and_triangulation_are_accurate_with_a_perfect_calibration(world):
    """Isolates SGBM and triangulation: every surface within 1%."""
    coverage, median, per_object = measure(world, true_calibration(world[0]))
    assert coverage > 90.0
    assert median < 1.0
    assert all(error < 1.0 for error in per_object.values()), per_object


def test_full_pipeline_from_chessboard_calibration(world, estimated_calibration):
    """Everything real: calibration estimated from rendered boards, then depth."""
    coverage, median, per_object = measure(world, estimated_calibration)

    assert coverage > 88.0
    assert median < 2.5
    # Near objects are dominated by matching and stay tight; the far wall
    # carries the residual calibration error, which grows with distance.
    assert per_object["tilted panel"] < 1.0
    assert per_object["sphere"] < 1.0
    assert per_object["back wall"] < 5.0


def test_planes_ignore_hits_behind_the_ray_origin():
    """Regression: an unbounded plane behind the camera must not count as a hit."""
    floor = Plane("floor", (0, 400, 0), (0, -1, 0))
    rays = np.array([[0.0, 1.0, 1.0], [0.0, -1.0, 1.0]])  # towards the floor, away from it
    t, _ = _intersect_plane(floor, np.zeros(3), rays)
    assert t[0] == pytest.approx(400.0)
    assert np.isinf(t[1])


class TestCoverage:
    def test_counts_grid_cells_reached_by_corners(self):
        points = [np.array([[[10.0, 10.0]], [[630.0, 470.0]]], np.float32)]
        percent, covered = frame_coverage(points, (640, 480), grid=(4, 2))
        assert covered.sum() == 2
        assert percent == pytest.approx(25.0)

    def test_edge_covering_boards_reach_most_of_the_frame(self, estimated_calibration):
        assert estimated_calibration.coverage_pct >= 65.0

    def test_old_calibration_files_still_load(self, tmp_path):
        path = true_calibration(default_rig()).save(tmp_path / "old.npz")
        with np.load(path) as data:
            legacy = {k: data[k] for k in data.files if k != "coverage_pct"}
        np.savez(path, **legacy)
        assert np.isnan(load_calibration(path).coverage_pct)


class TestDepthFiles:
    @pytest.mark.parametrize("suffix", [".png", ".npy"])
    def test_round_trip_keeps_values_and_gaps(self, tmp_path, suffix):
        depth = np.array([[750.0, np.nan], [0.0, 2150.0]], np.float32)
        loaded = load_depth(save_depth(tmp_path / f"d{suffix}", depth))
        assert loaded[0, 0] == pytest.approx(750.0)
        assert loaded[1, 1] == pytest.approx(2150.0)
        assert np.isnan(loaded[0, 1]) and np.isnan(loaded[1, 0])

    def test_png_is_16_bit_millimetres(self, tmp_path):
        path = save_depth(tmp_path / "d.png", np.array([[1234.4, 70000.0]], np.float32))
        raw = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        assert raw.dtype == np.uint16
        assert raw.tolist() == [[1234, 65535]]

    def test_rejects_a_colour_png(self, tmp_path):
        path = tmp_path / "colour.png"
        cv2.imwrite(str(path), np.zeros((2, 2, 3), np.uint8))
        with pytest.raises(ValueError, match="16-bit"):
            load_depth(path)

    def test_colorize_blacks_out_missing_depth(self):
        image = colorize_depth(np.array([[np.nan, 1000.0]], np.float32), 500, 1500)
        assert image[0, 0].tolist() == [0, 0, 0]
        assert image[0, 1].any()


class TestViewer:
    def test_click_to_measure_matches_the_scene_geometry(self):
        """Two points on the back wall (z = 2200 mm) measured through back_project."""
        from view_depth import back_project

        calibration = true_calibration(default_rig())
        P1, R1 = calibration.P1, calibration.R1

        def on_wall(u, v):
            ray = R1.T @ np.array([(u - P1[0, 2]) / P1[0, 0], (v - P1[1, 2]) / P1[1, 1], 1.0])
            point = ray * (2200.0 / ray[2])
            return point, (R1 @ point)[2]

        (a_true, a_depth), (b_true, b_depth) = on_wall(500, 150), on_wall(900, 150)
        a = back_project(500, 150, a_depth, P1)
        b = back_project(900, 150, b_depth, P1)
        assert np.linalg.norm(a - b) == pytest.approx(np.linalg.norm(a_true - b_true), rel=1e-6)

    def test_point_clouds_explain_how_to_view_them_without_open3d(
        self, monkeypatch, capsys, tmp_path
    ):
        import builtins

        import view_depth

        real_import = builtins.__import__

        def no_open3d(name, *args, **kwargs):
            if name == "open3d":
                raise ImportError(name)
            return real_import(name, *args, **kwargs)

        monkeypatch.setattr(builtins, "__import__", no_open3d)
        assert view_depth.view_point_cloud(tmp_path / "cloud.ply") == 1
        assert "viewer/index.html" in capsys.readouterr().out
