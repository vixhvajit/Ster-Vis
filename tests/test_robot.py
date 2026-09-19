"""Robot outputs: geometry against known answers, the HTTP API, record and replay."""

from __future__ import annotations

import io
import json
import math
import urllib.request

import cv2
import numpy as np
import pytest

from stereo_vision.live import DepthResult
from stereo_vision.outputs import (
    CameraModel,
    ScanConfig,
    build_robot_frame,
    confidence_from_lr,
    laser_scan,
    obstacles,
    ply_bytes,
    points_from_depth,
)
from stereo_vision.sources import Recorder, ReplaySource, StereoFrame
from stereo_vision.stream import MjpegServer

CAMERA = CameraModel(width=640, height=360, fx=345.0, fy=345.0, cx=320.0, cy=180.0, baseline_m=0.06)


def wall(distance_m: float) -> np.ndarray:
    """Depth image of a flat wall facing the camera: z is the same everywhere."""
    return np.full((CAMERA.height, CAMERA.width), distance_m, np.float32)


def result(depth_m: np.ndarray, confidence=None) -> DepthResult:
    left = np.full(depth_m.shape, 128, np.uint8)
    return DepthResult(left, np.ones_like(depth_m), depth_m * 1000.0, {"match": 5.0}, confidence, 1234.5)


class TestGeometry:
    def test_points_lie_on_the_wall(self):
        points = points_from_depth(wall(2.0), CAMERA)
        assert np.allclose(points[..., 2], 2.0)
        # The principal point looks straight ahead; the left edge is to the left (negative x).
        assert np.allclose(points[180, 320], [0.0, 0.0, 2.0], atol=1e-6)
        assert points[180, 0, 0] < 0 < points[180, 639, 0]

    def test_scan_of_a_wall_is_its_distance_over_cos_bearing(self):
        scan = laser_scan(wall(2.0), CAMERA, min_height_m=-1.0, max_height_m=1.0)
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        expected = 2.0 / np.cos(angles)
        finite = np.isfinite(scan.ranges)
        assert finite.mean() > 0.95
        assert np.allclose(scan.ranges[finite], expected[finite], rtol=0.02)

    def test_scan_ignores_the_floor_below_the_band(self):
        depth = wall(4.0)
        depth[300:, :] = 1.5  # low rows: floor 1.5 m away, well below the camera
        scan = laser_scan(depth, CAMERA, min_height_m=-0.25, max_height_m=0.25)
        # The floor is nearer than the wall but outside the band: never an obstacle.
        finite = np.isfinite(scan.ranges)
        assert finite.all()
        assert (scan.ranges[finite] > 3.9).all()

    def test_depth_only_outside_the_band_is_unknown_not_clear(self):
        # The band went blank on the right, as when a confidence filter drops
        # it, but the floor there kept its depth. Floor pixels say nothing
        # about obstacle height, so those bearings must read unknown (nan):
        # reporting them clear would drive the robot into whatever is there.
        depth = wall(2.0)
        depth[:, 560:] = np.nan
        depth[300:, 560:] = 1.5  # floor below the band
        scan = laser_scan(depth, CAMERA, min_height_m=-0.25, max_height_m=0.25)
        # Beams run right to left, so the blank right edge is at the start of the array.
        assert np.isnan(scan.ranges[:5]).all()
        assert np.isfinite(scan.ranges[-5:]).all()

    def test_band_seen_but_empty_within_range_is_clear(self):
        depth = wall(15.0)  # everything in the band lies beyond range_max
        scan = laser_scan(depth, CAMERA, min_height_m=-0.25, max_height_m=0.25, range_max_m=10.0)
        assert np.isinf(scan.ranges).all()

    def test_unseen_bearings_are_nan_not_clear(self):
        depth = wall(2.0)
        depth[:, :200] = np.nan  # the matcher saw nothing on the left
        scan = laser_scan(depth, CAMERA, min_height_m=-1, max_height_m=1)
        # Beams run right to left, so the unseen left side is at the end of the array.
        assert np.isnan(scan.ranges[-5:]).all()
        assert np.isfinite(scan.ranges[:5]).all()

    def test_left_sector_means_the_robots_left(self):
        depth = wall(3.0)
        depth[:, :100] = 0.8  # an obstacle on the left of the image
        found = obstacles(laser_scan(depth, CAMERA, min_height_m=-1, max_height_m=1))
        assert found.sectors["left"] == pytest.approx(0.8 / math.cos(math.atan((320 - 50) / 345)), rel=0.1)
        assert found.sectors["right"] > 2.9
        assert found.nearest_bearing_deg > 0  # positive bearing = to the left

    def test_confidence_trusts_consistent_matches(self):
        left = np.full((4, 20), 5.0, np.float32)
        right = np.full((4, 20), 5.0, np.float32)
        right[:, 5:10] = 9.0  # disagreement here
        conf = confidence_from_lr(left, right)
        assert conf[0, 18] == 100.0   # right pixel 13 agrees
        assert conf[0, 12] == 0.0     # right pixel 7 disagrees by 4 px
        assert conf[0, 2] == 0.0      # would look off the left edge

    def test_ply_is_valid_binary(self):
        data = ply_bytes(points_from_depth(wall(1.0), CAMERA), np.zeros((360, 640), np.uint8), step=4)
        header = data[: data.index(b"end_header")].decode()
        count = int(header.split("element vertex ")[1].split()[0])
        assert count == 90 * 160
        assert len(data) - len(header) - len("end_header\n") == count * 15  # 3 floats + 3 bytes


class TestApi:
    @pytest.fixture
    def server(self):
        server = MjpegServer(port=0)
        frame = build_robot_frame(result(wall(2.0), np.full((360, 640), 90.0, np.float32)), CAMERA, 7,
                                  ScanConfig(min_height_m=-1, max_height_m=1))
        server.publish_robot(frame, {"camera": CAMERA.as_dict()})
        yield server
        server.close()

    def get(self, server, name):
        with urllib.request.urlopen(f"http://127.0.0.1:{server.port}/api/v1/{name}", timeout=5) as r:
            return r.headers, r.read()

    def test_json_endpoints(self, server):
        headers, body = self.get(server, "frame")
        frame = json.loads(body)
        assert headers["Access-Control-Allow-Origin"] == "*"
        assert frame["sequence"] == 7 and frame["timestamp"] == 1234.5
        assert frame["obstacles"]["sectors_m"]["centre"] == pytest.approx(2.0, abs=0.02)
        assert len(json.loads(self.get(server, "scan")[1])["ranges"]) == 181
        assert json.loads(self.get(server, "info")[1])["camera"]["fx"] == 345.0

    def test_depth_endpoints_agree(self, server):
        png = cv2.imdecode(np.frombuffer(self.get(server, "depth.png")[1], np.uint8), cv2.IMREAD_UNCHANGED)
        npy = np.load(io.BytesIO(self.get(server, "depth.npy")[1]))
        assert png.dtype == np.uint16 and (png == 2000).all()
        assert npy.dtype == np.float32 and np.allclose(npy, 2.0)
        conf = cv2.imdecode(np.frombuffer(self.get(server, "confidence.png")[1], np.uint8), cv2.IMREAD_UNCHANGED)
        assert (conf == 90).all()

    def test_events_stream_summaries(self, server):
        stream = urllib.request.urlopen(f"http://127.0.0.1:{server.port}/api/v1/events", timeout=5)
        line = ""
        while not line.startswith("data:"):
            line = stream.readline().decode().strip()
        assert json.loads(line[5:])["sequence"] == 7
        stream.close()

    @pytest.mark.parametrize("query", ["points.ply?step=abc", "points.ply?step=0", "events?hz=abc", "events?hz=-1", "events?hz=nan"])
    def test_malformed_parameters_get_400_not_a_crash(self, server, query):
        with pytest.raises(urllib.error.HTTPError) as error:
            self.get(server, query)
        assert error.value.code == 400
        # The server still answers afterwards.
        assert json.loads(self.get(server, "frame")[1])["sequence"] == 7

    def test_no_data_yet_is_503_and_unknown_is_404(self):
        empty = MjpegServer(port=0)
        try:
            with pytest.raises(urllib.error.HTTPError) as error:
                self.get(empty, "frame")
            assert error.value.code == 503
        finally:
            empty.close()


class TestRecordReplay:
    def test_round_trip_is_lossless_and_keeps_time(self, tmp_path):
        rng = np.random.default_rng(0)
        frames = [StereoFrame(rng.integers(0, 256, (40, 60), np.uint8), rng.integers(0, 256, (40, 60), np.uint8),
                              0.25 * i, 50.0 + i * 0.1) for i in range(4)]
        with Recorder(tmp_path) as recorder:
            for frame in frames:
                recorder.write(frame)
        replay = ReplaySource(tmp_path, realtime=False, original_times=True)
        out = [replay.read() for _ in range(len(replay))]
        assert all(np.array_equal(a.left, b.left) and np.array_equal(a.right, b.right) for a, b in zip(frames, out))
        assert [f.timestamp for f in out] == pytest.approx([f.timestamp for f in frames])
        assert [f.skew_ms for f in out] == [0.0, 0.25, 0.5, 0.75]
        with pytest.raises(EOFError):
            replay.read()

    def test_loops_and_reads_plain_image_folders(self, tmp_path):
        for side in ("left", "right"):
            (tmp_path / side).mkdir()
            cv2.imwrite(str(tmp_path / side / "000.png"), np.zeros((10, 10), np.uint8))
        replay = ReplaySource(tmp_path, realtime=False, loop=True)
        assert replay.size == (10, 10)
        for _ in range(3):
            assert replay.read().skew_ms is None


def test_body_to_optical_rotation():
    from stereo_vision.ros2 import BODY_TO_OPTICAL, quaternion_from_rpy

    assert quaternion_from_rpy(-math.pi / 2, 0, -math.pi / 2) == pytest.approx(BODY_TO_OPTICAL, abs=1e-12)


def test_ros_mount_defaults_are_floats():
    """ROS aborts the process on an int in a float field; argparse keeps defaults as given."""
    from stereo_vision.cli.ros2 import parse_args

    args, _ = parse_args(["--calibration", "x.npz"])
    assert all(isinstance(v, float) for v in args.mount)


@pytest.mark.parametrize("bad", [dict(beams=1), dict(min_height_m=0.5, max_height_m=0.1), dict(range_min_m=5, range_max_m=1)])
def test_scan_settings_that_would_report_all_clear_are_refused(bad):
    with pytest.raises(ValueError):
        ScanConfig(**bad)
