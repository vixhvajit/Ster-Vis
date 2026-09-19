"""Raspberry Pi 5 features, tested without a Pi.

A stand-in picamera2 module mimics the calls this project makes: YUV420 arrays
with stride padding, sensor timestamps, and requests that must be released.
"""

from __future__ import annotations

import sys
import time
import types
import urllib.request

import cv2
import numpy as np
import pytest

from stereo_vision.autocapture import AutoTrigger, quick_corners
from stereo_vision.benchmark import (
    Timing,
    raspberry_pi_health,
    render_sweep,
    evaluate_preset,
    time_preset,
)
from stereo_vision.calibration import load_calibration
from stereo_vision.config import BoardSpec, SGBMParams
from stereo_vision.disparity import build_matcher, compute_disparity
from stereo_vision.live import DepthPipeline, RateMeter
from stereo_vision.presets import PRESETS, disparities_for, get_preset
from stereo_vision.sources import Picamera2Source, StereoFrame, StereoSource, ThreadedSource
from stereo_vision.stream import MjpegServer
from stereo_vision.synthetic import default_rig, render_pairs, true_calibration


# ---------------------------------------------------------------- fake camera

class FakeRequest:
    outstanding = 0

    def __init__(self, array, timestamp):
        self.array, self.timestamp, self.released = array, timestamp, False
        FakeRequest.outstanding += 1

    def make_array(self, name):
        assert name == "main" and not self.released
        return self.array.copy()

    def get_metadata(self):
        metadata = {"SensorTimestamp": self.timestamp}
        if hasattr(self, "sync_ready"):
            metadata["SyncReady"] = self.sync_ready
        return metadata

    def release(self):
        assert not self.released, "released twice"
        self.released = True
        FakeRequest.outstanding -= 1


def fake_picamera2(
    left: np.ndarray, right: np.ndarray, lag_every: int = 0, pad: int = 64,
    sync_support: bool = False, sync_after: int = 5, autofocus: bool = False,
):
    """A picamera2 module serving fixed greyscale images as padded YUV420.

    With ``sync_support`` the cameras advertise libcamera's SyncMode control
    and report SyncReady only after ``sync_after`` frames, like the real
    server/client handshake.
    """

    class Picamera2:
        instances = []
        start_order = []

        @staticmethod
        def global_camera_info():
            return [{"Num": 0, "Model": "imx708"}, {"Num": 1, "Model": "imx708"}]

        def __init__(self, camera_num=0):
            self.num, self.frame, self.started, self.closed = camera_num, 0, False, False
            self.camera_config = {}
            Picamera2.instances.append(self)

        @property
        def camera_controls(self):
            controls = {"FrameDurationLimits": (1, 1_000_000, 33_333), "ExposureTime": (1, 1_000_000, 0)}
            if sync_support:
                controls.update({"SyncMode": (0, 2, 0), "SyncFrames": (1, 1000, 100)})
            if autofocus:
                controls.update({"AfMode": (0, 2, 0), "LensPosition": (0.0, 32.0, 1.0)})
            return controls

        def create_video_configuration(self, main=None, controls=None, buffer_count=6, **_):
            return {"main": dict(main), "controls": dict(controls or {}), "buffer_count": buffer_count}

        def configure(self, config):
            self.camera_config = config

        def start(self):
            self.started = True
            Picamera2.start_order.append(self.num)

        def stop(self):
            self.started = False

        def close(self):
            self.closed = True

        def capture_request(self):
            grey = left if self.num == 0 else right
            h, w = grey.shape
            yuv = np.full((h * 3 // 2, w + pad), 128, np.uint8)
            yuv[:h, :w] = grey
            yuv[:h, w:] = 255  # stride padding that must not leak into the image
            lag = 1 if (lag_every and self.num == 1 and self.frame % lag_every == 0) else 0
            timestamp = int((self.frame - lag) * 33_333_333 + (2_000_000 if self.num == 1 else 0))
            self.frame += 1
            request = FakeRequest(yuv, timestamp)
            if self.camera_config["controls"].get("SyncMode"):
                request.sync_ready = self.frame > sync_after
            return request

    module = types.ModuleType("picamera2")
    module.Picamera2 = Picamera2
    return module


@pytest.fixture
def grey_pair():
    rng = np.random.default_rng(3)
    left = rng.integers(0, 256, (720, 1280), dtype=np.uint8)
    return left, np.roll(left, -12, axis=1)


# -------------------------------------------------------------------- sources

class TestPicamera2Source:
    def test_returns_the_grey_plane_without_stride_padding(self, monkeypatch, grey_pair):
        monkeypatch.setitem(sys.modules, "picamera2", fake_picamera2(*grey_pair))
        with Picamera2Source(width=1280, height=720) as source:
            frame = source.read()
        assert frame.left.shape == (720, 1280)
        assert np.array_equal(frame.left, grey_pair[0])
        assert np.array_equal(frame.right, grey_pair[1])

    def test_resynchronises_when_one_camera_lags_a_frame(self, monkeypatch, grey_pair):
        monkeypatch.setitem(sys.modules, "picamera2", fake_picamera2(*grey_pair, lag_every=3))
        with Picamera2Source(width=1280, height=720, fps=30) as source:
            skews = [source.read().skew_ms for _ in range(12)]
        # A lagging frame is 31 ms behind; after resync every pair is within half a frame.
        assert all(abs(s) <= 500 / 30 for s in skews), skews

    def test_releases_every_request_and_closes_cameras(self, monkeypatch, grey_pair):
        module = fake_picamera2(*grey_pair, lag_every=2)
        monkeypatch.setitem(sys.modules, "picamera2", module)
        FakeRequest.outstanding = 0
        with Picamera2Source(width=1280, height=720) as source:
            for _ in range(10):
                source.read()
        assert FakeRequest.outstanding == 0
        assert all(cam.closed and not cam.started for cam in module.Picamera2.instances)

    def test_uses_software_sync_when_libcamera_supports_it(self, monkeypatch, grey_pair):
        module = fake_picamera2(*grey_pair, sync_support=True, sync_after=5)
        monkeypatch.setitem(sys.modules, "picamera2", module)
        with Picamera2Source(width=1280, height=720) as source:
            assert source.synced
            left_cam, right_cam = module.Picamera2.instances
            assert left_cam.camera_config["controls"]["SyncMode"] == 1  # server
            assert right_cam.camera_config["controls"]["SyncMode"] == 2  # client
            assert module.Picamera2.start_order == [1, 0]  # client started first
            # Frames before SyncReady were discarded during start-up.
            assert left_cam.frame > 5 and right_cam.frame > 5
            source.read()

    def test_locks_autofocus_so_the_calibration_stays_valid(self, monkeypatch, grey_pair):
        module = fake_picamera2(*grey_pair, autofocus=True)
        monkeypatch.setitem(sys.modules, "picamera2", module)
        with Picamera2Source(width=1280, height=720, focus_dioptres=0.5) as source:
            assert source.focus_locked
            for camera in module.Picamera2.instances:
                assert camera.camera_config["controls"]["AfMode"] == 0  # manual
                assert camera.camera_config["controls"]["LensPosition"] == 0.5

    def test_fixed_focus_cameras_get_no_focus_controls(self, monkeypatch, grey_pair):
        monkeypatch.setitem(sys.modules, "picamera2", fake_picamera2(*grey_pair))
        with Picamera2Source(width=1280, height=720) as source:
            assert not source.focus_locked
            assert "AfMode" not in source.cameras[0].camera_config["controls"]

    def test_falls_back_to_timestamp_pairing_without_sync_support(self, monkeypatch, grey_pair):
        monkeypatch.setitem(sys.modules, "picamera2", fake_picamera2(*grey_pair))
        with Picamera2Source(width=1280, height=720) as source:
            assert not source.synced
            assert "SyncMode" not in source.cameras[0].camera_config["controls"]

    def test_requiring_sync_without_support_says_how_to_get_it(self, monkeypatch, grey_pair):
        monkeypatch.setitem(sys.modules, "picamera2", fake_picamera2(*grey_pair))
        with pytest.raises(RuntimeError, match="full-upgrade"):
            Picamera2Source(width=1280, height=720, sync=True)

    def test_explains_how_to_install_picamera2(self, monkeypatch):
        monkeypatch.setitem(sys.modules, "picamera2", None)  # import fails
        with pytest.raises(RuntimeError, match="apt install python3-picamera2"):
            Picamera2Source()

    def test_needs_two_cameras(self, monkeypatch, grey_pair):
        module = fake_picamera2(*grey_pair)
        module.Picamera2.global_camera_info = staticmethod(lambda: [{"Num": 0, "Model": "imx708"}])
        monkeypatch.setitem(sys.modules, "picamera2", module)
        with pytest.raises(RuntimeError, match="need two cameras"):
            Picamera2Source()


class CountingSource(StereoSource):
    def __init__(self, fail_after: int | None = None):
        self.size, self.count, self.fail_after, self.closed = (4, 4), 0, fail_after, False

    def read(self):
        time.sleep(0.002)
        self.count += 1
        if self.fail_after is not None and self.count > self.fail_after:
            raise OSError("camera unplugged")
        value = np.full((4, 4), self.count, np.uint8)
        return StereoFrame(value, value)

    def close(self):
        self.closed = True


class TestThreadedSource:
    def test_hands_out_the_newest_frame_and_never_repeats_one(self):
        with ThreadedSource(CountingSource()) as source:
            seen = []
            for _ in range(5):
                seen.append(int(source.read().left[0, 0]))
                time.sleep(0.02)
        assert seen == sorted(set(seen))
        assert seen[-1] - seen[0] > 5  # frames were skipped, not queued

    def test_a_capture_error_reaches_the_reader(self):
        with ThreadedSource(CountingSource(fail_after=3)) as source:
            with pytest.raises(RuntimeError, match="camera unplugged"):
                for _ in range(10):
                    source.read()

    def test_closes_the_underlying_source(self):
        inner = CountingSource()
        ThreadedSource(inner).close()
        assert inner.closed


# ------------------------------------------------------ presets and scaling

class TestPresets:
    def test_disparity_range_reaches_the_closest_distance(self):
        calibration = true_calibration(default_rig())
        n = disparities_for(calibration, 500.0)
        assert n % 16 == 0
        needed = calibration.focal_length_px * calibration.baseline / 500.0
        assert needed <= n < needed + 16

    def test_scaling_keeps_the_closest_distance(self):
        calibration = true_calibration(default_rig())
        full = disparities_for(calibration, 500.0)
        half = disparities_for(calibration.scaled(0.5), 500.0)
        assert half == pytest.approx(full / 2, abs=16)

    def test_every_preset_builds_a_working_matcher(self, grey_pair):
        calibration = true_calibration(default_rig())
        for preset in PRESETS.values():
            scaled = preset.calibration(calibration)
            matcher = build_matcher(preset.params(scaled, 500.0))
            size = scaled.output_size
            left = cv2.resize(grey_pair[0], size)
            assert matcher.compute(left, left).shape == (size[1], size[0])

    def test_unknown_preset_names_the_choices(self):
        with pytest.raises(ValueError, match="pi5"):
            get_preset("turbo")

    def test_block_matching_recovers_a_known_shift(self, grey_pair):
        left, right = (cv2.resize(i, (640, 360), interpolation=cv2.INTER_AREA) for i in grey_pair)
        disparity = compute_disparity(left, right, SGBMParams(num_disparities=32, block_size=9, mode="bm"))
        interior = disparity[40:-40, 80:-80]
        assert np.median(interior[interior > 0]) == pytest.approx(6.0, abs=0.5)

    def test_rejects_unknown_matcher_modes(self):
        with pytest.raises(ValueError, match="mode"):
            SGBMParams(mode="fast")


class TestScaledCalibration:
    def test_scaled_disparity_gives_the_same_3d_point(self):
        calibration = true_calibration(default_rig())
        half = calibration.scaled(0.5)
        full_point = cv2.perspectiveTransform(np.array([[[800.0, 300.0, 40.0]]]), calibration.Q)
        half_point = cv2.perspectiveTransform(np.array([[[400.0, 150.0, 20.0]]]), half.Q)
        assert np.allclose(full_point, half_point)

    def test_maps_rectify_straight_to_the_smaller_size(self):
        half = true_calibration(default_rig()).scaled(0.5)
        (map_x, _), _ = half.rectification_maps(cv2.CV_32FC1)
        assert map_x.shape == (360, 640)
        assert half.focal_length_px == pytest.approx(true_calibration(default_rig()).focal_length_px / 2)

    def test_rectified_size_survives_save_and_load(self, tmp_path):
        half = true_calibration(default_rig()).scaled(0.5)
        assert load_calibration(half.save(tmp_path / "h.npz")).output_size == (640, 360)


# ------------------------------------------------------------- live pipeline

class TestLivePipeline:
    def test_processes_a_frame_and_times_each_stage(self, grey_pair):
        pipeline = DepthPipeline(true_calibration(default_rig()), get_preset("pi5"), 500.0)
        result = pipeline.process(StereoFrame(*grey_pair))
        assert result.depth_mm.shape == (360, 640)
        assert set(result.stage_ms) == {"rectify", "match", "depth"}

    def test_refuses_frames_of_the_wrong_size(self):
        pipeline = DepthPipeline(true_calibration(default_rig()), get_preset("pi5"), 500.0)
        small = np.zeros((480, 640), np.uint8)
        with pytest.raises(ValueError, match="calibrated size|calibration is for"):
            pipeline.check_size(StereoFrame(small, small))

    def test_rate_meter_reports_frames_per_second(self):
        meter = RateMeter(smoothing=1.0)
        for _ in range(3):
            meter.tick({"match": 5.0})
            time.sleep(0.02)
        assert 10 < meter.fps < 60
        assert "match" in meter.line()


class TestStream:
    def test_serves_a_snapshot_and_skips_work_nobody_watches(self):
        server = MjpegServer(port=0, max_fps=1000)
        try:
            assert server.wants_frame()  # nothing published yet
            assert server.publish(np.zeros((40, 60, 3), np.uint8))
            assert not server.wants_frame()  # published, and nobody is watching
            with urllib.request.urlopen(f"http://127.0.0.1:{server.port}/snapshot.jpg", timeout=5) as r:
                data = r.read()
            assert data[:2] == b"\xff\xd8"  # JPEG start marker
            with urllib.request.urlopen(f"http://127.0.0.1:{server.port}/", timeout=5) as r:
                assert b"/stream" in r.read()
        finally:
            server.close()


# --------------------------------------------------------------- auto capture

class TestAutoTrigger:
    corners = np.stack(np.meshgrid(np.arange(9) * 30 + 100.0, np.arange(6) * 30 + 100.0), -1).reshape(-1, 1, 2)

    def feed(self, trigger, corners, frames, start=0.0):
        results = [trigger.update(corners, corners, now=start + i * 0.05) for i in range(frames)]
        return [save for save, _ in results]

    def test_waits_until_the_board_is_held_still(self):
        trigger = AutoTrigger((1280, 720))
        saves = self.feed(trigger, self.corners, 7)
        assert saves.index(True) == 5  # after five still frames

    def test_does_not_save_the_same_pose_twice(self):
        trigger = AutoTrigger((1280, 720), cooldown_s=0.0)
        assert any(self.feed(trigger, self.corners, 7))
        assert not any(self.feed(trigger, self.corners, 20, start=10.0))

    def test_a_new_spot_is_saved(self):
        trigger = AutoTrigger((1280, 720), cooldown_s=0.0)
        self.feed(trigger, self.corners, 7)
        moved = self.corners + np.array([700.0, 300.0])
        assert any(self.feed(trigger, moved, 7, start=10.0))

    def test_no_board_resets_the_stillness_count(self):
        trigger = AutoTrigger((1280, 720))
        self.feed(trigger, self.corners, 4)
        assert trigger.update(None, None) == (False, "board not in both views")
        assert not any(self.feed(trigger, self.corners, 5, start=1.0))

    def test_quick_corners_find_a_rendered_board(self):
        rig, board = default_rig(), BoardSpec()
        lefts, _, _ = render_pairs(rig, board, count=1, seed=0)
        corners = quick_corners(lefts[0], board)
        assert corners is not None and len(corners) == board.corner_count


# ------------------------------------------------------------------ benchmark

class TestBenchmark:
    def test_times_every_stage(self, grey_pair):
        calibration = true_calibration(default_rig())
        timing, scaled, params = time_preset(get_preset("pi5-fast"), calibration, *grey_pair, 500.0, repeats=2)
        assert timing.total_ms > 0 and timing.fps > 0
        assert scaled.output_size == (480, 270)

    def test_scores_accuracy_over_a_sweep(self):
        rig = default_rig()
        frames = render_sweep(rig, distances_mm=(800, 1400))
        result = evaluate_preset(get_preset("pi5"), true_calibration(rig), rig, frames, 500.0)
        assert result.coverage_pct > 85
        assert result.band_error_pct["near"] < 2.0
        assert len(result.per_distance) == 2

    def test_health_check_is_quiet_off_a_pi(self):
        assert raspberry_pi_health() is None or "temp" in raspberry_pi_health()

    def test_fps_follows_total_time(self):
        assert Timing(1.0, 8.0, 1.0).fps == pytest.approx(100.0)
