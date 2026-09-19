"""The ROS 2 node, against real rclpy. Skipped where ROS 2 is not installed.

CI runs these inside the official ros:jazzy container.
"""

from __future__ import annotations

import math
import struct
import subprocess
import sys
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, Range  # noqa: E402

from stereo_vision.live import DepthResult  # noqa: E402
from stereo_vision.outputs import CameraModel, ScanConfig, build_robot_frame  # noqa: E402
from stereo_vision.ros2 import RosPublisher  # noqa: E402

CAMERA = CameraModel(width=64, height=36, fx=34.5, fy=34.5, cx=32.0, cy=18.0, baseline_m=0.06)


def frame(sequence: int):
    depth = np.full((36, 64), 2.0, np.float32)
    depth[:, :10] = 0.8        # an obstacle on the left
    depth[0, 0] = np.nan       # one unknown pixel
    result = DepthResult(np.full((36, 64), 77, np.uint8), np.ones((36, 64), np.float32), depth * 1000,
                         {}, np.full((36, 64), 88.0, np.float32), 1700000000.25)
    return build_robot_frame(result, CAMERA, sequence, ScanConfig(beams=31, min_height_m=-1, max_height_m=1))


@pytest.fixture
def ros():
    publisher = RosPublisher(CAMERA, namespace="ster_vis", parent_frame="base_link",
                             mount_xyz=(0.1, 0.0, 0.3), confidence=True, ros_args=["test"])
    listener = rclpy.create_node("listener")
    yield publisher, listener
    listener.destroy_node()
    publisher.close()


def collect(publisher, listener, wanted: dict, timeout_s: float = 10.0) -> dict:
    received = {}
    for topic, kind in wanted.items():
        listener.create_subscription(kind, topic, lambda msg, t=topic: received.setdefault(t, msg), 10)
    deadline = time.monotonic() + timeout_s
    sequence = 0
    while len(received) < len(wanted) and time.monotonic() < deadline:
        publisher.publish(frame(sequence))
        sequence += 1
        # spin_once runs a single callback, so drain several per publish or the
        # first busy subscription starves the rest.
        for _ in range(2 * len(wanted)):
            rclpy.spin_once(listener, timeout_sec=0.01)
    missing = set(wanted) - set(received)
    assert not missing, f"no message on {missing}"
    return received


def test_publishes_standard_messages(ros):
    publisher, listener = ros
    got = collect(publisher, listener, {
        "/ster_vis/depth/image": Image,
        "/ster_vis/depth/camera_info": CameraInfo,
        "/ster_vis/left/image_rect": Image,
        "/ster_vis/confidence/image": Image,
        "/ster_vis/points": PointCloud2,
        "/ster_vis/scan": LaserScan,
        "/ster_vis/obstacles/left": Range,
        "/ster_vis/obstacles/right": Range,
    })

    depth = got["/ster_vis/depth/image"]
    assert (depth.encoding, depth.width, depth.height) == ("32FC1", 64, 36)
    values = np.frombuffer(bytes(depth.data), np.float32).reshape(36, 64)
    assert np.isnan(values[0, 0]) and values[20, 40] == pytest.approx(2.0)
    assert depth.header.frame_id == "ster_vis_left_optical_frame"
    assert depth.header.stamp.sec == 1700000000 and depth.header.stamp.nanosec == 250_000_000

    info = got["/ster_vis/depth/camera_info"]
    assert list(info.k) == pytest.approx(CAMERA.K) and list(info.p) == pytest.approx(CAMERA.P)

    assert got["/ster_vis/left/image_rect"].encoding == "mono8"
    assert bytes(got["/ster_vis/confidence/image"].data)[0] == 88

    cloud = got["/ster_vis/points"]
    assert (cloud.height, cloud.width, cloud.point_step) == (36, 64, 16)
    assert [f.name for f in cloud.fields] == ["x", "y", "z", "intensity"]
    x, y, z, intensity = struct.unpack_from("<4f", bytes(cloud.data), (20 * 64 + 40) * 16)
    assert z == pytest.approx(2.0) and intensity == 77.0

    scan = got["/ster_vis/scan"]
    assert len(scan.ranges) == 31 and scan.header.frame_id == "ster_vis_link"
    assert got["/ster_vis/obstacles/left"].range < 1.0 < got["/ster_vis/obstacles/right"].range


def test_static_tf_links_the_robot_to_the_camera(ros):
    from tf2_ros import Buffer, TransformListener

    publisher, listener = ros
    buffer = Buffer()
    TransformListener(buffer, listener)
    deadline = time.monotonic() + 10.0
    transform = None
    while transform is None and time.monotonic() < deadline:
        rclpy.spin_once(listener, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform("base_link", "ster_vis_left_optical_frame", rclpy.time.Time())
        except Exception:
            pass
    assert transform is not None, "no TF from base_link to the optical frame"
    t = transform.transform.translation
    assert (t.x, t.y, t.z) == pytest.approx((0.1, 0.0, 0.3))
    q = transform.transform.rotation
    # q and -q are the same rotation; tf2 may return either sign.
    sign = 1.0 if q.w > 0 else -1.0
    assert (sign * q.x, sign * q.y, sign * q.z, sign * q.w) == pytest.approx((-0.5, 0.5, -0.5, 0.5))


def test_cli_node_runs_from_a_recording(tmp_path):
    """The real `ster-vis ros2` command, as a separate process, seen from outside."""
    import cv2

    from stereo_vision.sources import Recorder, StereoFrame
    from stereo_vision.synthetic import default_rig, true_calibration

    rig = default_rig()
    true_calibration(rig).save(tmp_path / "rig.npz")
    rng = np.random.default_rng(0)
    image = cv2.resize(rng.integers(0, 256, (360, 640), np.uint8), (1280, 720))
    with Recorder(tmp_path / "rec") as recorder:
        recorder.write(StereoFrame(image, np.roll(image, -20, axis=1)))

    node = subprocess.Popen(
        [sys.executable, "-m", "stereo_vision", "ros2", "--calibration", str(tmp_path / "rig.npz"),
         "--replay", str(tmp_path / "rec"), "--loop", "--duration", "20", "--namespace", "cli_test"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        if not rclpy.ok():
            rclpy.init()
        listener = rclpy.create_node("cli_listener")
        got = {}
        listener.create_subscription(LaserScan, "/cli_test/scan", lambda m: got.setdefault("scan", m), 10)
        listener.create_subscription(Image, "/cli_test/depth/image", lambda m: got.setdefault("depth", m), 10)
        deadline = time.monotonic() + 30.0
        while len(got) < 2 and time.monotonic() < deadline and node.poll() is None:
            for _ in range(4):
                rclpy.spin_once(listener, timeout_sec=0.05)
        listener.destroy_node()
        assert len(got) == 2, f"received {list(got)}; node output:\n{node.stdout.read() if node.poll() is not None else ''}"
        assert got["depth"].width == 640 and got["depth"].encoding == "32FC1"
        assert len(got["scan"].ranges) == 181
        assert math.isclose(got["scan"].angle_max, -got["scan"].angle_min)
    finally:
        node.terminate()
        node.wait(timeout=10)
        if rclpy.ok():
            rclpy.shutdown()
