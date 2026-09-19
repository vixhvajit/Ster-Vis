"""Publish Ster-Vis depth as standard ROS 2 topics, like a ZED or RealSense driver.

Topics, under the node's namespace (default /ster_vis):

  left/image_rect      sensor_msgs/Image       mono8, the image depth lines up with
  left/camera_info     sensor_msgs/CameraInfo
  depth/image          sensor_msgs/Image       32FC1, metres, NaN = unknown (REP 118)
  depth/camera_info    sensor_msgs/CameraInfo
  confidence/image     sensor_msgs/Image       mono8, 0-100 (only with --confidence)
  points               sensor_msgs/PointCloud2 organised, x y z intensity, metres
  scan                 sensor_msgs/LaserScan   for Nav2, SLAM Toolbox and friends
  obstacles/left       sensor_msgs/Range       nearest distance per sector
  obstacles/centre
  obstacles/right

TF (static): <parent_frame> -> ster_vis_link -> ster_vis_left_optical_frame.
ster_vis_link follows REP 103 (x forward, y left, z up); the optical frame
follows the camera convention (x right, y down, z forward).

rclpy comes from a ROS 2 installation, not from pip: source ROS first
(source /opt/ros/<distro>/setup.bash) and install ster-vis into an environment
that can see it (a venv with --system-site-packages).
"""

from __future__ import annotations

import math

import numpy as np

from .outputs import BODY_FRAME, OPTICAL_FRAME, CameraModel, RobotFrame

ROS_MISSING = (
    "rclpy is not available. Install ROS 2 (for example Jazzy on Ubuntu 24.04), then:\n"
    "  source /opt/ros/jazzy/setup.bash\n"
    "and run ster-vis from a Python environment that can see ROS's packages\n"
    "(a venv created with --system-site-packages)."
)

# Rotation from a REP 103 body frame to the camera optical frame, as the
# quaternion (x, y, z, w) every ROS camera driver uses for that link.
BODY_TO_OPTICAL = (-0.5, 0.5, -0.5, 0.5)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class RosPublisher:
    """Owns the ROS node and turns RobotFrames into messages."""

    def __init__(
        self,
        camera: CameraModel,
        namespace: str = "ster_vis",
        parent_frame: str = "base_link",
        mount_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
        mount_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
        confidence: bool = False,
        ros_args: list[str] | None = None,
    ) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import TransformStamped
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField, Range
            from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        except ImportError as error:
            raise RuntimeError(ROS_MISSING) from error

        self.rclpy = rclpy
        self.msg = {"Image": Image, "CameraInfo": CameraInfo, "PointCloud2": PointCloud2,
                    "PointField": PointField, "LaserScan": LaserScan, "Range": Range}
        if not rclpy.ok():
            rclpy.init(args=ros_args)
        self.node = rclpy.create_node("ster_vis", namespace=namespace)
        self.camera = camera

        # Reliable publishers connect to both reliable subscribers (RViz's
        # default) and best-effort ones (Nav2's costmaps).
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        create = self.node.create_publisher
        self.pub = {
            "left": create(Image, "left/image_rect", qos),
            "left_info": create(CameraInfo, "left/camera_info", qos),
            "depth": create(Image, "depth/image", qos),
            "depth_info": create(CameraInfo, "depth/camera_info", qos),
            "points": create(PointCloud2, "points", qos),
            "scan": create(LaserScan, "scan", qos),
        }
        if confidence:
            self.pub["confidence"] = create(Image, "confidence/image", qos)
        self.sectors = ("left", "centre", "right")
        for name in self.sectors:
            self.pub[f"obstacle_{name}"] = create(Range, f"obstacles/{name}", qos)

        self.tf = StaticTransformBroadcaster(self.node)
        transforms = []
        for parent, child, xyz, quaternion in (
            (parent_frame, BODY_FRAME, mount_xyz, quaternion_from_rpy(*mount_rpy)),
            (BODY_FRAME, OPTICAL_FRAME, (0.0, 0.0, 0.0), BODY_TO_OPTICAL),
        ):
            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = quaternion
            transforms.append(t)
        self.tf.sendTransform(transforms)

    def _stamp(self, timestamp: float):
        from builtin_interfaces.msg import Time

        seconds = int(timestamp)
        return Time(sec=seconds, nanosec=int(round((timestamp - seconds) * 1e9)) % 1_000_000_000)

    def _image(self, array: np.ndarray, encoding: str, stamp, frame_id: str):
        msg = self.msg["Image"]()
        msg.header.stamp, msg.header.frame_id = stamp, frame_id
        msg.height, msg.width = array.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = array.strides[0]
        msg.data = np.ascontiguousarray(array).tobytes()
        return msg

    def _camera_info(self, stamp):
        info = self.msg["CameraInfo"]()
        info.header.stamp, info.header.frame_id = stamp, OPTICAL_FRAME
        info.width, info.height = self.camera.width, self.camera.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5  # images are already rectified
        info.k = self.camera.K
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = self.camera.P
        return info

    def _points(self, frame: RobotFrame, stamp):
        field_type = self.msg["PointField"]
        height, width = frame.points.shape[:2]
        cloud = np.empty((height, width, 4), np.float32)
        cloud[..., :3] = frame.points
        cloud[..., 3] = frame.left.astype(np.float32) if frame.left.ndim == 2 else frame.left.mean(axis=2)
        msg = self.msg["PointCloud2"]()
        msg.header.stamp, msg.header.frame_id = stamp, OPTICAL_FRAME
        msg.height, msg.width = height, width  # organised: keeps the pixel grid
        msg.fields = [field_type(name=n, offset=4 * i, datatype=field_type.FLOAT32, count=1)
                      for i, n in enumerate(("x", "y", "z", "intensity"))]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * width
        msg.is_dense = False  # contains NaN points where depth is unknown
        msg.data = cloud.tobytes()
        return msg

    def _scan(self, frame: RobotFrame, stamp):
        scan = frame.scan
        msg = self.msg["LaserScan"]()
        msg.header.stamp, msg.header.frame_id = stamp, BODY_FRAME
        msg.angle_min, msg.angle_max, msg.angle_increment = scan.angle_min, scan.angle_max, scan.angle_increment
        msg.range_min, msg.range_max = scan.range_min, scan.range_max
        msg.ranges = scan.ranges.astype(float).tolist()
        return msg

    def _ranges(self, frame: RobotFrame, stamp):
        field_of_view = self.camera.horizontal_fov_rad / len(self.sectors)
        for name in self.sectors:
            msg = self.msg["Range"]()
            msg.header.stamp, msg.header.frame_id = stamp, BODY_FRAME
            msg.radiation_type = msg.INFRARED  # closest listed type; the ROS message has no "stereo"
            msg.field_of_view = field_of_view
            msg.min_range, msg.max_range = frame.scan.range_min, frame.scan.range_max
            value = frame.obstacles.sectors.get(name)
            msg.range = float("inf") if value is None else float(value)
            yield name, msg

    def publish(self, frame: RobotFrame) -> None:
        stamp = self._stamp(frame.timestamp)
        info = self._camera_info(stamp)
        self.pub["left"].publish(self._image(frame.left, "mono8", stamp, OPTICAL_FRAME))
        self.pub["left_info"].publish(info)
        self.pub["depth"].publish(self._image(frame.depth_m, "32FC1", stamp, OPTICAL_FRAME))
        self.pub["depth_info"].publish(info)
        if "confidence" in self.pub and frame.confidence is not None:
            confidence = np.rint(frame.confidence).astype(np.uint8)
            self.pub["confidence"].publish(self._image(confidence, "mono8", stamp, OPTICAL_FRAME))
        self.pub["points"].publish(self._points(frame, stamp))
        self.pub["scan"].publish(self._scan(frame, stamp))
        for name, msg in self._ranges(frame, stamp):
            self.pub[f"obstacle_{name}"].publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def ok(self) -> bool:
        return self.rclpy.ok()

    def close(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
