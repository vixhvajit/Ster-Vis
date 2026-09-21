"""Fly a stereo drone around an indoor warehouse and map it with Ster-Vis.

This is the loop the drone's Raspberry Pi 5 would run:

    left + right camera  ->  Ster-Vis DepthPipeline (rectify, SGBM, depth)
    ->  laser scan  ->  reactive avoidance  ->  cmd_vel
    ->  point cloud + pose  ->  Ster-Vis PointCloudMap  ->  map of the warehouse

Nothing plans the flight: the drone avoids what the stereo pair sees and maps
whatever it flies past, so coverage is whatever the flight produced.

The pose of every frame comes from the simulator, as it would from a flight
controller's VIO or a motion capture rig on a real drone - Ster-Vis maps, it
does not localise. A second map is built at the same poses from Gazebo's
perfect depth camera, which is the best any mapping could do on this flight,
and the stereo map is scored against it and against the true geometry
(score_map.py).

Run with the server up (start-gz.ps1 warehouse.sdf):

    python fly.py                  # 240 s of sim time, writes output/warehouse/
    python fly.py --show           # also a live window
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
# Ster-Vis straight from this checkout; set STER_VIS_SRC to use another one.
sys.path.insert(0, os.environ.get("STER_VIS_SRC", str(HERE.parents[1] / "src")))

from stereo_vision.calibration import StereoCalibration  # noqa: E402
from stereo_vision.depth import colorize_depth  # noqa: E402
from stereo_vision.live import DepthPipeline  # noqa: E402
from stereo_vision.mapping import MapConfig, PointCloudMap, Pose, rotation_from_quaternion  # noqa: E402
from stereo_vision.outputs import CameraModel, ScanConfig, build_robot_frame, laser_scan  # noqa: E402
from stereo_vision.presets import get_preset  # noqa: E402
from stereo_vision.sources import StereoFrame  # noqa: E402

from avoid import Avoider, label, scan_error, scan_plot, text_panel  # noqa: E402
from score_map import score  # noqa: E402

from gz.msgs10.camera_info_pb2 import CameraInfo  # noqa: E402
from gz.msgs10.contacts_pb2 import Contacts  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
from gz.transport13 import Node  # noqa: E402

TRUTH = json.loads((HERE / "worlds" / "warehouse.json").read_text())
BASELINE_MM = TRUTH["baseline_m"] * 1000.0
CAMERA_IN_BODY = np.array(TRUTH["camera_in_body"], float)
BUILDING = TRUTH["building"]
RIGHT_EDGE_PX = 40  # at the pi5 preset's 320 px width
PANEL_SIZE = (1280, 720)


def stamp_of(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9


class Mailbox:
    """Latest sensor data from gz-transport callbacks, which run on their own threads."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.images: dict[float, dict[str, np.ndarray]] = {}
        self.truth: dict[float, np.ndarray] = {}
        self.info: CameraInfo | None = None
        self.poses: list[tuple[float, np.ndarray, np.ndarray]] = []  # stamp, position, quaternion
        self.touching: set[str] = set()
        self.touch_time = 0.0
        self.chase: np.ndarray | None = None

    def on_chase(self, msg: Image) -> None:
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        with self.lock:
            self.chase = image

    def on_image(self, side: str):
        def callback(msg: Image) -> None:
            image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            with self.lock:
                self.images.setdefault(stamp_of(msg), {})[side] = image
        return callback

    def on_truth(self, msg: Image) -> None:
        depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        with self.lock:
            self.truth[stamp_of(msg)] = depth

    def on_info(self, msg: CameraInfo) -> None:
        with self.lock:
            self.info = msg

    def on_poses(self, msg: Pose_V) -> None:
        """True drone pose from the world: the map's pose source, and the plots'."""
        stamp = stamp_of(msg)
        for pose in msg.pose:
            if pose.name != "drone":
                continue
            position = np.array([pose.position.x, pose.position.y, pose.position.z])
            q = pose.orientation
            with self.lock:
                self.poses.append((stamp or time.time(), position,
                                   np.array([q.x, q.y, q.z, q.w])))
                del self.poses[:-200]
            return

    def on_contacts(self, msg: Contacts) -> None:
        names = set()
        for contact in msg.contact:
            for name in (contact.collision1.name, contact.collision2.name):
                if "drone" not in name and "ground_plane" not in name:
                    names.add(name.split("::")[0])
        with self.lock:
            self.touching |= names
            self.touch_time = time.time()

    def newest_pair(self, after: float):
        """Newest stamp with both images, newer than ``after``; drops older ones."""
        with self.lock:
            complete = [s for s, v in self.images.items() if "left" in v and "right" in v and s > after]
            if not complete:
                return None
            stamp = max(complete)
            pair = self.images[stamp]
            for s in [s for s in self.images if s <= stamp]:
                del self.images[s]
            return stamp, pair["left"], pair["right"]

    def truth_at(self, stamp: float):
        with self.lock:
            depth = self.truth.get(stamp)
            for s in [s for s in self.truth if s < stamp - 1.0]:
                del self.truth[s]
            return depth

    def pose_at(self, stamp: float):
        """The drone pose closest in time to a frame, so the map is not smeared."""
        with self.lock:
            if not self.poses:
                return None
            stamps = np.array([p[0] for p in self.poses])
            return self.poses[int(np.argmin(np.abs(stamps - stamp)))]


def ideal_calibration(info: CameraInfo, baseline_mm: float) -> StereoCalibration:
    """What a perfect chessboard calibration of the simulated rig would return."""
    K = np.array(info.intrinsics.k, np.float64).reshape(3, 3)
    size = (int(info.width), int(info.height))
    dist = np.zeros(5)
    R = np.eye(3)
    # OpenCV's T maps left-camera points into the right camera, which sits
    # the baseline to the right (+x in the optical frame).
    T = np.array([[-baseline_mm], [0.0], [0.0]])
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K, dist, K, dist, size, R, T,
                                                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    return StereoCalibration(size, K, dist, K.copy(), dist.copy(), R, T, R1, R2, P1, P2, Q,
                             rms=0.0, coverage_pct=100.0)


def yaw_of(quaternion: np.ndarray) -> float:
    x, y, z, w = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def camera_pose(position: np.ndarray, quaternion: np.ndarray) -> Pose:
    """Where the left camera was, in the world, from the drone's pose."""
    rotation = rotation_from_quaternion(quaternion)
    return Pose.from_body(position + rotation @ CAMERA_IN_BODY, rotation=rotation)


class MapView:
    """The map so far, seen from above, with the flight path on it."""

    def __init__(self, size: int = 450) -> None:
        self.size = size
        span = 2 * max(BUILDING["half_x"], BUILDING["half_y"]) + 1.0
        self.scale = size / span
        self.trail: list[tuple[int, int]] = []
        self.cached = np.full((size, size, 3), 250, np.uint8)

    def px(self, x: float, y: float) -> tuple[int, int]:
        return (int(round(self.size / 2 + x * self.scale)), int(round(self.size / 2 - y * self.scale)))

    def refresh(self, area: PointCloudMap) -> None:
        """Redraw the mapped points; heights are coloured, so racking reads as racking."""
        view = np.full((self.size, self.size, 3), 250, np.uint8)
        points = area.points(min_hits=1)
        if len(points):
            columns = np.clip((self.size / 2 + points[:, 0] * self.scale).astype(int), 0, self.size - 1)
            rows = np.clip((self.size / 2 - points[:, 1] * self.scale).astype(int), 0, self.size - 1)
            height = np.clip(points[:, 2] / BUILDING["height"], 0, 1)
            colour = cv2.applyColorMap((height * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)[:, 0, :]
            view[rows, columns] = colour
        self.cached = view

    def draw(self, pose) -> np.ndarray:
        view = self.cached.copy()
        half_x, half_y = BUILDING["half_x"], BUILDING["half_y"]
        cv2.rectangle(view, self.px(-half_x, half_y), self.px(half_x, -half_y), (150, 150, 150), 1)
        if len(self.trail) > 1:
            cv2.polylines(view, [np.array(self.trail, np.int32)], False, (200, 120, 40), 1)
        if pose is not None:
            position, quaternion = pose[1], pose[2]
            self.trail.append(self.px(position[0], position[1]))
            del self.trail[:-4000]
            yaw = yaw_of(quaternion)
            nose = self.px(position[0] + 0.7 * math.cos(yaw), position[1] + 0.7 * math.sin(yaw))
            cv2.circle(view, self.px(position[0], position[1]), 5, (40, 40, 220), -1)
            cv2.line(view, self.px(position[0], position[1]), nose, (40, 40, 220), 2)
        return view


def compose(chase, left, depth_mm, scan, truth_scan, map_view, command, status) -> np.ndarray:
    """One 1280x720 frame: the flight and the map on top, what the drone sees below."""
    if chase is None:
        chase_view = np.full((450, 800, 3), 60, np.uint8)
        cv2.putText(chase_view, "waiting for /chase/image", (250, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (220, 220, 220), 1, cv2.LINE_AA)
    else:
        chase_view = cv2.resize(cv2.cvtColor(chase, cv2.COLOR_RGB2BGR), (800, 450),
                                interpolation=cv2.INTER_AREA)
    label(chase_view, "Gazebo warehouse - chase camera", (10, 24), 0.55)
    badge = f"{command[0].upper()}  v {command[1]:.2f} m/s  w {command[2]:+.2f} rad/s  z {command[3]:+.2f} m/s"
    (bw, _), _ = cv2.getTextSize(badge, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    colour = {"cruise": (60, 170, 60), "turn": (0, 140, 255), "climb": (200, 130, 40)}.get(command[0],
                                                                                          (90, 90, 90))
    cv2.rectangle(chase_view, (790 - bw - 16, 8), (790, 38), colour, -1)
    cv2.putText(chase_view, badge, (790 - bw - 8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 2, cv2.LINE_AA)

    map_panel = np.full((450, 480, 3), 250, np.uint8)
    map_panel[:, 15:465] = map_view
    label(map_panel, "Ster-Vis map so far, from above (colour = height)", (22, 24), 0.42)

    left_view = cv2.resize(cv2.cvtColor(left, cv2.COLOR_GRAY2BGR), (360, 270))
    depth_view = cv2.resize(colorize_depth(depth_mm, 300, 6000), (360, 270),
                            interpolation=cv2.INTER_NEAREST)
    label(left_view, "left camera, rectified (Ster-Vis input)", (8, 20), 0.42)
    label(depth_view, "Ster-Vis depth  red 0.3 m ... blue 6 m", (8, 20), 0.42)
    scan_view = scan_plot(scan, truth_scan, size=(280, 270), max_r=6.0)
    top = cv2.hconcat([chase_view, map_panel])
    bottom = cv2.hconcat([left_view, depth_view, scan_view, text_panel(status, size=(280, 270))])
    return cv2.vconcat([top, bottom])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=300.0, help="sim seconds to fly")
    parser.add_argument("--preset", default="pi5", help="Ster-Vis preset (pi5 = half resolution)")
    parser.add_argument("--min-distance", type=float, default=0.45, help="closest range to match, m")
    parser.add_argument("--min-confidence", type=float, default=50.0,
                        help="drop depth whose left-right check scores below this, 0-100 (0 = off)")
    # Up only, by default. The cameras look forward, so the drone cannot see
    # what is below it: a flight that came back down from 2.1 m landed on a
    # pallet stack it had flown over. A real drone descends on a downward
    # rangefinder, or on its map.
    parser.add_argument("--altitudes", type=float, nargs="+", default=[1.3, 2.1],
                        help="altitudes to fly at, in turn (default 1.3 2.1)")
    parser.add_argument("--level-seconds", type=float, default=150.0,
                        help="sim seconds at each altitude (default 150)")
    parser.add_argument("--pi-fps", type=float, default=0.0, help="cap processed pairs per sim second")
    parser.add_argument("--voxel", type=float, default=0.05, help="map resolution, m")
    # Depth error grows with the square of distance: at this rig's 12 cm
    # baseline and 234 px focal length, half a pixel of disparity is 3 cm at
    # 2 m but 28 cm at 4 m, so mapping further than 4 m adds smear, not detail.
    parser.add_argument("--map-max-range", type=float, default=4.0, help="ignore depth beyond this, m")
    parser.add_argument("--step", type=int, default=2, help="map every n-th pixel each way")
    parser.add_argument("--keyframe-distance", type=float, default=0.15, help="metres")
    parser.add_argument("--keyframe-angle", type=float, default=10.0, help="degrees")
    parser.add_argument("--min-hits", type=int, default=2, help="keep voxels seen this many times")
    parser.add_argument("--record", type=int, default=0,
                        help="save every Nth pair + true depth for offline work")
    parser.add_argument("--record-pairs", type=Path, default=None, metavar="DIR",
                        help="save every keyframe pair, its pose and the calibration here, so "
                             "`ster-vis map` can rebuild the map offline at other settings")
    parser.add_argument("--show", action="store_true", help="live window")
    parser.add_argument("--out", default=str(HERE / "output" / "warehouse"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    box = Mailbox()
    node = Node()
    node.subscribe(Image, "/stereo/left/image", box.on_image("left"))
    node.subscribe(Image, "/stereo/right/image", box.on_image("right"))
    node.subscribe(Image, "/stereo/truth/depth", box.on_truth)
    node.subscribe(CameraInfo, "/stereo/left/camera_info", box.on_info)
    node.subscribe(Image, "/chase/image", box.on_chase)
    node.subscribe(Pose_V, "/world/warehouse/dynamic_pose/info", box.on_poses)
    node.subscribe(Pose_V, "/world/warehouse/pose/info", box.on_poses)
    node.subscribe(Contacts,
                   "/world/warehouse/model/drone/link/base_link/sensor/bumper/contact",
                   box.on_contacts)
    cmd_pub = node.advertise("/model/drone/cmd_vel", Twist)

    print("waiting for camera_info ...", flush=True)
    deadline = time.time() + 60
    while box.info is None:
        if time.time() > deadline:
            sys.exit("no /stereo/left/camera_info - is the server running "
                     "(start-gz.ps1 warehouse.sdf)?")
        time.sleep(0.1)

    calibration = ideal_calibration(box.info, BASELINE_MM)
    pipeline = DepthPipeline(calibration, get_preset(args.preset),
                             min_distance_mm=args.min_distance * 1000,
                             confidence=args.min_confidence > 0)
    camera = CameraModel.from_calibration(pipeline.calibration)
    K = np.array(box.info.intrinsics.k).reshape(3, 3)
    truth_camera = CameraModel(int(box.info.width), int(box.info.height), K[0, 0], K[1, 1],
                               K[0, 2], K[1, 2], BASELINE_MM / 1000)
    # The drone can hit anything within about a third of a metre of its own
    # height, so that is the band the scan watches; the floor and the roof are
    # outside it at every altitude it flies.
    scan_config = ScanConfig(beams=121, min_height_m=-0.35, max_height_m=0.35,
                             range_min_m=0.25, range_max_m=6.0)
    map_config = MapConfig(voxel_m=args.voxel, min_range_m=0.4, max_range_m=args.map_max_range,
                           step=args.step, keyframe_distance_m=args.keyframe_distance,
                           keyframe_angle_deg=args.keyframe_angle, min_hits=args.min_hits)
    area = PointCloudMap(map_config)
    ideal_area = PointCloudMap(map_config)   # the same map, from the perfect depth camera
    print(f"Ster-Vis {args.preset}: rectified {camera.width}x{camera.height}, f {camera.fx:.1f}px, "
          f"baseline {BASELINE_MM:.0f} mm, closest {pipeline.closest_mm / 1000:.2f} m; "
          f"mapping at {args.voxel * 100:.0f} cm out to {args.map_max_range} m", flush=True)

    recorder = poses_file = None
    if args.record_pairs is not None:
        from stereo_vision.sources import Recorder

        recorder = Recorder(args.record_pairs)
        calibration.save(args.record_pairs / "rig.npz")
        poses_file = open(args.record_pairs / "poses.csv", "w", newline="", encoding="utf-8")
        # The pose of the camera's own body frame, so `ster-vis map` needs no
        # knowledge of where the head sits on the drone.
        poses_file.write("timestamp,x,y,z,qx,qy,qz,qw\n")

    avoider = Avoider(half_width=0.5, stop=1.1, slow=2.4, go=1.8, v_max=0.6, w_turn=0.8,
                      influence=1.6, gain=0.09)
    view = MapView(size=450)
    fps = args.pi_fps or 10
    video = cv2.VideoWriter(str(out / "run.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, PANEL_SIZE)
    log_file = open(out / "log.csv", "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["sim_t", "x", "y", "z", "yaw", "state", "v", "w", "vz", "front_m", "target_z",
                  "valid_pct", "total_ms", "map_points", "keyframes"])

    errors, missed, blind, close_beams, frame_ms = [], 0, 0, 0, []
    collisions, touched_names, was_touching = 0, set(), False
    last_stamp, last_processed, start_stamp, seq, sim_t = -1.0, -1e9, None, 0, 0.0
    distance, last_xy = 0.0, None
    try:
        while True:
            pair = box.newest_pair(last_stamp)
            if pair is None:
                time.sleep(0.005)
                continue
            stamp, left_rgb, right_rgb = pair
            last_stamp = stamp
            start_stamp = stamp if start_stamp is None else start_stamp
            sim_t = stamp - start_stamp
            if sim_t > args.duration:
                break
            if args.pi_fps and stamp - last_processed < 1.0 / args.pi_fps - 1e-6:
                continue
            last_processed = stamp

            start = time.perf_counter()
            frame = StereoFrame(cv2.cvtColor(left_rgb, cv2.COLOR_RGB2GRAY),
                                cv2.cvtColor(right_rgb, cv2.COLOR_RGB2GRAY), timestamp=stamp)
            result = pipeline.process(frame)
            if result.confidence is not None:
                # Untextured surfaces give SGBM false matches; matching
                # right-to-left disagrees on them, so drop those pixels. The
                # right matcher has no data in the last columns, so keep the
                # left matcher's depth there.
                low = result.confidence < args.min_confidence
                low[:, -round(RIGHT_EDGE_PX * camera.width / 320):] = False
                result.depth_mm[low] = np.nan
            robot = build_robot_frame(result, camera, seq, scan_config)

            # Fly: avoid on what the stereo pair sees, hold the current level.
            pose = box.pose_at(stamp)
            target_z = args.altitudes[int(sim_t // args.level_seconds) % len(args.altitudes)]
            command = avoider.step(robot.scan)
            altitude = float(pose[1][2]) if pose is not None else target_z
            climb = float(np.clip(1.2 * (target_z - altitude), -0.7, 0.7))
            state = command.state
            if abs(altitude - target_z) > 0.35:
                # Taking off, or changing level: hold position while climbing.
                # An earlier version crept forward through the climb and put
                # the drone into a rack post it had already seen.
                state = "climb"
            twist = Twist()
            twist.linear.x = 0.0 if state == "climb" else command.v
            twist.linear.z = climb
            twist.angular.z = 0.0 if state == "climb" else command.w
            cmd_pub.publish(twist)

            # Map: fuse this frame at the pose the simulator reports.
            keyframe = False
            if pose is not None:
                where = camera_pose(pose[1], pose[2])
                keyframe = area.add_frame(robot, where)
                if keyframe and recorder is not None:
                    recorder.write(StereoFrame(frame.left, frame.right, timestamp=stamp))
                    poses_file.write(f"{stamp:.6f}," + ",".join(
                        f"{v:.5f}" for v in (*where.translation, *pose[2])) + "\n")
            total_ms = (time.perf_counter() - start) * 1000
            frame_ms.append(total_ms)
            seq += 1

            truth_depth = box.truth_at(stamp)
            truth_scan = None
            if truth_depth is not None:
                metric = np.where(np.isfinite(truth_depth), truth_depth, np.inf).astype(np.float32)
                truth_scan = laser_scan(metric, truth_camera, scan_config.beams,
                                        scan_config.min_height_m, scan_config.max_height_m,
                                        scan_config.range_min_m, scan_config.range_max_m)
                error = scan_error(robot.scan, truth_scan)
                errors += error["abs_err"]
                missed, blind = missed + error["missed"], blind + error["blind"]
                close_beams += error["close"]
                if keyframe and pose is not None:
                    ideal_area.add_depth(np.where(np.isfinite(truth_depth), truth_depth, np.nan),
                                         truth_camera, camera_pose(pose[1], pose[2]), force=True)
            if args.record and seq % args.record == 0 and truth_depth is not None:
                (out / "frames").mkdir(exist_ok=True)
                np.savez_compressed(out / "frames" / f"{seq:05d}.npz", left=left_rgb, right=right_rgb,
                                    truth=truth_depth, k=np.array(box.info.intrinsics.k))

            with box.lock:
                chase = box.chase
                touching = bool(box.touching) and time.time() - box.touch_time < 0.3
                if altitude > 0.5:  # not the floor it is standing on before take-off
                    touched_names |= box.touching
                else:
                    touching = False
                box.touching = set() if not touching else box.touching
            if touching and not was_touching:
                collisions += 1
                print(f"  t={sim_t:6.1f}s  CONTACT with {sorted(touched_names)}", flush=True)
            was_touching = touching
            if pose is not None and last_xy is not None:
                distance += float(np.linalg.norm(pose[1][:2] - last_xy))
            last_xy = pose[1][:2] if pose is not None else last_xy

            summary = robot.summary()
            position = pose[1] if pose is not None else np.zeros(3)
            yaw_deg = math.degrees(yaw_of(pose[2])) if pose is not None else 0.0
            log.writerow([round(sim_t, 2), *(round(float(v), 3) for v in position), round(yaw_deg, 1),
                          state, round(twist.linear.x, 3), round(command.w, 3), round(climb, 3),
                          round(command.front_m, 3), target_z, summary["valid_pct"],
                          round(total_ms, 1), len(area), area.frames_integrated])

            if seq % 5 == 0:
                view.refresh(area)
            status = [
                f"sim time   {sim_t:6.1f} s",
                f"state      {state}",
                f"corridor   {command.front_m:4.2f} m ahead",
                f"altitude   {altitude:4.2f} m (target {target_z:.1f})",
                f"map        {len(area):,} voxels",
                f"keyframes  {area.frames_integrated}",
                f"stereo     {total_ms:4.0f} ms/pair",
                f"contacts   {collisions}",
                f"flown      {distance:5.1f} m",
            ]
            panel = compose(chase, result.left, result.depth_mm, robot.scan, truth_scan,
                            view.draw(pose), (state, twist.linear.x, command.w, climb), status)
            video.write(panel)
            if args.show:
                cv2.imshow("warehouse mapping", panel)
                if cv2.waitKey(1) == 27:
                    break
            if seq % 50 == 0:
                print(f"  t={sim_t:6.1f}s  {state:6s} front {command.front_m:4.2f} m  "
                      f"z {altitude:4.2f} m  map {len(area):,} voxels  "
                      f"{total_ms:5.1f} ms/pair  contacts {collisions}", flush=True)
    finally:
        cmd_pub.publish(Twist())
        video.release()
        log_file.close()
        if recorder is not None:
            recorder.close()
            poses_file.close()
        if args.show:
            cv2.destroyAllWindows()

    print("\nsaving the map ...", flush=True)
    written = area.save(out, cell_m=0.1, floor_plan_band=(0.3, 3.5))
    (out / "truth_map.ply").write_bytes(ideal_area.ply_bytes())
    view.refresh(area)
    cv2.imwrite(str(out / "map_top_down.png"), view.draw(box.pose_at(last_stamp)))

    print("scoring the map against the warehouse ...", flush=True)
    reconstruction = score(area.points(), ideal_area.points(), TRUTH["boxes"])
    error = np.array(errors) if errors else np.array([np.nan])
    summary = {
        "sim_seconds": round(float(sim_t), 1),
        "pairs_processed": seq,
        "distance_m": round(distance, 2),
        "contacts": collisions,
        "touched": sorted(touched_names),
        "stereo_ms_per_pair_median_this_pc": round(float(np.median(frame_ms)), 1),
        "scan_abs_error_m": {"median": round(float(np.nanmedian(error)), 3),
                             "p90": round(float(np.nanpercentile(error, 90)), 3)},
        "close_beams_(<2m)": close_beams,
        "missed_close_beams_pct": round(100 * missed / max(close_beams, 1), 2),
        "blind_close_beams_pct": round(100 * blind / max(close_beams, 1), 2),
        "map": area.stats(),
        "reconstruction": reconstruction,
        "settings": {"preset": args.preset, "voxel_m": args.voxel, "step": args.step,
                     "map_max_range_m": args.map_max_range, "min_hits": args.min_hits,
                     "min_confidence": args.min_confidence, "altitudes": args.altitudes},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"\nmap: {written['ply']}")


if __name__ == "__main__":
    main()
