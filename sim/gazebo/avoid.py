"""Stereo obstacle avoidance in Gazebo, with Ster-Vis doing the seeing.

This is the loop the rover's Raspberry Pi 5 would run:

    left + right camera  ->  Ster-Vis DepthPipeline (rectify, SGBM, depth)
    ->  laser scan + obstacle sectors  ->  reactive controller  ->  cmd_vel

The cameras are Gazebo's, so the calibration is the ideal one built from their
CameraInfo and the known 12 cm baseline (no chessboard step). A perfect depth
camera beside the left camera scores every stereo scan against the truth; the
controller never sees it.

A chase camera, also rendered by Gazebo, follows the rover so the run can be
watched: the live window (--show) and output/run.mp4 put it beside the depth
map, the stereo scan against the truth and a top-down map.

Run with the server up (start-gz.ps1, or `gz sim -r` on Linux):

    python avoid.py                 # 180 s of sim time, writes output/
    python avoid.py --show          # also a live window
    python avoid.py --pi-fps 5      # only process 5 pairs/s, like a slow Pi
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
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

# Ster-Vis straight from this checkout; set STER_VIS_SRC to use another one.
sys.path.insert(0, os.environ.get("STER_VIS_SRC", str(Path(__file__).resolve().parents[2] / "src")))
from stereo_vision.calibration import StereoCalibration  # noqa: E402
from stereo_vision.depth import colorize_depth  # noqa: E402
from stereo_vision.live import DepthPipeline  # noqa: E402
from stereo_vision.outputs import CameraModel, LaserScan, ScanConfig, build_robot_frame, laser_scan  # noqa: E402
from stereo_vision.presets import get_preset  # noqa: E402
from stereo_vision.sources import StereoFrame  # noqa: E402

from gz.msgs10.camera_info_pb2 import CameraInfo  # noqa: E402
from gz.msgs10.contacts_pb2 import Contacts  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from gz.msgs10.twist_pb2 import Twist  # noqa: E402
from gz.transport13 import Node  # noqa: E402

HERE = Path(__file__).parent
TRUTH = json.loads((HERE / "worlds" / "obstacles.json").read_text())
BASELINE_MM = TRUTH["baseline_m"] * 1000.0
# Left camera in the rover body frame (make_world.py: CAM_X, +baseline/2, and
# CAM_Z above a chassis centred 0.2 m off the ground).
CAM_IN_BODY = (0.30, TRUTH["baseline_m"] / 2)
CAM_HEIGHT = 0.2 + 0.17
RIGHT_EDGE_PX = 40  # at the pi5 preset's 320 px width


def stamp_of(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9


class Mailbox:
    """Latest sensor data from gz-transport callbacks, which run on their own threads."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.images: dict[float, dict[str, np.ndarray]] = {}
        self.truth: dict[float, np.ndarray] = {}
        self.info: CameraInfo | None = None
        self.pose: tuple[float, float, float] | None = None
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
        """True rover pose from the world, for the map and scoring only."""
        for pose in msg.pose:
            if pose.name == "rover":
                q = pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with self.lock:
                    self.pose = (pose.position.x, pose.position.y, yaw)
                return

    def on_contacts(self, msg: Contacts) -> None:
        names = set()
        for contact in msg.contact:
            for name in (contact.collision1.name, contact.collision2.name):
                if "rover" not in name and "ground_plane" not in name:
                    names.add(name.split("::")[0])
        with self.lock:
            self.touching |= names
            self.touch_time = time.time()

    def newest_pair(self, after: float) -> tuple[float, np.ndarray, np.ndarray] | None:
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

    def truth_at(self, stamp: float) -> np.ndarray | None:
        with self.lock:
            depth = self.truth.get(stamp)
            for s in [s for s in self.truth if s < stamp - 1.0]:
                del self.truth[s]
            return depth


def ideal_calibration(info: CameraInfo) -> StereoCalibration:
    """What a perfect chessboard calibration of the simulated rig would return."""
    K = np.array(info.intrinsics.k, np.float64).reshape(3, 3)
    size = (int(info.width), int(info.height))
    dist = np.zeros(5)
    R = np.eye(3)
    # OpenCV's T maps left-camera points into the right camera, which sits
    # BASELINE to the right (+x in the optical frame): X_r = X_l - (B, 0, 0).
    T = np.array([[-BASELINE_MM], [0.0], [0.0]])
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K, dist, K, dist, size, R, T,
                                                flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    return StereoCalibration(size, K, dist, K.copy(), dist.copy(), R, T, R1, R2, P1, P2, Q,
                             rms=0.0, coverage_pct=100.0)


@dataclass
class Command:
    v: float
    w: float
    state: str
    front_m: float


class Avoider:
    """Reactive avoidance on the stereo laser scan.

    - Look down a corridor as wide as the rover: the nearest hit in it sets speed.
    - Obstacles to the side, within ``influence``, push the heading away.
    - Too close ahead: stop and turn in place towards the clearer side, and keep
      turning that way until the corridor is open (committing stops dithering).

    Beams the matcher could not see (NaN) are ignored, not treated as clear, and
    'nothing in range' (inf) counts as clear out to the scan's range_max.
    """

    def __init__(self, half_width: float = 0.40, stop: float = 0.85, slow: float = 2.0,
                 go: float = 1.6, v_max: float = 0.5, w_turn: float = 0.7, influence: float = 1.2,
                 gain: float = 0.08) -> None:
        self.half_width, self.stop, self.slow, self.go = half_width, stop, slow, go
        self.v_max, self.w_turn, self.influence, self.gain = v_max, w_turn, influence, gain
        self.turn_dir = 0

    def step(self, scan: LaserScan) -> Command:
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        seen = ~np.isnan(scan.ranges)
        r = np.where(np.isinf(scan.ranges), scan.range_max, scan.ranges)
        x, y = r * np.cos(angles), r * np.sin(angles)
        corridor = seen & (np.abs(y) < self.half_width)
        front = float(x[corridor].min()) if corridor.any() else scan.range_max

        def clearance(mask):
            return float(np.mean(r[mask])) if mask.any() else 0.0
        left, right = clearance(seen & (angles > 0.1)), clearance(seen & (angles < -0.1))

        if self.turn_dir:
            if front > self.go:
                self.turn_dir = 0
            else:
                return Command(0.0, self.turn_dir * self.w_turn, "turn", front)
        if front < self.stop:
            self.turn_dir = 1 if left >= right else -1
            return Command(0.0, self.turn_dir * self.w_turn, "turn", front)

        v = self.v_max * float(np.clip((front - self.stop) / (self.slow - self.stop), 0.3, 1.0))
        near = seen & (r < self.influence)
        # Left obstacles (positive bearing) push the heading right (negative w).
        push = float(np.sum(-np.sign(angles[near]) * (1.0 / r[near] - 1.0 / self.influence)))
        w = self.gain * push
        if front < self.slow:
            w += 0.3 * np.sign(left - right)
        return Command(v, float(np.clip(w, -1.0, 1.0)), "cruise", front)


def scan_error(stereo: LaserScan, truth: LaserScan, near: float = 2.0) -> dict:
    """How the stereo scan compares with the true one, beam by beam."""
    s, t = stereo.ranges, truth.ranges
    both = np.isfinite(s) & np.isfinite(t) & (t < 4.0)
    close = np.isfinite(t) & (t < near)
    return {
        "abs_err": np.abs(s[both] - t[both]).tolist(),
        "close": int(close.sum()),
        "missed": int((close & np.isinf(s)).sum()),   # said clear, was not: the dangerous one
        "blind": int((close & np.isnan(s)).sum()),    # said unknown
        # said something was there, 0.5 m or more nearer than anything real
        "phantom": int((np.isfinite(s) & (s < near) & ~(np.isfinite(t) & (t < s + 0.5))).sum()),
        "stereo_close": int((np.isfinite(s) & (s < near)).sum()),
    }


class TopDown:
    """Top-down map of the arena, trajectory and live scan, for the video."""

    def __init__(self, size: int = 480) -> None:
        self.size = size
        self.scale = size / (TRUTH["arena"] + 1.0)
        self.base = np.full((size, size, 3), 245, np.uint8)
        half = TRUTH["arena"] / 2
        cv2.rectangle(self.base, self.px(-half, half), self.px(half, -half), (90, 90, 90), 2)
        colours = {"red": (70, 90, 230), "yellow": (80, 200, 230), "green": (110, 200, 90)}
        for i, o in enumerate(TRUTH["obstacles"]):
            colour = list(colours.values())[i % 3]
            if o["kind"] == "cylinder":
                cv2.circle(self.base, self.px(o["x"], o["y"]), max(1, round(o["radius"] * self.scale)), colour, -1)
            else:
                sx, sy = (float(v) for v in o["geometry"].split("<size>")[1].split()[:2])
                box = cv2.boxPoints(((o["x"], o["y"]), (sx, sy), math.degrees(o["yaw"])))
                cv2.fillPoly(self.base, [np.array([self.px(*p) for p in box], np.int32)], colour)
        self.trail: list[tuple[int, int]] = []

    def px(self, x: float, y: float) -> tuple[int, int]:
        return (int(round(self.size / 2 + x * self.scale)), int(round(self.size / 2 - y * self.scale)))

    def draw(self, pose, scan: LaserScan | None) -> np.ndarray:
        view = self.base.copy()
        if pose is None:
            return view
        x, y, yaw = pose
        self.trail.append(self.px(x, y))
        if len(self.trail) > 1:
            cv2.polylines(view, [np.array(self.trail, np.int32)], False, (200, 120, 40), 2)
        c, s = math.cos(yaw), math.sin(yaw)
        if scan is not None:
            angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
            cx, cy = CAM_IN_BODY
            for a, r in zip(angles, scan.ranges):
                if np.isfinite(r):
                    bx, by = cx + r * math.cos(a), cy + r * math.sin(a)
                    cv2.circle(view, self.px(x + c * bx - s * by, y + s * bx + c * by), 2, (0, 0, 200), -1)
        corners = [(0.3, 0.2), (0.3, -0.2), (-0.3, -0.2), (-0.3, 0.2)]
        poly = [self.px(x + c * bx - s * by, y + s * bx + c * by) for bx, by in corners]
        cv2.fillPoly(view, [np.array(poly, np.int32)], (170, 80, 40))
        cv2.line(view, self.px(x, y), self.px(x + 0.5 * c, y + 0.5 * s), (0, 0, 0), 2)
        return view


def scan_plot(stereo: LaserScan, truth: LaserScan | None, size=(320, 240), max_r=4.0) -> np.ndarray:
    """Polar scan, rover at the bottom: red = stereo, grey = truth."""
    w, h = size
    view = np.full((h, w, 3), 255, np.uint8)
    origin = (w // 2, h - 10)
    scale = (h - 20) / max_r
    for ring in (1, 2, 3, 4):
        cv2.ellipse(view, origin, (int(ring * scale), int(ring * scale)), 0, 180, 360, (225, 225, 225), 1)
    for scan, colour, radius in ((truth, (150, 150, 150), 2), (stereo, (0, 0, 220), 2)):
        if scan is None:
            continue
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        for a, r in zip(angles, scan.ranges):
            if np.isfinite(r) and r <= max_r:
                p = (int(origin[0] - r * math.sin(a) * scale), int(origin[1] - r * math.cos(a) * scale))
                cv2.circle(view, p, radius, colour, -1)
    cv2.putText(view, "scan: red stereo, grey truth", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
    return view


def text_panel(lines: list[str], size=(320, 240)) -> np.ndarray:
    view = np.full((size[1], size[0], 3), 30, np.uint8)
    for i, line in enumerate(lines):
        cv2.putText(view, line, (8, 24 + i * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (235, 235, 235), 1, cv2.LINE_AA)
    return view


def label(view: np.ndarray, text: str, org=(8, 20), scale=0.5) -> np.ndarray:
    """Caption on a dark strip, readable over any image."""
    (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    cv2.rectangle(view, (org[0] - 4, org[1] - h - 6), (org[0] + w + 4, org[1] + 6), (20, 20, 20), -1)
    cv2.putText(view, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
    return view


STATE_COLOURS = {"cruise": (60, 170, 60), "turn": (0, 140, 255)}
PANEL_SIZE = (1280, 720)


def compose(chase, left, depth_mm, scan, truth_scan, map_view, command, status) -> np.ndarray:
    """One 1280x720 frame: Gazebo chase view and map on top, what the rover sees below.

    +---------------------------+--------------+
    | Gazebo chase camera       | top-down map |
    | 800 x 450                 | 480 x 450    |
    +--------+--------+-------+-+--------------+
    | left   | depth  | scan  | status         |
    | 360x270| 360x270|280x270| 280x270        |
    +--------+--------+-------+----------------+
    """
    if chase is None:
        chase_view = np.full((450, 800, 3), 60, np.uint8)
        cv2.putText(chase_view, "waiting for /chase/image", (250, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (220, 220, 220), 1, cv2.LINE_AA)
    else:
        chase_view = cv2.resize(cv2.cvtColor(chase, cv2.COLOR_RGB2BGR), (800, 450), interpolation=cv2.INTER_AREA)
    label(chase_view, "Gazebo sim - chase camera", (10, 24), 0.55)
    badge = f"{command.state.upper()}  v {command.v:.2f} m/s  w {command.w:+.2f} rad/s"
    (bw, _), _ = cv2.getTextSize(badge, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.rectangle(chase_view, (790 - bw - 16, 8), (790, 38), STATE_COLOURS.get(command.state, (90, 90, 90)), -1)
    cv2.putText(chase_view, badge, (790 - bw - 8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

    map_panel = np.full((450, 480, 3), 245, np.uint8)
    map_panel[:, 15:465] = map_view
    label(map_panel, "map: path, stereo scan (red dots)", (22, 24), 0.45)

    left_view = cv2.resize(cv2.cvtColor(left, cv2.COLOR_GRAY2BGR), (360, 270))
    depth_view = cv2.resize(colorize_depth(depth_mm, 300, 5000), (360, 270), interpolation=cv2.INTER_NEAREST)
    label(left_view, "left camera, rectified (Ster-Vis input)", (8, 20), 0.42)
    label(depth_view, "Ster-Vis depth  red 0.3 m ... blue 5 m", (8, 20), 0.42)
    scan_view = scan_plot(scan, truth_scan, size=(280, 270))
    top = cv2.hconcat([chase_view, map_panel])
    bottom = cv2.hconcat([left_view, depth_view, scan_view, text_panel(status, size=(280, 270))])
    return cv2.vconcat([top, bottom])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=180.0, help="sim seconds to drive")
    parser.add_argument("--preset", default="pi5", help="Ster-Vis preset (pi5 = half resolution)")
    parser.add_argument("--min-distance", type=float, default=0.45, help="closest range to match, m")
    parser.add_argument("--min-confidence", type=float, default=50.0,
                        help="drop depth whose left-right check scores below this, 0-100 (0 = off)")
    parser.add_argument("--pi-fps", type=float, default=0.0, help="cap processed pairs per sim second (0 = all)")
    parser.add_argument("--record", type=int, default=0,
                        help="save every Nth pair + true depth to output/frames/ for offline tuning")
    parser.add_argument("--show", action="store_true", help="live window")
    parser.add_argument("--out", default=str(HERE / "output"))
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    box = Mailbox()
    node = Node()
    node.subscribe(Image, "/stereo/left/image", box.on_image("left"))
    node.subscribe(Image, "/stereo/right/image", box.on_image("right"))
    node.subscribe(Image, "/stereo/truth/depth", box.on_truth)
    node.subscribe(CameraInfo, "/stereo/left/camera_info", box.on_info)
    node.subscribe(Image, "/chase/image", box.on_chase)
    node.subscribe(Pose_V, "/world/stereo_avoid/pose/info", box.on_poses)
    node.subscribe(Contacts, "/world/stereo_avoid/model/rover/link/chassis/sensor/bumper/contact", box.on_contacts)
    cmd_pub = node.advertise("/model/rover/cmd_vel", Twist)

    print("waiting for camera_info ...", flush=True)
    deadline = time.time() + 60
    while box.info is None:
        if time.time() > deadline:
            sys.exit("no /stereo/left/camera_info - is the server running (run-gz.ps1)?")
        time.sleep(0.1)

    calibration = ideal_calibration(box.info)
    pipeline = DepthPipeline(calibration, get_preset(args.preset), min_distance_mm=args.min_distance * 1000,
                             confidence=args.min_confidence > 0)
    camera = CameraModel.from_calibration(pipeline.calibration)
    K = np.array(box.info.intrinsics.k).reshape(3, 3)
    truth_camera = CameraModel(int(box.info.width), int(box.info.height), K[0, 0], K[1, 1], K[0, 2], K[1, 2],
                               BASELINE_MM / 1000)
    # Scan band: 6 cm above the floor up to 10 cm above the cameras (0.47 m),
    # which covers everything the rover can hit. A taller band lets false
    # matches in the sky through as phantom obstacles: tune.py measured
    # phantoms at 10% of near beams with the top at +0.4 m and 1% at +0.1 m.
    scan_config = ScanConfig(beams=121, min_height_m=-(CAM_HEIGHT - 0.06), max_height_m=0.1,
                             range_min_m=0.2, range_max_m=5.0)
    print(f"Ster-Vis {args.preset}: rectified {camera.width}x{camera.height}, f {camera.fx:.1f}px, "
          f"baseline {BASELINE_MM:.0f} mm, {pipeline.params.num_disparities} disparities, "
          f"closest {pipeline.closest_mm / 1000:.2f} m", flush=True)

    avoider = Avoider()
    topdown = TopDown(size=450)
    # one video frame per processed pair; at 10 pairs/s that plays in real time
    fps = args.pi_fps or 10
    video = cv2.VideoWriter(str(out / "run.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, PANEL_SIZE)
    log_file = open(out / "log.csv", "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["sim_t", "x", "y", "yaw", "state", "v", "w", "front_m", "nearest_m", "valid_pct",
                  "match_ms", "total_ms", "missed_beams", "blind_beams"])

    errors, missed, blind, close_beams, frame_ms = [], 0, 0, 0, []
    phantom, stereo_close = 0, 0
    collisions, touched_names, was_touching = 0, set(), False
    last_stamp, last_processed, start_stamp, seq = -1.0, -1e9, None, 0
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

            t0 = time.perf_counter()
            frame = StereoFrame(cv2.cvtColor(left_rgb, cv2.COLOR_RGB2GRAY),
                                cv2.cvtColor(right_rgb, cv2.COLOR_RGB2GRAY), timestamp=stamp)
            result = pipeline.process(frame)
            if result.confidence is not None:
                # Untextured surfaces (sky, bare floor) give SGBM false matches;
                # matching right-to-left disagrees on them, so drop those pixels.
                low = result.confidence < args.min_confidence
                # The right-view matcher has no data in the last columns, so the
                # check scores them 0; keep the left matcher's depth there, or
                # the scan calls that edge clear (tune.py: missed 2% -> 0.03%).
                low[:, -round(RIGHT_EDGE_PX * camera.width / 320):] = False
                result.depth_mm[low] = np.nan
            robot = build_robot_frame(result, camera, seq, scan_config)
            command = avoider.step(robot.scan)
            total_ms = (time.perf_counter() - t0) * 1000
            frame_ms.append(total_ms)
            twist = Twist()
            twist.linear.x, twist.angular.z = command.v, command.w
            cmd_pub.publish(twist)
            seq += 1

            truth_depth = box.truth_at(stamp)
            if args.record and seq % args.record == 0 and truth_depth is not None:
                (out / "frames").mkdir(exist_ok=True)
                np.savez_compressed(out / "frames" / f"{seq:05d}.npz", left=left_rgb, right=right_rgb,
                                    truth=truth_depth, k=np.array(box.info.intrinsics.k))
            truth_scan = None
            miss = bl = 0
            if truth_depth is not None:
                truth_scan = laser_scan(np.where(np.isfinite(truth_depth), truth_depth, np.inf).astype(np.float32),
                                        truth_camera, scan_config.beams, scan_config.min_height_m,
                                        scan_config.max_height_m, scan_config.range_min_m, scan_config.range_max_m)
                err = scan_error(robot.scan, truth_scan)
                errors += err["abs_err"]
                miss, bl = err["missed"], err["blind"]
                missed, blind, close_beams = missed + miss, blind + bl, close_beams + err["close"]
                phantom, stereo_close = phantom + err["phantom"], stereo_close + err["stereo_close"]

            with box.lock:
                pose = box.pose
                chase = box.chase
                touching = bool(box.touching) and time.time() - box.touch_time < 0.3
                touched_names |= box.touching
                box.touching = set() if not touching else box.touching
            if touching and not was_touching:
                collisions += 1
                print(f"  t={sim_t:6.1f}s  CONTACT with {sorted(touched_names)}", flush=True)
            was_touching = touching
            if pose and last_xy:
                distance += math.hypot(pose[0] - last_xy[0], pose[1] - last_xy[1])
            last_xy = pose[:2] if pose else last_xy

            summary = robot.summary()
            nearest = summary["obstacles"]["nearest_m"]
            log.writerow([round(sim_t, 2), *(round(v, 3) for v in (pose or (0, 0, 0))), command.state,
                          round(command.v, 3), round(command.w, 3), round(command.front_m, 3), nearest,
                          summary["valid_pct"], round(result.stage_ms["match"], 1), round(total_ms, 1), miss, bl])

            status = [
                f"sim time   {sim_t:6.1f} s",
                f"state      {command.state}",
                f"corridor   {command.front_m:4.2f} m ahead",
                f"nearest    {nearest if nearest is not None else '-'} m",
                f"depth px   {summary['valid_pct']:.0f}%",
                f"stereo     {total_ms:4.0f} ms/pair",
                f"contacts   {collisions}",
                f"driven     {distance:5.1f} m",
                f"Ster-Vis {args.preset}, {camera.width}x{camera.height}",
            ]
            panel = compose(chase, result.left, result.depth_mm, robot.scan, truth_scan,
                            topdown.draw(pose, robot.scan), command, status)
            video.write(panel)
            if args.show:
                cv2.imshow("stereo avoid", panel)
                if cv2.waitKey(1) == 27:
                    break
            if seq % 50 == 0:
                print(f"  t={sim_t:6.1f}s  {command.state:6s} front {command.front_m:4.2f} m  "
                      f"{total_ms:5.1f} ms/pair  dist {distance:5.1f} m  contacts {collisions}", flush=True)
    finally:
        cmd_pub.publish(Twist())
        video.release()
        log_file.close()
        cv2.imwrite(str(out / "trajectory.png"), topdown.draw(box.pose, None))

    err = np.array(errors) if errors else np.array([np.nan])
    result_summary = {
        "sim_seconds": round(float(sim_t), 1),
        "pairs_processed": seq,
        "distance_m": round(distance, 2),
        "contacts": collisions,
        "touched": sorted(touched_names),
        "stereo_ms_per_pair_median_this_pc": round(float(np.median(frame_ms)), 1),
        "scan_abs_error_m": {"median": round(float(np.nanmedian(err)), 3),
                             "p90": round(float(np.nanpercentile(err, 90)), 3)},
        "close_beams_(<2m)": close_beams,
        "missed_close_beams_pct": round(100 * missed / max(close_beams, 1), 2),
        "blind_close_beams_pct": round(100 * blind / max(close_beams, 1), 2),
        "phantom_beams_pct": round(100 * phantom / max(stereo_close, 1), 2),
        "min_confidence": args.min_confidence,
        "preset": args.preset,
        "pi_fps_cap": args.pi_fps,
    }
    (out / "summary.json").write_text(json.dumps(result_summary, indent=2))
    print(json.dumps(result_summary, indent=2))


if __name__ == "__main__":
    main()
