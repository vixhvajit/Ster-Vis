"""Robot-ready outputs from a depth result: the data a robot actually consumes.

There are deliberately no per-pixel surface normals. Tested against ray-traced
truth, SGBM depth at a 60 mm baseline was too noisy for them: even smoothed,
they were 21 degrees off on a flat wall at 2.2 m, and cost more than the depth
itself. Fit planes to the point cloud instead (a RANSAC ground plane, say),
which averages over thousands of points.

Everything here follows ROS conventions so it drops into existing stacks:

- lengths in metres (REP 117/118)
- the camera optical frame for images and points: x right, y down, z forward
- a body frame for the laser scan: x forward, y left, z up (REP 103)

The camera is assumed to be mounted level and facing forward. The scan and
obstacle outputs measure in the camera's own frame; for a tilted or offset
mount, transform them with the robot's TF tree, or pass the mount height so
the scan slices the right band of the world.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import cached_property

import numpy as np

from .calibration import StereoCalibration

OPTICAL_FRAME = "ster_vis_left_optical_frame"
BODY_FRAME = "ster_vis_link"


@dataclass(frozen=True)
class CameraModel:
    """Pinhole model of the rectified left camera, as ROS CameraInfo needs it."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    baseline_m: float

    @classmethod
    def from_calibration(cls, calibration: StereoCalibration, unit_to_m: float = 0.001) -> "CameraModel":
        P = calibration.P1
        width, height = calibration.output_size
        return cls(width, height, float(P[0, 0]), float(P[1, 1]), float(P[0, 2]), float(P[1, 2]),
                   calibration.baseline * unit_to_m)

    @property
    def K(self) -> list[float]:
        return [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]

    @property
    def P(self) -> list[float]:
        return [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    @property
    def horizontal_fov_rad(self) -> float:
        return 2.0 * math.atan(self.width / (2.0 * self.fx))

    def as_dict(self) -> dict:
        return {
            "width": self.width, "height": self.height,
            "fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy,
            "baseline_m": self.baseline_m,
            "K": self.K, "P": self.P, "distortion_model": "plumb_bob", "D": [0.0] * 5,
            "horizontal_fov_deg": math.degrees(self.horizontal_fov_rad),
            "frame_id": OPTICAL_FRAME,
        }


def points_from_depth(depth_m: np.ndarray, camera: CameraModel) -> np.ndarray:
    """Organised point cloud, H x W x 3 in metres in the optical frame; NaN where no depth."""
    height, width = depth_m.shape
    u = np.arange(width, dtype=np.float32)
    v = np.arange(height, dtype=np.float32)
    x = (u[None, :] - camera.cx) / camera.fx * depth_m
    y = (v[:, None] - camera.cy) / camera.fy * depth_m
    return np.dstack([x, y, depth_m]).astype(np.float32)


@dataclass
class LaserScan:
    """A 2D scan in the body frame, as sensor_msgs/LaserScan defines it."""

    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: np.ndarray  # metres; inf = nothing within range_max, nan = no data

    def as_dict(self) -> dict:
        ranges = [None if not np.isfinite(r) else round(float(r), 4) for r in self.ranges]
        return {
            "frame_id": BODY_FRAME,
            "angle_min": self.angle_min, "angle_max": self.angle_max,
            "angle_increment": self.angle_increment,
            "range_min": self.range_min, "range_max": self.range_max,
            "ranges": ranges,
        }


def laser_scan(
    depth_m: np.ndarray,
    camera: CameraModel,
    beams: int = 181,
    min_height_m: float = -0.25,
    max_height_m: float = 0.25,
    range_min_m: float = 0.2,
    range_max_m: float = 10.0,
) -> LaserScan:
    """Flatten the depth image into a 2D scan: the nearest point per bearing.

    Only points whose height relative to the camera lies between
    ``min_height_m`` and ``max_height_m`` count, so the floor and ceiling are
    ignored. Set that band to the heights your robot can collide with: for a
    camera 0.3 m above the floor, a band of -0.25..+0.5 keeps everything from
    5 cm above the floor upwards. Bearings with no point in range read inf,
    as navigation stacks expect; bearings the matcher could not see read nan.

    For a pinhole camera a pixel's bearing depends only on its column, so the
    work is a minimum down each column followed by folding columns into
    beams, rather than a bearing for every point.
    """
    height, width = depth_m.shape
    half = camera.horizontal_fov_rad / 2.0
    angle_min, angle_max = -half, half
    increment = (angle_max - angle_min) / (beams - 1)

    # Per column: bearing (body frame, positive to the left) and the factor
    # from depth z to horizontal distance, sqrt(z^2 + x^2) = z * factor.
    slope = (np.arange(width, dtype=np.float32) - camera.cx) / camera.fx
    bearing = np.arctan(-slope)
    factor = np.sqrt(1.0 + slope * slope)
    beam_of_column = np.clip(np.rint((bearing - angle_min) / increment).astype(int), 0, beams - 1)

    # Height above the camera per pixel: up = -y = -(v - cy) / fy * z.
    rows = ((camera.cy - np.arange(height, dtype=np.float32)) / camera.fy)[:, None]
    with np.errstate(invalid="ignore"):
        up = rows * depth_m
        horizontal = depth_m * factor[None, :]
        usable = (up >= min_height_m) & (up <= max_height_m) & (horizontal >= range_min_m) & (horizontal <= range_max_m)
    column_min = np.where(usable, horizontal, np.inf).min(axis=0)
    column_seen = np.isfinite(depth_m).any(axis=0)

    ranges = np.full(beams, np.inf, np.float32)
    np.minimum.at(ranges, beam_of_column, column_min.astype(np.float32))
    seen = np.zeros(beams, bool)
    seen[beam_of_column[column_seen]] = True
    ranges[~seen] = np.nan
    # SGBM leaves single beams empty inside an obstacle; a lone gap between two
    # hits is filled with the nearer neighbour so a wall does not look porous.
    lone = np.isinf(ranges[1:-1]) & np.isfinite(ranges[:-2]) & np.isfinite(ranges[2:])
    ranges[1:-1][lone] = np.minimum(ranges[:-2], ranges[2:])[lone]

    return LaserScan(angle_min, angle_max, increment, range_min_m, range_max_m, ranges)


@dataclass
class Obstacles:
    """The simplest useful summary: how far is the nearest thing, and where."""

    nearest_m: float | None
    nearest_bearing_deg: float | None
    sectors: dict[str, float | None] = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            "nearest_m": self.nearest_m,
            "nearest_bearing_deg": self.nearest_bearing_deg,
            "sectors_m": self.sectors,
            "bearing_convention": "degrees, 0 = straight ahead, positive = left",
        }


def obstacles(scan: LaserScan, sector_names: tuple[str, ...] = ("left", "centre", "right")) -> Obstacles:
    """Nearest range overall and in equal-width sectors, left to right. None = clear or blind."""
    ranges = scan.ranges
    finite = np.isfinite(ranges)
    if finite.any():
        index = int(np.nanargmin(np.where(finite, ranges, np.nan)))
        nearest = round(float(ranges[index]), 4)
        bearing = round(math.degrees(scan.angle_min + index * scan.angle_increment), 2)
    else:
        nearest = bearing = None

    # Beams run right to left (angle_min is to the right), so reverse for left-first names.
    chunks = np.array_split(ranges[::-1], len(sector_names))
    sectors = {}
    for name, chunk in zip(sector_names, chunks):
        values = chunk[np.isfinite(chunk)]
        sectors[name] = round(float(values.min()), 4) if values.size else None
    return Obstacles(nearest, bearing, sectors)


def confidence_from_lr(left_disparity: np.ndarray, right_disparity: np.ndarray, tolerance_px: float = 2.0) -> np.ndarray:
    """Per-pixel confidence 0-100 from a left-right consistency check.

    A pixel's match is trusted when matching the other way round lands back on
    it. The confidence falls linearly with the disagreement, reaching 0 at
    ``tolerance_px``. Unmatched pixels get 0.
    """
    height, width = left_disparity.shape
    columns = np.arange(width)[None, :] - np.rint(left_disparity).astype(int)
    valid = (left_disparity > 0) & (columns >= 0)
    columns = np.clip(columns, 0, width - 1)
    back = np.take_along_axis(right_disparity, columns, axis=1)
    disagreement = np.abs(left_disparity - back)
    confidence = np.clip(1.0 - disagreement / tolerance_px, 0.0, 1.0) * 100.0
    confidence[~valid | ~np.isfinite(back) | (back <= 0)] = 0.0
    return confidence.astype(np.float32)


@dataclass(frozen=True)
class ScanConfig:
    beams: int = 181
    min_height_m: float = -0.25
    max_height_m: float = 0.25
    range_min_m: float = 0.2
    range_max_m: float = 10.0

    def __post_init__(self) -> None:
        # An inverted band or range would silently report "all clear", which a
        # robot would believe, so refuse it outright.
        if self.beams < 2:
            raise ValueError("the laser scan needs at least 2 beams")
        if self.min_height_m >= self.max_height_m:
            raise ValueError("scan min height must be below max height")
        if not 0 <= self.range_min_m < self.range_max_m:
            raise ValueError("scan range must satisfy 0 <= min < max")


@dataclass
class RobotFrame:
    """Everything one frame offers a robot, in metres and ROS frames."""

    sequence: int
    timestamp: float
    left: np.ndarray                 # rectified left image the rest lines up with
    depth_m: np.ndarray              # NaN where unknown
    camera: CameraModel
    scan: LaserScan
    obstacles: Obstacles
    confidence: np.ndarray | None = None
    stage_ms: dict[str, float] = field(default_factory=dict)

    @cached_property
    def points(self) -> np.ndarray:
        """H x W x 3 in the optical frame, NaN where unknown; built on first use."""
        return points_from_depth(self.depth_m, self.camera)

    def summary(self) -> dict:
        """The small, JSON-ready part: what a robot polls or streams every frame."""
        valid = np.isfinite(self.depth_m)
        return {
            "sequence": self.sequence,
            "timestamp": self.timestamp,
            "valid_pct": round(100.0 * float(valid.mean()), 2),
            "obstacles": self.obstacles.as_dict(),
            "scan": self.scan.as_dict(),
            "stage_ms": {k: round(v, 2) for k, v in self.stage_ms.items()},
        }


def build_robot_frame(
    result,
    camera: CameraModel,
    sequence: int,
    scan_config: ScanConfig = ScanConfig(),
    unit_to_m: float = 0.001,
) -> RobotFrame:
    """Turn a live.DepthResult into a RobotFrame."""
    depth_m = (result.depth_mm * unit_to_m).astype(np.float32)
    scan = laser_scan(depth_m, camera, scan_config.beams, scan_config.min_height_m,
                      scan_config.max_height_m, scan_config.range_min_m, scan_config.range_max_m)
    return RobotFrame(
        sequence=sequence,
        timestamp=result.timestamp if result.timestamp is not None else 0.0,
        left=result.left,
        depth_m=depth_m,
        camera=camera,
        scan=scan,
        obstacles=obstacles(scan),
        confidence=result.confidence,
        stage_ms=dict(result.stage_ms),
    )


def ply_bytes(points: np.ndarray, intensity: np.ndarray | None = None, step: int = 1) -> bytes:
    """Binary little-endian PLY of the finite points, every ``step``-th pixel each way."""
    sub = points[::step, ::step].reshape(-1, 3)
    keep = np.isfinite(sub).all(axis=1)
    xyz = sub[keep].astype("<f4")
    if intensity is not None:
        grey = intensity[::step, ::step].reshape(-1)[keep].astype(np.uint8)
        record = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                            ("red", "u1"), ("green", "u1"), ("blue", "u1")])
        record["x"], record["y"], record["z"] = xyz.T
        record["red"] = record["green"] = record["blue"] = grey
        props = "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n"
    else:
        record = xyz
        props = "property float x\nproperty float y\nproperty float z\n"
    header = f"ply\nformat binary_little_endian 1.0\nelement vertex {len(xyz)}\n{props}end_header\n"
    return header.encode("ascii") + record.tobytes()
