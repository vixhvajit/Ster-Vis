"""Fuse per-frame point clouds into one map of the environment.

A depth camera sees one view at a time. A map is what you get by putting many
views in the same frame and keeping what they agree on: a metric point cloud
of the whole room, which is what a "digital reconstruction" of a building or
a warehouse aisle actually is.

Ster-Vis does not estimate where the camera is. Every frame is fused at a
pose handed in from outside: a ROS 2 TF lookup, a wheel or flight controller's
odometry, a motion capture system, or ground truth in simulation. Stereo depth
is metrically correct but has no memory, so a map built on drifting odometry
drifts with it; the pose source is what decides the map's quality, and it is
deliberately someone else's job (`rtabmap_ros`, `slam_toolbox` and friends do
it from these same outputs).

The map itself is a voxel grid, hashed rather than allocated, so only the
space that was seen costs anything. Each voxel keeps the running mean of the
points that fell in it and how many there were:

- the mean averages away stereo noise, which is zero-mean along the ray, so a
  wall seen from several poses reconstructs better than from any one of them;
- the hit count is the evidence for a voxel being real, and exporting with
  ``min_hits`` above 1 drops the single-frame flyers that SGBM leaves on
  depth discontinuities.

Frames: points arrive in the camera optical frame (x right, y down, z
forward) and are fused in a map frame that follows REP 103 (x forward, y
left, z up), so a floor plan is the x-y plane and height is z.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .outputs import CameraModel, RobotFrame, ply_from_points, points_from_depth

# The camera optical frame expressed in a REP 103 body frame: optical z
# (forward) is body x, optical x (right) is body -y, optical y (down) is
# body -z. This is the rotation every ROS camera driver puts on the link
# between its body frame and its optical frame.
OPTICAL_TO_BODY = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])

MAP_FRAME = "map"


def rotation_from_quaternion(quaternion: tuple[float, float, float, float]) -> np.ndarray:
    """Rotation matrix from a ROS quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("quaternion has zero length")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation matrix from REP 103 roll, pitch and yaw (radians, applied Z-Y-X)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


@dataclass(frozen=True)
class Pose:
    """Where the camera was: the transform from its optical frame to the map frame.

    ``rotation`` is 3x3 and ``translation`` is in metres. Build one with
    :meth:`from_body` when what you have is the robot's pose in ROS axes,
    which is the usual case, or :meth:`from_optical` when the pose is already
    of the optical frame (a TF lookup of the optical frame, for instance).
    """

    rotation: np.ndarray
    translation: np.ndarray

    def __post_init__(self) -> None:
        rotation = np.asarray(self.rotation, float).reshape(3, 3)
        translation = np.asarray(self.translation, float).reshape(3)
        object.__setattr__(self, "rotation", rotation)
        object.__setattr__(self, "translation", translation)

    @classmethod
    def identity(cls) -> "Pose":
        return cls(np.eye(3), np.zeros(3))

    @classmethod
    def from_optical(cls, translation, rotation=None, quaternion=None, rpy=None) -> "Pose":
        """Pose of the camera optical frame itself, from a matrix, quaternion or RPY."""
        return cls(_rotation_of(rotation, quaternion, rpy), translation)

    @classmethod
    def from_body(cls, translation, rotation=None, quaternion=None, rpy=None) -> "Pose":
        """Pose of the camera's REP 103 body frame; the optical rotation is added here.

        This is what a robot reports about itself: x forward, y left, z up.
        """
        return cls(_rotation_of(rotation, quaternion, rpy) @ OPTICAL_TO_BODY, translation)

    @classmethod
    def from_matrix(cls, matrix) -> "Pose":
        """From a 4x4 homogeneous transform of the optical frame."""
        matrix = np.asarray(matrix, float).reshape(4, 4)
        return cls(matrix[:3, :3], matrix[:3, 3])

    @property
    def matrix(self) -> np.ndarray:
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = self.translation
        return matrix

    def transform(self, points: np.ndarray) -> np.ndarray:
        """Carry points (..., 3) from the optical frame into the map frame."""
        points = np.asarray(points, np.float32)
        return (points @ self.rotation.T.astype(np.float32)) + self.translation.astype(np.float32)

    def inverse(self) -> "Pose":
        return Pose(self.rotation.T, -self.rotation.T @ self.translation)

    def distance_to(self, other: "Pose") -> float:
        return float(np.linalg.norm(self.translation - other.translation))

    def angle_to(self, other: "Pose") -> float:
        """Angle between the two orientations, in degrees."""
        trace = float(np.trace(self.rotation.T @ other.rotation))
        return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def _rotation_of(rotation, quaternion, rpy) -> np.ndarray:
    given = [value is not None for value in (rotation, quaternion, rpy)]
    if sum(given) != 1:
        raise ValueError("give exactly one of rotation, quaternion or rpy")
    if rotation is not None:
        return np.asarray(rotation, float).reshape(3, 3)
    if quaternion is not None:
        return rotation_from_quaternion(quaternion)
    return rotation_from_rpy(*(float(v) for v in rpy))


@dataclass
class PoseTrack:
    """Poses from an outside source, looked up by time or by frame number."""

    poses: list[Pose]
    times: np.ndarray | None = None  # seconds, same length as poses, when the file had them

    def __len__(self) -> int:
        return len(self.poses)

    def at_index(self, index: int) -> Pose | None:
        return self.poses[index] if 0 <= index < len(self.poses) else None

    def at_time(self, timestamp: float, tolerance_s: float = 0.1) -> Pose | None:
        """Nearest pose in time, or None when the closest is further off than the tolerance."""
        if self.times is None or not len(self.poses):
            return None
        index = int(np.argmin(np.abs(self.times - timestamp)))
        if abs(float(self.times[index]) - timestamp) > tolerance_s:
            return None
        return self.poses[index]


# Column names accepted in a pose file, lower-cased. Position is required;
# orientation may be a quaternion or roll/pitch/yaw, and is optional (a
# camera that never turns still maps, badly).
_TIME_NAMES = ("timestamp", "time", "stamp", "t", "sec", "seconds")


def load_poses(path, body: bool = True) -> PoseTrack:
    """Read a CSV of poses: a header row, then one row a frame.

    Position columns ``x``, ``y``, ``z`` are required, in metres. Orientation
    is either ``qx qy qz qw`` or ``roll pitch yaw`` (radians); a column named
    ``timestamp`` (or ``time``, ``stamp``, ``t``) lets frames be matched by
    capture time instead of by row.

    ``body`` reads the pose as the robot's REP 103 frame, which is what
    odometry reports; pass False when the pose is of the camera optical frame
    itself.
    """
    import csv

    path = Path(path)
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"{path} has no pose rows")
    columns = {name.strip().lower(): name for name in rows[0] if name}
    missing = [axis for axis in "xyz" if axis not in columns]
    if missing:
        raise ValueError(f"{path} has no {', '.join(missing)} column; needs at least x, y, z")
    quaternion = all(f"q{axis}" in columns for axis in "xyzw")
    euler = all(name in columns for name in ("roll", "pitch", "yaw"))
    time_column = next((columns[name] for name in _TIME_NAMES if name in columns), None)

    build = Pose.from_body if body else Pose.from_optical
    poses, times = [], []
    for number, row in enumerate(rows, start=2):  # row 1 is the header
        try:
            position = [float(row[columns[axis]]) for axis in "xyz"]
            if quaternion:
                pose = build(position, quaternion=[float(row[columns[f"q{a}"]]) for a in "xyzw"])
            elif euler:
                pose = build(position, rpy=[float(row[columns[n]]) for n in ("roll", "pitch", "yaw")])
            else:
                pose = build(position, rotation=np.eye(3))
            if time_column is not None:
                times.append(float(row[time_column]))
        except (TypeError, ValueError) as error:
            raise ValueError(f"{path} line {number}: {error}") from None
        poses.append(pose)
    return PoseTrack(poses, np.array(times, float) if time_column is not None else None)


@dataclass(frozen=True)
class MapConfig:
    """What to fuse, at what resolution, and how often.

    ``voxel_m`` is the map's resolution: below the depth noise it only stores
    noise, and the point count (and memory) grows with its cube. 5 cm suits a
    room or a warehouse aisle at a 12 cm baseline.

    ``step`` uses every n-th pixel each way, which is the cheapest way to make
    mapping affordable on a Pi: at step 2 a frame contributes a quarter of the
    points and the voxel grid hides the difference, because neighbouring
    pixels land in the same voxel anyway.

    ``keyframe_distance_m`` and ``keyframe_angle_deg`` skip frames taken from
    somewhere the map has already seen: standing still adds nothing but
    repeated hits, which would also bias the running mean towards wherever the
    robot loitered.
    """

    voxel_m: float = 0.05
    min_range_m: float = 0.3
    max_range_m: float = 6.0
    min_confidence: float = 0.0
    step: int = 2
    keyframe_distance_m: float = 0.15
    keyframe_angle_deg: float = 10.0
    min_hits: int = 2
    max_voxels: int = 5_000_000

    def __post_init__(self) -> None:
        if self.voxel_m <= 0:
            raise ValueError("voxel_m must be positive")
        if not 0 <= self.min_range_m < self.max_range_m:
            raise ValueError("map range must satisfy 0 <= min < max")
        if self.step < 1:
            raise ValueError("step must be at least 1")
        if self.min_hits < 1:
            raise ValueError("min_hits must be at least 1")
        if self.max_voxels < 1:
            raise ValueError("max_voxels must be at least 1")
        if self.keyframe_distance_m < 0 or self.keyframe_angle_deg < 0:
            raise ValueError("keyframe thresholds cannot be negative")


# Voxel indices are packed into one int64 key so a whole frame merges with a
# sorted-array lookup instead of a Python dict. 21 bits an axis is +-1 million
# voxels from the origin, which at 5 cm is +-52 km: far beyond any map that
# fits in memory, and small enough to leave the sign bit alone.
_AXIS_BITS = 21
_AXIS_LIMIT = 1 << (_AXIS_BITS - 1)


def _pack(indices: np.ndarray) -> np.ndarray:
    if np.abs(indices).max(initial=0) >= _AXIS_LIMIT:
        raise ValueError("points are too far from the map origin for this voxel size")
    shifted = (indices + _AXIS_LIMIT).astype(np.int64)
    return (shifted[:, 0] << (2 * _AXIS_BITS)) | (shifted[:, 1] << _AXIS_BITS) | shifted[:, 2]


class _Voxels:
    """One sorted run of voxels: packed keys, and per voxel the sums and count."""

    def __init__(self, keys=None, sums=None, grey=None, hits=None) -> None:
        self.keys = np.empty(0, np.int64) if keys is None else keys
        self.sums = np.empty((0, 3), np.float64) if sums is None else sums
        self.grey = np.empty(0, np.float64) if grey is None else grey
        self.hits = np.empty(0, np.int64) if hits is None else hits

    def __len__(self) -> int:
        return len(self.keys)

    @property
    def nbytes(self) -> int:
        return self.keys.nbytes + self.sums.nbytes + self.grey.nbytes + self.hits.nbytes

    def accumulate(self, keys, sums, grey, hits) -> np.ndarray:
        """Add to the voxels already here; returns which of ``keys`` were not."""
        if not len(self.keys):
            return np.ones(len(keys), bool)
        at = np.clip(np.searchsorted(self.keys, keys), 0, len(self.keys) - 1)
        known = self.keys[at] == keys
        at = at[known]  # each key is unique, so there are no repeated indices
        self.sums[at] += sums[known]
        self.grey[at] += grey[known]
        self.hits[at] += hits[known]
        return ~known

    def insert(self, other: "_Voxels") -> None:
        """Merge in voxels this run does not have yet, keeping it sorted.

        One pass of copying, not a sort: both runs are sorted already, so
        where each new key goes is a binary search.
        """
        if not len(other):
            return
        at = np.searchsorted(self.keys, other.keys)
        self.keys = np.insert(self.keys, at, other.keys)
        self.sums = np.insert(self.sums, at, other.sums, axis=0)
        self.grey = np.insert(self.grey, at, other.grey)
        self.hits = np.insert(self.hits, at, other.hits)


@dataclass
class FloorPlan:
    """The map seen from above: a height band of it flattened onto a grid.

    Cells hold the ROS ``nav_msgs/OccupancyGrid`` values: 100 where the map
    has points, -1 where it has none. Nothing is marked free (0), because
    free space needs the rays between the camera and each point to be traced,
    which this does not do. It is a floor plan, not a costmap.
    """

    cells: np.ndarray          # int8, row 0 is the lowest y (south), as ROS maps are
    resolution: float          # metres per cell
    origin: tuple[float, float]  # map-frame x, y of the lower-left corner
    counts: np.ndarray = field(repr=False, default=None)  # points per cell

    @property
    def occupied(self) -> int:
        return int((self.cells == 100).sum())

    def save(self, path) -> Path:
        """Write ``map.pgm`` and ``map.yaml`` as ROS's map_server reads them."""
        import cv2

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        # map_server greyscale: 0 occupied, 205 unknown, 254 free. Rows run
        # north to south in the image, the opposite of the grid.
        image = np.full(self.cells.shape, 205, np.uint8)
        image[self.cells == 100] = 0
        if not cv2.imwrite(str(path.with_suffix(".pgm")), image[::-1]):
            raise OSError(f"could not write {path.with_suffix('.pgm')}")
        yaml = (
            f"image: {path.with_suffix('.pgm').name}\n"
            f"resolution: {self.resolution}\n"
            f"origin: [{self.origin[0]:.4f}, {self.origin[1]:.4f}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        )
        path.with_suffix(".yaml").write_text(yaml, encoding="utf-8")
        return path.with_suffix(".pgm")


class PointCloudMap:
    """A voxel-averaged point cloud map, built frame by frame at known poses.

        area = PointCloudMap(MapConfig(voxel_m=0.05))
        for frame, pose in stream:
            area.add_frame(frame, pose)
        area.save("map/")

    Not thread-safe: fuse from one thread, or lock around it.
    """

    def __init__(self, config: MapConfig | None = None) -> None:
        self.config = config or MapConfig()
        # Two sorted runs, like a log-structured store: new voxels land in the
        # small one, which is cheap to insert into, and it is folded into the
        # large one only when it has grown to a fraction of it. Inserting into
        # one big sorted array instead copies the whole map every keyframe,
        # which measured 200 ms a frame at 1.6 million voxels.
        self._main = _Voxels()
        self._recent = _Voxels()
        self.frames_seen = 0
        self.frames_integrated = 0
        self.points_fused = 0
        self.truncated = False                   # hit max_voxels and stopped growing
        self.trajectory: list[np.ndarray] = []   # camera position of each integrated frame
        self._last_pose: Pose | None = None

    # -- building -------------------------------------------------------

    def __len__(self) -> int:
        return len(self._main) + len(self._recent)

    def is_keyframe(self, pose: Pose) -> bool:
        """Has the camera moved enough since the last integrated frame?"""
        if self._last_pose is None:
            return True
        return (pose.distance_to(self._last_pose) >= self.config.keyframe_distance_m
                or pose.angle_to(self._last_pose) >= self.config.keyframe_angle_deg)

    def add_frame(self, frame: RobotFrame, pose: Pose, force: bool = False) -> bool:
        """Fuse one :class:`~stereo_vision.outputs.RobotFrame`; True if it was used."""
        self.frames_seen += 1
        if not force and not self.is_keyframe(pose):
            return False
        self.add_points(frame.points, pose, intensity=frame.left, confidence=frame.confidence)
        return True

    def add_depth(self, depth_m: np.ndarray, camera: CameraModel, pose: Pose,
                  intensity: np.ndarray | None = None, confidence: np.ndarray | None = None,
                  force: bool = False) -> bool:
        """Fuse a depth image in metres; True if it was used.

        The same keyframe rule as :meth:`add_frame`, for code that has depth
        and a camera model but no RobotFrame.
        """
        self.frames_seen += 1
        if not force and not self.is_keyframe(pose):
            return False
        self.add_points(points_from_depth(depth_m, camera), pose, intensity, confidence)
        return True

    def add_points(self, points: np.ndarray, pose: Pose, intensity: np.ndarray | None = None,
                   confidence: np.ndarray | None = None) -> int:
        """Fuse an organised or flat point array in the optical frame. Returns points used."""
        config = self.config
        points = np.asarray(points, np.float32)
        organised = points.ndim == 3
        if organised and config.step > 1:
            points = points[::config.step, ::config.step]
            if intensity is not None:
                intensity = intensity[::config.step, ::config.step]
            if confidence is not None:
                confidence = confidence[::config.step, ::config.step]
        flat = points.reshape(-1, 3)

        z = flat[:, 2]
        with np.errstate(invalid="ignore"):
            keep = np.isfinite(flat).all(axis=1) & (z >= config.min_range_m) & (z <= config.max_range_m)
            if confidence is not None and config.min_confidence > 0:
                keep &= np.asarray(confidence, np.float32).reshape(-1) >= config.min_confidence
        if not keep.any():
            self._remember(pose)
            return 0

        world = pose.transform(flat[keep])
        values = None
        if intensity is not None:
            grey = np.asarray(intensity)
            grey = grey.mean(axis=2) if grey.ndim == 3 else grey
            values = grey.reshape(-1)[keep].astype(np.float64)
        self._fuse(world, values)
        self._remember(pose)
        self.points_fused += int(world.shape[0])
        return int(world.shape[0])

    def _remember(self, pose: Pose) -> None:
        self._last_pose = pose
        self.frames_integrated += 1
        self.trajectory.append(pose.translation.copy())

    def _fuse(self, world: np.ndarray, values: np.ndarray | None) -> None:
        """Merge one frame's world-frame points into the voxel arrays."""
        indices = np.floor(world / self.config.voxel_m).astype(np.int64)
        keys = _pack(indices)

        # Aggregate within this frame first: many pixels share a voxel, and
        # bincount over the unique index is far cheaper than scattering.
        unique, inverse = np.unique(keys, return_inverse=True)
        counts = np.bincount(inverse, minlength=len(unique)).astype(np.int64)
        sums = np.column_stack([np.bincount(inverse, weights=world[:, axis].astype(np.float64),
                                            minlength=len(unique)) for axis in range(3)])
        grey = (np.bincount(inverse, weights=values, minlength=len(unique))
                if values is not None else np.zeros(len(unique)))

        fresh = self._main.accumulate(unique, sums, grey, counts)
        if fresh.any():
            fresh[fresh] = self._recent.accumulate(unique[fresh], sums[fresh], grey[fresh], counts[fresh])
        if not fresh.any():
            return
        room = self.config.max_voxels - len(self)
        if room <= 0:
            self.truncated = True
            return
        if int(fresh.sum()) > room:
            # Keep the first ones rather than refusing the frame: a map that
            # stops growing is more useful mid-flight than one that throws.
            self.truncated = True
            chosen = np.flatnonzero(fresh)[:room]
            fresh = np.zeros(len(unique), bool)
            fresh[chosen] = True
        self._recent.insert(_Voxels(unique[fresh], sums[fresh], grey[fresh], counts[fresh]))
        if len(self._recent) > max(65_536, len(self._main) // 8):
            self._main.insert(self._recent)
            self._recent = _Voxels()

    # -- reading out ----------------------------------------------------

    def _column(self, name: str) -> np.ndarray:
        return np.concatenate([getattr(self._main, name), getattr(self._recent, name)])

    def _mask(self, min_hits: int | None) -> np.ndarray:
        hits = max(1, self.config.min_hits if min_hits is None else min_hits)
        return self._column("hits") >= hits

    def points(self, min_hits: int | None = None) -> np.ndarray:
        """(N, 3) float32 of voxel means, in the map frame."""
        keep = self._mask(min_hits)
        if not keep.any():
            return np.empty((0, 3), np.float32)
        return (self._column("sums")[keep] / self._column("hits")[keep, None]).astype(np.float32)

    def intensities(self, min_hits: int | None = None) -> np.ndarray:
        """(N,) uint8 mean greyscale per point, matching :meth:`points`."""
        keep = self._mask(min_hits)
        if not keep.any():
            return np.empty(0, np.uint8)
        return np.clip(self._column("grey")[keep] / self._column("hits")[keep], 0, 255).astype(np.uint8)

    def hits(self, min_hits: int | None = None) -> np.ndarray:
        return self._column("hits")[self._mask(min_hits)]

    def bounds(self, min_hits: int | None = None) -> tuple[np.ndarray, np.ndarray] | None:
        points = self.points(min_hits)
        if not len(points):
            return None
        return points.min(axis=0), points.max(axis=0)

    def ply_bytes(self, min_hits: int | None = None) -> bytes:
        return ply_from_points(self.points(min_hits), self.intensities(min_hits))

    def floor_plan(self, cell_m: float = 0.1, min_height_m: float = -np.inf,
                   max_height_m: float = np.inf, min_points: int = 1,
                   min_hits: int | None = None) -> FloorPlan:
        """Flatten a height band of the map onto a top-down grid.

        Heights are map-frame z, so set the band from the floor up if the map
        frame's origin is on the ground: ``min_height_m=0.1`` drops the floor
        itself, and a ceiling drops out the same way.
        """
        if cell_m <= 0:
            raise ValueError("cell_m must be positive")
        points = self.points(min_hits)
        if len(points):
            band = (points[:, 2] >= min_height_m) & (points[:, 2] <= max_height_m)
            points = points[band]
        if not len(points):
            return FloorPlan(np.full((1, 1), -1, np.int8), cell_m, (0.0, 0.0), np.zeros((1, 1), np.int32))
        low = np.floor(points[:, :2].min(axis=0) / cell_m) * cell_m
        indices = np.floor((points[:, :2] - low) / cell_m).astype(np.int64)
        width = int(indices[:, 0].max()) + 1
        height = int(indices[:, 1].max()) + 1
        counts = np.bincount(indices[:, 1] * width + indices[:, 0],
                             minlength=width * height).reshape(height, width).astype(np.int32)
        cells = np.where(counts >= min_points, 100, -1).astype(np.int8)
        return FloorPlan(cells, cell_m, (float(low[0]), float(low[1])), counts)

    def stats(self, min_hits: int | None = None) -> dict:
        keep = self._mask(min_hits)
        bounds = self.bounds(min_hits)
        config = self.config
        return {
            "voxel_m": config.voxel_m,
            "points": int(keep.sum()),
            "voxels_seen": len(self),
            "min_hits": max(1, config.min_hits if min_hits is None else min_hits),
            "frames_seen": self.frames_seen,
            "frames_integrated": self.frames_integrated,
            "points_fused": self.points_fused,
            "trajectory_m": round(float(sum(
                float(np.linalg.norm(b - a)) for a, b in zip(self.trajectory, self.trajectory[1:]))), 3),
            "bounds_m": None if bounds is None else {
                "min": [round(float(v), 3) for v in bounds[0]],
                "max": [round(float(v), 3) for v in bounds[1]],
            },
            "memory_mb": round((self._main.nbytes + self._recent.nbytes) / 1e6, 2),
            "truncated": self.truncated,
        }

    def save(self, directory, min_hits: int | None = None, cell_m: float = 0.1,
             floor_plan_band: tuple[float, float] = (-np.inf, np.inf)) -> dict[str, Path]:
        """Write ``map.ply``, ``map.pgm`` + ``map.yaml`` and ``map.json`` into a folder."""
        import json

        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        ply = directory / "map.ply"
        ply.write_bytes(self.ply_bytes(min_hits))
        plan = self.floor_plan(cell_m, floor_plan_band[0], floor_plan_band[1], min_hits=min_hits)
        pgm = plan.save(directory / "map.pgm")
        stats = self.stats(min_hits)
        stats["floor_plan"] = {"resolution_m": plan.resolution, "origin_m": list(plan.origin),
                               "size": [int(plan.cells.shape[1]), int(plan.cells.shape[0])],
                               "occupied_cells": plan.occupied}
        stats["trajectory"] = [[round(float(v), 3) for v in position] for position in self.trajectory]
        (directory / "map.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
        return {"ply": ply, "pgm": pgm, "yaml": pgm.with_suffix(".yaml"), "json": directory / "map.json"}
