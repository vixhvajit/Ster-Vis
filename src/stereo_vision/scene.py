"""A ray-traced 3D scene seen through the virtual rig, with exact per-pixel depth.

The chessboard check proves the geometry on corners. This proves the whole
depth pipeline on dense images: rectification, SGBM matching and
triangulation, compared pixel by pixel against the true distance.

Surfaces carry a 3D value-noise texture that is a function of world position,
so both cameras see the same pattern on the same surface, which is what makes
stereo matching possible at all. Shading is Lambertian and therefore the same
from both viewpoints, like a matte real-world surface.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .synthetic import VirtualRig

EPSILON = 1e-6


@dataclass(frozen=True)
class Plane:
    """A plane through ``point`` facing ``normal``; bounded if ``half_extent`` is set."""

    name: str
    point: tuple[float, float, float]
    normal: tuple[float, float, float]
    half_extent: tuple[float, float] | None = None
    albedo: float = 1.0


@dataclass(frozen=True)
class Sphere:
    name: str
    centre: tuple[float, float, float]
    radius: float
    albedo: float = 1.0


@dataclass(frozen=True)
class Box:
    """A box of the given half sizes, turned ``yaw_deg`` about the vertical axis."""

    name: str
    centre: tuple[float, float, float]
    half_size: tuple[float, float, float]
    yaw_deg: float = 0.0
    albedo: float = 1.0


Primitive = Plane | Sphere | Box


def _plane_axes(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    helper = np.array([0.0, 1.0, 0.0]) if abs(normal[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(helper, normal)
    u /= np.linalg.norm(u)
    return u, np.cross(normal, u)


def _intersect_plane(p: Plane, origin: np.ndarray, dirs: np.ndarray):
    normal = np.asarray(p.normal, float)
    normal = normal / np.linalg.norm(normal)
    point = np.asarray(p.point, float)
    denom = dirs @ normal
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((point - origin) @ normal) / denom
    # Reject parallel rays and hits behind the origin; an unbounded plane is
    # otherwise hit by nearly every ray, most of them backwards.
    t = np.where((np.abs(denom) > EPSILON) & (t > EPSILON), t, np.inf)
    if p.half_extent is not None:
        u, v = _plane_axes(normal)
        local = origin + t[:, None] * dirs - point
        inside = (np.abs(local @ u) <= p.half_extent[0]) & (np.abs(local @ v) <= p.half_extent[1])
        t = np.where(inside, t, np.inf)
    return t, np.broadcast_to(normal, dirs.shape)


def _intersect_sphere(s: Sphere, origin: np.ndarray, dirs: np.ndarray):
    centre = np.asarray(s.centre, float)
    offset = origin - centre
    a = np.einsum("ij,ij->i", dirs, dirs)
    b = 2.0 * (dirs @ offset)
    c = offset @ offset - s.radius**2
    disc = b * b - 4 * a * c
    root = np.sqrt(np.maximum(disc, 0.0))
    near = (-b - root) / (2 * a)
    far = (-b + root) / (2 * a)
    t = np.where(near > EPSILON, near, far)
    t = np.where((disc >= 0) & (t > EPSILON), t, np.inf)
    hit = origin + np.where(np.isfinite(t), t, 0.0)[:, None] * dirs
    return t, (hit - centre) / s.radius


def _intersect_box(b: Box, origin: np.ndarray, dirs: np.ndarray):
    yaw = np.deg2rad(b.yaw_deg)
    rotation = np.array(
        [[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]]
    )
    centre = np.asarray(b.centre, float)
    half = np.asarray(b.half_size, float)
    local_origin = rotation.T @ (origin - centre)
    local_dirs = dirs @ rotation
    safe = np.where(np.abs(local_dirs) > EPSILON, local_dirs, EPSILON)
    t1 = (-half - local_origin) / safe
    t2 = (half - local_origin) / safe
    t_near = np.minimum(t1, t2)
    t_far = np.maximum(t1, t2)
    entry = t_near.max(axis=1)
    exit_ = t_far.min(axis=1)
    t = np.where((exit_ >= entry) & (entry > EPSILON), entry, np.inf)
    axis = t_near.argmax(axis=1)
    local_normal = np.zeros_like(local_dirs)
    rows = np.arange(len(dirs))
    local_normal[rows, axis] = -np.sign(safe[rows, axis])
    return t, local_normal @ rotation.T


def _intersect(primitive: Primitive, origin: np.ndarray, dirs: np.ndarray):
    if isinstance(primitive, Plane):
        return _intersect_plane(primitive, origin, dirs)
    if isinstance(primitive, Sphere):
        return _intersect_sphere(primitive, origin, dirs)
    return _intersect_box(primitive, origin, dirs)


class ValueNoise3D:
    """Smooth random texture defined everywhere in space."""

    def __init__(self, seed: int = 7, size: int = 64) -> None:
        self.size = size
        self.lattice = np.random.default_rng(seed).random((size, size, size)).astype(np.float32)

    def __call__(self, points: np.ndarray, cell: float) -> np.ndarray:
        grid = points / cell
        base = np.floor(grid)
        frac = grid - base
        frac = frac * frac * (3 - 2 * frac)
        i = base.astype(np.int64) % self.size
        j = (i + 1) % self.size
        lattice = self.lattice
        result = np.zeros(len(points), np.float32)
        for corner in range(8):
            pick = [(corner >> axis) & 1 for axis in range(3)]
            index = [np.where(pick[a], j[:, a], i[:, a]) for a in range(3)]
            weight = np.ones(len(points), np.float32)
            for a in range(3):
                weight *= frac[:, a] if pick[a] else 1 - frac[:, a]
            result += weight * lattice[index[0], index[1], index[2]]
        return result


@dataclass
class Scene:
    primitives: list[Primitive]
    # Direction from a surface towards the light: above, right and behind the
    # cameras, so every wall in the default scene catches some of it.
    light: tuple[float, float, float] = (0.35, -0.75, -0.55)
    noise: ValueNoise3D | None = None

    def __post_init__(self) -> None:
        self.noise = self.noise or ValueNoise3D()

    def cast(self, origin: np.ndarray, dirs: np.ndarray):
        """Nearest hit for each ray: distance parameter, normal and object id (0 = miss)."""
        best_t = np.full(len(dirs), np.inf)
        best_normal = np.zeros_like(dirs)
        best_id = np.zeros(len(dirs), np.int32)
        for number, primitive in enumerate(self.primitives, start=1):
            t, normal = _intersect(primitive, origin, dirs)
            closer = t < best_t
            best_t = np.where(closer, t, best_t)
            best_normal[closer] = normal[closer]
            best_id[closer] = number
        return best_t, best_normal, best_id

    def shade(self, points: np.ndarray, normals: np.ndarray, dirs: np.ndarray, ids: np.ndarray) -> np.ndarray:
        """Grey level in [0, 1] for each hit point."""
        facing = np.einsum("ij,ij->i", normals, dirs) > 0
        normals = np.where(facing[:, None], -normals, normals)
        light = np.asarray(self.light, float)
        light /= np.linalg.norm(light)
        lambert = np.clip(normals @ light, 0.0, 1.0)

        texture = (
            0.5 * self.noise(points, 6.0)
            + 0.3 * self.noise(points + 1000.0, 17.0)
            + 0.2 * self.noise(points + 2000.0, 50.0)
        )
        texture = np.clip((texture - 0.5) * 2.4 + 0.5, 0.0, 1.0)

        albedo = np.array([1.0] + [p.albedo for p in self.primitives])[ids]
        return (0.12 + 0.88 * texture) * albedo * (0.35 + 0.65 * lambert)


def default_scene() -> Scene:
    """A room corner with objects spread from 0.7 m to 2.2 m.

    Camera frame convention: x right, y down, z forward, millimetres.
    """
    return Scene(
        [
            Plane("back wall", (0, 0, 2200), (0, 0, -1)),
            Plane("floor", (0, 420, 0), (0, -1, 0), albedo=0.85),
            Plane("left wall", (-900, 0, 0), (1, 0, 0), albedo=0.9),
            Sphere("sphere", (-330, 170, 1250), 200, albedo=0.95),
            Box("box", (380, 170, 1500), (170, 250, 170), yaw_deg=32),
            Plane(
                "tilted panel", (60, -150, 760), (0.35, 0.25, -1.0),
                half_extent=(150, 100), albedo=0.9,
            ),
        ]
    )


def _camera_rays(rig: VirtualRig, side: str, pixels: np.ndarray):
    """World-frame (left camera frame) ray origin and directions through raw pixels."""
    camera = rig.left if side == "left" else rig.right
    normalized = cv2.undistortPoints(
        pixels.reshape(-1, 1, 2).astype(np.float64), camera.camera_matrix, camera.dist_coeffs
    ).reshape(-1, 2)
    dirs = np.column_stack([normalized, np.ones(len(normalized))])
    if side == "left":
        return np.zeros(3), dirs
    centre = (-rig.R.T @ rig.T).ravel()
    return centre, dirs @ rig.R  # (R.T @ d.T).T


def render_view(
    scene: Scene,
    rig: VirtualRig,
    side: str,
    supersample: int = 2,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Render one raw, distorted camera image as 8-bit BGR.

    Each pixel averages ``supersample**2`` rays, which anti-aliases the fine
    texture the way a real sensor's pixel area does.
    """
    width, height = rig.image_size
    s = supersample
    xs = (np.arange(width * s) + 0.5) / s - 0.5
    ys = (np.arange(height * s) + 0.5) / s - 0.5
    grid_x, grid_y = np.meshgrid(xs, ys)
    pixels = np.column_stack([grid_x.ravel(), grid_y.ravel()])

    origin, dirs = _camera_rays(rig, side, pixels)
    t, normals, ids = scene.cast(origin, dirs)
    hit = np.isfinite(t)
    grey = np.full(len(dirs), 0.05)
    points = origin + t[hit, None] * dirs[hit]
    grey[hit] = scene.shade(points, normals[hit], dirs[hit], ids[hit])

    image = (grey.reshape(height * s, width * s) * 255.0).astype(np.float32)
    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    image = cv2.GaussianBlur(image, (0, 0), 0.6)
    if rng is not None:
        image += rng.normal(0.0, 1.5, image.shape)
    image = np.clip(image, 0, 255).astype(np.uint8)
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def true_rectified_depth(scene: Scene, rig: VirtualRig, calibration):
    """Exact depth, object id and right-camera visibility for every rectified left pixel.

    Each rectified pixel is traced back through the same remap table the
    pipeline uses to the raw pixel it samples, and from there along the true
    camera ray into the scene. Depth is measured along the rectified optical
    axis, which is what disparity-based depth reports. Calibration error
    therefore shows up in the comparison instead of being assumed away.

    ``visible_right`` is False where the right camera cannot see the point,
    either because something is in the way or because it is out of frame.
    No matcher can recover those pixels, so they are scored separately.
    """
    (map_x, map_y), _ = calibration.rectification_maps(cv2.CV_32FC1)
    height, width = map_x.shape
    raw = np.column_stack([map_x.ravel(), map_y.ravel()])
    raw_w, raw_h = rig.image_size
    in_frame = (raw[:, 0] >= 0) & (raw[:, 0] <= raw_w - 1) & (raw[:, 1] >= 0) & (raw[:, 1] <= raw_h - 1)

    origin, dirs = _camera_rays(rig, "left", raw)
    t, _, ids = scene.cast(origin, dirs)
    valid = in_frame & np.isfinite(t)
    points = np.where(valid[:, None], t[:, None] * dirs, 0.0)
    depth = (points @ calibration.R1.T)[:, 2]
    depth = np.where(valid, depth, np.nan)

    # Visibility from the right camera: trace from its centre to the point.
    centre = (-rig.R.T @ rig.T).ravel()
    to_point = points - centre
    t_right, _, _ = scene.cast(centre, to_point)
    unobstructed = t_right > 1.0 - 1e-4
    in_right = rig.R @ points.T + rig.T.reshape(3, 1)
    projected, _ = cv2.projectPoints(
        in_right.T.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
        rig.right.camera_matrix, rig.right.dist_coeffs,
    )
    projected = projected.reshape(-1, 2)
    in_right_frame = (
        (projected[:, 0] >= 0) & (projected[:, 0] <= raw_w - 1)
        & (projected[:, 1] >= 0) & (projected[:, 1] <= raw_h - 1)
        & (in_right[2] > 0)
    )
    visible_right = valid & unobstructed & in_right_frame

    return (
        depth.reshape(height, width).astype(np.float32),
        np.where(valid, ids, 0).reshape(height, width),
        visible_right.reshape(height, width),
    )
