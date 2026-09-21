"""The point cloud map: frames, fusion, files, and a reconstruction scored against truth.

The reconstruction test builds a room analytically (exact depth by ray casting
against known boxes), maps it from several poses, and measures every mapped
point against the true surfaces. With exact depth in, the only error left is
the map's own: voxel quantisation and the running mean.
"""

from __future__ import annotations

import json
import math

import cv2
import numpy as np
import pytest

from stereo_vision.mapping import (
    MapConfig,
    PointCloudMap,
    Pose,
    load_poses,
    rotation_from_quaternion,
    rotation_from_rpy,
)
from stereo_vision.outputs import CameraModel, points_from_depth

CAMERA = CameraModel(width=160, height=120, fx=120.0, fy=120.0, cx=80.0, cy=60.0, baseline_m=0.12)

# A room 6 x 4 x 2.5 m with its floor at z = 0, and one square pillar in it.
ROOM_LOW = np.array([-3.0, -2.0, 0.0])
ROOM_HIGH = np.array([3.0, 2.0, 2.5])
PILLAR_CENTRE = np.array([1.0, 0.5, 1.25])
PILLAR_HALF = np.array([0.3, 0.3, 1.25])


def wall_depth(distance_m: float) -> np.ndarray:
    return np.full((CAMERA.height, CAMERA.width), distance_m, np.float32)


def optical_rays() -> np.ndarray:
    """Direction per pixel with z = 1, so the ray parameter is the depth itself."""
    u, v = np.meshgrid(np.arange(CAMERA.width, dtype=np.float64),
                       np.arange(CAMERA.height, dtype=np.float64))
    return np.stack([(u - CAMERA.cx) / CAMERA.fx, (v - CAMERA.cy) / CAMERA.fy, np.ones_like(u)], axis=-1)


def room_depth(pose: Pose) -> np.ndarray:
    """Exact depth of the room and pillar seen from ``pose``, by ray casting."""
    directions = optical_rays().reshape(-1, 3) @ pose.rotation.T
    origin = pose.translation

    with np.errstate(divide="ignore", invalid="ignore"):
        # Leaving the room: the nearest positive crossing of the six walls.
        low = (ROOM_LOW - origin) / directions
        high = (ROOM_HIGH - origin) / directions
        exit_t = np.where(directions > 0, high, low).min(axis=1)

        # Entering the pillar: the slab method from outside.
        near = (PILLAR_CENTRE - PILLAR_HALF - origin) / directions
        far = (PILLAR_CENTRE + PILLAR_HALF - origin) / directions
        enter = np.minimum(near, far).max(axis=1)
        leave = np.maximum(near, far).min(axis=1)
    hits_pillar = (leave >= np.maximum(enter, 0.0)) & (enter > 0)
    depth = np.where(hits_pillar, np.minimum(enter, exit_t), exit_t)
    # A ray exactly parallel to a wall never crosses it: no depth, as a matcher
    # would report for a pixel it could not match.
    depth = np.where(np.isfinite(depth), depth, np.nan)
    return depth.reshape(CAMERA.height, CAMERA.width).astype(np.float32)


def distance_to_surface(points: np.ndarray) -> np.ndarray:
    """Distance from each point to the nearest true surface: a wall, or the pillar."""
    walls = np.minimum(np.abs(points - ROOM_LOW), np.abs(points - ROOM_HIGH)).min(axis=1)
    q = np.abs(points - PILLAR_CENTRE) - PILLAR_HALF
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(q.max(axis=1), 0.0)
    return np.minimum(walls, np.abs(outside + inside))


def survey_poses() -> list[Pose]:
    """A camera walked down the room, looking around at each stop."""
    poses = []
    for x in (-2.0, -0.5, 1.0, 2.0):
        for yaw_deg in (0, 90, 180, 270):
            poses.append(Pose.from_body([x, 0.0, 1.2], rpy=[0.0, 0.0, math.radians(yaw_deg)]))
    return poses


@pytest.fixture(scope="module")
def surveyed() -> PointCloudMap:
    area = PointCloudMap(MapConfig(voxel_m=0.05, max_range_m=8.0, step=1, min_hits=1,
                                   keyframe_distance_m=0.0, keyframe_angle_deg=0.0))
    for pose in survey_poses():
        area.add_depth(room_depth(pose), CAMERA, pose, force=True)
    return area


class TestPose:
    def test_optical_points_land_in_ros_axes(self):
        """A point 2 m in front of the camera is 2 m forward of the robot."""
        pose = Pose.from_body([0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0])
        assert np.allclose(pose.transform(np.array([[0.0, 0.0, 2.0]])), [[2.0, 0.0, 0.0]], atol=1e-9)
        # optical x is to the right, which is -y in REP 103; optical y is down, which is -z.
        assert np.allclose(pose.transform(np.array([[1.0, 0.0, 0.0]])), [[0.0, -1.0, 0.0]], atol=1e-9)
        assert np.allclose(pose.transform(np.array([[0.0, 1.0, 0.0]])), [[0.0, 0.0, -1.0]], atol=1e-9)

    def test_yaw_and_translation_compose(self):
        pose = Pose.from_body([1.0, 0.0, 0.5], rpy=[0.0, 0.0, math.pi / 2])
        # 2 m ahead of a camera turned 90 degrees left, standing at (1, 0, 0.5).
        assert np.allclose(pose.transform(np.array([[0.0, 0.0, 2.0]])), [[1.0, 2.0, 0.5]], atol=1e-9)

    def test_quaternion_and_rpy_agree(self):
        roll, pitch, yaw = 0.2, -0.35, 1.1
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        quaternion = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                      cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
        assert np.allclose(rotation_from_quaternion(quaternion), rotation_from_rpy(roll, pitch, yaw))

    def test_inverse_and_distances(self):
        pose = Pose.from_body([2.0, 1.0, 0.0], rpy=[0.0, 0.0, 0.7])
        points = np.array([[0.3, -0.2, 1.5]], np.float32)
        back = pose.inverse().transform(pose.transform(points))
        assert np.allclose(back, points, atol=1e-5)
        other = Pose.from_body([2.5, 1.0, 0.0], rpy=[0.0, 0.0, 0.7 + math.radians(15)])
        assert pose.distance_to(other) == pytest.approx(0.5)
        assert pose.angle_to(other) == pytest.approx(15.0, abs=1e-6)

    def test_a_pose_needs_exactly_one_orientation(self):
        with pytest.raises(ValueError):
            Pose.from_body([0, 0, 0])
        with pytest.raises(ValueError):
            Pose.from_body([0, 0, 0], rpy=[0, 0, 0], quaternion=[0, 0, 0, 1])


class TestFusion:
    def test_a_wall_maps_where_the_camera_was_pointed(self):
        area = PointCloudMap(MapConfig(voxel_m=0.05, step=4, min_hits=1))
        pose = Pose.from_body([0.0, 0.0, 1.0], rpy=[0.0, 0.0, 0.0])
        area.add_depth(wall_depth(2.0), CAMERA, pose)
        points = area.points()
        assert len(points)
        # The wall is 2 m in front of a camera 1 m up, so x = 2 for every point.
        assert np.allclose(points[:, 0], 2.0, atol=0.05)
        assert abs(points[:, 2].mean() - 1.0) < 0.05

    def test_the_same_view_twice_adds_hits_not_voxels(self):
        area = PointCloudMap(MapConfig(voxel_m=0.05, step=4, min_hits=1, keyframe_distance_m=0.0,
                                       keyframe_angle_deg=0.0))
        pose = Pose.identity()
        area.add_depth(wall_depth(2.0), CAMERA, pose, force=True)
        voxels = len(area)
        area.add_depth(wall_depth(2.0), CAMERA, pose, force=True)
        assert len(area) == voxels
        assert area.hits(min_hits=1).min() >= 2

    def test_the_voxel_mean_averages_noise_away(self):
        area = PointCloudMap(MapConfig(voxel_m=0.10, min_hits=1))
        pose = Pose.from_body([0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0])
        for offset in (-0.02, 0.02):  # both inside the voxel spanning 1.0 to 1.1 m
            area.add_points(np.array([[0.0, 0.0, 1.05 + offset]], np.float32), pose)
        assert len(area) == 1
        assert area.points()[0][0] == pytest.approx(1.05, abs=1e-6)

    def test_keyframes_skip_a_camera_that_has_not_moved(self):
        area = PointCloudMap(MapConfig(keyframe_distance_m=0.15, keyframe_angle_deg=10.0))
        assert area.add_depth(wall_depth(2.0), CAMERA, Pose.from_body([0, 0, 0], rpy=[0, 0, 0]))
        assert not area.add_depth(wall_depth(2.0), CAMERA, Pose.from_body([0.1, 0, 0], rpy=[0, 0, 0]))
        assert area.add_depth(wall_depth(2.0), CAMERA, Pose.from_body([0.2, 0, 0], rpy=[0, 0, 0]))
        # Turning on the spot is a new view too.
        assert area.add_depth(wall_depth(2.0), CAMERA,
                              Pose.from_body([0.2, 0, 0], rpy=[0, 0, math.radians(20)]))
        assert area.frames_seen == 4 and area.frames_integrated == 3

    def test_range_limits_and_unknown_depth_are_dropped(self):
        area = PointCloudMap(MapConfig(min_range_m=0.5, max_range_m=3.0, step=1, min_hits=1))
        depth = wall_depth(2.0)
        depth[0, :] = np.nan   # no match
        depth[1, :] = 0.2      # too close to trust
        depth[2, :] = 9.0      # beyond the range
        used = area.add_points(points_from_depth(depth, CAMERA), Pose.identity())
        assert used == (CAMERA.height - 3) * CAMERA.width

    def test_confidence_filters_when_asked(self):
        area = PointCloudMap(MapConfig(min_confidence=50.0, step=1, min_hits=1))
        confidence = np.full((CAMERA.height, CAMERA.width), 80.0, np.float32)
        confidence[: CAMERA.height // 2] = 10.0
        used = area.add_points(points_from_depth(wall_depth(2.0), CAMERA), Pose.identity(),
                               confidence=confidence)
        assert used == (CAMERA.height // 2) * CAMERA.width

    def test_min_hits_drops_voxels_seen_once(self, surveyed):
        once = len(surveyed.points(min_hits=1))
        many = len(surveyed.points(min_hits=5))
        assert 0 < many < once

    def test_a_large_map_agrees_with_fusing_everything_at_once(self):
        """New voxels are kept in a small run that is folded into the main one as it grows;
        whatever the order, the map must equal one computed in a single pass."""
        rng = np.random.default_rng(5)
        points = rng.uniform(-5, 5, (240_000, 3)).astype(np.float32)
        points[120_000:] = points[:120_000] + rng.normal(0, 0.004, (120_000, 3))  # seen twice
        area = PointCloudMap(MapConfig(voxel_m=0.05, min_hits=1, max_range_m=100.0, min_range_m=0.0))
        identity = Pose.identity()
        # The range check reads z, so put the points in front of the camera.
        shifted = points + np.array([0.0, 0.0, 50.0], np.float32)
        for chunk in np.array_split(shifted, 24):
            area.add_points(chunk, identity)
        assert len(area._main) > 0  # the fold happened

        keys, inverse, counts = np.unique(np.floor(shifted / 0.05).astype(np.int64), axis=0,
                                          return_inverse=True, return_counts=True)
        inverse = inverse.reshape(-1)
        assert len(area) == len(keys)
        assert area.hits(min_hits=1).sum() == len(points)
        expected = np.column_stack([np.bincount(inverse, shifted[:, a].astype(np.float64))
                                    for a in range(3)]) / counts[:, None]
        got = area.points(min_hits=1)
        order_got = np.lexsort(np.floor(got / 0.05).astype(np.int64).T[::-1])
        order_expected = np.lexsort(keys.T[::-1])
        assert np.allclose(got[order_got], expected[order_expected], atol=1e-4)

    def test_the_map_stops_growing_at_max_voxels(self):
        area = PointCloudMap(MapConfig(voxel_m=0.02, max_voxels=500, min_hits=1))
        area.add_depth(wall_depth(2.0), CAMERA, Pose.identity())
        assert area.truncated
        assert len(area) == 500

    def test_points_far_from_the_origin_are_refused(self):
        area = PointCloudMap(MapConfig(voxel_m=0.05, min_hits=1))
        far = Pose.from_body([200_000.0, 0.0, 0.0], rpy=[0, 0, 0])
        with pytest.raises(ValueError, match="too far from the map origin"):
            area.add_points(np.array([[0.0, 0.0, 1.0]], np.float32), far)


class TestReconstruction:
    """Exact depth in, so what is measured is the map, not the stereo matching."""

    def test_every_mapped_point_lies_on_a_real_surface(self, surveyed):
        error = distance_to_surface(surveyed.points(min_hits=1))
        assert np.median(error) < 0.02
        assert np.percentile(error, 90) < 0.05
        assert error.max() < 0.12  # half a voxel diagonal, plus the mean of a slanted surface

    def test_the_whole_room_is_reconstructed(self, surveyed):
        points = surveyed.points(min_hits=1)
        near = lambda mask: int(mask.sum())  # noqa: E731
        assert near(np.abs(points[:, 2]) < 0.05) > 500                      # floor
        assert near(np.abs(points[:, 2] - ROOM_HIGH[2]) < 0.05) > 500       # ceiling
        for axis, bound in ((0, ROOM_LOW[0]), (0, ROOM_HIGH[0]), (1, ROOM_LOW[1]), (1, ROOM_HIGH[1])):
            assert near(np.abs(points[:, axis] - bound) < 0.05) > 200, f"wall {axis} {bound} missing"
        pillar = (np.abs(points[:, :2] - PILLAR_CENTRE[:2]) <= PILLAR_HALF[:2] + 0.06).all(axis=1)
        assert near(pillar) > 200

    def test_the_extent_matches_the_room(self, surveyed):
        low, high = surveyed.bounds(min_hits=1)
        assert np.allclose(low, ROOM_LOW, atol=0.08)
        assert np.allclose(high, ROOM_HIGH, atol=0.08)

    def test_more_viewpoints_map_more_of_the_room(self):
        def voxels(poses):
            area = PointCloudMap(MapConfig(voxel_m=0.05, max_range_m=8.0, min_hits=1,
                                           keyframe_distance_m=0.0, keyframe_angle_deg=0.0))
            for pose in poses:
                area.add_depth(room_depth(pose), CAMERA, pose, force=True)
            return len(area)

        poses = survey_poses()
        assert voxels(poses[:1]) < voxels(poses[:4]) < voxels(poses)


class TestFloorPlan:
    def test_a_pillar_shows_up_where_it_stands(self, surveyed):
        plan = surveyed.floor_plan(cell_m=0.1, min_height_m=0.3, max_height_m=2.2, min_hits=1)
        column = int((PILLAR_CENTRE[0] - plan.origin[0]) / plan.resolution)
        row = int((PILLAR_CENTRE[1] - plan.origin[1]) / plan.resolution)
        # Its faces are mapped, so the footprint is drawn; the inside was never
        # seen, so the middle of it stays unknown, as it should.
        footprint = plan.cells[row - 3:row + 4, column - 3:column + 4]
        assert (footprint == 100).sum() >= 8
        assert plan.cells[row, column] == -1
        # The floor is below the band, so the middle of an empty aisle is unknown.
        empty_row = int((-1.5 - plan.origin[1]) / plan.resolution)
        empty_column = int((-1.0 - plan.origin[0]) / plan.resolution)
        assert plan.cells[empty_row, empty_column] == -1

    def test_the_band_decides_what_is_flattened(self, surveyed):
        floor_only = surveyed.floor_plan(cell_m=0.2, min_height_m=-0.1, max_height_m=0.1, min_hits=1)
        assert floor_only.occupied > 100
        above_the_ceiling = surveyed.floor_plan(cell_m=0.2, min_height_m=3.0, min_hits=1)
        assert above_the_ceiling.occupied == 0

    def test_it_saves_what_map_server_reads(self, surveyed, tmp_path):
        plan = surveyed.floor_plan(cell_m=0.1, min_height_m=0.3, max_height_m=2.2, min_hits=1)
        pgm = plan.save(tmp_path / "map.pgm")
        image = cv2.imread(str(pgm), cv2.IMREAD_UNCHANGED)
        assert image.shape == plan.cells.shape
        assert set(np.unique(image)) <= {0, 205, 254}
        yaml = (tmp_path / "map.yaml").read_text(encoding="utf-8")
        assert "image: map.pgm" in yaml and "resolution: 0.1" in yaml
        # ROS images run north to south, the grid south to north.
        assert (image[::-1] == 0).sum() == plan.occupied


class TestFiles:
    def test_the_ply_holds_every_point(self, surveyed):
        data = surveyed.ply_bytes(min_hits=1)
        header, body = data.split(b"end_header\n", 1)
        count = int(header.split(b"element vertex ")[1].split()[0])
        assert count == len(surveyed.points(min_hits=1))
        record = np.frombuffer(body, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                            ("red", "u1"), ("green", "u1"), ("blue", "u1")])
        assert len(record) == count
        assert np.allclose(np.column_stack([record["x"], record["y"], record["z"]]),
                           surveyed.points(min_hits=1), atol=1e-6)

    def test_save_writes_the_map_the_plan_and_the_numbers(self, surveyed, tmp_path):
        written = surveyed.save(tmp_path / "map", min_hits=1, cell_m=0.1)
        assert written["ply"].stat().st_size > 1000
        stats = json.loads(written["json"].read_text(encoding="utf-8"))
        assert stats["points"] == len(surveyed.points(min_hits=1))
        assert stats["frames_integrated"] == len(survey_poses())
        assert stats["voxel_m"] == 0.05 and stats["truncated"] is False
        assert len(stats["trajectory"]) == len(survey_poses())
        assert stats["bounds_m"]["max"][2] == pytest.approx(ROOM_HIGH[2], abs=0.08)
        assert (tmp_path / "map" / "map.yaml").is_file()

    def test_intensity_comes_back_as_grey(self):
        area = PointCloudMap(MapConfig(voxel_m=0.05, step=4, min_hits=1))
        grey = np.full((CAMERA.height, CAMERA.width), 200, np.uint8)
        area.add_depth(wall_depth(2.0), CAMERA, Pose.identity(), intensity=grey)
        assert set(np.unique(area.intensities())) == {200}


class TestPoseFiles:
    def write(self, tmp_path, text: str):
        path = tmp_path / "poses.csv"
        path.write_text(text, encoding="utf-8")
        return path

    def test_quaternion_rows(self, tmp_path):
        track = load_poses(self.write(tmp_path, "timestamp,x,y,z,qx,qy,qz,qw\n"
                                               "10.0,1,2,3,0,0,0,1\n11.0,2,2,3,0,0,0,1\n"))
        assert len(track) == 2
        assert np.allclose(track.at_index(0).translation, [1, 2, 3])
        assert track.at_time(10.9, tolerance_s=0.2) is track.poses[1]
        assert track.at_time(20.0) is None

    def test_roll_pitch_yaw_rows_without_time(self, tmp_path):
        track = load_poses(self.write(tmp_path, "x,y,z,roll,pitch,yaw\n0,0,1,0,0,1.5708\n"))
        assert track.times is None
        assert np.allclose(track.at_index(0).transform(np.array([[0.0, 0.0, 2.0]])), [[0, 2, 1]], atol=1e-4)
        assert track.at_index(5) is None

    def test_optical_poses_skip_the_body_rotation(self, tmp_path):
        path = self.write(tmp_path, "x,y,z,qx,qy,qz,qw\n0,0,0,0,0,0,1\n")
        optical = load_poses(path, body=False).at_index(0)
        assert np.allclose(optical.transform(np.array([[0.0, 0.0, 2.0]])), [[0, 0, 2]])

    def test_a_file_without_positions_is_refused(self, tmp_path):
        with pytest.raises(ValueError, match="needs at least x, y, z"):
            load_poses(self.write(tmp_path, "timestamp,qx,qy,qz,qw\n1,0,0,0,1\n"))

    def test_a_bad_number_names_its_line(self, tmp_path):
        with pytest.raises(ValueError, match="line 3"):
            load_poses(self.write(tmp_path, "x,y,z\n1,2,3\n1,two,3\n"))


class TestConfig:
    @pytest.mark.parametrize("kwargs", [
        {"voxel_m": 0.0},
        {"min_range_m": 5.0, "max_range_m": 1.0},
        {"step": 0},
        {"min_hits": 0},
        {"keyframe_distance_m": -1.0},
    ])
    def test_settings_that_would_make_a_wrong_map_are_refused(self, kwargs):
        with pytest.raises(ValueError):
            MapConfig(**kwargs)


@pytest.fixture(scope="module")
def recording(tmp_path_factory):
    """Two frames of the ray-traced scene as a recording, with a pose for each."""
    from stereo_vision.scene import default_scene, render_view
    from stereo_vision.sources import Recorder, StereoFrame
    from stereo_vision.synthetic import default_rig, true_calibration

    path = tmp_path_factory.mktemp("recording")
    rig, scene = default_rig(), default_scene()
    true_calibration(rig).save(path / "rig.npz")
    left = cv2.cvtColor(render_view(scene, rig, "left", supersample=1), cv2.COLOR_BGR2GRAY)
    right = cv2.cvtColor(render_view(scene, rig, "right", supersample=1), cv2.COLOR_BGR2GRAY)
    with Recorder(path / "rec") as recorder:
        for index in range(2):
            recorder.write(StereoFrame(left, right, timestamp=1000.0 + index))
    # The camera moved half a metre forward between the two frames: the same
    # scene, fused half a metre further along the map's x axis.
    (path / "poses.csv").write_text("timestamp,x,y,z,roll,pitch,yaw\n"
                                    "1000.0,0,0,0,0,0,0\n1001.0,0.5,0,0,0,0,0\n", encoding="utf-8")
    return path


class TestCommand:
    """`ster-vis map` end to end: a rendered recording plus a pose file."""

    def run(self, recording, tmp_path, *extra):
        from stereo_vision.cli import main as cli

        return cli.main(["map", "--replay", str(recording / "rec"), "--poses",
                         str(recording / "poses.csv"), "--calibration", str(recording / "rig.npz"),
                         "--out", str(tmp_path / "map"), "--min-hits", "1", *extra])

    def test_it_maps_a_recording_at_its_poses(self, recording, tmp_path, capsys):
        assert self.run(recording, tmp_path) == 0
        out = capsys.readouterr().out
        assert "2 frames, 2 poses matched by timestamp" in out
        stats = json.loads((tmp_path / "map" / "map.json").read_text())
        assert stats["frames_integrated"] == 2 and stats["points"] > 5000
        assert (tmp_path / "map" / "map.ply").stat().st_size > 10_000
        assert (tmp_path / "map" / "map.pgm").is_file() and (tmp_path / "map" / "map.yaml").is_file()
        # The scene's nearest surface is 0.76 m ahead and its back wall 2.2 m,
        # and the second frame was fused half a metre further on.
        assert stats["bounds_m"]["min"][0] == pytest.approx(0.76, abs=0.25)
        assert 2.5 < stats["bounds_m"]["max"][0] < 4.5

    def test_poses_beside_the_recording_are_found(self, recording, tmp_path):
        import shutil

        from stereo_vision.cli import main as cli

        shutil.copy(recording / "poses.csv", recording / "rec" / "poses.csv")
        try:
            assert cli.main(["map", "--replay", str(recording / "rec"), "--calibration",
                             str(recording / "rig.npz"), "--out", str(tmp_path / "map")]) == 0
        finally:
            (recording / "rec" / "poses.csv").unlink()
        assert (tmp_path / "map" / "map.ply").is_file()

    def test_poses_that_match_no_frame_are_reported(self, recording, tmp_path, capsys):
        (tmp_path / "far.csv").write_text("timestamp,x,y,z,roll,pitch,yaw\n"
                                          "5000.0,0,0,0,0,0,0\n", encoding="utf-8")
        from stereo_vision.cli import main as cli

        code = cli.main(["map", "--replay", str(recording / "rec"), "--poses", str(tmp_path / "far.csv"),
                         "--calibration", str(recording / "rig.npz"), "--out", str(tmp_path / "none")])
        assert code == 1
        assert "nothing was mapped" in capsys.readouterr().out

    def test_a_missing_pose_file_is_a_clear_error(self, recording, tmp_path, capsys):
        from stereo_vision.cli import main as cli

        assert cli.main(["map", "--replay", str(recording / "rec"), "--poses", str(tmp_path / "no.csv"),
                         "--calibration", str(recording / "rig.npz")]) == 1
        assert "no pose file" in capsys.readouterr().out
