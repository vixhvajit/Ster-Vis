"""Build one point cloud map of a place from a recording and its poses.

    ster-vis map --replay run1/ --poses run1/poses.csv --out map/

Every frame is rectified, matched and turned into a point cloud exactly as
`ster-vis depth` does, then placed in the map frame using the pose supplied
for it and fused into a voxel grid. What comes out is a metric reconstruction
of the place the camera moved through:

    map/map.ply     the map as a point cloud (MeshLab, CloudCompare, ster-vis viewer)
    map/map.pgm     a top-down floor plan, with map.yaml, as ROS's map_server reads
    map/map.json    what was fused, the extent, and the trajectory

Ster-Vis does not work out where the camera was; the poses come from your
robot. `--poses` is a CSV with a header and columns x, y, z (metres) plus
either qx, qy, qz, qw or roll, pitch, yaw (radians), and optionally a
timestamp, which is matched against the recording's own frame times. Without
timestamps, row 1 is frame 1. A recording with its poses written to
`poses.csv` beside `frames.csv` needs no `--poses` at all. For a live robot
running ROS 2, use `ster-vis ros2 --map` instead, which takes the pose from
TF.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from stereo_vision.calibration import load_calibration
from stereo_vision.cli.common import UNIT_TO_M
from stereo_vision.live import DepthPipeline
from stereo_vision.mapping import MapConfig, PointCloudMap, load_poses
from stereo_vision.outputs import CameraModel
from stereo_vision.presets import DEFAULT_MIN_DISTANCE_MM, PRESETS, get_preset
from stereo_vision.sources import ReplaySource


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ster-vis map", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--replay", type=Path, required=True, metavar="DIR",
                        help="recording to map (ster-vis depth --record writes one)")
    parser.add_argument("--poses", type=Path, default=None, metavar="CSV",
                        help="pose per frame, from your robot's odometry or TF "
                             "(default: poses.csv inside the recording)")
    parser.add_argument("--out", type=Path, default=Path("map"), help="output folder (default map/)")
    parser.add_argument("--calibration", type=Path, default=Path("calib/stereo.npz"))
    parser.add_argument("--preset", choices=list(PRESETS), default="quality")
    parser.add_argument("--min-distance", type=float, default=DEFAULT_MIN_DISTANCE_MM,
                        help="closest distance to measure, in calibration units (default 500)")
    parser.add_argument("--unit", choices=list(UNIT_TO_M), default="mm",
                        help="unit the calibration was made in (default mm)")
    parser.add_argument("--confidence", action="store_true",
                        help="compute the left-right confidence map and use --min-confidence")
    parser.add_argument("--threads", type=int, default=None, help="OpenCV worker threads")

    fusing = parser.add_argument_group("map")
    fusing.add_argument("--voxel", type=float, default=0.05, metavar="M",
                        help="map resolution in metres (default 0.05)")
    fusing.add_argument("--min-range", type=float, default=0.3, metavar="M",
                        help="ignore depth closer than this (default 0.3)")
    fusing.add_argument("--max-range", type=float, default=6.0, metavar="M",
                        help="ignore depth further than this; stereo error grows with distance "
                             "squared, so far points blur the map (default 6)")
    fusing.add_argument("--min-confidence", type=float, default=0.0, metavar="0-100",
                        help="with --confidence, drop pixels scoring below this (default 0 = keep all)")
    fusing.add_argument("--step", type=int, default=2,
                        help="use every n-th pixel each way (default 2)")
    fusing.add_argument("--keyframe-distance", type=float, default=0.15, metavar="M",
                        help="skip frames taken within this distance of the last one (default 0.15)")
    fusing.add_argument("--keyframe-angle", type=float, default=10.0, metavar="DEG",
                        help="...and within this angle of it (default 10)")
    fusing.add_argument("--min-hits", type=int, default=2,
                        help="keep voxels seen by at least this many points (default 2)")
    fusing.add_argument("--max-voxels", type=int, default=5_000_000,
                        help="safety cap on map size (default 5 million)")
    fusing.add_argument("--pose-frame", choices=["body", "optical"], default="body",
                        help="are the poses of the robot body (REP 103, default) or of the "
                             "camera's optical frame?")
    fusing.add_argument("--pose-tolerance", type=float, default=0.1, metavar="S",
                        help="how far a pose may be from a frame's timestamp (default 0.1)")

    plan = parser.add_argument_group("floor plan")
    plan.add_argument("--cell", type=float, default=0.1, metavar="M",
                      help="floor plan resolution (default 0.1)")
    plan.add_argument("--floor-band", type=float, nargs=2, default=None, metavar=("MIN", "MAX"),
                      help="only map points in this height band reach the floor plan, in map-frame "
                           "metres (default: all of them)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.threads is not None:
        cv2.setNumThreads(args.threads)
    if not args.calibration.exists():
        print(f"no calibration at {args.calibration}; run ster-vis calibrate first")
        return 1
    poses = args.poses or args.replay / "poses.csv"
    if not poses.exists():
        print(f"no pose file at {poses}; mapping needs a pose per frame, from your "
              "robot's odometry (see --poses)")
        return 1

    try:
        track = load_poses(poses, body=args.pose_frame == "body")
    except (ValueError, OSError) as error:
        print(f"ster-vis map: {error}")
        return 1
    try:
        source = ReplaySource(args.replay, grey=True, realtime=False, original_times=True)
    except FileNotFoundError as error:
        print(f"ster-vis map: {error}")
        return 1

    calibration = load_calibration(args.calibration)
    pipeline = DepthPipeline(calibration, get_preset(args.preset), args.min_distance,
                             confidence=args.confidence)
    unit = UNIT_TO_M[args.unit]
    camera = CameraModel.from_calibration(pipeline.calibration, unit)
    config = MapConfig(voxel_m=args.voxel, min_range_m=args.min_range, max_range_m=args.max_range,
                       min_confidence=args.min_confidence, step=args.step,
                       keyframe_distance_m=args.keyframe_distance, keyframe_angle_deg=args.keyframe_angle,
                       min_hits=args.min_hits, max_voxels=args.max_voxels)
    area = PointCloudMap(config)

    by_time = track.times is not None
    print(f"{len(source)} frames, {len(track)} poses matched by "
          f"{'timestamp' if by_time else 'row'}; fusing at {config.voxel_m * 100:.0f} cm")
    unmatched, started = 0, time.monotonic()
    with source:
        for index in range(len(source)):
            try:
                frame = source.read()
            except EOFError:
                break
            if index == 0:
                pipeline.check_size(frame)
            pose = (track.at_time(frame.timestamp, args.pose_tolerance) if by_time
                    else track.at_index(index))
            if pose is None:
                unmatched += 1
                continue
            if not area.is_keyframe(pose):
                area.frames_seen += 1
                continue
            result = pipeline.process(frame)
            depth_m = (result.depth_mm * unit).astype(np.float32)
            area.add_depth(depth_m, camera, pose, intensity=result.left,
                           confidence=result.confidence, force=True)
            if area.frames_integrated % 20 == 0:
                print(f"  frame {index + 1}/{len(source)}: {len(area):,} voxels", flush=True)

    if not len(area):
        print("nothing was mapped: no frame had both depth and a pose "
              f"({unmatched} frames had no pose within {args.pose_tolerance} s)")
        return 1

    band = tuple(args.floor_band) if args.floor_band else (-np.inf, np.inf)
    written = area.save(args.out, cell_m=args.cell, floor_plan_band=band)
    stats = json.loads(written["json"].read_text(encoding="utf-8"))
    bounds = stats["bounds_m"]
    print(f"\nmapped {stats['points']:,} points from {stats['frames_integrated']} of "
          f"{stats['frames_seen']} frames in {time.monotonic() - started:.1f} s")
    if unmatched:
        print(f"{unmatched} frames had no pose within {args.pose_tolerance} s and were skipped")
    if stats["truncated"]:
        print(f"the map hit --max-voxels ({config.max_voxels:,}) and stopped growing; "
              "use a larger --voxel or a shorter --max-range")
    print(f"extent {bounds['min']} to {bounds['max']} m, "
          f"path {stats['trajectory_m']} m, {stats['memory_mb']} MB")
    for name in ("ply", "pgm", "json"):
        print(f"  {written[name]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
