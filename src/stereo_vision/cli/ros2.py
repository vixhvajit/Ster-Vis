"""Run the stereo camera as a ROS 2 node.

  source /opt/ros/jazzy/setup.bash
  ster-vis ros2 --calibration calib/stereo.npz --parent-frame base_link --mount 0.1 0 0.3 0 0 0

Publishes depth, point cloud, laser scan, obstacle ranges, images and camera
info, plus static TF, under /ster_vis (see stereo_vision/ros2.py for the full
topic list). Anything after --ros-args is passed to ROS unchanged, for
example: --ros-args -r __ns:=/robot/front_camera
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2

from stereo_vision.calibration import load_calibration
from stereo_vision.cli.common import UNIT_TO_M, add_camera_args, add_robot_args, open_frames, scan_config
from stereo_vision.live import DepthPipeline, RateMeter
from stereo_vision.outputs import CameraModel, build_robot_frame
from stereo_vision.presets import DEFAULT_MIN_DISTANCE_MM, PRESETS, get_preset


def parse_args(argv: list[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    argv = list(argv or [])
    ros_args = []
    if "--ros-args" in argv:
        split = argv.index("--ros-args")
        argv, ros_args = argv[:split], argv[split:]
    parser = argparse.ArgumentParser(prog="ster-vis ros2", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--calibration", type=Path, default=Path("calib/stereo.npz"))
    parser.add_argument("--preset", choices=list(PRESETS), default="pi5")
    parser.add_argument("--min-distance", type=float, default=DEFAULT_MIN_DISTANCE_MM,
                        help="closest distance to measure, in calibration units (default 500)")
    parser.add_argument("--threads", type=int, default=None, help="OpenCV worker threads")
    parser.add_argument("--namespace", default="ster_vis", help="topic namespace (default ster_vis)")
    parser.add_argument("--parent-frame", default="base_link",
                        help="TF frame the camera is mounted on (default base_link)")
    parser.add_argument("--mount", type=float, nargs=6, default=[0.0] * 6,
                        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
                        help="camera pose on the parent frame: metres and radians, REP 103 axes")
    parser.add_argument("--duration", type=float, default=None, help="stop after this many seconds")
    add_camera_args(parser)
    add_robot_args(parser)
    return parser.parse_args(argv), ros_args


def main(argv: list[str] | None = None) -> int:
    args, ros_args = parse_args(argv)
    if args.threads is not None:
        cv2.setNumThreads(args.threads)
    if not args.calibration.exists():
        print(f"no calibration at {args.calibration}; run ster-vis calibrate first")
        return 1

    from stereo_vision.ros2 import RosPublisher

    calibration = load_calibration(args.calibration)
    preset = get_preset(args.preset)
    pipeline = DepthPipeline(calibration, preset, args.min_distance, confidence=args.confidence)
    unit = UNIT_TO_M[args.unit]
    camera = CameraModel.from_calibration(pipeline.calibration, unit)
    scan = scan_config(args)
    try:
        publisher = RosPublisher(camera, args.namespace, args.parent_frame,
                                 tuple(args.mount[:3]), tuple(args.mount[3:]),
                                 confidence=args.confidence, ros_args=["ster-vis"] + ros_args)
    except RuntimeError as error:
        print(error)
        return 1

    node = publisher.node
    node.get_logger().info(
        f"{preset.name}: {camera.width}x{camera.height} {pipeline.params.mode}, closest "
        f"{pipeline.closest_mm * unit:.2f} m; publishing under /{args.namespace.strip('/')}"
    )
    source = open_frames(args, calibration)
    recorder = None
    if args.record is not None:
        from stereo_vision.sources import Recorder

        recorder = Recorder(args.record)
    meter = RateMeter()
    started = last_log = time.monotonic()
    try:
        with source:
            first = True
            while publisher.ok():
                try:
                    frame = source.read()
                except EOFError:
                    node.get_logger().info("end of recording")
                    break
                if recorder is not None:
                    recorder.write(frame)
                if first:
                    pipeline.check_size(frame)
                    first = False
                result = pipeline.process(frame)
                publisher.publish(build_robot_frame(result, camera, meter.frames, scan, unit))
                meter.tick(result.stage_ms)
                now = time.monotonic()
                if now - last_log >= 5.0:
                    node.get_logger().info(meter.line())
                    last_log = now
                if args.duration and now - started >= args.duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if recorder is not None:
            recorder.close()
        publisher.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
