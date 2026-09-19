"""Options shared by the live commands (depth --live and ros2)."""

from __future__ import annotations

import argparse
from pathlib import Path

from stereo_vision.outputs import ScanConfig

UNIT_TO_M = {"mm": 0.001, "cm": 0.01, "m": 1.0}


def add_camera_args(parser: argparse.ArgumentParser) -> None:
    group = parser.add_argument_group("cameras")
    group.add_argument("--backend", choices=["auto", "opencv", "picamera2"], default="auto",
                       help="auto uses Pi camera modules when two are attached, else USB")
    group.add_argument("--left-index", type=int, default=0, help="left camera number")
    group.add_argument("--right-index", type=int, default=1, help="right camera number")
    group.add_argument("--width", type=int, default=None, help="capture width (default: calibrated)")
    group.add_argument("--height", type=int, default=None, help="capture height (default: calibrated)")
    group.add_argument("--fps", type=float, default=30.0, help="camera frame rate to request")
    group.add_argument("--fourcc", default=None, help="USB camera format, e.g. MJPG")
    group.add_argument("--focus", type=float, default=1.0, metavar="DIOPTRES",
                       help="Pi cameras with autofocus: fixed focus, 1/metres (default 1.0 = 1 m); "
                            "use the value you calibrated with")
    group.add_argument("--no-thread", action="store_true", help="capture and match in turn")
    group.add_argument("--replay", type=Path, default=None, metavar="DIR",
                       help="play back a recording (or any left/right image folder) instead of cameras")
    group.add_argument("--loop", action="store_true", help="with --replay, start again at the end")
    group.add_argument("--record", type=Path, default=None, metavar="DIR",
                       help="save every raw frame pair here, for --replay later")


def add_robot_args(parser: argparse.ArgumentParser) -> None:
    group = parser.add_argument_group("robot outputs")
    group.add_argument("--confidence", action="store_true",
                       help="compute a per-pixel confidence map (about doubles matching time)")
    group.add_argument("--unit", choices=list(UNIT_TO_M), default="mm",
                       help="unit the calibration was made in, i.e. of --square-size (default mm)")
    group.add_argument("--scan-min-height", type=float, default=-0.25, metavar="M",
                       help="laser scan keeps points from this height relative to the camera (default -0.25)")
    group.add_argument("--scan-max-height", type=float, default=0.25, metavar="M",
                       help="...up to this height (default 0.25)")
    group.add_argument("--scan-range-max", type=float, default=10.0, metavar="M",
                       help="laser scan maximum range (default 10)")
    group.add_argument("--scan-beams", type=int, default=181, help="laser scan beams (default 181)")


def scan_config(args) -> ScanConfig:
    try:
        return ScanConfig(args.scan_beams, args.scan_min_height, args.scan_max_height, 0.2, args.scan_range_max)
    except ValueError as error:
        raise SystemExit(f"ster-vis: {error}") from None


def open_frames(args, calibration):
    """Cameras, or a recording when --replay is given."""
    if args.replay is not None:
        from stereo_vision.sources import ReplaySource

        return ReplaySource(args.replay, grey=True, realtime=True, fps=args.fps, loop=args.loop)
    from stereo_vision.sources import open_source

    width = args.width or calibration.image_size[0]
    height = args.height or calibration.image_size[1]
    return open_source(args.backend, args.left_index, args.right_index, width, height, args.fps,
                       grey=True, threaded=not args.no_thread, fourcc=args.fourcc,
                       focus_dioptres=args.focus)


def info_dict(camera, pipeline, preset_name: str) -> dict:
    """What /api/v1/info reports: enough for a client to interpret everything else."""
    from stereo_vision import __version__

    return {
        "version": __version__,
        "api": "v1",
        "camera": camera.as_dict(),
        "units": {"depth": "m", "depth.png": "mm", "points": "m", "scan": "m", "confidence": "0-100"},
        "frames": {
            "optical": "ster_vis_left_optical_frame (x right, y down, z forward)",
            "body": "ster_vis_link (x forward, y left, z up)",
        },
        "preset": preset_name,
        "matcher": pipeline.params.mode,
        "num_disparities": pipeline.params.num_disparities,
        "closest_m": round(pipeline.closest_mm / 1000.0, 4),
        "confidence": pipeline.right_matcher is not None,
    }
