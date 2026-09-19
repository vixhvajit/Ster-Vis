"""Rectify a stereo pair, compute depth, and optionally export a point cloud.

Works on two image files, or live from two cameras with --live. Live mode runs
on a desktop with a window, or headless on a Raspberry Pi, streaming the view
to a browser with --stream.

Speed is set by --preset (see stereo_vision/presets.py):
  quality   full resolution          balanced  3/4 resolution
  pi5       1/2 resolution           pi5-fast  3/8 resolution
  pi5-bm    1/2 resolution, block matching
--min-distance sets the closest distance to measure; a larger value is faster.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np


from stereo_vision.calibration import load_calibration, rectify_pair
from stereo_vision.depth import (
    colorize_depth,
    disparity_to_depth,
    point_cloud,
    save_depth,
    write_ply,
)
from stereo_vision.disparity import (
    colorize,
    compute_disparity,
    compute_disparity_wls,
    valid_mask,
)
from stereo_vision.live import DepthPipeline, RateMeter, overlay
from stereo_vision.presets import DEFAULT_MIN_DISTANCE_MM, PRESETS, get_preset
from stereo_vision.cli.common import (
    UNIT_TO_M,
    add_camera_args,
    add_robot_args,
    info_dict,
    open_frames,
    scan_config,
)
from stereo_vision.outputs import CameraModel, build_robot_frame


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ster-vis depth", 
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--calibration", type=Path, default=Path("calib/stereo.npz"))
    parser.add_argument("--left", type=Path, default=None, help="left image file")
    parser.add_argument("--right", type=Path, default=None, help="right image file")

    tuning = parser.add_argument_group("speed and accuracy")
    tuning.add_argument("--preset", choices=list(PRESETS), default=None,
                        help="default: quality for files, pi5 for --live")
    tuning.add_argument("--min-distance", type=float, default=DEFAULT_MIN_DISTANCE_MM,
                        help="closest distance to measure, in calibration units (default 500)")
    tuning.add_argument("--num-disparities", type=int, default=None, help="override the preset")
    tuning.add_argument("--block-size", type=int, default=None, help="override the preset")
    tuning.add_argument("--threads", type=int, default=None,
                        help="OpenCV worker threads (default: OpenCV's choice)")
    tuning.add_argument("--wls", action="store_true", help="apply the WLS filter (files only)")

    output = parser.add_argument_group("output (files)")
    output.add_argument("--save", type=Path, default=None, help="write the color disparity image")
    output.add_argument("--depth-out", type=Path, default=None,
                        help="write depth in mm: .png for 16-bit PNG, .npy for float32 "
                             "(open with ster-vis view)")
    output.add_argument("--ply", type=Path, default=None, help="write a point cloud")
    output.add_argument("--max-depth", type=float, default=None,
                        help="drop points beyond this depth from the point cloud")

    live = parser.add_argument_group("live")
    live.add_argument("--live", action="store_true", help="read from two cameras (or --replay)")
    live.add_argument("--headless", action="store_true", help="no window; for a Pi without a display")
    live.add_argument("--stream", type=int, default=None, metavar="PORT",
                      help="serve the live view and the robot API (/api/v1/...) on this port, e.g. 8080")
    live.add_argument("--save-dir", type=Path, default=None,
                      help="save depth PNGs here while running")
    live.add_argument("--save-every", type=float, default=1.0,
                      help="seconds between saved depth maps (default 1)")
    live.add_argument("--duration", type=float, default=None,
                      help="stop after this many seconds (default: run until Q or Ctrl+C)")
    add_camera_args(parser)
    add_robot_args(parser)
    return parser.parse_args(argv)


def report(disparity: np.ndarray, calibration, params) -> None:
    mask = valid_mask(disparity, params)
    print(f"matched {100.0 * mask.mean():.1f}% of pixels")
    if not mask.any():
        print("nothing matched: check rectification and --min-distance")
        return
    depth = disparity_to_depth(disparity, calibration.focal_length_px, calibration.baseline)
    finite = depth[np.isfinite(depth) & mask]
    if finite.size:
        print(f"depth range: {finite.min():.1f} to {finite.max():.1f} (median {np.median(finite):.1f})")


def run_files(args, calibration) -> int:
    if args.left is None or args.right is None:
        print("pass --left and --right, or use --live")
        return 2
    left_raw = cv2.imread(str(args.left), cv2.IMREAD_COLOR)
    right_raw = cv2.imread(str(args.right), cv2.IMREAD_COLOR)
    if left_raw is None or right_raw is None:
        print("could not read one of the input images")
        return 1

    pipeline = DepthPipeline(calibration, get_preset(args.preset or "quality"),
                             args.min_distance, args.num_disparities, args.block_size)
    scaled, params = pipeline.calibration, pipeline.params
    left, right = rectify_pair(left_raw, right_raw, pipeline.maps)
    if args.wls:
        disparity = compute_disparity_wls(left, right, params)
    else:
        disparity = compute_disparity(left, right, params, pipeline.matcher)

    print(f"{scaled.output_size[0]}x{scaled.output_size[1]}, {params.mode}, "
          f"{params.num_disparities} disparities (closest {pipeline.closest_mm:.0f})")
    report(disparity, scaled, params)

    colored = colorize(disparity, params)
    if args.depth_out:
        depth = disparity_to_depth(np.where(valid_mask(disparity, params), disparity, 0),
                                   scaled.focal_length_px, scaled.baseline)
        save_depth(args.depth_out, depth)
        print(f"wrote {args.depth_out}")
        preview = args.depth_out.with_name(args.depth_out.stem + "_color.png")
        cv2.imwrite(str(preview), colorize_depth(depth))
        print(f"wrote {preview}")
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.save), colored)
        print(f"wrote {args.save}")
    if args.ply:
        points, colors = point_cloud(disparity, scaled.Q, left, min_disparity=float(params.min_disparity),
                                     max_depth=args.max_depth)
        write_ply(args.ply, points, colors)
        print(f"wrote {args.ply} with {len(points)} points")
    if not args.save and not args.ply and not args.depth_out:
        cv2.imshow("disparity", colored)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0


def run_live(args, calibration) -> int:
    preset = get_preset(args.preset or "pi5")
    pipeline = DepthPipeline(calibration, preset, args.min_distance, args.num_disparities, args.block_size,
                             confidence=args.confidence)
    camera = CameraModel.from_calibration(pipeline.calibration, UNIT_TO_M[args.unit])
    scan = scan_config(args)
    info = info_dict(camera, pipeline, preset.name)
    near = pipeline.closest_mm
    far = max(near * 6, 3000.0)

    server = None
    if args.stream is not None:
        from stereo_vision.stream import MjpegServer

        server = MjpegServer(args.stream, title=f"preset {preset.name}")
        print("live view at " + "  ".join(server.urls()))
        print(f"robot API at .../api/v1/  (frame, obstacles, scan, depth.png, points.ply, events)")
        print("note: anyone on this network can open that address")
    show_window = not args.headless
    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)

    meter = RateMeter()
    started = last_save = last_print = time.monotonic()
    saved = 0
    source = open_frames(args, calibration)
    recorder = None
    if args.record is not None:
        from stereo_vision.sources import Recorder

        recorder = Recorder(args.record)
        print(f"recording raw frames to {args.record}")
    print(f"{preset.name}: {pipeline.calibration.output_size[0]}x{pipeline.calibration.output_size[1]} "
          f"{pipeline.params.mode}, {pipeline.params.num_disparities} disparities, closest "
          f"{pipeline.closest_mm:.0f}; cameras {source.size[0]}x{source.size[1]}")
    print("Q or ESC quits" if show_window else "Ctrl+C quits")
    try:
        with source:
            first = True
            while True:
                try:
                    frame = source.read()
                except EOFError:
                    print("end of recording")
                    break
                if recorder is not None:
                    recorder.write(frame)
                if first:
                    pipeline.check_size(frame)
                    first = False
                result = pipeline.process(frame)
                meter.tick(result.stage_ms)
                if server is not None:
                    server.publish_robot(build_robot_frame(result, camera, meter.frames, scan,
                                                           UNIT_TO_M[args.unit]), info)

                status = meter.line()
                if frame.skew_ms is not None:
                    status += f"  skew {frame.skew_ms:+.1f} ms"
                now = time.monotonic()
                stream_due = server is not None and server.wants_frame()
                if show_window or stream_due:
                    view = overlay(result, near, far, f"{preset.name}  {status}")
                    if stream_due:
                        server.publish(view)
                    if show_window:
                        cv2.imshow("Ster-Vis live (left | depth)", view)
                        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                            break
                if args.save_dir and now - last_save >= args.save_every:
                    save_depth(args.save_dir / f"depth_{saved:05d}.png", result.depth_mm)
                    saved += 1
                    last_save = now
                if now - last_print >= 2.0:
                    dropped = getattr(source, "dropped", None)
                    extra = f"  dropped {dropped}" if dropped else ""
                    print(status + extra, flush=True)
                    last_print = now
                if args.duration and now - started >= args.duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if show_window:
            cv2.destroyAllWindows()
        if server:
            server.close()
        if recorder is not None:
            recorder.close()
    print(f"{meter.frames} frames, {meter.fps:.1f} fps at the end" + (f", saved {saved} depth maps" if saved else ""))
    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.threads is not None:
        cv2.setNumThreads(args.threads)
    if not args.calibration.exists():
        print(f"no calibration at {args.calibration}; run ster-vis calibrate first")
        return 1
    calibration = load_calibration(args.calibration)
    return run_live(args, calibration) if args.live else run_files(args, calibration)


if __name__ == "__main__":
    raise SystemExit(main())
