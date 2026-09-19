"""The ``ster-vis`` command: one entry point for every tool."""

from __future__ import annotations

import importlib
import sys

from stereo_vision import __version__

# name: (module, one-line summary). Modules load only when their command runs,
# so ``ster-vis --help`` stays fast.
COMMANDS = {
    "chessboard": ("chessboard", "write a printable calibration target (PDF)"),
    "capture": ("capture", "capture calibration pairs; --auto for a headless Pi"),
    "calibrate": ("calibrate", "solve the rig from captured pairs"),
    "depth": ("depth", "depth from two images, or live; --stream serves the robot API"),
    "ros2": ("ros2", "run as a ROS 2 node: depth, point cloud, laser scan, TF"),
    "view": ("view", "inspect a depth map: hover for mm, click to measure"),
    "viewer": ("viewer", "open the browser viewer for point clouds and depth maps"),
    "benchmark": ("benchmark", "frame rate per preset on this machine; --accuracy too"),
    "doctor": ("doctor", "check this install: versions, cameras, calibration"),
    "upgrade": ("upgrade", "check for, verify and install another release"),
    "check": ("check", "calibration check against a virtual rig with known geometry"),
    "scene": ("scene", "depth map of a synthetic scene, scored against truth"),
}

USAGE = """usage: ster-vis <command> [options]

Stereo depth from two cameras.

commands:
{commands}

Run 'ster-vis <command> --help' for a command's options.
Typical order: chessboard, capture, calibrate, depth.
"""


def usage() -> str:
    width = max(map(len, COMMANDS))
    lines = "\n".join(f"  {name:<{width}}  {summary}" for name, (_, summary) in COMMANDS.items())
    return USAGE.format(commands=lines)


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(usage())
        return 0
    if argv[0] in ("-V", "--version"):
        print(f"ster-vis {__version__}")
        return 0
    command, rest = argv[0], argv[1:]
    if command not in COMMANDS:
        print(f"ster-vis: unknown command {command!r}\n", file=sys.stderr)
        print(usage(), file=sys.stderr)
        return 2
    module = importlib.import_module(f"stereo_vision.cli.{COMMANDS[command][0]}")
    return int(module.main(rest) or 0)


if __name__ == "__main__":
    raise SystemExit(main())
