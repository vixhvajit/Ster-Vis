"""Check that this install can run: versions, cameras, calibration, and Pi health.

Prints one line per check, marked ok, warn or FAIL, with a fix where one is
known. Exits non-zero if any check fails, so it can gate a deployment script.
"""

from __future__ import annotations

import argparse
import platform
import sys
from pathlib import Path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ster-vis doctor", description=__doc__)
    parser.add_argument("--calibration", type=Path, default=Path("calib/stereo.npz"))
    parser.add_argument("--cameras", action="store_true",
                        help="also open the cameras and read one frame pair")
    parser.add_argument("--backend", choices=["auto", "opencv", "picamera2"], default="auto")
    return parser.parse_args(argv)


class Report:
    def __init__(self) -> None:
        self.failed = False

    def line(self, status: str, what: str, detail: str = "", fix: str = "") -> None:
        mark = {"ok": "  ok ", "warn": " warn", "fail": " FAIL"}[status]
        print(f"{mark}  {what}" + (f": {detail}" if detail else ""))
        if fix:
            print(f"        fix: {fix}")
        self.failed |= status == "fail"


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = Report()
    from stereo_vision import __version__

    print(f"ster-vis {__version__} on Python {platform.python_version()}, {platform.system()} {platform.machine()}\n")

    if sys.version_info < (3, 11):
        report.line("fail", "Python", platform.python_version(), "Python 3.11 or newer is required")
    else:
        report.line("ok", "Python", platform.python_version())

    try:
        import cv2
        import numpy as np
    except ImportError as error:
        report.line("fail", "OpenCV / numpy", str(error), "pip install ster-vis (or pip install .)")
        return 1
    report.line("ok", "OpenCV", cv2.__version__)
    report.line("ok", "numpy", np.__version__)
    if hasattr(cv2, "ximgproc"):
        report.line("ok", "OpenCV contrib (WLS filter)", "available")
    else:
        report.line("warn", "OpenCV contrib (WLS filter)", "missing, --wls will not work",
                    "pip uninstall opencv-python && pip install opencv-contrib-python")

    model = ""
    try:
        model = Path("/proc/device-tree/model").read_text(errors="ignore").strip("\x00\n ")
    except OSError:
        pass
    on_pi = model.startswith("Raspberry Pi")
    if on_pi:
        report.line("ok" if "Raspberry Pi 5" in model else "warn", "board", model,
                    "" if "Raspberry Pi 5" in model else "presets are tuned for a Pi 5")
        try:
            import picamera2  # noqa: F401

            report.line("ok", "picamera2", "importable")
        except ImportError:
            report.line("fail", "picamera2", "not importable",
                        "sudo apt install python3-picamera2, and create the venv with --system-site-packages")
        except Exception as error:  # a numpy mismatch shows up here, not as ImportError
            report.line("fail", "picamera2", f"import failed: {error}",
                        "on Bookworm: pip install -c constraints-numpy1.txt --force-reinstall ster-vis (numpy 1.x)")

        from stereo_vision.benchmark import raspberry_pi_health

        health = raspberry_pi_health()
        if health is None:
            report.line("warn", "temperature / throttling", "vcgencmd unavailable")
        else:
            bad = "THROTTLED" in health or "UNDER-VOLTAGE" in health
            report.line("fail" if bad else "ok", "temperature / throttling", health,
                        "fit the Active Cooler and use the 27 W supply" if bad else "")

    from stereo_vision.sources import picamera2_available

    if on_pi:
        if picamera2_available():
            report.line("ok", "Pi cameras", "two found")
        else:
            report.line("warn", "Pi cameras", "fewer than two found (USB cameras still work)",
                        "check both ribbon cables, then run: rpicam-hello --list-cameras")

    if args.calibration.is_file():
        from stereo_vision.calibration import COVERAGE_WARN_PCT, load_calibration

        try:
            calibration = load_calibration(args.calibration)
        except Exception as error:
            report.line("fail", "calibration", f"{args.calibration} unreadable: {error}",
                        "ster-vis calibrate")
        else:
            w, h = calibration.image_size
            detail = f"{args.calibration}, {w}x{h}, baseline {calibration.baseline:.1f}, RMS {calibration.rms:.2f} px"
            coverage = calibration.coverage_pct
            if coverage == coverage and coverage < COVERAGE_WARN_PCT:  # NaN-safe
                report.line("warn", "calibration", f"{detail}, coverage only {coverage:.0f}%",
                            "recapture with the board in the corners (ster-vis capture --auto)")
            elif calibration.rms > 1.0:
                report.line("warn", "calibration", f"{detail} (RMS above 1 px)", "recapture sharper board views")
            else:
                report.line("ok", "calibration", detail)
    else:
        report.line("warn", "calibration", f"none at {args.calibration}",
                    "ster-vis capture, then ster-vis calibrate")

    if args.cameras:
        from stereo_vision.sources import open_source

        try:
            with open_source(args.backend, threaded=False) as source:
                frame = source.read()
            skew = f", skew {frame.skew_ms:+.1f} ms" if frame.skew_ms is not None else ""
            synced = getattr(getattr(source, "source", source), "synced", None)
            sync = {True: ", hardware-level sync on", False: ", timestamp pairing (no sync support)"}.get(synced, "")
            report.line("ok", "cameras", f"{source.size[0]}x{source.size[1]} pair read{skew}{sync}")
        except Exception as error:
            report.line("fail", "cameras", str(error).splitlines()[0])

    print("\n" + ("some checks failed" if report.failed else "all essential checks passed"))
    return 1 if report.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
