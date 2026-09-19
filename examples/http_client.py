"""Read Ster-Vis depth from a robot program, with nothing but the standard library.

Start the camera side first, on the Pi or any machine with the cameras:

    ster-vis depth --live --headless --stream 8080

then on the robot (or anywhere on the network):

    python http_client.py http://<pi-name>.local:8080            follow every frame
    python http_client.py http://<pi-name>.local:8080 --stop-at 0.5

The event stream pushes each frame's summary as soon as it is ready, so the
robot reacts within one frame rather than on its own polling schedule.
Anything that speaks HTTP can do the same: C++ (libcurl), JavaScript
(EventSource), Rust (reqwest), or a microcontroller with Wi-Fi.
"""

from __future__ import annotations

import argparse
import json
import urllib.request


def events(base_url: str, hz: float = 0):
    """Yield each frame summary from /api/v1/events as a dict."""
    url = f"{base_url.rstrip('/')}/api/v1/events" + (f"?hz={hz}" if hz else "")
    with urllib.request.urlopen(url, timeout=30) as stream:
        for raw in stream:
            line = raw.decode().strip()
            if line.startswith("data:"):
                yield json.loads(line[5:])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("url", help="the camera's address, e.g. http://raspberrypi.local:8080")
    parser.add_argument("--stop-at", type=float, default=0.5, help="metres: warn when anything is closer")
    parser.add_argument("--frames", type=int, default=0, help="stop after this many frames (0 = forever)")
    args = parser.parse_args()

    with urllib.request.urlopen(f"{args.url.rstrip('/')}/api/v1/info", timeout=10) as r:
        info = json.load(r)
    camera = info["camera"]
    print(f"camera {camera['width']}x{camera['height']}, field of view {camera['horizontal_fov_deg']:.0f} deg, "
          f"measures from {info['closest_m']} m")

    for count, frame in enumerate(events(args.url), start=1):
        o = frame["obstacles"]
        sectors = "  ".join(f"{k} {'clear' if v is None else f'{v:.2f} m'}" for k, v in o["sectors_m"].items())
        warning = ""
        if o["nearest_m"] is not None and o["nearest_m"] < args.stop_at:
            side = "left" if o["nearest_bearing_deg"] > 0 else "right"
            warning = f"  STOP: {o['nearest_m']:.2f} m, {abs(o['nearest_bearing_deg']):.0f} deg {side}"
        print(f"#{frame['sequence']:<6} {sectors}{warning}")
        if args.frames and count >= args.frames:
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
