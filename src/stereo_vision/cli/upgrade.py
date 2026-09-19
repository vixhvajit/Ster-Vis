"""Check for, verify and install another Ster-Vis release from GitHub.

  ster-vis upgrade --check        is there a newer release?
  ster-vis upgrade                install the latest release
  ster-vis upgrade --to 2.0.0     install exactly that version (also to go back)

The wheel is downloaded first and checked against the release's SHA256SUMS
before pip sees it; a mismatch stops the upgrade. When the installed numpy is
1.x (Raspberry Pi OS Bookworm, ROS 2 Jazzy), the pins from
constraints-numpy1.txt are applied so the upgrade cannot pull in numpy 2 and
break picamera2 or rclpy.

On a Pi installed with deploy/pi/install.sh, run it with sudo, then restart
the service: sudo systemctl restart ster-vis
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
from pathlib import Path

from stereo_vision import __version__

REPOSITORY = "vixhvajit/Ster-Vis"
API = f"https://api.github.com/repos/{REPOSITORY}/releases"
# The same pins as constraints-numpy1.txt, which is not inside the wheel.
NUMPY1_PINS = "opencv-contrib-python==4.11.0.86\nnumpy>=1.23.5,<2\n"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ster-vis upgrade", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="only report whether a newer release exists")
    parser.add_argument("--to", metavar="VERSION", help="install this version, e.g. 2.0.0 (older ones too)")
    parser.add_argument("--force", action="store_true", help="reinstall even if already on that version")
    return parser.parse_args(argv)


def version_key(version: str) -> tuple:
    """Sortable key for X.Y.Z, with pre-releases (X.Y.Z-rc.1) before the release."""
    match = re.fullmatch(r"v?(\d+)\.(\d+)\.(\d+)(?:[-.]?(.+))?", version.strip())
    if not match:
        raise ValueError(f"not a version: {version!r}")
    major, minor, patch, pre = match.groups()
    return (int(major), int(minor), int(patch), 0 if pre else 1, pre or "")


def fetch_json(url: str) -> dict:
    request = urllib.request.Request(url, headers={
        "Accept": "application/vnd.github+json", "User-Agent": f"ster-vis/{__version__}"})
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


def download(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": f"ster-vis/{__version__}"})
    with urllib.request.urlopen(request, timeout=120) as response:
        return response.read()


def find_release(version: str | None) -> dict:
    if version is None:
        return fetch_json(f"{API}/latest")
    tag = version if version.startswith("v") else f"v{version}"
    return fetch_json(f"{API}/tags/{tag}")


def expected_hashes(sums: str) -> dict[str, str]:
    """Parse a SHA256SUMS file: '<hash>  <name>' per line."""
    hashes = {}
    for line in sums.splitlines():
        parts = line.strip().split()
        if len(parts) == 2 and re.fullmatch(r"[0-9a-f]{64}", parts[0]):
            hashes[parts[1].lstrip("*")] = parts[0]
    return hashes


def needs_numpy1() -> bool:
    try:
        import numpy
    except ImportError:
        return False
    return version_key(numpy.__version__)[0] < 2


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        release = find_release(args.to)
    except urllib.error.HTTPError as error:
        if error.code == 404:
            print(f"no release {args.to or 'published yet'} at github.com/{REPOSITORY}/releases")
        else:
            print(f"GitHub answered {error.code}: {error.reason}")
        return 1
    except (urllib.error.URLError, TimeoutError) as error:
        print(f"could not reach GitHub: {getattr(error, 'reason', error)}")
        return 1

    target = release["tag_name"].lstrip("v")
    current = __version__
    newer = version_key(target) > version_key(current)
    same = version_key(target) == version_key(current)

    if args.check:
        if newer:
            print(f"ster-vis {target} is available (installed: {current})")
            print(f"  what's new: {release.get('html_url', '')}")
            print("  install it with: ster-vis upgrade")
        elif same:
            print(f"ster-vis {current} is the latest release")
        else:
            print(f"ster-vis {current} is newer than the latest release ({target}): a development build")
        return 0

    if args.to is None and not newer:
        print(f"already up to date (ster-vis {current}; latest release {target})")
        return 0
    if same and not args.force:
        print(f"ster-vis {current} is already installed; add --force to reinstall it")
        return 0

    assets = {asset["name"]: asset["browser_download_url"] for asset in release.get("assets", [])}
    wheels = [name for name in assets if name.endswith(".whl")]
    if not wheels or "SHA256SUMS" not in assets:
        print(f"release {target} has no wheel and SHA256SUMS to install from")
        return 1
    wheel_name = wheels[0]

    print(f"downloading {wheel_name} ...")
    wheel = download(assets[wheel_name])
    expected = expected_hashes(download(assets["SHA256SUMS"]).decode()).get(wheel_name)
    actual = hashlib.sha256(wheel).hexdigest()
    if expected is None:
        print(f"SHA256SUMS does not list {wheel_name}; not installing")
        return 1
    if actual != expected:
        print(f"checksum mismatch for {wheel_name}: expected {expected}, got {actual}; not installing")
        return 1
    print(f"checksum verified ({actual[:16]}...)")

    with tempfile.TemporaryDirectory() as folder:
        wheel_path = Path(folder) / wheel_name
        wheel_path.write_bytes(wheel)
        command = [sys.executable, "-m", "pip", "install", str(wheel_path)]
        if same:
            # pip skips a version that is already installed; given a wheel of a
            # different version it replaces the installed one by itself.
            command[4:4] = ["--force-reinstall", "--no-deps"]
        if needs_numpy1():
            pins = Path(folder) / "constraints-numpy1.txt"
            pins.write_text(NUMPY1_PINS, encoding="utf-8")
            command += ["-c", str(pins)]
            print("numpy 1.x installed: keeping it, with the OpenCV release that supports it")
        print(("going back" if version_key(target) < version_key(current) else "upgrading")
              + f" from {current} to {target} ...")
        result = subprocess.run(command)
    if result.returncode != 0:
        print("pip failed; the previous version is still installed")
        return result.returncode
    print(f"installed ster-vis {target}")
    print("  on a Pi running the service: sudo systemctl restart ster-vis")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
