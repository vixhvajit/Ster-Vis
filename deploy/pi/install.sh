#!/usr/bin/env bash
# Install Ster-Vis on a Raspberry Pi as a depth-camera service.
#
#   sudo deploy/pi/install.sh            install, leave the service off
#   sudo deploy/pi/install.sh --enable   install and start it now and at boot
#
# Run it from a checkout of the repository. It installs into /opt/ster-vis,
# reads its settings from /etc/ster-vis, and runs as a dedicated "ster-vis"
# user that belongs to the video group, which is what camera access needs.
# The service waits for a calibration at /etc/ster-vis/stereo.npz and does not
# start without one.
#
# Safe to run again: it upgrades the package and keeps your settings.
set -euo pipefail

PREFIX=/opt/ster-vis
CONFIG=/etc/ster-vis
DATA=/var/lib/ster-vis
UNIT=/etc/systemd/system/ster-vis.service
SERVICE_USER=ster-vis
ENABLE=0

for arg in "$@"; do
    case "$arg" in
        --enable) ENABLE=1 ;;
        -h|--help) sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo: sudo $0 $*" >&2
    exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
if [ ! -f "$REPO/pyproject.toml" ]; then
    echo "run this from a checkout of the Ster-Vis repository" >&2
    exit 1
fi

# shellcheck source=/dev/null
. /etc/os-release
CODENAME="${VERSION_CODENAME:-unknown}"
echo "==> Installing Ster-Vis on ${PRETTY_NAME:-this system}"

# System packages. STER_VIS_SKIP_APT=1 skips this step, for testing the
# installer on machines without Raspberry Pi packages.
if [ "${STER_VIS_SKIP_APT:-0}" != "1" ]; then
    echo "==> Installing system packages"
    apt-get update -qq
    apt-get install -y -qq --no-install-recommends python3-venv python3-pip python3-picamera2
fi

if ! id "$SERVICE_USER" >/dev/null 2>&1; then
    echo "==> Creating the $SERVICE_USER user"
    useradd --system --home-dir "$DATA" --shell /usr/sbin/nologin "$SERVICE_USER"
fi
getent group video >/dev/null || groupadd --system video
usermod -a -G video "$SERVICE_USER"
install -d -o "$SERVICE_USER" -g "$SERVICE_USER" -m 0755 "$DATA"
install -d -m 0755 "$CONFIG"

echo "==> Creating the Python environment in $PREFIX"
install -d -m 0755 "$PREFIX"
# --system-site-packages makes the apt-installed picamera2 visible inside it.
python3 -m venv --system-site-packages "$PREFIX/venv"
"$PREFIX/venv/bin/python" -m pip install --quiet --upgrade pip

echo "==> Installing the ster-vis package"
if [ "$CODENAME" = "bookworm" ]; then
    # Bookworm's picamera2 is built against numpy 1.x; see the constraints file.
    "$PREFIX/venv/bin/python" -m pip install --quiet --upgrade \
        -c "$REPO/constraints-numpy1.txt" "$REPO"
else
    "$PREFIX/venv/bin/python" -m pip install --quiet --upgrade "$REPO"
fi
ln -sf "$PREFIX/venv/bin/ster-vis" /usr/local/bin/ster-vis

if [ ! -f "$CONFIG/ster-vis.env" ]; then
    install -m 0644 "$HERE/ster-vis.env" "$CONFIG/ster-vis.env"
    echo "==> Wrote default settings to $CONFIG/ster-vis.env"
else
    echo "==> Kept existing settings in $CONFIG/ster-vis.env"
fi

install -m 0644 "$HERE/ster-vis.service" "$UNIT"
systemctl daemon-reload

if [ "$ENABLE" = "1" ]; then
    systemctl enable --now ster-vis.service
    echo "==> Service enabled"
fi

cat <<EOF

Installed $("$PREFIX/venv/bin/ster-vis" --version).

Next:
  1. Check the setup:       ster-vis doctor --cameras
  2. Calibrate (as yourself, in any folder):
       ster-vis capture --auto --headless --stream 8080
       ster-vis calibrate --square-size 25
  3. Hand the calibration to the service:
       sudo cp calib/stereo.npz $CONFIG/stereo.npz
  4. Start it:              sudo systemctl enable --now ster-vis
     Watch it:              journalctl -u ster-vis -f
     Settings:              $CONFIG/ster-vis.env (then: sudo systemctl restart ster-vis)
EOF
