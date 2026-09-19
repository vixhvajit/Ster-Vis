#!/usr/bin/env bash
# Remove the Ster-Vis service and its Python environment.
#
#   sudo deploy/pi/uninstall.sh           keep settings, calibration and saved depth maps
#   sudo deploy/pi/uninstall.sh --purge   remove those too, and the ster-vis user
set -euo pipefail

PURGE=0
for arg in "$@"; do
    case "$arg" in
        --purge) PURGE=1 ;;
        -h|--help) sed -n '2,5p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo: sudo $0 $*" >&2
    exit 1
fi

if systemctl list-unit-files ster-vis.service >/dev/null 2>&1; then
    systemctl disable --now ster-vis.service 2>/dev/null || true
fi
rm -f /etc/systemd/system/ster-vis.service
systemctl daemon-reload
rm -f /usr/local/bin/ster-vis
rm -rf /opt/ster-vis
echo "removed the service and /opt/ster-vis"

if [ "$PURGE" = "1" ]; then
    rm -rf /etc/ster-vis /var/lib/ster-vis
    if id ster-vis >/dev/null 2>&1; then
        userdel ster-vis
    fi
    echo "removed /etc/ster-vis, /var/lib/ster-vis and the ster-vis user"
else
    echo "kept /etc/ster-vis and /var/lib/ster-vis (use --purge to remove them)"
fi
