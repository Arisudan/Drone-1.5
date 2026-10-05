#!/usr/bin/env bash
# ==============================================================================
# Install (or remove) the Radxa's boot services. Run ON the Radxa:
#
#   sudo scripts/radxa/install_radxa_services.sh [--user radxa] [--start]
#   sudo scripts/radxa/install_radxa_services.sh --uninstall
#
#   drone-pipeline   the camera / SLAM / video / map pipeline, started at boot
#   drone-watchdog   restarts it when stuck; serves status on :8081
#   drone-janitor    daily log cleanup (timer)
#
# Idempotent: running it again just refreshes the unit files. The pipeline is
# ENABLED (starts at next boot) but only STARTED now with --start.
# ==============================================================================
set -euo pipefail

USER_NAME="radxa"
START=0
UNINSTALL=0
while [ $# -gt 0 ]; do
  case "$1" in
    --user) USER_NAME="$2"; shift 2 ;;
    --start) START=1; shift ;;
    --uninstall) UNINSTALL=1; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }

HOME_DIR="$(getent passwd "$USER_NAME" | cut -d: -f6)"
[ -n "$HOME_DIR" ] || { echo "no such user: $USER_NAME" >&2; exit 1; }
SRC="$HOME_DIR/Flop/scripts/radxa/deploy"
UNITS="drone-pipeline.service drone-watchdog.service drone-janitor.service drone-janitor.timer"

if [ "$UNINSTALL" -eq 1 ]; then
  systemctl disable --now drone-janitor.timer drone-watchdog.service drone-pipeline.service 2>/dev/null || true
  for u in $UNITS; do rm -f "/etc/systemd/system/$u"; done
  systemctl daemon-reload
  echo "removed."
  exit 0
fi

[ -x "$HOME_DIR/Flop/camera.sh" ] || { echo "$HOME_DIR/Flop/camera.sh is missing or not executable" >&2; exit 1; }
for u in $UNITS; do
  sed -e "s#@USER@#$USER_NAME#g" -e "s#@HOME@#$HOME_DIR#g" "$SRC/$u" > "/etc/systemd/system/$u"
  chmod 644 "/etc/systemd/system/$u"
done
systemctl daemon-reload
systemctl enable drone-pipeline.service drone-watchdog.service drone-janitor.timer
systemctl restart drone-watchdog.service
systemctl start drone-janitor.timer
if [ "$START" -eq 1 ]; then systemctl restart drone-pipeline.service; fi

echo "installed. status:"
systemctl is-enabled drone-pipeline.service drone-watchdog.service drone-janitor.timer || true
systemctl is-active drone-watchdog.service drone-janitor.timer || true
echo "pipeline: $(systemctl is-active drone-pipeline.service || true) (use --start, or: sudo systemctl start drone-pipeline)"
