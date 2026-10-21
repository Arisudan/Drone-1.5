#!/usr/bin/env bash
# ==============================================================================
# Drone-GCS Industrial Launch Script
# Configures ROS 2 Jazzy environment, system Qt5 plugins, and executes Drone-GCS
# ==============================================================================

# 1. Source ROS 2 Jazzy if present
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# 2. Qt platform defaults. QT_QPA_PLATFORM_PLUGIN_PATH is deliberately NOT
#    forced here any more - unconditionally pinning it to the system distro
#    path breaks a venv-installed PyQt5, which bundles its own Qt5 (5.15.19
#    it ships with != whatever the OS provides, and "Could not load the Qt
#    platform plugin" is the result). qt_env.py already makes this call
#    correctly, per-install, at import time - only pinning the system path
#    when the PyQt5 about to be imported actually IS the system one.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

# 3. Ensure DISPLAY is configured
export DISPLAY="${DISPLAY:-:0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

exec python3 drone_gcs.py "$@"
