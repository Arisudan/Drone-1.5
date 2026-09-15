#!/usr/bin/env bash
# ==============================================================================
# Drone-GCS Industrial Launch Script
# Configures ROS 2 Jazzy environment, system Qt5 plugins, and executes Drone-GCS
# ==============================================================================

# 1. Source ROS 2 Jazzy if present
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# 2. Force system Qt5 XCB backend (prevents cv2/qt conflict)
export QT_QPA_PLATFORM=xcb
if [ -d "/usr/lib/aarch64-linux-gnu/qt5/plugins" ]; then
    export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/qt5/plugins
elif [ -d "/usr/lib/x86_64-linux-gnu/qt5/plugins" ]; then
    export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
fi

# 3. Ensure DISPLAY is configured
export DISPLAY="${DISPLAY:-:0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

exec python3 drone_gcs.py "$@"
