#!/usr/bin/env bash
# ==============================================================================
# Master Drone SLAM Pipeline Launcher (Radxa SBC)
# Sources ROS 2 Jazzy and the rtabmap_drone_pkg overlay workspace
# ==============================================================================
set -e

# 1. Source ROS 2 Jazzy underlay
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: /opt/ros/jazzy/setup.bash not found!" >&2
    exit 1
fi

# 2. Source ros2_ws overlay (contains rtabmap_drone_pkg)
if [ -f "/home/radxa/ros2_ws/install/setup.bash" ]; then
    source /home/radxa/ros2_ws/install/setup.bash
elif [ -f "/home/radxa/Downloads/Drone-1.5-main/RTAB Map SLAM/install/setup.bash" ]; then
    source "/home/radxa/Downloads/Drone-1.5-main/RTAB Map SLAM/install/setup.bash"
else
    echo "ERROR: Workspace install/setup.bash not found!" >&2
    exit 1
fi

echo "✔ Sourced ROS 2 Jazzy & rtabmap_drone_pkg"
echo "🚀 Launching complete SLAM pipeline (Camera + RTAB-Map + Thinning + TCP Bridge)..."

exec ros2 launch /home/radxa/Flop/launch/drone_rtabmap_all.launch.py "$@"
