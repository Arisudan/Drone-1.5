#!/usr/bin/env bash
# ==============================================================================
# One-command drone pipeline launcher (Radxa SBC, run this over SSH)
# Sources ROS 2 Jazzy + the rtabmap_drone_pkg workspace, then launches the full
# pipeline (camera, stereo odometry, RTAB-Map SLAM, video streamer, PX4 vision
# bridge, map thinning, wall boundary, TCP map streamer).
#
# Defaults to launch_rviz:=false since the Radxa is mounted on the drone with
# no display attached - 3D visualization happens on the GCS laptop instead
# (its own embedded RViz view). Override if needed:
#   ./camera.sh launch_rviz:=true
# ==============================================================================
set -e

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: /opt/ros/jazzy/setup.bash not found!" >&2
    exit 1
fi

if [ -f "/home/radxa/ros2_ws/install/setup.bash" ]; then
    source /home/radxa/ros2_ws/install/setup.bash
else
    echo "ERROR: Workspace install/setup.bash not found!" >&2
    exit 1
fi

echo "Sourced ROS 2 Jazzy & rtabmap_drone_pkg"
echo "Launching drone pipeline (headless, launch_rviz:=false unless overridden)..."

exec ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py launch_rviz:=false "$@"
