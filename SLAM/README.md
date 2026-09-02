# RTAB-Map Stereo-Inertial 2D Occupancy Grid for Indoor Drone (ROS2 Jazzy + Intel RealSense D435i)

This package provides a complete visual-inertial SLAM and 2D occupancy grid mapping pipeline tuned specifically for indoor autonomous drone navigation using an Intel RealSense D435i camera and RTAB-Map on ROS2 Jazzy.

---

## Features

- **Stereo-Inertial Odometry**: Hardware-synchronized infrared stereo (IR1 & IR2) + IMU fusion for robust 6-DOF tracking.
- **3D Point Cloud & 2D Occupancy Grid**: Dense OctoMap-based 2D occupancy grid publishing on `/map`.
- **Drone Height Band Filtering**: Configurable obstacle height limits (`min_obstacle_height` and `max_obstacle_height`) to filter out ground/ceiling planes relative to the flight level.
- **Ray Tracing**: Clears free space along optical rays to maintain accurate obstacle and open space representation.
- **RViz2 Integration**: Includes a pre-configured top-down orthographic tracking viewport (`rtabmap_drone.rviz`).

---

## Prerequisites

- **OS**: Ubuntu 24.04 LTS (Noble)
- **ROS Version**: ROS2 Jazzy
- **Hardware**: Intel RealSense D435i connected via **USB 3.2 SuperSpeed**

---

## Installation

Install required ROS2 Jazzy packages via `apt`:

```bash
sudo apt update
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-rtabmap-ros
```

If python helper nodes or vision scripts are used, install dependencies:

```bash
pip install opencv-python numpy
```

---

## Building the Workspace

1. Clone or copy this repository into your ROS2 workspace source folder:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Build `rtabmap_drone_pkg`:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select rtabmap_drone_pkg --symlink-install
   source ~/ros2_ws/install/setup.bash
   ```

---

## Running the Pipeline

Bring up the complete pipeline (Camera → Stereo Odometry → RTAB-Map SLAM → RViz2):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py
```

### Overridable Launch Arguments:

You can pass custom parameters to tune height bands and grid resolution for your room:

```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py \
  min_obstacle_height:=0.1 \
  max_obstacle_height:=2.0 \
  cell_size:=0.05 \
  launch_rviz:=true
```

| Argument | Default | Description |
| :--- | :--- | :--- |
| `min_obstacle_height` | `0.1` | Minimum obstacle height threshold (meters) relative to drone link |
| `max_obstacle_height` | `2.0` | Maximum obstacle height threshold (meters) relative to drone link |
| `cell_size` | `0.05` | Resolution of the occupancy grid cells (meters) |
| `launch_rviz` | `true` | Whether to launch RViz2 automatically |

---

## Physical Walk-Test Guidance

1. **Boot Initialization**: Keep the camera completely stationary for **2–3 seconds** immediately after launching to initialize IMU bias calibration.
2. **Translation Speed**: Move the camera smoothly at speeds below **0.5 m/s**.
3. **Rotations**: Avoid rapid yaw or pitch rotation jerks to prevent feature tracking loss.
4. **Distance & Lighting**: Keep obstacles and surfaces within **0.3 m – 3.5 m** range with adequate ambient room lighting.

---

## Published ROS2 Topics

- `/map` (`nav_msgs/OccupancyGrid`): Flattened 2D occupancy grid map
- `/cloud_map` (`sensor_msgs/PointCloud2`): Accumulated 3D dense map
- `/odom` (`nav_msgs/Odometry`): Stereo-inertial odometry pose estimate
- `/camera/infra1/image_rect_raw` & `/camera/infra2/image_rect_raw`: Infrared stereo image streams
- `/camera/imu`: Hardware-united accel + gyro stream
