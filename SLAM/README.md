# RTAB-Map Stereo-Inertial 2D Occupancy Grid for Indoor Drone (ROS2 Jazzy + Intel RealSense D435i)

This package (`rtabmap_drone_pkg`) provides a complete visual-inertial SLAM and 2D occupancy grid mapping pipeline tuned specifically for indoor autonomous drone navigation using an Intel RealSense D435i camera and RTAB-Map on ROS2 Jazzy.

---

## 📋 Quick Start Guide (For a Fresh Machine / PC)

Follow these exact steps when cloning this repository onto a new computer:

### Step 1: Install Dependencies & Prerequisites

1. Ensure **ROS2 Jazzy** is installed on Ubuntu 24.04.
2. Plug the **Intel RealSense D435i** into a **USB 3.0/3.2 SuperSpeed** port.
3. Install required ROS2 packages:
   ```bash
   sudo apt update
   sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-rtabmap-ros
   ```

---

### Step 2: Clone & Build in ROS2 Workspace

1. Create a ROS2 workspace (if not already existing) and clone this repository:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/Arisudan/Drone-1.5.git
   ```

2. Build the workspace package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select rtabmap_drone_pkg --symlink-install
   source ~/ros2_ws/install/setup.bash
   ```

---

### Step 3: Run the Complete Occupancy Grid Pipeline

Launch the entire pipeline (Camera → Stereo Odometry → RTAB-Map 2D Occupancy Grid SLAM → Top-Down Tracking RViz2 Viewport) in a single command:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py
```

---

## ⚙️ Customizing Height Limits & Resolution

You can adjust the obstacle height band and cell resolution directly from the launch command:

```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py \
  min_obstacle_height:=0.1 \
  max_obstacle_height:=2.0 \
  cell_size:=0.05 \
  launch_rviz:=true
```

### Parameter Reference:

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `min_obstacle_height` | `0.1` | Minimum obstacle height threshold (meters) relative to drone link |
| `max_obstacle_height` | `2.0` | Maximum obstacle height threshold (meters) relative to drone link |
| `cell_size` | `0.05` | Resolution of grid cells in meters (5 cm cells) |
| `launch_rviz` | `true` | Whether to auto-start RViz2 with top-down tracking view |

---

## 🚶 Physical Mapping & Walk-Test Instructions

1. **Boot Calibration**: Keep the camera completely still for **2–3 seconds** immediately after launch for IMU bias initialization.
2. **Movement Speed**: Move smoothly (under 0.5 m/s). Avoid abrupt yaw/pitch rotations.
3. **Sensor Range**: Maintain 0.3 m to 3.5 m distance from walls and obstacles under normal ambient room lighting.

---

## 📡 Published ROS2 Topics

- `/map` (`nav_msgs/OccupancyGrid`): Live 2D occupancy grid map
- `/cloud_map` (`sensor_msgs/PointCloud2`): Dense 3D point cloud
- `/odom` (`nav_msgs/Odometry`): Visual-inertial odometry pose estimate
- `/camera/infra1/image_rect_raw` & `/camera/infra2/image_rect_raw`: Hardware time-synced IR stereo image streams
- `/camera/imu`: 200Hz fused accel/gyro stream
