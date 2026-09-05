# RTAB-Map Stereo-Inertial 2D Occupancy Grid for Indoor Drone (ROS2 Jazzy + Intel RealSense D435i)

This package (`rtabmap_drone_pkg`) provides a high-definition visual-inertial SLAM and 2D occupancy grid mapping pipeline tuned specifically for indoor autonomous drone navigation using an Intel RealSense D435i camera and RTAB-Map on ROS2 Jazzy.

---

## Key Features & Advanced Algorithms

1. **High-Definition 2.5cm Occupancy Grid (`rtabmap_slam`)**:
   - **Cell Size**: 2.5 cm resolution (`0.025`m) with 3D surface normal segmentation (`Grid/NormalsSegmentation: true`).
   - **Obstacle Height Band**: `0.30`m to `2.0`m (filters out carpet edges, baseboards, and low furniture underneath drone flight path).
   - **Depth Decimation**: Factor `2` spatial downsampling to retain sharp room corners.

2. **Stateful Hysteresis Wall Locking ($P \ge 80\%$) & Glass Protection**:
   - Locks structural wall boundaries, frames, and mullions into state memory once occupancy confidence reaches $\ge 80\%$.
   - Prevents ray-tracing rays passing through glass or specular reflections from erasing or flickering valid wall lines.
   - Requires at least 15 consecutive clear observations before releasing a locked wall.

3. **Phantom Noise Purging (<1% Noise, 2D LiDAR Quality)**:
   - Uses connected component analysis (`cv2.connectedComponentsWithStats`) in `map_thinning_node.py` to identify and purge isolated black noise blobs smaller than 20 pixels.
   - Applies morphological gap closing and Zhang-Suen skeletonization to publish a crisp, single-pixel wide wall outline on `/map_thin`.

4. **Freeze-Free Auto-Restart & USB Driver Recovery**:
   - Configured with `respawn=True` and `respawn_delay=5.0` to allow the Linux V4L2 USB subsystem to release hardware descriptors cleanly during reconnects, preventing SBC kernel lockups.

---

## Quick Start Guide (For a Fresh Machine / PC)

### Step 1: Install Dependencies & Prerequisites

1. Ensure **ROS2 Jazzy** is installed on Ubuntu 24.04.
2. Plug the **Intel RealSense D435i** into a **USB 3.0/3.2 SuperSpeed** port.
3. Install required ROS2 dependencies:
   ```bash
   sudo apt update
   sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-rtabmap-ros python3-opencv python3-numpy
   ```

---

### Step 2: Clone & Build in ROS2 Workspace

1. Create a ROS2 workspace and clone this repository:
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

Launch the entire pipeline (Camera → Stereo Odometry → RTAB-Map SLAM → Map Thinning Node → Top-Down RViz2 Viewport) in a single command in radxa:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py
```

---

## Published ROS2 Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/map` | `nav_msgs/OccupancyGrid` | High-definition 2.5cm raw RTAB-Map occupancy grid |
| `/map_thin` | `nav_msgs/OccupancyGrid` | **Single-pixel LiDAR-quality wall outline map** (post-processed with wall lock & noise purger) |
| `/odom` | `nav_msgs/Odometry` | Real-time stereo-inertial odometry pose estimate |
| `/cloud_map` | `sensor_msgs/PointCloud2` | Dense 3D point cloud map (disabled by default in RViz for performance) |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | Left infra stereo camera stream |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | Right infra stereo camera stream |
| `/camera/imu` | `sensor_msgs/Imu` | 200Hz fused IMU accelerometer/gyroscope stream |

---

## ⚙️ Customizing Launch Parameters

You can adjust height limits and cell resolution directly via command line arguments:

```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py \
  min_obstacle_height:=0.30 \
  max_obstacle_height:=2.0 \
  cell_size:=0.025 \
  launch_rviz:=true
```

---

## Physical Mapping & Walk-Test Instructions

1. **Boot Calibration**: Keep the camera stationary for **2–3 seconds** immediately after launch for IMU bias initialization.
2. **Movement Speed**: Move smoothly (under 0.5 m/s). Avoid abrupt high-speed yaw rotations.
3. **Sensor Range**: Maintain 0.3 m to 3.5 m distance from walls and obstacles under normal indoor lighting.
