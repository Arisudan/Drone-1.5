# RTAB-Map Stereo-Inertial 2D Occupancy Grid for Indoor Drone (ROS 2 Jazzy + Intel RealSense D435i)

`rtabmap_drone_pkg` is a ROS 2 Jazzy package providing a visual-inertial SLAM and high-definition 2D occupancy grid mapping pipeline tuned specifically for indoor autonomous drone navigation using an Intel RealSense D435i depth/stereo camera and RTAB-Map SLAM.

---

## 📂 Repository Structure

```
Netrein/
├── CMakeLists.txt              # Build configuration (installs scripts, launch, config)
├── package.xml                 # Package manifest (ament_cmake)
├── README.md                   # Package documentation
├── config/
│   └── rtabmap_drone.rviz      # Custom top-down RViz2 viewport configuration
├── launch/
│   ├── drone_rtabmap_all.launch.py    # Master staged launcher (Camera -> Odom -> SLAM -> RViz2)
│   ├── d435i_stereo_imu.launch.py     # RealSense D435i camera node setup
│   ├── stereo_inertial_odom.launch.py # RTAB-Map stereo-inertial odometry setup
│   └── rtabmap_slam.launch.py          # RTAB-Map SLAM node & map thinning node
└── scripts/
    ├── map_thinning_node.py     # Stateful hysteresis wall lock & skeletonization node
    └── wall_boundary_node.py    # Safe flight boundary extraction node (MarkerArray)
```

---

## 🌟 Key Features & Algorithms

### 1. High-Definition 2.5cm Occupancy Grid (`rtabmap_slam`)
- **Cell Resolution**: Ultra-fine 2.5 cm grid (`0.025` m) with 3D surface normal segmentation (`Grid/NormalsSegmentation: true`) to isolate vertical structures from floor noise.
- **Drone Altitude Band Filtering**: Obstacle height window set between `0.30` m and `2.0` m to filter out floor baseboards, carpets, and objects underneath the drone's flight path.
- **Depth Decimation**: Spatial downsampling factor of `2` to preserve sharp room corners while maintaining high performance.

### 2. Staged Launch Sequence with Freeze Prevention
- **Staged Initialization**: Prevents node initialization race conditions during startup:
  - **$t = 0\text{s}$**: Camera node initializes.
  - **$t = 3\text{s}$**: Stereo odometry starts after infra/IMU streams stabilize.
  - **$t = 6\text{s}$**: RTAB-Map SLAM node & map thinning node launch.
  - **$t = 8\text{s}$**: RViz2 opens with pre-configured viewport.
- **Auto-Respawn Protection**: Camera node configured with `respawn=True` and a `5.0`s delay to release V4L2 USB descriptors cleanly upon reconnects, preventing SBC/kernel lockups.

### 3. Stateful Hysteresis Wall Locking & Phantom Noise Purging (`map_thinning_node.py`)
- **Hysteresis Wall Lock ($P \ge 80\%$)**: Locks wall structures into state memory once occupancy confidence reaches $\ge 80\%$, preventing ray-tracing rays passing through glass windows or specular reflections from flickering or erasing structural walls.
- **Clear Observation Countdown**: Locked cells require 15 consecutive clear observations before being unlocked.
- **Phantom Noise Purging**: Uses connected component analysis (`cv2.connectedComponentsWithStats`) to strip out scattered noise clusters smaller than 20 pixels.
- **1-Pixel Skeletonization**: Applies OpenCV morphological closing and skeletonization (Zhang-Suen / cross-element fallback) to publish a crisp single-pixel wall line on `/map_thin`.

### 4. Safe Flight Wall Boundary Extraction (`wall_boundary_node.py`)
- **Obstacle Inflation**: Erodes free space by a safety inflation radius (`0.6` m default) to ensure safe clearance for drone operations.
- **Polygon Smoothing**: Applies OpenCV `approxPolyDP` polygon approximation to extract smooth room perimeter contours.
- **RViz Visualization**: Publishes continuous red boundary line strips to `/wall_boundaries` (`visualization_msgs/msg/MarkerArray`).

---

## 📋 Quick Start Guide

### Prerequisites & Dependencies

1. **ROS 2 Jazzy** installed on Ubuntu 24.04.
2. **Intel RealSense D435i** connected via **USB 3.0/3.2 SuperSpeed**.
3. Install required ROS 2 dependencies:
   ```bash
   sudo apt update
   sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-rtabmap-ros python3-opencv python3-numpy
   ```

### Building the Package

1. Navigate to your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws
   ```
2. Build `rtabmap_drone_pkg` with symlink install:
   ```bash
   colcon build --packages-select rtabmap_drone_pkg --symlink-install
   ```

---

## 🚀 Running the Pipeline

Source your ROS 2 environment and launch the master launch file:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py
```

### Launch Arguments & Configuration

You can customize execution parameters directly from the command line:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `min_obstacle_height` | `0.1` | Minimum obstacle height for 2D grid map (meters) |
| `max_obstacle_height` | `2.0` | Maximum obstacle height for 2D grid map (meters) |
| `cell_size` | `0.05` | Occupancy grid cell resolution (meters) |
| `launch_rviz` | `true` | Set to `false` to run headless without launching RViz2 |

**Example with custom parameters:**
```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py \
  min_obstacle_height:=0.30 \
  max_obstacle_height:=2.0 \
  cell_size:=0.025 \
  launch_rviz:=true
```

---

## 📡 Published ROS 2 Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Raw 2.5cm RTAB-Map occupancy grid map |
| `/map_thin` | `nav_msgs/msg/OccupancyGrid` | **Single-pixel wall outline map** (processed via wall-locking & noise purger) |
| `/wall_boundaries` | `visualization_msgs/msg/MarkerArray` | **Safe drone flight boundary visualization** (0.6m obstacle inflation line) |
| `/odom` | `nav_msgs/msg/Odometry` | Real-time stereo-inertial odometry pose estimate |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/msg/Image` | Left infrared camera stream (rectified) |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/msg/Image` | Right infrared camera stream (rectified) |
| `/camera/imu` | `sensor_msgs/msg/Imu` | 200 Hz fused accelerometer/gyroscope stream |
| `/cloud_map` | `sensor_msgs/msg/PointCloud2` | 3D point cloud map generated by RTAB-Map |

---

## 🚶 Operational Guidelines & Best Practices

1. **Boot Calibration**: Keep the D435i camera completely stationary for **2–3 seconds** immediately after launch to allow IMU bias initialization.
2. **Smooth Movement**: Move the camera/drone smoothly (under 0.5 m/s). Avoid rapid yaw rotations to prevent feature tracking loss in visual odometry.
3. **Lighting & Range**: Ensure adequate indoor lighting. Maintain a range of 0.3 m to 3.5 m from surrounding walls.
4. **Database Retention**: RTAB-Map automatically saves session database maps to `~/.ros/rtabmap.db` on clean shutdown (SIGINT / Ctrl+C).
