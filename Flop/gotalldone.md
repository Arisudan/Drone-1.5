# RTAB-Map Stereo-Inertial Autonomous Drone Pipeline — Complete Runbook (`gotalldone.md`)

This document consolidates the complete technical analysis, installation steps, troubleshooting history, Q&A log, and exact copy-paste commands to build, launch, monitor, and control this project.

---

## 1. Architecture & Component Overview

This package (`rtabmap_drone_pkg`) provides a high-definition visual-inertial SLAM and 2D occupancy grid mapping pipeline for indoor autonomous drone navigation using an **Intel RealSense D435i** camera and **Pixhawk (PX4)** flight controller on **ROS 2 Jazzy**.

- **Sensors**: Intel RealSense D435i streaming 30 Hz stereo infrared images (`/camera/infra1`, `/camera/infra2`) and 200 Hz fused IMU (`/camera/imu`).
- **Odometry & SLAM**: `stereo_odometry` + `rtabmap_slam` generating real-time `/odom`, a 2.5 cm raw 2D occupancy grid (`/map`), and a 3D point cloud (`/cloud_map`).
- **Post-Processing**: `map_thinning_node.py` (stateful hysteresis wall locking, isolated noise purging, skeletonization) publishing a LiDAR-quality 2D wall outline (`/map_thin`).
- **PX4 Flight Controller Bridge**: `px4_vision_bridge.py` converting ROS 2 `/odom` into MAVLink `VISION_POSITION_ESTIMATE` packets for PX4 EKF2 position estimation (GPS-denied navigation).
- **Obstacle Safety Layer**: `obstacle_distance_bridge.py` streaming 360° `OBSTACLE_DISTANCE` MAVLink messages to PX4 for onboard collision prevention.
- **Flight Control & REPL**: `px4_control.py` CLI tool for telemetry monitoring, arming/disarming, takeoff, and relative position movement commands.

---

## 2. Prerequisites & System Dependencies

Run this command once to install all required system packages on Ubuntu 24.04:

```bash
sudo apt update
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-rtabmap-ros python3-opencv python3-numpy
```

---

## 3. MAVLink Router Setup (Download, Build & Service Setup)

`mavlink-router` acts as a MAVLink proxy/multiplexer, allowing multiple processes (ROS 2 Vision Bridge, QGroundControl over WiFi, and `px4_control.py`) to talk to the Pixhawk concurrently without USB port conflicts.

### Step-by-Step Installation Commands:

```bash
# 1. Install build tools
sudo apt update
sudo apt install -y meson ninja-build pkg-config libsystemd-dev git gcc g++

# 2. Clone and compile mavlink-router
cd ~
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router
git submodule update --init --recursive
meson setup build
ninja -C build
sudo ninja -C build install

# 3. Create configuration file (/etc/mavlink-router/main.conf)
sudo mkdir -p /etc/mavlink-router
sudo tee /etc/mavlink-router/main.conf > /dev/null << 'EOF'
[General]
TcpServerPort=5760

[UartEndpoint pixhawk]
Device=/dev/ttyACM0
Baud=921600

[UdpEndpoint qgc]
Mode=normal
Address=127.0.0.1
Port=14550
EOF

# 4. Enable and start systemd service
sudo systemctl daemon-reload
sudo systemctl enable --now mavlink-router
sudo systemctl status mavlink-router
```

---

## 4. Complete Execution Runbook (Step-by-Step Commands)

### Step 1: Verify Hardware & MAVLink Link
```bash
# Check Pixhawk USB serial port
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

# Check MAVLink Router status
sudo systemctl status mavlink-router
```

### Step 2: Verify & Apply EKF2 Vision Parameters on Pixhawk
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Check EKF2 parameter compliance on Pixhawk
python3 ~/Flop/scripts/diagnostics/verify_ekf2_params.py --port tcp:127.0.0.1:5760
```

### Step 3: Build & Source ROS 2 Workspace
```bash
# Set script permissions
chmod +x ~/Flop/scripts/*.py ~/Flop/scripts/diagnostics/*.py

# Build workspace package
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select rtabmap_drone_pkg --symlink-install

# Source workspace
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Step 4: Launch the Full Autonomous Pipeline

#### Option A: Full System (Camera + SLAM + PX4 Vision Bridge + RViz)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py \
  enable_px4_bridge:=true \
  pixhawk_device:=tcp:127.0.0.1:5760 \
  launch_rviz:=true
```

#### Option B: Standalone / Bench Test (Camera + SLAM Mapping Only, No PX4)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py \
  enable_px4_bridge:=false \
  launch_rviz:=true
```

### Step 5: Visual Tracking Initialization & Walk Test
1. Keep the RealSense D435i camera stationary for **2–3 seconds** immediately after launch for IMU bias calibration.
2. Point the camera at a detailed, textured surface (cluttered desk, keyboard, bookshelf) **0.5m to 1.5m away**.
3. Pan slowly side-to-side (under 0.5 m/s) until terminal logs `Odom: quality > 0`. The 2D map in RViz will turn green (`Status: Ok`) and render live.

### Step 6: Check PX4 Status & Safety (In a 2nd Terminal)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

python3 ~/Flop/scripts/diagnostics/px4_control.py --port tcp:127.0.0.1:5760 status
```

### Step 7: Interactive Flight REPL (In a 2nd Terminal)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

python3 ~/Flop/scripts/diagnostics/px4_control.py --port tcp:127.0.0.1:5760 repl
```
Inside the interactive prompt (`px4>`):
- `status` — Show arming readiness, flight mode, and safety lock metrics.
- `arm` / `disarm` — Arm or disarm motors.
- `takeoff 1.0` — Take off to 1.0m altitude.
- `move 0.5 0 0` — Move 0.5m forward in body frame ($dx, dy, dz$).
- `yaw 90` — Rotate 90 degrees.
- `quit` — Exit prompt.

---

## 5. Complete Q&A & Troubleshooting Log

### Q1: Why did `ls -la /dev/pixhawk` and `systemctl status mavlink-router` fail initially?
- **Answer**: The setup documentation in `README.md` was written for the drone's onboard companion computer (Radxa SBC), where `mavlink-router` was installed as a systemd service and a udev rule created `/dev/pixhawk`. On a fresh desktop PC (`htic-pc`), Pixhawk appears directly as `/dev/ttyACM0` until `mavlink-router` is compiled and installed.

---

### Q2: Why did RViz open but show `! Map Status: Warn` with an empty grid?
- **Answer**: RTAB-Map's `stereo_odometry` requires visual parallax and $\ge 10$ feature inliers (`Vis/MinInliers = 10`) between frames to compute odometry. When the camera is stationary or facing a blank surface, `inliers = 0` and tracking reports `LOST!`. RTAB-Map intentionally waits until valid `/odom` updates are received before publishing `/map`. Moving the camera in front of a textured object 0.5m–1.5m away locks tracking and immediately renders the map.

---

### Q3: Why did `ros2 topic hz /camera/infra1/image_rect_raw` report "topic does not appear to be published yet" right after launch?
- **Answer**: `d435i_stereo_imu.launch.py` has `initial_reset: True` enabled. On launch, the node resets the D435i USB hardware bus, which takes **6 to 8 seconds** for the Linux kernel to re-enumerate. Topics start streaming once `[camera]: RealSense Node Is Up!` appears in the log.

---

### Q4: Why did `ros2 run rtabmap_drone_pkg slam_health_monitor.py` say "No executable found"?
- **Answer**: In ROS 2 / `colcon`, Python scripts must have executable file permissions (`chmod +x`) before running `colcon build`. You can run diagnostic scripts directly using `python3 ~/Flop/scripts/diagnostics/slam_health_monitor.py` or apply `chmod +x` and rebuild the package.

---

### Q5: What do warnings like `Received IMU doesn't have orientation set! It is ignored` mean?
- **Answer**:
  - `Received IMU doesn't have orientation set`: **Harmless / Expected**. RealSense D435i outputs raw 6-DOF gyro/accel data without an orientation quaternion filter. RTAB-Map ignores the orientation field and integrates raw gyro/accel data automatically.
  - `Stereo correspondences rejected`: Camera is too close (<0.3m) or facing a featureless wall/floor. Move to a textured scene 0.5m–1.5m away.

---

### Q6: Why did visual odometry get stuck in `LOST!` (`cov0 = 9999.0`) and how was it fixed?
- **Answer**: Frame-to-Map (F2M) visual odometry needs continuous feature matches. When the camera is static or faced a featureless area, `inliers` dropped below 10, latching the `LOST!` state.
- **Fix Applied**: Added `'Odom/ResetCountdown': '1'` and `'Vis/MaxFeatures': '1000'` to `launch/stereo_inertial_odom.launch.py`. When tracking drops, RTAB-Map now **automatically resets the odometry baseline after 1 lost frame** so it re-locks onto the current camera view immediately.

---

### Q7: What does `mavlink-router` do and why is it used?
- **Answer**: A physical USB serial port (like `/dev/ttyACM0`) can only be opened by **one process at a time**. `mavlink-router` opens `/dev/ttyACM0` once and splits the MAVLink telemetry stream into multiple virtual TCP/UDP endpoints. This allows ROS 2 Vision Bridge, QGroundControl over WiFi, and `px4_control.py` to communicate with the Pixhawk simultaneously without serial port conflicts.
