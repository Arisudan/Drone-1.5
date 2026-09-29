# Project Setup & Operations Guide (`setup.md`)

Everything needed to bring this project up on a fresh machine and fly the pipeline — installing dependencies, building the workspace, launching, and the day-to-day operating commands. This file only covers the **ROS 2 / SLAM / PX4 pipeline**, not the GCS application's own features (see [`guide.md`](guide.md) for that) and not the project's engineering history (see [`Progress.md`](Progress.md) for that).

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [MAVLink Router Setup](#2-mavlink-router-setup)
3. [First-Time Clone & Build](#3-first-time-clone--build-fresh-machine)
4. [Launching the Pipeline](#4-launching-the-pipeline)
5. [Visual Tracking Initialization & Walk-Test](#5-visual-tracking-initialization--walk-test)
6. [Flight Status & the REPL](#6-flight-status--the-repl)
7. [Published ROS 2 Topics](#7-published-ros-2-topics)
8. [Live FPV Video Streaming](#8-live-fpv-video-streaming-reference)
9. [Customizing Launch Parameters](#9-customizing-launch-parameters)
10. [Shutting Down Cleanly](#10-shutting-down-cleanly)
11. [FAQ & Troubleshooting](#11-faq--troubleshooting)

---

## 1. Prerequisites

- **ROS 2 Jazzy** on Ubuntu 24.04.
- **Intel RealSense D435i** plugged into a **USB 3.0/3.2 SuperSpeed** port.
- System packages:
  ```bash
  sudo apt update
  sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-rtabmap-ros python3-opencv python3-numpy
  ```

---

## 2. MAVLink Router Setup

`mavlink-router` acts as a MAVLink proxy/multiplexer, letting multiple processes (the ROS 2 vision bridge, QGroundControl over WiFi, the GCS, and `px4_control.py`) talk to the Pixhawk concurrently without USB/UART port conflicts. A physical serial port can only be opened by one process at a time — `mavlink-router` opens it once and splits the stream into multiple virtual TCP/UDP endpoints.

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
Device=/dev/pixhawk
Baud=115200

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

> `Device=/dev/pixhawk` is a udev symlink that matches the Pixhawk over either USB or UART (whichever is physically connected), so this config never needs editing when switching between them. `Baud=115200` is this vehicle's real configured UART rate — see `Progress.md`'s UART migration entry if you ever need to re-scan it. If you're on the older USB-only setup, `Device=/dev/ttyACM0` with `Baud=921600` also works (USB-CDC doesn't strictly enforce the requested baud).

---

## 3. First-Time Clone & Build (Fresh Machine)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Arisudan/Drone-1.5.git

# Set script permissions
chmod +x ~/Flop/scripts/*.py ~/Flop/scripts/diagnostics/*.py

# Build
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select rtabmap_drone_pkg --symlink-install
source ~/ros2_ws/install/setup.bash
```

Re-run just the build+source block above after any code change; you don't need to re-clone.

---

## 4. Launching the Pipeline

First, verify the hardware link is alive:
```bash
ls -la /dev/pixhawk
sudo systemctl status mavlink-router
```
If `mavlink-router` isn't `active (running)`, or you've just unplugged/replugged the Pixhawk, restart it: `sudo systemctl restart mavlink-router`.

Then, in every new terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

**Full system** (camera + SLAM + PX4 vision bridge + RViz):
```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py \
  enable_px4_bridge:=true \
  launch_rviz:=true
```

**Standalone / bench test** (camera + SLAM mapping only, no PX4):
```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py \
  enable_px4_bridge:=false \
  launch_rviz:=true
```

**One-command headless launcher** — `./camera.sh` at the repo root wraps the sourcing + launch commands above with `launch_rviz:=false` baked in as the default (safe for a Radxa mounted on the drone with no display attached). Override with `./camera.sh launch_rviz:=true` for bench debugging with a monitor attached.

Defaults: `pixhawk_device` is `tcp:127.0.0.1:5760` (routed via `mavlink-router`), `cell_size` is `0.025` (2.5 cm high-definition grid), `launch_rviz` is `true`. This starts: camera (RGB + stereo IR, depth off), stereo odometry, 2.5cm RTAB-Map SLAM, PX4 vision bridge, map thinning node, wall boundary extraction, the TCP map streamer, and the D435i JPEG video streamer on port `8080`.

> A headless Radxa (mounted on the drone, no HDMI) never needs `launch_rviz:=true` — the GCS laptop's own embedded RViz widget provides the same 3D view. Only bring up RViz on the Radxa itself for bench debugging with a monitor attached.

---

## 5. Visual Tracking Initialization & Walk-Test

1. **Boot calibration**: keep the camera stationary for **2–3 seconds** immediately after launch, for IMU bias initialization.
2. **Lock tracking**: point the camera at a detailed, textured surface (cluttered desk, keyboard, bookshelf) **0.5–1.5 m away**, and pan slowly side-to-side (under 0.5 m/s) until the terminal logs `Odom: quality > 0`. The 2D map in RViz turns green (`Status: Ok`) and renders live.
3. **Movement speed while mapping**: stay smooth (under 0.5 m/s) and avoid abrupt high-speed yaw rotations; maintain 0.3–3.5 m distance from walls/obstacles under normal indoor lighting.

If tracking won't lock, run the live diagnostic table:
```bash
python3 ~/Flop/scripts/diagnostics/slam_health_monitor.py
```

---

## 6. Flight Status & the REPL

Check the flight controller's vision-fusion settings (read-only, safe anytime):
```bash
python3 ~/Flop/scripts/diagnostics/verify_ekf2_params.py --port tcp:127.0.0.1:5760
```

Check armed/disarmed state, flight mode, position, and safety-layer health:
```bash
python3 ~/Flop/scripts/diagnostics/px4_control.py --port tcp:127.0.0.1:5760 status
```

Interactive flight REPL:
```bash
python3 ~/Flop/scripts/diagnostics/px4_control.py --port tcp:127.0.0.1:5760 repl
```

| Command | Effect |
|---|---|
| `status` | Arming readiness, flight mode, safety lock metrics |
| `arm` / `disarm` | Arm or disarm motors |
| `takeoff <alt_m>` | Take off to (and hold) an altitude, e.g. `takeoff 1.0` |
| `move <dx> <dy> <dz>` | Move relative to body frame in meters, e.g. `move 0.5 0 0` |
| `yaw <deg>` | Rotate to and hold a heading, e.g. `yaw 90` |
| `goto <N> <E> <D>` | Go to an absolute North/East/Down position in meters |
| `hold` / `loiter` | Switch to `AUTO.LOITER` |
| `land` | Initiate `AUTO.LAND` |
| `mode <PX4_MODE>` | Switch flight mode, e.g. `mode OFFBOARD` |
| `watch` | Live status feed (Ctrl+C to stop watching) |
| `quit` | Exit prompt |

> **Safety note**: with no battery or motors on the Pixhawk, all of the above is 100% safe to try — nothing will actually fly. Once a real airframe with motors/battery is attached, treat every one of these commands as if it will really fly, because it will.

---

## 7. Published ROS 2 Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/map` | `nav_msgs/OccupancyGrid` | High-definition 2.5cm raw RTAB-Map occupancy grid |
| `/map_thin` | `nav_msgs/OccupancyGrid` | Single-pixel LiDAR-quality wall outline map (post-processed with wall lock & noise purger) |
| `/wall_boundaries` | `visualization_msgs/MarkerArray` | Vectorized safe flight polygon boundaries (0.6m interior wall offset) |
| `/odom` | `nav_msgs/Odometry` | Real-time stereo-inertial odometry pose estimate |
| `/cloud_map` | `sensor_msgs/PointCloud2` | Dense 3D point cloud map (disabled by default in RViz for performance) |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | Left infra stereo camera stream |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | Right infra stereo camera stream |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB color stream (640x480@30) — consumed locally by `d435i_video_streamer.py`, not intended for remote ROS 2 subscription over Wi-Fi |
| `/camera/imu` | `sensor_msgs/Imu` | 200Hz fused IMU accelerometer/gyroscope stream |

---

## 8. Live FPV Video Streaming (Reference)

Raw RGB from the D435i (640×480@30, ~220 Mbps uncompressed) would choke a shared Wi-Fi link and start dropping MAVLink packets, so it's never sent over the network directly. `d435i_video_streamer.py` runs **on the Radxa**, compresses each frame to JPEG, and serves it as an MJPEG stream the GCS pulls over plain HTTP/TCP:

```
D435i --(USB 3.0)--> realsense2_camera_node --(/camera/color/image_raw)--> d435i_video_streamer.py
                                                                                    │
                                                            JPEG-compressed, ~6-8 Mbps │  HTTP :8080/video
                                                                                    ▼
                                                              Laptop GCS (video_feed_widget.py)
```

- **Endpoint**: `http://<radxa_ip>:8080/video` (default `http://172.16.101.84:8080/video`) — a standard `multipart/x-mixed-replace` MJPEG stream, viewable directly in a browser or via `cv2.VideoCapture(url)`.
- **Bandwidth**: JPEG quality 80 keeps frames to roughly 15–38 KB depending on scene complexity (~6–8 Mbps at 23–25 FPS real-world, measured on hardware), leaving the rest of the Wi-Fi link free for MAVLink telemetry/commands.
- **Toggle**: disable with `enable_video_streamer:=false` on the launch command if you need to free up CPU/bandwidth.
- Depth (`enable_depth`) is intentionally left disabled for this stream — only color is needed for FPV, and keeping depth off preserves USB/CPU headroom for the stereo VIO + SLAM pipeline.

For the GCS-side viewing experience (fullscreen, floating window, auto-reconnect), see [`guide.md`](guide.md#fpv-camera-tab).

---

## 9. Customizing Launch Parameters

```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py \
  min_obstacle_height:=0.30 \
  max_obstacle_height:=2.0 \
  cell_size:=0.025 \
  launch_rviz:=true \
  enable_video_streamer:=true
```

> ⚠️ **Known gap**: `min_obstacle_height` / `max_obstacle_height` / `cell_size` are declared as launch arguments but are **not actually wired** to the corresponding hardcoded values in `launch/rtabmap_slam.launch.py` — passing them on the command line currently has no effect. See `Progress.md`'s Known Issues section.

---

## 10. Shutting Down Cleanly

Press `Ctrl+C` **once** and give the pipeline a few seconds to exit — `rtabmap` flushes its database to disk on shutdown and `realsense2_camera_node` releases its USB descriptors, both of which take longer than `ros2 launch`'s default 5s/10s SIGINT/SIGTERM grace windows on an SBC. Don't press it again or force-kill.

If you see nodes escalate straight to SIGKILL, relaunch with more headroom:
```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py --sigterm-timeout=10 --sigkill-timeout=15
```

---

## 11. FAQ & Troubleshooting

| # | Question | Answer |
|---|---|---|
| Q1 | Why did `ls -la /dev/pixhawk` and `systemctl status mavlink-router` fail initially? | Pixhawk appears directly as `/dev/ttyACM0`/a native UART device until `mavlink-router` is compiled, installed, and its udev rule creates `/dev/pixhawk` (see [§2](#2-mavlink-router-setup)). |
| Q2 | Why did RViz open but show `! Map Status: Warn` with an empty grid? | `stereo_odometry` needs visual parallax and ≥10 feature inliers between frames. A stationary camera or blank surface gives `inliers = 0` and reports `LOST!`. RTAB-Map waits for valid `/odom` before publishing `/map` — move the camera toward a textured object 0.5–1.5 m away to lock tracking. |
| Q3 | Why did `ros2 topic hz /camera/infra1/image_rect_raw` report "topic does not appear to be published yet" right after launch? | `initial_reset: True` resets the D435i's USB bus on launch, which takes 6–8 seconds to re-enumerate. Topics start once `[camera]: RealSense Node Is Up!` appears in the log. |
| Q4 | Why did `ros2 run rtabmap_drone_pkg slam_health_monitor.py` say "No executable found"? | ROS 2/`colcon` needs Python scripts to be executable (`chmod +x`) before `colcon build`. Run directly with `python3 .../slam_health_monitor.py`, or `chmod +x` and rebuild. |
| Q5 | What do warnings like `Received IMU doesn't have orientation set! It is ignored` mean? | **Harmless/expected** — the D435i outputs raw 6-DOF gyro/accel without an orientation quaternion; RTAB-Map integrates the raw data automatically. `Stereo correspondences rejected` means the camera is too close (<0.3m) or facing a featureless surface — move to a textured scene 0.5–1.5 m away. |
| Q6 | Why did visual odometry get stuck in `LOST!` (`cov0 = 9999.0`)? | Frame-to-Map odometry needs continuous feature matches; a static or featureless view drops `inliers` below 10 and latches `LOST!`. Fixed with `'Odom/ResetCountdown': '1'` and `'Vis/MaxFeatures': '1000'` in `launch/stereo_inertial_odom.launch.py` — RTAB-Map now auto-resets the odometry baseline after 1 lost frame. |
| Q7 | What does `mavlink-router` do and why is it used? | See [§2](#2-mavlink-router-setup) — it lets multiple processes share one physical serial port. |

**Quick troubleshooting cheat-sheet:**

| Symptom | What to do |
|---|---|
| Can't connect / no PX4 heartbeat | Check `/dev/pixhawk` exists, then `sudo systemctl restart mavlink-router` |
| `quality=0` / `matches=0` stuck forever | Camera needs real motion — point it at something close and detailed, hold steady |
| `ros2 node list` shows almost nothing | `ros2 daemon stop && ros2 daemon start` (stale discovery cache, not a real node failure) |
| Everything seems frozen after running a long time | Ctrl+C the launch, wait for it to fully exit, then relaunch fresh |
| `Avoidance`/`Localization` flicker to "STALE" occasionally | Normal — `/map` updates once or twice a second, not continuously |
| GCS "FPV Camera Feed" tab shows "CAMERA NOT DETECTED" | Confirm `d435i_video_streamer` is running (`ros2 node list`) and reachable: `curl -I http://<radxa_ip>:8080/video`. It auto-retries every 2s. |

For the full technical story behind each of these — why each problem happened and exactly what was changed — see [`Progress.md`](Progress.md).
