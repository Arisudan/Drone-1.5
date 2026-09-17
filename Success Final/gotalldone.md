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

---

## Archived README.md (as of 2026-09-15)

Everything below is a full snapshot of `README.md` at the point it was replaced with a
short summary pointing back here. Kept in full so none of this history is lost.

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

5. **Live FPV RGB Video Streaming over Wi-Fi (Depth Stays Off)**:
   - Onboard `d435i_video_streamer.py` node compresses `/camera/color/image_raw` to JPEG and serves it as a low-latency MJPEG stream over HTTP, dropping bandwidth from ~220 Mbps raw to ~6 Mbps compressed so it can share a Wi-Fi link with MAVLink without stuttering.
   - Depth (`enable_depth`) is intentionally left disabled — only color is needed for FPV, and keeping depth off preserves USB/CPU headroom for the stereo VIO + SLAM pipeline.

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
| `/wall_boundaries` | `visualization_msgs/MarkerArray` | Vectorized safe flight polygon boundaries (0.6m interior wall offset) |
| `/odom` | `nav_msgs/Odometry` | Real-time stereo-inertial odometry pose estimate |
| `/cloud_map` | `sensor_msgs/PointCloud2` | Dense 3D point cloud map (disabled by default in RViz for performance) |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | Left infra stereo camera stream |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | Right infra stereo camera stream |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB color stream (640x480@30) — consumed locally by `d435i_video_streamer.py`, not intended for remote ROS 2 subscription over Wi-Fi |
| `/camera/imu` | `sensor_msgs/Imu` | 200Hz fused IMU accelerometer/gyroscope stream |

---

## 📷 Live FPV Video Streaming (Wi-Fi)

Raw RGB from the D435i (640x480@30, ~220 Mbps uncompressed) would choke a shared Wi-Fi
link and start dropping MAVLink packets, so it's never sent over the network directly.
Instead, `d435i_video_streamer.py` runs **on the Radxa**, compresses each frame to JPEG,
and serves it as an MJPEG stream that the laptop GCS pulls over plain HTTP/TCP:

```
D435i --(USB 3.0)--> realsense2_camera_node --(/camera/color/image_raw)--> d435i_video_streamer.py
                                                                                    │
                                                            JPEG-compressed, ~6 Mbps │  HTTP :8080/video
                                                                                    ▼
                                                              Laptop GCS (video_feed_widget.py)
```

- **Endpoint**: `http://<radxa_ip>:8080/video` (default `http://172.16.101.84:8080/video`), a standard `multipart/x-mixed-replace` MJPEG stream — viewable directly in a browser, or via `cv2.VideoCapture(url)`.
- **Bandwidth**: JPEG quality 80 keeps frames to roughly 15-25 KB (~4-6 Mbps at 30 FPS), leaving the rest of the Wi-Fi link free for MAVLink telemetry/commands.
- **Toggle**: disable with `enable_video_streamer:=false` on the launch command if you need to free up CPU/bandwidth for something else.
- **GCS side**: the "FPV Camera Feed" tab in `drone_gcs.py` auto-connects to the stream the first time it's opened. The stream URL is editable in the tab if the Radxa's IP changes — or use the header's Network dropdown (below), which updates it automatically. The tab shows the raw video feed only; there is no overlay drawn on top of it (flight status lives in the header and the Cockpit PFD tab instead).

**Technical stack** (`d435i_video_streamer.py`), ROS topic to browser/GCS pixel:
1. **Source**: `rclpy.Node` subscribes to `/camera/color/image_raw` with `qos_profile_sensor_data` (matches the realsense driver's own camera QoS) — event-driven, no polling.
2. **Decode**: the flat `sensor_msgs/Image` byte buffer is reshaped via NumPy using `msg.step` (row stride, tolerant of row padding), then RGB→BGR channel-flipped since `cv2.imencode` expects BGR.
3. **Compress**: `cv2.imencode(".jpg", ...)` (OpenCV/libjpeg) — the CPU-bound step, measured at ~53-54% of one core at ~24 FPS on real Radxa hardware.
4. **Handoff**: a `threading.Condition`-based `FrameHub` decouples the ROS callback thread from HTTP client threads, so a slow/stalled Wi-Fi client can never block the ROS executor.
5. **Transport**: Python's stdlib `http.server` (`ThreadingHTTPServer`) — no Flask/FastAPI, no external web framework.
6. **Wire protocol**: MJPEG over `multipart/x-mixed-replace` — the same format IP cameras/`mjpg-streamer` use, which is why it plays in a browser tab *and* why `cv2.VideoCapture(url)` decodes it via FFmpeg with zero custom client code.
7. **Reliability**: rides on **TCP**, not raw UDP, so there's no custom packet-loss/fragmentation handling to write — TCP already guarantees ordered, complete delivery of each JPEG chunk.

---

## 🖥️ GCS Ground Station UI

The laptop-side `drone_gcs.py` app's header and Tactical SLAM tab have a few operator-facing features worth knowing about:

- **Network dropdown** (top-left of the header): switch between the known deployment networks (`HTIC_RND` / `DroneBridge5`) and it fills in the Radxa's IP for MAVLink, the TCP map bridge, and the FPV stream **all at once** — no need to re-type the IP in three separate fields. Port and Protocol are independent of this and stay whatever you last set them to (MAVLink's UDP:14550/TCP:5760 choice doesn't depend on which network you're on).
- **Header layout**: two rows — branding + connection controls + Mode/Armed on row 1, live telemetry badges (Battery/VIO/EKF2), RX/TX throughput, and VIO NED position on row 2. A transient notification toast lands in row 2's own free space rather than floating over the workspace below.
- **Cockpit PFD tab**: deliberately minimal — just the SPEED (m/s) and ALT AGL (m) tapes plus a Mode/Armed banner, both tapes driven by live Pixhawk telemetry. No artificial horizon/compass/crosshair; that level of detail lives in the FPV camera feed itself.
- **Reset Map button** (Tactical SLAM tab): wipes the live SLAM map and restarts mapping from empty — for when RTAB-Map's map has drifted or accumulated garbage and you want a clean start without touching the headless Radxa directly. Confirmation-gated (no undo), and **disabled while armed** — never usable mid-flight, since it would pull the EKF2 vision-fusion reference and any live obstacle data out from under an actively flying vehicle. Under the hood it calls RTAB-Map's own `/rtabmap/reset` service plus a matching reset on the wall-thinning node's internal state, over ROS 2 natively with a TCP fallback if the network link is flaky.
  > ⚠️ **This is destructive and cannot be undone.** Only use it while disarmed and certain you want to discard the current map.

---

## ⚙️ Customizing Launch Parameters

You can adjust height limits and cell resolution directly via command line arguments:

```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py \
  min_obstacle_height:=0.30 \
  max_obstacle_height:=2.0 \
  cell_size:=0.025 \
  launch_rviz:=true \
  enable_video_streamer:=true
```

---

## Shutting Down Cleanly

Press `Ctrl+C` once and give the pipeline a few seconds to exit — `rtabmap` flushes its database to disk on shutdown and `realsense2_camera_node` releases its USB descriptors, both of which take longer than `ros2 launch`'s default 5s/10s SIGINT/SIGTERM grace windows on an SBC. If you see nodes escalate straight to SIGKILL, relaunch with more headroom:

```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py --sigterm-timeout=10 --sigkill-timeout=15
```

---

## Physical Mapping & Walk-Test Instructions

1. **Boot Calibration**: Keep the camera stationary for **2–3 seconds** immediately after launch for IMU bias initialization.
2. **Movement Speed**: Move smoothly (under 0.5 m/s). Avoid abrupt high-speed yaw rotations.
3. **Sensor Range**: Maintain 0.3 m to 3.5 m distance from walls and obstacles under normal indoor lighting.

---

## Manual Operation Guide (Running Everything Yourself)

A complete, step-by-step runbook for operating this project without any assistant —
just you, a terminal, and these commands, in order.

### 1. Check the Pixhawk link is alive
Before anything else, confirm the flight-controller connection is working:
```bash
ls -la /dev/pixhawk
systemctl status mavlink-router
```
If `mavlink-router` isn't `active (running)`, or you've just unplugged/replugged the
Pixhawk's USB cable, restart it (needs your password):
```bash
sudo systemctl restart mavlink-router
```

### 2. Build the project (only needed after code changes)
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select rtabmap_drone_pkg --symlink-install
```

### 3. Source the workspace (do this in every new terminal)
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 4. Launch the full pipeline
```bash
ros2 launch rtabmap_drone_pkg drone_rtabmap_all.launch.py
```
- `pixhawk_device` defaults to `tcp:127.0.0.1:5760` (routed via `mavlink-router` without serial port contention).
- `cell_size` defaults to `0.025` (2.5 cm high-definition grid).
- `launch_rviz` defaults to `true` (set `launch_rviz:=false` for headless SBC flight).
- Starts: camera (RGB + stereo IR, depth off), stereo odometry, 2.5cm RTAB-Map SLAM, PX4 vision bridge, map thinning node, wall boundary extraction, and the D435i JPEG video streamer on port `8080` (obstacle detection decoupled for maximum mapping fidelity).

### 5. Get real tracking going
Right after launch, the camera is standing still, so tracking won't lock on yet. Pick up
the camera and:
- Point it at something close and detailed (a bookshelf, cluttered desk, patterned
  surface) — **0.5 to 1.5 meters away**.
- Hold it **steady** for 3-4 seconds — don't sweep around fast, motion blur kills
  tracking just as much as a blank wall does.
- Watch the terminal for lines like `Odom: quality=300` — a number well above 0 means
  it's tracking; `quality=0` means still lost.

### 6. Check status (from a second terminal)
```bash
ros2 run rtabmap_drone_pkg px4_control.py -- --port tcp:127.0.0.1:5760 status
```
Shows armed/disarmed state, flight mode, position, and whether the obstacle-avoidance
and vision-localization safety layers are seeing live data.

### 7. Send commands interactively (the REPL)
```bash
ros2 run rtabmap_drone_pkg px4_control.py -- --port tcp:127.0.0.1:5760 repl
```
Useful commands once inside:
```
status              # check current state
arm / disarm        # arm or disarm the flight controller
takeoff 1.0          # climb 1 meter (force-arms first)
move 0.5 0 0         # move 0.5m forward (dx, dy, dz - body frame)
yaw 90               # rotate 90 degrees
goto 1 0 -1          # go to an absolute position (North, East, Down in meters)
watch                # live status feed, Ctrl+C to stop watching
quit                 # exit
```
**Safety note**: with no battery or motors on the Pixhawk, all of the above is 100% safe
to try — nothing will actually fly. Once a real airframe with motors/battery is
attached, treat every one of these commands as if it will really fly, because it will.

### 8. Useful diagnostic tools
If tracking seems bad and you want to know why, in real time:
```bash
python3 ~/Music/Netrein_sample/scripts/diagnostics/slam_health_monitor.py
```
To check the flight controller's vision-fusion settings (read-only, safe anytime):
```bash
python3 ~/Music/Netrein_sample/scripts/diagnostics/verify_ekf2_params.py --port tcp:127.0.0.1:5760
```

### 9. Shutting down
Go to the terminal running the launch and press **Ctrl+C once**, then wait a few
seconds — don't press it again or force-kill. RTAB-Map needs time to save its map file
and the camera needs time to release its USB connection cleanly.

### Quick troubleshooting cheat-sheet
| Symptom | What to do |
|---|---|
| Can't connect / no PX4 heartbeat | Check `/dev/pixhawk` exists, then `sudo systemctl restart mavlink-router` |
| `quality=0` / `matches=0` stuck forever | Camera needs real motion - point it at something close and detailed, hold steady |
| `ros2 node list` shows almost nothing | `ros2 daemon stop && ros2 daemon start` |
| Everything seems frozen after leaving it running a long time | Ctrl+C the launch, wait for it to fully exit, then relaunch fresh |
| `Avoidance`/`Localization` flicker to "STALE" occasionally | Normal - `/map` updates once or twice a second, not continuously |
| GCS "FPV Camera Feed" tab shows "CAMERA NOT DETECTED" | Confirm `d435i_video_streamer` is running (`ros2 node list`) and reachable: `curl -I http://172.16.101.84:8080/video`. It auto-retries every 2s, so it recovers on its own once the node is up. |

See the **Development Session Log** below for the full technical story behind each of
these — why each problem happened and exactly what fix was applied.

---

## Development Session Log — Problems Encountered & Fixes Applied

This section is a running, chronological record of real issues hit while bringing this
pipeline up on the actual hardware (Radxa SBC + Intel RealSense D435i + Pixhawk 6X flight
controller), what caused each one, and exactly what was changed to fix it. Kept separate
from the sections above so the original quick-start docs stay untouched — read this as
"what actually happened," not a spec.

### 1. Deployed package had silently diverged from the working source tree
- **Problem**: The actual ROS package built and run by `colcon`/`ros2 launch`
  (`~/ros2_ws/src/rtabmap_drone_pkg`, symlinked to `~/Netrein`) was an **older, less-safe
  copy** than `~/Music/Netrein_sample`, which had already received same-day fixes from
  earlier sessions.
- **Root cause**: Two independent copies of the same package existed on disk and had
  never been synced back together.
- **Fix**: Diffed both trees, copied every changed file (`px4_vision_bridge.py`,
  `wall_boundary_node.py`, `map_thinning_node.py`, launch files, `CMakeLists.txt`,
  `README.md`) from the working tree into `~/Netrein`, then rebuilt.
- **Verification**: `diff -rq` between both trees came back clean (only leftover build
  artifacts differed) before rebuilding.
- **Lesson carried forward**: every code change in this project since has been applied to
  **both** `~/Music/Netrein_sample` and `~/Netrein` and diff-checked before moving on.

### 2. `px4_vision_bridge.py` was missing safety-critical fixes (this was found *because* of #1)
- **Problem**: The stale deployed copy had none of: lost/degenerate-frame rejection
  (RTAB-Map signals a lost track with `pose.covariance[0]=9999`), the MAVLink socket
  receive-buffer drain (mavlink-router mirrors the *entire* MAVLink bus onto every
  connected endpoint; an unread buffer climbed to 15KB+ and stalled the router, breaking
  QGroundControl's connection too), and defaulted to opening `/dev/pixhawk` directly
  instead of routing through mavlink-router.
- **Fix**: All of the above were already implemented in the working tree; syncing it in
  (see #1) brought them into the deployed copy.
- **Verification (live, real PX4 v6X)**: Ran the full stack with the bench camera
  stationary — confirmed the bridge correctly **dropped 10,000+ degenerate frames**
  (`cov0=9999`) rather than forwarding a single bad pose to PX4's EKF2. Confirmed
  mavlink-router's receive queue stayed flat (~700 bytes) instead of climbing.

### 3. Bench SLAM tracking failure looked like a bug, wasn't
- **Problem**: With the camera stationary, `stereo_odometry` reported
  `matches=0, inliers=0` continuously — `/odom` never carried a valid pose.
- **Root cause**: Genuine environmental limitation, not a bug — RTAB-Map's frame-to-map
  visual odometry needs real motion/parallax between frames; a static camera (even
  pointed at a textured scene) never accumulates disparity-triangulated 3D points to
  register against.
- **Fix**: None needed in code. Confirmed the diagnosis using the project's own
  `scripts/diagnostics/slam_health_monitor.py`, which is purpose-built for exactly this
  ("Not enough inliers 0/10" bench failure) and reported the correct verdict:
  *"TRACKING FAILURE (timing clean - the scene/registration is the problem)"*.
- **Side fix**: `slam_health_monitor.py` itself crashed on its very first tick
  (`_extent()` returned a bare `0.0` instead of a `(0.0, 0.0)` tuple when fewer than 2
  odom points existed yet, and the caller always unpacked two values). One-line fix.
- **Resolution**: Panning the camera by hand toward a textured, close (~0.5–1.5 m) scene
  produced real tracking (`matches` in the hundreds, `cov0=0.0`), proving the pipeline
  itself was healthy the whole time.

### 4. `ros2 node list` showed only one node — looked like a mass node failure
- **Problem**: After the stack had been running idle for a while, `ros2 node list` showed
  only `/map_thinning_node`; all other nodes appeared to have vanished from the ROS graph,
  even though their OS processes were still alive.
- **Root cause**: The `ros2` CLI's background discovery daemon had a stale cache after a
  long idle period — not an actual node failure.
- **Fix**: `ros2 daemon stop && ros2 daemon start`. All nodes reappeared immediately.

### 5. System clock jump permanently wedged `stereo_odometry`
- **Problem**: After the daemon fix, `/odom` *still* wasn't publishing. The node's own
  log showed: *"Detected not valid consecutive stamps... new stamp should be always
  greater than previous stamp. This new data is ignored"* — every single incoming frame
  was being rejected, forever, explaining the ~100% CPU usage (busy-looping discarding
  data) and zero output.
- **Root cause**: The machine's system clock had jumped **backward** (confirmed via
  `timedatectl`/epoch checks) while this long-running process was idle. The node had
  latched a "previous timestamp" from before the jump and could never get past it once
  the clock settled — a monotonicity assumption that a real reboot would reset, but a
  clock skew on a continuously-running process does not.
- **Fix**: Confirmed NTP had re-synced the clock, then did a clean stack restart
  (`SIGINT` to the launch process, wait for full exit, relaunch) so every node re-latched
  its timestamp baseline against the now-stable clock.
- **Verification**: Zero timestamp-rejection errors in the fresh log; stereo odometry
  quality jumped from stuck-at-zero to real tracking (300–400 quality score) on the very
  next camera pan.

### 6. Physical Pixhawk USB link silently died mid-session (found while diagnosing #7 below)
- **Problem**: `verify_ekf2_params.py` timed out with zero response. A raw heartbeat
  probe confirmed the real flight controller (`sysid=1`) had **stopped sending
  heartbeats entirely** over the mavlink-router link — only QGroundControl's own mirrored
  heartbeat (`sysid=255`) was still present.
- **Root cause**: `lsusb` showed the Pixhawk's USB device number had changed
  (`Device 008` → `010`) since session start — a genuine USB re-enumeration (unplug/replug
  or a bus reset) had happened. `mavlink-routerd` still held a stale file handle to the
  *old* device node and looked "active (running)" in `systemctl status` while actually
  talking to nothing.
- **Fix**: `sudo systemctl restart mavlink-router` so it reopened the current
  `/dev/ttyACM0`. This happened **twice** in the same session — once for an
  already-occurred re-enumeration, and again immediately after a deliberate cable replug
  (confirmed via polling `lsusb` live: `010` → `012`, then stable) caused a second one.
- **Verification**: Raw heartbeat probe showed `sysid=1, autopilot=PX4` flowing again
  each time, before proceeding.

### 7. `verify_ekf2_params.py` had the same "wrong system" bug as `px4_control.py` (#9 below)
- **Problem**: Reported `EKF2_EV_CTRL = not present on this firmware` — implausible for a
  standard PX4 parameter — and then hung entirely on the next parameter read.
- **Root cause**: Used pymavlink's plain `wait_heartbeat()` + `self.master.target_system`,
  which (same as the bug in `px4_control.py`) isn't reliably populated on a
  mavlink-router-shared link with multiple systems on the bus, and can silently target
  the wrong one (or `sysid=0`, an uninitialized default).
- **Fix**: Same pattern as the `px4_control.py` fix below — filter for a real autopilot
  heartbeat (`autopilot != MAV_AUTOPILOT_INVALID`) and read `sysid`/`compid` off the
  message itself.
- **Verification (real values, once the link in #6 was restored)**:
  `EKF2_EV_CTRL=15` (all vision channels fused: position, height, yaw, velocity),
  `EKF2_GPS_CTRL=0` (GPS correctly disabled indoors), `EKF2_HGT_REF=3` (vision owns
  height), `EKF2_OF_CTRL=1` / `EKF2_RNG_CTRL=1` (optical flow + rangefinder also fusing
  in parallel with vision). Assessed as correctly configured for this pipeline.

### 8. `/home/radxa/Downloads/mavlink_radiomaster-main/px4_control.py` — evaluated and adopted
- Confirmed this third-party script genuinely does what it looks like: a PX4
  offboard-control CLI/REPL (`arm`/`disarm`/`mode`/`takeoff`/`land`/`rtl`/`goto`/`vel`/
  `move`/`yaw`/`mission`/`pattern`), directly compatible with our mavlink-router setup
  (`--port tcp:127.0.0.1:5760`).
- Copied into `scripts/diagnostics/px4_control.py` in both project trees, given a shebang
  + executable bit, and wired into `CMakeLists.txt` so it builds as a proper
  `ros2 run rtabmap_drone_pkg px4_control.py` executable.

### 9. `px4_control.py` bug: wrong-system targeting (same class of bug as #7)
- **Problem**: `Fleet.connect()`'s plain `wait_heartbeat()` raced against
  QGroundControl's mirrored heartbeat on the shared mavlink-router link and could latch
  onto `sysid=0` (an uninitialized default) instead of the real FMU (`sysid=1`).
- **Fix**: Replaced with an explicit loop filtering for `autopilot != MAV_AUTOPILOT_INVALID`,
  reading `sysid`/`compid` directly off the accepted message.
- **Verification (live, real hardware)**: `status` correctly showed `sysid=1,
  autopilot=MAV_AUTOPILOT_PX4`. `arm` was correctly **rejected by PX4 itself**
  ("Resolve system health failures first" — no battery connected, exactly expected).
  `takeoff` force-armed and entered OFFBOARD successfully. `disarm` cleanly returned the
  vehicle to a safe resting state. All confirmed against the physical Pixhawk 6X with no
  motors/battery attached (bench-safe).

### 10. `px4_control.py` bug: `Mode` always showed `UNKNOWN`
- **Root cause**: `print_status()` used `mavutil.mode_string_v10()`, which decodes
  ArduPilot-style flight modes, not PX4's packed `(main_mode, sub_mode)` custom-mode
  encoding — it always returns `UNKNOWN` for PX4, even in genuine `OFFBOARD`.
- **Fix**: Added `px4_mode_name()`, decoding `custom_mode` via the same `PX4_MODES`
  table the script already uses to *set* modes.
- **Verification**: Independently confirmed via a raw heartbeat probe
  (`main_mode=6, sub_mode=0` = `OFFBOARD`) that the fixed decoder now matches ground
  truth, where before it said `UNKNOWN` for the same real state.
- **Related, unfixed, and just noted**: switching mode away from `OFFBOARD` via
  `mode POSCTL` didn't stick while the background setpoint-stream thread was still
  running from an earlier `offboard` call in the same REPL session — not investigated
  further.

### 11. Learned: OFFBOARD mode cannot "stay" without a live process
- Asked to "leave it in OFFBOARD, disarmed" — checked and found PX4 had already fallen
  back to `POSCTL` on its own. **Root cause**: OFFBOARD requires a continuous ≥2 Hz
  setpoint stream from a live connection; once the commanding REPL process exits, the
  stream stops and PX4's OFFBOARD-loss failsafe switches away automatically. There is no
  way to "leave it in OFFBOARD" without a process staying alive to keep streaming.

### 12. Obstacle-avoidance / EV-failsafe layer was wired up for the first time
`px4_control.py` ships with an obstacle-avoidance + localization-staleness safety layer
that listens on a separate port (`--obstacle-port`, default `udpin:0.0.0.0:14541`) for
two message types it never actually received from this project before (`--no-avoid` was
used in every test up to this point):
- **`px4_vision_bridge.py`** gained an `extra_urls` parameter (default
  `udpout:127.0.0.1:14541`) that fans out a copy of every good `VISION_POSITION_ESTIMATE`
  to that port — feeding the EV-staleness half of the safety layer with a real
  heartbeat. Sent *after* the existing lost-frame rejection, so a lost SLAM frame
  produces no heartbeat here either, exactly as intended.
- **New node `scripts/obstacle_distance_bridge.py`**: this airframe has no lidar, so
  instead of the reference toolset's real-360°-lidar approach, it raycasts a 72-sector
  body-FRD ring directly against RTAB-Map's own `/map` occupancy grid (using the drone's
  current `/odom` pose as the raycast origin/heading), and sends it as a real MAVLink
  `OBSTACLE_DISTANCE` message to the same port. Wired into `CMakeLists.txt` and staged
  into `drone_rtabmap_all.launch.py` (launches ~9.2 s in, after `/odom` and `/map` are
  both up).
- **Bug found and fixed, blocking the whole thing**: `px4_control.py` never set
  `MAVLINK20`, so its *own* connection defaulted to MAVLink v1 — whose 8-bit message-ID
  field cannot even represent `OBSTACLE_DISTANCE` (MAVLink id 330, a v2-only message).
  That's why `VISION_POSITION_ESTIMATE` (id 102, fits in v1) worked immediately while the
  obstacle ring silently never parsed on the receiving end, even though it was
  genuinely being sent. Fixed with one line (`os.environ.setdefault("MAVLINK20", "1")`
  before the pymavlink import), synced to both project trees and the original
  `~/Downloads/mavlink_radiomaster-main/px4_control.py`.
- **Verification (live, real occupancy-grid data)**: `px4_control.py`'s own status
  eventually showed a genuine live ring —
  `Avoidance: ON, ring 338ms  F=0.9 R=0.7 B=clr L=1.4 m` — alongside
  `Localization: ON, vision Xms old`. Both halves of the safety layer are now driven by
  real sensor data instead of being disabled.
- **Known, accepted limitation**: the ring flickers between fresh and `STALE` because
  RTAB-Map's own `/map` only published roughly once per second, with multi-second jitter
  at the time — inherent to RTAB-Map's own grid-update cadence (see #13), not a bug in
  the new bridge node.

### 13. RTAB-Map map-update rate tuning
- **Problem**: `/map` published irregularly (~0.6–1 Hz, with gaps up to ~4.9 s), and the
  gaps grew as the mapped area grew during a session.
- **Root cause, identified from three specific parameters** in
  `launch/rtabmap_slam.launch.py`:
  - `Grid/CellSize: 0.025` (2.5 cm) — cell count scales as `1/CellSize²`, so this was 4x
    more cells to rasterize per update than necessary.
  - `Grid/GlobalFullUpdate: true` — recomputes the *entire* global grid on every
    publish, which gets more expensive as the map grows (matches the observed
    0.6 s → 4.9 s gap growth).
  - `Rtabmap/DetectionRate: 2.0` — was already below the rate actually being achieved
    (~0.6–1 Hz), so on its own this wasn't the bottleneck.
- **Fix applied**: `Grid/CellSize` → `0.05` (still finer than
  `obstacle_distance_bridge.py`'s own 5 cm ray step, so nothing downstream lost
  precision), `Grid/GlobalFullUpdate` → `false` (incremental updates instead of a full
  rebuild every time), `Rtabmap/DetectionRate` → `3.0` (raised now that each update is
  cheaper).
- **Verification, confirmed live**: RTAB-Map's own per-cycle log line showed
  `Rate=0.33s` matching the new `DetectionRate=3.0`, and `Maps update=0.0002–0.014s`
  (down from a full-map-recompute cost that used to grow with map size). A clean
  **before/after `/map` Hz** comparison was obtained once tracking was solid enough to
  grow the map past a single node (confirmed via RTAB-Map's own log: working memory grew
  `WM=1 → WM=2`, i.e. a real second keyframe was accepted, not stuck):
  **`/map` now publishes at ~1.3–2.3 Hz (settling ~2.1–2.3 Hz), with the worst-case gap
  capped at ~1.2 s** — versus ~0.6–1.0 Hz average and gaps growing up to ~4.9 s before
  this tuning. Roughly a 2–3x rate improvement, and — more importantly — the gap no
  longer grows unboundedly as the map accumulates, which was the actual complaint.
- **False alarm along the way**: at one point `rtabmap`'s process appeared to have
  vanished from `ps aux` — this was an artifact of a bad `grep` pattern
  (`"rtabmap-"`, matching the log file's launch-prefix label, not the real process name
  `rtabmap`), not an actual crash. The process was confirmed alive and processing the
  whole time.

### 14. Conceptual question: should everything run at one shared frequency?
Answered **no** — different components in this stack are multi-rate on purpose, not by
accident: PX4's OFFBOARD mode has a **hard ≥2 Hz requirement** (miss it and PX4 fails
safe out of the mode) which is why `px4_control.py` streams setpoints at 20 Hz; IMU needs
to run fast (~200 Hz) because EKF2's attitude estimate is only as good as how often it
can propagate between slower corrections; and full SLAM (loop closure + global grid
rebuild) is genuinely expensive, which is *why* `Rtabmap/DetectionRate` deliberately
throttles it below the camera's own 30 Hz. Forcing one shared rate would mean either
breaking OFFBOARD/degrading EKF2 (picking the slowest rate) or making RTAB-Map
perpetually miss its own processing deadline (picking the fastest one). The existing
design — each producer running at whatever its own sensor/computation actually supports,
each consumer (the stale-odom watchdog, the EV-staleness failsafe, the obstacle ring's
`STALE` indicator) built to tolerate that gracefully — is the correct architecture for
this kind of layered perception+control system.

### 15. Conceptual question: is the drone's height "fixed" in the 2D occupancy grid?
Clarified: the drone's real height (Z) is **not** fixed anywhere — it's tracked
continuously by RTAB-Map's stereo SLAM and fused into PX4's EKF2 (`EKF2_HGT_REF=3`,
vision owns height; the rangefinder/optical-flow module also fuses in per #7 but as a
secondary correction, not the primary source). The actual gap is that the **2D occupancy
grid itself** has no Z dimension: RTAB-Map flattens its internal 3D map using a *fixed*
height band relative to the floor (`Grid/MinObstacleHeight: 0.30` /
`Grid/MaxObstacleHeight: 2.0`), not relative to the drone's current altitude, so
`obstacle_distance_bridge.py`'s ring is altitude-blind by construction. Explicitly
**left unchanged** at the user's request — noted here only so the limitation is
documented, not silently forgotten.

### 16. Whole pipeline died when `mavlink-router` was restarted (found live, in actual use)
- **Problem**: Restarting `mavlink-router` (needed for #6's USB-link issue) took down
  the **entire pipeline** - camera, SLAM, everything - not just the PX4 bridge, even
  though only the bridge's connection was actually affected.
- **Root cause**: `px4_vision_bridge.py` holds a live TCP connection to
  `mavlink-router`; restarting that service drops it out from under the node. `ros2
  launch`'s default behavior when any node exits unexpectedly is to tear down the
  *entire* launch tree, not just that one node - confirmed live: the whole pipeline's
  process disappeared entirely after a `mavlink-router` restart.
- **Fix, two parts**:
  1. `px4_vision_bridge.py`'s `_drain_incoming()` now explicitly distinguishes a real
     connection failure (`ConnectionResetError`/`BrokenPipeError`/other `OSError`) from
     the normal "nothing to read this tick" case (`BlockingIOError` - itself technically
     an `OSError` subclass, which is why it has to be caught *first* or it would shadow
     the real-failure branch). On a real failure it now marks the connection dead
     immediately, so the existing reconnect-on-next-odom logic in `odom_callback()`
     kicks in right away instead of waiting for a send to fail first.
  2. `px4_bridge_node` in `drone_rtabmap_all.launch.py` is now `respawn=True,
     respawn_delay=2.0` (matching the same pattern this project already uses for the
     camera node's own USB-recovery). Belt-and-suspenders: even if some other,
     unanticipated failure mode ever crashes this node outright, only *this* node
     restarts, not the whole pipeline.
- **Verification, live**: launched the full pipeline, then restarted `mavlink-router`
  while it was running. Log evidence of the fix engaging exactly as designed:
  ```
  MAVLink connection error - will reconnect on next odom message.
  Connecting MAVLink to tcp:127.0.0.1:5760 (via mavlink-router)...
  MAVLink connected to PX4 system!
  ```
  All 6 pipeline processes (camera, stereo odometry, RTAB-Map, the bridge itself, map
  thinning, obstacle bridge, wall boundary) kept the exact same PIDs throughout - nothing
  crashed, nothing needed `ros2 launch` to restart anything.

### 17. High-Definition 2.5cm Occupancy Grid & Thinning Restoration (Obstacle Detection Decoupled)
- **Problem**: The 2D occupancy grid in `Flop` appeared degraded, fragmented, and blurry in RViz compared to the reference `SLAM` pipeline, and `/map_thin` frequently failed to publish or render any output.
- **Root Cause Analysis**:
  1. **Grid Resolution Mismatch (`Grid/CellSize: 0.05` vs `0.025`)**: In an earlier attempt to reduce CPU load for raycasting in `obstacle_distance_bridge.py`, cell size was doubled to 5.0 cm. In `map_thinning_node.py`, the noise filter discards clusters smaller than `min_wall_area_pixels = 20`. At 5.0 cm, 20 pixels equals $0.05 \text{ m}^2$ (a $22 \times 22 \text{ cm}$ cluster). Real continuous walls separated by small stereo occlusions fell below this threshold and were purged as noise, causing walls to vanish. Surface normal estimation (`Grid/NormalsSegmentation: true`) was also degraded at 5 cm.
  2. **Incremental Update Starvation (`Grid/GlobalFullUpdate: false`)**: RTAB-Map stopped republishing the global `/map` on each cycle, switching to incremental patches on `/map_updates`. However, `map_thinning_node.py` and `wall_boundary_node.py` only subscribe to `/map`. As a result, the thinning node was starved of input data and never executed.
  3. **ROS 2 QoS Durability Mismatch**: `rtabmap` publishes `/map` with `TRANSIENT_LOCAL` durability (latched). `map_thinning_node.py` and `wall_boundary_node.py` subscribed with `VOLATILE` durability (ROS 2 default for queue depth). Because `map_thinning_node` was delayed by 9 seconds, the initial `/map` arrived before subscription, causing the node to miss the latched map entirely.
  4. **CPU Contention**: Running the 72-sector raycasting loop in `obstacle_distance_bridge.py` on the Radxa ARM cores drove CPU usage up, which was the original reason `CellSize` and `GlobalFullUpdate` were compromised.
  5. **Launch Parameter Typo**: `drone_rtabmap_all.launch.py` passed `min_wall_cluster_size: 20` instead of `min_wall_area_pixels: 20`, which ROS 2 silently ignored.
- **Fixes Applied**:
  1. **Restored 2.5 cm Resolution & Global Updates**: In `rtabmap_slam.launch.py`, set `'Grid/CellSize': '0.025'`, `'Grid/GlobalFullUpdate': 'true'`, and `'Rtabmap/DetectionRate': '2.0'`.
  2. **Decoupled Obstacle Detection**: Removed `obstacle_distance_bridge.py` from `drone_rtabmap_all.launch.py`, eliminating raycast compute overhead and dedicating all SBC processing power to high-resolution SLAM and thinning.
  3. **QoS Profile Fixed**: Configured both `map_thinning_node.py` and `wall_boundary_node.py` with `QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)` for both their `/map` subscriptions and `/map_thin` / `/wall_boundaries` publishers.
  4. **Launch Defaults & Parameter Corrected**: Set launch parameter `'min_wall_area_pixels': 20`, default `cell_size: 0.025`, and default `pixhawk_device: tcp:127.0.0.1:5760`.
- **Live Hardware Verification**:
  - `/map`: Verified at `0.0250 m` (2.5 cm) resolution, `350 x 326` grid, `4,303` occupied cells, `57,729` free cells with active ray-traced clearing.
  - `/map_thin`: Verified at `0.0250 m` resolution, publishing `1,262` single-pixel skeleton cells with noise purged.
  - `/wall_boundaries`: Verified publishing 5 polygon boundary marker arrays outlining clean room borders in RViz.
  - Tracking: `stereo_odometry` maintained quality > 200, and Pixhawk EKF2 confirmed vision lock (`Localization: ON, vision ~30-100ms old`).

### 18. `DroneBridge5` Wi-Fi AP silently blocks Radxa↔laptop traffic (AP isolation)
- **Symptom**: after switching both the Radxa and the laptop onto a `DroneBridge5` access point, MAVLink wouldn't connect, the SLAM map only sparsely updated, and the FPV feed wouldn't load — despite both devices successfully associating and getting valid DHCP leases (Radxa `192.168.1.2`, laptop `192.168.1.10`).
- **Root cause**: this AP does not route traffic between its own two WiFi clients. A direct ping from the Radxa straight to the laptop's IP over that AP failed 100%, and even the AP's own gateway didn't answer ARP/ping — the textbook signature of **AP/client isolation**, common on hardware built for a single dedicated link rather than general multi-host networking.
- **Confirmed which layer, exactly**: plugging the Radxa into `DroneBridge5`'s wired LAN port (`enp1s0`) while the laptop stayed on its WiFi side worked immediately (4/4 ping, ~1-4ms) — isolation on this unit is WiFi-to-WiFi only, not wired-to-wireless.
- **Fix applied**: added a Network dropdown to the GCS header (see "GCS Ground Station UI" above) so the operator can pick the working network per-session, rather than editing IPs by hand. Reconfiguring the AP itself to disable isolation was attempted but blocked — admin credentials had already been changed from factory defaults by whoever set the unit up, and physical access for a factory reset wasn't available in-session.
- **Side effect discovered while testing the wired workaround**: the wired interface took over as the *default route* (lower metric than WiFi), and since `DroneBridge5` has no internet uplink, this would silently kill the Radxa's general internet access while the workaround is in use. Not yet fixed — would need an explicit route metric bump on the wired connection when this AP is in play.

### 19. Deploying this repo to a second machine (x86_64 laptop) surfaced two silent-failure modes
- **A copied `.venv` doesn't travel across architectures.** The laptop had a pre-existing `.venv` that turned out to be an ARM aarch64 Python build (apparently copied from this Radxa at some point) — completely non-functional on the laptop's real x86_64 CPU, and it failed in a confusing way (`cannot execute binary file`-style errors that the shell partially garbled rather than a clean rejection). Fix: ignore/delete a foreign-arch `.venv` and use the target machine's own system Python — this project's actual dependencies (PyQt5, OpenCV, NumPy, pymavlink) are common enough that they were already present system-wide.
- **A GUI app launched over SSH needs the real Wayland auth cookie, not just `DISPLAY`.** On a GNOME/Wayland laptop, `ssh ... DISPLAY=:0 python3 drone_gcs.py` fails with `Authorization required, but no authorization protocol specified` — the default `~/.Xauthority` isn't what's actually in effect. Fix: also export `XAUTHORITY=/run/user/<uid>/.mutter-Xwaylandauth.<random>`, found by reading `/proc/<pid>/environ` of an already-running desktop process (e.g. `gnome-shell`) on that machine.

### 20. Added a third network preset (`DroneNet`) to the GCS header dropdown
- A NetworkManager connection-sharing/hotspot link: Radxa is always the shared-connection gateway at `10.42.0.1` (the laptop gets a DHCP lease like `10.42.0.200`, but the GCS never needs that value since it only ever connects *to* the Radxa, never the other way).
- Pure data change — `KNOWN_NETWORKS` in `top_status_strip.py` gained one tuple (`("DroneNet", "10.42.0.1")`). No backend wiring needed: the existing `network_changed(ip)` signal already fans any preset's IP out to MAVLink, the TCP map bridge, and the FPV stream, since all three are driven off the same signal, not hardcoded per-network logic.
- Verified offscreen: dropdown now lists `HTIC_RND` / `DroneBridge5` / `DroneNet` / `Custom...`, and selecting `DroneNet` correctly fills the IP field with `10.42.0.1`.

### 21. Split `d435i_video_streamer.py`'s codec into its own file, then merged it back — both directions verified live on real hardware
- **Split** (`scripts/rgb_frame_codec.py`): pulled the two genuinely ROS-independent pieces — `extract_bgr_frame()` (reshapes the raw `sensor_msgs/Image` byte buffer into a BGR ndarray, tolerating row padding via `msg.step`) and `encode_jpeg()` (the `cv2.imencode` JPEG compression step) — out of the node file into a standalone module with zero ROS imports, so it could be read/shared/tested independently of the HTTP server + ROS plumbing. Verified with unit tests (rgb8/bgr8 paths, unsupported-encoding rejection, row-padding tolerance, JPEG round-trip decode) and a full real-hardware pipeline run: identical live numbers to before the split (~21-24 FPS, ~34 kB/frame, ~6 Mbit/s).
- **Merged back** (same day, on request): reverted `d435i_video_streamer.py` to a single self-contained file, deleted `rgb_frame_codec.py`, reverted the `CMakeLists.txt` install-list entry, rebuilt, and re-verified live on real hardware again (~22-24 FPS, ~36.5 kB/frame, ~7 Mbit/s — matching, confirming the merge changed nothing behaviorally).
- **Lesson worth keeping**: runtime performance is identical either way (same functions, same process, an extra one-time module `import` at startup is the only cost) — the one-file-vs-two-file choice is purely about code organization/shareability, not speed. Confirmed by measuring real FPS/bitrate before and after both changes rather than assuming.
- **Process-management mistake caught mid-verification**: after the merge-back, the first shutdown attempt sent `SIGINT` to the `nohup`-spawned bash wrapper PID instead of the actual `ros2 launch` process underneath it, leaving orphaned pipeline nodes holding port 8080 and causing a respawn-loop (`Address already in use`) on the very next launch attempt. Caught via `ss -ltnp`/`ps` inspection, cleaned up every orphaned process explicitly, and relaunched once cleanly — no impact on the actual code changes being verified.

### 22. Real-hardware C2 (Command & Control) round-trip validation: GCS laptop → Radxa → Pixhawk → back
Ran the actual production path (laptop over real Wi-Fi → Radxa's `mavlink-router` → Pixhawk via USB → reply back the same way) using this project's own diagnostic tools, plus a purpose-written instrumented latency test, rather than assuming the link works because heartbeats are visible in the GCS UI.
- **`duplex_check.py`** (from the laptop, `udpout:172.16.101.84:14550`): downlink **PROVEN** (real `sys=1` PX4 heartbeat) and uplink **PROVEN** (`AUTOPILOT_VERSION` request answered). Full live rate breakdown captured: 197.1 Hz total telemetry, ~11.7 KB/s (IMU 50Hz, attitude-quaternion 50Hz, position/odometry/attitude/VFR_HUD at 10Hz, etc.).
- **`c2_validate.py`** (`--arm` included, bench-safe with no battery/motors attached): **4/4 commands acknowledged** — mode → LOITER `ACCEPTED`, mode → RTL `ACCEPTED`, a deliberately-undefined command correctly `UNSUPPORTED`, and an arm attempt correctly `TEMPORARILY_REJECTED` (proves the *reply* path works too, since PX4 had to actually answer, not just receive).
- **Custom instrumented round-trip latency test** (written fresh, since neither existing tool reports per-command timing): `PARAM_REQUEST_READ → PARAM_VALUE` 15/15 received, 49–261 ms (avg 136 ms); `COMMAND_LONG → COMMAND_ACK` 15/15 received, 60–162 ms (avg 109 ms).
- **Bug caught in the test script itself, same known class as #7/#9 below**: the first version used a plain `wait_heartbeat()`, which raced against the router's own mirrored traffic and latched `sys=0/comp=0` instead of the real Pixhawk. Result: one dropped reply and a 1245 ms outlier. Fixed the same way the project's own `px4_control.py`/`verify_ekf2_params.py` bugs were fixed (filter for a real autopilot heartbeat, never trust a bare `wait_heartbeat()` on a shared mavlink-router link) — after the fix, both tests went to a clean 15/15 with tight, consistent timing.
- **Real finding, not a bug**: baseline `ping` to the laptop over `HTIC_RND` showed 137–289 ms RTT, matching the corrected command-latency numbers above — this is genuine current Wi-Fi link latency, not a code or router issue. Still comfortably inside PX4's OFFBOARD 2 Hz (500 ms) setpoint deadline, so no functional risk today, but the first thing to check if commands ever start feeling laggy.
- **Not checked**: no `sudo` access in this session, so `mavlink-router`'s own internal log couldn't be tailed directly for silently-dropped packets — the conclusion rests on 100% application-level ACK rates across two independent tools plus the custom test, not on router-internal instrumentation.

### 23. Migrated Pixhawk from USB to a real UART wire - multiple real failures found and fixed live
- **Symptom**: after physically switching the Radxa↔Pixhawk link from USB to a GPIO-header UART wire, `mavlink-router.service` was found stuck in a failing `ExecStartPre` loop - `/dev/ttyACM0` and the `/dev/pixhawk` udev symlink (USB VID/PID-matched, from `99-pixhawk.rules`) were both gone, replaced by a new native device `/dev/ttyMSM0`.
- **First real hazard caught**: the wire was initially on pins 13/15, which turned out to be **UART0 - this board's active kernel boot console** (`console=ttyMSM0,115200n8` in `/proc/cmdline`, plus a live `serial-getty@ttyMSM0` login prompt already running). Using it for MAVLink would have collided with the OS console/login itself. Recommended switching to a different UART instance instead of disabling the console (which would cost local recovery access).
- **Chose UART6** (pins 16 TX / 18 RX, per Radxa's own 40-pin GPIO pinout docs) - a pre-built but disabled overlay (`qcs6490-radxa-dragon-q6a-uart6.dtbo.disabled`) already existed on the board for it.
- **Overlay enabling took three real attempts**:
  1. Setting `U_BOOT_FDT_OVERLAYS` in `/etc/default/u-boot` to just the uart6 filename **silently dropped every other active overlay** (camera + 4 I²C buses). Root cause found by reading `/usr/sbin/u-boot-update`'s actual shell logic: leaving that variable unset makes the script auto-include *every* `.dtbo` file present; setting it to anything switches to an exact allow-list instead. Fixed by reverting the variable to unset and using the file's own `.dtbo`/`.dtbo.disabled` suffix as the real enable/disable mechanism (which is what `rsetup` does under the hood, confirmed by reading the script rather than guessing).
  2. Still didn't apply even with the file correctly renamed. Decompiling the overlay binary directly (`dtc -I dtb -O dts qcs6490-radxa-dragon-q6a-uart6.dtbo`) revealed why: **UART6 and I²C bus 6 are two personalities of the exact same physical Qualcomm GENI Serial Engine** (`geniqup@9c0000/serial@998000` vs `.../i2c@998000`) - the overlay's own fragments explicitly disable `i2c6` (`fragment@1: status="disabled"`) to enable `uart6` (`fragment@2: status="okay"`), and U-Boot wasn't merging it while that conflict was live.
  3. Third attempt, after the operator explicitly accepted disabling `i2c6` via `rsetup`, succeeded - confirmed via `/proc/device-tree/.../i2c@998000/status` (now `disabled`) and `.../serial@998000/status` (now `okay`), bound as `/dev/ttyHS1` (the `qcom_geni_serial` driver uses different naming than the console's driver, which produces `ttyMSM*` names - this cost some initial confusion when the expected `/dev/ttyMSM1` never appeared).
- **A wiring mix-up mid-diagnosis**: an initial report of "pins 6, 8, 10" turned out to be **UART5** - a completely different Serial Engine, with no overlay shipped for it at all on this board. Resolved once the operator confirmed the wire had actually been moved to pins 14/16/18 (UART6), matching the software side.
- **Made the setup resilient to either USB or UART, only one connected at a time**: added a second udev rule (`/etc/udev/rules.d/99-pixhawk-uart.rules`, `KERNEL=="ttyHS1"`) alongside the existing USB rule (`KERNEL` implicit via `idVendor`/`idProduct` match), both creating the same `/dev/pixhawk` symlink. `config/mavlink-router.conf`'s `Device=` now points at `/dev/pixhawk` instead of a hardcoded path, so switching between USB and UART physically never requires a config change.
- **Final blocker - baud mismatch**: `Baud=921600` in the config was inherited from the USB days, where baud is largely cosmetic (USB-CDC doesn't really enforce a requested baud the way real UART does). A direct baud-scan test written on the spot (cycling `/dev/pixhawk` through 57600/115200/921600/460800/38400/19200/9600, bypassing `mavlink-router` entirely) found the Pixhawk's real configured rate: **115200**. Config corrected and confirmed live - this was the one change that actually got MAVLink flowing after everything else was already correct.
- **Final verification, full round-trip on real hardware** (same tools as the earlier USB-based C2 validation, for direct comparison):
  - `duplex_check.py`: downlink **PROVEN** (real PX4 heartbeat), uplink **PROVEN**. Live rate table: 68.7 Hz total, ~4.9 KB/s (lower than USB's 197 Hz / 11.7 KB/s - expected, since 115200 baud has much less raw throughput than USB).
  - `c2_validate.py --arm`: **4/4 commands acknowledged** - LOITER `ACCEPTED`, RTL `ACCEPTED`, undefined command correctly `UNSUPPORTED`, arm attempt correctly `TEMPORARILY_REJECTED` (no battery, exactly expected).
- **Lesson carried forward**: on this board, "enable an overlay" always means renaming the `.dtbo.disabled` file, never setting `U_BOOT_FDT_OVERLAYS` directly (that variable should stay unset/commented) - and any GENI-based UART/I²C/SPI instance sharing the same address is mutually exclusive with the others at that address, never simultaneously available.

### 24. COMMAND_ACK console-spam bug - two dedup attempts before finding the correct one
- **Symptom**: the real GCS console occasionally flooded with dozens of identical `[ACCEPTED] Pixhawk confirmed <cmd> in X.XXs` lines for one dispatched command - worst case ~90 lines for a single `NAV_TAKEOFF`.
- **First fix attempt (sequence-number dedup)**: dropped a `COMMAND_ACK` if its MAVLink sequence number matched the immediately-preceding one. Didn't work - direct debug instrumentation showed each duplicate-looking ACK actually carried a *different* sequence number, meaning these are genuine repeated wire packets (a real PX4/link retransmission), not the same packet delivered twice.
- **Second fix attempt (content + time-window dedup)**: suppress a repeat `(cmd_id, result_code)` within a 1.5s sliding window. This correctly collapsed the `NAV_TAKEOFF` flood, but broke something else: `MAV_CMD_SET_MESSAGE_INTERVAL` (511) is dispatched ~7 times in a row during `_configure_streams()` (once per telemetry stream type) - all genuinely different requests sharing the same `cmd_id`, and the time-window approach silently collapsed 6 of those 7 real confirmations into 1.
- **Correct fix**: a per-`cmd_id` dispatch token, incremented immediately before each real `command_long_send()` in `arm()`/`disarm()`/`set_mode()`/`takeoff()`/`emergency_kill()` (two tokens there, one per command it sends). The `COMMAND_ACK` handler only reports the first ACK matching the *current* token for that `cmd_id` - true retransmit duplicates (same token, arrives again) are suppressed, a fresh dispatch of the same command type (new token) is always reported once. `request_message_interval()` is deliberately left un-instrumented (no token ever registered for `cmd_id=511`), so all of its genuinely-distinct dispatches pass through unsuppressed - correct, since `COMMAND_ACK` has no way to say which of several outstanding `SET_MESSAGE_INTERVAL` requests it's answering, so per-request dedup for that command isn't even possible, and isn't needed (7 lines from a known bounded burst was never really "spam").
- **A genuine debugging trap encountered along the way**: partway through, my own ad-hoc test harness reported the flood as "fixed" (clean output), but this was a false positive - a bug in the harness's own crude line-count diffing between polls, not the real fix working yet. Only trusted once a call-counter was placed directly inside the actual `_on_command_ack_received` slot and `_handle_msg`'s `COMMAND_ACK` branch, proving the real call counts (not a wrapper script's derived text diff).
- **Verified live, both directions**: `takeoff` (previously ~90-91 lines) down to a clean handful; `help`'s `SET_MESSAGE_INTERVAL` burst still correctly shows all 7 real results (6 accepted + 1 failed) - proving the fix neither under- nor over-suppresses.

### 25. Root-caused why arm and Motor Actuators telemetry never worked all session - two real Pixhawk config bugs, not a GCS bug
With a real battery connected and props confirmed off (bench-safe), a systematic diagnostic chain found the actual reasons arm kept failing and the Motor Actuators tab stayed frozen - none of it was in the GCS application code, which was correct throughout.
1. **`EKF2_EV_CTRL` had reverted to `0`** (vision fusion disabled), with `EKF2_HGT_REF=1` and `EKF2_GPS_CTRL=7` - all different from this project's own documented working config (`15`/`3`/`0`). Found via `SYS_STATUS`'s sensor-health bitmask (`present & enabled & ~health`, decoding bit 14 = `XY_POSITION_CONTROL` unhealthy) and `ESTIMATOR_STATUS.flags` (bits for `POS_HORIZ_REL`/`VELOCITY_HORIZ` both false). Likely cause: parameter changes from an earlier session were never saved to flash, and a power cycle during today's UART migration work reverted them to something close to factory defaults.
   - A candidate fix source, `Drone_1.5.params`, was checked *before* blindly applying it - turned out to contain the exact same broken values (`EV_CTRL=0`, `HGT_REF=2`), meaning that file is a stale/factory-baseline export, not the real target config. Flagged to the operator instead of applying it, then used the values documented in this file's own dev log #7 instead.
2. **PX4 gates arm eligibility by target flight mode, not just raw sensor health.** Arming while in the default `LOITER` mode kept failing generically ("Resolve system health failures first") because `LOITER` requires *global* position, which will never exist here (no GPS). Switching to `OFFBOARD` first (needs only *local* position, which fix #1 restored) exposed a sharper, different rejection: `Preflight Fail: heading estimate not stable`.
3. **Heading stability needs an actual physical yaw rotation, confirmed as a repeatable requirement, not a one-time calibration.** Translating the camera alone (for position lock) never stabilized heading; a deliberate slow yaw sweep (~30-45° each way, ~5s) did, twice, in two independent test sessions - but the effect didn't persist: after the camera sat idle and tracking re-locked, the *same* "heading estimate not stable" rejection came back, requiring the rotation to be redone. This is the vision-based equivalent of a magnetometer figure-8 calibration, and needs repeating per session rather than being fixed once.
4. **`PWM_MAIN_FUNC1` through `FUNC4` were all `0` (Disabled)** - the actual reason `SERVO_OUTPUT_RAW` and the Motor Actuators tab never showed real data, even across a fully successful ~26-second sustained armed `AUTO.TAKEOFF`. `CA_ROTOR_COUNT=4` was correctly set (control allocation knows it's a quad), but no physical output channel had ever been told to drive a motor - consistent with this project never having had real ESCs/motors wired to this Pixhawk. Confirmed live with a proper IEEE-754 int32 bitcast decode (a naive float read of an int-type param returns nonsense like `1.4e-45` for the integer value `1`), matching the same `0`s in the static `Drone_1.5.params` export. Fixed with `PWM_MAIN_FUNC1..4 = 101..104` (PX4's standard Control Allocation motor-function IDs) - the documented correct config, not a workaround.
- **Final live verification, all four fixes together**: armed for real, held ~26 seconds through `OFFBOARD` → `takeoff 1.0`, and the Motor Actuators tab showed genuinely live, changing PWM data for the first time all session - one motor saturating at 1900µs and holding, another actively swinging 1100-1800µs, matching PX4's real attitude controller fighting to stabilize a hand-held, prop-less airframe. Disarmed cleanly afterward and independently reconfirmed via a fresh connection.
- **Diagnostic techniques worth reusing**: AND the three `SYS_STATUS` sensor bitmasks together to isolate exactly which sensor axis is unhealthy rather than trusting the generic PX4 rejection text; read `ESTIMATOR_STATUS.flags` for EKF2's own internal per-axis confidence, which is more granular than `SYS_STATUS`; capture the *full* `STATUSTEXT` chain around an arm attempt since PX4 splits long messages across multiple packets (a single line is often truncated); always decode `PARAM_VALUE` via its real `param_type`, since reading an int32 param as a plain float silently produces nonsense denormalized values instead of an error.

### 26. Third IMU (Accel 2) genuinely mismatched, not a software issue - found by comparing raw IMU data directly
- **Symptom**: even after the EKF2 and PWM-mixer fixes (dev log #25), arm reliability stayed inconsistent - health checks flickered between clean and blocked across sessions in ways that didn't match any previously-found cause.
- **Diagnosis**: read `HIGHRES_IMU` (primary), `SCALED_IMU2`, and `SCALED_IMU3` simultaneously and directly compared their gravity-vector readings (converted to matching units, m/s²). Primary vs IMU1 agreed within ~0.11 m/s². **Primary vs IMU2 differed by ~1.1 m/s² on the horizontal axes** - well past PX4's own `COM_ARM_IMU_ACC=0.7` m/s² tolerance for arming. Checked `VIBRATION` telemetry too (all three axes near-zero, zero clipping counters on any IMU) specifically to rule out "this is just vibration/motion noise" before concluding it's a genuine sensor disagreement.
- **Independently confirmed via a live calibration attempt**: triggered `MAV_CMD_PREFLIGHT_CALIBRATION` (accelerometer) - it correctly auto-detected the first ("down") orientation and started, but the sequence didn't progress cleanly through further orientations and instead started repeating `Preflight Fail: Accel 2 inconsistent - check cal` alongside `High Accelerometer Bias` and `Attitude failure (roll)` - naming the exact same unit found by the raw comparison, from a completely independent code path (PX4's own calibration routine, not our diagnostic math).
- **Fix, chosen deliberately over the easier option**: `CAL_ACC2_PRIO = 0` (Disabled) - excludes IMU2 from the estimator's sensor selection entirely, rather than raising `COM_ARM_IMU_ACC` (which would just widen the tolerance and hide a real physical disagreement between two sensors instead of removing the faulty one from service). The remaining two IMUs (primary + IMU1) stay fully cross-checked against each other.
- **Required a full FC reboot** - sensor priority parameters are read once at boot by the estimator selector; a live `PARAM_SET` alone doesn't hot-reload which IMUs are in service. Sent `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` (only while disarmed), and confirmed the reboot was low-friction now that the link is a real UART wire, not USB - `mavlink-router` and `/dev/pixhawk` recovered on their own with zero re-enumeration issues, unlike the USB-era problems documented in dev log #16/#18.
- **Verified live after reboot**: `CAL_ACC2_PRIO=0` persisted correctly, `SYS_STATUS` showed `unhealthy=0x00000000` (fully clean), and a genuine `arm` succeeded from `OFFBOARD` mode with a real flight log created - confirming this accelerometer mismatch, not the earlier EKF2/heading/mode-gating issues, was the last remaining source of session-to-session arm flakiness.
- **A physical-reality correction from the operator, worth keeping**: throughout today's testing, instructions to "move/rotate the camera" for VIO tracking and heading stability were technically imprecise - the D435i is rigidly mounted to the drone frame, so "move the camera" and "move the whole drone" are the exact same physical action. The substance of every test was correct (real motion is real motion regardless of what it's called), but the terminology should be "move/rotate the drone airframe" going forward, since that's what's actually happening and avoids confusion once there's no separate handheld camera to point at.
- **Not yet done**: physically inspecting IMU2's mounting/damper on the Pixhawk 6X to see if there's a visible cause (each of the 3 IMUs is on its own vibration isolation mount) - not urgent, since excluding it from service is a complete workaround either way, but worth knowing the root physical cause eventually.

### 27. Clarified SSH/display architecture for a Radxa mounted on the drone (no HDMI) + `camera.sh` one-command launcher
- **The actual architecture, clarified rather than assumed**: the Radxa side has never needed a display for anything in this pipeline - every node (camera, SLAM, `mavlink-router`, video streamer) is headless, and every action taken on it all project has been over plain SSH. The GCS app is what needs a real display, and that's the **laptop's own physical screen** - normal operation there is just opening a terminal on the laptop directly and running `python3 drone_gcs.py`, no SSH or X11 forwarding needed at all. All the `XCB`/`XAUTHORITY` SSH-launch work done throughout this session existed only because the assistant is a remote agent without physical access to the operator's laptop, for testing/demo purposes - not something the operator needs to replicate for real use.
- **The one real gotcha for a truly headless Radxa**: `drone_rtabmap_all.launch.py` defaults to `launch_rviz:=true`, which tries to open an RViz window *on the Radxa itself* - fine on a bench with a monitor attached, but will fail or hang once mounted on the drone with none. Not a loss of capability though - the GCS app's own embedded RViz widget (`rviz_embed_widget.py`) already provides the same 3D view, rendered on the laptop instead.
- **New file**: [camera.sh](file:///home/radxa/Flop/camera.sh) at the repo root - a one-command wrapper (sources ROS 2 Jazzy + the workspace, then launches the full pipeline) with `launch_rviz:=false` baked in as the default, still overridable via `./camera.sh launch_rviz:=true`. Requested by name specifically, even though a near-identical script (`run_drone_slam.sh`) already existed - kept as a separate, explicitly-named convenience file rather than silently reusing or renaming the existing one.
- **Verified live**: ran it for real - full pipeline came up cleanly, and confirmed via `ps` that no `rviz2` process was ever spawned.
- **Discussed but not yet built**: fully hands-off operation (drone powers on, pipeline is already running with zero manual steps) would need `camera.sh` wrapped in a systemd unit that auto-starts on boot, the same pattern `mavlink-router` already uses. Flagged as the natural next step, not yet implemented.

### Known, unaddressed loose ends (for future reference)
- `drone_rtabmap_all.launch.py` declares `min_obstacle_height` / `max_obstacle_height` /
  `cell_size` as launch arguments, but they are **not actually wired** to the
  corresponding hardcoded values in `launch/rtabmap_slam.launch.py` — passing them on the
  command line currently has no effect. Not fixed as part of this session; flagged here
  for whoever tackles it next.
- No real flight (motors/battery/airframe) has occurred — every fix and test above was
  validated on the bench only, with the Pixhawk 6X not mounted to a powered airframe.
- `DroneBridge5`'s AP isolation was worked around (wired LAN port), not fixed at the
  source — the AP itself likely still blocks WiFi-to-WiFi traffic between the Radxa and
  laptop. Needs admin access to that unit to actually disable isolation (see dev log #18).
- The wired-Ethernet workaround for `DroneBridge5` silently takes over the Radxa's
  default route (no internet on that AP), which isn't fixed with an explicit route
  metric override yet — see dev log #18.
- Whether a WiFi interface physically cycling down/up during a network switch causes a
  brief local DDS discovery hiccup (even for same-machine topics like `/map`) was
  flagged as a real possibility but never actually tested live.
- `HTIC_RND` currently shows 50-260 ms C2 round-trip latency (matching a plain `ping`
  RTT of 137-289 ms) - functionally fine today, but not investigated further (channel
  congestion? power-saving on the Radxa's WiFi adapter? router load?) since it's still
  well inside every real-time deadline this pipeline has. Worth a closer look if it
  gets worse. See dev log #22.
- Couldn't inspect `mavlink-router`'s own internal log during the C2 validation in dev
  log #22 (no `sudo` in-session) - conclusions there rest on application-level ACK
  rates, not router-internal packet accounting.
- Enabling UART6 (dev log #23) permanently disables I²C bus 6 on the Radxa (they're
  the same physical hardware block) - nothing was found actively using that bus at
  the time, but it wasn't exhaustively scanned. If a future peripheral needs I²C6
  specifically, it will conflict with this UART connection and one will have to move.
- The UART6 link runs at 115200 baud (dev log #23), noticeably lower throughput than
  the old USB link's effective rate (68.7 Hz / ~4.9 KB/s vs. USB's 197 Hz / ~11.7
  KB/s telemetry). Not a problem for anything currently in this pipeline, but worth
  knowing if a future feature needs higher-rate telemetry over this same link -
  raising `SER_TELx_BAUD` on the Pixhawk (and matching `Baud=` in the config) is the
  fix, not yet done since 115200 already covers everything currently in use.
- The EKF2 (dev log #25) and PWM_MAIN_FUNC (dev log #25) parameter fixes were set via
  live `PARAM_SET` but were **not explicitly saved to flash** - the same failure mode
  that caused the EKF2 regression in the first place (dev log #25 item 1). A future
  power cycle will likely revert both fixes unless they're saved (PX4's
  `MAV_CMD_PREFLIGHT_STORAGE` / a parameter save action) before then.
- Heading-estimate stability (dev log #25 item 3) does not persist across a tracking
  reset - it needs a fresh yaw rotation motion every time tracking is re-acquired
  after sitting idle, not just once per boot. Not yet automated or documented as a
  standard pre-arm checklist step for an operator to follow.
- `Drone_1.5.params` (the static export at the repo root) is confirmed stale for at
  least `EKF2_EV_CTRL`/`EKF2_HGT_REF`/`EKF2_GPS_CTRL`/`PWM_MAIN_FUNC1-4` - it reflects
  a factory-baseline snapshot, not the actual working configuration. Worth
  re-exporting a fresh, correct snapshot now that real fixes are in place (assuming
  they get saved to flash per the point above), so a future session has an accurate
  reference instead of a misleading one. Also now missing `CAL_ACC2_PRIO=0` (dev log
  #26), so it's drifted further from reality since being written.
- IMU2's physical mounting/damper on the Pixhawk 6X hasn't been visually inspected
  (dev log #26) - the software workaround (`CAL_ACC2_PRIO=0`) is complete on its own,
  but the root physical cause of the ~1.1 m/s² mismatch is still unconfirmed.
- No systemd auto-start unit exists yet for the main pipeline (dev log #27) -
  `camera.sh` still needs to be run manually over SSH after each Radxa boot. For a
  drone with no display/keyboard, wrapping it in a boot-time service (mirroring how
  `mavlink-router` already auto-starts) is the natural next step, not yet built.
