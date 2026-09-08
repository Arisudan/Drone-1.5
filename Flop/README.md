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
| `/wall_boundaries` | `visualization_msgs/MarkerArray` | Vectorized safe flight polygon boundaries (0.6m interior wall offset) |
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
- Starts: camera, stereo odometry, 2.5cm RTAB-Map SLAM, PX4 vision bridge, map thinning node, and wall boundary extraction (obstacle detection decoupled for maximum mapping fidelity).

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

### Known, unaddressed loose ends (for future reference)
- `drone_rtabmap_all.launch.py` declares `min_obstacle_height` / `max_obstacle_height` /
  `cell_size` as launch arguments, but they are **not actually wired** to the
  corresponding hardcoded values in `launch/rtabmap_slam.launch.py` — passing them on the
  command line currently has no effect. Not fixed as part of this session; flagged here
  for whoever tackles it next.
- No real flight (motors/battery/airframe) has occurred — every fix and test above was
  validated on the bench only, with the Pixhawk 6X not mounted to a powered airframe.
