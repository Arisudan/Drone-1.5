# help.md — File-by-File Guide to This Repo

A map of every real file in this project (`rtabmap_drone_pkg`), in the same order as the
actual folder tree, so a new reader can find "what does X do" in one pass instead of
opening every file. Build artifacts (`.venv/`, `__pycache__/`, `build/`, `install/`,
`log/`) are excluded since they're generated, not source.

For the *why* behind design decisions, real-hardware numbers, and the full troubleshooting
history, see [`gotalldone.md`](gotalldone.md) and [`Progress.md`](Progress.md) instead —
this file is purely a navigation aid.

## Start here

If you only open two files in this whole repo, make it these:

- **[`launch/drone_rtabmap_all.launch.py`](#launch)** — the one command that brings up the entire onboard pipeline (camera, SLAM, video streamer, everything) on the Radxa.
- **[`scripts/gcs/drone_gcs.py`](#scriptsgcs-top-level)** (launched via `run_drone_gcs.sh`) — the actual Ground Control Station app you run on the laptop.

Everything else in this guide is reference material for once those two are running.

## Table of contents

- [Folder tree](#folder-tree)
- [What each file does](#what-each-file-does)
  - [`config/`](#config)
  - [`docs/`](#docs)
  - [`launch/`](#launch)
  - [`scratch/`](#scratch)
  - [`scripts/diagnostics/`](#scriptsdiagnostics)
  - [`scripts/gcs/core/`](#scriptsgcscore)
  - [`scripts/gcs/protocol/`](#scriptsgcsprotocol)
  - [`scripts/gcs/ui/`](#scriptsgcsui)
  - [`scripts/gcs/` (top level)](#scriptsgcs-top-level)
  - [`scripts/network/`](#scriptsnetwork)
  - [`scripts/` (top level)](#scripts-top-level)
  - [`tests/`](#tests)
  - [Root](#root)

## Folder tree

```
.
├── .github
│   └── workflows
│       └── tests.yml
├── config
│   ├── mavlink-router.conf
│   └── rtabmap_drone.rviz
├── docs
│   └── slam_evaluation.md
├── launch
│   ├── d435i_stereo_imu.launch.py
│   ├── drone_rtabmap_all.launch.py
│   ├── rtabmap_slam.launch.py
│   └── stereo_inertial_odom.launch.py
├── scratch
│   ├── test_dual_map_stream.py
│   └── test_gcs_dual_map_screenshot.py
├── scripts
│   ├── diagnostics
│   │   ├── apply_ekf2_params.py
│   │   ├── bench_check.py
│   │   ├── c2_validate.py
│   │   ├── drone_gcs_gui.py
│   │   ├── duplex_check.py
│   │   ├── live_status.py
│   │   ├── map_eval.py
│   │   ├── map_recorder.py
│   │   ├── px4_control.py
│   │   ├── slam_health_monitor.py
│   │   ├── test_udp_drone_control.py
│   │   └── verify_ekf2_params.py
│   ├── gcs
│   │   ├── core
│   │   │   ├── execution_tracker.py
│   │   │   ├── health.py
│   │   │   ├── __init__.py
│   │   │   ├── path_planner.py
│   │   │   └── telemetry.py
│   │   ├── protocol
│   │   │   ├── __init__.py
│   │   │   ├── mavlink_worker.py
│   │   │   └── ros2_map_listener.py
│   │   ├── ui
│   │   │   ├── cli_console.py
│   │   │   ├── hud_widget.py
│   │   │   ├── __init__.py
│   │   │   ├── motor_widget.py
│   │   │   ├── rviz_embed_widget.py
│   │   │   ├── sidebar_nav.py
│   │   │   ├── slam_map_widget.py
│   │   │   ├── styles.py
│   │   │   ├── toast.py
│   │   │   ├── top_status_strip.py
│   │   │   └── video_feed_widget.py
│   │   ├── drone_gcs.py
│   │   ├── radxa_monitor.py
│   │   └── run_drone_gcs.sh
│   ├── network
│   │   └── udp_mavlink_bridge.py
│   ├── d435i_video_streamer.py
│   ├── map_thinning_node.py
│   ├── obstacle_distance_bridge.py
│   ├── px4_vision_bridge.py
│   ├── tcp_map_streamer_node.py
│   └── wall_boundary_node.py
├── tests
│   ├── _env.py
│   ├── run_tests.sh
│   ├── test_health.py
│   ├── test_map_eval.py
│   ├── test_path_planner.py
│   ├── test_smoke.py
│   └── test_telemetry.py
├── CMakeLists.txt
├── Drone_1.5.params
├── GetItCorrect.md
├── gotalldone.md
├── mav.tlog
├── mav.tlog.raw
├── package.xml
├── Progress.md
├── README.md
├── requirements.txt
└── run_drone_slam.sh
```

## What each file does

### `config/`
- **mavlink-router.conf** — mavlink-router daemon config: TCP server on 5760, UART to the Pixhawk, a UDP endpoint for the GCS laptop (14550), and a local UDP endpoint (14541) for the vision/obstacle bridges.
- **rtabmap_drone.rviz** — Saved RViz2 layout used by `rviz_embed_widget.py` for the embedded 3D SLAM/point-cloud view.

### `docs/`
- **slam_evaluation.md** — The map-quality acceptance protocol: what to measure on the 2.5 cm occupancy grid, how to capture a run, and the numeric pass/fail threshold for each metric. Read this before arguing about whether a map is "good".

### `launch/`
- **d435i_stereo_imu.launch.py** — Starts the RealSense D435i driver (stereo IR + IMU + RGB color) with this project's exact profiles/topics.
- **drone_rtabmap_all.launch.py** — The master launch file: brings up the whole pipeline in one command (camera, stereo odometry, RTAB-Map, video streamer, PX4 vision bridge, map thinning, wall boundary, TCP map streamer).
- **rtabmap_slam.launch.py** — Configures and starts the RTAB-Map SLAM node (grid resolution, obstacle height band, wall-locking/segmentation parameters).
- **stereo_inertial_odom.launch.py** — Starts RTAB-Map's `stereo_odometry` node (visual-inertial odometry from the stereo IR + IMU streams).

### `scratch/`
- **test_dual_map_stream.py** — Standalone headless unit test for the dual-layer (`/map` raw + `/map_thin` skeleton) GCS canvas overlay logic.
- **test_gcs_dual_map_screenshot.py** — Renders the GCS Tactical SLAM tab offscreen with synthetic map data and saves it as an image, for verifying UI layout changes without needing real hardware running.

### `scripts/diagnostics/`
- **apply_ekf2_params.py** — CLI tool to safely set and verify individual Pixhawk parameters over MAVLink, with mandatory readback confirmation.
- **bench_check.py** — Publishes the flight controller's raw IMU/attitude/body-rate data straight to ROS 2 topics, unconverted, for bench-testing sensor health.
- **c2_validate.py** — Pre-flight ground test confirming the GCS→FC command/ack uplink is alive (mode changes, arm/disarm, takeoff probe) — no real flight needed.
- **drone_gcs_gui.py** — Earlier standalone Tkinter GCS prototype (dual send/receive mode + CLI); superseded by the PyQt5 app in `scripts/gcs/` but kept as a diagnostics fallback.
- **duplex_check.py** — Verifies uplink (commands reaching the FC) and downlink (telemetry reaching the GCS) independently, without arming anything.
- **map_eval.py** — Offline scorer for a recorded SLAM run: scale error, wall straightness, squareness, coverage, loop-closure drift, yaw drift and VIO health, each against its threshold, exiting non-zero on failure. Pure numpy — no ROS 2, no Qt. See [`docs/slam_evaluation.md`](docs/slam_evaluation.md).
- **map_recorder.py** — Read-only ROS 2 node that captures one run (`/map`, `/map_thin`, `/odom`) into a self-describing artifact folder for `map_eval.py`. Safe to run during a real flight; it publishes and commands nothing.
- **live_status.py** — Console-only live telemetry dashboard (position, attitude, battery, motor PWMs) with closed-loop execution verification — no GUI needed.
- **px4_control.py** — Interactive PX4 flight REPL/CLI (arm, takeoff, move, yaw, goto, mission patterns); also usable as a scripted command runner.
- **slam_health_monitor.py** — Live diagnostic table for SLAM/VIO tracking health (inlier counts, match ratio, IMU jitter) to pinpoint why tracking is failing.
- **test_udp_drone_control.py** — Interactive/scripted flight control + telemetry validator specifically over the UDP transport (14550/5760).
- **verify_ekf2_params.py** — Read-only audit of the Pixhawk's EKF2 sensor-fusion parameters (vision/GPS/optical-flow/height-reference config) — safe to run any time.

### `scripts/gcs/core/`
- **health.py** — Liveness and latency bookkeeping for background workers: heartbeats, measured rate, p95 latency, and stall detection against a declared deadline. Wired into the OFFBOARD setpoint pump (whose 500 ms limit is PX4's, not a UI preference) and the map listener, so a dead worker surfaces instead of silently freezing the last good value on screen.
- **execution_tracker.py** — Confirms a dispatched flight command was actually physically executed (not just ACKed) by watching real position displacement.
- **\_\_init\_\_.py** — Marks `core` as a Python package (empty).
- **path_planner.py** — Obstacle-aware A* path planner over the live occupancy grid with line-of-sight smoothing, used for click-to-navigate waypoints.
- **telemetry.py** — Central typed telemetry data model (`TelemetryState`) that decodes raw MAVLink messages into the single source of truth every GCS widget reads from.

### `scripts/gcs/protocol/`
- **\_\_init\_\_.py** — Marks `protocol` as a Python package (empty).
- **mavlink_worker.py** — Background `QThread` that owns the live MAVLink connection to the Pixhawk and streams decoded telemetry to the UI via Qt signals.
- **ros2_map_listener.py** — Receives the SLAM occupancy grid over ROS 2 or a raw TCP fallback, and drives the SLAM map reset feature.

### `scripts/gcs/ui/`
- **cli_console.py** — Embedded terminal widget (command history, live scrolling log) for typing flight commands directly inside the GCS.
- **hud_widget.py** — Cockpit PFD tab: speed/altitude tapes plus the mode/armed status banner.
- **\_\_init\_\_.py** — Marks `ui` as a Python package (empty).
- **motor_widget.py** — Live per-motor PWM gauge bars with color-coded saturation/idle/nominal thresholds.
- **rviz_embed_widget.py** — Embeds a real RViz2 3D window directly inside the GCS app (X11 window swallowing) for full point-cloud/SLAM 3D viewing.
- **sidebar_nav.py** — Left-hand vertical navigation rail that switches between GCS workspace tabs (Cockpit, SLAM, Video, Motors, Diagnostics, Terminal).
- **slam_map_widget.py** — The 2D Tactical SLAM tab: renders the occupancy grid, drone position/heading, planned path, and the Reset Map button.
- **styles.py** — Central dark aviation-themed Qt stylesheet (QSS) shared by the entire GCS app.
- **toast.py** — Small transient notification popup shown in the header after actions (connect, reset, etc.).
- **top_status_strip.py** — The header bar: network dropdown/connect controls, mode/arm badges, battery/VIO/EKF2 badges, RX/TX throughput, and VIO NED readout.
- **video_feed_widget.py** — FPV Camera tab: connects to the onboard MJPEG stream and displays live D435i video with auto-reconnect.

### `scripts/gcs/` (top level)
- **drone_gcs.py** — The main GCS application window; wires every widget/worker above together into the actual running app.
- **radxa_monitor.py** — A separate, passive-only onboard monitoring GUI (zero flight controls) meant to run on the Radxa itself or a field display.
- **run_drone_gcs.sh** — Launch script for the laptop GCS app (sources ROS 2, forces the XCB Qt backend, runs `drone_gcs.py`).

### `scripts/network/`
- **udp_mavlink_bridge.py** — Standalone UDP↔TCP MAVLink relay bridging the laptop's Wi-Fi UDP link to mavlink-router's local TCP port.

### `scripts/` (top level)
- **d435i_video_streamer.py** — Onboard node that JPEG-compresses the D435i color stream and serves it as an MJPEG HTTP stream to the GCS.
- **map_thinning_node.py** — Post-processes RTAB-Map's raw occupancy grid: purges noise blobs and skeletonizes walls to a single-pixel outline (`/map_thin`).
- **obstacle_distance_bridge.py** — Raycasts the occupancy grid into a 72-sector MAVLink `OBSTACLE_DISTANCE` ring for PX4's avoidance/failsafe layer (currently decoupled from the main launch — see `gotalldone.md` dev log #17).
- **px4_vision_bridge.py** — Converts RTAB-Map's `/odom` pose into MAVLink `VISION_POSITION_ESTIMATE` packets and feeds them to the Pixhawk's EKF2.
- **tcp_map_streamer_node.py** — Streams the occupancy grid map (and handles the SLAM map reset command) over a raw TCP socket, as a fallback when native ROS 2 discovery over Wi-Fi is unreliable.
- **wall_boundary_node.py** — Extracts vectorized wall/room-boundary polygons from the occupancy grid for flight-safety visualization.

### `tests/`
- **\_env.py** — Import-path and headless-Qt setup; every test module imports it first.
- **run_tests.sh** — Runs the suite the way CI does (`./run_tests.sh hermetic` for the numpy-only subset).
- **test_health.py** — Stall latching, recovery, rate/p95, registry and watchdog behaviour.
- **test_map_eval.py** — Every SLAM metric against synthetic grids, plus the CLI's exit codes.
- **test_path_planner.py** — A* over synthetic occupancy grids: doorways, sealed walls, inflation clearance, unknown-space policy, and the regression that a goal clicked inside a wall never becomes the final waypoint.
- **test_smoke.py** — Import-level checks for the PyQt5 modules (skipped where PyQt5 is absent).
- **test_telemetry.py** — PX4 mode decoding, the `landed_state` × `is_airborne` matrix behind the disarm-vs-land interlock, RC health from `SYS_STATUS`, and vision staleness.

### Root
- **.github/workflows/tests.yml** — CI: a hermetic numpy-only job, a PyQt5 offscreen smoke job, and a pyflakes lint gate.
- **CMakeLists.txt** — ROS 2 `ament_cmake` build file; lists every script that gets installed as an executable.
- **Drone_1.5.params** — Exported Pixhawk parameter dump (full onboard parameter table) for this specific vehicle.
- **GetItCorrect.md** — Short one-page summary of the system's sensor → SLAM → EKF2 data flow.
- **gotalldone.md** — Full historical runbook, Q&A troubleshooting log, and an archived full snapshot of an earlier README.
- **mav.tlog** / **mav.tlog.raw** — Raw MAVLink telemetry log recordings (binary flight-log dumps, not source code).
- **package.xml** — ROS 2 package manifest (name, maintainer, build type).
- **Progress.md** — Running development log of milestones, real-hardware findings, and what was fixed and why.
- **README.md** — Current short project summary (points to `gotalldone.md` for full history).
- **requirements.txt** — Python pip dependencies for the whole pipeline + GCS app.
- **run_drone_slam.sh** — Launch script for the onboard SLAM pipeline (sources ROS 2 + workspace, runs `drone_rtabmap_all.launch.py`).
