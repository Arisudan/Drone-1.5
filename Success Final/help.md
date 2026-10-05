# help.md — File-by-File Guide to This Repo

A map of every real file in this project (`rtabmap_drone_pkg`), in the same order as the
actual folder tree, so a new reader can find "what does X do" in one pass instead of
opening every file. Build artifacts (`.venv/`, `__pycache__/`, `build/`, `install/`,
`log/`) are excluded since they're generated, not source.

For how to install/run the pipeline, see [`setup.md`](setup.md). For how to use the GCS
application, see [`guide.md`](guide.md). For the *why* behind design decisions,
real-hardware numbers, and the full troubleshooting history, see [`Progress.md`](Progress.md)
instead — this file is purely a navigation aid.

## Start here

If you only open two files in this whole repo, make it these:

- **[`launch/drone_rtabmap_all.launch.py`](#launch)** — the one command that brings up the entire onboard pipeline (camera, SLAM, video streamer, everything) on the Radxa.
- **[`scripts/gcs/drone_gcs.py`](#scriptsgcs-top-level)** (launched via `run_drone_gcs.sh`) — the actual Ground Control Station app you run on the laptop.

Everything else in this guide is reference material for once those two are running.

## Table of contents

- [Folder tree](#folder-tree)
- [What each file does](#what-each-file-does)
  - [`assets/fonts/`](#assetsfonts)
  - [`config/`](#config)
  - [`docs/`](#docs)
  - [`launch/`](#launch)
  - [`scratch/`](#scratch)
  - [`scripts/diagnostics/`](#scriptsdiagnostics)
  - [`scripts/gcs/core/`](#scriptsgcscore)
  - [`scripts/gcs/protocol/`](#scriptsgcsprotocol)
  - [`scripts/gcs/controllers/`](#scriptsgcscontrollers)
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
├── assets
│   └── fonts
│       ├── RedHatDisplay-Black.ttf
│       ├── RedHatDisplay-Bold.ttf
│       ├── RedHatDisplay-ExtraBold.ttf
│       ├── RedHatDisplay-OFL.txt
│       ├── RedHatDisplay-Regular.ttf
│       ├── Ubuntu-Bold.ttf
│       ├── Ubuntu-LICENCE.txt
│       ├── Ubuntu-Medium.ttf
│       └── Ubuntu-Regular.ttf
├── config
│   ├── mavlink-router.conf
│   └── rtabmap_drone.rviz
├── docs
│   ├── images
│   │   ├── motor_frame_heading_arrow.png
│   │   ├── params_tab_phase1.png
│   │   └── slam_drone_icon_redesign.png
│   ├── link_range_test.md
│   ├── slam_evaluation.md
│   └── threading.md
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
│   │   ├── fix_accel2_bias.py
│   │   ├── link_probe_server.py
│   │   ├── link_range_gui.py
│   │   ├── link_range_test.py
│   │   ├── link_range_walk.py
│   │   ├── live_status.py
│   │   ├── map_eval.py
│   │   ├── map_recorder.py
│   │   ├── px4_control.py
│   │   ├── slam_health_monitor.py
│   │   ├── test_udp_drone_control.py
│   │   └── verify_ekf2_params.py
│   ├── gcs
│   │   ├── controllers
│   │   │   ├── alarm_control.py
│   │   │   ├── flight_commands.py
│   │   │   ├── __init__.py
│   │   │   └── mission_control.py
│   │   ├── core
│   │   │   ├── alarms.py
│   │   │   ├── audio.py
│   │   │   ├── execution_tracker.py
│   │   │   ├── flight_log.py
│   │   │   ├── health.py
│   │   │   ├── __init__.py
│   │   │   ├── log_bundle.py
│   │   │   ├── map_quality.py
│   │   │   ├── map_render.py
│   │   │   ├── motor_range.py
│   │   │   ├── param_codec.py
│   │   │   ├── path_planner.py
│   │   │   ├── planner_worker.py
│   │   │   ├── settings.py
│   │   │   ├── telemetry.py
│   │   │   ├── ui_stall.py
│   │   │   ├── video_health.py
│   │   │   └── video_recorder.py
│   │   ├── protocol
│   │   │   ├── __init__.py
│   │   │   ├── mavlink_worker.py
│   │   │   └── ros2_map_listener.py
│   │   ├── ui
│   │   │   ├── assets
│   │   │   │   ├── chevron_down.png
│   │   │   │   ├── chevron_down_hover.png
│   │   │   │   └── chevron_down_off.png
│   │   │   ├── actuator_widget.py
│   │   │   ├── alarm_banner.py
│   │   │   ├── battery_badge.py
│   │   │   ├── cli_console.py
│   │   │   ├── config_tab.py
│   │   │   ├── flight_detail.py
│   │   │   ├── fonts.py
│   │   │   ├── guided_confirm.py
│   │   │   ├── hud_widget.py
│   │   │   ├── __init__.py
│   │   │   ├── logs_tab.py
│   │   │   ├── map_canvas_render.py
│   │   │   ├── mini_feed.py
│   │   │   ├── mission_progress.py
│   │   │   ├── motor_widget.py
│   │   │   ├── params_tab.py
│   │   │   ├── rviz_embed_widget.py
│   │   │   ├── scaling.py
│   │   │   ├── shortcuts.py
│   │   │   ├── sidebar_nav.py
│   │   │   ├── slam_map_widget.py
│   │   │   ├── styles.py
│   │   │   ├── toast.py
│   │   │   ├── top_status_strip.py
│   │   │   ├── value_grid.py
│   │   │   └── video_feed_widget.py
│   │   ├── drone_gcs.py
│   │   ├── qt_env.py
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
│   ├── rviz_palettes_reference.txt
│   ├── test_actuator_button.py
│   ├── test_alarms_video.py
│   ├── test_audio.py
│   ├── test_command_ack.py
│   ├── test_diagnostics_redesign.py
│   ├── test_drone_glyph_zoom.py
│   ├── test_fonts.py
│   ├── test_gui_features.py
│   ├── test_gui_layout.py
│   ├── test_health.py
│   ├── test_infrastructure.py
│   ├── test_keepout_mission.py
│   ├── test_layout_density.py
│   ├── test_link_range.py
│   ├── test_link_range_gui.py
│   ├── test_link_range_walk.py
│   ├── test_map_eval.py
│   ├── test_mini_feed.py
│   ├── test_flight_logs.py
│   ├── test_preflight.py
│   ├── test_radxa_services.py
│   ├── test_fpv_tools.py
│   ├── test_config_tab.py
│   ├── test_slam_panel.py
│   ├── test_motor_response.py
│   ├── test_motor_test.py
│   ├── test_param_codec.py
│   ├── test_params_tab.py
│   ├── test_path_planner.py
│   ├── test_planner_worker.py
│   ├── test_position_uncertainty.py
│   ├── test_rviz_palette.py
│   ├── test_scaling.py
│   ├── test_smoke.py
│   ├── test_statustext.py
│   ├── test_telemetry.py
│   └── test_video_sources.py
├── .txt
├── camera.sh
├── CMakeLists.txt
├── Drone_1.5.params
├── guide.md
├── help.md
├── mav.tlog
├── mav.tlog.raw
├── package.xml
├── Progress.md
├── README.md
├── requirements.txt
├── run_drone_slam.sh
└── setup.md
```

## What each file does

### `assets/fonts/`
- **RedHatDisplay-{Regular,Bold,ExtraBold,Black}.ttf** + **RedHatDisplay-OFL.txt** — the brand font for the `DRONE-GCS` title (Black weight), with its SIL Open Font Licence.
- **Ubuntu-{Regular,Medium,Bold}.ttf** + **Ubuntu-LICENCE.txt** — the heading font (Navigation, Flight Mode, every tab's section/card headings), with its Ubuntu Font Licence.
- Registered with Qt at startup by `scripts/gcs/ui/fonts.py`, so they work on a machine that has neither installed. Static TTFs, not variable fonts (Qt 5 picks weights from a static family reliably).

### `config/`
- **mavlink-router.conf** — mavlink-router daemon config: TCP server on 5760, UART to the Pixhawk, a UDP endpoint for the GCS laptop (14550), and a local UDP endpoint (14541) for the vision/obstacle bridges.
- **rtabmap_drone.rviz** — Saved RViz2 layout used by `rviz_embed_widget.py` for the embedded 3D SLAM/point-cloud view.

### `docs/`
- **images/** — Screenshots referenced from `Progress.md` and `guide.md` (motor frame heading arrow, Parameters tab, SLAM drone icon redesign) — evidence, not source.
- **link_range_test.md** — Reference for the walk-away range test: setup, what is measured, how "loss-free up to N m" and every verdict are defined, how loss is counted from sequence numbers, output files, limits and troubleshooting.
- **threading.md** — The GCS threading contract: which thread owns what, which Qt signals cross to the UI thread, and the rules (workers never touch widgets; tokens not flags; the UI-stall detector).
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
- **fix_accel2_bias.py** — Retires the faulty IMU2 accelerometer and persists that across power cycles (sets the relevant parameters and verifies the readback).
- **link_probe_server.py** — Radxa-side UDP echo helper for the range test's packet-loss probe. Echoes each probe stamped with how many it has received, which separates uplink from downlink loss. Standalone (copied alone to `/tmp` over SSH), touches no MAVLink/camera port, exits when idle.
- **link_range_gui.py** — The window for the range test: setup (asks for the Radxa IP, checks the connection, chooses continuous walk or marked holds) → walking screen (live readouts, one big button per mark, or a per-second stream) → result (the loss-free distance, or the loss-free seconds and dBm, first). Opened by `link_range_test.py` when run with no arguments.
- **link_range_walk.py** — Continuous-walk mode of the range test: a point per second from a sliding 2 s window, the verdict in seconds and dBm (loss-free duration, weakest clean signal, first loss, per-signal-band table), its HTML/Markdown report and the streamed terminal lines. No distance.
- **link_range_test.py** — The walk-away range test engine and entry point: samplers (Wi-Fi via `iw`, ping, the real MJPEG stream, UDP probe), per-distance analysis, verdicts and the loss-free range, path-loss fit, HTML/Markdown/CSV/JSON report, a terminal flow (`--cli`) and a `--demo`. See [`docs/link_range_test.md`](docs/link_range_test.md).
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
- **alarms.py** — The prioritised, acknowledgeable alarm list (pure Python): one entry per active fault, `raise_alarm()` returns True only for a new or escalated alarm, acknowledging silences but never clears, elapsed time is computed on demand (never stored), cleared alarms go to a bounded history.
- **audio.py** — Spoken and tonal operator alerts. A daemon thread and a bounded queue, so a wedged sound device can never block the GUI; tones are synthesised to WAV once and played through `aplay`, speech goes through `spd-say`, and a machine with neither degrades to silence rather than failing. Per-event rate limiting is the point, not an optimisation: PX4 re-runs its preflight checks every ~2 s, and an alert that repeats forever is one the operator mutes.
- **flight_log.py** — Persistent flight recorder: one JSON-lines record per armed session (arm → disarm), written the moment the vehicle disarms, with a ~1 Hz time series and start/min voltage; backs the Logs tab.
- **log_bundle.py** — Builds the one-click diagnostics zip (flight history, settings, `*.log`/`*.tlog`, manifest) behind the Logs tab's Export Bundle button.
- **map_quality.py** — Live SLAM map quality score computed from the grid alone, plus the stale-map check.
- **map_render.py** — Occupancy grid → RGBA image with RViz's exact palettes, built off the GUI thread by the map listener.
- **motor_range.py** — The vehicle's configured motor range as one shared model: `PWM_MAIN_{MIN,MAX,DIS,FUNC}n` → per-motor percentage, idle/nominal/high/saturated state, output→motor mapping, and the throttle→µs map. Falls back to the documented 1000–2000 µs default (and says so) until the parameters arrive. Used by the Motors tab and the Diagnostics tiles.
- **param_codec.py** — Decodes MAVLink `PARAM_VALUE` (PX4's float32 bit-cast convention) so an int32 parameter doesn't read as `1.4e-45`.
- **planner_worker.py** — Runs A\*, path collision checks, map scoring and the inflation layer on a worker thread, with job tokens so a stale result is dropped.
- **settings.py** — Typed, validated, persisted ground-station settings (`$DRONE_GCS_HOME/settings.json`), with a schema version and one-step migrations; CLI/env overrides layered on top.
- **ui_stall.py** — Detects the UI thread blocking: feeds on the 30 Hz tick and reports a gap over 250 ms (rate-limited), with the worst gap kept.
- **preflight.py** — The preflight checklist logic (no Qt): turns link / battery / vision / position / video / map / heading facts into pass, fail or unknown rows; which rows are required; the READY / NOT READY line. Unknown never counts as pass.
- **radxa_status.py** — Fetches and interprets the Radxa watchdog's `/status` JSON (state, restarts, disk). None = not reporting, never healthy.
- **video_recorder.py** — FPV snapshot (PNG) and MJPEG-AVI recorder: real frames only, size-normalised, files in `<DRONE_GCS_HOME>/video/`; no Qt.
- **video_health.py** — Video feed health from frame arrivals: fps, jitter, capture→paint ("pipeline") latency, and freeze / no-signal detection. Placeholder frames never count as live.
- **health.py** — Liveness and latency bookkeeping for background workers: heartbeats, measured rate, p95 latency, and stall detection against a declared deadline. Wired into the OFFBOARD setpoint pump (whose 500 ms limit is PX4's, not a UI preference) and the map listener, so a dead worker surfaces instead of silently freezing the last good value on screen.
- **execution_tracker.py** — Confirms a dispatched flight command was actually physically executed (not just ACKed) by watching real position displacement.
- **\_\_init\_\_.py** — Marks `core` as a Python package (empty).
- **path_planner.py** — Obstacle-aware A* path planner over the live occupancy grid with line-of-sight smoothing, used for click-to-navigate waypoints.
- **telemetry.py** — Central typed telemetry data model (`TelemetryState`) that decodes raw MAVLink messages into the single source of truth every GCS widget reads from.

### `scripts/gcs/protocol/`
- **\_\_init\_\_.py** — Marks `protocol` as a Python package (empty).
- **mavlink_worker.py** — Background `QThread` that owns the live MAVLink connection to the Pixhawk: telemetry decode, command dispatch, the bench actuator test, the Parameters tab's list request, and the targeted `PWM_MAIN_*` reads for the motor scale (emitted on `motor_param_received`, and held back from the Parameters tab unless a bulk list is in flight).
- **ros2_map_listener.py** — Receives the SLAM occupancy grid over ROS 2 or a raw TCP fallback, and drives the SLAM map reset feature.

### `scripts/gcs/controllers/`
What the main window *does*, split out of `drone_gcs.py` as mixins (methods still read window state through `self`).
- **alarm_control.py** — Turns telemetry and video health into the standing alarm list (link, battery, vision and position while airborne, video), and handles Acknowledge.
- **flight_commands.py** — Arm, disarm, takeoff, yaw, change altitude, mode, kill: the operator-facing `_request_*` entry points (via the guided-confirm bar) and the `_cmd_*` executors that talk to the MAVLink worker.
- **mission_control.py** — Execute / pause / resume / abort path, collision verdicts, detours, the OFFBOARD setpoint pump, map reset, and mission progress.

### `scripts/gcs/ui/`
- **actuator_widget.py** — The ESP32 servo button: a status dot and one button (`SERVO 0° → 90°`) driven over async HTTP with a timeout, so an off ESP32 never freezes the GCS. Sits beside EMERGENCY KILL in the cockpit.
- **alarm_banner.py** — The standing alarm card under the header: severity bar, title, hint, running timer, `+N`, Acknowledge. Hidden when empty.
- **battery_badge.py** — The header's battery indicator: a drawn cell with fill level, percentage and voltage.
- **config_tab.py** — The Configuration workspace: section list + search, captions with unit suffixes, per-field modified dots, inline validation, restart-needed tags, per-section reset and Bench / Indoor presets; edits, validates and persists the station settings.
- **preflight_panel.py** — The checklist drawn under the Parameters table: a rule, a READY / NOT READY line, three columns of checks, the manual *Heading fixed* box.
- **preflight_control.py** — (controllers) Gathers the checklist facts once a second, gates the ARM button on it, runs the guarded *Save to flash*, and turns Radxa watchdog reports into alarms.
- **radxa_status_worker.py** — (protocol) Polls the watchdog every 5 s on its own thread.
- **radxa_watchdog.py / radxa_log_janitor.py / install_radxa_services.sh / deploy/*.service** — (scripts/radxa, run ON the Radxa) Watchdog + status endpoint, log cleanup, the installer and the systemd units. See guide.md §11.
- **flight_detail.py** — Flight-detail dialog for the Logs tab (altitude / speed / battery / voltage charts, ground track, CSV export) and the cross-flight battery-health trend charts; QPainter only.
- **fonts.py** — Registers the bundled Red Hat Display / Ubuntu fonts with Qt (idempotent; called from `build_stylesheet()`), and `resolved_family()` for tests.
- **logs_tab.py** — The Flight Logs workspace: statistics, filters, table, Details… (flight detail), Trends panel, CSV export, the password-gated Reset Logs, and Export Bundle.
- **map_canvas_render.py** — The Tactical map canvas's drawing code (grid, keep-outs, mission stops, trail, path, goal, drone glyph, ruler, uncertainty ring, overlay) as a mixin; `slam_map_widget.py` keeps the input and tool state.
- **mini_feed.py** — The small live camera thumbnail in the navigation rail on the tabs without a camera of their own: ~10 fps, click for fullscreen, cleared to `NO VIDEO` when the feed freezes.
- **params_tab.py** — The read-only live PX4 parameter table (search, sortable, batched redraws).
- **cli_console.py** — Embedded terminal widget (command history, live scrolling log) for typing flight commands directly inside the GCS.
- **guided_confirm.py** — The slide-to-confirm bar and its bounded value slider. Every irreversible action (arm, disarm, emergency kill) and every numeric one (takeoff altitude, yaw, change altitude) is gated behind a deliberate horizontal drag rather than a click or a modal whose default button is one Return away. It dispatches nothing itself - the main window still runs the same `_cmd_*` method, interlocks and all.
- **hud_widget.py** — Cockpit PFD tab: speed/altitude tapes plus the mode/armed status banner.
- **\_\_init\_\_.py** — Marks `ui` as a Python package (empty).
- **mission_progress.py** — Route progress strip for an executing A* path: a painted route bar with one pip per waypoint, plus waypoint index, distance to next, distance remaining along the route (not straight-line), ETA and elapsed. Display only. The ETA reads `--` rather than inventing a number when the vehicle is not moving.
- **motor_widget.py** — The actuator workspace: a realistic top-down drone (tapered arms, camera pod, LEDs, spinning propellers, rotation arrows), per-motor gauge bars, and the Bench Motor Test panel (`MAV_CMD_ACTUATOR_TEST`) with its interlock checklist, props-off countdown, 2×2 selector and `ThrottleSlider`. Everything is scaled to the vehicle's own motor range (`core/motor_range.py`), greys out when `SERVO_OUTPUT_RAW` goes stale, and sheds detail on a short page instead of overlapping. The test panel is gated on a live link, a disarmed and grounded vehicle, an expiring props-removed acknowledgement and a throttle ceiling; read the module header before changing any of it.
- **rviz_embed_widget.py** — Embeds a real RViz2 3D window directly inside the GCS app (X11 window swallowing) for full point-cloud/SLAM 3D viewing.
- **scaling.py** — One UI scale factor for the whole station. Rewrites the stylesheet's px dimensions and provides `px()`/`scaled_font()` for code, so a 4K panel or a desktop set to large text gets a readable station instead of the 96 DPI layout everything was authored against. Hairline `border:` widths are deliberately not scaled.
- **shortcuts.py** — The keyboard binding table and the F1 overlay generated from it. No destructive action fires directly from a key: arm, disarm and abort all open the `guided_confirm` bar, and the emergency kill has no binding at all.
- **sidebar_nav.py** — Left-hand vertical navigation rail that switches between GCS workspace tabs, shows the speed/altitude/mode/armed footer (armed green, disarmed red), hosts the camera thumbnail on the tabs without a camera, and tightens its rows when the window is short.
- **slam_map_widget.py** — The 2D Tactical SLAM tab: view and tool bar plus a side panel (map status + Reset Map, layers, route summary / stop list, state-aware EXECUTE / PAUSE-RESUME / ABORT), the grey-first map-quality strip, and the canvas input handling.
- **styles.py** — Central dark aviation-themed Qt stylesheet (QSS) shared by the entire GCS app, including the `font_brand` / `font_heading` tokens.
- **toast.py** — Small transient notification popup shown in the header after actions (connect, reset, etc.).
- **assets/** — Chevron images for the stylesheet's drop-down arrows.
- **top_status_strip.py** — The header bar: network dropdown/connect controls, mode/arm badges, battery/VIO/EKF2 badges, RX/TX throughput, and VIO NED readout.
- **value_grid.py** — The Diagnostics workspace: a glance strip (altitude, speed, battery, link, mode/armed, flight time) over one card per system with a health light, holding the telemetry values the operator chooses, orders and sizes, from a registry of ~50 fields. Grey-first colour (`FieldSpec.signals=False` marks a pure *state* colour such as armed/disarmed), one **Layout ▾** menu, "columns" is a maximum. Field keys are written to settings.json, so add a new key rather than renaming one; a saved layout always wins over the default.
- **video_feed_widget.py** — FPV Camera tab (with Snapshot / Record / Telemetry / Graph tools): connects to the onboard MJPEG stream and displays live D435i video with auto-reconnect, plus a health line (fps / pipeline latency / jitter, `FROZEN` / `NO SIGNAL`) that feeds the alarm card and the rail thumbnail.

### `scripts/gcs/` (top level)
- **drone_gcs.py** — The main GCS application window; wires every widget/worker together into the running app (the commands themselves live in `controllers/`).
- **qt_env.py** — Points Qt at the system platform plugins, but only when the PyQt5 about to be imported actually is the system one (a venv PyQt5 bundles its own).
- **radxa_monitor.py** — A separate, passive-only onboard monitoring GUI (zero flight controls) meant to run on the Radxa itself or a field display.
- **run_drone_gcs.sh** — Launch script for the laptop GCS app (sources ROS 2, forces the XCB Qt backend, runs `drone_gcs.py`).

### `scripts/network/`
- **udp_mavlink_bridge.py** — Standalone UDP↔TCP MAVLink relay bridging the laptop's Wi-Fi UDP link to mavlink-router's local TCP port.

### `scripts/` (top level)
- **d435i_video_streamer.py** — Onboard node that JPEG-compresses the D435i color stream and serves it as an MJPEG HTTP stream to the GCS. Each client socket is tuned for low latency (`TCP_NODELAY`, ~96 KB send buffer, send timeout) and each frame goes out as one buffer, so a Wi-Fi stall backs up into the streamer (which skips to the newest frame) instead of queueing stale frames in the kernel.
- **map_thinning_node.py** — Post-processes RTAB-Map's raw occupancy grid: purges noise blobs and skeletonizes walls to a single-pixel outline (`/map_thin`).
- **obstacle_distance_bridge.py** — Raycasts the occupancy grid into a 72-sector MAVLink `OBSTACLE_DISTANCE` ring for PX4's avoidance/failsafe layer (currently decoupled from the main launch — see `Progress.md`'s Phase 1 entry).
- **px4_vision_bridge.py** — Converts RTAB-Map's `/odom` pose into MAVLink `VISION_POSITION_ESTIMATE` packets and feeds them to the Pixhawk's EKF2.
- **tcp_map_streamer_node.py** — Streams the occupancy grid map (and handles the SLAM map reset command) over a raw TCP socket, as a fallback when native ROS 2 discovery over Wi-Fi is unreliable.
- **wall_boundary_node.py** — Extracts vectorized wall/room-boundary polygons from the occupancy grid for flight-safety visualization.

### `tests/`
- **\_env.py** — Import-path and headless-Qt setup; every test module imports it first.
- **run_tests.sh** — Runs the suite the way CI does (`./run_tests.sh hermetic` for the numpy-only subset).
- **rviz_palettes_reference.txt** — Reference palette values `test_rviz_palette.py` checks the canvas against.
- **test_actuator_button.py** — The compact servo button's text and status-dot colours (no ESP32 involved).
- **test_alarms_video.py** — The alarm manager, video health monitor, alarm card (including its running timer) and the FPV health signal.
- **test_audio.py** — Tone synthesis, the silent fallback with no backend, per-event rate limiting, and that submitting an alert never blocks the caller. Hermetic; makes no sound.
- **test_command_ack.py** — `COMMAND_ACK` dispatch dedup keyed on (token, result code).
- **test_diagnostics_redesign.py** — Glance strip, system cards and health lights, grey-first colours, responsive columns, the one-line heading and the Layout menu.
- **test_drone_glyph_zoom.py** — The tactical-map drone glyph scales with zoom, within bounds.
- **test_fonts.py** — The bundled font files and licences exist, both families register, headings *resolve* to Ubuntu and the title to Red Hat Display in the assembled window.
- **test_gui_features.py** — The value grid, the slide-to-confirm bar, map right-click vs pan, the measure tool, the adaptive scale bar, the route progress strip, the motor-test interlock matrix and the shortcut table. Needs PyQt5; skipped where it is absent.
- **test_gui_layout.py** — Layout regression: nothing clipped, overlapping or out of bounds, at every window size × UI scale, run twice (also with the alarm card showing and the vehicle armed); the rail-never-shorter-than-it-needs invariant; frame-geometry fit; toast placement; servo/kill row.
- **test_health.py** — Stall latching, recovery, rate/p95, registry and watchdog behaviour.
- **test_infrastructure.py** — Settings schema versioning and migrations, the UI stall monitor, the diagnostics bundle. Hermetic.
- **test_keepout_mission.py** — Keep-out zones, multi-stop missions and the inflation layer.
- **test_layout_density.py** — The rail's compact density (with hysteresis), the Motors page shedding detail and converging, the worst case, and the armed-green / disarmed-red convention.
- **test_link_range_walk.py** — Continuous-walk mode: per-second points (clean, ping blackout, video stall, no-data stays unknown, Wi-Fi fallback), the seconds-and-dBm verdict, report files and time axis, CLI flags, and a live-fed terminal stream run.
- **test_link_range.py** — The range test engine: `iw`/`ping` parsers on captured output, the JPEG frame counter at every split point, packet formats, sequence-based loss, verdicts, the loss-free range, path-loss fit, reports, and loopback integration (UDP up/down attribution, a real HTTP stream). Hermetic.
- **test_link_range_gui.py** — The range-test window: setup validation, prefill, connection checks, the whole walk → result flow against stand-ins. Runs on its own private Xvfb display; skipped without `tkinter`/Xvfb.
- **test_map_eval.py** — Every SLAM metric against synthetic grids, plus the CLI's exit codes.
- **test_preflight.py** — Checklist logic, the ARM gate and its overrides, vision-loss clearing the heading tick, Save to flash (guarded, disarmed only), parameter export format, the worker command, Radxa-status parsing end to end, and the map / screen-freeze / Radxa alarms.
- **test_radxa_services.py** — Watchdog decisions through time with an injected clock (grace, restart, rate limit, give-up, stopped-on-purpose), its HTTP endpoint, the video check, and the log janitor on temporary folders. Hermetic.
- **test_flight_logs.py** — Flight series recording (sampling, thinning, voltage, forward-compatible loading), the detail helpers/dialog/CSV export, and the Logs tab (ISO dates, Details, Trends).
- **test_fpv_tools.py** — Recorder (playable AVI, resize, no-frame = no file), snapshots, placeholder frames never recorded, overlay off by default and never burned in, the 60 s graph.
- **test_config_tab.py** — Captions for every field, restart rules, parsing, validation mapped to fields, change tracking, presets, search, save/reload, hidden fields preserved.
- **test_slam_panel.py** — SLAM side panel, state-aware path actions (including RESUME for `paused=True`), route summary, grey-first quality strip.
- **test_video_streaming.py** — Streamer socket tuning and one-buffer framing, a stalled loopback client receiving the newest frame rather than a backlog (fails without the tuning), and the capture thread's newest-frame-only hand-off to the GUI.
- **test_mini_feed.py** — The rail thumbnail: no work while hidden, ~10 fps thinning, cleared when frozen, click for fullscreen, shown only when the rail has room.
- **test_motor_response.py** — The motor range model, per-motor response (low → idle → off, sweep, mapping swap), staleness, commanded vs live, parameter routing, the throttle slider, and the caption/arrow declutter.
- **test_motor_test.py** — The bench test must use `MAV_CMD_ACTUATOR_TEST` (310), not `DO_MOTOR_TEST` (209), with the exact packet pinned.
- **test_param_codec.py** — PX4 float32 bit-cast decoding.
- **test_params_tab.py** — The read-only Parameters tab and the worker's `PARAM_VALUE` handling.
- **test_path_planner.py** — A* over synthetic occupancy grids: doorways, sealed walls, inflation clearance, unknown-space policy, and the regression that a goal clicked inside a wall never becomes the final waypoint.
- **test_planner_worker.py** — The planner thread, the live map score and the stale-map check.
- **test_position_uncertainty.py** — The EKF position-uncertainty ring on the tactical map.
- **test_rviz_palette.py** — The 2D canvas renders occupancy exactly as RViz does.
- **test_scaling.py** — UI scale resolution order, clamping, and the stylesheet rewrite - including that a `border: 1px solid` width is never scaled. Hermetic (no PyQt import).
- **test_smoke.py** — Import-level checks for the PyQt5 modules (skipped where PyQt5 is absent).
- **test_statustext.py** — `STATUSTEXT` reassembly and repeat suppression.
- **test_telemetry.py** — PX4 mode decoding, the `landed_state` × `is_airborne` matrix behind the disarm-vs-land interlock, RC health from `SYS_STATUS`, vision staleness, and motor-feed freshness.
- **test_video_sources.py** — Network-stream handling in the FPV video widget.

### Root
- **.github/workflows/tests.yml** — CI: a hermetic numpy-only job (which now includes the range-test engine tests), a PyQt5 offscreen GUI job, and a pyflakes lint gate over `core`, `protocol`, `controllers`, the diagnostics scripts and the tests.
- **camera.sh** — One-command headless pipeline launcher for the Radxa (run over SSH): sources ROS 2 + the workspace and launches the whole pipeline with `launch_rviz:=false`.
- **CMakeLists.txt** — ROS 2 `ament_cmake` build file; lists every script that gets installed as an executable.
- **Drone_1.5.params** — Exported Pixhawk parameter dump (full onboard parameter table) for this
  specific vehicle. Re-exported from the live aircraft on 2026-09-21 (PX4 1.17.0, git d6f12ad1).
  The previous copy had gone badly stale - it still had `PWM_MAIN_FUNC1..4 = 0` (no motor outputs
  assigned), `EKF2_EV_CTRL = 0` (vision fusion off) and a different board rotation, so anything
  diffed against it read as broken when the aircraft was fine. Re-export it after parameter work
  rather than letting it drift again.
- **guide.md** — User guide to the GCS application itself: architecture, every workspace/tab, the alarm card, look & feel, keyboard shortcuts, the guided-confirm safety gate, and how to run the range test. GUI content only — no install steps, no history.
- **mav.tlog** / **mav.tlog.raw** — Raw MAVLink telemetry log recordings (binary flight-log dumps, not source code).
- **package.xml** — ROS 2 package manifest (name, maintainer, build type).
- **Progress.md** — The project's single chronological engineering log: every real problem hit on hardware, why, and exactly what changed. Sensor → SLAM → EKF2 data flow, milestones, real-hardware findings, and a Known Issues & Roadmap section.
- **README.md** — Dated project updates, newest last (the latest: 2026-10-01), pointing to `guide.md`/`Progress.md` for detail.
- **requirements.txt** — Python pip dependencies for the whole pipeline + GCS app.
- **run_drone_slam.sh** — Launch script for the onboard SLAM pipeline (sources ROS 2 + workspace, runs `drone_rtabmap_all.launch.py`).
- **setup.md** — Everything needed for a fresh install: prerequisites, `mavlink-router` setup, build/launch commands, published ROS 2 topics, and FAQ/troubleshooting. Pipeline-side only — no GUI content.
