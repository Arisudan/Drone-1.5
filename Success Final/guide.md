# Drone-GCS User Guide (`guide.md`)

A guide to the laptop-side Ground Control Station application (`drone_gcs.py`) — what it's built from, how it's organized, and how to use every workspace. This file covers the **GUI application only**. For installing/running the underlying ROS 2 / SLAM / PX4 pipeline, see [`setup.md`](setup.md). For the engineering history behind these features (why they exist, what bugs were found and fixed), see [`Progress.md`](Progress.md).

---

## Table of Contents

1. [What Drone-GCS Is](#1-what-drone-gcs-is)
2. [Launching It](#2-launching-it)
3. [Header & Connection Bar](#3-header--connection-bar)
4. [Workspaces](#4-workspaces)
   - [Cockpit (PFD)](#cockpit-pfd)
   - [Tactical SLAM](#tactical-slam)
   - [Motors / Actuators](#motors--actuators)
   - [FPV Camera Tab](#fpv-camera-tab)
   - [Diagnostics](#diagnostics)
   - [Terminal / Logs](#terminal--logs)
   - [Parameters](#parameters)
5. [Guided Confirm: The Safety Gate](#5-guided-confirm-the-safety-gate)
6. [Keyboard Shortcuts](#6-keyboard-shortcuts)
7. [Audio Alerts](#7-audio-alerts)
8. [Current Limitations](#8-current-limitations)

---

## 1. What Drone-GCS Is

`drone_gcs.py` is a PyQt5 desktop application that connects to the Pixhawk (via `mavlink-router`) and to the Radxa's SLAM/video pipeline, presenting live telemetry, a 2D tactical map with click-to-fly path planning, motor diagnostics, camera feeds, and a read-only parameter browser — all in one aviation-styled dark-mode window.

It's built in three layers:

```
scripts/gcs/
├── core/        # Data models & logic — no Qt, no MAVLink wire format
│   ├── telemetry.py         # TelemetryState: the single source of truth every widget reads
│   ├── execution_tracker.py # Confirms a command was physically executed, not just ACKed
│   ├── path_planner.py      # Obstacle-aware A* over the live occupancy grid
│   ├── param_codec.py       # PX4 parameter IEEE-754 bit-cast decode/format
│   ├── audio.py             # Spoken/tonal operator alerts (rate-limited, non-blocking)
│   └── health.py            # Worker liveness/latency bookkeeping
├── protocol/     # Transport — owns the live connections, emits Qt signals
│   ├── mavlink_worker.py     # QThread: MAVLink connection, telemetry decode, command dispatch
│   └── ros2_map_listener.py  # QThread: SLAM map over ROS 2, with a raw-TCP fallback
└── ui/           # Every visible widget, one file per concern
    ├── top_status_strip.py, hud_widget.py, slam_map_widget.py, motor_widget.py,
    │ video_feed_widget.py, params_tab.py, logs_tab.py, cli_console.py,
    │ value_grid.py, sidebar_nav.py, guided_confirm.py, toast.py, styles.py,
    │ scaling.py, shortcuts.py, mission_progress.py, ...
```

A companion app, `radxa_monitor.py`, is a separate zero-control passive monitor meant to run on the Radxa itself or a field display — it is out of scope for this guide since it has no interactive features.

---

## 2. Launching It

```bash
./scripts/gcs/run_drone_gcs.sh
```
This sources ROS 2, forces the XCB Qt backend, and runs `drone_gcs.py`. No SSH or X11 forwarding is needed for real use — this runs directly on the laptop's own physical screen. See `setup.md` for getting the Radxa-side pipeline running first.

---

## 3. Header & Connection Bar

Two rows across the top of the window:

**Row 1** — branding, connection controls, Mode/Armed:
- **Network dropdown** (`HTIC_RND` / `DroneBridge5` / `DroneNet` / `Custom...`): picking a preset fills in the Radxa's IP for MAVLink, the TCP map bridge, and the FPV stream **all at once**. Port and Protocol are independent and keep whatever you last set (MAVLink's UDP:14550/TCP:5760 choice doesn't depend on which network you're on).
- **Connect / Disconnect** button — one control carries both the action and the current state (no separate redundant status badge).
- **Mode / Armed** badges, pinned top-right.

**Row 2** — live telemetry:
- Battery, VIO lock, EKF2 fusion badges.
- RX/TX telemetry throughput.
- VIO NED position readout.
- A transient notification **toast** lands in this row's own free space rather than floating over the workspace below it.

---

## 4. Workspaces

Switch between workspaces with the left-hand vertical sidebar, or `Ctrl+1` through `Ctrl+9`.

### Cockpit (PFD)
Deliberately minimal: just the **SPEED (m/s)** and **ALT AGL (m)** tapes plus a Mode/Armed banner, both driven by live Pixhawk telemetry (`ground_speed`/`altitude` from `LOCAL_POSITION_NED`). No artificial horizon, compass, or crosshair — that level of flight-attitude detail was deliberately removed in favor of the raw FPV feed itself.

### Tactical SLAM
The primary situational-awareness view, with two switchable layers:
- **2D Blueprint & A\* Planner** — a top-down occupancy-grid canvas (`/map` raw and/or `/map_thin` skeleton, togglable), with an arrow-shaped drone icon whose heading smoothly interpolates toward the true VIO/EKF2 yaw (shortest-angle blending, no snap or wrap-around glitch at 359°→0°), a breadcrumb flight trail, and pan/zoom/rotate controls.
- **3D RViz2 Viewport** — a real embedded RViz2 window (X11 window swallowing) for the full point-cloud/SLAM 3D view, with **Launch / Reload / Close** controls.

**Click-to-navigate**: clicking anywhere on the 2D map stages a goal pose and runs the obstacle-aware A* planner over the live grid, drawing the route with distance/ETA. Toolbar controls:
- **EXECUTE PATH** — dispatches the drone through the computed waypoints in OFFBOARD mode, gated behind the slide-to-confirm bar (see [§5](#5-guided-confirm-the-safety-gate)).
- **PAUSE / RESUME** — halts in place (`AUTO.LOITER`) or resumes the remaining route.
- **ABORT PATH** — cancels the active route and lands, also gated behind slide-to-confirm.
- **Reset Map** — wipes the live SLAM map and restarts mapping from empty, for when the map has drifted or accumulated garbage. Confirmation-gated (no undo) and **hard-disabled while armed**, since it would pull the EKF2 vision-fusion reference and any live obstacle data out from under an actively flying vehicle.
  > ⚠️ This is destructive and cannot be undone. Only use it while disarmed.
- **Dynamic collision re-check**: every 200ms during a flight, the remaining path is re-checked against the live grid; if an obstacle appears within 1.5m ahead, the GCS automatically engages `AUTO.LOITER` and computes a fresh A* detour.

### Motors / Actuators
A top-down Quad X frame diagram with each rotor drawn where it physically sits and tinted live by its commanded PWM, plus a per-motor PWM gauge bar (1000–2000 µs) and a heading arrow drawn inside the body hub (one heading indicator, not a separate floating chevron).

The **bench motor-test panel** (`MAV_CMD_ACTUATOR_TEST`) is gated on: a live link, a disarmed and grounded vehicle, an expiring props-removed acknowledgement, and a throttle ceiling. Running a test reflects the *commanded* throttle on the bars and diagram immediately (rather than waiting for `SERVO_OUTPUT_RAW` telemetry, which a test-driven output may not reliably produce) — a real telemetry value simply overwrites this once it arrives. Every stop path (hold-release, STOP ALL, the 60s safety-ack expiry, sequence advance) resets all channels to idle.

### FPV Camera Tab
Connects to the onboard MJPEG stream (`http://<radxa_ip>:8080/video`) and displays the live D435i color feed, auto-reconnecting every 2s on a dropped or never-opened stream. Also supports a USB webcam, a synthetic test pattern, or a custom RTSP/HTTP URL (with per-source URL memory and RTSP transport/timeout options).

- **Fullscreen (⛶)**: every video view — the docked tab, and the floating/undocked window — has a fullscreen toggle. Press `Esc` or click anywhere to exit.
- No overlay is drawn on top of the video (no HUD ladder/crosshair/compass) — flight status lives in the header and the Cockpit PFD instead.

### Diagnostics
A grid of telemetry tiles the operator chooses, orders, and sizes, drawn from a registry of ~50 available fields (IMU health, EKF2 fusion flags, motor PWMs, body velocities, attitude angles, and more). A saved layout always wins over the default set.

### Terminal / Logs
- An embedded command console (`cli_console.py`) with history recall (`↑`/`↓`) for typing flight commands directly.
- A flight log viewer.
- **Reset Logs** button: password-gated (password: `admin`). Prompts for the password via a dialog before clearing the flight log file — the in-progress session's own state is left untouched.

### Parameters
*(Ctrl+9)* — a live PX4 parameter table: search box, sortable columns (Name / Value / Type / Index), populated via the standard MAVLink parameter protocol (`PARAM_REQUEST_LIST` / `PARAM_VALUE`). Values are decoded through the same IEEE-754 bit-cast logic used elsewhere in this project for reading typed PX4 parameters correctly (an int32 param read as a naive float produces nonsense like `1.4e-45`).

**Read-only for now** — writing a parameter from this tab (with the same guarded-confirm treatment ARM/DISARM get, plus a mandatory readback) is a planned later phase, not yet built.

---

## 5. Guided Confirm: The Safety Gate

Every irreversible action (arm, disarm, emergency kill, abort path) and every numeric dispatch (takeoff altitude, yaw, relative move) is gated behind a **slide-to-confirm bar** rather than a click or a modal dialog whose default button is one accidental `Return` away — QGroundControl-style. The bar floats directly **above the button that triggered it** (falling back to below if there isn't enough headroom), and follows the window if it's resized while showing. Nothing is dispatched until the operator actually drags the slider through; the confirm bar itself has no execution logic of its own — the main window still runs the exact same command method either way.

---

## 6. Keyboard Shortcuts

Press **F1** at any time for the full shortcut overlay, generated live from the same binding table the app uses. Workspaces switch with `Ctrl+1` through `Ctrl+9` (Cockpit, Tactical SLAM, Motors, FPV, Diagnostics, Terminal, ..., Parameters). No destructive action fires directly from a bare key — arm, disarm, and abort all open the guided-confirm bar, and emergency kill has no keyboard binding at all.

---

## 7. Audio Alerts

Spoken and tonal alerts for key events (arm/disarm, command rejection, connection loss) run on a background daemon thread through a bounded queue, so a wedged sound device can never block the GUI. Tones are synthesized once and played via `aplay`; speech goes through `spd-say`; a machine with neither degrades silently rather than failing. Alerts are rate-limited per event — PX4 re-runs its preflight checks every ~2s, and an alert that repeats forever is one the operator just mutes.

---

## 8. Current Limitations

- The **Parameters tab** is read-only (no write-then-verify, no guided-confirm gate for reboot-required parameters yet), and has only been tested against a synthetic parameter burst, not a real vehicle's full 1000–1800+ entry set.
- No fullscreen support outside the video views (map/RViz remain windowed within their tab).

See [`Progress.md`](Progress.md)'s Known Issues & Roadmap section for the complete, up-to-date list across the whole project, not just the GUI.
