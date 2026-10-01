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
8. [Alarm Card & Video Health](#8-alarm-card--video-health)
9. [Look & Feel: Colour, Fonts, Short Windows](#9-look--feel-colour-fonts-short-windows)
10. [Range Test: How Far Does the Live Feed Run Without Packet Loss?](#10-range-test-how-far-does-the-live-feed-run-without-packet-loss)
11. [Current Limitations](#11-current-limitations)

---

## 1. What Drone-GCS Is

`drone_gcs.py` is a PyQt5 desktop application that connects to the Pixhawk (via `mavlink-router`) and to the Radxa's SLAM/video pipeline, presenting live telemetry, a 2D tactical map with click-to-fly path planning, motor diagnostics, camera feeds, a standing alarm card, and a read-only parameter browser — all in one aviation-styled dark-mode window.

It's built in four layers:

```
scripts/gcs/
├── core/         # Data models & logic — no Qt, no MAVLink wire format
│   ├── telemetry.py         # TelemetrySnapshot: the single source of truth every widget reads
│   ├── execution_tracker.py # Confirms a command was physically executed, not just ACKed
│   ├── path_planner.py      # Obstacle-aware A* over the live occupancy grid
│   ├── param_codec.py       # PX4 parameter IEEE-754 bit-cast decode/format
│   ├── motor_range.py       # The vehicle's motor min/max/disarm/function → % and idle/high/saturated (one model for the station)
│   ├── alarms.py            # Prioritised, acknowledgeable alarm list (pure Python)
│   ├── video_health.py      # fps / pipeline latency / jitter / freeze detection
│   ├── ui_stall.py          # Detects the UI thread blocking (30 Hz tick overrun)
│   ├── log_bundle.py        # One-click zip of flight history + settings + logs
│   ├── settings.py          # Typed, validated, versioned settings (+ migrations)
│   ├── audio.py             # Spoken/tonal operator alerts (rate-limited, non-blocking)
│   └── health.py            # Worker liveness/latency bookkeeping
├── protocol/     # Transport — owns the live connections, emits Qt signals
│   ├── mavlink_worker.py     # QThread: MAVLink connection, telemetry decode, command dispatch, parameter reads
│   └── ros2_map_listener.py  # QThread: SLAM map over ROS 2, with a raw-TCP fallback
├── controllers/  # What the main window DOES, split out of drone_gcs.py (mixins)
│   ├── flight_commands.py    # arm / disarm / takeoff / yaw / altitude / mode / kill
│   ├── mission_control.py    # execute / pause / resume / abort path, collision & detour handling
│   └── alarm_control.py      # telemetry & video health → the alarm list
└── ui/           # Every visible widget, one file per concern
    ├── top_status_strip.py, hud_widget.py, slam_map_widget.py (input) + map_canvas_render.py (drawing),
    │ motor_widget.py, actuator_widget.py, value_grid.py, alarm_banner.py, mini_feed.py,
    │ video_feed_widget.py, params_tab.py, logs_tab.py, cli_console.py, sidebar_nav.py,
    │ guided_confirm.py, toast.py, styles.py, fonts.py, scaling.py, shortcuts.py, mission_progress.py, ...
```

The threads behind this (which one owns what, and which signals cross to the UI) are documented in [`docs/threading.md`](docs/threading.md).

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

Under the header, **only while something is wrong**, a slim **alarm card** appears (see [§8](#8-alarm-card--video-health)).

---

## 4. Workspaces

Switch between workspaces with the left-hand vertical sidebar, or `Ctrl+1` through `Ctrl+9`.

**Camera thumbnail:** on the tabs that have no camera of their own — Motor Actuators, Diagnostics, Flight Terminal, Flight Logs, Configuration, Parameters — a small live thumbnail sits at the bottom of the rail, above the speed/altitude/mode footer, so the drone's view is never out of sight while you check or change something. Click it for fullscreen. A dot and word under it show `LIVE`, `SLOW`, `FROZEN`, `NO SIGNAL` or `NO VIDEO`; when the feed freezes the picture is **cleared**, never left up, so a dead frame cannot pass for a live one. It uses frames the one capture already decoded (about 10 fps, nothing while hidden) and only appears when the rail has room.

### Cockpit (PFD)
Deliberately minimal: just the **SPEED (m/s)** and **ALT AGL (m)** tapes plus a Mode/Armed banner, both driven by live Pixhawk telemetry (`ground_speed`/`altitude` from `LOCAL_POSITION_NED`). No artificial horizon, compass, or crosshair — that level of flight-attitude detail was deliberately removed in favor of the raw FPV feed itself.

**Control column** (right-hand side): ARM / DISARM / HOLD / LAND, then **Navigation** and **Flight Mode**, then one row holding the **servo button and EMERGENCY KILL**:

`● [ SERVO 0° → 90° ]  |  [ EMERGENCY KILL ]`

The servo is an ESP32-C3 + MG995 driven over WiFi **HTTP** (not MAVLink — it works whether or not the vehicle is linked). One press toggles 0°↔90° (first press from an unknown position goes to 0°), queued on the ESP32 so pressing while it moves is safe. The dot is grey idle / amber moving / red offline; the label reads `SERVO OFFLINE` when it can't be reached (click to retry); the IP and any error are in the tooltip. Set the address under *Configuration → ESP32 SERVO ACTUATOR*. The row is deliberately lopsided: kill gets 60% of the width, stays solid red, and sits behind a divider with a 16 px gap on each side; the servo stays neutral grey, so the two can't be mistaken for each other.

### Tactical SLAM
The primary situational-awareness view, with two switchable layers:
- **2D Blueprint & A\* Planner** — a top-down occupancy-grid canvas (`/map` raw and/or `/map_thin` skeleton, togglable), with an arrow-shaped drone icon whose heading smoothly interpolates toward the true VIO/EKF2 yaw (shortest-angle blending, no snap or wrap-around glitch at 359°→0°), a breadcrumb flight trail, and pan/zoom/rotate controls.
- **3D RViz2 Viewport** — a real embedded RViz2 window (X11 window swallowing) for the full point-cloud/SLAM 3D view, with **Launch / Reload / Close** controls.

![Redesigned drone icon: one arrow-shaped fuselage instead of a circle plus a separate floating chevron](docs/images/slam_drone_icon_redesign.png)

**Click-to-navigate**: clicking anywhere on the 2D map stages a goal pose and runs the obstacle-aware A* planner over the live grid, drawing the route with distance/ETA. Toolbar controls:
- **EXECUTE PATH** — dispatches the drone through the computed waypoints in OFFBOARD mode, gated behind the slide-to-confirm bar (see [§5](#5-guided-confirm-the-safety-gate)).
- **PAUSE / RESUME** — halts in place (`AUTO.LOITER`) or resumes the remaining route.
- **ABORT PATH** — cancels the active route and lands, also gated behind slide-to-confirm.
- **Reset Map** — wipes the live SLAM map and restarts mapping from empty, for when the map has drifted or accumulated garbage. Confirmation-gated (no undo) and **hard-disabled while armed**, since it would pull the EKF2 vision-fusion reference and any live obstacle data out from under an actively flying vehicle.
  > ⚠️ This is destructive and cannot be undone. Only use it while disarmed.
- **Dynamic collision re-check**: every 200ms during a flight, the remaining path is re-checked against the live grid; if an obstacle appears within 1.5m ahead, the GCS automatically engages `AUTO.LOITER` and computes a fresh A* detour.

### Motors / Actuators
Heading: **ACTUATOR OUTPUTS & MOTOR TELEMETRY**, with a one-line status on the right (`ALL MOTORS OFF` / `IDLE` / `MOTORS ACTIVE · 62%` / `HIGH LOAD` / `SATURATION · M3 1850 µs`). Three cards:

- **Frame geometry** — a top-down drone: tapered arms, a body with a camera pod and LEDs (white front, red rear — the heading is the drawing itself, there is no "NOSE" label), motor hubs **M1–M4**, and propellers that **turn at a rate proportional to the output**, CW and CCW motors in opposite directions. Each rotor's tint follows its state, each has a rotation arrow beside it, and a caption `FR · CCW` / `1500 µs · 50%`. Click a rotor to select it for the bench test.
- **Per-motor gauges** — one bar per motor with its µs, percentage and position.
- **Bench Motor Test** — see below.

**Scaled to the vehicle's own motor range.** After connecting, the GCS reads `PWM_MAIN_MINn / MAXn / DISn / FUNCn` from the vehicle (single parameter reads; the Parameters tab is not disturbed). Percentages and the idle / high / saturated states are relative to *that* range — a 1100–1900 µs vehicle at full throttle reads 100 %, not 80 % — and `FUNCn` decides which output is which motor. The footer says `Scale: 1100–1900 µs (from vehicle)`, or `default 1000–2000 µs (vehicle range not read yet)` until it arrives. The same thresholds (idle ≤ 5 %, high > 75 %, saturated > 90 % of range) drive the diagram, the bars, the status line and the Diagnostics motor tiles, so they cannot disagree.

**It never presents the past as the present.** `SERVO_OUTPUT_RAW` arrives at 5 Hz; if it stops for about 2 s the bars and rotors grey out, the propellers stop turning, and the status reads `MOTOR DATA STALE · N s` or `NO MOTOR DATA`. During a bench test the *commanded* output is shown at once and labelled `COMMANDED` (a test-driven output may never appear in `SERVO_OUTPUT_RAW`); a live sample that confirms it takes over, and `COMMANDED · not yet confirmed` says when the vehicle hasn't reported it.

**Bench Motor Test** (`MAV_CMD_ACTUATOR_TEST`), gated on a live link, a disarmed and grounded vehicle, an expiring props-removed acknowledgement, and a throttle ceiling:
- an **interlock checklist** — `LINK`, `DISARMED`, `ON GROUND`, `PROPS OFF` — so you can see which condition is holding it locked (red only for armed/airborne);
- the **props-off** confirmation with a visible countdown (`locks in 0:47`) — it re-locks itself after 60 s;
- a **2×2 motor selector** laid out like the aircraft seen from above (front row on top), with rotation arrows;
- the **throttle slider**: a thick track with a large handle, ticks every 5 %, the 25 % ceiling in amber, and a label such as `8 / 25 % · 1080 µs` (what that throttle means *on this vehicle*). Press anywhere on it to jump there, drag to follow, mouse wheel and the **−/+** buttons step 1 %;
- **HOLD TO SPIN** (momentary — the vehicle stops it within 2 s of release), **SEQ 1→4** to verify motor order and rotation against the diagram, and **STOP ALL**.
Every stop path (hold-release, STOP ALL, the 60 s expiry, sequence advance) resets all channels to idle.

On a short window the page sheds detail rather than overlap: first the footnote, then the checklist becomes a one-line reason and the selector folds into one row of four; it comes back when there is room again.

![Redesigned frame heading arrow, rendered offscreen inside the body hub](docs/images/motor_frame_heading_arrow.png)

### FPV Camera Tab
Connects to the onboard MJPEG stream (`http://<radxa_ip>:8080/video`) and displays the live D435i color feed, auto-reconnecting every 2s on a dropped or never-opened stream. Also supports a USB webcam, a synthetic test pattern, or a custom RTSP/HTTP URL (with per-source URL memory and RTSP transport/timeout options).

- **Fullscreen (⛶)**: every video view — the docked tab, and the floating/undocked window — has a fullscreen toggle. Press `Esc` or click anywhere to exit.
- No overlay is drawn on top of the video (no HUD ladder/crosshair/compass) — flight status lives in the header and the Cockpit PFD instead.
- A **health line** under the picture: `LIVE · 29 fps · pipeline 18 ms · jitter 4 ms`, or `DEGRADED`, `CONNECTING`, `FROZEN · no frame for 3.0 s`, `NO SIGNAL`. Grey while healthy, amber/red only when not. "pipeline" is the time from the capture thread reading a frame to it being painted *inside the GCS*; it excludes the camera, encoder and network, so it is an early warning, not true glass-to-glass latency. A frozen or dead feed also raises an alarm (§8).

### Diagnostics
Heading **TELEMETRY VALUES** (one line) and a single **Layout ▾** menu: *Max columns*, *Text size*, *Add or remove values…*, *Edit tiles (reorder / remove)*, *Reset to default*. Below it:

- A **glance strip** — the six things you must be able to read in one look: **Altitude, Ground speed, Battery** (with V and A under it), **Link, Mode** (with ARMED / DISARMED under it) and **Flight time**. Not configurable on purpose, so muscle memory works; it splits into two rows of three on a narrow window.
- **System cards** — one card per system (Link, Flight state, Power, Attitude, Position, Velocity, Vision / EKF2, Motors, RC link, GPS, Commands, Companion, Station) holding your chosen values for that system, packed into columns with no empty cells. Each card has a **health light** in its header: grey when nothing needs attention (the normal state), amber for caution, red for fault; hover it to see what.
- Each value is a quiet small-caps caption over a large number, the unit a size down. A thin bar at the left of a cell appears only when that value is in caution or fault.

**Grey-first colour:** healthy values are plain white; colour means something. Amber is caution (battery low, a saturated motor), red is fault (battery critical, a position feed that was flowing and stopped, no link); position is simply grey before any data arrives rather than a wall of red zeros. A stale motor feed shows `--`, not the last value.

**Max columns** is a maximum: the grid uses fewer when the window can't fit that many cards at their minimum width, so nothing is clipped or pushed off the right edge. You still choose, order and size the values from a registry of ~50 fields; a saved layout always wins over the default set.

### Terminal / Logs
- An embedded command console (`cli_console.py`) with history recall (`↑`/`↓`) for typing flight commands directly.
- A flight log viewer, with **Export CSV** and **Export Bundle**: one zip of the flight history, your settings, any `.log` / `.tlog` files and a manifest — attach it to a bug report as-is.
- **Reset Logs** button: password-gated (password: `admin`). Prompts for the password via a dialog before clearing the flight log file — the in-progress session's own state is left untouched.

### Parameters
*(Ctrl+9)* — a live PX4 parameter table: search box, sortable columns (Name / Value / Type / Index), populated via the standard MAVLink parameter protocol (`PARAM_REQUEST_LIST` / `PARAM_VALUE`). Values are decoded through the same IEEE-754 bit-cast logic used elsewhere in this project for reading typed PX4 parameters correctly (an int32 param read as a naive float produces nonsense like `1.4e-45`).

**Read-only for now** — writing a parameter from this tab (with the same guarded-confirm treatment ARM/DISARM get, plus a mandatory readback) is a planned later phase, not yet built.

![The Parameters tab, rendered offscreen with 30 synthetic parameters](docs/images/params_tab_phase1.png)

---

## 5. Guided Confirm: The Safety Gate

Every irreversible action (arm, disarm, emergency kill, abort path) and every numeric dispatch (takeoff altitude, yaw, relative move) is gated behind a **slide-to-confirm bar** rather than a click or a modal dialog whose default button is one accidental `Return` away — QGroundControl-style. The bar floats directly **above the button that triggered it** (falling back to below if there isn't enough headroom), and follows the window if it's resized while showing. Nothing is dispatched until the operator actually drags the slider through; the confirm bar itself has no execution logic of its own — the main window still runs the exact same command method either way.

---

## 6. Keyboard Shortcuts

Press **F1** at any time for the full shortcut overlay, generated live from the same binding table the app uses. Workspaces switch with `Ctrl+1` through `Ctrl+9` (Cockpit, Tactical SLAM, Motors, FPV, Diagnostics, Terminal, ..., Parameters). No destructive action fires directly from a bare key — arm, disarm, and abort all open the guided-confirm bar, and emergency kill has no keyboard binding at all.

---

## 7. Audio Alerts

Spoken and tonal alerts for key events (arm/disarm, command rejection, connection loss) run on a background daemon thread through a bounded queue, so a wedged sound device can never block the GUI. Tones are synthesized once and played via `aplay`; speech goes through `spd-say`; a machine with neither degrades silently rather than failing. Alerts are rate-limited per event — PX4 re-runs its preflight checks every ~2s, and an alert that repeats forever is one the operator just mutes. Audio is *edge-triggered* (it speaks once); the alarm card (§8) is its *standing* counterpart.

---

## 8. Alarm Card & Video Health

A slim card under the header that is **hidden while nothing is wrong**, so it costs no cockpit space in normal flight. When there is an alarm it shows the highest-priority one: a severity bar and badge, a bold title, a grey hint line (e.g. *"Check the stream URL and that the camera streamer is running."*), **how long it has been active** as a running `m:ss`, a `+N` count for others, and **Acknowledge**.

| Alarm | Level | When |
|---|---|---|
| Link lost | CRITICAL | telemetry stopped — only after the link has been up once (not at startup) |
| Battery critical / low | CRITICAL / WARN | at or below the configured thresholds |
| Vision lost | CRITICAL | no VIO / vision fusion **while airborne** |
| Position feed lost | CRITICAL | no local position **while airborne** |
| Video feed frozen / no signal | WARN | the camera stream stopped delivering frames |

- Red = unacknowledged CRITICAL, amber = unacknowledged WARN, neutral grey once acknowledged (with an `ACKNOWLEDGED` tag). **Acknowledging does not clear an alarm** — only the condition going away does; an acknowledged alarm that gets worse becomes unacknowledged again.
- Alarms are *level-triggered*: each condition holds its alarm for exactly as long as it is true, so the card always answers "what is wrong right now". A new or escalated alarm also writes a console line (and a toast for CRITICAL). It adds no sound of its own — the audio alerts already cover link, battery and vision.
- The video row comes from the FPV health monitor (§4). Map-stalled and UI-stall alarms are not wired yet.

---

## 9. Look & Feel: Colour, Fonts, Short Windows

- **Colour carries meaning.** The station is grey-first: strong colour appears only for a real state or problem. Armed is **green** and disarmed is **red** everywhere (header, navigation footer, Diagnostics) — as a *state*, so a resting disarmed vehicle never raises a health light or a side bar.
- **Fonts.** `DRONE-GCS` is set in **Red Hat Display**; every heading — Navigation, Flight Mode, each tab's section and card headings, the diagnostics group titles — in **Ubuntu**. Body text, numbers, buttons, the console and the sidebar tab names are unchanged. Both families ship in `assets/fonts/` (with their licences) and are loaded at startup, so they render on any machine without installing anything; a missing file falls back to the old font rather than breaking.
- **Short windows.** At the smallest supported window and the largest UI scale (with the alarm card showing and the vehicle armed — the tallest case) the layout tightens instead of overlapping: the navigation rail tightens its rows, the Motors page drops its footnote and folds its checklist, and the camera thumbnail hides when it would not fit. The layout test suite runs every page at every size and scale, and a second time in exactly that tallest state.

---

## 10. Range Test: How Far Does the Live Feed Run Without Packet Loss?

A walk-away test for the camera-feed Wi-Fi link. Put the Radxa and router together,
carry the laptop away from them, and the tool finds the furthest distance at which
the feed loses **no packets** and does not stall.

```
cd ~/Flop/scripts/diagnostics
python3 link_range_test.py
```

A window asks for the Radxa's IP address (it remembers the last one), shows which Wi-Fi network the laptop is on, can check the connection, and opens a walking screen:

1. Stand next to the Radxa and press **Measure here (0 m)**, then stand still for the countdown.
2. Walk to a measured spot (tape or floor tiles), type the distance if it is not the suggested one, press **I'm at the mark**, and stand still again. Repeat further out.
3. Press **Finish and show result**. The answer comes first: *live feed with no packet loss up to N m*, then a table per distance and a full report (`report.html`, with charts) saved under `~/link_range_results/`.

It measures the laptop's Wi-Fi signal and retries, ping loss and round-trip time, the
real MJPEG stream (frames per second, stalls, reconnects), and UDP packet loss in each
direction. The laptop must be on the **same Wi-Fi network as the Radxa**. The packet-loss probe needs
SSH access to the Radxa (`ssh-copy-id radxa@<ip>` once); it copies a small helper to
`/tmp` on the Radxa and stops it afterwards. "No loss" means exactly that by default;
raise *Allowed loss* under Advanced options to tolerate a little.

`python3 link_range_test.py --demo` shows a sample report from synthetic data, and
`python3 link_range_test.py --cli --radxa <ip> --udp --start-helper radxa@<ip>` runs it
from the terminal with no window. How every number is defined and calculated, and what the
test cannot tell you: [`docs/link_range_test.md`](docs/link_range_test.md).

---

## 11. Current Limitations

- The **Parameters tab** is read-only (no write-then-verify, no guided-confirm gate for reboot-required parameters yet), and has only been tested against a synthetic parameter burst, not a real vehicle's full 1000–1800+ entry set.
- The **motor scale** (`PWM_MAIN_*`) is read with single parameter reads that have not been tried against the real vehicle; if they never arrive the default 1000–2000 µs range is used and the footer says so. Only outputs 1–4 are covered (all `SERVO_OUTPUT_RAW` carries here).
- The **alarm card** covers link, battery, vision, position and video. There are no map-stalled or UI-stall alarms yet.
- The **servo** talks plain HTTP to the ESP32 with no authentication, and nothing stops it being pressed while the vehicle is flying.
- The **range test** has been verified on this laptop's Wi-Fi card, on loopback and in its window on a virtual display — not yet against the real Radxa or on a real walk.
- No fullscreen support outside the video views (map/RViz remain windowed within their tab).
- Everything above was checked offscreen / on the bench; no real flight has happened.

See [`Progress.md`](Progress.md)'s Known Issues & Roadmap section for the complete, up-to-date list across the whole project, not just the GUI.
