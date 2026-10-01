# RTAB-Map Drone Pkg — 2026-09-15 Update

**Video Streaming in GCS (with all /topics integrated) over WiFi Access Point (DroneBridge5)**

Live RGB FPV video from the Intel RealSense D435i now streams from the Radxa to the
laptop GCS over the `DroneBridge5` WiFi access point, alongside full MAVLink telemetry
and all existing ROS 2 `/topics` (SLAM map, odometry, wall boundaries, etc.) — all
integrated in a single GCS session over one network.

- Onboard `d435i_video_streamer.py` compresses `/camera/color/image_raw` to JPEG and
  serves it as an MJPEG stream over HTTP (`:8080/video`), keeping bandwidth low enough
  (~6 Mbps) to share the link with MAVLink and SLAM traffic without stutter.
- GCS `drone_gcs.py` header now has a Network dropdown (`HTIC_RND` / `DroneBridge5`)
  that auto-fills the Radxa's IP for MAVLink, the TCP map bridge, and the video stream
  together.
- GUI usage: see [`guide.md`](guide.md). Engineering history: see [`Progress.md`](Progress.md).

---

## 2026-09-16 Update

**2D Occupancy Grid Map Visualization in GCS**

The Tactical SLAM tab's live 2D map view got an accuracy and usability pass:

- Drone icon heading now smoothly interpolates toward the true VIO/EKF2 yaw (shortest-angle
  blending) instead of snapping — sharp real-world turns show up on the icon immediately and
  correctly, with no glitch at the 359°→0° wrap-around.
- Dual-layer rendering: view the raw RTAB-Map grid (`/map`), the thinned single-pixel wall
  skeleton (`/map_thin`), or both layers overlaid together.
- Click-to-navigate: clicking anywhere on the map stages a goal and runs an obstacle-aware
  A* planner over the live occupancy grid, drawing the planned path with distance/ETA.
- Tactical drone symbol with a heading chevron and RealSense FOV cone, a breadcrumb trail,
  auto-follow camera, rotation/zoom controls, and the Reset Map button (wipes the SLAM
  database and restarts mapping from scratch — safe for a headless Radxa with no display,
  keyboard, or mouse attached).
- GUI usage: see [`guide.md`](guide.md). Engineering history: see [`Progress.md`](Progress.md).

---

## 2026-09-30 Update

**ESP32 WiFi Servo Actuator Button in the GCS Cockpit**

> Layout superseded on 2026-10-01: the separate ACTUATOR card is gone — the servo button now
> shares one row with EMERGENCY KILL (see the 2026-10-01 update below). Everything else here
> (HTTP protocol, status polling, address settings) is unchanged.

The cockpit control panel has a new **ACTUATOR** card, between Navigation / Flight Mode and
EMERGENCY KILL, that drives an ESP32-C3 SuperMini + MG995 servo over WiFi. It talks HTTP to
the ESP32's own firmware (port 80) — not MAVLink — so it works whether or not the vehicle is
connected.

- One button, `SERVO → 0°` / `SERVO → 90°`: toggles 0↔90 exactly like the ESP32's BOOT
  button (first press from an unknown position goes to 0°). Commands go through the
  firmware's queue (`GET /setAngle?value=N`), so pressing while the servo moves is safe; a
  full queue (`503`) is reported in the console.
- Live status line polled from `GET /status` once a second: `ONLINE 0° idle`,
  `ONLINE 90° moving`, or a red `OFFLINE http://…`. The toggle follows the servo's real
  position, so it stays correct if the servo was moved from the ESP32's web page or button.
- Async requests with a 1.5 s timeout — an ESP32 that is off never freezes the GCS.
- Address: defaults to `192.168.4.1` (the firmware's fallback hotspot `ESP32C3-Servo`). On
  the `actuator` WiFi it takes a DHCP address — set it in *Configuration → ESP32 SERVO
  ACTUATOR* (applied on save, no restart), or with `--actuator-host <ip>` /
  `GCS_ACTUATOR_HOST`. Reserve the lease on the router so it does not move.
- Code: `scripts/gcs/ui/actuator_widget.py` (new), plus the `actuator` section in
  `core/settings.py`, its card in `ui/config_tab.py`, and the cockpit wiring in
  `drone_gcs.py`.

---

## 2026-10-01 Update

A large GCS pass: new structure under the hood, a reworked Motors tab and Diagnostics tab, a standing alarm
card, video health, bundled fonts, and a new walk-away range test for the camera link. Everything below is
covered by the offscreen/hermetic test suite (`tests/run_tests.sh`, 654 tests passing).
⚠️ Nothing here has been flown or seen on the real vehicle — see [`Progress.md`](Progress.md)'s Known Issues.

**Cockpit — servo and kill on one row**
- The ESP32 servo is now a compact button on the left, `● SERVO 0° → 90°`, with EMERGENCY KILL on the right and
  a divider between them (kill gets 60% of the row, stays solid red; the servo stays neutral). The status dot is
  grey idle / amber moving / red offline; the IP and any error moved into the tooltip. Saves a whole card of height.

**Motor Actuators tab**
- One-line heading `ACTUATOR OUTPUTS & MOTOR TELEMETRY`; a realistic top-down drone (tapered arms, camera pod, white
  front / red rear LEDs instead of a "NOSE" label, motor hubs, propellers that spin at a rate proportional to the
  output, CW/CCW arrows beside each rotor).
- Percentages, colours and states are relative to the **vehicle's own motor range**: `PWM_MAIN_MIN/MAX/DIS/FUNCn` are
  read after connecting (single `PARAM_REQUEST_READ`s — the Parameters tab is undisturbed). `FUNCn` maps outputs to
  motors. Until read, the documented 1000–2000 default is used and the footer says so. One threshold set is shared by
  the diagram, bars, status line and the Diagnostics tiles.
- Never shows the past as the present: if `SERVO_OUTPUT_RAW` stops for ~2 s the motors grey out and the status reads
  `MOTOR DATA STALE` / `NO MOTOR DATA`. A bench test shows `COMMANDED` until a live sample confirms it.
- **Bench Motor Test** redesign: interlock checklist (LINK / DISARMED / ON GROUND / PROPS OFF), a visible 60 s
  props-off countdown, a 2×2 motor selector laid out like the aircraft, and a new throttle slider (large handle,
  click-anywhere, −/+ buttons, wheel, the 25% ceiling marked, label `8 / 25 % · 1080 µs`). Interlock logic unchanged.

**Diagnostics tab**
- `TELEMETRY VALUES` heading on one line; the toolbar is one **Layout ▾** menu (max columns, text size, add/remove,
  edit tiles, reset). A **glance strip** (altitude, ground speed, battery, link, mode/armed, flight time) sits on top,
  then one **card per system** (Power, Attitude, Position, Motors, …) with a health light (grey / amber / red).
- Tiles are caption-over-value. **Grey-first colour**: healthy values are plain white; amber/red appear only for
  caution/fault; the red `+0.000` position values before any data are gone. Columns is now a *maximum* — fewer are
  used when the window can't fit them, which fixes the clipped third column at the smallest size.

**Armed green, disarmed red** everywhere (header, navigation footer, Diagnostics) — as a *state* colour, not a fault.

**Alarm card & video health**
- A slim card under the header, hidden until something is wrong: severity bar, title, hint line, **a running timer**
  (it used to freeze at its raise-time text), `+N` for others, Acknowledge. Raised for link lost, battery low/critical,
  vision/position lost while airborne, and a frozen/dead video feed. Acknowledging doesn't clear it; only the
  condition clearing does.
- The FPV tab shows a health line (`LIVE · 29 fps · pipeline 18 ms · jitter 4 ms`, or `FROZEN` / `NO SIGNAL`).
  "pipeline" is capture→paint inside the GCS only, not true glass-to-glass.

**Camera thumbnail in the navigation rail** on the tabs that have no camera of their own (Motors, Diagnostics,
Terminal, Logs, Configuration, Parameters): ~10 fps, click for fullscreen, cleared to `NO VIDEO` when the feed is
frozen so a dead frame never looks live. The rail tightens its rows, and the Motors page sheds detail, on a short window.

**Fonts** — `DRONE-GCS` in Red Hat Display, every heading in Ubuntu; both bundled in `assets/fonts/` (with licences)
and loaded at startup, so they work on any machine. Body text, numbers and the sidebar tabs are unchanged.

**Under the hood** — `drone_gcs.py` split into `controllers/` mixins (flight commands, mission control, alarms);
the map canvas's draw code moved to `ui/map_canvas_render.py`; settings gained a schema version + migrations; a
one-click **Export Bundle** (flight history + settings + logs) on the Logs tab; a UI-stall monitor; and
[`docs/threading.md`](docs/threading.md) documenting every thread and its signals.

**Range test (new tool)** — `python3 scripts/diagnostics/link_range_test.py` opens a window that asks for the Radxa IP,
then guides a walk-away test and reports **how far the live camera feed runs with no packet loss**. See
[`guide.md`](guide.md#10-range-test-how-far-does-the-live-feed-run-without-packet-loss) and
[`docs/link_range_test.md`](docs/link_range_test.md).

