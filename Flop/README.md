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
