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
- Full details, history, and troubleshooting log: see [`gotalldone.md`](gotalldone.md).

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
- Full details, history, and troubleshooting log: see [`gotalldone.md`](gotalldone.md).
