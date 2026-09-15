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
