#!/bin/bash
# Intel RealSense D435I color sensor (USB3) -> Iris hardware H.264 -> RTP -> OpenHD external camera input.
#
# This is the boot-enabled camera script (ohd-q6a-cam.service). It used to run the
# IMX415 CSI camera via libcamerasrc; that pipeline is preserved, unused, at
# ohd-q6a-cam-imx415.sh.bak in this same directory. Swapped to the D435i because
# that's the camera actually wired to this board now - libcamerasrc found no CSI sensor.
#
# D435i notes:
#  - the Iris encoder (v4l2h264enc) hard-rejects any declared input framerate above 8fps -
#    confirmed directly: a synthetic 30fps NV12 source fails the same "not-negotiated" error
#    with NO camera involved at all, while the same source at 8fps negotiates fine.
#  - the honest fix is to genuinely capture at REAL_FPS<=8 (6fps here, the D435i's highest
#    supported rate under that ceiling). This version instead reproduces the same trick the
#    IMX415/libcamerasrc script uses: v4l2src genuinely captures the D435i's real ~30fps (a
#    rate the sensor actually supports), then `capssetter` rewrites the caps right before the
#    encoder to falsely declare DECLARED_FPS (<=8), satisfying its negotiation check, while
#    the real buffers keep flowing underneath at the real rate - confirmed by direct timing
#    test (60 buffers captured in ~2.5s, i.e. ~24fps actual throughput, not 6).
#  - bitrate math stays keyed to the REAL frame rate (not the declared lie): the Iris rate
#    control budgets bits-per-frame, so VIDEO_BITRATE = TARGET_BPS / REAL_FPS regardless of
#    what DECLARED_FPS says.
#  - the periodic ~20s "no frame" stalls seen while testing this camera were traced to the
#    D435i running over a USB2 (480Mbit) port instead of its dedicated USB3 SuperSpeed port -
#    fixed by physically reconnecting it to the USB3 port, unrelated to this fps trick.
#  - the D435i's /dev/videoN node numbers (depth/IR/color/metadata) are not guaranteed stable
#    across reboots or replugs, so DEVICE is auto-detected as the first RealSense video node
#    that offers YUYV (confirmed by hand to be the color sensor), unless overridden.
WIDTH=${WIDTH:-1280}
HEIGHT=${HEIGHT:-720}
REAL_FPS=${REAL_FPS:-30}
DECLARED_FPS=${DECLARED_FPS:-6}               # must be in the encoder's supported (<=8) set
TARGET_BPS=${TARGET_BPS:-4000000}             # what actually goes on air
VIDEO_BITRATE=$(( TARGET_BPS / REAL_FPS ))    # what Iris must be told (bits-per-frame budget)
GOP=${GOP:-$REAL_FPS}                          # one keyframe per second at the real frame rate

find_realsense_color_node() {
  local dev
  for dev in $(v4l2-ctl --list-devices 2>/dev/null | sed -n '/RealSense/,/^$/p' | grep -o '/dev/video[0-9]*'); do
    if v4l2-ctl -d "$dev" --list-formats 2>/dev/null | grep -q YUYV; then
      echo "$dev"
      return 0
    fi
  done
  return 1
}

DEVICE=${DEVICE:-$(find_realsense_color_node)}
if [ -z "$DEVICE" ]; then
  echo "ohd-q6a-cam-d435i: could not find the D435i color (YUYV) video node; is the camera plugged in? Set DEVICE=/dev/videoN to override." >&2
  exit 1
fi

exec gst-launch-1.0 -e \
  v4l2src device="${DEVICE}" \
  ! "video/x-raw,format=YUY2,width=${WIDTH},height=${HEIGHT},framerate=${REAL_FPS}/1" \
  ! videoconvert ! "video/x-raw,format=NV12" \
  ! capssetter caps="video/x-raw,format=(string)NV12,framerate=(fraction)${DECLARED_FPS}/1" \
  ! v4l2h264enc extra-controls="controls,video_bitrate=${VIDEO_BITRATE},video_peak_bitrate=${VIDEO_BITRATE},video_bitrate_mode=1,video_gop_size=${GOP},prepend_sps_and_pps_to_idr=1" \
  ! h264parse config-interval=-1 \
  ! rtph264pay pt=96 mtu=1400 config-interval=-1 \
  ! udpsink host=127.0.0.1 port=5500 sync=false
