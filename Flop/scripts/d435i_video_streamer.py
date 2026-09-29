#!/usr/bin/env python3
"""
================================================================================
MODULE: d435i_video_streamer.py
PURPOSE: Compresses & Streams D435i RGB Camera Frames from Radxa to Laptop GCS
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Radxa Q6A Companion Computer (Onboard System)
  * Communicates:  Laptop GCS over Private Wi-Fi Access Point (HTTP Port 8080)
  * Upstream:      realsense2_camera_node (/camera/color/image_raw)
  * Downstream:    Laptop GCS video_feed_widget.py (VideoCaptureThread)

DATA FLOW & INTERFACES:
  * Subscribes To: /camera/color/image_raw [sensor_msgs/Image] (raw rgb8/bgr8, 640x480@30)
  * HTTP Server:   0.0.0.0:8080/video - multipart/x-mixed-replace MJPEG stream,
                    readable directly by a browser or by cv2.VideoCapture(url).
  * Wire Payload:  One JPEG per multipart chunk (~15-30 KB), no custom framing needed -
                    HTTP/TCP already guarantees ordered, lossless delivery.

WHY JPEG OVER WI-FI. Raw 640x480x3 @ 30 FPS is ~221 Mbit/s - enough to choke a shared
AP and start dropping MAVLink packets. JPEG at quality 80 drops a frame to roughly
15-25 KB, i.e. ~4-6 Mbit/s at 30 FPS, with no visible quality loss for FPV situational
awareness. Depth is intentionally NOT enabled or streamed here - FPV needs color only,
and leaving depth off preserves USB bandwidth/CPU for the stereo VIO + SLAM pipeline.

KEY LOGIC & FAILSAFES:
  * Silent-When-Idle Encoding: Frames are still received off the ROS 2 topic, but the
    JPEG encode only needs to happen once per frame regardless of client count - the
    HTTP server fans the same encoded bytes out to every connected client.
  * Non-Blocking Broadcast: A dropped/slow HTTP client (Wi-Fi hiccup) cannot block the
    ROS 2 executor - the encode step and the per-client write loops run independently,
    coordinated by a condition variable (FrameHub).
  * Multi-Client: Multiple GCS laptops (or a browser tab for bench debugging) can watch
    the same stream concurrently.

RUN:
  source /opt/ros/jazzy/setup.bash
  python3 d435i_video_streamer.py --ros-args -p port:=8080 -p jpeg_quality:=80
================================================================================
"""

from __future__ import annotations
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

# Ensure ROS 2 Jazzy paths are in sys.path
for _ros_path in [
    "/opt/ros/jazzy/lib/python3.12/site-packages",
    "/opt/ros/jazzy/lib/python3.10/site-packages",
]:
    if os.path.exists(_ros_path) and _ros_path not in sys.path:
        sys.path.insert(0, _ros_path)

import numpy as np
import cv2

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
except ImportError:
    Node = object
    qos_profile_sensor_data = None
    Image = None

BOUNDARY = b"FRAME"


class FrameHub:
    """Holds the latest encoded JPEG and wakes any HTTP client threads waiting on it."""

    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._frame_id = 0

    def publish(self, jpeg_bytes: bytes):
        with self._cond:
            self._jpeg = jpeg_bytes
            self._frame_id += 1
            self._cond.notify_all()

    def wait_next(self, last_id: int, timeout: float = 2.0):
        """Block until a frame newer than last_id exists, or timeout. Returns (jpeg, frame_id)."""
        with self._cond:
            if self._frame_id == last_id:
                self._cond.wait(timeout)
            return self._jpeg, self._frame_id


def make_handler(hub: FrameHub, log):
    class MJPEGHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self):
            if self.path.rstrip("/") not in ("/video", ""):
                self.send_response(404)
                self.end_headers()
                return

            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY.decode()}")
            self.end_headers()

            last_id = 0
            try:
                while True:
                    jpeg, last_id = hub.wait_next(last_id)
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--" + BOUNDARY + b"\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        def log_message(self, fmt, *args):
            pass  # suppress default stderr access log; the node logs connect/disconnect state instead

    return MJPEGHandler


class D435iVideoStreamer(Node):
    def __init__(self):
        super().__init__("d435i_video_streamer")

        self.declare_parameter("port", 8080)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("topic", "/camera/color/image_raw")
        self.declare_parameter("stats_interval", 5.0)

        self.port = int(self.get_parameter("port").value)
        self.quality = int(self.get_parameter("jpeg_quality").value)
        self.topic = str(self.get_parameter("topic").value)
        self.stats_interval = float(self.get_parameter("stats_interval").value)

        self.hub = FrameHub()
        self._warned_encoding = False
        self._frames_at_stats = 0
        self._bytes_at_stats = 0
        self._stats_at = time.time()

        self.sub = self.create_subscription(Image, self.topic, self._on_image, qos_profile_sensor_data)

        handler_cls = make_handler(self.hub, self.get_logger())
        self.server = ThreadingHTTPServer(("0.0.0.0", self.port), handler_cls)
        self.server.daemon_threads = True
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f"D435i JPEG video streamer active on http://0.0.0.0:{self.port}/video "
            f"(source: {self.topic}, quality={self.quality})"
        )

    def _on_image(self, msg: Image):
        if msg.encoding not in ("rgb8", "bgr8"):
            if not self._warned_encoding:
                self.get_logger().error(
                    f"unsupported image encoding '{msg.encoding}' - expected rgb8/bgr8"
                )
                self._warned_encoding = True
            return

        # Reshape via msg.step (row stride) rather than width*3, to tolerate any row padding.
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        frame = arr[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            frame = frame[:, :, ::-1]  # RGB -> BGR (cv2.imencode expects BGR channel order)

        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return
        jpeg_bytes = buf.tobytes()
        self.hub.publish(jpeg_bytes)

        self._frames_at_stats += 1
        self._bytes_at_stats += len(jpeg_bytes)
        now = time.time()
        if self.stats_interval and now - self._stats_at >= self.stats_interval:
            dt = now - self._stats_at
            self.get_logger().info(
                "%.1f fps  %.1f kB/frame  %.2f Mbit/s"
                % (
                    self._frames_at_stats / dt,
                    (self._bytes_at_stats / max(1, self._frames_at_stats)) / 1e3,
                    self._bytes_at_stats * 8 / dt / 1e6,
                )
            )
            self._stats_at, self._frames_at_stats, self._bytes_at_stats = now, 0, 0

    def destroy_node(self):
        try:
            self.server.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = D435iVideoStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
