"""
================================================================================
MODULE: video_feed_widget.py
PURPOSE: Low-Latency FPV Video Stream Ingestion Widget
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Video Tab)
  * Communicates:  VideoCaptureThread, OpenCV
  * Upstream:      Radxa d435i_video_streamer.py (MJPEG over HTTP, port 8080), USB
                    /dev/video* devices, RTSP/UDP streams, or Synthetic Pattern
  * Downstream:    Operator FPV situational awareness video display

DATA FLOW & INTERFACES:
  * Inputs:        Video frames (numpy BGR arrays) from camera devices or network URLs.
  * Outputs:       Rendered QPixmap video frame.
  * Qt Signals:    frame_ready(np.ndarray) emitted from threaded capture worker.

KEY LOGIC & FAILSAFES:
  * Dedicated Capture Thread: Isolates OpenCV `cap.read()` inside a non-blocking
    QThread to prevent video frame drops from lagging the GCS UI thread.
  * Auto-Reconnect: A dropped/never-opened stream is retried every 2s rather than
    latching into a permanent "offline" state - the Radxa side may still be booting
    ROS 2 nodes when the FPV tab is first opened.
  * Synthetic Test Pattern: Built-in 30 FPS simulated test generator allows bench
    testing and UI verification even when no physical camera is attached.
  * Qt Plugin Protection: Explicitly overrides OpenCV's internal Qt library hooks
    to prevent library pollution on Linux X11/Wayland sessions.

USAGE:
  video_widget = VideoFeedWidget()
  video_widget.ensure_started()          # auto-connects to the drone's MJPEG stream
================================================================================
"""

from __future__ import annotations
import math
import time
from typing import Optional

import os
import sys

# Ensure OpenCV does not hijack system Qt platform plugins
try:
    import cv2
except ImportError:
    cv2 = None

# Always enforce system Qt5 platform plugin path (overrides cv2's internal pollution)
if sys.platform.startswith("linux"):
    for _p in ["/usr/lib/aarch64-linux-gnu/qt5/plugins", "/usr/lib/x86_64-linux-gnu/qt5/plugins"]:
        if os.path.exists(_p):
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _p
            break
if "QT_QPA_PLATFORM" not in os.environ:
    os.environ["QT_QPA_PLATFORM"] = "xcb"

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QFrame, QSizePolicy, QLineEdit
)

DEFAULT_STREAM_URL = "http://172.16.101.84:8080/video"


class VideoCaptureThread(QThread):
    """Background video frame acquisition thread."""

    frame_ready = pyqtSignal(np.ndarray)

    def __init__(self, source=0):
        super().__init__()
        self.source = source
        self.running = False
        self.cap: Optional[cv2.VideoCapture] = None

    def set_source(self, source):
        self.source = source

    def run(self):
        self.running = True
        if self.source == "TEST_PATTERN":
            # Synthetic 30 FPS test generator
            t0 = time.time()
            while self.running:
                elapsed = time.time() - t0
                frame = np.zeros((360, 640, 3), dtype=np.uint8)
                # Grid background
                for x in range(0, 640, 40):
                    cv2.line(frame, (x, 0), (x, 360), (28, 33, 40), 1)
                for y in range(0, 360, 40):
                    cv2.line(frame, (0, y), (640, y), (28, 33, 40), 1)

                # Moving crosshairs & scan circle
                cx = 320 + int(math.sin(elapsed * 1.2) * 50)
                cy = 180 + int(math.cos(elapsed * 1.2) * 30)
                cv2.circle(frame, (cx, cy), 45, (0, 210, 255), 1)
                cv2.line(frame, (cx - 60, cy), (cx + 60, cy), (0, 210, 255), 1)
                cv2.line(frame, (cx, cy - 60), (cx, cy + 60), (0, 210, 255), 1)

                cv2.putText(
                    frame, "FPV FEED — SYNTHETIC TEST PATTERN", (140, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (201, 209, 217), 1, cv2.LINE_AA
                )
                cv2.putText(
                    frame, f"FPS: 30.0 | TIME: {elapsed:.1f}s", (220, 330),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (88, 166, 255), 1, cv2.LINE_AA
                )
                self.frame_ready.emit(frame)
                time.sleep(0.033)
            return

        # Real OpenCV capture, with reconnect-on-drop: the Radxa's video streamer node
        # may still be booting when this tab first opens, or Wi-Fi may briefly drop -
        # neither should latch this thread into a permanent "offline" state.
        last_open_attempt = 0.0
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                now = time.time()
                if now - last_open_attempt >= 2.0:
                    last_open_attempt = now
                    try:
                        cap = cv2.VideoCapture(self.source)
                        if isinstance(self.source, int):
                            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # low-latency: drop stale frames
                        self.cap = cap if cap.isOpened() else None
                        if cap is not self.cap:
                            cap.release()
                    except Exception:
                        self.cap = None

                if self.cap is None:
                    err_frame = np.zeros((360, 640, 3), dtype=np.uint8)
                    cv2.putText(
                        err_frame, f"CAMERA NOT DETECTED ({self.source})", (100, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (218, 54, 51), 1, cv2.LINE_AA
                    )
                    self.frame_ready.emit(err_frame)
                    time.sleep(0.2)
                    continue

            ret, frame = self.cap.read()
            if ret and frame is not None:
                self.frame_ready.emit(frame)
            else:
                # Stream dropped mid-read - release and let the top of the loop reconnect.
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
                time.sleep(0.05)

        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

    def stop(self):
        self.running = False
        self.wait(1000)


class VideoFeedWidget(QWidget):
    """Compound widget presenting live video viewport and camera controls."""

    SRC_DRONE_FPV, SRC_TEST_PATTERN, SRC_CAM0, SRC_CAM1, SRC_CUSTOM = range(5)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(540, 360)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        # Toolbar controls
        tb = QHBoxLayout()
        tb.setContentsMargins(8, 6, 8, 0)

        title = QLabel("FPV CAMERA & VISION FEED")
        title.setStyleSheet("font-weight: bold; color: #58a6ff; font-size: 11px;")
        tb.addWidget(title)

        tb.addSpacing(16)

        lbl_src = QLabel("Source:")
        lbl_src.setStyleSheet("color: #8b949e; font-size: 11px;")
        tb.addWidget(lbl_src)

        self.combo_source = QComboBox(self)
        self.combo_source.addItems([
            "Drone FPV (Wi-Fi MJPEG)", "Test Pattern",
            "Camera 0 (/dev/video0)", "Camera 1 (/dev/video1)", "Custom RTSP/HTTP URL",
        ])
        self.combo_source.currentIndexChanged.connect(self._on_source_changed)
        tb.addWidget(self.combo_source)

        self.txt_url = QLineEdit(DEFAULT_STREAM_URL, self)
        self.txt_url.setFixedWidth(200)
        self.txt_url.setToolTip("Radxa MJPEG stream URL (used for Drone FPV / Custom URL sources)")
        self.txt_url.editingFinished.connect(self._on_url_edited)
        tb.addWidget(self.txt_url)

        tb.addStretch()

        self.btn_capture = QPushButton("Start Video", self)
        self.btn_capture.clicked.connect(self._toggle_capture)
        tb.addWidget(self.btn_capture)

        layout.addLayout(tb)

        # Video Canvas Label
        self.video_label = QLabel("NO VIDEO FEED ACTIVE", self)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #090d12; border: 1px solid #30363d; border-radius: 6px; font-weight: bold; color: #8b949e;")
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.video_label, 1)

        # Capture thread
        self.cap_thread: Optional[VideoCaptureThread] = None

    # -------------------------------------------------------------------------
    # Public API (used by drone_gcs.py)
    # -------------------------------------------------------------------------

    def ensure_started(self):
        """Start the currently selected feed if nothing is running yet. Idempotent -
        safe to call every time the FPV tab is selected."""
        if self.cap_thread and self.cap_thread.isRunning():
            return
        self._start_capture()

    def set_stream_host(self, ip: str):
        """Re-target the drone stream URL to a new IP (e.g. a Network preset switch in
        the top strip), keeping the :8080/video port/path - those don't change with
        the network, only the host does. Reconnects live if a feed is already running
        off the drone source."""
        self.txt_url.setText(f"http://{ip}:8080/video")
        idx = self.combo_source.currentIndex()
        if idx in (self.SRC_DRONE_FPV, self.SRC_CUSTOM) and self.cap_thread and self.cap_thread.isRunning():
            self.cap_thread.stop()
            self.cap_thread.set_source(self._resolve_source(idx))
            self.cap_thread.start()

    # -------------------------------------------------------------------------
    # Source selection / capture control
    # -------------------------------------------------------------------------

    def _resolve_source(self, idx: int):
        if idx == self.SRC_DRONE_FPV:
            return self.txt_url.text().strip() or DEFAULT_STREAM_URL
        if idx == self.SRC_TEST_PATTERN:
            return "TEST_PATTERN"
        if idx == self.SRC_CAM0:
            return 0
        if idx == self.SRC_CAM1:
            return 1
        return self.txt_url.text().strip() or DEFAULT_STREAM_URL

    def _start_capture(self):
        src = self._resolve_source(self.combo_source.currentIndex())
        self.cap_thread = VideoCaptureThread(src)
        self.cap_thread.frame_ready.connect(self._on_frame_ready)
        self.cap_thread.start()
        self.btn_capture.setText("Stop Video")

    def _toggle_capture(self):
        if self.cap_thread and self.cap_thread.isRunning():
            self.cap_thread.stop()
            self.cap_thread = None
            self.btn_capture.setText("Start Video")
            self.video_label.setText("VIDEO FEED STOPPED")
        else:
            self._start_capture()

    def _on_source_changed(self, idx: int):
        if self.cap_thread and self.cap_thread.isRunning():
            self.cap_thread.stop()
            self.cap_thread.set_source(self._resolve_source(idx))
            self.cap_thread.start()

    def _on_url_edited(self):
        idx = self.combo_source.currentIndex()
        if idx in (self.SRC_DRONE_FPV, self.SRC_CUSTOM) and self.cap_thread and self.cap_thread.isRunning():
            self.cap_thread.stop()
            self.cap_thread.set_source(self._resolve_source(idx))
            self.cap_thread.start()

    # -------------------------------------------------------------------------
    # Frame rendering
    # -------------------------------------------------------------------------

    def _on_frame_ready(self, frame: np.ndarray):
        """Convert BGR OpenCV frame to QPixmap and display."""
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

        lbl_w = max(320, self.video_label.width())
        lbl_h = max(180, self.video_label.height())
        scaled_pixmap = QPixmap.fromImage(qimg).scaled(lbl_w, lbl_h, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        self.video_label.setPixmap(scaled_pixmap)

    def closeEvent(self, event):
        if self.cap_thread:
            self.cap_thread.stop()
        super().closeEvent(event)
