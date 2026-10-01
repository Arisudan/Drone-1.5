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
from typing import Optional, Tuple

import os

from qt_env import pin_system_qt_plugins

# Ensure OpenCV does not hijack system Qt platform plugins
try:
    import cv2
except ImportError:
    cv2 = None

# Re-pin the system Qt5 plugin path over cv2's (only for the distro PyQt5)
pin_system_qt_plugins()
if "QT_QPA_PLATFORM" not in os.environ:
    os.environ["QT_QPA_PLATFORM"] = "xcb"

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QFrame, QSizePolicy, QLineEdit
)

DEFAULT_STREAM_URL = "http://172.16.101.89:8080/video"


#: FFmpeg options applied to network streams before the capture is opened.
#: - rtsp_transport=tcp: UDP RTSP loses packets over Wi-Fi and FFmpeg has no
#:   way to recover them, which shows up as smeared frames rather than a clean
#:   dropout. TCP costs a little latency and removes that failure mode.
#: - timeout: FFmpeg's default is *no* timeout. Measured against a host that
#:   drops packets rather than refusing (drone powered down, wrong subnet,
#:   Wi-Fi gone - i.e. the field case), cv2.VideoCapture() blocked for 30 s
#:   before giving up, turning this widget's 2 s reconnect cadence into a 30 s
#:   one. With this set it gives up in 5 s, measured on the same host.
#:   The option is `timeout` (microseconds) on current FFmpeg; `stimeout` is
#:   the pre-5.x spelling and does nothing on a modern build - both are listed
#:   so this works either side of that rename. Verified: `stimeout` alone left
#:   the 30 s block in place.
#: - max_delay / fflags=nobuffer: keep the decoder from accumulating a backlog,
#:   which is what CAP_PROP_BUFFERSIZE is meant to do but does not on the
#:   FFmpeg backend (it is honoured by V4L2 and a few others, not this one).
NETWORK_CAPTURE_OPTIONS = (
    "rtsp_transport;tcp|timeout;5000000|stimeout;5000000"
    "|max_delay;500000|fflags;nobuffer"
)
from ui.scaling import px, fit_min_width
from core.video_health import VideoHealthMonitor, IDLE, CONNECTING, DEGRADED, FROZEN, NO_SIGNAL

#: URL schemes that go through FFmpeg and therefore want the options above.
NETWORK_SCHEMES = ("rtsp://", "rtsps://", "udp://", "rtp://", "rtmp://", "tcp://")


def is_network_stream(source) -> bool:
    """True for a URL FFmpeg will open over the network.

    HTTP is excluded deliberately: the drone's own MJPEG feed is plain HTTP and
    works well without RTSP-specific tuning, and stimeout would change its
    reconnect behaviour for no benefit.
    """
    return isinstance(source, str) and source.lower().startswith(NETWORK_SCHEMES)


class VideoCaptureThread(QThread):
    """Background video frame acquisition thread."""

    frame_ready = pyqtSignal(np.ndarray)
    # (monotonic capture time, is_real). Emitted immediately BEFORE each
    # frame_ready - same-sender queued signals keep their order, so the slot
    # always pairs a frame with its own info. is_real=False marks the
    # "camera not detected" placeholder, which must not count as live video.
    frame_info = pyqtSignal(float, bool)

    def __init__(self, source=0):
        super().__init__()
        self.source = source
        self.running = False
        self.cap: Optional[cv2.VideoCapture] = None

    def set_source(self, source):
        self.source = source

    def _apply_capture_options(self):
        """Set FFmpeg options for the upcoming open, if this is a live stream.

        OpenCV reads OPENCV_FFMPEG_CAPTURE_OPTIONS from the environment at
        VideoCapture construction, so it has to be set here rather than once at
        import - the source can change at runtime.
        """
        if is_network_stream(self.source):
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = NETWORK_CAPTURE_OPTIONS
        else:
            os.environ.pop("OPENCV_FFMPEG_CAPTURE_OPTIONS", None)

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
                self.frame_info.emit(time.monotonic(), True)
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
                        self._apply_capture_options()
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
                    self.frame_info.emit(time.monotonic(), False)
                    self.frame_ready.emit(err_frame)
                    time.sleep(0.2)
                    continue

            ret, frame = self.cap.read()
            if ret and frame is not None:
                self.frame_info.emit(time.monotonic(), True)
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


def bgr_to_pixmap(frame: np.ndarray, w: int, h: int) -> QPixmap:
    """BGR ndarray -> QPixmap scaled into (w, h), aspect preserved."""
    fh, fw, ch = frame.shape
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    qimg = QImage(rgb.data, fw, fh, ch * fw, QImage.Format_RGB888)
    return QPixmap.fromImage(qimg).scaled(
        max(32, w), max(32, h), Qt.KeepAspectRatio, Qt.SmoothTransformation)


class VideoSink(QLabel):
    """A passive viewport onto the FPV stream.

    Several places want to see the camera at once - the cockpit, the SLAM
    overlay, the FPV tab - but the Radxa's MJPEG stream costs ~8 Mbit/s and a
    second HTTP connection would double that for no new information. So there
    is exactly one capture thread (owned by VideoFeedWidget) and any number of
    these sinks subscribing to the frames it already decoded.
    """

    def __init__(self, placeholder: str = "NO VIDEO FEED", parent=None):
        super().__init__(parent)
        self._placeholder = placeholder
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(px(160), px(90))
        self.setStyleSheet(
            "background-color: #090d12; border: 1px solid #30363d;"
            "border-radius: 6px; color: #6e7681; font-size: 10px;"
            "font-weight: bold; letter-spacing: 1px;")
        self.setText(placeholder)

    def on_frame(self, frame: np.ndarray):
        self.setPixmap(bgr_to_pixmap(frame, self.width(), self.height()))

    def clear_feed(self):
        self.setPixmap(QPixmap())
        self.setText(self._placeholder)


class FullscreenVideoWindow(QWidget):
    """A real top-level fullscreen viewport onto the FPV stream.

    Shared by every camera-feed pane in the app (the FPV tab, the Cockpit
    HUD) rather than one fullscreen window per pane - they already all show
    the same broadcast frames (see VideoSink's own docstring on why there is
    only one capture thread), so a second instance would add nothing but
    another idle decode-free copy of the same pixmap. Esc, or a click
    anywhere on the feed, exits back to the normal window.
    """

    closed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle("Drone FPV - Fullscreen")
        self.setStyleSheet("background-color: #000000;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.sink = VideoSink("NO VIDEO FEED", self)
        self.sink.setStyleSheet(
            "background-color: #000000; border: none; color: #6e7681;"
            " font-size: 14px; font-weight: bold; letter-spacing: 1px;")
        layout.addWidget(self.sink)

        self._hint = QLabel(
            "Press Esc or click anywhere to exit fullscreen", self.sink)
        self._hint.setStyleSheet(
            "color: rgba(255, 255, 255, 200); background: rgba(0, 0, 0, 140);"
            " font-size: 11px; font-weight: bold; padding: 5px 12px;"
            " border-radius: 4px;")
        self._hint_timer = QTimer(self)
        self._hint_timer.setSingleShot(True)
        self._hint_timer.timeout.connect(self._hint.hide)

    def open(self) -> None:
        """Enter fullscreen, re-showing the exit hint for a few seconds."""
        self._hint.adjustSize()
        self._hint.move(px(16), px(16))
        self._hint.show()
        self._hint_timer.start(3000)
        self.showFullScreen()
        self.raise_()
        self.activateWindow()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self._exit_fullscreen()
            return
        super().keyPressEvent(event)

    def mousePressEvent(self, event):
        self._exit_fullscreen()

    def _exit_fullscreen(self) -> None:
        self.showNormal()
        self.hide()
        self.closed.emit()


class FloatingVideoWindow(QWidget):
    """Frameless always-on-top FPV window, draggable anywhere on the desktop.

    A child overlay could only travel inside the GCS window; a tool window can
    be parked on a second monitor or beside the map, which is the point of
    having it while flying a route on the SLAM view.
    """

    closed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent, Qt.Tool | Qt.FramelessWindowHint |
                         Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_TranslucentBackground, False)
        self.resize(328, 220)
        self._drag_offset = None
        self._pre_fullscreen_geometry = None

        root = QVBoxLayout(self)
        root.setContentsMargins(1, 1, 1, 1)
        root.setSpacing(0)

        shell = QFrame(self)
        shell.setStyleSheet(
            "QFrame { background-color: #161b22; border: 1px solid #30363d;"
            " border-radius: 6px; }")
        sl = QVBoxLayout(shell)
        sl.setContentsMargins(6, 4, 6, 6)
        sl.setSpacing(4)

        bar = QHBoxLayout()
        bar.setSpacing(6)
        title = QLabel("FPV", self)
        title.setStyleSheet(
            "color: #58a6ff; font-size: 9px; font-weight: bold;"
            " letter-spacing: 1.6px; background: transparent;"
            " font-family: 'Ubuntu', 'Ubuntu Sans', 'Noto Sans', sans-serif;")
        bar.addWidget(title)
        hint = QLabel("drag to move", self)
        hint.setStyleSheet(
            "color: #6e7681; font-size: 8px; background: transparent;")
        bar.addWidget(hint)
        bar.addStretch()
        btn_full = QPushButton("\u26f6", self)
        btn_full.setFixedSize(px(18), px(18))
        btn_full.setToolTip("Fullscreen (Esc to exit)")
        btn_full.setStyleSheet(
            "QPushButton { background: transparent; border: none; color: #8b949e;"
            " font-size: 12px; font-weight: bold; padding: 0; min-height: 0; }"
            "QPushButton:hover { color: #58a6ff; }")
        btn_full.clicked.connect(self._toggle_fullscreen)
        bar.addWidget(btn_full)
        btn_close = QPushButton("\u00d7", self)
        btn_close.setFixedSize(px(18), px(18))
        btn_close.setStyleSheet(
            "QPushButton { background: transparent; border: none; color: #8b949e;"
            " font-size: 14px; font-weight: bold; padding: 0; min-height: 0; }"
            "QPushButton:hover { color: #f85149; }")
        btn_close.clicked.connect(self._on_close_clicked)
        bar.addWidget(btn_close)
        sl.addLayout(bar)

        self.sink = VideoSink("FPV - NO FEED", self)
        sl.addWidget(self.sink, 1)
        root.addWidget(shell)

    def _on_close_clicked(self):
        self.hide()
        self.closed.emit()

    def _toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self._exit_fullscreen()
        else:
            self._pre_fullscreen_geometry = self.geometry()
            self.showFullScreen()

    def _exit_fullscreen(self) -> None:
        self.showNormal()
        if self._pre_fullscreen_geometry is not None:
            self.setGeometry(self._pre_fullscreen_geometry)
            self._pre_fullscreen_geometry = None

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape and self.isFullScreen():
            self._exit_fullscreen()
            return
        super().keyPressEvent(event)

    # Frameless windows have no system title bar, so dragging is ours to do.
    # Not while fullscreen - there is nowhere on screen left to drag it to.
    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton and not self.isFullScreen():
            self._drag_offset = ev.globalPos() - self.frameGeometry().topLeft()
            ev.accept()

    def mouseMoveEvent(self, ev):
        if self._drag_offset is not None and ev.buttons() & Qt.LeftButton:
            self.move(ev.globalPos() - self._drag_offset)
            ev.accept()

    def mouseReleaseEvent(self, ev):
        self._drag_offset = None


class VideoFeedWidget(QWidget):
    """Compound widget presenting live video viewport and camera controls.

    Owns the single capture thread and re-emits its frames on
    ``frame_broadcast`` so other views (cockpit, SLAM overlay) can render the
    same stream without opening their own connection to the Radxa.
    """

    # Re-emitted decoded frames, for any VideoSink that wants to mirror them.
    frame_broadcast = pyqtSignal(object)
    fullscreen_requested = pyqtSignal()
    # (state, one-line text) - emitted only when the health STATE changes, so a
    # listener (the alarm list) is not woken twice a second by a steady feed.
    health_changed = pyqtSignal(str, str)

    # The synthetic test pattern was dropped from the dropdown - it is a
    # bench aid, not a source anyone selects in flight. VideoCaptureThread
    # still understands the "TEST_PATTERN" source string, so it remains
    # available to tests and offline development.
    SRC_DRONE_FPV, SRC_CAM0, SRC_CAM1, SRC_CUSTOM = range(4)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(px(540), px(360))
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        # Toolbar controls
        tb = QHBoxLayout()
        tb.setContentsMargins(8, 6, 8, 0)

        title = QLabel("FPV CAMERA & VISION FEED")
        title.setObjectName("pageTitle")
        tb.addWidget(title)

        tb.addSpacing(16)

        lbl_src = QLabel("Source:")
        lbl_src.setStyleSheet("color: #8b949e; font-size: 11px;")
        tb.addWidget(lbl_src)

        self.combo_source = QComboBox(self)
        _source_items = [
            "Drone FPV (Wi-Fi MJPEG)",
            "Camera 0 (/dev/video0)", "Camera 1 (/dev/video1)", "Custom RTSP/HTTP URL",
        ]
        self.combo_source.addItems(_source_items)
        fit_min_width(self.combo_source, _source_items, h_pad_px=46)
        self.combo_source.currentIndexChanged.connect(self._on_source_changed)
        tb.addWidget(self.combo_source)

        # Two remembered URLs, not one shared field. The drone feed follows the
        # Network preset; the custom one is whatever the operator typed and must
        # survive a preset change.
        self._drone_url = DEFAULT_STREAM_URL
        self._custom_url = "rtsp://192.168.1.10:554/stream"

        self.txt_url = QLineEdit(self._drone_url, self)
        self.txt_url.setMinimumWidth(px(180))
        self.txt_url.setToolTip(
            "Stream URL for the selected source.\n"
            "Drone FPV: MJPEG over HTTP, follows the Network preset.\n"
            "Custom: any rtsp:// rtsps:// udp:// rtp:// or http:// URL - RTSP\n"
            "is opened over TCP with a 5 s timeout.")
        self.txt_url.editingFinished.connect(self._on_url_edited)
        tb.addWidget(self.txt_url)

        tb.addStretch()

        self.btn_capture = QPushButton("Start Video", self)
        self.btn_capture.clicked.connect(self._toggle_capture)
        tb.addWidget(self.btn_capture)

        # Icon-only, matching the fullscreen buttons on the other camera
        # panes (HUD, floating window) - the text form ("Fullscreen") was the
        # difference between this toolbar fitting and overlapping the URL
        # field at this station's narrowest supported size.
        self.btn_fullscreen = QPushButton("⛶", self)
        self.btn_fullscreen.setToolTip("Expand this feed to fill the whole screen (Esc to exit)")
        self.btn_fullscreen.clicked.connect(self.fullscreen_requested)
        tb.addWidget(self.btn_fullscreen)

        layout.addLayout(tb)

        # Video Canvas Label
        self.video_label = QLabel("NO VIDEO FEED ACTIVE", self)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #090d12; border: 1px solid #30363d; border-radius: 6px; font-weight: bold; color: #8b949e;")
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.video_label, 1)

        # Feed health line. Neutral grey while healthy; amber/red only when the
        # feed is actually degraded or dead (see _refresh_health).
        self.lbl_health = QLabel("IDLE", self)
        self.lbl_health.setToolTip(
            "Feed health. 'pipeline' is capture-thread read -> pixel painted inside\n"
            "this station only; it excludes camera, encoder and network delay, so it\n"
            "is a floor and an early warning, not true glass-to-glass latency.")
        self.lbl_health.setContentsMargins(8, 0, 8, 4)
        layout.addWidget(self.lbl_health)

        self.health = VideoHealthMonitor()
        self._last_info: Tuple[Optional[float], bool] = (None, True)
        self._last_health_state = IDLE
        self._health_timer = QTimer(self)
        self._health_timer.timeout.connect(self._refresh_health)
        self._health_timer.start(500)
        self._refresh_health()

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
        """Re-target the drone's MJPEG URL to a new IP (a Network preset switch
        in the top strip), keeping the :8080/video port and path - those do not
        change with the network, only the host does.

        Only the drone feed is re-targeted. This used to rewrite whichever URL
        happened to be in the field, so switching network preset silently
        destroyed a hand-typed RTSP address and reconnected to MJPEG; the custom
        URL is remembered separately now and restored when you switch back.
        """
        self._drone_url = f"http://{ip}:8080/video"
        idx = self.combo_source.currentIndex()
        if idx == self.SRC_DRONE_FPV:
            self.txt_url.setText(self._drone_url)
        if idx == self.SRC_DRONE_FPV and self.cap_thread and self.cap_thread.isRunning():
            self.cap_thread.stop()
            self.cap_thread.set_source(self._resolve_source(idx))
            self.cap_thread.start()

    # -------------------------------------------------------------------------
    # Source selection / capture control
    # -------------------------------------------------------------------------

    def _resolve_source(self, idx: int):
        if idx == self.SRC_DRONE_FPV:
            return self.txt_url.text().strip() or self._drone_url or DEFAULT_STREAM_URL
        if idx == self.SRC_CAM0:
            return 0
        if idx == self.SRC_CAM1:
            return 1
        return self.txt_url.text().strip() or self._custom_url

    def _start_capture(self):
        src = self._resolve_source(self.combo_source.currentIndex())
        self.cap_thread = VideoCaptureThread(src)
        self.cap_thread.frame_info.connect(self._on_frame_info)
        self.cap_thread.frame_ready.connect(self._on_frame_ready)
        self.health.start()
        self.cap_thread.start()
        self.btn_capture.setText("Stop Video")

    def _toggle_capture(self):
        if self.cap_thread and self.cap_thread.isRunning():
            self.cap_thread.stop()
            self.cap_thread = None
            self.health.stop()
            self._refresh_health()
            self.btn_capture.setText("Start Video")
            self.video_label.setText("VIDEO FEED STOPPED")
        else:
            self._start_capture()

    def _on_source_changed(self, idx: int):
        # Swap the field to the URL that belongs to the newly selected source.
        if idx == self.SRC_DRONE_FPV:
            self.txt_url.setText(self._drone_url)
        elif idx == self.SRC_CUSTOM:
            self.txt_url.setText(self._custom_url)
        self.txt_url.setEnabled(idx in (self.SRC_DRONE_FPV, self.SRC_CUSTOM))

        if self.cap_thread and self.cap_thread.isRunning():
            self.cap_thread.stop()
            self.cap_thread.set_source(self._resolve_source(idx))
            self.cap_thread.start()

    def _on_url_edited(self):
        idx = self.combo_source.currentIndex()
        # Remember the edit against whichever source it belongs to.
        text = self.txt_url.text().strip()
        if idx == self.SRC_DRONE_FPV:
            self._drone_url = text
        elif idx == self.SRC_CUSTOM:
            self._custom_url = text

        if idx in (self.SRC_DRONE_FPV, self.SRC_CUSTOM) and self.cap_thread and self.cap_thread.isRunning():
            self.cap_thread.stop()
            self.cap_thread.set_source(self._resolve_source(idx))
            self.cap_thread.start()

    # -------------------------------------------------------------------------
    # Frame rendering
    # -------------------------------------------------------------------------

    def _on_frame_info(self, captured_at: float, real: bool):
        self._last_info = (captured_at, real)

    def _refresh_health(self):
        h = self.health.snapshot()
        bad = h.state in (FROZEN, NO_SIGNAL)
        warn = h.state in (DEGRADED, CONNECTING)
        colour = "#da3633" if bad else "#d29922" if warn else "#8b949e"
        self.lbl_health.setStyleSheet(f"color: {colour}; font-size: 11px;")
        self.lbl_health.setText(h.text())
        if h.state != self._last_health_state:
            self._last_health_state = h.state
            self.health_changed.emit(h.state, h.text())

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
        captured_at, real = self._last_info
        self.health.on_frame(captured_at, real)
        # Mirror to every other viewport on the same decoded frame.
        self.frame_broadcast.emit(frame)

    def closeEvent(self, event):
        if self.cap_thread:
            self.cap_thread.stop()
        super().closeEvent(event)
