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
from PyQt5.QtCore import QRectF, QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QFrame, QSizePolicy, QLineEdit, QGridLayout
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
from ui.theme import GlassPanel, GlassBar
from ui.fpv_hud import FpvVideoLabel
from core.video_recorder import VideoRecorder, save_snapshot
from core.wfb_video import WfbVideoBridge
from core.video_health import VideoHealthMonitor, IDLE, CONNECTING, DEGRADED, FROZEN, NO_SIGNAL, LIVE

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
        self._pending_since = 0.0
        self.frames_skipped = 0

    def set_source(self, source):
        self.source = source

    # Newest-frame-only hand-off to the GUI thread. The loop below reads as fast
    # as the stream delivers; if the GUI has not yet consumed the previous frame
    # (it was busy, or a Wi-Fi stall just ended and a burst of old frames arrived
    # at once) the newer frame REPLACES the wait instead of queueing behind it.
    # Otherwise every queued signal holds a full decoded frame and the picture
    # runs further behind real time the longer the GUI lags.
    PENDING_TIMEOUT_S = 0.5      # a GUI that never answers must not freeze the feed

    def frame_consumed(self) -> None:
        """Called by the GUI slot once it has finished with the last frame."""
        self._pending_since = 0.0

    def _emit_real_frame(self, frame) -> bool:
        """Emit info+frame unless the GUI is still busy with the previous one.
        Returns True if emitted, False if this frame was skipped as stale."""
        now = time.monotonic()
        if self._pending_since and now - self._pending_since < self.PENDING_TIMEOUT_S:
            self.frames_skipped += 1
            return False
        self._pending_since = now
        self.frame_info.emit(now, True)
        self.frame_ready.emit(frame)
        return True

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
                self._emit_real_frame(frame)
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
        # v2: radial dark well, no border, quiet spaced caption.
        self.setStyleSheet(
            "background-color: qradialgradient(cx:0.5, cy:0.4, radius:1.0, fx:0.5, fy:0.4,"
            " stop:0 #171b19, stop:1 #070908); border: none;"
            " border-bottom-left-radius: 9px; border-bottom-right-radius: 9px;"
            " color: #6f7772; font-size: 11px; font-weight: 500; letter-spacing: 1.2px;")
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
            "QFrame { background-color: #161918; border: 1px solid rgba(255,255,255,41);"
            " border-radius: 10px; }")
        sl = QVBoxLayout(shell)
        sl.setContentsMargins(6, 4, 6, 6)
        sl.setSpacing(4)

        bar = QHBoxLayout()
        bar.setSpacing(6)
        title = QLabel("FPV", self)
        title.setStyleSheet(
            "color: #46d06a; font-size: 9px; font-weight: bold;"
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
            "QPushButton:hover { color: #46d06a; }")
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


class FeedGraph(QWidget):
    """Sparkline of the feed's frame rate and pipeline latency over the last
    minute - the shape of a problem (a slow decay vs sudden stalls) that the
    single live number on the health line cannot show. Grey lines; no colour."""

    WINDOW_S = 60.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(px(76))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.fps: list = []
        self.lat: list = []

    def push(self, fps: float, latency_ms: Optional[float], period_s: float = 0.5) -> None:
        keep = int(self.WINDOW_S / period_s)
        self.fps = (self.fps + [float(fps)])[-keep:]
        self.lat = (self.lat + [latency_ms])[-keep:]
        self.update()

    def clear(self) -> None:
        self.fps, self.lat = [], []
        self.update()

    def _line(self, p, rect, vals, top, colour):
        pts = [(i, v) for i, v in enumerate(vals) if v is not None]
        if len(pts) < 2:
            return
        n = max(len(vals) - 1, 1)
        path = None
        from PyQt5.QtGui import QPainterPath
        for i, v in pts:
            x = rect.left() + rect.width() * i / n
            y = rect.bottom() - rect.height() * min(max(v / top, 0.0), 1.0)
            if path is None:
                path = QPainterPath()
                path.moveTo(x, y)
            else:
                path.lineTo(x, y)
        p.setPen(QPen(colour, 1.5))
        p.drawPath(path)

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#0d1117"))
        rect = QRectF(self.rect()).adjusted(px(8), px(20), -px(8), -px(6))
        p.setPen(QPen(QColor("#262c34"), 1))
        p.drawLine(rect.bottomLeft(), rect.bottomRight())
        p.drawLine(rect.topLeft(), rect.topRight())
        top_fps = max(30.0, max(self.fps or [0.0]))
        lats = [v for v in self.lat if v is not None]
        top_lat = max(100.0, max(lats or [0.0]))
        self._line(p, rect, self.lat, top_lat, QColor("#6e7681"))
        self._line(p, rect, self.fps, top_fps, QColor("#58a6ff"))
        p.setFont(QFont("Ubuntu", 8))
        p.setPen(QColor("#8b949e"))
        p.drawText(QRectF(px(8), px(3), self.width() - px(16), px(14)), Qt.AlignLeft | Qt.AlignVCenter,
                   f"fps (blue, 0-{top_fps:.0f})   latency (grey, 0-{top_lat:.0f} ms)   last 60 s")
        p.end()


def _capture_option(key: str, default: str) -> str:
    """One value out of NETWORK_CAPTURE_OPTIONS ('k;v|k;v' form)."""
    for part in NETWORK_CAPTURE_OPTIONS.split("|"):
        k, _, v = part.partition(";")
        if k == key:
            return v
    return default


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
    # SRC_WFB is appended last so the existing indices (and anything persisted
    # against them) keep their meaning.
    SRC_DRONE_FPV, SRC_CAM0, SRC_CAM1, SRC_CUSTOM, SRC_WFB = range(5)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(px(380), px(300))

        # The picture is a full-bleed layer, not a child of the left column: while this
        # widget is standalone it fills the widget; the main window re-parents it behind
        # the whole shell (ShellRoot.set_bleed) so it runs under every glass panel.
        self.video_label = FpvVideoLabel(self)
        self.video_label.setGeometry(self.rect())
        self.video_label.lower()

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        left = QWidget(self)
        left.setFixedWidth(px(340))
        col = QVBoxLayout(left)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(px(8))
        root.addWidget(left)
        root.addStretch(1)

        def row(label: str, widget: QWidget, parent_panel) -> QHBoxLayout:
            r = QHBoxLayout()
            r.setSpacing(px(10))
            lbl = QLabel(label, parent_panel)
            lbl.setObjectName("fieldLabel")
            lbl.setFixedWidth(px(118))
            r.addWidget(lbl)
            r.addWidget(widget, 1)
            return r

        def readonly(text: str, tip: str, num: bool = False) -> QLabel:
            f = QLabel(text, self)
            f.setObjectName("fpvField")
            f.setProperty("num", num)
            f.setToolTip(tip)
            f.setAlignment((Qt.AlignRight if num else Qt.AlignLeft) | Qt.AlignVCenter)
            return f

        # ---- STREAM SOURCE ----
        pnl_src = GlassPanel("Stream source", "", True, left)
        sb = pnl_src.body_layout
        sb.setSpacing(px(9))

        self.combo_source = QComboBox(self)
        _source_items = [
            "Drone FPV (Wi-Fi MJPEG)",
            "Camera 0 (/dev/video0)", "Camera 1 (/dev/video1)", "Custom RTSP/HTTP URL",
            "Drone FPV (wfb-ng radio)",
        ]
        self.combo_source.addItems(_source_items)
        self.combo_source.setMinimumContentsLength(14)
        self.combo_source.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLength)
        self.combo_source.currentIndexChanged.connect(self._on_source_changed)
        sb.addLayout(row("Source", self.combo_source, pnl_src))

        # Two remembered URLs, not one shared field. The drone feed follows the
        # Network preset; the custom one is whatever the operator typed and must
        # survive a preset change.
        self._drone_url = DEFAULT_STREAM_URL
        self._custom_url = "rtsp://192.168.1.10:554/stream"
        # wfb-ng radio feed: RTP H.264 from the radio's ground side, re-wrapped for
        # OpenCV by a helper process that only runs while this source is selected.
        self.wfb = WfbVideoBridge()

        self.txt_url = QLineEdit(self._drone_url, self)
        self.txt_url.setMinimumWidth(px(120))
        self.txt_url.setToolTip(
            "Stream URL for the selected source.\n"
            "Drone FPV: MJPEG over HTTP, follows the Network preset.\n"
            "Custom: any rtsp:// rtsps:// udp:// rtp:// or http:// URL - RTSP\n"
            "is opened over TCP with a 5 s timeout.")
        self.txt_url.editingFinished.connect(self._on_url_edited)
        # Show the start of a long URL, not its tail, whenever the field is not being edited.
        self.txt_url.textChanged.connect(
            lambda _t: None if self.txt_url.hasFocus() else self.txt_url.setCursorPosition(0))
        sb.addLayout(row("URL", self.txt_url, pnl_src))

        # Read-only mirrors of what the capture actually uses (settings live in Configuration).
        cfg_tip = "Read-only here - change it in Configuration \u2192 Video"
        self.lbl_transport = readonly(_capture_option("rtsp_transport", "tcp"), cfg_tip)
        sb.addLayout(row("Transport", self.lbl_transport, pnl_src))
        _to = _capture_option("timeout", "5000000")
        try:
            _to = f"{int(_to) // 1_000_000}"
        except ValueError:
            pass
        self.lbl_timeout = readonly(f"{_to} s", cfg_tip, True)
        sb.addLayout(row("Timeout", self.lbl_timeout, pnl_src))
        self.lbl_wfb_rtp = readonly(str(self.wfb.rtp_port), cfg_tip, True)
        sb.addLayout(row("WFB RTP port", self.lbl_wfb_rtp, pnl_src))
        self.lbl_wfb_ts = readonly(str(self.wfb.ts_port), cfg_tip, True)
        sb.addLayout(row("WFB TS port", self.lbl_wfb_ts, pnl_src))

        sb.addSpacing(px(2))
        br = QHBoxLayout()
        br.setSpacing(px(8))
        self.btn_start = QPushButton("START", self)
        self.btn_start.setProperty("role", "go")
        self.btn_start.setMinimumHeight(px(32))
        self.btn_start.clicked.connect(self._start_if_stopped)
        self.btn_stop = QPushButton("STOP", self)
        self.btn_stop.setMinimumHeight(px(32))
        self.btn_stop.clicked.connect(self._stop_if_running)
        br.addWidget(self.btn_start, 1)
        br.addWidget(self.btn_stop, 1)
        sb.addLayout(br)
        col.addWidget(pnl_src)

        # Kept for API compatibility (the old single toggle); the design's START / STOP replace it.
        self.btn_capture = QPushButton("Start Video", self)
        self.btn_capture.clicked.connect(self._toggle_capture)
        self.btn_capture.setVisible(False)

        # ---- TOOLS ----
        pnl_tools = GlassPanel("Tools", "", False, left)
        tb = pnl_tools.body_layout
        tb.setSpacing(px(8))
        grid = QGridLayout()
        grid.setHorizontalSpacing(px(8))
        grid.setVerticalSpacing(px(8))

        self.btn_snapshot = QPushButton("SNAPSHOT", self)
        self.btn_snapshot.setToolTip("Save the current frame as a PNG in ~/.drone_gcs/video/")
        self.btn_snapshot.clicked.connect(self.take_snapshot)
        self.btn_record = QPushButton("RECORD", self)
        self.btn_record.setToolTip("Record the feed to an MJPEG .avi in ~/.drone_gcs/video/.\n"
                                   "Raw frames only - overlays are never burned in.")
        self.btn_record.clicked.connect(self.toggle_recording)
        # Icon-free text button, like the rest of the design (the old glyph-only form is gone).
        self.btn_fullscreen = QPushButton("FULLSCREEN", self)
        self.btn_fullscreen.setToolTip("Expand this feed to fill the whole screen (Esc to exit)")
        self.btn_fullscreen.clicked.connect(self.fullscreen_requested)
        self.btn_overlay = QPushButton("OVERLAY", self)
        self.btn_overlay.setCheckable(True)
        self.btn_overlay.setToolTip("Show the altitude / speed / heading HUD over the picture on screen only")
        for b in (self.btn_snapshot, self.btn_record, self.btn_fullscreen, self.btn_overlay):
            b.setMinimumHeight(px(32))
        grid.addWidget(self.btn_snapshot, 0, 0)
        grid.addWidget(self.btn_record, 0, 1)
        grid.addWidget(self.btn_fullscreen, 1, 0)
        grid.addWidget(self.btn_overlay, 1, 1)
        tb.addLayout(grid)
        col.addWidget(pnl_tools)

        # Slot the main window mounts the Vision-engines panel into.
        self.vision_slot = QVBoxLayout()
        self.vision_slot.setContentsMargins(0, 0, 0, 0)
        self.vision_slot.setSpacing(0)
        col.addLayout(self.vision_slot)

        # ---- feed health strip (health line, REC indicator, notices, graph) ----
        self.pnl_health = GlassPanel("Feed", "", False, left)
        hb = self.pnl_health.body_layout
        hb.setContentsMargins(px(14), px(8), px(14), px(10))
        hb.setSpacing(px(6))
        hrow = QHBoxLayout()
        hrow.setSpacing(px(8))
        # Feed health line. Neutral grey while healthy; amber/red only when the
        # feed is actually degraded or dead (see _refresh_health).
        self.lbl_health = QLabel("IDLE", self)
        self.lbl_health.setObjectName("fpvHealth")
        self.lbl_health.setToolTip(
            "Feed health. 'pipeline' is capture-thread read -> pixel painted inside\n"
            "this station only; it excludes camera, encoder and network delay, so it\n"
            "is a floor and an early warning, not true glass-to-glass latency.")
        self.lbl_health.setWordWrap(True)
        hrow.addWidget(self.lbl_health, 1)
        self.lbl_rec = QLabel("", self)
        self.lbl_rec.setObjectName("fpvRec")
        hrow.addWidget(self.lbl_rec)
        self.btn_graph = QPushButton("GRAPH", self)
        self.btn_graph.setCheckable(True)
        self.btn_graph.setToolTip("Frame rate and latency over the last minute")
        self.btn_graph.toggled.connect(self._on_graph_toggled)
        hrow.addWidget(self.btn_graph)
        hb.addLayout(hrow)
        self.lbl_note = QLabel("", self)
        self.lbl_note.setObjectName("fpvNote")
        self.lbl_note.setWordWrap(True)
        self.lbl_note.setVisible(False)
        hb.addWidget(self.lbl_note)
        self.graph = FeedGraph(self)
        self.graph.setVisible(False)
        hb.addWidget(self.graph)
        col.addWidget(self.pnl_health)
        col.addStretch(1)

        self.recorder = VideoRecorder()
        self._last_frame = None
        self.btn_overlay.toggled.connect(self._on_overlay_toggled)
        self.btn_overlay.setChecked(True)      # the design shows the HUD on
        self._frame_size = (0, 0)
        self._telemetry = None
        self._rec_timer = QTimer(self)
        self._rec_timer.timeout.connect(self._update_rec_label)

        self.health = VideoHealthMonitor()
        self._last_info: Tuple[Optional[float], bool] = (None, True)
        self._last_health_state = IDLE
        # Capture thread (before the first health refresh, which reads it)
        self.cap_thread: Optional[VideoCaptureThread] = None
        self._health_timer = QTimer(self)
        self._health_timer.timeout.connect(self._refresh_health)
        self._health_timer.start(500)
        self._refresh_health()

    # Layer plumbing --------------------------------------------------------
    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        if self.video_label.parent() is self:
            self.video_label.setGeometry(self.rect())
            self.video_label.lower()

    def showEvent(self, ev):
        super().showEvent(ev)
        # Re-parented behind the shell, the picture only shows while this workspace does.
        if self.video_label.parent() is not self:
            self.video_label.show()

    def hideEvent(self, ev):
        super().hideEvent(ev)
        if self.video_label.parent() is not self:
            self.video_label.hide()

    def _on_overlay_toggled(self, on: bool) -> None:
        self.video_label.hud.set_readout_visible(on)
        self._redraw_last()

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
        if idx == self.SRC_WFB:
            try:
                return self.wfb.start()
            except RuntimeError as e:
                # Shown in the feed as "CAMERA NOT DETECTED (...)" - better than a silent black box.
                return f"wfb-ng unavailable: {e}"
        return self.txt_url.text().strip() or self._custom_url

    def select_source(self, idx: int) -> None:
        """Switch the feed source programmatically (e.g. the wfb-ng network preset)."""
        if self.combo_source.currentIndex() != idx:
            self.combo_source.setCurrentIndex(idx)

    def set_wfb_ports(self, rtp_port: int, ts_port: int) -> None:
        """Apply video.wfb_rtp_port / wfb_ts_port; takes effect on the next start."""
        if (rtp_port, ts_port) != (self.wfb.rtp_port, self.wfb.ts_port):
            self.wfb.stop()
            self.wfb.rtp_port, self.wfb.ts_port = rtp_port, ts_port
            self.lbl_wfb_rtp.setText(str(rtp_port))
            self.lbl_wfb_ts.setText(str(ts_port))
            if self.combo_source.currentIndex() == self.SRC_WFB:
                self.txt_url.setText(self.wfb.url)

    def _start_capture(self):
        src = self._resolve_source(self.combo_source.currentIndex())
        self.cap_thread = VideoCaptureThread(src)
        self.cap_thread.frame_info.connect(self._on_frame_info)
        self.cap_thread.frame_ready.connect(self._on_frame_ready)
        self.health.start()
        self.cap_thread.start()
        self.btn_capture.setText("Stop Video")
        self._sync_run_buttons()

    def _running(self) -> bool:
        return bool(self.cap_thread and self.cap_thread.isRunning())

    def _sync_run_buttons(self) -> None:
        running = self._running()
        self.btn_start.setEnabled(not running)
        self.btn_stop.setEnabled(running)

    def _start_if_stopped(self):
        if not self._running():
            self._start_capture()

    def _stop_if_running(self):
        if self._running():
            self._toggle_capture()

    def _toggle_capture(self):
        if self.cap_thread and self.cap_thread.isRunning():
            self.cap_thread.stop()
            self.cap_thread = None
            self.wfb.stop()
            self.health.stop()
            self._refresh_health()
            self.btn_capture.setText("Start Video")
            # The label is a full-window layer now: clear the picture and say so in the
            # notice line rather than centring a caption over the whole screen.
            self.video_label.setPixmap(QPixmap())
            self.video_label.set_live(False)
            self._notice("Video feed stopped")
        else:
            self._start_capture()
        self._sync_run_buttons()

    def _on_source_changed(self, idx: int):
        # Swap the field to the URL that belongs to the newly selected source.
        if idx == self.SRC_DRONE_FPV:
            self.txt_url.setText(self._drone_url)
        elif idx == self.SRC_CUSTOM:
            self.txt_url.setText(self._custom_url)
        elif idx == self.SRC_WFB:
            self.txt_url.setText(self.wfb.url)   # informational: the local re-wrapped stream
        self.txt_url.setEnabled(idx in (self.SRC_DRONE_FPV, self.SRC_CUSTOM))

        if self.cap_thread and self.cap_thread.isRunning():
            self.cap_thread.stop()
            if idx != self.SRC_WFB:
                self.wfb.stop()
            self.cap_thread.set_source(self._resolve_source(idx))
            self.cap_thread.start()
        elif idx != self.SRC_WFB:
            self.wfb.stop()

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
        colour = "#ff5a4f" if bad else "#ffb02e" if warn else "#a3aaa5"
        self.lbl_health.setStyleSheet(f"color: {colour}; font-size: 11px;")
        self.lbl_health.setText(h.text())
        self._update_status_chip(h, bad, warn)
        self._sync_run_buttons()
        if self.graph.isVisibleTo(self):
            self.graph.push(h.fps, h.latency_ms)
        if h.state != self._last_health_state:
            self._last_health_state = h.state
            self.health_changed.emit(h.state, h.text())

    def _update_status_chip(self, h, bad: bool, warn: bool) -> None:
        """The HUD's top-right chip: "LIVE · 30 FPS · 1080p", or the health state."""
        if h.state == LIVE:
            fh = self._frame_size[1]
            text = f"LIVE \u00b7 {h.fps:.0f} FPS" + (f" \u00b7 {fh}p" if fh else "")
            self.video_label.hud.set_status(text, "ok")
        elif bad:
            self.video_label.hud.set_status(h.state, "bad")
        elif warn:
            self.video_label.hud.set_status(h.state, "warn")
        else:
            self.video_label.hud.set_status("NO FEED" if h.state == IDLE else h.state, "")

    def _on_frame_ready(self, frame: np.ndarray):
        """Convert BGR OpenCV frame to QPixmap and display."""
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

        lbl_w = max(320, self.video_label.width())
        lbl_h = max(180, self.video_label.height())
        scaled_pixmap = QPixmap.fromImage(qimg).scaled(lbl_w, lbl_h, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        self._frame_size = (w, h)
        captured_at, real = self._last_info
        if real:
            self._last_frame = frame
            if self.recorder.recording:
                self.recorder.write(frame)
        self._show_pixmap(scaled_pixmap)
        self.health.on_frame(captured_at, real)
        # Mirror to every other viewport on the same decoded frame.
        self.frame_broadcast.emit(frame)
        if self.cap_thread is not None:
            self.cap_thread.frame_consumed()

    # -------------------------------------------------------------------------
    # Snapshot / recording / overlay / graph
    # -------------------------------------------------------------------------

    def set_telemetry(self, t) -> None:
        """Latest telemetry, for the optional on-screen overlay (never recorded)."""
        self._telemetry = t
        self.video_label.hud.set_telemetry(t)

    def overlay_lines(self) -> list:
        t = self._telemetry
        if t is None:
            return []
        return [
            f"{getattr(t, 'flight_mode', '--')}  {'ARMED' if getattr(t, 'armed', False) else 'DISARMED'}",
            f"ALT {getattr(t, 'altitude', 0.0):.2f} m   SPD {getattr(t, 'ground_speed', 0.0):.1f} m/s",
            f"BATT {getattr(t, 'battery_percent', 0)}%   {getattr(t, 'battery_voltage', 0.0):.1f} V",
        ]

    def _show_pixmap(self, pm: QPixmap) -> None:
        # The on-screen overlay is the HUD widget (ui/fpv_hud.py); the picture itself is
        # never drawn on, so what is recorded and snapshotted is always the raw frame.
        self.video_label.setPixmap(pm)
        self.video_label.set_live(True)

    def _redraw_last(self) -> None:
        if self._last_frame is not None:
            self._on_frame_ready_display_only(self._last_frame)

    def _on_frame_ready_display_only(self, frame: np.ndarray) -> None:
        h, w, ch = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pm = QPixmap.fromImage(qimg).scaled(max(320, self.video_label.width()),
                                            max(180, self.video_label.height()),
                                            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self._show_pixmap(pm)

    def take_snapshot(self):
        """Save the last real frame as a PNG. Returns the path, or None."""
        path = save_snapshot(self._last_frame)
        self._notice("Snapshot: no live frame to save" if path is None else f"Snapshot saved: {path}")
        return path

    def toggle_recording(self):
        if self.recorder.recording:
            self.stop_recording()
        else:
            self.recorder.start()
            self.btn_record.setText("STOP REC")
            self._rec_timer.start(500)
            self._update_rec_label()

    def stop_recording(self):
        path = self.recorder.stop()
        self._rec_timer.stop()
        self.btn_record.setText("RECORD")
        self.lbl_rec.setText("")
        self._notice(f"Recording saved: {path}" if path else "Recording stopped: no frames received")
        return path

    def _notice(self, text: str) -> None:
        """A line under the picture for a few seconds (the health line is rewritten
        twice a second, so it cannot carry a confirmation)."""
        self.lbl_note.setText(text)
        self.lbl_note.setVisible(True)

        def clear():
            if self.lbl_note.text() == text:
                self.lbl_note.setText("")
                self.lbl_note.setVisible(False)
        QTimer.singleShot(6000, clear)

    def _update_rec_label(self):
        e = int(self.recorder.elapsed)
        self.lbl_rec.setText(f"● REC {e // 60:02d}:{e % 60:02d}")

    def _on_graph_toggled(self, on: bool):
        self.graph.setVisible(on)
        if on:
            self.graph.clear()

    def closeEvent(self, event):
        if self.recorder.recording:
            self.recorder.stop()
        if self.cap_thread:
            self.cap_thread.stop()
        self.wfb.stop()
        super().closeEvent(event)
