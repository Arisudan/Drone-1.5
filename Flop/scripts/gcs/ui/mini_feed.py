"""
================================================================================
MODULE: mini_feed.py
PURPOSE: Small always-visible camera thumbnail for the navigation rail
================================================================================

The cockpit, FPV and SLAM workspaces already show the camera. The rest
(Motor Actuators, Diagnostics, Terminal, Logs, Configuration, Parameters) did
not, so the operator lost the drone's view exactly when checking or changing
things on a live vehicle. This thumbnail fills that gap from the rail's
otherwise empty space.

IT ADDS NO LOAD:
  It is a passive subscriber to frames the one capture thread already decoded
  (see VideoSink). Frames are dropped, before any conversion, while the
  thumbnail is hidden, and thinned to ~10 fps while shown - a 160 px preview
  gains nothing from 30.

IT MUST NEVER SHOW A DEAD FRAME AS LIVE:
  A frozen feed leaves its last image on screen, indistinguishable from a still
  scene. When the shared video health says FROZEN / NO SIGNAL / IDLE the image
  is cleared to a placeholder, and a status dot says which.
================================================================================
"""

from __future__ import annotations

import time

import numpy as np
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout

from core.video_health import CONNECTING, DEGRADED, FROZEN, IDLE, LIVE, NO_SIGNAL
from ui.scaling import px
from ui.video_feed_widget import VideoSink

MIN_FRAME_INTERVAL_S = 0.1     # ~10 fps
ASPECT = 9 / 16

_DOT = {LIVE: "#3fb950", DEGRADED: "#d29922", CONNECTING: "#d29922",
        FROZEN: "#f85149", NO_SIGNAL: "#f85149", IDLE: "#6e7681"}
_WORD = {LIVE: "LIVE", DEGRADED: "SLOW", CONNECTING: "CONNECTING",
         FROZEN: "FROZEN", NO_SIGNAL: "NO SIGNAL", IDLE: "NO VIDEO"}


class _ClickableSink(VideoSink):
    clicked = pyqtSignal()

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(ev)


class MiniFeed(QFrame):
    fullscreen_requested = pyqtSignal()

    def __init__(self, parent=None, clock=time.monotonic):
        super().__init__(parent)
        self._clock = clock
        self._last_paint = 0.0
        self._state = IDLE
        self.setObjectName("miniFeed")

        lay = QVBoxLayout(self)
        lay.setContentsMargins(px(10), px(6), px(10), px(2))
        lay.setSpacing(px(3))

        self.sink = _ClickableSink("NO VIDEO", self)
        self.sink.setMinimumSize(px(64), px(36))     # override VideoSink's 160x90
        self.sink.setCursor(Qt.PointingHandCursor)
        self.sink.setToolTip("Click to expand the camera feed to fullscreen")
        self.sink.clicked.connect(self.fullscreen_requested)
        lay.addWidget(self.sink)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(px(6))
        self.dot = QLabel(self)
        self.dot.setFixedSize(px(8), px(8))
        row.addWidget(self.dot, 0, Qt.AlignVCenter)
        self.lbl_state = QLabel("", self)
        row.addWidget(self.lbl_state, 1)
        lay.addLayout(row)

        self.set_health(IDLE, "")

    # ── height follows width so the picture stays 16:9 ─────────────────────
    def feed_height(self, width: int) -> int:
        return max(px(36), int(width * ASPECT))

    def wanted_height(self) -> int:
        """Height this widget needs inside a rail of the current width."""
        m = self.layout().contentsMargins()
        inner_w = max(px(64), (self.parentWidget().width() if self.parentWidget() else px(176))
                      - m.left() - m.right())
        return (m.top() + self.feed_height(inner_w) + self.layout().spacing()
                + px(14) + m.bottom())

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        m = self.layout().contentsMargins()
        self.sink.setFixedHeight(self.feed_height(self.width() - m.left() - m.right()))

    # ── data in ─────────────────────────────────────────────────────────────
    def on_frame(self, frame: np.ndarray) -> None:
        if not self.isVisible() or self._state in (FROZEN, NO_SIGNAL, IDLE):
            return
        now = self._clock()
        if now - self._last_paint < MIN_FRAME_INTERVAL_S:
            return
        self._last_paint = now
        self.sink.on_frame(frame)

    def set_health(self, state: str, _text: str) -> None:
        self._state = state
        self.dot.setStyleSheet(
            f"background: {_DOT.get(state, '#6e7681')}; border-radius: {px(4)}px;")
        self.lbl_state.setText(_WORD.get(state, state))
        self.lbl_state.setStyleSheet(
            f"color: {'#8b949e' if state in (LIVE, IDLE) else _DOT.get(state, '#8b949e')};"
            f" font-size: {px(10)}px; letter-spacing: 1px; font-weight: 600;")
        if state in (FROZEN, NO_SIGNAL, IDLE):
            self.sink.clear_feed()
