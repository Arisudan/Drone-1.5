"""
================================================================================
MODULE: docked_feed.py
PURPOSE: The FPV camera feed, docked as the last glass panel of the Tactical SLAM column
================================================================================

WHY: the feed used to float over the map's top-left corner, covering map data and needing to be dragged out of the
way. Docked here it never hides the map and always sits in the same place. It keeps the floating window's size
(328 x 220) exactly - the panel was widened to hold it, the picture was not shrunk.

THE HEADER STRIP: the panel's fold/unfold arrow lives in this tile's header, before the "FPV" title.
So the arrow never disappears, the tile shrinks to just that header strip when the panel is folded
away (arrow only) or when the camera is popped out (arrow, "FPV" and a button to dock it back).

NO NEW VIDEO LOAD: like every other viewport this is a passive VideoSink on the frames the single capture thread
already decoded. The floating window is still there as an option: POP OUT hands the feed to it (and hides this
tile); closing the floating window docks the feed again.
================================================================================
"""

from __future__ import annotations

import time

from PyQt5.QtCore import QPointF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QPushButton, QSizePolicy

from ui.scaling import px
from ui.theme import GlassPanel
from ui.video_feed_widget import VideoSink

FEED_W, FEED_H = 328, 220        # kept for callers: the old tile size (the panel now follows its column)
STRIP_H = 34                     # header-only strip (camera popped out)
QWIDGETSIZE_MAX = (1 << 24) - 1


class Chevron(QPushButton):
    """Fold / unfold arrow. Not part of the v2 design (the column no longer folds), so it is kept
    as a hidden, working control for callers and tests."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.folded = False
        self.setFixedSize(px(22), px(22))
        self.setStyleSheet("QPushButton { min-height: 0px; padding: 0px; border: none; background: transparent; }")
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.NoFocus)
        self._hover = False
        self.set_folded(False)

    def set_folded(self, folded: bool) -> None:
        self.folded = bool(folded)
        self.setToolTip("Show the camera picture" if folded else "Fold the camera to its header")
        self.update()

    def enterEvent(self, _e):
        self._hover = True
        self.update()

    def leaveEvent(self, _e):
        self._hover = False
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        pen = QPen(QColor("#e9ece9" if self._hover else "#a3aaa5"), max(1.6, px(1.6)))
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        p.setPen(pen)
        cx, cy = self.width() / 2.0, self.height() / 2.0
        h, w = px(4), px(2.6)
        d = -1 if self.folded else 1
        for off in (-w, w):
            x = cx + off
            p.drawPolyline(QPolygonF([QPointF(x - d * w * 0.9, cy - h), QPointF(x + d * w * 0.9, cy),
                                      QPointF(x - d * w * 0.9, cy + h)]))
        p.end()


class _FeedSink(VideoSink):
    """A passive sink that also remembers when a frame last arrived (for the header caption)."""

    def __init__(self, placeholder: str, parent=None):
        super().__init__(placeholder, parent)
        self.last_frame_at = 0.0

    def on_frame(self, frame):
        self.last_frame_at = time.monotonic()
        super().on_frame(frame)

    def clear_feed(self):
        self.last_frame_at = 0.0
        super().clear_feed()


class DockedFeed(GlassPanel):
    """The FPV panel: glass header (FPV / live-or-no-feed + pop-out / fullscreen) over the picture."""

    fullscreen_requested = pyqtSignal()
    popout_requested = pyqtSignal()
    dock_requested = pyqtSignal()          # bring a popped-out camera back into the panel
    collapse_requested = pyqtSignal()      # the (hidden) arrow was clicked

    def __init__(self, parent=None):
        super().__init__("FPV", "no feed", False, parent)
        self.setObjectName("dockedFeed")
        self._docked, self._folded, self._compact = True, False, False
        self.lbl_title = self.lbl_title                    # "FPV"
        self.lbl_floating = self.lbl_sub                   # right-hand caption: no feed / live / floating

        hl = self.header.layout()
        self.btn_collapse = Chevron(self.header)
        self.btn_collapse.clicked.connect(self.collapse_requested)
        self.btn_collapse.setVisible(False)
        hl.insertWidget(0, self.btn_collapse)
        for attr, glyph, tip, sig in (
                ("btn_dock", "\u2199", "Dock the camera back into the panel", self.dock_requested),
                ("btn_popout", "\u2197", "Pop out into a movable floating window", self.popout_requested),
                ("btn_full", "\u26f6", "Fullscreen (Esc to exit)", self.fullscreen_requested)):
            b = QPushButton(glyph, self.header)
            b.setObjectName("feedIconBtn")
            b.setFixedSize(px(20), px(20))
            b.setToolTip(tip)
            b.clicked.connect(sig)
            setattr(self, attr, b)
            hl.addWidget(b)

        bl = self.body_layout
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(0)
        self.sink = _FeedSink("FPV FEED \u2014 OPEN CAMERA TAB OR CONNECT", self.body)
        self.sink.setMinimumSize(px(120), px(48))
        bl.addWidget(self.sink, 1)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

        self._caption_timer = QTimer(self)
        self._caption_timer.timeout.connect(self._refresh_caption)
        self._caption_timer.start(1000)
        self.set_mode(True, False)

    def _refresh_caption(self) -> None:
        if not self._docked:
            self.lbl_sub.setText("floating")
            return
        live = self.sink.last_frame_at > 0 and (time.monotonic() - self.sink.last_frame_at) < 2.0
        self.lbl_sub.setText("live" if live else "no feed")

    def set_mode(self, docked: bool, folded: bool, compact: bool = False) -> None:
        """full panel (docked) | header strip with the dock-back button (camera popped out) |
        header strip (folded)."""
        self._docked, self._folded, self._compact = bool(docked), bool(folded), bool(compact)
        full = self._docked and not self._folded
        self.body.setVisible(full)
        self.sink.setVisible(full)
        self.btn_popout.setVisible(self._docked)
        self.btn_full.setVisible(self._docked)
        self.btn_dock.setVisible(not self._docked)
        self.btn_collapse.set_folded(self._folded)
        self._refresh_caption()
        if full:
            self.setMinimumHeight(px(90))
            self.setMaximumHeight(QWIDGETSIZE_MAX)
        else:
            self.setMinimumHeight(px(STRIP_H))
            self.setMaximumHeight(px(STRIP_H))

    def is_compact(self) -> bool:
        return self._compact

    def strip_height(self) -> int:
        return px(STRIP_H)
