"""
================================================================================
MODULE: docked_feed.py
PURPOSE: The FPV camera feed, docked at the top of the Tactical SLAM side panel
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

from PyQt5.QtCore import QPointF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout

from ui.scaling import px
from ui.video_feed_widget import VideoSink

FEED_W, FEED_H = 328, 220        # the floating window's size, unchanged
STRIP_H = 32                     # header-only strip (panel folded, or camera popped out)


class Chevron(QPushButton):
    """A small flat button with a crisp, drawn chevron: "»" when the panel can be folded away
    (pointing the way it goes), "«" when it is folded and can be brought back."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.folded = False
        self.setFixedSize(px(22), px(22))
        # The app-wide button style carries a minimum height and padding that would make this taller than 22 px
        # (it stuck out of the header strip at large UI scale); it paints itself, so none of that applies.
        self.setStyleSheet("QPushButton { min-height: 0px; padding: 0px; border: none; background: transparent; }")
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.NoFocus)
        self._hover = False
        self.set_folded(False)

    def set_folded(self, folded: bool) -> None:
        self.folded = bool(folded)
        self.setToolTip("Show the side panel" if folded else "Hide the side panel (keeps the path actions)")
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
        r = px(6)
        if self._hover:
            p.setPen(QPen(QColor("#30363d"), 1))
            p.setBrush(QColor(255, 255, 255, 14))
            p.drawRoundedRect(self.rect().adjusted(0, 0, -1, -1), r, r)
        pen = QPen(QColor("#c9d1d9" if self._hover else "#8b949e"), max(1.6, px(1.6)))
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        cx, cy = self.width() / 2.0, self.height() / 2.0
        h, w = px(4), px(2.6)
        d = -1 if self.folded else 1                 # folded: points left, unfolded: points right
        for off in (-w, w):                          # two chevrons: "»" / "«"
            x = cx + off
            p.drawPolyline(QPolygonF([QPointF(x - d * w * 0.9, cy - h), QPointF(x + d * w * 0.9, cy),
                                      QPointF(x - d * w * 0.9, cy + h)]))
        p.end()


class DockedFeed(QFrame):
    fullscreen_requested = pyqtSignal()
    popout_requested = pyqtSignal()
    dock_requested = pyqtSignal()          # bring a popped-out camera back into the panel
    collapse_requested = pyqtSignal()      # the arrow was clicked

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("dockedFeed")
        self.setStyleSheet(
            "QFrame#dockedFeed { background-color: #161b22; border: 1px solid #30363d; border-radius: 6px; }")
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self._docked, self._folded, self._compact = True, False, False

        sl = QVBoxLayout(self)
        sl.setContentsMargins(6, 4, 6, 6)
        sl.setSpacing(4)

        bar = QHBoxLayout()
        bar.setSpacing(6)
        self._bar = bar
        self.btn_collapse = Chevron(self)
        self.btn_collapse.clicked.connect(self.collapse_requested)
        bar.addWidget(self.btn_collapse)
        self.lbl_title = QLabel("FPV", self)
        self.lbl_title.setStyleSheet("color: #58a6ff; font-size: 9px; font-weight: bold; letter-spacing: 1.6px;"
                                     " background: transparent; border: none;")
        bar.addWidget(self.lbl_title)
        self.lbl_floating = QLabel("floating", self)
        self.lbl_floating.setStyleSheet("color: #6e7681; font-size: 9px; background: transparent; border: none;")
        bar.addWidget(self.lbl_floating)
        bar.addStretch(1)
        for attr, glyph, tip, sig in (
                ("btn_dock", "\u2199", "Dock the camera back into the panel", self.dock_requested),
                ("btn_popout", "\u2197", "Pop out into a movable floating window", self.popout_requested),
                ("btn_full", "\u26f6", "Fullscreen (Esc to exit)", self.fullscreen_requested)):
            b = QPushButton(glyph, self)
            b.setFixedSize(px(18), px(18))
            b.setToolTip(tip)
            b.setStyleSheet("QPushButton { background: transparent; border: none; color: #8b949e;"
                            " font-size: 12px; font-weight: bold; padding: 0; min-height: 0; }"
                            "QPushButton:hover { color: #58a6ff; }")
            b.clicked.connect(sig)
            setattr(self, attr, b)
            bar.addWidget(b)
        sl.addLayout(bar)

        self.sink = VideoSink("FPV - NO FEED", self)
        sl.addWidget(self.sink, 1)
        self.set_mode(True, False)

    def set_mode(self, docked: bool, folded: bool, compact: bool = False) -> None:
        """full tile (docked, panel open) | header strip with the dock-back button (camera popped out) |
        arrow only (panel folded away) | header strip with the usual buttons (docked, but the panel is too short
        for the picture: never squeeze the controls below it)."""
        self._docked, self._folded, self._compact = bool(docked), bool(folded), bool(compact)
        full = self._docked and not self._folded and not self._compact
        strip_with_buttons = self._docked and not self._folded and self._compact
        self.sink.setVisible(full)
        self.btn_popout.setVisible(full or strip_with_buttons)
        self.btn_full.setVisible(full or strip_with_buttons)
        self.btn_dock.setVisible(not self._docked and not self._folded)
        self.lbl_title.setVisible(not self._folded)
        self.lbl_floating.setVisible(not self._docked and not self._folded)
        self.btn_collapse.set_folded(self._folded)
        if full:
            self.setFixedSize(FEED_W, FEED_H)                    # unscaled, like the floating window it replaces
        elif self._folded:
            self.setFixedSize(px(34), px(STRIP_H))               # just the arrow
        else:
            self.setFixedSize(FEED_W, px(STRIP_H))               # arrow, "FPV", buttons

    def is_compact(self) -> bool:
        return self._compact

    def strip_height(self) -> int:
        return px(STRIP_H)
