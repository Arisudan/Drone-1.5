"""
================================================================================
MODULE: status_dot.py
PURPOSE: A status dot that is always a true circle
================================================================================

Status dots used to be a small QLabel styled "background: <colour>; border-radius: Npx". Whether Qt
draws that as a circle depends on the platform, the style and the display scale, and on some screens
it showed up as a square. This widget paints its own antialiased circle with QPainter, so it is round
at every size and scale, and the colour is one plain value (set_colour / colour).
================================================================================
"""

from __future__ import annotations

from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import QWidget

from ui.scaling import px


class StatusDot(QWidget):
    def __init__(self, size_px: int = 10, colour: str = "#6e7681", parent=None, ring: str = ""):
        super().__init__(parent)
        self.setFixedSize(px(size_px), px(size_px))
        self._colour = colour
        self._ring = ring          # optional thin outline (e.g. dark, so a dot stays visible on a picture)

    def set_colour(self, colour: str) -> None:
        if colour != self._colour:
            self._colour = colour
            self.update()

    def colour(self) -> str:
        return self._colour

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        inset = 0.5
        rect = QRectF(inset, inset, self.width() - 2 * inset, self.height() - 2 * inset)
        if self._ring:
            p.setPen(QPen(QColor(self._ring), 1))
        else:
            p.setPen(Qt.NoPen)
        p.setBrush(QColor(self._colour))
        p.drawEllipse(rect)
        p.end()
