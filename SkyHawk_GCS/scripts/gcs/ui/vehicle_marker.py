"""Centre-of-screen vehicle marker for the Control workspace (design: green quad glyph,
dashed range ring, heading chevron and a name tag). Purely a view of the heading."""

from __future__ import annotations

import math
from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import QWidget

from ui.scaling import px


class VehicleMarker(QWidget):
    NAME = "HTIC-QUAD"

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self._heading = 0.0

    def set_heading(self, deg: float) -> None:
        if abs(deg - self._heading) > 0.5:
            self._heading = deg
            self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c = QPointF(self.width() / 2.0, self.height() * 0.46)
        k = px(1)
        p.translate(c)
        p.save()
        p.rotate(self._heading)
        p.setBrush(QColor(70, 208, 106, 26))
        pen = QPen(QColor(70, 208, 106, 128), 1.2 * k)
        pen.setStyle(Qt.CustomDashLine)
        pen.setDashPattern([3, 4])
        p.setPen(pen)
        p.drawEllipse(QPointF(0, 0), 46 * k, 46 * k)
        chev = QPainterPath()
        chev.moveTo(0, -62 * k); chev.lineTo(9 * k, -40 * k); chev.lineTo(-9 * k, -40 * k); chev.closeSubpath()
        p.fillPath(chev, QColor(70, 208, 106, 204))
        p.setPen(QPen(QColor("#46d06a"), 2.2 * k, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(-17 * k, -17 * k), QPointF(17 * k, 17 * k))
        p.drawLine(QPointF(17 * k, -17 * k), QPointF(-17 * k, 17 * k))
        p.setBrush(QColor("#0e1210"))
        for sx, sy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
            p.drawEllipse(QPointF(19 * sx * k, 19 * sy * k), 9 * k, 9 * k)
        p.setBrush(QColor("#46d06a"))
        p.drawRoundedRect(QRectF(-9 * k, -9 * k, 18 * k, 18 * k), 5 * k, 5 * k)
        p.restore()
        # name tag
        f = QFont(self.font()); f.setPixelSize(px(11)); f.setBold(True)
        p.setFont(f)
        fm = p.fontMetrics()
        w, h = fm.horizontalAdvance(self.NAME) + 16 * k, fm.height() + 6 * k
        r = QRectF(28 * k, -40 * k, w, h)
        p.setBrush(QColor(14, 16, 15, 217))
        p.setPen(QPen(QColor(255, 255, 255, 41), 1))
        p.drawRoundedRect(r, 6 * k, 6 * k)
        p.setPen(QColor("#e9ece9"))
        p.drawText(r, Qt.AlignCenter, self.NAME)
        p.end()
