"""
================================================================================
MODULE: fpv_hud.py
PURPOSE: Full-bleed FPV video layer and its HUD overlay (v2 design)
================================================================================

``FpvVideoLabel`` is the label the FPV workspace paints decoded frames into. The
main window parks it as the full-window "bleed" layer (ui/theme.ShellRoot) so the
picture runs behind every glass panel; with no frame it paints nothing and the
sharp backdrop shows through, exactly like the reference.

``FpvHud`` is a transparent child of that label. It draws the design's overlay:
ALT / SPD / HDG readout, a horizon line with the aircraft chevron, the ladder of
ticks on the right and the status chip ("LIVE · 30 FPS · 1080p") at the top-right
of the clear video area. It is screen-only - frames are never modified - and is
driven by ``set_telemetry`` / ``set_status``; it sends nothing anywhere.
================================================================================
"""

from __future__ import annotations

import math
from typing import Optional

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import QLabel, QWidget

from ui.scaling import px

HUD_GREEN = QColor(70, 208, 106)
HUD_TEXT = QColor("#9ff0b5")
_TONES = {"ok": QColor(70, 208, 106), "warn": QColor(255, 176, 46),
          "bad": QColor(255, 90, 79), "": QColor(163, 170, 165)}


class FpvHud(QWidget):
    """Transparent overlay; geometry always equals its parent's."""

    # The status chip sits at the right edge of the video area that is not covered by
    # the persistent right-hand column (400 + margins/gap in the shell).
    CHIP_RIGHT_INSET = 440
    CHIP_TOP = 112
    # Pixels of ladder travel per degree of pitch (decorative scale).
    PITCH_PX = 6.0

    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WA_NoSystemBackground, True)
        self.show_readout = True
        self._alt = 0.0
        self._spd = 0.0
        self._hdg = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._chip_text = "NO FEED"
        self._chip_tone = ""

    # -- inputs ----------------------------------------------------------
    def set_telemetry(self, t) -> None:
        self._alt = float(getattr(t, "altitude", 0.0) or 0.0)
        self._spd = float(getattr(t, "ground_speed", 0.0) or 0.0)
        self._hdg = float(getattr(t, "heading", 0.0) or 0.0) % 360.0
        self._roll = float(getattr(t, "roll", 0.0) or 0.0)
        self._pitch = float(getattr(t, "pitch", 0.0) or 0.0)
        if self.show_readout and self.isVisible():
            self.update()

    def set_status(self, text: str, tone: str = "") -> None:
        if (text, tone) != (self._chip_text, self._chip_tone):
            self._chip_text, self._chip_tone = text, tone
            self.update()

    def set_readout_visible(self, on: bool) -> None:
        self.show_readout = bool(on)
        self.update()

    # -- painting --------------------------------------------------------
    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)
        w, h = self.width(), self.height()
        self._paint_chip(p, w)
        if self.show_readout:
            self._paint_readout(p, w, h)
        p.end()

    def _mono(self, size: float) -> QFont:
        f = QFont("Ubuntu Mono")
        f.setStyleHint(QFont.Monospace)
        f.setPixelSize(max(8, px(size)))
        return f

    def _shadow_text(self, p: QPainter, x: float, y: float, text: str, col: QColor) -> None:
        p.setPen(QColor(0, 0, 0, 200))
        p.drawText(QPointF(x + 0.5, y + 1.5), text)
        p.setPen(QColor(0, 0, 0, 160))
        p.drawText(QPointF(x, y + 1), text)
        p.setPen(col)
        p.drawText(QPointF(x, y), text)

    def _paint_chip(self, p: QPainter, w: int) -> None:
        col = _TONES.get(self._chip_tone, _TONES[""])
        f = QFont(self.font())
        f.setPixelSize(px(10.5))
        f.setBold(True)
        f.setLetterSpacing(QFont.AbsoluteSpacing, 0.6)
        p.setFont(f)
        fm = p.fontMetrics()
        text = self._chip_text.upper()
        ch = px(24)
        cw = fm.horizontalAdvance(text) + px(9) * 2 + px(6) + px(6)
        right = w - px(self.CHIP_RIGHT_INSET)
        r = QRectF(right - cw, px(self.CHIP_TOP), cw, ch)
        p.setBrush(QColor(255, 255, 255, 10))
        edge = QColor(col)
        edge.setAlpha(102 if self._chip_tone else 40)
        p.setPen(QPen(edge, 1))
        p.drawRoundedRect(r.adjusted(0.5, 0.5, -0.5, -0.5), ch / 2, ch / 2)
        dot = QPointF(r.left() + px(9) + px(3), r.center().y())
        if self._chip_tone:
            glow = QColor(col)
            glow.setAlpha(70)
            p.setPen(Qt.NoPen)
            p.setBrush(glow)
            p.drawEllipse(dot, px(6), px(6))
        p.setPen(Qt.NoPen)
        p.setBrush(col if self._chip_tone else QColor("#6f7772"))
        p.drawEllipse(dot, px(3), px(3))
        p.setPen(col)
        p.drawText(QRectF(r.left() + px(9) + px(6) + px(6), r.top(), r.width(), r.height()),
                   Qt.AlignVCenter | Qt.AlignLeft, text)

    def _paint_readout(self, p: QPainter, w: int, h: int) -> None:
        cx = w / 2.0 - px(10)            # 790 on the 1600 wide design canvas
        cy = h * 0.4667                  # 420 on 900 high

        # ALT / SPD / HDG text block
        p.setFont(self._mono(14))
        lh = px(14) * 1.7
        x0 = cx - px(320)
        y0 = h / 3.0 + px(14)            # 300 + first baseline on the design canvas
        for i, line in enumerate((f"ALT {self._alt:.1f} m", f"SPD {self._spd:.2f} m/s",
                                  f"HDG {int(round(self._hdg)) % 360:03d}°")):
            self._shadow_text(p, x0, y0 + i * lh, line, HUD_TEXT)

        # horizon: rolls with the aircraft, slides with pitch
        p.save()
        p.translate(cx, cy + 1)
        p.translate(0, max(-px(120), min(px(120), self._pitch * self.PITCH_PX)))
        p.rotate(-self._roll)
        half = px(360)
        from PyQt5.QtGui import QLinearGradient
        g = QLinearGradient(-half, 0, half, 0)
        g.setColorAt(0, QColor(70, 208, 106, 0))
        g.setColorAt(0.5, QColor(70, 208, 106, 230))
        g.setColorAt(1, QColor(70, 208, 106, 0))
        p.setPen(QPen(g, max(2, px(2))))
        p.drawLine(QPointF(-half, 0), QPointF(half, 0))
        # aircraft chevron: M0 30 h50 l10 14 l20 -28 l10 14 h50 (140 wide, centred)
        path = QPainterPath()
        s = px(1)
        pts = [(-70, 30), (-20, 30), (-10, 44), (10, 16), (20, 30), (70, 30)]
        path.moveTo(pts[0][0] * s, (pts[0][1] - 1) * s)
        for x, y in pts[1:]:
            path.lineTo(x * s, (y - 1) * s)
        p.setPen(QPen(HUD_GREEN, 2.5 * s, Qt.SolidLine, Qt.FlatCap, Qt.MiterJoin))
        p.setBrush(Qt.NoBrush)
        p.drawPath(path)
        p.restore()

        # ladder of ticks on the right
        p.setFont(self._mono(13))
        tx = cx + px(240)
        lad = QColor(70, 208, 106, 204)
        for i in range(-5, 6):
            y = cy + px(30) + i * px(34) + self._pitch * 0.0
            ln = px(20 if i % 2 else 34)
            p.setPen(QPen(lad, max(1.5, px(1.5))))
            p.drawLine(QPointF(tx, y), QPointF(tx + ln, y))
            if i % 2 == 0:
                self._shadow_text(p, tx + px(40), y + px(4), str((5 - i) * 10), HUD_TEXT)


class FpvVideoLabel(QLabel):
    """Frame target for the FPV workspace; carries the HUD as a child."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("", parent)
        self.setObjectName("fpvVideo")
        self.setAlignment(Qt.AlignCenter)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.hud = FpvHud(self)
        self.hud.setGeometry(self.rect())

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self.hud.setGeometry(self.rect())
        self.hud.raise_()

    def set_live(self, live: bool) -> None:
        """Solid black behind a real picture (letterbox bars); transparent otherwise."""
        if bool(self.property("live")) != live:
            self.setProperty("live", live)
            self.style().unpolish(self)
            self.style().polish(self)
