"""
================================================================================
MODULE: battery_badge.py
PURPOSE: Battery State Indicator - drawn cell, fill level, percentage, voltage
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Header)
  * Upstream:      TelemetrySnapshot battery fields
  * Downstream:    The operator's peripheral vision

WHY A DRAWN CELL AND NOT A LABEL:
  Battery is the most-scanned value on the screen. A fill level is read
  pre-attentively - the shape says "low" before any text is parsed - whereas
  "BAT: 14.2V (18%)" has to be read to be understood.

WHY IT DOES NOT ANIMATE CONTINUOUSLY:
  A cycling fill is the universal idiom for *charging*, which is the opposite
  of what a flying battery is doing. More importantly, motion in a cockpit
  should mean something changed; an indicator that always moves teaches the eye
  to ignore that corner. So the only animation is a slow pulse below the
  critical threshold - the one moment attention should be pulled there.

THE NO-DATA STATE IS NOT ZERO PERCENT:
  With nothing connected the telemetry reads 0 V / 0%. Rendering that as an
  empty red cell would raise a battery alarm every time the app is open and the
  aircraft is off, which is exactly how an indicator trains people to disregard
  it. Unknown is drawn as a hollow grey cell with dashes, and never pulses.

USAGE:
  badge = BatteryBadge()
  badge.set_cell_count(4)
  badge.set_state(voltage=16.4, percent=78, connected=True)
================================================================================
"""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QWidget
from ui.scaling import px

from ui.styles import PALETTE

# Geometry, in device-independent pixels. Reading order is voltage, cell,
# percentage - the percentage last so it ends up in the header's corner.
CELL_W, CELL_H, CAP_W, CAP_H = 30.0, 14.0, 3.0, 7.0
VOLT_W, PCT_W, GAP = 54.0, 44.0, 7.0


class BatteryBadge(QWidget):
    """Battery cell glyph plus percentage and pack voltage."""

    def __init__(self, parent: Optional[QWidget] = None,
                 warn_pct: int = 35, crit_pct: int = 20):
        super().__init__(parent)
        self.warn_pct = warn_pct
        self.crit_pct = crit_pct
        self.cells = 4

        self.voltage = 0.0
        self.percent = 0
        self.connected = False

        self._pulse_on = True
        self._pulse = QTimer(self)
        self._pulse.setInterval(620)
        self._pulse.timeout.connect(self._tick_pulse)

        self.setFixedHeight(px(26))
        self.setFixedWidth(int(VOLT_W + GAP + CELL_W + CAP_W + GAP + PCT_W))
        self.setToolTip("Battery - no telemetry")

    # ── state ───────────────────────────────────────────────────────

    def set_cell_count(self, cells: int) -> None:
        self.cells = max(1, int(cells))
        self._refresh_tooltip()

    def set_thresholds(self, warn_pct: int, crit_pct: int) -> None:
        self.warn_pct, self.crit_pct = int(warn_pct), int(crit_pct)
        self.update()

    def set_state(self, voltage: float, percent: int, connected: bool) -> None:
        self.voltage = float(voltage or 0.0)
        self.percent = max(0, min(100, int(percent or 0)))
        # A pack reading 0 V is not a flat battery, it is no battery: treat the
        # absence of telemetry as unknown rather than as an emergency.
        self.connected = bool(connected) and self.voltage > 0.5

        if self.connected and self.percent <= self.crit_pct:
            if not self._pulse.isActive():
                self._pulse_on = True
                self._pulse.start()
        elif self._pulse.isActive():
            self._pulse.stop()
            self._pulse_on = True

        self._refresh_tooltip()
        self.update()

    def _tick_pulse(self) -> None:
        self._pulse_on = not self._pulse_on
        self.update()

    def _refresh_tooltip(self) -> None:
        if not self.connected:
            self.setToolTip("Battery - no telemetry")
            return
        per_cell = self.voltage / self.cells if self.cells else 0.0
        self.setToolTip(
            f"Battery: {self.percent}%  |  {self.voltage:.2f} V pack  |  "
            f"{per_cell:.2f} V/cell ({self.cells}S)\n"
            f"Warning below {self.warn_pct}%, critical below {self.crit_pct}%")

    # ── rendering ───────────────────────────────────────────────────

    def _state_colour(self) -> QColor:
        if not self.connected:
            return QColor(PALETTE["text_muted"])
        if self.percent <= self.crit_pct:
            return QColor(PALETTE["danger"])
        if self.percent <= self.warn_pct:
            return QColor(PALETTE["warn"])
        return QColor(PALETTE["ok"])

    def paintEvent(self, _event):
        """Voltage, then the cell, then the percentage - left to right.

        The percentage sits on the outside so it lands in the very corner of
        the header, which is where a laptop or phone puts it and therefore
        where the eye already goes looking.
        """
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)

        col = self._state_colour()
        # The pulse dims rather than blanks: a disappearing indicator is
        # ambiguous with a dead one.
        if self._pulse.isActive() and not self._pulse_on:
            col = QColor(col.red(), col.green(), col.blue(), 90)

        top = (self.height() - CELL_H) / 2.0

        # Voltage, right-aligned against the cell it describes.
        p.setFont(QFont("Noto Sans Mono", 9))
        p.setPen(QColor(PALETTE["text_dim"] if self.connected else PALETTE["text_muted"]))
        p.drawText(QRectF(0, 0, VOLT_W, self.height()),
                   Qt.AlignVCenter | Qt.AlignRight,
                   f"{self.voltage:.1f} V" if self.connected else "-- V")

        shell = QRectF(VOLT_W + GAP, top, CELL_W, CELL_H)
        p.setPen(QPen(col, 1.4))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(shell, 2.5, 2.5)

        cap = QRectF(shell.right() + 1.0, top + (CELL_H - CAP_H) / 2.0, CAP_W, CAP_H)
        p.fillRect(cap, col)

        if self.connected:
            inner_w = (CELL_W - 4.0) * (self.percent / 100.0)
            if inner_w > 0.5:
                p.fillRect(QRectF(shell.left() + 2.0, top + 2.0,
                                  inner_w, CELL_H - 4.0), col)
        else:
            # Unknown: a single muted dash inside the shell.
            p.setPen(QPen(QColor(PALETTE["text_muted"]), 1.4))
            mid = top + CELL_H / 2.0
            p.drawLine(int(shell.left() + 6), int(mid), int(shell.right() - 6), int(mid))

        p.setFont(QFont("Noto Sans Mono", 9, QFont.Bold))
        p.setPen(col)
        p.drawText(QRectF(cap.right() + GAP, 0, PCT_W, self.height()),
                   Qt.AlignVCenter | Qt.AlignLeft,
                   f"{self.percent}%" if self.connected else "--%")
        p.end()
