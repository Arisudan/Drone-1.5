"""
================================================================================
MODULE: hud_widget.py
PURPOSE: Minimal Speed / Altitude Primary Flight Display
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Flight Deck)
  * Communicates:  drone_gcs.py main window and TelemetryState instances
  * Upstream:      Decoded LOCAL_POSITION_NED (ground speed, altitude) and HEARTBEAT
                   (armed, flight mode)
  * Downstream:    Pilot PFD visual display (60 FPS smooth vector rendering)

DATA FLOW & INTERFACES:
  * Inputs:        Altitude (m AGL) and Ground Speed (m/s) from LOCAL_POSITION_NED,
                   Armed status and Flight Mode from HEARTBEAT.
  * GUI Engine:    PyQt5 QPainter with antialiased vector geometry.

KEY LOGIC & FAILSAFES:
  * Rolling Tape Ladders: Aviation-standard left speed tape and right altitude tape
    with moving numerical readouts and centered pointer bugs - both driven directly
    from live Pixhawk telemetry, not placeholder values.
  * Deliberately minimal: no artificial horizon, roll/pitch ladder, compass tape, or
    crosshair - the operator only needs speed/altitude/mode/armed at a glance here;
    attitude and heading are better served by the FPV/SLAM views.

USAGE:
  pfd = HUDWidget()
  pfd.update_telemetry(snapshot)
================================================================================
"""

from __future__ import annotations
from typing import Optional

from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QPainter, QColor, QFont, QPen
from PyQt5.QtWidgets import QWidget, QSizePolicy

from core.telemetry import TelemetrySnapshot


class HUDWidget(QWidget):
    """Minimal PFD: live speed tape, altitude tape, and a mode/armed banner - nothing else."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(540, 380)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Telemetry State - trimmed to only what's actually drawn now
        self.altitude: float = 0.0     # m AGL
        self.speed: float = 0.0        # m/s
        self.armed: bool = False
        self.flight_mode: str = "DISCONNECTED"

    def update_telemetry(self, t: TelemetrySnapshot):
        """Update HUD state from TelemetrySnapshot and trigger repaint."""
        self.altitude = t.altitude
        self.speed = t.ground_speed
        self.armed = t.armed
        self.flight_mode = t.flight_mode
        self.update()

    def paintEvent(self, event):
        """Paint full aviation PFD cockpit overlay."""
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0

        # 1. Background fill (Dark tactical cockpit)
        p.fillRect(0, 0, w, h, QColor(9, 13, 18))

        # 2. Dynamic Rolling Airspeed Tape Ladder (Left) - sized to fill most of the
        # now-empty canvas since the horizon/compass/reticle graphics are gone.
        tape_h = min(h * 0.75, 480)
        self._draw_scrolling_speed_tape(p, 90, cy, tape_h)

        # 3. Dynamic Rolling Altitude Tape Ladder (Right)
        self._draw_scrolling_altitude_tape(p, w - 96, cy, tape_h)

        # 4. Mode / Armed Status Banner
        self._draw_status_overlay(p, w, h)

        p.end()

    def _draw_scrolling_speed_tape(self, p: QPainter, x: float, cy: float, tape_h: float):
        """Draw true dynamic scrolling speed ladder on the left."""
        p.save()
        tape_w = 72.0  # widened from 54 - more room to fill now the horizon graphic is gone
        y0 = cy - (tape_h / 2.0)

        # Outer Frame
        p.fillRect(QRectF(x - tape_w / 2.0, y0, tape_w, tape_h), QColor(13, 17, 23, 220))
        p.setPen(QPen(QColor(48, 54, 61), 1))
        p.drawRect(QRectF(x - tape_w / 2.0, y0, tape_w, tape_h))

        # Title
        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        p.setPen(QColor(139, 148, 158))
        p.drawText(QRectF(x - tape_w / 2.0, y0 - 22, tape_w, 18), Qt.AlignCenter, "SPEED m/s")

        # Clip inside tape
        p.setClipRect(QRectF(x - tape_w / 2.0, y0, tape_w, tape_h))

        # Dynamic Scrolling Ticks
        px_per_unit = 30.0
        current_spd = max(0.0, self.speed)
        base_spd = int(current_spd)

        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        for s in range(base_spd - 8, base_spd + 9):
            if s < 0:
                continue
            y_pos = cy - ((s - current_spd) * px_per_unit)
            is_major = (s % 2 == 0)

            if is_major:
                p.setPen(QPen(QColor(255, 255, 255, 200), 1.5))
                p.drawLine(QPointF(x + tape_w / 2.0 - 16, y_pos), QPointF(x + tape_w / 2.0, y_pos))
                p.drawText(QRectF(x - tape_w / 2.0 + 4, y_pos - 8, 32, 16), Qt.AlignRight | Qt.AlignVCenter, f"{s}")
            else:
                p.setPen(QPen(QColor(139, 148, 158, 160), 1))
                p.drawLine(QPointF(x + tape_w / 2.0 - 9, y_pos), QPointF(x + tape_w / 2.0, y_pos))

        p.setClipping(False)

        # Center Numeric Pointer Box
        p.fillRect(QRectF(x - tape_w / 2.0 - 4, cy - 15, tape_w + 8, 30), QColor(31, 111, 235))
        p.setPen(QPen(QColor(255, 255, 255), 1.5))
        p.drawRect(QRectF(x - tape_w / 2.0 - 4, cy - 15, tape_w + 8, 30))
        p.setFont(QFont("Segoe UI", 13, QFont.Bold))
        p.drawText(QRectF(x - tape_w / 2.0 - 4, cy - 15, tape_w + 8, 30), Qt.AlignCenter, f"{self.speed:.1f}")

        p.restore()

    def _draw_scrolling_altitude_tape(self, p: QPainter, x: float, cy: float, tape_h: float):
        """Draw true dynamic scrolling altitude ladder on the right."""
        p.save()
        tape_w = 80.0  # widened from 60 - more room to fill now the horizon graphic is gone
        y0 = cy - (tape_h / 2.0)

        # Outer Frame
        p.fillRect(QRectF(x - tape_w / 2.0, y0, tape_w, tape_h), QColor(13, 17, 23, 220))
        p.setPen(QPen(QColor(48, 54, 61), 1))
        p.drawRect(QRectF(x - tape_w / 2.0, y0, tape_w, tape_h))

        # Title
        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        p.setPen(QColor(139, 148, 158))
        p.drawText(QRectF(x - tape_w / 2.0, y0 - 22, tape_w, 18), Qt.AlignCenter, "ALT AGL (m)")

        # Clip inside tape
        p.setClipRect(QRectF(x - tape_w / 2.0, y0, tape_w, tape_h))

        # Dynamic Scrolling Ticks
        px_per_unit = 30.0
        current_alt = self.altitude
        base_alt = int(current_alt)

        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        for a in range(base_alt - 8, base_alt + 9):
            y_pos = cy - ((a - current_alt) * px_per_unit)
            is_major = (a % 2 == 0)

            if is_major:
                p.setPen(QPen(QColor(255, 255, 255, 200), 1.5))
                p.drawLine(QPointF(x - tape_w / 2.0, y_pos), QPointF(x - tape_w / 2.0 + 16, y_pos))
                p.drawText(QRectF(x - tape_w / 2.0 + 20, y_pos - 8, 40, 16), Qt.AlignLeft | Qt.AlignVCenter, f"{a}")
            else:
                p.setPen(QPen(QColor(139, 148, 158, 160), 1))
                p.drawLine(QPointF(x - tape_w / 2.0, y_pos), QPointF(x - tape_w / 2.0 + 9, y_pos))

        p.setClipping(False)

        # Center Numeric Pointer Box
        p.fillRect(QRectF(x - tape_w / 2.0 - 4, cy - 15, tape_w + 8, 30), QColor(35, 134, 54))
        p.setPen(QPen(QColor(255, 255, 255), 1.5))
        p.drawRect(QRectF(x - tape_w / 2.0 - 4, cy - 15, tape_w + 8, 30))
        p.setFont(QFont("Segoe UI", 13, QFont.Bold))
        p.drawText(QRectF(x - tape_w / 2.0 - 4, cy - 15, tape_w + 8, 30), Qt.AlignCenter, f"{self.altitude:.2f}")

        p.restore()

    def _draw_status_overlay(self, p: QPainter, w: float, h: float):
        """Draw the Mode & Arming state banner, bottom-center. The D435i VIO pill and
        NED position readout that used to live here were dropped - both duplicate what
        the header now shows persistently on every tab (VIO badge, and a NED readout
        next to RX/TX), so keeping them here too was pure duplication."""
        p.save()

        arm_text = "ARMED" if self.armed else "DISARMED"
        banner_w = 280
        bx = (w - banner_w) / 2.0
        by = h - 36

        p.fillRect(QRectF(bx, by, banner_w, 28), QColor(13, 17, 23, 230))
        p.setPen(QPen(QColor(48, 54, 61), 1))
        p.drawRect(QRectF(bx, by, banner_w, 28))

        # Mode text
        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        p.setPen(QColor(88, 166, 255))
        p.drawText(QRectF(bx + 14, by, 150, 28), Qt.AlignVCenter | Qt.AlignLeft, f"MODE: {self.flight_mode}")

        # Armed pill
        pill_col = QColor(63, 185, 80) if self.armed else QColor(139, 148, 158)
        p.setPen(pill_col)
        p.drawText(QRectF(bx + 160, by, 106, 28), Qt.AlignVCenter | Qt.AlignRight, arm_text)

        p.restore()
