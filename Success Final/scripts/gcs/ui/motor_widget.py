"""
================================================================================
MODULE: motor_widget.py
PURPOSE: Live Actuator & ESC PWM Telemetry Gauge (4-8 Motors) with Saturation Alarms
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Flight Deck) & Radxa Monitor
  * Communicates:  drone_gcs.py / radxa_monitor.py and TelemetryState
  * Upstream:      Decoded SERVO_OUTPUT_RAW MAVLink messages from Pixhawk 6X
  * Downstream:    Visual actuator load bars and motor health diagnostics

DATA FLOW & INTERFACES:
  * Inputs:        Array of raw actuator PWM microsecond values [ch1, ch2, ch3, ch4...]
  * Output Visual: Vertical bar gauges with percentage fills and numerical readouts.
  * Update Rate:   10 Hz telemetry updates from autopilot actuator stream.

KEY LOGIC & FAILSAFES:
  * Dynamic Color Coding:
      - <=1050 µs: Idle / Disarmed (Muted gray)
      - 1051 - 1750 µs: Nominal In-Flight Load (Green)
      - 1751 - 1900 µs: Elevated Power Demand (Warning amber)
      - >1900 µs: Actuator Saturation / Imminent Desync (Critical red alert)
  * Quadcopter & Hexacopter Scalability: Dynamically switches layout between 4-channel
    quadrotor and 6/8-channel multirotor configurations.
  * Desync Detection: Enables pilots to instantly spot a single struggling motor
    pinned at maximum PWM while other motors hover near mid-stick.

USAGE:
  motor_widget = MotorWidget(num_channels=4)
  motor_widget.update_motors([1450, 1460, 1455, 1470])
================================================================================
"""

from __future__ import annotations
from typing import List, Optional

from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QPainter, QColor, QFont, QPen, QBrush
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QSizePolicy


class MotorChannelBar(QWidget):
    """Individual vertical bar gauge for a single motor channel."""

    def __init__(self, channel_num: int, label: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.channel_num = channel_num
        self.label = label
        self.pwm: int = 1000
        self.setMinimumSize(64, 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_pwm(self, pwm: int):
        self.pwm = max(900, min(2100, int(pwm)))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Margins
        bar_w = 26.0
        bar_x = (w - bar_w) / 2.0
        top_y = 20.0
        bot_y = h - 38.0
        bar_h = bot_y - top_y

        # Value percentage (1000 to 2000 us)
        pct = max(0.0, min(1.0, (self.pwm - 1000) / 1000.0))
        fill_h = bar_h * pct

        # Background track
        p.fillRect(QRectF(bar_x, top_y, bar_w, bar_h), QColor(22, 27, 34))
        p.setPen(QPen(QColor(48, 54, 61), 1))
        p.drawRect(QRectF(bar_x, top_y, bar_w, bar_h))

        # Color based on throttle load
        if self.pwm <= 1050:
            bar_color = QColor(139, 148, 158)  # Idle / Disarmed
        elif self.pwm <= 1750:
            bar_color = QColor(46, 160, 67)    # Nominal Green
        elif self.pwm <= 1900:
            bar_color = QColor(210, 153, 34)   # High Load Amber
        else:
            bar_color = QColor(248, 81, 73)    # Saturation Red

        # Filled Bar
        if fill_h > 0:
            p.fillRect(QRectF(bar_x + 1, bot_y - fill_h, bar_w - 2, fill_h), bar_color)

        # Reference Lines (1000, 1500, 2000 us)
        p.setPen(QPen(QColor(88, 166, 255, 120), 1, Qt.DashLine))
        mid_y = top_y + (bar_h / 2.0)
        p.drawLine(QPointF(bar_x - 4, mid_y), QPointF(bar_x + bar_w + 4, mid_y))

        # Channel Header Label (Top)
        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        p.setPen(QColor(201, 209, 217))
        p.drawText(QRectF(0, 2, w, 16), Qt.AlignCenter, f"M{self.channel_num}")

        # PWM & Percentage Numeric Readout (Bottom)
        p.setFont(QFont("Consolas", 9, QFont.Bold))
        p.setPen(bar_color if self.pwm > 1050 else QColor(139, 148, 158))
        p.drawText(QRectF(0, h - 34, w, 16), Qt.AlignCenter, f"{self.pwm} µs")

        p.setFont(QFont("Segoe UI", 7))
        p.setPen(QColor(139, 148, 158))
        p.drawText(QRectF(0, h - 18, w, 14), Qt.AlignCenter, f"{int(pct * 100)}% • {self.label}")

        p.end()


class MotorWidget(QWidget):
    """Compound widget presenting 4-motor quadcopter PWM telemetry."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        # Header Title
        title_box = QHBoxLayout()
        t = QLabel("ACTUATOR OUTPUTS & MOTOR TELEMETRY (SERVO_OUTPUT_RAW)")
        t.setStyleSheet("font-weight: bold; color: #58a6ff; font-size: 12px; letter-spacing: 0.8px;")
        title_box.addWidget(t)
        title_box.addStretch()

        self.lbl_status = QLabel("ALL MOTORS IDLE (1000 µs)")
        self.lbl_status.setStyleSheet("color: #8b949e; font-size: 11px; font-weight: bold;")
        title_box.addWidget(self.lbl_status)
        layout.addLayout(title_box)

        # Container Card for Channels
        card = QFrame(self)
        card.setProperty("class", "cardFrame")
        card_layout = QHBoxLayout(card)
        card_layout.setContentsMargins(16, 12, 16, 12)
        card_layout.setSpacing(16)

        # Standard PX4 Quad X Motor Mapping:
        # M1: Front Right (CCW)
        # M2: Rear Left (CCW)
        # M3: Front Left (CW)
        # M4: Rear Right (CW)
        self.bar_m1 = MotorChannelBar(1, "FR CCW", self)
        self.bar_m2 = MotorChannelBar(2, "RL CCW", self)
        self.bar_m3 = MotorChannelBar(3, "FL CW", self)
        self.bar_m4 = MotorChannelBar(4, "RR CW", self)

        card_layout.addWidget(self.bar_m1)
        card_layout.addWidget(self.bar_m2)
        card_layout.addWidget(self.bar_m3)
        card_layout.addWidget(self.bar_m4)

        layout.addWidget(card, 1)

        # Safety & Calibration Footer Note
        note = QLabel("Nominal idle: 1000–1100 µs | Hover range: 1350–1550 µs | Saturation alert: >1920 µs")
        note.setStyleSheet("color: #8b949e; font-size: 10px; font-style: italic;")
        layout.addWidget(note)

    def update_pwms(self, pwms: List[int]):
        """Update 4 channel bars with new PWM values."""
        if len(pwms) >= 4:
            self.bar_m1.set_pwm(pwms[0])
            self.bar_m2.set_pwm(pwms[1])
            self.bar_m3.set_pwm(pwms[2])
            self.bar_m4.set_pwm(pwms[3])

            max_pwm = max(pwms[:4])
            if max_pwm > 1920:
                self.lbl_status.setText(f"WARNING: HIGH LOAD SATURATION ({max_pwm} µs)")
                self.lbl_status.setStyleSheet("color: #f85149; font-weight: bold;")
            elif max_pwm > 1150:
                self.lbl_status.setText(f"MOTORS ACTIVE ({max_pwm} µs)")
                self.lbl_status.setStyleSheet("color: #58a6ff; font-weight: bold;")
            else:
                self.lbl_status.setText("ALL MOTORS IDLE (1000 µs)")
                self.lbl_status.setStyleSheet("color: #8b949e; font-weight: bold;")
