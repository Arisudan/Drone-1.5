"""
================================================================================
MODULE: shell.py
PURPOSE: The v2 persistent shell cards: route card, vehicle card, telemetry, commands
================================================================================

These four widgets are on screen in every workspace. They are *views*: the
command buttons are plain QPushButtons that the main window wires to the same
handlers the old cockpit buttons used, and ``TelemetryCard.update_from`` only
reads a TelemetrySnapshot. Nothing here sends MAVLink or touches settings.

Controls the design shows but this station has no backend for (UPLOAD MISSION,
AUTO MODE, JOYSTICK) are real, *disabled* buttons with a tooltip saying so -
they are never wired to something that merely looks similar.
================================================================================
"""

from __future__ import annotations

import math
from typing import Optional

from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (QGridLayout, QHBoxLayout, QLabel, QPushButton, QSizePolicy,
                             QVBoxLayout, QWidget, QFrame)

from core.telemetry import TelemetrySnapshot
from ui.scaling import px
from ui.theme import (Chip, GlassBar, GlassPanel, MetricCell, icon, GREEN, ORANGE, RED, MUTE, DIM)


class _IconBox(QWidget):
    """Square (or round) icon holder painted with the design's tinted border."""

    def __init__(self, name: str, size: int, round_: bool, tint: Optional[QColor],
                 colour: str, parent=None):
        super().__init__(parent)
        self._name, self._round, self._tint, self._colour = name, round_, tint, colour
        self._size = size
        self.setFixedSize(px(size), px(size))

    def set_colour(self, colour: str) -> None:
        self._colour = colour
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(self.rect()).adjusted(0.75, 0.75, -0.75, -0.75)
        rad = r.width() / 2 if self._round else 9
        if self._tint is not None:
            p.setBrush(self._tint)
            p.setPen(QPen(QColor(70, 208, 106, 102), 1))
        else:
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor(255, 255, 255, 41), 1.5))
        p.drawRoundedRect(r, rad, rad)
        pm = icon(self._name, self._colour, 22 if self._round else 18, 1.6 if self._round else 1.7)
        s = pm.width() / pm.devicePixelRatio()
        p.drawPixmap(int((self.width() - s) / 2), int((self.height() - s) / 2), pm)
        p.end()


class RouteCard(GlassBar):
    """Top-left card: the active workspace's name and a one-line status."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("routeCard")
        self.set_accent(QColor(70, 208, 106, 140), QColor(70, 208, 106, 24))
        self.setFixedSize(px(268), px(54))
        lay = QHBoxLayout(self)
        lay.setContentsMargins(px(14), px(10), px(14), px(10))
        lay.setSpacing(px(12))
        self._icon = _IconBox("drone", 34, False, QColor(70, 208, 106, 31), GREEN, self)
        lay.addWidget(self._icon)
        col = QVBoxLayout()
        col.setSpacing(0)
        col.setContentsMargins(0, 0, 0, 0)
        self.lbl_title = QLabel("Control", self)
        self.lbl_title.setObjectName("routeTitle")
        self.lbl_sub = QLabel("", self)
        self.lbl_sub.setObjectName("routeSub")
        col.addStretch(1)
        col.addWidget(self.lbl_title)
        col.addWidget(self.lbl_sub)
        col.addStretch(1)
        lay.addLayout(col, 1)

    def set_route(self, title: str, sub: str) -> None:
        self.lbl_title.setText(title)
        self.lbl_sub.setText(sub)

    def set_sub(self, sub: str) -> None:
        self.lbl_sub.setText(sub)


class VehicleCard(GlassBar):
    """Top-right card: vehicle name, link line and four state icons."""

    NAME = "HTIC-QUAD"

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("vehicleCard")
        self.setFixedHeight(px(54))
        lay = QHBoxLayout(self)
        lay.setContentsMargins(px(14), px(9), px(14), px(9))
        lay.setSpacing(px(12))
        lay.addWidget(_IconBox("drone", 38, True, None, DIM, self))
        col = QVBoxLayout()
        col.setSpacing(0)
        col.setContentsMargins(0, 0, 0, 0)
        col.addStretch(1)
        self.lbl_name = QLabel(self.NAME, self)
        self.lbl_name.setObjectName("vehName")
        self.lbl_sub = QLabel("Quadcopter · MAVLink SysID 1", self)
        self.lbl_sub.setObjectName("vehSub")
        col.addWidget(self.lbl_name)
        col.addWidget(self.lbl_sub)
        col.addStretch(1)
        lay.addLayout(col, 1)
        self._state_icons = {}
        for key, name in (("bat", "bat"), ("gps", "gps"), ("rc", "rc"), ("sat", "sat")):
            lbl = QLabel(self)
            lbl.setFixedSize(px(18), px(18))
            self._state_icons[key] = (lbl, name)
            self._paint_icon(key, MUTE)
            lay.addWidget(lbl)
        self._tones = {}

    def _paint_icon(self, key: str, colour: str) -> None:
        lbl, name = self._state_icons[key]
        lbl.setPixmap(icon(name, colour, 18, 1.7))

    def set_tone(self, key: str, tone: str, tip: str = "") -> None:
        """tone: "" muted | "ok" | "warn" | "bad"."""
        if self._tones.get(key) == tone:
            return
        self._tones[key] = tone
        self._paint_icon(key, {"": MUTE, "ok": GREEN, "warn": ORANGE, "bad": RED}[tone])
        if tip:
            self._state_icons[key][0].setToolTip(tip)

    def set_sysid(self, sysid: int) -> None:
        self.lbl_sub.setText(f"Quadcopter · MAVLink SysID {sysid}")


class TelemetryCard(GlassPanel):
    """Battery / GPS / link / RC tiles, state chips and altitude tiles."""

    def __init__(self, parent=None):
        super().__init__("Telemetry", VehicleCard.NAME, True, parent)
        body = self.body_layout
        body.setContentsMargins(px(14), px(12), px(14), px(12))
        body.setSpacing(px(6))

        g1 = QGridLayout()
        g1.setHorizontalSpacing(px(6))
        g1.setVerticalSpacing(0)
        self.c_batt = MetricCell("Battery", "--", "V")
        self.c_batt.add_line("--", "%", 17)
        self.c_gps = MetricCell("GPS", "--")
        self.c_gps.add_line("NO FIX", "", 15)
        self.c_link = MetricCell("Telemetry", "--", "%")
        self.c_rc = MetricCell("RC link", "N/A")
        for col, (cell, stretch) in enumerate(((self.c_batt, 11), (self.c_gps, 9),
                                               (self.c_link, 10), (self.c_rc, 10))):
            g1.addWidget(cell, 0, col)
            g1.setColumnStretch(col, stretch)
        body.addLayout(g1)

        self.chip_row = QHBoxLayout()
        self.chip_row.setSpacing(px(6))
        self.chip_arm = Chip("DISARMED", "warn", self)
        self.chip_conn = Chip("DISCONNECTED", "warn", self)
        self.chip_row.addWidget(self.chip_arm, 1)
        self.chip_row.addWidget(self.chip_conn, 1)
        body.addLayout(self.chip_row)

        g2 = QGridLayout()
        g2.setHorizontalSpacing(px(6))
        g2.setVerticalSpacing(0)
        self.c_raw = MetricCell("Raw", "0.0", "m")
        self.c_agl = MetricCell("AGL", "N/A")
        self.c_amsl = MetricCell("AMSL", "--")
        self.c_vs = MetricCell("Vert. speed", "0.00", "m/s")
        for col, (cell, stretch) in enumerate(((self.c_raw, 10), (self.c_agl, 10),
                                               (self.c_amsl, 10), (self.c_vs, 12.5))):
            g2.addWidget(cell, 0, col)
            g2.setColumnStretch(col, int(stretch * 2))
        body.addLayout(g2)

    def add_chip(self, widget: QWidget) -> None:
        """Third slot of the chip row (the GCS-control chip)."""
        self.chip_row.addWidget(widget, 1)

    def update_from(self, t: TelemetrySnapshot) -> None:
        connected = bool(t.connected)
        # battery
        have_batt = connected and t.battery_voltage > 0.5
        self.c_batt.set_line(0, f"{t.battery_voltage:.1f}" if have_batt else "--", "" if have_batt else "dim")
        pct = int(t.battery_percent)
        tone = "dim" if not have_batt else ("bad" if pct <= 20 else "warn" if pct <= 35 else "")
        self.c_batt.set_line(1, f"{pct}" if have_batt else "--", tone)
        # gps
        fix = str(t.fix_type or "NO_FIX")
        has_fix = connected and fix not in ("NO_FIX", "NO FIX", "")
        self.c_gps.set_line(0, str(int(t.satellites)) if has_fix else "--", "" if has_fix else "dim")
        self.c_gps.set_line(1, fix.replace("_", " ") if has_fix else "NO FIX", "" if has_fix else "dim")
        # link
        lq = int(t.link_quality)
        self.c_link.set_line(0, f"{lq}" if connected and lq > 0 else "--",
                             "" if connected and lq > 0 else "dim")
        # rc
        if not t.rc_receiver_present:
            self.c_rc.set_line(0, "N/A", "dim")
        elif not t.rc_receiver_healthy:
            self.c_rc.set_line(0, "LOST", "bad")
        else:
            self.c_rc.set_line(0, "OK", "ok")
        # state chips
        self.chip_arm.setText("ARMED" if t.armed else "DISARMED")
        self.chip_arm.set_state("ok" if t.armed else "warn")
        self.chip_conn.setText("CONNECTED" if connected else "DISCONNECTED")
        self.chip_conn.set_state("ok" if connected else "warn")
        # altitude tiles
        self.c_raw.set_line(0, f"{t.altitude:.1f}")
        rng = t.range_m
        has_rng = rng is not None and not (isinstance(rng, float) and math.isnan(rng))
        self.c_agl.set_line(0, f"{rng:.1f}" if has_rng else "N/A", "" if has_rng else "dim")
        has_amsl = t.last_global_time > 0
        self.c_amsl.set_line(0, f"{t.alt_amsl_m:.0f}" if has_amsl else "--", "" if has_amsl else "dim")
        vs = -t.vz
        self.c_vs.set_line(0, f"{(0.0 if abs(vs) < 0.005 else vs):.2f}")


class CommandsCard(GlassPanel):
    """The primary command grid. Buttons are exposed as attributes."""

    def __init__(self, parent=None):
        super().__init__("Commands", VehicleCard.NAME, False, parent)
        body = self.body_layout
        body.setSpacing(px(8))

        def btn(text: str, name: str = "", enabled: bool = True, tip: str = "") -> QPushButton:
            b = QPushButton(text, self)
            b.setMinimumHeight(px(34))
            if name:
                b.setObjectName(name)
            b.setEnabled(enabled)
            if tip:
                b.setToolTip(tip)
            return b

        na = "Not available: this station has no backend for it"
        self.btn_upload = btn("UPLOAD MISSION", "", False,
                              "Not available: plan a route on Tactical SLAM and press EXECUTE")
        self.btn_upload.setProperty("role", "off")
        body.addWidget(self.btn_upload)

        grid = QGridLayout()
        grid.setHorizontalSpacing(px(7))
        grid.setVerticalSpacing(px(7))
        self.btn_arm = btn("ARM", "btnArm")
        self.btn_disarm = btn("DISARM", "btnDisarm")
        self.btn_auto = btn("AUTO MODE", "btnNav", False, na)
        self.btn_hold = btn("HOLD", "btnNav")
        self.btn_continue = btn("CONTINUE", "btnNav", True, "Resume a paused route")
        self.btn_manual = btn("MANUAL MODE", "btnNav")
        self.btn_land = btn("LAND", "btnNav")
        self.btn_click_go = btn("CLICK && GO", "btnNav", True,
                                "Open Tactical SLAM and click the map to set a goal")
        self.btn_rtl = btn("RETURN HOME", "btnNav")
        self.btn_joystick = btn("JOYSTICK", "btnNav", False, na)
        for b in (self.btn_auto, self.btn_joystick):
            b.setProperty("role", "off")
        grid.addWidget(self.btn_arm, 0, 0)
        grid.addWidget(self.btn_disarm, 0, 1)
        grid.addWidget(self.btn_auto, 0, 2)
        grid.addWidget(self.btn_hold, 1, 0)
        grid.addWidget(self.btn_continue, 1, 1)
        grid.addWidget(self.btn_manual, 1, 2)
        grid.addWidget(self.btn_land, 2, 0, 1, 2)
        grid.addWidget(self.btn_click_go, 2, 2)
        grid.addWidget(self.btn_rtl, 3, 0, 1, 2)
        grid.addWidget(self.btn_joystick, 3, 2)
        for c in range(3):
            grid.setColumnStretch(c, 1)
        body.addLayout(grid)

        self.btn_kill = btn("EMERGENCY KILL", "btnKill")
        self.btn_kill.setMinimumHeight(px(38))
        body.addWidget(self.btn_kill)

    # State highlighting: "on" mirrors the vehicle, it never changes what a button does.
    @staticmethod
    def _flag(b: QPushButton, on: bool) -> None:
        if bool(b.property("on")) != on:
            b.setProperty("on", on)
            b.style().unpolish(b)
            b.style().polish(b)

    def reflect(self, armed: bool, mode: str) -> None:
        m = (mode or "").upper()
        self._flag(self.btn_arm, armed)
        self._flag(self.btn_disarm, not armed)
        self._flag(self.btn_hold, m in ("AUTO.LOITER", "LOITER", "HOLD"))
        self._flag(self.btn_land, m in ("AUTO.LAND", "LAND"))
        self._flag(self.btn_rtl, m in ("AUTO.RTL", "RTL"))
        self._flag(self.btn_manual, m == "MANUAL")
