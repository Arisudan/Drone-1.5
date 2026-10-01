"""
================================================================================
MODULE: value_grid.py
PURPOSE: Operator-configurable telemetry value grid (the Diagnostics workspace)
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Diagnostics Workspace)
  * Upstream:      core/telemetry.TelemetrySnapshot, plus station-side extras
                   (link throughput, map source) the vehicle does not report
  * Downstream:    core/settings.UIConfig - the chosen layout is persisted

WHY THIS EXISTS:
  The Diagnostics page was thirty labels in nine hardcoded cards, chosen once by
  whoever wrote the page. Which thirty numbers matter is not a property of the
  software - it changes between a bench session (motor PWMs, ACK results), a VIO
  tuning flight (vision age, packet count, EKF2 state) and a battery endurance
  run (voltage, current, flight time). Every one of those sessions previously
  meant reading past twenty-seven irrelevant fields to find the three live ones.

THE FIELD REGISTRY IS THE CONTRACT:
  FIELDS maps a stable string key to how one telemetry value is presented:
  caption, formatter, and an optional colour rule. Keys are what get written to
  settings.json, so renaming one silently drops that tile from every saved
  layout - add a new key and leave the old one rather than renaming in place.

COLOUR IS RATIONED, AS EVERYWHERE ELSE IN THIS STATION:
  A colour rule returns None for "nothing notable", and a palette colour only
  when the value has crossed into a state the operator should act on. A grid
  where every tile is coloured conveys exactly as much as one where none are.

ARMED IS GREEN, DISARMED IS RED:
  The same convention as the header badge and the navigation footer, so the one
  state shown in three places is never coloured two ways. It is a STATE colour,
  not a severity: disarmed is the normal resting state, so it never raises a
  health light or a side bar (see FieldSpec.signals).

USAGE:
  grid = ValueGridWidget(fields=cfg.ui.value_grid_fields, columns=3)
  grid.layout_changed.connect(save)
  grid.update_values(snapshot, extras={"rx": "12.4 kB/s"})
================================================================================
"""

from __future__ import annotations

import html
import re
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QFrame, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton,
    QScrollArea, QDialog, QLineEdit, QListWidget, QListWidgetItem,
    QDialogButtonBox, QSizePolicy, QToolButton, QMenu, QAction, QActionGroup,
)

from ui.styles import PALETTE
from ui.scaling import px, get_scale

OK = PALETTE["ok"]
WARN = PALETTE["warn"]
BAD = PALETTE["danger"]
DIM = PALETTE["text_dim"]
ACCENT = PALETTE["accent"]
INFO = PALETTE["info"]


@dataclass(frozen=True)
class FieldSpec:
    """One presentable telemetry value.

    `fmt` receives the snapshot and returns display text; `colour` receives the
    snapshot and returns a hex string or None. Both take the whole snapshot
    rather than the field value so a rule can be relative to another field -
    position staleness colours the position readouts, not a 'stale' tile.
    """
    key: str
    caption: str
    category: str
    fmt: Callable[[object], str]
    colour: Optional[Callable[[object], Optional[str]]] = None
    # False for a STATE colour (armed green / disarmed red): it colours the value
    # but is not a caution or fault, so it must not light the card's health light
    # or the tile's side bar. Red-because-disarmed is the resting state, not a fault.
    signals: bool = True


def _g(obj, name, default=0):
    return getattr(obj, name, default)


def _pos_colour(t) -> Optional[str]:
    """Position is red only when a feed that WAS flowing has stopped. Before the
    first position packet (or with no link) it is simply unreported: grey, not a
    wall of red zeros that look like a fault and train the eye to ignore red."""
    if not _g(t, "connected", False) or _g(t, "last_position_time", 0.0) <= 0.0:
        return DIM
    return BAD if _g(t, "position_stale", False) else None


def _batt_colour(t) -> Optional[str]:
    """Grey-first: a healthy battery is plain white. Colour appears only when it
    needs attention (amber low, red critical) - so colour itself means something."""
    pct = _g(t, "battery_percent", 0)
    if pct <= 0:
        return DIM
    if pct <= 20:
        return BAD
    if pct <= 35:
        return WARN
    return None


def _motor_pwm(t, i: int):
    """PWM for Motor i+1 (0-based i), or None when unknown or stale.

    ``motor_pwms`` is output-indexed; the shared range maps it to motor order.
    A feed older than STALE_S is reported as unknown rather than as the last
    value - the Diagnostics tile must not show a dead stream as a number.
    """
    from core.motor_range import SHARED, STALE_S
    pwms = _g(t, "motor_pwms", []) or []
    if len(pwms) < 4:
        return None
    last = _g(t, "last_motor_time", 0.0)
    if last > 0.0 and _g(t, "motor_age", 0.0) > STALE_S:
        return None
    return SHARED.motor_pwms(pwms)[i]


def _motor_fmt(i: int):
    def _f(t) -> str:
        pwm = _motor_pwm(t, i)
        return f"{pwm} µs" if pwm is not None else "--"
    return _f


def _motor_colour(i: int):
    def _c(t) -> Optional[str]:
        from core import motor_range as mr
        pwm = _motor_pwm(t, i)
        if pwm is None:
            return DIM
        state = mr.SHARED.state(i + 1, pwm)
        if state == mr.SATURATED:
            return WARN
        # Nominal and high are ordinary running values: plain white. Idle / off
        # recede to grey.
        return None if state in (mr.NOMINAL, mr.HIGH) else DIM
    return _c


_LANDED = {0: "UNDEFINED", 1: "ON GROUND", 2: "IN AIR", 3: "TAKEOFF", 4: "LANDING"}


def _build_fields() -> Dict[str, FieldSpec]:
    specs: List[FieldSpec] = [
        # ── Link ────────────────────────────────────────────────────
        FieldSpec("connected", "Link", "Link",
                  lambda t: "CONNECTED" if _g(t, "connected", False) else "NO LINK",
                  lambda t: OK if _g(t, "connected", False) else BAD),
        FieldSpec("sysid", "System / Comp ID", "Link",
                  lambda t: f"{_g(t, 'system_id', 0)} / {_g(t, 'component_id', 0)}"),
        FieldSpec("heartbeat_age", "Heartbeat age", "Link",
                  lambda t: f"{_g(t, 'heartbeat_age', 0.0):.1f} s",
                  lambda t: BAD if _g(t, "heartbeat_age", 0.0) > 3.0 else None),
        FieldSpec("link_quality", "Link quality", "Link",
                  lambda t: f"{_g(t, 'link_quality', 0)} %"),

        # ── Flight state ────────────────────────────────────────────
        FieldSpec("arm_state", "Arm state", "Flight state",
                  lambda t: "ARMED" if _g(t, "armed", False) else "DISARMED",
                  lambda t: OK if _g(t, "armed", False) else BAD,
                  signals=False),
        FieldSpec("flight_mode", "Flight mode", "Flight state",
                  lambda t: _g(t, "flight_mode", "") or "--",
                  lambda t: ACCENT if _g(t, "connected", False) else DIM),
        FieldSpec("flight_time", "Flight time", "Flight state",
                  lambda t: f"{int(_g(t, 'flight_time_sec', 0.0))} s"),
        FieldSpec("landed_state", "Landed state", "Flight state",
                  lambda t: _LANDED.get(_g(t, "landed_state", 0), "?")),
        FieldSpec("airborne", "Airborne", "Flight state",
                  lambda t: "YES" if _g(t, "is_airborne", False) else "NO",
                  lambda t: WARN if _g(t, "is_airborne", False) else DIM),

        # ── Power ───────────────────────────────────────────────────
        FieldSpec("volt", "Battery", "Power",
                  lambda t: f"{_g(t, 'battery_voltage', 0.0):.2f} V", _batt_colour),
        FieldSpec("curr", "Current", "Power",
                  lambda t: f"{_g(t, 'battery_current', 0.0):.1f} A"),
        FieldSpec("pct", "Remaining", "Power",
                  lambda t: f"{_g(t, 'battery_percent', 0)} %", _batt_colour),

        # ── Attitude ────────────────────────────────────────────────
        FieldSpec("roll", "Roll", "Attitude",
                  lambda t: f"{_g(t, 'roll', 0.0):+.1f}°"),
        FieldSpec("pitch", "Pitch", "Attitude",
                  lambda t: f"{_g(t, 'pitch', 0.0):+.1f}°"),
        FieldSpec("yaw", "Yaw", "Attitude",
                  lambda t: f"{_g(t, 'yaw', 0.0):+.1f}°"),
        FieldSpec("heading", "Heading", "Attitude",
                  lambda t: f"{_g(t, 'heading', 0.0):.1f}°"),

        # ── Position ────────────────────────────────────────────────
        FieldSpec("pos_x", "North (x)", "Position",
                  lambda t: f"{_g(t, 'x', 0.0):+.3f} m", _pos_colour),
        FieldSpec("pos_y", "East (y)", "Position",
                  lambda t: f"{_g(t, 'y', 0.0):+.3f} m", _pos_colour),
        FieldSpec("pos_z", "Down (z)", "Position",
                  lambda t: f"{_g(t, 'z', 0.0):+.3f} m", _pos_colour),
        FieldSpec("alt", "Altitude AGL", "Position",
                  lambda t: f"{_g(t, 'altitude', 0.0):.3f} m", _pos_colour),
        FieldSpec("position_age", "Position age", "Position",
                  lambda t: f"{_g(t, 'position_age', 0.0):.1f} s", _pos_colour),

        # ── Velocity ────────────────────────────────────────────────
        FieldSpec("vx", "Vx", "Velocity",
                  lambda t: f"{_g(t, 'vx', 0.0):+.2f} m/s"),
        FieldSpec("vy", "Vy", "Velocity",
                  lambda t: f"{_g(t, 'vy', 0.0):+.2f} m/s"),
        FieldSpec("vz", "Vz", "Velocity",
                  lambda t: f"{_g(t, 'vz', 0.0):+.2f} m/s"),
        FieldSpec("gspeed", "Ground speed", "Velocity",
                  lambda t: f"{_g(t, 'ground_speed', 0.0):.2f} m/s"),

        # ── Vision / EKF2 ───────────────────────────────────────────
        FieldSpec("vio_health", "VIO stream", "Vision / EKF2",
                  lambda t: "LOCKED (10 Hz)" if _g(t, "d435i_vio_health", False)
                  else "NO VISION DATA",
                  lambda t: OK if _g(t, "d435i_vio_health", False) else BAD),
        FieldSpec("vio_age", "Last vision packet", "Vision / EKF2",
                  lambda t: (f"{_g(t, 'd435i_vio_age', 0.0):.1f} s ago"
                             if _g(t, "last_vision_time", 0.0) else "never"),
                  lambda t: OK if _g(t, "d435i_vio_health", False) else BAD),
        FieldSpec("vio_fps", "VIO rate", "Vision / EKF2",
                  lambda t: f"{_g(t, 'd435i_vio_fps', 0.0):.1f} Hz"),
        FieldSpec("vision_packets", "Vision packets", "Vision / EKF2",
                  lambda t: str(_g(t, "vision_packets_count", 0))),
        FieldSpec("ekf2", "EKF2 fusion", "Vision / EKF2",
                  lambda t: "ACTIVE (indoor lock)" if _g(t, "ekf2_vision_fused", False)
                  else "WAITING",
                  lambda t: OK if _g(t, "ekf2_vision_fused", False) else WARN),

        # ── Motors ──────────────────────────────────────────────────
        FieldSpec("m1", "M1  FR CCW", "Motors", _motor_fmt(0), _motor_colour(0)),
        FieldSpec("m2", "M2  RL CCW", "Motors", _motor_fmt(1), _motor_colour(1)),
        FieldSpec("m3", "M3  FL CW", "Motors", _motor_fmt(2), _motor_colour(2)),
        FieldSpec("m4", "M4  RR CW", "Motors", _motor_fmt(3), _motor_colour(3)),

        # ── RC link ─────────────────────────────────────────────────
        FieldSpec("rc_health", "RC receiver", "RC link",
                  lambda t: ("HEALTHY" if _g(t, "rc_receiver_healthy", False)
                             else ("PRESENT" if _g(t, "rc_receiver_present", False)
                                   else "NO DATA")),
                  lambda t: OK if _g(t, "rc_receiver_healthy", False) else DIM),
        # 255 is this receiver's permanent "not reported" value - shown as such
        # rather than as a suspiciously perfect signal strength.
        FieldSpec("rc_rssi", "RC RSSI", "RC link",
                  lambda t: ("n/r" if _g(t, "rc_rssi", -1) in (-1, 255)
                             else str(_g(t, "rc_rssi", -1)))),
        FieldSpec("rc_channels", "RC channels", "RC link",
                  lambda t: str(_g(t, "rc_channel_count", 0))),

        # ── GPS ─────────────────────────────────────────────────────
        # Informational indoors - this airframe flies on vision, so a missing
        # fix is dimmed rather than flagged.
        FieldSpec("sats", "Satellites", "GPS",
                  lambda t: str(_g(t, "satellites", 0)),
                  lambda t: OK if _g(t, "fix_type", "") in
                  ("3D_FIX", "DGPS", "RTK_FLOAT", "RTK_FIXED") else DIM),
        FieldSpec("hdop", "HDOP", "GPS", lambda t: f"{_g(t, 'hdop', 0.0):.1f}"),
        FieldSpec("fix", "Fix type", "GPS",
                  lambda t: _g(t, "fix_type", "NO_FIX"),
                  lambda t: OK if _g(t, "fix_type", "") in
                  ("3D_FIX", "DGPS", "RTK_FLOAT", "RTK_FIXED") else DIM),
        FieldSpec("lat", "Latitude", "GPS",
                  lambda t: f"{_g(t, 'latitude', 0.0):.6f}"),
        FieldSpec("lon", "Longitude", "GPS",
                  lambda t: f"{_g(t, 'longitude', 0.0):.6f}"),

        # ── Commands ────────────────────────────────────────────────
        FieldSpec("last_ack_cmd", "Last command", "Commands",
                  lambda t: _g(t, "last_ack_cmd_name", "") or "--"),
        FieldSpec("last_ack_result", "Last result", "Commands",
                  lambda t: _g(t, "last_ack_result", "") or "--",
                  lambda t: (OK if _g(t, "last_ack_result_code", 0) == 0
                             else (BAD if _g(t, "last_ack_result", "") else DIM))),

        # ── Companion ───────────────────────────────────────────────
        FieldSpec("cpu", "Companion CPU", "Companion",
                  lambda t: f"{_g(t, 'cpu_percent', 0.0):.0f} %",
                  lambda t: WARN if _g(t, "cpu_percent", 0.0) > 85 else None),
        FieldSpec("ram", "Companion RAM", "Companion",
                  lambda t: f"{_g(t, 'ram_percent', 0.0):.0f} %",
                  lambda t: WARN if _g(t, "ram_percent", 0.0) > 85 else None),
        FieldSpec("temp", "Companion temp", "Companion",
                  lambda t: f"{_g(t, 'temp_c', 0.0):.0f} °C",
                  lambda t: WARN if _g(t, "temp_c", 0.0) > 75 else None),
    ]
    return {s.key: s for s in specs}


FIELDS: Dict[str, FieldSpec] = _build_fields()

# Station-side values with no home on the snapshot: measured by the GCS about
# its own link, not reported by the vehicle. Supplied through update_values'
# `extras` argument and rendered exactly like any other tile.
EXTRA_FIELDS: Dict[str, str] = {
    "rx": "RX rate",
    "tx": "TX rate",
    "map_source": "Map source",
    "audio": "Audio alerts",
}

# What a fresh install shows: the thirty fields the hardcoded Diagnostics page
# carried, in the same reading order. Changing the default must not change what
# an existing operator sees - a saved layout always wins over this list.
DEFAULT_FIELDS: List[str] = [
    "sysid", "connected", "arm_state", "flight_mode", "flight_time",
    "rx", "tx",
    "vio_health", "vio_age", "ekf2",
    "pos_x", "pos_y", "pos_z", "alt",
    "vx", "vy", "vz", "gspeed",
    "m1", "m2", "m3", "m4",
    "roll", "pitch", "yaw", "heading",
    "volt", "curr", "pct",
    "sats", "hdop", "fix",
]


def caption_for(key: str) -> str:
    spec = FIELDS.get(key)
    if spec is not None:
        return spec.caption
    return EXTRA_FIELDS.get(key, key)


def category_for(key: str) -> str:
    spec = FIELDS.get(key)
    return spec.category if spec is not None else "Station"


def known_keys() -> List[str]:
    return list(FIELDS.keys()) + list(EXTRA_FIELDS.keys())


def category_order() -> List[str]:
    """Every category, in the order it first appears in the field registry -
    the grid groups its tiles in this same order, so the layout reads the
    same regardless of which order the operator happened to add fields in.
    "Station" (extras with no FieldSpec) always sorts last: it is a catch-all,
    not a deliberate section."""
    order: List[str] = []
    for spec in FIELDS.values():
        if spec.category not in order:
            order.append(spec.category)
    order.append("Station")
    return order


# ─── Tiles ──────────────────────────────────────────────────────────

def _badge_qss(colour: str) -> str:
    """A small filled pill in the exact same colour language as the header's
    own status badges (top_status_strip.py) - keyed off the same four
    status colours used everywhere in this station, not a generic hex-to-rgba
    conversion, so a badge here can never quietly drift from what the header
    looks like."""
    if colour == OK:
        bg, border = "rgba(35, 134, 54, 0.13)", PALETTE["ok_dim"]
    elif colour == WARN:
        bg, border = "rgba(158, 106, 3, 0.13)", PALETTE["warn"]
    elif colour == BAD:
        bg, border = "rgba(218, 54, 51, 0.13)", PALETTE["danger_dim"]
    else:
        bg, border = PALETTE["bg_panel"], PALETTE["border_soft"]
    fg = colour or PALETTE["text_dim"]
    return (f"background-color: {bg}; color: {fg}; "
            f"border: 1px solid {border}; border-radius: {px(4)}px; "
            f"padding: {px(2)}px {px(8)}px; font-weight: bold;")


# A leading numeric token (sign, digits, optional decimal) - "+1.200", "16.40",
# "-0.4", "1000". Whatever follows is the candidate unit suffix.
_NUM_RE = re.compile(r"^([+-]?\d[\d,]*\.?\d*)(.*)$")


# Values that mean "nothing reported": drawn as plain muted text. As pills they
# looked like four tiny status badges all saying nothing.
PLACEHOLDERS = frozenset({"", "--", "—", "n/r"})


def split_value(text: str):
    """(number, unit) for a plain measurement, or None for status text.

    Status text ("ARMED", "NO LINK") is different in kind from a measurement, so
    it gets a pill rather than a big number. A value counts as a status whenever
    it has no leading number, or what follows the number still contains a digit
    (e.g. "1 / 1") - that is presentation noise to keep whole, not a unit."""
    m = _NUM_RE.match(text)
    if m and not any(ch.isdigit() for ch in m.group(2)):
        return m.group(1), m.group(2)
    return None


def severity_of(colour: Optional[str]) -> int:
    """2 = fault (red), 1 = caution (amber), 0 = nothing to act on."""
    return 2 if colour == BAD else 1 if colour == WARN else 0


_LED_IDLE = "#6e7681"


class ValueTile(QFrame):
    """One value cell: a small-caps caption over a large value, plus the
    edit-mode controls.

    Caption on top and number below is the whole hierarchy: the caption is quiet
    (small, grey, spaced capitals) so the number is the loudest thing in the
    cell. The unit is a size down and muted. A thin bar at the left appears ONLY
    when the value is in caution or fault - the same severity language as the
    alarm card - so a glance down the grid finds the abnormal cells by shape
    before reading anything.
    """

    remove_requested = pyqtSignal(str)
    move_requested = pyqtSignal(str, int)   # key, -1 left / +1 right

    BASE_VALUE_PX = 19

    def __init__(self, key: str, font_scale: float = 1.0, parent=None):
        super().__init__(parent)
        self.key = key
        self.setProperty("class", "cellFrame")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._caption = caption_for(key)
        spec = FIELDS.get(key)
        self._signals = spec.signals if spec is not None else True

        root = QHBoxLayout(self)
        root.setContentsMargins(px(2), px(3), px(2), px(3))
        root.setSpacing(px(8))

        self.bar = QFrame(self)
        self.bar.setFixedWidth(px(3))
        self.bar.setVisible(False)
        root.addWidget(self.bar)

        col = QVBoxLayout()
        col.setSpacing(px(2))
        col.setContentsMargins(0, 0, 0, 0)

        head = QHBoxLayout()
        head.setSpacing(px(4))
        self.lbl_caption = QLabel(self._caption.upper(), self)
        self.lbl_caption.setObjectName("diagCaption")
        head.addWidget(self.lbl_caption)
        head.addStretch(1)
        self.btn_left = self._chip("◀", "Move this value left")
        self.btn_left.clicked.connect(lambda: self.move_requested.emit(self.key, -1))
        head.addWidget(self.btn_left)
        self.btn_right = self._chip("▶", "Move this value right")
        self.btn_right.clicked.connect(lambda: self.move_requested.emit(self.key, +1))
        head.addWidget(self.btn_right)
        self.btn_remove = self._chip("✕", "Remove this value from the grid")
        self.btn_remove.clicked.connect(lambda: self.remove_requested.emit(self.key))
        head.addWidget(self.btn_remove)
        col.addLayout(head)

        val = QHBoxLayout()
        val.setSpacing(px(6))
        self.lbl_value = QLabel(self)
        self.lbl_value.setObjectName("diagValue")
        self.lbl_value.setTextFormat(Qt.RichText)
        self.lbl_value.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        val.addWidget(self.lbl_value)

        # A real widget, not another HTML span: Qt's rich text engine does not
        # support border/border-radius/padding on an inline <span>, only on a
        # block element - a genuine rounded, filled pill needs its own QLabel
        # styled with real QSS. Shown only for status-style values.
        self.lbl_badge = QLabel(self)
        self.lbl_badge.setAlignment(Qt.AlignCenter)
        self.lbl_badge.setVisible(False)
        val.addWidget(self.lbl_badge)
        val.addStretch(1)
        col.addLayout(val)
        root.addLayout(col, 1)

        self._raw_text = "--"
        self._base_colour = ""
        self.set_font_scale(font_scale)
        self.set_edit_mode(False)

    def _chip(self, glyph: str, tip: str) -> QPushButton:
        b = QPushButton(glyph, self)
        b.setToolTip(tip)
        b.setCursor(Qt.PointingHandCursor)
        b.setFixedSize(px(16), px(16))
        b.setStyleSheet(
            f"QPushButton {{ background: {PALETTE['bg_input']}; color: {PALETTE['text_dim']};"
            f" border: 1px solid {PALETTE['border_soft']}; border-radius: {px(3)}px;"
            f" font-size: {max(7, int(8 * get_scale()))}px; padding: 0px; }}"
            f"QPushButton:hover {{ color: {PALETTE['text_bright']};"
            f" border-color: {PALETTE['border_hover']}; }}")
        return b

    @property
    def severity(self) -> int:
        return severity_of(self._base_colour) if self._signals else 0

    def set_font_scale(self, font_scale: float) -> None:
        self._value_size = max(11, int(round(self.BASE_VALUE_PX * font_scale * get_scale())))
        self._apply_value_style()

    def set_edit_mode(self, editing: bool) -> None:
        for b in (self.btn_left, self.btn_right, self.btn_remove):
            b.setVisible(editing)

    def set_value(self, text: str, colour: Optional[str]) -> None:
        if text != self._raw_text or colour != self._base_colour:
            self._raw_text = text
            self._base_colour = colour or ""
            self._apply_value_style()

    def _apply_value_style(self) -> None:
        big = self._value_size
        small = max(9, int(round(big * 0.56)))
        number_colour = self._base_colour or PALETTE["text_bright"]
        dim = PALETTE["text_dim"]

        sev = self.severity
        self.bar.setVisible(sev > 0)
        if sev:
            self.bar.setStyleSheet(
                f"background: {self._base_colour}; border: none; border-radius: {px(1)}px;")

        if self._raw_text.strip() in PLACEHOLDERS:
            self.lbl_value.setText(
                f'<span style="font-size:{big}px; font-weight:bold; '
                f'color:{PALETTE["text_muted"]};">--</span>')
            self.lbl_value.setVisible(True)
            self.lbl_badge.setVisible(False)
            return
        parts = split_value(self._raw_text)
        if parts is not None:
            number, unit = parts
            self.lbl_value.setText(
                f'<span style="font-size:{big}px; font-weight:bold; '
                f'color:{number_colour};">{html.escape(number)}</span>'
                f'<span style="font-size:{small}px; color:{dim};"> '
                f'{html.escape(unit.strip())}</span>')
            self.lbl_value.setVisible(True)
            self.lbl_badge.setVisible(False)
        else:
            self.lbl_value.setVisible(False)
            self.lbl_badge.setText(self._raw_text)
            self.lbl_badge.setStyleSheet(
                _badge_qss(number_colour) + f" font-size:{max(10, int(big * 0.6))}px;")
            self.lbl_badge.setVisible(True)


# ─── Glance strip ───────────────────────────────────────────────────

class GlanceReadout(QFrame):
    """One big readout in the glance strip: caption, large value, small sub-line."""

    BASE_VALUE_PX = 30

    def __init__(self, caption: str, parent=None):
        super().__init__(parent)
        self.setObjectName("glanceCell")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(px(16), px(10), px(16), px(10))
        lay.setSpacing(px(3))

        self.lbl_caption = QLabel(caption.upper(), self)
        self.lbl_caption.setObjectName("diagCaption")
        lay.addWidget(self.lbl_caption)

        row = QHBoxLayout()
        row.setSpacing(px(8))
        self.lbl_main = QLabel("--", self)
        self.lbl_main.setObjectName("diagValue")
        self.lbl_main.setTextFormat(Qt.RichText)
        row.addWidget(self.lbl_main)
        self.lbl_badge = QLabel(self)
        self.lbl_badge.setAlignment(Qt.AlignCenter)
        self.lbl_badge.setVisible(False)
        row.addWidget(self.lbl_badge)
        row.addStretch(1)
        lay.addLayout(row)

        self.lbl_sub = QLabel("\u00a0", self)
        self.lbl_sub.setObjectName("fieldSubLabel")
        lay.addWidget(self.lbl_sub)
        lay.addStretch(1)

        self.main_text = "--"
        self.main_colour = ""
        self.sub_text = ""
        self._last = None

    def set_content(self, main: str, colour: Optional[str], sub: str = "",
                    sub_colour: Optional[str] = None, badge: bool = False) -> None:
        state = (main, colour, sub, sub_colour, badge)
        if state == self._last:
            return
        self._last = state
        self.main_text, self.main_colour, self.sub_text = main, colour or "", sub
        big = max(14, int(round(self.BASE_VALUE_PX * get_scale())))
        small = max(10, int(round(big * 0.5)))
        dim = PALETTE["text_dim"]
        fg = colour or PALETTE["text_bright"]

        if not badge and main.strip() in PLACEHOLDERS:
            self.lbl_main.setText(
                f'<span style="font-size:{big}px; font-weight:bold; '
                f'color:{PALETTE["text_muted"]};">--</span>')
            self.lbl_main.setVisible(True)
            self.lbl_badge.setVisible(False)
            self._finish_sub(sub, sub_colour, dim)
            return
        parts = None if badge else split_value(main)
        if parts is not None:
            number, unit = parts
            self.lbl_main.setText(
                f'<span style="font-size:{big}px; font-weight:bold; color:{fg};">'
                f'{html.escape(number)}</span>'
                f'<span style="font-size:{small}px; color:{dim};"> '
                f'{html.escape(unit.strip())}</span>')
            self.lbl_main.setVisible(True)
            self.lbl_badge.setVisible(False)
        elif badge:
            self.lbl_main.setVisible(False)
            self.lbl_badge.setText(main)
            self.lbl_badge.setStyleSheet(
                _badge_qss(fg) + f" font-size:{max(11, int(big * 0.5))}px;")
            self.lbl_badge.setVisible(True)
        else:                                   # plain text, e.g. a flight mode
            self.lbl_main.setText(
                f'<span style="font-size:{int(big * 0.78)}px; font-weight:bold; '
                f'color:{fg};">{html.escape(main)}</span>')
            self.lbl_main.setVisible(True)
            self.lbl_badge.setVisible(False)

        self._finish_sub(sub, sub_colour, dim)

    def _finish_sub(self, sub: str, sub_colour: Optional[str], dim: str) -> None:
        # The sub-line is ALWAYS present (a non-breaking space when empty) so every
        # cell in the strip is the same height and the captions line up.
        self.lbl_sub.setText(sub or "\u00a0")
        self.lbl_sub.setStyleSheet(f"color: {sub_colour or dim};")


class PrimaryStrip(QFrame):
    """The six numbers you must be able to read in one look: altitude, ground
    speed, battery, link, mode and flight time - the cockpit's "speedometer".

    Not configurable on purpose. The grid below is the place for choice; this is
    the place that is always the same, so muscle memory works. The cells follow
    the same grey-first colour rule as every tile.
    """

    ORDER = ("alt", "speed", "batt", "link", "mode", "time")
    CAPTIONS = {"alt": "Altitude", "speed": "Ground speed", "batt": "Battery",
                "link": "Link", "mode": "Mode", "time": "Flight time"}
    MIN_CELL_PX = 150

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("glanceStrip")
        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setSpacing(0)
        self.cells: Dict[str, GlanceReadout] = {
            k: GlanceReadout(self.CAPTIONS[k], self) for k in self.ORDER}
        self._per_row = 0
        self.reflow(10_000)

    def reflow(self, width: int) -> int:
        """One row when it fits, otherwise two rows of three. Returns cells/row."""
        per_row = 6 if width >= px(self.MIN_CELL_PX) * 6 else 3
        if per_row == self._per_row:
            return per_row
        self._per_row = per_row
        for cell in self.cells.values():
            self._grid.removeWidget(cell)
        for c in range(len(self.ORDER)):
            self._grid.setColumnStretch(c, 0)    # stale stretches squeezed the cells
        for i, key in enumerate(self.ORDER):
            cell = self.cells[key]
            r, c = divmod(i, per_row)
            self._grid.addWidget(cell, r, c)
            cell.setStyleSheet(
                "" if c == 0 else
                f"QFrame#glanceCell {{ border-left: 1px solid {PALETTE['border_soft']}; }}")
            self._grid.setColumnStretch(c, 1)
        return per_row

    def update_from(self, t) -> None:
        connected = bool(_g(t, "connected", False))
        pos_col = _pos_colour(t)

        self.cells["alt"].set_content(
            f"{_g(t, 'altitude', 0.0):.2f} m", pos_col)
        self.cells["speed"].set_content(
            f"{_g(t, 'ground_speed', 0.0):.2f} m/s", None if connected else DIM)

        pct = _g(t, "battery_percent", 0)
        self.cells["batt"].set_content(
            f"{pct} %" if pct > 0 else "--", _batt_colour(t),
            f"{_g(t, 'battery_voltage', 0.0):.2f} V · {_g(t, 'battery_current', 0.0):.1f} A")

        age = _g(t, "heartbeat_age", 0.0)
        self.cells["link"].set_content(
            "CONNECTED" if connected else "NO LINK", OK if connected else BAD,
            f"heartbeat {age:.1f} s" if connected and age < 100 else "", None, badge=True)

        armed = bool(_g(t, "armed", False))
        mode = _g(t, "flight_mode", "") or "--"
        self.cells["mode"].set_content(
            mode, None if connected else DIM,
            "ARMED" if armed else "DISARMED", OK if armed else BAD)

        secs = int(_g(t, "flight_time_sec", 0.0))
        self.cells["time"].set_content(
            f"{secs // 60:02d}:{secs % 60:02d}", None if connected else DIM)


# ─── System cards ───────────────────────────────────────────────────

class SystemCard(QFrame):
    """One system's values in a single card: Power, Attitude, Position, Motors...

    The header carries a health light that summarises the cells inside it: grey
    when nothing needs attention (the normal state - grey-first, so the light is
    only worth looking at when it is not grey), amber for caution, red for fault.
    """

    INNER_COLUMNS = 2

    def __init__(self, category: str, parent=None):
        super().__init__(parent)
        self.category = category
        self.setProperty("class", "cardFrame")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.tiles: List[ValueTile] = []

        lay = QVBoxLayout(self)
        lay.setContentsMargins(px(12), px(8), px(12), px(10))
        lay.setSpacing(px(6))

        head = QHBoxLayout()
        head.setSpacing(px(8))
        self.lbl_title = QLabel(category.upper(), self)
        self.lbl_title.setObjectName("cardGroupTitle")
        head.addWidget(self.lbl_title)
        head.addStretch(1)
        self.led = QLabel(self)
        self.led.setFixedSize(px(9), px(9))
        head.addWidget(self.led, 0, Qt.AlignVCenter)
        lay.addLayout(head)

        rule = QFrame(self)
        rule.setObjectName("hDivider")
        rule.setFixedHeight(1)
        lay.addWidget(rule)

        self._grid = QGridLayout()
        self._grid.setHorizontalSpacing(px(16))
        self._grid.setVerticalSpacing(px(8))
        lay.addLayout(self._grid)

        self._health = -1
        self.refresh_health()

    def set_tiles(self, tiles: List[ValueTile]) -> None:
        self.tiles = list(tiles)
        cols = self.INNER_COLUMNS if len(tiles) > 1 else 1
        for i, tile in enumerate(tiles):
            self._grid.addWidget(tile, i // cols, i % cols)
        for c in range(cols):
            self._grid.setColumnStretch(c, 1)

    @property
    def health(self) -> int:
        return max((t.severity for t in self.tiles), default=0)

    def refresh_health(self) -> None:
        h = self.health
        if h == self._health:
            return
        self._health = h
        colour = {2: BAD, 1: WARN}.get(h, _LED_IDLE)
        self.led.setStyleSheet(f"background: {colour}; border-radius: {px(4)}px;")
        worst = [t._caption for t in self.tiles if t.severity == h and h > 0]
        self.led.setToolTip(
            {2: "Fault: ", 1: "Caution: "}.get(h, "") + ", ".join(worst)
            if h else "Nothing needs attention")



# ─── Picker ─────────────────────────────────────────────────────────

class ValuePickerDialog(QDialog):
    """Searchable, category-grouped chooser for grid fields.

    Pre-checks whatever is already on the grid, so the dialog is an editor for
    the whole selection rather than an append-only 'add' action - removing three
    tiles and adding two is one trip through here.
    """

    def __init__(self, selected: Sequence[str], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Choose telemetry values")
        self.setMinimumSize(px(420), px(520))

        lay = QVBoxLayout(self)
        lay.setContentsMargins(px(12), px(12), px(12), px(12))
        lay.setSpacing(px(8))

        self.search = QLineEdit(self)
        self.search.setPlaceholderText("Filter values…")
        self.search.textChanged.connect(self._apply_filter)
        lay.addWidget(self.search)

        self.list = QListWidget(self)
        self.list.setAlternatingRowColors(False)
        lay.addWidget(self.list, 1)

        chosen = set(selected)
        grouped: Dict[str, List[str]] = {}
        for key in known_keys():
            grouped.setdefault(category_for(key), []).append(key)

        self._rows: List[QListWidgetItem] = []
        for category in sorted(grouped):
            header = QListWidgetItem(category.upper())
            header.setFlags(Qt.NoItemFlags)
            header.setData(Qt.UserRole, None)
            self.list.addItem(header)
            self._rows.append(header)
            for key in grouped[category]:
                item = QListWidgetItem(f"    {caption_for(key)}")
                item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
                item.setCheckState(Qt.Checked if key in chosen else Qt.Unchecked)
                item.setData(Qt.UserRole, key)
                self.list.addItem(item)
                self._rows.append(item)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel, Qt.Horizontal, self)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        lay.addWidget(buttons)

    def _apply_filter(self, text: str) -> None:
        needle = text.strip().lower()
        for item in self._rows:
            key = item.data(Qt.UserRole)
            if key is None:
                # Category headers stay only while unfiltered; with a filter
                # active they are noise between the two rows that matched.
                item.setHidden(bool(needle))
                continue
            haystack = f"{caption_for(key)} {key} {category_for(key)}".lower()
            item.setHidden(bool(needle) and needle not in haystack)

    def selected_keys(self) -> List[str]:
        out = []
        for item in self._rows:
            key = item.data(Qt.UserRole)
            if key is not None and item.checkState() == Qt.Checked:
                out.append(key)
        return out


# ─── The grid ───────────────────────────────────────────────────────

class ValueGridWidget(QWidget):
    """The Diagnostics workspace: a glance strip, then one card per system.

    Structure, top to bottom:
      * a one-line heading and a single "Layout" menu (it used to be five
        controls at the same level as the title),
      * the PrimaryStrip - six readouts readable in one look,
      * system cards (Power, Attitude, Position, ...) packed into as many columns
        as the width allows, each holding the operator's chosen values for that
        system.

    The operator still chooses, orders and sizes the values; the registry, the
    persisted layout and the signal are unchanged. "Columns" is now a MAXIMUM:
    the grid uses fewer when the window cannot fit that many without clipping,
    which is what used to push the third column off-screen at the smallest size.
    """

    layout_changed = pyqtSignal(list, int, float)   # keys, max columns, font_scale

    MIN_CARD_PX = 290
    GAP_PX = 12

    def __init__(self, fields: Optional[Sequence[str]] = None, columns: int = 3,
                 font_scale: float = 1.0, parent=None):
        super().__init__(parent)
        self._keys: List[str] = self._sanitise(fields)
        self._columns = max(1, min(8, int(columns)))
        self._font_scale = float(font_scale)
        self._tiles: Dict[str, ValueTile] = {}
        self._cards: List[SystemCard] = []
        self._active_cols = 0
        self._editing = False
        self._size_options = [("Small", 0.85), ("Normal", 1.0),
                              ("Large", 1.25), ("Huge", 1.6)]

        root = QVBoxLayout(self)
        root.setContentsMargins(px(16), px(12), px(16), px(12))
        root.setSpacing(px(10))
        root.addLayout(self._build_header())

        self._scroll = QScrollArea(self)
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.NoFrame)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        holder = QWidget()
        hv = QVBoxLayout(holder)
        hv.setContentsMargins(0, 0, px(4), 0)
        hv.setSpacing(px(self.GAP_PX))
        self.strip = PrimaryStrip(holder)
        hv.addWidget(self.strip)
        self._cards_host = QWidget(holder)
        self._cards_layout = QHBoxLayout(self._cards_host)
        self._cards_layout.setContentsMargins(0, 0, 0, 0)
        self._cards_layout.setSpacing(px(self.GAP_PX))
        hv.addWidget(self._cards_host)
        hv.addStretch(1)
        self._holder = holder
        self._scroll.setWidget(holder)
        root.addWidget(self._scroll, 1)

        self._rebuild()

    # ── construction ────────────────────────────────────────────────

    @staticmethod
    def _sanitise(fields: Optional[Sequence[str]]) -> List[str]:
        """Drop unknown keys and duplicates, preserving order.

        A settings file written by a newer build can name a field this build has
        never heard of. Ignoring it is right; refusing to start, or rendering an
        empty tile that never updates, is not.

        An empty selection falls back to DEFAULT_FIELDS. There is deliberately
        no distinction between "never configured" and "the operator removed
        every tile": both serialise to [] in settings.json, so the distinction
        could not survive a restart even if this function tried to make one.
        Falling back is also the forgiving reading - an operator who has cleared
        the entire grid has most likely made a mistake, and gets the default set
        back rather than a blank workspace with no obvious way out.
        """
        if fields is None:
            return list(DEFAULT_FIELDS)
        known = set(known_keys())
        seen = set()
        out = [k for k in fields
               if k in known and not (k in seen or seen.add(k))]
        return out or list(DEFAULT_FIELDS)

    def _build_header(self) -> QHBoxLayout:
        bar = QHBoxLayout()
        bar.setSpacing(px(10))

        title = QLabel("TELEMETRY VALUES", self)
        title.setObjectName("cardHeading")
        # One line, always: a non-wrapping label can never be laid out narrower
        # than its text, so the toolbar gives way instead of the heading.
        title.setWordWrap(False)
        bar.addWidget(title)

        self.lbl_hint = QLabel("", self)
        self.lbl_hint.setObjectName("fieldSubLabel")
        bar.addWidget(self.lbl_hint)
        bar.addStretch()

        # Everything that used to be five controls on this row lives in one menu.
        self.btn_layout = QToolButton(self)
        self.btn_layout.setObjectName("layoutBtn")
        self.btn_layout.setText("Layout  ▾")
        self.btn_layout.setToolTip("Columns, text size, which values to show, edit mode")
        self.btn_layout.setPopupMode(QToolButton.InstantPopup)
        self.btn_layout.setCursor(Qt.PointingHandCursor)
        self.btn_layout.setMenu(self._build_menu())
        bar.addWidget(self.btn_layout)

        self._set_hint()
        return bar

    def _build_menu(self) -> QMenu:
        menu = QMenu(self)

        cols = menu.addMenu("Max columns")
        self._col_group = QActionGroup(self)
        self._col_actions: Dict[int, QAction] = {}
        for n in range(1, 7):
            a = cols.addAction(f"{n}" + ("  (default)" if n == 3 else ""))
            a.setCheckable(True)
            a.setChecked(n == self._columns)
            a.triggered.connect(lambda _c, n=n: self._on_columns_changed(n))
            self._col_group.addAction(a)
            self._col_actions[n] = a

        sizes = menu.addMenu("Text size")
        self._size_group = QActionGroup(self)
        self._size_actions: List[QAction] = []
        nearest = min(range(len(self._size_options)),
                      key=lambda i: abs(self._size_options[i][1] - self._font_scale))
        for i, (label, _) in enumerate(self._size_options):
            a = sizes.addAction(label)
            a.setCheckable(True)
            a.setChecked(i == nearest)
            a.triggered.connect(lambda _c, i=i: self._on_font_changed(i))
            self._size_group.addAction(a)
            self._size_actions.append(a)

        menu.addSeparator()
        self.act_add = menu.addAction("Add or remove values…")
        self.act_add.triggered.connect(self._open_picker)
        self.act_edit = menu.addAction("Edit tiles (reorder / remove)")
        self.act_edit.setCheckable(True)
        self.act_edit.toggled.connect(self._set_edit_mode)
        menu.addSeparator()
        self.act_reset = menu.addAction("Reset to default")
        self.act_reset.triggered.connect(self._reset_layout)
        return menu

    # ── layout ──────────────────────────────────────────────────────

    def _rebuild(self) -> None:
        """Recreate the tiles and the cards from the selected keys."""
        for card in self._cards:
            card.setParent(None)
            card.deleteLater()
        self._cards.clear()
        self._tiles.clear()

        by_category: Dict[str, List[str]] = {}
        for key in self._keys:
            by_category.setdefault(category_for(key), []).append(key)

        for category in category_order():
            cat_keys = by_category.get(category)
            if not cat_keys:
                continue
            card = SystemCard(category, self._cards_host)
            tiles = []
            for key in cat_keys:
                tile = ValueTile(key, self._font_scale, card)
                tile.remove_requested.connect(self._remove_key)
                tile.move_requested.connect(self._move_key)
                tile.set_edit_mode(self._editing)
                self._tiles[key] = tile
                tiles.append(tile)
            card.set_tiles(tiles)
            self._cards.append(card)

        self._active_cols = 0           # force a fresh placement
        self._place_cards()
        self._set_hint()

    def _effective_columns(self) -> int:
        """How many card columns fit: never more than the operator's maximum,
        never more than there are cards, and never so many that a card would be
        squeezed under its minimum width (which is what clipped the third column
        at the smallest window)."""
        avail = max(1, self._scroll.viewport().width())
        gap = px(self.GAP_PX)
        min_w = px(int(self.MIN_CARD_PX * max(1.0, self._font_scale ** 0.5)))
        fit = max(1, (avail + gap) // (min_w + gap))
        return max(1, min(self._columns, fit, max(1, len(self._cards))))

    def _place_cards(self) -> None:
        """Pack cards into columns, each card going to the currently shortest
        column (masonry), so mixed-height cards leave no ragged holes."""
        cols = self._effective_columns()
        if cols == self._active_cols and self._cards_layout.count():
            return
        self._active_cols = cols

        while self._cards_layout.count():
            item = self._cards_layout.takeAt(0)
            lay = item.layout()
            if lay is not None:
                while lay.count():
                    lay.takeAt(0)           # widgets stay parented to the host
                lay.deleteLater()

        columns = [QVBoxLayout() for _ in range(cols)]
        heights = [0] * cols
        for col in columns:
            col.setSpacing(px(self.GAP_PX))
            col.setContentsMargins(0, 0, 0, 0)
        for card in self._cards:
            i = min(range(cols), key=heights.__getitem__)
            columns[i].addWidget(card)
            card.show()
            heights[i] += card.sizeHint().height() + px(self.GAP_PX)
        for col in columns:
            col.addStretch(1)
            self._cards_layout.addLayout(col, 1)

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self.strip.reflow(self._scroll.viewport().width())
        self._place_cards()

    def showEvent(self, ev):
        super().showEvent(ev)
        self.strip.reflow(self._scroll.viewport().width())
        self._place_cards()

    def _set_hint(self) -> None:
        if not self._keys:
            self.lbl_hint.setText("No values selected")
        else:
            self.lbl_hint.setText(f"{len(self._keys)} values")

    def _set_edit_mode(self, editing: bool) -> None:
        self._editing = bool(editing)
        for tile in self._tiles.values():
            tile.set_edit_mode(self._editing)

    # ── mutation ────────────────────────────────────────────────────

    def _emit_changed(self) -> None:
        self.layout_changed.emit(list(self._keys), self._columns, self._font_scale)

    def _remove_key(self, key: str) -> None:
        if key in self._keys:
            self._keys.remove(key)
            self._rebuild()
            self._emit_changed()

    def _move_key(self, key: str, delta: int) -> None:
        if key not in self._keys:
            return
        i = self._keys.index(key)
        j = i + delta
        if not 0 <= j < len(self._keys):
            return
        self._keys[i], self._keys[j] = self._keys[j], self._keys[i]
        self._rebuild()
        self._emit_changed()

    def _on_columns_changed(self, value: int) -> None:
        self._columns = max(1, min(8, int(value)))
        self._sync_menu()
        self._active_cols = 0
        self._place_cards()
        self._emit_changed()

    def _on_font_changed(self, index: int) -> None:
        if not 0 <= index < len(self._size_options):
            return
        self._font_scale = self._size_options[index][1]
        for tile in self._tiles.values():
            tile.set_font_scale(self._font_scale)
        self._sync_menu()
        self._active_cols = 0
        self._place_cards()
        self._emit_changed()

    def _sync_menu(self) -> None:
        a = self._col_actions.get(self._columns)
        if a is not None:
            a.setChecked(True)
        nearest = min(range(len(self._size_options)),
                      key=lambda i: abs(self._size_options[i][1] - self._font_scale))
        self._size_actions[nearest].setChecked(True)

    def _open_picker(self) -> None:
        dlg = ValuePickerDialog(self._keys, self)
        if dlg.exec_() != QDialog.Accepted:
            return
        chosen = dlg.selected_keys()
        # Keep the operator's existing order for anything still selected and
        # append what is new, so adding one field does not reshuffle the grid
        # back into registry order.
        kept = [k for k in self._keys if k in chosen]
        added = [k for k in chosen if k not in kept]
        self._keys = kept + added
        self._rebuild()
        self._emit_changed()

    def _reset_layout(self) -> None:
        self._keys = list(DEFAULT_FIELDS)
        self._columns = 3
        self._font_scale = 1.0
        self._sync_menu()
        self._rebuild()
        self._emit_changed()

    # ── live values ─────────────────────────────────────────────────

    def update_values(self, snapshot, extras: Optional[Dict[str, str]] = None) -> None:
        """Refresh the glance strip and every visible tile. Called from the GUI tick.

        Only the tiles actually on the grid are formatted: a field the operator
        removed costs nothing per frame, which is the other half of why this is
        configurable rather than a fixed thirty.
        """
        extras = extras or {}
        for key, tile in self._tiles.items():
            spec = FIELDS.get(key)
            if spec is not None:
                try:
                    text = spec.fmt(snapshot)
                    colour = spec.colour(snapshot) if spec.colour else None
                except Exception:
                    text, colour = "ERR", BAD
            else:
                value = extras.get(key)
                if isinstance(value, tuple):
                    text, colour = value
                else:
                    text, colour = (value if value is not None else "--"), None
            tile.set_value(text, colour)
        for card in self._cards:
            card.refresh_health()
        try:
            self.strip.update_from(snapshot)
        except Exception:
            pass      # a bad snapshot field must not take the whole tab down

    # ── state ───────────────────────────────────────────────────────

    def field_keys(self) -> List[str]:
        return list(self._keys)

    def columns(self) -> int:
        return self._columns

    def active_columns(self) -> int:
        """Columns actually in use right now (<= columns())."""
        return self._active_cols

    def font_scale(self) -> float:
        return self._font_scale
