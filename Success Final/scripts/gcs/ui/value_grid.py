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

ARMED IS RED, NOT GREEN:
  Consistent with the existing diagnostics page. Green reads as "safe", and the
  one state in which the propellers can spin is not the safe one.

USAGE:
  grid = ValueGridWidget(fields=cfg.ui.value_grid_fields, columns=3)
  grid.layout_changed.connect(save)
  grid.update_values(snapshot, extras={"rx": "12.4 kB/s"})
================================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QFrame, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton,
    QScrollArea, QDialog, QLineEdit, QListWidget, QListWidgetItem, QSpinBox,
    QComboBox, QDialogButtonBox, QSizePolicy,
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


def _g(obj, name, default=0):
    return getattr(obj, name, default)


def _pos_colour(t) -> Optional[str]:
    return BAD if _g(t, "position_stale", False) else None


def _batt_colour(t) -> Optional[str]:
    pct = _g(t, "battery_percent", 0)
    if pct <= 0:
        return DIM
    if pct <= 20:
        return BAD
    if pct <= 35:
        return WARN
    return OK


def _motor_fmt(i: int):
    def _f(t) -> str:
        pwms = _g(t, "motor_pwms", []) or []
        return f"{pwms[i]} µs" if i < len(pwms) else "--"
    return _f


def _motor_colour(i: int):
    def _c(t) -> Optional[str]:
        pwms = _g(t, "motor_pwms", []) or []
        if i >= len(pwms):
            return DIM
        pwm = pwms[i]
        if pwm > 1920:
            return WARN
        return OK if pwm > 1100 else DIM
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
                  lambda t: BAD if _g(t, "armed", False) else DIM),
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


# ─── Tiles ──────────────────────────────────────────────────────────

class ValueTile(QFrame):
    """One caption/value pair, plus its edit-mode controls."""

    remove_requested = pyqtSignal(str)
    move_requested = pyqtSignal(str, int)   # key, -1 left / +1 right

    def __init__(self, key: str, font_scale: float = 1.0, parent=None):
        super().__init__(parent)
        self.key = key
        self.setProperty("class", "cardFrame")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(px(10), px(6), px(10), px(7))
        lay.setSpacing(px(2))

        top = QHBoxLayout()
        top.setSpacing(px(4))
        self.lbl_caption = QLabel(caption_for(key), self)
        self.lbl_caption.setObjectName("fieldSubLabel")
        top.addWidget(self.lbl_caption, 1)

        self.btn_left = self._chip("◀", "Move this value left")
        self.btn_left.clicked.connect(lambda: self.move_requested.emit(self.key, -1))
        top.addWidget(self.btn_left)

        self.btn_right = self._chip("▶", "Move this value right")
        self.btn_right.clicked.connect(lambda: self.move_requested.emit(self.key, +1))
        top.addWidget(self.btn_right)

        self.btn_remove = self._chip("✕", "Remove this value from the grid")
        self.btn_remove.clicked.connect(lambda: self.remove_requested.emit(self.key))
        top.addWidget(self.btn_remove)
        lay.addLayout(top)

        self.lbl_value = QLabel("--", self)
        self.lbl_value.setObjectName("diagValue")
        self.lbl_value.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        lay.addWidget(self.lbl_value)

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

    def set_font_scale(self, font_scale: float) -> None:
        size = max(9, int(round(13 * font_scale * get_scale())))
        self._value_css = f"font-size: {size}px; font-weight: bold;"
        self._apply_value_style()

    def set_edit_mode(self, editing: bool) -> None:
        for b in (self.btn_left, self.btn_right, self.btn_remove):
            b.setVisible(editing)

    def set_value(self, text: str, colour: Optional[str]) -> None:
        if self.lbl_value.text() != text:
            self.lbl_value.setText(text)
        if colour != self._base_colour:
            self._base_colour = colour or ""
            self._apply_value_style()

    def _apply_value_style(self) -> None:
        colour = f"color: {self._base_colour};" if self._base_colour else ""
        self.lbl_value.setStyleSheet(self._value_css + colour)


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
    """A grid of telemetry tiles the operator chooses, orders and sizes."""

    layout_changed = pyqtSignal(list, int, float)   # keys, columns, font_scale

    def __init__(self, fields: Optional[Sequence[str]] = None, columns: int = 3,
                 font_scale: float = 1.0, parent=None):
        super().__init__(parent)
        self._keys: List[str] = self._sanitise(fields)
        self._columns = max(1, min(8, int(columns)))
        self._font_scale = float(font_scale)
        self._tiles: Dict[str, ValueTile] = {}
        self._editing = False

        root = QVBoxLayout(self)
        root.setContentsMargins(px(16), px(12), px(16), px(12))
        root.setSpacing(px(10))
        root.addLayout(self._build_header())

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        self._holder = QWidget()
        self._grid = QGridLayout(self._holder)
        self._grid.setHorizontalSpacing(px(10))
        self._grid.setVerticalSpacing(px(10))
        scroll.setWidget(self._holder)
        root.addWidget(scroll, 1)

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
        back rather than a blank workspace with no obvious way out. The "Add
        values..." button remains the way to build a smaller set deliberately.
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
        bar.setSpacing(px(8))

        title = QLabel("TELEMETRY VALUES", self)
        title.setObjectName("cardHeading")
        bar.addWidget(title)

        self.lbl_hint = QLabel("", self)
        self.lbl_hint.setObjectName("fieldSubLabel")
        bar.addWidget(self.lbl_hint)
        bar.addStretch()

        lbl_cols = QLabel("Columns", self)
        lbl_cols.setObjectName("fieldSubLabel")
        bar.addWidget(lbl_cols)

        self.spin_cols = QSpinBox(self)
        self.spin_cols.setRange(1, 8)
        self.spin_cols.setValue(self._columns)
        self.spin_cols.setFixedWidth(px(52))
        self.spin_cols.valueChanged.connect(self._on_columns_changed)
        bar.addWidget(self.spin_cols)

        lbl_size = QLabel("Text", self)
        lbl_size.setObjectName("fieldSubLabel")
        bar.addWidget(lbl_size)

        self.combo_size = QComboBox(self)
        self._size_options = [("Small", 0.85), ("Normal", 1.0),
                              ("Large", 1.25), ("Huge", 1.6)]
        for label, _ in self._size_options:
            self.combo_size.addItem(label)
        nearest = min(range(len(self._size_options)),
                      key=lambda i: abs(self._size_options[i][1] - self._font_scale))
        self.combo_size.setCurrentIndex(nearest)
        self.combo_size.setFixedWidth(px(86))
        self.combo_size.currentIndexChanged.connect(self._on_font_changed)
        bar.addWidget(self.combo_size)

        self.btn_add = QPushButton("Add…", self)
        self.btn_add.setToolTip("Choose which telemetry values this grid shows")
        self.btn_add.clicked.connect(self._open_picker)
        bar.addWidget(self.btn_add)

        self.btn_reset = QPushButton("Reset", self)
        self.btn_reset.setToolTip("Restore the default set of telemetry values")
        self.btn_reset.clicked.connect(self._reset_layout)
        bar.addWidget(self.btn_reset)

        self.btn_edit = QPushButton("Edit", self)
        self.btn_edit.setCheckable(True)
        self.btn_edit.setToolTip(
            "Show the per-tile remove and reorder controls")
        self.btn_edit.toggled.connect(self._set_edit_mode)
        bar.addWidget(self.btn_edit)

        self._set_hint()
        return bar

    # ── layout ──────────────────────────────────────────────────────

    def _rebuild(self) -> None:
        while self._grid.count():
            item = self._grid.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)
        self._tiles.clear()

        for i, key in enumerate(self._keys):
            tile = ValueTile(key, self._font_scale, self._holder)
            tile.remove_requested.connect(self._remove_key)
            tile.move_requested.connect(self._move_key)
            tile.set_edit_mode(self._editing)
            self._grid.addWidget(tile, i // self._columns, i % self._columns)
            self._tiles[key] = tile

        for c in range(self._columns):
            self._grid.setColumnStretch(c, 1)
        # Collapse the trailing space so tiles stay at their natural height
        # instead of stretching to fill a tall window.
        self._grid.setRowStretch(self._grid.rowCount(), 1)
        self._set_hint()

    def _set_hint(self) -> None:
        if not self._keys:
            self.lbl_hint.setText(
                "No values selected — use “Add values…”")
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
        self._rebuild()
        self._emit_changed()

    def _on_font_changed(self, index: int) -> None:
        if not 0 <= index < len(self._size_options):
            return
        self._font_scale = self._size_options[index][1]
        for tile in self._tiles.values():
            tile.set_font_scale(self._font_scale)
        self._emit_changed()

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
        self.spin_cols.blockSignals(True)
        self.spin_cols.setValue(self._columns)
        self.spin_cols.blockSignals(False)
        self.combo_size.blockSignals(True)
        self.combo_size.setCurrentIndex(1)
        self.combo_size.blockSignals(False)
        self._rebuild()
        self._emit_changed()

    # ── live values ─────────────────────────────────────────────────

    def update_values(self, snapshot, extras: Optional[Dict[str, str]] = None) -> None:
        """Refresh every visible tile. Called from the GUI tick.

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

    # ── state ───────────────────────────────────────────────────────

    def field_keys(self) -> List[str]:
        return list(self._keys)

    def columns(self) -> int:
        return self._columns

    def font_scale(self) -> float:
        return self._font_scale
