"""
================================================================================
MODULE: config_tab.py
PURPOSE: Settings Workspace - edit, validate and persist station configuration
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Config Workspace)
  * Upstream:      core/settings.py (typed, validated, JSON-backed)
  * Downstream:    drone_gcs.py applies the saved values

LAYOUT follows the Walle GCS settings workspace: a column of titled cards, each
a grid of label/field rows, with a save control at the foot.

EVERY FIELD HERE IS REAL. Walle's settings tab carries sections for hardware
this airframe does not have (WebRTC signalling, gimbal, RTH altitude for a
GPS vehicle); those are omitted rather than shown as controls that change
nothing. What is left maps one-to-one onto values the station actually reads.

WORKSPACE: a section list on the left (with a search box that filters every
field by caption), one card per section on the right. Each field shows its unit
as a suffix, a dot while it differs from what is saved, a "restart" tag when a
change only takes effect after the station is restarted, and its own validation
message as you type. The footer counts unsaved changes. Each card can reset
itself to defaults, and two modest presets (Bench, Indoor flight) fill in the
values that differ between those two situations.

SAVING VALIDATES FIRST: core.settings.save_settings raises on an out-of-range
value and the tab reports it inline, so a battery-critical threshold above the
warning threshold is refused at the point of entry rather than at the point of
flight.
================================================================================
"""

from __future__ import annotations

from dataclasses import fields, is_dataclass
from typing import Dict, List, Optional, Tuple

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QLineEdit,
    QPushButton, QComboBox, QCheckBox, QScrollArea, QListWidget, QListWidgetItem, QSizePolicy,
)

from core.settings import GCSSettings, load_settings, save_settings, settings_path
from ui.scaling import px
from ui.theme import GlassPanel

# (card title, {field: (caption, unit)}). Every visible field has a caption here,
# so no raw key ever reaches the screen; units are a suffix, not part of the label.
FIELD_INFO = {
    "profile": ("Drone profile", {
        "name": ("Airframe name", ""), "frame": ("Frame type", ""),
        "battery_cells": ("Battery cells", "S"),
        "battery_capacity_mah": ("Battery capacity", "mAh"),
        "drone_id": ("Drone ID", "")}),
    "connection": ("MAVLink connection", {
        "default_network": ("Default network", ""), "host": ("Companion IP", ""),
        "protocol": ("Protocol", ""), "udp_port": ("UDP port (flight)", ""),
        "tcp_port": ("TCP port (bench)", ""), "source_system": ("GCS system ID", ""),
        "watchdog_port": ("Radxa watchdog port", "")}),
    "video": ("Video & map bridge", {
        "stream_url": ("FPV stream URL", ""),
        "map_bridge_host": ("Map bridge host", ""),
        "map_bridge_port": ("Map bridge port (TCP)", ""),
        "jpeg_port": ("MJPEG port", ""),
        "wfb_rtp_port": ("wfb-ng video RTP port", ""),
        "wfb_ts_port": ("wfb-ng video local TS port", "")}),
    "arbiter": ("GCS command arbitration", {
        "enabled": ("Arbitrate GCS commands", ""),
        "max_step_h_m": ("Max move per command (horizontal)", "m"),
        "max_step_v_m": ("Max move per command (vertical)", "m"),
        "ceiling_m": ("Ceiling above take-off point", "m"),
        "box_half_m": ("Keep-in box half-width", "m"),
        "stick_takeover": ("Stick movement = pilot takeover", "x full"),
        "vision_max_age_s": ("Lidar pose max age", "s"),
        "estimator_max_age_s": ("Estimator status max age", "s"),
        "position_max_age_s": ("Local position max age", "s")}),
    "slam": ("SLAM & navigation", {
        "cruise_altitude_m": ("Cruise altitude", "m"),
        "robot_radius_m": ("Robot radius", "m"), "cell_size_m": ("Grid cell size", "m"),
        "treat_unknown_as_obstacle": ("Treat unknown space as obstacle", ""),
        "rviz_config": ("RViz config file (blank = bundled)", "")}),
    "limits": ("Command limits", {
        "takeoff_alt_min_m": ("Takeoff minimum", "m"), "takeoff_alt_max_m": ("Takeoff maximum", "m"),
        "move_max_delta_m": ("Largest single move", "m"),
        "move_frame": ("Move axes (body = forward/right, ned = North/East)", ""),
        "move_yaw_offset_deg": ("Move heading correction", "\u00b0"),
        "move_invert_forward": ("Reverse move forward/back", ""),
        "move_invert_right": ("Reverse move right/left", "")}),
    "alerts": ("Alert thresholds", {
        "batt_warn_pct": ("Battery warning", "%"), "batt_crit_pct": ("Battery critical", "%"),
        "vision_stale_s": ("LIO stale after", "s"),
        "map_stall_s": ("Map stalled after", "s")}),
    "ui": ("Display", {
        "scale": ("UI scale (0 = auto)", "x"),
        "value_grid_columns": ("Value grid columns", ""),
        "value_grid_font_scale": ("Value grid text size", "x")}),
    "audio": ("Audio alerts", {
        "enabled": ("Audio alerts", ""), "tones_enabled": ("Warning tones", ""),
        "speech_enabled": ("Spoken messages", ""),
        "min_repeat_s": ("Repeat the same alert after", "s")}),
    "actuator": ("ESP32 servo actuator", {
        "host": ("ESP32 IP", ""), "port": ("HTTP port", "")}),
}

# Kept for callers that read the old table (section -> (TITLE, {field: caption})).
CAPTIONS = {sec: (title.upper(), {k: v[0] for k, v in caps.items()})
            for sec, (title, caps) in FIELD_INFO.items()}

# Fields that belong to a settings section but are not typed in by hand here.
# value_grid_fields is a list of telemetry keys edited directly on the value
# grid itself; rendering it as a text box would invite someone to hand-write a
# Python list into a QLineEdit. Hidden fields are carried across a Save
# untouched - see _read_from_editors, which builds a fresh settings object and
# would otherwise silently reset the operator's grid layout to the default
# every time any unrelated setting was saved.
HIDDEN_FIELDS = {
    ("ui", "value_grid_fields"),
}

CHOICES = {
    ("connection", "protocol"): ["udp", "tcp", "udpin"],
    ("limits", "move_frame"): ["body", "ned"],
    ("connection", "default_network"): ["HTIC_RND", "DroneBridge5", "DroneNet", "wfb-ng", "Jetson_AP"],
    ("profile", "frame"): ["Quad X", "Quad +", "Hex X", "Hex +", "Y6", "Octo X"],
}

# What drone_gcs._on_settings_saved applies immediately. Anything not listed here
# is read once at start-up, so a change needs a restart to take effect.
LIVE_SECTIONS = {"limits", "alerts", "audio", "actuator"}
LIVE_FIELDS = {("video", "stream_url")}

# Two situations this station is used in; applied to the editors only (nothing
# is saved until Save). There is deliberately no "Field/outdoor" preset: the
# vehicle is indoor and vision-only.
PRESETS = {
    "Bench": {"connection.protocol": "tcp", "audio.enabled": False},
    "Indoor flight": {"connection.protocol": "udp", "audio.enabled": True,
                      "audio.tones_enabled": True, "audio.speech_enabled": True},
}


def needs_restart(section: str, name: str) -> bool:
    return section not in LIVE_SECTIONS and (section, name) not in LIVE_FIELDS


def parse_value(current, raw: str):
    """Text -> a value of the same type as `current`. Raises ValueError."""
    if isinstance(current, bool):
        return raw.strip().lower() in ("1", "true", "yes", "on")
    if isinstance(current, int):
        return int(float(raw))
    if isinstance(current, float):
        return float(raw)
    return raw


def validate_sections(cfg: GCSSettings) -> Dict[str, str]:
    """Run every section's validate() and map each message to a field key
    ("section.field"), or to "section" when it is about several fields."""
    problems: Dict[str, str] = {}
    for section in cfg.sections():
        obj = getattr(cfg, section)
        if not is_dataclass(obj):
            continue
        try:
            obj.validate()
        except ValueError as exc:
            msg = str(exc)
            key = section
            # The "section.field" form names the offender; a bare field name in
            # the text ("... must exceed takeoff_alt_min_m") is only a fallback.
            for needle in (lambda f: f"{section}.{f.name}", lambda f: f.name):
                hit = next((f for f in fields(obj) if needle(f) in msg), None)
                if hit is not None:
                    key = f"{section}.{hit.name}"
                    break
            problems[key] = msg
    return problems


class ConfigTabWidget(QWidget):
    """Editor over core.settings.GCSSettings."""

    settings_saved = pyqtSignal(object)   # emits the saved GCSSettings

    def __init__(self, settings: Optional[GCSSettings] = None, parent=None):
        super().__init__(parent)
        self.settings = settings or load_settings()
        self._editors: Dict[str, QWidget] = {}
        self._rows: Dict[str, Tuple[QWidget, ...]] = {}   # key -> (caption, editor, ...)
        self._dirty_marks: Dict[str, QLabel] = {}
        self._restart_marks: Dict[str, QLabel] = {}
        self._error_labels: Dict[str, QLabel] = {}
        self._cards: Dict[str, QFrame] = {}
        self._loading = False

        # v2 layout: a 'Categories' glass panel on the left, and one glass panel on the
        # right that shows the selected category's card (every card still exists; a
        # search shows every matching card instead of just the selected one).
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(px(12))

        self.pnl_cats = GlassPanel("Categories", "", True, self)
        self.pnl_cats.setFixedWidth(px(230))
        left = self.pnl_cats.body_layout
        left.setSpacing(px(8))
        self.search = QLineEdit(self)
        self.search.setPlaceholderText("Search settings...")
        self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(self._apply_search)
        left.addWidget(self.search)
        self.nav = QListWidget(self)
        self.nav.setObjectName("cfgNav")
        self.nav.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.nav.setSpacing(0)
        self.nav.setFocusPolicy(Qt.NoFocus)
        for section in self.settings.sections():
            item = QListWidgetItem(FIELD_INFO[section][0])
            item.setData(Qt.UserRole, section)
            self.nav.addItem(item)
        self.nav.currentItemChanged.connect(self._on_nav)
        self.nav.setMinimumHeight(px(44) * self.nav.count() + px(8))
        left.addWidget(self.nav, 1)

        lbl_preset = QLabel("Preset", self)
        lbl_preset.setObjectName("cfgUnit")
        left.addWidget(lbl_preset)
        self.combo_preset = QComboBox(self)
        self.combo_preset.addItems(list(PRESETS))
        left.addWidget(self.combo_preset)
        self.btn_preset = QPushButton("APPLY PRESET", self)
        self.btn_preset.setToolTip("Fills in the fields; nothing is saved until you press Save.")
        self.btn_preset.clicked.connect(self.apply_preset)
        left.addWidget(self.btn_preset)
        root.addWidget(self.pnl_cats, 0)

        self.pnl_form = GlassPanel("Video", "validated \u00b7 persisted", False, self)
        self.pnl_form.setMaximumWidth(px(660))
        fl = self.pnl_form.body_layout
        fl.setSpacing(px(8))
        self.scroll = QScrollArea(self)
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QFrame.NoFrame)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        holder = QWidget()
        self.cards_layout = QVBoxLayout(holder)
        self.cards_layout.setContentsMargins(0, 0, px(6), 0)
        self.cards_layout.setSpacing(px(10))
        for section in self.settings.sections():
            card = self._build_card(section)
            self._cards[section] = card
            self.cards_layout.addWidget(card)
        self.lbl_no_match = QLabel("No setting matches your search.", self)
        self.lbl_no_match.setObjectName("cfgUnit")
        self.lbl_no_match.setVisible(False)
        self.cards_layout.addWidget(self.lbl_no_match)
        self.cards_layout.addStretch(1)
        self.scroll.setWidget(holder)
        fl.addWidget(self.scroll, 1)

        self.lbl_status = QLabel(f"Settings file: {settings_path()}", self)
        self.lbl_status.setObjectName("fieldSubLabel")
        self.lbl_status.setWordWrap(True)
        fl.addWidget(self.lbl_status)

        foot = QHBoxLayout()
        foot.setSpacing(px(8))
        self.btn_save = QPushButton("SAVE", self)
        self.btn_save.setObjectName("btnConnect")
        self.btn_save.setMinimumHeight(px(32))
        self.btn_save.clicked.connect(self._on_save)
        foot.addWidget(self.btn_save)
        btn_reload = QPushButton("REVERT", self)
        btn_reload.setMinimumHeight(px(32))
        btn_reload.clicked.connect(self._on_reload)
        foot.addWidget(btn_reload)
        foot.addSpacing(px(8))
        self.lbl_changes = QLabel("", self)
        self.lbl_changes.setObjectName("cfgDirty")
        foot.addWidget(self.lbl_changes)
        foot.addStretch(1)
        fl.addLayout(foot)

        col = QVBoxLayout()
        col.setContentsMargins(0, 0, 0, 0)
        col.addWidget(self.pnl_form)
        # Top-aligned panels that stop at their content, as in the design.
        root.addLayout(col, 1)
        root.addStretch(0)
        left_wrap_h = px(380)
        self.pnl_form.setMaximumHeight(px(640))
        root.setAlignment(self.pnl_cats, Qt.AlignTop)
        col.addStretch(0)

        self._load_into_editors()
        self.nav.setCurrentRow(0)

    # ── construction ────────────────────────────────────────────────

    def _build_card(self, section: str) -> QFrame:
        title, info = FIELD_INFO[section]
        card = QFrame(self)
        card.setObjectName("cfgCard")
        cv = QVBoxLayout(card)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.setSpacing(px(6))

        head = QHBoxLayout()
        heading = QLabel(title.upper(), self)
        heading.setObjectName("cardHeading")
        heading.setVisible(False)        # shown only while a search lists several cards
        card._heading = heading
        head.addWidget(heading)
        head.addStretch()
        reset = QPushButton("RESET SECTION", self)
        reset.setObjectName("cfgReset")
        reset.setToolTip("Put every field in this section back to its default (not saved until you Save).")
        reset.clicked.connect(lambda _=False, s=section: self.reset_section(s))
        head.addWidget(reset)
        cv.addLayout(head)

        body = QGridLayout()
        body.setHorizontalSpacing(px(10))
        body.setVerticalSpacing(px(6))
        body.setColumnStretch(1, 1)
        body.setColumnMinimumWidth(0, px(170))

        obj = getattr(self.settings, section)
        visible = [f for f in fields(obj) if (section, f.name) not in HIDDEN_FIELDS]
        r = 0
        for f in visible:
            key = f"{section}.{f.name}"
            caption, unit = info.get(f.name, (f.name.replace("_", " ").capitalize(), ""))
            cap = QLabel(caption, self)
            cap.setObjectName("cfgLabel")
            cap.setWordWrap(True)
            editor = self._editor_for(section, f.name, getattr(obj, f.name), caption)
            self._editors[key] = editor
            self._connect_change(editor, key)

            cell = QHBoxLayout()
            cell.setContentsMargins(0, 0, 0, 0)
            cell.setSpacing(8)
            cell.addWidget(editor, 1)
            u = QLabel(unit, self)
            u.setObjectName("cfgUnit")
            cell.addWidget(u)
            dirty = QLabel("", self)
            dirty.setObjectName("cfgDirty")
            self._dirty_marks[key] = dirty
            cell.addWidget(dirty)
            restart = QLabel("", self)
            restart.setObjectName("cfgRestart")
            self._restart_marks[key] = restart
            cell.addWidget(restart)
            body.addWidget(cap, r, 0)
            body.addLayout(cell, r, 1)
            err = QLabel("", self)
            err.setObjectName("cfgError")
            err.setWordWrap(True)
            err.setVisible(False)
            self._error_labels[key] = err
            body.addWidget(err, r + 1, 1)
            self._rows[key] = (cap, editor, u, dirty, restart, err)
            r += 2
        # Cross-field messages for the section (e.g. "timeouts must be > 0").
        sec_err = QLabel("", self)
        sec_err.setObjectName("cfgError")
        sec_err.setWordWrap(True)
        sec_err.setVisible(False)
        self._error_labels[section] = sec_err
        body.addWidget(sec_err, r, 0, 1, 2)
        cv.addLayout(body)
        return card

    def _editor_for(self, section: str, name: str, value, caption: str = "") -> QWidget:
        choices = CHOICES.get((section, name))
        if choices is not None:
            combo = QComboBox(self)
            combo.addItems(choices)
            return combo
        if isinstance(value, bool):
            box = QCheckBox("On", self)
            box.setObjectName("cfgToggle")
            return box
        edit = QLineEdit(self)
        if isinstance(value, (int, float)):
            edit.setAlignment(Qt.AlignRight)
            edit.setProperty("num", True)
        edit.setMinimumWidth(px(160))
        edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        return edit
        # Scaled: a fixed 220px cap clipped the stream URL as soon as the
        # UI scale raised the font above the size it was measured at.
        # Genuinely fixed (min == max), not just capped: a maximum alone left
        # the floor at Qt's default, so a narrow grid column under space
        # pressure could still squeeze this well below the intended 220px.
        edit.setFixedWidth(px(220))
        return edit

    def _connect_change(self, editor: QWidget, key: str) -> None:
        if isinstance(editor, QCheckBox):
            editor.toggled.connect(lambda _v: self._on_edited())
        elif isinstance(editor, QComboBox):
            editor.currentIndexChanged.connect(lambda _i: self._on_edited())
        else:
            editor.textChanged.connect(lambda _t: self._on_edited())

    # ── value transfer ──────────────────────────────────────────────

    def _editor_text(self, w: QWidget) -> str:
        if isinstance(w, QCheckBox):
            return "true" if w.isChecked() else "false"
        if isinstance(w, QComboBox):
            return w.currentText()
        return w.text().strip()

    def _set_editor(self, w: QWidget, v) -> None:
        if isinstance(w, QCheckBox):
            w.setChecked(bool(v))
        elif isinstance(w, QComboBox):
            idx = w.findText(str(v))
            w.setCurrentIndex(idx if idx >= 0 else 0)
        else:
            w.setText(str(v))
            # A value wider than the field's own (scaled) max width
            # otherwise shows its tail, not its head - setText() alone
            # leaves the cursor (and the visible scroll position) at
            # the end. Real case: the FPV stream URL field showed
            # "6.101.84:8080/video", hiding the "http://..." prefix.
            w.setCursorPosition(0)

    def _load_into_editors(self) -> None:
        self._loading = True
        try:
            for section in self.settings.sections():
                obj = getattr(self.settings, section)
                for f in fields(obj):
                    if (section, f.name) in HIDDEN_FIELDS:
                        continue
                    self._set_editor(self._editors[f"{section}.{f.name}"], getattr(obj, f.name))
        finally:
            self._loading = False
        self._on_edited()

    def _read_from_editors(self) -> GCSSettings:
        """Build a fresh settings object; leaves self.settings untouched until
        it validates, so a rejected edit cannot half-apply."""
        cfg, errors = self._build()
        if errors:
            raise ValueError(next(iter(errors.values())))
        return cfg

    def _build(self) -> Tuple[GCSSettings, Dict[str, str]]:
        """(settings built from the editors, {field key: parse error})."""
        cfg = GCSSettings()
        errors: Dict[str, str] = {}
        for section in cfg.sections():
            obj = getattr(cfg, section)
            if not is_dataclass(obj):
                continue
            for f in fields(obj):
                if (section, f.name) in HIDDEN_FIELDS:
                    # Preserved verbatim from the live settings: this editor
                    # never showed it, so it has no opinion about its value.
                    setattr(obj, f.name, getattr(getattr(self.settings, section), f.name))
                    continue
                key = f"{section}.{f.name}"
                w = self._editors[key]
                current = getattr(obj, f.name)
                raw = self._editor_text(w)
                try:
                    setattr(obj, f.name, parse_value(current, raw))
                except ValueError:
                    caption = FIELD_INFO[section][1].get(f.name, (key, ""))[0]
                    errors[key] = f"{caption}: {raw!r} is not a number"
        return cfg, errors

    # ── change tracking / validation ────────────────────────────────

    def changed_keys(self) -> List[str]:
        out = []
        for section in self.settings.sections():
            obj = getattr(self.settings, section)
            for f in fields(obj):
                if (section, f.name) in HIDDEN_FIELDS:
                    continue
                key = f"{section}.{f.name}"
                if self._editor_text(self._editors[key]) != self._saved_text(getattr(obj, f.name)):
                    out.append(key)
        return out

    @staticmethod
    def _saved_text(v) -> str:
        if isinstance(v, bool):
            return "true" if v else "false"
        return str(v)

    def current_problems(self) -> Dict[str, str]:
        cfg, errors = self._build()
        problems = dict(errors)
        if not errors:
            problems.update(validate_sections(cfg))
        else:
            # Validate what did parse, so an unrelated field's error still shows.
            for k, v in validate_sections(cfg).items():
                problems.setdefault(k, v)
        return problems

    def _on_edited(self) -> None:
        if self._loading:
            return
        changed = set(self.changed_keys())
        for key, dirty in self._dirty_marks.items():
            on = key in changed
            dirty.setText("●" if on else "")
            dirty.setToolTip("Changed - not saved yet" if on else "")
            sec, name = key.split(".", 1)
            self._restart_marks[key].setText("restart needed" if on and needs_restart(sec, name) else "")
        problems = self.current_problems()
        for key, lab in self._error_labels.items():
            msg = problems.get(key, "")
            lab.setText(msg)
            lab.setVisible(bool(msg))
            ed = self._editors.get(key)
            if ed is not None:
                ed.setProperty("invalid", bool(msg))
                ed.style().unpolish(ed)
                ed.style().polish(ed)
        self.btn_save.setEnabled(not problems)
        n = len(changed)
        self.lbl_changes.setText(f"{n} unsaved change{'s' if n != 1 else ''}" if n else "")

    # ── navigation / search ─────────────────────────────────────────

    def _show_selected_card(self) -> None:
        """No search: only the selected category's card. Search: every matching card."""
        item = self.nav.currentItem()
        sel = item.data(Qt.UserRole) if item is not None else None
        searching = bool(self.search.text().strip())
        for section, card in self._cards.items():
            card._heading.setVisible(searching)
            if not searching:
                card.setVisible(section == sel)
        if sel is not None:
            self.pnl_form.set_title(FIELD_INFO[sel][0] if not searching else "Search results")

    def _on_nav(self, item, _prev=None) -> None:
        if item is None:
            return
        if self.search.text().strip():
            card = self._cards.get(item.data(Qt.UserRole))
            if card is not None and card.isVisible():
                self.scroll.ensureWidgetVisible(card, 0, 0)
                self.scroll.verticalScrollBar().setValue(card.y())
            return
        self._show_selected_card()
        self.scroll.verticalScrollBar().setValue(0)

    def _apply_search(self, text: str) -> None:
        q = text.strip().lower()
        any_card = False
        for section, card in self._cards.items():
            title, info = FIELD_INFO[section]
            show_all = not q or q in title.lower()
            visible_rows = 0
            for key, (cap, editor, u, dirty, restart, err) in self._rows.items():
                if not key.startswith(section + "."):
                    continue
                name = key.split(".", 1)[1]
                caption = info.get(name, (name, ""))[0]
                match = show_all or q in caption.lower() or q in key.lower()
                for w in (cap, editor, u, dirty, restart):
                    w.setVisible(match)
                if not match:
                    err.setVisible(False)
                visible_rows += 1 if match else 0
            card.setVisible(visible_rows > 0)
            any_card = any_card or visible_rows > 0
        self.lbl_no_match.setVisible(not any_card)
        if not q:
            self._on_edited()      # restore error labels hidden while filtering
            self._show_selected_card()
        else:
            self._show_selected_card()

    # ── actions ─────────────────────────────────────────────────────

    def reset_section(self, section: str) -> None:
        defaults = getattr(GCSSettings(), section)
        for f in fields(defaults):
            if (section, f.name) in HIDDEN_FIELDS:
                continue
            self._set_editor(self._editors[f"{section}.{f.name}"], getattr(defaults, f.name))

    def apply_preset(self, name: Optional[str] = None) -> None:
        name = name if isinstance(name, str) else self.combo_preset.currentText()
        for key, value in PRESETS.get(name, {}).items():
            self._set_editor(self._editors[key], value)
        self.lbl_status.setStyleSheet("")
        self.lbl_status.setText(f"Preset '{name}' filled in - review, then Save Settings.")

    def _on_save(self) -> None:
        problems = self.current_problems()
        if problems:
            self.lbl_status.setText(f"Not saved - {next(iter(problems.values()))}")
            self.lbl_status.setStyleSheet("color: #ff5a4f;")
            return
        try:
            cfg = self._read_from_editors()
            path = save_settings(cfg)
        except ValueError as exc:
            self.lbl_status.setText(f"Not saved - {exc}")
            self.lbl_status.setStyleSheet("color: #ff5a4f;")
            return
        restart = [k for k in self.changed_keys() if needs_restart(*k.split(".", 1))]
        self.settings = cfg
        self.lbl_status.setStyleSheet("color: #46d06a;")
        text = f"Saved to {path}"
        if restart:
            text += f" - restart the station for: {', '.join(restart)}"
        self.lbl_status.setText(text)
        self._on_edited()
        self.settings_saved.emit(cfg)

    def _on_reload(self) -> None:
        self.settings = load_settings()
        self._load_into_editors()
        self.lbl_status.setStyleSheet("")
        self.lbl_status.setText(f"Reloaded from {settings_path()}")
