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

SAVING VALIDATES FIRST: core.settings.save_settings raises on an out-of-range
value and the tab reports it inline, so a battery-critical threshold above the
warning threshold is refused at the point of entry rather than at the point of
flight.
================================================================================
"""

from __future__ import annotations

from dataclasses import fields, is_dataclass
from typing import Dict, Optional

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QLineEdit,
    QPushButton, QComboBox, QCheckBox, QScrollArea,
)

from core.settings import GCSSettings, load_settings, save_settings, settings_path

# Human captions and, where a value is an enumeration, its allowed set.
CAPTIONS = {
    "profile": ("DRONE PROFILE", {
        "name": "Airframe name", "frame": "Frame type",
        "battery_cells": "Battery cells (S)",
        "battery_capacity_mah": "Capacity (mAh)", "drone_id": "Drone ID"}),
    "connection": ("MAVLINK CONNECTION", {
        "default_network": "Default network", "host": "Companion IP",
        "protocol": "Protocol", "udp_port": "UDP port (flight)",
        "tcp_port": "TCP port (bench)", "source_system": "GCS system ID"}),
    "video": ("VIDEO & MAP BRIDGE", {
        "stream_url": "FPV stream URL", "map_bridge_port": "TCP map bridge port",
        "jpeg_port": "MJPEG port"}),
    "slam": ("SLAM & NAVIGATION", {
        "cruise_altitude_m": "Cruise altitude (m)",
        "robot_radius_m": "Robot radius (m)", "cell_size_m": "Grid cell size (m)",
        "treat_unknown_as_obstacle": "Treat unknown space as obstacle"}),
    "limits": ("COMMAND LIMITS", {
        "takeoff_alt_min_m": "Takeoff min (m)", "takeoff_alt_max_m": "Takeoff max (m)",
        "move_max_delta_m": "Max move delta (m)"}),
    "alerts": ("ALERT THRESHOLDS", {
        "batt_warn_pct": "Battery warning (%)", "batt_crit_pct": "Battery critical (%)",
        "vision_stale_s": "Vision stale after (s)",
        "map_stall_s": "Map stalled after (s)"}),
}

CHOICES = {
    ("connection", "protocol"): ["udp", "tcp"],
    ("connection", "default_network"): ["HTIC_RND", "DroneBridge5", "DroneNet"],
    ("profile", "frame"): ["Quad X", "Quad +", "Hex X", "Hex +", "Y6", "Octo X"],
}


class ConfigTabWidget(QWidget):
    """Editor over core.settings.GCSSettings."""

    settings_saved = pyqtSignal(object)   # emits the saved GCSSettings

    def __init__(self, settings: Optional[GCSSettings] = None, parent=None):
        super().__init__(parent)
        self.settings = settings or load_settings()
        self._editors: Dict[str, QWidget] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 12)
        root.setSpacing(10)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        holder = QWidget()
        grid = QGridLayout(holder)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)

        for i, section in enumerate(self.settings.sections()):
            grid.addWidget(self._build_card(section), i // 2, i % 2)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(grid.rowCount(), 1)
        scroll.setWidget(holder)
        root.addWidget(scroll, 1)

        foot = QHBoxLayout()
        self.lbl_status = QLabel(f"Settings file: {settings_path()}", self)
        self.lbl_status.setObjectName("fieldSubLabel")
        foot.addWidget(self.lbl_status)
        foot.addStretch()

        btn_reload = QPushButton("Reload", self)
        btn_reload.clicked.connect(self._on_reload)
        foot.addWidget(btn_reload)

        btn_save = QPushButton("Save Settings", self)
        btn_save.setObjectName("btnConnect")
        btn_save.clicked.connect(self._on_save)
        foot.addWidget(btn_save)
        root.addLayout(foot)

        self._load_into_editors()

    # ── construction ────────────────────────────────────────────────

    def _build_card(self, section: str) -> QFrame:
        title, captions = CAPTIONS[section]
        card = QFrame(self)
        card.setProperty("class", "cardFrame")
        cv = QVBoxLayout(card)
        cv.setContentsMargins(12, 10, 12, 10)
        cv.setSpacing(6)

        heading = QLabel(title, self)
        heading.setObjectName("cardHeading")
        cv.addWidget(heading)

        rule = QFrame(self)
        rule.setObjectName("hDivider")
        rule.setFixedHeight(1)
        cv.addWidget(rule)

        body = QGridLayout()
        body.setHorizontalSpacing(8)
        body.setVerticalSpacing(5)
        body.setColumnStretch(1, 1)

        obj = getattr(self.settings, section)
        for r, f in enumerate(fields(obj)):
            cap = QLabel(captions.get(f.name, f.name), self)
            cap.setObjectName("fieldSubLabel")
            body.addWidget(cap, r, 0)
            editor = self._editor_for(section, f.name, getattr(obj, f.name))
            body.addWidget(editor, r, 1)
            self._editors[f"{section}.{f.name}"] = editor
        cv.addLayout(body)
        return card

    def _editor_for(self, section: str, name: str, value) -> QWidget:
        choices = CHOICES.get((section, name))
        if choices is not None:
            combo = QComboBox(self)
            combo.addItems(choices)
            return combo
        if isinstance(value, bool):
            return QCheckBox("", self)
        edit = QLineEdit(self)
        edit.setMaximumWidth(220)
        return edit

    # ── value transfer ──────────────────────────────────────────────

    def _load_into_editors(self) -> None:
        for section in self.settings.sections():
            obj = getattr(self.settings, section)
            for f in fields(obj):
                w = self._editors[f"{section}.{f.name}"]
                v = getattr(obj, f.name)
                if isinstance(w, QCheckBox):
                    w.setChecked(bool(v))
                elif isinstance(w, QComboBox):
                    idx = w.findText(str(v))
                    w.setCurrentIndex(idx if idx >= 0 else 0)
                else:
                    w.setText(str(v))

    def _read_from_editors(self) -> GCSSettings:
        """Build a fresh settings object; leaves self.settings untouched until
        it validates, so a rejected edit cannot half-apply."""
        cfg = GCSSettings()
        for section in cfg.sections():
            obj = getattr(cfg, section)
            if not is_dataclass(obj):
                continue
            for f in fields(obj):
                w = self._editors[f"{section}.{f.name}"]
                current = getattr(obj, f.name)
                if isinstance(w, QCheckBox):
                    setattr(obj, f.name, w.isChecked())
                    continue
                raw = w.currentText() if isinstance(w, QComboBox) else w.text().strip()
                try:
                    if isinstance(current, bool):
                        setattr(obj, f.name, raw.lower() in ("1", "true", "yes", "on"))
                    elif isinstance(current, int):
                        setattr(obj, f.name, int(float(raw)))
                    elif isinstance(current, float):
                        setattr(obj, f.name, float(raw))
                    else:
                        setattr(obj, f.name, raw)
                except ValueError:
                    raise ValueError(f"{section}.{f.name}: {raw!r} is not a number")
        return cfg

    # ── actions ─────────────────────────────────────────────────────

    def _on_save(self) -> None:
        try:
            cfg = self._read_from_editors()
            path = save_settings(cfg)
        except ValueError as exc:
            self.lbl_status.setText(f"Not saved - {exc}")
            self.lbl_status.setStyleSheet("color: #f85149;")
            return
        self.settings = cfg
        self.lbl_status.setStyleSheet("color: #3fb950;")
        self.lbl_status.setText(f"Saved to {path}")
        self.settings_saved.emit(cfg)

    def _on_reload(self) -> None:
        self.settings = load_settings()
        self._load_into_editors()
        self.lbl_status.setStyleSheet("")
        self.lbl_status.setText(f"Reloaded from {settings_path()}")
