"""
================================================================================
MODULE: params_tab.py
PURPOSE: PX4 Parameters, laid out the way QGroundControl shows them (read-only)
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Parameters Workspace)
  * Upstream:      protocol/mavlink_worker.py's param_value_received signal
                   (one PARAM_VALUE per parameter, streamed by the vehicle
                   after a PARAM_REQUEST_LIST - standard MAVLink parameter
                   protocol, not a PX4-specific one) and core/param_meta.py
                   (PX4's own descriptions for the firmware version in use)
  * Downstream:    Operator review only - no PARAM_SET is sent from here

WHAT IT SHOWS (the same things QGroundControl shows):
  * a group list on the left (EKF2, Battery Calibration, MAVLink ...) with counts
  * NAME | VALUE | DESCRIPTION, with units, and a named choice instead of a bare
    number where PX4 defines one (EKF2_HGT_REF = "Range sensor (2)"), and the names
    of the set bits for a bitmask (EKF2_EV_CTRL = "Horizontal position, ...")
  * parameters that differ from PX4's default are shown in bold; "Modified only"
    filters to just those
  * a detail panel for the selected row: full value, default, range, units, type,
    "reboot required", the long description, every choice / every bit
  * search across names and descriptions (a search looks in ALL groups)

WHY READ-ONLY:
  Editing a real flight parameter needs the same guarded-confirm treatment this app
  already gives ARM/DISARM/TAKEOFF (see ui/guided_confirm.py) plus a readback
  verification after every write (the pattern scripts/diagnostics/
  apply_ekf2_params.py already uses). That is a separate piece of work; the only
  thing that can reach the vehicle from here is the guarded "Save to flash" request.

KEY LOGIC:
  * Batched UI updates: PARAM_VALUE can arrive in a fast burst (a real PX4
    can hold 1000-1800+ parameters). Incoming values are buffered and flushed to
    the table on a short timer (FLUSH_INTERVAL_MS), so a burst becomes a handful
    of batched redraws.
  * Row identity is tracked by the QTableWidgetItem itself, not by a stored
    row index: sorting is on by default and a row's index changes the moment the
    table resorts. QTableWidgetItem.row() always reflects a row's live position.
  * A parameter with no description (hardware-specific, or newer than the bundled
    PX4 reference) still shows - with its number and type, never an error.
================================================================================
"""

from __future__ import annotations

import html
import time
from collections import Counter
from typing import Dict, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QAbstractItemView, QCheckBox, QFileDialog, QFrame, QHBoxLayout, QHeaderView, QLabel,
    QLineEdit, QListWidget, QListWidgetItem, QPushButton, QTableWidget,
    QTableWidgetItem, QTextBrowser, QVBoxLayout, QWidget,
)
from pymavlink import mavutil

from core import param_meta as pm
from core.param_codec import format_param_value, param_type_name
from ui.scaling import px
from ui.styles import PALETTE
from ui.theme import Chip, GlassBar, GlassPanel

COLUMNS = ["NAME", "VALUE", "RANGE", "DESCRIPTION"]
_REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32


def params_file_text(params: Dict[str, tuple], stamp: Optional[str] = None) -> str:
    """The parameters in the QGroundControl / Mission Planner .params layout
    (tab-separated: vehicle id, component id, name, value, MAV_PARAM_TYPE), sorted
    by name, so the file can be diffed against Drone_1.5.params or loaded elsewhere."""
    stamp = stamp or time.strftime("%Y-%m-%dT%H:%M:%S")
    lines = ["# Onboard parameters for Vehicle 1", "#", "# Stack: PX4 Pro",
             f"# Exported from the Drone-GCS Parameters tab {stamp}",
             "#", "# Vehicle-Id Component-Id Name Value Type"]
    for name in sorted(params):
        value, ptype, _ = params[name]
        lines.append(f"1\t1\t{name}\t{format_param_value(value, ptype)}\t{ptype}")
    return "\n".join(lines) + "\n"


class ParamsTabWidget(QWidget):
    """Live parameter list, laid out like QGroundControl's: groups, search, named
    values, a detail panel. The only thing that can reach the vehicle from here is
    the guarded 'Save to flash' request; no parameter is ever written. (The
    preflight checklist lives in the left rail.)"""

    refresh_requested = pyqtSignal()
    # The operator asked to store the vehicle's current parameters in its flash.
    # The main window puts it behind the slide-to-confirm gate.
    save_flash_requested = pyqtSignal()

    FLUSH_INTERVAL_MS = 200
    ALL_GROUPS = "All parameters"

    def __init__(self, parent: Optional[QWidget] = None,
                 catalog: Optional[pm.ParamCatalog] = None):
        super().__init__(parent)
        self.catalog = catalog if catalog is not None else pm.load_catalog()
        # name -> (value, param_type, index), the authoritative last-known state.
        self._params: Dict[str, tuple] = {}
        # name -> (value, param_type, index) not yet drawn into the table.
        self._pending: Dict[str, tuple] = {}
        # name -> the QTableWidgetItem holding its NAME cell. .row() on this is the
        # only safe way to find a row once sorting is enabled.
        self._name_item: Dict[str, QTableWidgetItem] = {}
        self._group_of: Dict[str, str] = {}
        self._selected_group: Optional[str] = None      # None = all groups
        self._groups_dirty = False
        self._total_count = 0
        self._connected = False

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(px(10))

        # ── search bar (glass strip) ──
        bar = GlassBar(self)
        head = QHBoxLayout(bar)
        head.setContentsMargins(px(14), px(11), px(14), px(11))
        head.setSpacing(px(10))

        self.search = QLineEdit(self)
        self.search.setPlaceholderText("Search parameters...")
        self.search.setToolTip("Searches parameter names and descriptions in every group.")
        self.search.setClearButtonEnabled(True)
        self.search.setMinimumHeight(px(30))
        self.search.textChanged.connect(self._apply_filter)
        head.addWidget(self.search, 1)

        self.chk_modified = QCheckBox("Modified only", self)
        self.chk_modified.setObjectName("cfgToggle")
        self.chk_modified.setToolTip("Show only the parameters that differ from PX4's "
                                     "default value (shown in bold).")
        self.chk_modified.toggled.connect(lambda _on: self._apply_filter())
        head.addWidget(self.chk_modified)

        self.btn_refresh = QPushButton("REFRESH", self)
        self.btn_refresh.setObjectName("btnConnect")
        self.btn_refresh.setMinimumHeight(px(30))
        self.btn_refresh.setToolTip(
            "Re-fetch the full parameter list from the vehicle "
            "(PARAM_REQUEST_LIST - read-only)")
        self.btn_refresh.clicked.connect(self.refresh_requested)
        head.addWidget(self.btn_refresh)

        self.btn_export = QPushButton("EXPORT", self)
        self.btn_export.setMinimumHeight(px(30))
        self.btn_export.setToolTip("Save the parameters shown here to a .params file "
                                   "(same layout as Drone_1.5.params).")
        self.btn_export.clicked.connect(self._on_export)
        head.addWidget(self.btn_export)

        self.btn_save_flash = QPushButton("SAVE TO FLASH", self)
        self.btn_save_flash.setMinimumHeight(px(30))
        self.btn_save_flash.setToolTip(
            "Tell the flight controller to write its CURRENT parameters to flash so they "
            "survive a power cycle. Needs a link and a disarmed vehicle; asks you to confirm.")
        self.btn_save_flash.clicked.connect(self.save_flash_requested)
        head.addWidget(self.btn_save_flash)

        self.chip_ro = Chip("READ-ONLY", "warn", self)
        self.chip_ro.setToolTip("No parameter is ever written from here; only 'Save to flash' reaches the vehicle.")
        head.addWidget(self.chip_ro)
        root.addWidget(bar)

        self.lbl_status = QLabel("Not connected", self)
        self.lbl_status.setObjectName("fieldSubLabel")
        self.lbl_status.setContentsMargins(px(4), 0, 0, 0)
        root.addWidget(self.lbl_status)

        # ── body: groups on the left, table + detail on the right ──
        body = QHBoxLayout()
        body.setSpacing(px(12))

        self.pnl_groups = GlassPanel("Groups", "", False, self)
        self.pnl_groups.setFixedWidth(px(210))
        gl = self.pnl_groups.body_layout
        gl.setContentsMargins(px(8), px(8), px(8), px(8))
        self.groups = QListWidget(self)
        self.groups.setObjectName("paramGroups")
        self.groups.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.groups.setTextElideMode(Qt.ElideRight)
        self.groups.currentItemChanged.connect(self._on_group_changed)
        gl.addWidget(self.groups)
        body.addWidget(self.pnl_groups)

        self.pnl_table = GlassPanel("PX4 parameters", "0 parameters", True, self)
        tl = self.pnl_table.body_layout
        tl.setContentsMargins(0, 0, 0, 0)
        tl.setSpacing(0)

        self.table = QTableWidget(0, len(COLUMNS), self)
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setShowGrid(False)
        self.table.setFrameShape(QFrame.NoFrame)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setAlternatingRowColors(False)
        self.table.setSortingEnabled(True)
        self.table.sortByColumn(0, Qt.AscendingOrder)      # A to Z, like QGC
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Interactive)
        header.setSectionResizeMode(1, QHeaderView.Interactive)
        header.setSectionResizeMode(2, QHeaderView.Interactive)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        self.table.setColumnWidth(0, px(230))
        self.table.setColumnWidth(1, px(130))
        self.table.setColumnWidth(2, px(120))
        self.table.verticalHeader().setDefaultSectionSize(px(34))
        self.table.currentCellChanged.connect(lambda *_a: self._show_detail(self.selected_name()))
        tl.addWidget(self.table, 1)

        self.detail = QTextBrowser(self)
        self.detail.setObjectName("paramDetail")
        self.detail.setOpenLinks(False)
        self.detail.setMinimumHeight(px(150))
        self.detail.setMaximumHeight(px(230))
        tl.addWidget(self.detail)
        body.addWidget(self.pnl_table, 1)
        root.addLayout(body, 1)
        self._show_detail(None)

        self._flush_timer = QTimer(self)
        self._flush_timer.setInterval(self.FLUSH_INTERVAL_MS)
        self._flush_timer.timeout.connect(self._flush)

    # ── public API (driven by drone_gcs.py) ──────────────────────────

    def export_params(self, path: str) -> int:
        """Write the table to `path`. Returns the number of parameters written."""
        snapshot = dict(self._params)
        snapshot.update(self._pending)
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(params_file_text(snapshot))
        return len(snapshot)

    def _on_export(self) -> None:
        if not (self._params or self._pending):
            self.lbl_status.setText("Nothing to export yet - connect and refresh first.")
            return
        default = time.strftime("params_%Y%m%d_%H%M%S.params")
        path, _ = QFileDialog.getSaveFileName(self, "Export parameters", default,
                                              "Parameter files (*.params);;All files (*)")
        if not path:
            return
        try:
            n = self.export_params(path)
        except OSError as exc:
            self.lbl_status.setText(f"Could not export: {exc}")
            return
        self.lbl_status.setText(f"Exported {n} parameters to {path}")

    def set_save_flash_enabled(self, enabled: bool, why: str = "") -> None:
        self.btn_save_flash.setEnabled(enabled)
        if why:
            self.btn_save_flash.setToolTip(why)

    def has_data(self) -> bool:
        return bool(self._params) or bool(self._pending)

    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        if not connected:
            self.lbl_status.setText("Not connected")

    def begin_refresh(self) -> None:
        """Call right before actually sending PARAM_REQUEST_LIST."""
        self._flush_timer.stop()
        self._params.clear()
        self._pending.clear()
        self._name_item.clear()
        self._group_of.clear()
        self._selected_group = None
        self._total_count = 0
        self.table.setRowCount(0)
        self._rebuild_groups()
        self._show_detail(None)
        self.lbl_status.setText("Requesting parameters...")

    def refresh_unavailable(self, reason: str) -> None:
        self.lbl_status.setText(reason)

    def on_param_value(self, name: str, value, param_type: int,
                       index: int, count: int) -> None:
        self._total_count = count
        self._pending[name] = (value, param_type, index)
        if not self._flush_timer.isActive():
            self._flush_timer.start()
        self._update_status_label()

    def selected_name(self) -> Optional[str]:
        row = self.table.currentRow()
        item = self.table.item(row, 0) if row >= 0 else None
        return item.data(Qt.UserRole) if item is not None else None

    def select(self, name: str) -> bool:
        """Select a parameter's row (and show its detail). False if it is not loaded."""
        item = self._name_item.get(name)
        if item is None:
            return False
        self.table.setCurrentCell(item.row(), 0)
        self._show_detail(name)
        return True

    # ── internal ──────────────────────────────────────────────────────

    def _is_float(self, param_type: int) -> bool:
        return param_type == _REAL32

    def _modified_count(self) -> int:
        n = 0
        for name, (value, ptype, _i) in self._params.items():
            if pm.is_modified(self.catalog.get(name), value):
                n += 1
        return n

    def _update_status_label(self) -> None:
        have = len(self._params) + len(self._pending)
        if self._total_count:
            done = have >= self._total_count and not self._pending
            text = f"{have} / {self._total_count} parameters received" + (" - done" if done else "")
        else:
            done = False
            text = f"{have} parameters received"
        if done and self.catalog.available:
            text += (f"  ·  {self._modified_count()} changed from default (bold)"
                     f"  ·  descriptions: {self.catalog.label()}")
        elif done:
            text += "  ·  no parameter descriptions are bundled"
        self.lbl_status.setText(text)
        self.pnl_table.set_sub(f"{have} parameters")

    def _fill_row(self, name: str, row: int, value, param_type: int) -> None:
        meta = self.catalog.get(name)
        is_float = self._is_float(param_type)
        modified = bool(pm.is_modified(meta, value))
        bold = QFont(self.font())
        bold.setBold(modified)

        mono = QFont("Ubuntu Mono")
        mono.setPixelSize(px(13))
        mono_bold = QFont(mono)
        mono_bold.setBold(modified)
        name_item = self.table.item(row, 0)
        name_item.setFont(mono_bold)
        name_item.setToolTip(meta.short if meta and meta.short else name)

        value_item = QTableWidgetItem(pm.value_text(meta, value, is_float))
        value_item.setFont(mono_bold)
        value_item.setForeground(QColor("#ffb02e"))
        value_item.setData(Qt.UserRole, value)
        value_item.setToolTip(f"{format_param_value(value, param_type)}  "
                              f"({param_type_name(param_type)})")
        self.table.setItem(row, 1, value_item)

        rng = QTableWidgetItem(pm.range_text(meta) if meta else "")
        rng.setFont(mono)
        rng.setForeground(QColor("#a3aaa5"))
        self.table.setItem(row, 2, rng)

        desc = QTableWidgetItem(meta.short if meta else "")
        desc.setForeground(QColor("#a3aaa5"))
        if meta and meta.short:
            desc.setToolTip(meta.short)
        self.table.setItem(row, 3, desc)

    def _flush(self) -> None:
        if not self._pending:
            self._flush_timer.stop()
            return
        batch, self._pending = self._pending, {}

        self.table.setSortingEnabled(False)
        for name, (value, param_type, index) in batch.items():
            self._params[name] = (value, param_type, index)
            name_item = self._name_item.get(name)
            if name_item is None:
                row = self.table.rowCount()
                self.table.insertRow(row)
                name_item = QTableWidgetItem(name)
                name_item.setData(Qt.UserRole, name)
                self._name_item[name] = name_item
                self.table.setItem(row, 0, name_item)
                group = pm.group_of(name, self.catalog.get(name))
                self._group_of[name] = group
                self._groups_dirty = True
            self._fill_row(name, name_item.row(), value, param_type)
        self.table.setSortingEnabled(True)

        if self._groups_dirty:
            self._rebuild_groups()
        self._apply_filter()
        self._update_status_label()
        current = self.selected_name()
        if current in batch:
            self._show_detail(current)

    # ── groups, filter ────────────────────────────────────────────────

    def _rebuild_groups(self) -> None:
        self._groups_dirty = False
        counts = Counter(self._group_of.values())
        keep = self._selected_group
        self.groups.blockSignals(True)
        self.groups.clear()
        first = QListWidgetItem(f"{self.ALL_GROUPS}  ({len(self._group_of)})")
        first.setData(Qt.UserRole, None)
        self.groups.addItem(first)
        target = first
        for g in sorted(counts, key=str.lower):
            item = QListWidgetItem(f"{g}  ({counts[g]})")
            item.setData(Qt.UserRole, g)
            self.groups.addItem(item)
            if g == keep:
                target = item
        self.groups.setCurrentItem(target)
        self.groups.blockSignals(False)

    def _on_group_changed(self, current, _previous) -> None:
        self._selected_group = current.data(Qt.UserRole) if current is not None else None
        self._apply_filter()

    def _matches(self, name: str, needle: str, modified_only: bool) -> bool:
        meta = self.catalog.get(name)
        if needle:
            # A search looks in every group, like QGC; the group list is for browsing.
            hay = name.lower() + " " + (meta.short.lower() if meta else "")
            if needle not in hay:
                return False
        elif self._selected_group is not None and self._group_of.get(name) != self._selected_group:
            return False
        if modified_only:
            value = self._params.get(name, (None,))[0]
            if not pm.is_modified(meta, value):
                return False
        return True

    def _apply_filter(self, _text: Optional[str] = None) -> None:
        needle = self.search.text().strip().lower()
        modified_only = self.chk_modified.isChecked()
        for name, item in self._name_item.items():
            self.table.setRowHidden(item.row(), not self._matches(name, needle, modified_only))

    # ── detail panel ──────────────────────────────────────────────────

    def _show_detail(self, name: Optional[str]) -> None:
        if not name or name not in self._params:
            self.detail.setHtml(
                f"<span style='color:{PALETTE['text_muted']}'>Select a parameter to see its "
                "description, default, range and every choice.</span>")
            return
        value, ptype, index = self._params[name]
        meta = self.catalog.get(name)
        is_float = self._is_float(ptype)
        esc = html.escape
        dim, bright, text = PALETTE["text_muted"], PALETTE["text_bright"], PALETTE["text"]
        size = px(11)

        parts = [f"<div style='font-size:{size}px; color:{text}'>"
                 f"<span style='color:{bright}; font-weight:600; font-size:{px(13)}px'>{esc(name)}</span>"]
        if meta and meta.short:
            parts.append(f" &nbsp;<span style='color:{dim}'>{esc(meta.short)}</span>")
        facts = []

        def fact(label, val):
            facts.append(f"<span style='color:{dim}'>{esc(label)}</span>&nbsp;&nbsp;{val}")

        modified = pm.is_modified(meta, value)
        fact("Value", f"<b style='color:{bright}'>{esc(pm.value_text(meta, value, is_float))}</b>")
        fact("Default", esc(pm.default_text(meta)) +
             (f" <span style='color:{dim}'>(changed)</span>" if modified else ""))
        fact("Range", esc(pm.range_text(meta)))
        if meta and meta.units:
            fact("Units", esc(meta.units))
        if meta and meta.increment is not None:
            fact("Step", esc(pm.format_number(meta.increment, meta, True)))
        fact("Type", esc(param_type_name(ptype)))
        fact("Group", esc(self._group_of.get(name, pm.group_of(name, meta))))
        if meta:
            fact("Reboot", "required" if meta.reboot else "not needed")
        fact("Index", str(index))
        sep = f" &nbsp;<span style='color:{PALETTE['border_hover']}'>|</span>&nbsp; "
        parts.append(f"<div style='margin-top:6px'>{sep.join(facts)}</div>")

        if meta and meta.long:
            parts.append(f"<p style='margin-top:6px'>{esc(meta.long)}</p>")
        elif not meta or not meta.short:
            parts.append(f"<p style='color:{dim}; margin-top:6px'>{esc(pm.NO_DESCRIPTION)}</p>")

        if meta and meta.values:
            current = pm.enum_label(meta, value)
            parts.append(f"<p style='margin-top:6px; color:{dim}'>Choices</p>")
            for v, label in meta.values:
                on = current is not None and label == current and v == int(round(float(value)))
                mark = "&#9654;" if on else "&nbsp;&nbsp;"
                style = f"font-weight:600; color:{bright}" if on else f"color:{text}"
                parts.append(f"<div style='{style}'>{mark} &nbsp;{v} &nbsp;{esc(label)}</div>")
        elif meta and meta.bitmask and not is_float:
            parts.append(f"<p style='margin-top:6px; color:{dim}'>Bits</p>")
            for bit, label, is_on in pm.bit_states(meta, value):
                box = "&#9745;" if is_on else "&#9744;"
                style = f"font-weight:600; color:{bright}" if is_on else f"color:{text}"
                parts.append(f"<div style='{style}'>{box} &nbsp;bit {bit} &nbsp;{esc(label)}</div>")
        parts.append("</div>")
        self.detail.setHtml("".join(parts))
