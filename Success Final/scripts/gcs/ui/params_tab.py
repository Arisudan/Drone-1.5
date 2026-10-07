"""
================================================================================
MODULE: params_tab.py
PURPOSE: Live PX4 Parameter Table (read-only list, search, refresh, export, save-to-flash)

================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Parameters Workspace)
  * Upstream:      protocol/mavlink_worker.py's param_value_received signal
                   (one PARAM_VALUE per parameter, streamed by the vehicle
                   after a PARAM_REQUEST_LIST - standard MAVLink parameter
                   protocol, not a PX4-specific one)
  * Downstream:    Operator review only - no PARAM_SET is sent from here

WHY READ-ONLY FIRST:
  This is deliberately the first of several planned phases. Editing a real
  flight parameter needs the same guarded-confirm treatment this app already
  gives ARM/DISARM/TAKEOFF (see ui/guided_confirm.py) plus a readback
  verification after every write (the pattern scripts/diagnostics/
  apply_ekf2_params.py already uses) - both are real design work, not a
  checkbox to flip later. Proving the read path end to end first (connect,
  bulk-fetch, search, live-populate a ~1000-2000 row table without stalling
  the UI) is its own useful milestone and ships with none of that risk.

KEY LOGIC:
  * Batched UI updates: PARAM_VALUE can arrive in a fast burst (a real PX4
    can hold 1000-1800+ parameters). Touching the table once per signal would
    mean thousands of individual widget updates fired back-to-back; instead
    incoming values are buffered and flushed to the table on a short timer
    (FLUSH_INTERVAL_MS), so a fast burst becomes a handful of batched redraws.
  * Row identity is tracked by the QTableWidgetItem itself, not by a stored
    row index: sorting is on by default (operators expect to click NAME/
    INDEX to sort), and a row's index changes the moment the table resorts.
    QTableWidgetItem.row() always reflects a row's live position regardless
    of how many times it has moved, so that is what every lookup uses -
    a stored index dict would silently point at the wrong row after the
    first sort.
================================================================================
"""

from __future__ import annotations

import time
from typing import Dict, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QFileDialog,
    QPushButton, QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
)


from ui.scaling import px
from core.param_codec import format_param_value, param_type_name

COLUMNS = ["NAME", "VALUE", "TYPE", "INDEX"]


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
    """Live parameter list: connect, fetch, search, export. The only thing that can
    reach the vehicle from here is the guarded 'Save to flash' request; no
    parameter is ever written. (The preflight checklist lives in the left rail.)"""

    refresh_requested = pyqtSignal()
    # The operator asked to store the vehicle's current parameters in its flash.
    # The main window puts it behind the slide-to-confirm gate.
    save_flash_requested = pyqtSignal()

    FLUSH_INTERVAL_MS = 200

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        # name -> (value, param_type, index), the authoritative last-known state.
        self._params: Dict[str, tuple] = {}
        # name -> (value, param_type, index) not yet drawn into the table.
        self._pending: Dict[str, tuple] = {}
        # name -> the QTableWidgetItem holding its NAME cell. See module
        # docstring: .row() on this is the only safe way to find a row once
        # sorting is enabled.
        self._name_item: Dict[str, QTableWidgetItem] = {}
        self._total_count = 0
        self._connected = False

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 12)
        root.setSpacing(10)

        # ── header ──
        head = QHBoxLayout()
        head.setSpacing(10)
        title = QLabel("PX4 PARAMETERS", self)
        title.setObjectName("cardHeading")
        head.addWidget(title)

        self.search = QLineEdit(self)
        self.search.setPlaceholderText("Search parameters...")
        self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(self._apply_filter)
        head.addWidget(self.search, 1)

        self.btn_refresh = QPushButton("Refresh", self)
        self.btn_refresh.setToolTip(
            "Re-fetch the full parameter list from the vehicle "
            "(PARAM_REQUEST_LIST - read-only)")
        self.btn_refresh.clicked.connect(self.refresh_requested)
        head.addWidget(self.btn_refresh)

        self.btn_export = QPushButton("Export…", self)
        self.btn_export.setToolTip("Save the parameters shown here to a .params file "
                                   "(same layout as Drone_1.5.params).")
        self.btn_export.clicked.connect(self._on_export)
        head.addWidget(self.btn_export)

        self.btn_save_flash = QPushButton("Save to flash", self)
        self.btn_save_flash.setToolTip(
            "Tell the flight controller to write its CURRENT parameters to flash so they "
            "survive a power cycle. Needs a link and a disarmed vehicle; asks you to confirm.")
        self.btn_save_flash.clicked.connect(self.save_flash_requested)
        head.addWidget(self.btn_save_flash)
        root.addLayout(head)

        self.lbl_status = QLabel("Not connected", self)
        self.lbl_status.setObjectName("fieldSubLabel")
        root.addWidget(self.lbl_status)

        # ── table ──
        self.table = QTableWidget(0, len(COLUMNS), self)
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.setSortingEnabled(True)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        for c in (1, 2, 3):
            header.setSectionResizeMode(c, QHeaderView.ResizeToContents)
        root.addWidget(self.table, 1)

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
        self._total_count = 0
        self.table.setRowCount(0)
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

    # ── internal ──────────────────────────────────────────────────────

    def _update_status_label(self) -> None:
        have = len(self._params) + len(self._pending)
        if self._total_count:
            done = " - done" if have >= self._total_count and not self._pending else ""
            self.lbl_status.setText(
                f"{have} / {self._total_count} parameters received{done}")
        else:
            self.lbl_status.setText(f"{have} parameters received")

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
                self._name_item[name] = name_item
                self.table.setItem(row, 0, name_item)
            row = name_item.row()

            value_item = QTableWidgetItem(format_param_value(value, param_type))
            value_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setItem(row, 1, value_item)

            self.table.setItem(row, 2, QTableWidgetItem(param_type_name(param_type)))

            index_item = QTableWidgetItem()
            index_item.setData(Qt.DisplayRole, index)  # numeric sort, not "10" < "2"
            index_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setItem(row, 3, index_item)
        self.table.setSortingEnabled(True)

        self._apply_filter(self.search.text())
        self._update_status_label()

    def _apply_filter(self, text: str) -> None:
        needle = text.strip().lower()
        for name, item in self._name_item.items():
            self.table.setRowHidden(
                item.row(), bool(needle) and needle not in name.lower())
