"""
================================================================================
MODULE: logs_tab.py
PURPOSE: Flight History Workspace - statistics, filters, table, CSV export
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Logs Workspace)
  * Upstream:      core/flight_log.py (one record per armed session)
  * Downstream:    Operator review, CSV export for a report

LAYOUT follows the Walle GCS logs workspace - four headline stat cards, a
filter row, then the table - because that arrangement answers the two
questions in the right order: "how much flying is behind this airframe" before
"what happened on the 3rd of March".

WHAT IS NOT HERE:
  Walle's Replay Flight and Export Video buttons. Neither has anything behind
  it in this station - there is no per-flight telemetry capture to replay and
  no recorded video - and a button that cannot do its job is worse than an
  absent one. Both become straightforward once flight-frame recording exists.
================================================================================
"""

from __future__ import annotations

import csv
from typing import List, Optional

from PyQt5.QtCore import Qt, QDate
from PyQt5.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QComboBox, QDateEdit, QTableWidget, QTableWidgetItem,
    QHeaderView, QFileDialog, QAbstractItemView,
)

from ui.scaling import px
from core.flight_log import FlightLogger, FlightRecord, summarise

COLUMNS = ["DATE", "DURATION", "DISTANCE", "MAX ALT", "MAX SPEED",
           "BATTERY USED", "MODES", "STATUS"]


class LogsTabWidget(QWidget):
    """Flight history: statistics, filtering and export."""

    def __init__(self, logger: Optional[FlightLogger] = None, parent=None):
        super().__init__(parent)
        self.logger = logger or FlightLogger()
        self._records: List[FlightRecord] = []

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 12)
        root.setSpacing(10)

        # ── headline statistics ──
        stats = QHBoxLayout()
        stats.setSpacing(10)
        self.stat_labels = {}
        for key, caption in (("flights", "TOTAL FLIGHTS"),
                             ("total_time", "TOTAL TIME"),
                             ("total_distance", "TOTAL DISTANCE"),
                             ("avg_duration", "AVG DURATION")):
            card = QFrame(self)
            card.setProperty("class", "cardFrame")
            cl = QVBoxLayout(card)
            cl.setContentsMargins(12, 8, 12, 8)
            cl.setSpacing(2)
            cap = QLabel(caption, self)
            cap.setObjectName("tileCaption")
            cl.addWidget(cap)
            val = QLabel("--", self)
            val.setObjectName("statValue")
            cl.addWidget(val)
            self.stat_labels[key] = val
            stats.addWidget(card, 1)
        root.addLayout(stats)

        # ── filters ──
        filters = QFrame(self)
        filters.setProperty("class", "cardFrame")
        fl = QHBoxLayout(filters)
        fl.setContentsMargins(12, 8, 12, 8)
        fl.setSpacing(10)

        self.search = QLineEdit(self)
        self.search.setPlaceholderText("Search flights (mode, status, date)...")
        self.search.textChanged.connect(self._apply_filters)
        fl.addWidget(self.search, 2)

        lbl_from = QLabel("From:", self)
        lbl_from.setObjectName("fieldLabel")
        fl.addWidget(lbl_from)
        self.date_from = QDateEdit(self)
        # QDateEdit's own sizeHint came up a few pixels short of the
        # rendered date, clipping the year.
        self.date_from.setMinimumWidth(px(104))
        self.date_from.setCalendarPopup(True)
        self.date_from.setDate(QDate.currentDate().addMonths(-3))
        self.date_from.dateChanged.connect(self._apply_filters)
        fl.addWidget(self.date_from)

        lbl_to = QLabel("To:", self)
        lbl_to.setObjectName("fieldLabel")
        fl.addWidget(lbl_to)
        self.date_to = QDateEdit(self)
        self.date_to.setMinimumWidth(px(104))
        self.date_to.setCalendarPopup(True)
        self.date_to.setDate(QDate.currentDate())
        self.date_to.dateChanged.connect(self._apply_filters)
        fl.addWidget(self.date_to)

        self.mode_filter = QComboBox(self)
        self.mode_filter.addItems(["All Modes", "OFFBOARD", "POSCTL", "ALTCTL",
                                   "STABILIZED", "MANUAL", "AUTO.LOITER",
                                   "AUTO.LAND", "AUTO.RTL"])
        self.mode_filter.currentIndexChanged.connect(self._apply_filters)
        fl.addWidget(self.mode_filter)

        btn_reload = QPushButton("Reload", self)
        btn_reload.clicked.connect(self.reload)
        fl.addWidget(btn_reload)
        root.addWidget(filters)

        # ── table ──
        self.table = QTableWidget(0, len(COLUMNS), self)
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        head = self.table.horizontalHeader()
        for c in range(len(COLUMNS)):
            head.setSectionResizeMode(
                c, QHeaderView.Stretch if COLUMNS[c] == "MODES" else QHeaderView.ResizeToContents)
        root.addWidget(self.table, 1)

        # ── footer ──
        foot = QHBoxLayout()
        self.lbl_count = QLabel("No flights recorded yet", self)
        self.lbl_count.setObjectName("fieldSubLabel")
        foot.addWidget(self.lbl_count)
        foot.addStretch()
        btn_csv = QPushButton("Export CSV", self)
        btn_csv.clicked.connect(self._export_csv)
        foot.addWidget(btn_csv)
        root.addLayout(foot)

        self.reload()

    # ── data ────────────────────────────────────────────────────────

    def reload(self) -> None:
        self._records = self.logger.load_all()
        self._apply_filters()

    def add_record(self, rec: FlightRecord) -> None:
        """Called live when a session closes, so the tab is never stale."""
        self._records.insert(0, rec)
        self._apply_filters()

    def _visible(self) -> List[FlightRecord]:
        text = self.search.text().strip().lower()
        mode = self.mode_filter.currentText()
        d_from = self.date_from.date().toString("yyyy-MM-dd")
        d_to = self.date_to.date().toString("yyyy-MM-dd")

        out = []
        for r in self._records:
            day = (r.started_utc or "")[:10]
            if day and not (d_from <= day <= d_to):
                continue
            if mode != "All Modes" and mode not in r.modes:
                continue
            if text and text not in " ".join(
                    [r.started_utc, r.status, r.mode_summary]).lower():
                continue
            out.append(r)
        return out

    def _apply_filters(self) -> None:
        rows = self._visible()
        # Statistics describe the filtered set, not the file: a date range that
        # shows three flights should report those three, not all of history.
        for key, val in summarise(rows).items():
            self.stat_labels[key].setText(val)

        self.table.setRowCount(len(rows))
        for i, r in enumerate(rows):
            cells = [
                (r.started_utc or "--").replace("T", " ").rstrip("Z"),
                r.duration_hms(),
                f"{r.distance_m:.1f} m",
                f"{r.max_altitude_m:.2f} m",
                f"{r.max_speed_ms:.2f} m/s",
                f"{r.battery_used_pct}%",
                r.mode_summary,
                r.status + (" (FORCED)" if r.forced_arm else ""),
            ]
            for c, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if c:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.table.setItem(i, c, item)

        total = len(self._records)
        self.lbl_count.setText(
            f"Showing {len(rows)} of {total} recorded flight(s)" if total
            else "No flights recorded yet - a record is written each time the vehicle disarms")

    def _export_csv(self) -> None:
        rows = self._visible()
        if not rows:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export flight log", "flight_log.csv", "CSV files (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["started_utc", "duration_s", "distance_m", "max_altitude_m",
                        "max_speed_ms", "battery_used_pct", "modes", "status",
                        "forced_arm"])
            for r in rows:
                w.writerow([r.started_utc, f"{r.duration_s:.1f}", f"{r.distance_m:.2f}",
                            f"{r.max_altitude_m:.2f}", f"{r.max_speed_ms:.2f}",
                            r.battery_used_pct, r.mode_summary, r.status, r.forced_arm])
        self.lbl_count.setText(f"Exported {len(rows)} flight(s) to {path}")
