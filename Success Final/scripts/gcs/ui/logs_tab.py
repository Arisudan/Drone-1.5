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

FLIGHT DETAIL AND TRENDS:
  Each flight now carries a time series (core/flight_log.py), so a row opens a
  detail view (altitude, speed, battery and voltage charts plus the ground track,
  ui/flight_detail.py) and a Trends panel shows battery health across flights:
  percent used per minute and voltage sag. Both say so plainly when a flight
  predates the recording rather than drawing an empty chart.

WHAT IS NOT HERE:
  Walle's Export Video button: this station records no video. (A frame-by-frame
  replay of the flight on the map is possible now that positions are recorded,
  but is not built.)
================================================================================
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import List, Optional

from PyQt5.QtCore import Qt, QDate
from PyQt5.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QComboBox, QDateEdit, QTableWidget, QTableWidgetItem,
    QHeaderView, QFileDialog, QAbstractItemView, QInputDialog, QMessageBox,
)

from ui.scaling import px, fit_min_width
from core.flight_log import FlightLogger, FlightRecord, summarise
from core.log_bundle import create_bundle, default_bundle_name
from ui.flight_detail import FlightDetailDialog, TrendChart

COLUMNS = ["DATE", "DURATION", "DISTANCE", "MAX ALT", "MAX SPEED",
           "BATTERY USED", "MODES", "STATUS"]


def _fit_date_edit(edit: QDateEdit) -> None:
    """Size a date field to the widest date it can show, in the font in use.

    This used to be a fixed px(104). That was right for Lato and clipped the
    year under any wider fallback font - which is every machine without Lato
    installed, including a stock CI runner ("24 Sep 2026" needed 74 px of
    text area and got 67). QDateEdit's own sizeHint also comes up short, so
    measure every month rather than trusting it.
    """
    edit.ensurePolished()          # stylesheet font must be applied first
    fm = edit.fontMetrics()
    fmt = edit.displayFormat()
    widest = max(fm.horizontalAdvance(QDate(2026, month, 28).toString(fmt))
                 for month in range(1, 13))
    # The text sits in an inner QLineEdit, which keeps ~12 px of its own
    # margin; around that come the drop-down button and the stylesheet's
    # horizontal padding. Counting only the outer parts left it 2 px short.
    inner_margin = px(12)
    edit.setMinimumWidth(widest + inner_margin + px(18) + px(8) * 2 + px(4))


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
        # Short placeholder: the long form clipped at the minimum window width
        # with 135% scaling. What it matches is in the tooltip instead.
        self.search.setPlaceholderText("Search flights...")
        # The stretch factor below (2) only wins a fair share of what's left
        # after every other filter has taken its own minimum - at a narrow
        # window that share can still be less than the placeholder needs.
        fit_min_width(self.search, ["Search flights..."], h_pad_px=16)
        self.search.setToolTip("Filters by flight mode, status or date text")
        self.search.textChanged.connect(self._apply_filters)
        fl.addWidget(self.search, 2)

        lbl_from = QLabel("From:", self)
        lbl_from.setObjectName("fieldLabel")
        fl.addWidget(lbl_from)
        self.date_from = QDateEdit(self)
        self.date_from.setCalendarPopup(True)
        # ISO, to match the DATE column and every other date in the station (the
        # locale default read 7/3/26 - three different dates depending on who
        # is looking).
        self.date_from.setDisplayFormat("yyyy-MM-dd")
        self.date_from.setDate(QDate.currentDate().addMonths(-3))
        self.date_from.dateChanged.connect(self._apply_filters)
        fl.addWidget(self.date_from)

        lbl_to = QLabel("To:", self)
        lbl_to.setObjectName("fieldLabel")
        fl.addWidget(lbl_to)
        self.date_to = QDateEdit(self)
        self.date_to.setCalendarPopup(True)
        self.date_to.setDisplayFormat("yyyy-MM-dd")
        self.date_to.setDate(QDate.currentDate())
        self.date_to.dateChanged.connect(self._apply_filters)
        fl.addWidget(self.date_to)
        for edit in (self.date_from, self.date_to):
            _fit_date_edit(edit)

        self.mode_filter = QComboBox(self)
        _mode_items = ["All Modes", "OFFBOARD", "POSCTL", "ALTCTL",
                       "STABILIZED", "MANUAL", "AUTO.LOITER",
                       "AUTO.LAND", "AUTO.RTL"]
        self.mode_filter.addItems(_mode_items)
        fit_min_width(self.mode_filter, _mode_items, h_pad_px=46)
        self.mode_filter.currentIndexChanged.connect(self._apply_filters)
        fl.addWidget(self.mode_filter)

        btn_reload = QPushButton("Reload", self)
        btn_reload.clicked.connect(self.reload)
        fl.addWidget(btn_reload)
        root.addWidget(filters)

        # ── trends (collapsed until asked for) ──
        self.trends = QFrame(self)
        self.trends.setProperty("class", "cardFrame")
        tl = QHBoxLayout(self.trends)
        tl.setContentsMargins(px(8), px(8), px(8), px(8))
        tl.setSpacing(px(8))
        self.trend_use = TrendChart("Battery used per minute", "%/min", self, decimals=1)
        self.trend_use.setToolTip("Percent of battery consumed per minute of flight. "
                                  "A rising trend means the pack is ageing or the airframe is working harder.")
        self.trend_sag = TrendChart("Voltage sag under load", "V", self, decimals=2)
        self.trend_sag.setToolTip("Volts lost between arming and the lowest reading in flight. "
                                  "A growing sag is an early sign of a tired pack.")
        self.trend_dur = TrendChart("Flight duration", "min", self, decimals=1)
        for chart in (self.trend_use, self.trend_sag, self.trend_dur):
            tl.addWidget(chart, 1)
        self.trends.setVisible(False)
        root.addWidget(self.trends)

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
        self._shown: List[FlightRecord] = []
        self.table.itemSelectionChanged.connect(self._on_selection)
        self.table.itemDoubleClicked.connect(lambda _it: self.open_selected())
        root.addWidget(self.table, 1)

        # ── footer ──
        foot = QHBoxLayout()
        self.lbl_count = QLabel("No flights recorded yet", self)
        self.lbl_count.setToolTip("A record is written each time the vehicle disarms")
        self.lbl_count.setObjectName("fieldSubLabel")
        foot.addWidget(self.lbl_count)
        foot.addStretch()
        self.btn_trends = QPushButton("Trends", self)
        self.btn_trends.setCheckable(True)
        self.btn_trends.setToolTip("Battery health and duration across the flights shown")
        self.btn_trends.toggled.connect(self.trends.setVisible)
        foot.addWidget(self.btn_trends)
        self.btn_details = QPushButton("Details…", self)
        self.btn_details.setToolTip("Open the selected flight: charts and ground track (or double-click a row)")
        self.btn_details.setEnabled(False)
        self.btn_details.clicked.connect(self.open_selected)
        foot.addWidget(self.btn_details)
        btn_csv = QPushButton("Export CSV", self)
        btn_csv.clicked.connect(self._export_csv)
        foot.addWidget(btn_csv)
        btn_bundle = QPushButton("Export Bundle", self)
        btn_bundle.setToolTip("Zip flight history, settings and logs for a bug report")
        btn_bundle.clicked.connect(self._export_bundle)
        foot.addWidget(btn_bundle)
        btn_reset = QPushButton("Reset Logs", self)
        btn_reset.setObjectName("mapDanger")
        btn_reset.setToolTip("Permanently erase all recorded flight history")
        btn_reset.clicked.connect(self._reset_logs)
        foot.addWidget(btn_reset)
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
                if c and COLUMNS[c] != "MODES":
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                elif COLUMNS[c] == "MODES":
                    item.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                self.table.setItem(i, c, item)
        self._shown = list(rows)
        self._on_selection()
        self._update_trends(rows)

        total = len(self._records)
        self.lbl_count.setText(
            f"Showing {len(rows)} of {total} recorded flight(s)" if total
            else "No flights recorded yet")

    def _selected_record(self) -> Optional[FlightRecord]:
        row = self.table.currentRow()
        return self._shown[row] if 0 <= row < len(self._shown) else None

    def _on_selection(self) -> None:
        self.btn_details.setEnabled(self._selected_record() is not None)

    def open_selected(self) -> Optional[FlightDetailDialog]:
        """Open the detail view for the selected flight (None if nothing selected)."""
        rec = self._selected_record()
        if rec is None:
            return None
        dlg = FlightDetailDialog(rec, self)
        dlg.setAttribute(Qt.WA_DeleteOnClose, True)
        dlg.show()
        self._detail = dlg
        return dlg

    def _update_trends(self, rows: List[FlightRecord]) -> None:
        """Trend charts over the flights shown, oldest first."""
        chrono = sorted(rows, key=lambda r: r.started_epoch)
        self.trend_use.set_values([r.battery_used_per_min for r in chrono])
        self.trend_sag.set_values([r.voltage_sag for r in chrono])
        self.trend_dur.set_values([r.duration_s / 60.0 if r.duration_s > 0 else None for r in chrono])

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

    def _export_bundle(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Export diagnostics bundle", default_bundle_name(), "Zip files (*.zip)")
        if not path:
            return
        try:
            names = create_bundle(Path(path))
        except OSError as exc:
            QMessageBox.warning(self, "Export diagnostics bundle", f"Could not write bundle: {exc}")
            return
        self.lbl_count.setText(f"Exported diagnostics bundle ({len(names)} files) to {path}")

    # Deliberately simple: this is friction against an accidental click on an
    # irreversible action, the same purpose Reset Map's confirmation dialog
    # serves elsewhere in this app, not a real access-control boundary. The
    # password itself is the confirmation gate - correct password clears the
    # log immediately, no second Yes/No on top of it.
    RESET_PASSWORD = "admin"

    def _reset_logs(self) -> None:
        if not self._records:
            QMessageBox.information(self, "Reset Flight Logs",
                                    "There are no recorded flights to clear.")
            return
        password, ok = QInputDialog.getText(
            self, "Reset Flight Logs",
            "This permanently deletes all recorded flight history.\n"
            "Enter the admin password to continue:",
            QLineEdit.Password)
        if not ok:
            return
        if password != self.RESET_PASSWORD:
            QMessageBox.warning(self, "Reset Flight Logs", "Incorrect password.")
            return
        self.logger.clear_all()
        self._records = []
        self._apply_filters()
        QMessageBox.information(self, "Reset Flight Logs",
                                "Flight log history has been cleared.")
