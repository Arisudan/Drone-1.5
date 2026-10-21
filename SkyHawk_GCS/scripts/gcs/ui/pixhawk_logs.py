"""
================================================================================
MODULE: pixhawk_logs.py
PURPOSE: Flight Logs workspace - the Pixhawk's REAL flight logs (.ulg)
================================================================================
ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (Flight Logs tab, "PIXHAWK LOGS" view)
  * Upstream:      jetson/flight_log_server.py (mavlink_radiomaster) on http <jetson>:8095
  * Downstream:    ~/Documents/pixhawk_logs/*.ulg for PlotJuggler / PX4 Flight Review
WHERE THE LOGS COME FROM:
  The Jetson's mavlink-routerd records PX4's MAVLink log stream for every armed
  session (one .ulg per arm..disarm), so a flight's real log - raw IMU, every EKF2
  internal including the LIO (external-vision) fusion, motor outputs, failsafes - is
  on the Jetson the moment the vehicle disarms and comes to this station over Wi-Fi
  in seconds. PX4's own SD-card logs can be copied too ("SD card..."), but that runs
  over TELEM2 at ~20-40 KiB/s - minutes per flight - so it is the fallback.
WHY BOTH VIEWS:
  The GCS FLIGHTS view (logs_tab.py) is the station's own record - what the operator
  saw and commanded, at 1-10 Hz. This view is what the autopilot itself recorded, at
  full rate. An incident needs the second; airframe hours and battery trends come
  from the first. Each Pixhawk log is matched to its GCS flight by start time.
================================================================================
"""
from __future__ import annotations

import csv
import json
import shutil
import subprocess
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, List, Optional

from PyQt5.QtCore import QObject, Qt, QTimer, QUrl, pyqtSignal
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtWidgets import (
    QAbstractItemView, QDialog, QHBoxLayout, QHeaderView, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

PORT = 8095
DOWNLOAD_DIR = Path.home() / "Documents" / "pixhawk_logs"
MATCH_S = 120.0          # a .ulg and a GCS flight starting this close are the same flight
COLUMNS = ["DATE", "DURATION", "ARMED", "SOURCE", "SIZE", "MAX TILT", "BATTERY USED", "MIN BATTERY",
           "LIO REJECTED", "WARNINGS", "GCS FLIGHT"]
BATTERY_CSV = "battery_report.csv"
CSV_FIELDS = ["date", "log", "source", "duration_s", "armed_s", "used_mah", "used_pct", "energy_wh",
              "start_v", "end_v", "min_v", "start_pct", "end_pct", "avg_current_a", "max_current_a",
              "capacity_mah"]


# ── pure helpers (tested without a network) ──
def _epoch(iso: Optional[str]) -> Optional[float]:
    if not iso:
        return None
    try:
        return datetime.strptime(iso, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc).timestamp()
    except ValueError:
        return None


def match_flight(start_utc: Optional[str], records) -> Optional[object]:
    """The GCS FlightRecord that started within MATCH_S of this log, nearest first."""
    t = _epoch(start_utc)
    if t is None:
        return None
    best, best_dt = None, MATCH_S
    for r in records:
        dt = abs(getattr(r, "started_epoch", 0.0) - t)
        if dt <= best_dt:
            best, best_dt = r, dt
    return best


def _hms(s) -> str:
    if s is None:
        return "--"
    s = int(round(s))
    return f"{s // 60}:{s % 60:02d}" if s < 3600 else f"{s // 3600}:{s % 3600 // 60:02d}:{s % 60:02d}"


def battery_cell(s: dict):
    """(text, tooltip) for BATTERY USED from flight_log_server.py's summary["battery"]."""
    b = (s or {}).get("battery") or {}
    mah, pct = b.get("used_mah"), b.get("used_pct")
    if mah is None and pct is None:
        return "--", ""
    text = " · ".join(x for x in (None if mah is None else f"{mah:.0f} mAh",
                                    None if pct is None else f"{pct:.1f}%") if x)
    tip = []
    if b.get("start_v") is not None:
        tip.append(f"Voltage {b['start_v']:.2f} → {b['end_v']:.2f} V")
    if b.get("start_pct") is not None and b.get("end_pct") is not None:
        tip.append(f"Charge {b['start_pct']:.1f} → {b['end_pct']:.1f} %")
    if b.get("energy_wh") is not None:
        tip.append(f"Energy {b['energy_wh']:.2f} Wh")
    if b.get("avg_current_a") is not None:
        tip.append(f"Current avg {b['avg_current_a']:.1f} A, peak {b['max_current_a']:.1f} A")
    if b.get("capacity_mah"):
        tip.append(f"Pack {b['capacity_mah']} mAh (BAT1_CAPACITY)")
    return text, "\n".join(tip)


def battery_totals(logs) -> tuple:
    """(flights with battery data, total mAh, total Wh) over the listed logs."""
    n, mah, wh = 0, 0.0, 0.0
    for log in logs:
        b = (log.get("summary") or {}).get("battery") or {}
        if b.get("used_mah") is not None:
            n += 1
            mah += b["used_mah"]
            wh += b.get("energy_wh") or 0.0
    return n, mah, wh


def battery_csv_rows(logs) -> List[dict]:
    """One row per finished log for the battery report (CSV_FIELDS)."""
    rows = []
    for log in logs:
        if log.get("recording"):
            continue
        s = log.get("summary") or {}
        b = s.get("battery") or {}
        rows.append({
            "date": s.get("start_utc") or "", "log": log.get("name", ""), "source": log.get("source", ""),
            "duration_s": s.get("duration_s"), "armed_s": s.get("armed_s"),
            "used_mah": b.get("used_mah"), "used_pct": b.get("used_pct"), "energy_wh": b.get("energy_wh"),
            "start_v": b.get("start_v"), "end_v": b.get("end_v"), "min_v": s.get("min_voltage_v"),
            "start_pct": b.get("start_pct"), "end_pct": b.get("end_pct"),
            "avg_current_a": b.get("avg_current_a"), "max_current_a": b.get("max_current_a"),
            "capacity_mah": b.get("capacity_mah")})
    return rows


def row_cells(log: dict, records=()) -> List[str]:
    """Table text for one /logs entry (flight_log_server.py)."""
    s = log.get("summary") or {}
    if log.get("recording"):
        date = "recording now..."
    elif s.get("start_utc"):
        date = datetime.fromtimestamp(_epoch(s["start_utc"])).strftime("%Y-%m-%d %H:%M:%S")
    else:
        date = "no clock (SD, no GPS)" if log.get("source") == "sd" else "--"
    warn = s.get("warnings")
    warn_txt = "--" if warn is None else str(warn)
    if s.get("failsafe"):
        warn_txt = f"FAILSAFE, {warn_txt}"
    if s.get("error"):
        warn_txt = "unreadable"
    rec = match_flight(s.get("start_utc"), records)
    return [
        date,
        _hms(s.get("duration_s")),
        _hms(s.get("armed_s")),
        "Jetson (live)" if log.get("source") == "jetson" else "SD card",
        f"{log.get('size', 0) / 1048576:.1f} MB",
        "--" if s.get("max_tilt_deg") is None else f"{s['max_tilt_deg']:.1f}°",
        battery_cell(s)[0],
        "--" if s.get("min_voltage_v") is None else f"{s['min_voltage_v']:.2f} V",
        "--" if s.get("ev_rejected_pct") is None else f"{s['ev_rejected_pct']:.1f}%",
        warn_txt,
        (rec.started_utc or "").replace("T", " ").rstrip("Z")[11:19] if rec else "--",
    ]


# ── network on a worker thread ──
class _Relay(QObject):
    """Carries a worker's (callback, result, error) back to the GUI thread (queued signal)."""
    done = pyqtSignal(object, object, str)


def run_in_background(owner, fn, on_done) -> None:
    """fn() on a daemon thread, on_done(result, error) on the GUI thread.

    A plain daemon thread, NOT a QThread owned by the widget: closing the window (or a test
    tearing it down) mid-request destroyed a running QThread and aborted the whole process
    ("QThread: Destroyed while thread is still running"). If the owner is gone when the
    request ends, the result is dropped."""
    if not hasattr(owner, "_relay"):
        owner._relay = _Relay(owner)
        owner._relay.done.connect(lambda cb, res, err: cb(res, err))
    owner._jobs.append(fn)
    relay = owner._relay

    def work():
        try:
            res, err = fn(), ""
        except Exception as e:                         # noqa: BLE001 - shown to the operator
            res, err = None, str(e) or type(e).__name__

        def finish(r, e):
            if fn in owner._jobs:
                owner._jobs.remove(fn)
            on_done(r, e)
        try:
            relay.done.emit(finish, res, err)
        except RuntimeError:                           # owner already deleted
            pass
    threading.Thread(target=work, daemon=True).start()


def _get_json(url, timeout=60.0, method="GET"):
    req = urllib.request.Request(url, method=method, data=b"" if method == "POST" else None)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        raise RuntimeError(json.loads(e.read() or b"{}").get("error", str(e)))


class PixhawkLogsPanel(QWidget):
    """List / download the Pixhawk's .ulg logs from the Jetson's flight_log_server."""

    def __init__(self, host_fn: Optional[Callable[[], str]] = None,
                 records_fn: Optional[Callable[[], list]] = None, parent=None):
        super().__init__(parent)
        self.host_fn = host_fn
        self.records_fn = records_fn or (lambda: [])
        self.logs: List[dict] = []
        self._jobs = []

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        top = QHBoxLayout()
        self.lbl_source = QLabel("", self)
        self.lbl_source.setObjectName("fieldSubLabel")
        self.lbl_source.setToolTip("The Jetson records PX4's own flight log while the vehicle is armed")
        top.addWidget(self.lbl_source, 1)
        self.btn_sd = QPushButton("SD card…", self)
        self.btn_sd.setToolTip("Copy a log from the Pixhawk's SD card (slow: over the telemetry link; disarmed only)")
        self.btn_sd.clicked.connect(self.open_sd_dialog)
        top.addWidget(self.btn_sd)
        self.btn_refresh = QPushButton("Refresh", self)
        self.btn_refresh.clicked.connect(self.refresh)
        top.addWidget(self.btn_refresh)
        root.addLayout(top)

        self.table = QTableWidget(0, len(COLUMNS), self)
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        head = self.table.horizontalHeader()
        for c in range(len(COLUMNS)):
            head.setSectionResizeMode(c, QHeaderView.Stretch if c == 0 else QHeaderView.ResizeToContents)
        self.table.itemSelectionChanged.connect(self._on_selection)
        self.table.itemDoubleClicked.connect(lambda _it: self.download_selected())
        root.addWidget(self.table, 1)

        foot = QHBoxLayout()
        self.lbl_status = QLabel("", self)
        self.lbl_status.setObjectName("fieldSubLabel")
        foot.addWidget(self.lbl_status, 1)
        self.btn_download = QPushButton("Download", self)
        self.btn_download.setToolTip(f"Save the selected .ulg to {DOWNLOAD_DIR} (or double-click a row)")
        self.btn_download.clicked.connect(self.download_selected)
        foot.addWidget(self.btn_download)
        self.plotjuggler = shutil.which("plotjuggler") or shutil.which("PlotJuggler")
        self.btn_plot = QPushButton("Open in PlotJuggler", self)
        self.btn_plot.setVisible(bool(self.plotjuggler))
        self.btn_plot.clicked.connect(self.plot_selected)
        foot.addWidget(self.btn_plot)
        self.btn_csv = QPushButton("Export battery CSV", self)
        self.btn_csv.setToolTip(f"Battery used per flight (mAh, %, Wh, volts, current) -> {DOWNLOAD_DIR / BATTERY_CSV}")
        self.btn_csv.clicked.connect(self.export_battery_csv)
        self.btn_csv.setEnabled(False)
        foot.addWidget(self.btn_csv)
        btn_folder = QPushButton("Open folder", self)
        btn_folder.setToolTip("Downloaded logs; upload one to review.px4.io (PX4 Flight Review) for the full report")
        btn_folder.clicked.connect(self.open_folder)
        foot.addWidget(btn_folder)
        root.addLayout(foot)

        self._set_source_label()
        self._on_selection()
        self._refreshed = False

    # ── helpers ──
    def _host(self) -> Optional[str]:
        return self.host_fn() if self.host_fn else None

    def _base(self) -> str:
        return f"http://{self._host()}:{PORT}"

    def _set_source_label(self, extra: str = "") -> None:
        h = self._host()
        self.lbl_source.setText(
            (f"Pixhawk flight logs recorded on the Jetson ({h}) - one per armed flight" if h
             else "Pixhawk flight logs: no vehicle connection configured") + extra)

    def _run(self, fn, on_done) -> None:
        run_in_background(self, fn, on_done)

    def showEvent(self, ev):                     # first time the view is opened
        super().showEvent(ev)
        if not self._refreshed and self.host_fn:
            self._refreshed = True
            self.refresh()

    # ── list ──
    def refresh(self) -> None:
        if not self._host():
            return
        self.btn_refresh.setEnabled(False)
        self.lbl_status.setText("Loading the log list from the Jetson…")
        url = self._base() + "/logs"
        self._run(lambda: _get_json(url), self._on_list)

    def _on_list(self, logs, err) -> None:
        self.btn_refresh.setEnabled(True)
        if err:
            self.lbl_status.setText(f"Could not reach the Jetson's log server: {err}")
            return
        self.set_logs(logs)

    def set_logs(self, logs: List[dict]) -> None:
        self.logs = list(logs)
        records = self.records_fn()
        self.table.setRowCount(len(self.logs))
        for i, log in enumerate(self.logs):
            for c, text in enumerate(row_cells(log, records)):
                item = QTableWidgetItem(text)
                item.setTextAlignment((Qt.AlignLeft if c in (0, 3) else Qt.AlignRight) | Qt.AlignVCenter)
                s = log.get("summary") or {}
                if COLUMNS[c] == "WARNINGS" and (s.get("warning_texts") or s.get("error")):
                    item.setToolTip("\n".join(s.get("warning_texts") or [s.get("error", "")]))
                if COLUMNS[c] == "BATTERY USED":
                    item.setToolTip(battery_cell(s)[1])
                self.table.setItem(i, c, item)
        n, mah, wh = battery_totals(self.logs)
        total = f" - battery used: {mah:.0f} mAh, {wh:.1f} Wh over {n} flight(s)" if n else ""
        self.lbl_status.setText(f"{len(self.logs)} Pixhawk log(s){total}" if self.logs else
                                "No Pixhawk logs yet - one is recorded each time the vehicle is armed")
        self.btn_csv.setEnabled(bool(self.logs))
        self._on_selection()

    def export_battery_csv(self) -> Optional[Path]:
        """Write the battery report for the listed logs; returns its path (None on failure)."""
        dest = DOWNLOAD_DIR / BATTERY_CSV
        try:
            DOWNLOAD_DIR.mkdir(parents=True, exist_ok=True)
            with open(dest, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
                w.writeheader()
                w.writerows(battery_csv_rows(self.logs))
        except OSError as e:
            self.lbl_status.setText(f"Could not write {dest}: {e}")
            return None
        self.lbl_status.setText(f"Battery report saved: {dest}")
        return dest

    def _selected(self) -> Optional[dict]:
        row = self.table.currentRow()
        return self.logs[row] if 0 <= row < len(self.logs) else None

    def _on_selection(self) -> None:
        log = self._selected()
        ok = log is not None and not log.get("recording")
        self.btn_download.setEnabled(ok)
        self.btn_plot.setEnabled(ok)

    # ── download / open ──
    def _local_path(self, log: dict) -> Path:
        return DOWNLOAD_DIR / Path(log["name"]).name

    def download_selected(self, then: Optional[Callable[[Path], None]] = None) -> None:
        log = self._selected()
        if log is None or log.get("recording"):
            return
        dest = self._local_path(log)
        if dest.exists() and dest.stat().st_size == log.get("size"):
            self.lbl_status.setText(f"Already downloaded: {dest}")
            if then:
                then(dest)
            return
        url = f"{self._base()}/logs/{log['name']}"
        self.lbl_status.setText(f"Downloading {log['name']} ({log.get('size', 0) / 1048576:.1f} MB)…")
        self.btn_download.setEnabled(False)

        def fetch():
            DOWNLOAD_DIR.mkdir(parents=True, exist_ok=True)
            tmp = dest.with_suffix(".part")
            t0 = time.time()
            with urllib.request.urlopen(url, timeout=30) as r, open(tmp, "wb") as f:
                shutil.copyfileobj(r, f, 1 << 16)
            tmp.replace(dest)
            return dest, time.time() - t0

        def done(res, err):
            self._on_selection()
            if err:
                self.lbl_status.setText(f"Download failed: {err}")
                return
            path, secs = res
            self.lbl_status.setText(f"Saved {path} ({secs:.1f} s)")
            if then:
                then(path)
        self._run(fetch, done)

    def plot_selected(self) -> None:
        if self.plotjuggler:
            self.download_selected(then=lambda p: subprocess.Popen([self.plotjuggler, "-d", str(p)]))

    def open_folder(self) -> None:
        DOWNLOAD_DIR.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(DOWNLOAD_DIR)))

    # ── SD card ──
    def open_sd_dialog(self) -> Optional["SdCardDialog"]:
        if not self._host():
            return None
        dlg = SdCardDialog(self._base(), self)
        dlg.copied.connect(lambda _name: self.refresh())
        dlg.setAttribute(Qt.WA_DeleteOnClose, True)
        dlg.show()
        self._sd_dialog = dlg
        return dlg


class SdCardDialog(QDialog):
    """PX4's SD-card logs (MAVLink log protocol, via the Jetson) and a copy-to-Jetson button."""

    copied = pyqtSignal(str)

    def __init__(self, base: str, parent=None):
        super().__init__(parent)
        self.base = base
        self.entries: List[dict] = []
        self._jobs = []
        self.setWindowTitle("Pixhawk SD card logs")
        self.resize(560, 420)
        lay = QVBoxLayout(self)
        note = QLabel("Copies run over the telemetry link (~20-40 KiB/s: a few minutes per flight) "
                      "and only while DISARMED. Flights the Jetson recorded live are already in the list.", self)
        note.setWordWrap(True)
        note.setObjectName("fieldSubLabel")
        lay.addWidget(note)
        self.table = QTableWidget(0, 3, self)
        self.table.setHorizontalHeaderLabels(["LOG", "SIZE", "PX4 TIME (UTC)"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        lay.addWidget(self.table, 1)
        row = QHBoxLayout()
        self.lbl = QLabel("Reading the SD card list…", self)
        self.lbl.setObjectName("fieldSubLabel")
        row.addWidget(self.lbl, 1)
        self.btn_copy = QPushButton("Copy to Jetson", self)
        self.btn_copy.setEnabled(False)
        self.btn_copy.clicked.connect(self._copy)
        row.addWidget(self.btn_copy)
        lay.addLayout(row)
        self.table.itemSelectionChanged.connect(
            lambda: self.btn_copy.setEnabled(self.table.currentRow() >= 0))
        self.poll = QTimer(self)
        self.poll.setInterval(2000)
        self.poll.timeout.connect(self._poll)
        self._run(lambda: _get_json(self.base + "/sd"), self._on_list)

    def _run(self, fn, on_done):
        run_in_background(self, fn, on_done)

    def _on_list(self, entries, err):
        if err:
            self.lbl.setText(f"SD card list unavailable: {err}")
            return
        self.entries = list(entries)
        self.table.setRowCount(len(self.entries))
        for i, e in enumerate(self.entries):
            when = (datetime.fromtimestamp(e["time_utc"], timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
                    if e.get("time_utc", 0) > 1e9 else "no clock (no GPS)")
            for c, text in enumerate([f"#{e['id']}", f"{e['size'] / 1048576:.2f} MB", when]):
                self.table.setItem(i, c, QTableWidgetItem(text))
        self.lbl.setText(f"{len(self.entries)} log(s) on the SD card")

    def _copy(self):
        row = self.table.currentRow()
        if not 0 <= row < len(self.entries):
            return
        e = self.entries[row]
        self.btn_copy.setEnabled(False)
        self.lbl.setText(f"Starting the copy of log #{e['id']}…")
        self._run(lambda: _get_json(f"{self.base}/sd/{e['id']}/fetch", method="POST"), self._on_started)

    def _on_started(self, _res, err):
        if err:
            self.lbl.setText(f"Copy refused: {err}")
            self.btn_copy.setEnabled(True)
            return
        self.poll.start()

    def _poll(self):
        self._run(lambda: _get_json(self.base + "/sd/status", timeout=10), self._on_status)

    def _on_status(self, st, err):
        if err or not st:
            return
        if st.get("state") == "running":
            size = st.get("size") or 1
            self.lbl.setText(f"Copying #{st.get('id')}: {100 * st.get('got', 0) / size:.0f}% "
                             f"of {size / 1048576:.2f} MB")
        elif st.get("state") == "done":
            self.poll.stop()
            self.lbl.setText(f"Copied to the Jetson in {st.get('seconds')} s - it is now in the Pixhawk log list")
            self.btn_copy.setEnabled(True)
            self.copied.emit(st.get("name", ""))
        elif st.get("state") == "error":
            self.poll.stop()
            self.lbl.setText(f"Copy failed: {st.get('error')}")
            self.btn_copy.setEnabled(True)
