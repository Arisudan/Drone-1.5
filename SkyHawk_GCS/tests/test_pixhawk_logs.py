"""Tests for ui/pixhawk_logs.py - the Flight Logs tab's PIXHAWK LOGS view.

Hermetic: a stub HTTP server stands in for the Jetson's flight_log_server.py
(mavlink_radiomaster jetson/), with the same /logs JSON and /logs/<name> download.
"""

import json
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from PyQt5.QtWidgets import QApplication

import ui.pixhawk_logs as PL
from ui.logs_tab import LogsTabWidget
from core.flight_log import FlightLogger

APP = QApplication.instance() or QApplication([])

LIVE = {"name": "00000-2026-10-19_15-07-02.ulg", "size": 886151, "mtime": 1.0,
        "source": "jetson", "recording": False,
        "summary": {"start_utc": "2026-10-19T09:37:02Z", "duration_s": 29.4, "armed_s": 12.0,
                    "max_tilt_deg": 2.3, "min_voltage_v": 15.85, "ev_rejected_pct": 0.0,
                    "warnings": 2, "warning_texts": ["EKF2 IMU0 ev_pos fusion stopped", "Low battery"],
                    "failsafe": True,
                    "battery": {"start_v": 16.71, "end_v": 16.59, "start_pct": 89.3, "end_pct": 85.8,
                                "capacity_mah": 10000, "used_pct": 3.5, "used_mah": 166.1,
                                "energy_wh": 3.06, "avg_current_a": 26.5, "max_current_a": 57.4}}}
SD_NOCLOCK = {"name": "sd/sd105-notime.ulg", "size": 440470, "mtime": 2.0, "source": "sd",
              "recording": False, "summary": {"start_utc": None, "duration_s": 6.0, "warnings": 0}}
RECORDING = {"name": "00001-2026-10-19_16-00-00.ulg", "size": 1000, "mtime": 3.0,
             "source": "jetson", "recording": True, "summary": None}


def wait_until(cond, timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        APP.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    return False


class PureHelpersTest(unittest.TestCase):
    def test_live_row(self):
        rec = SimpleNamespace(started_epoch=PL._epoch("2026-10-19T09:37:40Z"),
                              started_utc="2026-10-19T09:37:40Z")
        cells = dict(zip(PL.COLUMNS, PL.row_cells(LIVE, [rec])))
        self.assertEqual(cells["DURATION"], "0:29")
        self.assertEqual(cells["ARMED"], "0:12")
        self.assertEqual(cells["SOURCE"], "Jetson (live)")
        self.assertEqual(cells["MAX TILT"], "2.3°")
        self.assertEqual(cells["MIN BATTERY"], "15.85 V")
        self.assertEqual(cells["WARNINGS"], "FAILSAFE, 2")
        self.assertEqual(cells["GCS FLIGHT"], "09:37:40")      # matched, 38 s apart
        self.assertEqual(cells["BATTERY USED"], "166 mAh · 3.5%")

    def test_battery_tooltip_totals_and_csv_rows(self):
        tip = PL.battery_cell(LIVE["summary"])[1]
        for part in ("16.71 → 16.59 V", "89.3 → 85.8 %", "3.06 Wh", "peak 57.4 A"):
            self.assertIn(part, tip)
        self.assertEqual(PL.battery_cell(SD_NOCLOCK["summary"]), ("--", ""))
        self.assertEqual(PL.battery_totals([LIVE, SD_NOCLOCK, RECORDING]), (1, 166.1, 3.06))
        rows = PL.battery_csv_rows([LIVE, SD_NOCLOCK, RECORDING])     # the recording one is left out
        self.assertEqual([r["log"] for r in rows], [LIVE["name"], SD_NOCLOCK["name"]])
        self.assertEqual(rows[0]["used_mah"], 166.1)
        self.assertEqual(set(rows[0]), set(PL.CSV_FIELDS))

    def test_sd_log_without_a_clock_is_never_matched_or_dated(self):
        rec = SimpleNamespace(started_epoch=time.time(), started_utc="x")
        cells = dict(zip(PL.COLUMNS, PL.row_cells(SD_NOCLOCK, [rec])))
        self.assertEqual(cells["DATE"], "no clock (SD, no GPS)")
        self.assertEqual(cells["SOURCE"], "SD card")
        self.assertEqual(cells["GCS FLIGHT"], "--")
        self.assertEqual(cells["MAX TILT"], "--")

    def test_match_window(self):
        t = PL._epoch("2026-10-19T09:37:02Z")
        near = SimpleNamespace(started_epoch=t + 100)
        far = SimpleNamespace(started_epoch=t + 500)
        self.assertIs(PL.match_flight("2026-10-19T09:37:02Z", [far, near]), near)
        self.assertIsNone(PL.match_flight("2026-10-19T09:37:02Z", [far]))
        self.assertIsNone(PL.match_flight(None, [near]))

    def test_recording_row(self):
        self.assertEqual(PL.row_cells(RECORDING)[0], "recording now...")


class _Stub(BaseHTTPRequestHandler):
    body = b"ULog\x01\x12\x35" + bytes(range(256)) * 10

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/logs":
            data = json.dumps([LIVE, SD_NOCLOCK, RECORDING]).encode()
            ctype = "application/json"
        elif self.path == "/logs/" + LIVE["name"]:
            data, ctype = self.body, "application/octet-stream"
        else:
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class PanelAgainstStubServerTest(unittest.TestCase):
    def setUp(self):
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), _Stub)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.tmp = TemporaryDirectory()
        self._dl, PL.DOWNLOAD_DIR = PL.DOWNLOAD_DIR, Path(self.tmp.name)
        self._port, PL.PORT = PL.PORT, self.srv.server_address[1]

    def tearDown(self):
        PL.PORT, PL.DOWNLOAD_DIR = self._port, self._dl
        self.srv.shutdown()
        self.srv.server_close()
        self.tmp.cleanup()

    def test_refresh_lists_and_download_saves_the_ulg(self):
        p = PL.PixhawkLogsPanel(lambda: "127.0.0.1")
        p.refresh()
        self.assertTrue(wait_until(lambda: p.table.rowCount() == 3))
        self.assertIn("2", p.table.item(0, PL.COLUMNS.index("WARNINGS")).text())
        self.assertIn("ev_pos", p.table.item(0, PL.COLUMNS.index("WARNINGS")).toolTip())
        p.table.selectRow(2)                         # still recording: cannot be downloaded
        self.assertFalse(p.btn_download.isEnabled())
        p.table.selectRow(0)
        self.assertTrue(p.btn_download.isEnabled())
        p.download_selected()
        dest = Path(self.tmp.name) / LIVE["name"]
        self.assertTrue(wait_until(lambda: dest.exists()))
        self.assertEqual(dest.read_bytes(), _Stub.body)
        self.assertTrue(wait_until(lambda: not p._jobs))
        self.assertIn("166 mAh", p.lbl_status.text() + p.table.item(0, PL.COLUMNS.index("BATTERY USED")).text())
        csv_path = p.export_battery_csv()
        lines = csv_path.read_text().splitlines()
        self.assertEqual(lines[0].split(",")[:3], ["date", "log", "source"])
        self.assertEqual(len(lines), 3)                 # header + 2 finished logs
        self.assertIn("166.1", lines[1])

    def test_unreachable_server_is_reported_not_raised(self):
        PL.PORT = 9                                  # discard port: refused
        p = PL.PixhawkLogsPanel(lambda: "127.0.0.1")
        p.refresh()
        self.assertTrue(wait_until(lambda: "Could not reach" in p.lbl_status.text()))
        self.assertTrue(wait_until(lambda: not p._jobs))


class LogsTabViewSwitchTest(unittest.TestCase):
    def test_pixhawk_logs_first_and_gcs_flights_still_there(self):
        with TemporaryDirectory() as d:
            w = LogsTabWidget(FlightLogger(Path(d) / "flights.jsonl"))
            self.assertIs(w.views.currentWidget(), w.pixhawk)
            w.view_buttons.button(1).click()
            self.assertIs(w.views.currentIndex(), 1)
            self.assertTrue(w.views.currentWidget().isAncestorOf(w.table))
            self.assertIn("no vehicle connection", w.pixhawk.lbl_source.text())


if __name__ == "__main__":
    unittest.main()
