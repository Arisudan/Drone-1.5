"""Flight logs: per-flight time series, the detail view and the Logs tab."""

import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication

from core.flight_log import FlightLogger, FlightRecord
from ui import flight_detail as fd


def tel(armed=True, t=0.0, **kw):
    d = dict(armed=armed, x=0.0, y=0.0, z=0.0, altitude=0.0, ground_speed=0.0,
             battery_percent=90, battery_voltage=16.0, flight_mode="POSCTL")
    d.update(kw)
    return SimpleNamespace(**d)


def record(n=60, **kw):
    r = FlightRecord(started_utc="2026-09-17T10:15:00Z", started_epoch=1.79e9,
                     duration_s=n, battery_start_pct=96, battery_end_pct=90,
                     voltage_start=16.6, voltage_min=16.0, modes=["POSCTL"], **kw)
    ts = list(range(n))
    r.series = {"t": ts, "alt": [1.0] * n, "speed": [0.5] * n, "batt": [90.0] * n,
                "volt": [16.0] * n, "x": [float(i) for i in ts], "y": [0.0] * n}
    return r


class SeriesRecordingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.lg = FlightLogger(Path(self.tmp.name) / "f.jsonl")

    def tearDown(self):
        self.tmp.cleanup()

    def test_a_flight_records_points_and_voltage(self):
        clock = [1000.0]
        with mock.patch("core.flight_log.time.time", lambda: clock[0]):
            self.lg.update(tel(battery_voltage=16.5))
            for i in range(1, 6):
                clock[0] += 1.0
                self.lg.update(tel(altitude=i * 0.2, battery_voltage=16.5 - i * 0.1))
            clock[0] += 1.0
            rec = self.lg.update(tel(armed=False))
        self.assertTrue(rec.has_series)
        self.assertGreaterEqual(len(rec.series["t"]), 5)
        self.assertAlmostEqual(rec.voltage_start, 16.5)
        self.assertAlmostEqual(rec.voltage_min, 16.0)
        self.assertAlmostEqual(rec.voltage_sag, 0.5)

    def test_sampling_is_about_once_a_second(self):
        clock = [1000.0]
        with mock.patch("core.flight_log.time.time", lambda: clock[0]):
            self.lg.update(tel())
            for _ in range(100):               # 100 ticks inside 1 s
                clock[0] += 0.01
                self.lg.update(tel())
        self.assertLessEqual(len(self.lg.active.series["t"]), 3)

    def test_long_flights_are_thinned_not_truncated(self):
        self.lg.update(tel())
        rec = self.lg.active
        rec.series = {k: list(range(self.lg.MAX_SERIES_POINTS)) for k in ("t", "alt", "speed", "batt", "volt", "x", "y")}
        self.lg._last_sample_t = -10
        self.lg._sample(tel(), force=True)
        self.assertLess(len(rec.series["t"]), self.lg.MAX_SERIES_POINTS)
        self.assertEqual(rec.series["t"][0], 0)

    def test_round_trip_and_forward_compat(self):
        self.lg.append(record())
        with open(self.lg.path, "a", encoding="utf-8") as fh:
            d = json.loads(open(self.lg.path).readline())
            d["from_the_future"] = 1
            fh.write(json.dumps(d) + "\n")
            fh.write(json.dumps({"started_utc": "x", "duration_s": 5}) + "\n")   # an old record
        recs = self.lg.load_all()
        self.assertEqual(len(recs), 3)
        self.assertTrue(recs[0].has_series)
        self.assertFalse(recs[2].has_series)
        self.assertIsNone(recs[2].voltage_sag)

    def test_battery_use_per_minute(self):
        self.assertAlmostEqual(record(n=120).battery_used_per_min, 3.0)
        self.assertIsNone(record(n=10).battery_used_per_min)


class HelperTest(unittest.TestCase):
    def test_ticks_are_round_and_inside(self):
        t = fd.nice_ticks(0.0, 1.4)
        self.assertTrue(all(0 <= v <= 1.4 for v in t))
        self.assertEqual(fd.nice_ticks(2, 2), [2])

    def test_clock_text(self):
        self.assertEqual(fd.clock_text(65), "1:05")
        self.assertEqual(fd.clock_text(3725), "1:02:05")

    def test_padded_range_is_never_flat(self):
        lo, hi = fd.padded_range([1.0, 1.0])
        self.assertLess(lo, hi)
        self.assertEqual(fd.padded_range([]), (0.0, 1.0))
        self.assertGreaterEqual(fd.padded_range([0.2, 3], floor_zero=True)[0], 0.0)

    def test_nearest_index(self):
        xs = [0, 10, 20]
        self.assertEqual(fd.nearest_index(xs, 4), 0)
        self.assertEqual(fd.nearest_index(xs, 6), 1)
        self.assertEqual(fd.nearest_index(xs, 99), 2)
        self.assertEqual(fd.nearest_index([], 1), -1)

    def test_scale_bar_is_round(self):
        self.assertIn(fd.scale_bar_length(5.0), (1.0, 2.0))
        self.assertEqual(fd.scale_bar_length(0), 1.0)


class UiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        from ui.styles import build_stylesheet
        cls.app.setStyleSheet(build_stylesheet())

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.lg = FlightLogger(Path(self.tmp.name) / "f.jsonl")
        self.lg.append(record(n=120))
        self.lg.append(FlightRecord(started_utc="2026-09-18T10:00:00Z", started_epoch=1.8e9,
                                    duration_s=40, modes=["POSCTL", "OFFBOARD"]))
        from ui.logs_tab import LogsTabWidget
        self.w = LogsTabWidget(self.lg)
        self.w.resize(1100, 700)
        self.w.show()
        self.app.processEvents()

    def tearDown(self):
        self.w.close()
        self.w.deleteLater()
        self.tmp.cleanup()

    def test_dates_are_iso(self):
        self.assertEqual(self.w.date_from.displayFormat(), "yyyy-MM-dd")

    def test_modes_are_left_aligned(self):
        for r in range(self.w.table.rowCount()):
            for c in range(self.w.table.columnCount()):
                it = self.w.table.item(r, c)
                if it and "POSCTL" in it.text():
                    self.assertTrue(it.textAlignment() & Qt.AlignLeft)

    def test_details_needs_a_selection_then_opens(self):
        self.w.table.clearSelection()
        self.assertIsNone(self.w.open_selected())
        self.w.table.selectRow(1)          # newest first -> the 2026-09-17 flight with a series
        d = self.w.open_selected()
        self.assertIsNotNone(d)
        self.assertTrue(d.rec.has_series)
        d.close()

    def test_flight_without_series_says_so(self):
        self.w.table.selectRow(0)
        d = self.w.open_selected()
        self.assertFalse(d.rec.has_series)
        self.assertTrue(d.lbl_note.text())
        self.assertFalse(d.btn_export.isEnabled())
        d.close()

    def test_csv_export(self):
        d = fd.FlightDetailDialog(record(n=30))
        p = os.path.join(self.tmp.name, "x.csv")
        self.assertEqual(d.write_csv(p), 30)
        lines = open(p).read().splitlines()
        self.assertEqual(len(lines), 31)
        self.assertTrue(lines[0].startswith("t_s,altitude_m"))
        d.close()

    def test_trends_toggle(self):
        self.w.btn_trends.setChecked(True)
        self.assertTrue(self.w.trends.isVisibleTo(self.w))
        self.w.btn_trends.setChecked(False)
        self.assertFalse(self.w.trends.isVisibleTo(self.w))


if __name__ == "__main__":
    unittest.main()
