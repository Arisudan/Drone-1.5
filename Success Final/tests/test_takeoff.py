"""Takeoff altitude: NAV_TAKEOFF param7 is absolute (above sea level), not a height."""

import os
import time
import unittest
from unittest import mock

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core.takeoff import AMSL_MAX_AGE_S, takeoff_altitude_param


class RuleTest(unittest.TestCase):
    def test_a_vehicle_with_an_altitude_reference_gets_current_plus_height(self):
        self.assertEqual(takeoff_altitude_param(1.5, 489.0, 0.2), (490.5, "amsl"))

    def test_no_reading_keeps_the_old_behaviour(self):
        self.assertEqual(takeoff_altitude_param(1.5, None, None), (1.5, "as-is"))

    def test_a_stale_reading_is_not_trusted(self):
        self.assertEqual(takeoff_altitude_param(1.5, 489.0, AMSL_MAX_AGE_S + 1), (1.5, "as-is"))
        self.assertEqual(takeoff_altitude_param(1.5, 489.0, AMSL_MAX_AGE_S), (490.5, "amsl"))

    def test_a_vehicle_near_zero_is_unchanged_in_effect(self):
        """The indoor airframe the old behaviour was proven on: ~0 m reference, so
        0 + 1.5 is the same 1.5 as before."""
        self.assertAlmostEqual(takeoff_altitude_param(1.5, 0.0, 0.1)[0], 1.5)

    def test_negative_altitudes_below_sea_level_work(self):
        self.assertEqual(takeoff_altitude_param(2.0, -30.0, 0.1), (-28.0, "amsl"))

    def test_a_negative_age_is_rejected(self):
        self.assertEqual(takeoff_altitude_param(1.5, 489.0, -1.0)[1], "as-is")


class TelemetryTest(unittest.TestCase):
    def test_global_position_is_stored_in_metres_with_a_timestamp(self):
        from core.telemetry import TelemetrySnapshot
        t = TelemetrySnapshot()
        self.assertEqual(t.amsl_reading(), (None, None))
        t.update_global_position(type("M", (), {"alt": 489_013})())
        alt, age = t.amsl_reading()
        self.assertAlmostEqual(alt, 489.013)
        self.assertLess(age, 1.0)


class WorkerTest(unittest.TestCase):
    def worker(self):
        from protocol.mavlink_worker import MAVLinkWorker
        w = MAVLinkWorker()
        w._connected = True
        w.master = mock.Mock()
        return w

    def test_param7_is_the_absolute_target_when_given(self):
        w = self.worker()
        w.takeoff(1.5, target_amsl=490.5)
        args = w.master.mav.command_long_send.call_args[0]
        self.assertEqual(args[2], 22)                  # MAV_CMD_NAV_TAKEOFF
        self.assertEqual(args[-1], 490.5)

    def test_with_a_reference_yaw_lat_lon_are_nan_meaning_current(self):
        import math
        w = self.worker()
        w.takeoff(1.5, target_amsl=490.5)
        a = w.master.mav.command_long_send.call_args[0]
        # (sys, comp, cmd, confirmation, p1, p2, p3, p4=yaw, p5=lat, p6=lon, p7=alt)
        self.assertTrue(all(math.isnan(x) for x in a[7:10]), a)
        self.assertEqual(a[4:7], (0, 0, 0))

    def test_without_a_reference_the_bench_verified_zeros_are_kept(self):
        w = self.worker()
        w.takeoff(1.5)
        a = w.master.mav.command_long_send.call_args[0]
        self.assertEqual(a[4:11], (0, 0, 0, 0, 0, 0, 1.5))

    def test_param7_is_the_height_unchanged_without_a_reference(self):
        w = self.worker()
        w.takeoff(1.5)
        self.assertEqual(w.master.mav.command_long_send.call_args[0][-1], 1.5)

    def test_the_offboard_backup_setpoint_stays_relative(self):
        w = self.worker()
        w.takeoff(1.5, target_amsl=490.5)
        self.assertEqual(w.sp_z, -1.5)                 # local frame: NED, relative to the origin


class CommandTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_takeoff_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        from ui.styles import build_stylesheet
        cls.app.setStyleSheet(build_stylesheet())
        cls.win = drone_gcs.DroneGCSMainWindow(settings=apply_overrides(load_settings()))

    @classmethod
    def tearDownClass(cls):
        cls.win.shutdown_workers()
        cls.win.close()

    def setUp(self):
        w = self.win
        w.worker = mock.Mock()
        w.worker.isRunning.return_value = True
        t = w.last_telemetry
        t.connected, t.armed = True, True
        t.altitude, t.z, t.landed_state = 0.0, 0.0, 1        # on the ground
        t.alt_amsl_m, t.last_global_time = 489.0, time.time()

    def tearDown(self):
        self.win.worker = None

    def test_takeoff_sends_the_absolute_target(self):
        self.win._cmd_takeoff(1.5)
        self.win.worker.takeoff.assert_called_once()
        args, kwargs = self.win.worker.takeoff.call_args
        self.assertEqual(args[0], 1.5)
        self.assertAlmostEqual(kwargs["target_amsl"], 490.5, places=1)

    def test_takeoff_without_a_reference_sends_none_and_says_so(self):
        w = self.win
        w.last_telemetry.last_global_time = 0.0
        w._cmd_takeoff(1.5)
        args, kwargs = w.worker.takeoff.call_args
        self.assertIsNone(kwargs["target_amsl"])

    def test_the_console_says_what_it_sent(self):
        w = self.win
        lines = []
        w.console.log_cmd = lambda m: lines.append(m)
        w._cmd_takeoff(1.5)
        self.assertTrue(any("490.5 m above sea level" in m for m in lines), lines)


if __name__ == "__main__":
    unittest.main()
