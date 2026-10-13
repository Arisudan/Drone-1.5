"""Body-relative `move` (forward / right / down) and its settings."""
import _env  # noqa: F401  -- must be first

import math
import unittest
from unittest import mock

from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])

from core.move_frame import body_to_ned  # noqa: E402


class MathTests(unittest.TestCase):
    def assertNE(self, got, n, e):
        self.assertAlmostEqual(got[0], n, places=6)
        self.assertAlmostEqual(got[1], e, places=6)

    def test_forward_follows_heading(self):
        self.assertNE(body_to_ned(1, 0, 0), 1, 0)        # facing North
        self.assertNE(body_to_ned(1, 0, 90), 0, 1)       # facing East
        self.assertNE(body_to_ned(1, 0, 180), -1, 0)     # facing South: forward is South
        self.assertNE(body_to_ned(1, 0, 270), 0, -1)     # facing West

    def test_right_is_clockwise_of_forward(self):
        self.assertNE(body_to_ned(0, 1, 0), 0, 1)        # facing North: right is East
        self.assertNE(body_to_ned(0, 1, 90), -1, 0)      # facing East: right is South
        self.assertNE(body_to_ned(0, 1, 180), 0, -1)

    def test_length_is_preserved(self):
        n, e = body_to_ned(1.0, 1.0, 153.0)
        self.assertAlmostEqual(math.hypot(n, e), math.sqrt(2), places=6)

    def test_offset_and_inversion(self):
        self.assertNE(body_to_ned(1, 0, 0, yaw_offset_deg=180), -1, 0)
        self.assertNE(body_to_ned(1, 0, 0, invert_forward=True), -1, 0)
        self.assertNE(body_to_ned(0, 1, 0, invert_right=True), 0, -1)


class SettingsTests(unittest.TestCase):
    def test_defaults_and_validation(self):
        from core.settings import LimitsConfig
        lim = LimitsConfig()
        self.assertEqual((lim.move_frame, lim.move_yaw_offset_deg), ("body", 0.0))
        lim.validate()
        for bad in ({"move_frame": "world"}, {"move_yaw_offset_deg": 270.0}):
            with self.assertRaises(ValueError):
                LimitsConfig(**bad).validate()


class DispatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        cls.win = drone_gcs.DroneGCSMainWindow(settings=apply_overrides(load_settings()))
        cls.win.worker = mock.Mock()
        cls.win.worker.isRunning.return_value = True

    @classmethod
    def tearDownClass(cls):
        cls.win.worker = None
        cls.win.close()

    def setUp(self):
        t = self.win.last_telemetry
        t.connected, t.armed, t.flight_mode, t.landed_state, t.heading = True, True, "OFFBOARD", 2, 90.0
        self.win.worker.reset_mock()
        self.win.settings.limits.move_frame = "body"
        self.win.settings.limits.move_yaw_offset_deg = 0.0

    def sent(self):
        return [round(v, 3) for v in self.win.worker.move_delta.call_args[0]]

    def test_body_move_is_forward_of_the_drone(self):
        self.win._cmd_move(1, 0, 0)                       # facing East
        self.assertEqual(self.sent(), [0.0, 1.0, 0.0])

    def test_typed_commands(self):
        self.win._execute_cli_command("move 1 0 0")       # facing East, body frame
        self.assertEqual(self.sent(), [0.0, 1.0, 0.0])
        self.win._execute_cli_command("moven 1 0 0")
        self.assertEqual(self.sent(), [1.0, 0.0, 0.0])
        self.win._execute_cli_command("moveb 0 1 0")      # right of an East-facing drone = South
        self.assertEqual(self.sent(), [-1.0, 0.0, 0.0])

    def test_moven_is_world_north(self):
        self.win._cmd_move(1, 0, 0, frame="ned")
        self.assertEqual(self.sent(), [1.0, 0.0, 0.0])

    def test_configured_ned_frame(self):
        self.win.settings.limits.move_frame = "ned"
        self.win._cmd_move(1, 1, 0)
        self.assertEqual(self.sent(), [1.0, 1.0, 0.0])

    def test_down_passes_through_and_limit_still_applies(self):
        self.win._cmd_move(0, 0, -1)
        self.assertEqual(self.sent(), [0.0, 0.0, -1.0])
        self.win.worker.reset_mock()
        self.win._cmd_move(5, 0, 0)
        self.win.worker.move_delta.assert_not_called()


if __name__ == "__main__":
    unittest.main()
