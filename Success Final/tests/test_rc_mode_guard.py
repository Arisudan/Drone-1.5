"""Mode box follows the vehicle; SET MODE asks first if the RC changed the mode in the last 3 s."""
import _env  # noqa: F401  -- must be first

import unittest
from unittest import mock

from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])


class RcModeGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        cls.win = drone_gcs.DroneGCSMainWindow(settings=drone_gcs.apply_overrides(load_settings()))
        cls.win.resize(1400, 850)
        cls.win.show()
        cls.win.worker = mock.Mock()
        cls.win.worker.isRunning.return_value = True
        app.processEvents()

    @classmethod
    def tearDownClass(cls):
        cls.win.worker = None
        cls.win.close()

    def setUp(self):
        self.t = [1000.0]
        p = mock.patch("controllers.flight_commands.time.monotonic", lambda: self.t[0])
        p.start()
        self.addCleanup(p.stop)
        self.win._seen_mode = ""
        self.win._gcs_mode_cmd_at = -1e9
        self.win._rc_mode_change_at = -1e9
        self.win.confirm_bar.hide()
        self.win.worker.reset_mock()
        self._tel("STABILIZED")

    def _tel(self, mode):
        t = self.win.last_telemetry
        t.connected, t.flight_mode = True, mode
        self.win._on_telemetry_updated(t)

    def test_control_box_follows_vehicle(self):
        self._tel("POSCTL")
        self.assertEqual(self.win.combo_modes.currentText(), "POSCTL")

    def test_rc_change_is_noted_in_console(self):
        with mock.patch.object(self.win.console, "log_info") as li:
            self._tel("POSCTL")
        self.assertTrue(any("Mode changed by RC" in str(c) for c in li.call_args_list))

    def test_gcs_change_is_not_called_rc(self):
        self.win._cmd_mode("AUTO.LOITER")
        self.t[0] += 0.5
        with mock.patch.object(self.win.console, "log_info") as li:
            self._tel("AUTO.LOITER")
        self.assertFalse(any("Mode changed by RC" in str(c) for c in li.call_args_list))
        self.assertLess(self.win._rc_mode_change_at, 0)

    def test_guard_asks_within_3s_then_sends_on_confirm(self):
        self._tel("POSCTL")
        self.t[0] += 2.0
        self.win._request_mode("OFFBOARD")
        self.assertTrue(self.win.confirm_bar.isVisible())
        self.win.worker.set_mode.assert_not_called()
        self.win.confirm_bar.confirmed.emit("mode_override", 0.0)
        self.win.worker.set_mode.assert_called_once_with("OFFBOARD")

    def test_no_question_after_3s(self):
        self._tel("POSCTL")
        self.t[0] += 3.5
        self.win._request_mode("OFFBOARD")
        self.assertFalse(self.win.confirm_bar.isVisible())
        self.win.worker.set_mode.assert_called_once_with("OFFBOARD")

    def test_no_question_when_rc_never_changed_it(self):
        self.win._request_mode("OFFBOARD")
        self.win.worker.set_mode.assert_called_once_with("OFFBOARD")


if __name__ == "__main__":
    unittest.main()
