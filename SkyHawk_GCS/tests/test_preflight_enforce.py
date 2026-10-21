"""Preflight "Block ARM until ready" switch: OFF = information only, ON = ARM refused until all checks pass."""
import _env  # noqa: F401  -- must be first

import unittest
from unittest import mock

from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])


class EnforceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        cls.win = drone_gcs.DroneGCSMainWindow(settings=apply_overrides(load_settings()))
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
        t = self.win.last_telemetry
        t.connected, t.armed = True, False
        self.win.worker.reset_mock()
        self.win.confirm_bar.hide()
        self.win.sidebar.checklist.chk_enforce.setChecked(False)

    def test_off_by_default_and_never_blocks(self):
        self.assertFalse(self.win.sidebar.checklist.enforce_enabled())
        self.win._request_arm()
        self.assertTrue(self.win.confirm_bar.isVisible())
        self.win._cmd_arm()
        self.win.worker.arm.assert_called_once()

    def test_on_and_not_ready_blocks_arm(self):
        self.win.sidebar.checklist.chk_enforce.setChecked(True)
        self.win._request_arm()
        self.assertFalse(self.win.confirm_bar.isVisible())
        self.win._cmd_arm()
        self.win._cmd_arm(force=True)
        self.win.worker.arm.assert_not_called()

    def test_on_and_ready_allows_arm(self):
        self.win.sidebar.checklist.chk_enforce.setChecked(True)
        with mock.patch("controllers.preflight_control.blockers", return_value=[]):
            self.win._cmd_arm()
        self.win.worker.arm.assert_called_once()

    def test_disarm_and_other_commands_never_blocked(self):
        self.win.sidebar.checklist.chk_enforce.setChecked(True)
        self.win.last_telemetry.armed = True
        self.win._cmd_disarm()
        self.win.worker.disarm.assert_called_once()
        self.win._cmd_mode("AUTO.LOITER")
        self.win.worker.set_mode.assert_called_once_with("AUTO.LOITER")


if __name__ == "__main__":
    unittest.main()
