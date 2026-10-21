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


    def test_stick_throttle_mode_while_armed_needs_a_slide(self):
        # MANUAL in the air from the GCS = throttle at the stick (usually low) = the drone falls.
        self.win.last_telemetry.armed = True
        try:
            for mode in ("MANUAL", "ACRO", "STABILIZED"):
                self.win.confirm_bar.hide()
                self.win.worker.reset_mock()
                self.win._request_mode(mode)
                self.assertTrue(self.win.confirm_bar.isVisible(), mode)
                self.win.worker.set_mode.assert_not_called()
            self.win.confirm_bar.confirmed.emit("mode_override", 0.0)
            self.win.worker.set_mode.assert_called_once_with("STABILIZED")
            self.win.confirm_bar.hide()
            self.win.worker.reset_mock()
            self.win._request_mode("AUTO.LOITER")          # HOLD stays one click
            self.win.worker.set_mode.assert_called_once_with("AUTO.LOITER")
        finally:
            self.win.last_telemetry.armed = False

    def test_stick_throttle_mode_while_disarmed_is_direct(self):
        self.win.last_telemetry.armed = False
        self.win._request_mode("MANUAL")
        self.win.worker.set_mode.assert_called_once_with("MANUAL")


class RcPriorityTests(RcModeGuardTests):
    """The RC pilot always wins over OFFBOARD."""

    def test_route_stops_when_rc_leaves_offboard(self):
        w = self.win
        self._tel("OFFBOARD")
        w.path_in_progress, w.path_paused = True, False
        w.active_waypoints, w.current_wpt_idx = [(1.0, 1.0)], 0
        self.t[0] += 5.0
        self._tel("POSCTL")
        self.assertFalse(w.path_in_progress)
        w.worker.stop_setpoints.assert_called()

    def test_gcs_commanded_exit_is_not_a_takeover(self):
        w = self.win
        self._tel("OFFBOARD")
        w.path_in_progress, w.path_paused = True, True      # paused = we sent AUTO.LOITER
        w._cmd_mode("AUTO.LOITER")
        self.t[0] += 0.5
        self._tel("AUTO.LOITER")
        self.assertTrue(w.path_in_progress)
        w.path_in_progress = False

    def test_worker_never_reenters_offboard_silently(self):
        from protocol.mavlink_worker import MAVLinkWorker
        wk = MAVLinkWorker.__new__(MAVLinkWorker)
        wk._connected, wk.master = True, object()
        wk.telemetry = mock.Mock(flight_mode="POSCTL", x=0, y=0, z=0, heading=0)
        wk.set_mode = mock.Mock()
        wk.move_delta(1, 0, 0)
        wk.rotate_yaw(10)
        wk.move_to_waypoint(1, 1)
        wk.set_mode.assert_not_called()


class OffboardEntryTests(unittest.TestCase):
    """Entering OFFBOARD must never command zero thrust to a vehicle that is in the air."""

    def _worker(self, airborne):
        from protocol.mavlink_worker import MAVLinkWorker
        wk = MAVLinkWorker.__new__(MAVLinkWorker)
        import threading
        wk._connected, wk.master = True, mock.Mock()
        wk.sp_lock = threading.Lock()
        wk.telemetry = mock.Mock(is_airborne=airborne, x=3.0, y=-2.0, z=-1.5, heading=90.0)
        wk.sent = []
        wk._send_setpoint = lambda: wk.sent.append(wk.setpoint_kind)
        wk.set_message = None
        wk.tx_count = 0
        wk.broadcast_cmd = lambda *a, **k: None
        return wk

    def _enter(self, airborne):
        wk = self._worker(airborne)
        with mock.patch("protocol.mavlink_worker.time.sleep"):
            try:
                wk.set_mode("OFFBOARD")
            except Exception:
                pass          # only the primed setpoint matters here, not the mode command that follows
        return wk

    def test_airborne_holds_position(self):
        wk = self._enter(True)
        self.assertEqual(wk.setpoint_kind, "position")
        self.assertEqual((wk.sp_x, wk.sp_y, wk.sp_z), (3.0, -2.0, -1.5))
        self.assertTrue(wk.streaming_setpoints)

    def test_on_ground_stays_idle_attitude(self):
        wk = self._enter(False)
        self.assertEqual(wk.setpoint_kind, "idle_attitude")


if __name__ == "__main__":
    unittest.main()
