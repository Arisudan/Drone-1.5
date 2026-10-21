"""GCS command arbitration wired into the real main window (controllers/arbitration.py).

A recording fake worker stands in for MAVLink; telemetry snapshots are fed through the window's
own _on_telemetry_updated. Covers the four jobs end to end plus the CHECK ORIGIN acknowledgement.
Not hermetic - needs PyQt5 (skipped otherwise), like test_gui_layout.py.
"""
import os
import time
import unittest

import _env  # noqa: F401

try:
    from PyQt5.QtWidgets import QApplication
    HAVE_QT = True
except Exception:
    HAVE_QT = False


class FakeWorker:
    def __init__(self):
        self.calls = []
        self._connected = True

    def isRunning(self):
        return True

    def __getattr__(self, name):          # takeoff, takeoff_offboard, set_mode, move_delta, ...
        def rec(*a, **k):
            self.calls.append((name, a, k))
        return rec

    def names(self):
        return [c[0] for c in self.calls]


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class ArbitrationWindowTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_arb_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        cls.drone_gcs = drone_gcs

    def setUp(self):
        from core.settings import GCSSettings
        self.win = self.drone_gcs.DroneGCSMainWindow(settings=GCSSettings())
        self.w = FakeWorker()
        self.win.worker = self.w

    def tearDown(self):
        self.win.worker = None
        self.win.shutdown_workers()
        self.win.close()

    def snap(self, **kw):
        from core.arbiter import EST_REQUIRED
        from core.telemetry import TelemetrySnapshot
        now = time.time()
        t = TelemetrySnapshot()
        t.connected, t.armed, t.flight_mode, t.landed_state = True, True, "POSCTL", 1
        t.est_flags, t.est_pos_ratio, t.last_est_time = EST_REQUIRED, 0.02, now
        t.last_vision_time, t.position_age, t.last_position_time = now, 0.05, now
        t.x, t.y, t.z, t.heading = 0.1, -0.2, -0.05, 12.0
        t.rc_raw = (1500, 1500, 1003, 1500, 1011, 1011, 1011, 1011)
        for k, v in kw.items():
            setattr(t, k, v)
        return t

    def feed(self, t):
        self.win._on_telemetry_updated(t)
        self.app.processEvents()

    def wait(self, s):
        end = time.time() + s
        while time.time() < end:
            self.app.processEvents()
            time.sleep(0.02)

    # 1. takeoff by frame
    def test_takeoff_with_origin_is_nav_takeoff_keeping_position_and_heading(self):
        self.feed(self.snap(alt_amsl_m=10.27, last_global_time=time.time()))
        self.win._cmd_takeoff(1.0)
        name, a, k = [c for c in self.w.calls if c[0] == "takeoff"][0]
        self.assertAlmostEqual(k["target_amsl"], 11.27)
        self.assertNotIn("takeoff_offboard", self.w.names())

    def test_takeoff_without_origin_is_an_offboard_climb_in_place(self):
        self.feed(self.snap())                                   # no GLOBAL_POSITION_INT
        self.win._cmd_takeoff(1.0)
        name, a, k = [c for c in self.w.calls if c[0] == "takeoff_offboard"][0]
        self.assertEqual(a[:2], (0.1, -0.2))
        self.assertAlmostEqual(a[2], -1.05)
        self.assertEqual(a[3], 12.0)                             # current heading, not 0
        self.assertNotIn("takeoff", self.w.names())
        self.wait(1.3)
        self.assertIn(("set_mode", ("OFFBOARD",), {}), self.w.calls)

    def test_takeoff_above_the_ceiling_is_rejected(self):
        self.feed(self.snap(alt_amsl_m=10.0, last_global_time=time.time()))
        self.win._cmd_takeoff(2.5)
        self.assertNotIn("takeoff", self.w.names())

    # 2. estimator gate
    def test_bad_estimator_blocks_takeoff_and_says_why(self):
        self.feed(self.snap(est_flags=1))                        # attitude only, no position
        self.win._cmd_takeoff(1.0)
        self.assertFalse({"takeoff", "takeoff_offboard"} & set(self.w.names()))
        self.assertEqual(self.win.chip_gcs_ctrl.text(), "GCS CTRL: BLOCKED")
        self.assertIn("horizontal position", self.win.chip_gcs_ctrl.toolTip())

    def test_estimator_loss_mid_command_lands_without_an_origin(self):
        self.feed(self.snap(flight_mode="OFFBOARD", landed_state=2))
        self.win._arb_engage()
        self.feed(self.snap(flight_mode="OFFBOARD", landed_state=2, est_flags=1))
        self.wait(0.6)
        self.feed(self.snap(flight_mode="OFFBOARD", landed_state=2, est_flags=1))
        self.assertIn("stop_setpoints", self.w.names())
        self.assertIn(("set_mode", ("AUTO.LAND",), {}), self.w.calls)

    def test_estimator_loss_with_an_origin_holds(self):
        kw = dict(flight_mode="OFFBOARD", landed_state=2, alt_amsl_m=10.0)
        self.feed(self.snap(last_global_time=time.time(), **kw))
        self.win._arb_engage()
        self.feed(self.snap(last_global_time=time.time(), est_flags=1, **kw))
        self.wait(0.6)
        self.feed(self.snap(last_global_time=time.time(), est_flags=1, **kw))
        self.assertIn(("set_mode", ("AUTO.LOITER",), {}), self.w.calls)

    # 3. pilot wins
    def test_stick_movement_hands_control_to_the_pilot_and_locks_the_gcs_out(self):
        self.feed(self.snap(flight_mode="OFFBOARD", landed_state=2))
        self.win._arb_engage()
        self.feed(self.snap(flight_mode="OFFBOARD", landed_state=2,
                            rc_raw=(1720, 1500, 1003, 1500, 1011, 1011, 1011, 1011)))
        self.assertIn("stop_setpoints", self.w.names())
        self.assertIn(("set_mode", ("POSCTL",), {}), self.w.calls)
        self.assertEqual(self.win.chip_gcs_ctrl.text(), "GCS CTRL: PILOT")
        self.w.calls.clear()
        self.assertFalse(self.win._arb_allow("move"))            # locked out ...
        self.win._arb_chip_clicked()                             # ... until the operator says so
        self.assertTrue(self.win._arb_allow("move"))

    def test_centred_sticks_do_not_interrupt(self):
        self.feed(self.snap(flight_mode="OFFBOARD", landed_state=2))
        self.win._arb_engage()
        self.feed(self.snap(flight_mode="OFFBOARD", landed_state=2,
                            rc_raw=(1530, 1480, 1003, 1510, 1011, 1011, 1011, 1011)))
        self.assertNotIn("stop_setpoints", self.w.names())

    # 4. motion limits
    def test_move_out_of_the_box_is_rejected(self):
        self.feed(self.snap(landed_state=1, x=0.0, y=0.0))       # arms here: home = (0, 0)
        self.feed(self.snap(flight_mode="OFFBOARD", landed_state=2, x=4.5, y=0.0, z=-1.0))
        self.win._cmd_move(1.5, 0.0, 0.0, frame="ned")           # to N 6.0: outside +/-5 m
        self.assertNotIn("move_delta", self.w.names())
        self.win._cmd_move(0.4, 0.0, 0.0, frame="ned")
        self.assertIn("move_delta", self.w.names())

    # CHECK ORIGIN acknowledgement
    def test_check_origin_reports_the_px4_answer(self):
        self.feed(self.snap())
        self.win._check_origin()
        self.assertIn("request_global_origin", self.w.names())
        t = self.snap(origin_lat=12.9915, origin_lon=80.2337, origin_alt=10.0,
                      origin_query_time=time.time() - 0.2, last_origin_time=time.time())
        self.feed(t)
        self.win._report_origin()
        self.assertEqual(self.win.btn_check_origin.text(), "ORIGIN: OK")
        self.assertIn("12.991500", self.win.btn_check_origin.toolTip())

    def test_check_origin_silence_means_none(self):
        self.feed(self.snap(origin_query_time=time.time() - 5.0))
        self.win._report_origin()
        self.assertEqual(self.win.btn_check_origin.text(), "ORIGIN: NONE")


if __name__ == "__main__":
    unittest.main(verbosity=2)
