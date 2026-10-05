"""Preflight checklist, ARM gate, save-to-flash, parameter export, station and Radxa alarms."""

import os
import tempfile
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
from unittest import mock

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "radxa"))

from core import preflight as P
from core import radxa_status as R


def good(**kw):
    d = dict(connected=True, heartbeat_age_s=0.2, armed=False, battery_pct=90, vision_ok=True,
             position_stale=False, video_state="LIVE", map_seen=True, map_stale=False,
             params_loaded=True, radxa_state="ok", heading_confirmed=True)
    d.update(kw)
    return P.PreflightInputs(**d)


class ChecklistLogicTest(unittest.TestCase):
    def by_key(self, **kw):
        return {c.key: c for c in P.evaluate(good(**kw))}

    def test_everything_good_is_ready(self):
        checks = P.evaluate(good())
        self.assertEqual(P.blockers(checks), [])
        ready, text = P.summary(checks)
        self.assertTrue(ready)
        self.assertEqual(text, "READY TO ARM")

    def test_each_required_fact_blocks_when_wrong(self):
        cases = {"link": dict(connected=False), "battery": dict(battery_pct=30),
                 "vision": dict(vision_ok=False), "position": dict(position_stale=True),
                 "video": dict(video_state="FROZEN"), "map": dict(map_seen=False),
                 "heading": dict(heading_confirmed=False)}
        for key, kw in cases.items():
            blocked = {c.key for c in P.blockers(P.evaluate(good(**kw)))}
            self.assertIn(key, blocked, key)

    def test_battery_thresholds_follow_the_alert_settings(self):
        self.assertTrue(self.by_key(battery_pct=36)["battery"].ok)
        self.assertEqual(self.by_key(battery_pct=35)["battery"].status, P.FAIL)
        self.assertIn("critical", self.by_key(battery_pct=15)["battery"].detail)
        self.assertEqual(self.by_key(battery_pct=0)["battery"].status, P.UNKNOWN)      # not reporting != full

    def test_a_stale_heartbeat_is_a_dead_link(self):
        self.assertFalse(self.by_key(heartbeat_age_s=5.0)["link"].ok)

    def test_no_link_makes_vehicle_facts_unknown_not_pass(self):
        c = self.by_key(connected=False)
        for k in ("battery", "vision", "position"):
            self.assertEqual(c[k].status, P.UNKNOWN, k)

    def test_unknown_never_counts_as_pass(self):
        self.assertFalse(self.by_key(heading_confirmed=False)["heading"].ok)
        self.assertIn("heading", {c.key for c in P.blockers(P.evaluate(good(heading_confirmed=False)))})

    def test_a_stale_or_missing_map_blocks(self):
        self.assertIn("stopped", self.by_key(map_stale=True)["map"].detail)
        self.assertEqual(self.by_key(map_stale=True)["map"].status, P.FAIL)

    def test_information_rows_never_block(self):
        checks = P.evaluate(good(radxa_state=None, params_loaded=False))
        self.assertEqual(P.blockers(checks), [])
        r = {c.key: c for c in checks}
        self.assertEqual(r["radxa"].status, P.UNKNOWN)
        self.assertFalse(r["radxa"].required)
        self.assertFalse(r["params"].required)
        down = {c.key: c for c in P.evaluate(good(radxa_state="down", radxa_detail="pipeline not running"))}
        self.assertEqual(down["radxa"].status, P.FAIL)
        self.assertEqual(P.blockers(list(down.values())), [])

    def test_summary_names_what_to_fix(self):
        ready, text = P.summary(P.evaluate(good(vision_ok=False, video_state="IDLE")))
        self.assertFalse(ready)
        self.assertIn("NOT READY - 2 to fix", text)
        self.assertIn("Vision tracking", text)
        _, many = P.summary(P.evaluate(P.PreflightInputs()))
        self.assertIn("more", many)
        self.assertTrue(P.summary(P.evaluate(P.PreflightInputs()), armed=True)[0])


class RadxaStatusTest(unittest.TestCase):
    SAMPLE = {"state": "degraded", "gave_up": False, "pipeline": {"active": True}, "video": {"ok": False},
              "router": {"active": True}, "disk": {"free_gb": 12.0, "free_pct": 8.0},
              "restarts": [{"time": 1000.0, "reason": "video not served for 20 s"}],
              "messages": ["pipeline is starting"], "time": 1005.0}

    def test_parse_and_helpers(self):
        st = R.parse_status(self.SAMPLE)
        self.assertEqual(st.state, "degraded")
        self.assertTrue(st.disk_low())
        self.assertEqual(st.recent_restart(1100.0)["reason"], "video not served for 20 s")
        self.assertIsNone(st.recent_restart(1000.0 + R.RECENT_RESTART_S + 1))
        self.assertIn("video not being served", st.detail())

    def test_used_pct_is_accepted_and_garbage_is_rejected(self):
        st = R.parse_status({"state": "ok", "disk": {"used_pct": 44}})
        self.assertAlmostEqual(st.disk_free_pct, 56.0)
        self.assertEqual(st.detail(), "")
        for junk in (None, [], {}, {"nope": 1}, "x"):
            self.assertIsNone(R.parse_status(junk))

    def test_end_to_end_against_the_real_watchdog_endpoint(self):
        import radxa_watchdog as W
        wd = W.Watchdog(lambda: 5000.0)
        wd.tick(pipeline_state="active", video_ok=True, router_active=True, disk_free_gb=100.0, disk_free_pct=40.0)
        srv = ThreadingHTTPServer(("127.0.0.1", 0), W.make_handler(lambda: dict(wd.status)))
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            st = R.fetch_status("127.0.0.1", srv.server_address[1])
            self.assertEqual(st.state, "ok")
            self.assertTrue(st.pipeline_active and st.video_ok and st.router_active)
            self.assertEqual(st.disk_free_gb, 100.0)
        finally:
            srv.shutdown()
            srv.server_close()

    def test_unreachable_or_not_installed_is_none_not_healthy(self):
        import socket
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        self.assertIsNone(R.fetch_status("127.0.0.1", port, timeout=0.5))


class ParamsExportTest(unittest.TestCase):
    def test_file_layout_matches_the_project_params_files(self):
        from ui.params_tab import params_file_text
        txt = params_file_text({"BAT1_CAPACITY": (5200.0, 9, 3), "ADC_ADS1115_EN": (0.0, 6, 0)}, stamp="T")
        lines = txt.splitlines()
        self.assertTrue(lines[0].startswith("# Onboard parameters"))
        self.assertEqual(lines[-2], "1\t1\tADC_ADS1115_EN\t0\t6")          # sorted, int type has no .0
        self.assertEqual(lines[-1], "1\t1\tBAT1_CAPACITY\t5200\t9")        # %g keeps 5200, not 5200.0
        self.assertTrue(txt.endswith("\n"))


class UiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])
        from ui.styles import build_stylesheet
        cls.app.setStyleSheet(build_stylesheet())

    def test_params_tab_has_the_new_buttons_and_the_checklist_under_the_table(self):
        from ui.params_tab import ParamsTabWidget
        w = ParamsTabWidget()
        w.resize(1100, 700)
        w.show()
        self.app.processEvents()
        seen = []
        w.save_flash_requested.connect(lambda: seen.append(1))
        w.btn_save_flash.click()
        self.assertEqual(seen, [1])
        self.assertGreater(w.checklist.mapTo(w, w.checklist.rect().topLeft()).y(),
                           w.table.mapTo(w, w.table.rect().bottomLeft()).y() - 2)
        self.assertEqual(w.checklist.rule.objectName(), "hDivider")
        w.close()
        w.deleteLater()

    def test_export_writes_a_file_and_refuses_when_empty(self):
        from ui.params_tab import ParamsTabWidget
        w = ParamsTabWidget()
        w.on_param_value("MC_ROLL_P", 6.5, 9, 0, 1)
        w._flush()
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "p.params")
            self.assertEqual(w.export_params(path), 1)
            with open(path, encoding="utf-8") as fh:
                self.assertIn("1\t1\tMC_ROLL_P\t6.5\t9", fh.read())
        empty = ParamsTabWidget()
        empty._on_export()
        self.assertIn("Nothing to export", empty.lbl_status.text())
        for x in (w, empty):
            x.deleteLater()

    def test_checklist_panel_draws_rows_and_the_heading_box_signals(self):
        from ui.preflight_panel import ChecklistPanel
        p = ChecklistPanel()
        p.show()
        checks = P.evaluate(good(vision_ok=False, heading_confirmed=False))
        ready, text = P.summary(checks)
        p.update_checks(checks, ready, text)
        self.assertIn("✕", p._rows["vision"].text())
        self.assertIn("✓", p._rows["link"].text())
        self.assertIn("NOT READY", p.lbl_summary.text())
        got = []
        p.heading_toggled.connect(got.append)
        p.chk_heading.setChecked(True)
        self.assertTrue(p.heading_confirmed())
        p.clear_heading()
        self.assertEqual(got, [True, False])
        p.close()
        p.deleteLater()

    def test_the_worker_sends_preflight_storage_param1_1_and_only_when_connected(self):
        from protocol.mavlink_worker import MAVLinkWorker
        w = MAVLinkWorker()
        self.assertFalse(w.save_params_to_flash())
        w._connected = True
        w.master = mock.Mock()
        self.assertTrue(w.save_params_to_flash())
        args = w.master.mav.command_long_send.call_args[0]
        self.assertEqual(args[2], 245)                  # MAV_CMD_PREFLIGHT_STORAGE
        self.assertEqual(args[4], 1)                    # param1 = 1: write all parameters
        self.assertEqual(sum(args[5:]), 0)              # everything else zero

    def test_new_settings_have_safe_defaults_and_validate(self):
        from core.settings import GCSSettings
        s = GCSSettings()
        self.assertTrue(s.limits.require_preflight)
        self.assertEqual(s.connection.watchdog_port, 8081)
        s.connection.watchdog_port = 70000
        with self.assertRaises(ValueError):
            s.validate()


class WindowTest(unittest.TestCase):
    """The assembled main window (as in test_float_position)."""

    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_preflight_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        from ui.styles import build_stylesheet
        cls.app.setStyleSheet(build_stylesheet())
        cls.win = drone_gcs.DroneGCSMainWindow(settings=apply_overrides(load_settings()))
        cls.win.resize(1400, 850)
        cls.win.show()
        for _ in range(6):
            cls.app.processEvents()

    @classmethod
    def tearDownClass(cls):
        cls.win.shutdown_workers()
        cls.win.close()

    def setUp(self):
        w = self.win
        t = w.last_telemetry
        t.connected, t.heartbeat_age, t.armed = True, 0.1, False
        t.battery_percent, t.d435i_vio_health, t.ekf2_vision_fused, t.position_stale = 90, True, True, False
        w._video_state = "LIVE"
        w.page_slam.canvas.last_map_time = time.time()
        w.page_params.checklist.chk_heading.setChecked(True)
        w.settings.limits.require_preflight = True
        w._preflight_vision_was_ok = True
        w.alarms = type(w.alarms)()
        w.confirm_bar.request = mock.Mock()
        w.toast.show_message = mock.Mock()

    def test_a_ready_vehicle_may_arm_and_reaches_the_confirm_gate(self):
        w = self.win
        self.assertTrue(w._preflight_allows_arm())
        with mock.patch.object(w, "_require_link", return_value=True):
            w._request_arm()
        self.assertEqual(w.confirm_bar.request.call_args[0][0], "arm")

    def test_a_failed_required_check_blocks_arm_and_says_why(self):
        w = self.win
        w._video_state = "FROZEN"
        with mock.patch.object(w, "_require_link", return_value=True):
            w._request_arm()
        w.confirm_bar.request.assert_not_called()
        self.assertIn("Camera feed", w.toast.show_message.call_args[0][0])

    def test_unticked_heading_blocks_arm(self):
        w = self.win
        w.page_params.checklist.chk_heading.setChecked(False)
        self.assertFalse(w._preflight_allows_arm())

    def test_the_gate_can_be_switched_off_in_settings(self):
        w = self.win
        w._video_state = "FROZEN"
        w.settings.limits.require_preflight = False
        self.assertTrue(w._preflight_allows_arm())

    def test_arm_force_in_the_terminal_skips_the_gate(self):
        w = self.win
        w._video_state = "FROZEN"
        w.worker = mock.Mock()
        w.worker.isRunning.return_value = True
        with mock.patch.object(w, "_preflight_allows_arm", side_effect=AssertionError("gate used")):
            w._execute_cli_command("arm force")        # what the flight terminal runs
        w.worker.arm.assert_called_once_with(force=True)
        w.worker = None

    def test_losing_vision_clears_the_heading_tick(self):
        w = self.win
        w.last_telemetry.d435i_vio_health = w.last_telemetry.ekf2_vision_fused = False
        w._refresh_preflight(force=True)
        self.assertFalse(w.page_params.checklist.heading_confirmed())

    def test_the_checklist_on_screen_matches_the_state(self):
        w = self.win
        w._refresh_preflight(force=True)
        self.assertEqual(w.page_params.checklist.lbl_summary.text(), "READY TO ARM")
        w._video_state = "NO SIGNAL"
        w._refresh_preflight(force=True)
        self.assertIn("NOT READY", w.page_params.checklist.lbl_summary.text())

    # save to flash
    def test_save_to_flash_is_refused_while_armed(self):
        w = self.win
        w.last_telemetry.armed = True
        with mock.patch.object(w, "_require_link", return_value=True):
            w._request_save_params()
        w.confirm_bar.request.assert_not_called()
        w._cmd_save_params()                # also refused at dispatch

    def test_save_to_flash_goes_through_the_slide_gate_and_then_sends(self):
        w = self.win
        with mock.patch.object(w, "_require_link", return_value=True):
            w._request_save_params()
        self.assertEqual(w.confirm_bar.request.call_args[0][0], "save_params")
        w.worker = mock.Mock()
        w.worker.save_params_to_flash.return_value = True
        w._on_guided_confirmed("save_params", 0.0)
        w.worker.save_params_to_flash.assert_called_once()
        w.worker = None

    # alarms
    def test_map_stalled_alarm_follows_the_map_clock_and_a_reset_clears_it(self):
        w = self.win
        c = w.page_slam.canvas
        c.last_map_time = time.time() - 100
        w._update_station_alarms()
        self.assertIn("map", w.alarms)
        c.last_map_time = time.time()
        w._update_station_alarms()
        self.assertNotIn("map", w.alarms)
        c.last_map_time = time.time() - 100
        w._update_station_alarms()
        w.page_slam.clear_local_map_display()
        w._update_station_alarms()
        self.assertNotIn("map", w.alarms)

    def test_no_map_yet_is_not_a_stalled_map(self):
        w = self.win
        w.page_slam.canvas.last_map_time = 0.0
        w._update_station_alarms()
        self.assertNotIn("map", w.alarms)

    def test_ui_stall_alarm_needs_a_real_freeze_and_is_held_then_released(self):
        w = self.win
        w._note_ui_stall(0.3)
        w._update_station_alarms()
        self.assertNotIn("ui", w.alarms)
        w._note_ui_stall(0.9)
        w._update_station_alarms()
        self.assertIn("ui", w.alarms)
        w._ui_stall_until = time.monotonic() - 1
        w._update_station_alarms()
        self.assertNotIn("ui", w.alarms)

    def test_radxa_alarms_from_the_watchdog_report(self):
        w = self.win
        w._on_radxa_status(R.RadxaStatus(state="down", pipeline_active=False))
        self.assertIn("radxa", w.alarms)
        w._on_radxa_status(R.RadxaStatus(state="ok", pipeline_active=True, video_ok=True, router_active=True))
        self.assertNotIn("radxa", w.alarms)
        w._on_radxa_status(R.RadxaStatus(state="ok", pipeline_active=True, video_ok=True, router_active=True,
                                         restarts=[{"time": time.time() - 30, "reason": "video not served"}]))
        self.assertIn("radxa_restart", w.alarms)
        w._on_radxa_status(R.RadxaStatus(state="ok", disk_free_pct=4.0))
        self.assertIn("radxa_disk", w.alarms)

    def test_a_stopped_pipeline_is_a_warning_not_an_emergency(self):
        from core.alarms import AlarmLevel
        w = self.win
        w._on_radxa_status(R.RadxaStatus(state="stopped"))
        self.assertIn("radxa_stopped", w.alarms)
        self.assertNotIn("radxa", w.alarms)
        self.assertEqual(w.alarms.active()[0].level, AlarmLevel.WARN)
        self.assertEqual(R.RadxaStatus(state="stopped").detail(), "pipeline is stopped")

    def test_a_radxa_that_does_not_report_raises_nothing_and_checks_unknown(self):
        w = self.win
        w._on_radxa_status(None)
        self.assertNotIn("radxa", w.alarms)
        row = {c.key: c for c in P.evaluate(w._preflight_inputs())}["radxa"]
        self.assertEqual(row.status, P.UNKNOWN)

    def test_an_old_radxa_report_is_treated_as_none(self):
        w = self.win
        w._on_radxa_status(R.RadxaStatus(state="down"))
        w._radxa_status_at = time.monotonic() - 100
        self.assertIsNone(w._radxa_status_fresh())


if __name__ == "__main__":
    unittest.main()
