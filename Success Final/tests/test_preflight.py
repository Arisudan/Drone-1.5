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
from ui.styles import PALETTE


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

    def test_params_tab_has_the_new_buttons_and_no_checklist_of_its_own(self):
        from ui.params_tab import ParamsTabWidget
        w = ParamsTabWidget()
        w.resize(1100, 700)
        w.show()
        self.app.processEvents()
        seen = []
        w.save_flash_requested.connect(lambda: seen.append(1))
        w.btn_save_flash.click()
        self.assertEqual(seen, [1])
        self.assertFalse(hasattr(w, "checklist"))       # it moved to the left rail
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
        self.assertEqual(p.row("vision").state, P.FAIL)
        self.assertEqual(p.row("vision").lbl_value.text(), "LOST")
        self.assertEqual(p.row("link").state, P.PASS)
        self.assertEqual(p.row("battery").lbl_value.text(), "90%")
        self.assertEqual(p.lbl_summary.text(), "5 of 7 ready")
        self.assertIn("NOT READY - 2 to fix", p.lbl_summary.toolTip())
        self.assertEqual(p.lbl_detail.text(), "Fix first: Vision tracking")
        self.assertTrue(p.lbl_detail.is_clickable())
        keys = []
        p.action_requested.connect(keys.append)
        p.lbl_detail.clicked.emit()
        self.assertEqual(keys, ["vision"])
        self.assertEqual(p.row("link").icon.text(), "\u2713")
        self.assertEqual(p.row("vision").icon.text(), "\u2715")
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
        self.assertFalse(hasattr(s.limits, "require_preflight"))      # there is no gate any more
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
        w.sidebar.checklist.chk_heading.setChecked(True)
        w._preflight_vision_was_ok = True
        w.alarms = type(w.alarms)()
        w.confirm_bar.request = mock.Mock()
        w.toast.show_message = mock.Mock()

    # ── the checklist only INFORMS: it never touches arming or any other command

    def worst_case(self):
        """Every line red or unknown at once: no video, no map, no vision, no heading."""
        w = self.win
        t = w.last_telemetry
        t.battery_percent, t.d435i_vio_health, t.ekf2_vision_fused, t.position_stale = 5, False, False, True
        w._video_state = "NO SIGNAL"
        w.page_slam.canvas.last_map_time = 0.0
        w.sidebar.checklist.chk_heading.setChecked(False)
        w._refresh_preflight(force=True)

    def test_arm_reaches_the_confirm_bar_even_when_every_line_is_red(self):
        w = self.win
        self.worst_case()
        self.assertIn(" of ", w.sidebar.checklist.lbl_summary.text())     # it does show the problems
        with mock.patch.object(w, "_require_link", return_value=True):
            w._request_arm()
        self.assertEqual(w.confirm_bar.request.call_args[0][0], "arm")
        w.toast.show_message.assert_not_called()                           # no "blocked" message, no warning

    def test_arm_is_identical_whether_the_checklist_is_green_or_red(self):
        w = self.win
        with mock.patch.object(w, "_require_link", return_value=True):
            w._refresh_preflight(force=True)
            w._request_arm()
            green = w.confirm_bar.request.call_args
            w.confirm_bar.request.reset_mock()
            self.worst_case()
            w._request_arm()
            red = w.confirm_bar.request.call_args
        self.assertEqual(green, red)                                       # same title, detail, text, anchor

    def test_the_confirmed_arm_command_is_sent_whatever_the_checklist_says(self):
        w = self.win
        self.worst_case()
        w.worker = mock.Mock()
        w.worker.isRunning.return_value = True
        w._on_guided_confirmed("arm", 0.0)
        w.worker.arm.assert_called_once_with(force=False)
        w.worker = None

    def test_terminal_arm_and_arm_force_are_unaffected(self):
        w = self.win
        self.worst_case()
        w.worker = mock.Mock()
        w.worker.isRunning.return_value = True
        w._execute_cli_command("arm")
        w._execute_cli_command("arm force")
        self.assertEqual([c.kwargs for c in w.worker.arm.call_args_list], [{"force": False}, {"force": True}])
        w.worker = None

    def test_takeoff_and_save_to_flash_do_not_look_at_the_checklist_either(self):
        w = self.win
        self.worst_case()
        w.worker = mock.Mock()
        w.worker.isRunning.return_value = True
        w.last_telemetry.armed = True
        w._cmd_takeoff(1.5)
        w.worker.takeoff.assert_called_once()
        w.last_telemetry.armed = False
        w._cmd_save_params()
        w.worker.save_params_to_flash.assert_called_once()
        w.worker = None

    def test_the_gate_is_gone_from_the_code_and_the_settings(self):
        w = self.win
        self.assertFalse(hasattr(w, "_preflight_allows_arm"))
        self.assertFalse(hasattr(w.settings.limits, "require_preflight"))
        from ui import config_tab
        self.assertNotIn("require_preflight", config_tab.FIELD_INFO["limits"][1])

    def test_an_old_settings_file_that_still_has_the_removed_switch_loads_fine(self):
        import json
        import tempfile
        from core.settings import load_settings
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "settings.json")
            with open(path, "w", encoding="utf-8") as fh:
                json.dump({"limits": {"require_preflight": True, "takeoff_alt_max_m": 2.5}}, fh)
            cfg = load_settings(path)
        self.assertEqual(cfg.limits.takeoff_alt_max_m, 2.5)

    def test_losing_vision_clears_the_heading_tick(self):
        w = self.win
        w.last_telemetry.d435i_vio_health = w.last_telemetry.ekf2_vision_fused = False
        w._refresh_preflight(force=True)
        self.assertFalse(w.sidebar.checklist.heading_confirmed())

    def test_the_checklist_on_screen_matches_the_state(self):
        w = self.win
        w._refresh_preflight(force=True)
        self.assertEqual(w.sidebar.checklist.lbl_summary.text(), "READY")
        self.assertEqual(w.sidebar.checklist.lbl_summary.toolTip(), "READY TO ARM")
        w._video_state = "NO SIGNAL"
        w._refresh_preflight(force=True)
        self.assertEqual(w.sidebar.checklist.lbl_summary.text(), "6 of 7 ready")
        self.assertIn("NOT READY", w.sidebar.checklist.lbl_summary.toolTip())


    # ── the checklist lives in the left rail, under "Parameters"

    def test_the_checklist_sits_in_the_left_rail_directly_under_the_parameters_entry(self):
        w = self.win
        self.app.processEvents()
        side = w.sidebar
        params_btn = side.btn_group.button(8)
        self.assertEqual(params_btn.text(), "Parameters")
        self.assertTrue(side.isAncestorOf(side.checklist))
        below = side.checklist.mapTo(side, side.checklist.rect().topLeft()).y()
        entry_bottom = params_btn.mapTo(side, params_btn.rect().bottomLeft()).y()
        self.assertGreaterEqual(below, entry_bottom)
        self.assertLess(below - entry_bottom, 30)               # right under it, not somewhere else
        self.assertEqual(side.checklist.rule.objectName(), "hDivider")
        rule_y = side.checklist.rule.mapTo(side, side.checklist.rule.rect().topLeft()).y()
        title_y = side.checklist.lbl_title.mapTo(side, side.checklist.lbl_title.rect().topLeft()).y()
        self.assertLess(rule_y, title_y)                        # the line comes before the list

    def test_the_rail_checklist_is_visible_from_every_workspace(self):
        w = self.win
        for idx in (0, 2, 8):
            w._switch_workspace(idx)
            self.app.processEvents()
            self.assertTrue(w.sidebar.checklist.isVisibleTo(w.sidebar), idx)

    def test_rail_card_shows_pill_bar_next_step_and_valued_lines(self):
        w = self.win
        w._video_state = "FROZEN"
        w._refresh_preflight(force=True)
        self.app.processEvents()
        panel = w.sidebar.checklist
        self.assertEqual(panel.lbl_summary.text(), "6 of 7 ready")
        self.assertIn("Camera feed", panel.lbl_summary.toolTip())
        self.assertEqual(panel.lbl_detail.text(), "Fix first: Camera feed")
        self.assertIn("Video is frozen.", panel.lbl_detail.toolTip())
        self.assertEqual(panel.row("video").lbl_value.text(), "FROZEN")
        self.assertEqual(panel.row("video").state, P.FAIL)
        self.assertEqual(panel.row("link").lbl_value.text(), "OK")
        self.assertAlmostEqual(panel.bar.fraction(), 6 / 7)             # 6 of the 7 required checks pass
        self.assertEqual(panel.bar.colour(), PALETTE["danger"])         # ONE colour: a required check failed
        self.assertEqual(panel.row("video").icon.text(), "\u2715")
        for key, row in panel._labels.items():
            if row.isVisible():
                self.assertLessEqual(row.sizeHint().width(), w.sidebar.width(), key)

    def test_ready_card_says_so(self):
        w = self.win
        w._refresh_preflight(force=True)
        panel = w.sidebar.checklist
        self.assertEqual(panel.lbl_summary.text(), "READY")
        self.assertEqual(panel.lbl_detail.text(), "All required checks passed.")
        self.assertEqual(panel.bar.fraction(), 1.0)
        self.assertEqual(panel.bar.colour(), PALETTE["ok"])
        self.assertFalse(panel.lbl_detail.is_clickable())

    def test_the_card_gives_up_height_in_steps_and_never_taxes_the_rail(self):
        w = self.win
        side = w.sidebar
        panel = side.checklist
        hs = [panel.height_for(lv) for lv in (0, 1, 2, 3, 4)]
        self.assertEqual(hs[0], 0)
        self.assertEqual(hs, sorted(hs))                      # each level needs more height
        self.assertEqual(len(set(hs)), 5)
        # the rail's own minimum does not depend on the card
        panel.set_level(0)
        base = side.layout().minimumSize().height()
        side._apply_checklist_level()
        self.assertLessEqual(side.layout().minimumSize().height(), side.height())
        self.assertGreaterEqual(side.layout().minimumSize().height(), base)

    def test_the_camera_thumbnail_keeps_its_space_and_the_card_only_uses_what_is_left(self):
        w = self.win
        side = w.sidebar
        w._switch_workspace(8)                                   # a tab that wants the thumbnail
        self.app.processEvents()
        panel = side.checklist
        panel.set_level(0)
        side.mini_feed.setVisible(False)
        base = side.layout().sizeHint().height()                 # buttons + footer only
        feed = side.mini_feed.wanted_height()
        original_rule = side.height() >= base + feed             # the rule before the checklist existed
        side._apply_checklist_level()
        self.assertEqual(side.mini_feed.isVisibleTo(side), original_rule)
        if side.mini_feed.isVisibleTo(side):
            self.assertGreaterEqual(side.height() - base - feed, panel.height_for(panel.level()))
        self.assertLessEqual(side.layout().minimumSize().height(), side.height())

    def test_with_little_height_the_card_shrinks_before_the_thumbnail_does(self):
        w = self.win
        side = w.sidebar
        w._switch_workspace(8)
        old = w.size()
        try:
            levels = []
            for h in (1000, 900, 820, 760):
                w.resize(old.width(), h)
                for _ in range(4):
                    self.app.processEvents()
                levels.append((h, side.checklist.level(), side.mini_feed.isVisibleTo(side)))
            lv = [x[1] for x in levels]
            self.assertEqual(lv, sorted(lv, reverse=True), levels)   # smaller window, never a bigger card
            for h, level, feed in levels:                            # card present => thumbnail present
                if level > 0:
                    self.assertTrue(feed, (h, level, feed))
        finally:
            w.resize(old)
            self.app.processEvents()

    # collapsed by default: the rail's height goes to the camera thumbnail
    def test_the_card_starts_collapsed_to_its_heading_line(self):
        w = self.win
        w._switch_workspace(8)
        self.app.processEvents()
        panel = w.sidebar.checklist
        self.assertFalse(panel.is_expanded())
        self.assertEqual(panel.level(), 1)
        self.assertTrue(panel.isVisibleTo(w.sidebar))
        self.assertTrue(panel.header.isVisibleTo(panel))
        self.assertFalse(panel.summary_block.isVisibleTo(panel))
        self.assertFalse(panel.rows.isVisibleTo(panel))
        self.assertTrue(panel.lbl_summary.text())                 # the pill still says how many to fix

    def test_clicking_the_heading_opens_and_closes_the_list(self):
        w = self.win
        w._switch_workspace(8)
        w.resize(w.width(), max(w.height(), 1000))
        self.app.processEvents()
        panel = w.sidebar.checklist
        seen = []
        panel.expanded_changed.connect(seen.append)
        try:
            panel.header.clicked.emit()
            self.app.processEvents()
            self.assertTrue(panel.is_expanded())
            self.assertGreaterEqual(panel.level(), 2)
            self.assertTrue(panel.summary_block.isVisibleTo(panel))
            panel.header.clicked.emit()
            self.app.processEvents()
            self.assertFalse(panel.is_expanded())
            self.assertEqual(panel.level(), 1)
            self.assertEqual(seen, [True, False])
        finally:
            panel.set_expanded(False)
            self.app.processEvents()

    def test_collapsed_card_is_much_shorter_than_the_open_list(self):
        panel = self.win.sidebar.checklist
        self.assertLess(panel.height_for(1) * 2, panel.height_for(4))

    def test_opening_the_list_in_a_short_window_still_shows_what_fits(self):
        # Regression: the rail measured a stale layout and concluded nothing fit,
        # so opening the list in a short window hid the whole card.
        w = self.win
        w._switch_workspace(8)
        old = w.size()
        panel = w.sidebar.checklist
        try:
            w.resize(old.width(), 768)
            for _ in range(4):
                self.app.processEvents()
            panel.set_expanded(True)
            for _ in range(4):
                self.app.processEvents()
            self.assertGreaterEqual(panel.level(), 1)
            self.assertLessEqual(w.sidebar.layout().minimumSize().height(), w.sidebar.height())
        finally:
            panel.set_expanded(False)
            w.resize(old)
            self.app.processEvents()

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
