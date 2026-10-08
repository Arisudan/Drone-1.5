"""Preflight card design: one summary, one bar, icon | name | status columns, "Fix first" that jumps."""
import _env  # noqa: F401  -- must be first

import os
import unittest

from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])

from core import preflight as P  # noqa: E402
from ui.preflight_panel import ACTION_TAB, ChecklistPanel  # noqa: E402
from ui.styles import PALETTE, build_stylesheet  # noqa: E402


def inputs(**kw):
    base = dict(connected=True, heartbeat_age_s=0.2, battery_pct=90, vision_ok=True, position_stale=False,
                video_state="LIVE", map_seen=True, map_stale=False, params_loaded=True, radxa_state="ok",
                heading_confirmed=True)
    base.update(kw)
    return P.PreflightInputs(**base)


class PreflightDesignTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        app.setStyleSheet(build_stylesheet())

    def panel(self, **kw):
        p = ChecklistPanel()
        p.resize(220, 400)
        p.set_level(4)
        p.show()
        checks = P.evaluate(inputs(**kw))
        ready, text = P.summary(checks)
        p.update_checks(checks, ready, text)
        for _ in range(4):
            app.processEvents()
        return p

    def test_icons_use_shape_as_well_as_colour(self):
        p = self.panel(vision_ok=False, connected=True, radxa_state="down", heading_confirmed=False)
        self.assertEqual(p.row("link").icon.text(), "✓")          # passed
        self.assertEqual(p.row("vision").icon.text(), "✕")        # required and failing
        self.assertEqual(p.row("radxa").icon.text(), "!")              # information-only and failing
        p.close()
        p = self.panel(connected=False)
        self.assertEqual(p.row("battery").icon.text(), "–")       # cannot tell yet
        p.close()

    def test_waiting_is_a_word_not_two_dashes(self):
        p = self.panel(connected=False)
        self.assertEqual(p.row("battery").lbl_value.text(), "waiting")
        self.assertNotIn("--", [r.lbl_value.text() for r in p._labels.values()])
        p.close()

    def test_one_summary_one_bar_one_colour(self):
        p = self.panel(vision_ok=False)
        self.assertEqual(p.lbl_summary.text(), "6 of 7 ready")
        self.assertAlmostEqual(p.bar.fraction(), 6 / 7)
        self.assertEqual(p.bar.colour(), PALETTE["danger"])
        p.close()
        p = self.panel()
        self.assertEqual(p.lbl_summary.text(), "READY")
        self.assertEqual(p.bar.colour(), PALETTE["ok"])
        p.close()

    def test_only_a_failed_required_check_is_red(self):
        p = self.panel(connected=False)            # everything below the link is "waiting", not failed
        reds = [k for k, r in p._labels.items() if PALETTE["danger"] in r.icon.styleSheet()]
        self.assertEqual(reds, ["link"])                 # the link really is lost; the rest merely cannot be judged
        for k in ("battery", "vision", "position"):
            self.assertNotIn(PALETTE["danger"], p.row(k).icon.styleSheet(), k)
        p.close()

    def test_columns_line_up_on_every_row(self):
        p = self.panel(vision_ok=False)
        rows = [r for r in p._labels.values() if r.isVisible()]
        self.assertGreater(len(rows), 4)
        icon_x = {r.icon.mapTo(p, r.icon.rect().topLeft()).x() for r in rows}
        right = {r.lbl_value.mapTo(p, r.lbl_value.rect().topRight()).x() for r in rows}
        widths = {r.lbl_value.width() for r in rows}
        self.assertEqual(len(icon_x), 1, icon_x)
        self.assertEqual(len(right), 1, right)
        self.assertEqual(len(widths), 1, widths)
        # the tick-box row shares the status column
        m = p.manual_row.lbl_value
        self.assertEqual(m.mapTo(p, m.rect().topRight()).x(), right.pop())
        p.close()

    def test_summary_keeps_one_footprint(self):
        p = self.panel()
        before = (p.lbl_summary.width(), p.lbl_summary.height())
        checks = P.evaluate(inputs(vision_ok=False, battery_pct=10, video_state="IDLE", map_seen=False))
        p.update_checks(checks, *P.summary(checks))
        app.processEvents()
        self.assertEqual((p.lbl_summary.width(), p.lbl_summary.height()), before)
        p.close()

    def test_fix_first_is_one_elided_line_with_the_reason_in_the_tooltip(self):
        p = self.panel(vision_ok=False)
        self.assertEqual(p.lbl_detail.text(), "Fix first: Vision tracking")
        self.assertIn("No visual-inertial odometry", p.lbl_detail.toolTip())
        self.assertEqual(p.lbl_detail.height(), p.lbl_detail.minimumHeight())
        p.close()

    def test_every_check_has_somewhere_to_jump_to(self):
        for c in P.evaluate(inputs()):
            self.assertIn(c.key, ACTION_TAB, c.key)


class FixFirstJumpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_preflight_design_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        app.setStyleSheet(build_stylesheet())
        cls.win = drone_gcs.DroneGCSMainWindow(settings=apply_overrides(load_settings()))
        cls.win.resize(1400, 900)
        cls.win.show()

    @classmethod
    def tearDownClass(cls):
        cls.win.shutdown_workers()
        cls.win.close()

    def test_clicking_fix_first_opens_the_page_that_shows_the_problem(self):
        w = self.win
        w._switch_workspace(0)
        w.sidebar.checklist.action_requested.emit("map")
        app.processEvents()
        self.assertIs(w.stack.currentWidget(), w.page_slam)
        w.sidebar.checklist.action_requested.emit("video")
        app.processEvents()
        self.assertIs(w.stack.currentWidget(), w.page_fpv)
        w._switch_workspace(0)


if __name__ == "__main__":
    unittest.main()
