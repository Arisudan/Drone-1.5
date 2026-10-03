"""Tactical SLAM tab: side panel, state-aware path actions, quiet quality strip."""

import unittest
from types import SimpleNamespace

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from PyQt5.QtWidgets import QApplication

from ui.slam_map_widget import SLAMMapWidget


class PanelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        from ui.styles import build_stylesheet
        cls.app.setStyleSheet(build_stylesheet())

    def setUp(self):
        self.w = SLAMMapWidget()
        self.w.resize(1250, 760)
        self.w.show()
        self.app.processEvents()

    def tearDown(self):
        self.w.close()
        self.w.deleteLater()

    def vis(self, widget):
        return widget.isVisibleTo(self.w)

    def test_map_state_layers_and_tools_live_in_the_side_panel(self):
        for name in ("pill_status", "btn_reset_map", "btn_layer_raw", "btn_layer_thin", "btn_layer_both",
                     "btn_inflation", "btn_keepout", "btn_ruler", "btn_execute_path", "btn_pause_path",
                     "btn_abort_path", "btn_clear_goal", "lbl_path_info", "mission_panel"):
            self.assertTrue(getattr(self.w, name).parent() is not None, name)
            self.assertTrue(self.w.side_panel.isAncestorOf(getattr(self.w, name)), name)

    def test_cruise_altitude_is_labelled_as_such(self):
        texts = [lbl.text() for lbl in self.w.findChildren(type(self.w.lbl_map_caption))]
        self.assertIn("CRUISE ALT", texts)
        self.assertNotIn("MISSION", [t for t in texts if t == "MISSION"])

    def test_idle_shows_execute_only(self):
        self.assertTrue(self.vis(self.w.btn_execute_path))
        self.assertFalse(self.w.btn_execute_path.isEnabled())
        self.assertFalse(self.vis(self.w.btn_pause_path))
        self.assertFalse(self.vis(self.w.btn_abort_path))

    def test_flying_shows_pause_and_abort_instead(self):
        self.w.set_executing_state(True)
        self.assertFalse(self.vis(self.w.btn_execute_path))
        self.assertTrue(self.vis(self.w.btn_pause_path))
        self.assertTrue(self.vis(self.w.btn_abort_path))
        self.assertTrue(self.w.btn_abort_path.isEnabled())
        self.assertEqual(self.w.btn_pause_path.text(), "PAUSE")
        self.assertEqual(self.w.btn_abort_path.objectName(), "btnAbort")

    def test_paused_offers_resume_and_still_abort(self):
        self.w.set_executing_state(False, paused=True)
        self.assertEqual(self.w.btn_pause_path.text(), "RESUME")
        self.assertTrue(self.vis(self.w.btn_abort_path))
        self.assertFalse(self.vis(self.w.btn_execute_path))

    def test_finishing_returns_to_idle(self):
        self.w.set_executing_state(True)
        self.w.set_executing_state(False)
        self.assertTrue(self.vis(self.w.btn_execute_path))
        self.assertFalse(self.vis(self.w.btn_abort_path))

    def test_action_buttons_are_large(self):
        for b in (self.w.btn_execute_path, self.w.btn_pause_path, self.w.btn_abort_path):
            self.assertGreaterEqual(b.minimumHeight(), 36)

    def test_route_summary_replaces_the_idle_hint(self):
        self.assertTrue(self.vis(self.w.lbl_route_idle))
        self.assertFalse(self.vis(self.w.lbl_path_info))
        self.w._on_goal_staged(1.0, 2.0, [(0, 0)] * 4, 3.5, 14.0)
        self.assertFalse(self.vis(self.w.lbl_route_idle))
        text = self.w.lbl_path_info.text()
        self.assertIn("GOAL", text)
        self.assertIn("3.50 m", text)
        self.assertIn("~14 s", text)
        self.assertTrue(self.w.btn_execute_path.isEnabled())
        self.w._on_goal_cleared()
        self.assertTrue(self.vis(self.w.lbl_route_idle))
        self.assertFalse(self.w.btn_execute_path.isEnabled())

    def test_blocked_goal_cannot_be_executed(self):
        self.w._on_goal_staged(1.0, 2.0, [], 0.0, 0.0)
        self.assertIn("BLOCKED", self.w.lbl_path_info.text())
        self.assertFalse(self.w.btn_execute_path.isEnabled())

    def test_layers_hide_in_the_3d_view(self):
        self.assertTrue(self.vis(self.w.layers_section))
        self.w._set_view_mode(1)
        self.assertFalse(self.vis(self.w.layers_section))
        self.assertTrue(self.vis(self.w.btn_reset_map))
        self.w._set_view_mode(0)
        self.assertTrue(self.vis(self.w.layers_section))

    def test_panel_width_does_not_change_when_the_stop_list_opens(self):
        before = (self.w.side_panel.width(), self.w.canvas.width())
        self.w.canvas.mission_stops = [(1, 1), (2, 2)]
        self.w._on_mission_changed(self.w.canvas.mission_stops, [])
        self.app.processEvents()
        self.assertEqual((self.w.side_panel.width(), self.w.canvas.width()), before)

    def test_quality_strip_is_grey_unless_a_threshold_fails(self):
        def metric(key, good):
            return SimpleNamespace(key=key, label=key, text="1.0", detail="", good=good)

        ms = [metric("coverage", True), metric("explored", None), metric("wall_rms", False)]
        q = SimpleNamespace(metrics=ms, by_key=lambda: {m.key: m for m in ms})
        self.w._apply_quality(q)
        cells = self.w._quality_cells
        self.assertEqual(cells["coverage"].styleSheet(), "")      # passing: neutral, not green
        self.assertEqual(cells["explored"].styleSheet(), "")
        self.assertIn("color", cells["wall_rms"].styleSheet())   # failing: caution colour
        self.assertIn("Walls".lower(), self.w.lbl_quality_note.text().lower() + "walls")
        self.assertEqual(cells["coverage"].objectName(), "qualValue")

    # ── second pass: calmer colour, one toolbar row, status strip, compact list

    def test_cruise_altitude_sits_in_the_panel_above_execute(self):
        self.assertTrue(self.w.side_panel.isAncestorOf(self.w.combo_alt))
        self.assertLess(self.w.combo_alt.mapTo(self.w, self.w.combo_alt.rect().topLeft()).y(),
                        self.w.btn_execute_path.mapTo(self.w, self.w.btn_execute_path.rect().topLeft()).y())

    def test_toolbar_is_one_row(self):
        top = self.w.btn_view_2d.mapTo(self.w, self.w.btn_view_2d.rect().center()).y()
        for b in (self.w.btn_zoom_in, self.w.btn_fit_map, self.w.btn_center):
            y = b.mapTo(self.w, b.rect().center()).y()
            self.assertLess(abs(y - top), 8, b.text())

    def test_no_data_is_grey_until_a_map_has_been_seen_then_red_when_lost(self):
        self.assertEqual(self.w.pill_status.text(), "NO DATA")
        self.assertEqual(self.w.pill_status.property("state"), "idle")
        self.w._has_raw_map = self.w._has_thin_map = True
        self.w._update_status_pill()
        self.assertEqual(self.w.pill_status.property("state"), "ok")
        self.w._has_raw_map = self.w._has_thin_map = False
        self.w._update_status_pill()
        self.assertEqual(self.w.pill_status.text(), "MAP LOST")
        self.assertEqual(self.w.pill_status.property("state"), "bad")
        self.w.clear_local_map_display()          # a deliberate reset is not a loss
        self.w._update_status_pill()
        self.assertEqual(self.w.pill_status.property("state"), "idle")

    def test_reset_map_and_latched_tools_are_not_alarm_coloured(self):
        qss = self.app.styleSheet()
        danger = qss[qss.index("QPushButton#mapDanger {"):].split("}")[0]
        self.assertNotIn("#f85149", danger)
        latched = qss[qss.index("QPushButton#mapTool:checked"):].split("}")[0]
        self.assertNotIn("#d29922", latched)

    def test_pose_and_uncertainty_live_in_a_strip_not_on_the_map(self):
        self.w.canvas.drone_x, self.w.canvas.drone_y = 1.5, -2.0
        self.w._refresh_status_strip()
        self.assertIn("N +1.50", self.w.lbl_pose.text())
        self.assertIn("E -2.00", self.w.lbl_pose.text())
        self.assertIn("POS", self.w.lbl_uncert.text())
        self.assertTrue(self.vis(self.w.status_strip))
        self.assertGreater(self.w.status_strip.mapTo(self.w, self.w.status_strip.rect().topLeft()).y(),
                           self.w.canvas.mapTo(self.w, self.w.canvas.rect().bottomLeft()).y() - 2)
        self.w._set_view_mode(1)
        self.assertFalse(self.vis(self.w.status_strip))

    def test_stop_list_height_follows_the_number_of_stops(self):
        def heights(n):
            self.w.canvas.mission_stops = [(float(i), 0.0) for i in range(n)]
            self.w._on_mission_changed(self.w.canvas.mission_stops, [])
            return self.w.list_mission.height()
        two, four, nine = heights(2), heights(4), heights(9)
        self.assertLess(two, four)
        self.assertEqual(nine, heights(6))        # capped at six rows, then it scrolls

    # ── third pass: narrower panel and the collapse button

    def test_panel_is_narrower_than_before(self):
        self.assertLessEqual(self.w.side_panel.width(), 215)

    def test_stop_rows_are_short(self):
        self.w.canvas.mission_stops = [(1.0, 1.0), (2.0, -1.0)]
        self.w._on_mission_changed(self.w.canvas.mission_stops, [{"distance_m": 1.4}])
        self.assertEqual(self.w.list_mission.item(0).text(), "1. N+1.0 E+1.0")

    def test_collapse_keeps_only_the_path_actions_and_gives_the_map_the_width(self):
        before = self.w.canvas.width()
        self.w.set_panel_collapsed(True)
        self.app.processEvents()
        self.assertTrue(self.w.panel_collapsed())
        self.assertLess(self.w.side_panel.width(), 110)
        self.assertGreater(self.w.canvas.width(), before + 80)
        for hidden in (self.w.btn_reset_map, self.w.btn_inflation, self.w.combo_alt, self.w.pill_status):
            self.assertFalse(self.vis(hidden))
        self.assertTrue(self.vis(self.w.btn_execute_path))
        self.assertTrue(self.vis(self.w.btn_collapse_panel))
        self.assertEqual(self.w.btn_collapse_panel.text(), "\u2039")

    def test_collapsed_panel_still_shows_pause_and_abort_stacked_when_flying(self):
        self.w.set_panel_collapsed(True)
        self.w.set_executing_state(True)
        self.app.processEvents()
        self.assertTrue(self.vis(self.w.btn_pause_path))
        self.assertTrue(self.vis(self.w.btn_abort_path))
        self.assertFalse(self.vis(self.w.btn_execute_path))
        self.assertGreater(self.w.btn_abort_path.y(), self.w.btn_pause_path.y())     # stacked
        self.assertLessEqual(self.w.btn_abort_path.geometry().right(), self.w.side_panel.width())

    def test_expanding_again_restores_everything(self):
        self.w.set_panel_collapsed(True)
        self.w.set_panel_collapsed(False)
        self.app.processEvents()
        self.assertFalse(self.w.panel_collapsed())
        self.assertTrue(self.vis(self.w.btn_reset_map))
        self.assertTrue(self.vis(self.w.combo_alt))
        self.assertTrue(self.vis(self.w.btn_clear_goal))


if __name__ == "__main__":
    unittest.main()
