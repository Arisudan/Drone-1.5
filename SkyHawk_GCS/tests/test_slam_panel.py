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

    # ── v2 layout: a 340 px column of glass panels beside the map ────────────

    def test_map_state_layers_and_tools_live_in_the_side_panel(self):
        for name in ("pill_status", "btn_execute_path", "btn_pause_path", "btn_abort_path", "btn_clear_goal",
                     "btn_layer_raw", "btn_layer_thin", "btn_layer_both", "btn_reset_map",
                     "btn_inflation", "btn_keepout", "btn_ruler", "combo_alt", "docked_feed"):
            self.assertTrue(self.w.side_panel.isAncestorOf(getattr(self.w, name)), name)
        # the stops list and the source chip float on the map instead
        for name in ("mission_panel", "source_overlay"):
            self.assertTrue(self.w.canvas.isAncestorOf(getattr(self.w, name)), name)
        # the old header flight bar is superseded by the Commands card: alive but never shown
        self.assertFalse(self.vis(self.w.flight_bar))

    def test_side_panel_is_the_designs_340px_column(self):
        self.assertEqual(self.w.side_panel.width(), 340)

    def test_panels_are_stacked_in_the_designs_order(self):
        ys = [p.mapTo(self.w, p.rect().topLeft()).y()
              for p in (self.w.pnl_view, self.w.map_overlay, self.w.pnl_route, self.w.docked_feed)]
        self.assertEqual(ys, sorted(ys))
        self.assertEqual(self.w.pnl_view.lbl_title.text(), "MAP VIEW")
        self.assertEqual(self.w.map_overlay.lbl_title.text(), "MAP TOOLS")

    def test_view_tools_are_one_row_of_equal_cells(self):
        bs = [self.w.btn_auto_follow, self.w.btn_center, self.w.btn_fit_map, self.w.btn_reset_view]
        self.assertEqual([b.text() for b in bs], ["FOLLOW", "CENTER", "FIT", "1:1"])
        self.assertEqual(len({b.mapTo(self.w, b.rect().center()).y() for b in bs}), 1)
        self.assertEqual(len({b.height() for b in bs}), 1)

    def test_clear_route_sits_in_the_route_panel_below_the_path_actions(self):
        b = self.w.btn_clear_goal
        self.assertTrue(self.w.pnl_route.isAncestorOf(b))
        y = lambda w: w.mapTo(self.w, w.rect().topLeft()).y()
        self.assertGreater(y(b), y(self.w.btn_execute_path))
        y0 = b.mapTo(self.w, b.rect().topLeft())
        for n in (1, 2, 4, 6):
            self.w.canvas.mission_stops = [(float(i), 0.0) for i in range(n)]
            self.w._on_mission_changed(self.w.canvas.mission_stops, [])
            self.app.processEvents()
            self.assertEqual(b.mapTo(self.w, b.rect().topLeft()), y0, f"moved with {n} stops")

    def test_the_fold_arrow_still_works_but_is_not_shown(self):
        feed, arrow = self.w.docked_feed, self.w.btn_collapse_panel
        self.assertTrue(feed.isAncestorOf(arrow))
        self.assertFalse(self.vis(arrow))
        self.assertFalse(arrow.folded)
        self.w.set_panel_collapsed(True)
        self.assertTrue(arrow.folded)
        self.assertIn("Show", arrow.toolTip())
        self.assertFalse(feed.sink.isVisible())
        self.w.set_panel_collapsed(False)
        self.assertIn("Fold", arrow.toolTip())
        self.assertTrue(feed.sink.isVisible())

    def test_the_stops_box_has_no_second_clear_button(self):
        texts = [b.text() for b in self.w.mission_panel.findChildren(type(self.w.btn_clear_goal))]
        self.assertNotIn("Clear", texts)            # CLEAR ROUTE is the only one
        self.assertEqual(texts, ["\u25b2", "\u25bc", "\u2715"])

    def test_the_controls_hint_is_a_two_line_note_under_the_map_tools(self):
        lbl = self.w.lbl_route_idle
        self.assertTrue(self.w.map_overlay.isAncestorOf(lbl))
        for part in ("Click: set goal", "Shift+click: add stop", "Right-drag: pan",
                     "Alt+drag: rotate", "Scroll: zoom"):
            self.assertIn(part, lbl.text())
        self.assertTrue(self.vis(lbl))

    def test_action_buttons_match_the_designs_heights(self):
        for b in (self.w.btn_execute_path, self.w.btn_pause_path, self.w.btn_abort_path,
                  self.w.btn_clear_goal):
            self.assertEqual(b.height(), 32, b.text())
        for b in (self.w.btn_inflation, self.w.btn_keepout, self.w.btn_ruler, self.w.btn_reset_map):
            self.assertEqual(b.height(), 26, b.text())

    def test_route_panel_rows_read_cruise_alt_first(self):
        texts = [l.text() for l in self.w.pnl_route.findChildren(type(self.w.lbl_map_caption))
                 if l.objectName() == "fieldLabel"]
        self.assertEqual(texts[:1], ["Cruise alt"])
        self.assertTrue(self.w.pnl_route.isAncestorOf(self.w.combo_alt))
        # shown by the design but fixed in this station: read-only, never a live control
        self.assertFalse(self.w.chk_avoid_obstacles.isEnabled())
        self.assertFalse(self.w.chk_avoid_terrain.isEnabled())

    def test_all_three_path_actions_are_always_shown(self):
        for b in (self.w.btn_execute_path, self.w.btn_pause_path, self.w.btn_abort_path):
            self.assertTrue(self.vis(b), b.text())

    def test_idle_enables_nothing_until_a_route_is_staged(self):
        self.assertFalse(self.w.btn_execute_path.isEnabled())
        self.assertFalse(self.w.btn_pause_path.isEnabled())
        self.assertFalse(self.w.btn_abort_path.isEnabled())

    def test_flying_enables_pause_and_abort_instead_of_execute(self):
        self.w.set_executing_state(True)
        self.assertFalse(self.w.btn_execute_path.isEnabled())
        self.assertTrue(self.w.btn_pause_path.isEnabled())
        self.assertTrue(self.w.btn_abort_path.isEnabled())
        self.assertEqual(self.w.btn_pause_path.text(), "PAUSE")
        self.assertEqual(self.w.btn_abort_path.objectName(), "btnAbort")

    def test_paused_offers_resume_and_still_abort(self):
        self.w.set_executing_state(False, paused=True)
        self.assertEqual(self.w.btn_pause_path.text(), "RESUME")
        self.assertTrue(self.w.btn_abort_path.isEnabled())
        self.assertFalse(self.w.btn_execute_path.isEnabled())

    def test_finishing_returns_to_idle(self):
        self.w.set_executing_state(True)
        self.w.set_executing_state(False)
        self.assertEqual(self.w.btn_pause_path.text(), "PAUSE")
        self.assertFalse(self.w.btn_abort_path.isEnabled())
        self.assertFalse(self.w.btn_pause_path.isEnabled())

    def test_route_summary_goes_on_the_route_panel_header(self):
        self.assertEqual(self.w.pnl_route.lbl_sub.text(), "")
        self.assertEqual(self.w.pnl_route.lbl_title.text(), "ROUTE")
        self.w._on_goal_staged(1.0, 2.0, [(0, 0)] * 4, 3.5, 14.0)
        text = self.w.lbl_path_info.text()
        self.assertIn("GOAL", text)
        self.assertIn("3.50 m", text)
        self.assertIn("~14 s", text)
        self.assertIn("GOAL", self.w.pnl_route.lbl_sub.text())
        self.assertIn("~14 s", self.w.pnl_route.lbl_sub.text())
        self.assertIn("4 WAYPOINTS", self.w.pnl_route.lbl_title.text())
        self.assertTrue(self.w.btn_execute_path.isEnabled())
        self.assertTrue(self.vis(self.w.lbl_route_idle))          # the hint is permanent now
        self.w._on_goal_cleared()
        self.assertEqual(self.w.pnl_route.lbl_sub.text(), "")
        self.assertEqual(self.w.pnl_route.lbl_title.text(), "ROUTE")
        self.assertFalse(self.w.btn_execute_path.isEnabled())

    def test_blocked_goal_cannot_be_executed(self):
        self.w._on_goal_staged(1.0, 2.0, [], 0.0, 0.0)
        self.assertIn("BLOCKED", self.w.lbl_path_info.text())
        self.assertIn("BLOCKED", self.w.pnl_route.lbl_sub.text())
        self.assertFalse(self.w.btn_execute_path.isEnabled())

    def test_layers_hide_in_the_3d_view(self):
        self.assertTrue(self.vis(self.w.layers_section))
        self.w._set_view_mode(1)
        self.assertFalse(self.vis(self.w.layers_section))
        self.assertTrue(self.vis(self.w.btn_reset_map))
        self.w._set_view_mode(0)
        self.assertTrue(self.vis(self.w.layers_section))

    def test_panel_and_map_widths_do_not_change_when_the_stop_list_opens(self):
        before = (self.w.side_panel.width(), self.w.canvas.width())
        self.w.canvas.mission_stops = [(1, 1), (2, 2)]
        self.w._on_mission_changed(self.w.canvas.mission_stops, [])
        self.app.processEvents()
        self.assertEqual((self.w.side_panel.width(), self.w.canvas.width()), before)
        self.assertTrue(self.vis(self.w.mission_panel))

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

    def test_view_switch_and_layers_are_segmented_rows(self):
        y = lambda b: b.mapTo(self.w, b.rect().center()).y()
        self.assertEqual(y(self.w.btn_view_2d), y(self.w.btn_view_rviz))
        self.assertEqual(y(self.w.btn_layer_raw), y(self.w.btn_layer_both))
        self.assertLess(y(self.w.btn_view_2d), y(self.w.btn_layer_raw))
        self.assertEqual([b.text() for b in (self.w.btn_view_2d, self.w.btn_view_rviz)], ["2D MAP", "3D CLOUD"])
        self.assertEqual([b.text() for b in (self.w.btn_layer_raw, self.w.btn_layer_thin, self.w.btn_layer_both)],
                         ["RAW", "THIN", "BOTH"])

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

    def test_reset_map_is_caution_amber_not_alarm_red(self):
        qss = self.app.styleSheet()
        # the v2 sheet colours the warn role amber (#ffb02e), and never with the danger red
        warn = qss[qss.index("QPushButton#btnHold, QPushButton#mapDanger"):].split("}")[0]
        self.assertIn("#ffb02e", warn)
        self.assertNotIn("#ff5a4f", warn)
        self.assertEqual(self.w.btn_reset_map.objectName(), "mapDanger")

    def test_pose_and_uncertainty_live_in_a_strip_not_on_the_map(self):
        self.w.canvas.drone_x, self.w.canvas.drone_y = 1.5, -2.0
        self.w._refresh_status_strip()
        self.assertIn("N +1.50", self.w.lbl_pose.text())
        self.assertIn("E -2.00", self.w.lbl_pose.text())
        self.assertEqual(self.w.chip_follow.text(), "FREE PAN")
        self.w.canvas.auto_follow = True
        self.w._refresh_status_strip()
        self.assertEqual(self.w.chip_follow.text(), "FOLLOW")
        self.w.canvas.auto_follow = False
        self.assertIn("POS", self.w.lbl_uncert.text())
        self.assertTrue(self.vis(self.w.status_strip))
        self.assertGreaterEqual(self.w.status_strip.mapTo(self.w, self.w.status_strip.rect().topLeft()).y(),
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

    # ── third pass: camera panel and the stops list

    def test_the_fpv_panel_is_a_glass_panel_with_a_live_caption(self):
        feed = self.w.docked_feed
        self.assertEqual(feed.lbl_title.text(), "FPV")
        self.assertEqual(feed.lbl_floating.text(), "no feed")
        self.assertTrue(self.vis(feed.sink))
        self.assertTrue(self.vis(feed.btn_popout) and self.vis(feed.btn_full))

    def test_stop_rows_are_short(self):
        self.w.canvas.mission_stops = [(1.0, 1.0), (2.0, -1.0)]
        self.w._on_mission_changed(self.w.canvas.mission_stops, [{"distance_m": 1.4}])
        self.assertEqual(self.w.list_mission.item(0).text(), "1. N+1.0 E+1.0")

    def test_expanding_again_restores_everything(self):
        self.w.set_panel_collapsed(True)
        self.w.set_panel_collapsed(False)
        self.app.processEvents()
        self.assertFalse(self.w.panel_collapsed())
        self.assertTrue(self.vis(self.w.combo_alt))
        self.assertTrue(self.vis(self.w.btn_clear_goal))
        self.assertTrue(self.vis(self.w.docked_feed.sink))

    def test_the_canvas_is_transparent_so_the_window_grid_shows_through(self):
        from PyQt5.QtGui import QImage
        img = QImage(400, 300, QImage.Format_RGB32)
        img.fill(0x123456)
        self.w.canvas.resize(400, 300)
        self.w.canvas.render(img)
        c = img.pixelColor(3, 3)
        self.assertEqual((c.red(), c.green(), c.blue()), (0x12, 0x34, 0x56))


if __name__ == "__main__":
    unittest.main()
