"""Widget-level tests for the value grid, confirm bar, map chrome, route strip
and motor-test interlocks.

Not hermetic - these need PyQt5, so they are skipped where it is absent, the
same convention test_smoke.py uses. They construct real widgets offscreen
rather than testing a reimplementation of their logic.

`drone_gcs.py` is deliberately not imported here, for the reason given in
test_smoke.py: its module body re-execs the interpreter to inject ROS 2
library paths, which would restart the test runner mid-suite.
"""

import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

try:
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import Qt, QPoint, QEvent
    from PyQt5.QtGui import QMouseEvent
    HAVE_QT = True
except Exception:
    HAVE_QT = False

_app = None


def setUpModule():
    global _app
    if HAVE_QT:
        _app = QApplication.instance() or QApplication([])


def _mouse(kind, pos, button):
    return QMouseEvent(kind, pos, button, button, Qt.NoModifier)


def _release(widget):
    """Drive SlideToConfirm's release path without a real pointer device."""
    widget.mouseReleaseEvent(type("E", (), {"button": lambda s: None})())


# ── A5 ──────────────────────────────────────────────────────────────

@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class ValueGridTest(unittest.TestCase):
    def setUp(self):
        from ui.value_grid import ValueGridWidget
        self.ValueGridWidget = ValueGridWidget

    def test_default_layout_is_the_original_diagnostics_set(self):
        from ui.value_grid import DEFAULT_FIELDS, known_keys
        # Every default must resolve, or a fresh install shows blank tiles.
        unknown = [k for k in DEFAULT_FIELDS if k not in known_keys()]
        self.assertEqual(unknown, [])

    def test_unknown_keys_from_a_newer_build_are_dropped_not_fatal(self):
        g = self.ValueGridWidget(["volt", "not_a_real_field", "alt"])
        self.assertEqual(g.field_keys(), ["volt", "alt"])

    def test_duplicates_are_collapsed_preserving_order(self):
        g = self.ValueGridWidget(["alt", "volt", "alt"])
        self.assertEqual(g.field_keys(), ["alt", "volt"])

    def test_none_means_first_run_and_gets_the_defaults(self):
        from ui.value_grid import DEFAULT_FIELDS
        self.assertEqual(self.ValueGridWidget(None).field_keys(),
                         list(DEFAULT_FIELDS))

    def test_an_empty_selection_falls_back_to_the_defaults(self):
        # settings.json cannot tell "never configured" from "every tile
        # removed" - both are [] - so an empty selection restores the default
        # set rather than leaving a blank workspace with no way back.
        from ui.value_grid import DEFAULT_FIELDS
        self.assertEqual(self.ValueGridWidget([]).field_keys(),
                         list(DEFAULT_FIELDS))

    def test_a_layout_of_only_unknown_keys_falls_back_too(self):
        from ui.value_grid import DEFAULT_FIELDS
        self.assertEqual(self.ValueGridWidget(["nope", "also_nope"]).field_keys(),
                         list(DEFAULT_FIELDS))

    def test_values_render_from_a_snapshot(self):
        from core.telemetry import TelemetrySnapshot
        g = self.ValueGridWidget(["volt", "pct", "arm_state"])
        t = TelemetrySnapshot()
        t.battery_voltage, t.battery_percent, t.armed = 15.42, 77, True
        g.update_values(t)
        self.assertEqual(g._tiles["volt"].lbl_value.text(), "15.42 V")
        self.assertEqual(g._tiles["pct"].lbl_value.text(), "77 %")
        self.assertEqual(g._tiles["arm_state"].lbl_value.text(), "ARMED")

    def test_armed_is_coloured_as_a_hazard_not_as_healthy(self):
        from core.telemetry import TelemetrySnapshot
        from ui.styles import PALETTE
        g = self.ValueGridWidget(["arm_state"])
        t = TelemetrySnapshot()
        t.armed = True
        g.update_values(t)
        self.assertEqual(g._tiles["arm_state"]._base_colour, PALETTE["danger"])

    def test_battery_colour_follows_the_band(self):
        from core.telemetry import TelemetrySnapshot
        from ui.styles import PALETTE
        g = self.ValueGridWidget(["pct"])
        t = TelemetrySnapshot()
        for pct, expected in ((80, PALETTE["ok"]), (30, PALETTE["warn"]),
                              (10, PALETTE["danger"])):
            t.battery_percent = pct
            g.update_values(t)
            self.assertEqual(g._tiles["pct"]._base_colour, expected, f"at {pct}%")

    def test_station_extras_render_alongside_vehicle_fields(self):
        from core.telemetry import TelemetrySnapshot
        g = self.ValueGridWidget(["rx", "tx"])
        g.update_values(TelemetrySnapshot(), {"rx": "9.0 msg/s", "tx": "1.0 msg/s"})
        self.assertEqual(g._tiles["rx"].lbl_value.text(), "9.0 msg/s")

    def test_a_missing_extra_shows_a_blank_not_a_crash(self):
        from core.telemetry import TelemetrySnapshot
        g = self.ValueGridWidget(["rx"])
        g.update_values(TelemetrySnapshot(), {})
        self.assertEqual(g._tiles["rx"].lbl_value.text(), "--")

    def test_a_raising_formatter_is_contained_to_its_own_tile(self):
        from core.telemetry import TelemetrySnapshot
        g = self.ValueGridWidget(["volt", "alt"])
        g._tiles["volt"]  # present
        import ui.value_grid as vg
        original = vg.FIELDS["volt"]
        try:
            vg.FIELDS["volt"] = vg.FieldSpec(
                "volt", "Battery", "Power",
                lambda t: 1 / 0, None)
            g.update_values(TelemetrySnapshot())
            self.assertEqual(g._tiles["volt"].lbl_value.text(), "ERR")
            self.assertNotEqual(g._tiles["alt"].lbl_value.text(), "ERR")
        finally:
            vg.FIELDS["volt"] = original

    def test_editing_emits_the_layout_for_persistence(self):
        seen = []
        g = self.ValueGridWidget(["volt", "pct", "alt"])
        g.layout_changed.connect(lambda k, c, f: seen.append((list(k), c, f)))
        g._remove_key("pct")
        self.assertEqual(seen[-1][0], ["volt", "alt"])
        g._on_columns_changed(5)
        self.assertEqual(seen[-1][1], 5)

    def test_reordering_moves_one_position(self):
        g = self.ValueGridWidget(["a_unused", "volt", "pct", "alt"])
        g._move_key("alt", -1)
        self.assertEqual(g.field_keys(), ["volt", "alt", "pct"])
        g._move_key("volt", -1)   # already first: no-op, not an error
        self.assertEqual(g.field_keys(), ["volt", "alt", "pct"])


# ── B7 ──────────────────────────────────────────────────────────────

@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class GuidedConfirmTest(unittest.TestCase):
    def setUp(self):
        from ui.guided_confirm import GuidedConfirmBar
        self.bar = GuidedConfirmBar()
        self.confirmed = []
        self.cancelled = []
        self.bar.confirmed.connect(lambda a, v: self.confirmed.append((a, v)))
        self.bar.cancelled.connect(self.cancelled.append)

    def test_a_partial_drag_does_not_confirm(self):
        self.bar.request("kill", "EMERGENCY MOTOR KILL", danger=True)
        self.bar.slide._position = 0.85      # just under the commit fraction
        self.bar.slide._dragging = True
        _release(self.bar.slide)
        self.assertEqual(self.confirmed, [])
        self.assertEqual(self.bar.slide.position, 0.0, "knob must spring back")
        self.assertTrue(self.bar.is_active(), "action must still be pending")

    def test_a_full_drag_confirms_once(self):
        self.bar.request("kill", "EMERGENCY MOTOR KILL", danger=True)
        self.bar.slide._position = 1.0
        self.bar.slide._dragging = True
        _release(self.bar.slide)
        self.assertEqual(self.confirmed, [("kill", 0.0)])
        # A second release must not re-fire: the action was consumed.
        self.bar.slide._position = 1.0
        self.bar.slide._dragging = True
        _release(self.bar.slide)
        self.assertEqual(len(self.confirmed), 1)

    def test_the_slider_value_reaches_the_callback(self):
        self.bar.request("takeoff", "TAKEOFF", value_label="Altitude",
                         vmin=0.2, vmax=3.0, vinit=1.0, unit="m", step=0.1)
        self.bar.value_slider.slider.setValue(8)      # 0.2 + 8*0.1
        self.bar.slide._position = 1.0
        self.bar.slide._dragging = True
        _release(self.bar.slide)
        self.assertEqual(self.confirmed[0][0], "takeoff")
        self.assertAlmostEqual(self.confirmed[0][1], 1.0, places=3)

    def test_the_slider_cannot_express_an_out_of_range_value(self):
        self.bar.request("takeoff", "TAKEOFF", value_label="Altitude",
                         vmin=0.2, vmax=3.0, vinit=1.0, unit="m", step=0.1)
        self.bar.value_slider.slider.setValue(10_000)
        self.assertLessEqual(self.bar.value_slider.value(), 3.0)
        self.bar.value_slider.slider.setValue(-10_000)
        self.assertGreaterEqual(self.bar.value_slider.value(), 0.2)

    def test_an_out_of_range_seed_is_clamped_into_the_bounds(self):
        self.bar.request("takeoff", "TAKEOFF", value_label="Altitude",
                         vmin=0.2, vmax=3.0, vinit=99.0, unit="m", step=0.1)
        self.assertLessEqual(self.bar.value_slider.value(), 3.0)

    def test_cancel_reports_and_clears(self):
        self.bar.request("arm", "ARM MOTORS", danger=True)
        self.bar.cancel()
        self.assertEqual(self.cancelled, ["arm"])
        self.assertFalse(self.bar.is_active())
        self.assertEqual(self.confirmed, [])

    def test_a_new_request_resets_a_half_completed_drag(self):
        # A part-dragged takeoff must never carry over into a kill.
        self.bar.request("takeoff", "TAKEOFF", value_label="Altitude",
                         vmin=0.2, vmax=3.0, vinit=1.0, unit="m", step=0.1)
        self.bar.slide._position = 0.88
        self.bar.request("kill", "EMERGENCY MOTOR KILL", danger=True)
        self.assertEqual(self.bar.slide.position, 0.0)
        self.assertEqual(self.bar.active_action(), "kill")

    def test_a_bare_confirm_has_no_slider_value(self):
        self.bar.request("disarm", "DISARM", danger=True)
        self.assertIsNone(self.bar.pending_value())


# ── D14 / D17 ───────────────────────────────────────────────────────

@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class MapInteractionTest(unittest.TestCase):
    def setUp(self):
        from ui.slam_map_widget import SLAMMapCanvas
        self.c = SLAMMapCanvas()
        self.c.resize(800, 600)
        self.menus = []
        self.c.context_menu_requested.connect(
            lambda p, x, y: self.menus.append((x, y)))

    def test_a_right_click_opens_the_menu_and_does_not_pan(self):
        self.c.pan_x = self.c.pan_y = 0
        at = QPoint(500, 200)
        self.c.mousePressEvent(_mouse(QEvent.MouseButtonPress, at, Qt.RightButton))
        self.c.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, at, Qt.RightButton))
        self.assertEqual(len(self.menus), 1)
        self.assertEqual((self.c.pan_x, self.c.pan_y), (0, 0))

    def test_a_right_drag_pans_and_opens_no_menu(self):
        self.c.pan_x = self.c.pan_y = 0
        self.c.mousePressEvent(
            _mouse(QEvent.MouseButtonPress, QPoint(400, 300), Qt.RightButton))
        for x in (420, 460, 500):
            self.c.mouseMoveEvent(
                _mouse(QEvent.MouseMove, QPoint(x, 300), Qt.RightButton))
        self.c.mouseReleaseEvent(
            _mouse(QEvent.MouseButtonRelease, QPoint(500, 300), Qt.RightButton))
        self.assertEqual(self.menus, [])
        self.assertNotEqual(self.c.pan_x, 0)

    def test_jitter_within_the_slop_still_counts_as_a_click(self):
        self.c.pan_x = self.c.pan_y = 0
        self.c.mousePressEvent(
            _mouse(QEvent.MouseButtonPress, QPoint(300, 300), Qt.RightButton))
        self.c.mouseMoveEvent(
            _mouse(QEvent.MouseMove, QPoint(302, 301), Qt.RightButton))
        self.c.mouseReleaseEvent(
            _mouse(QEvent.MouseButtonRelease, QPoint(302, 301), Qt.RightButton))
        self.assertEqual(len(self.menus), 1)
        self.assertEqual((self.c.pan_x, self.c.pan_y), (0, 0),
                         "a click must not nudge the map")

    def test_left_click_stages_a_goal_normally(self):
        self.c.mousePressEvent(
            _mouse(QEvent.MouseButtonPress, QPoint(450, 250), Qt.LeftButton))
        self.assertIsNotNone(self.c.goal_pose)

    def test_the_ruler_intercepts_left_click_and_leaves_the_goal_alone(self):
        self.c.mousePressEvent(
            _mouse(QEvent.MouseButtonPress, QPoint(450, 250), Qt.LeftButton))
        staged = self.c.goal_pose
        self.c.set_ruler_active(True)
        self.c.mousePressEvent(
            _mouse(QEvent.MouseButtonPress, QPoint(460, 260), Qt.LeftButton))
        self.assertEqual(len(self.c.ruler_points), 1)
        self.assertEqual(self.c.goal_pose, staged,
                         "measuring must never restage a flight goal")

    def test_ruler_measures_multiple_segments(self):
        self.c.set_ruler_active(True)
        for pt in ((0.0, 0.0), (3.0, 0.0), (3.0, 4.0)):
            self.c.add_ruler_point(*pt)
        self.assertAlmostEqual(self.c.ruler_total_m(), 7.0, places=6)

    def test_leaving_measure_mode_clears_the_measurement(self):
        self.c.set_ruler_active(True)
        self.c.add_ruler_point(0.0, 0.0)
        self.c.set_ruler_active(False)
        self.assertEqual(self.c.ruler_points, [])

    def test_the_scale_bar_stays_readable_at_every_zoom(self):
        # The old bar was a fixed one metre: a few pixels wide zoomed out,
        # off the canvas zoomed in.
        for zoom in (2, 4, 12, 36, 120, 400, 1200, 4000):
            self.c.scale = zoom
            span_m, span_px = self.c._pick_scale_span()
            self.assertGreater(span_px, 40, f"bar too short at {zoom} px/m")
            self.assertLessEqual(span_px, 190, f"bar too long at {zoom} px/m")
            self.assertIn(span_m, self.c.SCALE_SPANS_M)

    def test_sub_metre_spans_are_labelled_in_centimetres(self):
        self.assertEqual(self.c._format_span(0.25), "25 cm")
        self.assertEqual(self.c._format_span(1.0), "1 m")
        self.assertEqual(self.c._format_span(2.5), "2.5 m")


# ── D16 ─────────────────────────────────────────────────────────────

@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class MissionProgressTest(unittest.TestCase):
    def setUp(self):
        from ui.mission_progress import MissionProgressBar
        self.m = MissionProgressBar()
        self.route = [(1.0, 0.0), (2.0, 0.0), (2.0, 2.0), (4.0, 2.0)]

    def test_leg_lengths_are_measured_from_the_starting_position(self):
        self.m.set_route(self.route, origin=(0.0, 0.0))
        self.assertAlmostEqual(sum(self.m._leg_lengths), 6.0, places=6)

    def test_remaining_distance_follows_the_route_not_the_straight_line(self):
        self.m.set_route(self.route, origin=(0.0, 0.0))
        self.m.update_progress(0, (0.0, 0.0), 1.0)
        self.assertEqual(self.m._readouts["remaining"].text(), "6.00 m")
        self.m.update_progress(2, (2.0, 1.0), 1.0)
        self.assertEqual(self.m._readouts["remaining"].text(), "3.00 m")

    def test_waypoint_index_is_one_based_for_the_operator(self):
        self.m.set_route(self.route, origin=(0.0, 0.0))
        self.m.update_progress(2, (2.0, 1.0), 1.0)
        self.assertEqual(self.m.lbl_index.text(), "W 3 / 4")

    def test_a_stationary_vehicle_shows_no_eta_rather_than_a_fake_one(self):
        self.m.set_route(self.route, origin=(0.0, 0.0))
        for _ in range(30):
            self.m.update_progress(1, (1.0, 0.0), 0.0)
        self.assertEqual(self.m._readouts["eta"].text(), "--")

    def test_eta_appears_once_the_vehicle_is_moving(self):
        self.m.set_route(self.route, origin=(0.0, 0.0))
        for _ in range(5):
            self.m.update_progress(0, (0.0, 0.0), 1.0)
        self.assertNotEqual(self.m._readouts["eta"].text(), "--")

    def test_an_out_of_range_index_does_not_raise(self):
        self.m.set_route(self.route, origin=(0.0, 0.0))
        self.m.update_progress(99, (0.0, 0.0), 1.0)
        self.assertEqual(self.m.lbl_index.text(), "W 4 / 4")

    def test_a_single_waypoint_route_is_not_a_division_by_zero(self):
        self.m.set_route([(1.0, 1.0)], origin=(0.0, 0.0))
        self.m.update_progress(0, (0.5, 0.5), 0.5)
        self.assertEqual(self.m.lbl_index.text(), "W 1 / 1")

    def test_clearing_hides_the_strip(self):
        self.m.set_route(self.route, origin=(0.0, 0.0))
        self.m.clear()
        self.assertFalse(self.m.isVisible())


# ── F26 ─────────────────────────────────────────────────────────────

@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class MotorTestInterlockTest(unittest.TestCase):
    """The five gates in front of spinning a motor from the GUI."""

    def setUp(self):
        from ui.motor_widget import MotorWidget
        self.w = MotorWidget()
        self.panel = self.w.test_panel
        self.sent = []
        self.stops = []
        self.w.motor_test_requested.connect(
            lambda i, t: self.sent.append((i, t)))
        self.w.motor_test_stop_requested.connect(lambda: self.stops.append(1))

    def _bench(self):
        self.w.set_vehicle_state(connected=True, armed=False, airborne=False)
        self.panel.chk_safety.setChecked(True)

    def test_disconnected_is_refused(self):
        self.panel._start_hold()
        self.assertEqual(self.sent, [])
        self.assertFalse(self.panel.testing_allowed())

    def test_connected_but_unacknowledged_is_refused(self):
        self.w.set_vehicle_state(connected=True, armed=False, airborne=False)
        self.panel._start_hold()
        self.assertEqual(self.sent, [])

    def test_armed_is_refused(self):
        self._bench()
        self.w.set_vehicle_state(connected=True, armed=True, airborne=False)
        self.panel._start_hold()
        self.assertEqual(self.sent, [])
        self.assertIn("ARMED", self.panel._block_reason())

    def test_airborne_is_refused(self):
        self._bench()
        self.w.set_vehicle_state(connected=True, armed=False, airborne=True)
        self.panel._start_hold()
        self.assertEqual(self.sent, [])

    def test_the_safe_bench_state_is_allowed(self):
        self._bench()
        self.panel._start_hold()
        self.panel._repeat.stop()
        self.assertEqual(len(self.sent), 1)

    def test_arming_mid_test_stops_it_and_reclears_the_acknowledgement(self):
        self._bench()
        self.panel._start_hold()
        self.panel._repeat.stop()
        self.w.set_vehicle_state(connected=True, armed=True, airborne=False)
        self.assertTrue(self.stops, "must command a stop, not just disable buttons")
        self.assertFalse(self.panel.chk_safety.isChecked())

    def test_releasing_the_hold_commands_a_stop(self):
        self._bench()
        self.panel._start_hold()
        self.stops.clear()
        self.panel._stop_hold()
        self.assertTrue(self.stops)

    def test_throttle_is_capped_well_below_anything_that_lifts(self):
        self.assertLessEqual(self.panel.slider_throttle.maximum(),
                             self.panel.MAX_THROTTLE_PCT)
        self.assertLessEqual(self.panel.MAX_THROTTLE_PCT, 30)

    def test_the_repeat_interval_is_inside_the_vehicle_side_expiry(self):
        from protocol.mavlink_worker import MAVLinkWorker
        self.assertLess(self.panel.REPEAT_MS / 1000.0,
                        MAVLinkWorker.MOTOR_TEST_DEFAULT_TIMEOUT_S,
                        "a held motor would stutter, or worse, outlive the link")

    def test_stop_all_stays_available_when_everything_else_is_locked(self):
        self.w.set_vehicle_state(connected=True, armed=True, airborne=True)
        self.assertTrue(self.panel.btn_stop.isEnabled())
        self.assertFalse(self.panel.btn_spin.isEnabled())

    def test_the_safety_acknowledgement_expires(self):
        self._bench()
        self.assertTrue(self.panel.testing_allowed())
        self.panel._expire_safety()
        self.assertFalse(self.panel.testing_allowed())
        self.assertFalse(self.panel.chk_safety.isChecked())

    def test_selecting_a_rotor_on_the_diagram_selects_it_for_testing(self):
        self.w._on_geometry_clicked(3)
        self.assertEqual(self.panel.selected_motor(), 3)
        self.assertEqual(self.w.geometry.selected, 3)

    def test_the_sequence_walks_every_motor_then_stops(self):
        self._bench()
        self.panel.btn_sequence.setChecked(True)
        seen = {self.panel.selected_motor()}
        for _ in range(3):
            self.panel._advance_sequence()
            seen.add(self.panel.selected_motor())
        self.assertEqual(seen, {1, 2, 3, 4})
        self.panel._advance_sequence()     # past the end
        self.assertFalse(self.panel.btn_sequence.isChecked())

    def test_the_diagram_and_the_bars_read_the_same_mapping(self):
        from ui.motor_widget import QUAD_X_LAYOUT
        self.assertEqual([n for n, _l, _s, _p in QUAD_X_LAYOUT], [1, 2, 3, 4])
        senses = {n: s for n, _l, s, _p in QUAD_X_LAYOUT}
        # PX4 Quad X: the two CCW motors are diagonal, as are the two CW.
        self.assertEqual(senses[1], senses[2])
        self.assertEqual(senses[3], senses[4])
        self.assertNotEqual(senses[1], senses[3])


# ── G31 ─────────────────────────────────────────────────────────────

@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class ShortcutTableTest(unittest.TestCase):
    def test_no_duplicate_key_sequences(self):
        from ui.shortcuts import BINDINGS
        keys = [b.keys for b in BINDINGS]
        self.assertEqual(len(keys), len(set(keys)))

    def test_emergency_kill_has_no_binding(self):
        from ui.shortcuts import BINDINGS
        for b in BINDINGS:
            self.assertNotIn("kill", b.action.lower())

    def test_every_destructive_action_is_marked_guarded(self):
        from ui.shortcuts import BINDINGS
        guarded = {b.action for b in BINDINGS if b.guarded}
        self.assertEqual(guarded, {"arm", "disarm", "takeoff", "abort_path"})

    def test_no_bare_single_key_bindings_except_help_and_escape(self):
        """An application-context shortcut is matched before the focused widget
        sees the key, so a bare key is taken away from the entire UI.

        This is a regression test with a specific history: `Space` was bound to
        abort-path, which silently overrode Qt's "activate the focused button"
        behaviour app-wide - tabbing to ARM and pressing Space ran the abort
        action instead of arming. Esc and F1 are the two deliberate exceptions;
        both were measured to be safe.
        """
        from ui.shortcuts import BINDINGS
        allowed = {"Esc", "F1"}
        bare = [b.keys for b in BINDINGS if "+" not in b.keys and b.keys not in allowed]
        self.assertEqual(bare, [], f"bare single-key bindings steal keys app-wide: {bare}")

    def test_space_still_activates_a_focused_button(self):
        from PyQt5.QtWidgets import QMainWindow, QPushButton
        from PyQt5.QtTest import QTest
        from ui.shortcuts import install_shortcuts
        host = QMainWindow()
        button = QPushButton("ARM", host)
        host.setCentralWidget(button)
        host.show()
        QTest.qWaitForWindowExposed(host)
        activated = []
        button.clicked.connect(lambda: activated.append(1))
        install_shortcuts(host, {"abort_path": lambda: activated.append("ABORT")})
        button.setFocus()
        QTest.keyClick(button, Qt.Key_Space)
        self.assertEqual(activated, [1])

    def test_a_missing_callback_is_skipped_rather_than_fatal(self):
        from PyQt5.QtWidgets import QWidget
        from ui.shortcuts import install_shortcuts, BINDINGS
        host = QWidget()
        created = install_shortcuts(host, {"toggle_mute": lambda: None})
        self.assertEqual(len(created), 1)
        self.assertLess(len(created), len(BINDINGS))

    def test_the_help_overlay_lists_every_key_sequence(self):
        # The overlay is generated from BINDINGS, so it cannot drift from the
        # bindings - but only if it really renders all of them.
        from PyQt5.QtWidgets import QLabel
        from ui.shortcuts import ShortcutHelpOverlay, BINDINGS
        overlay = ShortcutHelpOverlay()
        shown = {lbl.text() for lbl in overlay.findChildren(QLabel)}
        for binding in BINDINGS:
            self.assertIn(binding.keys, shown,
                          f"{binding.action} is bound but undocumented")


if __name__ == "__main__":
    unittest.main(verbosity=2)
