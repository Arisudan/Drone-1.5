"""Motor tab response: range model, per-motor response, staleness, commanded vs
live, parameter plumbing, Diagnostics tiles.

The question these answer: when real motor values flow in, does each of M1-M4
react to its own value and only its own, scaled to the vehicle's range, and does
it fall back to idle when the value is low - and does a dead stream stop looking
like a live one? Time is injected; no vehicle involved.
"""

import time
import unittest
from types import SimpleNamespace

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core import motor_range as mr
from core.motor_range import MotorRange
from core.telemetry import TelemetrySnapshot


def vehicle_range(lo=1100, hi=1900, dis=1000, funcs=(101, 102, 103, 104)):
    r = MotorRange()
    for n in range(1, 5):
        r.on_param(f"PWM_MAIN_MIN{n}", lo)
        r.on_param(f"PWM_MAIN_MAX{n}", hi)
        r.on_param(f"PWM_MAIN_DIS{n}", dis)
        r.on_param(f"PWM_MAIN_FUNC{n}", funcs[n - 1])
    return r


class MotorRangeTest(unittest.TestCase):
    def test_unknown_uses_documented_defaults_and_says_so(self):
        r = MotorRange()
        self.assertFalse(r.known)
        self.assertAlmostEqual(r.fraction(1, 1500), 0.5)
        self.assertIn("not read yet", r.describe())
        self.assertEqual(len(r.missing()), 16)

    def test_scale_follows_the_vehicle_range(self):
        r = vehicle_range()
        self.assertTrue(r.known)
        self.assertEqual(r.fraction(1, 1100), 0.0)
        self.assertAlmostEqual(r.fraction(1, 1500), 0.5)
        self.assertEqual(r.fraction(1, 1900), 1.0)       # full throttle is 100 %, not 80 %
        self.assertIn("1100–1900", r.describe())

    def test_states_are_relative_to_the_range(self):
        r = vehicle_range()
        self.assertEqual(r.state(1, 1000), mr.OFF)        # disarmed output, below min
        self.assertEqual(r.state(1, 1100), mr.IDLE)       # at min
        self.assertEqual(r.state(1, 1500), mr.NOMINAL)
        self.assertEqual(r.state(1, 1750), mr.HIGH)
        self.assertEqual(r.state(1, 1880), mr.SATURATED)
        self.assertEqual(r.state(1, 2050), mr.SATURATED)  # beyond max

    def test_100_us_is_not_idle_on_a_wide_range_but_is_on_the_default(self):
        # The same 1100 us that is "at minimum" on 1100-1900 is 10 % on 1000-2000.
        self.assertEqual(vehicle_range().state(1, 1100), mr.IDLE)
        self.assertEqual(MotorRange().state(1, 1100), mr.NOMINAL)

    def test_throttle_maps_through_the_motors_own_range(self):
        r = vehicle_range()
        self.assertEqual(r.pwm_for_throttle(1, 0), 1100)
        self.assertEqual(r.pwm_for_throttle(1, 25), 1300)
        self.assertEqual(r.pwm_for_throttle(1, 100), 1900)
        self.assertEqual(r.pwm_for_throttle(1, 500), 1900)  # clamped

    def test_func_mapping_reorders_outputs_into_motor_order(self):
        # Motor 1 on output 2 and Motor 2 on output 1 (swapped).
        r = vehicle_range(funcs=(102, 101, 103, 104))
        self.assertEqual(r.motor_pwms([1111, 1222, 1333, 1444]), [1222, 1111, 1333, 1444])

    def test_a_half_known_or_broken_mapping_never_shuffles(self):
        r = vehicle_range(funcs=(101, 101, 103, 104))      # not a permutation
        self.assertEqual(r.motor_pwms([1, 2, 3, 4]), [1, 2, 3, 4])
        r2 = MotorRange()
        r2.on_param("PWM_MAIN_FUNC1", 102)                 # only one known
        self.assertEqual(r2.motor_pwms([1, 2, 3, 4]), [1, 2, 3, 4])

    def test_on_param_reports_changes_only(self):
        r = MotorRange()
        self.assertTrue(r.on_param("PWM_MAIN_MIN1", 1100))
        self.assertFalse(r.on_param("PWM_MAIN_MIN1", 1100))
        self.assertFalse(r.on_param("SOMETHING_ELSE", 5))
        self.assertFalse(r.on_param("PWM_MAIN_MIN7", 1000))   # beyond SERVO_OUTPUT_RAW 1..4

    def test_inverted_range_falls_back_rather_than_dividing_by_zero(self):
        r = vehicle_range(lo=1900, hi=1100)
        self.assertFalse(r.known)
        self.assertAlmostEqual(r.fraction(1, 1500), 0.5)      # default scale

    def test_reset_forgets_the_old_vehicle(self):
        r = vehicle_range()
        r.reset()
        self.assertFalse(r.known)


class TelemetryMotorFreshnessTest(unittest.TestCase):
    def _msg(self, a, b, c, d):
        return SimpleNamespace(servo1_raw=a, servo2_raw=b, servo3_raw=c, servo4_raw=d)

    def test_update_records_values_and_time(self):
        t = TelemetrySnapshot()
        self.assertEqual(t.motor_age, 999.0)
        t.update_servo_output(self._msg(1500, 1100, 1850, 1300))
        self.assertEqual(t.motor_pwms, [1500, 1100, 1850, 1300])
        self.assertGreater(t.last_motor_time, 0.0)
        self.assertEqual(t.motor_age, 0.0)

    def test_age_keeps_growing_after_the_stream_stops(self):
        t = TelemetrySnapshot()
        t.update_servo_output(self._msg(1500, 1500, 1500, 1500))
        t.last_motor_time = time.time() - 4.0
        t.check_motor_staleness()
        self.assertGreater(t.motor_age, 3.5)

    def test_never_received_stays_at_the_sentinel(self):
        t = TelemetrySnapshot()
        t.check_motor_staleness()
        self.assertEqual(t.motor_age, 999.0)

    def test_clone_keeps_the_freshness_fields(self):
        t = TelemetrySnapshot()
        t.update_servo_output(self._msg(1, 2, 3, 4))
        c = t.clone()
        self.assertEqual(c.last_motor_time, t.last_motor_time)


class MotorWidgetResponseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        from ui.motor_widget import MotorWidget
        self.t = [100.0]
        self.rng = vehicle_range()
        self.w = MotorWidget(clock=lambda: self.t[0])
        self.w.set_range(self.rng)

    def tearDown(self):
        self.w.geometry._anim.stop()

    def _states(self):
        return [self.w.bars[k].state() for k in (1, 2, 3, 4)]

    def test_each_motor_reacts_only_to_its_own_value(self):
        for hot in (1, 2, 3, 4):
            out = [1100, 1100, 1100, 1100]
            out[hot - 1] = 1700
            self.w.update_pwms(out, age_s=0.1)
            for k in (1, 2, 3, 4):
                bar = self.w.bars[k]
                if k == hot:
                    self.assertEqual(bar.state(), mr.NOMINAL, f"M{k} should respond")
                    self.assertAlmostEqual(bar.fraction(), 0.75)
                else:
                    self.assertEqual(bar.state(), mr.IDLE, f"M{k} must not move for M{hot}")
                    self.assertEqual(bar.fraction(), 0.0)
                self.assertEqual(self.w.geometry.pwms[k - 1], out[k - 1])

    def test_low_values_fall_back_to_idle_then_off(self):
        self.w.update_pwms([1800, 1800, 1800, 1800], age_s=0.1)
        self.assertEqual(self._states(), [mr.HIGH] * 4)
        self.w.update_pwms([1100, 1100, 1100, 1100], age_s=0.1)
        self.assertEqual(self._states(), [mr.IDLE] * 4)
        self.assertIn("IDLE", self.w.lbl_status.text())
        self.w.update_pwms([1000, 1000, 1000, 1000], age_s=0.1)
        self.assertEqual(self._states(), [mr.OFF] * 4)
        self.assertIn("OFF", self.w.lbl_status.text())

    def test_value_sweep_is_monotonic_per_motor(self):
        last = [-1.0] * 4
        for pwm in range(1000, 1950, 50):
            self.w.update_pwms([pwm] * 4, age_s=0.1)
            now = [self.w.bars[k].fraction() for k in (1, 2, 3, 4)]
            for a, b in zip(last, now):
                self.assertGreaterEqual(b, a)
            last = now
        self.assertEqual(last, [1.0] * 4)

    def test_status_names_the_saturated_motor(self):
        self.w.update_pwms([1500, 1500, 1890, 1500], age_s=0.1)
        self.assertIn("SATURATION", self.w.lbl_status.text())
        self.assertIn("M3", self.w.lbl_status.text())

    def test_swapped_function_mapping_lights_the_right_arm(self):
        self.w.set_range(vehicle_range(funcs=(102, 101, 103, 104)))
        self.w.update_pwms([1800, 1100, 1100, 1100], age_s=0.1)   # output 1 is Motor 2
        self.assertEqual(self.w.bars[2].state(), mr.HIGH)
        self.assertEqual(self.w.bars[1].state(), mr.IDLE)

    def test_the_diagram_and_the_bars_agree_on_every_state(self):
        for pwm in (1000, 1100, 1500, 1750, 1890):
            self.w.update_pwms([pwm] * 4, age_s=0.1)
            for k in (1, 2, 3, 4):
                self.assertEqual(self.rng.state(k, self.w.geometry.pwms[k - 1]),
                                 self.w.bars[k].state())

    # ── staleness ──────────────────────────────────────────────────────
    def test_stale_stream_is_greyed_and_labelled_not_frozen(self):
        self.w.update_pwms([1500] * 4, age_s=0.2)
        self.assertFalse(self.w.bars[1].stale)
        self.w.update_pwms([1500] * 4, age_s=3.5)
        self.assertTrue(all(self.w.bars[k].stale for k in (1, 2, 3, 4)))
        self.assertTrue(self.w.geometry.stale)
        self.assertIn("STALE", self.w.lbl_status.text())

    def test_never_received_says_no_motor_data(self):
        self.w.update_pwms([1000] * 4, age_s=999.0)
        self.assertEqual(self.w.lbl_status.text(), "NO MOTOR DATA")

    def test_propellers_never_spin_on_stale_data(self):
        g = self.w.geometry
        g.set_pwms([1800] * 4, stale=False)
        self.assertNotEqual(g.spin_rate_dps(1), 0.0)
        g.set_pwms([1800] * 4, stale=True)
        self.assertEqual(g.spin_rate_dps(1), 0.0)

    def test_recovery_after_stale_is_immediate(self):
        self.w.update_pwms([1500] * 4, age_s=5.0)
        self.w.update_pwms([1500] * 4, age_s=0.1)
        self.assertFalse(self.w.geometry.stale)
        self.assertIn("ACTIVE", self.w.lbl_status.text())

    # ── propeller animation ────────────────────────────────────────────
    def test_spin_direction_follows_rotation_sense_and_speed_follows_throttle(self):
        g = self.w.geometry
        g.set_pwms([1500, 1500, 1500, 1500])
        self.assertLess(g.spin_rate_dps(1), 0)        # M1 CCW
        self.assertGreater(g.spin_rate_dps(3), 0)     # M3 CW
        slow = abs(g.spin_rate_dps(1))
        g.set_pwms([1900, 1500, 1500, 1500])
        self.assertGreater(abs(g.spin_rate_dps(1)), slow)
        g.set_pwms([1000, 1500, 1500, 1500])
        self.assertEqual(g.spin_rate_dps(1), 0.0)     # output off: not turning

    def test_advance_turns_only_spinning_blades(self):
        g = self.w.geometry
        g.set_pwms([1000, 1900, 1000, 1000])
        before = list(g._angles)
        g.advance(0.1)
        self.assertEqual(g._angles[0], before[0])
        self.assertNotEqual(g._angles[1], before[1])

    def test_every_state_paints_without_error(self):
        self.w.show()
        for out, age in (([1000] * 4, 0.1), ([1100] * 4, 0.1), ([1500] * 4, 0.1),
                         ([1890] * 4, 0.1), ([1500] * 4, 9.0), ([1000] * 4, 999.0)):
            self.w.update_pwms(out, age_s=age)
            self.w.geometry.advance(0.05)
            self.assertFalse(self.w.grab().isNull())

    # ── commanded vs live ──────────────────────────────────────────────
    def test_bench_command_is_shown_at_once_and_marked_commanded(self):
        self.w._reflect_test_locally(2, 25.0)
        self.assertEqual(self.w.bars[2].pwm, self.rng.pwm_for_throttle(2, 25.0))   # 1300
        self.assertEqual(self.w.bars[1].pwm, 1000)
        self.assertIn("COMMANDED", self.w.lbl_status.text())

    def test_a_live_sample_that_confirms_takes_over(self):
        self.w._reflect_test_locally(2, 25.0)
        self.t[0] += 0.3
        self.w.update_pwms([1000, 1300, 1000, 1000], age_s=0.1)   # newer than the command
        self.assertNotIn("COMMANDED", self.w.lbl_status.text())

    def test_a_fresh_live_sample_that_is_idle_does_not_hide_the_command(self):
        self.w._reflect_test_locally(2, 25.0)
        self.t[0] += 0.3
        self.w.update_pwms([1000, 1000, 1000, 1000], age_s=0.1)   # vehicle not reporting it
        self.assertEqual(self.w.bars[2].pwm, 1300)
        self.assertIn("not yet confirmed", self.w.lbl_status.text())

    def test_a_sample_older_than_the_command_never_overrides_it(self):
        self.t[0] += 5.0
        self.w._reflect_test_locally(2, 25.0)
        self.w.update_pwms([1000, 1000, 1000, 1000], age_s=4.0)   # taken before the command
        self.assertEqual(self.w.bars[2].pwm, 1300)

    def test_stop_goes_to_idle_immediately_and_then_follows_live(self):
        self.w._reflect_test_locally(2, 25.0)
        self.w._reflect_stop_locally()
        self.assertEqual([self.w.bars[k].pwm for k in (1, 2, 3, 4)], [1000] * 4)
        self.t[0] += 1.0                                # hold lapsed
        self.w.update_pwms([1100, 1100, 1100, 1100], age_s=0.1)
        self.assertEqual(self.w.bars[2].pwm, 1100)

    def test_command_hold_lapses_without_telemetry(self):
        self.w._reflect_test_locally(1, 25.0)
        self.t[0] += self.w.COMMAND_HOLD_S + 0.1
        self.w._render()
        self.assertNotIn("COMMANDED", self.w.lbl_status.text())

    def test_the_throttle_label_shows_the_vehicles_micro_seconds(self):
        p = self.w.test_panel
        p.slider_throttle.setValue(25)
        self.assertEqual(p.lbl_throttle.text(), "25 / 25 % · 1300 µs")


class FrameGeometryDeclutterTest(unittest.TestCase):
    """Captions, rotation arrows, discs and the body must never sit on top of one
    another. The first design put each arrow on the rotor's diagonal outer side,
    which is exactly where the front/rear caption goes, so the two were drawn
    across each other."""

    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    SHAPES = [(300, 460), (360, 360), (520, 300), (380, 480), (260, 260),
              (700, 620), (900, 400)]

    def _each(self):
        from PyQt5.QtCore import QRectF
        from ui.scaling import set_scale
        from ui.motor_widget import FrameGeometryWidget
        try:
            for scale in (1.0, 1.35, 2.0):
                set_scale(scale)
                for w, h in self.SHAPES:
                    g = FrameGeometryWidget()
                    g.resize(w, h)
                    lay = g._layout()
                    yield g, lay, QRectF, f"{w}x{h}@{scale}"
        finally:
            set_scale(1.0)

    @staticmethod
    def _disc(g, lay, num, QRectF, factor=1.0):
        mx, my = g.rotor_centres()[num]
        r = lay["rotor_r"] * factor
        return QRectF(mx - r, my - r, 2 * r, 2 * r)

    def test_a_caption_never_touches_any_arrow(self):
        bad = []
        for g, lay, _R, tag in self._each():
            if not lay["show_captions"]:
                continue
            for a in (1, 2, 3, 4):
                for b in (1, 2, 3, 4):
                    if g.caption_rect(a).intersects(g.arrow_rect(b)):
                        bad.append(f"{tag}: caption M{a} hits arrow M{b}")
        self.assertEqual(bad, [])

    def test_a_caption_never_touches_a_disc_or_another_caption(self):
        bad = []
        for g, lay, R, tag in self._each():
            if not lay["show_captions"]:
                continue
            for a in (1, 2, 3, 4):
                for b in (1, 2, 3, 4):
                    if g.caption_rect(a).intersects(self._disc(g, lay, b, R, 1.04)):
                        bad.append(f"{tag}: caption M{a} on disc M{b}")
                    if a < b and g.caption_rect(a).intersects(g.caption_rect(b)):
                        bad.append(f"{tag}: caption M{a} on caption M{b}")
        self.assertEqual(bad, [])

    def test_an_arrow_stays_off_its_own_arm_and_other_discs(self):
        bad = []
        for g, lay, R, tag in self._each():
            for a in (1, 2, 3, 4):
                for b in (1, 2, 3, 4):
                    if a != b and g.arrow_rect(a).intersects(self._disc(g, lay, b, R)):
                        bad.append(f"{tag}: arrow M{a} on disc M{b}")
        self.assertEqual(bad, [])

    def test_arrows_sit_on_the_horizontal_outer_side(self):
        from ui.motor_widget import QUAD_X_LAYOUT
        from PyQt5.QtWidgets import QApplication  # noqa: F401
        g = __import__("ui.motor_widget", fromlist=["x"]).FrameGeometryWidget()
        g.resize(400, 400)
        cx = g.width() / 2.0
        for num, _l, _s, (rx, _fy) in QUAD_X_LAYOUT:
            mx, _my = g.rotor_centres()[num]
            a = g.arrow_rect(num)
            if rx > 0:
                self.assertGreater(a.center().x(), mx, f"M{num} arrow must be right of its rotor")
                self.assertGreater(a.left(), cx)
            else:
                self.assertLess(a.center().x(), mx, f"M{num} arrow must be left of its rotor")
                self.assertLess(a.right(), cx)

    def test_everything_still_fits_the_widget(self):
        bad = [tag for g, _lay, _R, tag in self._each() if not g.layout_fits()]
        self.assertEqual(bad, [])


class ThrottleSliderTest(unittest.TestCase):
    """The throttle slider must be easy to hit: press anywhere to jump, drag to
    follow, wheel and +/- to nudge - and never move while disabled."""

    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        from ui.motor_widget import MotorTestPanel
        self.panel = MotorTestPanel()
        self.panel.resize(420, 520)
        self.panel.show()
        self.panel.set_vehicle_state(True, False, False)
        self.panel.chk_safety.setChecked(True)
        self.s = self.panel.slider_throttle
        self.app.processEvents()

    def tearDown(self):
        # Close and delete while the event loop can still finish any pending
        # paint; letting Python garbage-collect a SHOWN top-level widget is what
        # crashes ("paint device that is being painted").
        for t in (self.panel._repeat, self.panel._sequence,
                  self.panel._arm_expiry, self.panel._countdown):
            t.stop()
        self.panel.close()
        self.panel.deleteLater()
        self.app.processEvents()

    def _click(self, x, y=None, release=True, move_to=None):
        from PyQt5.QtCore import Qt, QPoint
        from PyQt5.QtTest import QTest
        y = self.s.height() // 2 if y is None else y
        QTest.mousePress(self.s, Qt.LeftButton, Qt.NoModifier, QPoint(int(x), int(y)))
        if move_to is not None:
            QTest.mouseMove(self.s, QPoint(int(move_to), int(y)))
            self.s.mouseMoveEvent(__import__("PyQt5.QtGui", fromlist=["x"]).QMouseEvent(
                __import__("PyQt5.QtCore", fromlist=["x"]).QEvent.MouseMove,
                QPoint(int(move_to), int(y)), Qt.NoButton, Qt.LeftButton, Qt.NoModifier))
        if release:
            QTest.mouseRelease(self.s, Qt.LeftButton, Qt.NoModifier, QPoint(int(move_to if move_to is not None else x), int(y)))

    def test_it_is_a_big_target(self):
        from ui.scaling import px
        self.assertGreaterEqual(self.s.height(), px(34))
        self.assertGreaterEqual(px(self.s.THUMB_D), px(24))

    def test_value_and_position_round_trip_across_the_whole_range(self):
        last = -1.0
        for v in range(self.s.minimum(), self.s.maximum() + 1):
            x = self.s.value_to_x(v)
            self.assertGreater(x, last)
            last = x
            self.assertEqual(self.s.x_to_value(x), v)

    def test_pressing_anywhere_jumps_the_handle_there(self):
        self._click(self.s.value_to_x(20))
        self.assertEqual(self.s.value(), 20)
        self._click(self.s.value_to_x(3))
        self.assertEqual(self.s.value(), 3)

    def test_pressing_off_the_ends_clamps_to_min_and_max(self):
        self._click(0)
        self.assertEqual(self.s.value(), self.s.minimum())
        self._click(self.s.width() - 1)
        self.assertEqual(self.s.value(), self.s.maximum())

    def test_dragging_follows_the_pointer(self):
        self._click(self.s.value_to_x(5), release=False, move_to=self.s.value_to_x(17))
        self.assertEqual(self.s.value(), 17)
        self.assertTrue(self.s._dragging)
        from PyQt5.QtCore import Qt, QPoint
        from PyQt5.QtTest import QTest
        QTest.mouseRelease(self.s, Qt.LeftButton, Qt.NoModifier,
                           QPoint(int(self.s.value_to_x(17)), 5))
        self.assertFalse(self.s._dragging)

    def test_the_label_follows_and_carries_the_limit(self):
        self._click(self.s.value_to_x(12))
        self.assertTrue(self.panel.lbl_throttle.text().startswith("12 / 25 %"))

    def test_plus_and_minus_step_by_one_and_stop_at_the_limits(self):
        self.s.setValue(8)
        self.panel.btn_thr_plus.click()
        self.assertEqual(self.s.value(), 9)
        self.panel.btn_thr_minus.click()
        self.panel.btn_thr_minus.click()
        self.assertEqual(self.s.value(), 7)
        self.s.setValue(self.s.maximum())
        self.panel.btn_thr_plus.click()
        self.assertEqual(self.s.value(), self.s.maximum())     # the ceiling holds
        self.s.setValue(self.s.minimum())
        self.panel.btn_thr_minus.click()
        self.assertEqual(self.s.value(), self.s.minimum())

    def test_the_wheel_nudges_by_one_per_notch(self):
        from PyQt5.QtCore import QPoint, QPointF, Qt
        from PyQt5.QtGui import QWheelEvent
        self.s.setValue(10)
        ev = QWheelEvent(QPointF(10, 10), QPointF(10, 10), QPoint(0, 0), QPoint(0, 120),
                         Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase, False)
        self.s.wheelEvent(ev)
        self.assertEqual(self.s.value(), 11)

    def test_it_cannot_be_moved_while_locked(self):
        self.panel.chk_safety.setChecked(False)           # locks the panel
        self.assertFalse(self.s.isEnabled())
        before = self.s.value()
        self._click(self.s.value_to_x(22))
        self.assertEqual(self.s.value(), before)
        self.assertFalse(self.panel.btn_thr_plus.isEnabled())
        self.assertFalse(self.panel.btn_thr_minus.isEnabled())

    def test_it_never_exceeds_the_ceiling(self):
        self.assertEqual(self.s.maximum(), self.panel.MAX_THROTTLE_PCT)
        self.s.setValue(999)
        self.assertEqual(self.s.value(), self.panel.MAX_THROTTLE_PCT)

    def test_it_paints_enabled_pressed_and_disabled(self):
        for enabled in (True, False):
            self.s.setEnabled(enabled)
            self.s._dragging = enabled
            self.assertFalse(self.s.grab().isNull())
        self.s._dragging = False


class MotorParameterPlumbingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _worker(self):
        from protocol.mavlink_worker import MAVLinkWorker
        w = MAVLinkWorker(host="127.0.0.1", port=1)
        w.target_system = 1
        self.motor, self.table = [], []
        w.motor_param_received.connect(lambda n, v: self.motor.append((n, v)))
        w.param_value_received.connect(lambda *a: self.table.append(a[0]))
        return w

    @staticmethod
    def _pv(name, value, idx=5, count=1500):
        # param_value carries a float whose BITS are the value for INT types; for
        # REAL32 (type 9) it is the float itself.
        class M:
            def get_type(self): return "PARAM_VALUE"
            def get_srcSystem(self): return 1
        m = M()
        m.param_id, m.param_value, m.param_type = name, float(value), 9
        m.param_index, m.param_count = idx, count
        return m

    def test_a_single_read_reaches_the_motor_model_but_not_the_params_table(self):
        w = self._worker()
        w._single_reads.add("PWM_MAIN_MIN1")
        w._handle_msg(self._pv("PWM_MAIN_MIN1", 1100))
        self.assertEqual(self.motor, [("PWM_MAIN_MIN1", 1100.0)])
        self.assertEqual(self.table, [], "would make the Parameters tab skip its first full fetch")

    def test_during_a_bulk_list_the_same_parameter_also_reaches_the_table(self):
        import time as _t
        w = self._worker()
        w._single_reads.add("PWM_MAIN_MIN1")
        w._list_active_until = _t.time() + 60
        w._handle_msg(self._pv("PWM_MAIN_MIN1", 1100))
        self.assertEqual(len(self.motor), 1)
        self.assertEqual(self.table, ["PWM_MAIN_MIN1"])

    def test_motor_params_in_a_list_that_we_did_not_request_singly_pass_through(self):
        w = self._worker()
        w._handle_msg(self._pv("PWM_MAIN_MAX3", 1900))
        self.assertEqual(self.table, ["PWM_MAIN_MAX3"])
        self.assertEqual(self.motor, [("PWM_MAIN_MAX3", 1900.0)])

    def test_other_parameters_never_reach_the_motor_model(self):
        w = self._worker()
        w._handle_msg(self._pv("MC_ROLL_P", 6.5))
        self.assertEqual(self.motor, [])
        self.assertEqual(self.table, ["MC_ROLL_P"])

    def test_request_sends_one_read_per_needed_parameter(self):
        w = self._worker()
        sent = []

        class Mav:
            def param_request_read_send(self, ts, tc, name, idx):
                sent.append((name, idx))

        class Master:
            mav = Mav()

        w.master = Master()
        self.assertTrue(w.request_motor_range_params(["PWM_MAIN_MIN1", "PWM_MAIN_FUNC4"]))
        self.assertEqual(sent, [(b"PWM_MAIN_MIN1", -1), (b"PWM_MAIN_FUNC4", -1)])
        self.assertTrue(w.request_motor_range_params())
        self.assertEqual(len(sent), 2 + 16)

    def test_the_widget_rescales_when_the_vehicle_range_arrives(self):
        from ui.motor_widget import MotorWidget
        w = MotorWidget()
        r = MotorRange()
        w.set_range(r)
        w.update_pwms([1500] * 4, age_s=0.1)
        self.assertAlmostEqual(w.bars[1].fraction(), 0.5)           # default scale
        for n in range(1, 5):
            w.on_motor_param(f"PWM_MAIN_MIN{n}", 1100)
            w.on_motor_param(f"PWM_MAIN_MAX{n}", 1900)
        self.assertAlmostEqual(w.bars[1].fraction(), 0.5)
        w.update_pwms([1100] * 4, age_s=0.1)
        self.assertEqual(w.bars[1].fraction(), 0.0)                 # idle now reads 0 %
        self.assertIn("from vehicle", w.lbl_scale.text())
        w.geometry._anim.stop()


class DiagnosticsTileTest(unittest.TestCase):
    def setUp(self):
        self.saved = (list(mr.SHARED.out_min), list(mr.SHARED.out_max),
                      list(mr.SHARED.out_dis), list(mr.SHARED.out_func))
        mr.SHARED.reset()

    def tearDown(self):
        (mr.SHARED.out_min, mr.SHARED.out_max,
         mr.SHARED.out_dis, mr.SHARED.out_func) = self.saved

    def _t(self, pwms, age=0.1, last=1.0):
        return SimpleNamespace(motor_pwms=pwms, motor_age=age, last_motor_time=last)

    def test_tile_uses_motor_order_and_the_shared_thresholds(self):
        from ui.value_grid import _motor_colour, _motor_fmt, DIM
        shared = vehicle_range(funcs=(102, 101, 103, 104))
        (mr.SHARED.out_min, mr.SHARED.out_max,
         mr.SHARED.out_dis, mr.SHARED.out_func) = (shared.out_min, shared.out_max,
                                                    shared.out_dis, shared.out_func)
        t = self._t([1500, 1100, 1100, 1100])      # output 1 = Motor 2
        self.assertEqual(_motor_fmt(1)(t), "1500 µs")
        self.assertIsNone(_motor_colour(1)(t))      # ordinary running value: plain white
        self.assertEqual(_motor_colour(0)(t), DIM)  # idle recedes to grey

    def test_a_stale_stream_shows_dashes_not_the_last_value(self):
        from ui.value_grid import _motor_fmt, _motor_colour, DIM
        t = self._t([1500, 1500, 1500, 1500], age=5.0)
        self.assertEqual(_motor_fmt(0)(t), "--")
        self.assertEqual(_motor_colour(0)(t), DIM)


if __name__ == "__main__":
    unittest.main()
