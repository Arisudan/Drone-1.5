"""Disarm safety: a vehicle that is off the floor must be landed, never cut.

PX4's `landed_state` can say ON_GROUND while the vehicle is physically up (held
in the hand, some manual modes, just after lift-off). Disarm therefore also
consults the downward rangefinder, normal (non-forced) disarm is used on the
ground, and a refused AUTO.LAND is reported instead of waited on silently.
"""

import time
import unittest
from unittest import mock

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core.telemetry import (
    AIRBORNE_HEIGHT_M, RANGE_MAX_AGE_S, TelemetrySnapshot,
)


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def snap(armed=True, landed=1, range_m=None, age=0.0, z=0.0):
    t = TelemetrySnapshot()
    t.armed = armed
    t.landed_state = landed
    t.z = z
    if range_m is not None:
        t.range_m = range_m
        t.last_range_time = time.time() - age
    return t


class RangefinderParseTest(unittest.TestCase):
    def test_downward_reading_is_converted_to_metres(self):
        t = TelemetrySnapshot()
        t.update_distance_sensor(Msg(orientation=25, current_distance=82, max_distance=3000))
        self.assertAlmostEqual(t.range_m, 0.82)
        self.assertAlmostEqual(t.height_over_floor(), 0.82)

    def test_other_orientations_are_ignored(self):
        t = TelemetrySnapshot()
        t.update_distance_sensor(Msg(orientation=0, current_distance=300, max_distance=3000))
        self.assertIsNone(t.height_over_floor())

    def test_out_of_range_reading_is_unknown_not_a_height(self):
        t = TelemetrySnapshot()
        t.update_distance_sensor(Msg(orientation=25, current_distance=3000, max_distance=3000))
        self.assertIsNone(t.height_over_floor())

    def test_unknown_is_never_zero(self):
        self.assertIsNone(TelemetrySnapshot().height_over_floor())

    def test_stale_reading_is_not_trusted(self):
        t = snap(range_m=1.0, age=RANGE_MAX_AGE_S + 1.0)
        self.assertIsNone(t.height_over_floor())


class NeedsControlledLandingTest(unittest.TestCase):
    def test_px4_in_air_states_always_land(self):
        for state in (2, 3, 4):
            self.assertTrue(snap(landed=state).needs_controlled_landing, state)

    def test_on_ground_flag_but_held_up_is_still_a_landing(self):
        # The reported bug: ON_GROUND while 0.8 m up used to be force-disarmed.
        self.assertTrue(snap(landed=1, range_m=0.8).needs_controlled_landing)

    def test_on_ground_and_close_to_floor_is_a_plain_disarm(self):
        self.assertFalse(snap(landed=1, range_m=0.04).needs_controlled_landing)
        self.assertFalse(snap(landed=1, range_m=AIRBORNE_HEIGHT_M).needs_controlled_landing)

    def test_just_above_threshold_lands(self):
        self.assertTrue(snap(landed=1, range_m=AIRBORNE_HEIGHT_M + 0.01).needs_controlled_landing)

    def test_no_rangefinder_falls_back_to_px4_flag(self):
        self.assertFalse(snap(landed=1).needs_controlled_landing)
        # an EKF z far from the floor must NOT turn a grounded vehicle into a landing
        self.assertFalse(snap(landed=1, z=-0.82).needs_controlled_landing)

    def test_undefined_state_keeps_the_altitude_guess(self):
        self.assertTrue(snap(landed=0, z=-1.0).needs_controlled_landing)
        self.assertFalse(snap(landed=0, z=0.0).needs_controlled_landing)

    def test_stale_range_does_not_override_px4(self):
        self.assertFalse(snap(landed=1, range_m=2.0, age=10.0).needs_controlled_landing)

    def test_disarmed_never_needs_landing(self):
        self.assertFalse(snap(armed=False, landed=2, range_m=2.0).needs_controlled_landing)


class OnGroundForDisarmTest(unittest.TestCase):
    def test_requires_px4_on_ground(self):
        for state in (0, 2, 3, 4):
            self.assertFalse(snap(landed=state, range_m=0.03).is_on_ground_for_disarm(), state)

    def test_on_ground_and_near_floor(self):
        self.assertTrue(snap(landed=1, range_m=0.05).is_on_ground_for_disarm())

    def test_on_ground_flag_but_still_up_is_not_safe_to_cut(self):
        self.assertFalse(snap(landed=1, range_m=0.5).is_on_ground_for_disarm())

    def test_no_range_trusts_px4(self):
        self.assertTrue(snap(landed=1).is_on_ground_for_disarm())


try:
    from controllers.flight_commands import FlightCommandsMixin
    HAVE_QT = True
except Exception:  # pragma: no cover - PyQt5 absent
    HAVE_QT = False


class Sink:
    def __init__(self):
        self.lines = []

    def __getattr__(self, name):
        if name.startswith("log_") or name == "show_message":
            return lambda *a, **k: self.lines.append((name, a))
        raise AttributeError(name)


@unittest.skipUnless(HAVE_QT, "PyQt5 not available")
class DisarmHandlerTest(unittest.TestCase):
    def make_host(self, telem):
        class Host(FlightCommandsMixin):
            pass
        h = Host()
        h.worker = mock.Mock()
        h.worker.isRunning.return_value = True
        h.console = Sink()
        h.page_terminal = Sink()
        h.toast = Sink()
        h.exec_tracker = mock.Mock()
        h.page_slam = mock.Mock()
        h.offboard_pump_timer = mock.Mock()
        h._begin_path_generation = mock.Mock()
        h.last_telemetry = telem
        h._pending_autodisarm_after_land = False
        h.path_in_progress = h.path_paused = h.path_awaiting_climb = False
        h.climb_start_time = 0.0
        h.active_waypoints = []
        h.current_wpt_idx = 0
        return h

    def test_held_up_with_on_ground_flag_lands_instead_of_cutting(self):
        h = self.make_host(snap(landed=1, range_m=0.8))
        with mock.patch("controllers.flight_commands.QTimer.singleShot"):
            h._cmd_disarm()
        h.worker.set_mode.assert_called_once_with("AUTO.LAND")
        h.worker.disarm.assert_not_called()
        self.assertTrue(h._pending_autodisarm_after_land)

    def test_on_the_floor_disarms_without_force(self):
        h = self.make_host(snap(landed=1, range_m=0.04))
        h._cmd_disarm()
        h.worker.disarm.assert_called_once_with(force=False)
        h.worker.set_mode.assert_not_called()

    def test_land_is_verified_after_a_delay(self):
        h = self.make_host(snap(landed=2, range_m=1.0))
        with mock.patch("controllers.flight_commands.QTimer.singleShot") as ss:
            h._cmd_disarm()
        ss.assert_called_once()
        self.assertEqual(ss.call_args[0][0], h.LAND_VERIFY_MS)

    def test_refused_land_is_reported_and_never_falls_back_to_a_cutoff(self):
        t = snap(landed=2, range_m=1.0)
        t.flight_mode = "STABILIZED"
        h = self.make_host(t)
        h._pending_autodisarm_after_land = True
        h._verify_land_mode()
        self.assertFalse(h._pending_autodisarm_after_land)
        h.worker.disarm.assert_not_called()
        self.assertTrue(any(n == "log_error" for n, _ in h.console.lines))

    def test_accepted_land_keeps_waiting_quietly(self):
        t = snap(landed=4, range_m=0.6)
        t.flight_mode = "AUTO.LAND"
        h = self.make_host(t)
        h._pending_autodisarm_after_land = True
        h._verify_land_mode()
        self.assertTrue(h._pending_autodisarm_after_land)
        self.assertEqual(h.console.lines, [])

    def test_verify_does_nothing_once_disarmed(self):
        t = snap(armed=False, landed=1)
        h = self.make_host(t)
        h._pending_autodisarm_after_land = True
        h._verify_land_mode()
        self.assertFalse(h._pending_autodisarm_after_land)
        self.assertEqual(h.console.lines, [])


if __name__ == "__main__":
    unittest.main()
