"""core/arbiter.py - every GCS command arbitration rule, hermetic (no Qt, no vehicle).

The four jobs (module docstring there): takeoff by frame, estimator gate, pilot wins,
motion limits. The window wiring is covered by test_arbitration_window.py.
"""
import math
import unittest

import _env  # noqa: F401

from core.arbiter import (EST_ATTITUDE, EST_POS_HORIZ_REL, EST_REQUIRED, EST_VELOCITY_HORIZ,
                          ArbiterLimits, HealthInputs, check_target, health, pilot_took_over,
                          stick_norm, takeoff_plan)

LIM = ArbiterLimits()


def healthy(**kw):
    base = dict(connected=True, est_flags=EST_REQUIRED | 32, est_pos_ratio=0.05, est_age_s=0.2,
                vision_age_s=0.1, position_age_s=0.1)
    base.update(kw)
    return HealthInputs(**base)


class HealthGateTest(unittest.TestCase):
    def test_healthy_estimator_passes(self):
        self.assertEqual(health(healthy(), LIM), (True, "estimator healthy"))

    def test_each_failure_is_named(self):
        cases = [
            (dict(connected=False), "no link"),
            (dict(est_age_s=9.0), "no ESTIMATOR_STATUS"),
            (dict(est_flags=EST_REQUIRED & ~EST_POS_HORIZ_REL), "no valid horizontal position"),
            (dict(est_flags=EST_REQUIRED & ~EST_VELOCITY_HORIZ), "no valid horizontal velocity"),
            (dict(est_flags=EST_REQUIRED & ~EST_ATTITUDE), "attitude not valid"),
            (dict(est_pos_ratio=1.3), "rejecting position"),
            (dict(vision_age_s=2.5), "lidar position not reaching"),
            (dict(vision_age_s=999.0), "never seen"),
            (dict(position_age_s=3.0), "local position stale"),
        ]
        for kw, words in cases:
            ok, reason = health(healthy(**kw), LIM)
            self.assertFalse(ok, kw)
            self.assertIn(words, reason, kw)

    def test_unknown_ratio_is_not_a_failure(self):
        self.assertTrue(health(healthy(est_pos_ratio=float("nan")), LIM)[0])

    def test_never_received_estimator_status_blocks(self):
        # The pre-arbiter GCS called EKF2 "ACTIVE" from vision packets alone.
        self.assertFalse(health(healthy(est_flags=0, est_age_s=999.0), LIM)[0])


class TakeoffByFrameTest(unittest.TestCase):
    def test_origin_means_nav_takeoff_to_amsl_plus_height(self):
        p = takeoff_plan(1.0, True, 10.27, 0.1, -0.2, -0.05, 0.0)
        self.assertEqual(p.method, "nav_takeoff")
        self.assertAlmostEqual(p.param7, 11.27)

    def test_no_origin_means_offboard_climb_in_place(self):
        p = takeoff_plan(1.0, False, None, 0.1, -0.2, -0.05, 37.0)
        self.assertEqual(p.method, "offboard")
        self.assertEqual((p.x, p.y), (0.1, -0.2))
        self.assertAlmostEqual(p.z, -1.05)               # NED: 1 m above where it stands
        self.assertEqual(p.yaw_deg, 37.0)                # current heading, never 0

    def test_origin_without_altitude_is_not_trusted(self):
        self.assertEqual(takeoff_plan(1.0, True, None, 0, 0, 0, 0).method, "offboard")
        self.assertEqual(takeoff_plan(1.0, True, float("nan"), 0, 0, 0, 0).method, "offboard")


class PilotWinsTest(unittest.TestCase):
    def test_stick_scale(self):
        self.assertEqual(stick_norm(1500), 0.0)
        self.assertEqual(stick_norm(2000), 1.0)
        self.assertEqual(stick_norm(1000), -1.0)
        self.assertEqual(stick_norm(0), 0.0)             # channel missing
        self.assertEqual(stick_norm(65535), 0.0)         # MAVLink "unused"

    def test_centred_sticks_leave_the_gcs_in_control(self):
        self.assertIsNone(pilot_took_over(0.05, -0.08, 0.1, -1.0, -1.0, LIM))

    def test_ten_percent_of_travel_is_a_takeover(self):
        # 10 % of full travel = 100 us = 0.2 on the -1..1 scale
        self.assertIsNone(pilot_took_over(stick_norm(1595), 0, 0, -1, -1, LIM))
        self.assertEqual(pilot_took_over(stick_norm(1610), 0, 0, -1, -1, LIM), "roll stick moved")
        self.assertEqual(pilot_took_over(0, stick_norm(1390), 0, -1, -1, LIM), "pitch stick moved")
        self.assertEqual(pilot_took_over(0, 0, stick_norm(1700), -1, -1, LIM), "yaw stick moved")

    def test_throttle_is_judged_against_where_it_was(self):
        low = stick_norm(1003)
        self.assertIsNone(pilot_took_over(0, 0, 0, low, low, LIM))          # parked at the bottom
        self.assertEqual(pilot_took_over(0, 0, 0, stick_norm(1150), low, LIM), "throttle moved")
        self.assertIsNone(pilot_took_over(0, 0, 0, stick_norm(1500), None, LIM))


class MotionLimitsTest(unittest.TestCase):
    HOME = (0.0, 0.0, 0.0)

    def test_small_move_inside_the_box_passes(self):
        self.assertTrue(check_target((0, 0, -1.0), (1.0, 1.0, -1.2), self.HOME, LIM)[0])

    def test_steps_are_limited(self):
        ok, why = check_target((0, 0, -1.0), (1.5, 1.5, -1.0), self.HOME, LIM)
        self.assertFalse(ok)
        self.assertIn("horizontal step 2.12", why)
        ok, why = check_target((0, 0, -0.5), (0, 0, -1.8), self.HOME, LIM)
        self.assertFalse(ok)
        self.assertIn("vertical step", why)

    def test_ceiling_is_above_the_take_off_point(self):
        ok, why = check_target((0, 0, -1.5), (0, 0, -2.3), self.HOME, LIM)
        self.assertFalse(ok)
        self.assertIn("ceiling", why)
        # take-off point 0.3 m below the frame origin: 2.0 m above IT is still allowed
        self.assertTrue(check_target((0, 0, -1.5), (0, 0, -1.7), (0, 0, 0.3), LIM)[0])

    def test_keep_in_box(self):
        ok, why = check_target((4.5, 0, -1), (5.6, 0, -1), self.HOME, LIM)
        self.assertFalse(ok)
        self.assertIn("box", why)

    def test_route_waypoints_skip_only_the_step_limit(self):
        self.assertTrue(check_target((0, 0, -1), (4.0, 3.0, -1), self.HOME, LIM, check_step=False)[0])
        self.assertFalse(check_target((0, 0, -1), (7.0, 0, -1), self.HOME, LIM, check_step=False)[0])

    def test_without_a_home_only_steps_apply(self):
        self.assertTrue(check_target((9, 9, -5), (10, 9, -5), None, LIM)[0])

    def test_limits_validate(self):
        ArbiterLimits().validate()
        with self.assertRaises(ValueError):
            ArbiterLimits(stick_takeover=0.9).validate()
        with self.assertRaises(ValueError):
            ArbiterLimits(ceiling_m=0).validate()


if __name__ == "__main__":
    unittest.main(verbosity=2)
