"""EKF position-uncertainty ring on the tactical map.

The decoding tests are hermetic; the canvas tests need PyQt5.

Why this matters: indoors, the vehicle's position is VIO + SLAM through EKF2.
When tracking degrades the drone icon keeps moving smoothly while the real
aircraft drifts, and the planner's 0.25 m wall margin is only as good as the
position it is applied to. The ring shows how much of that margin position
error has already eaten.
"""

import math
import time
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core.telemetry import TelemetrySnapshot

AUTOPILOT, VIO = 8, 3    # MAV_ESTIMATOR_TYPE_*


class FakeOdometry:
    def __init__(self, var_n, var_e, estimator_type=AUTOPILOT):
        self.estimator_type = estimator_type
        cov = [float("nan")] * 21
        cov[0], cov[6] = var_n, var_e      # PX4 v1.17 streams/ODOMETRY.hpp layout
        self.pose_covariance = cov


class DecodeTest(unittest.TestCase):
    def test_starts_unknown_not_zero(self):
        t = TelemetrySnapshot()
        self.assertTrue(math.isnan(t.pos_var_n))
        self.assertTrue(math.isnan(t.pos_var_e))

    def test_autopilot_odometry_is_decoded(self):
        t = TelemetrySnapshot()
        t.update_position_covariance(FakeOdometry(0.004, 0.001))
        self.assertAlmostEqual(t.pos_var_n, 0.004)
        self.assertAlmostEqual(t.pos_var_e, 0.001)
        self.assertGreater(t.pos_var_time, 0)

    def test_companion_vio_odometry_is_ignored(self):
        # It can reach the GCS through mavlink-router, but it describes the
        # tracker, not the estimate the vehicle flies on.
        t = TelemetrySnapshot()
        t.update_position_covariance(FakeOdometry(9.0, 9.0, estimator_type=VIO))
        self.assertTrue(math.isnan(t.pos_var_n))

    def test_non_finite_or_negative_values_keep_the_last_good_one(self):
        t = TelemetrySnapshot()
        t.update_position_covariance(FakeOdometry(0.004, 0.001))
        t.update_position_covariance(FakeOdometry(float("nan"), 0.001))
        t.update_position_covariance(FakeOdometry(-1.0, 0.001))
        self.assertAlmostEqual(t.pos_var_n, 0.004)

    def test_clone_carries_the_variance(self):
        t = TelemetrySnapshot()
        t.update_position_covariance(FakeOdometry(0.004, 0.001))
        self.assertAlmostEqual(t.clone().pos_var_n, 0.004)


try:
    from PyQt5.QtWidgets import QApplication
    HAVE_QT = True
except Exception:
    HAVE_QT = False


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class CanvasStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        from ui import slam_map_widget as m
        cls.m = m

    def setUp(self):
        self.c = self.m.SLAMMapCanvas()
        self.c.planner.robot_radius_m = 0.25

    def _state(self, sigma, age=0.0):
        now = time.time()
        self.c.set_position_uncertainty(sigma ** 2, sigma ** 2, now - age)
        return self.c.uncertainty_state(now)

    def test_unknown_until_a_covariance_arrives(self):
        self.assertEqual(self.c.uncertainty_state()[0], "unknown")

    def test_radius_is_the_95_percent_region(self):
        _, r95 = self._state(0.04)
        self.assertAlmostEqual(r95, 2.4477 * 0.04, places=4)

    def test_thresholds_follow_the_planner_radius(self):
        k = self.m.POS_CONF_K
        self.assertEqual(self._state(0.10 / k)[0], "ok")        # r95 0.10 < 0.125
        self.assertEqual(self._state(0.20 / k)[0], "warn")      # 0.125 < 0.20 < 0.25
        self.assertEqual(self._state(0.30 / k)[0], "danger")    # 0.30 > 0.25
        self.c.planner.robot_radius_m = 0.40
        self.assertEqual(self._state(0.30 / k)[0], "warn",
                         "a wider margin tolerates more error")

    def test_larger_axis_decides(self):
        now = time.time()
        self.c.set_position_uncertainty(0.0001, (0.30 / self.m.POS_CONF_K) ** 2, now)
        self.assertEqual(self.c.uncertainty_state(now)[0], "danger")

    def test_goes_stale_rather_than_vanishing(self):
        state, r95 = self._state(0.05, age=self.m.POS_UNCERT_STALE_S + 1)
        self.assertEqual(state, "stale")
        self.assertGreater(r95, 0)

    def test_renders_in_every_state(self):
        from PyQt5.QtGui import QImage
        self.c.resize(300, 300)
        for sigma, age in ((0.02, 0), (0.07, 0), (0.2, 0), (0.07, 9)):
            self._state(sigma, age)
            img = QImage(300, 300, QImage.Format_RGB32)
            img.fill(0)
            self.c.render(img)          # must not raise


if __name__ == "__main__":
    unittest.main(verbosity=2)
