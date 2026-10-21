"""Only the injector's VISION_POSITION_ESTIMATE may refresh last_vision_time.

Found on the bench (2026-10-19): with the Jetson's EV injector stopped for 15 s, PX4 dropped
cs_ev_pos internally but, at rest, held a "valid" fake position - ESTIMATOR_STATUS flags never
changed. The arbiter's last line of defence is the vision-age gate, and it never tripped:
PX4's own ODOMETRY (its EKF output, 10 Hz, independent of EV) was being counted as a vision
packet. A takeoff could have been allowed with no lidar position reaching the FC.

Hermetic: MAVLinkWorker is never started (same pattern as test_command_ack.py).
"""

import math
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from protocol.mavlink_worker import MAVLinkWorker


class Msg:
    def __init__(self, mtype, **kw):
        self._t = mtype
        self.__dict__.update(kw)

    def get_type(self):
        return self._t

    def get_srcSystem(self):
        return 1


def odometry():
    cov = [math.nan] * 21
    cov[0], cov[6] = 0.01, 0.01
    return Msg("ODOMETRY", x=0.0, y=0.0, z=0.0, pose_covariance=cov)


class VisionAgeSourceTest(unittest.TestCase):
    def setUp(self):
        self.w = MAVLinkWorker(host="127.0.0.1", port=1)
        self.w.target_system = 1

    def test_px4_odometry_is_not_a_vision_packet(self):
        self.w._handle_msg(odometry())
        self.assertEqual(self.w.telemetry.last_vision_time, 0.0)
        self.assertEqual(self.w.telemetry.vision_packets_count, 0)

    def test_vision_position_estimate_is(self):
        self.w._handle_msg(Msg("VISION_POSITION_ESTIMATE", x=0.0, y=0.0, z=0.0,
                               roll=0.0, pitch=0.0, yaw=0.0))
        self.assertGreater(self.w.telemetry.last_vision_time, 0.0)
        self.assertEqual(self.w.telemetry.vision_packets_count, 1)


if __name__ == "__main__":
    unittest.main()
