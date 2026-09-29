"""
Bench motor test must use MAV_CMD_ACTUATOR_TEST (310), not DO_MOTOR_TEST (209).

PX4 v1.14+ Commander has no handler for 209 and ACKs every one with
MAV_RESULT_UNSUPPORTED - the GCS motor page did nothing on a v1.17 airframe.
These tests pin the exact packet so that cannot quietly come back.
"""

import math
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from protocol.mavlink_worker import MAVLinkWorker, MAV_CMD_ACTUATOR_TEST
from core.telemetry import MAV_CMD_NAMES


class _Mav:
    def __init__(self):
        self.sent = []

    def command_long_send(self, *args):
        self.sent.append(args)


class _Master:
    def __init__(self):
        self.mav = _Mav()


def _worker():
    w = MAVLinkWorker(host="127.0.0.1", port=1)
    w.master = _Master()
    w.target_system, w.target_component = 1, 1
    return w


class ActuatorTestCommandTest(unittest.TestCase):
    def test_uses_actuator_test_not_do_motor_test(self):
        w = _worker()
        self.assertTrue(w.test_actuator(3, 15.0))
        (_, _, cmd, _conf, p1, p2, _p3, _p4, p5, _p6, _p7), = w.master.mav.sent
        self.assertEqual(cmd, 310)
        self.assertEqual(MAV_CMD_ACTUATOR_TEST, 310)
        self.assertAlmostEqual(p1, 0.15)         # value is 0..1, not percent
        self.assertAlmostEqual(p2, w.MOTOR_TEST_DEFAULT_TIMEOUT_S)
        self.assertEqual(p5, 3.0)                # ACTUATOR_OUTPUT_FUNCTION_MOTOR3

    def test_throttle_is_clamped(self):
        w = _worker()
        w.test_actuator(1, 250.0)
        p1 = w.master.mav.sent[0][4]
        self.assertAlmostEqual(p1, w.MOTOR_TEST_MAX_THROTTLE_PCT / 100.0)
        self.assertFalse(math.isnan(p1))

    def test_stop_releases_every_channel(self):
        w = _worker()
        w.stop_all_motor_tests(motor_count=4)
        sent = w.master.mav.sent
        self.assertEqual([a[8] for a in sent], [1.0, 2.0, 3.0, 4.0])
        for a in sent:
            self.assertEqual(a[2], 310)
            self.assertEqual(a[4], 0.0)
            self.assertLessEqual(a[5], 0.0)      # timeout <= 0 -> RELEASE_CONTROL

    def test_ack_names_are_readable(self):
        self.assertEqual(MAV_CMD_NAMES[310], "ACTUATOR_TEST")
        self.assertEqual(MAV_CMD_NAMES[209], "DO_MOTOR_TEST")


if __name__ == "__main__":
    unittest.main()
