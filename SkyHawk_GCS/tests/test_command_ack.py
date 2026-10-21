"""Tests for COMMAND_ACK dispatch-dedup, keyed on (token, result_code).

Found via code audit: a single dispatch can legitimately receive two
different COMMAND_ACKs - MAV_RESULT_IN_PROGRESS (5) first, then the real
final result once PX4 finishes - and the dedup that exists to drop true
wire-level retransmit duplicates was keyed on the dispatch token alone, so
that second, genuinely different ACK was silently swallowed as a "duplicate"
of the first. The operator never learned whether the command actually
succeeded.

Hermetic: MAVLinkWorker is never started, so this touches no socket and no
Qt object beyond the signal it emits (same pattern as test_statustext.py).
"""

import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from protocol.mavlink_worker import MAVLinkWorker


class AckMsg:
    """Minimal stand-in for a decoded COMMAND_ACK."""

    def __init__(self, command, result):
        self.command = command
        self.result = result

    def get_type(self):
        return "COMMAND_ACK"

    def get_srcSystem(self):
        return 1


def worker() -> MAVLinkWorker:
    w = MAVLinkWorker(host="127.0.0.1", port=1)
    w.target_system = 1
    return w


class CommandAckDedupTest(unittest.TestCase):
    def setUp(self):
        self.w = worker()
        self.received = []
        self.w.command_ack_received.connect(lambda *a: self.received.append(a))

    def test_in_progress_then_final_ack_are_both_reported(self):
        self.w._begin_command_dispatch(400)
        self.w._handle_msg(AckMsg(400, 5))   # IN_PROGRESS
        self.w._handle_msg(AckMsg(400, 0))   # the real final ACK: ACCEPTED
        self.assertEqual(len(self.received), 2)
        self.assertEqual(self.received[0][1], 5)
        self.assertEqual(self.received[1][1], 0)

    def test_a_true_retransmit_duplicate_is_still_suppressed(self):
        self.w._begin_command_dispatch(400)
        self.w._handle_msg(AckMsg(400, 0))
        self.w._handle_msg(AckMsg(400, 0))   # identical wire retransmit
        self.assertEqual(len(self.received), 1)

    def test_a_fresh_dispatch_of_the_same_command_reports_again(self):
        self.w._begin_command_dispatch(400)
        self.w._handle_msg(AckMsg(400, 0))
        self.w._begin_command_dispatch(400)  # new dispatch, fresh token
        self.w._handle_msg(AckMsg(400, 0))
        self.assertEqual(len(self.received), 2)


if __name__ == "__main__":
    unittest.main()
