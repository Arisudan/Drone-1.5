"""Tests for STATUSTEXT reassembly and repeat suppression.

Both behaviours were found in a real flight log: a long PX4 rejection arrived
split in two ("...failures firs" / "t"), and an unresolved preflight fault
produced 25 identical lines in 45 seconds.

Hermetic: the two methods under test touch no socket and no Qt object beyond
the signal they emit, so the worker is constructed without ever connecting.
"""

import time
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from protocol.mavlink_worker import MAVLinkWorker


class Msg:
    """Minimal stand-in for a decoded STATUSTEXT."""

    def __init__(self, text, id=0, chunk_seq=0, severity=6):
        self.text = text
        self.id = id
        self.chunk_seq = chunk_seq
        self.severity = severity


def worker() -> MAVLinkWorker:
    # Never started, so no thread and no socket - only the parsing helpers.
    return MAVLinkWorker(host="127.0.0.1", port=1)


class ReassemblyTest(unittest.TestCase):
    def setUp(self):
        self.w = worker()

    def test_single_part_message_passes_straight_through(self):
        m = Msg("Ready to fly")
        self.assertEqual(
            self.w._reassemble_statustext(m, m.text, 6), "Ready to fly")

    def test_message_without_id_is_complete_by_definition(self):
        # Older autopilots omit id/chunk_seq entirely.
        m = Msg("legacy message", id=0, chunk_seq=0)
        self.assertEqual(
            self.w._reassemble_statustext(m, m.text, 6), "legacy message")

    def test_the_real_split_rejection_is_rejoined(self):
        # Verbatim from the flight log that prompted this fix.
        head = "Arming denied: Resolve system health failures firs"  # 50 chars
        tail = "t"
        self.assertEqual(len(head), 50)

        first = self.w._reassemble_statustext(Msg(head, id=7, chunk_seq=0), head, 4)
        self.assertIsNone(first, "a full-length chunk must wait for its tail")

        joined = self.w._reassemble_statustext(Msg(tail, id=7, chunk_seq=1), tail, 4)
        self.assertEqual(joined, "Arming denied: Resolve system health failures first")

    def test_three_chunk_message(self):
        a, b, c = "A" * 50, "B" * 50, "CC"
        self.assertIsNone(self.w._reassemble_statustext(Msg(a, id=9, chunk_seq=0), a, 6))
        self.assertIsNone(self.w._reassemble_statustext(Msg(b, id=9, chunk_seq=1), b, 6))
        self.assertEqual(
            self.w._reassemble_statustext(Msg(c, id=9, chunk_seq=2), c, 6), a + b + c)

    def test_two_interleaved_messages_do_not_mix(self):
        a0, b0 = "A" * 50, "B" * 50
        self.w._reassemble_statustext(Msg(a0, id=1, chunk_seq=0), a0, 6)
        self.w._reassemble_statustext(Msg(b0, id=2, chunk_seq=0), b0, 6)
        self.assertEqual(
            self.w._reassemble_statustext(Msg("a", id=1, chunk_seq=1), "a", 6), a0 + "a")
        self.assertEqual(
            self.w._reassemble_statustext(Msg("b", id=2, chunk_seq=1), "b", 6), b0 + "b")

    def test_a_missing_middle_chunk_does_not_emit_a_mangled_line(self):
        a = "A" * 50
        self.w._reassemble_statustext(Msg(a, id=3, chunk_seq=0), a, 6)
        # chunk 1 is lost; chunk 2 arrives
        self.assertIsNone(
            self.w._reassemble_statustext(Msg("end", id=3, chunk_seq=2), "end", 6))

    def test_abandoned_fragments_are_dropped_not_prepended(self):
        a = "A" * 50
        self.w._reassemble_statustext(Msg(a, id=4, chunk_seq=0), a, 6)
        # Age the pending entry past the 3 s abandonment window.
        self.w._statustext_parts[4]["t"] = time.time() - 5.0
        fresh = self.w._reassemble_statustext(Msg("fresh", id=4, chunk_seq=0), "fresh", 6)
        self.assertEqual(fresh, "fresh")


class RepeatSuppressionTest(unittest.TestCase):
    def setUp(self):
        self.w = worker()
        self.emitted = []
        self.w.statustext_received.connect(lambda t, s: self.emitted.append(t))

    def test_first_occurrence_is_always_shown(self):
        self.assertTrue(self.w._should_emit_statustext("Preflight Fail: High Accelerometer Bias"))

    def test_the_real_storm_collapses(self):
        msg = "Preflight Fail: High Accelerometer Bias"
        shown = sum(1 for _ in range(25) if self.w._should_emit_statustext(msg))
        # 25 identical messages in the log became 25 console lines; now one.
        self.assertEqual(shown, 1)

    def test_a_different_message_is_never_suppressed(self):
        self.w._should_emit_statustext("Preflight Fail: High Accelerometer Bias")
        self.w._should_emit_statustext("Preflight Fail: High Accelerometer Bias")
        self.assertTrue(self.w._should_emit_statustext("Arming denied"))

    def test_the_run_length_is_reported_when_it_ends(self):
        for _ in range(5):
            self.w._should_emit_statustext("spam")
        self.emitted.clear()
        self.w._should_emit_statustext("something else")
        self.assertTrue(any("repeated 4x" in t for t in self.emitted),
                        f"expected a repeat summary, got {self.emitted}")

    def test_a_persistent_fault_is_re_reported_periodically(self):
        msg = "Preflight Fail: High Accelerometer Bias"
        self.w._should_emit_statustext(msg)
        for _ in range(4):
            self.w._should_emit_statustext(msg)
        self.emitted.clear()
        # Push past the re-report interval without waiting for it.
        self.w._statustext_last_emit -= (self.w.STATUSTEXT_REPEAT_NOTICE_S + 1)
        self.w._should_emit_statustext(msg)
        self.assertTrue(any("repeated" in t for t in self.emitted),
                        "an unresolved fault must not go silent forever")

    def test_alternating_messages_both_get_through(self):
        for _ in range(3):
            self.assertTrue(self.w._should_emit_statustext("A"))
            self.assertTrue(self.w._should_emit_statustext("B"))


if __name__ == "__main__":
    unittest.main()
