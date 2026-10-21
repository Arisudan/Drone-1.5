"""Tests for scripts/gcs/core/health.py - liveness and stall detection.

Hermetic: stdlib only. These matter because the health module is what stands
between "a worker died" and "the operator finds out".
"""

import time
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core.health import (
    ComponentWatchdog, EngineHealth, EngineStatus, HealthRegistry,
    get_registry,
)


class ConstructionTest(unittest.TestCase):
    def test_degrade_defaults_to_half_of_stall(self):
        h = EngineHealth("X", stall_after_s=4.0)
        self.assertAlmostEqual(h.degrade_after_s, 2.0)

    def test_rejects_degrade_at_or_past_stall(self):
        # A degrade threshold >= the stall threshold can never fire, which
        # would silently disable the early warning.
        with self.assertRaises(ValueError):
            EngineHealth("X", stall_after_s=1.0, degrade_after_s=1.0)
        with self.assertRaises(ValueError):
            EngineHealth("X", stall_after_s=1.0, degrade_after_s=2.0)

    def test_rejects_non_positive_stall(self):
        with self.assertRaises(ValueError):
            EngineHealth("X", stall_after_s=0.0)


class StatusTest(unittest.TestCase):
    def test_starts_uninitialized_and_never_stalls_before_first_status(self):
        h = EngineHealth("X", stall_after_s=0.01)
        self.assertEqual(h.snapshot().status, EngineStatus.UNINITIALIZED)
        time.sleep(0.02)
        self.assertFalse(h.check_stall())

    def test_ready_then_silence_reads_as_stalled(self):
        h = EngineHealth("X", stall_after_s=0.05, degrade_after_s=0.02)
        h.set_status(EngineStatus.READY)
        h.heartbeat()
        self.assertEqual(h.snapshot().status, EngineStatus.READY)
        time.sleep(0.03)
        self.assertEqual(h.snapshot().status, EngineStatus.DEGRADED)
        time.sleep(0.04)
        self.assertEqual(h.snapshot().status, EngineStatus.STALLED)

    def test_stall_latches_exactly_once(self):
        # The GUI sweeps at 1 Hz; without the latch the operator would get a
        # fresh error line and toast every single sweep.
        h = EngineHealth("X", stall_after_s=0.02)
        h.set_status(EngineStatus.READY)
        h.heartbeat()
        time.sleep(0.03)
        self.assertTrue(h.check_stall())
        self.assertFalse(h.check_stall())
        self.assertFalse(h.check_stall())

    def test_heartbeat_after_stall_recovers_and_rearms(self):
        h = EngineHealth("X", stall_after_s=0.02)
        h.set_status(EngineStatus.READY)
        h.heartbeat()
        time.sleep(0.03)
        self.assertTrue(h.check_stall())
        h.heartbeat()
        self.assertEqual(h.snapshot().status, EngineStatus.READY)
        # And a second stall must be reportable, not swallowed by the latch.
        time.sleep(0.03)
        self.assertTrue(h.check_stall())

    def test_stopped_and_failed_never_stall(self):
        # A deliberately stopped pump is not a fault. This is what keeps the
        # OFFBOARD pump quiet between paths instead of alarming every second.
        for quiet in (EngineStatus.STOPPED, EngineStatus.FAILED,
                      EngineStatus.STARTING):
            h = EngineHealth("X", stall_after_s=0.01)
            h.set_status(EngineStatus.READY)
            h.heartbeat()
            h.set_status(quiet)
            time.sleep(0.02)
            self.assertFalse(h.check_stall(), f"{quiet} must not stall")

    def test_restart_clears_previous_stall_latch(self):
        h = EngineHealth("X", stall_after_s=0.02)
        h.set_status(EngineStatus.READY)
        h.heartbeat()
        time.sleep(0.03)
        self.assertTrue(h.check_stall())
        h.set_status(EngineStatus.STOPPED)
        h.set_status(EngineStatus.READY)   # restarted
        time.sleep(0.03)
        self.assertTrue(h.check_stall())


class MetricsTest(unittest.TestCase):
    def test_beats_and_latency_percentile(self):
        h = EngineHealth("X", stall_after_s=5.0)
        h.set_status(EngineStatus.READY)
        for ms in range(1, 101):
            h.record_latency(float(ms))
            h.heartbeat()
        snap = h.snapshot()
        self.assertEqual(snap.beats, 100)
        self.assertAlmostEqual(snap.last_latency_ms, 100.0)
        # Nearest-rank p95 over 1..100 is 95.
        self.assertAlmostEqual(snap.p95_latency_ms, 95.0)

    def test_rate_is_measured_not_assumed(self):
        h = EngineHealth("X", stall_after_s=5.0)
        h.set_status(EngineStatus.READY)
        for _ in range(5):
            h.heartbeat()
            time.sleep(0.01)
        rate = h.snapshot().rate_hz
        # ~100 Hz nominal; generous bounds so a loaded CI box doesn't flake.
        self.assertGreater(rate, 20.0)
        self.assertLess(rate, 400.0)

    def test_rate_decays_to_zero_once_past_degrade_deadline(self):
        # Otherwise a dead worker keeps advertising its last healthy rate.
        h = EngineHealth("X", stall_after_s=0.06, degrade_after_s=0.02)
        h.set_status(EngineStatus.READY)
        for _ in range(3):
            h.heartbeat()
            time.sleep(0.005)
        self.assertGreater(h.snapshot().rate_hz, 0.0)
        time.sleep(0.03)
        self.assertEqual(h.snapshot().rate_hz, 0.0)

    def test_one_line_is_human_readable(self):
        h = EngineHealth("MapListener", stall_after_s=5.0)
        h.set_status(EngineStatus.READY)
        h.record_latency(12.0)
        h.heartbeat()
        text = h.snapshot().one_line()
        self.assertIn("MapListener", text)
        self.assertIn("ready", text)


class RegistryTest(unittest.TestCase):
    def setUp(self):
        self.reg = HealthRegistry()

    def test_register_get_and_snapshot(self):
        a = self.reg.register(EngineHealth("A"))
        self.reg.register(EngineHealth("B"))
        self.assertIs(self.reg.get("A"), a)
        self.assertIsNone(self.reg.get("nope"))
        self.assertEqual({s.name for s in self.reg.snapshot_all()}, {"A", "B"})

    def test_unhealthy_filters_to_what_the_operator_should_see(self):
        ok = self.reg.register(EngineHealth("OK", stall_after_s=5.0))
        bad = self.reg.register(EngineHealth("BAD", stall_after_s=0.02))
        for h in (ok, bad):
            h.set_status(EngineStatus.READY)
            h.heartbeat()
        time.sleep(0.03)
        names = {s.name for s in self.reg.unhealthy()}
        self.assertIn("BAD", names)
        self.assertNotIn("OK", names)

    def test_global_registry_is_a_singleton(self):
        self.assertIs(get_registry(), get_registry())


class WatchdogTest(unittest.TestCase):
    def test_poll_once_reports_and_restarts_once(self):
        reg = HealthRegistry()
        h = reg.register(EngineHealth("Worker", stall_after_s=0.02))
        h.set_status(EngineStatus.READY)
        h.heartbeat()

        seen = []
        restarts = []
        h.attach_restart_callback(lambda: restarts.append(1))
        dog = ComponentWatchdog(reg, poll_s=0.01, on_stall=seen.append)

        time.sleep(0.03)
        stalled = dog.poll_once()
        self.assertEqual([s.name for s in stalled], ["Worker"])
        self.assertEqual(len(seen), 1)
        self.assertEqual(len(restarts), 1)

        # Latched: a second sweep must not re-fire the restart.
        dog.poll_once()
        self.assertEqual(len(restarts), 1)

    def test_callback_exception_does_not_kill_the_sweep(self):
        reg = HealthRegistry()
        boom = reg.register(EngineHealth("Boom", stall_after_s=0.02))
        boom.set_status(EngineStatus.READY)
        boom.heartbeat()
        boom.attach_restart_callback(lambda: (_ for _ in ()).throw(RuntimeError("x")))
        dog = ComponentWatchdog(reg, poll_s=0.01)
        time.sleep(0.03)
        self.assertEqual([s.name for s in dog.poll_once()], ["Boom"])

    def test_background_thread_starts_and_stops(self):
        reg = HealthRegistry()
        h = reg.register(EngineHealth("T", stall_after_s=0.02))
        h.set_status(EngineStatus.READY)
        h.heartbeat()
        seen = []
        dog = ComponentWatchdog(reg, poll_s=0.01, on_stall=seen.append)
        dog.start()
        try:
            deadline = time.time() + 2.0
            while not seen and time.time() < deadline:
                time.sleep(0.01)
        finally:
            dog.stop()
        self.assertEqual(len(seen), 1)


if __name__ == "__main__":
    unittest.main()
