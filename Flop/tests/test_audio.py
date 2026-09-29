"""Audio alert service (core/audio.py).

Hermetic: no PyQt, and no sound is ever produced - every test either disables
the service outright or replaces the subprocess spawn, because a test suite
that beeps at a CI box (or at whoever is sitting next to the bench) is a test
suite people turn off.

The behaviours under test are the ones that make the feature usable rather
than merely present: it must stay silent when there is no backend, it must
never block the caller, and it must refuse to repeat the same alert.
"""

import time
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core.audio import (
    AudioAlerts, NullAudioAlerts, Severity, write_tone_files, _render_tone,
    _TONE_SPECS, SAMPLE_RATE,
)


class SeverityMappingTest(unittest.TestCase):
    def test_mavlink_severities_collapse_to_four_audible_levels(self):
        # MAV_SEVERITY 0..7 -> the coarser set an operator can tell apart.
        self.assertEqual(Severity.from_mavlink(0), Severity.ALARM)      # EMERGENCY
        self.assertEqual(Severity.from_mavlink(1), Severity.ALARM)      # ALERT
        self.assertEqual(Severity.from_mavlink(2), Severity.CRITICAL)   # CRITICAL
        self.assertEqual(Severity.from_mavlink(3), Severity.CRITICAL)   # ERROR
        self.assertEqual(Severity.from_mavlink(4), Severity.WARN)       # WARNING
        self.assertEqual(Severity.from_mavlink(6), Severity.INFO)       # INFO

    def test_ordering_is_meaningful(self):
        self.assertLess(Severity.INFO, Severity.WARN)
        self.assertLess(Severity.WARN, Severity.CRITICAL)
        self.assertLess(Severity.CRITICAL, Severity.ALARM)


class ToneSynthesisTest(unittest.TestCase):
    def test_every_severity_gets_a_distinct_tone_file(self):
        with TemporaryDirectory() as d:
            files = write_tone_files(Path(d))
            self.assertEqual(set(files), set(Severity))
            sizes = {s: p.stat().st_size for s, p in files.items()}
            for severity, size in sizes.items():
                self.assertGreater(size, 0, f"{severity.name} tone is empty")
            # Distinguishable by ear means distinguishable as audio: no two
            # severities may render to the same waveform.
            payloads = {p.read_bytes() for p in files.values()}
            self.assertEqual(len(payloads), len(files))

    def test_existing_files_are_reused_not_rewritten(self):
        with TemporaryDirectory() as d:
            first = write_tone_files(Path(d))
            stamps = {s: p.stat().st_mtime_ns for s, p in first.items()}
            time.sleep(0.01)
            write_tone_files(Path(d))
            for severity, path in first.items():
                self.assertEqual(path.stat().st_mtime_ns, stamps[severity])

    def test_rendered_length_matches_the_spec(self):
        spec, _floor = _TONE_SPECS[Severity.WARN]
        expected_frames = sum(int(SAMPLE_RATE * dur) + int(SAMPLE_RATE * gap)
                              for _f, dur, gap in spec)
        self.assertEqual(len(_render_tone(spec)), expected_frames * 2)  # 16-bit


class SilentFallbackTest(unittest.TestCase):
    """No backend, or switched off, must be a working configuration."""

    def test_disabled_service_accepts_every_call(self):
        a = AudioAlerts(enabled=False)
        self.assertFalse(a.available)
        self.assertFalse(a.say("anything", Severity.ALARM))
        self.assertFalse(a.alert(Severity.CRITICAL))
        self.assertIn("unavailable", a.backend_description())
        a.shutdown()          # idempotent, and safe with no worker thread
        a.shutdown()

    def test_null_service_is_silent(self):
        a = NullAudioAlerts()
        self.assertFalse(a.available)
        self.assertFalse(a.alert(Severity.ALARM))
        a.shutdown()


class _Cfg:
    enabled = True
    tones_enabled = True
    speech_enabled = True
    min_repeat_s = 10.0


class RateLimitTest(unittest.TestCase):
    """The de-duplication that keeps a repeating fault from becoming a siren."""

    def setUp(self):
        self._dir = TemporaryDirectory()
        self.played = []
        self.a = AudioAlerts(_Cfg(), sound_dir=Path(self._dir.name))
        if not self.a.available:
            self.skipTest("no audio backend on this machine")
        # Replace the subprocess spawn: the queue, the worker and the rate
        # limiter are what is under test, not ALSA.
        self.a._spawn = staticmethod(lambda argv: self.played.append(argv))

    def tearDown(self):
        self.a.shutdown()
        self._dir.cleanup()

    def test_same_key_is_suppressed_within_the_window(self):
        self.assertTrue(self.a.say("Battery low", Severity.WARN, key="batt"))
        for _ in range(5):
            self.assertFalse(self.a.say("Battery low", Severity.WARN, key="batt"))

    def test_different_keys_are_independent(self):
        self.assertTrue(self.a.say("Battery low", Severity.WARN, key="batt"))
        self.assertTrue(self.a.say("Link lost", Severity.CRITICAL, key="link"))

    def test_unkeyed_alerts_are_never_suppressed(self):
        # The caller has asserted these are distinct events.
        for _ in range(4):
            self.assertTrue(self.a.alert(Severity.INFO, key=None))

    def test_zero_floor_disables_suppression(self):
        cfg = _Cfg()
        cfg.min_repeat_s = 0.0
        self.a.set_config(cfg)
        for _ in range(3):
            self.assertTrue(self.a.say("go", Severity.INFO, key="k"))

    def test_muting_drops_alerts_and_unmuting_restores(self):
        self.a.set_muted(True)
        self.assertFalse(self.a.say("hello", Severity.WARN, key="a"))
        self.assertTrue(self.a.muted)
        self.a.set_muted(False)
        self.assertTrue(self.a.say("hello", Severity.WARN, key="b"))

    def test_toggle_reports_the_new_state(self):
        self.assertTrue(self.a.toggle_muted())
        self.assertFalse(self.a.toggle_muted())

    def test_disabling_via_config_silences_without_a_restart(self):
        cfg = _Cfg()
        cfg.enabled = False
        self.a.set_config(cfg)
        self.assertFalse(self.a.say("hello", Severity.WARN, key="z"))

    def test_submitting_never_blocks_the_caller(self):
        # Far more alerts than the queue holds; the excess must be dropped
        # rather than making the caller wait. On the GUI thread, waiting here
        # would freeze the whole station.
        cfg = _Cfg()
        cfg.min_repeat_s = 0.0
        self.a.set_config(cfg)
        started = time.monotonic()
        for _ in range(500):
            self.a.alert(Severity.INFO, key=None)
        self.assertLess(time.monotonic() - started, 1.0)

    def test_worker_survives_a_failing_player(self):
        def boom(argv):
            raise OSError("no such device")
        self.a._spawn = staticmethod(boom)
        cfg = _Cfg()
        cfg.min_repeat_s = 0.0
        self.a.set_config(cfg)
        for _ in range(3):
            self.a.alert(Severity.WARN, key=None)
        time.sleep(0.3)
        self.a._spawn = staticmethod(lambda argv: self.played.append(argv))
        self.assertTrue(self.a.say("still here", Severity.OK, key=None))


if __name__ == "__main__":
    unittest.main(verbosity=2)
