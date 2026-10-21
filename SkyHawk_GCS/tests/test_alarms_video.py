"""Alarm list (core/alarms.py), video health (core/video_health.py), and their
Qt surfaces (ui/alarm_banner.py, the health line on VideoFeedWidget).

Time is injected everywhere; no camera, no network.
"""

import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core.alarms import AlarmLevel, AlarmManager, format_elapsed
from core import video_health as vh
from core.video_health import VideoHealthMonitor


class AlarmManagerTest(unittest.TestCase):
    def setUp(self):
        self.t = 100.0
        self.a = AlarmManager(clock=lambda: self.t)

    def test_new_alarm_reports_true_repeat_reports_false(self):
        self.assertTrue(self.a.raise_alarm("link", AlarmLevel.CRITICAL, "Link lost"))
        self.assertFalse(self.a.raise_alarm("link", AlarmLevel.CRITICAL, "Link lost"))
        self.assertEqual(len(self.a), 1)

    def test_escalation_reports_true_and_unacks(self):
        self.a.raise_alarm("batt", AlarmLevel.WARN, "low")
        self.a.acknowledge("batt")
        self.assertTrue(self.a.raise_alarm("batt", AlarmLevel.CRITICAL, "critical"))
        self.assertFalse(self.a.active()[0].acked)

    def test_deescalation_keeps_ack_and_does_not_notify(self):
        self.a.raise_alarm("batt", AlarmLevel.CRITICAL, "critical")
        self.a.acknowledge("batt")
        self.assertFalse(self.a.raise_alarm("batt", AlarmLevel.WARN, "low"))
        self.assertTrue(self.a.active()[0].acked)

    def test_ack_does_not_clear(self):
        self.a.raise_alarm("link", AlarmLevel.CRITICAL, "x")
        self.a.acknowledge_all()
        self.assertIn("link", self.a)
        self.assertEqual(self.a.unacked(), [])

    def test_order_unacked_then_level_then_newest(self):
        self.a.raise_alarm("w_old", AlarmLevel.WARN, "w1")
        self.t += 1
        self.a.raise_alarm("w_new", AlarmLevel.WARN, "w2")
        self.t += 1
        self.a.raise_alarm("crit", AlarmLevel.CRITICAL, "c")
        self.a.raise_alarm("crit_acked", AlarmLevel.CRITICAL, "c2")
        self.a.acknowledge("crit_acked")
        self.assertEqual([x.key for x in self.a.active()],
                         ["crit", "w_new", "w_old", "crit_acked"])
        self.assertEqual(self.a.top().key, "crit")

    def test_clear_moves_to_history_and_reraise_is_new(self):
        self.a.raise_alarm("link", AlarmLevel.CRITICAL, "x")
        self.a.acknowledge("link")
        self.assertTrue(self.a.clear("link"))
        self.assertFalse(self.a.clear("link"))
        self.assertEqual([h.key for h in self.a.history()], ["link"])
        self.assertTrue(self.a.raise_alarm("link", AlarmLevel.CRITICAL, "x"))
        self.assertFalse(self.a.active()[0].acked)

    def test_set_condition_is_level_triggered(self):
        self.assertTrue(self.a.set_condition("v", True, AlarmLevel.WARN, "frozen"))
        self.assertFalse(self.a.set_condition("v", True, AlarmLevel.WARN, "frozen"))
        self.assertFalse(self.a.set_condition("v", False))
        self.assertNotIn("v", self.a)

    def test_revision_only_moves_on_visible_change(self):
        self.a.raise_alarm("k", AlarmLevel.WARN, "t")
        r = self.a.revision
        self.a.raise_alarm("k", AlarmLevel.WARN, "t")
        self.assertEqual(self.a.revision, r)
        self.a.raise_alarm("k", AlarmLevel.WARN, "t2")
        self.assertGreater(self.a.revision, r)

    def test_history_is_bounded(self):
        a = AlarmManager(clock=lambda: 0.0, history_limit=3)
        for i in range(10):
            a.raise_alarm(f"k{i}", AlarmLevel.WARN, "x")
            a.clear(f"k{i}")
        self.assertEqual(len(a.history()), 3)


class VideoHealthTest(unittest.TestCase):
    def setUp(self):
        self.t = 0.0
        self.m = VideoHealthMonitor(freeze_after_s=2.0, degraded_fps=8.0, clock=lambda: self.t)

    def _frames(self, n, dt, latency=0.02):
        for _ in range(n):
            self.t += dt
            self.m.on_frame(captured_at=self.t - latency)

    def test_idle_before_start(self):
        self.assertEqual(self.m.snapshot().state, vh.IDLE)

    def test_connecting_then_no_signal(self):
        self.m.start()
        self.assertEqual(self.m.snapshot().state, vh.CONNECTING)
        self.t += 2.5
        self.assertEqual(self.m.snapshot().state, vh.NO_SIGNAL)

    def test_placeholder_frames_are_never_live(self):
        self.m.start()
        for _ in range(20):
            self.t += 0.05
            self.m.on_frame(self.t, real=False)
        self.assertNotEqual(self.m.snapshot().state, vh.LIVE)

    def test_live_reports_fps_latency_and_low_jitter(self):
        self.m.start()
        self._frames(40, 1 / 30.0, latency=0.02)
        h = self.m.snapshot()
        self.assertEqual(h.state, vh.LIVE)
        self.assertAlmostEqual(h.fps, 30.0, delta=1.0)
        self.assertAlmostEqual(h.latency_ms, 20.0, delta=1.0)
        self.assertLess(h.jitter_ms, 1.0)

    def test_slow_feed_is_degraded(self):
        self.m.start()
        self._frames(6, 0.4)
        self.assertEqual(self.m.snapshot().state, vh.DEGRADED)

    def test_freeze_detected_without_any_new_frame(self):
        self.m.start()
        self._frames(30, 1 / 30.0)
        self.t += 2.5          # frames simply stop - no call into the monitor
        h = self.m.snapshot()
        self.assertEqual(h.state, vh.FROZEN)
        self.assertAlmostEqual(h.age_s, 2.5, delta=0.1)
        self.assertIn("no frame", h.text())

    def test_recovers_after_freeze(self):
        self.m.start()
        self._frames(10, 0.05)
        self.t += 3.0
        self.assertEqual(self.m.snapshot().state, vh.FROZEN)
        self._frames(20, 1 / 30.0)
        self.assertEqual(self.m.snapshot().state, vh.LIVE)

    def test_stop_returns_to_idle(self):
        self.m.start()
        self._frames(5, 0.05)
        self.m.stop()
        self.assertEqual(self.m.snapshot().state, vh.IDLE)


class AlarmBannerUiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_hidden_when_empty_then_shows_top_and_count(self):
        from ui.alarm_banner import AlarmBanner
        from PyQt5.QtWidgets import QWidget
        host = QWidget()
        a = AlarmManager()
        b = AlarmBanner(a, host)
        host.show()
        self.assertFalse(b.isVisible())
        a.raise_alarm("w", AlarmLevel.WARN, "Video feed: FROZEN")
        a.raise_alarm("c", AlarmLevel.CRITICAL, "Link lost")
        b.refresh()
        self.assertTrue(b.isVisible())
        self.assertEqual(b.lbl_text.text(), "Link lost")
        self.assertEqual(b.lbl_more.text(), "+1")
        self.assertTrue(b.btn_ack.isEnabled())
        self.assertFalse(b.lbl_tag.isVisibleTo(b))

    def test_ack_button_signal_and_neutral_after_ack(self):
        from ui.alarm_banner import AlarmBanner
        a = AlarmManager()
        b = AlarmBanner(a)
        hits = []
        b.ack_all_requested.connect(lambda: hits.append(1))
        a.raise_alarm("c", AlarmLevel.CRITICAL, "Link lost")
        b.refresh()
        b.btn_ack.click()
        self.assertEqual(hits, [1])
        a.acknowledge_all()
        b.refresh()
        self.assertFalse(b.btn_ack.isEnabled())
        self.assertTrue(b.lbl_tag.isVisibleTo(b))
        self.assertEqual(b.lbl_level.text(), "CRITICAL")

    def test_hides_again_when_cleared(self):
        from ui.alarm_banner import AlarmBanner
        from PyQt5.QtWidgets import QWidget
        host = QWidget()
        a = AlarmManager()
        b = AlarmBanner(a, host)
        host.show()
        a.raise_alarm("c", AlarmLevel.CRITICAL, "x")
        b.refresh()
        self.assertTrue(b.isVisible())
        a.clear("c")
        b.refresh()
        self.assertFalse(b.isVisible())


class AlarmTimerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_format_elapsed(self):
        self.assertEqual(format_elapsed(0), "0:00")
        self.assertEqual(format_elapsed(2.9), "0:02")
        self.assertEqual(format_elapsed(75), "1:15")
        self.assertEqual(format_elapsed(3725), "1:02:05")
        self.assertEqual(format_elapsed(-4), "0:00")

    def test_elapsed_is_computed_not_stored(self):
        t = [10.0]
        a = AlarmManager(clock=lambda: t[0])
        a.raise_alarm("v", AlarmLevel.WARN, "Video feed: NO SIGNAL")
        t[0] += 7.0
        self.assertAlmostEqual(a.elapsed(a.top()), 7.0)
        t[0] += 60.0
        self.assertAlmostEqual(a.elapsed(a.top()), 67.0)

    def test_banner_timer_advances_without_reraising(self):
        from PyQt5.QtWidgets import QWidget
        from ui.alarm_banner import AlarmBanner
        t = [0.0]
        a = AlarmManager(clock=lambda: t[0])
        host = QWidget()
        b = AlarmBanner(a, host)
        host.show()
        a.raise_alarm("v", AlarmLevel.WARN, "Video feed: NO SIGNAL", "hint")
        b.refresh()
        self.assertEqual(b.lbl_timer.text(), "0:00")
        t[0] += 134.0
        b.refresh_timer()                      # what the 1 s QTimer calls
        self.assertEqual(b.lbl_timer.text(), "2:14")
        self.assertEqual(b.lbl_text.text(), "Video feed: NO SIGNAL")   # title never goes stale
        self.assertEqual(b.lbl_detail.text(), "hint")
        self.assertTrue(b._clock_timer.isActive())

    def test_timer_stops_when_alarm_clears(self):
        from PyQt5.QtWidgets import QWidget
        from ui.alarm_banner import AlarmBanner
        a = AlarmManager()
        host = QWidget()
        b = AlarmBanner(a, host)
        host.show()
        a.raise_alarm("v", AlarmLevel.WARN, "x")
        b.refresh()
        self.assertTrue(b._clock_timer.isActive())
        a.clear("v")
        b.refresh()
        self.assertFalse(b._clock_timer.isActive())

    def test_detail_change_updates_without_new_notification(self):
        a = AlarmManager()
        self.assertTrue(a.raise_alarm("k", AlarmLevel.WARN, "t", "a"))
        r = a.revision
        self.assertFalse(a.raise_alarm("k", AlarmLevel.WARN, "t", "b"))
        self.assertGreater(a.revision, r)


class VideoWidgetHealthTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_health_signal_fires_on_state_change_only(self):
        from ui.video_feed_widget import VideoFeedWidget
        w = VideoFeedWidget()
        seen = []
        w.health_changed.connect(lambda s, t: seen.append(s))
        t = [0.0]
        w.health._clock = lambda: t[0]
        w.health.start()
        w._refresh_health()
        w._refresh_health()
        self.assertEqual(seen, ["CONNECTING"])
        t[0] += 5.0
        w._refresh_health()
        self.assertEqual(seen, ["CONNECTING", "NO SIGNAL"])
        self.assertIn("NO SIGNAL", w.lbl_health.text())
        w._health_timer.stop()


if __name__ == "__main__":
    unittest.main()
