"""Continuous-walk mode of the range test (link_range_walk.py): per-second points,
the signal-and-time verdict, the report, and the terminal stream. Loopback / synthetic only."""

import os
import tempfile
import threading
import time
import unittest

import _env  # noqa: F401  -- sets sys.path; must import first

import link_range_test as L
import link_range_walk as W

CFG = L.Config(ping_interval=0.2, udp_enabled=False, video_enabled=True)


def synth_store(seconds=20, ping_gap=None, video_gap=None, signal=lambda t: -50.0 - t, wifi=True):
    """Pings every 0.2 s, video at 15 fps, Wi-Fi once a second - with optional dead stretches
    (start, end) in seconds where pings / frames stop arriving."""
    st = L.Store()
    seq = 0
    for i in range(1, int(seconds / 0.2) + 1):
        t = i * 0.2
        seq += 1
        if ping_gap and ping_gap[0] <= t < ping_gap[1]:
            continue
        st.add("ping_s", t, seq=seq, rtt=3.0)
    for i in range(1, int(seconds * 15) + 1):
        t = i / 15.0
        if video_gap and video_gap[0] <= t < video_gap[1]:
            continue
        st.add("video_frame", t)
        st.add("video_bytes", t, n=20_000)
    if wifi:
        for t in range(1, seconds + 1):
            st.add("wifi", float(t) - 0.2, signal_dbm=signal(t), tx_mbps=100.0)
    return st


class PointsTest(unittest.TestCase):
    def test_a_clean_link_is_loss_free_every_second_with_a_signal(self):
        pts = W.compute_points(synth_store(), 0.0, 20.0, CFG)
        self.assertEqual(len(pts), 20)
        self.assertEqual([p.t_s for p in pts][:3], [1.0, 2.0, 3.0])
        self.assertTrue(all(p.loss_free is True for p in pts), [(p.t_s, p.reasons) for p in pts if not p.loss_free])
        self.assertTrue(all(p.signal_dbm is not None for p in pts))

    def test_incremental_computation_matches_all_at_once(self):
        st = synth_store()
        a = W.compute_points(st, 0.0, 20.0, CFG)
        b = W.compute_points(st, 0.0, 10.0, CFG) + W.compute_points(st, 0.0, 20.0, CFG, start_k=11)
        self.assertEqual([p.t_s for p in a], [p.t_s for p in b])
        self.assertEqual([p.loss_free for p in a], [p.loss_free for p in b])

    def test_a_ping_blackout_makes_those_seconds_loss(self):
        pts = W.compute_points(synth_store(ping_gap=(10.0, 12.5)), 0.0, 20.0, CFG)
        bad = [p.t_s for p in pts if p.loss_free is False]
        self.assertTrue(bad and 10.0 <= min(bad) <= 13.0, bad)
        self.assertTrue(all(p.loss_free for p in pts if p.t_s <= 9.0))

    def test_a_video_stall_over_half_a_second_is_loss(self):
        pts = W.compute_points(synth_store(video_gap=(8.0, 9.2)), 0.0, 20.0, CFG)
        bad = [p for p in pts if p.loss_free is False]
        self.assertTrue(bad)
        self.assertTrue(any("video stall" in r for p in bad for r in p.reasons))

    def test_no_loss_figure_at_all_is_unknown_never_clean(self):
        st = L.Store()
        st.add("wifi", 1.0, signal_dbm=-60.0)
        pts = W.compute_points(st, 0.0, 5.0, L.Config(video_enabled=False))
        self.assertTrue(pts)
        self.assertTrue(all(p.loss_free is None for p in pts))
        v = W.walk_verdict(pts)
        self.assertIsNone(v["loss_free_pct"])
        self.assertIn("No loss figure", v["headline"])

    def test_missing_wifi_sample_falls_back_to_the_last_recent_one(self):
        st = synth_store(wifi=False)
        st.add("wifi", 0.5, signal_dbm=-55.0)
        pts = W.compute_points(st, 0.0, 3.0, CFG)
        self.assertEqual(pts[0].signal_dbm, -55.0)       # within 3 s
        self.assertIsNone(W.compute_points(st, 0.0, 10.0, CFG)[-1].signal_dbm)      # too old


def pt(t, sig, ok, reasons=()):
    return W.WalkPoint(t_s=float(t), signal_dbm=sig, signal_min_dbm=None if sig is None else sig - 1,
                       loss_free=ok, reasons=list(reasons), stall_s=0.1, ping_loss=0.0 if ok else 5.0)


class VerdictTest(unittest.TestCase):
    def points(self):
        out = []
        for t in range(1, 11):
            out.append(pt(t, -50 - 2 * t, True))
        out.append(pt(11, -72, False, ["ping loss 5.0 %"]))
        out.append(pt(12, -73, False, ["video stall 1.2 s"]))
        out.append(pt(13, -72, True))
        out.append(pt(14, -75, False, ["ping loss 9.0 %"]))
        return out

    def test_headline_counts_clean_seconds_and_names_the_first_loss(self):
        v = W.walk_verdict(self.points())
        self.assertEqual(v["clean_for_s"], 10)
        self.assertEqual(v["first_failure"]["t_s"], 11.0)
        self.assertEqual(v["first_failure"]["signal_dbm"], -72)
        self.assertEqual(v["clean_down_to_dbm"], -71)           # lowest signal_min of the clean run (-70 mean - 1)
        self.assertIn("first 10 s", v["headline"])
        self.assertIn("-72 dBm", v["headline"])

    def test_episodes_and_percentages(self):
        v = W.walk_verdict(self.points())
        self.assertEqual(v["failure_episodes"], 2)
        self.assertAlmostEqual(v["loss_free_pct"], 100 * 11 / 14)
        self.assertEqual(v["strongest_failing_dbm"], -72)

    def test_whole_walk_clean_says_the_limit_is_further(self):
        v = W.walk_verdict([pt(t, -50 - t, True) for t in range(1, 8)])
        self.assertIsNone(v["first_failure"])
        self.assertEqual(v["clean_for_s"], 7)
        self.assertIn("whole walk", v["headline"])

    def test_loss_from_the_start(self):
        v = W.walk_verdict([pt(1, -50, False, ["ping loss 5 %"]), pt(2, -50, True)])
        self.assertEqual(v["clean_for_s"], 0)
        self.assertIn("never loss-free", v["headline"])

    def test_bands_report_clean_share_per_signal_level(self):
        bands = {b["lo"]: b for b in W.walk_verdict(self.points())["bands"]}
        self.assertEqual(bands[-75]["seconds"], 4)
        self.assertEqual(bands[-75]["clean_seconds"], 1)
        self.assertEqual(bands[-60]["clean_pct"], 100.0)
        self.assertEqual(list(bands), sorted(bands, reverse=True))      # strongest first

    def test_signal_alone_not_deciding_is_called_out(self):
        pts = [pt(1, -80, True), pt(2, -50, False, ["video stall 1.5 s"])]
        notes = " ".join(W.walk_verdict(pts)["findings"])
        self.assertIn("signal strength alone did not decide", notes)

    def test_no_distance_anywhere_in_the_findings(self):
        text = " ".join(W.walk_verdict(self.points())["findings"]).lower()
        self.assertNotIn(" m.", text.replace("no distance", ""))
        self.assertIn("no distance", text)


class OutputTest(unittest.TestCase):
    def test_stream_line_shows_the_values_and_the_state(self):
        ok = W.stream_line(pt(7, -63, True))
        bad = W.stream_line(pt(8, -71, False, ["ping loss 5.0 %"]))
        self.assertIn("-63 dBm", ok)
        self.assertTrue(ok.rstrip().endswith("ok"))
        self.assertIn("LOSS: ping loss 5.0 %", bad)
        self.assertIn("no data", W.stream_line(pt(9, -60, None)))

    def test_report_files_and_time_axis(self):
        with tempfile.TemporaryDirectory() as d:
            s = W.finish_walk(W.demo_points(60), {"mode": "test"}, d)
            for f in ("report.html", "report.md", "summary.json", "walk.json"):
                self.assertTrue(os.path.isfile(os.path.join(d, f)), f)
            with open(os.path.join(d, "report.html"), encoding="utf-8") as fh:
                html = fh.read()
            with open(os.path.join(d, "report.md"), encoding="utf-8") as fh:
                md = fh.read()
            self.assertIn("LOSS-FREE FOR", html)
            self.assertIn("time (s)", html)
            self.assertNotIn("distance (m)", html)
            self.assertIn("## By signal level", md)
            self.assertEqual(s["mode"], "walk")

    def test_time_axis_labels_are_thinned(self):
        pts = [(float(t), float(t % 7)) for t in range(1, 400)]
        svg = L.svg_chart("x", [("a", "#fff", pts)], "u", xlabel="time (s)", max_xticks=10)
        self.assertLess(svg.count("text-anchor='middle'"), 20)
        self.assertGreater(svg.count("text-anchor='middle'"), 5)

    def test_long_loss_lists_are_capped_in_the_report(self):
        pts = [pt(t, -80, False, ["ping loss 9 %"]) for t in range(1, 200)]
        html = W.walk_html({"m": 1}, pts, W.walk_verdict(pts))
        self.assertIn("more (all of them are in walk.json)", html)
        self.assertLess(html.count("<tr><td>"), 130)


class CliTest(unittest.TestCase):
    def test_flags_and_config_mode(self):
        a = L.parse_args(["--cli", "--radxa", "1.2.3.4", "--walk", "--walk-seconds", "30", "--auto"])
        self.assertTrue(a.walk and a.auto)
        self.assertEqual(a.walk_seconds, 30.0)
        self.assertEqual(L.config_from_args(a).mode, "walk")
        self.assertEqual(L.config_from_args(L.parse_args(["--radxa", "1.2.3.4"])).mode, "holds")

    def test_demo_walk_writes_the_report(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(L.run(L.parse_args(["--cli", "--demo", "--walk", "--out", d])), 0)
            self.assertTrue(os.path.isfile(os.path.join(d, "report.html")))

    def test_streaming_run_prints_a_line_per_second_and_writes_the_report(self):
        """Drive run_walk with a fake session whose store is fed live from a thread."""
        with tempfile.TemporaryDirectory() as d:
            cfg = L.SessionConfig(radxa="127.0.0.1", use_video=True, use_udp=False, out_dir=d, mode="walk")
            stopped = threading.Event()

            class FakeSession:
                def __init__(self):
                    self.cfg, self.store = cfg, L.Store()
                    self.analysis = L.Config(ping_interval=0.2, video_enabled=True)
                    self.iface, self.radxa_before = "wlan0", None

                def stop(self):
                    stopped.set()

            sess = FakeSession()
            feed_stop = threading.Event()

            def feed():
                seq = 0
                while not feed_stop.is_set():
                    now = time.monotonic()
                    seq += 1
                    sess.store.add("ping_s", now, seq=seq, rtt=2.0)
                    sess.store.add("video_frame", now)
                    sess.store.add("video_bytes", now, n=10_000)
                    if seq % 5 == 0:
                        sess.store.add("wifi", now, signal_dbm=-60.0)
                    time.sleep(0.067)

            # one ping per 67 ms is not the 0.2 s the analysis assumes; use matching spacing
            sess.analysis.ping_interval = 0.067
            threading.Thread(target=feed, daemon=True).start()
            lines = []
            args = L.parse_args(["--cli", "--radxa", "127.0.0.1", "--walk", "--auto", "--walk-seconds", "3"])
            try:
                code = W.run_walk(args, sess, out=lines.append)
            finally:
                feed_stop.set()
            self.assertEqual(code, 0)
            self.assertTrue(stopped.is_set())
            streamed = [ln for ln in lines if " dBm" in ln and " s " in ln]
            self.assertGreaterEqual(len(streamed), 2, lines)
            self.assertTrue(os.path.isfile(os.path.join(d, "report.html")))
            self.assertTrue(any("Report:" in ln for ln in lines))


if __name__ == "__main__":
    unittest.main()
