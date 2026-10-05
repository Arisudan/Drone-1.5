"""Walk-away range test (scripts/diagnostics/link_range_test.py + link_probe_server.py).

Hermetic: parsers are fed captured real `iw` / `ping` output, the analysis is fed
synthetic timestamped events, and the only network used is loopback.
"""

import json
import math
import os
import socket
import tempfile
import threading
import time
import unittest

import _env  # noqa: F401  -- sets sys.path; must import first

import link_probe_server as server
import link_range_test as L

# Captured from this project's laptop (iw 6.x, ping iputils).
IW_LINK = """Connected to 20:a6:cd:fe:f9:31 (on wlp0s20f3)
\tSSID: HTIC_INCUBATION
\tfreq: 5260.0
\tRX: 111302 bytes (555 packets)
\tTX: 116942 bytes (539 packets)
\tsignal: -71 dBm
\trx bitrate: 234.0 MBit/s VHT-MCS 5 80MHz VHT-NSS 1
\ttx bitrate: 173.3 MBit/s VHT-MCS 8 short GI VHT-NSS 2
\tbss flags: short-slot-time
"""
IW_STATION = """Station 20:a6:cd:fe:f9:31 (on wlp0s20f3)
\tinactive time:\t8 ms
\ttx packets:\t539
\ttx retries:\t18
\ttx failed:\t2
\tbeacon loss:\t0
\tsignal:  \t-71 [-71] dBm
\tsignal avg:\t-70 dBm
\ttx bitrate:\t173.3 MBit/s VHT-MCS 8 short GI VHT-NSS 2
\trx bitrate:\t234.0 MBit/s VHT-MCS 5 80MHz VHT-NSS 1
"""
IW_DEV = "phy#1\n\tUnnamed/non-netdev interface\n\tInterface wlp0s20f3\n\t\tifindex 3\n"


def store_with(**kinds):
    """Store from {kind: [(t, {fields}), ...]}."""
    st = L.Store()
    for kind, evs in kinds.items():
        for t, f in evs:
            st.add(kind, t, **f)
    return st


class ParserTest(unittest.TestCase):
    def test_ping_line(self):
        self.assertEqual(L.parse_ping_line("64 bytes from 10.0.0.1: icmp_seq=7 ttl=64 time=3.42 ms"), (7, 3.42))
        self.assertEqual(L.parse_ping_line("1208 bytes from x: icmp_seq=12 ttl=63 time=104 ms"), (12, 104.0))
        for junk in ("PING 10.0.0.1 (10.0.0.1) 56(84) bytes of data.", "", "From 10.0.0.2 icmp_seq=3 Destination Host Unreachable"):
            self.assertIsNone(L.parse_ping_line(junk))

    def test_iw_link_connected(self):
        d = L.parse_iw_link(IW_LINK)
        self.assertTrue(d["connected"])
        self.assertEqual(d["ssid"], "HTIC_INCUBATION")
        self.assertEqual(d["signal_dbm"], -71)
        self.assertEqual(d["freq_mhz"], 5260.0)
        self.assertEqual(d["tx_mbps"], 173.3)
        self.assertEqual(d["rx_mbps"], 234.0)

    def test_iw_link_not_connected(self):
        self.assertEqual(L.parse_iw_link("Not connected."), {"connected": False})
        self.assertEqual(L.parse_iw_link(""), {"connected": False})

    def test_iw_station_counters_and_signal_avg_is_not_confused_with_signal(self):
        d = L.parse_iw_station(IW_STATION)
        self.assertEqual(d["signal_dbm"], -71)
        self.assertEqual(d["signal_avg_dbm"], -70)
        self.assertEqual((d["tx_retries"], d["tx_failed"], d["tx_packets"]), (18, 2, 539))
        self.assertEqual(d["beacon_loss"], 0)

    def test_missing_fields_are_none_not_zero(self):
        d = L.parse_iw_station("Station x\n\tsignal:\t-60 [-60] dBm\n")
        self.assertEqual(d["signal_dbm"], -60)
        self.assertIsNone(d["tx_retries"])

    def test_proc_wireless_fallback(self):
        txt = ("Inter-| sta-|   Quality        |   Discarded packets\n face | tus | link level noise |\n"
               "wlp0s20f3: 0000   45.  -65.  -256.  0 0 0 0 0 0\n")
        d = L.parse_proc_wireless(txt, "wlp0s20f3")
        self.assertEqual((d["link_quality"], d["signal_dbm"], d["noise_dbm"]), (45.0, -65.0, -256.0))
        self.assertEqual(L.parse_proc_wireless(txt, "wlan9"), {})

    def test_iw_dev_interfaces(self):
        self.assertEqual(L.parse_iw_dev(IW_DEV), ["wlp0s20f3"])


class JpegFrameCounterTest(unittest.TestCase):
    def frame(self, n=50):
        return b"\xff\xd8" + b"\x11\x22\x33" * n + b"\xff\xd9"

    def test_counts_frames_and_sizes_in_one_chunk(self):
        f = self.frame()
        c = L.JpegFrameCounter()
        stream = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n--frame\r\n\r\n" + f
        self.assertEqual(c.feed(stream), [len(f), len(f)])

    def test_any_split_point_gives_the_same_answer(self):
        f = self.frame(7)
        stream = b"hdr" + f + b"mid" + f + b"tail"
        for cut in range(1, len(stream)):
            c = L.JpegFrameCounter()
            sizes = c.feed(stream[:cut]) + c.feed(stream[cut:])
            self.assertEqual(sizes, [len(f), len(f)], f"split at {cut}")

    def test_many_tiny_chunks(self):
        f = self.frame(30)
        stream = f * 5
        c = L.JpegFrameCounter()
        got = []
        for i in range(0, len(stream), 3):
            got += c.feed(stream[i:i + 3])
        self.assertEqual(got, [len(f)] * 5)

    def test_a_partial_frame_is_not_counted_until_it_ends(self):
        f = self.frame()
        c = L.JpegFrameCounter()
        self.assertEqual(c.feed(f[:-10]), [])
        self.assertEqual(c.feed(f[-10:]), [len(f)])

    def test_garbage_without_markers_counts_nothing(self):
        self.assertEqual(L.JpegFrameCounter().feed(b"\x00\x01\xff\x02" * 100), [])

    def test_escaped_ff_bytes_inside_a_frame_do_not_end_it(self):
        f = b"\xff\xd8" + b"\xff\x00" * 20 + b"\xff\xd9"
        self.assertEqual(L.JpegFrameCounter().feed(f), [len(f)])


class PacketTest(unittest.TestCase):
    def test_round_trip_and_padding(self):
        p = L.make_packet(5, 12.5, 1200, srv_count=3)
        self.assertEqual(len(p), 1200)
        self.assertEqual(L.parse_packet(p), (5, 12.5, 3))

    def test_foreign_or_short_datagrams_are_ignored(self):
        self.assertIsNone(L.parse_packet(b"hello"))
        self.assertIsNone(L.parse_packet(b"XXXX" + b"\x00" * 30))

    def test_server_and_client_share_the_wire_format(self):
        self.assertEqual(server.HDR.format, L.HDR.format)
        self.assertEqual(server.MAGIC, L.MAGIC)

    def test_server_counts_per_client_and_echoes_same_size(self):
        sessions = {}
        a, b = ("10.0.0.1", 1), ("10.0.0.2", 1)
        for seq in range(3):
            r = server.handle_datagram(L.make_packet(seq, 1.0, 1200), a, sessions, 100.0)
        self.assertEqual(len(r), 1200)
        self.assertEqual(L.parse_packet(r), (2, 1.0, 3))
        r2 = server.handle_datagram(L.make_packet(0, 1.0, 600), b, sessions, 100.0)
        self.assertEqual(L.parse_packet(r2)[2], 1, "a second client has its own count")

    def test_server_restarts_the_count_after_a_silence(self):
        sessions = {}
        a = ("10.0.0.1", 1)
        server.handle_datagram(L.make_packet(0, 0, 100), a, sessions, 0.0)
        server.handle_datagram(L.make_packet(1, 0, 100), a, sessions, 1.0)
        r = server.handle_datagram(L.make_packet(2, 0, 100), a, sessions, 1.0 + server.SESSION_IDLE_S + 1)
        self.assertEqual(L.parse_packet(r)[2], 1)

    def test_server_ignores_non_probe_traffic(self):
        self.assertIsNone(server.handle_datagram(b"GET / HTTP/1.1", ("x", 1), {}, 0.0))


class StatsHelperTest(unittest.TestCase):
    def test_percentile(self):
        self.assertIsNone(L.percentile([], 95))
        self.assertEqual(L.percentile([5], 95), 5)
        self.assertEqual(L.percentile([1, 2, 3, 4], 50), 2.5)
        self.assertAlmostEqual(L.percentile(list(range(101)), 95), 95)

    def test_jitter(self):
        self.assertIsNone(L.jitter_ms([3.0]))
        self.assertAlmostEqual(L.jitter_ms([10, 12, 10, 14]), (2 + 2 + 4) / 3)

    def test_loss_pct_clamps(self):
        self.assertAlmostEqual(L.loss_pct(40, 50), 20.0)
        self.assertEqual(L.loss_pct(60, 50), 0.0)
        self.assertIsNone(L.loss_pct(5, 0))

    def test_counter_delta_survives_a_reset(self):
        # The card roamed: counters restarted from zero. Last-minus-first was -153.
        self.assertEqual(L.counter_delta([10, 20, 30]), 20)
        self.assertEqual(L.counter_delta([150, 160, 3, 8]), 10 + 3 + 5)
        self.assertGreaterEqual(L.counter_delta([200, 5, 6]), 0)
        self.assertEqual(L.counter_delta([7]), 0)


class AnalyseWindowTest(unittest.TestCase):
    CFG = L.Config(ping_interval=0.2)

    def test_ping_loss_counts_missing_sequence_numbers(self):
        # 50 pings in 10 s, seqs 10..14 never answered -> 5 of 50 lost.
        replies = [(0.2 * i + 0.1, {"seq": i, "rtt": 5.0}) for i in range(50) if not 10 <= i < 15]
        st = store_with(ping_s=replies)
        s = L.analyse_window(st, 0.0, 10.0, self.CFG)
        self.assertAlmostEqual(s.ping_loss, 10.0)
        self.assertAlmostEqual(s.ping_avg, 5.0)

    def test_a_perfect_link_is_exactly_zero_whatever_the_window_alignment(self):
        # Strict zero must survive reply-in-flight edge effects: 73, 74 or 75 replies
        # in a 15 s window of 0.2 s pings are all a clean link.
        for offset in (0.0, 0.05, 0.11, 0.19):
            replies = [(0.2 * i + offset, {"seq": i, "rtt": 4.0}) for i in range(80)]
            s = L.analyse_window(store_with(ping_s=replies), 1.0, 16.0, self.CFG)
            self.assertEqual(s.ping_loss, 0.0, f"offset {offset}")

    def test_not_measured_stays_none_never_a_perfect_zero(self):
        s = L.analyse_window(L.Store(), 0.0, 10.0, self.CFG)
        for attr in ("ping_loss", "pingL_loss", "video_fps", "udp_up_loss", "signal_mean"):
            self.assertIsNone(getattr(s, attr), attr)

    def test_a_dead_ping_window_is_100_percent_loss_not_none(self):
        st = store_with(ping_s=[(-5.0, {"seq": 1, "rtt": 3.0})])         # earlier reply only
        self.assertEqual(L.analyse_window(st, 0.0, 10.0, self.CFG).ping_loss, 100.0)

    def test_wifi_signal_rate_and_retries_per_second(self):
        wifi = [(float(t), {"signal_dbm": -60 - t, "tx_mbps": 100.0, "tx_retries": 10 * t, "tx_failed": t})
                for t in range(1, 11)]
        s = L.analyse_window(store_with(wifi=wifi), 0.0, 10.0, self.CFG)
        self.assertAlmostEqual(s.signal_mean, -65.5)
        self.assertEqual(s.signal_min, -70)
        self.assertEqual(s.tx_mbps, 100.0)
        self.assertAlmostEqual(s.retries_per_s, 10.0)
        self.assertAlmostEqual(s.failed_per_s, 1.0)

    def test_retries_are_never_negative_after_roaming(self):
        wifi = [(1.0, {"tx_retries": 200}), (2.0, {"tx_retries": 220}), (3.0, {"tx_retries": 4}), (4.0, {"tx_retries": 9})]
        s = L.analyse_window(store_with(wifi=wifi), 0.0, 4.0, self.CFG)
        self.assertGreaterEqual(s.retries_per_s, 0)

    def test_video_fps_mbps_and_stall(self):
        frames = [(0.5 + 0.1 * i, {"size": 1000}) for i in range(95)]       # all inside (0, 10]
        frames = [f for f in frames if not 3.0 <= f[0] < 6.0]            # a 3 s outage
        byts = [(t, {"n": 1250}) for t, _ in frames]
        st = store_with(video_frame=frames, video_bytes=byts)
        s = L.analyse_window(st, 0.0, 10.0, self.CFG)
        self.assertAlmostEqual(s.video_fps, len(frames) / 10)
        self.assertAlmostEqual(s.video_mbps, len(frames) * 1250 * 8 / 10 / 1e6)
        self.assertAlmostEqual(s.video_stall_max, 3.1, delta=0.15)

    def test_no_frames_at_all_is_a_stall_the_length_of_the_window(self):
        st = store_with(video_event=[(1.0, {"event": "disconnect"})], video_frame=[(-3.0, {"size": 5})])
        s = L.analyse_window(st, 0.0, 10.0, self.CFG)
        self.assertEqual(s.video_fps, 0.0)
        self.assertEqual(s.video_stall_max, 10.0)
        self.assertEqual(s.video_disconnects, 1)

    def test_disconnect_events_use_the_event_field_regression(self):
        # Store.add(kind, ...) once collided with an event field also named "kind",
        # silently killing the video thread. The field is "event" now.
        st = L.Store()
        st.add("video_event", 1.0, event="connect")
        st.add("video_event", 2.0, event="disconnect", info="x")
        self.assertEqual(st.count("video_event"), 2)

    def test_udp_up_and_down_loss_are_separated(self):
        # 100 probes seq 0..99 (one per 0.05 s). The server received 90 (seqs 9,19,...
        # lost on the way up) and of those echoed 81 (every 10th echo lost on the
        # way back). Echo carries (seq, server count).
        received = [q for q in range(100) if q % 10 != 9]                 # 90 arrived
        echoed = [q for i, q in enumerate(received) if i % 10 != 9]       # 81 came back
        count = {q: i + 1 for i, q in enumerate(received)}
        tx = [(0.05 * q + 0.01, {"seq": q}) for q in range(100)]
        echoes = [(0.05 * q + 0.03, {"seq": q, "rtt": 4.0, "srv": count[q]}) for q in echoed]
        s = L.analyse_window(store_with(udp_tx=tx, udp_rx=echoes), 0.0, 5.2, L.Config(udp_enabled=True))
        self.assertAlmostEqual(s.udp_up_loss, 10.0, delta=1.5)
        self.assertAlmostEqual(s.udp_down_loss, 10.0, delta=1.5)
        self.assertAlmostEqual(s.udp_rtt_p95, 4.0)

    def test_a_clean_udp_link_is_exactly_zero_both_ways(self):
        tx = [(0.01 * q, {"seq": q}) for q in range(1, 601)]
        echoes = [(0.01 * q + 0.004, {"seq": q, "rtt": 3.0, "srv": q + 1}) for q in range(1, 601)]
        for t0, t1 in ((0.0, 6.0), (0.3, 5.7), (1.234, 4.321)):
            s = L.analyse_window(store_with(udp_tx=tx, udp_rx=echoes), t0, t1, L.Config(udp_enabled=True))
            self.assertEqual((s.udp_up_loss, s.udp_down_loss), (0.0, 0.0), f"window {t0}-{t1}")

    def test_no_echo_at_all_is_total_loss(self):
        tx = [(0.1 * i, {"seq": i}) for i in range(1, 51)]
        s = L.analyse_window(store_with(udp_tx=tx), 0.0, 5.0, L.Config(udp_enabled=True))
        self.assertEqual((s.udp_up_loss, s.udp_down_loss), (100.0, 100.0))

    def test_a_dead_tail_after_the_last_echo_counts_as_loss(self):
        tx = [(0.01 * q, {"seq": q}) for q in range(1, 1001)]                 # 10 s of probes
        echoes = [(0.01 * q + 0.004, {"seq": q, "rtt": 3.0, "srv": q + 1}) for q in range(1, 501)]  # silent after 5 s
        s = L.analyse_window(store_with(udp_tx=tx, udp_rx=echoes), 0.0, 10.0, L.Config(udp_enabled=True))
        self.assertGreater(s.udp_up_loss, 40.0)
        self.assertGreater(s.udp_down_loss, 40.0)

    def test_a_ping_dead_tail_counts_but_a_short_gap_at_the_edge_does_not(self):
        replies = [(0.2 * i, {"seq": i, "rtt": 3.0}) for i in range(25)]      # alive 0-5 s then silent
        s = L.analyse_window(store_with(ping_s=replies), 0.0, 10.0, self.CFG)
        self.assertGreater(s.ping_loss, 40.0)
        short = [(0.2 * i, {"seq": i, "rtt": 3.0}) for i in range(0, 47)]      # last reply 9.2 s
        self.assertEqual(L.analyse_window(store_with(ping_s=short), 0.0, 10.0, self.CFG).ping_loss, 0.0)

    def test_udp_stats_are_unmeasured_when_the_probe_never_ran(self):
        s = L.analyse_window(L.Store(), 0.0, 5.0, L.Config(udp_enabled=True))
        self.assertIsNone(s.udp_up_loss)


class VerdictTest(unittest.TestCase):
    def hold(self, **kw):
        return L.HoldStats(distance_m=10.0, **kw)

    def test_good_marginal_poor(self):
        self.assertEqual(L.verdict_for(self.hold(signal_mean=-60, ping_loss=0.5, video_fps=24, video_stall_max=0.2), 24)[0], "Good")
        self.assertEqual(L.verdict_for(self.hold(signal_mean=-75), None)[0], "Marginal")
        self.assertEqual(L.verdict_for(self.hold(signal_mean=-85), None)[0], "Poor")
        self.assertEqual(L.verdict_for(self.hold(ping_loss=5.0), None)[0], "Marginal")
        self.assertEqual(L.verdict_for(self.hold(ping_loss=15.0), None)[0], "Poor")
        self.assertEqual(L.verdict_for(self.hold(video_stall_max=1.0), None)[0], "Marginal")
        self.assertEqual(L.verdict_for(self.hold(video_stall_max=3.0), None)[0], "Poor")

    def test_fps_is_judged_against_the_close_range_baseline(self):
        self.assertEqual(L.verdict_for(self.hold(video_fps=22), 24)[0], "Good")
        self.assertEqual(L.verdict_for(self.hold(video_fps=17), 24)[0], "Marginal")
        self.assertEqual(L.verdict_for(self.hold(video_fps=10), 24)[0], "Poor")
        self.assertEqual(L.verdict_for(self.hold(video_fps=10), None)[0], "Good")   # nothing to compare to

    def test_worst_reading_wins_and_reasons_are_given(self):
        label, reasons = L.verdict_for(self.hold(signal_mean=-75, ping_loss=20.0), None)
        self.assertEqual(label, "Poor")
        self.assertTrue(any("ping loss" in r for r in reasons))
        self.assertTrue(any("signal" in r for r in reasons))

    def test_boundaries_are_exclusive_of_the_limit(self):
        self.assertEqual(L.verdict_for(self.hold(ping_loss=2.0), None)[0], "Good")
        self.assertEqual(L.verdict_for(self.hold(ping_loss=10.0), None)[0], "Marginal")
        self.assertEqual(L.verdict_for(self.hold(signal_mean=-70.0), None)[0], "Good")

    def test_a_video_reconnect_is_at_least_marginal(self):
        self.assertEqual(L.verdict_for(self.hold(video_disconnects=2), None)[0], "Marginal")

    def test_missing_metrics_are_ignored(self):
        self.assertEqual(L.verdict_for(self.hold(), 24)[0], "Good")


class LossFreeTest(unittest.TestCase):
    def h(self, **kw):
        return L.HoldStats(distance_m=5.0, **kw)

    def test_clean_means_nothing_lost_and_no_stall(self):
        self.assertTrue(L.is_loss_free(self.h(ping_loss=0.0, udp_up_loss=0.0, udp_down_loss=0.0, video_stall_max=0.1)))

    def test_any_measured_loss_breaks_it(self):
        for kw in ({"ping_loss": 0.5}, {"pingL_loss": 0.5}, {"udp_up_loss": 0.5}, {"udp_down_loss": 0.5}):
            self.assertFalse(L.is_loss_free(self.h(**{"ping_loss": 0.0, **kw})), kw)

    def test_a_video_stall_or_reconnect_breaks_it(self):
        self.assertFalse(L.is_loss_free(self.h(ping_loss=0.0, video_stall_max=1.5)))
        self.assertFalse(L.is_loss_free(self.h(ping_loss=0.0, video_disconnects=1)))
        self.assertTrue(L.is_loss_free(self.h(ping_loss=0.0, video_stall_max=0.2)))

    def test_no_data_is_unknown_never_loss_free(self):
        self.assertIsNone(L.is_loss_free(self.h()))
        self.assertIsNone(L.is_loss_free(self.h(video_fps=20.0)))

    def test_tolerance(self):
        self.assertTrue(L.is_loss_free(self.h(ping_loss=1.0), tol_pct=2.0))
        self.assertFalse(L.is_loss_free(self.h(ping_loss=3.0), tol_pct=2.0))

    def test_reasons_name_what_was_lost(self):
        r = L.loss_reasons(self.h(ping_loss=3.3, udp_down_loss=1.0, video_disconnects=2, video_stall_max=2.0))
        self.assertEqual(len(r), 4)
        self.assertTrue(any("ping loss 3.3" in x for x in r))
        self.assertTrue(any("UDP down" in x for x in r))


class HeadlineTest(unittest.TestCase):
    def holds(self, losses):
        out = []
        for i, loss in enumerate(losses):
            h = L.HoldStats(distance_m=5.0 * i, ping_loss=loss)
            h.verdict = "Good"
            out.append(h)
        return out

    def test_states_the_distance(self):
        hs = self.holds([0.0, 0.0, 0.0, 2.0])
        self.assertIn("up to 10 m", L.headline(L.usable_ranges(hs), hs))

    def test_loss_at_the_first_mark(self):
        hs = self.holds([1.0, 2.0])
        self.assertIn("never loss-free", L.headline(L.usable_ranges(hs), hs))

    def test_never_lost_anything_says_the_limit_is_further(self):
        hs = self.holds([0.0, 0.0, 0.0])
        self.assertIn("further than you walked", L.headline(L.usable_ranges(hs), hs))

    def test_the_headline_leads_the_findings_and_the_report(self):
        hs = self.holds([0.0, 0.0, 3.0])
        for h in hs:
            h.loss_free = L.is_loss_free(h)
        rg = L.usable_ranges(hs)
        notes = L.findings(hs, None, rg, None)
        self.assertIn("loss-free", notes[0])
        self.assertTrue(any("First loss at 10 m" in n for n in notes))
        page = L.html_report({}, hs, notes, None, rg)
        self.assertIn("LOSS-FREE LIVE FEED UP TO", page)
        self.assertIn("5 m", page)


class SetupHelpersTest(unittest.TestCase):
    def test_valid_hosts(self):
        for ok in ("172.16.100.182", "192.168.1.2", "radxa-dragon-q6a", "radxa.local", " 10.0.0.1 "):
            self.assertTrue(L.valid_host(ok), ok)

    def test_invalid_hosts(self):
        for bad in ("", "   ", "256.1.1.1", "1.2.3", "1.2.3.4.5", "a b", "http://1.2.3.4", "-bad", "1.2.3.4:80",
                    "bad-.local", "a..b", "999.999.999.999"):
            self.assertFalse(L.valid_host(bad), bad)

    def test_prefs_round_trip_and_survive_corruption(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "prefs.json")
            self.assertEqual(L.load_prefs(path), {})
            L.save_prefs({"radxa": "1.2.3.4", "step": 3}, path)
            self.assertEqual(L.load_prefs(path), {"radxa": "1.2.3.4", "step": 3})
            with open(path, "w") as fh:
                fh.write("not json")
            self.assertEqual(L.load_prefs(path), {})
            with open(path, "w") as fh:
                fh.write("[1, 2]")
            self.assertEqual(L.load_prefs(path), {})

    def test_session_config_derives_urls_and_targets(self):
        c = L.SessionConfig(radxa="1.2.3.4", ssh_user="pi", video_port=9090)
        self.assertEqual(c.ssh_target, "pi@1.2.3.4")
        self.assertEqual(c.url, "http://1.2.3.4:9090/video")
        self.assertEqual(L.SessionConfig(radxa="1.2.3.4", video_url="http://x/y").url, "http://x/y")

    def test_cli_args_become_a_session_config(self):
        a = L.parse_args(["--radxa", "1.2.3.4", "--udp", "--start-helper", "radxa@1.2.3.4", "--no-video",
                          "--loss-tolerance", "1.5", "--step", "3"])
        c = L.config_from_args(a)
        self.assertEqual((c.radxa, c.use_udp, c.use_video, c.helper_target, c.loss_tolerance, c.step),
                         ("1.2.3.4", True, False, "radxa@1.2.3.4", 1.5, 3.0))

    def test_no_arguments_means_the_window(self):
        a = L.parse_args([])
        self.assertFalse(a.cli)
        self.assertIsNone(a.radxa)


class PathLossFitTest(unittest.TestCase):
    def model(self, d, r1=-45.0, n=3.0):
        return r1 - 10 * n * math.log10(d)

    def test_recovers_the_parameters(self):
        pts = [(d, self.model(d)) for d in (1, 2, 5, 10, 20, 40)]
        fit = L.fit_path_loss(pts)
        self.assertAlmostEqual(fit["rssi_1m"], -45.0, places=6)
        self.assertAlmostEqual(fit["exponent"], 3.0, places=6)
        self.assertAlmostEqual(fit["r2"], 1.0, places=6)

    def test_zero_metre_baseline_is_excluded_not_a_log_of_zero(self):
        pts = [(0, -40.0)] + [(d, self.model(d)) for d in (2, 5, 10)]
        self.assertAlmostEqual(L.fit_path_loss(pts)["exponent"], 3.0, places=6)

    def test_needs_three_distinct_distances(self):
        self.assertIsNone(L.fit_path_loss([(5, -60), (10, -70)]))
        self.assertIsNone(L.fit_path_loss([(5, -60), (5, -61), (5, -59)]))
        self.assertIsNone(L.fit_path_loss([(0, -40), (0.5, -50), (0.9, -55)]))

    def test_estimate_distance_round_trips(self):
        fit = L.fit_path_loss([(d, self.model(d)) for d in (1, 3, 9, 27)])
        for d in (2, 7, 15):
            self.assertAlmostEqual(L.estimate_distance(self.model(d), fit), d, places=4)

    def test_a_flat_signal_gives_no_estimate(self):
        self.assertIsNone(L.estimate_distance(-60, {"rssi_1m": -60.0, "exponent": 0.0, "r2": 0.0}))


class RangesTest(unittest.TestCase):
    def hs(self, *verdicts):
        out = []
        for i, v in enumerate(verdicts):
            h = L.HoldStats(distance_m=5.0 * i)
            h.verdict = v
            out.append(h)
        return out

    def test_good_to_and_marginal_to(self):
        r = L.usable_ranges(self.hs("Good", "Good", "Marginal", "Poor", "Good"))
        self.assertEqual((r["good_to"], r["marginal_to"]), (5.0, 10.0))

    def test_all_good(self):
        r = L.usable_ranges(self.hs("Good", "Good", "Good"))
        self.assertEqual((r["good_to"], r["marginal_to"]), (10.0, 10.0))

    def test_never_good(self):
        self.assertEqual(L.usable_ranges(self.hs("Poor", "Poor"))["good_to"], None)

    def test_loss_free_range_stops_at_the_first_hold_with_any_loss(self):
        holds = self.hs("Good", "Good", "Good", "Good")
        for h, loss in zip(holds, (0.0, 0.0, 1.0, 0.0)):
            h.ping_loss = loss
        r = L.usable_ranges(holds)
        self.assertEqual(r["loss_free_to"], 5.0)         # 10 m lost a packet; 15 m clean again doesn't count

    def test_a_hold_with_no_loss_data_cannot_extend_the_loss_free_range(self):
        holds = self.hs("Good", "Good")
        holds[0].ping_loss = 0.0                          # 5 m has no loss figure at all
        self.assertEqual(L.usable_ranges(holds)["loss_free_to"], 0.0)

    def test_tolerance_lets_a_little_loss_through(self):
        holds = self.hs("Good", "Good", "Good")
        for h, loss in zip(holds, (0.0, 1.5, 4.0)):
            h.ping_loss = loss
        self.assertEqual(L.usable_ranges(holds, 0.0)["loss_free_to"], 0.0)
        self.assertEqual(L.usable_ranges(holds, 2.0)["loss_free_to"], 5.0)

    def test_order_does_not_depend_on_input_order(self):
        hs = self.hs("Good", "Marginal", "Poor")
        self.assertEqual(L.usable_ranges(list(reversed(hs))), L.usable_ranges(hs))


class ReportTest(unittest.TestCase):
    def setUp(self):
        self.holds, self.base = L.demo_holds()
        for h in self.holds:
            h.verdict, h.reasons = L.verdict_for(h, self.base)

    def test_demo_has_a_loss_free_close_range_and_loses_packets_further_out(self):
        for h in self.holds:
            h.loss_free = L.is_loss_free(h)
        self.assertTrue(self.holds[0].loss_free)
        self.assertFalse(self.holds[-1].loss_free)
        r = L.usable_ranges(self.holds)
        self.assertGreaterEqual(r["loss_free_to"], 0.0)
        self.assertLess(r["loss_free_to"], self.holds[-1].distance_m)

    def test_demo_shows_all_three_verdicts_in_order(self):
        seen = [h.verdict for h in self.holds]
        self.assertEqual(seen[0], "Good")
        self.assertIn("Marginal", seen)
        self.assertEqual(seen[-1], "Poor")
        self.assertLess(seen.index("Marginal"), seen.index("Poor"))

    def test_demo_is_deterministic(self):
        again, _ = L.demo_holds()
        self.assertEqual([round(h.signal_mean, 6) for h in again], [round(h.signal_mean, 6) for h in self.holds])

    def test_text_table_has_a_row_per_distance_and_the_verdict(self):
        t = L.text_table(self.holds)
        self.assertEqual(len(t.splitlines()), len(self.holds) + 2)
        self.assertIn("Verdict", t.splitlines()[0])
        self.assertIn("Loss-free", t.splitlines()[0])
        self.assertIn("Poor", t)

    def test_findings_state_the_ranges_and_the_fit(self):
        fit = L.fit_path_loss([(h.distance_m, h.signal_mean) for h in self.holds])
        notes = L.findings(self.holds, fit, L.usable_ranges(self.holds), self.base)
        text = " ".join(notes)
        self.assertIn("loss-free", text)
        self.assertIn("stays Good", text)
        self.assertIn("exponent", text)
        self.assertIn("baseline", text)

    def test_findings_without_a_fit_say_why(self):
        notes = L.findings(self.holds[:2], None, L.usable_ranges(self.holds[:2]), None)
        self.assertTrue(any("Not enough distances" in n for n in notes))

    def test_html_report_contains_table_charts_and_escapes_text(self):
        fit = L.fit_path_loss([(h.distance_m, h.signal_mean) for h in self.holds])
        page = L.html_report({"note": "<script>x</script>"}, self.holds, ["a & b"], fit)
        self.assertIn("<table>", page)
        self.assertEqual(page.count("<svg"), 4)
        self.assertNotIn("<script>x</script>", page)
        self.assertIn("a &amp; b", page)
        self.assertIn("class='poor'", page)

    def test_svg_chart_with_no_data_says_so(self):
        self.assertIn("no data", L.svg_chart("T", [("s", "#fff", [(1, None)])], "y"))

    def test_markdown_report(self):
        md = L.markdown_report({"radxa": "1.2.3.4"}, self.holds, ["x"])
        self.assertIn("| Dist m |", md)
        self.assertIn("**radxa**: 1.2.3.4", md)

    def test_finish_writes_every_artifact_and_marks_the_verdicts(self):
        holds, base = L.demo_holds()
        timeline = [{"t_s": 0.0, "phase": "hold", "distance_m": 0}]
        with tempfile.TemporaryDirectory() as d:
            summary = L.finish(holds, base, {"mode": "test"}, d, timeline)
            names = set(os.listdir(d))
            self.assertEqual(names, {"holds.json", "summary.json", "report.html", "report.md", "samples.csv"})
            data = json.load(open(os.path.join(d, "holds.json")))
            self.assertEqual(data[0]["verdict"], "Good")
            self.assertIsNotNone(data[0]["video_drop"])
            self.assertIn("ranges", summary)
            json.load(open(os.path.join(d, "summary.json")))

    def test_timeline_labels_holds_and_walking(self):
        st = store_with(wifi=[(float(t) + 0.5, {"signal_dbm": -60}) for t in range(10)])
        rows = L.build_timeline(st, 0.0, 10.0, L.Config(), [(2.0, 5.0, 5.0)])
        phases = {r["t_s"]: r["phase"] for r in rows}
        self.assertEqual(phases[3.0], "hold")
        self.assertEqual(phases[0.0], "walk")
        self.assertEqual(next(r for r in rows if r["t_s"] == 3.0)["distance_m"], 5.0)
        self.assertEqual(len(rows), 10)


class CommandLineTest(unittest.TestCase):
    def test_defaults(self):
        a = L.parse_args(["--radxa", "1.2.3.4"])
        self.assertEqual((a.step, a.hold, a.udp, a.no_video, a.udp_mbps), (5.0, 15.0, False, False, 1.0))

    def test_demo_runs_without_a_network_and_writes_the_report(self):
        import contextlib
        import io
        with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(L.main(["--demo", "--out", d]), 0)
            self.assertTrue(os.path.exists(os.path.join(d, "report.html")))

    def test_a_missing_radxa_is_an_error_not_a_crash(self):
        import contextlib
        import io
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(L.run(L.parse_args(["--cli"])), 2)

    def test_cli_flag_never_opens_a_window_or_prompts(self):
        import contextlib
        import io
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(L.main(["--cli"]), 2)        # no --radxa: error, no prompt, no window


# ── loopback integration ────────────────────────────────────────────────

class _EchoServer(threading.Thread):
    """The helper's logic on a loopback socket, with optional deliberate loss."""

    def __init__(self, drop_uplink_every=0, drop_echo_every=0):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.up, self.echo = drop_uplink_every, drop_echo_every
        self.stop_ev = threading.Event()
        self.n = 0

    def run(self):
        sessions = {}
        while not self.stop_ev.is_set():
            try:
                data, addr = self.sock.recvfrom(4096)
            except (socket.timeout, OSError):
                continue
            self.n += 1
            if self.up and self.n % self.up == 0:
                continue                                # lost before reaching the server
            reply = server.handle_datagram(data, addr, sessions, time.monotonic())
            if reply is None or (self.echo and self.n % self.echo == 0):
                continue                                # received but the echo is lost
            self.sock.sendto(reply, addr)


class LoopbackTest(unittest.TestCase):
    def _run_probe(self, srv, seconds=3.0, mbps=0.8):
        st, stop = L.Store(), threading.Event()
        probe = L.UdpProbe("127.0.0.1", srv.port, mbps, 1200, st, stop)
        probe.start()
        t0 = time.monotonic()
        time.sleep(seconds)
        t1 = time.monotonic()
        stop.set()
        time.sleep(0.4)
        return L.analyse_window(st, t0 + 0.3, t1 - 0.3, L.Config(udp_enabled=True))

    def test_udp_probe_measures_no_loss_on_a_clean_link(self):
        srv = _EchoServer()
        srv.start()
        s = self._run_probe(srv)
        srv.stop_ev.set()
        self.assertLess(s.udp_up_loss, 3.0)
        self.assertLess(s.udp_down_loss, 3.0)

    def test_udp_probe_attributes_loss_to_the_right_direction(self):
        up = _EchoServer(drop_uplink_every=5)             # 20 % never reach the server
        up.start()
        s = self._run_probe(up)
        up.stop_ev.set()
        self.assertAlmostEqual(s.udp_up_loss, 20.0, delta=6.0)
        self.assertLess(s.udp_down_loss, 5.0)

        down = _EchoServer(drop_echo_every=4)             # 25 % of echoes never come back
        down.start()
        s = self._run_probe(down)
        down.stop_ev.set()
        self.assertLess(s.udp_up_loss, 5.0)
        self.assertAlmostEqual(s.udp_down_loss, 25.0, delta=7.0)

    def test_video_sampler_counts_frames_and_bytes_from_a_real_http_stream(self):
        import http.server
        import socketserver
        frame = b"\xff\xd8" + b"\x01\x02\x03" * 300 + b"\xff\xd9"

        class H(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    for _ in range(200):
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                        self.wfile.flush()
                        time.sleep(0.05)
                except OSError:
                    pass

        class S(socketserver.ThreadingMixIn, http.server.HTTPServer):
            daemon_threads = True

        httpd = S(("127.0.0.1", 0), H)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        st, stop = L.Store(), threading.Event()
        v = L.VideoSampler(f"http://127.0.0.1:{httpd.server_address[1]}/video", st, stop)
        v.start()
        t0 = time.monotonic()
        time.sleep(2.5)
        t1 = time.monotonic()
        stop.set()
        httpd.shutdown()
        s = L.analyse_window(st, t0 + 0.4, t1, L.Config())
        self.assertAlmostEqual(s.video_fps, 20.0, delta=4.0)
        self.assertGreater(s.video_mbps, 0.05)
        self.assertLess(s.video_stall_max, 0.5)

    def test_video_sampler_records_a_disconnect_when_nothing_listens(self):
        s0 = socket.socket()
        s0.bind(("127.0.0.1", 0))
        port = s0.getsockname()[1]
        s0.close()                                        # nothing is listening now
        st, stop = L.Store(), threading.Event()
        v = L.VideoSampler(f"http://127.0.0.1:{port}/video", st, stop, timeout=1.0)
        v.start()
        time.sleep(1.5)
        stop.set()
        events = [f for _, f in st.window("video_event", 0, time.monotonic() + 1)]
        self.assertTrue(any(e["event"] == "disconnect" for e in events))


if __name__ == "__main__":
    unittest.main()


class HelperFailureReasonTest(unittest.TestCase):
    """The window used to show a bare 'off' when the Radxa helper could not start."""

    def test_explanations_are_plain_language(self):
        e = L.explain_ssh_failure
        self.assertIn("ssh-copy-id radxa@1.2.3.4", e("Permission denied (publickey,password).", "radxa@1.2.3.4"))
        self.assertIn("cannot reach 1.2.3.4", e("ssh: connect to host 1.2.3.4 port 22: No route to host", "radxa@1.2.3.4"))
        self.assertIn("cannot reach", e("Connection timed out", "radxa@1.2.3.4"))
        self.assertIn("cannot reach", e("Connection refused", "radxa@1.2.3.4"))
        self.assertIn("does not trust", e("Host key verification failed.", "radxa@1.2.3.4"))
        self.assertIn("failed (", e("something odd", "radxa@1.2.3.4"))
        self.assertIn("no details", e("", "radxa@1.2.3.4"))

    def test_a_failed_copy_gives_the_reason(self):
        from unittest import mock
        done = mock.Mock(returncode=255, stdout="", stderr="Permission denied (publickey).")
        with mock.patch.object(L.subprocess, "run", return_value=done):
            ok, why = L.start_helper_checked("radxa@1.2.3.4", 9099)
        self.assertFalse(ok)
        self.assertIn("ssh-copy-id", why)

    def test_a_helper_that_exits_at_once_is_reported(self):
        from unittest import mock
        results = [mock.Mock(returncode=0, stdout="", stderr=""),             # scp ok
                   mock.Mock(returncode=1, stdout="", stderr="")]             # pgrep finds nothing
        with mock.patch.object(L.subprocess, "run", side_effect=results):
            ok, why = L.start_helper_checked("radxa@1.2.3.4", 9099)
        self.assertFalse(ok)
        self.assertIn("exited at once", why)

    def test_success_has_no_reason(self):
        from unittest import mock
        results = [mock.Mock(returncode=0, stdout="", stderr=""), mock.Mock(returncode=0, stdout="4242\n", stderr="")]
        with mock.patch.object(L.subprocess, "run", side_effect=results):
            self.assertEqual(L.start_helper_checked("radxa@1.2.3.4", 9099), (True, ""))

    def test_missing_ssh_and_timeouts(self):
        from unittest import mock
        with mock.patch.object(L.subprocess, "run", side_effect=FileNotFoundError):
            self.assertIn("not installed", L.start_helper_checked("r@h", 1)[1])
        with mock.patch.object(L.subprocess, "run", side_effect=L.subprocess.TimeoutExpired("scp", 20)):
            self.assertIn("timed out", L.start_helper_checked("r@h", 1)[1])

    def test_session_keeps_the_reason_and_reports_it(self):
        from unittest import mock
        cfg = L.SessionConfig(radxa="127.0.0.1", use_video=False, use_udp=True, helper_target="radxa@127.0.0.1")
        s = L.Session(cfg, log=lambda m: None)
        with mock.patch.object(L, "start_helper_checked", return_value=(False, "SSH login failed")), \
                mock.patch.object(L, "WifiSampler"), mock.patch.object(L, "PingSampler"), mock.patch("time.sleep"):
            s.start()
        self.assertFalse(s.analysis.udp_enabled)
        self.assertEqual(s.udp_off_reason, "SSH login failed")
        self.assertEqual(s.udp_summary(), "off - SSH login failed")

    def test_not_selected_is_its_own_reason_and_a_running_probe_has_none(self):
        s = L.Session(L.SessionConfig(radxa="127.0.0.1", use_udp=False), log=lambda m: None)
        self.assertEqual(s.udp_off_reason, "not selected")
        s2 = L.Session(L.SessionConfig(radxa="127.0.0.1", use_udp=True), log=lambda m: None)
        self.assertEqual(s2.udp_off_reason, "")
        self.assertEqual(s2.udp_summary(), "1.0 Mbit/s")
