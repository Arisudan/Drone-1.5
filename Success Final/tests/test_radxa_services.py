"""Radxa-side services: the watchdog's decisions, its status endpoint, and the log janitor.
Hermetic: injected clock and checks, temporary folders, loopback only."""

import gzip
import json
import os
import socket
import tempfile
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer

import _env  # noqa: F401  -- sets sys.path; must import first

import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "radxa"))
import radxa_log_janitor as J  # noqa: E402
import radxa_watchdog as W  # noqa: E402


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, s):
        self.t += s


class Rig:
    def __init__(self, restart_ok=True):
        self.clock = Clock()
        self.restarts = []
        self.router_starts = 0
        self.wd = W.Watchdog(self.clock, restart_pipeline=self._restart, start_router=self._router)
        self.restart_ok = restart_ok

    def _restart(self, reset):
        self.restarts.append(reset)
        return self.restart_ok

    def _router(self):
        self.router_starts += 1
        return True

    def tick(self, advance=5.0, **kw):
        self.clock.advance(advance)
        base = dict(pipeline_state="active", video_ok=True, router_active=True, disk_free_gb=100.0, disk_free_pct=40.0)
        base.update(kw)
        return self.wd.tick(**base)


class WatchdogLogicTest(unittest.TestCase):
    def test_a_healthy_pipeline_is_ok_and_nothing_is_restarted(self):
        r = Rig()
        for _ in range(20):
            st = r.tick()
        self.assertEqual(st["state"], "ok")
        self.assertEqual(r.restarts, [])
        self.assertEqual(st["video"]["ok"], True)

    def test_video_down_long_enough_restarts_the_pipeline_once(self):
        r = Rig()
        r.tick()
        for _ in range(3):
            st = r.tick(video_ok=False)                  # 15 s: not yet
        self.assertEqual(r.restarts, [])
        self.assertEqual(st["state"], "degraded")
        st = r.tick(video_ok=False)                      # 20 s
        st = r.tick(video_ok=False)
        self.assertEqual(r.restarts, [False])
        self.assertIn("video not served", st["restarts"][-1]["reason"])

    def test_a_freshly_restarted_pipeline_gets_a_grace_period(self):
        r = Rig()
        r.tick()
        for _ in range(6):
            r.tick(video_ok=False)
        self.assertEqual(len(r.restarts), 1)
        for _ in range(10):                              # 50 s of no video right after the restart
            st = r.tick(video_ok=False)
        self.assertEqual(len(r.restarts), 1)
        self.assertIn("starting", " ".join(st["messages"]))

    def test_a_pipeline_that_has_only_just_started_is_not_restarted_while_it_boots(self):
        """Found on the real Radxa: a pipeline started at boot or by hand had no grace
        period, so the watchdog restarted it while SLAM was still coming up."""
        r = Rig()
        for k in range(30):                                 # 150 s with no video...
            r.tick(video_ok=False, active_for_s=5.0 * (k + 1))
        self.assertEqual(len(r.restarts), 1)                # ...one restart, after the 90 s grace
        self.assertGreaterEqual(r.clock.t - 1000.0, W.START_GRACE_S)

    def test_a_pipeline_active_for_a_few_seconds_is_never_restarted(self):
        r = Rig()
        for _ in range(8):                                  # 40 s, active_for always under the grace
            st = r.tick(video_ok=False, active_for_s=10.0)
        self.assertEqual(r.restarts, [])
        self.assertEqual(st["state"], "degraded")
        self.assertIn("starting", " ".join(st["messages"]))

    def test_a_failed_unit_is_reset_and_restarted(self):
        r = Rig()
        st = r.tick(pipeline_state="failed", video_ok=False)
        self.assertEqual(r.restarts, [True])
        self.assertIn("failed", st["restarts"][-1]["reason"])

    def test_a_pipeline_stopped_on_purpose_is_left_alone(self):
        r = Rig()
        for _ in range(30):
            st = r.tick(pipeline_state="inactive", video_ok=False)
        self.assertEqual(r.restarts, [])
        self.assertEqual(st["state"], "stopped")             # reported, but not an emergency
        self.assertIn("stopped", " ".join(st["messages"]))

    def test_a_pipeline_started_by_hand_still_counts_as_alive(self):
        r = Rig()
        st = r.tick(pipeline_state="inactive", video_ok=True)
        self.assertEqual(st["state"], "ok")

    def test_it_gives_up_instead_of_restarting_in_a_loop(self):
        r = Rig()
        for _ in range(8):
            r.tick(pipeline_state="failed", video_ok=False)
        self.assertEqual(len(r.restarts), W.MAX_RESTARTS)
        st = r.tick(pipeline_state="failed", video_ok=False)
        self.assertTrue(st["gave_up"])
        self.assertEqual(st["state"], "down")
        self.assertIn("gave up", " ".join(st["messages"]))
        st = r.tick(video_ok=True)                       # healthy again: the give-up clears
        self.assertFalse(st["gave_up"])
        self.assertEqual(st["state"], "ok")

    def test_old_restarts_leave_the_rate_limit_window(self):
        r = Rig()
        for _ in range(W.MAX_RESTARTS):
            r.tick(pipeline_state="failed", video_ok=False)
        r.clock.advance(W.RESTART_WINDOW_S + 1)
        r.tick(pipeline_state="failed", video_ok=False)
        self.assertEqual(len(r.restarts), W.MAX_RESTARTS + 1)

    def test_a_dead_router_is_started_and_not_hammered(self):
        r = Rig()
        r.tick(router_active=False)
        r.tick(router_active=False)
        r.tick(router_active=False)
        self.assertEqual(r.router_starts, 1)
        r.clock.advance(301)
        r.tick(router_active=False)
        self.assertEqual(r.router_starts, 2)
        self.assertEqual(r.tick(router_active=False)["state"], "degraded")

    def test_status_carries_what_the_gcs_needs(self):
        r = Rig()
        st = r.tick()
        for key in ("version", "time", "state", "gave_up", "pipeline", "video", "router", "disk", "restarts", "messages"):
            self.assertIn(key, st)
        self.assertEqual(st["disk"]["free_gb"], 100.0)
        json.dumps(st)                                   # must be serialisable as is


class StatusEndpointTest(unittest.TestCase):
    def test_status_is_served_over_http_and_other_paths_are_404(self):
        rig = Rig()
        rig.tick()
        srv = ThreadingHTTPServer(("127.0.0.1", 0), W.make_handler(lambda: dict(rig.wd.status)))
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            port = srv.server_address[1]
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/status", timeout=3) as r:
                body = json.loads(r.read())
            self.assertEqual(body["state"], "ok")
            with self.assertRaises(urllib.error.HTTPError) as cm:
                urllib.request.urlopen(f"http://127.0.0.1:{port}/etc/passwd", timeout=3)
            self.assertEqual(cm.exception.code, 404)
        finally:
            srv.shutdown()
            srv.server_close()


class VideoCheckTest(unittest.TestCase):
    def serve(self, payload):
        srv = socket.socket()
        srv.bind(("127.0.0.1", 0))
        srv.listen(2)

        def run():
            try:
                conn, _ = srv.accept()
                conn.recv(1024)
                conn.sendall(payload)
                time.sleep(0.3)
                conn.close()
            except OSError:
                pass
        threading.Thread(target=run, daemon=True).start()
        return srv

    def test_a_streamer_that_sends_a_jpeg_counts_as_serving(self):
        srv = self.serve(b"HTTP/1.0 200 OK\r\n\r\n--F\r\n\r\n\xff\xd8" + b"x" * 100 + b"\xff\xd9\r\n")
        try:
            self.assertTrue(W.video_served(srv.getsockname()[1], timeout=2))
        finally:
            srv.close()

    def test_a_streamer_that_answers_but_sends_no_picture_does_not(self):
        srv = self.serve(b"HTTP/1.0 200 OK\r\n\r\n--F\r\n\r\n")
        try:
            self.assertFalse(W.video_served(srv.getsockname()[1], timeout=1))
        finally:
            srv.close()

    def test_nothing_listening_is_not_serving(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        self.assertFalse(W.video_served(port, timeout=1))


# ── janitor ───────────────────────────────────────────────────────────────

NOW = 2_000_000_000.0
DAY = 86400.0


class JanitorTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.ros = os.path.join(self.tmp.name, "ros_log")
        self.flop = os.path.join(self.tmp.name, "Flop")
        os.makedirs(self.ros)
        os.makedirs(self.flop)
        self.pol = J.Policy(ros_log_dir=self.ros, flop_dir=self.flop, keep_days=14, keep_newest=2)

    def tearDown(self):
        self.tmp.cleanup()

    def ros_dir(self, name, age_days, size=1000):
        p = os.path.join(self.ros, name)
        os.makedirs(p)
        with open(os.path.join(p, "launch.log"), "wb") as fh:
            fh.write(b"x" * size)
        t = NOW - age_days * DAY
        os.utime(os.path.join(p, "launch.log"), (t, t))
        os.utime(p, (t, t))
        return p

    def test_only_old_ros_folders_beyond_the_newest_few_are_removed(self):
        old1 = self.ros_dir("2026-01-01-00-00-00-111111-h-1", 60)
        old2 = self.ros_dir("2026-02-01-00-00-00-111111-h-2", 50)
        keep_new = self.ros_dir("2026-09-01-00-00-00-111111-h-3", 3)
        keep_recent_old = self.ros_dir("2026-08-01-00-00-00-111111-h-4", 5)
        acts = J.plan(self.pol, NOW)
        paths = {a.path for a in acts}
        self.assertEqual(paths, {old1, old2})
        self.assertTrue(all(a.kind == "delete" for a in acts))
        self.assertNotIn(keep_new, paths)
        self.assertNotIn(keep_recent_old, paths)

    def node_file(self, name, age_days, size=100):
        p = os.path.join(self.ros, name)
        with open(p, "wb") as fh:
            fh.write(b"x" * size)
        t = NOW - age_days * DAY
        os.utime(p, (t, t))
        return p

    def test_old_per_node_log_files_are_removed_and_recent_ones_kept(self):
        self.pol.keep_newest_files = 2
        old = self.node_file("stereo_odometry_9234_1785252255520.log", 40, size=5000)
        old2 = self.node_file("rtabmap_111_1785252255521.log", 30)
        newest = self.node_file("stereo_odometry_5_1790000000000.log", 20)
        young = self.node_file("rtabmap_7_1790000000001.log", 2)
        other = self.node_file("notes.txt", 400)                  # not a ROS node log: untouched
        paths = {a.path for a in J.plan(self.pol, NOW)}
        self.assertEqual(paths, {old, old2})
        self.assertNotIn(newest, paths)
        self.assertNotIn(young, paths)
        self.assertNotIn(other, paths)
        J.apply(J.plan(self.pol, NOW), NOW)
        self.assertFalse(os.path.exists(old))
        self.assertTrue(os.path.exists(other))

    def test_the_newest_folders_are_kept_even_when_all_are_old(self):
        a = self.ros_dir("2026-01-01-00-00-00-1-h-1", 90)
        b = self.ros_dir("2026-01-02-00-00-00-1-h-2", 80)
        c = self.ros_dir("2026-01-03-00-00-00-1-h-3", 70)
        self.assertEqual({x.path for x in J.plan(self.pol, NOW)}, {a})      # keep_newest=2 keeps b and c
        self.assertTrue(b and c)

    def test_things_that_are_not_ros_log_folders_are_never_touched(self):
        os.makedirs(os.path.join(self.ros, "latest_note"))
        t = NOW - 100 * DAY
        os.utime(os.path.join(self.ros, "latest_note"), (t, t))
        open(os.path.join(self.ros, "2026-01-01-00-00-00-1-h-9"), "w").close()      # a FILE with a log-like name
        os.symlink(self.tmp.name, os.path.join(self.ros, "2026-01-01-00-00-00-1-h-link"))
        self.assertEqual(J.plan(self.pol, NOW), [])

    def test_low_disk_makes_it_stricter(self):
        self.ros_dir("2026-09-10-00-00-00-1-h-1", 1)
        self.ros_dir("2026-09-09-00-00-00-1-h-2", 2)
        mid = self.ros_dir("2026-09-01-00-00-00-1-h-3", 5)
        self.ros_dir("2026-08-01-00-00-00-1-h-4", 6)
        self.assertEqual(J.plan(self.pol, NOW, free_pct=50.0), [])
        low = J.plan(self.pol, NOW, free_pct=5.0)
        self.assertIn(mid, {a.path for a in low})
        self.assertTrue(all("disk low" in a.reason for a in low))

    def test_apply_deletes_exactly_the_planned_folders(self):
        old = self.ros_dir("2026-01-01-00-00-00-1-h-1", 60, size=5000)
        new1 = self.ros_dir("2026-09-01-00-00-00-1-h-2", 2)
        new2 = self.ros_dir("2026-09-02-00-00-00-1-h-3", 1)
        acts = J.plan(self.pol, NOW)
        freed = J.apply(acts, NOW)
        self.assertFalse(os.path.exists(old))
        self.assertTrue(os.path.isdir(new1) and os.path.isdir(new2))
        self.assertGreaterEqual(freed, 5000)

    def test_a_big_telemetry_log_is_compressed_and_emptied_in_place(self):
        self.pol.rotate_mb = 0.0005                         # ~500 bytes
        p = os.path.join(self.flop, "mav.tlog")
        with open(p, "wb") as fh:
            fh.write(b"telemetry " * 500)
        acts = J.plan(self.pol, NOW)
        self.assertEqual([a.kind for a in acts], ["rotate"])
        J.apply(acts, NOW)
        self.assertEqual(os.path.getsize(p), 0)
        gz = [f for f in os.listdir(self.flop) if f.endswith(".gz")]
        self.assertEqual(len(gz), 1)
        with gzip.open(os.path.join(self.flop, gz[0])) as fh:
            self.assertEqual(fh.read(), b"telemetry " * 500)

    def test_a_small_or_empty_telemetry_log_is_left_alone(self):
        open(os.path.join(self.flop, "mav.tlog"), "wb").close()
        self.assertEqual(J.plan(self.pol, NOW), [])

    def test_old_compressed_logs_are_pruned_beyond_the_newest_few(self):
        self.pol.keep_tlog = 2
        for i, age in enumerate((100, 90, 5, 3)):
            p = os.path.join(self.flop, f"mav.tlog.2026010{i + 1}-000000.gz")
            with open(p, "wb") as fh:
                fh.write(b"z")
            t = NOW - age * DAY
            os.utime(p, (t, t))
        gone = {os.path.basename(a.path) for a in J.plan(self.pol, NOW)}
        self.assertEqual(gone, {"mav.tlog.20260101-000000.gz", "mav.tlog.20260102-000000.gz"})

    def test_a_dry_run_changes_nothing(self):
        old = self.ros_dir("2026-01-01-00-00-00-1-h-1", 60)
        self.ros_dir("2026-09-01-00-00-00-1-h-2", 2)
        self.ros_dir("2026-09-02-00-00-00-1-h-3", 1)
        self.assertEqual(J.main(["--ros-log-dir", self.ros, "--flop-dir", self.flop]), 0)
        self.assertTrue(os.path.isdir(old))


class DeployFilesTest(unittest.TestCase):
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "radxa")

    def read(self, rel):
        with open(os.path.join(self.base, rel), encoding="utf-8") as fh:
            return fh.read()

    def test_the_pipeline_unit_restarts_on_failure_but_not_forever(self):
        u = self.read("deploy/drone-pipeline.service")
        self.assertIn("ExecStart=@HOME@/Flop/camera.sh", u)
        self.assertIn("Restart=on-failure", u)
        self.assertIn("StartLimitBurst=5", u)
        self.assertIn("WantedBy=multi-user.target", u)
        self.assertIn("User=@USER@", u)

    def test_every_unit_uses_placeholders_not_a_hardcoded_home(self):
        for name in ("drone-pipeline.service", "drone-watchdog.service", "drone-janitor.service"):
            self.assertNotIn("/home/radxa", self.read("deploy/" + name), name)

    def test_the_janitor_service_applies_and_the_timer_is_daily(self):
        self.assertIn("radxa_log_janitor.py --apply", self.read("deploy/drone-janitor.service"))
        self.assertIn("OnCalendar=daily", self.read("deploy/drone-janitor.timer"))

    def test_the_installer_enables_but_does_not_start_the_pipeline_by_default(self):
        sh = self.read("install_radxa_services.sh")
        self.assertIn("systemctl enable drone-pipeline.service", sh)
        self.assertIn('if [ "$START" -eq 1 ]', sh)
        self.assertIn("--uninstall", sh)


if __name__ == "__main__":
    unittest.main()
