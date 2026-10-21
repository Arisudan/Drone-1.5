"""The window front-end (scripts/diagnostics/link_range_gui.py).

Runs on its OWN virtual display (Xvfb) so it never pops windows up on the desktop,
and is skipped when tkinter or Xvfb is missing. The camera and the Radxa are
replaced by local stand-ins on loopback.
"""

import http.server
import os
import shutil
import socket
import socketserver
import subprocess
import tempfile
import threading
import time
import unittest
from unittest import mock

import _env  # noqa: F401  -- sets sys.path; must import first

try:
    import tkinter  # noqa: F401
    HAVE_TK = True
except Exception:                                   # noqa: BLE001
    HAVE_TK = False

_XVFB = None


def setUpModule():
    """Start a private X server and point DISPLAY at it."""
    global _XVFB
    if not HAVE_TK or os.environ.get("LRT_GUI_TEST_USE_DISPLAY") == "1":
        return
    exe = shutil.which("Xvfb")
    if not exe:
        return
    for n in range(90, 120):
        if os.path.exists(f"/tmp/.X11-unix/X{n}"):
            continue
        _XVFB = subprocess.Popen([exe, f":{n}", "-screen", "0", "1100x800x24"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        os.environ["DISPLAY"] = f":{n}"
        os.environ.pop("WAYLAND_DISPLAY", None)
        for _ in range(50):                         # wait for the socket
            if os.path.exists(f"/tmp/.X11-unix/X{n}"):
                break
            time.sleep(0.1)
        return


def tearDownModule():
    if _XVFB is not None:
        _XVFB.terminate()
        _XVFB.wait(timeout=5)


def have_display():
    return HAVE_TK and (_XVFB is not None or os.environ.get("LRT_GUI_TEST_USE_DISPLAY") == "1")


@unittest.skipUnless(HAVE_TK, "tkinter not installed")
class GuiBase(unittest.TestCase):
    def setUp(self):
        if not have_display():
            self.skipTest("no Xvfb for an isolated display")
        import link_range_test as L
        import link_range_gui as G
        self.L, self.G = L, G
        self.tmp = tempfile.mkdtemp()
        self._prefs = L.PREFS_PATH
        L.PREFS_PATH = os.path.join(self.tmp, "prefs.json")
        self.apps = []

    def tearDown(self):
        self.L.PREFS_PATH = self._prefs
        for app in self.apps:
            try:
                if getattr(app, "run_screen", None) and app.run_screen.session.started:
                    app.run_screen.session.stop()
                app.destroy()
            except Exception:                       # noqa: BLE001
                pass
        shutil.rmtree(self.tmp, ignore_errors=True)

    def app(self, radxa=None):
        args = type("A", (), {"radxa": radxa})() if radxa else None
        a = self.G.App(args)
        self.apps.append(a)
        a.update()
        return a

    @staticmethod
    def pump(app, secs):
        end = time.time() + secs
        while time.time() < end:
            app.update()
            time.sleep(0.03)

    def until(self, app, cond, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            app.update()
            if cond():
                return True
            time.sleep(0.03)
        return False


class SetupScreenTest(GuiBase):
    def test_starts_on_the_setup_screen_asking_for_the_ip(self):
        a = self.app()
        self.assertIsInstance(a.frame, self.G.SetupScreen)
        self.assertEqual(a.title(), "Link range test")

    def test_ip_prefill_order_argument_then_saved_then_gcs(self):
        self.L.save_prefs({"radxa": "10.1.1.1"})
        with mock.patch.object(self.L, "gcs_saved_host", return_value="10.9.9.9"):
            self.assertEqual(self.app().setup.var_ip.get(), "10.1.1.1")
            self.assertEqual(self.app("10.2.2.2").setup.var_ip.get(), "10.2.2.2")
            os.remove(self.L.PREFS_PATH)
            self.assertEqual(self.app().setup.var_ip.get(), "10.9.9.9")

    def test_a_valid_ip_builds_the_session_config_with_ssh_targets(self):
        a = self.app("172.16.100.182")
        cfg = a.setup.build_config()
        self.assertEqual(cfg.radxa, "172.16.100.182")
        self.assertEqual(cfg.helper_target, "radxa@172.16.100.182")
        self.assertEqual(cfg.control_target, "radxa@172.16.100.182")
        self.assertTrue(cfg.use_video and cfg.use_udp)
        self.assertEqual((cfg.step, cfg.hold, cfg.loss_tolerance), (5.0, 15.0, 0.0))
        self.assertEqual(a.setup.lbl_err.cget("text"), "")

    def test_unticking_options_clears_the_matching_targets(self):
        a = self.app("1.2.3.4")
        a.setup.var_udp.set(False)
        a.setup.var_control.set(False)
        a.setup.var_video.set(False)
        cfg = a.setup.build_config()
        self.assertEqual((cfg.helper_target, cfg.control_target, cfg.use_video, cfg.use_udp), ("", "", False, False))

    def test_a_bad_ip_is_refused_with_a_message_not_a_crash(self):
        a = self.app("not an ip")
        self.assertIsNone(a.setup.build_config())
        self.assertIn("valid IP", a.setup.lbl_err.cget("text"))
        a.setup.var_ip.set("1.2.3")
        self.assertIsNone(a.setup.build_config())

    def test_bad_advanced_numbers_are_refused_and_the_advanced_box_opens(self):
        a = self.app("1.2.3.4")
        a.setup.var_step.set("abc")
        self.assertIsNone(a.setup.build_config())
        self.assertIn("Metres", a.setup.lbl_err.cget("text"))
        self.assertTrue(a.setup.var_adv.get())
        a.setup.var_step.set("5")
        a.setup.var_hold.set("1")                      # below the 3 s minimum
        self.assertIsNone(a.setup.build_config())
        a.setup.var_hold.set("15")
        a.setup.var_tol.set("-1")
        self.assertIsNone(a.setup.build_config())

    def test_custom_advanced_values_flow_into_the_config(self):
        a = self.app("1.2.3.4")
        a.setup.var_step.set("3")
        a.setup.var_hold.set("10")
        a.setup.var_tol.set("1.5")
        a.setup.var_user.set("pi")
        a.setup.var_vport.set("9090")
        cfg = a.setup.build_config()
        self.assertEqual((cfg.step, cfg.hold, cfg.baseline_hold, cfg.loss_tolerance), (3.0, 10.0, 10.0, 1.5))
        self.assertEqual((cfg.ssh_user, cfg.video_port, cfg.helper_target), ("pi", 9090, "pi@1.2.3.4"))

    def test_ok_saves_the_choices_for_next_time(self):
        a = self.app("172.16.100.182")
        with mock.patch.object(self.L.Session, "preflight", return_value={"reachable": True, "ssid": "X"}), \
                mock.patch.object(self.G.App, "show_run") as show:
            a.setup.ok()
            self.assertTrue(self.until(a, lambda: show.called))
        self.assertEqual(self.L.load_prefs()["radxa"], "172.16.100.182")
        self.assertEqual(show.call_args[0][0].radxa, "172.16.100.182")

    def test_unreachable_radxa_asks_and_stays_put_on_no(self):
        a = self.app("172.16.100.182")
        with mock.patch.object(self.L.Session, "preflight", return_value={"reachable": False, "ssid": "HTIC_INCUBATION"}), \
                mock.patch.object(self.G.messagebox, "askyesno", return_value=False) as ask, \
                mock.patch.object(self.G.App, "show_run") as show:
            a.setup.ok()
            self.assertTrue(self.until(a, lambda: ask.called))
            a.update()
        self.assertFalse(show.called)
        self.assertIn("HTIC_INCUBATION", ask.call_args[0][1])
        self.assertIsInstance(a.frame, self.G.SetupScreen)

    def test_unreachable_radxa_proceeds_on_yes(self):
        a = self.app("172.16.100.182")
        with mock.patch.object(self.L.Session, "preflight", return_value={"reachable": False, "ssid": "Y"}), \
                mock.patch.object(self.G.messagebox, "askyesno", return_value=True), \
                mock.patch.object(self.G.App, "show_run") as show:
            a.setup.ok()
            self.assertTrue(self.until(a, lambda: show.called))

    def test_check_connection_reports_in_the_window(self):
        a = self.app("1.2.3.4")
        with mock.patch.object(self.L.Session, "preflight", return_value={"reachable": True, "ssid": "Z"}):
            a.setup.check()
            self.assertTrue(self.until(a, lambda: "answers" in a.setup.lbl_status.cget("text")))
        with mock.patch.object(self.L.Session, "preflight", return_value={"reachable": False, "ssid": "Z"}):
            a.setup.check()
            self.assertTrue(self.until(a, lambda: "No answer" in a.setup.lbl_status.cget("text")))
        self.assertIn("'Z'", a.setup.lbl_status.cget("text"))

    def test_check_with_an_invalid_ip_does_not_ping(self):
        a = self.app("zzz zzz")
        with mock.patch.object(self.L.Session, "preflight") as pre:
            a.setup.check()
            a.update()
        self.assertFalse(pre.called)

    def test_the_network_name_is_shown(self):
        with mock.patch.object(self.G.SetupScreen, "_network_info", return_value="This laptop is on Wi-Fi network: TESTNET  (-60 dBm)"):
            a = self.app("1.2.3.4")
            self.assertTrue(self.until(a, lambda: "TESTNET" in a.setup.lbl_net.cget("text")))


# ── loopback stand-ins for the camera and the Radxa helper ───────────────

class StandIns:
    def __init__(self):
        import link_probe_server as S
        frame = b"\xff\xd8" + b"\x01\x02\x03" * 300 + b"\xff\xd9"

        class H(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        self.wfile.write(b"--frame\r\n\r\n" + frame + b"\r\n")
                        self.wfile.flush()
                        time.sleep(0.05)
                except OSError:
                    pass

        class HS(socketserver.ThreadingMixIn, http.server.HTTPServer):
            daemon_threads = True

        self.httpd = HS(("127.0.0.1", 0), H)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.stop = threading.Event()

        def echo():
            sessions = {}
            while not self.stop.is_set():
                try:
                    d, a = self.sock.recvfrom(4096)
                except (socket.timeout, OSError):
                    continue
                r = S.handle_datagram(d, a, sessions, time.monotonic())
                if r:
                    self.sock.sendto(r, a)

        threading.Thread(target=echo, daemon=True).start()

    def config(self, L, out_dir, hold=2.0, **kw):
        return L.SessionConfig(radxa="127.0.0.1", video_url=f"http://127.0.0.1:{self.httpd.server_address[1]}/video",
                               use_udp=True, udp_port=self.sock.getsockname()[1], udp_mbps=1.0,
                               step=5.0, hold=hold, baseline_hold=hold, out_dir=out_dir, **kw)

    def close(self):
        self.stop.set()
        self.httpd.shutdown()
        self.sock.close()


class RunFlowTest(GuiBase):
    def setUp(self):
        super().setUp()
        self.si = StandIns()

    def tearDown(self):
        self.si.close()
        super().tearDown()

    def start(self, **kw):
        a = self.app()
        a.show_run(self.si.config(self.L, os.path.join(self.tmp, "out"), **kw))
        r = a.run_screen
        self.assertTrue(self.until(a, lambda: r.state == "baseline_wait", 15))
        return a, r

    def test_it_waits_at_the_baseline_until_the_button_is_pressed(self):
        a, r = self.start()
        self.assertEqual(str(r.btn_main["state"]), "normal")
        self.assertIn("0 m", r.btn_main.cget("text"))
        self.assertEqual(self.session_holds(r), 0)

    @staticmethod
    def session_holds(r):
        return len(r.session.holds)

    def test_the_live_tiles_show_real_numbers_from_the_running_test(self):
        a, r = self.start()
        self.pump(a, 2.0)
        self.assertIn("fps", r.t_video.value.cget("text"))
        self.assertIn("%", r.t_ping.value.cget("text"))
        self.assertIn("/", r.t_udp.value.cget("text"))

    def test_full_walk_baseline_then_a_mark_then_result(self):
        a, r = self.start()
        r.on_main()                                             # measure at 0 m
        self.assertEqual(r.state, "holding")
        self.assertEqual(str(r.btn_main["state"]), "disabled")
        self.assertTrue(self.until(a, lambda: r.state == "between", 8))
        self.assertEqual(len(r.tree.get_children()), 1)
        self.assertEqual(r.var_dist.get(), "5")                  # next = last + step
        r.var_dist.set("7.5")
        r.on_main()
        self.assertTrue(self.until(a, lambda: r.state == "between" and len(r.tree.get_children()) == 2, 8))
        self.assertEqual(r.var_dist.get(), "12.5")
        r.finish()
        self.assertTrue(self.until(a, lambda: isinstance(a.frame, self.G.ResultScreen), 15))
        res = a.result
        self.assertTrue(os.path.exists(res.report))
        self.assertEqual(sorted(h.distance_m for h in r.session.holds), [0.0, 7.5])
        files = set(os.listdir(os.path.join(self.tmp, "out")))
        self.assertTrue({"report.html", "summary.json", "holds.json", "samples.csv"} <= files)

    def test_the_result_leads_with_the_loss_free_distance(self):
        a, r = self.start()
        r.on_main()
        self.assertTrue(self.until(a, lambda: r.state == "between", 8))
        r.finish()
        self.assertTrue(self.until(a, lambda: isinstance(a.frame, self.G.ResultScreen), 15))
        lf = a.result.summary["ranges"]["loss_free_to"]
        shown = a.result.lbl_big.cget("text")
        self.assertEqual(shown, "–" if lf is None else f"{lf:g} m")
        self.assertIn("loss-free", a.result.summary["findings"][0])

    def test_a_bad_distance_is_refused_and_nothing_starts(self):
        a, r = self.start()
        r.on_main()
        self.assertTrue(self.until(a, lambda: r.state == "between", 8))
        for bad in ("abc", "0", "-3", ""):
            r.var_dist.set(bad)
            r.on_main()
            self.assertEqual(r.state, "between", bad)
        self.assertEqual(len(r.session.holds), 1)

    def test_finishing_with_nothing_measured_asks_first(self):
        a, r = self.start()
        with mock.patch.object(self.G.messagebox, "askyesno", return_value=False) as ask:
            r.finish()
        self.assertTrue(ask.called)
        self.assertFalse(r._fin)                                 # declined: keeps going
        with mock.patch.object(self.G.messagebox, "askyesno", return_value=True), \
                mock.patch.object(a, "destroy") as destroy:
            r.finish()
        self.assertTrue(destroy.called)

    def test_closing_the_window_mid_run_offers_to_save(self):
        a, r = self.start()
        r.on_main()
        self.assertTrue(self.until(a, lambda: r.state == "between", 8))
        with mock.patch.object(self.G.messagebox, "askyesno", return_value=False):
            a._on_close()
        self.assertFalse(r._fin)
        with mock.patch.object(self.G.messagebox, "askyesno", return_value=True):
            a._on_close()
        self.assertTrue(self.until(a, lambda: isinstance(a.frame, self.G.ResultScreen), 15))

    def test_open_report_uses_a_file_url(self):
        a, r = self.start()
        r.on_main()
        self.assertTrue(self.until(a, lambda: r.state == "between", 8))
        r.finish()
        self.assertTrue(self.until(a, lambda: isinstance(a.frame, self.G.ResultScreen), 15))
        with mock.patch.object(self.G.webbrowser, "open") as op:
            a.result.open_report()
        self.assertTrue(op.call_args[0][0].startswith("file:///"))
        self.assertTrue(op.call_args[0][0].endswith("report.html"))

    def test_run_again_returns_to_setup(self):
        a, r = self.start()
        r.on_main()
        self.assertTrue(self.until(a, lambda: r.state == "between", 8))
        r.finish()
        self.assertTrue(self.until(a, lambda: isinstance(a.frame, self.G.ResultScreen), 15))
        a.result.again()
        self.assertIsInstance(a.frame, self.G.SetupScreen)


class WalkModeSetupTest(GuiBase):
    def test_the_mode_defaults_to_holds_and_flows_into_the_config(self):
        a = self.app("1.2.3.4")
        self.assertEqual(a.setup.build_config().mode, "holds")
        a.setup.var_mode.set("walk")
        self.assertEqual(a.setup.build_config().mode, "walk")

    def test_the_chosen_mode_is_remembered(self):
        self.L.save_prefs({"radxa": "1.2.3.4", "mode": "walk"})
        a = self.app()
        self.assertEqual(a.setup.var_mode.get(), "walk")
        self.L.save_prefs({"radxa": "1.2.3.4", "mode": "nonsense"})
        self.assertEqual(self.app().setup.var_mode.get(), "holds")


class WalkFlowTest(GuiBase):
    def setUp(self):
        super().setUp()
        self.si = StandIns()

    def tearDown(self):
        self.si.close()
        super().tearDown()

    def start(self):
        a = self.app()
        a.show_run(self.si.config(self.L, os.path.join(self.tmp, "out"), mode="walk"))
        r = a.run_screen
        self.assertIsInstance(r, self.G.WalkScreen)
        self.assertTrue(self.until(a, lambda: r.state == "ready", 15))
        return a, r

    def test_it_waits_for_the_start_button(self):
        a, r = self.start()
        self.assertIn("Start walking", r.btn_main.cget("text"))
        self.pump(a, 1.5)
        self.assertEqual(len(r.tree.get_children()), 0)          # nothing streams before the button

    def test_values_stream_once_a_second_after_start(self):
        a, r = self.start()
        r.on_main()
        self.assertEqual(r.state, "walking")
        self.assertTrue(self.until(a, lambda: len(r.tree.get_children()) >= 3, 10))
        self.assertIn("s", r.t_time.value.cget("text"))
        self.assertIn("fps", r.t_video.value.cget("text"))
        self.assertIn("Loss-free", r.lbl_head.cget("text"))
        self.assertIn("Stop", r.btn_main.cget("text"))

    def test_stop_writes_the_report_and_shows_seconds_not_distance(self):
        a, r = self.start()
        r.on_main()
        self.assertTrue(self.until(a, lambda: len(r.tree.get_children()) >= 3, 10))
        r.on_main()                                              # the same button now stops
        self.assertTrue(self.until(a, lambda: isinstance(a.frame, self.G.WalkResultScreen), 15))
        res = a.result
        self.assertTrue(os.path.exists(res.report))
        self.assertTrue(res.lbl_big.cget("text").endswith(" s") or res.lbl_big.cget("text") == "–")
        files = set(os.listdir(os.path.join(self.tmp, "out")))
        self.assertTrue({"report.html", "summary.json", "walk.json", "samples.csv"} <= files)
        self.assertEqual(res.summary["mode"], "walk")

    def test_stopping_before_starting_asks_first(self):
        a, r = self.start()
        with mock.patch.object(self.G.messagebox, "askyesno", return_value=False) as ask:
            r.finish()
        self.assertTrue(ask.called)
        self.assertFalse(r._fin)

    def test_closing_the_window_mid_walk_offers_to_save(self):
        a, r = self.start()
        r.on_main()
        self.assertTrue(self.until(a, lambda: len(r.tree.get_children()) >= 2, 10))
        with mock.patch.object(self.G.messagebox, "askyesno", return_value=True):
            a._on_close()
        self.assertTrue(self.until(a, lambda: isinstance(a.frame, self.G.WalkResultScreen), 15))

    def test_run_again_returns_to_setup(self):
        a, r = self.start()
        r.on_main()
        self.assertTrue(self.until(a, lambda: len(r.tree.get_children()) >= 2, 10))
        r.on_main()
        self.assertTrue(self.until(a, lambda: isinstance(a.frame, self.G.WalkResultScreen), 15))
        a.result.again()
        self.assertIsInstance(a.frame, self.G.SetupScreen)


class UdpOffReasonTest(GuiBase):
    """A bare 'off' on the UDP tile used to hide why the Radxa helper did not start."""

    def setUp(self):
        super().setUp()
        self.si = StandIns()

    def tearDown(self):
        self.si.close()
        super().tearDown()

    def run_screen(self, mode):
        a = self.app()
        cfg = self.si.config(self.L, os.path.join(self.tmp, "out"), mode=mode, helper_target="radxa@127.0.0.1")
        with mock.patch.object(self.L, "start_helper_checked",
                               return_value=(False, "SSH login to radxa@127.0.0.1 failed - no key is set up")):
            a.show_run(cfg)
            r = a.run_screen
            want = "baseline_wait" if mode == "holds" else "ready"
            self.assertTrue(self.until(a, lambda: r.state == want, 15))
        return a, r

    def test_marked_holds_screen_says_why_udp_is_off(self):
        a, r = self.run_screen("holds")
        self.pump(a, 1.0)
        self.assertIn("NOT being measured", r.lbl_udp_note.cget("text"))
        self.assertIn("no key is set up", r.lbl_udp_note.cget("text"))
        self.assertEqual(r.t_udp.value.cget("text"), "off")
        self.assertIn("helper did not start", r.t_udp.sub.cget("text"))

    def test_walk_screen_says_why_udp_is_off(self):
        a, r = self.run_screen("walk")
        r.on_main()
        self.assertTrue(self.until(a, lambda: len(r.tree.get_children()) >= 1, 10))
        self.assertIn("no key is set up", r.lbl_udp_note.cget("text"))
        self.assertEqual(r.t_udp.value.cget("text"), "off")

    def test_the_result_repeats_the_reason_and_the_report_records_it(self):
        a, r = self.run_screen("walk")
        r.on_main()
        self.assertTrue(self.until(a, lambda: len(r.tree.get_children()) >= 2, 10))
        r.on_main()
        self.assertTrue(self.until(a, lambda: isinstance(a.frame, self.G.WalkResultScreen), 15))
        texts = [w.cget("text") for w in a.frame.winfo_children() if w.winfo_class() == "TLabel"]
        self.assertTrue(any("NOT being measured" in t for t in texts), texts)
        with open(os.path.join(self.tmp, "out", "report.md"), encoding="utf-8") as fh:
            self.assertIn("off - SSH login", fh.read())

    def test_not_selected_is_stated_plainly(self):
        a = self.app()
        cfg = self.si.config(self.L, os.path.join(self.tmp, "out"), mode="walk")
        cfg.use_udp = False
        a.show_run(cfg)
        r = a.run_screen
        self.assertTrue(self.until(a, lambda: r.state == "ready", 15))
        self.assertIn("not ticked", r.lbl_udp_note.cget("text"))
        r.on_main()
        self.assertTrue(self.until(a, lambda: len(r.tree.get_children()) >= 1, 10))
        self.assertIn("not selected", r.t_udp.sub.cget("text"))

    def test_when_udp_works_there_is_no_warning(self):
        a = self.app()
        a.show_run(self.si.config(self.L, os.path.join(self.tmp, "out"), mode="walk"))
        r = a.run_screen
        self.assertTrue(self.until(a, lambda: r.state == "ready", 15))
        self.assertEqual(r.lbl_udp_note.cget("text"), "")


class ThemeAndHelpersTest(GuiBase):
    def test_signal_colours_follow_the_thresholds(self):
        G = self.G
        self.assertEqual(G.signal_colour(-60), G.BRIGHT)
        self.assertEqual(G.signal_colour(-75), G.WARN)
        self.assertEqual(G.signal_colour(-85), G.BAD)
        self.assertEqual(G.signal_colour(None), G.MUTED)

    def test_loss_colours_stay_neutral_until_there_is_loss(self):
        G = self.G
        self.assertEqual(G.loss_colour(0.0, 0.0), G.BRIGHT)
        self.assertEqual(G.loss_colour(1.0, 0.0), G.WARN)
        self.assertEqual(G.loss_colour(1.0, 2.0), G.BRIGHT)
        self.assertEqual(G.loss_colour(15.0, 0.0), G.BAD)
        self.assertEqual(G.loss_colour(None, 0.0), G.MUTED)

    def test_fmt(self):
        self.assertEqual(self.G.fmt(None), "–")
        self.assertEqual(self.G.fmt(3.14159, "{:.2f}"), "3.14")


class EntryPointTest(unittest.TestCase):
    def setUp(self):
        import link_range_test as L
        self.L = L

    def test_no_arguments_opens_the_window_when_a_display_exists(self):
        import sys
        import types
        fake = types.ModuleType("link_range_gui")
        fake.launch = mock.Mock(return_value=0)
        with mock.patch.object(self.L, "gui_available", return_value=True), \
                mock.patch.dict(sys.modules, {"link_range_gui": fake}):
            self.assertEqual(self.L.main([]), 0)
        self.assertTrue(fake.launch.called)

    def test_cli_flag_skips_the_window(self):
        import contextlib
        import io
        with mock.patch.object(self.L, "gui_available", return_value=True), \
                contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(self.L.main(["--cli"]), 2)

    def test_without_a_display_it_falls_back_to_asking_in_the_terminal(self):
        import contextlib
        import io
        with mock.patch.object(self.L, "gui_available", return_value=False), \
                mock.patch("builtins.input", return_value="not an ip"), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(self.L.main([]), 2)

    def test_gui_available_is_false_without_a_display(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            if os.name != "nt":
                self.assertFalse(self.L.gui_available())


if __name__ == "__main__":
    unittest.main()
