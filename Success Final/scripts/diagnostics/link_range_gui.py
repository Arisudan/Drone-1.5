#!/usr/bin/env python3
"""
================================================================================
MODULE: link_range_gui.py
PURPOSE: Window front-end for link_range_test.py (no command line needed)
================================================================================

  python3 link_range_test.py          # opens this window

TWO MODES (chosen on the setup screen)
  Hold at marked distances   the original: stand at measured spots, press a button.
  Continuous walk            walk away at your own pace while one line of values
                             streams every second; the verdict is in seconds and
                             dBm - there is no distance (see link_range_walk.py).

ONE WINDOW, THREE SCREENS
  1. Setup    asks for the Radxa IP (and a few options), checks the connection.
  2. Walking  big live readouts and one big button: stand still at a mark, press
              it, hold for the countdown, walk on. Built to be used standing up
              with a laptop in one hand.
  3. Result   the answer first: how far the live feed ran with NO packet loss,
              then the per-distance table and a link to the full report.

Everything measured comes from link_range_test.Session - the same engine the
terminal flow uses - so the window and the command line cannot disagree.

THREADING: Tk is touched only from the main thread. Anything slow (ping, SSH,
starting the samplers, writing the report) runs in a worker thread and hands its
result back through a queue that the main thread polls.
================================================================================
"""

from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
import webbrowser
from tkinter import font as tkfont
from tkinter import messagebox, ttk
from typing import Callable, Dict, List, Optional

import link_range_test as L
import link_range_walk as W

BG, PANEL, FIELD = "#0d1117", "#161b22", "#21262d"
BORDER, TEXT, BRIGHT, MUTED = "#30363d", "#c9d1d9", "#f0f6fc", "#8b949e"
ACCENT, ACCENT_DIM, WARN, BAD = "#58a6ff", "#1f6feb", "#d29922", "#f85149"

LIVE_WINDOW_S = 5.0
TICK_MS = 500

# ── Tk must never be finalised off the main thread ─────────────────────────────
# Python's cyclic garbage collector can run inside ANY thread - including the
# sampler threads this tool runs. If it then collects a Tk variable or an
# already-destroyed window there, Tcl is touched from the wrong thread: the
# variable's finaliser raises "main thread is not in main loop", and deleting the
# interpreter aborts the whole process ("Tcl_AsyncDelete: async handler deleted
# by the wrong thread"). Two guards:
#   * tk.Variable never runs its finaliser (variables are freed when their
#     interpreter is destroyed, which happens on the main thread);
#   * every App is kept referenced until the process exits, so its interpreter is
#     only ever deleted by the main thread at shutdown.
tk.Variable.__del__ = lambda self: None          # type: ignore[assignment,method-assign]
_KEEP_ALIVE: List[tk.Tk] = []


# ───────────────────────────── theme ───────────────────────────────────────

def pick_family(root: tk.Misc) -> str:
    have = set(tkfont.families(root))
    for name in ("Ubuntu", "Noto Sans", "DejaVu Sans", "Segoe UI", "Helvetica"):
        if name in have:
            return name
    return "TkDefaultFont"


def apply_theme(root: tk.Tk) -> Dict[str, tkfont.Font]:
    family = pick_family(root)
    fonts = {
        "body": tkfont.Font(root=root, family=family, size=10),
        "small": tkfont.Font(root=root, family=family, size=9),
        "bold": tkfont.Font(root=root, family=family, size=10, weight="bold"),
        "title": tkfont.Font(root=root, family=family, size=17, weight="bold"),
        "head": tkfont.Font(root=root, family=family, size=9, weight="bold"),
        "big": tkfont.Font(root=root, family=family, size=20, weight="bold"),
        "huge": tkfont.Font(root=root, family=family, size=34, weight="bold"),
        "mono": tkfont.Font(root=root, family="DejaVu Sans Mono", size=10),
    }
    root.configure(bg=BG)
    st = ttk.Style(root)
    st.theme_use("clam")
    st.configure(".", background=BG, foreground=TEXT, fieldbackground=FIELD, bordercolor=BORDER,
                 lightcolor=BORDER, darkcolor=BORDER, troughcolor=PANEL, font=fonts["body"])
    st.configure("TFrame", background=BG)
    st.configure("Panel.TFrame", background=PANEL)
    st.configure("TLabel", background=BG, foreground=TEXT)
    st.configure("Panel.TLabel", background=PANEL)
    st.configure("Muted.TLabel", foreground=MUTED, font=fonts["small"])
    st.configure("PanelMuted.TLabel", background=PANEL, foreground=MUTED, font=fonts["head"])
    st.configure("Title.TLabel", foreground=BRIGHT, font=fonts["title"])
    st.configure("Err.TLabel", foreground=BAD)
    st.configure("TButton", background=FIELD, foreground=TEXT, padding=(14, 7), borderwidth=1)
    st.map("TButton", background=[("active", "#2a313a"), ("disabled", PANEL)],
           foreground=[("disabled", "#484f58")])
    st.configure("Accent.TButton", background=ACCENT_DIM, foreground="#ffffff", padding=(18, 10),
                 font=fonts["bold"])
    st.map("Accent.TButton", background=[("active", ACCENT), ("disabled", PANEL)],
           foreground=[("disabled", "#484f58")])
    st.configure("TEntry", padding=6, insertcolor=BRIGHT)
    st.configure("TCheckbutton", background=BG, foreground=TEXT)
    st.map("TCheckbutton", background=[("active", BG)])
    st.configure("Treeview", background=PANEL, fieldbackground=PANEL, foreground=TEXT, rowheight=26,
                 borderwidth=0)
    st.configure("Treeview.Heading", background=FIELD, foreground=MUTED, font=fonts["head"], padding=5)
    st.map("Treeview", background=[("selected", "#1f2a38")])
    st.configure("Horizontal.TProgressbar", background=ACCENT, troughcolor=PANEL, borderwidth=0)
    return fonts


# ───────────────────────────── helpers ─────────────────────────────────────

def fmt(v: Optional[float], spec: str = "{:.1f}", dash: str = "–") -> str:
    return dash if v is None else spec.format(v)


def signal_colour(dbm: Optional[float]) -> str:
    if dbm is None:
        return MUTED
    if dbm < L.THRESHOLDS["poor_rssi"]:
        return BAD
    if dbm < L.THRESHOLDS["good_rssi"]:
        return WARN
    return BRIGHT


def udp_off_note(session: "L.Session") -> str:
    """One sentence on why UDP loss is not being measured ("" when it is)."""
    if session.analysis.udp_enabled:
        return ""
    why = session.udp_off_reason
    if why == "not selected":
        return "UDP packet loss is not being measured (the option was not ticked)."
    return f"UDP packet loss is NOT being measured: {why}." if why else ""


def udp_tile_sub(session: "L.Session") -> str:
    """Short reason for under the 'off' on the UDP tile."""
    why = session.udp_off_reason
    if not why:
        return " "
    return "not selected" if why == "not selected" else "helper did not start - see note"


def loss_colour(pct: Optional[float], tol: float) -> str:
    if pct is None:
        return MUTED
    if pct > L.THRESHOLDS["poor_loss_pct"]:
        return BAD
    if pct > tol + 1e-9:
        return WARN
    return BRIGHT


VERDICT_TAG = {"Good": "good", "Marginal": "marg", "Poor": "poor"}


# ───────────────────────────── the app ─────────────────────────────────────

class App(tk.Tk):
    def __init__(self, args=None):
        super().__init__()
        _KEEP_ALIVE.append(self)
        self.title("Link range test")
        self.geometry("880x700")
        self.minsize(780, 620)
        self.fonts = apply_theme(self)
        self.args = args
        self._q: "queue.Queue[tuple]" = queue.Queue()
        self._callbacks: Dict[int, Callable[[object], None]] = {}
        self._job_id = 0
        self._pump_id: Optional[str] = None
        self.frame: Optional[ttk.Frame] = None
        self.exit_code = 0
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._pump_id = self.after(80, self._pump)
        self.show_setup()

    # background work → main thread -------------------------------------------------
    def run_bg(self, work: Callable[[], object], done: Callable[[object], None]) -> None:
        """Run `work` in a thread, then call `done(result)` on the main thread.

        The worker thread holds ONLY `work` and the queue - never `done`, which
        usually belongs to a widget. Tk objects (variables especially) must be
        destroyed on the thread that owns Tk; if the worker were the last holder
        of one when it exited, Python would delete it there and Tcl aborts the
        whole process ("async handler deleted by the wrong thread")."""
        self._job_id += 1
        job = self._job_id
        self._callbacks[job] = done

        def go(q=self._q, fn=work, key=job) -> None:
            try:
                res: object = fn()
            except Exception as exc:                    # noqa: BLE001 - report, never crash the UI
                res = exc
            q.put((key, res))
        threading.Thread(target=go, daemon=True).start()

    def _pump(self) -> None:
        self._pump_id = None
        try:
            while True:
                key, res = self._q.get_nowait()
                done = self._callbacks.pop(key, None)
                if done is not None:
                    done(res)
        except queue.Empty:
            pass
        if self.winfo_exists():
            self._pump_id = self.after(80, self._pump)

    def destroy(self) -> None:
        if self._pump_id is not None:
            try:
                self.after_cancel(self._pump_id)
            except tk.TclError:
                pass
            self._pump_id = None
        run = getattr(self, "run_screen", None)
        if run is not None:
            run.cancel_timer()
        super().destroy()

    # screens --------------------------------------------------------------------
    def _swap(self, frame: ttk.Frame) -> None:
        if self.frame is not None:
            self.frame.destroy()
        self.frame = frame
        frame.pack(fill="both", expand=True, padx=26, pady=22)

    def show_setup(self) -> None:
        self.setup = SetupScreen(self, self.args)
        self._swap(self.setup)

    def show_run(self, cfg: L.SessionConfig) -> None:
        self.run_screen = WalkScreen(self, cfg) if cfg.mode == "walk" else RunScreen(self, cfg)
        self._swap(self.run_screen)

    def show_result(self, session: L.Session, summary: Dict[str, object]) -> None:
        screen = WalkResultScreen if session.cfg.mode == "walk" else ResultScreen
        self.result = screen(self, session, summary)
        self._swap(self.result)

    def _on_close(self) -> None:
        run = getattr(self, "run_screen", None)
        if self.frame is run and run is not None and run.active:
            if not messagebox.askyesno("Stop the test?", "Stop now and save what has been measured so far?",
                                       parent=self):
                return
            run.finish()
            return
        self.destroy()


# ───────────────────────────── screen 1: setup ─────────────────────────────

class SetupScreen(ttk.Frame):
    def __init__(self, app: App, args=None):
        super().__init__(app)
        self.app = app
        prefs = L.load_prefs()
        ip = (getattr(args, "radxa", None) or prefs.get("radxa") or L.gcs_saved_host() or "")

        def pref(key, default):
            return prefs.get(key, default)

        self.var_ip = tk.StringVar(value=str(ip))
        self.var_video = tk.BooleanVar(value=bool(pref("video", True)))
        self.var_udp = tk.BooleanVar(value=bool(pref("udp", True)))
        self.var_control = tk.BooleanVar(value=bool(pref("control", True)))
        self.var_user = tk.StringVar(value=str(pref("ssh_user", "radxa")))
        self.var_vport = tk.StringVar(value=str(pref("video_port", 8080)))
        self.var_step = tk.StringVar(value=str(pref("step", 5)))
        self.var_hold = tk.StringVar(value=str(pref("hold", 15)))
        self.var_tol = tk.StringVar(value=str(pref("loss_tolerance", 0)))
        self.var_mbps = tk.StringVar(value=str(pref("udp_mbps", 1.0)))
        self.var_adv = tk.BooleanVar(value=False)
        self.var_mode = tk.StringVar(value=str(pref("mode", "holds")) if pref("mode", "holds") in ("holds", "walk") else "holds")
        self.result: Optional[L.SessionConfig] = None
        self._busy = False

        ttk.Label(self, text="LINK RANGE TEST", style="Title.TLabel").pack(anchor="w")
        ttk.Label(self, text="Finds how far you can walk before the live camera feed starts losing packets.",
                  style="Muted.TLabel").pack(anchor="w", pady=(2, 18))

        ttk.Label(self, text="Radxa IP address").pack(anchor="w")
        row = ttk.Frame(self)
        row.pack(fill="x", pady=(4, 0))
        self.ent_ip = ttk.Entry(row, textvariable=self.var_ip, font=app.fonts["big"], width=18)
        self.ent_ip.pack(side="left")
        self.btn_check = ttk.Button(row, text="Check connection", command=self.check)
        self.btn_check.pack(side="left", padx=12)
        self.lbl_status = ttk.Label(self, text="Not checked yet.", style="Muted.TLabel")
        self.lbl_status.pack(anchor="w", pady=(6, 0))
        self.lbl_net = ttk.Label(self, text="", style="Muted.TLabel")
        self.lbl_net.pack(anchor="w")

        modes = ttk.Frame(self)
        modes.pack(fill="x", pady=(18, 0))
        ttk.Label(modes, text="How do you want to measure?").pack(anchor="w")
        ttk.Radiobutton(modes, text="Continuous walk - walk away at your own pace; values stream every second "
                                    "(result in seconds and dBm, no distance)",
                        variable=self.var_mode, value="walk").pack(anchor="w", pady=2)
        ttk.Radiobutton(modes, text="Hold at marked distances - stand at measured spots and press a button",
                        variable=self.var_mode, value="holds").pack(anchor="w", pady=2)

        box = ttk.Frame(self)
        box.pack(fill="x", pady=(14, 0))
        ttk.Checkbutton(box, text="Measure the live camera stream", variable=self.var_video).pack(anchor="w", pady=2)
        ttk.Checkbutton(box, text="Measure packet loss (starts a small helper on the Radxa over SSH)",
                        variable=self.var_udp).pack(anchor="w", pady=2)
        ttk.Checkbutton(box, text="Read the Radxa's own Wi-Fi signal as a control",
                        variable=self.var_control).pack(anchor="w", pady=2)

        ttk.Checkbutton(self, text="Advanced options", variable=self.var_adv, command=self._toggle_adv).pack(
            anchor="w", pady=(16, 4))
        self.adv = ttk.Frame(self)
        for r, (label, var, hint) in enumerate((
                ("SSH user on the Radxa", self.var_user, ""),
                ("Camera stream port", self.var_vport, "http://<ip>:port/video"),
                ("Metres between marks", self.var_step, ""),
                ("Seconds to hold at each mark", self.var_hold, ""),
                ("Allowed loss (%)", self.var_tol, "0 = no loss at all"),
                ("Probe rate (Mbit/s)", self.var_mbps, "keep low; it shares the link with the video"))):
            ttk.Label(self.adv, text=label).grid(row=r, column=0, sticky="w", pady=3, padx=(0, 14))
            ttk.Entry(self.adv, textvariable=var, width=10).grid(row=r, column=1, sticky="w")
            ttk.Label(self.adv, text=hint, style="Muted.TLabel").grid(row=r, column=2, sticky="w", padx=10)

        self.lbl_err = ttk.Label(self, text="", style="Err.TLabel", wraplength=780, justify="left")
        self.lbl_err.pack(anchor="w", pady=(14, 0))
        btns = ttk.Frame(self)
        btns.pack(side="bottom", fill="x", pady=(18, 0))
        self.btn_ok = ttk.Button(btns, text="Start test  ▸", style="Accent.TButton", command=self.ok)
        self.btn_ok.pack(side="right")
        ttk.Button(btns, text="Cancel", command=app.destroy).pack(side="right", padx=10)

        app.bind("<Return>", lambda _e: self.ok())
        app.bind("<Escape>", lambda _e: app.destroy())
        self.ent_ip.focus_set()
        self.ent_ip.select_range(0, "end")
        app.run_bg(self._network_info, self._show_net)

    # ── pieces ──
    def _toggle_adv(self) -> None:
        if self.var_adv.get():
            self.adv.pack(anchor="w", before=self.lbl_err)
        else:
            self.adv.pack_forget()

    def _network_info(self) -> str:
        iface = L.detect_wifi_iface()
        if not iface:
            return "This laptop: no Wi-Fi interface found."
        try:
            out = subprocess.run(["iw", "dev", iface, "link"], capture_output=True, text=True, timeout=4).stdout
        except (OSError, subprocess.SubprocessError):
            return ""
        d = L.parse_iw_link(out)
        if not d.get("connected"):
            return "This laptop is not connected to Wi-Fi."
        sig = d.get("signal_dbm")
        return f"This laptop is on Wi-Fi network: {d.get('ssid')}" + (f"  ({sig:.0f} dBm)" if sig is not None else "")

    def _show_net(self, text) -> None:
        if isinstance(text, str) and self.winfo_exists():
            self.lbl_net.configure(text=text)

    def error(self, text: str) -> None:
        self.lbl_err.configure(text=text)

    def _number(self, var: tk.StringVar, name: str, lo: float, hi: float) -> Optional[float]:
        try:
            v = float(var.get())
        except ValueError:
            self.error(f"{name} must be a number.")
            return None
        if not lo <= v <= hi:
            self.error(f"{name} must be between {lo:g} and {hi:g}.")
            return None
        return v

    def build_config(self) -> Optional[L.SessionConfig]:
        """Validate the form; returns the config or None (with the reason shown)."""
        self.error("")
        ip = self.var_ip.get().strip()
        if not L.valid_host(ip):
            self.error("That is not a valid IP address (like 172.16.100.182) or hostname.")
            self.ent_ip.focus_set()
            return None
        step = self._number(self.var_step, "Metres between marks", 0.5, 100)
        hold = self._number(self.var_hold, "Seconds to hold", 3, 120)
        tol = self._number(self.var_tol, "Allowed loss", 0, 50)
        mbps = self._number(self.var_mbps, "Probe rate", 0.1, 20)
        port = self._number(self.var_vport, "Camera stream port", 1, 65535)
        if None in (step, hold, tol, mbps, port):
            self.var_adv.set(True)
            self._toggle_adv()
            return None
        user = self.var_user.get().strip() or "radxa"
        target = f"{user}@{ip}"
        return L.SessionConfig(
            radxa=ip, ssh_user=user, video_port=int(port), use_video=self.var_video.get(),
            use_udp=self.var_udp.get(), udp_mbps=float(mbps),
            helper_target=target if self.var_udp.get() else "",
            control_target=target if self.var_control.get() else "",
            step=float(step), hold=float(hold), baseline_hold=float(hold), loss_tolerance=float(tol),
            mode=self.var_mode.get())

    def _save_prefs(self, cfg: L.SessionConfig) -> None:
        L.save_prefs({"radxa": cfg.radxa, "ssh_user": cfg.ssh_user, "video_port": cfg.video_port,
                      "video": cfg.use_video, "udp": cfg.use_udp, "control": bool(cfg.control_target),
                      "step": cfg.step, "hold": cfg.hold, "loss_tolerance": cfg.loss_tolerance,
                      "udp_mbps": cfg.udp_mbps, "mode": cfg.mode})

    # ── actions ──
    def check(self) -> None:
        ip = self.var_ip.get().strip()
        if not L.valid_host(ip):
            self.error("That is not a valid IP address or hostname.")
            return
        self.error("")
        self._set_busy(True, "Checking…")
        sess = L.Session(L.SessionConfig(radxa=ip))
        self.app.run_bg(sess.preflight, self._checked)

    def _checked(self, res) -> None:
        self._set_busy(False)
        if isinstance(res, Exception):
            self.lbl_status.configure(text=f"Check failed: {res}")
            return
        if res["reachable"]:
            self.lbl_status.configure(text="✓ The Radxa answers.", foreground=BRIGHT)
        else:
            ssid = res.get("ssid") or "unknown"
            self.lbl_status.configure(
                text=f"✗ No answer. This laptop is on '{ssid}' - is that the same network as the Radxa?",
                foreground=WARN)

    def _set_busy(self, busy: bool, text: str = "") -> None:
        self._busy = busy
        state = "disabled" if busy else "normal"
        self.btn_check.configure(state=state)
        self.btn_ok.configure(state=state)
        if busy:
            self.lbl_status.configure(text=text, foreground=MUTED)

    def ok(self) -> None:
        if self._busy:
            return
        cfg = self.build_config()
        if cfg is None:
            return
        self._save_prefs(cfg)
        self.result = cfg
        self._set_busy(True, "Checking the Radxa is reachable…")
        sess = L.Session(cfg)
        self.app.run_bg(sess.preflight, lambda res: self._after_preflight(cfg, res))

    def _after_preflight(self, cfg: L.SessionConfig, res) -> None:
        self._set_busy(False)
        if isinstance(res, Exception) or not res["reachable"]:
            ssid = (res.get("ssid") if not isinstance(res, Exception) else None) or "unknown"
            go = messagebox.askyesno(
                "The Radxa does not answer",
                f"{cfg.radxa} does not answer ping from this laptop.\n\n"
                f"This laptop is on Wi-Fi network: {ssid}\n"
                "Is that the same network as the Radxa?\n\nStart the test anyway?",
                default="no", parent=self.app)
            if not go:
                self.lbl_status.configure(text="Fix the network, then press Start again.", foreground=WARN)
                return
        self.app.unbind("<Return>")
        self.app.unbind("<Escape>")
        self.app.show_run(cfg)


# ───────────────────────────── screen 2: walking ───────────────────────────

class Tile(ttk.Frame):
    def __init__(self, parent, caption: str, fonts):
        super().__init__(parent, style="Panel.TFrame", padding=(14, 10))
        ttk.Label(self, text=caption, style="PanelMuted.TLabel").pack(anchor="w")
        self.value = tk.Label(self, text="–", bg=PANEL, fg=BRIGHT, font=fonts["big"], anchor="w")
        self.value.pack(anchor="w")
        self.sub = tk.Label(self, text=" ", bg=PANEL, fg=MUTED, font=fonts["small"], anchor="w")
        self.sub.pack(anchor="w")

    def set(self, text: str, colour: str = BRIGHT, sub: str = " ") -> None:
        self.value.configure(text=text, fg=colour)
        self.sub.configure(text=sub or " ")


class RunScreen(ttk.Frame):
    """States: starting → baseline_wait → holding → between → (finishing)."""

    def __init__(self, app: App, cfg: L.SessionConfig):
        super().__init__(app)
        self.app, self.cfg = app, cfg
        self.session = L.Session(cfg, log=lambda m: None)
        self.state = "starting"
        self.active = True
        self.last_distance = 0.0
        self.hold_t0 = 0.0
        self.hold_end = 0.0
        self.hold_distance = 0.0
        self.var_dist = tk.StringVar(value=f"{cfg.step:g}")
        self._fin = False

        head = ttk.Frame(self)
        head.pack(fill="x")
        ttk.Label(head, text="LINK RANGE TEST", style="Title.TLabel").pack(side="left")
        self.lbl_target = ttk.Label(head, text=f"Radxa {cfg.radxa}", style="Muted.TLabel")
        self.lbl_target.pack(side="right", anchor="s")

        self.lbl_instr = tk.Label(self, text="Starting the measurement…", bg=BG, fg=BRIGHT,
                                  font=app.fonts["big"], anchor="w", justify="left", wraplength=800)
        self.lbl_instr.pack(fill="x", pady=(14, 2))
        self.lbl_sub = ttk.Label(self, text="", style="Muted.TLabel", wraplength=800, justify="left")
        self.lbl_sub.pack(fill="x")
        self.lbl_udp_note = ttk.Label(self, text="", wraplength=800, justify="left", foreground=WARN)
        self.lbl_udp_note.pack(fill="x")

        tiles = ttk.Frame(self)
        tiles.pack(fill="x", pady=14)
        self.t_sig = Tile(tiles, "WI-FI SIGNAL", app.fonts)
        self.t_ping = Tile(tiles, "PING LOSS", app.fonts)
        self.t_video = Tile(tiles, "CAMERA FEED", app.fonts)
        self.t_udp = Tile(tiles, "LOSS UP / DOWN", app.fonts)
        self.t_state = Tile(tiles, "RIGHT NOW", app.fonts)
        for i, t in enumerate((self.t_sig, self.t_ping, self.t_video, self.t_udp, self.t_state)):
            t.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 6, 0))
            tiles.columnconfigure(i, weight=1, uniform="t")

        self.bar = ttk.Progressbar(self, mode="determinate", maximum=1000)
        self.bar.pack(fill="x")
        self.lbl_count = ttk.Label(self, text=" ", style="Muted.TLabel")
        self.lbl_count.pack(anchor="w", pady=(4, 8))

        controls = ttk.Frame(self)
        controls.pack(fill="x")
        self.btn_main = ttk.Button(controls, text="Starting…", style="Accent.TButton", command=self.on_main,
                                   state="disabled")
        self.btn_main.pack(side="left")
        self.dist_box = ttk.Frame(controls)
        ttk.Label(self.dist_box, text="Distance from the Radxa (m):").pack(side="left", padx=(18, 6))
        self.ent_dist = ttk.Entry(self.dist_box, textvariable=self.var_dist, width=7, font=app.fonts["bold"])
        self.ent_dist.pack(side="left")
        self.btn_finish = ttk.Button(controls, text="Finish and show result", command=self.finish, state="disabled")
        self.btn_finish.pack(side="right")

        self.lbl_head = ttk.Label(self, text="Loss-free so far: –", font=app.fonts["bold"], foreground=BRIGHT)
        self.lbl_head.pack(anchor="w", pady=(16, 6))
        cols = ("dist", "signal", "ping", "video", "udp", "lf", "verdict")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", height=7, selectmode="none")
        for c, label, w in (("dist", "Distance", 80), ("signal", "Signal", 90), ("ping", "Ping loss", 90),
                            ("video", "Video", 110), ("udp", "UDP up / down", 130), ("lf", "Loss-free", 90),
                            ("verdict", "Verdict", 100)):
            self.tree.heading(c, text=label)
            self.tree.column(c, width=w, anchor="center")
        self.tree.tag_configure("marg", foreground=WARN)
        self.tree.tag_configure("poor", foreground=BAD)
        self.tree.pack(fill="both", expand=True)

        app.bind("<Return>", lambda _e: self.on_main() if str(self.btn_main["state"]) == "normal" else None)
        app.run_bg(self.session.start, self._started)
        self._tick_id: Optional[str] = self.after(TICK_MS, self._tick)

    # ── flow ──
    def _started(self, res) -> None:
        if isinstance(res, Exception):
            messagebox.showerror("Could not start", str(res), parent=self.app)
            self.active = False
            self.app.destroy()
            return
        self.lbl_udp_note.configure(text=udp_off_note(self.session))
        self.state = "baseline_wait"
        self.lbl_instr.configure(text="Stand next to the Radxa and router (0 m).")
        self.lbl_sub.configure(text="Press the button, then stand still until the countdown ends. "
                                    "Nothing is measured against distance until you press it.")
        self.btn_main.configure(text="Measure here (0 m)", state="normal")
        self.btn_finish.configure(state="normal")

    def on_main(self) -> None:
        if self.state == "baseline_wait":
            self._begin_hold(0.0, self.cfg.baseline_hold)
        elif self.state == "between":
            try:
                d = float(self.var_dist.get())
            except ValueError:
                self.lbl_sub.configure(text="Type the distance as a number of metres.")
                return
            if d <= 0:
                self.lbl_sub.configure(text="The distance must be more than 0 m.")
                return
            self._begin_hold(d, self.cfg.hold)

    def _begin_hold(self, distance: float, seconds: float) -> None:
        self.state = "holding"
        self.hold_distance = distance
        self.hold_t0 = self.session.begin_hold()
        self.hold_end = self.hold_t0 + seconds
        self.btn_main.configure(state="disabled", text="Measuring…")
        self.btn_finish.configure(state="disabled")
        self.dist_box.pack_forget()
        self.lbl_instr.configure(text=f"Hold still at {distance:g} m")
        self.lbl_sub.configure(text="Keep the laptop where it is until the bar is full.")

    def _end_hold(self) -> None:
        h = self.session.end_hold(self.hold_distance, self.hold_t0)
        ranges = self.session.running_ranges()
        self.last_distance = max(self.last_distance, self.hold_distance)
        self._refresh_table()
        self.state = "between"
        nxt = self.last_distance + self.cfg.step
        self.var_dist.set(f"{nxt:g}")
        lf = ranges.get("loss_free_to")
        self.lbl_head.configure(text="Loss-free so far: " + ("nothing yet" if lf is None else f"up to {lf:g} m"))
        verdict = "no loss at all" if h.loss_free else ("loss detected" if h.loss_free is False else "no loss data")
        self.lbl_instr.configure(
            text=(f"{self.hold_distance:g} m done - {verdict}. Walk to the next mark."))
        self.lbl_sub.configure(text=f"Walk about {self.cfg.step:g} m further, stand still, enter the distance "
                                    "if it differs, then press the button.")
        self.bar["value"] = 0
        self.lbl_count.configure(text=" ")
        self.btn_main.configure(text="I'm at the mark - measure", state="normal")
        self.dist_box.pack(side="left")
        self.btn_finish.configure(state="normal")
        self.ent_dist.focus_set()
        self.ent_dist.select_range(0, "end")

    def _refresh_table(self) -> None:
        self.tree.delete(*self.tree.get_children())
        for h in sorted(self.session.holds, key=lambda x: x.distance_m):
            self.tree.insert("", "end", tags=(VERDICT_TAG.get(h.verdict, ""),), values=(
                f"{h.distance_m:g} m", fmt(h.signal_mean, "{:.0f} dBm"), fmt(h.ping_loss, "{:.1f} %"),
                fmt(h.video_fps, "{:.1f} fps"),
                f"{fmt(h.udp_up_loss, '{:.1f}')} / {fmt(h.udp_down_loss, '{:.1f}')} %"
                if h.udp_up_loss is not None else "–",
                {True: "Yes", False: "No"}.get(h.loss_free, "–"), h.verdict))

    # ── live display ──
    def cancel_timer(self) -> None:
        if getattr(self, "_tick_id", None) is not None:
            try:
                self.after_cancel(self._tick_id)
            except tk.TclError:
                pass
            self._tick_id = None

    def _tick(self) -> None:
        self._tick_id = None
        if not self.winfo_exists() or self._fin:
            return
        try:
            if self.session.started:
                self._update_live()
                if self.state == "holding":
                    now = time.monotonic()
                    total = max(0.1, self.hold_end - self.hold_t0)
                    self.bar["value"] = min(1000, (now - self.hold_t0) / total * 1000)
                    self.lbl_count.configure(text=f"{max(0, self.hold_end - now):.0f} s left")
                    if now >= self.hold_end:
                        self._end_hold()
        finally:
            if self.winfo_exists() and not self._fin:
                self._tick_id = self.after(TICK_MS, self._tick)

    def _update_live(self) -> None:
        s, tol = self.session.live(LIVE_WINDOW_S), self.cfg.loss_tolerance
        self.t_sig.set(fmt(s.signal_mean, "{:.0f} dBm"), signal_colour(s.signal_mean),
                       f"{fmt(s.tx_mbps, '{:.0f}')} Mbit/s link" if s.tx_mbps is not None else " ")
        self.t_ping.set(fmt(s.ping_loss, "{:.1f} %"), loss_colour(s.ping_loss, tol),
                        f"round trip {fmt(s.ping_avg, '{:.0f}')} ms" if s.ping_avg is not None else " ")
        if self.cfg.use_video:
            stall = f"longest stall {fmt(s.video_stall_max, '{:.1f}')} s" if s.video_stall_max is not None else " "
            self.t_video.set(fmt(s.video_fps, "{:.1f} fps"), BRIGHT if (s.video_fps or 0) > 0 else BAD, stall)
        else:
            self.t_video.set("off", MUTED)
        if self.session.analysis.udp_enabled:
            worst = max([v for v in (s.udp_up_loss, s.udp_down_loss) if v is not None], default=None)
            self.t_udp.set(f"{fmt(s.udp_up_loss, '{:.1f}')} / {fmt(s.udp_down_loss, '{:.1f}')} %",
                           loss_colour(worst, tol))
        else:
            self.t_udp.set("off", MUTED, udp_tile_sub(self.session))
        lf = L.is_loss_free(s, tol)
        if lf is None:
            self.t_state.set("measuring…", MUTED)
        elif lf:
            self.t_state.set("no loss", BRIGHT)
        else:
            self.t_state.set("LOSS", BAD, ", ".join(L.loss_reasons(s, tol))[:40])

    # ── finish ──
    def finish(self) -> None:
        if self._fin:
            return
        if not self.session.holds:
            if not messagebox.askyesno("Nothing measured yet", "No distance has been measured. Quit without a result?",
                                       parent=self.app):
                return
            self._fin = True
            self.active = False
            self.session.stop()
            self.app.destroy()
            return
        self._fin = True
        self.active = False
        self.btn_main.configure(state="disabled")
        self.btn_finish.configure(state="disabled", text="Writing the report…")
        self.lbl_instr.configure(text="Finishing - writing the report…")

        def work():
            self.session.stop()
            return self.session.finish()

        self.app.run_bg(work, self._finished)

    def _finished(self, summary) -> None:
        if isinstance(summary, Exception) or summary is None:
            messagebox.showerror("Could not write the report", str(summary), parent=self.app)
            self.app.destroy()
            return
        self.app.show_result(self.session, summary)


# ───────────────────────────── continuous walk ─────────────────────────────

class WalkScreen(ttk.Frame):
    """Continuous-walk mode: one button to start, one line of values per second,
    a live "loss-free so far" line, one button to stop. Same attributes as
    RunScreen (active, finish, cancel_timer) so the App treats both alike."""

    MAX_ROWS = 400

    def __init__(self, app: App, cfg: L.SessionConfig):
        super().__init__(app)
        self.app, self.cfg = app, cfg
        self.session = L.Session(cfg, log=lambda m: None)
        self.rec = W.WalkRecorder(self.session)
        self.state = "starting"
        self.active = True
        self._fin = False

        head = ttk.Frame(self)
        head.pack(fill="x")
        ttk.Label(head, text="LINK RANGE TEST - CONTINUOUS WALK", style="Title.TLabel").pack(side="left")
        ttk.Label(head, text=f"Radxa {cfg.radxa}", style="Muted.TLabel").pack(side="right", anchor="s")

        self.lbl_instr = tk.Label(self, text="Starting the measurement…", bg=BG, fg=BRIGHT,
                                  font=app.fonts["big"], anchor="w", justify="left", wraplength=800)
        self.lbl_instr.pack(fill="x", pady=(14, 2))
        self.lbl_sub = ttk.Label(self, text="", style="Muted.TLabel", wraplength=800, justify="left")
        self.lbl_sub.pack(fill="x")
        self.lbl_udp_note = ttk.Label(self, text="", wraplength=800, justify="left", foreground=WARN)
        self.lbl_udp_note.pack(fill="x")

        tiles = ttk.Frame(self)
        tiles.pack(fill="x", pady=14)
        self.t_time = Tile(tiles, "TIME", app.fonts)
        self.t_sig = Tile(tiles, "WI-FI SIGNAL", app.fonts)
        self.t_ping = Tile(tiles, "PING LOSS", app.fonts)
        self.t_video = Tile(tiles, "CAMERA FEED", app.fonts)
        self.t_udp = Tile(tiles, "LOSS UP / DOWN", app.fonts)
        self.t_state = Tile(tiles, "RIGHT NOW", app.fonts)
        for i, t in enumerate((self.t_time, self.t_sig, self.t_ping, self.t_video, self.t_udp, self.t_state)):
            t.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 6, 0))
            tiles.columnconfigure(i, weight=1, uniform="w")

        controls = ttk.Frame(self)
        controls.pack(fill="x")
        self.btn_main = ttk.Button(controls, text="Starting…", style="Accent.TButton", command=self.on_main,
                                   state="disabled")
        self.btn_main.pack(side="left")

        self.lbl_head = ttk.Label(self, text="Loss-free so far: –", font=app.fonts["bold"], foreground=BRIGHT)
        self.lbl_head.pack(anchor="w", pady=(16, 6))
        cols = ("t", "signal", "ping", "video", "stall", "udp", "state")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", height=9, selectmode="none")
        for c, label, w in (("t", "Time", 70), ("signal", "Signal", 90), ("ping", "Ping loss", 90),
                            ("video", "Video", 100), ("stall", "Longest gap", 100),
                            ("udp", "UDP up / down", 120), ("state", "State", 200)):
            self.tree.heading(c, text=label)
            self.tree.column(c, width=w, anchor="center" if c != "state" else "w")
        self.tree.tag_configure("loss", foreground=BAD)
        self.tree.tag_configure("unknown", foreground=MUTED)
        self.tree.pack(fill="both", expand=True)

        app.bind("<Return>", lambda _e: self.on_main() if str(self.btn_main["state"]) == "normal" else None)
        app.run_bg(self.session.start, self._started)
        self._tick_id: Optional[str] = self.after(TICK_MS, self._tick)

    # ── flow ──
    def _started(self, res) -> None:
        if isinstance(res, Exception):
            messagebox.showerror("Could not start", str(res), parent=self.app)
            self.active = False
            self.app.destroy()
            return
        self.lbl_udp_note.configure(text=udp_off_note(self.session))
        self.state = "ready"
        self.lbl_instr.configure(text="Stand next to the Radxa and router.")
        self.lbl_sub.configure(text="Press the button, then walk away at your own pace. Values stream below "
                                    "once a second. Press the button again when you want to stop.")
        self.btn_main.configure(text="Start walking  ▸", state="normal")

    def on_main(self) -> None:
        if self.state == "ready":
            self.rec.begin()
            self.state = "walking"
            self.lbl_instr.configure(text="Walking - measuring every second")
            self.lbl_sub.configure(text="Walk away steadily. Stop here when the picture has been bad for a while "
                                        "or you run out of room.")
            self.btn_main.configure(text="Stop and show result")
        elif self.state == "walking":
            self.finish()

    # ── live display ──
    def cancel_timer(self) -> None:
        if getattr(self, "_tick_id", None) is not None:
            try:
                self.after_cancel(self._tick_id)
            except tk.TclError:
                pass
            self._tick_id = None

    def _tick(self) -> None:
        self._tick_id = None
        if not self.winfo_exists() or self._fin:
            return
        try:
            if self.state == "walking":
                new = self.rec.update()
                for p in new:
                    self._add_row(p)
                if new:
                    self._show_point(new[-1])
                    self._show_verdict()
        finally:
            if self.winfo_exists() and not self._fin:
                self._tick_id = self.after(TICK_MS, self._tick)

    def _add_row(self, p: "W.WalkPoint") -> None:
        tag = {True: "", False: "loss", None: "unknown"}[p.loss_free]
        state = {True: "ok", False: "LOSS: " + ", ".join(p.reasons), None: "no data"}[p.loss_free]
        udp = (f"{fmt(p.udp_up_loss, '{:.1f}')} / {fmt(p.udp_down_loss, '{:.1f}')} %"
               if p.udp_up_loss is not None or p.udp_down_loss is not None else "–")
        item = self.tree.insert("", "end", tags=(tag,), values=(
            f"{p.t_s:.0f} s", fmt(p.signal_dbm, "{:.0f} dBm"), fmt(p.ping_loss, "{:.1f} %"),
            fmt(p.video_fps, "{:.1f} fps"), fmt(p.stall_s, "{:.1f} s"), udp, state))
        children = self.tree.get_children()
        if len(children) > self.MAX_ROWS:
            self.tree.delete(children[0])
        self.tree.see(item)

    def _show_point(self, p: "W.WalkPoint") -> None:
        tol = self.cfg.loss_tolerance
        self.t_time.set(f"{p.t_s:.0f} s")
        self.t_sig.set(fmt(p.signal_dbm, "{:.0f} dBm"), signal_colour(p.signal_dbm),
                       f"{fmt(p.tx_mbps, '{:.0f}')} Mbit/s link" if p.tx_mbps is not None else " ")
        self.t_ping.set(fmt(p.ping_loss, "{:.1f} %"), loss_colour(p.ping_loss, tol),
                        f"round trip p95 {fmt(p.ping_p95, '{:.0f}')} ms" if p.ping_p95 is not None else " ")
        if self.cfg.use_video:
            stall = f"longest gap {fmt(p.stall_s, '{:.1f}')} s" if p.stall_s is not None else " "
            self.t_video.set(fmt(p.video_fps, "{:.1f} fps"), BRIGHT if (p.video_fps or 0) > 0 else BAD, stall)
        else:
            self.t_video.set("off", MUTED)
        if self.session.analysis.udp_enabled:
            worst = max([v for v in (p.udp_up_loss, p.udp_down_loss) if v is not None], default=None)
            self.t_udp.set(f"{fmt(p.udp_up_loss, '{:.1f}')} / {fmt(p.udp_down_loss, '{:.1f}')} %",
                           loss_colour(worst, tol))
        else:
            self.t_udp.set("off", MUTED, udp_tile_sub(self.session))
        if p.loss_free is None:
            self.t_state.set("measuring…", MUTED)
        elif p.loss_free:
            self.t_state.set("no loss", BRIGHT)
        else:
            self.t_state.set("LOSS", BAD, ", ".join(p.reasons)[:40])

    def _show_verdict(self) -> None:
        v = self.rec.verdict()
        first = v.get("first_failure")
        if v.get("loss_free_pct") is None:
            self.lbl_head.configure(text="Loss-free so far: no loss data yet", foreground=MUTED)
        elif first is None:
            dbm = v.get("clean_down_to_dbm")
            self.lbl_head.configure(text=f"Loss-free so far: {v['clean_for_s']} s"
                                         + (f", down to {dbm:.0f} dBm" if dbm is not None else ""),
                                    foreground=BRIGHT)
        else:
            dbm = first.get("signal_dbm")
            self.lbl_head.configure(
                text=f"Loss-free for {v['clean_for_s']} s. First loss at {first['t_s']:.0f} s"
                     + (f" ({dbm:.0f} dBm)" if dbm is not None else ""), foreground=WARN)

    # ── finish ──
    def finish(self) -> None:
        if self._fin:
            return
        if self.state != "walking" or not self.rec.points:
            if not messagebox.askyesno("Nothing measured yet", "No walk has been measured. Quit without a result?",
                                       parent=self.app):
                return
            self._fin = True
            self.active = False
            self.session.stop()
            self.app.destroy()
            return
        self._fin = True
        self.active = False
        self.btn_main.configure(state="disabled", text="Writing the report…")
        self.lbl_instr.configure(text="Finishing - writing the report…")

        def work():
            self.rec.end()
            self.session.stop()
            return self.rec.finish()

        self.app.run_bg(work, self._finished)

    def _finished(self, summary) -> None:
        if isinstance(summary, Exception) or summary is None:
            messagebox.showerror("Could not write the report", str(summary), parent=self.app)
            self.app.destroy()
            return
        self.app.show_result(self.session, summary)


# ───────────────────────────── screen 3: result ────────────────────────────

class ResultScreen(ttk.Frame):
    def __init__(self, app: App, session: L.Session, summary: Dict[str, object]):
        super().__init__(app)
        self.app, self.session, self.summary = app, session, summary
        self.report = os.path.join(session.cfg.out_dir, "report.html")
        ranges = summary["ranges"]
        lf = ranges.get("loss_free_to")  # type: ignore[union-attr]
        notes: List[str] = summary["findings"]  # type: ignore[assignment]

        ttk.Label(self, text="RESULT", style="Title.TLabel").pack(anchor="w")
        card = ttk.Frame(self, style="Panel.TFrame", padding=(22, 16))
        card.pack(fill="x", pady=(12, 8))
        ttk.Label(card, text="LIVE FEED WITH NO PACKET LOSS UP TO", style="PanelMuted.TLabel").pack(anchor="w")
        self.lbl_big = tk.Label(card, text="–" if lf is None else f"{lf:g} m", bg=PANEL, fg=BRIGHT,
                                font=app.fonts["huge"], anchor="w")
        self.lbl_big.pack(anchor="w")
        tk.Label(card, text=notes[0] if notes else "", bg=PANEL, fg=MUTED, font=app.fonts["body"],
                 anchor="w", justify="left", wraplength=780).pack(anchor="w")

        note = udp_off_note(session)
        if note:
            ttk.Label(self, text=note, wraplength=780, justify="left", foreground=WARN).pack(anchor="w")
        cols = ("dist", "signal", "ping", "video", "udp", "lf", "verdict")
        tree = ttk.Treeview(self, columns=cols, show="headings", height=min(10, max(3, len(session.holds))),
                            selectmode="none")
        for c, label, w in (("dist", "Distance", 80), ("signal", "Signal", 90), ("ping", "Ping loss", 90),
                            ("video", "Video", 110), ("udp", "UDP up / down", 130), ("lf", "Loss-free", 90),
                            ("verdict", "Verdict", 100)):
            tree.heading(c, text=label)
            tree.column(c, width=w, anchor="center")
        tree.tag_configure("marg", foreground=WARN)
        tree.tag_configure("poor", foreground=BAD)
        for h in sorted(session.holds, key=lambda x: x.distance_m):
            tree.insert("", "end", tags=(VERDICT_TAG.get(h.verdict, ""),), values=(
                f"{h.distance_m:g} m", fmt(h.signal_mean, "{:.0f} dBm"), fmt(h.ping_loss, "{:.1f} %"),
                fmt(h.video_fps, "{:.1f} fps"),
                f"{fmt(h.udp_up_loss, '{:.1f}')} / {fmt(h.udp_down_loss, '{:.1f}')} %"
                if h.udp_up_loss is not None else "–",
                {True: "Yes", False: "No"}.get(h.loss_free, "–"), h.verdict))
        tree.pack(fill="x", pady=(8, 8))

        box = tk.Text(self, height=6, bg=PANEL, fg=TEXT, bd=0, wrap="word", font=app.fonts["small"],
                      highlightthickness=0, padx=10, pady=8)
        box.insert("1.0", "\n".join("• " + n for n in notes[1:]))
        box.configure(state="disabled")
        box.pack(fill="both", expand=True)

        ttk.Label(self, text=f"Saved in {session.cfg.out_dir}", style="Muted.TLabel").pack(anchor="w", pady=(8, 0))
        btns = ttk.Frame(self)
        btns.pack(fill="x", pady=(10, 0))
        ttk.Button(btns, text="Close", command=app.destroy).pack(side="right")
        ttk.Button(btns, text="Open folder", command=self.open_folder).pack(side="right", padx=8)
        ttk.Button(btns, text="Open full report", style="Accent.TButton", command=self.open_report).pack(side="right")
        ttk.Button(btns, text="Run again", command=self.again).pack(side="left")
        app.unbind("<Return>")

    def open_report(self) -> None:
        webbrowser.open("file://" + self.report)

    def open_folder(self) -> None:
        folder = self.session.cfg.out_dir
        try:
            if sys.platform.startswith("win"):
                os.startfile(folder)                      # type: ignore[attr-defined]  # noqa: S606
            elif sys.platform == "darwin":
                subprocess.Popen(["open", folder])
            else:
                subprocess.Popen(["xdg-open", folder])
        except OSError:
            pass

    def again(self) -> None:
        self.app.show_setup()


class WalkResultScreen(ResultScreen):
    """Result of a continuous walk: seconds and dBm, a by-signal-level table, findings."""

    def __init__(self, app: App, session: L.Session, summary: Dict[str, object]):
        ttk.Frame.__init__(self, app)
        self.app, self.session, self.summary = app, session, summary
        self.report = os.path.join(session.cfg.out_dir, "report.html")
        v = summary["verdict"]                                 # type: ignore[index]
        notes: List[str] = summary["findings"]                 # type: ignore[assignment]
        judged = v.get("loss_free_pct") is not None            # type: ignore[union-attr]

        ttk.Label(self, text="RESULT - CONTINUOUS WALK", style="Title.TLabel").pack(anchor="w")
        card = ttk.Frame(self, style="Panel.TFrame", padding=(22, 16))
        card.pack(fill="x", pady=(12, 8))
        ttk.Label(card, text="LIVE FEED LOSS-FREE FOR", style="PanelMuted.TLabel").pack(anchor="w")
        self.lbl_big = tk.Label(card, text=f"{v['clean_for_s']} s" if judged else "–", bg=PANEL, fg=BRIGHT,
                                font=app.fonts["huge"], anchor="w")
        self.lbl_big.pack(anchor="w")
        dbm = v.get("clean_down_to_dbm")                       # type: ignore[union-attr]
        first = v.get("first_failure")                         # type: ignore[union-attr]
        sub = (f"down to {dbm:.0f} dBm" if dbm is not None else "")
        if first:
            sub += (f"   •   first loss at {first['t_s']:.0f} s"
                    + (f" ({first['signal_dbm']:.0f} dBm)" if first.get("signal_dbm") is not None else ""))
        tk.Label(card, text=sub, bg=PANEL, fg=TEXT, font=app.fonts["body"], anchor="w").pack(anchor="w")
        tk.Label(card, text=notes[0] if notes else "", bg=PANEL, fg=MUTED, font=app.fonts["body"],
                 anchor="w", justify="left", wraplength=780).pack(anchor="w", pady=(4, 0))

        note = udp_off_note(session)
        if note:
            ttk.Label(self, text=note, wraplength=780, justify="left", foreground=WARN).pack(anchor="w")
        bands = v.get("bands", [])                             # type: ignore[union-attr]
        tree = ttk.Treeview(self, columns=("band", "sec", "clean", "pct"), show="headings",
                            height=min(8, max(2, len(bands))), selectmode="none")
        for c, label, w in (("band", "Signal level", 160), ("sec", "Seconds", 90),
                            ("clean", "Loss-free seconds", 140), ("pct", "Loss-free", 100)):
            tree.heading(c, text=label)
            tree.column(c, width=w, anchor="center")
        tree.tag_configure("marg", foreground=WARN)
        tree.tag_configure("poor", foreground=BAD)
        for b in bands:
            pct = b["clean_pct"]
            tree.insert("", "end", tags=("" if pct >= 100 else "marg" if pct >= 80 else "poor",), values=(
                f"{b['lo']} to {b['hi']} dBm", b["seconds"], b["clean_seconds"], f"{pct:.0f} %"))
        tree.pack(fill="x", pady=(8, 8))

        box = tk.Text(self, height=7, bg=PANEL, fg=TEXT, bd=0, wrap="word", font=app.fonts["small"],
                      highlightthickness=0, padx=10, pady=8)
        box.insert("1.0", "\n".join("• " + n for n in notes[1:]))
        box.configure(state="disabled")
        box.pack(fill="both", expand=True)

        ttk.Label(self, text=f"Saved in {session.cfg.out_dir}", style="Muted.TLabel").pack(anchor="w", pady=(8, 0))
        btns = ttk.Frame(self)
        btns.pack(fill="x", pady=(10, 0))
        ttk.Button(btns, text="Close", command=app.destroy).pack(side="right")
        ttk.Button(btns, text="Open folder", command=self.open_folder).pack(side="right", padx=8)
        ttk.Button(btns, text="Open full report", style="Accent.TButton", command=self.open_report).pack(side="right")
        ttk.Button(btns, text="Run again", command=self.again).pack(side="left")
        app.unbind("<Return>")


def launch(args=None) -> int:
    app = App(args)
    app.mainloop()
    return app.exit_code
