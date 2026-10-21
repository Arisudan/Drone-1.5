#!/usr/bin/env python3
"""
================================================================================
SCRIPT: link_range_walk.py
PURPOSE: Continuous-walk mode for the link range test (signal and time, no distance)
================================================================================

WHY THIS EXISTS
  The marked-distance mode needs you to stop at measured spots and press a key.
  Indoors that is slow, and a link that is fine standing still can still stall
  while you move. This mode just measures while you walk away at your own pace
  and streams one line of values per second. There is no distance in it at all:
  the answer is given in SECONDS and dBm, which are the two things the laptop
  actually knows.

  "Loss-free for the first 42 s, down to -68 dBm. First loss at 43 s (-71 dBm):
   video stall 1.8 s."

HOW A SECOND IS JUDGED
  Every second, a sliding 2-second window is analysed with the same code and the
  same strict rules as a hold (core of link_range_test.py): any ping or UDP loss
  beyond the allowed tolerance, any video reconnect, or a gap of more than 0.5 s
  between video frames makes that window LOSS. A window with no loss figure at
  all is UNKNOWN - never "clean". The first LOSS window ends the loss-free run.

  A 2 s window (not 1 s) because a single ping reply still in flight at a window
  edge must not look like loss; sequence-number accounting handles the rest.

WHAT IS NOT DONE
  No distance estimate. The signal-band table says how often each signal level
  was clean, which is the distance-free way to see where the link starts to fail.

STANDARD LIBRARY ONLY (plus link_range_test.py next to it).
================================================================================
"""

from __future__ import annotations

import csv
import html
import json
import math
import os
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

import link_range_test as L

WINDOW_S = 2.0          # analysed span ending at each second
STEP_S = 1.0            # one point per second
BAND_DB = 5             # signal-band width in the summary table
MAX_LISTED_LOSS_ROWS = 60  # rows of "seconds with loss" in the report; the rest are in walk.json
SIGNAL_MAX_AGE_S = 3.0  # a Wi-Fi reading this old may stand in for a missing one


@dataclass
class WalkPoint:
    t_s: float
    signal_dbm: Optional[float] = None
    signal_min_dbm: Optional[float] = None
    tx_mbps: Optional[float] = None
    ping_loss: Optional[float] = None
    ping_p95: Optional[float] = None
    video_fps: Optional[float] = None
    video_mbps: Optional[float] = None
    stall_s: Optional[float] = None
    reconnects: int = 0
    udp_up_loss: Optional[float] = None
    udp_down_loss: Optional[float] = None
    loss_free: Optional[bool] = None
    reasons: List[str] = field(default_factory=list)


# ───────────────────────────── points (pure) ───────────────────────────────

def point_from_stats(t_s: float, s: "L.HoldStats", tol: float = 0.0) -> WalkPoint:
    return WalkPoint(
        t_s=t_s, signal_dbm=s.signal_mean, signal_min_dbm=s.signal_min, tx_mbps=s.tx_mbps,
        ping_loss=s.ping_loss, ping_p95=s.ping_p95, video_fps=s.video_fps, video_mbps=s.video_mbps,
        stall_s=s.video_stall_max, reconnects=s.video_disconnects,
        udp_up_loss=s.udp_up_loss, udp_down_loss=s.udp_down_loss,
        loss_free=L.is_loss_free(s, tol), reasons=L.loss_reasons(s, tol))


def compute_points(store: "L.Store", t_start: float, t_to: float, cfg: "L.Config", tol: float = 0.0,
                   start_k: int = 1, window: float = WINDOW_S, step: float = STEP_S) -> List[WalkPoint]:
    """Points k = start_k, start_k+1, ... whose window ends at or before t_to.
    Point k covers (end - window, end] with end = t_start + k*step, clipped at the start."""
    out: List[WalkPoint] = []
    k = start_k
    while t_start + k * step <= t_to + 1e-9:
        t1 = t_start + k * step
        s = L.analyse_window(store, max(t_start, t1 - window), t1, cfg)
        if s.signal_mean is None:                       # Wi-Fi sampled less often than once a window
            last = store.last_at_or_before("wifi", t1)
            if last and t1 - last[0] <= SIGNAL_MAX_AGE_S and last[1].get("signal_dbm") is not None:
                s.signal_mean = s.signal_min = last[1]["signal_dbm"]
        out.append(point_from_stats(k * step, s, tol))
        k += 1
    return out


# ───────────────────────────── verdict (pure) ──────────────────────────────

def _worst(values: Sequence[Optional[float]]) -> Optional[float]:
    vals = [v for v in values if v is not None]
    return max(vals) if vals else None


def signal_bands(points: Sequence[WalkPoint], width: int = BAND_DB) -> List[Dict[str, object]]:
    """Seconds spent in each signal band, and how many of them were loss-free,
    strongest band first. Seconds with no signal or no loss figure are left out."""
    bands: Dict[int, List[int]] = {}
    for p in points:
        if p.signal_dbm is None or p.loss_free is None:
            continue
        lo = int(math.floor(p.signal_dbm / width) * width)
        b = bands.setdefault(lo, [0, 0])
        b[0] += 1
        b[1] += 1 if p.loss_free else 0
    return [{"lo": lo, "hi": lo + width, "seconds": n, "clean_seconds": c,
             "clean_pct": 100.0 * c / n} for lo, (n, c) in sorted(bands.items(), reverse=True)]


def _fmt_dbm(v: Optional[float]) -> str:
    return "–" if v is None else f"{v:.0f} dBm"


def walk_verdict(points: Sequence[WalkPoint], window: float = WINDOW_S) -> Dict[str, object]:
    """The answer, in seconds and dBm."""
    pts = list(points)
    judged = [p for p in pts if p.loss_free is not None]
    out: Dict[str, object] = {
        "duration_s": pts[-1].t_s if pts else 0.0, "seconds_judged": len(judged),
        "seconds_unknown": len(pts) - len(judged), "first_failure": None, "clean_for_s": 0,
        "clean_down_to_dbm": None, "strongest_failing_dbm": None, "loss_free_pct": None,
        "failure_episodes": 0, "worst_stall_s": _worst([p.stall_s for p in pts]),
        "worst_ping_loss": _worst([p.ping_loss for p in pts]),
        "worst_udp_loss": _worst([v for p in pts for v in (p.udp_up_loss, p.udp_down_loss)]),
        "weakest_dbm": min([p.signal_dbm for p in pts if p.signal_dbm is not None], default=None),
        "bands": signal_bands(pts), "window_s": window}
    if not judged:
        out["headline"] = ("No loss figure was measured (ping, UDP and video all missing), so no "
                           "verdict can be given.")
        out["findings"] = [out["headline"]]
        return out

    fails = [p for p in pts if p.loss_free is False]
    out["loss_free_pct"] = 100.0 * sum(1 for p in judged if p.loss_free) / len(judged)
    episodes, prev = 0, True
    for p in pts:
        if p.loss_free is False and prev:
            episodes += 1
        if p.loss_free is not None:
            prev = bool(p.loss_free)
    out["failure_episodes"] = episodes
    out["strongest_failing_dbm"] = max([p.signal_dbm for p in fails if p.signal_dbm is not None], default=None)

    first = fails[0] if fails else None
    before = [p for p in (pts[:pts.index(first)] if first else pts) if p.loss_free]
    out["clean_for_s"] = len(before)
    out["clean_down_to_dbm"] = min([p.signal_min_dbm if p.signal_min_dbm is not None else p.signal_dbm
                                    for p in before if p.signal_dbm is not None], default=None)
    if first is not None:
        out["first_failure"] = {"t_s": first.t_s, "signal_dbm": first.signal_dbm,
                                "signal_min_dbm": first.signal_min_dbm, "reasons": first.reasons}

    notes: List[str] = []
    if first is None:
        head = (f"Live feed stayed loss-free for the whole walk ({out['duration_s']:g} s)"
                + (f", down to {_fmt_dbm(out['clean_down_to_dbm'])}" if out["clean_down_to_dbm"] is not None else "")
                + ". The true limit is beyond where you walked.")
    elif out["clean_for_s"] == 0:
        head = (f"Loss was already present at the start (first loss at {first.t_s:g} s, "
                f"{_fmt_dbm(first.signal_dbm)}) - the live feed was never loss-free.")
    else:
        head = (f"Live feed was loss-free for the first {out['clean_for_s']} s"
                + (f", down to {_fmt_dbm(out['clean_down_to_dbm'])}" if out["clean_down_to_dbm"] is not None else "")
                + f". First loss at about {first.t_s:g} s ({_fmt_dbm(first.signal_dbm)}).")
    out["headline"] = head
    notes.append(head)
    if first is not None:
        notes.append("First loss: " + (", ".join(first.reasons) or "loss detected") + ".")
    sf, cd = out["strongest_failing_dbm"], out["clean_down_to_dbm"]
    if sf is not None and cd is not None:
        if sf > cd + 3.0:
            notes.append(
                f"Loss also occurred as strong as {_fmt_dbm(sf)}, well above the weakest clean signal "
                f"({_fmt_dbm(cd)}): signal strength alone did not decide it - look for walls, corners and "
                "interference at those moments.")
        else:
            notes.append(f"Loss began around {_fmt_dbm(sf)}; the weakest clean signal was {_fmt_dbm(cd)}.")
    notes.append(f"{out['loss_free_pct']:.0f} % of the {len(judged)} measured seconds were loss-free"
                 + (f"; {episodes} separate loss episode(s)." if episodes else "."))
    for label, key, unit in (("longest video stall", "worst_stall_s", " s"), ("worst ping loss", "worst_ping_loss", " %"),
                             ("worst UDP loss", "worst_udp_loss", " %")):
        if out[key] is not None:
            notes.append(f"{label[0].upper() + label[1:]}: {out[key]:.1f}{unit}.")
    if out["seconds_unknown"]:
        notes.append(f"{out['seconds_unknown']} second(s) had no loss figure and are not counted either way.")
    notes.append(f"Each second is judged over the {window:g} s ending then. There is no distance in this mode: "
                 "results are time and signal only.")
    out["findings"] = notes
    return out


# ───────────────────────────── recorder (drives a Session) ────────────────

class WalkRecorder:
    """Turns the sampler data of a running Session into per-second points.
    Incremental: each update() analyses only the seconds that are new."""

    def __init__(self, session):
        self.session = session
        self.t0: Optional[float] = None
        self.t_end: Optional[float] = None
        self.points: List[WalkPoint] = []
        self._k = 1

    @property
    def active(self) -> bool:
        return self.t0 is not None and self.t_end is None

    def begin(self) -> None:
        self.t0, self.t_end, self.points, self._k = time.monotonic(), None, [], 1

    def update(self, now: Optional[float] = None) -> List[WalkPoint]:
        """New points since the last call."""
        if self.t0 is None:
            return []
        upto = self.t_end if self.t_end is not None else (time.monotonic() if now is None else now)
        new = compute_points(self.session.store, self.t0, upto, self.session.analysis,
                             self.session.cfg.loss_tolerance, start_k=self._k)
        self._k += len(new)
        self.points.extend(new)
        return new

    def end(self) -> List[WalkPoint]:
        self.t_end = time.monotonic()
        return self.update()

    def verdict(self) -> Dict[str, object]:
        return walk_verdict(self.points)

    def finish(self) -> Optional[Dict[str, object]]:
        c = self.session.cfg
        if not self.points:
            return None
        out_dir = c.out_dir or os.path.join(os.path.expanduser("~"), "link_range_results",
                                            time.strftime("%Y%m%d_%H%M%S"))
        c.out_dir = out_dir
        radxa_after = L.radxa_wifi_signal(c.control_target) if c.control_target else None
        meta = {"mode": "continuous walk (time and signal, no distance)",
                "started": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), "radxa": c.radxa,
                "wifi_iface": self.session.iface, "video_url": c.url if c.use_video else "off",
                "udp": self.session.udp_summary(),
                "allowed loss %": c.loss_tolerance, "window_s": WINDOW_S,
                "radxa_wifi_signal_dbm (before/after)": f"{self.session.radxa_before} / {radxa_after}"}
        timeline = L.build_timeline(self.session.store, self.t0, self.t_end or time.monotonic(),
                                    self.session.analysis, [])
        return finish_walk(self.points, meta, out_dir, timeline)


# ───────────────────────────── reports ─────────────────────────────────────

def stream_line(p: WalkPoint) -> str:
    """One line of the live stream."""
    udp = ("" if p.udp_up_loss is None and p.udp_down_loss is None
           else f"  UDP {L._fmt(p.udp_up_loss, '{:.0f}')}/{L._fmt(p.udp_down_loss, '{:.0f}')} %")
    state = {True: "ok", False: "LOSS: " + ", ".join(p.reasons), None: "no data"}[p.loss_free]
    return (f"{p.t_s:6.0f} s  {L._fmt(p.signal_dbm, '{:.0f}'):>4} dBm  ping {L._fmt(p.ping_loss, '{:.0f}'):>3} %  "
            f"video {L._fmt(p.video_fps, '{:.1f}'):>5} fps  stall {L._fmt(p.stall_s, '{:.1f}'):>4} s{udp}  {state}")


def _series(points, attr):
    return [(p.t_s, getattr(p, attr)) for p in points]


def walk_html(meta: Dict[str, object], points: Sequence[WalkPoint], verdict: Dict[str, object]) -> str:
    th = L.THRESHOLDS
    charts = [
        L.svg_chart("Signal at the laptop", [("dBm", "#58a6ff", _series(points, "signal_dbm"))], "dBm",
                    hlines=[(th["good_rssi"], "good"), (th["poor_rssi"], "poor")],
                    xlabel="time (s)", max_xticks=10),
        L.svg_chart("Camera feed", [("frames/s", "#58a6ff", _series(points, "video_fps")),
                                    ("Mbit/s", "#8b949e", _series(points, "video_mbps"))], "fps / Mbit/s",
                    xlabel="time (s)", max_xticks=10),
        L.svg_chart("Longest gap between video frames", [("seconds", "#d29922", _series(points, "stall_s"))], "s",
                    hlines=[(th["good_stall_s"], "stall")], xlabel="time (s)", max_xticks=10),
        L.svg_chart("Data lost", [("ping %", "#d29922", _series(points, "ping_loss")),
                                  ("UDP up %", "#8b949e", _series(points, "udp_up_loss")),
                                  ("UDP down %", "#58a6ff", _series(points, "udp_down_loss"))], "% lost",
                    xlabel="time (s)", max_xticks=10),
    ]
    first = verdict.get("first_failure")
    clean = verdict.get("clean_for_s", 0)
    big = f"{clean} s" if verdict.get("loss_free_pct") is not None else "–"
    bands = "".join(
        f"<tr><td>{b['lo']} to {b['hi']} dBm</td><td>{b['seconds']}</td><td>{b['clean_seconds']}</td>"
        f"<td class='{'good' if b['clean_pct'] >= 100 else 'marg' if b['clean_pct'] >= 80 else 'poor'}'>"
        f"{b['clean_pct']:.0f} %</td></tr>" for b in verdict.get("bands", []))
    bad = [p for p in points if p.loss_free is False]
    bad_rows = "".join(
        f"<tr><td>{p.t_s:g}</td><td>{html.escape(L._fmt(p.signal_dbm, '{:.0f}'))}</td>"
        f"<td style='text-align:left'>{html.escape(', '.join(p.reasons))}</td></tr>"
        for p in bad[:MAX_LISTED_LOSS_ROWS])
    if len(bad) > MAX_LISTED_LOSS_ROWS:
        bad_rows += (f"<tr><td colspan='3' style='text-align:left'>... and {len(bad) - MAX_LISTED_LOSS_ROWS} "
                     "more (all of them are in walk.json)</td></tr>")
    meta_rows = "".join(f"<tr><th>{html.escape(str(k))}</th><td>{html.escape(str(v))}</td></tr>" for k, v in meta.items())
    notes = "".join(f"<li>{html.escape(n)}</li>" for n in verdict.get("findings", []))
    where = (f"<div class='hs'>First loss at {first['t_s']:g} s, {html.escape(_fmt_dbm(first['signal_dbm']))}</div>"
             if first else "")
    return f"""<!doctype html><html><head><meta charset='utf-8'><title>Link range test - continuous walk</title>
<style>
body{{font-family:'Ubuntu','Noto Sans',sans-serif;background:#0d1117;color:#c9d1d9;max-width:1000px;margin:24px auto;padding:0 16px}}
h1{{font-size:20px;letter-spacing:1px}} h2{{font-size:13px;letter-spacing:1.5px;color:#8b949e;text-transform:uppercase}}
table{{border-collapse:collapse;width:100%;font-size:13px}} th,td{{border:1px solid #30363d;padding:5px 8px;text-align:right}}
th{{background:#161b22;color:#8b949e;font-weight:600}} td.good{{color:#c9d1d9}} td.marg{{color:#d29922;font-weight:700}}
td.poor{{color:#f85149;font-weight:700}} .meta th{{text-align:left}} .meta td{{text-align:left}}
.charts{{display:grid;grid-template-columns:repeat(auto-fit,minmax(440px,1fr));gap:12px}}
svg.chart{{background:#161b22;border:1px solid #262c34;border-radius:8px;width:100%}} .grid{{stroke:#262c34}} .limit{{stroke:#6e7681;stroke-dasharray:4 3}}
.ax{{fill:#8b949e;font-size:10px}} .ct{{fill:#c9d1d9;font-size:12px;font-weight:600}} .muted{{color:#6e7681}}
li{{margin:4px 0}}
.hero{{background:#161b22;border:1px solid #262c34;border-radius:8px;padding:14px 18px;margin:10px 0 6px}}
.hl{{font-size:11px;letter-spacing:2px;color:#8b949e;font-weight:600}} .hv{{font-size:40px;font-weight:800;color:#f0f6fc;line-height:1.15}}
.hs{{color:#8b949e;font-size:13px;margin-top:4px}}
</style></head><body>
<h1>LINK RANGE TEST - CONTINUOUS WALK</h1>
<div class='hero'><div class='hl'>LIVE FEED LOSS-FREE FOR</div><div class='hv'>{html.escape(big)}</div>
<div class='hs'>down to {html.escape(_fmt_dbm(verdict.get('clean_down_to_dbm')))}</div>{where}</div>
<h2>Findings</h2><ul>{notes}</ul>
<h2>By signal level</h2>
<table><tr><th>Signal</th><th>Seconds</th><th>Loss-free seconds</th><th>Loss-free</th></tr>{bands or "<tr><td colspan='4'>no data</td></tr>"}</table>
{('<h2>Seconds with loss</h2><table><tr><th>t (s)</th><th>Signal dBm</th><th>What</th></tr>' + bad_rows + '</table>') if bad_rows else ''}
<h2>Charts</h2><div class='charts'>{''.join(charts)}</div>
<h2>Run details</h2><table class='meta'>{meta_rows}</table>
<p class='muted'>Time is seconds since you pressed start. There is no distance in this mode.</p>
</body></html>"""


def walk_markdown(meta: Dict[str, object], points: Sequence[WalkPoint], verdict: Dict[str, object]) -> str:
    lines = ["# Link range test - continuous walk", "", "## Findings", ""]
    lines += [f"- {n}" for n in verdict.get("findings", [])]
    lines += ["", "## By signal level", "", "| Signal | Seconds | Loss-free seconds | Loss-free |", "|---|---|---|---|"]
    lines += [f"| {b['lo']} to {b['hi']} dBm | {b['seconds']} | {b['clean_seconds']} | {b['clean_pct']:.0f} % |"
              for b in verdict.get("bands", [])]
    bad = [p for p in points if p.loss_free is False]
    if bad:
        lines += ["", "## Seconds with loss", "", "| t (s) | Signal dBm | What |", "|---|---|---|"]
        lines += [f"| {p.t_s:g} | {L._fmt(p.signal_dbm, '{:.0f}')} | {', '.join(p.reasons)} |" for p in bad]
    lines += ["", "## Run details", ""] + [f"- **{k}**: {v}" for k, v in meta.items()]
    return "\n".join(lines) + "\n"


def finish_walk(points: Sequence[WalkPoint], meta: Dict[str, object], out_dir: str,
                timeline: Optional[List[dict]] = None) -> Dict[str, object]:
    verdict = walk_verdict(points)
    summary = {"mode": "walk", "verdict": verdict, "findings": verdict["findings"], "meta": meta,
               "thresholds": L.THRESHOLDS}
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "walk.json"), "w", encoding="utf-8") as fh:
        json.dump([asdict(p) for p in points], fh, indent=2)
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2)
    with open(os.path.join(out_dir, "report.html"), "w", encoding="utf-8") as fh:
        fh.write(walk_html(meta, points, verdict))
    with open(os.path.join(out_dir, "report.md"), "w", encoding="utf-8") as fh:
        fh.write(walk_markdown(meta, points, verdict))
    if timeline:
        with open(os.path.join(out_dir, "samples.csv"), "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(timeline[0].keys()))
            w.writeheader()
            w.writerows(timeline)
    return summary


# ───────────────────────────── demo ────────────────────────────────────────

def demo_points(seconds: int = 90, seed: int = 11) -> List[WalkPoint]:
    """Synthetic walk: signal falls from -45 dBm, trouble from about -70 dBm."""
    import random
    rng = random.Random(seed)
    out = []
    for t in range(1, seconds + 1):
        sig = -45.0 - 0.38 * t + rng.uniform(-1.5, 1.5)
        stress = max(0.0, (-sig - 68.0) / 12.0)
        s = L.HoldStats(distance_m=float("nan"), signal_mean=sig, signal_min=sig - 2, tx_mbps=max(6.0, 150 * (1 - stress)),
                        ping_loss=min(60.0, 25 * stress ** 2) if stress > 0.2 else 0.0, ping_p95=5 + 80 * stress,
                        video_fps=24 * max(0.1, 1 - 0.7 * stress), video_mbps=8 * max(0.1, 1 - 0.7 * stress),
                        video_stall_max=0.1 + 2.5 * stress ** 2,
                        udp_up_loss=min(50.0, 10 * stress ** 2) if stress > 0.3 else 0.0,
                        udp_down_loss=min(50.0, 15 * stress ** 2) if stress > 0.3 else 0.0)
        out.append(point_from_stats(float(t), s))
    return out


# ───────────────────────────── terminal flow ───────────────────────────────

def run_walk(args, sess, out: Callable[[str], None] = print) -> int:
    """Stream one line per second while the operator walks; finish on Enter, on
    --walk-seconds, or on Ctrl+C; then write the report."""
    rec = WalkRecorder(sess)
    secs = float(getattr(args, "walk_seconds", 0) or 0)
    auto = bool(getattr(args, "auto", False))
    stop = threading.Event()
    try:
        out("\nStand next to the Radxa/router.")
        if not auto:
            input("Press Enter, then start walking away at your own pace... ")
        rec.begin()
        out("Measuring. One line per second; press Enter to finish"
            + (f" (or it stops by itself after {secs:g} s)." if secs else "."))
        if sys.stdin is not None and sys.stdin.isatty() and not (auto and secs):
            def _wait():
                try:
                    input()
                except EOFError:
                    return
                stop.set()
            threading.Thread(target=_wait, daemon=True).start()
        while not stop.is_set():
            stop.wait(STEP_S)
            for p in rec.update():
                out(stream_line(p))
            if secs and rec.points and rec.points[-1].t_s >= secs:
                break
    except KeyboardInterrupt:
        out("\nInterrupted - writing what was measured.")
    finally:
        for p in rec.end():
            out(stream_line(p))
        sess.stop()
    summary = rec.finish()
    if summary is None:
        out("No seconds were recorded - nothing to report.")
        return 1
    out("\n" + "\n".join("• " + n for n in summary["findings"]))
    out(f"\nReport: {os.path.join(sess.cfg.out_dir, 'report.html')}")
    return 0


def run_demo(args, out_dir: str, out: Callable[[str], None] = print) -> int:
    points = demo_points()
    meta = {"mode": "DEMO (synthetic data - not a measurement)"}
    summary = finish_walk(points, meta, out_dir)
    out("\n".join("• " + n for n in summary["findings"]))
    out(f"\nReport: {os.path.join(out_dir, 'report.html')}")
    return 0
