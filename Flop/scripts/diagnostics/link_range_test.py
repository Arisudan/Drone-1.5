#!/usr/bin/env python3
"""
================================================================================
SCRIPT: link_range_test.py
PURPOSE: Walk-away range test for the camera-feed Wi-Fi link
================================================================================

WHAT IT ANSWERS
  "How far can I carry the laptop from the Radxa/router before the camera feed
  degrades, and how much data is actually lost on the way?"

SETUP
  Radxa and router stay together. You walk away with the laptop. The only radio
  link that changes is the laptop's Wi-Fi to the router, so that is the link whose
  signal is read; everything else is measured end to end to the Radxa.

HOW DISTANCE IS KNOWN
  Indoors there is no GPS, so distance comes from YOU: walk to a measured spot
  (tape, floor tiles) and press Enter. Everything recorded while you stand there
  is tagged with that distance. A log-distance path-loss curve is then fitted to
  signal-vs-distance, so afterwards the distance can be ESTIMATED from signal
  alone. Indoors that estimate is rough, and the report says so.

WHAT IS MEASURED (all at once, all with timestamps)
  Wi-Fi radio     signal dBm, link speed, TX retries/failed        (iw, laptop card)
  Network         ping RTT, jitter, % lost - small and video-sized packets
  Camera feed     Mbit/s, frames/s, longest stall, reconnects      (the real MJPEG)
  Data loss       UDP packets lost up and down, at a chosen bitrate (optional)

  "Data lost" is reported three ways: ping loss, UDP packet loss, and video frame
  drop versus the close-range baseline. None of this sends a MAVLink command.

TYPICAL RUN
  python3 link_range_test.py --radxa 172.16.100.182 --step 5 --udp --start-helper radxa@172.16.100.182
  python3 link_range_test.py --demo            # synthetic data -> report, no network

OUTPUT (in --out, default ~/link_range_results/<timestamp>/)
  samples.csv  1 Hz timeline         holds.json  per-distance statistics
  summary.json range + path-loss fit report.html / report.md  the readable result

Standard library only. matplotlib is NOT needed: the charts are inline SVG.
================================================================================
"""

from __future__ import annotations

import argparse
import bisect
import csv
import html
import json
import math
import os
import re
import socket
import statistics
import struct
import subprocess
import sys
import threading
import time
import urllib.request
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# ───────────────────────────── thresholds ──────────────────────────────────

THRESHOLDS = {
    "good_fps_frac": 0.80, "poor_fps_frac": 0.50,    # video fps vs close-range baseline
    "good_loss_pct": 2.0, "poor_loss_pct": 10.0,     # ping / UDP loss
    "good_stall_s": 0.5, "poor_stall_s": 2.0,        # longest gap between video frames
    "good_rssi": -70.0, "poor_rssi": -80.0,          # dBm at the laptop
}

# ───────────────────────────── parsers (pure) ──────────────────────────────

PING_RE = re.compile(r"icmp_seq=(\d+).*?time=([\d.]+)\s*ms")


def parse_ping_line(line: str) -> Optional[Tuple[int, float]]:
    """(seq, rtt_ms) from one line of `ping` output, or None for anything else."""
    m = PING_RE.search(line)
    return (int(m.group(1)), float(m.group(2))) if m else None


def _num(pattern: str, text: str, flags: int = 0) -> Optional[float]:
    m = re.search(pattern, text, flags)
    return float(m.group(1)) if m else None


def parse_iw_link(text: str) -> Dict[str, object]:
    """Fields from `iw dev <if> link`. Missing fields are None; 'connected' says
    whether there is an association at all."""
    if "Not connected" in text or not text.strip():
        return {"connected": False}
    ssid = re.search(r"SSID:\s*(.+)", text)
    return {
        "connected": True,
        "ssid": ssid.group(1).strip() if ssid else None,
        "freq_mhz": _num(r"freq:\s*([\d.]+)", text),
        "signal_dbm": _num(r"^\s*signal:\s*(-?\d+)", text, re.M),
        "tx_mbps": _num(r"tx bitrate:\s*([\d.]+)\s*MBit/s", text),
        "rx_mbps": _num(r"rx bitrate:\s*([\d.]+)\s*MBit/s", text),
    }


def parse_iw_station(text: str) -> Dict[str, Optional[float]]:
    """Cumulative counters from `iw dev <if> station dump`."""
    return {
        "signal_dbm": _num(r"^\s*signal:\s*(-?\d+)", text, re.M),
        "signal_avg_dbm": _num(r"signal avg:\s*(-?\d+)", text),
        "tx_retries": _num(r"tx retries:\s*(\d+)", text),
        "tx_failed": _num(r"tx failed:\s*(\d+)", text),
        "tx_packets": _num(r"tx packets:\s*(\d+)", text),
        "rx_packets": _num(r"rx packets:\s*(\d+)", text),
        "beacon_loss": _num(r"beacon loss:\s*(\d+)", text),
        "tx_mbps": _num(r"tx bitrate:\s*([\d.]+)\s*MBit/s", text),
        "rx_mbps": _num(r"rx bitrate:\s*([\d.]+)\s*MBit/s", text),
    }


def parse_proc_wireless(text: str, iface: str) -> Dict[str, Optional[float]]:
    """Fallback when `iw` is missing: link quality / level / noise."""
    for line in text.splitlines():
        if line.strip().startswith(iface + ":"):
            parts = line.split(":", 1)[1].split()
            if len(parts) >= 4:
                try:
                    return {"signal_dbm": float(parts[2].rstrip(".")),
                            "link_quality": float(parts[1].rstrip(".")),
                            "noise_dbm": float(parts[3].rstrip("."))}
                except ValueError:
                    pass
    return {}


def parse_iw_dev(text: str) -> List[str]:
    return re.findall(r"^\s*Interface\s+(\S+)", text, re.M)


def detect_wifi_iface() -> Optional[str]:
    try:
        out = subprocess.run(["iw", "dev"], capture_output=True, text=True, timeout=4).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    names = parse_iw_dev(out)
    return names[0] if names else None


# ───────────────────────────── JPEG frame counting ─────────────────────────

SOI, EOI = b"\xff\xd8", b"\xff\xd9"


class JpegFrameCounter:
    """Counts complete JPEG frames in an arbitrarily chunked byte stream.

    An MJPEG feed is a run of JPEGs inside multipart framing. Entropy-coded JPEG
    data escapes every 0xFF as 0xFF00, so the two-byte sequences FFD8 / FFD9 occur
    only as start / end of image - which makes counting them a sound frame
    counter that does not need to parse the HTTP boundaries. A marker split
    across two network chunks is handled by carrying one byte over.
    """

    def __init__(self) -> None:
        self._tail = b""
        self._in_frame = False
        self._fb = 0            # frame bytes already consumed in earlier chunks

    def feed(self, data: bytes) -> List[int]:
        """Returns the byte size of each frame completed by this chunk."""
        buf = self._tail + data
        self._tail = b""
        sizes: List[int] = []
        pos = 0
        frame_start: Optional[int] = 0 if self._in_frame else None
        while True:
            if frame_start is None:
                j = buf.find(SOI, pos)
                if j < 0:
                    break
                frame_start, self._fb, pos, self._in_frame = j, 0, j + 2, True
            k = buf.find(EOI, pos)
            if k < 0:
                break
            sizes.append(self._fb + (k + 2 - frame_start))
            frame_start, self._fb, pos, self._in_frame = None, 0, k + 2, False
        if self._in_frame:
            keep = max(frame_start, len(buf) - 1)       # last byte may be half a marker
            self._fb += keep - frame_start
            self._tail = buf[keep:]
        elif buf.endswith(b"\xff"):
            self._tail = buf[-1:]
        return sizes


# ───────────────────────────── UDP probe packets ───────────────────────────

MAGIC = b"LRT1"
HDR = struct.Struct("!4sIdI")      # magic, seq, client send time (s), server count
DEFAULT_UDP_PORT = 9099


def make_packet(seq: int, t_send: float, size: int, srv_count: int = 0) -> bytes:
    head = HDR.pack(MAGIC, seq & 0xFFFFFFFF, t_send, srv_count & 0xFFFFFFFF)
    return head + b"\x00" * max(0, size - len(head))


def parse_packet(data: bytes) -> Optional[Tuple[int, float, int]]:
    """(seq, t_send, srv_count) or None for a datagram that is not ours."""
    if len(data) < HDR.size:
        return None
    magic, seq, t_send, srv_count = HDR.unpack_from(data)
    return (seq, t_send, srv_count) if magic == MAGIC else None


# ───────────────────────────── sample store ────────────────────────────────

class Store:
    """Timestamped events by kind. Thread-safe append, windowed read."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._t: Dict[str, List[float]] = {}
        self._v: Dict[str, List[dict]] = {}

    def add(self, kind: str, t: Optional[float] = None, **fields) -> None:
        t = time.monotonic() if t is None else t
        with self._lock:
            self._t.setdefault(kind, []).append(t)
            self._v.setdefault(kind, []).append(fields)

    def window(self, kind: str, t0: float, t1: float) -> List[Tuple[float, dict]]:
        """Events with t0 < t <= t1."""
        with self._lock:
            ts, vs = self._t.get(kind, []), self._v.get(kind, [])
            i, j = bisect.bisect_right(ts, t0), bisect.bisect_right(ts, t1)
            return list(zip(ts[i:j], vs[i:j]))

    def last_at_or_before(self, kind: str, t: float) -> Optional[Tuple[float, dict]]:
        with self._lock:
            ts = self._t.get(kind, [])
            i = bisect.bisect_right(ts, t)
            return (ts[i - 1], self._v[kind][i - 1]) if i else None

    def count(self, kind: str) -> int:
        with self._lock:
            return len(self._t.get(kind, []))


# ───────────────────────────── analysis (pure) ─────────────────────────────

def percentile(values: Sequence[float], p: float) -> Optional[float]:
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def counter_delta(values: Sequence[float]) -> float:
    """Total increase of a cumulative counter across samples, surviving resets.

    The Wi-Fi card's counters restart from zero when it roams to another access
    point; a plain last-minus-first then goes NEGATIVE. When a value drops, the
    counter restarted, so the new reading is itself the increase since the reset.
    """
    total = 0.0
    for prev, cur in zip(values, values[1:]):
        total += (cur - prev) if cur >= prev else cur
    return total


def jitter_ms(rtts: Sequence[float]) -> Optional[float]:
    if len(rtts) < 2:
        return None
    return statistics.fmean(abs(b - a) for a, b in zip(rtts, rtts[1:]))


def loss_pct(received: int, expected: float) -> Optional[float]:
    if expected <= 0:
        return None
    return max(0.0, min(100.0, (1.0 - received / expected) * 100.0))


EDGE_GRACE_S = 1.0     # silence at the start/end of a hold shorter than this is not "loss"


def ping_loss_from_replies(replies: Sequence[Tuple[float, int]], t0: float, t1: float,
                           interval: float, grace: float = EDGE_GRACE_S) -> Optional[float]:
    """Percent of pings lost in (t0, t1], counted from SEQUENCE NUMBERS.

    Comparing replies with duration / interval cannot give a strict zero: a reply
    still in flight when the window ends makes a perfect link look 1-2 packets
    short. So: packets missing INSIDE the received sequence span are loss, plus a
    dead stretch at the start or end if it lasted longer than `grace`. A window
    with no reply at all is 100 % loss."""
    dur = t1 - t0
    if dur <= 0:
        return None
    if not replies:
        return 100.0
    uniq = sorted({seq for _, seq in replies})
    span = uniq[-1] - uniq[0] + 1
    missing = span - len(uniq)
    t_first, t_last = min(t for t, _ in replies), max(t for t, _ in replies)
    head = max(0.0, (t_first - t0) / interval - 1) if t_first - t0 > grace else 0.0
    tail = max(0.0, (t1 - t_last) / interval - 1) if t1 - t_last > grace else 0.0
    total = span + head + tail
    return max(0.0, min(100.0, (missing + head + tail) / total * 100.0))


def udp_loss_from_echoes(tx_times: Sequence[float], echoes: Sequence[Tuple[float, int, int]],
                         t0: float, t1: float, grace: float = EDGE_GRACE_S
                         ) -> Tuple[Optional[float], Optional[float]]:
    """(uplink %, downlink %) from echoes = [(t, seq, server_count), ...].

    Each echo carries its own sequence number and how many probes the server had
    received when it sent it, so between two echoes:
        uplink lost   = (seq2 - seq1) - (count2 - count1)       sent minus received
        downlink lost = (count2 - count1) - (echoes between)    received minus echoed back
    exact, with no in-flight packets at the window edges. A stretch of silence at
    either end longer than `grace` is added to both directions (when the link goes
    quiet the direction cannot be told apart)."""
    if not tx_times:
        return None, None
    if not echoes:
        return 100.0, 100.0
    ev = sorted(echoes, key=lambda e: e[1])
    (_, s1, c1), (_, s2, c2) = ev[0], ev[-1]
    span_up, span_down = s2 - s1, c2 - c1
    up_lost = max(0, span_up - span_down)
    down_lost = max(0, span_down - (len(ev) - 1))
    t_first, t_last = min(e[0] for e in ev), max(e[0] for e in ev)
    tail = sum(1 for t in tx_times if t > t_last + 0.3) if t1 - t_last > grace else 0
    head = sum(1 for t in tx_times if t <= t_first - 0.3) if t_first - t0 > grace else 0
    extra = tail + head
    up_total, down_total = span_up + extra, span_down + extra
    if up_total <= 0 and down_total <= 0:
        return None, None
    up = min(100.0, (up_lost + extra) / up_total * 100.0) if up_total > 0 else None
    down = min(100.0, (down_lost + extra) / down_total * 100.0) if down_total > 0 else None
    return up, down


@dataclass
class Config:
    ping_interval: float = 0.2
    udp_enabled: bool = False
    video_enabled: bool = True


@dataclass
class HoldStats:
    distance_m: float
    t0: float = 0.0
    t1: float = 0.0
    signal_mean: Optional[float] = None
    signal_min: Optional[float] = None
    tx_mbps: Optional[float] = None
    retries_per_s: Optional[float] = None
    failed_per_s: Optional[float] = None
    ping_loss: Optional[float] = None
    ping_avg: Optional[float] = None
    ping_p95: Optional[float] = None
    ping_max: Optional[float] = None
    ping_jitter: Optional[float] = None
    pingL_loss: Optional[float] = None
    pingL_p95: Optional[float] = None
    video_fps: Optional[float] = None
    video_mbps: Optional[float] = None
    video_stall_max: Optional[float] = None
    video_disconnects: int = 0
    video_drop: Optional[float] = None
    udp_up_loss: Optional[float] = None
    udp_down_loss: Optional[float] = None
    udp_rtt_p95: Optional[float] = None
    loss_free: Optional[bool] = None
    verdict: str = "n/a"
    reasons: List[str] = field(default_factory=list)


def analyse_window(store: Store, t0: float, t1: float, cfg: Config,
                   distance_m: float = float("nan")) -> HoldStats:
    """Every metric for the window (t0, t1]. A metric with no data stays None -
    "not measured" is never reported as a perfect 0 % loss."""
    dur = max(1e-9, t1 - t0)
    s = HoldStats(distance_m=distance_m, t0=t0, t1=t1)

    wifi = store.window("wifi", t0, t1)
    sig = [f["signal_dbm"] for _, f in wifi if f.get("signal_dbm") is not None]
    if sig:
        s.signal_mean, s.signal_min = statistics.fmean(sig), min(sig)
    rate = [f["tx_mbps"] for _, f in wifi if f.get("tx_mbps") is not None]
    if rate:
        s.tx_mbps = statistics.fmean(rate)
    ctr = [(t, f) for t, f in wifi if f.get("tx_retries") is not None]
    if len(ctr) >= 2 and ctr[-1][0] > ctr[0][0]:
        span = ctr[-1][0] - ctr[0][0]
        s.retries_per_s = counter_delta([f["tx_retries"] for _, f in ctr]) / span
        failed = [f["tx_failed"] for _, f in ctr if f.get("tx_failed") is not None]
        if len(failed) == len(ctr):
            s.failed_per_s = counter_delta(failed) / span

    for kind in ("ping_s", "ping_l"):
        if store.count(kind) == 0:
            continue
        ev = store.window(kind, t0, t1)
        loss = ping_loss_from_replies([(t, f["seq"]) for t, f in ev], t0, t1, cfg.ping_interval)
        rtts = [f["rtt"] for _, f in ev]
        if kind == "ping_s":
            s.ping_loss = loss
            if rtts:
                s.ping_avg, s.ping_p95, s.ping_max = statistics.fmean(rtts), percentile(rtts, 95), max(rtts)
                s.ping_jitter = jitter_ms(rtts)
        else:
            s.pingL_loss = loss
            if rtts:
                s.pingL_p95 = percentile(rtts, 95)

    if cfg.video_enabled and (store.count("video_frame") or store.count("video_event")):
        frames = store.window("video_frame", t0, t1)
        s.video_fps = len(frames) / dur
        nbytes = sum(f["n"] for _, f in store.window("video_bytes", t0, t1))
        s.video_mbps = nbytes * 8 / dur / 1e6
        stamps = [t for t, _ in frames]
        edges = [t0] + stamps + [t1]
        s.video_stall_max = max(b - a for a, b in zip(edges, edges[1:]))
        s.video_disconnects = sum(1 for _, f in store.window("video_event", t0, t1)
                                  if f.get("event") == "disconnect")

    if cfg.udp_enabled and store.count("udp_tx"):
        tx_times = [t for t, _ in store.window("udp_tx", t0, t1)]
        echoes = store.window("udp_rx", t0, t1)
        s.udp_up_loss, s.udp_down_loss = udp_loss_from_echoes(
            tx_times, [(t, f["seq"], f["srv"]) for t, f in echoes], t0, t1)
        s.udp_rtt_p95 = percentile([f["rtt"] for _, f in echoes], 95)
    return s


def verdict_for(s: HoldStats, baseline_fps: Optional[float],
                thr: Dict[str, float] = THRESHOLDS) -> Tuple[str, List[str]]:
    """Good / Marginal / Poor from whatever was measured, with the reasons."""
    poor: List[str] = []
    marg: List[str] = []

    def check(value, good_bad, poor_bad, label, fmt, higher_is_bad=True):
        if value is None:
            return
        bad_p = value > poor_bad if higher_is_bad else value < poor_bad
        bad_m = value > good_bad if higher_is_bad else value < good_bad
        if bad_p:
            poor.append(f"{label} {fmt.format(value)}")
        elif bad_m:
            marg.append(f"{label} {fmt.format(value)}")

    check(s.signal_mean, thr["good_rssi"], thr["poor_rssi"], "signal", "{:.0f} dBm", higher_is_bad=False)
    for val, name in ((s.ping_loss, "ping loss"), (s.udp_up_loss, "UDP up loss"),
                      (s.udp_down_loss, "UDP down loss")):
        check(val, thr["good_loss_pct"], thr["poor_loss_pct"], name, "{:.1f} %")
    check(s.video_stall_max, thr["good_stall_s"], thr["poor_stall_s"], "video stall", "{:.1f} s")
    if baseline_fps and s.video_fps is not None:
        frac = s.video_fps / baseline_fps
        check(frac, thr["good_fps_frac"], thr["poor_fps_frac"], "video fps",
              "{:.0%} of baseline", higher_is_bad=False)
    if s.video_disconnects:
        marg.append(f"{s.video_disconnects} video reconnect(s)")
    if poor:
        return "Poor", poor + marg
    if marg:
        return "Marginal", marg
    return "Good", []


def loss_reasons(s: HoldStats, tol_pct: float = 0.0) -> List[str]:
    """Which measurements show loss (or a stall) beyond the tolerance."""
    out: List[str] = []
    for val, name in ((s.ping_loss, "ping"), (s.pingL_loss, "large ping"),
                      (s.udp_up_loss, "UDP up"), (s.udp_down_loss, "UDP down")):
        if val is not None and val > tol_pct + 1e-9:
            out.append(f"{name} loss {val:.1f} %")
    if s.video_disconnects:
        out.append(f"{s.video_disconnects} video reconnect(s)")
    if s.video_stall_max is not None and s.video_stall_max > THRESHOLDS["good_stall_s"]:
        out.append(f"video stall {s.video_stall_max:.1f} s")
    return out


def is_loss_free(s: HoldStats, tol_pct: float = 0.0) -> Optional[bool]:
    """True when NOTHING was lost: every measured loss figure is within the
    tolerance (default 0 = none at all), the video never reconnected, and it never
    stalled. None when no loss figure was measured at all - "no data" must never
    be read as "no loss"."""
    measured = [v for v in (s.ping_loss, s.pingL_loss, s.udp_up_loss, s.udp_down_loss) if v is not None]
    if not measured:
        return None
    return not loss_reasons(s, tol_pct)


def fit_path_loss(points: Sequence[Tuple[float, float]]) -> Optional[Dict[str, float]]:
    """Least-squares log-distance model  RSSI(d) = rssi_1m - 10*n*log10(d)  over
    the (distance_m, dBm) points with d >= 1 m. Needs at least three distinct
    distances with a real spread, otherwise a fit would be noise."""
    pts = [(d, r) for d, r in points if d is not None and r is not None and d >= 1.0]
    if len({round(d, 3) for d, _ in pts}) < 3:
        return None
    xs = [10.0 * math.log10(d) for d, _ in pts]
    ys = [r for _, r in pts]
    n_pts = len(xs)
    mx, my = sum(xs) / n_pts, sum(ys) / n_pts
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-9:
        return None
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx     # = -n
    rssi_1m = my - slope * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (rssi_1m + slope * x)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0
    return {"rssi_1m": rssi_1m, "exponent": -slope, "r2": r2}


def estimate_distance(rssi_dbm: float, fit: Dict[str, float]) -> Optional[float]:
    """Distance implied by a signal reading under the fitted model."""
    if fit["exponent"] <= 0.1:
        return None
    return 10.0 ** ((fit["rssi_1m"] - rssi_dbm) / (10.0 * fit["exponent"]))


def usable_ranges(holds: Sequence[HoldStats], tol_pct: float = 0.0) -> Dict[str, Optional[float]]:
    """Three distances, each "furthest d such that EVERY hold up to d qualifies":
      loss_free_to  nothing lost and no video stall (the headline answer)
      good_to       verdict Good (a small loss is tolerated)
      marginal_to   no Poor verdict yet
    A hold with no loss data cannot extend the loss-free range."""
    out: Dict[str, Optional[float]] = {"loss_free_to": None, "good_to": None, "marginal_to": None}
    ordered = sorted((h for h in holds if not math.isnan(h.distance_m)), key=lambda h: h.distance_m)
    lf_ok = good_ok = marg_ok = True
    for h in ordered:
        if is_loss_free(h, tol_pct) is not True:
            lf_ok = False
        if h.verdict != "Good":
            good_ok = False
        if h.verdict == "Poor":
            marg_ok = False
        if lf_ok:
            out["loss_free_to"] = h.distance_m
        if good_ok:
            out["good_to"] = h.distance_m
        if marg_ok:
            out["marginal_to"] = h.distance_m
    return out


# ───────────────────────────── samplers (threads) ──────────────────────────

class PingSampler(threading.Thread):
    def __init__(self, target: str, store: Store, kind: str, size: int,
                 interval: float, stop: threading.Event):
        super().__init__(daemon=True)
        self.target, self.store, self.kind = target, store, kind
        self.size, self.interval, self._stop_ev = size, interval, stop
        self.proc: Optional[subprocess.Popen] = None

    def run(self) -> None:
        try:
            self.proc = subprocess.Popen(
                ["ping", "-n", "-i", str(self.interval), "-s", str(self.size), self.target],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        except OSError:
            return
        assert self.proc.stdout is not None
        try:
            for line in self.proc.stdout:
                if self._stop_ev.is_set():
                    break
                parsed = parse_ping_line(line)
                if parsed:
                    self.store.add(self.kind, seq=parsed[0], rtt=parsed[1])
        except (ValueError, OSError):
            pass                                   # pipe closed by stop() while we were reading
        self.stop()

    def stop(self) -> None:
        """End the ping process and release its pipe (no leaked process or file)."""
        proc = self.proc
        if proc is None:
            return
        if proc.poll() is None:
            proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
        if proc.stdout is not None and not proc.stdout.closed:
            try:
                proc.stdout.close()
            except OSError:
                pass


class WifiSampler(threading.Thread):
    def __init__(self, iface: Optional[str], store: Store, stop: threading.Event,
                 period: float = 1.0):
        super().__init__(daemon=True)
        self.iface, self.store, self._stop_ev, self.period = iface, store, stop, period

    def _run(self, *args: str) -> str:
        try:
            return subprocess.run(list(args), capture_output=True, text=True, timeout=3).stdout
        except (OSError, subprocess.SubprocessError):
            return ""

    def run(self) -> None:
        while not self._stop_ev.is_set():
            fields: Dict[str, object] = {}
            if self.iface:
                link = parse_iw_link(self._run("iw", "dev", self.iface, "link"))
                st = parse_iw_station(self._run("iw", "dev", self.iface, "station", "dump"))
                fields.update({k: v for k, v in st.items() if v is not None})
                if link.get("signal_dbm") is not None:
                    fields["signal_dbm"] = link["signal_dbm"]
                if link.get("tx_mbps") is not None and "tx_mbps" not in fields:
                    fields["tx_mbps"] = link["tx_mbps"]
                fields["ssid"] = link.get("ssid")
                if not fields.get("signal_dbm"):
                    try:
                        with open("/proc/net/wireless", encoding="utf-8") as fh:
                            fields.update(parse_proc_wireless(fh.read(), self.iface))
                    except OSError:
                        pass
            self.store.add("wifi", **fields)
            self._stop_ev.wait(self.period)


class VideoSampler(threading.Thread):
    """Reads the real MJPEG stream and counts frames, bytes, stalls, reconnects."""

    def __init__(self, url: str, store: Store, stop: threading.Event, timeout: float = 3.0):
        super().__init__(daemon=True)
        self.url, self.store, self._stop_ev, self.timeout = url, store, stop, timeout

    def run(self) -> None:
        while not self._stop_ev.is_set():
            counter = JpegFrameCounter()
            try:
                with urllib.request.urlopen(self.url, timeout=self.timeout) as resp:
                    self.store.add("video_event", event="connect")
                    read = getattr(resp, "read1", resp.read)
                    while not self._stop_ev.is_set():
                        chunk = read(16384)
                        if not chunk:
                            raise ConnectionError("stream ended")
                        t = time.monotonic()
                        self.store.add("video_bytes", t, n=len(chunk))
                        for size in counter.feed(chunk):
                            self.store.add("video_frame", t, size=size)
            except Exception as exc:                    # noqa: BLE001 - any failure = a disconnect
                self.store.add("video_event", event="disconnect", info=str(exc)[:80])
                self._stop_ev.wait(1.0)


class UdpProbe(threading.Thread):
    """Sends numbered datagrams at a fixed rate; the Radxa helper echoes each one
    back stamped with how many it has received so far, which separates uplink loss
    from downlink loss."""

    def __init__(self, host: str, port: int, mbps: float, size: int,
                 store: Store, stop: threading.Event):
        super().__init__(daemon=True)
        self.addr, self.size, self.store, self._stop_ev = (host, port), size, store, stop
        self.interval = size * 8 / (mbps * 1e6)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self._rx = threading.Thread(target=self._receive, daemon=True)

    def _receive(self) -> None:
        while not self._stop_ev.is_set():
            try:
                data, _ = self.sock.recvfrom(4096)
            except (socket.timeout, OSError):
                continue
            parsed = parse_packet(data)
            if parsed:
                seq, t_send, srv = parsed
                now = time.monotonic()
                self.store.add("udp_rx", now, seq=seq, rtt=(now - t_send) * 1000.0, srv=srv)

    def run(self) -> None:
        try:
            self._send_loop()
        finally:
            self._stop_ev.wait(0.3)                    # let the receiver see the stop flag first
            self.sock.close()

    def _send_loop(self) -> None:
        self._rx.start()
        seq, nxt = 0, time.perf_counter()
        while not self._stop_ev.is_set():
            now = time.monotonic()
            try:
                self.sock.sendto(make_packet(seq, now, self.size), self.addr)
                self.store.add("udp_tx", now, seq=seq)
            except OSError:
                pass
            seq += 1
            nxt += self.interval
            delay = nxt - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.perf_counter()


# ───────────────────────────── Radxa helper control ────────────────────────

def _ssh(target: str, command: str, timeout: float = 12.0) -> subprocess.CompletedProcess:
    return subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", target, command],
                          capture_output=True, text=True, timeout=timeout)


def explain_ssh_failure(stderr: str, target: str) -> str:
    """A plain-language reason for a failed ssh/scp, from its error text."""
    host = target.split("@")[-1]
    low = (stderr or "").lower()
    if "permission denied" in low or "publickey" in low:
        return (f"SSH login to {target} failed - no key is set up. Run once: ssh-copy-id {target}")
    if any(w in low for w in ("timed out", "no route", "unreachable", "refused", "could not resolve", "name or service")):
        return (f"cannot reach {host} over SSH - the Radxa is off, the IP is wrong, "
                "or this laptop is on a different Wi-Fi network")
    if "host key verification" in low:
        return f"SSH does not trust {host} yet - run once: ssh {target}  (and answer yes)"
    detail = (stderr or "").strip().splitlines()[-1][:90] if (stderr or "").strip() else "no details"
    return f"SSH to {target} failed ({detail})"


def start_helper_checked(target: str, port: int) -> Tuple[bool, str]:
    """Copy the probe server to /tmp on the Radxa and start it. Returns (ok, reason);
    the reason is "" on success and a plain sentence otherwise. It writes only to
    /tmp and exits by itself when idle."""
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.join(here, "link_probe_server.py")
    try:
        cp = subprocess.run(["scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", src,
                             f"{target}:/tmp/link_probe_server.py"], capture_output=True, text=True, timeout=20)
        if cp.returncode != 0:
            return False, explain_ssh_failure(cp.stderr, target)
        run = _ssh(target, f"nohup python3 /tmp/link_probe_server.py --port {port} --idle-exit 30 "
                           ">/tmp/link_probe.log 2>&1 & sleep 1; pgrep -f link_probe_server.py")
        if run.returncode == 0 and run.stdout.strip():
            return True, ""
        err = (run.stderr or "").strip()
        if err and "python3" in err.lower() and "not found" in err.lower():
            return False, "python3 is not installed on the Radxa"
        if run.returncode != 0 and err:
            return False, explain_ssh_failure(err, target)
        return False, ("the helper was copied but exited at once - see /tmp/link_probe.log on the Radxa "
                       f"(is UDP port {port} already in use?)")
    except subprocess.TimeoutExpired:
        return False, f"SSH to {target} timed out"
    except FileNotFoundError:
        return False, "ssh/scp is not installed on this laptop"
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f"helper start failed: {exc}"


def start_helper(target: str, port: int) -> bool:
    ok, why = start_helper_checked(target, port)
    if not ok:
        print(f"  could not start the helper: {why}")
    return ok


def stop_helper(target: str) -> None:
    try:
        _ssh(target, "pkill -f link_probe_server.py; rm -f /tmp/link_probe_server.py", timeout=8)
    except (OSError, subprocess.SubprocessError):
        pass


def radxa_wifi_signal(target: str) -> Optional[float]:
    """The Radxa's own Wi-Fi signal (control reading - it should not change while
    only the laptop moves). None if it is wired or unreachable."""
    try:
        out = _ssh(target, 'iw dev "$(iw dev | awk \'/Interface/{print $2; exit}\')" link').stdout
    except (OSError, subprocess.SubprocessError):
        return None
    return parse_iw_link(out).get("signal_dbm")  # type: ignore[return-value]


# ───────────────────────────── reporting ───────────────────────────────────

def _fmt(v, spec="{:.1f}", dash="–"):
    return dash if v is None or (isinstance(v, float) and math.isnan(v)) else spec.format(v)


TABLE_COLS = [
    ("Dist m", lambda h: _fmt(h.distance_m, "{:g}")),
    ("Signal dBm", lambda h: _fmt(h.signal_mean, "{:.0f}")),
    ("Retries/s", lambda h: _fmt(h.retries_per_s)),
    ("Ping loss %", lambda h: _fmt(h.ping_loss)),
    ("RTT p95 ms", lambda h: _fmt(h.ping_p95)),
    ("Video fps", lambda h: _fmt(h.video_fps)),
    ("Video Mbit/s", lambda h: _fmt(h.video_mbps)),
    ("Stall s", lambda h: _fmt(h.video_stall_max)),
    ("UDP up %", lambda h: _fmt(h.udp_up_loss)),
    ("UDP down %", lambda h: _fmt(h.udp_down_loss)),
    ("Loss-free", lambda h: {True: "Yes", False: "No"}.get(h.loss_free, "–")),
    ("Verdict", lambda h: h.verdict),
]


def text_table(holds: Sequence[HoldStats]) -> str:
    rows = [[c for c, _ in TABLE_COLS]] + [[f(h) for _, f in TABLE_COLS] for h in holds]
    widths = [max(len(r[i]) for r in rows) for i in range(len(TABLE_COLS))]
    lines = ["  ".join(c.ljust(w) for c, w in zip(r, widths)) for r in rows]
    lines.insert(1, "  ".join("-" * w for w in widths))
    return "\n".join(lines)


def headline(ranges: Dict[str, Optional[float]], holds: Sequence[HoldStats], tol_pct: float = 0.0) -> str:
    """The one-line answer: how far the live feed runs with nothing lost."""
    lf = ranges.get("loss_free_to")
    measured = [h for h in holds if not math.isnan(h.distance_m)]
    tol = "no packet loss and no video stall" if tol_pct <= 0 else f"loss within {tol_pct:g} % and no video stall"
    if not measured:
        return "No distances were measured."
    farthest = max(h.distance_m for h in measured)
    if lf is None:
        return "Loss was already present at the first mark - the live feed was never loss-free."
    if lf >= farthest:
        return (f"Live feed was loss-free ({tol}) at every distance tested, up to {lf:g} m. "
                "The true limit is further than you walked.")
    return f"Live feed is loss-free ({tol}) up to {lf:g} m."


def findings(holds: Sequence[HoldStats], fit: Optional[Dict[str, float]],
             ranges: Dict[str, Optional[float]], baseline_fps: Optional[float],
             tol_pct: float = 0.0) -> List[str]:
    out: List[str] = [headline(ranges, holds, tol_pct)]
    ordered = sorted((h for h in holds if not math.isnan(h.distance_m)), key=lambda h: h.distance_m)
    first_loss = next((h for h in ordered if is_loss_free(h, tol_pct) is False), None)
    if first_loss is not None:
        out.append(f"First loss at {first_loss.distance_m:g} m: " + ", ".join(loss_reasons(first_loss, tol_pct)) + ".")
    gt, mt = ranges.get("good_to"), ranges.get("marginal_to")
    farthest = ordered[-1].distance_m if ordered else None
    if gt is not None:
        out.append(f"Verdict stays Good (a little loss tolerated) up to {gt:g} m.")
    if mt is not None and farthest is not None:
        out.append(f"No Poor reading up to {mt:g} m"
                   + ("." if mt >= farthest else "; beyond that it turns Poor."))
    first_bad = next((h for h in ordered if h.verdict != "Good"), None)
    if first_bad:
        out.append(f"First non-Good reading at {first_bad.distance_m:g} m: " + ", ".join(first_bad.reasons) + ".")
    worst = max((h.ping_loss for h in holds if h.ping_loss is not None), default=None)
    if worst is not None:
        out.append(f"Worst ping loss seen: {worst:.1f} %.")
    udp = [v for h in holds for v in (h.udp_up_loss, h.udp_down_loss) if v is not None]
    if udp:
        out.append(f"Worst UDP packet loss seen: {max(udp):.1f} %.")
    if baseline_fps:
        out.append(f"Close-range baseline: {baseline_fps:.1f} video frames/s.")
    if fit:
        out.append(f"Path-loss fit: {fit['rssi_1m']:.0f} dBm at 1 m, exponent {fit['exponent']:.1f} "
                   f"(R² {fit['r2']:.2f}). Indoors expect 2–4; distance estimated from signal alone "
                   "is only a rough guide.")
    else:
        out.append("Not enough distances to fit a signal-vs-distance curve (needs 3 or more at 1 m or beyond).")
    return out


def svg_chart(title: str, series: Sequence[Tuple[str, str, Sequence[Tuple[float, float]]]],
              ylabel: str, hlines: Sequence[Tuple[float, str]] = (), w: int = 560, h: int = 240,
              xlabel: str = "distance (m)", max_xticks: Optional[int] = None) -> str:
    """A small self-contained line chart. series = (label, colour, [(x, y), ...])."""
    pts = [p for _, _, s in series for p in s if p[1] is not None and not math.isnan(p[1])]
    if not pts:
        return f"<p class='muted'>{html.escape(title)}: no data</p>"
    xs, ys = [p[0] for p in pts], [p[1] for p in pts] + [v for v, _ in hlines]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    if x1 - x0 < 1e-9:
        x1 = x0 + 1
    if y1 - y0 < 1e-9:
        y1 = y0 + 1
    pad = (y1 - y0) * 0.08
    lower_bound_zero = min(ys) >= 0          # a loss % or fps axis must not dip below 0
    y0, y1 = y0 - pad, y1 + pad
    if lower_bound_zero:
        y0 = max(0.0, y0)
    L, R, T, B = 52, 12, 22, 34

    def X(x): return L + (x - x0) / (x1 - x0) * (w - L - R)
    def Y(y): return T + (1 - (y - y0) / (y1 - y0)) * (h - T - B)

    g = [f"<svg viewBox='0 0 {w} {h}' xmlns='http://www.w3.org/2000/svg' class='chart'>",
         f"<text x='{L}' y='14' class='ct'>{html.escape(title)}</text>"]
    for i in range(5):
        yv = y0 + (y1 - y0) * i / 4
        g.append(f"<line x1='{L}' x2='{w - R}' y1='{Y(yv):.1f}' y2='{Y(yv):.1f}' class='grid'/>"
                 f"<text x='{L - 6}' y='{Y(yv) + 4:.1f}' class='ax' text-anchor='end'>{yv:.0f}</text>")
    xticks = sorted({round(p[0], 3) for p in pts})
    if max_xticks and len(xticks) > max_xticks:          # a time axis has hundreds of x values
        every = math.ceil(len(xticks) / max_xticks)
        xticks = xticks[::every]
    for xv in xticks:
        g.append(f"<text x='{X(xv):.1f}' y='{h - 16}' class='ax' text-anchor='middle'>{xv:g}</text>")
    g.append(f"<text x='{(L + w - R) / 2:.0f}' y='{h - 3}' class='ax' text-anchor='middle'>{html.escape(xlabel)}</text>")
    g.append(f"<text x='12' y='{(T + h - B) / 2:.0f}' class='ax' transform='rotate(-90 12 {(T + h - B) / 2:.0f})' "
             f"text-anchor='middle'>{html.escape(ylabel)}</text>")
    for value, label in hlines:
        g.append(f"<line x1='{L}' x2='{w - R}' y1='{Y(value):.1f}' y2='{Y(value):.1f}' class='limit'/>"
                 f"<text x='{w - R - 2}' y='{Y(value) - 3:.1f}' class='ax' text-anchor='end'>{html.escape(label)}</text>")
    for k, (label, colour, s) in enumerate(series):
        s = [(x, y) for x, y in s if y is not None and not math.isnan(y)]
        if not s:
            continue
        d = " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in sorted(s))
        g.append(f"<polyline fill='none' stroke='{colour}' stroke-width='2' points='{d}'/>")
        for x, y in s:
            g.append(f"<circle cx='{X(x):.1f}' cy='{Y(y):.1f}' r='3' fill='{colour}'/>")
        g.append(f"<text x='{L + 6 + k * 130}' y='{T + 12}' class='ax' style='fill:{colour}'>● {html.escape(label)}</text>")
    g.append("</svg>")
    return "".join(g)


VERDICT_CLASS = {"Good": "good", "Marginal": "marg", "Poor": "poor"}


def html_report(meta: Dict[str, object], holds: Sequence[HoldStats], notes: Sequence[str],
                fit: Optional[Dict[str, float]], ranges: Optional[Dict[str, Optional[float]]] = None) -> str:
    ordered = sorted((h for h in holds if not math.isnan(h.distance_m)), key=lambda h: h.distance_m)
    sig = [(h.distance_m, h.signal_mean) for h in ordered]
    series_sig = [("measured", "#58a6ff", sig)]
    if fit:
        series_sig.append(("fitted", "#8b949e", [(d, fit["rssi_1m"] - 10 * fit["exponent"] * math.log10(max(d, 1)))
                                                  for d in [h.distance_m for h in ordered] if d >= 1]))
    charts = [
        svg_chart("Signal at the laptop", series_sig, "dBm",
                  hlines=[(THRESHOLDS["good_rssi"], "good"), (THRESHOLDS["poor_rssi"], "poor")]),
        svg_chart("Camera feed", [("frames/s", "#58a6ff", [(h.distance_m, h.video_fps) for h in ordered]),
                                  ("Mbit/s", "#8b949e", [(h.distance_m, h.video_mbps) for h in ordered])], "fps / Mbit/s"),
        svg_chart("Data lost", [("ping %", "#d29922", [(h.distance_m, h.ping_loss) for h in ordered]),
                                ("UDP up %", "#8b949e", [(h.distance_m, h.udp_up_loss) for h in ordered]),
                                ("UDP down %", "#58a6ff", [(h.distance_m, h.udp_down_loss) for h in ordered])], "% lost",
                  hlines=[(THRESHOLDS["poor_loss_pct"], "poor")]),
        svg_chart("Round-trip time (p95)", [("ping ms", "#58a6ff", [(h.distance_m, h.ping_p95) for h in ordered])], "ms"),
    ]
    head = "".join(f"<th>{html.escape(c)}</th>" for c, _ in TABLE_COLS)
    rows = "".join(
        "<tr>" + "".join(
            f"<td class='{VERDICT_CLASS.get(h.verdict, '')}'>{html.escape(f(h))}</td>" if c == "Verdict"
            else f"<td>{html.escape(f(h))}</td>" for c, f in TABLE_COLS) + "</tr>"
        for h in ordered)
    reasons = "".join(f"<li><b>{h.distance_m:g} m</b> – {html.escape(', '.join(h.reasons))}</li>"
                      for h in ordered if h.reasons)
    meta_rows = "".join(f"<tr><th>{html.escape(str(k))}</th><td>{html.escape(str(v))}</td></tr>" for k, v in meta.items())
    lf = (ranges or {}).get("loss_free_to")
    big = ("–" if lf is None else f"{lf:g} m")
    hero = (f"<div class='hero'><div class='hl'>LOSS-FREE LIVE FEED UP TO</div>"
            f"<div class='hv'>{html.escape(big)}</div>"
            f"<div class='hs'>{html.escape(notes[0]) if notes else ''}</div></div>") if ranges is not None else ""
    return f"""<!doctype html><html><head><meta charset='utf-8'><title>Link range test</title>
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
<h1>LINK RANGE TEST</h1>
{hero}
<h2>Findings</h2><ul>{''.join(f'<li>{html.escape(n)}</li>' for n in notes)}</ul>
<h2>Per distance</h2><table><tr>{head}</tr>{rows}</table>
{('<h2>Why not Good</h2><ul>' + reasons + '</ul>') if reasons else ''}
<h2>Charts</h2><div class='charts'>{''.join(charts)}</div>
<h2>Run details</h2><table class='meta'>{meta_rows}</table>
<p class='muted'>Distance values are the marks you pressed. Signal-based distance is an estimate; indoors it is rough.</p>
</body></html>"""


def markdown_report(meta: Dict[str, object], holds: Sequence[HoldStats], notes: Sequence[str]) -> str:
    ordered = sorted((h for h in holds if not math.isnan(h.distance_m)), key=lambda h: h.distance_m)
    cols = [c for c, _ in TABLE_COLS]
    lines = ["# Link range test", "", "## Findings", ""] + [f"- {n}" for n in notes] + ["", "## Per distance", "",
             "| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    lines += ["| " + " | ".join(f(h) for _, f in TABLE_COLS) + " |" for h in ordered]
    lines += ["", "## Run details", ""] + [f"- **{k}**: {v}" for k, v in meta.items()]
    return "\n".join(lines) + "\n"


# ───────────────────────────── finish: analyse + write ─────────────────────

def finish(holds: List[HoldStats], baseline_fps: Optional[float], meta: Dict[str, object],
           out_dir: str, timeline: Optional[List[dict]] = None,
           loss_tolerance: float = 0.0) -> Dict[str, object]:
    for h in holds:
        h.video_drop = (None if not baseline_fps or h.video_fps is None
                        else max(0.0, (1 - h.video_fps / baseline_fps) * 100))
        h.verdict, h.reasons = verdict_for(h, baseline_fps)
        h.loss_free = is_loss_free(h, loss_tolerance)
    fit = fit_path_loss([(h.distance_m, h.signal_mean) for h in holds])
    ranges = usable_ranges(holds, loss_tolerance)
    notes = findings(holds, fit, ranges, baseline_fps, loss_tolerance)
    summary = {"ranges": ranges, "path_loss_fit": fit, "baseline_fps": baseline_fps,
               "loss_tolerance_pct": loss_tolerance,
               "thresholds": THRESHOLDS, "findings": notes, "meta": meta}
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "holds.json"), "w", encoding="utf-8") as fh:
        json.dump([asdict(h) for h in holds], fh, indent=2)
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2)
    with open(os.path.join(out_dir, "report.html"), "w", encoding="utf-8") as fh:
        fh.write(html_report(meta, holds, notes, fit, ranges))
    with open(os.path.join(out_dir, "report.md"), "w", encoding="utf-8") as fh:
        fh.write(markdown_report(meta, holds, notes))
    if timeline:
        with open(os.path.join(out_dir, "samples.csv"), "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(timeline[0].keys()))
            w.writeheader()
            w.writerows(timeline)
    return summary


def build_timeline(store: Store, t_start: float, t_end: float, cfg: Config,
                   marks: Sequence[Tuple[float, float, float]]) -> List[dict]:
    """1 Hz rows. `marks` = (t0, t1, distance) per hold; seconds outside a hold are
    labelled as walking, with no distance."""
    rows: List[dict] = []
    wall0 = time.time() - (time.monotonic() - t_start)
    t = t_start
    while t + 1.0 <= t_end + 1e-9:
        s = analyse_window(store, t, t + 1.0, cfg)
        mid = t + 0.5
        dist = next((d for a, b, d in marks if a <= mid <= b), None)
        rows.append({
            "t_s": round(t - t_start, 1), "wall": time.strftime("%H:%M:%S", time.localtime(wall0 + (t - t_start))),
            "phase": "hold" if dist is not None else "walk", "distance_m": "" if dist is None else dist,
            "signal_dbm": _fmt(s.signal_mean, "{:.0f}", ""), "tx_mbps": _fmt(s.tx_mbps, "{:.1f}", ""),
            "ping_loss_pct": _fmt(s.ping_loss, "{:.0f}", ""), "ping_rtt_ms": _fmt(s.ping_avg, "{:.1f}", ""),
            "video_fps": _fmt(s.video_fps, "{:.1f}", ""), "video_mbps": _fmt(s.video_mbps, "{:.2f}", ""),
            "udp_up_loss_pct": _fmt(s.udp_up_loss, "{:.0f}", ""), "udp_down_loss_pct": _fmt(s.udp_down_loss, "{:.0f}", ""),
        })
        t += 1.0
    return rows


# ───────────────────────────── demo data ───────────────────────────────────

def demo_holds(step: float = 5.0, count: int = 9, seed: int = 7) -> Tuple[List[HoldStats], float]:
    """Deterministic synthetic walk-away so the report can be reviewed without
    hardware: -45 dBm at 1 m, exponent 2.9, trouble from about 25 m."""
    import random
    rng = random.Random(seed)
    holds, base_fps = [], 24.0
    for i in range(count):
        d = i * step
        sig = -45.0 - 29.0 * math.log10(max(d, 1.0)) + rng.uniform(-1.5, 1.5)
        stress = max(0.0, (-sig - 66.0) / 14.0)                # 0 at -66 dBm, 1 at -80
        h = HoldStats(distance_m=d, signal_mean=sig, signal_min=sig - 3,
                      tx_mbps=max(6.0, 173.0 * (1 - stress * 0.9)), retries_per_s=2 + 60 * stress ** 2,
                      ping_loss=min(60.0, 22 * stress ** 2), ping_avg=3 + 40 * stress ** 2,
                      ping_p95=5 + 120 * stress ** 2, video_fps=base_fps * max(0.05, 1 - 0.8 * stress ** 1.5),
                      video_mbps=8.0 * max(0.05, 1 - 0.8 * stress ** 1.5),
                      video_stall_max=0.1 + 3.0 * stress ** 2, udp_up_loss=min(70.0, 14 * stress ** 2),
                      udp_down_loss=min(70.0, 20 * stress ** 2))
        holds.append(h)
    return holds, base_fps


# ───────────────────────────── session (shared by terminal and window) ─────

PREFS_PATH = os.path.join(os.path.expanduser("~"), ".link_range_test.json")


def load_prefs(path: Optional[str] = None) -> Dict[str, object]:
    try:
        with open(path or PREFS_PATH, encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_prefs(prefs: Dict[str, object], path: Optional[str] = None) -> None:
    try:
        with open(path or PREFS_PATH, "w", encoding="utf-8") as fh:
            json.dump(prefs, fh, indent=2)
    except OSError:
        pass


def gcs_saved_host() -> str:
    """The Radxa address the GCS last used - a better first guess than nothing."""
    try:
        home = os.environ.get("DRONE_GCS_HOME", os.path.join(os.path.expanduser("~"), ".drone_gcs"))
        with open(os.path.join(home, "settings.json"), encoding="utf-8") as fh:
            return str(json.load(fh).get("connection", {}).get("host", "") or "")
    except (OSError, ValueError, AttributeError):
        return ""


LABEL = r"(?!-)[A-Za-z0-9-]{1,63}(?<!-)"
HOSTNAME_RE = re.compile(rf"^{LABEL}(?:\.{LABEL})*$")
IPV4_RE = re.compile(r"^\d{1,3}(?:\.\d{1,3}){3}$")


def valid_host(text: str) -> bool:
    """A proper dotted IPv4 (four octets, each 0-255) or a plausible hostname.

    Anything made only of digits and dots must be a valid IPv4: "1.2.3" or
    "300.1.1.1" are typos, not hostnames."""
    t = text.strip()
    if not t:
        return False
    if re.fullmatch(r"[\d.]+", t):
        return bool(IPV4_RE.match(t)) and all(0 <= int(o) <= 255 for o in t.split("."))
    return bool(HOSTNAME_RE.match(t))


@dataclass
class SessionConfig:
    radxa: str
    ssh_user: str = "radxa"
    video_port: int = 8080
    video_url: str = ""
    use_video: bool = True
    use_udp: bool = True
    udp_port: int = DEFAULT_UDP_PORT
    udp_mbps: float = 1.0
    helper_target: str = ""          # user@host to start the UDP helper on ("" = do not)
    control_target: str = ""         # user@host to read the Radxa's own Wi-Fi from ("" = skip)
    step: float = 5.0
    hold: float = 15.0
    baseline_hold: float = 15.0
    ping_interval: float = 0.2
    loss_tolerance: float = 0.0
    iface: Optional[str] = None
    out_dir: str = ""
    mode: str = "holds"              # "holds" (marked distances) | "walk" (continuous, signal only)

    @property
    def ssh_target(self) -> str:
        return f"{self.ssh_user}@{self.radxa}"

    @property
    def url(self) -> str:
        return self.video_url or f"http://{self.radxa}:{self.video_port}/video"


class Session:
    """One range test: the sampler threads, the holds, and the final report.

    Both the terminal flow and the window drive this same object, so what is
    measured and reported cannot differ between them."""

    def __init__(self, cfg: SessionConfig, log=print):
        self.cfg, self.log = cfg, log
        self.iface = cfg.iface or detect_wifi_iface()
        self.store, self.stop_ev = Store(), threading.Event()
        self.analysis = Config(ping_interval=cfg.ping_interval, udp_enabled=cfg.use_udp,
                               video_enabled=cfg.use_video)
        self.threads: List[threading.Thread] = []
        self.holds: List[HoldStats] = []
        self.marks: List[Tuple[float, float, float]] = []
        self.helper: Optional[str] = None
        self.t_start = 0.0
        self.radxa_before: Optional[float] = None
        self.started = False
        # Why UDP loss is not being measured ("" = it is, or it was never requested).
        self.udp_off_reason = "" if cfg.use_udp else "not selected"

    def udp_summary(self) -> str:
        """For the report and the window: the probe rate, or "off" and why."""
        if self.analysis.udp_enabled:
            return f"{self.cfg.udp_mbps} Mbit/s"
        return f"off - {self.udp_off_reason}" if self.udp_off_reason else "off"

    # ── before the test ────────────────────────────────────────────────
    def preflight(self) -> Dict[str, object]:
        """Is the Radxa reachable, and which Wi-Fi network is this laptop on?"""
        reach = False
        try:
            reach = subprocess.run(["ping", "-c", "3", "-W", "2", self.cfg.radxa],
                                   capture_output=True, text=True, timeout=12).returncode == 0
        except (OSError, subprocess.SubprocessError):
            pass
        link: Dict[str, object] = {}
        if self.iface:
            try:
                out = subprocess.run(["iw", "dev", self.iface, "link"], capture_output=True,
                                     text=True, timeout=4).stdout
                link = parse_iw_link(out)
            except (OSError, subprocess.SubprocessError):
                pass
        return {"reachable": reach, "iface": self.iface, "ssid": link.get("ssid"),
                "signal_dbm": link.get("signal_dbm")}

    def start(self) -> None:
        c, st, stop = self.cfg, self.store, self.stop_ev
        self.threads = [WifiSampler(self.iface, st, stop),
                        PingSampler(c.radxa, st, "ping_s", 56, c.ping_interval, stop),
                        PingSampler(c.radxa, st, "ping_l", 1200, c.ping_interval, stop)]
        if c.use_video:
            self.threads.append(VideoSampler(c.url, st, stop))
        if c.use_udp:
            if c.helper_target:
                ok, why = start_helper_checked(c.helper_target, c.udp_port)
                if not ok:
                    self.udp_off_reason = why
                    self.log(f"  UDP helper did not start - UDP loss will not be measured: {why}")
                    self.analysis.udp_enabled = False
            if self.analysis.udp_enabled:
                self.threads.append(UdpProbe(c.radxa, c.udp_port, c.udp_mbps, 1200, st, stop))
                self.helper = c.helper_target or None
        if c.control_target:
            self.radxa_before = radxa_wifi_signal(c.control_target)
        self.t_start = time.monotonic()
        for th in self.threads:
            th.start()
        self.started = True
        time.sleep(1.5)                       # let the first samples land

    # ── holds ──────────────────────────────────────────────────────────
    def begin_hold(self) -> float:
        return time.monotonic()

    def end_hold(self, distance: float, t0: float) -> HoldStats:
        t1 = time.monotonic()
        h = analyse_window(self.store, t0, t1, self.analysis, distance)
        h.loss_free = is_loss_free(h, self.cfg.loss_tolerance)
        self.holds.append(h)
        self.marks.append((t0, t1, distance))
        return h

    def live(self, window: float = 5.0) -> HoldStats:
        """Statistics over the last few seconds, for a live display."""
        now = time.monotonic()
        return analyse_window(self.store, max(self.t_start, now - window), now, self.analysis)

    def last_signal(self) -> Optional[float]:
        w = self.store.last_at_or_before("wifi", time.monotonic())
        return w[1].get("signal_dbm") if w else None

    def running_ranges(self) -> Dict[str, Optional[float]]:
        """Ranges so far (verdicts are filled in so good_to / marginal_to are real)."""
        base = self.baseline_fps()
        for h in self.holds:
            h.verdict, h.reasons = verdict_for(h, base)
            h.loss_free = is_loss_free(h, self.cfg.loss_tolerance)
        return usable_ranges(self.holds, self.cfg.loss_tolerance)

    def baseline_fps(self) -> Optional[float]:
        return next((h.video_fps for h in self.holds if h.distance_m == 0.0), None)

    # ── after the test ─────────────────────────────────────────────────
    def stop(self) -> None:
        self.t_end = time.monotonic()
        self.stop_ev.set()
        for th in self.threads:
            if isinstance(th, PingSampler):
                th.stop()
        if self.helper:
            stop_helper(self.helper)
            self.helper = None

    def finish(self) -> Optional[Dict[str, object]]:
        c = self.cfg
        if not self.holds:
            return None
        out_dir = c.out_dir or os.path.join(os.path.expanduser("~"), "link_range_results",
                                            time.strftime("%Y%m%d_%H%M%S"))
        c.out_dir = out_dir
        radxa_after = radxa_wifi_signal(c.control_target) if c.control_target else None
        meta = {"started": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), "radxa": c.radxa,
                "wifi_iface": self.iface, "video_url": c.url if c.use_video else "off",
                "step_m": c.step, "hold_s": c.hold,
                "udp": self.udp_summary(),
                "allowed loss %": c.loss_tolerance,
                "radxa_wifi_signal_dbm (before/after)": f"{self.radxa_before} / {radxa_after}"}
        timeline = build_timeline(self.store, self.t_start, getattr(self, "t_end", time.monotonic()),
                                  self.analysis, self.marks)
        return finish(self.holds, self.baseline_fps(), meta, out_dir, timeline, c.loss_tolerance)


# ───────────────────────────── command line ────────────────────────────────

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Walk-away range test for the camera-feed Wi-Fi link. "
                    "Run with no arguments to get a window that asks for the Radxa IP.")
    p.add_argument("--radxa", help="Radxa IP (ping target and default video/UDP host)")
    p.add_argument("--video-url", help="MJPEG URL (default http://<radxa>:8080/video)")
    p.add_argument("--no-video", action="store_true", help="skip the camera-stream measurement")
    p.add_argument("--iface", help="Wi-Fi interface (auto-detected)")
    p.add_argument("--step", type=float, default=5.0, help="metres between marks (default 5)")
    p.add_argument("--hold", type=float, default=15.0, help="seconds to stand still at each mark")
    p.add_argument("--baseline-hold", type=float, default=15.0, help="seconds at 0 m")
    p.add_argument("--ping-interval", type=float, default=0.2)
    p.add_argument("--loss-tolerance", type=float, default=0.0,
                   help="percent of loss still counted as loss-free (default 0 = none at all)")
    p.add_argument("--udp", action="store_true", help="UDP packet-loss probe (needs the Radxa helper)")
    p.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    p.add_argument("--udp-mbps", type=float, default=1.0, help="probe rate; keep low so it does not disturb the video")
    p.add_argument("--start-helper", metavar="USER@HOST", help="copy+start link_probe_server.py on the Radxa over SSH")
    p.add_argument("--control-ssh", metavar="USER@HOST", help="read the Radxa's own Wi-Fi signal as a control")
    p.add_argument("--auto-marks", help="comma list of distances, no keypress (e.g. 0,5,10); waits --walk-time between")
    p.add_argument("--walk-time", type=float, default=20.0, help="seconds between marks with --auto-marks")
    p.add_argument("--walk", action="store_true",
                   help="continuous walk: stream values every second, verdict in seconds and dBm (no distance)")
    p.add_argument("--walk-seconds", type=float, default=0.0,
                   help="with --walk: stop by itself after this many seconds (default: press Enter)")
    p.add_argument("--auto", action="store_true", help="with --walk: start at once, without waiting for Enter")
    p.add_argument("--out", help="output folder")
    p.add_argument("--force", action="store_true", help="continue even if the Radxa does not answer ping")
    p.add_argument("--demo", action="store_true", help="synthetic data -> report only (no network)")
    p.add_argument("--gui", action="store_true", help="open the window (default when run with no arguments)")
    p.add_argument("--cli", action="store_true", help="terminal only, never open a window")
    return p.parse_args(argv)


def config_from_args(args: argparse.Namespace) -> SessionConfig:
    return SessionConfig(
        radxa=args.radxa, video_url=args.video_url or "", use_video=not args.no_video,
        use_udp=args.udp, udp_port=args.udp_port, udp_mbps=args.udp_mbps,
        helper_target=args.start_helper or "", control_target=args.control_ssh or "",
        step=args.step, hold=args.hold, baseline_hold=args.baseline_hold,
        ping_interval=args.ping_interval, loss_tolerance=args.loss_tolerance,
        iface=args.iface, out_dir=args.out or "", mode="walk" if getattr(args, "walk", False) else "holds")


def countdown(label: str, seconds: float, session: "Session") -> None:
    if not sys.stdout.isatty():                       # piped / logged: no \r animation
        print(f"  {label}: holding {seconds:g} s ...", flush=True)
        time.sleep(seconds)
        return
    end = time.monotonic() + seconds
    while True:
        left = end - time.monotonic()
        if left <= 0:
            break
        print(f"\r  {label}: hold still {left:4.0f} s   signal {_fmt(session.last_signal(), '{:.0f}')} dBm   ",
              end="", flush=True)
        time.sleep(min(0.5, max(0.05, left)))
    print("\r" + " " * 70 + "\r", end="")


def print_result(summary: Dict[str, object], holds: Sequence[HoldStats], out_dir: str) -> None:
    print("\n" + text_table(holds))
    print("\n" + "\n".join("• " + n for n in summary["findings"]))
    print(f"\nReport: {os.path.join(out_dir, 'report.html')}")


def run(args: argparse.Namespace) -> int:
    """Terminal flow (all options on the command line)."""
    out_dir = args.out or os.path.join(os.path.expanduser("~"), "link_range_results",
                                       time.strftime("%Y%m%d_%H%M%S"))
    if args.demo and getattr(args, "walk", False):
        import link_range_walk
        return link_range_walk.run_demo(args, out_dir)
    if args.demo:
        holds, base = demo_holds(args.step)
        summary = finish(holds, base, {"mode": "DEMO (synthetic data - not a measurement)", "step_m": args.step},
                         out_dir, None, args.loss_tolerance)
        print_result(summary, holds, out_dir)
        return 0
    if not args.radxa:
        print("error: --radxa <ip> is required (or run with no arguments for the window, or use --demo)",
              file=sys.stderr)
        return 2

    sess = Session(config_from_args(args))
    print(f"Wi-Fi interface: {sess.iface or 'unknown (no signal readings)'}")
    pre = sess.preflight()
    if not pre["reachable"]:
        print(f"The Radxa ({args.radxa}) does not answer ping from this laptop.")
        print(f"  This laptop is on Wi-Fi network: {pre['ssid'] or 'unknown'} - is that the same network as the Radxa?")
        if not args.force:
            print("  Fix the network, or re-run with --force to measure anyway.")
            return 2
    sess.start()
    if getattr(args, "walk", False):
        import link_range_walk
        return link_range_walk.run_walk(args, sess)

    def do_hold(dist: float, secs: float) -> None:
        t0 = sess.begin_hold()
        countdown(f"{dist:g} m", secs, sess)
        h = sess.end_hold(dist, t0)
        print(f"  {dist:g} m: signal {_fmt(h.signal_mean, '{:.0f}')} dBm, ping loss {_fmt(h.ping_loss)} %, "
              f"video {_fmt(h.video_fps)} fps, UDP loss up/down {_fmt(h.udp_up_loss)}/{_fmt(h.udp_down_loss)} %")

    try:
        print("\nStand next to the Radxa/router (0 m).")
        if not args.auto_marks:
            input("Press Enter to measure the baseline... ")
        do_hold(0.0, args.baseline_hold)
        if args.auto_marks:
            for d in [float(x) for x in args.auto_marks.split(",") if float(x) > 0]:
                print(f"Walk to {d:g} m ({args.walk_time:g} s)...")
                time.sleep(args.walk_time)
                do_hold(d, args.hold)
        else:
            last = 0.0
            while True:
                try:
                    ans = input(f"\nWalk to the next mark. Enter = {last + args.step:g} m, a number = that distance, q = finish: ").strip()
                except EOFError:
                    break
                if ans.lower() in ("q", "quit", "done"):
                    break
                try:
                    last = float(ans) if ans else last + args.step
                except ValueError:
                    print("  not a number")
                    continue
                do_hold(last, args.hold)
    except KeyboardInterrupt:
        print("\nInterrupted - writing what was measured.")
    finally:
        sess.stop()

    summary = sess.finish()
    if summary is None:
        print("No holds were recorded - nothing to report.")
        return 1
    print_result(summary, sess.holds, sess.cfg.out_dir)
    return 0


def gui_available() -> bool:
    if os.name != "nt" and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return False
    try:
        import tkinter  # noqa: F401
        return True
    except Exception:                                  # noqa: BLE001 - tk not installed
        return False


def main(argv: Optional[Sequence[str]] = None) -> int:
    raw = list(sys.argv[1:] if argv is None else argv)
    args = parse_args(raw)
    if args.demo or args.cli:
        return run(args)
    if not raw or args.gui:
        if gui_available():
            try:
                import link_range_gui
            except ImportError as exc:
                print(f"window unavailable ({exc}); falling back to the terminal")
            else:
                return link_range_gui.launch(args)
        else:
            print("No display / tkinter available - using the terminal instead.")
        if not args.radxa:
            try:
                args.radxa = input("Radxa IP address: ").strip()
            except EOFError:
                args.radxa = ""
            if not valid_host(args.radxa):
                print("That is not a valid IP address or hostname.", file=sys.stderr)
                return 2
            args.udp = True
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
