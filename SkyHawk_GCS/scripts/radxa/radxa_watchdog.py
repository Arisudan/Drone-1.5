#!/usr/bin/env python3
"""
================================================================================
SCRIPT: radxa_watchdog.py
PURPOSE: Keeps the Radxa's camera / SLAM pipeline alive and tells the GCS about it
================================================================================

Runs on the Radxa as a systemd service (deploy/drone-watchdog.service). Standard
library only. Every few seconds it checks:

  * the pipeline - is drone-pipeline.service running, and is the video streamer
    really serving frames on port 8080 (a process can be alive and stuck);
  * mavlink-router - is its service running;
  * the disk - how much is free.

WHAT IT DOES ABOUT A PROBLEM
  * Video not served for VIDEO_FAIL_RESTART_S while the pipeline claims to be up:
    restart the pipeline.
  * The pipeline unit has FAILED (crashed past its own restart limit): reset it
    and start it again.
  * mavlink-router not running: start it.
  It never starts a pipeline that was stopped on purpose (`systemctl stop`), and
  it gives up after MAX_RESTARTS inside RESTART_WINDOW_S rather than restarting
  in a loop - then it reports state "down" and says so.

WHAT IT TELLS THE GCS
  GET http://<radxa>:8081/status returns JSON (state ok / degraded / stopped / down, whether
  it gave up, the recent restarts and why, disk space). The GCS turns that into
  alarms and a checklist row. Nothing else is exposed; the port is read-only.

  The decision logic is the Watchdog class, with the clock and every check
  injected, so tests can drive it through time without touching systemd.
================================================================================
"""

from __future__ import annotations

import argparse
import json
import shutil
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Dict, List, Optional

PIPELINE_UNIT = "drone-pipeline.service"
ROUTER_UNIT = "mavlink-router.service"

POLL_S = 5.0
START_GRACE_S = 90.0          # a freshly started pipeline gets this long to bring video up
VIDEO_FAIL_RESTART_S = 20.0   # video not served this long (after the grace) -> restart
MAX_RESTARTS = 3              # ...at most this many restarts
RESTART_WINDOW_S = 600.0      # ...inside this window, then give up
KEEP_RESTARTS = 20            # how many restarts the status remembers


class Watchdog:
    """Decides, from the readings of one tick, what the state is and what to do."""

    def __init__(self, clock: Callable[[], float] = time.time,
                 restart_pipeline: Callable[[bool], bool] = lambda reset: False,
                 start_router: Callable[[], bool] = lambda: False):
        self.clock = clock
        self._restart_pipeline = restart_pipeline
        self._start_router = start_router
        self.restarts: List[Dict[str, object]] = []      # [{"time", "reason", "ok"}]
        self.gave_up = False
        self.video_bad_since: Optional[float] = None
        self.grace_until = 0.0
        self.router_attempt_at = 0.0
        self.status: Dict[str, object] = {"version": 1, "state": "unknown"}

    def _recent_restarts(self, now: float) -> int:
        return sum(1 for r in self.restarts if now - float(r["time"]) <= RESTART_WINDOW_S)

    def _do_restart(self, now: float, reason: str, reset_failed: bool) -> None:
        if self._recent_restarts(now) >= MAX_RESTARTS:
            self.gave_up = True
            return
        ok = self._restart_pipeline(reset_failed)
        self.restarts.append({"time": now, "reason": reason, "ok": bool(ok)})
        del self.restarts[:-KEEP_RESTARTS]
        self.video_bad_since = None
        self.grace_until = now + START_GRACE_S

    def tick(self, *, pipeline_state: str, video_ok: bool, router_active: bool,
             disk_free_gb: Optional[float] = None, disk_free_pct: Optional[float] = None,
             active_for_s: Optional[float] = None) -> Dict[str, object]:
        """pipeline_state is systemd's ActiveState: active | activating | failed | inactive | ...
        active_for_s is how long the unit has been active (None if unknown): a pipeline
        that has only just started - at boot or by hand - gets START_GRACE_S to bring video
        up, exactly like one the watchdog restarted itself."""
        now = self.clock()
        messages: List[str] = []
        pipeline_up = pipeline_state in ("active", "activating")
        alive = pipeline_up or video_ok            # a manually started pipeline counts as alive
        in_grace = now < self.grace_until or (
            pipeline_state == "active" and active_for_s is not None and active_for_s < START_GRACE_S)

        if video_ok:
            self.video_bad_since = None
            self.gave_up = False                    # healthy again: forget the earlier give-up
        elif pipeline_state == "active" and not in_grace:
            if self.video_bad_since is None:
                self.video_bad_since = now
            elif now - self.video_bad_since >= VIDEO_FAIL_RESTART_S:
                self._do_restart(now, f"video not served for {VIDEO_FAIL_RESTART_S:.0f} s", False)
        else:
            self.video_bad_since = None

        if pipeline_state == "failed":
            self._do_restart(now, "pipeline unit had failed", True)
            messages.append("pipeline unit failed - restarting it")
        elif pipeline_state in ("inactive", "dead") and not video_ok:
            messages.append("pipeline is stopped (not restarting it: it may have been stopped on purpose)")

        if not router_active and now - self.router_attempt_at >= 300.0:
            self.router_attempt_at = now
            if self._start_router():
                messages.append("mavlink-router was not running - started it")
            else:
                messages.append("mavlink-router is not running and could not be started")
        elif not router_active:
            messages.append("mavlink-router is not running")

        if self.gave_up:
            messages.append(f"gave up: {MAX_RESTARTS} restarts in {RESTART_WINDOW_S / 60:.0f} min did not fix it")
        if in_grace and not video_ok:
            messages.append("pipeline is starting")

        if self.gave_up or pipeline_state == "failed":
            state = "down"
        elif not alive:
            # Stopped on purpose is not a fault the watchdog should shout about;
            # anything else (unknown unit state) is.
            state = "stopped" if pipeline_state in ("inactive", "dead") else "down"
        elif not video_ok or not router_active:
            state = "degraded"                      # includes "still starting"
        else:
            state = "ok"

        self.status = {
            "version": 1, "time": now, "state": state, "gave_up": self.gave_up,
            "pipeline": {"unit": PIPELINE_UNIT, "state": pipeline_state, "active": pipeline_up},
            "video": {"ok": bool(video_ok)}, "router": {"active": bool(router_active)},
            "disk": {"free_gb": disk_free_gb, "free_pct": disk_free_pct},
            "restarts": list(self.restarts), "messages": messages,
        }
        return self.status


# ───────────────────────────── real checks ─────────────────────────────────

def _run(cmd: List[str], timeout: float = 8.0) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def unit_state(unit: str) -> str:
    try:
        return _run(["systemctl", "show", "-p", "ActiveState", "--value", unit], 5.0).stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def unit_active_for_s(unit: str) -> Optional[float]:
    """Seconds since the unit became active, or None if unknown / not active."""
    try:
        us = _run(["systemctl", "show", "-p", "ActiveEnterTimestampMonotonic", "--value", unit], 5.0).stdout.strip()
        with open("/proc/uptime", encoding="utf-8") as fh:
            uptime = float(fh.read().split()[0])
        started = int(us) / 1e6
        return max(0.0, uptime - started) if started > 0 else None
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def video_served(port: int = 8080, timeout: float = 3.0) -> bool:
    """True if the MJPEG streamer answers and sends at least one whole JPEG."""
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=timeout) as s:
            s.settimeout(timeout)
            s.sendall(b"GET /video HTTP/1.0\r\nHost: localhost\r\n\r\n")
            buf = b""
            end = time.monotonic() + timeout
            while time.monotonic() < end and len(buf) < 400_000:
                chunk = s.recv(65536)
                if not chunk:
                    break
                buf += chunk
                a = buf.find(b"\xff\xd8")
                if a >= 0 and buf.find(b"\xff\xd9", a + 2) >= 0:
                    return True
    except OSError:
        pass
    return False


def disk_numbers(path: str = "/"):
    u = shutil.disk_usage(path)
    return round(u.free / 1e9, 1), round(100.0 * u.free / u.total, 1)


def restart_pipeline(reset_failed: bool) -> bool:
    try:
        if reset_failed:
            _run(["sudo", "-n", "systemctl", "reset-failed", PIPELINE_UNIT])
        return _run(["sudo", "-n", "systemctl", "restart", PIPELINE_UNIT], 30.0).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def start_router() -> bool:
    try:
        return _run(["sudo", "-n", "systemctl", "start", ROUTER_UNIT], 15.0).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


# ───────────────────────────── status server ───────────────────────────────

def make_handler(get_status: Callable[[], Dict[str, object]]):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):                                   # noqa: N802
            if self.path.split("?")[0] not in ("/status", "/"):
                self.send_response(404)
                self.end_headers()
                return
            body = json.dumps(get_status()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):                          # quiet
            pass
    return Handler


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Radxa pipeline watchdog")
    ap.add_argument("--port", type=int, default=8081, help="status port (default 8081)")
    ap.add_argument("--video-port", type=int, default=8080)
    ap.add_argument("--once", action="store_true", help="one check, print the status, exit (no restarts)")
    args = ap.parse_args(argv)

    if args.once:
        wd = Watchdog(restart_pipeline=lambda r: False, start_router=lambda: False)
        free_gb, free_pct = disk_numbers()
        st = wd.tick(pipeline_state=unit_state(PIPELINE_UNIT), video_ok=video_served(args.video_port),
                     router_active=unit_state(ROUTER_UNIT) == "active", disk_free_gb=free_gb, disk_free_pct=free_pct,
                     active_for_s=unit_active_for_s(PIPELINE_UNIT))
        print(json.dumps(st, indent=2))
        return 0

    wd = Watchdog(restart_pipeline=restart_pipeline, start_router=start_router)
    lock = threading.Lock()

    def get_status():
        with lock:
            return dict(wd.status)

    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(get_status))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"watchdog: status on :{args.port}/status, checking every {POLL_S:g} s", flush=True)
    last_line, last_logged = "", 0.0
    while True:
        try:
            free_gb, free_pct = disk_numbers()
            st = wd.tick(pipeline_state=unit_state(PIPELINE_UNIT), video_ok=video_served(args.video_port),
                         router_active=unit_state(ROUTER_UNIT) == "active",
                         disk_free_gb=free_gb, disk_free_pct=free_pct,
                         active_for_s=unit_active_for_s(PIPELINE_UNIT))
            with lock:
                wd.status = st
            # Log a change straight away, a steady problem only every 5 minutes, so the
            # journal is not one line per check.
            line = f"{st['state']}: {'; '.join(st['messages']) or 'see status'}"
            if st["state"] != "ok" and (line != last_line or time.time() - last_logged >= 300):
                print(f"watchdog: {line}", flush=True)
                last_line, last_logged = line, time.time()
            elif st["state"] == "ok" and last_line != "ok":
                print("watchdog: ok", flush=True)
                last_line, last_logged = "ok", time.time()
        except Exception as exc:                            # noqa: BLE001 - never let the watchdog die
            print(f"watchdog: check failed: {exc}", flush=True)
        time.sleep(POLL_S)


if __name__ == "__main__":
    raise SystemExit(main())
