"""
================================================================================
MODULE: radxa_status.py
PURPOSE: Read and interpret the Radxa watchdog's status report (no Qt)
================================================================================

The watchdog on the Radxa (scripts/radxa/radxa_watchdog.py) serves a small JSON
status on its own port. This module fetches it and turns it into the few facts
the station needs: is the pipeline up, did it have to be restarted lately, is
the disk filling. A Radxa that is not running the watchdog simply returns
None - that is "not reporting", never "healthy".
================================================================================
"""

from __future__ import annotations

import json
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

DEFAULT_PORT = 8081
RECENT_RESTART_S = 300.0        # a restart this recent still counts as "just restarted"
DISK_LOW_PCT = 10.0             # free space below this raises a warning


@dataclass
class RadxaStatus:
    state: str = "unknown"                    # ok | degraded | stopped | down
    gave_up: bool = False
    pipeline_active: bool = False
    video_ok: bool = False
    router_active: bool = False
    disk_free_pct: Optional[float] = None
    disk_free_gb: Optional[float] = None
    restarts: List[Dict[str, Any]] = field(default_factory=list)   # [{"time": epoch, "reason": str}]
    messages: List[str] = field(default_factory=list)
    report_time: float = 0.0

    def recent_restart(self, now: float, window_s: float = RECENT_RESTART_S) -> Optional[Dict[str, Any]]:
        last = max(self.restarts, key=lambda r: r.get("time", 0.0), default=None)
        if last and now - float(last.get("time", 0.0)) <= window_s:
            return last
        return None

    def disk_low(self) -> bool:
        return self.disk_free_pct is not None and self.disk_free_pct < DISK_LOW_PCT

    def detail(self) -> str:
        """One line for the checklist."""
        if self.state == "ok":
            return ""
        bits = list(self.messages[:2])
        if self.state == "stopped":
            return "pipeline is stopped"
        if not self.pipeline_active:
            bits.append("pipeline not running")
        elif not self.video_ok:
            bits.append("video not being served")
        if not self.router_active:
            bits.append("mavlink-router not running")
        if self.gave_up:
            bits.append("watchdog gave up restarting it")
        return "; ".join(dict.fromkeys(bits)) or f"state {self.state}"


def parse_status(data: Any) -> Optional[RadxaStatus]:
    """RadxaStatus from the decoded JSON, or None if it is not a status report."""
    if not isinstance(data, dict) or "state" not in data:
        return None
    pipe = data.get("pipeline") or {}
    disk = data.get("disk") or {}
    restarts = [r for r in (data.get("restarts") or []) if isinstance(r, dict)]
    free_pct = disk.get("free_pct")
    if free_pct is None and disk.get("used_pct") is not None:
        free_pct = 100.0 - float(disk["used_pct"])
    return RadxaStatus(
        state=str(data.get("state", "unknown")), gave_up=bool(data.get("gave_up", False)),
        pipeline_active=bool(pipe.get("active", False)),
        video_ok=bool((data.get("video") or {}).get("ok", False)),
        router_active=bool((data.get("router") or {}).get("active", False)),
        disk_free_pct=None if free_pct is None else float(free_pct),
        disk_free_gb=None if disk.get("free_gb") is None else float(disk["free_gb"]),
        restarts=restarts, messages=[str(m) for m in (data.get("messages") or [])],
        report_time=float(data.get("time", 0.0)))


def fetch_status(host: str, port: int = DEFAULT_PORT, timeout: float = 2.0) -> Optional[RadxaStatus]:
    """GET http://host:port/status. None on any failure (not installed, unreachable,
    bad reply) - callers treat that as 'not reporting'."""
    try:
        with urllib.request.urlopen(f"http://{host}:{port}/status", timeout=timeout) as resp:
            return parse_status(json.loads(resp.read(65536).decode("utf-8", "replace")))
    except (OSError, ValueError):
        return None
