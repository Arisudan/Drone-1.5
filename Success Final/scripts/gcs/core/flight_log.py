"""
================================================================================
MODULE: flight_log.py
PURPOSE: Persistent Flight Session Recorder (arm -> disarm)
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Core Data Layer)
  * Upstream:      TelemetrySnapshot, sampled by the GUI tick
  * Downstream:    ui/logs_tab.py (history table, statistics, CSV export)

WHY THIS EXISTS:
  This ground station has flown entirely without a flight record: once the app
  closed, how long the vehicle was armed, how far it moved and what mode it was
  in were gone. A log tab with nothing behind it would be a picture of a
  feature, so the tab is backed by this: one record per armed session, written
  the moment the vehicle disarms.

FILE FORMAT:
  JSON Lines at ``$DRONE_GCS_HOME/flights.jsonl`` (default ``~/.drone_gcs``).
  One self-describing object per line: append-only, survives a crash mid-flight
  (only the unfinished session is lost, never the file), and readable by any
  tool without this codebase. A single JSON array would have to be rewritten
  whole on every landing and would truncate if the process died doing it.

WHAT COUNTS AS A FLIGHT:
  Armed to disarmed. Not takeoff to landing: the vehicle can be armed on the
  bench without ever leaving the ground, and those sessions are worth keeping -
  they are most of this project's test history. ``max_altitude_m`` tells the
  two apart after the fact.
================================================================================
"""

from __future__ import annotations

import json
import logging
import math
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

log = logging.getLogger("gcs.flightlog")


def log_dir() -> Path:
    """Where records live. Overridable for tests via DRONE_GCS_HOME."""
    return Path(os.environ.get("DRONE_GCS_HOME", str(Path.home() / ".drone_gcs")))


@dataclass
class FlightRecord:
    """One armed session."""

    started_utc: str = ""
    started_epoch: float = 0.0
    duration_s: float = 0.0
    distance_m: float = 0.0
    max_altitude_m: float = 0.0
    max_speed_ms: float = 0.0
    battery_start_pct: int = 0
    battery_end_pct: int = 0
    modes: List[str] = field(default_factory=list)
    status: str = "COMPLETED"
    forced_arm: bool = False

    @property
    def battery_used_pct(self) -> int:
        used = self.battery_start_pct - self.battery_end_pct
        return used if used > 0 else 0

    @property
    def mode_summary(self) -> str:
        return " > ".join(self.modes) if self.modes else "--"

    def duration_hms(self) -> str:
        m, s = divmod(int(self.duration_s), 60)
        return f"{m:02d}:{s:02d}"


class FlightLogger:
    """Watches arm state and writes a record per session.

    Feed it every telemetry snapshot; it decides when a session starts and
    ends. Keeping that decision here rather than in the GUI means the same
    logic applies whether the samples come from a live link or a replay.
    """

    def __init__(self, path: Optional[Path] = None):
        self.path = Path(path) if path else (log_dir() / "flights.jsonl")
        self.active: Optional[FlightRecord] = None
        self._last_xyz: Optional[tuple] = None
        self._was_armed = False

    # ── recording ───────────────────────────────────────────────────

    def update(self, t, forced: bool = False) -> Optional[FlightRecord]:
        """Sample one telemetry snapshot. Returns a record when one closes."""
        armed = bool(getattr(t, "armed", False))
        closed = None

        if armed and not self._was_armed:
            self._begin(t, forced)
        elif not armed and self._was_armed:
            closed = self._end(t)
        elif armed and self.active is not None:
            self._accumulate(t)

        self._was_armed = armed
        return closed

    def _begin(self, t, forced: bool) -> None:
        self.active = FlightRecord(
            started_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            started_epoch=time.time(),
            battery_start_pct=int(getattr(t, "battery_percent", 0) or 0),
            forced_arm=forced,
        )
        self._last_xyz = (t.x, t.y, t.z)
        self._note_mode(t)
        log.info("flight session started")

    def _accumulate(self, t) -> None:
        rec = self.active
        if rec is None:
            return
        if self._last_xyz is not None:
            dx = t.x - self._last_xyz[0]
            dy = t.y - self._last_xyz[1]
            dz = t.z - self._last_xyz[2]
            step = math.sqrt(dx * dx + dy * dy + dz * dz)
            # Discard sub-centimetre steps: at 30 Hz, position noise alone
            # would otherwise accumulate kilometres over a stationary hover.
            if step > 0.01:
                rec.distance_m += step
                self._last_xyz = (t.x, t.y, t.z)
        rec.max_altitude_m = max(rec.max_altitude_m, float(getattr(t, "altitude", 0.0)))
        rec.max_speed_ms = max(rec.max_speed_ms, float(getattr(t, "ground_speed", 0.0)))
        self._note_mode(t)

    def _note_mode(self, t) -> None:
        mode = getattr(t, "flight_mode", "") or ""
        rec = self.active
        if rec is not None and mode and (not rec.modes or rec.modes[-1] != mode):
            rec.modes.append(mode)

    def _end(self, t) -> Optional[FlightRecord]:
        rec = self.active
        self.active = None
        self._last_xyz = None
        if rec is None:
            return None
        rec.duration_s = max(0.0, time.time() - rec.started_epoch)
        rec.battery_end_pct = int(getattr(t, "battery_percent", 0) or 0)
        self.append(rec)
        log.info("flight session ended: %.1fs, %.2f m", rec.duration_s, rec.distance_m)
        return rec

    # ── persistence ─────────────────────────────────────────────────

    def append(self, rec: FlightRecord) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(asdict(rec)) + "\n")
        except OSError:
            # A full or read-only disk must not take the GCS down mid-flight.
            log.exception("could not append flight record")

    def load_all(self) -> List[FlightRecord]:
        """Newest first. A corrupt line is skipped, never fatal."""
        out: List[FlightRecord] = []
        if not self.path.exists():
            return out
        try:
            with open(self.path, encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        out.append(FlightRecord(**json.loads(line)))
                    except (ValueError, TypeError):
                        continue
        except OSError:
            log.exception("could not read flight log")
        out.sort(key=lambda r: r.started_epoch, reverse=True)
        return out


def summarise(records: List[FlightRecord]) -> Dict[str, str]:
    """Headline figures for the stat cards."""
    if not records:
        return {"flights": "0", "total_time": "00:00",
                "total_distance": "0 m", "avg_duration": "00:00"}
    total_s = sum(r.duration_s for r in records)
    total_m = sum(r.distance_m for r in records)
    avg_s = total_s / len(records)

    def hms(sec: float) -> str:
        h, rem = divmod(int(sec), 3600)
        m, s = divmod(rem, 60)
        return f"{h:02d}:{m:02d}:{s:02d}" if h else f"{m:02d}:{s:02d}"

    return {
        "flights": str(len(records)),
        "total_time": hms(total_s),
        "total_distance": (f"{total_m / 1000:.2f} km" if total_m >= 1000
                           else f"{total_m:.1f} m"),
        "avg_duration": hms(avg_s),
    }
