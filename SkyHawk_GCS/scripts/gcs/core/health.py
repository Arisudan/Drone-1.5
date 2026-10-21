"""
================================================================================
MODULE: health.py
PURPOSE: Per-Component Health Metrics, Liveness Heartbeats & Stall Detection
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Core Data Layer) - also usable
                   headless by the diagnostics scripts.
  * Communicates:  Any long-lived worker (QThread, plain thread, or QTimer
                   callback) that produces something on a regular cadence.
  * Upstream:      heartbeat() / record_latency() calls from the worker itself.
  * Downstream:    Top status strip badges, CLI console warnings, watchdogs.

WHY THIS EXISTS:
  Every moving part of this GCS is a background worker that can die quietly:
  the MAVLink thread, the map listener's TCP client, the video reader, and -
  most dangerously - the 10 Hz OFFBOARD setpoint pump. When one stops
  producing, nothing in the UI currently notices: the widget simply keeps
  showing its last good value. From the operator's seat a frozen map and a
  live map look identical until the drone does something surprising.

  This module gives every component a uniform way to say "I am still
  running", and gives the UI a uniform way to ask "is anything stuck?".

DEADLINES, NOT JUST LIVENESS:
  A component registers the cadence it is supposed to keep. Two thresholds
  follow from it:
    * degrade_after_s - producing, but slower than it should be.
    * stall_after_s   - not producing at all; treat as dead.
  For the OFFBOARD setpoint pump these are not cosmetic: PX4 drops OFFBOARD
  mode if setpoints stop arriving for 500 ms, so the pump's stall threshold
  is a real flight deadline, not a UI nicety.

THREAD SAFETY:
  Every EngineHealth method takes an internal lock and touches no Qt object,
  so workers may call heartbeat()/record_latency() from any thread. Reading
  side (snapshot / snapshot_all) is equally safe. Nothing here emits Qt
  signals on purpose - the GUI polls snapshots from its own thread instead,
  which keeps all Qt access on the main thread by construction.

AUTO-RESTART IS NOT AUTOMATIC:
  Each component has different shutdown semantics and a blind restart can
  re-trigger the very fault that killed it. attach_restart_callback() lets a
  caller opt in; ComponentWatchdog will then invoke it exactly once per
  stall transition, never in a loop.

USAGE:
  from core.health import EngineHealth, EngineStatus, get_registry

  health = EngineHealth("OffboardPump", stall_after_s=0.5, degrade_after_s=0.25)
  get_registry().register(health)
  health.set_status(EngineStatus.READY)
  ...
  health.heartbeat()                 # once per produced item
  health.record_latency(ms)          # optional: how long producing took

  for snap in get_registry().snapshot_all():
      print(snap.name, snap.status, snap.rate_hz, snap.p95_latency_ms)
================================================================================
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional

log = logging.getLogger("gcs.health")


class EngineStatus(str, Enum):
    """Lifecycle state of a monitored component.

    str-valued so it renders directly in labels and serialises to JSON
    without a custom encoder.
    """

    UNINITIALIZED = "uninitialized"
    STARTING = "starting"
    READY = "ready"
    DEGRADED = "degraded"      # still producing, but slower than its cadence
    STALLED = "stalled"        # no heartbeat past the stall deadline
    FAILED = "failed"          # gave up / raised out of its own loop
    STOPPED = "stopped"        # deliberately shut down; absence is expected


#: Statuses where a missing heartbeat is expected and must NOT raise a stall.
_QUIET_STATUSES = frozenset(
    {EngineStatus.UNINITIALIZED, EngineStatus.STARTING,
     EngineStatus.STOPPED, EngineStatus.FAILED}
)


@dataclass
class HealthSnapshot:
    """Immutable point-in-time view of one component, safe to hand to the UI."""

    name: str
    status: EngineStatus
    beats: int
    age_s: float                       # seconds since the last heartbeat
    rate_hz: float                     # measured heartbeat rate over the window
    last_latency_ms: float
    p95_latency_ms: float
    stall_after_s: float
    degrade_after_s: float
    detail: str = ""

    @property
    def is_healthy(self) -> bool:
        return self.status in (EngineStatus.READY, EngineStatus.STARTING)

    def one_line(self) -> str:
        """Compact operator-facing summary, e.g. for a status strip tooltip."""
        if self.status in _QUIET_STATUSES and self.beats == 0:
            return f"{self.name}: {self.status.value}"
        rate = f"{self.rate_hz:.1f} Hz" if self.rate_hz > 0 else "-- Hz"
        lat = f"{self.p95_latency_ms:.0f} ms p95" if self.p95_latency_ms > 0 else "-- ms"
        return (f"{self.name}: {self.status.value} | {rate} | {lat} "
                f"| last beat {self.age_s:.1f}s ago")


class EngineHealth:
    """Liveness + latency bookkeeping for a single component.

    Parameters
    ----------
    name:
        Display name, also the registry key. Must be unique.
    stall_after_s:
        No heartbeat for this long (while in a live status) means STALLED.
    degrade_after_s:
        No heartbeat for this long means DEGRADED. Defaults to half the
        stall threshold. Must be < stall_after_s.
    window:
        How many recent heartbeats/latencies to keep for rate and p95.
    """

    def __init__(self, name: str, stall_after_s: float = 5.0,
                 degrade_after_s: Optional[float] = None,
                 window: int = 120):
        if stall_after_s <= 0:
            raise ValueError(f"{name}: stall_after_s must be > 0")
        if degrade_after_s is None:
            degrade_after_s = stall_after_s / 2.0
        if degrade_after_s <= 0 or degrade_after_s >= stall_after_s:
            raise ValueError(
                f"{name}: degrade_after_s ({degrade_after_s}) must be in "
                f"(0, stall_after_s={stall_after_s})")

        self.name = name
        self.stall_after_s = float(stall_after_s)
        self.degrade_after_s = float(degrade_after_s)

        self._lock = threading.Lock()
        self._status = EngineStatus.UNINITIALIZED
        self._detail = ""
        self._beats = 0
        self._last_beat = 0.0
        self._beat_times: deque = deque(maxlen=window)
        self._latencies: deque = deque(maxlen=window)
        self._last_latency = 0.0
        self._restart_cb: Optional[Callable[[], None]] = None
        self._stall_announced = False

    # ── producer side ───────────────────────────────────────────────

    def set_status(self, status: EngineStatus, detail: str = "") -> None:
        """Declare a lifecycle transition. Clears stall latch when going live."""
        with self._lock:
            if status != self._status:
                log.info("%s: %s -> %s%s", self.name, self._status.value,
                         status.value, f" ({detail})" if detail else "")
            self._status = status
            self._detail = detail
            if status in (EngineStatus.READY, EngineStatus.STARTING):
                # A fresh start must not inherit the previous run's stall
                # latch, or the watchdog would never fire again.
                self._stall_announced = False
                # Treat the transition itself as a beat so a component is not
                # instantly "stalled" for never having produced anything yet.
                self._last_beat = time.monotonic()

    def heartbeat(self) -> None:
        """Called once per produced item (frame, map, setpoint, packet)."""
        now = time.monotonic()
        with self._lock:
            self._beats += 1
            self._last_beat = now
            self._beat_times.append(now)
            # Producing again after a stall is itself the recovery signal.
            if self._status in (EngineStatus.STALLED, EngineStatus.DEGRADED):
                log.info("%s: recovered (%s -> ready)", self.name,
                         self._status.value)
                self._status = EngineStatus.READY
                self._stall_announced = False

    def record_latency(self, ms: float) -> None:
        """Record how long one production cycle took, in milliseconds."""
        with self._lock:
            self._last_latency = float(ms)
            self._latencies.append(float(ms))

    def attach_restart_callback(self, cb: Callable[[], None]) -> None:
        """Opt in to watchdog-driven recovery. Called at most once per stall."""
        with self._lock:
            self._restart_cb = cb

    # ── consumer side ───────────────────────────────────────────────

    def snapshot(self, now: Optional[float] = None) -> HealthSnapshot:
        """Current state. Pure read - never mutates status (see check_stall)."""
        now = time.monotonic() if now is None else now
        with self._lock:
            age = (now - self._last_beat) if self._last_beat else float("inf")
            status = self._status

            # Derive DEGRADED/STALLED for display without latching anything;
            # check_stall() owns the actual state transition.
            if status == EngineStatus.READY and self._last_beat:
                if age >= self.stall_after_s:
                    status = EngineStatus.STALLED
                elif age >= self.degrade_after_s:
                    status = EngineStatus.DEGRADED

            return HealthSnapshot(
                name=self.name,
                status=status,
                beats=self._beats,
                age_s=(0.0 if age == float("inf") else age),
                rate_hz=self._rate_locked(now),
                last_latency_ms=self._last_latency,
                p95_latency_ms=self._p95_locked(),
                stall_after_s=self.stall_after_s,
                degrade_after_s=self.degrade_after_s,
                detail=self._detail,
            )

    def check_stall(self, now: Optional[float] = None) -> bool:
        """Latch a STALLED transition. Returns True only on the first call
        that observes a new stall, so callers can alert exactly once."""
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._status in _QUIET_STATUSES:
                return False
            if not self._last_beat:
                return False
            if (now - self._last_beat) < self.stall_after_s:
                return False
            if self._stall_announced:
                return False
            self._stall_announced = True
            self._status = EngineStatus.STALLED
            log.warning("%s: STALLED - no heartbeat for %.2fs (limit %.2fs)",
                        self.name, now - self._last_beat, self.stall_after_s)
            return True

    def get_restart_callback(self) -> Optional[Callable[[], None]]:
        with self._lock:
            return self._restart_cb

    # ── internals (call with lock held) ──────────────────────────────

    def _rate_locked(self, now: float) -> float:
        if len(self._beat_times) < 2:
            return 0.0
        span = self._beat_times[-1] - self._beat_times[0]
        if span <= 0:
            return 0.0
        rate = (len(self._beat_times) - 1) / span
        # A stale window would keep reporting the old rate forever; decay it
        # once the component is past its own degrade deadline.
        idle = now - self._beat_times[-1]
        if idle >= self.degrade_after_s:
            return 0.0
        return rate

    def _p95_locked(self) -> float:
        if not self._latencies:
            return 0.0
        ordered = sorted(self._latencies)
        # Nearest-rank p95: index of the smallest value at or above the 95th
        # percentile. Avoids pulling numpy in for a handful of samples.
        idx = max(0, int(round(0.95 * len(ordered))) - 1)
        return ordered[idx]


class HealthRegistry:
    """Process-wide collection of EngineHealth objects."""

    def __init__(self):
        self._lock = threading.Lock()
        self._items: Dict[str, EngineHealth] = {}

    def register(self, health: EngineHealth) -> EngineHealth:
        with self._lock:
            self._items[health.name] = health
        return health

    def unregister(self, name: str) -> None:
        with self._lock:
            self._items.pop(name, None)

    def get(self, name: str) -> Optional[EngineHealth]:
        with self._lock:
            return self._items.get(name)

    def all(self) -> List[EngineHealth]:
        with self._lock:
            return list(self._items.values())

    def snapshot_all(self) -> List[HealthSnapshot]:
        now = time.monotonic()
        return [h.snapshot(now) for h in self.all()]

    def unhealthy(self) -> List[HealthSnapshot]:
        """Snapshots worth showing the operator (degraded / stalled / failed)."""
        return [s for s in self.snapshot_all()
                if s.status in (EngineStatus.DEGRADED, EngineStatus.STALLED,
                                EngineStatus.FAILED)]

    def clear(self) -> None:
        """Drop every registration. Intended for tests."""
        with self._lock:
            self._items.clear()


_REGISTRY = HealthRegistry()


def get_registry() -> HealthRegistry:
    """The process-wide registry every component registers with."""
    return _REGISTRY


class ComponentWatchdog:
    """Background poller that latches stalls and fires restart callbacks.

    The GUI does not need this - it polls snapshots from its own UI tick,
    which keeps every Qt touch on the main thread. This exists for headless
    users (diagnostics scripts, the Radxa-side monitor) that have no event
    loop to piggyback on.
    """

    def __init__(self, registry: Optional[HealthRegistry] = None,
                 poll_s: float = 1.0,
                 on_stall: Optional[Callable[[HealthSnapshot], None]] = None):
        self._registry = registry if registry is not None else get_registry()
        self._poll_s = float(poll_s)
        self._on_stall = on_stall
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="health-watchdog",
                                        daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None

    def poll_once(self) -> List[HealthSnapshot]:
        """One sweep. Returns the components that stalled on *this* sweep."""
        newly_stalled: List[HealthSnapshot] = []
        for health in self._registry.all():
            if not health.check_stall():
                continue
            snap = health.snapshot()
            newly_stalled.append(snap)
            if self._on_stall is not None:
                try:
                    self._on_stall(snap)
                except Exception:
                    log.exception("on_stall callback failed for %s", snap.name)
            cb = health.get_restart_callback()
            if cb is not None:
                try:
                    log.warning("%s: invoking restart callback", snap.name)
                    cb()
                except Exception:
                    log.exception("restart callback failed for %s", snap.name)
        return newly_stalled

    def _run(self) -> None:
        while not self._stop.wait(self._poll_s):
            try:
                self.poll_once()
            except Exception:
                log.exception("watchdog sweep failed")
