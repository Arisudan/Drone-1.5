"""
================================================================================
MODULE: video_health.py
PURPOSE: Video Feed Health - fps, pipeline latency, jitter, freeze detection
================================================================================

WHY THIS EXISTS:
  A frozen video frame looks exactly like a still scene. The last decoded frame
  stays on screen and nothing says the feed died, which is the worst possible
  failure for an operator flying by camera. This watches frame arrivals and
  says so.

WHAT "LATENCY" MEANS HERE (be precise, it is easy to over-claim):
  ``latency_ms`` is capture-thread read -> pixel painted *inside this station*:
  queueing, conversion and scaling. It does NOT include the encoder, the
  network or the camera's own exposure time, none of which this side can
  observe without a timestamp burned into the stream. It is a floor, and a good
  early-warning signal (it climbs when the UI thread or decoder falls behind),
  not a true glass-to-glass figure.

PURE PYTHON, INJECTABLE CLOCK: no Qt, so it is unit-testable.
================================================================================
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Optional, Tuple

IDLE = "IDLE"              # not started
CONNECTING = "CONNECTING"  # started, no real frame yet
LIVE = "LIVE"
DEGRADED = "DEGRADED"      # frames arriving, but slowly
FROZEN = "FROZEN"          # had frames, none for freeze_after_s
NO_SIGNAL = "NO SIGNAL"    # started, never got a real frame within freeze_after_s


@dataclass
class VideoHealth:
    state: str = IDLE
    fps: float = 0.0
    latency_ms: Optional[float] = None
    jitter_ms: float = 0.0
    age_s: Optional[float] = None   # seconds since the last real frame

    def text(self) -> str:
        if self.state in (LIVE, DEGRADED):
            lat = f" · pipeline {self.latency_ms:.0f} ms" if self.latency_ms is not None else ""
            return f"{self.state} · {self.fps:.0f} fps{lat} · jitter {self.jitter_ms:.0f} ms"
        if self.state in (FROZEN, NO_SIGNAL) and self.age_s is not None:
            return f"{self.state} · no frame for {self.age_s:.1f} s"
        return self.state


class VideoHealthMonitor:
    def __init__(self, freeze_after_s: float = 2.0, degraded_fps: float = 8.0,
                 window_s: float = 2.0, clock: Callable[[], float] = time.monotonic):
        self.freeze_after_s = freeze_after_s
        self.degraded_fps = degraded_fps
        self.window_s = window_s
        self._clock = clock
        self._arrivals: Deque[Tuple[float, Optional[float]]] = deque()  # (arrived, latency_s)
        self._started_at: Optional[float] = None
        self._last_real: Optional[float] = None

    def start(self) -> None:
        self._arrivals.clear()
        self._last_real = None
        self._started_at = self._clock()

    def stop(self) -> None:
        self._arrivals.clear()
        self._started_at = None
        self._last_real = None

    def on_frame(self, captured_at: Optional[float] = None, real: bool = True) -> None:
        """Record a displayed frame. ``real=False`` is a placeholder (e.g. the
        'camera not detected' card) and must not count as live video."""
        if not real:
            return
        now = self._clock()
        if self._started_at is None:
            self._started_at = now
        self._last_real = now
        lat = (now - captured_at) if captured_at is not None else None
        self._arrivals.append((now, lat if lat is None or lat >= 0 else None))
        self._prune(now)

    def _prune(self, now: float) -> None:
        while self._arrivals and now - self._arrivals[0][0] > self.window_s:
            self._arrivals.popleft()

    def snapshot(self) -> VideoHealth:
        now = self._clock()
        if self._started_at is None:
            return VideoHealth(IDLE)
        if self._last_real is None:
            waited = now - self._started_at
            return VideoHealth(NO_SIGNAL if waited > self.freeze_after_s else CONNECTING,
                               age_s=waited)
        age = now - self._last_real
        if age > self.freeze_after_s:
            return VideoHealth(FROZEN, age_s=age)
        self._prune(now)
        times = [a for a, _ in self._arrivals]
        fps = 0.0
        jitter = 0.0
        if len(times) >= 2 and times[-1] > times[0]:
            fps = (len(times) - 1) / (times[-1] - times[0])
            gaps = [b - a for a, b in zip(times, times[1:])]
            mean = sum(gaps) / len(gaps)
            jitter = math.sqrt(sum((g - mean) ** 2 for g in gaps) / len(gaps)) * 1000.0
        lats = [l for _, l in self._arrivals if l is not None]
        lat_ms = (sum(lats) / len(lats) * 1000.0) if lats else None
        state = LIVE if fps >= self.degraded_fps else DEGRADED
        return VideoHealth(state, fps, lat_ms, jitter, age)
