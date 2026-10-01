"""
================================================================================
MODULE: ui_stall.py
PURPOSE: UI-Thread Stall Detector (30 Hz tick overrun)
================================================================================

WHY THIS EXISTS:
  Every worker hands data to the UI through queued Qt signals, so a slow slot
  or a blocking call on the UI thread freezes the cockpit while telemetry keeps
  arriving unseen. The 30 Hz UI tick is the heartbeat of that thread: if the gap
  between two ticks balloons, something on the UI thread blocked. This measures
  that gap. Pure Python (no Qt) so it is unit-testable and the clock is
  injectable.
================================================================================
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional


@dataclass
class StallStats:
    ticks: int = 0
    stalls: int = 0
    last_gap_s: float = 0.0
    worst_gap_s: float = 0.0


class UiStallMonitor:
    """Feed it once per UI tick; ``tick()`` returns the gap if it was a stall.

    ``stall_after_s`` should sit well above the tick period (33 ms) so ordinary
    jitter is ignored. ``cooldown_s`` stops a long freeze reporting repeatedly.
    """

    def __init__(self, stall_after_s: float = 0.25, cooldown_s: float = 5.0,
                 clock: Callable[[], float] = time.monotonic):
        self.stall_after_s = stall_after_s
        self.cooldown_s = cooldown_s
        self._clock = clock
        self._last: Optional[float] = None
        self._last_report: Optional[float] = None
        self.stats = StallStats()

    def tick(self) -> Optional[float]:
        now = self._clock()
        prev, self._last = self._last, now
        if prev is None:
            return None
        gap = now - prev
        st = self.stats
        st.ticks += 1
        st.last_gap_s = gap
        st.worst_gap_s = max(st.worst_gap_s, gap)
        if gap < self.stall_after_s:
            return None
        st.stalls += 1
        if self._last_report is not None and now - self._last_report < self.cooldown_s:
            return None
        self._last_report = now
        return gap

    def reset(self) -> None:
        self._last = None
        self._last_report = None
        self.stats = StallStats()
