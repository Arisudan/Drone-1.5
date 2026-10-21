"""
================================================================================
MODULE: alarms.py
PURPOSE: Prioritised, Acknowledgeable Alarm List (pure Python, no Qt)
================================================================================

WHY THIS EXISTS:
  Toasts vanish, console lines scroll away, and audio alerts are edge-triggered
  and de-duplicated on purpose. None of them answers "what is wrong *right
  now*?". This keeps one entry per active fault, ordered by urgency, until the
  fault actually clears - the standing record an operator glances at.

SEMANTICS (they are the point):
  * One alarm per ``key``. Re-raising an active alarm refreshes it; it never
    stacks duplicates.
  * ``raise_alarm`` returns True only when the operator should be *told*: a new
    alarm, or an escalation to a higher level. A repeat returns False.
  * Acknowledging silences the attention-grabbing treatment, it does NOT clear
    the alarm - only the condition going away does. An acknowledged alarm that
    later escalates becomes unacknowledged again.
  * Cleared alarms move to a bounded history, so a flapping fault is visible
    after the fact.
================================================================================
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Deque, Dict, List, Optional
from collections import deque


class AlarmLevel(IntEnum):
    WARN = 1
    CRITICAL = 2


@dataclass
class Alarm:
    key: str
    level: AlarmLevel
    text: str
    raised_at: float
    last_seen: float
    acked: bool = False
    cleared_at: Optional[float] = None
    detail: str = ""        # one-line operator hint, shown under the title


def format_elapsed(seconds: float) -> str:
    """m:ss, or h:mm:ss past an hour. Negative input (clock step) reads 0:00."""
    total = max(0, int(seconds))
    h, rem = divmod(total, 3600)
    m, sec = divmod(rem, 60)
    return f"{h}:{m:02d}:{sec:02d}" if h else f"{m}:{sec:02d}"


class AlarmManager:
    def __init__(self, clock: Callable[[], float] = time.time, history_limit: int = 50):
        self._clock = clock
        self._active: Dict[str, Alarm] = {}
        self._history: Deque[Alarm] = deque(maxlen=history_limit)
        # Bumped on every visible change so a UI can skip redundant redraws.
        self.revision = 0

    # ── raising / clearing ─────────────────────────────────────────────────
    def now(self) -> float:
        return self._clock()

    def elapsed(self, alarm: Alarm) -> float:
        """Seconds the alarm has been active - computed on demand, never stored,
        so a display can show a running figure without anyone re-raising it."""
        return self._clock() - alarm.raised_at

    def raise_alarm(self, key: str, level: AlarmLevel, text: str, detail: str = "") -> bool:
        now = self._clock()
        cur = self._active.get(key)
        if cur is None:
            self._active[key] = Alarm(key, level, text, now, now, detail=detail)
            self.revision += 1
            return True
        cur.last_seen = now
        if text != cur.text or detail != cur.detail:
            cur.text = text
            cur.detail = detail
            self.revision += 1
        if level > cur.level:
            cur.level = level
            cur.acked = False
            self.revision += 1
            return True
        if level < cur.level:
            cur.level = level          # de-escalation keeps the ack state
            self.revision += 1
        return False

    def clear(self, key: str) -> bool:
        a = self._active.pop(key, None)
        if a is None:
            return False
        a.cleared_at = self._clock()
        self._history.appendleft(a)
        self.revision += 1
        return True

    def set_condition(self, key: str, active: bool, level: AlarmLevel = AlarmLevel.WARN,
                      text: str = "", detail: str = "") -> bool:
        """Level-triggered convenience for per-tick checks. Returns True when
        the operator should be told (see raise_alarm)."""
        if active:
            return self.raise_alarm(key, level, text, detail)
        self.clear(key)
        return False

    # ── acknowledging ──────────────────────────────────────────────────────
    def acknowledge(self, key: str) -> bool:
        a = self._active.get(key)
        if a is None or a.acked:
            return False
        a.acked = True
        self.revision += 1
        return True

    def acknowledge_all(self) -> int:
        n = 0
        for a in self._active.values():
            if not a.acked:
                a.acked = True
                n += 1
        if n:
            self.revision += 1
        return n

    # ── reading ────────────────────────────────────────────────────────────
    def active(self) -> List[Alarm]:
        """Unacknowledged first, then highest level, then most recently raised."""
        return sorted(self._active.values(),
                      key=lambda a: (a.acked, -int(a.level), -a.raised_at))

    def unacked(self) -> List[Alarm]:
        return [a for a in self.active() if not a.acked]

    def top(self) -> Optional[Alarm]:
        items = self.active()
        return items[0] if items else None

    def history(self) -> List[Alarm]:
        return list(self._history)

    def __len__(self) -> int:
        return len(self._active)

    def __contains__(self, key: str) -> bool:
        return key in self._active
