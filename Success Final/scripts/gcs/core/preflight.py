"""
================================================================================
MODULE: preflight.py
PURPOSE: Preflight checklist - what must be true before the drone is armed (no Qt)
================================================================================

WHY THIS EXISTS:
  Arming used to depend on the operator remembering a dozen things: is the link
  up, is the battery full enough, is the vision system actually tracking, is the
  camera alive, is the map flowing, was the heading re-fixed after tracking
  restarted (known issue 11). Each is already visible somewhere, but nothing put
  them in one place or stopped ARM when one was wrong.

HOW IT WORKS:
  evaluate() turns the facts the station already holds into a list of Check rows.
  A row is PASS, FAIL, or UNKNOWN ("we cannot tell" is never reported as PASS).
  Rows marked required must PASS before the ARM button is allowed; the others
  are shown for information. One row is MANUAL: the operator ticks it. It is
  cleared automatically when vision tracking is lost, because the heading has to
  be fixed again after every tracking reset.

  This module decides nothing about flight - the ARM button asks it, and the
  bench override (typing `arm force` in the flight terminal) deliberately skips
  it, exactly as it already skips PX4's own pre-arm checks.
================================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

PASS = "pass"
FAIL = "fail"
UNKNOWN = "unknown"

# A heartbeat older than this is a dead link, however recent the last one was.
HEARTBEAT_MAX_AGE_S = 3.0


@dataclass
class Check:
    key: str
    label: str
    status: str
    detail: str = ""
    required: bool = True
    manual: bool = False

    @property
    def ok(self) -> bool:
        return self.status == PASS


@dataclass
class PreflightInputs:
    """Plain facts, gathered by the main window once a second or so."""
    connected: bool = False
    heartbeat_age_s: float = 999.0
    armed: bool = False
    battery_pct: int = 0
    batt_warn_pct: int = 35
    batt_crit_pct: int = 20
    vision_ok: bool = False
    position_stale: bool = True
    video_state: str = "IDLE"
    map_seen: bool = False
    map_stale: bool = False
    params_loaded: bool = False
    radxa_state: Optional[str] = None       # "ok" | "degraded" | "down" | None (not reporting)
    radxa_detail: str = ""
    heading_confirmed: bool = False


def evaluate(i: PreflightInputs) -> List[Check]:
    """The checklist for these inputs, in the order it is shown."""
    out: List[Check] = []

    link_ok = i.connected and i.heartbeat_age_s < HEARTBEAT_MAX_AGE_S
    out.append(Check("link", "Link to the vehicle", PASS if link_ok else FAIL,
                     "" if link_ok else "No recent telemetry from the vehicle."))

    if not link_ok:
        # Everything below describes a vehicle we cannot hear.
        batt = Check("battery", "Battery", UNKNOWN, "No link.")
    elif i.battery_pct <= 0:
        batt = Check("battery", "Battery", UNKNOWN, "The vehicle is not reporting a battery level.")
    elif i.battery_pct <= i.batt_crit_pct:
        batt = Check("battery", "Battery", FAIL, f"{i.battery_pct}% - critical. Charge before flying.")
    elif i.battery_pct <= i.batt_warn_pct:
        batt = Check("battery", "Battery", FAIL, f"{i.battery_pct}% - low. Charge before flying.")
    else:
        batt = Check("battery", "Battery", PASS, f"{i.battery_pct}%")
    out.append(batt)

    out.append(Check("vision", "Vision tracking", PASS if (link_ok and i.vision_ok) else (FAIL if link_ok else UNKNOWN),
                     "" if (link_ok and i.vision_ok) else
                     ("No visual-inertial odometry. Move the drone slowly in front of textured surfaces."
                      if link_ok else "No link.")))
    out.append(Check("position", "Position feed", PASS if (link_ok and not i.position_stale) else (FAIL if link_ok else UNKNOWN),
                     "" if (link_ok and not i.position_stale) else
                     ("No fresh local position from the vehicle." if link_ok else "No link.")))

    live = i.video_state == "LIVE"
    out.append(Check("video", "Camera feed", PASS if live else FAIL,
                     "" if live else f"Video is {i.video_state.lower()}."))

    if not i.map_seen:
        out.append(Check("map", "Map", FAIL, "No map has arrived from the Radxa yet."))
    elif i.map_stale:
        out.append(Check("map", "Map", FAIL, "The map stopped updating."))
    else:
        out.append(Check("map", "Map", PASS))

    out.append(Check("heading", "Heading fixed", PASS if i.heading_confirmed else UNKNOWN,
                     "" if i.heading_confirmed else
                     "Tick this after turning the drone once so the heading is fixed (needed after every tracking restart).",
                     manual=True))

    if i.radxa_state is None:
        out.append(Check("radxa", "Radxa services", UNKNOWN, "The Radxa watchdog is not reporting.", required=False))
    elif i.radxa_state == "ok":
        out.append(Check("radxa", "Radxa services", PASS, i.radxa_detail, required=False))
    else:
        out.append(Check("radxa", "Radxa services", FAIL, i.radxa_detail or f"Radxa pipeline is {i.radxa_state}.",
                         required=False))

    out.append(Check("params", "Parameters read", PASS if i.params_loaded else UNKNOWN,
                     "" if i.params_loaded else "Open this tab while connected to read them.", required=False))
    return out


def blockers(checks: List[Check]) -> List[Check]:
    """Required rows that are not PASS."""
    return [c for c in checks if c.required and not c.ok]


def summary(checks: List[Check], armed: bool = False) -> Tuple[bool, str]:
    """(ready, one line for the footer of the checklist)."""
    if armed:
        return True, "Armed - the checklist applies before arming."
    bad = blockers(checks)
    if not bad:
        return True, "READY TO ARM"
    names = ", ".join(c.label for c in bad[:3]) + (f" and {len(bad) - 3} more" if len(bad) > 3 else "")
    return False, f"NOT READY - {len(bad)} to fix: {names}"
