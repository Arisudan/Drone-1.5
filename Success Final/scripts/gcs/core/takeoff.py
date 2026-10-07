"""
================================================================================
MODULE: takeoff.py
PURPOSE: The altitude number sent with MAV_CMD_NAV_TAKEOFF (no Qt)
================================================================================

WHY THIS EXISTS:
  MAV_CMD_NAV_TAKEOFF's last parameter is an ABSOLUTE altitude (metres above mean
  sea level), not a height above the ground. The GCS used to send the operator's
  "1.5 m" as that number. On a vehicle whose reference altitude is near zero (the
  indoor, vision-only airframe) that happened to work. On anything with a real
  altitude reference - PX4 SITL in Gazebo or SIH sits at 488-489 m - PX4 compared
  1.5 with the 489 m it was already at, logged "Already higher than takeoff
  altitude", answered ACCEPTED, and the drone never climbed.

THE RULE:
  If the vehicle reports its altitude above sea level (GLOBAL_POSITION_INT, seen
  within the last few seconds), the target is that altitude PLUS the requested
  height. If it does not report one, behave exactly as before and send the height
  as it is. The second case is deliberate: a vehicle with no global reference
  is the one the old behaviour was proven on, so it must not be taken away.
================================================================================
"""

from __future__ import annotations

from typing import Optional, Tuple

AMSL_MAX_AGE_S = 3.0          # a reading older than this is not trusted


def takeoff_altitude_param(height_m: float, amsl_m: Optional[float],
                           age_s: Optional[float],
                           max_age_s: float = AMSL_MAX_AGE_S) -> Tuple[float, str]:
    """(value for NAV_TAKEOFF param7, "amsl" | "as-is")."""
    if amsl_m is not None and age_s is not None and 0.0 <= age_s <= max_age_s:
        return float(amsl_m) + float(height_m), "amsl"
    return float(height_m), "as-is"
