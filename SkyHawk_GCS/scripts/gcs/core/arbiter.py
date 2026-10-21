"""
================================================================================
MODULE: arbiter.py
PURPOSE: GCS command arbitration - may the GCS move the vehicle, and how (no Qt)
================================================================================

WHY THIS EXISTS:
  On the Point-LIO drone a pilot in POSCTL held position to 0.05 m/s, while GCS-commanded
  flight drifted (2026-10-15). The causes were in how the GCS commanded, not in the
  estimate: NAV_TAKEOFF was sent with yaw/lat/lon = 0 when PX4 had no global origin (turn
  to an arbitrary "north", no horizontal target, then a Hold that needs global position),
  and nothing checked that PX4 was actually fusing the lidar before handing it autonomy.
  Every GCS motion command now passes one gatekeeper with four jobs:

  1. TAKEOFF BY FRAME   PX4 has a global origin -> NAV_TAKEOFF with yaw/lat/lon NaN ("keep
                        current"). No origin -> an OFFBOARD climb at the current x, y and
                        heading. The yaw-0 / (0, 0) path no longer exists.
  2. ESTIMATOR GATE     PX4's own ESTIMATOR_STATUS must report attitude, horizontal velocity
                        and relative horizontal position valid with the position test ratio
                        below 1, the lidar must be reaching the FC, and LOCAL_POSITION_NED
                        must be fresh. "Packets seen" is not "fused".
  3. PILOT WINS         roll/pitch/yaw sticks off centre, or the throttle moved from where it
                        was when the GCS took over, by more than stick_takeover -> the GCS
                        lets go and stays out until the operator re-enables it.
  4. MOTION LIMITS      per-command horizontal / vertical step, an altitude ceiling, and a
                        keep-in box around the take-off point. Rejected, never clipped.

Inputs are plain numbers so every rule is unit-tested without a vehicle (tests/test_arbiter.py).
================================================================================
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

# MAV ESTIMATOR_STATUS_FLAGS
EST_ATTITUDE = 1
EST_VELOCITY_HORIZ = 2
EST_POS_HORIZ_REL = 8
EST_REQUIRED = EST_ATTITUDE | EST_VELOCITY_HORIZ | EST_POS_HORIZ_REL

READY, BLOCKED, PILOT = "READY", "BLOCKED", "PILOT"


@dataclass
class ArbiterLimits:
    max_step_h_m: float = 2.0          # one command's horizontal displacement
    max_step_v_m: float = 1.0          # one command's vertical displacement
    ceiling_m: float = 2.0             # height above the take-off point
    box_half_m: float = 5.0            # keep-in square around the take-off point (half side)
    stick_takeover: float = 0.10       # fraction of full stick travel = "pilot took over"
    vision_max_age_s: float = 1.0      # lidar pose must have reached the FC this recently
    estimator_max_age_s: float = 2.0   # ESTIMATOR_STATUS must be this fresh
    position_max_age_s: float = 1.0    # LOCAL_POSITION_NED must be this fresh

    def validate(self) -> None:
        for name in ("max_step_h_m", "max_step_v_m", "ceiling_m", "box_half_m",
                     "vision_max_age_s", "estimator_max_age_s", "position_max_age_s"):
            if not getattr(self, name) > 0:
                raise ValueError(f"arbiter.{name} must be > 0")
        if not 0.02 <= self.stick_takeover <= 0.5:
            raise ValueError("arbiter.stick_takeover outside 0.02..0.5")


@dataclass
class HealthInputs:
    connected: bool
    est_flags: int                      # ESTIMATOR_STATUS.flags (0 if never received)
    est_pos_ratio: float                # ESTIMATOR_STATUS.pos_horiz_ratio (nan if unknown)
    est_age_s: float                    # since the last ESTIMATOR_STATUS
    vision_age_s: float                 # since the last VISION_POSITION_ESTIMATE seen
    position_age_s: float               # since the last LOCAL_POSITION_NED


def health(i: HealthInputs, lim: ArbiterLimits) -> Tuple[bool, str]:
    """(ok, reason). The reason names the FIRST failing condition, for the operator."""
    if not i.connected:
        return False, "no link to the vehicle"
    if i.est_age_s > lim.estimator_max_age_s:
        return False, "no ESTIMATOR_STATUS from PX4"
    missing = EST_REQUIRED & ~int(i.est_flags)
    if missing & EST_ATTITUDE:
        return False, "EKF attitude not valid"
    if missing & EST_POS_HORIZ_REL:
        return False, "EKF has no valid horizontal position"
    if missing & EST_VELOCITY_HORIZ:
        return False, "EKF has no valid horizontal velocity"
    if math.isfinite(i.est_pos_ratio) and i.est_pos_ratio >= 1.0:
        return False, f"EKF rejecting position (test ratio {i.est_pos_ratio:.2f})"
    if i.vision_age_s > lim.vision_max_age_s:
        return False, (f"lidar position not reaching the FC ({i.vision_age_s:.1f} s)"
                       if i.vision_age_s < 900 else "lidar position never seen")
    if i.position_age_s > lim.position_max_age_s:
        return False, f"local position stale ({i.position_age_s:.1f} s)"
    return True, "estimator healthy"


# ── 1. takeoff by frame ─────────────────────────────────────────────

@dataclass
class TakeoffPlan:
    method: str                      # "nav_takeoff" | "offboard"
    param7: float = float("nan")     # NAV_TAKEOFF absolute altitude (AMSL, m)
    x: float = 0.0                   # OFFBOARD target, local NED
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0
    why: str = ""


def takeoff_plan(height_m: float, has_origin: bool, amsl_m: Optional[float],
                 x: float, y: float, z: float, heading_deg: float) -> TakeoffPlan:
    """How to take off `height_m` above where the vehicle stands, in the frame PX4 actually has.
    NAV_TAKEOFF runs in PX4's navigator (lat/lon) and its Hold needs a global position, so it
    is only used with an origin AND a fresh altitude above sea level to add the height to."""
    if has_origin and amsl_m is not None and math.isfinite(amsl_m):
        return TakeoffPlan("nav_takeoff", param7=amsl_m + height_m,
                           why=(f"PX4 has a global origin: NAV_TAKEOFF to {amsl_m + height_m:.1f} m "
                                f"above sea level (vehicle's {amsl_m:.1f} m + {height_m:.1f} m), "
                                f"holding the current position and heading"))
    return TakeoffPlan("offboard", x=x, y=y, z=z - abs(height_m), yaw_deg=heading_deg,
                       why=("PX4 has NO global origin: OFFBOARD climb straight up at the current "
                            "position and heading (NAV_TAKEOFF would have no horizontal target)"))


# ── 3. pilot wins ───────────────────────────────────────────────────

def stick_norm(pwm: float) -> float:
    """1000..2000 us -> -1..+1 (0 = centre). Non-positive / missing channels read 0."""
    if not pwm or pwm <= 0 or pwm == 65535:
        return 0.0
    return max(-1.0, min(1.0, (float(pwm) - 1500.0) / 500.0))


def pilot_took_over(roll: float, pitch: float, yaw: float, throttle: float,
                    throttle_at_engage: Optional[float], lim: ArbiterLimits) -> Optional[str]:
    """None, or which stick moved. Inputs are -1..+1 (stick_norm). Roll/pitch/yaw are judged
    against centre; the throttle against where it sat when the GCS took control, because it
    rests at the bottom (or wherever the pilot left it), not in the middle."""
    th = lim.stick_takeover * 2.0           # full travel is 2.0 in the -1..+1 scale
    for name, v in (("roll", roll), ("pitch", pitch), ("yaw", yaw)):
        if abs(v) > th:
            return f"{name} stick moved"
    if throttle_at_engage is not None and abs(throttle - throttle_at_engage) > th:
        return "throttle moved"
    return None


# ── 4. motion limits ────────────────────────────────────────────────

def check_target(cur: Sequence[float], target: Sequence[float],
                 home: Optional[Sequence[float]], lim: ArbiterLimits,
                 check_step: bool = True) -> Tuple[bool, str]:
    """A local-NED target (x, y, z) from the current position. `home` is the take-off point;
    without one only the step limits apply. check_step=False for route waypoints, whose legs
    are planned rather than nudged (box and ceiling still apply)."""
    dx, dy, dz = (target[i] - cur[i] for i in range(3))
    if check_step:
        h = math.hypot(dx, dy)
        if h > lim.max_step_h_m + 1e-6:
            return False, f"horizontal step {h:.2f} m exceeds {lim.max_step_h_m:.1f} m"
        if abs(dz) > lim.max_step_v_m + 1e-6:
            return False, f"vertical step {abs(dz):.2f} m exceeds {lim.max_step_v_m:.1f} m"
    if home is not None:
        height = home[2] - target[2]                       # NED: up is -z
        if height > lim.ceiling_m + 1e-6:
            return False, f"target {height:.2f} m above take-off exceeds the {lim.ceiling_m:.1f} m ceiling"
        ox, oy = target[0] - home[0], target[1] - home[1]
        if max(abs(ox), abs(oy)) > lim.box_half_m + 1e-6:
            return False, (f"target ({ox:+.1f}, {oy:+.1f}) m from take-off leaves the "
                           f"+/-{lim.box_half_m:.1f} m box")
    return True, "within limits"
