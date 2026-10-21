"""
================================================================================
MODULE: motor_range.py
PURPOSE: The vehicle's configured motor range and the one set of PWM thresholds
================================================================================

WHY THIS EXISTS:
  The motor bars and the frame diagram used a fixed 1000-2000 us scale, so a
  vehicle configured for 1100-1900 us read 0 % at 1000 us and only ~80 % at full
  throttle, and three widgets disagreed about what "idle" or "saturated" meant
  (the status line's thresholds differed from the diagram's). This is the single
  place both questions are answered, from the vehicle's own parameters:

    PWM_MAIN_MINn / PWM_MAIN_MAXn   output n's throttle range
    PWM_MAIN_DISn                   what output n sits at while disarmed
    PWM_MAIN_FUNCn                  101..104 = Motor 1..4 on output n

  FUNCn matters: ACTUATOR_TEST addresses a motor by function (MOTORk), and
  SERVO_OUTPUT_RAW is indexed by output. If Motor 1 is not on output 1 the old
  code lit the wrong arm; ``motor_pwms`` reorders output-indexed PWMs into
  motor order.

UNKNOWN IS NOT SILENTLY GUESSED:
  Until MIN and MAX for all four outputs have arrived, ``known`` is False and the
  documented defaults (1000-2000, disarm 1000) are used - the UI says so rather
  than presenting a default scale as the vehicle's.

Pure Python, no Qt.
================================================================================
"""

from __future__ import annotations

import re
from typing import List, Optional, Sequence

DEFAULT_MIN = 1000
DEFAULT_MAX = 2000
DEFAULT_DIS = 1000

# Fractions of the output's own range. Same proportions the old fixed 1000-2000
# thresholds (1050 / 1750 / 1900) encoded, now relative to the real range.
IDLE_FRAC = 0.05
NOMINAL_FRAC = 0.75
HIGH_FRAC = 0.90

OFF = "off"            # below the output's minimum: disarmed / not driven
IDLE = "idle"
NOMINAL = "nominal"
HIGH = "high"
SATURATED = "saturated"

PARAM_RE = re.compile(r"^PWM_MAIN_(MIN|MAX|DIS|FUNC)([1-8])$")
MOTOR_FUNC_BASE = 100          # PX4: 101..104 = Motor1..Motor4
N_MOTORS = 4


def param_names() -> List[str]:
    """Every parameter this model reads, outputs 1..4 (all SERVO_OUTPUT_RAW
    carries here)."""
    return [f"PWM_MAIN_{k}{n}" for k in ("MIN", "MAX", "DIS", "FUNC")
            for n in range(1, N_MOTORS + 1)]


class MotorRange:
    def __init__(self) -> None:
        self.out_min: List[Optional[int]] = [None] * N_MOTORS
        self.out_max: List[Optional[int]] = [None] * N_MOTORS
        self.out_dis: List[Optional[int]] = [None] * N_MOTORS
        self.out_func: List[Optional[int]] = [None] * N_MOTORS

    # ── parameters in ──────────────────────────────────────────────────────
    def on_param(self, name: str, value: float) -> bool:
        """Feed one PARAM_VALUE. Returns True if it changed the model."""
        m = PARAM_RE.match(name)
        if not m:
            return False
        idx = int(m.group(2)) - 1
        if idx >= N_MOTORS:
            return False                # output beyond SERVO_OUTPUT_RAW 1..4
        table = {"MIN": self.out_min, "MAX": self.out_max,
                 "DIS": self.out_dis, "FUNC": self.out_func}[m.group(1)]
        new = int(round(float(value)))
        if table[idx] == new:
            return False
        table[idx] = new
        return True

    def reset(self) -> None:
        self.__init__()

    # ── what is known ──────────────────────────────────────────────────────
    @property
    def known(self) -> bool:
        """True once MIN and MAX have arrived for all four outputs and agree
        with each other enough to form a range."""
        return all(v is not None for v in self.out_min + self.out_max) and all(
            self.out_max[i] > self.out_min[i] for i in range(N_MOTORS))

    def missing(self) -> List[str]:
        """Parameter names still needed, for a targeted re-request."""
        out = []
        for kind, table in (("MIN", self.out_min), ("MAX", self.out_max),
                            ("DIS", self.out_dis), ("FUNC", self.out_func)):
            out += [f"PWM_MAIN_{kind}{i + 1}" for i in range(N_MOTORS) if table[i] is None]
        return out

    # ── motor <-> output mapping ───────────────────────────────────────────
    def output_for_motor(self, motor: int) -> int:
        """0-based output index feeding Motor ``motor`` (1..4).

        Falls back to the identity when FUNC is unknown or is not a clean
        permutation of the four outputs - a half-known mapping must not shuffle
        the diagram.
        """
        funcs = self.out_func
        if all(f is not None for f in funcs):
            wanted = [MOTOR_FUNC_BASE + k for k in range(1, N_MOTORS + 1)]
            if sorted(funcs) == wanted:
                return funcs.index(MOTOR_FUNC_BASE + motor)
        return motor - 1

    def motor_pwms(self, output_pwms: Sequence[int]) -> List[int]:
        """Reorder output-indexed PWMs (SERVO_OUTPUT_RAW 1..4) into motor order."""
        padded = list(output_pwms[:N_MOTORS]) + [DEFAULT_DIS] * max(0, N_MOTORS - len(output_pwms))
        return [padded[self.output_for_motor(k)] for k in range(1, N_MOTORS + 1)]

    # ── per-motor numbers ──────────────────────────────────────────────────
    def _bounds(self, motor: int):
        i = self.output_for_motor(motor)
        lo = self.out_min[i] if self.out_min[i] is not None else DEFAULT_MIN
        hi = self.out_max[i] if self.out_max[i] is not None else DEFAULT_MAX
        if hi <= lo:
            lo, hi = DEFAULT_MIN, DEFAULT_MAX
        return lo, hi

    def disarm_pwm(self, motor: int) -> int:
        i = self.output_for_motor(motor)
        return self.out_dis[i] if self.out_dis[i] is not None else DEFAULT_DIS

    def fraction(self, motor: int, pwm: float) -> float:
        """0..1 position of ``pwm`` within the motor's own min..max."""
        lo, hi = self._bounds(motor)
        return max(0.0, min(1.0, (pwm - lo) / float(hi - lo)))

    def state(self, motor: int, pwm: float) -> str:
        lo, hi = self._bounds(motor)
        if pwm < lo:
            return OFF
        frac = self.fraction(motor, pwm)
        if pwm > hi:
            return SATURATED
        if frac <= IDLE_FRAC:
            return IDLE
        if frac <= NOMINAL_FRAC:
            return NOMINAL
        if frac <= HIGH_FRAC:
            return HIGH
        return SATURATED

    def pwm_for_throttle(self, motor: int, pct: float) -> int:
        """PWM an ACTUATOR_TEST of ``pct`` % drives the motor's output to: PX4
        maps the 0..1 value onto that output's min..max."""
        lo, hi = self._bounds(motor)
        return int(round(lo + max(0.0, min(100.0, pct)) / 100.0 * (hi - lo)))

    def worst_state(self, pwms: Sequence[int]) -> str:
        """Most severe state across motors, for the one-line status."""
        order = [OFF, IDLE, NOMINAL, HIGH, SATURATED]
        states = [self.state(k, pwms[k - 1]) for k in range(1, N_MOTORS + 1)]
        return max(states, key=order.index)

    def describe(self) -> str:
        if not self.known:
            return f"default {DEFAULT_MIN}–{DEFAULT_MAX} µs (vehicle range not read yet)"
        los = {self.out_min[i] for i in range(N_MOTORS)}
        his = {self.out_max[i] for i in range(N_MOTORS)}
        if len(los) == 1 and len(his) == 1:
            return f"{los.pop()}–{his.pop()} µs (from vehicle)"
        return "per-motor ranges (from vehicle)"


# One model for the whole station: the Motors tab and the Diagnostics grid both
# read it, and the main window feeds it from the vehicle's parameters, so they
# can never disagree about the scale or what "idle" means.
SHARED = MotorRange()

# SERVO_OUTPUT_RAW arrives at 5 Hz. Older than this and the motor display is
# showing the past, not the vehicle.
STALE_S = 2.0
