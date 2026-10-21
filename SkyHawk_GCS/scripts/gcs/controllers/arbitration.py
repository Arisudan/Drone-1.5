"""
================================================================================
MODULE: controllers/arbitration.py
PURPOSE: GCS command arbitration in the main window - gate, takeoff dispatch, watchdog
================================================================================
Mixed into DroneGCSMainWindow, like controllers/flight_commands.py. The rules live in
core/arbiter.py (pure, unit-tested); this file applies them to the live window:

  _arb_allow(what)          before any GCS motion command: pilot lock + estimator gate
  _arb_check_target(...)    motion limits for one target (step, ceiling, keep-in box)
  _arb_takeoff(height)      takeoff by frame: NAV_TAKEOFF (origin) or an OFFBOARD climb
  _arb_tick(t)              every telemetry update while the GCS commands: pilot sticks and
                            estimator health; either one ends GCS control
  _check_origin()           the CHECK ORIGIN button: ask PX4, show its acknowledgement

States shown on the GCS CTRL chip: READY / BLOCKED (reason) / PILOT. After a pilot takeover
the GCS stays out until the operator clicks the chip - it never takes control back itself.
================================================================================
"""
from __future__ import annotations

import time

from PyQt5.QtCore import QTimer

from core.arbiter import (BLOCKED, PILOT, READY, HealthInputs, check_target, health,
                          pilot_took_over, stick_norm, takeoff_plan)

# RC_MAP_ROLL/PITCH/THROTTLE/YAW read off the FC on 2026-10-15: channels 1, 2, 3, 4.
RC_ROLL, RC_PITCH, RC_THROTTLE, RC_YAW = 0, 1, 2, 3

#: modes in which a GCS command is flying the vehicle (the watchdog only acts in these)
GCS_FLOWN_MODES = ("OFFBOARD", "AUTO.TAKEOFF", "AUTO.LOITER", "AUTO.MISSION")
#: an estimator failure must persist this long before GCS control is pulled (one bad packet is not a fault)
HEALTH_FAIL_DEBOUNCE_S = 0.5
#: how long PX4 gets to answer a GPS_GLOBAL_ORIGIN request
ORIGIN_ANSWER_S = 3.0


class ArbitrationMixin:
    """See module docstring. Expects: settings, worker, last_telemetry, console, page_terminal,
    toast, _cmd_mode, offboard_pump_timer and the arbitration widgets built in _init_ui."""

    # ── state ──
    def _arb_init(self) -> None:
        self._arb_state = READY
        self._arb_reason = ""
        self._arb_active = False          # a GCS command is currently flying the vehicle
        self._arb_throttle0 = None
        self._arb_home = None             # (x, y, z) local NED of the take-off point
        self._arb_fail_since = None
        self._arb_was_armed = False

    @property
    def _arb_on(self) -> bool:
        return bool(getattr(self.settings.arbiter, "enabled", True))

    def _arb_say(self, msg: str, error: bool = True, toast_colour: str = "#da3633") -> None:
        (self.console.log_error if error else self.console.log_info)(msg)
        (self.page_terminal.log_error if error else self.page_terminal.log_info)(msg)
        self.toast.show_message(msg if len(msg) < 70 else msg[:67] + "...", toast_colour, 4500)

    def _arb_inputs(self, t) -> HealthInputs:
        now = time.time()
        return HealthInputs(
            connected=bool(t.connected),
            est_flags=int(t.est_flags),
            est_pos_ratio=float(t.est_pos_ratio),
            est_age_s=(now - t.last_est_time) if t.last_est_time else 999.0,
            vision_age_s=(now - t.last_vision_time) if t.last_vision_time else 999.0,
            position_age_s=float(t.position_age),
        )

    def _arb_sticks(self, t):
        raw = tuple(t.rc_raw) + (0,) * 8
        return (stick_norm(raw[RC_ROLL]), stick_norm(raw[RC_PITCH]),
                stick_norm(raw[RC_YAW]), stick_norm(raw[RC_THROTTLE]))

    # ── before a command ──
    def _arb_allow(self, what: str) -> bool:
        """May the GCS fly this command? False = refused, reason already shown."""
        if not self._arb_on:
            return True
        if self._arb_state == PILOT:
            self._arb_say(f"Cannot {what}: the pilot has control. Click GCS CTRL to hand "
                          "control back to the GCS first.")
            return False
        ok, reason = health(self._arb_inputs(self.last_telemetry), self.settings.arbiter)
        if not ok:
            self._arb_set(BLOCKED, reason)
            self._arb_say(f"Cannot {what}: {reason}.")
            return False
        return True

    def _arb_check_target(self, what: str, target, check_step: bool = True) -> bool:
        if not self._arb_on:
            return True
        t = self.last_telemetry
        ok, reason = check_target((t.x, t.y, t.z), target, self._arb_home,
                                  self.settings.arbiter, check_step=check_step)
        if not ok:
            self._arb_say(f"{what} rejected: {reason}.")
        return ok

    def _arb_engage(self) -> None:
        """A GCS command is about to fly the vehicle: arm the watchdog."""
        self._arb_active = True
        self._arb_fail_since = None
        self._arb_throttle0 = self._arb_sticks(self.last_telemetry)[3]

    # ── takeoff by frame ──
    def _arb_takeoff(self, height_m: float) -> bool:
        """Dispatch a ground takeoff the way PX4's current frame supports. True if sent."""
        t = self.last_telemetry
        lim = self.settings.arbiter
        if self._arb_on and height_m > lim.ceiling_m + 1e-6:
            self._arb_say(f"Takeoff rejected: {height_m:.1f} m exceeds the {lim.ceiling_m:.1f} m ceiling.")
            return False
        amsl, age = t.amsl_reading()
        has_origin = amsl is not None and age is not None and age <= 3.0
        plan = takeoff_plan(height_m, has_origin, amsl, t.x, t.y, t.z, t.heading)
        self._arb_home = (t.x, t.y, t.z)
        self.console.log_cmd(f"Takeoff {height_m:.1f} m - {plan.why}")
        self.page_terminal.log_cmd(f"Takeoff {height_m:.1f} m - {plan.why}")
        self._arb_engage()
        if plan.method == "nav_takeoff":
            self.worker.takeoff(height_m, target_amsl=plan.param7)
        else:
            self.worker.takeoff_offboard(plan.x, plan.y, plan.z, plan.yaw_deg)
            # PX4 refuses OFFBOARD without a live stream; give it a second of setpoints first.
            QTimer.singleShot(1000, lambda: self._cmd_mode("OFFBOARD"))
        return True

    # ── watchdog ──
    def _arb_tick(self, t) -> None:
        # The ceiling and keep-in box are measured from where the vehicle ARMED, so they hold
        # for a GCS command after an RC takeoff too (a GCS takeoff records the same point).
        if t.armed and not self._arb_was_armed:
            self._arb_home = (t.x, t.y, t.z)
        elif not t.armed and self._arb_was_armed:
            self._arb_home = None
        self._arb_was_armed = bool(t.armed)
        self._arb_refresh_chip(t)
        if not (self._arb_on and self._arb_active):
            return
        if not t.armed:
            self._arb_active = False          # landed and disarmed: nothing to watch
            return
        if t.flight_mode not in GCS_FLOWN_MODES:
            return                            # the pilot's own mode: PX4/RC handle it
        roll, pitch, yaw, thr = self._arb_sticks(t)
        moved = pilot_took_over(roll, pitch, yaw, thr, self._arb_throttle0, self.settings.arbiter)
        if moved:
            self._arb_hand_to_pilot(moved)
            return
        ok, reason = health(self._arb_inputs(t), self.settings.arbiter)
        now = time.monotonic()
        if ok:
            self._arb_fail_since = None
            return
        if self._arb_fail_since is None:
            self._arb_fail_since = now
        elif now - self._arb_fail_since >= HEALTH_FAIL_DEBOUNCE_S:
            self._arb_abort(reason)

    def _arb_stop_gcs_motion(self) -> None:
        if self.worker:
            self.worker.stop_setpoints()
        try:
            self.offboard_pump_timer.stop()
        except AttributeError:
            pass
        self._arb_active = False

    def _arb_hand_to_pilot(self, why: str) -> None:
        self._arb_stop_gcs_motion()
        self._cmd_mode("POSCTL")
        self._arb_set(PILOT, why)
        self._arb_say(f"PILOT TAKEOVER ({why}): GCS commands stopped, POSCTL. "
                      "Click GCS CTRL to give control back to the GCS.", toast_colour="#d29922")

    def _arb_abort(self, reason: str) -> None:
        """The estimator stopped being trustworthy while the GCS was flying: stop commanding and
        let PX4 hold (Hold needs a global position) or land."""
        self._arb_stop_gcs_motion()
        amsl, age = self.last_telemetry.amsl_reading()
        mode = "AUTO.LOITER" if (amsl is not None and age is not None and age <= 3.0) else "AUTO.LAND"
        self._cmd_mode(mode)
        self._arb_set(BLOCKED, reason)
        self._arb_say(f"GCS CONTROL ABORTED: {reason}. Commanded {mode}.")

    # ── status chip ──
    def _arb_set(self, state: str, reason: str = "") -> None:
        self._arb_state, self._arb_reason = state, reason
        self._arb_refresh_chip(self.last_telemetry)

    def _arb_refresh_chip(self, t) -> None:
        chip = getattr(self, "chip_gcs_ctrl", None)
        if chip is None:
            return
        if not self._arb_on:
            text, kind, tip = "GCS CTRL: OFF", "warn", "Arbitration disabled in Configuration."
        elif self._arb_state == PILOT:
            text, kind, tip = "GCS CTRL: PILOT", "warn", f"{self._arb_reason}. Click to hand control back."
        else:
            ok, reason = health(self._arb_inputs(t), self.settings.arbiter)
            if self._arb_state == BLOCKED and ok and not self._arb_active:
                self._arb_state = READY       # a blocked state clears once the estimator recovers
            text, kind, tip = (("GCS CTRL: READY", "ok", reason) if ok else
                               ("GCS CTRL: BLOCKED", "bad", reason))
        if chip.text() != text:
            chip.setText(text)
            chip.setProperty("state", kind)
            chip.style().unpolish(chip)
            chip.style().polish(chip)
        chip.setToolTip(tip)

    def _arb_chip_clicked(self) -> None:
        if self._arb_state == PILOT:
            self._arb_set(READY)
            self._arb_say("GCS control re-enabled by the operator.", error=False,
                          toast_colour="#1f6feb")

    # ── CHECK ORIGIN ──
    def _check_origin(self) -> None:
        if not self.worker or not self.worker.isRunning():
            self._arb_say("Cannot check origin: not connected.")
            return
        self.worker.request_global_origin()
        self.btn_check_origin.setText("ORIGIN: ASKING...")
        QTimer.singleShot(int(ORIGIN_ANSWER_S * 1000) + 200, self._report_origin)

    def _report_origin(self) -> None:
        t = self.last_telemetry
        st = t.origin_status(answer_window_s=ORIGIN_ANSWER_S)
        if st == "set":
            msg = (f"PX4 ACK: global origin {t.origin_lat:.6f}, {t.origin_lon:.6f}, "
                   f"{t.origin_alt:.1f} m - Takeoff/Hold/Land can navigate")
            self.btn_check_origin.setText("ORIGIN: OK")
            self.btn_check_origin.setProperty("state", "ok")
            self._arb_say(msg, error=False, toast_colour="#2ea043")
        else:
            msg = ("PX4 reports NO global origin - GCS takeoff will use an OFFBOARD climb; "
                   "Hold/missions unavailable (is the Jetson injector running?)")
            self.btn_check_origin.setText("ORIGIN: NONE")
            self.btn_check_origin.setProperty("state", "bad")
            self._arb_say(msg)
        self.btn_check_origin.setToolTip(msg)
        self.btn_check_origin.style().unpolish(self.btn_check_origin)
        self.btn_check_origin.style().polish(self.btn_check_origin)
