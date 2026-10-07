"""
================================================================================
MODULE: controllers/preflight_control.py
PURPOSE: Feeds the preflight checklist, gates ARM on it, and saves parameters
================================================================================

Mixed into DroneGCSMainWindow. Jobs:
  * once a second, gather the facts the station already holds and show the
    checklist in the left rail under "Parameters" (core/preflight.py decides what
    each line's colour is). THE CHECKLIST ONLY INFORMS: it never blocks, delays or
    questions ARM, takeoff, or any other command. The flight controller's own
    arming checks are the real safety; this is a status display;
  * 'Save to flash' on the Parameters tab: a guarded, disarmed-only request that
    tells the flight controller to store its current parameters.
Radxa watchdog reports arrive here too and feed the alarm list and the checklist.
================================================================================
"""

from __future__ import annotations

import time
from typing import Optional

from core.alarms import AlarmLevel
from core.preflight import PreflightInputs, evaluate, summary
from core.radxa_status import RadxaStatus

PREFLIGHT_REFRESH_S = 1.0
# A Radxa report older than this is as good as none.
RADXA_REPORT_MAX_AGE_S = 20.0


class PreflightControlMixin:

    # ── checklist ───────────────────────────────────────────────────

    def _preflight_inputs(self) -> PreflightInputs:
        t = self.last_telemetry
        a = self.settings.alerts
        radxa = self._radxa_status_fresh()
        return PreflightInputs(
            connected=t.connected, heartbeat_age_s=t.heartbeat_age, armed=t.armed,
            battery_pct=t.battery_percent, batt_warn_pct=a.batt_warn_pct, batt_crit_pct=a.batt_crit_pct,
            vision_ok=bool(t.d435i_vio_health or t.ekf2_vision_fused), position_stale=t.position_stale,
            video_state=getattr(self, "_video_state", "IDLE"),
            map_seen=self.page_slam.canvas.last_map_time > 0, map_stale=self.page_slam.canvas.is_map_stale(),
            params_loaded=self.page_params.has_data(),
            radxa_state=None if radxa is None else radxa.state,
            radxa_detail="" if radxa is None else radxa.detail(),
            heading_confirmed=self.sidebar.checklist.heading_confirmed())

    def _refresh_preflight(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - getattr(self, "_preflight_at", 0.0) < PREFLIGHT_REFRESH_S:
            return
        self._preflight_at = now
        inp = self._preflight_inputs()
        # The heading has to be fixed again after every tracking reset.
        if self._preflight_vision_was_ok and not inp.vision_ok:
            self.sidebar.checklist.clear_heading()
            inp.heading_confirmed = False
        self._preflight_vision_was_ok = inp.vision_ok
        checks = evaluate(inp)
        ready, text = summary(checks, armed=inp.armed)
        self._preflight_checks = checks
        self.sidebar.checklist.update_checks(checks, ready, text)

    # ── save parameters to flash ────────────────────────────────────

    def _request_save_params(self) -> None:
        if not self._require_link("save parameters"):
            return
        if self.last_telemetry.armed:
            msg = "Save to flash refused: the vehicle is armed. Disarm first."
            self.console.log_error(msg)
            self.toast.show_message(msg, "#da3633", 4000)
            return
        self.confirm_bar.request(
            "save_params", "SAVE PARAMETERS TO FLASH", danger=False,
            detail="Stores the flight controller's CURRENT parameters in its flash so they survive a "
                   "power cycle. No value is changed.",
            confirm_text="Slide to save", anchor=self.page_params.btn_save_flash)

    def _cmd_save_params(self) -> None:
        if self.last_telemetry.armed:           # re-checked at dispatch, as every guarded action is
            self.console.log_error("Save to flash refused: the vehicle is armed.")
            return
        if self.worker and self.worker.save_params_to_flash():
            self.console.log_cmd("Dispatching PREFLIGHT_STORAGE (save parameters to flash)...")
            self.toast.show_message("Saving parameters to flash...", "#238636")
        else:
            self.console.log_error("Could not send the save-to-flash command (not connected).")

    # ── Radxa watchdog ──────────────────────────────────────────────

    def _on_radxa_status(self, status: Optional[RadxaStatus]) -> None:
        self._radxa_status = status
        self._radxa_status_at = time.monotonic()
        self._update_radxa_alarms()

    def _radxa_status_fresh(self) -> Optional[RadxaStatus]:
        st = getattr(self, "_radxa_status", None)
        if st is None or time.monotonic() - getattr(self, "_radxa_status_at", 0.0) > RADXA_REPORT_MAX_AGE_S:
            return None
        return st

    def _update_radxa_alarms(self) -> None:
        st = self._radxa_status_fresh()
        down = st is not None and (st.state == "down" or st.gave_up)
        self._set_alarm("radxa", down, AlarmLevel.CRITICAL, "Radxa pipeline down",
                        (st.detail() if st else "") or "The camera / SLAM pipeline on the Radxa is not running.")
        stopped = st is not None and st.state == "stopped"
        self._set_alarm("radxa_stopped", stopped, AlarmLevel.WARN, "Radxa pipeline is stopped",
                        "The camera / SLAM pipeline is not running on the Radxa. Start it before flying: "
                        "sudo systemctl start drone-pipeline (or ./camera.sh).")
        recent = st.recent_restart(time.time()) if st is not None else None
        self._set_alarm("radxa_restart", recent is not None and not down, AlarmLevel.WARN,
                        "Radxa pipeline was restarted",
                        f"Reason: {recent.get('reason', 'unknown')}. Check the map and camera are back."
                        if recent else "")
        self._set_alarm("radxa_disk", st is not None and st.disk_low(), AlarmLevel.WARN, "Radxa disk almost full",
                        "Free space is low. The log cleanup runs daily; check /home/radxa on the Radxa.")
        self.alarm_banner.refresh()
