"""
================================================================================
MODULE: controllers/alarm_control.py
PURPOSE: Feeds the standing alarm list from telemetry and video health
================================================================================

Mixed into DroneGCSMainWindow. Audio alerts (core/audio.py) are edge-triggered
and speak once; this is level-triggered and *standing*: each condition holds an
alarm for exactly as long as it is true, so the strip always answers "what is
wrong right now". It adds no sound of its own - audio already covers link,
battery and vision - only a console line (and a toast for CRITICAL) the moment
an alarm first appears or escalates.
================================================================================
"""

from __future__ import annotations

from core.alarms import AlarmLevel
from core.telemetry import TelemetrySnapshot
from core.video_health import FROZEN, NO_SIGNAL


class AlarmControlMixin:
    """Telemetry/video -> AlarmManager, and the ACK handlers for the banner."""

    def _announce_alarm(self, level: AlarmLevel, text: str) -> None:
        if level >= AlarmLevel.CRITICAL:
            self.console.log_error(f"ALARM: {text}")
            self.toast.show_message(text, "#da3633", 4000)
        else:
            self.console.log_warning(f"ALARM: {text}")

    def _set_alarm(self, key: str, active: bool, level: AlarmLevel, text: str,
                   detail: str = "") -> None:
        if self.alarms.set_condition(key, active, level, text, detail):
            self._announce_alarm(level, text)

    def _update_alarms(self, t: TelemetrySnapshot) -> None:
        if t.connected:
            self._ever_connected = True
        # Before the first heartbeat "not connected" is just startup, not a lost link.
        self._set_alarm("link", self._ever_connected and not t.connected,
                        AlarmLevel.CRITICAL, "Link lost",
                        "No telemetry from the vehicle. Check the network and the Radxa.")

        telemetry_keys = ("battery", "vision", "position")
        if not t.connected:
            # Everything below describes a vehicle we are no longer hearing from.
            for k in telemetry_keys:
                self.alarms.clear(k)
            self.alarm_banner.refresh()
            return

        alerts = self.settings.alerts
        pct = t.battery_percent
        if 0 < pct <= alerts.batt_crit_pct:
            self._set_alarm("battery", True, AlarmLevel.CRITICAL, f"Battery critical: {pct}%",
                            "Land now.")
        elif 0 < pct <= alerts.batt_warn_pct:
            self._set_alarm("battery", True, AlarmLevel.WARN, f"Battery low: {pct}%",
                            "Plan to land soon.")
        else:
            self.alarms.clear("battery")

        # Airborne-only, as in the audio alerts: on the bench these are constant.
        airborne = t.is_airborne
        vio_ok = t.d435i_vio_health or t.ekf2_vision_fused
        self._set_alarm("vision", airborne and not vio_ok, AlarmLevel.CRITICAL,
                        "Vision lost while airborne",
                        "Position hold may drift. Be ready to take manual control.")
        self._set_alarm("position", airborne and t.position_stale, AlarmLevel.CRITICAL,
                        "Position feed lost while airborne",
                        "No local position from the vehicle.")
        self.alarm_banner.refresh()

    def _on_video_health(self, state: str, text: str) -> None:
        dead = state in (FROZEN, NO_SIGNAL)
        # The title is fixed and the elapsed time is the banner's running timer;
        # baking "no frame for N s" into the text froze it at the raise-time value.
        self._set_alarm("video", dead, AlarmLevel.WARN, f"Video feed: {state}",
                        "Check the stream URL and that the camera streamer is running.")
        self.alarm_banner.refresh()

    def _on_alarm_ack_all(self) -> None:
        n = self.alarms.acknowledge_all()
        if n:
            self.console.log_info(f"Acknowledged {n} alarm(s).")
        self.alarm_banner.refresh()

    def _on_alarm_ack(self, key: str) -> None:
        self.alarms.acknowledge(key)
        self.alarm_banner.refresh()
