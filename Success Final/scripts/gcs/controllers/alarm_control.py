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

import time

from core.alarms import AlarmLevel
from core.telemetry import TelemetrySnapshot
from core.video_health import FROZEN, NO_SIGNAL


# A UI freeze shorter than this is ordinary jitter, not something to alarm on.
UI_STALL_ALARM_S = 0.5
# How long a UI-stall alarm stays up after the last freeze. Level-triggered like
# the others, but a freeze is over the moment it is reported, so it is held.
UI_STALL_HOLD_S = 10.0


class AlarmControlMixin:
    """Telemetry/video -> AlarmManager, and the ACK handlers for the banner."""

    def _note_ui_stall(self, gap_s: float) -> None:
        """Called from the UI tick whenever the stall monitor reports a freeze."""
        if gap_s >= UI_STALL_ALARM_S:
            self._ui_stall_until = time.monotonic() + UI_STALL_HOLD_S
            self._ui_stall_gap_s = gap_s

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

        self._update_station_alarms()

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

    def _update_station_alarms(self) -> None:
        """Alarms about this station's own health, not the vehicle's - they hold
        whether or not telemetry is flowing.

        MAP STALLED: the occupancy grid was arriving and has stopped for longer
        than alerts.map_stall_s. The canvas already dims itself; this puts the
        same fact in the standing alarm list, where it is visible from every tab.
        Nothing is raised before the first map (nothing has stalled yet) or after
        a deliberate Reset Map (which clears the map clock).

        UI STALLED: the 30 Hz UI tick overran, i.e. something blocked the cockpit
        while telemetry kept arriving unseen.
        """
        canvas = self.page_slam.canvas
        stale = canvas.is_map_stale()
        self._set_alarm("map", stale, AlarmLevel.WARN, "Map stalled",
                        "No map update from the Radxa. Check the SLAM pipeline and the map bridge; "
                        "the map on screen may be out of date.")
        frozen = time.monotonic() < getattr(self, "_ui_stall_until", 0.0)
        gap = getattr(self, "_ui_stall_gap_s", 0.0)
        self._set_alarm("ui", frozen, AlarmLevel.WARN, "Screen froze",
                        f"The station stopped redrawing for {gap:.1f} s. What you see may be a moment old.")

    def _on_video_health(self, state: str, text: str) -> None:
        self._video_state = state          # also read by the preflight checklist
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
