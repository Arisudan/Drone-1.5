"""
================================================================================
MODULE: controllers/flight_commands.py
PURPOSE: Flight command handlers (arm/disarm/takeoff/yaw/altitude/mode/kill)
================================================================================

Mixed into DroneGCSMainWindow. ``_request_*`` are the operator-facing entry
points (they route risky actions through the guided-confirm bar); ``_cmd_*``
are the executors that talk to MAVLinkWorker. Moved out of drone_gcs.py
unchanged - they read window state through ``self``.
================================================================================
"""

from __future__ import annotations

from PyQt5.QtCore import QTimer

from core.audio import Severity




class FlightCommandsMixin:
    """Flight command handlers: operator requests -> guided confirm -> MAVLink."""

    def _request_arm(self):
        """ARM used to dispatch on a single click, with no confirmation at all -
        despite this module's own docstring claiming a dual-stage modal. It now
        goes through the same slide-to-confirm gesture as every other guarded
        action. Bench force-arm remains terminal-only (`arm force`)."""
        if not self._require_link("arm"):
            return
        self.confirm_bar.request(
            "arm", "ARM MOTORS", danger=True,
            detail="Propellers will be live. Confirm the area is clear.",
            confirm_text="Slide to arm", anchor=self.btn_arm)

    def _request_disarm(self):
        """Disarm. Airborne, this is redirected to AUTO.LAND by _cmd_disarm -
        the confirmation says so, because 'disarm' and 'land' are different
        enough that the operator should know which one they are getting."""
        if not self._require_link("disarm"):
            return
        if self.last_telemetry.needs_controlled_landing:
            self.confirm_bar.request(
                "disarm", "DISARM (AIRBORNE)", danger=True,
                detail="Vehicle is off the ground: this commands AUTO.LAND and "
                       "disarms on touchdown. Use EMERGENCY KILL for an "
                       "immediate cutoff.",
                confirm_text="Slide to land and disarm", anchor=self.btn_disarm)
        else:
            self.confirm_bar.request(
                "disarm", "DISARM", danger=True,
                detail="Vehicle is on the ground.",
                confirm_text="Slide to disarm", anchor=self.btn_disarm)

    def _request_takeoff(self):
        """Takeoff, with the altitude on a slider bounded by settings.limits."""
        if not self._require_link("takeoff"):
            return
        try:
            seed = float(self.ent_takeoff_alt.text().strip())
        except ValueError:
            seed = self.settings.slam.cruise_altitude_m
        seed = max(self.TAKEOFF_ALT_MIN_M, min(self.TAKEOFF_ALT_MAX_M, seed))
        self.confirm_bar.request(
            "takeoff", "TAKEOFF", value_label="Altitude AGL",
            vmin=self.TAKEOFF_ALT_MIN_M, vmax=self.TAKEOFF_ALT_MAX_M,
            vinit=seed, unit="m", step=0.1,
            detail="Vehicle must already be armed.",
            confirm_text="Slide to take off", anchor=self.btn_takeoff)

    def _request_yaw(self):
        if not self._require_link("rotate yaw"):
            return
        try:
            seed = float(self.ent_yaw.text().strip())
        except ValueError:
            seed = 90.0
        self.confirm_bar.request(
            "yaw", "ROTATE YAW", value_label="Rotation",
            vmin=-180.0, vmax=180.0, vinit=max(-180.0, min(180.0, seed)),
            unit="\u00b0", step=5.0,
            detail="Relative rotation from the current heading. Requires "
                   "OFFBOARD and an airborne vehicle.",
            confirm_text="Slide to rotate", anchor=self.btn_yaw)

    def _request_change_alt(self):
        """Climb or descend in place to a new altitude."""
        if not self._require_link("change altitude"):
            return
        current = abs(self.last_telemetry.altitude)
        seed = max(self.TAKEOFF_ALT_MIN_M, min(self.TAKEOFF_ALT_MAX_M,
                                               current if current > 0.05 else 1.0))
        self.confirm_bar.request(
            "change_alt", "CHANGE ALTITUDE", value_label="Target AGL",
            vmin=self.TAKEOFF_ALT_MIN_M, vmax=self.TAKEOFF_ALT_MAX_M,
            vinit=seed, unit="m", step=0.1,
            detail=f"Currently {current:.2f} m AGL. Holds the present position "
                   f"and changes height only. Requires OFFBOARD.",
            confirm_text="Slide to change altitude", anchor=self.btn_change_alt)

    def _request_kill(self):
        """Emergency kill. The confirmation is a drag, not a modal with a
        default button one Return keypress away."""
        self.confirm_bar.request(
            "kill", "EMERGENCY MOTOR KILL", danger=True,
            detail="Cuts all motor outputs immediately. If the vehicle is "
                   "airborne it will fall. There is no recovery from this.",
            confirm_text="Slide to cut motors", anchor=self.btn_kill)

    def _request_abort_path(self):
        if not self.path_in_progress and not self.path_paused:
            self.console.log_info("No path is executing - nothing to abort.")
            return
        self.confirm_bar.request(
            "abort_path", "ABORT PATH", danger=True,
            detail="Stops the path and commands AUTO.LAND.",
            confirm_text="Slide to abort and land",
            anchor=self.page_slam.btn_abort_path)

    def _require_link(self, what: str) -> bool:
        """Refuse to even offer a confirmation with no link. Showing a confirm
        bar for a command that cannot be sent trains the operator to confirm
        things that do nothing."""
        if self.worker and self.worker.isRunning():
            return True
        msg = f"Cannot {what}: not connected to vehicle"
        self.console.log_error(msg)
        self.page_terminal.log_error(msg)
        self.toast.show_message(msg, "#da3633")
        return False

    def _on_guided_confirmed(self, action: str, value: float):
        """Single dispatch point for every confirmed guided action.

        Each branch calls the same _cmd_* method the direct control always
        called, so every interlock in those methods still applies. Confirmation
        gates the request; it does not replace the checks.
        """
        if action == "arm":
            self._cmd_arm(force=False)
        elif action == "disarm":
            self._cmd_disarm()
        elif action == "takeoff":
            self.ent_takeoff_alt.setText(f"{value:.2f}")
            self._cmd_takeoff(value)
        elif action == "yaw":
            self.ent_yaw.setText(f"{value:.1f}")
            self._cmd_yaw(value)
        elif action == "change_alt":
            self._cmd_change_altitude(value)
        elif action == "kill":
            self._cmd_kill()
        elif action == "abort_path":
            self._on_abort_path_requested()

    def _on_guided_cancelled(self, action: str):
        self.console.log_info(f"{action.replace('_', ' ').upper()} cancelled.")

    def _cmd_change_altitude(self, altitude_m: float):
        """Hold the current horizontal position and move to a new altitude.

        Uses the same OFFBOARD setpoint path as a waypoint, with the north/east
        components pinned to where the vehicle already is, so it is a pure climb
        or descent rather than a move that happens to change height.
        """
        if not self._require_link("change altitude"):
            return
        t = self.last_telemetry
        if not t.is_airborne:
            msg = "Cannot change altitude: not airborne - arm and take off first."
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Cannot change alt: not airborne", "#da3633", 4000)
            return
        if t.flight_mode != "OFFBOARD":
            msg = (f"Cannot change altitude: current mode is "
                   f"{t.flight_mode or 'UNKNOWN'}, not OFFBOARD. Switch to "
                   "OFFBOARD manually (Mode dropdown -> SET MODE) first.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Not in OFFBOARD - switch manually first",
                                    "#da3633", 4000)
            return

        target_z = -abs(altitude_m)
        self.cruise_z = target_z
        self.console.log_cmd(
            f"Changing altitude to {abs(target_z):.2f} m AGL "
            f"(holding N {t.x:+.2f}, E {t.y:+.2f})...")
        self.page_terminal.log_cmd(f"Changing altitude to {abs(target_z):.2f} m AGL...")
        self.worker.move_to_waypoint(t.x, t.y, z=target_z, yaw_deg=t.heading)
        self.offboard_pump_timer.start()
        self.toast.show_message(f"Altitude -> {abs(target_z):.2f} m", "#1f6feb")
        self.exec_tracker.start_tracking(
            f"altitude {abs(target_z):.2f}m", t.x, t.y, t.z,
            cur_armed=t.armed, cur_mode=t.flight_mode,
            target_dist=abs(target_z - t.z))

    def _cmd_arm_from_ui(self):
        # The ARM button is always a normal arm. Bench force-arm is reachable
        # only by typing `arm force` in the flight terminal.
        self._cmd_arm(force=False)

    def _cmd_arm(self, force: bool = False):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot arm: Not connected to vehicle")
            self.page_terminal.log_error("Cannot arm: Not connected to vehicle")
            return
        self._last_arm_was_forced = force
        label = "FORCE ARM (BENCH, param2=21196)" if force else "ARM (NORMAL)"
        self.console.log_cmd(f"Dispatching {label}...")
        self.page_terminal.log_cmd(f"Dispatching {label}...")
        self.worker.arm(force=force)
        self.toast.show_message(f"Dispatching: {label}", "#d29922" if force else "#238636")
        self.exec_tracker.start_tracking(
            "arm force" if force else "arm",
            self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode
        )

    def _cmd_disarm(self):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot disarm: Not connected to vehicle")
            self.page_terminal.log_error("Cannot disarm: Not connected to vehicle")
            return

        # Completely reset active navigation and waypoint pump
        self._begin_path_generation()
        self.offboard_pump_timer.stop()
        self.path_in_progress = False
        self.path_paused = False
        self.path_awaiting_climb = False
        self.climb_start_time = 0.0
        self.active_waypoints = []
        self.current_wpt_idx = 0
        self.page_slam.set_executing_state(False, paused=False)
        self.page_slam.canvas.clear_goal()

        t = self.last_telemetry
        if t.needs_controlled_landing:
            # `disarm` while genuinely airborne must never be an instant
            # motor cutoff - that free-falls the vehicle. Redirect to PX4's
            # own AUTO.LAND (a real, tested controlled descent using the
            # rangefinder/optical-flow height fusion already enabled -
            # EKF2_RNG_CTRL/EKF2_OF_CTRL), and only send the real disarm once
            # landed_state confirms touchdown (see _on_telemetry_updated).
            # `kill` remains the true, unconditional emergency cutoff.
            h = t.height_over_floor()
            where = f" ({h:.2f} m above the floor)" if h is not None else ""
            msg = (f"DISARM requested while off the ground{where} - redirecting to "
                   "AUTO.LAND for a safe controlled descent (will disarm automatically "
                   "on touchdown). Use KILL for an immediate cutoff instead.")
            self.console.log_warning(msg)
            self.page_terminal.log_warning(msg)
            self.toast.show_message("Airborne - landing safely, will disarm on touchdown", "#d29922", 5000)
            self.worker.set_mode("AUTO.LAND")
            self._pending_autodisarm_after_land = True
            self.exec_tracker.start_tracking(
                "land-then-disarm", t.x, t.y, t.z,
                cur_armed=t.armed, cur_mode=t.flight_mode
            )
            # PX4 can refuse AUTO.LAND (e.g. no valid position estimate). Do not
            # wait silently for a landing that will never start.
            QTimer.singleShot(self.LAND_VERIFY_MS, self._verify_land_mode)
            return

        self.console.log_cmd("Dispatching DISARM...")
        self.page_terminal.log_cmd("Dispatching DISARM...")
        # Not forced: if the vehicle is in fact flying, PX4 itself refuses a
        # normal disarm. Forcing here would switch that safety net off.
        self.worker.disarm(force=False)
        self.toast.show_message("Dispatching: DISARM", "#da3633")
        self.exec_tracker.start_tracking(
            "disarm", t.x, t.y, t.z,
            cur_armed=t.armed, cur_mode=t.flight_mode
        )

    #: How long to wait for PX4 to confirm AUTO.LAND before warning the operator.
    LAND_VERIFY_MS = 2500

    def _verify_land_mode(self):
        """Called shortly after a disarm was redirected to AUTO.LAND. If the
        vehicle is still armed and not in AUTO.LAND, PX4 refused the mode:
        say so loudly and leave the motors alone - never fall back to cutting
        power in the air."""
        if not getattr(self, "_pending_autodisarm_after_land", False):
            return
        t = self.last_telemetry
        if not t.armed:
            self._pending_autodisarm_after_land = False
            return
        if t.flight_mode == "AUTO.LAND":
            return
        self._pending_autodisarm_after_land = False
        msg = (f"AUTO.LAND was NOT accepted (mode is {t.flight_mode}) - the vehicle is "
               "still armed and NOT landing. Retry DISARM, fly it down manually, "
               "or use EMERGENCY KILL.")
        self.console.log_error(msg)
        self.page_terminal.log_error(msg)
        self.toast.show_message("LAND refused - still armed, not landing", "#da3633", 8000)

    def _cmd_takeoff_from_ui(self):
        try:
            alt = float(self.ent_takeoff_alt.text().strip())
        except ValueError:
            alt = 1.0
        self._cmd_takeoff(alt)

    def _cmd_takeoff(self, altitude: float = 1.0):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot takeoff: Not connected")
            self.page_terminal.log_error("Cannot takeoff: Not connected")
            return

        t = self.last_telemetry
        if not t.armed:
            msg = "Cannot takeoff: drone is disarmed - arm first."
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Cannot takeoff: disarmed", "#da3633", 4000)
            return
        if not (self.TAKEOFF_ALT_MIN_M <= altitude <= self.TAKEOFF_ALT_MAX_M):
            msg = (f"Cannot takeoff: {altitude:.2f}m is outside the allowed range "
                   f"[{self.TAKEOFF_ALT_MIN_M:.1f}, {self.TAKEOFF_ALT_MAX_M:.1f}]m.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Takeoff altitude out of range", "#da3633", 4000)
            return

        if t.is_airborne:
            # Already flying - "takeoff <alt>" here means "go to and hold at
            # this altitude," which may mean ascending OR descending from the
            # current height. PX4's native NAV_TAKEOFF is a ground-takeoff
            # maneuver and doesn't handle descending, so use a pure-Z OFFBOARD
            # position-hold setpoint instead (same mechanism move/yaw use),
            # which naturally auto-holds once the target altitude is reached.
            # Mode switching is never done silently on the operator's behalf -
            # OFFBOARD must already be active (set explicitly via the Mode
            # dropdown + SET MODE) before this will do anything.
            if t.flight_mode != "OFFBOARD":
                msg = (f"Cannot hold altitude: current mode is {t.flight_mode or 'UNKNOWN'}, not OFFBOARD. "
                       "Switch to OFFBOARD manually (Mode dropdown -> SET MODE) first.")
                self.console.log_error(msg)
                self.page_terminal.log_error(msg)
                self.toast.show_message("Not in OFFBOARD - switch manually first", "#da3633", 4000)
                return
            self.console.log_cmd(f"Already airborne - repositioning to altitude {altitude:.1f}m and holding...")
            self.page_terminal.log_cmd(f"Already airborne - repositioning to altitude {altitude:.1f}m and holding...")
            self.worker.move_to_waypoint(t.x, t.y, z=-abs(altitude))
            self.toast.show_message(f"Altitude hold: {altitude:.1f}m", "#1f6feb")
        else:
            self.console.log_cmd(f"Initiating takeoff to {altitude:.1f}m...")
            self.page_terminal.log_cmd(f"Initiating takeoff to {altitude:.1f}m...")
            self.worker.takeoff(altitude)
            self.toast.show_message(f"Takeoff Initiated ({altitude:.1f}m)", "#1f6feb")

        self.exec_tracker.start_tracking(
            f"takeoff {altitude}", t.x, t.y, t.z,
            cur_armed=t.armed, cur_mode=t.flight_mode
        )

    def _cmd_move(self, dx: float, dy: float, dz: float):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot move: Disconnected")
            self.page_terminal.log_error("Cannot move: Disconnected")
            return
        if not self.last_telemetry.is_airborne:
            msg = "Cannot move: not airborne - arm and takeoff first."
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Cannot move: not airborne", "#da3633", 4000)
            return
        if max(abs(dx), abs(dy), abs(dz)) > self.MOVE_MAX_DELTA_M:
            msg = f"Cannot move: displacement exceeds the {self.MOVE_MAX_DELTA_M:.1f}m safety bound per command."
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Move rejected: too large", "#da3633", 4000)
            return

        # OFFBOARD is never engaged silently on the operator's behalf - it
        # must already be active (Mode dropdown -> SET MODE) before a move
        # will be dispatched.
        if self.last_telemetry.flight_mode != "OFFBOARD":
            curr_mode = self.last_telemetry.flight_mode or "UNKNOWN"
            msg = (f"Cannot move: current mode is {curr_mode}, not OFFBOARD. "
                   "Switch to OFFBOARD manually (Mode dropdown -> SET MODE) first.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Not in OFFBOARD - switch manually first", "#da3633", 4000)
            return

        self.console.log_cmd(f"Dispatching translation: dx={dx:+.2f}m, dy={dy:+.2f}m, dz={dz:+.2f}m")
        self.page_terminal.log_cmd(f"Dispatching translation: dx={dx:+.2f}m, dy={dy:+.2f}m, dz={dz:+.2f}m")
        self.worker.move_delta(dx, dy, dz)
        self.toast.show_message(f"Move: ({dx:+.1f}, {dy:+.1f}, {dz:+.1f})m", "#1f6feb")
        self.exec_tracker.start_tracking(
            f"move {dx:.2f} {dy:.2f} {dz:.2f}",
            self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode
        )

    def _cmd_yaw_from_ui(self):
        try:
            angle = float(self.ent_yaw.text().strip())
        except ValueError:
            angle = 90.0
        self._cmd_yaw(angle)

    def _cmd_yaw(self, angle_deg: float):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot rotate yaw: Disconnected")
            self.page_terminal.log_error("Cannot rotate yaw: Disconnected")
            return
        if not self.last_telemetry.is_airborne:
            msg = "Cannot rotate yaw: not airborne - arm and takeoff first."
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Cannot yaw: not airborne", "#da3633", 4000)
            return

        # OFFBOARD is never engaged silently on the operator's behalf - it
        # must already be active (Mode dropdown -> SET MODE) before a yaw
        # rotation will be dispatched.
        if self.last_telemetry.flight_mode != "OFFBOARD":
            curr_mode = self.last_telemetry.flight_mode or "UNKNOWN"
            msg = (f"Cannot rotate yaw: current mode is {curr_mode}, not OFFBOARD. "
                   "Switch to OFFBOARD manually (Mode dropdown -> SET MODE) first.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Not in OFFBOARD - switch manually first", "#da3633", 4000)
            return

        self.console.log_cmd(f"Rotating yaw by {angle_deg:+.1f}°...")
        self.page_terminal.log_cmd(f"Rotating yaw by {angle_deg:+.1f}°...")
        self.worker.rotate_yaw(angle_deg)
        self.toast.show_message(f"Yaw Rotate: {angle_deg:+.1f}°", "#1f6feb")
        self.exec_tracker.start_tracking(
            f"yaw {angle_deg:.1f}",
            self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode,
            cur_heading=self.last_telemetry.heading
        )

    def _cmd_set_mode_from_ui(self):
        selected_mode = self.combo_modes.currentText().strip()
        self._cmd_mode(selected_mode)

    def _cmd_mode(self, mode_name: str):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error(f"Cannot set mode {mode_name}: Not connected")
            self.page_terminal.log_error(f"Cannot set mode {mode_name}: Not connected")
            return
        self.console.log_cmd(f"Switching mode to {mode_name}...")
        self.page_terminal.log_cmd(f"Switching mode to {mode_name}...")
        self.worker.set_mode(mode_name)
        self.toast.show_message(f"Mode set: {mode_name}", "#1f6feb")
        if "LAND" in mode_name.upper():
            self.exec_tracker.start_tracking(
                "land", self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
                cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode
            )
        elif "RTL" in mode_name.upper():
            self.exec_tracker.start_tracking(
                "rtl", self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
                cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode
            )

    def _cmd_kill(self):
        """Execute the kill. Confirmation is the caller's job.

        The Yes/No QMessageBox that used to live here is gone: its default
        button was one Return keypress from cutting the motors of a flying
        aircraft. _request_kill now gates this behind a deliberate drag. The
        terminal's `kill` command reaches this directly, which is the documented
        behaviour of a typed emergency command.
        """
        # Completely reset active navigation and waypoint pump
        self.offboard_pump_timer.stop()
        self.path_in_progress = False
        self.path_paused = False
        self.path_awaiting_climb = False
        self.climb_start_time = 0.0
        self.active_waypoints = []
        self.current_wpt_idx = 0
        self.page_slam.set_executing_state(False, paused=False)
        self.page_slam.canvas.clear_goal()
        self.page_slam.progress.set_state("ABORTED")
        self.page_motors.stop_motor_tests()

        if self.worker and self.worker.isRunning():
            self.worker.emergency_kill()
            self.console.log_error("!!! EMERGENCY MOTOR KILL EXECUTED !!!")
            self.page_terminal.log_error("!!! EMERGENCY MOTOR KILL EXECUTED !!!")
            self.toast.show_message("EMERGENCY KILL SENT", "#da3633", 5000)
            self.audio.say("Emergency kill", Severity.ALARM, key="kill")
