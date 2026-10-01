"""
================================================================================
MODULE: controllers/mission_control.py
PURPOSE: Path/mission execution handlers (execute/pause/resume/abort, collision
         verdicts, detours, progress)
================================================================================

Mixed into DroneGCSMainWindow. Moved out of drone_gcs.py unchanged - they read
window state through ``self``.
================================================================================
"""

from __future__ import annotations

import math
import time

from core.audio import Severity
from core.telemetry import TelemetrySnapshot
from protocol.ros2_map_listener import SlamMapResetWorker




class MissionControlMixin:
    """Mission execution: path start/pause/resume/abort, collision and detour handling."""

    def _on_execute_path_requested(self, waypoints: list):
        """Called when user clicks '[ EXECUTE PATH ]' in Tactical SLAM tab."""
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot execute path: Not connected to drone")
            self.toast.show_message("Cannot execute: Disconnected", "#da3633")
            return

        if not waypoints:
            self.console.log_error("Cannot execute path: Waypoint list is empty")
            return

        # 1. Arm Status Interlock
        if not self.last_telemetry.armed:
            self.console.log_error(
                "❌ FLIGHT INTERLOCK REJECTED: Drone is DISARMED! "
                "Arm drone and confirm clear airspace before executing autonomous path."
            )
            self.page_terminal.log_error(
                "❌ FLIGHT INTERLOCK REJECTED: Drone is DISARMED! Arm drone before executing path."
            )
            self.toast.show_message("Cannot Execute: Drone is Disarmed!", "#da3633", 5000)
            return

        # 1b. OFFBOARD Mode Interlock - never engaged silently on the
        # operator's behalf; must already be active before a path is flown.
        if self.last_telemetry.flight_mode != "OFFBOARD":
            curr_mode = self.last_telemetry.flight_mode or "UNKNOWN"
            msg = (f"Cannot execute path: current mode is {curr_mode}, not OFFBOARD. "
                   "Switch to OFFBOARD manually (Mode dropdown -> SET MODE) first.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Not in OFFBOARD - switch manually first", "#da3633", 4000)
            return

        # 2. VIO / Position Lock Interlock
        if not self.last_telemetry.d435i_vio_health and not self.last_telemetry.ekf2_vision_fused:
            self.console.log_warning(
                "⚠️ FLIGHT WARNING: D435i Visual Odometry is not confirmed locked. "
                "Proceed with extreme caution in GPS-denied environment."
            )
            self.toast.show_message("Warning: Vision Odometry Degraded", "#d29922", 4000)

        # Remember the mission's stops so an in-flight detour can re-route
        # through the ones not yet reached. Taken from the canvas only when its
        # route ends where this path ends - "Fly here now" and other callers
        # pass their own single-goal path.
        stops = list(getattr(self.page_slam.canvas, "mission_stops", []) or [])
        end = waypoints[-1]
        if not (stops and math.hypot(stops[-1][0] - end[0], stops[-1][1] - end[1]) < 0.6):
            stops = [tuple(end)]
        self._mission_stops_remaining = [tuple(st) for st in stops]

        # 3. Ground Takeoff vs In-Air Transition Check
        selected_alt = self.page_slam.get_cruise_altitude()
        self.cruise_z = -abs(selected_alt)

        current_alt = self.last_telemetry.altitude
        if current_alt < 0.40:
            self.console.log_cmd(
                f"[TAKEOFF INTERLOCK] Vehicle is on ground (alt={current_alt:.2f}m). "
                f"Holding climb to cruise altitude {abs(self.cruise_z):.2f}m..."
            )
            self.page_terminal.log_cmd(
                f"[TAKEOFF INTERLOCK] Vehicle on ground. Initiating climb to {abs(self.cruise_z):.2f}m..."
            )
            self.pending_path_waypoints = list(waypoints)
            self.path_awaiting_climb = True
            self.climb_start_time = time.time()
            self.takeoff_hover_x = self.last_telemetry.x
            self.takeoff_hover_y = self.last_telemetry.y
            self.path_in_progress = True
            self.page_slam.set_executing_state(True, paused=False)

            self.worker.move_to_waypoint(
                self.takeoff_hover_x, self.takeoff_hover_y, z=self.cruise_z, yaw_deg=self.last_telemetry.heading
            )
            self.offboard_pump_timer.start()
            self.toast.show_message("Climbing to Cruise Alt...", "#1f6feb", 4000)
            return

        # Already airborne: proceed immediately
        self.console.log_cmd(
            f"[AIRBORNE] Executing path at cruise altitude {abs(self.cruise_z):.2f}m AGL ({len(waypoints)} waypoints)..."
        )
        self.page_terminal.log_cmd(
            f"[AIRBORNE] Executing path ({len(waypoints)} waypoints)..."
        )
        self._dispatch_path_start(waypoints)

    def _dispatch_path_start(self, waypoints: list):
        """Dispatches the first waypoint with tangent yaw alignment."""
        self._begin_path_generation()
        self.active_waypoints = list(waypoints)
        self.current_wpt_idx = 0
        self.path_in_progress = True
        self.path_paused = False
        self.page_slam.set_executing_state(True, paused=False)

        # Dispatch first waypoint with tangent heading
        first_wpt = self.active_waypoints[0]
        dx = first_wpt[0] - self.last_telemetry.x
        dy = first_wpt[1] - self.last_telemetry.y
        seg_dist = math.sqrt(dx * dx + dy * dy)
        yaw_deg = math.degrees(math.atan2(dy, dx)) if seg_dist > 0.05 else self.last_telemetry.heading
        self.current_target_yaw = yaw_deg

        self.worker.move_to_waypoint(first_wpt[0], first_wpt[1], z=self.cruise_z, yaw_deg=yaw_deg)
        self.offboard_pump_timer.start()  # Start 10 Hz continuous stream

        self.exec_tracker.start_tracking(
            "waypoint W1", self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            target_dist=seg_dist
        )
        self.console.log_cmd(
            f"Dispatching W1: ({first_wpt[0]:+.2f}m, {first_wpt[1]:+.2f}m) alt={-self.cruise_z:.2f}m yaw={yaw_deg:+.1f}°"
        )

    def _begin_path_generation(self) -> int:
        """Start a new path 'generation'.

        Collision verdicts and detour plans are computed on a background thread
        and can land after the path they were about has gone away. Each carries
        the generation it was requested under; a mismatch means the answer is
        about a path that no longer exists, and acting on it could halt a good
        path over an obstacle that is no longer ahead of the aircraft.
        """
        self._path_generation += 1
        self._collision_token = None
        self._detour_token = None
        return self._path_generation

    def _pump_active_waypoint_setpoint(self):
        """Continuously stream active waypoint setpoint at 10 Hz to satisfy PX4 500ms timeout."""
        if (
            self.path_in_progress
            and not self.path_paused
            and self.worker
            and self.worker.isRunning()
            and self.active_waypoints
            and self.current_wpt_idx < len(self.active_waypoints)
        ):
            wpt = self.active_waypoints[self.current_wpt_idx]
            _t0 = time.perf_counter()
            self.worker.move_to_waypoint(wpt[0], wpt[1], z=self.cruise_z, yaw_deg=self.current_target_yaw)
            # Heartbeat only on a setpoint that actually went out. Timing the
            # send matters as much as counting it: this runs on the GUI thread,
            # so a socket that starts blocking here stalls the pump AND the UI
            # together, and PX4 notices within 500 ms.
            self.pump_health.record_latency((time.perf_counter() - _t0) * 1000.0)
            self.pump_health.heartbeat()

    def _on_cruise_altitude_changed(self, alt_m: float):
        self.cruise_z = -abs(alt_m)
        self.console.log_info(f"[TACTICAL SLAM] Cruise altitude updated to {abs(alt_m):.1f}m AGL (z={self.cruise_z:.1f}m)")

    def _on_reset_map_requested(self):
        """User confirmed 'Reset Map' in the Tactical SLAM tab. Runs the reset on a
        background worker (ROS 2 service calls, TCP fallback) so the GUI stays
        responsive - this can take a few seconds if it has to fall through to the
        fallback path."""
        if self._reset_map_worker is not None and self._reset_map_worker.isRunning():
            return  # already in flight - the button is disabled meanwhile anyway

        self.page_slam.btn_reset_map.setEnabled(False)
        self.console.log_warning("[TACTICAL SLAM] Resetting SLAM map...")
        self.page_terminal.log_warning("[TACTICAL SLAM] Resetting SLAM map...")
        self.toast.show_message("Resetting SLAM map...", "#d29922", 4000)

        self._reset_map_worker = SlamMapResetWorker(self.map_listener.tcp_host, tcp_port=5765, parent=self)
        self._reset_map_worker.finished_result.connect(self._on_reset_map_result)
        self._reset_map_worker.start()

    def _on_reset_map_result(self, success: bool, message: str):
        if success:
            self.console.log_success(f"[TACTICAL SLAM] Map reset: {message}")
            self.page_terminal.log_success(f"[TACTICAL SLAM] Map reset: {message}")
            self.toast.show_message("SLAM map reset", "#3fb950", 3000)
            self.page_slam.clear_local_map_display()
        else:
            self.console.log_error(f"[TACTICAL SLAM] Map reset FAILED: {message}")
            self.page_terminal.log_error(f"[TACTICAL SLAM] Map reset FAILED: {message}")
            self.toast.show_message("Map reset failed - see console", "#f85149", 5000)
            # Map wasn't actually touched, so leave the display exactly as it was.

        # Re-enable unless armed state changed to True while the reset was in flight.
        self.page_slam.set_armed_state(self.last_telemetry.armed)

    def _on_pause_path_requested(self):
        """Called when user clicks 'PAUSE / HOLD' in Tactical SLAM tab."""
        self.offboard_pump_timer.stop()
        if not self.worker or not self.worker.isRunning():
            return
        self.worker.set_mode("AUTO.LOITER")
        self.path_in_progress = False
        self.path_paused = True
        self.loiter_pause_start_time = time.time()
        self._loiter_warned = False
        self.path_awaiting_climb = False
        self.page_slam.set_executing_state(False, paused=True)
        self.console.log_warning("[PAUSE] Drone holding position in AUTO.LOITER")
        self.page_terminal.log_warning("[PAUSE] Drone holding position in AUTO.LOITER")
        self.toast.show_message("Path Paused: Position Hold", "#d29922", 3000)

    def _on_resume_path_requested(self):
        """Called when user clicks 'RESUME PATH' in Tactical SLAM tab."""
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot resume path: Not connected to drone")
            self.toast.show_message("Cannot resume: Disconnected", "#da3633")
            return

        # 1. Arm Status Interlock
        if not self.last_telemetry.armed:
            self.console.log_error(
                "❌ FLIGHT INTERLOCK REJECTED: Cannot resume path while vehicle is DISARMED! "
                "Arm drone and confirm clear airspace first."
            )
            self.page_terminal.log_error(
                "❌ FLIGHT INTERLOCK REJECTED: Drone is DISARMED! Arm before resuming path."
            )
            self.toast.show_message("Cannot Resume: Drone is Disarmed!", "#da3633", 5000)
            return

        if not self.active_waypoints or self.current_wpt_idx >= len(self.active_waypoints):
            self.console.log_warning("No remaining waypoints to resume.")
            return

        # 2. Dynamic Collision Re-validation Before Resuming
        if self.latest_map_data is not None:
            grid, res, ox, oy = self.latest_map_data
            remaining_wpts = self.active_waypoints[self.current_wpt_idx:]
            is_blocked, col_pt, col_dist = self.path_planner.check_path_collision(
                grid, res, ox, oy, remaining_wpts, (self.last_telemetry.x, self.last_telemetry.y), lookahead_m=1.5
            )
            if is_blocked:
                self.console.log_error(
                    f"🚨 CANNOT RESUME: Path still blocked by obstacle {col_dist:.2f}m ahead! "
                    "Clear the obstacle or plan a new detour."
                )
                self.toast.show_message(f"Cannot Resume: Blocked ({col_dist:.2f}m)!", "#da3633", 5000)
                return

        # 3. OFFBOARD Mode Interlock - PAUSE puts PX4 into AUTO.LOITER; mode
        # is never re-engaged silently, so the operator must switch back to
        # OFFBOARD manually (Mode dropdown -> SET MODE) before resuming.
        if self.last_telemetry.flight_mode != "OFFBOARD":
            curr_mode = self.last_telemetry.flight_mode or "UNKNOWN"
            msg = (f"Cannot resume path: current mode is {curr_mode}, not OFFBOARD. "
                   "Switch to OFFBOARD manually (Mode dropdown -> SET MODE) first.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Not in OFFBOARD - switch manually first", "#da3633", 4000)
            return

        self.console.log_cmd("[RESUME] Resuming path...")
        self.page_terminal.log_cmd("[RESUME] Resuming path...")
        self.path_in_progress = True
        self.path_paused = False
        self.page_slam.set_executing_state(True, paused=False)

        next_wpt = self.active_waypoints[self.current_wpt_idx]
        dx = next_wpt[0] - self.last_telemetry.x
        dy = next_wpt[1] - self.last_telemetry.y
        seg_dist = math.sqrt(dx * dx + dy * dy)
        yaw_deg = math.degrees(math.atan2(dy, dx)) if seg_dist > 0.05 else self.last_telemetry.heading
        self.current_target_yaw = yaw_deg

        self.worker.move_to_waypoint(next_wpt[0], next_wpt[1], z=self.cruise_z, yaw_deg=yaw_deg)
        self.offboard_pump_timer.start()

        self.exec_tracker.start_tracking(
            f"waypoint W{self.current_wpt_idx+1}", self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            target_dist=seg_dist
        )
        self.toast.show_message("Path Resumed", "#238636", 3000)

    def _on_abort_path_requested(self):
        """Called when user clicks 'ABORT & LAND' in Tactical SLAM tab."""
        self._begin_path_generation()
        self.offboard_pump_timer.stop()
        self.path_in_progress = False
        self.path_paused = False
        self.path_awaiting_climb = False
        self.active_waypoints = []
        self.current_wpt_idx = 0
        self.page_slam.set_executing_state(False, paused=False)
        self.page_slam.canvas.clear_goal()

        if self.worker and self.worker.isRunning():
            self.worker.set_mode("AUTO.LAND")
            self.console.log_error("[ABORT] Emergency transition to AUTO.LAND initiated!")
            self.page_terminal.log_error("[ABORT] Emergency transition to AUTO.LAND initiated!")
            self.toast.show_message("ABORT: AUTO.LAND Engaged", "#da3633", 5000)

    def _on_collision_result(self, token: int, is_blocked: bool,
                             collision_point, distance_m: float) -> None:
        """Act on a collision verdict, if it is still about the current path.

        The generation check is the important part. A verdict computed against
        a path that has since been aborted, completed or replaced by a detour
        would otherwise halt a perfectly good new path on the strength of an
        obstacle that is no longer in front of the aircraft.
        """
        if token != self._collision_token:
            return
        self._collision_token = None

        if getattr(self, "_collision_generation", None) != self._path_generation:
            return
        if not self.path_in_progress or not self.active_waypoints:
            return
        if not is_blocked:
            return

        now = time.time()
        t = self.last_telemetry
        self.worker.set_mode("AUTO.LOITER")
        self.path_in_progress = False
        self.path_paused = True
        self.loiter_pause_start_time = now
        self._loiter_warned = False
        self.page_slam.set_executing_state(False, paused=True)

        self.console.log_error(
            f"🚨 COLLISION ALERT: Obstacle detected {distance_m:.2f}m ahead on "
            "flight path! Halting in AUTO.LOITER.")
        self.page_terminal.log_error(
            f"🚨 COLLISION ALERT: Obstacle {distance_m:.2f}m ahead! Switched to AUTO.LOITER.")
        self.toast.show_message(
            f"Obstacle Ahead ({distance_m:.2f}m)! Drone Halted", "#da3633", 6000)
        self.audio.say("Obstacle ahead, holding", Severity.ALARM, key="collision")

        # Look for a way round, also off this thread. The aircraft is already
        # holding, so the answer can take as long as it takes.
        if self.latest_map_data is None:
            return
        grid, res, ox, oy = self.latest_map_data
        final_goal = self.active_waypoints[-1]
        self._detour_generation = self._path_generation
        remaining = list(getattr(self, "_mission_stops_remaining", None) or [])
        if len(remaining) > 1:
            # A mission: detour through every stop not yet reached. Planning
            # only to the final waypoint - as a single goal did - would drop
            # the intermediate stops without a word.
            self._detour_token = self.planner_worker.request_route(
                grid, res, ox, oy, (t.x, t.y), remaining)
        else:
            self._detour_token = self.planner_worker.request_plan(
                grid, res, ox, oy, (t.x, t.y), final_goal)
        self.console.log_info("Searching for a detour around the obstacle...")

    def _on_detour_ready(self, token: int, result) -> None:
        """Adopt a detour, if it is still wanted."""
        if token != getattr(self, "_detour_token", None):
            return
        self._detour_token = None
        if getattr(self, "_detour_generation", None) != self._path_generation:
            return
        if result and result.get("success"):
            waypoints = result["waypoints"]
            self.console.log_success(
                f"🔄 DETOUR READY: Clear path found ({len(waypoints)} WPTs, "
                f"{result['total_distance_m']:.2f}m). Press RESUME to fly it.")
            self.page_slam.canvas.planned_waypoints = waypoints
            self.page_slam.canvas.update()
            self.active_waypoints = list(waypoints)
            self.current_wpt_idx = 0
            self._progress_route_key = None      # the route changed; re-seed
        else:
            self.console.log_warning(
                "No clear detour found. Maintain hold or command manual RTL / LAND.")

    def _progress_state(self) -> str:
        """Translate the window's navigation flags into one operator-facing word."""
        if self.path_awaiting_climb:
            return "CLIMBING"
        if self.path_paused:
            # The collision handler pauses and leaves path_in_progress False,
            # which is what distinguishes an obstacle hold from an operator
            # pause - worth showing differently, because one of them means
            # something is in the way.
            return "PAUSED" if self.path_in_progress else "OBSTACLE HOLD"
        if self.path_in_progress:
            return "EN ROUTE"
        return "STAGED"

    def _update_mission_progress(self, t: TelemetrySnapshot) -> None:
        """Feed the route strip from state the tick has already computed."""
        strip = self.page_slam.progress
        waypoints = self.active_waypoints or self.pending_path_waypoints
        if not waypoints:
            if strip.isVisible():
                strip.clear()
            self._progress_route_key = None
            return

        # Re-seed only when the route itself changes. A detour replaces the
        # waypoint list mid-flight, and the strip has to restart against the new
        # one rather than keep measuring against the route that was blocked.
        key = (len(waypoints), waypoints[0], waypoints[-1])
        if key != self._progress_route_key:
            self._progress_route_key = key
            strip.set_route(waypoints, origin=(t.x, t.y), state=self._progress_state())

        strip.update_progress(self.current_wpt_idx, (t.x, t.y), t.ground_speed,
                              state=self._progress_state())
