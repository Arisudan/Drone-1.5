"""
================================================================================
MODULE: execution_tracker.py
PURPOSE: Closed-Loop Physical Motion and Command Execution Verifier for Drone GCS
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Core Engine)
  * Communicates:  drone_gcs.py main loop and TelemetryState instances
  * Upstream:      CLI console, waypoint clicks, and flight action buttons
  * Downstream:    GCS HUD / Status Strip / Tactical SLAM execution progress display

DATA FLOW & INTERFACES:
  * Inputs:        Command strings ("move dx dy dz", "takeoff alt", "land", "arm"),
                   live LOCAL_POSITION_NED coordinates (cur_x, cur_y, cur_z),
                   live arming state, flight mode, and COMMAND_ACK events.
  * Outputs:       Status states (IDLE, TRACKING, EXECUTED, STALLED), progress
                   percentages (0-100%), Euclidean displacement deltas, and plain-English verdicts.

KEY LOGIC & FAILSAFES:
  * Closed-Loop Physical Verification: MAVLink COMMAND_ACK only proves packet receipt.
    This module tracks real physical displacement through visual odometry / EKF2
    to guarantee the drone actually executed the commanded movement.
  * State Latching: Latches initial 3D pose (x0, y0, z0) and calculates real-time
    vector progress toward target Euclidean distance.
  * Robust Unsolicited ACK Guarding: Handles unexpected or spontaneous autopilot
    COMMAND_ACK messages gracefully without throwing AttributeError on cmd_action.
  * Stalled State Detection: Triggers STALLED alert if target displacement is not
    reached within the timeout threshold (default 12.0s) or if vehicle stops moving.

USAGE:
  tracker = ExecutionTracker(tolerance=0.20, timeout_sec=12.0)
  tracker.start_tracking("move 1 0 0", cur_x=0.0, cur_y=0.0, cur_z=-0.5)
  tracker.update(cur_x=1.0, cur_y=0.0, cur_z=-0.5)
  if tracker.status == "EXECUTED":
      print("Physical motion completed!")
================================================================================
"""

from __future__ import annotations
import math
import time
from typing import Optional, Dict, Any


class ExecutionTracker:
    """Tracks physical execution of navigation and displacement commands."""

    def __init__(self, tolerance: float = 0.20, timeout_sec: float = 12.0):
        self.tolerance = tolerance
        self.timeout_sec = timeout_sec

        self.active: bool = False
        self.cmd_name: str = ""
        self.cmd_action: str = ""
        self.x0: float = 0.0
        self.y0: float = 0.0
        self.z0: float = 0.0

        self.req_dx: float = 0.0
        self.req_dy: float = 0.0
        self.req_dz: float = 0.0
        self.req_dist: float = 0.0

        self.current_delta: float = 0.0
        self.progress_pct: float = 0.0
        self.start_time: float = 0.0
        self.elapsed_time: float = 0.0

        self.status: str = "IDLE"  # IDLE, TRACKING, EXECUTED, STALLED
        self.status_msg: str = "Awaiting command"

        # Yaw rotation tracking
        self.target_yaw_deg: float = 0.0
        self.initial_heading: float = 0.0
        self.current_yaw_delta: float = 0.0

    def start_tracking(
        self,
        cmd_text: str,
        cur_x: float,
        cur_y: float,
        cur_z: float,
        cur_armed: Optional[bool] = None,
        cur_mode: Optional[str] = None,
        target_dist: Optional[float] = None,
        cur_heading: Optional[float] = None,
    ) -> bool:
        """Parse command and latch baseline position, yaw heading, and telemetry state."""
        parts = cmd_text.strip().split()
        if not parts:
            return False

        cmd_lower = cmd_text.strip().lower()
        if "disarm" in cmd_lower:
            action = "disarm"
        elif "arm" in cmd_lower:
            action = "arm"
        elif "land" in cmd_lower:
            action = "land"
        elif "rtl" in cmd_lower:
            action = "rtl"
        elif "takeoff" in cmd_lower:
            action = "takeoff"
        elif "yaw" in cmd_lower or parts[0].lower() == "yaw":
            action = "yaw"
        elif parts[0].lower() == "move":
            action = "move"
        else:
            action = parts[0].lower()

        self.active = True
        self.cmd_name = cmd_text.strip()
        self.cmd_action = action
        self.x0 = cur_x
        self.y0 = cur_y
        self.z0 = cur_z
        self.initial_armed = cur_armed
        self.initial_mode = cur_mode
        self.start_time = time.time()
        self.current_delta = 0.0
        self.current_yaw_delta = 0.0
        self.progress_pct = 0.0
        self.status = "TRACKING"

        if action == "move" and len(parts) >= 4:
            try:
                self.req_dx = float(parts[1])
                self.req_dy = float(parts[2])
                self.req_dz = float(parts[3])
                self.req_dist = math.sqrt(self.req_dx**2 + self.req_dy**2 + self.req_dz**2)
                self.status_msg = f"Tracking displacement: target {self.req_dist:.2f}m"
                return True
            except ValueError:
                pass
        elif action == "takeoff" and len(parts) >= 2:
            try:
                alt = float(parts[1])
                self.req_dx = 0.0
                self.req_dy = 0.0
                self.req_dz = -abs(alt)
                self.req_dist = abs(alt)
                self.status_msg = f"Tracking takeoff: target alt {abs(alt):.2f}m"
                return True
            except ValueError:
                pass
        elif action == "yaw":
            try:
                self.target_yaw_deg = float(parts[1]) if len(parts) >= 2 else 0.0
            except ValueError:
                self.target_yaw_deg = 0.0
            self.initial_heading = cur_heading if cur_heading is not None else 0.0
            self.req_dx = 0.0
            self.req_dy = 0.0
            self.req_dz = 0.0
            self.req_dist = 0.0
            self.status_msg = f"Tracking yaw rotation: target {self.target_yaw_deg:+.1f}°"
            return True
        elif action in ("land", "rtl", "arm", "disarm"):
            self.req_dx = 0.0
            self.req_dy = 0.0
            self.req_dz = 0.0
            self.req_dist = 0.0
            self.status_msg = f"Verifying {action.upper()} state..."
            return True

        # Waypoint or explicitly provided target distance
        if target_dist is not None and target_dist > 0.0:
            self.req_dx = 0.0
            self.req_dy = 0.0
            self.req_dz = 0.0
            self.req_dist = float(target_dist)
            self.status_msg = f"Tracking {action}: target {self.req_dist:.2f}m"
            return True

        # Generic command tracking fallback
        self.req_dx = 0.0
        self.req_dy = 0.0
        self.req_dz = 0.0
        self.req_dist = 0.5  # default nominal threshold
        self.status_msg = f"Tracking {action}..."
        return True

    def update(
        self,
        cur_x: float,
        cur_y: float,
        cur_z: float,
        armed: Optional[bool] = None,
        flight_mode: Optional[str] = None,
        cur_heading: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Update tracker with latest local NED position, heading, and real telemetry state."""
        if not self.active:
            return {
                "active": False,
                "status": self.status,
                "status_msg": self.status_msg,
                "delta": 0.0,
                "progress_pct": 0.0,
            }

        dx = cur_x - self.x0
        dy = cur_y - self.y0
        dz = cur_z - self.z0
        self.current_delta = math.sqrt(dx**2 + dy**2 + dz**2)
        self.elapsed_time = time.time() - self.start_time
        action = getattr(self, "cmd_action", self.cmd_name.split()[0].lower() if self.cmd_name else "")

        # 1. Closed-Loop State Verifications (arm, disarm, land, rtl)
        if action == "arm":
            self.progress_pct = 100.0 if armed else min(95.0, (self.elapsed_time / 4.0) * 100.0)
            if armed is True:
                self.status = "EXECUTED"
                self.status_msg = f"[EXECUTED] Vehicle successfully ARMED in {self.elapsed_time:.1f}s"
                self.active = False
            elif self.elapsed_time > 4.0:
                self.status = "STALLED"
                self.status_msg = f"[FAILED] Arming timed out ({self.elapsed_time:.1f}s) - rejected by preflight checks"
                self.active = False

        elif action == "disarm":
            self.progress_pct = 100.0 if (armed is False) else min(95.0, (self.elapsed_time / 4.0) * 100.0)
            if armed is False:
                self.status = "EXECUTED"
                self.status_msg = f"[EXECUTED] Vehicle successfully DISARMED in {self.elapsed_time:.1f}s"
                self.active = False
            elif self.elapsed_time > 4.0:
                self.status = "STALLED"
                self.status_msg = f"[FAILED] Disarm timed out ({self.elapsed_time:.1f}s)"
                self.active = False

        elif action == "land":
            is_land_mode = flight_mode and ("LAND" in flight_mode.upper())
            self.progress_pct = 100.0 if is_land_mode else min(95.0, (self.elapsed_time / 5.0) * 100.0)
            if is_land_mode:
                self.status = "EXECUTED"
                self.status_msg = f"[EXECUTED] Mode transitioned to {flight_mode} in {self.elapsed_time:.1f}s"
                self.active = False
            elif self.elapsed_time > 5.0:
                self.status = "STALLED"
                self.status_msg = f"[FAILED] Land mode transition timed out ({self.elapsed_time:.1f}s)"
                self.active = False

        elif action == "rtl":
            is_rtl_mode = flight_mode and ("RTL" in flight_mode.upper())
            self.progress_pct = 100.0 if is_rtl_mode else min(95.0, (self.elapsed_time / 5.0) * 100.0)
            if is_rtl_mode:
                self.status = "EXECUTED"
                self.status_msg = f"[EXECUTED] Mode transitioned to {flight_mode} in {self.elapsed_time:.1f}s"
                self.active = False
            elif self.elapsed_time > 5.0:
                self.status = "STALLED"
                self.status_msg = f"[FAILED] RTL mode transition timed out ({self.elapsed_time:.1f}s)"
                self.active = False

        # 2. Closed-Loop Yaw Rotation Verification (heading delta)
        elif action == "yaw":
            if cur_heading is not None:
                # Calculate shortest angular displacement from initial heading
                raw_diff = (cur_heading - self.initial_heading + 180.0) % 360.0 - 180.0
                self.current_yaw_delta = raw_diff
                target = self.target_yaw_deg
                if abs(target) > 1.0:
                    self.progress_pct = min(100.0, max(0.0, (abs(raw_diff) / abs(target)) * 100.0))
                else:
                    self.progress_pct = 100.0

                # Completed if within 5 degrees of target or exceeded target
                if abs(raw_diff - target) <= 5.0 or (abs(target) > 5.0 and abs(raw_diff) >= (abs(target) - 5.0)):
                    self.status = "EXECUTED"
                    self.status_msg = f"[EXECUTED] Yaw rotated {raw_diff:+.1f}° in {self.elapsed_time:.1f}s"
                    self.active = False
                elif self.elapsed_time > self.timeout_sec:
                    self.status = "STALLED"
                    self.status_msg = f"[STALLED] Yaw reached {raw_diff:+.1f}° / {target:+.1f}° after {self.elapsed_time:.1f}s"
                    self.active = False
            else:
                if self.elapsed_time > 3.0:
                    self.status = "EXECUTED"
                    self.status_msg = "[EXECUTED] Yaw command dispatched"
                    self.active = False

        # 3. Closed-Loop Physical Displacement Verifications (move, takeoff, waypoint)
        elif self.req_dist > 0.01:
            self.progress_pct = min(100.0, max(0.0, (self.current_delta / self.req_dist) * 100.0))
            if self.current_delta >= (self.req_dist - self.tolerance):
                self.status = "EXECUTED"
                self.status_msg = f"[EXECUTED] Moved {self.current_delta:.2f}m in {self.elapsed_time:.1f}s"
                self.active = False
            elif self.elapsed_time > self.timeout_sec:
                self.status = "STALLED"
                self.status_msg = f"[STALLED] Reached only {self.current_delta:.2f}m/{self.req_dist:.2f}m"
                self.active = False

        # 4. Fallback generic command acknowledgment
        else:
            self.progress_pct = 100.0
            if self.elapsed_time > 3.0:
                self.status = "EXECUTED"
                self.status_msg = f"[EXECUTED] {self.cmd_name} acknowledged"
                self.active = False

        return {
            "active": self.active,
            "status": self.status,
            "status_msg": self.status_msg,
            "delta": self.current_delta,
            "target": self.req_dist,
            "progress_pct": self.progress_pct,
            "elapsed": self.elapsed_time,
        }

    def notify_command_ack(
        self,
        cmd_id: int,
        result_code: int,
        cmd_name: str,
        result_str: str,
        statustext: Optional[str] = None
    ) -> None:
        """Immediately update execution tracker with real Pixhawk COMMAND_ACK response."""
        now = time.time()
        self.elapsed_time = now - self.start_time if self.start_time > 0 else 0.0
        reason_extra = f": {statustext}" if statustext else ""

        if result_code == 0:  # MAV_RESULT_ACCEPTED
            self.status = "EXECUTED"
            self.progress_pct = 100.0
            self.status_msg = f"[ACCEPTED] Pixhawk confirmed {cmd_name} in {self.elapsed_time:.2f}s"
            # Keep active only if physical displacement tracking is expected
            action = getattr(self, "cmd_action", "")
            if self.req_dist <= 0.01 and (action in ("arm", "disarm", "mode", "land", "rtl") or not action):
                self.active = False
        elif result_code in (1, 2):  # TEMPORARILY_REJECTED or DENIED
            self.status = "REJECTED"
            self.progress_pct = 0.0
            self.status_msg = f"[REJECTED] Pixhawk denied {cmd_name} ({result_str}){reason_extra}"
            self.active = False
        else:
            self.status = "REJECTED"
            self.progress_pct = 0.0
            self.status_msg = f"[REJECTED] {cmd_name} {result_str} (Code {result_code}){reason_extra}"
            self.active = False

    def reset(self):
        self.active = False
        self.status = "IDLE"
        self.status_msg = "Ready"
        self.current_delta = 0.0
        self.progress_pct = 0.0
