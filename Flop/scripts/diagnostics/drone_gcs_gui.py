#!/usr/bin/env python3
"""Drone-1.5 Python Ground Control Station (GCS) GUI.

A standalone, modern, dark-themed Python GUI for monitoring and commanding a
PX4-powered drone over Wi-Fi / Ethernet via MAVLink.

Key Features:
  * Dynamic IP & Port entry for easy network reconfiguration (hostname -I support).
  * Mutually Exclusive Modes:
      - RECEIVE MODE: Passive read-only telemetry dashboard (live_status.py style).
      - SEND MODE: Active command dispatcher (px4_control.py style) with setpoint streaming.
  * Direct CLI Command Input in Sender Mode:
      - Enter commands directly: 'move 1 0 0', 'takeoff 1.5', 'arm force', 'yaw 90', etc.
      - Full command history with Up / Down arrow keys.
      - Retains all flight action buttons for one-click operations.
  * Closed-Loop Execution Verification in Receiver Mode:
      - Verifies that the vehicle ACTUALLY moved in physical space (via live LOCAL_POSITION_NED).
      - Displays real-time displacement (dx, dy, dz), progress %, and confirms execution (✔ EXECUTED vs ✖ STALLED).
  * Live Telemetry: Position (NED), Velocities, Attitude/Yaw, Battery, Altitude, Flight Mode.
  * Flight Commands: Arm/Disarm (with Bench Force Override), Takeoff, Land, Hold, Offboard, Move, Yaw.
  * Command Acknowledgment Inspector: Real-time MAVLink ACK feedback with plain-English diagnostics.
"""

import os
import sys
import time
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox

# Ensure MAVLink 2.0 is used before importing pymavlink
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")

try:
    from pymavlink import mavutil
except ImportError:
    print("Error: pymavlink is required. Install it using: pip3 install pymavlink")
    sys.exit(1)

# PX4 Custom Modes mapping: (main_mode, sub_mode)
PX4_MODES = {
    "MANUAL": (1, 0),
    "ALTCTL": (2, 0),
    "POSCTL": (3, 0),
    "AUTO.TAKEOFF": (4, 2),
    "AUTO.LOITER": (4, 3),
    "AUTO.MISSION": (4, 4),
    "AUTO.RTL": (4, 5),
    "AUTO.LAND": (4, 6),
    "ACRO": (5, 0),
    "OFFBOARD": (6, 0),
    "STABILIZED": (7, 0),
}
_PX4_MODE_NAMES = {v: k for k, v in PX4_MODES.items()}


def decode_px4_mode(custom_mode):
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF
    name = _PX4_MODE_NAMES.get((main_mode, sub_mode))
    if name is not None:
        return name
    if main_mode == 4:
        return f"AUTO(sub={sub_mode})"
    return f"UNKNOWN({main_mode},{sub_mode})"


ACK_RESULTS = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}


# ==============================================================================
# MAVLink Backend Worker with Closed-Loop Execution Verification
# ==============================================================================
class PX4Backend:
    def __init__(self, on_telemetry_callback, on_log_callback):
        self.on_telemetry = on_telemetry_callback
        self.on_log = on_log_callback

        self.master = None
        self.connected = False
        self.target_system = 1
        self.target_component = 1

        self.mode = "RECEIVE"  # "RECEIVE" or "SEND"
        self._stop_event = threading.Event()
        self._rx_thread = None
        self._stream_thread = None

        # Offboard state & setpoint
        self._offboard_active = False
        self._current_sp = None
        self._tx_lock = threading.Lock()

        # Telemetry snapshot cache
        self.telemetry = {
            "connected": False,
            "armed": False,
            "mode": "DISCONNECTED",
            "x": 0.0, "y": 0.0, "z": 0.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "battery_v": 0.0, "battery_pct": 0,
            "dist_bottom": None,
            "last_heartbeat": 0.0,
            "last_ev_time": 0.0,
            "exec_status": "IDLE (holding)",
            "exec_details": "Displacement: dx=0.00m dy=0.00m dz=0.00m | Speed: 0.00 m/s",
            "exec_color": "#cdd6f4",
        }

        # Active command for closed-loop physical execution verification (when commanded locally)
        self.active_cmd = None
        self._cmd_lock = threading.Lock()

        # Passive displacement verifier (for RECEIVE mode when commands originate from remote Laptop)
        self.passive_base_x = None
        self.passive_base_y = None
        self.passive_base_z = None
        self.passive_moving = False
        self.passive_move_start = 0.0

    def reset_passive_baseline(self):
        """Reset reference coordinates for passive motion tracking."""
        if self.telemetry.get("x") is not None:
            self.passive_base_x = self.telemetry["x"]
            self.passive_base_y = self.telemetry["y"]
            self.passive_base_z = self.telemetry["z"]
            self.passive_moving = False

    def connect(self, ip, port):
        if self.connected:
            self.disconnect()

        endpoint = f"tcp:{ip}:{port}"
        # Distinct MAVLink system ID: 255 for Sender (active GCS), 254 for Receiver (passive monitor)
        src_sys = 255 if self.mode == "SEND" else 254
        self.on_log(f"Connecting to MAVLink endpoint: {endpoint} (System ID {src_sys}) ...", "info")
        self._stop_event.clear()

        try:
            self.master = mavutil.mavlink_connection(endpoint, source_system=src_sys)
        except Exception as e:
            self.on_log(f"Connection failed: {e}", "error")
            return False

        # Start RX listener thread
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Start 20 Hz offboard setpoint stream thread
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

        return True

    def disconnect(self):
        self._stop_event.set()
        self._offboard_active = False
        self._current_sp = None

        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

        self.connected = False
        self.telemetry["connected"] = False
        self.telemetry["mode"] = "DISCONNECTED"
        self.telemetry["exec_status"] = "DISCONNECTED"
        self.telemetry["exec_color"] = "#f38ba8"
        self.on_telemetry(self.telemetry)
        self.on_log("Disconnected from MAVLink endpoint.", "warning")

    def set_mode(self, mode):
        """Switch between 'RECEIVE' and 'SEND'."""
        self.mode = mode
        if mode == "RECEIVE":
            # Safety: halt any active offboard setpoint stream
            self._offboard_active = False
            self.on_log("Switched to [RECEIVE MODE] (Passive Monitor). Flight commands & CLI locked.", "info")
        else:
            self.on_log("Switched to [SEND MODE] (Command Dispatcher). Controls & CLI unlocked.", "info")

    def _rx_loop(self):
        while not self._stop_event.is_set():
            if not self.master:
                break
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
            except Exception as e:
                if not self._stop_event.is_set():
                    self.on_log(f"Socket receive error: {e}", "error")
                break

            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                autopilot = getattr(msg, "autopilot", 0)
                # Filter for real autopilot heartbeats (ignore GCS mirrors)
                if autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                    src_sys = msg.get_srcSystem()
                    src_comp = msg.get_srcComponent()
                    if not self.connected:
                        self.connected = True
                        self.target_system = src_sys
                        self.target_component = src_comp
                        self.telemetry["connected"] = True
                        self.on_log(f"Heartbeat locked from System {src_sys}, Component {src_comp} (PX4 Autopilot)", "success")
                        # Request telemetry streams
                        self._request_streams()

                    self.telemetry["last_heartbeat"] = time.time()
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if self.telemetry["armed"] != armed and self.connected:
                        tag = "ARMED (Motors Active)" if armed else "DISARMED"
                        self.on_log(f"Autopilot safety state: {tag}", "cmd" if armed else "info")
                        self.reset_passive_baseline()
                    new_mode = decode_px4_mode(msg.custom_mode)
                    if self.telemetry["mode"] != new_mode and self.connected:
                        self.reset_passive_baseline()
                    self.telemetry["armed"] = armed
                    self.telemetry["mode"] = new_mode

            elif msg_type == "LOCAL_POSITION_NED":
                self.telemetry["x"] = msg.x
                self.telemetry["y"] = msg.y
                self.telemetry["z"] = msg.z
                self.telemetry["vx"] = msg.vx
                self.telemetry["vy"] = msg.vy
                self.telemetry["vz"] = msg.vz
                self._verify_execution(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)

            elif msg_type == "ATTITUDE":
                self.telemetry["roll"] = math.degrees(msg.roll)
                self.telemetry["pitch"] = math.degrees(msg.pitch)
                self.telemetry["yaw"] = math.degrees(msg.yaw)

            elif msg_type == "DISTANCE_SENSOR":
                self.telemetry["dist_bottom"] = msg.current_distance / 100.0  # cm to m

            elif msg_type == "BATTERY_STATUS":
                if msg.voltages and msg.voltages[0] != 0xFFFF:
                    self.telemetry["battery_v"] = msg.voltages[0] / 1000.0
                self.telemetry["battery_pct"] = msg.battery_remaining

            elif msg_type == "VISION_POSITION_ESTIMATE":
                self.telemetry["last_ev_time"] = time.time()

            elif msg_type == "COMMAND_ACK":
                cmd_id = msg.command
                res_code = msg.result
                res_str = ACK_RESULTS.get(res_code, f"CODE_{res_code}")
                msg_text = f"COMMAND_ACK [cmd={cmd_id}]: {res_str} ({res_code})"
                if res_code == 0:
                    self.on_log(msg_text, "success")
                    self.reset_passive_baseline()
                elif res_code == 1:
                    self.on_log(f"{msg_text} -> TEMPORARILY REJECTED (Check safety circuit breakers / battery)", "warning")
                else:
                    self.on_log(msg_text, "error")

            elif msg_type == "STATUSTEXT":
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode("utf-8", errors="ignore")
                severity = msg.severity

                if "[GCS CMD]" in text:
                    cmd_payload = text.split("[GCS CMD]", 1)[1].strip()
                    self._handle_remote_gcs_cmd(cmd_payload)
                else:
                    self.on_log(f"[PX4 STATUSTEXT] {text}", "info" if severity > 3 else "warning")

            self.on_telemetry(self.telemetry)

    def _request_streams(self):
        """Ask PX4 to stream navigation & telemetry data at 10 Hz."""
        if not self.master:
            return
        stream_ids = [
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
            mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        ]
        interval_us = int(1e6 / 10)  # 10 Hz
        for sid in stream_ids:
            self.master.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, sid, interval_us, 0, 0, 0, 0, 0
            )

    # ---- Closed-Loop Physical Execution Tracking ----
    def _verify_execution(self, x, y, z, vx, vy, vz):
        speed = math.sqrt(max(0.0, vx * vx + vy * vy + vz * vz))

        with self._cmd_lock:
            cmd = self.active_cmd

        if not cmd:
            # Passive Closed-Loop Tracking (for RECEIVE MODE or commands sent from remote Laptop)
            if self.passive_base_x is None:
                self.reset_passive_baseline()
                self.telemetry["exec_status"] = "IDLE (holding)"
                self.telemetry["exec_details"] = f"Position: X={x:.2f} Y={y:.2f} Z={z:.2f} | Speed: {speed:.2f} m/s"
                self.telemetry["exec_color"] = "#cdd6f4"
                return

            dx = x - self.passive_base_x
            dy = y - self.passive_base_y
            dz = z - self.passive_base_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if not self.passive_moving:
                if speed > 0.08 and dist > 0.06:
                    self.passive_moving = True
                    self.passive_move_start = time.time()
                    self.telemetry["exec_status"] = f"MOVING (d={dist:.2f}m)"
                    self.telemetry["exec_color"] = "#89b4fa"
                    self.telemetry["exec_details"] = f"Displaced: dx={dx:+.2f} dy={dy:+.2f} dz={dz:+.2f}m | Speed: {speed:.2f} m/s"
                else:
                    self.telemetry["exec_status"] = "IDLE (holding)"
                    self.telemetry["exec_color"] = "#cdd6f4"
                    self.telemetry["exec_details"] = f"Holding at X={x:.2f} Y={y:.2f} Z={z:.2f} | Speed: {speed:.2f} m/s"
            else:
                self.telemetry["exec_status"] = f"MOVING: dx={dx:+.2f} dy={dy:+.2f} dz={dz:+.2f}m"
                self.telemetry["exec_details"] = f"Displaced: {dist:.2f}m | Speed: {speed:.2f} m/s"
                self.telemetry["exec_color"] = "#89b4fa"

                if speed < 0.06 and (time.time() - self.passive_move_start) > 1.0:
                    self.passive_moving = False
                    if dist >= 0.10:
                        self.telemetry["exec_status"] = f"✔ EXECUTED (Moved {dist:.2f}m)"
                        self.telemetry["exec_color"] = "#a6e3a1"
                        self.on_log(
                            f"✔ [PHYSICAL EXECUTION CONFIRMED] Vehicle moved: dx={dx:+.2f}m dy={dy:+.2f}m dz={dz:+.2f}m (dist={dist:.2f}m, speed={speed:.2f}m/s)",
                            "success"
                        )
                        self.reset_passive_baseline()
                    else:
                        self.telemetry["exec_status"] = "IDLE (settled)"
                        self.telemetry["exec_color"] = "#cdd6f4"
            return

        elapsed = time.time() - cmd["start_time"]

        if cmd["type"] == "MOVE":
            # Calculate displacement from start
            dx = x - cmd["start_pos"][0]
            dy = y - cmd["start_pos"][1]
            dz = z - cmd["start_pos"][2]
            d_moved = math.sqrt(dx * dx + dy * dy + dz * dz)

            tgt_dx, tgt_dy, tgt_dz = cmd["target"]
            d_target = math.sqrt(tgt_dx**2 + tgt_dy**2 + tgt_dz**2)
            d_err = math.sqrt((dx - tgt_dx)**2 + (dy - tgt_dy)**2 + (dz - tgt_dz)**2)
            pct = min(100, int((d_moved / d_target) * 100)) if d_target > 0.05 else 100

            self.telemetry["exec_details"] = (
                f"Displaced: dx={dx:+.2f} dy={dy:+.2f} dz={dz:+.2f}m (dist={d_moved:.2f}/{d_target:.2f}m) | Speed: {speed:.2f}m/s"
            )

            # Check if execution finished
            if (d_err <= 0.20 or (d_moved >= 0.85 * d_target and speed < 0.10)) and elapsed > 0.8:
                self.telemetry["exec_status"] = f"✔ EXECUTED (Moved {d_moved:.2f}m)"
                self.telemetry["exec_color"] = "#a6e3a1"
                self.on_log(
                    f"✔ [PHYSICAL EXECUTION CONFIRMED] Vehicle moved: dx={dx:+.2f}m dy={dy:+.2f}m dz={dz:+.2f}m (Target: {tgt_dx:+.2f}, {tgt_dy:+.2f}, {tgt_dz:+.2f}m)",
                    "success"
                )
                with self._cmd_lock:
                    self.active_cmd = None

            elif elapsed > 8.0 and d_moved < 0.10:
                self.telemetry["exec_status"] = "✖ STALLED (No motion)"
                self.telemetry["exec_color"] = "#f38ba8"
                self.on_log(
                    f"✖ [EXECUTION FAILED / STALLED] Vehicle did not move (dx={dx:+.2f}m, dy={dy:+.2f}m). Check armed state & safety locks!",
                    "error"
                )
                with self._cmd_lock:
                    self.active_cmd = None
            else:
                self.telemetry["exec_status"] = f"MOVING: {pct}% ({d_moved:.2f}m / {d_target:.2f}m)"
                self.telemetry["exec_color"] = "#89b4fa"

        elif cmd["type"] == "TAKEOFF":
            climb = cmd["start_z"] - z  # In NED, -Z is up
            tgt_alt = cmd["target_alt"]
            pct = min(100, int((climb / tgt_alt) * 100)) if tgt_alt > 0.05 else 100

            self.telemetry["exec_details"] = f"Climbed: {climb:+.2f}m / {tgt_alt:.2f}m AGL | Speed: {speed:.2f} m/s"

            if (climb >= 0.85 * tgt_alt and speed < 0.15) and elapsed > 1.0:
                self.telemetry["exec_status"] = f"✔ EXECUTED (Reached {climb:.2f}m)"
                self.telemetry["exec_color"] = "#a6e3a1"
                self.on_log(f"✔ [TAKEOFF CONFIRMED] Vehicle reached {climb:.2f} m altitude!", "success")
                with self._cmd_lock:
                    self.active_cmd = None
            elif elapsed > 8.0 and climb < 0.15:
                self.telemetry["exec_status"] = "✖ STALLED (Takeoff failed)"
                self.telemetry["exec_color"] = "#f38ba8"
                self.on_log("✖ [TAKEOFF STALLED] Drone did not leave ground. Check arm state & safety override!", "error")
                with self._cmd_lock:
                    self.active_cmd = None
            else:
                self.telemetry["exec_status"] = f"CLIMBING: {pct}% ({climb:.2f}m / {tgt_alt:.2f}m)"
                self.telemetry["exec_color"] = "#89b4fa"

    # ---- Inter-GCS Broadcast & Remote Command Handlers ----
    def _broadcast_gcs_cmd(self, cmd_text):
        """Broadcast command to all connected MAVLink router clients (e.g. Radxa GUI)."""
        if self.master:
            try:
                msg_str = f"[GCS CMD] {cmd_text}"[:48]
                self.master.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_NOTICE,
                    msg_str.encode("utf-8")
                )
            except Exception:
                pass

    def _handle_remote_gcs_cmd(self, payload):
        """Handle broadcast commands sent from remote Laptop when running in RECEIVE mode."""
        self.on_log(f">>> [COMMAND RECEIVED FROM LAPTOP] {payload}", "cmd")

        tokens = payload.split()
        if not tokens:
            return
        cmd_name = tokens[0].lower()

        if cmd_name == "move":
            try:
                dx = float(tokens[1])
                dy = float(tokens[2])
                dz = float(tokens[3])
            except (IndexError, ValueError):
                dx, dy, dz = 1.0, 0.0, 0.0

            if not self.telemetry["armed"]:
                self.on_log("✖ [BENCH WARNING] Pixhawk is DISARMED! Move setpoints will be ignored. Arm vehicle first (use 'arm force' on bench).", "warning")

            with self._cmd_lock:
                self.active_cmd = {
                    "type": "MOVE",
                    "target": (dx, dy, dz),
                    "start_pos": (self.telemetry.get("x", 0.0), self.telemetry.get("y", 0.0), self.telemetry.get("z", 0.0)),
                    "start_time": time.time(),
                }
            self.telemetry["exec_status"] = f"TARGET: dx={dx:+.2f} dy={dy:+.2f} dz={dz:+.2f}m"
            self.telemetry["exec_color"] = "#fab387"
            self.telemetry["exec_details"] = f"Target setpoint received from Laptop. Monitoring physical displacement..."

        elif cmd_name == "takeoff":
            try:
                alt = float(tokens[1])
            except (IndexError, ValueError):
                alt = 1.0

            if not self.telemetry["armed"]:
                self.on_log("✖ [BENCH WARNING] Pixhawk is DISARMED! Takeoff will be ignored. Arm vehicle first.", "warning")

            with self._cmd_lock:
                self.active_cmd = {
                    "type": "TAKEOFF",
                    "target_alt": alt,
                    "start_z": self.telemetry.get("z", 0.0),
                    "start_time": time.time(),
                }
            self.telemetry["exec_status"] = f"TARGET: Climb to {alt:.2f}m"
            self.telemetry["exec_color"] = "#fab387"
            self.telemetry["exec_details"] = f"Takeoff climb initiated from Laptop. Monitoring altitude..."

        elif cmd_name == "arm":
            self.reset_passive_baseline()
            self.on_log("Laptop requested ARM. Awaiting Pixhawk confirmation...", "info")

        elif cmd_name == "disarm":
            self.reset_passive_baseline()
            with self._cmd_lock:
                self.active_cmd = None
            self.on_log("Laptop requested DISARM.", "info")

        elif cmd_name in ("hold", "land", "mode", "yaw"):
            self.reset_passive_baseline()
            self.on_log(f"Laptop commanded: {payload}", "info")

    # ---- Offboard 20 Hz Setpoint Streamer ----
    def _stream_loop(self):
        while not self._stop_event.is_set():
            if self._offboard_active and self._current_sp is not None and self.master:
                with self._tx_lock:
                    self._send_setpoint_mavlink(self._current_sp)
            time.sleep(0.05)  # 20 Hz

    def _send_setpoint_mavlink(self, sp):
        POS = mavutil.mavlink
        mask = (POS.POSITION_TARGET_TYPEMASK_VX_IGNORE | POS.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                POS.POSITION_TARGET_TYPEMASK_VZ_IGNORE | POS.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                POS.POSITION_TARGET_TYPEMASK_AY_IGNORE | POS.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                POS.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        if sp["type"] == "pos":
            frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
        else:
            frame = mavutil.mavlink.MAV_FRAME_BODY_NED

        self.master.mav.set_position_target_local_ned_send(
            0, self.target_system, self.target_component,
            frame, mask,
            sp["x"], sp["y"], sp["z"],
            0, 0, 0, 0, 0, 0, sp.get("yaw", 0.0), 0
        )

    # ---- Command Senders (Only executed in SEND mode) ----
    def send_arm(self, force=False):
        if not self._check_can_send(): return
        force_val = 21196 if force else 0
        self.on_log(f"Sending ARM command (force_override={force}) ...", "cmd")
        self._broadcast_gcs_cmd(f"arm force={'1' if force else '0'}")
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, force_val, 0, 0, 0, 0, 0
        )

    def send_disarm(self, force=False):
        if not self._check_can_send(): return
        self._offboard_active = False
        force_val = 21196 if force else 0
        self.on_log(f"Sending DISARM command (force_override={force}) ...", "cmd")
        self._broadcast_gcs_cmd(f"disarm force={'1' if force else '0'}")
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, force_val, 0, 0, 0, 0, 0
        )

    def set_px4_mode(self, mode_name):
        if not self._check_can_send(): return
        if mode_name not in PX4_MODES:
            self.on_log(f"Unknown mode name: {mode_name}", "error")
            return
        main_mode, sub_mode = PX4_MODES[mode_name]
        self.on_log(f"Sending Mode Request: {mode_name} ...", "cmd")
        self._broadcast_gcs_cmd(f"mode {mode_name}")
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            main_mode, sub_mode, 0, 0, 0, 0
        )

    def send_takeoff(self, alt_m):
        if not self._check_can_send(): return
        self.on_log(f"Initiating Offboard Takeoff to {alt_m:.1f} m ...", "cmd")
        self._broadcast_gcs_cmd(f"takeoff {alt_m:.2f}")
        cur_z = self.telemetry.get("z", 0.0)
        target_z = cur_z - float(alt_m)

        with self._cmd_lock:
            self.active_cmd = {
                "type": "TAKEOFF",
                "target_alt": float(alt_m),
                "start_z": cur_z,
                "start_time": time.time(),
            }

        self._current_sp = {
            "type": "pos",
            "x": self.telemetry.get("x", 0.0),
            "y": self.telemetry.get("y", 0.0),
            "z": target_z,
            "yaw": math.radians(self.telemetry.get("yaw", 0.0))
        }
        self._offboard_active = True
        threading.Thread(target=self._delayed_mode_switch, args=("OFFBOARD",), daemon=True).start()

    def _delayed_mode_switch(self, mode_name):
        time.sleep(0.4)
        self.set_px4_mode(mode_name)

    def send_move(self, dx, dy, dz):
        if not self._check_can_send(): return
        self.on_log(f"Sending Move: dx={dx:+.2f}m, dy={dy:+.2f}m, dz={dz:+.2f}m (Body Frame) ...", "cmd")
        self._broadcast_gcs_cmd(f"move {dx:.2f} {dy:.2f} {dz:.2f}")

        with self._cmd_lock:
            self.active_cmd = {
                "type": "MOVE",
                "target": (float(dx), float(dy), float(dz)),
                "start_pos": (self.telemetry.get("x", 0.0), self.telemetry.get("y", 0.0), self.telemetry.get("z", 0.0)),
                "start_time": time.time(),
            }

        self._current_sp = {
            "type": "body_pos",
            "x": float(dx),
            "y": float(dy),
            "z": float(dz),
            "yaw": math.radians(self.telemetry.get("yaw", 0.0))
        }
        self._offboard_active = True
        self.set_px4_mode("OFFBOARD")

    def send_yaw(self, deg):
        if not self._check_can_send(): return
        self.on_log(f"Setting Yaw to {deg}° ...", "cmd")
        self._broadcast_gcs_cmd(f"yaw {deg:.1f}")
        cur_x = self.telemetry.get("x", 0.0)
        cur_y = self.telemetry.get("y", 0.0)
        cur_z = self.telemetry.get("z", 0.0)
        self._current_sp = {
            "type": "pos",
            "x": cur_x, "y": cur_y, "z": cur_z,
            "yaw": math.radians(float(deg))
        }
        self._offboard_active = True
        self.set_px4_mode("OFFBOARD")

    def send_hold(self):
        if not self._check_can_send(): return
        self.on_log("Sending Position Hold (AUTO.LOITER) ...", "cmd")
        self._broadcast_gcs_cmd("hold")
        with self._cmd_lock:
            self.active_cmd = None
        self.set_px4_mode("AUTO.LOITER")

    def send_land(self):
        if not self._check_can_send(): return
        self._offboard_active = False
        self.on_log("Sending Auto Land (AUTO.LAND) ...", "cmd")
        self._broadcast_gcs_cmd("land")
        with self._cmd_lock:
            self.active_cmd = None
        self.set_px4_mode("AUTO.LAND")

    def _check_can_send(self):
        if not self.connected or not self.master:
            self.on_log("Error: Drone is not connected! Connect first.", "error")
            return False
        if self.mode != "SEND":
            self.on_log("Security Gate: Cannot send command while in [RECEIVE MODE]!", "warning")
            return False
        return True


# ==============================================================================
# Modern Dark Tkinter GUI
# ==============================================================================
class DroneGCSApp:
    def __init__(self, root):
        self.root = root
        self.root.title("DRONE-1.5 | Python Ground Control Station (GCS)")
        self.root.geometry("1060x760")
        self.root.minsize(980, 680)

        # Catppuccin Mocha Dark Palette
        self.BG_MAIN = "#181825"
        self.BG_CARD = "#1e1e2e"
        self.BG_CARD_LIGHT = "#313244"
        self.TEXT_MAIN = "#cdd6f4"
        self.TEXT_MUTED = "#9399b2"
        self.ACCENT_BLUE = "#89b4fa"
        self.ACCENT_GREEN = "#a6e3a1"
        self.ACCENT_RED = "#f38ba8"
        self.ACCENT_AMBER = "#fab387"
        self.ACCENT_PURPLE = "#cba6f7"

        self.root.configure(bg=self.BG_MAIN)
        self._setup_styles()

        # Command history for CLI
        self.cmd_history = []
        self.cmd_hist_idx = 0

        self.backend = PX4Backend(
            on_telemetry_callback=self._on_telemetry_async,
            on_log_callback=self._on_log_async
        )

        self._build_ui()

        # Window close handler
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Periodic GUI update tick (10 Hz)
        self._tick()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=self.BG_MAIN)
        style.configure("Card.TFrame", background=self.BG_CARD, relief="flat")
        style.configure("Header.TLabel", background=self.BG_MAIN, foreground=self.TEXT_MAIN, font=("Helvetica", 14, "bold"))
        style.configure("CardTitle.TLabel", background=self.BG_CARD, foreground=self.ACCENT_BLUE, font=("Helvetica", 11, "bold"))
        style.configure("CardVal.TLabel", background=self.BG_CARD, foreground=self.TEXT_MAIN, font=("Helvetica", 12, "bold"))
        style.configure("Muted.TLabel", background=self.BG_CARD, foreground=self.TEXT_MUTED, font=("Helvetica", 9))

    def _build_ui(self):
        # 1. Top Connection Bar
        top_bar = tk.Frame(self.root, bg=self.BG_CARD, padx=14, pady=10)
        top_bar.pack(fill="x", side="top", padx=12, pady=(10, 6))

        tk.Label(top_bar, text="Target SBC IP:", bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Helvetica", 10, "bold")).pack(side="left", padx=(0, 6))
        self.ip_entry = tk.Entry(top_bar, bg=self.BG_CARD_LIGHT, fg=self.TEXT_MAIN, insertbackground=self.TEXT_MAIN, font=("Monospace", 10, "bold"), width=16, relief="flat")
        self.ip_entry.insert(0, "172.16.101.84")
        self.ip_entry.pack(side="left", padx=(0, 14))

        tk.Label(top_bar, text="Port:", bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Helvetica", 10, "bold")).pack(side="left", padx=(0, 6))
        self.port_entry = tk.Entry(top_bar, bg=self.BG_CARD_LIGHT, fg=self.TEXT_MAIN, insertbackground=self.TEXT_MAIN, font=("Monospace", 10, "bold"), width=6, relief="flat")
        self.port_entry.insert(0, "5760")
        self.port_entry.pack(side="left", padx=(0, 16))

        self.btn_connect = tk.Button(
            top_bar, text="CONNECT", bg=self.ACCENT_BLUE, fg="#11111b", font=("Helvetica", 9, "bold"),
            relief="flat", activebackground="#74c7ec", padx=14, pady=3, command=self._toggle_connection
        )
        self.btn_connect.pack(side="left", padx=(0, 16))

        self.lbl_conn_status = tk.Label(top_bar, text="● DISCONNECTED", bg=self.BG_CARD, fg=self.ACCENT_RED, font=("Helvetica", 10, "bold"))
        self.lbl_conn_status.pack(side="left")

        # 2. Mode Switcher (Mutually Exclusive)
        mode_frame = tk.Frame(self.root, bg=self.BG_CARD, padx=12, pady=8)
        mode_frame.pack(fill="x", side="top", padx=12, pady=(0, 8))

        tk.Label(mode_frame, text="OPERATING MODE:", bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Helvetica", 10, "bold")).pack(side="left", padx=(0, 16))

        self.mode_var = tk.StringVar(value="RECEIVE")
        self.btn_mode_recv = tk.Radiobutton(
            mode_frame, text="◀ RECEIVE MODE (Passive Monitor & Verifier)", variable=self.mode_var, value="RECEIVE",
            bg=self.BG_CARD, fg=self.ACCENT_GREEN, selectcolor=self.BG_CARD_LIGHT, activebackground=self.BG_CARD,
            activeforeground=self.ACCENT_GREEN, font=("Helvetica", 10, "bold"), command=self._on_mode_change
        )
        self.btn_mode_recv.pack(side="left", padx=(0, 20))

        self.btn_mode_send = tk.Radiobutton(
            mode_frame, text="▶ SEND MODE (Command Dispatcher & CLI)", variable=self.mode_var, value="SEND",
            bg=self.BG_CARD, fg=self.ACCENT_AMBER, selectcolor=self.BG_CARD_LIGHT, activebackground=self.BG_CARD,
            activeforeground=self.ACCENT_AMBER, font=("Helvetica", 10, "bold"), command=self._on_mode_change
        )
        self.btn_mode_send.pack(side="left")

        # 3. Main Workspace (Split Pane: Left = Send Controls & CLI, Right = Receive Dashboard & Verifier)
        content_pane = tk.Frame(self.root, bg=self.BG_MAIN)
        content_pane.pack(fill="both", expand=True, padx=12, pady=(0, 6))

        # ---- LEFT PANEL: SEND DISPATCHER ----
        self.panel_send = tk.Frame(content_pane, bg=self.BG_CARD, padx=12, pady=10)
        self.panel_send.pack(side="left", fill="both", expand=True, padx=(0, 6))

        tk.Label(self.panel_send, text="COMMAND DISPATCHER (SEND)", bg=self.BG_CARD, fg=self.ACCENT_AMBER, font=("Helvetica", 11, "bold")).pack(anchor="w", pady=(0, 8))

        # Flight Action Buttons
        f_btn_frame = tk.Frame(self.panel_send, bg=self.BG_CARD)
        f_btn_frame.pack(fill="x", pady=(0, 6))

        self.btn_arm = tk.Button(f_btn_frame, text="ARM", bg=self.ACCENT_GREEN, fg="#11111b", font=("Helvetica", 9, "bold"), relief="flat", width=9, pady=3, command=self._cmd_arm)
        self.btn_arm.grid(row=0, column=0, padx=3, pady=3)

        self.btn_disarm = tk.Button(f_btn_frame, text="DISARM", bg=self.ACCENT_RED, fg="#11111b", font=("Helvetica", 9, "bold"), relief="flat", width=9, pady=3, command=self._cmd_disarm)
        self.btn_disarm.grid(row=0, column=1, padx=3, pady=3)

        self.btn_hold = tk.Button(f_btn_frame, text="HOLD", bg=self.ACCENT_BLUE, fg="#11111b", font=("Helvetica", 9, "bold"), relief="flat", width=9, pady=3, command=self._cmd_hold)
        self.btn_hold.grid(row=0, column=2, padx=3, pady=3)

        self.btn_land = tk.Button(f_btn_frame, text="LAND", bg=self.ACCENT_PURPLE, fg="#11111b", font=("Helvetica", 9, "bold"), relief="flat", width=9, pady=3, command=self._cmd_land)
        self.btn_land.grid(row=0, column=3, padx=3, pady=3)

        # Bench Force Override Checkbox
        self.var_force_arm = tk.BooleanVar(value=False)
        self.chk_force_arm = tk.Checkbutton(
            self.panel_send, text="Bench Force Override (Bypass USB/Battery Safety Locks)", variable=self.var_force_arm,
            bg=self.BG_CARD, fg=self.ACCENT_AMBER, selectcolor=self.BG_CARD_LIGHT, activebackground=self.BG_CARD,
            activeforeground=self.ACCENT_AMBER, font=("Helvetica", 8)
        )
        self.chk_force_arm.pack(anchor="w", pady=(0, 8))

        # Takeoff Group
        tk.Label(self.panel_send, text="Takeoff Control:", bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Helvetica", 9, "bold")).pack(anchor="w")
        takeoff_box = tk.Frame(self.panel_send, bg=self.BG_CARD)
        takeoff_box.pack(fill="x", pady=(2, 6))
        tk.Label(takeoff_box, text="Alt (m):", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Helvetica", 9)).pack(side="left", padx=(0, 6))
        self.ent_takeoff_alt = tk.Entry(takeoff_box, bg=self.BG_CARD_LIGHT, fg=self.TEXT_MAIN, width=6, relief="flat", insertbackground=self.TEXT_MAIN)
        self.ent_takeoff_alt.insert(0, "1.0")
        self.ent_takeoff_alt.pack(side="left", padx=(0, 8))
        self.btn_takeoff = tk.Button(takeoff_box, text="TAKEOFF", bg=self.ACCENT_BLUE, fg="#11111b", font=("Helvetica", 8, "bold"), relief="flat", padx=10, command=self._cmd_takeoff)
        self.btn_takeoff.pack(side="left")

        # Relative Translation Group
        tk.Label(self.panel_send, text="Body Movement (dx, dy, dz meters):", bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Helvetica", 9, "bold")).pack(anchor="w")
        move_box = tk.Frame(self.panel_send, bg=self.BG_CARD)
        move_box.pack(fill="x", pady=(2, 6))

        tk.Label(move_box, text="dx:", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Helvetica", 9)).pack(side="left", padx=(0, 2))
        self.ent_dx = tk.Entry(move_box, bg=self.BG_CARD_LIGHT, fg=self.TEXT_MAIN, width=5, relief="flat", insertbackground=self.TEXT_MAIN)
        self.ent_dx.insert(0, "0.5")
        self.ent_dx.pack(side="left", padx=(0, 6))

        tk.Label(move_box, text="dy:", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Helvetica", 9)).pack(side="left", padx=(0, 2))
        self.ent_dy = tk.Entry(move_box, bg=self.BG_CARD_LIGHT, fg=self.TEXT_MAIN, width=5, relief="flat", insertbackground=self.TEXT_MAIN)
        self.ent_dy.insert(0, "0.0")
        self.ent_dy.pack(side="left", padx=(0, 6))

        tk.Label(move_box, text="dz:", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Helvetica", 9)).pack(side="left", padx=(0, 2))
        self.ent_dz = tk.Entry(move_box, bg=self.BG_CARD_LIGHT, fg=self.TEXT_MAIN, width=5, relief="flat", insertbackground=self.TEXT_MAIN)
        self.ent_dz.insert(0, "0.0")
        self.ent_dz.pack(side="left", padx=(0, 8))

        self.btn_move = tk.Button(move_box, text="MOVE", bg=self.ACCENT_BLUE, fg="#11111b", font=("Helvetica", 8, "bold"), relief="flat", padx=10, command=self._cmd_move)
        self.btn_move.pack(side="left")

        # Heading / Yaw Group
        tk.Label(self.panel_send, text="Heading Control (Yaw):", bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Helvetica", 9, "bold")).pack(anchor="w")
        yaw_box = tk.Frame(self.panel_send, bg=self.BG_CARD)
        yaw_box.pack(fill="x", pady=(2, 6))
        tk.Label(yaw_box, text="Angle (°):", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Helvetica", 9)).pack(side="left", padx=(0, 4))
        self.ent_yaw = tk.Entry(yaw_box, bg=self.BG_CARD_LIGHT, fg=self.TEXT_MAIN, width=6, relief="flat", insertbackground=self.TEXT_MAIN)
        self.ent_yaw.insert(0, "90.0")
        self.ent_yaw.pack(side="left", padx=(0, 8))
        self.btn_yaw = tk.Button(yaw_box, text="ROTATE YAW", bg=self.ACCENT_BLUE, fg="#11111b", font=("Helvetica", 8, "bold"), relief="flat", padx=8, command=self._cmd_yaw)
        self.btn_yaw.pack(side="left")

        # Mode Selector
        tk.Label(self.panel_send, text="Switch Flight Mode directly:", bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Helvetica", 9, "bold")).pack(anchor="w")
        mode_select_box = tk.Frame(self.panel_send, bg=self.BG_CARD)
        mode_select_box.pack(fill="x", pady=(2, 8))
        self.combo_modes = ttk.Combobox(mode_select_box, values=list(PX4_MODES.keys()), state="readonly", width=14)
        self.combo_modes.set("OFFBOARD")
        self.combo_modes.pack(side="left", padx=(0, 8))
        self.btn_set_mode = tk.Button(mode_select_box, text="SET MODE", bg=self.ACCENT_BLUE, fg="#11111b", font=("Helvetica", 8, "bold"), relief="flat", padx=8, command=self._cmd_set_mode)
        self.btn_set_mode.pack(side="left")

        # Direct CLI Command Input Bar (Sender Mode Addition)
        cli_lbl_frame = tk.Frame(self.panel_send, bg=self.BG_CARD)
        cli_lbl_frame.pack(fill="x", pady=(4, 2))
        tk.Label(cli_lbl_frame, text="DIRECT COMMAND INPUT (CLI):", bg=self.BG_CARD, fg=self.ACCENT_AMBER, font=("Helvetica", 9, "bold")).pack(side="left")
        tk.Label(cli_lbl_frame, text="(e.g. 'move 1 0 0', 'takeoff 1.5')", bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Helvetica", 8)).pack(side="left", padx=(6, 0))

        cli_box = tk.Frame(self.panel_send, bg=self.BG_CARD)
        cli_box.pack(fill="x", pady=(2, 4))
        tk.Label(cli_box, text="cmd>", bg=self.BG_CARD, fg=self.ACCENT_AMBER, font=("Monospace", 10, "bold")).pack(side="left", padx=(0, 4))
        self.ent_cli = tk.Entry(cli_box, bg=self.BG_CARD_LIGHT, fg=self.TEXT_MAIN, insertbackground=self.TEXT_MAIN, font=("Monospace", 9, "bold"), relief="flat")
        self.ent_cli.pack(side="left", fill="x", expand=True, padx=(0, 6))
        self.ent_cli.bind("<Return>", lambda e: self._cmd_cli_execute())
        self.ent_cli.bind("<Up>", self._cmd_cli_hist_up)
        self.ent_cli.bind("<Down>", self._cmd_cli_hist_down)

        self.btn_cli_exec = tk.Button(cli_box, text="EXECUTE", bg=self.ACCENT_AMBER, fg="#11111b", font=("Helvetica", 8, "bold"), relief="flat", padx=10, command=self._cmd_cli_execute)
        self.btn_cli_exec.pack(side="left")

        # ---- RIGHT PANEL: RECEIVE TELEMETRY DASHBOARD & VERIFIER ----
        self.panel_recv = tk.Frame(content_pane, bg=self.BG_CARD, padx=12, pady=10)
        self.panel_recv.pack(side="right", fill="both", expand=True, padx=(6, 0))

        tk.Label(self.panel_recv, text="TELEMETRY INSPECTOR & VERIFIER (RECEIVE)", bg=self.BG_CARD, fg=self.ACCENT_GREEN, font=("Helvetica", 11, "bold")).pack(anchor="w", pady=(0, 8))

        # Metrics Grid
        grid_frame = tk.Frame(self.panel_recv, bg=self.BG_CARD)
        grid_frame.pack(fill="x", pady=(0, 6))

        # Row 1: Mode, Armed, Battery
        self.lbl_mode_val = self._create_metric_card(grid_frame, 0, 0, "FLIGHT MODE", "DISCONNECTED", self.ACCENT_BLUE)
        self.lbl_arm_val = self._create_metric_card(grid_frame, 0, 1, "ARM STATE", "DISARMED", self.ACCENT_RED)
        self.lbl_bat_val = self._create_metric_card(grid_frame, 0, 2, "BATTERY", "-- V (--%)", self.ACCENT_AMBER)

        # Row 2: Local NED (X, Y, Z)
        self.lbl_x_val = self._create_metric_card(grid_frame, 1, 0, "POSITION X", "0.00 m", self.TEXT_MAIN)
        self.lbl_y_val = self._create_metric_card(grid_frame, 1, 1, "POSITION Y", "0.00 m", self.TEXT_MAIN)
        self.lbl_z_val = self._create_metric_card(grid_frame, 1, 2, "ALTITUDE Z (NED)", "0.00 m", self.TEXT_MAIN)

        # Row 3: Velocities & Yaw
        self.lbl_vx_val = self._create_metric_card(grid_frame, 2, 0, "VELOCITY Vx", "0.00 m/s", self.TEXT_MAIN)
        self.lbl_vy_val = self._create_metric_card(grid_frame, 2, 1, "VELOCITY Vy", "0.00 m/s", self.TEXT_MAIN)
        self.lbl_yaw_val = self._create_metric_card(grid_frame, 2, 2, "YAW HEADING", "0.0°", self.TEXT_MAIN)

        # Row 4: Sensors & VIO Health
        self.lbl_lidar_val = self._create_metric_card(grid_frame, 3, 0, "DOWNWARD LIDAR", "-- m", self.TEXT_MAIN)
        self.lbl_ev_val = self._create_metric_card(grid_frame, 3, 1, "VIO/SLAM HEARTBEAT", "NO DATA", self.ACCENT_AMBER)
        self.lbl_hb_val = self._create_metric_card(grid_frame, 3, 2, "LINK FRESHNESS", "-- s", self.TEXT_MAIN)

        # Closed-Loop Execution Verification Banner Card
        exec_frame = tk.Frame(self.panel_recv, bg=self.BG_CARD_LIGHT, padx=10, pady=8)
        exec_frame.pack(fill="x", pady=(6, 0))

        tk.Label(exec_frame, text="CLOSED-LOOP PHYSICAL EXECUTION VERIFIER:", bg=self.BG_CARD_LIGHT, fg=self.ACCENT_PURPLE, font=("Helvetica", 9, "bold")).pack(anchor="w")
        self.lbl_exec_val = tk.Label(exec_frame, text="IDLE (holding)", bg=self.BG_CARD_LIGHT, fg=self.TEXT_MAIN, font=("Helvetica", 11, "bold"))
        self.lbl_exec_val.pack(anchor="w", pady=(2, 0))
        self.lbl_exec_details = tk.Label(exec_frame, text="Displacement: dx=+0.00m dy=+0.00m dz=+0.00m | Speed: 0.00 m/s", bg=self.BG_CARD_LIGHT, fg=self.TEXT_MUTED, font=("Monospace", 8))
        self.lbl_exec_details.pack(anchor="w")

        # 4. Bottom Activity & ACK Log Console
        log_frame = tk.Frame(self.root, bg=self.BG_CARD, padx=12, pady=8)
        log_frame.pack(fill="both", expand=True, padx=12, pady=(0, 10))

        tk.Label(log_frame, text="COMMUNICATION & CLOSED-LOOP EXECUTION CONSOLE:", bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Helvetica", 9, "bold")).pack(anchor="w")

        self.txt_log = tk.Text(log_frame, height=7, bg=self.BG_CARD_LIGHT, fg=self.TEXT_MAIN, font=("Monospace", 9), relief="flat", padx=8, pady=6)
        self.txt_log.pack(fill="both", expand=True, side="left", pady=(4, 0))

        scrollbar = tk.Scrollbar(log_frame, command=self.txt_log.yview, bg=self.BG_CARD)
        scrollbar.pack(side="right", fill="y")
        self.txt_log.config(yscrollcommand=scrollbar.set)

        # Configure Log Tag Colors
        self.txt_log.tag_config("info", foreground=self.TEXT_MUTED)
        self.txt_log.tag_config("cmd", foreground=self.ACCENT_BLUE)
        self.txt_log.tag_config("success", foreground=self.ACCENT_GREEN)
        self.txt_log.tag_config("warning", foreground=self.ACCENT_AMBER)
        self.txt_log.tag_config("error", foreground=self.ACCENT_RED)

        # Initial lock state (Starts in RECEIVE mode)
        self._on_mode_change()

    def _create_metric_card(self, parent, row, col, title, initial_val, color):
        card = tk.Frame(parent, bg=self.BG_CARD_LIGHT, padx=8, pady=6)
        card.grid(row=row, column=col, padx=4, pady=4, sticky="nsew")
        parent.grid_columnconfigure(col, weight=1)

        tk.Label(card, text=title, bg=self.BG_CARD_LIGHT, fg=self.TEXT_MUTED, font=("Helvetica", 8, "bold")).pack(anchor="w")
        val_lbl = tk.Label(card, text=initial_val, bg=self.BG_CARD_LIGHT, fg=color, font=("Helvetica", 11, "bold"))
        val_lbl.pack(anchor="w", pady=(2, 0))
        return val_lbl

    # ---- Mode Switching Logic ----
    def _on_mode_change(self):
        mode = self.mode_var.get()
        self.backend.set_mode(mode)

        if mode == "RECEIVE":
            # Disable / Lock all send widgets
            self._set_send_panel_state("disabled")
            self.panel_send.configure(highlightbackground=self.BG_CARD, highlightthickness=1)
            self.panel_recv.configure(highlightbackground=self.ACCENT_GREEN, highlightthickness=2)
        else:
            # Enable Send widgets
            self._set_send_panel_state("normal")
            self.panel_send.configure(highlightbackground=self.ACCENT_AMBER, highlightthickness=2)
            self.panel_recv.configure(highlightbackground=self.BG_CARD, highlightthickness=1)

    def _set_send_panel_state(self, state):
        widgets = [
            self.btn_arm, self.btn_disarm, self.btn_hold, self.btn_land,
            self.btn_takeoff, self.btn_move, self.btn_yaw, self.btn_set_mode,
            self.chk_force_arm, self.combo_modes, self.btn_cli_exec,
            self.ent_takeoff_alt, self.ent_dx, self.ent_dy, self.ent_dz, self.ent_yaw, self.ent_cli
        ]

        for w in widgets:
            try:
                w.configure(state=state)
            except Exception:
                pass

    # ---- Connection Handler ----
    def _toggle_connection(self):
        if not self.backend.connected:
            ip = self.ip_entry.get().strip()
            port = self.port_entry.get().strip()
            if not ip or not port:
                messagebox.showerror("Invalid Input", "Please enter a valid IP and Port.")
                return
            self.btn_connect.config(text="DISCONNECT", bg=self.ACCENT_RED)
            self.backend.connect(ip, port)
        else:
            self.btn_connect.config(text="CONNECT", bg=self.ACCENT_BLUE)
            self.backend.disconnect()

    # ---- Command Actions ----
    def _cmd_arm(self):
        force = self.var_force_arm.get()
        self.backend.send_arm(force=force)

    def _cmd_disarm(self):
        force = self.var_force_arm.get()
        self.backend.send_disarm(force=force)

    def _cmd_hold(self):
        self.backend.send_hold()

    def _cmd_land(self):
        self.backend.send_land()

    def _cmd_takeoff(self):
        if not self.backend.telemetry["armed"]:
            self._log_msg("Notice: Drone is currently DISARMED! Arm first (use 'Bench Force Override' on desk).", "warning")
        try:
            alt = float(self.ent_takeoff_alt.get())
            self.backend.send_takeoff(alt)
        except ValueError:
            self._log_msg("Invalid altitude value!", "error")

    def _cmd_move(self):
        if not self.backend.telemetry["armed"]:
            self._log_msg("Notice: Drone is currently DISARMED! Pixhawk ignores movement commands when disarmed. Arm first (use 'Bench Force Override' on desk).", "warning")
        try:
            dx = float(self.ent_dx.get())
            dy = float(self.ent_dy.get())
            dz = float(self.ent_dz.get())
            self.backend.send_move(dx, dy, dz)
        except ValueError:
            self._log_msg("Invalid dx, dy, or dz values!", "error")

    def _cmd_yaw(self):
        try:
            deg = float(self.ent_yaw.get())
            self.backend.send_yaw(deg)
        except ValueError:
            self._log_msg("Invalid yaw value!", "error")

    def _cmd_set_mode(self):
        mode_name = self.combo_modes.get()
        self.backend.set_px4_mode(mode_name)

    # ---- CLI Interactive Command Parser ----
    def _cmd_cli_execute(self):
        raw_cmd = self.ent_cli.get().strip()
        if not raw_cmd:
            return

        self.cmd_history.append(raw_cmd)
        self.cmd_hist_idx = len(self.cmd_history)
        self.ent_cli.delete(0, "end")

        tokens = raw_cmd.split()
        cmd = tokens[0].lower()
        args = tokens[1:]

        self._log_msg(f"cmd> {raw_cmd}", "cmd")

        try:
            if cmd == "arm":
                force = "force" in args or self.var_force_arm.get()
                self.backend.send_arm(force=force)

            elif cmd == "disarm":
                force = "force" in args or self.var_force_arm.get()
                self.backend.send_disarm(force=force)

            elif cmd == "takeoff":
                if not self.backend.telemetry["armed"]:
                    self._log_msg("Notice: Drone is currently DISARMED! Arm first (use 'arm force' on desk).", "warning")
                if not args:
                    alt = float(self.ent_takeoff_alt.get())
                else:
                    alt = float(args[0])
                self.backend.send_takeoff(alt)

            elif cmd == "move":
                if len(args) < 3:
                    self._log_msg("Usage: move <dx> <dy> <dz>  (e.g., move 1 0 0)", "error")
                    return
                if not self.backend.telemetry["armed"]:
                    self._log_msg("Notice: Drone is currently DISARMED! Pixhawk ignores movement commands when disarmed. Arm first (use 'arm force' on desk).", "warning")
                dx, dy, dz = float(args[0]), float(args[1]), float(args[2])
                self.backend.send_move(dx, dy, dz)

            elif cmd == "yaw":
                if not args:
                    self._log_msg("Usage: yaw <degrees>  (e.g., yaw 90)", "error")
                    return
                deg = float(args[0])
                self.backend.send_yaw(deg)

            elif cmd in ("hold", "loiter"):
                self.backend.send_hold()

            elif cmd == "land":
                self.backend.send_land()

            elif cmd == "mode":
                if not args:
                    self._log_msg("Usage: mode <PX4_MODE>  (e.g., mode OFFBOARD, mode POSCTL)", "error")
                    return
                mode_target = args[0].upper()
                self.backend.set_px4_mode(mode_target)

            elif cmd == "clear":
                self.txt_log.delete("1.0", "end")

            elif cmd in ("help", "?"):
                self._log_msg("Supported CLI commands:", "info")
                self._log_msg("  arm [force]           - Arm the drone (optional bench force bypass)", "info")
                self._log_msg("  disarm [force]        - Disarm the drone", "info")
                self._log_msg("  takeoff <alt_m>       - Takeoff to altitude in meters (e.g. takeoff 1.5)", "info")
                self._log_msg("  move <dx> <dy> <dz>   - Move in body frame meters (e.g. move 1 0 0)", "info")
                self._log_msg("  yaw <degrees>         - Rotate to heading in degrees (e.g. yaw 90)", "info")
                self._log_msg("  hold                  - Position hold (AUTO.LOITER)", "info")
                self._log_msg("  land                  - Land at current position (AUTO.LAND)", "info")
                self._log_msg("  mode <MODE>           - Change mode (OFFBOARD, POSCTL, ALTCTL)", "info")
                self._log_msg("  clear                 - Clear console window", "info")

            else:
                self._log_msg(f"Unknown command '{cmd}'. Type 'help' for command list.", "error")

        except ValueError as ve:
            self._log_msg(f"Command syntax error: {ve}", "error")

    def _cmd_cli_hist_up(self, event):
        if self.cmd_history and self.cmd_hist_idx > 0:
            self.cmd_hist_idx -= 1
            self.ent_cli.delete(0, "end")
            self.ent_cli.insert(0, self.cmd_history[self.cmd_hist_idx])
        return "break"

    def _cmd_cli_hist_down(self, event):
        if self.cmd_history and self.cmd_hist_idx < len(self.cmd_history) - 1:
            self.cmd_hist_idx += 1
            self.ent_cli.delete(0, "end")
            self.ent_cli.insert(0, self.cmd_history[self.cmd_hist_idx])
        else:
            self.cmd_hist_idx = len(self.cmd_history)
            self.ent_cli.delete(0, "end")
        return "break"

    # ---- Telemetry & Log Handlers (Thread-safe) ----
    def _on_telemetry_async(self, telem):
        pass  # Data polled in _tick

    def _on_log_async(self, text, tag):
        self.root.after(0, self._log_msg, text, tag)

    def _log_msg(self, text, tag="info"):
        timestamp = time.strftime("[%H:%M:%S] ")
        self.txt_log.insert("end", timestamp + text + "\n", tag)
        self.txt_log.see("end")

    # ---- Periodic 10 Hz UI Refresh ----
    def _tick(self):
        t = self.backend.telemetry

        if t["connected"]:
            self.lbl_conn_status.config(text="● CONNECTED", fg=self.ACCENT_GREEN)
            age = time.time() - t["last_heartbeat"] if t["last_heartbeat"] > 0 else 999.0
            self.lbl_hb_val.config(text=f"{age:.1f} s ago", fg=self.ACCENT_GREEN if age < 2.0 else self.ACCENT_RED)
        else:
            self.lbl_conn_status.config(text="● DISCONNECTED", fg=self.ACCENT_RED)
            self.lbl_hb_val.config(text="-- s", fg=self.TEXT_MUTED)

        # Mode & Armed Status
        self.lbl_mode_val.config(text=t["mode"])
        if t["armed"]:
            self.lbl_arm_val.config(text="ARMED (Active)", fg=self.ACCENT_GREEN)
        else:
            self.lbl_arm_val.config(text="DISARMED", fg=self.ACCENT_RED)

        # Battery
        if t["battery_v"] > 0:
            pct = t["battery_pct"]
            self.lbl_bat_val.config(text=f"{t['battery_v']:.1f} V ({pct}%)")
        else:
            self.lbl_bat_val.config(text="No Battery (5V USB)")

        # Position NED
        self.lbl_x_val.config(text=f"{t['x']:.2f} m")
        self.lbl_y_val.config(text=f"{t['y']:.2f} m")
        self.lbl_z_val.config(text=f"{t['z']:.2f} m")

        # Velocities
        self.lbl_vx_val.config(text=f"{t['vx']:.2f} m/s")
        self.lbl_vy_val.config(text=f"{t['vy']:.2f} m/s")
        self.lbl_yaw_val.config(text=f"{t['yaw']:.1f}°")

        # Downward Lidar
        if t["dist_bottom"] is not None:
            self.lbl_lidar_val.config(text=f"{t['dist_bottom']:.2f} m")
        else:
            self.lbl_lidar_val.config(text="-- m")

        # VIO / SLAM Heartbeat
        ev_age = time.time() - t["last_ev_time"] if t["last_ev_time"] > 0 else 999.0
        if ev_age < 1.0:
            self.lbl_ev_val.config(text=f"LOCKED ({int(ev_age*1000)}ms)", fg=self.ACCENT_GREEN)
        else:
            self.lbl_ev_val.config(text="STALE / NO VIO", fg=self.ACCENT_AMBER)

        # Execution Verifier Status
        self.lbl_exec_val.config(text=t["exec_status"], fg=t["exec_color"])
        self.lbl_exec_details.config(text=t["exec_details"])

        self.root.after(100, self._tick)

    def _on_close(self):
        self.backend.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = DroneGCSApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
