#!/usr/bin/env python3
"""Live, read-only FC status watcher with Closed-Loop Execution Verification.

Run this in a terminal on the Radxa SBC or the GCS Laptop to monitor vehicle
status, telemetry, commanded flight modes, and verify physical execution of
commands in real time.

Never sends an arm, disarm, mode-change, or flight command of any kind.
The only messages it transmits are read-only MAV_CMD_SET_MESSAGE_INTERVAL requests
at startup asking PX4 to stream:
  1. SERVO_OUTPUT_RAW (commanded actuator PWM per channel)
  2. LOCAL_POSITION_NED (live position and velocity estimates)

Closed-Loop Execution Verification:
  Unlike raw MAVLink ACKs (which only confirm that the autopilot received a packet),
  this watcher tracks live Local NED displacement (dx, dy, dz) and speed to confirm
  whether the drone ACTUALLY moved in physical space as commanded.

Usage:
    python3 live_status.py                           # Radxa local (tcp:127.0.0.1:5760)
    python3 live_status.py --port tcp:172.16.101.84:5760  # Remote Laptop over Wi-Fi
    python3 live_status.py --period 1.0              # 1 Hz print interval

Ctrl+C to stop.
"""
import argparse
import math
import os
import sys
import time

# Ensure MAVLink 2.0 is used
os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

# Line-buffer stdout so output is immediately visible in piped/background terminals
sys.stdout.reconfigure(line_buffering=True)

# PX4 Custom Modes mapping: (main_mode, sub_mode)
PX4_MODES = {
    "MANUAL":       (1, 0),
    "ALTCTL":       (2, 0),
    "POSCTL":       (3, 0),
    "AUTO.TAKEOFF": (4, 2),
    "AUTO.LOITER":  (4, 3),
    "AUTO.MISSION": (4, 4),
    "AUTO.RTL":     (4, 5),
    "AUTO.LAND":    (4, 6),
    "ACRO":         (5, 0),
    "OFFBOARD":     (6, 0),
    "STABILIZED":   (7, 0),
}
_PX4_MODE_NAMES = {v: k for k, v in PX4_MODES.items()}


def px4_mode_name(custom_mode):
    """Decode a PX4 HEARTBEAT custom_mode bitfield into a readable flight mode name."""
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF
    name = _PX4_MODE_NAMES.get((main_mode, sub_mode))
    if name is not None:
        return name
    if main_mode == 4:
        return f"AUTO(sub={sub_mode})"
    return f"UNKNOWN({main_mode},{sub_mode})"


RESULT = mavutil.mavlink.enums.get("MAV_RESULT", {})
CMD_ENUM = mavutil.mavlink.enums.get("MAV_CMD", {})


def result_name(result):
    if result is None:
        return "-"
    e = RESULT.get(result)
    return e.name if e else f"result={result}"


def cmd_name(cmd_id):
    if cmd_id is None:
        return "-"
    e = CMD_ENUM.get(cmd_id)
    return e.name if e else f"CMD_{cmd_id}"


class ExecutionTracker:
    """Tracks physical vehicle movement relative to baseline coordinates."""

    def __init__(self):
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_z = 0.0
        self.cur_vx = 0.0
        self.cur_vy = 0.0
        self.cur_vz = 0.0
        self.has_pos = False

        # Baseline origin for delta tracking
        self.base_x = None
        self.base_y = None
        self.base_z = None
        self.base_time = 0.0

        self.moving = False
        self.move_start_time = 0.0
        self.last_exec_msg = "IDLE (holding)"

    def update_position(self, x, y, z, vx, vy, vz):
        self.cur_x = x
        self.cur_y = y
        self.cur_z = z
        self.cur_vx = vx
        self.cur_vy = vy
        self.cur_vz = vz

        if not self.has_pos:
            self.has_pos = True
            self.reset_baseline()
            return None

        speed = math.sqrt(max(0.0, vx * vx + vy * vy + vz * vz))
        dx = x - self.base_x
        dy = y - self.base_y
        dz = z - self.base_z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        event_msg = None

        # Detect motion start
        if not self.moving and speed > 0.08 and dist > 0.05:
            self.moving = True
            self.move_start_time = time.time()
            self.last_exec_msg = f"MOVING (d={dist:.2f}m)"

        # Detect motion completed & settled
        elif self.moving:
            self.last_exec_msg = f"MOVING (dx={dx:+.2f} dy={dy:+.2f} dz={dz:+.2f}m)"
            if speed < 0.06 and (time.time() - self.move_start_time) > 1.2:
                self.moving = False
                if dist >= 0.10:
                    event_msg = (
                        f">>> [✔ EXECUTED] Motion confirmed: "
                        f"dx={dx:+.2f}m dy={dy:+.2f}m dz={dz:+.2f}m (dist={dist:.2f}m, speed={speed:.2f}m/s)"
                    )
                    self.last_exec_msg = f"✔ EXECUTED (d={dist:.2f}m)"
                    self.reset_baseline()
                else:
                    self.last_exec_msg = "IDLE (settled)"

        return event_msg

    def reset_baseline(self):
        if self.has_pos:
            self.base_x = self.cur_x
            self.base_y = self.cur_y
            self.base_z = self.cur_z
            self.base_time = time.time()

    def get_status_str(self):
        if not self.has_pos:
            return "WAITING FOR EKF"
        if self.base_x is None:
            return "READY"
        dx = self.cur_x - self.base_x
        dy = self.cur_y - self.base_y
        dz = self.cur_z - self.base_z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if self.moving:
            return f"MOVING: dx={dx:+.2f} dy={dy:+.2f} dz={dz:+.2f}m"
        return f"{self.last_exec_msg} [Δd={dist:.2f}m]"


def main():
    ap = argparse.ArgumentParser(description="PX4 Live Status & Closed-Loop Execution Monitor")
    ap.add_argument("--port", default="tcp:127.0.0.1:5760",
                    help="mavlink-router endpoint (e.g., tcp:127.0.0.1:5760 or tcp:172.16.101.84:5760)")
    ap.add_argument("--period", type=float, default=1.0, help="Periodic print interval in seconds")
    args = ap.parse_args()

    print(f"=======================================================================")
    print(f" DRONE-1.5 | PX4 LIVE STATUS & EXECUTION VERIFIER")
    print(f" Connecting to: {args.port}")
    print(f"=======================================================================")

    try:
        m = mavutil.mavlink_connection(args.port, source_system=251)
    except Exception as e:
        print(f"Connection error: {e}")
        return 1

    # Validate two consecutive heartbeats from vehicle
    print("Waiting for validated vehicle heartbeat ...", flush=True)
    if m.wait_heartbeat(timeout=10) is None:
        print("NO HEARTBEAT! Check mavlink-routerd service (systemctl status mavlink-router).", flush=True)
        return 1

    hb = None
    prev = None
    deadline = time.time() + 10
    while time.time() < deadline:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None or msg.get_srcSystem() == 0:
            continue
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            continue  # Ignore GCS heartbeat echoes
        if prev is not None and (msg.get_srcSystem(), msg.get_srcComponent()) == \
                (prev.get_srcSystem(), prev.get_srcComponent()):
            hb = msg
            break
        prev = msg

    if hb is None:
        print("Failed to validate stable heartbeats from autopilot. Aborting.", flush=True)
        return 1

    tsys, tcomp = hb.get_srcSystem(), hb.get_srcComponent()
    print(f"Heartbeat LOCKED: System ID={tsys}, Component ID={tcomp} (PX4 Autopilot)\n", flush=True)

    # Request required telemetry streams via read-only SET_MESSAGE_INTERVAL
    # 1. SERVO_OUTPUT_RAW at 2 Hz (500,000 us)
    m.mav.command_long_send(
        tsys, tcomp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        500000, 0, 0, 0, 0, 0
    )
    # 2. LOCAL_POSITION_NED at 10 Hz (100,000 us)
    m.mav.command_long_send(
        tsys, tcomp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        100000, 0, 0, 0, 0, 0
    )

    tracker = ExecutionTracker()
    last_ack = None
    last_servo = None
    armed_prev = None
    mode_prev = None

    print(f"{'t(s)':>5}  {'ARM':<8} {'MODE':<14} {'POS NED (m)':<24} {'SPEED':<9} {'EXECUTION STATUS':<30} LAST ACK")
    print("-" * 115, flush=True)

    start_time = time.time()
    last_print = 0.0

    try:
        while True:
            msg = m.recv_match(blocking=True, timeout=0.5)
            if msg is not None:
                src_sys = msg.get_srcSystem()
                src_comp = msg.get_srcComponent()
                if src_sys == 0:
                    continue

                msg_type = msg.get_type()

                if msg_type == "HEARTBEAT" and src_sys == tsys and src_comp == tcomp:
                    hb = msg

                elif msg_type == "LOCAL_POSITION_NED" and src_sys == tsys:
                    evt = tracker.update_position(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
                    if evt:
                        print(f"\n{evt}\n", flush=True)

                elif msg_type == "COMMAND_ACK" and src_sys == tsys:
                    cmd_id = msg.command
                    res_code = msg.result
                    last_ack = (cmd_id, res_code)
                    r_name = result_name(res_code)
                    c_name = cmd_name(cmd_id)
                    print(f"\n>>> [COMMAND_ACK] {c_name} ({cmd_id}) -> {r_name} ({res_code})", flush=True)
                    if res_code == 0:
                        # Reset baseline to track motion following an accepted command
                        tracker.reset_baseline()
                    elif res_code == 1:
                        print(f"    Notice: TEMPORARILY_REJECTED. Safety locks active (USB/Battery check).", flush=True)

                elif msg_type == "SERVO_OUTPUT_RAW" and src_sys == tsys:
                    last_servo = (msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw)

                elif msg_type == "STATUSTEXT":
                    txt = msg.text
                    if isinstance(txt, bytes):
                        txt = txt.decode("utf-8", errors="ignore")
                    if "[GCS CMD]" in txt:
                        cmd_payload = txt.split("[GCS CMD]", 1)[1].strip()
                        print(f"\n>>> [COMMAND RECEIVED FROM LAPTOP] {cmd_payload}", flush=True)
                        tracker.reset_baseline()
                    elif src_sys == tsys:
                        print(f"\n[PX4 STATUSTEXT] {txt}", flush=True)

            now = time.time()
            if now - last_print < args.period:
                continue
            last_print = now

            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode = px4_mode_name(hb.custom_mode)

            # Announce state changes
            if armed_prev is not None and armed != armed_prev:
                tag = "*** VEHICLE ARMED ***" if armed else "*** VEHICLE DISARMED ***"
                print(f"\n{tag}\n", flush=True)
                tracker.reset_baseline()
            if mode_prev is not None and mode != mode_prev:
                print(f"\n>>> [MODE CHANGE] {mode_prev} -> {mode}\n", flush=True)
                tracker.reset_baseline()

            armed_prev = armed
            mode_prev = mode

            # Telemetry formatting
            if tracker.has_pos:
                pos_s = f"X:{tracker.cur_x:5.2f} Y:{tracker.cur_y:5.2f} Z:{tracker.cur_z:5.2f}"
                speed = math.sqrt(tracker.cur_vx**2 + tracker.cur_vy**2 + tracker.cur_vz**2)
                spd_s = f"{speed:4.2f} m/s"
            else:
                pos_s = "No Local Pos (Wait)"
                spd_s = "-- m/s"

            exec_s = tracker.get_status_str()
            ack_s = f"{cmd_name(last_ack[0])}:{result_name(last_ack[1])}" if last_ack else "-"

            elapsed = int(now - start_time)
            arm_s = "ARMED" if armed else "disarmed"

            print(f"{elapsed:5d}  {arm_s:<8} {mode:<14} {pos_s:<24} {spd_s:<9} {exec_s:<30} {ack_s}", flush=True)

    except KeyboardInterrupt:
        print("\nMonitor stopped by user.")
    finally:
        m.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
