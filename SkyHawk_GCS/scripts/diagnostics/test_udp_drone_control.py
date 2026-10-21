#!/usr/bin/env python3
"""
================================================================================
MODULE: test_udp_drone_control.py
PURPOSE: Interactive & Automated UDP Flight Control, Telemetry & Command ACK Validator
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop GCS or Radxa Q6A Terminal
  * Communicates:  Pixhawk 6X Autopilot via UDP (14550) or TCP (5760)
  * Upstream:      Operator interactive keyboard commands or CI/headless CLI flags
  * Downstream:    Autopilot flight mode changes, arm/disarm, setpoints, and motor stops

DATA FLOW & INTERFACES:
  * Outbound:      GCS Heartbeat (1 Hz), MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_DO_SET_MODE,
                   MAV_CMD_NAV_TAKEOFF, MAV_CMD_DO_FLIGHTTERMINATION, SET_POSITION_TARGET_LOCAL_NED.
  * Inbound:       HEARTBEAT, COMMAND_ACK, STATUSTEXT, SYS_STATUS, BATTERY_STATUS,
                   LOCAL_POSITION_NED, ATTITUDE.

KEY LOGIC & FAILSAFES:
  * Real-Time Low-Latency UDP Protocol: Directly communicates via udpout:<host>:14550.
  * Bidirectional ACK & STATUSTEXT Capture: Catches and decodes all Pixhawk preflight
    rejection reasons (health failures, safety locks, sensor calibration).
  * OFFBOARD Mode Priming: Automatically streams 20 Hz setpoint frames for 500ms
    before dispatching OFFBOARD switch to satisfy PX4 safety stream rules.
  * Dual Operation Mode: Interactive aerospace console interface or scripted non-interactive CLI.

USAGE:
  # Interactive mode over UDP 14550:
  python3 scripts/diagnostics/test_udp_drone_control.py --host 172.16.101.84 --port 14550

  # Single-shot command execution:
  python3 scripts/diagnostics/test_udp_drone_control.py --host 172.16.101.84 --port 14550 --command status
  python3 scripts/diagnostics/test_udp_drone_control.py --host 172.16.101.84 --port 14550 --command force-arm
================================================================================
"""

import sys
import time
import math
import struct
import threading
import argparse
from typing import Optional, Dict, Any

from pymavlink import mavutil

# ANSI Color Codes for Aerospace Terminal UI
CLR_RESET = "\033[0m"
CLR_BOLD = "\033[1m"
CLR_RED = "\033[31m"
CLR_GREEN = "\033[32m"
CLR_YELLOW = "\033[33m"
CLR_BLUE = "\033[34m"
CLR_CYAN = "\033[36m"
CLR_MAGENTA = "\033[35m"
CLR_DIM = "\033[2m"

ACK_RESULTS = {
    0: ("ACCEPTED", CLR_GREEN),
    1: ("TEMPORARILY_REJECTED", CLR_YELLOW),
    2: ("DENIED", CLR_RED),
    3: ("UNSUPPORTED", CLR_RED),
    4: ("FAILED", CLR_RED),
    5: ("IN_PROGRESS", CLR_CYAN),
    6: ("CANCELLED", CLR_YELLOW),
}

CMD_NAMES = {
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: "ARM_DISARM (400)",
    mavutil.mavlink.MAV_CMD_DO_SET_MODE: "DO_SET_MODE (176)",
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "NAV_TAKEOFF (22)",
    mavutil.mavlink.MAV_CMD_NAV_LAND: "NAV_LAND (21)",
    mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION: "FLIGHT_TERMINATION (185)",
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL: "SET_MESSAGE_INTERVAL (511)",
}


class UDPDroneController:
    """Robust UDP/TCP MAVLink flight controller and telemetry validator."""

    def __init__(self, host: str = "172.16.101.84", port: int = 14550, protocol: str = "udp"):
        self.host = host
        self.port = port
        self.protocol = protocol.lower().strip()
        self.master: Optional[mavutil.mavlink_connection] = None

        self.running = False
        self.connected = False
        self.last_heartbeat_time = 0.0

        # State cache
        self.armed = False
        self.flight_mode = "UNKNOWN"
        self.custom_mode = 0
        self.battery_voltage = 0.0
        self.battery_percent = -1
        self.sensors_healthy = 0
        self.sensors_enabled = 0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.rx_count = 0
        self.tx_count = 0

        # Offboard streaming state
        self.streaming_setpoints = False
        self.sp_x = 0.0
        self.sp_y = 0.0
        self.sp_z = -1.0

        # Command response synchronization
        self.last_ack: Optional[Dict[str, Any]] = None
        self.ack_event = threading.Event()
        self.recent_statustexts = []

        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None

    def connect(self) -> bool:
        """Establish connection via UDP or TCP."""
        if self.host.startswith(("tcp:", "udp:", "udpout:", "udpin:")):
            conn_str = self.host
        elif self.protocol in ("udp", "udpout"):
            conn_str = f"udpout:{self.host}:{self.port}"
        elif self.protocol == "udpin":
            conn_str = f"udpin:0.0.0.0:{self.port}"
        else:
            conn_str = f"tcp:{self.host}:{self.port}"

        print(f"{CLR_CYAN}[CONNECTING]{CLR_RESET} Initializing MAVLink endpoint: {CLR_BOLD}{conn_str}{CLR_RESET} (SysID 255)...")

        try:
            self.master = mavutil.mavlink_connection(
                conn_str,
                source_system=255,
                source_component=190,
            )
        except Exception as e:
            print(f"{CLR_RED}[ERROR]{CLR_RESET} Socket initialization failed: {e}")
            return False

        self.running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx_thread.start()
        self._tx_thread.start()

        # Send initial heartbeat immediately to prompt UDP router client registration
        self._send_heartbeat()

        # Request high-frequency telemetry streams
        self._request_streams()

        return True

    def disconnect(self):
        """Clean shutdown of threads and sockets."""
        self.running = False
        self.streaming_setpoints = False
        time.sleep(0.1)
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None
        self.connected = False
        print(f"{CLR_YELLOW}[DISCONNECTED]{CLR_RESET} MAVLink connection terminated.")

    def _send_heartbeat(self):
        if not self.master:
            return
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            self.tx_count += 1
        except Exception:
            pass

    def _request_streams(self):
        """Negotiate standard telemetry stream intervals."""
        if not self.master:
            return
        streams = [
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10.0),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2.0),
            (mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 2.0),
        ]
        for msg_id, hz in streams:
            interval_us = int(1e6 / hz)
            try:
                self.master.mav.command_long_send(
                    1, 1,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    msg_id,
                    interval_us,
                    0, 0, 0, 0, 0
                )
                self.tx_count += 1
            except Exception:
                pass

    def _tx_loop(self):
        """Background 20 Hz thread: pumps GCS heartbeats and offboard setpoints."""
        last_hb = 0.0
        while self.running:
            now = time.time()

            # 1 Hz GCS Heartbeat (maintains UDP routing in mavlink-router)
            if now - last_hb >= 1.0:
                self._send_heartbeat()
                last_hb = now

            # 20 Hz Setpoint Pump (prevents PX4 500ms offboard timeout)
            if self.streaming_setpoints and self.master:
                try:
                    self.master.mav.set_position_target_local_ned_send(
                        int(now * 1000) & 0xFFFFFFFF,
                        1, 1,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        0b0000111111111000,  # Position valid only
                        self.sp_x, self.sp_y, self.sp_z,
                        0, 0, 0,
                        0, 0, 0,
                        0, 0
                    )
                    self.tx_count += 1
                except Exception:
                    pass

            time.sleep(0.05)

    def _rx_loop(self):
        """Background non-blocking MAVLink message parser."""
        while self.running and self.master:
            try:
                msg = self.master.recv_match(blocking=False)
                if msg is None:
                    time.sleep(0.005)
                    continue

                self.rx_count += 1
                mtype = msg.get_type()

                if mtype == "HEARTBEAT":
                    self.connected = True
                    self.last_heartbeat_time = time.time()
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.custom_mode = msg.custom_mode
                    self.flight_mode = self._decode_flight_mode(msg.custom_mode)

                elif mtype == "COMMAND_ACK":
                    res_name, _ = ACK_RESULTS.get(msg.result, (f"UNKNOWN({msg.result})", CLR_RESET))
                    cmd_name = CMD_NAMES.get(msg.command, f"CMD_{msg.command}")
                    self.last_ack = {
                        "command": msg.command,
                        "command_name": cmd_name,
                        "result": msg.result,
                        "result_name": res_name,
                        "time": time.time(),
                    }
                    self.ack_event.set()

                elif mtype == "STATUSTEXT":
                    txt = msg.text.strip()
                    severity = msg.severity
                    self.recent_statustexts.append((time.time(), severity, txt))
                    if len(self.recent_statustexts) > 20:
                        self.recent_statustexts.pop(0)

                elif mtype == "SYS_STATUS":
                    if msg.voltage_battery != 65535:
                        self.battery_voltage = msg.voltage_battery / 1000.0
                    self.battery_percent = msg.battery_remaining
                    self.sensors_healthy = msg.onboard_control_sensors_health
                    self.sensors_enabled = msg.onboard_control_sensors_enabled

                elif mtype == "BATTERY_STATUS":
                    if msg.voltages and msg.voltages[0] != 65535:
                        self.battery_voltage = msg.voltages[0] / 1000.0
                    if msg.battery_remaining != -1:
                        self.battery_percent = msg.battery_remaining

                elif mtype == "LOCAL_POSITION_NED":
                    self.pos_x = msg.x
                    self.pos_y = msg.y
                    self.pos_z = msg.z

                elif mtype == "ATTITUDE":
                    self.roll = math.degrees(msg.roll)
                    self.pitch = math.degrees(msg.pitch)
                    self.yaw = math.degrees(msg.yaw)

            except Exception:
                time.sleep(0.01)

    def _decode_flight_mode(self, custom_mode: int) -> str:
        main_mode = (custom_mode >> 16) & 0xFF
        sub_mode = (custom_mode >> 24) & 0xFF
        if main_mode == 1:
            return "MANUAL"
        elif main_mode == 2:
            return "ALTCTL"
        elif main_mode == 3:
            return "POSCTL"
        elif main_mode == 4:
            if sub_mode == 3:
                return "AUTO.LOITER"
            elif sub_mode == 4:
                return "AUTO.MISSION"
            elif sub_mode == 5:
                return "AUTO.RTL"
            elif sub_mode == 6:
                return "AUTO.LAND"
            elif sub_mode == 8:
                return "AUTO.FOLLOW_TARGET"
            return "AUTO"
        elif main_mode == 5:
            return "ACRO"
        elif main_mode == 6:
            return "OFFBOARD"
        elif main_mode == 7:
            return "STABILIZED"
        return f"CUSTOM({main_mode}:{sub_mode})"

    def wait_for_connection(self, timeout: float = 5.0) -> bool:
        """Wait until first valid heartbeat received."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.connected:
                return True
            time.sleep(0.1)
        return False

    def send_command(self, cmd_id: int, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0, timeout: float = 3.0):
        """Send command and wait for COMMAND_ACK with associated STATUSTEXT alerts."""
        if not self.master:
            print(f"{CLR_RED}[ERROR]{CLR_RESET} Master not connected.")
            return None

        cmd_name = CMD_NAMES.get(cmd_id, f"CMD_{cmd_id}")
        print(f"\n{CLR_CYAN}>> DISPATCHING {CLR_BOLD}{cmd_name}{CLR_RESET} (p1={p1}, p2={p2})...")

        self.last_ack = None
        self.ack_event.clear()
        t_start = time.time()

        try:
            self.master.mav.command_long_send(
                1, 1,
                cmd_id,
                0,
                float(p1), float(p2), float(p3), float(p4), float(p5), float(p6), float(p7)
            )
            self.tx_count += 1
        except Exception as e:
            print(f"{CLR_RED}[TX ERROR]{CLR_RESET} {e}")
            return None

        # Wait for ACK
        acked = self.ack_event.wait(timeout=timeout)
        time.sleep(0.15)  # brief slice to allow following STATUSTEXT packets to arrive

        if acked and self.last_ack:
            res_code = self.last_ack["result"]
            res_str, color = ACK_RESULTS.get(res_code, (f"CODE_{res_code}", CLR_RESET))
            elapsed = (self.last_ack["time"] - t_start) * 1000.0
            print(f"<< {color}[{res_str}]{CLR_RESET} Pixhawk response in {elapsed:.1f} ms")

            # Print any STATUSTEXT messages received during or right after command
            relevant_texts = [txt for (t_stamp, sev, txt) in self.recent_statustexts if t_stamp >= t_start - 0.2]
            for txt in relevant_texts:
                print(f"   {CLR_YELLOW}🚨 PX4 STATUSTEXT:{CLR_RESET} {txt}")
            return self.last_ack
        else:
            print(f"<< {CLR_RED}[TIMEOUT]{CLR_RESET} No COMMAND_ACK received within {timeout:.1f}s.")
            return None

    def arm(self, force: bool = False):
        param2 = 21196 if force else 0
        return self.send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1.0, p2=param2)

    def disarm(self, force: bool = False):
        param2 = 21196 if force else 0
        return self.send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=0.0, p2=param2)

    def set_mode(self, mode_str: str):
        mode_str = mode_str.upper().strip()
        custom_mode = 0
        if mode_str == "MANUAL":
            custom_mode = 1 << 16
            self.streaming_setpoints = False
        elif mode_str == "POSCTL":
            custom_mode = 3 << 16
            self.streaming_setpoints = False
        elif mode_str == "ALTCTL":
            custom_mode = 2 << 16
            self.streaming_setpoints = False
        elif mode_str in ("HOLD", "LOITER", "AUTO.LOITER"):
            custom_mode = (4 << 16) | (3 << 24)
            self.streaming_setpoints = False
        elif mode_str in ("LAND", "AUTO.LAND"):
            custom_mode = (4 << 16) | (6 << 24)
            self.streaming_setpoints = False
        elif mode_str == "OFFBOARD":
            custom_mode = 6 << 16
            self.sp_x = self.pos_x
            self.sp_y = self.pos_y
            self.sp_z = self.pos_z
            self.streaming_setpoints = True
            print(f"{CLR_CYAN}[OFFBOARD]{CLR_RESET} Pre-streaming 20 Hz setpoints for 500ms...")
            time.sleep(0.5)

        if custom_mode != 0:
            main_mode = (custom_mode >> 16) & 0xFF
            sub_mode = (custom_mode >> 24) & 0xFF
            return self.send_command(
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                p2=main_mode,
                p3=sub_mode
            )
        return None

    def takeoff(self, altitude: float = 1.0):
        return self.send_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=altitude)

    def emergency_kill(self):
        print(f"{CLR_RED}{CLR_BOLD}!!! INITIATING EMERGENCY FLIGHT TERMINATION (MOTOR KILL) !!!{CLR_RESET}")
        return self.send_command(mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, p1=1.0)

    def print_status(self):
        """Display comprehensive vehicle health and sensor breakdown."""
        hb_age = time.time() - self.last_heartbeat_time if self.last_heartbeat_time > 0 else 999.0
        link_str = f"{CLR_GREEN}CONNECTED ({hb_age:.1f}s ago){CLR_RESET}" if hb_age < 3.0 else f"{CLR_RED}LOST ({hb_age:.1f}s ago){CLR_RESET}"
        arm_str = f"{CLR_RED}{CLR_BOLD}ARMED{CLR_RESET}" if self.armed else f"{CLR_GREEN}DISARMED{CLR_RESET}"
        batt_str = f"{self.battery_voltage:.2f}V ({self.battery_percent}%)" if self.battery_percent >= 0 else "UNKNOWN"

        print("=" * 80)
        print(f"{CLR_BOLD}AEROSPACE VEHICLE STATUS SUMMARY{CLR_RESET}")
        print("=" * 80)
        print(f"  Target Endpoint:    {self.protocol.upper()} -> {self.host}:{self.port}")
        print(f"  Link Status:        {link_str}")
        print(f"  Arm State:          {arm_str}")
        print(f"  Flight Mode:        {CLR_CYAN}{self.flight_mode}{CLR_RESET} (Custom: 0x{self.custom_mode:08X})")
        print(f"  Battery:            {batt_str}")
        print(f"  Position (NED):     x={self.pos_x:+.2f}m, y={self.pos_y:+.2f}m, z={self.pos_z:+.2f}m")
        print(f"  Attitude:           roll={self.roll:+.1f}°, pitch={self.pitch:+.1f}°, yaw={self.yaw:+.1f}°")
        print(f"  MAVLink Throughput: RX={self.rx_count} pkts, TX={self.tx_count} pkts")

        # Sensor breakdown
        unhealthy = self.sensors_enabled & (~self.sensors_healthy)
        flags = {
            1: "3D_GYRO", 2: "3D_ACCEL", 4: "3D_MAG", 8: "PRESSURE", 32: "GPS",
            128: "VISION_POS", 16384: "XY_POS_CTL", 65536: "RC_RECEIVER", 33554432: "BATTERY"
        }
        print("  Active Sensors:")
        for bit, name in flags.items():
            if self.sensors_enabled & bit:
                if unhealthy & bit:
                    print(f"    - {CLR_RED}[UNHEALTHY]{CLR_RESET} {name}")
                else:
                    print(f"    - {CLR_GREEN}[HEALTHY]{CLR_RESET}   {name}")

        if self.recent_statustexts:
            print("\n  Recent Autopilot Messages:")
            for t_stamp, sev, txt in self.recent_statustexts[-5:]:
                print(f"    [{sev}] {txt}")
        print("=" * 80)


def run_interactive(ctrl: UDPDroneController):
    """Interactive CLI menu for flight operations."""
    while ctrl.running:
        hb_age = time.time() - ctrl.last_heartbeat_time if ctrl.last_heartbeat_time > 0 else 999.0
        link_tag = f"{CLR_GREEN}CONNECTED{CLR_RESET}" if hb_age < 3.0 else f"{CLR_RED}DISCONNECTED{CLR_RESET}"
        arm_tag = f"{CLR_RED}{CLR_BOLD}ARMED{CLR_RESET}" if ctrl.armed else f"{CLR_GREEN}DISARMED{CLR_RESET}"

        print("\n" + "=" * 70)
        print(f" UDP FLIGHT TESTER | Link: {link_tag} | State: {arm_tag} | Mode: {CLR_CYAN}{ctrl.flight_mode}{CLR_RESET}")
        print("=" * 70)
        print("  [1] Live Vehicle Status & Sensor Diagnostics")
        print("  [2] ARM (Normal: param1=1, param2=0)")
        print("  [3] FORCE ARM (Bench Override: param1=1, param2=21196)")
        print("  [4] DISARM (Normal: param1=0, param2=0)")
        print("  [5] FORCE DISARM (param1=0, param2=21196)")
        print("  [6] Switch Mode -> MANUAL")
        print("  [7] Switch Mode -> OFFBOARD (Auto 20Hz setpoint stream)")
        print("  [8] Switch Mode -> HOLD / AUTO.LOITER")
        print("  [9] Switch Mode -> AUTO.LAND")
        print("  [10] Dispatch TAKEOFF (1.0m altitude)")
        print("  [11] EMERGENCY MOTOR KILL (Flight Termination)")
        print("  [q] Exit")
        print("-" * 70)

        choice = input("Select Action [1-11, q]: ").strip().lower()

        if choice == "q":
            break
        elif choice == "1":
            ctrl.print_status()
        elif choice == "2":
            ctrl.arm(force=False)
        elif choice == "3":
            ctrl.arm(force=True)
        elif choice == "4":
            ctrl.disarm(force=False)
        elif choice == "5":
            ctrl.disarm(force=True)
        elif choice == "6":
            ctrl.set_mode("MANUAL")
        elif choice == "7":
            ctrl.set_mode("OFFBOARD")
        elif choice == "8":
            ctrl.set_mode("HOLD")
        elif choice == "9":
            ctrl.set_mode("LAND")
        elif choice == "10":
            ctrl.takeoff(1.0)
        elif choice == "11":
            confirm = input(f"{CLR_RED}Confirm Emergency Motor Kill? (type 'yes'): {CLR_RESET}").strip().lower()
            if confirm == "yes":
                ctrl.emergency_kill()
            else:
                print("Aborted.")
        else:
            print("Invalid selection.")
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="UDP MAVLink Flight Control & Command ACK Validator")
    parser.add_argument("--host", type=str, default="172.16.101.84", help="Target IP or endpoint (default: 172.16.101.84)")
    parser.add_argument("--port", type=int, default=14550, help="Target Port (default: 14550 for UDP, 5760 for TCP)")
    parser.add_argument("--protocol", type=str, default="udp", choices=["udp", "tcp", "udpout", "udpin"], help="Protocol (default: udp)")
    parser.add_argument("--command", type=str, choices=["status", "arm", "force-arm", "disarm", "force-disarm", "manual", "offboard", "loiter", "land", "takeoff", "kill"], help="Single command to execute non-interactively")
    parser.add_argument("--timeout", type=float, default=4.0, help="Connection / ACK timeout in seconds")

    args = parser.parse_args()

    ctrl = UDPDroneController(host=args.host, port=args.port, protocol=args.protocol)
    if not ctrl.connect():
        sys.exit(1)

    print(f"[WAITING] Awaiting autopilot heartbeat from {args.host}:{args.port}...")
    if not ctrl.wait_for_connection(timeout=args.timeout):
        print(f"{CLR_RED}[ERROR]{CLR_RESET} Could not establish heartbeat with {args.host}:{args.port}.")
        if args.protocol == "udp":
            print(f"{CLR_YELLOW}[HINT]{CLR_RESET} If mavlink-routerd has not been restarted with Mode=server on 14550,")
            print(f"       start the companion bridge: python3 scripts/network/udp_mavlink_bridge.py")
            print(f"       or test directly via TCP: --protocol tcp --port 5760")
        ctrl.disconnect()
        sys.exit(1)

    print(f"{CLR_GREEN}[CONNECTED]{CLR_RESET} Pixhawk Autopilot online! (Mode: {ctrl.flight_mode}, Armed: {ctrl.armed})")

    # Non-interactive command execution
    if args.command:
        cmd = args.command
        if cmd == "status":
            ctrl.print_status()
        elif cmd == "arm":
            ctrl.arm(force=False)
        elif cmd == "force-arm":
            ctrl.arm(force=True)
        elif cmd == "disarm":
            ctrl.disarm(force=False)
        elif cmd == "force-disarm":
            ctrl.disarm(force=True)
        elif cmd == "manual":
            ctrl.set_mode("MANUAL")
        elif cmd == "offboard":
            ctrl.set_mode("OFFBOARD")
        elif cmd == "loiter":
            ctrl.set_mode("HOLD")
        elif cmd == "land":
            ctrl.set_mode("LAND")
        elif cmd == "takeoff":
            ctrl.takeoff(1.0)
        elif cmd == "kill":
            ctrl.emergency_kill()
        time.sleep(0.5)
        ctrl.disconnect()
        sys.exit(0)

    # Interactive mode
    try:
        run_interactive(ctrl)
    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
