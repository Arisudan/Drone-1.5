"""
================================================================================
MODULE: mavlink_worker.py
PURPOSE: Asynchronous QThread MAVLink Protocol Engine & Offboard Setpoint Streamer
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Networking Thread)
  * Communicates:  Pixhawk 6X via Radxa mavlink-routerd (tcp:172.16.101.84:5760)
  * Upstream:      GCS UI actions, waypoint commands, and OFFBOARD setpoints
  * Downstream:    Qt GUI Main Thread (via thread-safe PyQt5 Signals)

DATA FLOW & INTERFACES:
  * MAVLink Out:   HEARTBEAT (SysID 255), MAV_CMD_DO_SET_MODE,
                   MAV_CMD_COMPONENT_ARM_DISARM (normal & force 21196),
                   SET_POSITION_TARGET_LOCAL_NED (20 Hz daemon),
                   MAV_CMD_SET_MESSAGE_INTERVAL (telemetry rates).
  * MAVLink In:    HEARTBEAT, LOCAL_POSITION_NED, ATTITUDE, SYS_STATUS,
                   SERVO_OUTPUT_RAW, STATUSTEXT, COMMAND_ACK.
  * Qt Signals:    telemetry_updated(TelemetrySnapshot),
                   connection_changed(bool, str),
                   command_ack_received(cmd_id, result_code, cmd_name, result_str),
                   statustext_received(str, severity),
                   rates_updated(rx_rate, tx_rate).

KEY LOGIC & FAILSAFES:
  * Non-Blocking Architecture: Runs all network socket I/O in a dedicated QThread
    so GUI animations, map zooming, and HUD rendering never stutter or freeze.
  * Continuous 20 Hz Setpoint Pre-Streaming: PX4 rejects OFFBOARD mode if setpoints
    are not streaming before mode engagement. This worker starts the setpoint stream
    before dispatching MAV_CMD_DO_SET_MODE(OFFBOARD).
  * Auto-Reconnection & Stream Renegotiation: Detects Wi-Fi drops, retries socket
    connects every 2.0s, and re-requests message streaming rates upon reconnect.
  * Inter-Client Broadcast Interception: Detects commands sent from companion SBC
    or secondary consoles to keep the GCS UI synchronized with fleet operations.

USAGE:
  worker = MAVLinkWorker(host="172.16.101.84", port=5760, source_system=255)
  worker.telemetry_updated.connect(self.on_telemetry)
  worker.command_ack_received.connect(self.on_ack)
  worker.start()
================================================================================
"""

from __future__ import annotations
import math
import socket
import struct
import threading
import time
from typing import Optional, Tuple

from PyQt5.QtCore import QThread, pyqtSignal
from pymavlink import mavutil

from core.telemetry import TelemetrySnapshot, MAV_CMD_NAMES, MAV_RESULT_NAMES


class MAVLinkWorker(QThread):
    """Worker thread running continuous MAVLink RX/TX loops."""

    # Qt Signals
    telemetry_updated = pyqtSignal(object)              # Emits TelemetrySnapshot
    connection_changed = pyqtSignal(bool, str)          # Emits (connected, message)
    command_broadcast_received = pyqtSignal(str)        # Emits intercepted [GCS CMD]
    statustext_received = pyqtSignal(str, int)          # Emits (text, severity)
    command_ack_received = pyqtSignal(int, int, str, str)  # Emits (cmd_id, result_code, cmd_name, result_str)
    rates_updated = pyqtSignal(float, float)            # Emits (rx_rate, tx_rate)

    def __init__(
        self,
        host: str = "172.16.101.84",
        port: int = 14550,
        protocol: str = "udp",
        source_system: int = 255
    ):
        super().__init__()
        import os
        env_host = os.environ.get("GCS_HOST", "").strip()
        if env_host:
            host = env_host
        env_proto = os.environ.get("GCS_PROTOCOL", "").strip()
        if env_proto:
            protocol = env_proto

        self.host: str = host
        self.port: int = int(port)
        self.protocol: str = protocol.lower().strip()
        self.source_system: int = source_system  # 255 for GCS, 254 for Radxa monitor
        self.running: bool = False
        self._connected: bool = False
        self._streams_configured: bool = False

        self.master: Optional[mavutil.mavfile] = None
        # Dedup state for COMMAND_ACK, keyed per dispatch rather than per time window.
        # PX4 (or the link) can re-transmit the same logical ACK as multiple genuinely
        # distinct wire packets - different MAVLink sequence numbers each time, a real
        # retransmission, not just a duplicated UDP datagram - observed live as ~70
        # identical ACKs for one NAV_TAKEOFF. But a shared MAV_CMD id (e.g.
        # MAV_CMD_SET_MESSAGE_INTERVAL=511) is also legitimately reused for several
        # DIFFERENT dispatches close together (one _configure_streams() burst sends it
        # ~7 times, once per telemetry stream), which a content+time-window dedup
        # would incorrectly collapse into one. A monotonically bumped token per
        # cmd_id, incremented right before each real send (see _begin_command_dispatch
        # below), tells these apart: only the first ACK matching the *current* token
        # for that cmd_id is reported; further ACKs against the same token are true
        # retransmit duplicates and are suppressed, while a fresh dispatch of the same
        # command type always gets its own fresh token and is reported once more.
        self._ack_dispatch_token: dict = {}
        self._ack_reported_token: dict = {}
        self.telemetry = TelemetrySnapshot()
        self.telemetry.system_id = 1
        self.telemetry.component_id = 1
        self.telemetry_lock = threading.Lock()

        # Target IDs
        self.target_system: int = 1
        self.target_component: int = 1

        # Setpoint streaming state (for OFFBOARD control)
        self.streaming_setpoints: bool = False
        # "idle_attitude" holds near-zero thrust so arming in OFFBOARD doesn't
        # itself command a climb; only takeoff()/move_*() switch to "position".
        self.setpoint_kind: str = "idle_attitude"
        self.sp_x: float = 0.0
        self.sp_y: float = 0.0
        self.sp_z: float = 0.0
        self.sp_yaw: float = 0.0
        self.sp_lock = threading.Lock()

        # Stats
        self.rx_count: int = 0
        self.tx_count: int = 0
        self._last_rate_time: float = time.time()

    def set_endpoint(self, host: str, port: int, protocol: str = "udp"):
        """Update endpoint for subsequent reconnects."""
        self.host = host.strip()
        self.port = int(port)
        self.protocol = protocol.lower().strip()

    def _begin_command_dispatch(self, cmd_id: int):
        """Call immediately before sending a MAV_CMD - see the dedup comment on
        _ack_dispatch_token in __init__ for why this exists."""
        self._ack_dispatch_token[cmd_id] = self._ack_dispatch_token.get(cmd_id, 0) + 1

    def connect_endpoint(self, host: str, port: int, protocol: str = "udp"):
        """Configure endpoint and start thread."""
        self.set_endpoint(host, port, protocol)
        if not self.isRunning():
            self.running = True
            self.start()

    def disconnect_endpoint(self):
        """Stop worker and close connection cleanly."""
        self.running = False
        self.streaming_setpoints = False
        self.wait(1000)
        self._close_connection()
        self._connected = False
        self._streams_configured = False
        self.connection_changed.emit(False, "Disconnected by user")

    def _close_connection(self):
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

    def request_message_interval(self, message_id: int, frequency_hz: float):
        """Send MAV_CMD_SET_MESSAGE_INTERVAL to negotiate stream frequency."""
        if not self.master:
            return
        interval_us = int(1e6 / frequency_hz) if frequency_hz > 0 else -1
        try:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0, 0, 0, 0, 0
            )
            self.tx_count += 1
        except Exception:
            pass

    def _configure_streams(self):
        """Request all required telemetry streams at high refresh rates."""
        if self._streams_configured or not self.master:
            return
        # Stream negotiations:
        # LOCAL_POSITION_NED: 10 Hz
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0)
        # ATTITUDE: 10 Hz
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10.0)
        # SERVO_OUTPUT_RAW: 5 Hz
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 5.0)
        # SYS_STATUS: 2 Hz
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2.0)
        # BATTERY_STATUS: 2 Hz
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 2.0)
        # ODOMETRY / VISION_POSITION_ESTIMATE: 10 Hz
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ODOMETRY, 10.0)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, 10.0)
        # EXTENDED_SYS_STATE: 2 Hz - landed_state drives the disarm-vs-land
        # safety logic and the move/yaw airborne precondition.
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2.0)
        # RC_CHANNELS: 2 Hz - RC link status badge (rssi + SYS_STATUS health bit).
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 2.0)
        self._streams_configured = True

    def run(self):
        """Worker thread entry point. Owns the reconnect loop: an initial connect
        failure, a heartbeat timeout, or a dead socket during recv() all tear down
        self.master and fall through to the reconnect branch below instead of
        exiting the thread or spinning on a zombie socket - retrying every
        RECONNECT_INTERVAL_S seconds until self.running is cleared. Without this,
        a Wi-Fi drop to the Radxa (the actual recurring failure mode observed
        against this link) left the thread alive but permanently unable to
        recover without the operator manually hitting reconnect in the GUI."""
        RECONNECT_INTERVAL_S = 2.0

        if str(self.host).startswith(("tcp:", "udp:", "udpout:", "udpin:")):
            conn_str = str(self.host)
        elif self.protocol in ("udp", "udpout"):
            conn_str = f"udpout:{self.host}:{self.port}"
        elif self.protocol == "udpin":
            conn_str = f"udpin:0.0.0.0:{self.port}"
        else:
            conn_str = f"tcp:{self.host}:{self.port}"

        self.running = True
        self._connected = False
        self.telemetry.connected = False
        self._streams_configured = False
        self.master = None
        last_heartbeat_tx = 0.0
        last_setpoint_tx = 0.0
        self.rx_count = 0
        self.tx_count = 0
        self._last_rate_time = time.time()
        last_connect_attempt = 0.0
        first_attempt = True

        while self.running:
            now = time.time()

            # 0. (Re)connect whenever we don't currently hold a live socket.
            if self.master is None:
                if now - last_connect_attempt < RECONNECT_INTERVAL_S:
                    time.sleep(0.05)
                    continue
                last_connect_attempt = now
                verb = "Connecting to" if first_attempt else "Reconnecting to"
                first_attempt = False
                self.connection_changed.emit(False, f"{verb} {conn_str}...")
                try:
                    self.master = mavutil.mavlink_connection(
                        conn_str,
                        source_system=self.source_system,
                        source_component=190 if self.source_system == 255 else 197,
                    )
                    self._streams_configured = False
                    self.rx_count = 0
                    self.tx_count = 0
                    self._last_rate_time = time.time()
                    last_heartbeat_tx = 0.0
                    last_setpoint_tx = 0.0
                except Exception as e:
                    self.master = None
                    self.connection_changed.emit(
                        False, f"Socket error: {e} - retrying in {RECONNECT_INTERVAL_S:.0f}s"
                    )
                continue

            # 1. Send 1 Hz GCS Heartbeat
            if now - last_heartbeat_tx >= 1.0:
                self._send_heartbeat()
                last_heartbeat_tx = now

            # 2. Stream 20 Hz OFFBOARD Setpoints (if active)
            if self.streaming_setpoints and (now - last_setpoint_tx >= 0.05):
                self._send_setpoint()
                last_setpoint_tx = now

            # 3. Calculate message throughput rates every second
            if now - self._last_rate_time >= 1.0:
                elapsed = now - self._last_rate_time
                rx_rate = self.rx_count / elapsed
                tx_rate = self.tx_count / elapsed
                self.rates_updated.emit(rx_rate, tx_rate)
                self.rx_count = 0
                self.tx_count = 0
                self._last_rate_time = now

                # Liveness check - tear the socket down on timeout so the
                # reconnect branch above rebuilds it fresh, instead of leaving
                # a stale-but-still-"connected" master that never recovers.
                if self._connected and (now - self.telemetry.last_heartbeat_time > 4.0):
                    lost_for = now - self.telemetry.last_heartbeat_time
                    self._connected = False
                    self.telemetry.connected = False
                    self._streams_configured = False
                    self._close_connection()
                    self.connection_changed.emit(False, f"Heartbeat lost ({lost_for:.1f}s ago) - reconnecting...")
                    continue

            # 4. Receive incoming MAVLink packets (non-blocking slice)
            try:
                msg = self.master.recv_match(blocking=False)
                if msg is not None:
                    self.rx_count += 1
                    try:
                        self._handle_msg(msg)
                    except Exception as e:
                        # Item 23: Explicit error logging inside message parser
                        print(f"[MAVLink Parse Error] {msg.get_type()}: {e}")
                else:
                    time.sleep(0.005)
            except Exception as e:
                # A genuinely dead socket (e.g. TCP RST / "Connection reset by
                # peer") raises here on every subsequent poll if left alone -
                # tear it down so the reconnect branch above rebuilds it instead
                # of silently spinning forever.
                self._connected = False
                self.telemetry.connected = False
                self._streams_configured = False
                self._close_connection()
                self.connection_changed.emit(False, f"Link error: {e} - reconnecting...")

        self._close_connection()
        self._connected = False
        self.connection_changed.emit(False, "Worker stopped")

    def _emit_telemetry(self):
        """Item 25: Thread-safe defensive copying for TelemetrySnapshot emission."""
        with self.telemetry_lock:
            snap = self.telemetry.clone()
        self.telemetry_updated.emit(snap)

    def _send_heartbeat(self):
        """Send GCS Heartbeat to maintain mavlink-router routing entries."""
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

    def _send_setpoint(self):
        """Dispatch the 20 Hz OFFBOARD setpoint as whichever kind is active."""
        if self.setpoint_kind == "idle_attitude":
            self._send_idle_attitude()
        else:
            self._send_position_setpoint()

    def _send_idle_attitude(self):
        """Stream a near-zero-thrust attitude setpoint. Satisfies OFFBOARD's
        requirement for a live setpoint stream without asking the position
        controller to hold/fly anywhere, so an armed vehicle stays grounded
        until an explicit takeoff/move command switches to position control."""
        if not self.master:
            return
        try:
            q = [1.0, 0.0, 0.0, 0.0]  # level attitude, no rotation
            type_mask = 0b00000111    # ignore body roll/pitch/yaw rate; use attitude+thrust
            self.master.mav.set_attitude_target_send(
                0, self.target_system, self.target_component,
                type_mask, q, 0.0, 0.0, 0.0, 0.0  # thrust = 0.0
            )
            self.tx_count += 1
        except Exception:
            pass

    def _send_position_setpoint(self):
        """Send NED position setpoint at 20 Hz."""
        if not self.master:
            return
        with self.sp_lock:
            x, y, z, yaw = self.sp_x, self.sp_y, self.sp_z, self.sp_yaw
        try:
            # Type mask: Ignore velocity, acc, yaw_rate (0b0000_1011_1111_1000 = 0x0DF8)
            type_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            )
            self.master.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                x, y, z,
                0, 0, 0,
                0, 0, 0,
                yaw, 0
            )
            self.tx_count += 1
        except Exception:
            pass

    def _handle_msg(self, msg):
        """Process incoming MAVLink messages and update telemetry."""
        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT":
            autopilot = getattr(msg, "autopilot", 0)
            # Filter for real autopilot heartbeats (ignore mirror GCS heartbeats)
            if autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                src_sys = msg.get_srcSystem()
                src_comp = msg.get_srcComponent()
                self.target_system = src_sys
                self.target_component = src_comp
                was_connected = self._connected
                with self.telemetry_lock:
                    self.telemetry.update_heartbeat(msg)
                self._connected = True
                if not was_connected:
                    self.connection_changed.emit(True, f"Connected to PX4 (SysID {src_sys}, CompID {src_comp})")
                    self._configure_streams()
                self._emit_telemetry()

        elif msg_type == "ATTITUDE":
            with self.telemetry_lock:
                self.telemetry.update_attitude(msg)
            self._emit_telemetry()

        elif msg_type == "LOCAL_POSITION_NED":
            with self.telemetry_lock:
                self.telemetry.update_local_position(msg)
            self._emit_telemetry()

        elif msg_type == "SERVO_OUTPUT_RAW":
            with self.telemetry_lock:
                self.telemetry.update_servo_output(msg)
            self._emit_telemetry()

        elif msg_type == "SYS_STATUS":
            with self.telemetry_lock:
                self.telemetry.update_battery(msg)
                self.telemetry.update_rc_health(msg)
            self._emit_telemetry()

        elif msg_type == "BATTERY_STATUS":
            with self.telemetry_lock:
                self.telemetry.update_battery(msg)
            self._emit_telemetry()

        elif msg_type == "RC_CHANNELS":
            with self.telemetry_lock:
                self.telemetry.update_rc_channels(msg)
            self._emit_telemetry()

        elif msg_type == "EXTENDED_SYS_STATE":
            with self.telemetry_lock:
                self.telemetry.update_extended_sys_state(msg)
            self._emit_telemetry()

        elif msg_type in ("GPS_RAW_INT", "GLOBAL_POSITION_INT"):
            with self.telemetry_lock:
                self.telemetry.update_gps(msg)
            self._emit_telemetry()

        elif msg_type in ("ODOMETRY", "VISION_POSITION_ESTIMATE"):
            with self.telemetry_lock:
                self.telemetry.update_vision_estimate(msg)
            self._emit_telemetry()

        elif msg_type == "COMMAND_ACK":
            # Item 28: Verify ACK is from our target Pixhawk (SysID)
            if self.target_system > 0 and msg.get_srcSystem() != self.target_system:
                return
            with self.telemetry_lock:
                self.telemetry.update_command_ack(msg)
                cmd_id = self.telemetry.last_ack_cmd
                res_code = self.telemetry.last_ack_result_code
                cmd_name = self.telemetry.last_ack_cmd_name
                res_str = self.telemetry.last_ack_result

            # Suppress true retransmit duplicates for the dispatch currently in
            # flight for this cmd_id, while still reporting the first ACK of every
            # fresh dispatch (see _begin_command_dispatch / class docstring above).
            token = self._ack_dispatch_token.get(cmd_id)
            if token is not None and self._ack_reported_token.get(cmd_id) == token:
                return
            self._ack_reported_token[cmd_id] = token

            self.command_ack_received.emit(cmd_id, res_code, cmd_name, res_str)

        elif msg_type == "STATUSTEXT":
            # Item 24: Safe decode of STATUSTEXT bytes before string operations
            raw_text = getattr(msg, "text", "")
            if isinstance(raw_text, bytes):
                text = raw_text.decode("utf-8", errors="ignore").rstrip("\x00")
            elif isinstance(raw_text, (list, tuple, bytearray)):
                text = bytes(raw_text).decode("utf-8", errors="ignore").rstrip("\x00")
            else:
                text = str(raw_text).rstrip("\x00")

            severity = getattr(msg, "severity", 6)
            self.statustext_received.emit(text, severity)

            # Check for inter-GCS broadcast command
            if text.startswith("[GCS CMD]"):
                cmd_content = text.replace("[GCS CMD]", "").strip()
                with self.telemetry_lock:
                    self.telemetry.last_cmd_received = cmd_content
                    self.telemetry.last_cmd_timestamp = time.time()
                self.command_broadcast_received.emit(cmd_content)

    # -------------------------------------------------------------------------
    # Command Dispatch Helpers
    # -------------------------------------------------------------------------

    def broadcast_cmd(self, cmd_text: str):
        """Broadcast command string to peer clients via STATUSTEXT."""
        if not self._connected or not self.master:
            return
        try:
            full_text = f"[GCS CMD] {cmd_text.strip()}"
            chunks = [full_text[i:i+48] for i in range(0, len(full_text), 48)]
            for ch in chunks:
                self.master.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_NOTICE,
                    ch.encode("utf-8")
                )
                self.tx_count += 1
        except Exception as e:
            print(f"[MAVLink TX Error] broadcast_cmd: {e}")

    def arm(self, force: bool = False):
        """Send MAV_CMD_COMPONENT_ARM_DISARM with bench force support."""
        if not self._connected or not self.master:
            return
        param2 = 21196 if force else 0
        try:
            self._begin_command_dispatch(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,  # Arm
                param2,
                0, 0, 0, 0, 0
            )
            self.tx_count += 1
            self.broadcast_cmd("arm force" if force else "arm")
        except Exception as e:
            self.connection_changed.emit(False, f"Arm transmit error: {e}")

    def disarm(self, force: bool = False):
        """Send MAV_CMD_COMPONENT_ARM_DISARM (disarm)."""
        if not self._connected or not self.master:
            return
        param2 = 21196 if force else 0
        try:
            self._begin_command_dispatch(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,  # Disarm
                param2,
                0, 0, 0, 0, 0
            )
            self.tx_count += 1
            self.broadcast_cmd("disarm")
            # Reset to idle so a future re-arm never inherits a stale climb setpoint
            self.setpoint_kind = "idle_attitude"
        except Exception as e:
            self.connection_changed.emit(False, f"Disarm transmit error: {e}")

    def set_mode(self, mode_str: str):
        """Set PX4 flight mode with setpoint pre-streaming for OFFBOARD."""
        if not self._connected or not self.master:
            return
        mode_str = mode_str.upper().strip()
        custom_mode = 0

        if mode_str == "OFFBOARD":
            custom_mode = 6 << 16  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
            # Prime setpoints to current coordinates (used only if/when something
            # later switches setpoint_kind to "position" without setting its own sp_*)
            with self.sp_lock:
                self.sp_x = self.telemetry.x
                self.sp_y = self.telemetry.y
                self.sp_z = self.telemetry.z
                self.sp_yaw = math.radians(self.telemetry.heading)
            # Arm into idle attitude, not a position hold - see setpoint_kind's
            # docstring: entering OFFBOARD must not itself command a climb.
            self.setpoint_kind = "idle_attitude"
            self.streaming_setpoints = True
            # Pre-send 20 Hz setpoint frames for 500ms (Milestone 12 specification) so PX4 accepts OFFBOARD switch
            for _ in range(10):
                self._send_setpoint()
                time.sleep(0.05)
        elif mode_str in ("POSCTL", "POSITION"):
            custom_mode = 3 << 16
            self.streaming_setpoints = False
        elif mode_str in ("ALTCTL", "ALTITUDE"):
            custom_mode = 2 << 16
            self.streaming_setpoints = False
        elif mode_str == "MANUAL":
            custom_mode = 1 << 16
            self.streaming_setpoints = False
        elif mode_str == "ACRO":
            custom_mode = 5 << 16
            self.streaming_setpoints = False
        elif mode_str == "STABILIZED":
            custom_mode = 7 << 16
            self.streaming_setpoints = False
        elif mode_str in ("HOLD", "AUTO.LOITER", "LOITER"):
            custom_mode = (4 << 16) | (3 << 24)  # AUTO.LOITER
            self.streaming_setpoints = False
        elif mode_str in ("LAND", "AUTO.LAND"):
            custom_mode = (4 << 16) | (6 << 24)  # AUTO.LAND
            self.streaming_setpoints = False
        elif mode_str in ("RTL", "AUTO.RTL"):
            custom_mode = (4 << 16) | (5 << 24)  # AUTO.RTL
            self.streaming_setpoints = False

        if custom_mode != 0:
            main_mode = (custom_mode >> 16) & 0xFF
            sub_mode = (custom_mode >> 24) & 0xFF
            try:
                # 1. Send via MAV_CMD_DO_SET_MODE to elicit real COMMAND_ACK from Pixhawk
                self._begin_command_dispatch(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    main_mode,
                    sub_mode,
                    0, 0, 0, 0
                )
                # 2. Also send standard set_mode_send
                self.master.mav.set_mode_send(
                    self.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    custom_mode
                )
                self.tx_count += 2
                self.broadcast_cmd(f"mode {mode_str}")
            except Exception as e:
                self.connection_changed.emit(False, f"Set mode transmit error: {e}")

    def takeoff(self, altitude: float = 1.0):
        """Initiate robust takeoff sequence using MAV_CMD_NAV_TAKEOFF."""
        if not self._connected or not self.master:
            return
        try:
            # 1. Send PX4 standard takeoff command
            self._begin_command_dispatch(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0, 0, 0,
                altitude
            )
            self.tx_count += 1

            # 2. Also prime OFFBOARD setpoints as backup
            with self.sp_lock:
                self.sp_x = self.telemetry.x
                self.sp_y = self.telemetry.y
                self.sp_z = -abs(altitude)
                self.sp_yaw = math.radians(self.telemetry.heading)
            self.setpoint_kind = "position"
            self.streaming_setpoints = True
            self.broadcast_cmd(f"takeoff {altitude:.1f}")
        except Exception as e:
            self.connection_changed.emit(False, f"Takeoff transmit error: {e}")

    def move_delta(self, dx: float, dy: float, dz: float, dyaw: float = 0.0):
        """Update target position relative to current position and ensure OFFBOARD mode."""
        if not self._connected or not self.master:
            return
        if self.telemetry.flight_mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
        with self.sp_lock:
            self.sp_x = self.telemetry.x + dx
            self.sp_y = self.telemetry.y + dy
            self.sp_z = self.telemetry.z + dz
            self.sp_yaw = math.radians(self.telemetry.heading + dyaw)
        self.setpoint_kind = "position"
        self.streaming_setpoints = True
        self.broadcast_cmd(f"move {dx:.2f} {dy:.2f} {dz:.2f}")

    def rotate_yaw(self, dyaw_deg: float):
        """Rotate heading relative to current heading and ensure OFFBOARD mode."""
        if not self._connected or not self.master:
            return
        if self.telemetry.flight_mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
        with self.sp_lock:
            self.sp_yaw = math.radians(self.telemetry.heading + dyaw_deg)
            self.sp_x = self.telemetry.x
            self.sp_y = self.telemetry.y
            self.sp_z = self.telemetry.z
        self.setpoint_kind = "position"
        self.streaming_setpoints = True
        self.broadcast_cmd(f"yaw {dyaw_deg:+.1f}")

    def move_to_waypoint(self, x: float, y: float, z: Optional[float] = None, yaw_deg: Optional[float] = None):
        """Send offboard setpoint to absolute NED coordinates and ensure OFFBOARD mode."""
        if not self._connected or not self.master:
            return
        if self.telemetry.flight_mode != "OFFBOARD":
            self.set_mode("OFFBOARD")
        with self.sp_lock:
            self.sp_x = x
            self.sp_y = y
            self.sp_z = z if z is not None else self.telemetry.z
            if yaw_deg is not None:
                self.sp_yaw = math.radians(yaw_deg)
        self.setpoint_kind = "position"
        self.streaming_setpoints = True
        self.broadcast_cmd(f"waypoint {x:.2f} {y:.2f} {self.sp_z:.2f}")

    def emergency_kill(self):
        """Immediately cut motor outputs via emergency force-disarm and termination."""
        if not self.master:
            return

        try:
            # 1. Primary effective motor cutoff: Force-disarm (param2=21196.0 bypasses in-air checks)
            self._begin_command_dispatch(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0.0,      # 0 = Disarm
                21196.0,  # PX4 magic force-disarm param
                0, 0, 0, 0, 0
            )

            # 2. Secondary best-effort send: Flight termination (effective if circuit breaker allows)
            self._begin_command_dispatch(mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION)
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
                0,
                1.0,  # 1 = Terminate immediately
                0, 0, 0, 0, 0, 0
            )
            self.tx_count += 2
            self.broadcast_cmd("EMERGENCY KILL")
        except Exception as e:
            print(f"[MAVLink TX Error] emergency_kill: {e}")
