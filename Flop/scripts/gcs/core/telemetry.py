"""
================================================================================
MODULE: telemetry.py
PURPOSE: Strongly Typed Central Telemetry Model & PX4 MAVLink State Decoder
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Core Data Layer)
  * Communicates:  MAVLinkWorker, Ros2MapListener, DroneGCS, and all UI widgets
  * Upstream:      Decoded MAVLink packets from Pixhawk 6X (SysID 1)
  * Downstream:    HUD, Motor Gauge, 2D SLAM View, Top Status Strip, CLI Console

DATA FLOW & INTERFACES:
  * Decodes:       HEARTBEAT (modes, armed, connection status),
                   LOCAL_POSITION_NED (x, y, z, vx, vy, vz),
                   ATTITUDE (pitch, roll, yaw, rates),
                   SYS_STATUS (battery voltage, current, capacity, sensor health),
                   SERVO_OUTPUT_RAW (individual actuator PWMs for 4-8 motors),
                   VFR_HUD (airspeed, groundspeed, climb rate, throttle),
                   COMMAND_ACK (command ID, result enum, plain-English message),
                   STATUSTEXT (autopilot safety notifications and warnings).
  * Data Model:    Thread-safe `@dataclass TelemetryState` snapshot.

KEY LOGIC & FAILSAFES:
  * Custom Mode De-aliasing: Unpacks 32-bit PX4 custom_mode bitfields into
    standard flight modes (OFFBOARD, POSCTL, ALTCTL, MANUAL, AUTO.TAKEOFF/LOITER/LAND).
  * Plain-English ACK Translation: Converts cryptic MAV_RESULT integer codes
    (0=ACCEPTED, 1=TEMPORARILY_REJECTED, 2=DENIED) into actionable operator text.
  * Actuator Saturation Profiling: Normalizes raw microsecond PWM signals (1000-2000 us)
    into 0-100% motor load indicators to visually reveal actuator desync or stalls.
  * Stale Sensor Watchdogs: Computes timestamp latency to alert when odometry or
    autopilot heartbeats lapse.

USAGE:
  state = TelemetryState()
  state.update_from_heartbeat(msg)
  state.update_from_local_pos(msg)
  print(f"Mode: {state.flight_mode} | Armed: {state.armed} | Alt: {state.z:.2f}m")
================================================================================
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional


# PX4 Custom Mode Mapping
PX4_CUSTOM_MAIN_MODES = {
    1: "MANUAL",
    2: "ALTCTL",
    3: "POSCTL",
    4: "AUTO",
    5: "ACRO",
    6: "OFFBOARD",
    7: "STABILIZED",
    8: "RATTITUDE",
}

PX4_CUSTOM_SUB_MODES_AUTO = {
    1: "READY",
    2: "TAKEOFF",
    3: "LOITER",
    4: "MISSION",
    5: "RTL",
    6: "LAND",
    7: "RTGS",
    8: "FOLLOW_TARGET",
    9: "PRECLAND",
}

# MAV_RESULT Mapping
MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
    # Rare for the commands this GCS actually sends, but real pymavlink
    # MAV_RESULT values - without these, either one prints as a raw
    # RES_7/RES_8 fallback instead of a name (see last_ack_result below).
    7: "COMMAND_LONG_ONLY",
    8: "COMMAND_INT_ONLY",
}

# SYS_STATUS onboard_control_sensors_* bit for the RC receiver, used to detect
# real RC signal loss (see TelemetrySnapshot.update_rc_health) - proven live
# during this project's ELRS failsafe testing to be the only reliable signal:
# RC_CHANNELS values otherwise freeze in place but keep being reported as if
# nothing were wrong.
MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 1 << 16

# Common MAV_CMD Mapping
MAV_CMD_NAMES = {
    400: "ARM_DISARM",
    176: "DO_SET_MODE",
    22: "NAV_TAKEOFF",
    21: "NAV_LAND",
    20: "NAV_RETURN_TO_LAUNCH",
    185: "EMERGENCY_KILL",
    511: "SET_MESSAGE_INTERVAL",
    209: "DO_MOTOR_TEST",
    310: "ACTUATOR_TEST",
}


def decode_px4_mode(custom_mode: int) -> str:
    """Decode PX4 packed custom_mode integer into human-readable flight mode."""
    if custom_mode == 0:
        return "UNKNOWN"
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF
    main_str = PX4_CUSTOM_MAIN_MODES.get(main_mode, f"MAIN_{main_mode}")
    if main_mode == 4:  # AUTO
        sub_str = PX4_CUSTOM_SUB_MODES_AUTO.get(sub_mode, f"SUB_{sub_mode}")
        return f"AUTO.{sub_str}"
    return main_str


#: Above this height over the floor (m) a disarm is a controlled landing, never a
#: motor cutoff. Below it the vehicle counts as on the ground for auto-disarm.
AIRBORNE_HEIGHT_M = 0.15

#: A downward range reading older than this (s) is not trusted for that decision.
RANGE_MAX_AGE_S = 1.5


@dataclass
class TelemetrySnapshot:
    """Comprehensive snapshot of current vehicle, D435i VIO, and companion state."""
    # Link
    connected: bool = False
    last_heartbeat_time: float = 0.0
    heartbeat_age: float = 999.0
    link_quality: int = 0
    system_id: int = 1
    component_id: int = 1

    # Arming and Modes
    armed: bool = False
    flight_mode: str = "DISCONNECTED"
    base_mode: int = 0
    custom_mode: int = 0
    flight_time_sec: float = 0.0
    arm_timestamp: float = 0.0

    # Power
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_percent: int = 0

    # Attitude (degrees)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    heading: float = 0.0

    # Local Position NED (meters)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    altitude: float = 0.0  # -z AGL from VIO/EKF2
    # Downward rangefinder (DISTANCE_SENSOR, orientation 25) in metres above the
    # floor. NaN until a valid reading arrives - unknown must never read as 0 m.
    range_m: float = float("nan")
    last_range_time: float = 0.0
    last_position_time: float = 0.0
    position_stale: bool = True
    position_age: float = 999.0

    # Velocities (m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    ground_speed: float = 0.0

    # GPS / Global (if available)
    latitude: float = 0.0
    longitude: float = 0.0
    satellites: int = 0
    hdop: float = 1.0
    fix_type: str = "NO_FIX"

    # Intel RealSense D435i VIO & EKF2 Vision Fusion
    d435i_vio_health: bool = False
    d435i_vio_fps: float = 0.0
    d435i_vio_age: float = 999.0
    ekf2_vision_fused: bool = False
    last_vision_time: float = 0.0
    vision_packets_count: int = 0

    # EKF2 horizontal position uncertainty (m^2), from the autopilot's own
    # ODOMETRY stream. NaN until the first valid message - "unknown" must never
    # read as "zero uncertainty". See update_position_covariance.
    pos_var_n: float = float("nan")
    pos_var_e: float = float("nan")
    pos_var_time: float = 0.0

    # Motor / Actuator Telemetry (PWM µs)
    motor_pwms: List[int] = field(default_factory=lambda: [1000, 1000, 1000, 1000])
    # Output-indexed (SERVO_OUTPUT_RAW servo1..4), exactly as received. When the
    # last SERVO_OUTPUT_RAW arrived, so a dead stream is distinguishable from
    # motors that are genuinely sitting still.
    last_motor_time: float = 0.0
    motor_age: float = 999.0

    # Companion Computer Metrics (Radxa Dragon Q6A)
    cpu_percent: float = 0.0
    ram_percent: float = 0.0
    temp_c: float = 0.0

    # Command Execution & Feedback (COMMAND_ACK)
    last_ack_cmd: int = 0
    last_ack_cmd_name: str = ""
    last_ack_result: str = ""
    last_ack_result_code: int = 0
    last_ack_time: float = 0.0

    # Inter-GCS Command Broadcast feedback
    last_cmd_received: str = ""
    last_cmd_timestamp: float = 0.0

    # MAV_LANDED_STATE from EXTENDED_SYS_STATE: 0=UNDEFINED, 1=ON_GROUND,
    # 2=IN_AIR, 3=TAKEOFF, 4=LANDING. Used to distinguish "armed but sitting
    # on the ground" from "genuinely airborne" - the disarm-vs-land safety
    # logic and the move/yaw preconditions both depend on this, not just on
    # `armed` alone.
    landed_state: int = 0

    # RC Link (ExpressLRS receiver). rc_rssi follows the MAVLink RC_CHANNELS
    # convention: 0-254 valid range, 255 = "not reported"/unknown - this specific
    # receiver was confirmed live to always report 255 even on a fully healthy
    # link, so rc_receiver_healthy (from SYS_STATUS, not rssi) is the trustworthy
    # half of link status; rssi is shown only as a best-effort extra number.
    rc_rssi: int = -1
    rc_channel_count: int = 0
    rc_receiver_present: bool = False
    rc_receiver_healthy: bool = False
    last_rc_time: float = 0.0

    @property
    def is_airborne(self) -> bool:
        """True only when PX4 itself reports being off the ground. Falls back
        to an altitude-based guess if landed_state has never been reported
        (e.g. EXTENDED_SYS_STATE not yet received), so a stale UNDEFINED
        state doesn't silently allow unsafe move/yaw commands."""
        if self.landed_state in (2, 3, 4):  # IN_AIR, TAKEOFF, LANDING
            return True
        if self.landed_state == 1:  # ON_GROUND, reported explicitly
            return False
        # landed_state == 0 (UNDEFINED) - never received a real report yet.
        return self.armed and abs(self.z) > 0.15

    def height_over_floor(self, max_age_s: float = RANGE_MAX_AGE_S) -> Optional[float]:
        """Height above the floor from the downward rangefinder, or None when
        there is no fresh valid reading. Deliberately rangefinder-only: -z is
        relative to an EKF origin that can sit away from the floor, so using it
        here could make a grounded vehicle look airborne."""
        if self.last_range_time <= 0.0 or not math.isfinite(self.range_m):
            return None
        if time.time() - self.last_range_time > max_age_s:
            return None
        return self.range_m

    @property
    def needs_controlled_landing(self) -> bool:
        """Should Disarm be a controlled AUTO.LAND instead of a motor cutoff?

        True when PX4 says it is off the ground OR the rangefinder says the
        floor is further than AIRBORNE_HEIGHT_M away - landed_state alone can
        report ON_GROUND while the vehicle is physically up (held in the hand,
        some manual modes, just after lift-off), and a forced disarm there
        would free-fall it."""
        if not self.armed:
            return False
        if self.landed_state in (2, 3, 4):
            return True
        h = self.height_over_floor()
        if h is not None and h > AIRBORNE_HEIGHT_M:
            return True
        return self.is_airborne

    def is_on_ground_for_disarm(self) -> bool:
        """Safe to cut power: PX4 says ON_GROUND and the floor is within
        AIRBORNE_HEIGHT_M (or the rangefinder has no fresh reading)."""
        if self.landed_state != 1:
            return False
        h = self.height_over_floor()
        return h is None or h <= AIRBORNE_HEIGHT_M

    def clone(self) -> TelemetrySnapshot:
        """Create a thread-safe shallow copy with cloned mutable lists."""
        import copy
        cp = copy.copy(self)
        cp.motor_pwms = list(self.motor_pwms)
        return cp

    def update_extended_sys_state(self, msg) -> None:
        """Update landed_state from EXTENDED_SYS_STATE."""
        self.landed_state = getattr(msg, "landed_state", 0)

    def update_heartbeat(self, msg) -> None:
        now = time.time()
        self.last_heartbeat_time = now
        self.heartbeat_age = 0.0
        self.connected = True
        self.base_mode = getattr(msg, "base_mode", 0)
        self.custom_mode = getattr(msg, "custom_mode", 0)
        was_armed = self.armed
        self.armed = bool(self.base_mode & 128)  # MAV_MODE_FLAG_SAFETY_ARMED
        if not was_armed and self.armed:
            self.arm_timestamp = now
        elif not self.armed:
            self.flight_time_sec = 0.0

        if self.armed and self.arm_timestamp > 0:
            self.flight_time_sec = now - self.arm_timestamp

        self.flight_mode = decode_px4_mode(self.custom_mode)

    def update_attitude(self, msg) -> None:
        self.roll = math.degrees(msg.roll)
        self.pitch = math.degrees(msg.pitch)
        self.yaw = math.degrees(msg.yaw)
        self.heading = (self.yaw + 360.0) % 360.0

    def update_local_position(self, msg) -> None:
        now = time.time()
        self.last_position_time = now
        self.position_stale = False
        self.position_age = 0.0
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.altitude = -msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz
        self.ground_speed = math.sqrt(msg.vx**2 + msg.vy**2)

    def update_distance_sensor(self, msg) -> None:
        """Downward rangefinder only (MAV_SENSOR_ORIENTATION 25 = PITCH_270).
        Readings at or beyond max_distance mean 'nothing in range', not height."""
        if getattr(msg, "orientation", -1) != 25:
            return
        d = msg.current_distance * 0.01
        mx = msg.max_distance * 0.01
        self.last_range_time = time.time()
        self.range_m = d if (d >= 0.0 and (mx <= 0.0 or d < mx)) else float("nan")

    def check_position_staleness(self, max_age_sec: float = 2.0) -> None:
        """Flag position as stale if LOCAL_POSITION_NED has stopped arriving."""
        now = time.time()
        if self.last_position_time > 0.0:
            self.position_age = now - self.last_position_time
            self.position_stale = (self.position_age > max_age_sec)
        else:
            self.position_age = 999.0
            self.position_stale = True

    def update_battery(self, msg) -> None:
        if hasattr(msg, "voltage_battery"):
            self.battery_voltage = msg.voltage_battery / 1000.0
        if hasattr(msg, "current_battery") and msg.current_battery != -1:
            self.battery_current = msg.current_battery / 100.0
        if hasattr(msg, "battery_remaining") and msg.battery_remaining != -1:
            self.battery_percent = max(0, min(100, msg.battery_remaining))

    def update_rc_health(self, msg) -> None:
        """Decode the RC_RECEIVER bit out of SYS_STATUS's sensor health bitmask.
        Call only for actual SYS_STATUS messages - BATTERY_STATUS has no such
        field and would otherwise silently re-zero a previously-good reading."""
        present = getattr(msg, "onboard_control_sensors_present", 0)
        health = getattr(msg, "onboard_control_sensors_health", 0)
        self.rc_receiver_present = bool(present & MAV_SYS_STATUS_SENSOR_RC_RECEIVER)
        self.rc_receiver_healthy = (
            bool(health & MAV_SYS_STATUS_SENSOR_RC_RECEIVER) if self.rc_receiver_present else False
        )

    def update_rc_channels(self, msg) -> None:
        """Update RC receiver telemetry from RC_CHANNELS."""
        self.rc_rssi = getattr(msg, "rssi", 255)
        self.rc_channel_count = getattr(msg, "chancount", 0)
        self.last_rc_time = time.time()

    def update_gps(self, msg) -> None:
        self.latitude = msg.lat / 1e7
        self.longitude = msg.lon / 1e7
        self.satellites = getattr(msg, "satellites_visible", 0)
        eph = getattr(msg, "eph", 100)
        self.hdop = eph / 100.0
        fix = getattr(msg, "fix_type", 0)
        fix_map = {0: "NO_GPS", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX", 4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}
        self.fix_type = fix_map.get(fix, f"FIX_{fix}")

    def update_servo_output(self, msg) -> None:
        """Update live motor PWMs (Motors 1 to 4) from SERVO_OUTPUT_RAW."""
        pwms = [
            getattr(msg, "servo1_raw", 1000),
            getattr(msg, "servo2_raw", 1000),
            getattr(msg, "servo3_raw", 1000),
            getattr(msg, "servo4_raw", 1000),
        ]
        self.motor_pwms = pwms
        self.last_motor_time = time.time()
        self.motor_age = 0.0

    def check_motor_staleness(self) -> None:
        """Refresh ``motor_age``. Called every UI tick, like the position check,
        so the age keeps growing after SERVO_OUTPUT_RAW stops arriving."""
        if self.last_motor_time > 0.0:
            self.motor_age = max(0.0, time.time() - self.last_motor_time)
        else:
            self.motor_age = 999.0

    def update_command_ack(self, msg) -> None:
        """Decode COMMAND_ACK response from Pixhawk."""
        self.last_ack_cmd = getattr(msg, "command", 0)
        self.last_ack_cmd_name = MAV_CMD_NAMES.get(self.last_ack_cmd, f"CMD_{self.last_ack_cmd}")
        self.last_ack_result_code = getattr(msg, "result", 0)
        self.last_ack_result = MAV_RESULT_NAMES.get(self.last_ack_result_code, f"RES_{self.last_ack_result_code}")
        self.last_ack_time = time.time()

    def update_vision_estimate(self, msg) -> None:
        """Update Intel RealSense D435i VIO tracking state."""
        now = time.time()
        self.last_vision_time = now
        self.d435i_vio_age = 0.0
        self.d435i_vio_health = True
        self.ekf2_vision_fused = True
        self.vision_packets_count += 1

    # MAV_ESTIMATOR_TYPE_AUTOPILOT. Written out rather than imported so this
    # module stays importable without pymavlink (the hermetic test job).
    ESTIMATOR_TYPE_AUTOPILOT = 8

    def update_position_covariance(self, msg) -> None:
        """Take the EKF's horizontal position variance from an ODOMETRY message.

        PX4 v1.17 (streams/ODOMETRY.hpp) fills only the diagonal of
        pose_covariance: [0] = North variance, [6] = East variance, everything
        else NaN. Only messages from the autopilot's own estimator are used -
        the companion's VIO odometry can arrive as ODOMETRY too, through
        mavlink-router, and its covariance describes the tracker, not the
        estimate the vehicle is actually flying on. Non-finite or negative
        values are ignored rather than stored, so a bad packet leaves the last
        good value to age out instead of drawing a nonsense ring.
        """
        if getattr(msg, "estimator_type", None) != self.ESTIMATOR_TYPE_AUTOPILOT:
            return
        cov = getattr(msg, "pose_covariance", None)
        if cov is None or len(cov) < 7:
            return
        var_n, var_e = float(cov[0]), float(cov[6])
        if not (math.isfinite(var_n) and math.isfinite(var_e)) or var_n < 0 or var_e < 0:
            return
        self.pos_var_n = var_n
        self.pos_var_e = var_e
        self.pos_var_time = time.time()

    def check_vision_staleness(self, max_age_sec: float = 3.0) -> None:
        """Age out Intel RealSense D435i vision tracking if no recent packets arrived."""
        now = time.time()
        if self.last_vision_time > 0.0:
            self.d435i_vio_age = now - self.last_vision_time
            if self.d435i_vio_age > max_age_sec:
                self.d435i_vio_health = False
                self.ekf2_vision_fused = False
            else:
                self.d435i_vio_health = True
                self.ekf2_vision_fused = True
        else:
            self.d435i_vio_age = 999.0
            self.d435i_vio_health = False
            self.ekf2_vision_fused = False

