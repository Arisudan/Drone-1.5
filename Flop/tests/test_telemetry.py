"""Tests for scripts/gcs/core/telemetry.py - MAVLink decoding and flight state.

Hermetic: stdlib only (telemetry.py imports nothing beyond math/time/dataclasses).

The landed_state matrix below is the one that matters most: `is_airborne` gates
the disarm-vs-AUTO.LAND redirect and the move/yaw preconditions, so getting it
wrong means either refusing a valid command or cutting the motors in mid-air.
"""

import time
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core.telemetry import (
    MAV_CMD_NAMES, MAV_RESULT_NAMES, MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
    TelemetrySnapshot, decode_px4_mode,
)


class Msg:
    """Stand-in for a decoded pymavlink message (attribute bag)."""

    def __init__(self, **kw):
        self.__dict__.update(kw)


def custom_mode(main: int, sub: int = 0) -> int:
    """Pack a PX4 custom_mode the way the autopilot does."""
    return (main << 16) | (sub << 24)


class ModeDecodeTest(unittest.TestCase):
    def test_zero_is_unknown_not_a_real_mode(self):
        # PX4 sends custom_mode=0 before it has decided; decoding it as
        # MAIN_0 would put a fake mode in the header badge.
        self.assertEqual(decode_px4_mode(0), "UNKNOWN")

    def test_simple_main_modes(self):
        self.assertEqual(decode_px4_mode(custom_mode(1)), "MANUAL")
        self.assertEqual(decode_px4_mode(custom_mode(2)), "ALTCTL")
        self.assertEqual(decode_px4_mode(custom_mode(3)), "POSCTL")
        self.assertEqual(decode_px4_mode(custom_mode(6)), "OFFBOARD")
        self.assertEqual(decode_px4_mode(custom_mode(7)), "STABILIZED")

    def test_auto_submodes_are_qualified(self):
        self.assertEqual(decode_px4_mode(custom_mode(4, 2)), "AUTO.TAKEOFF")
        self.assertEqual(decode_px4_mode(custom_mode(4, 3)), "AUTO.LOITER")
        self.assertEqual(decode_px4_mode(custom_mode(4, 5)), "AUTO.RTL")
        self.assertEqual(decode_px4_mode(custom_mode(4, 6)), "AUTO.LAND")

    def test_unknown_codes_stay_visible_rather_than_silently_mapping(self):
        self.assertEqual(decode_px4_mode(custom_mode(4, 99)), "AUTO.SUB_99")
        self.assertEqual(decode_px4_mode(custom_mode(200)), "MAIN_200")

    def test_sub_mode_is_ignored_outside_auto(self):
        # A stale sub-mode field must not turn OFFBOARD into something else.
        self.assertEqual(decode_px4_mode(custom_mode(6, 5)), "OFFBOARD")


class IsAirborneTest(unittest.TestCase):
    """MAV_LANDED_STATE: 0=UNDEFINED 1=ON_GROUND 2=IN_AIR 3=TAKEOFF 4=LANDING."""

    def test_px4_reported_airborne_states_win_outright(self):
        for state in (2, 3, 4):
            for armed in (True, False):
                t = TelemetrySnapshot()
                t.landed_state = state
                t.armed = armed
                t.z = 0.0
                self.assertTrue(t.is_airborne,
                                f"landed_state={state} armed={armed}")

    def test_on_ground_is_believed_even_when_armed_and_high(self):
        # PX4's own land detector outranks an altitude guess: trusting z here
        # would keep an on-ground vehicle in the "airborne" branch and route a
        # disarm into AUTO.LAND that can never complete.
        t = TelemetrySnapshot()
        t.landed_state = 1
        t.armed = True
        t.z = -5.0
        self.assertFalse(t.is_airborne)

    def test_undefined_falls_back_to_armed_plus_altitude(self):
        cases = [
            # (armed, z, expected)
            (True, -1.0, True),     # armed and 1 m up
            (True, -0.10, False),   # armed but inside the 15 cm deadband
            (True, 0.0, False),     # armed on the ground
            (False, -5.0, False),   # disarmed: cannot be flying
        ]
        for armed, z, expected in cases:
            t = TelemetrySnapshot()
            t.landed_state = 0
            t.armed = armed
            t.z = z
            self.assertEqual(t.is_airborne, expected,
                             f"armed={armed} z={z}")

    def test_deadband_is_symmetric_in_z_sign(self):
        # z is NED (negative = up), but a positive z of the same magnitude is
        # still "not on the ground" as far as the guard is concerned.
        t = TelemetrySnapshot()
        t.landed_state = 0
        t.armed = True
        t.z = 1.0
        self.assertTrue(t.is_airborne)

    def test_update_extended_sys_state_sets_it(self):
        t = TelemetrySnapshot()
        t.update_extended_sys_state(Msg(landed_state=2))
        self.assertEqual(t.landed_state, 2)
        self.assertTrue(t.is_airborne)


class HeartbeatTest(unittest.TestCase):
    def test_armed_bit_and_mode_are_decoded_together(self):
        t = TelemetrySnapshot()
        t.update_heartbeat(Msg(base_mode=128, custom_mode=custom_mode(6)))
        self.assertTrue(t.armed)
        self.assertTrue(t.connected)
        self.assertEqual(t.flight_mode, "OFFBOARD")

    def test_disarmed_when_safety_bit_clear(self):
        t = TelemetrySnapshot()
        t.update_heartbeat(Msg(base_mode=1, custom_mode=custom_mode(1)))
        self.assertFalse(t.armed)
        self.assertEqual(t.flight_mode, "MANUAL")

    def test_arm_transition_stamps_and_disarm_resets_flight_time(self):
        t = TelemetrySnapshot()
        t.update_heartbeat(Msg(base_mode=128, custom_mode=custom_mode(3)))
        first_stamp = t.arm_timestamp
        self.assertGreater(first_stamp, 0.0)

        # Still armed: the stamp must not be re-taken, or flight time resets
        # to zero on every heartbeat.
        t.update_heartbeat(Msg(base_mode=128, custom_mode=custom_mode(3)))
        self.assertEqual(t.arm_timestamp, first_stamp)

        t.update_heartbeat(Msg(base_mode=0, custom_mode=custom_mode(3)))
        self.assertFalse(t.armed)
        self.assertEqual(t.flight_time_sec, 0.0)


class SensorDecodeTest(unittest.TestCase):
    def test_local_position_derives_altitude_and_ground_speed(self):
        t = TelemetrySnapshot()
        t.update_local_position(Msg(x=1.0, y=2.0, z=-3.0, vx=3.0, vy=4.0, vz=0.5))
        self.assertAlmostEqual(t.altitude, 3.0)        # NED z is negative up
        self.assertAlmostEqual(t.ground_speed, 5.0)    # 3-4-5 triangle
        self.assertFalse(t.position_stale)

    def test_position_staleness_flags_a_dead_feed(self):
        t = TelemetrySnapshot()
        t.check_position_staleness()                   # never received one
        self.assertTrue(t.position_stale)

        t.update_local_position(Msg(x=0, y=0, z=0, vx=0, vy=0, vz=0))
        t.check_position_staleness(max_age_sec=5.0)
        self.assertFalse(t.position_stale)

        t.last_position_time = time.time() - 10.0
        t.check_position_staleness(max_age_sec=2.0)
        self.assertTrue(t.position_stale)

    def test_rc_health_reads_sys_status_bit_not_rssi(self):
        # The project's ELRS receiver reports rssi=255 even on a healthy link,
        # so SYS_STATUS is the only trustworthy source (see telemetry.py).
        bit = MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        t = TelemetrySnapshot()

        t.update_rc_health(Msg(onboard_control_sensors_present=bit,
                               onboard_control_sensors_health=bit))
        self.assertTrue(t.rc_receiver_present)
        self.assertTrue(t.rc_receiver_healthy)

        # Present but unhealthy = transmitter off. This is the state the
        # ELRS "No Pulses" failsafe fix made detectable.
        t.update_rc_health(Msg(onboard_control_sensors_present=bit,
                               onboard_control_sensors_health=0))
        self.assertTrue(t.rc_receiver_present)
        self.assertFalse(t.rc_receiver_healthy)

        # Not present at all cannot be "healthy".
        t.update_rc_health(Msg(onboard_control_sensors_present=0,
                               onboard_control_sensors_health=bit))
        self.assertFalse(t.rc_receiver_present)
        self.assertFalse(t.rc_receiver_healthy)

    def test_vision_staleness_drops_the_ekf2_fusion_flag(self):
        t = TelemetrySnapshot()
        t.check_vision_staleness()
        self.assertFalse(t.d435i_vio_health)
        self.assertFalse(t.ekf2_vision_fused)

        t.update_vision_estimate(Msg())
        t.check_vision_staleness(max_age_sec=3.0)
        self.assertTrue(t.d435i_vio_health)

        t.last_vision_time = time.time() - 10.0
        t.check_vision_staleness(max_age_sec=3.0)
        self.assertFalse(t.d435i_vio_health)
        self.assertFalse(t.ekf2_vision_fused)

    def test_battery_ignores_the_not_reported_sentinels(self):
        t = TelemetrySnapshot()
        t.battery_percent = 77
        t.update_battery(Msg(voltage_battery=16800, current_battery=-1,
                             battery_remaining=-1))
        self.assertAlmostEqual(t.battery_voltage, 16.8)
        self.assertEqual(t.battery_percent, 77)   # -1 must not zero a good value

    def test_command_ack_translates_codes_to_names(self):
        t = TelemetrySnapshot()
        t.update_command_ack(Msg(command=400, result=1))
        self.assertEqual(t.last_ack_cmd_name, "ARM_DISARM")
        self.assertEqual(t.last_ack_result, "TEMPORARILY_REJECTED")

        t.update_command_ack(Msg(command=9999, result=42))
        self.assertEqual(t.last_ack_cmd_name, "CMD_9999")
        self.assertEqual(t.last_ack_result, "RES_42")

    def test_known_code_tables_cover_the_commands_this_gcs_sends(self):
        for cmd in (400, 176, 22, 21, 20, 185, 511):
            self.assertIn(cmd, MAV_CMD_NAMES)
        for res in range(0, 7):
            self.assertIn(res, MAV_RESULT_NAMES)


class CloneTest(unittest.TestCase):
    def test_clone_detaches_the_motor_list(self):
        # The worker thread clones snapshots for the GUI; a shared list would
        # let PWM values mutate mid-paint.
        t = TelemetrySnapshot()
        t.update_servo_output(Msg(servo1_raw=1100, servo2_raw=1200,
                                  servo3_raw=1300, servo4_raw=1400))
        c = t.clone()
        self.assertEqual(c.motor_pwms, [1100, 1200, 1300, 1400])
        c.motor_pwms[0] = 9999
        self.assertEqual(t.motor_pwms[0], 1100)


if __name__ == "__main__":
    unittest.main()
