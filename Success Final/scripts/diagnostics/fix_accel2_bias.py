#!/usr/bin/env python3
"""
================================================================================
TOOL: fix_accel2_bias.py
PURPOSE: Retire the Faulty IMU2 Accelerometer and Persist It Across Power Cycles
================================================================================

THE FAULT THIS FIXES:
  PX4 refuses to arm with "Preflight Fail: High Accelerometer Bias", repeating
  every ~2 s, followed by "Arming denied: Resolve system health failures
  first". No amount of re-sending the arm command helps - PX4's commander gates
  arming on preflight health, so the command is received and refused, not lost.

THE CAUSE (see gotalldone.md dev log #26):
  This airframe's IMU2 accelerometer disagrees with the primary by ~1.1 m/s2 on
  the horizontal axes - well past PX4's own COM_ARM_IMU_ACC=0.7 tolerance, and
  confirmed independently by PX4's own calibration routine naming "Accel 2
  inconsistent". The fix is CAL_ACC2_PRIO=0, which removes that sensor from the
  estimator's selection entirely rather than widening the tolerance and hiding
  a genuine physical disagreement between two sensors.

WHY IT CAME BACK:
  Sensor priority was set once with a live PARAM_SET and never written to
  flash, so the next power cycle restored CAL_ACC2_PRIO=50 and put the faulty
  unit back in service. That is exactly the failure mode this script exists to
  close: it writes the parameter, **saves it to flash**, reboots, and then
  re-reads it to prove the value survived.

WHY MOVING THE VEHICLE MAKES IT WORSE:
  EKF2's accelerometer-bias states absorb the difference between measured
  specific force and what the filter expects from its own position solution.
  This airframe's position source is vision, not GPS, and vision lags. Carrying
  the drone by hand produces real accelerations the VIO estimate trails behind,
  and the filter charges the discrepancy to accel bias. A genuinely mis-
  calibrated IMU in the blend makes that climb from a standing start.

SAFETY:
  Refuses to run while armed. The reboot is only ever sent disarmed, which is
  also PX4's own precondition for accepting it.

USAGE:
  ./fix_accel2_bias.py                          # UART via mavlink-router
  ./fix_accel2_bias.py --port udpout:172.16.101.84:14550
  ./fix_accel2_bias.py --check                  # read-only: report, change nothing
  ./fix_accel2_bias.py --no-reboot              # set + save, reboot yourself later
================================================================================
"""

from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil

PARAM = "CAL_ACC2_PRIO"
DISABLED = 0
DEFAULT_PORT = "tcp:127.0.0.1:5760"


def find_autopilot(m, timeout: float = 10.0):
    """Latch onto a real autopilot heartbeat, not the first system seen.

    mavlink-router mirrors every system on the link, including a GCS's own
    heartbeat and uninitialised sysid=0 traffic. A bare wait_heartbeat() has
    latched onto the wrong one before - the same bug already fixed in
    px4_control.py, verify_ekf2_params.py and apply_ekf2_params.py.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="HEARTBEAT", blocking=True,
                           timeout=max(0.0, deadline - time.time()))
        if msg is None:
            break
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            continue
        return msg.get_srcSystem(), msg.get_srcComponent(), msg
    return None, None, None


def read_param(m, tsys, tcomp, name, timeout=2.0, retries=4):
    for _ in range(retries):
        m.mav.param_request_read_send(tsys, tcomp, name.encode("ascii"), -1)
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            return int(msg.param_value)
    return None


def set_param_int(m, tsys, tcomp, name, value, timeout=2.0, retries=4):
    """Write, then read back. A PARAM_SET with no readback proves nothing."""
    for _ in range(retries):
        m.mav.param_set_send(tsys, tcomp, name.encode("ascii"), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            return int(msg.param_value)
    return None


def save_to_flash(m, tsys, tcomp) -> bool:
    """MAV_CMD_PREFLIGHT_STORAGE param1=1 -> write parameters to storage.

    This is the step whose absence caused the original fix to evaporate on the
    next power cycle.
    """
    m.mav.command_long_send(
        tsys, tcomp, mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0,
        1, 0, 0, 0, 0, 0, 0)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5.0)
    if ack is None:
        print("  ! no COMMAND_ACK for the storage write")
        return False
    ok = ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    print(f"  storage write ack: {'ACCEPTED' if ok else ack.result}")
    return ok


def reboot(m, tsys, tcomp) -> bool:
    m.mav.command_long_send(
        tsys, tcomp, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1, 0, 0, 0, 0, 0, 0)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5.0)
    # PX4 often reboots before the ack reaches us; absence is not failure here.
    if ack is not None:
        print(f"  reboot ack: {ack.result}")
    return True


def describe_health(m, timeout: float = 6.0) -> None:
    msg = m.recv_match(type="SYS_STATUS", blocking=True, timeout=timeout)
    if msg is None:
        print("  (no SYS_STATUS yet - give it a few seconds and re-check)")
        return
    unhealthy = msg.onboard_control_sensors_health ^ msg.onboard_control_sensors_enabled
    unhealthy &= msg.onboard_control_sensors_enabled
    if unhealthy:
        print(f"  SYS_STATUS unhealthy mask: 0x{unhealthy:08X}  (not yet clean)")
    else:
        print("  SYS_STATUS: all enabled sensors healthy")


def connect(port: str, baud: int):
    is_serial = not port.startswith(("tcp:", "udp:", "udpin:", "udpout:"))
    kwargs = {"baud": baud} if is_serial else {}
    return mavutil.mavlink_connection(port, **kwargs)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Disable the faulty IMU2 accelerometer and persist it to flash.")
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help=f"MAVLink endpoint (default {DEFAULT_PORT})")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--check", action="store_true",
                    help="report the current value and exit without changing anything")
    ap.add_argument("--no-reboot", action="store_true",
                    help="set and save but do not reboot (the change is inert until a reboot)")
    args = ap.parse_args(argv)

    print(f"connecting to {args.port} ...")
    m = connect(args.port, args.baud)
    tsys, tcomp, hb = find_autopilot(m)
    if hb is None:
        print("NO AUTOPILOT HEARTBEAT. Check the link and that mavlink-router is up.")
        return 2
    print(f"autopilot: sys={tsys} comp={tcomp}")

    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    current = read_param(m, tsys, tcomp, PARAM)
    print(f"{PARAM} currently = {current}"
          f"  ({'faulty IMU2 IN SERVICE' if current not in (0, None) else 'disabled'})")
    describe_health(m)

    if args.check:
        return 0
    if armed:
        print("REFUSING: vehicle is ARMED. Disarm before changing sensor priority.")
        return 3
    if current == DISABLED:
        print(f"{PARAM} is already {DISABLED}; saving to flash anyway so it "
              "survives the next power cycle.")

    print(f"setting {PARAM} = {DISABLED} ...")
    got = set_param_int(m, tsys, tcomp, PARAM, DISABLED)
    if got != DISABLED:
        print(f"  ! readback returned {got!r}, expected {DISABLED}. Aborting.")
        return 4
    print(f"  readback confirms {PARAM} = {got}")

    print("writing parameters to flash ...")
    if not save_to_flash(m, tsys, tcomp):
        print("  ! flash write not confirmed - the change will be lost on power cycle")
        return 5

    if args.no_reboot:
        print("\nSet and saved. NOT rebooted: sensor priority is read once at boot, "
              "so the faulty IMU stays in service until you reboot the FC.")
        return 0

    print("rebooting the flight controller (sensor priority is read at boot only) ...")
    reboot(m, tsys, tcomp)
    m.close()

    print("waiting for it to come back ...")
    time.sleep(8.0)
    m2 = connect(args.port, args.baud)
    tsys2, tcomp2, hb2 = find_autopilot(m2, timeout=25.0)
    if hb2 is None:
        print("  ! no heartbeat after reboot. Check mavlink-router and /dev/pixhawk.")
        return 6

    after = read_param(m2, tsys2, tcomp2, PARAM)
    print(f"after reboot: {PARAM} = {after}")
    describe_health(m2)

    if after != DISABLED:
        print("  ! the value did NOT survive the reboot - the flash write did not take.")
        return 7

    print("\nDONE. The faulty IMU2 accelerometer is out of the estimator and the "
          "setting is persisted.\nIf 'High Accelerometer Bias' returns, leave the "
          "vehicle still for ~10 s after boot so EKF2 seeds its bias states from a "
          "stationary vehicle, and re-check.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
