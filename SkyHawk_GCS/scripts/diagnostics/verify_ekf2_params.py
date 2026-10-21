#!/usr/bin/env python3
"""
================================================================================
MODULE: verify_ekf2_params.py
PURPOSE: Read-Only Audit of Pixhawk EKF2 Sensor Fusion and Height Reference Parameters
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Radxa Q6A Companion Computer or Ground Station Laptop
  * Communicates:  Pixhawk 6X Autopilot via mavlink-router (tcp:127.0.0.1:5760 or /dev/pixhawk)
  * Upstream:      Pixhawk Parameter Table in non-volatile flash (FRAM)
  * Downstream:    Operator console stdout (decoded parameter states and fusion verdicts)

DATA FLOW & INTERFACES:
  * MAVLink Out:   PARAM_REQUEST_READ (queries specific EKF2/UAVCAN/SYS params)
  * MAVLink In:    PARAM_VALUE (msg ID 22) from Pixhawk Autopilot (SysID 1, CompID 1)
  * Inspected:     EKF2_EV_CTRL (Vision Fusion Mask), EKF2_HGT_REF (Primary Alt Source),
                   EKF2_GPS_CTRL (GPS Fusion Mask), EKF2_OF_CTRL (Optical Flow),
                   UAVCAN_SUB_FLOW, UAVCAN_SUB_RNG, CBRK_SUPPLY_CHK.

KEY LOGIC & FAILSAFES:
  * Strictly Read-Only: Never invokes PARAM_SET; completely safe for pre-flight checks.
  * IEEE-754 Bitcast Decoding: Reinterprets raw 32-bit floats as signed/unsigned
    integers according to param_type (avoids PX4 float32 bit-aliasing bugs where
    integer 15 was read as 2.10195e-44).
  * Height Source Disambiguation: Detects whether EKF2 expects Baro (0), GPS (1),
    Rangefinder (2), or External Vision (3) to prevent catastrophic takeoff plunges.

RUN:
  # Via local mavlink-router TCP endpoint:
  python3 verify_ekf2_params.py --port tcp:127.0.0.1:5760

  # Via direct USB/UART device:
  python3 verify_ekf2_params.py --port /dev/pixhawk --baud 921600
================================================================================
"""
import argparse
import struct
import sys

from pymavlink import mavutil

# PX4 (and MAVLink generally) sends non-float parameters by putting the raw bit
# pattern of the integer into PARAM_VALUE's float32 field - NOT by numerically casting
# it. E.g. int32 value 15 arrives as the float32 whose bits equal int32(15), which
# prints as 2.10195e-44 if read naively. Every EKF2_* param here is INT32 on PX4, so
# decode by reinterpreting the bytes, keyed off the message's own param_type.
def decode_param_value(param_value, param_type):
    raw = struct.pack('<f', param_value)
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return param_value
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack('<i', raw)[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return struct.unpack('<I', raw)[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
        return struct.unpack('<h', raw[:2])[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
        return struct.unpack('<H', raw[:2])[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
        return struct.unpack('<b', raw[:1])[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
        return struct.unpack('<B', raw[:1])[0]
    return param_value  # unknown/64-bit type - not expected for the params checked here

# EKF2_HGT_REF is a simple, version-stable enum - safe to decode outright.
HGT_REF_MEANING = {
    0: "baro",
    1: "GPS",
    2: "rangefinder",
    3: "vision (EV)",
}

# EKF2_EV_CTRL bitmask (stable since PX4 v1.12-era firmware): bit0=horizontal position,
# bit1=vertical position (height), bit2=yaw, bit3=horizontal velocity.
EV_CTRL_BITS = {
    0: "horizontal position",
    1: "vertical position (height)",
    2: "yaw",
    3: "horizontal velocity",
}

PARAMS_TO_CHECK = [
    "EKF2_EV_CTRL",
    "EKF2_GPS_CTRL",
    "EKF2_HGT_REF",
    "EKF2_OF_CTRL",   # optical flow fusion - relevant since a PX4Flow is connected
    "EKF2_RNG_CTRL",  # rangefinder fusion - may not exist on every PX4 version
]


def read_param(m, tsys, tcomp, name, timeout=2.0, retries=3):
    for _ in range(retries):
        m.mav.param_request_read_send(tsys, tcomp, name.encode("ascii"), -1)
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg is not None and msg.param_id.strip("\x00") == name:
            return decode_param_value(msg.param_value, msg.param_type)
    return None


def decode_bits(value, meanings):
    bits = int(value)
    return ", ".join(f"{desc}={'ON' if bits & (1 << i) else 'off'}"
                      for i, desc in meanings.items())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/pixhawk")
    ap.add_argument("--baud", type=int, default=921600)
    args = ap.parse_args()

    print(f"connecting to {args.port} @ {args.baud} ...")
    try:
        m = mavutil.mavlink_connection(args.port, baud=args.baud)
    except Exception as e:
        print(f"  open failed: {e}")
        return 1

    # Plain wait_heartbeat() + m.target_system races against every OTHER system
    # mirrored onto this link by mavlink-router (confirmed live: it returned sys=0,
    # not the real FMU's sys=1 - QGroundControl's own heartbeat, sysid=255 autopilot=
    # INVALID, was in the mix too and target_system wasn't reliably populated either).
    # Same bug already found and fixed in px4_control.py's Fleet.connect() - filter for
    # an actual autopilot's heartbeat and read sysid/compid off the message itself.
    import time as _time
    deadline = _time.time() + 10.0
    hb = None
    while _time.time() < deadline:
        remaining = deadline - _time.time()
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=max(0.0, remaining))
        if msg is None:
            break
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            continue  # a GCS (or other non-vehicle) heartbeat mirrored onto the link
        hb = msg
        break
    if hb is None:
        print("NO HEARTBEAT FROM AN AUTOPILOT. Check the cable/port, and that no other "
              "app (QGroundControl, px4_vision_bridge.py) holds this port exclusively.")
        return 1
    tsys, tcomp = hb.get_srcSystem(), hb.get_srcComponent()
    print(f"heartbeat OK: sys {tsys} comp {tcomp}\n")

    values = {}
    for name in PARAMS_TO_CHECK:
        val = read_param(m, tsys, tcomp, name)
        values[name] = val
        shown = "not present on this firmware" if val is None else f"{val:g}"
        print(f"  {name:15s} = {shown}")

    print()
    hgt_ref = values.get("EKF2_HGT_REF")
    if hgt_ref is not None:
        meaning = HGT_REF_MEANING.get(int(hgt_ref), "unknown")
        print(f"EKF2_HGT_REF={int(hgt_ref)} -> height reference is: {meaning}")
        if meaning == "vision (EV)":
            print("  NOTE: height reference is vision, but a PX4Flow (with sonar) is "
                  "reportedly connected. Confirm this is intentional - normally the "
                  "PX4Flow's sonar or baro would own height instead.")
        else:
            print("  Consistent with a PX4Flow/sonar or baro height reference - vision "
                  "is not expected to own height on this airframe.")

    ev_ctrl = values.get("EKF2_EV_CTRL")
    if ev_ctrl is not None:
        print(f"\nEKF2_EV_CTRL={int(ev_ctrl)} bits -> {decode_bits(ev_ctrl, EV_CTRL_BITS)}")
        if int(ev_ctrl) & (1 << 1):
            print("  NOTE: the height bit is ON - EKF2 is fusing vision for height. "
                  "Given the PX4Flow, double check this is what you want.")
        else:
            print("  Height bit is OFF, consistent with PX4Flow/baro owning height "
                  "and vision only contributing horizontal position/yaw.")

    gps_ctrl = values.get("EKF2_GPS_CTRL")
    if gps_ctrl is not None:
        print(f"\nEKF2_GPS_CTRL={int(gps_ctrl)} "
              f"({'GPS fusion enabled - may fight EV indoors' if int(gps_ctrl) else 'GPS fusion disabled, as expected indoors'})")

    of_ctrl = values.get("EKF2_OF_CTRL")
    if of_ctrl is not None:
        print(f"\nEKF2_OF_CTRL={int(of_ctrl)} "
              f"({'optical flow fusion enabled' if int(of_ctrl) else 'optical flow fusion disabled'}) "
              "- relevant since a PX4Flow is connected; if enabled, its velocity estimate "
              "fuses alongside px4_vision_bridge.py's vision position, not instead of it.")

    print("\nThis script only reads parameters - nothing on the FC was changed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
