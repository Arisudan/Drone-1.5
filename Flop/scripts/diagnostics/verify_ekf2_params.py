#!/usr/bin/env python3
"""Read-only pre-flight check: what does PX4's EKF2 actually think its position/height
source is, right now, on the real Pixhawk 6X?

This is deliberately READ-ONLY - it never calls param_set_send. Adapted from the
push-only pattern in a reference implementation's fc_inject.py (--set-params), which
pushes EKF2_EV_CTRL/EKF2_GPS_CTRL/EKF2_HGT_REF blind. Pushing the wrong value on real
flight hardware is a real safety issue, so that half is a separate, explicitly-gated
script (apply_ekf2_params.py) - this one only reads and reports.

Why this matters for THIS airframe specifically: a PX4Flow module (optical flow +
integrated sonar) is connected, so PX4 most likely already has a non-vision height
reference (baro or the PX4Flow sonar), not vision. That's the opposite of a camera-only
airframe with no rangefinder, where EV would be expected to own height. This script
prints the raw values and a plain-English read of what they imply so that assumption
gets checked against the real FC instead of assumed.

Run (bench only, no props needed):
    python3 verify_ekf2_params.py
    python3 verify_ekf2_params.py --port /dev/pixhawk --baud 921600   # defaults shown
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
