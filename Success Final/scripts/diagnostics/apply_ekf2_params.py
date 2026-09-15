#!/usr/bin/env python3
"""
================================================================================
MODULE: apply_ekf2_params.py
PURPOSE: Safely Sets and Verifies Pixhawk Autopilot Parameters via MAVLink
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Radxa Q6A Companion Computer or Ground Station Laptop
  * Communicates:  Pixhawk 6X Autopilot via mavlink-router (tcp:127.0.0.1:5760 or /dev/pixhawk)
  * Upstream:      Operator CLI arguments (NAME=VALUE key-value pairs)
  * Downstream:    Pixhawk Parameter Storage & Non-Volatile Flash (FRAM)

DATA FLOW & INTERFACES:
  * MAVLink Out:   PARAM_SET (msg ID 23) with IEEE-754 bitcast encoded value
  * MAVLink In:    PARAM_VALUE (msg ID 22) confirmation & verification readback
  * Target:        Autopilot SysID 1, Component ID 1

KEY LOGIC & FAILSAFES:
  * Mandatory Readback Verification: Does not assume PARAM_SET ACK equals success;
    actively dispatches PARAM_REQUEST_READ to confirm parameter is committed to memory.
  * IEEE-754 Bitcast Encoding: Re-encodes integer bit patterns into 32-bit floats
    (e.g., int32 1 encoded as raw float32 bits, NOT float(1.0) which produces 1065353216).
  * Strict Isolation: Only updates parameters explicitly provided on the command line;
    no wildcard bulk-updating to eliminate accidental configuration corruption.

RUN:
  # Enable UAVCAN optical flow and rangefinder:
  python3 apply_ekf2_params.py --port tcp:127.0.0.1:5760 UAVCAN_SUB_FLOW=1 UAVCAN_SUB_RNG=1

  # Bypass bench power checks on test stand:
  python3 apply_ekf2_params.py --port tcp:127.0.0.1:5760 CBRK_SUPPLY_CHK=894281
================================================================================
"""
import argparse
import struct
import sys
import time

from pymavlink import mavutil

sys.stdout.reconfigure(line_buffering=True)


def decode(value, ptype):
    """PX4 sends non-float params as bit-reinterpreted floats, not numeric casts -
    same decode as verify_ekf2_params.py. Getting this wrong once already produced
    completely wrong values (2.10195e-44 read as "the value" instead of int 15)."""
    raw = struct.pack('<f', value)
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return value
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack('<i', raw)[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return struct.unpack('<I', raw)[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
        return struct.unpack('<h', raw[:2])[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
        return struct.unpack('<H', raw[:2])[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
        return struct.unpack('<b', raw[:1])[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
        return struct.unpack('<B', raw[:1])[0]
    return value


def encode_for_set(val, ptype):
    """Inverse of decode(): PX4 expects a SET value encoded the same way its own
    PARAM_VALUE replies encode it - an INT32 goes in as the float32 whose raw bits
    equal the integer, not the integer's numeric float equivalent. Sending
    float(1) = 1.0 directly for an intended int32 value of 1 is wrong: PX4 will
    store/report back whatever int32 shares 1.0's bit pattern (1065353216) instead
    of 1 - confirmed for real on 2026-09-05, which is exactly why this function
    exists instead of a bare float(val) call."""
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(val)
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack('<f', struct.pack('<i', int(val)))[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return struct.unpack('<f', struct.pack('<I', int(val)))[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
        return struct.unpack('<f', struct.pack('<h', int(val)) + b'\x00\x00')[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
        return struct.unpack('<f', struct.pack('<H', int(val)) + b'\x00\x00')[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
        return struct.unpack('<f', struct.pack('<b', int(val)) + b'\x00\x00\x00')[0]
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
        return struct.unpack('<f', struct.pack('<B', int(val)) + b'\x00\x00\x00')[0]
    return float(val)


def read_param(m, tsys, tcomp, name, timeout=2.0, retries=3):
    for _ in range(retries):
        m.mav.param_request_read_send(tsys, tcomp, name.encode('ascii'), -1)
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout)
        if msg is not None and msg.param_id.rstrip('\x00') == name:
            return decode(msg.param_value, msg.param_type), msg.param_type
    return None, None


def main():
    ap = argparse.ArgumentParser(
        description="Push and verify specific PX4 int32 parameters. "
                    "Give NAME=VALUE pairs, e.g. UAVCAN_SUB_FLOW=1")
    ap.add_argument("--port", default="tcp:127.0.0.1:5760")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("params", nargs="+", help="NAME=VALUE pairs")
    args = ap.parse_args()

    to_set = []
    for p in args.params:
        if "=" not in p:
            print(f"skipping malformed arg {p!r} - expected NAME=VALUE")
            continue
        name, val = p.split("=", 1)
        to_set.append((name.strip(), int(val.strip())))

    if not to_set:
        print("nothing to do")
        return 1

    is_serial = not args.port.startswith(('tcp:', 'udp:', 'udpin:', 'udpout:'))
    kwargs = {'baud': args.baud} if is_serial else {}
    print(f"connecting to {args.port} ...")
    m = mavutil.mavlink_connection(args.port, **kwargs)

    # Plain wait_heartbeat() + m.target_system races against every OTHER system
    # mirrored onto this link by mavlink-router (e.g. sysid=0/uninitialized, or a GCS's
    # own mirrored heartbeat) and can silently latch onto the wrong one instead of the
    # real FMU (sysid=1). Same bug already found and fixed in px4_control.py and
    # verify_ekf2_params.py - filter for an actual autopilot heartbeat and read
    # sysid/compid off the message itself.
    deadline = time.time() + 10.0
    hb = None
    while time.time() < deadline:
        remaining = deadline - time.time()
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
    print(f"heartbeat OK: sys={tsys} comp={tcomp}\n")

    ok = True
    for name, val in to_set:
        before, ptype = read_param(m, tsys, tcomp, name)
        if before is None:
            print(f"  {name}: could not read current value - skipping, not setting blind")
            ok = False
            continue
        print(f"  {name}: current={before} (type={ptype}), setting to {val} ...")
        wire_value = encode_for_set(val, ptype)
        m.mav.param_set_send(tsys, tcomp, name.encode('ascii'), wire_value, ptype)
        time.sleep(0.3)
        after, _ = read_param(m, tsys, tcomp, name)
        if after == val:
            print(f"    confirmed: {name} = {after}")
        else:
            print(f"    MISMATCH: read back {after!r}, expected {val} - do not trust this changed")
            ok = False

    m.close()
    print("\nCross-check these in QGroundControl's own parameter viewer too before flying.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
