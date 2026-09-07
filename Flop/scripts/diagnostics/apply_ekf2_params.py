#!/usr/bin/env python3
"""Push specific PX4 parameters over MAVLink, then read them back to verify the
change actually took effect - the SET acknowledgment alone is not proof, only a
re-read is. This is the counterpart to verify_ekf2_params.py, which is read-only;
this script is the explicit, opt-in "apply" half, gated on a confirmed mismatch.

Only ever touches the exact NAME=VALUE pairs given on the command line - no
defaults, no "set everything" mode, so a mistyped invocation can't silently
change something unintended. Integer-only (PX4's EKF2/UAVCAN config params here
are all INT32).

Run (through mavlink-router, not the raw serial device - it's already in use):
    python3 apply_ekf2_params.py UAVCAN_SUB_FLOW=1 UAVCAN_SUB_RNG=1
    python3 apply_ekf2_params.py --port tcp:127.0.0.1:5760 UAVCAN_SUB_FLOW=1

Cross-check the result in QGroundControl's own parameter viewer too before
trusting it for flight.
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
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("NO HEARTBEAT")
        return 1
    tsys, tcomp = m.target_system, (m.target_component or 1)
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
