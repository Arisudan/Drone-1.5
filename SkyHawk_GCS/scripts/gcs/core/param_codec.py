"""
================================================================================
MODULE: param_codec.py
PURPOSE: MAVLink PARAM_VALUE decoding (PX4's float32 bit-cast convention)
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS UI Layer) and CLI diagnostics
  * Upstream:      PARAM_VALUE messages (protocol/mavlink_worker.py)
  * Downstream:    ui/params_tab.py's live parameter table

WHY THIS EXISTS:
  PX4 (and MAVLink generally) sends non-float parameters by putting the raw
  bit pattern of the integer into PARAM_VALUE's float32 field - not by
  numerically casting it. E.g. int32 value 15 arrives as the float32 whose
  bits equal int32(15), which prints as 2.10195e-44 if read naively. This
  exact bug, and this exact fix, were already found and verified live against
  real hardware in scripts/diagnostics/verify_ekf2_params.py and
  apply_ekf2_params.py (2026-09-05) - factored out here so a third
  implementation of the same decode never gets written by accident.
================================================================================
"""

from __future__ import annotations

import struct

from pymavlink import mavutil

#: Human-readable name for each MAV_PARAM_TYPE value, for a UI column - not
#: meant to be exhaustive of every MAVLink type, only the ones PX4 actually
#: uses on the wire for PARAM_VALUE.
PARAM_TYPE_NAMES = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8: "UINT8",
    mavutil.mavlink.MAV_PARAM_TYPE_INT8: "INT8",
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16: "UINT16",
    mavutil.mavlink.MAV_PARAM_TYPE_INT16: "INT16",
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32: "UINT32",
    mavutil.mavlink.MAV_PARAM_TYPE_INT32: "INT32",
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32: "REAL32",
}


def decode_param_value(param_value: float, param_type: int):
    """Reinterpret a PARAM_VALUE's raw float32 as the type PX4 actually meant.

    Returns a Python int for every integer param_type, or the float itself
    for REAL32 (and for any 64-bit/unrecognised type, which PX4 does not use
    here - returned as-is rather than guessed at).
    """
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return param_value
    raw = struct.pack('<f', param_value)
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
    return param_value  # unknown/64-bit type - not expected from PX4


def format_param_value(value, param_type: int) -> str:
    """Display text for a decoded value: no trailing .0 on an integer type,
    but real precision kept for REAL32."""
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return f"{float(value):g}"
    return str(int(value))


def param_type_name(param_type: int) -> str:
    return PARAM_TYPE_NAMES.get(param_type, f"TYPE_{param_type}")
