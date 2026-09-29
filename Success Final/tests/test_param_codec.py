"""Tests for core/param_codec.py's PX4 float32 bit-cast decoding.

This is the same decode this project already validated live against real
hardware in scripts/diagnostics/verify_ekf2_params.py / apply_ekf2_params.py
(2026-09-05) - locked down here so a future edit cannot reintroduce the
original bug (an int32 15 printing as 2.10195e-44 because it was numerically
cast instead of bit-reinterpreted).

Needs pymavlink (for the MAV_PARAM_TYPE_* constants) but no PyQt5 - runs in
the GUI job's dependency set, not the numpy-only hermetic one.
"""

import struct
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from pymavlink import mavutil

from core.param_codec import decode_param_value, format_param_value, param_type_name


def _as_float_bits(fmt: str, value) -> float:
    """Pack `value` with struct format `fmt`, then reinterpret those bytes as
    a float32 - exactly what PX4 puts on the wire in PARAM_VALUE."""
    packed = struct.pack(fmt, value)
    padded = packed + b"\x00" * (4 - len(packed))
    return struct.unpack('<f', padded)[0]


class DecodeParamValueTest(unittest.TestCase):
    def test_real32_passes_through_unchanged(self):
        self.assertAlmostEqual(
            decode_param_value(3.5, mavutil.mavlink.MAV_PARAM_TYPE_REAL32), 3.5)

    def test_int32_is_bitcast_not_numerically_cast(self):
        # The exact real-world bug: int32(15) sent naively as float(15.0)
        # would decode to 15 too, so this only proves something if the raw
        # bits are NOT the numeric value 15 - which they are not for a
        # bit-pattern that represents 15 as an int32.
        raw = _as_float_bits('<i', 15)
        self.assertNotEqual(raw, 15.0)
        self.assertEqual(
            decode_param_value(raw, mavutil.mavlink.MAV_PARAM_TYPE_INT32), 15)

    def test_negative_int32(self):
        raw = _as_float_bits('<i', -7)
        self.assertEqual(
            decode_param_value(raw, mavutil.mavlink.MAV_PARAM_TYPE_INT32), -7)

    def test_uint8(self):
        raw = _as_float_bits('<B', 200)
        self.assertEqual(
            decode_param_value(raw, mavutil.mavlink.MAV_PARAM_TYPE_UINT8), 200)

    def test_uint32(self):
        raw = _as_float_bits('<I', 4000000000)
        self.assertEqual(
            decode_param_value(raw, mavutil.mavlink.MAV_PARAM_TYPE_UINT32), 4000000000)

    def test_int16(self):
        raw = _as_float_bits('<h', -1234)
        self.assertEqual(
            decode_param_value(raw, mavutil.mavlink.MAV_PARAM_TYPE_INT16), -1234)


class FormatAndNameTest(unittest.TestCase):
    def test_integer_type_has_no_trailing_decimal(self):
        self.assertEqual(
            format_param_value(15, mavutil.mavlink.MAV_PARAM_TYPE_INT32), "15")

    def test_real_type_keeps_precision(self):
        self.assertEqual(
            format_param_value(0.025, mavutil.mavlink.MAV_PARAM_TYPE_REAL32), "0.025")

    def test_type_name_is_readable(self):
        self.assertEqual(
            param_type_name(mavutil.mavlink.MAV_PARAM_TYPE_REAL32), "REAL32")
        self.assertEqual(
            param_type_name(mavutil.mavlink.MAV_PARAM_TYPE_INT32), "INT32")

    def test_unknown_type_falls_back_to_a_labelled_number(self):
        self.assertEqual(param_type_name(999), "TYPE_999")


if __name__ == "__main__":
    unittest.main()
