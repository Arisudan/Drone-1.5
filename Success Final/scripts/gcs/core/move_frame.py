"""
================================================================================
MODULE: move_frame.py
PURPOSE: Turn a body-relative move (forward / right / down) into North / East / Down (no Qt)
================================================================================

WHY THIS EXISTS:
  `move 1 0 0` used to be a displacement in the world's North/East/Down frame, so "1" meant
  "1 m North" whichever way the drone was facing. A drone facing South-East moved BACKWARD.
  The default is now the drone's own frame: x = forward, y = right, z = down. The heading
  is read from the vehicle (ATTITUDE yaw, degrees clockwise from North).

  A fixed `yaw_offset_deg` and the two `invert_*` switches are there to correct a mounting or
  compass rotation found on the real vehicle; all three default to "no correction".
================================================================================
"""

from __future__ import annotations

import math
from typing import Tuple


def body_to_ned(forward: float, right: float, heading_deg: float,
                yaw_offset_deg: float = 0.0,
                invert_forward: bool = False, invert_right: bool = False) -> Tuple[float, float]:
    """(north, east) metres for a move of `forward` / `right` metres made while facing `heading_deg`."""
    if invert_forward:
        forward = -forward
    if invert_right:
        right = -right
    h = math.radians(heading_deg + yaw_offset_deg)
    north = forward * math.cos(h) - right * math.sin(h)
    east = forward * math.sin(h) + right * math.cos(h)
    return north, east
