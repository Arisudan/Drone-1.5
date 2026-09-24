"""
================================================================================
MODULE: map_render.py
PURPOSE: Occupancy grid -> RGBA image, off the GUI thread
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station - the map listener's worker thread
  * Upstream:      protocol/ros2_map_listener.py, as each grid arrives
  * Downstream:    ui/slam_map_widget.py, which draws the finished QImage

WHY THIS IS NOT IN THE WIDGET ANY MORE:
  Turning a grid into an image is a lookup over every cell. Measured on a 25 x
  25 m room at this project's 2.5 cm resolution - a 1000 x 1000 grid - it costs
  about 12 ms, and it used to run inside SLAMMapCanvas.set_occupancy_grid on the
  GUI thread, five times a second at the streamer's default rate. That is 60 ms
  of every second spent not drawing, not responding, and - because the same
  thread carries the 10 Hz OFFBOARD setpoint pump whose PX4 deadline is 500 ms -
  not flying the aircraft either. The conversion is pure numpy plus a QImage, so
  it belongs on the thread that already receives the data.

QImage IS SAFE OFF THE GUI THREAD. QPixmap IS NOT.
  Qt allows QImage to be created and manipulated on any thread; only QPixmap is
  restricted to the GUI thread. Nothing here may be changed to a QPixmap.

THE IMAGE OWNS ITS PIXELS:
  QImage built from a numpy buffer does not copy - it references the array, and
  PyQt keeps that array alive for as long as the image lives. That is safe, but
  it also means the image and a numpy array share memory across a thread
  boundary. `.copy()` makes the image self-contained, which is what makes the
  hand-off to the GUI thread obviously correct rather than merely working.
================================================================================
"""

from __future__ import annotations

import numpy as np
from PyQt5.QtGui import QImage


# RViz-exact colour palettes.
#
# These are not an approximation of how RViz looks - they are the byte-for-byte
# palettes RViz builds, read out of the installed librviz_default_plugins.so via
# makeMapPalette()/makeCostmapPalette() (see rviz_default_plugins/displays/map/
# palette_builder.hpp). config/rtabmap_drone.rviz asks for "map" on /map and
# "costmap" on /map_thin, so those are the two reproduced here. The point is that
# the tactical 2D canvas and the embedded RViz view render identical data
# identically - an operator switching between the two tabs should not have to
# re-learn what a colour means.
#
# THE UNKNOWN COLOUR IS THE IMPORTANT ONE. RViz paints never-observed cells
# (-1) as an opaque, deliberately off-hue teal-grey (112, 137, 134) rather than
# a lighter shade of the free/occupied ramp. That matters for flight: "clear"
# and "never looked at" must not be separated by brightness alone, because
# brightness is exactly what a grayscale ramp already spends on occupancy.
#
# Integer division below is not sloppiness - RViz uses C integer division and
# rounding differently would put the ramp a level off at most stops.

_UNKNOWN_RGBA = [112, 137, 134, 255]        # RViz: -1, never observed
_ILLEGAL_POSITIVE_RGBA = [0, 255, 0, 255]   # RViz: 101..127, out-of-spec value


def _illegal_negative_rgba(idx: int) -> list:
    """RViz's red->yellow ramp for out-of-spec negative values (128..254).

    The divisor is 126, not 128: RViz spreads the ramp across the 127 entries
    from 128 to 254 inclusive so that 254 lands on pure yellow. Using a power of
    two here looks tidier and is wrong from index 170 upward.
    """
    return [255, (255 * (idx - 128)) // 126, 0, 255]


# "map" scheme -> /map. Free (0) white through occupied (100) black.
RAW_MAP_LUT = np.zeros((256, 4), dtype=np.uint8)
for _v in range(-128, 128):
    _idx = _v & 0xFF
    if 0 <= _v <= 100:
        _s = 255 - (255 * _v) // 100
        RAW_MAP_LUT[_idx] = [_s, _s, _s, 255]
    elif 101 <= _v <= 127:
        RAW_MAP_LUT[_idx] = _ILLEGAL_POSITIVE_RGBA
    elif _v == -1:
        RAW_MAP_LUT[_idx] = _UNKNOWN_RGBA
    else:
        RAW_MAP_LUT[_idx] = _illegal_negative_rgba(_idx)

# "costmap" scheme -> /map_thin. Free is fully transparent so the /map layer
# below shows through; cost ramps blue->red; 99 and 100 are the two values a
# planner actually acts on, so RViz gives them their own unmistakable colours
# (cyan = inscribed, magenta = lethal) instead of continuing the ramp.
THIN_MAP_LUT = np.zeros((256, 4), dtype=np.uint8)
for _v in range(-128, 128):
    _idx = _v & 0xFF
    if _v == 0:
        THIN_MAP_LUT[_idx] = [0, 0, 0, 0]
    elif 1 <= _v <= 98:
        _r = (255 * _v) // 100
        THIN_MAP_LUT[_idx] = [_r, 0, 255 - _r, 255]
    elif _v == 99:
        THIN_MAP_LUT[_idx] = [0, 255, 255, 255]     # inscribed inflated obstacle
    elif _v == 100:
        THIN_MAP_LUT[_idx] = [255, 0, 255, 255]     # lethal obstacle
    elif 101 <= _v <= 127:
        THIN_MAP_LUT[_idx] = _ILLEGAL_POSITIVE_RGBA
    elif _v == -1:
        THIN_MAP_LUT[_idx] = _UNKNOWN_RGBA
    else:
        THIN_MAP_LUT[_idx] = _illegal_negative_rgba(_idx)

# Global Options from config/rtabmap_drone.rviz, so both views share one look.
RVIZ_BACKGROUND = (48, 48, 48)
RVIZ_GRID_RGB = (160, 160, 160)
RVIZ_GRID_ALPHA = 178              # 0.7 * 255, the Grid display's Alpha
RVIZ_MAP_ALPHA = 0.7               # /map display Alpha
RVIZ_MAP_THIN_ALPHA = 1.0          # /map_thin display Alpha
RVIZ_GRID_CELL_SIZE_M = 1.0        # Grid "Cell Size"
RVIZ_GRID_PLANE_CELL_COUNT = 10    # Grid "Plane Cell Count" - a finite 10x10 m
                                   # plane centred on the origin, exactly as
                                   # RViz draws it. Raise this if you want grid
                                   # under a larger flying area.


def build_map_image(grid: np.ndarray, thin: bool) -> QImage:
    """Render an occupancy grid to an RGBA QImage using the RViz palette.

    `thin` selects the costmap scheme used for /map_thin (transparent free
    space, so the raw layer shows through) rather than the map scheme used for
    /map. Returns an image that owns its own pixels and can be handed to
    another thread.

    Rows are flipped because grid row 0 is the southern edge in ROS and the top
    of the image in Qt.
    """
    if grid is None or grid.ndim != 2 or grid.size == 0:
        return QImage()
    h, w = grid.shape
    lut = THIN_MAP_LUT if thin else RAW_MAP_LUT
    rgba = np.ascontiguousarray(lut[grid.view(np.uint8)][::-1])
    return QImage(rgba.data, w, h, 4 * w, QImage.Format_RGBA8888).copy()
