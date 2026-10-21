#!/usr/bin/env python3
"""
Unit test for dual-channel /map (raw) and /map_thin (skeleton) streaming & GCS canvas overlay.
"""
import sys
import os
import time
import socket
import struct
import json
import zlib
import numpy as np

# Add gcs path
sys.path.insert(0, "/home/radxa/Flop/scripts/gcs")

# Mock PyQt5 GUI environment for headless testing
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QImage, QColor
from PyQt5.QtCore import QPointF, QRectF

app = QApplication.instance() or QApplication(sys.argv)

from ui.slam_map_widget import SLAMMapCanvas, SLAMMapWidget
from protocol.ros2_map_listener import MAGIC

def test_dual_map_overlay():
    print("Testing SLAMMapCanvas dual-layer overlay...")
    canvas = SLAMMapCanvas()
    assert canvas.display_mode == "both", f"Expected default display_mode 'both', got {canvas.display_mode}"

    # 1. Create a synthetic Raw Map (200x200, 2.5cm resolution)
    # Free space = 0, Obstacle = 100, Unknown = -1
    h, w = 120, 160
    raw_grid = np.full((h, w), -1, dtype=np.int8)
    # Free space in room center
    raw_grid[20:100, 30:130] = 0
    # Outer obstacle walls (5-cell thickness)
    raw_grid[20:25, 30:130] = 100
    raw_grid[95:100, 30:130] = 100
    raw_grid[20:100, 30:35] = 100
    raw_grid[20:100, 125:130] = 100

    canvas.set_occupancy_grid(raw_grid, 0.025, -2.0, -1.5, "map_raw")
    assert canvas.map_raw_qimage is not None, "map_raw_qimage should be created"
    assert canvas.grid_raw is not None, "grid_raw should be stored"

    # Verify raw map colors
    # Free space pixel should have alpha > 0 and slate color
    img_raw = canvas.map_raw_qimage
    assert img_raw.width() == w and img_raw.height() == h

    # 2. Create a synthetic Thin Map (skeleton wall, 1-cell thickness)
    thin_grid = np.full((h, w), -1, dtype=np.int8)
    thin_grid[22, 32:128] = 100
    thin_grid[97, 32:128] = 100
    thin_grid[22:98, 32] = 100
    thin_grid[22:98, 127] = 100

    canvas.set_occupancy_grid(thin_grid, 0.025, -2.0, -1.5, "map_thin")
    assert canvas.map_thin_qimage is not None, "map_thin_qimage should be created"
    assert canvas.grid_thin is not None, "grid_thin should be stored"
    assert canvas.grid_raw is not None, "grid_raw should NOT be overwritten by grid_thin!"

    # Verify thin map transparency: non-wall pixels MUST be alpha=0
    img_thin = canvas.map_thin_qimage
    free_px = img_thin.pixelColor(50, 50)
    assert free_px.alpha() == 0, f"Thin map non-wall pixels must be transparent (alpha=0), got {free_px.alpha()}"
    
    # Wall pixel MUST be bright luminous cyan [0, 225, 255, 255]
    # Note: image is flipped vertically for North-up
    # thin_grid[22, 32] -> row 22 is near top in numpy, flipped row is h - 1 - 22
    wall_px = img_thin.pixelColor(32, h - 1 - 22)
    assert wall_px.alpha() == 255, f"Thin map wall pixel must be alpha=255, got {wall_px.alpha()}"
    assert wall_px.red() == 0 and wall_px.green() == 225 and wall_px.blue() == 255, f"Unexpected wall color: {wall_px.getRgb()}"

    # 3. Test Layer Mode switching
    canvas.set_display_mode("thin")
    assert canvas.display_mode == "thin"
    canvas.set_display_mode("raw")
    assert canvas.display_mode == "raw"
    canvas.set_display_mode("both")
    assert canvas.display_mode == "both"

    print("✔ SLAMMapCanvas dual-layer overlay verified successfully!")

def test_widget_layer_controls():
    print("Testing SLAMMapWidget layer controls & status pill...")
    widget = SLAMMapWidget()
    assert widget.btn_layer_overlay is not None
    assert widget.btn_layer_thin is not None
    assert widget.btn_layer_raw is not None

    # Feed raw map
    grid_raw = np.zeros((50, 50), dtype=np.int8)
    widget.on_ros2_map_received(grid_raw, 0.025, 0.0, 0.0, "/map")
    assert widget._has_raw_map is True
    assert "LIVE /map" in widget.pill_status.text()

    # Feed thin map
    grid_thin = np.zeros((50, 50), dtype=np.int8)
    widget.on_ros2_map_received(grid_thin, 0.025, 0.0, 0.0, "/map_thin")
    assert widget._has_thin_map is True
    assert "OVERLAY" in widget.pill_status.text(), f"Expected 'OVERLAY', got '{widget.pill_status.text()}'"

    # Test toggling layer modes
    widget.set_layer_mode("thin")
    assert widget.canvas.display_mode == "thin"
    widget.set_layer_mode("raw")
    assert widget.canvas.display_mode == "raw"
    widget.set_layer_mode("both")
    assert widget.canvas.display_mode == "both"

    print("✔ SLAMMapWidget layer controls verified successfully!")

if __name__ == "__main__":
    test_dual_map_overlay()
    test_widget_layer_controls()
    print("\nALL DUAL-MAP TESTS PASSED!")
