#!/usr/bin/env python3
"""
Render GCS Tactical SLAM tab with dual-layer overlay to an image artifact.
"""
import sys
import os
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QSize

sys.path.insert(0, "/home/radxa/Flop/scripts/gcs")
app = QApplication.instance() or QApplication(sys.argv)

from drone_gcs import DroneGCSMainWindow

def capture_dual_layer_ui():
    gcs = DroneGCSMainWindow()
    gcs.resize(1280, 800)

    # Switch to Tactical SLAM tab (page 2)
    gcs._on_view_changed(2)

    # Feed synthetic raw map (with obstacle thickness & floor)
    h, w = 150, 200
    raw_grid = np.full((h, w), -1, dtype=np.int8)
    raw_grid[20:130, 20:180] = 0           # free floor
    raw_grid[20:30, 20:180] = 100          # north wall 10-cell thick
    raw_grid[120:130, 20:180] = 100        # south wall 10-cell thick
    raw_grid[20:130, 20:30] = 100          # west wall 10-cell thick
    raw_grid[20:130, 170:180] = 100        # east wall 10-cell thick
    # Interior pillar
    raw_grid[60:80, 80:100] = 100

    gcs.page_slam.on_ros2_map_received(raw_grid, 0.025, -2.5, -2.0, "map_raw")

    # Feed synthetic thin map (luminous cyan 1-pixel skeleton)
    thin_grid = np.full((h, w), -1, dtype=np.int8)
    thin_grid[25, 25:175] = 100
    thin_grid[125, 25:175] = 100
    thin_grid[25:125, 25] = 100
    thin_grid[25:125, 175] = 100
    # Pillar outline
    thin_grid[60, 80:100] = 100
    thin_grid[79, 80:100] = 100
    thin_grid[60:80, 80] = 100
    thin_grid[60:80, 99] = 100

    gcs.page_slam.on_ros2_map_received(thin_grid, 0.025, -2.5, -2.0, "map_thin")

    # Update drone pose to center of room
    gcs.page_slam.update_pose(0.0, 0.0, 45.0)

    # Process events and render
    app.processEvents()
    pix = gcs.grab()
    out_path = "/home/radxa/.gemini/antigravity-ide/brain/62367388-355a-4399-9bde-dba8bef0d635/scratch/dual_layer_slam_preview.png"
    pix.save(out_path)
    print(f"Screenshot saved to {out_path}")

if __name__ == "__main__":
    capture_dual_layer_ui()
