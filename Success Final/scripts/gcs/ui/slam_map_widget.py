"""
================================================================================
MODULE: slam_map_widget.py
PURPOSE: Interactive 2D SLAM Occupancy Grid Visualizer & A* Obstacle-Aware Path Planner
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Tactical SLAM View)
  * Communicates:  drone_gcs.py, AStarPathPlanner, and MAVLinkWorker
  * Upstream:      ROS2MapListener (/map_thin & /map), MAVLink OBSTACLE_DISTANCE,
                   and live LOCAL_POSITION_NED drone coordinates
  * Downstream:    GCS Pilot UI & MAVLink Offboard Trajectory Dispatcher

DATA FLOW & INTERFACES:
  * Map Inputs:    NumPy int8 2D arrays, resolution (m/cell), origin offsets (x, y).
  * State Inputs:  Drone position (x, y), yaw heading, breadcrumb history,
                   OBSTACLE_DISTANCE 360-degree rangefinder sectors.
  * Waypoint Out:  Staged goal coordinates (gx, gy), planned vector waypoints,
                   total trajectory length, and travel time estimates.
  * Qt Signals:    goal_staged(gx, gy, waypoints, dist, est_time), goal_cleared(),
                   auto_follow_changed(bool), rotation_changed(float).

KEY LOGIC & FAILSAFES:
  * Dual-Layer Crisp Rendering: Blends dense `/map` base with razor-sharp `/map_thin`
    skeleton walls; prevents blurriness and pixelation under high zoom.
  * Interactive Click-to-Plan: Click anywhere on free space to calculate optimal
    collision-free route around walls using AStarPathPlanner.
  * Virtual Lidar Safety Ring: Projects 72-sector OBSTACLE_DISTANCE proximity ring
    around vehicle, coloring proximity red (<0.6m) and yellow (<1.2m).
  * Auto-Follow Toggle (Default: OFF): Keeps view static by default so operators
    can inspect distant map sections, with optional drone-centric auto-follow lock.
  * Coordinate Transformation: Accurately maps ROS ENU / FLU grid frames to
    PX4 Local NED ground display coordinates with 90-degree cardinal rotation support.

USAGE:
  map_widget = SLAMMapWidget(path_planner=planner)
  map_widget.update_map(grid_array, resolution=0.05, origin_x=-10.0, origin_y=-10.0)
  map_widget.update_drone_pose(x=1.5, y=2.0, heading_deg=45.0)
================================================================================
"""

from __future__ import annotations
import math
from collections import deque
from typing import Optional, List, Tuple, Dict, Any
import numpy as np

from PyQt5.QtCore import Qt, QRectF, QPointF, pyqtSignal
from PyQt5.QtGui import (
    QPainter, QColor, QFont, QPen, QBrush, QPolygonF, QImage, QWheelEvent, QMouseEvent
)
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QFrame, QSizePolicy, QStackedWidget, QComboBox,
    QMessageBox
)

from core.path_planner import AStarPathPlanner


# Item 3: Precomputed 256-entry RGBA Lookup Tables for single-pass vectorized C-speed rendering
#
# Continuous grayscale gradient (0=white/free -> 100=black/occupied), matching RViz's
# own "map" color scheme for the same /map topic (config/rtabmap_drone.rviz). RTAB-Map's
# occupancy values are a continuous Bayesian probability, not a handful of discrete
# states, so a 4-bucket scheme (the previous white/amber/charcoal/black bands) painted
# every partially-confident cell - wall edges, under-observed areas - as a blotchy
# amber/charcoal patch instead of the smooth gradient RViz shows for the identical data.
RAW_MAP_LUT = np.zeros((256, 4), dtype=np.uint8)
for _v in range(-128, 128):
    _idx = _v & 0xFF
    if _v < 0:
        RAW_MAP_LUT[_idx] = [205, 205, 205, 160]  # Unknown space: neutral grey, translucent
    else:
        shade = int(round(255 * (1.0 - min(100, _v) / 100.0)))
        RAW_MAP_LUT[_idx] = [shade, shade, shade, 255]

THIN_MAP_LUT = np.zeros((256, 4), dtype=np.uint8)
for _v in range(-128, 128):
    _idx = _v & 0xFF
    if _v >= 50:
        THIN_MAP_LUT[_idx] = [220, 38, 38, 235]   # Skeleton walls: Crimson Red (softened slightly
                                                   # now that the raw layer below is a clean gradient
                                                   # rather than a competing amber/charcoal blotch)
    else:
        THIN_MAP_LUT[_idx] = [0, 0, 0, 0]         # Transparent


class SLAMMapCanvas(QWidget):
    """
    High-performance QPainter canvas for 2D SLAM occupancy grid visualization,
    breadcrumb trails, Goal Pose crosshairs, and A* collision-free paths.
    """

    goal_staged = pyqtSignal(float, float, list, float, float)  # (gx, gy, waypoints, dist, est_time)
    goal_cleared = pyqtSignal()
    auto_follow_changed = pyqtSignal(bool)
    rotation_changed = pyqtSignal(float)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(450, 320)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)

        # View transform: scale (pixels per meter) and origin pan offsets (pixels)
        self.scale: float = 36.0  # Wide overview scale: 36 px = 1 meter
        self.pan_x: float = 0.0
        self.pan_y: float = 0.0

        # Planar Rotation Angle (0, 90, 180, 270 degrees)
        self.rotation_deg: float = 0.0

        # Auto-Follow Drone-Centric Tracking Mode (Default: OFF)
        self.auto_follow: bool = False

        # Mouse interaction state
        self._dragging_pan: bool = False
        self._last_mouse_pos: Optional[QPointF] = None

        # Drone State (NED: x=North, y=East) - the values actually drawn, after smoothing
        self.drone_x: float = 0.0
        self.drone_y: float = 0.0
        self.heading: float = 0.0
        self._pose_initialized: bool = False

        # Breadcrumb trail (up to 800 positions). Hidden and not recorded until
        # the first Goal Pose is set - during ordinary manual flight the trail
        # isn't meaningful and just adds visual clutter. Once shown, it stays
        # visible (as a record of that goal run) even after arrival/abort, and
        # only resets when the *next* goal is set - see set_goal().
        self.trail: deque = deque(maxlen=800)
        self.trail_visible: bool = False

        # 2D Occupancy Grid Data - Dual Buffers for RViz-style Composite Overlay
        # 1. Raw Occupancy Grid (/map: free space + obstacle mass)
        self.grid_raw: Optional[np.ndarray] = None
        self.map_raw_res: float = 0.025
        self.map_raw_ox: float = 0.0
        self.map_raw_oy: float = 0.0
        self.map_raw_qimage: Optional[QImage] = None

        # 2. Thin Occupancy Grid (/map_thin: 1-pixel skeletonized walls)
        self.grid_thin: Optional[np.ndarray] = None
        self.map_thin_res: float = 0.025
        self.map_thin_ox: float = 0.0
        self.map_thin_oy: float = 0.0
        self.map_thin_qimage: Optional[QImage] = None

        # Active layer display mode: "both" (default overlay), "thin", or "raw"
        self.display_mode: str = "both"

        # Tracks whether we've auto-scaled to the map extent yet (RViz-style
        # zoom-to-fit on first data arrival, instead of always opening at a
        # fixed 36px/m that can dwarf a small just-started SLAM map under the
        # 1-meter reference grid).
        self._auto_fit_done: bool = False

        # Unified Planner Reference
        self.occupancy_grid: Optional[np.ndarray] = None
        self.map_res: float = 0.025
        self.map_ox: float = 0.0
        self.map_oy: float = 0.0
        self.map_source: str = "Awaiting Map"

        # Path Planner & Goal Pose
        self.planner = AStarPathPlanner(robot_radius_m=0.22)
        self.goal_pose: Optional[Tuple[float, float]] = None
        self.planned_waypoints: List[Tuple[float, float]] = []
        self.planned_distance: float = 0.0
        self.planned_est_time: float = 0.0
        self.path_status_msg: str = "Click on map to set Goal Pose"

    # -------------------------------------------------------------------------
    # Public Data Setters
    # -------------------------------------------------------------------------

    def set_display_mode(self, mode: str):
        """Set visual layer mode: 'both' (composite overlay), 'thin', or 'raw'."""
        self.display_mode = mode
        self.update()

    def turn_left(self):
        """Turn 2D map 90 degrees counter-clockwise."""
        self.rotation_deg = (self.rotation_deg - 90.0) % 360.0
        self.rotation_changed.emit(self.rotation_deg)
        self.update()

    def turn_right(self):
        """Turn 2D map 90 degrees clockwise."""
        self.rotation_deg = (self.rotation_deg + 90.0) % 360.0
        self.rotation_changed.emit(self.rotation_deg)
        self.update()

    def reset_rotation(self):
        """Reset orientation to default North-Up (0 degrees)."""
        self.rotation_deg = 0.0
        self.rotation_changed.emit(self.rotation_deg)
        self.update()

    def set_auto_follow(self, enabled: bool):
        """Toggle drone-centric continuous camera tracking."""
        self.auto_follow = enabled
        self.auto_follow_changed.emit(enabled)
        self.update()

    def set_drone_pose(self, x: float, y: float, heading: float):
        """Update drone pose in metric coordinates.

        Applies light exponential smoothing rather than drawing the incoming pose
        raw: stereo-VIO position/yaw estimates carry real frame-to-frame noise
        (visible live as periodic 'SLAM /odom LOST' rejections in px4_vision_bridge.py
        when the camera has few features to re-register against), and with no
        damping that noise was drawn 1:1, making the icon visibly twitch even while
        physically stationary. This only smooths the on-screen icon - it never
        touches the pose actually sent to the flight controller.
        """
        alpha = 0.35  # lower = smoother/slower to react, higher = snappier/noisier
        if not self._pose_initialized:
            self.drone_x = x
            self.drone_y = y
            self.heading = heading % 360.0
            self._pose_initialized = True
        else:
            self.drone_x += (x - self.drone_x) * alpha
            self.drone_y += (y - self.drone_y) * alpha
            # Shortest angular delta so smoothing doesn't spin the long way around
            # when heading wraps past 359 -> 0.
            delta = ((heading - self.heading + 180.0) % 360.0) - 180.0
            self.heading = (self.heading + delta * alpha) % 360.0

        # Add to trail if drone has moved > 4cm (smoothed position, so the trail
        # matches where the icon is actually drawn) - only while a goal run has
        # ever been started (see trail_visible in __init__ / set_goal()).
        if self.trail_visible:
            if not self.trail:
                self.trail.append((self.drone_x, self.drone_y))
            else:
                lx, ly = self.trail[-1]
                if (self.drone_x - lx)**2 + (self.drone_y - ly)**2 > 0.0016:
                    self.trail.append((self.drone_x, self.drone_y))

        # Auto-follow camera tracking (smooth gliding), tracking the smoothed
        # position so the view doesn't glide toward a spot the icon isn't at.
        if self.auto_follow:
            target_pan_x = -(self.drone_y * self.scale)
            target_pan_y = +(self.drone_x * self.scale)
            self.pan_x += (target_pan_x - self.pan_x) * 0.35
            self.pan_y += (target_pan_y - self.pan_y) * 0.35

        self.update()

    def set_occupancy_grid(
        self, grid: np.ndarray, resolution: float, origin_x: float, origin_y: float, source_name: str
    ):
        """Update live occupancy grid (handles raw /map, thin /map_thin, or bench)."""
        is_thin = "thin" in source_name.lower()
        h, w = grid.shape

        # Item 3: Vectorized single-pass C-speed lookup & upside-down view
        uint8_view = grid.view(np.uint8)

        if is_thin:
            # Thin Skeleton Wall Layer: Crimson Red outline, transparent elsewhere
            rgba_flipped = np.ascontiguousarray(THIN_MAP_LUT[uint8_view][::-1])
            self.map_thin_qimage = QImage(
                rgba_flipped.data, w, h, 4 * w, QImage.Format_RGBA8888
            ).copy()
            self.grid_thin = grid
            self.map_thin_res = resolution
            self.map_thin_ox = origin_x
            self.map_thin_oy = origin_y
        else:
            # Raw Occupancy Grid Layer (Architectural CAD + RViz Parity Engine)
            rgba_flipped = np.ascontiguousarray(RAW_MAP_LUT[uint8_view][::-1])
            self.map_raw_qimage = QImage(
                rgba_flipped.data, w, h, 4 * w, QImage.Format_RGBA8888
            ).copy()
            self.grid_raw = grid
            self.map_raw_res = resolution
            self.map_raw_ox = origin_x
            self.map_raw_oy = origin_y

        # Maintain unified planner reference (prefer raw grid with real obstacle footprint)
        self.occupancy_grid = self.grid_raw if self.grid_raw is not None else self.grid_thin
        self.map_res = self.map_raw_res if self.grid_raw is not None else self.map_thin_res
        self.map_ox = self.map_raw_ox if self.grid_raw is not None else self.map_thin_ox
        self.map_oy = self.map_raw_oy if self.grid_raw is not None else self.map_thin_oy
        self.map_source = source_name

        # If a goal is already staged, re-plan path on updated map
        if self.goal_pose is not None:
            self._replan_path()

        # First time real map data arrives, auto-scale/pan to fit its extent
        if not self._auto_fit_done:
            self._auto_fit_done = True
            self.fit_to_map()

        self.update()

    def _get_min_scale(self) -> float:
        """Item 4: Dynamically compute minimum zoom lower bound from map extent."""
        grid = self.grid_raw if self.grid_raw is not None else self.grid_thin
        res = self.map_raw_res if self.grid_raw is not None else self.map_thin_res
        if grid is not None and res > 0.0:
            gh, gw = grid.shape
            extent_max = max(gh * res, gw * res, 1.0)
            view_min = min(max(self.width(), 100), max(self.height(), 100))
            return max(2.0, min(12.0, (view_min * 0.85) / extent_max))
        return 6.0

    def fit_to_map(self):
        """Auto-scale and center the viewport on the full extent of the loaded
        occupancy grid(s), the same 'zoom to fit' behavior RViz gives you for
        free because it sizes the map to the panel on load."""
        grid = self.grid_raw if self.grid_raw is not None else self.grid_thin
        res = self.map_raw_res if self.grid_raw is not None else self.map_thin_res
        ox = self.map_raw_ox if self.grid_raw is not None else self.map_thin_ox
        oy = self.map_raw_oy if self.grid_raw is not None else self.map_thin_oy
        if grid is None or res <= 0.0:
            return

        gh, gw = grid.shape
        extent_north = gh * res
        extent_east = gw * res
        if extent_north <= 0.0 or extent_east <= 0.0:
            return

        view_w = max(self.width(), 1)
        view_h = max(self.height(), 1)
        margin = 0.85  # leave a little breathing room around the map edges

        scale_from_width = (view_w * margin) / extent_east
        scale_from_height = (view_h * margin) / extent_north
        min_s = self._get_min_scale()
        self.scale = max(min_s, min(260.0, min(scale_from_width, scale_from_height)))

        north_center = ox + extent_north / 2.0
        east_center = oy + extent_east / 2.0
        self.pan_x = -(east_center * self.scale)
        self.pan_y = +(north_center * self.scale)

        self.update()

    def set_goal(self, x: float, y: float):
        """Set goal pose in world coordinates and compute collision-free path."""
        self.goal_pose = (x, y)
        # A new goal starts a fresh trail recording, replacing whatever was left
        # on screen from the previous goal run (kept visible until now so it
        # could be reviewed after arrival/abort - see trail_visible docstring).
        self.trail.clear()
        self.trail_visible = True
        self._replan_path()
        self.update()

    def clear_goal(self):
        """Clear current goal pose and planned path. Deliberately leaves the
        trail alone - it stays visible as a record of this run until the next
        goal is set (see set_goal()) or it's cleared manually."""
        self.goal_pose = None
        self.planned_waypoints = []
        self.planned_distance = 0.0
        self.planned_est_time = 0.0
        self.path_status_msg = "Goal cleared. Click map to set new Goal Pose."
        self.goal_cleared.emit()
        self.update()

    def clear_trail(self):
        self.trail.clear()
        self.trail_visible = False
        self.update()

    def reset_view(self):
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.scale = 36.0
        self.update()

    def center_on_drone(self):
        """Center the viewport on current drone location and re-engage auto-follow."""
        self.pan_x = -(self.drone_y * self.scale)
        self.pan_y = +(self.drone_x * self.scale)
        self.set_auto_follow(True)
        self.update()

    def zoom(self, factor: float):
        min_s = self._get_min_scale()
        self.scale = max(min_s, min(260.0, self.scale * factor))
        self.update()

    # -------------------------------------------------------------------------
    # Internal Path Planning
    # -------------------------------------------------------------------------

    def _replan_path(self):
        if self.goal_pose is None:
            return

        if self.occupancy_grid is None:
            # Direct straight line if no map loaded yet
            gx, gy = self.goal_pose
            dist = math.sqrt((gx - self.drone_x)**2 + (gy - self.drone_y)**2)
            self.planned_waypoints = [(gx, gy)]
            self.planned_distance = dist
            self.planned_est_time = dist / 0.5
            self.path_status_msg = f"[WARNING] No SLAM map loaded. Direct line staged: {dist:.2f}m (Load Bench Floorplan or start SLAM for A* obstacle avoidance)"
            self.goal_staged.emit(gx, gy, self.planned_waypoints, dist, self.planned_est_time)
            return

        # Run A* on active occupancy grid
        res = self.planner.plan(
            self.occupancy_grid,
            self.map_res,
            self.map_ox,
            self.map_oy,
            start_world=(self.drone_x, self.drone_y),
            goal_world=self.goal_pose,
        )

        if res["success"]:
            self.planned_waypoints = res["waypoints"]
            self.planned_distance = res["total_distance_m"]
            self.planned_est_time = res["est_flight_time_s"]
            self.path_status_msg = f"[OK] A* Path: {len(self.planned_waypoints)} WPTs | {self.planned_distance:.2f}m (~{self.planned_est_time:.1f}s)"
            self.goal_staged.emit(
                self.goal_pose[0], self.goal_pose[1],
                self.planned_waypoints, self.planned_distance, self.planned_est_time
            )
        else:
            self.planned_waypoints = []
            self.planned_distance = 0.0
            self.planned_est_time = 0.0
            self.path_status_msg = f"[BLOCKED] {res['message']}"
            self.goal_staged.emit(self.goal_pose[0], self.goal_pose[1], [], 0.0, 0.0)

    # -------------------------------------------------------------------------
    # Coordinate Conversions
    # -------------------------------------------------------------------------

    def _world_to_screen(self, x: float, y: float, cx: float, cy: float) -> QPointF:
        """Convert NED (North=x, East=y) to Canvas Screen pixels."""
        sx = cx + (y * self.scale)
        sy = cy - (x * self.scale)
        return QPointF(sx, sy)

    def _screen_to_world(self, sx: float, sy: float, cx: float, cy: float) -> Tuple[float, float]:
        """Convert Canvas Screen pixels to NED (North=x, East=y) in meters, accounting for rotation."""
        dx = sx - cx
        dy = sy - cy
        if abs(self.rotation_deg) > 0.001:
            rad = math.radians(-self.rotation_deg)
            c = math.cos(rad)
            s = math.sin(rad)
            unrot_dx = dx * c - dy * s
            unrot_dy = dx * s + dy * c
            dx = unrot_dx
            dy = unrot_dy
        y = dx / self.scale
        x = -dy / self.scale
        return x, y

    # -------------------------------------------------------------------------
    # Mouse & Interactive Events
    # -------------------------------------------------------------------------

    def mousePressEvent(self, event: QMouseEvent):
        cx = self.width() / 2.0 + self.pan_x
        cy = self.height() / 2.0 + self.pan_y

        if event.button() == Qt.LeftButton:
            # Set Goal Pose at click point
            gx, gy = self._screen_to_world(event.pos().x(), event.pos().y(), cx, cy)
            self.set_goal(gx, gy)
        elif event.button() in (Qt.RightButton, Qt.MiddleButton):
            # Begin panning map
            self._dragging_pan = True
            self._last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event: QMouseEvent):
        if self._dragging_pan and self._last_mouse_pos is not None:
            delta = event.pos() - self._last_mouse_pos
            self.pan_x += delta.x()
            self.pan_y += delta.y()
            self._last_mouse_pos = event.pos()
            # If user manually drags canvas, pause auto-follow
            if self.auto_follow:
                self.auto_follow = False
                self.auto_follow_changed.emit(False)
            self.update()

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() in (Qt.RightButton, Qt.MiddleButton):
            self._dragging_pan = False
            self._last_mouse_pos = None

    def wheelEvent(self, event: QWheelEvent):
        # Zoom with wheel
        num_degrees = event.angleDelta().y() / 8.0
        num_steps = num_degrees / 15.0
        factor = 1.15 ** num_steps
        self.zoom(factor)

    # -------------------------------------------------------------------------
    # Rendering
    # -------------------------------------------------------------------------

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        # Intentionally NOT setting SmoothPixmapTransform: RViz renders occupancy
        # grid cells as crisp flat quads with no interpolation. Bilinear smoothing
        # here blurs the map more the further you zoom in, since each map pixel
        # gets stretched across many screen pixels. Nearest-neighbor (the default
        # when this hint is unset) keeps cell edges sharp at any zoom level.

        w = self.width()
        h = self.height()
        cx = w / 2.0 + self.pan_x
        cy = h / 2.0 + self.pan_y

        # 1. Clean Crisp White CAD Background
        p.fillRect(0, 0, w, h, QColor(255, 255, 255))

        # 2. Planar Rotated Scene Elements
        p.save()
        p.translate(cx, cy)
        p.rotate(self.rotation_deg)
        p.translate(-cx, -cy)

        # Metric Grid Lines
        self._draw_grid(p, cx, cy, w, h)

        # Live 2D Occupancy Grid (if loaded)
        self._draw_occupancy_grid(p, cx, cy)

        # Trajectory Breadcrumb Trail
        self._draw_trail(p, cx, cy)

        # Planned A* Collision-Free Path
        self._draw_planned_path(p, cx, cy)

        # Goal Pose Crosshair
        self._draw_goal_crosshair(p, cx, cy)

        # Drone Icon with Heading Vector
        self._draw_drone(p, cx, cy)

        p.restore()

        # 3. Legends, Status Badge, and Metric Scale Bar (unrotated for upright clarity)
        self._draw_overlay(p, w, h)

        p.end()

    def _draw_grid(self, p: QPainter, cx: float, cy: float, w: float, h: float):
        p.save()
        p.setFont(QFont("Segoe UI", 7))
        grid_step = self.scale

        dim = max(w, h) * 2.0
        x_min = int((-cx - dim) / grid_step)
        x_max = int((w - cx + dim) / grid_step)
        y_min = int((-cy - dim) / grid_step)
        y_max = int((h - cy + dim) / grid_step)

        for i in range(x_min, x_max):
            x = cx + (i * grid_step)
            is_major = (i % 5 == 0)
            # Item 5: Subtle alpha (60/40) so grid lines never overpower map features
            p.setPen(QPen(QColor(148, 163, 184, 60) if is_major else QColor(203, 213, 225, 40), 1.5 if is_major else 1))
            p.drawLine(QPointF(x, cy - dim), QPointF(x, cy + dim))

        for j in range(y_min, y_max):
            y = cy + (j * grid_step)
            is_major = (j % 5 == 0)
            p.setPen(QPen(QColor(148, 163, 184, 60) if is_major else QColor(203, 213, 225, 40), 1.5 if is_major else 1))
            p.drawLine(QPointF(cx - dim, y), QPointF(cx + dim, y))

        # Origin (0,0) Cross in high-contrast slate grey with balanced opacity
        p.setPen(QPen(QColor(100, 116, 139, 120), 2))
        p.drawLine(QPointF(cx - 14, cy), QPointF(cx + 14, cy))
        p.drawLine(QPointF(cx, cy - 14), QPointF(cx, cy + 14))
        p.setPen(QColor(100, 116, 139, 140))
        p.drawText(int(cx + 6), int(cy - 6), "(0,0)")
        p.restore()

    def _draw_occupancy_grid(self, p: QPainter, cx: float, cy: float):
        """Render cached occupancy grid QImage(s) properly scaled and aligned to world."""
        # 1. Background Pass: Raw Occupancy Grid (/map - explored space & obstacle mass in slight black)
        if self.display_mode in ("both", "raw") and self.map_raw_qimage is not None and self.grid_raw is not None:
            gh, gw = self.grid_raw.shape
            x_min = self.map_raw_ox
            x_max = self.map_raw_ox + (gh * self.map_raw_res)
            y_min = self.map_raw_oy
            y_max = self.map_raw_oy + (gw * self.map_raw_res)

            screen_left = cx + (y_min * self.scale)
            screen_top = cy - (x_max * self.scale)
            screen_width = (y_max - y_min) * self.scale
            screen_height = (x_max - x_min) * self.scale
            target_rect = QRectF(screen_left, screen_top, screen_width, screen_height)
            p.drawImage(target_rect, self.map_raw_qimage)

        # 2. Foreground Pass: Thin Skeleton Walls (/map_thin - crimson red 1-pixel outline)
        if self.display_mode in ("both", "thin") and self.map_thin_qimage is not None and self.grid_thin is not None:
            gh, gw = self.grid_thin.shape
            x_min = self.map_thin_ox
            x_max = self.map_thin_ox + (gh * self.map_thin_res)
            y_min = self.map_thin_oy
            y_max = self.map_thin_oy + (gw * self.map_thin_res)

            screen_left = cx + (y_min * self.scale)
            screen_top = cy - (x_max * self.scale)
            screen_width = (y_max - y_min) * self.scale
            screen_height = (x_max - x_min) * self.scale
            target_rect = QRectF(screen_left, screen_top, screen_width, screen_height)
            p.drawImage(target_rect, self.map_thin_qimage)

    def _draw_trail(self, p: QPainter, cx: float, cy: float):
        if not self.trail_visible or len(self.trail) < 2:
            return
        p.save()
        p.setPen(QPen(QColor(100, 116, 139, 200), 2, Qt.DashLine))
        pts = [self._world_to_screen(x, y, cx, cy) for x, y in self.trail]
        for i in range(len(pts) - 1):
            p.drawLine(pts[i], pts[i + 1])
        p.restore()

    def _draw_planned_path(self, p: QPainter, cx: float, cy: float):
        """Draw emerald-green A* planned route with waypoint nodes and wingspan safety corridor."""
        if not self.planned_waypoints:
            return

        p.save()
        drone_pt = self._world_to_screen(self.drone_x, self.drone_y, cx, cy)
        screen_pts = [drone_pt] + [self._world_to_screen(wx, wy, cx, cy) for wx, wy in self.planned_waypoints]

        # Check path clearance against occupancy grid (wingspan radius = 0.25m / 50cm diameter)
        tight_clearance = False
        min_clearance_cm = 50.0
        if self.occupancy_grid is not None and self.map_res > 0.001:
            gh, gw = self.occupancy_grid.shape
            for wx, wy in self.planned_waypoints:
                r = int((wx - self.map_ox) / self.map_res)
                c = int((wy - self.map_oy) / self.map_res)
                for dr in range(-5, 6):
                    for dc in range(-5, 6):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < gh and 0 <= nc < gw:
                            if self.occupancy_grid[nr, nc] >= 50:
                                d_cm = math.sqrt(dr*dr + dc*dc) * self.map_res * 100.0
                                if d_cm < min_clearance_cm:
                                    min_clearance_cm = d_cm
                                if d_cm < 20.0:
                                    tight_clearance = True

        # 1. Drone Wingspan Safety Corridor Tube (0.50m diameter)
        corridor_width_px = 0.50 * self.scale
        if tight_clearance:
            # Low clearance warning: Amber/Coral
            p.setPen(QPen(QColor(234, 88, 12, 60), corridor_width_px, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        else:
            # Safe corridor: Translucent Emerald Green
            p.setPen(QPen(QColor(16, 185, 129, 45), corridor_width_px, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))

        for i in range(len(screen_pts) - 1):
            p.drawLine(screen_pts[i], screen_pts[i + 1])

        # 2. Outer Glow & Core Polyline
        p.setPen(QPen(QColor(16, 185, 129, 90), 6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for i in range(len(screen_pts) - 1):
            p.drawLine(screen_pts[i], screen_pts[i + 1])

        p.setPen(QPen(QColor(5, 150, 105, 230), 2.5, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for i in range(len(screen_pts) - 1):
            p.drawLine(screen_pts[i], screen_pts[i + 1])

        # 3. Waypoint nodes and labels
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        for idx, wpt in enumerate(self.planned_waypoints):
            s_pt = self._world_to_screen(wpt[0], wpt[1], cx, cy)
            p.setBrush(QBrush(QColor(5, 150, 105)))
            p.setPen(QPen(QColor(255, 255, 255), 2))
            p.drawEllipse(s_pt, 6, 6)

            label = f"W{idx+1}" if idx < len(self.planned_waypoints) - 1 else "GOAL"
            p.setBrush(QBrush(QColor(15, 23, 42, 220)))
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(QRectF(s_pt.x() + 8, s_pt.y() - 14, 38, 16), 3, 3)
            p.setPen(QColor(255, 255, 255))
            p.drawText(int(s_pt.x() + 12), int(s_pt.y() - 2), label)

        # Clearance warning badge if tight
        if tight_clearance and self.planned_waypoints:
            last_pt = screen_pts[-1]
            p.setBrush(QBrush(QColor(220, 38, 38, 230)))
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(QRectF(last_pt.x() + 10, last_pt.y() + 6, 130, 18), 3, 3)
            p.setPen(QColor(255, 255, 255))
            p.drawText(int(last_pt.x() + 14), int(last_pt.y() + 19), f"MARGIN: {min_clearance_cm:.0f}cm")

        p.restore()

    def _draw_goal_crosshair(self, p: QPainter, cx: float, cy: float):
        """Draw target crosshair at goal pose."""
        if self.goal_pose is None:
            return

        gx, gy = self.goal_pose
        gpt = self._world_to_screen(gx, gy, cx, cy)

        p.save()
        p.setPen(QPen(QColor(217, 119, 6, 240), 2))
        p.setBrush(Qt.NoBrush)

        # Concentric rings
        p.drawEllipse(gpt, 14, 14)
        p.drawEllipse(gpt, 5, 5)
        # Crosshair lines
        p.drawLine(QPointF(gpt.x() - 18, gpt.y()), QPointF(gpt.x() + 18, gpt.y()))
        p.drawLine(QPointF(gpt.x(), gpt.y() - 18), QPointF(gpt.x(), gpt.y() + 18))

        # Coordinate label with dark background pill
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.setBrush(QBrush(QColor(15, 23, 42, 220)))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(QRectF(gpt.x() + 12, gpt.y() + 4, 115, 18), 3, 3)
        p.setPen(QColor(255, 255, 255))
        p.drawText(int(gpt.x() + 16), int(gpt.y() + 17), f"GOAL ({gx:+.2f}, {gy:+.2f})")

        p.restore()

    def _draw_drone(self, p: QPainter, cx: float, cy: float):
        """Draw tactical drone symbol with heading indicator and RealSense FOV cone."""
        p.save()
        d_pt = self._world_to_screen(self.drone_x, self.drone_y, cx, cy)
        p.translate(d_pt)
        p.rotate(self.heading)

        # RealSense D435i Camera FOV Cone (~87 degrees, 2.5m range forward)
        fov_len = 2.5 * self.scale
        fov_half_rad = math.radians(43.5)
        fov_dx = fov_len * math.sin(fov_half_rad)
        fov_dy = fov_len * math.cos(fov_half_rad)
        fov_cone = QPolygonF([
            QPointF(0, 0),
            QPointF(fov_dx, -fov_dy),
            QPointF(-fov_dx, -fov_dy),
        ])
        p.setPen(QPen(QColor(56, 189, 248, 140), 1, Qt.DashLine))
        p.setBrush(QBrush(QColor(56, 189, 248, 35)))
        p.drawPolygon(fov_cone)

        arm_len = 16.0
        p.setPen(QPen(QColor(71, 85, 105), 2.5))
        p.drawLine(QPointF(-arm_len, -arm_len), QPointF(arm_len, arm_len))
        p.drawLine(QPointF(-arm_len, arm_len), QPointF(arm_len, -arm_len))

        # Propeller discs
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(16, 185, 129, 220)))  # Front motors green
        p.drawEllipse(QPointF(-arm_len, -arm_len), 6.5, 6.5)
        p.drawEllipse(QPointF(arm_len, -arm_len), 6.5, 6.5)

        p.setBrush(QBrush(QColor(220, 38, 38, 220)))  # Rear motors red
        p.drawEllipse(QPointF(-arm_len, arm_len), 6.5, 6.5)
        p.drawEllipse(QPointF(arm_len, arm_len), 6.5, 6.5)

        # Front Heading Chevron (pointing North relative to heading)
        p.setBrush(QBrush(QColor(245, 158, 11)))
        chevron = QPolygonF([
            QPointF(0, -arm_len - 10),
            QPointF(6, -arm_len),
            QPointF(-6, -arm_len)
        ])
        p.drawPolygon(chevron)

        # Fuselage (high contrast Dark Navy on white)
        p.setBrush(QBrush(QColor(30, 41, 59)))
        p.setPen(QPen(QColor(15, 23, 42), 2))
        p.drawEllipse(QPointF(0, 0), 8, 8)

        p.restore()

    def _draw_overlay(self, p: QPainter, w: float, h: float):
        """Draw UI overlays: metric scale, pose legend, and active map indicator."""
        p.save()

        # 1. Metric Scale Bar (1 meter)
        bar_len = self.scale
        p.setPen(QPen(QColor(51, 65, 85), 2))
        p.drawLine(QPointF(20, h - 22), QPointF(20 + bar_len, h - 22))
        p.drawLine(QPointF(20, h - 27), QPointF(20, h - 17))
        p.drawLine(QPointF(20 + bar_len, h - 27), QPointF(20 + bar_len, h - 17))

        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.setPen(QColor(51, 65, 85))
        p.drawText(20, int(h - 30), "1.0 Meter")

        # 2. Controls Hint
        p.setFont(QFont("Segoe UI", 7))
        p.setPen(QColor(100, 116, 139))
        p.drawText(20, int(h - 8), "Left Click: Set Goal | Right-Drag: Pan Map | Scroll: Zoom")

        # 3. Live Pose & Orientation Readout
        follow_str = "FOLLOW" if self.auto_follow else "FREE PAN"
        pose_text = (
            f"POSE: N {self.drone_x:+.2f}m | E {self.drone_y:+.2f}m | "
            f"HDG {self.heading:.0f}° | ROT {self.rotation_deg:.0f}° | {follow_str}"
        )
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.setPen(QColor(30, 41, 59))
        p.drawText(int(w - 390), int(h - 18), pose_text)

        # 4. Tactical Orientation Compass Rose (Top-Right)
        comp_cx = w - 50
        comp_cy = 45
        p.save()
        p.translate(comp_cx, comp_cy)
        # Rotate opposite to map rotation so N always points to true physical North!
        p.rotate(-self.rotation_deg)

        # Outer ring
        p.setPen(QPen(QColor(100, 116, 139, 140), 1.5))
        p.setBrush(QBrush(QColor(255, 255, 255, 210)))
        p.drawEllipse(QPointF(0, 0), 22, 22)

        # North pointer (Crimson Red triangle)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(220, 38, 38)))
        p.drawPolygon(QPolygonF([QPointF(0, -20), QPointF(5, -6), QPointF(-5, -6)]))

        # South pointer (Dark Slate triangle)
        p.setBrush(QBrush(QColor(100, 116, 139)))
        p.drawPolygon(QPolygonF([QPointF(0, 20), QPointF(5, 6), QPointF(-5, 6)]))

        # Center pivot
        p.setBrush(QBrush(QColor(15, 23, 42)))
        p.drawEllipse(QPointF(0, 0), 3, 3)

        # North label
        p.setFont(QFont("Segoe UI", 7, QFont.Bold))
        p.setPen(QColor(220, 38, 38))
        p.drawText(QRectF(-10, -32, 20, 12), Qt.AlignCenter, "N")

        p.restore()
        p.restore()


from ui.rviz_embed_widget import RVizEmbedWidget


class SLAMMapWidget(QWidget):
    """
    Complete Tactical SLAM Workspace with dual-view navigation:
    1. 2D Tactical Blueprint: Interactive Click-to-Fly A* Path Planning on /map_thin
    2. 3D RViz2 Viewport: Live embedded ROS 2 RViz2 perception pipeline
    """

    execute_path_requested = pyqtSignal(list)  # list of (x, y) waypoints in meters
    pause_path_requested = pyqtSignal()
    resume_path_requested = pyqtSignal()
    abort_path_requested = pyqtSignal()
    altitude_changed = pyqtSignal(float)
    reset_map_requested = pyqtSignal()  # emitted only after the user confirms

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.is_path_paused: bool = False
        self.current_view_mode: int = 0
        self.map_source: str = "none"
        self._armed: bool = False
        self._has_raw_map: bool = False
        self._has_thin_map: bool = False
        self.rviz_status: str = "OFFLINE"
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        # 1. Industrial Header Toolbar
        header_card = QFrame(self)
        header_card.setProperty("class", "cardFrame")
        header_card.setStyleSheet(
            "QFrame { background-color: #161b22; border: 1px solid #30363d; border-radius: 6px; padding: 4px; }"
        )
        main_hl = QVBoxLayout(header_card)
        main_hl.setContentsMargins(6, 4, 6, 4)
        main_hl.setSpacing(6)

        # Row 1: Merged View Switcher + Unified Status Pill + Goal/Action Bar
        r1 = QHBoxLayout()
        r1.setSpacing(6)

        # Segmented View Mode Switcher [2D Blueprint | 3D Point Cloud] (equal width)
        self.btn_view_2d = QPushButton("2D Blueprint", self)
        self.btn_view_2d.setFixedWidth(150)
        self.btn_view_2d.setMinimumHeight(28)
        self.btn_view_2d.setToolTip("Primary 2D Occupancy Grid & A* Path Planner (Resilient TCP Stream)")
        self.btn_view_2d.clicked.connect(lambda: self._set_view_mode(0))
        r1.addWidget(self.btn_view_2d)

        self.btn_view_rviz = QPushButton("3D Point Cloud", self)
        self.btn_view_rviz.setFixedWidth(150)
        self.btn_view_rviz.setMinimumHeight(28)
        self.btn_view_rviz.setToolTip("Advanced 3D perception inspector (ROS 2 RViz2 PointCloud2 & TF)")
        self.btn_view_rviz.clicked.connect(lambda: self._set_view_mode(1))
        r1.addWidget(self.btn_view_rviz)

        r1.addSpacing(6)

        # Single Unified Status Pill (replaces both MAP and RVIZ badges)
        self.pill_status = QLabel("MAP: NO DATA", self)
        self.pill_status.setMinimumHeight(28)
        self.pill_status.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.pill_status.setAlignment(Qt.AlignCenter)
        self.pill_status.setStyleSheet(
            "background-color: #21262d; color: #8b949e; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 10px; font-size: 10px; font-weight: bold;"
        )
        self.badge_source = self.pill_status  # Backward-compatible alias
        r1.addWidget(self.pill_status)

        # Path / Goal Status Readout
        self.lbl_path_info = QLabel("Click 2D map to stage Goal Pose...", self)
        self.lbl_path_info.setStyleSheet("color: #c9d1d9; font-size: 11px; font-weight: 500;")
        r1.addWidget(self.lbl_path_info, 1)

        # Action Buttons (2D Navigation)
        self.btn_execute_path = QPushButton("EXECUTE PATH", self)
        self.btn_execute_path.setStyleSheet(
            "QPushButton { background-color: #238636; color: #ffffff; font-weight: bold; "
            "border-radius: 4px; padding: 6px 14px; min-height: 28px; }"
            "QPushButton:disabled { background-color: #21262d; color: #484f58; }"
            "QPushButton:hover:!disabled { background-color: #2ea043; }"
        )
        self.btn_execute_path.setEnabled(False)
        self.btn_execute_path.clicked.connect(self._handle_execute_path)
        r1.addWidget(self.btn_execute_path)

        self.btn_pause_path = QPushButton("PAUSE / HOLD", self)
        self.btn_pause_path.setToolTip("Pause path execution and hold position (AUTO.LOITER)")
        self.btn_pause_path.setStyleSheet(
            "QPushButton { background-color: #d29922; color: #ffffff; font-weight: bold; "
            "border-radius: 4px; padding: 6px 12px; min-height: 28px; }"
            "QPushButton:disabled { background-color: #21262d; color: #484f58; }"
            "QPushButton:hover:!disabled { background-color: #e3b341; }"
        )
        self.btn_pause_path.setEnabled(False)
        self.btn_pause_path.clicked.connect(self._handle_pause_path)
        r1.addWidget(self.btn_pause_path)

        self.btn_abort_path = QPushButton("ABORT / LAND", self)
        self.btn_abort_path.setToolTip("Emergency abort path execution and Land immediately")
        self.btn_abort_path.setStyleSheet(
            "QPushButton { background-color: #da3633; color: #ffffff; font-weight: bold; "
            "border-radius: 4px; padding: 6px 12px; min-height: 28px; }"
            "QPushButton:disabled { background-color: #21262d; color: #484f58; }"
            "QPushButton:hover:!disabled { background-color: #f85149; }"
        )
        self.btn_abort_path.setEnabled(False)
        self.btn_abort_path.clicked.connect(self._handle_abort_path)
        r1.addWidget(self.btn_abort_path)

        self.btn_clear_goal = QPushButton("Clear Goal", self)
        self.btn_clear_goal.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 6px 12px; min-height: 28px; }"
            "QPushButton:hover { background-color: #30363d; }"
        )
        self.btn_clear_goal.clicked.connect(self._handle_clear_goal)
        r1.addWidget(self.btn_clear_goal)

        main_hl.addLayout(r1)

        # Row 2: Context Bar that swaps content based on active view mode
        self.context_stack = QStackedWidget(self)

        # Context Page 0: 2D Canvas Controls
        self.row_2d_controls = QWidget(self)
        r2 = QHBoxLayout(self.row_2d_controls)
        r2.setContentsMargins(0, 0, 0, 0)
        r2.setSpacing(6)

        # Single 2D Map Button (composite overlay)
        self.btn_layer_map = QPushButton("2D Map", self)
        self.btn_layer_map.setToolTip("Active 2D Composite Map: Explored Floor + Obstacles + Red Wall Skeleton")
        self.btn_layer_map.setStyleSheet(
            "QPushButton { background-color: #238636; color: #ffffff; border: 1px solid #2ea043; "
            "border-radius: 4px; padding: 4px 12px; min-height: 24px; font-size: 10px; font-weight: bold; }"
        )
        self.btn_layer_map.clicked.connect(lambda: self.set_layer_mode("both"))
        r2.addWidget(self.btn_layer_map)

        # Cruise Altitude Selector
        r2.addSpacing(6)
        lbl_alt = QLabel("Cruise Alt:", self)
        lbl_alt.setStyleSheet("color: #8b949e; font-size: 10px; font-weight: bold;")
        r2.addWidget(lbl_alt)

        self.combo_alt = QComboBox(self)
        self.combo_alt.addItems(["0.8m", "1.0m", "1.2m", "1.5m", "2.0m"])
        self.combo_alt.setCurrentText("1.0m")
        self.combo_alt.setToolTip("Target cruise altitude AGL for autonomous indoor path traversal")
        self.combo_alt.setStyleSheet(
            "QComboBox { background-color: #21262d; color: #58a6ff; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 2px 6px; min-height: 24px; font-size: 10px; font-weight: bold; }"
            "QComboBox::drop-down { border: none; }"
        )
        self.combo_alt.currentTextChanged.connect(self._on_altitude_selected)
        r2.addWidget(self.combo_alt)

        # Planar Turn Controls
        r2.addSpacing(6)
        lbl_orient = QLabel("Turn:", self)
        lbl_orient.setStyleSheet("color: #8b949e; font-size: 10px; font-weight: bold;")
        r2.addWidget(lbl_orient)

        self.btn_turn_left = QPushButton("Left 90°", self)
        self.btn_turn_left.setToolTip("Turn top-down map 90° counter-clockwise")
        self.btn_turn_left.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 8px; min-height: 24px; font-size: 10px; font-weight: bold; }"
            "QPushButton:hover { background-color: #30363d; }"
        )
        self.btn_turn_left.clicked.connect(self._handle_turn_left)
        r2.addWidget(self.btn_turn_left)

        self.btn_turn_right = QPushButton("Right 90°", self)
        self.btn_turn_right.setToolTip("Turn top-down map 90° clockwise")
        self.btn_turn_right.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 8px; min-height: 24px; font-size: 10px; font-weight: bold; }"
            "QPushButton:hover { background-color: #30363d; }"
        )
        self.btn_turn_right.clicked.connect(self._handle_turn_right)
        r2.addWidget(self.btn_turn_right)

        self.btn_reset_rot = QPushButton("Reset 0°", self)
        self.btn_reset_rot.setToolTip("Reset orientation to default North-Up (0°)")
        self.btn_reset_rot.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #8b949e; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 8px; min-height: 24px; font-size: 10px; }"
            "QPushButton:hover { background-color: #30363d; color: #c9d1d9; }"
        )
        self.btn_reset_rot.clicked.connect(self._handle_reset_rotation)
        r2.addWidget(self.btn_reset_rot)

        # Auto-Follow Tracking Toggle
        r2.addSpacing(6)
        self.btn_auto_follow = QPushButton("Auto-Follow: OFF", self)
        self.btn_auto_follow.setToolTip("Vehicle-Centric View: keeps drone locked at screen center as it flies (Default: OFF)")
        self.btn_auto_follow.clicked.connect(self._toggle_auto_follow)
        r2.addWidget(self.btn_auto_follow)
        self._update_auto_follow_button(False)

        r2.addSpacing(6)
        self.btn_center = QPushButton("Center Drone", self)
        self.btn_center.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 10px; min-height: 24px; font-size: 10px; }"
        )
        self.btn_center.clicked.connect(lambda: self.canvas.center_on_drone())
        r2.addWidget(self.btn_center)

        self.btn_zoom_in = QPushButton("+ Zoom In", self)
        self.btn_zoom_in.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 10px; min-height: 24px; font-size: 10px; }"
        )
        self.btn_zoom_in.clicked.connect(lambda: self.canvas.zoom(1.25))
        r2.addWidget(self.btn_zoom_in)

        self.btn_zoom_out = QPushButton("- Zoom Out", self)
        self.btn_zoom_out.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 10px; min-height: 24px; font-size: 10px; }"
        )
        self.btn_zoom_out.clicked.connect(lambda: self.canvas.zoom(0.80))
        r2.addWidget(self.btn_zoom_out)

        self.btn_fit_map = QPushButton("Fit to Map", self)
        self.btn_fit_map.setToolTip("Auto-scale and center the view to the full extent of the current map (RViz-style zoom to fit)")
        self.btn_fit_map.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 10px; min-height: 24px; font-size: 10px; }"
            "QPushButton:hover { background-color: #30363d; }"
        )
        self.btn_fit_map.clicked.connect(lambda: self.canvas.fit_to_map())
        r2.addWidget(self.btn_fit_map)

        self.btn_clear_trail = QPushButton("Clear Trail", self)
        self.btn_clear_trail.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 10px; min-height: 24px; font-size: 10px; }"
        )
        self.btn_clear_trail.clicked.connect(lambda: self.canvas.clear_trail())
        r2.addWidget(self.btn_clear_trail)

        # Reset Map: destructive (wipes the live SLAM database and restarts mapping
        # from empty) - this drone has no local display/keyboard, so an accidental
        # click has no physical undo. Confirmation-gated, and hard-disabled whenever
        # armed (see set_armed_state) - resetting mid-flight would pull the EKF2
        # vision-fusion reference and any live A* obstacle data out from under an
        # actively flying vehicle.
        self.btn_reset_map = QPushButton("Reset Map", self)
        self.btn_reset_map.setToolTip(
            "Wipe the live SLAM map and restart mapping from empty. Disabled while armed."
        )
        self.btn_reset_map.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #f85149; border: 1px solid #da3633; "
            "border-radius: 4px; padding: 4px 10px; min-height: 24px; font-size: 10px; font-weight: bold; }"
            "QPushButton:hover:!disabled { background-color: #da3633; color: #ffffff; }"
            "QPushButton:disabled { background-color: #161b22; color: #484f58; border-color: #21262d; }"
        )
        self.btn_reset_map.clicked.connect(self._handle_reset_map_clicked)
        r2.addWidget(self.btn_reset_map)

        r2.addStretch()
        self.context_stack.addWidget(self.row_2d_controls)

        # Context Page 1: 3D RViz Viewport Controls
        self.row_3d_controls = QWidget(self)
        r3 = QHBoxLayout(self.row_3d_controls)
        r3.setContentsMargins(0, 0, 0, 0)
        r3.setSpacing(6)

        lbl_rviz_hint = QLabel("3D Viewport Controls:", self)
        lbl_rviz_hint.setStyleSheet("color: #8b949e; font-size: 10px; font-weight: bold;")
        r3.addWidget(lbl_rviz_hint)

        self.btn_rviz_launch = QPushButton("Launch RViz2", self)
        self.btn_rviz_launch.setStyleSheet(
            "QPushButton { background-color: #238636; color: #ffffff; font-weight: bold; "
            "border-radius: 4px; padding: 4px 12px; min-height: 24px; font-size: 10px; }"
            "QPushButton:disabled { background-color: #21262d; color: #484f58; }"
            "QPushButton:hover:!disabled { background-color: #2ea043; }"
        )
        r3.addWidget(self.btn_rviz_launch)

        self.btn_rviz_reload = QPushButton("Reload", self)
        self.btn_rviz_reload.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 10px; min-height: 24px; font-size: 10px; }"
            "QPushButton:disabled { background-color: #161b22; color: #484f58; border-color: #21262d; }"
            "QPushButton:hover:!disabled { background-color: #30363d; }"
        )
        self.btn_rviz_reload.setEnabled(False)
        r3.addWidget(self.btn_rviz_reload)

        self.btn_rviz_close = QPushButton("Close", self)
        self.btn_rviz_close.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #f85149; border: 1px solid #da3633; "
            "border-radius: 4px; padding: 4px 10px; min-height: 24px; font-size: 10px; }"
            "QPushButton:disabled { background-color: #161b22; color: #484f58; border-color: #21262d; }"
            "QPushButton:hover:!disabled { background-color: #da3633; color: #ffffff; }"
        )
        self.btn_rviz_close.setEnabled(False)
        r3.addWidget(self.btn_rviz_close)

        lbl_desc = QLabel("3D PointCloud2 & camera TF frame viewer (software-rendered OpenGL)", self)
        lbl_desc.setStyleSheet("color: #8b949e; font-size: 10px; font-style: italic;")
        r3.addWidget(lbl_desc)

        r3.addStretch()
        self.context_stack.addWidget(self.row_3d_controls)

        main_hl.addWidget(self.context_stack)
        layout.addWidget(header_card)

        # 2. Main Stacked Workspace (Index 0: 2D Canvas | Index 1: Embedded RViz2)
        self.view_stack = QStackedWidget(self)

        # Page 0: Interactive 2D SLAM Canvas
        self.canvas = SLAMMapCanvas(self)
        self.canvas.goal_staged.connect(self._on_goal_staged)
        self.canvas.goal_cleared.connect(self._on_goal_cleared)
        self.canvas.auto_follow_changed.connect(self._update_auto_follow_button)
        self.view_stack.addWidget(self.canvas)

        # Page 1: Live Embedded RViz2 Viewport
        self.rviz_widget = RVizEmbedWidget("/home/radxa/Flop/config/rtabmap_drone.rviz", self)
        self.view_stack.addWidget(self.rviz_widget)

        # Wire RViz controls & lifecycle signals
        self.btn_rviz_launch.clicked.connect(self.rviz_widget.launch_rviz)
        self.btn_rviz_reload.clicked.connect(self.rviz_widget.reload_rviz)
        self.btn_rviz_close.clicked.connect(self.rviz_widget.stop_rviz)
        self.rviz_widget.rviz_state_changed.connect(self._on_rviz_state_changed)

        layout.addWidget(self.view_stack, 1)

        # Default to 2D Blueprint view on launch
        self._set_view_mode(0)

    def _handle_turn_left(self):
        self.canvas.turn_left()

    def _handle_turn_right(self):
        self.canvas.turn_right()

    def _handle_reset_rotation(self):
        self.canvas.reset_rotation()

    def _toggle_auto_follow(self):
        new_val = not self.canvas.auto_follow
        self.canvas.set_auto_follow(new_val)

    def _update_auto_follow_button(self, enabled: bool):
        if not hasattr(self, "btn_auto_follow"):
            return
        if enabled:
            self.btn_auto_follow.setText("Auto-Follow: ON")
            self.btn_auto_follow.setStyleSheet(
                "QPushButton { background-color: #238636; color: #ffffff; border: 1px solid #2ea043; "
                "border-radius: 4px; padding: 4px 10px; min-height: 24px; font-size: 10px; font-weight: bold; }"
            )
        else:
            self.btn_auto_follow.setText("Auto-Follow: OFF")
            self.btn_auto_follow.setStyleSheet(
                "QPushButton { background-color: #21262d; color: #8b949e; border: 1px solid #30363d; "
                "border-radius: 4px; padding: 4px 10px; min-height: 24px; font-size: 10px; font-weight: bold; }"
                "QPushButton:hover { background-color: #30363d; color: #c9d1d9; }"
            )

    def set_layer_mode(self, mode: str = "both"):
        """Set visual layer display mode on the 2D canvas."""
        self.canvas.set_display_mode("both")
        self._update_status_pill()

    def _update_status_pill(self):
        """Update single unified status pill based on active view mode and data state."""
        if self.current_view_mode == 0:
            # 2D Blueprint Mode
            if getattr(self, "_has_thin_map", False) and getattr(self, "_has_raw_map", False):
                self.pill_status.setText("2D MAP: LIVE")
                self.pill_status.setToolTip("Active composite: Raw Occupancy Grid (/map) + Red Skeleton Walls (/map_thin)")
                self.pill_status.setStyleSheet(
                    "background-color: rgba(35, 134, 54, 0.13); color: #3fb950; border: 1px solid #238636; "
                    "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
                )
            elif getattr(self, "_has_thin_map", False):
                self.pill_status.setText("LIVE /map_thin")
                self.pill_status.setToolTip("Active red skeleton walls (/map_thin)")
                self.pill_status.setStyleSheet(
                    "background-color: rgba(31, 111, 235, 0.13); color: #58a6ff; border: 1px solid #1f6feb; "
                    "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
                )
            elif getattr(self, "_has_raw_map", False):
                self.pill_status.setText("LIVE /map")
                self.pill_status.setToolTip("Active raw 2.5cm occupancy grid (/map)")
                self.pill_status.setStyleSheet(
                    "background-color: rgba(35, 134, 54, 0.13); color: #3fb950; border: 1px solid #238636; "
                    "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
                )
            elif self.map_source == "bench":
                self.pill_status.setText("MAP: BENCH FLOORPLAN")
                self.pill_status.setStyleSheet(
                    "background-color: rgba(158, 106, 3, 0.13); color: #d29922; border: 1px solid #d29922; "
                    "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
                )
            else:
                self.pill_status.setText("MAP: NO DATA")
                self.pill_status.setStyleSheet(
                    "background-color: #21262d; color: #8b949e; border: 1px solid #30363d; "
                    "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
                )
        else:
            # 3D Point Cloud Mode
            if self.rviz_status == "ACTIVE":
                self.pill_status.setText("RVIZ2: ACTIVE")
                self.pill_status.setStyleSheet(
                    "background-color: rgba(35, 134, 54, 0.13); color: #3fb950; border: 1px solid #238636; "
                    "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
                )
            elif self.rviz_status == "LAUNCHING...":
                self.pill_status.setText("RVIZ2: LAUNCHING...")
                self.pill_status.setStyleSheet(
                    "background-color: rgba(158, 106, 3, 0.13); color: #d29922; border: 1px solid #d29922; "
                    "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
                )
            elif any(err in self.rviz_status for err in ("ERROR", "TIMEOUT", "CRASHED")):
                self.pill_status.setText(f"RVIZ2: {self.rviz_status}")
                self.pill_status.setStyleSheet(
                    "background-color: rgba(218, 54, 51, 0.13); color: #f85149; border: 1px solid #da3633; "
                    "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
                )
            else:
                self.pill_status.setText("RVIZ2: NOT LAUNCHED")
                self.pill_status.setStyleSheet(
                    "background-color: #21262d; color: #8b949e; border: 1px solid #30363d; "
                    "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
                )

    def _set_view_mode(self, mode_idx: int):
        """Toggle between 2D Blueprint (0) and Embedded 3D RViz2 (1)."""
        self.current_view_mode = mode_idx
        self.view_stack.setCurrentIndex(mode_idx)
        self.context_stack.setCurrentIndex(mode_idx)
        if mode_idx == 0:
            self.btn_view_2d.setStyleSheet(
                "QPushButton { background-color: #1f6feb; color: #ffffff; font-weight: bold; "
                "border-radius: 4px; padding: 6px 14px; min-height: 28px; }"
            )
            self.btn_view_rviz.setStyleSheet(
                "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
                "border-radius: 4px; padding: 6px 14px; min-height: 28px; font-weight: normal; }"
                "QPushButton:hover { background-color: #30363d; color: #ffffff; }"
            )
        else:
            self.btn_view_rviz.setStyleSheet(
                "QPushButton { background-color: #1f6feb; color: #ffffff; font-weight: bold; "
                "border-radius: 4px; padding: 6px 14px; min-height: 28px; }"
            )
            self.btn_view_2d.setStyleSheet(
                "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
                "border-radius: 4px; padding: 6px 14px; min-height: 28px; font-weight: normal; }"
                "QPushButton:hover { background-color: #30363d; color: #ffffff; }"
            )
        self._update_status_pill()

    def _on_rviz_state_changed(self, is_running: bool, status_msg: str):
        self.rviz_status = status_msg
        self.btn_rviz_launch.setEnabled(not is_running)
        self.btn_rviz_reload.setEnabled(is_running)
        self.btn_rviz_close.setEnabled(is_running)
        self._update_status_pill()

    def stop_rviz(self):
        """Gracefully terminate embedded rviz2 process."""
        self.rviz_widget.stop_rviz()

    # -------------------------------------------------------------------------
    # Public Slot Methods
    # -------------------------------------------------------------------------

    def _on_altitude_selected(self, text: str):
        val = self.get_cruise_altitude()
        self.altitude_changed.emit(val)

    def get_cruise_altitude(self) -> float:
        text = self.combo_alt.currentText().replace("m", "").strip()
        try:
            return float(text)
        except ValueError:
            return 1.0

    def update_pose(self, x: float, y: float, heading: float):
        self.canvas.set_drone_pose(x, y, heading)

    def set_armed_state(self, armed: bool):
        """Hard-disable Reset Map while armed - wiping the SLAM map mid-flight would
        pull the EKF2 vision-fusion reference and any live A* obstacle data out from
        under an actively flying vehicle. Called from drone_gcs.py on every telemetry
        update, same as every other armed-gated control in this app."""
        self._armed = armed
        self.btn_reset_map.setEnabled(not armed)

    def _handle_reset_map_clicked(self):
        if self._armed:
            return  # belt-and-suspenders - the button is disabled while armed anyway
        reply = QMessageBox.warning(
            self,
            "Reset SLAM Map",
            "This permanently deletes the current SLAM map and restarts mapping "
            "from empty. There is no undo.\n\nContinue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.reset_map_requested.emit()

    def clear_local_map_display(self):
        """Blank the canvas immediately on a confirmed, successful reset - rather than
        waiting for the next /map message to (eventually) show an empty grid."""
        self.canvas.grid_raw = None
        self.canvas.grid_thin = None
        self.canvas.map_raw_qimage = None
        self.canvas.map_thin_qimage = None
        self.canvas.occupancy_grid = None
        self.canvas.trail.clear()
        self.canvas.trail_visible = False
        self.canvas.clear_goal()
        self.canvas._auto_fit_done = False
        self._has_raw_map = False
        self._has_thin_map = False
        self.canvas.update()

    def on_ros2_map_received(
        self, grid: np.ndarray, resolution: float, origin_x: float, origin_y: float, source: str
    ):
        """Called when live /map_thin or /map arrives from ROS 2."""
        if "thin" in source.lower():
            self._has_thin_map = True
        else:
            self._has_raw_map = True
        self.map_source = source
        self._update_status_pill()
        self.canvas.set_occupancy_grid(grid, resolution, origin_x, origin_y, source)

    def load_bench_mock_map(self):
        """Generate and display realistic bench indoor floorplan for testing."""
        from protocol.ros2_map_listener import ROS2MapListener
        grid, res, ox, oy = ROS2MapListener.generate_bench_mock_map()
        self.map_source = "bench"
        self._has_raw_map = True
        self._has_thin_map = False
        self._update_status_pill()
        self.canvas.set_occupancy_grid(grid, res, ox, oy, "Bench Mock Map")

    # -------------------------------------------------------------------------
    # Interaction Handlers
    # -------------------------------------------------------------------------

    def _on_goal_staged(
        self, gx: float, gy: float, waypoints: list, dist: float, est_time: float
    ):
        if waypoints:
            self.lbl_path_info.setText(
                f"GOAL: N {gx:+.2f}m, E {gy:+.2f}m  |  Path: {dist:.2f}m ({len(waypoints)} WPTs, ~{est_time:.1f}s)"
            )
            self.lbl_path_info.setStyleSheet("color: #3fb950; font-weight: bold; font-size: 11px;")
            self.btn_execute_path.setEnabled(True)
        else:
            self.lbl_path_info.setText(f"[BLOCKED] Target N {gx:+.2f}m, E {gy:+.2f}m is BLOCKED by walls!")
            self.lbl_path_info.setStyleSheet("color: #f85149; font-weight: bold; font-size: 11px;")
            self.btn_execute_path.setEnabled(False)

    def _on_goal_cleared(self):
        self.lbl_path_info.setText("Click map to stage Goal Pose...")
        self.lbl_path_info.setStyleSheet("color: #c9d1d9; font-size: 11px;")
        self.btn_execute_path.setEnabled(False)

    def _handle_execute_path(self):
        if self.canvas.planned_waypoints:
            self.execute_path_requested.emit(self.canvas.planned_waypoints)

    def _handle_pause_path(self):
        if self.is_path_paused:
            self.resume_path_requested.emit()
        else:
            self.pause_path_requested.emit()

    def _handle_abort_path(self):
        self.abort_path_requested.emit()

    def set_executing_state(self, is_running: bool, is_paused: bool = False, paused: bool = False):
        """Update button states and visual indicators during path execution."""
        self.is_path_paused = is_paused or paused
        if is_running:
            self.btn_execute_path.setEnabled(False)
            self.btn_pause_path.setEnabled(True)
            self.btn_pause_path.setText("PAUSE / HOLD")
            self.btn_pause_path.setStyleSheet(
                "QPushButton { background-color: #d29922; color: #ffffff; font-weight: bold; "
                "border-radius: 4px; padding: 6px 12px; min-height: 28px; }"
                "QPushButton:hover { background-color: #e3b341; }"
            )
            self.btn_abort_path.setEnabled(True)
            self.btn_clear_goal.setEnabled(False)
        elif is_paused:
            self.btn_execute_path.setEnabled(False)
            self.btn_pause_path.setEnabled(True)
            self.btn_pause_path.setText("RESUME PATH")
            self.btn_pause_path.setStyleSheet(
                "QPushButton { background-color: #238636; color: #ffffff; font-weight: bold; "
                "border-radius: 4px; padding: 6px 12px; min-height: 28px; }"
                "QPushButton:hover { background-color: #2ea043; }"
            )
            self.btn_abort_path.setEnabled(True)
            self.btn_clear_goal.setEnabled(True)
        else:
            self.btn_execute_path.setEnabled(bool(self.canvas.planned_waypoints))
            self.btn_pause_path.setEnabled(False)
            self.btn_pause_path.setText("PAUSE / HOLD")
            self.btn_pause_path.setStyleSheet(
                "QPushButton { background-color: #21262d; color: #484f58; font-weight: bold; "
                "border-radius: 4px; padding: 6px 12px; min-height: 28px; }"
            )
            self.btn_abort_path.setEnabled(False)
            self.btn_clear_goal.setEnabled(True)

    def _handle_clear_goal(self):
        self.canvas.clear_goal()
