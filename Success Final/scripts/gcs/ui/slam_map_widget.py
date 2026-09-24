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
import time
from collections import deque
from typing import Optional, List, Tuple, Dict, Any
import numpy as np

from PyQt5.QtCore import Qt, QRectF, QPointF, pyqtSignal
from PyQt5.QtGui import (
    QPainter, QColor, QFont, QPen, QBrush, QPolygonF, QImage, QWheelEvent, QMouseEvent
)
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QFrame, QSizePolicy, QStackedWidget, QComboBox,
    QMessageBox, QMenu, QAction, QApplication, QButtonGroup
)

from core.path_planner import AStarPathPlanner
from ui.scaling import px
from ui.mission_progress import MissionProgressBar
from ui.styles import PALETTE


# The RViz-exact palettes and the grid -> image conversion now live in
# core/map_render.py, so the map listener's worker thread can build the image
# instead of the GUI thread doing it inside set_occupancy_grid. Re-exported here
# because this is where the rest of the codebase - and the palette tests - have
# always imported them from.
from core import map_render
from core.map_render import (
    build_map_image,
    RVIZ_BACKGROUND, RVIZ_GRID_RGB, RVIZ_GRID_ALPHA, RVIZ_MAP_ALPHA,
    RVIZ_MAP_THIN_ALPHA, RVIZ_GRID_CELL_SIZE_M, RVIZ_GRID_PLANE_CELL_COUNT,
)

# Explicit re-exports rather than bare imports: pyflakes has no noqa, and an
# assignment states the intent better than a suppressed warning would.
RAW_MAP_LUT = map_render.RAW_MAP_LUT
THIN_MAP_LUT = map_render.THIN_MAP_LUT


class SLAMMapCanvas(QWidget):
    """
    High-performance QPainter canvas for 2D SLAM occupancy grid visualization,
    breadcrumb trails, Goal Pose crosshairs, and A* collision-free paths.
    """

    goal_staged = pyqtSignal(float, float, list, float, float)  # (gx, gy, waypoints, dist, est_time)
    goal_cleared = pyqtSignal()
    auto_follow_changed = pyqtSignal(bool)
    rotation_changed = pyqtSignal(float)
    # (global QPoint for menu placement, world north x, world east y). Emitted
    # only for a right-click that did NOT pan - see mouseReleaseEvent.
    context_menu_requested = pyqtSignal(object, float, float)
    ruler_changed = pyqtSignal(float, int)   # running total metres, point count
    # (grid, resolution, origin_x, origin_y, start_world, goal_world). The owning
    # widget forwards this to the planner thread; the answer comes back through
    # apply_plan(). The canvas never calls the planner itself any more.
    plan_requested = pyqtSignal(object, float, float, float, object, object)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(px(450), px(320))
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

        # Right-button press bookkeeping. The right button both pans the map and
        # opens the context menu, so the two are told apart by how far the
        # cursor travelled between press and release: a pan is a drag, a menu is
        # a click. Without the threshold, every pan would end in a popup menu.
        self._right_press_pos: Optional[QPointF] = None
        self._right_moved: bool = False

        # Ruler / measure tool. While active, the LEFT button measures instead
        # of staging a goal - deliberate, so that pacing out a doorway cannot
        # accidentally commit a waypoint to a flying aircraft.
        self.ruler_active: bool = False
        self.ruler_points: List[Tuple[float, float]] = []
        self.ruler_cursor: Optional[Tuple[float, float]] = None

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

        # True between asking for a path and getting one back, so the canvas can
        # say "Planning..." rather than showing a stale path as if it were
        # current. Planning is asynchronous now - see request_replan/apply_plan.
        self.planning: bool = False

        # Map freshness. A frozen map looks exactly like a correct one, which is
        # the whole problem: if the bridge dies mid-flight the last picture sits
        # there showing walls that may no longer be where the aircraft is.
        self.last_map_time: float = 0.0
        self.map_stale_after_s: float = 20.0

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
        self, grid: np.ndarray, resolution: float, origin_x: float,
        origin_y: float, source_name: str, image=None
    ):
        """Adopt a new occupancy layer.

        `image` is the already-rendered QImage from the map listener's thread.
        Building it here is still supported - the bench floorplan and the tests
        call this method directly - but the live path passes one in, because the
        conversion costs about 12 ms on a 25 x 25 m room and the GUI thread
        cannot afford it at the stream's rate.

        This method no longer plans. It used to call _replan_path() on every
        incoming map, so with a goal staged the A* search ran at the map rate -
        5 times a second, 42 to 334 ms each, on the thread that also feeds the
        OFFBOARD setpoint pump. Replanning is now requested and answered
        asynchronously; see plan_requested / apply_plan.
        """
        is_thin = "thin" in source_name.lower()
        if image is None:
            image = build_map_image(grid, thin=is_thin)

        if is_thin:
            self.map_thin_qimage = image
            self.grid_thin = grid
            self.map_thin_res = resolution
            self.map_thin_ox = origin_x
            self.map_thin_oy = origin_y
        else:
            self.map_raw_qimage = image
            self.grid_raw = grid
            self.map_raw_res = resolution
            self.map_raw_ox = origin_x
            self.map_raw_oy = origin_y

        # Unified planner reference (prefer the raw grid - it carries the real
        # obstacle footprint, the skeleton is one pixel wide).
        self.occupancy_grid = self.grid_raw if self.grid_raw is not None else self.grid_thin
        self.map_res = self.map_raw_res if self.grid_raw is not None else self.map_thin_res
        self.map_ox = self.map_raw_ox if self.grid_raw is not None else self.map_thin_ox
        self.map_oy = self.map_raw_oy if self.grid_raw is not None else self.map_thin_oy
        self.map_source = source_name
        self.last_map_time = time.time()

        # A staged goal is replanned against the new map, but off this thread.
        if self.goal_pose is not None:
            self.request_replan()

        if not self._auto_fit_done:
            self._auto_fit_done = True
            self.fit_to_map()

        self.update()

    def map_age_s(self) -> float:
        """Seconds since the last occupancy layer arrived; inf before the first."""
        if not self.last_map_time:
            return float("inf")
        return time.time() - self.last_map_time

    def is_map_stale(self) -> bool:
        return self.last_map_time > 0 and self.map_age_s() > self.map_stale_after_s

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

    # -------------------------------------------------------------------------
    # Ruler / Measure Tool
    # -------------------------------------------------------------------------

    def set_ruler_active(self, active: bool):
        """Enter or leave measure mode. Leaving always clears the measurement:
        a stale dashed line left lying across the map reads as a planned path."""
        self.ruler_active = bool(active)
        if not self.ruler_active:
            self.ruler_points.clear()
            self.ruler_cursor = None
            self.ruler_changed.emit(0.0, 0)
        self.setCursor(Qt.CrossCursor if self.ruler_active else Qt.ArrowCursor)
        self.update()

    def clear_ruler(self):
        self.ruler_points.clear()
        self.ruler_cursor = None
        self.ruler_changed.emit(0.0, 0)
        self.update()

    def add_ruler_point(self, x: float, y: float):
        """Append a vertex. Multi-segment on purpose - an indoor route is rarely
        a straight line, and measuring it in one straight hop understates it."""
        self.ruler_points.append((x, y))
        self.ruler_changed.emit(self.ruler_total_m(), len(self.ruler_points))
        self.update()

    def ruler_total_m(self) -> float:
        total = 0.0
        for a, b in zip(self.ruler_points, self.ruler_points[1:]):
            total += math.hypot(b[0] - a[0], b[1] - a[1])
        return total

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

    def request_replan(self):
        """Ask for a path to the staged goal. Returns immediately.

        With no map loaded there is nothing to search, so a straight line is
        staged directly - that costs nothing and keeps the bench behaviour
        callers rely on. With a map, the request goes to the planner thread and
        the answer arrives in apply_plan().
        """
        if self.goal_pose is None:
            return

        gx, gy = self.goal_pose
        if self.occupancy_grid is None:
            dist = math.hypot(gx - self.drone_x, gy - self.drone_y)
            self.planned_waypoints = [(gx, gy)]
            self.planned_distance = dist
            self.planned_est_time = dist / 0.5
            self.path_status_msg = (
                f"[WARNING] No SLAM map loaded. Direct line staged: {dist:.2f}m "
                "(start SLAM for A* obstacle avoidance)")
            self.planning = False
            self.goal_staged.emit(gx, gy, self.planned_waypoints, dist,
                                  self.planned_est_time)
            self.update()
            return

        self.planning = True
        self.path_status_msg = "Planning..."
        self.plan_requested.emit(
            self.occupancy_grid, self.map_res, self.map_ox, self.map_oy,
            (self.drone_x, self.drone_y), (gx, gy))
        self.update()

    # Kept as the old name so nothing that calls it breaks; it is now a request.
    _replan_path = request_replan

    def apply_plan(self, result: dict):
        """Adopt a planner result. Called on the GUI thread with a result the
        owning widget has already checked is the one it is waiting for."""
        self.planning = False
        if self.goal_pose is None:
            return
        gx, gy = self.goal_pose

        if result and result.get("success"):
            self.planned_waypoints = result["waypoints"]
            self.planned_distance = result["total_distance_m"]
            self.planned_est_time = result.get("est_flight_time_s", 0.0)
            self.path_status_msg = (
                f"[OK] A* Path: {len(self.planned_waypoints)} WPTs | "
                f"{self.planned_distance:.2f}m (~{self.planned_est_time:.1f}s)")
            self.goal_staged.emit(gx, gy, self.planned_waypoints,
                                  self.planned_distance, self.planned_est_time)
        else:
            self.planned_waypoints = []
            self.planned_distance = 0.0
            self.planned_est_time = 0.0
            message = (result or {}).get("message", "No clear path")
            self.path_status_msg = f"[BLOCKED] {message}"
            self.goal_staged.emit(gx, gy, [], 0.0, 0.0)
        self.update()

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

    # Pixels of travel between right-press and right-release above which the
    # gesture counts as a pan rather than a click. Six is about the largest
    # unintended movement a hand makes while clicking, and well below the
    # smallest movement anyone makes when meaning to drag.
    RIGHT_CLICK_SLOP_PX = 6

    def mousePressEvent(self, event: QMouseEvent):
        cx = self.width() / 2.0 + self.pan_x
        cy = self.height() / 2.0 + self.pan_y

        if event.button() == Qt.LeftButton:
            wx, wy = self._screen_to_world(event.pos().x(), event.pos().y(), cx, cy)
            if self.ruler_active:
                # Measuring, not commanding. See set_ruler_active.
                self.add_ruler_point(wx, wy)
            else:
                self.set_goal(wx, wy)
        elif event.button() in (Qt.RightButton, Qt.MiddleButton):
            # Begin panning map. For the right button this is provisional: if
            # the cursor never actually moves, the release opens the context
            # menu instead.
            self._dragging_pan = True
            self._last_mouse_pos = event.pos()
            if event.button() == Qt.RightButton:
                self._right_press_pos = event.pos()
                self._right_moved = False

    def mouseMoveEvent(self, event: QMouseEvent):
        if self.ruler_active and not self._dragging_pan:
            cx = self.width() / 2.0 + self.pan_x
            cy = self.height() / 2.0 + self.pan_y
            self.ruler_cursor = self._screen_to_world(
                event.pos().x(), event.pos().y(), cx, cy)
            if self.ruler_points:
                self.update()

        if self._dragging_pan and self._last_mouse_pos is not None:
            delta = event.pos() - self._last_mouse_pos
            if self._right_press_pos is not None:
                moved = (event.pos() - self._right_press_pos).manhattanLength()
                if moved > self.RIGHT_CLICK_SLOP_PX:
                    self._right_moved = True
            # A right-press that has not yet exceeded the slop must not pan
            # either, or the map creeps by a few pixels every time the menu is
            # opened.
            if self._right_press_pos is not None and not self._right_moved:
                return
            self.pan_x += delta.x()
            self.pan_y += delta.y()
            self._last_mouse_pos = event.pos()
            # If user manually drags canvas, pause auto-follow
            if self.auto_follow:
                self.auto_follow = False
                self.auto_follow_changed.emit(False)
            self.update()

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.RightButton:
            was_click = not self._right_moved
            self._dragging_pan = False
            self._last_mouse_pos = None
            self._right_press_pos = None
            self._right_moved = False
            if was_click:
                cx = self.width() / 2.0 + self.pan_x
                cy = self.height() / 2.0 + self.pan_y
                wx, wy = self._screen_to_world(
                    event.pos().x(), event.pos().y(), cx, cy)
                self.context_menu_requested.emit(
                    self.mapToGlobal(event.pos()), wx, wy)
            return
        if event.button() == Qt.MiddleButton:
            self._dragging_pan = False
            self._last_mouse_pos = None

    def keyPressEvent(self, event):
        """Escape clears an in-progress measurement before anything else sees it."""
        if event.key() == Qt.Key_Escape and (self.ruler_points or self.ruler_cursor):
            self.clear_ruler()
            return
        super().keyPressEvent(event)

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

        # 1. RViz "Background Color: 48; 48; 48". The canvas used to be white,
        # which put a 100%-luminance slab inside an otherwise near-black GCS and
        # disagreed with the embedded RViz view showing the same data.
        p.fillRect(0, 0, w, h, QColor(*RVIZ_BACKGROUND))

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

        # Measure-tool polyline (drawn with the scene so it stays pinned to the
        # world as the map is rotated or panned)
        self._draw_ruler(p, cx, cy)

        # Drone Icon with Heading Vector
        self._draw_drone(p, cx, cy)

        p.restore()

        # 3. Legends, Status Badge, and Metric Scale Bar (unrotated for upright clarity)
        self._draw_overlay(p, w, h)

        # 4. Stale-map warning, on top of everything. Drawn last and deliberately
        # over the map itself rather than as another badge in a corner: the thing
        # that is wrong IS the map, and a corner indicator is exactly what an
        # operator does not look at while flying.
        self._draw_stale_warning(p, w, h)

        p.end()

    def _draw_stale_warning(self, p: QPainter, w: float, h: float):
        """Dim the map and say so when updates have stopped.

        A map that has stopped updating renders identically to one that is
        live - same walls, same colours, same everything. That is dangerous in a
        way a missing map is not: the operator keeps trusting geometry that may
        already be wrong, and clicks goals against it. The dimming is what makes
        it obvious at a glance; the countdown is what tells them how long it has
        been.
        """
        if not self.is_map_stale():
            return

        age = self.map_age_s()
        p.save()
        # Veil the whole canvas. Heavy enough to be unmistakable, light enough
        # that the map is still readable - the operator may still need to see
        # where the vehicle is.
        p.fillRect(0, 0, int(w), int(h), QColor(10, 12, 16, 150))

        amber = QColor(210, 153, 34)
        band_h = 34.0
        band_y = h * 0.5 - band_h / 2.0
        p.setPen(QPen(amber, 2))
        p.setBrush(QBrush(QColor(20, 16, 6, 230)))
        p.drawRoundedRect(QRectF(w * 0.5 - 150, band_y, 300, band_h), 6, 6)

        p.setFont(QFont("Segoe UI", 11, QFont.Bold))
        p.setPen(amber)
        p.drawText(QRectF(w * 0.5 - 150, band_y, 300, band_h), Qt.AlignCenter,
                   f"MAP STALE \u2014 no update for {age:.0f}s")
        p.restore()

    def _draw_grid(self, p: QPainter, cx: float, cy: float, w: float, h: float):
        """RViz Grid display: a finite Cell Size x Plane Cell Count plane on XY.

        RViz does not draw an unbounded grid, and neither does this any more.
        The previous version swept +/- twice the viewport in 1 m steps, which at
        low zoom meant several thousand drawLine calls per repaint at 30 Hz for
        lines that had already merged into a solid wash.
        """
        p.save()
        p.setFont(QFont("Segoe UI", 7))
        step = RVIZ_GRID_CELL_SIZE_M * self.scale
        half = RVIZ_GRID_PLANE_CELL_COUNT / 2.0
        extent = half * RVIZ_GRID_CELL_SIZE_M * self.scale

        grid_pen = QPen(QColor(*RVIZ_GRID_RGB, RVIZ_GRID_ALPHA), 1)
        p.setPen(grid_pen)
        n = int(RVIZ_GRID_PLANE_CELL_COUNT)
        for i in range(n + 1):
            off = -extent + i * step
            p.drawLine(QPointF(cx + off, cy - extent), QPointF(cx + off, cy + extent))
            p.drawLine(QPointF(cx - extent, cy + off), QPointF(cx + extent, cy + off))

        # Origin (0,0) cross, brightened for the dark background.
        p.setPen(QPen(QColor(226, 232, 240, 150), 2))
        p.drawLine(QPointF(cx - 14, cy), QPointF(cx + 14, cy))
        p.drawLine(QPointF(cx, cy - 14), QPointF(cx, cy + 14))
        p.setPen(QColor(226, 232, 240, 170))
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
            p.setOpacity(RVIZ_MAP_ALPHA)
            p.drawImage(target_rect, self.map_raw_qimage)
            p.setOpacity(1.0)

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
            p.setOpacity(RVIZ_MAP_THIN_ALPHA)
            p.drawImage(target_rect, self.map_thin_qimage)
            p.setOpacity(1.0)

    def _draw_trail(self, p: QPainter, cx: float, cy: float):
        if not self.trail_visible or len(self.trail) < 2:
            return
        p.save()
        p.setPen(QPen(QColor(203, 213, 225, 200), 2, Qt.DashLine))
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
        p.setPen(QPen(QColor(203, 213, 225), 2.5))
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
        p.setBrush(QBrush(QColor(71, 85, 105)))
        p.setPen(QPen(QColor(226, 232, 240), 2))
        p.drawEllipse(QPointF(0, 0), 8, 8)

        p.restore()

    def _draw_ruler(self, p: QPainter, cx: float, cy: float):
        """Dashed measure polyline with a per-segment length and a running total.

        Deliberately drawn in amber and dashed, so it can never be mistaken for
        the solid planned A* path in nav blue. The two appear on the same canvas
        and one of them is a flight instruction.
        """
        if not self.ruler_points:
            return

        points = list(self.ruler_points)
        live = self.ruler_cursor if self.ruler_active else None
        if live is not None:
            points.append(live)

        amber = QColor(210, 153, 34)
        screen = [self._world_to_screen(x, y, cx, cy) for x, y in points]

        p.save()
        pen = QPen(amber, 1.8, Qt.DashLine)
        pen.setDashPattern([5, 4])
        p.setPen(pen)
        for a, b in zip(screen, screen[1:]):
            p.drawLine(a, b)

        # Per-segment length, placed at the segment midpoint. Counter-rotated so
        # the text stays upright whatever the map rotation is.
        p.setFont(QFont("Segoe UI", 7, QFont.Bold))
        for (wa, wb), (sa, sb) in zip(zip(points, points[1:]), zip(screen, screen[1:])):
            seg = math.hypot(wb[0] - wa[0], wb[1] - wa[1])
            if seg < 0.02:
                continue
            mid = QPointF((sa.x() + sb.x()) / 2.0, (sa.y() + sb.y()) / 2.0)
            p.save()
            p.translate(mid)
            p.rotate(-self.rotation_deg)
            label = f"{seg:.2f} m"
            p.setPen(QColor(15, 20, 26, 220))
            p.drawText(QRectF(-38, -20, 76, 14), Qt.AlignCenter, label)
            p.setPen(amber)
            p.drawText(QRectF(-39, -21, 76, 14), Qt.AlignCenter, label)
            p.restore()

        # Vertices
        p.setPen(QPen(amber, 1.5))
        p.setBrush(QBrush(QColor(15, 20, 26)))
        for i, pt_ in enumerate(screen):
            if live is not None and i == len(screen) - 1:
                continue
            p.drawEllipse(pt_, 3.5, 3.5)
        p.restore()

    # Round distances a scale bar is allowed to show. Anything else ("0.37 m")
    # is unreadable at a glance, which defeats the point of a scale bar.
    SCALE_SPANS_M = (0.01, 0.02, 0.05, 0.1, 0.2, 0.25, 0.5, 1.0, 2.0, 2.5,
                     5.0, 10.0, 20.0, 25.0, 50.0, 100.0, 200.0, 500.0)
    SCALE_TARGET_PX = 110.0

    def _pick_scale_span(self) -> Tuple[float, float]:
        """Choose the round distance whose on-screen length is closest to the
        target width. Returns (metres, pixels)."""
        if self.scale <= 0:
            return 1.0, self.SCALE_TARGET_PX
        span = min(self.SCALE_SPANS_M,
                   key=lambda m: abs(m * self.scale - self.SCALE_TARGET_PX))
        return span, span * self.scale

    @staticmethod
    def _format_span(span_m: float) -> str:
        """Sub-metre spans read in centimetres: '25 cm' beats '0.25 Meter' on a
        2.5 cm occupancy grid, where centimetres are the working unit."""
        if span_m < 1.0:
            return f"{span_m * 100:.0f} cm"
        if float(span_m).is_integer():
            return f"{int(span_m)} m"
        return f"{span_m:g} m"

    def _draw_overlay(self, p: QPainter, w: float, h: float):
        """Draw UI overlays: metric scale, pose legend, and active map indicator."""
        p.save()

        # 1. Adaptive Metric Scale Bar.
        # The bar used to be hardcoded to exactly one metre of screen distance,
        # which meant it was a handful of pixels wide zoomed out over a whole
        # floor and ran off the side of the canvas zoomed in on a doorway - in
        # both cases telling the operator nothing. It now picks a round distance
        # from a 1-2-5 sequence so the bar stays a readable width at any zoom,
        # and states which distance it picked.
        span_m, bar_len = self._pick_scale_span()
        # Light ink from here down: the canvas is RViz's 48,48,48, so the
        # previous near-black overlay colours were invisible on it.
        # The whole block sits a line higher than it used to: the bar's end
        # ticks reached down to h-17 while the controls hint below it was drawn
        # on the h-8 baseline, so the tick and the "L" of "Left Click" touched.
        p.setPen(QPen(QColor(226, 232, 240, 220), 2))
        p.drawLine(QPointF(20, h - 32), QPointF(20 + bar_len, h - 32))
        p.drawLine(QPointF(20, h - 37), QPointF(20, h - 27))
        p.drawLine(QPointF(20 + bar_len, h - 37), QPointF(20 + bar_len, h - 27))
        # Mid tick: halves the bar by eye, which is how a scale bar is actually
        # read when estimating a distance that is not a whole multiple.
        p.setPen(QPen(QColor(226, 232, 240, 150), 1))
        p.drawLine(QPointF(20 + bar_len / 2.0, h - 35), QPointF(20 + bar_len / 2.0, h - 29))

        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.setPen(QColor(226, 232, 240, 220))
        p.drawText(20, int(h - 42), self._format_span(span_m))

        # Zoom readout, beside the bar: the scale bar says how long a distance
        # is, this says how zoomed in the view is, and they answer different
        # questions when comparing two screenshots of the same room.
        p.setFont(QFont("Segoe UI", 7))
        p.setPen(QColor(148, 163, 184, 200))
        p.drawText(int(24 + bar_len), int(h - 42), f"{self.scale:.0f} px/m")

        # 2. Controls Hint
        p.setFont(QFont("Segoe UI", 7))
        p.setPen(QColor(148, 163, 184, 200))
        if self.ruler_active:
            total = self.ruler_total_m()
            hint = (f"MEASURING - Left Click: add point | Esc: clear | "
                    f"total {total:.2f} m over {max(0, len(self.ruler_points) - 1)} segment(s)")
            p.setPen(QColor(210, 153, 34, 230))
        else:
            hint = ("Left Click: Set Goal | Right-Click: Menu | "
                    "Right-Drag: Pan | Scroll: Zoom")
        p.drawText(20, int(h - 12), hint)

        # 3. Live Pose & Orientation Readout
        follow_str = "FOLLOW" if self.auto_follow else "FREE PAN"
        pose_text = (
            f"POSE: N {self.drone_x:+.2f}m | E {self.drone_y:+.2f}m | "
            f"HDG {self.heading:.0f}° | ROT {self.rotation_deg:.0f}° | {follow_str}"
        )
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.setPen(QColor(226, 232, 240, 230))
        p.drawText(int(w - 390), int(h - 30), pose_text)

        # 4. Tactical Orientation Compass Rose (Top-Right)
        comp_cx = w - 50
        comp_cy = 45
        p.save()
        p.translate(comp_cx, comp_cy)
        # Rotate opposite to map rotation so N always points to true physical North!
        p.rotate(-self.rotation_deg)

        # Outer ring
        p.setPen(QPen(QColor(148, 163, 184, 170), 1.5))
        p.setBrush(QBrush(QColor(22, 27, 34, 220)))
        p.drawEllipse(QPointF(0, 0), 22, 22)

        # North pointer (Crimson Red triangle)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(220, 38, 38)))
        p.drawPolygon(QPolygonF([QPointF(0, -20), QPointF(5, -6), QPointF(-5, -6)]))

        # South pointer (Dark Slate triangle)
        p.setBrush(QBrush(QColor(148, 163, 184)))
        p.drawPolygon(QPolygonF([QPointF(0, 20), QPointF(5, 6), QPointF(-5, 6)]))

        # Center pivot
        p.setBrush(QBrush(QColor(226, 232, 240)))
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
    # "Fly here now" from the map context menu: stage this point and execute it
    # in one action. The main window still routes it through the same interlock
    # checks as the EXECUTE PATH button - this is a shortcut through the UI, not
    # through the safety logic.
    fly_here_requested = pyqtSignal(float, float)

    def __init__(self, parent: Optional[QWidget] = None, rviz_config: str = ""):
        super().__init__(parent)
        # Empty falls back to the copy shipped with this checkout; the caller
        # passes whatever --rviz-config / GCS_RVIZ_CONFIG resolved to.
        self._rviz_config = rviz_config
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

        # 1. Header toolbar.
        #
        # Two rows, both on the design system. Row 1 is what the aircraft does:
        # the view switcher, the staged-goal chip and the four path actions.
        # Row 2 is how the map is shown, grouped into VIEW / MISSION / MAP
        # clusters separated by rules, so that a zoom button and a control that
        # wipes the SLAM database no longer look like the same kind of thing.
        header_card = QFrame(self)
        header_card.setProperty("class", "cardFrame")
        main_hl = QVBoxLayout(header_card)
        main_hl.setContentsMargins(px(8), px(6), px(8), px(6))
        main_hl.setSpacing(px(6))

        # ── Row 1: view switcher | goal chip | path actions ──────────────
        r1 = QHBoxLayout()
        r1.setSpacing(px(4))

        self.view_group = QButtonGroup(self)
        self.view_group.setExclusive(True)

        self.btn_view_2d = self._seg_button(
            "2D Map",
            "Occupancy grid and A* path planner (resilient TCP stream)")
        self.btn_view_2d.setChecked(True)
        self.btn_view_2d.clicked.connect(lambda: self._set_view_mode(0))
        self.view_group.addButton(self.btn_view_2d, 0)
        r1.addWidget(self.btn_view_2d)

        self.btn_view_rviz = self._seg_button(
            "3D Cloud",
            "3D perception inspector (ROS 2 RViz2 PointCloud2 and TF)")
        self.btn_view_rviz.clicked.connect(lambda: self._set_view_mode(1))
        self.view_group.addButton(self.btn_view_rviz, 1)
        r1.addWidget(self.btn_view_rviz)

        # MAP cluster. This is map *state* and a map-level action, so it belongs
        # beside the view switcher rather than among the zoom buttons below -
        # and row 2 could not hold it once the layer control became three
        # buttons instead of one.
        r1.addWidget(self._cluster_rule())
        self.lbl_map_caption = self._cluster_caption("MAP")
        r1.addWidget(self.lbl_map_caption)

        self.pill_status = QLabel("NO DATA", self)
        self.pill_status.setObjectName("mapPill")
        self.pill_status.setMinimumHeight(px(24))
        self.pill_status.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.pill_status.setAlignment(Qt.AlignCenter)
        self.badge_source = self.pill_status  # Backward-compatible alias
        r1.addWidget(self.pill_status)

        # Reset Map wipes the live SLAM database and restarts mapping from
        # empty, on a vehicle with no display or keyboard of its own. It keeps
        # its own colour and never sits inline among the view tools.
        self.btn_reset_map = QPushButton("Reset Map", self)
        self.btn_reset_map.setObjectName("mapDanger")
        self.btn_reset_map.setToolTip(
            "Wipe the live SLAM map and restart mapping from empty. Disabled while armed.")
        self.btn_reset_map.clicked.connect(self._handle_reset_map_clicked)
        r1.addWidget(self.btn_reset_map)

        r1.addStretch(1)

        # The staged-goal chip. Hidden until there is a goal: this row used to
        # carry a ~1000px bordered box reading "Click map to stage a goal",
        # empty for most of a flight and occupying the most prominent space on
        # the tab. The instruction it carried now lives on the canvas overlay,
        # where the click actually happens.
        # MISSION cluster. Cruise altitude belongs beside EXECUTE - it is the
        # altitude the path about to be flown will be flown at - not among the
        # zoom buttons, which is where it used to sit.
        r1.addWidget(self._cluster_caption("MISSION"))

        self.combo_alt = QComboBox(self)
        self.combo_alt.addItems(["0.8m", "1.0m", "1.2m", "1.5m", "2.0m"])
        self.combo_alt.setCurrentText("1.0m")
        self.combo_alt.setToolTip("Cruise altitude AGL for autonomous path traversal")
        self.combo_alt.currentTextChanged.connect(self._on_altitude_selected)
        r1.addWidget(self.combo_alt)

        r1.addWidget(self._cluster_rule())

        self.lbl_path_info = QLabel("", self)
        self.lbl_path_info.setObjectName("goalChip")
        self.lbl_path_info.setVisible(False)
        r1.addWidget(self.lbl_path_info)

        self.btn_execute_path = QPushButton("EXECUTE", self)
        self.btn_execute_path.setObjectName("btnGo")
        self.btn_execute_path.setToolTip("Fly the planned path (Ctrl+E)")
        self.btn_execute_path.setEnabled(False)
        self.btn_execute_path.clicked.connect(self._handle_execute_path)
        r1.addWidget(self.btn_execute_path)

        self.btn_pause_path = QPushButton("PAUSE", self)
        self.btn_pause_path.setObjectName("btnHold")
        self.btn_pause_path.setToolTip("Pause the path and hold position (AUTO.LOITER)")
        self.btn_pause_path.setEnabled(False)
        self.btn_pause_path.clicked.connect(self._handle_pause_path)
        r1.addWidget(self.btn_pause_path)

        self.btn_abort_path = QPushButton("ABORT", self)
        self.btn_abort_path.setObjectName("btnAbort")
        self.btn_abort_path.setToolTip("Abort the path and land immediately")
        self.btn_abort_path.setEnabled(False)
        self.btn_abort_path.clicked.connect(self._handle_abort_path)
        r1.addWidget(self.btn_abort_path)

        self.btn_clear_goal = QPushButton("Clear", self)
        self.btn_clear_goal.setObjectName("mapTool")
        self.btn_clear_goal.setToolTip("Clear the staged goal and its planned path")
        self.btn_clear_goal.clicked.connect(self._handle_clear_goal)
        r1.addWidget(self.btn_clear_goal)

        main_hl.addLayout(r1)

        # ── Row 2: context bar, swapped by view mode ─────────────────────
        self.context_stack = QStackedWidget(self)

        self.row_2d_controls = QWidget(self)
        r2 = QHBoxLayout(self.row_2d_controls)
        r2.setContentsMargins(0, 0, 0, 0)
        r2.setSpacing(px(4))

        # VIEW cluster ---------------------------------------------------
        r2.addWidget(self._cluster_caption("VIEW"))

        # Map layers, as a real three-way control. The canvas has always
        # supported raw / skeleton / both, and the README advertises all three,
        # but the UI was a single button whose handler ignored its own argument
        # and set "both" every time - so the skeleton could never be inspected
        # on its own, which is exactly what docs/slam_evaluation.md asks for.
        self.layer_group = QButtonGroup(self)
        self.layer_group.setExclusive(True)
        self.btn_layer_raw = self._seg_button(
            "Raw", "RTAB-Map occupancy grid only (/map): free space and obstacle mass")
        self.btn_layer_thin = self._seg_button(
            "Thin", "Thinned single-pixel wall skeleton only (/map_thin)")
        self.btn_layer_both = self._seg_button(
            "Both", "Skeleton overlaid on the raw grid (default)")
        self.btn_layer_both.setChecked(True)
        for i, (btn, mode) in enumerate(((self.btn_layer_raw, "raw"),
                                         (self.btn_layer_thin, "thin"),
                                         (self.btn_layer_both, "both"))):
            self.layer_group.addButton(btn, i)
            btn.clicked.connect(lambda _c, m=mode: self.set_layer_mode(m))
            r2.addWidget(btn)
        # Backward-compatible alias for callers that knew the old single button.
        self.btn_layer_map = self.btn_layer_both

        r2.addWidget(self._cluster_rule())

        self.btn_turn_left = self._map_tool("\u21ba 90\u00b0", "Rotate the map 90\u00b0 counter-clockwise")
        self.btn_turn_left.clicked.connect(self._handle_turn_left)
        r2.addWidget(self.btn_turn_left)

        self.btn_turn_right = self._map_tool("\u21bb 90\u00b0", "Rotate the map 90\u00b0 clockwise")
        self.btn_turn_right.clicked.connect(self._handle_turn_right)
        r2.addWidget(self.btn_turn_right)

        self.btn_reset_rot = self._map_tool("0\u00b0", "Reset the map to North-up")
        self.btn_reset_rot.clicked.connect(self._handle_reset_rotation)
        r2.addWidget(self.btn_reset_rot)

        self.btn_auto_follow = self._map_tool(
            "Follow", "Keep the vehicle centred as it flies", checkable=True)
        self.btn_auto_follow.clicked.connect(self._toggle_auto_follow)
        r2.addWidget(self.btn_auto_follow)

        self.btn_center = self._map_tool("Center", "Centre the view on the vehicle")
        self.btn_center.clicked.connect(lambda: self.canvas.center_on_drone())
        r2.addWidget(self.btn_center)

        self.btn_zoom_in = self._map_tool("+", "Zoom in (Ctrl+=)")
        self.btn_zoom_in.clicked.connect(lambda: self.canvas.zoom(1.25))
        r2.addWidget(self.btn_zoom_in)

        self.btn_zoom_out = self._map_tool("\u2212", "Zoom out (Ctrl+-)")
        self.btn_zoom_out.clicked.connect(lambda: self.canvas.zoom(0.80))
        r2.addWidget(self.btn_zoom_out)

        self.btn_fit_map = self._map_tool("Fit", "Fit the view to the whole map (Ctrl+F)")
        self.btn_fit_map.clicked.connect(lambda: self.canvas.fit_to_map())
        r2.addWidget(self.btn_fit_map)

        self.btn_reset_view = self._map_tool("1:1", "Reset zoom and pan to the default overview")
        self.btn_reset_view.clicked.connect(lambda: self.canvas.reset_view())
        r2.addWidget(self.btn_reset_view)

        self.btn_ruler = self._map_tool(
            "Measure",
            "Measure distances. While active, left-click adds a measuring "
            "point instead of staging a goal. Esc clears.",
            checkable=True)
        self.btn_ruler.toggled.connect(self._on_ruler_toggled)
        r2.addWidget(self.btn_ruler)

        self.lbl_ruler = QLabel("", self)
        self.lbl_ruler.setObjectName("rulerTotal")
        self.lbl_ruler.setVisible(False)
        r2.addWidget(self.lbl_ruler)

        # Kept for callers and tests, but off the toolbar: "Clear trail" is in
        # the map right-click menu and the row could not hold both.
        self.btn_clear_trail = QPushButton("Trail", self)
        self.btn_clear_trail.setObjectName("mapTool")
        self.btn_clear_trail.setVisible(False)
        self.btn_clear_trail.clicked.connect(lambda: self.canvas.clear_trail())

        r2.addStretch(1)

        self.context_stack.addWidget(self.row_2d_controls)

        # Context page 1: RViz controls --------------------------------
        self.row_3d_controls = QWidget(self)
        r3 = QHBoxLayout(self.row_3d_controls)
        r3.setContentsMargins(0, 0, 0, 0)
        r3.setSpacing(px(6))

        r3.addWidget(self._cluster_caption("3D VIEWPORT"))

        self.btn_rviz_launch = QPushButton("Launch RViz2", self)
        self.btn_rviz_launch.setObjectName("btnGo")
        r3.addWidget(self.btn_rviz_launch)

        self.btn_rviz_reload = self._map_tool("Reload", "Restart the embedded RViz2 process")
        self.btn_rviz_reload.setEnabled(False)
        r3.addWidget(self.btn_rviz_reload)

        self.btn_rviz_close = QPushButton("Close", self)
        self.btn_rviz_close.setObjectName("mapDanger")
        self.btn_rviz_close.setEnabled(False)
        r3.addWidget(self.btn_rviz_close)

        lbl_desc = QLabel("PointCloud2 and camera TF viewer (software-rendered OpenGL)", self)
        lbl_desc.setObjectName("fieldSubLabel")
        r3.addWidget(lbl_desc)
        r3.addStretch()
        self.context_stack.addWidget(self.row_3d_controls)

        main_hl.addWidget(self.context_stack)
        layout.addWidget(header_card)

        # Route progress. Hidden until a path is staged - an empty progress bar
        # over an idle aircraft is furniture, and this strip is only meaningful
        # when there is a route to be partway through.
        self.progress = MissionProgressBar(self)
        layout.addWidget(self.progress)

        # Live map quality. scripts/diagnostics/map_eval.py has scored SLAM maps
        # properly all along - straightness, squareness, coverage - but only
        # offline, from a terminal, against a recorded run. An operator watching
        # a map build had no way to tell a good one from a bad one. Same
        # thresholds, same maths, shown while it matters.
        self.quality_bar = QFrame(self)
        self.quality_bar.setProperty("class", "cardFrame")
        ql = QHBoxLayout(self.quality_bar)
        ql.setContentsMargins(px(10), px(4), px(10), px(4))
        ql.setSpacing(px(10))
        ql.addWidget(self._cluster_caption("MAP QUALITY"))
        self._quality_cells = {}
        for key, label in (("coverage", "Coverage"), ("explored", "Explored"),
                           ("frontier", "Frontier"), ("wall_rms", "Walls"),
                           ("squareness", "Square"), ("obstacles", "Obstacles")):
            cap = QLabel(label, self)
            cap.setObjectName("execAxis")
            ql.addWidget(cap)
            val = QLabel("--", self)
            val.setObjectName("execReadout")
            ql.addWidget(val)
            ql.addSpacing(px(4))
            self._quality_cells[key] = val
        ql.addStretch(1)
        self.lbl_quality_note = QLabel("", self)
        self.lbl_quality_note.setObjectName("fieldSubLabel")
        ql.addWidget(self.lbl_quality_note)
        self.quality_bar.setVisible(False)
        layout.addWidget(self.quality_bar)

        # 2. Main Stacked Workspace (Index 0: 2D Canvas | Index 1: Embedded RViz2)
        self.view_stack = QStackedWidget(self)

        # Page 0: Interactive 2D SLAM Canvas
        self.canvas = SLAMMapCanvas(self)
        self.canvas.goal_staged.connect(self._on_goal_staged)
        self.canvas.goal_cleared.connect(self._on_goal_cleared)
        self.canvas.auto_follow_changed.connect(self._update_auto_follow_button)
        self.canvas.context_menu_requested.connect(self._on_map_context_menu)
        self.canvas.ruler_changed.connect(self._on_ruler_changed)
        self.canvas.plan_requested.connect(self._on_plan_requested)
        # Needed for the canvas keyPressEvent (Esc clears a measurement) to be
        # reachable at all - without it the canvas never receives key events.
        self.canvas.setFocusPolicy(Qt.StrongFocus)
        self.view_stack.addWidget(self.canvas)

        # Page 1: Live Embedded RViz2 Viewport
        self.rviz_widget = RVizEmbedWidget(self._rviz_config or None, self)
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
        """Reflect follow state on the toggle. The latched look is the
        #mapTool:checked rule in the design system, not an inline sheet."""
        if not hasattr(self, "btn_auto_follow"):
            return
        self.btn_auto_follow.blockSignals(True)
        self.btn_auto_follow.setChecked(bool(enabled))
        self.btn_auto_follow.blockSignals(False)
        self.btn_auto_follow.setText("Follow: ON" if enabled else "Follow")

    LAYER_MODES = ("raw", "thin", "both")

    def set_layer_mode(self, mode: str = "both"):
        """Select which occupancy layers the canvas draws.

        This used to ignore `mode` entirely and call set_display_mode("both")
        every time, so the raw grid and the thinned skeleton could never be
        looked at on their own - despite the canvas supporting all three and the
        README documenting them. Inspecting the skeleton alone is how
        docs/slam_evaluation.md says to judge wall quality.
        """
        if mode not in self.LAYER_MODES:
            mode = "both"
        self.canvas.set_display_mode(mode)
        for btn, name in ((self.btn_layer_raw, "raw"),
                          (self.btn_layer_thin, "thin"),
                          (self.btn_layer_both, "both")):
            btn.blockSignals(True)
            btn.setChecked(name == mode)
            btn.blockSignals(False)
        self._update_status_pill()

    def layer_mode(self) -> str:
        return self.canvas.display_mode

    def _refresh_layer_availability(self):
        """A layer with no data cannot be selected.

        Offering "Raw" before /map has ever arrived would blank the canvas and
        look like a fault rather than an empty topic.
        """
        if not hasattr(self, "btn_layer_raw"):
            return
        has_raw = bool(getattr(self, "_has_raw_map", False))
        has_thin = bool(getattr(self, "_has_thin_map", False))
        self.btn_layer_raw.setEnabled(has_raw)
        self.btn_layer_thin.setEnabled(has_thin)
        self.btn_layer_both.setEnabled(has_raw or has_thin)
        # If the selected layer just went away, fall back rather than showing
        # an empty canvas with a layer button still lit.
        mode = self.canvas.display_mode
        if (mode == "raw" and not has_raw) or (mode == "thin" and not has_thin):
            self.set_layer_mode("both")

    def _update_status_pill(self):
        """Map / RViz state, as text plus one of four QSS states.

        Nine inline stylesheets collapsed into a property the design system
        reads. The wording still distinguishes which topics are actually live,
        because "LIVE /map_thin" and "2D MAP: LIVE" mean different things when
        a bridge has dropped one of the two streams.
        """
        has_thin = bool(getattr(self, "_has_thin_map", False))
        has_raw = bool(getattr(self, "_has_raw_map", False))

        if self.current_view_mode == 0:
            if has_thin and has_raw:
                text, state = "LIVE", "ok"
                tip = "Composite: raw occupancy grid (/map) + wall skeleton (/map_thin)"
            elif has_thin:
                text, state = "LIVE /map_thin", "warn"
                tip = "Only the thinned skeleton is arriving; the raw grid is not"
            elif has_raw:
                text, state = "LIVE /map", "warn"
                tip = "Only the raw occupancy grid is arriving; the skeleton is not"
            elif self.map_source == "bench":
                text, state = "BENCH", "warn"
                tip = "Synthetic bench floorplan - not live vehicle data"
            else:
                text, state = "NO DATA", "bad"
                tip = "No occupancy grid received on either topic"
        else:
            if self.rviz_status == "ACTIVE":
                text, state, tip = "ACTIVE", "ok", "Embedded RViz2 is running"
            elif self.rviz_status == "LAUNCHING...":
                text, state, tip = "LAUNCHING", "warn", "Starting the embedded RViz2 process"
            elif any(err in self.rviz_status for err in ("ERROR", "TIMEOUT", "CRASHED")):
                text, state, tip = str(self.rviz_status), "bad", "RViz2 failed to start"
            else:
                text, state, tip = "NOT LAUNCHED", "idle", "Press Launch RViz2 to start it"

        if hasattr(self, "lbl_map_caption"):
            self.lbl_map_caption.setText("MAP" if self.current_view_mode == 0 else "RVIZ")
        self.pill_status.setText(text)
        self.pill_status.setToolTip(tip)
        self._set_state(self.pill_status, state)
        self._refresh_layer_availability()

    def _set_view_mode(self, mode_idx: int):
        """Toggle between 2D Blueprint (0) and the embedded 3D RViz2 view (1).

        The switcher is a segmented control in a QButtonGroup, so the selected
        look is the #segItem:checked rule rather than four inline stylesheets
        swapped between the two buttons.
        """
        self.current_view_mode = mode_idx
        self.view_stack.setCurrentIndex(mode_idx)
        self.context_stack.setCurrentIndex(mode_idx)
        button = self.view_group.button(mode_idx)
        if button is not None and not button.isChecked():
            button.setChecked(True)
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

    # -------------------------------------------------------------------------
    # Background planning and map scoring
    #
    # Nothing in this section may do the work itself. It forwards requests to
    # the planner thread and applies answers - see core/planner_worker.py for
    # why the work cannot stay on the GUI thread.
    # -------------------------------------------------------------------------

    def attach_planner_worker(self, worker):
        """Adopt the shared planner thread. Called once by the main window."""
        self.planner_worker = worker
        worker.plan_ready.connect(self._on_plan_ready)
        worker.quality_ready.connect(self._on_quality_ready)

    def _on_plan_requested(self, grid, resolution, origin_x, origin_y,
                           start, goal):
        """The canvas wants a path. Forward it; remember which answer is ours."""
        worker = getattr(self, "planner_worker", None)
        if worker is None:
            # No worker attached (bench use, tests): plan inline rather than
            # leaving the goal permanently stuck on "Planning...".
            self.canvas.apply_plan(self.canvas.planner.plan(
                grid, resolution, origin_x, origin_y, start, goal))
            return
        self._plan_token = worker.request_plan(
            grid, resolution, origin_x, origin_y, start, goal)

    def _on_plan_ready(self, token: int, result):
        """Apply a plan, but only the one we are still waiting for.

        A search for an abandoned goal can finish after a newer one - it may
        have been the slow far-corner case - and applying it would quietly
        replace a good path with an obsolete one.
        """
        if token != getattr(self, "_plan_token", None):
            return
        self._plan_token = None
        self.canvas.apply_plan(result)

    def request_map_quality(self):
        """Score the current map. Cheap to ask, answered on the worker thread."""
        worker = getattr(self, "planner_worker", None)
        grid = self.canvas.occupancy_grid
        if worker is None or grid is None or self.canvas.map_res <= 0:
            return
        self._quality_token = worker.request_quality(grid, self.canvas.map_res)

    def _on_quality_ready(self, token: int, quality):
        if token != getattr(self, "_quality_token", None):
            return
        self._quality_token = None
        self._apply_quality(quality)

    def _apply_quality(self, quality):
        """Fill the quality strip. Colour only where there is a real threshold
        to judge against - a number with no pass/fail stays neutral rather than
        implying an opinion the metric does not have."""
        if quality is None or not quality.metrics:
            self.quality_bar.setVisible(False)
            return
        by_key = quality.by_key()
        for key, cell in self._quality_cells.items():
            metric = by_key.get(key)
            if metric is None:
                cell.setText("--")
                cell.setStyleSheet("")
                cell.setToolTip("")
                continue
            cell.setText(metric.text)
            cell.setToolTip(metric.detail)
            if metric.good is None:
                cell.setStyleSheet("")
            else:
                colour = PALETTE["ok"] if metric.good else PALETTE["warn"]
                cell.setStyleSheet(f"color: {colour};")
        failed = [m.label for m in quality.metrics if m.good is False]
        if failed:
            self.lbl_quality_note.setText(
                "below map_eval thresholds: " + ", ".join(failed))
            self.lbl_quality_note.setStyleSheet(f"color: {PALETTE['warn']};")
        else:
            self.lbl_quality_note.setText("within map_eval thresholds")
            self.lbl_quality_note.setStyleSheet(f"color: {PALETTE['text_muted']};")
        self.quality_bar.setVisible(True)

    def set_map_stale_after(self, seconds: float):
        """Threshold for the canvas stale-map warning, from settings.alerts."""
        if seconds and seconds > 0:
            self.canvas.map_stale_after_s = float(seconds)

    # -------------------------------------------------------------------------
    # Toolbar builders
    #
    # Every control on this tab goes through one of these. The point is that the
    # styling lives in ui/styles.py and nowhere else: this module used to carry
    # 47 inline setStyleSheet calls and about 150 hardcoded hex values, all of
    # them duplicates of PALETTE tokens, and all of them invisible to the UI
    # scale factor because inline sheets never pass through scale_qss.
    # -------------------------------------------------------------------------

    def _map_tool(self, text: str, tooltip: str = "",
                  checkable: bool = False) -> QPushButton:
        """A neutral map tool: view, zoom, rotate. Never commands the aircraft."""
        btn = QPushButton(text, self)
        btn.setObjectName("mapTool")
        if tooltip:
            btn.setToolTip(tooltip)
        btn.setCheckable(checkable)
        return btn

    def _seg_button(self, text: str, tooltip: str = "") -> QPushButton:
        """One cell of a segmented control (view mode, map layer)."""
        btn = QPushButton(text, self)
        btn.setObjectName("segItem")
        btn.setCheckable(True)
        if tooltip:
            btn.setToolTip(tooltip)
        return btn

    def _cluster_caption(self, text: str) -> QLabel:
        lbl = QLabel(text, self)
        lbl.setObjectName("mapCaption")
        return lbl

    def _cluster_rule(self) -> QFrame:
        """Vertical separator between toolbar clusters."""
        rule = QFrame(self)
        rule.setObjectName("vDivider")
        rule.setFrameShape(QFrame.VLine)
        rule.setFixedWidth(1)
        return rule

    @staticmethod
    def _set_state(widget, state: str) -> None:
        """Drive a QSS state property and re-polish.

        Qt caches style by property value, so a property change with no
        unpolish/polish pair silently does nothing - which is the trap that
        made inline stylesheets look like the easier option in the first place.
        """
        if widget.property("state") == state:
            return
        widget.setProperty("state", state)
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    # -------------------------------------------------------------------------
    # Map Context Menu  (right-click on the 2D canvas)
    # -------------------------------------------------------------------------

    def _on_map_context_menu(self, global_pos, wx: float, wy: float):
        """Build and show the right-click menu for the clicked world point.

        Every entry here is reachable some other way - this exists because a
        left-click that silently stages a goal was the entire interaction model
        of the map, and nothing on screen said what else the map could do.

        Nothing in this menu bypasses a flight interlock. "Fly here now" emits a
        request; the main window applies the identical armed / OFFBOARD / VIO
        checks it applies to the EXECUTE PATH button.
        """
        menu = QMenu(self)
        menu.setToolTipsVisible(True)

        header = QAction(f"N {wx:+.2f} m,  E {wy:+.2f} m", menu)
        header.setEnabled(False)
        menu.addAction(header)
        menu.addSeparator()

        act_goal = QAction("Set goal here", menu)
        act_goal.setToolTip("Plan an obstacle-aware path to this point (same as left-click)")
        act_goal.triggered.connect(lambda: self.canvas.set_goal(wx, wy))
        menu.addAction(act_goal)

        act_fly = QAction("Fly here now…", menu)
        act_fly.setToolTip(
            "Stage this point and execute immediately. Still requires ARMED, "
            "OFFBOARD and a planned path - the interlocks are unchanged."
        )
        act_fly.triggered.connect(lambda: self._request_fly_here(wx, wy))
        menu.addAction(act_fly)

        # Altitude submenu mirrors the toolbar selector rather than duplicating
        # its list, so the two can never drift apart.
        alt_menu = menu.addMenu("Cruise altitude")
        current_alt = self.combo_alt.currentText()
        for i in range(self.combo_alt.count()):
            text = self.combo_alt.itemText(i)
            act = QAction(text, alt_menu)
            act.setCheckable(True)
            act.setChecked(text == current_alt)
            act.triggered.connect(
                lambda _checked, t=text: self.combo_alt.setCurrentText(t))
            alt_menu.addAction(act)

        menu.addSeparator()

        act_measure = QAction("Measure from here", menu)
        act_measure.setToolTip("Start a measurement with this point as the first vertex")
        act_measure.triggered.connect(lambda: self._start_measure_at(wx, wy))
        menu.addAction(act_measure)

        act_centre = QAction("Centre view here", menu)
        act_centre.triggered.connect(lambda: self._centre_view_on(wx, wy))
        menu.addAction(act_centre)

        menu.addSeparator()

        act_clear_goal = QAction("Clear goal", menu)
        act_clear_goal.setEnabled(self.canvas.goal_pose is not None)
        act_clear_goal.triggered.connect(self._handle_clear_goal)
        menu.addAction(act_clear_goal)

        act_clear_trail = QAction("Clear trail", menu)
        act_clear_trail.triggered.connect(lambda: self.canvas.clear_trail())
        menu.addAction(act_clear_trail)

        act_copy = QAction("Copy coordinates", menu)
        act_copy.setToolTip("Copy 'N, E' in metres to the clipboard")
        act_copy.triggered.connect(lambda: self._copy_coordinates(wx, wy))
        menu.addAction(act_copy)

        menu.exec_(global_pos)

    def _request_fly_here(self, wx: float, wy: float):
        """Stage the goal, then ask the main window to execute the plan.

        The goal is staged first and the plan read back, rather than emitting
        the raw click point: if the A* planner cannot reach it (sealed room,
        point inside a wall) there is no path to fly, and the operator sees the
        BLOCKED readout instead of a command that silently does nothing.
        """
        self.canvas.set_goal(wx, wy)
        if self.canvas.planned_waypoints:
            self.fly_here_requested.emit(wx, wy)

    def _start_measure_at(self, wx: float, wy: float):
        if not self.btn_ruler.isChecked():
            self.btn_ruler.setChecked(True)      # toggled -> _on_ruler_toggled
        else:
            self.canvas.clear_ruler()
        self.canvas.add_ruler_point(wx, wy)

    def _centre_view_on(self, wx: float, wy: float):
        """Pan so the given world point sits at the canvas centre."""
        self.canvas.auto_follow = False
        self.canvas.auto_follow_changed.emit(False)
        self.canvas.pan_x = -wy * self.canvas.scale
        self.canvas.pan_y = wx * self.canvas.scale
        self.canvas.update()

    @staticmethod
    def _copy_coordinates(wx: float, wy: float):
        clipboard = QApplication.clipboard()
        if clipboard is not None:
            clipboard.setText(f"{wx:.3f}, {wy:.3f}")

    # -------------------------------------------------------------------------
    # Measure Tool
    # -------------------------------------------------------------------------

    def _on_ruler_toggled(self, checked: bool):
        self.canvas.set_ruler_active(checked)
        self.lbl_ruler.setVisible(checked)
        if checked:
            self.lbl_ruler.setText("0.00 m")
            self.canvas.setFocus()
        else:
            self.lbl_ruler.setText("")

    def _on_ruler_changed(self, total_m: float, point_count: int):
        segments = max(0, point_count - 1)
        if segments <= 0:
            self.lbl_ruler.setText("0.00 m")
        elif segments == 1:
            self.lbl_ruler.setText(f"{total_m:.2f} m")
        else:
            self.lbl_ruler.setText(f"{total_m:.2f} m  ({segments} segs)")

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
        self, grid: np.ndarray, resolution: float, origin_x: float,
        origin_y: float, source: str, image=None
    ):
        """A live /map or /map_thin layer has arrived.

        `image` is the QImage the listener already rendered on its own thread.
        It stays optional so the bench floorplan and the tests can call this
        without one.
        """
        if "thin" in source.lower():
            self._has_thin_map = True
        else:
            self._has_raw_map = True
        self.map_source = source
        self._update_status_pill()
        self.canvas.set_occupancy_grid(grid, resolution, origin_x, origin_y,
                                       source, image)

        # Score the map, but not on every frame - the scoring is a few
        # milliseconds of numpy and the numbers move slowly. Once a second is
        # faster than anyone reads them.
        now = time.time()
        if now - getattr(self, "_last_quality_request", 0.0) >= 1.0:
            self._last_quality_request = now
            self.request_map_quality()

    def load_bench_mock_map(self):
        """Generate and display realistic bench indoor floorplan for testing."""
        from protocol.ros2_map_listener import ROS2MapListener
        grid, res, ox, oy = ROS2MapListener.generate_bench_mock_map()
        self.map_source = "bench"
        self._has_raw_map = True
        self._has_thin_map = False
        self._update_status_pill()
        self.canvas.set_occupancy_grid(grid, res, ox, oy, "Bench Mock Map")
        self.request_map_quality()

    # -------------------------------------------------------------------------
    # Interaction Handlers
    # -------------------------------------------------------------------------

    def _on_goal_staged(
        self, gx: float, gy: float, waypoints: list, dist: float, est_time: float
    ):
        """Show the staged goal as a compact chip beside the path actions.

        Only visible while a goal exists. The row used to carry a permanently
        present ~1000px box whose text was "Click map to stage a goal" for most
        of a flight; that instruction now lives on the canvas overlay, next to
        where the click happens.
        """
        if waypoints:
            self.lbl_path_info.setText(
                f"GOAL  N {gx:+.2f}  E {gy:+.2f}   \u2022   {dist:.2f} m   "
                f"\u2022   {len(waypoints)} WP   \u2022   ~{est_time:.0f}s")
            self.lbl_path_info.setToolTip(
                "Planned obstacle-aware path to the staged goal")
            self._set_state(self.lbl_path_info, "ok")
            self.btn_execute_path.setEnabled(True)
        else:
            self.lbl_path_info.setText(
                f"BLOCKED  N {gx:+.2f}  E {gy:+.2f}   \u2022   no clear path")
            self.lbl_path_info.setToolTip(
                "The A* planner found no obstacle-free route to this point")
            self._set_state(self.lbl_path_info, "blocked")
            self.btn_execute_path.setEnabled(False)
        self.lbl_path_info.setVisible(True)

    def _on_goal_cleared(self):
        self.lbl_path_info.setVisible(False)
        self.lbl_path_info.setText("")
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
        """Enable/disable the path actions for the current execution state.

        The pause control swaps role between PAUSE and RESUME, so it swaps
        objectName too - amber while it would pause, green while it would
        resume - rather than carrying three inline stylesheets. Disabled
        appearance comes from the shared :disabled rule.
        """
        self.is_path_paused = is_paused or paused

        def set_role(button, role):
            if button.objectName() != role:
                button.setObjectName(role)
                button.style().unpolish(button)
                button.style().polish(button)

        if is_running:
            self.btn_execute_path.setEnabled(False)
            self.btn_pause_path.setEnabled(True)
            self.btn_pause_path.setText("PAUSE")
            self.btn_pause_path.setToolTip("Pause the path and hold position (AUTO.LOITER)")
            set_role(self.btn_pause_path, "btnHold")
            self.btn_abort_path.setEnabled(True)
            self.btn_clear_goal.setEnabled(False)
        elif is_paused:
            self.btn_execute_path.setEnabled(False)
            self.btn_pause_path.setEnabled(True)
            self.btn_pause_path.setText("RESUME")
            self.btn_pause_path.setToolTip("Resume flying the remaining waypoints")
            set_role(self.btn_pause_path, "btnGo")
            self.btn_abort_path.setEnabled(True)
            self.btn_clear_goal.setEnabled(True)
        else:
            self.btn_execute_path.setEnabled(bool(self.canvas.planned_waypoints))
            self.btn_pause_path.setEnabled(False)
            self.btn_pause_path.setText("PAUSE")
            set_role(self.btn_pause_path, "btnHold")
            self.btn_abort_path.setEnabled(False)
            self.btn_clear_goal.setEnabled(True)

    def _handle_clear_goal(self):
        self.canvas.clear_goal()
