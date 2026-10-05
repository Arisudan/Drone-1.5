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
from typing import Optional, List, Tuple
import numpy as np

from PyQt5.QtCore import Qt, QPointF, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QWheelEvent, QMouseEvent
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QBoxLayout, QGridLayout, QPushButton, QLabel, QFrame, QSizePolicy, QStackedWidget, QComboBox,
    QMessageBox, QMenu, QAction, QApplication, QButtonGroup, QListWidget, QScrollArea,
    QListWidgetItem
)

from core.path_planner import AStarPathPlanner
from ui.scaling import px, fit_min_width, grow_min_width
from ui.mission_progress import MissionProgressBar
from ui.styles import PALETTE
from ui import map_canvas_render
from ui.map_canvas_render import CanvasRenderMixin


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

# RVIZ_* above are re-exported for the palette tests; the canvas itself now
# reads them in ui/map_canvas_render.py.
# Explicit re-exports rather than bare imports: pyflakes has no noqa, and an
# assignment states the intent better than a suppressed warning would.
RAW_MAP_LUT = map_render.RAW_MAP_LUT
THIN_MAP_LUT = map_render.THIN_MAP_LUT

# Glyph/ring constants live with the code that draws them (ui/map_canvas_render.py)
# and are re-exported: the zoom tests and the uncertainty maths read them here.
DRONE_GLYPH_REF_SCALE = map_canvas_render.DRONE_GLYPH_REF_SCALE
DRONE_GLYPH_MIN_SCALE = map_canvas_render.DRONE_GLYPH_MIN_SCALE
DRONE_GLYPH_MAX_SCALE = map_canvas_render.DRONE_GLYPH_MAX_SCALE
POS_CONF_K = map_canvas_render.POS_CONF_K
# No valid covariance for this long and the ring is shown as stale: the last
# size is kept, dashed and grey, rather than vanishing - an estimate whose
# quality has stopped being reported is not an estimate known to be good.
POS_UNCERT_STALE_S = 2.0


def _point_in_polygon(x: float, y: float, poly) -> bool:
    """Even-odd test, same rule the planner uses to rasterise zones."""
    inside = False
    n = len(poly)
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[(i + 1) % n]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
    return inside


class SLAMMapCanvas(CanvasRenderMixin, QWidget):
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
    # Keep-out zones changed: the full list of polygons in world (x, y).
    keepout_changed = pyqtSignal(object)
    # Mission stops or their planned legs changed: (stops, legs).
    mission_changed = pyqtSignal(object, object)
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
        # 0.25m matches the real flight planner in drone_gcs.py and the
        # wingspan safety corridor drawn below - this instance is only the
        # fallback used when no PlannerWorker is attached (see
        # attach_planner_worker()), and must not silently plan tighter than
        # production.
        self.planner = AStarPathPlanner(robot_radius_m=0.25)
        self.goal_pose: Optional[Tuple[float, float]] = None
        self.planned_waypoints: List[Tuple[float, float]] = []
        self.planned_distance: float = 0.0
        self.planned_est_time: float = 0.0
        self.path_status_msg: str = "Click on map to set Goal Pose"

        # True between asking for a path and getting one back, so the canvas can
        # say "Planning..." rather than showing a stale path as if it were
        # current. Planning is asynchronous now - see request_replan/apply_plan.
        self.planning: bool = False

        # Multi-stop mission. goal_pose is always the LAST stop, so every path
        # that only knows about a single goal (the goal crosshair, the main
        # window's staged-goal handling) keeps working unchanged; the stops
        # before it are what makes it a mission.
        self.mission_stops: List[Tuple[float, float]] = []
        self.mission_legs: list = []

        # Operator keep-out zones: polygons in world (x, y). Drawn by dragging
        # with the Keep-out tool active; the planner treats them as walls.
        self.keepout_zones: List[List[Tuple[float, float]]] = []
        self.keepout_active: bool = False
        self._keepout_press_px = None          # screen QPoint of drag start
        self._keepout_now_px = None            # screen QPoint under the cursor

        # Inflation band, built on the planner thread (see
        # SLAMMapWidget.request_inflation). Geometry is carried with the image
        # because the map it was built from can differ from the one now shown.
        self.inflation_qimage = None
        self.inflation_geom = None             # (resolution, origin_x, origin_y)
        self.inflation_visible: bool = True
        # Set by the owner just before a side panel opens or closes, so the
        # next resize keeps the map still under the cursor (see resizeEvent).
        self._hold_left_edge_on_resize: bool = False

        # EKF2 horizontal position variance (m^2) and when it was last valid.
        # NaN = never received; see uncertainty_state().
        self.pos_var_n: float = float("nan")
        self.pos_var_e: float = float("nan")
        self.pos_var_time: float = 0.0

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
        """Set goal pose in world coordinates and compute collision-free path.

        A single goal is a one-stop mission: this replaces any mission in
        progress, exactly as a plain left-click always has.
        """
        self.mission_stops = [(x, y)]
        self.mission_legs = []
        self.goal_pose = (x, y)
        # A new goal starts a fresh trail recording, replacing whatever was left
        # on screen from the previous goal run (kept visible until now so it
        # could be reviewed after arrival/abort - see trail_visible docstring).
        self.trail.clear()
        self.trail_visible = True
        self._replan_path()
        self.update()

    # ── multi-stop missions ─────────────────────────────────────────

    def add_mission_stop(self, x: float, y: float):
        """Append a stop to the mission (Shift+click, or the context menu).

        With no mission yet this is the same as set_goal, so the first
        Shift+click behaves like a normal click rather than silently doing
        nothing.
        """
        if not self.mission_stops:
            self.set_goal(x, y)
            return
        self.mission_stops.append((x, y))
        self.goal_pose = (x, y)
        self._mission_edited()

    def remove_mission_stop(self, index: int):
        """Drop one stop. Removing the only stop clears the goal."""
        if not (0 <= index < len(self.mission_stops)):
            return
        del self.mission_stops[index]
        if not self.mission_stops:
            self.clear_goal()
            return
        self.goal_pose = self.mission_stops[-1]
        self._mission_edited()

    def move_mission_stop(self, index: int, delta: int):
        """Reorder: move stop `index` by `delta` places (-1 up, +1 down)."""
        j = index + delta
        if not (0 <= index < len(self.mission_stops)) or not (0 <= j < len(self.mission_stops)):
            return
        stops = self.mission_stops
        stops[index], stops[j] = stops[j], stops[index]
        self.goal_pose = stops[-1]
        self._mission_edited()

    def _mission_edited(self):
        """Any change to the stops invalidates the old route - replan it."""
        self.mission_legs = []
        self.mission_changed.emit(list(self.mission_stops), [])
        self._replan_path()
        self.update()

    def is_mission(self) -> bool:
        return len(self.mission_stops) > 1

    # ── keep-out zones ──────────────────────────────────────────────

    def set_keepout_active(self, active: bool):
        """While active, left-drag draws a keep-out rectangle instead of
        staging a goal. Mutually exclusive with the ruler (the widget enforces
        it) - both repurpose left-click."""
        self.keepout_active = bool(active)
        self._keepout_press_px = None
        self._keepout_now_px = None
        self.setCursor(Qt.CrossCursor if active else Qt.ArrowCursor)
        self.update()

    def add_keepout_zone(self, polygon):
        pts = [(float(x), float(y)) for x, y in polygon]
        if len(pts) >= 3:
            self.keepout_zones.append(pts)
            self._keepouts_edited()

    def remove_keepout_at(self, x: float, y: float) -> bool:
        """Remove the most recently drawn zone containing (x, y)."""
        for i in range(len(self.keepout_zones) - 1, -1, -1):
            if _point_in_polygon(x, y, self.keepout_zones[i]):
                del self.keepout_zones[i]
                self._keepouts_edited()
                return True
        return False

    def zone_at(self, x: float, y: float) -> bool:
        return any(_point_in_polygon(x, y, z) for z in self.keepout_zones)

    def clear_keepouts(self):
        if self.keepout_zones:
            self.keepout_zones = []
            self._keepouts_edited()

    def _keepouts_edited(self):
        """Tell the owner (it pushes zones into the shared planner), then
        replan: a route planned before a zone was drawn may cross it."""
        self.keepout_changed.emit([list(z) for z in self.keepout_zones])
        if self.goal_pose is not None:
            self._replan_path()
        self.update()

    # ── position uncertainty ────────────────────────────────────────

    def set_position_uncertainty(self, var_n: float, var_e: float, stamp: float):
        """Feed the EKF's North/East position variance (m^2) and its timestamp."""
        self.pos_var_n = var_n
        self.pos_var_e = var_e
        self.pos_var_time = stamp

    def uncertainty_state(self, now: Optional[float] = None):
        """-> (state, r95_m). state is one of:

        unknown  no covariance ever received
        stale    none received for POS_UNCERT_STALE_S
        ok       95% radius under half the robot radius
        warn     95% radius eating into the robot radius
        danger   95% radius exceeds the robot radius - the vehicle could be
                 touching walls the map shows it clearing, because the
                 planner's 0.25 m margin is smaller than the position error

        The thresholds are tied to the planner's own radius on purpose: the
        margin the route is planned with is exactly what position error eats.
        """
        if not (math.isfinite(self.pos_var_n) and math.isfinite(self.pos_var_e)):
            return "unknown", float("nan")
        r95 = POS_CONF_K * math.sqrt(max(self.pos_var_n, self.pos_var_e, 0.0))
        now = time.time() if now is None else now
        if now - self.pos_var_time > POS_UNCERT_STALE_S:
            return "stale", r95
        radius = max(1e-3, float(getattr(self.planner, "robot_radius_m", 0.25)))
        if r95 > radius:
            return "danger", r95
        if r95 > 0.5 * radius:
            return "warn", r95
        return "ok", r95

    def set_inflation_image(self, image, geom):
        self.inflation_qimage = image
        self.inflation_geom = geom
        self.update()

    def set_inflation_visible(self, visible: bool):
        self.inflation_visible = bool(visible)
        self.update()

    def clear_goal(self):
        """Clear current goal pose and planned path. Deliberately leaves the
        trail alone - it stays visible as a record of this run until the next
        goal is set (see set_goal()) or it's cleared manually."""
        self.mission_stops = []
        self.mission_legs = []
        self.mission_changed.emit([], [])
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
        stops = list(self.mission_stops) or [(gx, gy)]
        if self.occupancy_grid is None and len(stops) > 1:
            # No map: straight lines through every stop, flagged as unplanned.
            dist, here = 0.0, (self.drone_x, self.drone_y)
            for st in stops:
                dist += math.hypot(st[0] - here[0], st[1] - here[1])
                here = st
            self.planned_waypoints = list(stops)
            self.planned_distance = dist
            self.planned_est_time = dist / 0.5
            self.path_status_msg = (
                f"[WARNING] No SLAM map loaded. Direct lines through "
                f"{len(stops)} stops: {dist:.2f}m")
            self.planning = False
            self.goal_staged.emit(gx, gy, self.planned_waypoints, dist,
                                  self.planned_est_time)
            self.update()
            return
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
        # A mission is sent as a list of stops, a single goal as one point;
        # the planner thread tells them apart by type (see PlannerWorker).
        target = stops if len(stops) > 1 else (gx, gy)
        self.plan_requested.emit(
            self.occupancy_grid, self.map_res, self.map_ox, self.map_oy,
            (self.drone_x, self.drone_y), target)
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
            self.mission_legs = list(result.get("legs") or [])
            self.mission_changed.emit(list(self.mission_stops), self.mission_legs)
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
            self.mission_legs = list((result or {}).get("legs") or [])
            self.mission_changed.emit(list(self.mission_stops), self.mission_legs)
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
    # A keep-out drag smaller than this in either direction is a slip, not a zone.
    KEEPOUT_MIN_PX = 6

    def resizeEvent(self, event):
        """Keep the map still when a side panel takes or returns width.

        The view is centred on the canvas, so narrowing it by a panel's width
        moved the whole map sideways by half of that - including under the
        cursor, on the very click that added the second mission stop. Only
        panel toggles do this; an ordinary window resize still recentres.
        """
        if self._hold_left_edge_on_resize and event.oldSize().width() > 0:
            self.pan_x += (event.oldSize().width() - event.size().width()) / 2.0
            self._hold_left_edge_on_resize = False
        super().resizeEvent(event)

    def mousePressEvent(self, event: QMouseEvent):
        cx = self.width() / 2.0 + self.pan_x
        cy = self.height() / 2.0 + self.pan_y

        if event.button() == Qt.LeftButton:
            wx, wy = self._screen_to_world(event.pos().x(), event.pos().y(), cx, cy)
            if self.ruler_active:
                # Measuring, not commanding. See set_ruler_active.
                self.add_ruler_point(wx, wy)
            elif self.keepout_active:
                self._keepout_press_px = event.pos()
                self._keepout_now_px = event.pos()
            elif event.modifiers() & Qt.ShiftModifier:
                # Shift+click appends a stop: the QGroundControl convention
                # of building a route by clicking points in order.
                self.add_mission_stop(wx, wy)
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
        if self._keepout_press_px is not None:
            self._keepout_now_px = event.pos()
            self.update()
            return
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
        if event.button() == Qt.LeftButton and self._keepout_press_px is not None:
            a, b = self._keepout_press_px, event.pos()
            self._keepout_press_px = self._keepout_now_px = None
            if abs(a.x() - b.x()) >= self.KEEPOUT_MIN_PX and abs(a.y() - b.y()) >= self.KEEPOUT_MIN_PX:
                # Convert all four SCREEN corners, not two world corners: with
                # the map rotated, a screen rectangle is a rotated rectangle in
                # the world, and the zone must be what the operator drew.
                cx = self.width() / 2.0 + self.pan_x
                cy = self.height() / 2.0 + self.pan_y
                corners = [(a.x(), a.y()), (b.x(), a.y()), (b.x(), b.y()), (a.x(), b.y())]
                self.add_keepout_zone(
                    [self._screen_to_world(sx, sy, cx, cy) for sx, sy in corners])
            else:
                self.update()
            return
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
        if event.key() == Qt.Key_Escape and self._keepout_press_px is not None:
            self._keepout_press_px = self._keepout_now_px = None
            self.update()
            return
        if event.key() in (Qt.Key_Backspace, Qt.Key_Delete) and self.mission_stops:
            self.remove_mission_stop(len(self.mission_stops) - 1)
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

    # Round distances a scale bar is allowed to show. Anything else ("0.37 m")
    # is unreadable at a glance, which defeats the point of a scale bar.
    SCALE_SPANS_M = (0.01, 0.02, 0.05, 0.1, 0.2, 0.25, 0.5, 1.0, 2.0, 2.5,
                     5.0, 10.0, 20.0, 25.0, 50.0, 100.0, 200.0, 500.0)
    SCALE_TARGET_PX = 110.0


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
        self._map_ever_seen: bool = False
        self._panel_collapsed: bool = False
        self._actions_running: bool = False
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


        r1.addSpacing(px(8))

        main_hl.addLayout(r1)

        # ── Row 2: context bar, swapped by view mode ─────────────────────
        self.context_stack = QStackedWidget(self)
        self.context_stack.setObjectName("transparentRow")

        self.row_2d_controls = QWidget(self)
        self.row_2d_controls.setObjectName("transparentRow")
        r2 = QHBoxLayout(self.row_2d_controls)
        r2.setContentsMargins(0, 0, 0, 0)
        r2.setSpacing(px(4))

        # VIEW cluster ---------------------------------------------------
        r2.addWidget(self._cluster_caption("VIEW"))

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
            "Follow", "Keep the vehicle centred as it flies", checkable=True,
            extra_labels=("Follow: ON",))
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
        self.row_3d_controls.setObjectName("transparentRow")
        r3 = QHBoxLayout(self.row_3d_controls)
        r3.setContentsMargins(0, 0, 0, 0)
        r3.setSpacing(px(6))

        r3.addWidget(self._cluster_caption("3D VIEWPORT"))

        self.btn_rviz_launch = QPushButton("Launch RViz2", self)
        self.btn_rviz_launch.setObjectName("btnGo")
        self._fit_button_to_text(self.btn_rviz_launch)
        r3.addWidget(self.btn_rviz_launch)

        self.btn_rviz_reload = self._map_tool("Reload", "Restart the embedded RViz2 process")
        self.btn_rviz_reload.setEnabled(False)
        r3.addWidget(self.btn_rviz_reload)

        self.btn_rviz_close = QPushButton("Close", self)
        self.btn_rviz_close.setObjectName("mapDanger")
        self._fit_button_to_text(self.btn_rviz_close)
        self.btn_rviz_close.setEnabled(False)
        r3.addWidget(self.btn_rviz_close)

        lbl_desc = QLabel("PointCloud2 and camera TF viewer (software-rendered OpenGL)", self)
        lbl_desc.setObjectName("fieldSubLabel")
        r3.addWidget(lbl_desc)
        r3.addStretch()
        self.context_stack.addWidget(self.row_3d_controls)

        # One toolbar row: the view switch, then that view's own tools.
        r1.addWidget(self.context_stack, 1)
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
            cap.setObjectName("qualCaption")
            ql.addWidget(cap)
            val = QLabel("--", self)
            val.setObjectName("qualValue")
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
        self.canvas.keepout_changed.connect(self._on_keepout_changed)
        self.canvas.mission_changed.connect(self._on_mission_changed)

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

        # The view stack shares a row with the mission panel, which only
        # appears once the route has more than one stop.
        view_row = QHBoxLayout()
        view_row.setContentsMargins(0, 0, 0, 0)
        view_row.setSpacing(6)
        left_col = QVBoxLayout()
        left_col.setContentsMargins(0, 0, 0, 0)
        left_col.setSpacing(4)
        left_col.addWidget(self.view_stack, 1)
        left_col.addWidget(self._build_status_strip())
        view_row.addLayout(left_col, 1)
        self.side_panel = self._build_side_panel()
        view_row.addWidget(self.side_panel)
        layout.addLayout(view_row, 1)

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
        if has_thin or has_raw:
            self._map_ever_seen = True

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
                # Grey until a map has actually been flowing: "nothing yet" is the
                # normal state at start-up. Red only when a live map has gone away.
                seen = getattr(self, "_map_ever_seen", False)
                text, state = ("MAP LOST", "bad") if seen else ("NO DATA", "idle")
                tip = ("The occupancy grid stopped arriving on both topics" if seen
                       else "No occupancy grid received yet on either topic")
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
        if hasattr(self, "layers_section"):
            self.layers_section.setVisible(self.current_view_mode == 0)
        if hasattr(self, "status_strip"):
            self.status_strip.setVisible(self.current_view_mode == 0)
        self.pill_status.setText(text)
        grow_min_width(self.pill_status, h_pad_px=22)
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
        worker.inflation_ready.connect(self._on_inflation_ready)
        # Zones drawn before the worker existed must reach its planner too.
        worker.planner.set_keepout_zones(self.canvas.keepout_zones)

    def _on_plan_requested(self, grid, resolution, origin_x, origin_y,
                           start, goal):
        """The canvas wants a path. Forward it; remember which answer is ours."""
        worker = getattr(self, "planner_worker", None)
        if worker is None:
            # No worker attached (bench use, tests): plan inline rather than
            # leaving the goal permanently stuck on "Planning...".
            planner = self.canvas.planner
            if isinstance(goal, list):
                result = planner.plan_route(grid, resolution, origin_x, origin_y, start, goal)
            else:
                result = planner.plan(grid, resolution, origin_x, origin_y, start, goal)
            self.canvas.apply_plan(result)
            return
        if isinstance(goal, list):
            self._plan_token = worker.request_route(
                grid, resolution, origin_x, origin_y, start, goal)
        else:
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

    # ── inflation layer ────────────────────────────────────────────

    def canvas_set_inflation_visible(self, visible: bool):
        self.canvas.set_inflation_visible(visible)
        if visible:
            self.request_inflation()

    def request_inflation(self):
        """Rebuild the inflation band on the planner thread (~50 ms on a large
        map - far too slow for the GUI thread, which also feeds the OFFBOARD
        setpoint pump). Skipped while the layer is hidden."""
        if not self.canvas.inflation_visible:
            return
        grid = self.canvas.occupancy_grid
        if grid is None or self.canvas.map_res <= 0:
            return
        worker = getattr(self, "planner_worker", None)
        if worker is None:
            from core.map_render import build_inflation_image
            raw, inflated = self.canvas.planner.inflated_mask(
                grid, self.canvas.map_res, self.canvas.map_ox, self.canvas.map_oy)
            self.canvas.set_inflation_image(
                build_inflation_image(raw, inflated),
                (self.canvas.map_res, self.canvas.map_ox, self.canvas.map_oy))
            return
        self._inflation_token = worker.request_inflation(
            grid, self.canvas.map_res, self.canvas.map_ox, self.canvas.map_oy)

    def _on_inflation_ready(self, token: int, image, geom):
        if token != getattr(self, "_inflation_token", None):
            return
        self._inflation_token = None
        self.canvas.set_inflation_image(image, geom)

    # ── keep-out zones ─────────────────────────────────────────────

    def _on_keepout_toggled(self, checked: bool):
        if checked and self.btn_ruler.isChecked():
            self.btn_ruler.setChecked(False)   # both repurpose left-click
        self.canvas.set_keepout_active(checked)
        if checked:
            self.canvas.setFocus()

    def _on_keepout_changed(self, zones):
        """Push zones into every planner that could route the aircraft.

        The worker's planner is the one the main window flies with - flight
        detours and the in-flight collision check both use it - so a zone that
        only reached the canvas's fallback planner would be drawn on screen
        and ignored in the air.
        """
        self.canvas.planner.set_keepout_zones(zones)
        worker = getattr(self, "planner_worker", None)
        if worker is not None:
            worker.planner.set_keepout_zones(zones)
        self.request_inflation()

    # ── side panel: map status, layers, route, path actions ───────────

    def _section_caption(self, text: str) -> QLabel:
        lbl = QLabel(text, self)
        lbl.setObjectName("mapCaption")
        return lbl

    def _build_side_panel(self) -> QFrame:
        """Everything that is not "how the map is zoomed" or "which view".

        It used to be scattered: the map-state pill and Reset Map in the top
        row, the layer switch in the second, Inflation/Keep-out on the canvas's
        own left edge, Measure beside the zoom buttons and the path actions
        in the first row with a chip nobody could read at a glance. One column
        now, in the order the operator works: what the map is (MAP), what is
        drawn on it (LAYERS), what route is staged (ROUTE), then the actions
        that move the aircraft at the bottom, large and shown only when they
        apply - EXECUTE while idle, PAUSE and ABORT while flying.
        """
        panel = QFrame(self)
        panel.setObjectName("sidePanel")
        panel.setProperty("class", "cardFrame")
        self._panel_expanded_w = px(206)
        self._panel_collapsed_w = px(92)
        panel.setFixedWidth(self._panel_expanded_w)
        outer = QVBoxLayout(panel)
        outer.setContentsMargins(px(8), px(6), px(8), px(8))
        outer.setSpacing(px(4))
        # Status, layers and route scroll if the window is too short to hold
        # them; the path actions below stay pinned and always fully visible.
        scroll = QScrollArea(panel)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setObjectName("transparentRow")
        scroll.viewport().setAutoFillBackground(False)
        body = QWidget()
        body.setObjectName("transparentRow")
        v = QVBoxLayout(body)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(px(4))

        # MAP ------------------------------------------------------------
        # The header stays outside the scroll area so the collapse button is
        # always reachable.
        head = QHBoxLayout()
        head.setSpacing(px(4))
        self.lbl_map_caption = self._section_caption("MAP")
        head.addWidget(self.lbl_map_caption)
        self.pill_status = QLabel("NO DATA", self)
        self.pill_status.setObjectName("mapPill")
        self.pill_status.setMinimumHeight(px(24))
        self.pill_status.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.pill_status.setAlignment(Qt.AlignCenter)
        # Floored on today's text only, then grown further in _refresh_map_pill()
        # as wider states are actually seen.
        self._fit_button_to_text(self.pill_status)
        self.badge_source = self.pill_status  # Backward-compatible alias
        head.addWidget(self.pill_status)
        head.addStretch(1)
        self.btn_collapse_panel = QPushButton("\u203a", self)
        self.btn_collapse_panel.setObjectName("mapTool")
        self.btn_collapse_panel.setToolTip("Hide the side panel (keeps the path actions)")
        self.btn_collapse_panel.setFixedWidth(px(26))
        self.btn_collapse_panel.clicked.connect(lambda: self.set_panel_collapsed(not self._panel_collapsed))
        head.addWidget(self.btn_collapse_panel)
        outer.addLayout(head)
        self._panel_head_row = head

        # Reset Map wipes the live SLAM database and restarts mapping from
        # empty, on a vehicle with no display or keyboard of its own. It keeps
        # its own colour and never sits inline among the view tools.
        self.btn_reset_map = QPushButton("Reset Map", self)
        self.btn_reset_map.setObjectName("mapDanger")
        self.btn_reset_map.setToolTip(
            "Wipe the live SLAM map and restart mapping from empty. Disabled while armed.")
        self._fit_button_to_text(self.btn_reset_map)
        self.btn_reset_map.clicked.connect(self._handle_reset_map_clicked)
        v.addWidget(self.btn_reset_map)

        # LAYERS ---------------------------------------------------------
        self.layers_section = QWidget(self)
        self.layers_section.setObjectName("transparentRow")
        lv = QVBoxLayout(self.layers_section)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.setSpacing(px(6))
        lv.addWidget(self._cluster_rule_h())
        lv.addWidget(self._section_caption("LAYERS"))

        # A real three-way control. The canvas has always supported raw /
        # skeleton / both, and the README advertises all three; judging wall
        # quality means looking at the skeleton alone (docs/slam_evaluation.md).
        self.layer_group = QButtonGroup(self)
        self.layer_group.setExclusive(True)
        self.btn_layer_raw = self._seg_button(
            "Raw", "RTAB-Map occupancy grid only (/map): free space and obstacle mass")
        self.btn_layer_thin = self._seg_button(
            "Thin", "Thinned single-pixel wall skeleton only (/map_thin)")
        self.btn_layer_both = self._seg_button(
            "Both", "Skeleton overlaid on the raw grid (default)")
        self.btn_layer_both.setChecked(True)
        seg = QHBoxLayout()
        seg.setSpacing(px(2))
        for i, (btn, mode) in enumerate(((self.btn_layer_raw, "raw"),
                                         (self.btn_layer_thin, "thin"),
                                         (self.btn_layer_both, "both"))):
            self.layer_group.addButton(btn, i)
            btn.clicked.connect(lambda _c, m=mode: self.set_layer_mode(m))
            seg.addWidget(btn)
        lv.addLayout(seg)
        # Backward-compatible alias for callers that knew the old single button.
        self.btn_layer_map = self.btn_layer_both

        tools = QGridLayout()
        tools.setHorizontalSpacing(px(4))
        tools.setVerticalSpacing(px(4))
        self.btn_inflation = self._map_tool(
            "Inflation",
            "Show the planner's safety margin: cells the vehicle's centre cannot "
            "enter because its body would touch a wall or keep-out zone "
            "(RViz costmap 'inscribed' cyan)",
            checkable=True)
        self.btn_inflation.setChecked(True)
        self.btn_inflation.toggled.connect(self.canvas_set_inflation_visible)
        self.btn_keepout = self._map_tool(
            "Keep-out",
            "Draw keep-out zones. While active, left-drag draws a rectangle the "
            "planner will never route through. Right-click a zone to remove it.",
            checkable=True)
        self.btn_keepout.toggled.connect(self._on_keepout_toggled)
        self.btn_ruler = self._map_tool(
            "Measure",
            "Measure distances. While active, left-click adds a measuring "
            "point instead of staging a goal. Esc clears.",
            checkable=True)
        self.btn_ruler.toggled.connect(self._on_ruler_toggled)
        self.lbl_ruler = QLabel("", self)
        self.lbl_ruler.setObjectName("rulerTotal")
        self.lbl_ruler.setVisible(False)
        tools.addWidget(self.btn_inflation, 0, 0)
        tools.addWidget(self.btn_keepout, 0, 1)
        tools.addWidget(self.btn_ruler, 1, 0)
        tools.addWidget(self.lbl_ruler, 1, 1)
        lv.addLayout(tools)
        v.addWidget(self.layers_section)

        # ROUTE ----------------------------------------------------------
        v.addWidget(self._cluster_rule_h())
        v.addWidget(self._section_caption("ROUTE"))
        self.lbl_route_idle = QLabel("Click the map to stage a goal.\nShift+click adds more stops.", self)
        self.lbl_route_idle.setObjectName("fieldSubLabel")
        self.lbl_route_idle.setWordWrap(True)
        v.addWidget(self.lbl_route_idle)

        self.lbl_path_info = QLabel("", self)
        self.lbl_path_info.setObjectName("goalChip")
        self.lbl_path_info.setWordWrap(True)
        self.lbl_path_info.setVisible(False)
        v.addWidget(self.lbl_path_info)

        self.mission_panel = self._build_mission_panel()
        v.addWidget(self.mission_panel)
        self.btn_clear_goal = QPushButton("Clear route", self)
        self.btn_clear_goal.setObjectName("mapTool")
        self.btn_clear_goal.setToolTip("Clear the staged goal and its planned path")
        self._fit_button_to_text(self.btn_clear_goal)
        self.btn_clear_goal.clicked.connect(self._handle_clear_goal)
        v.addWidget(self.btn_clear_goal)
        v.addStretch(1)
        scroll.setWidget(body)
        outer.addWidget(scroll, 1)
        outer.addStretch(0)           # takes the space when the scroll area is hidden
        self._panel_outer = outer
        v = outer

        # CRUISE ALT: the altitude the route about to be flown will be flown at,
        # so it sits directly above EXECUTE rather than in the top bar.
        self._alt_widgets = []
        alt_row = QHBoxLayout()
        alt_row.setSpacing(px(8))
        self._lbl_alt_caption = self._section_caption("CRUISE ALT")
        alt_row.addWidget(self._lbl_alt_caption)
        alt_row.addStretch(1)
        self.combo_alt = QComboBox(self)
        self.combo_alt.addItems(["0.8m", "1.0m", "1.2m", "1.5m", "2.0m"])
        self.combo_alt.setCurrentText("1.0m")
        # padding (20) + border (2) + drop-down arrow well (18) + slack
        fit_min_width(self.combo_alt, ["0.8m", "1.0m", "1.2m", "1.5m", "2.0m"], h_pad_px=46)
        self.combo_alt.setToolTip("Cruise altitude AGL for autonomous path traversal")
        self.combo_alt.currentTextChanged.connect(self._on_altitude_selected)
        alt_row.addWidget(self.combo_alt)
        v.addLayout(alt_row)

        # PATH ACTIONS ---------------------------------------------------
        # Large, because these move the aircraft, and state-aware: only the
        # actions that apply right now are shown (see set_executing_state).
        self.btn_execute_path = QPushButton("EXECUTE", self)
        self.btn_execute_path.setObjectName("btnGo")
        self.btn_execute_path.setToolTip("Fly the planned path (Ctrl+E)")
        self._fit_button_to_text(self.btn_execute_path)
        self.btn_execute_path.setEnabled(False)
        self.btn_execute_path.clicked.connect(self._handle_execute_path)
        v.addWidget(self.btn_execute_path)

        run_row = QBoxLayout(QBoxLayout.LeftToRight)
        self._run_row = run_row
        run_row.setSpacing(px(6))
        self.btn_pause_path = QPushButton("PAUSE", self)
        self.btn_pause_path.setObjectName("btnHold")
        self.btn_pause_path.setToolTip("Pause the path and hold position (AUTO.LOITER)")
        self._fit_button_to_text(self.btn_pause_path, extra_labels=("RESUME",))
        self.btn_pause_path.setEnabled(False)
        self.btn_pause_path.clicked.connect(self._handle_pause_path)
        run_row.addWidget(self.btn_pause_path)
        self.btn_abort_path = QPushButton("ABORT", self)
        self.btn_abort_path.setObjectName("btnAbort")
        self.btn_abort_path.setToolTip("Abort the path and land immediately")
        self._fit_button_to_text(self.btn_abort_path)
        self.btn_abort_path.setEnabled(False)
        self.btn_abort_path.clicked.connect(self._handle_abort_path)
        run_row.addWidget(self.btn_abort_path)
        v.addLayout(run_row)
        for b in (self.btn_execute_path, self.btn_pause_path, self.btn_abort_path):
            b.setMinimumHeight(px(38))

        self._scroll_area = scroll
        self._apply_action_visibility(running=False)
        return panel

    def _build_status_strip(self) -> QFrame:
        """Pose and position-uncertainty, under the map.

        These used to be painted on the canvas's bottom-right corner, where
        they sat on top of the map and the uncertainty line collided with the
        pose line. A strip of their own never covers the map and has room for
        both.
        """
        self.status_strip = QFrame(self)
        self.status_strip.setObjectName("mapStatusStrip")
        h = QHBoxLayout(self.status_strip)
        h.setContentsMargins(px(10), px(3), px(10), px(3))
        h.setSpacing(px(16))
        self.lbl_pose = QLabel("", self)
        self.lbl_pose.setObjectName("statusPose")
        h.addWidget(self.lbl_pose)
        h.addStretch(1)
        self.lbl_uncert = QLabel("", self)
        self.lbl_uncert.setObjectName("statusUncert")
        h.addWidget(self.lbl_uncert)
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status_strip)
        self._status_timer.start(250)
        self._refresh_status_strip()
        return self.status_strip

    def _refresh_status_strip(self) -> None:
        c = self.canvas
        follow = "FOLLOW" if c.auto_follow else "FREE PAN"
        self.lbl_pose.setText(
            f"N {c.drone_x:+.2f}  E {c.drone_y:+.2f} m   "
            f"HDG {c.heading:.0f}\u00b0   ROT {c.rotation_deg:.0f}\u00b0   {follow}")
        state, r95 = c.uncertainty_state()
        if state == "unknown":
            text = "POS \u00b1 --  (no EKF covariance)"
        elif state == "stale":
            text = f"POS \u00b1{r95:.2f} m (95%)  STALE"
        else:
            text = f"POS \u00b1{r95:.2f} m (95%)"
            if state == "danger":
                text += "  > safety margin"
        self.lbl_uncert.setText(text)
        col = c._uncertainty_colour(state)
        self.lbl_uncert.setStyleSheet(f"color: {col.name()};")

    def _cluster_rule_h(self) -> QFrame:
        rule = QFrame(self)
        rule.setObjectName("hDivider")
        rule.setFixedHeight(1)
        return rule

    def panel_collapsed(self) -> bool:
        return self._panel_collapsed

    def set_panel_collapsed(self, collapsed: bool) -> None:
        """Fold the side panel to a narrow strip that keeps only the path actions
        (stacked), giving the map the rest of the width."""
        self._panel_collapsed = bool(collapsed)
        c = self._panel_collapsed
        self.side_panel.setFixedWidth(self._panel_collapsed_w if c else self._panel_expanded_w)
        for w in (self._scroll_area, self.lbl_map_caption, self.pill_status,
                  self._lbl_alt_caption, self.combo_alt):
            w.setVisible(not c)
        self._panel_outer.setStretch(2, 1 if c else 0)
        self._run_row.setDirection(QBoxLayout.TopToBottom if c else QBoxLayout.LeftToRight)
        self.btn_collapse_panel.setText("\u2039" if c else "\u203a")
        self.btn_collapse_panel.setToolTip("Show the side panel" if c else "Hide the side panel (keeps the path actions)")
        self._apply_action_visibility(running=self._actions_running)
        self.canvas.update()

    def _apply_action_visibility(self, running: bool) -> None:
        self._actions_running = running
        """EXECUTE while idle; PAUSE/RESUME and ABORT while a path is flying or held."""
        self.btn_execute_path.setVisible(not running)
        self.btn_pause_path.setVisible(running)
        self.btn_abort_path.setVisible(running)
        self.btn_clear_goal.setVisible(not running and not self._panel_collapsed)

    # ── mission panel ──────────────────────────────────────────────

    def _build_mission_panel(self) -> QFrame:
        panel = QFrame(self)
        panel.setObjectName("missionPanel")
        panel.setProperty("class", "cardFrame")
        v = QVBoxLayout(panel)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(4)
        title = QLabel("STOPS", panel)
        title.setObjectName("mapCaption")
        v.addWidget(title)
        self.lbl_mission_stats = QLabel("", panel)
        self.lbl_mission_stats.setObjectName("fieldSubLabel")
        self.lbl_mission_stats.setWordWrap(True)
        v.addWidget(self.lbl_mission_stats)
        self.list_mission = QListWidget(panel)
        self.list_mission.setToolTip(
            "Stops in flight order. Shift+click the map to add, Backspace removes the last.")
        v.addWidget(self.list_mission)
        row = QHBoxLayout()
        row.setSpacing(3)
        for text, tip, fn in (
                ("▲", "Move selected stop earlier", lambda: self._move_selected(-1)),
                ("▼", "Move selected stop later", lambda: self._move_selected(+1)),
                ("✕", "Remove selected stop", self._remove_selected),
                ("Clear", "Clear the whole mission", self._handle_clear_goal)):
            b = QPushButton(text, panel)
            b.setObjectName("mapTool")
            b.setToolTip(tip)
            b.clicked.connect(fn)
            row.addWidget(b)
        v.addLayout(row)
        hint = QLabel("Shift+click adds a stop", panel)
        hint.setObjectName("fieldSubLabel")
        v.addWidget(hint)
        panel.setVisible(False)
        return panel

    def _selected_stop(self) -> int:
        row = self.list_mission.currentRow()
        return row if row >= 0 else -1

    def _move_selected(self, delta: int):
        i = self._selected_stop()
        if i < 0:
            return
        self.canvas.move_mission_stop(i, delta)
        self.list_mission.setCurrentRow(max(0, min(len(self.canvas.mission_stops) - 1, i + delta)))

    def _remove_selected(self):
        i = self._selected_stop()
        if i >= 0:
            self.canvas.remove_mission_stop(i)

    def _on_mission_changed(self, stops, legs):
        """Refresh the list and the stats line. Leg figures appear only once
        the route has been planned; until then the stops are listed bare."""
        show = len(stops) > 1
        if show == self.mission_panel.isHidden():
            self.canvas._hold_left_edge_on_resize = True
        self.mission_panel.setVisible(show)
        current = self.list_mission.currentRow()
        self.list_mission.clear()
        for i, (x, y) in enumerate(stops):
            # No per-leg distance here: the summary line above already gives the
            # total, and the row has to fit the narrow panel without wrapping.
            self.list_mission.addItem(QListWidgetItem(f"{i + 1}. N{x:+.1f} E{y:+.1f}"))
        if 0 <= current < self.list_mission.count():
            self.list_mission.setCurrentRow(current)
        # Height follows the number of stops (2..6 rows) instead of reserving a
        # tall empty box; beyond six it scrolls.
        row_h = max(self.list_mission.sizeHintForRow(0), px(18)) if self.list_mission.count() else px(18)
        rows = max(2, min(6, self.list_mission.count()))
        self.list_mission.setFixedHeight(rows * row_h + 2 * self.list_mission.frameWidth() + px(6))
        if legs and len(legs) == len(stops):
            total = sum(l["distance_m"] for l in legs)
            t = sum(l.get("est_flight_time_s", 0.0) for l in legs)
            self.lbl_mission_stats.setText(
                f"{len(stops)} stops  •  {total:.2f} m  •  ~{t:.0f} s")
        elif legs:
            self.lbl_mission_stats.setText(
                f"{len(stops)} stops  •  blocked at stop {len(legs) + 1}")
        else:
            self.lbl_mission_stats.setText(f"{len(stops)} stops  •  planning…")

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
            # Grey-first: a passing or unjudged number stays neutral; only a
            # value that FAILS its threshold takes the caution colour.
            cell.setStyleSheet(f"color: {PALETTE['warn']};" if metric.good is False else "")
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

    # Horizontal (padding + border) budget per stylesheet role, read off the
    # matching rule in ui/styles.py. Used only as a floor for setMinimumWidth
    # below - a few px of slack here just makes the control a touch wider
    # than strictly needed, never wrong.
    _CONTROL_H_PADDING = {
        "mapTool": 20, "segItem": 22, "mapDanger": 20, "mapPill": 22,
        "btnGo": 24, "btnHold": 24, "btnAbort": 24,
    }

    def _fit_button_to_text(self, widget, extra_labels: tuple = ()) -> None:
        """Floor a button/label's width at what its text(s) actually need.

        QHBoxLayout treats sizeHint() as a preference, not a floor: under
        space pressure (a narrow window, a large UI scale) it compresses a
        control below the width its text needs, and Qt then clips the label
        from both sides with no ellipsis - see
        tests/test_gui_layout.py's CLIPPED check, and the real EXECUTE/PAUSE/
        ABORT/Follow/Center/Measure clipping this fixes. setMinimumWidth is
        the actual floor, so measure the real text in the real (post-
        stylesheet) font rather than trusting sizeHint().

        `extra_labels` covers controls whose text changes at runtime (Follow
        -> "Follow: ON", PAUSE -> RESUME, the map pill's LIVE/BENCH/NO DATA/...
        states): the floor is set from the widest of every label it will ever
        show, not just its label today.
        """
        widget.ensurePolished()
        h_pad = px(self._CONTROL_H_PADDING.get(widget.objectName(), 28))
        fm = widget.fontMetrics()
        widest = max((fm.horizontalAdvance(t) for t in (widget.text(),) + tuple(extra_labels)), default=0)
        widget.setMinimumWidth(widest + h_pad)

    def _map_tool(self, text: str, tooltip: str = "",
                  checkable: bool = False, extra_labels: tuple = ()) -> QPushButton:
        """A neutral map tool: view, zoom, rotate. Never commands the aircraft."""
        btn = QPushButton(text, self)
        btn.setObjectName("mapTool")
        if tooltip:
            btn.setToolTip(tooltip)
        btn.setCheckable(checkable)
        self._fit_button_to_text(btn, extra_labels)
        return btn

    def _seg_button(self, text: str, tooltip: str = "") -> QPushButton:
        """One cell of a segmented control (view mode, map layer)."""
        btn = QPushButton(text, self)
        btn.setObjectName("segItem")
        btn.setCheckable(True)
        if tooltip:
            btn.setToolTip(tooltip)
        self._fit_button_to_text(btn)
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

        act_stop = QAction("Add mission stop here", menu)
        act_stop.setToolTip("Append this point to the route (same as Shift+click)")
        act_stop.triggered.connect(lambda: self.canvas.add_mission_stop(wx, wy))
        menu.addAction(act_stop)

        if self.canvas.mission_stops:
            act_pop = QAction("Remove last stop", menu)
            act_pop.setToolTip("Backspace does the same on the map")
            act_pop.triggered.connect(lambda: self.canvas.remove_mission_stop(
                len(self.canvas.mission_stops) - 1))
            menu.addAction(act_pop)

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
        if self.canvas.zone_at(wx, wy):
            act_rm_zone = QAction("Remove this keep-out zone", menu)
            act_rm_zone.triggered.connect(lambda: self.canvas.remove_keepout_at(wx, wy))
            menu.addAction(act_rm_zone)
        act_clear_zones = QAction("Clear all keep-out zones", menu)
        act_clear_zones.setEnabled(bool(self.canvas.keepout_zones))
        act_clear_zones.triggered.connect(self.canvas.clear_keepouts)
        menu.addAction(act_clear_zones)

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
        if checked and getattr(self, "btn_keepout", None) is not None and self.btn_keepout.isChecked():
            self.btn_keepout.setChecked(False)
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
        # A deliberate reset is not a stalled map: restart the staleness clock.
        self.canvas.last_map_time = 0.0
        self.canvas.trail.clear()
        self.canvas.trail_visible = False
        self.canvas.clear_goal()
        self.canvas._auto_fit_done = False
        self._has_raw_map = False
        self._has_thin_map = False
        self._map_ever_seen = False       # a deliberate reset is not a lost map
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
            self.request_inflation()

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
        n_stops = len(self.canvas.mission_stops)
        if waypoints and n_stops > 1:
            self.lbl_path_info.setText(
                f"MISSION  {n_stops} stops\n{dist:.2f} m  \u2022  {len(waypoints)} WP  \u2022  ~{est_time:.0f} s")
            self.lbl_path_info.setToolTip("Planned obstacle-aware route through every stop")
            self._set_state(self.lbl_path_info, "ok")
            self.btn_execute_path.setEnabled(True)
        elif waypoints:
            self.lbl_path_info.setText(
                f"GOAL  N {gx:+.2f}  E {gy:+.2f}\n{dist:.2f} m  \u2022  {len(waypoints)} WP  \u2022  ~{est_time:.0f} s")
            self.lbl_path_info.setToolTip(
                "Planned obstacle-aware path to the staged goal")
            self._set_state(self.lbl_path_info, "ok")
            self.btn_execute_path.setEnabled(True)
        else:
            self.lbl_path_info.setText(
                f"BLOCKED  N {gx:+.2f}  E {gy:+.2f}\nno clear path")
            self.lbl_path_info.setToolTip(
                "The A* planner found no obstacle-free route to this point")
            self._set_state(self.lbl_path_info, "blocked")
            self.btn_execute_path.setEnabled(False)
        self.lbl_path_info.setVisible(True)
        self.lbl_route_idle.setVisible(False)

    def _on_goal_cleared(self):
        self.lbl_route_idle.setVisible(True)
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
        # Callers pass either spelling (mission_control uses paused=True); the
        # branches below used to test only is_paused, so a paused mission never
        # showed RESUME.
        is_paused = bool(is_paused or paused)
        self.is_path_paused = is_paused

        def set_role(button, role):
            if button.objectName() != role:
                button.setObjectName(role)
                button.style().unpolish(button)
                button.style().polish(button)

        self._apply_action_visibility(running=bool(is_running or is_paused or paused))
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
