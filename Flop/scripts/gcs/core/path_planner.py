"""
================================================================================
MODULE: path_planner.py
PURPOSE: Obstacle-Aware 2D A* Path Planner & Line-of-Sight Trajectory Smoother
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station or Radxa Companion Computer
  * Communicates:  SlamMapWidget, DroneGCS MainWindow, and MAVLink Offboard Worker
  * Upstream:      Live Occupancy Grid (/map_thin or /map) & Target Waypoint Click
  * Downstream:    GCS Map Visualizer (green trajectory overlay) & Offboard Waypoints

DATA FLOW & INTERFACES:
  * Inputs:        Occupancy grid array (int8 HxW), map resolution (m/cell),
                   map origin (x, y), start world coordinate (x, y), goal coordinate.
  * Outputs:       Dict containing status ("OK", "NO_PATH", "START_IN_COLLISION"),
                   detailed path list [[x0, y0], [x1, y1]...], simplified waypoints,
                   total path length in meters, and computation latency in ms.

KEY LOGIC & FAILSAFES:
  * Pure NumPy Obstacle Inflation: Dilates obstacles by configurable vehicle radius
    (default 0.25m / 5 cells at 0.05m res) without requiring heavy external dependencies.
  * Corner-Cutting Prevention: Enforces diagonal traversal checks to ensure the
    airframe never clips sharp obstacle corners.
  * Bresenham Line-of-Sight Smoothing: Collapses redundant collinear cells into
    sparse, flyable waypoint vectors with direct line-of-sight clearance.
  * Unreachable Goal Fallback: Detects goals placed inside obstacles or walls and
    locates the nearest accessible free cell within search radius.

USAGE:
  planner = AStarPathPlanner(robot_radius_m=0.25)
  res = planner.plan(grid_array, resolution=0.05, origin_x=-10.0, origin_y=-10.0,
                     start_world=(0.0, 0.0), goal_world=(2.5, 1.2))
  if res["status"] == "OK":
      print(f"Path planned: {len(res['waypoints'])} waypoints, length: {res['length_m']:.2f}m")
================================================================================
"""

from __future__ import annotations
import heapq
import math
from typing import List, Tuple, Optional, Dict, Any
import numpy as np

from core.map_quality import OCC_THRESH

def _inflate_obstacles_numpy(obstacle_mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Dilate obstacle mask by circular disk in pure NumPy (zero external dependencies)."""
    if radius_cells <= 0:
        return obstacle_mask.astype(np.uint8)
    h, w = obstacle_mask.shape
    inflated = obstacle_mask.copy()
    r_sq = radius_cells * radius_cells
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr * dr + dc * dc <= r_sq:
                r_s_src = max(0, -dr)
                r_e_src = min(h, h - dr)
                c_s_src = max(0, -dc)
                c_e_src = min(w, w - dc)

                r_s_dst = max(0, dr)
                r_e_dst = min(h, h + dr)
                c_s_dst = max(0, dc)
                c_e_dst = min(w, w + dc)

                inflated[r_s_dst:r_e_dst, c_s_dst:c_e_dst] |= obstacle_mask[r_s_src:r_e_src, c_s_src:c_e_src]
    return inflated.astype(np.uint8)


def rasterise_zones(zones, shape: Tuple[int, int], resolution: float,
                    origin_x: float, origin_y: float) -> np.ndarray:
    """Boolean mask of the grid cells whose centre lies inside any zone.

    ``zones`` is a sequence of polygons, each a sequence of (x, y) world points
    in the planner's frame (x = North, y = East, metres). A cell belongs to a
    zone when its centre does, using even-odd ray casting, so rectangles,
    L-shapes and any other simple polygon behave the same way.

    Only each polygon's bounding box is tested, not the whole grid: a keep-out
    zone is typically a few square metres of a map that can be 1000 x 1000
    cells, and testing every cell for every zone would cost more than the A*
    search it is feeding.
    """
    h, w = shape
    mask = np.zeros((h, w), dtype=bool)
    if resolution <= 0:
        return mask
    for zone in zones or ():
        pts = [(float(x), float(y)) for x, y in zone]
        if len(pts) < 3:
            continue
        xs = [q[0] for q in pts]
        ys = [q[1] for q in pts]
        r0 = max(0, int(math.floor((min(xs) - origin_x) / resolution)))
        r1 = min(h - 1, int(math.floor((max(xs) - origin_x) / resolution)))
        c0 = max(0, int(math.floor((min(ys) - origin_y) / resolution)))
        c1 = min(w - 1, int(math.floor((max(ys) - origin_y) / resolution)))
        if r1 < r0 or c1 < c0:
            continue                      # zone lies wholly off this map
        rr, cc = np.mgrid[r0:r1 + 1, c0:c1 + 1]
        px = origin_x + (rr + 0.5) * resolution
        py = origin_y + (cc + 0.5) * resolution
        inside = np.zeros(px.shape, dtype=bool)
        n = len(pts)
        for i in range(n):
            xi, yi = pts[i]
            xj, yj = pts[(i + 1) % n]
            if yi == yj:
                continue                  # horizontal edge never crosses the ray
            crosses = ((yi > py) != (yj > py)) & (
                px < (xj - xi) * (py - yi) / (yj - yi) + xi)
            inside ^= crosses
        mask[r0:r1 + 1, c0:c1 + 1] |= inside
    return mask


class AStarPathPlanner:
    """
    2D Grid Path Planner with obstacle inflation, corner-cutting prevention,
    and line-of-sight waypoint extraction for autonomous indoor drone navigation.
    """

    def __init__(self, robot_radius_m: float = 0.25, treat_unknown_as_obstacle: bool = False):
        self.robot_radius_m = robot_radius_m
        self.treat_unknown_as_obstacle = treat_unknown_as_obstacle
        # Operator-drawn keep-out zones: a tuple of polygons in world (x, y).
        # Replaced whole, never mutated in place, because this planner is read
        # from the planner thread while the GUI thread edits zones - swapping
        # one immutable reference is atomic, editing a shared list is not.
        self.keepout_zones: tuple = ()

    # ── obstacle model ──────────────────────────────────────────────

    def set_keepout_zones(self, zones) -> None:
        """Replace every keep-out zone at once (see keepout_zones)."""
        self.keepout_zones = tuple(tuple((float(x), float(y)) for x, y in z)
                                   for z in (zones or ()))

    def _build_masks(self, grid: np.ndarray, resolution: float, origin_x: float,
                     origin_y: float, block_unknown: bool):
        """-> (keep-out mask, inflated obstacle mask) for this grid.

        Keep-out cells are added to the obstacles BEFORE inflation, so the
        vehicle keeps one robot radius clear of a zone's edge exactly as it
        does from a wall. Inflating walls and then pasting zones on top would
        let a route skim a zone boundary with no margin at all.

        plan() and check_path_collision() both come through here. They used to
        each build the mask inline, and a keep-out zone that only one of them
        honoured would mean a route planned around a zone and an in-flight
        collision check that did not know it was there.
        """
        keepout = rasterise_zones(self.keepout_zones, grid.shape, resolution,
                                  origin_x, origin_y)
        obstacle_mask = (grid >= OCC_THRESH) | ((grid < 0) if block_unknown else False)
        obstacle_mask = obstacle_mask | keepout
        inflation_cells = max(1, int(math.ceil(self.robot_radius_m / resolution)))
        return keepout, _inflate_obstacles_numpy(obstacle_mask, inflation_cells)

    def inflated_mask(self, grid: np.ndarray, resolution: float, origin_x: float,
                      origin_y: float, treat_unknown_as_obstacle: Optional[bool] = None):
        """-> (obstacle mask incl. keep-out, inflated mask), for display.

        The tactical map draws exactly this - the same mask the search uses -
        rather than a separately computed look-alike, so what the operator sees
        as "the vehicle cannot go here" is what the planner acts on.
        """
        block_unknown = (self.treat_unknown_as_obstacle
                         if treat_unknown_as_obstacle is None
                         else treat_unknown_as_obstacle)
        keepout, inflated = self._build_masks(grid, resolution, origin_x,
                                              origin_y, block_unknown)
        raw = (grid >= OCC_THRESH) | keepout
        return raw, inflated

    def plan(
        self,
        grid: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        treat_unknown_as_obstacle: Optional[bool] = None,
        _masks=None,
    ) -> Dict[str, Any]:
        """
        Plan shortest collision-free path from start_world to goal_world.

        ``_masks`` is private: plan_route() builds the masks once and hands them
        to every leg, instead of re-inflating the whole map per stop.

        Args:
            grid: 2D numpy array (H, W) where >=OCC_THRESH is obstacle, -1 unknown, else free.
            resolution: meters per cell.
            origin_x: world X (North) of grid[0, 0].
            origin_y: world Y (East) of grid[0, 0].
            start_world: (x, y) start pose in meters.
            goal_world: (x, y) target goal pose in meters.
            treat_unknown_as_obstacle: If True, treats unmapped/unknown (-1) cells as obstacles.

        Returns:
            Dict containing:
                - success (bool)
                - waypoints (list of (x, y) metric world coords)
                - total_distance_m (float)
                - est_flight_time_s (float at 0.5 m/s)
                - traverses_unknown (bool)
                - message (str)
        """
        h, w = grid.shape
        if h < 3 or w < 3 or resolution <= 0:
            return {
                "success": False,
                "waypoints": [],
                "total_distance_m": 0.0,
                "est_flight_time_s": 0.0,
                "traverses_unknown": False,
                "message": "Invalid occupancy grid dimensions or resolution.",
            }

        block_unknown = self.treat_unknown_as_obstacle if treat_unknown_as_obstacle is None else treat_unknown_as_obstacle

        # 1. Obstacles (occupied, unknown if asked, keep-out zones), inflated.
        if _masks is None:
            _masks = self._build_masks(grid, resolution, origin_x, origin_y, block_unknown)
        keepout, inflated_obstacles = _masks

        # 2. Convert World Coordinates to Grid Indices using math.floor for negative coordinates
        def world_to_grid(x: float, y: float) -> Tuple[int, int]:
            r = int(math.floor((x - origin_x) / resolution))
            c = int(math.floor((y - origin_y) / resolution))
            return r, c

        def grid_to_world(r: int, c: int) -> Tuple[float, float]:
            x = origin_x + (r + 0.5) * resolution
            y = origin_y + (c + 0.5) * resolution
            return x, y

        start_r, start_c = world_to_grid(start_world[0], start_world[1])
        goal_r, goal_c = world_to_grid(goal_world[0], goal_world[1])

        # A goal outside the grid is outside the SLAM data altogether. It used
        # to be clamped to the map edge for the search and then written back as
        # the final waypoint (step 6), so the last leg flew a straight line from
        # the edge of the map to the clicked point - through space no sensor had
        # seen, with no A* and no collision check. Refuse it instead. The start
        # is still clamped: the vehicle is where it is, and a pose a cell or two
        # off a freshly started map must not strand it.
        if not (0 <= goal_r < h and 0 <= goal_c < w):
            return {
                "success": False,
                "waypoints": [],
                "total_distance_m": 0.0,
                "est_flight_time_s": 0.0,
                "message": "Goal is outside the mapped area.",
            }

        # Clamp start & goal within bounds
        start_r = max(0, min(h - 1, start_r))
        start_c = max(0, min(w - 1, start_c))
        goal_r = max(0, min(h - 1, goal_r))
        goal_c = max(0, min(w - 1, goal_c))

        # A goal the operator placed inside a keep-out zone is refused outright,
        # not snapped to the nearest free cell the way a goal inside a wall is.
        # A wall-snap corrects imprecise clicking; moving a goal out of a zone
        # the operator drew would quietly overrule an explicit instruction.
        if keepout[goal_r, goal_c]:
            return {
                "success": False,
                "waypoints": [],
                "total_distance_m": 0.0,
                "est_flight_time_s": 0.0,
                "message": "Goal is inside a keep-out zone.",
            }
        if keepout[start_r, start_c]:
            return {
                "success": False,
                "waypoints": [],
                "total_distance_m": 0.0,
                "est_flight_time_s": 0.0,
                "message": "Vehicle is inside a keep-out zone - fly it out or remove the zone.",
            }

        # Check if start is inside obstacle; if so, snap to nearest free cell
        if inflated_obstacles[start_r, start_c] > 0:
            snapped = self._find_nearest_free_cell(inflated_obstacles, start_r, start_c, max_radius=15)
            if snapped:
                start_r, start_c = snapped
            else:
                return {
                    "success": False,
                    "waypoints": [],
                    "total_distance_m": 0.0,
                    "est_flight_time_s": 0.0,
                    "message": "Drone is trapped inside an obstacle zone.",
                }

        # Check if goal is inside obstacle; snap to nearest reachable free cell
        goal_was_snapped = False
        if inflated_obstacles[goal_r, goal_c] > 0:
            snapped = self._find_nearest_free_cell(inflated_obstacles, goal_r, goal_c, max_radius=20)
            if snapped:
                goal_r, goal_c = snapped
                goal_was_snapped = True
            else:
                return {
                    "success": False,
                    "waypoints": [],
                    "total_distance_m": 0.0,
                    "est_flight_time_s": 0.0,
                    "message": "Goal pose is inside a wall or obstacle.",
                }

        if (start_r, start_c) == (goal_r, goal_c):
            return {
                "success": True,
                "waypoints": [goal_world],
                "total_distance_m": 0.0,
                "est_flight_time_s": 0.0,
                "message": "Start and Goal are the same position.",
            }

        # 3. A* Search on 8-connected grid
        # Neighbors: (dr, dc, move_cost)
        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)
        ]

        def heuristic(r1: int, c1: int, r2: int, c2: int) -> float:
            # Octile distance heuristic
            dr = abs(r1 - r2)
            dc = abs(c1 - c2)
            return (dr + dc) + (1.4142 - 2.0) * min(dr, dc)

        open_set: List[Tuple[float, int, Tuple[int, int]]] = []
        counter = 0
        heapq.heappush(open_set, (heuristic(start_r, start_c, goal_r, goal_c), counter, (start_r, start_c)))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {(start_r, start_c): 0.0}
        closed_set = set()

        max_iterations = 25000
        iterations = 0
        found = False

        while open_set and iterations < max_iterations:
            iterations += 1
            _, _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue
            closed_set.add(current)

            cr, cc = current
            if (cr, cc) == (goal_r, goal_c):
                found = True
                break

            current_g = g_score[current]

            for dr, dc, cost in neighbors:
                nr, nc = cr + dr, cc + dc
                if not (0 <= nr < h and 0 <= nc < w):
                    continue

                if inflated_obstacles[nr, nc] > 0:
                    continue

                # Prevent cutting through diagonal obstacle corners
                if dr != 0 and dc != 0:
                    if inflated_obstacles[cr + dr, cc] > 0 or inflated_obstacles[cr, cc + dc] > 0:
                        continue

                neighbor = (nr, nc)
                if neighbor in closed_set:
                    continue

                tentative_g = current_g + cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(nr, nc, goal_r, goal_c)
                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))

        if not found:
            return {
                "success": False,
                "waypoints": [],
                "total_distance_m": 0.0,
                "est_flight_time_s": 0.0,
                "message": "No collision-free path found. Target is blocked by obstacles.",
            }

        # 4. Reconstruct Dense Grid Path
        curr = (goal_r, goal_c)
        dense_path = [curr]
        while curr in came_from:
            curr = came_from[curr]
            dense_path.append(curr)
        dense_path.reverse()

        # 5. Line-of-Sight Waypoint Smoothing (Bresenham Raycast)
        smoothed_cells = self._smooth_path(dense_path, inflated_obstacles)

        # 6. Convert Smoothed Cells to Metric Coordinates
        waypoints: List[Tuple[float, float]] = []
        for r, c in smoothed_cells:
            waypoints.append(grid_to_world(r, c))

        # Snap the final waypoint to the exact requested goal - but ONLY when
        # the goal was actually reachable. If it had to be moved out of an
        # obstacle above, overwriting the endpoint here would hand the
        # operator a path whose last setpoint sits inside the wall they
        # clicked on, which is precisely the case the snap existed to avoid.
        if waypoints and not goal_was_snapped:
            waypoints[-1] = (goal_world[0], goal_world[1])

        # Compute total distance
        total_dist = 0.0
        for i in range(len(waypoints) - 1):
            dx = waypoints[i + 1][0] - waypoints[i][0]
            dy = waypoints[i + 1][1] - waypoints[i][1]
            total_dist += math.sqrt(dx * dx + dy * dy)

        # Estimate flight time at nominal 0.5 m/s plus 1s per waypoint deceleration
        est_time = (total_dist / 0.5) + (len(waypoints) * 1.0)
        # Check if any part of the path traverses unmapped space
        traverses_unknown = any(grid[r, c] < 0 for r, c in dense_path if 0 <= r < h and 0 <= c < w)

        return {
            "success": True,
            "waypoints": waypoints,
            "dense_path_cells": dense_path,
            "total_distance_m": total_dist,
            "est_flight_time_s": est_time,
            "traverses_unknown": traverses_unknown,
            "goal_adjusted": goal_was_snapped,
            "message": f"Planned collision-free path: {len(waypoints)} waypoints, {total_dist:.2f}m"
            + (" (Traverses unmapped space)" if traverses_unknown else "")
            + (" (Goal moved clear of an obstacle)" if goal_was_snapped else ""),
        }

    def plan_route(
        self,
        grid: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        start_world: Tuple[float, float],
        stops: List[Tuple[float, float]],
        treat_unknown_as_obstacle: Optional[bool] = None,
    ) -> Dict[str, Any]:
        """Plan a multi-stop mission: start -> stop 1 -> stop 2 -> ... -> last.

        Each leg is an independent A* search from where the previous leg really
        ended (a goal snapped off a wall ends slightly off the click), and the
        legs are joined into one waypoint list, so the flight side executes a
        mission exactly as it executes a single goal.

        Returns plan()'s dict plus ``legs`` (per-leg distance and waypoints) and
        ``stop_indices`` (the index in ``waypoints`` where each stop is reached).
        If any leg fails the whole route fails and ``failed_leg`` says which -
        flying the reachable half of a mission is not what was asked for.
        """
        base = {"success": False, "waypoints": [], "total_distance_m": 0.0,
                "est_flight_time_s": 0.0, "traverses_unknown": False,
                "legs": [], "stop_indices": [], "message": ""}
        if not stops:
            base["message"] = "No stops in mission."
            return base
        if grid is None or grid.ndim != 2 or grid.shape[0] < 3 or grid.shape[1] < 3 or resolution <= 0:
            base["message"] = "Invalid occupancy grid dimensions or resolution."
            return base

        block_unknown = (self.treat_unknown_as_obstacle
                         if treat_unknown_as_obstacle is None
                         else treat_unknown_as_obstacle)
        masks = self._build_masks(grid, resolution, origin_x, origin_y, block_unknown)

        waypoints: List[Tuple[float, float]] = []
        legs = []
        stop_indices = []
        total_d = total_t = 0.0
        unknown = False
        here = start_world
        for i, stop in enumerate(stops):
            leg = self.plan(grid, resolution, origin_x, origin_y, here, stop,
                            treat_unknown_as_obstacle=block_unknown, _masks=masks)
            if not leg.get("success"):
                out = dict(base, legs=legs, stop_indices=stop_indices,
                           waypoints=waypoints, failed_leg=i)
                out["message"] = f"Stop {i + 1}: {leg.get('message', 'no path')}"
                return out
            leg_wps = list(leg["waypoints"])
            waypoints.extend(leg_wps)
            stop_indices.append(len(waypoints) - 1)
            legs.append({"stop": i, "distance_m": leg["total_distance_m"],
                         "est_flight_time_s": leg.get("est_flight_time_s", 0.0),
                         "waypoints": leg_wps})
            total_d += leg["total_distance_m"]
            total_t += leg.get("est_flight_time_s", 0.0)
            unknown = unknown or bool(leg.get("traverses_unknown"))
            here = leg_wps[-1] if leg_wps else stop

        return {"success": True, "waypoints": waypoints,
                "total_distance_m": total_d, "est_flight_time_s": total_t,
                "traverses_unknown": unknown, "legs": legs,
                "stop_indices": stop_indices,
                "message": f"Mission: {len(stops)} stops, {total_d:.2f} m"}

    def check_path_collision(
        self,
        grid: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        waypoints: List[Tuple[float, float]],
        current_pos: Tuple[float, float],
        lookahead_m: float = 1.5,
        treat_unknown_as_obstacle: Optional[bool] = None,
    ) -> Tuple[bool, Optional[Tuple[float, float]], float]:
        """
        Dynamically check if remaining flight path is clear of obstacles within lookahead_m distance.
        Samples the trajectory path at fine spatial intervals and checks against inflated obstacles.

        Returns:
            (is_blocked, collision_point_world, distance_to_collision_m)
        """
        if not waypoints or grid is None or grid.shape[0] < 3 or grid.shape[1] < 3 or resolution <= 0:
            return False, None, 0.0

        h, w = grid.shape
        block_unknown = self.treat_unknown_as_obstacle if treat_unknown_as_obstacle is None else treat_unknown_as_obstacle
        _keepout, inflated_obstacles = self._build_masks(
            grid, resolution, origin_x, origin_y, block_unknown)

        def world_to_grid(x: float, y: float) -> Tuple[int, int]:
            r = int(math.floor((x - origin_x) / resolution))
            c = int(math.floor((y - origin_y) / resolution))
            return r, c

        # Traverse along line segments starting from current_pos to waypoints[0], waypoints[1], ...
        pts_chain = [current_pos] + list(waypoints)
        accumulated_dist = 0.0
        step_m = max(0.02, resolution * 0.8)

        for i in range(len(pts_chain) - 1):
            p_start = pts_chain[i]
            p_end = pts_chain[i + 1]
            dx = p_end[0] - p_start[0]
            dy = p_end[1] - p_start[1]
            seg_len = math.sqrt(dx * dx + dy * dy)
            if seg_len < 1e-4:
                continue

            num_steps = max(1, int(math.ceil(seg_len / step_m)))
            for s in range(1, num_steps + 1):
                fraction = min(1.0, s / num_steps)
                step_dist = seg_len * (1.0 / num_steps)
                accumulated_dist += step_dist

                if accumulated_dist > lookahead_m:
                    return False, None, 0.0

                sx = p_start[0] + dx * fraction
                sy = p_start[1] + dy * fraction

                r, c = world_to_grid(sx, sy)
                if 0 <= r < h and 0 <= c < w:
                    if inflated_obstacles[r, c] > 0:
                        return True, (sx, sy), accumulated_dist

        return False, None, 0.0

    def _find_nearest_free_cell(
        self, obstacles: np.ndarray, r: int, c: int, max_radius: int = 15
    ) -> Optional[Tuple[int, int]]:
        """Find the closest non-obstacle cell using breadth-first spiral expansion."""
        h, w = obstacles.shape
        for radius in range(1, max_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in (-radius, radius):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and obstacles[nr, nc] == 0:
                        return nr, nc
            for dc in range(-radius + 1, radius):
                for dr in (-radius, radius):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and obstacles[nr, nc] == 0:
                        return nr, nc
        return None

    def _smooth_path(
        self, path: List[Tuple[int, int]], obstacles: np.ndarray
    ) -> List[Tuple[int, int]]:
        """Greedy line-of-sight shortcutting to extract sparse waypoints."""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        curr_idx = 0

        while curr_idx < len(path) - 1:
            next_idx = len(path) - 1
            while next_idx > curr_idx + 1:
                if self._line_of_sight(path[curr_idx], path[next_idx], obstacles):
                    break
                next_idx -= 1
            smoothed.append(path[next_idx])
            curr_idx = next_idx

        return smoothed

    def _line_of_sight(
        self, p1: Tuple[int, int], p2: Tuple[int, int], obstacles: np.ndarray
    ) -> bool:
        """Bresenham raycast line check between two cells."""
        r0, c0 = p1
        r1, c1 = p2
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc

        r, c = r0, c0
        while True:
            if obstacles[r, c] > 0:
                return False
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc
        return True
