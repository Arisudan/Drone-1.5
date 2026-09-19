"""Tests for scripts/gcs/core/path_planner.py - A* over the live occupancy grid.

Needs numpy only. This planner is what click-to-navigate flies, so the cases
that matter are the ones where it must REFUSE: a sealed wall, a goal inside an
obstacle, an unknown corridor when unknown space is being treated as solid.

Grid convention (same as the GCS canvas feeds it): 2-D array indexed
[row=X/North, col=Y/East]; >= 50 occupied, -1 unknown, < 50 free.
"""

import math
import unittest

import numpy as np

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core.path_planner import AStarPathPlanner

RES = 0.1          # 10 cm cells
DIM = 40           # 4 m x 4 m arena
START = (0.25, 0.25)
GOAL = (3.55, 3.55)


def empty_grid(value: int = 0) -> np.ndarray:
    return np.full((DIM, DIM), value, dtype=np.int8)


def planner(radius_m: float = 0.05, **kw) -> AStarPathPlanner:
    # Radius kept below one cell so inflation is a predictable 1 cell and the
    # tests exercise the search, not the inflation arithmetic.
    return AStarPathPlanner(robot_radius_m=radius_m, **kw)


def plan(grid, p=None, start=START, goal=GOAL, **kw):
    p = p or planner()
    return p.plan(grid, RES, 0.0, 0.0, start, goal, **kw)


class BasicsTest(unittest.TestCase):
    def test_open_arena_gives_a_near_straight_path(self):
        res = plan(empty_grid())
        self.assertTrue(res["success"], res["message"])
        self.assertGreaterEqual(len(res["waypoints"]), 1)
        straight = math.dist(START, GOAL)
        # Line-of-sight smoothing should land within ~15% of the straight line.
        self.assertLess(res["total_distance_m"], straight * 1.15)
        self.assertGreater(res["total_distance_m"], straight * 0.80)

    def test_final_waypoint_reaches_the_goal(self):
        res = plan(empty_grid())
        last = res["waypoints"][-1]
        self.assertLess(math.dist(last, GOAL), 0.30)

    def test_eta_is_distance_at_half_a_metre_per_second_plus_per_waypoint_settle(self):
        res = plan(empty_grid())
        # 0.5 m/s cruise, plus 1 s of deceleration allowance per waypoint.
        expected = res["total_distance_m"] / 0.5 + len(res["waypoints"]) * 1.0
        self.assertAlmostEqual(res["est_flight_time_s"], expected, delta=0.01)

    def test_same_start_and_goal_is_a_no_op_not_a_failure(self):
        res = plan(empty_grid(), start=START, goal=START)
        self.assertTrue(res["success"])
        self.assertEqual(res["total_distance_m"], 0.0)

    def test_degenerate_grid_is_rejected_not_crashed(self):
        res = plan(np.zeros((2, 2), dtype=np.int8))
        self.assertFalse(res["success"])
        self.assertIn("Invalid", res["message"])

    def test_zero_resolution_is_rejected(self):
        p = planner()
        res = p.plan(empty_grid(), 0.0, 0.0, 0.0, START, GOAL)
        self.assertFalse(res["success"])


class ObstacleTest(unittest.TestCase):
    def test_routes_through_a_gap_in_a_wall(self):
        grid = empty_grid()
        grid[20, :] = 100          # wall across the arena
        grid[20, 2:7] = 0          # 50 cm doorway, deliberately off the
                                   # start-goal diagonal so reaching it is a
                                   # real detour rather than the direct line
        res = plan(grid)
        self.assertTrue(res["success"], res["message"])
        self.assertGreater(res["total_distance_m"], math.dist(START, GOAL) * 1.05)

    def test_path_never_crosses_an_occupied_cell(self):
        grid = empty_grid()
        grid[20, :] = 100
        grid[20, 2:7] = 0
        res = plan(grid)
        self.assertTrue(res["success"])

        pts = [START] + list(res["waypoints"])
        for a, b in zip(pts, pts[1:]):
            steps = max(2, int(math.dist(a, b) / (RES / 2)))
            for i in range(steps + 1):
                x = a[0] + (b[0] - a[0]) * i / steps
                y = a[1] + (b[1] - a[1]) * i / steps
                r = min(DIM - 1, max(0, int(x / RES)))
                c = min(DIM - 1, max(0, int(y / RES)))
                self.assertLess(grid[r, c], 50,
                                f"path enters obstacle cell ({r}, {c})")

    def test_sealed_wall_fails_instead_of_inventing_a_path(self):
        grid = empty_grid()
        grid[20, :] = 100
        res = plan(grid)
        self.assertFalse(res["success"])

    def test_goal_inside_a_wall_is_snapped_or_refused_never_flown_into(self):
        """Regression: the endpoint must never end up inside the clicked wall.

        plan() used to unconditionally overwrite the last waypoint with the
        operator's raw goal ("always ensure exact goal pose is final
        waypoint"), which undid the snap-out-of-obstacle logic and produced a
        path whose final OFFBOARD setpoint sat inside the obstacle.
        """
        grid = empty_grid()
        grid[30:36, 30:36] = 100          # solid block containing the goal
        res = plan(grid)
        if res["success"]:
            gx, gy = res["waypoints"][-1]
            self.assertLess(grid[int(gx / RES), int(gy / RES)], 50,
                            "final waypoint is inside an obstacle")
            self.assertTrue(res.get("goal_adjusted"),
                            "a moved goal must be reported to the operator")
        else:
            self.assertIn("obstacle", res["message"].lower())

    def test_reachable_goal_is_hit_exactly_and_not_flagged_as_adjusted(self):
        res = plan(empty_grid())
        self.assertFalse(res.get("goal_adjusted", False))
        self.assertAlmostEqual(res["waypoints"][-1][0], GOAL[0], places=6)
        self.assertAlmostEqual(res["waypoints"][-1][1], GOAL[1], places=6)

    def test_start_inside_a_wall_is_snapped_out(self):
        grid = empty_grid()
        grid[0:6, 0:6] = 100              # the drone's own cell is occupied
        res = plan(grid)
        # Either it escapes or it says so - silently planning from inside a
        # wall would hand the operator a path that starts through concrete.
        if not res["success"]:
            self.assertIn("trapped", res["message"].lower())

    def test_inflation_keeps_clearance_from_walls(self):
        # A one-cell doorway is narrower than the inflated footprint of a
        # 25 cm drone at 10 cm cells, so it must not be used.
        grid = empty_grid()
        grid[20, :] = 100
        grid[20, 20] = 0
        res = plan(grid, p=planner(radius_m=0.25))
        self.assertFalse(res["success"])


class UnknownSpaceTest(unittest.TestCase):
    def test_unknown_is_traversable_by_default(self):
        grid = empty_grid(-1)
        res = plan(grid)
        self.assertTrue(res["success"], res["message"])

    def test_unknown_blocks_when_asked_to_treat_it_as_solid(self):
        grid = empty_grid()
        grid[20, :] = -1               # unmapped band across the arena
        self.assertTrue(plan(grid)["success"])
        blocked = plan(grid, treat_unknown_as_obstacle=True)
        self.assertFalse(blocked["success"])

    def test_constructor_default_can_be_overridden_per_call(self):
        grid = empty_grid()
        grid[20, :] = -1
        strict = planner(treat_unknown_as_obstacle=True)
        self.assertFalse(plan(grid, p=strict)["success"])
        self.assertTrue(plan(grid, p=strict,
                             treat_unknown_as_obstacle=False)["success"])


class CollisionCheckTest(unittest.TestCase):
    def test_clear_path_reports_no_collision(self):
        p = planner()
        blocked, point, dist = p.check_path_collision(
            empty_grid(), RES, 0.0, 0.0, [(2.0, 0.25)], (0.25, 0.25))
        self.assertFalse(blocked)
        self.assertIsNone(point)

    def test_obstacle_appearing_on_the_path_is_caught(self):
        grid = empty_grid()
        grid[10, 0:10] = 100           # new wall 1 m ahead
        p = planner()
        blocked, point, dist = p.check_path_collision(
            grid, RES, 0.0, 0.0, [(3.0, 0.25)], (0.25, 0.25), lookahead_m=2.0)
        self.assertTrue(blocked)
        self.assertIsNotNone(point)
        self.assertGreater(dist, 0.0)
        self.assertLessEqual(dist, 2.0)

    def test_obstacle_beyond_the_lookahead_is_ignored(self):
        grid = empty_grid()
        grid[35, 0:10] = 100           # 3.5 m ahead
        p = planner()
        blocked, _pt, _d = p.check_path_collision(
            grid, RES, 0.0, 0.0, [(3.9, 0.25)], (0.25, 0.25), lookahead_m=0.5)
        self.assertFalse(blocked)

    def test_empty_waypoints_is_not_a_collision(self):
        p = planner()
        blocked, _pt, _d = p.check_path_collision(
            empty_grid(), RES, 0.0, 0.0, [], (0.25, 0.25))
        self.assertFalse(blocked)


class OriginTest(unittest.TestCase):
    def test_negative_origin_is_handled(self):
        # RTAB-Map grids routinely have a negative origin; floor division on
        # negative coordinates is the classic off-by-one here.
        grid = empty_grid()
        p = planner()
        res = p.plan(grid, RES, -2.0, -2.0, (-1.75, -1.75), (1.55, 1.55))
        self.assertTrue(res["success"], res["message"])
        last = res["waypoints"][-1]
        self.assertLess(math.dist(last, (1.55, 1.55)), 0.30)


if __name__ == "__main__":
    unittest.main()
