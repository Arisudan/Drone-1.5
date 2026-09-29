"""Keep-out zones, multi-stop missions and the inflation layer.

Planner tests are hermetic (numpy only); the widget and worker tests need
PyQt5 and are skipped where it is absent.

What is being protected:
  * A keep-out zone is honoured by BOTH plan() and check_path_collision(). A
    zone only the route planner knew about would be flown around on the way
    out and then flown through by an in-flight detour.
  * Zones are inflated like walls, so a route keeps a robot radius clear.
  * A goal inside a zone is refused, not snapped out of it.
  * A goal off the map is refused. It used to be clamped for the search and
    then written back as the last waypoint - a straight, unchecked leg into
    space no sensor had seen.
  * A mission is planned leg by leg through every stop, and fails as a whole
    if any stop is unreachable.
"""

import time
import unittest

import numpy as np

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core.path_planner import AStarPathPlanner, rasterise_zones

RES = 0.025


def open_room(n=200):
    """A 5 x 5 m room of free space, origin (0, 0)."""
    return np.zeros((n, n), dtype=np.int8)


def _point_in_polygon(x, y, poly):
    """Local even-odd test, so the planner tests stay numpy-only (CI's
    hermetic job has no PyQt5, and the widget module imports it)."""
    inside = False
    n = len(poly)
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[(i + 1) % n]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
    return inside


def route_enters(waypoints, start, zone, samples=80):
    prev = start
    for wp in waypoints:
        for t in np.linspace(0, 1, samples):
            x = prev[0] + (wp[0] - prev[0]) * t
            y = prev[1] + (wp[1] - prev[1]) * t
            if _point_in_polygon(x, y, zone):
                return True
        prev = wp
    return False


BAR = [(2.3, 0.0), (2.7, 0.0), (2.7, 4.0), (2.3, 4.0)]   # across the room, gap at E > 4


class RasteriseTest(unittest.TestCase):
    def test_rectangle_covers_its_cells(self):
        m = rasterise_zones([[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)]],
                            (200, 200), RES, 0.0, 0.0)
        self.assertTrue(m[60, 60])            # (1.5, 1.5) m
        self.assertFalse(m[20, 20])           # (0.5, 0.5) m
        self.assertEqual(int(m.sum()), 40 * 40)

    def test_concave_polygon_excludes_its_notch(self):
        ell = [(0, 0), (3, 0), (3, 1), (1, 1), (1, 3), (0, 3)]
        m = rasterise_zones([ell], (200, 200), RES, 0.0, 0.0)
        self.assertTrue(m[20, 100])           # inside the vertical arm
        self.assertFalse(m[80, 80])           # (2, 2) m - in the notch

    def test_zone_off_the_map_is_ignored(self):
        m = rasterise_zones([[(50, 50), (51, 50), (51, 51), (50, 51)]],
                            (200, 200), RES, 0.0, 0.0)
        self.assertFalse(m.any())

    def test_degenerate_zone_is_ignored(self):
        self.assertFalse(rasterise_zones([[(0, 0), (1, 1)]], (50, 50), RES, 0, 0).any())


class KeepoutPlanningTest(unittest.TestCase):
    def setUp(self):
        self.p = AStarPathPlanner(robot_radius_m=0.25)

    def test_route_detours_around_a_zone(self):
        straight = self.p.plan(open_room(), RES, 0, 0, (0.5, 2.5), (4.5, 2.5))
        self.p.set_keepout_zones([BAR])
        r = self.p.plan(open_room(), RES, 0, 0, (0.5, 2.5), (4.5, 2.5))
        self.assertTrue(r["success"])
        self.assertGreater(r["total_distance_m"], straight["total_distance_m"] + 1.0)
        self.assertFalse(route_enters(r["waypoints"], (0.5, 2.5), BAR))

    def test_zone_is_inflated_like_a_wall(self):
        self.p.set_keepout_zones([BAR])
        r = self.p.plan(open_room(), RES, 0, 0, (0.5, 2.5), (4.5, 2.5))
        east = max(wp[1] for wp in r["waypoints"])
        self.assertGreaterEqual(east, 4.0 + 0.25 - RES,
                                "route must clear the zone edge by the robot radius")

    def test_goal_inside_a_zone_is_refused(self):
        self.p.set_keepout_zones([BAR])
        r = self.p.plan(open_room(), RES, 0, 0, (0.5, 2.5), (2.5, 2.0))
        self.assertFalse(r["success"])
        self.assertIn("keep-out", r["message"])

    def test_start_inside_a_zone_is_refused_with_a_clear_message(self):
        self.p.set_keepout_zones([BAR])
        r = self.p.plan(open_room(), RES, 0, 0, (2.5, 2.0), (4.5, 4.5))
        self.assertFalse(r["success"])
        self.assertIn("inside a keep-out", r["message"])

    def test_collision_check_honours_zones(self):
        clear, _, _ = self.p.check_path_collision(
            open_room(), RES, 0, 0, [(4.5, 2.5)], (0.5, 2.5), lookahead_m=5)
        self.assertFalse(clear)
        self.p.set_keepout_zones([BAR])
        blocked, _, _ = self.p.check_path_collision(
            open_room(), RES, 0, 0, [(4.5, 2.5)], (0.5, 2.5), lookahead_m=5)
        self.assertTrue(blocked)

    def test_zones_are_replaced_whole(self):
        self.p.set_keepout_zones([BAR])
        self.p.set_keepout_zones([])
        self.assertEqual(self.p.keepout_zones, ())
        r = self.p.plan(open_room(), RES, 0, 0, (0.5, 2.5), (4.5, 2.5))
        self.assertLess(r["total_distance_m"], 4.5)

    def test_inflated_mask_matches_what_the_search_uses(self):
        self.p.set_keepout_zones([BAR])
        raw, inflated = self.p.inflated_mask(open_room(), RES, 0, 0)
        self.assertTrue(raw[100, 80])                  # inside the zone
        self.assertTrue(inflated[86, 80])              # 0.1 m south of it
        self.assertFalse(inflated[40, 40])             # open floor


class OffMapGoalTest(unittest.TestCase):
    def test_off_map_goal_is_refused_not_clamped(self):
        r = AStarPathPlanner(0.25).plan(open_room(), RES, 0, 0, (0.5, 0.5), (99.0, 99.0))
        self.assertFalse(r["success"])
        self.assertIn("outside the mapped area", r["message"])

    def test_goal_just_inside_the_edge_still_plans(self):
        r = AStarPathPlanner(0.25).plan(open_room(), RES, 0, 0, (0.5, 0.5), (4.9, 4.9))
        self.assertTrue(r["success"])


class RouteTest(unittest.TestCase):
    def setUp(self):
        self.p = AStarPathPlanner(robot_radius_m=0.25)

    def test_route_visits_every_stop_in_order(self):
        stops = [(4.5, 0.5), (4.5, 4.5), (0.5, 4.5)]
        r = self.p.plan_route(open_room(), RES, 0, 0, (0.5, 0.5), stops)
        self.assertTrue(r["success"])
        self.assertEqual(len(r["legs"]), 3)
        for idx, stop in zip(r["stop_indices"], stops):
            wx, wy = r["waypoints"][idx]
            self.assertAlmostEqual(wx, stop[0], delta=0.05)
            self.assertAlmostEqual(wy, stop[1], delta=0.05)

    def test_route_distance_is_the_sum_of_its_legs(self):
        r = self.p.plan_route(open_room(), RES, 0, 0, (0.5, 0.5),
                              [(4.5, 0.5), (4.5, 4.5)])
        self.assertAlmostEqual(r["total_distance_m"],
                               sum(l["distance_m"] for l in r["legs"]), places=6)
        self.assertAlmostEqual(r["total_distance_m"], 8.0, delta=0.1)

    def test_an_unreachable_stop_fails_the_whole_route_and_names_it(self):
        r = self.p.plan_route(open_room(), RES, 0, 0, (0.5, 0.5),
                              [(4.5, 0.5), (99.0, 99.0), (0.5, 4.5)])
        self.assertFalse(r["success"])
        self.assertEqual(r["failed_leg"], 1)
        self.assertTrue(r["message"].startswith("Stop 2"))
        self.assertEqual(len(r["legs"]), 1, "legs before the failure are reported")

    def test_route_respects_keepout_zones_on_every_leg(self):
        self.p.set_keepout_zones([BAR])
        r = self.p.plan_route(open_room(), RES, 0, 0, (0.5, 1.0),
                              [(4.5, 1.0), (0.5, 3.0)])
        self.assertTrue(r["success"])
        self.assertFalse(route_enters(r["waypoints"], (0.5, 1.0), BAR))

    def test_empty_mission_is_refused(self):
        self.assertFalse(self.p.plan_route(open_room(), RES, 0, 0, (0, 0), [])["success"])


try:
    from PyQt5.QtWidgets import QApplication
    HAVE_QT = True
except Exception:
    HAVE_QT = False


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class WorkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def _wait(self, sig_holder, timeout=5.0):
        t0 = time.time()
        while not sig_holder and time.time() - t0 < timeout:
            self.app.processEvents()
            time.sleep(0.01)

    def test_route_request_is_answered_with_legs(self):
        from core.planner_worker import PlannerWorker
        w = PlannerWorker(AStarPathPlanner(0.25))
        got = []
        w.plan_ready.connect(lambda tok, res: got.append((tok, res)))
        w.start()
        try:
            tok = w.request_route(open_room(), RES, 0, 0, (0.5, 0.5),
                                  [(4.5, 0.5), (4.5, 4.5)])
            self._wait(got)
            self.assertEqual(got[0][0], tok)
            self.assertEqual(len(got[0][1]["legs"]), 2)
        finally:
            w.stop()

    def test_inflation_request_returns_an_image_with_geometry(self):
        from core.planner_worker import PlannerWorker
        planner = AStarPathPlanner(0.25)
        planner.set_keepout_zones([BAR])
        w = PlannerWorker(planner)
        got = []
        w.inflation_ready.connect(lambda tok, img, geom: got.append((img, geom)))
        w.start()
        try:
            w.request_inflation(open_room(), RES, 0.0, 0.0)
            self._wait(got)
            img, geom = got[0]
            self.assertEqual((img.width(), img.height()), (200, 200))
            self.assertEqual(geom, (RES, 0.0, 0.0))
        finally:
            w.stop()


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class WidgetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        from ui.slam_map_widget import SLAMMapWidget
        self.w = SLAMMapWidget()
        self.w.resize(1200, 700)
        self.w.show()
        self.w.on_ros2_map_received(open_room(), RES, 0.0, 0.0, "/map", None)
        self.cv = self.w.canvas
        self.cv.set_drone_pose(0.5, 0.5, 0.0)
        self.app.processEvents()

    def tearDown(self):
        self.w.close()

    def _scr(self, x, y):
        from PyQt5.QtCore import QPoint
        cx = self.cv.width() / 2 + self.cv.pan_x
        cy = self.cv.height() / 2 + self.cv.pan_y
        q = self.cv._world_to_screen(x, y, cx, cy)
        return QPoint(int(round(q.x())), int(round(q.y())))

    def _click(self, x, y, shift=False):
        from PyQt5.QtCore import Qt, QEvent
        from PyQt5.QtGui import QMouseEvent
        mod = Qt.ShiftModifier if shift else Qt.NoModifier
        pt = self._scr(x, y)
        self.cv.mousePressEvent(QMouseEvent(QEvent.MouseButtonPress, pt, Qt.LeftButton, Qt.LeftButton, mod))
        self.cv.mouseReleaseEvent(QMouseEvent(QEvent.MouseButtonRelease, pt, Qt.LeftButton, Qt.NoButton, mod))
        self.app.processEvents()

    def test_shift_click_builds_a_mission_and_plain_click_resets_it(self):
        self._click(4.0, 0.5)
        self._click(4.0, 4.0, shift=True)
        self._click(1.0, 4.0, shift=True)
        self.assertEqual(len(self.cv.mission_stops), 3)
        self.assertTrue(self.w.mission_panel.isVisible())
        self.assertTrue(self.cv.mission_legs, "planned inline without a worker")
        self.assertIn("MISSION", self.w.lbl_path_info.text())
        self._click(2.0, 2.0)
        self.assertEqual(len(self.cv.mission_stops), 1)
        self.assertFalse(self.w.mission_panel.isVisible())

    def test_opening_the_mission_panel_does_not_move_the_map(self):
        self._click(4.0, 0.5)
        before = self._scr(2.0, 2.0)
        self._click(4.0, 4.0, shift=True)       # panel appears, canvas narrows
        self.app.processEvents()
        after = self._scr(2.0, 2.0)
        self.assertLessEqual(abs(before.x() - after.x()), 1)

    def test_remove_and_reorder_stops(self):
        self._click(4.0, 0.5)
        self._click(4.0, 4.0, shift=True)
        self._click(1.0, 4.0, shift=True)
        first = self.cv.mission_stops[0]
        self.cv.move_mission_stop(0, +1)
        self.assertEqual(self.cv.mission_stops[1], first)
        self.cv.remove_mission_stop(2)
        self.assertEqual(len(self.cv.mission_stops), 2)
        self.assertEqual(self.cv.goal_pose, self.cv.mission_stops[-1])

    def test_drawn_keepout_reaches_the_planner_and_blocks_a_goal(self):
        from PyQt5.QtCore import Qt, QEvent
        from PyQt5.QtGui import QMouseEvent
        self.w.btn_keepout.setChecked(True)
        a, b = self._scr(2.0, 1.5), self._scr(3.0, 3.5)
        self.cv.mousePressEvent(QMouseEvent(QEvent.MouseButtonPress, a, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
        self.cv.mouseMoveEvent(QMouseEvent(QEvent.MouseMove, b, Qt.NoButton, Qt.LeftButton, Qt.NoModifier))
        self.cv.mouseReleaseEvent(QMouseEvent(QEvent.MouseButtonRelease, b, Qt.LeftButton, Qt.NoButton, Qt.NoModifier))
        self.w.btn_keepout.setChecked(False)
        self.assertEqual(len(self.cv.keepout_zones), 1)
        self.assertEqual(len(self.cv.planner.keepout_zones), 1)
        self._click(2.5, 2.5)
        self.assertEqual(self.cv.planned_waypoints, [])
        self.assertFalse(self.w.btn_execute_path.isEnabled())
        self.assertTrue(self.cv.remove_keepout_at(2.5, 2.5))
        self.assertEqual(self.cv.planner.keepout_zones, ())

    def test_keepout_tool_and_ruler_are_exclusive(self):
        self.w.btn_ruler.setChecked(True)
        self.w.btn_keepout.setChecked(True)
        self.assertFalse(self.w.btn_ruler.isChecked())
        self.w.btn_ruler.setChecked(True)
        self.assertFalse(self.w.btn_keepout.isChecked())

    def test_inflation_layer_toggles(self):
        self.w.request_inflation()
        self.assertIsNotNone(self.cv.inflation_qimage)
        self.w.btn_inflation.setChecked(False)
        self.assertFalse(self.cv.inflation_visible)


if __name__ == "__main__":
    unittest.main(verbosity=2)
