"""The planner thread, the live map score, and the stale-map check.

Hermetic apart from PyQt: no ROS 2, no network, no vehicle.

WHY THESE EXIST:
  A*, path collision checking and map scoring all used to run on the GUI
  thread. Measured on a 25 x 25 m room at this project's 2.5 cm resolution:
  42-334 ms for a plan, 34 ms per collision check five times a second. The same
  thread carries the 10 Hz OFFBOARD setpoint pump, and PX4 drops the aircraft
  out of OFFBOARD if setpoints stop for 500 ms. These tests hold the work off
  that thread, and hold the correctness properties that make an asynchronous
  answer safe to act on.
"""

import time
import unittest

import numpy as np

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

try:
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QEventLoop, QTimer
    HAVE_QT = True
except Exception:
    HAVE_QT = False

from core.map_quality import evaluate_live, HAVE_MAP_EVAL


def _room(metres: float = 12.0, res: float = 0.025) -> np.ndarray:
    n = int(metres / res)
    g = np.full((n, n), -1, dtype=np.int8)
    g[20:n - 20, 20:n - 20] = 0
    g[20:n - 20, 20] = 100
    g[20:n - 20, n - 21] = 100
    g[20, 20:n - 20] = 100
    g[n - 21, 20:n - 20] = 100
    return g


class MapQualityTest(unittest.TestCase):
    """Scoring is pure numpy - no Qt needed."""

    def test_reuses_map_evals_own_maths(self):
        self.assertTrue(HAVE_MAP_EVAL,
                        "map_eval must be importable, or the live readout and "
                        "the offline report can disagree about what good means")

    def test_a_clean_room_passes(self):
        q = evaluate_live(_room(), 0.025)
        by = q.by_key()
        self.assertTrue(by["wall_rms"].good, by["wall_rms"].text)
        self.assertTrue(by["squareness"].good, by["squareness"].text)
        self.assertTrue(q.ok)

    def test_noisy_walls_fail(self):
        rng = np.random.default_rng(3)
        n = 400
        g = np.full((n, n), -1, dtype=np.int8)
        g[20:n - 20, 20:n - 20] = 0
        for r in range(20, n - 20):
            g[r, 20 + int(rng.normal(0, 3))] = 100
            g[r, n - 21 + int(rng.normal(0, 3))] = 100
        q = evaluate_live(g, 0.025)
        self.assertFalse(q.by_key()["wall_rms"].good)
        self.assertFalse(q.ok)

    def test_coverage_and_area_are_real_units(self):
        res = 0.05
        g = np.full((100, 100), -1, dtype=np.int8)
        g[:50, :] = 0                        # exactly half observed
        q = evaluate_live(g, res).by_key()
        self.assertAlmostEqual(q["coverage"].value, 50.0, delta=0.5)
        # 5000 cells x 0.05 x 0.05 = 12.5 m2
        self.assertAlmostEqual(q["explored"].value, 12.5, delta=0.1)

    def test_an_empty_map_is_reported_honestly_not_as_a_pass(self):
        g = np.full((80, 80), -1, dtype=np.int8)
        q = evaluate_live(g, 0.025).by_key()
        # Nothing observed: there is no wall to measure, and the metric says so
        # rather than reporting a perfect score for a map that does not exist.
        self.assertEqual(q["wall_rms"].text, "--")
        self.assertIsNone(q["wall_rms"].good)

    def test_degenerate_input_does_not_raise(self):
        for grid, res in ((None, 0.05), (np.zeros((0, 0), dtype=np.int8), 0.05),
                          (_room(), 0.0), (_room(), -1.0)):
            self.assertIsNotNone(evaluate_live(grid, res))


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class PlannerWorkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        from core.path_planner import AStarPathPlanner
        from core.planner_worker import PlannerWorker
        self.grid = _room()
        self.worker = PlannerWorker(AStarPathPlanner(robot_radius_m=0.25))
        self.worker.start()

    def tearDown(self):
        self.worker.stop()

    def _pump(self, predicate, timeout_s=5.0):
        """Run the event loop until predicate() or the timeout."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not predicate():
            self.app.processEvents(QEventLoop.AllEvents, 20)
            time.sleep(0.01)
        return predicate()

    def test_submitting_work_never_blocks_the_caller(self):
        started = time.perf_counter()
        for _ in range(50):
            self.worker.request_plan(self.grid, 0.025, -1.0, -1.0, (1.0, 1.0), (8.0, 8.0))
            self.worker.request_collision_check(
                self.grid, 0.025, -1.0, -1.0, [(4.0, 4.0)], (1.0, 1.0))
            self.worker.request_quality(self.grid, 0.025)
        elapsed = (time.perf_counter() - started) * 1000.0
        # 150 submissions. Anything near the cost of even one real job would
        # mean the work is still happening on the calling thread.
        self.assertLess(elapsed, 50.0, f"submitting took {elapsed:.1f} ms")

    def test_only_the_newest_request_of_each_kind_is_answered(self):
        plans = []
        self.worker.plan_ready.connect(lambda t, r: plans.append(t))
        tokens = [self.worker.request_plan(self.grid, 0.025, -1.0, -1.0,
                                           (1.0, 1.0), (8.0, 8.0))
                  for _ in range(8)]
        self._pump(lambda: len(plans) >= 1)
        time.sleep(0.4)
        self.app.processEvents()
        self.assertLess(len(plans), len(tokens),
                        "a queue would answer every stale request in turn")
        self.assertIn(tokens[-1], plans, "the newest request must be answered")

    def test_a_collision_check_is_not_starved_by_planning(self):
        order = []
        self.worker.plan_ready.connect(lambda t, r: order.append("plan"))
        self.worker.collision_ready.connect(lambda *a: order.append("collision"))
        self.worker.request_plan(self.grid, 0.025, -1.0, -1.0, (1.0, 1.0), (9.0, 9.0))
        self.worker.request_collision_check(
            self.grid, 0.025, -1.0, -1.0, [(4.0, 4.0)], (1.0, 1.0))
        self._pump(lambda: len(order) >= 2)
        self.assertEqual(order[0], "collision",
                         "the check that keeps the aircraft off a wall goes first")

    def test_map_scoring_is_not_starved_by_a_steady_stream_of_checks(self):
        """Regression: the loop only slept when collision and plan were both
        empty, so with checks arriving every 200 ms a pending map score was
        never reached at all."""
        scores = []
        self.worker.quality_ready.connect(lambda t, q: scores.append(q))
        self.worker.request_quality(self.grid, 0.025)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not scores:
            self.worker.request_collision_check(
                self.grid, 0.025, -1.0, -1.0, [(4.0, 4.0)], (1.0, 1.0))
            self.app.processEvents(QEventLoop.AllEvents, 20)
            time.sleep(0.05)
        self.assertTrue(scores, "map scoring was starved by collision checks")

    def test_results_carry_the_token_of_their_request(self):
        seen = {}
        self.worker.plan_ready.connect(lambda t, r: seen.setdefault("plan", t))
        token = self.worker.request_plan(self.grid, 0.025, -1.0, -1.0,
                                         (1.0, 1.0), (8.0, 8.0))
        self._pump(lambda: "plan" in seen)
        self.assertEqual(seen["plan"], token)

    def test_a_planner_fault_does_not_kill_the_worker(self):
        class Exploding:
            def plan(self, *a, **k):
                raise RuntimeError("boom")

            def check_path_collision(self, *a, **k):
                raise RuntimeError("boom")

        results = []
        self.worker.planner = Exploding()
        self.worker.plan_ready.connect(lambda t, r: results.append(r))
        self.worker.request_plan(self.grid, 0.025, -1.0, -1.0, (1.0, 1.0), (8.0, 8.0))
        self._pump(lambda: len(results) >= 1)
        self.assertTrue(results, "a failing planner must still answer")
        self.assertFalse(results[0]["success"])
        self.assertTrue(self.worker.isRunning(), "the worker must survive it")

    def test_stop_is_idempotent(self):
        self.worker.stop()
        self.worker.stop()
        self.assertFalse(self.worker.isRunning())


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class StaleMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_a_map_that_stops_arriving_is_reported_stale(self):
        from ui.slam_map_widget import SLAMMapCanvas
        from core.map_render import build_map_image
        canvas = SLAMMapCanvas()
        canvas.resize(600, 400)
        grid = _room(6.0)
        self.assertFalse(canvas.is_map_stale(),
                         "before any map has arrived there is nothing to be stale")

        canvas.set_occupancy_grid(grid, 0.025, -1.0, -1.0, "/map",
                                  build_map_image(grid, thin=False))
        self.assertFalse(canvas.is_map_stale())

        canvas.map_stale_after_s = 5.0
        canvas.last_map_time = time.time() - 12.0
        self.assertTrue(canvas.is_map_stale())
        self.assertGreater(canvas.map_age_s(), 11.0)
        canvas.grab()          # the overlay painter must not raise

    def test_the_threshold_comes_from_settings(self):
        from ui.slam_map_widget import SLAMMapWidget
        widget = SLAMMapWidget()
        widget.set_map_stale_after(7.5)
        self.assertEqual(widget.canvas.map_stale_after_s, 7.5)
        widget.set_map_stale_after(0)      # nonsense values are ignored
        self.assertEqual(widget.canvas.map_stale_after_s, 7.5)


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class MapImageOffThreadTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_the_canvas_accepts_a_prebuilt_image(self):
        from ui.slam_map_widget import SLAMMapCanvas
        from core.map_render import build_map_image
        grid = _room(6.0)
        image = build_map_image(grid, thin=False)
        canvas = SLAMMapCanvas()
        canvas.set_occupancy_grid(grid, 0.025, -1.0, -1.0, "/map", image)
        self.assertIs(canvas.map_raw_qimage, image)

    def test_it_still_builds_one_when_none_is_given(self):
        from ui.slam_map_widget import SLAMMapCanvas
        grid = _room(6.0)
        canvas = SLAMMapCanvas()
        canvas.set_occupancy_grid(grid, 0.025, -1.0, -1.0, "/map")
        self.assertIsNotNone(canvas.map_raw_qimage)
        self.assertFalse(canvas.map_raw_qimage.isNull())

    def test_the_image_owns_its_pixels(self):
        """It crosses a thread boundary, so it must not reference a buffer the
        producing thread could reuse."""
        import gc
        from core.map_render import build_map_image
        image = build_map_image(_room(6.0), thin=False)
        for _ in range(3):
            gc.collect()
        self.assertFalse(image.isNull())
        self.assertNotEqual(image.pixel(10, 10), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
