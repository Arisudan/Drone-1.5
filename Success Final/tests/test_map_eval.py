"""Tests for scripts/diagnostics/map_eval.py - SLAM map metrics.

Needs numpy only. Everything here runs against synthetic grids, so CI can
grade the evaluator itself without any recorded flight data.

See docs/slam_evaluation.md for what each metric means and why its threshold
is set where it is.
"""

import csv
import json
import math
import os
import tempfile
import unittest

import numpy as np

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

import map_eval as me

RES = 0.025      # matches Grid/CellSize


def write_run(tmp, grid, poses=None, thin=None, meta_extra=None):
    """Materialise a run directory the way map_recorder.py would."""
    np.save(os.path.join(tmp, "map.npy"), grid.astype(np.int8))
    if thin is not None:
        np.save(os.path.join(tmp, "map_thin.npy"), thin.astype(np.int8))
    meta = {"resolution": RES, "origin_x": 0.0, "origin_y": 0.0,
            "width": grid.shape[1], "height": grid.shape[0]}
    meta.update(meta_extra or {})
    with open(os.path.join(tmp, "meta.json"), "w") as fh:
        json.dump(meta, fh)
    if poses is not None:
        with open(os.path.join(tmp, "pose.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "x", "y", "z", "yaw_deg", "cov_xx", "lost"])
            for row in poses:
                w.writerow(row)
    return tmp


class ClassifyTest(unittest.TestCase):
    def test_ros_occupancy_values_split_three_ways(self):
        grid = np.array([[-1, 0, 100], [50, 25, 65]], dtype=np.int8)
        occ, free, unknown = me.classify(grid)
        self.assertTrue(unknown[0, 0])
        self.assertTrue(free[0, 1])        # 0
        self.assertTrue(occ[0, 2])         # 100
        self.assertTrue(occ[1, 2])         # 65 == threshold
        self.assertTrue(free[1, 1])        # 25 == threshold

    def test_undecided_band_belongs_to_no_class(self):
        # Folding 26..64 into "free" is what makes a map look safer than it is.
        grid = np.array([[50]], dtype=np.int8)
        occ, free, unknown = me.classify(grid)
        self.assertFalse(occ[0, 0])
        self.assertFalse(free[0, 0])
        self.assertFalse(unknown[0, 0])

    def test_unknown_is_never_counted_as_free(self):
        grid = np.full((4, 4), -1, dtype=np.int8)
        occ, free, unknown = me.classify(grid)
        self.assertEqual(free.sum(), 0)
        self.assertEqual(unknown.sum(), 16)

    def test_thresholds_are_tunable(self):
        grid = np.array([[40]], dtype=np.int8)
        occ, _f, _u = me.classify(grid, occ_thresh=30)
        self.assertTrue(occ[0, 0])


class CoordinateTest(unittest.TestCase):
    def test_cell_to_world_uses_cell_centres(self):
        x, y = me.cell_to_world(0, 0, RES, (0.0, 0.0))
        self.assertAlmostEqual(x, RES / 2)
        self.assertAlmostEqual(y, RES / 2)

    def test_round_trip_through_world_coordinates(self):
        for col, row in [(0, 0), (37, 91), (199, 199)]:
            x, y = me.cell_to_world(col, row, RES, (-2.5, -1.25))
            self.assertEqual(me.world_to_cell(x, y, RES, (-2.5, -1.25)),
                             (col, row))

    def test_negative_origin(self):
        x, y = me.cell_to_world(100, 100, RES, (-2.5, -2.5))
        self.assertAlmostEqual(x, -2.5 + 100.5 * RES)
        self.assertAlmostEqual(y, -2.5 + 100.5 * RES)


class LineFitTest(unittest.TestCase):
    def test_perfect_line_has_zero_residual(self):
        pts = np.array([[float(i), 10.0] for i in range(20)])
        _c, _d, rms = me.fit_line(pts)
        self.assertAlmostEqual(rms, 0.0, places=9)

    def test_vertical_line_is_handled(self):
        # A y = mx + c fit would blow up here; indoor walls are often vertical.
        pts = np.array([[7.0, float(i)] for i in range(20)])
        _c, d, rms = me.fit_line(pts)
        self.assertAlmostEqual(rms, 0.0, places=9)
        self.assertAlmostEqual(abs(d[1]), 1.0, places=6)

    def test_residual_grows_with_scatter(self):
        base = np.array([[float(i), 0.0] for i in range(40)])
        noisy = base.copy()
        noisy[::2, 1] += 1.0
        noisy[1::2, 1] -= 1.0
        _c, _d, rms = me.fit_line(noisy)
        # Not exactly 1.0: the total-least-squares fit tilts a hair to absorb
        # the alternating offsets, which is the correct behaviour.
        self.assertAlmostEqual(rms, 1.0, delta=0.01)

    def test_too_few_points_is_an_error_not_a_silent_zero(self):
        with self.assertRaises(ValueError):
            me.fit_line(np.array([[1.0, 2.0]]))

    def test_angle_between_perpendicular_and_parallel_lines(self):
        horiz = np.array([1.0, 0.0])
        vert = np.array([0.0, 1.0])
        self.assertAlmostEqual(me.angle_between_deg(horiz, vert), 90.0, places=6)
        self.assertAlmostEqual(me.angle_between_deg(horiz, -horiz), 0.0, places=6)

    def test_angle_is_direction_agnostic(self):
        # Lines are undirected: a wall fitted "backwards" is the same wall.
        d1 = np.array([1.0, 1.0]) / math.sqrt(2)
        self.assertAlmostEqual(me.angle_between_deg(d1, -d1), 0.0, places=6)

    def test_occupied_points_in_box_clips_to_the_grid(self):
        occ = np.zeros((10, 10), dtype=bool)
        occ[5, 5] = True
        pts = me.occupied_points_in_box(occ, (-100, -100, 100, 100))
        self.assertEqual(pts.tolist(), [[5.0, 5.0]])


class WrapTest(unittest.TestCase):
    def test_wrap_handles_the_360_boundary(self):
        self.assertAlmostEqual(me.wrap_deg(350.0), -10.0)
        self.assertAlmostEqual(me.wrap_deg(-350.0), 10.0)
        self.assertAlmostEqual(me.wrap_deg(180.0), 180.0)
        self.assertAlmostEqual(me.wrap_deg(0.0), 0.0)


class TrajectoryTest(unittest.TestCase):
    def poses(self, coords, yaws=None):
        out = []
        for i, (x, y) in enumerate(coords):
            rec = {"x": x, "y": y, "z": 0.0}
            if yaws is not None:
                rec["yaw_deg"] = yaws[i]
            out.append(rec)
        return out

    def test_path_length_sums_segments(self):
        p = self.poses([(0, 0), (3, 0), (3, 4)])
        self.assertAlmostEqual(me.path_length(p), 7.0)   # 3 + 4

    def test_path_length_is_three_dimensional(self):
        p = [{"x": 0, "y": 0, "z": 0}, {"x": 3, "y": 4, "z": 12}]
        self.assertAlmostEqual(me.path_length(p), 13.0)

    def test_perfect_loop_has_zero_gap(self):
        p = self.poses([(0, 0), (5, 0), (5, 5), (0, 5), (0, 0)])
        gap, dist = me.loop_closure_gap(p)
        self.assertAlmostEqual(gap, 0.0)
        self.assertAlmostEqual(dist, 20.0)

    def test_drifted_loop_reports_the_gap(self):
        p = self.poses([(0, 0), (5, 0), (5, 5), (0, 5), (0.4, 0.3)])
        gap, dist = me.loop_closure_gap(p)
        self.assertAlmostEqual(gap, 0.5)
        self.assertGreater(dist, 19.0)

    def test_single_sample_is_not_a_loop(self):
        self.assertEqual(me.loop_closure_gap([{"x": 1, "y": 1}]), (0.0, 0.0))

    def test_yaw_drift_wraps_instead_of_reporting_358_degrees(self):
        p = self.poses([(0, 0), (1, 1)], yaws=[359.0, 1.0])
        self.assertAlmostEqual(me.yaw_drift(p), 2.0)

    def test_yaw_drift_is_none_without_the_column(self):
        self.assertIsNone(me.yaw_drift(self.poses([(0, 0), (1, 1)])))


class VoHealthTest(unittest.TestCase):
    def test_explicit_lost_column_is_preferred(self):
        poses = [{"x": 0, "y": 0, "lost": 0.0}] * 9 + [{"x": 0, "y": 0, "lost": 1.0}]
        self.assertAlmostEqual(me.vo_lost_fraction(poses), 10.0)

    def test_falls_back_to_the_rtabmap_covariance_sentinel(self):
        # stereo_odometry publishes cov 9999.0 when tracking is lost.
        poses = [{"x": 0, "y": 0, "cov_xx": 0.01}] * 8 + \
                [{"x": 0, "y": 0, "cov_xx": 9999.0}] * 2
        self.assertAlmostEqual(me.vo_lost_fraction(poses), 20.0)

    def test_no_poses_gives_none_not_a_false_pass(self):
        self.assertIsNone(me.vo_lost_fraction([]))

    def test_inlier_ratio_ignores_zero_match_frames(self):
        poses = [{"x": 0, "y": 0, "inliers": 40, "matches": 80},
                 {"x": 0, "y": 0, "inliers": 0, "matches": 0}]
        self.assertAlmostEqual(me.mean_inlier_ratio(poses), 0.5)

    def test_inlier_ratio_is_none_without_columns(self):
        self.assertIsNone(me.mean_inlier_ratio([{"x": 0, "y": 0}]))


class CoverageTest(unittest.TestCase):
    def test_sweep_disk_has_the_requested_radius(self):
        poses = [{"x": 1.0, "y": 1.0}]
        mask = me.swept_mask((100, 100), poses, RES, (0.0, 0.0), radius_m=0.25)
        rad_cells = 0.25 / RES
        self.assertAlmostEqual(mask.sum(), math.pi * rad_cells ** 2, delta=60)

    def test_sweep_clips_at_grid_edges_without_wrapping(self):
        poses = [{"x": 0.0, "y": 0.0}]     # corner cell
        mask = me.swept_mask((50, 50), poses, RES, (0.0, 0.0), radius_m=0.25)
        self.assertTrue(mask[0, 0])
        self.assertFalse(mask[49, 49])     # a wrap bug would light this up

    def test_no_poses_means_no_swept_area(self):
        self.assertEqual(me.swept_mask((10, 10), [], RES, (0.0, 0.0)).sum(), 0)

    def test_completeness_only_counts_inside_the_swept_area(self):
        occ = np.zeros((10, 10), dtype=bool)
        free = np.zeros((10, 10), dtype=bool)
        sweep = np.zeros((10, 10), dtype=bool)
        sweep[0:2, 0:2] = True          # 4 cells swept
        free[0:2, 0:1] = True           # 2 of them known
        free[5:8, 5:8] = True           # known, but never swept: must not count
        self.assertAlmostEqual(me.completeness_pct(occ, free, sweep), 50.0)

    def test_completeness_is_none_when_nothing_was_swept(self):
        z = np.zeros((4, 4), dtype=bool)
        self.assertIsNone(me.completeness_pct(z, z, z))


class GroundTruthTest(unittest.TestCase):
    def test_free_space_precision_punishes_false_free(self):
        free = np.zeros((10, 10), dtype=bool)
        free[0, 0:10] = True                    # 10 cells called free
        gt = np.zeros((10, 10), dtype=bool)
        gt[0, 0:9] = True                       # 9 of them truly are
        self.assertAlmostEqual(me.free_space_precision_pct(free, gt), 90.0)

    def test_obstacle_recall_counts_missed_obstacles(self):
        occ = np.zeros((10, 10), dtype=bool)
        occ[5, 0:5] = True
        gt = np.zeros((10, 10), dtype=bool)
        gt[5, 0:10] = True                      # half the wall was missed
        self.assertAlmostEqual(me.obstacle_recall_pct(occ, gt), 50.0)

    def test_empty_inputs_give_none(self):
        z = np.zeros((4, 4), dtype=bool)
        self.assertIsNone(me.free_space_precision_pct(z, z))
        self.assertIsNone(me.obstacle_recall_pct(z, z))


class MetricVerdictTest(unittest.TestCase):
    def test_lower_is_better(self):
        self.assertEqual(me.Metric("m", 1.0, "%", 2.0, "<=").status, "PASS")
        self.assertEqual(me.Metric("m", 3.0, "%", 2.0, "<=").status, "FAIL")

    def test_higher_is_better(self):
        self.assertEqual(me.Metric("m", 95.0, "%", 90.0, ">=").status, "PASS")
        self.assertEqual(me.Metric("m", 80.0, "%", 90.0, ">=").status, "FAIL")

    def test_exactly_on_the_limit_passes(self):
        self.assertEqual(me.Metric("m", 2.0, "%", 2.0, "<=").status, "PASS")
        self.assertEqual(me.Metric("m", 90.0, "%", 90.0, ">=").status, "PASS")

    def test_missing_value_skips_rather_than_fails(self):
        m = me.Metric("m", None, "%", 2.0, "<=")
        self.assertEqual(m.status, "SKIP")
        self.assertFalse(m.failed)

    def test_no_threshold_is_informational(self):
        self.assertEqual(me.Metric("m", 5.0, "cells", None).status, "INFO")


class EndToEndTest(unittest.TestCase):
    """A clean synthetic run must pass; a drifted one must fail, and say why."""

    def _good_run(self, tmp):
        # 4 m x 4 m room at 2.5 cm: free interior, occupied walls.
        n = 160
        grid = np.zeros((n, n), dtype=np.int8)
        grid[0, :] = grid[-1, :] = 100
        grid[:, 0] = grid[:, -1] = 100
        # A closed square loop well inside the walls.
        poses = []
        t = 0.0
        pts = [(0.5, 0.5), (3.5, 0.5), (3.5, 3.5), (0.5, 3.5), (0.5, 0.5)]
        yaws = [0.0, 90.0, 180.0, 270.0, 0.0]
        for i in range(len(pts) - 1):
            (x0, y0), (x1, y1) = pts[i], pts[i + 1]
            for s in range(41):
                f = s / 40.0
                poses.append([f"{t:.2f}", x0 + (x1 - x0) * f, y0 + (y1 - y0) * f,
                              0.0, yaws[i], 0.01, 0])
                t += 0.1
        poses.append([f"{t:.2f}", 0.5, 0.5, 0.0, 0.0, 0.01, 0])
        return write_run(tmp, grid, poses)

    def test_clean_run_passes_every_graded_metric(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = me.load_run(self._good_run(tmp))
            metrics = me.evaluate(run, is_loop=True, sweep_radius_m=3.5,
                                  true_path_m=12.0)
            failures = [m.name for m in metrics if m.failed]
            self.assertEqual(failures, [], f"unexpected failures: {failures}")

    def test_wall_straightness_and_squareness_on_real_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = me.load_run(self._good_run(tmp))
            metrics = {m.name: m for m in me.evaluate(
                run,
                wall_box=(0, 0, 0, 159),                     # the left wall
                corner_boxes=((0, 0, 0, 159), (0, 0, 159, 0)),
            )}
            self.assertEqual(metrics["wall_straightness"].status, "PASS")
            self.assertAlmostEqual(metrics["wall_straightness"].value, 0.0, places=6)
            self.assertEqual(metrics["squareness_error"].status, "PASS")

    def test_scale_error_from_a_surveyed_landmark_pair(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = me.load_run(self._good_run(tmp))
            # Two cells 100 apart at 2.5 cm = exactly 2.50 m.
            metrics = {m.name: m for m in me.evaluate(
                run, landmark=((10, 10), (110, 10)), landmark_true_m=2.50)}
            self.assertEqual(metrics["scale_error"].status, "PASS")
            self.assertLess(metrics["scale_error"].value, 0.1)

    def test_scale_error_catches_a_wrong_scale(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = me.load_run(self._good_run(tmp))
            metrics = {m.name: m for m in me.evaluate(
                run, landmark=((10, 10), (110, 10)), landmark_true_m=2.00)}
            self.assertEqual(metrics["scale_error"].status, "FAIL")

    def test_drifted_loop_fails_closure_and_yaw(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = self._good_run(tmp)
            run = me.load_run(run_dir)
            # End the loop 1 m and 20 deg away from where it started.
            run.poses[-1]["x"] += 1.0
            run.poses[-1]["yaw_deg"] = 20.0
            metrics = {m.name: m for m in me.evaluate(run, is_loop=True)}
            self.assertEqual(metrics["loop_closure_gap"].status, "FAIL")
            self.assertEqual(metrics["yaw_drift"].status, "FAIL")

    def test_lost_tracking_fails_vo_health(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = me.load_run(self._good_run(tmp))
            for p in run.poses[:60]:
                p["lost"] = 1.0
            metrics = {m.name: m for m in me.evaluate(run)}
            self.assertEqual(metrics["vo_lost"].status, "FAIL")

    def test_metrics_without_inputs_report_skip(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = me.load_run(self._good_run(tmp))
            metrics = {m.name: m for m in me.evaluate(run)}
            for name in ("scale_error", "wall_straightness", "squareness_error",
                         "loop_closure_gap", "path_scale_error"):
                self.assertEqual(metrics[name].status, "SKIP", name)

    def test_report_and_json_render(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = me.load_run(self._good_run(tmp))
            metrics = me.evaluate(run, is_loop=True)
            text = me.format_report(run, metrics)
            self.assertIn("SLAM MAP EVALUATION", text)
            self.assertIn("VERDICT:", text)
            payload = me.metrics_to_dict(run, metrics)
            self.assertEqual(payload["verdict"], "PASS")
            json.dumps(payload)      # must be serialisable


class LoadingTest(unittest.TestCase):
    def test_missing_run_directory_is_reported(self):
        with self.assertRaises(FileNotFoundError):
            me.load_run("/nonexistent/run_dir")

    def test_pose_csv_is_optional(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_run(tmp, np.zeros((20, 20), dtype=np.int8))
            run = me.load_run(tmp)
            self.assertEqual(run.poses, [])
            metrics = {m.name: m for m in me.evaluate(run)}
            self.assertEqual(metrics["completeness"].status, "SKIP")

    def test_non_numeric_pose_columns_are_skipped_not_fatal(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "pose.csv")
            with open(path, "w", newline="") as fh:
                fh.write("t_s,x,y,note\n0.0,1.0,2.0,takeoff\n")
            poses = me.load_poses(path)
            self.assertEqual(len(poses), 1)
            self.assertNotIn("note", poses[0])

    def test_one_dimensional_grid_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            np.save(os.path.join(tmp, "map.npy"), np.zeros(10, dtype=np.int8))
            with open(os.path.join(tmp, "meta.json"), "w") as fh:
                json.dump({"resolution": RES}, fh)
            with self.assertRaises(ValueError):
                me.load_run(tmp)


class CliTest(unittest.TestCase):
    def test_exit_code_is_zero_on_pass_and_one_on_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            grid = np.zeros((160, 160), dtype=np.int8)
            poses = [["0.0", 1.0, 1.0, 0.0, 0.0, 0.01, 0],
                     ["0.1", 1.5, 1.0, 0.0, 0.0, 0.01, 0],
                     ["0.2", 1.0, 1.0, 0.0, 0.0, 0.01, 0]]
            write_run(tmp, grid, poses)
            self.assertEqual(me.main([tmp, "--quiet"]), 0)
            # 1 m surveyed vs 1 m flown is fine; 10 m is not.
            self.assertEqual(me.main([tmp, "--quiet", "--true-path-m", "10.0"]), 1)

    def test_missing_run_exits_two(self):
        self.assertEqual(me.main(["/nonexistent/run", "--quiet"]), 2)

    def test_json_output_is_written(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_run(tmp, np.zeros((40, 40), dtype=np.int8))
            out = os.path.join(tmp, "report.json")
            me.main([tmp, "--quiet", "--json", out])
            with open(out) as fh:
                self.assertIn("metrics", json.load(fh))

    def test_landmark_argument_parsing_rejects_garbage(self):
        with self.assertRaises(SystemExit):
            me.build_parser().parse_args(["run", "--landmark", "bad", "4,5"])


if __name__ == "__main__":
    unittest.main()
