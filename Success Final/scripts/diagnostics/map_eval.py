#!/usr/bin/env python3
"""
================================================================================
TOOL: map_eval.py
PURPOSE: Numeric Pass/Fail Evaluation of a Recorded 2.5 cm RTAB-Map Occupancy Grid
================================================================================

WHAT THIS IS FOR:
  Until now the only test this project's map has faced is "it looks about right
  in the GCS". This turns that into numbers - scale error, wall straightness,
  squareness, coverage, loop-closure drift, yaw drift and VIO health - each
  against a stated threshold, with a non-zero exit code when a hard metric
  fails. Method and capture procedure: docs/slam_evaluation.md

INPUT - a run directory written by map_recorder.py:
    out_mapping/run_<timestamp>/
      map.npy        int8 (H, W)  raw /map grid, ROS row-major
      map_thin.npy   int8 (H, W)  skeletonised /map_thin grid (optional)
      meta.json                   resolution, origin, dims, topics, stamps
      pose.csv                    t_s,x,y,z,yaw_deg,cov_xx,lost[,inliers,matches]

GRID ENCODING (nav_msgs/OccupancyGrid - NOT log-odds):
  -1       unknown / never observed
   0..100  occupancy probability, percent
  There is no sigmoid step: thresholds apply directly to the stored integers.
  Defaults are occupied >= 65, free <= 25; values in between stay undecided
  rather than being forced into a class.

DEPENDENCIES: numpy only. No rclpy, no scipy, no Qt - runs on the laptop, on
the Radxa, or in CI against a synthetic grid.

USAGE:
  ./map_eval.py out_mapping/run_20260918_1204
  ./map_eval.py <run> --landmark 412,300 412,460 --landmark-true-m 4.00
  ./map_eval.py <run> --wall 400,290 425,470
  ./map_eval.py <run> --corner 400,290 425,470 600,300 780,325
  ./map_eval.py <run> --loop --true-path-m 38.5
  ./map_eval.py <run> --json report.json
================================================================================
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


# ─── Defaults ───────────────────────────────────────────────────────
# Occupancy classification. RTAB-Map emits 0 / 100 / -1 in practice, but the
# band leaves room for probabilistic grids without silently reclassifying
# half-confident cells.
OCC_THRESH = 65
FREE_THRESH = 25

# Sensor horizon used to define "area the drone actually swept". Mirrors
# Grid/RangeMax in launch/rtabmap_slam.launch.py - cells further than this
# from the flown path were never observable and must not count against
# coverage.
DEFAULT_SWEEP_RADIUS_M = 3.5

# RTAB-Map's stereo_odometry publishes this covariance when tracking is lost
# (see gotalldone.md dev log #6, "cov0 = 9999.0").
VO_LOST_COV = 9999.0

# Acceptance thresholds. Sources: docs/slam_evaluation.md.
THRESHOLDS = {
    "scale_error_pct": 2.0,
    "wall_rms_cells": 1.0,
    "squareness_err_deg": 3.0,
    "completeness_pct": 90.0,
    "loop_gap_pct": 2.0,
    "yaw_drift_deg": 5.0,
    "path_scale_err_pct": 3.0,
    "vo_lost_pct": 5.0,
    "free_precision_pct": 95.0,
    "obstacle_recall_pct": 90.0,
    "inlier_ratio": 0.50,
}


# ─── Result plumbing ────────────────────────────────────────────────

@dataclass
class Metric:
    """One measured quantity plus the verdict against its threshold."""

    name: str
    value: Optional[float]
    unit: str
    threshold: Optional[float]
    # "<=" means lower is better, ">=" means higher is better.
    op: str = "<="
    note: str = ""

    @property
    def status(self) -> str:
        if self.value is None:
            return "SKIP"
        if self.threshold is None:
            return "INFO"
        if self.op == "<=":
            return "PASS" if self.value <= self.threshold else "FAIL"
        return "PASS" if self.value >= self.threshold else "FAIL"

    @property
    def failed(self) -> bool:
        return self.status == "FAIL"


@dataclass
class Run:
    """A loaded recording: grids + metadata + trajectory."""

    path: str
    meta: Dict
    grid: np.ndarray                      # int8 (H, W), raw /map
    grid_thin: Optional[np.ndarray] = None
    poses: List[Dict[str, float]] = field(default_factory=list)

    @property
    def resolution(self) -> float:
        return float(self.meta.get("resolution", 0.025))

    @property
    def origin(self) -> Tuple[float, float]:
        return (float(self.meta.get("origin_x", 0.0)),
                float(self.meta.get("origin_y", 0.0)))


# ─── Loading ────────────────────────────────────────────────────────

def load_run(run_dir: str) -> Run:
    """Read a run directory produced by map_recorder.py."""
    meta_path = os.path.join(run_dir, "meta.json")
    grid_path = os.path.join(run_dir, "map.npy")
    if not os.path.isfile(meta_path):
        raise FileNotFoundError(f"missing {meta_path}")
    if not os.path.isfile(grid_path):
        raise FileNotFoundError(f"missing {grid_path}")

    with open(meta_path, encoding="utf-8") as fh:
        meta = json.load(fh)

    grid = np.load(grid_path)
    if grid.ndim != 2:
        raise ValueError(f"map.npy must be 2-D (H, W), got shape {grid.shape}")

    thin = None
    thin_path = os.path.join(run_dir, "map_thin.npy")
    if os.path.isfile(thin_path):
        thin = np.load(thin_path)

    poses = load_poses(os.path.join(run_dir, "pose.csv"))
    return Run(path=run_dir, meta=meta, grid=grid, grid_thin=thin, poses=poses)


def load_poses(csv_path: str) -> List[Dict[str, float]]:
    """Read pose.csv. Missing file is not an error - grid-only metrics still run."""
    if not os.path.isfile(csv_path):
        return []
    out: List[Dict[str, float]] = []
    with open(csv_path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            rec: Dict[str, float] = {}
            for key, raw in row.items():
                if key is None or raw is None or raw == "":
                    continue
                try:
                    rec[key.strip()] = float(raw)
                except ValueError:
                    continue
            if "x" in rec and "y" in rec:
                out.append(rec)
    return out


# ─── Grid classification ────────────────────────────────────────────

def classify(grid: np.ndarray, occ_thresh: int = OCC_THRESH,
             free_thresh: int = FREE_THRESH
             ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Split a grid into (occupied, free, unknown) boolean masks.

    Cells between free_thresh and occ_thresh belong to none of the three -
    they are observed but undecided, and lumping them into "free" is exactly
    the mistake that makes a map look safer than it is.
    """
    g = np.asarray(grid)
    unknown = g < 0
    occupied = g >= occ_thresh
    free = (~unknown) & (g <= free_thresh)
    return occupied, free, unknown


def cell_to_world(col: float, row: float, resolution: float,
                  origin: Tuple[float, float]) -> Tuple[float, float]:
    """(col, row) in the stored (H, W) grid -> ROS world metres.

    Column indexes X, row indexes Y, both measured from the grid origin -
    the standard nav_msgs/OccupancyGrid layout as written by map_recorder.py.
    Note the GCS's own listener transposes to (W, H) for rendering; this tool
    deliberately stays in ROS-native orientation.
    """
    return (origin[0] + (col + 0.5) * resolution,
            origin[1] + (row + 0.5) * resolution)


def world_to_cell(x: float, y: float, resolution: float,
                  origin: Tuple[float, float]) -> Tuple[int, int]:
    """ROS world metres -> (col, row) index."""
    return (int((x - origin[0]) / resolution),
            int((y - origin[1]) / resolution))


# ─── Geometry primitives ────────────────────────────────────────────

def fit_line(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """Total-least-squares line fit through Nx2 points.

    Returns (centroid, unit_direction, rms_perpendicular_residual). Uses SVD
    rather than a y = mx + c fit so a perfectly vertical wall is handled the
    same as a horizontal one - which matters here, since walls in an indoor
    grid are usually axis-aligned.
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[0] < 2 or pts.shape[1] != 2:
        raise ValueError("fit_line needs at least 2 points of shape (N, 2)")
    centroid = pts.mean(axis=0)
    centred = pts - centroid
    _u, _s, vt = np.linalg.svd(centred, full_matrices=False)
    direction = vt[0] / np.linalg.norm(vt[0])
    normal = np.array([-direction[1], direction[0]])
    residuals = centred @ normal
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    return centroid, direction, rms


def occupied_points_in_box(occupied: np.ndarray,
                           box: Tuple[int, int, int, int]) -> np.ndarray:
    """Occupied cells inside an axis-aligned (c0, r0, c1, r1) box, as (col, row)."""
    c0, r0, c1, r1 = box
    c0, c1 = sorted((int(c0), int(c1)))
    r0, r1 = sorted((int(r0), int(r1)))
    h, w = occupied.shape
    c0, c1 = max(0, c0), min(w - 1, c1)
    r0, r1 = max(0, r0), min(h - 1, r1)
    sub = occupied[r0:r1 + 1, c0:c1 + 1]
    rows, cols = np.nonzero(sub)
    return np.stack([cols + c0, rows + r0], axis=1).astype(float)


def line_angle_deg(direction: np.ndarray) -> float:
    """Direction vector -> angle in [0, 180). Lines are undirected."""
    ang = math.degrees(math.atan2(direction[1], direction[0])) % 180.0
    return ang


def angle_between_deg(d1: np.ndarray, d2: np.ndarray) -> float:
    """Smallest angle between two undirected lines, in [0, 90]."""
    diff = abs(line_angle_deg(d1) - line_angle_deg(d2)) % 180.0
    return diff if diff <= 90.0 else 180.0 - diff


def wrap_deg(angle: float) -> float:
    """Wrap to (-180, 180] so a 359 deg -> 1 deg turn reads as +2, not -358.

    The half-open end matters: modulo alone maps exactly 180 to -180, which
    would flip the reported sign of a half-turn.
    """
    wrapped = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


# ─── Trajectory metrics ─────────────────────────────────────────────

def path_length(poses: Sequence[Dict[str, float]]) -> float:
    """Total travelled distance in metres (3-D when z is present)."""
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        dx = b["x"] - a["x"]
        dy = b["y"] - a["y"]
        dz = b.get("z", 0.0) - a.get("z", 0.0)
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total


def loop_closure_gap(poses: Sequence[Dict[str, float]]) -> Tuple[float, float]:
    """(start-to-end gap in metres, travelled path length in metres).

    On a trajectory that physically returns to its start, the gap IS the
    accumulated drift - no external ground truth needed. This is the cheapest
    real drift number this project can produce.
    """
    if len(poses) < 2:
        return 0.0, 0.0
    a, b = poses[0], poses[-1]
    dx = b["x"] - a["x"]
    dy = b["y"] - a["y"]
    dz = b.get("z", 0.0) - a.get("z", 0.0)
    return math.sqrt(dx * dx + dy * dy + dz * dz), path_length(poses)


def yaw_drift(poses: Sequence[Dict[str, float]]) -> Optional[float]:
    """Signed heading error over a closed loop, degrees."""
    if len(poses) < 2 or "yaw_deg" not in poses[0] or "yaw_deg" not in poses[-1]:
        return None
    return wrap_deg(poses[-1]["yaw_deg"] - poses[0]["yaw_deg"])


def vo_lost_fraction(poses: Sequence[Dict[str, float]]) -> Optional[float]:
    """Fraction of samples where stereo VIO reported LOST.

    Prefers an explicit `lost` column; falls back to the covariance sentinel
    RTAB-Map publishes when tracking drops (cov0 = 9999.0).
    """
    if not poses:
        return None
    flagged = 0
    counted = 0
    for p in poses:
        if "lost" in p:
            counted += 1
            flagged += 1 if p["lost"] >= 0.5 else 0
        elif "cov_xx" in p:
            counted += 1
            flagged += 1 if p["cov_xx"] >= VO_LOST_COV else 0
    if counted == 0:
        return None
    return 100.0 * flagged / counted


def mean_inlier_ratio(poses: Sequence[Dict[str, float]]) -> Optional[float]:
    """Mean VO inliers/matches, when the recorder captured those columns."""
    ratios = [p["inliers"] / p["matches"] for p in poses
              if p.get("matches", 0) > 0 and "inliers" in p]
    if not ratios:
        return None
    return float(np.mean(ratios))


# ─── Coverage ───────────────────────────────────────────────────────

def swept_mask(shape: Tuple[int, int], poses: Sequence[Dict[str, float]],
               resolution: float, origin: Tuple[float, float],
               radius_m: float = DEFAULT_SWEEP_RADIUS_M) -> np.ndarray:
    """Cells within sensor range of the flown trajectory.

    Coverage must only be scored where the drone could actually see. Scoring
    the whole grid would punish the map for the far side of a wall it never
    flew past, which says nothing about SLAM quality.
    """
    h, w = shape
    mask = np.zeros((h, w), dtype=bool)
    if not poses:
        return mask

    rad_cells = max(1, int(round(radius_m / resolution)))
    # Stamp one disk per unique visited cell. Deduplicating first keeps this
    # cheap on long runs, where thousands of poses land in the same cells.
    visited = {world_to_cell(p["x"], p["y"], resolution, origin) for p in poses}

    size = 2 * rad_cells + 1
    yy, xx = np.ogrid[-rad_cells:rad_cells + 1, -rad_cells:rad_cells + 1]
    disk = (xx * xx + yy * yy) <= rad_cells * rad_cells

    for col, row in visited:
        r0, r1 = row - rad_cells, row + rad_cells + 1
        c0, c1 = col - rad_cells, col + rad_cells + 1
        # Clip the stamp against the grid edge on both sides at once.
        dr0, dc0 = max(0, -r0), max(0, -c0)
        dr1 = size - max(0, r1 - h)
        dc1 = size - max(0, c1 - w)
        if dr0 >= dr1 or dc0 >= dc1:
            continue
        mask[r0 + dr0:r0 + dr1, c0 + dc0:c0 + dc1] |= disk[dr0:dr1, dc0:dc1]
    return mask


def completeness_pct(occupied: np.ndarray, free: np.ndarray,
                     sweep: np.ndarray) -> Optional[float]:
    """Percentage of the swept area that ended up classified, not unknown."""
    denom = int(sweep.sum())
    if denom == 0:
        return None
    known = int((sweep & (occupied | free)).sum())
    return 100.0 * known / denom


# ─── Ground-truth comparisons (optional) ────────────────────────────

def free_space_precision_pct(free: np.ndarray,
                             gt_traversable: np.ndarray) -> Optional[float]:
    """Of the cells the map calls free, how many really are traversable.

    Asymmetric on purpose: a false "free" cell is what flies a drone into a
    wall, whereas a false "occupied" cell only costs a detour.
    """
    denom = int(free.sum())
    if denom == 0:
        return None
    return 100.0 * int((free & gt_traversable).sum()) / denom


def obstacle_recall_pct(occupied: np.ndarray,
                        gt_obstacle: np.ndarray) -> Optional[float]:
    """Of the real obstacles, how many the map actually marked occupied."""
    denom = int(gt_obstacle.sum())
    if denom == 0:
        return None
    return 100.0 * int((occupied & gt_obstacle).sum()) / denom


# ─── Evaluation ─────────────────────────────────────────────────────

def evaluate(run: Run, *,
             landmark: Optional[Tuple[Tuple[int, int], Tuple[int, int]]] = None,
             landmark_true_m: Optional[float] = None,
             wall_box: Optional[Tuple[int, int, int, int]] = None,
             corner_boxes: Optional[Tuple[Tuple[int, int, int, int],
                                          Tuple[int, int, int, int]]] = None,
             is_loop: bool = False,
             true_path_m: Optional[float] = None,
             sweep_radius_m: float = DEFAULT_SWEEP_RADIUS_M,
             occ_thresh: int = OCC_THRESH,
             free_thresh: int = FREE_THRESH,
             gt_traversable: Optional[np.ndarray] = None,
             gt_obstacle: Optional[np.ndarray] = None,
             thresholds: Optional[Dict[str, float]] = None
             ) -> List[Metric]:
    """Run every applicable metric. Metrics lacking their inputs report SKIP."""
    th = dict(THRESHOLDS)
    if thresholds:
        th.update(thresholds)

    res = run.resolution
    origin = run.origin
    occupied, free, unknown = classify(run.grid, occ_thresh, free_thresh)
    metrics: List[Metric] = []

    # ── Inventory (context, never pass/fail) ──
    total = occupied.size
    metrics.append(Metric("cells_total", float(total), "cells", None, note="grid size"))
    metrics.append(Metric("occupied_pct", 100.0 * occupied.sum() / total, "%", None))
    metrics.append(Metric("free_pct", 100.0 * free.sum() / total, "%", None))
    metrics.append(Metric("unknown_pct", 100.0 * unknown.sum() / total, "%", None))
    metrics.append(Metric("resolution", res, "m/cell", None,
                          note="Grid/CellSize"))

    # ── 1.1 Geometric accuracy ──
    scale_err = None
    if landmark is not None and landmark_true_m:
        (c1, r1), (c2, r2) = landmark
        x1, y1 = cell_to_world(c1, r1, res, origin)
        x2, y2 = cell_to_world(c2, r2, res, origin)
        measured = math.hypot(x2 - x1, y2 - y1)
        scale_err = abs(measured - landmark_true_m) / landmark_true_m * 100.0
        note = f"measured {measured:.3f} m vs surveyed {landmark_true_m:.3f} m"
    else:
        note = "needs --landmark and --landmark-true-m"
    metrics.append(Metric("scale_error", scale_err, "%",
                          th["scale_error_pct"], "<=", note))

    # Straightness is measured on /map_thin when present: the skeleton is one
    # cell wide, so residuals reflect wall geometry rather than wall thickness.
    straight_src = "map_thin" if run.grid_thin is not None else "map"
    straight_grid = run.grid_thin if run.grid_thin is not None else run.grid
    occ_straight, _, _ = classify(straight_grid, occ_thresh, free_thresh)

    wall_rms = None
    wall_note = "needs --wall"
    if wall_box is not None:
        pts = occupied_points_in_box(occ_straight, wall_box)
        if len(pts) >= 2:
            _c, _d, wall_rms = fit_line(pts)
            wall_note = f"{len(pts)} cells from /{straight_src}"
        else:
            wall_note = f"only {len(pts)} occupied cells in the box"
    metrics.append(Metric("wall_straightness", wall_rms, "cells",
                          th["wall_rms_cells"], "<=", wall_note))

    sq_err = None
    sq_note = "needs --corner"
    if corner_boxes is not None:
        pa = occupied_points_in_box(occ_straight, corner_boxes[0])
        pb = occupied_points_in_box(occ_straight, corner_boxes[1])
        if len(pa) >= 2 and len(pb) >= 2:
            _, da, _ = fit_line(pa)
            _, db, _ = fit_line(pb)
            measured_angle = angle_between_deg(da, db)
            sq_err = abs(measured_angle - 90.0)
            sq_note = f"corner measured {measured_angle:.2f} deg"
        else:
            sq_note = f"too few cells ({len(pa)}, {len(pb)})"
    metrics.append(Metric("squareness_error", sq_err, "deg",
                          th["squareness_err_deg"], "<=", sq_note))

    # ── 1.4 Coverage ──
    comp = None
    comp_note = "needs pose.csv"
    if run.poses:
        sweep = swept_mask(run.grid.shape, run.poses, res, origin, sweep_radius_m)
        comp = completeness_pct(occupied, free, sweep)
        comp_note = f"within {sweep_radius_m:.1f} m of path ({int(sweep.sum())} cells)"
    metrics.append(Metric("completeness", comp, "%",
                          th["completeness_pct"], ">=", comp_note))

    # ── 1.2 Trajectory / drift ──
    gap_pct = None
    gap_note = "needs --loop with pose.csv"
    dist = path_length(run.poses) if run.poses else 0.0
    if is_loop and len(run.poses) >= 2:
        gap, dist = loop_closure_gap(run.poses)
        if dist > 0:
            gap_pct = 100.0 * gap / dist
            gap_note = f"gap {gap:.3f} m over {dist:.2f} m flown"
        else:
            gap_note = "trajectory has zero length"
    metrics.append(Metric("loop_closure_gap", gap_pct, "% of path",
                          th["loop_gap_pct"], "<=", gap_note))
    metrics.append(Metric("path_length", dist if run.poses else None, "m",
                          None, note=f"{len(run.poses)} pose samples"))

    yaw_err = None
    yaw_note = "needs --loop with yaw_deg column"
    if is_loop:
        drift = yaw_drift(run.poses)
        if drift is not None:
            yaw_err = abs(drift)
            yaw_note = f"{drift:+.2f} deg over the loop"
    metrics.append(Metric("yaw_drift", yaw_err, "deg",
                          th["yaw_drift_deg"], "<=", yaw_note))

    path_scale = None
    path_note = "needs --true-path-m"
    if true_path_m and dist > 0:
        path_scale = abs(dist - true_path_m) / true_path_m * 100.0
        path_note = f"odometry {dist:.2f} m vs surveyed {true_path_m:.2f} m"
    metrics.append(Metric("path_scale_error", path_scale, "%",
                          th["path_scale_err_pct"], "<=", path_note))

    # ── 1.3 VO health (explains WHY a map is good or bad) ──
    lost = vo_lost_fraction(run.poses)
    metrics.append(Metric("vo_lost", lost, "% of samples",
                          th["vo_lost_pct"], "<=",
                          "cov_xx >= 9999 or lost flag"))
    inl = mean_inlier_ratio(run.poses)
    metrics.append(Metric("vo_inlier_ratio", inl, "ratio",
                          th["inlier_ratio"], ">=",
                          "needs inliers/matches columns"))

    # ── Ground-truth comparisons ──
    fp = (free_space_precision_pct(free, gt_traversable)
          if gt_traversable is not None else None)
    metrics.append(Metric("free_space_precision", fp, "%",
                          th["free_precision_pct"], ">=",
                          "needs --gt-traversable"))
    orc = (obstacle_recall_pct(occupied, gt_obstacle)
           if gt_obstacle is not None else None)
    metrics.append(Metric("obstacle_recall", orc, "%",
                          th["obstacle_recall_pct"], ">=",
                          "needs --gt-obstacle"))

    return metrics


# ─── Reporting ──────────────────────────────────────────────────────

def format_report(run: Run, metrics: Sequence[Metric]) -> str:
    lines: List[str] = []
    lines.append("=" * 78)
    lines.append(f"SLAM MAP EVALUATION - {os.path.basename(run.path.rstrip('/'))}")
    lines.append("=" * 78)
    scene = run.meta.get("notes") or run.meta.get("scene") or ""
    lines.append(f"grid {run.grid.shape[1]}x{run.grid.shape[0]} cells @ "
                 f"{run.resolution * 100:.1f} cm  |  origin "
                 f"({run.origin[0]:+.2f}, {run.origin[1]:+.2f}) m"
                 + (f"  |  {scene}" if scene else ""))
    lines.append("-" * 78)
    lines.append(f"{'METRIC':<22}{'VALUE':>14}  {'UNIT':<12}{'LIMIT':>10}  RESULT")
    lines.append("-" * 78)
    for m in metrics:
        val = "--" if m.value is None else f"{m.value:,.3f}"
        lim = "--" if m.threshold is None else f"{m.op} {m.threshold:g}"
        lines.append(f"{m.name:<22}{val:>14}  {m.unit:<12}{lim:>10}  {m.status}")
        if m.note:
            lines.append(f"{'':<22}{'':>14}  {m.note}")
    lines.append("-" * 78)

    failed = [m for m in metrics if m.failed]
    skipped = [m for m in metrics if m.status == "SKIP"]
    if failed:
        lines.append(f"VERDICT: FAIL - {len(failed)} metric(s) outside limits: "
                     + ", ".join(m.name for m in failed))
    else:
        graded = [m for m in metrics if m.status == "PASS"]
        lines.append(f"VERDICT: PASS - {len(graded)} metric(s) within limits")
    if skipped:
        lines.append(f"({len(skipped)} skipped for missing inputs: "
                     + ", ".join(m.name for m in skipped) + ")")
    lines.append("=" * 78)
    return "\n".join(lines)


def metrics_to_dict(run: Run, metrics: Sequence[Metric]) -> Dict:
    return {
        "run": os.path.abspath(run.path),
        "resolution_m": run.resolution,
        "grid_shape": list(run.grid.shape),
        "meta": run.meta,
        "metrics": [
            {"name": m.name, "value": m.value, "unit": m.unit,
             "threshold": m.threshold, "op": m.op, "status": m.status,
             "note": m.note}
            for m in metrics
        ],
        "verdict": "FAIL" if any(m.failed for m in metrics) else "PASS",
    }


# ─── CLI ────────────────────────────────────────────────────────────

def _cell_pair(text: str) -> Tuple[int, int]:
    try:
        col, row = text.split(",")
        return int(col), int(row)
    except Exception:
        raise argparse.ArgumentTypeError(
            f"expected COL,ROW (e.g. 412,300), got {text!r}")


def _load_mask(path: Optional[str], shape: Tuple[int, int]) -> Optional[np.ndarray]:
    if not path:
        return None
    mask = np.load(path)
    if mask.shape != shape:
        raise SystemExit(f"ground-truth mask {path} has shape {mask.shape}, "
                         f"grid is {shape}")
    return mask.astype(bool)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Numeric pass/fail evaluation of a recorded RTAB-Map "
                    "occupancy grid (see docs/slam_evaluation.md).",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run_dir", help="run directory written by map_recorder.py")
    p.add_argument("--landmark", nargs=2, type=_cell_pair, metavar="COL,ROW",
                   help="two landmark cells to measure between")
    p.add_argument("--landmark-true-m", type=float,
                   help="surveyed distance between those landmarks, metres")
    p.add_argument("--wall", nargs=2, type=_cell_pair, metavar="COL,ROW",
                   help="opposite corners of a box around one straight wall")
    p.add_argument("--corner", nargs=4, type=_cell_pair, metavar="COL,ROW",
                   help="two wall boxes (4 cells) meeting at a known 90 deg corner")
    p.add_argument("--loop", action="store_true",
                   help="trajectory returns to its start: grade closure and yaw drift")
    p.add_argument("--true-path-m", type=float,
                   help="surveyed length of the flown path, metres")
    p.add_argument("--sweep-radius-m", type=float, default=DEFAULT_SWEEP_RADIUS_M,
                   help=f"sensor horizon for coverage (default {DEFAULT_SWEEP_RADIUS_M}, "
                        "matches Grid/RangeMax)")
    p.add_argument("--occ-thresh", type=int, default=OCC_THRESH)
    p.add_argument("--free-thresh", type=int, default=FREE_THRESH)
    p.add_argument("--gt-traversable", help=".npy bool mask of truly free cells")
    p.add_argument("--gt-obstacle", help=".npy bool mask of real obstacles")
    p.add_argument("--json", dest="json_out", help="also write the report as JSON")
    p.add_argument("--quiet", action="store_true", help="suppress the table")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        run = load_run(args.run_dir)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    landmark = tuple(args.landmark) if args.landmark else None
    wall_box = None
    if args.wall:
        (c0, r0), (c1, r1) = args.wall
        wall_box = (c0, r0, c1, r1)
    corner_boxes = None
    if args.corner:
        (a0, b0), (a1, b1), (c0, d0), (c1, d1) = args.corner
        corner_boxes = ((a0, b0, a1, b1), (c0, d0, c1, d1))

    metrics = evaluate(
        run,
        landmark=landmark,
        landmark_true_m=args.landmark_true_m,
        wall_box=wall_box,
        corner_boxes=corner_boxes,
        is_loop=args.loop,
        true_path_m=args.true_path_m,
        sweep_radius_m=args.sweep_radius_m,
        occ_thresh=args.occ_thresh,
        free_thresh=args.free_thresh,
        gt_traversable=_load_mask(args.gt_traversable, run.grid.shape),
        gt_obstacle=_load_mask(args.gt_obstacle, run.grid.shape),
    )

    if not args.quiet:
        print(format_report(run, metrics))

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump(metrics_to_dict(run, metrics), fh, indent=2)
        print(f"JSON report written to {args.json_out}")

    return 1 if any(m.failed for m in metrics) else 0


if __name__ == "__main__":
    sys.exit(main())
