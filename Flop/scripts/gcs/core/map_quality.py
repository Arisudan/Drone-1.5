"""
================================================================================
MODULE: map_quality.py
PURPOSE: Live SLAM map quality scoring, from the grid alone
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station, the planner worker's thread
  * Upstream:      Each occupancy grid that arrives from the map bridge
  * Downstream:    ui/slam_map_widget.py's quality strip

WHY THIS EXISTS:
  scripts/diagnostics/map_eval.py already scores a SLAM run properly - scale
  error, wall straightness, squareness, coverage, loop-closure drift, yaw drift,
  VIO health - against the thresholds in docs/slam_evaluation.md. None of it was
  visible in the application. It ran offline, from a terminal, against a
  recorded run, after the flight. An operator watching a map build had no way to
  tell a good map from a bad one except by looking at it.

WHAT CAN AND CANNOT BE MEASURED LIVE:
  Half of map_eval's metrics need things a single grid does not contain. Scale
  error needs a known reference distance. Loop-closure gap, yaw drift and VIO
  health need the pose history of a recorded run. Those stay offline, and this
  module does not pretend otherwise.

  What a single grid does support, and what is computed here:
    * coverage      - how much of the grid has been observed at all
    * explored area - observed area in real square metres
    * frontier      - free cells touching unknown space, i.e. how much is left
                      to explore. Falling frontier means the map is converging.
    * wall RMS      - straightness of the walls, in cells, measured the way
                      map_eval measures it (total-least-squares line fits)
    * squareness    - how close the wall directions are to a right-angle pair

  The thresholds are map_eval's own, imported rather than restated, so the live
  readout and the offline report cannot disagree about what "good" means.

COST:
  Sampled, not exhaustive. The wall fits look at a bounded number of windows
  rather than every occupied cell, so the whole thing stays a few milliseconds
  on a 1000 x 1000 grid. It runs on the planner's worker thread regardless -
  nothing here may run on the GUI thread.
================================================================================
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

# map_eval lives in scripts/diagnostics; it is deliberately ROS-free and Qt-free
# so it can be imported here without dragging either in.
# core/ -> gcs/ -> scripts/ -> scripts/diagnostics
_DIAG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "diagnostics")
if _DIAG not in sys.path:
    sys.path.insert(0, _DIAG)

try:
    from map_eval import (                      # type: ignore
        classify, fit_line, occupied_points_in_box, line_angle_deg,
        THRESHOLDS, OCC_THRESH, FREE_THRESH,
    )
    HAVE_MAP_EVAL = True
except Exception:                                # pragma: no cover
    HAVE_MAP_EVAL = False
    THRESHOLDS = {"wall_rms_cells": 1.0, "squareness_err_deg": 3.0,
                  "completeness_pct": 90.0}
    OCC_THRESH, FREE_THRESH = 65, 25


# How many sample windows the wall fit looks at, and how few occupied cells a
# window may hold before it is too sparse to fit a line through honestly.
MAX_WALL_WINDOWS = 48
MIN_POINTS_PER_WINDOW = 12
WINDOW_CELLS = 24

# Spread along the fitted line divided by spread across it. Below this a
# window is a corner or a blob rather than a wall segment.
#
# 3.0 chosen by measurement, not taste. Swept against clean rooms from 5 m to
# 25 m and a deliberately noisy one: at 2.0 a 6 m room scored 3.9 degrees and
# failed a 3 degree threshold it should pass comfortably, because a short
# perimeter means corners are a large share of the samples. At 5.0 nothing
# survives on a noisy map and the straightness metric goes blind. At 3.0 every
# clean room scores 0.0 degrees and the noisy map still fails at 2.1 cells.
MIN_ELONGATION = 3.0


@dataclass
class QualityMetric:
    """One scored value. `good` is None when there is no threshold to judge it
    against - an unscored number is reported as a number, not as a pass."""
    key: str
    label: str
    value: float
    text: str
    good: Optional[bool] = None
    detail: str = ""


@dataclass
class MapQuality:
    metrics: List[QualityMetric] = field(default_factory=list)
    cells: int = 0
    resolution: float = 0.0
    ok: bool = True

    def by_key(self) -> Dict[str, QualityMetric]:
        return {m.key: m for m in self.metrics}


def _wall_geometry(occupied: np.ndarray,
                   rng: Optional[np.random.Generator] = None
                   ) -> Tuple[Optional[float], Optional[float], int]:
    """Median straightness and squareness of the walls, from sampled windows.

    Returns (rms_cells, squareness_error_deg, windows_used). Either value is
    None when there were too few usable windows to say anything - which is the
    honest answer for a map that has barely started, and better than reporting a
    confident number derived from three cells.
    """
    rows, cols = np.nonzero(occupied)
    if rows.size < MIN_POINTS_PER_WINDOW:
        return None, None, 0

    rng = rng or np.random.default_rng(12345)   # fixed: a readout that flickers
                                                # between frames is unreadable
    picks = rng.choice(rows.size, size=min(MAX_WALL_WINDOWS * 4, rows.size),
                       replace=False)

    residuals: List[float] = []
    angles: List[float] = []
    half = WINDOW_CELLS // 2
    for idx in picks:
        if len(residuals) >= MAX_WALL_WINDOWS:
            break
        r, c = int(rows[idx]), int(cols[idx])
        box = (c - half, r - half, c + half, r + half)
        pts = occupied_points_in_box(occupied, box) if HAVE_MAP_EVAL else None
        if pts is None or pts.shape[0] < MIN_POINTS_PER_WINDOW:
            continue
        try:
            _centroid, direction, rms = fit_line(pts)
        except Exception:
            continue
        # A window holding a corner is not a wall. Its fit is a 45-degree line
        # through two perpendicular segments, and both the straightness and the
        # angle it yields are meaningless. Reject anything that is not
        # convincingly line-shaped: compare the spread along the fitted
        # direction with the spread across it. A wall segment is long and thin;
        # a corner or a blob is not.
        along = pts @ direction
        elongation = float(np.std(along)) / max(rms, 1e-6)
        if elongation < MIN_ELONGATION:
            continue
        residuals.append(rms)
        angles.append(line_angle_deg(direction))

    if not residuals:
        return None, None, 0

    rms_cells = float(np.median(residuals))

    # Squareness: indoor walls should lie on one of two perpendicular
    # directions, so every angle taken modulo 90 should cluster on one value.
    # The spread around that cluster is the squareness error.
    mod90 = np.asarray(angles) % 90.0
    radians = np.radians(mod90 * 4.0)           # 90 deg -> full circle
    mean_angle = math.atan2(float(np.sin(radians).mean()),
                            float(np.cos(radians).mean()))
    centred = np.degrees(np.angle(np.exp(1j * (radians - mean_angle)))) / 4.0

    # Median absolute deviation, scaled to compare with a standard deviation.
    # Not RMS: however carefully corners are filtered, a handful survive on a
    # small room where the perimeter is short, and one 45-degree outlier in
    # forty samples moves an RMS by several degrees while leaving a median
    # untouched. Measured: a clean 12 m room scored 6.4 degrees by RMS - a
    # failure - against a 3 degree threshold it should pass comfortably.
    squareness = 1.4826 * float(np.median(np.abs(centred)))
    return rms_cells, squareness, len(residuals)


def evaluate_live(grid: np.ndarray, resolution: float) -> MapQuality:
    """Score a single occupancy grid. Pure numpy; safe on a worker thread."""
    out = MapQuality(resolution=float(resolution))
    if grid is None or grid.ndim != 2 or grid.size == 0 or resolution <= 0:
        return out

    if HAVE_MAP_EVAL:
        occupied, free, unknown = classify(grid)
    else:                                        # pragma: no cover
        g = np.asarray(grid)
        unknown = g < 0
        occupied = g >= OCC_THRESH
        free = (~unknown) & (g <= FREE_THRESH)

    cells = int(grid.size)
    out.cells = cells
    n_occ = int(occupied.sum())
    n_free = int(free.sum())
    observed = n_occ + n_free
    cell_area = resolution * resolution

    coverage_pct = 100.0 * observed / cells if cells else 0.0
    out.metrics.append(QualityMetric(
        "coverage", "Coverage", coverage_pct, f"{coverage_pct:.0f}%",
        detail="Share of the grid that has been observed at all"))

    explored_m2 = observed * cell_area
    out.metrics.append(QualityMetric(
        "explored", "Explored", explored_m2, f"{explored_m2:.1f} m²",
        detail="Observed floor area"))

    # Frontier: free cells that touch unknown space. This is what is left to
    # explore, and it is the number that tells an operator whether the map is
    # still growing or has converged.
    if n_free:
        padded = np.pad(unknown, 1, mode="constant", constant_values=False)
        touches_unknown = (padded[:-2, 1:-1] | padded[2:, 1:-1] |
                           padded[1:-1, :-2] | padded[1:-1, 2:])
        frontier_cells = int((free & touches_unknown).sum())
    else:
        frontier_cells = 0
    frontier_m = frontier_cells * resolution
    out.metrics.append(QualityMetric(
        "frontier", "Frontier", frontier_m, f"{frontier_m:.1f} m",
        detail="Edge between explored and unexplored space; falls as the map converges"))

    rms_cells, squareness, windows = _wall_geometry(occupied)
    if rms_cells is None:
        out.metrics.append(QualityMetric(
            "wall_rms", "Walls", float("nan"), "--",
            detail="Not enough wall to measure yet"))
        out.metrics.append(QualityMetric(
            "squareness", "Square", float("nan"), "--",
            detail="Not enough wall to measure yet"))
    else:
        limit = THRESHOLDS.get("wall_rms_cells", 1.0)
        out.metrics.append(QualityMetric(
            "wall_rms", "Walls", rms_cells,
            f"{rms_cells:.2f} cell" if rms_cells < 2 else f"{rms_cells:.1f} cells",
            good=rms_cells <= limit,
            detail=(f"Median straightness over {windows} wall samples; "
                    f"map_eval passes at ≤ {limit:g} cells")))
        sq_limit = THRESHOLDS.get("squareness_err_deg", 3.0)
        out.metrics.append(QualityMetric(
            "squareness", "Square", squareness, f"{squareness:.1f}°",
            good=squareness <= sq_limit,
            detail=(f"Spread of wall directions from a right-angle pair; "
                    f"map_eval passes at ≤ {sq_limit:g}°")))

    out.metrics.append(QualityMetric(
        "obstacles", "Obstacles", float(n_occ), f"{n_occ:,}",
        detail="Occupied cells in the grid"))

    out.ok = all(m.good for m in out.metrics if m.good is not None)
    return out
