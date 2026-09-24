"""
================================================================================
MODULE: planner_worker.py
PURPOSE: Run A* and path collision checks off the GUI thread
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station, its own QThread
  * Upstream:      ui/slam_map_widget.py (goal planning), drone_gcs.py (live
                   collision checks and detour planning)
  * Downstream:    Qt signals carrying results back to the GUI thread

WHY THIS EXISTS - THIS IS A FLIGHT-SAFETY FIX, NOT A RESPONSIVENESS ONE:
  Both jobs used to run on the GUI thread. Measured on a 25 x 25 m room at this
  project's 2.5 cm resolution (a 1000 x 1000 grid):

      A* to a nearby point        42 ms
      A* to a far corner         334 ms
      one path collision check    34 ms, and it ran 5 times a second

  That is ~170 ms of every second blocked, plus up to a third of a second in one
  hit whenever a goal was clicked or a detour was replanned. The same GUI thread
  carries the 10 Hz OFFBOARD setpoint pump, and PX4 drops OFFBOARD if setpoints
  stop arriving for 500 ms - so a slow plan on a large map could take the
  aircraft out of OFFBOARD mid-path. Worse, set_occupancy_grid replanned on
  every incoming map message, so with a goal staged the planner ran at the map
  stream's rate.

  Nothing on the GUI thread may block behind the planner any more.

LATEST-REQUEST-WINS, NOT A QUEUE:
  Each job kind keeps exactly one pending request. A new goal click replaces the
  pending plan; a new collision check replaces the pending one. A queue would be
  actively harmful here - it would spend CPU answering questions about where the
  vehicle used to be, and the answers would arrive progressively later.

EVERY RESULT CARRIES ITS REQUEST'S TOKEN:
  The caller compares the token it gets back with the one it is waiting for and
  discards anything stale. Without that, a slow plan for an abandoned goal could
  land after a newer one and quietly replace a good path with an obsolete one.

THE GRID IS READ, NEVER WRITTEN:
  AStarPathPlanner.plan and check_path_collision do not modify the grid they are
  given (verified), and the map listener allocates a fresh array per message, so
  passing the reference across the thread boundary needs no copy.

USAGE:
  worker = PlannerWorker(planner)
  worker.plan_ready.connect(on_plan)
  worker.collision_ready.connect(on_collision)
  worker.quality_ready.connect(on_quality)
  worker.start()
  token = worker.request_plan(grid, res, ox, oy, start, goal)
================================================================================
"""

from __future__ import annotations

import itertools
import logging
import threading
import time
from typing import Optional, Sequence, Tuple

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

log = logging.getLogger("gcs.planner")


class PlannerWorker(QThread):
    """Serialises planning work onto one background thread."""

    # (token, result dict from AStarPathPlanner.plan)
    plan_ready = pyqtSignal(int, object)
    # (token, is_blocked, collision_point or None, distance_m)
    collision_ready = pyqtSignal(int, bool, object, float)
    # (token, MapQuality) - live map scoring, same thread, lowest priority
    quality_ready = pyqtSignal(int, object)
    # (job kind, milliseconds) - lets the caller see how long work actually took
    job_timing = pyqtSignal(str, float)

    def __init__(self, planner, parent=None):
        super().__init__(parent)
        self.planner = planner
        self._running = False

        # One pending request per kind; a newer one overwrites it. Guarded by
        # _lock, signalled by _wake.
        self._lock = threading.Lock()
        self._wake = threading.Condition(self._lock)
        self._pending_plan: Optional[tuple] = None
        self._pending_collision: Optional[tuple] = None
        self._pending_quality: Optional[tuple] = None
        self._tokens = itertools.count(1)

        # Stop on application shutdown as well as on the owner's closeEvent.
        # A QThread that is still running when its C++ object is destroyed
        # aborts the process - "QThread: Destroyed while thread is still
        # running" - and relying on one tidy exit path to prevent that is how
        # a crash on exit gets shipped.
        try:
            from PyQt5.QtWidgets import QApplication
            instance = QApplication.instance()
            if instance is not None:
                instance.aboutToQuit.connect(self.stop)
        except Exception:      # pragma: no cover - no QApplication in a test
            log.debug("could not hook aboutToQuit", exc_info=True)

    # ── request API (called from the GUI thread) ────────────────────

    def _next_token(self) -> int:
        return next(self._tokens)

    def request_plan(self, grid: np.ndarray, resolution: float,
                     origin_x: float, origin_y: float,
                     start: Tuple[float, float],
                     goal: Tuple[float, float]) -> int:
        """Queue a plan, replacing any plan not yet started. Returns its token."""
        token = self._next_token()
        with self._wake:
            self._pending_plan = (token, grid, resolution, origin_x, origin_y,
                                  start, goal)
            self._wake.notify()
        return token

    def request_collision_check(self, grid: np.ndarray, resolution: float,
                                origin_x: float, origin_y: float,
                                waypoints: Sequence[Tuple[float, float]],
                                current_pos: Tuple[float, float],
                                lookahead_m: float = 1.5) -> int:
        """Queue a collision check, replacing any check not yet started."""
        token = self._next_token()
        with self._wake:
            self._pending_collision = (token, grid, resolution, origin_x,
                                       origin_y, list(waypoints), current_pos,
                                       lookahead_m)
            self._wake.notify()
        return token

    def request_quality(self, grid: np.ndarray, resolution: float) -> int:
        """Queue a live map-quality score. Lowest priority of the three: it is
        information, not a decision, and must never delay a collision check."""
        token = self._next_token()
        with self._wake:
            self._pending_quality = (token, grid, resolution)
            self._wake.notify()
        return token

    def cancel_pending(self) -> None:
        """Drop anything not yet started. Work already running still finishes -
        its result is discarded by the caller's token check."""
        with self._wake:
            self._pending_plan = None
            self._pending_collision = None
            self._pending_quality = None

    def stop(self) -> None:
        """Stop the thread and wait for it. Idempotent and safe to call twice."""
        if not self.isRunning():
            self._running = False
            return
        self._running = False
        with self._wake:
            self._wake.notify_all()
        self.wait(2000)

    # ── worker loop ─────────────────────────────────────────────────

    def run(self) -> None:
        self._running = True
        while self._running:
            with self._wake:
                if (self._pending_collision is None and self._pending_plan is None
                        and self._pending_quality is None):
                    # Timed wait rather than an indefinite one so stop() is
                    # always honoured promptly.
                    #
                    # All three kinds are checked here. Testing only the first
                    # two meant that with collision checks arriving every 200 ms
                    # the loop went back to sleep for 250 ms the instant one
                    # finished, woke to find a fresh collision check waiting,
                    # and ran that instead - so a pending map score was never
                    # reached at all. Strict priority still decides what runs
                    # first; it must not decide whether the loop sleeps.
                    self._wake.wait(0.25)
                # Collision checks first: a plan is a convenience, a collision
                # check is the thing standing between a flying aircraft and a
                # wall, and it must not wait behind a 300 ms search.
                job_collision = self._pending_collision
                self._pending_collision = None
                job_plan = job_quality = None
                if job_collision is None:
                    job_plan = self._pending_plan
                    self._pending_plan = None
                    if job_plan is None:
                        job_quality = self._pending_quality
                        self._pending_quality = None

            if not self._running:
                break
            if job_collision is not None:
                self._run_collision(job_collision)
            elif job_plan is not None:
                self._run_plan(job_plan)
            elif job_quality is not None:
                self._run_quality(job_quality)

    def _run_collision(self, job) -> None:
        token, grid, res, ox, oy, waypoints, pos, lookahead = job
        started = time.perf_counter()
        try:
            blocked, point, distance = self.planner.check_path_collision(
                grid, res, ox, oy, waypoints, pos, lookahead_m=lookahead)
        except Exception:
            # A planner fault must not kill the worker; the caller's watchdog
            # sees the missing reply and the path keeps its last verdict.
            log.exception("collision check failed")
            return
        self.job_timing.emit("collision", (time.perf_counter() - started) * 1000.0)
        self.collision_ready.emit(token, bool(blocked), point, float(distance))

    def _run_plan(self, job) -> None:
        token, grid, res, ox, oy, start, goal = job
        started = time.perf_counter()
        try:
            result = self.planner.plan(grid, res, ox, oy, start, goal)
        except Exception:
            log.exception("path planning failed")
            result = {"success": False, "waypoints": [], "total_distance_m": 0.0,
                      "message": "Planner error - see the log"}
        self.job_timing.emit("plan", (time.perf_counter() - started) * 1000.0)
        self.plan_ready.emit(token, result)

    def _run_quality(self, job) -> None:
        token, grid, resolution = job
        started = time.perf_counter()
        try:
            from core.map_quality import evaluate_live
            quality = evaluate_live(grid, resolution)
        except Exception:
            log.exception("map quality scoring failed")
            return
        self.job_timing.emit("quality", (time.perf_counter() - started) * 1000.0)
        self.quality_ready.emit(token, quality)
