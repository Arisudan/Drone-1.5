"""
================================================================================
MODULE: mission_progress.py
PURPOSE: Live route progress strip for an executing A* path
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (Tactical SLAM Workspace)
  * Upstream:      drone_gcs.py's GUI tick, which already owns active_waypoints,
                   current_wpt_idx and the live position
  * Downstream:    Display only. This widget commands nothing and decides
                   nothing; it cannot advance a waypoint or end a path.

WHY THIS EXISTS:
  While a path was flying, the only progress indication was log lines scrolling
  past in the console ("Reached Waypoint W3") and a distance figure buried in
  the map overlay. There was no way to answer the two questions an operator
  actually has in the air - how much further, and how much longer - without
  reading back through a scrollback that is also carrying collision warnings.

ETA IS FLOORED, NOT INSTANTANEOUS:
  Remaining distance divided by current ground speed is meaningless at the exact
  moment the vehicle is stationary over a waypoint: it produces infinity, and
  then a wildly different number a tenth of a second later. The speed used here
  is a moving average with a floor, and when the vehicle is genuinely not moving
  the ETA reads "--" rather than a fabricated figure. A confident wrong number
  is worse than an honest blank.

DISTANCE REMAINING IS ALONG THE ROUTE, NOT STRAIGHT-LINE:
  Summed over the unflown legs, because the path is the thing that avoids the
  walls. Straight-line distance to the goal through three rooms would read as
  encouraging progress right up until it did not.

USAGE:
  strip = MissionProgressBar(parent)
  strip.set_route(waypoints)
  strip.update_progress(idx, (x, y), ground_speed, state="EN ROUTE")
  strip.clear()
================================================================================
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import List, Optional, Sequence, Tuple

from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
from PyQt5.QtWidgets import QWidget, QFrame, QLabel, QHBoxLayout, QVBoxLayout, QSizePolicy

from ui.styles import PALETTE
from ui.scaling import px

# State -> (caption colour, whether the route bar animates as "live")
STATE_COLOURS = {
    "STAGED":        PALETTE["text_dim"],
    "CLIMBING":      PALETTE["warn"],
    "EN ROUTE":      PALETTE["nav"],
    "PAUSED":        PALETTE["warn"],
    "OBSTACLE HOLD": PALETTE["danger"],
    "COMPLETE":      PALETTE["ok"],
    "ABORTED":       PALETTE["danger"],
}

# Below this the vehicle is hovering or repositioning, not making progress
# along the route, and an ETA computed from it would be fiction.
MIN_ETA_SPEED_MPS = 0.08


class RouteBar(QWidget):
    """The painted route: one pip per waypoint, filled as each is reached."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumHeight(px(26))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.total_waypoints = 0
        self.reached = 0
        self.leg_fraction = 0.0     # progress across the leg being flown, 0..1
        self.state = "STAGED"

    def configure(self, total: int) -> None:
        self.total_waypoints = max(0, int(total))
        self.reached = 0
        self.leg_fraction = 0.0
        self.update()

    def set_progress(self, reached: int, leg_fraction: float, state: str) -> None:
        self.reached = max(0, min(self.total_waypoints, int(reached)))
        self.leg_fraction = max(0.0, min(1.0, float(leg_fraction)))
        self.state = state
        self.update()

    def paintEvent(self, event):
        if self.total_waypoints <= 0:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        margin = px(10)
        cy = h / 2.0
        track_w = max(1.0, w - 2 * margin)
        accent = QColor(STATE_COLOURS.get(self.state, PALETTE["nav"]))

        # Track
        p.setPen(QPen(QColor(PALETTE["border_soft"]), max(1, px(3)), Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(margin, cy), QPointF(margin + track_w, cy))

        # Completed portion. One waypoint is a degenerate route with no legs, so
        # the fraction is taken over max(1, n-1) rather than n-1.
        legs = max(1, self.total_waypoints - 1)
        done = min(1.0, (self.reached + self.leg_fraction) / legs) if legs else 0.0
        if done > 0.001:
            p.setPen(QPen(accent, max(1, px(3)), Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(margin, cy), QPointF(margin + track_w * done, cy))

        # Pips. Above about 40 waypoints they merge into a solid line and cost
        # more to draw than they convey, so only the endpoints are marked.
        pip_r = px(4)
        if self.total_waypoints <= 40:
            for i in range(self.total_waypoints):
                frac = i / legs if legs else 0.0
                x = margin + track_w * min(1.0, frac)
                passed = i <= self.reached
                p.setPen(QPen(accent if passed else QColor(PALETTE["border_hover"]),
                              max(1, px(1))))
                p.setBrush(QBrush(accent if passed else QColor(PALETTE["bg_panel"])))
                p.drawEllipse(QPointF(x, cy), pip_r, pip_r)
        else:
            for frac in (0.0, 1.0):
                x = margin + track_w * frac
                p.setPen(QPen(QColor(PALETTE["border_hover"]), max(1, px(1))))
                p.setBrush(QBrush(QColor(PALETTE["bg_panel"])))
                p.drawEllipse(QPointF(x, cy), pip_r, pip_r)

        # Live vehicle marker
        vx = margin + track_w * done
        p.setPen(QPen(QColor(PALETTE["text_bright"]), max(1, px(2))))
        p.setBrush(QBrush(accent))
        marker = px(6)
        p.drawEllipse(QPointF(vx, cy), marker, marker)
        p.end()


class MissionProgressBar(QFrame):
    """Route bar plus the numbers: waypoint index, distances, ETA, elapsed."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setProperty("class", "cardFrame")

        self._waypoints: List[Tuple[float, float]] = []
        self._leg_lengths: List[float] = []
        self._start_time: float = 0.0
        self._state = "STAGED"
        # Ground speed is noisy at 10 Hz; a short moving average keeps the ETA
        # from flickering between two values a second apart.
        self._speed_window: deque = deque(maxlen=20)

        root = QVBoxLayout(self)
        root.setContentsMargins(px(10), px(6), px(10), px(7))
        root.setSpacing(px(4))

        head = QHBoxLayout()
        head.setSpacing(px(10))

        self.lbl_state = QLabel("STAGED", self)
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setFixedWidth(px(116))
        head.addWidget(self.lbl_state)

        self.lbl_index = QLabel("W -- / --", self)
        self.lbl_index.setObjectName("valueBright")
        head.addWidget(self.lbl_index)

        head.addStretch()

        self._readouts = {}
        for key, caption in (("next", "NEXT"), ("remaining", "REMAINING"),
                             ("eta", "ETA"), ("elapsed", "ELAPSED")):
            cell = QHBoxLayout()
            cell.setSpacing(px(5))
            cap = QLabel(caption, self)
            cap.setObjectName("execAxis")
            cell.addWidget(cap)
            val = QLabel("--", self)
            val.setObjectName("execReadout")
            cell.addWidget(val)
            self._readouts[key] = val
            head.addLayout(cell)
            head.addSpacing(px(4))

        root.addLayout(head)

        self.route = RouteBar(self)
        root.addWidget(self.route)

        self._apply_state_style()
        self.hide()

    # ── route lifecycle ─────────────────────────────────────────────

    def set_route(self, waypoints: Sequence[Tuple[float, float]],
                  origin: Optional[Tuple[float, float]] = None,
                  state: str = "STAGED") -> None:
        """Adopt a new route. `origin` is the vehicle position the first leg
        starts from, so the first leg's length is real rather than zero."""
        self._waypoints = [(float(x), float(y)) for x, y in waypoints]
        self._leg_lengths = []
        if self._waypoints:
            previous = origin if origin is not None else self._waypoints[0]
            for wp in self._waypoints:
                self._leg_lengths.append(math.dist(previous, wp))
                previous = wp
        self._speed_window.clear()
        self._start_time = time.time()
        self.route.configure(len(self._waypoints))
        self.set_state(state)
        self.setVisible(bool(self._waypoints))

    def clear(self) -> None:
        self._waypoints = []
        self._leg_lengths = []
        self._speed_window.clear()
        self._start_time = 0.0
        self.route.configure(0)
        self.hide()

    def set_state(self, state: str) -> None:
        if state == self._state:
            return
        self._state = state
        self.lbl_state.setText(state)
        self._apply_state_style()
        self.route.state = state
        self.route.update()

    def _apply_state_style(self) -> None:
        colour = STATE_COLOURS.get(self._state, PALETTE["text_dim"])
        self.lbl_state.setStyleSheet(
            f"color: {colour}; border: 1px solid {colour}; border-radius: {px(3)}px;"
            f" padding: {px(2)}px {px(6)}px; font-weight: bold;"
            f" letter-spacing: 0.8px; font-size: {px(10)}px;")

    # ── live update ─────────────────────────────────────────────────

    def update_progress(self, current_index: int, position: Tuple[float, float],
                        ground_speed: float, state: Optional[str] = None) -> None:
        """Refresh from the main window's already-computed navigation state."""
        if state is not None:
            self.set_state(state)
        if not self._waypoints:
            return

        n = len(self._waypoints)
        idx = max(0, min(n - 1, int(current_index)))
        target = self._waypoints[idx]
        dist_next = math.dist(position, target)

        # Everything after the leg in progress is known exactly; only the
        # current leg depends on where the vehicle is right now.
        remaining = dist_next + sum(self._leg_lengths[idx + 1:])

        leg_len = self._leg_lengths[idx] if idx < len(self._leg_lengths) else 0.0
        leg_fraction = 0.0
        if leg_len > 1e-3:
            leg_fraction = max(0.0, min(1.0, 1.0 - dist_next / leg_len))

        self._speed_window.append(max(0.0, float(ground_speed)))
        avg_speed = sum(self._speed_window) / len(self._speed_window)

        self._readouts["next"].setText(f"{dist_next:.2f} m")
        self._readouts["remaining"].setText(f"{remaining:.2f} m")

        if avg_speed >= MIN_ETA_SPEED_MPS and remaining > 0.05:
            eta = remaining / avg_speed
            self._readouts["eta"].setText(self._hms(eta))
        else:
            self._readouts["eta"].setText("--")

        if self._start_time:
            self._readouts["elapsed"].setText(self._hms(time.time() - self._start_time))

        # Waypoints are 1-indexed for the operator, matching the console's
        # "Reached Waypoint W3" and the dispatch log lines.
        self.lbl_index.setText(f"W {idx + 1} / {n}")
        self.route.set_progress(idx, leg_fraction, self._state)

    def mark_complete(self) -> None:
        self.set_state("COMPLETE")
        n = len(self._waypoints)
        if n:
            self.lbl_index.setText(f"W {n} / {n}")
            self.route.set_progress(n, 1.0, self._state)
        self._readouts["next"].setText("0.00 m")
        self._readouts["remaining"].setText("0.00 m")
        self._readouts["eta"].setText("--")

    @staticmethod
    def _hms(seconds: float) -> str:
        seconds = max(0.0, float(seconds))
        if seconds >= 3600:
            return f"{int(seconds // 3600)}h{int((seconds % 3600) // 60):02d}m"
        if seconds >= 60:
            return f"{int(seconds // 60)}m{int(seconds % 60):02d}s"
        return f"{seconds:.0f}s"
