"""
================================================================================
MODULE: map_canvas_render.py
PURPOSE: Paint layer of the SLAM map canvas (draw calls only, no input)
================================================================================

SLAMMapCanvas used to hold both the interaction state machine (mouse, keys,
ruler, keep-out and mission editing) and every draw call in one 2,800-line
file. The draw half lives here as a mixin: it reads canvas state through
``self`` and never mutates it, so rendering can be reasoned about (and tested)
apart from input handling.
================================================================================
"""

from __future__ import annotations

import math
from typing import Tuple

from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QPainter, QColor, QFont, QPen, QBrush, QPolygonF

from core.map_quality import OCC_THRESH
from core.map_render import (
    RVIZ_BACKGROUND, RVIZ_GRID_RGB, RVIZ_GRID_ALPHA, RVIZ_MAP_ALPHA,
    RVIZ_MAP_THIN_ALPHA, RVIZ_GRID_CELL_SIZE_M, RVIZ_GRID_PLANE_CELL_COUNT,
)
from ui.styles import PALETTE

# Drone glyph zoom behaviour. REF is the zoom (px per metre) at which the glyph
# is drawn at its designed size - the canvas default; MIN/MAX bound how far it
# follows zoom in either direction.
DRONE_GLYPH_REF_SCALE = 36.0
DRONE_GLYPH_MIN_SCALE = 0.4
DRONE_GLYPH_MAX_SCALE = 3.0

# Position-uncertainty ring. K scales 1-sigma to a 95% confidence region for a
# 2-D Gaussian (sqrt of the chi-square 95% quantile at 2 degrees of freedom):
# the vehicle is inside the drawn ellipse 95% of the time, which is the reading
# an operator actually wants ("where could it really be?"), not 1-sigma (39%).
POS_CONF_K = 2.4477

class CanvasRenderMixin:
    """Paint layer of SLAMMapCanvas: every draw call, no input handling."""

    def _uncertainty_colour(self, state: str) -> QColor:
        key = {"ok": "ok", "warn": "warn", "danger": "danger"}.get(state)
        if key is None:
            return QColor(148, 163, 184)
        return QColor(PALETTE.get(key, "#8b949e"))

    def _draw_uncertainty(self, p: QPainter, cx: float, cy: float, outline: bool = False):
        """95% position-confidence ellipse around the vehicle, in metres.

        Axis-aligned North/East because PX4 v1.17 reports only the diagonal of
        the covariance. Drawn inside the rotated scene, so it turns with the
        map. Two passes: the translucent fill goes under the airframe, the
        outline over it. The glyph is drawn larger than the real frame, so a
        ring of a healthy 10-20 cm sat entirely beneath it and was invisible -
        the good case must be as readable as the bad one.
        """
        state, _r95 = self.uncertainty_state()
        if state == "unknown":
            return
        k = POS_CONF_K
        rx = k * math.sqrt(max(self.pos_var_e, 0.0)) * self.scale   # East -> screen x
        ry = k * math.sqrt(max(self.pos_var_n, 0.0)) * self.scale   # North -> screen y
        if max(rx, ry) < 2.0:
            return                     # smaller than the glyph's own outline
        centre = self._world_to_screen(self.drone_x, self.drone_y, cx, cy)
        colour = self._uncertainty_colour(state)
        p.save()
        if outline:
            # Dark underlay first, so the ring reads on the blue fuselage and
            # on white map cells alike.
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor(13, 17, 23, 180), 3.6))
            p.drawEllipse(centre, rx, ry)
            p.setPen(QPen(colour, 1.8, Qt.DashLine if state == "stale" else Qt.SolidLine))
            p.drawEllipse(centre, rx, ry)
        elif state != "stale":
            fill = QColor(colour)
            fill.setAlpha(38)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(fill))
            p.drawEllipse(centre, rx, ry)
        p.restore()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        # Intentionally NOT setting SmoothPixmapTransform: RViz renders occupancy
        # grid cells as crisp flat quads with no interpolation. Bilinear smoothing
        # here blurs the map more the further you zoom in, since each map pixel
        # gets stretched across many screen pixels. Nearest-neighbor (the default
        # when this hint is unset) keeps cell edges sharp at any zoom level.

        w = self.width()
        h = self.height()
        cx = w / 2.0 + self.pan_x
        cy = h / 2.0 + self.pan_y

        # 1. RViz "Background Color: 48; 48; 48". The canvas used to be white,
        # which put a 100%-luminance slab inside an otherwise near-black GCS and
        # disagreed with the embedded RViz view showing the same data.
        p.fillRect(0, 0, w, h, QColor(*RVIZ_BACKGROUND))

        # 2. Planar Rotated Scene Elements
        p.save()
        p.translate(cx, cy)
        p.rotate(self.rotation_deg)
        p.translate(-cx, -cy)

        # Metric Grid Lines
        self._draw_grid(p, cx, cy, w, h)

        # Live 2D Occupancy Grid (if loaded)
        self._draw_occupancy_grid(p, cx, cy)

        # Planner inflation band, then operator keep-out zones: both are
        # "the vehicle may not go here", drawn above the map, below the route.
        self._draw_inflation(p, cx, cy)
        self._draw_keepouts(p, cx, cy)

        # Trajectory Breadcrumb Trail
        self._draw_trail(p, cx, cy)

        # Planned A* Collision-Free Path
        self._draw_planned_path(p, cx, cy)

        # Goal Pose Crosshair
        self._draw_goal_crosshair(p, cx, cy)

        # Numbered mission stops (only when there is more than one)
        self._draw_mission_stops(p, cx, cy)

        # Measure-tool polyline (drawn with the scene so it stays pinned to the
        # world as the map is rotated or panned)
        self._draw_ruler(p, cx, cy)

        # Position-uncertainty ellipse, under the airframe
        self._draw_uncertainty(p, cx, cy)

        # Drone Icon with Heading Vector
        self._draw_drone(p, cx, cy)

        # ...and the uncertainty outline over it (see _draw_uncertainty)
        self._draw_uncertainty(p, cx, cy, outline=True)

        p.restore()

        # 3. Legends, Status Badge, and Metric Scale Bar (unrotated for upright clarity)
        self._draw_overlay(p, w, h)

        # 4. Stale-map warning, on top of everything. Drawn last and deliberately
        # over the map itself rather than as another badge in a corner: the thing
        # that is wrong IS the map, and a corner indicator is exactly what an
        # operator does not look at while flying.
        self._draw_stale_warning(p, w, h)

        p.end()

    def _draw_stale_warning(self, p: QPainter, w: float, h: float):
        """Dim the map and say so when updates have stopped.

        A map that has stopped updating renders identically to one that is
        live - same walls, same colours, same everything. That is dangerous in a
        way a missing map is not: the operator keeps trusting geometry that may
        already be wrong, and clicks goals against it. The dimming is what makes
        it obvious at a glance; the countdown is what tells them how long it has
        been.
        """
        if not self.is_map_stale():
            return

        age = self.map_age_s()
        p.save()
        # Veil the whole canvas. Heavy enough to be unmistakable, light enough
        # that the map is still readable - the operator may still need to see
        # where the vehicle is.
        p.fillRect(0, 0, int(w), int(h), QColor(10, 12, 16, 150))

        amber = QColor(210, 153, 34)
        band_h = 34.0
        band_y = h * 0.5 - band_h / 2.0
        p.setPen(QPen(amber, 2))
        p.setBrush(QBrush(QColor(20, 16, 6, 230)))
        p.drawRoundedRect(QRectF(w * 0.5 - 150, band_y, 300, band_h), 6, 6)

        p.setFont(QFont("Segoe UI", 11, QFont.Bold))
        p.setPen(amber)
        p.drawText(QRectF(w * 0.5 - 150, band_y, 300, band_h), Qt.AlignCenter,
                   f"MAP STALE \u2014 no update for {age:.0f}s")
        p.restore()

    def _draw_grid(self, p: QPainter, cx: float, cy: float, w: float, h: float):
        """RViz Grid display: a finite Cell Size x Plane Cell Count plane on XY.

        RViz does not draw an unbounded grid, and neither does this any more.
        The previous version swept +/- twice the viewport in 1 m steps, which at
        low zoom meant several thousand drawLine calls per repaint at 30 Hz for
        lines that had already merged into a solid wash.
        """
        p.save()
        p.setFont(QFont("Segoe UI", 7))
        step = RVIZ_GRID_CELL_SIZE_M * self.scale
        half = RVIZ_GRID_PLANE_CELL_COUNT / 2.0
        extent = half * RVIZ_GRID_CELL_SIZE_M * self.scale

        grid_pen = QPen(QColor(*RVIZ_GRID_RGB, RVIZ_GRID_ALPHA), 1)
        p.setPen(grid_pen)
        n = int(RVIZ_GRID_PLANE_CELL_COUNT)
        for i in range(n + 1):
            off = -extent + i * step
            p.drawLine(QPointF(cx + off, cy - extent), QPointF(cx + off, cy + extent))
            p.drawLine(QPointF(cx - extent, cy + off), QPointF(cx + extent, cy + off))

        # Origin (0,0) cross, brightened for the dark background.
        p.setPen(QPen(QColor(226, 232, 240, 150), 2))
        p.drawLine(QPointF(cx - 14, cy), QPointF(cx + 14, cy))
        p.drawLine(QPointF(cx, cy - 14), QPointF(cx, cy + 14))
        p.setPen(QColor(226, 232, 240, 170))
        p.drawText(int(cx + 6), int(cy - 6), "(0,0)")
        p.restore()

    def _draw_occupancy_grid(self, p: QPainter, cx: float, cy: float):
        """Render cached occupancy grid QImage(s) properly scaled and aligned to world."""
        # 1. Background Pass: Raw Occupancy Grid (/map - explored space & obstacle mass in slight black)
        if self.display_mode in ("both", "raw") and self.map_raw_qimage is not None and self.grid_raw is not None:
            gh, gw = self.grid_raw.shape
            x_min = self.map_raw_ox
            x_max = self.map_raw_ox + (gh * self.map_raw_res)
            y_min = self.map_raw_oy
            y_max = self.map_raw_oy + (gw * self.map_raw_res)

            screen_left = cx + (y_min * self.scale)
            screen_top = cy - (x_max * self.scale)
            screen_width = (y_max - y_min) * self.scale
            screen_height = (x_max - x_min) * self.scale
            target_rect = QRectF(screen_left, screen_top, screen_width, screen_height)
            p.setOpacity(RVIZ_MAP_ALPHA)
            p.drawImage(target_rect, self.map_raw_qimage)
            p.setOpacity(1.0)

        # 2. Foreground Pass: Thin Skeleton Walls (/map_thin - crimson red 1-pixel outline)
        if self.display_mode in ("both", "thin") and self.map_thin_qimage is not None and self.grid_thin is not None:
            gh, gw = self.grid_thin.shape
            x_min = self.map_thin_ox
            x_max = self.map_thin_ox + (gh * self.map_thin_res)
            y_min = self.map_thin_oy
            y_max = self.map_thin_oy + (gw * self.map_thin_res)

            screen_left = cx + (y_min * self.scale)
            screen_top = cy - (x_max * self.scale)
            screen_width = (y_max - y_min) * self.scale
            screen_height = (x_max - x_min) * self.scale
            target_rect = QRectF(screen_left, screen_top, screen_width, screen_height)
            p.setOpacity(RVIZ_MAP_THIN_ALPHA)
            p.drawImage(target_rect, self.map_thin_qimage)
            p.setOpacity(1.0)

    def _draw_inflation(self, p: QPainter, cx: float, cy: float):
        """The planner's inflated margin, from the same mask the search uses."""
        if not self.inflation_visible or self.inflation_qimage is None or not self.inflation_geom:
            return
        img = self.inflation_qimage
        if img.isNull():
            return
        res, ox, oy = self.inflation_geom
        rows, cols = img.height(), img.width()
        target = QRectF(cx + oy * self.scale,
                        cy - (ox + rows * res) * self.scale,
                        cols * res * self.scale,
                        rows * res * self.scale)
        p.drawImage(target, img)

    def _draw_keepouts(self, p: QPainter, cx: float, cy: float):
        """Keep-out zones: red, hatched, dashed edge - unmistakably not a map
        feature - plus the in-progress drag preview."""
        danger = QColor(PALETTE.get("danger", "#f85149"))
        zones = list(self.keepout_zones)
        if not zones and self._keepout_press_px is None:
            return
        p.save()
        fill = QColor(danger)
        fill.setAlpha(45)
        hatch = QColor(danger)
        hatch.setAlpha(110)
        edge = QPen(danger, 1.8, Qt.DashLine)
        for zone in zones:
            poly = QPolygonF([self._world_to_screen(x, y, cx, cy) for x, y in zone])
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(fill))
            p.drawPolygon(poly)
            p.setBrush(QBrush(hatch, Qt.BDiagPattern))
            p.drawPolygon(poly)
            p.setPen(edge)
            p.setBrush(Qt.NoBrush)
            p.drawPolygon(poly)
            c = poly.boundingRect().center()
            label = QRectF(c.x() - 34, c.y() - 8, 68, 16)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(13, 17, 23, 200)))
            p.drawRoundedRect(label, 3, 3)
            p.setFont(QFont("Segoe UI", 7, QFont.Bold))
            p.setPen(QColor(255, 255, 255))
            p.drawText(label, Qt.AlignCenter, "KEEP-OUT")
        p.restore()

        if self._keepout_press_px is not None and self._keepout_now_px is not None:
            # The preview is drawn in screen space, un-rotated, because that
            # is the rectangle the operator is dragging.
            p.save()
            p.resetTransform()
            a, b = self._keepout_press_px, self._keepout_now_px
            rect = QRectF(QPointF(a), QPointF(b)).normalized()
            p.setPen(QPen(danger, 1.5, Qt.DashLine))
            p.setBrush(QBrush(fill))
            p.drawRect(rect)
            p.restore()

    def _draw_mission_stops(self, p: QPainter, cx: float, cy: float):
        """Numbered stop markers, so the order the vehicle will fly them is on
        the map itself and not only in the list."""
        if len(self.mission_stops) < 2:
            return
        p.save()
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        accent = QColor(PALETTE.get("accent", "#58a6ff"))
        for i, (x, y) in enumerate(self.mission_stops):
            pt = self._world_to_screen(x, y, cx, cy)
            p.setPen(QPen(QColor(226, 232, 240), 1.8))
            p.setBrush(QBrush(accent))
            p.drawEllipse(pt, 9, 9)
            p.setPen(QColor(13, 17, 23))
            p.drawText(QRectF(pt.x() - 9, pt.y() - 9, 18, 18), Qt.AlignCenter, str(i + 1))
        p.restore()

    def _draw_trail(self, p: QPainter, cx: float, cy: float):
        if not self.trail_visible or len(self.trail) < 2:
            return
        p.save()
        p.setPen(QPen(QColor(203, 213, 225, 200), 2, Qt.DashLine))
        pts = [self._world_to_screen(x, y, cx, cy) for x, y in self.trail]
        for i in range(len(pts) - 1):
            p.drawLine(pts[i], pts[i + 1])
        p.restore()

    def _draw_planned_path(self, p: QPainter, cx: float, cy: float):
        """Draw emerald-green A* planned route with waypoint nodes and wingspan safety corridor."""
        if not self.planned_waypoints:
            return

        p.save()
        drone_pt = self._world_to_screen(self.drone_x, self.drone_y, cx, cy)
        screen_pts = [drone_pt] + [self._world_to_screen(wx, wy, cx, cy) for wx, wy in self.planned_waypoints]

        # Check path clearance against occupancy grid. Wingspan radius comes
        # from the same planner instance that actually inflated the obstacles
        # for this path, so the drawn corridor and the "tight" warning below
        # can never disagree with the planning margin that produced the path.
        robot_radius_cm = self.planner.robot_radius_m * 100.0
        tight_clearance = False
        min_clearance_cm = robot_radius_cm * 2.0
        if self.occupancy_grid is not None and self.map_res > 0.001:
            gh, gw = self.occupancy_grid.shape
            for wx, wy in self.planned_waypoints:
                r = int((wx - self.map_ox) / self.map_res)
                c = int((wy - self.map_oy) / self.map_res)
                for dr in range(-5, 6):
                    for dc in range(-5, 6):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < gh and 0 <= nc < gw:
                            if self.occupancy_grid[nr, nc] >= OCC_THRESH:
                                d_cm = math.sqrt(dr*dr + dc*dc) * self.map_res * 100.0
                                if d_cm < min_clearance_cm:
                                    min_clearance_cm = d_cm
                                if d_cm < robot_radius_cm:
                                    tight_clearance = True

        # 1. Drone Wingspan Safety Corridor Tube (drawn at the planner's own
        # diameter, not a separate hardcoded figure)
        corridor_width_px = (robot_radius_cm / 100.0 * 2.0) * self.scale
        if tight_clearance:
            # Low clearance warning: Amber/Coral
            p.setPen(QPen(QColor(234, 88, 12, 60), corridor_width_px, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        else:
            # Safe corridor: Translucent Emerald Green
            p.setPen(QPen(QColor(16, 185, 129, 45), corridor_width_px, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))

        for i in range(len(screen_pts) - 1):
            p.drawLine(screen_pts[i], screen_pts[i + 1])

        # 2. Outer Glow & Core Polyline
        p.setPen(QPen(QColor(16, 185, 129, 90), 6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for i in range(len(screen_pts) - 1):
            p.drawLine(screen_pts[i], screen_pts[i + 1])

        p.setPen(QPen(QColor(5, 150, 105, 230), 2.5, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for i in range(len(screen_pts) - 1):
            p.drawLine(screen_pts[i], screen_pts[i + 1])

        # 3. Waypoint nodes and labels
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        for idx, wpt in enumerate(self.planned_waypoints):
            s_pt = self._world_to_screen(wpt[0], wpt[1], cx, cy)
            p.setBrush(QBrush(QColor(5, 150, 105)))
            p.setPen(QPen(QColor(255, 255, 255), 2))
            p.drawEllipse(s_pt, 6, 6)

            label = f"W{idx+1}" if idx < len(self.planned_waypoints) - 1 else "GOAL"
            p.setBrush(QBrush(QColor(15, 23, 42, 220)))
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(QRectF(s_pt.x() + 8, s_pt.y() - 14, 38, 16), 3, 3)
            p.setPen(QColor(255, 255, 255))
            p.drawText(int(s_pt.x() + 12), int(s_pt.y() - 2), label)

        # Clearance warning badge if tight
        if tight_clearance and self.planned_waypoints:
            last_pt = screen_pts[-1]
            p.setBrush(QBrush(QColor(220, 38, 38, 230)))
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(QRectF(last_pt.x() + 10, last_pt.y() + 6, 130, 18), 3, 3)
            p.setPen(QColor(255, 255, 255))
            p.drawText(int(last_pt.x() + 14), int(last_pt.y() + 19), f"MARGIN: {min_clearance_cm:.0f}cm")

        p.restore()

    def _draw_goal_crosshair(self, p: QPainter, cx: float, cy: float):
        """Draw target crosshair at goal pose."""
        if self.goal_pose is None:
            return

        gx, gy = self.goal_pose
        gpt = self._world_to_screen(gx, gy, cx, cy)

        p.save()
        p.setPen(QPen(QColor(217, 119, 6, 240), 2))
        p.setBrush(Qt.NoBrush)

        # Concentric rings
        p.drawEllipse(gpt, 14, 14)
        p.drawEllipse(gpt, 5, 5)
        # Crosshair lines
        p.drawLine(QPointF(gpt.x() - 18, gpt.y()), QPointF(gpt.x() + 18, gpt.y()))
        p.drawLine(QPointF(gpt.x(), gpt.y() - 18), QPointF(gpt.x(), gpt.y() + 18))

        # Coordinate label with dark background pill
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.setBrush(QBrush(QColor(15, 23, 42, 220)))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(QRectF(gpt.x() + 12, gpt.y() + 4, 115, 18), 3, 3)
        p.setPen(QColor(255, 255, 255))
        p.drawText(int(gpt.x() + 16), int(gpt.y() + 17), f"GOAL ({gx:+.2f}, {gy:+.2f})")

        p.restore()

    def _draw_drone(self, p: QPainter, cx: float, cy: float):
        """Draw the tactical drone symbol: heading, frame, FOV cone.

        One arrow-shaped fuselage carries the heading now, rather than a
        plain circle plus a second, separate floating chevron above it -
        the two used to say "forward" twice in two different visual
        languages; this says it once, unambiguously, in the accent colour
        used for aircraft-moving state everywhere else in this app.
        """
        p.save()
        d_pt = self._world_to_screen(self.drone_x, self.drone_y, cx, cy)
        p.translate(d_pt)
        p.rotate(self.heading)

        # RealSense D435i Camera FOV Cone (~87 degrees, 2.5m range forward)
        fov_len = 2.5 * self.scale
        fov_half_rad = math.radians(43.5)
        fov_dx = fov_len * math.sin(fov_half_rad)
        fov_dy = fov_len * math.cos(fov_half_rad)
        fov_cone = QPolygonF([
            QPointF(0, 0),
            QPointF(fov_dx, -fov_dy),
            QPointF(-fov_dx, -fov_dy),
        ])
        p.setPen(QPen(QColor(56, 189, 248, 140), 1, Qt.DashLine))
        p.setBrush(QBrush(QColor(56, 189, 248, 35)))
        p.drawPolygon(fov_cone)

        # The airframe scales with the map. It used to be drawn in fixed
        # pixels, so zooming in left a thumbnail-sized icon on a huge room and
        # zooming out left a large icon covering the walls around it - the one
        # symbol whose size the operator reads against the map told them
        # nothing. It is now proportional to zoom (1.0 at the default 36 px/m,
        # so the default view looks exactly as before) and clamped: below the
        # floor it becomes an unreadable dot, above the ceiling it hides the
        # cells the operator zoomed in to inspect. Pens scale with it, so line
        # weights stay in proportion. The FOV cone above is already in metres
        # and is deliberately left outside this transform.
        k = max(DRONE_GLYPH_MIN_SCALE,
                min(DRONE_GLYPH_MAX_SCALE, self.scale / DRONE_GLYPH_REF_SCALE))
        p.scale(k, k)

        glyph_r = 11.0

        # Soft halo behind the arrow so it reads clearly against a busy
        # occupancy grid instead of blending into it - the one piece of the
        # old airframe glyph kept, since "clearly visible" was an explicit
        # requirement and the arrow alone is small.
        p.setPen(Qt.NoPen)
        halo = QColor(PALETTE["accent"])
        halo.setAlpha(30)
        p.setBrush(QBrush(halo))
        p.drawEllipse(QPointF(0, 0), glyph_r + 7, glyph_r + 7)

        # A single small heading arrow (the Google-Maps/QGroundControl style
        # "location arrow" dart) replaces the simulated quad airframe - no
        # frame, no arms, no per-motor colour coding to misread as real motor
        # status. One shape, unambiguous heading, nothing else to interpret.
        arrow = QPolygonF([
            QPointF(0, -glyph_r),               # nose
            QPointF(glyph_r * 0.62, glyph_r * 0.75),   # right wingtip
            QPointF(0, glyph_r * 0.35),          # tail notch (concave back)
            QPointF(-glyph_r * 0.62, glyph_r * 0.75),  # left wingtip
        ])
        p.setPen(QPen(QColor(226, 232, 240), 1.5))
        p.setBrush(QBrush(QColor(PALETTE["accent_bright"])))
        p.drawPolygon(arrow)

        p.restore()

    def _draw_ruler(self, p: QPainter, cx: float, cy: float):
        """Dashed measure polyline with a per-segment length and a running total.

        Deliberately drawn in amber and dashed, so it can never be mistaken for
        the solid planned A* path in nav blue. The two appear on the same canvas
        and one of them is a flight instruction.
        """
        if not self.ruler_points:
            return

        points = list(self.ruler_points)
        live = self.ruler_cursor if self.ruler_active else None
        if live is not None:
            points.append(live)

        amber = QColor(210, 153, 34)
        screen = [self._world_to_screen(x, y, cx, cy) for x, y in points]

        p.save()
        pen = QPen(amber, 1.8, Qt.DashLine)
        pen.setDashPattern([5, 4])
        p.setPen(pen)
        for a, b in zip(screen, screen[1:]):
            p.drawLine(a, b)

        # Per-segment length, placed at the segment midpoint. Counter-rotated so
        # the text stays upright whatever the map rotation is.
        p.setFont(QFont("Segoe UI", 7, QFont.Bold))
        for (wa, wb), (sa, sb) in zip(zip(points, points[1:]), zip(screen, screen[1:])):
            seg = math.hypot(wb[0] - wa[0], wb[1] - wa[1])
            if seg < 0.02:
                continue
            mid = QPointF((sa.x() + sb.x()) / 2.0, (sa.y() + sb.y()) / 2.0)
            p.save()
            p.translate(mid)
            p.rotate(-self.rotation_deg)
            label = f"{seg:.2f} m"
            p.setPen(QColor(15, 20, 26, 220))
            p.drawText(QRectF(-38, -20, 76, 14), Qt.AlignCenter, label)
            p.setPen(amber)
            p.drawText(QRectF(-39, -21, 76, 14), Qt.AlignCenter, label)
            p.restore()

        # Vertices
        p.setPen(QPen(amber, 1.5))
        p.setBrush(QBrush(QColor(15, 20, 26)))
        for i, pt_ in enumerate(screen):
            if live is not None and i == len(screen) - 1:
                continue
            p.drawEllipse(pt_, 3.5, 3.5)
        p.restore()

    def _pick_scale_span(self) -> Tuple[float, float]:
        """Choose the round distance whose on-screen length is closest to the
        target width. Returns (metres, pixels)."""
        if self.scale <= 0:
            return 1.0, self.SCALE_TARGET_PX
        span = min(self.SCALE_SPANS_M,
                   key=lambda m: abs(m * self.scale - self.SCALE_TARGET_PX))
        return span, span * self.scale

    @staticmethod
    def _format_span(span_m: float) -> str:
        """Sub-metre spans read in centimetres: '25 cm' beats '0.25 Meter' on a
        2.5 cm occupancy grid, where centimetres are the working unit."""
        if span_m < 1.0:
            return f"{span_m * 100:.0f} cm"
        if float(span_m).is_integer():
            return f"{int(span_m)} m"
        return f"{span_m:g} m"

    def _draw_overlay(self, p: QPainter, w: float, h: float):
        """Draw UI overlays: metric scale, pose legend, and active map indicator."""
        p.save()

        # 1. Adaptive Metric Scale Bar.
        # The bar used to be hardcoded to exactly one metre of screen distance,
        # which meant it was a handful of pixels wide zoomed out over a whole
        # floor and ran off the side of the canvas zoomed in on a doorway - in
        # both cases telling the operator nothing. It now picks a round distance
        # from a 1-2-5 sequence so the bar stays a readable width at any zoom,
        # and states which distance it picked.
        span_m, bar_len = self._pick_scale_span()
        # Light ink from here down: the canvas is RViz's 48,48,48, so the
        # previous near-black overlay colours were invisible on it.
        # The whole block sits a line higher than it used to: the bar's end
        # ticks reached down to h-17 while the controls hint below it was drawn
        # on the h-8 baseline, so the tick and the "L" of "Left Click" touched.
        p.setPen(QPen(QColor(226, 232, 240, 220), 2))
        p.drawLine(QPointF(20, h - 32), QPointF(20 + bar_len, h - 32))
        p.drawLine(QPointF(20, h - 37), QPointF(20, h - 27))
        p.drawLine(QPointF(20 + bar_len, h - 37), QPointF(20 + bar_len, h - 27))
        # Mid tick: halves the bar by eye, which is how a scale bar is actually
        # read when estimating a distance that is not a whole multiple.
        p.setPen(QPen(QColor(226, 232, 240, 150), 1))
        p.drawLine(QPointF(20 + bar_len / 2.0, h - 35), QPointF(20 + bar_len / 2.0, h - 29))

        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.setPen(QColor(226, 232, 240, 220))
        p.drawText(20, int(h - 42), self._format_span(span_m))

        # Zoom readout, beside the bar: the scale bar says how long a distance
        # is, this says how zoomed in the view is, and they answer different
        # questions when comparing two screenshots of the same room.
        p.setFont(QFont("Segoe UI", 7))
        p.setPen(QColor(148, 163, 184, 200))
        p.drawText(int(24 + bar_len), int(h - 42), f"{self.scale:.0f} px/m")

        # 2. Controls Hint
        p.setFont(QFont("Segoe UI", 7))
        p.setPen(QColor(148, 163, 184, 200))
        if self.ruler_active:
            total = self.ruler_total_m()
            hint = (f"MEASURING - Left Click: add point | Esc: clear | "
                    f"total {total:.2f} m over {max(0, len(self.ruler_points) - 1)} segment(s)")
            p.setPen(QColor(210, 153, 34, 230))
        else:
            hint = ("Left Click: Set Goal | Right-Click: Menu | "
                    "Right-Drag: Pan | Scroll: Zoom")
        p.drawText(20, int(h - 12), hint)

        # 3. Pose and position-uncertainty text moved to the status strip under
        # the map (SLAMMapWidget._build_status_strip): here they covered the map.

        # 4. Tactical Orientation Compass Rose (Top-Right)
        comp_cx = w - 50
        comp_cy = 45
        p.save()
        p.translate(comp_cx, comp_cy)
        # Rotate opposite to map rotation so N always points to true physical North!
        p.rotate(-self.rotation_deg)

        # Outer ring
        p.setPen(QPen(QColor(148, 163, 184, 170), 1.5))
        p.setBrush(QBrush(QColor(22, 27, 34, 220)))
        p.drawEllipse(QPointF(0, 0), 22, 22)

        # North pointer (Crimson Red triangle)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(220, 38, 38)))
        p.drawPolygon(QPolygonF([QPointF(0, -20), QPointF(5, -6), QPointF(-5, -6)]))

        # South pointer (Dark Slate triangle)
        p.setBrush(QBrush(QColor(148, 163, 184)))
        p.drawPolygon(QPolygonF([QPointF(0, 20), QPointF(5, 6), QPointF(-5, 6)]))

        # Center pivot
        p.setBrush(QBrush(QColor(226, 232, 240)))
        p.drawEllipse(QPointF(0, 0), 3, 3)

        # North label
        p.setFont(QFont("Segoe UI", 7, QFont.Bold))
        p.setPen(QColor(220, 38, 38))
        p.drawText(QRectF(-10, -32, 20, 12), Qt.AlignCenter, "N")

        p.restore()
        p.restore()
