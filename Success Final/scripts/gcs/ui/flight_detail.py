"""
================================================================================
MODULE: flight_detail.py
PURPOSE: Flight-detail view and battery-health trend charts for the Logs tab
================================================================================

WHY THIS EXISTS:
  The Logs tab used to be a table of one-line summaries - how long, how far, how
  much battery - and nothing to open. The questions that matter after a flight
  are "what did the altitude and speed actually do?", "where did it go?" and
  "is this battery getting worse?". FlightLogger now records a time series per
  flight (core/flight_log.py); this module draws it, with no plotting library
  (QPainter only), in the station's restrained palette: grey grid, one accent
  line, colour never used for decoration.

WHAT IS HERE:
  SeriesChart          one time series with axes, ticks and a hover read-out
  PathMap              the flight's ground track, north up, with a scale bar
  TrendChart           one value per flight across flights (battery health)
  FlightDetailDialog   summary + four charts + the path, with CSV export

The pure helpers at the top (ticks, clock text, bounds, nearest sample) have no
Qt in them and are unit-tested directly.
================================================================================
"""

from __future__ import annotations

import bisect
import csv
import math
from typing import List, Optional, Sequence, Tuple

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QDialog, QFileDialog, QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QSizePolicy, QVBoxLayout, QWidget,
)

from core.flight_log import FlightRecord
from ui.scaling import px

BG = QColor("#161b22")
GRID = QColor("#262c34")
AXIS = QColor("#30363d")
TEXT = QColor("#8b949e")
BRIGHT = QColor("#c9d1d9")
ACCENT = QColor("#58a6ff")
MUTED_LINE = QColor("#6e7681")


# ───────────────────────────── pure helpers ────────────────────────────────

def nice_ticks(lo: float, hi: float, target: int = 4) -> List[float]:
    """Round tick values spanning [lo, hi], about `target` of them (1-2-5 steps)."""
    if hi < lo:
        lo, hi = hi, lo
    if hi - lo < 1e-12:
        return [lo]
    raw = (hi - lo) / max(1, target)
    mag = 10 ** math.floor(math.log10(raw))
    step = mag
    for m in (1, 2, 5, 10):
        step = m * mag
        if step >= raw:
            break
    first = math.ceil(lo / step) * step
    ticks, v = [], first
    while v <= hi + step * 1e-9:
        ticks.append(round(v, 10))
        v += step
    return ticks


def clock_text(seconds: float) -> str:
    """m:ss, or h:mm:ss from an hour up."""
    total = max(0, int(round(seconds)))
    h, rem = divmod(total, 3600)
    m, s = divmod(rem, 60)
    return f"{h}:{m:02d}:{s:02d}" if h else f"{m}:{s:02d}"


def padded_range(values: Sequence[float], floor_zero: bool = False,
                 min_span: float = 1e-6) -> Tuple[float, float]:
    """(lo, hi) around the data with a little headroom; never a zero-height range."""
    vals = [v for v in values if v is not None and not math.isnan(v)]
    if not vals:
        return 0.0, 1.0
    lo, hi = min(vals), max(vals)
    if hi - lo < min_span:
        pad = max(abs(hi) * 0.1, 0.5)
        lo, hi = lo - pad, hi + pad
    else:
        pad = (hi - lo) * 0.08
        lo, hi = lo - pad, hi + pad
    if floor_zero and min(vals) >= 0:
        lo = max(0.0, lo)
    return lo, hi


def nearest_index(xs: Sequence[float], x: float) -> int:
    """Index of the sample closest to x in a sorted sequence (-1 if empty)."""
    if not xs:
        return -1
    i = bisect.bisect_left(xs, x)
    if i <= 0:
        return 0
    if i >= len(xs):
        return len(xs) - 1
    return i if (xs[i] - x) < (x - xs[i - 1]) else i - 1


def path_bounds(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float, float, float]:
    """(min_x, max_x, min_y, max_y) of a ground track."""
    return min(xs), max(xs), min(ys), max(ys)


def scale_bar_length(extent_m: float) -> float:
    """A round length (1-2-5 sequence) about a fifth of the track's extent."""
    if extent_m <= 0:
        return 1.0
    ticks = nice_ticks(0.0, extent_m / 2.5, 1)
    return max([t for t in ticks if t > 0] or [1.0])


def series_rows(rec: FlightRecord) -> List[List[float]]:
    """Rows [t, alt, speed, batt, volt, x, y] for a CSV export."""
    s = rec.series
    n = len(s.get("t", []))
    cols = ("t", "alt", "speed", "batt", "volt", "x", "y")
    return [[s.get(c, [0.0] * n)[i] if i < len(s.get(c, [])) else "" for c in cols]
            for i in range(n)]


# ───────────────────────────── charts ──────────────────────────────────────

class SeriesChart(QWidget):
    """One time series: titled, with ticks, grid and a hover read-out."""

    def __init__(self, title: str, unit: str, parent=None, floor_zero: bool = False):
        super().__init__(parent)
        self.title, self.unit, self.floor_zero = title, unit, floor_zero
        self.ts: List[float] = []
        self.vs: List[float] = []
        self._hover: Optional[int] = None
        self.setMinimumSize(px(240), px(130))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)

    def set_data(self, ts: Sequence[float], vs: Sequence[float]) -> None:
        n = min(len(ts), len(vs))
        self.ts, self.vs = list(ts[:n]), list(vs[:n])
        self._hover = None
        self.update()

    # geometry shared by painting and hit-testing
    def _plot(self) -> QRectF:
        return QRectF(px(44), px(26), max(10, self.width() - px(44) - px(12)),
                      max(10, self.height() - px(26) - px(22)))

    def _x_of(self, t: float, r: QRectF) -> float:
        t0, t1 = self.ts[0], self.ts[-1]
        return r.left() + (t - t0) / max(1e-9, t1 - t0) * r.width()

    def _y_of(self, v: float, r: QRectF, lo: float, hi: float) -> float:
        return r.bottom() - (v - lo) / max(1e-9, hi - lo) * r.height()

    def mouseMoveEvent(self, ev):
        if len(self.ts) >= 2:
            r = self._plot()
            frac = (ev.pos().x() - r.left()) / max(1.0, r.width())
            t = self.ts[0] + max(0.0, min(1.0, frac)) * (self.ts[-1] - self.ts[0])
            self._hover = nearest_index(self.ts, t)
            self.update()

    def leaveEvent(self, ev):
        self._hover = None
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), BG)
        p.setFont(QFont("Ubuntu", 9, QFont.Bold))
        p.setPen(BRIGHT)
        p.drawText(QPointF(px(10), px(16)), f"{self.title}")
        if len(self.ts) < 2:
            p.setPen(TEXT)
            p.setFont(QFont("Ubuntu", 9))
            p.drawText(self.rect(), Qt.AlignCenter, "no data recorded")
            p.end()
            return

        r = self._plot()
        lo, hi = padded_range(self.vs, self.floor_zero)
        p.setFont(QFont("Noto Sans Mono", 7))
        for v in nice_ticks(lo, hi, 4):
            y = self._y_of(v, r, lo, hi)
            if not r.top() - 1 <= y <= r.bottom() + 1:
                continue
            p.setPen(QPen(GRID, 1))
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y))
            p.setPen(TEXT)
            p.drawText(QRectF(0, y - 7, r.left() - px(4), 14), Qt.AlignRight | Qt.AlignVCenter,
                       f"{v:g}")
        for t in nice_ticks(self.ts[0], self.ts[-1], 5):
            x = self._x_of(t, r)
            p.setPen(QPen(GRID, 1))
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))
            p.setPen(TEXT)
            p.drawText(QRectF(x - 24, r.bottom() + 3, 48, 14), Qt.AlignCenter, clock_text(t))
        p.setPen(QPen(AXIS, 1))
        p.drawRect(r)

        poly = QPolygonF([QPointF(self._x_of(t, r), self._y_of(v, r, lo, hi))
                          for t, v in zip(self.ts, self.vs)])
        p.setPen(QPen(ACCENT, max(1.5, px(2))))
        p.setBrush(Qt.NoBrush)
        p.drawPolyline(poly)

        # right-hand summary: the unit, and the min / max of the flight
        p.setPen(TEXT)
        p.setFont(QFont("Ubuntu", 8))
        p.drawText(QRectF(0, px(4), self.width() - px(10), px(16)), Qt.AlignRight | Qt.AlignVCenter,
                   f"{self.unit}   min {min(self.vs):.3g}  max {max(self.vs):.3g}")

        if self._hover is not None and 0 <= self._hover < len(self.ts):
            i = self._hover
            x, y = self._x_of(self.ts[i], r), self._y_of(self.vs[i], r, lo, hi)
            p.setPen(QPen(MUTED_LINE, 1, Qt.DashLine))
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))
            p.setPen(Qt.NoPen)
            p.setBrush(BRIGHT)
            p.drawEllipse(QPointF(x, y), 3.5, 3.5)
            label = f"{clock_text(self.ts[i])}  {self.vs[i]:g} {self.unit}"
            p.setFont(QFont("Noto Sans Mono", 8, QFont.Bold))
            w = p.fontMetrics().horizontalAdvance(label) + 12
            bx = min(max(r.left(), x - w / 2), r.right() - w)
            p.setBrush(QColor("#0d1117"))
            p.setPen(QPen(AXIS, 1))
            p.drawRoundedRect(QRectF(bx, r.top() + 4, w, 18), 3, 3)
            p.setPen(BRIGHT)
            p.drawText(QRectF(bx, r.top() + 4, w, 18), Qt.AlignCenter, label)
        p.end()


class PathMap(QWidget):
    """The flight's ground track, north up, equal aspect, with start/end marks."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.xs: List[float] = []      # north
        self.ys: List[float] = []      # east
        self.setMinimumSize(px(240), px(240))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_track(self, north: Sequence[float], east: Sequence[float]) -> None:
        n = min(len(north), len(east))
        self.xs, self.ys = list(north[:n]), list(east[:n])
        self.update()

    def project(self, x_north: float, y_east: float, rect: QRectF,
                bounds: Tuple[float, float, float, float]) -> QPointF:
        """World (north, east) -> screen, equal scale on both axes, centred in rect."""
        mn, mx, me, xe = bounds
        span_n, span_e = max(mx - mn, 1e-6), max(xe - me, 1e-6)
        scale = min(rect.width() / span_e, rect.height() / span_n)
        cx, cy = rect.center().x(), rect.center().y()
        mid_e, mid_n = (me + xe) / 2.0, (mn + mx) / 2.0
        return QPointF(cx + (y_east - mid_e) * scale, cy - (x_north - mid_n) * scale)

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), BG)
        p.setFont(QFont("Ubuntu", 9, QFont.Bold))
        p.setPen(BRIGHT)
        p.drawText(QPointF(px(10), px(16)), "Path (top-down, north up)")
        if len(self.xs) < 2:
            p.setPen(TEXT)
            p.setFont(QFont("Ubuntu", 9))
            p.drawText(self.rect(), Qt.AlignCenter, "no position recorded")
            p.end()
            return
        area = QRectF(px(18), px(30), self.width() - px(36), self.height() - px(30) - px(28))
        bounds = path_bounds(self.xs, self.ys)
        extent = max(bounds[1] - bounds[0], bounds[3] - bounds[2])
        if extent < 0.2:
            p.setPen(TEXT)
            p.setFont(QFont("Ubuntu", 9))
            p.drawText(self.rect(), Qt.AlignCenter, "no movement (hover only)")
            p.end()
            return
        pts = [self.project(x, y, area, bounds) for x, y in zip(self.xs, self.ys)]
        p.setPen(QPen(ACCENT, max(1.5, px(2)), Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        p.drawPolyline(QPolygonF(pts))
        # start: hollow ring; end: filled dot
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(BRIGHT, 2))
        p.drawEllipse(pts[0], 6, 6)
        p.setBrush(BRIGHT)
        p.setPen(Qt.NoPen)
        p.drawEllipse(pts[-1], 4.5, 4.5)
        p.setFont(QFont("Ubuntu", 8))
        p.setPen(TEXT)
        p.drawText(pts[0] + QPointF(9, -7), "start")
        p.drawText(pts[-1] + QPointF(9, 14), "end")
        # scale bar
        span = scale_bar_length(extent)
        scale = min(area.width() / max(bounds[3] - bounds[2], 1e-6),
                    area.height() / max(bounds[1] - bounds[0], 1e-6))
        bar = span * scale
        y = self.height() - px(14)
        p.setPen(QPen(TEXT, 2))
        p.drawLine(QPointF(px(18), y), QPointF(px(18) + bar, y))
        p.setFont(QFont("Noto Sans Mono", 8))
        p.drawText(QPointF(px(18) + bar + 6, y + 4), f"{span:g} m")
        p.end()


class TrendChart(QWidget):
    """One value per flight, oldest to newest: is it getting better or worse?"""

    def __init__(self, title: str, unit: str, parent=None, decimals: int = 1):
        super().__init__(parent)
        self.title, self.unit, self.decimals = title, unit, decimals
        self.values: List[Optional[float]] = []
        self.setMinimumSize(px(200), px(110))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_values(self, values: Sequence[Optional[float]]) -> None:
        self.values = list(values)
        self.update()

    def summary(self) -> str:
        """'latest 3.2 %/min (avg 2.8)' - or why there is nothing to say."""
        real = [v for v in self.values if v is not None]
        if not real:
            return "no data"
        avg = sum(real) / len(real)
        return f"latest {real[-1]:.{self.decimals}f} {self.unit}   avg {avg:.{self.decimals}f}"

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), BG)
        p.setFont(QFont("Ubuntu", 9, QFont.Bold))
        p.setPen(BRIGHT)
        p.drawText(QPointF(px(10), px(16)), self.title)
        p.setFont(QFont("Ubuntu", 8))
        p.setPen(TEXT)
        p.drawText(QRectF(0, px(4), self.width() - px(10), px(16)), Qt.AlignRight | Qt.AlignVCenter,
                   self.summary())
        pts = [(i, v) for i, v in enumerate(self.values) if v is not None]
        r = QRectF(px(36), px(28), max(10, self.width() - px(36) - px(12)), max(10, self.height() - px(28) - px(14)))
        if len(pts) < 2:
            p.setPen(TEXT)
            p.drawText(self.rect(), Qt.AlignCenter, "needs 2+ flights")
            p.end()
            return
        lo, hi = padded_range([v for _, v in pts])
        n = max(1, len(self.values) - 1)
        p.setFont(QFont("Noto Sans Mono", 7))
        for v in nice_ticks(lo, hi, 3):
            y = r.bottom() - (v - lo) / (hi - lo) * r.height()
            if not r.top() - 1 <= y <= r.bottom() + 1:
                continue
            p.setPen(QPen(GRID, 1))
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y))
            p.setPen(TEXT)
            p.drawText(QRectF(0, y - 7, r.left() - px(4), 14), Qt.AlignRight | Qt.AlignVCenter, f"{v:g}")
        avg = sum(v for _, v in pts) / len(pts)
        ya = r.bottom() - (avg - lo) / (hi - lo) * r.height()
        p.setPen(QPen(MUTED_LINE, 1, Qt.DashLine))
        p.drawLine(QPointF(r.left(), ya), QPointF(r.right(), ya))
        xy = [QPointF(r.left() + i / n * r.width(), r.bottom() - (v - lo) / (hi - lo) * r.height())
              for i, v in pts]
        p.setPen(QPen(ACCENT, max(1.5, px(2))))
        p.drawPolyline(QPolygonF(xy))
        p.setPen(Qt.NoPen)
        p.setBrush(ACCENT)
        for q in xy:
            p.drawEllipse(q, 3, 3)
        p.setBrush(BRIGHT)
        p.drawEllipse(xy[-1], 4.5, 4.5)
        p.end()


# ───────────────────────────── the dialog ──────────────────────────────────

class FlightDetailDialog(QDialog):
    """One flight, opened from the Logs table: summary, charts, ground track."""

    def __init__(self, rec: FlightRecord, parent=None):
        super().__init__(parent)
        self.rec = rec
        day = (rec.started_utc or "--").replace("T", " ").rstrip("Z")
        self.setWindowTitle(f"Flight {day}")
        self.setMinimumSize(px(900), px(620))

        root = QVBoxLayout(self)
        root.setContentsMargins(px(16), px(14), px(16), px(14))
        root.setSpacing(px(10))

        head = QHBoxLayout()
        title = QLabel(f"FLIGHT  {day}", self)
        title.setObjectName("pageTitle")
        head.addWidget(title)
        head.addStretch(1)
        status = rec.status + (" (FORCED ARM)" if rec.forced_arm else "")
        self.lbl_status = QLabel(status, self)
        self.lbl_status.setObjectName("fieldSubLabel")
        head.addWidget(self.lbl_status)
        root.addLayout(head)

        # summary tiles
        tiles = QHBoxLayout()
        tiles.setSpacing(px(8))
        self.tiles = {}
        sag = rec.voltage_sag
        for key, caption, value in (
                ("duration", "DURATION", rec.duration_hms()),
                ("distance", "DISTANCE", f"{rec.distance_m:.1f} m"),
                ("alt", "MAX ALTITUDE", f"{rec.max_altitude_m:.2f} m"),
                ("speed", "MAX SPEED", f"{rec.max_speed_ms:.2f} m/s"),
                ("battery", "BATTERY", f"{rec.battery_start_pct}% → {rec.battery_end_pct}%"),
                ("sag", "VOLTAGE SAG", "–" if sag is None else f"{sag:.2f} V")):
            card = QFrame(self)
            card.setProperty("class", "cardFrame")
            cl = QVBoxLayout(card)
            cl.setContentsMargins(px(10), px(6), px(10), px(6))
            cl.setSpacing(px(1))
            cap = QLabel(caption, card)
            cap.setObjectName("diagCaption")
            cl.addWidget(cap)
            val = QLabel(value, card)
            val.setObjectName("statValue")
            cl.addWidget(val)
            self.tiles[key] = val
            tiles.addWidget(card, 1)
        root.addLayout(tiles)

        self.lbl_modes = QLabel(f"Modes: {rec.mode_summary}", self)
        self.lbl_modes.setObjectName("fieldSubLabel")
        root.addWidget(self.lbl_modes)

        s = rec.series
        t = s.get("t", [])
        self.chart_alt = SeriesChart("Altitude", "m", self, floor_zero=True)
        self.chart_speed = SeriesChart("Ground speed", "m/s", self, floor_zero=True)
        self.chart_batt = SeriesChart("Battery", "%", self)
        self.chart_volt = SeriesChart("Battery voltage", "V", self)
        self.path = PathMap(self)
        for chart, key in ((self.chart_alt, "alt"), (self.chart_speed, "speed"),
                           (self.chart_batt, "batt"), (self.chart_volt, "volt")):
            chart.set_data(t, s.get(key, []))
        self.path.set_track(s.get("x", []), s.get("y", []))

        body = QGridLayout()
        body.setSpacing(px(8))
        body.addWidget(self.path, 0, 0, 2, 1)
        body.addWidget(self.chart_alt, 0, 1)
        body.addWidget(self.chart_speed, 0, 2)
        body.addWidget(self.chart_batt, 1, 1)
        body.addWidget(self.chart_volt, 1, 2)
        body.setColumnStretch(0, 3)
        body.setColumnStretch(1, 3)
        body.setColumnStretch(2, 3)
        root.addLayout(body, 1)

        self.lbl_note = QLabel("", self)
        self.lbl_note.setObjectName("fieldSubLabel")
        self.lbl_note.setWordWrap(True)
        if not rec.has_series:
            self.lbl_note.setText(
                "No time series was recorded for this flight (it was written by an older "
                "build, or the flight was shorter than a sample). Summary figures only.")
        root.addWidget(self.lbl_note)

        foot = QHBoxLayout()
        foot.addStretch(1)
        self.btn_export = QPushButton("Export this flight (CSV)", self)
        self.btn_export.setEnabled(rec.has_series)
        self.btn_export.clicked.connect(self._export)
        foot.addWidget(self.btn_export)
        btn_close = QPushButton("Close", self)
        btn_close.clicked.connect(self.accept)
        foot.addWidget(btn_close)
        root.addLayout(foot)

    def _export(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Export flight", "flight.csv", "CSV files (*.csv)")
        if not path:
            return
        self.write_csv(path)
        self.lbl_note.setText(f"Exported {len(series_rows(self.rec))} samples to {path}")

    def write_csv(self, path: str) -> int:
        """Write the series as t,alt,speed,batt,volt,x,y. Returns the row count."""
        rows = series_rows(self.rec)
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "altitude_m", "speed_ms", "battery_pct", "voltage_v", "north_m", "east_m"])
            w.writerows(rows)
        return len(rows)
