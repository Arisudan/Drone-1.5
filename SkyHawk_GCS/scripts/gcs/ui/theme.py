"""
================================================================================
MODULE: theme.py
PURPOSE: v2 "glass over imagery" visual primitives (backdrop, glass panels, icons)
================================================================================

Visual layer only. Nothing here talks to MAVLink, ROS, settings or workers; it
is the set of building blocks the redesigned shell and pages are drawn from:

  * ``Backdrop``    full-window background painted behind every workspace
                    (sharp imagery, dimmed+blurred imagery, or a dark map grid),
                    plus the vignette. Also keeps a pre-blurred copy that glass
                    panels paint through, which is how Qt gets "frosted glass"
                    without a compositor backdrop-filter.
  * ``GlassPanel``  rounded translucent card with the uppercase 34px header
                    (optional green pip and right-aligned sub caption).
  * ``Chip``        pill status label: neutral / ok / warn / bad.
  * ``MetricCell``  small caption-over-value tile used by the telemetry card.
  * ``icon()``      the line icons of the design (24x24 stroke SVG paths).

Colours come from ``ui.styles.PALETTE`` where an opaque colour is enough; the
translucent glass tints below are theme-local because QColor/QSS alpha values
do not belong in the palette dict (several modules feed it to QColor()).
================================================================================
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt, QRectF, QByteArray, QPointF, QSize
from PyQt5.QtGui import (QColor, QPainter, QPainterPath, QPixmap, QImage, QPen,
                         QBrush, QRadialGradient, QLinearGradient)
from PyQt5.QtSvg import QSvgRenderer
from PyQt5.QtWidgets import (QFrame, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QSizePolicy,
                             QStackedWidget)

from ui.scaling import px

# ── design tokens (mirrors the reference style.css) ─────────────────────────
GREEN = "#46d06a"
GREEN_D = "#2a9a48"
ORANGE = "#ffb02e"
RED = "#ff5a4f"
TEXT = "#e9ece9"
DIM = "#a3aaa5"
MUTE = "#6f7772"

GLASS_TINT = QColor(22, 25, 24, 189)        # rgba(22,25,24,.74)
GLASS_LINE = QColor(255, 255, 255, 22)      # rgba(255,255,255,.085)
GLASS_HILITE = QColor(255, 255, 255, 18)
RADIUS = 10

_ASSET_DIR = Path(__file__).resolve().parents[3] / "assets" / "backdrop"

# ── icons ───────────────────────────────────────────────────────────────────
ICONS = {
    "control": '<circle cx="12" cy="12" r="9"/><path d="M12 7l3.5 9L12 14l-3.5 2z"/>',
    "fpv": '<rect x="2.5" y="6" width="13" height="12" rx="2.5"/><path d="M15.5 10.5l6-3.5v10l-6-3.5"/>',
    "slam": '<path d="M3 6l6-3 6 3 6-3v15l-6 3-6-3-6 3z"/><path d="M9 3v15M15 6v15"/>',
    "motor": '<circle cx="12" cy="12" r="2"/><path d="M12 10C12 5 9 3.5 7 4.5S5.5 8.5 10 10.7M14 12c5 0 6.5 3 5.5 5S15.5 18.5 13.3 14M12 14c0 5 3 6.5 5 5.5S18.5 15.5 14 13.3M10 12c-5 0-6.5-3-5.5-5S8.5 5.5 10.7 10"/>',
    "diag": '<path d="M22 12h-4l-3 9L9 3l-3 9H2"/>',
    "term": '<path d="M4 17l6-6-6-6M12 19h8"/>',
    "logs": '<path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"/><path d="M14 2v6h6M8 13h8M8 17h6"/>',
    "cfg": '<path d="M20 7h-9M14 17H5"/><circle cx="17" cy="17" r="3"/><circle cx="7" cy="7" r="3"/>',
    "params": '<path d="M21 4h-7M10 4H3M21 12h-9M8 12H3M21 20h-5M12 20H3M14 2v4M8 10v4M16 18v4"/>',
    "drone": '<circle cx="5" cy="5" r="2.4"/><circle cx="19" cy="5" r="2.4"/><circle cx="5" cy="19" r="2.4"/><circle cx="19" cy="19" r="2.4"/><path d="M7 7l3 3M17 7l-3 3M7 17l3-3M17 17l-3-3"/><rect x="9.5" y="9.5" width="5" height="5" rx="1.5"/>',
    "bat": '<rect x="2" y="7" width="17" height="10" rx="2"/><path d="M22 11v2"/>',
    "gps": '<circle cx="12" cy="12" r="3"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/>',
    "rc": '<path d="M5 12a10 10 0 0 1 14 0M8.5 15.5a5 5 0 0 1 7 0"/><circle cx="12" cy="19" r="1"/>',
    "sat": '<path d="M4 4l6 6M14 14l6 6"/><rect x="9" y="9" width="6" height="6" rx="1" transform="rotate(45 12 12)"/>',
}

_icon_cache: dict = {}


def icon(name: str, color: str = DIM, size: int = 20, stroke: float = 1.7) -> QPixmap:
    """Render a design line icon to a (device-pixel-aware) QPixmap."""
    key = (name, color, size, stroke)
    pm = _icon_cache.get(key)
    if pm is not None:
        return pm
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" '
           f'stroke="{color}" stroke-width="{stroke}" stroke-linecap="round" '
           f'stroke-linejoin="round">{ICONS[name]}</svg>')
    r = QSvgRenderer(QByteArray(svg.encode()))
    dpr = 2
    pm = QPixmap(size * dpr, size * dpr)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    r.render(p, QRectF(0, 0, size * dpr, size * dpr))
    p.end()
    pm.setDevicePixelRatio(dpr)
    _icon_cache[key] = pm
    return pm


# ── backdrop ────────────────────────────────────────────────────────────────
def _blur(img: QImage, radius_div: int) -> QImage:
    """Cheap, allocation-light blur: shrink hard, then smooth-scale back up."""
    w, h = img.width(), img.height()
    small = img.scaled(max(2, w // radius_div), max(2, h // radius_div),
                       Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
    return small.scaled(w, h, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)


def _cover(img: QImage, w: int, h: int) -> QImage:
    """CSS ``background-size: cover; background-position: center``."""
    if img.isNull() or w <= 0 or h <= 0:
        return QImage(max(1, w), max(1, h), QImage.Format_RGB32)
    scaled = img.scaled(w, h, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
    x, y = (scaled.width() - w) // 2, (scaled.height() - h) // 2
    return scaled.copy(x, y, w, h)


class Backdrop(QWidget):
    """Full-window background. ``set_mode`` is the only thing pages touch.

    modes:  "map"   sharp imagery            (Control, FPV)
            "dim"   blurred, darkened imagery (Motors, Diagnostics, ...)
            "grid"  flat dark with a 50px grid (Tactical SLAM)
    """

    MODES = ("map", "dim", "grid")

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_OpaquePaintEvent, True)
        self._mode = "map"
        self._image_name = "sat_campus.jpg"
        self._src: dict = {}
        self._cache_key = None
        self._bg = QImage()
        self._glass = QImage()

    # -- public ------------------------------------------------------------
    def set_mode(self, mode: str, image: str = "sat_campus.jpg") -> None:
        if mode not in self.MODES:
            mode = "map"
        if (mode, image) == (self._mode, self._image_name):
            return
        self._mode, self._image_name = mode, image
        self._cache_key = None
        self.update()
        # Glass panels read the blurred copy, so they must repaint too.
        for cls in (GlassPanel, GlassBar):
            for w in self.window().findChildren(cls):
                w.update()

    def mode(self) -> str:
        return self._mode

    def glass_image(self) -> QImage:
        self._ensure()
        return self._glass

    # -- internals ---------------------------------------------------------
    def _source(self, name: str) -> QImage:
        img = self._src.get(name)
        if img is None:
            img = QImage(str(_ASSET_DIR / name))
            self._src[name] = img
        return img

    def _ensure(self) -> None:
        key = (self._mode, self._image_name, self.width(), self.height())
        if key == self._cache_key:
            return
        self._cache_key = key
        w, h = max(1, self.width()), max(1, self.height())
        if self._mode == "grid":
            self._bg = QImage(w, h, QImage.Format_RGB32)
            self._bg.fill(QColor("#0b0e0d"))
            p = QPainter(self._bg)
            p.setPen(QPen(QColor(255, 255, 255, 13), 1))
            for x in range(0, w, 50):
                p.drawLine(x, 0, x, h)
            for y in range(0, h, 50):
                p.drawLine(0, y, w, y)
            p.end()
            self._glass = self._bg
            return
        base = _cover(self._source(self._image_name), w, h).convertToFormat(QImage.Format_RGB32)
        if self._mode == "dim":
            # filter: blur(6px) brightness(.38) saturate(.7); transform: scale(1.04)
            img = _blur(base, 10)
            p = QPainter(img)
            p.fillRect(img.rect(), QColor(0, 0, 0, int(255 * 0.62)))
            p.end()
            self._bg = img
        else:
            # filter: saturate(.92) contrast(1.04) brightness(.9)
            img = base.copy()
            p = QPainter(img)
            p.fillRect(img.rect(), QColor(0, 0, 0, int(255 * 0.10)))
            p.end()
            self._bg = img
        # vignette (radial + top fade)
        p = QPainter(self._bg)
        grad = QRadialGradient(QPointF(w * 0.5, h * 0.45), max(w, h) * 0.72)
        grad.setColorAt(0.55, QColor(0, 0, 0, 0))
        grad.setColorAt(1.0, QColor(0, 0, 0, 140))
        p.fillRect(self._bg.rect(), QBrush(grad))
        top = QLinearGradient(0, 0, 0, px(140))
        top.setColorAt(0, QColor(0, 0, 0, 128))
        top.setColorAt(1, QColor(0, 0, 0, 0))
        p.fillRect(0, 0, w, px(140), QBrush(top))
        p.end()
        # frosted copy for the glass panels
        self._glass = _blur(self._bg, 14) if self._mode == "map" else self._bg

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._cache_key = None

    def paintEvent(self, _ev):
        self._ensure()
        p = QPainter(self)
        p.drawImage(0, 0, self._bg)
        p.end()


# ── glass ───────────────────────────────────────────────────────────────────
def _find_backdrop(widget: QWidget):
    top = widget.window()
    bd = top.__dict__.get("_v2_backdrop")
    if bd is None:
        for w in top.findChildren(Backdrop):
            bd = w
            top.__dict__["_v2_backdrop"] = bd
            break
    return bd


def paint_glass(widget: QWidget, p: QPainter, rect: QRectF, radius: float = RADIUS,
                border: Optional[QColor] = None, glow: Optional[QColor] = None) -> None:
    """Frosted-glass card background for ``widget`` inside ``rect``."""
    p.setRenderHint(QPainter.Antialiasing)
    path = QPainterPath()
    path.addRoundedRect(rect, radius, radius)
    bd = _find_backdrop(widget)
    if bd is not None and bd.width() > 0:
        img = bd.glass_image()
        # The backdrop is a sibling of the UI (not an ancestor): map through their parent.
        origin = widget.mapTo(bd.parentWidget(), rect.topLeft().toPoint()) - bd.pos()
        src = QRectF(origin.x(), origin.y(), rect.width(), rect.height()).intersected(
            QRectF(0, 0, img.width(), img.height()))
        if not img.isNull() and not src.isEmpty():
            p.save()
            p.setClipPath(path)
            dst = QRectF(rect.left() + (src.x() - origin.x()), rect.top() + (src.y() - origin.y()),
                         src.width(), src.height())
            p.drawImage(dst, img, src)
            p.restore()
    if glow is not None:
        g = QPainterPath()
        g.addRoundedRect(rect.adjusted(-3, -3, 3, 3), radius + 3, radius + 3)
        p.fillPath(g, glow)
    p.fillPath(path, GLASS_TINT)
    p.setPen(QPen(border or GLASS_LINE, 1))
    p.setBrush(Qt.NoBrush)
    p.drawRoundedRect(rect.adjusted(0.5, 0.5, -0.5, -0.5), radius, radius)
    # inset top highlight
    p.setPen(QPen(GLASS_HILITE, 1))
    p.drawLine(QPointF(rect.left() + radius, rect.top() + 1.5),
               QPointF(rect.right() - radius, rect.top() + 1.5))


class GlassPanel(QFrame):
    """Translucent card: optional 34px uppercase header, free-form body."""

    HEADER_H = 34

    def __init__(self, title: str = "", sub: str = "", pip: bool = False,
                 parent: Optional[QWidget] = None, header: bool = True):
        super().__init__(parent)
        self.setObjectName("glassPanel")
        self.setAttribute(Qt.WA_StyledBackground, False)
        self._border: Optional[QColor] = None
        self._glow: Optional[QColor] = None
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        self.header = None
        self.lbl_title = self.lbl_sub = self.pip = None
        if header:
            self.header = QWidget(self)
            self.header.setObjectName("glassHeader")
            self.header.setFixedHeight(px(self.HEADER_H))
            hl = QHBoxLayout(self.header)
            hl.setContentsMargins(px(14), 0, px(14), 0)
            hl.setSpacing(px(8))
            self.pip = QLabel(self.header)
            self.pip.setObjectName("glassPip")
            self.pip.setFixedSize(px(6), px(6))
            self.pip.setVisible(pip)
            hl.addWidget(self.pip)
            self.lbl_title = QLabel(title.upper(), self.header)
            self.lbl_title.setObjectName("glassTitle")
            hl.addWidget(self.lbl_title)
            hl.addStretch(1)
            self.lbl_sub = QLabel(sub, self.header)
            self.lbl_sub.setObjectName("glassSub")
            hl.addWidget(self.lbl_sub)
            outer.addWidget(self.header)

        self.body = QWidget(self)
        self.body.setObjectName("glassBody")
        self.body_layout = QVBoxLayout(self.body)
        self.body_layout.setContentsMargins(px(14), px(12), px(14), px(12))
        self.body_layout.setSpacing(px(8))
        outer.addWidget(self.body, 1)

    def set_title(self, title: str) -> None:
        if self.lbl_title is not None:
            self.lbl_title.setText(title.upper())

    def set_sub(self, text: str) -> None:
        if self.lbl_sub is not None:
            self.lbl_sub.setText(text)

    def set_accent(self, border: Optional[QColor], glow: Optional[QColor] = None) -> None:
        self._border, self._glow = border, glow
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        paint_glass(self, p, QRectF(self.rect()), border=self._border, glow=self._glow)
        if self.header is not None:
            p.setPen(QPen(GLASS_LINE, 1))
            y = px(self.HEADER_H)
            p.drawLine(1, y, self.width() - 2, y)
        p.end()


class GlassBar(QFrame):
    """Headerless glass strip (rail, route card, vehicle card, status bar)."""

    def __init__(self, parent: Optional[QWidget] = None, radius: float = RADIUS):
        super().__init__(parent)
        self._radius = radius
        self._border: Optional[QColor] = None
        self._glow: Optional[QColor] = None

    def set_accent(self, border: Optional[QColor], glow: Optional[QColor] = None) -> None:
        self._border, self._glow = border, glow
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        paint_glass(self, p, QRectF(self.rect()), self._radius, self._border, self._glow)
        p.end()


class Chip(QLabel):
    """Pill label. ``state``: "" (neutral) | "ok" | "warn" | "bad"."""

    def __init__(self, text: str = "", state: str = "", parent: Optional[QWidget] = None):
        super().__init__(text, parent)
        self.setObjectName("chip")
        self.setAlignment(Qt.AlignCenter)
        self.set_state(state)

    def set_state(self, state: str) -> None:
        if self.property("state") != state:
            self.setProperty("state", state)
            self.style().unpolish(self)
            self.style().polish(self)


class MetricCell(QFrame):
    """Caption-over-value tile (telemetry card)."""

    def __init__(self, caption: str, value: str = "--", unit: str = "",
                 parent: Optional[QWidget] = None, big: int = 23):
        super().__init__(parent)
        self.setObjectName("metricCell")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(px(10), px(8), px(10), px(8))
        lay.setSpacing(px(3))
        self.cap = QLabel(caption.upper(), self)
        self.cap.setObjectName("metricCap")
        self.cap.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.cap)
        self.lines = []
        self._big = big
        self.add_line(value, unit, big)
        lay.addStretch(1)

    def add_line(self, value: str, unit: str = "", size: Optional[int] = None) -> QLabel:
        lbl = QLabel(self)
        lbl.setObjectName("metricVal")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setProperty("px", size or self._big)
        lbl.setTextFormat(Qt.RichText)
        self.layout().insertWidget(self.layout().count() - (1 if self.lines else 0), lbl)
        self.lines.append((lbl, unit, size or self._big))
        self.set_line(len(self.lines) - 1, value, "dim")
        return lbl

    def set_line(self, i: int, value: str, tone: str = "") -> None:
        """tone: "" white | "dim" muted | "ok" | "warn" | "bad"."""
        lbl, unit, size = self.lines[i]
        colour = {"": TEXT, "dim": MUTE, "ok": GREEN, "warn": ORANGE, "bad": RED}.get(tone, TEXT)
        u = (f'<span style="font-size:{px(12)}px;font-weight:500;color:{DIM}"> {unit}</span>'
             if unit else "")
        lbl.setText(f'<span style="font-size:{px(size)}px;font-weight:700;color:{colour}">'
                    f'{value}</span>{u}')


class ShellRoot(QWidget):
    """Central widget: the backdrop at the bottom, an optional full-bleed layer
    (live video) above it, and the real UI (a layout installed by the caller)."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("shellRoot")
        self.backdrop = Backdrop(self)
        self.backdrop.lower()
        self._bleed: Optional[QWidget] = None

    def set_bleed(self, widget: QWidget) -> None:
        """Park ``widget`` as the full-bleed layer between backdrop and UI."""
        self._bleed = widget
        widget.setParent(self)
        widget.setGeometry(self.rect())
        widget.stackUnder(self._first_ui_child())

    def _first_ui_child(self) -> QWidget:
        for w in self.children():
            if isinstance(w, QWidget) and w is not self.backdrop and w is not self._bleed:
                return w
        return self.backdrop

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self.backdrop.setGeometry(self.rect())
        if self._bleed is not None:
            self._bleed.setGeometry(self.rect())


class CurrentPageStack(QStackedWidget):
    """QStackedWidget that reports the size hints of the *visible* page only, so a
    small window is sized by the page being shown (and scrolls if even that does not
    fit) instead of by the largest page in the application."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.currentChanged.connect(lambda _i: self.updateGeometry())

    def minimumSizeHint(self) -> QSize:
        w = self.currentWidget()
        return w.minimumSizeHint() if w is not None else QSize(0, 0)

    def sizeHint(self) -> QSize:
        w = self.currentWidget()
        return w.sizeHint() if w is not None else QSize(0, 0)
