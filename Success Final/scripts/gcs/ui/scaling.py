"""
================================================================================
MODULE: scaling.py
PURPOSE: One UI scale factor for the whole ground station (HiDPI / font size)
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station & Radxa SBC (GCS UI Layer)
  * Upstream:      --ui-scale flag, GCS_UI_SCALE env, settings.ui.scale, screen DPI
  * Downstream:    ui/styles.py (QSS rewrite) and every widget that sizes itself

WHY THIS EXISTS:
  Every dimension in this station was a literal pixel count: 47 `font-size: Npx`
  rules in the stylesheet and ~21 setFixedWidth/setMinimumHeight calls across the
  widgets, all authored against one 1920x1080 laptop at 96 DPI. On the 4K panel
  in the lab the entire station renders at roughly half its intended size, and on
  a display with 150% desktop scaling the fixed-width fields clip their own text.
  Neither is a styling preference - the numbers on screen stop being readable
  from arm's length, which is the distance an operator actually stands at.

TWO INDEPENDENT SCALES, DELIBERATELY:
  Qt's AA_EnableHighDpiScaling handles the *device* pixel ratio - a 4K panel
  reporting 2x. That is Qt's job and it does it correctly, including pixmaps.
  What Qt does NOT do is honour "this operator wants bigger text", or correct a
  panel that reports 1x while being physically 27" at 4K. That second factor is
  what this module owns. The two multiply; neither double-counts the other,
  because AA_EnableHighDpiScaling normalises logicalDotsPerInch back to the base
  DPI before we ever read it.

WHAT GETS SCALED AND WHAT DOES NOT:
  Scaled:     font-size, padding, margin, min/max-width, min/max-height,
              border-radius, spacing, icon and widget dimensions.
  Not scaled: `border: 1px solid ...` widths. A hairline is a hairline at every
              scale; multiplying it turns every panel edge into a 2-3 px slab and
              the card grid reads as a table of boxes instead of a layout.

RESOLUTION ORDER (first non-zero wins):
  --ui-scale flag > GCS_UI_SCALE env > settings.ui.scale > automatic.
  Automatic measures the platform's own default font against the size this UI
  was authored at, so a desktop configured for large text scales the station
  with it. Clamped to 0.75..3.0: outside that the layout stops fitting on any
  real screen, and a typo in the env var should not produce an unusable window.

USAGE:
  from ui.scaling import px, pt, scaled_font, init_scale, get_scale

  init_scale(app, settings)          # once, in main(), before setStyleSheet
  widget.setFixedWidth(px(120))
  painter.setFont(scaled_font("Segoe UI", 8, bold=True))
================================================================================
"""

from __future__ import annotations

import logging
import os
import re
from typing import Optional

log = logging.getLogger("gcs.scaling")

# The UI was authored against these. Do not "fix" them to match a new machine -
# they are the reference the scale factor is measured *against*, so changing one
# silently rescales every dimension in the station.
REFERENCE_DPI = 96.0
REFERENCE_FONT_HEIGHT_PX = 17.0

MIN_SCALE = 0.75
MAX_SCALE = 3.0

_scale: float = 1.0


# ─── Query ──────────────────────────────────────────────────────────

def get_scale() -> float:
    """The active factor. 1.0 until init_scale() runs, so importing this module
    from a test or a headless tool needs no Qt application."""
    return _scale


def set_scale(value: float) -> float:
    """Set the factor directly, clamped. Returns what was actually applied."""
    global _scale
    _scale = clamp_scale(value)
    return _scale


def clamp_scale(value: float) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 1.0
    if v <= 0:
        return 1.0
    return max(MIN_SCALE, min(MAX_SCALE, v))


# ─── Dimension helpers ──────────────────────────────────────────────

def px(value: float) -> int:
    """Scale a pixel dimension. Always returns at least 1 for a positive input,
    so a 1 px separator never scales away to nothing at a sub-1.0 factor."""
    scaled = float(value) * _scale
    if value > 0:
        return max(1, int(round(scaled)))
    return int(round(scaled))


def pt(value: float) -> int:
    """Scale a point size (QFont). Floors at 5 pt - below that the glyphs stop
    resolving and a 'smaller' font just becomes a grey smear."""
    return max(5, int(round(float(value) * _scale)))


def scaled_font(family: str, point_size: float, bold: bool = False,
                italic: bool = False):
    """A QFont at the scaled point size. Imported lazily so this module stays
    usable (for px()/scale_qss()) in a process with no QtGui."""
    from PyQt5.QtGui import QFont
    f = QFont(family, pt(point_size))
    f.setBold(bold)
    f.setItalic(italic)
    return f


# ─── Stylesheet rewriting ───────────────────────────────────────────

# Properties whose px values are real dimensions. `border` is absent on purpose
# (see the module docstring); so are `border-top/right/bottom/left`, which in
# this stylesheet are only ever used to draw rules.
_SCALABLE_PROPERTIES = (
    "font-size", "padding", "padding-top", "padding-right", "padding-bottom",
    "padding-left", "margin", "margin-top", "margin-right", "margin-bottom",
    "margin-left", "min-width", "max-width", "min-height", "max-height",
    "width", "height", "border-radius", "spacing", "left", "right", "top",
    "bottom",
)

_PROP_RE = re.compile(
    r"(?P<prop>\b(?:%s)\s*:)(?P<values>[^;}]*)" % "|".join(
        re.escape(p) for p in _SCALABLE_PROPERTIES),
    re.IGNORECASE,
)
_PX_RE = re.compile(r"(-?\d+(?:\.\d+)?)px")


def scale_qss(qss: str, scale: Optional[float] = None) -> str:
    """Multiply every px value of every scalable property in a Qt stylesheet.

    Operates only inside the value span of a recognised property, so a px count
    appearing in a url() path, a comment, or a `border:` shorthand is left
    exactly as written.
    """
    factor = _scale if scale is None else clamp_scale(scale)
    if abs(factor - 1.0) < 1e-6:
        return qss

    def _scale_values(m: "re.Match") -> str:
        def _one(pm: "re.Match") -> str:
            n = float(pm.group(1)) * factor
            # Round away from zero so a 1px padding never collapses to 0 when
            # the factor is only slightly above 1.
            out = int(n) if abs(n - int(n)) < 1e-9 else int(round(n))
            if out == 0 and float(pm.group(1)) != 0:
                out = 1 if float(pm.group(1)) > 0 else -1
            return f"{out}px"
        return m.group("prop") + _PX_RE.sub(_one, m.group("values"))

    return _PROP_RE.sub(_scale_values, qss)


# ─── Automatic detection ────────────────────────────────────────────

def detect_scale(app=None) -> float:
    """Best-effort factor for the current display, or 1.0 if it cannot tell.

    Two independent signals, and the larger wins: the platform's default font
    height relative to what this UI was drawn against, and the screen's logical
    DPI relative to 96. The font is the better signal on Linux (it tracks the
    desktop's own text-scaling setting); DPI is the fallback where the font is
    left at its default on a genuinely high-density panel.
    """
    font_scale = 1.0
    dpi_scale = 1.0

    try:
        from PyQt5.QtGui import QFontMetricsF, QGuiApplication
        inst = app or QGuiApplication.instance()
        if inst is not None:
            height = QFontMetricsF(inst.font()).height()
            if height > 0:
                font_scale = height / REFERENCE_FONT_HEIGHT_PX
            screen = inst.primaryScreen()
            if screen is not None:
                dpi = screen.logicalDotsPerInch()
                if dpi > 0:
                    dpi_scale = dpi / REFERENCE_DPI
    except Exception:
        log.debug("UI scale auto-detection unavailable", exc_info=True)
        return 1.0

    # A few percent either way is measurement noise, not an operator asking for
    # bigger text. Snapping it to 1.0 keeps the stylesheet byte-identical to the
    # authored one on the machine it was authored on.
    best = max(font_scale, dpi_scale)
    if 0.92 <= best <= 1.08:
        return 1.0
    return clamp_scale(best)


def init_scale(app=None, settings=None, override=None) -> float:
    """Resolve and install the factor. Call once, before applying a stylesheet.

    Precedence: explicit override (the --ui-scale flag) > GCS_UI_SCALE >
    settings.ui.scale > automatic. A zero or absent value at any level means
    "not specified, keep looking", which is why settings.ui.scale defaults to 0
    rather than 1.
    """
    candidate = None
    source = "auto"

    if override:
        candidate, source = override, "--ui-scale"

    if candidate is None:
        env = os.environ.get("GCS_UI_SCALE")
        if env:
            candidate, source = env, "GCS_UI_SCALE"

    if candidate is None and settings is not None:
        configured = getattr(getattr(settings, "ui", None), "scale", 0.0) or 0.0
        if configured:
            candidate, source = configured, "settings.ui.scale"

    if candidate is None:
        value = detect_scale(app)
    else:
        try:
            value = float(candidate)
        except (TypeError, ValueError):
            log.warning("ignoring unparsable UI scale %r from %s", candidate, source)
            value = detect_scale(app)
            source = "auto"

    applied = set_scale(value)
    log.info("UI scale %.2f (from %s)", applied, source)
    return applied


def enable_high_dpi() -> None:
    """Turn on Qt's own device-pixel-ratio handling.

    Must run before the QApplication is constructed - Qt reads these attributes
    once at startup and silently ignores them afterwards, which is exactly the
    kind of failure that looks like "the setting does nothing".
    """
    try:
        from PyQt5.QtCore import Qt, QCoreApplication
        QCoreApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QCoreApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    except Exception:
        log.debug("high-DPI attributes unavailable", exc_info=True)
