"""
================================================================================
MODULE: sidebar_nav.py
PURPOSE: v2 icon rail - workspace switcher with the preflight counter underneath
================================================================================

A 50px glass rail of nine icon buttons (tooltip = workspace name), a rule, and a
"0/7 PRE" preflight counter. The active workspace is a solid green tile.

What is *not* here any more, and where it went:
  * the text list / SPD-ALT-MODE-ARMED footer -> the top bar, vehicle card and
    telemetry card show that state on every workspace;
  * the PREFLIGHT checklist panel -> the Diagnostics workspace. The
    ``ChecklistPanel`` object still exists as ``self.checklist`` because the
    preflight controller feeds it and the ARM interlock reads its switches; the
    main window re-parents it into the Diagnostics card.
  * the camera thumbnail (``mini_feed``) -> kept as an attribute so frame wiring
    is unchanged, but never shown: the design has no rail thumbnail.

Public API kept: ``view_changed``, ``select_tab``, ``btn_group``, ``checklist``,
``mini_feed``, ``set_flight_state``, ``set_instruments`` and the width helpers.
================================================================================
"""

from __future__ import annotations
from typing import Optional

from PyQt5.QtCore import QRectF, QSize, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QLinearGradient, QPainter, QPen, QBrush, QPainterPath
from PyQt5.QtWidgets import (QButtonGroup, QFrame, QLabel, QPushButton, QVBoxLayout, QWidget)

from ui.mini_feed import MiniFeed
from ui.preflight_panel import ChecklistPanel
from ui.scaling import px
from ui.theme import GlassBar, icon, GREEN, ORANGE, MUTE, DIM

# (icon, tooltip, page index)
ITEMS = [
    ("control", "Control", 0),
    ("fpv", "FPV Camera", 1),
    ("slam", "Tactical SLAM", 2),
    ("motor", "Motor Actuators", 3),
    ("diag", "Diagnostics", 4),
    ("term", "Flight Terminal", 5),
    ("logs", "Flight Logs", 6),
    ("cfg", "Configuration", 7),
    ("params", "Parameters", 8),
]


class RailButton(QPushButton):
    def __init__(self, icon_name: str, tip: str, parent=None):
        super().__init__("", parent)
        self._icon = icon_name
        self.setCheckable(True)
        self.setToolTip(tip)
        self.setAccessibleName(tip)
        self.setCursor(Qt.PointingHandCursor)
        self.setFixedSize(px(38), px(38))
        self.setFocusPolicy(Qt.NoFocus)

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        on = self.isChecked()
        if on:
            glow = QPainterPath()
            glow.addRoundedRect(r.adjusted(-3, -3, 3, 3), 11, 11)
            p.fillPath(glow, QColor(70, 208, 106, 46))
            g = QLinearGradient(0, 0, 0, self.height())
            g.setColorAt(0, QColor("#58dc7c"))
            g.setColorAt(1, QColor("#34b857"))
            p.setBrush(QBrush(g))
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(r, 9, 9)
        elif self.underMouse():
            p.setBrush(QColor(255, 255, 255, 18))
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(r, 9, 9)
        pm = icon(self._icon, "#06210f" if on else (("#e9ece9") if self.underMouse() else DIM), 20)
        s = pm.width() / pm.devicePixelRatio()
        p.drawPixmap(int((self.width() - s) / 2), int((self.height() - s) / 2), pm)
        p.end()

    def enterEvent(self, e):
        self.update()
        super().enterEvent(e)

    def leaveEvent(self, e):
        self.update()
        super().leaveEvent(e)


class _PreCounter(QWidget):
    clicked = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCursor(Qt.PointingHandCursor)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, px(4), 0, px(2))
        lay.setSpacing(px(1))
        self.val = QLabel("0/7", self)
        self.val.setObjectName("railPreVal")
        self.val.setAlignment(Qt.AlignCenter)
        self.cap = QLabel("PRE", self)
        self.cap.setObjectName("railPreCap")
        self.cap.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.val)
        lay.addWidget(self.cap)

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(ev)


class SidebarNav(GlassBar):
    """Vertical icon rail."""

    view_changed = pyqtSignal(int)

    WIDE_PX = 50
    NARROW_PX = 50
    WIDE_MIN_WINDOW_PX = 1280
    # Workspaces that used to show the rail thumbnail (kept for API compatibility).
    MINI_FEED_TABS = frozenset({3, 4, 5, 6, 7, 8})

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("navRail")
        self.setFixedWidth(px(self.WIDE_PX))
        self._wide = True
        self._compact = False
        self._mini_wanted = False

        lay = QVBoxLayout(self)
        lay.setContentsMargins(px(6), px(8), px(6), px(8))
        lay.setSpacing(px(4))
        lay.setAlignment(Qt.AlignHCenter)

        self.btn_group = QButtonGroup(self)
        self.btn_group.setExclusive(True)
        for name, tip, idx in ITEMS:
            btn = RailButton(name, tip, self)
            if idx == 0:
                btn.setChecked(True)
            self.btn_group.addButton(btn, idx)
            lay.addWidget(btn, 0, Qt.AlignHCenter)
        self.btn_group.buttonClicked[int].connect(self.view_changed.emit)

        sep = QFrame(self)
        sep.setObjectName("railSep")
        sep.setFixedSize(px(26), 1)
        lay.addSpacing(px(2))
        lay.addWidget(sep, 0, Qt.AlignHCenter)
        self.pre = _PreCounter(self)
        self.pre.clicked.connect(lambda: self.select_tab(4))
        self.pre.setToolTip("Preflight checks - click to open Diagnostics")
        lay.addWidget(self.pre, 0, Qt.AlignHCenter)

        # The checklist is a model + the two ARM-interlock switches; its view lives on
        # the Diagnostics page. Parented here (hidden) until the main window moves it.
        self.checklist = ChecklistPanel(self)
        self.checklist.setVisible(False)
        self.checklist.summary_changed.connect(self.set_preflight)
        self.mini_feed = MiniFeed(self)
        self.mini_feed.setVisible(False)

        # Footer readouts: kept as plain (hidden) state holders so the telemetry wiring
        # and its tests are unchanged. The design shows this on the top bar / cards.
        self.lbl_spd = QLabel("SPD 0.00 m/s", self)
        self.lbl_alt = QLabel("ALT 0.00 m", self)
        self.lbl_mode = QLabel("MODE: DISCONNECTED", self)
        self.lbl_armed = QLabel("DISARMED", self)
        self.lbl_armed.setObjectName("navFooterDisarmed")
        for w in (self.lbl_spd, self.lbl_alt, self.lbl_mode, self.lbl_armed):
            w.setVisible(False)

    # ── preflight counter ────────────────────────────────────────────
    def set_preflight(self, passed: int, total: int, ready: bool) -> None:
        self.pre.val.setText(f"{passed}/{total}")
        self.pre.val.setProperty("ready", bool(ready))
        self.pre.val.style().unpolish(self.pre.val)
        self.pre.val.style().polish(self.pre.val)

    # ── state holders (API compatibility) ────────────────────────────
    def set_flight_state(self, flight_mode: str, armed: bool) -> None:
        self.lbl_mode.setText(f"MODE: {flight_mode or 'DISCONNECTED'}")
        self.lbl_armed.setText("ARMED" if armed else "DISARMED")
        self.lbl_armed.setObjectName("navFooterArmed" if armed else "navFooterDisarmed")

    def set_instruments(self, speed_ms: float, altitude_m: float) -> None:
        self.lbl_spd.setText(f"SPD {speed_ms:.2f} m/s")
        self.lbl_alt.setText(f"ALT {altitude_m:.2f} m")

    def set_wide(self, wide: bool) -> None:
        self._wide = bool(wide)

    def is_wide(self) -> bool:
        return self._wide

    def set_mini_feed_wanted(self, wanted: bool) -> None:
        self._mini_wanted = wanted

    def is_compact(self) -> bool:
        return self._compact

    # ── navigation ───────────────────────────────────────────────────
    def select_tab(self, idx: int):
        btn = self.btn_group.button(idx)
        if btn:
            btn.setChecked(True)
            self.view_changed.emit(idx)
