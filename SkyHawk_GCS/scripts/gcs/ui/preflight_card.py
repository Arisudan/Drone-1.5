"""
================================================================================
MODULE: preflight_card.py
PURPOSE: Diagnostics workspace's "Preflight checklist" view (progress ring + rows)
================================================================================

A pure view over ``ChecklistPanel``. The panel stays the model: the preflight
controller keeps feeding it, and it still owns the "Block ARM until ready" and
"Heading fixed" switches the ARM interlock reads. This card re-parents those two
switches into itself so they sit with the list they govern.

FIX FIRST emits ``action_requested(key)`` - the same signal, with the same keys
(see ui.preflight_panel.ACTION_TAB), as the old rail's "Fix first" link.
================================================================================
"""

from __future__ import annotations

from typing import Dict, List

from PyQt5.QtCore import QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (QFrame, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget)

from core.preflight import FAIL, PASS, Check, blockers
from ui.preflight_panel import ChecklistPanel, status_text
from ui.scaling import px

GREEN, AMBER, RED, MUTE = "#46d06a", "#ffb02e", "#ff5a4f", "#6f7772"


class ProgressRing(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(px(96), px(96))
        self._passed, self._total, self._colour = 0, 7, AMBER

    def set_progress(self, passed: int, total: int, colour: str) -> None:
        self._passed, self._total, self._colour = passed, max(1, total), colour
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = px(9)
        r = QRectF(self.rect()).adjusted(w, w, -w, -w).adjusted(px(1), px(1), -px(1), -px(1))
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QColor(255, 255, 255, 26), w))
        p.drawEllipse(r)
        frac = self._passed / self._total
        if frac > 0:
            pen = QPen(QColor(self._colour), w)
            pen.setCapStyle(Qt.RoundCap)
            p.setPen(pen)
            p.drawArc(r, 90 * 16, int(-360 * 16 * frac))
        else:
            # the design shows a single dot at 12 o'clock when nothing has passed
            pen = QPen(QColor(self._colour), w)
            pen.setCapStyle(Qt.RoundCap)
            p.setPen(pen)
            p.drawPoint(int(r.center().x()), int(r.top()))
        f = QFont(self.font())
        f.setPixelSize(px(22))
        f.setBold(True)
        p.setFont(f)
        p.setPen(QColor(self._colour))
        p.drawText(self.rect(), Qt.AlignCenter, f"{self._passed}/{self._total}")
        p.end()


class _Dot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(px(10), px(10))
        self._c = MUTE

    def set_colour(self, c: str) -> None:
        self._c = c
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        if self._c in (RED, GREEN):
            g = QColor(self._c)
            g.setAlpha(90)
            p.setBrush(g)
            p.setPen(Qt.NoPen)
            p.drawEllipse(self.rect())
        p.setBrush(QColor(self._c))
        p.setPen(Qt.NoPen)
        p.drawEllipse(self.rect().adjusted(px(1), px(1), -px(1), -px(1)))
        p.end()


class _Row(QWidget):
    def __init__(self, key: str, on_fix, parent=None):
        super().__init__(parent)
        self.key = key
        lay = QHBoxLayout(self)
        lay.setContentsMargins(px(4), px(11), px(4), px(11))
        lay.setSpacing(px(14))
        self.dot = _Dot(self)
        lay.addWidget(self.dot)
        self.lbl_name = QLabel("", self)
        self.lbl_name.setObjectName("pfName")
        self.lbl_name.setFixedWidth(px(130))
        lay.addWidget(self.lbl_name)
        self.lbl_detail = QLabel("", self)
        self.lbl_detail.setObjectName("pfDetail")
        lay.addWidget(self.lbl_detail, 1)
        self.btn_fix = QPushButton("FIX FIRST", self)
        self.btn_fix.setObjectName("pfFix")
        self.btn_fix.clicked.connect(lambda: on_fix(self.key))
        lay.addWidget(self.btn_fix)

    def set_check(self, c: Check) -> None:
        colour = GREEN if c.status == PASS else (RED if (c.status == FAIL and c.required) else
                                                 AMBER if c.status == FAIL else MUTE)
        self.dot.set_colour(colour)
        self.lbl_name.setText(c.label)
        self.lbl_detail.setText(c.detail if (c.detail and c.status != PASS) else status_text(c))
        self.lbl_detail.setToolTip(c.detail or c.label)
        self.btn_fix.setVisible(c.status != PASS)

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setPen(QPen(QColor(255, 255, 255, 13), 1))
        p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)
        p.end()


class PreflightCard(QWidget):
    """View + the two ARM-interlock switches (re-parented from the model)."""

    action_requested = pyqtSignal(str)
    # (passed, total, ready) so the hosting panel can show "N of M ready" in its header.
    summary_changed = pyqtSignal(int, int, bool)

    def __init__(self, model: ChecklistPanel, parent=None):
        super().__init__(parent)
        self.model = model
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(px(6))

        head = QHBoxLayout()
        head.setSpacing(px(10))
        self.ring = ProgressRing(self)
        head.addWidget(self.ring)
        col = QVBoxLayout()
        col.setSpacing(px(2))
        col.addStretch(1)
        self.lbl_title = QLabel("Not ready for flight", self)
        self.lbl_title.setObjectName("pfTitle")
        self.lbl_sub = QLabel("0 of 7 checks passed", self)
        self.lbl_sub.setObjectName("pfSub")
        col.addWidget(self.lbl_title)
        col.addWidget(self.lbl_sub)
        col.addStretch(1)
        head.addLayout(col, 1)
        lay.addLayout(head)

        self.rows_box = QVBoxLayout()
        self.rows_box.setSpacing(0)
        lay.addLayout(self.rows_box)
        self._rows: Dict[str, _Row] = {}

        # the interlock switches live with the list they govern
        sw = QHBoxLayout()
        sw.setSpacing(px(24))
        sw.setContentsMargins(px(4), px(8), px(4), 0)
        for chk in (model.chk_enforce, model.chk_heading):
            chk.setParent(self)
            sw.addWidget(chk)
        sw.addStretch(1)
        lay.addLayout(sw)
        lay.addStretch(1)

        model.checks_updated.connect(self.update_checks)

    def update_checks(self, checks: List[Check], ready: bool, summary: str) -> None:
        shown = [c for c in checks if not c.manual]
        if [c.key for c in shown] != list(self._rows):
            for r in self._rows.values():
                r.setParent(None)
                r.deleteLater()
            self._rows = {}
            for c in shown:
                row = _Row(c.key, self.action_requested.emit, self)
                self._rows[c.key] = row
                self.rows_box.addWidget(row)
        for c in shown:
            self._rows[c.key].set_check(c)
        required = [c for c in checks if c.required]
        passed = sum(1 for c in required if c.ok)
        total = len(required)
        bad = blockers(checks)
        colour = GREEN if ready else (RED if any(b.status == FAIL for b in bad) else AMBER)
        colour = GREEN if ready else AMBER
        self.ring.set_progress(passed, total, colour)
        self.lbl_title.setText("Ready for flight" if ready else "Not ready for flight")
        self.lbl_sub.setText(f"{passed} of {total} checks passed")
        self.lbl_sub.setToolTip(summary)
        self.summary_changed.emit(passed, total, bool(ready))
