"""
================================================================================
MODULE: preflight_panel.py
PURPOSE: The preflight checklist, shown under the parameter table
================================================================================

A horizontal rule, a heading with the overall verdict, and the checks in three
columns so the whole list costs only a few lines of height. Pass is quiet grey,
a failed required check is red, a check that cannot be judged is amber - the
station's grey-first rule. The one manual row ("Heading fixed") is a tick box.
The logic lives in core/preflight.py; this widget only draws it.
================================================================================
"""

from __future__ import annotations

from typing import Dict, List

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QCheckBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from core.preflight import FAIL, PASS, Check
from ui.scaling import px
from ui.styles import PALETTE

COLUMNS = 3


class ChecklistPanel(QWidget):
    heading_toggled = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(px(4))

        self.rule = QFrame(self)
        self.rule.setObjectName("hDivider")
        self.rule.setFixedHeight(1)
        root.addWidget(self.rule)

        head = QHBoxLayout()
        head.setSpacing(px(12))
        title = QLabel("PREFLIGHT CHECKLIST", self)
        title.setObjectName("cardHeading")
        head.addWidget(title)
        self.lbl_summary = QLabel("", self)
        self.lbl_summary.setObjectName("preflightSummary")
        head.addWidget(self.lbl_summary, 1)
        root.addLayout(head)

        self.grid = QGridLayout()
        self.grid.setHorizontalSpacing(px(16))
        self.grid.setVerticalSpacing(px(2))
        for c in range(COLUMNS):
            self.grid.setColumnStretch(c, 1)
        root.addLayout(self.grid)

        self._rows: Dict[str, QLabel] = {}
        self._cells: Dict[str, QWidget] = {}
        self.chk_heading = QCheckBox("Heading fixed", self)
        self.chk_heading.setObjectName("cfgToggle")
        self.chk_heading.setToolTip("Turn the drone once so the heading is fixed, then tick this. "
                                    "It clears itself whenever vision tracking is lost.")
        self.chk_heading.toggled.connect(self.heading_toggled)
        self._built = False

    # ── public ──────────────────────────────────────────────────────

    def heading_confirmed(self) -> bool:
        return self.chk_heading.isChecked()

    def clear_heading(self) -> None:
        if self.chk_heading.isChecked():
            self.chk_heading.setChecked(False)

    def update_checks(self, checks: List[Check], ready: bool, summary: str) -> None:
        if not self._built:
            self._build(checks)
        for c in checks:
            if c.manual:
                self.chk_heading.setToolTip(c.detail or self.chk_heading.toolTip())
                continue
            lbl = self._rows[c.key]
            glyph, colour = self._look(c)
            lbl.setText(f"{glyph}  {c.label}")
            lbl.setStyleSheet(f"color: {colour};")
            lbl.setToolTip(c.detail or c.label)
        self.lbl_summary.setText(summary)
        self.lbl_summary.setProperty("ready", bool(ready))
        self.lbl_summary.style().unpolish(self.lbl_summary)
        self.lbl_summary.style().polish(self.lbl_summary)

    # ── internals ───────────────────────────────────────────────────

    def _build(self, checks: List[Check]) -> None:
        for n, c in enumerate(checks):
            if c.manual:
                w: QWidget = self.chk_heading
            else:
                w = QLabel("", self)
                w.setObjectName("cfgLabel")
                self._rows[c.key] = w
            self._cells[c.key] = w
            self.grid.addWidget(w, n // COLUMNS, n % COLUMNS)
        self._built = True

    @staticmethod
    def _look(c: Check):
        if c.status == PASS:
            return "✓", PALETTE["text_dim"]
        if c.status == FAIL:
            return "✕", PALETTE["danger"] if c.required else PALETTE["warn"]
        return "–", PALETTE["warn"] if c.required else PALETTE["text_muted"]
