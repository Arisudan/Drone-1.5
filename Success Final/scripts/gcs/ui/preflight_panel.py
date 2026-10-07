"""
================================================================================
MODULE: preflight_panel.py
PURPOSE: The preflight checklist card in the left navigation rail
================================================================================

Sits directly under the "Parameters" entry, behind a horizontal line:

  ───────────────────────────
  PREFLIGHT            [3 TO FIX]      heading + a status pill
  ▰▰▰▰▱▱▰                              one segment per required check
  Next: Camera feed - video is idle.   the first thing to fix, in words
  ● Link to the vehicle          OK    one line per check: status dot, name,
  ● Battery                     88%    and the value that decided it
  ● Camera feed                IDLE
  ...

Passing lines are quiet; a failed required check is red and brighter; a check that
cannot be judged is amber (never counted as a pass). Hover any line for the reason.
The one manual line, "Heading fixed", is a tick box.

HOW MUCH SHOWS depends on the height the rail has to spare (ui/sidebar_nav.py asks
for a level): 0 nothing, 1 heading + pill, 2 + bar + next step, 3 + the required
lines, 4 + the information lines. The card is deliberately compact and ranks BELOW
the camera thumbnail: it only ever uses height the thumbnail does not need. It starts
COLLAPSED to the heading line (level 1); clicking the heading opens the list.
The logic lives in core/preflight.py; this widget only draws it.
================================================================================
"""

from __future__ import annotations

from typing import Dict, List

from PyQt5.QtCore import QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter
from PyQt5.QtWidgets import QCheckBox, QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget

from core.preflight import FAIL, PASS, Check, blockers
from ui.scaling import px
from ui.styles import PALETTE


def status_colour(c: Check) -> str:
    """Dot / value colour for a check."""
    if c.status == PASS:
        return PALETTE["ok"]
    if c.status == FAIL:
        return PALETTE["danger"] if c.required else PALETTE["warn"]
    return PALETTE["warn"] if c.required else PALETTE["text_muted"]


class SegmentBar(QWidget):
    """One small rounded segment per required check: a glance at how close to ready."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(px(5))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._colours: List[str] = []

    def set_checks(self, checks: List[Check]) -> None:
        self._colours = [status_colour(c) for c in checks if c.required]
        self.update()

    def segment_colours(self) -> List[str]:
        return list(self._colours)

    def paintEvent(self, _ev):
        if not self._colours:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        n, gap = len(self._colours), px(2)
        w = (self.width() - gap * (n - 1)) / n
        for i, colour in enumerate(self._colours):
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(colour))
            p.drawRoundedRect(QRectF(i * (w + gap), 0, w, self.height()), self.height() / 2, self.height() / 2)
        p.end()


class CheckRow(QWidget):
    """status dot  |  name  |  value"""

    def __init__(self, parent=None):
        super().__init__(parent)
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(px(6))
        self.dot = QLabel(self)
        self.dot.setFixedSize(px(8), px(8))
        h.addWidget(self.dot, 0, Qt.AlignVCenter)
        self.lbl_name = QLabel("", self)
        self.lbl_name.setObjectName("railCheck")
        h.addWidget(self.lbl_name, 1)
        self.lbl_value = QLabel("", self)
        self.lbl_value.setObjectName("railCheckValue")
        self.lbl_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        h.addWidget(self.lbl_value, 0)
        self.setMinimumHeight(px(15))
        self.state = ""

    def set_check(self, c: Check) -> None:
        colour = status_colour(c)
        self.state = c.status
        self.dot.setStyleSheet(f"background-color: {colour}; border-radius: {px(4)}px; border: none;")
        quiet = c.status == PASS or not c.required
        self.lbl_name.setText(c.label)
        self.lbl_name.setStyleSheet(f"color: {PALETTE['text_dim'] if quiet else PALETTE['text_bright']};")
        self.lbl_value.setText(c.value)
        self.lbl_value.setStyleSheet(f"color: {PALETTE['text_muted'] if c.status == PASS else colour};")
        self.setToolTip(c.detail or c.label)


class ClickableHeader(QWidget):
    """The checklist's heading row: a click anywhere on it opens/closes the list."""
    clicked = pyqtSignal()

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self.clicked.emit()
            ev.accept()
        else:
            super().mousePressEvent(ev)


class ChecklistPanel(QWidget):
    heading_toggled = pyqtSignal(bool)
    # The operator opened or closed the list (collapsed = the heading line only).
    expanded_changed = pyqtSignal(bool)
    # The set of lines was created (first update): the rail must re-check what fits.
    lines_built = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("railChecklist")
        root = QVBoxLayout(self)
        root.setContentsMargins(px(10), px(4), px(10), px(3))
        root.setSpacing(px(3))

        # The horizontal line under the "Parameters" entry.
        self.rule = QFrame(self)
        self.rule.setObjectName("hDivider")
        self.rule.setFixedHeight(1)
        root.addWidget(self.rule)

        # heading + status pill
        self.header = ClickableHeader(self)
        self.header.setCursor(Qt.PointingHandCursor)
        self.header.clicked.connect(self.toggle_expanded)
        hh = QHBoxLayout(self.header)
        hh.setContentsMargins(0, px(1), 0, 0)
        hh.setSpacing(px(6))
        self.lbl_title = QLabel("PREFLIGHT", self.header)
        self.lbl_title.setObjectName("railCheckTitle")
        hh.addWidget(self.lbl_title)
        hh.addStretch(1)
        self.lbl_summary = QLabel("", self.header)          # the pill
        self.lbl_summary.setObjectName("railPill")
        self.lbl_summary.setAlignment(Qt.AlignCenter)
        hh.addWidget(self.lbl_summary)
        root.addWidget(self.header)

        # bar + the first thing to fix
        self.summary_block = QWidget(self)
        sb = QVBoxLayout(self.summary_block)
        sb.setContentsMargins(0, 0, 0, 0)
        sb.setSpacing(px(3))
        self.bar = SegmentBar(self.summary_block)
        sb.addWidget(self.bar)
        self.lbl_detail = QLabel("", self.summary_block)
        self.lbl_detail.setObjectName("railCheckDetail")
        self.lbl_detail.setWordWrap(True)
        sb.addWidget(self.lbl_detail)
        root.addWidget(self.summary_block)

        # one line per check
        self.rows = QWidget(self)
        self._rows_layout = QVBoxLayout(self.rows)
        self._rows_layout.setContentsMargins(0, px(1), 0, 0)
        self._rows_layout.setSpacing(px(1))
        root.addWidget(self.rows)

        # information-only lines (Radxa services, Parameters read): only with spare room
        self.info_rows = QWidget(self)
        self._info_layout = QVBoxLayout(self.info_rows)
        self._info_layout.setContentsMargins(0, 0, 0, 0)
        self._info_layout.setSpacing(px(1))
        root.addWidget(self.info_rows)

        self._labels: Dict[str, CheckRow] = {}
        self.chk_heading = QCheckBox("Heading fixed", self.rows)
        self.chk_heading.setObjectName("cfgToggle")
        self.chk_heading.setToolTip("Turn the drone once so the heading is fixed, then tick this. "
                                    "It clears itself whenever vision tracking is lost.")
        self.chk_heading.toggled.connect(self.heading_toggled)
        self._built = False
        self._level = 4
        # Collapsed by default: just the heading + pill, so the rail's height goes to
        # the camera thumbnail. A click on the heading opens the full list.
        self._expanded = False
        self._refresh_title()

    # ── public ──────────────────────────────────────────────────────

    def row(self, key: str) -> CheckRow:
        return self._labels[key]

    def heading_confirmed(self) -> bool:
        return self.chk_heading.isChecked()

    def clear_heading(self) -> None:
        if self.chk_heading.isChecked():
            self.chk_heading.setChecked(False)

    def is_expanded(self) -> bool:
        return self._expanded

    def set_expanded(self, expanded: bool) -> None:
        if expanded == self._expanded:
            return
        self._expanded = expanded
        self._refresh_title()
        self.expanded_changed.emit(expanded)

    def toggle_expanded(self) -> None:
        self.set_expanded(not self._expanded)

    def _refresh_title(self) -> None:
        self.lbl_title.setText("PREFLIGHT  " + ("\u25be" if self._expanded else "\u25b8"))
        self.header.setToolTip("Click to hide the checklist" if self._expanded
                               else "Click to show the checklist")

    # How much shows. The rail picks the highest level that fits its height.
    def set_level(self, level: int) -> None:
        self._level = level
        self.setVisible(level >= 1)
        self.summary_block.setVisible(level >= 2)
        self.rows.setVisible(level >= 3)
        self.info_rows.setVisible(level >= 4)

    def level(self) -> int:
        return self._level

    def height_for(self, level: int) -> int:
        """Height this panel needs at `level` (the rail's fit test). Summed from the parts'
        own size hints: a hidden widget reports no size of its own."""
        if level < 1:
            return 0
        m = self.layout().contentsMargins()
        sp = self.layout().spacing()
        parts = [1, self.header.sizeHint().height()]
        if level >= 2:
            parts.append(self.summary_block.sizeHint().height())
        if level >= 3:
            parts.append(self._rows_height(self._rows_layout))
        if level >= 4:
            parts.append(self._rows_height(self._info_layout))
        return sum(parts) + sp * (len(parts) - 1) + m.top() + m.bottom()

    @staticmethod
    def _rows_height(lay) -> int:
        hs = [lay.itemAt(i).widget().sizeHint().height() for i in range(lay.count())
              if lay.itemAt(i).widget() is not None]
        if not hs:
            return 0
        m = lay.contentsMargins()
        return sum(hs) + lay.spacing() * (len(hs) - 1) + m.top() + m.bottom()

    def update_checks(self, checks: List[Check], ready: bool, summary: str) -> None:
        if not self._built:
            self._build(checks)
        for c in checks:
            if c.manual:
                self.chk_heading.setToolTip(c.detail or self.chk_heading.toolTip())
                continue
            self._labels[c.key].set_check(c)
        bad = blockers(checks)
        self.bar.set_checks(checks)

        pill_colour = PALETTE["ok"] if ready else (PALETTE["danger"] if any(b.status == FAIL for b in bad) else PALETTE["warn"])
        self.lbl_summary.setText("READY" if ready else f"{len(bad)} TO FIX")
        self.lbl_summary.setToolTip(summary)
        self.lbl_summary.setStyleSheet(
            f"color: {pill_colour}; border: 1px solid {pill_colour}; border-radius: {px(7)}px; "
            f"padding: 1px {px(7)}px; font-size: {px(9)}px; font-weight: 800; letter-spacing: 0.6px; "
            f"background-color: rgba(0, 0, 0, 0.18);")
        if ready:
            self.lbl_detail.setText("All required checks passed.")
        else:
            first = bad[0]
            self.lbl_detail.setText(f"Next: {first.label}" + (f" - {first.detail}" if first.detail else ""))
        self.lbl_detail.setToolTip(summary)

    # ── internals ───────────────────────────────────────────────────

    def _build(self, checks: List[Check]) -> None:
        for c in checks:
            if c.manual:
                self._rows_layout.addWidget(self.chk_heading)
                continue
            holder, layout = (self.rows, self._rows_layout) if c.required else (self.info_rows, self._info_layout)
            row = CheckRow(holder)
            self._labels[c.key] = row
            layout.addWidget(row)
        self._built = True
        self.lines_built.emit()
