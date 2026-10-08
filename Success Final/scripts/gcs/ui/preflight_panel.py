"""
================================================================================
MODULE: preflight_panel.py
PURPOSE: The preflight checklist card in the left navigation rail
================================================================================

Sits directly under the "Parameters" entry, behind a horizontal line:

  ───────────────────────────
  PREFLIGHT ▾            5 of 7 ready     heading + ONE summary
  ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬                one thin bar, one colour for the overall state
  Fix first: Camera feed                  the first thing to fix - click to go there
  ✓  Link to the vehicle            OK    a table: icon | name | status, fixed columns
  ✓  Battery                       88%
  ✕  Camera feed                 IDLE
  –  Vision tracking           waiting

The icon carries the meaning by SHAPE, not only by colour: ✓ passed (green, quiet),
✕ a required check that failed (red), ! a failed information-only check (amber),
– cannot tell yet (grey, "waiting"; never counted as a pass). Hover any line for the
reason. The one manual line, "Heading fixed", is a tick box in the same table.

HOW MUCH SHOWS depends on the height the rail has to spare (ui/sidebar_nav.py asks
for a level): 0 nothing, 1 heading + summary, 2 + bar + next step, 3 + the required
lines, 4 + the information lines. The card is deliberately compact and ranks BELOW
the camera thumbnail: it only ever uses height the thumbnail does not need. It starts
COLLAPSED to the heading line (level 1); clicking the heading opens the list.
The logic lives in core/preflight.py; this widget only draws it.
================================================================================
"""

from __future__ import annotations

from typing import Dict, List

from PyQt5.QtCore import QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter
from PyQt5.QtWidgets import QCheckBox, QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget

from core.preflight import FAIL, PASS, Check, blockers
from ui.scaling import fit_min_width, px
from ui.styles import PALETTE


def status_colour(c: Check) -> str:
    """Icon / status colour for a check. Grey for "cannot tell yet": waiting is not an alarm."""
    if c.status == PASS:
        return PALETTE["ok"]
    if c.status == FAIL:
        return PALETTE["danger"] if c.required else PALETTE["warn"]
    return PALETTE["text_muted"]


def status_icon(c: Check) -> str:
    """The shape that says the same thing as the colour."""
    if c.status == PASS:
        return "\u2713"                                   # check mark
    if c.status == FAIL:
        return "\u2715" if c.required else "!"            # cross for a blocker, ! for a warning
    return "\u2013"                                       # en dash: waiting


def status_text(c: Check) -> str:
    """Right-hand column text. "--" never reaches the screen: it was ambiguous."""
    return "waiting" if c.value in ("", "--") and c.status not in (PASS, FAIL) else c.value


def overall_colour(checks: List[Check]) -> str:
    bad = blockers(checks)
    if not bad:
        return PALETTE["ok"]
    return PALETTE["danger"] if any(b.status == FAIL for b in bad) else PALETTE["warn"]


class ProgressStrip(QWidget):
    """One thin bar: how many required checks have passed, in a single colour for the overall state."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(px(4))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._fraction = 0.0
        self._colour = PALETTE["text_muted"]

    def set_checks(self, checks: List[Check]) -> None:
        req = [c for c in checks if c.required]
        self._fraction = (sum(1 for c in req if c.ok) / len(req)) if req else 0.0
        self._colour = overall_colour(checks)
        self.update()

    def fraction(self) -> float:
        return self._fraction

    def colour(self) -> str:
        return self._colour

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = self.height() / 2.0
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(PALETTE["border"]))
        p.drawRoundedRect(QRectF(0, 0, self.width(), self.height()), r, r)
        if self._fraction > 0:
            p.setBrush(QColor(self._colour))
            p.drawRoundedRect(QRectF(0, 0, max(self.height(), self.width() * self._fraction), self.height()), r, r)
        p.end()


class ActionLabel(QLabel):
    """One line, elided with "...", that jumps somewhere when clicked (when there is somewhere to go)."""
    clicked = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._clickable = False
        self._colour = PALETTE["text_dim"]
        self._hover = False
        self.setMinimumWidth(px(20))
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.setMouseTracking(True)

    def set_state(self, text: str, tooltip: str, colour: str, clickable: bool) -> None:
        self._colour, self._clickable = colour, clickable
        self.setText(text)
        self.setToolTip(tooltip)
        self.setCursor(Qt.PointingHandCursor if clickable else Qt.ArrowCursor)
        self.update()

    def is_clickable(self) -> bool:
        return self._clickable

    def enterEvent(self, _e):
        self._hover = True
        self.update()

    def leaveEvent(self, _e):
        self._hover = False
        self.update()

    def mousePressEvent(self, ev):
        if self._clickable and ev.button() == Qt.LeftButton:
            self.clicked.emit()
            ev.accept()
        else:
            super().mousePressEvent(ev)

    def paintEvent(self, _ev):
        p = QPainter(self)
        f = QFont(self.font())
        f.setUnderline(self._clickable and self._hover)
        p.setFont(f)
        p.setPen(QColor(self._colour))
        p.drawText(self.rect(), Qt.AlignLeft | Qt.AlignVCenter,
                   p.fontMetrics().elidedText(self.text(), Qt.ElideRight, self.width()))
        p.end()


class CheckRow(QWidget):
    """icon  |  name  |  status      - three fixed columns, so every row lines up"""

    def __init__(self, parent=None):
        super().__init__(parent)
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(px(5))
        self.icon = QLabel(self)
        self.icon.setObjectName("railCheckIcon")
        self.icon.setFixedWidth(px(12))
        self.icon.setAlignment(Qt.AlignCenter)
        h.addWidget(self.icon, 0, Qt.AlignVCenter)
        self.lbl_name = QLabel("", self)
        self.lbl_name.setObjectName("railCheck")
        h.addWidget(self.lbl_name, 1)
        self.lbl_value = QLabel("", self)
        self.lbl_value.setObjectName("railCheckValue")
        self.lbl_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        fit_min_width(self.lbl_value, ["NO SIGNAL", "waiting"], h_pad_px=2)
        h.addWidget(self.lbl_value, 0)
        self.setMinimumHeight(px(17))
        self.state = ""

    def set_check(self, c: Check) -> None:
        colour = status_colour(c)
        self.state = c.status
        self.icon.setText(status_icon(c))
        self.icon.setStyleSheet(f"color: {colour}; font-weight: 800; font-size: {px(11)}px;")
        quiet = c.status == PASS or not c.required
        self.lbl_name.setText(c.label)
        self.lbl_name.setStyleSheet(f"color: {PALETTE['text_dim'] if quiet else PALETTE['text_bright']};")
        self.lbl_value.setText(status_text(c))
        self.lbl_value.setStyleSheet(f"color: {PALETTE['text_muted'] if c.status == PASS else colour};")
        self.setToolTip(c.detail or c.label)


class ManualRow(QWidget):
    """The one tick-box line, laid out in the same columns: box | name | SET / NOT SET."""

    def __init__(self, checkbox: QCheckBox, parent=None):
        super().__init__(parent)
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(px(6))
        h.addWidget(checkbox, 1)
        self.lbl_value = QLabel("", self)
        self.lbl_value.setObjectName("railCheckValue")
        self.lbl_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        fit_min_width(self.lbl_value, ["NO SIGNAL", "waiting"], h_pad_px=2)
        h.addWidget(self.lbl_value, 0)
        self.setMinimumHeight(px(17))

    def set_check(self, c: Check) -> None:
        colour = status_colour(c)
        self.lbl_value.setText("SET" if c.status == PASS else "NOT SET")
        self.lbl_value.setStyleSheet(f"color: {PALETTE['text_muted'] if c.status == PASS else colour};")


# Where "Fix first" takes you for each check: the workspace that shows or fixes it (indices as in sidebar_nav).
ACTION_TAB = {"link": 0, "battery": 4, "vision": 4, "position": 4, "video": 1, "map": 2, "heading": 0,
              "radxa": 4, "params": 8}


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
    # "Fix first" was clicked: the key of the check to fix (see ACTION_TAB).
    action_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("railChecklist")
        root = QVBoxLayout(self)
        root.setContentsMargins(px(8), px(4), px(8), px(3))
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
        self.lbl_summary = QLabel("", self.header)          # "5 of 7 ready" - the one summary
        self.lbl_summary.setObjectName("railSummary")
        self.lbl_summary.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        fit_min_width(self.lbl_summary, ["10 of 10 ready", "READY"], h_pad_px=2)
        self.lbl_summary.setFixedHeight(px(18))
        hh.addWidget(self.lbl_summary)
        root.addWidget(self.header)

        # bar + the first thing to fix
        self.summary_block = QWidget(self)
        sb = QVBoxLayout(self.summary_block)
        sb.setContentsMargins(0, 0, 0, 0)
        sb.setSpacing(px(3))
        self.bar = ProgressStrip(self.summary_block)
        sb.addWidget(self.bar)
        self.lbl_detail = ActionLabel(self.summary_block)
        self.lbl_detail.setObjectName("railCheckDetail")
        self.lbl_detail.setFixedHeight(px(16))
        self.lbl_detail.clicked.connect(self._on_action_clicked)
        sb.addWidget(self.lbl_detail)
        self._action_key = ""
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
        self.manual_row = ManualRow(self.chk_heading, self.rows)
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
                self.manual_row.set_check(c)
                continue
            self._labels[c.key].set_check(c)
        bad = blockers(checks)
        self.bar.set_checks(checks)

        required = [c for c in checks if c.required]
        passed = sum(1 for c in required if c.ok)
        colour = overall_colour(checks)
        # One plain summary instead of a pill: nothing here is drawn twice.
        self.lbl_summary.setText("READY" if ready else f"{passed} of {len(required)} ready")
        self.lbl_summary.setToolTip(summary)
        self.lbl_summary.setStyleSheet(
            f"color: {colour if ready else PALETTE['text_dim']}; font-size: {px(10)}px; font-weight: 700;")
        if ready:
            self._action_key = ""
            self.lbl_detail.set_state("All required checks passed.", summary, PALETTE["ok"], False)
        else:
            first = bad[0]
            self._action_key = first.key
            self.lbl_detail.set_state(
                f"Fix first: {first.label}",
                (first.detail + "\n" if first.detail else "") + "Click to open the page that shows it.",
                PALETTE["accent"], True)

    def _on_action_clicked(self) -> None:
        if self._action_key:
            self.action_requested.emit(self._action_key)

    # ── internals ───────────────────────────────────────────────────

    def _build(self, checks: List[Check]) -> None:
        for c in checks:
            if c.manual:
                self._rows_layout.addWidget(self.manual_row)
                continue
            holder, layout = (self.rows, self._rows_layout) if c.required else (self.info_rows, self._info_layout)
            row = CheckRow(holder)
            self._labels[c.key] = row
            layout.addWidget(row)
        self._built = True
        self.lines_built.emit()
