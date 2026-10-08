"""
================================================================================
MODULE: sidebar_nav.py
PURPOSE: Vertical Workspace Navigation Rail with Active Tab State Indicators
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Primary Navigation Rail)
  * Communicates:  drone_gcs.py main window QStackedWidget
  * Upstream:      Pilot mouse clicks on workspace category buttons
  * Downstream:    Workspace view index switcher (PyQt QStackedWidget)

DATA FLOW & INTERFACES:
  * Navigation:    Control (0), FPV (1), SLAM (2), Motors (3), Diagnostics (4),
                   Diagnostics (4), Terminal / CLI (5).
  * Outbound Qt:   view_changed(int index) signal to change active view.

KEY LOGIC & FAILSAFES:
  * Mutually Exclusive Selection: Employs QButtonGroup with exclusive selection
    so exactly one workspace is marked active with a cyan indicator accent.
  * Keyboard & Mouse Responsive: Instantaneous zero-latency workspace swapping
    without re-instantiating heavy sub-widgets.
  * Dark Aerospace Aesthetics: High-contrast GitHub dark mode styling with hover effects.

USAGE:
  nav = SidebarNav()
  nav.view_changed.connect(stacked_widget.setCurrentIndex)
================================================================================
"""

from __future__ import annotations
from typing import Optional

from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QButtonGroup
)
from ui.mini_feed import MiniFeed
from ui.preflight_panel import ChecklistPanel
from ui.scaling import px, scale_qss


class SidebarNav(QFrame):
    """Vertical sidebar navigation bar with active indicators."""

    view_changed = pyqtSignal(int)

    def __init__(self, parent: Optional[QFrame] = None):
        super().__init__(parent)
        # 176, not 162: adding units pushed "ALT 0.00 m" flush against the
        # rail's right edge, which would elide at a higher DPI.
        self._wide = True
        self.setFixedWidth(px(self.WIDE_PX))
        # Scoped to the rail itself. As a bare `QFrame` rule this also matched
        # every descendant - QLabel derives from QFrame - so each label in the
        # rail painted its own right-hand border, scattering stray vertical
        # lines through the footer readouts.
        self.setObjectName("navRail")
        # Routed through scale_qss() rather than set as a raw literal - an
        # inline setStyleSheet() call never passes through the app's central
        # scaling pipeline on its own, so every px value here would otherwise
        # stay fixed regardless of GCS_UI_SCALE while the rest of the station
        # scales around it (the same class of bug once found and fixed in the
        # header's telemetry badges).
        self.setStyleSheet(scale_qss("""
            QFrame#navRail {
                background-color: #0d1117;
                border-right: 1px solid #30363d;
            }
            QPushButton {
                background-color: transparent;
                color: #8b949e;
                border: none;
                border-left: 3px solid transparent;
                border-radius: 0px;
                text-align: left;
                padding: 4px 14px;
                min-height: 21px;
                font-size: 11px;
                font-weight: 600;
            }
            /* Compact density: tighter rows when the rail is short (a small window
               at a large UI scale, with the alarm card showing). Same buttons,
               less vertical padding - nothing is hidden or overlapped. */
            QPushButton[compact="true"] {
                padding: 2px 14px;
                min-height: 17px;
            }
            QPushButton:hover {
                background-color: #161b22;
                color: #c9d1d9;
            }
            QPushButton:checked {
                background-color: #161b22;
                color: #58a6ff;
                border-left: 3px solid #58a6ff;
            }
            QLabel#navInstIdle {
                color: #8b949e;
                font-size: 10px;
                font-weight: bold;
                font-family: 'Noto Sans Mono', monospace;
                padding: 2px 0;
            }
            QLabel#navInstLive {
                color: #3fb950;
                font-size: 10px;
                font-weight: bold;
                font-family: 'Noto Sans Mono', monospace;
                padding: 2px 0;
            }
            QFrame#navFootRule {
                background-color: #30363d;
                min-height: 1px;
                max-height: 1px;
                border: none;
                margin: 0 10px;
            }
            QFrame#navInstRule {
                background-color: #30363d;
                min-width: 1px;
                max-width: 1px;
                border: none;
            }
            QLabel#navFooterMode {
                color: #58a6ff;
                background-color: rgba(31, 111, 235, 0.13);
                border: 1px solid #1f6feb;
                border-radius: 4px;
                font-size: 9px;
                font-weight: bold;
                letter-spacing: 0.6px;
                padding: 4px 6px;
            }
            QLabel#navFooterArmed {
                color: #3fb950;
                background-color: rgba(35, 134, 54, 0.13);
                border: 1px solid #238636;
                border-radius: 4px;
                font-size: 9px;
                font-weight: 900;
                letter-spacing: 0.6px;
                padding: 4px 6px;
            }
            QLabel#railCheckTitle {
                color: #6e7681;
                font-size: 9px;
                font-weight: 800;
                letter-spacing: 1px;
            }
            QLabel#railCheck {
                font-size: 10px;
            }
            QLabel#railCheckValue {
                font-size: 9px;
                font-weight: 700;
                letter-spacing: 0.3px;
            }
            QLabel#railCheckDetail {
                font-size: 10px;
                font-weight: 600;
            }
            QLabel#railSummary {
                font-size: 10px;
                font-weight: 700;
            }
            QCheckBox#cfgToggle {
                font-size: 10px;
                spacing: 7px;
            }
            QCheckBox#cfgToggle::indicator {
                width: 10px;
                height: 10px;
                border-radius: 6px;
            }
            QCheckBox#cfgToggle::indicator:unchecked {
                border: 1px solid #d29922;
                background-color: transparent;
            }
            QLabel#navFooterDisarmed {
                color: #f85149;
                background-color: rgba(218, 54, 51, 0.13);
                border: 1px solid #da3633;
                border-radius: 4px;
                font-size: 9px;
                font-weight: 900;
                letter-spacing: 0.6px;
                padding: 4px 6px;
            }
        """))

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 8, 0, 2)
        layout.setSpacing(2)

        self.btn_group = QButtonGroup(self)
        self.btn_group.setExclusive(True)

        items = [
            ("Control", 0),
            ("FPV Camera", 1),
            ("Tactical SLAM", 2),
            ("Motor Actuators", 3),
            ("Diagnostics", 4),
            ("Flight Terminal", 5),
            ("Flight Logs", 6),
            ("Configuration", 7),
            ("Parameters", 8),
        ]

        for text, idx in items:
            btn = QPushButton(text, self)
            btn.setCheckable(True)
            if idx == 0:
                btn.setChecked(True)
            self.btn_group.addButton(btn, idx)
            layout.addWidget(btn)

        self.btn_group.buttonClicked[int].connect(self._on_button_clicked)

        # Preflight checklist: directly under the last workspace entry, behind a
        # horizontal rule. Whether its lines fit is decided in _apply_checklist_level.
        layout.addSpacing(px(10))       # air between the last workspace entry and the checklist
        self.checklist = ChecklistPanel(self)
        layout.addWidget(self.checklist)
        # The lines do not exist until the first update; once they do, re-check
        # what fits (a resize event alone would not come).
        self.checklist.lines_built.connect(self._refit_after_checklist)
        # Collapsed by default (heading + pill only); a click re-decides what fits.
        self.checklist.expanded_changed.connect(lambda _e: self._apply_checklist_level())

        layout.addStretch()
        layout.addSpacing(px(12))       # a guaranteed gap between the checklist and the camera thumbnail

        # Camera thumbnail in the rail's empty space (see ui/mini_feed.py). Shown
        # only on the workspaces that have no camera of their own, and only when
        # the rail is tall enough - it must never push the nav buttons or the
        # footer, or force the window taller than its supported minimum.
        self.mini_feed = MiniFeed(self)
        self.mini_feed.setVisible(False)
        layout.addWidget(self.mini_feed)
        self._mini_wanted = False
        self._fit_recheck_pending = False
        self._fit_rechecks = 0
        self._compact = False
        self._need_normal = 0

        # Footer: live flight state. This replaced a two-line build-version
        # string - a constant that told the operator nothing during a flight,
        # sitting in the one spot visible from every workspace.
        # Each in its own bordered pill and centred in the rail, so the two
        # read as separate state indicators rather than one wrapped sentence.
        # A rule closes off the workspace list: below it is state the vehicle
        # reports, above it is navigation. Without the separation the readouts
        # read as two more entries in the list.
        foot_rule = QFrame(self)
        foot_rule.setObjectName("navFootRule")
        foot_rule.setFixedHeight(1)
        layout.addWidget(foot_rule)

        foot = QVBoxLayout()
        foot.setContentsMargins(10, 8, 10, 4)
        foot.setSpacing(5)

        # Speed and altitude share one line, split by a rule. They belong with
        # the other persistent state on the rail rather than in the header:
        # visible from every workspace, and out of the way of the controls.
        inst_row = QHBoxLayout()
        inst_row.setContentsMargins(0, 0, 0, 0)
        inst_row.setSpacing(6)

        self.lbl_spd = QLabel("SPD 0.00 m/s", self)
        self.lbl_spd.setObjectName("navInstIdle")
        self.lbl_spd.setAlignment(Qt.AlignCenter)
        inst_row.addWidget(self.lbl_spd, 1)

        inst_rule = QFrame(self)
        inst_rule.setObjectName("navInstRule")
        inst_rule.setFixedWidth(1)
        inst_row.addWidget(inst_rule)

        self.lbl_alt = QLabel("ALT 0.00 m", self)
        self.lbl_alt.setObjectName("navInstIdle")
        self.lbl_alt.setAlignment(Qt.AlignCenter)
        inst_row.addWidget(self.lbl_alt, 1)

        foot.addLayout(inst_row)

        self.lbl_mode = QLabel("MODE: DISCONNECTED", self)
        self.lbl_mode.setObjectName("navFooterMode")
        self.lbl_mode.setAlignment(Qt.AlignCenter)
        foot.addWidget(self.lbl_mode)

        self.lbl_armed = QLabel("DISARMED", self)
        self.lbl_armed.setObjectName("navFooterDisarmed")
        self.lbl_armed.setAlignment(Qt.AlignCenter)
        foot.addWidget(self.lbl_armed)

        layout.addLayout(foot)

    def set_flight_state(self, flight_mode: str, armed: bool) -> None:
        """Update the footer's mode / arm readout.

        Green for armed, red for disarmed - the ready-to-fly convention, kept
        identical here and in the header badge so the two never disagree.
        """
        self.lbl_mode.setText(f"MODE: {flight_mode or 'DISCONNECTED'}")
        self.lbl_armed.setText("ARMED" if armed else "DISARMED")
        name = "navFooterArmed" if armed else "navFooterDisarmed"
        if self.lbl_armed.objectName() != name:
            self.lbl_armed.setObjectName(name)
            # Qt caches style by objectName; without a re-polish the label keeps
            # the colour it had before the state changed.
            self.lbl_armed.style().unpolish(self.lbl_armed)
            self.lbl_armed.style().polish(self.lbl_armed)

    def set_instruments(self, speed_ms: float, altitude_m: float) -> None:
        """Update the rail's speed / altitude readouts.

        Plain grey while a value is effectively zero, green once it is
        genuinely changing - the dead-bands are the ones the flight logic
        already trusts (0.05 m/s and 0.05 m are inside VIO noise), so a
        stationary aircraft does not flicker.
        """
        for lbl, caption, unit, value in ((self.lbl_spd, "SPD", "m/s", speed_ms),
                                          (self.lbl_alt, "ALT", "m", altitude_m)):
            lbl.setText(f"{caption} {value:.2f} {unit}")
            name = "navInstLive" if abs(value) > 0.05 else "navInstIdle"
            if lbl.objectName() != name:
                lbl.setObjectName(name)
                # Qt caches style by objectName; re-polish or the colour sticks.
                lbl.style().unpolish(lbl)
                lbl.style().polish(lbl)

    # The rail is wider (a bigger camera thumbnail, more room for the checklist) when the window can spare it, and
    # a little narrower when it cannot: a small window or a large UI scale needs that width for the page itself.
    WIDE_PX = 200
    NARROW_PX = 184
    WIDE_MIN_WINDOW_PX = 1280        # logical (pre-scale) window width from which the wide rail is used

    def set_wide(self, wide: bool) -> None:
        if bool(wide) == self._wide:
            return
        self._wide = bool(wide)
        self.setFixedWidth(px(self.WIDE_PX if wide else self.NARROW_PX))
        self.updateGeometry()
        self._apply_checklist_level()

    def is_wide(self) -> bool:
        return self._wide

    # Workspaces with no camera of their own: Motor Actuators, Diagnostics,
    # Flight Terminal, Flight Logs, Configuration, Parameters.
    MINI_FEED_TABS = frozenset({3, 4, 5, 6, 7, 8})

    def set_mini_feed_wanted(self, wanted: bool) -> None:
        self._mini_wanted = wanted
        self._apply_checklist_level()

    def _apply_checklist_level(self) -> None:
        """Decide, in this order, what the rail's spare height is used for:

          1. the camera thumbnail - exactly as before the checklist existed;
          2. the checklist card: the heading line, or - when the operator has opened
             it - as much of the list as fits in what is left.

        Both are measured against the rail's own minimum (buttons + footer) with
        the checklist gone, so neither can push the window taller than its
        supported minimum: at the smallest sizes the checklist is simply not shown."""
        panel = self.checklist
        self.mini_feed.setVisible(False)
        panel.set_level(0)
        base = self.layout().sizeHint().height()
        feed_h = self.mini_feed.wanted_height() if getattr(self, "_mini_wanted", False) else 0
        feed_fits = feed_h > 0 and self.height() >= base + feed_h
        self.mini_feed.setVisible(feed_fits)
        # Largest card whose real minimum height still fits. Checked against the
        # layout's own minimum (not an estimate): a wrapped label's minimum can be a
        # line taller than its size hint, which cost one pixel at 1280x760.
        level = 0
        # Collapsed: the heading line only, so the camera thumbnail keeps the height.
        for lv in ((4, 3, 2, 1) if panel.is_expanded() else (1,)):
            panel.set_level(lv)
            # Qt recomputes a layout lazily (on a posted event). Force the card's own
            # layout, then the rail's, to refresh now - otherwise the minimum measured
            # here is still the previous level's and nothing ever seems to fit.
            panel.layout().invalidate()
            panel.layout().activate()
            self.layout().invalidate()
            if self.layout().minimumSize().height() <= self.height():
                level = lv
                break
        panel.set_level(level)
        self.layout().invalidate()
        # The rail's real minimum can still grow a few pixels once Qt has finished
        # laying out (density change, wrapped text). Re-check once it has settled and
        # step the card down if it no longer fits - the rail must never be shorter
        # than it needs.
        if level > 0 and not self._fit_recheck_pending and self._fit_rechecks < 3:
            self._fit_recheck_pending = True
            QTimer.singleShot(0, self._recheck_checklist_fit)

    def _recheck_checklist_fit(self) -> None:
        self._fit_recheck_pending = False
        if self.checklist.level() > 0 and self.layout().minimumSize().height() > self.height():
            self._fit_rechecks += 1
            self._apply_checklist_level()
        else:
            self._fit_rechecks = 0

    def _refit_after_checklist(self) -> None:
        self._apply_density()
        self._apply_checklist_level()

    def _apply_mini_feed(self) -> None:
        """Kept for callers: the thumbnail and the checklist share the rail's spare
        height, so they are decided together (see _apply_checklist_level)."""
        self._apply_checklist_level()

    def _set_compact(self, compact: bool) -> None:
        if compact == self._compact:
            return
        self._compact = compact
        for b in self.btn_group.buttons():
            b.setProperty("compact", compact)
            b.style().unpolish(b)
            b.style().polish(b)
            b.updateGeometry()

    def is_compact(self) -> bool:
        return self._compact

    def _apply_density(self) -> None:
        """Tighten the nav rows when the rail is shorter than its normal height.

        The height the rail needs is measured while it is in normal density and
        remembered, so the decision does not depend on the (smaller) compact
        measurement - that is what keeps it from flapping between the two.
        """
        # Measured without the checklist (it is optional height, see
        # _apply_checklist_level), so it can never decide the rail's density.
        keep = self.checklist.level()
        self.checklist.set_level(0)
        if not self._compact:
            self._need_normal = self.layout().minimumSize().height()
        self._set_compact(self.height() < self._need_normal)
        self.checklist.set_level(keep)

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._apply_density()
        self._apply_checklist_level()

    def _on_button_clicked(self, idx: int):
        self.view_changed.emit(idx)

    def select_tab(self, idx: int):
        btn = self.btn_group.button(idx)
        if btn:
            btn.setChecked(True)
            self.view_changed.emit(idx)
