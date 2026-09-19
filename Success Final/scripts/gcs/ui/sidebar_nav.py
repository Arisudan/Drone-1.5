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

from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QButtonGroup
)


class SidebarNav(QFrame):
    """Vertical sidebar navigation bar with active indicators."""

    view_changed = pyqtSignal(int)

    def __init__(self, parent: Optional[QFrame] = None):
        super().__init__(parent)
        # 176, not 162: adding units pushed "ALT 0.00 m" flush against the
        # rail's right edge, which would elide at a higher DPI.
        self.setFixedWidth(176)
        # Scoped to the rail itself. As a bare `QFrame` rule this also matched
        # every descendant - QLabel derives from QFrame - so each label in the
        # rail painted its own right-hand border, scattering stray vertical
        # lines through the footer readouts.
        self.setObjectName("navRail")
        self.setStyleSheet("""
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
                padding: 10px 14px;
                font-size: 11px;
                font-weight: 600;
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
        """)

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
        ]

        for text, idx in items:
            btn = QPushButton(text, self)
            btn.setCheckable(True)
            if idx == 0:
                btn.setChecked(True)
            self.btn_group.addButton(btn, idx)
            layout.addWidget(btn)

        self.btn_group.buttonClicked[int].connect(self._on_button_clicked)

        layout.addStretch()

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

    def _on_button_clicked(self, idx: int):
        self.view_changed.emit(idx)

    def select_tab(self, idx: int):
        btn = self.btn_group.button(idx)
        if btn:
            btn.setChecked(True)
            self.view_changed.emit(idx)
