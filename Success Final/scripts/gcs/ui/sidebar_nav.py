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
  * Navigation:    Cockpit PFD (0), Tactical SLAM (1), Video (2), Motors (3),
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
from typing import List, Optional

from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import QFrame, QVBoxLayout, QPushButton, QLabel, QButtonGroup


class SidebarNav(QFrame):
    """Vertical sidebar navigation bar with active indicators."""

    view_changed = pyqtSignal(int)

    def __init__(self, parent: Optional[QFrame] = None):
        super().__init__(parent)
        self.setFixedWidth(170)
        self.setStyleSheet("""
            QFrame {
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
                padding: 12px 14px;
                font-size: 12px;
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
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 8, 0, 8)
        layout.setSpacing(2)

        lbl_nav = QLabel("  WORKSPACES", self)
        lbl_nav.setStyleSheet("color: #484f58; font-size: 10px; font-weight: bold; letter-spacing: 1px; padding: 4px 6px;")
        layout.addWidget(lbl_nav)

        self.btn_group = QButtonGroup(self)
        self.btn_group.setExclusive(True)

        items = [
            ("Cockpit PFD", 0),
            ("FPV Camera", 1),
            ("Tactical SLAM", 2),
            ("Motor Actuators", 3),
            ("Diagnostics", 4),
            ("Flight Terminal", 5),
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

        # System version footer
        lbl_ver = QLabel("  Drone-1.5 GCS\n  v2.0 Industrial", self)
        lbl_ver.setStyleSheet("color: #484f58; font-size: 9px; font-style: italic; padding: 8px 12px;")
        layout.addWidget(lbl_ver)

    def _on_button_clicked(self, idx: int):
        self.view_changed.emit(idx)

    def select_tab(self, idx: int):
        btn = self.btn_group.button(idx)
        if btn:
            btn.setChecked(True)
            self.view_changed.emit(idx)
