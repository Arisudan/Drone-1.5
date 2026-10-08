"""
================================================================================
MODULE: header_flight_bar.py
PURPOSE: MODE, SET, ARM and DISARM in one horizontal row at the right of the Tactical SLAM header
================================================================================

WHY THIS EXISTS:
  Steering from the SLAM map meant leaving it for the Control tab to arm, disarm or change mode and coming back.
  These are the same controls, on the free right-hand end of the map's header.

IT ADDS NO FLIGHT LOGIC. The bar only raises signals; the main window routes ARM and DISARM through the same
slide-to-confirm handlers as the Control tab's buttons and MODE through _cmd_mode, so every interlock that already
applies still applies. Nothing here talks to the vehicle.

NEVER GREYED OUT. The Control tab's ARM / DISARM / SET MODE are always clickable and refuse with a message when
there is no link; these do the same, so a lagging telemetry flag can never leave them dead. Colours match the
design system: SET MODE blue (btnNav), ARM green (btnArm), DISARM red (btnDisarm).

STATE CHIP: a small neutral label left of the mode box reads "DISARMED", "ARMED" or "NO LINK" (the mode box beside it already shows the mode). It is the only
thing here that changes with state, and it takes colour only when something matters: amber while ARMED, red text
with NO LINK. Information only - it never gates a button.

STATE: set_state(...) keeps the mode box on the vehicle's actual mode unless the operator has the list open.
================================================================================
"""

from __future__ import annotations

from typing import Optional, Sequence

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QComboBox, QHBoxLayout, QLabel, QPushButton, QWidget

from ui.scaling import fit_min_width, px


class HeaderFlightBar(QWidget):
    """MODE [combo] [SET MODE] [ARM] [DISARM], one line. Buttons are public so the confirm bar can anchor to them."""

    arm_requested = pyqtSignal()
    disarm_requested = pyqtSignal()
    mode_requested = pyqtSignal(str)

    HEIGHT_PX = 32
    FONT_PX = 12   # one size for all four controls (ARM used to be larger than the rest)

    def __init__(self, modes: Sequence[str], parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("transparentRow")

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(px(4))

        self.lbl_state = QLabel("NO LINK", self)
        self.lbl_state.setObjectName("flightStateChip")
        self.lbl_state.setProperty("state", "offline")
        self.lbl_state.setToolTip("Link and arm state")
        row.addWidget(self.lbl_state)
        row.addSpacing(px(6))

        self.combo_mode = QComboBox(self)
        self.combo_mode.addItems(list(modes))
        self.combo_mode.setCurrentText("STABILIZED")
        self.combo_mode.setToolTip("Flight mode to switch to")
        fit_min_width(self.combo_mode, list(modes), h_pad_px=36)
        row.addWidget(self.combo_mode)

        self.btn_set_mode = QPushButton("SET MODE", self)
        self.btn_set_mode.setObjectName("btnNav")
        self.btn_set_mode.setToolTip("Switch the vehicle to the selected mode")
        self.btn_set_mode.clicked.connect(lambda: self.mode_requested.emit(self.combo_mode.currentText().strip()))
        row.addWidget(self.btn_set_mode)

        self.btn_arm = QPushButton("ARM", self)
        self.btn_arm.setObjectName("btnArm")
        self.btn_arm.clicked.connect(self.arm_requested)
        row.addWidget(self.btn_arm)

        self.btn_disarm = QPushButton("DISARM", self)
        self.btn_disarm.setObjectName("btnDisarm")
        self.btn_disarm.clicked.connect(self.disarm_requested)
        row.addWidget(self.btn_disarm)

        # One height and one font size for the whole row.
        h = px(self.HEIGHT_PX)
        self.setStyleSheet(f"QPushButton, QComboBox, QLabel#flightStateChip "
                           f"{{ font-size: {self.FONT_PX}px; min-height: 0px; }}")
        for w in (self.lbl_state, self.combo_mode, self.btn_set_mode, self.btn_arm, self.btn_disarm):
            w.setFixedHeight(px(self.HEIGHT_PX))
        fit_min_width(self.lbl_state, ["DISARMED", "ARMED", "NO LINK"], h_pad_px=14)
        for b, texts in ((self.btn_set_mode, ["SET MODE"]), (self.btn_arm, ["ARM"]), (self.btn_disarm, ["DISARM"])):
            fit_min_width(b, texts)
        # ARM and DISARM are a pair: same width, so the row reads as one even strip.
        # sizeHint included: the measured floor alone came out 3 px short of the real text at this font size.
        for b in (self.btn_set_mode, self.btn_arm, self.btn_disarm):
            b.setMinimumWidth(max(b.minimumWidth(), b.sizeHint().width()))
        pair = max(self.btn_arm.minimumWidth(), self.btn_disarm.minimumWidth(),
                   self.btn_arm.sizeHint().width(), self.btn_disarm.sizeHint().width())
        self.btn_arm.setFixedWidth(pair)
        self.btn_disarm.setFixedWidth(pair)

    def set_state(self, connected: bool, armed: bool, mode: str = "") -> None:
        """Follow the vehicle's real mode, but never fight an operator who is choosing one.
        `connected` / `armed` only drive the state chip; they never disable a button."""
        if not connected:
            text, state = "NO LINK", "offline"
        else:
            text, state = ("ARMED" if armed else "DISARMED"), "armed" if armed else "idle"
        if self.lbl_state.property("state") != state:
            self.lbl_state.setProperty("state", state)
            self.lbl_state.style().unpolish(self.lbl_state)
            self.lbl_state.style().polish(self.lbl_state)
        self.lbl_state.setText(text)
        if mode and not self.combo_mode.view().isVisible():
            i = self.combo_mode.findText(mode)
            if i >= 0 and i != self.combo_mode.currentIndex():
                self.combo_mode.blockSignals(True)
                self.combo_mode.setCurrentIndex(i)
                self.combo_mode.blockSignals(False)
