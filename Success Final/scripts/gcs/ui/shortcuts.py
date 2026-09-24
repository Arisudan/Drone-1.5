"""
================================================================================
MODULE: shortcuts.py
PURPOSE: Application keyboard shortcuts + the overlay that documents them
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS UI Layer)
  * Upstream:      Operator keystrokes
  * Downstream:    drone_gcs.py action callbacks, by name from BINDINGS

WHY THIS EXISTS:
  The station had no keyboard shortcuts at all - not one QShortcut in the whole
  codebase. Every action, including switching workspaces to see the map while
  something is happening, required finding a control with the mouse. During an
  actual flight the operator has one hand on a transmitter.

NO DESTRUCTIVE ACTION FIRES DIRECTLY FROM A KEY:
  This is the rule the whole table is built around. Arm, disarm and abort are
  bound, but each one routes into the same GuidedConfirmBar a mouse click goes
  through, so a key press *opens* the confirmation and a deliberate drag
  completes it. A keystroke is exactly the kind of input that happens by
  accident - a focused window, a cat, a palm - and the confirmation is what
  makes that survivable.

  EMERGENCY KILL is deliberately not bound at all. It is the one action with no
  recovery path, it is needed roughly never, and when it is needed the operator
  is looking at the screen anyway. A key combination that cuts the motors of a
  flying aircraft is a liability with no compensating benefit.

SINGLE-KEY BINDINGS ARE AVOIDED, WITH TWO EXCEPTIONS:
  An application-context shortcut is matched before the focused widget sees
  the key, so a bare letter or Space silently takes that key away from the
  entire UI. Space is how Qt activates a focused button, and binding it broke
  that app-wide. The exceptions are F1, which nothing else wants and which
  nobody finds behind a modifier, and Escape, which is safe because Qt gives
  an open dialog priority over application shortcuts. Both were measured, not
  assumed.

ESCAPE IS A UNIVERSAL "STOP THAT":
  One key cancels the confirmation bar, clears an in-progress measurement and
  halts a running motor test. Under stress people press Escape; it should
  always do the least surprising safe thing rather than nothing.

DISCOVERABILITY IS PART OF THE FEATURE:
  A shortcut nobody knows about is not a feature, so F1 opens an overlay listing
  every binding, generated from this same table - the documentation cannot drift
  from the bindings because it is the bindings.

USAGE:
  from ui.shortcuts import install_shortcuts, ShortcutHelpOverlay
  install_shortcuts(window, {"workspace_1": cb, "arm": cb, ...})
================================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QWidget, QShortcut, QDialog, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout,
    QFrame, QPushButton, QScrollArea,
)

from ui.styles import PALETTE
from ui.scaling import px


@dataclass(frozen=True)
class Binding:
    """One shortcut. `action` is the key drone_gcs.py registers a callback under.

    `guarded` marks a binding that opens a confirmation rather than acting -
    rendered differently in the help overlay, so the operator can see at a
    glance which keys do something immediately and which ask first.
    """
    keys: str
    action: str
    description: str
    category: str
    guarded: bool = False


BINDINGS: List[Binding] = [
    # ── Workspaces ──────────────────────────────────────────────────
    Binding("Ctrl+1", "workspace_0", "Control workspace", "Workspaces"),
    Binding("Ctrl+2", "workspace_1", "FPV camera", "Workspaces"),
    Binding("Ctrl+3", "workspace_2", "Tactical SLAM", "Workspaces"),
    Binding("Ctrl+4", "workspace_3", "Motor actuators", "Workspaces"),
    Binding("Ctrl+5", "workspace_4", "Diagnostics", "Workspaces"),
    Binding("Ctrl+6", "workspace_5", "Flight terminal", "Workspaces"),
    Binding("Ctrl+7", "workspace_6", "Flight logs", "Workspaces"),
    Binding("Ctrl+8", "workspace_7", "Configuration", "Workspaces"),

    # ── Link ────────────────────────────────────────────────────────
    Binding("Ctrl+K", "toggle_link", "Connect / disconnect", "Link"),

    # ── Flight ──────────────────────────────────────────────────────
    Binding("Ctrl+Shift+A", "arm", "Arm — opens confirmation", "Flight", True),
    Binding("Ctrl+D", "disarm", "Disarm — opens confirmation", "Flight", True),
    Binding("Ctrl+T", "takeoff", "Take off — opens confirmation", "Flight", True),
    Binding("Ctrl+L", "land", "Land (AUTO.LAND)", "Flight"),
    Binding("Ctrl+H", "hold", "Hold position (AUTO.LOITER)", "Flight"),

    # ── Path ────────────────────────────────────────────────────────
    Binding("Ctrl+E", "execute_path", "Execute the staged path", "Path"),
    Binding("Ctrl+P", "pause_path", "Pause / resume the path", "Path"),
    # Not Space. A bare Space registered as an application shortcut overrides
    # Qt's own "activate the focused button" behaviour everywhere in the app -
    # measured, not assumed - so tabbing to ARM and pressing Space would have
    # silently run the abort action instead of arming. It also hands the most
    # easily-brushed key on the keyboard to the one Path action that stops a
    # flight. Every other binding here takes a modifier; so does this one.
    Binding("Ctrl+Shift+X", "abort_path", "Abort path — opens confirmation",
            "Path", True),

    # ── Map ─────────────────────────────────────────────────────────
    Binding("Ctrl+F", "fit_map", "Fit the map to its extent", "Map"),
    Binding("Ctrl+=", "zoom_in", "Zoom in", "Map"),
    Binding("Ctrl+-", "zoom_out", "Zoom out", "Map"),
    Binding("Ctrl+R", "toggle_ruler", "Toggle the measure tool", "Map"),

    # ── Station ─────────────────────────────────────────────────────
    Binding("Ctrl+M", "toggle_mute", "Mute / unmute audio alerts", "Station"),
    Binding("Esc", "cancel", "Cancel confirmation, measurement or motor test",
            "Station"),
    Binding("F1", "show_help", "Show this shortcut list", "Station"),
]

# Ctrl+= is what an unshifted "+" key actually produces on most layouts, but
# some operators will press Ctrl+Shift+= (i.e. Ctrl++). Both are registered for
# the same action; only the first is listed in the overlay to keep it readable.
ALIASES: Dict[str, List[str]] = {
    "zoom_in": ["Ctrl++"],
}


def install_shortcuts(window: QWidget,
                      callbacks: Dict[str, Callable[[], None]]) -> List[QShortcut]:
    """Bind every entry in BINDINGS that has a callback registered.

    A missing callback is skipped silently rather than raising: the table is the
    full catalogue of intended shortcuts, and a build where one action is not
    wired yet should still start with the other twenty working.

    Returns the created QShortcut objects so the caller can keep them alive -
    a QShortcut that gets garbage collected stops working, and does so silently.
    """
    created: List[QShortcut] = []
    for binding in BINDINGS:
        callback = callbacks.get(binding.action)
        if callback is None:
            continue
        for keys in [binding.keys] + ALIASES.get(binding.action, []):
            sc = QShortcut(QKeySequence(keys), window)
            # ApplicationShortcut so a shortcut still works when focus is inside
            # the embedded terminal or a text field's sibling widget. The CLI
            # console handles its own keys and is unaffected by these
            # combinations.
            sc.setContext(Qt.ApplicationShortcut)
            sc.activated.connect(callback)
            created.append(sc)
    return created


class ShortcutHelpOverlay(QDialog):
    """The F1 list, generated from BINDINGS so it cannot fall out of date."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle("Keyboard shortcuts")
        self.setMinimumSize(px(560), px(520))

        root = QVBoxLayout(self)
        root.setContentsMargins(px(16), px(14), px(16), px(14))
        root.setSpacing(px(10))

        title = QLabel("KEYBOARD SHORTCUTS", self)
        title.setObjectName("cardHeading")
        root.addWidget(title)

        note = QLabel(
            "Keys marked ◆ open a confirmation rather than acting immediately. "
            "Emergency kill has no shortcut by design.", self)
        note.setObjectName("fieldSubLabel")
        note.setWordWrap(True)
        root.addWidget(note)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        holder = QWidget()
        grid = QGridLayout(holder)
        grid.setHorizontalSpacing(px(14))
        grid.setVerticalSpacing(px(4))
        grid.setColumnStretch(1, 1)

        row = 0
        last_category = None
        for binding in BINDINGS:
            if binding.category != last_category:
                if row:
                    row += 1
                header = QLabel(binding.category.upper(), holder)
                header.setStyleSheet(
                    f"color: {PALETTE['accent']}; font-weight: bold;"
                    " letter-spacing: 1px;")
                grid.addWidget(header, row, 0, 1, 2)
                row += 1
                last_category = binding.category

            key = QLabel(binding.keys, holder)
            key.setStyleSheet(
                f"background: {PALETTE['bg_input']}; color: {PALETTE['text_bright']};"
                f" border: 1px solid {PALETTE['border_soft']};"
                f" border-radius: {px(3)}px; padding: {px(1)}px {px(6)}px;"
                f" font-family: {PALETTE['font_mono']}; font-weight: bold;")
            key.setAlignment(Qt.AlignCenter)
            grid.addWidget(key, row, 0)

            text = binding.description
            desc = QLabel(("◆ " if binding.guarded else "") + text, holder)
            desc.setStyleSheet(
                f"color: {PALETTE['warn'] if binding.guarded else PALETTE['text']};")
            grid.addWidget(desc, row, 1)
            row += 1

        grid.setRowStretch(row, 1)
        scroll.setWidget(holder)
        root.addWidget(scroll, 1)

        foot = QHBoxLayout()
        foot.addStretch()
        close = QPushButton("Close", self)
        close.clicked.connect(self.accept)
        foot.addWidget(close)
        root.addLayout(foot)
