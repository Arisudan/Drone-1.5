"""
================================================================================
MODULE: toast.py
PURPOSE: Non-Blocking Floating Notification Banner & Pilot Alert Overlay
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS UI Layer)
  * Communicates:  drone_gcs.py main window & top-level viewports
  * Upstream:      Flight command dispatches, MAVLink ACKs, and safety warnings
  * Downstream:    Operator heads-up floating transient alert display

DATA FLOW & INTERFACES:
  * Inputs:        Notification text string, hex border/font color, duration in ms.
  * Behavior:      Anchors into the top status strip's own freed right-hand space
                   (top_status_strip.py's row 2 is left-aligned specifically to leave
                   this area clear) rather than floating over the workspace content
                   below - auto-hides upon QTimer expiration.

KEY LOGIC & FAILSAFES:
  * Non-Modal & Click-Through: Sets `Qt.WA_TransparentForMouseEvents` so active
    piloting, clicking, and map dragging are never blocked by an alert banner.
  * Color Coded Severity:
      - Green (#3fb950): Successful mode change, arming confirmation, waypoint reached.
      - Amber (#d29922): Preflight check warning, GPS glitch, battery warning.
      - Red (#f85149): Command rejection, emergency kill, tracking stall.
      - Blue (#58a6ff): Informational state changes.

USAGE:
  toast = NotificationToast(parent=self)
  toast.show_message("COMMAND ACCEPTED: OFFBOARD", color="#3fb950", duration_ms=3000)
================================================================================
"""

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QApplication


class NotificationToast(QWidget):
    """Floating notification toast that auto-hides after duration."""

    def __init__(self, parent=None):
        super().__init__(parent)
        # No fixed height/min-width: sized entirely by adjustSize() off the label's own
        # padding/font below, scaled to match the header's own badges (row 2) it now
        # sits beside, rather than a large standalone pill that overflowed the header.
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.setStyleSheet("background: transparent;")
        self.hide()

        self._label = QLabel(self)
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setStyleSheet(
            "background-color: rgba(22, 27, 34, 0.95); "
            "color: #c9d1d9; "
            "border-radius: 4px; "
            "padding: 4px 10px; "
            "font-size: 11px; "
            "font-weight: bold; "
            "border: 1px solid #30363d;"
        )

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._label)

        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self.hide)

    def show_message(self, text: str, color: str = "#58a6ff", duration_ms: int = 3200):
        """Display toast message with custom color and auto-hide timer."""
        self._label.setText(text)
        self._label.setStyleSheet(
            f"background-color: rgba(13, 17, 23, 0.96); "
            f"color: {color}; "
            f"border-radius: 4px; "
            f"padding: 4px 10px; "
            f"font-size: 11px; "
            f"font-weight: bold; "
            f"border: 1px solid {color};"
        )
        self.adjustSize()
        top_strip = getattr(self.parent(), "top_strip", None)
        # Anchor to the rightmost real content of the header's second row. This
        # used to be the VIO NED readout; that moved to the Diagnostics tab, so
        # the altitude instrument is now the last thing on the row. Falling
        # back through the list rather than naming one widget means the next
        # header reshuffle degrades to the centred fallback instead of dropping
        # the toast onto the navigation rail.
        # Anchor to the end of the header's second line. Row 1 is the controls
        # you operate and row 2 is what the vehicle reports back, so a
        # notification about what just happened belongs on row 2 - and that row
        # has a wide gap between the status badges and the arm/mode pair.
        # Falling through a list rather than naming one widget means a header
        # reshuffle degrades to the centred fallback instead of dropping the
        # toast onto the navigation rail.
        anchor = None
        if top_strip is not None:
            for name in ("badge_rc", "badge_vision_conf", "badge_vio"):
                anchor = getattr(top_strip, name, None)
                if anchor is not None:
                    break
        if anchor is not None:
            w, h = self.width(), self.height()
            margin = 14  # matches top_status_strip.py's SPACING constant
            anchor_top_left = anchor.mapTo(self.parent(), anchor.rect().topLeft())
            # sizeHint(), not the possibly one-frame-stale allocated width - the
            # readout's text length changes with live telemetry, and the current
            # geometry may still be last frame's (shorter) measurement.
            anchor_w = max(anchor.width(), anchor.sizeHint().width())
            x = anchor_top_left.x() + anchor_w + margin
            y = anchor_top_left.y() + (anchor.height() - h) // 2

            # Clamp to the header's own right/bottom edge so it can never spill onto
            # the workspace content below or past the window's right edge.
            max_x = top_strip.x() + top_strip.width() - w - 8
            max_y = top_strip.y() + top_strip.height() - h - 4
            x = max(0, min(x, max_x))
            y = max(0, min(y, max_y))
            self.move(x, y)
        elif self.parent():
            # Fallback: center at top of the window
            pw = self.parent().width()
            w = self.width()
            self.move((pw - w) // 2, 65)

        self.show()
        if QApplication.platformName() != "wayland":
            self.raise_()
        self._timer.start(duration_ms)
