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

from PyQt5.QtCore import Qt, QEvent, QRect, QTimer
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

        self._full_text = ""
        self._watched = []
        self._placing = False

    def show_message(self, text: str, color: str = "#58a6ff", duration_ms: int = 3200):
        """Display a toast, sized and placed so it cannot cover anything.

        The toast is confined to the header's reserved notification band (see
        TopStatusStrip.notification_rect) and its text is elided to fit. It
        previously anchored itself off one named badge's right edge and grew to
        whatever width the message needed, which meant two separate failures:
        adding the AUDIO control to the row after that badge put every
        notification on top of the word AUDIO, and a long message ran past the
        band and was clamped back left across the status badges.

        Eliding loses nothing: every caller in drone_gcs.py writes the same text
        to the console and the flight terminal before raising a toast, so the
        full wording is always recoverable a glance away.
        """
        self._full_text = text
        self._label.setStyleSheet(
            f"background-color: rgba(13, 17, 23, 0.96); "
            f"color: {color}; "
            f"border-radius: 4px; "
            f"padding: 4px 10px; "
            f"font-size: 11px; "
            f"font-weight: bold; "
            f"border: 1px solid {color};"
        )

        self._place()
        self.show()
        if QApplication.platformName() != "wayland":
            self.raise_()
        self._timer.start(duration_ms)

    def _place(self):
        """Fit the current text into the band and position the toast there."""
        self._placing = True
        try:
            self._place_now()
        finally:
            self._placing = False

    def _place_now(self):
        text = self._full_text
        band = self._band()
        self._label.setText(text)
        if band is not None and band.width() > 0:
            metrics = self._label.fontMetrics()
            # Measure the label's own padding and border rather than hardcoding
            # them, so restyling the toast cannot silently break the fit.
            chrome = max(0, self._label.sizeHint().width()
                         - metrics.horizontalAdvance(text))
            room = max(0, band.width() - chrome)
            self._label.setText(
                metrics.elidedText(text, Qt.ElideRight, room))
            self.adjustSize()
            width = min(self.width(), band.width())
            height = min(self.height(), max(band.height(), self.height()))
            self.resize(width, height)
            x = band.x() + (band.width() - width) // 2
            y = band.y() + (band.height() - height) // 2
            self.move(max(0, x), max(0, y))
        else:
            # No measurable band - centre under the header rather than guessing
            # at an anchor that might sit on top of a readout.
            self.adjustSize()
            parent = self.parent()
            if parent is not None:
                self.move(max(0, (parent.width() - self.width()) // 2), 65)

    # The band is measured from the header's live geometry, so a toast placed
    # just before the header reflows (window resize, a badge's text changing
    # width, the connection controls moving between rows) would be left over
    # whatever slid into its old spot - the AUDIO control, in practice. Watch
    # the widgets that bound the band and re-place whenever one moves. Their
    # Move/Resize events are sent synchronously as the layout positions them,
    # so the toast follows in the same pass rather than a frame late.
    _RELAYOUT_EVENTS = (QEvent.Resize, QEvent.Move)

    def _watch(self, top_strip):
        targets = [top_strip] + list(getattr(top_strip, "band_widgets", lambda: [])())
        if [id(w) for w in targets] == [id(w) for w in self._watched]:
            return
        for w in self._watched:
            try:
                w.removeEventFilter(self)
            except RuntimeError:        # already deleted with its window
                pass
        for w in targets:
            w.installEventFilter(self)
        self._watched = targets

    def eventFilter(self, obj, event):
        if (event.type() in self._RELAYOUT_EVENTS and self.isVisible()
                and not self._placing and obj in self._watched):
            self._place()
        return False

    def _band(self):
        """The header's reserved notification band, in this widget's parent's
        coordinates, or None when the header cannot supply one."""
        parent = self.parent()
        top_strip = getattr(parent, "top_strip", None)
        if top_strip is None or not hasattr(top_strip, "notification_rect"):
            return None
        self._watch(top_strip)
        rect = top_strip.notification_rect()
        if rect.isEmpty() or rect.width() <= 0:
            return None
        top_left = top_strip.mapTo(parent, rect.topLeft())
        return QRect(top_left.x(), top_left.y(), rect.width(), rect.height())

    def current_rect(self):
        """Geometry the toast currently occupies, for layout tests."""
        return QRect(self.x(), self.y(), self.width(), self.height())
