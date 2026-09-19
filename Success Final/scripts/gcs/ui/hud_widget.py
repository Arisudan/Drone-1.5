"""
================================================================================
MODULE: hud_widget.py
PURPOSE: Minimal Speed / Altitude Primary Flight Display
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Flight Deck)
  * Communicates:  drone_gcs.py main window and TelemetryState instances
  * Upstream:      Decoded LOCAL_POSITION_NED (ground speed, altitude) and HEARTBEAT
                   (armed, flight mode)
  * Downstream:    Pilot PFD visual display (60 FPS smooth vector rendering)

DATA FLOW & INTERFACES:
  * Inputs:        Altitude (m AGL) and Ground Speed (m/s) from LOCAL_POSITION_NED,
                   Armed status and Flight Mode from HEARTBEAT.
  * GUI Engine:    PyQt5 QPainter with antialiased vector geometry.

KEY LOGIC & FAILSAFES:
  * Rolling Tape Ladders: Aviation-standard left speed tape and right altitude tape
    with moving numerical readouts and centered pointer bugs - both driven directly
    from live Pixhawk telemetry, not placeholder values.
  * Deliberately minimal: no artificial horizon, roll/pitch ladder, compass tape, or
    crosshair - the operator only needs speed/altitude/mode/armed at a glance here;
    attitude and heading are better served by the FPV/SLAM views.

USAGE:
  pfd = HUDWidget()
  pfd.update_telemetry(snapshot)
================================================================================
"""

from __future__ import annotations
from typing import Optional

from PyQt5.QtGui import QPainter, QColor
from PyQt5.QtWidgets import QWidget, QSizePolicy

from core.telemetry import TelemetrySnapshot
from ui.video_feed_widget import VideoSink


class HUDWidget(QWidget):
    """Minimal PFD: live speed tape, altitude tape, and a mode/armed banner - nothing else."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(540, 380)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Telemetry State - trimmed to only what's actually drawn now
        self.altitude: float = 0.0     # m AGL
        self.speed: float = 0.0        # m/s
        self.armed: bool = False
        self.flight_mode: str = "DISCONNECTED"

        # The PFD's centre was empty canvas - the tapes live at the edges and
        # nothing occupied the ~80% between them. The camera goes there, so the
        # cockpit shows attitude and what the aircraft is looking at together
        # instead of making the operator change tabs to see one or the other.
        self.video = VideoSink("FPV FEED - OPEN CAMERA TAB OR CONNECT", self)

    def update_telemetry(self, t: TelemetrySnapshot):
        """Update HUD state from TelemetrySnapshot and trigger repaint."""
        self.altitude = t.altitude
        self.speed = t.ground_speed
        self.armed = t.armed
        self.flight_mode = t.flight_mode
        self.update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._place_video()

    def _place_video(self):
        """The camera owns the whole pane now that the tapes have gone.

        No aspect cap: the sink letterboxes with KeepAspectRatio, so handing it
        the full rect fills the space without distorting the image.
        """
        margin = 2
        self.video.setGeometry(margin, margin,
                               max(160, self.width() - 2 * margin),
                               max(90, self.height() - 2 * margin))

    def paintEvent(self, event):
        """Paint full aviation PFD cockpit overlay."""
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)

        # Backdrop only: the camera sink covers this, and the fill is what
        # shows through the letterbox bars when the frame is not 16:9.
        p.fillRect(0, 0, self.width(), self.height(), QColor(9, 13, 18))

        # The speed and altitude tapes moved to the header, where they are
        # visible from every workspace instead of only this one. What is left
        # of this widget is the camera pane and its backdrop; the mode/arm
        # banner went to the navigation rail's footer for the same reason.
        p.end()

