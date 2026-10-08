"""
================================================================================
MODULE: actuator_widget.py
PURPOSE: Compact cockpit servo button for the ESP32-C3 WiFi actuator (MG995)
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (Cockpit control panel)
  * Communicates:  ESP32-C3 SuperMini servo firmware over HTTP (port 80)
  * Upstream:      Operator button press
  * Downstream:    GET /setAngle?value=N (queued on the ESP32), GET /status

PROTOCOL (as served by the firmware):
  /status            -> {"angle": int (-1 = unknown), "moving": bool, "queued": int}
  /setAngle?value=N  -> 200 QUEUED | 503 QUEUE FULL
  /clear             -> 200 CLEARED

BEHAVIOUR:
  Deliberately tiny: a status dot and ONE button, no card, no title. It shares a
  row with EMERGENCY KILL (see drone_gcs.py), so everything that used to be a
  second line of text - the IP, the error - lives in the tooltip instead. The
  button text carries the live state ("SERVO 0° → 90°"); the dot carries
  health (grey idle, amber moving, red offline - colour only for real status).

  The button mirrors the ESP32's own BOOT button: it toggles 0 <-> 90 deg, and
  the first press from an unknown position goes to 0. Commands go through the
  firmware's queue, so pressing while the servo is still moving is safe.

NEVER BLOCKS THE UI: requests are asynchronous (QNetworkAccessManager) with a
short transfer timeout, so an ESP32 that is off or out of range shows as
OFFLINE instead of freezing the station.
================================================================================
"""

from __future__ import annotations

import json
from typing import Optional

from PyQt5.QtCore import Qt, QTimer, QUrl, pyqtSignal
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QWidget

from ui.status_dot import StatusDot
from ui.scaling import fit_min_width, px

POLL_MS = 1000
TIMEOUT_MS = 1500


class ActuatorPanel(QWidget):
    """One-button servo toggle with a status dot."""

    # (message, is_error) - lets the host window echo actions to its console.
    log_message = pyqtSignal(str, bool)

    def __init__(self, host: str, port: int = 80, parent=None):
        super().__init__(parent)
        self._base = ""
        self._online = False
        self._angle = -1                    # last angle reported by /status
        self._busy = False                  # moving or queue non-empty
        self._last_sent: Optional[int] = None
        self._status_inflight = False

        # One manager-level handler, dispatching on a property tagged onto each
        # reply. Per-reply Python lambdas crashed (segfault) inside the full
        # station, where the reply's own deleteLater raced its closure.
        self._net = QNetworkAccessManager(self)
        self._net.finished.connect(self._on_reply)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(px(8))

        self.dot = StatusDot(10, parent=self)
        lay.addWidget(self.dot, 0, Qt.AlignVCenter)

        self.btn_toggle = QPushButton("SERVO", self)
        self.btn_toggle.setMinimumHeight(px(32))
        # Widest state text, so the button never resizes (and shoves its
        # neighbour) when the label changes.
        fit_min_width(self.btn_toggle,
                      ["SERVO 90° → 90°", "SERVO -- → 90°", "SERVO OFFLINE"], h_pad_px=28)
        self.btn_toggle.clicked.connect(self._on_toggle)
        lay.addWidget(self.btn_toggle, 1)

        self.set_endpoint(host, port)

        self._poll = QTimer(self)
        self._poll.setInterval(POLL_MS)
        self._poll.timeout.connect(self._request_status)
        self._poll.start()
        self._request_status()

    # ── public ──────────────────────────────────────────────────────

    def set_endpoint(self, host: str, port: int = 80) -> None:
        self._base = f"http://{host}:{port}" if port != 80 else f"http://{host}"
        self._online = False
        self._render()

    # ── toggle ──────────────────────────────────────────────────────

    def _next_angle(self) -> int:
        # While commands are still in flight, toggle relative to what we last
        # asked for; once idle, relative to where the servo actually is (it may
        # have been moved by the ESP32's own button or its web page).
        ref = self._last_sent if (self._busy and self._last_sent is not None) else self._angle
        return 90 if ref == 0 else 0

    def _on_toggle(self) -> None:
        angle = self._next_angle()
        reply = self._get(f"/setAngle?value={angle}")
        reply.setProperty("kind", "set")
        reply.setProperty("angle", angle)

    def _on_reply(self, reply: QNetworkReply) -> None:
        kind = reply.property("kind")
        if kind == "set":
            self._on_set_reply(reply, int(reply.property("angle")))
        elif kind == "status":
            self._on_status_reply(reply)
        reply.deleteLater()

    def _on_set_reply(self, reply: QNetworkReply, angle: int) -> None:
        code = reply.attribute(QNetworkRequest.HttpStatusCodeAttribute)
        err = reply.error()
        if err == QNetworkReply.NoError and code == 200:
            self._last_sent = angle
            self._busy = True
            self.log_message.emit(f"Actuator: servo -> {angle} deg queued", False)
        elif code == 503:
            self.log_message.emit("Actuator: ESP32 queue full - command dropped", True)
        else:
            self._online = False
            self.log_message.emit(
                f"Actuator: ESP32 unreachable at {self._base} ({reply.errorString()})", True)
        self._render()
        self._request_status()

    # ── status poll ─────────────────────────────────────────────────

    def _request_status(self) -> None:
        if self._status_inflight:
            return
        self._status_inflight = True
        self._get("/status").setProperty("kind", "status")

    def _on_status_reply(self, reply: QNetworkReply) -> None:
        self._status_inflight = False
        ok = reply.error() == QNetworkReply.NoError
        body = bytes(reply.readAll()) if ok else b""
        try:
            d = json.loads(body) if ok else None
        except ValueError:
            d = None
        if d is None:
            self._online = False
        else:
            self._online = True
            self._angle = int(d.get("angle", -1))
            self._busy = bool(d.get("moving")) or int(d.get("queued", 0)) > 0
        self._render()

    # ── helpers ─────────────────────────────────────────────────────

    def _get(self, path: str) -> QNetworkReply:
        req = QNetworkRequest(QUrl(self._base + path))
        req.setTransferTimeout(TIMEOUT_MS)
        return self._net.get(req)

    def _render(self) -> None:
        if not self._online:
            self.btn_toggle.setText("SERVO OFFLINE")
            self.btn_toggle.setToolTip(
                f"ESP32 not reachable at {self._base}.\nClick to retry. "
                "Set the address in Configuration -> ESP32 SERVO ACTUATOR.")
            self._set_dot("#f85149")
            return
        pos = "--" if self._angle < 0 else f"{self._angle}°"
        self.btn_toggle.setText(f"SERVO {pos} → {self._next_angle()}°")
        state = "moving" if self._busy else "idle"
        self.btn_toggle.setToolTip(
            f"ESP32 online at {self._base} - {state}.\n"
            f"Click to move the servo to {self._next_angle()}°.")
        self._set_dot("#d29922" if self._busy else "#6e7681")

    def _set_dot(self, colour: str) -> None:
        self.dot.set_colour(colour)
