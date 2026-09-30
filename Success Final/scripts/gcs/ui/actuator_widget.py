"""
================================================================================
MODULE: actuator_widget.py
PURPOSE: Cockpit control for the ESP32-C3 WiFi servo actuator (MG995)
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

from PyQt5.QtCore import QTimer, QUrl, pyqtSignal
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QVBoxLayout

from ui.scaling import px

POLL_MS = 1000
TIMEOUT_MS = 1500


class ActuatorPanel(QFrame):
    """One-button servo toggle with a live status line."""

    # (message, is_error) - lets the host window echo actions to its console.
    log_message = pyqtSignal(str, bool)

    def __init__(self, host: str, port: int = 80, parent=None):
        super().__init__(parent)
        self.setProperty("class", "cardFrame")
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

        lay = QVBoxLayout(self)
        lay.setContentsMargins(px(10), px(8), px(10), px(10))
        lay.setSpacing(px(6))

        head = QHBoxLayout()
        head.setSpacing(px(8))
        title = QLabel("ACTUATOR", self)
        title.setObjectName("sectionTitle")
        head.addWidget(title)
        self.lbl_status = QLabel("", self)
        self.lbl_status.setObjectName("valueMono")
        head.addWidget(self.lbl_status, 1)
        lay.addLayout(head)

        self.btn_toggle = QPushButton("SERVO → 0°", self)
        self.btn_toggle.setObjectName("btnNav")
        self.btn_toggle.setMinimumHeight(px(30))
        self.btn_toggle.clicked.connect(self._on_toggle)
        lay.addWidget(self.btn_toggle)

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
        self.btn_toggle.setText(f"SERVO → {self._next_angle()}°")
        if not self._online:
            self.lbl_status.setText(f"OFFLINE  {self._base}")
            self.lbl_status.setStyleSheet("color: #f85149;")
            return
        pos = "--" if self._angle < 0 else f"{self._angle}°"
        state = "moving" if self._busy else "idle"
        self.lbl_status.setText(f"ONLINE  {pos}  {state}")
        self.lbl_status.setStyleSheet("color: #3fb950;")
