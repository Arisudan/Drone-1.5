"""
================================================================================
MODULE: top_status_strip.py
PURPOSE: Aerospace Header Status Strip, Network Connect Bar, & System Telemetry Badges
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Header Banner)
  * Communicates:  drone_gcs.py main window and MAVLinkWorker
  * Upstream:      TelemetrySnapshot updates and user IP/port input fields
  * Downstream:    Operator quick-look status indicators and network connect signals

DATA FLOW & INTERFACES:
  * Telemetry In:  Flight mode, armed status, battery voltage & percentage,
                   D435i VIO tracking state, MAVLink packet rates (RX/TX Hz).
  * Outbound Qt:   connect_requested(host: str, port: int), disconnect_requested().

KEY LOGIC & FAILSAFES:
  * Runtime Dynamic Network Reconfiguration: Allows pilot to change target IP/port
    and reconnect without restarting the GCS application.
  * Multi-State Status Badges:
      - Flight Mode: Distinct tactical color coding for OFFBOARD, POSCTL, ALTCTL, MANUAL.
      - Armed / Disarmed: green when armed, red when disarmed (same convention in
        the navigation footer and the Diagnostics tab).
      - VIO Tracking Lock: Visual green/red badge for RealSense D435i odometry health.
      - Battery Capacity: Transitions green (>50%) -> amber (20-50%) -> flashing red (<20%).
  * Throughput Monitor: Displays live packet exchange frequencies to diagnose Wi-Fi link jitter.

USAGE:
  status_strip = TopStatusStrip()
  status_strip.connect_requested.connect(self.handle_connect)
  status_strip.update_telemetry(snapshot)
================================================================================
"""

from __future__ import annotations
from typing import Optional

from PyQt5.QtCore import Qt, QRect, QRectF, QTimer, QTime, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QLinearGradient, QPen
from PyQt5.QtWidgets import (
    QWidget, QFrame, QHBoxLayout, QVBoxLayout, QLabel, QLineEdit, QPushButton,
    QComboBox, QSpacerItem, QSizePolicy
)

from core.telemetry import TelemetrySnapshot
from ui.scaling import px, fit_min_width, grow_min_width
from ui.battery_badge import BatteryBadge
from ui.theme import Chip, icon, GLASS_LINE

# The networks this rig is actually deployed on. Picking one here just fills in the
# IP - port and protocol are independent axes (same MAVLink/map/video ports apply on
# any of these networks) and are left for the operator to choose separately.
WFB_NETWORK = "wfb-ng"   # short: the header has no spare width (tests/test_gui_layout.py)
# Jetson Orin access point (Holybro Jetson baseboard). The FC's TELEM2 is routed by
# mavlink-routerd on the Jetson (fcu-router.service), which serves GCSs on TCP 5760 only -
# so, like wfb-ng, picking it also fixes the protocol and port.
JETSON_NETWORK = "Jetson_AP"
JETSON_PROTO, JETSON_PORT = "tcp", "5760"
KNOWN_NETWORKS = [
    ("HTIC_RND", "172.16.101.89"),
    ("DroneBridge5", "192.168.1.2"),
    # DroneNet: a NetworkManager connection-sharing/hotspot link - Radxa is always the
    # shared-connection gateway at 10.42.0.1 (the laptop gets a lease like 10.42.0.200,
    # but that's not needed here since the GCS only ever connects out to the Radxa).
    ("DroneNet", "10.42.0.1"),
    # wfb-ng radio: the ground side of the radio runs on this laptop and hands the air
    # unit's MAVLink to 127.0.0.1:14550 and its video to udp 5600, so the GCS listens
    # locally (protocol udpin) instead of connecting out. Picking it also selects the
    # matching protocol and the wfb-ng video source - see _on_network_selected.
    (WFB_NETWORK, "127.0.0.1"),
    # The Jetson is the hotspot's shared-connection gateway, fixed at 10.42.0.1.
    (JETSON_NETWORK, "10.42.0.1"),
]


class TopStatusStrip(QFrame):
    """v2 top bar: burger + page title | connection pill | status chips | battery | clock.

    Same public surface as before (signals, the connection fields, the VIO / EKF2 /
    RC / AUDIO indicators, ``update_telemetry``); only the presentation changed.
    ``badge_arm`` / ``badge_mode`` are kept as real labels so existing callers and
    tests keep working, but the v2 design shows arm / mode state in the telemetry
    card instead, so they are not placed in this bar.
    """

    SPACING = 14
    HEIGHT = 46

    connect_requested = pyqtSignal(str, int, str)
    disconnect_requested = pyqtSignal()
    network_changed = pyqtSignal(str)
    mute_toggled = pyqtSignal(bool)
    menu_requested = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("topBar")
        self.is_connected = False
        self.setFixedHeight(px(self.HEIGHT))

        layout = QHBoxLayout(self)
        layout.setContentsMargins(px(14), 0, px(14), 0)
        layout.setSpacing(px(10))

        # ── brand: burger + current page title ───────────────────────────
        brand = QWidget(self)
        brand.setObjectName("topBrand")
        bl = QHBoxLayout(brand)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(px(10))
        self.btn_menu = QPushButton("", brand)
        self.btn_menu.setObjectName("burger")
        self.btn_menu.setFixedSize(px(30), px(30))
        self.btn_menu.setCursor(Qt.PointingHandCursor)
        self.btn_menu.setToolTip("Keyboard shortcuts (F1)")
        self.btn_menu.clicked.connect(self.menu_requested)
        bl.addWidget(self.btn_menu)
        self.title_lbl = QLabel("Control", brand)
        self.title_lbl.setObjectName("appTitle")
        bl.addWidget(self.title_lbl)
        self.sub_lbl = QLabel("", brand)
        self.sub_lbl.setObjectName("appSubtitle")
        bl.addWidget(self.sub_lbl)
        bl.addStretch(1)
        brand.setFixedWidth(px(206))
        layout.addWidget(brand)

        # ── connection pill ──────────────────────────────────────────────
        self.conn_panel = QFrame(self)
        self.conn_panel.setObjectName("connPill")
        conn_box = QHBoxLayout(self.conn_panel)
        conn_box.setContentsMargins(px(4), px(4), px(4), px(4))
        conn_box.setSpacing(px(6))

        self.lbl_net = QLabel("NET", self)
        self.lbl_net.setObjectName("connCaption")
        conn_box.addWidget(self.lbl_net)

        self.network_combo = QComboBox(self)
        for name, ip in KNOWN_NETWORKS:
            self.network_combo.addItem(name, ip)
        self.network_combo.addItem("Custom...", None)
        self.network_combo.setToolTip(
            "Pick the Wi-Fi network in use - fills in the Radxa's IP for MAVLink, the map "
            "bridge, and the FPV stream in one go.\n" +
            "\n".join(f"{name}: {ip}" for name, ip in KNOWN_NETWORKS))
        self.network_combo.currentIndexChanged.connect(self._on_network_selected)
        fit_min_width(self.network_combo,
                      [n for n, _ in KNOWN_NETWORKS] + ["Custom..."], h_pad_px=40)
        conn_box.addWidget(self.network_combo)

        self.lbl_ip = QLabel("IP", self)
        self.lbl_ip.setObjectName("connCaption")
        conn_box.addWidget(self.lbl_ip)

        self.ip_input = QLineEdit(KNOWN_NETWORKS[0][1], self)
        self.ip_input.setFixedWidth(px(100))
        self.ip_input.setToolTip("Target SBC IP (e.g. 172.16.101.89 or 127.0.0.1) - auto-filled by the Network dropdown, or type your own")
        self.ip_input.textEdited.connect(self._on_ip_hand_edited)
        conn_box.addWidget(self.ip_input)

        # No caption in the design: the protocol and port read as one field pair.
        self.lbl_proto = QLabel("", self)
        self.lbl_proto.setVisible(False)
        self.proto_combo = QComboBox(self)
        self.proto_combo.addItem("UDP", "udp")
        self.proto_combo.addItem("TCP", "tcp")
        self.proto_combo.addItem("WFB", "udpin")
        self.proto_combo.setToolTip(
            "Protocol: UDP for real-time flight over Wi-Fi (port 14550); TCP for bench fallback (port 5760);\n"
            "WFB for the wfb-ng radio link - the GCS listens on 14550 where wfb-ng delivers MAVLink")
        self.proto_combo.currentIndexChanged.connect(self._on_protocol_changed)
        conn_box.addWidget(self.proto_combo)

        self.lbl_port = QLabel("", self)
        self.lbl_port.setVisible(False)
        self.port_input = QLineEdit("14550", self)
        self.port_input.setFixedWidth(px(56))
        self.port_input.setToolTip("Target Port (UDP flight: 14550, TCP bench: 5760)")
        conn_box.addWidget(self.port_input)

        self.btn_toggle = QPushButton("CONNECT", self)
        self.btn_toggle.setObjectName("btnConnect")
        self.btn_toggle.clicked.connect(self._handle_connect_toggle)
        fit_min_width(self.btn_toggle, ["CONNECT", "DISCONNECT"], h_pad_px=26)
        conn_box.addWidget(self.btn_toggle)
        layout.addWidget(self.conn_panel)

        layout.addStretch(1)

        # ── status chips ─────────────────────────────────────────────────
        self.telemetry_strip_widget = QWidget(self)
        ts = QHBoxLayout(self.telemetry_strip_widget)
        ts.setContentsMargins(0, 0, 0, 0)
        ts.setSpacing(px(6))
        self.badge_vio = Chip("VIO", "", self)
        self.badge_vision_conf = Chip("EKF2", "", self)
        self.badge_gps = self.badge_vision_conf        # backward-compatible alias
        self.badge_rc = Chip("RC", "", self)
        for chip, tip in ((self.badge_vio, "LIO / visual odometry - no data"),
                          (self.badge_vision_conf, "EKF2 vision fusion - no vision"),
                          (self.badge_rc, "RC link - no data")):
            chip.setToolTip(tip)
            ts.addWidget(chip)
        self.btn_mute = QPushButton("AUDIO", self)
        self.btn_mute.setObjectName("audioChip")
        self.btn_mute.setCheckable(True)
        self.btn_mute.setCursor(Qt.PointingHandCursor)
        self.btn_mute.setToolTip("Mute or unmute spoken and tonal alerts (Ctrl+M)")
        self.btn_mute.clicked.connect(self._on_mute_clicked)
        self._style_mute(False, True)
        fit_min_width(self.btn_mute, ["NO AUDIO", "MUTED", "AUDIO"], h_pad_px=26)
        ts.addWidget(self.btn_mute)
        layout.addWidget(self.telemetry_strip_widget)

        # Arm / mode: not drawn here (see class docstring) but kept live.
        self.badge_arm = QLabel("DISARMED", self)
        self.badge_arm.setObjectName("badgeDisarmed")
        self.badge_arm.setVisible(False)
        self.badge_mode = QLabel("MODE: DISCONNECTED", self)
        self.badge_mode.setVisible(False)

        self.battery = BatteryBadge(self)
        layout.addWidget(self.battery)

        self.clock = QLabel("", self)
        self.clock.setObjectName("topClock")
        self.clock.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.clock.setMinimumWidth(px(72))
        layout.addWidget(self.clock)
        self._clock_timer = QTimer(self)
        self._clock_timer.timeout.connect(self._tick_clock)
        self._clock_timer.start(1000)
        self._tick_clock()

    # ── chrome ───────────────────────────────────────────────────────────
    def _tick_clock(self) -> None:
        t = QTime.currentTime()
        h12 = t.hour() % 12 or 12
        self.clock.setText(f"{h12}:{t.minute():02d} {'AM' if t.hour() < 12 else 'PM'}")

    def set_page_title(self, title: str, sub: str = "") -> None:
        self.title_lbl.setText(title)
        self.sub_lbl.setText(sub)

    def paintEvent(self, _ev):
        p = QPainter(self)
        g = QLinearGradient(0, 0, 0, self.height())
        g.setColorAt(0, QColor(14, 16, 15, 235))
        g.setColorAt(1, QColor(14, 16, 15, 199))
        p.fillRect(self.rect(), g)
        p.setPen(QPen(GLASS_LINE, 1))
        p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)
        # burger glyph
        r = self.btn_menu.geometry()
        cx, cy = r.center().x(), r.center().y()
        p.setPen(QPen(QColor("#e9ece9"), max(1.6, px(1.6)), Qt.SolidLine, Qt.RoundCap))
        for dy in (-px(5), 0, px(5)):
            p.drawLine(cx - px(7), cy + dy, cx + px(7), cy + dy)
        p.end()

    # ── notification band (toast) ────────────────────────────────────────
    _BAND_LEFT = ("conn_panel",)
    _BAND_RIGHT = ("telemetry_strip_widget",)

    def band_widgets(self) -> list:
        return [w for w in (getattr(self, n, None) for n in self._BAND_LEFT + self._BAND_RIGHT)
                if w is not None]

    def _rect_in_self(self, w) -> QRect:
        return QRect(w.mapTo(self, w.rect().topLeft()), w.size())

    def notification_rect(self) -> QRect:
        left = self._rect_in_self(self.conn_panel)
        right = self._rect_in_self(self.telemetry_strip_widget)
        gap = px(10)
        x = left.right() + 1 + gap
        limit = right.left() - gap
        if limit - x <= 0:
            return QRect()
        return QRect(x, left.top(), limit - x, left.height())

    def _style_mute(self, muted: bool, available: bool) -> None:
        """Sounding / deliberately muted / no backend: three different things."""
        if not available:
            text, state = "NO AUDIO", "off"
        elif muted:
            text, state = "MUTED", "warn"
        else:
            text, state = "AUDIO", "ok"
        self.btn_mute.setText(text)
        self.btn_mute.setProperty("state", state)
        self.btn_mute.style().unpolish(self.btn_mute)
        self.btn_mute.style().polish(self.btn_mute)

    def _on_mute_clicked(self) -> None:
        self.mute_toggled.emit(self.btn_mute.isChecked())

    def set_audio_state(self, muted: bool, available: bool) -> None:
        self.btn_mute.setEnabled(available)
        self.btn_mute.blockSignals(True)
        self.btn_mute.setChecked(muted)
        self.btn_mute.blockSignals(False)
        self._style_mute(muted, available)

    def _on_network_selected(self, idx: int):
        ip = self.network_combo.currentData()
        if ip is None:  # "Custom..." - leave whatever is already typed in ip_input alone
            return
        self.ip_input.setText(ip)
        # wfb-ng is the one preset that also fixes the protocol: the radio's ground side
        # delivers MAVLink to this machine, so the GCS must listen rather than connect out.
        # Leaving it puts plain UDP back, since udpin means nothing on a Wi-Fi network.
        proto = self.proto_combo.currentData()
        if self.is_wfb_selected():
            self._set_protocol("udpin")
        elif self.is_jetson_selected():
            self._set_protocol(JETSON_PROTO)
            self.port_input.setText(JETSON_PORT)   # _on_protocol_changed keeps a custom port
        elif proto == "udpin":
            self._set_protocol("udp")
        self.network_changed.emit(ip)

    def is_wfb_selected(self) -> bool:
        return self.network_combo.currentText() == WFB_NETWORK

    def is_jetson_selected(self) -> bool:
        return self.network_combo.currentText() == JETSON_NETWORK

    def _set_protocol(self, proto: str) -> None:
        idx = self.proto_combo.findData(proto)
        if idx >= 0:
            self.proto_combo.setCurrentIndex(idx)   # _on_protocol_changed fixes the port

    def _on_ip_hand_edited(self, _text: str):
        """Typing directly into the IP field means the active preset no longer matches -
        flip the dropdown to Custom rather than silently disagreeing with what's typed."""
        if self.network_combo.currentData() is not None:
            self.network_combo.blockSignals(True)
            self.network_combo.setCurrentIndex(self.network_combo.count() - 1)
            self.network_combo.blockSignals(False)

    def _on_protocol_changed(self, idx: int):
        proto = self.proto_combo.currentData()
        current_port = self.port_input.text().strip()
        if proto in ("udp", "udpin") and current_port in ("5760", ""):
            self.port_input.setText("14550")
        elif proto == "tcp" and current_port in ("14550", ""):
            self.port_input.setText("5760")

    def _handle_connect_toggle(self):
        if self.is_connected:
            self.disconnect_requested.emit()
        else:
            ip = self.ip_input.text().strip()
            port = int(self.port_input.text().strip())
            proto = self.proto_combo.currentData() or "udp"
            self.connect_requested.emit(ip, port, proto)

    def set_connection_state(self, connected: bool, message: str = ""):
        self.is_connected = connected
        if connected:
            self.btn_toggle.setText("DISCONNECT")
            self.btn_toggle.setObjectName("btnDisconnect")
        else:
            self.btn_toggle.setText("CONNECT")
            self.btn_toggle.setObjectName("btnConnect")
            self.badge_mode.setText("MODE: DISCONNECTED")
            self.badge_arm.setText("DISARMED")
            self.badge_arm.setObjectName("badgeDisarmed")
        self.badge_arm.setStyle(self.badge_arm.style())
        self.btn_toggle.style().unpolish(self.btn_toggle)
        self.btn_toggle.style().polish(self.btn_toggle)
        self.btn_toggle.setStyle(self.btn_toggle.style())

    def update_telemetry(self, t: TelemetrySnapshot):
        self.battery.set_state(t.battery_voltage, t.battery_percent, t.connected)

        # LIO pose (Point-LIO -> injector -> FC): LOCKED / LOST / NO DATA.
        if t.d435i_vio_health and t.d435i_vio_age <= 3.0:
            self.badge_vio.set_state("ok")
            self.badge_vio.setToolTip(f"LIO: LOCKED ({t.d435i_vio_age:.1f}s)")
        else:
            self.badge_vio.set_state("bad" if t.last_vision_time > 0 else "")
            self.badge_vio.setToolTip("LIO: LOST" if t.last_vision_time > 0 else "LIO: NO DATA")

        # EKF2 external-vision fusion (GPS-denied indicator).
        if t.ekf2_vision_fused and t.d435i_vio_age < 1.0:
            self.badge_vision_conf.set_state("ok")
            self.badge_vision_conf.setToolTip("EKF2: POS LOCK")
        elif t.ekf2_vision_fused and t.d435i_vio_age <= 3.0:
            self.badge_vision_conf.set_state("warn")
            self.badge_vision_conf.setToolTip(f"EKF2: DEGRADED ({t.d435i_vio_age:.1f}s)")
        elif t.last_vision_time > 0:
            self.badge_vision_conf.set_state("bad")
            self.badge_vision_conf.setToolTip(f"EKF2: LOST ({t.d435i_vio_age:.0f}s)")
        else:
            self.badge_vision_conf.set_state("")
            self.badge_vision_conf.setToolTip("EKF2: NO VISION")

        # RC link: the SYS_STATUS RC_RECEIVER health bit is the trusted signal.
        if not t.rc_receiver_present:
            self.badge_rc.set_state("")
            self.badge_rc.setToolTip("RC: NO DATA")
        elif not t.rc_receiver_healthy:
            self.badge_rc.set_state("bad")
            self.badge_rc.setToolTip("RC: LOST")
        else:
            rssi_str = f" RSSI {t.rc_rssi}" if 0 <= t.rc_rssi < 255 else ""
            self.badge_rc.set_state("ok")
            self.badge_rc.setToolTip(f"RC: OK{rssi_str}")

        self.badge_mode.setText(f"MODE: {t.flight_mode}")
        if t.armed:
            self.badge_arm.setText("ARMED")
            self.badge_arm.setObjectName("badgeArmed")
        else:
            self.badge_arm.setText("DISARMED")
            self.badge_arm.setObjectName("badgeDisarmed")
        self.badge_arm.setStyle(self.badge_arm.style())

    def update_rates(self, rx: float, tx: float):
        """Retained so MAVLinkWorker's rates_updated wiring is unchanged."""
        return
