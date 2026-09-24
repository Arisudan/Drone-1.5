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
      - Armed / Disarmed: Bright warning red when armed, subdued slate when disarmed.
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

from PyQt5.QtCore import Qt, QRect, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QFrame, QHBoxLayout, QVBoxLayout, QLabel, QLineEdit, QPushButton,
    QComboBox, QSpacerItem, QSizePolicy
)

from core.telemetry import TelemetrySnapshot
from ui.scaling import px
from ui.battery_badge import BatteryBadge

# The networks this rig is actually deployed on. Picking one here just fills in the
# IP - port and protocol are independent axes (same MAVLink/map/video ports apply on
# any of these networks) and are left for the operator to choose separately.
KNOWN_NETWORKS = [
    ("HTIC_RND", "172.16.101.84"),
    ("DroneBridge5", "192.168.1.2"),
    # DroneNet: a NetworkManager connection-sharing/hotspot link - Radxa is always the
    # shared-connection gateway at 10.42.0.1 (the laptop gets a lease like 10.42.0.200,
    # but that's not needed here since the GCS only ever connects out to the Radxa).
    ("DroneNet", "10.42.0.1"),
]


class TopStatusStrip(QFrame):
    # Gap between header controls. A class attribute rather than an
    # __init__ local: the relayout logic below needs it too, and a local
    # meant those methods raised NameError inside a resize event, where
    # Qt swallows the traceback and simply stops laying the header out.
    SPACING = 14

    """Aerospace status bar with dynamic connection toggling and telemetry badges."""

    connect_requested = pyqtSignal(str, int, str)
    disconnect_requested = pyqtSignal()
    network_changed = pyqtSignal(str)
    mute_toggled = pyqtSignal(bool)  # emits the resolved IP whenever a network preset is picked

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("topBar")
        self.is_connected = False

        # Two logical rows instead of one packed line: row 1 is "things you configure/do"
        # (branding + connection controls), row 2 is "things you observe" (live telemetry
        # badges + throughput). Splitting them keeps either row from overflowing on a
        # narrower screen instead of everything fighting for space in one long strip.
        # One spacing constant used everywhere in this header instead of a different
        # ad hoc value per row/group (was 8/10/12/16/24 scattered around) - every gap
        # between adjacent items is now the same size.
        SPACING = self.SPACING

        layout = QHBoxLayout(self)
        layout.setContentsMargins(14, 6, 10, 6)
        layout.setSpacing(18)

        # 1. System Branding - occupies the full header height on the left
        # rather than sharing row 1 with the connection controls. The brand is
        # the one element that never changes, so it anchors the corner and the
        # live readouts get the whole width to its right.
        title_box = QVBoxLayout()
        title_box.setSpacing(0)
        title_box.setContentsMargins(0, 0, 0, 0)
        self.title_lbl = QLabel("DRONE-GCS", self)
        self.title_lbl.setObjectName("appTitle")
        # "SYS 255" dropped: the MAVLink source ID is a protocol detail, not
        # something an operator reads off the masthead.
        self.sub_lbl = QLabel("AUTONOMOUS FLIGHT SYSTEM", self)
        self.sub_lbl.setObjectName("appSubtitle")
        title_box.addStretch()
        title_box.addWidget(self.title_lbl)
        title_box.addWidget(self.sub_lbl)
        title_box.addStretch()
        layout.addLayout(title_box)

        brand_rule = QFrame(self)
        brand_rule.setObjectName("vDivider")
        brand_rule.setFixedWidth(1)
        layout.addWidget(brand_rule)

        rows = QVBoxLayout()
        rows.setSpacing(6)

        row1 = QHBoxLayout()
        row1.setSpacing(SPACING)

        # 2. Dynamic IP, Port, & Protocol Inputs.
        # These live in their own widget rather than straight in row 1 so the
        # header can move them onto a line of their own when it is too narrow
        # to hold them beside the brand and the battery - see _relayout_rows.
        self.conn_panel = QWidget(self)
        conn_box = QHBoxLayout(self.conn_panel)
        conn_box.setContentsMargins(0, 0, 0, 0)
        conn_box.setSpacing(SPACING)

        lbl_net = QLabel("Net:")
        lbl_net.setStyleSheet("color: #8b949e; font-weight: bold; font-size: 11px;")
        conn_box.addWidget(lbl_net)

        self.network_combo = QComboBox(self)
        # Name only, not "Name (IP)" - the IP is already shown right next to this in the
        # Target IP field, so repeating it here just forces the combo wider for no
        # extra information; the full mapping is in the tooltip instead.
        for name, ip in KNOWN_NETWORKS:
            self.network_combo.addItem(name, ip)
        self.network_combo.addItem("Custom...", None)
        self.network_combo.setToolTip(
            "Pick the Wi-Fi network in use - fills in the Radxa's IP for MAVLink, the map "
            "bridge, and the FPV stream in one go.\n" +
            "\n".join(f"{name}: {ip}" for name, ip in KNOWN_NETWORKS)
        )
        self.network_combo.currentIndexChanged.connect(self._on_network_selected)
        conn_box.addWidget(self.network_combo)

        lbl_ip = QLabel("IP:")
        lbl_ip.setStyleSheet("color: #8b949e; font-weight: bold; font-size: 11px;")
        conn_box.addWidget(lbl_ip)

        self.ip_input = QLineEdit(KNOWN_NETWORKS[0][1], self)
        self.ip_input.setFixedWidth(px(120))
        self.ip_input.setToolTip("Target SBC IP (e.g. 172.16.101.84 or 127.0.0.1) - auto-filled by the Network dropdown, or type your own")
        self.ip_input.textEdited.connect(self._on_ip_hand_edited)
        conn_box.addWidget(self.ip_input)

        lbl_proto = QLabel("Proto:")
        lbl_proto.setStyleSheet("color: #8b949e; font-weight: bold; font-size: 11px;")
        conn_box.addWidget(lbl_proto)

        self.proto_combo = QComboBox(self)
        self.proto_combo.addItem("UDP", "udp")
        self.proto_combo.addItem("TCP", "tcp")
        self.proto_combo.setToolTip("Protocol: UDP for real-time flight (port 14550); TCP for bench fallback (port 5760)")
        self.proto_combo.currentIndexChanged.connect(self._on_protocol_changed)
        conn_box.addWidget(self.proto_combo)

        lbl_port = QLabel("Port:")
        lbl_port.setStyleSheet("color: #8b949e; font-weight: bold; font-size: 11px;")
        conn_box.addWidget(lbl_port)

        self.port_input = QLineEdit("14550", self)
        self.port_input.setFixedWidth(px(70))  # 60 clipped the leading digit of "14550"
        self.port_input.setToolTip("Target Port (UDP flight: 14550, TCP bench: 5760)")
        conn_box.addWidget(self.port_input)

        # The Connect/Disconnect button alone carries connection state now - its text
        # ("Connect"/"Disconnect") and color (btnConnect/btnDisconnect) already say it
        # fully. A separate "CONNECTED"/"DISCONNECTED" badge used to sit next to it
        # saying the same thing in different words; dropped rather than kept as a
        # second thing to read for one piece of information.
        # The button is a different kind of thing from the four fields before
        # it - a dispatch, not a value - so it gets a wider gap than the
        # uniform spacing between the fields themselves.
        conn_box.addSpacing(SPACING)

        self.btn_toggle = QPushButton("CONNECT", self)
        self.btn_toggle.setObjectName("btnConnect")
        self.btn_toggle.clicked.connect(self._handle_connect_toggle)
        conn_box.addWidget(self.btn_toggle)

        # Left-aligned so the controls sit directly above the RX/TX and VIO NED
        # readouts in row 2: what you configure and what it produces line up in
        # the same column instead of sitting at opposite ends of the header.
        row1.addWidget(self.conn_panel)
        row1.addStretch()

        # 3. Mode / Arm status - pinned to the top-right corner, single line (Arm to the
        # left of Mode). Was a two-line stack (Mode/Arm), but that made the top-right
        # corner reach as far down as row 2, crowding the notification toast's new home
        # in row 2's freed right-hand space; one line keeps it entirely inside row 1.
        mode_arm_box = QHBoxLayout()
        mode_arm_box.setSpacing(SPACING)

        self.badge_arm = QLabel("DISARMED", self)
        self.badge_arm.setObjectName("badgeDisarmed")
        mode_arm_box.addWidget(self.badge_arm)

        self.badge_mode = QLabel("MODE: DISCONNECTED", self)
        # font-size matches QLabel#badgeArmed/#badgeDisarmed in styles.py so the
        # pair reads as one control rather than two sizes of label.
        self.badge_mode.setStyleSheet("background-color: rgba(31, 111, 235, 0.13); color: #58a6ff; border: 1px solid #1f6feb; border-radius: 4px; padding: 3px 8px; font-weight: bold; font-size: 10px; letter-spacing: 1px;")
        mode_arm_box.addWidget(self.badge_mode)

        # Battery in the very corner, laptop-style: voltage, cell, percentage.
        # It is the value most often glanced at from across a room, so it gets
        # the one position the eye finds without searching.
        row1.addSpacing(SPACING)
        self.battery = BatteryBadge(self)
        row1.addWidget(self.battery)

        rows.addLayout(row1)

        # Row 2: remaining live telemetry readouts (Battery, D435i VIO, EKF2, throughput)
        row2 = QHBoxLayout()
        row2.setSpacing(SPACING)

        telemetry_strip = QHBoxLayout()
        telemetry_strip.setSpacing(SPACING)

        # D435i VIO Badge (LOCKED / LOST, matching the HUD's own wording)
        self.badge_vio = QLabel("VIO: LOST", self)
        self.badge_vio.setStyleSheet("background-color: #161b22; color: #8b949e; border: 1px solid #30363d; border-radius: 4px; padding: 4px 8px; font-weight: bold; font-size: 11px;")
        telemetry_strip.addWidget(self.badge_vio)

        # Vision Confidence / Fusion Badge (Replaces dead GPS badge on GPS-denied airframe)
        self.badge_vision_conf = QLabel("EKF2: NO VISION", self)
        self.badge_vision_conf.setStyleSheet("background-color: #161b22; color: #8b949e; border: 1px solid #30363d; border-radius: 4px; padding: 4px 8px; font-weight: bold; font-size: 11px;")
        telemetry_strip.addWidget(self.badge_vision_conf)
        self.badge_gps = self.badge_vision_conf  # Backward-compatible alias

        # RC Link Badge (ExpressLRS receiver). Primary signal is the SYS_STATUS
        # RC_RECEIVER health bit, proven live to be the only reliable RC-loss
        # detector on this hardware - RC_CHANNELS.rssi is shown as a secondary
        # best-effort number only when the receiver actually populates one
        # (this ELRS receiver reports rssi=255/"unknown" even on a healthy link).
        self.badge_rc = QLabel("RC: NO DATA", self)
        self.badge_rc.setStyleSheet("background-color: #161b22; color: #8b949e; border: 1px solid #30363d; border-radius: 4px; padding: 4px 8px; font-weight: bold; font-size: 11px;")
        telemetry_strip.addWidget(self.badge_rc)

        # Audio mute. A control rather than a badge, because muting is something
        # the operator does - typically the moment a fault starts repeating and
        # they have already seen it. Ctrl+M does the same thing.
        self.btn_mute = QPushButton("\U0001F50A AUDIO", self)
        self.btn_mute.setCheckable(True)
        self.btn_mute.setCursor(Qt.PointingHandCursor)
        self.btn_mute.setToolTip("Mute or unmute spoken and tonal alerts (Ctrl+M)")
        self.btn_mute.clicked.connect(self._on_mute_clicked)
        self._style_mute(False, True)
        telemetry_strip.addWidget(self.btn_mute)

        # Badges left-aligned so they sit directly beneath the connection
        # controls above them: one column of things you set, one column of
        # things the vehicle reports back.
        row2.addLayout(telemetry_strip)
        # Reserved notification band. The toast lives in this gap, so the gap is
        # a real layout requirement rather than incidental slack: without a
        # floor it collapses on a narrow window and the toast has nowhere to go
        # that is not already occupied by a badge.
        row2.addSpacerItem(QSpacerItem(px(110), 0, QSizePolicy.MinimumExpanding,
                                       QSizePolicy.Minimum))

        # Arm state and flight mode sit on the second line, directly beneath the
        # battery: the header's right-hand column is then "how is the vehicle",
        # top to bottom, with the controls you operate on the left.
        row2.addLayout(mode_arm_box)



        rows.addLayout(row2)

        # The narrow-layout home for the connection controls. Empty and zero
        # height until _relayout_rows needs it.
        self.row_conn = QHBoxLayout()
        self.row_conn.setSpacing(SPACING)
        self.row_conn.setContentsMargins(0, 0, 0, 0)
        rows.addLayout(self.row_conn)

        self._row1 = row1
        self._conn_on_own_row = False

        layout.addLayout(rows, 1)

    # Widgets that bound the notification band, left and right. Named as lists
    # rather than as one anchor widget because the header gets reshuffled: the
    # toast used to anchor off badge_rc's right edge, and when the AUDIO control
    # was added to the row after it, every notification was drawn straight over
    # the word AUDIO. Bounding the band by "the rightmost thing on the left" and
    # "the leftmost thing on the right" survives that.
    _BAND_LEFT = ("badge_vio", "badge_vision_conf", "badge_rc", "btn_mute")
    _BAND_RIGHT = ("badge_arm", "badge_mode")

    def notification_rect(self) -> QRect:
        """The clear band on row 2, in this widget's own coordinates.

        Empty when the row cannot be measured, which the caller must treat as
        "no safe place here" rather than falling back to an overlapping guess.
        """
        def present(names):
            out = []
            for name in names:
                w = getattr(self, name, None)
                if w is not None and not w.isHidden():
                    out.append(w)
            return out

        left = present(self._BAND_LEFT)
        right = present(self._BAND_RIGHT)
        if not left or not right:
            return QRect()

        gap = px(10)
        x = max(w.geometry().right() for w in left) + 1 + gap
        limit = min(w.geometry().left() for w in right) - gap
        reference = left[-1].geometry()
        return QRect(x, reference.top(), max(0, limit - x), reference.height())

    # Extra width demanded before the connection controls are allowed back up
    # onto the first row. Without hysteresis the two layouts sit either side of
    # a single pixel and the header flickers between them while the window is
    # being dragged.
    _RELAYOUT_HYSTERESIS_PX = 48

    def _row1_width_needed(self) -> int:
        """Width row 1 wants with the connection controls on it."""
        need = self.conn_panel.sizeHint().width()
        for widget in (self.badge_arm, self.badge_mode):
            if widget is not None and not widget.isHidden():
                need += widget.sizeHint().width() + self.SPACING
        battery = getattr(self, "battery", None)
        if battery is not None:
            need += max(battery.sizeHint().width(),
                        battery.minimumWidth()) + self.SPACING
        # The brand block and the rule to its right are outside `rows`.
        need += self.title_lbl.sizeHint().width() + self.SPACING * 3
        return need

    def _relayout_rows(self) -> None:
        """Put the connection controls on their own line when they do not fit.

        Row 1 holds the brand, the connection controls and the battery. At a
        large UI scale on a small screen those cannot sit side by side: measured
        at 1.35x in a 1220px window, row 1 wanted about 1430px, and a Qt layout
        given less than it needs does not shrink its children - it overlaps
        them. The target IP field was drawn straight across the "Protocol:"
        caption and CONNECT was clipped to ")NNE(".

        Moving one block onto its own line costs a row of header height and
        fixes both rows, because the strip's minimum width - and therefore the
        compression applied to every row in it - is set by the widest one.
        """
        if not hasattr(self, "row_conn"):
            return
        needed = self._row1_width_needed()
        available = self.width()

        if not self._conn_on_own_row and available < needed:
            self._row1.removeWidget(self.conn_panel)
            self.row_conn.addWidget(self.conn_panel)
            self.row_conn.addStretch()
            # The subtitle is decoration, and the brand block is as wide as
            # whichever of its two lines is longer - so dropping it is the
            # cheapest width available at exactly the moment width is scarce.
            self.sub_lbl.setVisible(False)
            self._conn_on_own_row = True
        elif self._conn_on_own_row and available > needed + self._RELAYOUT_HYSTERESIS_PX:
            self.row_conn.removeWidget(self.conn_panel)
            while self.row_conn.count():
                self.row_conn.takeAt(0)
            self._row1.insertWidget(0, self.conn_panel)
            self.sub_lbl.setVisible(True)
            self._conn_on_own_row = False

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._relayout_rows()

    def _style_mute(self, muted: bool, available: bool) -> None:
        """Three visual states, because they mean three different things:
        sounding, deliberately muted, and no audio backend at all."""
        if not available:
            text, fg, bg, border = "\U0001F507 NO AUDIO", "#6e7681", "#161b22", "#30363d"
        elif muted:
            text, fg, bg, border = "\U0001F507 MUTED", "#d29922", "#2b2109", "#9e6a03"
        else:
            text, fg, bg, border = "\U0001F50A AUDIO", "#3fb950", "#122a19", "#238636"
        self.btn_mute.setText(text)
        self.btn_mute.setStyleSheet(
            f"QPushButton {{ background-color: {bg}; color: {fg};"
            f" border: 1px solid {border}; border-radius: 4px;"
            " padding: 4px 8px; font-weight: bold; font-size: 11px; }")

    def _on_mute_clicked(self) -> None:
        self.mute_toggled.emit(self.btn_mute.isChecked())

    def set_audio_state(self, muted: bool, available: bool) -> None:
        """Reflect the alert service's real state, whoever changed it - the
        header button, Ctrl+M, or a settings save."""
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
        self.network_changed.emit(ip)

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
        if proto == "udp" and current_port in ("5760", ""):
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
        self.btn_toggle.setStyle(self.btn_toggle.style())

    def update_telemetry(self, t: TelemetrySnapshot):
        # Battery - the widget owns its own colour thresholds and no-data state.
        self.battery.set_state(t.battery_voltage, t.battery_percent, t.connected)

        # Speed and altitude now live on the navigation rail - see
        # SidebarNav.set_instruments, driven from drone_gcs's telemetry tick.

        # D435i VIO Raw Stream Badge - LOCKED (healthy) / LOST (stale or never seen),
        # matching the HUD widget's own "D435i VIO: LOCKED / NO DATA" wording.
        if t.d435i_vio_health and t.d435i_vio_age <= 3.0:
            self.badge_vio.setText(f"VIO: LOCKED ({t.d435i_vio_age:.1f}s)")
            self.badge_vio.setStyleSheet("background-color: rgba(35, 134, 54, 0.13); color: #3fb950; border: 1px solid #238636; border-radius: 4px; padding: 4px 7px; font-weight: bold; font-size: 11px;")
        else:
            self.badge_vio.setText("VIO: LOST" if t.last_vision_time > 0 else "VIO: NO DATA")
            self.badge_vio.setStyleSheet("background-color: #161b22; color: #8b949e; border: 1px solid #30363d; border-radius: 4px; padding: 4px 7px; font-weight: bold; font-size: 11px;")

        # EKF2 Vision Confidence / Fusion Stat (GPS-denied indicator)
        if t.ekf2_vision_fused and t.d435i_vio_age < 1.0:
            self.badge_vision_conf.setText("EKF2: POS LOCK")
            self.badge_vision_conf.setStyleSheet("background-color: rgba(35, 134, 54, 0.13); color: #3fb950; border: 1px solid #238636; border-radius: 4px; padding: 4px 7px; font-weight: bold; font-size: 11px;")
        elif t.ekf2_vision_fused and t.d435i_vio_age <= 3.0:
            self.badge_vision_conf.setText(f"EKF2: DEGRADED ({t.d435i_vio_age:.1f}s)")
            self.badge_vision_conf.setStyleSheet("background-color: rgba(158, 106, 3, 0.13); color: #d29922; border: 1px solid #d29922; border-radius: 4px; padding: 4px 7px; font-weight: bold; font-size: 11px;")
        elif t.last_vision_time > 0:
            self.badge_vision_conf.setText(f"EKF2: LOST ({t.d435i_vio_age:.0f}s)")
            self.badge_vision_conf.setStyleSheet("background-color: rgba(218, 54, 51, 0.13); color: #f85149; border: 1px solid #da3633; border-radius: 4px; padding: 4px 7px; font-weight: bold; font-size: 11px;")
        else:
            self.badge_vision_conf.setText("EKF2: NO VISION")
            self.badge_vision_conf.setStyleSheet("background-color: #161b22; color: #8b949e; border: 1px solid #30363d; border-radius: 4px; padding: 4px 7px; font-weight: bold; font-size: 11px;")

        # RC Link (ExpressLRS receiver) - see badge_rc creation comment for why
        # rc_receiver_healthy (SYS_STATUS bit), not rssi, is the trusted signal.
        if not t.rc_receiver_present:
            self.badge_rc.setText("RC: NO DATA")
            self.badge_rc.setStyleSheet("background-color: #161b22; color: #8b949e; border: 1px solid #30363d; border-radius: 4px; padding: 4px 7px; font-weight: bold; font-size: 11px;")
        elif not t.rc_receiver_healthy:
            self.badge_rc.setText("RC: LOST")
            self.badge_rc.setStyleSheet("background-color: rgba(218, 54, 51, 0.13); color: #f85149; border: 1px solid #da3633; border-radius: 4px; padding: 4px 7px; font-weight: bold; font-size: 11px;")
        else:
            rssi_str = f" RSSI {t.rc_rssi}" if 0 <= t.rc_rssi < 255 else ""
            self.badge_rc.setText(f"RC: OK{rssi_str}")
            self.badge_rc.setStyleSheet("background-color: rgba(35, 134, 54, 0.13); color: #3fb950; border: 1px solid #238636; border-radius: 4px; padding: 4px 7px; font-weight: bold; font-size: 11px;")

        # Mode
        self.badge_mode.setText(f"MODE: {t.flight_mode}")

        # Arm
        if t.armed:
            self.badge_arm.setText("ARMED")
            self.badge_arm.setObjectName("badgeArmed")
        else:
            self.badge_arm.setText("DISARMED")
            self.badge_arm.setObjectName("badgeDisarmed")
        self.badge_arm.setStyle(self.badge_arm.style())

    def update_rates(self, rx: float, tx: float):
        """Retained so MAVLinkWorker's rates_updated wiring is unchanged.

        Packet rates and the NED triple moved to the Diagnostics tab: both are
        debugging detail rather than at-a-glance flight state, and the NED
        readout was already duplicated by that tab's own position card.
        """
        return
