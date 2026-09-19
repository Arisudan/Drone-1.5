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

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QFrame, QHBoxLayout, QVBoxLayout, QLabel, QLineEdit, QPushButton, QComboBox
)

from core.telemetry import TelemetrySnapshot
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
    """Aerospace status bar with dynamic connection toggling and telemetry badges."""

    connect_requested = pyqtSignal(str, int, str)
    disconnect_requested = pyqtSignal()
    network_changed = pyqtSignal(str)  # emits the resolved IP whenever a network preset is picked

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
        SPACING = 14

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

        # 2. Dynamic IP, Port, & Protocol Inputs
        conn_box = QHBoxLayout()
        conn_box.setSpacing(SPACING)

        lbl_net = QLabel("Network:")
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
        self.ip_input.setFixedWidth(120)
        self.ip_input.setToolTip("Target SBC IP (e.g. 172.16.101.84 or 127.0.0.1) - auto-filled by the Network dropdown, or type your own")
        self.ip_input.textEdited.connect(self._on_ip_hand_edited)
        conn_box.addWidget(self.ip_input)

        lbl_proto = QLabel("Protocol:")
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
        self.port_input.setFixedWidth(70)  # 60 clipped the leading digit of "14550"
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
        row1.addLayout(conn_box)
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

        # Badges left-aligned so they sit directly beneath the connection
        # controls above them: one column of things you set, one column of
        # things the vehicle reports back.
        row2.addLayout(telemetry_strip)
        row2.addStretch(1)

        # Arm state and flight mode sit on the second line, directly beneath the
        # battery: the header's right-hand column is then "how is the vehicle",
        # top to bottom, with the controls you operate on the left.
        row2.addLayout(mode_arm_box)



        rows.addLayout(row2)
        layout.addLayout(rows, 1)

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
