#!/usr/bin/env python3
"""
================================================================================
MODULE: radxa_monitor.py
PURPOSE: Dedicated Onboard Passive Telemetry, Motor Actuator, & Command Auditor GUI
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Radxa Q6A Companion Computer (SysID 254) or Remote Field Display
  * Communicates:  Pixhawk 6X Autopilot via mavlink-router (tcp:127.0.0.1:5760)
  * Upstream:      Pixhawk MAVLink stream & Linux sysfs hardware sensors
  * Downstream:    Operator onboard touchscreen / field HDMI monitor

DATA FLOW & INTERFACES:
  * MAVLink In:    HEARTBEAT, LOCAL_POSITION_NED, ATTITUDE, SYS_STATUS,
                   SERVO_OUTPUT_RAW, COMMAND_ACK, STATUSTEXT.
  * Sysfs Read:    /sys/class/thermal/thermal_zone*/temp (SoC temperature),
                   /proc/meminfo (RAM utilization), /proc/stat (CPU load).
  * Outbound C2:   STRICTLY ZERO FLIGHT COMMANDS (No arm, disarm, or mode controls).

KEY LOGIC & FAILSAFES:
  * Purely Passive Safety Guarantee: Designed with zero flight actuation buttons
    to prevent accidental flight commands from being triggered on the companion SBC.
  * Companion Hardware Health Telemetry: Probes SoC core temperatures, RAM buffers,
    and CPU utilization in real time to catch onboard thermal throttling.
  * Live Motor Saturation & Desync Gauges: Visualizes individual ESC PWM levels
    (1000-2000 us) to spot failing motors or esc timing desync before launch.
  * Dual-System Auditing: Logs all flight mode changes, ACKs, and commands dispatched
    by external GCS laptops over the shared mavlink-router network.

RUN:
  # Run directly on Radxa SBC:
  python3 radxa_monitor.py

  # Connect to custom endpoint:
  python3 radxa_monitor.py --host 127.0.0.1 --port 5760
================================================================================
"""

from __future__ import annotations
import math
import os
import sys
import time
from typing import Optional

# Ensure system Qt5 XCB backend for X11 / Wayland compatibility
if sys.platform.startswith("linux"):
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    for _p in ["/usr/lib/aarch64-linux-gnu/qt5/plugins", "/usr/lib/x86_64-linux-gnu/qt5/plugins"]:
        if os.path.exists(_p):
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _p
            break

# Add package root to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QTextCursor
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QFrame, QLabel, QLineEdit, QPushButton,
    QSplitter, QProgressBar, QPlainTextEdit, QTabWidget
)

from core.telemetry import TelemetrySnapshot
from core.execution_tracker import ExecutionTracker
from protocol.mavlink_worker import MAVLinkWorker
from ui.styles import DARK_STYLESHEET
from ui.toast import NotificationToast
from ui.hud_widget import HUDWidget
from ui.motor_widget import MotorWidget


def read_sbc_temperature() -> float:
    """Read SoC thermal sensor on Linux / Radxa."""
    thermal_paths = [
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/thermal/thermal_zone1/temp",
    ]
    for path in thermal_paths:
        if os.path.exists(path):
            try:
                with open(path, "r") as f:
                    val = float(f.read().strip())
                    return val / 1000.0 if val > 1000 else val
            except Exception:
                pass
    return 0.0


def read_sbc_memory_usage() -> float:
    """Read RAM utilization percentage from /proc/meminfo."""
    try:
        with open("/proc/meminfo", "r") as f:
            lines = f.readlines()
        mem_total = 1
        mem_avail = 1
        for line in lines:
            if "MemTotal:" in line:
                mem_total = float(line.split()[1])
            elif "MemAvailable:" in line:
                mem_avail = float(line.split()[1])
        used = mem_total - mem_avail
        return (used / mem_total) * 100.0
    except Exception:
        return 0.0


class RadxaMonitorMainWindow(QMainWindow):
    """Main application window for Radxa Onboard Companion Monitor."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Radxa Onboard Companion Monitor | Pure Receiver & Auditor")
        self.resize(1200, 750)
        self.setMinimumSize(1000, 640)

        # Core Engines
        self.worker: Optional[MAVLinkWorker] = None
        self.exec_tracker = ExecutionTracker(tolerance=0.20, timeout_sec=12.0)
        self.last_telemetry = TelemetrySnapshot()

        # Build UI
        self._init_ui()

        # Toast manager
        self.toast = NotificationToast(self)

        # Periodic refresh timer (20 Hz)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._on_ui_tick)
        self.ui_timer.start(50)

        # Auto-connect locally on launch
        self._toggle_connection()

    def _init_ui(self):
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        root_layout = QVBoxLayout(main_widget)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(8)

        # 1. Top Bar: Title, Local IP/Port Changer, Connection Badge
        top_bar = self._build_top_bar()
        root_layout.addWidget(top_bar)

        # 2. Summary & Hardware Health Cards
        summary_cards = self._build_health_cards()
        root_layout.addWidget(summary_cards)

        # 3. Main Splitter: Left HUD display & Motors, Right Command Auditor
        splitter = QSplitter(Qt.Horizontal, self)
        splitter.setHandleWidth(4)

        # Left: Tabbed view for HUD and Motor Actuators
        left_tabs = QTabWidget(self)

        # Tab 1: PFD HUD
        self.hud = HUDWidget(self)
        left_tabs.addTab(self.hud, "PASSIVE PFD HUD")

        # Tab 2: Motors
        self.motors = MotorWidget(self)
        left_tabs.addTab(self.motors, "LIVE MOTOR PWMs")

        splitter.addWidget(left_tabs)

        # Right: Command Auditor & Displacement Tracker
        audit_container = self._build_audit_container()
        splitter.addWidget(audit_container)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 3)
        root_layout.addWidget(splitter, 1)

    def _build_top_bar(self) -> QFrame:
        bar = QFrame(self)
        bar.setObjectName("topBar")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(10)

        # Title
        title_box = QVBoxLayout()
        title = QLabel("RADXA COMPANION MONITOR")
        title.setObjectName("appTitle")
        title.setStyleSheet("color: #3fb950; font-size: 16px; font-weight: bold;")
        subtitle = QLabel("PASSIVE ONBOARD RECEIVER & AUDITOR • SYS 254 • NO CONTROL BUTTONS")
        subtitle.setObjectName("appSubtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        layout.addLayout(title_box)

        layout.addSpacing(20)

        # Target IP Entry (defaults to 127.0.0.1 on Radxa)
        lbl_ip = QLabel("Mavlink IP:")
        lbl_ip.setStyleSheet("font-weight: bold; color: #8b949e;")
        layout.addWidget(lbl_ip)

        self.ip_input = QLineEdit("127.0.0.1", self)
        self.ip_input.setFixedWidth(120)
        layout.addWidget(self.ip_input)

        # Target Port Entry
        lbl_port = QLabel("Port:")
        lbl_port.setStyleSheet("font-weight: bold; color: #8b949e;")
        layout.addWidget(lbl_port)

        self.port_input = QLineEdit("5760", self)
        self.port_input.setFixedWidth(65)
        layout.addWidget(self.port_input)

        # Connect / Disconnect Button
        self.btn_connect = QPushButton("Connect", self)
        self.btn_connect.setObjectName("btnConnect")
        self.btn_connect.clicked.connect(self._toggle_connection)
        layout.addWidget(self.btn_connect)

        layout.addSpacing(10)

        # Connection Badge
        self.conn_badge = QLabel("DISCONNECTED", self)
        self.conn_badge.setObjectName("badgeDisconnected")
        layout.addWidget(self.conn_badge)

        layout.addStretch()

        self.rates_lbl = QLabel("RX: 0.0 msg/s", self)
        self.rates_lbl.setStyleSheet("color: #8b949e; font-family: Consolas; font-size: 11px;")
        layout.addWidget(self.rates_lbl)

        return bar

    def _build_health_cards(self) -> QFrame:
        frame = QFrame(self)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        def make_card(title_text: str):
            c = QFrame(self)
            c.setProperty("class", "cardFrame")
            cl = QVBoxLayout(c)
            cl.setContentsMargins(10, 6, 10, 6)
            cl.setSpacing(2)
            t = QLabel(title_text)
            t.setProperty("class", "cardTitle")
            v = QLabel("--")
            v.setProperty("class", "cardValue")
            cl.addWidget(t)
            cl.addWidget(v)
            return c, v

        self.card_cpu, self.val_cpu = make_card("RADXA TEMP")
        self.card_ram, self.val_ram = make_card("RADXA RAM")
        self.card_vio, self.val_vio = make_card("D435i VIO")
        self.card_mode, self.val_mode = make_card("PX4 MODE")
        self.card_arm, self.val_arm = make_card("ARM STATE")
        self.card_batt, self.val_batt = make_card("BATTERY")
        self.card_audit, self.val_audit = make_card("AUDITOR VERIFICATION")

        layout.addWidget(self.card_cpu)
        layout.addWidget(self.card_ram)
        layout.addWidget(self.card_vio)
        layout.addWidget(self.card_mode)
        layout.addWidget(self.card_arm)
        layout.addWidget(self.card_batt)
        layout.addWidget(self.card_audit, 1)

        return frame

    def _build_audit_container(self) -> QWidget:
        container = QFrame(self)
        container.setProperty("class", "cardFrame")
        layout = QVBoxLayout(container)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # Title
        header = QHBoxLayout()
        h_title = QLabel("INCOMING GCS COMMAND & EXECUTION AUDITOR")
        h_title.setStyleSheet("font-weight: bold; color: #58a6ff; font-size: 12px;")
        header.addWidget(h_title)
        header.addStretch()

        btn_clear = QPushButton("Clear Audit Log")
        btn_clear.clicked.connect(self._clear_audit_log)
        header.addWidget(btn_clear)
        layout.addLayout(header)

        # Live Physical Displacement Status Box
        disp_box = QFrame(self)
        disp_box.setStyleSheet("background-color: #0d1117; border: 1px solid #30363d; border-radius: 6px; padding: 6px;")
        dl = QVBoxLayout(disp_box)
        dl.setContentsMargins(8, 6, 8, 6)
        dl.setSpacing(4)

        self.lbl_active_cmd = QLabel("Active Command: None (Listening for [GCS CMD])...", self)
        self.lbl_active_cmd.setStyleSheet("font-weight: bold; color: #f0f6fc; font-size: 12px;")
        dl.addWidget(self.lbl_active_cmd)

        self.lbl_disp_detail = QLabel("Displacement Δd: 0.00m | Target: 0.00m | Status: IDLE", self)
        self.lbl_disp_detail.setStyleSheet("color: #8b949e; font-family: Consolas; font-size: 11px;")
        dl.addWidget(self.lbl_disp_detail)

        self.audit_progress = QProgressBar(self)
        self.audit_progress.setFixedHeight(12)
        self.audit_progress.setTextVisible(False)
        self.audit_progress.setValue(0)
        dl.addWidget(self.audit_progress)

        layout.addWidget(disp_box)

        # Audit History Text Box
        self.audit_log = QPlainTextEdit(self)
        self.audit_log.setReadOnly(True)
        self.audit_log.setFont(QFont("Consolas", 10))
        self.audit_log.setMaximumBlockCount(1000)
        layout.addWidget(self.audit_log, 1)

        return container

    def _clear_audit_log(self):
        self.audit_log.clear()

    def _append_audit_log(self, text: str, color_hex: str = "#c9d1d9"):
        ts = time.strftime("%H:%M:%S")
        html = f'<span style="color:#8b949e">[{ts}]</span> <span style="color:{color_hex}">{text}</span>'
        self.audit_log.appendHtml(html)
        self.audit_log.moveCursor(QTextCursor.End)

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------

    def _toggle_connection(self):
        if self.worker and self.worker.isRunning():
            self._append_audit_log("Disconnecting from MAVLink stream...", "#d29922")
            self.worker.disconnect_endpoint()
            self.worker = None
            self.btn_connect.setText("Connect")
            self.btn_connect.setObjectName("btnConnect")
            self.btn_connect.setStyle(self.btn_connect.style())
            self.conn_badge.setText("DISCONNECTED")
            self.conn_badge.setObjectName("badgeDisconnected")
            self.conn_badge.setStyle(self.conn_badge.style())
        else:
            ip = self.ip_input.text().strip()
            port = int(self.port_input.text().strip())
            self._append_audit_log(f"Listening on tcp:{ip}:{port} (SysID 254 - Passive)...", "#58a6ff")

            # Receiver uses source_system=254 (Passive Monitor)
            self.worker = MAVLinkWorker(host=ip, port=port, source_system=254)
            self.worker.telemetry_updated.connect(self._on_telemetry_updated)
            self.worker.connection_changed.connect(self._on_connection_changed)
            self.worker.command_broadcast_received.connect(self._on_command_intercepted)
            self.worker.command_ack_received.connect(self._on_command_ack_intercepted)
            self.worker.rates_updated.connect(self._on_rates_updated)
            self.worker.start()

            self.btn_connect.setText("Disconnect")
            self.btn_connect.setObjectName("btnDisconnect")
            self.btn_connect.setStyle(self.btn_connect.style())

    def _on_connection_changed(self, connected: bool, message: str):
        if connected:
            self.conn_badge.setText(f"CONNECTED ({self.worker.host})")
            self.conn_badge.setObjectName("badgeConnected")
            self.conn_badge.setStyle(self.conn_badge.style())
            self._append_audit_log(f"Link Established: {message}", "#3fb950")
            self.toast.show_message(f"Radxa Monitor: {message}", "#238636")
        else:
            self.conn_badge.setText("DISCONNECTED")
            self.conn_badge.setObjectName("badgeDisconnected")
            self.conn_badge.setStyle(self.conn_badge.style())
            self._append_audit_log(f"Link Status: {message}", "#d29922")

    def _on_rates_updated(self, rx_rate: float, tx_rate: float):
        self.rates_lbl.setText(f"RX: {rx_rate:.1f} msg/s")

    def _on_telemetry_updated(self, t: TelemetrySnapshot):
        self.last_telemetry = t
        self.hud.update_telemetry(t)
        self.motors.update_pwms(t.motor_pwms)

    def _on_command_intercepted(self, cmd_text: str):
        """Called when Pilot GCS sends [GCS CMD] broadcast."""
        t = self.last_telemetry
        self._append_audit_log(
            f"[GCS CMD] '{cmd_text}' | Baseline NED: ({t.x:+.2f}, {t.y:+.2f}, {t.z:+.2f})m",
            "#58a6ff"
        )
        self.lbl_active_cmd.setText(f"Active Command: {cmd_text}")
        self.toast.show_message(f"Incoming GCS Cmd: {cmd_text}", "#58a6ff")

        # Prime the closed-loop motion verifier
        self.exec_tracker.start_tracking(cmd_text, t.x, t.y, t.z)

    def _on_command_ack_intercepted(self, cmd_id: int, result_code: int, cmd_name: str, result_str: str):
        """Audit Pixhawk COMMAND_ACK responses."""
        if result_code == 0:
            self._append_audit_log(f"[PIXHAWK ACK] {cmd_name} ACCEPTED", "#3fb950")
        else:
            self._append_audit_log(f"[PIXHAWK ACK] {cmd_name} {result_str} (Code {result_code})", "#da3633")

    # -------------------------------------------------------------------------
    # UI Periodic Tick (20 Hz)
    # -------------------------------------------------------------------------

    def _on_ui_tick(self):
        t = self.last_telemetry

        # 1. Update Hardware Health
        temp_c = read_sbc_temperature()
        ram_pct = read_sbc_memory_usage()

        if temp_c > 0:
            self.val_cpu.setText(f"{temp_c:.1f} °C")
            self.val_cpu.setStyleSheet("color: #da3633;" if temp_c > 75 else "color: #3fb950;")
        else:
            self.val_cpu.setText("N/A")

        self.val_ram.setText(f"{ram_pct:.1f}%")

        # 2. Update D435i VIO state
        if t.d435i_vio_health:
            self.val_vio.setText("LOCKED")
            self.val_vio.setStyleSheet("color: #3fb950;")
        else:
            self.val_vio.setText("NO VIO")
            self.val_vio.setStyleSheet("color: #d29922;")

        # 3. Update Drone State Cards
        if t.armed:
            self.val_arm.setText("ARMED")
            self.val_arm.setStyleSheet("color: #3fb950;")
        else:
            self.val_arm.setText("DISARMED")
            self.val_arm.setStyleSheet("color: #8b949e;")

        self.val_mode.setText(t.flight_mode)
        self.val_batt.setText(f"{t.battery_voltage:.1f}V ({t.battery_percent}%)")

        # 4. Update Closed-Loop Execution Tracking
        if self.exec_tracker.active:
            res = self.exec_tracker.update(t.x, t.y, t.z)
            pct = int(res["progress_pct"])
            self.audit_progress.setValue(pct)
            self.lbl_disp_detail.setText(
                f"Δd: {res['delta']:.2f}m / {res['target']:.2f}m ({pct}%) | Status: {res['status']}"
            )
            self.val_audit.setText(f"TRACKING {pct}% ({res['delta']:.2f}m)")
            self.val_audit.setStyleSheet("color: #d29922;")

            if res["status"] == "EXECUTED":
                self.val_audit.setText("[EXECUTED]")
                self.val_audit.setStyleSheet("color: #3fb950;")
                self._append_audit_log(f"[PHYSICALLY VERIFIED] {res['status_msg']}", "#3fb950")
                self.toast.show_message(res["status_msg"], "#238636")
            elif res["status"] == "STALLED":
                self.val_audit.setText("[STALLED]")
                self.val_audit.setStyleSheet("color: #da3633;")
                self._append_audit_log(f"[EXECUTION STALLED] {res['status_msg']}", "#da3633")
                self.toast.show_message(res["status_msg"], "#da3633")
        else:
            if self.exec_tracker.status == "EXECUTED":
                self.val_audit.setText("[EXECUTED]")
                self.val_audit.setStyleSheet("color: #3fb950;")
            elif self.exec_tracker.status == "STALLED":
                self.val_audit.setText("[STALLED]")
                self.val_audit.setStyleSheet("color: #da3633;")
            else:
                self.val_audit.setText("IDLE (Awaiting)")
                self.val_audit.setStyleSheet("color: #8b949e;")


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLESHEET)
    window = RadxaMonitorMainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
