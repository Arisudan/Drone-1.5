#!/usr/bin/env python3
"""
================================================================================
MODULE: drone_gcs.py
PURPOSE: Next-Gen Industrial Ground Control Station (GCS) Main Window & Pilot Deck
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station or Radxa Q6A SBC with GUI Display
  * Communicates:  Pixhawk 6X (MAVLink port 5760) & Radxa Map Streamer (TCP port 5765)
  * Upstream:      Pilot user interactions, click-to-fly waypoints, and CLI console
  * Downstream:    MAVLinkWorker, ROS2MapListener, RViz Embed, HUD, and SLAM Widgets

DATA FLOW & INTERFACES:
  * Telemetry In:  LOCAL_POSITION_NED, ATTITUDE, SYS_STATUS, SERVO_OUTPUT_RAW,
                   COMMAND_ACK, STATUSTEXT, OBSTACLE_DISTANCE, /map_thin, /map.
  * Commands Out:  MAV_CMD_DO_SET_MODE (OFFBOARD/POSCTL/MANUAL/LAND),
                   MAV_CMD_COMPONENT_ARM_DISARM (normal & bench force-arm 21196),
                   SET_POSITION_TARGET_LOCAL_NED (A* waypoint missions & 20 Hz streams),
                   MAV_CMD_NAV_TAKEOFF, EMERGENCY_KILL.
  * UI Tabs:       Flight Deck (PFD HUD, Motors), Tactical SLAM (2D Grid & 3D RViz),
                   Video Feed, Diagnostics, and Interactive CLI Console.

KEY LOGIC & FAILSAFES:
  * Closed-Loop Physical Verification: Employs ExecutionTracker to confirm physical
    displacement in Euclidean space, alerting if commands stall or fail to move.
  * A* Collision-Free Path Planning: Plans real-time paths around SLAM obstacles
    with automated robot radius dilation and line-of-sight waypoint extraction.
  * Gated Arming & Emergency Kill: Requires dual-stage confirmation modals for motor
    arming and provides an instant one-click EMERGENCY KILL failsafe.
  * Dual-Transport SLAM Map Ingestion: Bypasses Wi-Fi multicast issues via automated
    fallback to TCP binary stream on port 5765.

RUN:
  source /opt/ros/jazzy/setup.bash
  python3 drone_gcs.py

  # Connect to custom companion SBC IP:
  python3 drone_gcs.py --host 172.16.101.84 --port 5760
================================================================================
"""

from __future__ import annotations
import math
import os
import sys
import time
from typing import Optional, List, Tuple

# Auto-configure ROS 2 Jazzy dynamic library path before any C-extensions load
if sys.platform.startswith("linux"):
    if "QT_QPA_PLATFORM" not in os.environ:
        os.environ["QT_QPA_PLATFORM"] = "xcb"
    for _p in ["/usr/lib/aarch64-linux-gnu/qt5/plugins", "/usr/lib/x86_64-linux-gnu/qt5/plugins"]:
        if os.path.exists(_p):
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _p
            break

    # If ROS 2 Jazzy libraries are not in LD_LIBRARY_PATH, re-exec once
    ros_ld = "/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib"
    curr_ld = os.environ.get("LD_LIBRARY_PATH", "")
    if "/opt/ros/jazzy/lib" not in curr_ld and os.path.exists("/opt/ros/jazzy/lib"):
        os.environ["LD_LIBRARY_PATH"] = f"{ros_ld}:{curr_ld}" if curr_ld else ros_ld
        ros_py = "/opt/ros/jazzy/lib/python3.12/site-packages"
        curr_py = os.environ.get("PYTHONPATH", "")
        if ros_py not in curr_py:
            os.environ["PYTHONPATH"] = f"{ros_py}:{curr_py}" if curr_py else ros_py
        try:
            os.execv(sys.executable, [sys.executable] + sys.argv)
        except Exception:
            pass

# Add package root to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QDoubleValidator
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QFrame, QLabel, QPushButton, QStackedWidget,
    QSplitter, QProgressBar, QMessageBox, QLineEdit, QComboBox, QCheckBox
)

from core.telemetry import TelemetrySnapshot
from core.execution_tracker import ExecutionTracker
from core.path_planner import AStarPathPlanner
from protocol.mavlink_worker import MAVLinkWorker
from protocol.ros2_map_listener import ROS2MapListener, SlamMapResetWorker
from ui.styles import DARK_STYLESHEET
from ui.toast import NotificationToast
from ui.hud_widget import HUDWidget
from ui.slam_map_widget import SLAMMapWidget
from ui.cli_console import CLIConsoleWidget
from ui.motor_widget import MotorWidget
from ui.video_feed_widget import VideoFeedWidget
from ui.top_status_strip import TopStatusStrip
from ui.sidebar_nav import SidebarNav


PX4_MODES_LIST = [
    "OFFBOARD",
    "POSCTL",
    "ALTCTL",
    "AUTO.LOITER",
    "AUTO.LAND",
    "AUTO.RTL",
    "MANUAL",
    "ACRO",
    "STABILIZED",
]


class DroneGCSMainWindow(QMainWindow):
    """Main application window for industrial-grade Drone-GCS Pilot Station."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone-GCS | Autonomous Industrial Ground Station")
        self.resize(1400, 860)
        # Lowered from 1150x740: that floor didn't actually fit common smaller laptop
        # panels (1366x768 and especially 1280x720) once OS window-chrome/taskbar space
        # is subtracted, forcing clipping instead of a graceful shrink. 1024x700 still
        # comfortably fits every control - it was headroom, not a real usability need.
        self.setMinimumSize(1220, 700)

        # Core Engines
        self.worker: Optional[MAVLinkWorker] = None
        self.exec_tracker = ExecutionTracker(tolerance=0.20, timeout_sec=14.0)
        self.last_telemetry = TelemetrySnapshot()

        # Bench Force Arm tracking state
        self._last_arm_was_forced: bool = False

        # Autonomous Waypoint Navigation & Safety Interlocks
        self.active_waypoints: List[Tuple[float, float]] = []
        self.current_wpt_idx: int = 0
        self.path_in_progress: bool = False
        self.path_paused: bool = False
        self.path_awaiting_climb: bool = False
        self.climb_start_time: float = 0.0
        self.loiter_pause_start_time: float = 0.0
        self._loiter_warned: bool = False
        self.pending_path_waypoints: List[Tuple[float, float]] = []
        self.cruise_z: float = -1.0
        self.current_target_yaw: float = 0.0
        self.takeoff_hover_x: float = 0.0
        self.takeoff_hover_y: float = 0.0
        self.last_collision_check_time: float = 0.0
        self.latest_map_data: Optional[Tuple[np.ndarray, float, float, float]] = None
        self.path_planner = AStarPathPlanner(robot_radius_m=0.25)

        # Continuous 10 Hz OFFBOARD Setpoint Pump (prevents PX4 500ms loss timeout)
        self.offboard_pump_timer = QTimer(self)
        self.offboard_pump_timer.setInterval(100)  # 100ms = 10 Hz
        self.offboard_pump_timer.timeout.connect(self._pump_active_waypoint_setpoint)

        # Build Industrial UI Shell
        self._init_ui()

        # Toast manager
        self.toast = NotificationToast(self)

        # Periodic UI refresh timer (30 Hz)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._on_ui_tick)
        self.ui_timer.start(33)

        # Background ROS 2 Map Listener for live /map_thin (with automatic TCP fallback)
        self.map_listener = ROS2MapListener(tcp_host="172.16.101.84", tcp_port=5765, parent=self)
        self.map_listener.map_received.connect(self._on_map_received_dispatch)
        self.map_listener.status_updated.connect(self._on_ros2_status_updated)
        self.map_listener.start()

        # Auto-connect to default autopilot endpoint on launch (UDP 14550)
        self._connect_to_endpoint("172.16.101.84", 14550, protocol="udp")

    def _init_ui(self):
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        root_layout = QVBoxLayout(main_widget)
        root_layout.setContentsMargins(6, 6, 6, 6)
        root_layout.setSpacing(6)

        # 1. Aerospace Header Status Strip
        self.top_strip = TopStatusStrip(self)
        self.top_strip.connect_requested.connect(self._connect_to_endpoint)
        self.top_strip.disconnect_requested.connect(self._disconnect_from_endpoint)
        self.top_strip.network_changed.connect(self._on_network_selected)
        root_layout.addWidget(self.top_strip)

        # 2. Main Workspace Splitter: SidebarNav (Left) + Stacked Workspaces (Center/Right)
        workspace_box = QHBoxLayout()
        workspace_box.setContentsMargins(0, 0, 0, 0)
        workspace_box.setSpacing(6)

        # Vertical Sidebar
        self.sidebar = SidebarNav(self)
        self.sidebar.view_changed.connect(self._on_view_changed)
        workspace_box.addWidget(self.sidebar)

        # Stacked Pages
        self.stack = QStackedWidget(self)

        # Page 0: Cockpit PFD & Primary Control
        self.page_cockpit = self._build_cockpit_page()
        self.stack.addWidget(self.page_cockpit)

        # Page 1: FPV Camera Feed
        self.page_fpv = VideoFeedWidget(self)
        self.stack.addWidget(self.page_fpv)

        # Page 2: Tactical 2D SLAM & Waypoint Stager
        self.page_slam = SLAMMapWidget(self)
        self.page_slam.execute_path_requested.connect(self._on_execute_path_requested)
        self.page_slam.pause_path_requested.connect(self._on_pause_path_requested)
        self.page_slam.resume_path_requested.connect(self._on_resume_path_requested)
        self.page_slam.abort_path_requested.connect(self._on_abort_path_requested)
        self.page_slam.altitude_changed.connect(self._on_cruise_altitude_changed)
        self.page_slam.reset_map_requested.connect(self._on_reset_map_requested)
        self.stack.addWidget(self.page_slam)
        self._reset_map_worker: Optional[SlamMapResetWorker] = None

        # Page 3: Live Motor & Actuator Telemetry
        self.page_motors = MotorWidget(self)
        self.stack.addWidget(self.page_motors)

        # Page 4: Diagnostics & Bench Safety
        self.page_diagnostics = self._build_diagnostics_page()
        self.stack.addWidget(self.page_diagnostics)

        # Page 5: Flight Terminal & Mission Logs
        self.page_terminal = CLIConsoleWidget(self)
        self.page_terminal.command_submitted.connect(self._execute_cli_command)
        self.stack.addWidget(self.page_terminal)

        workspace_box.addWidget(self.stack, 1)
        root_layout.addLayout(workspace_box, 1)

    def _build_cockpit_page(self) -> QWidget:
        """Page 0: Aviation PFD side-by-side with Proven Control Dispatcher and Closed-Loop Verifier."""
        page = QWidget(self)
        layout = QHBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        splitter = QSplitter(Qt.Horizontal, page)
        splitter.setHandleWidth(4)

        # Left: Aviation Primary Flight Display (PFD)
        self.hud = HUDWidget(self)
        splitter.addWidget(self.hud)

        # Right: Comprehensive Control Request & Output Panel (Matching drone_gcs_gui.py proven workflow)
        right_panel = QWidget(self)
        rl = QVBoxLayout(right_panel)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(8)

        # ---- Card 1: COMMAND DISPATCHER & CONTROL REQUEST ----
        ctrl_card = QFrame(self)
        ctrl_card.setProperty("class", "cardFrame")
        cl = QVBoxLayout(ctrl_card)
        cl.setContentsMargins(10, 8, 10, 8)
        cl.setSpacing(6)

        # Section Header. Arm/Mode status used to be repeated here as its own pair of
        # pills - dropped as a duplicate of the always-visible header badges (the top
        # strip is the single source of truth now, since it's the only one of the three
        # copies that was visible on every tab, not just this one). The HUD's own PFD
        # banner stays - that one reads as a genuine flight instrument, not GUI chrome.
        hdr_box = QHBoxLayout()
        lbl_title = QLabel("COMMAND DISPATCHER (CONTROL REQUEST)", self)
        lbl_title.setStyleSheet("font-weight: bold; color: #d29922; font-size: 11px; letter-spacing: 0.5px;")
        hdr_box.addWidget(lbl_title)
        hdr_box.addStretch()
        cl.addLayout(hdr_box)

        # Row 1: Primary Flight Actions (ARM, DISARM, HOLD, LAND)
        actions_box = QHBoxLayout()
        actions_box.setSpacing(6)

        self.btn_arm = QPushButton("ARM", self)
        self.btn_arm.setObjectName("btnArm")
        self.btn_arm.setMinimumHeight(32)
        self.btn_arm.clicked.connect(self._cmd_arm_from_ui)
        actions_box.addWidget(self.btn_arm, 1)

        self.btn_disarm = QPushButton("DISARM", self)
        self.btn_disarm.setObjectName("btnDisarm")
        self.btn_disarm.setMinimumHeight(32)
        self.btn_disarm.clicked.connect(self._cmd_disarm)
        actions_box.addWidget(self.btn_disarm, 1)

        self.btn_hold = QPushButton("HOLD", self)
        self.btn_hold.setStyleSheet("background-color: #1f6feb; color: #ffffff; font-weight: bold;")
        self.btn_hold.setMinimumHeight(32)
        self.btn_hold.clicked.connect(lambda: self._cmd_mode("AUTO.LOITER"))
        actions_box.addWidget(self.btn_hold, 1)

        self.btn_land = QPushButton("LAND", self)
        self.btn_land.setStyleSheet("background-color: #8957e5; color: #ffffff; font-weight: bold;")
        self.btn_land.setMinimumHeight(32)
        self.btn_land.clicked.connect(lambda: self._cmd_mode("AUTO.LAND"))
        actions_box.addWidget(self.btn_land, 1)

        cl.addLayout(actions_box)

        # Row 2: Bench Force Override Checkbox (Bypass USB/Battery checks via param2=21196)
        self.chk_force_arm = QCheckBox(
            "Bench Force Override (Bypass USB/Battery Safety Locks via param2=21196)", self
        )
        self.chk_force_arm.setChecked(False)
        self.chk_force_arm.setStyleSheet("color: #d29922; font-weight: 600; font-size: 11px;")
        cl.addWidget(self.chk_force_arm)

        # Row 3: Takeoff Control (Altitude numeric box + Button)
        takeoff_box = QHBoxLayout()
        takeoff_box.setSpacing(6)
        lbl_to = QLabel("Takeoff Alt (m):", self)
        lbl_to.setStyleSheet("color: #c9d1d9; font-weight: 600; font-size: 11px;")
        takeoff_box.addWidget(lbl_to)

        self.ent_takeoff_alt = QLineEdit("1.0", self)
        self.ent_takeoff_alt.setFixedWidth(55)
        self.ent_takeoff_alt.setAlignment(Qt.AlignCenter)
        self.ent_takeoff_alt.setValidator(QDoubleValidator(0.2, 10.0, 2, self))
        takeoff_box.addWidget(self.ent_takeoff_alt)

        self.btn_takeoff = QPushButton("TAKEOFF", self)
        self.btn_takeoff.setStyleSheet("background-color: #1f6feb; color: #ffffff; font-weight: bold;")
        self.btn_takeoff.setMinimumHeight(30)
        self.btn_takeoff.clicked.connect(self._cmd_takeoff_from_ui)
        takeoff_box.addWidget(self.btn_takeoff)

        takeoff_box.addStretch()
        cl.addLayout(takeoff_box)

        # Row 4: Relative Translation Group (dx, dy, dz)
        move_box = QHBoxLayout()
        move_box.setSpacing(4)
        lbl_mv = QLabel("Move (m):", self)
        lbl_mv.setStyleSheet("color: #c9d1d9; font-weight: 600; font-size: 11px;")
        move_box.addWidget(lbl_mv)

        lbl_dx = QLabel("dx:", self)
        lbl_dx.setStyleSheet("color: #8b949e; font-size: 11px;")
        move_box.addWidget(lbl_dx)
        self.ent_dx = QLineEdit("0.5", self)
        self.ent_dx.setFixedWidth(46)
        self.ent_dx.setAlignment(Qt.AlignCenter)
        self.ent_dx.setValidator(QDoubleValidator(-10.0, 10.0, 2, self))
        move_box.addWidget(self.ent_dx)

        lbl_dy = QLabel("dy:", self)
        lbl_dy.setStyleSheet("color: #8b949e; font-size: 11px;")
        move_box.addWidget(lbl_dy)
        self.ent_dy = QLineEdit("0.0", self)
        self.ent_dy.setFixedWidth(46)
        self.ent_dy.setAlignment(Qt.AlignCenter)
        self.ent_dy.setValidator(QDoubleValidator(-10.0, 10.0, 2, self))
        move_box.addWidget(self.ent_dy)

        lbl_dz = QLabel("dz:", self)
        lbl_dz.setStyleSheet("color: #8b949e; font-size: 11px;")
        move_box.addWidget(lbl_dz)
        self.ent_dz = QLineEdit("0.0", self)
        self.ent_dz.setFixedWidth(46)
        self.ent_dz.setAlignment(Qt.AlignCenter)
        self.ent_dz.setValidator(QDoubleValidator(-10.0, 10.0, 2, self))
        move_box.addWidget(self.ent_dz)

        self.btn_move = QPushButton("MOVE", self)
        self.btn_move.setStyleSheet("background-color: #1f6feb; color: #ffffff; font-weight: bold;")
        self.btn_move.setMinimumHeight(30)
        self.btn_move.clicked.connect(self._cmd_move_from_ui)
        move_box.addWidget(self.btn_move)

        cl.addLayout(move_box)

        # Row 5: Heading Control (Yaw) & Flight Mode Dropdown Selector
        row5 = QHBoxLayout()
        row5.setSpacing(6)

        # Yaw
        lbl_yaw = QLabel("Yaw (°):", self)
        lbl_yaw.setStyleSheet("color: #c9d1d9; font-weight: 600; font-size: 11px;")
        row5.addWidget(lbl_yaw)

        self.ent_yaw = QLineEdit("90.0", self)
        self.ent_yaw.setFixedWidth(50)
        self.ent_yaw.setAlignment(Qt.AlignCenter)
        self.ent_yaw.setValidator(QDoubleValidator(-360.0, 360.0, 1, self))
        row5.addWidget(self.ent_yaw)

        self.btn_yaw = QPushButton("ROTATE YAW", self)
        self.btn_yaw.setStyleSheet("background-color: #1f6feb; color: #ffffff; font-weight: bold;")
        self.btn_yaw.setMinimumHeight(30)
        self.btn_yaw.clicked.connect(self._cmd_yaw_from_ui)
        row5.addWidget(self.btn_yaw)

        row5.addSpacing(8)

        # Mode Dropdown
        lbl_mode = QLabel("Mode:", self)
        lbl_mode.setStyleSheet("color: #c9d1d9; font-weight: 600; font-size: 11px;")
        row5.addWidget(lbl_mode)

        self.combo_modes = QComboBox(self)
        self.combo_modes.addItems(PX4_MODES_LIST)
        self.combo_modes.setCurrentText("OFFBOARD")
        self.combo_modes.setMinimumHeight(30)
        row5.addWidget(self.combo_modes, 1)

        self.btn_set_mode = QPushButton("SET MODE", self)
        self.btn_set_mode.setStyleSheet("background-color: #238636; color: #ffffff; font-weight: bold;")
        self.btn_set_mode.setMinimumHeight(30)
        self.btn_set_mode.clicked.connect(self._cmd_set_mode_from_ui)
        row5.addWidget(self.btn_set_mode)

        cl.addLayout(row5)

        # Emergency Cutoff
        self.btn_kill = QPushButton("⛔ EMERGENCY MOTOR KILL", self)
        self.btn_kill.setObjectName("btnKill")
        self.btn_kill.setMinimumHeight(32)
        self.btn_kill.clicked.connect(self._cmd_kill)
        cl.addWidget(self.btn_kill)

        rl.addWidget(ctrl_card)

        # ---- Card 2: CLOSED-LOOP PHYSICAL EXECUTION VERIFIER ----
        verif_card = QFrame(self)
        verif_card.setProperty("class", "cardFrame")
        vl = QVBoxLayout(verif_card)
        vl.setContentsMargins(10, 8, 10, 8)
        vl.setSpacing(4)

        vl_title = QLabel("CLOSED-LOOP PHYSICAL EXECUTION VERIFIER")
        vl_title.setStyleSheet("font-weight: bold; color: #8957e5; font-size: 11px; letter-spacing: 0.5px;")
        vl.addWidget(vl_title)

        self.lbl_exec_val = QLabel("IDLE (holding position)", self)
        self.lbl_exec_val.setStyleSheet("color: #f0f6fc; font-size: 12px; font-weight: bold;")
        vl.addWidget(self.lbl_exec_val)

        self.lbl_exec_details = QLabel(
            "Displacement: dx=+0.00m  dy=+0.00m  dz=+0.00m  |  Speed: 0.00 m/s", self
        )
        self.lbl_exec_details.setStyleSheet("color: #8b949e; font-size: 11px; font-family: monospace;")
        vl.addWidget(self.lbl_exec_details)

        self.progress_verif = QProgressBar(self)
        self.progress_verif.setFixedHeight(12)
        self.progress_verif.setTextVisible(False)
        self.progress_verif.setValue(0)
        vl.addWidget(self.progress_verif)

        rl.addWidget(verif_card)

        # ---- Card 3: Interactive Terminal / Console Bar ----
        self.console = CLIConsoleWidget(self)
        self.console.command_submitted.connect(self._execute_cli_command)
        rl.addWidget(self.console, 1)

        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        layout.addWidget(splitter)
        return page

    def _build_diagnostics_page(self) -> QWidget:
        """Page 4: Detailed Diagnostics & Bench Safety view."""
        page = QWidget(self)
        layout = QVBoxLayout(page)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        title = QLabel("SYSTEM DIAGNOSTICS & INTEL REALSENSE D435i VIO SENSOR HEALTH")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #58a6ff;")
        layout.addWidget(title)

        self.diag_text = QLabel("Connecting to telemetry stream...", self)
        self.diag_text.setFont(QFont("Consolas", 11))
        self.diag_text.setStyleSheet("color: #c9d1d9;")
        layout.addWidget(self.diag_text)
        layout.addStretch()
        return page

    def _on_view_changed(self, idx: int):
        self.stack.setCurrentIndex(idx)
        if self.stack.widget(idx) is self.page_fpv:
            # Auto-connect to the drone's MJPEG stream the first time this tab is opened.
            # Idempotent - does nothing if a feed is already running.
            self.page_fpv.ensure_started()

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------

    def _on_network_selected(self, ip: str):
        """Fired the moment a Network preset is picked in the top strip - independent of
        Connect/Disconnect, so the map bridge and FPV stream re-target immediately rather
        than waiting for a MAVLink (re)connect. Port/protocol are untouched: they're a
        property of the link, not of which Wi-Fi network is active."""
        if hasattr(self, 'map_listener') and self.map_listener:
            self.map_listener.set_tcp_host(ip)
        if hasattr(self, 'page_fpv') and self.page_fpv:
            self.page_fpv.set_stream_host(ip)

    def _connect_to_endpoint(self, ip: str, port: int, protocol: str = "udp"):
        if self.worker and self.worker.isRunning():
            self._disconnect_from_endpoint()

        proto_str = protocol.lower().strip()
        prefix = "udpout:" if proto_str == "udp" else "tcp:"
        self.console.log_info(f"Connecting to {prefix}{ip}:{port} (SysID 255)...")
        self.page_terminal.log_info(f"Connecting to {prefix}{ip}:{port} (SysID 255)...")

        if hasattr(self, 'map_listener') and self.map_listener:
            self.map_listener.set_tcp_host(ip)

        self.worker = MAVLinkWorker(host=ip, port=port, protocol=proto_str, source_system=255)
        self.worker.telemetry_updated.connect(self._on_telemetry_updated)
        self.worker.connection_changed.connect(self._on_connection_changed)
        self.worker.statustext_received.connect(self._on_statustext_received)
        self.worker.command_ack_received.connect(self._on_command_ack_received)
        self.worker.rates_updated.connect(self._on_rates_updated)
        self.worker.start()

    def _disconnect_from_endpoint(self):
        if self.worker and self.worker.isRunning():
            self.console.log_warning("Disconnecting from SBC...")
            self.page_terminal.log_warning("Disconnecting from SBC...")
            self.worker.disconnect_endpoint()
            self.worker = None
            self.top_strip.set_connection_state(False, "Disconnected")
            self.toast.show_message("Disconnected from Drone", "#da3633")

    def _on_connection_changed(self, connected: bool, message: str):
        self.top_strip.set_connection_state(connected, message)
        if connected:
            self.console.log_success(f"MAVLink Link: {message}")
            self.page_terminal.log_success(f"MAVLink Link: {message}")
            self.toast.show_message(message, "#238636")
        else:
            self.console.log_warning(f"Link status: {message}")
            self.page_terminal.log_warning(f"Link status: {message}")

    def _on_rates_updated(self, rx_rate: float, tx_rate: float):
        self.top_strip.update_rates(rx_rate, tx_rate)

    def _on_statustext_received(self, text: str, severity: int):
        if text.startswith("[GCS CMD]"):
            return  # Filter out own echoes
        clean_text = text.strip().replace("\t", " ")
        if not clean_text:
            return

        is_preflight = "preflight fail" in clean_text.lower() or "arming denied" in clean_text.lower()
        if is_preflight or severity <= 3:
            log_msg = f"🚨 PX4 REJECTION: {clean_text}" if is_preflight else f"PX4 ALERT: {clean_text}"
            self.console.log_error(log_msg)
            self.page_terminal.log_error(log_msg)
            self.toast.show_message(clean_text, "#da3633", 6000)
        elif severity <= 5:
            self.console.log_warning(f"PX4 NOTICE: {clean_text}")
            self.page_terminal.log_warning(f"PX4 NOTICE: {clean_text}")
        else:
            self.console.log_info(f"PX4: {clean_text}")
            self.page_terminal.log_info(f"PX4: {clean_text}")

    def _on_command_ack_received(self, cmd_id: int, result_code: int, cmd_name: str, result_str: str):
        """Real-time feedback when Pixhawk acknowledges or rejects a command."""
        # Immediately notify execution tracker so countdown timers are aborted and status is updated
        self.exec_tracker.notify_command_ack(cmd_id, result_code, cmd_name, result_str)
        self.lbl_exec_val.setText(self.exec_tracker.status_msg)
        self.progress_verif.setValue(int(self.exec_tracker.progress_pct))

        if result_code == 0:  # ACCEPTED
            msg = f"[ACCEPTED] PX4 ACK: {cmd_name} ACCEPTED"
            self.console.log_success(msg)
            self.page_terminal.log_success(msg)
            self.toast.show_message(msg, "#238636", 3000)
        elif result_code == 1:  # TEMPORARILY_REJECTED
            msg = f"[REJECTED] PX4 ACK: {cmd_name} TEMPORARILY REJECTED (Preflight check or safety failure blocked command)"
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message(f"[REJECTED] {cmd_name} by PX4", "#da3633", 5000)
        elif result_code == 2:  # DENIED
            msg = f"[DENIED] PX4 ACK: {cmd_name} DENIED (Operation not permitted by Pixhawk in current mode/state)"
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message(f"[DENIED] {cmd_name} by PX4", "#da3633", 5000)
        else:
            msg = f"[REJECTED] PX4 ACK: {cmd_name} {result_str} (Code {result_code})"
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message(msg, "#da3633", 4000)

    def _on_ros2_status_updated(self, msg: str):
        self.console.log_info(f"ROS 2: {msg}")
        self.page_terminal.log_info(f"ROS 2: {msg}")

    def _on_map_received_dispatch(self, grid: np.ndarray, res: float, ox: float, oy: float, source: str):
        self.latest_map_data = (grid, res, ox, oy)
        self.page_slam.on_ros2_map_received(grid, res, ox, oy, source)

    def _on_telemetry_updated(self, t: TelemetrySnapshot):
        self.last_telemetry = t
        self.hud.update_telemetry(t)
        self.top_strip.update_telemetry(t)
        self.page_slam.update_pose(t.x, t.y, t.heading)
        self.page_slam.set_armed_state(t.armed)
        self.page_motors.update_pwms(t.motor_pwms)

    # -------------------------------------------------------------------------
    # UI Periodic Tick (30 Hz)
    # -------------------------------------------------------------------------

    def _on_ui_tick(self):
        t = self.last_telemetry
        t.check_vision_staleness(max_age_sec=3.0)
        self.top_strip.update_telemetry(t)

        # Arm/Mode status: shown in the header badges (top_strip.update_telemetry above)
        # and the HUD's own PFD banner - no longer duplicated here.

        # 3. Update Closed-Loop Execution Tracker
        if self.exec_tracker.active:
            res = self.exec_tracker.update(t.x, t.y, t.z, armed=t.armed, flight_mode=t.flight_mode, cur_heading=t.heading)
            pct = int(res["progress_pct"])
            self.progress_verif.setValue(pct)
            self.lbl_exec_val.setText(res["status_msg"])

            dx = t.x - self.exec_tracker.x0
            dy = t.y - self.exec_tracker.y0
            dz = t.z - self.exec_tracker.z0
            self.lbl_exec_details.setText(
                f"Displacement: dx={dx:+.2f}m dy={dy:+.2f}m dz={dz:+.2f}m | Speed: {t.ground_speed:.2f} m/s"
            )

            if res["status"] == "EXECUTED":
                self.console.log_success(res["status_msg"])
                self.page_terminal.log_success(res["status_msg"])
                self.toast.show_message(res["status_msg"], "#238636")
            elif res["status"] in ("STALLED", "REJECTED"):
                self.console.log_error(res["status_msg"])
                self.page_terminal.log_error(res["status_msg"])
                self.toast.show_message(res["status_msg"], "#da3633")

        # 3. Pre-Flight Auto-Climb Interlock for Ground Start (Decoupled threshold & 15s timeout)
        now = time.time()
        if self.path_awaiting_climb and self.worker:
            climb_threshold = max(0.5, abs(self.cruise_z) * 0.80)
            if t.altitude >= climb_threshold:
                self.path_awaiting_climb = False
                self.climb_start_time = 0.0
                self.console.log_success(
                    f"[OK] Safe cruise altitude achieved ({t.altitude:.2f}m >= {climb_threshold:.2f}m). Starting flight path..."
                )
                self._dispatch_path_start(self.pending_path_waypoints)
            else:
                if self.climb_start_time == 0.0:
                    self.climb_start_time = now
                elif (now - self.climb_start_time) > 15.0:
                    # Climb timed out! Failsafe transition to AUTO.LOITER
                    self.path_awaiting_climb = False
                    self.climb_start_time = 0.0
                    self.path_in_progress = False
                    self.offboard_pump_timer.stop()
                    self.worker.set_mode("AUTO.LOITER")
                    self.page_slam.set_executing_state(False, paused=False)
                    self.console.log_error(
                        f"❌ CLIMB TIMEOUT: Drone reached only {t.altitude:.2f}m / target {abs(self.cruise_z):.2f}m after 15s. "
                        "Halting in AUTO.LOITER."
                    )
                    self.page_terminal.log_error(f"❌ CLIMB TIMEOUT: Reached only {t.altitude:.2f}m. Halting in AUTO.LOITER.")
                    self.toast.show_message("Climb Timeout! Halting in LOITER", "#da3633", 6000)
                else:
                    # Hold vertical climb setpoint directly over initial position
                    self.worker.move_to_waypoint(
                        self.takeoff_hover_x, self.takeoff_hover_y, z=self.cruise_z, yaw_deg=t.heading
                    )

        # 4. Loiter Hold Watchdog (Warn pilot if holding paused path for >45 seconds)
        if self.path_paused and not self._loiter_warned:
            if (now - self.loiter_pause_start_time) > 45.0:
                self._loiter_warned = True
                self.console.log_warning("⚠️ LOITER WATCHDOG: Drone has been holding in AUTO.LOITER for 45s! Command RESUME or LAND.")
                self.toast.show_message("Holding in LOITER for 45s! Action needed", "#d29922", 6000)

        # 5. Live Dynamic Collision Avoidance on Active Flight Path
        if (
            self.path_in_progress
            and self.active_waypoints
            and self.latest_map_data is not None
            and (now - self.last_collision_check_time > 0.20)
        ):
            self.last_collision_check_time = now
            grid, res, ox, oy = self.latest_map_data
            remaining_wpts = self.active_waypoints[self.current_wpt_idx:]
            is_blocked, col_pt, col_dist = self.path_planner.check_path_collision(
                grid, res, ox, oy, remaining_wpts, (t.x, t.y), lookahead_m=1.5
            )
            if is_blocked:
                # Obstacle detected directly in path!
                self.worker.set_mode("AUTO.LOITER")
                self.path_in_progress = False
                self.path_paused = True
                self.loiter_pause_start_time = now
                self._loiter_warned = False
                self.page_slam.set_executing_state(False, paused=True)

                self.console.log_error(
                    f"🚨 COLLISION ALERT: Obstacle detected {col_dist:.2f}m ahead on flight path! Halting in AUTO.LOITER."
                )
                self.page_terminal.log_error(f"🚨 COLLISION ALERT: Obstacle {col_dist:.2f}m ahead! Switched to AUTO.LOITER.")
                self.toast.show_message(f"Obstacle Ahead ({col_dist:.2f}m)! Drone Halted", "#da3633", 6000)

                # Attempt automatic detour replanning to final goal
                final_goal = self.active_waypoints[-1]
                detour = self.path_planner.plan(grid, res, ox, oy, (t.x, t.y), final_goal)
                if detour["success"]:
                    self.console.log_success(
                        f"🔄 DETOUR READY: Clear path found ({len(detour['waypoints'])} WPTs, {detour['total_distance_m']:.2f}m). "
                        "Click '▶ RESUME PATH' on SLAM screen to fly detour."
                    )
                    self.page_slam.canvas.planned_waypoints = detour["waypoints"]
                    self.page_slam.canvas.update()
                    self.active_waypoints = list(detour["waypoints"])
                    self.current_wpt_idx = 0
                else:
                    self.console.log_warning("No clear detour found. Maintain hold or command manual RTL / LAND.")

        # 6. Sequential Waypoint Navigation along Planned A* Path with Tangent Yaw
        if self.path_in_progress and self.active_waypoints and self.worker:
            curr_target = self.active_waypoints[self.current_wpt_idx]
            dist_to_wpt = math.sqrt((curr_target[0] - t.x)**2 + (curr_target[1] - t.y)**2)

            if dist_to_wpt < 0.25:
                # Reached waypoint
                self.console.log_success(
                    f"Reached Waypoint W{self.current_wpt_idx+1} ({curr_target[0]:+.2f}, {curr_target[1]:+.2f})"
                )
                self.current_wpt_idx += 1
                if self.current_wpt_idx < len(self.active_waypoints):
                    next_wpt = self.active_waypoints[self.current_wpt_idx]
                    dx = next_wpt[0] - t.x
                    dy = next_wpt[1] - t.y
                    seg_dist = math.sqrt(dx * dx + dy * dy)
                    # Tangent heading: nose & camera point forward along flight segment
                    yaw_deg = math.degrees(math.atan2(dy, dx)) if seg_dist > 0.05 else t.heading
                    self.current_target_yaw = yaw_deg

                    self.console.log_cmd(
                        f"Advancing to W{self.current_wpt_idx+1}: ({next_wpt[0]:+.2f}, {next_wpt[1]:+.2f}) "
                        f"alt={-self.cruise_z:.2f}m yaw={yaw_deg:+.1f}°"
                    )
                    self.worker.move_to_waypoint(next_wpt[0], next_wpt[1], z=self.cruise_z, yaw_deg=yaw_deg)
                    self.exec_tracker.start_tracking(
                        f"waypoint W{self.current_wpt_idx+1}", t.x, t.y, t.z, target_dist=seg_dist
                    )
                else:
                    self.path_in_progress = False
                    self.path_paused = False
                    self.offboard_pump_timer.stop()
                    self.page_slam.set_executing_state(False, paused=False)
                    self.console.log_success("MISSION COMPLETE: Drone arrived at final Goal Pose!")
                    self.page_terminal.log_success("MISSION COMPLETE: Drone arrived at final Goal Pose!")
                    self.toast.show_message("Goal Pose Reached!", "#238636", 5000)

        # 5. Update Diagnostics Tab
        vio_str = "LOCKED (Streaming 10 Hz)" if t.d435i_vio_health else "NO VISION DATA"
        self.diag_text.setText(
            f"=== PX4 AUTOPILOT STATUS ===\n"
            f"SysID: {t.system_id} | CompID: {t.component_id} | Connected: {t.connected}\n"
            f"Arm State: {'ARMED' if t.armed else 'DISARMED'} | Flight Mode: {t.flight_mode}\n"
            f"Flight Time: {int(t.flight_time_sec)}s\n\n"
            f"=== INTEL REALSENSE D435i VIO STATUS ===\n"
            f"VIO Stream Health: {vio_str}\n"
            f"EKF2 Vision Fusion: {'ACTIVE (Indoor Position Locked)' if t.ekf2_vision_fused else 'WAITING'}\n\n"
            f"=== LOCAL POSITION NED (VIO AGL) ===\n"
            f"North (x): {t.x:+.3f} m\n"
            f"East  (y): {t.y:+.3f} m\n"
            f"Down  (z): {t.z:+.3f} m  (Altitude AGL: {t.altitude:.3f} m)\n\n"
            f"=== BODY VELOCITIES & SPEED ===\n"
            f"Vx: {t.vx:+.2f} m/s | Vy: {t.vy:+.2f} m/s | Vz: {t.vz:+.2f} m/s\n"
            f"Ground Speed: {t.ground_speed:.2f} m/s\n\n"
            f"=== MOTOR ACTUATOR OUTPUTS (SERVO_OUTPUT_RAW) ===\n"
            f"M1 (FR CCW): {t.motor_pwms[0]} µs | M2 (RL CCW): {t.motor_pwms[1]} µs\n"
            f"M3 (FL CW):  {t.motor_pwms[2]} µs | M4 (RR CW):  {t.motor_pwms[3]} µs\n\n"
            f"=== ATTITUDE & HEADING ===\n"
            f"Roll: {t.roll:+.1f}° | Pitch: {t.pitch:+.1f}° | Yaw: {t.yaw:+.1f}° | Heading: {t.heading:.1f}°\n\n"
            f"=== POWER & GPS ===\n"
            f"Battery: {t.battery_voltage:.2f} V | Current: {t.battery_current:.1f} A | Remaining: {t.battery_percent}%\n"
            f"GPS Satellites: {t.satellites} | HDOP: {t.hdop:.1f} | Fix: {t.fix_type}\n"
        )

    # -------------------------------------------------------------------------
    # Command Dispatch Helpers
    # -------------------------------------------------------------------------

    def _cmd_arm_from_ui(self):
        force = self.chk_force_arm.isChecked()
        self._cmd_arm(force=force)

    def _cmd_arm(self, force: bool = False):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot arm: Not connected to vehicle")
            self.page_terminal.log_error("Cannot arm: Not connected to vehicle")
            return
        self._last_arm_was_forced = force
        label = "FORCE ARM (BENCH, param2=21196)" if force else "ARM (NORMAL)"
        self.console.log_cmd(f"Dispatching {label}...")
        self.page_terminal.log_cmd(f"Dispatching {label}...")
        self.worker.arm(force=force)
        self.toast.show_message(f"Dispatching: {label}", "#d29922" if force else "#238636")
        self.exec_tracker.start_tracking(
            "arm force" if force else "arm",
            self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode
        )

    def _cmd_disarm(self):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot disarm: Not connected to vehicle")
            self.page_terminal.log_error("Cannot disarm: Not connected to vehicle")
            return

        # Completely reset active navigation and waypoint pump
        self.offboard_pump_timer.stop()
        self.path_in_progress = False
        self.path_paused = False
        self.path_awaiting_climb = False
        self.climb_start_time = 0.0
        self.active_waypoints = []
        self.current_wpt_idx = 0
        self.page_slam.set_executing_state(False, paused=False)
        self.page_slam.canvas.clear_goal()

        self.console.log_cmd("Dispatching DISARM...")
        self.page_terminal.log_cmd("Dispatching DISARM...")
        self.worker.disarm(force=True)
        self.toast.show_message("Dispatching: DISARM", "#da3633")
        self.exec_tracker.start_tracking(
            "disarm", self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode
        )

    def _cmd_takeoff_from_ui(self):
        try:
            alt = float(self.ent_takeoff_alt.text().strip())
        except ValueError:
            alt = 1.0
        self._cmd_takeoff(alt)

    def _cmd_takeoff(self, altitude: float = 1.0):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot takeoff: Not connected")
            self.page_terminal.log_error("Cannot takeoff: Not connected")
            return
        self.console.log_cmd(f"Initiating takeoff to {altitude:.1f}m...")
        self.page_terminal.log_cmd(f"Initiating takeoff to {altitude:.1f}m...")
        self.worker.takeoff(altitude)
        self.toast.show_message(f"Takeoff Initiated ({altitude:.1f}m)", "#1f6feb")
        self.exec_tracker.start_tracking(
            f"takeoff {altitude}",
            self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode
        )

    def _cmd_move_from_ui(self):
        try:
            dx = float(self.ent_dx.text().strip())
            dy = float(self.ent_dy.text().strip())
            dz = float(self.ent_dz.text().strip())
        except ValueError:
            self.console.log_error("Coordinates must be valid numbers")
            self.page_terminal.log_error("Coordinates must be valid numbers")
            return
        self._cmd_move(dx, dy, dz)

    def _cmd_move(self, dx: float, dy: float, dz: float):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot move: Disconnected")
            self.page_terminal.log_error("Cannot move: Disconnected")
            return

        # Ensure OFFBOARD mode before dispatching setpoints, alerting the operator
        if self.last_telemetry.flight_mode != "OFFBOARD":
            curr_mode = self.last_telemetry.flight_mode or "UNKNOWN"
            self.console.log_warning(
                f"[MODE] Current mode is {curr_mode}. Switching to OFFBOARD to execute move..."
            )
            self.page_terminal.log_warning(
                f"[MODE] Current mode is {curr_mode}. Switching to OFFBOARD to execute move..."
            )
            self.toast.show_message("Switching to OFFBOARD to execute move", "#d29922", 2500)
            self.worker.set_mode("OFFBOARD")

        self.console.log_cmd(f"Dispatching translation: dx={dx:+.2f}m, dy={dy:+.2f}m, dz={dz:+.2f}m")
        self.page_terminal.log_cmd(f"Dispatching translation: dx={dx:+.2f}m, dy={dy:+.2f}m, dz={dz:+.2f}m")
        self.worker.move_delta(dx, dy, dz)
        self.toast.show_message(f"Move: ({dx:+.1f}, {dy:+.1f}, {dz:+.1f})m", "#1f6feb")
        self.exec_tracker.start_tracking(
            f"move {dx:.2f} {dy:.2f} {dz:.2f}",
            self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode
        )

    def _cmd_yaw_from_ui(self):
        try:
            angle = float(self.ent_yaw.text().strip())
        except ValueError:
            angle = 90.0
        self._cmd_yaw(angle)

    def _cmd_yaw(self, angle_deg: float):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot rotate yaw: Disconnected")
            self.page_terminal.log_error("Cannot rotate yaw: Disconnected")
            return

        # Ensure OFFBOARD mode before dispatching yaw setpoint, alerting the operator
        if self.last_telemetry.flight_mode != "OFFBOARD":
            curr_mode = self.last_telemetry.flight_mode or "UNKNOWN"
            self.console.log_warning(
                f"[MODE] Current mode is {curr_mode}. Switching to OFFBOARD to rotate yaw..."
            )
            self.page_terminal.log_warning(
                f"[MODE] Current mode is {curr_mode}. Switching to OFFBOARD to rotate yaw..."
            )
            self.toast.show_message("Switching to OFFBOARD to rotate yaw", "#d29922", 2500)
            self.worker.set_mode("OFFBOARD")

        self.console.log_cmd(f"Rotating yaw by {angle_deg:+.1f}°...")
        self.page_terminal.log_cmd(f"Rotating yaw by {angle_deg:+.1f}°...")
        self.worker.rotate_yaw(angle_deg)
        self.toast.show_message(f"Yaw Rotate: {angle_deg:+.1f}°", "#1f6feb")
        self.exec_tracker.start_tracking(
            f"yaw {angle_deg:.1f}",
            self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode,
            cur_heading=self.last_telemetry.heading
        )

    def _cmd_set_mode_from_ui(self):
        selected_mode = self.combo_modes.currentText().strip()
        self._cmd_mode(selected_mode)

    def _cmd_mode(self, mode_name: str):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error(f"Cannot set mode {mode_name}: Not connected")
            self.page_terminal.log_error(f"Cannot set mode {mode_name}: Not connected")
            return
        self.console.log_cmd(f"Switching mode to {mode_name}...")
        self.page_terminal.log_cmd(f"Switching mode to {mode_name}...")
        self.worker.set_mode(mode_name)
        self.toast.show_message(f"Mode set: {mode_name}", "#1f6feb")
        if "LAND" in mode_name.upper():
            self.exec_tracker.start_tracking(
                "land", self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
                cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode
            )
        elif "RTL" in mode_name.upper():
            self.exec_tracker.start_tracking(
                "rtl", self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
                cur_armed=self.last_telemetry.armed, cur_mode=self.last_telemetry.flight_mode
            )

    def _cmd_kill(self):
        reply = QMessageBox.critical(
            self,
            "EMERGENCY MOTOR KILL",
            "WARNING: This will immediately cut all motor outputs!\nAre you sure you want to terminate flight?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            # Completely reset active navigation and waypoint pump
            self.offboard_pump_timer.stop()
            self.path_in_progress = False
            self.path_paused = False
            self.path_awaiting_climb = False
            self.climb_start_time = 0.0
            self.active_waypoints = []
            self.current_wpt_idx = 0
            self.page_slam.set_executing_state(False, paused=False)
            self.page_slam.canvas.clear_goal()

            if self.worker and self.worker.isRunning():
                self.worker.emergency_kill()
                self.console.log_error("!!! EMERGENCY MOTOR KILL EXECUTED !!!")
                self.page_terminal.log_error("!!! EMERGENCY MOTOR KILL EXECUTED !!!")
                self.toast.show_message("EMERGENCY KILL SENT", "#da3633", 5000)

    # -------------------------------------------------------------------------
    # A* Path Planning Execution & Flight Safety Handlers
    # -------------------------------------------------------------------------

    def _on_execute_path_requested(self, waypoints: list):
        """Called when user clicks '[ EXECUTE PATH ]' in Tactical SLAM tab."""
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot execute path: Not connected to drone")
            self.toast.show_message("Cannot execute: Disconnected", "#da3633")
            return

        if not waypoints:
            self.console.log_error("Cannot execute path: Waypoint list is empty")
            return

        # 1. Arm Status Interlock
        if not self.last_telemetry.armed:
            self.console.log_error(
                "❌ FLIGHT INTERLOCK REJECTED: Drone is DISARMED! "
                "Arm drone and confirm clear airspace before executing autonomous path."
            )
            self.page_terminal.log_error(
                "❌ FLIGHT INTERLOCK REJECTED: Drone is DISARMED! Arm drone before executing path."
            )
            self.toast.show_message("Cannot Execute: Drone is Disarmed!", "#da3633", 5000)
            return

        # 2. VIO / Position Lock Interlock
        if not self.last_telemetry.d435i_vio_health and not self.last_telemetry.ekf2_vision_fused:
            self.console.log_warning(
                "⚠️ FLIGHT WARNING: D435i Visual Odometry is not confirmed locked. "
                "Proceed with extreme caution in GPS-denied environment."
            )
            self.toast.show_message("Warning: Vision Odometry Degraded", "#d29922", 4000)

        # 3. Ground Takeoff vs In-Air Transition Check
        selected_alt = self.page_slam.get_cruise_altitude()
        self.cruise_z = -abs(selected_alt)

        current_alt = self.last_telemetry.altitude
        if current_alt < 0.40:
            self.console.log_cmd(
                f"[TAKEOFF INTERLOCK] Vehicle is on ground (alt={current_alt:.2f}m). "
                f"Holding climb to cruise altitude {abs(self.cruise_z):.2f}m..."
            )
            self.page_terminal.log_cmd(
                f"[TAKEOFF INTERLOCK] Vehicle on ground. Initiating climb to {abs(self.cruise_z):.2f}m..."
            )
            self.pending_path_waypoints = list(waypoints)
            self.path_awaiting_climb = True
            self.climb_start_time = time.time()
            self.takeoff_hover_x = self.last_telemetry.x
            self.takeoff_hover_y = self.last_telemetry.y
            self.path_in_progress = True
            self.page_slam.set_executing_state(True, paused=False)

            self.worker.set_mode("OFFBOARD")
            self.worker.move_to_waypoint(
                self.takeoff_hover_x, self.takeoff_hover_y, z=self.cruise_z, yaw_deg=self.last_telemetry.heading
            )
            self.offboard_pump_timer.start()
            self.toast.show_message("Climbing to Cruise Alt...", "#1f6feb", 4000)
            return

        # Already airborne: proceed immediately
        self.console.log_cmd(
            f"[AIRBORNE] Executing path at cruise altitude {abs(self.cruise_z):.2f}m AGL ({len(waypoints)} waypoints)..."
        )
        self.page_terminal.log_cmd(
            f"[AIRBORNE] Executing path ({len(waypoints)} waypoints)..."
        )
        self.worker.set_mode("OFFBOARD")
        self._dispatch_path_start(waypoints)

    def _dispatch_path_start(self, waypoints: list):
        """Dispatches the first waypoint with tangent yaw alignment."""
        self.active_waypoints = list(waypoints)
        self.current_wpt_idx = 0
        self.path_in_progress = True
        self.path_paused = False
        self.page_slam.set_executing_state(True, paused=False)

        # Dispatch first waypoint with tangent heading
        first_wpt = self.active_waypoints[0]
        dx = first_wpt[0] - self.last_telemetry.x
        dy = first_wpt[1] - self.last_telemetry.y
        seg_dist = math.sqrt(dx * dx + dy * dy)
        yaw_deg = math.degrees(math.atan2(dy, dx)) if seg_dist > 0.05 else self.last_telemetry.heading
        self.current_target_yaw = yaw_deg

        self.worker.move_to_waypoint(first_wpt[0], first_wpt[1], z=self.cruise_z, yaw_deg=yaw_deg)
        self.offboard_pump_timer.start()  # Start 10 Hz continuous stream

        self.exec_tracker.start_tracking(
            f"waypoint W1", self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            target_dist=seg_dist
        )
        self.console.log_cmd(
            f"Dispatching W1: ({first_wpt[0]:+.2f}m, {first_wpt[1]:+.2f}m) alt={-self.cruise_z:.2f}m yaw={yaw_deg:+.1f}°"
        )

    def _pump_active_waypoint_setpoint(self):
        """Continuously stream active waypoint setpoint at 10 Hz to satisfy PX4 500ms timeout."""
        if (
            self.path_in_progress
            and not self.path_paused
            and self.worker
            and self.worker.isRunning()
            and self.active_waypoints
            and self.current_wpt_idx < len(self.active_waypoints)
        ):
            wpt = self.active_waypoints[self.current_wpt_idx]
            self.worker.move_to_waypoint(wpt[0], wpt[1], z=self.cruise_z, yaw_deg=self.current_target_yaw)

    def _on_cruise_altitude_changed(self, alt_m: float):
        self.cruise_z = -abs(alt_m)
        self.console.log_info(f"[TACTICAL SLAM] Cruise altitude updated to {abs(alt_m):.1f}m AGL (z={self.cruise_z:.1f}m)")

    def _on_reset_map_requested(self):
        """User confirmed 'Reset Map' in the Tactical SLAM tab. Runs the reset on a
        background worker (ROS 2 service calls, TCP fallback) so the GUI stays
        responsive - this can take a few seconds if it has to fall through to the
        fallback path."""
        if self._reset_map_worker is not None and self._reset_map_worker.isRunning():
            return  # already in flight - the button is disabled meanwhile anyway

        self.page_slam.btn_reset_map.setEnabled(False)
        self.console.log_warning("[TACTICAL SLAM] Resetting SLAM map...")
        self.page_terminal.log_warning("[TACTICAL SLAM] Resetting SLAM map...")
        self.toast.show_message("Resetting SLAM map...", "#d29922", 4000)

        self._reset_map_worker = SlamMapResetWorker(self.map_listener.tcp_host, tcp_port=5765, parent=self)
        self._reset_map_worker.finished_result.connect(self._on_reset_map_result)
        self._reset_map_worker.start()

    def _on_reset_map_result(self, success: bool, message: str):
        if success:
            self.console.log_success(f"[TACTICAL SLAM] Map reset: {message}")
            self.page_terminal.log_success(f"[TACTICAL SLAM] Map reset: {message}")
            self.toast.show_message("SLAM map reset", "#3fb950", 3000)
            self.page_slam.clear_local_map_display()
        else:
            self.console.log_error(f"[TACTICAL SLAM] Map reset FAILED: {message}")
            self.page_terminal.log_error(f"[TACTICAL SLAM] Map reset FAILED: {message}")
            self.toast.show_message("Map reset failed - see console", "#f85149", 5000)
            # Map wasn't actually touched, so leave the display exactly as it was.

        # Re-enable unless armed state changed to True while the reset was in flight.
        self.page_slam.set_armed_state(self.last_telemetry.armed)

    def _on_pause_path_requested(self):
        """Called when user clicks 'PAUSE / HOLD' in Tactical SLAM tab."""
        self.offboard_pump_timer.stop()
        if not self.worker or not self.worker.isRunning():
            return
        self.worker.set_mode("AUTO.LOITER")
        self.path_in_progress = False
        self.path_paused = True
        self.loiter_pause_start_time = time.time()
        self._loiter_warned = False
        self.path_awaiting_climb = False
        self.page_slam.set_executing_state(False, paused=True)
        self.console.log_warning("[PAUSE] Drone holding position in AUTO.LOITER")
        self.page_terminal.log_warning("[PAUSE] Drone holding position in AUTO.LOITER")
        self.toast.show_message("Path Paused: Position Hold", "#d29922", 3000)

    def _on_resume_path_requested(self):
        """Called when user clicks 'RESUME PATH' in Tactical SLAM tab."""
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot resume path: Not connected to drone")
            self.toast.show_message("Cannot resume: Disconnected", "#da3633")
            return

        # 1. Arm Status Interlock
        if not self.last_telemetry.armed:
            self.console.log_error(
                "❌ FLIGHT INTERLOCK REJECTED: Cannot resume path while vehicle is DISARMED! "
                "Arm drone and confirm clear airspace first."
            )
            self.page_terminal.log_error(
                "❌ FLIGHT INTERLOCK REJECTED: Drone is DISARMED! Arm before resuming path."
            )
            self.toast.show_message("Cannot Resume: Drone is Disarmed!", "#da3633", 5000)
            return

        if not self.active_waypoints or self.current_wpt_idx >= len(self.active_waypoints):
            self.console.log_warning("No remaining waypoints to resume.")
            return

        # 2. Dynamic Collision Re-validation Before Resuming
        if self.latest_map_data is not None:
            grid, res, ox, oy = self.latest_map_data
            remaining_wpts = self.active_waypoints[self.current_wpt_idx:]
            is_blocked, col_pt, col_dist = self.path_planner.check_path_collision(
                grid, res, ox, oy, remaining_wpts, (self.last_telemetry.x, self.last_telemetry.y), lookahead_m=1.5
            )
            if is_blocked:
                self.console.log_error(
                    f"🚨 CANNOT RESUME: Path still blocked by obstacle {col_dist:.2f}m ahead! "
                    "Clear the obstacle or plan a new detour."
                )
                self.toast.show_message(f"Cannot Resume: Blocked ({col_dist:.2f}m)!", "#da3633", 5000)
                return

        self.console.log_cmd("[RESUME] Switching to OFFBOARD...")
        self.page_terminal.log_cmd("[RESUME] Switching to OFFBOARD...")
        self.worker.set_mode("OFFBOARD")
        self.path_in_progress = True
        self.path_paused = False
        self.page_slam.set_executing_state(True, paused=False)

        next_wpt = self.active_waypoints[self.current_wpt_idx]
        dx = next_wpt[0] - self.last_telemetry.x
        dy = next_wpt[1] - self.last_telemetry.y
        seg_dist = math.sqrt(dx * dx + dy * dy)
        yaw_deg = math.degrees(math.atan2(dy, dx)) if seg_dist > 0.05 else self.last_telemetry.heading
        self.current_target_yaw = yaw_deg

        self.worker.move_to_waypoint(next_wpt[0], next_wpt[1], z=self.cruise_z, yaw_deg=yaw_deg)
        self.offboard_pump_timer.start()

        self.exec_tracker.start_tracking(
            f"waypoint W{self.current_wpt_idx+1}", self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
            target_dist=seg_dist
        )
        self.toast.show_message("Path Resumed", "#238636", 3000)

    def _on_abort_path_requested(self):
        """Called when user clicks 'ABORT & LAND' in Tactical SLAM tab."""
        self.offboard_pump_timer.stop()
        self.path_in_progress = False
        self.path_paused = False
        self.path_awaiting_climb = False
        self.active_waypoints = []
        self.current_wpt_idx = 0
        self.page_slam.set_executing_state(False, paused=False)
        self.page_slam.canvas.clear_goal()

        if self.worker and self.worker.isRunning():
            self.worker.set_mode("AUTO.LAND")
            self.console.log_error("[ABORT] Emergency transition to AUTO.LAND initiated!")
            self.page_terminal.log_error("[ABORT] Emergency transition to AUTO.LAND initiated!")
            self.toast.show_message("ABORT: AUTO.LAND Engaged", "#da3633", 5000)

    # -------------------------------------------------------------------------
    # CLI Command Dispatcher
    # -------------------------------------------------------------------------

    def _execute_cli_command(self, cmd_text: str):
        parts = cmd_text.strip().split()
        if not parts:
            return
        action = parts[0].lower()

        if not self.worker or not self.worker.isRunning():
            if action not in ("help", "clear"):
                self.console.log_error("Cannot execute: GCS is disconnected")
                self.page_terminal.log_error("Cannot execute: GCS is disconnected")
                return

        if action == "help":
            help_msg = (
                "Available Commands:\n"
                "  move <dx> <dy> <dz>  - Relative NED displacement in meters\n"
                "  takeoff <alt>        - Takeoff to altitude AGL (meters)\n"
                "  arm [force]          - Arm drone (force for bench test)\n"
                "  disarm               - Disarm motors\n"
                "  mode <NAME>          - offboard, posctl, hold, land, rtl\n"
                "  yaw <deg>            - Relative heading rotation in degrees\n"
                "  kill                 - Immediate emergency motor cutoff\n"
                "  clear                - Clear console log window\n"
            )
            self.console.log_info(help_msg)
            self.page_terminal.log_info(help_msg)
        elif action == "clear":
            self.console.log_box.clear()
            self.page_terminal.log_box.clear()
        elif action == "arm":
            force = (len(parts) > 1 and parts[1].lower() == "force") or self.chk_force_arm.isChecked()
            self._cmd_arm(force=force)
        elif action == "disarm":
            self._cmd_disarm()
        elif action == "mode" and len(parts) >= 2:
            self._cmd_mode(parts[1])
        elif action == "takeoff":
            alt = float(parts[1]) if len(parts) >= 2 else 1.0
            self._cmd_takeoff(alt)
        elif action == "land":
            self._cmd_mode("AUTO.LAND")
        elif action == "rtl":
            self._cmd_mode("AUTO.RTL")
        elif action == "kill":
            self._cmd_kill()
        elif action == "yaw" and len(parts) >= 2:
            try:
                dyaw = float(parts[1])
                self._cmd_yaw(dyaw)
            except ValueError:
                self.console.log_error("Yaw angle must be a number")
        elif action == "move":
            if len(parts) < 4:
                self.console.log_error("Usage: move <dx> <dy> <dz> (e.g. move 1 0 0)")
                return
            try:
                dx = float(parts[1])
                dy = float(parts[2])
                dz = float(parts[3])
                self._cmd_move(dx, dy, dz)
            except ValueError:
                self.console.log_error("Coordinates must be floating point numbers")
        else:
            self.console.log_warning(f"Unknown command: '{action}'. Type 'help' for instructions.")

    def closeEvent(self, event):
        """Clean shutdown of threads and embedded processes on window close."""
        if hasattr(self, "page_slam"):
            self.page_slam.stop_rviz()
        if self.map_listener:
            self.map_listener.stop()
        if self.worker and self.worker.isRunning():
            self.worker.disconnect_endpoint()
        event.accept()


def main():
    if sys.platform.startswith("linux"):
        for _p in ["/usr/lib/aarch64-linux-gnu/qt5/plugins", "/usr/lib/x86_64-linux-gnu/qt5/plugins"]:
            if os.path.exists(_p):
                os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _p
                break
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLESHEET)
    window = DroneGCSMainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
