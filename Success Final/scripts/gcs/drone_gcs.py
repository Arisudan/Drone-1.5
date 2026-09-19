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

import numpy as np

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
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QFrame, QLabel, QPushButton, QStackedWidget,
    QSplitter, QProgressBar, QMessageBox, QLineEdit, QComboBox,
    QGroupBox
)

from core.telemetry import TelemetrySnapshot
from core.health import EngineHealth, EngineStatus, get_registry
from core.execution_tracker import ExecutionTracker
from core.path_planner import AStarPathPlanner
from protocol.mavlink_worker import MAVLinkWorker
from protocol.ros2_map_listener import ROS2MapListener, SlamMapResetWorker
from ui.styles import DARK_STYLESHEET, PALETTE
from ui.toast import NotificationToast
from ui.hud_widget import HUDWidget
from ui.slam_map_widget import SLAMMapWidget
from ui.cli_console import CLIConsoleWidget
from ui.motor_widget import MotorWidget
from ui.video_feed_widget import VideoFeedWidget, FloatingVideoWindow
from ui.top_status_strip import TopStatusStrip
from ui.sidebar_nav import SidebarNav
from ui.logs_tab import LogsTabWidget
from ui.config_tab import ConfigTabWidget
from core.flight_log import FlightLogger
from core.settings import load_settings


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

        # Persisted settings and the flight recorder are built before the UI,
        # because the Config and Logs workspaces are views onto them.
        self.settings = load_settings()
        self.flight_logger = FlightLogger()

        # Core Engines
        self.worker: Optional[MAVLinkWorker] = None
        self.exec_tracker = ExecutionTracker(tolerance=0.20, timeout_sec=14.0)
        self.last_telemetry = TelemetrySnapshot()

        # Bench Force Arm tracking state
        self._last_arm_was_forced: bool = False

        # Safety state machine: when `disarm` is issued while genuinely
        # airborne, it's redirected to AUTO.LAND instead of an instant motor
        # cutoff. This flag tracks that a real disarm is still owed once
        # PX4's own landed_state confirms touchdown - checked every telemetry
        # update in _on_telemetry_updated.
        self._pending_autodisarm_after_land: bool = False
        # Sane bounds for operator-entered command values - guards against a
        # typo or fat-fingered altitude/displacement being sent as a real
        # flight command. Adjust for your actual room/airframe if needed.
        self.TAKEOFF_ALT_MIN_M: float = self.settings.limits.takeoff_alt_min_m
        self.TAKEOFF_ALT_MAX_M: float = self.settings.limits.takeoff_alt_max_m
        self.MOVE_MAX_DELTA_M: float = self.settings.limits.move_max_delta_m

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

        # Liveness watch on that pump. Its stall threshold is not a UI
        # preference - PX4 drops OFFBOARD mode if setpoints stop arriving
        # for 500 ms, so a stalled pump mid-path is a flight event. 250 ms
        # (2.5 missed ticks at 10 Hz) is already degraded.
        self.pump_health = EngineHealth(
            "OffboardPump", stall_after_s=0.5, degrade_after_s=0.25)
        get_registry().register(self.pump_health)
        # The pump timer is started/stopped from nine different places; rather
        # than touching each one, the UI tick mirrors its real state into the
        # health object from a single site (see _on_ui_tick).
        self._pump_was_live: bool = False
        # True while the execution verifier is showing a live measurement, so
        # the panel can be reset exactly once when tracking ends.
        self._verifier_dirty: bool = False
        self._last_health_sweep: float = 0.0

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

        # One capture thread, three viewports: the FPV tab's own label, the
        # cockpit's centre panel, and the draggable SLAM overlay.
        self.fpv_float = FloatingVideoWindow(self)
        self.page_fpv.frame_broadcast.connect(self.hud.video.on_frame)
        self.page_fpv.frame_broadcast.connect(self.fpv_float.sink.on_frame)

        # Auto-connect to default autopilot endpoint on launch (UDP 14550)
        self._connect_to_endpoint("172.16.101.84", 14550, protocol="udp")

    def _init_ui(self):
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        root_layout = QVBoxLayout(main_widget)
        root_layout.setContentsMargins(6, 6, 6, 6)
        root_layout.setSpacing(6)

        # Footer built after the workspace (see _build_footer_bar), so the
        # stacked pages keep their stretch and the bar stays a fixed strip.
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

        # Workspace column: pages above, vision toggles below. The footer lives
        # here rather than at the window root so it begins where the navigation
        # rail ends, instead of running underneath it.
        work_col = QVBoxLayout()
        work_col.setContentsMargins(0, 0, 0, 0)
        work_col.setSpacing(6)

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

        # Page 6: Flight Logs - history of every armed session
        self.page_logs = LogsTabWidget(self.flight_logger, self)
        self.stack.addWidget(self.page_logs)

        # Page 7: Configuration - typed settings, validated and persisted
        self.page_config = ConfigTabWidget(self.settings, self)
        self.page_config.settings_saved.connect(self._on_settings_saved)
        self.stack.addWidget(self.page_config)

        work_col.addWidget(self.stack, 1)

        # 3. Footer: vision-engine toggles, aligned to the workspace's left edge
        self.footer_bar = self._build_footer_bar()
        work_col.addWidget(self.footer_bar)

        workspace_box.addLayout(work_col, 1)
        root_layout.addLayout(workspace_box, 1)

    def _build_cockpit_page(self) -> QWidget:
        """Page 0: Aviation PFD side-by-side with Proven Control Dispatcher and Closed-Loop Verifier.

        The dispatcher is split into titled groups - COMMANDS, NAVIGATION,
        FLIGHT MODE, SAFETY & OVERRIDE - rather than one undifferentiated stack
        of rows. A titled frame states what a cluster of controls is for, and it
        puts the irreversible actions in their own box at the bottom instead of
        one tab-stop away from ARM.
        """
        page = QWidget(self)
        layout = QHBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        splitter = QSplitter(Qt.Horizontal, page)
        splitter.setHandleWidth(4)

        # Left: Aviation Primary Flight Display (PFD)
        self.hud = HUDWidget(self)
        splitter.addWidget(self.hud)

        # Right: Comprehensive Control Request & Output Panel
        right_panel = QWidget(self)
        rl = QVBoxLayout(right_panel)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(8)

        # ---- COMMAND DISPATCHER ----
        # Arm/Mode status used to be repeated here as its own pair of pills -
        # dropped as a duplicate of the always-visible header badges. The HUD's
        # own PFD banner stays: that one reads as a genuine flight instrument.
        ctrl_card = QWidget(self)
        cl = QVBoxLayout(ctrl_card)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(6)

        # ---- Primary commands ----
        # No title: ARM / DISARM / HOLD / LAND already say what they are, and a
        # caption over them spent a row restating it.
        grp_commands = QFrame(self)
        grp_commands.setProperty("class", "cardFrame")
        gc = QVBoxLayout(grp_commands)
        gc.setContentsMargins(10, 8, 10, 10)
        gc.setSpacing(6)

        actions_box = QHBoxLayout()
        actions_box.setSpacing(6)

        self.btn_arm = QPushButton("ARM", self)
        self.btn_arm.setObjectName("btnArm")
        self.btn_arm.setMinimumHeight(34)
        self.btn_arm.clicked.connect(self._cmd_arm_from_ui)
        actions_box.addWidget(self.btn_arm, 1)

        self.btn_disarm = QPushButton("DISARM", self)
        self.btn_disarm.setObjectName("btnDisarm")
        self.btn_disarm.setMinimumHeight(34)
        self.btn_disarm.clicked.connect(self._cmd_disarm)
        actions_box.addWidget(self.btn_disarm, 1)

        # HOLD and LAND move the aircraft, so they carry the navigation colour
        # rather than a decorative one - LAND in particular used to be purple,
        # which signalled nothing about what it does.
        self.btn_hold = QPushButton("HOLD", self)
        self.btn_hold.setObjectName("btnNav")
        self.btn_hold.setMinimumHeight(34)
        self.btn_hold.clicked.connect(lambda: self._cmd_mode("AUTO.LOITER"))
        actions_box.addWidget(self.btn_hold, 1)

        self.btn_land = QPushButton("LAND", self)
        self.btn_land.setObjectName("btnNav")
        self.btn_land.setMinimumHeight(34)
        self.btn_land.clicked.connect(lambda: self._cmd_mode("AUTO.LAND"))
        actions_box.addWidget(self.btn_land, 1)

        gc.addLayout(actions_box)
        cl.addWidget(grp_commands)

        # ---- NAVIGATION | FLIGHT MODE ----
        # One frame, two halves, divided by a rule: both answer "where do I
        # point the aircraft", and giving each its own bordered card made the
        # panel read as four unrelated things instead of two.
        flight_frame = QFrame(self)
        flight_frame.setProperty("class", "cardFrame")
        ff = QHBoxLayout(flight_frame)
        ff.setContentsMargins(6, 0, 6, 6)
        ff.setSpacing(12)

        grp_nav = QGroupBox("NAVIGATION", self)
        grp_nav.setObjectName("groupFlat")
        gn = QGridLayout(grp_nav)
        gn.setContentsMargins(4, 2, 4, 2)
        gn.setHorizontalSpacing(8)
        gn.setVerticalSpacing(6)
        gn.setColumnStretch(1, 1)

        FIELD_W = 70      # numeric entry
        ACTION_W = 120    # dispatch button

        lbl_to = QLabel("Takeoff Alt (m):", self)
        lbl_to.setObjectName("fieldLabel")
        gn.addWidget(lbl_to, 0, 0)

        self.ent_takeoff_alt = QLineEdit("1.0", self)
        self.ent_takeoff_alt.setFixedWidth(FIELD_W)
        self.ent_takeoff_alt.setAlignment(Qt.AlignCenter)
        self.ent_takeoff_alt.setValidator(QDoubleValidator(0.2, 10.0, 2, self))
        gn.addWidget(self.ent_takeoff_alt, 0, 1, Qt.AlignLeft)

        self.btn_takeoff = QPushButton("TAKEOFF", self)
        self.btn_takeoff.setObjectName("btnNav")
        self.btn_takeoff.setFixedWidth(ACTION_W)
        gn.addWidget(self.btn_takeoff, 0, 2)
        self.btn_takeoff.clicked.connect(self._cmd_takeoff_from_ui)

        lbl_yaw = QLabel("Yaw (\u00b0):", self)
        lbl_yaw.setObjectName("fieldLabel")
        gn.addWidget(lbl_yaw, 1, 0)

        self.ent_yaw = QLineEdit("90.0", self)
        self.ent_yaw.setFixedWidth(FIELD_W)
        self.ent_yaw.setAlignment(Qt.AlignCenter)
        self.ent_yaw.setValidator(QDoubleValidator(-360.0, 360.0, 1, self))
        gn.addWidget(self.ent_yaw, 1, 1, Qt.AlignLeft)

        self.btn_yaw = QPushButton("ROTATE YAW", self)
        self.btn_yaw.setObjectName("btnNav")
        self.btn_yaw.setFixedWidth(ACTION_W)
        gn.addWidget(self.btn_yaw, 1, 2)
        self.btn_yaw.clicked.connect(self._cmd_yaw_from_ui)

        ff.addWidget(grp_nav, 3)

        divider = QFrame(self)
        divider.setObjectName("vDivider")
        divider.setFixedWidth(1)
        ff.addWidget(divider)

        grp_mode = QGroupBox("FLIGHT MODE", self)
        grp_mode.setObjectName("groupFlat")
        gm = QVBoxLayout(grp_mode)
        gm.setContentsMargins(4, 2, 4, 2)
        gm.setSpacing(6)

        self.combo_modes = QComboBox(self)
        self.combo_modes.addItems(PX4_MODES_LIST)
        self.combo_modes.setCurrentText("STABILIZED")
        self.combo_modes.setMinimumHeight(28)
        gm.addWidget(self.combo_modes)

        # Neutral on purpose: a mode change is routine, and colouring it would
        # dilute the three colours that do mean something here.
        self.btn_set_mode = QPushButton("SET MODE", self)
        self.btn_set_mode.setMinimumHeight(28)
        self.btn_set_mode.clicked.connect(self._cmd_set_mode_from_ui)
        gm.addWidget(self.btn_set_mode)

        ff.addWidget(grp_mode, 2)
        cl.addWidget(flight_frame)

        # ---- Override ----
        # The two controls that bypass or cut flight safety live together, at
        # the far end of the panel from ARM. Untitled: an amber warning
        # checkbox above a red bar is not mistakable for anything else.
        grp_safety = QFrame(self)
        grp_safety.setProperty("class", "cardFrame")
        gs = QVBoxLayout(grp_safety)
        gs.setContentsMargins(10, 8, 10, 10)
        gs.setSpacing(6)

        # No glyph prefix: the U+26D4 "no entry" symbol has no coverage in the
        # UI font here and rendered as a tofu box.
        self.btn_kill = QPushButton("EMERGENCY KILL", self)
        self.btn_kill.setObjectName("btnKill")
        self.btn_kill.setMinimumHeight(32)
        self.btn_kill.clicked.connect(self._cmd_kill)
        gs.addWidget(self.btn_kill)

        cl.addWidget(grp_safety)
        rl.addWidget(ctrl_card)

        # ---- Execution verifier ----
        # (6) No title: the badge states what this is at a glance, and a header
        # over three lines of content was spending a row to say so in words.
        # (7) Three compact rows instead of a caption/value tile grid - same
        # four numbers, roughly half the vertical space.
        grp_verif = QFrame(self)
        grp_verif.setProperty("class", "cardFrame")
        vl = QVBoxLayout(grp_verif)
        vl.setContentsMargins(10, 8, 10, 8)
        vl.setSpacing(5)

        state_row = QHBoxLayout()
        state_row.setSpacing(8)
        self.lbl_exec_state = QLabel("IDLE", self)
        self.lbl_exec_state.setObjectName("execBadgeIdle")
        self.lbl_exec_state.setFixedWidth(84)
        self.lbl_exec_state.setAlignment(Qt.AlignCenter)
        state_row.addWidget(self.lbl_exec_state)

        self.lbl_exec_val = QLabel("Holding position - no command in flight", self)
        self.lbl_exec_val.setObjectName("execMessage")
        state_row.addWidget(self.lbl_exec_val, 1)

        self.lbl_exec_elapsed = QLabel("--", self)
        self.lbl_exec_elapsed.setObjectName("valueMono")
        self.lbl_exec_elapsed.setFixedWidth(52)
        self.lbl_exec_elapsed.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        state_row.addWidget(self.lbl_exec_elapsed)
        vl.addLayout(state_row)

        # Four evenly-weighted columns rather than one padded string: each axis
        # owns its own slot, so a value changing width cannot shunt the others
        # sideways while you are watching them move.
        read_row = QGridLayout()
        read_row.setHorizontalSpacing(8)
        read_row.setVerticalSpacing(0)
        self.lbl_exec_axes = {}
        for col, (key, axis) in enumerate(
            (("dx", "\u0394X"), ("dy", "\u0394Y"), ("dz", "\u0394Z"), ("spd", "SPD"))
        ):
            # Caption and value stay welded together and the pair is centred in
            # its column. Right-aligning the value inside a stretched cell put
            # a gulf between "dX" and its number, so each label read as if it
            # belonged to the figure on its left.
            cell = QHBoxLayout()
            cell.setSpacing(7)
            cell.addStretch()
            cap = QLabel(axis, self)
            cap.setObjectName("execAxis")
            cell.addWidget(cap)
            val = QLabel("0.00" if key == "spd" else "+0.00", self)
            val.setObjectName("execReadout")
            val.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            cell.addWidget(val)
            cell.addStretch()
            read_row.addLayout(cell, 0, col)
            read_row.setColumnStretch(col, 1)
            self.lbl_exec_axes[key] = val
        vl.addLayout(read_row)

        self.progress_verif = QProgressBar(self)
        self.progress_verif.setFixedHeight(14)
        self.progress_verif.setTextVisible(True)
        self.progress_verif.setFormat("no displacement target")
        self.progress_verif.setValue(0)
        vl.addWidget(self.progress_verif)

        rl.addWidget(grp_verif)

        # ---- Interactive Terminal / Console Bar ----
        self.console = CLIConsoleWidget(self)
        self.console.command_submitted.connect(self._execute_cli_command)
        rl.addWidget(self.console, 1)

        splitter.addWidget(right_panel)
        # 7:3 rather than 3:2 - the command column only ever holds fixed-height
        # controls, so the extra width went to padding while the camera pane
        # could use it.
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        right_panel.setMaximumWidth(640)

        layout.addWidget(splitter)
        return page

    def _build_diagnostics_page(self) -> QWidget:
        """Page 4: Detailed Diagnostics & Bench Safety view.

        Nine cards, each a plain frame with its heading and a rule INSIDE the
        border. QGroupBox paints its title across the top border line, which
        left every heading straddling the edge of its own box - fine for one
        group in a control panel, visually broken in a nine-card grid.
        """
        page = QWidget(self)
        outer = QVBoxLayout(page)
        outer.setContentsMargins(16, 12, 16, 12)
        outer.setSpacing(10)

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)

        # key -> QLabel, so the telemetry tick addresses fields by name instead
        # of rebuilding a 25-line f-string 30 times a second.
        self.diag_values: dict = {}

        cards = [
            ("PX4 AUTOPILOT", [
                ("sysid", "System / Comp ID"), ("connected", "Link"),
                ("arm_state", "Arm State"), ("flight_mode", "Flight Mode"),
                ("flight_time", "Flight Time"),
            ]),
            ("LINK THROUGHPUT", [
                ("rx", "RX rate"), ("tx", "TX rate"),
            ]),
            ("D435i VIO / EKF2", [
                ("vio_health", "VIO Stream"), ("vio_age", "Last Vision Packet"),
                ("ekf2", "EKF2 Fusion"),
            ]),
            ("LOCAL POSITION NED", [
                ("pos_x", "North (x)"), ("pos_y", "East (y)"),
                ("pos_z", "Down (z)"), ("alt", "Altitude AGL"),
            ]),
            ("BODY VELOCITIES", [
                ("vx", "Vx"), ("vy", "Vy"), ("vz", "Vz"),
                ("gspeed", "Ground Speed"),
            ]),
            ("MOTOR OUTPUTS", [
                ("m1", "M1  FR CCW"), ("m2", "M2  RL CCW"),
                ("m3", "M3  FL CW"), ("m4", "M4  RR CW"),
            ]),
            ("ATTITUDE / HEADING", [
                ("roll", "Roll"), ("pitch", "Pitch"),
                ("yaw", "Yaw"), ("heading", "Heading"),
            ]),
            ("POWER", [
                ("volt", "Battery"), ("curr", "Current"), ("pct", "Remaining"),
            ]),
            ("GPS", [
                ("sats", "Satellites"), ("hdop", "HDOP"), ("fix", "Fix Type"),
            ]),
        ]

        for i, (card_title, rows) in enumerate(cards):
            card = QFrame(self)
            card.setProperty("class", "cardFrame")
            cv = QVBoxLayout(card)
            cv.setContentsMargins(12, 10, 12, 10)
            cv.setSpacing(6)

            heading = QLabel(card_title, self)
            heading.setObjectName("cardHeading")
            cv.addWidget(heading)

            rule = QFrame(self)
            rule.setObjectName("hDivider")
            rule.setFixedHeight(1)
            cv.addWidget(rule)

            body = QGridLayout()
            body.setHorizontalSpacing(8)
            body.setVerticalSpacing(3)
            body.setColumnStretch(1, 1)
            for r, (key, caption) in enumerate(rows):
                cap = QLabel(caption, self)
                cap.setObjectName("fieldSubLabel")
                body.addWidget(cap, r, 0)
                val = QLabel("--", self)
                val.setObjectName("diagValue")
                val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                body.addWidget(val, r, 1)
                self.diag_values[key] = val
            cv.addLayout(body)
            cv.addStretch()

            grid.addWidget(card, i // 3, i % 3)

        for c in range(3):
            grid.setColumnStretch(c, 1)
        outer.addLayout(grid)
        outer.addStretch()
        return page

    def _build_footer_bar(self) -> QFrame:
        """Vision-engine toggles, laid out as in the Walle GCS control row.

        These four request perception overlays that this ground station does
        not yet carry: detection, monocular depth, face recognition and human
        tracking all live in the Walle codebase and need torch, RF-DETR,
        Depth-Anything and InsightFace, none of which are installed here.

        The switches are live and latch, so the requested set is real UI state
        ready to drive an engine the moment one exists. What they must not do
        is imply the overlay is running, so the footer note names every
        requested-but-unavailable engine for as long as it is switched on.
        """
        bar = QFrame(self)
        bar.setObjectName("footerBar")
        fl = QHBoxLayout(bar)
        fl.setContentsMargins(10, 5, 10, 5)
        fl.setSpacing(8)

        self.vision_buttons = {}
        specs = [
            ("detect", "Detect", "detect",
             "Object detection (RF-DETR) - engine not installed on this station"),
            ("depth", "Depth", "depth",
             "Monocular depth overlay (Depth-Anything V2) - engine not installed"),
            ("face", "Face Rec", "face",
             "Face recognition (InsightFace SCRFD + ArcFace) - engine not installed"),
            ("track", "Track", "track",
             "Human tracking (YOLOv8-pose + DeepSort) - engine not installed"),
        ]
        for key, label, obj, tip in specs:
            btn = QPushButton(label, self)
            btn.setObjectName("btnVision")
            btn.setProperty("visionRole", key)
            btn.setCheckable(True)
            btn.setToolTip(tip)
            btn.clicked.connect(lambda checked, k=key: self._on_vision_toggle(k, checked))
            fl.addWidget(btn)
            self.vision_buttons[key] = btn

        fl.addStretch()

        self.lbl_footer_note = QLabel(
            "Vision engines unavailable - no onboard perception pipeline connected", self)
        self.lbl_footer_note.setObjectName("footerNote")
        fl.addWidget(self.lbl_footer_note)
        return bar

    def _on_vision_toggle(self, key: str, checked: bool) -> None:
        """Latch the request and keep the footer honest about what is missing.

        The button state is kept - it is a genuine operator request - but the
        note spells out that nothing is rendering it, so a lit switch can never
        be mistaken for a live overlay.
        """
        label = {"detect": "Detect", "depth": "Depth",
                 "face": "Face Rec", "track": "Track"}.get(key, key)
        if checked:
            msg = (f"{label} requested - no vision engine on this station "
                   "(needs the onboard perception pipeline)")
            self.console.log_warning(msg)
            self.toast.show_message(f"{label}: no vision engine available",
                                    "#d29922", 3000)
        else:
            self.console.log_info(f"{label} request cleared")
        self._refresh_vision_note()

    def _refresh_vision_note(self) -> None:
        names = [{"detect": "Detect", "depth": "Depth", "face": "Face Rec",
                  "track": "Track"}[k]
                 for k, b in self.vision_buttons.items() if b.isChecked()]
        if names:
            self.lbl_footer_note.setText(
                "Requested but unavailable: " + ", ".join(names)
                + " - no onboard perception pipeline connected")
        else:
            self.lbl_footer_note.setText(
                "Vision engines unavailable - no onboard perception pipeline connected")

    def _diag_set(self, key: str, text: str, colour: str = None) -> None:
        """Write one diagnostics field, optionally colouring it by state.

        Colour is applied per-widget rather than through the global sheet
        because it encodes a live value, not a widget role.
        """
        lbl = self.diag_values.get(key)
        if lbl is None:
            return
        lbl.setText(text)
        lbl.setStyleSheet(
            f"color: {colour};" if colour else f"color: {PALETTE['text_bright']};")

    def _on_view_changed(self, idx: int):
        self.stack.setCurrentIndex(idx)
        page = self.stack.widget(idx)

        # Any workspace that shows the camera starts it. Idempotent - does
        # nothing if a feed is already running, and there is only ever one
        # capture thread regardless of how many viewports are open.
        if page in (self.page_fpv, self.page_cockpit, self.page_slam):
            self.page_fpv.ensure_started()

        # The floating viewport belongs to the SLAM workspace: while flying a
        # planned route you want the map full-size and the camera beside it.
        if page is self.page_slam:
            if not self.fpv_float.isVisible():
                self._position_fpv_float()
            self.fpv_float.show()
            self.fpv_float.raise_()
        else:
            self.fpv_float.hide()

    def _position_fpv_float(self):
        """Park the floating viewport near the GCS's bottom-right on first show."""
        geo = self.geometry()
        self.fpv_float.move(geo.right() - self.fpv_float.width() - 40,
                            geo.bottom() - self.fpv_float.height() - 60)

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
        # Header no longer carries these - see TopStatusStrip.update_rates.
        self._diag_set("rx", f"{rx_rate:.1f} msg/s")
        self._diag_set("tx", f"{tx_rate:.1f} msg/s")

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

    def _on_settings_saved(self, cfg) -> None:
        """Apply what can change without a restart, and say what cannot."""
        self.settings = cfg
        self.TAKEOFF_ALT_MIN_M = cfg.limits.takeoff_alt_min_m
        self.TAKEOFF_ALT_MAX_M = cfg.limits.takeoff_alt_max_m
        self.MOVE_MAX_DELTA_M = cfg.limits.move_max_delta_m
        self.page_fpv.txt_url.setText(cfg.video.stream_url)
        msg = ("Settings saved. Command limits and stream URL applied now; "
               "connection changes take effect on the next Connect.")
        self.console.log_success(msg)
        self.toast.show_message("Settings saved", "#238636", 3000)

    def _set_exec_state(self, state: str) -> None:
        """Colour the verifier's state badge. Purely presentational."""
        badge = {
            "IDLE": "execBadgeIdle",
            "TRACKING": "execBadgeActive",
            "EXECUTED": "execBadgeOk",
            "STALLED": "execBadgeFail",
            "REJECTED": "execBadgeFail",
        }.get(state, "execBadgeIdle")
        self.lbl_exec_state.setText(state)
        self.lbl_exec_state.setObjectName(badge)
        # Qt caches the style per objectName, so it must be re-polished by hand
        # after the name changes or the badge keeps its previous colour.
        self.lbl_exec_state.style().unpolish(self.lbl_exec_state)
        self.lbl_exec_state.style().polish(self.lbl_exec_state)

    def _update_verifier(self, state: str, msg: str, pct: float = 0.0,
                         dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                         speed: float = 0.0, elapsed: float = 0.0,
                         delta: float = 0.0, target: float = 0.0) -> None:
        """Single writer for every widget in the execution verifier panel.

        Previously the panel was written from two places with different
        formats and never reset, so after a command finished it kept showing
        the last run's numbers indefinitely - the display implied a live
        measurement when nothing was being measured.
        """
        self._set_exec_state(state)
        self.lbl_exec_val.setText(msg)
        self.lbl_exec_elapsed.setText(f"{elapsed:.1f}s" if elapsed > 0 else "--")
        self.lbl_exec_axes["dx"].setText(f"{dx:+.2f}")
        self.lbl_exec_axes["dy"].setText(f"{dy:+.2f}")
        self.lbl_exec_axes["dz"].setText(f"{dz:+.2f}")
        self.lbl_exec_axes["spd"].setText(f"{speed:.2f}")
        self.progress_verif.setValue(max(0, min(100, int(pct))))
        # Travelled-vs-target belongs inside the bar it describes, rather than
        # on a fourth line repeating what the bar already shows.
        self.progress_verif.setFormat(
            f"{delta:.2f} / {target:.2f} m" if target > 0 else "no displacement target")

    def _on_command_ack_received(self, cmd_id: int, result_code: int, cmd_name: str, result_str: str):
        """Real-time feedback when Pixhawk acknowledges or rejects a command."""
        # Immediately notify execution tracker so countdown timers are aborted and status is updated
        self.exec_tracker.notify_command_ack(cmd_id, result_code, cmd_name, result_str)
        self._update_verifier(
            self.exec_tracker.status if self.exec_tracker.status in
            ("EXECUTED", "STALLED", "REJECTED") else "TRACKING",
            self.exec_tracker.status_msg,
            pct=self.exec_tracker.progress_pct,
            elapsed=self.exec_tracker.elapsed_time,
            delta=self.exec_tracker.current_delta,
            target=getattr(self.exec_tracker, "req_dist", 0.0) or 0.0,
        )

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

        # Safety state machine: a mid-flight `disarm` was redirected to
        # AUTO.LAND (see _cmd_disarm) - watch for PX4's own landed_state to
        # confirm real touchdown before actually cutting power.
        if self._pending_autodisarm_after_land and t.landed_state == 1:
            self._pending_autodisarm_after_land = False
            self.console.log_success("Landing detected (ON_GROUND) - disarming now.")
            self.page_terminal.log_success("Landing detected (ON_GROUND) - disarming now.")
            self.toast.show_message("Landed - disarming", "#238636", 3000)
            if self.worker and self.worker.isRunning():
                self.worker.disarm(force=False)

    # -------------------------------------------------------------------------
    # UI Periodic Tick (30 Hz)
    # -------------------------------------------------------------------------

    def _on_ui_tick(self):
        t = self.last_telemetry
        t.check_vision_staleness(max_age_sec=self.settings.alerts.vision_stale_s)
        self.top_strip.update_telemetry(t)
        self.sidebar.set_flight_state(t.flight_mode, t.armed)
        self.sidebar.set_instruments(t.ground_speed, t.altitude)

        # One record per armed session; the Logs tab updates the moment it closes.
        closed = self.flight_logger.update(t, forced=self._last_arm_was_forced)
        if closed is not None:
            self.page_logs.add_record(closed)
            self.console.log_success(
                f"Flight recorded: {closed.duration_hms()}, "
                f"{closed.distance_m:.1f} m travelled")

        # Arm/Mode status: shown in the header badges (top_strip.update_telemetry above)
        # and the HUD's own PFD banner - no longer duplicated here.

        # 3. Update Closed-Loop Execution Tracker
        if self.exec_tracker.active:
            res = self.exec_tracker.update(t.x, t.y, t.z, armed=t.armed, flight_mode=t.flight_mode, cur_heading=t.heading)
            self._verifier_dirty = True
            self._update_verifier(
                "TRACKING" if res["status"] not in ("EXECUTED", "STALLED", "REJECTED")
                else res["status"],
                res["status_msg"],
                pct=res["progress_pct"],
                dx=t.x - self.exec_tracker.x0,
                dy=t.y - self.exec_tracker.y0,
                dz=t.z - self.exec_tracker.z0,
                speed=t.ground_speed,
                elapsed=res.get("elapsed", 0.0),
                delta=res.get("delta", 0.0),
                target=res.get("target", 0.0) or 0.0,
            )

            if res["status"] == "EXECUTED":
                self.console.log_success(res["status_msg"])
                self.page_terminal.log_success(res["status_msg"])
                self.toast.show_message(res["status_msg"], "#238636")
            elif res["status"] in ("STALLED", "REJECTED"):
                self.console.log_error(res["status_msg"])
                self.page_terminal.log_error(res["status_msg"])
                self.toast.show_message(res["status_msg"], "#da3633")
        elif self._verifier_dirty:
            # Tracking just ended. Clear the readouts so stale numbers are never
            # mistaken for a live measurement.
            self._verifier_dirty = False
            self._update_verifier("IDLE", "Holding position - no command in flight")

        # 3. Pre-Flight Auto-Climb Interlock for Ground Start (Decoupled threshold & 15s timeout)
        now = time.time()

        # 3a. Component health. The pump is only "live" while a path is
        # actually streaming - a deliberately stopped pump is not a fault,
        # so its status follows the timer rather than wall-clock silence.
        pump_live = (
            self.offboard_pump_timer.isActive()
            and self.path_in_progress
            and not self.path_paused
        )
        if pump_live != self._pump_was_live:
            self._pump_was_live = pump_live
            self.pump_health.set_status(
                EngineStatus.READY if pump_live else EngineStatus.STOPPED)

        # Sweep every registered component once a second. check_stall() latches,
        # so each stall is reported exactly once instead of 30x per second.
        if now - self._last_health_sweep >= 1.0:
            self._last_health_sweep = now
            for _h in get_registry().all():
                if not _h.check_stall():
                    continue
                _snap = _h.snapshot()
                _msg = f"[HEALTH] STALLED - {_snap.one_line()}"
                self.console.log_error(_msg)
                self.page_terminal.log_error(_msg)
                self.toast.show_message(
                    f"{_snap.name} stalled - no output for "
                    f"{_snap.stall_after_s:.1f}s", "#da3633", 5000)
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
        OK, WARN, BAD = PALETTE["ok"], PALETTE["warn"], PALETTE["danger"]
        DIM = PALETTE["text_dim"]

        self._diag_set("sysid", f"{t.system_id} / {t.component_id}")
        self._diag_set("connected", "CONNECTED" if t.connected else "NO LINK",
                       OK if t.connected else BAD)
        # Armed is red, not green: green would read as "safe" for the one state
        # in which the props can actually spin.
        self._diag_set("arm_state", "ARMED" if t.armed else "DISARMED",
                       BAD if t.armed else DIM)
        self._diag_set("flight_mode", t.flight_mode or "--",
                       PALETTE["accent"] if t.connected else DIM)
        self._diag_set("flight_time", f"{int(t.flight_time_sec)} s")

        self._diag_set("vio_health",
                       "LOCKED (10 Hz)" if t.d435i_vio_health else "NO VISION DATA",
                       OK if t.d435i_vio_health else BAD)
        self._diag_set("vio_age",
                       f"{t.d435i_vio_age:.1f} s ago" if t.last_vision_time else "never",
                       OK if t.d435i_vio_health else BAD)
        self._diag_set("ekf2",
                       "ACTIVE (indoor lock)" if t.ekf2_vision_fused else "WAITING",
                       OK if t.ekf2_vision_fused else WARN)

        pos_col = BAD if t.position_stale else None
        self._diag_set("pos_x", f"{t.x:+.3f} m", pos_col)
        self._diag_set("pos_y", f"{t.y:+.3f} m", pos_col)
        self._diag_set("pos_z", f"{t.z:+.3f} m", pos_col)
        self._diag_set("alt", f"{t.altitude:.3f} m", pos_col)

        self._diag_set("vx", f"{t.vx:+.2f} m/s")
        self._diag_set("vy", f"{t.vy:+.2f} m/s")
        self._diag_set("vz", f"{t.vz:+.2f} m/s")
        self._diag_set("gspeed", f"{t.ground_speed:.2f} m/s")

        # Thresholds mirror the motor widget's own legend: idle 1000-1100,
        # saturation above 1920.
        for i, key in enumerate(("m1", "m2", "m3", "m4")):
            pwm = t.motor_pwms[i] if i < len(t.motor_pwms) else 0
            col = WARN if pwm > 1920 else (OK if pwm > 1100 else DIM)
            self._diag_set(key, f"{pwm} \u00b5s", col)

        self._diag_set("roll", f"{t.roll:+.1f}\u00b0")
        self._diag_set("pitch", f"{t.pitch:+.1f}\u00b0")
        self._diag_set("yaw", f"{t.yaw:+.1f}\u00b0")
        self._diag_set("heading", f"{t.heading:.1f}\u00b0")

        batt_col = OK if t.battery_percent > 35 else (WARN if t.battery_percent > 20 else BAD)
        self._diag_set("volt", f"{t.battery_voltage:.2f} V", batt_col)
        self._diag_set("curr", f"{t.battery_current:.1f} A")
        self._diag_set("pct", f"{t.battery_percent}%", batt_col)

        # GPS is informational indoors - this airframe flies on vision - so a
        # missing fix is dimmed, not flagged red.
        fix_col = OK if t.fix_type in ("3D_FIX", "DGPS", "RTK_FLOAT", "RTK_FIXED") else DIM
        self._diag_set("sats", str(t.satellites), fix_col)
        self._diag_set("hdop", f"{t.hdop:.1f}")
        self._diag_set("fix", t.fix_type, fix_col)

    # -------------------------------------------------------------------------
    # Command Dispatch Helpers
    # -------------------------------------------------------------------------

    def _cmd_arm_from_ui(self):
        # The ARM button is always a normal arm. Bench force-arm is reachable
        # only by typing `arm force` in the flight terminal.
        self._cmd_arm(force=False)

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

        t = self.last_telemetry
        if t.is_airborne:
            # `disarm` while genuinely airborne must never be an instant
            # motor cutoff - that free-falls the vehicle. Redirect to PX4's
            # own AUTO.LAND (a real, tested controlled descent using the
            # rangefinder/optical-flow height fusion already enabled -
            # EKF2_RNG_CTRL/EKF2_OF_CTRL), and only send the real disarm once
            # landed_state confirms touchdown (see _on_telemetry_updated).
            # `kill` remains the true, unconditional emergency cutoff.
            msg = "DISARM requested while airborne - redirecting to AUTO.LAND for a safe controlled descent (will disarm automatically on touchdown). Use KILL for an immediate cutoff instead."
            self.console.log_warning(msg)
            self.page_terminal.log_warning(msg)
            self.toast.show_message("Airborne - landing safely, will disarm on touchdown", "#d29922", 5000)
            self.worker.set_mode("AUTO.LAND")
            self._pending_autodisarm_after_land = True
            self.exec_tracker.start_tracking(
                "land-then-disarm", t.x, t.y, t.z,
                cur_armed=t.armed, cur_mode=t.flight_mode
            )
            return

        self.console.log_cmd("Dispatching DISARM...")
        self.page_terminal.log_cmd("Dispatching DISARM...")
        self.worker.disarm(force=True)
        self.toast.show_message("Dispatching: DISARM", "#da3633")
        self.exec_tracker.start_tracking(
            "disarm", t.x, t.y, t.z,
            cur_armed=t.armed, cur_mode=t.flight_mode
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

        t = self.last_telemetry
        if not t.armed:
            msg = "Cannot takeoff: drone is disarmed - arm first."
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Cannot takeoff: disarmed", "#da3633", 4000)
            return
        if not (self.TAKEOFF_ALT_MIN_M <= altitude <= self.TAKEOFF_ALT_MAX_M):
            msg = (f"Cannot takeoff: {altitude:.2f}m is outside the allowed range "
                   f"[{self.TAKEOFF_ALT_MIN_M:.1f}, {self.TAKEOFF_ALT_MAX_M:.1f}]m.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Takeoff altitude out of range", "#da3633", 4000)
            return

        if t.is_airborne:
            # Already flying - "takeoff <alt>" here means "go to and hold at
            # this altitude," which may mean ascending OR descending from the
            # current height. PX4's native NAV_TAKEOFF is a ground-takeoff
            # maneuver and doesn't handle descending, so use a pure-Z OFFBOARD
            # position-hold setpoint instead (same mechanism move/yaw use),
            # which naturally auto-holds once the target altitude is reached.
            # Mode switching is never done silently on the operator's behalf -
            # OFFBOARD must already be active (set explicitly via the Mode
            # dropdown + SET MODE) before this will do anything.
            if t.flight_mode != "OFFBOARD":
                msg = (f"Cannot hold altitude: current mode is {t.flight_mode or 'UNKNOWN'}, not OFFBOARD. "
                       "Switch to OFFBOARD manually (Mode dropdown -> SET MODE) first.")
                self.console.log_error(msg)
                self.page_terminal.log_error(msg)
                self.toast.show_message("Not in OFFBOARD - switch manually first", "#da3633", 4000)
                return
            self.console.log_cmd(f"Already airborne - repositioning to altitude {altitude:.1f}m and holding...")
            self.page_terminal.log_cmd(f"Already airborne - repositioning to altitude {altitude:.1f}m and holding...")
            self.worker.move_to_waypoint(t.x, t.y, z=-abs(altitude))
            self.toast.show_message(f"Altitude hold: {altitude:.1f}m", "#1f6feb")
        else:
            self.console.log_cmd(f"Initiating takeoff to {altitude:.1f}m...")
            self.page_terminal.log_cmd(f"Initiating takeoff to {altitude:.1f}m...")
            self.worker.takeoff(altitude)
            self.toast.show_message(f"Takeoff Initiated ({altitude:.1f}m)", "#1f6feb")

        self.exec_tracker.start_tracking(
            f"takeoff {altitude}", t.x, t.y, t.z,
            cur_armed=t.armed, cur_mode=t.flight_mode
        )

    # _cmd_move_from_ui() removed with the Move (m) row: relative translation is
    # now issued from the flight terminal ("move <dx> <dy> <dz>") and by the
    # Tactical SLAM path executor, both of which call _cmd_move() directly.

    def _cmd_move(self, dx: float, dy: float, dz: float):
        if not self.worker or not self.worker.isRunning():
            self.console.log_error("Cannot move: Disconnected")
            self.page_terminal.log_error("Cannot move: Disconnected")
            return
        if not self.last_telemetry.is_airborne:
            msg = "Cannot move: not airborne - arm and takeoff first."
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Cannot move: not airborne", "#da3633", 4000)
            return
        if max(abs(dx), abs(dy), abs(dz)) > self.MOVE_MAX_DELTA_M:
            msg = f"Cannot move: displacement exceeds the {self.MOVE_MAX_DELTA_M:.1f}m safety bound per command."
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Move rejected: too large", "#da3633", 4000)
            return

        # OFFBOARD is never engaged silently on the operator's behalf - it
        # must already be active (Mode dropdown -> SET MODE) before a move
        # will be dispatched.
        if self.last_telemetry.flight_mode != "OFFBOARD":
            curr_mode = self.last_telemetry.flight_mode or "UNKNOWN"
            msg = (f"Cannot move: current mode is {curr_mode}, not OFFBOARD. "
                   "Switch to OFFBOARD manually (Mode dropdown -> SET MODE) first.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Not in OFFBOARD - switch manually first", "#da3633", 4000)
            return

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
        if not self.last_telemetry.is_airborne:
            msg = "Cannot rotate yaw: not airborne - arm and takeoff first."
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Cannot yaw: not airborne", "#da3633", 4000)
            return

        # OFFBOARD is never engaged silently on the operator's behalf - it
        # must already be active (Mode dropdown -> SET MODE) before a yaw
        # rotation will be dispatched.
        if self.last_telemetry.flight_mode != "OFFBOARD":
            curr_mode = self.last_telemetry.flight_mode or "UNKNOWN"
            msg = (f"Cannot rotate yaw: current mode is {curr_mode}, not OFFBOARD. "
                   "Switch to OFFBOARD manually (Mode dropdown -> SET MODE) first.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Not in OFFBOARD - switch manually first", "#da3633", 4000)
            return

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

        # 1b. OFFBOARD Mode Interlock - never engaged silently on the
        # operator's behalf; must already be active before a path is flown.
        if self.last_telemetry.flight_mode != "OFFBOARD":
            curr_mode = self.last_telemetry.flight_mode or "UNKNOWN"
            msg = (f"Cannot execute path: current mode is {curr_mode}, not OFFBOARD. "
                   "Switch to OFFBOARD manually (Mode dropdown -> SET MODE) first.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Not in OFFBOARD - switch manually first", "#da3633", 4000)
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
            "waypoint W1", self.last_telemetry.x, self.last_telemetry.y, self.last_telemetry.z,
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
            _t0 = time.perf_counter()
            self.worker.move_to_waypoint(wpt[0], wpt[1], z=self.cruise_z, yaw_deg=self.current_target_yaw)
            # Heartbeat only on a setpoint that actually went out. Timing the
            # send matters as much as counting it: this runs on the GUI thread,
            # so a socket that starts blocking here stalls the pump AND the UI
            # together, and PX4 notices within 500 ms.
            self.pump_health.record_latency((time.perf_counter() - _t0) * 1000.0)
            self.pump_health.heartbeat()

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

        # 3. OFFBOARD Mode Interlock - PAUSE puts PX4 into AUTO.LOITER; mode
        # is never re-engaged silently, so the operator must switch back to
        # OFFBOARD manually (Mode dropdown -> SET MODE) before resuming.
        if self.last_telemetry.flight_mode != "OFFBOARD":
            curr_mode = self.last_telemetry.flight_mode or "UNKNOWN"
            msg = (f"Cannot resume path: current mode is {curr_mode}, not OFFBOARD. "
                   "Switch to OFFBOARD manually (Mode dropdown -> SET MODE) first.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Not in OFFBOARD - switch manually first", "#da3633", 4000)
            return

        self.console.log_cmd("[RESUME] Resuming path...")
        self.page_terminal.log_cmd("[RESUME] Resuming path...")
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
            force = len(parts) > 1 and parts[1].lower() == "force"
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
        # A Qt.Tool window is not a child in the window-manager sense; without
        # this it outlives the main window as an orphan always-on-top frame.
        self.fpv_float.close()
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
