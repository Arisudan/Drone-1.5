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
import argparse
import os
import sys
import time
from typing import Optional, List, Tuple

import numpy as np

from qt_env import pin_system_qt_plugins

# Auto-configure ROS 2 Jazzy dynamic library path before any C-extensions load
if sys.platform.startswith("linux"):
    if "QT_QPA_PLATFORM" not in os.environ:
        os.environ["QT_QPA_PLATFORM"] = "xcb"
    pin_system_qt_plugins()

    # If ROS 2 Jazzy libraries are not in LD_LIBRARY_PATH, re-exec once.
    # Only when launched as a script: the dynamic loader reads LD_LIBRARY_PATH
    # at process start, so the fix needs a fresh process - but doing that on
    # *import* replaced whatever process imported this module. Under
    # `python -m unittest` sys.argv[0] is the literal "python -m unittest",
    # so the re-exec tried to run a file by that name and killed the test run
    # on any machine with ROS 2 installed but not sourced.
    ros_ld = "/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib"
    curr_ld = os.environ.get("LD_LIBRARY_PATH", "")
    if (__name__ == "__main__"
            and "/opt/ros/jazzy/lib" not in curr_ld
            and os.path.exists("/opt/ros/jazzy/lib")):
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
    QSplitter, QProgressBar, QLineEdit, QComboBox,
    QGroupBox, QScrollArea
)

from core.telemetry import TelemetrySnapshot
from core.health import EngineHealth, EngineStatus, get_registry
from core.execution_tracker import ExecutionTracker
from core.path_planner import AStarPathPlanner
from core.planner_worker import PlannerWorker
from protocol.mavlink_worker import MAVLinkWorker
from protocol.ros2_map_listener import ROS2MapListener, SlamMapResetWorker
from ui.styles import build_stylesheet
from ui.scaling import init_scale, enable_high_dpi, px
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
from ui.value_grid import ValueGridWidget
from ui.guided_confirm import GuidedConfirmBar
from ui.shortcuts import install_shortcuts, ShortcutHelpOverlay
from core.flight_log import FlightLogger
from core.audio import AudioAlerts, Severity
from core.settings import load_settings, apply_overrides, resolve_rviz_config, save_settings


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

    def __init__(self, settings=None):
        super().__init__()
        self.setWindowTitle("Drone-GCS | Autonomous Industrial Ground Station")
        # Window geometry follows the UI scale: on a display that needs 1.5x
        # text, a 1400px window holds two-thirds of the content it was sized for.
        # Both the opening size and the floor are then clamped to the screen -
        # scaling the floor without clamping it makes the station unusable on
        # exactly the machines that need scaling most. At 1.4x the raw floor is
        # 1708x980, which no 1600x900 or 1366x768 laptop can satisfy: Qt would
        # hold the window larger than the display and put the command column
        # off-screen with no way to reach it.
        avail = self._available_screen_size()
        self.resize(min(px(1400), avail[0]), min(px(860), avail[1]))
        # Lowered from 1150x740: that floor didn't actually fit common smaller laptop
        # panels (1366x768 and especially 1280x720) once OS window-chrome/taskbar space
        # is subtracted, forcing clipping instead of a graceful shrink. 1024x700 still
        # comfortably fits every control - it was headroom, not a real usability need.
        self.setMinimumSize(min(px(1220), avail[0]), min(px(700), avail[1]))

        # Persisted settings and the flight recorder are built before the UI,
        # because the Config and Logs workspaces are views onto them.
        self.settings = settings if settings is not None else apply_overrides(load_settings())
        self.flight_logger = FlightLogger()

        # Audio alerts. Constructed before the UI so the header's mute control
        # can reflect whether a backend was actually found, rather than offering
        # to mute something that was never going to make a sound.
        self.audio = AudioAlerts(self.settings.audio)

        # Edge-trigger memory for the audio alerts. Every one of these exists to
        # answer "has this just changed?" rather than "is this true?" - a
        # condition that is continuously true must sound once, not every tick.
        self._alert_prev_armed: Optional[bool] = None
        self._alert_prev_mode: Optional[str] = None
        self._alert_prev_connected: Optional[bool] = None
        self._alert_prev_batt_band: Optional[str] = None
        self._alert_prev_vio_ok: Optional[bool] = None

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

        # One background thread for every A* search, path collision check and
        # map score in the station. All three used to run on the GUI thread -
        # 42 to 334 ms for a plan, 34 ms per collision check five times a
        # second - on the same thread as the 10 Hz OFFBOARD setpoint pump,
        # whose PX4 deadline is 500 ms. See core/planner_worker.py.
        self.planner_worker = PlannerWorker(self.path_planner, parent=self)
        self.planner_worker.collision_ready.connect(self._on_collision_result)
        # Detour searches come back on the same signal the SLAM tab uses for
        # goal planning; each handler ignores tokens that are not its own.
        self.planner_worker.plan_ready.connect(self._on_detour_ready)
        self.planner_worker.start()

        # Token of the collision check we are waiting for, and the path
        # generation it belongs to. A verdict about a path that has since been
        # replaced, aborted or completed must never halt the current one.
        self._collision_token: Optional[int] = None
        self._collision_generation: int = -1
        self._detour_token: Optional[int] = None
        self._detour_generation: int = -1
        self._path_generation: int = 0
        self._collision_requested_at: float = 0.0

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
        # Link throughput, cached as display text for the value grid.
        self._rx_rate_text: str = "--"
        self._tx_rate_text: str = "--"
        # Mission progress bookkeeping (see _update_mission_progress).
        self._progress_route_key: Optional[tuple] = None

        # Build Industrial UI Shell
        self._init_ui()

        # Keyboard shortcuts. Held on the instance because a QShortcut that gets
        # garbage collected stops working without any error.
        self._shortcuts = install_shortcuts(self, self._shortcut_callbacks())
        self._help_overlay: Optional[ShortcutHelpOverlay] = None

        # Toast manager
        self.toast = NotificationToast(self)

        # Periodic UI refresh timer (30 Hz)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._on_ui_tick)
        self.ui_timer.start(33)

        # Background ROS 2 Map Listener for live /map_thin (with automatic TCP fallback)
        self.map_listener = ROS2MapListener(
            tcp_host=self.settings.video.map_bridge_host,
            tcp_port=self.settings.video.map_bridge_port,
            parent=self,
            stall_after_s=self.settings.alerts.map_stall_s)
        self.map_listener.map_received.connect(self._on_map_received_dispatch)
        self.map_listener.status_updated.connect(self._on_ros2_status_updated)
        self.map_listener.start()

        # One capture thread, three viewports: the FPV tab's own label, the
        # cockpit's centre panel, and the draggable SLAM overlay.
        self.fpv_float = FloatingVideoWindow(self)
        self.page_fpv.frame_broadcast.connect(self.hud.video.on_frame)
        self.page_fpv.frame_broadcast.connect(self.fpv_float.sink.on_frame)

        # Auto-connect to default autopilot endpoint on launch (UDP 14550)
        conn = self.settings.connection
        port = conn.udp_port if conn.protocol == "udp" else conn.tcp_port
        self._connect_to_endpoint(conn.host, port, protocol=conn.protocol)

    @staticmethod
    def _available_screen_size() -> Tuple[int, int]:
        """Usable desktop area, minus a little for window chrome and panels.

        Falls back to a conservative 1280x720 when Qt cannot report a screen
        (offscreen rendering, a headless test), which is small enough to be a
        safe floor and large enough that the layout still resolves.
        """
        try:
            from PyQt5.QtWidgets import QApplication as _QApp
            screen = _QApp.primaryScreen()
            if screen is not None:
                geo = screen.availableGeometry()
                if geo.width() > 0 and geo.height() > 0:
                    return max(640, geo.width() - 40), max(480, geo.height() - 80)
        except Exception:
            pass
        return 1280, 720

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
        self.top_strip.mute_toggled.connect(self._on_mute_toggled)
        # Reflect whether a backend was found at all, so the control is not
        # offered on a machine where it would do nothing.
        self.top_strip.set_audio_state(self.audio.muted, self.audio.available)
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
        self.page_fpv.txt_url.setText(self.settings.video.stream_url)
        self.stack.addWidget(self.page_fpv)

        # Page 2: Tactical 2D SLAM & Waypoint Stager
        self.page_slam = SLAMMapWidget(self, rviz_config=resolve_rviz_config(self.settings))
        self.page_slam.execute_path_requested.connect(self._on_execute_path_requested)
        self.page_slam.pause_path_requested.connect(self._on_pause_path_requested)
        self.page_slam.resume_path_requested.connect(self._on_resume_path_requested)
        self.page_slam.abort_path_requested.connect(self._on_abort_path_requested)
        self.page_slam.altitude_changed.connect(self._on_cruise_altitude_changed)
        self.page_slam.reset_map_requested.connect(self._on_reset_map_requested)
        self.page_slam.fly_here_requested.connect(self._on_fly_here_requested)
        self.page_slam.attach_planner_worker(self.planner_worker)
        self.page_slam.set_map_stale_after(self.settings.alerts.map_stall_s)
        self.stack.addWidget(self.page_slam)
        self._reset_map_worker: Optional[SlamMapResetWorker] = None

        # Page 3: Live Motor & Actuator Telemetry
        self.page_motors = MotorWidget(self)
        self.page_motors.motor_test_requested.connect(self._on_motor_test_requested)
        self.page_motors.motor_test_stop_requested.connect(self._on_motor_test_stop)
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
        self.btn_arm.setMinimumHeight(px(34))
        self.btn_arm.clicked.connect(self._request_arm)
        actions_box.addWidget(self.btn_arm, 1)

        self.btn_disarm = QPushButton("DISARM", self)
        self.btn_disarm.setObjectName("btnDisarm")
        self.btn_disarm.setMinimumHeight(px(34))
        self.btn_disarm.clicked.connect(self._request_disarm)
        actions_box.addWidget(self.btn_disarm, 1)

        # HOLD and LAND move the aircraft, so they carry the navigation colour
        # rather than a decorative one - LAND in particular used to be purple,
        # which signalled nothing about what it does.
        self.btn_hold = QPushButton("HOLD", self)
        self.btn_hold.setObjectName("btnNav")
        self.btn_hold.setMinimumHeight(px(34))
        self.btn_hold.clicked.connect(lambda: self._cmd_mode("AUTO.LOITER"))
        actions_box.addWidget(self.btn_hold, 1)

        self.btn_land = QPushButton("LAND", self)
        self.btn_land.setObjectName("btnNav")
        self.btn_land.setMinimumHeight(px(34))
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

        # Scaled: left as raw pixels these two capped the field and the
        # dispatch button at their 96 DPI widths while the text inside them
        # grew, which at 1.35x pushed the NAVIGATION rows into each other.
        FIELD_W = px(70)       # numeric entry
        ACTION_W = px(120)     # dispatch button

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
        self.btn_takeoff.clicked.connect(self._request_takeoff)

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
        self.btn_yaw.clicked.connect(self._request_yaw)

        # Change altitude in flight. Previously the only way to change height
        # once airborne was to re-fly a path or land - the takeoff field sets an
        # altitude for a takeoff, not for a vehicle already in the air.
        lbl_alt = QLabel("Change Alt:", self)
        lbl_alt.setObjectName("fieldLabel")
        gn.addWidget(lbl_alt, 2, 0)

        self.lbl_current_alt = QLabel("-- m AGL", self)
        self.lbl_current_alt.setObjectName("valueMono")
        gn.addWidget(self.lbl_current_alt, 2, 1, Qt.AlignLeft)

        self.btn_change_alt = QPushButton("CHANGE ALT", self)
        self.btn_change_alt.setObjectName("btnNav")
        self.btn_change_alt.setFixedWidth(ACTION_W)
        self.btn_change_alt.setToolTip(
            "Hold position and climb or descend to a new altitude (requires OFFBOARD)")
        gn.addWidget(self.btn_change_alt, 2, 2)
        self.btn_change_alt.clicked.connect(self._request_change_alt)

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
        self.combo_modes.setMinimumHeight(px(28))
        gm.addWidget(self.combo_modes)

        # Neutral on purpose: a mode change is routine, and colouring it would
        # dilute the three colours that do mean something here.
        self.btn_set_mode = QPushButton("SET MODE", self)
        self.btn_set_mode.setMinimumHeight(px(28))
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
        self.btn_kill.setMinimumHeight(px(32))
        self.btn_kill.clicked.connect(self._request_kill)
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
        self.lbl_exec_state.setFixedWidth(px(84))
        self.lbl_exec_state.setAlignment(Qt.AlignCenter)
        state_row.addWidget(self.lbl_exec_state)

        self.lbl_exec_val = QLabel("Holding position - no command in flight", self)
        self.lbl_exec_val.setObjectName("execMessage")
        state_row.addWidget(self.lbl_exec_val, 1)

        self.lbl_exec_elapsed = QLabel("--", self)
        self.lbl_exec_elapsed.setObjectName("valueMono")
        self.lbl_exec_elapsed.setFixedWidth(px(52))
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
        self.progress_verif.setFixedHeight(px(14))
        self.progress_verif.setTextVisible(True)
        self.progress_verif.setFormat("no displacement target")
        self.progress_verif.setValue(0)
        vl.addWidget(self.progress_verif)

        rl.addWidget(grp_verif)

        # ---- Guided action confirmation ----
        # Hidden until an action needs confirming. It sits directly above the
        # console, in the column the operator is already looking at when they
        # press a command button, rather than as a modal over the instruments.
        self.confirm_bar = GuidedConfirmBar(self)
        self.confirm_bar.confirmed.connect(self._on_guided_confirmed)
        self.confirm_bar.cancelled.connect(self._on_guided_cancelled)
        rl.addWidget(self.confirm_bar)

        # ---- Interactive Terminal / Console Bar ----
        self.console = CLIConsoleWidget(self)
        self.console.command_submitted.connect(self._execute_cli_command)
        # A floor, because inside the scroll area a stretch factor alone would
        # let the console collapse to nothing on a short window.
        self.console.setMinimumHeight(px(150))
        rl.addWidget(self.console, 1)

        # The command column scrolls rather than compressing. Every control in
        # it has a natural height, and when the window is short - a small screen,
        # or a large UI scale, or both - a QVBoxLayout that cannot honour those
        # heights does not shrink them, it lets the children overlap. Measured:
        # at 1.35x in a 1220x700 window, TAKEOFF, ROTATE YAW and CHANGE ALT were
        # drawn on top of each other and their labels collapsed to zero height.
        # Overlapping controls on a panel that arms and flies an aircraft is not
        # an acceptable failure mode; a scrollbar is.
        right_scroll = QScrollArea(self)
        right_scroll.setWidgetResizable(True)
        right_scroll.setFrameShape(QFrame.NoFrame)
        right_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        right_scroll.setWidget(right_panel)

        splitter.addWidget(right_scroll)
        # 7:3 rather than 3:2 - the command column only ever holds fixed-height
        # controls, so the extra width went to padding while the camera pane
        # could use it.
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        right_scroll.setMaximumWidth(px(660))
        # Below this the NAVIGATION grid's label / field / button triple cannot
        # lay out side by side, so the splitter is not allowed to go under it.
        right_scroll.setMinimumWidth(px(430))

        layout.addWidget(splitter)
        return page

    def _build_diagnostics_page(self) -> QWidget:
        """Page 4: the operator-configurable telemetry value grid.

        This replaced nine hardcoded cards holding thirty fixed fields. The
        fields were a reasonable default and are still the default - see
        ui.value_grid.DEFAULT_FIELDS, which is exactly that original set in the
        original reading order - but which thirty numbers matter depends on the
        session, and previously changing them meant editing this method.

        The saved layout wins over the default, so an operator who has tuned
        this page keeps it across upgrades that change DEFAULT_FIELDS.
        """
        cfg = self.settings.ui
        self.value_grid = ValueGridWidget(
            fields=cfg.value_grid_fields or None,
            columns=cfg.value_grid_columns,
            font_scale=cfg.value_grid_font_scale,
            parent=self)
        self.value_grid.layout_changed.connect(self._on_value_grid_changed)
        return self.value_grid

    def _on_value_grid_changed(self, keys: list, columns: int,
                               font_scale: float) -> None:
        """Persist the grid layout as soon as it is edited.

        Written immediately rather than on exit: this is a preference the
        operator just expressed by hand, and losing it to a crash - on a station
        whose whole job is talking to hardware that sometimes wedges the app -
        would teach them not to bother arranging it.
        """
        self.settings.ui.value_grid_fields = list(keys)
        self.settings.ui.value_grid_columns = int(columns)
        self.settings.ui.value_grid_font_scale = float(font_scale)
        try:
            save_settings(self.settings)
        except (ValueError, OSError) as exc:
            # A settings file that cannot be written is worth reporting but is
            # not worth interrupting a flight over; the grid still works for
            # this session.
            self.console.log_warning(f"Could not save value grid layout: {exc}")

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

    def _grid_extras(self) -> dict:
        """Station-measured values the vehicle does not report.

        Everything the *vehicle* sends is formatted by the value grid's own
        field registry straight off the telemetry snapshot. These four are
        different in kind - they describe the ground station and its link - so
        they are handed in rather than derived.
        """
        return {
            "rx": self._rx_rate_text,
            "tx": self._tx_rate_text,
            "map_source": self.page_slam.map_source or "none",
            "audio": ("MUTED" if self.audio.muted else
                      ("ON" if self.audio.available else "UNAVAILABLE")),
        }

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
        # Cached as text rather than pushed straight at a label: the value grid
        # may or may not currently be showing them, and it reads its own values
        # once per tick.
        self._rx_rate_text = f"{rx_rate:.1f} msg/s"
        self._tx_rate_text = f"{tx_rate:.1f} msg/s"

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
            # Keyed on the message text, so one unresolved fault that PX4
            # re-reports every two seconds is spoken once per rate-limit window
            # rather than continuously. The worker already collapses identical
            # STATUSTEXT storms; this is the second line of defence.
            self.audio.say(clean_text, Severity.from_mavlink(severity),
                           key=f"status:{clean_text[:40]}")
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
        self.audio.set_config(cfg.audio)
        msg = ("Settings saved. Command limits, audio and stream URL applied "
               "now; connection and UI scale changes take effect on the next "
               "start.")
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

        if result_code != 0:
            # Only the rejections are sounded. An accepted command already
            # announces itself by the aircraft doing what was asked, and a tone
            # on every ACK would cover the telemetry stream in beeps.
            self.audio.say(f"{cmd_name} rejected", Severity.WARN,
                           key=f"ack:{cmd_id}")

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

    def _on_map_received_dispatch(self, grid: np.ndarray, res: float, ox: float,
                                  oy: float, source: str, image=None):
        self.latest_map_data = (grid, res, ox, oy)
        self.page_slam.on_ros2_map_received(grid, res, ox, oy, source, image)

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
                self.audio.say("Command did not execute", Severity.WARN,
                               key="exec_failed")
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
                self.audio.say(f"{_snap.name} stalled", Severity.CRITICAL,
                               key=f"stall:{_snap.name}")
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
                    self.audio.say("Climb timeout, holding", Severity.CRITICAL, key="climb")
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

        # 5. Live collision avoidance on the active path.
        #
        # This only *asks*. The check itself costs ~34 ms on a 25 x 25 m grid
        # and used to run right here, five times a second, on the GUI thread -
        # 170 ms of every second blocked, on the same thread as the OFFBOARD
        # setpoint pump that PX4 drops the aircraft out of OFFBOARD for if it
        # goes quiet for 500 ms. The verdict now arrives in
        # _on_collision_result. Only one request is ever outstanding: asking
        # again while the last answer is still coming would queue up verdicts
        # about where the vehicle used to be.
        if (
            self.path_in_progress
            and self.active_waypoints
            and self.latest_map_data is not None
            and self._collision_token is None
            and (now - self.last_collision_check_time > 0.20)
        ):
            self.last_collision_check_time = now
            self._collision_requested_at = now
            grid, res, ox, oy = self.latest_map_data
            remaining_wpts = self.active_waypoints[self.current_wpt_idx:]
            self._collision_generation = self._path_generation
            self._collision_token = self.planner_worker.request_collision_check(
                grid, res, ox, oy, remaining_wpts, (t.x, t.y), lookahead_m=1.5)

        # 6. Sequential Waypoint Navigation along Planned A* Path with Tangent Yaw
        if self.path_in_progress and self.active_waypoints and self.worker:
            curr_target = self.active_waypoints[self.current_wpt_idx]
            dist_to_wpt = math.sqrt((curr_target[0] - t.x)**2 + (curr_target[1] - t.y)**2)

            if dist_to_wpt < 0.25:
                # Reached waypoint
                self.console.log_success(
                    f"Reached Waypoint W{self.current_wpt_idx+1} ({curr_target[0]:+.2f}, {curr_target[1]:+.2f})"
                )
                self.audio.alert(Severity.INFO, key=None)
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
                    self._begin_path_generation()
                    self.offboard_pump_timer.stop()
                    self.page_slam.set_executing_state(False, paused=False)
                    self.console.log_success("MISSION COMPLETE: Drone arrived at final Goal Pose!")
                    self.page_terminal.log_success("MISSION COMPLETE: Drone arrived at final Goal Pose!")
                    self.toast.show_message("Goal Pose Reached!", "#238636", 5000)
                    self.page_slam.progress.mark_complete()
                    self.audio.say("Goal reached", Severity.OK, key="mission")

        # 5. Diagnostics workspace: one call, because which values are on
        # screen is now the operator's choice rather than this method's.
        self.value_grid.update_values(t, self._grid_extras())

        # 6. Current altitude, next to the CHANGE ALT control it is the
        # starting point for.
        self.lbl_current_alt.setText(f"{t.altitude:.2f} m AGL")

        # 7. Mission progress strip and the motor-test interlocks, both of
        # which are pure functions of state already computed above.
        self._update_mission_progress(t)
        self.page_motors.set_vehicle_state(
            connected=t.connected, armed=t.armed, airborne=t.is_airborne)

        # 8. Audio alerts for state transitions.
        self._update_audio_alerts(t)

    # -------------------------------------------------------------------------
    # Command Dispatch Helpers
    # -------------------------------------------------------------------------

    # -------------------------------------------------------------------------
    # Guided Action Confirmation  (ui/guided_confirm.GuidedConfirmBar)
    # -------------------------------------------------------------------------

    def _request_arm(self):
        """ARM used to dispatch on a single click, with no confirmation at all -
        despite this module's own docstring claiming a dual-stage modal. It now
        goes through the same slide-to-confirm gesture as every other guarded
        action. Bench force-arm remains terminal-only (`arm force`)."""
        if not self._require_link("arm"):
            return
        self.confirm_bar.request(
            "arm", "ARM MOTORS", danger=True,
            detail="Propellers will be live. Confirm the area is clear.",
            confirm_text="Slide to arm")

    def _request_disarm(self):
        """Disarm. Airborne, this is redirected to AUTO.LAND by _cmd_disarm -
        the confirmation says so, because 'disarm' and 'land' are different
        enough that the operator should know which one they are getting."""
        if not self._require_link("disarm"):
            return
        if self.last_telemetry.is_airborne:
            self.confirm_bar.request(
                "disarm", "DISARM (AIRBORNE)", danger=True,
                detail="Vehicle is airborne: this commands AUTO.LAND and "
                       "disarms on touchdown. Use EMERGENCY KILL for an "
                       "immediate cutoff.",
                confirm_text="Slide to land and disarm")
        else:
            self.confirm_bar.request(
                "disarm", "DISARM", danger=True,
                detail="Vehicle is on the ground.",
                confirm_text="Slide to disarm")

    def _request_takeoff(self):
        """Takeoff, with the altitude on a slider bounded by settings.limits."""
        if not self._require_link("takeoff"):
            return
        try:
            seed = float(self.ent_takeoff_alt.text().strip())
        except ValueError:
            seed = self.settings.slam.cruise_altitude_m
        seed = max(self.TAKEOFF_ALT_MIN_M, min(self.TAKEOFF_ALT_MAX_M, seed))
        self.confirm_bar.request(
            "takeoff", "TAKEOFF", value_label="Altitude AGL",
            vmin=self.TAKEOFF_ALT_MIN_M, vmax=self.TAKEOFF_ALT_MAX_M,
            vinit=seed, unit="m", step=0.1,
            detail="Vehicle must already be armed.",
            confirm_text="Slide to take off")

    def _request_yaw(self):
        if not self._require_link("rotate yaw"):
            return
        try:
            seed = float(self.ent_yaw.text().strip())
        except ValueError:
            seed = 90.0
        self.confirm_bar.request(
            "yaw", "ROTATE YAW", value_label="Rotation",
            vmin=-180.0, vmax=180.0, vinit=max(-180.0, min(180.0, seed)),
            unit="\u00b0", step=5.0,
            detail="Relative rotation from the current heading. Requires "
                   "OFFBOARD and an airborne vehicle.",
            confirm_text="Slide to rotate")

    def _request_change_alt(self):
        """Climb or descend in place to a new altitude."""
        if not self._require_link("change altitude"):
            return
        current = abs(self.last_telemetry.altitude)
        seed = max(self.TAKEOFF_ALT_MIN_M, min(self.TAKEOFF_ALT_MAX_M,
                                               current if current > 0.05 else 1.0))
        self.confirm_bar.request(
            "change_alt", "CHANGE ALTITUDE", value_label="Target AGL",
            vmin=self.TAKEOFF_ALT_MIN_M, vmax=self.TAKEOFF_ALT_MAX_M,
            vinit=seed, unit="m", step=0.1,
            detail=f"Currently {current:.2f} m AGL. Holds the present position "
                   f"and changes height only. Requires OFFBOARD.",
            confirm_text="Slide to change altitude")

    def _request_kill(self):
        """Emergency kill. The confirmation is a drag, not a modal with a
        default button one Return keypress away."""
        self.confirm_bar.request(
            "kill", "EMERGENCY MOTOR KILL", danger=True,
            detail="Cuts all motor outputs immediately. If the vehicle is "
                   "airborne it will fall. There is no recovery from this.",
            confirm_text="Slide to cut motors")

    def _request_abort_path(self):
        if not self.path_in_progress and not self.path_paused:
            self.console.log_info("No path is executing - nothing to abort.")
            return
        self.confirm_bar.request(
            "abort_path", "ABORT PATH", danger=True,
            detail="Stops the path and commands AUTO.LAND.",
            confirm_text="Slide to abort and land")

    def _require_link(self, what: str) -> bool:
        """Refuse to even offer a confirmation with no link. Showing a confirm
        bar for a command that cannot be sent trains the operator to confirm
        things that do nothing."""
        if self.worker and self.worker.isRunning():
            return True
        msg = f"Cannot {what}: not connected to vehicle"
        self.console.log_error(msg)
        self.page_terminal.log_error(msg)
        self.toast.show_message(msg, "#da3633")
        return False

    def _on_guided_confirmed(self, action: str, value: float):
        """Single dispatch point for every confirmed guided action.

        Each branch calls the same _cmd_* method the direct control always
        called, so every interlock in those methods still applies. Confirmation
        gates the request; it does not replace the checks.
        """
        if action == "arm":
            self._cmd_arm(force=False)
        elif action == "disarm":
            self._cmd_disarm()
        elif action == "takeoff":
            self.ent_takeoff_alt.setText(f"{value:.2f}")
            self._cmd_takeoff(value)
        elif action == "yaw":
            self.ent_yaw.setText(f"{value:.1f}")
            self._cmd_yaw(value)
        elif action == "change_alt":
            self._cmd_change_altitude(value)
        elif action == "kill":
            self._cmd_kill()
        elif action == "abort_path":
            self._on_abort_path_requested()

    def _on_guided_cancelled(self, action: str):
        self.console.log_info(f"{action.replace('_', ' ').upper()} cancelled.")

    def _cmd_change_altitude(self, altitude_m: float):
        """Hold the current horizontal position and move to a new altitude.

        Uses the same OFFBOARD setpoint path as a waypoint, with the north/east
        components pinned to where the vehicle already is, so it is a pure climb
        or descent rather than a move that happens to change height.
        """
        if not self._require_link("change altitude"):
            return
        t = self.last_telemetry
        if not t.is_airborne:
            msg = "Cannot change altitude: not airborne - arm and take off first."
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Cannot change alt: not airborne", "#da3633", 4000)
            return
        if t.flight_mode != "OFFBOARD":
            msg = (f"Cannot change altitude: current mode is "
                   f"{t.flight_mode or 'UNKNOWN'}, not OFFBOARD. Switch to "
                   "OFFBOARD manually (Mode dropdown -> SET MODE) first.")
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message("Not in OFFBOARD - switch manually first",
                                    "#da3633", 4000)
            return

        target_z = -abs(altitude_m)
        self.cruise_z = target_z
        self.console.log_cmd(
            f"Changing altitude to {abs(target_z):.2f} m AGL "
            f"(holding N {t.x:+.2f}, E {t.y:+.2f})...")
        self.page_terminal.log_cmd(f"Changing altitude to {abs(target_z):.2f} m AGL...")
        self.worker.move_to_waypoint(t.x, t.y, z=target_z, yaw_deg=t.heading)
        self.offboard_pump_timer.start()
        self.toast.show_message(f"Altitude -> {abs(target_z):.2f} m", "#1f6feb")
        self.exec_tracker.start_tracking(
            f"altitude {abs(target_z):.2f}m", t.x, t.y, t.z,
            cur_armed=t.armed, cur_mode=t.flight_mode,
            target_dist=abs(target_z - t.z))

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
        self._begin_path_generation()
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
        """Execute the kill. Confirmation is the caller's job.

        The Yes/No QMessageBox that used to live here is gone: its default
        button was one Return keypress from cutting the motors of a flying
        aircraft. _request_kill now gates this behind a deliberate drag. The
        terminal's `kill` command reaches this directly, which is the documented
        behaviour of a typed emergency command.
        """
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
        self.page_slam.progress.set_state("ABORTED")
        self.page_motors.stop_motor_tests()

        if self.worker and self.worker.isRunning():
            self.worker.emergency_kill()
            self.console.log_error("!!! EMERGENCY MOTOR KILL EXECUTED !!!")
            self.page_terminal.log_error("!!! EMERGENCY MOTOR KILL EXECUTED !!!")
            self.toast.show_message("EMERGENCY KILL SENT", "#da3633", 5000)
            self.audio.say("Emergency kill", Severity.ALARM, key="kill")

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
        self._begin_path_generation()
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

    def _begin_path_generation(self) -> int:
        """Start a new path 'generation'.

        Collision verdicts and detour plans are computed on a background thread
        and can land after the path they were about has gone away. Each carries
        the generation it was requested under; a mismatch means the answer is
        about a path that no longer exists, and acting on it could halt a good
        path over an obstacle that is no longer ahead of the aircraft.
        """
        self._path_generation += 1
        self._collision_token = None
        self._detour_token = None
        return self._path_generation

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
        self._begin_path_generation()
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

    # -------------------------------------------------------------------------
    # Live collision avoidance  (answers from core/planner_worker.PlannerWorker)
    # -------------------------------------------------------------------------

    def _on_collision_result(self, token: int, is_blocked: bool,
                             collision_point, distance_m: float) -> None:
        """Act on a collision verdict, if it is still about the current path.

        The generation check is the important part. A verdict computed against
        a path that has since been aborted, completed or replaced by a detour
        would otherwise halt a perfectly good new path on the strength of an
        obstacle that is no longer in front of the aircraft.
        """
        if token != self._collision_token:
            return
        self._collision_token = None

        if getattr(self, "_collision_generation", None) != self._path_generation:
            return
        if not self.path_in_progress or not self.active_waypoints:
            return
        if not is_blocked:
            return

        now = time.time()
        t = self.last_telemetry
        self.worker.set_mode("AUTO.LOITER")
        self.path_in_progress = False
        self.path_paused = True
        self.loiter_pause_start_time = now
        self._loiter_warned = False
        self.page_slam.set_executing_state(False, paused=True)

        self.console.log_error(
            f"🚨 COLLISION ALERT: Obstacle detected {distance_m:.2f}m ahead on "
            "flight path! Halting in AUTO.LOITER.")
        self.page_terminal.log_error(
            f"🚨 COLLISION ALERT: Obstacle {distance_m:.2f}m ahead! Switched to AUTO.LOITER.")
        self.toast.show_message(
            f"Obstacle Ahead ({distance_m:.2f}m)! Drone Halted", "#da3633", 6000)
        self.audio.say("Obstacle ahead, holding", Severity.ALARM, key="collision")

        # Look for a way round, also off this thread. The aircraft is already
        # holding, so the answer can take as long as it takes.
        if self.latest_map_data is None:
            return
        grid, res, ox, oy = self.latest_map_data
        final_goal = self.active_waypoints[-1]
        self._detour_generation = self._path_generation
        self._detour_token = self.planner_worker.request_plan(
            grid, res, ox, oy, (t.x, t.y), final_goal)
        self.console.log_info("Searching for a detour around the obstacle...")

    def _on_detour_ready(self, token: int, result) -> None:
        """Adopt a detour, if it is still wanted."""
        if token != getattr(self, "_detour_token", None):
            return
        self._detour_token = None
        if getattr(self, "_detour_generation", None) != self._path_generation:
            return
        if result and result.get("success"):
            waypoints = result["waypoints"]
            self.console.log_success(
                f"🔄 DETOUR READY: Clear path found ({len(waypoints)} WPTs, "
                f"{result['total_distance_m']:.2f}m). Press RESUME to fly it.")
            self.page_slam.canvas.planned_waypoints = waypoints
            self.page_slam.canvas.update()
            self.active_waypoints = list(waypoints)
            self.current_wpt_idx = 0
            self._progress_route_key = None      # the route changed; re-seed
        else:
            self.console.log_warning(
                "No clear detour found. Maintain hold or command manual RTL / LAND.")

    # -------------------------------------------------------------------------
    # Mission Progress  (ui/mission_progress.MissionProgressBar)
    # -------------------------------------------------------------------------

    def _progress_state(self) -> str:
        """Translate the window's navigation flags into one operator-facing word."""
        if self.path_awaiting_climb:
            return "CLIMBING"
        if self.path_paused:
            # The collision handler pauses and leaves path_in_progress False,
            # which is what distinguishes an obstacle hold from an operator
            # pause - worth showing differently, because one of them means
            # something is in the way.
            return "PAUSED" if self.path_in_progress else "OBSTACLE HOLD"
        if self.path_in_progress:
            return "EN ROUTE"
        return "STAGED"

    def _update_mission_progress(self, t: TelemetrySnapshot) -> None:
        """Feed the route strip from state the tick has already computed."""
        strip = self.page_slam.progress
        waypoints = self.active_waypoints or self.pending_path_waypoints
        if not waypoints:
            if strip.isVisible():
                strip.clear()
            self._progress_route_key = None
            return

        # Re-seed only when the route itself changes. A detour replaces the
        # waypoint list mid-flight, and the strip has to restart against the new
        # one rather than keep measuring against the route that was blocked.
        key = (len(waypoints), waypoints[0], waypoints[-1])
        if key != self._progress_route_key:
            self._progress_route_key = key
            strip.set_route(waypoints, origin=(t.x, t.y), state=self._progress_state())

        strip.update_progress(self.current_wpt_idx, (t.x, t.y), t.ground_speed,
                              state=self._progress_state())

    # -------------------------------------------------------------------------
    # Audio Alerts  (core/audio.AudioAlerts)
    # -------------------------------------------------------------------------

    def _update_audio_alerts(self, t: TelemetrySnapshot) -> None:
        """Sound the state transitions worth hearing without looking.

        Every branch here is edge-triggered against the previous tick. A
        condition that is continuously true - a flat battery, a dead link -
        must announce itself once and then be quiet, or the operator mutes the
        station and loses the alerts that matter.
        """
        # Link
        if self._alert_prev_connected is not None and t.connected != self._alert_prev_connected:
            if t.connected:
                self.audio.say("Link restored", Severity.OK, key="link")
            else:
                self.audio.say("Link lost", Severity.CRITICAL, key="link")
        self._alert_prev_connected = t.connected

        if not t.connected:
            # Everything below describes a vehicle we are no longer hearing
            # from; announcing stale state would be worse than silence.
            return

        # Arming
        if self._alert_prev_armed is not None and t.armed != self._alert_prev_armed:
            if t.armed:
                self.audio.say("Armed", Severity.WARN, key="arm_state")
            else:
                self.audio.say("Disarmed", Severity.OK, key="arm_state")
        self._alert_prev_armed = t.armed

        # Flight mode
        mode = t.flight_mode or ""
        if self._alert_prev_mode is not None and mode and mode != self._alert_prev_mode:
            spoken = mode.replace("AUTO.", "").replace("_", " ").title()
            self.audio.say(spoken, Severity.INFO, key="mode")
        self._alert_prev_mode = mode

        # Battery, by band rather than by percentage, so a value oscillating
        # across a threshold cannot produce a stream of warnings.
        alerts = self.settings.alerts
        pct = t.battery_percent
        if pct <= 0:
            band = "unknown"
        elif pct <= alerts.batt_crit_pct:
            band = "critical"
        elif pct <= alerts.batt_warn_pct:
            band = "warn"
        else:
            band = "ok"
        if self._alert_prev_batt_band is not None and band != self._alert_prev_batt_band:
            if band == "critical":
                self.audio.say(f"Battery critical, {pct} percent",
                               Severity.ALARM, key="battery")
            elif band == "warn":
                self.audio.say(f"Battery low, {pct} percent",
                               Severity.WARN, key="battery")
        self._alert_prev_batt_band = band

        # Vision, but only while airborne: losing VIO on the bench is normal
        # and constant, and announcing it there would make the alert worthless
        # in the air, which is the only place it matters.
        vio_ok = t.d435i_vio_health or t.ekf2_vision_fused
        if self._alert_prev_vio_ok is not None and vio_ok != self._alert_prev_vio_ok:
            if not vio_ok and t.is_airborne:
                self.audio.say("Vision lost", Severity.CRITICAL, key="vision")
            elif vio_ok and t.is_airborne:
                self.audio.say("Vision restored", Severity.OK, key="vision")
        self._alert_prev_vio_ok = vio_ok

    def _on_mute_toggled(self, muted: bool) -> None:
        """The header control. Ctrl+M routes through _toggle_mute instead, and
        both end up calling set_audio_state so the button can never disagree
        with the alert service."""
        self.audio.set_muted(muted)
        self.top_strip.set_audio_state(self.audio.muted, self.audio.available)
        self.console.log_info(
            f"Audio alerts {'muted' if self.audio.muted else 'unmuted'}.")

    def _toggle_mute(self) -> None:
        muted = self.audio.toggle_muted()
        self.top_strip.set_audio_state(muted, self.audio.available)
        state = "muted" if muted else "unmuted"
        self.console.log_info(f"Audio alerts {state}.")
        self.toast.show_message(f"Audio {state}",
                                "#d29922" if muted else "#238636", 2000)
        if not muted:
            self.audio.alert(Severity.OK)

    # -------------------------------------------------------------------------
    # Keyboard Shortcuts  (ui/shortcuts)
    # -------------------------------------------------------------------------

    def _shortcut_callbacks(self) -> dict:
        """Map ui.shortcuts action names to callables.

        Every destructive action points at a _request_* method, never a _cmd_*
        one, so a keystroke opens the confirmation rather than committing. There
        is deliberately no entry for the emergency kill.
        """
        callbacks = {
            "toggle_link": self._toggle_link,
            "arm": self._request_arm,
            "disarm": self._request_disarm,
            "takeoff": self._request_takeoff,
            "land": lambda: self._cmd_mode("AUTO.LAND"),
            "hold": lambda: self._cmd_mode("AUTO.LOITER"),
            "execute_path": self._shortcut_execute_path,
            "pause_path": self._shortcut_pause_path,
            "abort_path": self._request_abort_path,
            "fit_map": lambda: self.page_slam.canvas.fit_to_map(),
            "zoom_in": lambda: self.page_slam.canvas.zoom(1.25),
            "zoom_out": lambda: self.page_slam.canvas.zoom(0.80),
            "toggle_ruler": self._shortcut_toggle_ruler,
            "toggle_mute": self._toggle_mute,
            "cancel": self._shortcut_cancel,
            "show_help": self._show_shortcut_help,
        }
        for index in range(8):
            callbacks[f"workspace_{index}"] = (
                lambda i=index: self._switch_workspace(i))
        return callbacks

    def _switch_workspace(self, index: int) -> None:
        # select_tab checks the rail button and emits view_changed, which the
        # window already routes to the stack - so the rail and the page can
        # never disagree about which workspace is showing.
        self.sidebar.select_tab(index)

    def _toggle_link(self) -> None:
        if self.worker and self.worker.isRunning():
            self._disconnect_from_endpoint()
        else:
            conn = self.settings.connection
            port = conn.udp_port if conn.protocol == "udp" else conn.tcp_port
            self._connect_to_endpoint(conn.host, port, protocol=conn.protocol)

    def _shortcut_execute_path(self) -> None:
        if self.page_slam.canvas.planned_waypoints:
            self._on_execute_path_requested(self.page_slam.canvas.planned_waypoints)
        else:
            self.console.log_info("No path staged - click the map to set a goal.")

    def _shortcut_pause_path(self) -> None:
        if self.path_paused:
            self._on_resume_path_requested()
        elif self.path_in_progress:
            self._on_pause_path_requested()
        else:
            self.console.log_info("No path executing.")

    def _shortcut_toggle_ruler(self) -> None:
        self._switch_workspace(2)
        self.page_slam.btn_ruler.setChecked(not self.page_slam.btn_ruler.isChecked())

    def _shortcut_cancel(self) -> None:
        """Escape: one key that stops whatever is pending, in order of danger."""
        self.page_motors.stop_motor_tests()
        if self.confirm_bar.is_active():
            self.confirm_bar.cancel()
        if self.page_slam.btn_ruler.isChecked():
            self.page_slam.btn_ruler.setChecked(False)
        else:
            self.page_slam.canvas.clear_ruler()

    def _show_shortcut_help(self) -> None:
        if self._help_overlay is None:
            self._help_overlay = ShortcutHelpOverlay(self)
        self._help_overlay.show()
        self._help_overlay.raise_()

    # -------------------------------------------------------------------------
    # Bench Motor Test  (ui/motor_widget.MotorTestPanel)
    # -------------------------------------------------------------------------

    def _on_motor_test_requested(self, motor_num: int, throttle_pct: float) -> None:
        """Forward a bench motor test, after re-checking the interlocks here.

        The panel already refuses to emit unless the vehicle is connected,
        disarmed and on the ground, and MAVLinkWorker clamps the throttle. This
        is a third check at the last point before the command reaches the link,
        because the cost of a stale UI state here is a spinning propeller.
        """
        if not (self.worker and self.worker.isRunning()):
            return
        t = self.last_telemetry
        if t.armed or t.is_airborne:
            self.page_motors.stop_motor_tests()
            self.console.log_error(
                "Motor test refused: vehicle is armed or airborne.")
            return
        self.worker.test_actuator(motor_num, throttle_pct)

    def _on_motor_test_stop(self) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.stop_all_motor_tests()

    # -------------------------------------------------------------------------
    # Map "Fly here now"  (ui/slam_map_widget context menu)
    # -------------------------------------------------------------------------

    def _on_fly_here_requested(self, wx: float, wy: float) -> None:
        """Execute the path the map already planned to the clicked point.

        Routed through _on_execute_path_requested rather than dispatching
        setpoints directly, so the arm / OFFBOARD / VIO / ground-takeoff
        interlocks all apply exactly as they do to the EXECUTE PATH button.
        """
        waypoints = self.page_slam.canvas.planned_waypoints
        if not waypoints:
            self.console.log_error(
                f"Cannot fly to N {wx:+.2f}, E {wy:+.2f}: no clear path.")
            return
        self.console.log_cmd(
            f"Fly-here requested: N {wx:+.2f} m, E {wy:+.2f} m "
            f"({len(waypoints)} waypoints).")
        self._on_execute_path_requested(waypoints)

    def shutdown_workers(self) -> None:
        """Stop every background thread this window owns.

        Separate from closeEvent so anything that disposes of the window
        without a close - a test, a programmatic teardown - can still stop the
        threads. A QThread destroyed while running aborts the process.
        """
        worker = getattr(self, "planner_worker", None)
        if worker is not None:
            worker.stop()
        listener = getattr(self, "map_listener", None)
        if listener is not None:
            listener.stop()
        audio = getattr(self, "audio", None)
        if audio is not None:
            audio.shutdown()

    def closeEvent(self, event):
        # A Qt.Tool window is not a child in the window-manager sense; without
        # this it outlives the main window as an orphan always-on-top frame.
        self.fpv_float.close()
        """Clean shutdown of threads and embedded processes on window close."""
        if hasattr(self, "page_slam"):
            self.page_slam.stop_rviz()
        if self.map_listener:
            self.map_listener.stop()
        # Stop any bench motor test before the link goes away, rather than
        # relying on the vehicle-side expiry to catch it.
        if hasattr(self, "page_motors"):
            self.page_motors.stop_motor_tests()
            self._on_motor_test_stop()
        if self.worker and self.worker.isRunning():
            self.worker.disconnect_endpoint()
        self.shutdown_workers()
        event.accept()


def build_arg_parser() -> argparse.ArgumentParser:
    """Every network endpoint is a flag, so this app can be pointed at a
    different vehicle without editing source.

    Precedence is flag > environment variable > ~/.drone_gcs/settings.json >
    built-in default. --host alone re-points MAVLink, the map bridge and the
    video stream together, which is the common case.
    """
    ap = argparse.ArgumentParser(
        prog="drone_gcs.py",
        description="Drone-GCS - PyQt5 ground control station for PX4 + ROS 2.")
    ap.add_argument("--host", help="autopilot / companion host (env GCS_HOST)")
    ap.add_argument("--port", help="MAVLink port (env GCS_PORT)")
    ap.add_argument("--protocol", choices=("udp", "tcp"),
                    help="MAVLink transport (env GCS_PROTOCOL)")
    ap.add_argument("--map-host", help="TCP map bridge host (env GCS_MAP_HOST)")
    ap.add_argument("--map-port", help="TCP map bridge port (env GCS_MAP_PORT)")
    ap.add_argument("--video-url", help="FPV stream URL (env GCS_VIDEO_URL)")
    ap.add_argument("--rviz-config", help="RViz2 layout path (env GCS_RVIZ_CONFIG)")
    ap.add_argument("--ui-scale", type=float,
                    help="UI scale factor, 0.75-3.0 (env GCS_UI_SCALE; "
                         "omit or 0 to detect from the display)")
    ap.add_argument("--no-audio", action="store_true",
                    help="start with audio alerts muted")
    return ap


def main():
    # parse_known_args: Qt consumes its own switches from argv and must not
    # trip over ours, nor ours over its.
    args, _qt_args = build_arg_parser().parse_known_args()
    settings = apply_overrides(load_settings(), args)

    pin_system_qt_plugins()
    # Must precede the QApplication: Qt reads these attributes once, at
    # construction, and silently ignores them afterwards.
    enable_high_dpi()

    app = QApplication(sys.argv)
    # Resolve the UI scale before any stylesheet or widget exists, so every
    # dimension in the station is computed from the same factor.
    init_scale(app, settings, override=getattr(args, "ui_scale", None))
    app.setStyleSheet(build_stylesheet())
    window = DroneGCSMainWindow(settings=settings)
    if getattr(args, "no_audio", False):
        # A session flag, not a saved preference: --no-audio for one bench run
        # must not silence the station permanently, and must leave the header
        # control live so alerts can be turned back on without a restart.
        window._on_mute_toggled(True)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
