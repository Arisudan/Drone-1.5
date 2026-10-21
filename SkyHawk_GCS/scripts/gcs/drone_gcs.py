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
  python3 drone_gcs.py --host 172.16.101.89 --port 5760
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

from PyQt5.QtCore import Qt, QTimer, QEvent
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
from ui.scaling import init_scale, enable_high_dpi, px, fit_min_width, design_window_size
from ui.toast import NotificationToast
from ui.hud_widget import HUDWidget
from ui.preflight_panel import ACTION_TAB
from ui.slam_map_widget import SLAMMapWidget
from ui.cli_console import CLIConsoleWidget
from ui.motor_widget import MotorWidget
from ui.video_feed_widget import VideoFeedWidget, FloatingVideoWindow, FullscreenVideoWindow
from ui.top_status_strip import TopStatusStrip, WFB_NETWORK
from ui.sidebar_nav import SidebarNav
from ui.theme import ShellRoot, GlassPanel, MetricCell, CurrentPageStack
from ui.shell import RouteCard, VehicleCard, TelemetryCard, CommandsCard
from ui.vehicle_marker import VehicleMarker
from ui.preflight_card import PreflightCard
from ui.logs_tab import LogsTabWidget
from ui.config_tab import ConfigTabWidget
from ui.actuator_widget import ActuatorPanel
from ui.params_tab import ParamsTabWidget
from ui.value_grid import ValueGridWidget
from ui.guided_confirm import GuidedConfirmBar
from ui.shortcuts import install_shortcuts, ShortcutHelpOverlay
from controllers.flight_commands import FlightCommandsMixin
from controllers.arbitration import ArbitrationMixin
from controllers.mission_control import MissionControlMixin
from controllers.alarm_control import AlarmControlMixin
from controllers.preflight_control import PreflightControlMixin
from protocol.radxa_status_worker import RadxaStatusWorker
from core.alarms import AlarmManager
from ui.alarm_banner import AlarmBanner
from core.flight_log import FlightLogger
from core.ui_stall import UiStallMonitor
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


class DroneGCSMainWindow(ArbitrationMixin, FlightCommandsMixin, MissionControlMixin, AlarmControlMixin,
                         PreflightControlMixin, QMainWindow):
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
        # Open as the 16:9 design canvas at the active scale (ui/scaling.py), shrunk with
        # its aspect kept if it still exceeds the screen, and centred - one station shape
        # on every display. init_scale() already capped the automatic scale to fit.
        self.resize(*design_window_size(*avail))
        self._center_on_screen()
        # Lowered from 1150x740: that floor didn't actually fit common smaller laptop
        # panels (1366x768 and especially 1280x720) once OS window-chrome/taskbar space
        # is subtracted, forcing clipping instead of a graceful shrink. 1024x700 still
        # comfortably fits every control - it was headroom, not a real usability need.
        self.setMinimumSize(min(px(1220), avail[0]), min(px(700), avail[1]))

        # Persisted settings and the flight recorder are built before the UI,
        # because the Config and Logs workspaces are views onto them.
        self.settings = settings if settings is not None else apply_overrides(load_settings())
        self.flight_logger = FlightLogger()
        self.alarms = AlarmManager()
        # Preflight checklist / Radxa watchdog state (controllers/preflight_control.py)
        self._video_state = "IDLE"
        self._preflight_vision_was_ok = False
        self._preflight_checks = []
        self._preflight_at = 0.0
        self._radxa_status = None
        self._radxa_status_at = 0.0
        self._radxa_host = self.settings.connection.host
        self._ever_connected = False

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
        self._arb_init()

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
        self.ui_stall = UiStallMonitor()
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

        # One capture thread, four viewports: the FPV tab's own label, the
        # cockpit's centre panel, the draggable floating window, and the
        # shared fullscreen window any of the others can expand into.
        self.fpv_float = FloatingVideoWindow(self)
        self.fpv_fullscreen = FullscreenVideoWindow(self)
        self.page_fpv.frame_broadcast.connect(self.hud.video.on_frame)
        self.page_fpv.frame_broadcast.connect(self.fpv_float.sink.on_frame)
        self.page_fpv.frame_broadcast.connect(self.fpv_fullscreen.sink.on_frame)
        self.page_fpv.fullscreen_requested.connect(self.fpv_fullscreen.open)
        self.page_fpv.health_changed.connect(self._on_video_health)
        self._fpv_popped_out = False       # the SLAM feed is docked in the side panel unless popped out
        # Camera: docked at the top of the SLAM side panel (full floating-window size). The same
        # single capture thread feeds it; POP OUT hands the feed to the floating window instead.
        self.page_fpv.frame_broadcast.connect(self.page_slam.docked_feed.sink.on_frame)
        self.page_slam.docked_feed.fullscreen_requested.connect(self.fpv_fullscreen.open)
        self.page_slam.docked_feed.popout_requested.connect(lambda: self._set_fpv_popped_out(True))
        self.page_slam.docked_feed.dock_requested.connect(lambda: self._set_fpv_popped_out(False))
        self.fpv_float.closed.connect(lambda: self._set_fpv_popped_out(False))
        self.page_fpv.frame_broadcast.connect(self.sidebar.mini_feed.on_frame)
        self.page_fpv.health_changed.connect(self.sidebar.mini_feed.set_health)
        self.sidebar.mini_feed.fullscreen_requested.connect(self.fpv_fullscreen.open)
        self.hud.fullscreen_requested.connect(self.fpv_fullscreen.open)

        # Radxa watchdog poller. Off under tests (DRONE_GCS_NO_RADXA_POLL=1): a
        # real socket attempt per window would only slow them down.
        self.radxa_poller = None
        if os.environ.get("DRONE_GCS_NO_RADXA_POLL") != "1":
            self.radxa_poller = RadxaStatusWorker(
                lambda: self._radxa_host, lambda: self.settings.connection.watchdog_port, parent=self)
            self.radxa_poller.status_received.connect(self._on_radxa_status)
            self.radxa_poller.start()
        self.page_params.save_flash_requested.connect(self._request_save_params)

        # A saved wfb-ng (udpin) link also means the wfb-ng network preset and video
        # source - select them so the header and FPV tab match what is about to connect.
        conn = self.settings.connection
        if conn.protocol == "udpin":
            wfb_idx = self.top_strip.network_combo.findText(WFB_NETWORK)
            if wfb_idx >= 0:
                self.top_strip.network_combo.setCurrentIndex(wfb_idx)

        # Auto-connect to default autopilot endpoint on launch (UDP 14550)
        port = conn.udp_port if conn.protocol in ("udp", "udpin") else conn.tcp_port
        self._connect_to_endpoint(conn.host, port, protocol=conn.protocol)

    def _center_on_screen(self) -> None:
        try:
            from PyQt5.QtWidgets import QApplication as _QApp
            screen = _QApp.primaryScreen()
            if screen is not None:
                frame = self.frameGeometry()
                frame.moveCenter(screen.availableGeometry().center())
                self.move(frame.topLeft())
        except Exception:
            pass

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
        """v2 shell: backdrop + top bar, icon rail, route card, page stack, and the
        persistent right column (vehicle / telemetry / commands)."""
        self.shell_root = ShellRoot(self)
        self.setCentralWidget(self.shell_root)
        root_layout = QVBoxLayout(self.shell_root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        self.top_strip = TopStatusStrip(self)
        self.top_strip.connect_requested.connect(self._connect_to_endpoint)
        self.top_strip.disconnect_requested.connect(self._disconnect_from_endpoint)
        self.top_strip.network_changed.connect(self._on_network_selected)
        self.top_strip.mute_toggled.connect(self._on_mute_toggled)
        self.top_strip.menu_requested.connect(self._show_shortcut_help)
        self.top_strip.set_audio_state(self.audio.muted, self.audio.available)
        root_layout.addWidget(self.top_strip)

        # Standing alarm strip: hidden until something is wrong.
        self.alarm_banner = AlarmBanner(self.alarms, self)
        self.alarm_banner.ack_all_requested.connect(self._on_alarm_ack_all)
        self.alarm_banner.ack_requested.connect(self._on_alarm_ack)
        # Floats over the page area just under the top bar (never pushes the layout down).
        self.alarm_banner.setParent(self.shell_root)
        self.alarm_banner.installEventFilter(self)

        body = QHBoxLayout()
        body.setContentsMargins(px(12), px(12), px(16), px(12))
        body.setSpacing(px(14))

        self.sidebar = SidebarNav(self)
        self.sidebar.view_changed.connect(self._on_view_changed)
        body.addWidget(self.sidebar, 0, Qt.AlignTop)

        page_col = QVBoxLayout()
        page_col.setContentsMargins(0, 0, 0, 0)
        page_col.setSpacing(0)
        self.route_card = RouteCard(self)
        page_col.addWidget(self.route_card, 0, Qt.AlignLeft)
        self.stack = CurrentPageStack(self)
        # Scrolls (never overlaps or forces the window bigger) when the window is
        # smaller than the visible page needs.
        self.page_scroll = QScrollArea(self)
        self.page_scroll.setObjectName("pageScroll")
        self.page_scroll.setWidgetResizable(True)
        self.page_scroll.setFrameShape(QFrame.NoFrame)
        self.page_scroll.viewport().setAutoFillBackground(False)
        self.page_scroll.setWidget(self.stack)
        page_col.addWidget(self.page_scroll, 1)
        body.addLayout(page_col, 1)

        # Persistent right column
        right = QWidget(self)
        right.setFixedWidth(px(400))
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(0)
        self.vehicle_card = VehicleCard(self)
        self.telemetry_card = TelemetryCard(self)
        self.commands_card = CommandsCard(self)
        rl.addWidget(self.vehicle_card)
        rl.addWidget(self.telemetry_card)
        rl.addSpacing(px(8))
        rl.addWidget(self.commands_card)
        rl.addStretch(1)
        self._wire_commands_card()
        body.addWidget(right, 0)
        root_layout.addLayout(body, 1)

        # Page 0: Control
        self.page_cockpit = self._build_cockpit_page()
        self.stack.addWidget(self.page_cockpit)

        # Page 1: FPV Camera Feed
        self.page_fpv = VideoFeedWidget(self)
        self.page_fpv.txt_url.setText(self.settings.video.stream_url)
        self.page_fpv.set_wfb_ports(self.settings.video.wfb_rtp_port, self.settings.video.wfb_ts_port)
        self.stack.addWidget(self.page_fpv)
        # Full-bleed picture: behind every panel, above the backdrop (shown only on this page).
        self.shell_root.set_bleed(self.page_fpv.video_label)

        # Page 2: Tactical 2D SLAM & Waypoint Stager
        self.page_slam = SLAMMapWidget(self, rviz_config=resolve_rviz_config(self.settings),
                                    flight_modes=PX4_MODES_LIST)
        # The 3D view's map stream comes from whatever the GCS is connected to (the Jetson).
        self.page_slam.rviz_widget.host_provider = lambda: (
            getattr(self.worker, "host", "") if self.worker else "") or self._radxa_host
        self.page_slam.execute_path_requested.connect(self._on_execute_path_requested)
        self.page_slam.pause_path_requested.connect(self._on_pause_path_requested)
        self.page_slam.resume_path_requested.connect(self._on_resume_path_requested)
        # Through _request_abort_path (the slide-to-confirm gate), not straight to the executor.
        self.page_slam.abort_path_requested.connect(self._request_abort_path)
        self.page_slam.altitude_changed.connect(self._on_cruise_altitude_changed)
        self.page_slam.reset_map_requested.connect(self._on_reset_map_requested)
        self.page_slam.set_map_source_port(self.settings.video.map_bridge_port)
        self.page_slam.map_source_changed.connect(self._on_map_source_changed)
        self.page_slam.fly_here_requested.connect(self._on_fly_here_requested)
        # Header ARM / DISARM / MODE: the same handlers as the Control tab.
        self.page_slam.flight_bar.arm_requested.connect(self._request_arm)
        self.page_slam.flight_bar.disarm_requested.connect(self._cmd_disarm)
        self.page_slam.flight_bar.mode_requested.connect(
            lambda m: self._request_mode(m, self.page_slam.flight_bar.btn_set_mode))
        self.page_slam.flight_bar.takeoff_requested.connect(self._request_takeoff)
        # "Fix first" in the preflight checklist jumps to the page that shows that check.
        self.sidebar.checklist.action_requested.connect(
            lambda key: self._switch_workspace(ACTION_TAB.get(key, 0)))
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
        term_page = QWidget(self)
        tl = QHBoxLayout(term_page)
        tl.setContentsMargins(0, 0, 0, 0)
        pnl_term = GlassPanel("Mission console", "flight terminal", True, term_page)
        pnl_term.setMaximumWidth(px(1090))
        pnl_term.body_layout.setContentsMargins(px(6), px(8), px(6), px(10))
        pnl_term.body_layout.addWidget(self.page_terminal)
        tl.addWidget(pnl_term, 1)
        tl.addStretch(0)
        self.page_terminal_view = term_page
        self.stack.addWidget(term_page)

        # Page 6: Flight Logs - history of every armed session
        self.page_logs = LogsTabWidget(self.flight_logger, self,
                                       log_host=lambda: self.settings.connection.host)
        self.stack.addWidget(self.page_logs)

        # Page 7: Configuration - typed settings, validated and persisted
        self.page_config = ConfigTabWidget(self.settings, self)
        self.page_config.settings_saved.connect(self._on_settings_saved)
        self.stack.addWidget(self.page_config)

        # Page 8: PX4 Parameters
        self.page_params = ParamsTabWidget(self)
        self.page_params.refresh_requested.connect(self._request_param_refresh)
        self.stack.addWidget(self.page_params)

        # Vision-engine toggles: one panel, mounted on whichever page shows it.
        self.footer_bar = self._build_footer_bar()
        self._vision_slots = {}            # page index -> QVBoxLayout that hosts the panel
        self._vision_slots[0] = self._cockpit_left
        self._vision_slots[1] = self.page_fpv.vision_slot
        self._apply_page_chrome(0)

    # Per-page chrome: backdrop mode, route-card text, top-bar title. Pages may call
    # set_route_sub() to change the live status line.
    PAGE_CHROME = {
        0: ("map", "sat_campus.jpg", "Control", "Control", "", "Quadcopter \u00b7 not connected"),
        1: ("map", "sat_close.jpg", "FPV Camera", "FPV", "live", "Stream active"),
        2: ("grid", "sat_campus.jpg", "Tactical SLAM", "Route", "2D map", "Untitled route"),
        3: ("dim", "sat_campus.jpg", "Motor Actuators", "Actuators", "", "Telemetry idle"),
        4: ("dim", "sat_close.jpg", "Diagnostics", "Diagnostics", "", "Bench checks"),
        5: ("dim", "sat_campus.jpg", "Flight Terminal", "Terminal", "", "Console ready"),
        6: ("dim", "sat_close.jpg", "Flight Logs", "Logs", "", "history"),
        7: ("dim", "sat_campus.jpg", "Configuration", "Settings", "", "All changes saved"),
        8: ("dim", "sat_close.jpg", "Parameters", "Parameters", "PX4 \u00b7 read-only", "PX4 parameters"),
    }

    def _apply_page_chrome(self, idx: int) -> None:
        mode, image, title, route, sub, route_sub = self.PAGE_CHROME.get(idx, self.PAGE_CHROME[0])
        self.shell_root.backdrop.set_mode(mode, image)
        self.top_strip.set_page_title(title, sub)
        self.route_card.set_route(route, route_sub)
        # The live-video layer is full-bleed behind the UI: only the FPV page shows it.
        self.page_fpv.video_label.setVisible(idx == 1)
        # Mount the vision-engines panel where this page wants it (if anywhere).
        lay = self._vision_slots.get(idx)
        if getattr(self, "_vision_mounted", None) is not lay:
            if getattr(self, "_vision_mounted", None) is not None:
                self._vision_mounted.removeWidget(self.footer_bar)
            self.footer_bar.setVisible(lay is not None)
            if lay is not None:
                lay.addWidget(self.footer_bar)
            self._vision_mounted = lay

    def set_route_sub(self, text: str) -> None:
        self.route_card.set_sub(text)

    def _wire_commands_card(self) -> None:
        """Hook the command card's buttons to the existing handlers (unchanged)."""
        c = self.commands_card
        self.btn_arm, self.btn_disarm = c.btn_arm, c.btn_disarm
        self.btn_hold, self.btn_land, self.btn_kill = c.btn_hold, c.btn_land, c.btn_kill
        self.btn_arm.clicked.connect(self._request_arm)
        # Deliberately not behind the slide-to-confirm bar (see _cmd_disarm: airborne -> AUTO.LAND).
        self.btn_disarm.clicked.connect(self._cmd_disarm)
        self.btn_hold.clicked.connect(lambda: self._cmd_mode("AUTO.LOITER"))
        self.btn_land.clicked.connect(lambda: self._cmd_mode("AUTO.LAND"))
        self.btn_kill.clicked.connect(self._request_kill)
        c.btn_rtl.clicked.connect(lambda: self._request_mode("AUTO.RTL", c.btn_rtl))
        c.btn_manual.clicked.connect(lambda: self._request_mode("MANUAL", c.btn_manual))
        c.btn_continue.clicked.connect(self._on_resume_path_requested)
        c.btn_click_go.clicked.connect(lambda: self._switch_workspace(2))

    def _build_cockpit_page(self) -> QWidget:
        """Page 0: Navigation / guided displacement / FPV / vision engines on the left,
        the map-style backdrop with the vehicle marker in the middle, console below it.
        Same widgets and handlers as before; only the arrangement and look changed."""
        page = QWidget(self)
        layout = QHBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(px(24))

        left = QWidget(page)
        left.setFixedWidth(px(340))
        lc = QVBoxLayout(left)
        lc.setContentsMargins(0, 0, 0, 0)
        lc.setSpacing(px(8))
        self._cockpit_left = lc

        # ---- NAVIGATION ----
        pnl_nav = GlassPanel("Navigation", "", True, left)
        nb = pnl_nav.body_layout
        nb.setSpacing(px(9))

        def field_row(label: str, widget: QWidget) -> QHBoxLayout:
            row = QHBoxLayout()
            row.setSpacing(px(10))
            lbl = QLabel(label, pnl_nav)
            lbl.setObjectName("fieldLabel")
            lbl.setFixedWidth(px(118))
            row.addWidget(lbl)
            row.addWidget(widget, 1)
            return row

        self.ent_takeoff_alt = QLineEdit("1.0", self)
        self.ent_takeoff_alt.setAlignment(Qt.AlignRight)
        self.ent_takeoff_alt.setValidator(QDoubleValidator(0.2, 10.0, 2, self))
        self.ent_takeoff_alt.setProperty("num", True)
        nb.addLayout(field_row("Takeoff alt", self.ent_takeoff_alt))

        self.ent_yaw = QLineEdit("90.0", self)
        self.ent_yaw.setAlignment(Qt.AlignRight)
        self.ent_yaw.setValidator(QDoubleValidator(-360.0, 360.0, 1, self))
        self.ent_yaw.setProperty("num", True)
        nb.addLayout(field_row("Yaw", self.ent_yaw))

        # Read-only: the current altitude, the starting point for CHANGE ALT.
        self.lbl_current_alt = QLabel("-- m AGL", self)
        self.lbl_current_alt.setObjectName("fieldReadout")
        self.lbl_current_alt.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        fit_min_width(self.lbl_current_alt, ["-- m AGL", "-000.00 m AGL"], h_pad_px=6)
        nb.addLayout(field_row("Change alt", self.lbl_current_alt))

        btn_row = QHBoxLayout()
        btn_row.setSpacing(px(8))
        self.btn_takeoff = QPushButton("TAKEOFF", self)
        self.btn_takeoff.setObjectName("btnNav")
        self.btn_takeoff.setProperty("on", True)
        self.btn_takeoff.clicked.connect(self._request_takeoff)
        self.btn_yaw = QPushButton("ROTATE YAW", self)
        self.btn_yaw.setObjectName("btnNav")
        self.btn_yaw.clicked.connect(self._request_yaw)
        self.btn_change_alt = QPushButton("CHANGE ALT", self)
        self.btn_change_alt.setObjectName("btnNav")
        self.btn_change_alt.setToolTip(
            "Hold position and climb or descend to a new altitude (requires OFFBOARD)")
        self.btn_change_alt.clicked.connect(self._request_change_alt)
        for b in (self.btn_takeoff, self.btn_yaw, self.btn_change_alt):
            b.setMinimumHeight(px(32))
            btn_row.addWidget(b, 1)
        nb.addSpacing(px(2))
        nb.addLayout(btn_row)
        nb.addSpacing(px(4))

        self.combo_modes = QComboBox(self)
        self.combo_modes.addItems(PX4_MODES_LIST)
        self.combo_modes.setCurrentText("STABILIZED")
        nb.addLayout(field_row("Flight mode", self.combo_modes))

        mode_row = QHBoxLayout()
        mode_row.setSpacing(px(8))
        self.btn_set_mode = QPushButton("SET MODE", self)
        self.btn_set_mode.setMinimumHeight(px(32))
        self.btn_set_mode.clicked.connect(self._cmd_set_mode_from_ui)
        mode_row.addWidget(self.btn_set_mode, 1)
        self.btn_check_origin = QPushButton("CHECK ORIGIN", self)
        self.btn_check_origin.setObjectName("originBtn")
        self.btn_check_origin.setMinimumHeight(px(32))
        self.btn_check_origin.setToolTip("Ask PX4 whether EKF2 has a global origin")
        self.btn_check_origin.clicked.connect(self._check_origin)
        mode_row.addWidget(self.btn_check_origin, 1)
        nb.addLayout(mode_row)

        # GCS command arbitration chip (READY / BLOCKED / PILOT): lives in the telemetry card's chip row.
        self.chip_gcs_ctrl = QPushButton("GCS CTRL: --", self)
        self.chip_gcs_ctrl.setObjectName("arbChip")
        self.chip_gcs_ctrl.setToolTip("Whether the GCS may fly the vehicle right now")
        self.chip_gcs_ctrl.clicked.connect(self._arb_chip_clicked)
        self.telemetry_card.add_chip(self.chip_gcs_ctrl)
        # ---- ESP32 servo actuator (HTTP, not MAVLink) ----
        self.actuator = ActuatorPanel(
            self.settings.actuator.host, self.settings.actuator.port, self)
        self.actuator.log_message.connect(
            lambda msg, err: (self.console.log_error if err else self.console.log_success)(msg))

        nb.addWidget(self.actuator)
        lc.addWidget(pnl_nav)

        # ---- GUIDED DISPLACEMENT (execution verifier) ----
        pnl_gd = GlassPanel("Guided displacement", "", False, left)
        gb = pnl_gd.body_layout
        gb.setSpacing(px(8))
        read_row = QHBoxLayout()
        read_row.setSpacing(px(6))
        self.lbl_exec_axes = {}
        for key, axis in (("dx", "\u0394X"), ("dy", "\u0394Y"), ("dz", "\u0394Z"), ("spd", "SPD")):
            cell = MetricCell(axis, "0.00" if key == "spd" else "+0.00", "", pnl_gd, big=19)
            cell.setProperty("mono", True)
            cell.layout().setContentsMargins(px(2), px(8), px(2), px(8))   # room for "-000.00" in a 340px panel
            val = cell.lines[0][0]
            val.setObjectName("execReadout")      # mono, amber, set per update by the tick
            val.setTextFormat(Qt.PlainText)
            val.setText("0.00" if key == "spd" else "+0.00")
            self.lbl_exec_axes[key] = val
            read_row.addWidget(cell, 1)
        gb.addLayout(read_row)

        state_row = QHBoxLayout()
        state_row.setSpacing(px(8))
        self.lbl_exec_state = QLabel("IDLE", self)
        self.lbl_exec_state.setObjectName("execBadgeIdle")
        self.lbl_exec_state.setAlignment(Qt.AlignCenter)
        self.lbl_exec_state.setFixedWidth(px(84))
        state_row.addWidget(self.lbl_exec_state)
        state_row.addStretch(1)
        self.lbl_exec_elapsed = QLabel("--", self)
        self.lbl_exec_elapsed.setObjectName("valueMono")
        self.lbl_exec_elapsed.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_exec_elapsed.setFixedWidth(px(52))
        state_row.addWidget(self.lbl_exec_elapsed)
        gb.addLayout(state_row)
        self.lbl_exec_val = QLabel("Holding position - no command in flight", self)
        self.lbl_exec_val.setObjectName("execMessage")
        self.lbl_exec_val.setWordWrap(True)
        gb.addWidget(self.lbl_exec_val)

        self.progress_verif = QProgressBar(self)
        self.progress_verif.setFixedHeight(px(8))
        self.progress_verif.setTextVisible(False)
        self.progress_verif.setFormat("no displacement target")
        self.progress_verif.setValue(0)
        gb.addWidget(self.progress_verif)
        lc.addWidget(pnl_gd)

        # ---- FPV thumbnail (the HUD pane owns the video sink) ----
        self.pnl_fpv = GlassPanel("FPV", "no feed", False, left)
        self.hud = HUDWidget(self)
        self.hud.setMinimumHeight(px(126))
        fb = self.pnl_fpv.body_layout
        fb.setContentsMargins(0, 0, 0, 0)
        fb.addWidget(self.hud)
        lc.addWidget(self.pnl_fpv, 1)

        # (vision engines panel is mounted here by _apply_page_chrome)

        # ---- Guided action confirmation (floating overlay, parented to the main window) ----
        self.confirm_bar = GuidedConfirmBar(self)
        self.confirm_bar.confirmed.connect(self._on_guided_confirmed)
        self.confirm_bar.cancelled.connect(self._on_guided_cancelled)

        layout.addWidget(left)

        # ---- centre: vehicle marker over the backdrop, console below ----
        centre = QWidget(page)
        cc = QVBoxLayout(centre)
        cc.setContentsMargins(0, 0, 0, px(38))
        cc.setSpacing(px(12))
        self.vehicle_marker = VehicleMarker(centre)
        cc.addWidget(self.vehicle_marker, 1)

        self.console = CLIConsoleWidget(self)
        self.console.command_submitted.connect(self._execute_cli_command)
        pnl_con = GlassPanel("Console", "flight terminal", False, centre)
        pnl_con.setFixedHeight(px(150))
        pnl_con.setMaximumWidth(px(720))
        pc = pnl_con.body_layout
        pc.setContentsMargins(px(6), px(2), px(6), px(6))
        pc.addWidget(self.console)
        row = QHBoxLayout()
        row.addWidget(pnl_con, 1)
        row.addStretch(0)
        cc.addLayout(row)
        layout.addWidget(centre, 1)
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

        # v2 layout: preflight checklist + bench safety (640 wide) over the telemetry value
        # grid, the whole column scrolling vertically.
        page = QScrollArea(self)
        page.setObjectName("diagScroll")
        page.setWidgetResizable(True)
        page.setFrameShape(QFrame.NoFrame)
        page.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        inner = QWidget()
        inner.setObjectName("diagInner")
        col = QVBoxLayout(inner)
        col.setContentsMargins(0, 0, px(6), 0)
        col.setSpacing(px(10))

        pnl_pf = GlassPanel("Preflight checklist", "0 of 7 ready", True, inner)
        pnl_pf.setMaximumWidth(px(640))
        self.preflight_card = PreflightCard(self.sidebar.checklist, pnl_pf)
        self.preflight_card.action_requested.connect(self.sidebar.checklist.action_requested.emit)
        self.preflight_card.summary_changed.connect(
            lambda p_, t_, r_: pnl_pf.set_sub("ready" if r_ else f"{p_} of {t_} ready"))
        pnl_pf.body_layout.addWidget(self.preflight_card)
        col.addWidget(pnl_pf)

        pnl_bench = GlassPanel("Bench safety", "", False, inner)
        pnl_bench.setMaximumWidth(px(640))
        bl = pnl_bench.body_layout
        bl.setSpacing(px(8))
        self.bench_buttons = {}
        for r_ in (("SERVO TEST", "RADIO CALIBRATION"), ("ESC BEEP", "GIMBAL CHECK")):
            row = QHBoxLayout()
            row.setSpacing(px(8))
            for label in r_:
                b_ = QPushButton(label, pnl_bench)
                b_.setMinimumHeight(px(32))
                b_.setEnabled(False)
                b_.setProperty("role", "off")
                b_.setToolTip("Not available: this station has no bench-test backend for it "
                              "(use Motor Actuators for the motor test)")
                self.bench_buttons[label] = b_
                row.addWidget(b_, 1)
            bl.addLayout(row)
        note = QLabel("Requires: vehicle disarmed \u00b7 props off \u00b7 slide to confirm", pnl_bench)
        note.setObjectName("noteWarn")
        bl.addWidget(note)
        col.addWidget(pnl_bench)

        pnl_vals = GlassPanel("Telemetry values", "", False, inner)
        pnl_vals.body_layout.setContentsMargins(px(10), px(8), px(10), px(10))
        pnl_vals.body_layout.addWidget(self.value_grid)
        col.addWidget(pnl_vals)
        col.addStretch(1)
        page.setWidget(inner)
        return page

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

    def _build_footer_bar(self) -> QWidget:
        """Vision-engine toggles (panel). These request perception overlays this station
        does not yet carry; the switches latch as UI state and the note names every
        requested-but-unavailable engine, so a lit switch is never read as a live overlay."""
        panel = GlassPanel("Vision engines", "", False, self)
        lay = panel.body_layout
        row = QHBoxLayout()
        row.setSpacing(px(8))
        self.vision_buttons = {}
        specs = [
            ("detect", "Detect", "Object detection (RF-DETR) - engine not installed on this station"),
            ("depth", "Depth", "Monocular depth overlay (Depth-Anything V2) - engine not installed"),
            ("face", "Face Rec", "Face recognition (InsightFace SCRFD + ArcFace) - engine not installed"),
            ("track", "Track", "Human tracking (YOLOv8-pose + DeepSort) - engine not installed"),
        ]
        for key, label, tip in specs:
            btn = QPushButton(label.upper(), self)
            btn.setObjectName("btnVision")
            btn.setProperty("visionRole", key)
            btn.setCheckable(True)
            btn.setToolTip(tip)
            btn.clicked.connect(lambda checked, k=key: self._on_vision_toggle(k, checked))
            row.addWidget(btn, 1)
            self.vision_buttons[key] = btn
        lay.addLayout(row)
        self.lbl_footer_note = QLabel(
            "Unavailable - no onboard perception pipeline", self)
        self.lbl_footer_note.setObjectName("footerNote")
        self.lbl_footer_note.setWordWrap(True)
        lay.addWidget(self.lbl_footer_note)
        return panel

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
                "Unavailable - no onboard perception pipeline")

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

    def _place_alarm_banner(self) -> None:
        b = getattr(self, "alarm_banner", None)
        if b is None or not hasattr(self, "top_strip"):
            return
        margin = px(12)
        w = max(100, self.shell_root.width() - 2 * margin)
        b.setFixedWidth(w)
        b.adjustSize()
        b.move(margin, self.top_strip.height() + px(4))
        b.raise_()

    def eventFilter(self, obj, event):
        if obj is getattr(self, "alarm_banner", None) and event.type() in (
                QEvent.Show, QEvent.LayoutRequest):
            QTimer.singleShot(0, self._place_alarm_banner)
        return super().eventFilter(obj, event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._place_alarm_banner()
        # Wide rail only when the window can spare it (see SidebarNav.set_wide).
        if hasattr(self, "sidebar"):
            self.sidebar.set_wide(self.width() >= px(self.sidebar.WIDE_MIN_WINDOW_PX))

    def _on_view_changed(self, idx: int):
        self.stack.setCurrentIndex(idx)
        self._apply_page_chrome(idx)
        page = self.stack.widget(idx)

        # Any workspace that shows the camera starts it. Idempotent - does
        # nothing if a feed is already running, and there is only ever one
        # capture thread regardless of how many viewports are open.
        # The rail thumbnail covers the workspaces with no camera of their own,
        # so those start the feed too.
        mini = idx in self.sidebar.MINI_FEED_TABS
        self.sidebar.set_mini_feed_wanted(mini)
        if mini or page in (self.page_fpv, self.page_cockpit, self.page_slam):
            self.page_fpv.ensure_started()

        # On the SLAM workspace the camera is docked in the side panel; the floating
        # window only appears when the operator pops it out.
        if page is self.page_slam and self._fpv_popped_out:
            first = not self.fpv_float.isVisible()
            if first:
                self._position_fpv_float()
            self.fpv_float.show()
            self.fpv_float.raise_()
            if first:
                # The page has just become current, so the canvas has its real
                # geometry only after the event loop runs once.
                QTimer.singleShot(0, self._position_fpv_float)
        else:
            self.fpv_float.hide()

        # Fetch the parameter list the first time the tab is opened, not on
        # every connect - a full PX4 parameter set is 1000-1800+ messages,
        # and the operator may never open this tab in a given session.
        if page is self.page_motors:
            self._request_motor_range()

        if page is self.page_params and not self.page_params.has_data():
            self._request_param_refresh()

    def _set_fpv_popped_out(self, popped: bool) -> None:
        """Dock the SLAM camera in the side panel (False) or float it as a movable window (True)."""
        self._fpv_popped_out = bool(popped)
        self.page_slam.set_feed_docked(not popped)
        on_slam = self.stack.currentWidget() is self.page_slam
        if popped and on_slam:
            self._position_fpv_float()
            self.fpv_float.show()
            self.fpv_float.raise_()
        elif not popped:
            self.fpv_float.hide()

    def _position_fpv_float(self):
        """Park the floating viewport over the map's top-left corner.

        Top-left, not bottom-right: the map's own readouts (scale bar, hints)
        live along the bottom, the compass is top-right, and the corner nearest
        the toolbar is where the operator's eye already is. It stays a separate
        window, so it can still be dragged anywhere.
        """
        canvas = self.page_slam.canvas
        margin = 12
        corner = canvas.mapToGlobal(canvas.rect().topLeft())
        # Clear of the small Inflation / Keep-out / Measure strip on the map's top-left.
        corner.setX(corner.x() + 8)
        if canvas.width() < 50:           # page not laid out yet: fall back to the window corner
            corner = self.mapToGlobal(self.rect().topLeft())
            margin = 160
        self.fpv_float.move(corner.x() + margin, corner.y() + margin)

    def _request_param_refresh(self) -> None:
        """(Re)fetch the full parameter list - the Parameters tab's only
        entry point to the vehicle. Read-only: this only ever sends
        PARAM_REQUEST_LIST, never PARAM_SET."""
        if not (self.worker and self.worker.isRunning()):
            self.page_params.refresh_unavailable(
                "Not connected - connect to a vehicle first.")
            return
        self.page_params.begin_refresh()
        if not self.worker.request_param_list():
            self.page_params.refresh_unavailable("Could not send PARAM_REQUEST_LIST.")

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------

    def _on_network_selected(self, ip: str):
        """Fired the moment a Network preset is picked in the top strip - independent of
        Connect/Disconnect, so the map bridge and FPV stream re-target immediately rather
        than waiting for a MAVLink (re)connect. Port/protocol are untouched: they're a
        property of the link, not of which Wi-Fi network is active.

        The wfb-ng radio preset is the exception: telemetry and video both arrive on
        this machine (the radio's ground side), so the FPV tab switches to the wfb-ng
        source and the TCP map bridge is left alone - it has no host on the radio link."""
        if self.top_strip.is_wfb_selected():
            if hasattr(self, 'page_fpv') and self.page_fpv:
                self.page_fpv.select_source(VideoFeedWidget.SRC_WFB)
            return
        self._radxa_host = ip
        if hasattr(self, 'map_listener') and self.map_listener:
            self.map_listener.set_tcp_host(ip)
        if hasattr(self, 'page_fpv') and self.page_fpv:
            if self.page_fpv.combo_source.currentIndex() == VideoFeedWidget.SRC_WFB:
                self.page_fpv.select_source(VideoFeedWidget.SRC_DRONE_FPV)
            self.page_fpv.set_stream_host(ip)

    def _connect_to_endpoint(self, ip: str, port: int, protocol: str = "udp"):
        if self.worker and self.worker.isRunning():
            self._disconnect_from_endpoint()

        proto_str = protocol.lower().strip()
        if proto_str == "udpin":
            # wfb-ng radio: listen locally; the radio's ground side sends MAVLink here.
            target = f"udpin:0.0.0.0:{port} (wfb-ng radio)"
        else:
            target = f"{'udpout:' if proto_str == 'udp' else 'tcp:'}{ip}:{port}"
        self.console.log_info(f"Connecting to {target} (SysID 255)...")
        self.page_terminal.log_info(f"Connecting to {target} (SysID 255)...")

        if proto_str != "udpin" and hasattr(self, 'map_listener') and self.map_listener:
            self.map_listener.set_tcp_host(ip)

        self.worker = MAVLinkWorker(host=ip, port=port, protocol=proto_str, source_system=255)
        self.worker.telemetry_updated.connect(self._on_telemetry_updated)
        self.worker.connection_changed.connect(self._on_connection_changed)
        self.worker.statustext_received.connect(self._on_statustext_received)
        self.worker.command_ack_received.connect(self._on_command_ack_received)
        self.worker.rates_updated.connect(self._on_rates_updated)
        self.worker.param_value_received.connect(self.page_params.on_param_value)
        self.worker.motor_param_received.connect(self.page_motors.on_motor_param)
        self.worker.start()
        # A fresh connect may be a different vehicle with a different
        # parameter set - whatever the Parameters tab showed before is no
        # longer trustworthy. Cleared here, and only re-fetched if that tab
        # is open (see _on_view_changed) or the operator clicks Refresh.
        self.page_params.begin_refresh()
        self.page_params.refresh_unavailable(
            "Not fetched yet - open the Parameters tab or click Refresh.")

    def _disconnect_from_endpoint(self):
        if self.worker and self.worker.isRunning():
            self.console.log_warning("Disconnecting from SBC...")
            self.page_terminal.log_warning("Disconnecting from SBC...")
            self.worker.disconnect_endpoint()
            self.worker = None
            self.top_strip.set_connection_state(False, "Disconnected")
            self.toast.show_message("Disconnected from Drone", "#da3633")
            self.page_params.set_connected(False)

    def _request_motor_range(self) -> None:
        """Read whichever PWM_MAIN_* parameters the motor scale still lacks.
        Single reads (not the full list), so the Parameters tab is undisturbed;
        re-sent on connect and when the Motors tab opens, since UDP can drop."""
        from core.motor_range import SHARED
        if self.worker and self.worker.isRunning() and not SHARED.known:
            self.worker.request_motor_range_params(SHARED.missing() or None)

    def _on_connection_changed(self, connected: bool, message: str):
        self.top_strip.set_connection_state(connected, message)
        if not connected:
            self.page_slam.flight_bar.set_state(False, False)
        self.page_params.set_connected(connected)
        if connected:
            # A new link may be a different vehicle: forget the old motor scale,
            # then read it once the stream has settled.
            from core.motor_range import SHARED
            SHARED.reset()
            self.page_motors.set_range(SHARED)
            QTimer.singleShot(1500, self._request_motor_range)
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
        self.page_fpv.set_wfb_ports(cfg.video.wfb_rtp_port, cfg.video.wfb_ts_port)
        self.audio.set_config(cfg.audio)
        self.actuator.set_endpoint(cfg.actuator.host, cfg.actuator.port)
        msg = ("Settings saved. Command limits, audio, stream URL and actuator IP applied "
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

        # Only genuine rejections/failures are sounded - not IN_PROGRESS (5,
        # more is coming for this same dispatch) or CANCELLED (6, superseded
        # by a newer command, not a failure of this one). An accepted command
        # already announces itself by the aircraft doing what was asked, and a
        # tone on every ACK would cover the telemetry stream in beeps.
        if result_code in (1, 2, 3, 4) or result_code >= 7:
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
        elif result_code == 3:  # UNSUPPORTED
            msg = f"[UNSUPPORTED] PX4 ACK: {cmd_name} UNSUPPORTED (Pixhawk does not recognise this command)"
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message(f"[UNSUPPORTED] {cmd_name} by PX4", "#da3633", 5000)
        elif result_code == 4:  # FAILED
            msg = f"[FAILED] PX4 ACK: {cmd_name} FAILED (Command was accepted but execution failed)"
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message(f"[FAILED] {cmd_name} by PX4", "#da3633", 5000)
        elif result_code == 5:  # IN_PROGRESS
            # Not a rejection: PX4 is still executing this dispatch and a
            # real final ACK (ACCEPTED/FAILED/...) for the same command is
            # still expected - see the (token, result_code) dedup fix in
            # mavlink_worker.py that lets that follow-up ACK actually arrive.
            msg = f"[IN PROGRESS] PX4 ACK: {cmd_name} IN PROGRESS (still executing, final result to follow)"
            self.console.log_info(msg)
            self.page_terminal.log_info(msg)
            self.toast.show_message(f"[IN PROGRESS] {cmd_name}", "#58a6ff", 3000)
        elif result_code == 6:  # CANCELLED
            msg = f"[CANCELLED] PX4 ACK: {cmd_name} CANCELLED (Superseded by a newer command before it finished)"
            self.console.log_warning(msg)
            self.page_terminal.log_warning(msg)
            self.toast.show_message(f"[CANCELLED] {cmd_name}", "#d29922", 4000)
        else:
            msg = f"[REJECTED] PX4 ACK: {cmd_name} {result_str} (Code {result_code})"
            self.console.log_error(msg)
            self.page_terminal.log_error(msg)
            self.toast.show_message(msg, "#da3633", 4000)

    def _on_ros2_status_updated(self, msg: str):
        """Route every ROS2MapListener status line to the console at a
        severity that matches what actually happened - this used to go
        through log_info() unconditionally, so a real connection loss (or a
        frame decode error) read in the exact same neutral grey as "map
        listener started" and a healthy reconnect, with no visual signal
        that anything had gone wrong."""
        text = f"ROS 2: {msg}"
        if msg.startswith("Connected to"):
            self.console.log_success(text)
            self.page_terminal.log_success(text)
        elif msg.startswith("Disconnected from"):
            self.console.log_error(text)
            self.page_terminal.log_error(text)
        elif "Invalid" in msg or "decode error" in msg or "reconnecting" in msg:
            # Recoverable, per-frame issues (one bad frame skipped, one
            # reconnect) - amber, not the same red as a real disconnect.
            self.console.log_warning(text)
            self.page_terminal.log_warning(text)
        elif "error" in msg.lower():
            self.console.log_error(text)
            self.page_terminal.log_error(text)
        else:
            self.console.log_info(text)
            self.page_terminal.log_info(text)

    def _on_map_received_dispatch(self, grid: np.ndarray, res: float, ox: float,
                                  oy: float, source: str, image=None):
        self.latest_map_data = (grid, res, ox, oy)
        self.page_slam.on_ros2_map_received(grid, res, ox, oy, source, image)

    def _update_shell_cards(self, t: TelemetrySnapshot) -> None:
        """Feed the persistent right-column cards and the marker (read-only views)."""
        self.telemetry_card.update_from(t)
        self.commands_card.reflect(t.armed, t.flight_mode)
        self.vehicle_marker.set_heading(t.heading)
        vc = self.vehicle_card
        vc.set_sysid(t.system_id)
        have_batt = t.connected and t.battery_voltage > 0.5
        vc.set_tone("bat", "" if not have_batt else ("bad" if t.battery_percent <= 20 else
                    "warn" if t.battery_percent <= 35 else "ok"), "Battery")
        has_fix = t.connected and str(t.fix_type) not in ("NO_FIX", "NO FIX", "")
        vc.set_tone("gps", "ok" if has_fix else "", "GPS")
        vc.set_tone("rc", "" if not t.rc_receiver_present else
                    ("ok" if t.rc_receiver_healthy else "bad"), "RC link")
        vc.set_tone("sat", "ok" if t.ekf2_vision_fused and t.d435i_vio_age < 1.0 else "", "Vision / EKF2")
        if self.stack.currentIndex() == 0:
            self.route_card.set_sub("Quadcopter \u00b7 connected" if t.connected
                                    else "Quadcopter \u00b7 not connected")

    def _on_telemetry_updated(self, t: TelemetrySnapshot):
        self.last_telemetry = t
        self.hud.update_telemetry(t)
        self.top_strip.update_telemetry(t)
        self._update_shell_cards(t)
        # Sidebar footer must be driven from the same event as the header
        # badge above, not only from the 30Hz _on_ui_tick - otherwise the two
        # can show contradictory arm/mode state for up to one tick (e.g. right
        # after connect, before the first tick fires).
        self.sidebar.set_flight_state(t.flight_mode, t.armed)
        self.sidebar.set_instruments(t.ground_speed, t.altitude)
        self.page_slam.update_pose(t.x, t.y, t.heading)
        self.page_slam.canvas.set_position_uncertainty(
            t.pos_var_n, t.pos_var_e, t.pos_var_time)
        self.page_slam.set_armed_state(t.armed)
        self.page_slam.flight_bar.set_state(t.connected, t.armed, t.flight_mode)
        self._track_mode_source(t.flight_mode)
        self._arb_tick(t)
        self._sync_mode_box(self.combo_modes, t.flight_mode)
        t.check_motor_staleness()
        self.page_motors.update_pwms(t.motor_pwms, age_s=t.motor_age)

        # Safety state machine: a mid-flight `disarm` was redirected to
        # AUTO.LAND (see _cmd_disarm) - watch for PX4's own landed_state to
        # confirm real touchdown before actually cutting power.
        if self._pending_autodisarm_after_land and t.is_on_ground_for_disarm():
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
        gap = self.ui_stall.tick()
        if gap is not None:
            self._note_ui_stall(gap)
            self.console.log_warning(
                f"UI thread stalled {gap * 1000:.0f} ms (worst "
                f"{self.ui_stall.stats.worst_gap_s * 1000:.0f} ms) - "
                "something blocked the cockpit")
        t = self.last_telemetry
        t.check_vision_staleness(max_age_sec=self.settings.alerts.vision_stale_s)
        # Re-evaluated every tick, the same way vision staleness is above -
        # without this, position_stale/position_age only ever reflected the
        # instant a LOCAL_POSITION_NED packet last arrived, and could never
        # age past that if the stream stopped (the Diagnostics tab's Position
        # tile would then show "fresh" forever during a real dropout).
        t.check_position_staleness()
        self.top_strip.update_telemetry(t)
        self._update_shell_cards(t)
        self.page_fpv.set_telemetry(t)
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
                    self._note_gcs_mode_cmd()
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
                # A reached waypoint that is (within a wall-snap) the next
                # mission stop retires that stop, so a later detour does not
                # send the vehicle back to it.
                remaining = getattr(self, "_mission_stops_remaining", None)
                if remaining and math.hypot(remaining[0][0] - curr_target[0],
                                            remaining[0][1] - curr_target[1]) < 0.6:
                    done = remaining.pop(0)
                    if remaining:
                        self.console.log_success(
                            f"Mission stop reached ({done[0]:+.2f}, {done[1]:+.2f}) - "
                            f"{len(remaining)} to go")
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
        self._update_alarms(t)
        self._refresh_preflight()

    # -------------------------------------------------------------------------
    # Command Dispatch Helpers
    # -------------------------------------------------------------------------

    # -------------------------------------------------------------------------
    # Guided Action Confirmation  (ui/guided_confirm.GuidedConfirmBar)
    # -------------------------------------------------------------------------

    # _cmd_move_from_ui() removed with the Move (m) row: relative translation is
    # now issued from the flight terminal ("move <dx> <dy> <dz>") and by the
    # Tactical SLAM path executor, both of which call _cmd_move() directly.

    # -------------------------------------------------------------------------
    # A* Path Planning Execution & Flight Safety Handlers
    # -------------------------------------------------------------------------

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
                "  move <x> <y> <z>    - Relative move in meters: forward, right, down (of the drone)\n"
                "  moven <n> <e> <d>    - Same, but North / East / Down (world frame)\n"
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
        elif action in ("move", "moveb", "moven"):
            if len(parts) < 4:
                self.console.log_error(f"Usage: {action} <x> <y> <z> (e.g. {action} 1 0 0)")
                return
            try:
                dx = float(parts[1])
                dy = float(parts[2])
                dz = float(parts[3])
                self._cmd_move(dx, dy, dz, frame={"moveb": "body", "moven": "ned"}.get(action))
            except ValueError:
                self.console.log_error("Coordinates must be floating point numbers")
        else:
            self.console.log_warning(f"Unknown command: '{action}'. Type 'help' for instructions.")

    # -------------------------------------------------------------------------
    # Live collision avoidance  (answers from core/planner_worker.PlannerWorker)
    # -------------------------------------------------------------------------

    # -------------------------------------------------------------------------
    # Mission Progress  (ui/mission_progress.MissionProgressBar)
    # -------------------------------------------------------------------------

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
                self.audio.say("LIO lost", Severity.CRITICAL, key="vision")
            elif vio_ok and t.is_airborne:
                self.audio.say("LIO restored", Severity.OK, key="vision")
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
        for index in range(9):
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
            port = conn.udp_port if conn.protocol in ("udp", "udpin") else conn.tcp_port
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
        # Mirror the same optimistic reflection motor_widget.py already shows
        # on its own bars into the shared telemetry snapshot, so every OTHER
        # consumer of t.motor_pwms (the Diagnostics tab's M1-M4 tiles) shows
        # the same commanded value too - not just the Motors tab. Without
        # this, a real bench test could spin a motor and update one tab while
        # the other kept reading idle, since the Diagnostics grid only ever
        # reads t.motor_pwms directly and has no reflection of its own.
        from core.motor_range import SHARED
        # t.motor_pwms is OUTPUT-indexed (as SERVO_OUTPUT_RAW is), so the
        # commanded value goes on the output that feeds this motor.
        by_output = self._disarmed_output_pwms()
        by_output[SHARED.output_for_motor(motor_num)] = SHARED.pwm_for_throttle(
            motor_num, throttle_pct)
        t.motor_pwms = by_output

    @staticmethod
    def _disarmed_output_pwms() -> list:
        from core.motor_range import SHARED
        return [SHARED.out_dis[i] if SHARED.out_dis[i] is not None else 1000
                for i in range(4)]

    def _on_motor_test_stop(self) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.stop_all_motor_tests()
        self.last_telemetry.motor_pwms = self._disarmed_output_pwms()

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
        poller = getattr(self, "radxa_poller", None)
        if poller is not None:
            poller.stop()
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
    ap.add_argument("--protocol", choices=("udp", "tcp", "udpin"),
                    help="MAVLink transport (env GCS_PROTOCOL); udpin = listen on --port for "
                         "the wfb-ng radio link")
    ap.add_argument("--map-host", help="TCP map bridge host (env GCS_MAP_HOST)")
    ap.add_argument("--map-port", help="TCP map bridge port (env GCS_MAP_PORT)")
    ap.add_argument("--video-url", help="FPV stream URL (env GCS_VIDEO_URL)")
    ap.add_argument("--rviz-config", help="RViz2 layout path (env GCS_RVIZ_CONFIG)")
    ap.add_argument("--ui-scale", type=float,
                    help="UI scale factor, 0.75-3.0 (env GCS_UI_SCALE; "
                         "omit or 0 to detect from the display)")
    ap.add_argument("--no-audio", action="store_true",
                    help="start with audio alerts muted")
    ap.add_argument("--actuator-host", help="ESP32 servo IP (env GCS_ACTUATOR_HOST)")
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
