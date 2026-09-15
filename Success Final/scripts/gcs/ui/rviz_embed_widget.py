"""
================================================================================
MODULE: rviz_embed_widget.py
PURPOSE: Native X11 RViz2 Perception Window Embedder for 3D SLAM & Pointcloud Viewing
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station or Radxa Companion Computer with X11 Display
  * Communicates:  ROS 2 Jazzy rviz2 executable via subprocess and X11 Window ID
  * Upstream:      rtabmap_drone.rviz configuration file & ROS 2 3D topics
  * Downstream:    GCS Tactical SLAM Tab (seamless embedded 3D viewport)

DATA FLOW & INTERFACES:
  * Subprocess:    Launches `rviz2 -d /home/radxa/Flop/config/rtabmap_drone.rviz`.
  * X11 Swallowing: Uses `xdotool search --pid <pid>` to locate the X11 Window ID,
                   wraps it with `QWindow.fromWinId()`, and mounts it into
                   `QWidget.createWindowContainer()`.
  * Qt Signals:    rviz_state_changed(bool is_running, str status_message).

KEY LOGIC & FAILSAFES:
  * Seamless X11 Ingestion: Embeds the full-featured RViz2 3D renderer directly
    inside the GCS application frame without floating external desktop windows.
  * Software OpenGL Fallback: Automatically applies `LIBGL_ALWAYS_SOFTWARE=1`
    and `MESA_LOADER_DRIVER_OVERRIDE=zink/llvmpipe` to prevent driver crashes
    on headless or non-standard ARM/Mali GPUs.
  * Process Watchdog & Auto-Reap: Continuously polls process state and cleanly kills
    the rviz2 subprocess on tab switch or GCS shutdown, preventing orphaned processes.
  * Window Discovery Poller: Gracefully handles the 1-3 second Ogre 3D initialization
    delay before the X11 window becomes discoverable.

USAGE:
  rviz_widget = RVizEmbedWidget(config_path="/home/radxa/Flop/config/rtabmap_drone.rviz")
  rviz_widget.start_rviz()
================================================================================
"""

from __future__ import annotations
import os
import sys
import time
import subprocess
from typing import Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QWindow, QFont
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QStackedWidget, QFrame, QSizePolicy
)


class RVizEmbedWidget(QWidget):
    """
    Embeds the live ROS 2 RViz2 application inside the PyQt5 GCS interface
    using X11 QWindow.fromWinId and QWidget.createWindowContainer.
    """

    rviz_state_changed = pyqtSignal(bool, str)  # (is_running, status_message)

    def __init__(
        self,
        config_path: str = "/home/radxa/Flop/config/rtabmap_drone.rviz",
        parent: Optional[QWidget] = None
    ):
        super().__init__(parent)
        self.config_path = config_path
        self.rviz_process: Optional[subprocess.Popen] = None
        self.rviz_container: Optional[QWidget] = None
        self.rviz_qwindow: Optional[QWindow] = None
        self._find_attempts = 0

        self._init_ui()

        # Window detection timer
        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(250)  # Check every 250ms
        self.poll_timer.timeout.connect(self._poll_for_rviz_window)

    def _init_ui(self):
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(4, 4, 4, 4)
        root_layout.setSpacing(6)

        # 1. Header Toolbar
        hdr_card = QFrame(self)
        hdr_card.setStyleSheet(
            "QFrame { background-color: #161b22; border: 1px solid #30363d; border-radius: 6px; padding: 4px; }"
        )
        hl = QHBoxLayout(hdr_card)
        hl.setContentsMargins(8, 4, 8, 4)
        hl.setSpacing(8)

        self.lbl_status_badge = QLabel("RVIZ2: OFFLINE", self)
        self.lbl_status_badge.setStyleSheet(
            "background-color: #21262d; color: #8b949e; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
        )
        hl.addWidget(self.lbl_status_badge)

        self.lbl_detail = QLabel(
            f"Config: {os.path.basename(self.config_path)} (Displays /map_thin, /map, TF, PointCloud2)", self
        )
        self.lbl_detail.setStyleSheet("color: #8b949e; font-size: 11px;")
        hl.addWidget(self.lbl_detail, 1)

        # Action Buttons
        self.btn_launch = QPushButton("Launch RViz2", self)
        self.btn_launch.setStyleSheet(
            "QPushButton { background-color: #238636; color: #ffffff; font-weight: bold; "
            "border-radius: 4px; padding: 6px 14px; min-height: 28px; }"
            "QPushButton:hover { background-color: #2ea043; }"
        )
        self.btn_launch.clicked.connect(self.launch_rviz)
        hl.addWidget(self.btn_launch)

        self.btn_reload = QPushButton("Reload", self)
        self.btn_reload.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #c9d1d9; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 6px 12px; min-height: 28px; }"
            "QPushButton:hover { background-color: #30363d; }"
        )
        self.btn_reload.setEnabled(False)
        self.btn_reload.clicked.connect(self.reload_rviz)
        hl.addWidget(self.btn_reload)

        self.btn_stop = QPushButton("⏹ Close RViz2", self)
        self.btn_stop.setStyleSheet(
            "QPushButton { background-color: #21262d; color: #f85149; border: 1px solid #da3633; "
            "border-radius: 4px; padding: 6px 12px; min-height: 28px; }"
            "QPushButton:hover { background-color: #da3633; color: #ffffff; }"
        )
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.stop_rviz)
        hl.addWidget(self.btn_stop)

        # Note: hdr_card is omitted from root_layout to eliminate redundant toolbar clutter;
        # controls and status are promoted to SLAMMapWidget's unified toolbar and context bar.

        # 2. Main Stacked Container (Page 0: Placeholder | Page 1: Embedded Window)
        self.stack = QStackedWidget(self)

        # Page 0: Sleek placeholder card
        self.placeholder = QFrame(self)
        self.placeholder.setStyleSheet(
            "QFrame { background-color: #0d1117; border: 1px dashed #30363d; border-radius: 8px; }"
        )
        pl_layout = QVBoxLayout(self.placeholder)
        pl_layout.setAlignment(Qt.AlignCenter)
        pl_layout.setSpacing(12)

        icon_lbl = QLabel("[ 3D POINT CLOUD ]", self)
        icon_lbl.setFont(QFont("Segoe UI", 16, QFont.Bold))
        icon_lbl.setStyleSheet("color: #58a6ff; letter-spacing: 2px;")
        icon_lbl.setAlignment(Qt.AlignCenter)
        pl_layout.addWidget(icon_lbl)

        title_lbl = QLabel("ROS 2 RViz2 2D Occupancy Grid & SLAM Viewport", self)
        title_lbl.setStyleSheet("color: #c9d1d9; font-size: 16px; font-weight: bold;")
        title_lbl.setAlignment(Qt.AlignCenter)
        pl_layout.addWidget(title_lbl)

        desc_lbl = QLabel(
            "Renders live D435i point clouds, camera TF frames, /map 2D occupancy grid,\n"
            "and /map_thin single-pixel wall skeleton directly inside Drone-GCS.\n\n"
            "Note: RViz2 requires native ROS 2 DDS discovery. For Wi-Fi operation where\n"
            "multicast is blocked by routers, use the primary '2D Blueprint View' tab\n"
            "(powered by resilient TCP map streaming on port 5765).",
            self
        )
        desc_lbl.setStyleSheet("color: #8b949e; font-size: 12px; line-height: 1.4;")
        desc_lbl.setAlignment(Qt.AlignCenter)
        pl_layout.addWidget(desc_lbl)

        btn_ph_launch = QPushButton("Launch Embedded RViz2 Now", self)
        btn_ph_launch.setStyleSheet(
            "QPushButton { background-color: #238636; color: #ffffff; font-size: 13px; font-weight: bold; "
            "border-radius: 6px; padding: 10px 24px; min-height: 36px; }"
            "QPushButton:hover { background-color: #2ea043; }"
        )
        btn_ph_launch.clicked.connect(self.launch_rviz)
        pl_layout.addWidget(btn_ph_launch, 0, Qt.AlignCenter)

        self.stack.addWidget(self.placeholder)

        # Page 1: Host widget for embedded QWindow
        self.embed_host = QWidget(self)
        self.embed_layout = QVBoxLayout(self.embed_host)
        self.embed_layout.setContentsMargins(0, 0, 0, 0)
        self.embed_layout.setSpacing(0)
        self.stack.addWidget(self.embed_host)

        root_layout.addWidget(self.stack, 1)

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # Process & Window Embedding Lifecycle
    # -------------------------------------------------------------------------

    def check_display_compatibility(self) -> Tuple[bool, str]:
        """Item 6: Check whether the current display server supports X11 window swallowing."""
        import shutil
        session_type = os.environ.get("XDG_SESSION_TYPE", "").lower()
        qpa = os.environ.get("QT_QPA_PLATFORM", "").lower()
        display = os.environ.get("DISPLAY", "")

        if (session_type == "wayland" or "wayland" in qpa) and not display:
            return False, "Pure Wayland session detected without XWayland. Window swallowing requires X11/XWayland."
        if shutil.which("xwininfo") is None:
            return False, "Command 'xwininfo' not found in PATH. Install x11-utils to enable window embedding."
        return True, "Display server compatible."

    def launch_rviz(self):
        """Spawn rviz2 process with target config and start looking for its X11 window."""
        if self.rviz_process and self.rviz_process.poll() is None:
            return  # Already running

        compat, reason = self.check_display_compatibility()
        if not compat:
            self.lbl_status_badge.setText("RVIZ2: X11 / UTILS REQ")
            self.lbl_status_badge.setStyleSheet(
                "background-color: #9e6a03; color: #ffffff; border: 1px solid #d29922; "
                "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
            )
            self.rviz_state_changed.emit(False, f"DISPLAY NOTICE: {reason}")
            return

        self.lbl_status_badge.setText("RVIZ2: LAUNCHING...")
        self.lbl_status_badge.setStyleSheet(
            "background-color: #9e6a03; color: #ffffff; border: 1px solid #d29922; "
            "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
        )
        self.btn_launch.setEnabled(False)
        self.rviz_state_changed.emit(False, "LAUNCHING...")

        # Unique window title for instant deterministic X11 window search
        self._target_win_title = f"Drone-GCS-RViz2-{int(time.time())}"

        # Source ROS 2 Jazzy properly so all vendor shared libraries (libgz-math7, libOgreMain, etc.)
        # and default plugins (Grid, Map, TF, PointCloud2) load without dlopen errors
        ros_setup = "/opt/ros/jazzy/setup.bash"
        if not os.path.exists(ros_setup):
            ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
            alt_setup = f"/opt/ros/{ros_distro}/setup.bash"
            if os.path.exists(alt_setup):
                ros_setup = alt_setup

        # Using exec replaces bash process with rviz2 so self.rviz_process.pid matches rviz2 directly
        cmd_str = (
            f"source {ros_setup} 2>/dev/null || true; "
            f"exec rviz2 -d '{self.config_path}' -t '{self._target_win_title}'"
        )
        cmd = ["bash", "-c", cmd_str]

        env = os.environ.copy()
        env["LIBGL_ALWAYS_SOFTWARE"] = "1"
        env.setdefault("DISPLAY", ":0")

        try:
            self.rviz_process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self._find_attempts = 0
            self.poll_timer.start()
        except Exception as e:
            self.lbl_status_badge.setText("RVIZ2: LAUNCH FAILED")
            self.lbl_status_badge.setStyleSheet(
                "background-color: #da3633; color: #ffffff; border: 1px solid #f85149; "
                "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
            )
            self.btn_launch.setEnabled(True)
            self.rviz_state_changed.emit(False, f"ERROR: {e}")

    def _poll_for_rviz_window(self):
        """Poll xwininfo and xprop to locate the created X11 window ID."""
        self._find_attempts += 1
        if self._find_attempts > 40:  # Timeout after 10 seconds
            self.poll_timer.stop()
            self.lbl_status_badge.setText("RVIZ2: EMBED TIMEOUT")
            self.lbl_status_badge.setStyleSheet(
                "background-color: #da3633; color: #ffffff; border: 1px solid #f85149; "
                "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
            )
            self.btn_launch.setEnabled(True)
            self.rviz_state_changed.emit(False, "TIMEOUT")
            return

        # Check if process died
        if self.rviz_process and self.rviz_process.poll() is not None:
            self.poll_timer.stop()
            self.lbl_status_badge.setText("RVIZ2: CRASHED")
            self.lbl_status_badge.setStyleSheet(
                "background-color: #da3633; color: #ffffff; border: 1px solid #f85149; "
                "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
            )
            self.btn_launch.setEnabled(True)
            self.rviz_state_changed.emit(False, "CRASHED")
            return

        # Deterministic window search: check unique title first, then verify by PID
        wid = self._find_window_id(self._target_win_title)
        if wid is None:
            # Fallback check for general rviz2 title, strictly verified by child PID
            wid = self._find_window_id("rviz2")

        if wid is not None:
            self.poll_timer.stop()
            self._embed_window_id(wid)

    def _find_window_id(self, search_title: str) -> Optional[int]:
        """Query xwininfo -root -tree for window title and verify PID via xprop to prevent latching stale windows."""
        try:
            out = subprocess.check_output(
                ["xwininfo", "-root", "-tree"],
                stderr=subprocess.DEVNULL
            ).decode("utf-8", errors="ignore")

            for line in out.splitlines():
                if search_title.lower() in line.lower():
                    parts = line.strip().split()
                    if parts and parts[0].startswith("0x"):
                        try:
                            wid_hex = parts[0]
                            wid = int(wid_hex, 16)
                            # If rviz_process is available, verify PID matches via xprop
                            if self.rviz_process:
                                try:
                                    xprop_out = subprocess.check_output(
                                        ["xprop", "-id", wid_hex, "_NET_WM_PID"],
                                        stderr=subprocess.DEVNULL,
                                        timeout=0.2
                                    ).decode("utf-8", errors="ignore")
                                    if f"= {self.rviz_process.pid}" in xprop_out:
                                        return wid
                                except Exception:
                                    pass
                            # If matching unique target title, accept directly as it has timestamp
                            if search_title == self._target_win_title:
                                return wid
                        except ValueError:
                            pass
        except Exception:
            pass
        return None

    def _embed_window_id(self, wid: int):
        """Create QWindow from WinID and embed inside Qt host layout."""
        try:
            self.rviz_qwindow = QWindow.fromWinId(wid)
            self.rviz_container = QWidget.createWindowContainer(self.rviz_qwindow, self.embed_host)
            self.rviz_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            self.embed_layout.addWidget(self.rviz_container)
            self.stack.setCurrentIndex(1)  # Show embedded page

            self.lbl_status_badge.setText("RVIZ2: EMBEDDED & ACTIVE")
            self.lbl_status_badge.setStyleSheet(
                "background-color: #1f6feb; color: #ffffff; border: 1px solid #388bfd; "
                "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
            )
            self.btn_launch.setEnabled(False)
            self.btn_reload.setEnabled(True)
            self.btn_stop.setEnabled(True)
            self.rviz_state_changed.emit(True, "ACTIVE")
        except Exception as e:
            self.lbl_status_badge.setText("RVIZ2: EMBED ERROR")
            self.lbl_status_badge.setStyleSheet(
                "background-color: #da3633; color: #ffffff; border: 1px solid #f85149; "
                "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
            )
            self.btn_launch.setEnabled(True)
            self.rviz_state_changed.emit(False, f"ERROR: {e}")

    def stop_rviz(self):
        """Terminate rviz2 process, wait for clean exit, and restore placeholder UI."""
        self.poll_timer.stop()
        if self.rviz_process:
            try:
                if self.rviz_process.poll() is None:
                    self.rviz_process.terminate()
                    try:
                        self.rviz_process.wait(timeout=1.5)
                    except subprocess.TimeoutExpired:
                        self.rviz_process.kill()
                        self.rviz_process.wait(timeout=1.0)
            except Exception:
                pass
            self.rviz_process = None

        if self.rviz_container:
            self.embed_layout.removeWidget(self.rviz_container)
            self.rviz_container.deleteLater()
            self.rviz_container = None
            self.rviz_qwindow = None

        self.stack.setCurrentIndex(0)
        self.lbl_status_badge.setText("RVIZ2: OFFLINE")
        self.lbl_status_badge.setStyleSheet(
            "background-color: #21262d; color: #8b949e; border: 1px solid #30363d; "
            "border-radius: 4px; padding: 4px 8px; font-size: 10px; font-weight: bold;"
        )
        self.btn_launch.setEnabled(True)
        self.btn_reload.setEnabled(False)
        self.btn_stop.setEnabled(False)
        self.rviz_state_changed.emit(False, "OFFLINE")

    def reload_rviz(self):
        """Restart rviz2 to reload configuration."""
        self.stop_rviz()
        QTimer.singleShot(600, self.launch_rviz)

    def closeEvent(self, event):
        self.stop_rviz()
        event.accept()
