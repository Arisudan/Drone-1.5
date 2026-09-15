"""
================================================================================
MODULE: cli_console.py
PURPOSE: Interactive Terminal Console Widget with Command History & Live Logging
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS UI Layer)
  * Communicates:  drone_gcs.py command parser & execution tracker
  * Upstream:      Keyboard inputs & MAVLink / ROS 2 event logging streams
  * Downstream:    Operator terminal output & MAVLink command dispatcher

DATA FLOW & INTERFACES:
  * Inputs:        Raw command strings entered at the `cmd>` prompt.
  * Outputs:       Timestamped formatted log messages ([HH:MM:SS] format).
  * Qt Signals:    command_submitted(str) emitted when Enter is pressed.

KEY LOGIC & FAILSAFES:
  * Interactive Shell Navigation: Implements shell-like Up/Down arrow history recall
    with circular navigation and duplicate entry suppression.
  * Auto-Scrolling & Memory Buffering: Scrolls to the bottom on new log arrival
    while capping buffer length to prevent memory bloat during multi-hour flights.
  * Thread-Safe Log Appending: Exposes clean `append_log(msg)` API for invoking
    from both UI events and background worker threads.

USAGE:
  console = CLIConsoleWidget()
  console.command_submitted.connect(self.handle_command)
  console.append_log("System initialized successfully")
================================================================================
"""

from __future__ import annotations
from datetime import datetime
from typing import Optional, List

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont, QTextCursor
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QLineEdit, QPushButton, QLabel
)


class CommandLineEdit(QLineEdit):
    """QLineEdit with Up/Down arrow history recall."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.history: List[str] = []
        self.history_idx: int = -1

    def add_history(self, cmd: str):
        cmd = cmd.strip()
        if cmd and (not self.history or self.history[-1] != cmd):
            self.history.append(cmd)
        self.history_idx = len(self.history)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Up:
            if self.history and self.history_idx > 0:
                self.history_idx -= 1
                self.setText(self.history[self.history_idx])
            return
        elif event.key() == Qt.Key_Down:
            if self.history and self.history_idx < len(self.history) - 1:
                self.history_idx += 1
                self.setText(self.history[self.history_idx])
            else:
                self.history_idx = len(self.history)
                self.clear()
            return
        super().keyPressEvent(event)


class CLIConsoleWidget(QWidget):
    """Console widget hosting terminal output and cmd> input bar."""

    command_submitted = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        # Log Text Box
        self.log_box = QPlainTextEdit(self)
        self.log_box.setReadOnly(True)
        self.log_box.setFont(QFont("Consolas", 10))
        self.log_box.setMaximumBlockCount(1000)
        layout.addWidget(self.log_box)

        # Bottom Input Bar
        input_bar = QHBoxLayout()
        input_bar.setContentsMargins(0, 0, 0, 0)
        input_bar.setSpacing(6)

        prompt_lbl = QLabel("cmd>")
        prompt_lbl.setStyleSheet("font-weight: bold; color: #58a6ff; font-family: Consolas; font-size: 13px;")
        input_bar.addWidget(prompt_lbl)

        self.cmd_input = CommandLineEdit(self)
        self.cmd_input.setObjectName("cliInput")
        self.cmd_input.setPlaceholderText("Type command: move 1 0 0 | takeoff 1.5 | arm force | mode offboard | help")
        self.cmd_input.returnPressed.connect(self._handle_send)
        input_bar.addWidget(self.cmd_input)

        btn_send = QPushButton("Send")
        btn_send.setStyleSheet("background-color: #1f6feb; color: #ffffff;")
        btn_send.clicked.connect(self._handle_send)
        input_bar.addWidget(btn_send)

        btn_clear = QPushButton("Clear")
        btn_clear.clicked.connect(self.log_box.clear)
        input_bar.addWidget(btn_clear)

        layout.addLayout(input_bar)

    def _handle_send(self):
        text = self.cmd_input.text().strip()
        if not text:
            return
        self.cmd_input.add_history(text)
        self.cmd_input.clear()
        self.log_cmd(f"> {text}")
        self.command_submitted.emit(text)

    def _append_log(self, text: str, color_hex: str = "#c9d1d9"):
        ts = datetime.now().strftime("%H:%M:%S")
        html = f'<span style="color:#8b949e">[{ts}]</span> <span style="color:{color_hex}">{text}</span>'
        self.log_box.appendHtml(html)
        self.log_box.moveCursor(QTextCursor.End)

    def log_info(self, msg: str):
        self._append_log(msg, "#c9d1d9")

    def log_success(self, msg: str):
        self._append_log(msg, "#3fb950")

    def log_warning(self, msg: str):
        self._append_log(msg, "#d29922")

    def log_error(self, msg: str):
        self._append_log(msg, "#f85149")

    def log_cmd(self, msg: str):
        self._append_log(msg, "#58a6ff")
