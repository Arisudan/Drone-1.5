"""
================================================================================
MODULE: styles.py
PURPOSE: Tactical Aviation Matte-Dark QSS (Qt Style Sheet) Design System
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station & Radxa SBC (GCS UI Design System)
  * Communicates:  QApplication global stylesheet and all custom QWidget subclasses
  * Upstream:      Aviation UI design tokens, color palettes, and typography
  * Downstream:    drone_gcs.py, radxa_monitor.py, and all child widgets

DATA FLOW & INTERFACES:
  * Export:        `DARK_STYLESHEET` (multi-line CSS/QSS string).
  * Consumed By:   `app.setStyleSheet(DARK_STYLESHEET)`.

KEY LOGIC & FAILSAFES:
  * Tactical Dark Palette: Engineered for high daylight readability and night ops
    (#0d1117 canvas, #161b22 cards, #30363d borders, #58a6ff avionics blue).
  * Aviation Action Color System:
      - Primary Action (Takeoff / Offboard / Plan): Blue (#1f6feb / #388bfd)
      - Success / Safe Arming: Emerald Green (#238636 / #2ea043)
      - Warning / Hold: Amber (#9e6a03 / #bb8009)
      - Danger / Emergency Kill / Force Arm: High-Contrast Red (#da3633 / #f85149)
  * Hardware-Agnostic Typography: Prioritizes Segoe UI, DejaVu Sans, and Roboto
    with clean monospaced font declarations for flight telemetry.

USAGE:
  from ui.styles import DARK_STYLESHEET
  app.setStyleSheet(DARK_STYLESHEET)
================================================================================
"""

DARK_STYLESHEET = """
/* Global Application Styling */
QWidget {
    background-color: #0d1117;
    color: #c9d1d9;
    font-family: 'Segoe UI', 'DejaVu Sans', sans-serif;
    font-size: 13px;
    selection-background-color: #1f6feb;
    selection-color: #ffffff;
}

QMainWindow {
    background-color: #090d12;
}

/* Header & Connection Bar */
QFrame#topBar {
    background-color: #161b22;
    border-bottom: 1px solid #30363d;
    padding: 6px 12px;
}

QLabel#appTitle {
    font-size: 17px;
    font-weight: bold;
    color: #58a6ff;
    letter-spacing: 1px;
}

QLabel#appSubtitle {
    font-size: 11px;
    color: #8b949e;
}

/* Card Containers */
QFrame.cardFrame {
    background-color: #161b22;
    border: 1px solid #30363d;
    border-radius: 8px;
    padding: 8px;
}

QFrame.cardFrame:hover {
    border-color: #484f58;
}

QLabel.cardTitle {
    font-size: 11px;
    font-weight: bold;
    color: #8b949e;
    text-transform: uppercase;
    letter-spacing: 0.8px;
}

QLabel.cardValue {
    font-size: 18px;
    font-weight: bold;
    color: #f0f6fc;
}

/* Buttons */
QPushButton {
    background-color: #21262d;
    color: #c9d1d9;
    border: 1px solid #363b42;
    border-radius: 6px;
    padding: 7px 16px;
    font-weight: 600;
    font-size: 11px;
    min-height: 28px;
}

QPushButton:hover {
    background-color: #30363d;
    border-color: #8b949e;
    color: #ffffff;
}

QPushButton:pressed {
    background-color: #161b22;
}

QPushButton:disabled {
    background-color: #161b22;
    color: #484f58;
    border-color: #21262d;
}

/* ComboBox Style */
QComboBox {
    background-color: #161b22;
    color: #f0f6fc;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 5px 12px;
    font-weight: 600;
    font-size: 12px;
    min-height: 28px;
}

QComboBox:hover {
    border-color: #58a6ff;
}

QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 24px;
    border-left: 1px solid #30363d;
}

QComboBox QAbstractItemView {
    background-color: #161b22;
    color: #f0f6fc;
    selection-background-color: #1f6feb;
    selection-color: #ffffff;
    border: 1px solid #30363d;
    outline: none;
    padding: 4px;
}

/* Checkbox Style */
QCheckBox {
    color: #d29922;
    font-size: 12px;
    font-weight: 600;
    spacing: 8px;
}

QCheckBox::indicator {
    width: 18px;
    height: 18px;
    border-radius: 4px;
    border: 1px solid #d29922;
    background-color: #161b22;
}

QCheckBox::indicator:checked {
    background-color: #d29922;
    image: none;
}

QCheckBox::indicator:hover {
    border-color: #e3b341;
}

/* Specialized Buttons */
QPushButton#btnConnect {
    background-color: #238636;
    color: #ffffff;
    border: 1px solid #2ea043;
}
QPushButton#btnConnect:hover {
    background-color: #2ea043;
}

QPushButton#btnDisconnect {
    background-color: #da3633;
    color: #ffffff;
    border: 1px solid #f85149;
}
QPushButton#btnDisconnect:hover {
    background-color: #b62324;
}

QPushButton#btnArm {
    background-color: #238636;
    color: #ffffff;
    border: 1px solid #2ea043;
    font-size: 13px;
}
QPushButton#btnArm:hover {
    background-color: #2ea043;
}

QPushButton#btnForceArm {
    background-color: #9e6a03;
    color: #ffffff;
    border: 1px solid #d29922;
}
QPushButton#btnForceArm:hover {
    background-color: #bb8009;
}

QPushButton#btnDisarm {
    background-color: #da3633;
    color: #ffffff;
    border: 1px solid #f85149;
}
QPushButton#btnDisarm:hover {
    background-color: #b62324;
}

QPushButton#btnKill {
    background-color: #b62324;
    color: #ffffff;
    border: 1px solid #f85149;
    font-weight: 800;
}
QPushButton#btnKill:hover {
    background-color: #e5534b;
}

QPushButton#btnOffboard {
    background-color: #1f6feb;
    color: #ffffff;
    border: 1px solid #388bfd;
}
QPushButton#btnOffboard:hover {
    background-color: #388bfd;
}

/* Input Fields */
QLineEdit {
    background-color: #0d1117;
    color: #c9d1d9;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 6px 10px;
    font-size: 13px;
}

QLineEdit:focus {
    border-color: #58a6ff;
}

/* CLI Bar */
QLineEdit#cliInput {
    background-color: #090d12;
    color: #58a6ff;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 13px;
    font-weight: bold;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 8px 12px;
}

QLineEdit#cliInput:focus {
    border-color: #58a6ff;
}

/* Tabs */
QTabWidget::pane {
    border: 1px solid #30363d;
    background-color: #0d1117;
    border-radius: 0 0 8px 8px;
}

QTabBar::tab {
    background-color: #161b22;
    color: #8b949e;
    border: 1px solid #30363d;
    border-bottom: none;
    padding: 8px 18px;
    margin-right: 2px;
    font-weight: 600;
}

QTabBar::tab:selected {
    background-color: #0d1117;
    color: #58a6ff;
    border-top: 2px solid #58a6ff;
}

QTabBar::tab:hover:!selected {
    background-color: #21262d;
    color: #c9d1d9;
}

/* Terminal / Text Edit */
QTextEdit, QPlainTextEdit {
    background-color: #090d12;
    color: #c9d1d9;
    font-family: 'Consolas', 'DejaVu Sans Mono', monospace;
    font-size: 12px;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 6px;
}

/* Scrollbars */
QScrollBar:vertical {
    border: none;
    background: #0d1117;
    width: 8px;
    margin: 0px;
}
QScrollBar::handle:vertical {
    background: #30363d;
    min-height: 20px;
    border-radius: 4px;
}
QScrollBar::handle:vertical:hover {
    background: #58a6ff;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

/* Status Badges */
QLabel#badgeConnected {
    background-color: #1f6feb22;
    color: #58a6ff;
    border: 1px solid #1f6feb;
    border-radius: 4px;
    padding: 3px 8px;
    font-weight: bold;
    font-size: 11px;
}

QLabel#badgeDisconnected {
    background-color: #da363322;
    color: #f85149;
    border: 1px solid #da3633;
    border-radius: 4px;
    padding: 3px 8px;
    font-weight: bold;
    font-size: 11px;
}

QLabel#badgeArmed {
    background-color: #23863622;
    color: #3fb950;
    border: 1px solid #238636;
    border-radius: 4px;
    padding: 3px 8px;
    font-weight: bold;
    font-size: 11px;
}

QLabel#badgeDisarmed {
    background-color: #8b949e22;
    color: #8b949e;
    border: 1px solid #484f58;
    border-radius: 4px;
    padding: 3px 8px;
    font-weight: bold;
    font-size: 11px;
}
"""
