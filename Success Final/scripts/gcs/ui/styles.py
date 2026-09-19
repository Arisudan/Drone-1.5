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
  * Export:        `PALETTE` (dict of design tokens) and `DARK_STYLESHEET` (QSS).
  * Consumed By:   `app.setStyleSheet(DARK_STYLESHEET)`.

THE PALETTE IS DATA, NOT LITERALS:
  Every colour lives once in `PALETTE` and is substituted into the QSS below via
  string.Template ($token). Retheming the whole station is editing one dict -
  previously the same six hex values were copy-pasted across ~40 rules, so a
  palette change meant a find-and-replace that always missed something. QSS has
  no `$` of its own, so Template substitution is unambiguous here.

  Widgets that must set a stylesheet inline (state-dependent badges, for
  example) import PALETTE and index it rather than retyping hex.

TACTICAL NIGHT-OPS PALETTE:
  * Canvas is neutral near-black (#0a0a0a), not blue-tinted - so the coloured
    state indicators are the only chroma on screen and read instantly in a
    darkened room.
  * Phosphor green (#00cc00) is the primary accent: section titles, focus
    rings, active navigation, healthy state.
  * Chroma is rationed by consequence, not decoration:
      - Green  (#00aa00/#00dd00): arming, healthy, connected
      - Blue   (#5588ee):         commands that move the aircraft
      - Amber  (#ffcc00):         caution, overrides, degraded state
      - Red    (#cc0000/#ff4444): disarm, emergency cutoff, failure
      - Cyan   (#00d8d8):         live telemetry numbers
    Anything not in one of those five categories stays neutral grey, so a
    coloured control on screen always means something.

CONTROLS ARE OUTLINED, NOT FILLED:
  Buttons are a dark fill with a coloured border and coloured text. A screen of
  saturated blocks competes for attention; an outline conveys the same category
  while leaving the eye free to find the one control that is actually lit.

USAGE:
  from ui.styles import DARK_STYLESHEET, PALETTE
  app.setStyleSheet(DARK_STYLESHEET)
================================================================================
"""

import os
from string import Template

_ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")

# ─── Design tokens ──────────────────────────────────────────────────
# Edit here to retheme the entire ground station.

PALETTE = {
    # Surfaces. Lifted off pure black on purpose: an accent on #0a0a0a sits at
    # near-maximum luminance contrast, which is what made the previous theme
    # tiring to read for any length of time.
    "bg_window":     "#0d1117",   # application canvas
    "bg_panel":      "#161b22",   # cards, group boxes
    "bg_input":      "#21262d",   # inputs, buttons, tables
    "bg_raised":     "#2d333b",   # hover
    "bg_sunken":     "#090d12",   # consoles, status bar

    # Lines
    "border":        "#262c34",   # panel edges
    "border_soft":   "#30363d",   # control edges
    "border_hover":  "#484f58",

    # Chrome accent. Titles, headings, focus rings, the active workspace -
    # everything that is furniture rather than vehicle state.
    "accent":        "#58a6ff",
    "accent_bright": "#79c0ff",
    "accent_dim":    "#1f6feb",
    "accent_fill":   "#132741",

    # Semantic. Each hue means exactly one thing, and green is reserved for
    # vehicle state (armed, healthy, connected) so that seeing green on screen
    # is information rather than decoration.
    "info":          "#39c5cf",   # live telemetry numbers
    "ok":            "#3fb950",   # healthy / executed
    "ok_dim":        "#238636",
    "ok_bright":     "#56d364",
    "ok_fill":       "#122a19",
    "nav":           "#58a6ff",   # aircraft-moving commands
    "nav_dim":       "#1f6feb",
    "nav_fill":      "#10213a",
    "warn":          "#d29922",
    "warn_dim":      "#9e6a03",
    "warn_fill":     "#2b2109",
    "danger":        "#f85149",
    "danger_dim":    "#da3633",
    "danger_fill":   "#2d1113",

    # Type
    "text":          "#c9d1d9",
    "text_bright":   "#f0f6fc",
    "text_dim":      "#8b949e",
    "text_muted":    "#6e7681",

    # Fonts
    "font_ui":       "'Lato', 'Noto Sans', 'Segoe UI', 'DejaVu Sans', sans-serif",
    "font_mono":     "'Consolas', 'DejaVu Sans Mono', monospace",
    "font_gauge":    "'Noto Sans Mono', 'DejaVu Sans Mono', monospace",

    # Assets. Absolute paths: QSS url() resolves against the process working
    # directory, which the GCS does not control (it is launched from a shell
    # script, a .desktop file, or ros2 run).
    "arrow_down":      os.path.join(_ASSETS, "chevron_down.png"),
    "arrow_down_hot":  os.path.join(_ASSETS, "chevron_down_hover.png"),
    "arrow_down_off":  os.path.join(_ASSETS, "chevron_down_off.png"),
}

_QSS = """
/* ── Global ──────────────────────────────────────────────────────── */
QWidget {
    background-color: $bg_window;
    color: $text;
    font-family: $font_ui;
    font-size: 12px;
    selection-background-color: $accent_fill;
    selection-color: $accent_bright;
}

QMainWindow { background-color: $bg_window; }

/* Labels must not paint their own background: they inherit the window colour
   from the QWidget rule above, which shows as a darker rectangle wherever a
   label sits on a panel or card. */
QLabel { background-color: transparent; }

QToolTip {
    background-color: $bg_input;
    color: $text;
    border: 1px solid $border_soft;
    padding: 4px 8px;
}

/* ── Header & Connection Bar ─────────────────────────────────────── */
QFrame#topBar {
    background-color: $bg_panel;
    border-bottom: 1px solid $border;
    padding: 6px 12px;
}

QLabel#appTitle {
    font-size: 27px;
    font-weight: 900;
    color: $accent;
    letter-spacing: 4px;
}

QLabel#appSubtitle {
    font-size: 11px;
    font-weight: 600;
    color: $text_muted;
    letter-spacing: 2.6px;
}

/* ── Group Boxes ─────────────────────────────────────────────────── */
/* The primary container. A titled frame states what a cluster of controls
   is FOR, which a bare row of buttons never does. */
QGroupBox {
    color: $accent;
    background-color: $bg_panel;
    border: 1px solid $border;
    border-radius: 8px;
    margin-top: 13px;
    padding-top: 14px;
    font-weight: 600;
    font-size: 11px;
    letter-spacing: 2.4px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 10px;
    left: 8px;
}
/* (2) COMMANDS is the panel's primary action block, so its title is centred
   over the buttons it governs rather than tucked into a corner. */
QGroupBox#groupCentered::title { subcontrol-position: top center; left: 0; }

/* Borderless variant: used where several groups share one outer frame and
   their own borders would just add noise. */
QGroupBox#groupFlat {
    border: none;
    background-color: transparent;
    margin-top: 13px;
    padding-top: 12px;
}
QGroupBox#groupFlat::title { subcontrol-position: top center; left: 0; }

QGroupBox#groupDanger { color: $danger; }
QGroupBox#groupDanger::title { color: $danger; }

/* (3) Divider between two sections sharing one frame. */
QFrame#vDivider {
    background-color: $border_soft;
    min-width: 1px;
    max-width: 1px;
    border: none;
}

/* ── Card Containers ─────────────────────────────────────────────── */
QFrame.cardFrame {
    background-color: $bg_panel;
    border: 1px solid $border;
    border-radius: 8px;
    padding: 8px;
}
QFrame.cardFrame:hover { border-color: $border_soft; }

QLabel.cardTitle {
    font-size: 10px;
    font-weight: bold;
    color: $text_muted;
    text-transform: uppercase;
    letter-spacing: 1px;
}
QLabel.cardValue {
    font-size: 18px;
    font-weight: bold;
    color: $text_bright;
}

/* Section header above a cluster of group boxes. */
/* Card heading rendered inside its own frame, with a rule beneath it.
   QGroupBox paints its title across the top border, which reads fine for a
   single group but leaves every heading straddling the edge in a card grid. */
/* Headline figure on a statistics card. */
QLabel#statValue {
    color: $text_bright;
    font-size: 19px;
    font-weight: 800;
    font-family: $font_gauge;
}

QTableWidget {
    background-color: $bg_panel;
    alternate-background-color: #1a2029;
    gridline-color: $border;
    border: 1px solid $border;
    border-radius: 6px;
    font-size: 11px;
    selection-background-color: $accent_fill;
    selection-color: $accent_bright;
}
QTableWidget::item { padding: 4px 8px; border: none; }
QHeaderView::section {
    background-color: $bg_input;
    color: $text_muted;
    border: none;
    border-bottom: 1px solid $border_soft;
    padding: 6px 8px;
    font-weight: bold;
    font-size: 9px;
    letter-spacing: 1.2px;
}
QDateEdit {
    background-color: $bg_input;
    color: $text_bright;
    border: 1px solid $border_soft;
    border-radius: 5px;
    padding: 4px 8px;
    font-size: 11px;
    min-height: 22px;
}
QDateEdit::drop-down { width: 18px; border-left: 1px solid $border_soft; }
QDateEdit::down-arrow { image: url($arrow_down); width: 10px; height: 6px; margin-right: 5px; }

/* Header instrument readouts: caption above a gauge-face numeral, so speed
   and altitude read as instruments rather than as more badge text. */
QLabel#instCaption {
    color: $text_muted;
    font-size: 8px;
    font-weight: bold;
    letter-spacing: 1.2px;
}
QLabel#instValue {
    color: $info;
    font-size: 15px;
    font-weight: 800;
    font-family: $font_gauge;
}

QLabel#cardHeading {
    color: $accent;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 2px;
}
QFrame#hDivider {
    background-color: $border;
    min-height: 1px;
    max-height: 1px;
    border: none;
}

/* Vision-engine toggles in the footer. Off is muted; on takes the engine's
   own colour, so which overlays are live is readable at a glance. */
QPushButton#btnVision {
    background-color: $bg_input;
    color: $text_muted;
    border: 1px solid $border_soft;
    font-size: 10px;
    min-height: 24px;
    padding: 4px 16px;
}
QPushButton#btnVision:hover { color: $text; border-color: $border_hover; }
QPushButton#btnVision:disabled {
    background-color: $bg_panel;
    color: #3f464e;
    border-color: $border;
}
/* Per-engine colour keyed off a widget property: every button shares the
   objectName that carries the base style, so a prefix selector is not an
   option and Qt's attribute selector is. */
QPushButton#btnVision[visionRole="detect"]:checked { color: #39c5cf; border-color: #1f8f99; font-weight: bold; }
QPushButton#btnVision[visionRole="depth"]:checked  { color: #ff8800; border-color: #b35f00; font-weight: bold; }
QPushButton#btnVision[visionRole="face"]:checked   { color: #ffd000; border-color: #b39000; font-weight: bold; }
QPushButton#btnVision[visionRole="track"]:checked  { color: #ff66cc; border-color: #b3428e; font-weight: bold; }

QFrame#footerBar {
    background-color: $bg_panel;
    border: 1px solid $border;
    border-radius: 6px;
}
QLabel#footerNote {
    color: $text_muted;
    font-size: 10px;
    letter-spacing: 0.4px;
}

QLabel#sectionTitle {
    color: $accent;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 1.5px;
}
QLabel#fieldLabel    { color: $text;       font-size: 10px; font-weight: 600; }
QLabel#fieldSubLabel { color: $text_muted; font-size: 10px; }
QLabel#valueBright   { color: $text_bright; font-size: 11px; font-weight: bold; }
QLabel#valueMono     { color: $text_dim; font-size: 10px; font-family: $font_mono; }

/* Diagnostics field value. Monospaced so columns of numbers stay aligned as
   they change; the colour is set per-widget at runtime because it encodes a
   live value rather than a widget role. */
QLabel#diagValue {
    color: $text_bright;
    font-size: 11px;
    font-weight: bold;
    font-family: $font_mono;
}

/* ── Execution verifier ──────────────────────────────────────────── */
/* State badge. The objectName is swapped at runtime and the widget
   re-polished, so the colour tracks the tracker's own state machine. */
QLabel#execBadgeIdle {
    background-color: $bg_input; color: $text_muted;
    border: 1px solid $border_soft; border-radius: 4px;
    padding: 2px 6px; font-size: 10px; font-weight: bold; letter-spacing: 1px;
}
QLabel#execBadgeActive {
    background-color: $nav_fill; color: $nav;
    border: 1px solid $nav_dim; border-radius: 4px;
    padding: 2px 6px; font-size: 10px; font-weight: bold; letter-spacing: 1px;
}
QLabel#execBadgeOk {
    background-color: $ok_fill; color: $ok;
    border: 1px solid $ok_dim; border-radius: 4px;
    padding: 2px 6px; font-size: 10px; font-weight: bold; letter-spacing: 1px;
}
QLabel#execBadgeFail {
    background-color: $danger_fill; color: $danger;
    border: 1px solid $danger_dim; border-radius: 4px;
    padding: 2px 6px; font-size: 10px; font-weight: bold; letter-spacing: 1px;
}

/* Per-axis displacement tiles: caption above a monospaced number, so the
   digits sit in fixed columns and a changing value is readable at a glance. */
QLabel#tileCaption {
    color: $text_muted; font-size: 9px;
    font-weight: bold; letter-spacing: 0.5px;
}
QLabel#tileValue {
    color: $info; font-size: 15px;
    font-weight: bold; font-family: $font_mono;
}
/* Displacement readout. Monospaced so the columns hold still while the
   numbers move - the whole point of the panel is watching them change. */
QLabel#execAxis {
    color: $text_muted;
    font-size: 10px;
    font-weight: bold;
    font-family: $font_gauge;
    letter-spacing: 1.2px;
}
QLabel#execReadout {
    color: $text_bright;
    font-size: 15px;
    font-weight: 800;
    font-family: $font_gauge;
    letter-spacing: 0.6px;
}
QLabel#execCaption {
    color: $text_muted;
    font-size: 9px;
    font-weight: bold;
    letter-spacing: 1.4px;
}
QLabel#execMessage {
    color: $text_bright;
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.3px;
}

/* ── Buttons: dark fill, coloured edge, coloured text ────────────── */
QPushButton {
    background-color: $bg_input;
    color: $text;
    border: 1px solid $border_soft;
    border-radius: 5px;
    padding: 4px 12px;
    font-weight: 600;
    font-size: 10px;
    min-height: 22px;
}
QPushButton:hover   { background-color: $bg_raised; border-color: $border_hover; color: $text_bright; }
QPushButton:pressed { background-color: $bg_sunken; }
QPushButton:checked { background-color: $accent_fill; border-color: $accent_dim; color: $accent_bright; }
QPushButton:disabled {
    background-color: $bg_panel;
    color: #3a3a3a;
    border-color: $border;
}

/* Semantic variants. Applied with setObjectName(), never inline, so the
   consequence of a control lives in the design system rather than being
   retyped at each call site. */
/* Dispatching controls are filled, not outlined: these are the handful of
   buttons that actually command the aircraft, and a solid block separates
   them at a glance from the neutral controls that only change a view. White
   text on the fill keeps contrast high at every state colour. */
QPushButton#btnArm {
    background-color: $ok_dim; color: #ffffff;
    border: 1px solid $ok; font-size: 12px;
}
QPushButton#btnArm:hover { background-color: $ok; }

QPushButton#btnDisarm {
    background-color: $danger_dim; color: #ffffff;
    border: 1px solid $danger;
}
QPushButton#btnDisarm:hover { background-color: #ef3f37; }

/* Aircraft-moving commands. */
QPushButton#btnNav {
    background-color: $nav_dim; color: #ffffff;
    border: 1px solid $nav;
}
QPushButton#btnNav:hover { background-color: $accent; }

QPushButton#btnCaution {
    background-color: $warn_fill; color: $warn;
    border: 2px solid $warn_dim;
}
QPushButton#btnCaution:hover { background-color: #3a2f00; }

/* Irreversible. The only fully saturated red on screen. */
QPushButton#btnKill {
    background-color: #b62324; color: #ffffff;
    border: 1px solid $danger;
    font-weight: 800; letter-spacing: 1px;
}
QPushButton#btnKill:hover { background-color: $danger_dim; }

QPushButton#btnConnect {
    background-color: $ok_dim; color: #ffffff;
    border: 1px solid $ok;
}
QPushButton#btnConnect:hover { background-color: $ok; }

QPushButton#btnDisconnect {
    background-color: $danger_dim; color: #ffffff;
    border: 1px solid $danger;
}
QPushButton#btnDisconnect:hover { background-color: #ef3f37; }

QPushButton#btnForceArm {
    background-color: $warn_fill; color: $warn;
    border: 2px solid $warn_dim;
}
QPushButton#btnForceArm:hover { background-color: #3a2f00; }

QPushButton#btnOffboard {
    background-color: $nav_fill; color: $nav;
    border: 2px solid $nav_dim;
}
QPushButton#btnOffboard:hover { background-color: #16203a; }

/* ── Inputs ──────────────────────────────────────────────────────── */
QComboBox {
    background-color: $bg_input;
    color: $text_bright;
    border: 1px solid $border_soft;
    border-radius: 5px;
    padding: 4px 10px;
    font-weight: 600;
    font-size: 11px;
    min-height: 22px;
}
QComboBox:hover { border-color: $border_hover; }
QComboBox:focus { border-color: $accent_dim; }
QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 18px;
    border-left: 1px solid $border_soft;
}
/* Qt's stock indicator is a light-theme asset that disappears on a near-black
   background - every combo in the station looked like a plain text field.
   A QSS border-triangle does not work here (Qt paints the whole border box,
   which renders as a solid square), so this uses a generated chevron asset. */
QComboBox::down-arrow {
    image: url($arrow_down);
    width: 10px;
    height: 6px;
    margin-right: 5px;
}
QComboBox::down-arrow:hover    { image: url($arrow_down_hot); }
QComboBox::down-arrow:disabled { image: url($arrow_down_off); }
QComboBox::down-arrow:on       { image: url($arrow_down_hot); }
QComboBox QAbstractItemView {
    background-color: $bg_input;
    color: $text;
    selection-background-color: $accent_fill;
    selection-color: $accent_bright;
    border: 1px solid $border_soft;
    outline: none;
    padding: 4px;
}

QLineEdit {
    background-color: $bg_input;
    color: $text;
    border: 1px solid $border_soft;
    border-radius: 5px;
    padding: 4px 8px;
    font-size: 11px;
}
QLineEdit:hover { border-color: $border_hover; }
QLineEdit:focus { border-color: $accent_dim; color: $text_bright; }

QLineEdit#cliInput {
    background-color: $bg_sunken;
    color: $accent;
    font-family: $font_mono;
    font-size: 12px;
    font-weight: bold;
    border: 1px solid $border_soft;
    border-radius: 6px;
    padding: 7px 12px;
    min-height: 22px;
}
QLineEdit#cliInput:focus { border-color: $accent_dim; }

QLabel#cliPrompt {
    color: $accent;
    font-family: $font_mono;
    font-size: 13px;
    font-weight: bold;
}
QPushButton#btnCliSend {
    background-color: $nav_dim; color: #ffffff;
    border: 1px solid $nav;
    font-size: 11px; min-height: 28px; padding: 6px 20px;
}
QPushButton#btnCliSend:hover { background-color: $accent; }
QPushButton#btnCliClear {
    font-size: 11px; min-height: 28px; padding: 6px 18px;
}

/* ── Checkboxes ──────────────────────────────────────────────────── */
QCheckBox {
    color: $warn;
    font-size: 10px;
    font-weight: 600;
    spacing: 8px;
}
QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border-radius: 4px;
    border: 1px solid $warn_dim;
    background-color: $bg_input;
}
QCheckBox::indicator:checked { background-color: $warn; }
QCheckBox::indicator:hover   { border-color: $warn; }

/* ── Tabs ────────────────────────────────────────────────────────── */
QTabWidget::pane {
    border: 1px solid $border;
    background-color: $bg_panel;
    border-radius: 0 0 8px 8px;
}
QTabBar::tab {
    background-color: $bg_input;
    color: $text_muted;
    border: 1px solid $border;
    border-bottom: none;
    padding: 6px 14px;
    margin-right: 2px;
    font-weight: bold;
    font-size: 10px;
}
QTabBar::tab:selected {
    background-color: $bg_panel;
    color: $accent;
    border-top: 2px solid $accent;
}
QTabBar::tab:hover:!selected { background-color: $bg_raised; color: $text; }

/* ── Console ─────────────────────────────────────────────────────── */
QTextEdit, QPlainTextEdit {
    background-color: $bg_sunken;
    color: $text;
    font-family: $font_mono;
    font-size: 12px;
    border: 1px solid $border;
    border-radius: 6px;
    padding: 6px;
}

/* ── Progress ────────────────────────────────────────────────────── */
QProgressBar {
    background-color: $bg_input;
    border: 1px solid $border_soft;
    border-radius: 4px;
    text-align: center;
    color: $text_dim;
    font-size: 10px;
}
QProgressBar::chunk { background-color: $accent_dim; border-radius: 3px; }

/* ── Scrollbars ──────────────────────────────────────────────────── */
QScrollBar:vertical {
    border: none;
    background: $bg_window;
    width: 8px;
    margin: 0px;
}
QScrollBar::handle:vertical {
    background: $border_soft;
    min-height: 20px;
    border-radius: 4px;
}
QScrollBar::handle:vertical:hover { background: $border_hover; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }

/* ── Status Badges ───────────────────────────────────────────────── */
QLabel#badgeConnected {
    background-color: $accent_fill;
    color: $accent;
    border: 1px solid $accent_dim;
    border-radius: 4px;
    padding: 3px 8px;
    font-weight: bold;
    font-size: 10px;
}
QLabel#badgeDisconnected {
    background-color: $danger_fill;
    color: $danger;
    border: 1px solid $danger_dim;
    border-radius: 4px;
    padding: 3px 8px;
    font-weight: bold;
    font-size: 10px;
}
QLabel#badgeArmed {
    background-color: $ok_fill;
    color: $ok;
    border: 1px solid $ok_dim;
    border-radius: 4px;
    padding: 3px 8px;
    font-weight: bold;
    font-size: 10px;
}
QLabel#badgeDisarmed {
    background-color: $danger_fill;
    color: $danger;
    border: 1px solid $danger_dim;
    border-radius: 4px;
    padding: 3px 8px;
    font-weight: bold;
    font-size: 10px;
}
"""

DARK_STYLESHEET = Template(_QSS).substitute(PALETTE)
