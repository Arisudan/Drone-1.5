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
  * Export:        `PALETTE` (design tokens), `DARK_STYLESHEET` (QSS at 1.0)
                   and `build_stylesheet(scale)` (QSS at the active UI scale).
  * Consumed By:   `app.setStyleSheet(build_stylesheet())`.

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

EVERY DIMENSION HERE IS A BASE VALUE, NOT A FINAL ONE:
  The px counts below are authored against a 96 DPI display. ui/scaling.py
  multiplies them by the resolved UI scale before the sheet is applied, so a
  4K panel or a desktop set to large text gets a station that is still
  readable. Write new rules in base pixels and let that happen; hardcoding a
  'big' value here breaks the one display it was not measured on.

USAGE:
  from ui.styles import build_stylesheet, PALETTE
  app.setStyleSheet(build_stylesheet())
================================================================================
"""

import os
from string import Template

_ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")

# ─── Design tokens ──────────────────────────────────────────────────
# Edit here to retheme the entire ground station.

PALETTE = {
    # v2 "glass over imagery" palette. Opaque equivalents only: several modules
    # feed these to QColor(), which has no rgba() string form.
    "bg_window":     "#0b0d0c",   # application canvas (behind the backdrop)
    "bg_panel":      "#161918",   # glass card tone
    "bg_input":      "#1e2220",   # buttons / inputs
    "bg_raised":     "#2a2f2c",   # hover
    "bg_sunken":     "#080a09",   # consoles

    "border":        "#262a28",
    "border_soft":   "#353a37",
    "border_hover":  "#4d534f",

    # Chrome accent is the brand green.
    "accent":        "#46d06a",
    "accent_bright": "#7fe496",
    "accent_dim":    "#2a9a48",
    "accent_fill":   "#12311c",

    # Semantic.
    "info":          "#ffb02e",   # editable / live numeric readouts are amber in this design
    "ok":            "#46d06a",
    "ok_dim":        "#2a9a48",
    "ok_bright":     "#6fe790",
    "ok_fill":       "#12311c",
    "nav":           "#e9ece9",   # aircraft-moving commands are neutral glass buttons
    "nav_dim":       "#353a37",
    "nav_fill":      "#1e2220",
    "warn":          "#ffb02e",
    "warn_dim":      "#b07a1f",
    "warn_fill":     "#2e2410",
    "danger":        "#ff5a4f",
    "danger_dim":    "#b8332b",
    "danger_fill":   "#35161a",

    # Type
    "text":          "#e9ece9",
    "text_bright":   "#ffffff",
    "text_dim":      "#a3aaa5",
    "text_muted":    "#6f7772",

    # Fonts: same chain as the reference mocks (Lato, then Ubuntu Sans), with the
    # bundled Ubuntu as the guaranteed tail.
    "font_ui":       "'Lato', 'Noto Sans', 'Ubuntu Sans', 'Ubuntu', 'Segoe UI', 'DejaVu Sans', sans-serif",
    "font_brand":    "'Lato', 'Noto Sans', 'Ubuntu Sans', 'Ubuntu', sans-serif",
    "font_heading":  "'Lato', 'Noto Sans', 'Ubuntu Sans', 'Ubuntu', sans-serif",
    "font_mono":     "'Ubuntu Mono', 'DejaVu Sans Mono', monospace",
    "font_gauge":    "'Ubuntu Mono', 'DejaVu Sans Mono', monospace",

    "arrow_down":      os.path.join(_ASSETS, "chevron_down.png"),
    "arrow_down_hot":  os.path.join(_ASSETS, "chevron_down_hover.png"),
    "arrow_down_off":  os.path.join(_ASSETS, "chevron_down_off.png"),
    "check_dark":      os.path.join(_ASSETS, "check_dark.png"),
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
    font-family: $font_brand;
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
    font-family: $font_heading;
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
    font-family: $font_heading;
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
    font-family: $font_heading;
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
    font-family: $font_heading;
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

/* Diagnostics: caption over value. The caption is a quiet small-caps label so
   the number is the loudest thing in the cell. */
QLabel#diagCaption {
    font-family: $font_heading;
    color: $text_muted;
    font-size: 9px;
    font-weight: 600;
    letter-spacing: 1.2px;
}
/* A system card's title (POWER, ATTITUDE, ...): quiet, so the numbers lead. */
QLabel#cardGroupTitle {
    font-family: $font_heading;
    color: $text_dim;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 1.6px;
}
/* A value cell inside a system card: no border of its own - the card is the
   container, nested boxes would only add noise. */
QFrame.cellFrame {
    background-color: transparent;
    border: none;
    padding: 0px;
}
/* The glance strip: the few readouts you must be able to read in one look. */
QFrame#glanceStrip {
    background-color: $bg_panel;
    border: 1px solid $border;
    border-radius: 8px;
}
QFrame#glanceCell { background-color: transparent; border: none; padding: 0px; }

/* Diagnostics tab: groups its tiles by category (Power, Position, Motors,
   ...) under one of these instead of one undifferentiated wall of cards. */
QLabel#diagSectionHeader {
    font-family: $font_heading;
    color: $text_muted;
    font-size: 10px;
    font-weight: bold;
    text-transform: uppercase;
    letter-spacing: 1.5px;
    border-bottom: 1px solid $border_soft;
    padding-bottom: 4px;
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
/* The Diagnostics "Layout" menu button: same family as QPushButton. */
QToolButton#layoutBtn {
    background-color: $bg_input;
    color: $text;
    border: 1px solid $border_soft;
    border-radius: 5px;
    padding: 4px 12px;
    font-weight: 600;
    font-size: 10px;
    min-height: 22px;
}
QToolButton#layoutBtn:hover { background-color: $bg_raised; border-color: $border_hover; color: $text_bright; }
QToolButton#layoutBtn::menu-indicator { image: none; width: 0px; }
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
/* Flight-state chip beside the SLAM header's mode box. Neutral unless something matters. */
QLabel#flightStateChip {
    background-color: transparent; color: $text_muted;
    border: 1px solid $border; border-radius: 4px; padding: 0 8px;
    font-weight: 600; letter-spacing: 0.6px;
}
QLabel#flightStateChip[state="armed"]   { color: $warn;   border: 1px solid $warn_dim; }
QLabel#flightStateChip[state="offline"] { color: $danger; border: 1px solid $danger_dim; }

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

/* Disabled dispatching controls. Without this rule a filled command button
   keeps its full saturated colour when setEnabled(False) - it looks live and
   clickable while silently ignoring every click. Harmless for a greyed-out
   TAKEOFF; not harmless for the bench motor-test controls, which are disabled
   precisely when spinning a motor would be unsafe. A disabled control has to
   read as disabled. */
QPushButton#btnArm:disabled,
QPushButton#btnDisarm:disabled,
QPushButton#btnNav:disabled,
QPushButton#btnCaution:disabled,
QPushButton#btnKill:disabled,
QPushButton#btnConnect:disabled {
    background-color: $bg_panel;
    color: $text_muted;
    border: 1px solid $border;
}

/* ─── Tactical SLAM map toolbar ────────────────────────────────────────
   Every control on that tab used to carry its own inline setStyleSheet with
   a hardcoded font-size. Inline sheets never pass through scale_qss, so the
   whole toolbar stayed at 10px while the rest of the station scaled - at 2x
   it was the one tab that could not be read. These roles put it back on the
   design system, which is also the only way a palette change reaches it. */

/* Neutral map tool: view controls, zoom, rotate. Outlined, never filled -
   none of these command the aircraft. */
QPushButton#mapTool {
    background-color: $bg_input;
    color: $text;
    border: 1px solid $border_soft;
    border-radius: 4px;
    padding: 4px 8px;
    min-height: 24px;
    font-size: 10px;
}
QPushButton#mapTool:hover:!disabled {
    background-color: $bg_raised;
    border-color: $border_hover;
    color: $text_bright;
}
QPushButton#mapTool:disabled {
    background-color: $bg_panel;
    color: $text_muted;
    border-color: $border;
}
/* Latched tool (measure, follow, inflation, keep-out). Blue like every other
   "on" state - amber is reserved for caution. */
QPushButton#mapTool:checked {
    background-color: $accent_fill;
    color: $accent_bright;
    border-color: $accent_dim;
    font-weight: bold;
}

/* Segmented control: view switcher and map layers. One row of buttons that
   reads as a single multi-state control rather than three separate ones. */
QPushButton#segItem {
    background-color: $bg_input;
    color: $text_dim;
    border: 1px solid $border_soft;
    border-radius: 4px;
    padding: 4px 9px;
    min-height: 24px;
    font-size: 10px;
    font-weight: bold;
}
QPushButton#segItem:hover:!disabled {
    background-color: $bg_raised;
    color: $text_bright;
}
QPushButton#segItem:checked {
    background-color: $accent_fill;
    color: $accent_bright;
    border-color: $accent_dim;
}
QPushButton#segItem:disabled {
    background-color: $bg_panel;
    color: $text_muted;
    border-color: $border;
}

/* Cluster caption above the toolbar groups. */
QLabel#rulerTotal {
    color: $text;
    font-size: 10px;
    font-weight: bold;
}

QLabel#mapCaption {
    color: $text_muted;
    font-size: 9px;
    font-weight: bold;
    letter-spacing: 1px;
}

/* Map source / RViz state pill. Four states, selected by a dynamic property
   so the widget never needs an inline sheet to change colour. */
QLabel#mapPill {
    border-radius: 4px;
    padding: 4px 10px;
    font-size: 10px;
    font-weight: bold;
    background-color: $bg_input;
    color: $text_dim;
    border: 1px solid $border_soft;
}
/* GCS command arbitration chip + CHECK ORIGIN acknowledgement (controllers/arbitration.py) */
QPushButton#arbChip, QPushButton#originBtn {
    border-radius: 9px; padding: 4px 10px; font-size: 11px; font-weight: 700;
    color: $text_muted; border: 1px solid $border; background: transparent;
}
QPushButton#arbChip[state="ok"], QPushButton#originBtn[state="ok"] { color: $ok; border: 1px solid $ok; }
QPushButton#arbChip[state="warn"], QPushButton#originBtn[state="warn"] { color: $warn; border: 1px solid $warn; }
QPushButton#arbChip[state="bad"], QPushButton#originBtn[state="bad"] { color: $danger; border: 1px solid $danger; }
QLabel#mapPill[state="ok"] {
    background-color: $ok_fill; color: $ok; border-color: $ok_dim;
}
QLabel#mapPill[state="warn"] {
    background-color: $warn_fill; color: $warn; border-color: $warn_dim;
}
QLabel#mapPill[state="bad"] {
    background-color: $danger_fill; color: $danger; border-color: $danger_dim;
}

/* Staged-goal chip. Hidden until there is a goal, so it is never an empty
   box taking up the most prominent row on the tab. */
QLabel#goalChip {
    border-radius: 4px;
    padding: 5px 10px;
    font-size: 11px;
    font-weight: bold;
    background-color: $bg_input;
    color: $text;
    border: 1px solid $border_soft;
}
QWidget#transparentRow { background: transparent; }
QFrame#mapStatusStrip { background-color: $bg_panel; border: 1px solid $border; border-radius: 4px; }
QLabel#statusPose { color: $text; font-size: 11px; font-weight: 600; }
QLabel#statusUncert { font-size: 11px; font-weight: 600; }
/* Map-quality strip: quiet labels, plain (not monospace) values. */
QLabel#qualCaption { color: $text_muted; font-size: 10px; }
QLabel#qualValue { color: $text; font-size: 11px; font-weight: 600; }
QLabel#goalChip[state="blocked"] {
    background-color: $danger_fill; color: $danger; border-color: $danger_dim;
}

/* Path actions. Filled, because these three move the aircraft. ABORT uses
   the dimmer red: the fully saturated one is reserved for EMERGENCY KILL,
   which cuts the motors rather than landing. */
QPushButton#btnGo {
    background-color: $ok_dim; color: #ffffff;
    border: 1px solid $ok; border-radius: 4px;
    padding: 6px 10px; min-height: 28px; font-weight: bold;
}
QPushButton#btnGo:hover:!disabled { background-color: $ok; }

QPushButton#btnHold {
    background-color: $warn_dim; color: #ffffff;
    border: 1px solid $warn; border-radius: 4px;
    padding: 6px 10px; min-height: 28px; font-weight: bold;
}
QPushButton#btnHold:hover:!disabled { background-color: $warn; }

QPushButton#btnAbort {
    background-color: $danger_dim; color: #ffffff;
    border: 1px solid $danger; border-radius: 4px;
    padding: 6px 10px; min-height: 28px; font-weight: bold;
}
QPushButton#btnAbort:hover:!disabled { background-color: $danger; }

QPushButton#btnGo:disabled,
QPushButton#btnHold:disabled,
QPushButton#btnAbort:disabled {
    background-color: $bg_panel; color: $text_muted; border-color: $border;
}

/* Destructive map action: plain grey until you reach for it, red on hover. */
QPushButton#mapDanger {
    background-color: $bg_input;
    color: $text;
    border: 1px solid $border_soft;
    border-radius: 4px;
    padding: 4px 8px;
    min-height: 24px;
    font-size: 10px;
    font-weight: bold;
}
QPushButton#mapDanger:hover:!disabled { background-color: $danger_dim; color: #ffffff; border-color: $danger; }
QPushButton#mapDanger:disabled {
    background-color: $bg_panel; color: $text_muted; border-color: $border;
}

/* Embedded RViz2 placeholder, shown before the 3D view is launched. */
QFrame#rvizPlaceholder {
    background-color: $bg_window;
    border: 1px dashed $border_soft;
    border-radius: 8px;
}
QLabel#rvizPlaceholderIcon  { color: $accent; letter-spacing: 2px; }
QLabel#rvizPlaceholderTitle { font-family: $font_heading; color: $text; font-size: 16px; font-weight: bold; }

/* A tab's own page title (the one-line heading above its cards). */
QLabel#pageTitle {
    font-family: $font_heading;
    color: $accent;
    font-size: 11px;
    font-weight: bold;
    letter-spacing: 0.5px;
}
QLabel#rvizPlaceholderBody  { color: $text_dim; font-size: 12px; }

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

/* ── Configuration tab ───────────────────────────────────────────── */
QLabel#cfgLabel { color: $text; font-size: 11px; }
QLabel#preflightSummary { color: $warn; font-size: 11px; font-weight: 700; }
QLabel#preflightSummary[ready="true"] { color: $text_bright; }
QLabel#cfgUnit  { color: $text_muted; font-size: 10px; }
QLabel#cfgDirty { color: $accent; font-size: 11px; font-weight: 700; }
QLabel#cfgRestart { color: $text_dim; font-size: 9px; font-weight: 700; }
QLabel#cfgError { color: $danger; font-size: 10px; }
QLineEdit[invalid="true"], QComboBox[invalid="true"] { border: 1px solid $danger_dim; }
/* Neutral on/off box. The global QCheckBox is amber because its first user was
   the PROPS OFF safety interlock; ordinary settings must not look like warnings. */
QCheckBox#cfgToggle { color: $text; font-size: 11px; font-weight: 400; spacing: 8px; }
QCheckBox#cfgToggle::indicator { border: 1px solid $border_hover; }
QCheckBox#cfgToggle::indicator:checked { background-color: $accent_dim; border-color: $accent; }
QCheckBox#cfgToggle::indicator:hover { border-color: $accent; }
QListWidget#cfgNav { background: transparent; border: none; outline: none; }
QListWidget#cfgNav::item { padding: 7px 10px; color: $text_dim; border-radius: 6px; }
QListWidget#cfgNav::item:selected { background: $accent_fill; color: $text_bright; }
QListWidget#cfgNav::item:hover:!selected { background: $bg_input; }

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

/* ════════════════════════════════════════════════════════════════════
   v2 — glass-over-imagery design. Everything below overrides the legacy
   rules above (same selectors, later position / higher specificity).
   ════════════════════════════════════════════════════════════════════ */
QWidget {
    background-color: transparent;
    color: $text;
    font-family: $font_ui;
    font-size: 12px;
    selection-background-color: rgba(70, 208, 106, 70);
    selection-color: #ffffff;
}
QMainWindow, QDialog, QMessageBox, QFileDialog { background-color: $bg_window; }
QWidget#solidWindow { background-color: $bg_window; }
QMenu {
    background-color: #151817; color: $text;
    border: 1px solid rgba(255,255,255,40); border-radius: 8px; padding: 4px;
}
QMenu::item { padding: 6px 18px; border-radius: 5px; }
QMenu::item:selected { background-color: rgba(70,208,106,40); color: #ffffff; }
QMenu::separator { height: 1px; background: rgba(255,255,255,22); margin: 4px 6px; }
QToolTip {
    background-color: #151817; color: $text;
    border: 1px solid rgba(255,255,255,41); border-radius: 7px;
    padding: 5px 10px; font-weight: 700; font-size: 11px;
}

/* ── Glass panel furniture ─────────────────────────────────────────── */
QLabel#glassTitle {
    font-size: 11px; font-weight: 700; letter-spacing: 1.2px; color: $text_dim;
}
QLabel#glassSub { font-size: 11px; font-weight: 500; color: $text_muted; }
QLabel#glassPip {
    background-color: $accent; border-radius: 3px;
}
QWidget#glassHeader, QWidget#glassBody { background: transparent; }

/* chips (pills) */
QLabel#chip {
    min-height: 22px; max-height: 22px; padding: 0px 10px; border-radius: 12px;
    font-size: 10px; font-weight: 700; letter-spacing: 0.6px;
    color: $text_dim; border: 1px solid rgba(255,255,255,22);
    background-color: rgba(255,255,255,10);
}
QLabel#chip[state="ok"]   { color: $ok;     border-color: rgba(70,208,106,100); }
QLabel#chip[state="bad"]  { color: $danger; border-color: rgba(255,90,79,100); }
QLabel#chip[state="warn"] { color: $warn;   border-color: rgba(255,176,46,100); }

/* metric cells */
QFrame#metricCell {
    background-color: rgba(8,10,9,158); border: 1px solid rgba(255,255,255,22);
    border-radius: 8px;
}
QLabel#metricCap { font-size: 10px; color: $text_muted; letter-spacing: 1px; }
QLabel#metricVal { font-size: 23px; font-weight: 700; }

/* ── Buttons: glass, uppercase-ish, green only when "on"/primary ───── */
QPushButton {
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,255,255,23), stop:1 rgba(255,255,255,10));
    color: $text;
    border: 1px solid rgba(255,255,255,41);
    border-radius: 6px;
    padding: 4px 12px;
    font-size: 11px; font-weight: 700; letter-spacing: 0.4px;
    min-height: 24px;
}
QPushButton:hover   { background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,255,255,40), stop:1 rgba(255,255,255,20)); border-color: rgba(255,255,255,70); color: #ffffff; }
QPushButton:pressed { background-color: rgba(0,0,0,80); }
QPushButton:checked, QPushButton[on="true"] {
    color: $ok; border: 1px solid $ok;
    background-color: rgba(70,208,106,26);
}
QPushButton:disabled {
    color: $text_muted; background-color: rgba(255,255,255,8);
    border: 1px solid rgba(255,255,255,22);
}

/* primary / "go" */
QPushButton#btnConnect, QPushButton#btnGo, QPushButton#btnCliSend, QPushButton[role="go"] {
    color: #06210f; border: 1px solid #6fe790;
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #58dc7c, stop:1 #34b857);
}
QPushButton#btnConnect:hover, QPushButton#btnGo:hover:!disabled, QPushButton#btnCliSend:hover, QPushButton[role="go"]:hover {
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #6ee891, stop:1 #3fc862); color: #04170a;
}
QPushButton#btnConnect:disabled, QPushButton#btnGo:disabled, QPushButton[role="go"]:disabled {
    color: $text_muted; background-color: rgba(255,255,255,8); border: 1px solid rgba(255,255,255,22);
}
/* connected state: the same button, now offering DISCONNECT */
QPushButton#btnDisconnect {
    color: #ffd9d6; border: 1px solid $danger;
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,90,79,82), stop:1 rgba(184,51,43,102));
}
QPushButton#btnDisconnect:hover { background-color: rgba(255,90,79,120); }

/* danger */
QPushButton#btnKill, QPushButton#btnAbort, QPushButton[role="danger"] {
    color: #ffd9d6; border: 1px solid $danger; letter-spacing: 1.4px;
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,90,79,82), stop:1 rgba(184,51,43,102));
}
QPushButton#btnKill:hover, QPushButton#btnAbort:hover:!disabled, QPushButton[role="danger"]:hover {
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,90,79,120), stop:1 rgba(184,51,43,140));
    color: #ffffff;
}
QPushButton#btnKill:disabled, QPushButton#btnAbort:disabled {
    color: $text_muted; background-color: rgba(255,255,255,8); border: 1px solid rgba(255,255,255,22);
}
/* caution */
QPushButton#btnHold, QPushButton#mapDanger, QPushButton#btnCaution, QPushButton#btnForceArm, QPushButton[role="warn"] {
    color: $warn; border: 1px solid rgba(255,176,46,153); background-color: rgba(255,176,46,20);
}
QPushButton#btnHold:hover:!disabled, QPushButton#mapDanger:hover:!disabled, QPushButton#btnCaution:hover, QPushButton#btnForceArm:hover {
    background-color: rgba(255,176,46,50); color: #ffd28a; border-color: $warn;
}
QPushButton#btnHold:disabled, QPushButton#mapDanger:disabled, QPushButton#btnCaution:disabled {
    color: $text_muted; background-color: rgba(255,255,255,8); border: 1px solid rgba(255,255,255,22);
}
/* former coloured command buttons are plain glass now */
QPushButton#btnArm, QPushButton#btnDisarm, QPushButton#btnNav, QPushButton#btnOffboard, QPushButton#mapTool, QPushButton#segItem {
    color: $text;
    border: 1px solid rgba(255,255,255,41);
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,255,255,23), stop:1 rgba(255,255,255,10));
    font-size: 11px; padding: 4px 12px; border-radius: 6px;
}
QPushButton#btnArm:hover, QPushButton#btnDisarm:hover, QPushButton#btnNav:hover, QPushButton#btnOffboard:hover,
QPushButton#mapTool:hover:!disabled, QPushButton#segItem:hover:!disabled {
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,255,255,40), stop:1 rgba(255,255,255,20));
    color: #ffffff; border-color: rgba(255,255,255,70);
}
QPushButton#btnArm:checked, QPushButton#btnDisarm:checked, QPushButton#btnNav:checked, QPushButton#mapTool:checked, QPushButton#segItem:checked,
QPushButton#btnArm[on="true"], QPushButton#btnDisarm[on="true"], QPushButton#btnNav[on="true"], QPushButton#btnOffboard[on="true"] {
    color: $ok; border: 1px solid $ok; background-color: rgba(70,208,106,26); font-weight: 700;
}
QPushButton#btnArm:disabled, QPushButton#btnDisarm:disabled, QPushButton#btnNav:disabled,
QPushButton#mapTool:disabled, QPushButton#segItem:disabled {
    color: $text_muted; background-color: rgba(255,255,255,8); border: 1px solid rgba(255,255,255,22);
}
/* "off"/unavailable look (vision engines, unsupported actions) */
QPushButton[role="off"], QPushButton#btnVision {
    color: $text_muted; background-color: rgba(255,255,255,8);
    border: 1px solid rgba(255,255,255,22); font-size: 10px; padding: 4px 12px; min-height: 22px;
}
QPushButton#btnVision:checked { color: $ok; border-color: $ok; background-color: rgba(70,208,106,26); }
QPushButton#btnVision[visionRole]:checked { color: $ok; border-color: $ok; }

/* ── Inputs ────────────────────────────────────────────────────────── */
QLineEdit, QComboBox, QDateEdit, QSpinBox, QDoubleSpinBox {
    background-color: rgba(0,0,0,90); color: $text;
    border: 1px solid rgba(255,255,255,22); border-radius: 6px;
    padding: 3px 9px; font-size: 12px; font-weight: 500; min-height: 22px;
}
QLineEdit:hover, QComboBox:hover { border-color: rgba(255,255,255,56); }
QLineEdit:focus, QComboBox:focus { border-color: $ok; color: #ffffff; }
QLineEdit:disabled, QComboBox:disabled { color: $text_muted; }
QComboBox::drop-down { border: none; width: 20px; }
QComboBox::down-arrow { image: url($arrow_down); width: 10px; height: 6px; margin-right: 6px; }
QComboBox QAbstractItemView {
    background-color: #151817; color: $text; border: 1px solid rgba(255,255,255,41);
    selection-background-color: rgba(70,208,106,50); selection-color: #ffffff;
    border-radius: 6px; padding: 3px; outline: none;
}
QLineEdit[invalid="true"], QComboBox[invalid="true"] { border: 1px solid $danger; }
QLineEdit#cliInput {
    background-color: rgba(0,0,0,110); color: $text; font-family: $font_mono;
    font-size: 13px; font-weight: 500; border: 1px solid rgba(255,255,255,22);
    border-radius: 6px; padding: 5px 10px; min-height: 22px;
}
QLineEdit#cliInput:focus { border-color: $ok; }
QLabel#cliPrompt { color: $ok; font-family: $font_mono; font-size: 13px; font-weight: 700; }
QPushButton#btnCliClear { font-size: 11px; min-height: 24px; padding: 4px 14px; }
QPushButton#btnCliSend  { font-size: 11px; min-height: 24px; padding: 4px 16px; }

/* ── Checkboxes: the design's rounded square with a green fill ─────── */
QCheckBox { color: $text; font-size: 12px; font-weight: 400; spacing: 8px; }
QCheckBox::indicator {
    width: 17px; height: 17px; border-radius: 5px;
    border: 1.5px solid rgba(255,255,255,41); background-color: transparent;
}
QCheckBox::indicator:hover { border-color: rgba(255,255,255,90); }
QCheckBox::indicator:checked { background-color: $ok; border-color: $ok; image: url($check_dark); }
QCheckBox#cfgToggle { color: $text; font-size: 12px; font-weight: 400; spacing: 8px; }
QCheckBox#cfgToggle::indicator { border: 1.5px solid rgba(255,255,255,41); }
QCheckBox#cfgToggle::indicator:checked { background-color: $ok; border-color: $ok; image: url($check_dark); }

/* ── Labels ────────────────────────────────────────────────────────── */
QLabel#fieldLabel    { color: $text_dim; font-size: 12px; font-weight: 400; }
QLabel#fieldSubLabel { color: $text_muted; font-size: 11px; }
QLabel#cfgLabel      { color: $text_dim; font-size: 12px; }
QLabel#cfgUnit       { color: $text_muted; font-size: 11px; }
QLabel#valueMono, QLabel#valueBright { color: $text; font-size: 12px; font-family: $font_mono; }
QLabel#sectionTitle, QLabel#cardHeading, QLabel#pageTitle, QLabel#cardGroupTitle, QLabel#diagSectionHeader {
    color: $text_dim; font-size: 11px; font-weight: 700; letter-spacing: 1.2px;
    border: none;
}
QLabel#diagCaption { color: $text_muted; font-size: 10px; font-weight: 400; letter-spacing: 1px; }
QLabel#diagValue   { color: $text; font-size: 13px; font-weight: 700; font-family: $font_mono; }
QLabel#mapCaption, QLabel#execCaption, QLabel#tileCaption, QLabel#execAxis, QLabel#qualCaption {
    color: $text_muted; font-size: 10px; font-weight: 400; letter-spacing: 1px;
}
QLabel#execReadout, QLabel#tileValue { color: $warn; font-size: 19px; font-weight: 700; font-family: $font_mono; }
QLabel#execMessage { color: $text_muted; font-size: 11px; font-weight: 400; letter-spacing: 0px; }
QLabel#execBadgeIdle, QLabel#execBadgeActive, QLabel#execBadgeOk, QLabel#execBadgeFail {
    border-radius: 11px; padding: 2px 10px; font-size: 10px; font-weight: 700; letter-spacing: 0.6px;
    background-color: rgba(255,255,255,10); color: $text_dim; border: 1px solid rgba(255,255,255,22);
}
QLabel#execBadgeActive { color: $warn; border-color: rgba(255,176,46,100); background-color: rgba(255,176,46,20); }
QLabel#execBadgeOk     { color: $ok;   border-color: rgba(70,208,106,100); background-color: rgba(70,208,106,20); }
QLabel#execBadgeFail   { color: $danger; border-color: rgba(255,90,79,100); background-color: rgba(255,90,79,20); }
QLabel#mapPill, QLabel#goalChip, QLabel#flightStateChip {
    border-radius: 11px; padding: 2px 10px; font-size: 10px; font-weight: 700; letter-spacing: 0.6px;
    background-color: rgba(255,255,255,10); color: $text_dim; border: 1px solid rgba(255,255,255,22);
}
QLabel#mapPill[state="ok"]   { color: $ok;     border-color: rgba(70,208,106,100); background-color: rgba(70,208,106,20); }
QLabel#mapPill[state="warn"] { color: $warn;   border-color: rgba(255,176,46,100); background-color: rgba(255,176,46,20); }
QLabel#mapPill[state="bad"], QLabel#goalChip[state="blocked"] { color: $danger; border-color: rgba(255,90,79,100); background-color: rgba(255,90,79,20); }
QLabel#badgeArmed, QLabel#badgeDisarmed, QLabel#badgeConnected, QLabel#badgeDisconnected {
    border-radius: 11px; padding: 2px 10px; font-size: 10px; font-weight: 700; letter-spacing: 0.6px;
}
QLabel#badgeArmed    { color: $ok;   background-color: rgba(70,208,106,20); border: 1px solid rgba(70,208,106,100); }
QLabel#badgeDisarmed { color: $warn; background-color: rgba(255,176,46,20); border: 1px solid rgba(255,176,46,100); }
QLabel#badgeConnected    { color: $ok;     background-color: rgba(70,208,106,20); border: 1px solid rgba(70,208,106,100); }
QLabel#badgeDisconnected { color: $warn;   background-color: rgba(255,176,46,20); border: 1px solid rgba(255,176,46,100); }
QPushButton#arbChip, QPushButton#originBtn {
    border-radius: 6px; padding: 4px 12px; font-size: 11px; font-weight: 700; letter-spacing: 0.4px;
    color: $text; border: 1px solid rgba(255,255,255,41);
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,255,255,23), stop:1 rgba(255,255,255,10));
}
QPushButton#arbChip[state="ok"], QPushButton#originBtn[state="ok"] { color: $ok; border: 1px solid $ok; }
QPushButton#arbChip[state="warn"], QPushButton#originBtn[state="warn"] { color: $warn; border: 1px solid $warn; }
QPushButton#arbChip[state="bad"], QPushButton#originBtn[state="bad"] { color: $danger; border: 1px solid $danger; }

/* ── Group boxes / legacy cards become flat, titled glass sections ─── */
QGroupBox {
    background-color: transparent; border: none; margin-top: 0px; padding-top: 18px;
    color: $text_dim; font-size: 11px; font-weight: 700; letter-spacing: 1.2px;
}
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0px; left: 0px; color: $text_dim; }
QGroupBox#groupFlat { padding-top: 16px; }
QGroupBox#groupFlat::title { subcontrol-position: top left; left: 0px; }
QFrame.cardFrame, QFrame#glanceStrip, QFrame#footerBar, QFrame#mapStatusStrip {
    background-color: rgba(22,25,24,189); border: 1px solid rgba(255,255,255,22); border-radius: 10px; padding: 0px;
}
QFrame.cardFrame:hover { border-color: rgba(255,255,255,41); }
QFrame#vDivider, QFrame#rowDivider { background-color: rgba(255,255,255,22); min-width: 1px; max-width: 1px; border: none; }
QFrame#hDivider { background-color: rgba(255,255,255,22); min-height: 1px; max-height: 1px; border: none; }

/* ── Tables ────────────────────────────────────────────────────────── */
QTableWidget, QTableView, QTreeWidget {
    background-color: transparent; alternate-background-color: transparent;
    gridline-color: rgba(255,255,255,12); border: none; border-radius: 0px;
    font-size: 12px; color: $text;
    selection-background-color: rgba(70,208,106,26); selection-color: #ffffff;
}
QTableWidget::item { padding: 8px 12px; border: none; border-bottom: 1px solid rgba(255,255,255,12); }
QTableWidget::item:selected { background-color: rgba(70,208,106,26); color: #ffffff; }
QHeaderView { background: transparent; }
QHeaderView::section {
    background-color: transparent; color: $text_muted; border: none;
    border-bottom: 1px solid rgba(255,255,255,22); padding: 8px 12px;
    font-weight: 700; font-size: 10px; letter-spacing: 1px;
}
QTableCornerButton::section { background: transparent; border: none; }
QListWidget, QListView { background: transparent; border: none; outline: none; color: $text; }

/* ── Console / text ────────────────────────────────────────────────── */
QTextEdit, QPlainTextEdit, QTextBrowser {
    background-color: transparent; color: $text_dim; font-family: $font_mono;
    font-size: 13px; border: none; border-radius: 0px; padding: 2px;
}

/* ── Progress ──────────────────────────────────────────────────────── */
QProgressBar {
    background-color: rgba(0,0,0,115); border: 1px solid rgba(255,255,255,22);
    border-radius: 5px; text-align: center; color: $text_dim; font-size: 10px; min-height: 8px;
}
QProgressBar::chunk {
    border-radius: 4px;
    background-color: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 $accent_dim, stop:1 $accent);
}

/* ── Scrollbars ────────────────────────────────────────────────────── */
QScrollArea, QScrollArea > QWidget > QWidget { background: transparent; border: none; }
QScrollBar:vertical { border: none; background: transparent; width: 8px; margin: 0px; }
QScrollBar::handle:vertical { background: rgba(255,255,255,46); min-height: 24px; border-radius: 4px; }
QScrollBar::handle:vertical:hover { background: rgba(255,255,255,90); }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
QScrollBar:horizontal { border: none; background: transparent; height: 8px; margin: 0px; }
QScrollBar::handle:horizontal { background: rgba(255,255,255,46); min-width: 24px; border-radius: 4px; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0px; }

/* ── Tabs ──────────────────────────────────────────────────────────── */
QTabWidget::pane { border: 1px solid rgba(255,255,255,22); background: transparent; border-radius: 8px; }
QTabBar::tab {
    background: transparent; color: $text_muted; border: none; padding: 6px 14px;
    margin-right: 2px; font-weight: 700; font-size: 11px; letter-spacing: 0.6px;
}
QTabBar::tab:selected { color: $ok; border-bottom: 2px solid $ok; }
QTabBar::tab:hover:!selected { color: $text; }
QListWidget#cfgNav::item { padding: 7px 10px; color: $text_dim; border-radius: 6px; }
QListWidget#cfgNav::item:selected { background: rgba(70,208,106,26); color: $ok; }
QListWidget#cfgNav::item:hover:!selected { background: rgba(255,255,255,12); }

/* ---- SHELL (owner: main agent) ---- */
QFrame#topBar { background: transparent; border: none; padding: 0px; }
QWidget#topBrand { background: transparent; }
QPushButton#burger {
    background: transparent; border: 1px solid rgba(255,255,255,22); border-radius: 8px;
    min-height: 0px; padding: 0px;
}
QPushButton#burger:hover { border-color: rgba(255,255,255,70); }
QLabel#appTitle { font-family: $font_brand; font-size: 17px; font-weight: 700; letter-spacing: 0.2px; color: $text; }
QLabel#appSubtitle { font-size: 11px; font-weight: 500; color: $text_muted; }
QFrame#connPill { background-color: rgba(0,0,0,64); border: 1px solid rgba(255,255,255,22); border-radius: 9px; }
QLabel#connCaption { font-size: 10px; color: $text_muted; letter-spacing: 0.9px; padding: 0px 2px 0px 6px; }
QFrame#connPill QLineEdit, QFrame#connPill QComboBox {
    background-color: rgba(255,255,255,18); min-height: 24px; max-height: 26px;
    border: 1px solid rgba(255,255,255,22); border-radius: 6px; padding: 0px 8px; font-size: 12px;
}
QFrame#connPill QPushButton { min-height: 24px; max-height: 26px; padding: 0px 14px; font-size: 11px; }
QPushButton#audioChip {
    min-height: 22px; max-height: 24px; padding: 0px 10px; border-radius: 12px;
    font-size: 10px; font-weight: 700; letter-spacing: 0.6px; color: $text_dim;
    border: 1px solid rgba(255,255,255,22); background-color: rgba(255,255,255,10);
}
QPushButton#audioChip[state="ok"]   { color: $ok;   border-color: $ok;   background-color: rgba(70,208,106,20); }
QPushButton#audioChip[state="warn"] { color: $warn; border-color: $warn; background-color: rgba(255,176,46,20); }
QPushButton#audioChip[state="off"]  { color: $text_muted; }
QLabel#topClock { font-size: 15px; font-weight: 700; color: $text; }
QLabel#routeTitle { font-size: 13px; font-weight: 700; color: $ok; }
QLabel#routeSub   { font-size: 11px; color: $text_dim; }
QLabel#vehName { font-size: 13px; font-weight: 700; color: $text; }
QLabel#vehSub  { font-size: 11px; color: $text_muted; }
QFrame#railSep { background-color: rgba(255,255,255,22); border: none; }
QLabel#railPreVal { font-size: 13px; font-weight: 700; color: $warn; }
QLabel#railPreVal[ready="true"] { color: $ok; }
QLabel#railPreCap { font-size: 8px; letter-spacing: 1.4px; color: $text_muted; }
QLabel#fieldReadout {
    background-color: rgba(0,0,0,90); border: 1px solid rgba(255,255,255,22); border-radius: 6px;
    padding: 3px 9px; font-family: $font_mono; font-size: 13px; color: $text; min-height: 22px;
}
QLineEdit[num="true"] { font-family: $font_mono; font-size: 13px; }
QLabel#footerNote { font-size: 11px; color: $text_muted; letter-spacing: 0px; }
QLabel#railCheckIcon, QLabel#railCheck { background: transparent; }

/* ==== config/params agent ==== */
/* ---- v2 config + params pages ---- */
QListWidget#cfgNav { background: transparent; border: none; outline: none; }
QListWidget#cfgNav::item {
    padding: 8px 12px; margin: 0px 0px 8px 0px; color: $text; font-size: 12px; font-weight: 700;
    border: 1px solid rgba(255,255,255,41); border-radius: 6px;
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,255,255,23), stop:1 rgba(255,255,255,10));
}
QListWidget#cfgNav::item:hover:!selected { background-color: rgba(255,255,255,30); color: #ffffff; }
QListWidget#cfgNav::item:selected {
    color: $ok; border: 1px solid $ok; background-color: rgba(70,208,106,26);
}
QListWidget#paramGroups { background: transparent; border: none; outline: none; }
QListWidget#paramGroups::item { padding: 7px 10px; color: $text_dim; border-radius: 6px; font-size: 12px; }
QListWidget#paramGroups::item:selected { background: rgba(70,208,106,26); color: $ok; }
QListWidget#paramGroups::item:hover:!selected { background: rgba(255,255,255,12); }
QPushButton#cfgReset { font-size: 10px; min-height: 20px; padding: 2px 10px; }
QLineEdit[num="true"] { font-family: $font_mono; font-size: 13px; }
QFrame#cfgCard { background: transparent; border: none; }
QTextBrowser#paramDetail {
    background: rgba(0,0,0,40); border: none; border-top: 1px solid rgba(255,255,255,22);
    border-radius: 0px; padding: 8px 14px; font-family: $font_ui; font-size: 12px; color: $text;
}
QHeaderView::section { text-align: left; }
QTableWidget::item:selected { border-left: 2px solid $ok; }
/* ==== end config/params agent ==== */


/* ---- FPV page (video_feed_widget / fpv_hud) ---- */
QLabel#fpvVideo { background: transparent; border: none; }
QLabel#fpvVideo[live="true"] { background-color: #000000; }
QLabel#fpvField {
    background-color: rgba(0,0,0,90); color: $text; border: 1px solid rgba(255,255,255,22);
    border-radius: 6px; padding: 3px 9px; font-size: 12px; min-height: 22px;
}
QLabel#fpvField[num="true"] { font-family: $font_mono; font-size: 13px; }
QLabel#fpvHealth { color: $text_dim; font-size: 11px; }
QLabel#fpvRec { color: $danger; font-size: 11px; font-weight: 700; }
QLabel#fpvNote { color: $text_muted; font-size: 11px; }

/* ---- diag/terminal/logs page block ---- */
QLabel#pfTitle { font-size: 16px; font-weight: 700; color: $text; }
QLabel#pfSub   { font-size: 11px; color: $text_muted; }
QLabel#pfName  { font-size: 13px; font-weight: 700; color: $text; }
QLabel#pfDetail { font-size: 12px; color: $text_dim; }
QPushButton#pfFix {
    min-height: 20px; padding: 2px 10px; font-size: 10px; letter-spacing: 0.6px;
    background-color: rgba(255,255,255,18); border: 1px solid rgba(255,255,255,41); border-radius: 5px;
}
QLabel#noteWarn { font-size: 11px; color: $warn; margin-top: 4px; }
QLabel#logStat { font-size: 13px; font-weight: 700; color: $text; }
QLabel#detailValue { font-size: 22px; font-weight: 700; color: $text; }
QLabel#detailValueAlt { font-size: 22px; font-weight: 700; color: $warn; }
QWidget#diagInner, QScrollArea#diagScroll { background: transparent; border: none; }
QWidget#cliConsole { background: transparent; }
QTableWidget::item:selected { border: none; border-bottom: 1px solid rgba(255,255,255,12); background-color: rgba(70,208,106,26); }
QTableWidget::item:selected:first { border-left: 2px solid $ok; }
QPlainTextEdit { selection-background-color: rgba(70,208,106,70); }

/* ---- [motors page] ------------------------------------------------- */
QLabel#motorField {
    background-color: rgba(0,0,0,90); border: 1px solid rgba(255,255,255,22);
    border-radius: 6px; padding: 3px 9px; min-height: 22px;
    color: $text; font-family: $font_mono; font-size: 13px;
}
QLabel#motorStatus { color: $text_muted; font-size: 11px; font-weight: 700; }
QLabel#motorScaleNote { color: $text_muted; font-size: 10px; }

/* ==== SLAM page (slam agent) ==== */
QFrame#segBar {
    background-color: rgba(0,0,0,90); border: 1px solid rgba(255,255,255,22); border-radius: 8px;
}
QFrame#segBar QPushButton#segItem {
    background-color: transparent; border: 1px solid transparent; border-radius: 6px;
    color: $text; font-size: 11px; font-weight: 700; padding: 2px 8px; min-height: 22px;
}
QFrame#segBar QPushButton#segItem:hover:!disabled { background-color: rgba(255,255,255,18); color: #ffffff; }
QFrame#segBar QPushButton#segItem:checked {
    background-color: rgba(70,208,106,41); color: $ok; border: 1px solid rgba(70,208,106,102);
}
QFrame#segBar QPushButton#segItem:disabled { color: $text_muted; background-color: transparent; border: 1px solid transparent; }
QLabel#slamNote { color: $text_muted; font-size: 11px; line-height: 150%; }
QLabel#slamFieldOff {
    background-color: rgba(0,0,0,60); border: 1px solid rgba(255,255,255,15); border-radius: 6px;
    padding: 3px 9px; color: $text_muted; font-size: 12px; min-height: 22px;
}
QWidget#slamRoute QPushButton#btnHold:!disabled {
    color: $text; border: 1px solid rgba(255,255,255,41);
    background-color: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 rgba(255,255,255,23), stop:1 rgba(255,255,255,10));
}
QWidget#slamRoute QPushButton#btnHold:hover:!disabled { color: #ffffff; border-color: rgba(255,255,255,70); }
QWidget#slamRoute QPushButton#btnHold:disabled { color: $text_muted; border: 1px solid rgba(255,255,255,22); background-color: rgba(255,255,255,8); }
QPushButton#feedIconBtn {
    background: transparent; border: none; color: $text_muted; font-size: 12px; font-weight: 700;
    padding: 0px; min-height: 0px; letter-spacing: 0px;
}
QPushButton#feedIconBtn:hover { color: $ok; background: transparent; border: none; }
QFrame#mapStatusStrip { background-color: transparent; border: none; }
QLabel#statusPose { color: $text; font-family: $font_mono; font-size: 12px; font-weight: 500; }
QLabel#statusUncert { color: $text_muted; font-size: 11px; }
QLabel#rulerTotal { color: $warn; font-family: $font_mono; font-size: 12px; font-weight: 700; }
QLabel#qualValue { color: $text; font-family: $font_mono; font-size: 11px; font-weight: 700; }
QPushButton#mapTool:disabled, QPushButton#mapDanger:disabled { color: $text_muted; }
QPushButton[cell="true"], QPushButton#mapTool[cell="true"], QPushButton#mapDanger[cell="true"], QPushButton#btnGo[cell="true"] { min-height: 20px; max-height: 20px; padding: 2px 8px; }
QPushButton[cell="lg"], QPushButton#mapTool[cell="lg"], QPushButton#btnGo[cell="lg"], QPushButton#btnHold[cell="lg"], QPushButton#btnAbort[cell="lg"] { min-height: 22px; max-height: 22px; padding: 4px 8px; }
QFrame#segBar QPushButton#segItem { min-height: 20px; max-height: 20px; padding: 2px 8px; }

/* ==== PAGE QSS (each page agent appends its own block above this line) ==== */
"""


def build_stylesheet(scale: float = None) -> str:
    """The full QSS with the palette substituted and every dimension scaled.

    `scale=None` means "whatever ui.scaling resolved at startup", which is what
    the application wants. An explicit value is for tests and for previewing a
    factor without touching the global one.

    Only dimensions move. The palette, the selectors and the hairline border
    widths are identical at every scale - see ui/scaling.scale_qss for exactly
    which properties are rewritten and why `border:` is not among them.
    """
    from ui.scaling import scale_qss
    from ui.fonts import load_bundled_fonts
    load_bundled_fonts()      # before the stylesheet names them
    return scale_qss(Template(_QSS).substitute(PALETTE), scale)


# Unscaled stylesheet at factor 1.0. Kept as a module constant because
# radxa_monitor.py and the import smoke tests consume it directly, and because
# it is the honest default for any caller that never initialised a scale.
DARK_STYLESHEET = Template(_QSS).substitute(PALETTE)
