"""
================================================================================
MODULE: motor_widget.py
PURPOSE: Actuator workspace - frame geometry, live PWM gauges, bench motor test
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Flight Deck) & Radxa Monitor
  * Upstream:      Decoded SERVO_OUTPUT_RAW from the Pixhawk, plus arm/airborne
                   state from TelemetrySnapshot for the test interlocks
  * Downstream:    motor_test_requested / motor_test_stop_requested, which
                   drone_gcs.py forwards to MAVLinkWorker.test_actuator

WHY THE GEOMETRY DIAGRAM EXISTS:
  This workspace used to be four abstract bars labelled "M1 FR CCW". That label
  is the whole mapping, written as text, and reading it is a translation step
  performed under time pressure: front-right is a physical arm on a physical
  aircraft, and a column of bars in channel order looks nothing like it. When
  one motor reads high, the question is always "which arm is that" - the
  diagram answers it by putting each rotor where it actually is.

WHY THE TEST PANEL EXISTS:
  Verifying motor order and direction of rotation is the standard bench check
  after any ESC or wiring work, and it was not possible from this station at
  all - the station could display PWM but not command a motor, so the check was
  done from a separate CLI tool or not at all. A quad with two motors swapped
  flies exactly once.

THE INTERLOCKS ARE NOT DECORATION:
  Spinning a motor from a GUI is the most directly dangerous thing this station
  can do - more so than arming, because arming at least implies an intent to
  fly. Five independent conditions gate it, and all five are enforced here in
  addition to the clamp inside MAVLinkWorker.test_actuator:
    1. A live link. No connection, no controls.
    2. Vehicle disarmed. An armed vehicle is refused outright.
    3. Vehicle not airborne. Belt and braces with (2); landed_state is the
       authority, not altitude.
    4. An explicit "props removed" acknowledgement, which the operator must
       tick, and which expires by itself after 60 seconds of no testing. A
       safety toggle that stays on for the rest of the session is a safety
       toggle that is always on.
    5. A throttle ceiling well below anything that produces useful lift.
  Every command also expires on the vehicle side (see test_actuator), so a
  dropped Wi-Fi link stops the motor without the GCS having to do anything.

USAGE:
  w = MotorWidget()
  w.update_pwms([1450, 1460, 1455, 1470])
  w.set_vehicle_state(connected=True, armed=False, airborne=False)
  w.motor_test_requested.connect(worker.test_actuator)
================================================================================
"""

from __future__ import annotations
import math
from typing import List, Optional

from PyQt5.QtCore import Qt, QRectF, QPointF, QTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPolygonF
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QSizePolicy, QPushButton,
    QCheckBox, QSlider,
)

from ui.scaling import px, scaled_font

# PX4 Quad X channel -> (label, unit-square position, rotation sense).
# Position is (right+, forward+) in body axes, so the diagram is drawn from the
# same numbers the mapping is stated in rather than from hand-placed pixels.
QUAD_X_LAYOUT = [
    (1, "FR", "CCW", (+1.0, +1.0)),
    (2, "RL", "CCW", (-1.0, -1.0)),
    (3, "FL", "CW",  (-1.0, +1.0)),
    (4, "RR", "CW",  (+1.0, -1.0)),
]

# Shared with the diagram, the bars and the diagnostics grid, so one PWM value
# never reads as "nominal" in one widget and "high" in another.
PWM_IDLE_MAX = 1050
PWM_NOMINAL_MAX = 1750
PWM_HIGH_MAX = 1900

COL_IDLE = QColor(139, 148, 158)
COL_NOMINAL = QColor(46, 160, 67)
COL_HIGH = QColor(210, 153, 34)
COL_SATURATED = QColor(248, 81, 73)


def pwm_colour(pwm: int) -> QColor:
    if pwm <= PWM_IDLE_MAX:
        return COL_IDLE
    if pwm <= PWM_NOMINAL_MAX:
        return COL_NOMINAL
    if pwm <= PWM_HIGH_MAX:
        return COL_HIGH
    return COL_SATURATED


class FrameGeometryWidget(QWidget):
    """Top-down Quad X diagram: rotors where they physically are, live-tinted.

    THE LAYOUT IS SOLVED, NOT GUESSED:
      The first version sized the arm as `min(w, h) / 2 - margin` and then drew
      a rotor disc and a two-line caption outside that radius. Every one of
      those extras was unbudgeted, so the discs were clipped by the widget edge
      at every aspect ratio and the front rotors' captions were painted across
      their own discs. `_layout()` now solves the arm length against all four
      constraints at once - disc, caption block, nose marker and margin, on both
      axes - and the painter and the hit test both read the result, so they
      cannot disagree. `layout_fits()` lets a test assert the whole drawing
      stays inside the widget.

    It degrades rather than overlaps: below the width where captions fit, they
    are dropped and the rotor numbers stay, which is still the information the
    diagram exists to convey.
    """

    motor_clicked = pyqtSignal(int)

    # Disc radius as a fraction of the arm. Well under 0.707 (the point at
    # which adjacent discs would touch), and large enough to hold "M1".
    ROTOR_FRACTION = 0.30
    MIN_ARM_PX = 26
    MIN_ROTOR_PX = 11

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(px(210), px(190))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setCursor(Qt.PointingHandCursor)
        self.pwms: List[int] = [1000, 1000, 1000, 1000]
        self.selected: int = 1

    def set_pwms(self, pwms: List[int]) -> None:
        self.pwms = list(pwms[:4]) + [1000] * max(0, 4 - len(pwms))
        self.update()

    def set_selected(self, motor_num: int) -> None:
        self.selected = int(motor_num)
        self.update()

    # ── geometry ────────────────────────────────────────────────────

    def _layout(self) -> dict:
        """Solve the diagram's geometry for the current size.

        Vertical budget, measured from the centre outwards:
            arm + rotor_r + caption block      (rear rotors and their captions)
            arm + nose reserve                 (the nose marker above the front)
        Horizontal budget:
            arm + rotor_r                      (the discs themselves)
            arm + caption width / 2            (captions are centred on the disc)

        The arm is the largest value satisfying all four, floored so the diagram
        stays recognisable in a small pane rather than collapsing to a dot.
        """
        w, h = max(1, self.width()), max(1, self.height())
        margin = px(4)
        gap = px(3)

        # Captions shrink with the pane and are dropped entirely when even the
        # shrunken form would squeeze the arm below its floor.
        cap_font_pt = 7 if w >= px(250) else 6
        caption_h = px(24)
        caption_w = min(px(96), w * 0.46)
        nose_reserve = px(28)

        k = self.ROTOR_FRACTION
        limits = [
            (h / 2.0 - margin - caption_h) / (1.0 + k),   # rear disc + caption
            h / 2.0 - margin - nose_reserve,              # nose marker
            (w / 2.0 - margin) / (1.0 + k),               # disc width
            w / 2.0 - margin - caption_w / 2.0,           # caption width
        ]
        arm = min(limits)
        show_captions = arm >= px(self.MIN_ARM_PX)

        if not show_captions:
            # Re-solve without the caption constraints rather than overlapping.
            arm = min((h / 2.0 - margin) / (1.0 + k),
                      h / 2.0 - margin - nose_reserve,
                      (w / 2.0 - margin) / (1.0 + k))

        arm = max(float(px(self.MIN_ARM_PX)), arm)
        rotor_r = max(float(px(self.MIN_ROTOR_PX)), arm * k)

        return {
            "cx": w / 2.0,
            "cy": h / 2.0,
            "arm": arm,
            "rotor_r": rotor_r,
            "gap": gap,
            "caption_h": caption_h,
            "caption_w": caption_w,
            "nose_reserve": nose_reserve,
            "show_captions": show_captions,
            "cap_font_pt": cap_font_pt,
            # Numerals scale with the disc so "M1" always fits inside it.
            "num_font_pt": max(6, min(13, int(rotor_r * 0.42))),
        }

    def rotor_centres(self) -> dict:
        """Motor number -> (x, y) in widget coordinates."""
        g = self._layout()
        out = {}
        for num, _label, _sense, (rx, fy) in QUAD_X_LAYOUT:
            out[num] = (g["cx"] + rx * g["arm"], g["cy"] - fy * g["arm"])
        return out

    def layout_fits(self) -> bool:
        """True when every drawn element is inside the widget rect.

        Used by the tests. Cheaper and far more reliable than eyeballing a
        screenshot at one size and assuming the other sizes are fine.
        """
        g = self._layout()
        w, h = self.width(), self.height()
        for (mx, my) in self.rotor_centres().values():
            if mx - g["rotor_r"] < 0 or mx + g["rotor_r"] > w:
                return False
            if my - g["rotor_r"] < 0 or my + g["rotor_r"] > h:
                return False
            if g["show_captions"]:
                if mx - g["caption_w"] / 2.0 < 0 or mx + g["caption_w"] / 2.0 > w:
                    return False
                top = my - g["rotor_r"] - g["gap"] - g["caption_h"]
                bottom = my + g["rotor_r"] + g["gap"] + g["caption_h"]
                if top < 0 or bottom > h:
                    return False
        if g["cy"] - g["arm"] - g["nose_reserve"] < 0:
            return False
        return True

    # ── interaction ─────────────────────────────────────────────────

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        radius = self._layout()["rotor_r"]
        for num, (mx, my) in self.rotor_centres().items():
            if math.hypot(event.pos().x() - mx, event.pos().y() - my) <= radius:
                self.set_selected(num)
                self.motor_clicked.emit(num)
                return

    # ── painting ────────────────────────────────────────────────────

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)

        g = self._layout()
        cx, cy, arm, rotor_r = g["cx"], g["cy"], g["arm"], g["rotor_r"]

        # Nose marker. Without an explicit "forward" the diagram is ambiguous by
        # 180 degrees, which is exactly the error it exists to prevent.
        nose_tip = cy - arm - g["nose_reserve"] + px(2)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(88, 166, 255)))
        p.drawPolygon(QPolygonF([
            QPointF(cx, nose_tip + px(10)),
            QPointF(cx - px(6), nose_tip),
            QPointF(cx + px(6), nose_tip),
        ]))
        p.setFont(scaled_font("Segoe UI", 7, bold=True))
        p.setPen(QColor(88, 166, 255))
        p.drawText(QRectF(cx - px(34), nose_tip + px(10), px(68), px(13)),
                   Qt.AlignHCenter | Qt.AlignTop, "NOSE")

        # Arms
        p.setPen(QPen(QColor(72, 79, 88), max(2, px(4)), Qt.SolidLine, Qt.RoundCap))
        for _num, _label, _sense, (rx, fy) in QUAD_X_LAYOUT:
            p.drawLine(QPointF(cx, cy), QPointF(cx + rx * arm, cy - fy * arm))

        # Body hub
        p.setPen(QPen(QColor(48, 54, 61), max(1, px(1))))
        p.setBrush(QBrush(QColor(33, 38, 45)))
        hub = arm * 0.20
        p.drawRoundedRect(QRectF(cx - hub, cy - hub, hub * 2, hub * 2),
                          px(4), px(4))

        centres = self.rotor_centres()
        for num, label, sense, (_rx, fy) in QUAD_X_LAYOUT:
            mx, my = centres[num]
            pwm = self.pwms[num - 1] if num - 1 < len(self.pwms) else 1000
            colour = pwm_colour(pwm)
            centre = QPointF(mx, my)

            # Disc, filled in proportion to throttle so the diagram carries the
            # same information as the bars without being read as a gauge.
            frac = max(0.0, min(1.0, (pwm - 1000) / 1000.0))
            p.setPen(Qt.NoPen)
            faint = QColor(colour)
            faint.setAlpha(45)
            p.setBrush(QBrush(faint))
            p.drawEllipse(centre, rotor_r, rotor_r)
            if frac > 0.005:
                strong = QColor(colour)
                strong.setAlpha(150)
                inner = rotor_r * (0.35 + 0.65 * frac)
                p.setBrush(QBrush(strong))
                p.drawEllipse(centre, inner, inner)

            # Selection ring
            p.setBrush(Qt.NoBrush)
            if num == self.selected:
                p.setPen(QPen(QColor(88, 166, 255), max(2, px(2))))
            else:
                p.setPen(QPen(colour.darker(120), max(1, px(1))))
            p.drawEllipse(centre, rotor_r, rotor_r)

            # Rotation-sense arc with an arrowhead. CW and CCW differ only by
            # the sweep direction, so the head is what actually distinguishes
            # them - an unheaded arc reads identically either way round.
            p.setPen(QPen(colour, max(1, px(2))))
            arc_r = rotor_r * 0.64
            box = QRectF(mx - arc_r, my - arc_r, arc_r * 2, arc_r * 2)
            start_deg, span_deg = (40, -250) if sense == "CW" else (140, 250)
            p.drawArc(box, int(start_deg * 16), int(span_deg * 16))
            end_rad = math.radians(start_deg + span_deg)
            ax = mx + arc_r * math.cos(end_rad)
            ay = my - arc_r * math.sin(end_rad)
            tangent = end_rad + (math.pi / 2 if sense == "CCW" else -math.pi / 2)
            head = max(px(3), rotor_r * 0.24)
            p.setBrush(QBrush(colour))
            p.setPen(Qt.NoPen)
            p.drawPolygon(QPolygonF([
                QPointF(ax + head * math.cos(tangent), ay - head * math.sin(tangent)),
                QPointF(ax + head * 0.55 * math.cos(tangent + 2.4),
                        ay - head * 0.55 * math.sin(tangent + 2.4)),
                QPointF(ax + head * 0.55 * math.cos(tangent - 2.4),
                        ay - head * 0.55 * math.sin(tangent - 2.4)),
            ]))

            # Channel number, centred in the disc.
            p.setFont(scaled_font("Segoe UI", g["num_font_pt"], bold=True))
            p.setPen(QColor(240, 246, 252))
            p.drawText(QRectF(mx - rotor_r, my - rotor_r, rotor_r * 2, rotor_r * 2),
                       Qt.AlignCenter, f"M{num}")

            if not g["show_captions"]:
                continue

            # Caption block, placed wholly clear of the disc: above the front
            # rotors, below the rear ones, never across either.
            cw, ch = g["caption_w"], g["caption_h"]
            if fy > 0:      # front
                block_top = my - rotor_r - g["gap"] - ch
            else:           # rear
                block_top = my + rotor_r + g["gap"]
            line_h = ch / 2.0

            p.setFont(scaled_font("Segoe UI", g["cap_font_pt"], bold=True))
            p.setPen(QColor(201, 209, 217))
            p.drawText(QRectF(mx - cw / 2.0, block_top, cw, line_h),
                       Qt.AlignCenter, f"{label} \u00b7 {sense}")
            p.setFont(scaled_font("Consolas", g["cap_font_pt"], bold=True))
            p.setPen(colour)
            p.drawText(QRectF(mx - cw / 2.0, block_top + line_h, cw, line_h),
                       Qt.AlignCenter, f"{pwm} \u00b5s")

        p.end()


class MotorChannelBar(QWidget):
    """Individual vertical bar gauge for a single motor channel."""

    def __init__(self, channel_num: int, label: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.channel_num = channel_num
        self.label = label
        self.pwm: int = 1000
        self.setMinimumSize(px(66), px(160))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_pwm(self, pwm: int):
        self.pwm = max(900, min(2100, int(pwm)))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        bar_w = float(px(26))
        bar_x = (w - bar_w) / 2.0
        top_y = float(px(20))
        # Three readout lines live below the track (microseconds,
        # percentage, position), so the track has to stop above them.
        # It previously ended at h-38 while the first line started at
        # h-48, and the bar fill was painted straight through the
        # microsecond value.
        bot_y = h - px(52)
        bar_h = max(1.0, bot_y - top_y)

        # Value percentage (1000 to 2000 us)
        pct = max(0.0, min(1.0, (self.pwm - 1000) / 1000.0))
        fill_h = bar_h * pct

        # Background track
        p.fillRect(QRectF(bar_x, top_y, bar_w, bar_h), QColor(22, 27, 34))
        p.setPen(QPen(QColor(48, 54, 61), 1))
        p.drawRect(QRectF(bar_x, top_y, bar_w, bar_h))

        bar_color = pwm_colour(self.pwm)

        # Filled Bar
        if fill_h > 0:
            p.fillRect(QRectF(bar_x + 1, bot_y - fill_h, bar_w - 2, fill_h), bar_color)

        # Mid reference line (1500 us)
        p.setPen(QPen(QColor(88, 166, 255, 120), 1, Qt.DashLine))
        mid_y = top_y + (bar_h / 2.0)
        p.drawLine(QPointF(bar_x - px(4), mid_y), QPointF(bar_x + bar_w + px(4), mid_y))

        # Channel Header Label (Top)
        p.setFont(scaled_font("Segoe UI", 9, bold=True))
        p.setPen(QColor(201, 209, 217))
        p.drawText(QRectF(0, px(2), w, px(16)), Qt.AlignCenter, f"M{self.channel_num}")

        # PWM, percentage and position, one per line so none of them clip.
        p.setFont(scaled_font("Consolas", 9, bold=True))
        p.setPen(bar_color if self.pwm > PWM_IDLE_MAX else COL_IDLE)
        p.drawText(QRectF(0, h - px(48), w, px(15)), Qt.AlignCenter, f"{self.pwm} µs")

        p.setFont(scaled_font("Segoe UI", 8, bold=True))
        p.setPen(QColor(139, 148, 158))
        p.drawText(QRectF(0, h - px(33), w, px(14)), Qt.AlignCenter,
                   f"{int(pct * 100)}%")

        p.setFont(scaled_font("Segoe UI", 7))
        p.setPen(QColor(139, 148, 158))
        p.drawText(QRectF(0, h - px(18), w, px(14)), Qt.AlignCenter, self.label)

        p.end()


class MotorTestPanel(QFrame):
    """Bench actuator test controls, behind the interlocks described up top."""

    test_requested = pyqtSignal(int, float)   # motor number (1-based), throttle %
    stop_requested = pyqtSignal()
    selection_changed = pyqtSignal(int)

    MAX_THROTTLE_PCT = 25
    # The safety acknowledgement is a statement about the aircraft's physical
    # state right now ("the props are off"), and physical state changes while a
    # session stays open. Sixty seconds is long enough to run the whole
    # four-motor sequence and short enough that walking away re-locks it.
    ARM_TIMEOUT_MS = 60_000
    # Re-send interval while a test is held. Comfortably inside the 2 s vehicle-
    # side expiry, so the motor keeps turning while held and stops on its own
    # within two seconds of release, a crash, or a link drop.
    REPEAT_MS = 700
    SEQUENCE_STEP_MS = 1500

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setProperty("class", "cardFrame")
        self.setMinimumWidth(px(250))

        self._selected = 1
        self._connected = False
        self._armed = False
        self._airborne = False
        self._sequence_index = 0

        self._repeat = QTimer(self)
        self._repeat.setInterval(self.REPEAT_MS)
        self._repeat.timeout.connect(self._emit_current)

        self._sequence = QTimer(self)
        self._sequence.setInterval(self.SEQUENCE_STEP_MS)
        self._sequence.timeout.connect(self._advance_sequence)

        self._arm_expiry = QTimer(self)
        self._arm_expiry.setSingleShot(True)
        self._arm_expiry.setInterval(self.ARM_TIMEOUT_MS)
        self._arm_expiry.timeout.connect(self._expire_safety)

        root = QVBoxLayout(self)
        root.setContentsMargins(px(12), px(10), px(12), px(11))
        root.setSpacing(px(7))

        head = QHBoxLayout()
        title = QLabel("BENCH MOTOR TEST", self)
        title.setObjectName("cardHeading")
        head.addWidget(title)
        head.addStretch()
        self.lbl_state = QLabel("", self)
        self.lbl_state.setObjectName("fieldSubLabel")
        head.addWidget(self.lbl_state)
        root.addLayout(head)

        rule = QFrame(self)
        rule.setObjectName("hDivider")
        rule.setFixedHeight(1)
        root.addWidget(rule)

        self.chk_safety = QCheckBox("PROPS OFF — enable motor test", self)
        self.chk_safety.setToolTip(
            "Confirms the propellers are physically off the aircraft.\n"
            "Re-locks automatically after 60 seconds.")
        self.chk_safety.setStyleSheet(
            "QCheckBox { color: #d29922; font-weight: bold; }")
        self.chk_safety.toggled.connect(self._on_safety_toggled)
        root.addWidget(self.chk_safety)

        lbl_sel = QLabel("Motor", self)
        lbl_sel.setObjectName("fieldLabel")
        root.addWidget(lbl_sel)

        sel = QHBoxLayout()
        sel.setSpacing(px(6))
        self.motor_buttons = {}
        for num, label, sense, _pos in QUAD_X_LAYOUT:
            b = QPushButton(f"M{num}", self)
            b.setCheckable(True)
            b.setChecked(num == 1)
            b.setToolTip(f"Motor {num} — {label}, {sense}")
            # Minimum, not fixed: four fixed cells plus their label
            # overran the panel at large UI scales and the buttons
            # overlapped each other.
            b.setMinimumWidth(px(42))
            b.clicked.connect(lambda _c, n=num: self.set_selected(n))
            self.motor_buttons[num] = b
            sel.addWidget(b)
        sel.addStretch()
        root.addLayout(sel)

        thr = QHBoxLayout()
        thr.setSpacing(px(8))
        lbl_thr = QLabel("Throttle", self)
        lbl_thr.setObjectName("fieldLabel")
        thr.addWidget(lbl_thr)
        self.slider_throttle = QSlider(Qt.Horizontal, self)
        self.slider_throttle.setRange(1, self.MAX_THROTTLE_PCT)
        self.slider_throttle.setValue(8)
        self.slider_throttle.valueChanged.connect(self._on_throttle_changed)
        thr.addWidget(self.slider_throttle, 1)
        self.lbl_throttle = QLabel("8 %", self)
        self.lbl_throttle.setObjectName("valueMono")
        self.lbl_throttle.setFixedWidth(px(44))
        self.lbl_throttle.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        thr.addWidget(self.lbl_throttle)
        root.addLayout(thr)

        actions = QVBoxLayout()
        actions.setSpacing(px(6))

        self.btn_spin = QPushButton("HOLD TO SPIN", self)
        self.btn_spin.setObjectName("btnNav")
        self.btn_spin.setToolTip(
            "Spins the selected motor for as long as the button is held.\n"
            "The vehicle stops it by itself within 2 s of release.")
        # Press/release rather than click: a momentary control cannot leave a
        # motor running because the operator's attention moved elsewhere.
        self.btn_spin.pressed.connect(self._start_hold)
        self.btn_spin.released.connect(self._stop_hold)
        actions.addWidget(self.btn_spin)

        self.btn_sequence = QPushButton("SEQ 1→4", self)
        self.btn_sequence.setCheckable(True)
        self.btn_sequence.setToolTip(
            "Spin each motor in turn, M1 to M4, to verify motor order and "
            "direction of rotation against the frame diagram.")
        self.btn_sequence.toggled.connect(self._on_sequence_toggled)
        second_row = QHBoxLayout()
        second_row.setSpacing(px(6))
        second_row.addWidget(self.btn_sequence, 1)

        self.btn_stop = QPushButton("STOP ALL", self)
        self.btn_stop.setObjectName("btnKill")
        self.btn_stop.setToolTip("Command zero throttle on every channel (Esc)")
        self.btn_stop.clicked.connect(self.stop_all)
        second_row.addWidget(self.btn_stop, 1)
        actions.addLayout(second_row)
        root.addLayout(actions)

        self.lbl_reason = QLabel("", self)
        self.lbl_reason.setObjectName("fieldSubLabel")
        self.lbl_reason.setWordWrap(True)
        root.addWidget(self.lbl_reason)

        # Absorb the slack at the bottom. Without it the column layout spreads
        # the gap between every control, so the checkbox, the throttle and the
        # buttons drift apart as the window grows and stop reading as one
        # sequence of steps.
        root.addStretch(1)

        self._refresh_enabled()

    # ── vehicle state ───────────────────────────────────────────────

    def set_vehicle_state(self, connected: bool, armed: bool, airborne: bool) -> None:
        """Re-evaluate the interlocks. Any transition into a disallowed state
        stops an in-progress test immediately rather than waiting for the
        vehicle-side timeout."""
        was_allowed = self.testing_allowed()
        self._connected = bool(connected)
        self._armed = bool(armed)
        self._airborne = bool(airborne)
        if was_allowed and not self.testing_allowed():
            self.stop_all()
            self.chk_safety.setChecked(False)
        self._refresh_enabled()

    def testing_allowed(self) -> bool:
        return (self._connected and not self._armed and not self._airborne
                and self.chk_safety.isChecked())

    def _block_reason(self) -> str:
        if not self._connected:
            return "No link to the vehicle."
        if self._armed:
            return "Vehicle is ARMED. Disarm before testing actuators."
        if self._airborne:
            return "Vehicle is airborne."
        if not self.chk_safety.isChecked():
            return "Tick the propellers-removed acknowledgement to enable."
        return ""

    def _refresh_enabled(self) -> None:
        allowed = self.testing_allowed()
        for w in (self.btn_spin, self.btn_sequence, self.slider_throttle):
            w.setEnabled(allowed)
        for b in self.motor_buttons.values():
            b.setEnabled(allowed)
        # STOP ALL stays live whenever there is a link. It is the one control
        # that must work in exactly the states the others are disabled for.
        self.btn_stop.setEnabled(self._connected)
        self.chk_safety.setEnabled(
            self._connected and not self._armed and not self._airborne)

        reason = self._block_reason()
        self.lbl_reason.setText(reason)
        self.lbl_reason.setStyleSheet(
            "color: #f85149;" if reason and self._armed else "color: #8b949e;")
        self.lbl_state.setText("READY" if allowed else "LOCKED")
        self.lbl_state.setStyleSheet(
            "color: #3fb950; font-weight: bold;" if allowed
            else "color: #8b949e; font-weight: bold;")

    # ── selection & throttle ────────────────────────────────────────

    def set_selected(self, motor_num: int) -> None:
        self._selected = int(motor_num)
        for num, b in self.motor_buttons.items():
            b.setChecked(num == self._selected)
        self.selection_changed.emit(self._selected)

    def selected_motor(self) -> int:
        return self._selected

    def throttle_pct(self) -> float:
        return float(self.slider_throttle.value())

    def _on_throttle_changed(self, value: int) -> None:
        self.lbl_throttle.setText(f"{value} %")

    # ── safety acknowledgement ──────────────────────────────────────

    def _on_safety_toggled(self, checked: bool) -> None:
        if checked:
            self._arm_expiry.start()
        else:
            self._arm_expiry.stop()
            self.stop_all()
        self._refresh_enabled()

    def _expire_safety(self) -> None:
        self.stop_all()
        self.chk_safety.setChecked(False)
        self.lbl_reason.setText(
            "Safety acknowledgement expired after 60 s — re-confirm to test again.")

    # ── test dispatch ───────────────────────────────────────────────

    def _emit_current(self) -> None:
        if not self.testing_allowed():
            self.stop_all()
            return
        self._arm_expiry.start()      # active testing keeps the window open
        self.test_requested.emit(self._selected, self.throttle_pct())

    def _start_hold(self) -> None:
        if not self.testing_allowed():
            return
        self._emit_current()
        self._repeat.start()

    def _stop_hold(self) -> None:
        self._repeat.stop()
        self.stop_requested.emit()

    def _on_sequence_toggled(self, running: bool) -> None:
        if running and self.testing_allowed():
            self._sequence_index = 0
            self.btn_sequence.setText("STOP SEQUENCE")
            self._advance_sequence()
            self._sequence.start()
            self._repeat.start()
        else:
            self._sequence.stop()
            self._repeat.stop()
            self.btn_sequence.setText("SEQ 1→4")
            if self.btn_sequence.isChecked():
                self.btn_sequence.setChecked(False)
            self.stop_requested.emit()

    def _advance_sequence(self) -> None:
        if not self.testing_allowed():
            self.btn_sequence.setChecked(False)
            return
        if self._sequence_index >= len(QUAD_X_LAYOUT):
            self.btn_sequence.setChecked(False)
            return
        self.set_selected(QUAD_X_LAYOUT[self._sequence_index][0])
        self._sequence_index += 1
        self._emit_current()

    def stop_all(self) -> None:
        """Halt every test path. Safe to call at any time, from any state."""
        self._repeat.stop()
        self._sequence.stop()
        if self.btn_sequence.isChecked():
            self.btn_sequence.blockSignals(True)
            self.btn_sequence.setChecked(False)
            self.btn_sequence.blockSignals(False)
        self.btn_sequence.setText("SEQ 1→4")
        self.stop_requested.emit()


class MotorWidget(QWidget):
    """The actuator workspace: geometry diagram, PWM bars and the test panel."""

    motor_test_requested = pyqtSignal(int, float)
    motor_test_stop_requested = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(px(12), px(12), px(12), px(12))
        layout.setSpacing(px(10))

        title_box = QHBoxLayout()
        t = QLabel("ACTUATOR OUTPUTS & MOTOR TELEMETRY (SERVO_OUTPUT_RAW)")
        t.setStyleSheet(
            f"font-weight: bold; color: #58a6ff; font-size: {px(12)}px;"
            " letter-spacing: 0.8px;")
        title_box.addWidget(t)
        title_box.addStretch()

        self.lbl_status = QLabel("ALL MOTORS IDLE (1000 µs)")
        self.lbl_status.setStyleSheet(
            f"color: #8b949e; font-size: {px(11)}px; font-weight: bold;")
        title_box.addWidget(self.lbl_status)
        layout.addLayout(title_box)

        body = QHBoxLayout()
        body.setSpacing(px(10))

        # Left: the physical picture.
        geo_card = QFrame(self)
        geo_card.setProperty("class", "cardFrame")
        gl = QVBoxLayout(geo_card)
        gl.setContentsMargins(px(10), px(8), px(10), px(8))
        gl.setSpacing(px(4))
        geo_head = QLabel("FRAME GEOMETRY", self)
        geo_head.setObjectName("cardHeading")
        gl.addWidget(geo_head)
        self.geometry = FrameGeometryWidget(self)
        self.geometry.motor_clicked.connect(self._on_geometry_clicked)
        gl.addWidget(self.geometry, 1)
        geo_hint = QLabel("Click a rotor to select it for testing", self)
        geo_hint.setObjectName("fieldSubLabel")
        geo_hint.setAlignment(Qt.AlignCenter)
        gl.addWidget(geo_hint)
        body.addWidget(geo_card, 3)

        # Middle: the numbers.
        card = QFrame(self)
        card.setProperty("class", "cardFrame")
        card_layout = QHBoxLayout(card)
        card_layout.setContentsMargins(px(16), px(12), px(16), px(12))
        card_layout.setSpacing(px(16))

        # Standard PX4 Quad X Motor Mapping - see QUAD_X_LAYOUT, which the
        # diagram is also drawn from so the two cannot disagree.
        self.bars = {}
        for num, label, sense, _pos in QUAD_X_LAYOUT:
            bar = MotorChannelBar(num, f"{label} {sense}", self)
            self.bars[num] = bar
            card_layout.addWidget(bar)
        # Names kept for existing callers and tests.
        self.bar_m1, self.bar_m2 = self.bars[1], self.bars[2]
        self.bar_m3, self.bar_m4 = self.bars[3], self.bars[4]
        # The gauges are four narrow bars and need the least width; the test
        # panel holds the widest controls and was the one being squeezed.
        body.addWidget(card, 2)

        # Right: the only controls in this workspace that command anything.
        self.test_panel = MotorTestPanel(self)
        self.test_panel.test_requested.connect(self.motor_test_requested)
        self.test_panel.stop_requested.connect(self.motor_test_stop_requested)
        self.test_panel.selection_changed.connect(self.geometry.set_selected)
        body.addWidget(self.test_panel, 4)

        layout.addLayout(body, 1)

        note = QLabel(
            "Nominal idle: 1000–1100 µs | Hover range: 1350–1550 µs | "
            "Saturation alert: >1920 µs")
        note.setStyleSheet(
            f"color: #8b949e; font-size: {px(10)}px; font-style: italic;")
        layout.addWidget(note)

    def _on_geometry_clicked(self, motor_num: int) -> None:
        self.test_panel.set_selected(motor_num)

    def set_vehicle_state(self, connected: bool, armed: bool, airborne: bool) -> None:
        self.test_panel.set_vehicle_state(connected, armed, airborne)

    def stop_motor_tests(self) -> None:
        self.test_panel.stop_all()

    def update_pwms(self, pwms: List[int]):
        """Update the bars and the geometry diagram with new PWM values."""
        if len(pwms) < 4:
            return
        for num in (1, 2, 3, 4):
            self.bars[num].set_pwm(pwms[num - 1])
        self.geometry.set_pwms(pwms)

        max_pwm = max(pwms[:4])
        if max_pwm > 1920:
            self.lbl_status.setText(f"WARNING: HIGH LOAD SATURATION ({max_pwm} µs)")
            self.lbl_status.setStyleSheet("color: #f85149; font-weight: bold;")
        elif max_pwm > 1150:
            self.lbl_status.setText(f"MOTORS ACTIVE ({max_pwm} µs)")
            self.lbl_status.setStyleSheet("color: #58a6ff; font-weight: bold;")
        else:
            self.lbl_status.setText("ALL MOTORS IDLE (1000 µs)")
            self.lbl_status.setStyleSheet("color: #8b949e; font-weight: bold;")
