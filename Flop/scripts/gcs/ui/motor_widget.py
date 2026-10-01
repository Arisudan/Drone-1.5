"""
================================================================================
MODULE: motor_widget.py
PURPOSE: Actuator workspace - drone diagram, live PWM gauges, bench motor test
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Flight Deck) & Radxa Monitor
  * Upstream:      Decoded SERVO_OUTPUT_RAW from the Pixhawk, the vehicle's own
                   PWM_MAIN_{MIN,MAX,DIS,FUNC}n parameters (core/motor_range.py),
                   and arm/airborne state for the test interlocks
  * Downstream:    motor_test_requested / motor_test_stop_requested, which
                   drone_gcs.py forwards to MAVLinkWorker.test_actuator

WHY THE DRONE DIAGRAM EXISTS:
  When one motor reads high the question is always "which arm is that". A column
  of bars in channel order does not answer it; a drawing of the aircraft with
  each rotor where it physically is, does. It is drawn top-down: tapered arms, a
  body with a camera pod and white front / red rear LEDs (so the heading is
  readable without a text label), motor hubs, and propellers that turn at a rate
  proportional to the commanded output.

THE NUMBERS ARE RELATIVE TO THE VEHICLE'S OWN RANGE:
  Percentages, colours and "idle / high / saturated" all come from
  core.motor_range - the vehicle's configured min..max per output, with the
  output -> motor mapping from PWM_MAIN_FUNCn. A 1100-1900 us vehicle at full
  throttle reads 100 %, not 80 %. Until the parameters have been read the
  documented defaults are used and the card says so.

THE DISPLAY NEVER PRESENTS THE PAST AS THE PRESENT:
  SERVO_OUTPUT_RAW arrives at 5 Hz. If it stops, the motors are shown greyed and
  labelled, not frozen at their last value. During a bench test the commanded
  output is shown immediately and marked COMMANDED until a live sample confirms
  it.

THE INTERLOCKS ARE NOT DECORATION:
  Spinning a motor from a GUI is the most directly dangerous thing this station
  can do. Five independent conditions gate it, all enforced here in addition to
  the clamp inside MAVLinkWorker.test_actuator:
    1. A live link.
    2. Vehicle disarmed.
    3. Vehicle not airborne (landed_state is the authority).
    4. An explicit "props removed" acknowledgement that expires by itself after
       60 seconds - the countdown is shown.
    5. A throttle ceiling well below anything that produces useful lift.
  Every command also expires on the vehicle side, so a dropped link stops the
  motor without the GCS having to do anything.
================================================================================
"""

from __future__ import annotations
import math
import time
from typing import List, Optional

from PyQt5.QtCore import Qt, QEvent, QRectF, QPointF, QTimer, pyqtSignal
from PyQt5.QtGui import (
    QPainter, QColor, QPen, QBrush, QPolygonF, QLinearGradient, QRadialGradient,
)
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QFrame, QSizePolicy,
    QPushButton, QCheckBox, QSlider,
)

from core import motor_range as mr
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

COL_IDLE = QColor(139, 148, 158)
COL_NOMINAL = QColor(46, 160, 67)
COL_HIGH = QColor(210, 153, 34)
COL_SATURATED = QColor(248, 81, 73)
COL_STALE = QColor(88, 96, 105)

_STATE_COLOUR = {
    mr.OFF: COL_IDLE, mr.IDLE: COL_IDLE, mr.NOMINAL: COL_NOMINAL,
    mr.HIGH: COL_HIGH, mr.SATURATED: COL_SATURATED,
}


def state_colour(state: str, stale: bool = False) -> QColor:
    return COL_STALE if stale else _STATE_COLOUR.get(state, COL_IDLE)


class FrameGeometryWidget(QWidget):
    """Top-down quadcopter: arms, body, camera pod, hubs, spinning propellers.

    THE LAYOUT IS SOLVED, NOT GUESSED:
      ``_layout()`` solves the arm length against every constraint at once - the
      rotor disc and its rotation ring, the caption block, and the margin, on
      both axes - and the painter and the hit test both read the result, so they
      cannot disagree. ``layout_fits()`` lets a test assert the whole drawing
      stays inside the widget. It degrades rather than overlaps: below the width
      where captions fit they are dropped and the rotor numbers stay.
    """

    motor_clicked = pyqtSignal(int)

    # Disc radius as a fraction of the arm. Well under 0.707 (the point at which
    # adjacent discs would touch), and large enough to hold "M1".
    ROTOR_FRACTION = 0.30
    # The rotation arrow rides just outside the disc.
    # The arrow rides well clear of the disc (and its selection ring at 1.04) so
    # it reads as a separate rotation cue, not as a smudge on the propeller.
    RING_FACTOR = 1.40
    ARROW_HALF_SPAN = 36.0       # degrees either side of the horizontal outer side
    ARROW_HEAD_FRACTION = 0.34   # head size as a fraction of the disc radius
    MIN_ARM_PX = 26
    # Narrowest caption that still holds "1850 µs · 94%"; below it captions drop.
    MIN_CAPTION_PX = 76
    MIN_ROTOR_PX = 11
    ANIM_MS = 33

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMinimumSize(px(210), px(190))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setCursor(Qt.PointingHandCursor)
        self.pwms: List[int] = [1000, 1000, 1000, 1000]     # motor order
        self.selected: int = 1
        self.range: mr.MotorRange = mr.SHARED
        self.stale: bool = False
        self._angles = [0.0, 0.0, 0.0, 0.0]                  # blade angle, deg
        self._anim = QTimer(self)
        self._anim.setInterval(self.ANIM_MS)
        self._anim.timeout.connect(lambda: self.advance(self.ANIM_MS / 1000.0))

    # ── data ────────────────────────────────────────────────────────

    def set_range(self, rng: mr.MotorRange) -> None:
        self.range = rng
        self.update()

    def set_pwms(self, pwms: List[int], stale: Optional[bool] = None) -> None:
        self.pwms = list(pwms[:4]) + [1000] * max(0, 4 - len(pwms))
        if stale is not None:
            self.stale = bool(stale)
        self._sync_animation()
        self.update()

    def set_stale(self, stale: bool) -> None:
        if stale != self.stale:
            self.stale = bool(stale)
            self._sync_animation()
            self.update()

    def set_selected(self, motor_num: int) -> None:
        self.selected = int(motor_num)
        self.update()

    # ── animation ───────────────────────────────────────────────────

    def spin_rate_dps(self, motor: int) -> float:
        """Blade speed in degrees/second for a motor, signed by rotation sense.

        Zero when the output is off or the data is stale: a propeller must never
        appear to spin on a number that is not live. A motor at its minimum
        (idle) turns slowly; full range is a fast blur.
        """
        if self.stale:
            return 0.0
        pwm = self.pwms[motor - 1]
        state = self.range.state(motor, pwm)
        if state == mr.OFF:
            return 0.0
        frac = self.range.fraction(motor, pwm)
        rate = 90.0 + 1100.0 * frac
        sense = next(s for n, _l, s, _p in QUAD_X_LAYOUT if n == motor)
        return rate if sense == "CW" else -rate

    def advance(self, dt: float) -> None:
        """Turn the blades by ``dt`` seconds of motion. Public so tests can
        step the animation without a running event loop."""
        for k in range(4):
            self._angles[k] = (self._angles[k] + self.spin_rate_dps(k + 1) * dt) % 360.0
        self.update()

    def _sync_animation(self) -> None:
        spinning = any(self.spin_rate_dps(k) != 0.0 for k in (1, 2, 3, 4))
        if spinning and self.isVisible():
            if not self._anim.isActive():
                self._anim.start()
        elif self._anim.isActive():
            self._anim.stop()

    def showEvent(self, ev):
        super().showEvent(ev)
        self._sync_animation()

    def hideEvent(self, ev):
        super().hideEvent(ev)
        self._anim.stop()

    # ── geometry ────────────────────────────────────────────────────

    def _layout(self) -> dict:
        """Solve the diagram's geometry for the current size.

        Vertical budget, from the centre outwards, is symmetric: a rotor ring
        plus its caption block above the front rotors and below the rear ones.
        Horizontal: the ring, and the caption (centred on the rotor).
        """
        w, h = max(1, self.width()), max(1, self.height())
        margin = px(4)
        gap = px(6)

        cap_font_pt = 7 if w >= px(250) else 6
        caption_h = px(24)
        caption_w = min(px(104), w * 0.46)

        k = self.ROTOR_FRACTION * self.RING_FACTOR
        # The arrow's head pokes a little beyond the ring on the horizontal side.
        k_w = k + self.ROTOR_FRACTION * self.ARROW_HEAD_FRACTION + 0.01
        limits = [
            (h / 2.0 - margin - caption_h - gap) / (1.0 + k),   # ring + caption
            (w / 2.0 - margin) / (1.0 + k_w),                   # ring + arrow head
            w / 2.0 - margin - caption_w / 2.0,                 # caption width
        ]
        arm = min(limits)
        # Left and right captions are centred on rotors 2*arm apart: they must be
        # narrower than that or they print across each other.
        caption_w = min(caption_w, 2.0 * arm - px(8))
        show_captions = arm >= px(self.MIN_ARM_PX) and caption_w >= px(self.MIN_CAPTION_PX)

        if not show_captions:
            # Re-solve without the caption constraints rather than overlapping.
            arm = min((h / 2.0 - margin) / (1.0 + k),
                      (w / 2.0 - margin) / (1.0 + k_w))

        arm = max(float(px(self.MIN_ARM_PX)), arm)
        rotor_r = max(float(px(self.MIN_ROTOR_PX)), arm * self.ROTOR_FRACTION)

        return {
            "cx": w / 2.0,
            "cy": h / 2.0,
            "arm": arm,
            "rotor_r": rotor_r,
            "ring_r": rotor_r * self.RING_FACTOR,
            "gap": gap,
            "caption_h": caption_h,
            "caption_w": caption_w,
            "show_captions": show_captions,
            "cap_font_pt": cap_font_pt,
            "num_font_pt": max(6, min(11, int(rotor_r * 0.36))),
        }

    def rotor_centres(self) -> dict:
        """Motor number -> (x, y) in widget coordinates."""
        g = self._layout()
        out = {}
        for num, _label, _sense, (rx, fy) in QUAD_X_LAYOUT:
            out[num] = (g["cx"] + rx * g["arm"], g["cy"] - fy * g["arm"])
        return out

    def caption_rect(self, num: int) -> QRectF:
        """Where rotor ``num``'s two-line caption is drawn: wholly above a front
        rotor and wholly below a rear one, centred on it, clear of its disc."""
        g = self._layout()
        mx, my = self.rotor_centres()[num]
        fy = next(pos[1] for n, _l, _s, pos in QUAD_X_LAYOUT if n == num)
        cw, ch = g["caption_w"], g["caption_h"]
        # Clear of the disc AND of the arrow's vertical reach: at large sizes the
        # arrow (sitting beside the rotor) rises higher than the disc does.
        arrow_reach = (g["ring_r"] * math.sin(math.radians(self.ARROW_HALF_SPAN))
                       + self._arrow_head(g["rotor_r"]) + px(2))
        off = max(g["rotor_r"] + g["gap"], arrow_reach)
        top = (my - off - ch) if fy > 0 else (my + off)
        return QRectF(mx - cw / 2.0, top, cw, ch)

    def _arrow_head(self, rotor_r: float) -> float:
        return max(float(px(4)), rotor_r * self.ARROW_HEAD_FRACTION)

    def arrow_rect(self, num: int) -> QRectF:
        """Bounding box of the rotation arrow: on the HORIZONTAL outer side of
        the rotor (left rotors point left, right rotors right), which is the one
        side neither the caption (above/below) nor the arm (diagonal, inward)
        occupies. The arc spans +-ARROW_HALF_SPAN degrees about that side."""
        g = self._layout()
        mx, my = self.rotor_centres()[num]
        ring = g["ring_r"]
        rx = next(pos[0] for n, _l, _s, pos in QUAD_X_LAYOUT if n == num)
        head = self._arrow_head(g["rotor_r"])
        half = ring * math.sin(math.radians(self.ARROW_HALF_SPAN)) + head
        inner = g["rotor_r"] * 1.05          # just outside the selection ring
        width = ring - inner + head
        if rx > 0:
            return QRectF(mx + inner, my - half, width, half * 2)
        return QRectF(mx - ring - head, my - half, width, half * 2)

    def layout_fits(self) -> bool:
        """True when every drawn element is inside the widget rect."""
        g = self._layout()
        w, h = self.width(), self.height()
        for (mx, my) in self.rotor_centres().values():
            if mx - g["ring_r"] < 0 or mx + g["ring_r"] > w:
                return False
            if my - g["ring_r"] < 0 or my + g["ring_r"] > h:
                return False
        bounds = QRectF(0, 0, w, h)
        for num in self.rotor_centres():
            if not bounds.contains(self.arrow_rect(num)):
                return False
            if g["show_captions"] and not bounds.contains(self.caption_rect(num)):
                return False
        return True

    # ── interaction ─────────────────────────────────────────────────

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        radius = self._layout()["ring_r"]
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
        cx, cy, arm = g["cx"], g["cy"], g["arm"]

        self._paint_arms(p, cx, cy, arm)
        self._paint_body(p, cx, cy, arm)
        for num, label, sense, (rx, fy) in QUAD_X_LAYOUT:
            self._paint_rotor(p, g, num, label, sense, rx, fy)
        p.end()

    def _paint_arms(self, p: QPainter, cx: float, cy: float, arm: float) -> None:
        """Tapered carbon-style arms: wide at the body, narrow at the motor."""
        root_w, tip_w = arm * 0.15, arm * 0.065
        for _num, _label, _sense, (rx, fy) in QUAD_X_LAYOUT:
            tx, ty = cx + rx * arm, cy - fy * arm
            length = math.hypot(tx - cx, ty - cy) or 1.0
            ux, uy = (tx - cx) / length, (ty - cy) / length
            nx, ny = -uy, ux
            poly = QPolygonF([
                QPointF(cx + nx * root_w, cy + ny * root_w),
                QPointF(tx + nx * tip_w, ty + ny * tip_w),
                QPointF(tx - nx * tip_w, ty - ny * tip_w),
                QPointF(cx - nx * root_w, cy - ny * root_w),
            ])
            grad = QLinearGradient(cx - nx * root_w, cy - ny * root_w,
                                   cx + nx * root_w, cy + ny * root_w)
            grad.setColorAt(0.0, QColor(36, 41, 48))
            grad.setColorAt(0.5, QColor(58, 65, 74))
            grad.setColorAt(1.0, QColor(36, 41, 48))
            p.setPen(QPen(QColor(24, 28, 33), max(1, px(1))))
            p.setBrush(QBrush(grad))
            p.drawPolygon(poly)

    def _paint_body(self, p: QPainter, cx: float, cy: float, arm: float) -> None:
        """Centre plate, battery strap, camera pod and front/rear LEDs.

        The heading is carried by the drawing itself - camera pod and white LEDs
        forward, red LEDs aft - so no "NOSE" caption is needed.
        """
        bw, bh = arm * 0.62, arm * 0.92
        rect = QRectF(cx - bw / 2, cy - bh / 2, bw, bh)
        grad = QLinearGradient(rect.topLeft(), rect.bottomLeft())
        grad.setColorAt(0.0, QColor(52, 59, 68))
        grad.setColorAt(1.0, QColor(30, 35, 41))
        p.setPen(QPen(QColor(84, 92, 102), max(1, px(1))))
        p.setBrush(QBrush(grad))
        p.drawRoundedRect(rect, bw * 0.34, bw * 0.34)

        # Top plate.
        inset = bw * 0.12
        plate = rect.adjusted(inset, inset, -inset, -inset)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(38, 44, 51)))
        p.drawRoundedRect(plate, bw * 0.24, bw * 0.24)

        # Battery strap across the lower half.
        strap = QRectF(cx - bw * 0.36, cy + bh * 0.06, bw * 0.72, bh * 0.20)
        p.setBrush(QBrush(QColor(24, 28, 33)))
        p.drawRoundedRect(strap, bw * 0.08, bw * 0.08)
        p.setPen(QPen(QColor(64, 72, 82), max(1, px(1))))
        p.drawLine(QPointF(strap.left() + bw * 0.1, strap.center().y()),
                   QPointF(strap.right() - bw * 0.1, strap.center().y()))

        # Camera pod at the front.
        pod_r = bw * 0.22
        pod = QPointF(cx, cy - bh * 0.30)
        p.setPen(QPen(QColor(110, 118, 128), max(1, px(1))))
        p.setBrush(QBrush(QColor(16, 19, 23)))
        p.drawEllipse(pod, pod_r, pod_r)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(28, 34, 42)))
        p.drawEllipse(pod, pod_r * 0.62, pod_r * 0.62)
        p.setBrush(QBrush(QColor(120, 135, 150, 170)))
        p.drawEllipse(QPointF(pod.x() - pod_r * 0.2, pod.y() - pod_r * 0.2),
                      pod_r * 0.18, pod_r * 0.18)

        # LEDs: white forward, red aft.
        led = max(1.5, bw * 0.055)
        p.setBrush(QBrush(QColor(225, 232, 240)))
        for sx in (-1, 1):
            p.drawEllipse(QPointF(cx + sx * bw * 0.34, cy - bh * 0.43), led, led)
        p.setBrush(QBrush(QColor(196, 62, 62)))
        for sx in (-1, 1):
            p.drawEllipse(QPointF(cx + sx * bw * 0.34, cy + bh * 0.43), led, led)

    def _paint_rotor(self, p: QPainter, g: dict, num: int, label: str, sense: str,
                     rx: float, fy: float) -> None:
        rr = g["rotor_r"]
        mx = g["cx"] + rx * g["arm"]
        my = g["cy"] - fy * g["arm"]
        centre = QPointF(mx, my)
        pwm = self.pwms[num - 1]
        state = self.range.state(num, pwm)
        frac = self.range.fraction(num, pwm)
        colour = state_colour(state, self.stale)
        live = (not self.stale) and state != mr.OFF

        # Propeller disc: a faint tint of the state colour.
        faint = QColor(colour)
        faint.setAlpha(22 if self.stale else 38)
        p.setPen(QPen(colour.darker(130), max(1, px(1)), Qt.DotLine))
        p.setBrush(QBrush(faint))
        p.drawEllipse(centre, rr, rr)

        # Motion blur: a translucent wedge trailing each blade.
        angle = self._angles[num - 1]
        direction = 1.0 if sense == "CW" else -1.0
        if live and frac > 0.02:
            span = 24.0 + 70.0 * frac
            blur = QColor(colour)
            blur.setAlpha(int(40 + 110 * frac))
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(blur))
            box = QRectF(mx - rr, my - rr, rr * 2, rr * 2)
            for k in (0, 180):
                phi = angle + k                       # screen deg, clockwise from east
                if direction > 0:                     # CW: trail is behind (smaller phi)
                    start = -phi
                else:                                 # CCW: trail is ahead
                    start = -(phi + span)
                p.drawPie(box, int(start * 16), int(span * 16))

        # Blades.
        blade = QColor(150, 158, 168) if not self.stale else QColor(96, 103, 112)
        p.save()
        p.translate(centre)
        p.rotate(angle)
        p.setPen(QPen(QColor(40, 45, 52), max(1, px(1))))
        p.setBrush(QBrush(blade))
        for _ in range(2):
            p.drawEllipse(QPointF(rr * 0.52, 0.0), rr * 0.48, rr * 0.105)
            p.rotate(180)
        p.restore()

        # Motor hub with the channel number as a cap.
        hub = QRadialGradient(centre, rr * 0.36)
        hub.setColorAt(0.0, QColor(70, 78, 88))
        hub.setColorAt(1.0, QColor(26, 30, 36))
        p.setPen(QPen(QColor(100, 108, 118), max(1, px(1))))
        p.setBrush(QBrush(hub))
        p.drawEllipse(centre, rr * 0.36, rr * 0.36)
        p.setFont(scaled_font("Segoe UI", g["num_font_pt"], bold=True))
        p.setPen(QColor(235, 241, 247))
        p.drawText(QRectF(mx - rr * 0.5, my - rr * 0.5, rr, rr), Qt.AlignCenter, f"M{num}")

        # Rotation sense: an arc with a head on the rim, on the outer side.
        outward = 0.0 if rx > 0 else 180.0                   # screen deg, CW-from-east
        ring = g["ring_r"]
        half = self.ARROW_HALF_SPAN
        a0, a1 = outward - half, outward + half
        arrow_col = QColor(170, 179, 189)
        p.setPen(QPen(arrow_col, max(2, px(2)), Qt.SolidLine, Qt.RoundCap))
        p.setBrush(Qt.NoBrush)
        rect = QRectF(mx - ring, my - ring, ring * 2, ring * 2)
        p.drawArc(rect, int(-a1 * 16), int(2 * half * 16))
        end = a1 if sense == "CW" else a0
        ex = mx + ring * math.cos(math.radians(end))
        ey = my + ring * math.sin(math.radians(end))
        tangent = math.radians(end + (90.0 if sense == "CW" else -90.0))
        head = self._arrow_head(rr)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(arrow_col))
        p.drawPolygon(QPolygonF([
            QPointF(ex + head * math.cos(tangent), ey + head * math.sin(tangent)),
            QPointF(ex + head * 0.55 * math.cos(tangent + 2.4),
                    ey + head * 0.55 * math.sin(tangent + 2.4)),
            QPointF(ex + head * 0.55 * math.cos(tangent - 2.4),
                    ey + head * 0.55 * math.sin(tangent - 2.4)),
        ]))

        # Selection ring.
        if num == self.selected:
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor(88, 166, 255), max(2, px(2))))
            p.drawEllipse(centre, rr * 1.04, rr * 1.04)

        if not g["show_captions"]:
            return
        cap = self.caption_rect(num)
        line_h = cap.height() / 2.0
        p.setFont(scaled_font("Segoe UI", g["cap_font_pt"], bold=True))
        p.setPen(QColor(201, 209, 217))
        p.drawText(QRectF(cap.left(), cap.top(), cap.width(), line_h),
                   Qt.AlignCenter, f"{label} \u00b7 {sense}")
        p.setFont(scaled_font("Consolas", g["cap_font_pt"], bold=True))
        p.setPen(colour if not self.stale else COL_STALE)
        p.drawText(QRectF(cap.left(), cap.top() + line_h, cap.width(), line_h),
                   Qt.AlignCenter, f"{pwm} \u00b5s \u00b7 {int(round(frac * 100))}%")


class MotorChannelBar(QWidget):
    """Vertical bar gauge for one motor, scaled to that motor's own range."""

    def __init__(self, channel_num: int, label: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.channel_num = channel_num
        self.label = label
        self.pwm: int = 1000
        self.range: mr.MotorRange = mr.SHARED
        self.stale: bool = False
        self.setMinimumSize(px(66), px(160))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_range(self, rng: mr.MotorRange) -> None:
        self.range = rng
        self.update()

    def set_pwm(self, pwm: int, stale: Optional[bool] = None):
        self.pwm = max(0, min(3000, int(pwm)))
        if stale is not None:
            self.stale = bool(stale)
        self.update()

    def fraction(self) -> float:
        return self.range.fraction(self.channel_num, self.pwm)

    def state(self) -> str:
        return self.range.state(self.channel_num, self.pwm)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        bar_w = float(px(26))
        bar_x = (w - bar_w) / 2.0
        top_y = float(px(20))
        # Three readout lines live below the track (microseconds, percentage,
        # position), so the track has to stop above them.
        bot_y = h - px(52)
        bar_h = max(1.0, bot_y - top_y)

        pct = self.fraction()
        fill_h = bar_h * pct
        colour = state_colour(self.state(), self.stale)

        p.fillRect(QRectF(bar_x, top_y, bar_w, bar_h), QColor(22, 27, 34))
        p.setPen(QPen(QColor(48, 54, 61), 1))
        p.drawRect(QRectF(bar_x, top_y, bar_w, bar_h))

        if fill_h > 0:
            p.fillRect(QRectF(bar_x + 1, bot_y - fill_h, bar_w - 2, fill_h), colour)

        # Reference ticks at 0 / 50 / 100 % of THIS motor's range.
        p.setPen(QPen(QColor(88, 166, 255, 110), 1, Qt.DashLine))
        mid_y = top_y + (bar_h / 2.0)
        p.drawLine(QPointF(bar_x - px(4), mid_y), QPointF(bar_x + bar_w + px(4), mid_y))

        p.setFont(scaled_font("Segoe UI", 9, bold=True))
        p.setPen(QColor(201, 209, 217) if not self.stale else COL_STALE)
        p.drawText(QRectF(0, px(2), w, px(16)), Qt.AlignCenter, f"M{self.channel_num}")

        p.setFont(scaled_font("Consolas", 9, bold=True))
        p.setPen(colour)
        p.drawText(QRectF(0, h - px(48), w, px(15)), Qt.AlignCenter, f"{self.pwm} µs")

        p.setFont(scaled_font("Segoe UI", 8, bold=True))
        p.setPen(QColor(139, 148, 158) if not self.stale else COL_STALE)
        p.drawText(QRectF(0, h - px(33), w, px(14)), Qt.AlignCenter,
                   f"{int(round(pct * 100))}%")

        p.setFont(scaled_font("Segoe UI", 7))
        p.setPen(QColor(139, 148, 158) if not self.stale else COL_STALE)
        p.drawText(QRectF(0, h - px(18), w, px(14)), Qt.AlignCenter, self.label)

        p.end()


class ThrottleSlider(QSlider):
    """A throttle slider that is easy to hit and easy to read.

    The stock QSlider groove is a few pixels high with a handle barely wider -
    a fiddly target when you are also watching a propeller. This one is a
    QSlider underneath (same range / value / valueChanged / setEnabled API, so
    nothing that talks to it changes) with:

      * a thick filled track and a large round handle,
      * the whole widget as the hit area: press anywhere on it and the handle
        jumps there and keeps following the drag,
      * ticks every 5 %, with the hard ceiling marked in amber,
      * one step per mouse-wheel notch, and the usual keys.
    """

    TRACK_H = 10
    THUMB_D = 26
    TICK_STEP = 5

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(Qt.Horizontal, parent)
        self.setMinimumHeight(px(36))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.StrongFocus)
        self._dragging = False

    def sizeHint(self):
        from PyQt5.QtCore import QSize
        return QSize(px(180), px(36))

    # ── geometry ────────────────────────────────────────────────────
    def _span(self):
        pad = px(self.THUMB_D) / 2.0 + px(2)
        return pad, max(pad + 1.0, self.width() - pad)

    def _track_y(self) -> float:
        return self.height() * 0.42

    def value_to_x(self, value: int) -> float:
        lo, hi = self._span()
        rng = max(1, self.maximum() - self.minimum())
        return lo + (hi - lo) * (value - self.minimum()) / rng

    def x_to_value(self, x: float) -> int:
        lo, hi = self._span()
        ratio = max(0.0, min(1.0, (x - lo) / (hi - lo)))
        return int(round(self.minimum() + ratio * (self.maximum() - self.minimum())))

    # ── interaction ─────────────────────────────────────────────────
    def mousePressEvent(self, ev):
        if ev.button() != Qt.LeftButton or not self.isEnabled():
            return super().mousePressEvent(ev)
        self._dragging = True
        self.setSliderDown(True)
        self.setValue(self.x_to_value(ev.pos().x()))
        ev.accept()

    def mouseMoveEvent(self, ev):
        if self._dragging:
            self.setValue(self.x_to_value(ev.pos().x()))
            ev.accept()
        else:
            super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        if self._dragging:
            self._dragging = False
            self.setSliderDown(False)
            ev.accept()
        else:
            super().mouseReleaseEvent(ev)

    def wheelEvent(self, ev):
        if not self.isEnabled():
            return ev.ignore()
        steps = ev.angleDelta().y() // 120
        if steps:
            self.setValue(self.value() + int(steps))
        ev.accept()

    # ── painting ────────────────────────────────────────────────────
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        on = self.isEnabled()
        lo_x, hi_x = self._span()
        cy = self._track_y()
        th = float(px(self.TRACK_H))
        x = self.value_to_x(self.value())

        accent = QColor(88, 166, 255) if on else QColor(72, 79, 88)
        groove = QRectF(lo_x, cy - th / 2, hi_x - lo_x, th)
        p.setPen(QPen(QColor(48, 54, 61), 1))
        p.setBrush(QBrush(QColor(13, 17, 23)))
        p.drawRoundedRect(groove, th / 2, th / 2)

        # Filled part, with a soft highlight along the top edge.
        fill = QRectF(lo_x, cy - th / 2, max(th, x - lo_x), th)
        grad = QLinearGradient(fill.topLeft(), fill.bottomLeft())
        grad.setColorAt(0.0, accent.lighter(125))
        grad.setColorAt(1.0, accent.darker(115))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(grad))
        p.drawRoundedRect(fill, th / 2, th / 2)

        # Ticks every 5 %; the hard ceiling is the amber one. (The numbers live
        # in the value label above - repeating them here cost a whole text row.)
        tick_top = cy + th / 2 + px(3)
        lo_v, hi_v = self.minimum(), self.maximum()
        marks = [v for v in range(0, hi_v + 1, self.TICK_STEP) if lo_v <= v <= hi_v]
        if hi_v not in marks:
            marks.append(hi_v)
        for v in marks:
            tx = self.value_to_x(v)
            ceiling = (v == hi_v)
            col = QColor(210, 153, 34) if ceiling else QColor(110, 118, 128)
            if not on:
                col = QColor(72, 79, 88)
            p.setPen(QPen(col, max(1, px(1)) + (1 if ceiling else 0)))
            p.drawLine(QPointF(tx, tick_top), QPointF(tx, tick_top + px(6 if ceiling else 4)))

        # Handle: a large disc with a ring, bigger while pressed.
        d = px(self.THUMB_D) * (1.12 if self._dragging else 1.0)
        centre = QPointF(x, cy)
        if on:
            glow = QColor(accent)
            glow.setAlpha(70 if self._dragging else 40)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(glow))
            p.drawEllipse(centre, d / 2 + px(4), d / 2 + px(4))
        hub = QRadialGradient(QPointF(x - d * 0.15, cy - d * 0.2), d * 0.7)
        hub.setColorAt(0.0, QColor(245, 248, 252) if on else QColor(120, 127, 136))
        hub.setColorAt(1.0, QColor(190, 198, 208) if on else QColor(84, 91, 100))
        p.setPen(QPen(accent, max(2, px(2))))
        p.setBrush(QBrush(hub))
        p.drawEllipse(centre, d / 2, d / 2)
        # Grip ridges.
        p.setPen(QPen(QColor(120, 130, 142) if on else QColor(70, 76, 84), max(1, px(1))))
        for dx in (-px(3), 0, px(3)):
            p.drawLine(QPointF(x + dx, cy - d * 0.2), QPointF(x + dx, cy + d * 0.2))
        if self.hasFocus() and on:
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor(88, 166, 255, 150), 1, Qt.DotLine))
            p.drawEllipse(centre, d / 2 + px(6), d / 2 + px(6))
        p.end()


class _Check(QWidget):
    """One interlock condition: a status dot and a short word."""

    def __init__(self, text: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(px(5))
        self.dot = QLabel(self)
        self.dot.setFixedSize(px(8), px(8))
        lay.addWidget(self.dot, 0, Qt.AlignVCenter)
        self.lbl = QLabel(text, self)
        lay.addWidget(self.lbl)
        lay.addStretch(1)
        self.set_state(False)

    def set_state(self, ok: bool, danger: bool = False) -> None:
        # Grey = not satisfied, green = satisfied, red only for a real hazard
        # (the vehicle is armed). Colour carries meaning here, not decoration.
        colour = "#3fb950" if ok else ("#f85149" if danger else "#6e7681")
        self.dot.setStyleSheet(f"background: {colour}; border-radius: {px(4)}px;")
        self.lbl.setStyleSheet(
            f"color: {'#c9d1d9' if ok else '#8b949e'}; font-size: {px(11)}px;"
            " font-weight: 600; letter-spacing: 0.5px;")
        self.ok = ok


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
        self._compact = False
        self._expired_note = ""
        self.range: mr.MotorRange = mr.SHARED

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

        # Drives the visible countdown only; the real expiry is _arm_expiry.
        self._countdown = QTimer(self)
        self._countdown.setInterval(1000)
        self._countdown.timeout.connect(self._update_countdown)

        root = QVBoxLayout(self)
        root.setContentsMargins(px(12), px(6), px(12), px(4))
        root.setSpacing(px(4))

        head = QHBoxLayout()
        title = QLabel("BENCH MOTOR TEST", self)
        title.setObjectName("cardHeading")
        head.addWidget(title)
        head.addStretch()
        self.lbl_state = QLabel("", self)
        self.lbl_state.setObjectName("fieldSubLabel")
        head.addWidget(self.lbl_state)
        root.addLayout(head)

        # Interlock checklist: which of the four live conditions is holding the
        # test locked. (The fifth, the throttle ceiling, is the slider itself.)
        checks = QGridLayout()
        checks.setHorizontalSpacing(px(10))
        checks.setVerticalSpacing(px(0))
        self.chk_link = _Check("LINK", self)
        self.chk_disarmed = _Check("DISARMED", self)
        self.chk_ground = _Check("ON GROUND", self)
        self.chk_props = _Check("PROPS OFF", self)
        checks.addWidget(self.chk_link, 0, 0)
        checks.addWidget(self.chk_disarmed, 0, 1)
        checks.addWidget(self.chk_ground, 1, 0)
        checks.addWidget(self.chk_props, 1, 1)
        root.addLayout(checks)

        safety = QHBoxLayout()
        self.chk_safety = QCheckBox("PROPS OFF — enable motor test", self)
        self.chk_safety.setToolTip(
            "Confirms the propellers are physically off the aircraft.\n"
            "Re-locks automatically after 60 seconds.")
        self.chk_safety.setStyleSheet(
            "QCheckBox { color: #d29922; font-weight: bold; }")
        self.chk_safety.toggled.connect(self._on_safety_toggled)
        safety.addWidget(self.chk_safety, 1)
        self.lbl_countdown = QLabel("", self)
        self.lbl_countdown.setObjectName("valueMono")
        self.lbl_countdown.setToolTip("Time until the props-off confirmation re-locks")
        safety.addWidget(self.lbl_countdown)
        root.addLayout(safety)

        # 2x2, laid out like the aircraft seen from above: front row on top.
        sel = QGridLayout()
        sel.setHorizontalSpacing(px(6))
        sel.setVerticalSpacing(px(4))
        self._sel_grid = sel
        self.motor_buttons = {}
        cells = {3: (0, 0), 1: (0, 1), 2: (1, 0), 4: (1, 1)}
        self._sel_cells = cells
        self._sel_text = {}
        for num, label, sense, _pos in QUAD_X_LAYOUT:
            arrow = "↻" if sense == "CW" else "↺"
            self._sel_text[num] = f"M{num}  {label} {arrow}"
            b = QPushButton(self._sel_text[num], self)
            b.setCheckable(True)
            b.setChecked(num == 1)
            b.setToolTip(f"Motor {num} — {label}, {sense}")
            b.setMinimumWidth(px(78))
            b.setFixedHeight(px(24))
            b.clicked.connect(lambda _c, n=num: self.set_selected(n))
            self.motor_buttons[num] = b
            sel.addWidget(b, *cells[num])
        root.addLayout(sel)

        thr_head = QHBoxLayout()
        lbl_thr = QLabel("Throttle", self)
        lbl_thr.setObjectName("fieldLabel")
        thr_head.addWidget(lbl_thr)
        thr_head.addStretch(1)
        self.lbl_throttle = QLabel("", self)
        self.lbl_throttle.setObjectName("valueMono")
        self.lbl_throttle.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        thr_head.addWidget(self.lbl_throttle)
        root.addLayout(thr_head)

        thr = QHBoxLayout()
        thr.setSpacing(px(6))
        self.btn_thr_minus = self._step_button("−", -1)
        thr.addWidget(self.btn_thr_minus, 0, Qt.AlignTop)
        self.slider_throttle = ThrottleSlider(self)
        self.slider_throttle.setRange(1, self.MAX_THROTTLE_PCT)
        self.slider_throttle.setValue(8)
        self.slider_throttle.setToolTip(
            f"Throttle for the selected motor. Hard limit {self.MAX_THROTTLE_PCT} %.\n"
            "Click or drag anywhere on the bar; mouse wheel and arrow keys step by 1.\n"
            "The µs figure is what this vehicle's output is driven to (its own min..max).")
        self.slider_throttle.valueChanged.connect(self._on_throttle_changed)
        thr.addWidget(self.slider_throttle, 1)
        self.btn_thr_plus = self._step_button("+", +1)
        thr.addWidget(self.btn_thr_plus, 0, Qt.AlignTop)
        root.addLayout(thr)

        actions = QVBoxLayout()
        actions.setSpacing(px(4))

        self.btn_spin = QPushButton("HOLD TO SPIN", self)
        self.btn_spin.setObjectName("btnNav")
        self.btn_spin.setFixedHeight(px(28))
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
            "direction of rotation against the drone diagram.")
        self.btn_sequence.toggled.connect(self._on_sequence_toggled)
        self.btn_sequence.setFixedHeight(px(24))
        second_row = QHBoxLayout()
        second_row.setSpacing(px(6))
        second_row.addWidget(self.btn_sequence, 1)

        self.btn_stop = QPushButton("STOP ALL", self)
        self.btn_stop.setObjectName("btnKill")
        self.btn_stop.setToolTip("Command zero throttle on every channel (Esc)")
        self.btn_stop.setFixedHeight(px(24))
        self.btn_stop.clicked.connect(self.stop_all)
        second_row.addWidget(self.btn_stop, 1)
        actions.addLayout(second_row)
        root.addLayout(actions)

        self.lbl_reason = QLabel("", self)
        self.lbl_reason.setObjectName("fieldSubLabel")
        self.lbl_reason.setWordWrap(True)
        root.addWidget(self.lbl_reason)

        # Absorb the slack at the bottom so the controls stay one sequence of
        # steps instead of drifting apart as the window grows.
        root.addStretch(1)

        self._update_output_label()
        self._refresh_enabled()

    # ── room ────────────────────────────────────────────────────────

    def set_compact(self, compact: bool) -> None:
        """Short page: swap the four-dot checklist for the one-line reason, which
        says the same thing (what is blocking the test) in a third of the height."""
        if compact == self._compact:
            return
        self._compact = compact
        for chk in (self.chk_link, self.chk_disarmed, self.chk_ground, self.chk_props):
            chk.setVisible(not compact)
        # The 2x2 motor selector (laid out like the aircraft) folds into one row of
        # four short buttons; the drone diagram still selects any rotor too.
        for num, b in self.motor_buttons.items():
            self._sel_grid.removeWidget(b)
            if compact:
                b.setText(f"M{num}")
                b.setMinimumWidth(px(40))
                self._sel_grid.addWidget(b, 0, num - 1)
            else:
                b.setText(self._sel_text[num])
                b.setMinimumWidth(px(78))
                self._sel_grid.addWidget(b, *self._sel_cells[num])
        self._refresh_enabled()

    def _step_button(self, text: str, delta: int) -> QPushButton:
        """A big −/+ for fine steps; holding it repeats."""
        b = QPushButton(text, self)
        b.setFixedSize(px(30), px(30))
        b.setAutoRepeat(True)
        b.setAutoRepeatDelay(350)
        b.setAutoRepeatInterval(90)
        b.setToolTip(f"{'Lower' if delta < 0 else 'Raise'} throttle by 1 %")
        b.setStyleSheet(f"font-size: {px(16)}px; font-weight: 700; padding: 0;")
        b.clicked.connect(lambda: self.slider_throttle.setValue(
            self.slider_throttle.value() + delta))
        return b

    # ── range ───────────────────────────────────────────────────────

    def set_range(self, rng: mr.MotorRange) -> None:
        self.range = rng
        self._update_output_label()

    def _update_output_label(self) -> None:
        """"8 % · 1180 µs": the throttle, and what it means on THIS vehicle."""
        pct = int(self.throttle_pct())
        pwm = self.range.pwm_for_throttle(self._selected, pct)
        self.lbl_throttle.setText(f"{pct} / {self.MAX_THROTTLE_PCT} % · {pwm} µs")

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

    def _short_reason(self) -> str:
        """One-line form of _block_reason() for a page too short to wrap it."""
        if not self._connected:
            return "No link."
        if self._armed:
            return "ARMED - disarm to test."
        if self._airborne:
            return "Airborne."
        if not self.chk_safety.isChecked():
            return "Confirm props are off."
        return ""

    def _refresh_enabled(self) -> None:
        allowed = self.testing_allowed()
        for w in (self.btn_spin, self.btn_sequence, self.slider_throttle,
                  self.btn_thr_minus, self.btn_thr_plus):
            w.setEnabled(allowed)
        for b in self.motor_buttons.values():
            b.setEnabled(allowed)
        # STOP ALL stays live whenever there is a link. It is the one control
        # that must work in exactly the states the others are disabled for.
        self.btn_stop.setEnabled(self._connected)
        self.chk_safety.setEnabled(
            self._connected and not self._armed and not self._airborne)

        self.chk_link.set_state(self._connected)
        self.chk_disarmed.set_state(self._connected and not self._armed,
                                    danger=self._armed)
        self.chk_ground.set_state(self._connected and not self._airborne,
                                  danger=self._airborne)
        self.chk_props.set_state(self.chk_safety.isChecked())

        reason = self._short_reason() if self._compact else self._block_reason()
        # The checklist already shows "no link" / "props not confirmed"; the line
        # is for what it cannot say - armed, airborne, or the 60 s expiry.
        if not (self._armed or self._airborne) and not self._compact:
            reason = "" if not self._expired_note else self._expired_note
        elif self._compact and not reason and self._expired_note:
            reason = self._expired_note
        self.lbl_reason.setText(reason)
        self.lbl_reason.setVisible(bool(reason))
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
        self._update_output_label()
        self.selection_changed.emit(self._selected)

    def selected_motor(self) -> int:
        return self._selected

    def throttle_pct(self) -> float:
        return float(self.slider_throttle.value())

    def _on_throttle_changed(self, value: int) -> None:
        self._update_output_label()

    # ── safety acknowledgement ──────────────────────────────────────

    def _on_safety_toggled(self, checked: bool) -> None:
        if checked:
            self._expired_note = ""
            self._arm_expiry.start()
            self._countdown.start()
            self._update_countdown()
        else:
            self._arm_expiry.stop()
            self._countdown.stop()
            self.lbl_countdown.setText("")
            self.stop_all()
        self._refresh_enabled()

    def _update_countdown(self) -> None:
        remaining_ms = self._arm_expiry.remainingTime()
        if remaining_ms < 0:
            self.lbl_countdown.setText("")
            return
        secs = (remaining_ms + 999) // 1000
        self.lbl_countdown.setText(f"locks in {secs // 60}:{secs % 60:02d}")

    def _expire_safety(self) -> None:
        self.stop_all()
        self.chk_safety.setChecked(False)
        self._expired_note = ("Safety acknowledgement expired after 60 s — "
                              "re-confirm to test again.")
        self._refresh_enabled()

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
    """The actuator workspace: drone diagram, PWM bars and the test panel."""

    motor_test_requested = pyqtSignal(int, float)
    motor_test_stop_requested = pyqtSignal()

    # While a bench test is commanded the commanded output is shown, re-armed by
    # every repeat (700 ms). After a stop it is held briefly at idle so the last
    # stale live sample cannot flash the motor back up.
    COMMAND_HOLD_S = 2.5
    STOP_HOLD_S = 0.6

    def __init__(self, parent: Optional[QWidget] = None, clock=time.monotonic):
        super().__init__(parent)
        self._clock = clock
        self._fitting = False
        self._level = 0
        self.range: mr.MotorRange = mr.SHARED
        self._live_out: List[int] = [1000, 1000, 1000, 1000]   # output order
        self._age_s: Optional[float] = None
        self._cmd_motor_pwms: Optional[List[int]] = None       # motor order
        self._cmd_started = 0.0
        self._cmd_until = 0.0
        self._cmd_motor = 1
        self._cmd_pct = 0.0
        self._cmd_is_stop = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(px(12), px(8), px(12), px(6))
        layout.setSpacing(px(8))

        title_box = QHBoxLayout()
        t = QLabel("ACTUATOR OUTPUTS & MOTOR TELEMETRY")
        t.setObjectName("pageTitle")
        title_box.addWidget(t)
        title_box.addStretch()

        self.lbl_status = QLabel("")
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
        geo_hint.setWordWrap(True)
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
        body.addWidget(card, 2)

        # Right: the only controls in this workspace that command anything.
        self.test_panel = MotorTestPanel(self)
        self.test_panel.test_requested.connect(self.motor_test_requested)
        self.test_panel.test_requested.connect(self._reflect_test_locally)
        self.test_panel.stop_requested.connect(self.motor_test_stop_requested)
        self.test_panel.stop_requested.connect(self._reflect_stop_locally)
        self.test_panel.selection_changed.connect(self.geometry.set_selected)
        body.addWidget(self.test_panel, 4)

        layout.addLayout(body, 1)

        # Where the numbers come from, and the thresholds the colours use.
        self.lbl_scale = QLabel("")
        self.lbl_scale.setStyleSheet(
            f"color: #8b949e; font-size: {px(10)}px; font-style: italic;")
        self.lbl_scale.setWordWrap(True)
        layout.addWidget(self.lbl_scale)

        # Re-render when a COMMANDED hold lapses, with no telemetry to trigger it.
        self._hold_timer = QTimer(self)
        self._hold_timer.setSingleShot(True)
        self._hold_timer.timeout.connect(self._render)

        self.set_range(self.range)

    # ── room ────────────────────────────────────────────────────────

    def _apply_level(self, level: int) -> None:
        """0 = everything; 1 = no footnote; 2 = also the compact test panel."""
        self.lbl_scale.setVisible(level == 0)
        self.test_panel.set_compact(level >= 2)

    def _fits(self) -> bool:
        self.layout().activate()
        return self.height() >= self.layout().minimumSize().height()

    def _fit(self, allow_relax: bool) -> None:
        """Shed detail, least important first, until the page fits - never let the
        Bench Motor Test controls be squeezed into overlapping. The footnote is
        reference, not control, so it goes first; then the checklist gives way to
        the one-line reason.

        STABLE BY CONSTRUCTION: on a layout change the level only ever goes UP
        (more compact) until it fits, so the page converges instead of
        oscillating (every toggle posts another layout request). Going back DOWN
        to more detail is tried only on a real resize, one level at a time, and
        undone at once if it does not fit.
        """
        if self._fitting:
            return
        self._fitting = True
        try:
            while not self._fits() and self._level < 2:
                self._level += 1
                self._apply_level(self._level)
            if allow_relax:
                while self._level > 0:
                    self._apply_level(self._level - 1)
                    if self._fits():
                        self._level -= 1
                    else:
                        self._apply_level(self._level)
                        break
        finally:
            self._fitting = False

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._fit(allow_relax=True)

    def event(self, ev):
        # The room NEEDED changes without the window being resized - arming the
        # vehicle adds a warning line to the test panel, for one. Check on every
        # layout request, not only on resize (but never relax from one).
        handled = super().event(ev)
        if ev.type() == QEvent.LayoutRequest:
            self._fit(allow_relax=False)
        return handled

    # ── range / parameters ──────────────────────────────────────────

    def set_range(self, rng: mr.MotorRange) -> None:
        self.range = rng
        for bar in self.bars.values():
            bar.set_range(rng)
        self.geometry.set_range(rng)
        self.test_panel.set_range(rng)
        self._render()

    def on_motor_param(self, name: str, value: float) -> None:
        """Feed a PWM_MAIN_* parameter. Re-renders only if it changed the model."""
        if self.range.on_param(name, value):
            self.set_range(self.range)

    # ── vehicle state / actions ─────────────────────────────────────

    def _on_geometry_clicked(self, motor_num: int) -> None:
        self.test_panel.set_selected(motor_num)

    def set_vehicle_state(self, connected: bool, armed: bool, airborne: bool) -> None:
        self.test_panel.set_vehicle_state(connected, armed, airborne)

    def stop_motor_tests(self) -> None:
        self.test_panel.stop_all()

    def _idle_motor_pwms(self) -> List[int]:
        return [self.range.disarm_pwm(k) for k in (1, 2, 3, 4)]

    def _reflect_test_locally(self, motor_num: int, throttle_pct: float) -> None:
        """Show the commanded output immediately, marked COMMANDED.

        SERVO_OUTPUT_RAW arrives at 5 Hz and PX4 may not reflect an
        ACTUATOR_TEST-driven output in it at all, so without this a motor could
        audibly spin while every bar read idle. The commanded value is mapped
        through the motor's own min..max, exactly as PX4 maps it. A live sample
        that confirms it takes over (see _render).
        """
        if motor_num not in self.bars:
            return
        pcts = max(0.0, min(100.0, float(throttle_pct)))
        pwms = self._idle_motor_pwms()
        pwms[motor_num - 1] = self.range.pwm_for_throttle(motor_num, pcts)
        now = self._clock()
        self._cmd_motor_pwms = pwms
        self._cmd_motor, self._cmd_pct, self._cmd_is_stop = motor_num, pcts, False
        if now >= self._cmd_until:
            self._cmd_started = now          # a new command, not a repeat of one
        self._cmd_until = now + self.COMMAND_HOLD_S
        self._hold_timer.start(int(self.COMMAND_HOLD_S * 1000) + 50)
        self._render()

    def _reflect_stop_locally(self) -> None:
        """Mirror STOP ALL / hold-release / safety-expiry back to idle at once."""
        now = self._clock()
        self._cmd_motor_pwms = self._idle_motor_pwms()
        self._cmd_is_stop = True
        self._cmd_started = now
        self._cmd_until = now + self.STOP_HOLD_S
        self._hold_timer.start(int(self.STOP_HOLD_S * 1000) + 50)
        self._render()

    # ── data in ─────────────────────────────────────────────────────

    def update_pwms(self, pwms: List[int], age_s: Optional[float] = None):
        """Live SERVO_OUTPUT_RAW (output order). ``age_s`` is seconds since the
        last sample arrived; None means "fresh" (callers that do not track it)."""
        if len(pwms) < 4:
            return
        self._live_out = [int(v) for v in pwms[:4]]
        self._age_s = age_s
        self._render()

    # ── presentation ────────────────────────────────────────────────

    def _live_confirms_command(self) -> bool:
        """A live sample newer than the command, with the commanded motor
        actually moving toward what was asked, replaces the COMMANDED mirror."""
        if self._age_s is None or self._cmd_is_stop or self._cmd_motor_pwms is None:
            return False
        if self._age_s >= (self._clock() - self._cmd_started):
            return False                      # sample predates the command
        live = self.range.motor_pwms(self._live_out)
        k = self._cmd_motor
        want = self.range.fraction(k, self._cmd_motor_pwms[k - 1])
        got = self.range.fraction(k, live[k - 1])
        return got >= 0.5 * want

    def _commanded_label(self) -> str:
        """Why the COMMANDED mirror is on screen instead of live data."""
        if self._age_s is None:
            return "COMMANDED"
        if self._age_s > mr.STALE_S:
            return "COMMANDED · no motor data"
        if self._age_s < (self._clock() - self._cmd_started):
            return "COMMANDED · not yet confirmed"     # a newer sample, not moving
        return "COMMANDED"

    def _render(self) -> None:
        now = self._clock()
        commanded = now < self._cmd_until and self._cmd_motor_pwms is not None
        stale = False
        source = "LIVE"
        if commanded and (self._cmd_is_stop or not self._live_confirms_command()):
            pwms = list(self._cmd_motor_pwms)
            source = "LIVE" if self._cmd_is_stop else self._commanded_label()
        else:
            pwms = self.range.motor_pwms(self._live_out)
            if self._age_s is not None:
                stale = self._age_s > mr.STALE_S

        for num in (1, 2, 3, 4):
            self.bars[num].set_pwm(pwms[num - 1], stale)
        self.geometry.set_pwms(pwms, stale)
        self._update_status(pwms, stale, source)
        self._update_scale_note()

    def _update_status(self, pwms: List[int], stale: bool, source: str) -> None:
        if stale:
            if self._age_s is not None and self._age_s >= 900:
                text, colour = "NO MOTOR DATA", "#8b949e"
            else:
                text, colour = f"MOTOR DATA STALE · {self._age_s:.0f} s", "#d29922"
        else:
            worst = self.range.worst_state(pwms)
            if worst == mr.SATURATED:
                hot = max(range(4), key=lambda i: self.range.fraction(i + 1, pwms[i]))
                text = f"SATURATION · M{hot + 1} {pwms[hot]} µs"
                colour = "#f85149"
            elif worst == mr.HIGH:
                text, colour = "HIGH LOAD", "#d29922"
            elif worst == mr.NOMINAL:
                top = max(self.range.fraction(i + 1, pwms[i]) for i in range(4))
                text, colour = f"MOTORS ACTIVE · {int(round(top * 100))}%", "#58a6ff"
            elif worst == mr.IDLE:
                text, colour = "ALL MOTORS IDLE", "#8b949e"
            else:
                text, colour = "ALL MOTORS OFF", "#8b949e"
            if source.startswith("COMMANDED"):
                text += f" · {source}"
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet(
            f"color: {colour}; font-size: {px(11)}px; font-weight: bold;")

    def _update_scale_note(self) -> None:
        self.lbl_scale.setText(
            f"Scale: {self.range.describe()}  |  idle ≤ {int(mr.IDLE_FRAC * 100)} %  ·  "
            f"high > {int(mr.NOMINAL_FRAC * 100)} %  ·  saturated > {int(mr.HIGH_FRAC * 100)} %")
