"""
================================================================================
MODULE: guided_confirm.py
PURPOSE: Drag-to-set value slider + slide-to-confirm bar for guided actions
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station (GCS Cockpit Workspace)
  * Upstream:      drone_gcs.py - TAKEOFF, ROTATE YAW, CHANGE ALT, ARM,
                   DISARM-while-airborne, EMERGENCY KILL
  * Downstream:    Nothing. This widget dispatches no MAVLink; it emits a
                   confirmed(action, value) signal and the main window runs the
                   same _cmd_* method a mouse click has always run, interlocks
                   and all.

WHY THIS EXISTS:
  Two distinct problems with the same fix.

  (1) Irreversible actions were one click deep. EMERGENCY KILL was a plain
      QPushButton behind a Yes/No QMessageBox - and a modal whose default button
      is one Return keypress away is not a safety gate, it is a speed bump.
      ARM had no confirmation whatsoever, despite this module's own main-window
      docstring claiming a "dual-stage confirmation modal". A deliberate
      horizontal drag cannot be produced by a mis-click, a double-click landing
      on a dialog that appeared under the cursor, or a stray Return.

  (2) Numeric entry was type-then-press. Setting a takeoff altitude meant
      selecting a QLineEdit, clearing it, typing, then finding the button -
      four interactions, with a typo landing a real altitude into a real
      aircraft. A slider bounded by settings.limits cannot express 15 m when the
      ceiling is 3 m, and shows the value growing as you drag it.

THE SLIDER IS BOUNDED BY THE SAME LIMITS THE COMMAND CHECKS:
  Callers pass vmin/vmax straight from core.settings.LimitsConfig. This widget
  does not re-implement the check - the _cmd_* methods still validate - but an
  operator should not be able to *express* an out-of-range value and then be
  told no. Refusing to render the impossible beats rejecting it after the fact.

NOTHING HERE IS MODAL:
  A modal dialog over a flying aircraft blocks the telemetry the operator needs
  in order to decide. This is an inline strip: the HUD, the map and every badge
  keep updating behind it, and Escape cancels.

USAGE:
  bar = GuidedConfirmBar(parent)
  bar.confirmed.connect(on_confirmed)      # (action_key, value)
  bar.request("takeoff", "TAKEOFF", value_label="Altitude",
              vmin=0.2, vmax=3.0, vinit=1.0, unit="m", step=0.1)
  bar.request("kill", "EMERGENCY MOTOR KILL", danger=True,
              detail="Cuts all motor outputs immediately.")
================================================================================
"""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import Qt, QRectF, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont
from PyQt5.QtWidgets import (
    QWidget, QFrame, QLabel, QHBoxLayout, QVBoxLayout, QPushButton, QSlider,
    QSizePolicy,
)

from ui.styles import PALETTE
from ui.scaling import px, pt, get_scale


class SlideToConfirm(QWidget):
    """Drag the knob fully to the right to confirm.

    Modelled on QGroundControl's SliderSwitch. The knob snaps back on release
    below the commit threshold, so a partial drag is an explicit "no" rather
    than an ambiguous state the operator has to undo.
    """

    confirmed = pyqtSignal()

    # Fraction of the travel that counts as committed. Not 1.0: requiring the
    # knob to reach the exact final pixel makes the control feel broken on a
    # trackpad, and 90% of a deliberate drag is still unmistakably deliberate.
    COMMIT_FRACTION = 0.90

    def __init__(self, text: str = "Slide to confirm", danger: bool = False,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._text = text
        self._danger = danger
        self._position = 0.0          # 0..1 along the travel
        self._dragging = False
        self._grab_dx = 0.0
        self.setCursor(Qt.OpenHandCursor)
        self.setMinimumSize(px(230), px(34))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    # ── state ───────────────────────────────────────────────────────

    def set_text(self, text: str) -> None:
        self._text = text
        self.update()

    def set_danger(self, danger: bool) -> None:
        self._danger = bool(danger)
        self.update()

    def reset(self) -> None:
        self._position = 0.0
        self._dragging = False
        self.setCursor(Qt.OpenHandCursor)
        self.update()

    @property
    def position(self) -> float:
        return self._position

    # ── geometry ────────────────────────────────────────────────────

    def _knob_diameter(self) -> float:
        return max(px(20), self.height() - px(8))

    def _travel(self) -> float:
        return max(1.0, self.width() - self._knob_diameter() - px(8))

    def _knob_x(self) -> float:
        return px(4) + self._position * self._travel()

    # ── interaction ─────────────────────────────────────────────────

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        knob_x = self._knob_x()
        d = self._knob_diameter()
        # Grabbing the track rather than the knob still starts a drag: the knob
        # is small, and forcing a precise hit on it makes a safety control feel
        # unreliable, which teaches operators to hammer at it.
        self._dragging = True
        if knob_x <= event.pos().x() <= knob_x + d:
            self._grab_dx = event.pos().x() - knob_x
        else:
            self._grab_dx = d / 2.0
            self._update_from_x(event.pos().x())
        self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._update_from_x(event.pos().x())

    def mouseReleaseEvent(self, event):
        if not self._dragging:
            return
        self._dragging = False
        self.setCursor(Qt.OpenHandCursor)
        if self._position >= self.COMMIT_FRACTION:
            self._position = 1.0
            self.update()
            self.confirmed.emit()
        else:
            self.reset()

    def _update_from_x(self, x: float) -> None:
        self._position = max(0.0, min(1.0, (x - self._grab_dx - px(4)) / self._travel()))
        self.update()

    # ── painting ────────────────────────────────────────────────────

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)

        w, h = self.width(), self.height()
        radius = h / 2.0
        accent = QColor(PALETTE["danger"] if self._danger else PALETTE["nav"])
        fill = QColor(PALETTE["danger_fill"] if self._danger else PALETTE["nav_fill"])

        # Track
        p.setPen(QPen(accent.darker(130), max(1, px(1))))
        p.setBrush(QBrush(fill))
        p.drawRoundedRect(QRectF(0.5, 0.5, w - 1, h - 1), radius, radius)

        # Progress fill behind the knob, so travel is visible as it happens
        # rather than only at the moment of commit.
        d = self._knob_diameter()
        if self._position > 0.001:
            progress = QColor(accent)
            progress.setAlpha(70)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(progress))
            p.drawRoundedRect(QRectF(1, 1, self._knob_x() + d - 1, h - 2),
                              radius, radius)

        # Caption, faded out as the knob approaches it
        p.setFont(QFont(PALETTE["font_ui"].split(",")[0].strip("'"),
                        pt(9), QFont.Bold))
        label = QColor(accent)
        label.setAlpha(int(235 * (1.0 - 0.6 * self._position)))
        p.setPen(label)
        p.drawText(QRectF(d, 0, w - d - px(8), h), Qt.AlignCenter, self._text)

        # Knob
        p.setPen(QPen(accent, max(1, px(1))))
        p.setBrush(QBrush(QColor(PALETTE["bg_raised"])))
        p.drawEllipse(QRectF(self._knob_x(), px(4), d, d))

        # Chevrons on the knob pointing the way to drag
        p.setPen(QPen(accent, max(1, px(2)), Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        cx = self._knob_x() + d / 2.0
        cy = px(4) + d / 2.0
        arm = d * 0.16
        for offset in (-arm, arm * 0.6):
            p.drawLine(int(cx + offset - arm * 0.5), int(cy - arm),
                       int(cx + offset + arm * 0.5), int(cy))
            p.drawLine(int(cx + offset + arm * 0.5), int(cy),
                       int(cx + offset - arm * 0.5), int(cy + arm))
        p.end()


class ValueSlider(QWidget):
    """A bounded numeric slider with a large live readout and its unit.

    Works in integer steps internally because QSlider is integral; `step` sets
    the resolution, so a 0.1 m step over 0.2..3.0 m gives 28 positions and every
    reachable value is one the operator could also have typed.
    """

    value_changed = pyqtSignal(float)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._vmin = 0.0
        self._vmax = 1.0
        self._step = 0.1
        self._unit = ""
        self._decimals = 1

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(px(2))

        top = QHBoxLayout()
        top.setSpacing(px(8))
        self.lbl_caption = QLabel("", self)
        self.lbl_caption.setObjectName("fieldLabel")
        top.addWidget(self.lbl_caption)
        top.addStretch()

        self.lbl_value = QLabel("--", self)
        self.lbl_value.setStyleSheet(
            f"color: {PALETTE['info']}; font-family: {PALETTE['font_mono']};"
            f" font-size: {int(17 * get_scale())}px; font-weight: bold;")
        top.addWidget(self.lbl_value)
        lay.addLayout(top)

        self.slider = QSlider(Qt.Horizontal, self)
        self.slider.setMinimumWidth(px(200))
        self.slider.valueChanged.connect(self._on_slider)
        lay.addWidget(self.slider)

        self.lbl_bounds = QLabel("", self)
        self.lbl_bounds.setObjectName("fieldSubLabel")
        lay.addWidget(self.lbl_bounds)

    def configure(self, caption: str, vmin: float, vmax: float, vinit: float,
                  unit: str = "", step: float = 0.1) -> None:
        self._vmin, self._vmax = float(vmin), float(vmax)
        self._step = max(1e-6, float(step))
        self._unit = unit
        self._decimals = 0 if self._step >= 1.0 else (1 if self._step >= 0.1 else 2)

        steps = max(1, int(round((self._vmax - self._vmin) / self._step)))
        self.lbl_caption.setText(caption)
        self.lbl_bounds.setText(
            f"{self._fmt(self._vmin)} … {self._fmt(self._vmax)}")

        self.slider.blockSignals(True)
        self.slider.setRange(0, steps)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(max(1, steps // 10))
        self.slider.setValue(self._to_slider(vinit))
        self.slider.blockSignals(False)
        self._refresh_readout()

    def _to_slider(self, value: float) -> int:
        clamped = max(self._vmin, min(self._vmax, float(value)))
        return int(round((clamped - self._vmin) / self._step))

    def value(self) -> float:
        raw = self._vmin + self.slider.value() * self._step
        return round(max(self._vmin, min(self._vmax, raw)), 4)

    def _fmt(self, v: float) -> str:
        return f"{v:+.{self._decimals}f} {self._unit}".strip() if self._vmin < 0 \
            else f"{v:.{self._decimals}f} {self._unit}".strip()

    def _refresh_readout(self) -> None:
        self.lbl_value.setText(self._fmt(self.value()))

    def _on_slider(self, _v: int) -> None:
        self._refresh_readout()
        self.value_changed.emit(self.value())


class GuidedConfirmBar(QFrame):
    """The inline strip: title, optional value slider, slide-to-confirm, cancel.

    One bar is reused for every action rather than one widget per action, so the
    confirmation gesture is identical everywhere and there is exactly one place
    where "has the operator confirmed?" is decided.
    """

    confirmed = pyqtSignal(str, float)   # action key, value (0.0 when no slider)
    cancelled = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setProperty("class", "cardFrame")
        self._action: Optional[str] = None
        # Tracked explicitly rather than read back off the slider's isVisible().
        # A child of a window that has not been shown yet reports itself as not
        # visible regardless of setVisible(True), so asking Qt "is there a
        # slider?" silently returned a 0.0 altitude for a confirmed takeoff.
        self._has_slider = False

        lay = QVBoxLayout(self)
        lay.setContentsMargins(px(12), px(9), px(12), px(10))
        lay.setSpacing(px(7))

        head = QHBoxLayout()
        head.setSpacing(px(8))
        self.lbl_title = QLabel("", self)
        head.addWidget(self.lbl_title)
        head.addStretch()
        self.btn_cancel = QPushButton("Cancel", self)
        self.btn_cancel.setToolTip("Dismiss without sending (Esc)")
        self.btn_cancel.clicked.connect(self.cancel)
        head.addWidget(self.btn_cancel)
        lay.addLayout(head)

        self.lbl_detail = QLabel("", self)
        self.lbl_detail.setObjectName("fieldSubLabel")
        self.lbl_detail.setWordWrap(True)
        lay.addWidget(self.lbl_detail)

        self.value_slider = ValueSlider(self)
        lay.addWidget(self.value_slider)

        self.slide = SlideToConfirm(parent=self)
        self.slide.confirmed.connect(self._on_confirmed)
        lay.addWidget(self.slide)

        self.hide()

    # ── lifecycle ───────────────────────────────────────────────────

    def request(self, action: str, title: str, *, detail: str = "",
                value_label: str = "", vmin: float = 0.0, vmax: float = 0.0,
                vinit: float = 0.0, unit: str = "", step: float = 0.1,
                danger: bool = False, confirm_text: str = "") -> None:
        """Show the bar for `action`. A zero-width range means no slider.

        Re-requesting while another action is pending replaces it outright and
        resets the knob - a half-dragged confirmation for a takeoff must never
        carry over into a kill.
        """
        self._action = action
        colour = PALETTE["danger"] if danger else PALETTE["accent"]
        self.lbl_title.setText(title)
        self.lbl_title.setStyleSheet(
            f"color: {colour}; font-weight: bold; letter-spacing: 0.8px;"
            f" font-size: {int(12 * get_scale())}px;")
        self.lbl_detail.setText(detail)
        self.lbl_detail.setVisible(bool(detail))

        self._has_slider = vmax > vmin
        self.value_slider.setVisible(self._has_slider)
        if self._has_slider:
            self.value_slider.configure(value_label or "Value", vmin, vmax,
                                        vinit, unit, step)

        self.slide.set_danger(danger)
        self.slide.set_text(confirm_text or f"Slide to {title.split()[0].lower()}")
        self.slide.reset()

        self.setStyleSheet(
            f"QFrame[class=\"cardFrame\"] {{ border: 1px solid {colour}; }}")
        self.show()

    def cancel(self) -> None:
        """Dismiss without sending. Safe to call when nothing is pending."""
        if self._action is None:
            self.hide()
            return
        action, self._action = self._action, None
        self.slide.reset()
        self.hide()
        self.cancelled.emit(action)

    def is_active(self) -> bool:
        """Whether an action is awaiting confirmation.

        Deliberately does not consult isVisible(): "is something pending?" is a
        question about this widget's state, not about whether Qt has painted it
        yet, and Escape must cancel a pending action either way.
        """
        return self._action is not None

    def active_action(self) -> Optional[str]:
        return self._action

    def pending_value(self) -> Optional[float]:
        """The value the slider currently shows, or None for a bare confirm."""
        return self.value_slider.value() if self._has_slider else None

    def _on_confirmed(self) -> None:
        if self._action is None:
            return
        action, self._action = self._action, None
        value = self.value_slider.value() if self._has_slider else 0.0
        self.slide.reset()
        self.hide()
        self.confirmed.emit(action, value)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.cancel()
            return
        super().keyPressEvent(event)
