"""
================================================================================
MODULE: alarm_banner.py
PURPOSE: Standing Alarm Card (top priority alarm, running timer, ACK, full list)
================================================================================

A slim card under the header that is HIDDEN while nothing is wrong, so it costs
no cockpit space in normal flight. When alarms exist it shows the highest
priority one: a severity bar and badge, a bold title, a grey hint line, how long
the alarm has been active (a running m:ss), a "+N" pill for the others, and ACK.

THE TIMER RUNS ON ITS OWN:
  The elapsed time is computed from the alarm's ``raised_at`` every second by a
  small QTimer that only runs while the card is visible. It is deliberately NOT
  part of the alarm text: text written once at raise time goes stale, which is
  exactly the bug this replaced ("no frame for 2.4 s" frozen at 2.4 s).

Colour carries real status only: red = unacknowledged CRITICAL, amber =
unacknowledged WARN. Once acknowledged the card drops to neutral grey and shows
an ACKNOWLEDGED tag - seen, but still listed until the condition truly clears.

It never owns alarm state: it renders an AlarmManager and asks the window to
acknowledge through signals.
================================================================================
"""

from __future__ import annotations

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QMenu, QPushButton, QVBoxLayout

from core.alarms import AlarmLevel, AlarmManager, format_elapsed
from ui.scaling import px

_CRIT = "#da3633"
_WARN = "#d29922"
_NEUTRAL = "#30363d"
_MUTED = "#8b949e"
_TEXT = "#e6edf3"


class AlarmBanner(QFrame):
    ack_all_requested = pyqtSignal()
    ack_requested = pyqtSignal(str)   # alarm key

    def __init__(self, alarms: AlarmManager, parent=None):
        super().__init__(parent)
        self.alarms = alarms
        self._seen_revision = -1
        self.setObjectName("alarmBanner")

        row = QHBoxLayout(self)
        row.setContentsMargins(px(12), px(6), px(10), px(6))
        row.setSpacing(px(10))

        # Severity badge: a round "!" - colour is the only thing that varies.
        self.lbl_badge = QLabel("!", self)
        self.lbl_badge.setAlignment(Qt.AlignCenter)
        self.lbl_badge.setFixedSize(px(22), px(22))
        row.addWidget(self.lbl_badge, 0, Qt.AlignVCenter)

        col = QVBoxLayout()
        col.setSpacing(px(1))
        head = QHBoxLayout()
        head.setSpacing(px(8))
        self.lbl_level = QLabel("", self)     # WARN / CRITICAL
        self.lbl_text = QLabel("", self)      # title
        self.lbl_tag = QLabel("ACKNOWLEDGED", self)
        head.addWidget(self.lbl_level)
        head.addWidget(self.lbl_text)
        head.addWidget(self.lbl_tag)
        head.addStretch(1)
        col.addLayout(head)
        self.lbl_detail = QLabel("", self)    # hint line
        col.addWidget(self.lbl_detail)
        row.addLayout(col, 1)

        self.lbl_more = QLabel("", self)      # "+2" pill
        row.addWidget(self.lbl_more, 0, Qt.AlignVCenter)
        self.lbl_timer = QLabel("0:00", self)
        self.lbl_timer.setToolTip("How long this alarm has been active")
        row.addWidget(self.lbl_timer, 0, Qt.AlignVCenter)

        self.btn_list = QPushButton("All alarms", self)
        self.btn_list.setToolTip("Show every active alarm")
        self.btn_list.setFlat(True)
        self.btn_list.clicked.connect(self._show_list)
        row.addWidget(self.btn_list, 0, Qt.AlignVCenter)
        self.btn_ack = QPushButton("Acknowledge", self)
        self.btn_ack.setToolTip("Acknowledge alarms (they stay listed until the condition clears)")
        self.btn_ack.clicked.connect(self.ack_all_requested)
        row.addWidget(self.btn_ack, 0, Qt.AlignVCenter)

        # Runs only while the card is showing; see module docstring.
        self._clock_timer = QTimer(self)
        self._clock_timer.setInterval(1000)
        self._clock_timer.timeout.connect(self.refresh_timer)

        self.setVisible(False)
        self.refresh()

    # ── rendering ──────────────────────────────────────────────────────────
    def refresh(self) -> None:
        """Cheap to call every UI tick: it redraws only when the list changed."""
        if self.alarms.revision == self._seen_revision:
            return
        self._seen_revision = self.alarms.revision
        top = self.alarms.top()
        if top is None:
            self._clock_timer.stop()
            self.setVisible(False)
            return

        acked = top.acked
        colour = _NEUTRAL if acked else (_CRIT if top.level >= AlarmLevel.CRITICAL else _WARN)
        accent = _MUTED if acked else colour
        self.setStyleSheet(
            f"QFrame#alarmBanner {{ background: #0d1117; border: 1px solid {_NEUTRAL}; "
            f"border-left: {px(4)}px solid {colour}; border-radius: {px(5)}px; }}"
            f"QLabel {{ background: transparent; border: none; font-size: {px(12)}px; }}"
            f"QPushButton {{ border: none; background: transparent; color: {_MUTED}; "
            f"font-size: {px(12)}px; padding: {px(4)}px {px(8)}px; }}"
            f"QPushButton:hover {{ color: {_TEXT}; }}"
            f"QPushButton:disabled {{ color: #484f58; }}")

        self.lbl_badge.setStyleSheet(
            f"background: {accent}; color: #0d1117; border-radius: {px(11)}px; "
            f"font-weight: 800; font-size: {px(13)}px;")
        self.lbl_level.setText(top.level.name)
        self.lbl_level.setStyleSheet(
            f"color: {accent}; font-weight: 700; letter-spacing: 1px; font-size: {px(10)}px;")
        self.lbl_text.setText(top.text)
        self.lbl_text.setStyleSheet(f"color: {_MUTED if acked else _TEXT}; font-weight: 600;")
        self.lbl_tag.setVisible(acked)
        self.lbl_tag.setStyleSheet(
            f"color: {_MUTED}; border: 1px solid {_NEUTRAL}; border-radius: {px(3)}px; "
            f"padding: 0 {px(5)}px; font-size: {px(9)}px; letter-spacing: 1px;")
        self.lbl_detail.setText(top.detail)
        self.lbl_detail.setVisible(bool(top.detail))
        self.lbl_detail.setStyleSheet(f"color: {_MUTED}; font-size: {px(11)}px;")

        extra = len(self.alarms) - 1
        self.lbl_more.setVisible(extra > 0)
        self.lbl_more.setText(f"+{extra}")
        self.lbl_more.setStyleSheet(
            f"color: {_TEXT}; background: {_NEUTRAL}; border-radius: {px(9)}px; "
            f"padding: {px(1)}px {px(7)}px; font-weight: 600;")
        self.lbl_timer.setStyleSheet(
            f"color: {_MUTED}; font-family: monospace; font-size: {px(12)}px;")

        n_unacked = len(self.alarms.unacked())
        self.btn_ack.setEnabled(n_unacked > 0)
        self.btn_ack.setText("Acknowledge" if n_unacked <= 1 else f"Acknowledge all ({n_unacked})")
        self.btn_list.setVisible(extra > 0)

        self.setVisible(True)
        self.refresh_timer()
        if not self._clock_timer.isActive():
            self._clock_timer.start()

    def refresh_timer(self) -> None:
        """Re-read the running time for the shown alarm. Called every second."""
        top = self.alarms.top()
        if top is not None:
            self.lbl_timer.setText(format_elapsed(self.alarms.elapsed(top)))

    def _show_list(self) -> None:
        menu = QMenu(self)
        for a in self.alarms.active():
            state = "  (acknowledged)" if a.acked else ""
            act = menu.addAction(
                f"{a.level.name}   {a.text}   ·   {format_elapsed(self.alarms.elapsed(a))}{state}")
            act.setData(a.key)
            act.setEnabled(not a.acked)
        chosen = menu.exec_(self.btn_list.mapToGlobal(self.btn_list.rect().bottomLeft()))
        if chosen is not None and chosen.data():
            self.ack_requested.emit(chosen.data())
