"""The command console is locked: text arriving never resizes the log or moves anything around it."""
import _env  # noqa: F401  -- must be first

import os
import unittest

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget

app = QApplication.instance() or QApplication([])


def snapshot(root):
    snap = {}
    for w in root.findChildren(QWidget):
        if w.isVisible():
            p = w.mapTo(root, w.rect().topLeft())
            snap[id(w)] = (w.__class__.__name__, w.objectName(), p.x(), p.y(), w.width(), w.height())
    return snap


class ConsoleLockedTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_console_lock_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        from ui.styles import build_stylesheet
        app.setStyleSheet(build_stylesheet())
        cls.win = drone_gcs.DroneGCSMainWindow(settings=apply_overrides(load_settings()))

    @classmethod
    def tearDownClass(cls):
        cls.win.shutdown_workers()
        cls.win.close()

    def open(self, w, h):
        self.win.resize(w, h)
        self.win.show()
        self.win._switch_workspace(0)
        for _ in range(8):
            app.processEvents()
        self.win.console.log_box.clear()
        app.processEvents()

    def spam(self, n=120):
        c = self.win.console
        for i in range(n):
            c.log_error(f"[HEALTH] STALLED - MapListener: stalled | -- Hz | 0 ms p95 | last beat {i}.7s ago " + "x" * (i % 70))
            c.log_info(f"line {i}")
            c.log_cmd(f"> move {i} 0 0")
        for _ in range(8):
            app.processEvents()

    def test_log_box_never_changes_size_when_text_arrives(self):
        for size in ((560, 1000), (1100, 760), (1500, 900)):
            self.open(*size)
            box = self.win.console.log_box
            before = (box.width(), box.height())
            self.spam()
            self.assertEqual((box.width(), box.height()), before, size)

    def test_nothing_around_the_console_moves(self):
        """The cmd row and the displacement readouts keep their exact geometry while the log fills."""
        for size in ((560, 1000), (1100, 800)):
            self.open(*size)
            c = self.win.console
            watched = [c.cmd_input, c.findChild(QWidget, "cliPrompt"), c.findChild(QWidget, "btnCliSend"),
                       c.findChild(QWidget, "btnCliClear")] + list(self.win.lbl_exec_axes.values())

            def geo():
                return [(w.mapTo(self.win, w.rect().topLeft()).x(), w.mapTo(self.win, w.rect().topLeft()).y(),
                         w.width(), w.height()) for w in watched]
            before = geo()
            self.spam()
            self.assertEqual(geo(), before, size)

    def test_wrap_and_scrollbars_are_back_to_qt_defaults(self):
        box = self.win.console.log_box
        self.assertEqual(box.lineWrapMode(), box.WidgetWidth)
        self.assertEqual(box.verticalScrollBarPolicy(), Qt.ScrollBarAsNeeded)
        self.assertEqual(box.horizontalScrollBarPolicy(), Qt.ScrollBarAsNeeded)

    def test_console_fills_spare_height_and_has_a_ten_line_floor(self):
        self.open(1100, 760)
        console = self.win.console
        short = console.log_box.height()
        floor = console.log_box.fontMetrics().lineSpacing() * 10
        self.assertGreaterEqual(short, floor)
        self.open(1100, 1300)
        tall = console.log_box.height()
        self.assertGreater(tall, short, "spare window height must go to the console, not an empty gap")
        # ...and the console reaches the bottom of its column: nothing but the cmd row below the log
        self.assertLess(console.height() - console.log_box.height(), 80)

    def test_cmd_row_keeps_its_size_while_the_log_fills(self):
        self.open(1100, 800)
        c = self.win.console
        before = (c.cmd_input.width(), c.cmd_input.height())
        self.spam(60)
        self.assertEqual((c.cmd_input.width(), c.cmd_input.height()), before)

    def test_exec_readouts_reserve_room_for_their_widest_value(self):
        self.open(1100, 800)
        for lbl in self.win.lbl_exec_axes.values():
            self.assertGreaterEqual(lbl.width(), lbl.fontMetrics().horizontalAdvance("-000.00"))
        w0 = [l.width() for l in self.win.lbl_exec_axes.values()]
        for lbl in self.win.lbl_exec_axes.values():
            lbl.setText("-123.45")
        app.processEvents()
        self.assertEqual([l.width() for l in self.win.lbl_exec_axes.values()], w0)

    def test_preflight_pill_keeps_one_size(self):
        self.open(1100, 800)
        pill = self.win.sidebar.findChild(QWidget, "railPill")
        if pill is None:
            self.skipTest("no preflight pill in this build")
        a = (pill.width(), pill.height())
        pill.setText("READY")
        pill.setText("10 TO FIX")
        app.processEvents()
        self.assertEqual(pill.height(), a[1])


if __name__ == "__main__":
    unittest.main()
