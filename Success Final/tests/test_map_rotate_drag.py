"""Alt + left-drag rotates the 2D map about its centre; a plain left click still stages a goal."""
import _env  # noqa: F401  -- must be first

import math
import unittest

from PyQt5.QtCore import QPoint, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])

from ui.slam_map_widget import SLAMMapWidget  # noqa: E402


class RotateDragTest(unittest.TestCase):
    def setUp(self):
        self.w = SLAMMapWidget()
        self.w.resize(1400, 800)
        self.w.show()
        app.processEvents()
        self.c = self.w.canvas
        self.cx, self.cy = self.c.width() // 2, self.c.height() // 2
        self.goals = []
        self.c.goal_staged.connect(lambda *a: self.goals.append(a))

    def tearDown(self):
        self.w.close()

    def P(self, dx, dy):
        return QPoint(self.cx + dx, self.cy + dy)

    def drag(self, a, b, mods=Qt.AltModifier, release=True):
        QTest.mousePress(self.c, Qt.LeftButton, mods, a)
        # mouseMoveEvent only fires for real moves with the button held
        from PyQt5.QtCore import QEvent
        from PyQt5.QtGui import QMouseEvent
        ev = QMouseEvent(QEvent.MouseMove, b, Qt.NoButton, Qt.LeftButton, mods)
        QApplication.sendEvent(self.c, ev)
        if release:
            QTest.mouseRelease(self.c, Qt.LeftButton, mods, b)

    def test_clockwise_quarter_turn_matches_turn_right(self):
        self.drag(self.P(150, 0), self.P(0, 150))           # right of centre -> below centre: clockwise 90
        self.assertAlmostEqual(self.c.rotation_deg, 90.0, delta=0.5)
        self.c.reset_rotation()
        self.c.turn_right()
        self.assertAlmostEqual(self.c.rotation_deg, 90.0)

    def test_counter_clockwise_drag_turns_the_other_way(self):
        self.drag(self.P(150, 0), self.P(0, -150))
        self.assertAlmostEqual(self.c.rotation_deg, 270.0, delta=0.5)

    def test_rotation_is_relative_to_where_it_already_was(self):
        self.c.turn_right()                                  # 90
        self.drag(self.P(150, 0), self.P(0, 150))            # +90
        self.assertAlmostEqual(self.c.rotation_deg, 180.0, delta=0.5)

    def test_alt_drag_never_stages_a_goal(self):
        self.drag(self.P(150, 0), self.P(0, 150))
        self.assertEqual(self.goals, [])
        self.assertEqual(self.c.mission_stops, [])

    def test_plain_left_click_still_stages_a_goal(self):
        QTest.mouseClick(self.c, Qt.LeftButton, Qt.NoModifier, self.P(60, -60))
        self.assertEqual(len(self.c.mission_stops), 1)
        self.assertAlmostEqual(self.c.rotation_deg, 0.0)

    def test_shift_snaps_to_15_degrees(self):
        self.drag(self.P(150, 0), self.P(150, 40), mods=Qt.AltModifier | Qt.ShiftModifier)
        self.assertEqual(self.c.rotation_deg % 15.0, 0.0)

    def test_release_ends_the_drag(self):
        self.drag(self.P(150, 0), self.P(0, 150))
        done = self.c.rotation_deg
        from PyQt5.QtCore import QEvent
        from PyQt5.QtGui import QMouseEvent
        QApplication.sendEvent(self.c, QMouseEvent(QEvent.MouseMove, self.P(-150, 0), Qt.NoButton,
                                                   Qt.NoButton, Qt.AltModifier))
        self.assertEqual(self.c.rotation_deg, done)

    def test_signal_and_clicks_stay_correct_when_rotated(self):
        seen = []
        self.c.rotation_changed.connect(seen.append)
        self.drag(self.P(150, 0), self.P(0, 150))
        self.assertTrue(seen)
        # a point straight "up" on screen must map to the same world point the 90-degree button gives
        x1, y1 = self.c._screen_to_world(self.cx, self.cy - 100, self.cx, self.cy)
        self.c.reset_rotation()
        self.c.turn_right()
        x2, y2 = self.c._screen_to_world(self.cx, self.cy - 100, self.cx, self.cy)
        self.assertAlmostEqual(x1, x2, delta=0.02)
        self.assertAlmostEqual(y1, y2, delta=0.02)

    def test_help_text_mentions_the_gesture(self):
        self.assertIn("Alt", self.w.lbl_help.toolTip())


if __name__ == "__main__":
    unittest.main()
