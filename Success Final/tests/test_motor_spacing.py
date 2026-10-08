"""Bench Motor Test: proper spacing between the blocks (spacing only - nothing else about the panel changed)."""
import _env  # noqa: F401  -- must be first

import unittest

from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])

from ui.motor_widget import MotorTestPanel  # noqa: E402
from ui.styles import build_stylesheet  # noqa: E402


def top(w, root):
    return w.mapTo(root, w.rect().topLeft()).y()


def bottom(w, root):
    return top(w, root) + w.height()


class MotorSpacingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        app.setStyleSheet(build_stylesheet())

    def panel(self, compact=False):
        p = MotorTestPanel()
        p.resize(520, 760)
        p.set_compact(compact)
        p.show()
        for _ in range(5):
            app.processEvents()
        return p

    def test_blocks_are_spaced_apart(self):
        p = self.panel()
        self.assertGreaterEqual(top(p.chk_safety, p) - bottom(p.chk_ground, p), 12)           # interlocks -> bar
        self.assertGreaterEqual(top(p.motor_buttons[3], p) - bottom(p.chk_safety, p), 12)     # bar -> motors
        throttle_top = min(top(p.lbl_throttle, p), top(p.slider_throttle, p))
        self.assertGreaterEqual(throttle_top - bottom(p.motor_buttons[2], p), 12)            # motors -> throttle
        self.assertGreaterEqual(top(p.btn_spin, p) - bottom(p.slider_throttle, p), 12)        # throttle -> actions
        p.close()

    def test_interlock_rows_and_motor_buttons_have_room_between_them(self):
        p = self.panel()
        self.assertGreaterEqual(top(p.chk_ground, p) - bottom(p.chk_link, p), 6)
        self.assertGreaterEqual(top(p.motor_buttons[2], p) - bottom(p.motor_buttons[3], p), 6)
        self.assertGreaterEqual(top(p.btn_sequence, p) - bottom(p.btn_spin, p), 6)
        p.close()

    def test_throttle_label_readout_and_slider_stay_together(self):
        p = self.panel()
        self.assertLessEqual(top(p.slider_throttle, p) - bottom(p.lbl_throttle, p), 10)
        self.assertGreaterEqual(top(p.slider_throttle, p) - bottom(p.lbl_throttle, p), 2)
        p.close()

    def test_buttons_are_taller(self):
        p = self.panel()
        for b in p.motor_buttons.values():
            self.assertGreaterEqual(b.height(), 36)
        self.assertGreaterEqual(p.btn_spin.height(), 38)
        self.assertGreaterEqual(p.btn_sequence.height(), 34)
        self.assertGreaterEqual(p.btn_stop.height(), 34)
        self.assertGreaterEqual(p.btn_thr_minus.height(), 34)
        p.close()

    def test_a_short_page_gets_the_original_tight_numbers_back(self):
        roomy, tight = self.panel(False), self.panel(True)
        self.assertLess(tight.layout().spacing(), roomy.layout().spacing())
        self.assertLess(tight.btn_spin.height(), roomy.btn_spin.height())
        self.assertLessEqual(tight.motor_buttons[1].height(), 34)       # what it always was
        roomy.close()
        tight.close()

    def test_nothing_overlaps_or_clips_at_the_roomy_size(self):
        p = self.panel()
        order = [p.chk_link, p.chk_safety, p.motor_buttons[3], p.motor_buttons[2], p.slider_throttle,
                 p.btn_spin, p.btn_sequence]
        for a, b in zip(order, order[1:]):
            self.assertLessEqual(bottom(a, p), top(b, p), (a, b))
        self.assertLessEqual(bottom(p.btn_stop, p), p.height())
        p.close()


if __name__ == "__main__":
    unittest.main()
