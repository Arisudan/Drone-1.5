"""Compact servo button (ui/actuator_widget.py): state text and dot colour.

No ESP32 involved - _render() is driven directly from the panel's own state.
"""

import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first


class ActuatorButtonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        from ui.actuator_widget import ActuatorPanel
        self.p = ActuatorPanel("192.0.2.1")        # TEST-NET: never answers
        self.p._poll.stop()

    def tearDown(self):
        self.p._poll.stop()

    def _set(self, online, angle=-1, busy=False):
        self.p._online, self.p._angle, self.p._busy = online, angle, busy
        self.p._render()

    def test_offline_text_and_red_dot(self):
        self._set(False)
        self.assertEqual(self.p.btn_toggle.text(), "SERVO OFFLINE")
        self.assertEqual(self.p.dot.colour(), "#f85149")
        self.assertIn("192.0.2.1", self.p.btn_toggle.toolTip())

    def test_idle_shows_position_and_next_angle(self):
        self._set(True, angle=0)
        self.assertEqual(self.p.btn_toggle.text(), "SERVO 0° → 90°")
        self._set(True, angle=90)
        self.assertEqual(self.p.btn_toggle.text(), "SERVO 90° → 0°")
        self.assertEqual(self.p.dot.colour(), "#6e7681")

    def test_unknown_position_first_press_goes_to_zero(self):
        self._set(True, angle=-1)
        self.assertEqual(self.p.btn_toggle.text(), "SERVO -- → 0°")

    def test_moving_is_amber(self):
        self._set(True, angle=0, busy=True)
        self.assertEqual(self.p.dot.colour(), "#d29922")

    def test_widest_label_fits_the_button(self):
        self.p.show()
        self._set(True, angle=90)
        self.app.processEvents()
        need = self.p.btn_toggle.fontMetrics().horizontalAdvance("SERVO 90° → 0°")
        self.assertLessEqual(need, self.p.btn_toggle.minimumWidth())


if __name__ == "__main__":
    unittest.main()
