"""Status dots are drawn as true circles - never squares - at any size and scale."""
import _env  # noqa: F401  -- must be first

import os
import unittest

from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QApplication, QWidget

app = QApplication.instance() or QApplication([])

from ui.status_dot import StatusDot  # noqa: E402
from ui.scaling import set_scale  # noqa: E402
from ui.styles import build_stylesheet  # noqa: E402

BACKDROP = "#161b22"


def probe(dot_size, colour="#f85149"):
    host = QWidget()
    host.setStyleSheet(f"background: {BACKDROP};")
    host.resize(60, 40)
    d = StatusDot(dot_size, colour, host)
    d.move(20, 10)
    host.show()
    for _ in range(3):
        app.processEvents()
    img = host.grab().toImage()
    w, h = d.width(), d.height()
    corner = QColor(img.pixel(20, 10)).name()
    centre = QColor(img.pixel(20 + w // 2, 10 + h // 2)).name()
    host.close()
    return corner, centre


class StatusDotTest(unittest.TestCase):
    def tearDown(self):
        set_scale(1.0)
        app.setStyleSheet(build_stylesheet())

    def test_round_at_every_size_and_scale(self):
        for scale in (1.0, 1.15, 1.35):
            set_scale(scale)
            for size in (8, 9, 10):
                corner, centre = probe(size)
                self.assertEqual(centre, "#f85149", (scale, size))
                self.assertEqual(corner, BACKDROP, f"the corner must be empty - a square would fill it ({scale}, {size})")

    def test_colour_is_one_plain_value(self):
        d = StatusDot(10, "#6e7681")
        self.assertEqual(d.colour(), "#6e7681")
        d.set_colour("#3fb950")
        self.assertEqual(d.colour(), "#3fb950")

    def test_every_status_dot_in_the_app_is_a_StatusDot(self):
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_status_dot_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        w = drone_gcs.DroneGCSMainWindow(settings=apply_overrides(load_settings()))
        from ui.actuator_widget import ActuatorPanel
        from ui.motor_widget import _Check
        for a in w.findChildren(ActuatorPanel):
            self.assertIsInstance(a.dot, StatusDot)
        for c in w.findChildren(_Check):
            self.assertIsInstance(c.dot, StatusDot)
        self.assertIsInstance(w.sidebar.mini_feed.dot, StatusDot)
        w.shutdown_workers()
        w.close()


if __name__ == "__main__":
    unittest.main()
