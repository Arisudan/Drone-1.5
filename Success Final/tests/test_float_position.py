"""The floating camera window opens over the SLAM map's top-left corner."""

import os
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from PyQt5.QtWidgets import QApplication


class FloatPositionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_float_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        from ui.styles import build_stylesheet
        cls.app.setStyleSheet(build_stylesheet())
        cls.win = drone_gcs.DroneGCSMainWindow(settings=apply_overrides(load_settings()))
        cls.win.resize(1400, 850)
        cls.win.show()
        for _ in range(6):
            cls.app.processEvents()

    @classmethod
    def tearDownClass(cls):
        cls.win.shutdown_workers()
        cls.win.close()

    def slam_index(self):
        return self.win.stack.indexOf(self.win.page_slam)

    def settle(self):
        for _ in range(6):
            self.app.processEvents()

    def test_opens_inside_the_top_left_of_the_map(self):
        w = self.win
        w._switch_workspace(self.slam_index())
        self.settle()
        canvas = w.page_slam.canvas
        origin = canvas.mapToGlobal(canvas.rect().topLeft())
        f = w.fpv_float.pos()
        self.assertTrue(w.fpv_float.isVisible())
        self.assertGreaterEqual(f.x(), origin.x())
        self.assertGreaterEqual(f.y(), origin.y())
        self.assertLess(f.x() - origin.x(), canvas.width() / 3)
        self.assertLess(f.y() - origin.y(), canvas.height() / 3)

    def test_it_is_not_in_the_bottom_right_any_more(self):
        w = self.win
        w._switch_workspace(self.slam_index())
        self.settle()
        canvas = w.page_slam.canvas
        centre = canvas.mapToGlobal(canvas.rect().center())
        self.assertLess(w.fpv_float.pos().x(), centre.x())
        self.assertLess(w.fpv_float.pos().y(), centre.y())

    def test_hidden_off_the_slam_tab_and_reparked_on_return(self):
        w = self.win
        w._switch_workspace(self.slam_index())
        w._switch_workspace(0)
        self.settle()
        self.assertFalse(w.fpv_float.isVisible())
        w.fpv_float.move(5, 5)
        w._switch_workspace(self.slam_index())
        self.settle()
        canvas = w.page_slam.canvas
        self.assertGreaterEqual(w.fpv_float.pos().x(), canvas.mapToGlobal(canvas.rect().topLeft()).x())


if __name__ == "__main__":
    unittest.main()
