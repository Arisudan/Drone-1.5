"""Left rail: slimmer nav rows, with real air around the Preflight checklist and above the camera thumbnail."""
import _env  # noqa: F401  -- must be first

import os
import unittest

from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])


class SidebarSpacingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_sidebar_spacing_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        cls.dg = drone_gcs
        cls.cfg = apply_overrides(load_settings())

    def window(self, scale=1.0, size=(1500, 950)):
        from ui.scaling import set_scale
        from ui.styles import build_stylesheet
        set_scale(scale)
        app.setStyleSheet(build_stylesheet())
        w = self.dg.DroneGCSMainWindow(settings=self.cfg)
        w.resize(*size)
        w.show()
        w._switch_workspace(4)          # Diagnostics: a workspace that shows the thumbnail
        for _ in range(8):
            app.processEvents()
        return w

    def tearDown(self):
        from ui.scaling import set_scale
        from ui.styles import build_stylesheet
        set_scale(1.0)
        app.setStyleSheet(build_stylesheet())

    def test_nav_rows_are_slimmer_but_still_comfortable_to_click(self):
        w = self.window()
        for b in w.sidebar.btn_group.buttons():
            self.assertLessEqual(b.height(), 31, b.text())
            self.assertGreaterEqual(b.height(), 26, b.text())
        w.shutdown_workers()
        w.close()

    def test_air_between_nav_checklist_and_camera_thumbnail(self):
        for scale in (1.0, 1.35):
            w = self.window(scale)
            sb = w.sidebar
            sb.checklist.toggle_expanded()          # worst case: the long, open list
            for _ in range(6):
                app.processEvents()
            last = sb.btn_group.buttons()[-1]
            self.assertGreaterEqual(sb.checklist.y() - (last.y() + last.height()), 8, scale)
            if sb.mini_feed.isVisible():
                gap = sb.mini_feed.y() - (sb.checklist.y() + sb.checklist.height())
                self.assertGreaterEqual(gap, 10, scale)
            w.shutdown_workers()
            w.close()

    def test_no_nav_text_is_clipped(self):
        for scale in (1.0, 1.35):
            w = self.window(scale)
            for b in w.sidebar.btn_group.buttons():
                need = b.fontMetrics().horizontalAdvance(b.text())
                self.assertGreaterEqual(b.width(), need, b.text())
                self.assertGreaterEqual(b.height(), b.fontMetrics().height(), b.text())
            w.shutdown_workers()
            w.close()


if __name__ == "__main__":
    unittest.main()
