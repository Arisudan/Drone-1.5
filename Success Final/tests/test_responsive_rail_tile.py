"""The wide rail and the full camera tile give way when the window cannot spare them - nothing is squeezed."""
import _env  # noqa: F401  -- must be first

import os
import unittest

from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])


class ResponsiveTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_responsive_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        cls.dg = drone_gcs
        cls.cfg = apply_overrides(load_settings())

    def window(self, scale, size, page=2):
        from ui.scaling import set_scale
        from ui.styles import build_stylesheet
        set_scale(scale)
        app.setStyleSheet(build_stylesheet())
        w = self.dg.DroneGCSMainWindow(settings=self.cfg)
        w.resize(*size)
        w.show()
        w._switch_workspace(page)
        for _ in range(10):
            app.processEvents()
        return w

    def tearDown(self):
        from ui.scaling import set_scale
        from ui.styles import build_stylesheet
        set_scale(1.0)
        app.setStyleSheet(build_stylesheet())

    def done(self, w):
        w.shutdown_workers()
        w.close()

    def test_wide_rail_on_a_roomy_window_narrow_rail_on_a_small_one(self):
        from ui.scaling import px
        w = self.window(1.0, (1500, 950))
        self.assertTrue(w.sidebar.is_wide())
        self.assertEqual(w.sidebar.width(), px(w.sidebar.WIDE_PX))
        w.resize(1220, 700)
        for _ in range(6):
            app.processEvents()
        self.assertFalse(w.sidebar.is_wide())
        self.assertEqual(w.sidebar.width(), px(w.sidebar.NARROW_PX))
        w.resize(1500, 950)
        for _ in range(6):
            app.processEvents()
        self.assertTrue(w.sidebar.is_wide())
        self.done(w)

    def test_a_large_ui_scale_always_gets_the_narrow_rail(self):
        from ui.scaling import px
        w = self.window(1.35, (1600, 950))
        self.assertFalse(w.sidebar.is_wide())
        self.assertEqual(w.sidebar.width(), px(w.sidebar.NARROW_PX))
        self.done(w)

    def test_a_short_side_panel_folds_the_camera_to_its_header_and_nothing_sticks_out(self):
        w = self.window(1.35, (1220, 700))
        sp = w.page_slam
        feed, panel = sp.docked_feed, sp.side_panel
        self.assertTrue(feed.is_compact())
        self.assertFalse(feed.sink.isVisible())
        for b in (feed.btn_collapse, feed.btn_popout, feed.btn_full):       # the controls stay reachable
            self.assertTrue(b.isVisible())
        ex = sp.btn_execute_path
        self.assertLessEqual(ex.mapTo(panel, ex.rect().bottomLeft()).y(), panel.height())
        self.done(w)

    def test_the_full_picture_comes_back_when_there_is_room(self):
        w = self.window(1.35, (1220, 700))
        self.assertTrue(w.page_slam.docked_feed.is_compact())
        w.resize(1600, 1000)
        for _ in range(8):
            app.processEvents()
        feed = w.page_slam.docked_feed
        self.assertFalse(feed.is_compact())
        self.assertTrue(feed.sink.isVisible())
        self.assertEqual((feed.width(), feed.height()), (328, 220))
        self.done(w)

    def test_a_roomy_window_always_shows_the_full_tile(self):
        w = self.window(1.0, (1500, 950))
        self.assertFalse(w.page_slam.docked_feed.is_compact())
        self.assertTrue(w.page_slam.docked_feed.sink.isVisible())
        self.done(w)


class ArrowSizeTest(unittest.TestCase):
    def test_the_fold_arrow_keeps_its_size_at_every_scale(self):
        from ui.docked_feed import Chevron
        from ui.scaling import px, set_scale
        from ui.styles import build_stylesheet
        try:
            for scale in (1.0, 1.15, 1.35):
                set_scale(scale)
                app.setStyleSheet(build_stylesheet())
                c = Chevron()
                c.show()
                app.processEvents()
                self.assertEqual((c.width(), c.height()), (px(22), px(22)), scale)
                c.close()
        finally:
            set_scale(1.0)
            app.setStyleSheet(build_stylesheet())


if __name__ == "__main__":
    unittest.main()
