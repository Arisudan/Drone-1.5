"""The SLAM camera feed: docked in the side panel at the floating window's size; pop-out floats it."""

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

    def go_slam(self):
        self.win._set_fpv_popped_out(False)
        self.win._switch_workspace(self.slam_index())
        self.settle()
        return self.win

    def test_docked_in_the_side_panel_by_default_not_floating(self):
        w = self.go_slam()
        feed = w.page_slam.docked_feed
        self.assertTrue(feed.isVisible())
        self.assertFalse(w.fpv_float.isVisible())
        self.assertTrue(w.page_slam.side_panel.isAncestorOf(feed))

    def test_docked_tile_keeps_the_floating_windows_size(self):
        from ui.docked_feed import FEED_H, FEED_W
        from ui.scaling import px
        w = self.go_slam()
        feed = w.page_slam.docked_feed
        self.assertEqual((feed.width(), feed.height()), (FEED_W, FEED_H))
        self.assertGreaterEqual(w.fpv_float.width(), 328)
        self.assertEqual((FEED_W, FEED_H), (328, 220))

    def test_it_no_longer_covers_the_map(self):
        w = self.go_slam()
        canvas, feed = w.page_slam.canvas, w.page_slam.docked_feed
        c = canvas.mapToGlobal(canvas.rect().topLeft())
        f = feed.mapToGlobal(feed.rect().topLeft())
        self.assertGreaterEqual(f.x(), c.x() + canvas.width() - 2, "tile must be right of the map, not on it")

    def test_frames_reach_the_docked_sink(self):
        import numpy as np
        w = self.go_slam()
        w.page_fpv.frame_broadcast.emit(np.zeros((90, 160, 3), dtype=np.uint8))
        self.settle()
        self.assertFalse(w.page_slam.docked_feed.sink.pixmap() is None or w.page_slam.docked_feed.sink.pixmap().isNull())

    def test_pop_out_floats_it_and_closing_docks_it_again(self):
        w = self.go_slam()
        w.page_slam.docked_feed.btn_popout.click()
        self.settle()
        self.assertTrue(w.fpv_float.isVisible())
        feed = w.page_slam.docked_feed
        self.assertFalse(feed.sink.isVisible())                  # the picture left the panel...
        self.assertTrue(feed.isVisible() and feed.btn_collapse.isVisible())   # ...but the header strip stays
        canvas = w.page_slam.canvas
        self.assertGreaterEqual(w.fpv_float.pos().x(), canvas.mapToGlobal(canvas.rect().topLeft()).x())
        w.fpv_float._on_close_clicked()
        self.settle()
        self.assertFalse(w.fpv_float.isVisible())
        self.assertTrue(w.page_slam.docked_feed.sink.isVisible())

    def test_floating_window_hidden_off_the_slam_tab(self):
        w = self.go_slam()
        w.page_slam.docked_feed.btn_popout.click()
        w._switch_workspace(0)
        self.settle()
        self.assertFalse(w.fpv_float.isVisible())
        w._switch_workspace(self.slam_index())
        self.settle()
        self.assertTrue(w.fpv_float.isVisible())
        w._set_fpv_popped_out(False)

    def test_collapsing_the_panel_folds_the_picture_away_but_keeps_the_arrow(self):
        w = self.go_slam()
        feed = w.page_slam.docked_feed
        w.page_slam.set_panel_collapsed(True)
        self.settle()
        self.assertFalse(feed.sink.isVisible())
        self.assertTrue(feed.btn_collapse.isVisible())            # the way back is always there
        self.assertLessEqual(feed.width(), w.page_slam.side_panel.width())
        w.page_slam.set_panel_collapsed(False)
        self.settle()
        self.assertTrue(feed.sink.isVisible())

    def test_dock_back_button_returns_a_popped_out_camera(self):
        w = self.go_slam()
        feed = w.page_slam.docked_feed
        feed.btn_popout.click()
        self.settle()
        self.assertTrue(feed.btn_dock.isVisible())
        self.assertTrue(w.fpv_float.isVisible())
        feed.btn_dock.click()
        self.settle()
        self.assertFalse(w.fpv_float.isVisible())
        self.assertTrue(feed.sink.isVisible())
        self.assertFalse(feed.btn_dock.isVisible())

    def test_fullscreen_button_opens_the_shared_fullscreen_window(self):
        w = self.go_slam()
        w.fpv_fullscreen.hide()
        w.page_slam.docked_feed.btn_full.click()
        self.settle()
        self.assertTrue(w.fpv_fullscreen.isVisible())
        w.fpv_fullscreen.hide()


if __name__ == "__main__":
    unittest.main()
