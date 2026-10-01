"""Rail camera thumbnail (ui/mini_feed.py) and its wiring into the sidebar.

Time is injected; frames are synthetic ndarrays. No camera, no network.
"""

import unittest

import numpy as np

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first


def _frame():
    return np.full((90, 160, 3), 128, dtype=np.uint8)


class MiniFeedTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _make(self):
        from PyQt5.QtWidgets import QWidget
        from ui.mini_feed import MiniFeed
        t = [0.0]
        host = QWidget()
        host.resize(176, 300)
        m = MiniFeed(host, clock=lambda: t[0])
        host.show()
        self.app.processEvents()
        return host, m, t

    def test_hidden_thumbnail_does_no_work(self):
        host, m, t = self._make()
        m.set_health("LIVE", "")
        m.hide()
        t[0] = 1.0
        m.on_frame(_frame())
        pm = m.sink.pixmap()
        self.assertTrue(pm is None or pm.isNull())

    def test_live_frames_paint_and_are_thinned_to_about_10fps(self):
        host, m, t = self._make()
        m.set_health("LIVE", "")
        t[0] = 1.0
        m.on_frame(_frame())
        self.assertFalse(m.sink.pixmap().isNull())
        first = m._last_paint
        t[0] = 1.03
        m.on_frame(_frame())
        self.assertEqual(m._last_paint, first)         # too soon - dropped
        t[0] = 1.2
        m.on_frame(_frame())
        self.assertEqual(m._last_paint, 1.2)

    def test_frozen_clears_stale_image_and_ignores_frames(self):
        host, m, t = self._make()
        m.set_health("LIVE", "")
        t[0] = 1.0
        m.on_frame(_frame())
        m.set_health("FROZEN", "")
        pm = m.sink.pixmap()
        self.assertTrue(pm is None or pm.isNull())
        self.assertEqual(m.sink.text(), "NO VIDEO")
        self.assertEqual(m.lbl_state.text(), "FROZEN")
        self.assertIn("#f85149", m.dot.styleSheet())
        t[0] = 5.0
        m.on_frame(_frame())
        pm = m.sink.pixmap()
        self.assertTrue(pm is None or pm.isNull())     # a dead feed never paints

    def test_click_requests_fullscreen(self):
        from PyQt5.QtCore import Qt
        from PyQt5.QtTest import QTest
        host, m, t = self._make()
        hits = []
        m.fullscreen_requested.connect(lambda: hits.append(1))
        QTest.mouseClick(m.sink, Qt.LeftButton)
        self.assertEqual(hits, [1])

    def test_picture_is_16_9(self):
        host, m, t = self._make()
        self.assertAlmostEqual(m.sink.height() / m.sink.width(), 9 / 16, delta=0.03)


class SidebarMiniFeedTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def _rail(self, h):
        from ui.sidebar_nav import SidebarNav
        r = SidebarNav()
        r.resize(176, h)
        r.show()
        self.app.processEvents()
        return r

    def test_shown_only_when_wanted_and_tall_enough(self):
        tall = self._rail(900)
        tall.set_mini_feed_wanted(False)
        self.assertFalse(tall.mini_feed.isVisible())
        tall.set_mini_feed_wanted(True)
        self.assertTrue(tall.mini_feed.isVisible())
        short = self._rail(420)
        short.set_mini_feed_wanted(True)
        self.assertFalse(short.mini_feed.isVisible())   # no room: never squeeze the nav

    def test_resize_reevaluates(self):
        r = self._rail(420)
        r.set_mini_feed_wanted(True)
        self.assertFalse(r.mini_feed.isVisible())
        r.resize(176, 900)
        self.app.processEvents()
        self.assertTrue(r.mini_feed.isVisible())

    def test_camera_less_tabs_are_exactly_the_six(self):
        from ui.sidebar_nav import SidebarNav
        self.assertEqual(SidebarNav.MINI_FEED_TABS, frozenset({3, 4, 5, 6, 7, 8}))


if __name__ == "__main__":
    unittest.main()
