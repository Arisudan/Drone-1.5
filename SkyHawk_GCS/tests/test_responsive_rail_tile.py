"""v2: the rail is a fixed 50px icon column at every window size and UI scale
(the old width / thumbnail / checklist folding logic no longer exists)."""
import _env  # noqa: F401  -- must be first

import unittest

from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])


class RailWidthTest(unittest.TestCase):
    def test_rail_width_is_fixed_whatever_the_window_does(self):
        from ui.sidebar_nav import SidebarNav
        from ui.scaling import px
        r = SidebarNav()
        w0 = r.width() or r.minimumWidth()
        for wide in (True, False, True):
            r.set_wide(wide)
            self.assertEqual(r.maximumWidth(), px(SidebarNav.WIDE_PX))
            self.assertEqual(r.minimumWidth(), px(SidebarNav.WIDE_PX))
        self.assertTrue(w0 >= 0)


if __name__ == "__main__":
    unittest.main()
