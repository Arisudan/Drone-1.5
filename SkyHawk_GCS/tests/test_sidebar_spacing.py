"""v2 icon rail: nine icon-only workspace buttons, tooltips, preflight counter."""
import _env  # noqa: F401  -- must be first

import unittest

from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])


class IconRailTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from ui.sidebar_nav import SidebarNav
        cls.rail = SidebarNav()
        cls.rail.resize(60, 480)
        cls.rail.show()
        app.processEvents()

    def test_nine_icon_only_buttons_each_with_a_tooltip(self):
        btns = self.rail.btn_group.buttons()
        self.assertEqual(len(btns), 9)
        for b in btns:
            self.assertEqual(b.text(), "")
            self.assertTrue(b.toolTip())

    def test_buttons_are_comfortable_to_click_and_do_not_overlap(self):
        btns = sorted(self.rail.btn_group.buttons(), key=lambda b: b.y())
        for b in btns:
            self.assertGreaterEqual(b.height(), 34)
            self.assertGreaterEqual(b.width(), 34)
        for a, b in zip(btns, btns[1:]):
            self.assertLessEqual(a.geometry().bottom(), b.geometry().top())

    def test_selecting_a_tab_checks_it_and_emits(self):
        seen = []
        self.rail.view_changed.connect(seen.append)
        self.rail.select_tab(3)
        self.assertEqual(seen, [3])
        self.assertTrue(self.rail.btn_group.button(3).isChecked())
        self.rail.select_tab(0)

    def test_the_preflight_counter_follows_the_checklist(self):
        self.rail.set_preflight(5, 7, False)
        self.assertEqual(self.rail.pre.val.text(), "5/7")
        self.rail.set_preflight(7, 7, True)
        self.assertEqual(self.rail.pre.val.text(), "7/7")
        self.assertTrue(self.rail.pre.val.property("ready"))


if __name__ == "__main__":
    unittest.main()
