"""Heading fonts: Red Hat Display for the DRONE-GCS title, Ubuntu for headings.

The point of these tests is the "silent fallback" trap: naming a font in a
stylesheet that the machine does not have shows another font and nothing
complains. So they check the bundled files exist, that Qt registers them, and
that real widgets in the assembled window RESOLVE to the intended family.
"""

import os
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from ui import fonts

HEADING_NAMES = ("cardHeading", "sectionTitle", "diagSectionHeader", "pageTitle")


class BundledFilesTest(unittest.TestCase):
    def test_every_font_file_and_both_licences_are_in_the_repo(self):
        base = fonts.fonts_dir()
        for name in fonts.BUNDLED:
            self.assertTrue((base / name).is_file(), f"missing {name}")
        self.assertTrue((base / "RedHatDisplay-OFL.txt").is_file())
        self.assertTrue((base / "Ubuntu-LICENCE.txt").is_file())

    def test_a_black_weight_exists_for_the_900_weight_title(self):
        self.assertIn("RedHatDisplay-Black.ttf", fonts.BUNDLED)


class FontRegistrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_both_families_register_with_qt(self):
        fams = fonts.load_bundled_fonts()
        self.assertIn(fonts.BRAND_FAMILY, fams)
        self.assertIn(fonts.HEADING_FAMILY, fams)

    def test_loading_is_idempotent(self):
        a = fonts.load_bundled_fonts()
        b = fonts.load_bundled_fonts()
        self.assertEqual(a, b)

    def test_qt_resolves_the_names_to_themselves_not_a_fallback(self):
        fonts.load_bundled_fonts()
        self.assertEqual(fonts.resolved_family("Red Hat Display"), "Red Hat Display")
        self.assertEqual(fonts.resolved_family("Ubuntu"), "Ubuntu")

    def test_build_stylesheet_registers_them_itself(self):
        from ui.styles import build_stylesheet
        qss = build_stylesheet()
        # v2: one UI chain (Lato -> Ubuntu Sans -> bundled Ubuntu); the bundled Ubuntu is
        # the guaranteed tail. Red Hat Display is still shipped and registered, but no
        # longer named by the stylesheet (the v2 design has no separate brand face).
        self.assertIn("'Ubuntu'", qss)


class StylesheetAssignmentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])
        from ui.styles import build_stylesheet
        cls.qss = build_stylesheet()

    def _block(self, selector):
        import re
        m = re.search(re.escape(selector) + r"\s*\{", self.qss)
        self.assertIsNotNone(m, f"selector {selector} not found")
        return self.qss[m.start():self.qss.index("}", m.start())]

    def test_title_uses_the_design_ui_font(self):
        self.assertIn("'Lato'", self._block("QLabel#appTitle"))

    def test_every_heading_selector_uses_ubuntu(self):
        # v2: headings use the shared UI chain, whose tail is the bundled Ubuntu.
        for sel in ("QGroupBox::title", "QLabel.cardTitle", "QLabel#cardHeading",
                    "QLabel#sectionTitle", "QLabel#diagSectionHeader",
                    "QLabel#pageTitle", "QLabel#rvizPlaceholderTitle"):
            block = self._block(sel)
            self.assertNotIn("Red Hat Display", block, f"{sel}: no separate brand face in v2")

    def test_body_and_number_fonts_are_untouched(self):
        self.assertIn("'Lato'", self._block("QWidget"))
        self.assertIn("Mono", self._block("QLabel#valueMono"))   # v2: Ubuntu Mono for numbers


class AssembledWindowTest(unittest.TestCase):
    """The real window: what each heading actually resolves to."""

    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_fonts_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        from ui.styles import build_stylesheet
        cls.app.setStyleSheet(build_stylesheet())
        cls.win = drone_gcs.DroneGCSMainWindow()
        cls.win.resize(1600, 950)
        cls.win.show()
        for _ in range(6):
            cls.app.processEvents()

    @classmethod
    def tearDownClass(cls):
        cls.win.shutdown_workers()
        cls.win.close()

    def _family(self, widget):
        from PyQt5.QtGui import QFontInfo
        widget.ensurePolished()
        return QFontInfo(widget.font()).family()

    UI_FAMILIES = ("Lato", "Noto Sans", "Ubuntu Sans", "Ubuntu")

    def test_the_top_bar_title_names_the_page_and_renders_in_the_ui_font(self):
        self.win._switch_workspace(0)
        self.assertEqual(self.win.top_strip.title_lbl.text(), "Control")
        self.assertIn(self._family(self.win.top_strip.title_lbl), self.UI_FAMILIES)

    def test_every_heading_label_in_every_tab_renders_in_ubuntu(self):
        from PyQt5.QtWidgets import QLabel
        seen, bad = 0, []
        for idx in range(self.win.stack.count()):
            self.win._switch_workspace(idx)
            for _ in range(3):
                self.app.processEvents()
            for lbl in self.win.stack.widget(idx).findChildren(QLabel):
                if lbl.objectName() in HEADING_NAMES:
                    seen += 1
                    fam = self._family(lbl)
                    if fam not in self.UI_FAMILIES:
                        bad.append(f"tab {idx}: {lbl.text()!r} -> {fam}")
        self.assertGreater(seen, 8, "expected headings across the tabs")
        self.assertEqual(bad, [])

    def test_the_control_page_panels_are_titled_glass_panels(self):
        from PyQt5.QtWidgets import QLabel
        titles = {l.text() for l in self.win.page_cockpit.findChildren(QLabel)
                  if l.objectName() == "glassTitle"}
        self.assertIn("NAVIGATION", titles)
        self.assertIn("GUIDED DISPLACEMENT", titles)

    def test_the_rail_is_icon_only_with_tooltips(self):
        for btn in self.win.sidebar.btn_group.buttons():
            self.assertEqual(btn.text(), "")
            self.assertTrue(btn.toolTip())

    def test_the_motors_page_is_named_in_the_top_bar(self):
        self.win._switch_workspace(3)
        self.assertEqual(self.win.top_strip.title_lbl.text(), "Motor Actuators")


if __name__ == "__main__":
    unittest.main()
