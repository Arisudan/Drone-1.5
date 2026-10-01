"""Short-window behaviour (rail compact density, Motors page shedding detail) and
the one armed/disarmed colour convention (green armed, red disarmed)."""

import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from ui.styles import PALETTE, build_stylesheet


class _Qt(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def tearDown(self):
        from ui.scaling import set_scale
        _dispose_hosts()
        set_scale(1.0)
        self.app.setStyleSheet(build_stylesheet())


_HOSTS = []


def _dispose_hosts():
    """Close and delete every embedded host while the event loop can still finish
    pending paints. Letting Python garbage-collect a SHOWN top-level widget is what
    crashes ("Cannot destroy paint device that is being painted")."""
    from PyQt5.QtWidgets import QApplication
    for host in _HOSTS:
        for t in host.findChildren(__import__("PyQt5.QtCore", fromlist=["QTimer"]).QTimer):
            t.stop()
        host.close()
        host.deleteLater()
    _HOSTS.clear()
    QApplication.instance().processEvents()


def _embed(widget_cls, w, h, scale):
    """Put a widget in a host WITH NO LAYOUT and give it an exact size.

    A top-level widget refuses to shrink below its minimum size, so it can never
    be "too short". Inside the real window it is a layout child and is simply
    handed whatever height is left - which can be less than it needs. A child
    with explicit geometry reproduces that."""
    from PyQt5.QtWidgets import QWidget
    from ui.scaling import set_scale
    set_scale(scale)
    from PyQt5.QtWidgets import QApplication
    QApplication.instance().setStyleSheet(build_stylesheet())
    host = QWidget()
    host.resize(w + 20, h + 20)
    child = widget_cls(host)
    child.setGeometry(0, 0, w, h)
    host.show()
    for _ in range(4):
        QApplication.instance().processEvents()
    child._host = host
    _HOSTS.append(host)
    return child


class RailDensityTest(_Qt):
    def _rail(self, scale, height):
        from ui.sidebar_nav import SidebarNav
        return _embed(SidebarNav, 176, height, scale)

    def _resize(self, rail, height):
        rail.setGeometry(0, 0, 176, height)
        for _ in range(4):
            self.app.processEvents()

    def test_a_tall_rail_keeps_normal_density(self):
        r = self._rail(1.35, 900)
        self.assertFalse(r.is_compact())

    def test_a_short_rail_goes_compact_and_then_fits(self):
        r = self._rail(1.35, 470)
        self.assertTrue(r.is_compact())
        self.assertGreaterEqual(r.height(), r.layout().minimumSize().height())

    def test_compact_actually_makes_the_rows_shorter(self):
        tall = self._rail(1.35, 900)
        short = self._rail(1.35, 470)
        b_tall = tall.btn_group.button(0).sizeHint().height()
        b_short = short.btn_group.button(0).sizeHint().height()
        self.assertLess(b_short, b_tall)

    def test_it_returns_to_normal_when_there_is_room_again(self):
        r = self._rail(1.35, 470)
        self.assertTrue(r.is_compact())
        self._resize(r, 900)
        self.assertFalse(r.is_compact())

    def test_no_flapping_at_the_boundary(self):
        r = self._rail(1.35, 900)
        need = r._need_normal
        states = []
        for h in (need + 6, need + 1, need, need - 1, need + 1, need + 6):
            self._resize(r, h)
            states.append(r.is_compact())
        # Compact exactly while the height is below the normal need, never random.
        self.assertEqual(states, [False, False, False, True, False, False])

    def test_normal_scale_never_needs_compact_at_the_supported_minimum(self):
        r = self._rail(1.0, 557)             # 1220x700, no alarm
        self.assertFalse(r.is_compact())

    def test_nav_buttons_stay_usable_in_compact(self):
        r = self._rail(1.35, 470)
        for b in r.btn_group.buttons():
            self.assertTrue(b.isVisible())
            self.assertGreaterEqual(b.height(), b.fontMetrics().height())


class MotorsPageSheddingTest(_Qt):
    def _page(self, scale, height):
        from ui.motor_widget import MotorWidget
        return _embed(MotorWidget, 1180, height, scale)

    def _resize(self, page, height):
        page.setGeometry(0, 0, 1180, height)
        for _ in range(4):
            self.app.processEvents()

    def test_a_tall_page_shows_everything(self):
        w = self._page(1.0, 800)
        self.assertTrue(w.lbl_scale.isVisible())
        self.assertFalse(w.test_panel._compact)
        self.assertTrue(w.test_panel.chk_link.isVisible())

    def test_a_short_page_drops_the_footnote_first(self):
        w = self._page(1.35, 800)
        # just short enough to need level 1 but not 2
        need0 = w.layout().minimumSize().height()
        self._resize(w, need0 - 10)
        self.assertFalse(w.lbl_scale.isVisible())
        self.assertFalse(w.test_panel._compact, "the checklist stays while the footnote suffices")

    def test_a_very_short_page_goes_compact_and_fits(self):
        w = self._page(1.35, 400)
        self.assertTrue(w.test_panel._compact)
        self.assertGreaterEqual(w.height(), w.layout().minimumSize().height())

    def test_compact_swaps_the_checklist_for_the_reason_line(self):
        w = self._page(1.35, 400)
        p = w.test_panel
        self.assertFalse(p.chk_link.isVisible())
        self.assertTrue(p.lbl_reason.isVisible())
        self.assertIn("No link", p.lbl_reason.text())
        w.set_vehicle_state(True, False, False)
        self.assertIn("props", p.lbl_reason.text())              # the next thing blocking the test

    def test_compact_motor_selector_is_one_row_and_still_works(self):
        w = self._page(1.35, 400)
        p = w.test_panel
        ys = {b.mapTo(p, b.rect().topLeft()).y() for b in p.motor_buttons.values()}
        self.assertEqual(len(ys), 1, "four buttons on one row")
        self.assertEqual([b.text() for b in p.motor_buttons.values()], ["M1", "M2", "M3", "M4"])
        w.set_vehicle_state(True, False, False)
        p.chk_safety.setChecked(True)
        p.motor_buttons[3].click()
        self.assertEqual(p.selected_motor(), 3)
        p._stop_hold()
        p._arm_expiry.stop()
        p._countdown.stop()

    def test_it_returns_to_full_detail_when_there_is_room_again(self):
        w = self._page(1.35, 400)
        self.assertTrue(w.test_panel._compact)
        self._resize(w, 900)
        self.assertFalse(w.test_panel._compact)
        self.assertTrue(w.lbl_scale.isVisible())
        self.assertEqual([b.text() for b in w.test_panel.motor_buttons.values()][0][:2], "M1")
        self.assertIn("FR", w.test_panel.motor_buttons[1].text())
        self.assertTrue(w.test_panel.chk_link.isVisible())

    def test_safety_interlocks_are_untouched_in_compact(self):
        w = self._page(1.35, 400)
        p = w.test_panel
        sent = []
        w.motor_test_requested.connect(lambda i, t: sent.append(i))
        p._start_hold()                      # no link, not acknowledged
        self.assertEqual(sent, [])
        w.set_vehicle_state(True, True, False)   # armed
        p._start_hold()
        self.assertEqual(sent, [])
        self.assertIn("ARMED", p.lbl_reason.text())


class MotorsWorstCaseTest(_Qt):
    """Smallest window, largest scale, alarm card showing, vehicle ARMED: the
    tallest state the Motors page can be in. Content grows here without the window
    being resized (arming adds a warning line), which an on-resize-only fit missed."""

    def _page(self, height=400):
        from ui.motor_widget import MotorWidget
        return _embed(MotorWidget, 1180, height, 1.35)

    def test_arming_while_the_page_is_open_still_fits(self):
        w = self._page()
        w.set_vehicle_state(True, True, False)          # armed -> a warning line appears
        for _ in range(8):
            self.app.processEvents()
        self.assertGreaterEqual(w.height(), w.layout().minimumSize().height())
        p = w.test_panel
        self.assertTrue(p._compact)
        self.assertIn("ARMED", p.lbl_reason.text())
        self.assertLess(p.lbl_reason.height(), p.lbl_reason.fontMetrics().height() * 2,
                        "the compact warning must be one line")

    def test_nothing_overlaps_in_the_worst_case(self):
        from PyQt5.QtWidgets import QPushButton, QLabel
        w = self._page()
        w.set_vehicle_state(True, True, False)
        for _ in range(8):
            self.app.processEvents()
        p = w.test_panel
        widgets = [x for x in p.findChildren((QPushButton, QLabel))
                   if x.isVisibleTo(p) and (getattr(x, "text", lambda: "")() or "").strip()]
        boxes = [(x, x.mapTo(p, x.rect().topLeft())) for x in widgets]
        bad = []
        for i, (a, pa) in enumerate(boxes):
            ra = a.rect().translated(pa)
            for b, pb in boxes[i + 1:]:
                if a.parentWidget() is not b.parentWidget():
                    continue
                inter = ra.intersected(b.rect().translated(pb))
                if inter.width() > 2 and inter.height() > 2:
                    bad.append(f"{a.text()!r} over {b.text()!r} ({inter.width()}x{inter.height()})")
        self.assertEqual(bad, [])

    def test_the_fit_converges_instead_of_ping_ponging(self):
        w = self._page()
        calls = []
        original = w._apply_level
        w._apply_level = lambda lvl: (calls.append(lvl), original(lvl))[1]
        w.set_vehicle_state(True, True, False)
        for _ in range(20):
            self.app.processEvents()
        n = len(calls)
        for _ in range(20):
            self.app.processEvents()
        self.assertEqual(len(calls), n, "still toggling after the layout settled")
        self.assertLessEqual(n, 12)

    def test_disarming_gives_the_detail_back_when_resized(self):
        w = self._page()
        w.set_vehicle_state(True, True, False)
        for _ in range(6):
            self.app.processEvents()
        w.setGeometry(0, 0, 1180, 900)                  # a real resize: room again
        for _ in range(6):
            self.app.processEvents()
        self.assertFalse(w.test_panel._compact)
        self.assertTrue(w.lbl_scale.isVisible())


class ArmedColourConventionTest(_Qt):
    """ARMED green, DISARMED red - one convention in the header, the navigation
    footer, and the Diagnostics tab."""

    def _block(self, qss, selector):
        import re
        m = re.search(re.escape(selector) + r"\s*\{", qss)
        self.assertIsNotNone(m, selector)
        return qss[m.start():qss.index("}", m.start())]

    def test_header_badges(self):
        qss = build_stylesheet()
        self.assertIn(PALETTE["ok"], self._block(qss, "QLabel#badgeArmed"))
        self.assertIn(PALETTE["danger"], self._block(qss, "QLabel#badgeDisarmed"))

    def test_navigation_footer(self):
        from ui.sidebar_nav import SidebarNav
        r = SidebarNav()
        r.setStyleSheet(r.styleSheet())
        qss = r.styleSheet()
        self.assertIn("#3fb950", self._block(qss, "QLabel#navFooterArmed"))
        self.assertIn("#f85149", self._block(qss, "QLabel#navFooterDisarmed"))
        r.set_flight_state("OFFBOARD", True)
        self.assertEqual(r.lbl_armed.objectName(), "navFooterArmed")
        r.set_flight_state("OFFBOARD", False)
        self.assertEqual(r.lbl_armed.objectName(), "navFooterDisarmed")

    def test_diagnostics_tile_and_glance_strip_agree(self):
        from core.telemetry import TelemetrySnapshot
        from ui import value_grid as vg
        g = vg.ValueGridWidget(["arm_state"])
        t = TelemetrySnapshot()
        for armed, colour in ((True, vg.OK), (False, vg.BAD)):
            t.armed = armed
            t.connected = True
            g.update_values(t)
            self.assertEqual(g._tiles["arm_state"]._base_colour, colour)
            self.assertEqual(g.strip.cells["mode"]._last[3], colour)

    def test_disarmed_red_is_a_state_not_a_fault(self):
        from core.telemetry import TelemetrySnapshot
        from ui import value_grid as vg
        g = vg.ValueGridWidget(["arm_state", "flight_mode"])
        g.update_values(TelemetrySnapshot())                  # disarmed
        self.assertEqual(g._tiles["arm_state"].severity, 0)
        self.assertEqual(g._cards[0].health, 0)
        self.assertFalse(g._tiles["arm_state"].bar.isVisible())


if __name__ == "__main__":
    unittest.main()
