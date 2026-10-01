"""Diagnostics tab redesign: glance strip, system cards, grey-first colour,
responsive columns, one-line heading, compact Layout menu."""

import os
import time
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core.telemetry import TelemetrySnapshot
from ui import value_grid as vg


def snap(**kw):
    t = TelemetrySnapshot()
    for k, v in kw.items():
        setattr(t, k, v)
    return t


def live(**kw):
    base = dict(connected=True, battery_percent=62, battery_voltage=15.2,
                battery_current=8.4, altitude=1.43, ground_speed=0.42,
                flight_mode="OFFBOARD", flight_time_sec=137,
                last_position_time=time.time(), position_stale=False,
                heartbeat_age=0.2)
    base.update(kw)
    return snap(**base)


class GreyFirstColourTest(unittest.TestCase):
    def test_position_is_grey_not_red_before_any_data(self):
        t = snap(connected=True, last_position_time=0.0, position_stale=True)
        self.assertEqual(vg._pos_colour(t), vg.DIM)
        self.assertEqual(vg._pos_colour(snap(connected=False)), vg.DIM)

    def test_position_is_red_only_when_a_live_feed_stops(self):
        t = snap(connected=True, last_position_time=time.time() - 10, position_stale=True)
        self.assertEqual(vg._pos_colour(t), vg.BAD)
        t.position_stale = False
        self.assertIsNone(vg._pos_colour(t))

    def test_healthy_battery_is_plain_white_low_is_amber_critical_is_red(self):
        self.assertIsNone(vg._batt_colour(snap(battery_percent=80)))
        self.assertEqual(vg._batt_colour(snap(battery_percent=30)), vg.WARN)
        self.assertEqual(vg._batt_colour(snap(battery_percent=10)), vg.BAD)
        self.assertEqual(vg._batt_colour(snap(battery_percent=0)), vg.DIM)

    def test_split_value_and_placeholders(self):
        self.assertEqual(vg.split_value("15.20 V"), ("15.20", " V"))
        self.assertIsNone(vg.split_value("ARMED"))
        self.assertIsNone(vg.split_value("1 / 1"))
        for p in ("--", "", "n/r"):
            self.assertIn(p, vg.PLACEHOLDERS)

    def test_severity_levels(self):
        self.assertEqual(vg.severity_of(vg.BAD), 2)
        self.assertEqual(vg.severity_of(vg.WARN), 1)
        self.assertEqual(vg.severity_of(vg.OK), 0)
        self.assertEqual(vg.severity_of(None), 0)
        self.assertEqual(vg.severity_of(vg.DIM), 0)


class _Qt(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])
        from ui.styles import build_stylesheet
        cls.app.setStyleSheet(build_stylesheet())

    def _grid(self, w=1100, h=700, **kw):
        g = vg.ValueGridWidget(**kw)
        g.resize(w, h)
        g.show()
        for _ in range(4):
            self.app.processEvents()
        return g

    def tearDown(self):
        from ui.scaling import set_scale
        set_scale(1.0)


class TileTest(_Qt):
    def test_caption_is_above_the_value_and_small_caps(self):
        t = vg.ValueTile("volt")
        self.assertEqual(t.lbl_caption.text(), "BATTERY")
        self.assertEqual(t.lbl_caption.objectName(), "diagCaption")
        lay = t.layout().itemAt(1).layout()           # the caption/value column
        self.assertIs(lay.itemAt(0).layout().itemAt(0).widget(), t.lbl_caption)

    def test_severity_bar_only_appears_for_caution_or_fault(self):
        t = vg.ValueTile("volt")
        t.show()
        t.set_value("15.20 V", None)
        self.assertFalse(t.bar.isVisible())
        t.set_value("7.00 V", vg.WARN)
        self.assertTrue(t.bar.isVisible())
        self.assertEqual(t.severity, 1)
        t.set_value("5.00 V", vg.BAD)
        self.assertEqual(t.severity, 2)
        t.set_value("15.20 V", vg.DIM)
        self.assertFalse(t.bar.isVisible())

    def test_placeholder_is_plain_text_not_a_pill(self):
        t = vg.ValueTile("m1")
        t.show()
        t.set_value("--", vg.DIM)
        self.assertFalse(t.lbl_badge.isVisible())
        self.assertTrue(t.lbl_value.isVisible())

    def test_status_text_is_a_pill(self):
        t = vg.ValueTile("arm_state")
        t.show()
        t.set_value("ARMED", vg.BAD)
        self.assertTrue(t.lbl_badge.isVisible())
        self.assertFalse(t.lbl_value.isVisible())


class GlanceStripTest(_Qt):
    def test_six_readouts_in_order(self):
        s = vg.PrimaryStrip()
        self.assertEqual(list(s.cells), ["alt", "speed", "batt", "link", "mode", "time"])

    def test_values_from_a_snapshot(self):
        s = vg.PrimaryStrip()
        s.show()
        s.update_from(live())
        c = s.cells
        self.assertEqual(c["alt"].main_text, "1.43 m")
        self.assertEqual(c["speed"].main_text, "0.42 m/s")
        self.assertEqual(c["batt"].main_text, "62 %")
        self.assertIn("15.20 V", c["batt"].sub_text)
        self.assertEqual(c["link"].main_text, "CONNECTED")
        self.assertEqual(c["mode"].main_text, "OFFBOARD")
        self.assertEqual(c["time"].main_text, "02:17")

    def test_armed_green_disarmed_red_and_healthy_numbers_are_not_coloured(self):
        s = vg.PrimaryStrip()
        s.update_from(live(armed=True))
        self.assertEqual(s.cells["mode"].sub_text, "ARMED")
        self.assertEqual(s.cells["mode"]._last[3], vg.OK)
        s.update_from(live(armed=False))
        self.assertEqual(s.cells["mode"].sub_text, "DISARMED")
        self.assertEqual(s.cells["mode"]._last[3], vg.BAD)
        self.assertEqual(s.cells["alt"].main_colour, "")
        self.assertEqual(s.cells["batt"].main_colour, "")

    def test_low_battery_turns_the_battery_cell_amber(self):
        s = vg.PrimaryStrip()
        s.update_from(live(battery_percent=30))
        self.assertEqual(s.cells["batt"].main_colour, vg.WARN)

    def test_no_link_is_red_and_everything_else_recedes(self):
        s = vg.PrimaryStrip()
        s.update_from(snap(connected=False))
        self.assertEqual(s.cells["link"].main_text, "NO LINK")
        self.assertEqual(s.cells["link"].main_colour, vg.BAD)
        self.assertEqual(s.cells["speed"].main_colour, vg.DIM)
        self.assertEqual(s.cells["batt"].main_text, "--")

    def test_a_meaningless_heartbeat_age_is_not_shown(self):
        s = vg.PrimaryStrip()
        s.update_from(live(heartbeat_age=999.0))
        self.assertEqual(s.cells["link"].sub_text, "")

    def test_reflows_to_two_rows_when_narrow_and_one_when_wide(self):
        s = vg.PrimaryStrip()
        self.assertEqual(s.reflow(2000), 6)
        self.assertEqual(s.reflow(500), 3)
        self.assertEqual(s.reflow(2000), 6)

    def test_no_stale_column_stretch_after_reflow(self):
        s = vg.PrimaryStrip()
        s.reflow(500)
        self.assertEqual([s._grid.columnStretch(c) for c in range(3, 6)], [0, 0, 0])
        self.assertEqual([s._grid.columnStretch(c) for c in range(3)], [1, 1, 1])

    def test_all_cells_are_the_same_height_so_captions_line_up(self):
        s = vg.PrimaryStrip()
        s.resize(1200, 120)
        s.show()
        s.update_from(live())
        for _ in range(3):
            self.app.processEvents()
        heights = {c.height() for c in s.cells.values()}
        tops = {c.lbl_caption.mapTo(s, c.lbl_caption.rect().topLeft()).y()
                for c in s.cells.values()}
        self.assertEqual(len(heights), 1)
        self.assertEqual(len(tops), 1, "captions must start at the same height")


class SystemCardTest(_Qt):
    def test_one_card_per_system_in_registry_order(self):
        g = self._grid(fields=["volt", "alt", "roll", "pct"])
        self.assertEqual([c.category for c in g._cards],
                         [c for c in vg.category_order() if c in {"Power", "Attitude", "Position"}])
        power = next(c for c in g._cards if c.category == "Power")
        self.assertEqual({t.key for t in power.tiles}, {"volt", "pct"})

    def test_health_light_is_grey_amber_red(self):
        g = self._grid(fields=["volt", "pct", "curr"])
        card = g._cards[0]
        g.update_values(live(battery_percent=80))
        self.assertEqual(card.health, 0)
        g.update_values(live(battery_percent=30))
        self.assertEqual(card.health, 1)
        self.assertIn(vg.WARN, card.led.styleSheet())
        g.update_values(live(battery_percent=10))
        self.assertEqual(card.health, 2)
        self.assertIn(vg.BAD, card.led.styleSheet())
        g.update_values(live(battery_percent=80))
        self.assertEqual(card.health, 0)
        self.assertIn(vg._LED_IDLE, card.led.styleSheet())

    def test_the_tooltip_names_what_is_wrong(self):
        g = self._grid(fields=["volt", "pct"])
        g.update_values(live(battery_percent=10))
        self.assertIn("Fault", g._cards[0].led.toolTip())

    def test_every_tile_lives_in_exactly_one_card(self):
        g = self._grid()
        in_cards = [t.key for c in g._cards for t in c.tiles]
        self.assertEqual(sorted(in_cards), sorted(g.field_keys()))
        self.assertEqual(len(in_cards), len(set(in_cards)))


class ResponsiveColumnsTest(_Qt):
    def test_columns_is_a_maximum_never_more_than_fit(self):
        from ui.scaling import set_scale
        for scale in (1.0, 1.35):
            set_scale(scale)
            for w in (500, 700, 900, 1200, 1700):
                g = self._grid(w=w, columns=3)
                cols = g.active_columns()
                self.assertLessEqual(cols, 3)
                self.assertGreaterEqual(cols, 1)
                from ui.scaling import px
                avail = g._scroll.viewport().width()
                card_w = (avail - (cols - 1) * px(g.GAP_PX)) / cols
                self.assertGreaterEqual(card_w, px(g.MIN_CARD_PX) - 2,
                                        f"cards squeezed at w={w} scale={scale}")
                self.assertEqual(g._scroll.horizontalScrollBar().maximum(), 0,
                                 f"horizontal overflow at w={w} scale={scale}")
                g.close()

    def test_wide_window_uses_the_operators_maximum(self):
        g = self._grid(w=1700, columns=3)
        self.assertEqual(g.active_columns(), 3)

    def test_changing_the_maximum_relays_and_persists(self):
        g = self._grid(w=1700, columns=3)
        seen = []
        g.layout_changed.connect(lambda k, c, f: seen.append(c))
        g._on_columns_changed(2)
        self.assertEqual(g.active_columns(), 2)
        self.assertEqual(seen[-1], 2)
        self.assertTrue(g._col_actions[2].isChecked())

    def test_resizing_narrower_drops_a_column_without_rebuilding_tiles(self):
        g = self._grid(w=1700, columns=3)
        tiles_before = dict(g._tiles)
        g.resize(700, 700)
        for _ in range(4):
            self.app.processEvents()
        self.assertLess(g.active_columns(), 3)
        self.assertEqual(g._tiles, tiles_before)

    def test_masonry_keeps_every_card_and_balances_columns(self):
        g = self._grid(w=1700, columns=3)
        placed = []
        heights = []
        for i in range(g._cards_layout.count()):
            lay = g._cards_layout.itemAt(i).layout()
            col = [lay.itemAt(j).widget() for j in range(lay.count())
                   if lay.itemAt(j).widget() is not None]
            placed += col
            heights.append(sum(c.sizeHint().height() for c in col))
        self.assertEqual(len(placed), len(g._cards))
        self.assertLess(max(heights) - min(heights), max(heights) * 0.6)


class HeaderTest(_Qt):
    def test_heading_is_one_line_and_never_wraps(self):
        from PyQt5.QtWidgets import QLabel
        from ui.scaling import set_scale
        for scale in (1.0, 1.35, 2.0):
            set_scale(scale)
            g = self._grid(w=900)
            lbl = next(x for x in g.findChildren(QLabel) if x.text() == "TELEMETRY VALUES")
            self.assertFalse(lbl.wordWrap())
            self.assertGreaterEqual(lbl.width(), lbl.sizeHint().width())
            g.close()

    def test_the_toolbar_is_one_menu_not_five_controls(self):
        g = self._grid()
        from PyQt5.QtWidgets import QPushButton, QSpinBox, QComboBox
        self.assertEqual(g.findChildren(QSpinBox), [])
        self.assertEqual(g.findChildren(QComboBox), [])
        self.assertEqual([b for b in g.findChildren(QPushButton) if b.text() in ("Add…", "Reset", "Edit")], [])
        self.assertEqual(g.btn_layout.text().strip(" ▾"), "Layout")
        names = [a.text() for a in g.btn_layout.menu().actions()]
        self.assertIn("Add or remove values…", names)
        self.assertIn("Reset to default", names)

    def test_text_size_menu_changes_tiles_and_persists(self):
        g = self._grid()
        seen = []
        g.layout_changed.connect(lambda k, c, f: seen.append(f))
        g._on_font_changed(2)
        self.assertEqual(g.font_scale(), 1.25)
        self.assertEqual(seen[-1], 1.25)
        self.assertTrue(g._size_actions[2].isChecked())
        bigger = next(iter(g._tiles.values()))._value_size
        g._on_font_changed(0)
        self.assertLess(next(iter(g._tiles.values()))._value_size, bigger)

    def test_edit_mode_shows_the_chips(self):
        g = self._grid(fields=["volt", "alt"])
        g.act_edit.setChecked(True)
        t = g._tiles["volt"]
        self.assertTrue(t.btn_remove.isVisibleTo(t))
        g.act_edit.setChecked(False)
        self.assertFalse(t.btn_remove.isVisibleTo(t))

    def test_reset_restores_defaults_and_the_menu_state(self):
        g = self._grid(fields=["volt"], columns=5, font_scale=1.6)
        g._reset_layout()
        self.assertEqual(g.field_keys(), vg.DEFAULT_FIELDS)
        self.assertEqual(g.columns(), 3)
        self.assertEqual(g.font_scale(), 1.0)
        self.assertTrue(g._col_actions[3].isChecked())
        self.assertTrue(g._size_actions[1].isChecked())


class AssembledTabTest(_Qt):
    def test_no_horizontal_scrollbar_in_the_real_window_at_any_size_or_scale(self):
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_diag_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from ui.scaling import set_scale
        from ui.styles import build_stylesheet
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        bad = []
        for scale in (1.0, 1.35):
            set_scale(scale)
            self.app.setStyleSheet(build_stylesheet())
            for size in ((1600, 950), (1280, 760), (1220, 700)):
                w = drone_gcs.DroneGCSMainWindow()
                w.resize(*size)
                w.show()
                w.sidebar.select_tab(4)
                for _ in range(6):
                    self.app.processEvents()
                if w.value_grid._scroll.horizontalScrollBar().maximum() > 0:
                    bad.append(f"{size}@{scale}")
                w.shutdown_workers()
                w.close()
        self.assertEqual(bad, [])


if __name__ == "__main__":
    unittest.main()
