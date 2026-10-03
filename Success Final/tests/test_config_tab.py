"""Configuration tab: labels, change tracking, validation, restart tags, presets."""

import os
import tempfile
import unittest
from dataclasses import fields

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from PyQt5.QtWidgets import QApplication, QCheckBox

from core.settings import GCSSettings
from ui import config_tab as ct


class PureTest(unittest.TestCase):
    def test_every_visible_field_has_a_human_caption(self):
        cfg = GCSSettings()
        for section in cfg.sections():
            for f in fields(getattr(cfg, section)):
                if (section, f.name) in ct.HIDDEN_FIELDS:
                    continue
                cap = ct.FIELD_INFO[section][1].get(f.name)
                self.assertIsNotNone(cap, f"{section}.{f.name} has no caption")
                self.assertNotIn("_", cap[0], f"{section}.{f.name} caption shows a raw key")

    def test_restart_rules_follow_what_the_main_window_applies_live(self):
        for sec in ("limits", "alerts", "audio", "actuator"):
            self.assertFalse(ct.needs_restart(sec, "x"))
        self.assertFalse(ct.needs_restart("video", "stream_url"))
        self.assertTrue(ct.needs_restart("video", "map_bridge_host"))
        self.assertTrue(ct.needs_restart("connection", "host"))
        self.assertTrue(ct.needs_restart("ui", "scale"))

    def test_parse_value_keeps_types(self):
        self.assertEqual(ct.parse_value(3, "4.0"), 4)
        self.assertEqual(ct.parse_value(1.0, "2.5"), 2.5)
        self.assertIs(ct.parse_value(True, "no"), False)
        with self.assertRaises(ValueError):
            ct.parse_value(1, "abc")

    def test_validation_messages_map_to_the_offending_field(self):
        cfg = GCSSettings()
        cfg.alerts.batt_crit_pct = 60
        cfg.limits.takeoff_alt_max_m = 0.1
        p = ct.validate_sections(cfg)
        self.assertIn("alerts.batt_crit_pct", p)
        self.assertIn("limits.takeoff_alt_max_m", p)
        self.assertEqual(ct.validate_sections(GCSSettings()), {})

    def test_presets_only_name_real_fields(self):
        cfg = GCSSettings()
        for preset in ct.PRESETS.values():
            for key in preset:
                sec, name = key.split(".")
                self.assertTrue(hasattr(getattr(cfg, sec), name), key)
        self.assertNotIn("Field", ct.PRESETS)


class WidgetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        from ui.styles import build_stylesheet
        cls.app.setStyleSheet(build_stylesheet())

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        os.environ["DRONE_GCS_HOME"] = self.tmp.name
        self.w = ct.ConfigTabWidget(GCSSettings())
        self.w.resize(1200, 760)
        self.w.show()
        self.app.processEvents()

    def tearDown(self):
        self.w.close()
        self.w.deleteLater()
        os.environ.pop("DRONE_GCS_HOME", None)
        self.tmp.cleanup()

    def ed(self, key):
        return self.w._editors[key]

    def test_starts_clean(self):
        self.assertEqual(self.w.changed_keys(), [])
        self.assertEqual(self.w.lbl_changes.text(), "")
        self.assertTrue(self.w.btn_save.isEnabled())

    def test_edit_marks_field_counts_changes_and_flags_restart(self):
        self.ed("connection.host").setText("10.1.1.1")
        self.ed("limits.move_max_delta_m").setText("2.0")
        self.assertEqual(self.w.lbl_changes.text(), "2 unsaved changes")
        self.assertEqual(self.w._dirty_marks["connection.host"].text(), "●")
        self.assertEqual(self.w._restart_marks["connection.host"].text(), "restart needed")
        self.assertEqual(self.w._restart_marks["limits.move_max_delta_m"].text(), "")
        self.ed("connection.host").setText(GCSSettings().connection.host)
        self.assertEqual(self.w.lbl_changes.text(), "1 unsaved change")

    def test_inline_validation_blocks_save_and_names_the_field(self):
        self.ed("alerts.batt_crit_pct").setText("90")
        self.assertFalse(self.w.btn_save.isEnabled())
        self.assertTrue(self.w._error_labels["alerts.batt_crit_pct"].isVisibleTo(self.w))
        self.ed("alerts.batt_crit_pct").setText("20")
        self.assertTrue(self.w.btn_save.isEnabled())
        self.assertFalse(self.w._error_labels["alerts.batt_crit_pct"].isVisibleTo(self.w))

    def test_non_numeric_text_is_reported_in_the_field_row(self):
        self.ed("limits.takeoff_alt_max_m").setText("abc")
        msg = self.w._error_labels["limits.takeoff_alt_max_m"].text()
        self.assertIn("Takeoff maximum", msg)
        self.assertFalse(self.w.btn_save.isEnabled())

    def test_save_persists_and_clears_the_marks(self):
        got = []
        self.w.settings_saved.connect(got.append)
        self.ed("limits.move_max_delta_m").setText("2.5")
        self.ed("connection.host").setText("10.1.1.1")
        self.w._on_save()
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0].limits.move_max_delta_m, 2.5)
        self.assertEqual(self.w.changed_keys(), [])
        self.assertIn("connection.host", self.w.lbl_status.text())     # the restart note
        self.assertNotIn("limits.move_max_delta_m", self.w.lbl_status.text())

    def test_hidden_value_grid_fields_survive_a_save(self):
        self.w.settings.ui.value_grid_fields = ["alt", "batt"]
        self.ed("ui.value_grid_columns").setText("4")
        self.w._on_save()
        self.assertEqual(self.w.settings.ui.value_grid_fields, ["alt", "batt"])

    def test_reset_section_restores_defaults_but_marks_changes(self):
        self.ed("alerts.batt_warn_pct").setText("50")
        self.w._on_save()
        self.w.reset_section("alerts")
        self.assertEqual(self.ed("alerts.batt_warn_pct").text(), "35")
        self.assertEqual(self.w.changed_keys(), ["alerts.batt_warn_pct"])

    def test_presets_fill_in_but_do_not_save(self):
        self.w.apply_preset("Bench")
        self.assertEqual(self.ed("connection.protocol").currentText(), "tcp")
        self.assertFalse(self.ed("audio.enabled").isChecked())
        self.assertIn("connection.protocol", self.w.changed_keys())
        self.assertEqual(self.w.settings.connection.protocol, "udp")

    def test_search_filters_rows_and_cards(self):
        self.w.search.setText("battery")
        vis = lambda k: self.w._rows[k][1].isVisibleTo(self.w)   # noqa: E731
        self.assertTrue(vis("alerts.batt_warn_pct"))
        self.assertTrue(vis("profile.battery_cells"))
        self.assertFalse(vis("connection.host"))
        self.assertFalse(self.w._cards["connection"].isVisibleTo(self.w))
        self.w.search.setText("zzzz")
        self.assertTrue(self.w.lbl_no_match.isVisibleTo(self.w))
        self.w.search.setText("")
        self.assertTrue(self.w._cards["connection"].isVisibleTo(self.w))

    def test_toggles_use_the_neutral_style_not_the_amber_one(self):
        boxes = [e for e in self.w._editors.values() if isinstance(e, QCheckBox)]
        self.assertTrue(boxes)
        for b in boxes:
            self.assertEqual(b.objectName(), "cfgToggle")

    def test_reload_discards_edits(self):
        self.ed("connection.host").setText("1.2.3.4")
        self.w._on_reload()
        self.assertEqual(self.w.changed_keys(), [])


if __name__ == "__main__":
    unittest.main()
