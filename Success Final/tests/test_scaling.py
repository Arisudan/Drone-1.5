"""UI scale resolution and stylesheet rewriting (ui/scaling.py).

Hermetic: nothing here imports PyQt. The point of ui.scaling is that px() and
scale_qss() are pure functions of one number, usable before a QApplication
exists, and these tests hold that property in place.

The rule worth protecting is the one that is easy to "fix" wrongly: a
`border: 1px solid` width must NOT scale. Multiply it and every panel edge in
the station becomes a 2-3px slab.
"""

import os
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from ui import scaling


class ClampTest(unittest.TestCase):
    def tearDown(self):
        scaling.set_scale(1.0)

    def test_zero_and_negative_mean_unset_not_tiny(self):
        # 0 is the "auto" sentinel in settings, and must never be taken
        # literally as a scale factor.
        self.assertEqual(scaling.clamp_scale(0), 1.0)
        self.assertEqual(scaling.clamp_scale(-3), 1.0)

    def test_garbage_falls_back_to_identity(self):
        self.assertEqual(scaling.clamp_scale("wide"), 1.0)
        self.assertEqual(scaling.clamp_scale(None), 1.0)

    def test_extremes_are_clamped_to_a_usable_window(self):
        self.assertEqual(scaling.clamp_scale(0.1), scaling.MIN_SCALE)
        self.assertEqual(scaling.clamp_scale(99), scaling.MAX_SCALE)


class PixelTest(unittest.TestCase):
    def tearDown(self):
        scaling.set_scale(1.0)

    def test_identity_at_one(self):
        scaling.set_scale(1.0)
        self.assertEqual(scaling.px(37), 37)

    def test_scales_and_rounds(self):
        # Rounding is Python's round(), i.e. banker's rounding on an exact
        # half. Pinned here so the convention is a decision rather than an
        # accident: 16.5 -> 16, 19.5 -> 20. Sub-pixel either way, and
        # consistent, which is what matters for a layout.
        scaling.set_scale(1.5)
        self.assertEqual(scaling.px(10), 15)
        self.assertEqual(scaling.px(11), 16)   # 16.5 -> 16
        self.assertEqual(scaling.px(13), 20)   # 19.5 -> 20

    def test_a_one_pixel_rule_never_disappears(self):
        # A 1px separator scaled by 0.75 rounds to 1, not 0: a divider that
        # vanishes at small scales reads as a layout bug.
        scaling.set_scale(0.75)
        self.assertEqual(scaling.px(1), 1)

    def test_point_sizes_have_a_legibility_floor(self):
        scaling.set_scale(0.75)
        self.assertGreaterEqual(scaling.pt(6), 5)


class StylesheetRewriteTest(unittest.TestCase):
    def tearDown(self):
        scaling.set_scale(1.0)

    def test_font_size_scales(self):
        out = scaling.scale_qss("QLabel { font-size: 10px; }", 2.0)
        self.assertIn("font-size: 20px", out)

    def test_border_width_does_not_scale(self):
        out = scaling.scale_qss(
            "QFrame { border: 1px solid #fff; border-radius: 4px; }", 2.0)
        self.assertIn("border: 1px solid #fff", out)
        self.assertIn("border-radius: 8px", out)

    def test_multi_value_padding_scales_every_component(self):
        out = scaling.scale_qss("QPushButton { padding: 3px 8px; }", 2.0)
        self.assertIn("padding: 6px 16px", out)

    def test_unrecognised_properties_are_left_alone(self):
        # A px count inside a url() or an unknown property is not a dimension
        # this module understands, and guessing would corrupt it.
        qss = "QComboBox { image: url(/a/chevron_16px.png); outline-offset: 2px; }"
        self.assertEqual(scaling.scale_qss(qss, 2.0), qss)

    def test_identity_scale_returns_the_input_unchanged(self):
        qss = "QLabel { font-size: 11px; padding: 2px; }"
        self.assertEqual(scaling.scale_qss(qss, 1.0), qss)

    def test_the_real_stylesheet_scales_every_font_rule(self):
        import re
        from ui.styles import build_stylesheet
        base = build_stylesheet(1.0)
        big = build_stylesheet(2.0)
        base_sizes = [int(m) for m in re.findall(r"font-size: (\d+)px", base)]
        big_sizes = [int(m) for m in re.findall(r"font-size: (\d+)px", big)]
        self.assertTrue(base_sizes, "stylesheet has no font-size rules to scale")
        self.assertEqual(len(base_sizes), len(big_sizes))
        self.assertEqual([s * 2 for s in base_sizes], big_sizes)
        # And the hairlines are untouched at any scale.
        self.assertEqual(base.count("border: 1px solid"),
                         big.count("border: 1px solid"))


class ResolutionOrderTest(unittest.TestCase):
    """--ui-scale beats GCS_UI_SCALE beats settings.ui.scale beats auto."""

    def setUp(self):
        self._saved = os.environ.pop("GCS_UI_SCALE", None)

    def tearDown(self):
        os.environ.pop("GCS_UI_SCALE", None)
        if self._saved is not None:
            os.environ["GCS_UI_SCALE"] = self._saved
        scaling.set_scale(1.0)

    class _Cfg:
        class ui:
            scale = 1.25

    def test_explicit_override_wins(self):
        os.environ["GCS_UI_SCALE"] = "2.0"
        self.assertAlmostEqual(
            scaling.init_scale(None, self._Cfg(), override=1.75), 1.75)

    def test_environment_beats_settings(self):
        os.environ["GCS_UI_SCALE"] = "2.0"
        self.assertAlmostEqual(scaling.init_scale(None, self._Cfg()), 2.0)

    def test_settings_used_when_nothing_else_set(self):
        self.assertAlmostEqual(scaling.init_scale(None, self._Cfg()), 1.25)

    def test_zero_in_settings_means_auto_not_collapse(self):
        class Auto:
            class ui:
                scale = 0.0
        # No Qt application here, so detection returns the 1.0 fallback -
        # what matters is that a 0 does not become a 0-sized UI.
        self.assertGreaterEqual(scaling.init_scale(None, Auto()), scaling.MIN_SCALE)

    def test_unparsable_value_does_not_raise(self):
        os.environ["GCS_UI_SCALE"] = "enormous"
        self.assertGreaterEqual(scaling.init_scale(None, None), scaling.MIN_SCALE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
