"""The tactical-map drone glyph must scale with zoom, within bounds.

Not hermetic - needs PyQt5 (skipped where absent). The glyph was once drawn in
fixed pixels, so it read the same size whether the operator was looking at a
whole floor or a single doorway. These tests render the canvas and measure the
diagonal-pair motor discs (light-blue / amber), which only the glyph draws.
"""

import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

try:
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtGui import QImage
    HAVE_QT = True
except Exception:
    HAVE_QT = False


def _front_prop_span(canvas, scale):
    """Horizontal pixel distance spanned by the glyph's motor discs.

    The four motor discs are coloured by diagonal pair (light-blue /
    amber - matching a real X-quad's CW/CCW motor layout), not a single
    front colour any more, so this matches either one.
    """
    canvas.scale = scale
    canvas.pan_x = canvas.pan_y = 0
    img = QImage(600, 600, QImage.Format_RGB32)
    img.fill(0)
    canvas.render(img)
    xs = []
    # Only look near the centre: the glyph sits at the origin, and this keeps
    # the compass, scale bar and any other overlay out of the measurement.
    for y in range(150, 450):
        for x in range(150, 450):
            q = img.pixelColor(x, y)
            r, g, b = q.red(), q.green(), q.blue()
            is_light_blue = b > 200 and g > 150 and r < 180
            is_amber = r > 180 and 100 < g < 200 and b < 100
            if is_light_blue or is_amber:
                xs.append(x)
    return (max(xs) - min(xs) + 1) if xs else 0


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class DroneGlyphZoomTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        from ui import slam_map_widget as m
        cls.m = m
        cls.canvas = m.SLAMMapCanvas()
        cls.canvas.resize(600, 600)
        cls.canvas.set_drone_pose(0.0, 0.0, 0.0)

    def test_glyph_grows_when_zooming_in(self):
        self.assertGreater(_front_prop_span(self.canvas, 260.0),
                           _front_prop_span(self.canvas, 150.0) * 1.6)

    def test_glyph_shrinks_when_zooming_out(self):
        self.assertLess(_front_prop_span(self.canvas, 75.0),
                        _front_prop_span(self.canvas, 150.0) * 0.7)

    def test_glyph_never_vanishes_when_fully_zoomed_out(self):
        self.assertGreater(_front_prop_span(self.canvas, 4.0), 0)
        self.assertEqual(_front_prop_span(self.canvas, 4.0),
                         _front_prop_span(self.canvas, 8.0),
                         "below the floor the glyph should stop shrinking")

    def test_glyph_stops_growing_at_the_ceiling(self):
        cap = self.m.DRONE_GLYPH_MAX_SCALE * self.m.DRONE_GLYPH_REF_SCALE
        self.assertEqual(_front_prop_span(self.canvas, cap),
                         _front_prop_span(self.canvas, cap * 2))

    def test_default_zoom_is_the_designed_size(self):
        self.assertEqual(self.m.DRONE_GLYPH_REF_SCALE, 150.0)
        self.assertEqual(self.canvas.scale, self.m.DRONE_GLYPH_REF_SCALE)
        self.canvas.scale = 33.0
        self.canvas.reset_view()
        self.assertEqual(self.canvas.scale, 150.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
