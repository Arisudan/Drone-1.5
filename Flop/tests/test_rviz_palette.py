"""The 2D tactical canvas must render occupancy exactly as RViz does.

Not hermetic - needs numpy; skipped where PyQt5 is absent since importing the
widget pulls it in.

WHY A RECORDED REFERENCE FILE:
  ``rviz_palettes_reference.txt`` is not hand-written. It was produced by
  linking a throwaway C++ program against the installed
  ``librviz_default_plugins.so`` and dumping all 256 entries of
  ``makeMapPalette(false, 100)`` and ``makeCostmapPalette(false, 100)`` -
  the exact functions RViz itself calls for the "map" and "costmap" colour
  schemes that config/rtabmap_drone.rviz selects for /map and /map_thin.

  Checking against a recording rather than against a re-implementation of the
  formula is the point. If someone "tidies" the LUT and shifts the ramp by one
  level, a formula-based test would move with it and still pass; this will not.
  Regenerating the file requires ROS 2 on the machine, so it is committed.

WHAT THIS PROTECTS:
  An operator switches between the 2D tab and the embedded RViz view looking at
  the same /map. If magenta means "lethal obstacle" in one and something else in
  the other, the colour has stopped being information. The unknown colour
  matters most: RViz marks never-observed cells with an opaque off-hue
  teal-grey, deliberately not a lighter shade of the occupancy ramp, so "clear"
  and "never looked at" are not separated by brightness alone.
"""

import re
import unittest
from pathlib import Path

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

try:
    import PyQt5  # noqa: F401  -- probe only; the widget import needs it
    HAVE_QT = True
except Exception:
    HAVE_QT = False

REFERENCE = Path(__file__).with_name("rviz_palettes_reference.txt")


def load_reference():
    """-> {"MAP": {value: (r,g,b,a)}, "COSTMAP": {...}} from the RViz dump."""
    out, cur = {}, None
    for line in REFERENCE.read_text().splitlines():
        m = re.match(r"=+\s*(\w+)\s*=+", line.strip())
        if m:
            cur = m.group(1)
            out[cur] = {}
            continue
        m = re.match(r"\s*(\d+):\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$", line)
        if m and cur:
            idx, r, g, b, a = (int(x) for x in m.groups())
            out[cur][idx] = (r, g, b, a)
    return out


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class PaletteMatchesRVizTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ref = load_reference()
        from ui.slam_map_widget import RAW_MAP_LUT, THIN_MAP_LUT
        cls.raw, cls.thin = RAW_MAP_LUT, THIN_MAP_LUT

    def test_reference_file_is_complete(self):
        for scheme in ("MAP", "COSTMAP"):
            self.assertIn(scheme, self.ref)
            self.assertEqual(len(self.ref[scheme]), 256, scheme)

    def _compare(self, lut, scheme):
        bad = []
        for idx, expected in self.ref[scheme].items():
            got = tuple(int(x) for x in lut[idx])
            if got != expected:
                bad.append(f"  [{idx}] got {got} want {expected}")
        self.assertEqual(bad, [], f"\n{scheme} differs from RViz:\n" + "\n".join(bad[:20]))

    def test_map_palette_matches_rviz_entry_for_entry(self):
        self._compare(self.raw, "MAP")

    def test_costmap_palette_matches_rviz_entry_for_entry(self):
        self._compare(self.thin, "COSTMAP")

    # The handful below are spelled out because they are the ones an operator
    # actually reads off the screen; a failure here should name the meaning,
    # not just an index.
    def test_free_space_is_white_and_occupied_is_black(self):
        self.assertEqual(tuple(self.raw[0]), (255, 255, 255, 255))
        self.assertEqual(tuple(self.raw[100]), (0, 0, 0, 255))

    def test_unknown_is_rvizs_opaque_teal_grey(self):
        self.assertEqual(tuple(self.raw[255]), (112, 137, 134, 255))
        self.assertEqual(tuple(self.thin[255]), (112, 137, 134, 255))

    def test_unknown_is_not_merely_a_lighter_free(self):
        # Guards the previous (205,205,205,160): unknown must differ in hue,
        # not only in brightness, from the free/occupied ramp.
        r, g, b, a = (int(x) for x in self.raw[255])
        self.assertEqual(a, 255, "unknown must be opaque")
        self.assertGreater(max(r, g, b) - min(r, g, b), 8,
                           "unknown must be off-hue, not a grey")

    def test_costmap_free_is_fully_transparent(self):
        self.assertEqual(tuple(self.thin[0]), (0, 0, 0, 0))

    def test_inscribed_is_cyan_and_lethal_is_magenta(self):
        self.assertEqual(tuple(self.thin[99]), (0, 255, 255, 255))
        self.assertEqual(tuple(self.thin[100]), (255, 0, 255, 255))

    def test_costmap_ramps_blue_to_red(self):
        self.assertEqual(tuple(self.thin[1]), (2, 0, 253, 255))
        self.assertEqual(tuple(self.thin[50]), (127, 0, 128, 255))
        self.assertEqual(tuple(self.thin[98]), (249, 0, 6, 255))


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class CanvasMatchesRVizGlobalOptionsTest(unittest.TestCase):
    def test_constants_match_the_shipped_rviz_config(self):
        from ui import slam_map_widget as m
        self.assertEqual(m.RVIZ_BACKGROUND, (48, 48, 48))
        self.assertEqual(m.RVIZ_GRID_RGB, (160, 160, 160))
        self.assertEqual(m.RVIZ_GRID_CELL_SIZE_M, 1.0)
        self.assertEqual(m.RVIZ_GRID_PLANE_CELL_COUNT, 10)
        self.assertAlmostEqual(m.RVIZ_MAP_ALPHA, 0.7)
        self.assertAlmostEqual(m.RVIZ_MAP_THIN_ALPHA, 1.0)
        self.assertEqual(m.RVIZ_GRID_ALPHA, int(round(0.7 * 255)))

    def test_constants_agree_with_the_rviz_file_itself(self):
        """Parse config/rtabmap_drone.rviz so the two cannot drift apart."""
        from ui import slam_map_widget as m
        cfg = (_env.REPO_ROOT / "config" / "rtabmap_drone.rviz").read_text()
        bg = re.search(r"Background Color:\s*(\d+);\s*(\d+);\s*(\d+)", cfg)
        self.assertIsNotNone(bg)
        self.assertEqual(tuple(int(g) for g in bg.groups()), m.RVIZ_BACKGROUND)
        self.assertIn("Color Scheme: map", cfg)
        self.assertIn("Color Scheme: costmap", cfg)
        cells = re.search(r"Plane Cell Count:\s*(\d+)", cfg)
        self.assertEqual(int(cells.group(1)), m.RVIZ_GRID_PLANE_CELL_COUNT)

    def test_canvas_paints_the_rviz_background(self):
        """Render offscreen and read the corner pixel back."""
        from PyQt5.QtWidgets import QApplication
        from PyQt5.QtGui import QImage
        app = QApplication.instance() or QApplication([])   # noqa: F841
        from ui.slam_map_widget import SLAMMapCanvas
        c = SLAMMapCanvas()
        c.resize(400, 300)
        img = QImage(400, 300, QImage.Format_RGB32)
        img.fill(0)
        c.render(img)
        px = QImage.pixelColor(img, 3, 3)
        self.assertEqual((px.red(), px.green(), px.blue()), (48, 48, 48))

    def test_canvas_renders_a_map_without_error(self):
        import numpy as np
        from PyQt5.QtWidgets import QApplication
        from PyQt5.QtGui import QImage
        app = QApplication.instance() or QApplication([])   # noqa: F841
        from ui.slam_map_widget import SLAMMapCanvas
        c = SLAMMapCanvas()
        c.resize(400, 300)
        grid = np.full((40, 40), -1, dtype=np.int8)
        grid[10:30, 10] = 100
        grid[10:30, 29] = 99
        grid[15:25, 15:25] = 0
        c.set_occupancy_grid(grid, 0.05, -1.0, -1.0, "/map_thin")
        c.set_drone_pose(0.0, 0.0, 0.0)
        img = QImage(400, 300, QImage.Format_RGB32)
        img.fill(0)
        c.render(img)     # must not raise
        self.assertEqual(img.width(), 400)


if __name__ == "__main__":
    unittest.main(verbosity=2)
