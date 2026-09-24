"""Layout regression tests: nothing clipped, overlapping, or out of bounds.

Not hermetic - needs PyQt5, skipped where it is absent.

WHY THIS EXISTS AS A TEST RATHER THAN A SCREENSHOT REVIEW:
  A screenshot proves one window size at one UI scale. The failures that
  actually reached this codebase were all conditional on the other
  combinations: labels that only elide on a narrow window, and - after the UI
  gained a scale factor - controls that only collide once the text grows. At
  1.35x in a 1220x700 window the cockpit drew TAKEOFF, ROTATE YAW and CHANGE
  ALT on top of each other, with their captions collapsed to zero height, and
  nothing at the authored 1.0x scale hinted at it.

WHAT EACH CHECK MEANS:
  CLIPPED  a widget is narrower than the text it is being asked to render, so
           Qt elides it. On a label that is cosmetic; on a button that names a
           flight action it is not.
  OVERLAP  two sibling widgets occupy the same pixels. Always a bug here: this
           UI has no intentionally stacked controls inside a layout.
  ESCAPES  a child's geometry leaves its parent's rect, which is what a Qt
           layout does when it is given less space than its contents demand.

`drone_gcs.py` is imported here, unlike in test_smoke.py, because the whole
point is to exercise the assembled window. Its module body re-execs the
interpreter to inject ROS 2 paths, so this module is listed after the others
in run_tests.sh and guards the import.
"""

import os
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

try:
    from PyQt5.QtWidgets import (QApplication, QLabel, QPushButton, QCheckBox,
                                 QComboBox, QLineEdit, QScrollArea,
                                 QAbstractScrollArea)
    HAVE_QT = True
except Exception:
    HAVE_QT = False

# Widgets that are meant to sit on top of other content.
OVERLAP_EXEMPT = {"NotificationToast", "FloatingVideoWindow", "VideoSink",
                  "HUDWidget", "QScrollBar", "SLAMMapCanvas"}

PAGES = ["Control", "FPV", "SLAM", "Motors", "Diagnostics", "Terminal",
         "Logs", "Config"]

# The authored scale, and a large-text desktop. Window sizes span a roomy
# display down to the station's own declared minimum.
SCALES = (1.0, 1.35)
SIZES = ((1600, 950), (1280, 760), (1220, 700))


def _text(widget):
    if isinstance(widget, QComboBox):
        return widget.currentText()
    if isinstance(widget, QLineEdit):
        return widget.text() or widget.placeholderText()
    try:
        return widget.text()
    except Exception:
        return ""


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class LayoutIntegrityTest(unittest.TestCase):
    """One assembled window per (scale, size); every workspace inspected."""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_layout_test")

        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides

        # Keep the test off the network and off the camera: this is a geometry
        # test, and a blocking socket would make it a flaky one.
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None

        cls.drone_gcs = drone_gcs
        cls.cfg = apply_overrides(load_settings())

    def _windows(self):
        from ui.scaling import set_scale
        from ui.styles import build_stylesheet
        for scale in SCALES:
            set_scale(scale)
            self.app.setStyleSheet(build_stylesheet())
            for size in SIZES:
                win = self.drone_gcs.DroneGCSMainWindow(settings=self.cfg)
                win.resize(*size)
                win.show()
                for _ in range(6):
                    self.app.processEvents()
                yield win, scale, size
                win.shutdown_workers()
                win.close()
        set_scale(1.0)

    def _each_page(self):
        for win, scale, size in self._windows():
            for idx in range(8):
                win._switch_workspace(idx)
                for _ in range(4):
                    self.app.processEvents()
                # The whole window, not just the page: the header strip,
                # the navigation rail and the footer are as capable of
                # overlapping themselves as any workspace is.
                yield win, f"{PAGES[idx]} @{size[0]}x{size[1]} scale={scale}"

    def test_no_text_is_clipped(self):
        problems = []
        for page, tag in self._each_page():
            for child in page.findChildren((QLabel, QPushButton, QCheckBox,
                                            QComboBox, QLineEdit)):
                if not child.isVisibleTo(page) or child.width() <= 0:
                    continue
                txt = _text(child)
                if not txt or not txt.strip():
                    continue
                if isinstance(child, QLabel) and child.wordWrap():
                    continue
                if isinstance(child, QLineEdit):
                    # A line edit's sizeHint is "room for a typical value", not
                    # the width of what it holds - measure the string instead.
                    need = child.fontMetrics().horizontalAdvance(txt) + 12
                    if need > child.width() + 1:
                        problems.append(f"{tag}: {txt[:40]!r} needs {need}px, has {child.width()}px")
                    continue
                need = child.sizeHint().width()
                if need > child.width() + 1:
                    problems.append(f"{tag}: {txt[:40]!r} needs {need}px, has {child.width()}px")
        self.assertEqual(problems, [], "clipped text:\n  " + "\n  ".join(problems))

    def test_no_sibling_widgets_overlap(self):
        problems = []
        for page, tag in self._each_page():
            by_parent = {}
            for child in page.findChildren((QLabel, QPushButton, QCheckBox,
                                            QComboBox, QLineEdit)):
                parent = child.parentWidget()
                if parent is None or not child.isVisibleTo(page):
                    continue
                if type(child).__name__ in OVERLAP_EXEMPT:
                    continue
                by_parent.setdefault(id(parent), []).append(child)
            for kids in by_parent.values():
                for i in range(len(kids)):
                    for j in range(i + 1, len(kids)):
                        a, b = kids[i], kids[j]
                        hit = a.geometry().intersected(b.geometry())
                        if hit.width() > 1 and hit.height() > 1:
                            problems.append(
                                f"{tag}: {_text(a)[:18]!r} over {_text(b)[:18]!r} "
                                f"({hit.width()}x{hit.height()}px)")
        self.assertEqual(problems, [], "overlapping widgets:\n  " + "\n  ".join(problems))

    def test_no_widget_escapes_its_parent(self):
        problems = []
        for page, tag in self._each_page():
            for child in page.findChildren((QLabel, QPushButton, QCheckBox,
                                            QComboBox, QLineEdit)):
                parent = child.parentWidget()
                if parent is None or not child.isVisibleTo(page):
                    continue
                if parent.parentWidget() is None:
                    continue
                # A scroll area's contents are legitimately larger than its
                # viewport; that is what the scrollbar is for.
                if isinstance(parent, QAbstractScrollArea):
                    continue
                if any(isinstance(p, QScrollArea)
                       for p in (parent, parent.parentWidget()) if p is not None):
                    continue
                g, pr = child.geometry(), parent.rect()
                if (g.left() < pr.left() - 1 or g.top() < pr.top() - 1 or
                        g.right() > pr.right() + 1 or g.bottom() > pr.bottom() + 1):
                    problems.append(
                        f"{tag}: {_text(child)[:22]!r} at {g.x()},{g.y()} "
                        f"{g.width()}x{g.height()} outside parent {pr.width()}x{pr.height()}")
        self.assertEqual(problems, [], "widgets outside their parent:\n  " + "\n  ".join(problems))


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class FrameGeometryFitTest(unittest.TestCase):
    """The Quad X diagram must stay inside its widget at any shape.

    The first implementation sized the arm from `min(w, h) / 2` and then drew a
    rotor disc and a two-line caption beyond it, so the rotors were clipped by
    the widget edge at every aspect ratio and the front captions were painted
    across their own discs.
    """

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_fits_at_every_shape_and_scale(self):
        from ui.scaling import set_scale
        from ui.motor_widget import FrameGeometryWidget
        shapes = [(300, 460), (360, 360), (520, 300), (240, 240),
                  (200, 180), (700, 620), (900, 260), (260, 900)]
        try:
            for scale in (1.0, 1.35, 2.0):
                set_scale(scale)
                for w, h in shapes:
                    widget = FrameGeometryWidget()
                    widget.resize(w, h)
                    self.assertTrue(
                        widget.layout_fits(),
                        f"diagram overflows at {w}x{h} scale={scale}")
        finally:
            set_scale(1.0)

    def test_rotors_are_where_the_px4_mapping_says(self):
        from ui.motor_widget import FrameGeometryWidget
        widget = FrameGeometryWidget()
        widget.resize(400, 400)
        c = widget.rotor_centres()
        # Screen y grows downwards, so "front" is the smaller y.
        self.assertLess(c[1][1], c[4][1], "M1 (front right) must be above M4 (rear right)")
        self.assertLess(c[3][1], c[2][1], "M3 (front left) must be above M2 (rear left)")
        self.assertGreater(c[1][0], c[3][0], "M1 (right) must be right of M3 (left)")
        self.assertGreater(c[4][0], c[2][0], "M4 (right) must be right of M2 (left)")

    def test_clicking_a_rotor_selects_it(self):
        from ui.motor_widget import FrameGeometryWidget
        widget = FrameGeometryWidget()
        widget.resize(400, 400)
        for num, (x, y) in widget.rotor_centres().items():
            from PyQt5.QtCore import QPoint, Qt, QEvent
            from PyQt5.QtGui import QMouseEvent
            ev = QMouseEvent(QEvent.MouseButtonPress, QPoint(int(x), int(y)),
                             Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
            widget.mousePressEvent(ev)
            self.assertEqual(widget.selected, num)


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class NotificationToastTest(unittest.TestCase):
    """The toast must never cover anything, at any message length.

    It used to anchor off one named badge's right edge and grow to whatever
    width its message needed. Adding the AUDIO control to the header row after
    that badge put every notification straight on top of the word AUDIO, and a
    long message ran past the free gap and was clamped back left across the
    status badges.
    """

    MESSAGES = [
        "ARMED",
        "Mode set: OFFBOARD",
        "Obstacle Ahead (0.42m)! Drone Halted",
        "Airborne - landing safely, will disarm on touchdown",
        "FLIGHT INTERLOCK REJECTED: Drone is DISARMED! Arm drone and confirm "
        "clear airspace before executing autonomous path.",
    ]

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        cls.drone_gcs = drone_gcs
        cls.cfg = apply_overrides(load_settings())

    def test_toast_never_covers_header_text(self):
        from ui.scaling import set_scale
        from ui.styles import build_stylesheet
        problems = []
        try:
            for scale in SCALES:
                set_scale(scale)
                self.app.setStyleSheet(build_stylesheet())
                for size in SIZES:
                    win = self.drone_gcs.DroneGCSMainWindow(settings=self.cfg)
                    win.resize(*size)
                    win.show()
                    for _ in range(6):
                        self.app.processEvents()
                    strip = win.top_strip
                    for message in self.MESSAGES:
                        win.toast.show_message(message, "#3fb950", 60000)
                        for _ in range(2):
                            self.app.processEvents()
                        toast = win.toast.current_rect()
                        for child in strip.findChildren(
                                (QLabel, QPushButton, QLineEdit, QComboBox)):
                            if child.isHidden() or not _text(child).strip():
                                continue
                            geo = child.geometry()
                            geo.moveTopLeft(child.mapTo(win, child.rect().topLeft()))
                            hit = geo.intersected(toast)
                            if hit.width() > 1 and hit.height() > 1:
                                problems.append(
                                    f"{size} scale={scale}: {message[:30]!r} covers "
                                    f"{_text(child)[:20]!r} by {hit.width()}x{hit.height()}px")
                    win.shutdown_workers()
                    win.close()
        finally:
            set_scale(1.0)
        self.assertEqual(problems, [],
                         "toast overlaps header text:\n  " + "\n  ".join(problems))

    def test_the_header_always_offers_a_usable_band(self):
        """The gap the toast lives in is a reserved layout requirement, not
        incidental slack - without a floor it collapses on a narrow window and
        the toast has nowhere to go that is not already occupied."""
        from ui.scaling import set_scale
        from ui.styles import build_stylesheet
        try:
            for scale in SCALES:
                set_scale(scale)
                self.app.setStyleSheet(build_stylesheet())
                for size in SIZES:
                    win = self.drone_gcs.DroneGCSMainWindow(settings=self.cfg)
                    win.resize(*size)
                    win.show()
                    for _ in range(6):
                        self.app.processEvents()
                    band = win.top_strip.notification_rect()
                    self.assertGreaterEqual(
                        band.width(), 100,
                        f"notification band collapsed to {band.width()}px "
                        f"at {size} scale={scale}")
                    win.shutdown_workers()
                    win.close()
        finally:
            set_scale(1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
