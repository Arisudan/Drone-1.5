"""The MODE / SET MODE / ARM / DISARM row at the top right of the Tactical SLAM header."""
import _env  # noqa: F401  -- must be first

import unittest

from PyQt5.QtWidgets import QApplication

app = QApplication.instance() or QApplication([])

from ui.header_flight_bar import HeaderFlightBar  # noqa: E402
from ui.slam_map_widget import SLAMMapWidget  # noqa: E402

MODES = ["OFFBOARD", "POSCTL", "AUTO.LOITER", "AUTO.LAND", "STABILIZED"]


class HeaderFlightBarTests(unittest.TestCase):
    def setUp(self):
        self.bar = HeaderFlightBar(MODES)

    def test_never_greyed_out(self):
        for connected, armed in ((False, False), (True, False), (True, True)):
            self.bar.set_state(connected, armed, "STABILIZED")
            for w in (self.bar.combo_mode, self.bar.btn_set_mode, self.bar.btn_arm, self.bar.btn_disarm):
                self.assertTrue(w.isEnabled(), (connected, armed))

    def test_state_chip_text_and_state(self):
        self.bar.set_state(False, False)
        self.assertEqual((self.bar.lbl_state.text(), self.bar.lbl_state.property("state")), ("NO LINK", "offline"))
        self.bar.set_state(True, False, "STABILIZED")
        self.assertEqual((self.bar.lbl_state.text(), self.bar.lbl_state.property("state")), ("DISARMED", "idle"))
        self.bar.set_state(True, True, "AUTO.LOITER")
        self.assertEqual((self.bar.lbl_state.text(), self.bar.lbl_state.property("state")), ("ARMED", "armed"))

    def test_one_height_and_one_font_size(self):
        parts = (self.bar.lbl_state, self.bar.combo_mode, self.bar.btn_set_mode, self.bar.btn_arm, self.bar.btn_disarm)
        self.bar.show()
        app.processEvents()
        self.assertEqual(len({p.height() for p in parts}), 1)
        sizes = {p.font().pixelSize() if p.font().pixelSize() > 0 else round(p.font().pointSizeF()) for p in parts[1:]}
        self.assertEqual(len(sizes), 1, sizes)
        self.bar.hide()

    def test_colours_are_blue_green_red(self):
        self.assertEqual(self.bar.btn_set_mode.objectName(), "btnNav")      # blue
        self.assertEqual(self.bar.btn_arm.objectName(), "btnArm")           # green
        self.assertEqual(self.bar.btn_disarm.objectName(), "btnDisarm")     # red

    def test_mode_box_follows_vehicle_mode(self):
        self.bar.set_state(True, False, "AUTO.LAND")
        self.assertEqual(self.bar.combo_mode.currentText(), "AUTO.LAND")

    def test_signals(self):
        got = []
        self.bar.arm_requested.connect(lambda: got.append("arm"))
        self.bar.disarm_requested.connect(lambda: got.append("disarm"))
        self.bar.mode_requested.connect(lambda m: got.append(m))
        self.bar.set_state(True, False, "STABILIZED")
        self.bar.btn_arm.click()
        self.bar.combo_mode.setCurrentText("POSCTL")
        self.bar.btn_set_mode.click()
        self.bar.set_state(True, True, "POSCTL")
        self.bar.btn_disarm.click()
        self.assertEqual(got, ["arm", "POSCTL", "disarm"])


class HeaderPlacementTests(unittest.TestCase):
    def test_same_line_top_right_no_clipping(self):
        w = SLAMMapWidget(flight_modes=MODES)
        w.resize(1400, 800)   # a typical SLAM page; narrower ones wrap the bar onto its own row (next test)
        w.show()
        app.processEvents()
        self.assertEqual(w._header_state, 0)
        bar = w.flight_bar
        parts = [bar.combo_mode, bar.btn_set_mode, bar.btn_arm, bar.btn_disarm]
        ys = {(p.mapTo(w, p.rect().topLeft()).y(), p.height()) for p in parts}
        self.assertEqual(len(ys), 1, f"not on one line / one height: {ys}")
        xs = [p.mapTo(w, p.rect().topLeft()).x() for p in parts]
        self.assertEqual(xs, sorted(xs))
        # right of the view switch, inside the window, and at the top
        self.assertGreater(xs[0], w.btn_view_rviz.mapTo(w, w.btn_view_rviz.rect().topRight()).x())
        last = parts[-1]
        self.assertLessEqual(last.mapTo(w, last.rect().topRight()).x(), w.width())
        self.assertLess(ys.pop()[0], 60)
        for p in parts:
            self.assertGreaterEqual(p.width(), p.minimumWidth())
        self.assertEqual(bar.btn_arm.width(), bar.btn_disarm.width())
        w.close()


class MapHintTests(unittest.TestCase):
    def test_controls_hint_is_temporary_and_help_stays_on_hover(self):
        w = SLAMMapWidget(flight_modes=MODES)
        w.resize(1280, 800)
        w.show()
        app.processEvents()
        self.assertTrue(w.canvas.show_controls_hint)
        w._hide_controls_hint()
        self.assertFalse(w.canvas.show_controls_hint)
        self.assertIn("Right click", w.lbl_help.toolTip())
        w.close()


class NarrowHeaderTests(unittest.TestCase):
    def test_bar_never_clipped_or_outside_at_any_size_or_scale(self):
        from ui.scaling import set_scale
        from ui.styles import build_stylesheet
        try:
            for scale in (1.0, 1.35):
                set_scale(scale)
                app.setStyleSheet(build_stylesheet())
                for size in ((1280, 760), (1220, 700), (1000, 700), (1600, 900)):
                    w = SLAMMapWidget(flight_modes=MODES)
                    w.resize(*size)
                    w.show()
                    for _ in range(4):
                        app.processEvents()
                    bar = w.flight_bar
                    parts = [bar.combo_mode, bar.btn_set_mode, bar.btn_arm, bar.btn_disarm]
                    ys = {p.mapTo(w, p.rect().topLeft()).y() for p in parts}
                    self.assertEqual(len(ys), 1, (scale, size))
                    for p in parts:
                        tl = p.mapTo(w, p.rect().topLeft())
                        br = p.mapTo(w, p.rect().bottomRight())
                        self.assertGreaterEqual(tl.x(), 0, (scale, size))
                        self.assertLessEqual(br.x(), w.width(), (scale, size))
                    w.close()
        finally:
            set_scale(1.0)
            app.setStyleSheet(build_stylesheet())


class MainWindowWiringTests(unittest.TestCase):
    """Header buttons -> the Control tab's own handlers -> the worker."""

    @classmethod
    def setUpClass(cls):
        import os
        from unittest import mock
        os.environ.setdefault("DRONE_GCS_HOME", "/tmp/.drone_gcs_header_bar_test")
        import drone_gcs
        from protocol.ros2_map_listener import ROS2MapListener
        from core.settings import load_settings, apply_overrides
        drone_gcs.DroneGCSMainWindow._connect_to_endpoint = lambda self, *a, **k: None
        ROS2MapListener.start = lambda self: None
        drone_gcs.VideoFeedWidget.ensure_started = lambda self: None
        cls.win = drone_gcs.DroneGCSMainWindow(settings=apply_overrides(load_settings()))
        cls.win.resize(1400, 850)
        cls.win.show()
        cls.win.worker = mock.Mock()
        cls.win.worker.isRunning.return_value = True
        cls.win.stack.setCurrentWidget(cls.win.page_slam)
        app.processEvents()

    @classmethod
    def tearDownClass(cls):
        cls.win.worker = None
        cls.win.close()

    def _telemetry(self, armed, mode="STABILIZED"):
        t = self.win.last_telemetry
        t.connected, t.armed, t.flight_mode = True, armed, mode
        self.win._on_telemetry_updated(t)
        app.processEvents()

    def test_arm_goes_through_confirm_bar_then_worker(self):
        self._telemetry(False)
        bar = self.win.page_slam.flight_bar
        self.assertTrue(bar.btn_arm.isEnabled())
        self.win.worker.reset_mock()
        bar.btn_arm.click()
        self.assertTrue(self.win.confirm_bar.isVisible(), "ARM must ask for confirmation first")
        self.win.worker.arm.assert_not_called()
        self.win.confirm_bar.confirmed.emit("arm", 0.0)
        self.win.worker.arm.assert_called_once_with(force=False)

    def test_disarm_is_a_single_click_like_the_control_tab(self):
        self._telemetry(True)
        bar = self.win.page_slam.flight_bar
        self.assertTrue(bar.btn_disarm.isEnabled())
        self.win.worker.reset_mock()
        self.win.confirm_bar.hide()
        bar.btn_disarm.click()
        self.assertFalse(self.win.confirm_bar.isVisible(), "DISARM must not ask for a slide")
        self.win.worker.disarm.assert_called_once()

    def test_mode_set_reaches_worker(self):
        self._telemetry(False)
        bar = self.win.page_slam.flight_bar
        self.win.worker.reset_mock()
        bar.combo_mode.setCurrentText("AUTO.LOITER")
        bar.btn_set_mode.click()
        self.win.worker.set_mode.assert_called_once_with("AUTO.LOITER")


if __name__ == "__main__":
    unittest.main()
