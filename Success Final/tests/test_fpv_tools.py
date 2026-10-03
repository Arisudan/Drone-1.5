"""FPV tab: snapshot, recording, telemetry overlay and the fps/latency graph."""

import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication

from core.video_recorder import VideoRecorder, save_snapshot, timestamped


def frame(w=64, h=48, v=120):
    return np.full((h, w, 3), v, np.uint8)


class RecorderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_records_frames_to_a_playable_avi(self):
        r = VideoRecorder(fps=10)
        path = r.start(self.dir)
        self.assertTrue(r.recording)
        for i in range(8):
            self.assertTrue(r.write(frame(v=20 * i)))
        self.assertEqual(r.stop(), path)
        cap = cv2.VideoCapture(str(path))
        self.assertEqual(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), 8)
        cap.release()

    def test_a_different_frame_size_is_resized_not_dropped(self):
        r = VideoRecorder()
        path = r.start(self.dir)
        r.write(frame(64, 48))
        self.assertTrue(r.write(frame(32, 24)))
        r.stop()
        cap = cv2.VideoCapture(str(path))
        self.assertEqual((int(cap.get(3)), int(cap.get(4))), (64, 48))
        self.assertEqual(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), 2)
        cap.release()

    def test_no_frames_means_no_file_and_none(self):
        r = VideoRecorder()
        r.start(self.dir)
        self.assertIsNone(r.stop())
        self.assertEqual(list(self.dir.glob("*.avi")), [])
        self.assertFalse(r.recording)

    def test_write_when_not_recording_is_ignored(self):
        self.assertFalse(VideoRecorder().write(frame()))

    def test_snapshot_png_and_unique_names(self):
        p = save_snapshot(frame(), self.dir)
        self.assertTrue(p.is_file())
        self.assertEqual(cv2.imread(str(p)).shape, (48, 64, 3))
        self.assertIsNone(save_snapshot(None, self.dir))
        a = timestamped("x", "png", self.dir)
        a.write_bytes(b"")
        self.assertNotEqual(a, timestamped("x", "png", self.dir))


class WidgetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        os.environ["DRONE_GCS_HOME"] = self.tmp.name
        from ui.video_feed_widget import VideoFeedWidget
        self.w = VideoFeedWidget()
        self.w.resize(900, 600)
        self.w.show()
        self.app.processEvents()

    def tearDown(self):
        self.w.close()
        self.w.deleteLater()
        os.environ.pop("DRONE_GCS_HOME", None)
        self.tmp.cleanup()

    def feed(self, f=None, real=True):
        self.w._last_info = (None, real)
        self.w._on_frame_ready(frame() if f is None else f)

    def test_extras_are_off_by_default(self):
        self.assertFalse(self.w.btn_overlay.isChecked())
        self.assertFalse(self.w.btn_graph.isChecked())
        self.assertFalse(self.w.graph.isVisibleTo(self.w))

    def test_snapshot_without_a_frame_says_so(self):
        self.assertIsNone(self.w.take_snapshot())
        self.assertIn("no live frame", self.w.lbl_note.text())

    def test_snapshot_saves_the_last_real_frame(self):
        self.feed()
        p = self.w.take_snapshot()
        self.assertTrue(p.is_file())
        self.assertIn("saved", self.w.lbl_note.text())

    def test_placeholder_frames_are_not_recorded_or_snapshotted(self):
        self.w.toggle_recording()
        self.feed(real=False)
        self.assertEqual(self.w.recorder.frames, 0)
        self.assertIsNone(self.w.take_snapshot())
        self.feed(real=True)
        self.assertEqual(self.w.recorder.frames, 1)
        self.w.toggle_recording()
        self.assertEqual(self.w.btn_record.text(), "Record")

    def test_record_button_and_indicator(self):
        self.w.toggle_recording()
        self.assertEqual(self.w.btn_record.text(), "Stop rec")
        self.assertIn("REC", self.w.lbl_rec.text())
        self.feed()
        path = self.w.stop_recording()
        self.assertTrue(path.is_file())
        self.assertEqual(self.w.lbl_rec.text(), "")

    def test_overlay_text_and_never_in_the_recording(self):
        self.assertEqual(self.w.overlay_lines(), [])
        self.w.set_telemetry(SimpleNamespace(flight_mode="POSCTL", armed=True, altitude=1.25,
                                             ground_speed=0.4, battery_percent=88, battery_voltage=16.2))
        lines = self.w.overlay_lines()
        self.assertIn("POSCTL", lines[0])
        self.assertIn("ARMED", lines[0])
        self.assertIn("1.25", lines[1])
        self.w.btn_overlay.setChecked(True)
        self.w.recorder.start()
        raw = frame()
        before = raw.copy()
        self.feed(raw)
        self.assertTrue(np.array_equal(raw, before))     # the frame itself is untouched
        self.w.stop_recording()

    def test_graph_collects_history_only_when_shown(self):
        self.w._refresh_health()
        self.assertEqual(self.w.graph.fps, [])
        self.w.btn_graph.setChecked(True)
        self.w._refresh_health()
        self.w._refresh_health()
        self.assertEqual(len(self.w.graph.fps), 2)
        self.w.btn_graph.setChecked(False)
        self.assertFalse(self.w.graph.isVisibleTo(self.w))

    def test_graph_window_is_sixty_seconds(self):
        for _ in range(300):
            self.w.graph.push(20.0, 50.0)
        self.assertEqual(len(self.w.graph.fps), 120)


if __name__ == "__main__":
    unittest.main()
