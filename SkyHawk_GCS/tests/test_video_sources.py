"""Tests for network-stream handling in the FPV video widget.

Two behaviours are covered, both found by inspection and then measured:

  * RTSP streams are opened with explicit FFmpeg options. Without a timeout,
    cv2.VideoCapture() against a host that drops packets (drone powered down,
    wrong subnet) blocks for 30 s - measured - which silently turns the
    widget's 2 s reconnect cadence into a 30 s one.
  * The drone feed and a hand-typed custom URL are remembered separately.
    A single shared field meant switching Network preset overwrote a typed
    RTSP address and reconnected to MJPEG with no warning.
"""

import os
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from PyQt5.QtWidgets import QApplication

from ui.video_feed_widget import (
    NETWORK_CAPTURE_OPTIONS, VideoCaptureThread, VideoFeedWidget,
    is_network_stream,
)

app = QApplication.instance() or QApplication([])


class SchemeDetectionTest(unittest.TestCase):
    def test_streaming_schemes_are_network(self):
        for url in ("rtsp://1.2.3.4:554/s", "RTSP://1.2.3.4/s",
                    "rtsps://1.2.3.4/s", "udp://239.0.0.1:5600",
                    "rtp://1.2.3.4:5600", "rtmp://a/b", "tcp://1.2.3.4:5000"):
            self.assertTrue(is_network_stream(url), url)

    def test_http_mjpeg_is_left_alone(self):
        # The drone's own feed is plain HTTP and works without RTSP tuning;
        # applying stimeout there would change its reconnect behaviour for
        # no benefit.
        self.assertFalse(is_network_stream("http://172.16.101.84:8080/video"))

    def test_local_devices_and_synthetic_source_are_not_network(self):
        self.assertFalse(is_network_stream(0))
        self.assertFalse(is_network_stream(1))
        self.assertFalse(is_network_stream("TEST_PATTERN"))


class CaptureOptionsTest(unittest.TestCase):
    def setUp(self):
        os.environ.pop("OPENCV_FFMPEG_CAPTURE_OPTIONS", None)

    tearDown = setUp

    def test_options_carry_a_timeout_and_tcp_transport(self):
        self.assertIn("rtsp_transport;tcp", NETWORK_CAPTURE_OPTIONS)
        # Both spellings: `timeout` is current FFmpeg, `stimeout` is pre-5.x.
        # Measured on this build, `stimeout` alone left the 30 s block in
        # place while `timeout` cut it to 5 s - so the modern name must be
        # present, not just the legacy one.
        self.assertIn("timeout;", NETWORK_CAPTURE_OPTIONS)
        parts = dict(p.split(";", 1) for p in NETWORK_CAPTURE_OPTIONS.split("|"))
        self.assertIn("timeout", parts, "the current FFmpeg option name is required")
        timeout_us = int(parts["timeout"])
        # Microseconds. Under a second would break a slow-but-working link;
        # over ~10 s defeats the purpose.
        self.assertGreaterEqual(timeout_us, 1_000_000)
        self.assertLessEqual(timeout_us, 10_000_000)

    def test_rtsp_source_sets_the_options(self):
        t = VideoCaptureThread("rtsp://10.0.0.9:554/live")
        t._apply_capture_options()
        self.assertEqual(os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS"),
                         NETWORK_CAPTURE_OPTIONS)

    def test_switching_away_from_rtsp_clears_them(self):
        t = VideoCaptureThread("rtsp://10.0.0.9:554/live")
        t._apply_capture_options()
        t.set_source("http://10.0.0.9:8080/video")
        t._apply_capture_options()
        self.assertIsNone(os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS"),
                          "MJPEG must not inherit the RTSP timeout")

    def test_usb_device_clears_them(self):
        t = VideoCaptureThread("rtsp://10.0.0.9:554/live")
        t._apply_capture_options()
        t.set_source(0)
        t._apply_capture_options()
        self.assertIsNone(os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS"))


class UrlMemoryTest(unittest.TestCase):
    def setUp(self):
        self.w = VideoFeedWidget()

    def test_custom_url_survives_a_network_preset_change(self):
        self.w.combo_source.setCurrentIndex(self.w.SRC_CUSTOM)
        self.w.txt_url.setText("rtsp://10.9.9.9:554/front")
        self.w._on_url_edited()

        # Operator switches Network preset in the header.
        self.w.set_stream_host("192.168.4.1")

        self.assertEqual(self.w.txt_url.text(), "rtsp://10.9.9.9:554/front",
                         "the preset must not overwrite a typed RTSP URL")
        self.assertEqual(self.w._resolve_source(self.w.SRC_CUSTOM),
                         "rtsp://10.9.9.9:554/front")

    def test_drone_url_does_follow_the_preset(self):
        self.w.combo_source.setCurrentIndex(self.w.SRC_DRONE_FPV)
        self.w.set_stream_host("192.168.4.1")
        self.assertEqual(self.w.txt_url.text(), "http://192.168.4.1:8080/video")

    def test_each_source_keeps_its_own_url_across_switches(self):
        self.w.combo_source.setCurrentIndex(self.w.SRC_CUSTOM)
        self.w.txt_url.setText("rtsp://cam.local:554/a")
        self.w._on_url_edited()

        self.w.combo_source.setCurrentIndex(self.w.SRC_DRONE_FPV)
        self.w._on_source_changed(self.w.SRC_DRONE_FPV)
        self.assertTrue(self.w.txt_url.text().startswith("http://"))

        self.w.combo_source.setCurrentIndex(self.w.SRC_CUSTOM)
        self.w._on_source_changed(self.w.SRC_CUSTOM)
        self.assertEqual(self.w.txt_url.text(), "rtsp://cam.local:554/a")

    def test_url_field_is_disabled_for_device_sources(self):
        self.w._on_source_changed(self.w.SRC_CAM0)
        self.assertFalse(self.w.txt_url.isEnabled())
        self.w._on_source_changed(self.w.SRC_CUSTOM)
        self.assertTrue(self.w.txt_url.isEnabled())


if __name__ == "__main__":
    unittest.main()
