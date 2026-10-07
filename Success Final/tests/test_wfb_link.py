"""Tests for the wfb-ng radio link option (telemetry + video over wfb-ng).

  * MAVLink: wfb-ng's ground side *sends* the air unit's MAVLink to
    127.0.0.1:14550, so the GCS must listen (protocol "udpin") rather than
    connect out. Settings, the CLI and the header must all accept it.
  * Video: the pip OpenCV here has no GStreamer and its FFmpeg cannot open raw
    RTP (an .sdp source fails "Protocol 'rtp' not on whitelist", measured), so
    core/wfb_video.py re-wraps wfb-ng's RTP H.264 as MPEG-TS on a local UDP
    port. The widget must start that helper only for the wfb-ng source and
    stop it when the source changes.
"""

import unittest
from unittest import mock

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from PyQt5.QtWidgets import QApplication

from core.settings import ConnectionConfig, VideoConfig
from core.wfb_video import WfbVideoBridge
from ui.top_status_strip import TopStatusStrip, WFB_NETWORK
from ui.video_feed_widget import VideoFeedWidget, is_network_stream

app = QApplication.instance() or QApplication([])


class SettingsTest(unittest.TestCase):
    def test_udpin_is_a_valid_protocol(self):
        ConnectionConfig(protocol="udpin").validate()

    def test_unknown_protocol_still_rejected(self):
        with self.assertRaises(ValueError):
            ConnectionConfig(protocol="serial").validate()

    def test_wfb_video_ports_default_and_must_differ(self):
        v = VideoConfig()
        self.assertEqual((v.wfb_rtp_port, v.wfb_ts_port), (5600, 5610))
        v.validate()
        with self.assertRaises(ValueError):
            VideoConfig(wfb_rtp_port=5600, wfb_ts_port=5600).validate()


class BridgeTest(unittest.TestCase):
    def test_url_is_a_network_stream_on_the_ts_port(self):
        b = WfbVideoBridge(rtp_port=5601, ts_port=5620)
        self.assertTrue(b.url.startswith("udp://127.0.0.1:5620"))
        self.assertTrue(is_network_stream(b.url))

    def test_pipeline_listens_on_rtp_port_and_sends_ts(self):
        with mock.patch("core.wfb_video.shutil.which", return_value="/usr/bin/gst-launch-1.0"), \
             mock.patch("core.wfb_video.subprocess.Popen") as popen:
            popen.return_value.poll.return_value = None
            WfbVideoBridge(rtp_port=5601, ts_port=5620).start()
        cmd = popen.call_args[0][0]
        self.assertIn("port=5601", cmd)
        self.assertIn("rtph264depay", cmd)
        self.assertIn("mpegtsmux", cmd)
        self.assertIn("port=5620", cmd)

    def test_missing_gstreamer_is_reported(self):
        with mock.patch("core.wfb_video.shutil.which", return_value=None):
            with self.assertRaises(RuntimeError):
                WfbVideoBridge().start()


class HeaderPresetTest(unittest.TestCase):
    def setUp(self):
        self.strip = TopStatusStrip()

    def _pick(self, name):
        self.strip.network_combo.setCurrentIndex(self.strip.network_combo.findText(name))

    def test_wfb_preset_selects_listen_protocol_and_port(self):
        self._pick(WFB_NETWORK)
        self.assertTrue(self.strip.is_wfb_selected())
        self.assertEqual(self.strip.proto_combo.currentData(), "udpin")
        self.assertEqual(self.strip.port_input.text(), "14550")

    def test_leaving_wfb_restores_plain_udp(self):
        self._pick(WFB_NETWORK)
        self._pick("HTIC_RND")
        self.assertEqual(self.strip.proto_combo.currentData(), "udp")


class VideoSourceTest(unittest.TestCase):
    def setUp(self):
        self.w = VideoFeedWidget()

    def test_wfb_source_starts_the_bridge_and_opens_its_url(self):
        with mock.patch.object(self.w.wfb, "start", return_value="udp://127.0.0.1:5610") as start:
            self.assertEqual(self.w._resolve_source(VideoFeedWidget.SRC_WFB), "udp://127.0.0.1:5610")
        start.assert_called_once()

    def test_other_sources_do_not_start_the_bridge(self):
        with mock.patch.object(self.w.wfb, "start") as start:
            for idx in (VideoFeedWidget.SRC_DRONE_FPV, VideoFeedWidget.SRC_CUSTOM):
                self.w._resolve_source(idx)
        start.assert_not_called()

    def test_switching_away_from_wfb_stops_the_bridge(self):
        self.w.select_source(VideoFeedWidget.SRC_WFB)
        with mock.patch.object(self.w.wfb, "stop") as stop:
            self.w.select_source(VideoFeedWidget.SRC_DRONE_FPV)
        stop.assert_called()

    def test_existing_source_indices_unchanged(self):
        self.assertEqual((VideoFeedWidget.SRC_DRONE_FPV, VideoFeedWidget.SRC_CAM0,
                          VideoFeedWidget.SRC_CAM1, VideoFeedWidget.SRC_CUSTOM), (0, 1, 2, 3))


if __name__ == "__main__":
    unittest.main()
