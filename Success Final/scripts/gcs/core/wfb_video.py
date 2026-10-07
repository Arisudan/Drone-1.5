"""
================================================================================
MODULE: wfb_video.py
PURPOSE: Feed the wfb-ng radio video link into the GCS's OpenCV capture
================================================================================

ARCHITECTURE & CONTEXT:
  * Runs On:       Laptop Ground Station
  * Upstream:      wfb-ng ground side (wifibroadcast@gs), which delivers the air
                   unit's RTP H.264 (payload 96) to udp 127.0.0.1:<rtp_port>
                   (5600 by default, see /etc/wifibroadcast.cfg [gs_video]).
  * Downstream:    ui/video_feed_widget.VideoCaptureThread opens
                   udp://127.0.0.1:<ts_port> with OpenCV/FFmpeg.

WHY A HELPER PROCESS:
  The pip OpenCV this station uses has FFmpeg but no GStreamer, and its FFmpeg
  cannot open raw RTP: a .sdp source fails with "Protocol 'rtp' not on
  whitelist" because OpenCV does not pass protocol_whitelist down to the RTP
  layer (tested with OpenCV 4.11). The system GStreamer therefore re-wraps the
  RTP H.264 into MPEG-TS on a local UDP port - no re-encode, just a container
  change - which FFmpeg opens natively. Measured on the Jetson D435i feed:
  720p, 28.6 fps, first frame 0.5 s after open.

FAILSAFES:
  * The helper is started with PR_SET_PDEATHSIG, so it dies with the GCS even
    on a crash, and never outlives the video source that started it.
  * Only one process may bind the RTP port: stop link_ctl.sh's rtp_monitor
    relay first, or point rtp_port at the relay's forward port (5601).

USAGE:
  bridge = WfbVideoBridge(rtp_port=5600, ts_port=5610)
  url = bridge.start()        # -> "udp://127.0.0.1:5610?overrun_nonfatal=1&fifo_size=5000000"
  ...
  bridge.stop()
================================================================================
"""

from __future__ import annotations

import ctypes
import logging
import shutil
import signal
import subprocess
from typing import Optional

log = logging.getLogger(__name__)

_PR_SET_PDEATHSIG = 1


def _die_with_parent() -> None:  # runs in the child between fork and exec
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(_PR_SET_PDEATHSIG, signal.SIGTERM)
    except OSError:
        pass


class WfbVideoBridge:
    """Owns the gst-launch helper that turns wfb-ng's RTP H.264 into local MPEG-TS."""

    def __init__(self, rtp_port: int = 5600, ts_port: int = 5610, jitter_ms: int = 30):
        self.rtp_port = rtp_port
        self.ts_port = ts_port
        self.jitter_ms = jitter_ms
        self._proc: Optional[subprocess.Popen] = None

    @property
    def url(self) -> str:
        return f"udp://127.0.0.1:{self.ts_port}?overrun_nonfatal=1&fifo_size=5000000"

    @staticmethod
    def available() -> bool:
        return shutil.which("gst-launch-1.0") is not None

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self) -> str:
        """Start the helper if it is not running; return the URL OpenCV should open."""
        if self.is_running():
            return self.url
        gst = shutil.which("gst-launch-1.0")
        if not gst:
            raise RuntimeError("gst-launch-1.0 not found (install gstreamer1.0-tools and -plugins-good/-bad)")
        cmd = [
            gst, "-q",
            "udpsrc", f"port={self.rtp_port}",
            "caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000",
            "!", "rtpjitterbuffer", f"latency={self.jitter_ms}",
            "!", "rtph264depay", "!", "h264parse", "config-interval=-1",
            "!", "mpegtsmux", "alignment=7",
            "!", "udpsink", "host=127.0.0.1", f"port={self.ts_port}", "sync=false",
        ]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                      stderr=subprocess.DEVNULL, preexec_fn=_die_with_parent)
        log.info("wfb-ng video bridge started: rtp :%d -> ts :%d (pid %d)",
                 self.rtp_port, self.ts_port, self._proc.pid)
        return self.url

    def stop(self) -> None:
        if self._proc is None:
            return
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._proc = None
