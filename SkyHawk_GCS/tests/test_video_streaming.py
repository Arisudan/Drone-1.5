"""Streaming latency behaviour: the Radxa streamer's client sockets and chunking,
and the GCS capture thread's newest-frame-only hand-off. Loopback only."""

import os
import socket
import sys
import threading
import time
import unittest

import numpy as np
from http.server import ThreadingHTTPServer

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import d435i_video_streamer as S  # noqa: E402


def F(k):
    return np.full((4, 4, 3), k, np.uint8)


def payload(frame_id, size=40_000):
    return frame_id.to_bytes(4, "big") + bytes(size - 4)


class ChunkTest(unittest.TestCase):
    def test_a_frame_is_one_buffer_with_the_right_framing(self):
        j = payload(7)
        c = S.frame_chunk(j)
        self.assertTrue(c.startswith(b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: 40000\r\n\r\n"))
        self.assertTrue(c.endswith(j + b"\r\n"))
        self.assertIsInstance(c, bytes)

    def test_socket_tuning_sets_nodelay_small_sendbuf_and_a_timeout(self):
        srv = socket.socket()
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        c = socket.create_connection(srv.getsockname())
        conn, _ = srv.accept()
        try:
            S.tune_client_socket(conn)
            self.assertEqual(conn.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY), 1)
            # Linux reports double the requested value; it must be far below the ~2.5 MB default.
            self.assertLessEqual(conn.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),
                                 2 * S.CLIENT_SNDBUF_BYTES + 4096)
            self.assertEqual(conn.gettimeout(), S.CLIENT_SEND_TIMEOUT_S)
        finally:
            conn.close(); c.close(); srv.close()


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.hub = S.FrameHub()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), S.make_handler(self.hub, None))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def connect(self, rcvbuf=None):
        s = socket.socket()
        if rcvbuf:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        s.connect(("127.0.0.1", self.port))
        s.sendall(b"GET /video HTTP/1.1\r\nHost: x\r\n\r\n")
        s.settimeout(3)
        return s

    @staticmethod
    def read_frames(sock, want_idle=0.6):
        """Parse multipart parts until the stream is idle; return their ids."""
        buf, ids = b"", []
        sock.settimeout(want_idle)
        try:
            while True:
                d = sock.recv(65536)
                if not d:
                    break
                buf += d
        except socket.timeout:
            pass
        i = buf.find(b"--FRAME")
        while i >= 0:
            h = buf.find(b"\r\n\r\n", i)
            if h < 0:
                break
            hdr = buf[i:h].decode(errors="ignore")
            n = int(hdr.split("Content-Length:")[1].split()[0])
            body = buf[h + 4:h + 4 + n]
            if len(body) < n:
                break
            ids.append(int.from_bytes(body[:4], "big"))
            i = buf.find(b"--FRAME", h + 4 + n)
        return ids

    def test_a_connected_client_receives_frames_in_order(self):
        c = self.connect()
        time.sleep(0.2)
        for k in range(1, 6):
            self.hub.publish(payload(k))
            time.sleep(0.05)
        ids = self.read_frames(c)
        c.close()
        self.assertEqual(ids, sorted(ids))
        self.assertIn(5, ids)

    def test_a_stalled_client_gets_the_newest_frame_not_a_backlog(self):
        c = self.connect(rcvbuf=4096)          # tiny receive window: the client "can't keep up"
        time.sleep(0.2)
        total = 30
        for k in range(1, total + 1):
            self.hub.publish(payload(k, size=60_000))    # 1.8 MB offered while the client reads nothing
            time.sleep(0.01)
        time.sleep(0.3)
        ids = self.read_frames(c, want_idle=1.0)
        c.close()
        self.assertTrue(ids)
        self.assertLess(len(ids), total // 2, f"received {len(ids)} of {total}: stale frames were queued")
        self.assertEqual(ids[-1], total)       # and the last one it gets is the newest


class CaptureThreadHandOffTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def make(self):
        from ui.video_feed_widget import VideoCaptureThread
        t = VideoCaptureThread("http://x/video")
        got = []
        t.frame_ready.connect(lambda f: got.append(f))
        return t, got

    def test_while_the_gui_is_busy_newer_frames_are_skipped_not_queued(self):
        t, got = self.make()
        self.assertTrue(t._emit_real_frame(F(1)))
        for k in range(2, 8):
            self.assertFalse(t._emit_real_frame(F(k)))
        self.assertEqual([int(g[0, 0, 0]) for g in got], [1])
        self.assertEqual(t.frames_skipped, 6)

    def test_once_the_gui_has_consumed_the_next_frame_goes_through(self):
        t, got = self.make()
        t._emit_real_frame(F(1))
        t.frame_consumed()
        self.assertTrue(t._emit_real_frame(F(2)))
        self.assertEqual([int(g[0, 0, 0]) for g in got], [1, 2])

    def test_a_gui_that_never_answers_cannot_freeze_the_feed(self):
        t, got = self.make()
        t.PENDING_TIMEOUT_S = 0.05
        t._emit_real_frame(F(1))
        time.sleep(0.08)
        self.assertTrue(t._emit_real_frame(F(2)))

    def test_widget_acknowledges_each_displayed_frame(self):
        from ui.video_feed_widget import VideoFeedWidget
        w = VideoFeedWidget()
        t, _ = self.make()
        w.cap_thread = t
        t._emit_real_frame(F(1))
        self.assertGreater(t._pending_since, 0)
        w._last_info = (None, True)
        w._on_frame_ready(np.zeros((48, 64, 3), np.uint8))
        self.assertEqual(t._pending_since, 0.0)
        w.cap_thread = None
        w.deleteLater()


if __name__ == "__main__":
    unittest.main()
