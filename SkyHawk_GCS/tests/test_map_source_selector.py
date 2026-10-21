"""Tactical SLAM 2D map source selector: Point-LIO grid (tcp 5765) vs graph map (tcp 5767).

Hermetic: two stub DMAP servers stand in for the Jetson's lio_grid_map.py and graph_grid_gcs.py;
each sends a grid of its own size so the test can tell which one the listener is reading.
"""

import json
import socket
import struct
import threading
import time
import unittest
import zlib

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

import numpy as np
from PyQt5.QtWidgets import QApplication

from protocol.ros2_map_listener import ROS2MapListener
from ui.slam_map_widget import MAP_SOURCES, SLAMMapWidget

APP = QApplication.instance() or QApplication([])


def dmap(width, height):
    grid = np.zeros((height, width), dtype=np.int8)
    hdr = json.dumps({"seq": 1, "source": "map_raw", "width": width, "height": height,
                      "resolution": 0.05, "origin_x": 0.0, "origin_y": 0.0,
                      "timestamp": time.time()}).encode()
    body = zlib.compress(grid.tobytes())
    return b"DMAP" + struct.pack("!I", len(hdr)) + hdr + struct.pack("!I", len(body)) + body


class StubMapServer:
    """Sends one DMAP frame every 0.2 s to whoever connects."""

    def __init__(self, width):
        self.width = width
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(4)
        self.port = self.srv.getsockname()[1]
        self.running = True
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        while self.running:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._feed, args=(c,), daemon=True).start()

    def _feed(self, c):
        try:
            while self.running:
                c.sendall(dmap(self.width, 4))
                time.sleep(0.2)
        except OSError:
            pass
        finally:
            c.close()

    def close(self):
        self.running = False
        self.srv.close()


def wait_until(cond, timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        APP.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    return False


class ListenerSwitchTest(unittest.TestCase):
    def setUp(self):
        self.grid_srv, self.graph_srv = StubMapServer(10), StubMapServer(20)
        self.lst = ROS2MapListener("127.0.0.1", self.grid_srv.port)
        self.widths = []
        self.lst.map_received.connect(lambda g, *a: self.widths.append(g.shape[0]))
        self.lst.start()

    def tearDown(self):
        self.lst.stop()
        self.lst.wait(3000)
        self.grid_srv.close()
        self.graph_srv.close()

    def test_set_tcp_port_reconnects_to_the_other_map(self):
        self.assertTrue(wait_until(lambda: 10 in self.widths))
        self.lst.set_tcp_port(self.graph_srv.port)
        self.assertEqual(self.lst.tcp_port, self.graph_srv.port)
        self.widths.clear()
        self.assertTrue(wait_until(lambda: 20 in self.widths, timeout=6.0))
        n = len(self.widths)
        self.assertTrue(wait_until(lambda: len(self.widths) >= n + 3))
        self.assertNotIn(10, self.widths[n:])      # the old source is no longer read


class WidgetSelectorTest(unittest.TestCase):
    def test_sources_and_signal(self):
        self.assertEqual([p for _, p, _ in MAP_SOURCES], [5765, 5767])
        w = SLAMMapWidget()
        got = []
        w.map_source_changed.connect(got.append)
        w.set_map_source_port(5767)                 # programmatic: no signal
        self.assertEqual(w.combo_map_source.currentData(), 5767)
        self.assertEqual(got, [])
        w.canvas.occupancy_grid = np.zeros((3, 3))
        w.combo_map_source.setCurrentIndex(0)      # user pick: signal + old grid blanked
        self.assertEqual(got, [5765])
        self.assertIsNone(w.canvas.occupancy_grid)
        w.close()
        w.deleteLater()


class ResetFollowsSourceTest(unittest.TestCase):
    def test_reset_map_goes_to_the_selected_port(self):
        import controllers.mission_control as mc
        src = open(mc.__file__).read()
        self.assertIn("tcp_port=self.map_listener.tcp_port", src)
        self.assertNotIn("tcp_port=5765", src)


if __name__ == "__main__":
    unittest.main()
