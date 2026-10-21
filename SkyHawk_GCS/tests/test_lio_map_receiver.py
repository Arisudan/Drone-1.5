"""Point-LIO 3D map stream (Jetson tcp 5766 -> protocol/lio_map_receiver.py -> RViz2).

Hermetic: the wire parser and the layout selection, no ROS or sockets. The live path
(streamer -> receiver -> /lio/* topics, including a Point-LIO restart clearing the map)
was exercised end to end on 2026-10-15 against jetson/lio_map_stream.py.
"""
import struct
import unittest

import _env  # noqa: F401

from protocol.lio_map_receiver import (MAGIC, T_HELLO, T_MAP_ADD, T_POSE, T_RESET,
                                       parse_frames)


def frame(ftype, payload=b""):
    return MAGIC + struct.pack("<BI", ftype, len(payload)) + payload


class ParseFramesTest(unittest.TestCase):
    def test_whole_frames_in_order(self):
        buf = bytearray(frame(T_HELLO, b"{}") + frame(T_RESET) + frame(T_POSE, b"x" * 36))
        self.assertEqual([t for t, _ in parse_frames(buf)], [T_HELLO, T_RESET, T_POSE])
        self.assertEqual(len(buf), 0)

    def test_a_split_frame_waits_for_the_rest(self):
        data = frame(T_MAP_ADD, b"\x00" * 120)
        buf = bytearray(data[:50])
        self.assertEqual(parse_frames(buf), [])
        buf += data[50:]
        out = parse_frames(buf)
        self.assertEqual(len(out), 1)
        self.assertEqual(len(out[0][1]), 120)

    def test_garbage_before_a_frame_is_skipped(self):
        buf = bytearray(b"\x01\x02noise" + frame(T_RESET))
        self.assertEqual([t for t, _ in parse_frames(buf)], [T_RESET])

    def test_byte_at_a_time(self):
        data = frame(T_HELLO, b'{"frame": "lio_world"}') + frame(T_RESET)
        buf, got = bytearray(), []
        for b in data:
            buf.append(b)
            got += parse_frames(buf)
        self.assertEqual([t for t, _ in got], [T_HELLO, T_RESET])


class LayoutSelectionTest(unittest.TestCase):
    def test_pointlio_is_the_default_3d_layout(self):
        from core.settings import default_rviz_config
        self.assertTrue(default_rviz_config().endswith("pointlio_3d.rviz"))

    def test_only_pointlio_layouts_start_the_receiver(self):
        try:
            from ui.rviz_embed_widget import is_lio_config
        except Exception as e:                       # PyQt5 absent in the hermetic job
            self.skipTest(str(e))
        self.assertTrue(is_lio_config("/x/config/pointlio_3d.rviz"))
        self.assertFalse(is_lio_config("/x/config/rtabmap_drone.rviz"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
