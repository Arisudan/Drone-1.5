"""Import-level smoke tests for the GCS modules.

Not hermetic - these need PyQt5, so they are skipped where it is absent (CI).
Their job is narrow but real: catch a module that no longer imports, which is
the failure mode a refactor produces most often and which no amount of unit
testing of the other modules would notice.

`drone_gcs.py` is deliberately NOT imported here. Its module body re-execs the
interpreter (os.execv) to inject ROS 2 library paths, which would restart the
test runner mid-suite. The widgets and workers it wires together are all
covered below instead.
"""

import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

try:
    import PyQt5  # noqa: F401
    HAVE_QT = True
except Exception:
    HAVE_QT = False


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class ImportTest(unittest.TestCase):
    def test_core_modules_import(self):
        import core.execution_tracker  # noqa: F401
        import core.health             # noqa: F401
        import core.path_planner       # noqa: F401
        import core.telemetry          # noqa: F401

    def test_protocol_modules_import(self):
        import protocol.mavlink_worker     # noqa: F401
        import protocol.ros2_map_listener  # noqa: F401

    def test_ui_modules_import(self):
        import ui.cli_console      # noqa: F401
        import ui.hud_widget       # noqa: F401
        import ui.motor_widget     # noqa: F401
        import ui.sidebar_nav      # noqa: F401
        import ui.slam_map_widget  # noqa: F401
        import ui.styles           # noqa: F401
        import ui.toast            # noqa: F401
        import ui.top_status_strip # noqa: F401
        import ui.video_feed_widget  # noqa: F401


@unittest.skipUnless(HAVE_QT, "PyQt5 not installed")
class MapListenerHealthTest(unittest.TestCase):
    """The map listener must register itself for liveness on construction."""

    def test_listener_registers_a_health_component(self):
        from core.health import EngineStatus, get_registry
        from protocol.ros2_map_listener import ROS2MapListener

        listener = ROS2MapListener(tcp_host="127.0.0.1", tcp_port=59999)
        try:
            self.assertIs(get_registry().get("MapListener"), listener.health)
            snap = listener.health.snapshot()
            self.assertEqual(snap.status, EngineStatus.UNINITIALIZED)
            self.assertEqual(snap.beats, 0)
        finally:
            get_registry().unregister("MapListener")

    def test_thresholds_are_configurable_per_deployment(self):
        from protocol.ros2_map_listener import ROS2MapListener
        from core.health import get_registry

        listener = ROS2MapListener(tcp_host="127.0.0.1", tcp_port=59999,
                                   stall_after_s=4.0, degrade_after_s=1.0)
        try:
            self.assertEqual(listener.health.stall_after_s, 4.0)
            self.assertEqual(listener.health.degrade_after_s, 1.0)
        finally:
            get_registry().unregister("MapListener")


class DiagnosticsImportTest(unittest.TestCase):
    """map_eval must stay free of Qt/ROS so it runs anywhere, including CI."""

    def test_map_eval_imports_without_qt_or_ros(self):
        import map_eval  # noqa: F401

    def test_map_eval_declares_the_documented_thresholds(self):
        import map_eval
        for key in ("scale_error_pct", "wall_rms_cells", "squareness_err_deg",
                    "completeness_pct", "loop_gap_pct", "yaw_drift_deg",
                    "vo_lost_pct"):
            self.assertIn(key, map_eval.THRESHOLDS)


if __name__ == "__main__":
    unittest.main()
