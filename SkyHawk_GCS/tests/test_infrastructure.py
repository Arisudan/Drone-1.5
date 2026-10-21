"""Settings schema versioning, UI stall monitor, diagnostics bundle.

Hermetic: no PyQt, no hardware. Time is injected, files live in temp dirs.
"""

import json
import unittest
import zipfile
from pathlib import Path
from tempfile import TemporaryDirectory

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core import settings as S
from core.log_bundle import collect_files, create_bundle, default_bundle_name
from core.ui_stall import UiStallMonitor


class SettingsSchemaTest(unittest.TestCase):
    def test_save_writes_schema_version(self):
        with TemporaryDirectory() as d:
            p = S.save_settings(S.GCSSettings(), Path(d) / "settings.json")
            self.assertEqual(json.loads(p.read_text())[S.SCHEMA_KEY], S.SCHEMA_VERSION)

    def test_legacy_file_without_schema_still_loads(self):
        with TemporaryDirectory() as d:
            p = Path(d) / "settings.json"
            p.write_text(json.dumps({"actuator": {"host": "10.0.0.9"}}))
            self.assertEqual(S.load_settings(p).actuator.host, "10.0.0.9")

    def test_newer_file_passes_through_untouched(self):
        payload = {S.SCHEMA_KEY: S.SCHEMA_VERSION + 5, "future": {"x": 1}}
        self.assertEqual(S.migrate_payload(dict(payload)), payload)

    def test_migration_steps_run_in_order(self):
        old_ver, old_mig = S.SCHEMA_VERSION, dict(S._MIGRATIONS)
        try:
            S.SCHEMA_VERSION = 3
            S._MIGRATIONS[1] = lambda p: {**p, "a": 1}
            S._MIGRATIONS[2] = lambda p: {**p, "b": p["a"] + 1}
            out = S.migrate_payload({S.SCHEMA_KEY: 1})
            self.assertEqual((out["a"], out["b"], out[S.SCHEMA_KEY]), (1, 2, 3))
        finally:
            S.SCHEMA_VERSION = old_ver
            S._MIGRATIONS.clear()
            S._MIGRATIONS.update(old_mig)

    def test_garbage_schema_value_is_treated_as_v1(self):
        self.assertEqual(S.migrate_payload({S.SCHEMA_KEY: "x"}), {S.SCHEMA_KEY: "x"})


class UiStallMonitorTest(unittest.TestCase):
    def setUp(self):
        self.t = 0.0
        self.m = UiStallMonitor(stall_after_s=0.25, cooldown_s=5.0, clock=lambda: self.t)

    def _tick(self, dt):
        self.t += dt
        return self.m.tick()

    def test_first_tick_never_stalls(self):
        self.assertIsNone(self.m.tick())

    def test_normal_jitter_is_ignored(self):
        self.m.tick()
        for _ in range(30):
            self.assertIsNone(self._tick(0.04))
        self.assertEqual(self.m.stats.stalls, 0)

    def test_stall_reports_gap_once_per_cooldown(self):
        self.m.tick()
        self.assertAlmostEqual(self._tick(0.6), 0.6)
        self.assertIsNone(self._tick(0.6))          # inside cooldown: counted, not reported
        self.assertEqual(self.m.stats.stalls, 2)
        self.assertAlmostEqual(self.m.stats.worst_gap_s, 0.6)
        self._tick(6.0)                              # after cooldown it reports again
        self.assertEqual(self.m.stats.stalls, 3)

    def test_reset_clears_state(self):
        self.m.tick()
        self._tick(1.0)
        self.m.reset()
        self.assertEqual(self.m.stats.stalls, 0)
        self.assertIsNone(self.m.tick())


class LogBundleTest(unittest.TestCase):
    def test_bundle_contains_existing_files_and_manifest(self):
        with TemporaryDirectory() as d:
            base = Path(d)
            (base / "flights.jsonl").write_text("{}\n")
            (base / "settings.json").write_text("{}")
            (base / "gcs.log").write_text("x")
            (base / "ignored.txt").write_text("x")
            out = base / "b.zip"
            names = create_bundle(out, base)
            self.assertEqual(names, ["flights.jsonl", "settings.json", "gcs.log", "manifest.json"])
            with zipfile.ZipFile(out) as zf:
                man = json.loads(zf.read("manifest.json"))
            self.assertEqual(man["files"], ["flights.jsonl", "settings.json", "gcs.log"])

    def test_empty_directory_still_produces_a_manifest(self):
        with TemporaryDirectory() as d:
            self.assertEqual(collect_files(Path(d)), [])
            self.assertEqual(create_bundle(Path(d) / "b.zip", Path(d)), ["manifest.json"])

    def test_default_name_is_a_zip(self):
        self.assertTrue(default_bundle_name().startswith("gcs_bundle_"))
        self.assertTrue(default_bundle_name().endswith(".zip"))


if __name__ == "__main__":
    unittest.main()
