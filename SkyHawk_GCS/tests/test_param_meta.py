"""Parameter descriptions: the bundled PX4 metadata, QGC-style value formatting,
and the tool that builds the bundled file.

Hermetic: stdlib only (no Qt, no MAVLink, no PX4 checkout needed).
"""

import importlib.util
import json
import lzma
import os
import tempfile
import unittest
from pathlib import Path

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from core import param_meta as pm

ROOT = Path(__file__).resolve().parents[1]
BUILDER = ROOT / "scripts" / "diagnostics" / "build_param_metadata.py"
BOARD_PARAMS = ROOT / "params_new"


def _builder():
    spec = importlib.util.spec_from_file_location("build_param_metadata", BUILDER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def meta(**kw):
    base = dict(name="X", group="G", type="FLOAT")
    base.update(kw)
    return pm.ParamMeta(**base)


class FormattingTest(unittest.TestCase):
    def test_float32_noise_is_hidden(self):
        m = meta(decimals=2, units="m")
        self.assertEqual(pm.format_number(0.10000000149011612, m, True), "0.1")
        self.assertEqual(pm.format_number(0.07999999821186066, m, True), "0.08")

    def test_precision_is_never_cut_below_the_real_value(self):
        # decimalPlaces=1 must not turn 0.055 into 0.1
        self.assertEqual(pm.format_number(0.055, meta(decimals=1), True), "0.055")

    def test_trailing_zeros_and_negative_zero_go(self):
        self.assertEqual(pm.format_number(130.0, meta(decimals=3), True), "130")
        self.assertEqual(pm.format_number(-0.0, meta(decimals=2), True), "0")

    def test_integers_and_unknown_floats(self):
        self.assertEqual(pm.format_number(5, None, False), "5")
        self.assertEqual(pm.format_number(0.123456789, None, True), "0.123457")
        self.assertEqual(pm.format_number(1e-9, None, True), "1e-09")

    def test_nan_and_inf_do_not_crash(self):
        self.assertEqual(pm.format_number(float("nan"), None, True), "NaN")
        self.assertEqual(pm.format_number(float("inf"), None, True), "inf")

    def test_enum_shows_the_label_and_keeps_the_number(self):
        m = meta(type="INT32", values=((0, "Baro"), (2, "Range sensor")))
        self.assertEqual(pm.value_text(m, 2, False), "Range sensor (2)")
        self.assertEqual(pm.value_text(m, 7, False), "7")          # no such choice

    def test_bitmask_lists_the_set_bits(self):
        m = meta(type="INT32", bitmask=((0, "Horizontal"), (1, "Vertical"), (3, "Yaw")))
        self.assertEqual(pm.value_text(m, 3, False), "Horizontal, Vertical (3)")
        self.assertEqual(pm.value_text(m, 0, False), "0")
        self.assertEqual([on for _b, _l, on in pm.bit_states(m, 9)], [True, False, True])

    def test_units_are_appended_to_plain_numbers_only(self):
        self.assertEqual(pm.value_text(meta(units="m/s", decimals=1), 12.0, True), "12 m/s")
        enum = meta(type="INT32", units="x", values=((1, "One"),))
        self.assertEqual(pm.value_text(enum, 1, False), "One (1)")

    def test_no_metadata_is_just_the_number(self):
        self.assertEqual(pm.value_text(None, 7, False), "7")
        self.assertEqual(pm.default_text(None), "-")
        self.assertEqual(pm.range_text(None), "-")

    def test_range_and_default_text(self):
        m = meta(type="INT32", minimum=0, maximum=15, default=0)
        self.assertEqual(pm.range_text(m), "0 to 15")
        self.assertEqual(pm.default_text(m), "0")
        self.assertEqual(pm.range_text(meta(minimum=0.01)), "0.01 to ...")

    def test_modified_uses_a_float32_tolerance_and_is_unknown_without_a_default(self):
        m = meta(default=0.1)
        self.assertFalse(pm.is_modified(m, 0.10000000149011612))
        self.assertTrue(pm.is_modified(m, 0.2))
        self.assertIsNone(pm.is_modified(meta(), 5))
        self.assertIsNone(pm.is_modified(None, 5))

    def test_group_falls_back_to_the_name_prefix(self):
        self.assertEqual(pm.group_of("EKF2_EV_CTRL", meta(group="EKF2")), "EKF2")
        self.assertEqual(pm.group_of("FOO_BAR", None), "FOO")


class CatalogLoadingTest(unittest.TestCase):
    def test_a_missing_file_gives_an_empty_catalog_not_an_error(self):
        c = pm.load_catalog("/nonexistent/params.json.xz")
        self.assertFalse(c.available)
        self.assertEqual(len(c), 0)
        self.assertIn("no descriptions", c.label())

    def test_a_corrupt_file_gives_an_empty_catalog(self):
        with tempfile.NamedTemporaryFile(suffix=".xz", delete=False) as fh:
            fh.write(b"not lzma at all")
        try:
            self.assertFalse(pm.load_catalog(fh.name).available)
        finally:
            os.unlink(fh.name)

    def test_parse_catalog_reads_the_compact_format(self):
        c = pm.parse_catalog({"px4": {"version": "v9", "revision": "abc"},
                              "params": {"A_B": {"g": "Grp", "t": "INT32", "s": "short",
                                                 "d": 1, "mn": 0, "mx": 5, "rb": True,
                                                 "v": [[0, "off"], [1, "on"]]}}})
        m = c.get("A_B")
        self.assertEqual((m.group, m.type, m.default, m.maximum, m.reboot), ("Grp", "INT32", 1, 5, True))
        self.assertEqual(m.values, ((0, "off"), (1, "on")))
        self.assertEqual(c.label(), "PX4 v9 (abc)")


class BundledDataTest(unittest.TestCase):
    """The file that ships with the GCS."""

    @classmethod
    def setUpClass(cls):
        cls.cat = pm.load_catalog()

    def test_it_loads_and_names_the_firmware(self):
        self.assertTrue(self.cat.available)
        self.assertEqual(self.cat.version, "v1.17.0")
        self.assertTrue(self.cat.revision.startswith("d6f12ad"))   # this drone's firmware

    def test_the_parameters_we_tune_most_are_described_the_qgc_way(self):
        ev = self.cat.get("EKF2_EV_CTRL")
        self.assertEqual([l for _b, l in ev.bitmask],
                         ["Horizontal position", "Vertical position", "3D velocity", "Yaw"])
        self.assertEqual((ev.minimum, ev.maximum, ev.default), (0, 15, 0))
        hgt = self.cat.get("EKF2_HGT_REF")
        self.assertEqual(pm.enum_label(hgt, 2), "Range sensor")
        self.assertTrue(hgt.reboot)
        self.assertEqual(self.cat.get("EKF2_EV_DELAY").units, "ms")
        self.assertEqual(self.cat.get("COM_ARM_IMU_ACC").units, "m/s^2")
        self.assertEqual(pm.enum_label(self.cat.get("MAV_1_CONFIG"), 102), "TELEM 2")

    def test_hardware_specific_parameters_come_from_the_reference(self):
        for name in ("BAT1_V_DIV", "DSHOT_MIN", "MAV_1_CONFIG", "GPS_1_CONFIG"):
            m = self.cat.get(name)
            self.assertIsNotNone(m, name)
            self.assertTrue(m.short, name)

    @unittest.skipUnless(BOARD_PARAMS.exists(), "params_new backup not present")
    def test_it_describes_nearly_every_parameter_on_this_drone(self):
        names = []
        for line in BOARD_PARAMS.read_text().splitlines():
            if line.startswith("#") or not line.strip():
                continue
            names.append(line.split("\t")[2])
        have = sum(1 for n in names if self.cat.get(n))
        self.assertGreaterEqual(have / len(names), 0.99, f"{have}/{len(names)}")


class BuilderTest(unittest.TestCase):
    SAMPLE_MD = """# Parameter Reference

## Alpha

### ALPHA_MODE (`INT32`) {#ALPHA_MODE}

Alpha mode.

Longer explanation of the alpha mode.
It spans two lines.

**Values:**

- `0`: Off
- `1`: On

| Reboot  | minValue | maxValue | increment | default | unit |
| ------- | -------- | -------- | --------- | ------- | ---- |
| &check; | 0        | 1        |           | 1       |

### ALPHA_GAIN (`FLOAT`) {#ALPHA_GAIN}

Gain.

| Reboot | minValue | maxValue | increment | default | unit |
| ------ | -------- | -------- | --------- | ------- | ---- |
| &nbsp; | 0.5      | 9.5      | 0.01      | 4.05    | V    |

## Beta

### BETA_BITS (`INT32`) {#BETA_BITS}

Beta bits.

**Bitmask:**

- `0`: First
- `2`: Third

| Reboot | minValue | maxValue | increment | default | unit |
| ------ | -------- | -------- | --------- | ------- | ---- |
| &nbsp; | 0        | 5        |           | 0       |
"""

    def setUp(self):
        self.b = _builder()
        self.tmp = tempfile.TemporaryDirectory()
        self.md = Path(self.tmp.name) / "ref.md"
        self.md.write_text(self.SAMPLE_MD, encoding="utf-8")

    def tearDown(self):
        self.tmp.cleanup()

    def test_markdown_enum_range_default_and_reboot(self):
        e = self.b.from_markdown(self.md)["ALPHA_MODE"]
        self.assertEqual(e["g"], "Alpha")
        self.assertEqual(e["t"], "INT32")
        self.assertEqual(e["s"], "Alpha mode.")
        self.assertIn("Longer explanation", e["l"])
        self.assertEqual(e["v"], [[0, "Off"], [1, "On"]])
        self.assertEqual((e["mn"], e["mx"], e["d"]), (0, 1, 1))
        self.assertTrue(e["rb"])

    def test_markdown_float_units_and_increment(self):
        e = self.b.from_markdown(self.md)["ALPHA_GAIN"]
        self.assertEqual((e["mn"], e["mx"], e["inc"], e["d"], e["u"]), (0.5, 9.5, 0.01, 4.05, "V"))
        self.assertNotIn("rb", e)

    def test_markdown_bitmask_and_group_change(self):
        e = self.b.from_markdown(self.md)["BETA_BITS"]
        self.assertEqual(e["g"], "Beta")
        self.assertEqual(e["bm"], [[0, "First"], [2, "Third"]])

    def test_json_wins_over_the_reference_and_sources_are_recorded(self):
        j = {"ALPHA_MODE": {"g": "Alpha", "t": "INT32", "s": "from json", "d": 1}}
        merged = self.b.merge(j, self.b.from_markdown(self.md))
        self.assertEqual(merged["ALPHA_MODE"]["s"], "from json")
        self.assertEqual(merged["ALPHA_MODE"]["src"], "json")
        self.assertEqual(merged["BETA_BITS"]["src"], "doc")

    def test_the_built_file_round_trips_through_the_loader(self):
        jpath = Path(self.tmp.name) / "parameters.json"
        jpath.write_text(json.dumps({"parameters": [
            {"name": "ZED_X", "group": "Zed", "type": "Float", "shortDesc": "Zed x",
             "default": 1.5, "units": "m", "decimalPlaces": 2, "rebootRequired": True,
             "values": [{"value": 1, "description": "one"}]}]}), encoding="utf-8")
        out = Path(self.tmp.name) / "meta.json.xz"
        self.assertEqual(self.b.main(["--px4", self.tmp.name, "--json", str(jpath),
                                      "--markdown", str(self.md), "--out", str(out)]), 0)
        cat = pm.parse_catalog(json.loads(lzma.decompress(out.read_bytes())))
        self.assertEqual(cat.get("ZED_X").units, "m")
        self.assertTrue(cat.get("ZED_X").reboot)
        self.assertEqual(cat.get("ALPHA_MODE").values, ((0, "Off"), (1, "On")))

    def test_missing_input_is_reported_not_a_traceback(self):
        self.assertEqual(self.b.main(["--px4", self.tmp.name, "--out",
                                      str(Path(self.tmp.name) / "o.xz")]), 2)


if __name__ == "__main__":
    unittest.main()
