"""Tests for the Parameters tab (Phase 1: read-only) and the MAVLinkWorker
PARAM_VALUE dispatch it is fed from.

Covers the one thing most likely to break silently here: row identity across
a sort. ParamsTabWidget's table has sorting enabled (operators expect to
click NAME/INDEX to sort), and a row's numeric index changes the instant the
table resorts - so anything that remembered "row 7" instead of "this
QTableWidgetItem" would silently start updating or hiding the wrong row.
test_search_filter_hides_non_matching_rows_after_sort exercises exactly that.
"""

import struct
import unittest

import _env  # noqa: F401  -- sets sys.path + offscreen Qt; must import first

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication
from pymavlink import mavutil

from protocol.mavlink_worker import MAVLinkWorker
from ui.params_tab import ParamsTabWidget

INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32


class ParamMsg:
    """Minimal stand-in for a decoded PARAM_VALUE."""

    def __init__(self, param_id, param_value, param_type, param_index, param_count):
        self.param_id = param_id
        self.param_value = param_value
        self.param_type = param_type
        self.param_index = param_index
        self.param_count = param_count

    def get_type(self):
        return "PARAM_VALUE"

    def get_srcSystem(self):
        return 1


def _worker() -> MAVLinkWorker:
    w = MAVLinkWorker(host="127.0.0.1", port=1)
    w.target_system = 1
    return w


def _int32_wire_value(n: int) -> float:
    return struct.unpack('<f', struct.pack('<i', n))[0]


class WorkerParamDispatchTest(unittest.TestCase):
    def setUp(self):
        self.app = QApplication.instance() or QApplication([])
        self.w = _worker()
        self.received = []
        self.w.param_value_received.connect(lambda *a: self.received.append(a))

    def test_int32_param_is_decoded_and_emitted(self):
        self.w._handle_msg(ParamMsg("EKF2_HGT_REF", _int32_wire_value(42), INT32, 3, 100))
        self.assertEqual(len(self.received), 1)
        name, value, ptype, index, count = self.received[0]
        self.assertEqual(name, "EKF2_HGT_REF")
        self.assertEqual(value, 42)
        self.assertEqual(ptype, INT32)
        self.assertEqual(index, 3)
        self.assertEqual(count, 100)

    def test_nul_padded_name_is_stripped(self):
        self.w._handle_msg(ParamMsg("SYS_AUTOSTART\x00\x00\x00", 1.0, REAL32, 0, 1))
        self.assertEqual(self.received[0][0], "SYS_AUTOSTART")

    def test_wrong_source_system_is_ignored(self):
        self.w.target_system = 7  # not this message's srcSystem (1)
        self.w._handle_msg(ParamMsg("SOME_PARAM", 1.0, REAL32, 0, 1))
        self.assertEqual(self.received, [])


class ParamsTabWidgetTest(unittest.TestCase):
    def setUp(self):
        self.app = QApplication.instance() or QApplication([])
        self.tab = ParamsTabWidget()

    def test_values_appear_in_the_table_after_flush(self):
        self.tab.begin_refresh()
        self.tab.on_param_value("AAA_PARAM", 1, INT32, 0, 2)
        self.tab.on_param_value("BBB_PARAM", 2, INT32, 1, 2)
        self.tab._flush()
        self.assertEqual(self.tab.table.rowCount(), 2)
        self.assertTrue(self.tab.has_data())

    def test_updating_an_existing_param_does_not_duplicate_its_row(self):
        self.tab.begin_refresh()
        self.tab.on_param_value("SAME_PARAM", 1, INT32, 0, 1)
        self.tab._flush()
        self.tab.on_param_value("SAME_PARAM", 2, INT32, 0, 1)
        self.tab._flush()
        self.assertEqual(self.tab.table.rowCount(), 1)
        self.assertEqual(self.tab.table.item(0, 1).text(), "2")

    def test_search_filter_hides_non_matching_rows_after_sort(self):
        self.tab.begin_refresh()
        self.tab.on_param_value("ZZZ_ALPHA", 1, INT32, 0, 2)
        self.tab.on_param_value("AAA_BETA", 2, INT32, 1, 2)
        self.tab._flush()

        # Force a real re-sort, as if the operator clicked the NAME header -
        # exactly the moment a stored-row-index lookup would go stale.
        self.tab.table.sortItems(0, Qt.AscendingOrder)

        self.tab.search.setText("alpha")
        visible = [r for r in range(self.tab.table.rowCount())
                  if not self.tab.table.isRowHidden(r)]
        self.assertEqual(len(visible), 1)
        self.assertEqual(self.tab.table.item(visible[0], 0).text(), "ZZZ_ALPHA")

    def test_begin_refresh_clears_previous_data(self):
        self.tab.begin_refresh()
        self.tab.on_param_value("OLD_PARAM", 1, INT32, 0, 1)
        self.tab._flush()
        self.tab.begin_refresh()
        self.assertEqual(self.tab.table.rowCount(), 0)
        self.assertFalse(self.tab.has_data())

    def test_names_list_a_to_z_by_default(self):
        self.tab.begin_refresh()
        for n, name in enumerate(("MC_YAW_P", "EKF2_EN", "BAT1_N_CELLS")):
            self.tab.on_param_value(name, 1, INT32, n, 3)
        self.tab._flush()
        names = [self.tab.table.item(r, 0).text() for r in range(self.tab.table.rowCount())]
        self.assertEqual(names, sorted(names))

    def test_a_later_update_keeps_its_row_after_the_operator_resorts(self):
        self.tab.begin_refresh()
        for n, name in enumerate(("AAA", "BBB", "CCC")):
            self.tab.on_param_value(name, n, INT32, n, 3)
        self.tab._flush()
        self.tab.table.sortItems(0, Qt.DescendingOrder)
        self.tab.on_param_value("BBB", 99, INT32, 1, 3)
        self.tab._flush()
        self.assertEqual(self.tab.table.rowCount(), 3)
        row = self.tab._name_item["BBB"].row()
        self.assertEqual(self.tab.table.item(row, 1).text(), "99")


class QgcLayoutTest(unittest.TestCase):
    """The tab shows what QGroundControl shows: groups, named values, units,
    changed-from-default in bold, and a detail panel."""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tab = ParamsTabWidget()
        self.tab.begin_refresh()
        self.rows = [
            ("EKF2_EV_CTRL", 3, INT32),            # bitmask, changed
            ("EKF2_HGT_REF", 2, INT32),            # choice, changed
            ("EKF2_EVP_NOISE", 0.10000000149011612, REAL32),   # float32 noise, at default
            ("EKF2_EV_DELAY", 130.0, REAL32),      # units, changed
            ("BAT1_N_CELLS", 0, INT32),            # other group, at PX4's default (0)
            ("TOTALLY_UNKNOWN", 7, INT32),         # no description anywhere
        ]
        for n, (name, value, ptype) in enumerate(self.rows):
            self.tab.on_param_value(name, value, ptype, n, len(self.rows))
        self.tab._flush()

    def text(self, name, col=1):
        return self.tab.table.item(self.tab._name_item[name].row(), col).text()

    def visible(self):
        t = self.tab.table
        return sorted(t.item(r, 0).text() for r in range(t.rowCount()) if not t.isRowHidden(r))

    def test_values_read_like_qgc(self):
        self.assertEqual(self.text("EKF2_HGT_REF"), "Range sensor (2)")
        self.assertEqual(self.text("EKF2_EV_CTRL"), "Horizontal position, Vertical position (3)")
        self.assertEqual(self.text("EKF2_EV_DELAY"), "130 ms")
        self.assertEqual(self.text("EKF2_EVP_NOISE"), "0.1 m")     # no float32 noise
        self.assertEqual(self.text("TOTALLY_UNKNOWN"), "7")

    def test_description_column_holds_the_short_description(self):
        self.assertEqual(self.text("EKF2_EV_CTRL", 3), "External vision (EV) sensor aiding")  # col 2 is RANGE in v2
        self.assertEqual(self.text("TOTALLY_UNKNOWN", 3), "")

    def test_groups_are_listed_with_counts_and_filter_the_table(self):
        labels = [self.tab.groups.item(i).text() for i in range(self.tab.groups.count())]
        self.assertTrue(labels[0].startswith("All parameters"))
        self.assertIn("EKF2  (4)", labels)
        for i in range(self.tab.groups.count()):
            if self.tab.groups.item(i).data(Qt.UserRole) == "EKF2":
                self.tab.groups.setCurrentRow(i)
        self.assertEqual(self.visible(),
                         ["EKF2_EVP_NOISE", "EKF2_EV_CTRL", "EKF2_EV_DELAY", "EKF2_HGT_REF"])

    def test_a_parameter_without_a_description_groups_by_its_name_prefix(self):
        labels = [self.tab.groups.item(i).text() for i in range(self.tab.groups.count())]
        self.assertIn("TOTALLY  (1)", labels)

    def test_changed_from_default_is_bold_and_modified_only_filters(self):
        def bold(name):
            return self.tab.table.item(self.tab._name_item[name].row(), 0).font().bold()
        self.assertTrue(bold("EKF2_EV_CTRL"))
        self.assertTrue(bold("EKF2_HGT_REF"))
        self.assertFalse(bold("EKF2_EVP_NOISE"))        # equals PX4's default
        self.assertFalse(bold("TOTALLY_UNKNOWN"))       # default unknown: never claimed changed
        self.tab.chk_modified.setChecked(True)
        self.assertEqual(self.visible(), ["EKF2_EV_CTRL", "EKF2_EV_DELAY", "EKF2_HGT_REF"])

    def test_search_looks_in_every_group_and_in_descriptions(self):
        for i in range(self.tab.groups.count()):
            if self.tab.groups.item(i).data(Qt.UserRole) == "BAT1" or \
               self.tab.groups.item(i).data(Qt.UserRole) == "Battery Calibration":
                self.tab.groups.setCurrentRow(i)
        self.tab.search.setText("height")                 # in EKF2_HGT_REF's description
        self.assertIn("EKF2_HGT_REF", self.visible())

    def test_detail_panel_shows_the_choices_and_marks_the_current_one(self):
        self.assertTrue(self.tab.select("EKF2_HGT_REF"))
        html = self.tab.detail.toPlainText()
        self.assertIn("Range sensor", html)
        self.assertIn("Barometric pressure", html)
        self.assertIn("Choices", html)

    def test_detail_panel_shows_each_bit_of_a_bitmask(self):
        self.tab.select("EKF2_EV_CTRL")
        text = self.tab.detail.toPlainText()
        self.assertIn("bit 0", text)
        self.assertIn("Horizontal position", text)
        self.assertIn("3D velocity", text)

    def test_detail_panel_has_default_range_type_group_and_reboot(self):
        self.tab.select("EKF2_EV_CTRL")
        text = self.tab.detail.toPlainText()
        for needle in ("Default", "Range", "0 to 15", "INT32", "EKF2", "Reboot", "Index"):
            self.assertIn(needle, text)

    def test_detail_panel_for_an_undescribed_parameter_says_so_not_an_error(self):
        self.assertTrue(self.tab.select("TOTALLY_UNKNOWN"))
        self.assertIn("No description", self.tab.detail.toPlainText())

    def test_status_names_the_firmware_the_descriptions_come_from(self):
        self.assertIn("PX4 v1.17.0", self.tab.lbl_status.text())
        self.assertIn("changed from default", self.tab.lbl_status.text())

    def test_it_still_works_with_no_descriptions_at_all(self):
        from core.param_meta import ParamCatalog
        bare = ParamsTabWidget(catalog=ParamCatalog())
        bare.begin_refresh()
        bare.on_param_value("EKF2_HGT_REF", 2, INT32, 0, 1)
        bare._flush()
        self.assertEqual(bare.table.item(0, 1).text(), "2")      # the plain number
        self.assertEqual(bare.table.rowCount(), 1)
        self.assertIn("no parameter descriptions", bare.lbl_status.text())

    def test_it_never_writes_to_the_vehicle(self):
        for attr in ("set_param", "param_set", "write_param", "send_param"):
            self.assertFalse(hasattr(self.tab, attr), attr)


if __name__ == "__main__":
    unittest.main()
