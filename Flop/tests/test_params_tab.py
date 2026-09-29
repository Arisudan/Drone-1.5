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

    def test_index_column_sorts_numerically_not_alphabetically(self):
        self.tab.begin_refresh()
        for i in (2, 10, 1):
            self.tab.on_param_value(f"P_{i}", i, INT32, i, 10)
        self.tab._flush()
        self.tab.table.sortItems(3, Qt.AscendingOrder)
        ordered = [self.tab.table.item(r, 3).data(Qt.DisplayRole)
                  for r in range(self.tab.table.rowCount())]
        self.assertEqual(ordered, [1, 2, 10])


if __name__ == "__main__":
    unittest.main()
