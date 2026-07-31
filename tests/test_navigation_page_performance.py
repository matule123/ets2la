import math
import os
import sys
import unittest
import importlib.util
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from PyQt6.QtWidgets import QApplication

spec = importlib.util.spec_from_file_location("ultrapilot_map_page",
                                              ROOT / "ui" / "map_page.py")
map_page = importlib.util.module_from_spec(spec)
spec.loader.exec_module(map_page)
MapView = map_page.MapView


class _State:
    def get(self, _key, default=None):
        return default


class NavigationPagePerformanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_live_snapshot_is_bounded_and_rejects_non_finite_geometry(self):
        view = MapView(_State())
        valid = [[0.0, 1.0, 20.0], [2.0, 3.0, 21.0], "road"]
        invalid = [[math.nan, 1.0, 20.0], [2.0, 3.0, 21.0], "road"]
        view.set_road_segments([valid] * 1300 + [invalid])
        self.assertEqual(1200, len(view.road_segments))
        self.assertEqual((0.0, 1.0), view.road_segments[0]["a"])
        self.assertEqual((2.0, 3.0), view.road_segments[0]["b"])
        self.assertEqual("road", view.road_segments[0]["kind"])

    def test_navigation_view_has_no_full_road_network(self):
        view = MapView(_State())
        self.assertFalse(hasattr(view, "road_net"))
        self.assertEqual([], view.road_segments)


if __name__ == "__main__":
    unittest.main()
