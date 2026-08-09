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
        view.set_road_segments([valid] * 17000)
        self.assertEqual(16000, len(view.road_segments))
        self.assertEqual((0.0, 1.0), view.road_segments[0]["a"])
        self.assertEqual((2.0, 3.0), view.road_segments[0]["b"])
        self.assertEqual("road", view.road_segments[0]["kind"])
        view.set_road_segments([valid, invalid])
        self.assertEqual(1, len(view.road_segments))

    def test_navigation_view_has_no_full_road_network(self):
        view = MapView(_State())
        self.assertFalse(hasattr(view, "road_net"))
        self.assertEqual([], view.road_segments)

    def test_continuous_roads_are_prebuilt_once_per_scene_revision(self):
        view = MapView(_State())
        payload = [
            [[float(index), 0.0, 0.0],
             [float(index + 1), 0.0, 0.0],
             "road", 2, False, True, False, False, 4.5, False,
             "r10:0", index, "local"]
            for index in range(1000)
        ]
        view.set_road_segments(payload)
        self.assertEqual(1000, len(view.road_segments))
        self.assertEqual(1, len(view._road_runs))
        self.assertEqual(1001, len(view._road_runs[0][3]))


if __name__ == "__main__":
    unittest.main()
