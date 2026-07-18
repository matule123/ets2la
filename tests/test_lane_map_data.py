import os
import unittest

from core.navigation.lane_model import LaneLocator
from core.navigation.road_network import RoadNetwork


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATASET = os.path.join(ROOT, "map-cache", "promods-1.59")


@unittest.skipUnless(os.path.isdir(DATASET), "ProMods 1.59 dataset not installed")
class RealMapLaneDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.net = RoadNetwork()
        assert cls.net.load(DATASET)

    def test_real_lane_metadata_is_preserved(self):
        self.assertGreater(len(self.net.road_looks), 1000)
        look = next(value for value in self.net.road_looks.values()
                    if value["lanes_left"] >= 2 and value["lanes_right"] >= 2)
        self.assertEqual(len(look["lane_types_left"]), look["lanes_left"])
        self.assertEqual(len(look["lane_types_right"]), look["lanes_right"])
        self.assertIn("offset_m", look)

    def test_prefab_lane_connectivity_is_preserved(self):
        self.assertGreater(len(self.net._prefab_lane_data), 4000)
        item = next(value for value in self.net._prefab_lane_data.values()
                    if value["curves"] and
                    any(c["next_lines"] or c["prev_lines"] for c in value["curves"]))
        self.assertTrue(item["curves"])
        self.assertIn("nav_node_index", item["curves"][0])

    def test_lane_index_and_locator_on_real_road(self):
        index = next(i for i in range(len(self.net.segments))
                     if self.net._build_lane_segments(i))
        lanes = self.net._build_lane_segments(index)
        target = lanes[0]
        point = target.centerline[len(target.centerline) // 2]
        match = LaneLocator(self.net).locate(
            (point.x, point.y, point.z), point.heading,
            (target.start_uid, target.end_uid))
        self.assertIsNotNone(match)
        self.assertEqual(match.lane_id, target.lane_id)
        self.assertLess(match.lateral_error_m, 0.1)
        self.assertLess(match.vertical_error_m, 0.1)


if __name__ == "__main__":
    unittest.main()
