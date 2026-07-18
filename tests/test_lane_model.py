import math
import unittest

from core.navigation.lane_model import LaneId, LanePoint, LaneSegment


class LaneModelTests(unittest.TestCase):
    def test_lane_identity_is_stable_and_hashable(self):
        first = LaneId(123, 1, 0)
        second = LaneId(123, 1, 0)
        self.assertEqual(first, second)
        self.assertEqual(hash(first), hash(second))
        self.assertNotEqual(first, LaneId(123, -1, 0))

    def test_lane_segment_preserves_height_and_provenance(self):
        lane = LaneSegment(
            lane_id=LaneId(7, 1, 0), start_uid=10, end_uid=11,
            direction=1, lane_index=0, lane_count=2,
            width_m=4.5, width_source="derived", elevation_layer=8,
            road_look_token="look", lane_type="traffic_lane.road.local",
            centerline=(LanePoint(0, 24, 0), LanePoint(0, 25, 10, 10)),
        )
        self.assertEqual(lane.centerline[0].y, 24)
        self.assertEqual(lane.width_source, "derived")
        self.assertEqual(lane.elevation_layer, 8)


if __name__ == "__main__":
    unittest.main()
