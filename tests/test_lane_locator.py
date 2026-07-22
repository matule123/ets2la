import math
import unittest

from core.navigation.lane_model import (
    LaneId, LaneLocator, LaneLocatorConfig, LanePoint, LaneSegment,
)


def lane(road_uid, x, direction=1, height=0.0, gps=(10, 11), lane_index=0):
    points = [LanePoint(x, height, z, float(z), math.pi)
              for z in (0.0, 20.0, 40.0)]
    if direction < 0:
        points = [LanePoint(x, height, z, float(40-z), 0.0)
                  for z in (40.0, 20.0, 0.0)]
    return LaneSegment(
        LaneId(road_uid, direction, lane_index), gps[0], gps[1], direction,
        lane_index, 2,
        4.5, "derived", int(round(height / 3)), "look",
        "traffic_lane.road.local", tuple(points), gps_uids=frozenset(gps))


class FakeNetwork:
    def __init__(self, lanes):
        self.lanes = lanes
        self.connected = set()

    def lane_segments_near(self, _pos, _radius):
        return self.lanes

    def altitude_near(self, _pos):
        return 0.0

    def lanes_connected(self, first, second):
        return (first == second or (first, second) in self.connected
                or (first.road_uid == second.road_uid
                    and first.direction == second.direction
                    and abs(first.lane_index - second.lane_index) == 1))


class LaneLocatorTests(unittest.TestCase):
    def test_heading_rejects_opposite_carriageway(self):
        forward = lane(1, 0, 1)
        backward = lane(2, 0.2, -1)
        match = LaneLocator(FakeNetwork([backward, forward])).locate(
            (0.1, 0.0, 15.0), math.pi, (10, 11))
        self.assertEqual(match.lane_id, forward.lane_id)

    def test_height_separates_bridge_from_road_below(self):
        lower = lane(1, 0, 1, 0.0)
        bridge = lane(2, 0, 1, 12.0)
        locator = LaneLocator(FakeNetwork([lower, bridge]))
        match = locator.locate((0, 11.8, 15), math.pi)
        self.assertEqual(match.lane_id, bridge.lane_id)
        self.assertLess(match.vertical_error_m, 0.3)

    def test_gps_membership_beats_near_parallel_road(self):
        wrong = lane(1, 0.0, gps=(90, 91))
        route = lane(2, 1.0, gps=(10, 11))
        match = LaneLocator(FakeNetwork([wrong, route])).locate(
            (0.1, 0, 15), math.pi, (10, 11))
        self.assertEqual(match.lane_id, route.lane_id)

    def test_directed_gps_edge_beats_wrong_arm_sharing_junction_uid(self):
        wrong_arm = lane(1, 0.0, gps=(9, 10))
        route_arm = lane(2, 0.3, gps=(10, 11))
        match = LaneLocator(FakeNetwork([wrong_arm, route_arm])).locate(
            (0.0, 0.0, 15.0), math.pi, (10, 11, 12))
        self.assertIsNotNone(match)
        self.assertEqual(match.lane_id, route_arm.lane_id)

    def test_hysteresis_holds_previous_lane_for_small_score_change(self):
        left = lane(1, -1.0, lane_index=0)
        right = lane(1, 1.0, lane_index=1)
        locator = LaneLocator(FakeNetwork([left, right]),
                              LaneLocatorConfig(switch_margin=1.5))
        first = locator.locate((-0.8, 0, 15), math.pi)
        self.assertEqual(first.lane_id, left.lane_id)
        second = locator.locate((0.15, 0, 15), math.pi)
        self.assertEqual(second.lane_id, left.lane_id)
        self.assertEqual(second.switch_reason, "hysteresis_hold")
        third = locator.locate((0.95, 0, 15), math.pi)
        self.assertEqual(third.lane_id, right.lane_id)

    def test_no_match_when_height_is_ambiguous_and_too_far(self):
        match = LaneLocator(FakeNetwork([lane(1, 0, height=15)])).locate(
            (0, 0, 15), math.pi)
        self.assertIsNone(match)


if __name__ == "__main__":
    unittest.main()
