import math
import unittest

from core.navigation.lane_model import LaneLocator
from core.navigation.road_network import RoadNetwork


class SyntheticMap:
    def __init__(self):
        self.net = RoadNetwork()
        self.net.loaded = True
        self.next_road_uid = 1000

    def node(self, uid, x, z, y=0.0):
        self.net.nodes[uid] = (float(x), float(z))
        self.net.node_alt[uid] = float(y)
        self.net.node_rot[uid] = 0.0
        self.net.node_forward[uid] = (0.0, 1.0)
        self.net._ngrid.setdefault(self.net._cell(x, z), []).append(uid)

    def road(self, start, end, lanes=2):
        road_uid = self.next_road_uid
        self.next_road_uid += 1
        token = f"look-{lanes}"
        lane_types = tuple("traffic_lane.road.local" for _ in range(lanes))
        self.net.road_looks[token] = {
            "type": "local", "lanes": lanes,
            "lanes_left": 0, "lanes_right": lanes,
            "lane_types_left": (), "lane_types_right": lane_types,
            "offset_m": 0.0,
        }
        index = len(self.net.segments)
        a, b = self.net.nodes[start], self.net.nodes[end]
        self.net.segments.append((a, b))
        self.net._seg_uids.append((start, end))
        self.net._seg_road_uids.append(road_uid)
        self.net._seg_look_tokens.append(token)
        self.net._road_length[(start, end)] = math.dist(a, b)
        self.net._road_look_token[start] = token
        self.net._road_look_token[end] = token
        self.net._seg_grid.setdefault(self.net._cell(*a), []).append(index)
        if self.net._cell(*a) != self.net._cell(*b):
            self.net._seg_grid.setdefault(self.net._cell(*b), []).append(index)
        self.net.fwd.setdefault(start, []).append(end)
        self.net.bwd.setdefault(end, []).append(start)
        return index

    def match_on(self, segment_index, lane_index, gps):
        target = next(lane for lane in self.net._build_lane_segments(segment_index)
                      if lane.direction == 1 and lane.lane_index == lane_index)
        point = target.centerline[len(target.centerline) // 2]
        match = LaneLocator(self.net).locate(
            (point.x, point.y, point.z), point.heading, gps)
        self.assert_match(match, target.lane_id)
        return match

    @staticmethod
    def assert_match(match, lane_id):
        if match is None or match.lane_id != lane_id:
            raise AssertionError(f"expected {lane_id}, got {match}")


class LaneRouteBuilderTests(unittest.TestCase):
    def test_straight_multi_lane_keeps_locator_lane(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 40); m.node(3, 0, 80)
        first = m.road(1, 2, 3); m.road(2, 3, 3)
        match = m.match_on(first, 1, (1, 2, 3))
        corridor = m.net.resolve_gps_corridor((1, 2, 3))
        segments, reason = m.net.select_lane_sequence(corridor, match)
        path = m.net.connect_lane_sequence(segments, corridor.gps_uids)
        self.assertEqual(reason, "")
        self.assertTrue(path.valid, path.failure_reason)
        self.assertEqual([lane.lane_index for lane in segments], [1, 1])

    def test_lane_count_change_merge_and_split(self):
        m = SyntheticMap()
        for uid, z in enumerate((0, 40, 80, 120), 1):
            m.node(uid, 0, z)
        first = m.road(1, 2, 3)
        m.road(2, 3, 2)
        m.road(3, 4, 3)
        match = m.match_on(first, 2, (1, 2, 3, 4))
        corridor = m.net.resolve_gps_corridor((1, 2, 3, 4))
        segments, reason = m.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        self.assertEqual([lane.lane_index for lane in segments], [2, 1, 1])
        self.assertEqual(segments[0].successors[0].kind, "merge")
        self.assertEqual(segments[1].successors[0].kind, "split")
        self.assertTrue(m.net.connect_lane_sequence(
            segments, corridor.gps_uids).valid)

    def test_intersection_follows_authoritative_uid_branch(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 30)
        m.node(3, -30, 60); m.node(4, 30, 60)
        first = m.road(1, 2, 1)
        m.road(2, 3, 1); m.road(2, 4, 1)
        match = m.match_on(first, 0, (1, 2, 4))
        corridor = m.net.resolve_gps_corridor((1, 2, 4))
        self.assertEqual([(e.start_uid, e.end_uid) for e in corridor.edges],
                         [(1, 2), (2, 4)])
        segments, reason = m.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        self.assertEqual(segments[-1].end_uid, 4)

    def test_parallel_road_is_not_selected_by_geometry(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 50)
        m.node(10, 0.5, 0); m.node(11, 0.5, 50)
        route = m.road(1, 2, 1); m.road(10, 11, 1)
        match = m.match_on(route, 0, (1, 2))
        corridor = m.net.resolve_gps_corridor((1, 2))
        segments, reason = m.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        self.assertEqual((segments[0].start_uid, segments[0].end_uid), (1, 2))

    def test_bridge_altitude_selects_correct_layer(self):
        m = SyntheticMap()
        m.node(1, 0, 0, 0); m.node(2, 0, 50, 0)
        m.node(10, 0, 0, 12); m.node(11, 0, 50, 12)
        m.road(1, 2, 1); bridge = m.road(10, 11, 1)
        match = m.match_on(bridge, 0, (10, 11))
        self.assertLess(match.vertical_error_m, 0.01)
        corridor = m.net.resolve_gps_corridor((10, 11))
        segments, reason = m.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        self.assertTrue(all(abs(point.y - 12) < 0.01
                            for point in segments[0].centerline))

    def test_unproven_gap_is_rejected(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 20)
        corridor = m.net.resolve_gps_corridor((1, 2))
        self.assertFalse(corridor.valid)
        self.assertIn("no directed topological path", corridor.failure_reason)


if __name__ == "__main__":
    unittest.main()
