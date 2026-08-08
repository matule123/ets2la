import io
import math
import pickle
import unittest
from unittest import mock

from core.navigation.lane_model import (
    LaneConnection, LaneId, LaneLocator, LanePoint, LaneSegment,
)
from core.navigation.road_network import CACHE_VERSION, RoadNetwork


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
        self.net.node_forward_item[uid] = 0
        self.net.node_backward_item[uid] = 0
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
        self.net._road_segment_by_uid[road_uid] = index
        self.net._seg_look_tokens.append(token)
        self.net._road_length[(start, end)] = math.dist(a, b)
        self.net._road_look_token[start] = token
        self.net._road_look_token[end] = token
        self.net._seg_grid.setdefault(self.net._cell(*a), []).append(index)
        if self.net._cell(*a) != self.net._cell(*b):
            self.net._seg_grid.setdefault(self.net._cell(*b), []).append(index)
        self.net.fwd.setdefault(start, []).append(end)
        self.net.bwd.setdefault(end, []).append(start)
        self.net.node_forward_item[start] = road_uid
        self.net.node_backward_item[end] = road_uid
        return index

    def look(self, token, left, right, offset=0.0):
        lane = "traffic_lane.road.local"
        self.net.road_looks[token] = {
            "type": "local", "lanes": left + right,
            "lanes_left": left, "lanes_right": right,
            "lane_types_left": (lane,) * left,
            "lane_types_right": (lane,) * right,
            "offset_m": offset,
        }

    def set_road_look(self, segment_index, token):
        self.net._seg_look_tokens[segment_index] = token
        self.net._lane_cache.pop(segment_index, None)

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
    def test_map_load_progress_is_phase_based_and_observational(self):
        net = RoadNetwork()
        phases = []
        with mock.patch.object(net, "_try_load_cache", return_value=True):
            self.assertTrue(net.load(
                "unused",
                progress_cb=lambda fraction, phase:
                    phases.append((fraction, phase))))
        self.assertEqual(phases[0][0], 0.02)
        self.assertEqual(phases[-1][0], 1.0)
        self.assertIn("cache", phases[0][1].lower())

        def broken_observer(_fraction, _phase):
            raise RuntimeError("UI observer failed")

        isolated = RoadNetwork()
        with mock.patch.object(isolated, "_try_load_cache", return_value=True):
            self.assertTrue(isolated.load(
                "unused", progress_cb=broken_observer))

    def test_cache_unpickle_reports_byte_progress_instead_of_staying_at_two(self):
        payload = pickle.dumps({
            "version": CACHE_VERSION,
            "sig": [],
            "data": {"nodes": {1: (0.0, 0.0)}, "loaded": True},
        }, protocol=pickle.HIGHEST_PROTOCOL)
        phases = []
        network = RoadNetwork()
        with (mock.patch("os.path.exists", return_value=True),
              mock.patch("os.path.getsize", return_value=len(payload)),
              mock.patch("builtins.open",
                         side_effect=lambda *_args, **_kwargs:
                             io.BytesIO(payload)),
              mock.patch.object(network, "_source_signature", return_value=[])):
            self.assertTrue(network._try_load_cache(
                "unused", progress_cb=lambda fraction, phase:
                    phases.append((fraction, phase))))
        self.assertTrue(phases)
        self.assertGreaterEqual(phases[-1][0], 0.88)
        self.assertIn("prefaby", phases[-1][1].lower())

    def test_confirmed_prefab_exit_cannot_move_following_road_geometry(self):
        prefab_id = LaneId(10, 1, 0, "junction", 3, (3,))
        road_id = LaneId(20, 1, 0)
        prefab = LaneSegment(
            prefab_id, 1, 2, 1, 0, 1, 4.5, "dataset", 0, None,
            "prefab", (
                LanePoint(3.0, 0.0, 10.0, 0.0, 0.0, lane_id=prefab_id),
                LanePoint(3.0, 0.0, 0.0, 10.0, 0.0, lane_id=prefab_id),
            ), successors=(LaneConnection(road_id, "prefab"),),
            gps_pair_index=0)
        road_points = tuple(
            LanePoint(0.0, 0.0, -distance, distance, 0.0,
                      lane_id=road_id)
            for distance in (0.0, 10.0, 20.0, 30.0, 40.0, 50.0))
        road = LaneSegment(
            road_id, 2, 3, 1, 0, 1, 4.5, "derived", 0, "look",
            "road", road_points, gps_pair_index=1)
        original = tuple((point.x, point.y, point.z)
                         for point in road.centerline)
        path = RoadNetwork().connect_lane_sequence(
            (prefab, road), (1, 2, 3))
        self.assertFalse(path.valid)
        self.assertIn("3.00 m geometry gap", path.failure_reason)
        self.assertIn("no chord", path.failure_reason)
        self.assertEqual(tuple((point.x, point.y, point.z)
                               for point in road.centerline), original)

    def test_ets2la_lane_centres_cover_balanced_unbalanced_and_one_way(self):
        net = RoadNetwork()
        lane = "traffic_lane.road.local"
        make = lambda left, right, offset=0.0: {
            "lane_types_left": (lane,) * left,
            "lane_types_right": (lane,) * right,
            "offset_m": offset,
        }
        self.assertEqual(net._lane_center_offsets(make(1, 1)),
                         ((-2.25,), (2.25,)))
        self.assertEqual(net._lane_center_offsets(make(1, 2)),
                         ((-6.75,), (-2.25, 2.25)))
        self.assertEqual(net._lane_center_offsets(make(2, 1)),
                         ((2.25, -2.25), (6.75,)))
        self.assertEqual(net._lane_center_offsets(make(0, 3)),
                         ((), (-2.25, 2.25, 6.75)))
        self.assertEqual(net._lane_center_offsets(make(1, 1, 2.0)),
                         ((-4.25,), (4.25,)))

    def test_road_look_offset_transition_is_continuous_not_diagonal_gap(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 40); m.node(3, 0, 80)
        first = m.road(1, 2, 2)
        second = m.road(2, 3, 2)
        m.look("normal", 1, 1, 0.0)
        m.look("divided", 1, 1, 5.0)
        m.set_road_look(first, "normal")
        m.set_road_look(second, "divided")

        first_lane = next(lane for lane in m.net._build_lane_segments(first)
                          if lane.direction == 1)
        second_lane = next(lane for lane in m.net._build_lane_segments(second)
                           if lane.direction == 1)
        a, b = first_lane.centerline[-1], second_lane.centerline[0]
        self.assertLess(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)), 1e-6)
        # The new road reaches its own +5 m road offset gradually instead of
        # inserting a 5 m sideways chord at the junction.
        self.assertAlmostEqual(second_lane.centerline[0].x,
                               first_lane.centerline[-1].x, places=6)
        self.assertAlmostEqual(abs(second_lane.centerline[-1].x
                                   - second_lane.centerline[0].x), 5.0, places=3)

    def test_non_drivable_lane_does_not_renumber_physical_continuation(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 40); m.node(3, 0, 80)
        first = m.road(1, 2, 2)
        second = m.road(2, 3, 2)
        road = "traffic_lane.road.motorway"
        m.net.road_looks["two"] = {
            "type": "motorway", "lanes": 2,
            "lanes_left": 0, "lanes_right": 2,
            "lane_types_left": (), "lane_types_right": (road, road),
            "offset_m": 0.0,
        }
        m.net.road_looks["player-only"] = {
            "type": "motorway", "lanes": 2,
            "lanes_left": 0, "lanes_right": 2,
            "lane_types_left": (),
            "lane_types_right": ("traffic_lane.no_vehicles", road),
            "offset_m": 0.0,
        }
        m.set_road_look(first, "two")
        m.set_road_look(second, "player-only")
        match = m.match_on(first, 1, (1, 2, 3))
        corridor = m.net.resolve_gps_corridor((1, 2, 3))
        segments, reason = m.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        self.assertEqual([segment.lane_index for segment in segments], [1, 0])
        self.assertEqual([segment.raw_lane_index for segment in segments], [1, 1])
        self.assertLess(math.dist(
            (segments[0].centerline[-1].x, segments[0].centerline[-1].y,
             segments[0].centerline[-1].z),
            (segments[1].centerline[0].x, segments[1].centerline[0].y,
             segments[1].centerline[0].z)), 1e-6)

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

    def test_lane_count_change_merge_and_split_requires_real_geometry(self):
        m = SyntheticMap()
        for uid, z in enumerate((0, 40, 80, 120), 1):
            m.node(uid, 0, z)
        first = m.road(1, 2, 3)
        m.road(2, 3, 2)
        m.road(3, 4, 3)
        match = m.match_on(first, 2, (1, 2, 3, 4))
        corridor = m.net.resolve_gps_corridor((1, 2, 3, 4))
        segments, reason = m.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason,
            "LANE_CHANGE_INSUFFICIENT_APPROACH: available approach 40.00 m "
            "is shorter than required 42.75 m")
        self.assertEqual(len(segments), 1)

        # The same directed merge is accepted only when the common road is
        # long enough for the separately validated transition.
        long = SyntheticMap()
        for uid, z in enumerate((0, 80, 160, 240), 1):
            long.node(uid, 0, z)
        long_first = long.road(1, 2, 3)
        long.road(2, 3, 2)
        long.road(3, 4, 3)
        long_match = long.match_on(long_first, 2, (1, 2, 3, 4))
        long_corridor = long.net.resolve_gps_corridor((1, 2, 3, 4))
        planned, reason = long.net.select_lane_sequence(
            long_corridor, long_match)
        self.assertEqual(reason, "")
        self.assertEqual([lane.lane_index for lane in planned], [2, 1, 1])
        self.assertIsNotNone(planned[0].lane_change)
        self.assertEqual(planned[0].successors[0].kind, "merge")
        self.assertEqual(planned[1].successors[0].kind, "split")
        self.assertTrue(long.net.connect_lane_sequence(
            planned, long_corridor.gps_uids).valid)

        # The middle lane remains physically continuous through the same
        # explicit merge/split topology and therefore is safe to publish.
        middle = m.match_on(first, 1, (1, 2, 3, 4))
        continuous, reason = m.net.select_lane_sequence(corridor, middle)
        self.assertEqual(reason, "")
        self.assertEqual([lane.lane_index for lane in continuous], [1, 1, 1])
        self.assertTrue(m.net.connect_lane_sequence(
            continuous, corridor.gps_uids).valid)

    def test_missing_prefab_description_reports_pair_token_and_index(self):
        m = SyntheticMap()
        m.node(10, 0, 0); m.node(20, 0, 20)
        m.net._missing_prefab_pairs[(10, 20)] = {"blkw_1401i"}
        corridor = m.net.resolve_gps_corridor((10, 20))
        self.assertFalse(corridor.valid)
        self.assertEqual(corridor.failure_reason,
                         "missing prefab description blkw_1401i for GPS UID "
                         "pair 10 -> 20 at index 0")

        # A damaged unrelated prefab item must not shadow a complete direct
        # road edge for the same pair.
        road = SyntheticMap()
        road.node(10, 0, 0); road.node(20, 0, 20)
        road.road(10, 20, 1)
        road.net._missing_prefab_pairs[(10, 20)] = {"damaged-prefab"}
        corridor = road.net.resolve_gps_corridor((10, 20))
        self.assertTrue(corridor.valid, corridor.failure_reason)
        self.assertEqual(corridor.edges[0].kind, "road")

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
