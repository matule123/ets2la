import os
import math
import unittest

from core.navigation.lane_model import LaneLocator, wrap_angle
from core.navigation.road_network import RoadNetwork


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATASET = os.path.join(ROOT, "map-cache", "promods-1.59")


@unittest.skipUnless(os.path.isdir(DATASET), "ProMods 1.59 dataset not installed")
class RealMapLaneDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.net = RoadNetwork()
        assert cls.net.load(DATASET)

    def incoming_match(self, uid, gps_uids, lane_index=0):
        lanes = [lane for lane in self.net.lane_segments_near(
                 self.net.nodes[uid], 45.0)
                 if lane.end_uid == uid and lane.lane_index == lane_index]
        self.assertTrue(lanes, f"no incoming lane at UID {uid}")
        point = lanes[0].centerline[-2]
        match = LaneLocator(self.net).locate(
            (point.x, point.y, point.z), point.heading, gps_uids)
        self.assertIsNotNone(match)
        return match

    def print_metrics(self, label, gps_uids, path):
        gaps = [math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                for a, b in zip(path.points, path.points[1:])]
        jumps = [abs(math.degrees(wrap_angle(b.heading - a.heading)))
                 for a, b in zip(path.points, path.points[1:])]
        heights = [abs(b.y - a.y)
                   for a, b in zip(path.points, path.points[1:])]
        prefab_count = sum(segment.lane_id.prefab_token not in (None, "graph")
                           for segment in path.segments)
        print(
            f"\n[{label}] GPS UID={len(gps_uids)} "
            f"LaneSegment={len(path.segments)} prefab={prefab_count} "
            f"length={path.distance_m:.2f}m max_gap={max(gaps, default=0):.2f}m "
            f"max_heading_jump={max(jumps, default=0):.2f}deg "
            f"height_continuity={max(heights, default=0):.3f}m "
            f"confidence={path.confidence:.3f} "
            f"failure_reason={path.failure_reason or '-'}")

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

    def test_known_prefab_pair_uses_full_lane_curve_chain(self):
        gps = (3764330771318505475, 3808790278165430272)
        corridor = self.net.resolve_gps_corridor(gps)
        self.assertTrue(corridor.valid, corridor.failure_reason)
        self.assertEqual(corridor.edges[0].kind, "prefab")
        match = self.incoming_match(gps[0], gps)
        segments, reason = self.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        self.assertEqual(segments[0].lane_id.prefab_token, "ibe94")
        self.assertEqual(segments[0].lane_id.connector_index, 2)
        self.assertEqual(segments[0].connector_curve_indices,
                         (2, 4, 14, 12, 10, 11, 8, 7))
        path = self.net.connect_lane_sequence(segments, gps)
        self.print_metrics("known-prefab-pair", gps, path)
        self.assertTrue(path.valid, path.failure_reason)
        self.assertGreater(len(path.points), 20)
        self.assertLess(max(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                            for a, b in zip(path.points, path.points[1:])), 4.0)

    def test_roundabout_selects_authoritative_exit(self):
        start = 5462850010004422086
        first_exit = 5462850012948823206
        second_exit = 5462850010641956039
        match = self.incoming_match(start, (start, first_exit), lane_index=0)
        paths = []
        for goal in (first_exit, second_exit):
            gps = (start, goal)
            corridor = self.net.resolve_gps_corridor(gps)
            self.assertTrue(corridor.valid, corridor.failure_reason)
            segments, reason = self.net.select_lane_sequence(corridor, match)
            self.assertEqual(reason, "")
            path = self.net.connect_lane_sequence(segments, gps)
            self.print_metrics(f"roundabout-exit-{goal}", gps, path)
            self.assertTrue(path.valid, path.failure_reason)
            self.assertEqual(path.segments[-1].end_uid, goal)
            self.assertEqual(path.segments[-1].lane_type, "roundabout")
            paths.append(path)
        self.assertNotEqual(
            (paths[0].points[-1].x, paths[0].points[-1].z),
            (paths[1].points[-1].x, paths[1].points[-1].z))
        self.assertNotEqual(paths[0].segments[-1].lane_id,
                            paths[1].segments[-1].lane_id)
        self.assertNotEqual(paths[0].segments[-1].lane_id.connector_path,
                            paths[1].segments[-1].lane_id.connector_path)

    def test_long_real_promods_sequence(self):
        gps = (
            3387693061483872985, 3387693063555859135,
            3387693063476167462, 3387693064285668028,
            3387693065049031437, 3387693064101118710,
            3387693061467095984, 3387693064109507326,
            3387693062708609794, 3387693061966218143,
            3387693064323417066, 3387693064621212532,
        )
        corridor = self.net.resolve_gps_corridor(gps)
        self.assertTrue(corridor.valid, corridor.failure_reason)
        first_edge = corridor.edges[0]
        lane = next(lane for lane in self.net._build_lane_segments(
                    first_edge.segment_index)
                    if lane.start_uid == first_edge.start_uid)
        point = lane.centerline[len(lane.centerline) // 2]
        match = LaneLocator(self.net).locate(
            (point.x, point.y, point.z), point.heading, gps)
        segments, reason = self.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        path = self.net.connect_lane_sequence(segments, gps)
        self.print_metrics("long-promods-sequence", gps, path)
        self.assertTrue(path.valid, path.failure_reason)
        self.assertEqual(len(segments), 11)
        self.assertGreater(path.distance_m, 500.0)


if __name__ == "__main__":
    unittest.main()
