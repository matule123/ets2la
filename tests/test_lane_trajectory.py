import math
import os
import time
import unittest
from dataclasses import replace

from core.navigation.lane_model import (
    LaneConnection, LaneId, LaneLocator, LanePath, LanePoint, LaneSegment,
)
from core.navigation.lane_trajectory import (
    _count_self_intersections, build_lane_trajectory, derive_display_points,
    validate_lane_trajectory,
)
from core.navigation.road_network import RoadNetwork
from tests.test_lane_route_builder import SyntheticMap


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATASET = os.path.join(ROOT, "map-cache", "promods-1.59")


def path_length(points):
    return sum(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
               for a, b in zip(points, points[1:]))


def single_lane_path(coords, lane_type="road", width=4.5, uid=1):
    lane_id = LaneId(uid, 1, 0)
    points = tuple(LanePoint(float(x), float(y), float(z))
                   for x, y, z in coords)
    segment = LaneSegment(
        lane_id, uid, uid+1, 1, 0, 1, width, "derived",
        int(round(points[len(points)//2].y / 3)), "test",
        lane_type, points, gps_uids=frozenset((uid, uid+1)))
    return LanePath((segment,), points, (uid, uid+1), path_length(points),
                    0.99, True)


class LaneTrajectorySyntheticTests(unittest.TestCase):
    def assert_valid_trajectory(self, source):
        trajectory = build_lane_trajectory(source)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        validation = validate_lane_trajectory(trajectory)
        self.assertTrue(validation.valid, validation.failure_reason)
        self.assertLessEqual(validation.max_spacing_m, 2.05)
        self.assertEqual((trajectory.points[0].x, trajectory.points[0].y,
                          trajectory.points[0].z),
                         (source.segments[0].centerline[0].x,
                          source.segments[0].centerline[0].y,
                          source.segments[0].centerline[0].z))
        self.assertEqual((trajectory.points[-1].x, trajectory.points[-1].y,
                          trajectory.points[-1].z),
                         (source.segments[-1].centerline[-1].x,
                          source.segments[-1].centerline[-1].y,
                          source.segments[-1].centerline[-1].z))
        return trajectory, validation

    def test_straight_road_uniform_resampling_and_display_derivation(self):
        source = single_lane_path([(0, 0, 0), (0, 0, 17), (0, 0, 41)])
        trajectory, validation = self.assert_valid_trajectory(source)
        self.assertAlmostEqual(validation.average_spacing_m, 2.0, delta=0.1)
        display = derive_display_points(trajectory, 4.0)
        self.assertGreater(len(trajectory.points), len(display))
        self.assertEqual((display[0].x, display[0].z), (0.0, 0.0))
        self.assertEqual((display[-1].x, display[-1].z), (0.0, 41.0))

    def test_isolated_straight_road_bump_is_faired_inside_corridor(self):
        source = single_lane_path([
            (0, 0, 0), (0, 0, 10), (1.2, 0.7, 20),
            (0, 0, 30), (0, 0, 40),
        ])
        trajectory, validation = self.assert_valid_trajectory(source)
        middle = min(trajectory.points, key=lambda point: abs(point.z - 20.0))
        self.assertLess(abs(middle.x), 0.75)
        self.assertLess(abs(middle.y), 0.45)
        self.assertLess(validation.max_corridor_deviation_m, 0.75)

    def test_spatial_self_intersection_check_scales_to_100_km(self):
        segments = []
        for index in range(1000):
            lane_id = LaneId(10_000 + index, 1, 0)
            points = (LanePoint(0.0, 0.0, index * 100.0),
                      LanePoint(0.0, 0.0, (index + 1) * 100.0))
            segments.append(LaneSegment(
                lane_id, index + 1, index + 2, 1, 0, 1, 4.5,
                "derived", 0, "long-straight", "road", points,
                gps_uids=frozenset((index + 1, index + 2))))
        for index in range(len(segments) - 1):
            segments[index] = replace(
                segments[index],
                successors=(LaneConnection(segments[index + 1].lane_id,
                                           "road"),))
        source = LanePath(tuple(segments), (), tuple(range(1, 1002)),
                          100_000.0, 0.99, True)
        started = time.monotonic()
        trajectory = build_lane_trajectory(source)
        elapsed = time.monotonic() - started
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        self.assertGreater(len(trajectory.points), 49_000)
        self.assertAlmostEqual(trajectory.distance_m, 100_000.0, delta=1.0)
        # The previous quadratic scan does not complete this case in a useful
        # runtime. Keep a generous bound for slower CI machines.
        self.assertLess(elapsed, 25.0)

    def test_spatial_self_intersection_check_still_detects_crossing(self):
        points = tuple(LanePoint(x, 0.0, z) for x, z in (
            (0, 0), (10, 10), (20, 0), (10, -10), (0, 0),
            (10, 10), (0, 20), (10, 0),
        ))
        self.assertGreater(_count_self_intersections(points), 0)

    def test_smooth_curve(self):
        radius = 30.0
        coords = [(radius*(1-math.cos(t)), 0, radius*math.sin(t))
                  for t in [i*math.pi/40 for i in range(11)]]
        _, validation = self.assert_valid_trajectory(single_lane_path(coords))
        self.assertLess(validation.max_curvature, 0.06)

    def test_sharp_but_continuous_curve(self):
        radius = 8.0
        coords = [(radius*(1-math.cos(t)), 0, radius*math.sin(t))
                  for t in [i*math.pi/48 for i in range(25)]]
        _, validation = self.assert_valid_trajectory(single_lane_path(coords))
        self.assertGreater(validation.max_curvature, 0.08)
        self.assertLess(validation.max_heading_jump_deg, 38.0)

    def test_s_curve(self):
        coords = [(5.0*math.sin(z/18.0), 0, float(z))
                  for z in range(0, 91, 3)]
        trajectory, validation = self.assert_valid_trajectory(
            single_lane_path(coords))
        signs = {1 if point.curvature > 0.002 else -1
                 for point in trajectory.points if abs(point.curvature) > 0.002}
        self.assertEqual(signs, {-1, 1})
        self.assertEqual(validation.self_intersections, 0)

    def test_merge_and_split_boundaries_keep_lane_identity(self):
        m = SyntheticMap()
        for uid, z in enumerate((0, 40, 80, 120), 1):
            m.node(uid, 0, z)
        first = m.road(1, 2, 3); m.road(2, 3, 2); m.road(3, 4, 3)
        match = m.match_on(first, 1, (1, 2, 3, 4))
        corridor = m.net.resolve_gps_corridor((1, 2, 3, 4))
        segments, reason = m.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        source = m.net.connect_lane_sequence(segments, corridor.gps_uids)
        trajectory, _ = self.assert_valid_trajectory(source)
        self.assertEqual([segment.successors[0].kind
                          for segment in segments[:-1]], ["merge", "split"])
        self.assertEqual({point.segment_index for point in trajectory.points},
                         {0, 1, 2})

    def test_bridge_and_road_below_remain_on_separate_heights(self):
        lower, _ = self.assert_valid_trajectory(single_lane_path(
            [(0, 0, 0), (0, 0, 40)], uid=20))
        upper, _ = self.assert_valid_trajectory(single_lane_path(
            [(0, 12, 0), (0, 12, 40)], uid=30))
        self.assertTrue(all(point.y == 0 for point in lower.points))
        self.assertTrue(all(point.y == 12 for point in upper.points))

    def test_invalid_segment_gap_is_not_repaired(self):
        first = single_lane_path([(0, 0, 0), (0, 0, 20)], uid=40).segments[0]
        second = single_lane_path([(10, 0, 20), (10, 0, 40)], uid=50).segments[0]
        second = LaneSegment(
            second.lane_id, first.end_uid, 52, 1, 0, 1, 4.5, "derived", 0,
            "test", "road", second.centerline)
        first = LaneSegment(
            first.lane_id, first.start_uid, first.end_uid, 1, 0, 1, 4.5,
            "derived", 0, "test", "road", first.centerline,
            successors=(LaneConnection(second.lane_id, "road"),))
        source = LanePath((first, second), first.centerline+second.centerline,
                          (40, 41, 52), 50, 0.9, True)
        trajectory = build_lane_trajectory(source)
        self.assertFalse(trajectory.valid)
        self.assertIn("gap", trajectory.failure_reason)

    def test_invalid_heading_jump_is_rejected(self):
        source = single_lane_path([(0, 0, 0), (0, 0, 10),
                                   (10, 0, 10), (20, 0, 10)], uid=60)
        trajectory = build_lane_trajectory(source)
        self.assertFalse(trajectory.valid)
        self.assertIn("heading jump", trajectory.failure_reason)


@unittest.skipUnless(os.path.isdir(DATASET), "ProMods 1.59 dataset not installed")
class LaneTrajectoryRealMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.net = RoadNetwork()
        assert cls.net.load(DATASET)

    def incoming_match(self, uid, gps):
        lane = next(lane for lane in self.net.lane_segments_near(
                    self.net.nodes[uid], 45.0)
                    if lane.end_uid == uid and lane.lane_index == 0)
        point = lane.centerline[-2]
        return LaneLocator(self.net).locate(
            (point.x, point.y, point.z), point.heading, gps)

    def build_source(self, gps, match=None):
        corridor = self.net.resolve_gps_corridor(gps)
        self.assertTrue(corridor.valid, corridor.failure_reason)
        if match is None:
            match = self.incoming_match(gps[0], gps)
        segments, reason = self.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        source = self.net.connect_lane_sequence(segments, gps)
        self.assertTrue(source.valid, source.failure_reason)
        return source

    def print_metrics(self, label, source, trajectory):
        validation = validate_lane_trajectory(trajectory)
        print(
            f"\n[{label}] input={sum(len(s.centerline) for s in source.segments)} "
            f"output={len(trajectory.points)} "
            f"length={validation.original_length_m:.2f}->{validation.result_length_m:.2f}m "
            f"spacing_avg/max={validation.average_spacing_m:.2f}/"
            f"{validation.max_spacing_m:.2f}m "
            f"heading_jump={validation.max_heading_jump_deg:.2f}deg "
            f"curvature/jump={validation.max_curvature:.4f}/"
            f"{validation.max_curvature_jump:.4f} "
            f"corridor_deviation={validation.max_corridor_deviation_m:.3f}m "
            f"height_jump={validation.max_height_jump_m:.3f}m "
            f"confidence={trajectory.confidence:.3f} "
            f"failure_reason={trajectory.failure_reason or '-'}")
        return validation

    def assert_real_valid(self, label, source):
        trajectory = build_lane_trajectory(source)
        validation = self.print_metrics(label, source, trajectory)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        self.assertTrue(validation.valid, validation.failure_reason)
        self.assertTrue(derive_display_points(trajectory, 4.0))
        return trajectory

    def test_prefab_ibe94(self):
        gps = (3764330771318505475, 3808790278165430272)
        self.assert_real_valid("trajectory-ibe94", self.build_source(gps))

    def test_roundabout_both_exits(self):
        start = 5462850010004422086
        match = self.incoming_match(start, (start, 5462850012948823206))
        for goal in (5462850012948823206, 5462850010641956039):
            source = self.build_source((start, goal), match)
            trajectory = self.assert_real_valid(
                f"trajectory-roundabout-{goal}", source)
            self.assertEqual(trajectory.segments[-1].end_uid, goal)

    def test_long_real_promods_lane_path(self):
        gps = (
            3387693061483872985, 3387693063555859135,
            3387693063476167462, 3387693064285668028,
            3387693065049031437, 3387693064101118710,
            3387693061467095984, 3387693064109507326,
            3387693062708609794, 3387693061966218143,
            3387693064323417066, 3387693064621212532,
        )
        corridor = self.net.resolve_gps_corridor(gps)
        first = corridor.edges[0]
        lane = next(lane for lane in self.net._build_lane_segments(
                    first.segment_index) if lane.start_uid == first.start_uid)
        point = lane.centerline[len(lane.centerline)//2]
        match = LaneLocator(self.net).locate(
            (point.x, point.y, point.z), point.heading, gps)
        trajectory = self.assert_real_valid(
            "trajectory-long-promods", self.build_source(gps, match))
        self.assertGreater(len(trajectory.points), 300)


if __name__ == "__main__":
    unittest.main()
