import os
import math
import unittest
from dataclasses import replace

from core.navigation.lane_model import LaneLocator, wrap_angle
from core.navigation.lane_trajectory import (
    build_lane_trajectory, derive_display_points, validate_lane_trajectory,
)
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

    def assert_captured_route_valid(self, gps, position, heading):
        match = LaneLocator(self.net).locate(position, heading, gps)
        self.assertIsNotNone(match)
        path, returned = self.net.build_lane_path(
            gps, (position[0], position[2]), heading,
            altitude=position[1], start_match=match)
        self.assertIsNotNone(returned)
        self.assertTrue(path.valid, path.failure_reason)
        trajectory = build_lane_trajectory(path)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        validation = validate_lane_trajectory(trajectory)
        self.assertTrue(validation.valid, validation.failure_reason)
        return path, trajectory

    def test_phase1_localizes_three_captured_prefab_boundary_poses(self):
        captures = (
            (
                (5962819264745725766, 5962819261197344539,
                 5962819257825124179, 5962819264393417967,
                 5962819277395732982, 5962819251021962418),
                (41933.31673049927, 59.50890350341797,
                 61005.002868652344), 0.676961338637013,
            ),
            (
                (5962819253681172399, 5962819261264473948,
                 5962819264712191947, 5962819264636694476),
                (41886.58908843994, 69.0354232788086,
                 61468.78616142273), 2.5009504447960342,
            ),
            (
                (5962819253597286226, 5962819260593385297,
                 5962819260786301797),
                (42201.28845214844, 67.72754669189453,
                 61596.53224182129), 3.120494987404456,
            ),
        )
        for gps, position, heading in captures:
            with self.subTest(gps=gps):
                self.assert_captured_route_valid(gps, position, heading)

    def test_2026_07_29_adjacent_junction_lane_fails_before_chord(self):
        """Replay the last real intersection failure from route diagnostics."""
        gps = (
            5962819257346994300, 5962819250132791422,
            5962819264754134939, 5962819262464044943,
            5962819253924441988, 5962819266230529939,
            5962819258336849769, 5962819250829045611,
            5962819265894985573, 5962819266272472931,
            5962819260727582455, 5962819256013183871,
            5962819264745725766, 5962819261197344539,
            5962819257825124179, 5962819264393417967,
            5962819277395732982, 5962819251021962418,
            5962819270986837516, 5962819255350497429,
            5962819262178825461, 5962819266725450992,
            5962819280986057213, 5962819263655219268,
            5962819262229155909,
        )
        position = (42085.586853027344, 59.514347076416016,
                    61259.12966918945)
        heading = -0.8143520507687896
        match = LaneLocator(self.net).locate(position, heading, gps)
        self.assertIsNotNone(match)
        self.assertEqual(match.lane_id.lane_index, 1)

        path, returned = self.net.build_lane_path(
            gps, (position[0], position[2]), heading,
            altitude=position[1], start_match=match)
        self.assertEqual(returned.lane_id, match.lane_id)
        self.assertFalse(path.valid)
        self.assertEqual(len(path.points), 0)
        self.assertEqual(path.failure_reason,
            "LANE_CHANGE_INSUFFICIENT_APPROACH: available approach 8.73 m "
            "is shorter than required 42.75 m")
        # A rejected transition must not reach trajectory resampling, where
        # the old one-sample chord appeared as a 52.16-degree heading jump.
        trajectory = build_lane_trajectory(path)
        self.assertFalse(trajectory.valid)
        self.assertNotIn("heading jump", trajectory.failure_reason)

    def test_2026_07_30_parallel_prefab_lane_uses_exact_ppd_sibling(self):
        """route_build_id=e54cd411da724b68af363bc0ebb935a0.

        dlc_blkw_94's navNode connection contains only the representative
        curve 1, 4.50 m beside the confirmed incoming lane. The PPD lane
        tables prove parallel curve 0 for the same physical GPS endpoints.
        It must be selected without a chord or a validation-limit increase.
        """
        road_edge = self.net._classify_corridor_edge(
            5337536181859392302, 5337536182148795575, 0)
        self.assertEqual(road_edge.kind, "road")
        incoming_lane = next(
            lane for lane in self.net._build_lane_segments(
                road_edge.segment_index)
            if lane.start_uid == road_edge.start_uid
            and lane.end_uid == road_edge.end_uid
            and lane.raw_lane_index == 1)
        prefab_edge = self.net._classify_corridor_edge(
            5337536182148795575, 5337536182526315133, 1)
        self.assertEqual(prefab_edge.kind, "prefab")
        instance = prefab_edge.prefab_instance[0]
        representative = self.net._prefab_connector_options(
            instance, prefab_edge.start_uid, prefab_edge.end_uid)
        self.assertEqual(representative, [(1,)])
        representative_points = self.net._prefab_curve_chain_3d(
            instance, representative[0])
        self.assertAlmostEqual(math.dist(
            (incoming_lane.centerline[-1].x,
             incoming_lane.centerline[-1].y,
             incoming_lane.centerline[-1].z),
            (representative_points[0].x, representative_points[0].y,
             representative_points[0].z)), 4.5, places=3)

        selected, reason = self.net._prefab_lane_segment(
            prefab_edge, incoming_lane.lane_index,
            incoming_lane.centerline[-1], register=False,
            allow_parallel_sibling=True)
        self.assertEqual(reason, "")
        self.assertEqual(selected.lane_id.connector_path, (0,))
        self.assertLess(math.dist(
            (incoming_lane.centerline[-1].x,
             incoming_lane.centerline[-1].y,
             incoming_lane.centerline[-1].z),
            (selected.centerline[0].x, selected.centerline[0].y,
             selected.centerline[0].z)), 0.01)

    def test_2026_07_31_parallel_lane_continues_across_two_prefabs(self):
        """route_build_id=0cc3209ceb5245418158a5f76eae2410.

        The live log proves a valid outer lane entering dlc_blkw_94 curve 0.
        Its next GPS edge immediately enters dlc_blkw_105, whose navNode
        representative starts 4.50 m away.  PPD input/output lane topology
        proves sibling (6, 1, 7) at the exact same endpoint and heading.
        """
        gps = (
            5337536179598665046, 5337536179854516426,
            5337536180630458241, 5337536180324276875,
            5337536233533273874,
        )
        corridor = self.net.resolve_gps_corridor(gps)
        self.assertTrue(corridor.valid)
        match = self.incoming_match(gps[1], gps, lane_index=1)
        segments, reason = self.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        self.assertGreaterEqual(len(segments), 3)
        self.assertEqual(segments[1].lane_id.prefab_token, "dlc_blkw_94")
        self.assertEqual(segments[1].lane_id.connector_path, (0,))
        self.assertEqual(segments[2].lane_id.prefab_token, "dlc_blkw_105")
        self.assertEqual(segments[2].lane_id.connector_path, (6, 1, 7))
        gap = math.dist(
            (segments[1].centerline[-1].x, segments[1].centerline[-1].y,
             segments[1].centerline[-1].z),
            (segments[2].centerline[0].x, segments[2].centerline[0].y,
             segments[2].centerline[0].z),
        )
        heading_jump = abs((segments[2].centerline[0].heading
                            - segments[1].centerline[-1].heading + math.pi)
                           % (2 * math.pi) - math.pi)
        self.assertLess(gap, 0.01)
        self.assertLess(math.degrees(heading_jump), 1.0)
        path = self.net.connect_lane_sequence(segments, gps)
        trajectory = build_lane_trajectory(path)
        self.assertTrue(path.valid, path.failure_reason)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)

    def test_2026_07_31_adjacent_prefab_lane_uses_proven_approach_changes(self):
        """Replay the real 4.50 m/49.8 degree failure without a chord.

        Two long ordinary-road approaches prove reciprocal adjacent lanes and
        exact PPD target connectors.  Phase 3 may plan on those roads while
        retaining both original map centrelines and every GPS pair identity.
        """
        gps = (
            5337536181112822720, 5337536178877240122,
            5337536181687423927, 5337536180630458150,
            5337536182782138148, 5337536182522091770,
            5337536179686741705, 5337536182727615511,
            5337536180336860780, 5337536182220102206,
            5337536182589199430, 5337536178910797982,
            5337536179598665046, 5337536179854516426,
            5337536180630458241, 5337536180324276875,
            5337536233533273874, 5337536179405721942,
        )
        corridor = self.net.resolve_gps_corridor(gps)
        self.assertTrue(corridor.valid, corridor.failure_reason)
        match = self.incoming_match(gps[0], gps, lane_index=1)
        failure_details = {}
        segments, reason = self.net.select_lane_sequence(
            corridor, match, failure_details=failure_details)
        self.assertEqual(reason, "")
        self.assertEqual(len(segments), 17)
        self.assertEqual(segments[0].gps_pair_index, 0)
        self.assertEqual(segments[0].raw_lane_index, 1)
        self.assertEqual(corridor.edges[1].prefab_instance[0][0],
                         "dlc_blkw_83")
        changes = [segment for segment in segments if segment.lane_change]
        self.assertEqual(len(changes), 2)
        self.assertEqual(len(failure_details["lane_changes"]), 2)
        self.assertEqual(
            {change.lane_change.prefab_token for change in changes},
            {"dlc_blkw_83", "dlc_blkw_80"})
        for change in changes:
            proof = change.lane_change
            self.assertNotEqual(proof.source_lane_id, proof.target_lane_id)
            self.assertEqual(abs(proof.source_raw_lane_index
                                 - proof.target_raw_lane_index), 1)
            self.assertGreaterEqual(proof.available_length_m,
                                    proof.required_length_m)
            cached = self.net._lane_id_index[proof.source_lane_id]
            self.assertNotEqual(change.centerline, cached.centerline)
            self.assertEqual(proof.source_centerline, cached.centerline)
        path = self.net.connect_lane_sequence(segments, gps)
        trajectory = build_lane_trajectory(path)
        validation = validate_lane_trajectory(trajectory)
        self.assertTrue(path.valid, path.failure_reason)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        self.assertTrue(validation.valid, validation.failure_reason)
        self.assertEqual(validation.lane_change_count, 2)

    def test_phase1_prefab_diagnostic_replay_does_not_mutate_lane_cache(self):
        gps = (
            5962819253681172399, 5962819261264473948,
            5962819264712191947, 5962819264636694476,
        )
        position = (41886.58908843994, 69.0354232788086,
                    61468.78616142273)
        heading = 2.5009504447960342
        locator = LaneLocator(self.net)
        match = locator.locate(position, heading, gps)
        self.assertIsNotNone(match)
        lane_cache = dict(self.net._lane_cache)
        lane_index = dict(self.net._lane_id_index)
        road_pair_index = dict(self.net._road_pair_index_cache)
        spatial_indexes = {
            name: (id(getattr(self.net, name)), len(getattr(self.net, name)))
            for name in ("_ngrid", "_grid", "_seg_grid", "_prefab_grid",
                         "_prefab_pairs")
        }
        capture = {}
        observed = locator.locate(
            position, heading, gps, match, diagnostics=capture,
            diagnostic_mode=True)
        self.assertIsNotNone(observed)
        self.assertEqual(observed.lane_id, match.lane_id)
        self.assertEqual(observed.point, match.point)
        self.assertIs(locator.previous, match)
        self.assertEqual(self.net._lane_cache, lane_cache)
        self.assertEqual(self.net._lane_id_index, lane_index)
        self.assertEqual(self.net._road_pair_index_cache, road_pair_index)
        self.assertEqual({
            name: (id(getattr(self.net, name)), len(getattr(self.net, name)))
            for name in spatial_indexes
        }, spatial_indexes)

    def test_phase1_legacy_order_restores_captured_missing_connector(self):
        gps = (
            5962819260727582455, 5962819266272472931,
            5962819260870209380, 5962819259855187862,
            5962819264846409621, 5962819270055709408,
            5962819273662810836, 5962819260803100519,
            5962819254058659704, 5962819259569975345,
            5962819266264084346, 5962819254041882580,
            5962819257288273893,
        )
        pair = (5962819259855187862, 5962819264846409621)
        instance = self.net._prefab_pairs[(min(pair), max(pair))][0]
        self.assertEqual(
            instance[1],
            (5962819264846409621, 5962819259855187862,
             5962819264569585556))
        self.assertEqual(
            self.net._prefab_connector_options(instance, *pair), [(3, 4)])
        self.assert_captured_route_valid(
            gps,
            (41966.15946960449, 59.51948165893555, 61062.694396972656),
            -2.4511833293273426)

    def test_phase1_legacy_order_removes_captured_18m_prefab_gap(self):
        gps = (
            5962819260593385297, 5962819260786301797,
            5962819255727992656, 5962819266683514703,
            5962819254075415764, 5962819253060394194,
            5962819256810101952, 5962819261331562124,
            5962819268579312546,
        )
        path, _trajectory = self.assert_captured_route_valid(
            gps,
            (42197.8532409668, 67.53638458251953, 61634.702560424805),
            2.960869605749446)
        gaps = [math.dist(
            (first.centerline[-1].x, first.centerline[-1].y,
             first.centerline[-1].z),
            (second.centerline[0].x, second.centerline[0].y,
             second.centerline[0].z))
            for first, second in zip(path.segments, path.segments[1:])]
        self.assertLess(max(gaps, default=0.0), 6.0)

    def test_phase3_exact_prefab_sibling_removes_phase1_lane_shift_failure(self):
        gps = (
            5962819255727992656, 5962819266683514703,
            5962819254075415764, 5962819251944709334,
            5962819252473191639, 5962819266733850743,
            5962819280843480628, 5962819250728386678,
        )
        position = (42366.01986694336, 59.59323501586914,
                    61849.797927856445)
        heading = -2.1086505875894694
        match = LaneLocator(self.net).locate(position, heading, gps)
        self.assertIsNotNone(match)
        path, _ = self.net.build_lane_path(
            gps, (position[0], position[2]), heading,
            altitude=position[1], start_match=match)
        trajectory = build_lane_trajectory(path)
        validation = validate_lane_trajectory(trajectory)
        self.assertTrue(path.valid, path.failure_reason)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        self.assertTrue(validation.valid, validation.failure_reason)
        self.assertEqual(validation.lane_change_count, 0)
        prefabs = [segment for segment in path.segments
                   if segment.lane_id.prefab_token]
        self.assertEqual(prefabs[0].lane_id.connector_path,
                         (12, 36, 1, 60))
        self.assertEqual(prefabs[1].lane_id.connector_path, (6, 1, 7))

    def test_phase3_replays_624f89_with_exact_prefab_continuation(self):
        gps = (
            5962819250946497307, 5962819255744781085,
            5962819252305451794, 5962819259024692285,
            5962819252766829126, 5962819255425996029,
            5962819251760174334, 5962819264049468519,
            5962819263453877369, 5962819262229155909,
            5962819263655219268, 5962819280986057213,
            5962819266725450992, 5962819262178825461,
            5962819255350497429, 5962819270986837516,
            5962819251021962418, 5962819277395732982,
            5962819264393417967, 5962819257825124179,
            5962819261197344539, 5962819264745725766,
            5962819256013183871, 5962819260727582455,
            5962819266272472931, 5962819260870209380,
            5962819259855187862, 5962819264846409621,
            5962819270055709408, 5962819273662810836,
            5962819260803100519, 5962819254058659704,
            5962819259569975345, 5962819266264084346,
            5962819254041882580, 5962819257288273893,
            5962819257816756200, 5962819265030959063,
            5962819253681172399, 5962819261264473948,
            5962819264712191947, 5962819264636694476,
            5962819258999549903, 5962819258538176462,
            5962819256021594061, 5962819251122646873,
            5962819253597286226, 5962819260593385297,
            5962819260786301797, 5962819255727992656,
            5962819266683514703, 5962819254075415764,
            5962819251944709334, 5962819252473191639,
            5962819266733850743, 5962819280843480628,
            5962819250728386678,
        )
        position = (40970.454233169556, 34.807640075683594,
                    60294.166259765625)
        heading = -1.201390458734874
        path, trajectory = self.assert_captured_route_valid(
            gps, position, heading)
        validation = validate_lane_trajectory(trajectory)
        self.assertEqual(validation.lane_change_count, 0)
        first = next(segment for segment in path.segments
                     if segment.lane_id.prefab_token == "dlc_blkw_81"
                     and segment.end_uid == 5962819251944709334)
        second = next(segment for segment in path.segments
                      if segment.start_uid == 5962819251944709334
                      and segment.lane_id.prefab_token == "dlc_blkw_105")
        self.assertEqual(first.lane_id.connector_path, (12, 36, 1, 60))
        self.assertEqual(second.lane_id.connector_path, (6, 1, 7))
        self.assertLess(math.dist(
            (first.centerline[-1].x, first.centerline[-1].y,
             first.centerline[-1].z),
            (second.centerline[0].x, second.centerline[0].y,
             second.centerline[0].z)), 0.01)
        self.assertAlmostEqual(path.distance_m, 3607.65, delta=0.5)

    def test_phase3_replays_e54cd4_prefab_pair_without_4_5m_chord(self):
        previous_edge = self.net._classify_corridor_edge(
            5337536180030676173, 5337536182018777762, 0)
        target_edge = self.net._classify_corridor_edge(
            5337536182018777762, 5337536178692692820, 1)
        self.assertEqual((previous_edge.kind, target_edge.kind),
                         ("prefab", "prefab"))
        previous_instance = previous_edge.prefab_instance[0]
        previous_points = self.net._prefab_curve_chain_3d(
            previous_instance, (7, 1, 5, 9, 10))
        previous = self.net._make_prefab_lane_segment(
            previous_edge, previous_instance, (7, 1, 5, 9, 10),
            previous_points, 1)

        representative_instance = target_edge.prefab_instance[0]
        representative_points = self.net._prefab_curve_chain_3d(
            representative_instance, (2,))
        self.assertAlmostEqual(math.dist(
            (previous.centerline[-1].x, previous.centerline[-1].y,
             previous.centerline[-1].z),
            (representative_points[0].x, representative_points[0].y,
             representative_points[0].z)), 4.498488, places=3)
        selected, reason = self.net._prefab_lane_segment(
            target_edge, previous.lane_index, previous.centerline[-1],
            register=False, allow_parallel_sibling=True)
        self.assertEqual(reason, "")
        self.assertEqual(selected.lane_id.connector_path, (1,))
        self.assertLess(math.dist(
            (previous.centerline[-1].x, previous.centerline[-1].y,
             previous.centerline[-1].z),
            (selected.centerline[0].x, selected.centerline[0].y,
             selected.centerline[0].z)), 0.01)
        previous = replace(
            previous,
            successors=(self.net._lane_connection(previous, selected),))
        path = self.net.connect_lane_sequence(
            (previous, selected),
            (5337536180030676173, 5337536182018777762,
             5337536178692692820))
        trajectory = build_lane_trajectory(path)
        self.assertTrue(path.valid, path.failure_reason)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)

    def test_phase3_e54cd4_later_dual_output_stays_fail_closed(self):
        previous_edge = self.net._classify_corridor_edge(
            5337536180831803664, 5337536180890523938, 196)
        ambiguous_edge = self.net._classify_corridor_edge(
            5337536180890523938, 5337536181700024638, 197)
        previous_instance = previous_edge.prefab_instance[0]
        previous_points = self.net._prefab_curve_chain_3d(
            previous_instance, (1,))
        previous = self.net._make_prefab_lane_segment(
            previous_edge, previous_instance, (1,), previous_points, 0)
        instance = ambiguous_edge.prefab_instance[0]
        options = self.net._prefab_connector_options(
            instance, ambiguous_edge.start_uid, ambiguous_edge.end_uid)
        self.assertEqual(options, [
            (0, 27, 2, 3, 18, 24, 22),
            (0, 27, 2, 3, 19, 34),
        ])
        self.assertEqual({option[0] for option in options}, {0})
        self.assertEqual({option[-1] for option in options}, {22, 34})
        for option in options:
            points = self.net._prefab_curve_chain_3d(instance, option)
            self.assertLess(math.dist(
                (previous.centerline[-1].x, previous.centerline[-1].y,
                 previous.centerline[-1].z),
                (points[0].x, points[0].y, points[0].z)), 0.01)
        selected, reason = self.net._prefab_lane_segment(
            ambiguous_edge, previous.lane_index, previous.centerline[-1],
            register=False, allow_parallel_sibling=True)
        self.assertIsNone(selected)
        self.assertEqual(reason, "ambiguous prefab lane connector")

    def test_real_lane_change_capture_keeps_pitched_prefab_continuous(self):
        # route_build_id=c500b655c6624ffe9b73fcf317071dad:
        # the truck is stably localized in lane 1 (0.163 m lateral error), but
        # blkw_1403y's flat-Y transform ended 2.207 m above the following road
        # and resampling reported an artificial 179.931 degree heading jump.
        gps = (
            5962819255744781085, 5962819252305451794,
            5962819259024692285, 5962819252766829126,
            5962819255425996029, 5962819251760174334,
            5962819264049468519, 5962819263453877369,
        )
        path, _trajectory = self.assert_captured_route_valid(
            gps,
            (41018.54218292236, 33.21265411376953,
             60260.180892944336),
            -0.718520145254435,
        )
        prefab_index = next(index for index, segment in enumerate(path.segments)
                            if segment.lane_id.prefab_token == "blkw_1403y")
        before = path.segments[prefab_index]
        after = path.segments[prefab_index + 1]
        self.assertLess(math.dist(
            (before.centerline[-1].x, before.centerline[-1].y,
             before.centerline[-1].z),
            (after.centerline[0].x, after.centerline[0].y,
             after.centerline[0].z)), 0.35)

    def test_real_progressive_lane_change_survives_rolling_gps_window(self):
        # Revisions 248, 249, 252 and 253 from ultrapilot.log. The signed
        # lateral movement progresses 1.660 -> 2.041 -> 2.809 -> 4.480 m from
        # lane 0 while the lane-1 error falls to 0.020 m. Revision 252 also
        # drops the already-passed first GPS UID.
        locator = LaneLocator(self.net)
        full = (
            5962819256667524768, 5962819256197731271,
            5962819259108578392,
        )
        rolled = full[1:]
        samples = (
            ((40428.47589492798, 50.11819839477539,
              60471.55988693237), -0.8192929219930516, full),
            ((40447.95772626996, 50.11360168457031,
              60456.00723648071), -0.9070635126647417, full),
            ((40459.36179637909, 50.11345672607422,
              60447.604835510254), -0.959580633242183, rolled),
            ((40479.433322906494, 50.11083984375,
              60433.173290252686), -0.935485225548879, rolled),
        )
        matches = [locator.locate(position, heading, gps)
                   for position, heading, gps in samples]
        self.assertTrue(all(match is not None for match in matches), matches)
        self.assertEqual([match.lane_id.lane_index for match in matches],
                         [0, 0, 0, 1])
        self.assertEqual(matches[2].switch_reason, "lane_change_pending")
        self.assertEqual(matches[3].switch_reason, "lane_change_confirmed")
        self.assertAlmostEqual(abs(matches[3].lateral_error_m),
                               0.019703162057498575)

    def test_real_consecutive_pitched_prefabs_remove_vertical_gap(self):
        # route_build_id=8f690899d44b49fdb8a00ff2ca02fc95
        # failed at blkw_1401i -> blkw_14038 with only 0.050 m horizontal
        # separation but 1.832 m of missing prefab pitch.
        gps = (
            5962819255744781085, 5962819252305451794,
            5962819259024692285, 5962819252766829126,
            5962819255425996029, 5962819251760174334,
            5962819264049468519, 5962819263453877369,
            5962819262229155909, 5962819263655219268,
            5962819280986057213, 5962819266725450992,
            5962819262178825461, 5962819255350497429,
            5962819270986837516, 5962819251021962418,
            5962819277395732982, 5962819264393417967,
            5962819257825124179, 5962819261197344539,
            5962819264745725766, 5962819256013183871,
            5962819260727582455, 5962819266272472931,
            5962819260870209380, 5962819259855187862,
            5962819264846409621, 5962819270055709408,
            5962819273662810836, 5962819260803100519,
            5962819254058659704, 5962819259569975345,
            5962819266264084346, 5962819254041882580,
            5962819257288273893, 5962819257816756200,
            5962819265030959063,
        )
        path, _trajectory = self.assert_captured_route_valid(
            gps,
            (41018.542194366455, 33.212642669677734,
             60260.18101501465),
            -0.7184652813316748,
        )
        boundary = next(index for index, segment in enumerate(path.segments[:-1])
                        if (segment.lane_id.prefab_token == "blkw_1401i"
                            and path.segments[index + 1].lane_id.prefab_token
                                == "blkw_14038"))
        first, second = path.segments[boundary:boundary + 2]
        self.assertLess(math.dist(
            (first.centerline[-1].x, first.centerline[-1].y,
             first.centerline[-1].z),
            (second.centerline[0].x, second.centerline[0].y,
             second.centerline[0].z)), 0.35)

    def test_real_lane_metadata_is_preserved(self):
        self.assertGreater(len(self.net.road_looks), 1000)
        look = next(value for value in self.net.road_looks.values()
                    if value["lanes_left"] >= 2 and value["lanes_right"] >= 2)
        self.assertEqual(len(look["lane_types_left"]), look["lanes_left"])
        self.assertEqual(len(look["lane_types_right"]), look["lanes_right"])
        self.assertIn("offset_m", look)

    def test_real_blkw2c_legacy_offsets_place_truck_in_outer_lane(self):
        look = self.net.road_looks["blkw2c"]
        self.assertEqual(look["lane_offsets_left_m"], (-4.75, -4.75))
        self.assertEqual(look["lane_offsets_right_m"], (-4.75, -4.75))
        self.assertEqual(self.net._lane_center_offsets(look),
                         ((-3.25, -7.75), (3.25, 7.75)))

        # Captured from the running ETS2 1.59 session that exposed the HUD
        # one-lane shift. With the omitted SII offsets restored, this pose is
        # unambiguously the outer/right lane (raw lane index 1), not index 0.
        match = LaneLocator(self.net).locate(
            (42308.221267700195, 60.259498596191406, 61796.968002319336),
            1.033263954791141)
        self.assertIsNotNone(match)
        self.assertEqual(match.lane_id.road_uid, 5962819239990939632)
        self.assertEqual(match.lane_id.direction, 1)
        self.assertEqual(match.lane_id.lane_index, 1)
        self.assertLess(abs(match.lateral_error_m), 0.35)
        self.assertGreaterEqual(match.confidence, 0.72)

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

    def test_real_player_only_transition_keeps_raw_lane_without_spike(self):
        """Regression for a real ProMods/SCS road→player-only boundary.

        The destination look disables raw lane 0. Compressing drivable lane
        indices connected the previous raw lane 0 to raw lane 1 and inserted a
        4.5 m diagonal. Physical raw lane 1 must remain continuous instead.
        """
        first_index = self.net._road_segment_by_uid[366954835157188609]
        second_index = self.net._road_segment_by_uid[366954883399024641]
        first = next(lane for lane in self.net._build_lane_segments(first_index)
                     if lane.direction == 1 and lane.raw_lane_index == 1)
        second = next(lane for lane in self.net._build_lane_segments(second_index)
                      if lane.direction == 1 and lane.raw_lane_index == 1)
        self.assertEqual((first.lane_index, second.lane_index), (1, 0))
        self.assertLess(math.dist(
            (first.centerline[-1].x, first.centerline[-1].y,
             first.centerline[-1].z),
            (second.centerline[0].x, second.centerline[0].y,
             second.centerline[0].z)), 1e-6)

    def test_known_prefab_pair_uses_full_lane_curve_chain(self):
        gps = (3764330771318505475, 3808790278165430272)
        corridor = self.net.resolve_gps_corridor(gps)
        self.assertTrue(corridor.valid, corridor.failure_reason)
        self.assertEqual(corridor.edges[0].kind, "prefab")
        match = self.incoming_match(gps[0], gps)
        segments, reason = self.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        self.assertEqual(segments[0].lane_id.prefab_token, "ibe94")
        self.assertEqual(segments[0].lane_id.connector_index, 0)
        self.assertEqual(segments[0].connector_curve_indices,
                         (0, 5, 9, 19, 20, 21, 17, 22))
        path = self.net.connect_lane_sequence(segments, gps)
        self.print_metrics("known-prefab-pair", gps, path)
        self.assertTrue(path.valid, path.failure_reason)
        self.assertGreater(len(path.points), 20)
        self.assertLess(max(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                            for a, b in zip(path.points, path.points[1:])), 4.0)

    def test_dlc_blkw_81_descriptor_order_and_output_lane_are_continuous(self):
        """Regression for the reported 141 degree prefab-exit jump.

        Legacy ETS2LA nodeUids must first be rotated into PPD descriptor order.
        Its outputLanes order also does not equal the raw road lane order, so
        physical continuity must select the confirmed outgoing lane after
        topology selects the road edge.
        """
        approaches = (
            (5962819256810101952, 5962819253060394194,
             5962819254075415764, 5962819266683514703),
            (5962819256810101952, 5962819253060394194,
             5962819254436125907, 5962819257229532404),
        )
        incoming = next(
            lane for lane in self.net._build_lane_segments(
                self.net._road_segment_by_uid[5962819239009456979])
            if lane.direction == 1 and lane.raw_lane_index == 0)
        truck = incoming.centerline[3]
        expected_exit_lane = {
            5962819254075415764: 0,
            5962819254436125907: 1,
        }
        for gps in approaches:
            with self.subTest(exit_uid=gps[2]):
                path, match = self.net.build_lane_path(
                    gps, (truck.x, truck.z), truck.heading, truck.y)
                self.assertIsNotNone(match)
                self.assertTrue(path.valid, path.failure_reason)
                self.assertEqual(path.segments[1].lane_id.prefab_token,
                                 "dlc_blkw_81")
                # The two confirmed prefab exits terminate on different raw
                # lanes. The dropped -4.75 m SII offsets previously collapsed
                # this physical distinction in the legacy expectations.
                self.assertEqual(path.segments[2].raw_lane_index,
                                 expected_exit_lane[gps[2]])
                boundary_gaps = [math.dist(
                    (first.centerline[-1].x, first.centerline[-1].y,
                     first.centerline[-1].z),
                    (second.centerline[0].x, second.centerline[0].y,
                     second.centerline[0].z))
                    for first, second in zip(path.segments,
                                             path.segments[1:])]
                self.assertLess(max(boundary_gaps), 0.30)
                trajectory = build_lane_trajectory(path)
                self.assertTrue(trajectory.valid,
                                trajectory.failure_reason)
                heading_jumps = [abs(math.degrees(wrap_angle(
                    second.heading-first.heading)))
                    for first, second in zip(trajectory.points,
                                             trajectory.points[1:])]
                self.assertLess(max(heading_jumps), 12.0)

    def test_blkw_prefab_to_offset_road_transition_uses_ppd_lane_centres(self):
        """Regression for the reported 6.27 m prefab-output failure.

        The blkw_2k00o arm ends near a 1.75 m lane offset while the following
        road look ends at a 7.75 m offset. SCS interpolates those offsets along
        the road; applying the road's end offset at its start creates a false
        lateral gap even though topology and headings agree exactly.
        """
        routes = (
            (5962819259746114755, 5962819261264452801,
             5962819256810101952, 5962819253060394194,
             5962819254075415764, 5962819266683514703),
            (5962819268579312546, 5962819261331562124,
             5962819256810101952, 5962819253060394194,
             5962819254436125907, 5962819257229532404),
        )
        expected_exit_lane = {
            5962819254075415764: 0,
            5962819254436125907: 1,
        }
        for gps in routes:
            with self.subTest(approach_uid=gps[1], exit_uid=gps[-2]):
                corridor = self.net.resolve_gps_corridor(gps)
                self.assertTrue(corridor.valid, corridor.failure_reason)
                first_edge = corridor.edges[0]
                incoming = next(
                    lane for lane in self.net._build_lane_segments(
                        first_edge.segment_index)
                    if lane.start_uid == first_edge.start_uid
                    and lane.raw_lane_index == 0)
                truck = incoming.centerline[len(incoming.centerline) // 2]
                path, match = self.net.build_lane_path(
                    gps, (truck.x, truck.z), truck.heading, truck.y)
                self.assertIsNotNone(match)
                self.assertTrue(path.valid, path.failure_reason)
                gaps = [math.dist(
                    (first.centerline[-1].x, first.centerline[-1].y,
                     first.centerline[-1].z),
                    (second.centerline[0].x, second.centerline[0].y,
                     second.centerline[0].z))
                    for first, second in zip(path.segments,
                                             path.segments[1:])]
                self.assertLess(max(gaps), 0.51)
                trajectory = build_lane_trajectory(path)
                self.assertTrue(trajectory.valid,
                                trajectory.failure_reason)
                self.assertEqual(path.segments[-1].raw_lane_index,
                                 expected_exit_lane[gps[-2]])

    def test_live_ioannina_uses_proven_road_lane_change_before_exit(self):
        """Regression for the reported 54.2 degree sub-kilometre failure.

        The selected ``dlc_blkw_81`` exit begins one lane beside the occupied
        raw lane 1. The captured approach contains a long common road segment
        whose adjacent raw lane leads exactly to the PPD input, so phase 3
        plans there without moving either map centreline.
        """
        gps = (
            5962819263713953727, 5962819268579312546,
            5962819261331562124, 5962819256810101952,
            5962819253060394194, 5962819251944709334,
            5962819252473191639, 5962819266733850743,
            5962819280843480628, 5962819250728386678,
        )
        truck = (42593.63119506836, 59.73221206665039,
                 62200.77322387695)
        heading = 0.03585750237107277 * math.tau
        path, match = self.net.build_lane_path(
            gps, (truck[0], truck[2]), heading, truck[1])
        self.assertIsNotNone(match)
        self.assertEqual(match.lane_id.road_uid, 5962819243967164376)
        self.assertEqual(match.lane_id.lane_index, 1)
        self.assertAlmostEqual(match.lateral_error_m, -1.0537, places=3)
        trajectory = build_lane_trajectory(path)
        validation = validate_lane_trajectory(trajectory)
        self.assertTrue(path.valid, path.failure_reason)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        self.assertTrue(validation.valid, validation.failure_reason)
        self.assertEqual(validation.lane_change_count, 1)
        transition = next(segment for segment in path.segments
                          if segment.lane_change)
        proof = transition.lane_change
        self.assertEqual(proof.prefab_token, "dlc_blkw_81")
        self.assertEqual((proof.source_raw_lane_index,
                          proof.target_raw_lane_index), (1, 0))
        self.assertEqual(proof.gps_pair_index, 3)
        self.assertGreaterEqual(proof.available_length_m,
                                proof.required_length_m)
        self.assertIs(self.net._lane_id_index[proof.source_lane_id]
                      .lane_change, None)

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

    def test_confirmed_prefab_approaches_do_not_leave_geometry_gaps(self):
        gps = (
            3808772981329690624, 3808774081340440578,
            3808588792760303618, 3808812646816481282,
            3808775487350833152, 3808777359876882432,
            3764330771381420034,
        )
        match = LaneLocator(self.net).locate(
            (-90092.1956, 22.1638, 48571.8930), 2.004394, gps)
        self.assertIsNotNone(match)
        corridor = self.net.resolve_gps_corridor(gps)
        segments, reason = self.net.select_lane_sequence(corridor, match)
        self.assertEqual(reason, "")
        path = self.net.connect_lane_sequence(segments, gps)
        self.assertTrue(path.valid, path.failure_reason)
        # The SDK buffer is a rolling local horizon; its distance-to-go field
        # can be kilometres while the currently published lane geometry is a
        # few hundred metres long.
        self.assertGreater(path.distance_m, 300.0)
        self.assertLessEqual(max(
            math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
            for a, b in zip(path.points, path.points[1:])), 3.2)

        runtime_path, _ = self.net.build_lane_path(
            gps, (-90092.1956, 48571.8930), 2.004394,
            altitude=22.1638, start_match=match)
        self.assertTrue(runtime_path.valid, runtime_path.failure_reason)
        # Runtime geometry must begin at the exact confirmed projection.  A
        # nearest centreline sample can lie behind the truck and creates a
        # large HUD/AR spike even though the remaining road is straight.
        self.assertLess(math.dist(
            (runtime_path.points[0].x, runtime_path.points[0].y,
             runtime_path.points[0].z),
            (match.point.x, match.point.y, match.point.z)), 1e-6)
        self.assertLess(math.dist(
            (runtime_path.segments[0].centerline[0].x,
             runtime_path.segments[0].centerline[0].y,
             runtime_path.segments[0].centerline[0].z),
            (match.point.x, match.point.y, match.point.z)), 1e-6)
        forward_x = -math.sin(match.point.heading)
        forward_z = -math.cos(match.point.heading)
        for point in runtime_path.segments[0].centerline[1:]:
            along = ((point.x - match.point.x) * forward_x
                     + (point.z - match.point.z) * forward_z)
            self.assertGreaterEqual(along, -1e-6)
        trajectory = build_lane_trajectory(runtime_path)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        self.assertLess(math.dist(
            (trajectory.points[0].x, trajectory.points[0].y,
             trajectory.points[0].z),
            (match.point.x, match.point.y, match.point.z)), 1e-6)

    def test_runtime_route_at_reported_ar_spike_starts_at_truck_projection(self):
        gps = (
            3808812423411073026, 3808810055118290944,
            3808827302817759232, 3808826220347588608,
            3808834757710774275, 3808834379455856640,
            3808823298989686786,
        )
        position = (-90243.50639343262, 22.167076110839844,
                    48817.4098815918)
        heading = -1.131815292032769
        match = LaneLocator(self.net).locate(position, heading, gps)
        self.assertIsNotNone(match)
        path, _ = self.net.build_lane_path(
            gps, (position[0], position[2]), heading,
            altitude=position[1], start_match=match)
        self.assertTrue(path.valid, path.failure_reason)
        trajectory = build_lane_trajectory(path)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        display_points = derive_display_points(trajectory)
        self.assertTrue(display_points)

        for points in (path.segments[0].centerline, path.points,
                       trajectory.points, display_points):
            self.assertLess(math.dist(
                (points[0].x, points[0].y, points[0].z),
                (match.point.x, match.point.y, match.point.z)), 1e-6)
            # The old nearest-sample trim started 1.37 m behind the confirmed
            # projection. This forward projection catches that visual chord
            # independently of resampling density.
            forward_x = -math.sin(match.point.heading)
            forward_z = -math.cos(match.point.heading)
            along = ((points[1].x - points[0].x) * forward_x
                     + (points[1].z - points[0].z) * forward_z)
            self.assertGreater(along, 0.0)

    def test_runtime_route_advances_past_retained_gps_prefix(self):
        """A truck on edge two must not be prepended before GPS edge one."""
        gps = (
            3808812423411073026, 3808810055118290944,
            3808827302817759232, 3808826220347588608,
            3808834757710774275, 3808834379455856640,
            3808823298989686786,
        )
        active_id = 3808826221916258305
        lanes = []
        for segment_index, road_uid in enumerate(self.net._seg_road_uids):
            if road_uid == active_id:
                lanes.extend(self.net._build_lane_segments(segment_index))
        active = next(lane for lane in lanes
                      if lane.direction == -1 and lane.lane_index == 0)
        truck = active.centerline[10]
        match = LaneLocator(self.net).locate(
            (truck.x, truck.y, truck.z), truck.heading, gps)
        self.assertIsNotNone(match)
        self.assertEqual(match.lane_id, active.lane_id)

        path, returned = self.net.build_lane_path(
            gps, (truck.x, truck.z), truck.heading,
            altitude=truck.y, start_match=match)
        self.assertTrue(path.valid, path.failure_reason)
        self.assertEqual(returned.lane_id, active.lane_id)
        self.assertEqual(path.segments[0].lane_id, active.lane_id)
        self.assertNotIn("does not connect to the first GPS lane",
                         path.failure_reason)
        self.assertLess(math.dist(
            (path.points[0].x, path.points[0].y, path.points[0].z),
            (match.point.x, match.point.y, match.point.z)), 1e-6)

        trajectory = build_lane_trajectory(path)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        validation = validate_lane_trajectory(trajectory)
        self.assertTrue(validation.valid, validation.failure_reason)
        self.assertEqual(path.expected_first_gps_pair_index, 2)
        self.assertEqual(validation.first_gps_pair_index, 2)
        self.assertEqual(validation.last_gps_pair_index, 5)
        self.assertLess(math.dist(
            (trajectory.points[0].x, trajectory.points[0].y,
             trajectory.points[0].z),
            (match.point.x, match.point.y, match.point.z)), 1e-6)
        # No backwards prefix means there is no artificial U-turn/spike at
        # the live truck position. The real ibe91 bend remains below the
        # trajectory validator's strict heading limit.
        jumps = [abs(math.degrees(wrap_angle(b.heading - a.heading)))
                 for a, b in zip(trajectory.points, trajectory.points[1:])]
        self.assertLess(max(jumps, default=0.0), 35.0)

    def test_roundabout_same_exit_rejects_unrequested_extra_lap(self):
        gps = (3764330771381420034, 3808790278165430272)
        corridor = self.net.resolve_gps_corridor(gps)
        self.assertTrue(corridor.valid, corridor.failure_reason)
        segment, reason = self.net._prefab_lane_segment(corridor.edges[0], 0)
        self.assertEqual(reason, "")
        self.assertIsNotNone(segment)
        self.assertEqual(segment.connector_curve_indices, (1, 3, 21, 17, 22))
        self.assertLess(sum(math.dist(
            (a.x, a.y, a.z), (b.x, b.y, b.z))
            for a, b in zip(segment.centerline, segment.centerline[1:])), 70.0)


if __name__ == "__main__":
    unittest.main()
