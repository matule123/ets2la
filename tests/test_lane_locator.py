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


def path_lane(road_uid, start_uid, end_uid, coordinates, *, lane_type="road",
              lane_index=0, prefab_token=None, connector_index=None,
              elevation_layer=0):
    lane_id = LaneId(road_uid, 1, lane_index, prefab_token, connector_index,
                     (() if connector_index is None else (connector_index,)))
    travelled = 0.0
    points = []
    for index, (x, y, z) in enumerate(coordinates):
        if index:
            travelled += math.dist(coordinates[index - 1], coordinates[index])
        points.append(LanePoint(x, y, z, travelled, lane_id=lane_id))
    return LaneSegment(
        lane_id, start_uid, end_uid, 1, lane_index, 2, 4.5,
        "dataset" if prefab_token else "derived", elevation_layer,
        None if prefab_token else "look", lane_type, tuple(points),
        gps_uids=frozenset((start_uid, end_uid)))


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


class TransitionNetwork(FakeNetwork):
    def __init__(self, roads, prefabs=(), connected=()):
        super().__init__(list(roads))
        self.prefabs = list(prefabs)
        self.connected = set(connected)

    def route_prefix_lane_segments_near(
            self, _position, _gps_uids, _radius, register=True):
        return ()

    def gps_prefab_lane_segments_near(
            self, _position, gps_uids, _radius, register=True):
        edges = set(zip(gps_uids, gps_uids[1:]))
        return [lane for lane in self.prefabs
                if (lane.start_uid, lane.end_uid) in edges]


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

    def test_same_deck_telemetry_height_bias_does_not_drop_confidence(self):
        locator = LaneLocator(FakeNetwork([lane(1, 0, height=0.0)]))
        match = locator.locate((0.0, 1.5, 15.0), math.pi, (10, 11))
        self.assertIsNotNone(match)
        self.assertEqual(match.vertical_error_m, 1.5)
        self.assertGreaterEqual(match.confidence, 0.72)
        self.assertEqual(dict(match.score_components)["vertical"], 0.0)

    def test_reported_0893m_bias_stays_above_threshold_near_route_start(self):
        # Runtime revision 3398 scored 0.707883 because the 0.893 m vehicle-vs-
        # road reference-height bias alone contributed 2.68 points.  The lane
        # shared the first GPS UID but was not the first directed edge, so its
        # intentional near-route penalty must remain; only the same-deck height
        # bias is removed from the confidence score.
        locator = LaneLocator(FakeNetwork([lane(1, 0, height=0.0)]))
        match = locator.locate((0.0, 0.893, 15.0), math.pi, (9, 10))
        self.assertIsNotNone(match)
        components = dict(match.score_components)
        self.assertEqual(components["off_route"], 2.0)
        self.assertEqual(components["vertical"], 0.0)
        self.assertAlmostEqual(match.vertical_error_m, 0.893)
        self.assertGreaterEqual(match.confidence, 0.72)

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
        self.assertEqual(second.switch_reason, "lane_change_pending")
        third = locator.locate((0.95, 0, 15), math.pi)
        self.assertEqual(third.lane_id, right.lane_id)
        self.assertEqual(third.switch_reason, "lane_change_confirmed")

    def test_progressive_left_and_right_lane_changes_are_confirmed(self):
        left = lane(70, -2.25, lane_index=0)
        right = lane(70, 2.25, lane_index=1)
        locator = LaneLocator(FakeNetwork([left, right]))

        first = locator.locate((-2.1, 0.0, 2.0), math.pi, (10, 11, 12))
        pending = locator.locate((0.6, 0.0, 10.0), math.pi, (10, 11, 12))
        changed = locator.locate((1.8, 0.0, 25.0), math.pi, (11, 12))
        self.assertEqual(first.lane_id, left.lane_id)
        self.assertEqual(pending.lane_id, left.lane_id)
        self.assertEqual(pending.switch_reason, "lane_change_pending")
        self.assertEqual(changed.lane_id, right.lane_id)
        self.assertEqual(changed.switch_reason, "lane_change_confirmed")

        pending_back = locator.locate(
            (-0.6, 0.0, 32.0), math.pi, (11, 12))
        changed_back = locator.locate(
            (-1.8, 0.0, 39.0), math.pi, (11, 12))
        self.assertEqual(pending_back.lane_id, right.lane_id)
        self.assertEqual(pending_back.switch_reason, "lane_change_pending")
        self.assertEqual(changed_back.lane_id, left.lane_id)
        self.assertEqual(changed_back.switch_reason, "lane_change_confirmed")

    def test_interrupted_or_too_fast_lane_change_never_switches(self):
        left = lane(71, -2.25, lane_index=0)
        right = lane(71, 2.25, lane_index=1)
        locator = LaneLocator(FakeNetwork([left, right]))
        self.assertEqual(locator.locate(
            (-2.1, 0.0, 1.0), math.pi).lane_id, left.lane_id)
        self.assertEqual(locator.locate(
            (0.6, 0.0, 6.0), math.pi).lane_id, left.lane_id)
        interrupted = locator.locate((-0.4, 0.0, 12.0), math.pi)
        self.assertEqual(interrupted.lane_id, left.lane_id)
        # A later one-frame jump is only a new unconfirmed observation.
        self.assertIsNone(locator.locate((2.2, 0.0, 18.0), math.pi))
        self.assertEqual(locator.previous.lane_id, left.lane_id)

        locator = LaneLocator(FakeNetwork([left, right]))
        locator.locate((-2.1, 0.0, 0.0), math.pi)
        locator.locate((0.6, 0.0, 1.0), math.pi)
        self.assertIsNone(locator.locate((1.8, 0.0, 39.0), math.pi))
        self.assertEqual(locator.previous.lane_id, left.lane_id)

    def test_opposite_and_non_adjacent_lanes_cannot_be_lane_changes(self):
        source = lane(72, -2.25, lane_index=0)
        opposite = lane(72, 2.25, direction=-1, lane_index=1)
        locator = LaneLocator(FakeNetwork([source, opposite]))
        first = locator.locate((-2.1, 0.0, 5.0), math.pi)
        self.assertEqual(first.lane_id, source.lane_id)
        self.assertIsNone(locator.locate((2.1, 0.0, 15.0), 0.0))

        distant_lane = lane(73, 2.25, lane_index=2)
        source = lane(73, -2.25, lane_index=0)
        locator = LaneLocator(FakeNetwork([source, distant_lane]))
        self.assertEqual(locator.locate(
            (-2.1, 0.0, 5.0), math.pi).lane_id, source.lane_id)
        self.assertIsNone(locator.locate((2.1, 0.0, 15.0), math.pi))

        upper = lane(74, 2.25, height=6.0, lane_index=1)
        lower = lane(74, -2.25, height=0.0, lane_index=0)
        locator = LaneLocator(FakeNetwork([lower, upper]))
        self.assertEqual(locator.locate(
            (-2.1, 0.0, 5.0), math.pi).lane_id, lower.lane_id)
        self.assertIsNone(locator.locate((2.1, 6.0, 15.0), math.pi))

    def test_diagnostic_lookup_does_not_consume_lane_change_evidence(self):
        left = lane(75, -2.25, lane_index=0)
        right = lane(75, 2.25, lane_index=1)
        locator = LaneLocator(FakeNetwork([left, right]))
        locator.locate((-2.1, 0.0, 2.0), math.pi)
        locator.locate((0.6, 0.0, 10.0), math.pi)
        evidence = locator._lane_change_evidence
        self.assertIsNotNone(evidence)
        samples = evidence.samples
        previous = locator.previous
        diagnostic = locator.locate(
            (1.8, 0.0, 25.0), math.pi, diagnostics={},
            diagnostic_mode=True)
        self.assertEqual(diagnostic.lane_id, right.lane_id)
        self.assertIs(locator.previous, previous)
        self.assertIs(locator._lane_change_evidence, evidence)
        self.assertEqual(evidence.samples, samples)
        confirmed = locator.locate((1.8, 0.0, 25.0), math.pi)
        self.assertEqual(confirmed.lane_id, right.lane_id)
        self.assertEqual(confirmed.switch_reason, "lane_change_confirmed")

    def test_confirmed_lane_change_immediately_before_prefab(self):
        left = path_lane(80, 1, 2, (
            (-2.25, 0.0, 0.0), (-2.25, 0.0, 40.0)), lane_index=0)
        right = path_lane(80, 1, 2, (
            (2.25, 0.0, 0.0), (2.25, 0.0, 40.0)), lane_index=1)
        prefab = path_lane(81, 2, 3, (
            (2.25, 0.0, 40.0), (3.0, 0.0, 50.0),
            (4.0, 0.0, 60.0)), lane_type="prefab", lane_index=1,
            prefab_token="gps-junction", connector_index=4)
        network = TransitionNetwork(
            (left, right), (prefab,), ((right.lane_id, prefab.lane_id),))
        locator = LaneLocator(network)
        locator.locate((-2.1, 0.0, 5.0), math.pi, (1, 2, 3))
        locator.locate((0.6, 0.0, 18.0), math.pi, (1, 2, 3))
        changed = locator.locate((1.8, 0.0, 32.0), math.pi, (1, 2, 3))
        self.assertEqual(changed.lane_id, right.lane_id)
        on_prefab = locator.locate(
            (2.6, 0.0, 45.0), math.atan2(-0.75, -10.0), (2, 3))
        self.assertIsNotNone(on_prefab)
        self.assertEqual(on_prefab.lane_id, prefab.lane_id)

    def test_same_lane_retention_runs_before_strict_acquisition_gate(self):
        """Reproduce the ProMods rev45 -> rev46 runtime LaneMatch loss."""
        route_lane = lane(5962819247825894584, 0.0, gps=(10, 11))
        locator = LaneLocator(FakeNetwork([route_lane]))
        first = locator.locate((1.497, 0.0, 5.0), math.pi, (10, 11))
        self.assertIsNotNone(first)

        # The real diagnostic changed from 1.497 m to 2.915 m on this exact
        # LaneId after a 551 ms route build, while heading/elevation stayed
        # valid. A fresh acquisition remains rejected, but the confirmed lane
        # must survive the bounded same-lane motion.
        self.assertIsNone(LaneLocator(FakeNetwork([route_lane])).locate(
            (2.915, 0.0, 20.0), math.pi, (10, 11)))
        diagnostics = {}
        retained = locator.locate(
            (2.915, 0.0, 20.0), math.pi, (10, 11),
            diagnostics=diagnostics)
        self.assertIsNotNone(retained)
        self.assertEqual(retained.lane_id, first.lane_id)
        self.assertEqual(retained.switch_reason, "hysteresis_hold")
        candidate = diagnostics["candidate_lanes"][0]
        self.assertTrue(candidate["same_lane_retention"])
        self.assertAlmostEqual(candidate["distance_limit_m"], 3.375)
        self.assertIsNone(locator.locate(
            (3.5, 0.0, 21.0), math.pi, (10, 11)))

    def test_same_lane_retention_rejects_teleport_and_wrong_direction(self):
        route_lane = lane(1, 0.0, gps=(10, 11))
        locator = LaneLocator(FakeNetwork([route_lane]))
        self.assertIsNotNone(locator.locate(
            (1.0, 0.0, 0.0), math.pi, (10, 11)))
        self.assertIsNone(locator.locate(
            (2.9, 0.0, 40.0), math.pi, (10, 11)))

        locator = LaneLocator(FakeNetwork([route_lane]))
        self.assertIsNotNone(locator.locate(
            (1.0, 0.0, 10.0), math.pi, (10, 11)))
        self.assertIsNone(locator.locate(
            (2.9, 0.0, 20.0), 0.0, (10, 11)))

    def test_same_lane_retention_never_transfers_to_neighbor(self):
        left = lane(7, 0.0, lane_index=0)
        right = lane(7, 0.0, lane_index=1)
        network = FakeNetwork([left])
        locator = LaneLocator(network)
        previous = locator.locate((1.0, 0.0, 10.0), math.pi, (10, 11))
        self.assertEqual(previous.lane_id, left.lane_id)
        network.lanes = [right]
        self.assertIsNone(locator.locate(
            (2.9, 0.0, 20.0), math.pi, (10, 11)))

    def test_retention_does_not_expand_fresh_endpoint_acquisition(self):
        route_lane = lane(1, 0.0, gps=(10, 11))
        diagnostics = {}
        self.assertIsNone(LaneLocator(FakeNetwork([route_lane])).locate(
            (2.0, 0.0, -2.0), math.pi, (10, 11),
            diagnostics=diagnostics))
        candidate = diagnostics["candidate_lanes"][0]
        self.assertAlmostEqual(abs(candidate["signed_lateral_m"]), 2.0)
        self.assertAlmostEqual(candidate["longitudinal_overrun_m"], 2.0)
        self.assertEqual(candidate["rejection"], "lateral")

    def test_continuous_road_prefab_road_localization_and_rolling_prefix(self):
        road_in = path_lane(101, 1, 2, (
            (0.0, 0.0, 0.0), (0.0, 0.0, 20.0)))
        prefab = path_lane(202, 2, 3, (
            (0.0, 0.0, 20.0), (2.0, 0.0, 30.0),
            (4.0, 0.0, 40.0)), lane_type="prefab",
            prefab_token="gps-junction", connector_index=4)
        road_out = path_lane(303, 3, 4, (
            (4.0, 0.0, 40.0), (4.0, 0.0, 60.0)))
        network = TransitionNetwork(
            (road_in, road_out), (prefab,), (
                (road_in.lane_id, prefab.lane_id),
                (prefab.lane_id, road_out.lane_id),
            ))
        locator = LaneLocator(network)
        samples = (
            ((0.0, 0.0, 10.0), math.pi, (1, 2, 3, 4)),
            # The rolling buffer drops UID 1 while the truck is still on the
            # proven incoming road. Its exact previous LaneId remains valid.
            ((2.9, 0.0, 18.0), math.pi, (2, 3, 4)),
            ((0.4, 0.0, 22.0), math.atan2(-0.2, -1.0), (2, 3, 4)),
            ((2.0, 0.0, 30.0), math.atan2(-0.2, -1.0), (2, 3, 4)),
            ((3.8, 0.0, 39.0), math.atan2(-0.2, -1.0), (2, 3, 4)),
            ((4.0, 0.0, 43.0), math.pi, (2, 3, 4)),
            ((4.0, 0.0, 52.0), math.pi, (2, 3, 4)),
        )
        matches = [locator.locate(position, heading, gps)
                   for position, heading, gps in samples]
        self.assertTrue(all(match is not None for match in matches), matches)
        self.assertEqual(matches[0].lane_id, road_in.lane_id)
        self.assertEqual(matches[1].lane_id, road_in.lane_id)
        self.assertIn(prefab.lane_id, [match.lane_id for match in matches])
        self.assertEqual(matches[-1].lane_id, road_out.lane_id)

    def test_validated_prefab_remains_localisable_after_rolling_prefix_trim(self):
        """Reproduce the 14:15:51 live-match loss from the game log.

        The SDK has already dropped the prefab's entry UID, so current GPS
        queries expose only the outgoing road. The truck is still on the
        validated navCurve and must use that immutable segment until the
        proven prefab -> road transition completes.
        """
        prefab = path_lane(501, 1, 2, (
            (0.0, 0.0, 20.0), (2.0, 0.0, 30.0),
            (4.0, 0.0, 40.0)), lane_type="prefab",
            prefab_token="rolling-junction", connector_index=7)
        road_out = path_lane(502, 2, 3, (
            (4.0, 0.0, 40.0), (4.0, 0.0, 70.0)))
        network = TransitionNetwork(
            (road_out,), (prefab,), ((prefab.lane_id, road_out.lane_id),))

        without_snapshot = LaneLocator(network)
        first = without_snapshot.locate(
            (2.0, 0.0, 30.0), math.atan2(-2.0, -10.0), (1, 2, 3))
        self.assertEqual(first.lane_id, prefab.lane_id)
        self.assertIsNone(without_snapshot.locate(
            (3.3, 0.0, 36.5), math.atan2(-2.0, -10.0), (2, 3)))

        locator = LaneLocator(network)
        first = locator.locate(
            (2.0, 0.0, 30.0), math.atan2(-2.0, -10.0), (1, 2, 3))
        still_prefab = locator.locate(
            (3.3, 0.0, 36.5), math.atan2(-2.0, -10.0), (2, 3),
            authoritative_segments=(prefab, road_out))
        on_road = locator.locate(
            (4.0, 0.0, 45.0), math.pi, (2, 3),
            authoritative_segments=(prefab, road_out))
        self.assertEqual(first.lane_id, prefab.lane_id)
        self.assertEqual(still_prefab.lane_id, prefab.lane_id)
        self.assertEqual(on_road.lane_id, road_out.lane_id)

    def test_validated_path_transition_survives_trimmed_runtime_graph_edge(self):
        """A rolling prefix must not erase an already validated transition.

        This reproduces the August 1 runtime failure: the match was centred
        (CTE -0.009 m, heading error 0.0 deg), then the generic runtime graph
        no longer contained the next edge and localisation became unavailable.
        The immediate ordered LanePath neighbour is sufficient proof, without
        exposing any later segment or relaxing distance/heading/height gates.
        """
        road_in = path_lane(701, 1, 2, (
            (0.0, 0.0, 0.0), (0.0, 0.0, 20.0)))
        road_out = path_lane(702, 2, 3, (
            (0.0, 0.0, 20.0), (0.0, 0.0, 55.0)))
        network = TransitionNetwork((road_in, road_out), connected=())
        locator = LaneLocator(network)
        first = locator.locate(
            (0.0, 0.0, 18.0), math.pi, (1, 2, 3),
            authoritative_segments=(road_in, road_out))
        self.assertEqual(first.lane_id, road_in.lane_id)

        # The SDK has trimmed UID 1 and the runtime graph has no registered
        # edge, but the truck advances continuously onto the next validated
        # segment.
        transitioned = locator.locate(
            (0.0, 0.0, 24.0), math.pi, (2, 3),
            authoritative_segments=(road_in, road_out))
        self.assertIsNotNone(transitioned)
        self.assertEqual(transitioned.lane_id, road_out.lane_id)
        self.assertEqual(transitioned.switch_reason, "topology_transition")

        # The proof remains local: a non-neighbouring later segment cannot be
        # selected merely because it is geometrically close.
        remote = path_lane(703, 3, 4, (
            (0.1, 0.0, 24.0), (0.1, 0.0, 45.0)))
        locator = LaneLocator(TransitionNetwork((road_in, remote), connected=()))
        locator.locate((0.0, 0.0, 18.0), math.pi, (1, 2, 3, 4),
                       authoritative_segments=(road_in, road_out, remote))
        local = locator.locate(
            (0.1, 0.0, 25.0), math.pi, (2, 3, 4),
            authoritative_segments=(road_in, road_out, remote))
        self.assertIsNotNone(local)
        self.assertEqual(local.lane_id, road_out.lane_id)
        self.assertNotEqual(local.lane_id, remote.lane_id)

    def test_authoritative_path_candidates_are_progress_bounded_and_read_only(self):
        current = path_lane(601, 1, 2, (
            (1.0, 0.0, 0.0), (1.0, 0.0, 20.0)))
        next_one = path_lane(602, 2, 3, (
            (1.0, 0.0, 20.0), (1.0, 0.0, 40.0)))
        next_two = path_lane(603, 3, 4, (
            (1.0, 0.0, 40.0), (1.0, 0.0, 60.0)))
        next_three = path_lane(604, 4, 5, (
            (1.0, 0.0, 60.0), (1.0, 0.0, 80.0)))
        # Same UID edge on a later lap, geometrically slightly closer at the
        # crossing. Global proximity would select it, ordered path progress
        # must not expose it while the locator is still at occurrence zero.
        later_lap = path_lane(605, 1, 2, (
            (0.0, 0.0, 0.0), (0.0, 0.0, 20.0)))
        path = (current, next_one, next_two, next_three, later_lap)
        network = TransitionNetwork((current,), connected=(
            (current.lane_id, later_lap.lane_id),
            (current.lane_id, next_one.lane_id),
        ))
        locator = LaneLocator(network)
        acquired = locator.locate((1.0, 0.0, 10.0), math.pi, (1, 2))
        self.assertEqual(acquired.lane_id, current.lane_id)
        network.lanes = []

        diagnostics = {}
        held = locator.locate(
            (0.0, 0.0, 10.0), math.pi, (9, 10),
            authoritative_segments=path, diagnostics=diagnostics)
        self.assertEqual(held.lane_id, current.lane_id)
        candidate_ids = {
            item["lane_id"]["road_uid"]
            for item in diagnostics["candidate_lanes"]
        }
        self.assertNotIn(later_lap.lane_id.road_uid, candidate_ids)
        self.assertEqual(locator._authoritative_segment_index, 0)

        signature = locator._authoritative_path_signature
        index = locator._authoritative_segment_index
        observed = locator.locate(
            (1.0, 0.0, 25.0), math.pi, (9, 10),
            authoritative_segments=path, diagnostic_mode=True,
            diagnostics={})
        self.assertEqual(observed.lane_id, next_one.lane_id)
        self.assertEqual(locator._authoritative_path_signature, signature)
        self.assertEqual(locator._authoritative_segment_index, index)
        self.assertEqual(locator.previous.lane_id, current.lane_id)

        # A replacement horizon containing two coincident occurrences of the
        # same LaneId cannot prove which roundabout lap is current. The old
        # occurrence must not be selected merely because it appears first.
        locator._authoritative_path_signature = locator._path_signature(path)
        locator._authoritative_segment_index = 0
        repeated = (current, next_one, current)
        self.assertIsNone(locator.locate(
            (1.0, 0.0, 10.0), math.pi, (9, 10),
            authoritative_segments=repeated))
        self.assertEqual(locator._authoritative_segment_index, 0)

    def test_authoritative_segments_still_require_motion_heading_and_elevation(self):
        current = path_lane(611, 1, 2, (
            (0.0, 0.0, 0.0), (0.0, 0.0, 20.0)))
        forward = path_lane(612, 2, 3, (
            (0.0, 0.0, 20.0), (0.0, 0.0, 60.0)))
        upper = path_lane(613, 2, 3, (
            (0.0, 8.0, 20.0), (0.0, 8.0, 60.0)),
            elevation_layer=3)
        network = TransitionNetwork((current,), connected=(
            (current.lane_id, forward.lane_id),
            (current.lane_id, upper.lane_id),
        ))
        locator = LaneLocator(network)
        locator.locate((0.0, 0.0, 10.0), math.pi, (1, 2, 3))
        network.lanes = []
        self.assertIsNone(locator.locate(
            (0.0, 0.0, 50.0), math.pi, (2, 3),
            authoritative_segments=(current, forward)))

        locator = LaneLocator(TransitionNetwork(
            (current,), connected=((current.lane_id, upper.lane_id),)))
        locator.locate((0.0, 0.0, 10.0), math.pi, (1, 2, 3))
        locator.network.lanes = []
        self.assertIsNone(locator.locate(
            (0.0, 0.0, 24.0), math.pi, (2, 3),
            authoritative_segments=(current, upper)))

        locator = LaneLocator(TransitionNetwork(
            (current,), connected=((current.lane_id, forward.lane_id),)))
        locator.locate((0.0, 0.0, 10.0), math.pi, (1, 2, 3))
        locator.network.lanes = []
        self.assertIsNone(locator.locate(
            (0.0, 0.0, 24.0), 0.0, (2, 3),
            authoritative_segments=(current, forward)))

    def test_merge_split_and_intersection_change_active_segment_without_loss(self):
        incoming = path_lane(10, 1, 2, (
            (0.0, 0.0, 0.0), (0.0, 0.0, 20.0)), lane_index=1)
        merged = path_lane(20, 2, 3, (
            (0.0, 0.0, 20.0), (1.0, 0.0, 40.0)), lane_index=0)
        selected = path_lane(30, 3, 4, (
            (1.0, 0.0, 40.0), (16.0, 0.0, 55.0)), lane_index=0)
        wrong_arm = path_lane(40, 3, 5, (
            (1.0, 0.0, 40.0), (-14.0, 0.0, 55.0)), lane_index=0)
        network = TransitionNetwork(
            (incoming, merged, selected, wrong_arm), connected=(
                (incoming.lane_id, merged.lane_id),
                (merged.lane_id, selected.lane_id),
                (merged.lane_id, wrong_arm.lane_id),
            ))
        locator = LaneLocator(network)
        samples = (
            ((0.0, 0.0, 10.0), math.pi),
            ((0.2, 0.0, 24.0), math.atan2(-1.0, -20.0)),
            ((0.8, 0.0, 38.0), math.atan2(-1.0, -20.0)),
            ((6.0, 0.0, 45.0), math.radians(-135.0)),
            ((12.0, 0.0, 51.0), math.radians(-135.0)),
        )
        matches = [locator.locate(position, heading, (1, 2, 3, 4))
                   for position, heading in samples]
        self.assertTrue(all(match is not None for match in matches), matches)
        self.assertEqual(matches[0].lane_id, incoming.lane_id)
        self.assertIn(merged.lane_id, [match.lane_id for match in matches])
        self.assertEqual(matches[-1].lane_id, selected.lane_id)
        self.assertNotIn(wrong_arm.lane_id,
                         [match.lane_id for match in matches])

    def test_no_match_when_height_is_ambiguous_and_too_far(self):
        match = LaneLocator(FakeNetwork([lane(1, 0, height=15)])).locate(
            (0, 0, 15), math.pi)
        self.assertIsNone(match)


if __name__ == "__main__":
    unittest.main()
