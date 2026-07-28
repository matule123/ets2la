import json
import math
import pickle
import unittest
from dataclasses import replace
from unittest import mock

import core.navigation.road_network as road_network_module
from core.navigation.lane_model import (
    GpsCorridor, GpsCorridorEdge, LaneConnection, LaneId, LaneLocator,
    LaneMatch, LanePoint, LaneSegment,
)
from core.navigation.road_network import CACHE_VERSION, RoadNetwork
from core.navigation.route_diagnostics import anonymize_failure_record


def _lane(lane_id, start_uid, end_uid, coordinates, *, lane_index=0,
          lane_count=1, elevation_layer=0, direction=1, gps_uids=()):
    points = []
    travelled = 0.0
    for index, (x, y, z) in enumerate(coordinates):
        if index:
            travelled += math.dist(coordinates[index - 1], coordinates[index])
        before = coordinates[max(0, index - 1)]
        after = coordinates[min(len(coordinates) - 1, index + 1)]
        heading = math.atan2(-(after[0] - before[0]),
                             -(after[2] - before[2]))
        points.append(LanePoint(x, y, z, travelled, heading,
                                lane_id=lane_id))
    return LaneSegment(
        lane_id, start_uid, end_uid, direction, lane_index, lane_count,
        4.5, "derived", elevation_layer, None, "prefab", tuple(points),
        gps_uids=frozenset(gps_uids),
    )


class Phase1SafetyAuditTests(unittest.TestCase):
    @staticmethod
    def _load_prefab_instance(instance):
        description = {
            "token": "audit-prefab", "path": "audit",
            "nodes": [{}, {}, {}, {}], "navCurves": [], "navNodes": [],
        }
        net = RoadNetwork()
        with (mock.patch.object(
                road_network_module, "_find_json",
                side_effect=lambda _directory, name: name),
              mock.patch.object(
                road_network_module, "_loadf",
                side_effect=lambda path: ([description] if
                                          path == "prefabDescriptions" else
                                          [instance]))):
            net._load_prefabs("unused")
        return net

    def test_legacy_node_uids_rotate_exactly_into_descriptor_order(self):
        net = self._load_prefab_instance({
            "token": "audit-prefab", "nodeUids": [11, 22, 33, 44],
            "originNodeIndex": 2, "x": 0.0, "y": 0.0,
        })
        loaded = next(iter(net._prefab_grid.values()))[0]
        self.assertEqual(loaded[1], (33, 44, 11, 22))

    def test_correct_descriptor_node_uids_are_preserved_and_mismatch_fails_closed(self):
        correct = {
            "token": "audit-prefab", "nodeUids": [11, 22, 33, 44],
            "descriptorNodeUids": [33, 44, 11, 22],
            "originNodeIndex": 2, "x": 0.0, "y": 0.0,
        }
        net = self._load_prefab_instance(correct)
        loaded = next(iter(net._prefab_grid.values()))[0]
        self.assertEqual(loaded[1], tuple(correct["descriptorNodeUids"]))

        damaged = dict(correct, descriptorNodeUids=[44, 33, 11, 22])
        rejected = self._load_prefab_instance(damaged)
        self.assertFalse(rejected._prefab_grid)
        self.assertFalse(rejected._prefab_lane_data)

    def test_old_cache_version_is_rejected_before_state_is_loaded(self):
        net = RoadNetwork()
        original_nodes = dict(net.nodes)
        payload = {
            "version": CACHE_VERSION - 1,
            "sig": [],
            "data": {"nodes": {999: (1.0, 2.0)}, "loaded": True},
        }
        with (mock.patch("os.path.exists", return_value=True),
              mock.patch("builtins.open", mock.mock_open()),
              mock.patch.object(pickle, "load", return_value=payload)):
            self.assertFalse(net._try_load_cache("unused"))
        self.assertEqual(net.nodes, original_nodes)
        self.assertFalse(net.loaded)

    def test_prefab_candidates_stop_at_first_unproven_gps_pair(self):
        net = RoadNetwork()
        net.loaded = True
        net.nodes.update({1: (0.0, 0.0), 2: (0.0, 10.0),
                          3: (0.0, 20.0)})
        lane_id = LaneId(20, 1, 0, "gps-prefab", 0, (0,))
        candidate = _lane(
            lane_id, 2, 3, ((0.0, 0.0, 0.0), (0.0, 0.0, 20.0)),
            gps_uids=(2, 3),
        )
        later_edge = GpsCorridorEdge(
            2, 3, "prefab", 1, prefab_instance=(),
        )
        with (mock.patch.object(
                net, "_classify_corridor_edge",
                side_effect=(None, later_edge)),
              mock.patch.object(
                net, "_prefab_lane_segment",
                return_value=(candidate, ""))):
            candidates = net.gps_prefab_lane_segments_near(
                (0.0, 0.0, 10.0), (1, 2, 3), 28.0,
            )
        self.assertEqual(candidates, ())

    def test_route_prefix_query_receives_the_complete_current_gps_order(self):
        class Network:
            def __init__(self):
                self.received = None

            def lane_segments_near(self, _position, _radius):
                return ()

            def route_prefix_lane_segments_near(
                    self, _position, gps_uids, _radius, register=True):
                self.received = (tuple(gps_uids), register)
                return ()

            def gps_prefab_lane_segments_near(
                    self, _position, _gps_uids, _radius, register=True):
                return ()

            def lanes_connected(self, first, second):
                return first == second

        network = Network()
        LaneLocator(network).locate(
            (0.0, 0.0, 0.0), 0.0, (101, 102, 103),
            diagnostic_mode=True,
        )
        self.assertEqual(network.received, ((101, 102, 103), False))

    def test_colliding_lane_identity_across_decks_or_directions_fails_closed(self):
        lane_id = LaneId(50, 1, 0, "converged", 4, (4,))
        correct = _lane(
            lane_id, 1, 2,
            ((0.0, 0.0, 0.0), (0.0, 0.0, 20.0)),
            elevation_layer=0,
        )
        wrong_deck = replace(
            correct, elevation_layer=4,
            centerline=tuple(replace(point, y=12.0)
                             for point in correct.centerline),
        )
        wrong_direction = replace(
            correct, direction=-1,
            centerline=tuple(reversed(tuple(
                replace(point, heading=0.0) for point in correct.centerline))),
        )

        class Network:
            def __init__(self, lanes):
                self.lanes = lanes

            def lane_segments_near(self, _position, _radius):
                return self.lanes

            def lanes_connected(self, first, second):
                return first == second

        for collision in (wrong_deck, wrong_direction):
            with self.subTest(collision=collision.direction,
                              layer=collision.elevation_layer):
                match = LaneLocator(Network((correct, collision))).locate(
                    (0.0, 0.0, 10.0), math.pi,
                )
                self.assertIsNone(match)

    def test_identical_converged_prefab_geometry_is_deduplicated(self):
        lane_id = LaneId(60, 1, 0, "converged", 5, (5,))
        first = _lane(
            lane_id, 1, 2,
            ((0.0, 0.0, 0.0), (0.0, 0.0, 20.0)), gps_uids=(1,),
        )
        second = replace(first, gps_uids=frozenset((1, 2)))

        class Network:
            def lane_segments_near(self, _position, _radius):
                return (first, second)

            def lanes_connected(self, first_id, second_id):
                return first_id == second_id

        match = LaneLocator(Network()).locate(
            (0.0, 0.0, 10.0), math.pi,
        )
        self.assertIsNotNone(match)
        self.assertEqual(match.lane_id, lane_id)

    @staticmethod
    def _select_prefab_pair(second_x=0.0, second_y=0.0,
                            first_lane_count=2, second_lane_count=1,
                            second_lane_index=0):
        net = RoadNetwork()
        first_id = LaneId(70, 1, 0, "merge", 0, (0,))
        second_id = LaneId(71, 1, second_lane_index, "merge", 1, (1,))
        first = _lane(
            first_id, 1, 2,
            ((0.0, 0.0, 0.0), (0.0, 0.0, 20.0)),
            lane_count=first_lane_count,
        )
        second = _lane(
            second_id, 2, 3,
            ((second_x, second_y, 20.0),
             (second_x, second_y, 40.0)),
            lane_index=second_lane_index, lane_count=second_lane_count,
            elevation_layer=int(round(second_y / 3.0)),
        )
        corridor = GpsCorridor(
            (1, 2, 3),
            (GpsCorridorEdge(1, 2, "prefab", 0),
             GpsCorridorEdge(2, 3, "prefab", 1)),
            True,
        )
        match = LaneMatch(
            first_id, first.centerline[0], 0, 0, 0.0, 0.0, 0.0,
            0.0, 1.0, "test",
        )
        with mock.patch.object(
                net, "_prefab_lane_segment",
                side_effect=((first, ""), (second, ""))):
            selected, reason = net.select_lane_sequence(corridor, match)
        return net, first, second, selected, reason

    def test_proven_merge_split_and_lane_change_boundaries_remain_valid(self):
        cases = ((2, 1, 0), (1, 2, 0), (2, 2, 1))
        for first_count, second_count, second_index in cases:
            with self.subTest(first=first_count, second=second_count,
                              lane=second_index):
                net, _first, _second, selected, reason = \
                    self._select_prefab_pair(
                        first_lane_count=first_count,
                        second_lane_count=second_count,
                        second_lane_index=second_index,
                    )
                self.assertEqual(reason, "")
                path = net.connect_lane_sequence(selected, (1, 2, 3))
                self.assertTrue(path.valid, path.failure_reason)

    def test_prefab_boundary_never_densifies_an_unproven_chord(self):
        for x, y in ((1.0, 0.0), (0.0, 10.0)):
            with self.subTest(x=x, y=y):
                net, _first, _second, selected, reason = \
                    self._select_prefab_pair(second_x=x, second_y=y)
                self.assertTrue(reason)
                self.assertIn("prefab lane identity mismatch", reason)
                first, second = _first, _second
                connection = net._lane_connection(first, second)
                linked = replace(first, successors=(connection,))
                path = net.connect_lane_sequence(
                    (linked, second), (1, 2, 3))
                self.assertFalse(path.valid)

    def test_anonymization_scrubs_coordinate_arrays_and_their_nested_tail(self):
        record = {
            "route_build_id": "audit-export", "revision": 3,
            "status": "failed", "started_at": "2026-07-28T12:00:00Z",
            "context": {
                "world": {"x": 12345.67, "y": 89.0, "z": -76543.21},
                "gps_uid": 987654321,
                "candidate_lanes": [{
                    "coordinates": (
                        12346.67, 90.0, -76540.21,
                        {"node_uids": (123456789, 987654321),
                         "road_uid": 987654321,
                         "prefab_token": "private.prefab.token"},
                    ),
                    "note": "position 12346.67 / -76540.21",
                }],
            },
        }
        anonymized = anonymize_failure_record(record)
        encoded = json.dumps(anonymized, sort_keys=True)
        self.assertNotIn("12346.67", encoded)
        self.assertNotIn("-76540.21", encoded)
        self.assertNotIn("987654321", encoded)
        self.assertNotIn("123456789", encoded)
        self.assertNotIn("private.prefab.token", encoded)
        coordinates = anonymized["context"]["candidate_lanes"][0][
            "coordinates"]
        self.assertEqual(coordinates[:3], [1.0, 1.0, 3.0])
        self.assertTrue(coordinates[3]["road_uid"].startswith("anon-"))


if __name__ == "__main__":
    unittest.main()
