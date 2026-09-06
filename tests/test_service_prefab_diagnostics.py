import unittest

from core.navigation.lane_model import LaneId, LanePoint, LaneSegment
from core.navigation.road_network import RoadNetwork


def lane_segment(lane_id, start_uid, end_uid, points):
    return LaneSegment(
        lane_id, start_uid, end_uid, 1, 0, 1, 4.5, "derived", 17,
        None, "prefab", tuple(points),
        connector_curve_indices=lane_id.connector_path,
        gps_uids=frozenset((start_uid, end_uid)), gps_pair_index=-1)


class ModGer63DiagnosticReplayTests(unittest.TestCase):
    def setUp(self):
        self.net = RoadNetwork()
        token = "mod_ger_63"
        descriptor_uids = tuple(int(value, 16) for value in (
            "450863504a0d7dda", "45086350550d7655",
            "450863501d8d72b6", "45086350344d6d51"))
        self.instance = (token, descriptor_uids, 1, True)
        curves = [dict(nav_node_index=-1, next_lines=(), prev_lines=())
                  for _ in range(36)]

        def chain(indices, nav_node):
            for position, curve_index in enumerate(indices):
                curves[curve_index] = {
                    "nav_node_index": nav_node,
                    "next_lines": (() if position == len(indices)-1
                                   else (indices[position+1],)),
                    "prev_lines": (() if position == 0
                                   else (indices[position-1],)),
                }

        chain((1, 22, 16, 12), 4)
        chain((13, 29, 17, 25), 5)
        self.net._prefab_lane_data[token] = {
            "path": "prefab2/cross_temp/ger/ger_r1_x_r1_narrow_tmpl.ppd",
            "curves": tuple(curves),
            "nodes": (
                {"input_lanes": (1,), "output_lanes": (14,)},
                {"input_lanes": (35,), "output_lanes": (25,)},
                {"input_lanes": (13,), "output_lanes": (12,)},
                {"input_lanes": (2,), "output_lanes": (7,)},
            ),
        }
        source_start = int("450863504a0d7dda", 16)
        source_end = int("450863501d8d72b6", 16)
        self.source = lane_segment(
            LaneId(source_end, 1, 0, token, 1, (1, 22, 16, 12)),
            source_start, source_end,
            (LanePoint(21193.51, 50.921875, 17031.26),
             LanePoint(21191.99, 50.921875, 17031.08)))
        self.required = lane_segment(
            LaneId(source_end, 1, 0, token, 13, (13, 29, 17, 25)),
            source_end, int("45086350550d7655", 16),
            (LanePoint(21187.55, 50.921875, 17031.26),
             LanePoint(21185.32, 50.921875, 17012.78)))
        pair = (min(source_start, source_end), max(source_start, source_end))
        self.net._prefab_pairs[pair] = [self.instance]

        garage_instance = (
            "280", (int("45086350654d726a", 16), source_end), 0, True)
        self.net._prefab_lane_data["280"] = {
            "path": "prefab/garage_scandinavia/garage_sc.ppd",
            "curves": (), "nodes": ({}, {}),
        }
        cell = self.net._cell(21191.99, 17031.08)
        self.net._prefab_grid[cell] = [self.instance, garage_instance]

    def test_real_connector_paths_expose_no_directed_cross_lane_edge(self):
        available = self.net._prefab_lane_topology_evidence(
            self.source, self.instance)
        required = self.net._prefab_lane_topology_evidence(
            self.required, self.instance)
        self.assertEqual(available["connector_path"], [1, 22, 16, 12])
        self.assertEqual(available["input_descriptor_node_indices"], [0])
        self.assertEqual(available["output_descriptor_node_indices"], [2])
        self.assertTrue(available["curve_chain_links_proven"])
        self.assertEqual(required["connector_path"], [13, 29, 17, 25])
        self.assertEqual(required["input_descriptor_node_indices"], [2])
        self.assertEqual(required["output_descriptor_node_indices"], [1])
        self.assertTrue(required["curve_chain_links_proven"])
        available_edges = {
            (row["curve_index"], target)
            for row in available["curve_chain"]
            for target in row["next_lines"]}
        self.assertNotIn((12, 13), available_edges)
        self.assertEqual(available["drivable_boundary_source"], "unavailable")

    def test_shared_service_prefab_is_reported_but_not_invented_as_lane(self):
        adjacent = self.net._adjacent_prefab_data_evidence(
            self.source, self.instance)
        self.assertEqual(len(adjacent), 1)
        self.assertEqual(adjacent[0]["prefab_token"], "280")
        self.assertEqual(adjacent[0]["nav_curve_count"], 0)
        self.assertEqual(adjacent[0]["drivable_boundary_source"],
                         "unavailable")


if __name__ == "__main__":
    unittest.main()
