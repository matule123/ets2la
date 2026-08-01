import unittest
from unittest.mock import patch

from core.navigation.road_network import RoadNetwork


class LiveMapSceneTests(unittest.TestCase):
    def test_live_map_keeps_nearby_disconnected_roads_outside_hud_only(self):
        network = RoadNetwork()
        network.loaded = True
        network.nodes = {
            1: (0.0, 0.0), 2: (100.0, 0.0),
            3: (0.0, 35.0), 4: (100.0, 35.0),
        }
        network.node_alt = {uid: 0.0 for uid in network.nodes}
        network.node_forward = {uid: (1.0, 0.0) for uid in network.nodes}
        network.segments = [
            ((0.0, 0.0), (100.0, 0.0)),
            ((0.0, 35.0), (100.0, 35.0)),
        ]
        network._seg_uids = [(1, 2), (3, 4)]
        network._seg_road_uids = [10, 11]
        network._seg_look_tokens = ["local", "local"]
        network.road_looks = {
            "local": {"type": "local", "lanes": 2,
                      "lanes_left": 1, "lanes_right": 1},
        }
        network._seg_grid = {(0, 0): [0, 1]}
        network._nearest_segment_index = lambda _pos: 0
        network._build_lane_segments = lambda _index: ()
        network._road_curve_3d = lambda first, second: [
            (*network.nodes[first], 0.0), (*network.nodes[second], 0.0)]
        network.prefab_segments_3d_near = lambda *args, **kwargs: []

        hud = network.hud_segments_3d_near((10.0, 0.0), radius=100.0)
        live = network.live_map_segments_3d_near(
            (10.0, 0.0), radius=100.0)
        self.assertEqual({item[10].split(":", 1)[0] for item in hud}, {"r0"})
        self.assertEqual({item[10].split(":", 1)[0] for item in live},
                         {"r0", "r1"})

    def test_prefab_polygons_use_real_placed_neighbour_loop_geometry(self):
        network = RoadNetwork()
        network.loaded = True
        network.nodes = {1: (100.0, 200.0)}
        network.node_rot = {1: 0.0}
        network._prefab_desc = {"junction": (((0.0, 0.0, 0.0),), (), ())}
        instance = ("junction", (1,), 0, True)
        network._prefab_grid = {(0, 0): [instance]}
        network._prefab_map_polygons = {
            "junction": ((((0.0, 0.0), (20.0, 0.0),
                              (20.0, 10.0), (0.0, 10.0)), 2, 2),),
        }
        polygons = network.live_map_polygons_near(
            (105.0, 205.0), radius=80.0)
        self.assertEqual(len(polygons), 1)
        points, colour, z_index = polygons[0]
        self.assertEqual((colour, z_index), (2, 2))
        self.assertEqual(points[0], (100.0, 200.0))
        self.assertEqual(points[2], (120.0, 210.0))

    def test_optional_map_features_are_indexed_without_affecting_network(self):
        network = RoadNetwork()
        network.loaded = True
        payloads = {
            "companyDefs": [
                {"token": "acme", "name": "ACME"},
            ],
            "companies": [
                {"x": 10.0, "y": 20.0, "token": "acme"},
            ],
            "pois": [
                {"x": 12.0, "y": 21.0, "type": "facility",
                 "icon": "gas_ico"},
            ],
            "cities": [
                {"x": 14.0, "y": 19.0, "token": "town",
                 "name": "Town"},
            ],
        }
        with patch("core.navigation.road_network._find_json",
                   side_effect=lambda _root, category: category), \
                patch("core.navigation.road_network._loadf",
                      side_effect=lambda path: payloads[path]):
            network._load_map_features("unused")

        features = network.map_features_near((10.0, 20.0), radius=50.0)
        self.assertEqual({feature[2] for feature in features},
                         {"company", "facility", "city"})
        self.assertEqual(network.loaded, True)


if __name__ == "__main__":
    unittest.main()
