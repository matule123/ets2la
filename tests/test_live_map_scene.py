import unittest
from datetime import datetime
from unittest.mock import patch

from PyQt6.QtCore import QRectF

from core.navigation.road_network import RoadNetwork
from UI.map_page import MapView, navigation_trip_summary


class LiveMapSceneTests(unittest.TestCase):
    def test_navigation_trip_panel_uses_real_route_telemetry(self):
        summary = navigation_trip_summary({
            "game_route_distance": 469_400.0,
            "game_route_time": 27 * 60.0,
        }, now=datetime(2026, 7, 30, 22, 1))
        self.assertEqual(summary, ("22:28", 27, 469))
        self.assertIsNone(navigation_trip_summary({
            "game_route_distance": 0.0, "game_route_time": 0.0,
        }))

    def test_loading_statistics_deduplicate_spatial_prefab_instances(self):
        network = RoadNetwork()
        instance = ("junction", (1, 2, 3), 0, True)
        network.nodes = {1: (0.0, 0.0), 2: (10.0, 0.0)}
        network.segments = [((0.0, 0.0), (10.0, 0.0))]
        network._prefab_grid = {(0, 0): [instance], (1, 0): [instance]}
        self.assertEqual(network.load_statistics(), {
            "nodes": 2, "roads": 1, "prefabs": 1,
        })

    def test_live_map_poi_symbols_are_vectors_not_placeholder_letters(self):
        class Painter:
            def __init__(self):
                self.calls = []

            def __getattr__(self, name):
                def call(*_args, **_kwargs):
                    self.calls.append(name)
                return call

        painter = Painter()
        rect = QRectF(0.0, 0.0, 16.0, 16.0)
        for icon, kind in (("gas_ico", "facility"),
                           ("service_ico", "facility"),
                           ("parking_ico", "facility"),
                           ("viewpoint_ico", "viewpoint"),
                           ("recruitment_ico", "facility"),
                           ("", "company")):
            MapView._paint_feature_symbol(painter, rect, icon, kind)
        self.assertNotIn("drawText", painter.calls)
        self.assertTrue(any(name in painter.calls for name in
                            ("drawPath", "drawRoundedRect", "drawRect")))

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
                 "icon": "gas_ico", "label": "Central fuel"},
                {"x": 10.0, "y": 20.0, "type": "company",
                 "icon": "acme", "label": "ACME duplicate"},
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
        self.assertIn("Central fuel", {feature[4] for feature in features})
        self.assertEqual(sum(feature[2] == "company" for feature in features), 1)
        self.assertEqual(network.loaded, True)


if __name__ == "__main__":
    unittest.main()
