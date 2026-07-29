import math
import struct
import unittest

from PyQt6.QtCore import QPointF, QRectF
from PyQt6.QtGui import QPainterPath

from core.hud import (UltraPilotHUD, _continuous_lane_chunks,
                      _lane_boundary_points, _ordered_display_path_runs,
                      _rounded_screen_path, _variable_lane_boundary_points)
from core.sdk.scs_sdk import SCSTelemetry
from tests.test_lane_route_builder import SyntheticMap


class HudPoseStabilityTests(unittest.TestCase):
    def make_hud(self):
        hud = UltraPilotHUD.__new__(UltraPilotHUD)
        hud._display_truck_pos = None
        hud._display_truck_heading = None
        return hud

    def test_stationary_sdk_chatter_does_not_move_scene(self):
        hud = self.make_hud()
        first = hud._stabilize_display_pose((100.0, 200.0), 0.5, 0.0)
        second = hud._stabilize_display_pose((100.11, 199.91), 0.504, 0.0)
        self.assertEqual(first, second)

    def test_real_lateral_motion_is_preserved_outside_dead_band(self):
        hud = self.make_hud()
        hud._stabilize_display_pose((0.0, 0.0), 0.0, 40.0)
        (x, z), heading = hud._stabilize_display_pose((1.50, 0.0), 0.0, 40.0)
        self.assertAlmostEqual(x, 1.44, places=6)
        self.assertAlmostEqual(z, 0.0, places=6)
        self.assertAlmostEqual(heading, 0.0, places=6)

    def test_heading_wrap_is_stable(self):
        hud = self.make_hud()
        hud._stabilize_display_pose((0.0, 0.0), math.pi - 0.01, 30.0)
        _, heading = hud._stabilize_display_pose(
            (0.0, 0.1), -math.pi + 0.01, 30.0)
        error = (heading - (-math.pi + 0.01) + math.pi) % (2 * math.pi) - math.pi
        self.assertLess(abs(error), math.radians(0.13))

    def test_lane_match_does_not_move_model_twice_from_telemetry_origin(self):
        samples = (0.0, 0.35, 1.20, 2.35, -1.75)
        for lateral_error in samples:
            data = {
                "lane_revision": 12,
                "lane_match": {"lateral_error_m": lateral_error},
            }
            model_lateral = UltraPilotHUD._matched_ego_lateral(data)
            # The road was already transformed by subtracting truck_world_pos.
            # Moving the model by -lateral_error again depicts the lane centre,
            # not the real truck, and causes visible drift off the road.
            self.assertEqual(model_lateral, 0.0)

    def test_invalid_or_unmatched_lane_never_moves_model(self):
        self.assertEqual(UltraPilotHUD._matched_ego_lateral({
            "lane_revision": -1,
            "lane_match": {"lateral_error_m": 2.0},
        }), 0.0)

    def test_driving_view_renders_authoritative_xyz_path_without_legacy_shift(self):
        class Painter:
            def __init__(self):
                self.curves = 0
                self.polylines = 0

            def drawPath(self, _path):
                self.curves += 1

            def drawPolyline(self, _polyline):
                self.polylines += 1

            def __getattr__(self, _name):
                return lambda *_args, **_kwargs: None

        class View:
            def top(self): return 0.0
            def bottom(self): return 600.0
            def left(self): return 0.0
            def width(self): return 900.0
            def height(self): return 600.0
            def center(self): return QPointF(450.0, 300.0)

        hud = self.make_hud()
        hud._view_yaw = 0.0
        hud._road_scene_shift = 2.5
        hud._draw_low_poly_ego = lambda *_args, **_kwargs: None
        painter = Painter()
        data = {
            "pos": (100.0, 200.0), "heading": 0.0, "speed_kmh": 20.0,
            "altitude": 10.0, "road_segments": [], "traffic": [],
            "nav_path": [[100.0, 10.0, 200.0],
                         [100.0, 10.0, 194.0],
                         [101.0, 10.0, 188.0],
                         [102.0, 10.0, 182.0],
                         [104.0, 10.0, 176.0]],
            "lanes": 2, "lane_revision": 5,
            "trailer_attached": False,
        }
        hud._draw_driving_view(painter, View(), data)
        self.assertEqual(hud._road_scene_shift, 0.0)
        self.assertGreaterEqual(painter.curves, 4)
        self.assertEqual(painter.polylines, 0)
        self.assertEqual(UltraPilotHUD._matched_ego_lateral({
            "lane_revision": 2,
            "lane_match": {"lateral_error_m": float("nan")},
        }), 0.0)

    def test_confirmed_lane_boundaries_follow_roundabout_curve(self):
        radius = 18.0
        centreline = [
            (radius * math.sin(angle), radius * math.cos(angle), 0.0)
            for angle in (index * math.pi / 24 for index in range(13))
        ]
        left, right = _lane_boundary_points(centreline, 2.25)
        self.assertEqual(len(left), len(centreline))
        self.assertEqual(len(right), len(centreline))
        for centre, outer, inner in zip(centreline, left, right):
            self.assertAlmostEqual(math.dist(centre[:2], outer[:2]), 2.25,
                                   places=6)
            self.assertAlmostEqual(math.dist(centre[:2], inner[:2]), 2.25,
                                   places=6)
        # The middle of the quarter-circle remains curved; neither boundary
        # degenerates into a direct chord from entry to exit.
        chord_mid = ((left[0][0] + left[-1][0]) * .5,
                     (left[0][1] + left[-1][1]) * .5)
        self.assertGreater(math.dist(left[len(left) // 2][:2], chord_mid), 3.0)

    def test_projected_lane_uses_curves_not_rotated_straight_segments(self):
        points = [QPointF(0.0, 20.0), QPointF(8.0, 13.0),
                  QPointF(13.0, 5.0), QPointF(15.0, -5.0)]
        path = _rounded_screen_path(points)
        element_types = [path.elementAt(index).type
                         for index in range(path.elementCount())]
        self.assertIn(QPainterPath.ElementType.CurveToElement, element_types)
        self.assertEqual((path.elementAt(0).x, path.elementAt(0).y),
                         (points[0].x(), points[0].y()))
        self.assertEqual((path.elementAt(path.elementCount() - 1).x,
                          path.elementAt(path.elementCount() - 1).y),
                         (points[-1].x(), points[-1].y()))

    def test_lane_display_never_draws_across_unproven_junction_gap(self):
        samples = [(0.0, 0.0, 0.0), (3.0, 0.0, 0.0),
                   (20.0, 10.0, 0.0), (23.0, 10.0, 0.0)]
        chunks = _continuous_lane_chunks(samples)
        self.assertEqual(chunks, [samples[:2], samples[2:]])

    def test_live_trailer_heading_drives_articulation_across_wrap(self):
        data = {
            "trailer_attached": True,
            "heading": -math.pi + 0.05,
            "trailer_heading": math.pi - 0.10,
            "trailer_articulation": 0.0,
        }
        articulation = UltraPilotHUD._resolved_trailer_articulation(data)
        self.assertAlmostEqual(articulation, 0.15, places=6)

    def test_trailer_rotates_around_fixed_hitch_in_both_directions(self):
        straight_hinge, _straight_angle, straight_tail = (
            UltraPilotHUD._articulated_trailer_pose(0.8, 0.0))
        left_hinge, _left_angle, left_tail = (
            UltraPilotHUD._articulated_trailer_pose(0.8, math.radians(25.0)))
        right_hinge, _right_angle, right_tail = (
            UltraPilotHUD._articulated_trailer_pose(0.8, math.radians(-25.0)))
        self.assertEqual(straight_hinge, left_hinge)
        self.assertEqual(straight_hinge, right_hinge)
        self.assertNotAlmostEqual(left_tail[1], straight_tail[1])
        self.assertNotAlmostEqual(right_tail[1], straight_tail[1])
        self.assertLess((left_tail[1] - straight_tail[1])
                        * (right_tail[1] - straight_tail[1]), 0.0)
        for tail in (straight_tail, left_tail, right_tail):
            self.assertAlmostEqual(math.dist(straight_hinge, tail), 11.45,
                                   places=6)

    def test_hud_road_mesh_excludes_unconnected_parallel_road(self):
        synthetic = SyntheticMap()
        synthetic.node(1, 0.0, 20.0)
        synthetic.node(2, 0.0, -30.0)
        synthetic.node(3, 0.0, -80.0)
        synthetic.node(4, 15.0, 20.0)
        synthetic.node(5, 15.0, -80.0)
        synthetic.road(1, 2)
        synthetic.road(2, 3)
        synthetic.road(4, 5)
        segments = synthetic.net.hud_segments_3d_near(
            (0.0, 0.0), radius=120.0, altitude=0.0)
        self.assertTrue(segments)
        xs = [point[0] for segment in segments
              for point in segment[:2]]
        self.assertLess(max(abs(x) for x in xs), 1.0)
        zs = [point[1] for segment in segments
              for point in segment[:2]]
        self.assertLess(min(zs), -50.0)

    def test_hud_uses_offset_lane_carriageway_under_truck_model(self):
        synthetic = SyntheticMap()
        synthetic.node(1, 0.0, 0.0)
        synthetic.node(2, 0.0, 100.0)
        index = synthetic.road(1, 2, 4)
        synthetic.look("divided-offset", 2, 2, 5.75)
        synthetic.set_road_look(index, "divided-offset")
        lane = next(item for item in synthetic.net._build_lane_segments(index)
                    if item.direction == 1 and item.lane_index == 0)
        truck = lane.centerline[len(lane.centerline) // 2]
        segments = synthetic.net.hud_segments_3d_near(
            (truck.x, truck.z), radius=80.0, altitude=truck.y)
        roads = [segment for segment in segments if segment[2] == "road"]
        self.assertTrue(roads)
        # A divided road is published as two real two-lane carriageways,
        # rather than one undersized ribbon around the raw item centre.
        self.assertEqual({segment[3] for segment in roads}, {2})
        self.assertTrue(all(len(segment) >= 9 for segment in roads))
        covering = []
        for segment in roads:
            a, b, half = segment[0], segment[1], segment[8]
            if min(a[1], b[1]) <= truck.z <= max(a[1], b[1]):
                centre_x = (a[0] + b[0]) * .5
                covering.append(centre_x-half <= truck.x <= centre_x+half)
        self.assertIn(True, covering)

    def test_connected_prefab_curves_publish_continuous_lane_envelope(self):
        synthetic = SyntheticMap()
        synthetic.node(1, 0.0, 0.0)
        synthetic.node(2, 0.0, 80.0)
        synthetic.road(1, 2)
        calls = []

        def prefab_segments(pos, radius, limit, allowed_node_uids,
                            include_path_metadata=False):
            calls.append(frozenset(allowed_node_uids))
            segment = ((0.0, 32.0, 0.0), (7.0, 38.0, 0.0))
            return [segment + ("p0:0", 0)] if include_path_metadata else [segment]

        synthetic.net.prefab_segments_3d_near = prefab_segments
        segments = synthetic.net.hud_segments_3d_near(
            (0.0, 20.0), radius=100.0, altitude=0.0)
        prefab = [segment for segment in segments if segment[2] == "lane"]
        self.assertEqual(len(prefab), 1)
        self.assertEqual(calls, [frozenset((1, 2))])
        self.assertFalse(prefab[0][5])       # no dashed line
        self.assertTrue(prefab[0][9])        # no invented painted divider
        self.assertEqual(prefab[0][10:], ("p0:0", 0))

    def test_prefab_lane_outline_is_curved_and_never_chords_between_arms(self):
        samples = []
        radius = 16.0
        points = [(radius * math.sin(index * math.pi / 24),
                   radius * math.cos(index * math.pi / 24), 0.0)
                  for index in range(13)]
        for index, (first, second) in enumerate(zip(points, points[1:])):
            samples.append((index, first, second, 2.25))
        runs = _ordered_display_path_runs(samples)
        self.assertEqual(len(runs), 1)
        centreline = [item[0] for item in runs[0]]
        widths = [item[1] for item in runs[0]]
        left, right = _variable_lane_boundary_points(centreline, widths)
        self.assertEqual(len(left), len(points))
        self.assertEqual(len(right), len(points))
        chord_mid = ((left[0][0] + left[-1][0]) * .5,
                     (left[0][1] + left[-1][1]) * .5)
        self.assertGreater(math.dist(left[len(left)//2][:2], chord_mid), 2.5)

        # A missing curve sample is never bridged by the display smoother.
        split = samples[:4] + [
            (8, (50.0, 50.0, 0.0), (52.0, 50.0, 0.0), 2.25)]
        self.assertEqual(len(_ordered_display_path_runs(split)), 2)

    def test_hud_draws_prefab_and_suppressed_approach_outer_boundaries(self):
        class Painter:
            def __init__(self): self.paths = 0
            def drawPath(self, _path): self.paths += 1
            def __getattr__(self, _name): return lambda *_a, **_k: None

        class View:
            def top(self): return 0.0
            def bottom(self): return 600.0
            def left(self): return 0.0
            def width(self): return 900.0
            def height(self): return 600.0
            def center(self): return QPointF(450.0, 300.0)

        hud = self.make_hud()
        hud._view_yaw = 0.0
        hud._draw_low_poly_ego = lambda *_args, **_kwargs: None
        data = {
            "pos": (0.0, 0.0), "heading": 0.0, "speed_kmh": 15.0,
            "altitude": 0.0, "traffic": [], "nav_path": [], "lanes": 1,
            "lane_revision": -1, "trailer_attached": False,
            "road_segments": [
                [[0.0, -5.0, 0.0], [0.0, -10.0, 0.0], "road", 2,
                 False, True, False, False, 5.0, True, "r0:0", 0],
                [[0.0, -10.0, 0.0], [2.0, -14.0, 0.0], "lane", 1,
                 False, False, False, False, 3.05, True, "p0:0", 0],
                [[2.0, -14.0, 0.0], [5.0, -17.0, 0.0], "lane", 1,
                 False, False, False, False, 3.05, True, "p0:0", 1],
            ],
        }
        painter = Painter()
        hud._draw_driving_view(painter, View(), data)
        # Two road edges and two continuously curved prefab-lane edges.
        self.assertGreaterEqual(painter.paths, 4)

    def test_traffic_light_is_world_anchored_and_never_uses_string_brush(self):
        class StrictPainter:
            def setBrush(self, brush):
                if isinstance(brush, str):
                    raise TypeError("QPainter.setBrush must not receive str")

            def __getattr__(self, _name):
                return lambda *_args, **_kwargs: None

        hud = self.make_hud()
        hud._view_yaw = 0.0
        projections = []
        original_project = hud._project

        def project(ahead, lateral, view, height=0.0):
            point = original_project(ahead, lateral, view, height)
            projections.append((ahead, lateral, height, point))
            return point

        hud._project = project
        hud._draw_light(
            StrictPainter(), QRectF(0.0, 0.0, 900.0, 600.0),
            {"color": "red", "time_left": 7.2},
            ahead=30.0, lateral=5.0, ground=0.0)
        self.assertTrue(projections)
        self.assertTrue(all(29.9 <= ahead <= 30.0
                            for ahead, _lateral, _height, _point in projections))
        # A signal 5 m right of the truck projects near the scene centre. The
        # removed overlay implementation was always pinned at x ~= 836.
        self.assertLess(max(point.x() for *_rest, point in projections
                            if point is not None), 700.0)

    def test_sdk_reads_attached_after_all_eighty_trailer_flags(self):
        sdk = SCSTelemetry()
        sdk.mm = bytearray(sdk.mmap_size)
        base = sdk.TRAILER_BLOCK_START
        sdk.mm[base + 80] = 1
        sdk.mm[base + 81] = 0  # padding must not be read as attached
        for offset, value in ((872, 10.0), (880, 20.0), (888, 30.0),
                              (896, 0.25), (904, 0.0), (912, 0.0)):
            sdk.mm[base + offset:base + offset + 8] = struct.pack("d", value)
        trailer = sdk.read_trailer(0)
        self.assertTrue(trailer["attached"])
        self.assertEqual((trailer["worldX"], trailer["worldY"],
                          trailer["worldZ"]), (10.0, 20.0, 30.0))


if __name__ == "__main__":
    unittest.main()
