import math
import multiprocessing as mp
import threading
import time
import unittest

from core.ar_overlay import AROverlay
from core.hud import UltraPilotHUD
from core.engine import UltraPilotEngine
from core.ipc.shared_state import SharedState
from core.navigation.lane_model import (
    GpsCorridorEdge, LaneId, LaneLocator, LaneMatch, LanePath, LanePoint,
    LaneSegment,
)
from core.navigation.lane_trajectory import build_lane_trajectory
from core.navigation.road_network import RoadNetwork
from core.navigation.route import Route
from plugins.autopilot.main import Plugin as AutopilotPlugin
from tests.test_lane_authority_integration import (
    Controller, MapSDK, State, Tags, Telemetry, build_map_plugin,
)
from tests.test_lane_route_builder import SyntheticMap
from tests.test_lane_trajectory import single_lane_path


def hud_reader(state):
    return type("HUDReader", (), {
        "shared_state": state, "_rear_cam_side": "off",
        "_rear_cam_until": 0.0,
    })()


def autopilot_state(confidence, *, valid=True, heartbeat=None,
                    telemetry_valid=True):
    now = time.monotonic() if heartbeat is None else heartbeat
    state = State({
        "system_state": "CRUISE", "danger_level": 0.0,
        "lane_offset": 0.9, "traffic": [], "nav_active": True,
        "nav_steering": 0.4, "acc_throttle": 0.0, "acc_brake": 0.0,
        "autopilot_active": True, "game_route_distance": 100.0,
        "game_route_node_uids": [1, 2], "telemetry_valid": telemetry_valid,
        "lane_trajectory_heartbeat": now, "lane_trajectory_revision": 7,
        "lane_trajectory": {
            "revision": 7, "valid": valid, "confidence": confidence,
            "source_gps_uids": [1, 2],
            "points": [[0, 0, 0], [0, 0, 10]],
            "display_points": [[0, 0, 0], [0, 0, 10]],
        },
    })
    sdk = type("SDK", (), {})()
    sdk.shared_state, sdk.controller, sdk.telemetry = state, Controller(), Telemetry()
    plugin = AutopilotPlugin(sdk)
    plugin.tags = Tags()
    plugin.on_start()
    return plugin, state


class LaneGeometryAuditTests(unittest.TestCase):
    def test_runtime_path_rejects_parallel_first_lane_offset(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 40)
        m.node(10, 12, 0); m.node(11, 12, 40)
        active_index = m.road(1, 2, 1)
        m.road(10, 11, 1)
        active = next(lane for lane in m.net._build_lane_segments(active_index)
                      if lane.direction == 1)
        m.net._lane_id_index[active.lane_id] = active
        match = LaneMatch(active.lane_id, active.centerline[1], 0, 1,
                          0.0, 0.0, 0.0, 0.0, 1.0, "test")

        # Force a valid GPS corridor on the nearby but disconnected road.  It
        # must be rejected rather than drawing a lateral jump to that road.
        path, returned = m.net.build_lane_path(
            (10, 11), (active.centerline[1].x, active.centerline[1].z),
            active.centerline[1].heading, start_match=match)
        self.assertIs(returned, match)
        self.assertFalse(path.valid)
        self.assertIn("does not connect", path.failure_reason)

    def test_lanes_right_is_physically_right_under_ets2_heading_convention(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 40)
        index = m.road(1, 2, 2)
        token = m.net._seg_look_tokens[index]
        look = m.net.road_looks[token]
        look.update({
            "lane_types_left": ("traffic_lane.road.local",) * 2,
            "lanes_left": 2, "lanes_right": 2, "offset_m": 2.0,
        })
        m.net._lane_cache.clear()
        lanes = m.net._build_lane_segments(index)
        right = [lane for lane in lanes if lane.direction == 1]
        left = [lane for lane in lanes if lane.direction == -1]
        # start->end is +Z; project convention says physical right is -X.
        self.assertEqual([round(l.centerline[0].x, 2) for l in right],
                         [-3.25, -7.75])
        self.assertEqual([round(l.centerline[-1].x, 2) for l in left],
                         [3.25, 7.75])
        self.assertEqual(right[0].width_m, 4.5)
        self.assertEqual(right[0].width_source, "derived")
        self.assertIsNone(right[0].left_neighbor)
        self.assertEqual(right[0].right_neighbor, right[1].lane_id)
        self.assertEqual(right[1].left_neighbor, right[0].lane_id)

    def test_lateral_and_steering_signs_are_consistent(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 40)
        index = m.road(1, 2, 1)
        lane = next(l for l in m.net._build_lane_segments(index)
                    if l.direction == 1)
        point = lane.centerline[3]
        # Physical right of a +Z path is -X: locator error is positive-right.
        match = LaneLocator(m.net).locate(
            (point.x - 1.0, point.y, point.z), point.heading, (1, 2))
        self.assertGreater(match.lateral_error_m, 0.0)
        # A truck right of the target must steer left (negative).
        route = Route([[point.x, point.y, 0.0], [point.x, point.y, 80.0]])
        self.assertLess(route.steering((point.x - 1.0, point.z),
                                       point.heading, 10.0), 0.0)

    def test_gentle_curve_cannot_wind_steering_at_standstill(self):
        points = [[0.0, 0.0, 0.0], [1.0, 0.0, 10.0],
                  [3.0, 0.0, 20.0], [6.0, 0.0, 30.0],
                  [10.0, 0.0, 40.0], [15.0, 0.0, 50.0],
                  [21.0, 0.0, 60.0]]
        route = Route(points)
        heading = math.pi
        self.assertGreater(route.curvature_ahead((0.0, 0.0), heading), 100.0)
        self.assertLessEqual(abs(route.steering(
            (0.0, 0.0), heading, speed_ms=0.0)), 0.22 + 1e-9)

    def test_equal_parallel_candidates_are_rejected_without_history(self):
        def lane(uid, x):
            lid = LaneId(uid, 1, 0)
            pts = tuple(LanePoint(x, 0, z, heading=math.pi)
                        for z in (0.0, 20.0, 40.0))
            return LaneSegment(lid, 1, 2, 1, 0, 1, 4.5, "derived", 0,
                               "look", "traffic_lane.road.local", pts,
                               gps_uids=frozenset((1, 2)))
        net = type("N", (), {
            "lane_segments_near": lambda self, pos, radius: [lane(10, -1), lane(20, 1)],
            "lanes_connected": lambda self, a, b: a == b,
        })()
        self.assertIsNone(LaneLocator(net).locate((0, 0, 10), math.pi, (1, 2)))

    def test_locator_cannot_jump_to_disconnected_parallel_road(self):
        def lane(uid, x):
            lid = LaneId(uid, 1, 0)
            pts = tuple(LanePoint(x, 0, z, heading=math.pi)
                        for z in (0.0, 20.0, 40.0))
            return LaneSegment(lid, uid, uid+1, 1, 0, 1, 4.5,
                               "derived", 0, "look",
                               "traffic_lane.road.local", pts,
                               gps_uids=frozenset((uid, uid+1)))
        first, parallel = lane(10, 0.0), lane(20, 1.0)
        net = type("N", (), {
            "lanes": [first],
            "lane_segments_near": lambda self, pos, radius: self.lanes,
            "lanes_connected": lambda self, a, b: a == b,
        })()
        locator = LaneLocator(net)
        previous = locator.locate((0, 0, 10), math.pi, (10, 11))
        self.assertEqual(previous.lane_id, first.lane_id)
        net.lanes = [parallel]
        self.assertIsNone(locator.locate((1, 0, 20), math.pi, (20, 21), previous))

    def test_missing_middle_uid_and_graph_only_gap_are_rejected(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 80)
        missing = m.net.resolve_gps_corridor((1, 99, 2))
        self.assertFalse(missing.valid)
        self.assertIn("absent", missing.failure_reason)
        m.net.fwd[1] = [2]
        corridor = m.net.resolve_gps_corridor((1, 2))
        self.assertTrue(corridor.valid)
        fake_lane = LaneId(1, 1, 0)
        match = LaneMatch(fake_lane, LanePoint(0, 0, 0), 0, 0,
                          0, 0, 0, 0, 1, "test")
        segments, reason = m.net.select_lane_sequence(corridor, match)
        self.assertEqual(segments, ())
        self.assertIn("no lane-confirmed geometry", reason)

    def test_prefab_wrong_exit_and_ambiguous_roundabout_fail_closed(self):
        net = RoadNetwork(); net.loaded = True
        net.nodes.update({1: (0.0, 0.0), 2: (0.0, 10.0), 3: (10.0, 10.0)})
        net.node_alt.update({1: 0.0, 2: 0.0, 3: 0.0})
        net.node_rot.update({1: 0.0, 2: 0.0, 3: 0.0})
        token = "roundabout-test"
        net._prefab_desc[token] = (
            ((0.0, 0.0, 0.0), (0.0, 10.0, 0.0)),
            ((0.0, 0.0, 0.0, 10.0, 0.0, 1.0, 0.0, 1.0),),
            (("physical", 0, ((1, (0,)),)), ("physical", 1, ())),
        )
        net._prefab_lane_data[token] = {
            "path": "roundabout", "nodes": (
                {"input_lanes": (0,), "output_lanes": (), "y": 0.0},
                {"input_lanes": (), "output_lanes": (0,), "y": 0.0},
            ),
            "curves": ({"nav_node_index": 0, "next_lines": (),
                        "prev_lines": (), "start_y": 0.0, "end_y": 0.0},),
        }
        instance = (token, (1, 2), 0)
        wrong = GpsCorridorEdge(1, 3, "prefab", 0,
                                prefab_instance=(instance,))
        segment, reason = net._prefab_lane_segment(wrong, 0)
        self.assertIsNone(segment)
        self.assertIn("missing", reason)
        ambiguous = GpsCorridorEdge(1, 2, "prefab", 0,
                                    prefab_instance=(instance, instance))
        segment, reason = net._prefab_lane_segment(ambiguous, 0)
        self.assertIsNone(segment)
        self.assertIn("ambiguous", reason)

    def test_prefab_origin_node_index_controls_world_anchor(self):
        net = RoadNetwork(); net.loaded = True
        net.nodes.update({1: (100.0, 100.0), 2: (1000.0, 1000.0)})
        net.node_alt.update({1: 1.0, 2: 20.0})
        net.node_rot.update({1: 0.0, 2: 0.0})
        token = "origin-index-test"
        net._prefab_desc[token] = (
            ((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.0, 10.0, 0.0, 1.0, 0.0, 1.0),), ())
        net._prefab_lane_data[token] = {
            "nodes": ({"y": 0.0}, {"y": 0.0}),
            "curves": ({"start_y": 0.0, "end_y": 0.0},),
        }
        points = net._prefab_curve_chain_3d((token, (1, 2), 1), (0,))
        self.assertAlmostEqual(points[0].x, 1000.0)
        self.assertAlmostEqual(points[0].z, 1000.0)
        self.assertAlmostEqual(points[0].y, 20.0)

    def test_descriptor_order_uses_origin_node_position_and_rotation(self):
        net = RoadNetwork(); net.loaded = True
        net.nodes.update({1: (100.0, 100.0), 2: (1000.0, 1000.0)})
        net.node_rot.update({1: 0.0, 2: math.pi / 2.0})
        token = "descriptor-order-transform"
        net._prefab_desc[token] = (
            ((-10.0, 0.0, math.pi / 2.0), (0.0, 0.0, 0.0)), (), ())
        # New format: UIDs are in PPD descriptor order. originNodeIndex=1
        # therefore selects UID 1 for both translation and rotation.
        transformed = net._transform_prefab_points(
            (token, (2, 1), 1, True), ((0.0, 10.0),))
        self.assertAlmostEqual(transformed[0][0], 100.0)
        self.assertAlmostEqual(transformed[0][1], 110.0)


class TrajectoryNegativeAuditTests(unittest.TestCase):
    def test_zero_duplicate_reversed_nan_and_infinity_are_rejected(self):
        cases = (
            ([(0, 0, 0), (0, 0, 0)], "duplicate"),
            ([(0, 0, 0), (0, 0, 10), (0, 0, 5)], "reverses direction"),
            ([(0, 0, 0), (math.nan, 0, 10)], "non-finite"),
            ([(0, 0, 0), (math.inf, 0, 10)], "non-finite"),
        )
        for coordinates, reason in cases:
            with self.subTest(reason=reason):
                result = build_lane_trajectory(single_lane_path(coordinates))
                self.assertFalse(result.valid)
                self.assertIn(reason, result.failure_reason)


class SnapshotAndConsumerAuditTests(unittest.TestCase):
    def test_delayed_old_build_cannot_overwrite_changed_target(self):
        plugin, sdk, point = build_map_plugin()
        original = plugin.road_net.build_lane_path
        sdk.set("game_route_node_uids", [1, 2])

        def delayed_old(*args, **kwargs):
            result = original(*args, **kwargs)
            new_revision = sdk.get("lane_trajectory_revision") + 1
            sdk.shared_state.update_batch({
                "game_route_node_uids": [2, 3],
                "lane_trajectory_revision": new_revision,
                "lane_trajectory": {
                    "revision": new_revision, "valid": False,
                    "confidence": 0.0, "points": [], "display_points": [],
                    "source_gps_uids": [2, 3],
                    "failure_reason": "new-target-wins",
                },
            })
            return result

        plugin.road_net.build_lane_path = delayed_old
        plugin._update_lane_trajectory((point.x, point.z), point.heading)
        self.assertEqual(sdk.get("lane_trajectory")["failure_reason"],
                         "new-target-wins")
        self.assertEqual(sdk.get("lane_trajectory")["source_gps_uids"], [2, 3])

    def test_map_or_telemetry_loss_hides_consumers_and_blocks_autopilot(self):
        plugin, sdk, _ = build_map_plugin()
        sdk.set("lane_trajectory_heartbeat", time.monotonic() - 2.0)
        self.assertEqual(UltraPilotHUD._read(hud_reader(sdk.shared_state))["nav_path"], [])
        ar = type("ARReader", (), {"state": sdk.shared_state})()
        self.assertEqual(AROverlay._current_display_points(ar), (-1, []))
        ap, state = autopilot_state(0.95, heartbeat=time.monotonic() - 2.0)
        ap.on_tick(0.1)
        self.assertEqual(state.get("autopilot_lane_revision"), -1)

        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = State({
            "autopilot_active": True, "telemetry_valid": False,
            "ctl_steering": 0.8, "ctl_throttle": 1.0, "ctl_brake": 0.0,
        })
        engine.controller = Controller()
        engine._was_active = True
        engine._flush_controls()
        self.assertGreater(engine.controller.steering, 0.0)
        self.assertLess(engine.controller.steering, 0.8)
        self.assertEqual(engine.controller.throttle, 0.0)
        self.assertGreater(engine.controller.brake, 0.0)
        self.assertLess(engine.controller.brake, 0.70)
        for _ in range(20):
            engine._last_control_flush -= 0.1
            engine._flush_controls()
        self.assertEqual(engine.controller.steering, 0.0)
        self.assertEqual(engine.controller.brake, 0.70)

        engine.shared_state = State({
            "autopilot_active": True, "telemetry_valid": True,
            "autopilot_control_heartbeat": time.monotonic() - 1.0,
            "ctl_steering": 0.8, "ctl_throttle": 1.0, "ctl_brake": 0.0,
        })
        engine.controller = Controller()
        engine._last_output_steering = 0.8
        engine._last_output_brake = 0.0
        engine._last_control_flush = time.monotonic() - 0.02
        engine._flush_controls()
        self.assertGreater(engine.controller.steering, 0.0)
        self.assertLess(engine.controller.steering, 0.8)
        self.assertEqual(engine.controller.throttle, 0.0)
        self.assertGreater(engine.controller.brake, 0.0)
        self.assertLess(engine.controller.brake, 0.70)
        ap, state = autopilot_state(0.95, telemetry_valid=False)
        ap.on_tick(0.1)
        self.assertEqual(state.get("autopilot_lane_revision"), -1)

    def test_confidence_threshold_below_equal_and_above_is_unambiguous(self):
        for confidence, expected in ((0.719999, -1), (0.72, 7), (0.720001, 7)):
            with self.subTest(confidence=confidence):
                plugin, state = autopilot_state(confidence)
                plugin.on_tick(0.05)
                self.assertEqual(state.get("autopilot_lane_revision"), expected)
                readiness = state.get("autopilot_navigation_readiness")
                self.assertEqual(readiness["ready"], expected == 7)
                if confidence < 0.72:
                    self.assertIn("below 0.72", readiness["reason"])
                else:
                    self.assertEqual(readiness["reason"], "")

    def test_camera_matrix_requires_proven_metadata_and_time_sync(self):
        now = time.monotonic()
        state = State({"telemetry_timestamp": now})
        overlay = type("Projection", (), {
            "state": state, "width": lambda self: 100,
            "height": lambda self: 100,
        })()
        self.assertIsNone(AROverlay._project_world(overlay, [0, 0, 0]))
        state.update_batch({
            "game_camera_view_projection": [1.0] * 15,
            "game_camera_view_projection_meta": {
                "layout": "row-major", "handedness": "right-handed",
                "clip_space": "opengl-negative-one-to-one", "timestamp": now,
            },
        })
        self.assertIsNone(AROverlay._project_world(overlay, [0, 0, 0]))
        state.set("game_camera_view_projection", [math.nan] + [0.0] * 14 + [1.0])
        self.assertIsNone(AROverlay._project_world(overlay, [0, 0, 0]))

    def test_concurrent_publication_never_accepts_mixed_revision(self):
        manager = None
        try:
            manager = mp.Manager()
            raw_state = manager.dict()
        except (OSError, PermissionError):
            # Restricted CI/sandboxes can deny Windows named pipes. The same
            # publication/read protocol is still exercised in-process there;
            # the audit also runs this test unsandboxed against Manager.dict.
            raw_state = {}
        shared = SharedState(raw_state)
        try:
            shared.update_batch({
                "lane_trajectory_revision": 0,
                "lane_trajectory": {"revision": 0, "points": [[0, 0, 0]]},
            })
            failures = []

            def writer():
                for revision in range(1, 250):
                    shared.update_batch({
                        "lane_trajectory_revision": revision,
                        "lane_trajectory": {
                            "revision": revision,
                            "points": [[revision, revision, revision]],
                        },
                    })

            thread = threading.Thread(target=writer)
            thread.start()
            while thread.is_alive():
                snapshot = shared.get("lane_trajectory", {})
                revision = shared.get("lane_trajectory_revision", -1)
                # Mismatches are rejected by consumers. If revisions agree,
                # the nested geometry must belong to that same publication.
                if snapshot.get("revision") == revision:
                    if snapshot.get("points") != [[revision, revision, revision]]:
                        failures.append((snapshot, revision))
            thread.join()
            self.assertEqual(failures, [])
        finally:
            if manager is not None:
                manager.shutdown()


if __name__ == "__main__":
    unittest.main()
