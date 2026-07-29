import unittest
import time
import math
from dataclasses import replace
from unittest import mock

from core.ar_overlay import AROverlay
from core.hud import UltraPilotHUD
from core.navigation.route import Route
from core.navigation.lane_model import LaneId
from plugins.autopilot.main import (
    Plugin as AutopilotPlugin, lane_authority_rejection_reason,
)
from plugins.map.main import (
    LANE_MATCH_GRACE_FRAMES, RUNTIME_ROUTE_HORIZON_M,
    RUNTIME_ROUTE_MAX_UIDS, Plugin as MapPlugin,
)
from UI.map_page import (
    MapView, live_map_navigation_points,
    rejected_navigation_command_message,
)
from tests.test_lane_route_builder import SyntheticMap


class State:
    def __init__(self, values=None):
        self.data = dict(values or {})
        self.batches = []

    def get(self, key, default=None):
        return self.data.get(key, default)

    def set(self, key, value):
        self.data[key] = value

    def update_batch(self, values):
        self.batches.append(dict(values))
        self.data.update(values)


class MapSDK:
    def __init__(self, values=None):
        self.shared_state = State(values)

    def get(self, key, default=None):
        return self.shared_state.get(key, default)

    def set(self, key, value):
        self.shared_state.set(key, value)


class Controller:
    def __init__(self):
        self.steering = self.throttle = self.brake = 0.0

    def set_steering(self, value): self.steering = value
    def set_throttle(self, value): self.throttle = value
    def set_brake(self, value): self.brake = value
    def set_blinker(self, value): pass
    def pay_toll(self): pass


class Tags:
    pass


class Telemetry:
    def get(self, key, default=None):
        return {"speed": 15.0} if key == "truck" else default


def build_map_plugin(y=3.0):
    synthetic = SyntheticMap()
    synthetic.node(1, 0, 0, y)
    synthetic.node(2, 0, 40, y)
    synthetic.node(3, 0, 80, y)
    first = synthetic.road(1, 2, 2)
    synthetic.road(2, 3, 2)
    lane = synthetic.net._build_lane_segments(first)[0]
    point = lane.centerline[2]
    sdk = MapSDK({
        "game_route_node_uids": [1, 2, 3],
        "truck_altitude": y,
        "lane_trajectory_revision": 0,
    })
    plugin = MapPlugin(sdk)
    plugin.on_start()
    plugin.road_net = synthetic.net
    plugin._net_attempted = True
    plugin._update_lane_trajectory((point.x, point.z), point.heading)
    return plugin, sdk, point


class LaneAuthorityIntegrationTests(unittest.TestCase):
    def test_rolling_gps_prefix_does_not_invalidate_valid_snapshot(self):
        plugin, sdk, point = build_map_plugin()
        before = dict(sdk.get("lane_trajectory"))
        revision = before["revision"]
        route_build_id = before["route_build_id"]
        points = list(before["points"])
        request = sdk.get("nav_recalc_request")
        lane_match = plugin._lane_match
        batch_count = len(sdk.shared_state.batches)

        sdk.set("game_route_node_uids", [2, 3])
        plugin._update_lane_trajectory((point.x, point.z), point.heading)

        after = sdk.get("lane_trajectory")
        self.assertTrue(after["valid"])
        self.assertEqual(after["revision"], revision)
        self.assertEqual(after["route_build_id"], route_build_id)
        self.assertEqual(after["points"], points)
        self.assertEqual(after["source_gps_uids"], [2, 3])
        self.assertEqual(after["covered_gps_uids"], [2, 3])
        self.assertEqual(sdk.get("nav_recalc_request"), request)
        self.assertEqual(sdk.get("nav_path"), after["display_points"])
        self.assertEqual(sdk.get("map_path"), after["points"])
        self.assertEqual(sdk.get("nav_trajectory_revision"), revision)
        self.assertIs(plugin._lane_match, lane_match)
        new_batches = sdk.shared_state.batches[batch_count:]
        self.assertFalse(any(
            isinstance(batch.get("lane_trajectory"), dict)
            and not batch["lane_trajectory"].get("valid", False)
            for batch in new_batches))

    def test_normal_active_lane_segment_change_does_not_schedule_build(self):
        plugin, sdk, _point = build_map_plugin()
        snapshot = sdk.get("lane_trajectory")
        revision = snapshot["revision"]
        build_id = snapshot["route_build_id"]
        segments = plugin._lane_path.segments

        near_end = segments[0].centerline[-2]
        plugin._update_lane_trajectory(
            (near_end.x, near_end.z), near_end.heading)
        on_next = segments[1].centerline[1]
        plugin._update_lane_trajectory(
            (on_next.x, on_next.z), on_next.heading)

        current = sdk.get("lane_trajectory")
        self.assertEqual(current["revision"], revision)
        self.assertEqual(current["route_build_id"], build_id)
        self.assertEqual(current["points"], snapshot["points"])
        self.assertEqual(plugin._lane_match.lane_id, segments[1].lane_id)

    def test_rolling_rebase_refreshes_only_near_horizon_end(self):
        plugin, sdk, _point = build_map_plugin()
        snapshot = dict(sdk.get("lane_trajectory"))
        snapshot.update({
            "source_gps_uids": list(range(1, 11)),
            "covered_gps_uids": list(range(1, 7)),
            "covered_gps_uid_capacity": 6,
            "route_horizon_complete": False,
        })
        rebased, refresh = plugin._rebase_rolling_snapshot(
            snapshot, tuple(range(5, 11)), sdk.get("nav_recalc_request"))
        self.assertIsNotNone(rebased)
        self.assertEqual(rebased["covered_gps_uids"], [5, 6])
        self.assertTrue(refresh)

    def test_rolling_rebase_rejects_target_or_authority_change(self):
        plugin, sdk, _point = build_map_plugin()
        snapshot = dict(sdk.get("lane_trajectory"))
        request = sdk.get("nav_recalc_request")

        changed_target, _refresh = plugin._rebase_rolling_snapshot(
            snapshot, (2, 99), request)
        self.assertIsNone(changed_target)

        changed_request, _refresh = plugin._rebase_rolling_snapshot(
            snapshot, (2, 3), "different-request")
        self.assertIsNone(changed_request)

        sdk.set("game_session_id", "different-session")
        changed_session, _refresh = plugin._rebase_rolling_snapshot(
            snapshot, (2, 3), request)
        self.assertIsNone(changed_session)

    def test_horizon_refresh_accepts_proven_lane_change_and_prefab_entry(self):
        plugin, sdk, _point = build_map_plugin()
        snapshot = sdk.get("lane_trajectory")
        previous = plugin._lane_path
        old_first, common_segment = previous.segments

        lane_ids = (
            LaneId(old_first.lane_id.road_uid, old_first.direction, 1),
            LaneId(old_first.lane_id.road_uid, old_first.direction, 0,
                   prefab_token="audit-prefab", connector_index=2,
                   connector_path=(2,)),
        )
        for lane_id in lane_ids:
            with self.subTest(lane_id=lane_id):
                entered = replace(
                    old_first, lane_id=lane_id,
                    lane_index=lane_id.lane_index)
                refreshed = replace(
                    previous, segments=(entered, common_segment))
                live_match = replace(plugin._lane_match, lane_id=lane_id)
                self.assertEqual(plugin._horizon_continuity_reason(
                    snapshot, previous, refreshed, (1, 2, 3),
                    live_match), "")

    def test_horizon_refresh_rejects_common_lane_without_shared_uid_edge(self):
        plugin, sdk, _point = build_map_plugin()
        snapshot = sdk.get("lane_trajectory")
        original = plugin._lane_path.segments[0]
        fake_common = replace(original, start_uid=90, end_uid=91)
        previous = replace(plugin._lane_path, segments=(fake_common,))
        refreshed = replace(plugin._lane_path, segments=(fake_common,))
        live_match = replace(plugin._lane_match, lane_id=fake_common.lane_id)
        reason = plugin._horizon_continuity_reason(
            snapshot, previous, refreshed, (1, 2, 3), live_match)
        self.assertIn("shared UID edge", reason)

    def test_100_km_gps_route_uses_ordered_rolling_runtime_horizon(self):
        plugin = MapPlugin.__new__(MapPlugin)
        uids = tuple(range(1, 5002))
        plugin.road_net = type("LongRoad", (), {
            "nodes": {uid: (0.0, (uid - 1) * 20.0) for uid in uids},
        })()
        selected = plugin._runtime_gps_window(uids)
        self.assertEqual(selected, uids[:len(selected)])
        self.assertLess(len(selected), len(uids))
        self.assertLessEqual(len(selected), RUNTIME_ROUTE_MAX_UIDS)
        covered = (selected[-1] - selected[0]) * 20.0
        self.assertGreaterEqual(covered, RUNTIME_ROUTE_HORIZON_M)
        self.assertEqual(uids[-1] * 20.0 // 1000, 100)

    def test_runtime_horizon_keeps_first_missing_uid_for_exact_failure(self):
        plugin = MapPlugin.__new__(MapPlugin)
        plugin.road_net = type("DamagedRoad", (), {
            "nodes": {1: (0.0, 0.0), 2: (0.0, 20.0)},
        })()
        self.assertEqual(plugin._runtime_gps_window((1, 2, 99, 100)),
                         (1, 2, 99))

    def test_one_snapshot_drives_controller_hud_ar_and_compatibility(self):
        plugin, sdk, _ = build_map_plugin()
        snapshot = sdk.get("lane_trajectory")
        self.assertTrue(snapshot["valid"], snapshot["failure_reason"])
        self.assertEqual(plugin._lane_route.world_points,
                         [tuple(point) for point in snapshot["points"]])
        self.assertEqual(snapshot["display_points"], snapshot["points"])
        self.assertEqual(snapshot["covered_gps_uids"], [1, 2, 3])
        self.assertTrue(snapshot["route_horizon_complete"])
        self.assertEqual(sdk.get("nav_path"), snapshot["display_points"])
        self.assertEqual(sdk.get("nav_trajectory_revision"), snapshot["revision"])

        hud = type("HUDReader", (), {
            "shared_state": sdk.shared_state,
            "_rear_cam_side": "off", "_rear_cam_until": 0.0,
        })()
        hud_data = UltraPilotHUD._read(hud)
        ar = type("ARReader", (), {"state": sdk.shared_state})()
        ar_revision, ar_points = AROverlay._current_display_points(ar)
        live_map_points = live_map_navigation_points(sdk.shared_state)
        self.assertIs(hud_data["nav_path"], snapshot["display_points"])
        self.assertIs(ar_points, snapshot["display_points"])
        self.assertIs(live_map_points, snapshot["display_points"])
        self.assertEqual(hud_data["lane_revision"], snapshot["revision"])
        self.assertEqual(ar_revision, snapshot["revision"])

        # The map controller is built from the same control samples, and the
        # autopilot acknowledges exactly that lane revision before accepting
        # its derived steering output.
        sdk.set("system_state", "CRUISE")
        sdk.set("danger_level", 0.0)
        sdk.set("traffic", [])
        sdk.set("autopilot_active", False)
        sdk.controller = Controller()
        sdk.telemetry = Telemetry()
        autopilot = AutopilotPlugin(sdk)
        autopilot.tags = Tags()
        autopilot.on_start()
        autopilot.on_tick(0.02)
        readiness = sdk.get("autopilot_navigation_readiness")
        self.assertTrue(readiness["ready"], readiness["reason"])
        self.assertEqual(readiness["source"], "gps_lane")
        self.assertEqual(readiness["revision"], snapshot["revision"])
        self.assertEqual(sdk.get("autopilot_lane_revision"), snapshot["revision"])

        publication = next(
            batch for batch in sdk.shared_state.batches
            if (batch.get("lane_trajectory") or {}).get("valid", False))
        self.assertEqual(publication["lane_trajectory_revision"],
                         snapshot["revision"])
        self.assertIs(publication["lane_trajectory"], snapshot)
        self.assertIs(publication["nav_path"], snapshot["display_points"])
        self.assertEqual(publication["nav_trajectory_revision"],
                         snapshot["revision"])
        self.assertFalse(publication["nav_active"])
        self.assertEqual(publication["nav_steering"], 0.0)

    def test_all_consumers_use_intent_and_revision_not_moving_uid_equality(self):
        plugin, sdk, _ = build_map_plugin()
        snapshot = sdk.get("lane_trajectory")
        sdk.set("game_route_node_uids", [2, 3])
        hud = type("HUDReader", (), {
            "shared_state": sdk.shared_state,
            "_rear_cam_side": "off", "_rear_cam_until": 0.0,
        })()
        ar = type("ARReader", (), {"state": sdk.shared_state})()

        self.assertIs(UltraPilotHUD._read(hud)["nav_path"],
                      snapshot["display_points"])
        self.assertIs(AROverlay._current_display_points(ar)[1],
                      snapshot["display_points"])
        self.assertIs(live_map_navigation_points(sdk.shared_state),
                      snapshot["display_points"])
        self.assertEqual(lane_authority_rejection_reason(
            sdk.shared_state, snapshot), "")

        sdk.set("navigation_intent_id", "different-intent")
        self.assertEqual(UltraPilotHUD._read(hud)["nav_path"], [])
        self.assertEqual(AROverlay._current_display_points(ar), (-1, []))
        self.assertEqual(live_map_navigation_points(sdk.shared_state), [])
        self.assertIn("different navigation intent",
                      lane_authority_rejection_reason(
                          sdk.shared_state, snapshot))

        sdk.set("navigation_intent_id", snapshot["navigation_intent_id"])
        sdk.set("lane_trajectory_revision", snapshot["revision"] + 1)
        self.assertEqual(UltraPilotHUD._read(hud)["nav_path"], [])
        self.assertEqual(AROverlay._current_display_points(ar), (-1, []))
        self.assertEqual(live_map_navigation_points(sdk.shared_state), [])
        self.assertIn("stale", lane_authority_rejection_reason(
            sdk.shared_state, snapshot))

    def test_destination_change_removes_old_revision_and_unproven_route(self):
        plugin, sdk, point = build_map_plugin()
        old_revision = sdk.get("lane_trajectory")["revision"]
        sdk.set("game_route_node_uids", [1, 99])
        plugin._update_lane_trajectory((point.x, point.z), point.heading)
        snapshot = sdk.get("lane_trajectory")
        self.assertGreater(snapshot["revision"], old_revision)
        self.assertFalse(snapshot["valid"])
        self.assertEqual(snapshot["points"], [])
        self.assertEqual(sdk.get("nav_path"), [])
        self.assertFalse(sdk.get("nav_active"))

    def test_transient_localization_miss_keeps_geometry_but_stops_control(self):
        plugin, sdk, point = build_map_plugin()
        original = sdk.get("lane_trajectory")
        original_revision = original["revision"]
        original_points = original["points"]
        original_build = original["route_build_id"]
        original_diagnostic = sdk.get("route_diagnostic_last_result")
        locator = plugin.road_net._runtime_lane_locator
        plugin.tags = Tags()
        sdk.set("truck_world_pos", (point.x, point.z))
        sdk.set("truck_heading", point.heading)
        sdk.set("truck_speed_ms", 10.0)
        sdk.set("telemetry_valid", True)

        with mock.patch.object(locator, "locate", return_value=None):
            plugin.on_tick(0.02)

        held = sdk.get("lane_trajectory")
        self.assertIs(held, original)
        self.assertEqual(held["revision"], original_revision)
        self.assertIs(held["points"], original_points)
        self.assertEqual(held["route_build_id"], original_build)
        self.assertIs(sdk.get("nav_path"), original["display_points"])
        self.assertEqual(sdk.get("nav_path"), original_points)
        self.assertFalse(sdk.get("lane_match")["valid"])
        self.assertFalse(sdk.get("nav_active"))
        self.assertEqual(sdk.get("nav_steering"), 0.0)
        self.assertTrue(sdk.get("navigation_unreliable"))
        loss_batch = next(
            batch for batch in reversed(sdk.shared_state.batches)
            if (batch.get("lane_match") or {}).get("valid") is False)
        self.assertFalse(loss_batch["nav_active"])
        self.assertEqual(loss_batch["nav_steering"], 0.0)
        hud = type("HUDReader", (), {
            "shared_state": sdk.shared_state,
            "_rear_cam_side": "off", "_rear_cam_until": 0.0,
        })()
        ar = type("ARReader", (), {"state": sdk.shared_state})()
        self.assertIs(UltraPilotHUD._read(hud)["nav_path"],
                      original["display_points"])
        self.assertIs(AROverlay._current_display_points(ar)[1],
                      original["display_points"])
        self.assertIs(live_map_navigation_points(sdk.shared_state),
                      original["display_points"])
        self.assertIs(sdk.get("route_diagnostic_last_result"),
                      original_diagnostic)

        plugin.on_tick(0.02)
        recovered = sdk.get("lane_trajectory")
        self.assertIs(recovered, original)
        self.assertEqual(recovered["revision"], original_revision)
        self.assertTrue(sdk.get("lane_match")["valid"])
        self.assertEqual(sdk.get("lane_match")["active_lane_id"],
                         original["active_lane_id"])
        self.assertFalse(sdk.get("navigation_unreliable"))
        self.assertEqual(sdk.get("navigation_failure_reason"), "")
        self.assertTrue(sdk.get("nav_active"))

    def test_repeated_localization_miss_revokes_control_not_route_identity(self):
        plugin, sdk, point = build_map_plugin()
        original_revision = sdk.get("lane_trajectory_revision")
        locator = plugin.road_net._runtime_lane_locator
        with mock.patch.object(locator, "locate", return_value=None):
            for _ in range(LANE_MATCH_GRACE_FRAMES):
                plugin._update_lane_trajectory(
                    (point.x, point.z), point.heading)
                self.assertTrue(sdk.get("lane_trajectory")["valid"])
                self.assertFalse(sdk.get("nav_active"))
                self.assertEqual(sdk.get("nav_steering"), 0.0)
            plugin._update_lane_trajectory((point.x, point.z), point.heading)

        held = sdk.get("lane_trajectory")
        self.assertTrue(held["valid"])
        self.assertEqual(held["revision"], original_revision)
        self.assertTrue(held["points"])
        self.assertFalse(sdk.get("lane_match")["valid"])
        self.assertFalse(sdk.get("nav_active"))
        self.assertEqual(sdk.get("nav_steering"), 0.0)

    def test_long_localization_miss_still_cannot_create_a_new_build(self):
        plugin, sdk, point = build_map_plugin()
        locator = plugin.road_net._runtime_lane_locator
        plugin._lane_loss_started_at = time.monotonic() - 1.0
        plugin._lane_loss_frames = 1
        with mock.patch.object(locator, "locate", return_value=None):
            plugin._update_lane_trajectory((point.x, point.z), point.heading)
        original = sdk.get("lane_trajectory")
        self.assertTrue(original["valid"])
        self.assertTrue(original["points"])
        self.assertFalse(sdk.get("nav_active"))
        self.assertEqual(sdk.get("nav_steering"), 0.0)

    def test_snapshot_hold_requires_same_session_map_and_dataset(self):
        mutations = (
            ("game_session_id", "new-session"),
            ("active_map_key", "another-map"),
            ("active_dataset_fingerprint", "another-fingerprint"),
        )
        for key, changed in mutations:
            with self.subTest(key=key):
                plugin, sdk, point = build_map_plugin()
                old = sdk.get("lane_trajectory")
                sdk.set(key, changed)
                plugin._update_lane_trajectory(
                    (point.x, point.z), point.heading)
                current = sdk.get("lane_trajectory")
                self.assertIsNot(current, old)
                self.assertFalse(current["valid"])
                self.assertEqual(current["points"], [])
                self.assertFalse(sdk.get("nav_active"))
                self.assertEqual(sdk.get("nav_steering"), 0.0)
                self.assertIsNone(plugin._lane_match)
                self.assertIsNone(
                    plugin.road_net._runtime_lane_locator.previous)

        plugin, sdk, point = build_map_plugin()
        changed_build = dict(sdk.get("lane_trajectory"))
        changed_build["route_build_id"] = "different-build"
        sdk.set("lane_trajectory", changed_build)
        plugin._update_lane_trajectory((point.x, point.z), point.heading)
        self.assertFalse(sdk.get("lane_trajectory")["valid"])
        self.assertEqual(sdk.get("lane_trajectory")["points"], [])
        self.assertIsNone(plugin._lane_match)

    def test_teleport_invalidates_before_fresh_localization(self):
        plugin, sdk, point = build_map_plugin()
        old_revision = sdk.get("lane_trajectory_revision")
        plugin._update_lane_trajectory((point.x, point.z + 60.0),
                                       point.heading)
        current = sdk.get("lane_trajectory")
        self.assertFalse(current["valid"])
        self.assertEqual(current["points"], [])
        self.assertGreater(current["revision"], old_revision)
        self.assertIn("discontinuously", current["failure_reason"])
        self.assertFalse(sdk.get("nav_active"))
        self.assertEqual(sdk.get("nav_steering"), 0.0)

    def test_wrong_direction_cannot_recover_held_snapshot(self):
        plugin, sdk, point = build_map_plugin()
        original = sdk.get("lane_trajectory")
        opposite = point.heading + math.pi
        for _ in range(LANE_MATCH_GRACE_FRAMES + 1):
            plugin._update_lane_trajectory((point.x, point.z), opposite)
        current = sdk.get("lane_trajectory")
        self.assertIs(current, original)
        self.assertTrue(current["valid"])
        self.assertFalse(sdk.get("lane_match")["valid"])
        self.assertFalse(sdk.get("nav_active"))

    def test_xyz_and_vertical_layers_are_preserved(self):
        plugin, sdk, _ = build_map_plugin(y=12.0)
        snapshot = sdk.get("lane_trajectory")
        self.assertTrue(all(len(point) == 3 for point in snapshot["points"]))
        self.assertTrue(all(abs(point[1] - 12.0) < 1e-6
                            for point in snapshot["points"]))
        route = Route(snapshot["points"])
        self.assertEqual(route.world_points[0][1], 12.0)
        self.assertEqual(route.points[0],
                         (snapshot["points"][0][0], snapshot["points"][0][2]))

    def test_legacy_recorded_route_cannot_override_live_gps_snapshot(self):
        plugin, sdk, point = build_map_plugin()
        snapshot = sdk.get("lane_trajectory")
        plugin.tags = Tags()
        plugin.active_route = Route([[100, 0, 0], [100, 0, 100]], "legacy")
        plugin.on_tick(0.02)
        self.assertEqual(sdk.get("nav_path"), snapshot["display_points"])
        self.assertEqual(sdk.get("nav_trajectory_revision"), snapshot["revision"])
        self.assertIsNone(plugin.active_route)
        self.assertEqual(sdk.get("navigation_source"), "gps_lane")

    def test_live_map_rejects_recorded_geometry_during_invalid_gps(self):
        now = time.monotonic()
        state = State({
            "game_gps_navigation_active": True,
            "game_route_distance": 900.0,
            "game_route_node_uids": [],
            "lane_trajectory_revision": 8,
            "lane_trajectory_heartbeat": now,
            "lane_trajectory": {
                "revision": 8, "valid": False,
                "failure_reason": "GPS topology is invalid",
                "source_gps_uids": [], "display_points": [],
            },
            "navigation_source": "recorded_route",
            "recorded_route_active": True,
            "nav_path": [[90.0, 0.0], [90.0, 100.0]],
        })
        self.assertEqual(live_map_navigation_points(state, now), [])

    def test_invalid_game_gps_without_uids_disarms_recorded_route(self):
        sdk = MapSDK({
            "truck_world_pos": (0.0, 0.0), "truck_heading": 0.0,
            "truck_speed_ms": 5.0, "truck_altitude": 0.0,
            "telemetry_valid": True,
            "game_gps_navigation_active": True,
            "game_route_distance": 1200.0,
            "game_route_node_uids": [],
            "lane_trajectory_revision": 4,
            "lane_trajectory": {
                "revision": 4, "valid": False,
                "failure_reason": "native route buffer is stale",
                "source_gps_uids": [], "points": [], "display_points": [],
            },
            "navigation_source": "recorded_route",
            "recorded_route_active": True,
            "nav_path": [[50.0, 0.0], [50.0, 100.0]],
            "nav_active": True, "nav_steering": 0.8,
        })
        plugin = MapPlugin(sdk)
        plugin.on_start()
        plugin.tags = Tags()
        plugin._net_attempted = True
        plugin.active_route = Route([(50.0, 0.0), (50.0, 100.0)], "legacy")

        plugin.on_tick(0.02)

        self.assertIsNone(plugin.active_route)
        self.assertFalse(sdk.get("recorded_route_active"))
        self.assertEqual(sdk.get("navigation_source"), "gps_lane")
        self.assertEqual(sdk.get("nav_path"), [])
        self.assertFalse(sdk.get("nav_active"))
        self.assertEqual(sdk.get("nav_steering"), 0.0)
        self.assertEqual(sdk.get("nav_trajectory_revision"), -1)

    def test_recorded_route_load_is_rejected_while_game_gps_exists(self):
        sdk = MapSDK({
            "nav_cmd": "load", "nav_arg": "legacy",
            "game_gps_navigation_active": True,
            "game_route_distance": 500.0,
            "game_route_node_uids": [],
        })
        plugin = MapPlugin(sdk)
        plugin.on_start()
        with mock.patch.object(Route, "load") as load:
            plugin._handle_command((0.0, 0.0))
        load.assert_not_called()
        self.assertIsNone(plugin.active_route)
        self.assertFalse(sdk.get("recorded_route_active"))
        result = sdk.get("nav_command_result")
        self.assertFalse(result["ok"])
        self.assertEqual(result["command"], "load")
        self.assertIn("game GPS", result["message"])
        self.assertEqual(rejected_navigation_command_message(sdk.shared_state),
                         result["message"])

    def test_recorded_route_runs_only_after_explicit_load_without_gps(self):
        sdk = MapSDK({
            "nav_cmd": "load", "nav_arg": "legacy",
            "truck_world_pos": (0.0, 0.0), "truck_heading": math.pi,
            "truck_speed_ms": 5.0, "truck_altitude": 0.0,
            "telemetry_valid": True,
            "game_gps_navigation_active": False,
            "game_route_distance": 0.0, "game_route_node_uids": [],
            "lane_trajectory_revision": 0,
        })
        plugin = MapPlugin(sdk)
        plugin.on_start()
        plugin.tags = Tags()
        plugin._net_attempted = True
        recorded = Route([(0.0, 0.0), (0.0, 100.0), (0.0, 200.0)], "legacy")
        with mock.patch.object(Route, "load", return_value=recorded) as load:
            plugin.on_tick(0.02)
        load.assert_called_once()
        self.assertIs(plugin.active_route, recorded)
        self.assertTrue(sdk.get("recorded_route_active"))
        self.assertEqual(sdk.get("navigation_source"), "recorded_route")
        self.assertTrue(sdk.get("nav_active"))
        self.assertGreaterEqual(len(sdk.get("nav_path")), 2)
        self.assertEqual(sdk.get("nav_trajectory_revision"), -1)
        publication = next(
            batch for batch in sdk.shared_state.batches
            if batch.get("navigation_source") == "recorded_route"
            and batch.get("nav_active") is True)
        self.assertEqual(publication["nav_path"], sdk.get("nav_path"))
        self.assertIn("nav_steering", publication)
        self.assertTrue(publication["recorded_route_active"])

        # Replay is a real, explicit no-GPS authority for the autopilot, not a
        # hidden fallback from an invalid GPS snapshot.
        sdk.set("system_state", "CRUISE")
        sdk.set("danger_level", 0.0)
        sdk.set("traffic", [])
        sdk.set("autopilot_active", True)
        sdk.controller = Controller()
        sdk.telemetry = Telemetry()
        autopilot = AutopilotPlugin(sdk)
        autopilot.tags = Tags()
        autopilot.on_start()
        autopilot.on_tick(0.05)
        readiness = sdk.get("autopilot_navigation_readiness")
        self.assertTrue(readiness["ready"], readiness["reason"])
        self.assertEqual(readiness["source"], "recorded_route")

    def test_recorded_route_does_not_resume_after_gps_is_removed(self):
        sdk = MapSDK({
            "truck_world_pos": (0.0, 0.0), "truck_heading": math.pi,
            "truck_speed_ms": 5.0, "truck_altitude": 0.0,
            "telemetry_valid": True,
            "game_gps_navigation_active": False,
            "game_route_distance": 0.0, "game_route_node_uids": [],
            "lane_trajectory_revision": 0,
            "navigation_source": "recorded_route",
            "recorded_route_active": True,
        })
        plugin = MapPlugin(sdk)
        plugin.on_start()
        plugin.tags = Tags()
        plugin._net_attempted = True
        plugin.active_route = Route(
            [(0.0, 0.0), (0.0, 100.0), (0.0, 200.0)], "legacy")
        sdk.set("navigation_source", "recorded_route")
        sdk.set("recorded_route_active", True)
        plugin.on_tick(0.02)
        self.assertTrue(sdk.get("nav_active"))

        sdk.set("game_gps_navigation_active", True)
        sdk.set("game_route_distance", 1000.0)
        plugin.on_tick(0.02)
        self.assertIsNone(plugin.active_route)

        sdk.set("game_gps_navigation_active", False)
        sdk.set("game_route_distance", 0.0)
        sdk.set("game_route_node_uids", [])
        sdk.set("lane_trajectory", {
            "revision": sdk.get("lane_trajectory_revision", 0) + 1,
            "valid": False, "failure_reason": "no GPS target",
            "source_gps_uids": [], "points": [], "display_points": [],
        })
        sdk.set("lane_trajectory_revision",
                sdk.get("lane_trajectory")["revision"])
        plugin.on_tick(0.02)
        self.assertIsNone(plugin.active_route)
        self.assertFalse(sdk.get("recorded_route_active"))
        self.assertFalse(sdk.get("nav_active"))
        self.assertEqual(sdk.get("nav_path"), [])

    def test_stop_atomically_clears_recorded_geometry_and_steering(self):
        sdk = MapSDK({
            "navigation_source": "none", "recorded_route_active": False,
            "nav_path": [], "nav_active": False, "nav_steering": 0.0,
        })
        plugin = MapPlugin(sdk)
        plugin.on_start()
        plugin.active_route = Route([(0.0, 0.0), (0.0, 100.0)], "legacy")
        sdk.shared_state.update_batch({
            "navigation_source": "recorded_route",
            "recorded_route_active": True,
            "nav_path": [[0.0, 0.0], [0.0, 100.0]],
            "nav_active": True, "nav_steering": 0.7,
            "nav_trajectory_revision": -1,
        })
        sdk.set("nav_cmd", "stop")

        plugin._handle_command((0.0, 0.0))

        self.assertIsNone(plugin.active_route)
        self.assertEqual(sdk.get("navigation_source"), "none")
        self.assertFalse(sdk.get("recorded_route_active"))
        self.assertEqual(sdk.get("nav_path"), [])
        self.assertFalse(sdk.get("nav_active"))
        self.assertEqual(sdk.get("nav_steering"), 0.0)
        cleared = next(
            batch for batch in reversed(sdk.shared_state.batches)
            if batch.get("recorded_route_active") is False
            and "nav_path" in batch)
        self.assertEqual(cleared["nav_path"], [])
        self.assertFalse(cleared["nav_active"])
        self.assertEqual(cleared["nav_steering"], 0.0)

    def test_switch_map_disarms_recorded_and_invalidates_gps_geometry(self):
        sdk = MapSDK({
            "game_gps_navigation_active": True,
            "game_route_node_uids": [1, 2], "lane_trajectory_revision": 3,
            "lane_trajectory": {
                "revision": 3, "valid": True,
                "source_gps_uids": [1, 2],
                "points": [[0.0, 0.0, 0.0], [0.0, 0.0, 10.0]],
                "display_points": [[0.0, 0.0, 0.0], [0.0, 0.0, 10.0]],
            },
            "navigation_source": "none", "recorded_route_active": False,
        })
        plugin = MapPlugin(sdk)
        plugin.on_start()
        plugin.active_route = Route([(0.0, 0.0), (0.0, 100.0)], "legacy")
        sdk.shared_state.update_batch({
            "navigation_source": "recorded_route",
            "recorded_route_active": True,
            "nav_path": [[0.0, 0.0], [0.0, 100.0]],
            "nav_active": True, "nav_steering": 0.4,
        })
        sdk.set("nav_cmd", "switch_map")
        sdk.set("nav_arg", "europe-1.60")

        plugin._handle_command((0.0, 0.0))

        snapshot = sdk.get("lane_trajectory")
        self.assertIsNone(plugin.active_route)
        self.assertFalse(snapshot["valid"])
        self.assertEqual(snapshot["points"], [])
        self.assertEqual(sdk.get("nav_path"), [])
        self.assertFalse(sdk.get("recorded_route_active"))
        self.assertFalse(sdk.get("nav_active"))
        self.assertEqual(sdk.get("nav_steering"), 0.0)

    def test_transient_telemetry_loss_preserves_gps_intent_but_stops_control(self):
        sdk = MapSDK({
            "truck_world_pos": (0.0, 0.0), "telemetry_valid": False,
            "game_gps_navigation_active": True,
            "game_route_node_uids": [1, 2],
            "lane_trajectory_revision": 5,
            "lane_trajectory": {
                "revision": 5, "valid": True, "source_gps_uids": [1, 2],
                "points": [[0.0, 0.0, 0.0], [0.0, 0.0, 10.0]],
                "display_points": [[0.0, 0.0, 0.0], [0.0, 0.0, 10.0]],
            },
            "navigation_source": "recorded_route",
            "recorded_route_active": True,
            "nav_path": [[50.0, 0.0], [50.0, 100.0]],
            "nav_active": True, "nav_steering": 0.8,
        })
        plugin = MapPlugin(sdk)
        plugin.on_start()
        plugin.tags = Tags()
        plugin.active_route = Route([(50.0, 0.0), (50.0, 100.0)], "legacy")

        plugin.on_tick(0.02)

        snapshot = sdk.get("lane_trajectory")
        self.assertTrue(sdk.get("game_gps_navigation_active"))
        self.assertEqual(sdk.get("game_route_node_uids"), [1, 2])
        self.assertFalse(sdk.get("recorded_route_active"))
        self.assertIsNone(plugin.active_route)
        self.assertTrue(snapshot["valid"])
        self.assertTrue(snapshot["points"])
        self.assertEqual(sdk.get("nav_path"), [])
        self.assertFalse(sdk.get("nav_active"))
        self.assertEqual(sdk.get("nav_steering"), 0.0)

    def test_new_authority_flags_default_fail_closed_for_older_state(self):
        sdk = MapSDK({})
        plugin = MapPlugin(sdk)
        plugin.on_start()
        self.assertFalse(plugin._game_gps_navigation_present())
        self.assertFalse(sdk.get("recorded_route_active"))
        self.assertEqual(live_map_navigation_points(sdk.shared_state), [])
        self.assertFalse(hasattr(MapView, "set_route"))

    def test_stale_or_invalid_snapshot_hides_hud_and_ar(self):
        plugin, sdk, _ = build_map_plugin()
        sdk.set("lane_trajectory_revision",
                sdk.get("lane_trajectory_revision") + 1)
        hud = type("HUDReader", (), {
            "shared_state": sdk.shared_state,
            "_rear_cam_side": "off", "_rear_cam_until": 0.0,
        })()
        ar = type("ARReader", (), {"state": sdk.shared_state})()
        self.assertEqual(UltraPilotHUD._read(hud)["nav_path"], [])
        self.assertEqual(AROverlay._current_display_points(ar), (-1, []))

    def test_low_confidence_and_invalid_route_brake_and_center_smoothly(self):
        for confidence, valid in ((0.50, True), (0.95, False)):
            state = State({
                "system_state": "CRUISE", "danger_level": 0.0,
                "lane_offset": 0.8, "traffic": [], "nav_active": True,
                "nav_steering": 0.7, "acc_throttle": 0.6,
                "acc_brake": 0.0, "autopilot_active": True,
                "game_route_distance": 100.0,
                "game_route_node_uids": [1, 2],
                "lane_trajectory_heartbeat": time.monotonic(),
                "lane_trajectory_revision": 7,
                "lane_trajectory": {
                    "revision": 7, "valid": valid, "confidence": confidence,
                    "source_gps_uids": [1, 2], "points": [[0, 0, 0], [0, 0, 10]],
                    "display_points": [[0, 0, 0], [0, 0, 10]],
                },
            })
            sdk = type("SDK", (), {})()
            sdk.shared_state, sdk.controller, sdk.telemetry = state, Controller(), Telemetry()
            plugin = AutopilotPlugin(sdk)
            plugin.tags = Tags()
            plugin.on_start()
            plugin._last_steering = 0.5
            plugin._last_throttle = 0.5
            plugin._engage_blend = 1.0
            plugin._was_active = True
            plugin.on_tick(0.1)
            self.assertGreater(plugin._last_steering, 0.0)
            self.assertLess(plugin._last_steering, 0.5)
            self.assertGreater(plugin._last_brake, 0.0)
            self.assertLess(plugin._last_brake, 0.70)
            self.assertEqual(state.get("autopilot_lane_revision"), -1)


if __name__ == "__main__":
    unittest.main()
