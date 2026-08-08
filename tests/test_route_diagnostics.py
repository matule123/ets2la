import json
import os
import unittest
import copy
from unittest import mock

from core.navigation.lane_model import LaneLocator
from core.navigation.lane_trajectory import build_lane_trajectory
from core.navigation.route_diagnostics import (
    FAILURE_CODES, RouteBuildDiagnostics, anonymize_failure_record,
    classify_failure, export_anonymized_failure,
)
from tests.test_lane_authority_integration import State, build_map_plugin
from tests.test_lane_locator import FakeNetwork, lane
from tests.test_lane_route_builder import SyntheticMap
from UI.map_page import MapPage


class RouteDiagnosticFormatTests(unittest.TestCase):
    def test_every_stable_failure_code_has_a_deterministic_classifier_case(self):
        cases = {
            "DATASET_MISSING_PREFAB": (
                "resolve_gps_corridor",
                "missing prefab description blkw_1401i for GPS UID pair 1 -> 2",
                {"prefab_token": "blkw_1401i"}),
            "DATASET_VERSION_MISMATCH": (
                "dataset_version", "dataset version differs from game", {}),
            "DATASET_MISSING_UID": (
                "resolve_gps_corridor", "GPS UID 42 is absent from the active map", {}),
            "LOCALIZATION_NO_MATCH": (
                "LaneLocator", "no matching lane", {"outcome": "no_match"}),
            "LOCALIZATION_AMBIGUOUS": (
                "LaneLocator", "candidate tie", {"outcome": "ambiguous"}),
            "TOPOLOGY_NO_CONNECTION": (
                "select_lane_sequence", "no LaneConnection between lanes", {}),
            "TOPOLOGY_AMBIGUOUS": (
                "select_lane_sequence", "prefab exit is ambiguous", {}),
            "GEOMETRY_GAP": (
                "connect_lane_sequence", "confirmed boundary has 9.0 m gap", {}),
            "GEOMETRY_HEADING_JUMP": (
                "validate_lane_trajectory", "heading jump is 80 degrees", {}),
            "GEOMETRY_ELEVATION_JUMP": (
                "validate_lane_trajectory", "height jump is 8 metres", {}),
            "TRAJECTORY_VALIDATION_FAILED": (
                "validate_lane_trajectory", "trajectory has a self-intersection", {}),
            "STALE_REVISION": (
                "stale_revision", "GPS revision changed", {}),
            "INTERNAL_ERROR": (
                "route_build", "unexpected exception", {}),
        }
        self.assertEqual(set(cases), set(FAILURE_CODES))
        for expected, (phase, reason, details) in cases.items():
            with self.subTest(expected):
                self.assertEqual(classify_failure(phase, reason, details), expected)

    def test_failed_record_has_complete_versioned_shape(self):
        diagnostic = RouteBuildDiagnostics(
            17, (10, 20), (100.0, 5.0, 200.0), 1.25,
            {
                "active_map_key": "promods-1.59",
                "active_map_name": "ProMods",
                "game_version": "1.59",
                "dataset_fingerprint": "abc123",
            }, route_build_id="build-format")
        diagnostic.start_phase("LaneLocator")
        diagnostic.fail_phase("LaneLocator", "no matching lane", {
            "outcome": "no_match",
            "gps_uid": 20,
            "gps_uid_index": 1,
            "road_token": "road.secret",
            "prefab_token": "prefab.secret",
            "lane_id_before": {"road_uid": 999, "lane_index": 0},
            "lane_id_after": None,
            "lane_heading_rad": 1.0,
            "lane_heading_deg": 57.3,
            "elevation_difference_m": 2.0,
            "geometry": {
                "gap_m": 7.0, "heading_jump_deg": 45.0,
                "elevation_jump_m": 2.0,
            },
            "planned_lane_connection": {
                "type": "prefab", "source": {"road_uid": 999},
                "target": {"road_uid": 1000},
            },
        })
        record = diagnostic.finish("failed")
        self.assertEqual(record["schema_version"], 1)
        self.assertEqual(record["route_build_id"], "build-format")
        self.assertEqual(record["revision"], 17)
        self.assertEqual(record["status"], "failed")
        self.assertGreaterEqual(record["duration_ms"], 0.0)
        self.assertEqual(record["failure"]["code"], "LOCALIZATION_NO_MATCH")
        context = record["context"]
        for key in (
                "gps_uids", "gps_uid", "gps_uid_index", "road_token",
                "prefab_token", "lane_id_before", "lane_id_after", "world",
                "truck_heading_rad", "lane_heading_rad",
                "elevation_difference_m", "candidate_lanes",
                "planned_lane_connection", "geometry", "confidence",
                "environment"):
            self.assertIn(key, context)
        self.assertEqual(record["phases"][0]["name"], "LaneLocator")
        self.assertEqual(record["phases"][0]["status"], "failed")

    def test_anonymized_export_removes_ids_tokens_text_and_absolute_world(self):
        diagnostic = RouteBuildDiagnostics(
            5, (987654321, 123456789), (12345.67, 89.0, -76543.21), 0.5,
            route_build_id="export-one")
        diagnostic.record["context"].update({
            "gps_uid": 987654321,
            "road_token": "road.private.token",
            "prefab_token": "prefab.private.token",
            "lane_id_before": {"road_uid": 987654321, "lane_index": 1},
            "candidate_lanes": [{
                "lane_id": {"road_uid": 123456789, "lane_index": 0},
                "nearest_world": {"x": 12347.67, "y": 90.0, "z": -76540.21},
                "nested": {
                    "unknown_uid": 444444444,
                    "unknown_token": "nested.secret.token",
                    "arbitrary_position": {
                        "x": 12346.67, "y": 91.0, "z": -76539.21,
                    },
                    "note": "LaneId(987654321, road.private.token)",
                    "position_note": "truck was at 12345.67 / -76543.21",
                },
            }],
        })
        diagnostic.fail_phase(
            "connect_lane_sequence",
            "UID 987654321 road.private.token has a geometry gap")
        record = diagnostic.finish("failed")
        anonymized = anonymize_failure_record(record)
        encoded = json.dumps(anonymized, sort_keys=True)
        self.assertEqual(anonymized["route_build_id"], "export-one")
        self.assertEqual(anonymized["started_at"], record["started_at"])
        self.assertEqual(
            anonymized["context"]["environment"]["dataset_fingerprint"],
            record["context"]["environment"]["dataset_fingerprint"])
        self.assertNotIn("987654321", encoded)
        self.assertNotIn("123456789", encoded)
        self.assertNotIn("444444444", encoded)
        self.assertNotIn("nested.secret.token", encoded)
        self.assertNotIn("road.private.token", encoded)
        self.assertNotIn("prefab.private.token", encoded)
        self.assertNotIn("12345.67", encoded)
        self.assertNotIn("12346.67", encoded)
        self.assertNotIn("LaneId(", encoded)
        self.assertNotIn("truck was at", encoded)
        self.assertEqual(
            anonymized["context"]["candidate_lanes"][0]["nearest_world"],
            {"relative_x": 2.0, "relative_y": 1.0, "relative_z": 3.0})
        self.assertEqual(
            anonymized["context"]["candidate_lanes"][0]["nested"][
                "arbitrary_position"],
            {"relative_x": 1.0, "relative_y": 2.0, "relative_z": 4.0})
        writer = mock.mock_open()
        with (mock.patch("os.makedirs"),
              mock.patch("builtins.open", writer),
              mock.patch("os.fsync") as fsync,
              mock.patch("os.replace") as replace):
            path = export_anonymized_failure(record, r"C:\diagnostics")
        self.assertEqual(path,
                         r"C:\diagnostics\route-failure-export-one.json")
        temporary = writer.call_args.args[0]
        self.assertTrue(temporary.startswith(path + ".tmp-"))
        writer.assert_called_once_with(temporary, "x", encoding="utf-8")
        fsync.assert_called_once()
        replace.assert_called_once_with(temporary, path)

    def test_failed_atomic_export_preserves_record_identity_and_cleans_temp(self):
        diagnostic = RouteBuildDiagnostics(
            22, (10, 20), (1.0, 2.0, 3.0), 0.4,
            route_build_id="stable-build")
        diagnostic.fail_phase("LaneLocator", "no match")
        record = diagnostic.finish("failed")
        original = copy.deepcopy(record)
        writer = mock.mock_open()
        with (mock.patch("os.makedirs"),
              mock.patch("builtins.open", writer),
              mock.patch("os.fsync"),
              mock.patch("os.replace", side_effect=OSError("disk")),
              mock.patch("os.remove") as remove):
            with self.assertRaises(OSError):
                export_anonymized_failure(record, r"C:\diagnostics")
        remove.assert_called_once_with(writer.call_args.args[0])
        self.assertEqual(record, original)
        self.assertEqual(record["route_build_id"], "stable-build")
        self.assertEqual(record["revision"], 22)

    def test_broken_logging_handler_is_nonfatal(self):
        with mock.patch("logging.log", side_effect=RuntimeError("handler")):
            diagnostic = RouteBuildDiagnostics(
                7, (1, 2), (0.0, 0.0, 0.0), 0.0,
                route_build_id="logging-safe")
            diagnostic.start_phase("LaneLocator")
            diagnostic.fail_phase("LaneLocator", "no match")
            record = diagnostic.finish("failed")
        self.assertEqual(record["route_build_id"], "logging-safe")
        self.assertEqual(record["revision"], 7)


class RouteDiagnosticPipelineTests(unittest.TestCase):
    class ThrowingDiagnostic:
        record = {"failure": {"code": None}}
        revision = 71
        build_id = "throwing-observer"

        def __getattr__(self, _name):
            def fail(*_args, **_kwargs):
                raise RuntimeError("observer failed")
            return fail

    def test_diagnostic_locator_call_preserves_hysteresis_and_network_caches(self):
        network = FakeNetwork([lane(1, 0.0), lane(1, 2.0, lane_index=1)])
        locator = LaneLocator(network)
        previous = locator.locate((0.0, 0.0, 15.0), 3.141592653589793,
                                  (10, 11))
        self.assertIsNotNone(previous)
        before = dict(locator.__dict__)
        capture = {}
        observed = locator.locate(
            (50.0, 0.0, 15.0), 3.141592653589793, (10, 11), previous,
            diagnostics=capture, diagnostic_mode=True)
        self.assertIsNone(observed)
        self.assertEqual(locator.__dict__, before)
        self.assertIs(locator.previous, previous)

        synthetic = SyntheticMap()
        synthetic.node(1, 0, 0)
        synthetic.node(2, 0, 40)
        segment = synthetic.road(1, 2, 2)
        source = synthetic.net._build_lane_segments(segment)[0]
        point = source.centerline[2]
        real_locator = LaneLocator(synthetic.net)
        match = real_locator.locate(
            (point.x, point.y, point.z), point.heading, (1, 2))
        lane_cache = dict(synthetic.net._lane_cache)
        lane_index = dict(synthetic.net._lane_id_index)
        capture = {}
        real_locator.locate(
            (point.x, point.y, point.z), point.heading, (1, 2), match,
            diagnostics=capture, diagnostic_mode=True)
        self.assertIs(real_locator.previous, match)
        self.assertEqual(synthetic.net._lane_cache, lane_cache)
        self.assertEqual(synthetic.net._lane_id_index, lane_index)

    def test_throwing_callbacks_do_not_change_lane_or_trajectory_result(self):
        synthetic = SyntheticMap()
        synthetic.node(1, 0, 0)
        synthetic.node(2, 0, 40)
        synthetic.node(3, 0, 80)
        segment = synthetic.road(1, 2, 2)
        synthetic.road(2, 3, 2)
        source = synthetic.net._build_lane_segments(segment)[0]
        point = source.centerline[2]
        match = LaneLocator(synthetic.net).locate(
            (point.x, point.y, point.z), point.heading, (1, 2, 3))
        baseline, _ = synthetic.net.build_lane_path(
            (1, 2, 3), (point.x, point.z), point.heading,
            altitude=point.y, start_match=match)
        observed, returned_match = synthetic.net.build_lane_path(
            (1, 2, 3), (point.x, point.z), point.heading,
            altitude=point.y, start_match=match,
            diagnostics=self.ThrowingDiagnostic())
        self.assertEqual(returned_match, match)
        self.assertEqual(observed.valid, baseline.valid)
        self.assertEqual(observed.failure_reason, baseline.failure_reason)
        self.assertEqual(
            [(p.x, p.y, p.z) for p in observed.points],
            [(p.x, p.y, p.z) for p in baseline.points])

        baseline_trajectory = build_lane_trajectory(baseline)
        observed_trajectory = build_lane_trajectory(
            baseline, diagnostics=self.ThrowingDiagnostic())
        self.assertEqual(observed_trajectory.valid, baseline_trajectory.valid)
        self.assertEqual(observed_trajectory.failure_reason,
                         baseline_trajectory.failure_reason)
        self.assertEqual(
            [(p.x, p.y, p.z) for p in observed_trajectory.points],
            [(p.x, p.y, p.z) for p in baseline_trajectory.points])

    def test_route_diagnostic_creation_does_not_reserve_revision(self):
        plugin, sdk, point = build_map_plugin()
        before_internal = plugin._lane_revision
        before_shared = sdk.get("lane_trajectory_revision")
        diagnostic = plugin._new_route_diagnostics(
            (1, 2, 3), (point.x, point.z), point.y, point.heading)
        self.assertEqual(plugin._lane_revision, before_internal)
        self.assertEqual(sdk.get("lane_trajectory_revision"), before_shared)
        self.assertEqual(diagnostic.revision, before_shared + 1)

    def test_throwing_diagnostic_still_publishes_safe_invalid_snapshot(self):
        plugin, sdk, _ = build_map_plugin()
        diagnostic = self.ThrowingDiagnostic()
        snapshot = plugin._fail_route_build(
            diagnostic, "unexpected observer-side failure", (1, 2, 3))
        self.assertFalse(snapshot["valid"])
        self.assertEqual(snapshot["revision"], diagnostic.revision)
        self.assertEqual(sdk.get("lane_trajectory"), snapshot)
        self.assertEqual(sdk.get("nav_path"), [])
        self.assertFalse(sdk.get("nav_active"))
        self.assertEqual(sdk.get("nav_steering"), 0.0)

    def test_throwing_route_diagnostic_callbacks_do_not_change_valid_publish(self):
        plugin, sdk, point = build_map_plugin()
        expected_points = copy.deepcopy(sdk.get("lane_trajectory")["points"])
        plugin._publish_invalid_lane_trajectory(
            "force one audited rebuild", (1, 2, 3), log_failure=False)
        plugin._lane_failure_signature = None
        callbacks = (
            "start_phase", "finish_phase", "fail_phase", "observe_locator",
            "observe_lane_path", "observe_validation", "finish",
        )
        patches = [mock.patch.object(
            RouteBuildDiagnostics, name,
            side_effect=RuntimeError("observer failed")) for name in callbacks]
        for patcher in patches:
            patcher.start()
            self.addCleanup(patcher.stop)
        plugin._update_lane_trajectory((point.x, point.z), point.heading)
        snapshot = sdk.get("lane_trajectory")
        self.assertTrue(snapshot["valid"], snapshot["failure_reason"])
        self.assertEqual(snapshot["points"], expected_points)
        self.assertEqual(sdk.get("nav_path"), expected_points)
        self.assertEqual(plugin._lane_route.world_points,
                         [tuple(item) for item in expected_points])

    def test_map_page_requests_export_for_exact_last_failed_build(self):
        state = State({
            "route_diagnostic_last_result": {
                "route_build_id": "failed-build-7", "status": "failed",
            },
        })
        status = mock.Mock()
        page = type("DiagnosticPage", (), {"state": state, "status": status})()
        MapPage.save_route_diagnostic(page)
        self.assertEqual(state.get("route_diagnostic_export_request"),
                         "failed-build-7")
        self.assertIsNone(state.get("route_diagnostic_export_result"))
        status.setText.assert_called_once()

    def test_successful_snapshot_carries_one_build_id_revision_and_all_phases(self):
        plugin, sdk, point = build_map_plugin()
        snapshot = sdk.get("lane_trajectory")
        result = sdk.get("route_diagnostic_last_result")
        self.assertTrue(snapshot["valid"])
        self.assertTrue(snapshot["route_build_id"])
        self.assertEqual(result["route_build_id"], snapshot["route_build_id"])
        self.assertEqual(result["revision"], snapshot["revision"])
        self.assertEqual(result["status"], "success")
        self.assertEqual(
            [phase["name"] for phase in result["phases"]],
            ["LaneLocator", "resolve_gps_corridor", "select_lane_sequence",
             "connect_lane_sequence", "LanePath", "build_lane_trajectory",
             "validate_lane_trajectory", "publish_snapshot"])

        build_id = snapshot["route_build_id"]
        revision = snapshot["revision"]
        locator = plugin.road_net._runtime_lane_locator
        with mock.patch.object(locator, "locate", wraps=locator.locate) as locate:
            plugin._update_lane_trajectory((point.x, point.z), point.heading)
        self.assertIsNone(locate.call_args.kwargs["diagnostics"])
        self.assertEqual(sdk.get("lane_trajectory")["route_build_id"], build_id)
        self.assertEqual(sdk.get("lane_trajectory_revision"), revision)
        self.assertEqual(sdk.get("route_diagnostic_last_result")[
            "route_build_id"], build_id)

    def test_failure_keeps_technical_detail_out_of_island_and_captures_candidates(self):
        plugin, sdk, point = build_map_plugin()
        sdk.set("game_route_node_uids", [1, 999999])
        plugin._update_lane_trajectory((point.x, point.z), point.heading)
        snapshot = sdk.get("lane_trajectory")
        result = sdk.get("route_diagnostic_last_result")
        event = sdk.get("navigation_log_event")
        self.assertFalse(snapshot["valid"])
        self.assertEqual(snapshot["failure_code"], "DATASET_MISSING_UID")
        self.assertEqual(result["failure_code"], "DATASET_MISSING_UID")
        self.assertNotIn("999999", event["message"])
        self.assertNotIn("GPS UID", event["message"])
        record = plugin._last_failed_route_diagnostic
        self.assertEqual(record["context"]["gps_uid"], 999999)
        self.assertEqual(record["context"]["gps_uid_index"], 1)
        self.assertTrue(record["context"]["candidate_lanes"])
        self.assertTrue(all("distance_m" in candidate
                            for candidate in record["context"]["candidate_lanes"]))

    def test_export_request_has_unambiguous_result_for_exact_failed_build(self):
        plugin, sdk, point = build_map_plugin()
        sdk.set("game_route_node_uids", [1, 999999])
        plugin._update_lane_trajectory((point.x, point.z), point.heading)
        build_id = plugin._last_failed_route_diagnostic["route_build_id"]
        sdk.set("route_diagnostic_export_request", build_id)
        with mock.patch(
                "plugins.map.main.export_anonymized_failure",
                return_value=r"C:\logs\route-failure.json") as export:
            plugin._handle_diagnostic_export()
        export.assert_called_once()
        result = sdk.get("route_diagnostic_export_result")
        self.assertTrue(result["ok"])
        self.assertEqual(result["route_build_id"], build_id)
        self.assertIsNone(sdk.get("route_diagnostic_export_request"))

    def test_export_and_logging_exceptions_do_not_change_navigation_state(self):
        plugin, sdk, point = build_map_plugin()
        before_snapshot = copy.deepcopy(sdk.get("lane_trajectory"))
        before_revision = sdk.get("lane_trajectory_revision")
        plugin._last_failed_route_diagnostic = {
            "route_build_id": "failed-export", "revision": 123,
            "status": "failed", "failure": {"code": "INTERNAL_ERROR"},
        }
        sdk.set("route_diagnostic_export_request", "failed-export")
        with (mock.patch(
                "plugins.map.main.export_anonymized_failure",
                side_effect=OSError("disk full")),
              mock.patch("logging.exception",
                         side_effect=RuntimeError("broken handler"))):
            plugin._handle_diagnostic_export()
        result = sdk.get("route_diagnostic_export_result")
        self.assertFalse(result["ok"])
        self.assertEqual(result["route_build_id"], "failed-export")
        self.assertEqual(sdk.get("lane_trajectory"), before_snapshot)
        self.assertEqual(sdk.get("lane_trajectory_revision"), before_revision)

        # A completely broken optional handler is isolated at the tick boundary.
        with (mock.patch.object(
                plugin, "_handle_diagnostic_export",
                side_effect=RuntimeError("unexpected export defect")),
              mock.patch.object(plugin, "_load_road_net"),
              mock.patch.object(plugin, "_update_lane_trajectory") as update,
              mock.patch.object(plugin, "_handle_command"),
              mock.patch.object(plugin, "_publish_road_type")):
            plugin.tags = type("Tags", (), {})()
            sdk.set("truck_world_pos", (point.x, point.z))
            sdk.set("truck_heading", point.heading)
            sdk.set("telemetry_valid", True)
            plugin.on_tick(0.01)
        update.assert_called_once_with((point.x, point.z), point.heading)


if __name__ == "__main__":
    unittest.main()
