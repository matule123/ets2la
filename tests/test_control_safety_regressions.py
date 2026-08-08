import os
import io
import math
import struct
import sys
import time
from types import SimpleNamespace
import unittest
from unittest import mock

from PyQt6.QtCore import QPointF

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from core.ar_overlay import _perspective_route_widths, _segment_is_occluded
from core.hud import (
    HUD_CAMERA_BACK_M, HUD_EGO_AHEAD_M, HUD_ROAD_BEHIND_M, UltraPilotHUD,
    _clip_truck_road_segment,
)
from core.engine import UltraPilotEngine
from core.controller import (
    Controller as PhysicalController, _discover_blinker_keys,
)
from core.navigation.route import (
    NORMALIZED_STEERING_ANGLE_RAD, TRUCK_WHEELBASE_M, Route, K_CTE,
    K_CTE_CURVE, curve_cte_gain, curve_speed_limit_ms,
)
from core.navigation.lane_model import LaneId, LaneMatch, LanePoint
from core.navigation.runtime_preflight import effective_lane_confidence
from core.sdk.scs_controller_writer import SCSControlsWriter, _FIELDS, _SIZE
from plugins.autopilot.main import (
    Plugin as AutopilotPlugin, _authority_reason_key,
    authority_retention_lateral_limit, engagement_lateral_limit,
    lane_authority_rejection_reason,
)
from plugins.lanecontrol.main import Plugin as LaneControlPlugin
from plugins.map.main import Plugin as MapPlugin
from sdk.plugin_sdk import (
    PluginSDK, _ControllerProxy, CTL_BRAKE, CTL_SELECT_DRIVE,
    CTL_STEERING, CTL_THROTTLE,
)


class State:
    def __init__(self, values=None):
        self.values = dict(values or {})

    def get(self, key, default=None):
        return self.values.get(key, default)

    def set(self, key, value):
        self.values[key] = value

    def update_batch(self, values):
        self.values.update(values)


class Controller:
    def __init__(self):
        self.steering = self.throttle = self.brake = 0.0
        self.drive_events = []
        self.blinker = "off"
        self.hazard = False

    def set_steering(self, value): self.steering = value
    def set_throttle(self, value): self.throttle = value
    def set_brake(self, value): self.brake = value
    def set_blinker(self, value): self.blinker = value
    def set_hazard(self, value): self.hazard = bool(value)
    def pay_toll(self): pass
    def select_drive(self, pressed=True):
        self.drive = pressed
        self.drive_events.append(pressed)
        return True
    def release_all(self):
        self.steering = self.throttle = self.brake = 0.0
        self.select_drive(False)


class Telemetry:
    def __init__(self, truck): self.truck = truck
    def get(self, key, default=None):
        return self.truck if key == "truck" else default


class Tags:
    pass


def autopilot(truck, state):
    plugin = AutopilotPlugin.__new__(AutopilotPlugin)
    plugin.sdk = type("SDK", (), {})()
    plugin.sdk.shared_state = state
    plugin.sdk.controller = Controller()
    plugin.sdk.telemetry = Telemetry(truck)
    plugin.tags = Tags()
    plugin.on_start()
    return plugin


def ready_navigation_state(**extra):
    now = time.monotonic()
    snapshot = {
        "revision": 7, "valid": True, "confidence": 0.95,
        "request_id": "test-request", "source_gps_uids": [11, 12],
        "points": [[0.0, 10.0, 0.0], [0.0, 10.0, -50.0]],
        "lane_match": {"revision": 7, "lateral_error_m": 0.0,
                       "heading_error_rad": 0.0},
    }
    values = {
        "autopilot_active": True, "lane_trajectory": snapshot,
        "lane_trajectory_revision": 7,
        "lane_trajectory_heartbeat": now,
        "game_route_node_uids": [11, 12],
        "nav_recalc_request": "test-request", "telemetry_valid": True,
        "lane_match": snapshot["lane_match"], "game_route_distance": 500.0,
    }
    values.update(extra)
    return State(values)


class ControlSafetyRegressionTests(unittest.TestCase):
    def test_live_same_revision_confidence_replaces_stale_build_locator_score(self):
        # Real revision 10: the immutable LanePath scored 0.93, but the build
        # happened while the rolling prefix charged the locator an off-route
        # penalty and froze snapshot confidence at 0.676.  Once live
        # localisation was centred, that stale number blocked N for 53 s.
        state = ready_navigation_state(autopilot_active=False)
        snapshot = state.values["lane_trajectory"]
        snapshot.update({
            "confidence": 0.676422763479866,
            "confidence_components": {
                "trajectory": 0.93,
                "locator": 0.676422763479866,
            },
        })
        state.set("lane_match", {
            "revision": 7, "valid": True, "confidence": 0.94,
            "lateral_error_m": 0.542, "heading_error_rad": 0.014,
        })
        self.assertAlmostEqual(
            effective_lane_confidence(snapshot, state.get("lane_match")),
            0.93)
        self.assertEqual(lane_authority_rejection_reason(state, snapshot), "")

        # The threshold is not weakened: a genuinely weak current match stays
        # fail-closed even though the route geometry itself is excellent.
        state.get("lane_match")["confidence"] = 0.70
        self.assertIn("confidence", lane_authority_rejection_reason(
            state, snapshot))

    def test_real_065_match_keeps_proven_lane_identity_but_not_legacy_score(self):
        # Real 22:25:34 sample: confidence 0.6506 was produced by adding the
        # current 1.807 m CTE, 15.4 degree heading error and a rolling-prefix
        # off-route charge. Those same displacement values are independently
        # checked below; they must not also erase the already proven LaneId.
        state = ready_navigation_state()
        snapshot = state.get("lane_trajectory")
        snapshot["confidence_components"] = {
            "trajectory": 0.95, "locator": 0.777856,
        }
        live = {
            "revision": 7, "valid": True,
            "confidence": 0.6506, "authority_confidence": 0.9666666667,
            "lateral_error_m": 1.807, "heading_error_rad": math.radians(15.4),
            "lane_width_m": 4.5,
        }
        state.set("lane_match", live)
        self.assertAlmostEqual(
            effective_lane_confidence(snapshot, live), 0.95)
        self.assertEqual(lane_authority_rejection_reason(state, snapshot), "")

        # A legacy producer has no separate identity proof and therefore
        # remains fail-closed at the unchanged 0.72 authority threshold.
        live.pop("authority_confidence")
        self.assertIn("confidence", lane_authority_rejection_reason(
            state, snapshot))

    def test_map_match_publishes_identity_confidence_without_hiding_tracking(self):
        lane_id = LaneId(101, 1, 0)
        match = LaneMatch(
            lane_id, LanePoint(1.0, 12.0, 3.0, lane_id=lane_id), 0, 0,
            1.807, 0.0, math.radians(15.4), 6.289, 0.6506,
            "same_lane", (("lateral", 1.807), ("heading", 1.882),
                          ("vertical", 0.0), ("off_route", 2.0),
                          ("derived_width", 0.6)))
        plugin = MapPlugin.__new__(MapPlugin)
        plugin.road_net = SimpleNamespace(_lane_id_index={
            lane_id: SimpleNamespace(width_m=4.5, elevation_layer=0),
        })
        payload = plugin._lane_match_payload(match, 7)
        self.assertAlmostEqual(payload["confidence"], 0.6506)
        self.assertAlmostEqual(payload["authority_confidence"], 0.9666666667)
        self.assertAlmostEqual(payload["lateral_error_m"], 1.807)
        self.assertAlmostEqual(payload["heading_error_rad"],
                               math.radians(15.4))

    def test_real_start_match_engages_once_and_stays_enabled(self):
        # Runtime capture 2026-07-29 09:13:06: lateral=1.1742428 m and
        # heading error=0.0127 rad. Engine said enabled, then the plugin's old
        # hidden 1.10 m gate disabled it on the next process tick.
        state = ready_navigation_state(autopilot_active=False)
        state.values["lane_trajectory"]["confidence"] = 0.7853809672165706
        plugin = autopilot({"speed": 15.0, "gear": 3}, state)
        live_match = {
            "revision": 7,
            "lateral_error_m": 1.174242800891218,
            "heading_error_rad": 0.0126999698872159,
            "lane_width_m": 4.5,
        }
        state.set("lane_match", live_match)
        plugin.on_tick(0.05)
        self.assertTrue(state.get("autopilot_navigation_readiness")["ready"])

        state.set("autopilot_active", True)
        plugin.on_tick(0.05)
        self.assertTrue(state.get("autopilot_active"))
        self.assertTrue(plugin._lane_lock_acquired)

    def test_unsafe_start_is_rejected_before_engine_reports_enabled(self):
        state = ready_navigation_state(autopilot_active=False)
        state.values["lane_trajectory"]["confidence"] = 0.90
        plugin = autopilot({"speed": 15.0, "gear": 3}, state)
        state.set("lane_match", {
            "revision": 7,
            "lateral_error_m": 1.60,
            "heading_error_rad": 0.0,
            "lane_width_m": 4.5,
        })
        plugin.on_tick(0.05)
        readiness = state.get("autopilot_navigation_readiness")
        self.assertFalse(readiness["ready"])
        self.assertIn("not centred", readiness["reason"])

    def test_invalid_live_match_reports_localisation_not_sentinel_distance(self):
        state = ready_navigation_state()
        state.set("lane_match", {
            "revision": 7, "valid": False,
            "lateral_error_m": 1_000_000.0,
            "heading_error_rad": math.pi,
            "failure_reason": "live lane localization unavailable",
        })
        reason = lane_authority_rejection_reason(
            state, state.get("lane_trajectory"))
        self.assertEqual(reason, "live lane localization unavailable")
        self.assertNotIn("1000000", reason)

    def test_automatic_safety_disengagement_logs_exact_reason_once(self):
        state = ready_navigation_state()
        state.set("lane_match", {
            "revision": 7, "valid": False,
            "lateral_error_m": 1_000_000.0,
            "heading_error_rad": math.pi,
            "failure_reason": "live lane localization unavailable",
        })
        plugin = autopilot({"speed": 0.0, "gear": 1}, state)
        with self.assertLogs(level="WARNING") as captured:
            plugin.on_tick(0.05)
        self.assertFalse(state.get("autopilot_active"))
        self.assertEqual(state.get("autopilot_disable_reason"),
                         "live lane localization unavailable")
        self.assertTrue(state.get("safety_hazard_active"))
        self.assertEqual(sum("automatically disengaged" in line
                             for line in captured.output), 1)
        self.assertNotIn("1000000.00", "\n".join(captured.output))
        event = state.get("autopilot_log_event")
        self.assertEqual(event["level"], "WARNING")
        self.assertEqual(
            event["message"],
            "Autopilot automatically disabled: live lane localization unavailable")

    def test_engagement_gate_is_derived_from_lane_width(self):
        self.assertAlmostEqual(engagement_lateral_limit({"lane_width_m": 4.5}),
                               1.50)
        self.assertAlmostEqual(engagement_lateral_limit({"lane_width_m": 3.0}),
                               1.00)
        self.assertAlmostEqual(engagement_lateral_limit({}), 1.10)
        self.assertAlmostEqual(engagement_lateral_limit(
            {"lane_width_m": float("nan")}), 1.10)

    def test_continued_authority_matches_locator_same_lane_retention(self):
        live = {"lane_width_m": 4.5}
        self.assertAlmostEqual(
            authority_retention_lateral_limit(live), 3.375)
        state = ready_navigation_state()
        state.set("lane_match", {
            "revision": 7, "valid": True, "lateral_error_m": 2.93,
            "heading_error_rad": 0.02, "lane_width_m": 4.5,
        })
        self.assertEqual(lane_authority_rejection_reason(
            state, state.get("lane_trajectory")), "")
        state.get("lane_match")["lateral_error_m"] = 3.38
        self.assertIn("outside the confirmed GPS lane",
                      lane_authority_rejection_reason(
                          state, state.get("lane_trajectory")))

    def test_confirmed_curve_drift_keeps_correction_but_not_reengagement(self):
        state = ready_navigation_state(
            nav_active=True, nav_steering=-0.32,
            path_curvature_radius=80.0, path_curve_distance_m=0.0)
        state.set("lane_match", {
            "revision": 7, "valid": True, "lateral_error_m": 2.93,
            "heading_error_rad": 0.02, "lane_width_m": 4.5,
        })
        plugin = autopilot({"speed": 10.0, "gear": 5}, state)
        plugin._lane_lock_acquired = True
        plugin._engage_blend = 1.0
        plugin._was_active = True
        plugin.on_tick(0.10)
        self.assertTrue(state.get("autopilot_active"))
        self.assertTrue(state.get("nav_active"))
        self.assertLess(plugin.sdk.controller.steering, 0.0)

        # The same displacement is never sufficient for a fresh activation.
        state.set("autopilot_active", False)
        plugin.on_tick(0.10)
        self.assertFalse(
            state.get("autopilot_navigation_readiness")["ready"])

    def test_enabled_message_waits_for_control_initialization(self):
        state = ready_navigation_state(
            autopilot_active=False, nav_active=True, nav_steering=0.1)
        state.get("lane_match")["lane_width_m"] = 4.5
        state.set("autopilot_navigation_readiness", {
            "ready": True, "reason": "", "timestamp": time.monotonic(),
        })
        state.set("autopilot_command", {"seq": 71, "enabled": True})
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = state
        engine.controller = Controller()
        engine._last_autopilot_command = None
        engine._process_autopilot_command()
        self.assertTrue(state.get("autopilot_active"))
        self.assertNotEqual(state.get("autopilot_engagement_confirmed"), 71)
        self.assertNotEqual(state.get("tts_message"), "Autopilot enabled.")

        plugin = autopilot({"speed": 10.0, "gear": 3}, state)
        plugin.on_tick(0.05)
        self.assertEqual(state.get("autopilot_engagement_confirmed"), 71)
        self.assertEqual(state.get("tts_message"), "Autopilot enabled.")

    def test_neighbour_lane_opposite_heading_and_wrong_deck_fail_closed(self):
        state = ready_navigation_state()
        snapshot = state.get("lane_trajectory")
        lane_a = {"road_uid": 4, "direction": 1, "lane_index": 0}
        lane_b = {"road_uid": 4, "direction": 1, "lane_index": 1}
        snapshot["active_lane_id"] = lane_a
        snapshot["lane_match"].update({
            "active_lane_id": lane_a, "elevation_layer": 0,
        })
        live = dict(snapshot["lane_match"])
        live.update({"revision": 7, "active_lane_id": lane_b,
                     "elevation_layer": 0})
        state.set("lane_match", live)
        self.assertIn("different GPS lane", lane_authority_rejection_reason(
            state, snapshot))

        live["active_lane_id"] = lane_a
        live["heading_error_rad"] = math.radians(179.0)
        self.assertIn("heading differs", lane_authority_rejection_reason(
            state, snapshot))

        live["heading_error_rad"] = 0.0
        live["elevation_layer"] = 3
        self.assertIn("elevation layer", lane_authority_rejection_reason(
            state, snapshot))

    def test_steering_unwinds_at_same_smooth_rate_as_curve_acquisition(self):
        plugin = autopilot({"speed": 10.0, "gear": 3}, State())
        plugin._last_steering = 0.40
        released = plugin._ramp_steering(0.0, 0.10)
        release_step = 0.40 - released
        plugin._last_steering = 0.0
        applied = plugin._ramp_steering(0.40, 0.10)
        self.assertAlmostEqual(release_step, applied, places=7)
        self.assertGreater(released, 0.0)  # still continuous, never snaps

    def test_steering_reversal_passes_smoothly_through_zero(self):
        plugin = autopilot({"speed": 10.0, "gear": 3}, State())
        plugin._last_steering = 0.03
        first = plugin._ramp_steering(-0.40, 0.10)
        self.assertLess(first, 0.03)
        self.assertGreaterEqual(first, -0.03)

    def test_navigation_filter_rejects_one_frame_opposite_lock(self):
        plugin = autopilot({"speed": 10.0, "gear": 3}, State())
        first = plugin._smooth_navigation_steering(0.40, 0.05, 10)
        opposite_spike = plugin._smooth_navigation_steering(-0.40, 0.05, 10)
        self.assertEqual(first, 0.40)
        self.assertGreater(opposite_spike, 0.0)
        # A persistent genuine direction change still crosses zero promptly.
        settled = [plugin._smooth_navigation_steering(-0.40, 0.05, 10)
                   for _ in range(12)]
        self.assertLess(settled[-1], -0.35)

    def test_navigation_filter_never_blends_stale_revision(self):
        plugin = autopilot({"speed": 10.0, "gear": 3}, State())
        plugin._smooth_navigation_steering(0.55, 0.05, 10)
        changed = plugin._smooth_navigation_steering(-0.25, 0.05, 11)
        self.assertEqual(changed, -0.25)

    def test_scs_writer_layout_matches_shipped_controller_dll(self):
        offsets, total = {}, 0
        for name, field_type in _FIELDS:
            offsets[name] = total
            total += _SIZE[field_type]
        self.assertEqual(total, 342)
        self.assertEqual(offsets["steering"], 118)
        self.assertEqual(offsets["aforward"], 122)
        self.assertEqual(offsets["abackward"], 126)
        self.assertEqual(offsets["geardrive"], 268)

        dll_path = os.path.join(ROOT, "assets", "scs_sdk_controller.dll")
        with open(dll_path, "rb") as stream:
            dll = stream.read()
        self.assertIn(b"Local\\SCSControls", dll)
        for name, _field_type in _FIELDS:
            self.assertIn(("ETS2LA " + name).encode("ascii"), dll)

        writer = SCSControlsWriter.__new__(SCSControlsWriter)
        writer.connected = True
        writer.invert_steering = False
        writer._buf = io.BytesIO(bytes(total))
        writer._offsets = offsets
        writer._retry = 0
        writer.set_steering(0.25)
        writer.set_throttle(0.40)
        writer.set_brake(0.15)
        writer.select_drive()
        writer.set_right_blinker(True)
        writer.set_hazard(True)
        payload = writer._buf.getvalue()
        self.assertAlmostEqual(struct.unpack_from(
            "f", payload, offsets["steering"])[0], 0.25)
        self.assertAlmostEqual(struct.unpack_from(
            "f", payload, offsets["aforward"])[0], 0.40)
        self.assertAlmostEqual(struct.unpack_from(
            "f", payload, offsets["abackward"])[0], 0.15)
        self.assertTrue(struct.unpack_from(
            "?", payload, offsets["geardrive"])[0])
        self.assertTrue(struct.unpack_from(
            "?", payload, offsets["rblinker"])[0])
        self.assertTrue(struct.unpack_from(
            "?", payload, offsets["flasher4way"])[0])

    def test_plugin_controller_proxy_supports_drive_selector(self):
        state = {}
        proxy = _ControllerProxy(state)
        self.assertTrue(proxy.select_drive(True))
        self.assertIs(state[CTL_SELECT_DRIVE], True)
        self.assertTrue(proxy.select_drive(False))
        self.assertIs(state[CTL_SELECT_DRIVE], False)

    def test_lanecontrol_accepts_authoritative_xyz_path(self):
        plugin = LaneControlPlugin.__new__(LaneControlPlugin)
        plugin.sdk = type("SDK", (), {})()
        plugin.sdk.shared_state = State({
            "nav_path": [(0.0, 12.0, 0.0), (8.0, 12.0, -35.0)],
            "truck_world_pos": (0.0, 0.0), "truck_heading": 0.0,
        })
        # Regression for "too many values to unpack (expected 2)".
        self.assertIsInstance(plugin._route_lateral_hint(), (float, type(None)))

    def test_reverse_gear_stops_then_disengages_without_selecting_drive(self):
        state = ready_navigation_state()
        truck = {"speed": -0.5, "gear": -1}
        plugin = autopilot(truck, state)
        plugin.on_tick(0.05)
        self.assertTrue(state.get("autopilot_active"))
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        truck["speed"] = 0.0
        plugin.sdk.telemetry.truck = truck
        plugin._drive_request_t = -1.0
        plugin.on_tick(0.05)
        self.assertFalse(state.get("autopilot_active"))
        self.assertFalse(plugin._reverse_recovery)
        self.assertEqual(plugin.sdk.controller.drive_events, [False])

    def test_neutral_selects_drive_before_throttle(self):
        state = ready_navigation_state()
        plugin = autopilot({"speed": 0.0, "gear": 0}, state)
        plugin.on_tick(0.05)
        self.assertTrue(state.get("autopilot_active"))
        self.assertTrue(plugin.sdk.controller.drive)
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        self.assertEqual(plugin.sdk.controller.brake, 0.0)

        # The plugin must not overwrite this event with False before the
        # slower engine process has consumed it. The engine owns the physical
        # release half of the momentary pulse.
        plugin.on_tick(0.05)
        self.assertEqual(plugin.sdk.controller.drive_events, [True])

    def test_automatic_drive_mode_with_zero_ratio_does_not_deadlock(self):
        state = ready_navigation_state()
        plugin = autopilot({"speed": 0.0, "gear": 0}, state)
        plugin.on_tick(0.05)
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        self.assertEqual(plugin.tags.throttle, 0.0)

        # ETS2 can show selector D while its current-ratio telemetry remains
        # zero until throttle is applied. After the bounded settling time the
        # fallback cruise ramp must be allowed to engage first gear.
        plugin._drive_engage_started = time.monotonic() - 1.0
        plugin._drive_request_t = time.monotonic() - 1.0
        plugin.on_tick(0.10)
        self.assertTrue(state.get("autopilot_active"))
        self.assertGreater(plugin.sdk.controller.throttle, 0.0)
        self.assertGreater(plugin.tags.throttle, 0.0)
        self.assertEqual(plugin.sdk.controller.brake, 0.0)
        self.assertEqual(state.get("navigation_status"),
                         "Jazda dopredu pripravená")

    def test_red_light_brake_suppresses_throttle_on_valid_route(self):
        state = ready_navigation_state(
            nav_active=True, nav_steering=0.0, light_brake=1.0,
            acc_throttle=0.8, acc_brake=0.0)
        plugin = autopilot({"speed": 8.0, "gear": 5}, state)
        plugin.on_tick(0.10)
        self.assertGreater(plugin.sdk.controller.brake, 0.0)
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        self.assertTrue(state.get("autopilot_active"))

    def test_queue_brake_keeps_fresh_gps_steering_and_authority(self):
        state = ready_navigation_state(
            system_state="CRUISE", nav_active=True, nav_steering=0.30,
            traffic_brake=1.0, acc_throttle=0.8, acc_brake=0.0)
        plugin = autopilot({"speed": 8.0, "gear": 5}, state)
        plugin._engage_blend = 1.0
        plugin._was_active = True
        plugin.on_tick(0.10)
        self.assertGreater(plugin.sdk.controller.brake, 0.0)
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        self.assertGreater(plugin.sdk.controller.steering, 0.0)
        self.assertTrue(state.get("autopilot_active"))

    def test_emergency_queue_stop_does_not_latch_old_curve_steering(self):
        state = ready_navigation_state(
            system_state="EMERGENCY", nav_active=True, nav_steering=-0.25)
        plugin = autopilot({"speed": 5.0, "gear": 3}, state)
        plugin._engage_blend = 1.0
        plugin._was_active = True
        plugin._last_steering = 0.40
        outputs = []
        for _ in range(8):
            plugin.on_tick(0.10)
            outputs.append(plugin.sdk.controller.steering)
        self.assertLess(outputs[0], 0.40)
        self.assertLess(outputs[-1], 0.0)
        self.assertGreater(plugin.sdk.controller.brake, 0.0)
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        self.assertTrue(state.get("autopilot_active"))

    def test_validated_gps_steering_is_not_clipped_again_by_engine(self):
        state = ready_navigation_state(
            navigation_source="gps_lane", nav_active=True,
            autopilot_lane_revision=7, truck_speed_ms=25.0,
            autopilot_control_heartbeat=time.monotonic(),
            ctl_steering=0.70, ctl_throttle=0.0, ctl_brake=0.0)
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = state
        engine.controller = Controller()
        engine._was_active = True
        engine._drive_selector_pressed = False
        engine._flush_controls()
        self.assertAlmostEqual(engine.controller.steering, 0.70)

        # The legacy/vision path retains its high-speed safety clamp.
        state.set("navigation_source", "none")
        engine._flush_controls()
        self.assertLess(engine.controller.steering, 0.70)

        # Merely claiming GPS ownership is insufficient: a mixed/stale
        # revision must retain the conservative clamp.
        state.set("navigation_source", "gps_lane")
        state.set("autopilot_lane_revision", 6)
        engine._flush_controls()
        self.assertLess(engine.controller.steering, 0.70)

    def test_engine_turns_coalesced_drive_requests_into_real_pulses(self):
        state = State({
            "autopilot_active": True,
            "autopilot_control_heartbeat": time.monotonic(),
            "telemetry_valid": True,
            CTL_STEERING: 0.0, CTL_THROTTLE: 0.0, CTL_BRAKE: 0.0,
            CTL_SELECT_DRIVE: True,
        })
        controller = Controller()
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = state
        engine.controller = controller
        engine._was_active = False
        engine._last_output_steering = 0.0
        engine._last_output_brake = 0.0
        engine._last_control_flush = time.monotonic()
        engine._drive_selector_pressed = False

        engine._flush_controls()
        # Simulate a 100 Hz plugin publishing True again before the slower
        # engine frame. The engine must release first instead of holding it.
        state.set(CTL_SELECT_DRIVE, True)
        engine._flush_controls()
        state.set(CTL_SELECT_DRIVE, True)
        engine._flush_controls()
        self.assertEqual(controller.drive_events, [True, False, True])

    def test_route_blinker_survives_legacy_planner_frames(self):
        state = State({
            "autopilot_active": True, "telemetry_valid": True,
            "autopilot_control_heartbeat": time.monotonic(),
            CTL_STEERING: 0.0, CTL_THROTTLE: 0.0, CTL_BRAKE: 0.0,
            "route_blinker": "right", "planner_blinker": "off",
        })
        controller = Controller()
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = state
        engine.controller = controller
        engine._was_active = False
        engine._drive_selector_pressed = False
        engine._last_output_steering = 0.0
        engine._last_output_brake = 0.0
        engine._last_control_flush = time.monotonic()
        engine._flush_controls()
        engine._flush_controls()
        self.assertEqual(controller.blinker, "right")
        self.assertEqual(state.get("active_blinker"), "right")

        state.set("autopilot_active", False)
        engine._flush_controls()
        self.assertEqual(controller.blinker, "off")
        self.assertEqual(state.get("active_blinker"), "off")

    def test_drive_request_survives_worker_engine_scheduling_race(self):
        shared = ready_navigation_state().values
        shared["telemetry"] = {"truck": {"speed": 0.0, "gear": 0}}
        sdk = PluginSDK(shared, "autopilot")
        plugin = AutopilotPlugin(sdk)
        plugin.on_start()
        plugin.on_tick(0.01)
        plugin.on_tick(0.01)
        # Two fast worker ticks happened before the engine got CPU time. The
        # selector request must still be pending instead of being overwritten.
        self.assertIs(shared.get(CTL_SELECT_DRIVE), True)

        controller = Controller()
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = sdk.shared_state
        engine.controller = controller
        engine._was_active = False
        engine._last_output_steering = 0.0
        engine._last_output_brake = 0.0
        engine._last_control_flush = time.monotonic()
        engine._drive_selector_pressed = False
        engine._flush_controls()
        engine._flush_controls()
        self.assertEqual(controller.drive_events, [True, False])
        self.assertIsNone(shared.get(CTL_SELECT_DRIVE))

    def test_master_release_also_releases_drive_selector(self):
        class FakeSCS:
            def __init__(self): self.drive_released = False
            def set_steering(self, _value): pass
            def set_throttle(self, _value): pass
            def set_brake(self, _value): pass
            def release_drive(self): self.drive_released = True

        controller = PhysicalController.__new__(PhysicalController)
        controller.mode = "SCS_SDK"
        controller.scs = FakeSCS()
        controller.release_all()
        self.assertTrue(controller.scs.drive_released)

    def test_scs_blinker_emits_one_frame_press_and_release(self):
        class FakeSCS:
            def __init__(self): self.events = []
            def set_left_blinker(self, value):
                self.events.append(("left", value))
            def set_right_blinker(self, value):
                self.events.append(("right", value))

        controller = PhysicalController.__new__(PhysicalController)
        controller.mode = "SCS_SDK"
        controller.scs = FakeSCS()
        controller.current_blinker = "off"
        controller._scs_blinker_button = None
        with mock.patch("core.controller._HAS_PDI", False):
            controller.set_blinker("right")
            controller.set_blinker("right")
        self.assertEqual(controller.scs.events,
                         [("right", True), ("right", False)])
        with mock.patch("core.controller._HAS_PDI", False):
            controller.set_blinker("off")
            controller.set_blinker("off")
        self.assertEqual(controller.scs.events[-2:],
                         [("right", True), ("right", False)])

    def test_scs_hazard_emits_one_toggle_edge_and_following_release(self):
        class FakeSCS:
            def __init__(self): self.events = []
            def set_hazard(self, value): self.events.append(bool(value))

        controller = PhysicalController.__new__(PhysicalController)
        controller.mode = "SCS_SDK"
        controller.scs = FakeSCS()
        controller.current_hazard = False
        controller._scs_hazard_button = False
        controller._blinker_keys = {}
        with mock.patch("core.controller._HAS_PDI", False):
            controller.set_hazard(True)
            controller.set_hazard(True)
        self.assertEqual(controller.scs.events, [True, False])
        self.assertTrue(controller.current_hazard)

    def test_safety_hazard_overrides_route_indicator_in_engine_flush(self):
        state = ready_navigation_state(
            nav_active=True, safety_hazard_active=True,
            autopilot_control_heartbeat=time.monotonic(),
            ctl_steering=0.0, ctl_throttle=0.0, ctl_brake=0.4,
            route_blinker="right")
        controller = Controller()
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = state
        engine.controller = controller
        engine._was_active = True
        engine._drive_selector_pressed = False
        engine._last_output_steering = 0.0
        engine._last_output_brake = 0.0
        engine._last_control_flush = time.monotonic()
        engine._flush_controls()
        self.assertTrue(controller.hazard)
        self.assertEqual(controller.blinker, "off")

    def test_scs_steering_is_not_disabled_when_keyboard_backend_exists(self):
        class FakeSCS:
            def __init__(self): self.steering = None
            def set_steering(self, value): self.steering = value

        controller = PhysicalController.__new__(PhysicalController)
        controller.mode = "SCS_SDK"
        controller.scs = FakeSCS()
        with mock.patch("core.controller._HAS_PDI", True):
            controller.set_steering(0.375)
        self.assertAlmostEqual(controller.scs.steering, 0.375)

    def test_active_profile_blinker_keys_are_read_from_controls(self):
        profile_log = io.StringIO(
            "Set profile finished: 'my wheel profile'\n")
        controls = io.StringIO(
            ' "mix lblinker `keyboard.x?0 | semantical.lblinker?0`"\n'
            ' "mix rblinker `keyboard.c?0 | semantical.rblinker?0`"\n')
        with (mock.patch("core.controller.os.path.isfile", return_value=True),
              mock.patch("builtins.open",
                         side_effect=[profile_log, controls])):
            self.assertEqual(_discover_blinker_keys("X:\\ETS2"),
                             {"left": "x", "right": "c"})

    def test_scs_mode_uses_profile_key_for_indicator_when_available(self):
        controller = PhysicalController.__new__(PhysicalController)
        controller.mode = "SCS_SDK"
        controller.current_blinker = "off"
        controller._blinker_keys = {"left": "x", "right": "c"}
        controller._scs_blinker_button = None
        with (mock.patch("core.controller._HAS_PDI", True),
              mock.patch("core.controller.pydirectinput.press") as press):
            controller.set_blinker("right")
            controller.set_blinker("right")
            controller.set_blinker("off")
        self.assertEqual(press.call_args_list,
                         [mock.call("c"), mock.call("c")])

    def test_autopilot_without_valid_route_never_selects_drive(self):
        state = State({"autopilot_active": True})
        plugin = autopilot({"speed": 0.0, "gear": 0}, state)
        plugin.on_tick(0.05)
        self.assertFalse(state.get("autopilot_active"))
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        self.assertEqual(plugin.sdk.controller.drive_events, [False])

    def test_engine_rejects_enable_command_without_fresh_navigation(self):
        state = State({
            "autopilot_active": False,
            "autopilot_command": {"seq": 4, "enabled": True},
        })
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = state
        engine.controller = Controller()
        engine._last_autopilot_command = None
        engine._process_autopilot_command()
        self.assertFalse(state.get("autopilot_active"))
        self.assertIn("trajektória", state.get("autopilot_disable_reason"))

    def test_engine_accepts_enable_only_with_fresh_navigation_authority(self):
        state = State({
            "autopilot_active": False,
            "autopilot_command": {"seq": 5, "enabled": True},
            "autopilot_navigation_readiness": {
                "ready": True, "reason": "", "timestamp": time.monotonic(),
            },
        })
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = state
        engine.controller = Controller()
        engine._last_autopilot_command = None
        engine._process_autopilot_command()
        self.assertTrue(state.get("autopilot_active"))

    def test_navigation_stop_does_not_turn_off_master_autopilot(self):
        state = State({
            "nav_cmd": "stop", "autopilot_active": True,
            "nav_active": True, "nav_steering": 0.4,
            "path_curvature_radius": 55.0,
            "path_curve_distance_m": 18.0,
        })
        plugin = MapPlugin.__new__(MapPlugin)
        plugin.sdk = type("MapSDK", (), {
            "get": lambda _self, key, default=None: state.get(key, default),
            "set": lambda _self, key, value: state.set(key, value),
        })()
        plugin.active_route = object()
        plugin._handle_command(None)
        self.assertTrue(state.get("autopilot_active"))
        self.assertFalse(state.get("nav_active"))
        self.assertEqual(state.get("nav_steering"), 0.0)
        self.assertIsNone(state.get("path_curvature_radius"))
        self.assertIsNone(state.get("path_curve_distance_m"))

    def test_arrival_stops_and_disengages(self):
        for gear in (1, 0, -1):
            with self.subTest(gear=gear):
                state = State({
                    "autopilot_active": True,
                    "navigation_arrival_pending": True,
                    "game_route_distance": 3.0,
                })
                plugin = autopilot({"speed": 0.1, "gear": gear}, state)
                plugin.on_tick(0.05)
                self.assertFalse(state.get("autopilot_active"))
                self.assertFalse(state.get("nav_active"))
                self.assertEqual(state.get("nav_steering"), 0.0)
                self.assertEqual(plugin.sdk.controller.throttle, 0.0)
                self.assertEqual(plugin.sdk.controller.brake, 0.0)
                # Arrival only releases the momentary selector. It must never
                # request D, including when telemetry already reports R.
                self.assertEqual(plugin.sdk.controller.drive_events, [False])
                self.assertEqual(state.get("navigation_status"), "Cieľ dosiahnutý")

    def test_curve_cross_track_gain_holds_lane_without_straight_hunting(self):
        self.assertAlmostEqual(curve_cte_gain(1e6), K_CTE)
        self.assertAlmostEqual(curve_cte_gain(60.0), K_CTE_CURVE)
        self.assertGreater(curve_cte_gain(200.0), curve_cte_gain(400.0))
        self.assertGreater(curve_cte_gain(400.0), K_CTE)
        self.assertAlmostEqual(curve_cte_gain(60.0, 0.0), K_CTE_CURVE)
        self.assertAlmostEqual(curve_cte_gain(60.0, 1.50), K_CTE_CURVE)
        # A large error on a straight must not manufacture curve authority.
        self.assertAlmostEqual(curve_cte_gain(1e6, 2.0), K_CTE)

    def test_sharp_junction_radius_is_not_discarded_by_speed_envelope(self):
        sharp_apex = curve_speed_limit_ms(20.0, 0.0)
        sharp_approach = curve_speed_limit_ms(20.0, 45.0)
        gentle_apex = curve_speed_limit_ms(120.0, 0.0)
        self.assertLess(sharp_apex, sharp_approach)
        self.assertLess(sharp_approach, gentle_apex)
        self.assertLess(sharp_apex * 3.6, 25.0)

    def test_autopilot_brakes_for_sub_thirty_metre_confirmed_curve(self):
        state = ready_navigation_state(
            nav_active=True, nav_steering=0.15, acc_throttle=1.0,
            acc_brake=0.0, path_curvature_radius=20.0,
            path_curve_distance_m=0.0)
        plugin = autopilot({"speed": 12.0, "gear": 6}, state)
        plugin._engage_blend = 1.0
        plugin._was_active = True
        plugin.on_tick(0.10)
        self.assertGreater(plugin.sdk.controller.brake, 0.0)
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        self.assertLess(state.get("path_curve_speed_limit_ms"), 7.0)

    def test_captured_eighteen_metre_roundabout_reserves_steering_setup(self):
        # At 22:26:05 the compact curve was 44 m ahead but the old point-mass
        # envelope still allowed about 47 km/h. A tractor-trailer needs a
        # bounded setup distance before the steering apex, without changing
        # the validated curve radius or the steering lookahead itself.
        state = ready_navigation_state(
            nav_active=True, nav_steering=0.35, acc_throttle=1.0,
            acc_brake=0.0, path_curvature_radius=18.0,
            path_curve_distance_m=44.0)
        plugin = autopilot({"speed": 12.0, "gear": 6}, state)
        plugin._engage_blend = 1.0
        plugin._was_active = True
        plugin.on_tick(0.10)
        expected = curve_speed_limit_ms(18.0, 24.0, 1.8)
        self.assertAlmostEqual(
            state.get("path_curve_speed_limit_ms"), expected, places=6)
        self.assertGreater(plugin.sdk.controller.brake, 0.0)
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)

    def test_live_distance_reason_logs_as_one_stable_category(self):
        self.assertEqual(_authority_reason_key(
            "truck is 1.87 m outside the confirmed GPS lane"),
            _authority_reason_key(
            "truck is 3.00 m outside the confirmed GPS lane"))

    def test_curvature_uses_path_projection_not_off_centre_truck(self):
        straight = Route([(0.0, 0.0), (0.0, 100.0), (0.0, 200.0)])
        # A 1.5 m CTE is still a straight road, not a fictitious bend whose
        # gain changes as the truck approaches the centreline.
        self.assertGreater(straight.curvature_ahead(
            (1.5, 20.0), 3.141592653589793), 100000.0)

    def test_ar_width_is_compact_original_size(self):
        snapshot = {
            "viewport": {"width": 1920}, "fov_horizontal_deg": 75.0,
        }
        halo, core = _perspective_route_widths(50.0, snapshot)
        self.assertGreater(core, 4.0)
        self.assertLess(core, 8.0)
        self.assertGreater(halo, core)

    def test_nearer_vehicle_occludes_route_segment(self):
        rects = [(40.0, 40.0, 60.0, 60.0, 10.0)]
        self.assertTrue(_segment_is_occluded(
            QPointF(45.0, 50.0), QPointF(55.0, 50.0), 20.0, rects))
        self.assertFalse(_segment_is_occluded(
            QPointF(45.0, 50.0), QPointF(55.0, 50.0), 5.0, rects))

    def test_hud_ego_and_road_share_the_telemetry_origin(self):
        self.assertEqual(HUD_EGO_AHEAD_M, 0.0)
        self.assertGreater(HUD_CAMERA_BACK_M, 40.0)
        hud = UltraPilotHUD.__new__(UltraPilotHUD)
        hud._view_yaw = 0.0

        class View:
            def height(self): return 500.0
            def top(self): return 0.0
            def center(self): return QPointF(400.0, 250.0)

        road_origin = UltraPilotHUD._project(hud, 0.0, 0.0, View())
        ego_origin = UltraPilotHUD._project(
            hud, HUD_EGO_AHEAD_M, 0.0, View())
        self.assertEqual(road_origin, ego_origin)

    def test_hud_road_continues_behind_complete_tractor_trailer(self):
        self.assertGreaterEqual(HUD_ROAD_BEHIND_M, 40.0)
        clipped = _clip_truck_road_segment((-60.0, 0.0), (-10.0, 0.0))
        self.assertIsNotNone(clipped)
        first, second, _t0, _t1 = clipped
        self.assertAlmostEqual(first[0], -HUD_ROAD_BEHIND_M)
        self.assertEqual(second, (-10.0, 0.0))

if __name__ == "__main__":
    unittest.main()
