"""Regressions reconstructed from the 2026-09-06 06:57--07:02 game run."""

import inspect
import math
import threading
import time
import unittest

from core.engine import UltraPilotEngine
from core.lateral_controller import solve
from core.navigation.navigation_intent import (
    NavigationBufferClass,
    NavigationIntentTracker,
    classify_navigation_buffer,
)
from plugins.map.main import Plugin as MapPlugin


class State:
    def __init__(self, values=None):
        self.values = dict(values or {})

    def get(self, key, default=None):
        return self.values.get(key, default)

    def set(self, key, value):
        self.values[key] = value


def route_items(uids, terminal_distance=30.0):
    """Preserve the ordered UID evidence while emulating an SDK horizon."""
    return [{"uid": uid} for uid in uids]


class RealStraightSteeringReplayTests(unittest.TestCase):
    def test_old_yaw_response_is_not_extrapolated_as_future_yaw(self):
        # 07:01:48--51: same intent/revision, straight confirmed geometry,
        # CTE <= 6.1 cm and heading <= 0.4 deg.  Only the measured yaw
        # curvature changes sign; it belongs to earlier actuator commands.
        rows = (
            (0.061, math.radians(-0.1), -0.00181, 44.0, 0.0, -0.00003),
            (0.042, math.radians(-0.4), -0.00375, 49.0, 0.0, -0.00005),
            (0.012, math.radians(-0.3), +0.00480, 51.0, 0.0, +0.00001),
            (0.030, math.radians(-0.4), -0.00303, 51.0, 0.0, +0.00003),
        )
        commands = [solve(
            curvature, preview, cte, heading, speed_kmh / 3.6,
            vehicle_curvature_per_m=yaw_curvature,
            response_s=0.42,
            steering_lock_rad=0.78,
        )[0] for (cte, heading, yaw_curvature, speed_kmh,
                  curvature, preview) in rows]

        # A co-temporal spatial controller must react to the small lane errors,
        # not materially to the delayed physical yaw on confirmed straight
        # geometry.  The predictor's continuous observability weight tends to
        # zero here; it does not need a deadband or retained state.
        opposite_yaw = [solve(
            curvature, preview, cte, heading, speed_kmh / 3.6,
            vehicle_curvature_per_m=-yaw_curvature,
            response_s=0.42,
            steering_lock_rad=0.78,
        )[0] for (cte, heading, yaw_curvature, speed_kmh,
                  curvature, preview) in rows]
        for command, reversed_response in zip(commands, opposite_yaw):
            self.assertLess(abs(command-reversed_response), 0.00003)
        self.assertLess(max(abs(command) for command in commands), 0.01)


class RealRollingIntentReplayTests(unittest.TestCase):
    def test_terminal_uid_replacement_does_not_beat_proven_horizon_contraction(self):
        old_uids = list(range(1, 35))
        new_uids = list(range(1, 24)) + [900]
        decision = classify_navigation_buffer(
            route_items(old_uids), route_items(new_uids),
            old_destination=("terminal_uid", 34),
            new_destination=("terminal_uid", 900),
        )
        self.assertEqual(decision[0],
                         NavigationBufferClass.OVERLAPPING_CONTINUATION)
        self.assertEqual(decision[1], 23)

    def test_service_rolling_windows_keep_one_intent_until_disjoint_reroute(self):
        windows = (
            list(range(1, 35)),
            list(range(1, 24)) + [900],
            list(range(1, 21)),
            list(range(1, 15)),
            list(range(1, 9)),
            list(range(3, 9)),
            list(range(5, 9)),
        )
        tracker = NavigationIntentTracker()
        first = tracker.update(
            route_items(windows[0]), destination_present=True,
            destination=("terminal_uid", windows[0][-1]),
            context=(1, "promods-1.59", "dataset"))
        intent = first.intent_id
        changes = 1
        for window in windows[1:]:
            decision = tracker.update(
                route_items(window), destination_present=True,
                destination=("terminal_uid", window[-1]),
                context=(1, "promods-1.59", "dataset"))
            changes += int(decision.intent_changed)
            self.assertEqual(decision.intent_id, intent)
        self.assertEqual(changes, 1)

        reroute = tracker.update(
            route_items([700, 701, 702, 703]), destination_present=True,
            destination=("terminal_uid", 703),
            context=(1, "promods-1.59", "dataset"))
        self.assertEqual(reroute.classification,
                         NavigationBufferClass.TRUE_REROUTE)
        self.assertTrue(reroute.intent_changed)


class RealTrafficWaitReplayTests(unittest.TestCase):
    @staticmethod
    def engine(speed=10.0):
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = State({"truck_speed_ms": speed})
        return engine

    def test_perpendicular_vehicle_with_non_conflicting_arrival_is_not_a_lead(self):
        # Truck reaches z=20 in 2 s.  This car is already only 2 m lateral but
        # clears the corridor in 0.4 s; the old lane-strip test falsely braked.
        traffic = [{"x": 2.0, "z": 20.0, "yaw": math.pi / 2,
                    "speed": 5.0, "width": 2.0, "length": 4.5}]
        brake = self.engine()._lead_brake(
            traffic, (0.0, 0.0), math.pi)
        self.assertEqual(brake, 0.0)

    def test_perpendicular_vehicle_with_matching_conflict_time_is_braked_for(self):
        # Both trajectories reach the origin of the crossing in two seconds.
        traffic = [{"x": 10.0, "z": 20.0, "yaw": math.pi / 2,
                    "speed": 5.0, "width": 2.0, "length": 4.5}]
        brake = self.engine()._lead_brake(
            traffic, (0.0, 0.0), math.pi)
        self.assertGreater(brake, 0.0)


class ControlPublicationOrderingTests(unittest.TestCase):
    def test_synchronous_exploration_save_runs_after_control_publication(self):
        source = inspect.getsource(MapPlugin.on_tick)
        packet = source.rindex('"nav_steering_debug": steering_debug')
        save = source.rindex("self._schedule_map_exploration_save()")
        self.assertGreater(save, packet)

    def test_blocked_presentation_save_cannot_stop_fresh_lateral_packets(self):
        from tests.test_lane_authority_integration import build_map_plugin, Tags

        plugin, sdk, point = build_map_plugin()
        plugin.tags = Tags()
        sdk.set("truck_world_pos", (point.x, point.z))
        sdk.set("truck_heading", point.heading)
        sdk.set("truck_speed_ms", 0.0)
        sdk.set("vehicle_envelope_snapshot", {
            "timestamp": time.monotonic(), "sdk_frame_us": 1,
            "tractor_position": [point.x, point.y, point.z],
            "tractor_heading": point.heading, "tractor_speed_ms": 0.0,
            "yaw_rate_valid": False,
        })
        plugin._load_road_net = lambda: None
        plugin._update_lane_trajectory = lambda *_args: None
        plugin._publish_road_type = lambda *_args: None
        plugin._schedule_hud_road_scene = lambda *_args, **_kwargs: False
        plugin._schedule_live_map_scene = lambda *_args, **_kwargs: False
        plugin._exploration_dirty = True
        plugin._exploration_save_t = 30.0
        started = threading.Event()
        release = threading.Event()

        def blocked_save():
            started.set()
            release.wait(2.0)
            return True

        plugin._save_map_exploration = blocked_save
        try:
            begin = time.monotonic()
            plugin.on_tick(30.0)
            elapsed = time.monotonic() - begin
            self.assertTrue(started.wait(0.2))
            self.assertLess(elapsed, 0.1)
            first_packet_time = sdk.get("nav_steering_debug")["computed_at"]

            # A second control tick proceeds while persistence is still held.
            plugin.on_tick(0.05)
            second_packet_time = sdk.get("nav_steering_debug")["computed_at"]
            self.assertGreaterEqual(second_packet_time, first_packet_time)
            self.assertTrue(sdk.get("nav_active"))
        finally:
            release.set()


if __name__ == "__main__":
    unittest.main()
