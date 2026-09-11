"""Phase 4D regressions for deterministic lateral-control timing."""

import json
import threading
import time
import unittest
from unittest import mock

from core.control_timing import (
    FrameGate, MonotonicSequenceGate, wait_for_next_tick,
)
from core.engine import UltraPilotEngine
from core.steering_dynamics import SteeringDynamics
from core.steering_executor import (
    STEERING_EXECUTION_HZ, STEERING_TARGET_MAX_AGE_S, SteeringExecutor,
)
from core.steering_replay import SCHEMA_VERSION, SteeringReplayBuffer


class State:
    def __init__(self, values=None):
        self.values = dict(values or {})
        self.lock = threading.Lock()

    def get(self, key, default=None):
        with self.lock:
            return self.values.get(key, default)

    def set(self, key, value):
        with self.lock:
            self.values[key] = value

    def update_batch(self, values):
        with self.lock:
            self.values.update(values)


class OrderingTests(unittest.TestCase):
    def test_calculation_packet_order_is_monotonic_per_identity(self):
        gate = MonotonicSequenceGate()
        identity = ("intent", "build", 8, 1, "map", "dataset")
        self.assertTrue(gate.observe(identity, 10, 100_000).is_new)
        duplicate = gate.observe(identity, 10, 100_000)
        self.assertTrue(duplicate.accepted)
        self.assertFalse(duplicate.is_new)
        self.assertFalse(gate.observe(identity, 9, 90_000).accepted)
        self.assertFalse(gate.observe(identity, 10, 110_000).accepted)
        self.assertFalse(gate.observe(identity, 11, 99_000).accepted)
        self.assertTrue(gate.observe(identity, 11, 166_664).is_new)

    def test_real_identity_change_resets_frame_and_packet_order(self):
        packet_gate = MonotonicSequenceGate()
        frame_gate = FrameGate()
        old = (1, "map", "dataset")
        new = (2, "map", "dataset")
        self.assertTrue(packet_gate.observe(old, 50, 5_000_000).accepted)
        self.assertTrue(packet_gate.observe(new, 1, 20_000).accepted)
        self.assertTrue(frame_gate.observe(old, 5_000_000).is_new)
        self.assertTrue(frame_gate.observe(new, 20_000).is_new)

    def test_sdk_frame_is_calculated_once_and_regression_fails_closed(self):
        gate = FrameGate()
        identity = (3, 1, "map", "dataset")
        first = gate.observe(identity, 100_000)
        duplicate = gate.observe(identity, 100_000)
        regressed = gate.observe(identity, 99_000)
        self.assertTrue(first.accepted and first.is_new)
        self.assertTrue(duplicate.accepted and not duplicate.is_new)
        self.assertFalse(regressed.accepted)
        self.assertIn("regressed", regressed.reason)

    def test_legacy_frame_zero_keeps_explicit_compatibility(self):
        gate = FrameGate()
        self.assertTrue(gate.observe("legacy", 0).is_new)
        self.assertTrue(gate.observe("legacy", None).is_new)
        self.assertFalse(gate.observe("legacy", -1).accepted)


class FixedCadenceWaitTests(unittest.TestCase):
    def test_early_windows_wake_is_rechecked_before_tick(self):
        clock = [10.0]

        class Event:
            def wait(self, _timeout):
                return False

        sleeps = [0]

        def early_sleep(timeout):
            sleeps[0] += 1
            # Reproduce an early first wake, then reach the real deadline.
            clock[0] += timeout * (0.5 if sleeps[0] == 1 else 2.0)

        deadline, stopped = wait_for_next_tick(
            Event(), 10.0, 1.0 / 60.0, clock=lambda: clock[0],
            sleeper=early_sleep)
        self.assertFalse(stopped)
        self.assertGreaterEqual(clock[0], deadline)
        self.assertEqual(sleeps[0], 2)

    def test_missed_deadline_does_not_create_catch_up_double_tick(self):
        clock = [10.050]

        class Event:
            def wait(self, timeout):
                clock[0] += timeout
                return False

        deadline, stopped = wait_for_next_tick(
            Event(), 10.0, 1.0 / 60.0, clock=lambda: clock[0],
            sleeper=lambda timeout: clock.__setitem__(0, clock[0] + timeout))
        self.assertFalse(stopped)
        self.assertAlmostEqual(deadline, 10.050 + 1.0 / 60.0)
        self.assertGreaterEqual(clock[0], deadline)

    def test_stop_interrupts_wait_without_executing_next_tick(self):
        clock = [10.0]

        class StopEvent:
            def wait(self, _timeout):
                return True

        deadline, stopped = wait_for_next_tick(
            StopEvent(), 10.0, 1.0 / 60.0, clock=lambda: clock[0])
        self.assertTrue(stopped)
        self.assertGreater(deadline, clock[0])


class SteeringExecutorTests(unittest.TestCase):
    def test_physical_trajectory_advances_between_sparse_route_targets(self):
        clock = [0.0]
        writes = []
        executor = SteeringExecutor(
            SteeringDynamics(), writer=writes.append, clock=lambda: clock[0])
        executor.submit(
            0.30, speed_ms=7.0, curvature_per_m=1.0 / 35.0,
            submitted_at=0.0)
        outputs = []
        for frame in range(24):
            clock[0] = (frame + 1) / STEERING_EXECUTION_HZ
            outputs.append(executor.step(
                1.0 / STEERING_EXECUTION_HZ, now=clock[0]))

        # The old plugin-cadence execution held one output for several display
        # frames.  One current Route target now yields a continuous physical
        # trajectory at the output clock without interpolating target values.
        self.assertEqual(outputs, writes)
        self.assertTrue(all(b > a for a, b in zip(outputs, outputs[1:])))
        self.assertLess(max(b - a for a, b in zip(outputs, outputs[1:])), 0.012)
        self.assertGreater(outputs[-1], 0.15)
        self.assertLess(outputs[-1], 0.30)

    def test_execution_obeys_existing_rate_and_acceleration_limits(self):
        clock = [0.0]
        executor = SteeringExecutor(
            SteeringDynamics(), clock=lambda: clock[0])
        executor.submit(
            -0.5, speed_ms=12.0, curvature_per_m=-1.0 / 35.0,
            submitted_at=0.0)
        previous_rate = 0.0
        for frame in range(20):
            clock[0] = (frame + 1) / STEERING_EXECUTION_HZ
            executor.step(1.0 / STEERING_EXECUTION_HZ, now=clock[0])
            debug = executor.last_debug
            self.assertLessEqual(
                abs(debug["rate_per_s"]), debug["max_rate_per_s"] + 1e-9)
            measured_acceleration = ((debug["rate_per_s"] - previous_rate)
                                     * STEERING_EXECUTION_HZ)
            self.assertLessEqual(
                abs(measured_acceleration),
                debug["max_acceleration_per_s2"] + 1e-8)
            previous_rate = debug["rate_per_s"]

    def test_stale_route_target_is_released_by_same_dynamics(self):
        clock = [0.0]
        executor = SteeringExecutor(
            SteeringDynamics(0.20), clock=lambda: clock[0])
        executor.submit(
            0.20, speed_ms=8.0, curvature_per_m=0.01,
            submitted_at=0.0)
        clock[0] = STEERING_TARGET_MAX_AGE_S + 0.01
        output = executor.step(0.01, now=clock[0])
        self.assertFalse(executor.last_debug["target_fresh"])
        self.assertLess(output, 0.20)
        self.assertEqual(executor.last_debug["raw_target"], 0.0)

    def test_reset_is_bumpless_and_does_not_retain_old_velocity(self):
        executor = SteeringExecutor(SteeringDynamics(0.3))
        executor.dynamics.rate = 0.4
        self.assertEqual(executor.reset(-0.12, active=True), -0.12)
        self.assertEqual(executor.output, -0.12)
        self.assertEqual(executor.dynamics.rate, 0.0)


class EngineRealtimeBoundaryTests(unittest.TestCase):
    class FakeTelemetry:
        def __init__(self):
            self.frame = 0
            self.data = {}

        def update(self):
            self.frame += 16_667
            self.data = {
                "raw": {"sdkActive": True},
                "truck": {
                    "pose_valid": True, "sdkFrameTimeUs": self.frame,
                    "speed": 8.0, "gameSteer": 0.1,
                    "roadWheelAnglesRad": [0.07],
                    "referenceGeometry": {"wheelbase_m": 3.8},
                    "yawRateRadS": 0.02, "yawRateValid": True,
                    "x": 10.0, "y": 20.0, "z": 30.0,
                    "rotation": 0.4,
                },
                "trailer": {},
            }
            return True

    class FakeController:
        def __init__(self):
            self.steering_writes = []

        def set_steering(self, value):
            self.steering_writes.append(float(value))

        def set_throttle(self, _value):
            pass

        def set_brake(self, _value):
            pass

        def set_blinker(self, _value):
            pass

        def set_hazard(self, _value):
            pass

        def observe_blinker(self, _value):
            pass

        def release_blinker_pulse(self):
            pass

        def release_all(self):
            pass

    @staticmethod
    def bare_engine(state):
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = state
        engine.fps = 60
        engine.running = True
        engine._realtime_stop = threading.Event()
        engine._telemetry_lock = threading.Lock()
        engine._controller_io_lock = threading.Lock()
        engine._latest_telemetry_data = {}
        engine._latest_telemetry_timestamp = 0.0
        engine._latest_telemetry_success = False
        engine._telemetry_sequence = 0
        engine._last_control_flush = time.monotonic()
        engine._last_output_steering = 0.0
        engine._last_output_brake = 0.0
        engine._drive_selector_pressed = False
        engine._was_active = False
        return engine

    def test_telemetry_acquisition_progresses_without_main_loop(self):
        state = State()
        engine = self.bare_engine(state)
        engine.telemetry = self.FakeTelemetry()
        thread = threading.Thread(target=engine._telemetry_loop)
        thread.start()
        time.sleep(0.12)
        engine.running = False
        engine._realtime_stop.set()
        thread.join(1.0)
        self.assertFalse(thread.is_alive())
        self.assertGreaterEqual(state.get("telemetry_sequence", 0), 4)
        envelope = state.get("vehicle_envelope_snapshot")
        self.assertEqual(envelope["tractor_position"], [10.0, 20.0, 30.0])
        self.assertEqual(envelope["sdk_frame_us"],
                         state.get("telemetry")["truck"]["sdkFrameTimeUs"])

    def test_physical_output_progresses_without_main_loop(self):
        state = State({
            "autopilot_active": True,
            "telemetry_valid": True,
            "autopilot_control_heartbeat": time.monotonic(),
            "ctl_steering": 0.2, "ctl_throttle": 0.0, "ctl_brake": 0.0,
            "truck_speed_ms": 8.0,
            "navigation_source": "gps_lane", "nav_active": True,
            "lane_trajectory_revision": 4, "autopilot_lane_revision": 4,
            "lane_trajectory": {"valid": True, "revision": 4},
        })
        engine = self.bare_engine(state)
        engine.controller = self.FakeController()
        thread = threading.Thread(target=engine._control_loop)
        thread.start()
        time.sleep(0.12)
        engine.running = False
        engine._realtime_stop.set()
        thread.join(1.0)
        self.assertFalse(thread.is_alive())
        self.assertGreaterEqual(len(engine.controller.steering_writes), 4)
        self.assertTrue(all(value == 0.2
                            for value in engine.controller.steering_writes))


class ReplayExecutionEvidenceTests(unittest.TestCase):
    def test_execution_ring_retains_same_time_horizon_as_plugin_ring(self):
        replay = SteeringReplayBuffer(4)
        for sequence in range(10):
            replay.append_execution({"output": sequence / 10.0})
        self.assertEqual(replay.capacity, 4)
        self.assertEqual(replay.execution_capacity, 12)
        self.assertEqual(len(replay.execution_snapshot()), 10)

    def test_export_binds_calculation_and_execution_streams(self):
        replay = SteeringReplayBuffer(
            4, monotonic=lambda: 5.0, wall_time=lambda: 10.0)
        replay.append({"calculation_sequence": 7})
        replay.append_execution({
            "submission_sequence": 3, "output": 0.12,
            "calculation_sequence": 7,
        })

        class CapturedFile:
            def __init__(self):
                self.value = ""

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def write(self, value):
                self.value += value

            def flush(self):
                pass

            def fileno(self):
                return 7

        stream = CapturedFile()
        with (mock.patch("builtins.open", return_value=stream),
              mock.patch("core.steering_replay.os.makedirs"),
              mock.patch("core.steering_replay.os.fsync"),
              mock.patch("core.steering_replay.os.replace")):
            replay.export("C:\\diagnostics", reason="phase4d")
        payload = json.loads(stream.value)
        self.assertEqual(payload["schema_version"], SCHEMA_VERSION)
        self.assertEqual(payload["execution_capacity"], 12)
        self.assertEqual(payload["execution_sample_count"], 1)
        self.assertEqual(payload["execution_samples"][0][
            "calculation_sequence"], 7)


class MapFrameIntegrationTests(unittest.TestCase):
    def test_map_uses_sdk_interval_and_never_recalculates_same_frame(self):
        from tests.test_lane_authority_integration import build_map_plugin, Tags

        plugin, sdk, point = build_map_plugin()
        plugin.tags = Tags()
        sdk.set("truck_world_pos", (point.x, point.z))
        sdk.set("truck_heading", point.heading)
        sdk.set("truck_speed_ms", 8.0)
        sdk.set("telemetry_generation", 2)
        plugin._load_road_net = lambda: None
        plugin._publish_road_type = lambda *_args: None
        plugin._schedule_hud_road_scene = lambda *_args, **_kwargs: False
        plugin._schedule_live_map_scene = lambda *_args, **_kwargs: False

        def observation(frame, timestamp):
            return {
                "timestamp": timestamp, "sdk_frame_us": frame,
                "tractor_position": [point.x, point.y, point.z],
                "tractor_heading": point.heading, "tractor_speed_ms": 8.0,
                "road_wheel_angles_rad": [0.0],
                "tractor_reference_geometry": {
                    "valid": True, "source": "stage4d",
                    "reference_ahead_m": 2.1, "wheelbase_m": 3.8,
                },
            }

        now = time.monotonic()
        sdk.set("vehicle_envelope_snapshot", observation(100_000, now))
        plugin.on_tick(0.01)
        first = sdk.get("nav_steering_debug")
        plugin.on_tick(0.09)
        duplicate = sdk.get("nav_steering_debug")
        self.assertEqual(duplicate["calculation_sequence"],
                         first["calculation_sequence"])

        sdk.set("vehicle_envelope_snapshot",
                observation(166_664, now + 0.066664))
        plugin.on_tick(0.002)
        second = sdk.get("nav_steering_debug")
        self.assertEqual(second["calculation_sequence"],
                         first["calculation_sequence"] + 1)
        self.assertAlmostEqual(second["control_dt_s"], 0.066664, places=6)

        sdk.set("vehicle_envelope_snapshot",
                observation(150_000, now + 0.07))
        plugin.on_tick(0.01)
        self.assertFalse(sdk.get("nav_active"))
        self.assertIn("regressed", sdk.get("steering_observation_failure"))


if __name__ == "__main__":
    unittest.main()
