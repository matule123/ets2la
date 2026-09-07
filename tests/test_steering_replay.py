import json
import io
import unittest
from unittest import mock

from core.steering_replay import SteeringReplayBuffer
from plugins.autopilot.main import Plugin as AutopilotPlugin


class State:
    def __init__(self, values=None):
        self.values = dict(values or {})

    def get(self, key, default=None):
        return self.values.get(key, default)

    def set(self, key, value):
        self.values[key] = value

    def update_batch(self, values):
        self.values.update(values)


class SteeringReplayBufferTests(unittest.TestCase):
    def test_ring_is_bounded_and_export_is_atomic_json(self):
        ticks = iter((1.0, 2.0, 3.0, 4.0))
        wall = iter((101.0, 102.0, 103.0, 104.0))
        replay = SteeringReplayBuffer(
            2, monotonic=lambda: next(ticks), wall_time=lambda: next(wall))
        replay.append({"steer_raw": -0.1, "bad": float("nan")})
        replay.append({"steer_raw": -0.2})
        replay.append({"steer_raw": -0.3})
        self.assertEqual(len(replay), 2)
        self.assertEqual([row["sequence"] for row in replay.snapshot()], [2, 3])

        class CapturedFile(io.StringIO):
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def close(self):
                pass

            def fileno(self):
                return 7

        stream = CapturedFile()
        with (mock.patch("builtins.open", return_value=stream),
              mock.patch("core.steering_replay.os.makedirs"),
              mock.patch("core.steering_replay.os.fsync"),
              mock.patch("core.steering_replay.os.replace") as replace):
            path = replay.export(
                "C:\\diagnostics", reason="automatic disable",
                identity={"revision": 9, "navigation_intent_id": "intent"})
        payload = json.loads(stream.getvalue())
        self.assertEqual(payload["sample_count"], 2)
        self.assertEqual(payload["dropped_sample_count"], 1)
        self.assertEqual(payload["identity"]["revision"], 9)
        self.assertEqual(payload["samples"][-1]["steer_raw"], -0.3)
        replace.assert_called_once()
        self.assertEqual(replace.call_args.args[1], path)

    def test_autopilot_tick_capture_contains_requested_dense_evidence(self):
        state = State({
            "autopilot_active": True,
            "nav_active": True,
            "engine_applied_steering": -0.20,
            "navigation_intent_id": "intent-9",
            "lane_match": {
                "revision": 9,
                "lateral_error_m": 0.12,
                "heading_error_rad": -0.03,
                "elevation_layer": 4,
                "active_lane_id": {"road_uid": 77, "direction": 1,
                                   "lane_index": 0},
            },
            "lane_trajectory": {
                "revision": 9, "route_build_id": "build-9",
                "navigation_intent_id": "intent-9",
                "source_game_session_id": "session-9",
                "source_map_key": "promods-1.59",
                "source_dataset_fingerprint": "fingerprint-9",
            },
            "nav_steering_debug": {
                "computed_at": 40.0,
                "observation_timestamp": 39.99,
                "preview_curvature": -0.012,
                "local_curvature": -0.010,
                "feed_forward": -0.15,
                "feedback": 0.03,
                "feedback_applied": 0.03,
                "heading_feedback": -0.01,
                "cte_feedback": 0.04,
                "vehicle_curvature_source": "road_wheel_angles_rad",
                "curvature_observation_weight": 0.8,
                "predicted_frenet_cte_m": 0.08,
                "predicted_frenet_heading_error_rad": -0.02,
                "tracking_progress_m": 81.5,
                "tracking_segment_index": 41,
                "tracking_segment_fraction": 0.25,
                "tracking_projection_xz": [10.0, 20.0],
                "local_tangent_heading_rad": 0.4,
                "observation_xz": [10.1, 20.2],
                "observation_heading_rad": 0.42,
                "trailer_envelope": {
                    "trailer_cte_m": 0.5,
                    "required_offset_m": 0.2,
                    "applied_offset_m": 0.2,
                    "preview_curvature_per_m": -0.011,
                },
            },
        })
        plugin = AutopilotPlugin.__new__(AutopilotPlugin)
        plugin.sdk = type("SDK", (), {"shared_state": state})()
        plugin._last_control_dt = 0.016
        plugin._steering_dynamics_debug = {
            "raw_target": -0.12, "bounded_target": -0.12,
            "rate_per_s": -0.2, "acceleration_per_s2": 0.4,
        }
        plugin._accepted_navigation_command = {
            "computed_at": 39.98,
            "observation_timestamp": 39.97,
            "sdk_frame_us": 123455,
            "output": -0.12,
            "command": -0.12,
            "curvature_per_m": -0.010,
            "rejection": "",
            "controller": "frenet_bicycle",
            "authority_revision": 9,
            "navigation_intent_id": "intent-9",
            "route_build_id": "build-9",
        }
        plugin._steering_replay = SteeringReplayBuffer(8)
        truck = {"sdkFrameTimeUs": 123456, "roadWheelAnglesRad": [-0.14],
                 "yawRateRadS": -0.2}
        plugin._record_steering_replay_tick(
            truck, state.get("lane_trajectory"), -0.11, -0.10, 35.0, "")
        row = plugin._steering_replay.snapshot()[0]
        for key in (
                "lane_cte", "lane_cte_m", "lane_heading",
                "lane_heading_error_rad", "preview_k", "preview_k_per_m",
                "steer_raw", "steer_out", "game_steer", "game_steer_right",
                "tyre_angles_rad", "yaw_right_rad_s", "sdk_frame_us",
                "vehicle_curvature_source", "curvature_observation_weight",
                "predicted_frenet_cte_m",
                "predicted_frenet_heading_error_rad", "route_build_id",
                "revision", "lane_id", "accepted_nav_command",
                "accepted_packet_output", "accepted_packet_sdk_frame_us",
                "accepted_packet_revision", "tracking_progress_m",
                "tracking_segment_index", "tracking_segment_fraction",
                "tracking_projection_xz", "local_tangent_heading_rad",
                "observation_xz", "observation_heading_rad"):
            self.assertIn(key, row)
        self.assertEqual(row["sdk_frame_us"], 123456)
        self.assertEqual(row["route_build_id"], "build-9")
        self.assertEqual(row["accepted_nav_command"], -0.12)
        self.assertEqual(row["accepted_packet_sdk_frame_us"], 123455)

    def test_export_failure_cannot_block_automatic_disable(self):
        state = State({"autopilot_active": True,
                       "lane_trajectory": {"revision": 1}})
        plugin = AutopilotPlugin.__new__(AutopilotPlugin)
        plugin.sdk = type("SDK", (), {"shared_state": state})()
        plugin._steering_replay = mock.Mock()
        plugin._steering_replay.__len__ = mock.Mock(return_value=1)

        def fail_after_disable(*_args, **_kwargs):
            self.assertFalse(state.get("autopilot_active"))
            self.assertFalse(state.get("nav_active"))
            raise OSError("disk full")

        plugin._steering_replay.export.side_effect = fail_after_disable
        with mock.patch("plugins.autopilot.main.app_dir", return_value="X:\\bad"):
            plugin._publish_automatic_disable("test safety reason")
        self.assertFalse(state.get("autopilot_active"))
        self.assertEqual(state.get("autopilot_disable_reason"),
                         "test safety reason")


if __name__ == "__main__":
    unittest.main()
