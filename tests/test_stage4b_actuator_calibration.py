import math
import statistics
import time
import unittest

from core.navigation.route import Route
from core.settings.manager import SettingsManager
from core.steering_calibration import (
    DEFAULT_COMMAND_DELAY_S,
    DEFAULT_OBSERVATION_DELAY_S,
    DEFAULT_TYRE_ANGLE_PER_INPUT_RAD,
    steering_calibration_from_settings,
)
from plugins.autopilot.main import navigation_command
from tests.steering_bench import path, run
from tools.audit_steering_actuator import analyze_documents


REFERENCE_GEOMETRY = {
    "valid": True,
    "source": "phase4b_test_4x2",
    "reference_ahead_m": 2.1,
    "wheelbase_m": 3.8,
}


class Stage4BActuatorCalibrationTests(unittest.TestCase):
    def test_clean_install_has_one_explicit_static_calibration(self):
        manager = SettingsManager.__new__(SettingsManager)
        autopilot = manager._get_defaults()["autopilot"]
        self.assertNotIn("steering_lock_rad", autopilot)
        calibration = steering_calibration_from_settings(autopilot)
        self.assertTrue(calibration.valid)
        self.assertEqual(calibration.tyre_angle_per_input_rad, 0.70)
        self.assertAlmostEqual(calibration.preview_horizon_s, 0.134)
        self.assertFalse(calibration.as_dict()["online_adaptation"])

    def test_existing_scalar_is_migrated_without_becoming_second_authority(self):
        calibration = steering_calibration_from_settings({
            "steering_lock_rad": 0.72,
        })
        self.assertTrue(calibration.valid)
        self.assertTrue(calibration.legacy_lock_setting)
        self.assertEqual(calibration.tyre_angle_per_input_rad, 0.72)
        self.assertEqual(calibration.command_delay_s,
                         DEFAULT_COMMAND_DELAY_S)
        self.assertEqual(calibration.observation_delay_s,
                         DEFAULT_OBSERVATION_DELAY_S)

    def test_structured_calibration_is_the_only_authority(self):
        calibration = steering_calibration_from_settings({
            "steering_lock_rad": 0.95,
            "steering_actuator_calibration": {
                "schema_version": 1,
                "tyre_angle_per_input_rad": 0.68,
                "command_delay_s": 0.08,
                "observation_delay_s": 0.06,
                "source": "bench",
            },
        })
        self.assertTrue(calibration.valid)
        self.assertFalse(calibration.legacy_lock_setting)
        self.assertEqual(calibration.tyre_angle_per_input_rad, 0.68)
        self.assertAlmostEqual(calibration.preview_horizon_s, 0.14)

    def test_malformed_or_out_of_range_calibration_fails_closed(self):
        valid = {
            "schema_version": 1,
            "tyre_angle_per_input_rad": 0.70,
            "command_delay_s": 0.067,
            "observation_delay_s": 0.067,
            "source": "test",
        }
        invalid = (
            None,
            {**valid, "schema_version": 2},
            {**valid, "schema_version": 1.5},
            {**valid, "source": ""},
            {**valid, "tyre_angle_per_input_rad": 0.59},
            {**valid, "tyre_angle_per_input_rad": 0.951},
            {**valid, "command_delay_s": -0.01},
            {**valid, "command_delay_s": 0.201},
            {**valid, "observation_delay_s": 0.251},
            {**valid, "tyre_angle_per_input_rad": float("nan")},
        )
        for payload in invalid:
            settings = ({"steering_actuator_calibration": payload}
                        if payload is not None else
                        {"steering_actuator_calibration": "invalid"})
            with self.subTest(payload=payload):
                calibration = steering_calibration_from_settings(settings)
                self.assertFalse(calibration.valid)
                self.assertTrue(calibration.failure_reason)

    def test_route_reports_exact_calibration_failure(self):
        route = Route([(0.0, 0.0), (0.0, 50.0)])
        command = route.steering(
            (0.0, 5.0), math.pi, 5.0,
            reference_geometry=REFERENCE_GEOMETRY,
            actuator_calibration_failure="unsupported chassis calibration")
        self.assertEqual(command, 0.0)
        self.assertFalse(route.last_steering_debug["authority_valid"])
        self.assertIn("unsupported chassis calibration",
                      route.last_steering_debug["control_failure"])

    def test_preview_latency_does_not_change_feedback_pole_placement(self):
        data = path([(0.0, 55.0), (1.0 / 35.0, 100.0), (0.0, 80.0)])
        diagnostics = []
        for preview in (0.134, 0.45):
            route = Route(data["points"])
            progress = 50.0
            position = route._point_at_progress(progress)
            before = route._point_at_progress(progress - 2.0)
            after = route._point_at_progress(progress + 2.0)
            heading = math.atan2(
                -(after[0] - before[0]), -(after[1] - before[1]))
            route.steering(
                position, heading, 25.0, cross_track_error_m=0.0,
                steering_lock_rad=0.70,
                reference_geometry=REFERENCE_GEOMETRY,
                actuator_response_s=0.45,
                curvature_preview_s=preview)
            diagnostics.append(route.last_steering_debug)
        short, old = diagnostics
        self.assertAlmostEqual(short["feedback_length_m"],
                               old["feedback_length_m"], places=12)
        self.assertAlmostEqual(short["local_curvature"],
                               old["local_curvature"], places=12)
        self.assertLess(short["preview_curvature"],
                        old["preview_curvature"] * 0.10)
        self.assertLess(abs(short["feed_forward"]),
                        abs(old["feed_forward"]) * 0.10)

    def test_real_20260910_wheel_samples_identify_point_70_rad_gain(self):
        # Frame-bound gameSteer and roadWheelAnglesRad pairs copied from the
        # no-trailer replay. They cover both signs and several speeds.
        pairs = (
            (-0.08026495575904846, -0.056048537526561526),
            (-0.12075679749250412, -0.08434942508069668),
            (-0.1631418764591217, -0.11400792109324487),
            (-0.22354549169540405, -0.15635183912739548),
            (-0.1740790605545044, -0.12166601050626491),
            (-0.13261984288692474, -0.09264683895271805),
            (-0.0913451761007309, -0.06379060721483998),
            (0.09077009558677673, 0.06338878458017151),
        )
        ratios = [tyre / game for game, tyre in pairs]
        self.assertAlmostEqual(statistics.median(ratios),
                               DEFAULT_TYRE_ANGLE_PER_INPUT_RAD,
                               delta=0.002)
        self.assertLess(max(ratios) - min(ratios), 0.003)

    def test_calibration_change_invalidates_older_bound_packet(self):
        current = steering_calibration_from_settings({}).as_dict()
        packet_calibration = dict(current)
        packet_calibration["command_delay_s"] = 0.08
        snapshot = {
            "revision": 8,
            "navigation_intent_id": "intent",
            "route_build_id": "build",
            "source_game_session_id": 1,
            "source_map_key": "map",
            "source_dataset_fingerprint": "dataset",
        }
        packet = {
            "calculation_packet_schema_version": 1,
            "controller": "frenet_bicycle",
            "authority_valid": True,
            "authority_revision": 8,
            "navigation_intent_id": "intent",
            "route_build_id": "build",
            "source_game_session_id": 1,
            "source_map_key": "map",
            "source_dataset_fingerprint": "dataset",
            "computed_at": time.monotonic(),
            "observation_timestamp": time.monotonic(),
            "output": 0.1,
            "local_curvature": 0.01,
            "actuator_calibration": packet_calibration,
        }
        state = {"steering_actuator_calibration": current}
        command, curvature, reason = navigation_command(
            state, snapshot, gps_active=True, packet=packet)
        self.assertEqual((command, curvature), (0.0, 0.0))
        self.assertEqual(reason,
                         "steering command actuator calibration is stale")

    def test_dense_analyser_finds_one_frame_delay_without_interpolation(self):
        samples = []
        commands = (0.0, 0.10, 0.20, 0.15, -0.05, -0.10)
        for index, command in enumerate(commands):
            game = commands[max(0, index - 1)]
            samples.append({
                "application_sdk_frame_us": index * 66_664,
                "engine_applied_steering": command,
                "game_steer_right": game,
                "tyre_angles_rad": [0.70 * game, 0.70 * game],
                "speed_kmh": 30.0,
                "yaw_right_rad_s": 0.0,
                "packet_age_s": 0.04,
                "frame_lag_us": 66_664,
                "autopilot_active": True,
                "packet_binding_valid": True,
            })
        document = {
            "identity": {
                "source_game_session_id": 1,
                "source_map_key": "map",
                "source_dataset_fingerprint": "dataset",
            },
            "samples": samples,
        }
        result = analyze_documents([document])
        self.assertEqual(result["best_command_lag_frames"], 1)
        self.assertAlmostEqual(result["identified_command_delay_s"],
                               0.066664, places=6)
        self.assertAlmostEqual(
            result["tyre_rad_per_game_input"]["median"], 0.70)


class Stage4BClosedLoopTests(unittest.TestCase):
    def _measure(self, data, options, preview, *, lag=0.01,
                 transport=0.067):
        return run(
            data, lock_rad=0.70, controller_lock_rad=0.70,
            controller_preview_s=preview, lag=lag, transport=transport,
            noisy=True, jitter=True, observation_ahead_m=2.1,
            controller_ahead_m=2.1, **options)[0]

    def test_identified_preview_improves_observed_fast_plant_both_directions(self):
        for direction in (-1.0, 1.0):
            for radius, speed in ((250.0, 25.0), (83.0, 12.0),
                                  (35.0, 7.4), (25.0, 6.0)):
                data = path([(0.0, 55.0),
                             (direction / radius, max(100.0, radius * 1.2)),
                             (0.0, 100.0)])
                old = self._measure(data, {"speed": speed}, 0.45)
                new = self._measure(data, {"speed": speed}, 0.134)
                with self.subTest(direction=direction, radius=radius):
                    self.assertTrue(new["completed"])
                    self.assertFalse(new["lost_lane"])
                    self.assertEqual(new["monotone_opposite_samples"], 0)
                    self.assertLess(new["max_cte"], old["max_cte"])
                    self.assertLess(new["rms_cte"], old["rms_cte"])
                    self.assertLess(new["max_cte"], 0.35)

    def test_s_curve_and_slow_actuator_stress_remain_bounded(self):
        data = path([(0.0, 55.0), (-1.0 / 35.0, 75.0),
                     (1.0 / 35.0, 75.0), (0.0, 100.0)])
        observed = self._measure(data, {"speed": 7.4}, 0.134)
        slow = self._measure(data, {"speed": 7.4}, 0.134,
                             lag=0.60, transport=0.10)
        self.assertLess(observed["max_cte"], 0.35)
        self.assertLess(slow["max_cte"], 0.70)
        self.assertTrue(slow["completed"])
        self.assertFalse(slow["lost_lane"])
        self.assertEqual(slow["monotone_opposite_samples"], 0)

    def test_straight_response_is_identical_at_10_to_90_kmh(self):
        data = path([(0.0, 150.0), (0.0, 150.0)])
        for speed_kmh in (10.0, 30.0, 60.0, 90.0):
            options = {"speed": speed_kmh / 3.6, "initial_cte": 0.5}
            old = self._measure(data, options, 0.45)
            new = self._measure(data, options, 0.134)
            with self.subTest(speed_kmh=speed_kmh):
                self.assertAlmostEqual(new["max_cte"], old["max_cte"],
                                       places=12)
                self.assertAlmostEqual(new["rms_cte"], old["rms_cte"],
                                       places=12)
                self.assertFalse(new["lost_lane"])


if __name__ == "__main__":
    unittest.main()
