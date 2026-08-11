import math
import unittest

from core.navigation.route import Route
from core.steering_dynamics import SteeringDynamics


class Phase4DCurveCoherenceTests(unittest.TestCase):
    """Regressions for the complete 2026-08-11 22:09:46 drive."""

    # Exact one-second diagnostics from 22:10:09--22:10:19.  Intermediate
    # control ticks are linearly reconstructed between captured endpoints;
    # no spatial route or authority value is invented.
    REAL_CURVE_SAMPLES = (
        # time, curve_reference, heading_fb, cte_fb, lane_cte,
        # guidance heading degrees, speed km/h
        ("22:10:09.066", -0.076, -0.048, 0.049, -0.139, -1.4, 55.0),
        ("22:10:10.090", -0.099, -0.014, 0.117, -0.515, -0.4, 54.0),
        ("22:10:11.104", -0.123, -0.040, 0.125, -0.418, -1.2, 49.0),
        ("22:10:12.111", -0.148, -0.031, 0.133, -0.383, -0.9, 47.0),
        ("22:10:13.113", -0.157, -0.042, 0.148, -0.419, -1.2, 44.0),
        ("22:10:14.124", -0.152, -0.035, 0.135, -0.315, -1.0, 41.0),
        ("22:10:15.153", -0.135, 0.040, 0.151, -0.461, 1.2, 42.0),
        ("22:10:16.167", -0.139, -0.072, 0.161, -0.498, -2.1, 43.0),
        ("22:10:17.175", -0.146, -0.031, 0.144, -0.428, -0.9, 45.0),
        ("22:10:18.186", -0.152, -0.045, 0.135, -0.398, -1.3, 44.0),
        ("22:10:19.206", -0.142, -0.026, 0.120, -0.303, -0.8, 44.0),
    )

    @staticmethod
    def _legacy_phase4c_command(feed_forward, heading_fb, cte_fb):
        candidate = feed_forward + heading_fb + cte_fb
        if abs(feed_forward) >= 0.06:
            sign = 1.0 if feed_forward > 0.0 else -1.0
            minimum = abs(feed_forward) * 0.20
            if candidate * sign < minimum:
                candidate = sign * minimum
        return candidate

    def test_real_221009_curve_replay_removes_feedback_pulses(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        dynamics = SteeringDynamics()
        legacy = []
        endpoints = []
        physical = []
        feedback_rates = []
        feedback_accels = []
        feedback_jerks = []
        previous = self.REAL_CURVE_SAMPLES[0]
        for sample_index, current in enumerate(self.REAL_CURVE_SAMPLES):
            legacy.append(self._legacy_phase4c_command(
                current[1], current[2], current[3]))
            for tick in range(20):
                fraction = 1.0 if sample_index == 0 else (tick + 1) / 20.0
                values = [
                    previous[index] + (current[index] - previous[index]) * fraction
                    for index in range(1, 7)
                ]
                feed_forward, heading_fb, cte_fb, lane_cte, heading_deg, speed = values
                command, debug = route._compose_curve_steering(
                    feed_forward, heading_fb, cte_fb, -lane_cte,
                    math.radians(heading_deg), True, 4.7, 0.05,
                    speed_ms=speed / 3.6,
                    curve_coherence_enabled=True)
                curvature = math.tan(feed_forward * 0.28) / 3.8
                physical_command = dynamics.update(
                    command, 0.05, speed_ms=speed / 3.6,
                    curvature_per_m=curvature)
                feedback_rates.append(debug["coherent_feedback_rate_per_s"])
                feedback_accels.append(
                    debug["coherent_feedback_acceleration_per_s2"])
                feedback_jerks.append(debug["coherent_feedback_jerk_per_s3"])
            endpoints.append(command)
            physical.append(physical_command)
            previous = current

        legacy_variation = sum(abs(second - first) for first, second in zip(
            legacy, legacy[1:]))
        repaired_variation = sum(abs(second - first) for first, second in zip(
            endpoints, endpoints[1:]))
        # 22:10:12 onward is the monotonic R77--R117 bend identified in the
        # report.  The earlier samples contain the real curve-entry ramp and
        # must not be flattened merely to improve a whole-window statistic.
        legacy_bend_variation = sum(
            abs(second - first) for first, second in zip(
                legacy[3:], legacy[4:]))
        repaired_bend_variation = sum(
            abs(second - first) for first, second in zip(
                endpoints[3:], endpoints[4:]))
        repaired_bend_steps = [
            abs(second - first) for first, second in zip(
                endpoints[3:], endpoints[4:])]
        self.assertGreater(legacy_variation, 0.18)
        self.assertLess(repaired_variation, legacy_variation * 0.85)
        self.assertLess(
            repaired_bend_variation, legacy_bend_variation * 0.68)
        self.assertLess(max(repaired_bend_steps), 0.012)
        self.assertLess(
            max(endpoints[3:]) - min(endpoints[3:]), 0.012)
        self.assertTrue(all(value < 0.0 for value in endpoints))
        self.assertTrue(all(value < 0.0 for value in physical[2:]))
        self.assertLess(max(map(abs, feedback_rates)), 0.30)
        self.assertLess(max(map(abs, feedback_accels)), 2.10)
        self.assertLess(max(map(abs, feedback_jerks)), 14.10)

    def test_loaded_left_right_curves_are_coherent_across_speeds(self):
        for direction in (-1.0, 1.0):
            for speed_ms in (5.0, 12.0, 20.0):
                for loaded_cte_feedback in (0.06, 0.12, 0.18):
                    route = Route([(0.0, 0.0), (0.0, -20.0)])
                    commands = []
                    for tick in range(120):
                        heading_noise = (0.045, -0.050, 0.035, -0.040)[
                            tick % 4]
                        command, debug = route._compose_curve_steering(
                            direction * 0.15,
                            -direction * heading_noise,
                            -direction * loaded_cte_feedback,
                            0.34, math.radians(1.2), True, 4.7, 0.05,
                            speed_ms=speed_ms,
                            curve_coherence_enabled=True)
                        commands.append(command)
                        self.assertLessEqual(
                            abs(debug["coherent_feedback_rate_per_s"]),
                            debug["coherent_feedback_rate_limit_per_s"] + 1e-9)
                        self.assertLessEqual(
                            abs(debug[
                                "coherent_feedback_acceleration_per_s2"]),
                            debug[
                                "coherent_feedback_accel_limit_per_s2"] + 1e-9)
                        self.assertLessEqual(
                            abs(debug["coherent_feedback_jerk_per_s3"]),
                            debug[
                                "coherent_feedback_jerk_limit_per_s3"] + 1e-9)
                    settled = commands[40:]
                    with self.subTest(
                            direction=direction, speed=speed_ms,
                            load=loaded_cte_feedback):
                        self.assertTrue(all(
                            value * direction > 0.0 for value in settled))
                        self.assertLess(max(settled) - min(settled), 0.025)

    def test_loaded_s_curve_switches_immediately_and_exit_does_not_hunt(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        for _ in range(50):
            left, _debug = route._compose_curve_steering(
                -0.20, 0.04, 0.12, 0.30, math.radians(1.0),
                True, 4.7, 0.05, speed_ms=12.0,
                curve_coherence_enabled=True)
        right, debug = route._compose_curve_steering(
            0.20, -0.04, -0.12, -0.30, math.radians(-1.0),
            True, 4.7, 0.05, speed_ms=12.0,
            curve_coherence_enabled=True)
        self.assertLess(left, 0.0)
        self.assertGreater(right, 0.0)
        self.assertEqual(debug["curve_foundation_sign"], 1)

        exit_commands = []
        for tick in range(40):
            fraction = 1.0 - (tick + 1) / 40.0
            command, _debug = route._compose_curve_steering(
                0.0, -0.04 * fraction, -0.12 * fraction,
                -0.30 * fraction, math.radians(-1.0 * fraction),
                True, 4.7, 0.05, speed_ms=12.0,
                curve_coherence_enabled=True)
            exit_commands.append(command)
        self.assertTrue(all(value <= 1e-9 for value in exit_commands))
        self.assertLess(abs(exit_commands[-1]), 0.005)

    def test_proven_lane_departure_retains_fast_opposite_recovery(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        commands = []
        for _ in range(14):
            command, debug = route._compose_curve_steering(
                -0.24, 0.18, 0.27, 1.10, math.radians(12.0),
                True, 4.7, 0.05, speed_ms=12.0,
                curve_coherence_enabled=True)
            commands.append(command)
        self.assertTrue(debug["opposite_correction_authorized"])
        self.assertTrue(debug["feedback_urgent"])
        self.assertGreater(commands[-1], 0.0)


if __name__ == "__main__":
    unittest.main()
