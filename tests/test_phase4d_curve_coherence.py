import math
import unittest

from core.navigation.route import Route


class StatelessCurveCompositionTests(unittest.TestCase):
    """Regressions for removing the Phase 4C/4D stateful correction relay."""

    # Exact contributions reported once per second during the monotonic left
    # bend at 22:10:09--22:10:19.  They remain useful as real input coverage,
    # but the new controller does not reconstruct or filter hidden ticks.
    REAL_CURVE_SAMPLES = (
        # time, curve_reference, heading_fb, cte_fb, Route CTE,
        # guidance heading degrees, speed km/h
        ("22:10:09.066", -0.076, -0.048, 0.049, 0.139, -1.4, 55.0),
        ("22:10:10.090", -0.099, -0.014, 0.117, 0.515, -0.4, 54.0),
        ("22:10:11.104", -0.123, -0.040, 0.125, 0.418, -1.2, 49.0),
        ("22:10:12.111", -0.148, -0.031, 0.133, 0.383, -0.9, 47.0),
        ("22:10:13.113", -0.157, -0.042, 0.148, 0.419, -1.2, 44.0),
        ("22:10:14.124", -0.152, -0.035, 0.135, 0.315, -1.0, 41.0),
        ("22:10:15.153", -0.135, 0.040, 0.151, 0.461, 1.2, 42.0),
        ("22:10:16.167", -0.139, -0.072, 0.161, 0.498, -2.1, 43.0),
        ("22:10:17.175", -0.146, -0.031, 0.144, 0.428, -0.9, 45.0),
        ("22:10:18.186", -0.152, -0.045, 0.135, 0.398, -1.3, 44.0),
        ("22:10:19.206", -0.142, -0.026, 0.120, 0.303, -0.8, 44.0),
    )

    @staticmethod
    def _compose(route, sample):
        (_timestamp, feed_forward, heading_fb, cte_fb, tractor_cte,
         heading_deg, speed_kmh) = sample
        return route._compose_curve_steering(
            feed_forward, heading_fb, cte_fb)

    def test_real_monotonic_curve_replay_is_exact_and_never_reverses(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        commands = []
        for sample in self.REAL_CURVE_SAMPLES:
            timestamp, feed_forward, heading_fb, cte_fb, *_rest = sample
            command, debug = self._compose(route, sample)
            unprojected = feed_forward + heading_fb + cte_fb
            expected = (0.0 if unprojected * feed_forward < 0.0
                        else unprojected)
            commands.append(command)
            self.assertAlmostEqual(command, expected, places=12,
                                   msg=timestamp)
            self.assertGreaterEqual(command * feed_forward, 0.0,
                                    msg=timestamp)
            self.assertEqual(
                debug["curve_sign_projection_active"],
                unprojected * feed_forward < 0.0)
            self.assertNotIn("opposite_correction_authorized", debug)
            self.assertNotIn("coherent_feedback_rate_per_s", debug)
            self.assertNotIn("feedback_worsening_s", debug)
        self.assertTrue(all(command <= 0.0 for command in commands))

    def test_same_inputs_are_independent_of_history_speed_and_dt(self):
        target = (-0.18, 0.035, 0.070, 0.42, math.radians(1.8))
        results = []
        for seed in range(8):
            route = Route([(0.0, 0.0), (0.0, -20.0)])
            for tick in range(seed * 11):
                sign = -1.0 if tick % 2 else 1.0
                route._compose_curve_steering(
                    sign * 0.30, -sign * 0.50, sign * 0.45)
            command, debug = route._compose_curve_steering(
                *target[:3])
            results.append(command)
            self.assertFalse(debug["curve_sign_projection_active"])
        self.assertTrue(all(value == results[0] for value in results))
        self.assertAlmostEqual(results[0], -0.075, places=12)

    def test_feedback_reduces_both_curve_directions_to_zero_not_beyond(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        for direction in (-1.0, 1.0):
            for correction in (0.04, 0.12, 0.30, 0.90):
                command, debug = route._compose_curve_steering(
                    direction * 0.24,
                    -direction * correction * 0.40,
                    -direction * correction * 0.60)
                with self.subTest(direction=direction,
                                  correction=correction):
                    self.assertGreaterEqual(command * direction, 0.0)
                    self.assertLessEqual(abs(command), 0.24 + 1e-12)
                    self.assertEqual(
                        debug["curve_sign_projection_active"],
                        correction > 0.24)
            # A severe controller-created error is not geometric authority to
            # steer against the immutable monotonic bend.
            command, debug = route._compose_curve_steering(
                direction * 0.24, -direction * 0.30,
                -direction * 0.35)
            self.assertEqual(command, 0.0)
            self.assertTrue(debug["curve_sign_projection_active"])
            self.assertNotIn("opposite_correction_authorized", debug)

    def test_true_s_curve_switches_foundation_sign_immediately(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        left, left_debug = route._compose_curve_steering(
            -0.20, 0.04, 0.08)
        right, right_debug = route._compose_curve_steering(
            0.20, -0.04, -0.08)
        self.assertLess(left, 0.0)
        self.assertGreater(right, 0.0)
        self.assertEqual(left_debug["curve_foundation_sign"], -1)
        self.assertEqual(right_debug["curve_foundation_sign"], 1)

    def test_straight_uses_signed_feedback_without_curve_projection(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        for feedback in (-0.30, -0.05, 0.0, 0.05, 0.30):
            command, debug = route._compose_curve_steering(
                0.0, feedback * 0.40, feedback * 0.60)
            self.assertAlmostEqual(command, feedback, places=12)
            self.assertFalse(debug["curve_foundation_active"])
            self.assertFalse(debug["curve_sign_projection_active"])


if __name__ == "__main__":
    unittest.main()
