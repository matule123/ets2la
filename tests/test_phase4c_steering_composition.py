import math
import unittest

from core.navigation.route import (
    TRAILER_BALANCED_REFERENCE_FRACTION,
    Route,
)
from tests.test_game_like_control_simulation import _path_with_curve, _simulate


class Phase4CSteeringCompositionTests(unittest.TestCase):
    """Deterministic regressions for the 2026-08-10 13:42 drive."""

    @staticmethod
    def _arc(direction, radius, length_m):
        return [
            (direction * (radius - radius * math.cos(s / radius)),
             radius * math.sin(s / radius))
            for s in range(0, int(length_m) + 1, 2)
        ]

    @staticmethod
    def _heading(first, second):
        return math.atan2(-(second[0] - first[0]),
                          -(second[1] - first[1]))

    def test_legacy_log_terms_are_not_a_second_steering_authority(self):
        # Exact one-second controller contributions from the failed
        # 2026-08-11 drive.  The old stateful controller returned +0.473 at
        # 22:59:33 in this *left* bend; the unified composition is deliberately
        # a pure function of the three signed steering contributions.
        samples = (
            # time, curve_ref, heading_fb, cte_fb, Route CTE, heading
            ("22:59:29.360", -0.020, 0.020, 0.011, 0.034, 0.6),
            ("22:59:30.383", -0.036, 0.010, 0.024, 0.118, 0.3),
            ("22:59:31.396", -0.042, 0.001, 0.036, 0.205, 0.0),
            ("22:59:32.428", -0.083, 0.083, 0.077, 0.311, 2.5),
            ("22:59:33.428", -0.103, 0.251, 0.392, 2.478, 7.5),
            ("22:59:34.830", -0.134, -0.930, -0.450, -3.169, -27.6,
             ),
        )
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        for (timestamp, feed_forward, heading_fb, cte_fb, tractor_cte,
             heading_deg) in samples:
            command, debug = route._compose_curve_steering(
                feed_forward, heading_fb, cte_fb)
            self.assertAlmostEqual(
                command, feed_forward + heading_fb + cte_fb, places=9,
                                   msg=timestamp)
            self.assertAlmostEqual(
                debug["raw_feedback"], heading_fb + cte_fb, places=9)
            self.assertFalse(debug["curve_sign_projection_active"])
            self.assertNotIn("opposite_correction_authorized", debug)
            self.assertNotIn("opposite_correction_proof_s", debug)
        # These captured values came from the removed three-controller path.
        # Production Route now obtains all three debug terms by decomposing one
        # pursuit solution; this helper cannot clip or authorize it again.
        self.assertFalse(hasattr(route, "_curve_composition_state"))

    def test_composition_preserves_the_complete_geometric_solution(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        reduced, reduced_debug = route._compose_curve_steering(
            -0.240, 0.080, 0.100)
        crossed, crossed_debug = route._compose_curve_steering(
            -0.240, 0.180, 0.270)
        self.assertAlmostEqual(reduced, -0.060)
        self.assertFalse(reduced_debug["curve_sign_projection_active"])
        self.assertAlmostEqual(crossed, 0.210)
        self.assertFalse(crossed_debug["curve_sign_projection_active"])
        self.assertNotIn("opposite_correction_authorized", crossed_debug)

    def test_s_curve_changes_foundation_direction_immediately(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        left, _ = route._compose_curve_steering(
            -0.45, 0.0, 0.0)
        right, debug = route._compose_curve_steering(
            0.45, 0.0, 0.0)
        self.assertLess(left, 0.0)
        self.assertGreater(right, 0.0)
        self.assertEqual(debug["curve_foundation_sign"], 1)
        self.assertNotIn("opposite_correction_proof_s", debug)

    def test_curve_composition_is_history_and_authority_independent(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        expected, _ = route._compose_curve_steering(
            -0.30, 0.10, 0.05)
        for index in range(60):
            sign = -1.0 if index % 2 else 1.0
            route._compose_curve_steering(
                sign * 0.45, -sign * 0.50, sign * 0.30)
        actual, _ = route._compose_curve_steering(
            -0.30, 0.10, 0.05)
        fresh, _ = Route([(0.0, 0.0), (0.0, -20.0)])._compose_curve_steering(
            -0.30, 0.10, 0.05)
        self.assertEqual(actual, expected)
        self.assertEqual(actual, fresh)
        self.assertFalse(hasattr(route, "_curve_composition_state"))
        self.assertFalse(hasattr(route, "_trailer_offset_state"))

    def test_r18_r35_r83_and_roundabout_keep_monotonic_curve_direction(self):
        for direction in (-1.0, 1.0):
            for radius, length in ((18.0, 100.0), (35.0, 140.0),
                                   (83.0, 180.0), (18.0, 210.0)):
                route = Route(self._arc(direction, radius, length))
                commands = []
                for index in range(5, min(len(route.points) - 3, 45), 3):
                    position = route.points[index]
                    heading = self._heading(position, route.points[index + 2])
                    command = route.steering(
                        position, heading, 8.0,
                        cross_track_error_m=0.0, control_dt_s=0.05)
                    commands.append((command, route.last_steering_debug[
                        "feed_forward"]))
                with self.subTest(direction=direction, radius=radius,
                                  length=length):
                    self.assertTrue(commands)
                    self.assertTrue(all(command * reference > 0.0
                                        for command, reference in commands))

    def test_r18_r35_r83_closed_loop_cte_does_not_grow_cycle_by_cycle(self):
        limits = {18.0: 0.60, 35.0: 0.45, 83.0: 0.30}
        for direction in (-1.0, 1.0):
            for radius, cte_limit in limits.items():
                errors, commands, _progress = _simulate(
                    _path_with_curve(direction, radius),
                    math.sqrt(1.6 * radius))
                meaningful = [value for value in commands
                              if abs(value) > 0.05]
                flips = sum(first * second < 0.0 for first, second in zip(
                    meaningful, meaningful[1:]))
                with self.subTest(direction=direction, radius=radius):
                    self.assertLess(max(map(abs, errors)), cte_limit)
                    self.assertEqual(flips, 0)
                    # Entry transients may grow while steering authority is
                    # acquired, but the closed loop must recover from its peak
                    # on the same arc rather than diverging cycle after cycle.
                    peak = max(map(abs, errors))
                    peak_index = max(range(len(errors)),
                                     key=lambda index: abs(errors[index]))
                    self.assertLess(peak_index, len(errors) - 1)
                    self.assertLess(abs(errors[-1]), peak * 0.40)

    def test_curve_to_straight_settles_without_post_exit_hunting(self):
        for direction in (-1.0, 1.0):
            points = _path_with_curve(direction, 35.0)
            curve_end_progress = sum(math.dist(first, second)
                                     for first, second in zip(
                                         points, points[1:]))
            exit_x, exit_z = points[-1]
            before_x, before_z = points[-2]
            dx, dz = exit_x-before_x, exit_z-before_z
            length = math.hypot(dx, dz)
            points.extend((
                exit_x + dx / length * distance,
                exit_z + dz / length * distance,
            ) for distance in range(2, 62, 2))
            errors, commands, progresses = _simulate(
                points, math.sqrt(1.6 * 35.0))
            exit_index = next(index for index, progress in enumerate(progresses)
                              if progress >= curve_end_progress)
            settle_index = next((index for index in range(
                exit_index, len(commands) - 10)
                if max(map(abs, commands[index:index + 10])) < 0.05), None)
            with self.subTest(direction=direction):
                self.assertIsNotNone(settle_index)
                self.assertLessEqual((settle_index-exit_index) * 0.05, 1.5)
                self.assertLess(max(map(abs, errors[exit_index:])), 0.40)

    def test_trailer_side_is_spatial_and_measured_cte_is_diagnostic_only(self):
        route = Route(self._arc(1.0, 18.0, 140.0))
        tractor = route.points[30]
        heading = self._heading(tractor, route.points[32])
        base_trailer = route.points[26]
        trailer_heading = self._heading(base_trailer, route.points[28])
        applied = []
        measured = []
        predicted = []
        for index in range(30):
            # Alternate the measured axle across the centre.  This reproduced
            # the frame-dependent sign source, while map curvature stays one
            # continuous R18 bend.
            lateral = 0.70 if index % 2 else -0.70
            dx = route.points[28][0] - base_trailer[0]
            dz = route.points[28][1] - base_trailer[1]
            length = math.hypot(dx, dz)
            trailer = (base_trailer[0] + dz / length * lateral,
                       base_trailer[1] - dx / length * lateral)
            route.steering(
                tractor, heading, 5.5, cross_track_error_m=0.0,
                vehicle_envelope={
                    "attached": True, "position": trailer,
                    "heading": trailer_heading, "lane_width_m": 4.7,
                    "tractor_altitude_m": 0.0,
                    "trailer_altitude_m": 0.0,
                }, control_dt_s=0.05)
            debug = route.last_steering_debug["trailer_envelope"]
            self.assertTrue(debug["curve_side_proven"])
            applied.append(debug["applied_offset_m"])
            measured.append(debug["measured_offtrack_m"])
            predicted.append(debug["predicted_offtrack_m"])
            self.assertAlmostEqual(
                debug["required_offset_m"],
                debug["predicted_offtrack_m"]
                * TRAILER_BALANCED_REFERENCE_FRACTION,
                places=9)
        nonzero_signs = {1 if value > 0.0 else -1 for value in applied
                         if abs(value) > 1e-6}
        self.assertEqual(len(nonzero_signs), 1)
        self.assertGreater(max(measured) - min(measured), 1.0)
        self.assertLess(max(predicted) - min(predicted), 1e-9)
        self.assertLess(max(applied) - min(applied), 1e-9)

    def test_composition_has_no_noise_driven_authority_state(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        outputs = []
        for index in range(80):
            noise = 1.0 if index % 2 else -1.0
            command, debug = route._compose_curve_steering(
                0.40, -0.34 * noise, -0.20 * noise)
            outputs.append(command)
        self.assertTrue(all(abs(value - 0.94) < 1e-12
                            for value in outputs[::2]))
        self.assertTrue(all(abs(value + 0.14) < 1e-12
                            for value in outputs[1::2]))
        self.assertFalse(debug["curve_sign_projection_active"])
        self.assertNotIn("opposite_correction_authorized", debug)

    def test_lane_tangent_is_scoped_to_lane_id_revision_and_deck(self):
        lane_a = (101, 1, 0, "prefab-a", 0, ())
        lane_b = (202, 1, 0, "prefab-b", 1, ())
        points = [(0.0, 0.0), (0.0, -5.0), (0.0, -10.0),
                  (5.0, -10.0), (10.0, -10.0)]
        authorities = [
            (lane_a, 0), (lane_a, 0), (lane_a, 0),
            (lane_b, 1), (lane_b, 1),
        ]
        route = Route(points, point_authorities=authorities)
        command = route.steering(
            (0.0, -8.0), 0.0, 5.0, cross_track_error_m=0.0,
            control_authority={
                "lane_identity": lane_a, "elevation_layer": 0,
                "lane_width_m": 4.2, "revision": 9,
            }, control_dt_s=0.05)
        debug = route.last_steering_debug
        self.assertTrue(debug["authority_valid"])
        self.assertEqual(debug["authority_lane_id"], lane_a)
        self.assertEqual(debug["authority_revision"], 9)
        self.assertLess(abs(debug["guidance_heading_error_rad"]),
                        math.radians(1.0))
        self.assertLess(abs(command), 0.05)


if __name__ == "__main__":
    unittest.main()
