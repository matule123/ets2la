import math
import unittest

from core.navigation.route import (
    CURVE_FOUNDATION_RETAIN_FRACTION,
    CURVE_OPPOSITE_PROOF_S,
    TRAILER_OFFSET_RATE_MPS,
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

    def test_real_log_curve_composition_replay_has_no_unproven_reversals(self):
        # Exact one-second diagnostics from 13:46:34--13:47:18.  Logged
        # lane_cte uses LaneMatch's sign; Route consumes its negation.
        samples = (
            # time, curve_reference, heading_fb, cte_fb, lane_cte,
            # guidance_heading_deg
            ("13:46:34.967", -0.382, -0.099, 0.595, -1.363, -3.0),
            ("13:46:35.979", -0.386, -0.458, 0.267, -0.038, -13.6),
            ("13:46:36.995", -0.426, 0.118, 0.404, -0.372, 3.5),
            ("13:46:40.046", -0.624, -0.148, 0.602, -0.733, -4.4),
            ("13:46:42.110", -0.515, -0.093, 0.587, -0.842, -2.8),
            ("13:46:43.120", -0.469, -0.233, 0.483, -0.477, -6.9),
            ("13:47:10.632", -0.397, 0.030, -0.021, -0.142, 0.9),
            ("13:47:11.655", -0.469, 0.170, 0.292, -1.418, 5.0),
            ("13:47:12.671", -0.442, -0.405, 0.489, -0.894, -12.0),
            ("13:47:13.688", -0.439, 0.021, 0.333, -0.440, 0.6),
            ("13:47:14.706", -0.436, -0.057, 0.627, -1.115, -1.7),
            ("13:47:15.726", -0.386, -0.577, 0.112, 0.469, -17.1),
            ("13:47:16.773", -0.400, 0.328, 0.288, -0.127, 9.8),
            ("13:47:17.765", -0.428, 0.616, 0.593, -2.628, 18.3),
            ("13:47:18.800", -0.463, -0.639, 0.845, -2.332, -19.0),
        )
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        route._reset_control_composition((9, "captured-lane"))
        old_reversals = 0
        new_reversals = 0
        limited_times = []
        for (timestamp, feed_forward, heading_fb, cte_fb, lane_cte,
             heading_deg) in samples:
            old = feed_forward + heading_fb + cte_fb
            old_reversals += int(old * feed_forward < 0.0)
            command, debug = route._compose_curve_steering(
                feed_forward, heading_fb, cte_fb, -lane_cte,
                math.radians(heading_deg), True, 4.7, 0.05)
            new_reversals += int(command * feed_forward < 0.0)
            if debug["curve_foundation_limited"]:
                limited_times.append(timestamp)
                self.assertGreaterEqual(
                    abs(command) + 1e-9,
                    abs(feed_forward) * CURVE_FOUNDATION_RETAIN_FRACTION)
        self.assertGreaterEqual(old_reversals, 3)
        self.assertEqual(new_reversals, 0)
        self.assertIn("13:46:34.967", limited_times)
        self.assertIn("13:47:17.765", limited_times)

    def test_sustained_geometric_departure_authorizes_opposite_recovery(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        outputs = []
        for _ in range(12):
            command, debug = route._compose_curve_steering(
                -0.240, 0.180, 0.270, 1.10,
                math.radians(12.0), True, 4.2, 0.05)
            outputs.append(command)
        self.assertGreater(outputs[0] * -0.240, 0.0)
        self.assertGreaterEqual(
            debug["opposite_correction_proof_s"], CURVE_OPPOSITE_PROOF_S)
        self.assertTrue(debug["opposite_correction_authorized"])
        self.assertGreater(outputs[-1], 0.0)

    def test_s_curve_changes_foundation_direction_immediately(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        left, _ = route._compose_curve_steering(
            -0.45, 0.0, 0.0, 0.0, 0.0, True, 4.2, 0.05)
        right, debug = route._compose_curve_steering(
            0.45, 0.0, 0.0, 0.0, 0.0, True, 4.2, 0.05)
        self.assertLess(left, 0.0)
        self.assertGreater(right, 0.0)
        self.assertEqual(debug["curve_foundation_sign"], 1)
        self.assertEqual(debug["opposite_correction_proof_s"], 0.0)

    def test_lane_or_revision_change_cannot_inherit_reversal_proof(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        route._reset_control_composition((9, "lane-a"))
        for _ in range(6):
            _command, debug = route._compose_curve_steering(
                -0.30, 0.25, 0.20, 1.2, math.radians(10.0),
                True, 4.2, 0.05)
        self.assertGreater(debug["opposite_correction_proof_s"], 0.0)
        route._reset_control_composition((10, "lane-b"))
        command, debug = route._compose_curve_steering(
            -0.30, 0.25, 0.20, 1.2, math.radians(10.0),
            True, 4.2, 0.05)
        self.assertFalse(debug["opposite_correction_authorized"])
        self.assertEqual(debug["opposite_correction_proof_s"], 0.05)
        self.assertLess(command, 0.0)

    def test_same_revision_lane_change_preserves_only_physical_trailer_offset(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        lane_a = (101, 1, 0, "road-a", 0, ())
        lane_b = (202, 1, 0, "prefab-b", 0, ())
        route._reset_control_composition((9, (lane_a, 0)))
        route._trailer_offset_state.update({
            "applied_m": 0.825,
            "pending_side": -1,
            "pending_side_s": 0.25,
        })
        route._curve_composition_state["opposite_proof_s"] = 0.30
        route._curve_composition_state.update({
            "coherent_feedback": 0.08,
            "coherent_feedback_rate": 0.02,
            "coherent_feedback_accel": -0.01,
        })

        route._reset_control_composition(
            (9, (lane_b, 0)), preserve_trailer_offset=True,
            preserve_curve_feedback=True)

        self.assertEqual(route._trailer_offset_state["applied_m"], 0.825)
        self.assertEqual(route._trailer_offset_state["pending_side"], 0)
        self.assertEqual(route._trailer_offset_state["pending_side_s"], 0.0)
        self.assertEqual(
            route._curve_composition_state["opposite_proof_s"], 0.0)
        self.assertEqual(
            route._curve_composition_state["coherent_feedback"], 0.08)
        self.assertEqual(
            route._curve_composition_state["feedback_worsening_s"], 0.0)

        route._reset_control_composition((10, (lane_b, 0)))
        self.assertEqual(route._trailer_offset_state["applied_m"], 0.0)
        self.assertEqual(
            route._curve_composition_state["coherent_feedback"], 0.0)

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

    def test_trailer_side_comes_from_spatial_curve_and_is_rate_limited(self):
        route = Route(self._arc(1.0, 18.0, 140.0))
        tractor = route.points[30]
        heading = self._heading(tractor, route.points[32])
        base_trailer = route.points[26]
        trailer_heading = self._heading(base_trailer, route.points[28])
        applied = []
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
        nonzero_signs = {1 if value > 0.0 else -1 for value in applied
                         if abs(value) > 1e-6}
        self.assertEqual(len(nonzero_signs), 1)
        self.assertLessEqual(max(abs(second - first)
                                 for first, second in zip(applied, applied[1:])),
                             TRAILER_OFFSET_RATE_MPS * 0.05 + 1e-9)

    def test_cte_heading_noise_cannot_authorize_curve_reversal(self):
        route = Route([(0.0, 0.0), (0.0, -20.0)])
        outputs = []
        for index in range(80):
            noise = 1.0 if index % 2 else -1.0
            command, debug = route._compose_curve_steering(
                0.40, -0.34 * noise, -0.20 * noise,
                0.28 * noise, math.radians(3.0 * noise),
                True, 4.2, 0.05)
            outputs.append(command)
        self.assertTrue(all(command > 0.0 for command in outputs))
        self.assertFalse(debug["opposite_correction_authorized"])

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
