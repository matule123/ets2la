"""Deterministic 20 Hz closed-loop replays for demanding road geometry."""

import math
import unittest

from core.navigation.route import (
    NORMALIZED_STEERING_ANGLE_RAD, TRUCK_WHEELBASE_M, Route,
    curve_speed_limit_ms,
)
from plugins.autopilot.main import Plugin as AutopilotPlugin


def _path_with_curve(direction, radius):
    points = [(0.0, float(z)) for z in range(0, 81, 2)]
    sample_count = int(radius * math.pi * 0.5 / 2.0)
    for index in range(1, sample_count + 1):
        angle = index * 2.0 / radius
        points.append((
            direction * (radius - radius * math.cos(angle)),
            80.0 + radius * math.sin(angle),
        ))
    return points


def _compound_palisade_path():
    """Straight, left bend, short recovery and opposing tight bend."""
    x = z = 0.0
    heading = math.pi
    points = [(x, z)]
    sections = ((0.0, 60.0), (1.0 / 35.0, 38.0),
                (0.0, 12.0), (-1.0 / 30.0, 36.0), (0.0, 35.0))
    for curvature, length in sections:
        for _ in range(int(length / 2.0)):
            heading -= curvature * 2.0
            x += -math.sin(heading) * 2.0
            z += -math.cos(heading) * 2.0
            points.append((x, z))
    return points


def _simulate(points, speed_ms):
    route = Route(points)
    autopilot = AutopilotPlugin.__new__(AutopilotPlugin)
    autopilot._last_steering = 0.0
    x, z = route.points[0]
    heading = math.atan2(
        -(route.points[2][0] - x), -(route.points[2][1] - z))
    dt = 0.05
    errors, commands, progresses = [], [], []
    duration = (route._cumulative_m[-1] - 8.0) / speed_ms
    for _ in range(int(duration / dt)):
        segment = route.tracking_index((x, z), heading)
        live_cte = route.cross_track_error(segment, (x, z))
        target = route.steering(
            (x, z), heading, speed_ms,
            cross_track_error_m=live_cte)
        autopilot._last_steering = autopilot._ramp_steering(target, dt)
        heading -= (speed_ms / TRUCK_WHEELBASE_M
                    * (autopilot._last_steering
                       * NORMALIZED_STEERING_ANGLE_RAD) * dt)
        x += -math.sin(heading) * speed_ms * dt
        z += -math.cos(heading) * speed_ms * dt
        segment = route.tracking_index((x, z), heading)
        errors.append(route.cross_track_error(segment, (x, z)))
        commands.append(autopilot._last_steering)
        progresses.append(route.tracking_progress((x, z), heading))
    return errors, commands, progresses


class GameLikeControlSimulationTests(unittest.TestCase):
    def test_left_and_right_prefab_exits_stay_inside_lane_without_jerk(self):
        for direction in (-1.0, 1.0):
            for radius in (25.0, 35.0, 45.0):
                with self.subTest(direction=direction, radius=radius):
                    speed = curve_speed_limit_ms(radius, 0.0)
                    errors, commands, progresses = _simulate(
                        _path_with_curve(direction, radius), speed)
                    self.assertLess(max(map(abs, errors)), 0.82)
                    self.assertLessEqual(max(
                        abs(current-previous)
                        for previous, current in zip(commands, commands[1:])),
                        0.031)
                    self.assertEqual(progresses, sorted(progresses))
                    if radius == 35.0:
                        first_turn = next(
                            index for index, command in enumerate(commands)
                            if abs(command) > 0.02)
                        # The bend starts at 80 m. Acquire it progressively
                        # several metres beforehand, but never cut the long
                        # straight as the former far-chord controller did.
                        self.assertGreaterEqual(progresses[first_turn], 70.0)
                        self.assertLess(progresses[first_turn], 78.0)
                        self.assertLess(max(
                            abs(commands[index]) for index, progress in
                            enumerate(progresses) if progress < 70.0), 0.01)
                        self.assertLess(max(
                            abs(errors[index]) for index, progress in
                            enumerate(progresses)
                            if 78.0 <= progress <= 82.0), 0.35)

    def test_compound_palisade_replay_unwinds_and_stays_lane_centred(self):
        errors, commands, progresses = _simulate(
            _compound_palisade_path(), curve_speed_limit_ms(30.0, 0.0))
        self.assertLess(max(map(abs, errors)), 1.20)
        self.assertTrue(any(value > 0.05 for value in commands))
        self.assertTrue(any(value < -0.05 for value in commands))
        self.assertEqual(progresses, sorted(progresses))
        self.assertLessEqual(max(
            abs(current-previous)
            for previous, current in zip(commands, commands[1:])), 0.091)

    def test_roundabout_survives_localization_chatter_and_wheel_lag(self):
        """Replay the 15:07 R18 failure with noisy CTE and a slow game wheel."""
        dt, radius = 0.05, 18.0
        speed = curve_speed_limit_ms(radius, 0.0)
        for direction in (-1.0, 1.0):
            points = _path_with_curve(direction, radius)
            exit_x, exit_z = points[-1]
            approach_x, approach_z = points[-2]
            tangent_x, tangent_z = (
                exit_x - approach_x, exit_z - approach_z)
            tangent_length = math.hypot(tangent_x, tangent_z)
            tangent_x /= tangent_length
            tangent_z /= tangent_length
            # Include 40 m of the receiving road.  The real failure happened
            # while leaving the prefab, so stopping the replay at the final
            # arc point would measure an end-of-path artefact instead of the
            # controller's ability to settle into the outgoing lane.
            points.extend((
                exit_x + tangent_x * distance,
                exit_z + tangent_z * distance,
            ) for distance in range(2, 42, 2))
            route = Route(points)
            autopilot = AutopilotPlugin.__new__(AutopilotPlugin)
            autopilot._last_steering = 0.0
            autopilot._filtered_nav_steering = 0.0
            autopilot._filtered_nav_revision = None
            x, z = route.points[0]
            heading = math.atan2(
                -(route.points[2][0] - x),
                -(route.points[2][1] - z))
            physical_wheel = 0.0
            errors, commands = [], []
            duration = (route._cumulative_m[-1] - 4.0) / speed
            for frame in range(int(duration / dt)):
                segment = route.tracking_index((x, z), heading)
                true_cte = route.cross_track_error(segment, (x, z))
                # The captured LaneMatch moved within roughly a metre while
                # heading residual changed several degrees between log frames.
                measured_cte = (true_cte + 0.45 * math.sin(
                    frame * dt * 2.0 * math.pi / 0.85))
                measured_heading = (heading + math.radians(3.0) * math.sin(
                    frame * dt * 2.0 * math.pi / 1.10))
                raw = route.steering(
                    (x, z), measured_heading, speed,
                    cross_track_error_m=measured_cte)
                filtered = autopilot._smooth_navigation_steering(
                    raw, dt, 10)
                target = 0.72 * filtered + 0.28 * autopilot._last_steering
                autopilot._last_steering = autopilot._ramp_steering(
                    target, dt)
                # ETS steering does not reach a requested wheel angle in one
                # control frame; reproduce a 320 ms first-order response.
                physical_wheel += ((autopilot._last_steering - physical_wheel)
                                   * min(1.0, dt / 0.32))
                heading -= (speed / TRUCK_WHEELBASE_M
                            * physical_wheel
                            * NORMALIZED_STEERING_ANGLE_RAD * dt)
                x += -math.sin(heading) * speed * dt
                z += -math.cos(heading) * speed * dt
                segment = route.tracking_index((x, z), heading)
                errors.append(route.cross_track_error(segment, (x, z)))
                commands.append(autopilot._last_steering)
            with self.subTest(direction=direction):
                self.assertLess(abs(errors[-1]), 0.50)
                # Even with deliberately injected LaneMatch chatter the truck
                # stays within the inner metre of the confirmed centreline,
                # then settles below 0.5 m on the receiving road.
                self.assertLess(max(map(abs, errors)), 0.95)
                self.assertLessEqual(max(
                    abs(current-previous) for previous, current
                    in zip(commands, commands[1:])), 0.031)


if __name__ == "__main__":
    unittest.main()
