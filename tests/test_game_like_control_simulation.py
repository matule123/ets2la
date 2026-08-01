"""Deterministic 20 Hz closed-loop replays for demanding road geometry."""

import math
import unittest

from core.navigation.route import Route, curve_speed_limit_ms
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
        heading -= (speed_ms / 5.0
                    * (autopilot._last_steering * 0.18) * dt)
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


if __name__ == "__main__":
    unittest.main()
