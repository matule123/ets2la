"""Deterministic Phase 4 regressions for physical steering dynamics.

The tests deliberately keep the Phase 3 LanePath immutable.  They exercise
the geometric command already produced by :class:`Route` and the independent
angle/rate/acceleration actuator that follows it.
"""

from __future__ import annotations

import math
import statistics
import time
import unittest

from core.navigation.route import (
    NORMALIZED_STEERING_ANGLE_RAD,
    TRUCK_WHEELBASE_M,
    Route,
    curve_speed_limit_ms,
)
from core.sdk.scs_sdk import SCSTelemetry
from core.steering_dynamics import (
    STEERING_DYNAMICS_MAX_DT_S,
    SteeringDynamics,
)
from plugins.autopilot.main import lane_authority_rejection_reason
from tests.test_control_safety_regressions import (
    autopilot,
    ready_navigation_state,
)


def _sign_changes(values, threshold=0.01):
    signs = []
    for value in values:
        if abs(value) <= threshold:
            continue
        sign = 1 if value > 0.0 else -1
        if not signs or signs[-1] != sign:
            signs.append(sign)
    return max(0, len(signs) - 1)


def _derivative_metrics(values, dts):
    rates = [
        (values[index] - values[index - 1]) / dts[index]
        for index in range(1, len(values))
    ]
    accelerations = [
        (rates[index] - rates[index - 1]) / dts[index + 1]
        for index in range(1, len(rates))
    ]
    return {
        "max_step": max((abs(b - a) for a, b in zip(values, values[1:])),
                        default=0.0),
        "max_rate": max(map(abs, rates), default=0.0),
        "max_acceleration": max(map(abs, accelerations), default=0.0),
        "sign_changes": _sign_changes(values),
    }


def _old_first_order_profile(targets, dts, rate_per_s=0.60):
    """Exact pre-Phase-4 steering limiter, retained only as a baseline."""
    output = 0.0
    outputs = []
    for target, dt in zip(targets, dts):
        step = rate_per_s * max(dt, 0.001)
        if output * target < 0.0:
            output = (max(0.0, output - step) if output > 0.0
                      else min(0.0, output + step))
        else:
            output += max(-step, min(step, target - output))
        outputs.append(output)
    return outputs


class _Phase4CriticalSettlingDynamics(SteeringDynamics):
    """Superseded Phase-4 settling law used only as a regression baseline."""

    def update(self, target, dt, *, speed_ms=0.0, curvature_per_m=0.0):
        raw_target = float(target)
        used_dt = max(0.001, min(STEERING_DYNAMICS_MAX_DT_S, float(dt)))
        command_demand = max(abs(raw_target), abs(self.command))
        authority = self._authority_fraction(
            curvature_per_m, command_demand)
        max_command, max_rate, max_accel = self.limits(
            speed_ms, curvature_per_m, command_demand)
        bounded_target = max(-max_command, min(max_command, raw_target))
        speed_fraction = max(0.0, min(
            1.0, (abs(float(speed_ms)) - 5.0) / 20.0))
        straight_settling = 0.33 + (0.50 - 0.33) * speed_fraction
        curve_settling = 0.17 + (0.30 - 0.17) * speed_fraction
        settling_time = (straight_settling
                         + (curve_settling - straight_settling) * authority)
        natural_frequency = 4.0 / settling_time
        error = bounded_target - self.command
        previous_rate = self.rate
        desired_acceleration = (
            natural_frequency * natural_frequency * error
            - 2.0 * natural_frequency * previous_rate)
        acceleration = max(-max_accel, min(max_accel,
                                            desired_acceleration))
        unrestricted_rate = previous_rate + acceleration * used_dt
        new_rate = max(-max_rate, min(max_rate, unrestricted_rate))
        acceleration = (new_rate - previous_rate) / used_dt
        self.command = max(-max_command, min(
            max_command,
            self.command + 0.5 * (previous_rate + new_rate) * used_dt))
        self.rate = new_rate
        self.last_debug = {
            **self._empty_debug(self.command),
            "raw_target": raw_target,
            "bounded_target": bounded_target,
            "output": self.command,
            "rate_per_s": self.rate,
            "acceleration_per_s2": acceleration,
            "dt_s": float(dt),
            "dt_used_s": used_dt,
            "speed_ms": abs(float(speed_ms)),
            "max_command": max_command,
            "max_rate_per_s": max_rate,
            "max_acceleration_per_s2": max_accel,
        }
        return self.command


def _arc_path(direction, radius_m, sweep_degrees=90.0, exit_m=60.0):
    points = [(0.0, float(z)) for z in range(0, 81, 2)]
    sweep = math.radians(sweep_degrees)
    sample_count = max(2, int(radius_m * sweep / 2.0))
    for index in range(1, sample_count + 1):
        angle = min(sweep, index * sweep / sample_count)
        points.append((
            direction * (radius_m - radius_m * math.cos(angle)),
            80.0 + radius_m * math.sin(angle),
        ))
    end_x, end_z = points[-1]
    previous_x, previous_z = points[-2]
    tangent_x = end_x - previous_x
    tangent_z = end_z - previous_z
    length = math.hypot(tangent_x, tangent_z)
    tangent_x /= length
    tangent_z /= length
    points.extend((
        end_x + tangent_x * distance,
        end_z + tangent_z * distance,
    ) for distance in range(2, int(exit_m) + 2, 2))
    return points


def _straight_path(length_m=420.0):
    return [(0.0, float(z)) for z in range(0, int(length_m) + 2, 2)]


_CAPTURED_CURVE_EXIT_TARGETS = (
    -0.027, -0.009, -0.008, 0.011, 0.030, -0.029, -0.084,
    0.091, 0.118, -0.158, 0.170, -0.160, 0.160, -0.281,
    0.325, -0.372, 0.068,
)


def _replay_captured_curve_exit(dynamics_factory):
    dynamics = dynamics_factory(-0.031)
    targets = []
    outputs = []
    # The log samples at 1 Hz; interpolate only between those measured values
    # at the observed 24--37 ms control cadence. This does not invent another
    # controller signal or feed the result back into geometry.
    dt = 0.03
    for start, end in zip(
            _CAPTURED_CURVE_EXIT_TARGETS,
            _CAPTURED_CURVE_EXIT_TARGETS[1:]):
        for tick in range(33):
            fraction = (tick + 1) / 33.0
            target = start + (end - start) * fraction
            targets.append(target)
            outputs.append(dynamics.update(
                target, dt, speed_ms=16.67, curvature_per_m=0.0))
    sign_disagreements = sum(
        target * output < 0.0 and abs(target) > 0.02
        for target, output in zip(targets, outputs))
    return {
        "mean_tracking_error": statistics.fmean(
            abs(target - output)
            for target, output in zip(targets, outputs)),
        "sign_disagreements": sign_disagreements,
    }


def _lane_change_path(direction, length_m=70.0, lane_width_m=3.6):
    points = [(0.0, float(z)) for z in range(0, 41, 2)]
    samples = int(length_m / 2.0)
    for index in range(1, samples + 1):
        u = index / samples
        # Quintic smoothstep: zero lateral velocity and acceleration at both
        # proven lane-segment boundaries.
        blend = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        points.append((direction * lane_width_m * blend, 40.0 + index * 2.0))
    end_z = points[-1][1]
    points.extend((direction * lane_width_m, end_z + distance)
                  for distance in range(2, 52, 2))
    return points


def _s_curve_path():
    x = z = 0.0
    heading = math.pi
    points = [(x, z)]
    sections = (
        (0.0, 60.0), (1.0 / 45.0, 48.0), (0.0, 14.0),
        (-1.0 / 45.0, 48.0), (0.0, 60.0),
    )
    for curvature, length_m in sections:
        for _ in range(int(length_m / 2.0)):
            heading -= curvature * 2.0
            x += -math.sin(heading) * 2.0
            z += -math.cos(heading) * 2.0
            points.append((x, z))
    return points


def _simulate_route(points, speed_ms, *, wheel_response_s=0.32,
                    noisy_localization=False,
                    dynamics_factory=SteeringDynamics,
                    initial_lateral_m=0.0,
                    initial_heading_error_deg=0.0):
    route = Route(points)
    dynamics = dynamics_factory()
    x, z = route.points[0]
    x += float(initial_lateral_m)
    heading = math.atan2(
        -(route.points[2][0] - x), -(route.points[2][1] - z))
    heading += math.radians(float(initial_heading_error_deg))
    dt = 0.05
    physical_wheel = 0.0
    ctes = []
    heading_errors = []
    raw_commands = []
    commands = []
    rates = []
    accelerations = []
    progresses = []
    duration = (route._cumulative_m[-1] - 4.0) / speed_ms
    for frame in range(int(duration / dt)):
        segment = route.tracking_index((x, z), heading)
        true_cte = route.cross_track_error(segment, (x, z))
        measured_cte = true_cte
        measured_heading = heading
        if noisy_localization:
            measured_cte += 0.10 * math.sin(frame * dt * 2.0 * math.pi / 0.9)
            measured_heading += math.radians(0.8) * math.sin(
                frame * dt * 2.0 * math.pi / 1.3)
        raw = route.steering(
            (x, z), measured_heading, speed_ms,
            cross_track_error_m=measured_cte)
        command = dynamics.update(
            raw, dt, speed_ms=speed_ms,
            curvature_per_m=route.last_steering_debug.get(
                "local_curvature", 0.0))
        physical_wheel += ((command - physical_wheel)
                           * min(1.0, dt / wheel_response_s))
        heading -= (speed_ms / TRUCK_WHEELBASE_M
                    * physical_wheel
                    * NORMALIZED_STEERING_ANGLE_RAD * dt)
        x += -math.sin(heading) * speed_ms * dt
        z += -math.cos(heading) * speed_ms * dt

        segment = route.tracking_index((x, z), heading)
        next_index = min(segment + 1, len(route.points) - 1)
        path_dx = route.points[next_index][0] - route.points[segment][0]
        path_dz = route.points[next_index][1] - route.points[segment][1]
        path_heading = math.atan2(-path_dx, -path_dz)
        heading_error = ((heading - path_heading + math.pi)
                         % (2.0 * math.pi) - math.pi)
        ctes.append(route.cross_track_error(segment, (x, z)))
        heading_errors.append(heading_error)
        raw_commands.append(raw)
        commands.append(command)
        rates.append(dynamics.last_debug["rate_per_s"])
        accelerations.append(dynamics.last_debug["acceleration_per_s2"])
        progresses.append(route.tracking_progress((x, z), heading))

    raw_start = next((index for index, value in enumerate(raw_commands)
                      if abs(value) > 0.02), None)
    output_start = next((index for index, value in enumerate(commands)
                         if abs(value) > 0.02), None)
    reaction_delay = (0.0 if raw_start is None or output_start is None
                      else (output_start - raw_start) * dt)
    return {
        "rms_cte_m": math.sqrt(statistics.fmean(value * value for value in ctes)),
        "max_cte_m": max(map(abs, ctes), default=0.0),
        "final_cte_m": ctes[-1],
        "max_heading_error_deg": math.degrees(
            max(map(abs, heading_errors), default=0.0)),
        "final_heading_error_deg": math.degrees(heading_errors[-1]),
        "max_step": max((abs(b - a) for a, b in zip(commands, commands[1:])),
                        default=0.0),
        "max_rate": max(map(abs, rates), default=0.0),
        "max_acceleration": max(map(abs, accelerations), default=0.0),
        # Ignore sub-0.02 normalized settling corrections; this metric counts
        # direction changes large enough to be visible at the road wheel.
        "sign_changes": _sign_changes(commands, threshold=0.02),
        "reaction_delay_s": reaction_delay,
        "progress_monotonic": progresses == sorted(progresses),
    }


class Phase4SteeringDynamicsTests(unittest.TestCase):
    def test_reproduced_first_order_root_cause_has_unbounded_rate_reversal(self):
        dt = 0.01
        dts = [dt] * 240
        targets = ([0.45] * 80) + ([-0.45] * 100) + ([0.0] * 60)
        old_outputs = _old_first_order_profile(targets, dts)
        old_metrics = _derivative_metrics(old_outputs, dts)

        dynamics = SteeringDynamics()
        new_outputs = [
            dynamics.update(target, tick, speed_ms=8.0,
                            curvature_per_m=1.0 / 45.0)
            for target, tick in zip(targets, dts)
        ]
        new_metrics = _derivative_metrics(new_outputs, dts)

        # The old +/-0.60/s rate flips in one 10 ms tick: 120/s2.  Phase 4
        # preserves the rate envelope but gives that flip a finite ramp.
        self.assertGreaterEqual(old_metrics["max_acceleration"], 119.0)
        self.assertLessEqual(new_metrics["max_acceleration"], 14.01)
        self.assertLessEqual(new_metrics["max_rate"], 0.601)
        self.assertLessEqual(new_metrics["max_step"], 0.0061)

    def test_straight_noise_deadband_does_not_hide_real_curve_demand(self):
        dt = 0.01
        noise = [0.0035 * math.sin(index * dt * 2.0 * math.pi * 3.0)
                 for index in range(300)]
        old = _old_first_order_profile(noise, [dt] * len(noise))
        dynamics = SteeringDynamics()
        quiet = [dynamics.update(value, dt, speed_ms=20.0,
                                 curvature_per_m=0.0) for value in noise]
        self.assertGreater(_sign_changes(old, threshold=0.001), 10)
        self.assertEqual(_sign_changes(quiet, threshold=0.001), 0)
        self.assertLessEqual(max(map(abs, quiet)), 1e-12)

        response = [dynamics.update(0.30, dt, speed_ms=20.0,
                                    curvature_per_m=1.0 / 45.0)
                    for _ in range(50)]
        self.assertGreater(response[-1], 0.15)
        self.assertFalse(dynamics.last_debug["deadband_active"])

    def test_speed_scheduled_angle_rate_and_acceleration_are_monotonic(self):
        speeds = (0.5, 5.0, 10.0, 15.0, 20.0, 25.0)
        limits = [SteeringDynamics.limits(
            speed, 1.0 / 30.0, 0.5) for speed in speeds]
        for column in range(3):
            values = [limit[column] for limit in limits]
            self.assertEqual(values, sorted(values, reverse=True))
            self.assertGreater(values[0], values[-1])
        self.assertAlmostEqual(limits[0][0], 1.0)
        self.assertAlmostEqual(limits[-1][0], 0.55)

    def test_left_and_right_radii_keep_lane_with_bounded_dynamics(self):
        for radius in (18.0, 30.0, 45.0, 80.0, 120.0, 220.0):
            speed = min(15.0, curve_speed_limit_ms(radius, 0.0))
            for direction in (-1.0, 1.0):
                metrics = _simulate_route(
                    _arc_path(direction, radius), speed)
                with self.subTest(radius=radius, direction=direction):
                    self.assertLess(metrics["max_cte_m"], 0.65)
                    self.assertLess(metrics["rms_cte_m"], 0.30)
                    self.assertLess(abs(metrics["final_cte_m"]), 0.12)
                    self.assertLess(
                        abs(metrics["final_heading_error_deg"]), 1.5)
                    self.assertLessEqual(metrics["max_rate"], 0.601)
                    self.assertLessEqual(metrics["max_acceleration"], 14.01)
                    self.assertLessEqual(metrics["reaction_delay_s"], 0.151)
                    self.assertTrue(metrics["progress_monotonic"])

    def test_real_curve_exit_phase_lag_settles_on_the_following_straight(self):
        # 12:45:44--12:46:00 capture: the R83 exit reaches 60 km/h, then the
        # old target/output pair alternates at 0.5--0.8 Hz.  Phase 4E removed
        # the Route-level coherent-feedback lag that was previously folded
        # into the historical 0.85 s "effective" response.  Modelling that
        # removed controller delay again would double-count the root cause, so
        # this replay now uses the independently observed 0.32 s game wheel.
        for direction in (-1.0, 1.0):
            points = _arc_path(direction, 83.0, 90.0, 300.0)
            repaired = _simulate_route(
                points, 16.67, wheel_response_s=0.32)
            with self.subTest(direction=direction):
                self.assertLessEqual(repaired["reaction_delay_s"], 0.05)
                self.assertLess(repaired["max_cte_m"], 0.70)
                self.assertLess(repaired["rms_cte_m"], 0.25)
                self.assertLess(abs(repaired["final_cte_m"]), 0.08)
                self.assertLess(repaired["max_heading_error_deg"], 4.5)
                self.assertLessEqual(repaired["sign_changes"], 2)

    def test_captured_targets_do_not_retain_the_previous_half_cycle(self):
        old = _replay_captured_curve_exit(
            _Phase4CriticalSettlingDynamics)
        repaired = _replay_captured_curve_exit(SteeringDynamics)
        self.assertGreater(old["mean_tracking_error"], 0.035)
        self.assertLess(repaired["mean_tracking_error"], 0.030)
        self.assertGreaterEqual(old["sign_disagreements"], 35)
        self.assertLessEqual(repaired["sign_disagreements"], 25)

    def test_high_speed_straight_small_error_converges_without_hunting(self):
        for lateral, heading in ((0.20, 1.2), (-0.20, -1.2)):
            metrics = _simulate_route(
                _straight_path(), 16.67, wheel_response_s=0.85,
                initial_lateral_m=lateral,
                initial_heading_error_deg=heading)
            with self.subTest(lateral=lateral, heading=heading):
                self.assertLess(metrics["max_cte_m"], 0.45)
                self.assertLess(metrics["rms_cte_m"], 0.12)
                self.assertLess(abs(metrics["final_cte_m"]), 0.05)
                self.assertLess(
                    abs(metrics["final_heading_error_deg"]), 0.20)
                self.assertLessEqual(metrics["sign_changes"], 6)

    def test_s_curve_reversal_is_continuous_and_settles(self):
        metrics = _simulate_route(_s_curve_path(), 8.5)
        self.assertLess(metrics["max_cte_m"], 0.80)
        self.assertLess(abs(metrics["final_cte_m"]), 0.15)
        self.assertLessEqual(metrics["sign_changes"], 2)
        self.assertLessEqual(metrics["max_step"], 0.031)
        self.assertLessEqual(metrics["max_acceleration"], 14.01)

    def test_roundabout_and_ninety_degree_prefab_bends(self):
        scenarios = (
            ("ninety", 30.0, 90.0),
            ("roundabout", 22.0, 270.0),
        )
        for name, radius, sweep in scenarios:
            for direction in (-1.0, 1.0):
                metrics = _simulate_route(
                    _arc_path(direction, radius, sweep),
                    curve_speed_limit_ms(radius, 0.0),
                    noisy_localization=True)
                with self.subTest(name=name, direction=direction):
                    self.assertLess(metrics["max_cte_m"], 0.85)
                    self.assertLess(abs(metrics["final_cte_m"]), 0.20)
                    self.assertLessEqual(metrics["max_step"], 0.031)

    def test_merge_split_and_lane_change_keep_smooth_segment_boundary(self):
        for direction in (-1.0, 1.0):
            metrics = _simulate_route(_lane_change_path(direction), 12.0)
            with self.subTest(direction=direction):
                self.assertLess(metrics["max_cte_m"], 0.35)
                self.assertLess(abs(metrics["final_cte_m"]), 0.10)
                self.assertLessEqual(metrics["sign_changes"], 2)
                self.assertLessEqual(metrics["max_step"], 0.031)
                self.assertTrue(metrics["progress_monotonic"])

    def test_irregular_dt_drop_and_short_lag_cannot_create_large_jump(self):
        dynamics = SteeringDynamics()
        for _ in range(20):
            dynamics.update(0.70, 0.01, speed_ms=15.0,
                            curvature_per_m=1.0 / 45.0)
        before_lag = dynamics.command
        after_lag = dynamics.update(
            0.70, 0.35, speed_ms=15.0, curvature_per_m=1.0 / 45.0)
        debug = dynamics.last_debug
        self.assertTrue(debug["dt_limited"])
        self.assertAlmostEqual(debug["dt_used_s"], STEERING_DYNAMICS_MAX_DT_S)
        self.assertLessEqual(
            abs(after_lag - before_lag),
            debug["max_rate_per_s"] * STEERING_DYNAMICS_MAX_DT_S + 1e-9)
        for dt in (0.004, 0.018, 0.007, 0.026, 0.011):
            previous = dynamics.command
            current = dynamics.update(
                -0.40, dt, speed_ms=15.0, curvature_per_m=1.0 / 45.0)
            self.assertLessEqual(
                abs(current - previous),
                dynamics.last_debug["max_rate_per_s"] * dt + 1e-9)

    def test_engagement_takes_over_measured_game_wheel_without_zero_jump(self):
        state = ready_navigation_state(
            nav_active=True, nav_steering=0.55,
            nav_trajectory_revision=7,
            path_curvature_radius=45.0,
            path_curve_distance_m=0.0)
        plugin = autopilot(
            {"speed": 8.0, "gear": 4, "gameSteer": -0.25}, state)
        plugin.on_tick(0.01)
        self.assertGreater(plugin.sdk.controller.steering, 0.25)
        self.assertLess(plugin.sdk.controller.steering, 0.2561)
        self.assertAlmostEqual(
            plugin._steering_dynamics_debug["dt_used_s"], 0.01)

        state.set("autopilot_active", False)
        plugin.sdk.telemetry.truck["gameSteer"] = -0.18
        plugin.on_tick(0.01)
        self.assertEqual(plugin.sdk.controller.steering, 0.0)
        self.assertAlmostEqual(plugin._steering_dynamics.command, 0.18)

    def test_scs_abi_publishes_manual_and_game_wheel_positions(self):
        reader = SCSTelemetry.__new__(SCSTelemetry)
        reader.mm = object()
        reader.read_bool = lambda offset: (False, offset + 1)
        reader.read_long_long = lambda offset: (0, offset + 8)
        reader.read_int = lambda offset: (0, offset + 4)
        reader.read_float = lambda offset: ({
            956: 0.17, 972: -0.23,
        }.get(offset, 0.0), offset + 4)
        reader.read_double = lambda offset: (0.0, offset + 8)
        snapshot = reader.update()
        self.assertAlmostEqual(snapshot["truckFloat"]["userSteer"], 0.17)
        self.assertAlmostEqual(snapshot["truckFloat"]["gameSteer"], -0.23)

    def test_diagnostics_are_rate_limited_and_include_phase4_terms(self):
        state = ready_navigation_state(
            nav_active=True, nav_steering=0.25,
            nav_trajectory_revision=7,
            nav_steering_debug={
                "feed_forward": 0.20, "feedback": 0.05,
                "heading_feedback": 0.02, "cte_feedback": 0.03,
                "local_curvature": 1.0 / 80.0,
                "guidance_curvature": 1.0 / 80.0,
                "guidance_lookahead_m": 12.0,
            },
            navigation_intent_id="phase4-intent",
            path_curvature_radius=80.0,
            path_curve_distance_m=0.0)
        state.get("lane_trajectory").update({
            "navigation_intent_id": "phase4-intent",
            "request_id": "phase4-intent",
        })
        state.set("nav_recalc_request", "phase4-intent")
        plugin = autopilot(
            {"speed": 8.0, "gear": 4, "gameSteer": 0.0}, state)
        plugin._was_active = True
        plugin._lane_lock_acquired = True
        for _ in range(19):
            plugin.on_tick(0.05)
        self.assertIsNone(state.get("steering_dynamics_diagnostic"))
        plugin.on_tick(0.06)
        first = state.get("steering_dynamics_diagnostic")
        self.assertIsNotNone(first)
        for key in (
                "raw_target", "bounded_target", "output", "rate_per_s",
                "acceleration_per_s2", "dt_s", "speed_ms",
                "target_error", "stopping_distance", "safe_rate_per_s",
                "terminal_rate_per_s",
                "trajectory_phase", "observed_game_steering",
                "game_steer_tracking_error",
                "feed_forward", "heading_feedback", "cte_feedback",
                "lane_cte_m", "lane_heading_deg", "curvature_per_m",
                "lookahead_m", "navigation_intent_id", "revision"):
            self.assertIn(key, first)
        for _ in range(10):
            plugin.on_tick(0.05)
        self.assertIs(state.get("steering_dynamics_diagnostic"), first)

    def test_authority_loss_unwinds_and_stale_steering_revision_fails_closed(self):
        state = ready_navigation_state(
            nav_active=True, nav_steering=0.45,
            nav_trajectory_revision=7,
            path_curvature_radius=45.0,
            path_curve_distance_m=0.0)
        plugin = autopilot(
            {"speed": 8.0, "gear": 4, "gameSteer": 0.0}, state)
        plugin._was_active = True
        plugin._lane_lock_acquired = True
        for _ in range(8):
            plugin.on_tick(0.05)
        before_loss = plugin.sdk.controller.steering
        self.assertGreater(before_loss, 0.10)

        state.set("lane_trajectory_revision", 8)
        release = []
        for _ in range(10):
            plugin.on_tick(0.05)
            release.append(plugin.sdk.controller.steering)
        # Existing positive wheel velocity is first decelerated instead of
        # being snapped. The angle may grow for one frame, then releases.
        self.assertGreater(release[0], 0.0)
        self.assertLess(release[-1], before_loss)
        self.assertLess(plugin._steering_dynamics_debug["rate_per_s"], 0.0)
        self.assertGreater(plugin.sdk.controller.brake, 0.0)

        state.set("lane_trajectory_revision", 7)
        state.set("nav_trajectory_revision", 6)
        reason = lane_authority_rejection_reason(
            state, state.get("lane_trajectory"), now=time.monotonic())
        self.assertIn("steering revision 6 is stale", reason)

    def test_wrong_lane_heading_and_elevation_remain_fail_closed(self):
        state = ready_navigation_state(nav_active=True, nav_trajectory_revision=7)
        snapshot = state.get("lane_trajectory")
        lane = {"road_uid": 90, "direction": 1, "lane_index": 0}
        snapshot["active_lane_id"] = lane
        snapshot["lane_match"].update({
            "active_lane_id": lane, "elevation_layer": 0,
        })
        wrong = dict(snapshot["lane_match"])
        wrong["active_lane_id"] = {
            "road_uid": 90, "direction": -1, "lane_index": 0,
        }
        state.set("lane_match", wrong)
        self.assertIn("different GPS lane", lane_authority_rejection_reason(
            state, snapshot, now=time.monotonic()))
        wrong["active_lane_id"] = lane
        wrong["heading_error_rad"] = math.radians(170.0)
        self.assertIn("heading differs", lane_authority_rejection_reason(
            state, snapshot, now=time.monotonic()))
        wrong["heading_error_rad"] = 0.0
        wrong["elevation_layer"] = 2
        self.assertIn("elevation layer", lane_authority_rejection_reason(
            state, snapshot, now=time.monotonic()))

    def test_loaded_truck_slow_wheel_response_remains_stable(self):
        for direction in (-1.0, 1.0):
            metrics = _simulate_route(
                _arc_path(direction, 30.0),
                curve_speed_limit_ms(30.0, 0.0),
                wheel_response_s=0.52,
                noisy_localization=True)
            with self.subTest(direction=direction):
                self.assertLess(metrics["max_cte_m"], 0.75)
                self.assertLess(abs(metrics["final_cte_m"]), 0.20)
                self.assertLessEqual(metrics["max_step"], 0.031)


if __name__ == "__main__":
    unittest.main()
