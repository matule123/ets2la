"""Phase 4E deterministic closed-loop steering-controller regressions.

Unlike the historical log replays, these tests feed every steering result back
into the next measured pose.  The loop runs at a nominal 20 Hz and contains
both the UltraPilot :class:`SteeringDynamics` trajectory and a separate delayed
game actuator.  Vehicle yaw uses the nonlinear bicycle equation
``yaw_rate = v / wheelbase * tan(road_wheel_angle)``.
"""

from __future__ import annotations

from collections import deque
import math
import statistics
import unittest

from core.navigation.route import (
    NORMALIZED_STEERING_ANGLE_RAD,
    TRAILER_BALANCED_REFERENCE_FRACTION,
    TRAILER_EFFECTIVE_AXLE_DISTANCE_M,
    TRUCK_WHEELBASE_M,
    Route,
    curve_speed_limit_ms,
)
from core.steering_dynamics import SteeringDynamics


DT_S = 0.05
GAME_STEERING_RESPONSE_S = 0.32
GAME_COMMAND_DELAY_TICKS = 2

# Real runtime point authorities are ``(LaneId.sort_key(), elevation_layer)``.
# Keep the road and prefab identities distinct while the geometry itself stays
# one mathematically continuous R83 arc.
ROAD_R83_LANE = (624008901, 1, 0, "", -1, ())
PREFAB_R83_LANE = (
    624008902, 1, 0, "phase4e-r83-prefab", 0, (3, 7))
R83_ELEVATION_LAYER = 45


def _append_section(points, pose, curvature_per_m, length_m,
                    sample_step_m=1.0):
    """Append immutable path samples using UltraPilot's heading convention."""
    x, z, heading = pose
    remaining = float(length_m)
    while remaining > 1e-9:
        step = min(float(sample_step_m), remaining)
        heading -= float(curvature_per_m) * step
        x += -math.sin(heading) * step
        z += -math.cos(heading) * step
        points.append((x, z))
        remaining -= step
    return x, z, heading


def _path_from_sections(sections):
    points = [(0.0, 0.0)]
    pose = (0.0, 0.0, math.pi)
    boundaries = []
    progress = 0.0
    for curvature, length_m in sections:
        pose = _append_section(points, pose, curvature, length_m)
        progress += float(length_m)
        boundaries.append(progress)
    return points, tuple(boundaries)


def _constant_curve_path(direction, radius_m, *, sweep_deg=105.0,
                         entry_m=45.0, exit_m=55.0):
    turn_length = float(radius_m) * math.radians(float(sweep_deg))
    return _path_from_sections((
        (0.0, entry_m),
        (float(direction) / float(radius_m), turn_length),
        (0.0, exit_m),
    ))


def _s_curve_path(radius_m=45.0):
    turn_length = float(radius_m) * math.radians(72.0)
    return _path_from_sections((
        (0.0, 45.0),
        (1.0 / radius_m, turn_length),
        (0.0, 10.0),
        (-1.0 / radius_m, turn_length),
        (0.0, 55.0),
    ))


def _heading_between(first, second):
    return math.atan2(-(second[0] - first[0]), -(second[1] - first[1]))


def _wrapped_angle(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _sign_changes(values, threshold=0.02):
    signs = []
    for value in values:
        if abs(value) <= threshold:
            continue
        sign = 1 if value > 0.0 else -1
        if not signs or signs[-1] != sign:
            signs.append(sign)
    return max(0, len(signs) - 1)


def _point_authority_r83_fixture():
    """Return a smooth road->prefab boundary in one continuous R83 bend."""
    points, boundaries = _constant_curve_path(
        1.0, 83.0, entry_m=45.0, exit_m=55.0)
    unscoped = Route(points)
    boundary_progress = boundaries[0] + 35.0
    boundary_index = min(
        range(len(points)),
        key=lambda index: abs(
            unscoped._cumulative_m[index] - boundary_progress))
    authorities = [
        (ROAD_R83_LANE, R83_ELEVATION_LAYER)
        if index <= boundary_index else
        (PREFAB_R83_LANE, R83_ELEVATION_LAYER)
        for index in range(len(points))
    ]
    return points, authorities, boundary_index


def _authority_payload(lane_identity, *, revision, elevation_layer=None):
    return {
        "lane_identity": lane_identity,
        "revision": int(revision),
        "elevation_layer": (
            R83_ELEVATION_LAYER if elevation_layer is None
            else int(elevation_layer)),
        "lane_width_m": 4.7,
    }


def _simulate_closed_loop(
        points, speed_ms, *, noisy_lane_match=False, delayed_ticks=False,
        authority_change_progress_m=None, with_trailer=False,
        initial_lateral_m=0.0, initial_heading_error_deg=0.0):
    """Run command -> actuator -> bicycle pose -> next command at 20 Hz.

    ``SteeringDynamics`` is UltraPilot's physical execution stage.  The queue
    and first-order wheel response represent the independent game-side delay;
    neither is part of the controller being tested.
    """
    route = Route(points)
    dynamics = SteeringDynamics()
    x, z = route.points[0]
    path_heading = _heading_between(route.points[0], route.points[2])
    initial_dx = route.points[2][0] - route.points[0][0]
    initial_dz = route.points[2][1] - route.points[0][1]
    initial_length = math.hypot(initial_dx, initial_dz)
    x += initial_dz / initial_length * float(initial_lateral_m)
    z -= initial_dx / initial_length * float(initial_lateral_m)
    heading = path_heading + math.radians(float(initial_heading_error_deg))
    game_wheel = 0.0
    delay = deque([0.0] * GAME_COMMAND_DELAY_TICKS,
                  maxlen=GAME_COMMAND_DELAY_TICKS + 1)

    trailer_heading = heading
    trailer_x = x + math.sin(heading) * TRAILER_EFFECTIVE_AXLE_DISTANCE_M
    trailer_z = z + math.cos(heading) * TRAILER_EFFECTIVE_AXLE_DISTANCE_M

    samples = []
    elapsed = 0.0
    maximum_time = (route._cumulative_m[-1] + 15.0) / float(speed_ms)
    frame = 0
    while elapsed < maximum_time:
        dt = (0.10 if delayed_ticks and frame > 0 and frame % 47 == 0
              else DT_S)
        progress = route.tracking_progress((x, z), heading)
        if progress >= route._cumulative_m[-1] - 4.0:
            break
        segment = route.tracking_index((x, z), heading)
        true_cte = route.cross_track_error(segment, (x, z))
        measured_cte = true_cte
        measured_heading = heading
        if noisy_lane_match:
            # Deterministic, non-commensurate disturbances prevent a replay
            # from accidentally sampling the same phase every turn.
            measured_cte += (0.08 * math.sin(elapsed * 2.0 * math.pi / 0.83)
                             + 0.025 * math.sin(
                                 elapsed * 2.0 * math.pi / 0.19))
            measured_heading += math.radians(
                0.65 * math.sin(elapsed * 2.0 * math.pi / 1.17))

        lane_name = "road:before"
        revision = 12
        if (authority_change_progress_m is not None
                and progress >= float(authority_change_progress_m)):
            lane_name = "prefab:after"
            revision = 13
        authority = {
            "lane_identity": lane_name,
            "revision": revision,
            "elevation_layer": 45,
            "lane_width_m": 4.7,
        }
        envelope = None
        if with_trailer:
            envelope = {
                "attached": True,
                "position": (trailer_x, trailer_z),
                "heading": trailer_heading,
                "lane_width_m": 4.7,
                "tractor_altitude_m": 45.0,
                "trailer_altitude_m": 45.0,
            }
        raw = route.steering(
            (x, z), measured_heading, speed_ms,
            cross_track_error_m=measured_cte,
            vehicle_envelope=envelope,
            control_authority=authority,
            control_dt_s=dt,
        )
        curvature = float(route.last_steering_debug.get(
            "local_curvature", 0.0))
        output = dynamics.update(
            raw, dt, speed_ms=speed_ms, curvature_per_m=curvature)
        delay.append(output)
        delayed_output = delay.popleft()
        response = min(1.0, dt / GAME_STEERING_RESPONSE_S)
        game_wheel += (delayed_output - game_wheel) * response

        # Nonlinear bicycle plant. Positive road-wheel angle decreases the ETS2
        # heading, which is a right turn in this coordinate convention.
        road_wheel_angle = game_wheel * NORMALIZED_STEERING_ANGLE_RAD
        heading -= (float(speed_ms) / TRUCK_WHEELBASE_M
                    * math.tan(road_wheel_angle) * dt)
        x += -math.sin(heading) * float(speed_ms) * dt
        z += -math.cos(heading) * float(speed_ms) * dt

        if with_trailer:
            articulation = _wrapped_angle(heading - trailer_heading)
            trailer_heading += (
                float(speed_ms) / TRAILER_EFFECTIVE_AXLE_DISTANCE_M
                * math.sin(articulation) * dt)
            trailer_speed = float(speed_ms) * math.cos(articulation)
            trailer_x += -math.sin(trailer_heading) * trailer_speed * dt
            trailer_z += -math.cos(trailer_heading) * trailer_speed * dt

        new_segment = route.tracking_index((x, z), heading)
        new_progress = route.tracking_progress((x, z), heading)
        next_index = min(new_segment + 1, len(route.points) - 1)
        path_heading = _heading_between(
            route.points[new_segment], route.points[next_index])
        samples.append({
            "time_s": elapsed,
            "dt_s": dt,
            "progress_m": new_progress,
            "cte_m": route.cross_track_error(new_segment, (x, z)),
            "heading_error_rad": _wrapped_angle(heading - path_heading),
            "curvature_per_m": curvature,
            "raw": raw,
            "output": output,
            "game_wheel": game_wheel,
            "rate": float(dynamics.last_debug["rate_per_s"]),
            "acceleration": float(
                dynamics.last_debug["acceleration_per_s2"]),
            "lane": lane_name,
            "revision": revision,
            "trailer_offset_m": float(
                route.last_steering_debug.get("trailer_envelope", {}).get(
                    "applied_offset_m", 0.0)),
        })
        elapsed += dt
        frame += 1

    if not samples:
        raise AssertionError("closed-loop replay produced no samples")
    ctes = [sample["cte_m"] for sample in samples]
    outputs = [sample["output"] for sample in samples]
    acceleration_steps = [
        abs(second["acceleration"] - first["acceleration"])
        / max(second["dt_s"], 1e-9)
        for first, second in zip(samples, samples[1:])
    ]
    return {
        "samples": samples,
        "max_cte_m": max(map(abs, ctes)),
        "rms_cte_m": math.sqrt(statistics.fmean(value * value
                                                for value in ctes)),
        "max_heading_error_deg": math.degrees(max(
            abs(sample["heading_error_rad"]) for sample in samples)),
        "max_step": max((abs(second - first)
                         for first, second in zip(outputs, outputs[1:])),
                        default=0.0),
        "max_rate": max(abs(sample["rate"]) for sample in samples),
        "max_acceleration": max(
            abs(sample["acceleration"]) for sample in samples),
        "max_jerk": max(acceleration_steps, default=0.0),
        "sign_changes": _sign_changes(outputs),
        "final_cte_m": ctes[-1],
        "final_heading_error_deg": math.degrees(
            samples[-1]["heading_error_rad"]),
        "progress_monotonic": all(
            second["progress_m"] + 1e-6 >= first["progress_m"]
            for first, second in zip(samples, samples[1:])),
    }


class Phase4EClosedLoopControllerTests(unittest.TestCase):
    def test_real_225932_relay_and_212431_negative_zero_gate_are_removed(self):
        # 22:59:33: the old opposite-authority relay applied +0.576 to a
        # -0.103 left-curve foundation and emitted +0.473 (right steering).
        curve_reference = -0.103
        legacy_applied_feedback = +0.576
        legacy_command = curve_reference + legacy_applied_feedback
        self.assertAlmostEqual(legacy_command, +0.473, places=6)

        route = Route([(0.0, 0.0), (0.0, 20.0)])
        decomposed_legacy_terms, debug = route._compose_curve_steering(
            curve_reference,
            heading_feedback=+0.251,
            cte_feedback=+0.392,
        )
        self.assertAlmostEqual(decomposed_legacy_terms, +0.540, places=9)
        self.assertFalse(debug["curve_sign_projection_active"])
        self.assertNotIn("opposite_correction_authorized", debug)

        # 21:24:31--33 was a different failure of the replacement gate: local
        # curvature logged as -0.000, but the old sign projection still forced
        # the required +0.250 recovery to zero. The composition layer now
        # preserves the one geometric pursuit result even for tiny curvature.
        microscopic, microscopic_debug = route._compose_curve_steering(
            -1e-7, +0.133, +0.117)
        self.assertAlmostEqual(microscopic, 0.2499999, places=9)
        self.assertFalse(
            microscopic_debug["curve_sign_projection_active"])

        # Feed the command through the real 20 Hz actuator/bicycle loop on a
        # numerically left but physically straight R500 km lane. Recovery must
        # be allowed to oppose that microscopic sign and must not create the
        # 2.805 m localisation-loss excursion from the log.
        points, _ = _path_from_sections(((-2e-6, 180.0),))
        metrics = _simulate_closed_loop(
            points, 12.0, initial_lateral_m=1.159,
            initial_heading_error_deg=3.0)
        self.assertLess(metrics["samples"][0]["curvature_per_m"], 0.0)
        self.assertGreater(metrics["samples"][0]["raw"], 0.0)
        self.assertLess(metrics["max_cte_m"], 1.55)
        self.assertLess(abs(metrics["final_cte_m"]), 0.05)

    def test_left_right_r35_r83_are_stable_at_low_and_road_speed(self):
        for direction in (-1.0, 1.0):
            for radius_m in (35.0, 83.0):
                road_speed = (16.67 if radius_m == 83.0
                              else curve_speed_limit_ms(radius_m, 0.0))
                for speed_ms in (5.5, road_speed):
                    points, _boundaries = _constant_curve_path(
                        direction, radius_m)
                    metrics = _simulate_closed_loop(points, speed_ms)
                    turn = [
                        sample for sample in metrics["samples"]
                        if (_boundaries[0] + 8.0
                            < sample["progress_m"]
                            < _boundaries[1] - 4.0)
                    ]
                    with self.subTest(direction=direction, radius=radius_m,
                                      speed_ms=speed_ms):
                        self.assertLessEqual(metrics["max_cte_m"], 0.70)
                        self.assertLessEqual(metrics["max_heading_error_deg"],
                                             4.0)
                        self.assertLessEqual(metrics["max_step"], 0.031)
                        # Entry/exit recovery can legitimately use the other
                        # sign. Inside the one confirmed bend it cannot.
                        self.assertEqual(_sign_changes(
                            [sample["output"] for sample in turn]), 0)
                        self.assertFalse(any(
                            sample["raw"] * sample["curvature_per_m"] < -1e-9
                            for sample in turn))
                        self.assertLessEqual(metrics["max_rate"], 0.601)
                        self.assertLessEqual(metrics["max_acceleration"], 14.01)
                        self.assertLessEqual(metrics["max_jerk"], 561.0)
                        self.assertTrue(metrics["progress_monotonic"])

    def test_r18_roundabout_is_stable_in_both_directions(self):
        for direction in (-1.0, 1.0):
            points, boundaries = _constant_curve_path(
                direction, 18.0, sweep_deg=270.0, exit_m=45.0)
            metrics = _simulate_closed_loop(points, 5.5)
            turn = [sample for sample in metrics["samples"]
                    if boundaries[0] + 8.0 < sample["progress_m"]
                    < boundaries[1] - 4.0]
            with self.subTest(direction=direction):
                self.assertLess(metrics["max_cte_m"], 0.35)
                self.assertLess(metrics["rms_cte_m"], 0.12)
                self.assertLess(metrics["max_heading_error_deg"], 5.0)
                self.assertEqual(_sign_changes(
                    [sample["output"] for sample in turn]), 0)
                self.assertLessEqual(metrics["max_step"], 0.031)

    def test_offcentre_and_heading_error_recover_before_lane_loss(self):
        paths = [("straight", _path_from_sections(((0.0, 180.0),))[0])]
        for direction in (-1.0, 1.0):
            paths.append((
                "r83-%+d" % direction,
                _constant_curve_path(direction, 83.0)[0]))
        initial_states = (
            (-1.20, -3.0), (-1.20, +3.0),
            (+1.20, -3.0), (+1.20, +3.0),
        )
        for name, points in paths:
            for lateral_m, heading_deg in initial_states:
                metrics = _simulate_closed_loop(
                    points, 12.0, initial_lateral_m=lateral_m,
                    initial_heading_error_deg=heading_deg)
                with self.subTest(
                        path=name, lateral=lateral_m,
                        heading=heading_deg):
                    self.assertLess(metrics["max_cte_m"], 1.60)
                    self.assertLess(abs(metrics["final_cte_m"]), 0.08)
                    self.assertLess(metrics["max_heading_error_deg"], 6.5)
                    self.assertLessEqual(metrics["max_step"], 0.031)
                    self.assertLessEqual(metrics["sign_changes"], 4)
                    self.assertTrue(metrics["progress_monotonic"])

    def test_66_kmh_lane_error_recovers_inside_authority_boundary(self):
        points, _ = _path_from_sections(((0.0, 300.0),))
        for lateral_m, heading_deg in (
                (-1.20, -3.0), (-1.20, +3.0),
                (+1.20, -3.0), (+1.20, +3.0)):
            metrics = _simulate_closed_loop(
                points, 66.0 / 3.6, initial_lateral_m=lateral_m,
                initial_heading_error_deg=heading_deg)
            with self.subTest(lateral=lateral_m, heading=heading_deg):
                self.assertLess(metrics["max_cte_m"], 1.75)
                self.assertLess(abs(metrics["final_cte_m"]), 0.05)
                self.assertLess(metrics["max_heading_error_deg"], 5.5)
                self.assertLessEqual(metrics["max_step"], 0.031)
                self.assertLessEqual(metrics["sign_changes"], 4)

    def test_s_curve_reverses_once_and_remains_closed_loop_stable(self):
        points, boundaries = _s_curve_path()
        metrics = _simulate_closed_loop(points, 9.5)
        first_turn = [sample for sample in metrics["samples"]
                      if boundaries[0] + 8.0 < sample["progress_m"]
                      < boundaries[1] - 4.0]
        second_turn = [sample for sample in metrics["samples"]
                       if boundaries[2] + 8.0 < sample["progress_m"]
                       < boundaries[3] - 4.0]
        self.assertTrue(any(sample["output"] > 0.05
                            for sample in first_turn))
        self.assertTrue(any(sample["output"] < -0.05
                            for sample in second_turn))
        self.assertLessEqual(metrics["max_cte_m"], 0.70)
        self.assertLessEqual(metrics["sign_changes"], 3)
        self.assertTrue(metrics["progress_monotonic"])

    def test_curve_exit_settles_without_opposite_oscillation(self):
        points, boundaries = _constant_curve_path(1.0, 35.0, exit_m=75.0)
        metrics = _simulate_closed_loop(
            points, curve_speed_limit_ms(35.0, 0.0))
        exit_progress = boundaries[1]
        straight = [sample for sample in metrics["samples"]
                    if sample["progress_m"] >= exit_progress + 15.0]
        settled = [sample for sample in metrics["samples"]
                   if sample["progress_m"] >= exit_progress + 45.0]
        self.assertTrue(straight)
        self.assertTrue(settled)
        # The delayed physical wheel still carries curve lock just after the
        # geometry changes.  Once it has unwound, the straight must stay in
        # the settled 0.25 m band rather than starting another oscillation.
        self.assertLessEqual(max(abs(sample["cte_m"])
                                 for sample in settled), 0.25)
        self.assertLessEqual(_sign_changes(
            [sample["output"] for sample in straight], threshold=0.02), 1)
        self.assertLessEqual(abs(straight[-1]["cte_m"]), 0.12)
        self.assertLessEqual(abs(math.degrees(
            straight[-1]["heading_error_rad"])), 1.0)

    def test_noise_and_delayed_tick_do_not_create_lane_loss(self):
        points, boundaries = _constant_curve_path(-1.0, 83.0)
        metrics = _simulate_closed_loop(
            points, 12.0, noisy_lane_match=True, delayed_ticks=True)
        turn = [sample for sample in metrics["samples"]
                if boundaries[0] + 8.0 < sample["progress_m"]
                < boundaries[1] - 4.0]
        self.assertLessEqual(metrics["max_cte_m"], 0.70)
        self.assertLessEqual(metrics["max_heading_error_deg"], 4.0)
        self.assertLessEqual(metrics["max_step"], 0.061)
        self.assertEqual(_sign_changes(
            [sample["output"] for sample in turn]), 0)
        self.assertFalse(any(
            sample["raw"] * sample["curvature_per_m"] < -1e-9
            for sample in turn))
        self.assertTrue(metrics["progress_monotonic"])

    def test_lane_id_revision_boundary_does_not_inject_steering_state(self):
        points, boundaries = _constant_curve_path(1.0, 83.0)
        boundary = boundaries[0] + 30.0
        baseline = _simulate_closed_loop(points, 11.0)
        changed = _simulate_closed_loop(
            points, 11.0, authority_change_progress_m=boundary)
        self.assertEqual(len(baseline["samples"]), len(changed["samples"]))
        differences = [
            abs(first["raw"] - second["raw"])
            for first, second in zip(baseline["samples"], changed["samples"])
        ]
        self.assertLessEqual(max(differences, default=0.0), 1e-12)
        transitions = [index for index, (first, second) in enumerate(zip(
            changed["samples"], changed["samples"][1:]), 1)
            if first["revision"] != second["revision"]]
        self.assertEqual(len(transitions), 1)
        index = transitions[0]
        changed_step = abs(changed["samples"][index]["raw"]
                           - changed["samples"][index - 1]["raw"])
        baseline_step = abs(baseline["samples"][index]["raw"]
                            - baseline["samples"][index - 1]["raw"])
        self.assertAlmostEqual(changed_step, baseline_step, places=12)

    def test_real_point_authority_road_prefab_boundary_is_continuous(self):
        """A LaneId label change cannot bend one smooth R83 tangent twice."""
        points, authorities, boundary = _point_authority_r83_fixture()
        samples = []
        for index, lane, revision in (
                (boundary - 1, ROAD_R83_LANE, 12),
                (boundary + 1, PREFAB_R83_LANE, 13)):
            route = Route(points, point_authorities=authorities)
            heading = _heading_between(
                route.points[index - 1], route.points[index + 1])
            command = route.steering(
                route.points[index], heading, 11.0,
                cross_track_error_m=0.0,
                control_authority=_authority_payload(
                    lane, revision=revision),
                control_dt_s=DT_S,
            )
            debug = route.last_steering_debug
            self.assertTrue(debug["authority_valid"])
            samples.append({
                "raw": command,
                "heading_error_rad": debug["guidance_heading_error_rad"],
                "curvature_per_m": debug["local_curvature"],
                "feed_forward": debug["feed_forward"],
            })

        before, after = samples
        # Geometry is exactly one circle. A label boundary therefore has no
        # physical curvature or tangent discontinuity to justify a wheel step.
        self.assertLess(abs(after["curvature_per_m"]
                            - before["curvature_per_m"]), 1e-4)
        self.assertLess(abs(after["feed_forward"]
                            - before["feed_forward"]), 0.005)
        self.assertLess(abs(after["heading_error_rad"]
                            - before["heading_error_rad"]),
                        math.radians(0.5))
        self.assertLess(abs(after["raw"] - before["raw"]), 0.04)

    def test_quantised_elevation_change_uses_real_3d_continuity(self):
        """A continuous grade may cross layer 47->49 without a wheel step."""
        points, _authorities, boundary = _point_authority_r83_fixture()
        world_points = [
            (point[0], index * 0.20, point[1])
            for index, point in enumerate(points)
        ]
        authorities = [
            (ROAD_R83_LANE, 47) if index <= boundary
            else (PREFAB_R83_LANE, 49)
            for index in range(len(points))
        ]
        commands = []
        for index, lane, revision, layer in (
                (boundary - 1, ROAD_R83_LANE, 12, 47),
                (boundary + 1, PREFAB_R83_LANE, 13, 49)):
            route = Route(
                world_points, point_authorities=authorities)
            heading = _heading_between(
                route.points[index - 1], route.points[index + 1])
            command = route.steering(
                route.points[index], heading, 11.0,
                cross_track_error_m=0.0,
                control_authority=_authority_payload(
                    lane, revision=revision, elevation_layer=layer),
                control_dt_s=DT_S)
            self.assertTrue(route.last_steering_debug["authority_valid"])
            commands.append(command)
        self.assertLess(abs(commands[1] - commands[0]), 0.04)

        probe = Route(world_points, point_authorities=authorities)
        self.assertTrue(probe._authority_geometry_compatible(
            authorities[boundary], authorities[boundary + 1],
            boundary, boundary + 1))

    def test_bridge_vertical_step_cannot_enter_pursuit_geometry(self):
        """A nearby elevated road is not a forward pursuit target."""
        lower = [(0.0, 0.0, float(z)) for z in range(0, 21, 2)]
        upper = [
            (float(x), 7.0, 22.0) for x in range(0, 21, 2)
        ]
        world_points = lower + upper
        boundary = len(lower) - 1
        authorities = [
            (ROAD_R83_LANE, 0) if index <= boundary
            else (PREFAB_R83_LANE, 2)
            for index in range(len(world_points))
        ]
        route = Route(world_points, point_authorities=authorities)
        self.assertFalse(route._authority_geometry_compatible(
            authorities[boundary], authorities[boundary + 1],
            boundary, boundary + 1))
        index = boundary - 1
        command = route.steering(
            route.points[index], math.pi, 8.0,
            cross_track_error_m=0.0,
            control_authority=_authority_payload(
                ROAD_R83_LANE, revision=12, elevation_layer=0),
            control_dt_s=DT_S)
        self.assertTrue(route.last_steering_debug["authority_valid"])
        self.assertLess(abs(command), 1e-9)
        self.assertLessEqual(
            route.last_steering_debug["pursuit_target_progress_m"],
            route._cumulative_m[boundary] + 1e-9)

    def test_early_live_lane_id_cannot_jump_to_distant_future_run(self):
        """A future prefab LaneId 13 m ahead is not current authority."""
        points, authorities, boundary = _point_authority_r83_fixture()
        route = Route(points, point_authorities=authorities)
        index = boundary - 12
        heading = _heading_between(
            route.points[index - 1], route.points[index + 1])
        actual_progress = route._cumulative_m[index]
        command = route.steering(
            route.points[index], heading, 11.0,
            cross_track_error_m=0.0,
            control_authority=_authority_payload(
                PREFAB_R83_LANE, revision=13),
            control_dt_s=DT_S,
        )
        debug = route.last_steering_debug
        self.assertFalse(debug["authority_valid"])
        self.assertEqual(command, 0.0)
        # Rejection must also avoid advancing the cached route projection to
        # the first point of the future authority run.
        if route._tracking_state is not None:
            self.assertLessEqual(route._tracking_state[4],
                                 actual_progress + 2.0)

    def test_overlapping_future_lane_run_cannot_jump_route_progress(self):
        """XZ distance zero cannot authorize a LaneId 80 route metres ahead."""
        points = [(0.0, 0.0)]

        def append_line(end_x, end_z):
            start_x, start_z = points[-1]
            length = math.hypot(end_x - start_x, end_z - start_z)
            count = max(1, int(round(length)))
            points.extend((
                start_x + (end_x - start_x) * index / count,
                start_z + (end_z - start_z) * index / count,
            ) for index in range(1, count + 1))

        # A: first traversal of the shared straight. The connector then makes
        # a real ordered loop and B traverses the same XZ straight later. This
        # reproduces a future roundabout/loop arm which passes a distance-only
        # projection gate even though its route progress is unrelated.
        append_line(0.0, 20.0)
        first_run_last = len(points) - 1
        append_line(20.0, 20.0)
        append_line(20.0, 0.0)
        append_line(0.0, 0.0)
        connector_last = len(points) - 1
        append_line(0.0, 20.0)

        connector_lane = (624008950, 1, 0, "loop", 0, (1, 2))
        authorities = []
        for index in range(len(points)):
            lane = (ROAD_R83_LANE if index <= first_run_last else
                    connector_lane if index <= connector_last else
                    PREFAB_R83_LANE)
            authorities.append((lane, R83_ELEVATION_LAYER))

        route = Route(points, point_authorities=authorities)
        position = (0.0, 5.0)
        heading = math.pi
        route.steering(
            position, heading, 8.0, cross_track_error_m=0.0,
            control_authority=_authority_payload(
                ROAD_R83_LANE, revision=12),
            control_dt_s=DT_S,
        )
        state_before = route._tracking_state
        self.assertIsNotNone(state_before)
        self.assertAlmostEqual(state_before[4], 5.0, places=9)
        future_indices = [
            index for index, authority in enumerate(authorities)
            if authority == (PREFAB_R83_LANE, R83_ELEVATION_LAYER)
        ]
        overlapping_future = min(
            future_indices,
            key=lambda index: math.dist(position, route.points[index]))
        self.assertEqual(
            math.dist(position, route.points[overlapping_future]), 0.0)
        self.assertAlmostEqual(
            route._cumulative_m[overlapping_future], 85.0, places=9)

        command = route.steering(
            position, heading, 8.0, cross_track_error_m=0.0,
            control_authority=_authority_payload(
                PREFAB_R83_LANE, revision=13),
            control_dt_s=DT_S,
        )
        debug = route.last_steering_debug
        self.assertEqual(command, 0.0)
        self.assertFalse(debug["authority_valid"])
        self.assertEqual(route._tracking_state, state_before)
        self.assertAlmostEqual(route._tracking_state[4], 5.0, places=9)

    def test_point_authority_wrong_deck_is_fail_neutral(self):
        points, authorities, boundary = _point_authority_r83_fixture()
        route = Route(points, point_authorities=authorities)
        index = boundary - 3
        heading = _heading_between(
            route.points[index - 1], route.points[index + 1])
        command = route.steering(
            route.points[index], heading, 11.0,
            cross_track_error_m=0.0,
            control_authority=_authority_payload(
                ROAD_R83_LANE, revision=13,
                elevation_layer=R83_ELEVATION_LAYER + 1),
            control_dt_s=DT_S,
        )
        self.assertEqual(command, 0.0)
        self.assertFalse(route.last_steering_debug["authority_valid"])

    def test_spatial_trailer_offset_is_stable_across_lane_id_boundary(self):
        points, authorities, boundary = _point_authority_r83_fixture()
        offsets = []
        predictions = []
        for index, lane, revision in (
                (boundary - 2, ROAD_R83_LANE, 12),
                (boundary - 1, ROAD_R83_LANE, 12),
                (boundary + 1, PREFAB_R83_LANE, 13),
                (boundary + 2, PREFAB_R83_LANE, 13)):
            route = Route(points, point_authorities=authorities)
            progress = route._cumulative_m[index]
            tractor = route.points[index]
            heading = _heading_between(
                route.points[index - 1], route.points[index + 1])
            trailer = route._point_at_progress(
                progress - TRAILER_EFFECTIVE_AXLE_DISTANCE_M)
            trailer_before = route._point_at_progress(
                progress - TRAILER_EFFECTIVE_AXLE_DISTANCE_M - 1.0)
            trailer_heading = _heading_between(trailer_before, trailer)
            route.steering(
                tractor, heading, 8.0, cross_track_error_m=0.0,
                vehicle_envelope={
                    "attached": True,
                    "position": trailer,
                    "heading": trailer_heading,
                    "lane_width_m": 4.7,
                    "tractor_altitude_m": float(R83_ELEVATION_LAYER),
                    "trailer_altitude_m": float(R83_ELEVATION_LAYER),
                },
                control_authority=_authority_payload(
                    lane, revision=revision),
                control_dt_s=DT_S,
            )
            debug = route.last_steering_debug["trailer_envelope"]
            self.assertTrue(debug["accepted"])
            self.assertTrue(debug["curve_side_proven"])
            offsets.append(debug["applied_offset_m"])
            predictions.append(debug["predicted_offtrack_m"])

        self.assertGreater(abs(offsets[0]), 0.05)
        self.assertLess(max(offsets) - min(offsets), 1e-9)
        self.assertLess(max(predictions) - min(predictions), 1e-9)

    def test_trailer_target_is_spatial_balanced_and_not_a_fast_cte_sensor(self):
        points, boundaries = _constant_curve_path(1.0, 35.0)
        route = Route(points)
        progress = boundaries[0] + 30.0
        tractor = route._point_at_progress(progress)
        forward = route._point_at_progress(progress + 2.0)
        heading = _heading_between(tractor, forward)
        base_trailer = route._point_at_progress(
            progress - TRAILER_EFFECTIVE_AXLE_DISTANCE_M)
        behind = route._point_at_progress(
            progress - TRAILER_EFFECTIVE_AXLE_DISTANCE_M - 2.0)
        trailer_heading = _heading_between(behind, base_trailer)
        offsets = []
        predicted = []
        for measured_side in (-0.70, +0.70, -0.35, +0.35):
            dx = base_trailer[0] - behind[0]
            dz = base_trailer[1] - behind[1]
            length = math.hypot(dx, dz)
            trailer = (base_trailer[0] + dz / length * measured_side,
                       base_trailer[1] - dx / length * measured_side)
            route.steering(
                tractor, heading, 8.0, cross_track_error_m=0.0,
                vehicle_envelope={
                    "attached": True,
                    "position": trailer,
                    "heading": trailer_heading,
                    "lane_width_m": 4.7,
                    "tractor_altitude_m": 45.0,
                    "trailer_altitude_m": 45.0,
                },
                control_dt_s=DT_S,
            )
            debug = route.last_steering_debug["trailer_envelope"]
            self.assertTrue(debug["accepted"])
            self.assertTrue(debug["curve_side_proven"])
            self.assertEqual(debug["balanced_reference_fraction"],
                             TRAILER_BALANCED_REFERENCE_FRACTION)
            offsets.append(debug["applied_offset_m"])
            predicted.append(debug["predicted_offtrack_m"])
        self.assertLess(max(offsets) - min(offsets), 1e-12)
        self.assertLess(max(predicted) - min(predicted), 1e-12)
        self.assertAlmostEqual(
            offsets[0], predicted[0] * TRAILER_BALANCED_REFERENCE_FRACTION,
            places=9)


if __name__ == "__main__":
    unittest.main()
