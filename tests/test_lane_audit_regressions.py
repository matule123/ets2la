import math
import multiprocessing as mp
import threading
import time
import unittest

from core.ar_overlay import AROverlay
from core.hud import UltraPilotHUD
from core.engine import UltraPilotEngine
from core.ipc.shared_state import SharedState
from core.navigation.lane_model import (
    GpsCorridorEdge, LaneId, LaneLocator, LaneMatch, LanePath, LanePoint,
    LaneSegment,
)
from core.navigation.lane_trajectory import build_lane_trajectory
from core.navigation.road_network import RoadNetwork
from core.navigation.route import (
    NORMALIZED_STEERING_ANGLE_RAD,
    TRACTOR_BODY_WIDTH_M, TRUCK_WHEELBASE_M, Route,
    curve_speed_limit_ms,
)
from plugins.autopilot.main import Plugin as AutopilotPlugin
from tests.test_lane_authority_integration import (
    Controller, MapSDK, State, Tags, Telemetry, build_map_plugin,
)
from tests.test_lane_route_builder import SyntheticMap
from tests.test_lane_trajectory import single_lane_path


def hud_reader(state):
    return type("HUDReader", (), {
        "shared_state": state, "_rear_cam_side": "off",
        "_rear_cam_until": 0.0,
    })()


def autopilot_state(confidence, *, valid=True, heartbeat=None,
                    telemetry_valid=True):
    now = time.monotonic() if heartbeat is None else heartbeat
    state = State({
        "system_state": "CRUISE", "danger_level": 0.0,
        "lane_offset": 0.9, "traffic": [], "nav_active": True,
        "nav_steering": 0.4, "acc_throttle": 0.0, "acc_brake": 0.0,
        "autopilot_active": True, "game_route_distance": 100.0,
        "game_route_node_uids": [1, 2], "telemetry_valid": telemetry_valid,
        "lane_trajectory_heartbeat": now, "lane_trajectory_revision": 7,
        "lane_trajectory": {
            "revision": 7, "valid": valid, "confidence": confidence,
            "source_gps_uids": [1, 2],
            "points": [[0, 0, 0], [0, 0, 10]],
            "display_points": [[0, 0, 0], [0, 0, 10]],
        },
    })
    sdk = type("SDK", (), {})()
    sdk.shared_state, sdk.controller, sdk.telemetry = state, Controller(), Telemetry()
    plugin = AutopilotPlugin(sdk)
    plugin.tags = Tags()
    plugin.on_start()
    return plugin, state


class LaneGeometryAuditTests(unittest.TestCase):
    @staticmethod
    def _arc(direction, radius=100.0, length_m=100.0):
        return [
            (direction * (radius - radius * math.cos(s / radius)),
             radius * math.sin(s / radius))
            for s in range(0, int(length_m) + 1, 2)
        ]

    @staticmethod
    def _path_heading(first, second):
        return math.atan2(-(second[0] - first[0]),
                          -(second[1] - first[1]))

    def test_wide_live_map_scene_never_blocks_navigation_tick(self):
        started, release = threading.Event(), threading.Event()
        requests = []

        class SlowPresentationNetwork:
            loaded = True

            def live_map_segments_3d_near(self, *_args, **kwargs):
                requests.append(("roads", dict(kwargs)))
                started.set()
                release.wait(2.0)
                return [((0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                         "road", 2, False, True, False, False,
                         4.5, False, "r0:0", 0)]

            def live_map_road_type(self, *_args, **_kwargs):
                return "local"

            def live_map_polygons_near(self, *_args, **kwargs):
                requests.append(("areas", dict(kwargs)))
                return []

            def map_features_near(self, *_args, **kwargs):
                requests.append(("features", dict(kwargs)))
                return []

        plugin, sdk, _point = build_map_plugin()
        plugin.road_net = SlowPresentationNetwork()
        before = time.monotonic()
        self.assertTrue(plugin._schedule_live_map_scene((0.0, 0.0), 3.0))
        elapsed = time.monotonic() - before
        self.assertLess(elapsed, 0.20)
        self.assertTrue(started.wait(0.5))

        # The deliberately blocked presentation worker does not own or delay
        # the authoritative lane heartbeat.
        heartbeat = time.monotonic()
        sdk.set("lane_trajectory_heartbeat", heartbeat)
        self.assertEqual(sdk.get("lane_trajectory_heartbeat"), heartbeat)

        release.set()
        deadline = time.monotonic() + 1.0
        while plugin._live_map_loading and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertFalse(plugin._live_map_loading)
        self.assertEqual(sdk.get("live_map_scene_revision"), 1)
        self.assertEqual(len(sdk.get("live_map_road_segments")), 1)
        self.assertEqual(requests, [
            ("roads", {"radius": 2000.0, "limit": 16000,
                       "altitude": 3.0}),
            ("areas", {"radius": 2000.0, "limit": 3000}),
            ("features", {"radius": 2000.0, "limit": 1800}),
        ])
        self.assertEqual(sdk.get("live_map_scene_radius_m"), 2000.0)
        self.assertEqual(sdk.get("live_map_scene_counts"), {
            "roads": 1, "areas": 0, "features": 0})

    def test_hidden_live_map_never_schedules_wide_presentation_scan(self):
        plugin, sdk, point = build_map_plugin()
        sdk.set("truck_world_pos", (point.x, point.z))
        sdk.set("truck_heading", point.heading)
        sdk.set("truck_speed_ms", 10.0)
        sdk.set("telemetry_valid", True)
        plugin._load_road_net = lambda: None
        plugin._update_lane_trajectory = lambda *_args: plugin._lane_path
        plugin._publish_road_type = lambda *_args: None
        plugin.tags = Tags()
        plugin._roads_t = 0.0
        plugin._diag_t = 0.0
        scheduled = []
        plugin._schedule_live_map_scene = lambda *args: (
            scheduled.append(args) or True)

        plugin._live_map_t = 2.0
        sdk.set("live_map_view_active", False)
        plugin.on_tick(0.1)
        self.assertEqual(scheduled, [])
        self.assertEqual(plugin._live_map_t, 0.0)

        sdk.set("live_map_view_active", True)
        plugin._live_map_t = 1.45
        plugin.on_tick(0.1)
        self.assertEqual(len(scheduled), 1)

        # Keep the wide tile cached while the truck moves inside its margin.
        plugin._live_map_pos = (point.x, point.z)
        plugin._live_map_t = 2.0
        sdk.set("truck_world_pos", (point.x + 100.0, point.z))
        plugin.on_tick(0.1)
        self.assertEqual(len(scheduled), 1)

        plugin._live_map_t = 2.0
        sdk.set("truck_world_pos", (point.x + 151.0, point.z))
        plugin.on_tick(0.1)
        self.assertEqual(len(scheduled), 2)

    def test_live_map_exploration_records_only_confirmed_ordinary_lane(self):
        plugin, sdk, point = build_map_plugin()
        self.assertTrue(plugin._lane_localization_current)
        road_uid = plugin._lane_match.lane_id.road_uid
        self.assertTrue(plugin._record_explored_road())
        self.assertIn(road_uid, plugin._explored_road_uids)
        self.assertEqual(sdk.get("live_map_explored_road_count"), 1)
        self.assertFalse(plugin._record_explored_road())

        original = plugin._lane_match
        plugin._lane_match = type("PrefabMatch", (), {
            "lane_id": type("PrefabLane", (), {
                "road_uid": 999, "prefab_token": "roundabout"})(),
        })()
        self.assertFalse(plugin._record_explored_road())
        self.assertNotIn(999, plugin._explored_road_uids)
        plugin._lane_match = original

    def test_hud_prefab_scene_never_blocks_navigation_heartbeat(self):
        """15:16 trace: the synchronous HUD prefab scan froze map ticks."""
        started, release = threading.Event(), threading.Event()

        class SlowHudNetwork:
            loaded = True

            def hud_segments_3d_near(self, *_args, **_kwargs):
                started.set()
                release.wait(2.0)
                return [((0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                         "lane", 1, False, False, False, False,
                         3.05, False, "roundabout:4", 0)]

        plugin, sdk, _point = build_map_plugin()
        plugin.road_net = SlowHudNetwork()
        heartbeat = time.monotonic()
        sdk.set("lane_trajectory_heartbeat", heartbeat)
        before = time.monotonic()
        self.assertTrue(plugin._schedule_hud_road_scene(
            (0.0, 0.0), 3.0))
        elapsed = time.monotonic() - before
        self.assertLess(elapsed, 0.20)
        self.assertTrue(started.wait(0.5))
        self.assertEqual(sdk.get("lane_trajectory_heartbeat"), heartbeat)

        release.set()
        deadline = time.monotonic() + 1.0
        while plugin._roads_loading and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertFalse(plugin._roads_loading)
        self.assertEqual(sdk.get("map_road_segments_revision"), 1)
        payload = sdk.get("map_road_segments")
        self.assertEqual(payload[0][2], "lane")
        self.assertEqual(payload[0][10:], ["roundabout:4", 0])

    def test_straight_left_right_and_feedforward_units(self):
        straight = Route([(0.0, 0.0), (0.0, 50.0), (0.0, 100.0)])
        self.assertAlmostEqual(straight.steering(
            (0.0, 10.0), math.pi, 15.0), 0.0, places=7)

        for direction in (-1.0, 1.0):
            with self.subTest(direction=direction):
                radius = 100.0
                route = Route(self._arc(direction, radius, 140.0))
                pos = route.points[10]
                tangent = route.lookahead_point(10, pos, 6.0)
                heading = self._path_heading(pos, tangent)
                steering = route.steering(
                    pos, heading, 15.0, cross_track_error_m=0.0)
                guidance_curvature = route.last_steering_debug[
                    "guidance_curvature"]
                # Positive X is left of +Z travel in ETS world coordinates;
                # therefore ``direction=+1`` is a left bend and must command
                # negative steering (positive controller output is right).
                self.assertEqual(math.copysign(1.0, steering), -direction)
                # Curvature is 1/metre; wheelbase*curvature is dimensionless,
                # atan returns radians, and division maps it to controller
                # normalized steering units.
                expected = (math.atan(TRUCK_WHEELBASE_M
                                      * guidance_curvature)
                            / NORMALIZED_STEERING_ANGLE_RAD)
                self.assertAlmostEqual(steering, expected, places=7)

    def test_physical_steering_model_can_reach_captured_roundabout_radius(self):
        # The real failed route contains an approximately 18 m prefab bend.
        # The former 5.0 m / 0.18 rad model bottomed out at 27.3 m and could
        # only saturate the command before leaving the lane.
        minimum_radius = (TRUCK_WHEELBASE_M
                          / math.tan(NORMALIZED_STEERING_ANGLE_RAD))
        self.assertLess(minimum_radius, 18.0)

    def test_eighteen_metre_prefab_bend_tracks_without_saturation_loss(self):
        dt, radius = 0.05, 18.0
        speed = curve_speed_limit_ms(radius, 0.0)
        for direction in (-1.0, 1.0):
            route = Route(self._arc(direction, radius, 42.0))
            x, z = route.points[0]
            heading = self._path_heading(route.points[0], route.points[2])
            plugin = AutopilotPlugin.__new__(AutopilotPlugin)
            plugin._last_steering = 0.0
            errors, commands = [], []
            for _ in range(int(36.0 / speed / dt)):
                index = route.tracking_index((x, z), heading)
                cte = route.cross_track_error(index, (x, z))
                raw = route.steering(
                    (x, z), heading, speed, cross_track_error_m=cte)
                plugin._last_steering = plugin._ramp_steering(
                    raw, dt, speed_ms=speed,
                    curvature_per_m=route.last_steering_debug.get(
                        "local_curvature", 0.0))
                heading -= (speed / TRUCK_WHEELBASE_M
                            * plugin._last_steering
                            * NORMALIZED_STEERING_ANGLE_RAD * dt)
                x += -math.sin(heading) * speed * dt
                z += -math.cos(heading) * speed * dt
                index = route.tracking_index((x, z), heading)
                errors.append(route.cross_track_error(index, (x, z)))
                commands.append(plugin._last_steering)
            with self.subTest(direction=direction):
                self.assertLess(max(map(abs, errors)), 1.50)
                self.assertLess(abs(errors[-1]), 0.55)
                self.assertLessEqual(max(
                    abs(current - previous) for previous, current
                    in zip(commands, commands[1:])), 0.031)

    def test_real_roundabout_feedback_cannot_reverse_confirmed_arc(self):
        """15:07 trace: R18 recovery is combined in one physical frame."""
        for direction in (-1.0, 1.0):
            route = Route(self._arc(direction, 18.0, 48.0))
            base = route.points[15]
            following = route.points[16]
            dx, dz = following[0] - base[0], following[1] - base[1]
            segment_length = math.hypot(dx, dz)
            heading = self._path_heading(base, route.points[18])
            for requested_cte in (-0.60, 0.60):
                position = (
                    base[0] + dz / segment_length * requested_cte,
                    base[1] - dx / segment_length * requested_cte,
                )
                index = route.tracking_index(position, heading)
                live_cte = route.cross_track_error(index, position)
                command = route.steering(
                    position, heading, 5.5,
                    cross_track_error_m=live_cte)
                debug = route.last_steering_debug
                with self.subTest(direction=direction, cte=requested_cte):
                    self.assertAlmostEqual(live_cte, requested_cte, places=6)
                    self.assertLess(debug["cte_geometry_residual"], 1e-6)
                    self.assertFalse(debug["curve_direction_hold"])
                    # The CTE component always points toward lane centre.  It
                    # may reduce the Ackermann turn all the way to the local
                    # tangent, but cannot reverse the complete wheel command.
                    self.assertGreater(debug["cte_steer"] * live_cte, 0.0)
                    self.assertGreaterEqual(
                        command * debug["local_curvature"], 0.0)
                    if command == 0.0:
                        self.assertTrue(
                            debug["curve_sign_projection_active"])

    def test_attached_trailer_uses_proven_lane_width_to_swing_outward(self):
        """The 05:30--06:10 video hairpin must account for the semi axle."""
        for direction in (-1.0, 1.0):
            route = Route(self._arc(direction, 18.0, 140.0))
            tractor = route.points[30]
            tractor_heading = self._path_heading(
                tractor, route.points[32])
            trailer = route.points[26]
            trailer_heading = self._path_heading(
                trailer, route.points[28])
            original_geometry = tuple(route.world_points)
            base = route.steering(
                tractor, tractor_heading, 5.5,
                cross_track_error_m=0.0)
            envelope = {
                "attached": True,
                "position": trailer,
                "heading": trailer_heading,
                "lane_width_m": 4.7,
                "tractor_altitude_m": 0.0,
                "trailer_altitude_m": 0.0,
                "elevation_layer": 0,
            }
            corrected = route.steering(
                tractor, tractor_heading, 5.5,
                cross_track_error_m=0.0,
                vehicle_envelope=envelope)
            debug = route.last_steering_debug
            trailer_debug = debug["trailer_envelope"]
            with self.subTest(direction=direction):
                self.assertTrue(trailer_debug["accepted"])
                self.assertLess(
                    trailer_debug["applied_offset_m"]
                    * debug["local_curvature"], 0.0)
                self.assertLess(abs(corrected), abs(base))
                self.assertEqual(tuple(route.world_points), original_geometry)

            # A trailer pose on another deck is not permission to move the
            # tractor target; bridge and ground-level geometry stay separate.
            rejected = dict(envelope, trailer_altitude_m=7.0)
            same_as_tractor_only = route.steering(
                tractor, tractor_heading, 5.5,
                cross_track_error_m=0.0,
                vehicle_envelope=rejected)
            self.assertAlmostEqual(same_as_tractor_only, base, places=9)
            self.assertFalse(
                route.last_steering_debug["trailer_envelope"]["accepted"])

    def test_articulated_r18_replay_keeps_both_axles_near_lane_centre(self):
        """Spatial trailer proof fixes off-tracking without steering history."""
        def approach_and_hairpin(direction):
            points = [(0.0, float(z)) for z in range(0, 41, 2)]
            for index in range(1, 61):
                angle = index * 2.0 / 18.0
                points.append((
                    direction * (18.0 - 18.0 * math.cos(angle)),
                    40.0 + 18.0 * math.sin(angle)))
            return points

        def replay(direction, use_envelope):
            route = Route(approach_and_hairpin(direction))
            x, z = route.points[5]
            trailer_x, trailer_z = route.points[1]
            heading = trailer_heading = math.pi
            speed, dt = 5.5, 0.05
            plugin = AutopilotPlugin.__new__(AutopilotPlugin)
            plugin._last_steering = 0.0
            samples = []
            for _ in range(420):
                index = route.tracking_index((x, z), heading)
                cte = route.cross_track_error(index, (x, z))
                envelope = ({
                    "attached": True,
                    "position": (trailer_x, trailer_z),
                    "heading": trailer_heading,
                    "lane_width_m": 4.7,
                    "tractor_altitude_m": 0.0,
                    "trailer_altitude_m": 0.0,
                } if use_envelope else None)
                target = route.steering(
                    (x, z), heading, speed,
                    cross_track_error_m=cte,
                    vehicle_envelope=envelope)
                plugin._last_steering = plugin._ramp_steering(
                    target, dt, speed_ms=speed,
                    curvature_per_m=route.last_steering_debug.get(
                        "local_curvature", 0.0))
                heading -= (speed / TRUCK_WHEELBASE_M
                            * plugin._last_steering
                            * NORMALIZED_STEERING_ANGLE_RAD * dt)
                x += -math.sin(heading) * speed * dt
                z += -math.cos(heading) * speed * dt
                articulation = ((heading - trailer_heading + math.pi)
                                % (2.0 * math.pi) - math.pi)
                trailer_heading += (speed / 8.0
                                    * math.sin(articulation) * dt)
                trailer_x = x + 8.0 * math.sin(trailer_heading)
                trailer_z = z + 8.0 * math.cos(trailer_heading)
                if route.tracking_progress((x, z), heading) > 42.0:
                    trailer_index = route.tracking_index(
                        (trailer_x, trailer_z), trailer_heading)
                    tractor_index = route.tracking_index((x, z), heading)
                    samples.append((
                        route.cross_track_error(
                            trailer_index, (trailer_x, trailer_z)),
                        route.cross_track_error(tractor_index, (x, z))))
            return (max(abs(item[0]) for item in samples),
                    max(abs(item[1]) for item in samples))

        for direction in (-1.0, 1.0):
            old_trailer_peak, _old_tractor_peak = replay(direction, False)
            trailer_peak, tractor_peak = replay(direction, True)
            with self.subTest(direction=direction):
                self.assertGreater(old_trailer_peak, 1.80)
                # Validate the complete vehicle envelope against the proven
                # 4.7 m lane instead of an arbitrary 1.00 m axle threshold.
                self.assertLess(
                    trailer_peak + TRACTOR_BODY_WIDTH_M * 0.5, 4.7 * 0.5)
                self.assertLess(
                    tractor_peak + TRACTOR_BODY_WIDTH_M * 0.5, 4.7 * 0.5)
                self.assertLess(trailer_peak, old_trailer_peak - 0.75)

    def test_real_222100_r54_entry_keeps_imminent_r18_curve_authority(self):
        """A proven R54-to-R18 entry is followed as one real geometry."""
        for curve_sign in (-1.0, 1.0):
            x = z = 0.0
            path_heading = math.pi
            points = [(x, z)]
            # Real topology, not mocked curvature on a straight: 40 m entry,
            # 14 m R54 transition, 32 m R18.8 apex, then receiving road.
            for curvature, length in (
                    (0.0, 40.0), (curve_sign / 54.0, 14.0),
                    (curve_sign / 18.8, 32.0), (0.0, 30.0)):
                for _ in range(int(length / 2.0)):
                    path_heading -= curvature * 2.0
                    x += -math.sin(path_heading) * 2.0
                    z += -math.cos(path_heading) * 2.0
                    points.append((x, z))
            route = Route(points)
            position = points[15]  # 10 m before the proved transition
            heading = self._path_heading(position, points[17])
            command = route.steering(
                position, heading, 30.0 / 3.6, cross_track_error_m=0.0)
            debug = route.last_steering_debug
            profile = route.curve_profile_ahead(position, heading, 35.0)
            with self.subTest(curve_sign=curve_sign):
                self.assertFalse(debug["curve_direction_hold"])
                self.assertAlmostEqual(profile["radius_m"], 18.8, delta=0.4)
                self.assertGreater(command * curve_sign, 0.0)
                self.assertLess(abs(command), 0.08)

    def test_real_224447_palisade_error_cannot_authorize_opposite_curve(self):
        """A controller-created lane error cannot reverse curve authority.

        At 22:44:47 feed-forward was -0.240 while the independent feedback was
        +0.450 and CTE had reached -0.974 m. The old approach hold forced
        -0.204 anyway; one second later CTE was -2.515 m and localisation was
        lost.  Phase 4E removes that error-as-authority relay entirely.  Even
        when Route projection and LaneMatch agree on the displacement, zero
        steering follows the tangent back outward; an opposite command would
        recreate the captured positive-feedback loop.
        """
        for curve_sign in (-1.0, 1.0):
            route = Route([(0.0, 0.0), (0.0, -100.0), (0.0, -200.0)])
            route._curvature_at_progress = (
                lambda _progress, _span=6.0, s=curve_sign: s / 56.0)
            route.curve_profile_ahead = (
                lambda *_args, s=curve_sign, **_kwargs: {
                    "radius_m": 19.6, "distance_m": 28.0,
                    "signed_curvature": s / 19.6, "horizon_m": 35.0,
                })
            cte = curve_sign * 0.974
            position = (-cte, -20.0)
            # The logger's 15.2° LaneMatch residual and Route's short local
            # tangent are different frames; 30° reproduces the logged +0.4
            # Route feedback against the -0.24 curvature feed-forward.
            heading = -curve_sign * math.radians(30.0)
            first_command = route.steering(
                position, heading, 22.0 / 3.6,
                cross_track_error_m=cte, control_dt_s=0.05)
            self.assertEqual(first_command, 0.0)
            for _ in range(7):
                command = route.steering(
                    position, heading, 22.0 / 3.6,
                    cross_track_error_m=cte, control_dt_s=0.05)
            debug = route.last_steering_debug
            with self.subTest(curve_sign=curve_sign):
                self.assertLess(debug["cte_geometry_residual"], 0.01)
                self.assertTrue(debug["curve_direction_hold_error_proven"])
                self.assertEqual(debug["curve_direction_hold_fraction"], 0.0)
                self.assertFalse(debug["curve_direction_hold"])
                self.assertLess(
                    debug["feed_forward"] * debug["feedback"], 0.0)
                self.assertEqual(command, 0.0)
                self.assertTrue(debug["curve_sign_projection_active"])
                self.assertNotIn("opposite_correction_authorized", debug)

    def test_real_214845_r65_error_cannot_reverse_monotonic_curve(self):
        """Replay the R65 error without reintroducing opposite authority.

        At 38 km/h the captured R65 bend had +0.203 feed-forward, -0.564
        confirmed feedback, -9 degrees of heading error and 2.128 m CTE.  The
        old stateful controller eventually allowed the feedback to cross zero.
        The real 22:59 replay proves that this creates a positive feedback
        relay.  The unified controller may release to the tangent (zero) but
        cannot command against unchanged immutable curvature.
        """
        route = Route(self._arc(-1.0, 65.0, 170.0))
        base_position = route.points[25]
        following = route.points[26]
        dx, dz = (following[0] - base_position[0],
                  following[1] - base_position[1])
        segment_length = math.hypot(dx, dz)
        requested_cte = -2.128
        position = (
            base_position[0] + dz / segment_length * requested_cte,
            base_position[1] - dx / segment_length * requested_cte,
        )
        heading = (self._path_heading(base_position, route.points[28])
                   - math.radians(9.0))
        first_command = route.steering(
            position, heading, 38.0 / 3.6,
            cross_track_error_m=requested_cte, control_dt_s=0.05)
        self.assertEqual(first_command, 0.0)
        for _ in range(7):
            command = route.steering(
                position, heading, 38.0 / 3.6,
                cross_track_error_m=requested_cte, control_dt_s=0.05)
        debug = route.last_steering_debug
        self.assertAlmostEqual(debug["feed_forward"], 0.209, delta=0.012)
        self.assertLess(debug["feedback"], -0.50)
        self.assertFalse(debug["curve_direction_hold_eligible"])
        self.assertFalse(debug["curve_direction_hold"])
        self.assertEqual(command, 0.0)
        self.assertTrue(debug["curve_sign_projection_active"])
        self.assertNotIn("opposite_correction_authorized", debug)
        large_error_lookahead = debug["guidance_lookahead_m"]

        # Recovery is continuous geometry, not a thresholded direction hold.
        # The spatial preview depends only on speed and immutable road
        # geometry; allowing CTE to move it caused the captured left/right
        # target jumps on the same bend.
        route.steering(
            position, heading, 38.0 / 3.6,
            cross_track_error_m=-0.40)
        small_error_debug = dict(route.last_steering_debug)
        route.steering(
            position, heading, 38.0 / 3.6,
            cross_track_error_m=-0.91)
        medium_error_debug = dict(route.last_steering_debug)
        self.assertFalse(small_error_debug["curve_direction_hold_eligible"])
        self.assertFalse(medium_error_debug["curve_direction_hold_eligible"])
        self.assertAlmostEqual(
            large_error_lookahead,
            medium_error_debug["guidance_lookahead_m"], places=9)
        self.assertAlmostEqual(
            medium_error_debug["guidance_lookahead_m"],
            small_error_debug["guidance_lookahead_m"], places=9)

    def test_captured_ten_metre_service_exit_stays_inside_lane(self):
        # The same drive exposed a short R~=10.2 m service connector: 16 m
        # ahead, then roughly 12 m of tight curvature. It is not a sustained
        # circular road; replay its measured compact shape instead of
        # pretending a tractor must drive an entire 10 m-radius lap.
        points = [(0.0, float(z)) for z in range(0, 21)]
        radius = 10.2
        for distance in range(1, 13):
            points.append((
                radius - radius * math.cos(distance / radius),
                20.0 + radius * math.sin(distance / radius)))
        route = Route(points)
        x, z = route.points[0]
        heading = self._path_heading(route.points[0], route.points[2])
        plugin = AutopilotPlugin.__new__(AutopilotPlugin)
        plugin._last_steering = 0.0
        speed, dt = curve_speed_limit_ms(radius, 0.0), 0.05
        errors = []
        for _ in range(int((route._cumulative_m[-1] - 1.0) / speed / dt)):
            index = route.tracking_index((x, z), heading)
            cte = route.cross_track_error(index, (x, z))
            raw = route.steering(
                (x, z), heading, speed, cross_track_error_m=cte)
            plugin._last_steering = plugin._ramp_steering(
                raw, dt, speed_ms=speed,
                curvature_per_m=route.last_steering_debug.get(
                    "local_curvature", 0.0))
            heading -= (speed / TRUCK_WHEELBASE_M
                        * plugin._last_steering
                        * NORMALIZED_STEERING_ANGLE_RAD * dt)
            x += -math.sin(heading) * speed * dt
            z += -math.cos(heading) * speed * dt
            index = route.tracking_index((x, z), heading)
            errors.append(route.cross_track_error(index, (x, z)))
        self.assertLess(max(map(abs, errors)), 1.50)
        self.assertLess(abs(errors[-1]), 1.50)

    def test_fifty_kilometre_mixed_curve_replay_stays_in_lane(self):
        # Deterministic long-run control replay.  It does not replace the game
        # test, but detects accumulated route progress, sign, centring and
        # steering-slew faults over the requested 50 km horizon.
        length_m, sample_m = 50_000.0, 4.0
        points = []
        for index in range(int(length_m / sample_m) + 1):
            distance = index * sample_m
            x = (4.0 * math.sin(distance / 55.0)
                 + 1.5 * math.sin(distance / 21.0))
            points.append((x, -distance))
        route = Route(points)
        x, z = route.points[0]
        heading = self._path_heading(route.points[0], route.points[2])
        plugin = AutopilotPlugin.__new__(AutopilotPlugin)
        plugin._last_steering = 0.0
        speed, dt = 12.0, 0.10
        peak_error = 0.0
        for _ in range(int((length_m - 100.0) / speed / dt)):
            index = route.tracking_index((x, z), heading)
            cte = route.cross_track_error(index, (x, z))
            raw = route.steering(
                (x, z), heading, speed, cross_track_error_m=cte)
            plugin._last_steering = plugin._ramp_steering(
                raw, dt, speed_ms=speed,
                curvature_per_m=route.last_steering_debug.get(
                    "local_curvature", 0.0))
            heading -= (speed / TRUCK_WHEELBASE_M
                        * plugin._last_steering
                        * NORMALIZED_STEERING_ANGLE_RAD * dt)
            x += -math.sin(heading) * speed * dt
            z += -math.cos(heading) * speed * dt
            index = route.tracking_index((x, z), heading)
            peak_error = max(
                peak_error, abs(route.cross_track_error(index, (x, z))))
        self.assertLess(peak_error, 1.50)
        self.assertGreater(route.tracking_progress((x, z), heading), 49_800.0)

    def test_two_metre_curve_sampling_noise_does_not_flip_the_wheel(self):
        """Regression for the ±0.62 steering reversals seen in the game log."""
        radius = 55.0
        points = []
        ideal_points = []
        for index in range(101):
            distance = index * 2.0
            angle = distance / radius
            # LaneTrajectory is resampled at about two metres. Small alternating
            # lateral quantisation must be averaged, not interpreted as an
            # alternating left/right bend by a four-metre curvature window.
            noise = 0.08 * ((index % 3) - 1)
            ideal_points.append((
                radius * (1.0 - math.cos(angle)),
                radius * math.sin(angle)))
            points.append((
                ideal_points[-1][0] + noise, ideal_points[-1][1]))
        route = Route(points)

        for speed in (1.0, 4.2, 8.0, 15.0, 25.0):
            plugin = AutopilotPlugin.__new__(AutopilotPlugin)
            plugin._last_steering = 0.0
            raw_commands, applied_commands = [], []
            for index in range(3, 90):
                position = route.points[index]
                # Quantisation belongs to the map samples. Truck telemetry is
                # the smooth physical tangent, not the direction between two
                # alternating noisy map points.
                heading = self._path_heading(
                    ideal_points[index], ideal_points[index + 1])
                raw = route.steering(
                    position, heading, speed,
                    cross_track_error_m=0.0)
                raw_commands.append(raw)
                plugin._last_steering = plugin._ramp_steering(
                    raw, 0.05, speed_ms=speed,
                    curvature_per_m=route.last_steering_debug.get(
                        "local_curvature", 0.0))
                applied_commands.append(plugin._last_steering)
            with self.subTest(speed=speed):
                self.assertTrue(all(command < 0.0
                                    for command in raw_commands))
                self.assertLessEqual(max(
                    abs(current - previous) for previous, current
                    in zip(raw_commands, raw_commands[1:])), 0.07)
                # The only physical shaping stage has its explicit 0.03/tick
                # bound and cannot recreate the historical +/-0.62 reversal.
                self.assertLessEqual(max(
                    abs(current - previous)
                    for previous, current in zip(
                        applied_commands, applied_commands[1:])), 0.031)

    def test_real_broad_curve_error_releases_to_tangent_not_opposite(self):
        """Regression for the 16:08:21--23 drift-then-snap drive trace.

        The captured truck reached 1.760 m from lane centre at 66 km/h while
        the command remained 0.010.  Exercise both turn directions with the
        opposing confirmed CTE on a measured 400 m-radius bend.  Releasing the
        wheel to the tangent is sufficient to cross outward relative to the
        bend; the CTE sample itself may not manufacture opposite authority.
        """
        for direction in (-1.0, 1.0):
            route = Route(self._arc(direction, 400.0, 300.0))
            position = route.points[50]
            heading = self._path_heading(position, route.points[53])
            adverse_cte = direction * 1.760
            command = route.steering(
                position, heading, 66.0 / 3.6,
                cross_track_error_m=adverse_cte)
            with self.subTest(direction=direction):
                self.assertEqual(command, 0.0)
                self.assertTrue(route.last_steering_debug[
                    "curve_sign_projection_active"])
                self.assertNotIn(
                    "opposite_correction_authorized",
                    route.last_steering_debug)

    def test_real_211258_r120_bend_never_uses_straight_recovery_gain(self):
        """The captured R119--R122 boundary has no binary gain switch."""
        for direction in (-1.0, 1.0):
            commands = []
            for nominal_radius in (119.0, 120.0, 121.0):
                route = Route(self._arc(
                    direction, nominal_radius, 260.0))
                base = route.points[50]
                following = route.points[51]
                dx = following[0] - base[0]
                dz = following[1] - base[1]
                segment_length = math.hypot(dx, dz)
                requested_cte = -direction * 1.258
                position = (
                    base[0] + dz / segment_length * requested_cte,
                    base[1] - dx / segment_length * requested_cte,
                )
                heading = self._path_heading(base, route.points[53])
                index = route.tracking_index(position, heading)
                live_cte = route.cross_track_error(index, position)
                command = route.steering(
                    position, heading, 12.5,
                    cross_track_error_m=live_cte)
                debug = route.last_steering_debug
                with self.subTest(direction=direction,
                                  radius=nominal_radius):
                    measured_radius = 1.0 / abs(debug["local_curvature"])
                    self.assertAlmostEqual(
                        measured_radius, nominal_radius, delta=0.15)
                    self.assertLess(debug["cte_geometry_residual"], 1e-6)
                    self.assertFalse(debug["straight_recovery_active"])
                    self.assertFalse(debug["curve_direction_hold"])
                    self.assertEqual(
                        math.copysign(1.0, command),
                        math.copysign(1.0, requested_cte))
                    commands.append(abs(command))
            # No R100/R120 branch may create a command discontinuity.
            self.assertLess(max(commands) - min(commands), 0.01)

    def test_real_trace_lane_error_has_explicit_controller_unit_response(self):
        """The 16:59 drive must correct before CTE reaches the lane edge.

        At 66 km/h the captured run held only a few percent of steering while
        CTE grew through 1.023, 1.147 and 1.293 m.  This regression exercises
        those measured displacements on a straight local tangent, isolating
        feedback units from curvature feed-forward.
        """
        route = Route([(0.0, 0.0), (0.0, 100.0), (0.0, 200.0)])
        commands = [route.steering(
            (0.0, 20.0), math.pi, 66.0 / 3.6,
            cross_track_error_m=cte)
            for cte in (1.023, 1.147, 1.293)]
        self.assertEqual(commands, sorted(commands))
        self.assertGreaterEqual(commands[0], 0.07)
        self.assertGreaterEqual(commands[-1], 0.10)

    def test_confirmed_lane_recovery_is_smooth_on_broad_curves_at_safe_speeds(self):
        """Game-like 20 Hz replay starts 1.5 m off-centre in both bends."""
        dt, wheelbase = 0.05, TRUCK_WHEELBASE_M
        for direction in (-1.0, 1.0):
            for radius in (120.0, 220.0, 400.0):
                speed = min(18.0, curve_speed_limit_ms(radius, 0.0))
                route = Route(self._arc(direction, radius, 350.0))
                x, z = route.points[0]
                x += 1.50
                heading = self._path_heading(
                    route.points[0], route.points[2])
                plugin = AutopilotPlugin.__new__(AutopilotPlugin)
                plugin._last_steering = 0.0
                errors, commands = [], []
                for _ in range(int(250.0 / speed / dt)):
                    index = route.tracking_index((x, z), heading)
                    live_cte = route.cross_track_error(index, (x, z))
                    target = route.steering(
                        (x, z), heading, speed,
                        cross_track_error_m=live_cte)
                    plugin._last_steering = plugin._ramp_steering(
                        target, dt, speed_ms=speed,
                        curvature_per_m=route.last_steering_debug.get(
                            "local_curvature", 0.0))
                    heading -= (speed / wheelbase
                                * plugin._last_steering
                                * NORMALIZED_STEERING_ANGLE_RAD * dt)
                    x += -math.sin(heading) * speed * dt
                    z += -math.cos(heading) * speed * dt
                    index = route.tracking_index((x, z), heading)
                    errors.append(route.cross_track_error(index, (x, z)))
                    commands.append(plugin._last_steering)
                with self.subTest(direction=direction, radius=radius):
                    self.assertLess(max(map(abs, errors)), 1.55)
                    self.assertLess(abs(errors[-1]), 0.40)
                    self.assertLessEqual(max(
                        abs(current - previous) for previous, current
                        in zip(commands, commands[1:])), 0.031)

    def test_live_lane_cte_replaces_not_duplicates_route_cte(self):
        route = Route([(0.0, 0.0), (0.0, 100.0), (0.0, 200.0)])
        # The geometric CTE is deliberately large and opposite to the supplied
        # live LaneMatch CTE. Supplying the latter must be the only CTE term.
        supplied = route.steering(
            (4.0, 20.0), math.pi, 10.0, cross_track_error_m=0.4)
        reference = route.steering(
            (0.4, 20.0), math.pi, 10.0, cross_track_error_m=0.4)
        self.assertAlmostEqual(supplied, reference, places=7)

    def test_lane_local_and_route_cte_signs_are_opposites_in_both_curves(self):
        class Network:
            def __init__(self, segment):
                self.segment = segment

            def lane_segments_near(self, _position, _radius):
                return [self.segment]

            def altitude_near(self, _position):
                return 0.0

            @staticmethod
            def lanes_connected(first, second):
                return first == second

        for direction in (-1.0, 1.0):
            with self.subTest(direction=direction):
                coordinates = self._arc(direction, 100.0, 100.0)
                lane_id = LaneId(90 + int(direction), 1, 0)
                lane_points = []
                travelled = 0.0
                for index, point in enumerate(coordinates):
                    if index:
                        travelled += math.dist(coordinates[index-1], point)
                    next_point = coordinates[min(index+1, len(coordinates)-1)]
                    previous = coordinates[max(0, index-1)]
                    lane_points.append(LanePoint(
                        point[0], 0.0, point[1], travelled,
                        self._path_heading(previous, next_point),
                        lane_id=lane_id))
                segment = LaneSegment(
                    lane_id, 1, 2, 1, 0, 1, 4.5, "derived", 0,
                    "look", "road", tuple(lane_points),
                    gps_uids=frozenset((1, 2)))
                route = Route(coordinates)
                index = 20
                first, second = coordinates[index], coordinates[index+1]
                dx, dz = second[0]-first[0], second[1]-first[1]
                length = math.hypot(dx, dz)
                offset = 0.8
                position = (first[0] - dz/length*offset,
                            first[1] + dx/length*offset)
                heading = self._path_heading(first, second)
                match = LaneLocator(Network(segment)).locate(
                    (position[0], 0.0, position[1]), heading, (1, 2))
                self.assertIsNotNone(match)
                route_cte = route.cross_track_error(
                    route.tracking_index(position, heading), position)
                self.assertAlmostEqual(
                    route_cte, -match.lateral_error_m, delta=0.03)

    def test_s_curve_changes_steering_sign_without_stale_lock(self):
        x, z, heading = 0.0, 0.0, math.pi
        points = [(x, z)]
        step = 2.0
        for index in range(96):
            curvature = (1.0 / 120.0 if index < 48 else -1.0 / 120.0)
            heading -= curvature * step
            x += -math.sin(heading) * step
            z += -math.cos(heading) * step
            points.append((x, z))
        route = Route(points)
        commands = []
        for index in range(4, len(route.points) - 8, 3):
            pos = route.points[index]
            tangent = route.points[index + 2]
            commands.append(route.steering(
                pos, self._path_heading(pos, tangent), 12.0,
                cross_track_error_m=0.0))
        self.assertTrue(any(value > 0.03 for value in commands))
        self.assertTrue(any(value < -0.03 for value in commands))
        crossings = [i for i, (a, b) in enumerate(zip(commands, commands[1:]))
                     if a*b <= 0.0]
        self.assertTrue(crossings)

    def test_speed_schedule_is_stable_at_crawl_and_motorway_speed(self):
        route = Route(self._arc(1.0, 160.0, 160.0))
        pos = route.points[15]
        heading = self._path_heading(pos, route.points[18])
        centred = [route.steering(
            pos, heading, speed, cross_track_error_m=0.0)
            for speed in (0.5, 5.0, 12.0, 25.0)]
        self.assertTrue(all(math.isfinite(value) for value in centred))
        self.assertTrue(all(abs(value) <= 0.7 for value in centred))
        low = route.steering(pos, heading, 3.0, cross_track_error_m=0.8)
        high = route.steering(pos, heading, 25.0, cross_track_error_m=0.8)
        self.assertLessEqual(abs(high), abs(low) + 1e-6)

    def test_s_curve_closed_loop_stays_inside_lane_at_multiple_speeds(self):
        x, z, heading = 0.0, 0.0, math.pi
        points = [(x, z)]
        for index in range(250):
            curvature = (1.0 / 160.0 if index < 125 else -1.0 / 160.0)
            heading -= curvature * 2.0
            x += -math.sin(heading) * 2.0
            z += -math.cos(heading) * 2.0
            points.append((x, z))
        for speed in (5.0, 12.0, 20.0, 25.0):
            with self.subTest(speed=speed):
                route = Route(points)
                plugin = AutopilotPlugin.__new__(AutopilotPlugin)
                plugin._last_steering = 0.0
                x, z, heading = 0.0, 0.0, math.pi
                errors = []
                for _ in range(int(400.0 / speed / 0.05)):
                    target = route.steering((x, z), heading, speed)
                    plugin._last_steering = plugin._ramp_steering(
                        target, 0.05, speed_ms=speed,
                        curvature_per_m=route.last_steering_debug.get(
                            "local_curvature", 0.0))
                    heading -= (speed / TRUCK_WHEELBASE_M
                                * plugin._last_steering
                                * NORMALIZED_STEERING_ANGLE_RAD * 0.05)
                    x += -math.sin(heading) * speed * 0.05
                    z += -math.cos(heading) * speed * 0.05
                    index = route.tracking_index((x, z), heading)
                    errors.append(route.cross_track_error(index, (x, z)))
                # The centre of a 4.5 m lane remains at least 0.75 m from its
                # edge even through the steering sign reversal at 90 km/h.
                self.assertLess(max(map(abs, errors)), 1.50)
                # At 90 km/h the final residual remains well inside the lane;
                # the stronger safety assertion above still limits peak CTE.
                self.assertLess(abs(errors[-1]), 0.40)

    def test_prefab_merge_split_and_roundabout_geometry_stays_local(self):
        paths = {
            "road-prefab-road": [
                (0.0, 0.0), (0.0, 20.0), (-1.0, 30.0),
                (-4.0, 40.0), (-5.0, 50.0), (-5.0, 70.0)],
            "merge-split": [
                (0.0, 0.0), (0.0, 20.0), (1.0, 30.0),
                (2.0, 40.0), (2.0, 60.0), (1.0, 70.0), (0.0, 80.0)],
            "roundabout": self._arc(1.0, 35.0, 120.0),
        }
        for label, points in paths.items():
            with self.subTest(label=label):
                route = Route(points)
                commands = []
                progresses = []
                for index in range(len(points) - 1):
                    heading = self._path_heading(points[index], points[index+1])
                    commands.append(route.steering(
                        points[index], heading, 8.0,
                        cross_track_error_m=0.0))
                    progresses.append(route._tracking_projection(
                        points[index], heading)[2])
                self.assertTrue(all(math.isfinite(v) and abs(v) <= 1.0
                                    for v in commands))
                if label == "roundabout":
                    self.assertTrue(any(0.30 < abs(v) < 0.65
                                        for v in commands))
                self.assertEqual(progresses, sorted(progresses))
    def test_route_tracking_cannot_jump_to_later_overlapping_arm(self):
        # The final arm deliberately runs almost on top of the first one in the
        # same direction. Once the truck acquired segment 0, a few centimetres
        # of lateral motion must not jump route progress through the loop.
        route = Route([
            (0.20, 0.0), (0.20, 40.0), (20.0, 40.0), (20.0, 0.0),
            (-10.0, 0.0), (-10.0, 40.0), (0.0, 40.0), (0.0, 80.0),
        ])
        heading = math.pi
        self.assertEqual(route.tracking_index((0.20, 5.0), heading), 0)
        self.assertEqual(route.tracking_index((0.0, 20.0), heading), 0)
        # All consumers in the same frame must observe the same projection.
        self.assertEqual(route.tracking_index((0.0, 20.0), heading), 0)
        route.curvature_ahead((0.0, 20.0), heading)
        self.assertEqual(route.tracking_index((0.0, 20.0), heading), 0)

    def test_route_tracking_reacquires_after_real_teleport(self):
        route = Route([(0.0, 0.0), (0.0, 40.0), (100.0, 40.0),
                       (100.0, 100.0)])
        self.assertEqual(route.tracking_index((0.0, 5.0), math.pi), 0)
        # A world-space displacement larger than the telemetry continuity
        # window is a genuine teleport/load and permits global reacquisition.
        self.assertEqual(route.tracking_index((100.0, 80.0), math.pi), 2)

    def test_lookahead_starts_at_projection_not_previous_waypoint(self):
        route = Route([(0.0, 0.0), (0.0, 20.0), (0.0, 40.0)])
        point = route.lookahead_point(0, (0.0, 15.0), 10.0)
        self.assertAlmostEqual(point[0], 0.0)
        self.assertAlmostEqual(point[1], 25.0)

    def test_lane_centre_recovery_is_damped_not_right_left_hunting(self):
        # Deterministic bicycle approximation: start 1.5 m off a straight lane
        # at 43 km/h. The controller may cross centre while settling, but must
        # not swing deeply into the other lane or keep oscillating.
        route = Route([(0.0, float(z)) for z in range(0, 501, 2)])
        x, z, heading = 1.5, 0.0, math.pi
        errors = []
        speed, dt, wheelbase = 12.0, 0.05, TRUCK_WHEELBASE_M
        for _ in range(400):
            steering = route.steering((x, z), heading, speed)
            heading -= (speed / wheelbase *
                        (steering * NORMALIZED_STEERING_ANGLE_RAD) * dt)
            x += -math.sin(heading) * speed * dt
            z += -math.cos(heading) * speed * dt
            errors.append(x)
        crossings = sum(first*second < 0.0
                        for first, second in zip(errors, errors[1:]))
        self.assertGreater(min(errors), -0.50)
        self.assertLessEqual(crossings, 4)
        self.assertLess(max(abs(error) for error in errors[-100:]), 0.06)

    def test_confirmed_curves_hold_lane_centre_on_both_sides(self):
        # The old 70 m chord-heading plus a 0.16/s steering ramp reproduced
        # the real failure: 2-7 m of drift on 80-120 m bends. Exercise the
        # complete Route target + Autopilot ramp in both turn directions.
        plugin = AutopilotPlugin.__new__(AutopilotPlugin)
        speed, dt, wheelbase = 12.0, 0.05, TRUCK_WHEELBASE_M
        for direction in (-1.0, 1.0):
            for radius in (80.0, 120.0):
                with self.subTest(direction=direction, radius=radius):
                    points = [
                        (direction * (radius - radius * math.cos(i * 2 / radius)),
                         radius * math.sin(i * 2 / radius))
                        for i in range(180)
                    ]
                    route = Route(points)
                    x, z, heading = 0.0, 0.0, math.pi
                    plugin._last_steering = 0.0
                    errors = []
                    for _ in range(200):
                        target = route.steering((x, z), heading, speed)
                        plugin._last_steering = plugin._ramp_steering(
                            target, dt, speed_ms=speed,
                            curvature_per_m=route.last_steering_debug.get(
                                "local_curvature", 0.0))
                        heading -= (speed / wheelbase
                                    * (plugin._last_steering
                                       * NORMALIZED_STEERING_ANGLE_RAD) * dt)
                        x += -math.sin(heading) * speed * dt
                        z += -math.cos(heading) * speed * dt
                        index = route.tracking_index((x, z), heading)
                        errors.append(route.cross_track_error(index, (x, z)))
                    self.assertLess(max(map(abs, errors)), 0.70)
                    self.assertLess(
                        sum(map(abs, errors[-40:])) / 40.0, 0.25)

    def test_game_like_sharp_curve_is_smooth_and_stays_inside_lane(self):
        """Closed-loop 20 Hz replay of a 45 m junction/roundabout bend."""
        radius, dt, wheelbase = 45.0, 0.05, TRUCK_WHEELBASE_M
        speed = curve_speed_limit_ms(radius, 0.0)
        for direction in (-1.0, 1.0):
            with self.subTest(direction=direction):
                route = Route(self._arc(direction, radius, 180.0))
                x, z = route.points[0]
                heading = self._path_heading(
                    route.points[0], route.points[2])
                plugin = AutopilotPlugin.__new__(AutopilotPlugin)
                plugin._last_steering = 0.0
                errors, commands = [], []
                for _ in range(int(150.0 / speed / dt)):
                    raw = route.steering((x, z), heading, speed)
                    plugin._last_steering = plugin._ramp_steering(
                        raw, dt, speed_ms=speed,
                        curvature_per_m=route.last_steering_debug.get(
                            "local_curvature", 0.0))
                    heading -= (speed / wheelbase
                                * plugin._last_steering
                                * NORMALIZED_STEERING_ANGLE_RAD * dt)
                    x += -math.sin(heading) * speed * dt
                    z += -math.cos(heading) * speed * dt
                    index = route.tracking_index((x, z), heading)
                    errors.append(route.cross_track_error(index, (x, z)))
                    commands.append(plugin._last_steering)
                self.assertAlmostEqual(
                    speed, math.sqrt(1.6 * radius), places=6)
                self.assertLess(max(map(abs, errors)), 0.80)
                self.assertLessEqual(max(
                    abs(current - previous)
                    for previous, current in zip(commands, commands[1:])),
                    0.031)

    def test_curve_recovery_tolerates_real_scs_actuator_shortfall(self):
        # The 2026-07-29 drive reached 1.9 m inward error at 68 km/h although
        # the ProMods lane line itself stayed exactly 2.25 m from road centre.
        # Reproduce the measured control shortfall by applying only 0.14 rad
        # of tyre angle per normalized command while the feed-forward model is
        # calibrated at NORMALIZED_STEERING_ANGLE_RAD. Feedback must recover
        # before the 1.80 m runtime authority boundary in either turn direction.
        dt, wheelbase = 0.05, TRUCK_WHEELBASE_M
        for direction in (-1.0, 1.0):
            for radius in (80.0, 120.0, 220.0):
                with self.subTest(direction=direction, radius=radius):
                    # The former test drove every radius at 68 km/h. The new
                    # speed envelope is part of the safety system: a loaded
                    # truck must enter the curve at its radius-safe speed.
                    speed = min(18.9, curve_speed_limit_ms(radius, 0.0))
                    route = Route(self._arc(direction, radius, 500.0))
                    x, z = route.points[0]
                    heading = self._path_heading(
                        route.points[0], route.points[2])
                    plugin = AutopilotPlugin.__new__(AutopilotPlugin)
                    plugin._last_steering = 0.0
                    errors = []
                    for _ in range(400):
                        raw = route.steering((x, z), heading, speed)
                        plugin._last_steering = plugin._ramp_steering(
                            raw, dt, speed_ms=speed,
                            curvature_per_m=route.last_steering_debug.get(
                                "local_curvature", 0.0))
                        heading -= (speed / wheelbase
                                    * plugin._last_steering * 0.14 * dt)
                        x += -math.sin(heading) * speed * dt
                        z += -math.cos(heading) * speed * dt
                        index = route.tracking_index((x, z), heading)
                        errors.append(route.cross_track_error(
                            index, (x, z)))
                    self.assertLess(max(map(abs, errors)), 1.50)
                    # Even an artificial 50% actuator-response loss remains
                    # inside the central 0.80 m; the normal calibrated model is
                    # covered by the much tighter game-like centring tests.
                    self.assertLess(
                        sum(map(abs, errors[-80:])) / 80.0, 0.80)

    def test_runtime_path_rejects_parallel_first_lane_offset(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 40)
        m.node(10, 12, 0); m.node(11, 12, 40)
        active_index = m.road(1, 2, 1)
        m.road(10, 11, 1)
        active = next(lane for lane in m.net._build_lane_segments(active_index)
                      if lane.direction == 1)
        m.net._lane_id_index[active.lane_id] = active
        match = LaneMatch(active.lane_id, active.centerline[1], 0, 1,
                          0.0, 0.0, 0.0, 0.0, 1.0, "test")

        # Force a valid GPS corridor on the nearby but disconnected road.  It
        # must be rejected rather than drawing a lateral jump to that road.
        path, returned = m.net.build_lane_path(
            (10, 11), (active.centerline[1].x, active.centerline[1].z),
            active.centerline[1].heading, start_match=match)
        self.assertIs(returned, match)
        self.assertFalse(path.valid)
        self.assertIn("does not connect", path.failure_reason)

    def test_lanes_right_is_physically_right_under_ets2_heading_convention(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 40)
        index = m.road(1, 2, 2)
        token = m.net._seg_look_tokens[index]
        look = m.net.road_looks[token]
        look.update({
            "lane_types_left": ("traffic_lane.road.local",) * 2,
            "lanes_left": 2, "lanes_right": 2, "offset_m": 2.0,
        })
        m.net._lane_cache.clear()
        lanes = m.net._build_lane_segments(index)
        right = [lane for lane in lanes if lane.direction == 1]
        left = [lane for lane in lanes if lane.direction == -1]
        # start->end is +Z; project convention says physical right is -X.
        # ETS2LA applies the full 2 m SII road_offset to both carriageways.
        self.assertEqual([round(l.centerline[0].x, 2) for l in right],
                         [-4.25, -8.75])
        self.assertEqual([round(l.centerline[-1].x, 2) for l in left],
                         [4.25, 8.75])
        self.assertEqual(right[0].width_m, 4.5)
        self.assertEqual(right[0].width_source, "derived")
        self.assertIsNone(right[0].left_neighbor)
        self.assertEqual(right[0].right_neighbor, right[1].lane_id)
        self.assertEqual(right[1].left_neighbor, right[0].lane_id)

    def test_per_lane_offsets_are_applied_after_full_road_offset(self):
        look = {
            "lane_types_left": ("traffic_lane.road.local",) * 2,
            "lane_types_right": ("traffic_lane.road.local",) * 2,
            "offset_m": 5.75,
            "lane_offsets_left_m": (-4.75, -4.75),
            "lane_offsets_right_m": (-4.75, -4.75),
        }
        left, right = RoadNetwork._lane_center_offsets(look)
        # Exact ETS2 1.59 road.blkw2c definition. Omitting lane_offsets moved
        # both carriageways 4.75 m and made a right-lane truck look left-lane.
        self.assertEqual(left, (-3.25, -7.75))
        self.assertEqual(right, (3.25, 7.75))

    def test_lateral_and_steering_signs_are_consistent(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 40)
        index = m.road(1, 2, 1)
        lane = next(l for l in m.net._build_lane_segments(index)
                    if l.direction == 1)
        point = lane.centerline[3]
        # Physical right of a +Z path is -X: locator error is positive-right.
        match = LaneLocator(m.net).locate(
            (point.x - 1.0, point.y, point.z), point.heading, (1, 2))
        self.assertGreater(match.lateral_error_m, 0.0)
        # A truck right of the target must steer left (negative).
        route = Route([[point.x, point.y, 0.0], [point.x, point.y, 80.0]])
        self.assertLess(route.steering((point.x - 1.0, point.z),
                                       point.heading, 10.0), 0.0)

    def test_gentle_curve_cannot_wind_steering_at_standstill(self):
        points = [[0.0, 0.0, 0.0], [1.0, 0.0, 10.0],
                  [3.0, 0.0, 20.0], [6.0, 0.0, 30.0],
                  [10.0, 0.0, 40.0], [15.0, 0.0, 50.0],
                  [21.0, 0.0, 60.0]]
        route = Route(points)
        heading = math.pi
        self.assertLess(route.curvature_ahead((0.0, 0.0), heading), 100.0)
        self.assertLessEqual(abs(route.steering(
            (0.0, 0.0), heading, speed_ms=0.0)), 0.22 + 1e-9)

    def test_captured_offcentre_engagement_uses_one_shallow_intercept(self):
        # 22:41:56 replay: confirmed CTE 0.809 m and heading error about 2°
        # previously saturated feedback at -1.3 before the truck moved. That
        # created a left impulse followed by an opposite centre correction.
        route = Route([[0.0, 0.0, 0.0], [0.0, 0.0, 40.0],
                       [0.0, 0.0, 80.0]])
        steering = route.steering(
            (-0.809, 0.0), math.pi + math.radians(2.0), speed_ms=0.0,
            cross_track_error_m=-0.809)
        debug = route.last_steering_debug
        self.assertTrue(debug["low_speed_capture_active"])
        self.assertLess(abs(debug["cte_steer"]), math.radians(3.0))
        self.assertLess(abs(steering), 0.08)

    def test_equal_parallel_candidates_are_rejected_without_history(self):
        def lane(uid, x):
            lid = LaneId(uid, 1, 0)
            pts = tuple(LanePoint(x, 0, z, heading=math.pi)
                        for z in (0.0, 20.0, 40.0))
            return LaneSegment(lid, 1, 2, 1, 0, 1, 4.5, "derived", 0,
                               "look", "traffic_lane.road.local", pts,
                               gps_uids=frozenset((1, 2)))
        net = type("N", (), {
            "lane_segments_near": lambda self, pos, radius: [lane(10, -1), lane(20, 1)],
            "lanes_connected": lambda self, a, b: a == b,
        })()
        self.assertIsNone(LaneLocator(net).locate((0, 0, 10), math.pi, (1, 2)))

    def test_locator_cannot_jump_to_disconnected_parallel_road(self):
        def lane(uid, x):
            lid = LaneId(uid, 1, 0)
            pts = tuple(LanePoint(x, 0, z, heading=math.pi)
                        for z in (0.0, 20.0, 40.0))
            return LaneSegment(lid, uid, uid+1, 1, 0, 1, 4.5,
                               "derived", 0, "look",
                               "traffic_lane.road.local", pts,
                               gps_uids=frozenset((uid, uid+1)))
        first, parallel = lane(10, 0.0), lane(20, 1.0)
        net = type("N", (), {
            "lanes": [first],
            "lane_segments_near": lambda self, pos, radius: self.lanes,
            "lanes_connected": lambda self, a, b: a == b,
        })()
        locator = LaneLocator(net)
        previous = locator.locate((0, 0, 10), math.pi, (10, 11))
        self.assertEqual(previous.lane_id, first.lane_id)
        net.lanes = [parallel]
        self.assertIsNone(locator.locate((1, 0, 20), math.pi, (20, 21), previous))

    def test_missing_middle_uid_and_graph_only_gap_are_rejected(self):
        m = SyntheticMap()
        m.node(1, 0, 0); m.node(2, 0, 80)
        missing = m.net.resolve_gps_corridor((1, 99, 2))
        self.assertFalse(missing.valid)
        self.assertIn("absent", missing.failure_reason)
        m.net.fwd[1] = [2]
        corridor = m.net.resolve_gps_corridor((1, 2))
        self.assertTrue(corridor.valid)
        fake_lane = LaneId(1, 1, 0)
        match = LaneMatch(fake_lane, LanePoint(0, 0, 0), 0, 0,
                          0, 0, 0, 0, 1, "test")
        segments, reason = m.net.select_lane_sequence(corridor, match)
        self.assertEqual(segments, ())
        self.assertIn("no lane-confirmed geometry", reason)

    def test_prefab_wrong_exit_and_ambiguous_roundabout_fail_closed(self):
        net = RoadNetwork(); net.loaded = True
        net.nodes.update({1: (0.0, 0.0), 2: (0.0, 10.0), 3: (10.0, 10.0)})
        net.node_alt.update({1: 0.0, 2: 0.0, 3: 0.0})
        net.node_rot.update({1: 0.0, 2: 0.0, 3: 0.0})
        token = "roundabout-test"
        net._prefab_desc[token] = (
            ((0.0, 0.0, 0.0), (0.0, 10.0, 0.0)),
            ((0.0, 0.0, 0.0, 10.0, 0.0, 1.0, 0.0, 1.0),),
            (("physical", 0, ((1, (0,)),)), ("physical", 1, ())),
        )
        net._prefab_lane_data[token] = {
            "path": "roundabout", "nodes": (
                {"input_lanes": (0,), "output_lanes": (), "y": 0.0},
                {"input_lanes": (), "output_lanes": (0,), "y": 0.0},
            ),
            "curves": ({"nav_node_index": 1, "next_lines": (),
                        "prev_lines": (), "start_y": 0.0, "end_y": 0.0},),
        }
        instance = (token, (1, 2), 0)
        wrong = GpsCorridorEdge(1, 3, "prefab", 0,
                                prefab_instance=(instance,))
        segment, reason = net._prefab_lane_segment(wrong, 0)
        self.assertIsNone(segment)
        self.assertIn("missing", reason)
        ambiguous = GpsCorridorEdge(1, 2, "prefab", 0,
                                    prefab_instance=(instance, instance))
        segment, reason = net._prefab_lane_segment(ambiguous, 0)
        self.assertIsNone(segment)
        self.assertIn("ambiguous", reason)

        # A curve chain can be geometrically connected yet belong to another
        # navNode/exit. navNodeIndex is authoritative and must reject it.
        net._prefab_lane_data[token]["curves"][0]["nav_node_index"] = 0
        segment, reason = net._prefab_lane_segment(
            GpsCorridorEdge(1, 2, "prefab", 0,
                            prefab_instance=(instance,)), 0)
        self.assertIsNone(segment)
        self.assertIn("missing", reason)

        # Geometry plus input/output/next/prev is still insufficient without
        # the navNode identity of the GPS-selected physical exit.
        net._prefab_lane_data[token]["curves"][0]["nav_node_index"] = -1
        segment, reason = net._prefab_lane_segment(
            GpsCorridorEdge(1, 2, "prefab", 0,
                            prefab_instance=(instance,)), 0)
        self.assertIsNone(segment)
        self.assertIn("missing", reason)

    def test_legacy_prefab_uses_item_node_zero_as_world_anchor(self):
        net = RoadNetwork(); net.loaded = True
        net.nodes.update({1: (100.0, 100.0), 2: (1000.0, 1000.0)})
        net.node_alt.update({1: 1.0, 2: 20.0})
        net.node_rot.update({1: 0.0, 2: 0.0})
        token = "origin-index-test"
        net._prefab_desc[token] = (
            ((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.0, 10.0, 0.0, 1.0, 0.0, 1.0),), ())
        net._prefab_lane_data[token] = {
            "nodes": ({"y": 0.0}, {"y": 0.0}),
            "curves": ({"start_y": 0.0, "end_y": 0.0},),
        }
        points = net._prefab_curve_chain_3d((token, (1, 2), 1), (0,))
        self.assertAlmostEqual(points[0].x, 110.0)
        self.assertAlmostEqual(points[0].z, 100.0)
        self.assertAlmostEqual(points[0].y, 1.0)

    def test_descriptor_order_uses_origin_node_position_and_rotation(self):
        net = RoadNetwork(); net.loaded = True
        net.nodes.update({1: (100.0, 100.0), 2: (1000.0, 1000.0)})
        net.node_rot.update({1: 0.0, 2: math.pi / 2.0})
        token = "descriptor-order-transform"
        net._prefab_desc[token] = (
            ((-10.0, 0.0, math.pi / 2.0), (0.0, 0.0, 0.0)), (), ())
        # New format: UIDs are in PPD descriptor order. originNodeIndex=1
        # therefore selects UID 1 for both translation and rotation.
        transformed = net._transform_prefab_points(
            (token, (2, 1), 1, True), ((0.0, 10.0),))
        self.assertAlmostEqual(transformed[0][0], 100.0)
        self.assertAlmostEqual(transformed[0][1], 110.0)

    def test_hud_prefab_height_comes_from_nav_curve_not_nearest_node(self):
        net = RoadNetwork(); net.loaded = True
        net.nodes[1] = (100.0, 200.0)
        net.node_alt[1] = 10.0
        net.node_rot[1] = 0.0
        token = "hud-height-test"
        net._prefab_desc[token] = (
            ((0.0, 0.0, 0.0),),
            ((0.0, 0.0, 10.0, 0.0, 1.0, 0.0, 1.0, 0.0),), ())
        net._prefab_lane_data[token] = {
            "nodes": ({"y": 2.0},),
            "curves": ({"start_y": 2.0, "end_y": 5.0},),
        }
        instance = (token, (1,), 0, False)
        net._prefab_grid[net._cell(100.0, 200.0)] = [instance]
        segments = net.prefab_segments_3d_near(
            (100.0, 200.0), radius=40.0)
        self.assertTrue(segments)
        self.assertAlmostEqual(segments[0][0][2], 10.0)
        self.assertAlmostEqual(segments[-1][1][2], 13.0)

    def test_hud_rejects_wrong_deck_vertical_chord(self):
        self.assertTrue(RoadNetwork._hud_chord_is_sane(
            (0.0, 0.0, 10.0), (2.5, 0.0, 10.4), 10.0, 0.0))
        self.assertFalse(RoadNetwork._hud_chord_is_sane(
            (0.0, 0.0, 10.0), (2.5, 0.0, 30.0), 10.0, 0.0))

    def test_hud_connected_grade_has_no_truck_centred_ninety_metre_gap(self):
        """21:48 HUD gap moved because the same chord changed at radius 90 m."""
        grade = ((0.0, 0.0, 112.89), (3.04, 0.0, 113.04))
        inside = RoadNetwork._hud_chord_is_sane(
            *grade, altitude=109.0, distance2=89.9 ** 2)
        outside = RoadNetwork._hud_chord_is_sane(
            *grade, altitude=109.0, distance2=90.1 ** 2)
        self.assertTrue(inside)
        self.assertEqual(inside, outside)


class TrajectoryNegativeAuditTests(unittest.TestCase):
    def test_zero_duplicate_reversed_nan_and_infinity_are_rejected(self):
        cases = (
            ([(0, 0, 0), (0, 0, 0)], "duplicate"),
            ([(0, 0, 0), (0, 0, 10), (0, 0, 5)], "reverses direction"),
            ([(0, 0, 0), (math.nan, 0, 10)], "non-finite"),
            ([(0, 0, 0), (math.inf, 0, 10)], "non-finite"),
        )
        for coordinates, reason in cases:
            with self.subTest(reason=reason):
                result = build_lane_trajectory(single_lane_path(coordinates))
                self.assertFalse(result.valid)
                self.assertIn(reason, result.failure_reason)


class SnapshotAndConsumerAuditTests(unittest.TestCase):
    def test_delayed_old_build_cannot_overwrite_changed_target(self):
        plugin, sdk, point = build_map_plugin()
        original = plugin.road_net.build_lane_path
        sdk.set("game_route_node_uids", [1, 2])

        def delayed_old(*args, **kwargs):
            result = original(*args, **kwargs)
            new_revision = sdk.get("lane_trajectory_revision") + 1
            sdk.shared_state.update_batch({
                "game_route_node_uids": [2, 3],
                "lane_trajectory_revision": new_revision,
                "lane_trajectory": {
                    "revision": new_revision, "valid": False,
                    "confidence": 0.0, "points": [], "display_points": [],
                    "source_gps_uids": [2, 3],
                    "failure_reason": "new-target-wins",
                },
            })
            return result

        plugin.road_net.build_lane_path = delayed_old
        plugin._update_lane_trajectory((point.x, point.z), point.heading)
        self.assertEqual(sdk.get("lane_trajectory")["failure_reason"],
                         "new-target-wins")
        self.assertEqual(sdk.get("lane_trajectory")["source_gps_uids"], [2, 3])

    def test_map_or_telemetry_loss_hides_consumers_and_blocks_autopilot(self):
        plugin, sdk, _ = build_map_plugin()
        sdk.set("lane_trajectory_heartbeat", time.monotonic() - 2.0)
        self.assertEqual(UltraPilotHUD._read(hud_reader(sdk.shared_state))["nav_path"], [])
        ar = type("ARReader", (), {"state": sdk.shared_state})()
        self.assertEqual(AROverlay._current_display_points(ar), (-1, []))
        ap, state = autopilot_state(0.95, heartbeat=time.monotonic() - 2.0)
        ap.on_tick(0.1)
        self.assertEqual(state.get("autopilot_lane_revision"), -1)

        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = State({
            "autopilot_active": True, "telemetry_valid": False,
            "ctl_steering": 0.8, "ctl_throttle": 1.0, "ctl_brake": 0.0,
        })
        engine.controller = Controller()
        engine._was_active = True
        engine._flush_controls()
        self.assertGreater(engine.controller.steering, 0.0)
        self.assertLess(engine.controller.steering, 0.8)
        self.assertEqual(engine.controller.throttle, 0.0)
        self.assertGreater(engine.controller.brake, 0.0)
        self.assertLess(engine.controller.brake, 0.70)
        for _ in range(20):
            engine._last_control_flush -= 0.1
            engine._flush_controls()
        self.assertEqual(engine.controller.steering, 0.0)
        self.assertEqual(engine.controller.brake, 0.70)

        engine.shared_state = State({
            "autopilot_active": True, "telemetry_valid": True,
            "autopilot_control_heartbeat": time.monotonic() - 1.0,
            "ctl_steering": 0.8, "ctl_throttle": 1.0, "ctl_brake": 0.0,
        })
        engine.controller = Controller()
        engine._last_output_steering = 0.8
        engine._last_output_brake = 0.0
        engine._last_control_flush = time.monotonic() - 0.02
        engine._flush_controls()
        self.assertGreater(engine.controller.steering, 0.0)
        self.assertLess(engine.controller.steering, 0.8)
        self.assertEqual(engine.controller.throttle, 0.0)
        self.assertGreater(engine.controller.brake, 0.0)
        self.assertLess(engine.controller.brake, 0.70)
        ap, state = autopilot_state(0.95, telemetry_valid=False)
        ap.on_tick(0.1)
        self.assertEqual(state.get("autopilot_lane_revision"), -1)

    def test_confidence_threshold_below_equal_and_above_is_unambiguous(self):
        for confidence, expected in ((0.719999, -1), (0.72, 7), (0.720001, 7)):
            with self.subTest(confidence=confidence):
                plugin, state = autopilot_state(confidence)
                plugin.on_tick(0.05)
                self.assertEqual(state.get("autopilot_lane_revision"), expected)
                readiness = state.get("autopilot_navigation_readiness")
                self.assertEqual(readiness["ready"], expected == 7)
                if confidence < 0.72:
                    self.assertIn("below 0.72", readiness["reason"])
                else:
                    self.assertEqual(readiness["reason"], "")

    def test_camera_matrix_requires_proven_metadata_and_time_sync(self):
        now = time.monotonic()
        state = State({"telemetry_timestamp": now})
        overlay = type("Projection", (), {
            "state": state, "width": lambda self: 100,
            "height": lambda self: 100,
        })()
        self.assertIsNone(AROverlay._project_world(overlay, [0, 0, 0]))
        state.update_batch({
            "game_camera_view_projection": [1.0] * 15,
            "game_camera_view_projection_meta": {
                "layout": "row-major", "handedness": "right-handed",
                "clip_space": "opengl-negative-one-to-one", "timestamp": now,
            },
        })
        self.assertIsNone(AROverlay._project_world(overlay, [0, 0, 0]))
        state.set("game_camera_view_projection", [math.nan] + [0.0] * 14 + [1.0])
        self.assertIsNone(AROverlay._project_world(overlay, [0, 0, 0]))

    def test_concurrent_publication_never_accepts_mixed_revision(self):
        manager = None
        try:
            manager = mp.Manager()
            raw_state = manager.dict()
        except (OSError, PermissionError):
            # Restricted CI/sandboxes can deny Windows named pipes. The same
            # publication/read protocol is still exercised in-process there;
            # the audit also runs this test unsandboxed against Manager.dict.
            raw_state = {}
        shared = SharedState(raw_state)
        try:
            shared.update_batch({
                "lane_trajectory_revision": 0,
                "lane_trajectory": {"revision": 0, "points": [[0, 0, 0]]},
            })
            failures = []

            def writer():
                for revision in range(1, 250):
                    shared.update_batch({
                        "lane_trajectory_revision": revision,
                        "lane_trajectory": {
                            "revision": revision,
                            "points": [[revision, revision, revision]],
                        },
                    })

            thread = threading.Thread(target=writer)
            thread.start()
            while thread.is_alive():
                snapshot = shared.get("lane_trajectory", {})
                revision = shared.get("lane_trajectory_revision", -1)
                # Mismatches are rejected by consumers. If revisions agree,
                # the nested geometry must belong to that same publication.
                if snapshot.get("revision") == revision:
                    if snapshot.get("points") != [[revision, revision, revision]]:
                        failures.append((snapshot, revision))
            thread.join()
            self.assertEqual(failures, [])
        finally:
            if manager is not None:
                manager.shutdown()


if __name__ == "__main__":
    unittest.main()
