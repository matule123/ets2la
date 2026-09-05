"""Independent plant and IPC regressions for the complete steering audit."""
import json
import io
import math
from pathlib import Path
import struct
import time
import unittest
from unittest import mock
from types import SimpleNamespace

from core.lateral_controller import solve, offset_frame
from core.navigation.lane_model import LaneId, LaneLocator, LanePoint, LaneSegment
from core.navigation.route import Route
from core.sdk.scs_sdk import SCSTelemetry
from core.sdk.scs_controller_writer import SCSControlsWriter, _FIELDS, _SIZE
from core.steering_dynamics import SteeringDynamics
from core.telemetry import Telemetry
from tests.steering_bench import path, run
from tests.test_control_safety_regressions import autopilot, ready_navigation_state
from tools.run_steering_bench import cases


class SteeringSystemAuditTests(unittest.TestCase):
    def test_left_error_to_right_command_sign_across_complete_control_chain(self):
        lane_id = LaneId(1, 1, 0)
        points = (LanePoint(0., 0., 0., heading=math.pi, lane_id=lane_id),
                  LanePoint(0., 0., 20., heading=math.pi, lane_id=lane_id))
        lane = LaneSegment(lane_id, 1, 2, 1, 0, 1, 4.5, 'dataset', 0,
                           None, 'road', points)
        lane_match_cte = LaneLocator._project((.5, 0., 5.), lane)[4]
        # LaneLocator's historical ABI is RIGHT-positive. Map converts it once
        # to Route's LEFT-positive Frenet error.
        self.assertLess(lane_match_cte, 0.)
        route_cte = -lane_match_cte
        route = Route([(0., 0.), (0., 20.)])
        raw = route.steering((.5, 5.), math.pi, 8.,
                             cross_track_error_m=route_cte,
                             actuator_response_s=0.)
        self.assertGreater(raw, 0.)  # right command corrects a left error
        dynamics = SteeringDynamics()
        output = dynamics.update(raw, .05, speed_ms=8.)
        self.assertGreater(output, 0.)

        writer = SCSControlsWriter.__new__(SCSControlsWriter)
        writer.invert_steering = False
        writer.connected = True
        writer._buf = io.BytesIO(bytearray(sum(_SIZE[t] for _, t in _FIELDS)))
        writer._offsets = {}
        offset = 0
        for name, kind in _FIELDS:
            writer._offsets[name] = offset
            offset += _SIZE[kind]
        writer.set_steering(output)
        writer._buf.seek(writer._offsets['steering'])
        self.assertAlmostEqual(struct.unpack('f', writer._buf.read(4))[0],
                               output, places=6)

        # Telemetry feedback ABIs are left-positive and therefore each convert
        # once at their boundary to the right-positive controller convention.
        normalized = Telemetry.__new__(Telemetry)._normalize_sdk({
            'truckFloat': {
                'gameSteer': -output,
                'wheelSteeringTurns': [-.02],
            },
            'wheelSteerable': [True],
            'yawRateTurnsPerSecond': -.01,
        })['truck']
        self.assertGreater(-normalized['gameSteer'], 0.)
        self.assertGreater(normalized['roadWheelAnglesRad'][0], 0.)
        self.assertGreater(normalized['yawRateRadS'], 0.)

    def test_nonfinite_pose_or_cte_cannot_be_hidden_by_a_clamp(self):
        route=Route([(0.,0.),(0.,50.)])
        for value in (float('nan'),float('inf'),-float('inf')):
            command=route.steering((0.,10.),math.pi,8.,cross_track_error_m=value)
            self.assertEqual(command,0.)
            self.assertFalse(route.last_steering_debug['authority_valid'])

    def test_worker_dt_uses_elapsed_time_not_adjustable_wall_clock(self):
        from core.plugin_manager import plugin_worker
        observed=[]
        class Worker:
            enabled=True
            def __init__(self, sdk): pass
            def on_start(self): pass
            def on_stop(self): pass
            def on_tick(self, dt): observed.append(dt)
        event=SimpleNamespace(is_set=lambda:len(observed)>=2)
        with mock.patch('core.plugin_manager.PluginSDK'), \
                mock.patch('core.plugin_manager.logging.basicConfig'), \
                mock.patch('core.plugin_manager.time.sleep'), \
                mock.patch('core.plugin_manager.time.monotonic', side_effect=[10.,10.05,10.1]), \
                mock.patch('core.plugin_manager.time.time', side_effect=[100.,90.,90.05]):
            plugin_worker(Worker,'clock-test',{},event)
        self.assertEqual(len(observed),2)
        for dt in observed:
            self.assertAlmostEqual(dt,.05)

    def test_map_observation_keeps_xyz_heading_and_speed_from_one_frame(self):
        from tests.test_lane_authority_integration import build_map_plugin, Tags
        plugin,sdk,point=build_map_plugin()
        captured=[]
        sdk.set('truck_world_pos',(999.,999.))
        sdk.set('truck_heading',point.heading+1.)
        sdk.set('truck_speed_ms',100.)
        sdk.set('truck_altitude',100.)
        sdk.set('vehicle_envelope_snapshot',dict(timestamp=time.monotonic(),
            tractor_position=[point.x,point.y,point.z],tractor_heading=point.heading,
            tractor_speed_ms=10.))
        plugin._load_road_net=lambda:None
        plugin._update_lane_trajectory=lambda *args:captured.append(args)
        plugin._publish_road_type=lambda *_args:None
        plugin.tags=Tags()
        plugin._roads_t=plugin._diag_t=0.
        plugin.on_tick(.01)
        self.assertEqual(captured, [((point.x,point.z),point.heading,point.y)])

    def test_independent_plant_before_after(self):
        baseline = json.loads((Path(__file__).parent / 'fixtures' /
            'steering-baseline-e8d5c57.json').read_text())['cases']
        self.assertTrue(baseline['straight_90']['lost_lane'])
        self.assertTrue(baseline['log_speed_R22_lag0.5']['lost_lane'])
        for name, data, options in cases():
            with self.subTest(name=name):
                measured, rows = run(data, **options)
                self.assertTrue(measured['completed'])
                self.assertFalse(measured['lost_lane'])
                self.assertEqual(measured['monotone_opposite_samples'], 0)
                # Cab body fits a 4.5 m lane even with the deliberate outward
                # tractor reference needed by the R18 trailer's swept path.
                self.assertLess(measured['max_cte'] + 1.275, 2.25)
                self.assertLess(measured['steady_cte'], .25)
                self.assertLess(measured['max_heading_deg'], 4.)
                # Existing actuator limits, not thresholds fitted to results.
                self.assertLessEqual(measured['max_rate'], .60 + 1e-8)
                self.assertLessEqual(measured['max_accel'], 14. + 1e-8)
                # Acceleration is bounded, not separately jerk-limited. The
                # discrete derivative cannot exceed a full +/-14 change.
                self.assertLessEqual(measured['max_jerk'],
                    28. / min(row['dt'] for row in rows) + 1e-8)

    def test_uncalibrated_gain_load_and_delay_are_not_the_controller_constants(self):
        for lock in (.70, .85):
            for sign in (-1, 1):
                for speed, radius in ((2.78, 35), (16.67, 200), (25., 400)):
                    with self.subTest(lock=lock, sign=sign, speed=speed):
                        data = path([(0, 55), (sign/radius, radius*1.5), (0, 100)])
                        measured, rows = run(data, speed, lock_rad=lock,
                            load=1., noisy=True, jitter=True)
                        self.assertTrue(measured['completed'])
                        self.assertLess(measured['max_cte'], .70)
                        self.assertLess(measured['steady_cte'], .25)
                        # Exclude only the geometric entry/exit preview region,
                        # never samples based on their steering or CTE result.
                        middle = [r for r in rows if 75 < r['s'] < data['boundaries'][1]-20]
                        self.assertTrue(middle)
                        self.assertTrue(all(r['out']*sign > 0 for r in middle))

    def test_emergency_braking_closed_loop_keeps_latest_lateral_demand(self):
        def speed(t):
            if t < 10: return 8.
            if t < 13.5: return 8 - 2*(t-10)
            if t < 18: return 1.
            return min(8., 1+(t-18))
        data = path([(0, 55), (-1/40, 120), (0, 90)])
        measured, rows = run(data, speed_profile=speed, trailer=True,
                             noisy=True, jitter=True)
        self.assertTrue(measured['completed'])
        self.assertLess(measured['max_cte'], .70)
        braking = [r for r in rows if 10 < r['t'] < 13.5]
        self.assertGreater(max(r['out'] for r in braking)-min(r['out'] for r in braking), .005)

    def test_drive_selector_wait_executes_current_command_packet(self):
        now = time.monotonic()
        state = ready_navigation_state(system_state='CRUISE', nav_active=True,
            nav_steering=.4, nav_steering_debug=dict(
                controller='frenet_bicycle', output=-.25,
                local_curvature=-.02, authority_valid=True,
                authority_revision=7, navigation_intent_id=None,
                computed_at=now, observation_timestamp=now,
                route_build_id='test-build',
                source_game_session_id='test-session',
                source_map_key='test-map',
                source_dataset_fingerprint='test-fingerprint'))
        plugin = autopilot({'speed': 0., 'gear': 0, 'gameSteer': 0.}, state)
        plugin.on_tick(.05)
        self.assertLess(plugin.sdk.controller.steering, 0.)
        self.assertAlmostEqual(plugin.sdk.controller.throttle, 0.)
        self.assertAlmostEqual(plugin.sdk.controller.brake, 0.)

    def test_frenet_signs_and_nonlinear_error_equation(self):
        for k in (-1/18, -1/35, 0., 1/83, 1/18):
            for e in (-.60, 0., .60):
                for h in (-.025, 0., .025):
                    for v in (2.78, 8.33, 25.):
                        command, debug = solve(k, k, e, h, v, response_s=0.)
                        self.assertTrue(debug['valid'])
                        actual = math.tan(command*debug['lock_rad'])/3.8
                        e_dot = v*math.sin(h)
                        h_dot = v*k*math.cos(h)/(1+k*e)-v*actual
                        e_ddot = v*math.cos(h)*h_dot
                        ell = debug['feedback_length_m']
                        self.assertAlmostEqual(e_ddot + 2*v/ell*e_dot + (v/ell)**2*e,
                                               0., places=10)
                        if e == h == 0:
                            self.assertAlmostEqual(actual, k, places=12)

    def test_target_frame_is_not_a_second_lane(self):
        for k in (-1/18, 1/18):
            e = math.copysign(.8, k)
            h, target_k = offset_frame(k, 0., e, 0., 0.)
            self.assertEqual(h, 0.)
            self.assertAlmostEqual(target_k, k/(1+k*e))
        self.assertEqual(offset_frame(0., 0., 0., 0., 0.), (0., 0.))

    def test_sdk_wheel_yaw_units_and_signs_using_real_byte_layout(self):
        for sign in (-1, 1):
            sdk = SCSTelemetry()
            sdk.mm = bytearray(sdk.mmap_size)
            # SCS positive = counterclockwise/left; controller = right.
            struct.pack_into('f', sdk.mm, 1200, sign*.02)
            struct.pack_into('?', sdk.mm, 1500, True)
            struct.pack_into('f', sdk.mm, 1884, sign*.01)
            raw = sdk.update()
            normalized = Telemetry.__new__(Telemetry)._normalize_sdk(raw)['truck']
            self.assertEqual(len(normalized['roadWheelAnglesRad']), 1)
            self.assertAlmostEqual(normalized['roadWheelAnglesRad'][0], -sign*.02*math.tau)
            self.assertAlmostEqual(normalized['yawRateRadS'], -sign*.01*math.tau)
            self.assertTrue(normalized['yawRateValid'])
        normalized = Telemetry.__new__(Telemetry)._normalize_sdk({})['truck']
        self.assertFalse(normalized['yawRateValid'])
        self.assertEqual(normalized['roadWheelAnglesRad'], [])

    def test_fresh_command_packet_wins_over_a_different_scalar_tick(self):
        for mode in ('CRUISE', 'EMERGENCY', 'PAY_TOLL'):
            with self.subTest(mode=mode):
                state = ready_navigation_state(system_state=mode, nav_active=True,
                    nav_steering=.40, nav_steering_debug=dict(
                        controller='frenet_bicycle', output=-.15, local_curvature=-.02,
                        authority_valid=True, authority_revision=7,
                        navigation_intent_id=None, computed_at=time.monotonic(),
                        observation_timestamp=time.monotonic(),
                        route_build_id='test-build',
                        source_game_session_id='test-session',
                        source_map_key='test-map',
                        source_dataset_fingerprint='test-fingerprint'))
                plugin = autopilot({'speed': 6., 'gear': 4}, state)
                plugin.on_tick(.05)
                self.assertLess(plugin.sdk.controller.steering, 0.)

    def test_old_command_cannot_borrow_a_new_trajectory_heartbeat(self):
        for changes in ({'authority_revision': 6}, {'navigation_intent_id': 'old'},
                        {'computed_at': time.monotonic()-1.},
                        {'observation_timestamp': time.monotonic()-1.},
                        {'output': float('nan')}):
            with self.subTest(changes=changes):
                packet = dict(controller='frenet_bicycle', output=.4, local_curvature=.02,
                    authority_valid=True, authority_revision=7, navigation_intent_id=None,
                    computed_at=time.monotonic(), observation_timestamp=time.monotonic(),
                    route_build_id='test-build', source_game_session_id='test-session',
                    source_map_key='test-map',
                    source_dataset_fingerprint='test-fingerprint')
                packet.update(changes)
                state = ready_navigation_state(nav_active=True, nav_steering=.4,
                                               nav_steering_debug=packet)
                plugin = autopilot({'speed': 6., 'gear': 4}, state)
                plugin.on_tick(.05)
                self.assertFalse(state.get('autopilot_navigation_readiness')['ready'])
                self.assertEqual(plugin.sdk.controller.throttle, 0.)
                self.assertGreater(plugin.sdk.controller.brake, 0.)

    def test_command_packet_cannot_cross_build_session_or_dataset_identity(self):
        for key in ('route_build_id', 'source_game_session_id',
                    'source_map_key', 'source_dataset_fingerprint'):
            with self.subTest(key=key):
                state = ready_navigation_state(nav_active=True, nav_steering=.4)
                snapshot = state.get('lane_trajectory')
                packet = dict(controller='frenet_bicycle', output=.4,
                    local_curvature=.02, authority_valid=True,
                    authority_revision=7, navigation_intent_id=None,
                    computed_at=time.monotonic(),
                    observation_timestamp=time.monotonic(),
                    route_build_id=snapshot['route_build_id'],
                    source_game_session_id=snapshot['source_game_session_id'],
                    source_map_key=snapshot['source_map_key'],
                    source_dataset_fingerprint=snapshot['source_dataset_fingerprint'])
                packet[key] = 'stale'
                state.set('nav_steering_debug', packet)
                plugin = autopilot({'speed': 6., 'gear': 4}, state)
                plugin.on_tick(.05)
                self.assertFalse(state.get('autopilot_navigation_readiness')['ready'])
                self.assertEqual(plugin.sdk.controller.throttle, 0.)
                self.assertGreater(plugin.sdk.controller.brake, 0.)

    def test_snapshot_cannot_cross_session_map_or_dataset_identity(self):
        checks = (
            ('game_session_id', 'other-session'),
            ('active_map_key', 'other-map'),
            ('active_dataset_fingerprint', 'other-fingerprint'),
        )
        for key, value in checks:
            with self.subTest(key=key):
                state = ready_navigation_state(**{key: value})
                plugin = autopilot({'speed': 6., 'gear': 4}, state)
                plugin.on_tick(.05)
                self.assertFalse(state.get('autopilot_navigation_readiness')['ready'])
                self.assertEqual(plugin.sdk.controller.throttle, 0.)
                self.assertGreater(plugin.sdk.controller.brake, 0.)

    def test_runtime_steering_calibration_is_bounded_and_fail_closed(self):
        route = Route([(0., 0.), (0., 50.)])
        for invalid in (.59, .951, float('nan'), float('inf')):
            with self.subTest(invalid=invalid):
                self.assertEqual(route.steering(
                    (0., 5.), math.pi, 5., steering_lock_rad=invalid), 0.)
                self.assertFalse(route.last_steering_debug['authority_valid'])
                self.assertEqual(route.last_steering_debug['control_failure'],
                                 'invalid actuator calibration')
        for valid in (.60, .78, .95):
            with self.subTest(valid=valid):
                command = route.steering(
                    (.2, 5.), math.pi, 5., steering_lock_rad=valid,
                    actuator_response_s=0.)
                self.assertTrue(math.isfinite(command))
                self.assertTrue(route.last_steering_debug['authority_valid'])

    def test_r18_narrow_trailer_envelope_remains_explicitly_unproven(self):
        data = path([(0, 55), (-1/18, 18*math.pi), (0, 80)])
        measured, _ = run(data, 5.4, noisy=True, jitter=True, trailer=True)
        half_trailer_width = 1.275
        half_lane_width = 2.25
        self.assertGreater(measured['trailer_max_cte'] + half_trailer_width,
                           half_lane_width)


if __name__ == '__main__':
    unittest.main()
