"""SDK chassis-origin reproduction; the plant reports a point ahead of rear axle."""
import math
import copy
import struct
import unittest

from core.lateral_controller import solve
from core.navigation.route import Route
from core.sdk.scs_sdk import SCSTelemetry
from core.telemetry import Telemetry
from core.vehicle_geometry import sdk_reference_geometry
from tests.steering_bench import path, run


class ReferencePointTests(unittest.TestCase):
    @staticmethod
    def sdk_geometry():
        return dict(truckWheelCount=4, wheelPositionX=[-1.,1.,-1.,1.],
            wheelPositionZ=[-1.7,-1.7,2.1,2.1],
            wheelSteerable=[True,True,False,False])

    def test_real_dense_pose_has_nonzero_transverse_velocity_at_chassis_origin(self):
        # Unmodified Sept 10 15:18:47.440 / .716 samples. SDK times, not
        # export/apply times. revision=8, road_uid=5962819247976890548.
        xz1=(37163.83401489258,58739.60398864746)
        xz2=(37165.00003051758,58740.505859375)
        h1,h2=-2.082005910554579,-2.1571852004280503
        dt=(207358372-207108382)/1e6
        vx,vz=((b-a)/dt for a,b in zip(xz1,xz2))
        h=(h1+h2)/2
        transverse=-math.cos(h)*vx+math.sin(h)*vz
        ahead=transverse/((h2-h1)/dt)
        self.assertGreater(ahead,2.0)
        self.assertLess(ahead,2.25)
        # Original logged inputs/output remain reproducible with rear-axle
        # assumption. This is input replay, not a claim of new game behaviour.
        old,_=solve(.04951742980832228,.04514074776782407,
                    -2.246763414818357,.10738069029650132,21.9214/3.6)
        self.assertAlmostEqual(old,.2689122969439746,places=10)

    def test_circle_equilibrium_is_body_angle_not_path_tangent(self):
        for radius in (18.,22.,35.,83.,250.):
            for sign in (-1,1):
                k=sign/radius
                a,wb=2.1,3.8
                body_angle=math.asin(a*k)
                command,debug=solve(k,k,0.,body_angle,6.,
                    reference_ahead_m=a,wheelbase_m=wb)
                expected_k=k/math.sqrt(1-(a*k)**2)
                self.assertAlmostEqual(command,math.atan(wb*expected_k)/.70,places=12)
                self.assertAlmostEqual(debug['heading_feedback'],0.,places=12)
                self.assertAlmostEqual(debug['cte_feedback'],0.,places=12)
                # Verify actual observed-point transverse velocity, not only
                # a convenient expected command from the same controller.
                self.assertAlmostEqual(math.sin(body_angle)-a*expected_k*math.cos(body_angle),0.)
                old,_=solve(k,k,0.,body_angle,6.)
                self.assertGreater(abs(old),abs(command)*1.4)

    def test_axle_geometry_abi_and_normalisation(self):
        reader=SCSTelemetry()
        reader.mm=bytearray(32768)
        struct.pack_into('i',reader.mm,80,4)
        struct.pack_into('4f',reader.mm,1676,-1.,1.,-1.,1.)
        struct.pack_into('4f',reader.mm,1804,-1.7,-1.7,2.1,2.1)
        struct.pack_into('4?',reader.mm,1500,True,True,False,False)
        raw=reader.update()
        geometry=Telemetry.__new__(Telemetry)._normalize_sdk(raw)['truck']['referenceGeometry']
        self.assertTrue(geometry['valid'],geometry)
        self.assertAlmostEqual(geometry['wheelbase_m'],3.8,places=6)
        self.assertAlmostEqual(geometry['reference_ahead_m'],2.1,places=6)

    def test_missing_malformed_tandem_and_rear_steer_are_not_guessed(self):
        base=self.sdk_geometry()
        for changes in ({'truckWheelCount':0},{'truckWheelCount':True},
                {'wheelPositionZ':[]},{'wheelPositionZ':[0,0,float('nan'),2.]},
                {'wheelPositionZ':[1.7,1.7,-2.1,-2.1]},
                {'wheelPositionX':[1.,1.,1.,1.]},
                {'wheelPositionX':[-100.,100.,-100.,100.]},
                {'wheelSteerable':[True]*4},
                {'truckWheelCount':6,'wheelPositionX':[-1.,1.,-1.,1.,-1.,1.],
                 'wheelPositionZ':[-1.7,-1.7,2.1,2.1,3.2,3.2],
                 'wheelSteerable':[True,True,False,False,False,False]}):
            with self.subTest(changes=changes):
                geometry=sdk_reference_geometry(dict(base,**changes))
                self.assertFalse(geometry['valid'],geometry)
                self.assertTrue(geometry['reason'])
        self.assertFalse(sdk_reference_geometry({})['valid'])
        # Changing yaw, tyre or trailer readings cannot change chassis geometry.
        self.assertEqual(sdk_reference_geometry(base),sdk_reference_geometry(dict(
            base, yawRateTurnsPerSecond=99., trailer={'attached':True})))

    def test_invalid_geometry_fails_closed_in_route_and_packet_without_scalar_fallback(self):
        from plugins.autopilot.main import navigation_command
        route=Route([(0.,0.),(0.,-100.)])
        for geometry in ({},dict(valid=False,reason='unknown chassis'),
                dict(valid=True,wheelbase_m=3.8,reference_ahead_m=float('nan'))):
            raw=route.steering((0.,-10.),0.,10.,reference_geometry=geometry)
            self.assertEqual(raw,0.)
            packet=dict(route.last_steering_debug,calculation_packet_schema_version=1)
            self.assertFalse(packet['authority_valid'])
            target,_,reason=navigation_command({'nav_steering':.7},{},
                gps_active=True,now=1.,packet=packet)
            self.assertEqual(target,0.)
            self.assertIn(packet['control_failure'],reason)

    def test_no_actuator_feedback_or_state_can_pulse_constant_bend(self):
        for sign in (-1,1):
            k=sign/35
            expected=None
            for i in range(100):
                result=solve(k,k,.02,math.asin(2.1*k),8.,
                    vehicle_curvature_per_m=.05*math.sin(i),reference_ahead_m=2.1)
                if expected is None:
                    expected=result[0]
                self.assertEqual(result[0],expected)
                self.assertEqual(result[1]['curvature_observation_weight'],0.)

    def test_observed_cab_point_stays_centred_in_both_r22_bends(self):
        # Sept 10: pose/yaw independently identifies 2.04..2.14 m, not 0 m.
        # No changes to the lane, controller gain, lag, or feedback thresholds.
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                data=path([(0,55),(sign/22,100),(0,100)])
                result, rows=run(data, speed=6., lock_rad=.70,
                    controller_lock_rad=.70, observation_ahead_m=2.1)
                self.assertTrue(result['completed'])
                self.assertLess(result['max_cte'], .70)
                middle=[r for r in rows if 85<r['s']<135]
                self.assertLess(max(abs(r['cte']) for r in middle), .25)
                self.assertEqual(result['monotone_opposite_samples'],0)
                self.assertLess(result['steady_cte'],.25)

    def test_origin_aware_loop_radii_loads_gains_responses_and_noise(self):
        for radius,speed in ((18,5.4),(35,7.4),(83,12.),(250,25.)):
            for sign in (-1,1):
                # run() adds 80 ms for a full load. Total response here is
                # .25 / .60 s, not an accidentally double-counted .68 s.
                for lock,lag,trailer in ((.60,.25,False),(.95,.52,True)):
                    with self.subTest(radius=radius,sign=sign,lock=lock):
                        data=path([(0,55),(sign/radius,radius*1.8),(0,300)])
                        measured,rows=run(data,speed,lock_rad=lock,lag=lag,
                            controller_lock_rad=lock,observation_ahead_m=2.1,
                            trailer=trailer,load=float(trailer),noisy=True,jitter=True)
                        self.assertTrue(measured['completed'])
                        self.assertFalse(measured['lost_lane'])
                        self.assertLess(measured['max_cte'],.70)
                        # Historical 'steady_cte' means s>exit+15 m: at 90
                        # km/h that starts only .6 s into the transient, NOT
                        # after settling. Keep that benchmark metric intact;
                        # additionally assert the entire tail after 4 seconds.
                        exit_t=next(r['t'] for r in rows if r['s']>data['boundaries'][1])
                        tail=[r for r in rows if r['t']>exit_t+4.]
                        self.assertTrue(tail)
                        self.assertLess(max(abs(r['cte']) for r in tail),.25)
                        self.assertEqual(measured['monotone_opposite_samples'],0)
                        self.assertLessEqual(measured['max_rate'],.60+1e-8)
                        self.assertLessEqual(measured['max_accel'],14.+1e-8)

    def test_unidentified_extremely_slow_actuator_is_not_certified(self):
        # Explicit unresolved sensitivity, NOT a successful comfort/settling
        # acceptance test. A .68 s plant at 90 km/h is outside the nominal
        # .32 s identified vehicle; keep the failed .25 m target visible.
        measured,rows=run(path([(0,55),(-1/250,450),(0,350)]),25.,
            lock_rad=.95,lag=.60,load=1.,trailer=True,controller_lock_rad=.95,
            observation_ahead_m=2.1,noisy=True,jitter=True)
        self.assertLess(measured['max_cte'],.70)
        exit_t=next(r['t'] for r in rows if r['s']>505)
        self.assertGreater(max(abs(r['cte']) for r in rows if r['t']>exit_t+8),.25)

    def test_reference_geometry_is_not_retained_across_revision_or_new_chassis(self):
        route=Route([(0.,0.),(0.,-100.)])
        points=copy.deepcopy(route.points)
        first=sdk_reference_geometry(self.sdk_geometry())
        second=dict(first,reference_ahead_m=1.5,wheelbase_m=3.2)
        for revision,geometry in ((1,first),(2,second),(3,first)):
            route.steering((.1,-20.),0.,8.,reference_geometry=geometry,
                control_authority={'revision':revision,'lane_identity':'road'})
            self.assertEqual(route.last_steering_debug['reference_geometry'],geometry)
            self.assertEqual(route.last_steering_debug['authority_revision'],revision)
        self.assertEqual(route.points,points)

    def test_map_preserves_exact_geometry_in_calculation_packet(self):
        import time
        from tests.test_lane_authority_integration import build_map_plugin, Tags
        plugin,sdk,point=build_map_plugin()
        geometry=sdk_reference_geometry(self.sdk_geometry())
        sdk.set('vehicle_envelope_snapshot',dict(timestamp=time.monotonic(),sdk_frame_us=120000,
            tractor_position=[point.x,point.y,point.z],tractor_heading=point.heading,
            tractor_speed_ms=8.,tractor_reference_geometry=geometry))
        sdk.set('truck_world_pos',(point.x,point.z))
        sdk.set('telemetry_valid',True)
        plugin.tags=Tags()
        plugin._load_road_net=lambda:None
        plugin._publish_road_type=lambda *_:None
        plugin._schedule_hud_road_scene=lambda *a,**kw:False
        plugin._schedule_live_map_scene=lambda *a,**kw:False
        plugin.on_tick(.05)
        packet=sdk.get('nav_steering_debug')
        self.assertTrue(packet['authority_valid'],packet)
        self.assertEqual(packet['reference_geometry'],geometry)
        self.assertEqual(packet['sdk_frame_us'],120000)
        self.assertEqual(packet['reference_ahead_m'],2.1)
        # Explicit null must not select Route's mathematical a=0 default.
        observation=sdk.get('vehicle_envelope_snapshot')
        observation['tractor_reference_geometry']=None
        sdk.set('vehicle_envelope_snapshot',observation)
        plugin.on_tick(.05)
        self.assertFalse(sdk.get('nav_steering_debug')['authority_valid'])
        self.assertFalse(sdk.get('nav_active'))

    def test_origin_reference_survives_proven_road_prefab_boundary(self):
        first=((1,1,0,'',-1,()),10)
        second=((2,1,0,'test_prefab',0,(0,1)),10)
        data=path([(0,40),(1/35,80),(0,50)])
        points=[(x,10.,z) for x,z in data['points']]
        authorities=[first if i<160 else second for i in range(len(points))]
        # The shared directed A->B segment belongs to both proven identities.
        p1,p2=points[159:161]
        pos=((p1[0]+p2[0])/2,(p1[2]+p2[2])/2)
        heading=math.atan2(p1[0]-p2[0],p1[2]-p2[2])+math.asin(2.1/35)
        outputs=[]
        for revision,identity in ((8,first),(9,second)):
            route=Route(points,point_authorities=authorities)
            outputs.append(route.steering(pos,heading,7.4,cross_track_error_m=0.,
                reference_geometry=sdk_reference_geometry(self.sdk_geometry()),
                control_authority=dict(lane_identity=identity[0],elevation_layer=10,
                                       lane_width_m=4.5,revision=revision)))
            self.assertTrue(route.last_steering_debug['authority_valid'])
            self.assertEqual(route.last_steering_debug['authority_revision'],revision)
        self.assertAlmostEqual(outputs[0],outputs[1],places=10)

    def test_benchmark_loads_historical_solver_not_current_solver(self):
        from unittest import mock
        from tools.run_steering_bench import factories_for_ref
        # Test the loader without requiring a full git history in a shallow
        # checkout. Real 089cd0c comparisons are made by the benchmark CLI.
        def historical_source(command,**_kwargs):
            if command[1]=='ls-tree':
                return 'core/lateral_controller.py' if command[3]=='separate' else ''
            ref,path=command[2].split(':',1)
            if path=='core/steering_dynamics.py':
                return 'class SteeringDynamics: pass'
            if path=='core/lateral_controller.py':
                return 'def solve(*args): return 0.37'
            if ref=='separate':
                return ('from core.lateral_controller import solve as solve_lateral\n'
                        'class Route:\n    def steering(self): return solve_lateral()')
            return 'class Route: pass'
        with mock.patch('tools.run_steering_bench.subprocess.check_output',side_effect=historical_source):
            old,_,hashes=factories_for_ref('separate')
            self.assertIn('core/lateral_controller.py',hashes)
            solver=old.steering.__globals__['solve_lateral']
            self.assertIsNot(solver,solve)
            self.assertEqual(old().steering(),.37)
            # e8d5c57-style trees predate the separate solver module.
            oldest,_,hashes=factories_for_ref('before_separate')
            self.assertNotIn('core/lateral_controller.py',hashes)
            self.assertTrue(callable(oldest))


if __name__ == '__main__':
    unittest.main()
