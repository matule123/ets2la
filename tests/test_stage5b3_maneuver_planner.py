"""Offline behavior, independent physics and rejection tests for 5B3."""
from dataclasses import replace
import math
import unittest

import numpy as np

from core.navigation.maneuver_planner import (
    PlannerLimits, plan_maneuver, validate_plan_request,
    _curve, _geometry, _limits, _gate_indices,
)
from core.navigation.drivable_surface import evaluate_on_confirmed_surface
from tests.maneuver_cases import junction, centreline_frames


class Stage5B3Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.limits = PlannerLimits(max_candidates=9)
        cls.left = junction(half_width=3.5)
        cls.right = junction(half_width=3.5,right=True)
        cls.lp = plan_maneuver(*cls.left,limits=cls.limits)
        cls.rp = plan_maneuver(*cls.right,limits=cls.limits)

    def test_left_right_with_trailer_have_positive_complete_sweep(self):
        for plan in (self.lp,self.rp):
            with self.subTest(direction=plan.identity):
                self.assertTrue(plan.accepted,plan.failure_reason)
                self.assertGreater(plan.envelope.minimum_clearance_m,.15)
                self.assertFalse(plan.runtime_authorized)
                self.assertEqual(len(plan.samples),len(plan.envelope.steps))
                self.assertTrue(all(len(step.swept_polygons)==2
                                    for step in plan.envelope.steps))
        self.assertAlmostEqual(self.lp.envelope.minimum_clearance_m,
                               self.rp.envelope.minimum_clearance_m,places=7)

    def test_exit_exactly_centres_cab_and_clears_complete_trailer(self):
        for request,plan in ((self.left,self.lp),(self.right,self.rp)):
            v,_,path,_,_=request
            end=plan.samples[-1].frame
            self.assertAlmostEqual(end.poses[0].x,path.points[-1].x,places=9)
            self.assertAlmostEqual(end.poses[0].z,path.points[-1].z,places=9)
            self.assertLess(abs(end.poses[1].heading-end.poses[0].heading),math.radians(5))
            self.assertAlmostEqual(plan.samples[-1].tyre_rad,0.,places=10)

    def test_physical_signs_and_bicycle_equation_independent(self):
        for plan,sign in ((self.lp,-1),(self.rp,1)):
            turn=[s for s in plan.samples if abs(s.curvature_right_m_inv)>.001]
            self.assertTrue(turn)
            for sample in turn:
                self.assertGreater(sample.tyre_rad*sign,0)
                self.assertAlmostEqual(math.tan(sample.tyre_rad)/3.8,
                                       sample.curvature_right_m_inv,places=12)
            # SDK heading is positive left; tyre angle is positive right.
            for a,b in zip(plan.samples,plan.samples[1:]):
                ds=b.s_m-a.s_m
                expected=-(a.curvature_right_m_inv+b.curvature_right_m_inv)*ds/2
                actual=b.frame.poses[0].heading-a.frame.poses[0].heading
                self.assertLess(abs(expected-actual),2e-6)

    def test_off_axle_hitches_obey_independent_no_lateral_slip_constraint(self):
        from core.swept_envelope import articulated_poses, offset
        from core.navigation.maneuver_planner import _frames
        vehicle,start,_,_,_=junction(half_width=4.,trailers=2,trailer_length=6.)
        vehicle=replace(vehicle,bodies=(replace(vehicle.bodies[0],hitch_rear_m=.7),
            replace(vehicle.bodies[1],hitch_rear_m=-.7),vehicle.bodies[2]))
        start=replace(start,poses=articulated_poses(vehicle,start.poses[0],(0.,0.)))
        frames,_,_=_frames(vehicle,start,np.asarray(self.lp.controls_xz),
                           np.linspace(0,1,3001),2.)
        # Finite displacement of each fixed axle must be along its own
        # heading; independently test the constraint, not the RK4 formula.
        for a,b in zip(frames,frames[1:]):
            for i,(p,q) in enumerate(zip(a.poses,b.poses)):
                h=(p.heading+q.heading)/2
                lateral=((q.x-p.x)*math.cos(h)-(q.z-p.z)*math.sin(h))
                self.assertLess(abs(lateral),2e-6)
                if i:
                    parent=offset(b.poses[i-1],vehicle.bodies[i-1].hitch_rear_m)
                    child=offset(q,vehicle.bodies[i].hitch_front_m)
                    self.assertLess(math.hypot(parent.x-child.x,parent.z-child.z),1e-10)

    def test_analytic_interval_bounds_enclose_independent_dense_derivatives(self):
        ctrl=np.asarray(self.lp.controls_xz)
        bound=_limits(ctrl,3.8)
        for piece in ctrl:
            u=np.linspace(0,1,2001)
            _,d,dd,ddd,dddd=_curve(piece,u)
            q2=np.sum(d*d,axis=1)
            A=d[:,0]*dd[:,1]-d[:,1]*dd[:,0]
            Ap=d[:,0]*ddd[:,1]-d[:,1]*ddd[:,0]
            App=dd[:,0]*ddd[:,1]-dd[:,1]*ddd[:,0]+d[:,0]*dddd[:,1]-d[:,1]*dddd[:,0]
            B=np.sum(d*dd,axis=1);Bp=np.sum(dd*dd+d*ddd,axis=1)
            k=A/q2**1.5
            ks=Ap/q2**2-3*A*B/q2**3
            kss=(App/q2**2-7*Ap*B/q2**3-3*A*Bp/q2**3+18*A*B*B/q2**4)/np.sqrt(q2)
            delta_s=3.8*ks/(1+(3.8*k)**2)
            delta_ss=3.8*kss/(1+(3.8*k)**2)-2*3.8**3*k*ks**2/(1+(3.8*k)**2)**2
            for value,ceiling in zip((k,delta_s,delta_ss),bound):
                self.assertLessEqual(float(np.max(abs(value))),ceiling+1e-10)

    def test_g3_joins_prevent_steering_rate_step(self):
        ctrl=np.asarray(self.lp.controls_xz)
        for a,b in zip(ctrl,ctrl[1:]):
            p1,h1,k1,_=_geometry(a,np.array([1.]))
            p2,h2,k2,_=_geometry(b,np.array([0.]))
            np.testing.assert_allclose(p1,p2,atol=1e-10)
            self.assertAlmostEqual(float(h1[0]),float(h2[0]),places=10)
            self.assertLess(abs(float(k1[0]-k2[0])),1e-10)
            # Curvature change per metre tends to zero on both sides.
            for c,u in ((a,np.array([1-1e-6,1.])),(b,np.array([0.,1e-6]))):
                _,_,k,q=_geometry(c,u)
                self.assertLess(abs(float((k[1]-k[0])/(1e-6*q.mean()))),1e-5)

    def test_dense_rate_acceleration_and_time_bounds(self):
        p=self.lp
        times=np.array([s.frame.time_s for s in p.samples])
        angles=np.array([s.tyre_rad for s in p.samples])
        dt=np.diff(times); rates=np.diff(angles)/dt
        acceleration=np.diff(rates)/((dt[1:]+dt[:-1])/2)
        self.assertLessEqual(max(dt),.020001)
        self.assertGreater(min(dt),0)
        self.assertLessEqual(max(abs(rates)),self.limits.tyre_rate_rad_s+1e-6)
        self.assertLessEqual(max(abs(acceleration)),self.limits.tyre_accel_rad_s2+1e-5)

    def test_narrow_r18_is_not_claimed_safe_for_trailer(self):
        request=junction(half_width=2.25)
        result=plan_maneuver(*request,limits=self.limits)
        self.assertFalse(result.accepted)
        self.assertEqual(result.failure_reason,'NO_VALIDATED_CANDIDATE_IN_SEARCH_BUDGET')
        self.assertTrue(any('RESERVE_VIOLATION' in row[1] for row in result.diagnostics))

    def test_no_trailer_is_supported(self):
        result=plan_maneuver(*junction(trailers=0,half_width=3.5),limits=self.limits)
        self.assertTrue(result.accepted,result.failure_reason)
        self.assertTrue(all(len(s.frame.poses)==1 for s in result.samples))

    def test_missing_boundary_and_body_never_fabricated(self):
        v,start,path,surface,ident=self.left
        self.assertFalse(plan_maneuver(v,start,path,None,ident).accepted)
        v=replace(v,bodies=(replace(v.bodies[0],confirmed=False),)+v.bodies[1:])
        self.assertEqual(plan_maneuver(v,start,path,surface,ident).failure_reason,
                         'UNPROVEN_BODY_DIMENSIONS')

    def test_all_context_changes_reject_old_callback(self):
        v,start,path,surface,ident=self.left
        for key,value in {'intent':'new','revision':9,'build':'new','session':'new',
                          'map_key':'new','dataset':'new','layer':'bridge'}.items():
            with self.subTest(key=key):
                reason=validate_plan_request(self.lp,v,start,path,surface,
                    replace(ident,**{key:value}),self.limits)
                self.assertEqual(reason,'STALE_MANEUVER_IDENTITY')

    def test_replacement_surface_vehicle_and_start_invalidate_request(self):
        v,start,path,surface,ident=self.left
        for change in ('vehicle','surface','start'):
            vv,ss,aa=v,start,surface
            if change=='vehicle': vv=replace(v,uncertainty_m=.03)
            if change=='surface': aa=replace(surface,token='new')
            if change=='start': ss=replace(start,time_s=.1)
            with self.subTest(change=change):
                self.assertEqual(validate_plan_request(self.lp,vv,ss,path,aa,ident,
                    self.limits),'STALE_MANEUVER_REQUEST')
        self.assertEqual(validate_plan_request(self.lp,*self.left,self.limits),'')

    def test_plan_result_integrity_rejects_changed_samples(self):
        bad=replace(self.lp,samples=self.lp.samples[:-1])
        self.assertEqual(validate_plan_request(bad,*self.left,self.limits),
                         'MANEUVER_RESULT_INTEGRITY_MISMATCH')

    def test_changed_result_metadata_is_not_accepted(self):
        for fields in ({'surface_token':'new'}, {'source_path_fingerprint':'new'},
                       {'requires_entry_speed_reduction':True}):
            with self.subTest(fields=fields):
                self.assertEqual(validate_plan_request(replace(self.lp,**fields),
                    *self.left,self.limits),'MANEUVER_RESULT_METADATA_MISMATCH')

    def test_directional_gates_preserve_road_prefab_road(self):
        sequence=list(dict.fromkeys(s.source_segment_index for s in self.lp.samples))
        self.assertEqual(sequence,[0,1,2])
        for s in self.lp.samples:
            self.assertEqual(s.source_lane_id,self.left[4].lanes[s.source_segment_index])
            self.assertEqual(s.gps_pair,self.left[4].gps_pairs[s.source_segment_index])

    def test_stop_and_restart_requests_do_not_create_wait_timer(self):
        # Planner timestamps originate at the request; it introduces no wait.
        p=self.lp
        self.assertEqual(p.samples[0].frame.time_s,self.left[1].time_s)
        self.assertGreater(p.speed_mps,0)
        self.assertFalse(p.requires_entry_speed_reduction)

    def test_invalid_limits_fail_closed(self):
        for limits in (replace(self.limits,tyre_rate_rad_s=float('nan')),
                       replace(self.limits,max_candidates=True),
                       replace(self.limits,sample_dt_s=.1)):
            self.assertFalse(plan_maneuver(*self.left,limits=limits).accepted)

    def test_bridge_y_and_tilt_are_not_flattened(self):
        v,start,path,surface,ident=self.left
        for pose in (replace(start.poses[0],y=5),replace(start.poses[0],roll=.01)):
            bad=replace(start,poses=(pose,)+start.poses[1:])
            self.assertFalse(plan_maneuver(v,bad,path,surface,ident).accepted)

    def test_island_in_body_path_is_rejected(self):
        v,start,path,surface,ident=self.left
        # Obstacle across the entire entry width. Start body already intersects.
        hole=((-3.,-3.),(3.,-3.),(3.,-2.),(-3.,-2.))
        modified=replace(surface,surface=replace(surface.surface,holes=(hole,)))
        p=plan_maneuver(v,start,path,modified,ident,limits=self.limits)
        self.assertFalse(p.accepted)

    def test_path_not_mutated_by_search(self):
        from core.navigation.drivable_surface import lane_path_fingerprint
        self.assertEqual(self.lp.source_path_fingerprint,lane_path_fingerprint(self.left[2]))

    def test_short_entry_is_explicitly_rejected(self):
        result=plan_maneuver(*junction(half_width=4.,approach=5.),limits=self.limits)
        self.assertEqual(result.failure_reason,'INSUFFICIENT_APPROACH_OR_EXIT')

    def test_safe_speed_is_derived_from_physical_rate_bound(self):
        limits=replace(self.limits,max_candidates=1,tyre_rate_rad_s=.02)
        p=plan_maneuver(*self.left,limits=limits,entry_speed_mps=3.)
        self.assertTrue(p.accepted,p.failure_reason)
        _,rate,_=_limits(np.asarray(p.controls_xz),3.8)
        self.assertLess(p.speed_mps,2.)
        self.assertAlmostEqual(p.speed_mps*rate,.02,places=10)
        self.assertTrue(p.requires_entry_speed_reduction)
        self.assertFalse(p.runtime_authorized)

    def test_search_finds_safe_stop_where_same_surface_centreline_fails(self):
        request=junction(half_width=3.25)
        vehicle,_,path,surface,identity=request
        baseline=evaluate_on_confirmed_surface(vehicle,centreline_frames(request,18.),
                                               surface,path,identity)
        self.assertFalse(baseline.accepted)
        p=plan_maneuver(*request)
        self.assertTrue(p.accepted,p.failure_reason)
        self.assertGreater(p.envelope.minimum_clearance_m,.10)
        self.assertEqual(p.surface_token,surface.token)

    def test_r35_does_not_oversample_straights_by_turn_parameter_speed(self):
        from core.navigation.maneuver_planner import _sample_parameters
        p=plan_maneuver(*junction(radius=35.,trailers=0))
        self.assertTrue(p.accepted,p.failure_reason)
        c=np.asarray(p.controls_xz)
        old_count=math.ceil(float(np.max(np.linalg.norm(7*np.diff(c,axis=1),axis=2)))
                            *3/(p.speed_mps*.02))
        self.assertGreater(old_count,9999)
        self.assertLess(len(p.samples),10001)
        us=_sample_parameters(c,p.speed_mps,.02)
        self.assertIn(1/3,us)
        self.assertIn(2/3,us)
        self.assertLessEqual(max(b.frame.time_s-a.frame.time_s
            for a,b in zip(p.samples,p.samples[1:])),.020001)

    def test_nonzero_entry_tyre_is_not_silently_zeroed(self):
        p=plan_maneuver(*self.left,limits=self.limits,entry_tyre_rad=.05)
        self.assertFalse(p.accepted)
        self.assertEqual(p.failure_reason,'ENTRY_NOT_STRAIGHT')

    def test_looping_control_net_has_no_acceptance_by_endpoint_only(self):
        from core.swept_envelope import EnvelopeError
        ctrl=np.asarray(self.lp.controls_xz).copy()
        ctrl[1,3]=ctrl[1,0]+np.array((20.,20.))
        with self.assertRaises(EnvelopeError): _limits(ctrl,3.8)

    def test_repeated_gps_uids_keep_occurrence_order(self):
        v,start,path,surface,ident=self.left
        pairs=((1,2),(2,1),(1,2))
        ident=replace(ident,gps_pairs=pairs)
        segments=tuple(replace(seg,start_uid=pairs[i][0],end_uid=pairs[i][1])
                       for i,seg in enumerate(path.segments))
        repeated=replace(path,segments=segments,source_gps_uids=(1,2,1,2))
        from core.navigation.drivable_surface import validate_path_identity
        validate_path_identity(repeated,ident,ident)
        # Repeated IDs do not collapse the three occurrence-indexed gates.
        self.assertEqual(_gate_indices(repeated,tuple(s.frame for s in self.lp.samples)),
                         tuple(s.source_segment_index for s in self.lp.samples))

    def test_rolling_prefix_cannot_reuse_old_geometry(self):
        v,start,path,surface,ident=self.left
        shortened=replace(path,source_gps_uids=(2,3,4))
        reason=validate_plan_request(self.lp,v,start,shortened,surface,ident,self.limits)
        self.assertTrue(reason)

    def test_invalid_current_plan_is_not_a_driving_packet(self):
        self.assertNotEqual(validate_plan_request(replace(self.lp,runtime_authorized=True),
            *self.left,self.limits),'')


if __name__=='__main__':
    unittest.main()
