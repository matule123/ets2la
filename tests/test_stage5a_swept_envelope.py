"""Independent analytical geometry and continuous occupancy regressions for 5A."""
from dataclasses import replace
import math
import unittest

from core.navigation.lane_model import LaneId
from core.swept_envelope import (
    Body, Vehicle, Pose, Frame, Identity, Surface, evaluate, footprint, inside,
    articulated_poses, propagate, offset, clearance, EnvelopeError,
)


def identity():
    return Identity('intent',8,'build','session','promods-1.59','fingerprint','ground',
        (LaneId(10,1,0),LaneId(20,1,0,'prefab',1,(1,22,16,12)),LaneId(30,1,0)),
        ((1,2),(2,3),(3,4)))


def vehicle(trailers=0):
    bodies = (Body('tractor',2.5,5.0,1.2,0.,0.,'measured fixture',True,'fixed_axle'),)
    bodies += tuple(Body(f'trailer{i}',2.5,9.,3.,8.,-1.,'measured fixture',True,'fixed_axle')
                    for i in range(trailers))
    return Vehicle(bodies,3.8,.70,2.,.15,.02)


def rect(x0,z0,x1,z1):
    return ((x0,z0),(x1,z0),(x1,z1),(x0,z1))


def surface(exterior=None, holes=(), y=0.):
    return Surface(identity(),exterior or rect(-100.,-100.,100.,100.),holes,y,
                   'surveyed fixture, not game map',True,True)


def frame(v, t=0., p=None, headings=None):
    p = p or Pose(0.,0.,0.,0.)
    return Frame(t,identity(),articulated_poses(v,p,headings or (0.,)*(len(v.bodies)-1)))


def circle_case(radius=18., right=False, trailer=False):
    """Analytic steady on-axle tractor/semitrailer, independent of propagate()."""
    v=vehicle(int(trailer))
    sign=-1 if right else 1
    frames=[]
    for i in range(81):
        h=sign*(.3+i*.002)
        p=Pose(sign*radius*(math.cos(h)-1),0.,-sign*radius*math.sin(h),h)
        heads=(h-sign*math.asin(8./radius),) if trailer else ()
        frames.append(frame(v,i*.02,p,heads))
    def ring(r):
        return tuple((-sign*radius+r*math.cos(i*math.tau/256),
                       r*math.sin(i*math.tau/256)) for i in range(256))
    area=surface(ring(radius+2.25),(ring(radius-2.25),))
    return v,tuple(frames),area


class Stage5AEnvelopeTests(unittest.TestCase):
    def test_static_body_includes_front_and_rear_overhang(self):
        v=vehicle(); f=frame(v)
        p=footprint(v.bodies[0],f.poses[0])
        self.assertEqual(set(p),{(-1.25,-5.),(1.25,-5.),(-1.25,1.2),(1.25,1.2)})
        result=evaluate(v,(f,),surface(rect(-3.,-10.,3.,10.)),identity())
        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.minimum_clearance_m,1.58)

    def test_heading_quarter_turn_and_translation(self):
        p=footprint(vehicle().bodies[0],Pose(20.,7.,30.,math.pi/2))
        self.assertAlmostEqual(min(x for x,z in p),15.)
        self.assertAlmostEqual(max(x for x,z in p),21.2)
        self.assertAlmostEqual(min(z for x,z in p),28.75)

    def test_clearance_checks_edges_across_concave_notch(self):
        # All corners inside; the middle of the top edge crosses the notch.
        area=surface(((-5.,-5.),(5.,-5.),(5.,5.),(1.,5.),(1.,0.),
                      (-1.,0.),(-1.,5.),(-5.,5.)))
        poly=rect(-3.,-3.,3.,3.)
        self.assertTrue(all(inside(p,area.exterior) for p in poly))
        self.assertEqual(clearance(poly,area),0.)

    def test_island_fully_enclosed_by_body_is_not_missed(self):
        area=surface(holes=(rect(-.1,-2.,.1,-1.),))
        result=evaluate(vehicle(),(frame(vehicle()),),area,identity())
        self.assertEqual(result.failure_reason,'SWEPT_ENVELOPE_RESERVE_VIOLATION')

    def test_touching_and_insufficient_reserve_rejected(self):
        v=vehicle()
        for half_width in (1.25,1.40,1.42):
            with self.subTest(half_width=half_width):
                self.assertFalse(evaluate(v,(frame(v),),surface(rect(-half_width,-20.,half_width,20.)),identity()).accepted)

    def test_between_frames_thin_obstacle_detected(self):
        # Individual tiny-body footprints clear an island; sweep crosses it.
        v=replace(vehicle(),bodies=(Body('tractor',.1,2.,0.,0.,0.,'fixture',True,'fixed_axle'),),
                  wheelbase_m=2.,safety_margin_m=.001,uncertainty_m=.001,max_speed_mps=5.)
        area=surface(holes=(rect(-.25,-.01,-.2,.01),))
        # Use perpendicular motion for a bounded-pose occupancy test, not a path planner.
        a=frame(v,p=Pose(-.4,0.,0.,0.))
        b=frame(v,.16,Pose(.4,0.,0.,0.))
        self.assertTrue(evaluate(v,(a,),area,identity()).accepted)
        self.assertTrue(evaluate(v,(b,),area,identity()).accepted)
        self.assertFalse(evaluate(v,(a,b),area,identity()).accepted)

    def test_sweep_encloses_all_intermediate_rotating_corners(self):
        v=vehicle(1); f=frame(v); frames=[f]
        for _ in range(10):
            f=propagate(v,f,2.,.3,.02); frames.append(f)
        result=evaluate(v,(frames[0],frames[-1]),surface(),identity())
        self.assertTrue(result.accepted)
        for f in frames:
            for i,body in enumerate(v.bodies):
                for p in footprint(body,f.poses[i]):
                    self.assertTrue(inside(p,result.steps[-1].swept_polygons[i]))

    def test_analytic_radii_mirror_and_full_body_clearance(self):
        for radius in (18.,22.,25.,35.,83.):
            for trailer in (False,True):
                with self.subTest(radius=radius,trailer=trailer):
                    a=evaluate(*circle_case(radius,False,trailer),identity())
                    b=evaluate(*circle_case(radius,True,trailer),identity())
                    self.assertEqual(a.accepted,b.accepted)
                    self.assertAlmostEqual(a.minimum_clearance_m,b.minimum_clearance_m,places=8)
                    if radius==18.:
                        self.assertEqual(a.accepted,not trailer)

    def test_multiple_articles_preserve_every_hitch(self):
        v=vehicle(3); f=frame(v)
        for _ in range(100):
            f=propagate(v,f,2.,.2,.02)
        for i in range(1,4):
            a=offset(f.poses[i-1],v.bodies[i-1].hitch_rear_m)
            b=offset(f.poses[i],v.bodies[i].hitch_front_m)
            self.assertAlmostEqual(a.x,b.x,places=12)
            self.assertAlmostEqual(a.z,b.z,places=12)
        self.assertEqual(len(evaluate(v,(f,),surface(),identity()).steps[0].footprints),4)

    def test_off_axle_hitch_yaw_from_velocity_not_tractor_heading(self):
        v=vehicle(1)
        v=replace(v,bodies=(replace(v.bodies[0],hitch_rear_m=-1.),v.bodies[1]))
        f=frame(v)
        out=propagate(v,f,2.,.2,.00001)
        expected=(-1.)*(-2.*math.tan(.2)/3.8)/8.
        self.assertAlmostEqual(out.poses[1].heading/.00001,expected,places=6)
        self.assertGreater(out.poses[1].heading,0.)  # initial hitch swings outward

    def test_tractor_exact_circle_and_left_right_signs(self):
        v=vehicle()
        for delta in (-.2,.2):
            f=frame(v)
            for _ in range(100):
                f=propagate(v,f,2.,delta,.02)
            w=-2.*math.tan(delta)/3.8
            h=w*2.
            self.assertAlmostEqual(f.poses[0].heading,h,places=10)
            self.assertAlmostEqual(f.poses[0].x,2./w*(math.cos(h)-1),places=9)
            self.assertAlmostEqual(f.poses[0].z,-2./w*math.sin(h),places=9)

    def test_steady_trailer_circle_matches_analytic_radius(self):
        v=vehicle(1); radius=18.; h=.4
        f=frame(v,p=Pose(0.,0.,0.,h),headings=(h-math.asin(8./radius),))
        out=propagate(v,f,2.,-math.atan(3.8/radius),.02)
        self.assertAlmostEqual(out.poses[0].heading-out.poses[1].heading,
                               math.asin(8./radius),places=10)

    def test_no_motion_and_restart_are_deterministic(self):
        v=vehicle(1); f=frame(v)
        stopped=propagate(v,f,0.,.3,.02)
        self.assertEqual(f.poses,stopped.poses)
        self.assertEqual(propagate(v,stopped,2.,.3,.02).poses,
                         propagate(v,f,2.,.3,.02).poses)

    def test_no_state_or_input_mutation(self):
        v=vehicle(1); f=frame(v); area=surface()
        first=evaluate(v,(f,),area,identity())
        evaluate(vehicle(),(frame(vehicle()),),surface(y=10.),identity())
        self.assertEqual(first,evaluate(v,(f,),area,identity()))

    def test_missing_dimensions_and_axle_model_fail_closed(self):
        for body in (replace(vehicle().bodies[0],confirmed=False),
                     replace(vehicle().bodies[0],width_m=float('nan')),
                     replace(vehicle().bodies[0],source=''),
                     replace(vehicle().bodies[0],axle_model='unproven_tandem')):
            v=replace(vehicle(),bodies=(body,))
            self.assertFalse(evaluate(v,(frame(vehicle()),),surface(),identity()).accepted)

    def test_missing_and_display_only_boundaries_rejected(self):
        v=vehicle()
        for area in (None,replace(surface(),confirmed=False),replace(surface(),horizontal=False)):
            self.assertFalse(evaluate(v,(frame(v),),area,identity()).accepted)

    def test_identity_all_tokens_and_lane_sequence_are_bound(self):
        v=vehicle(); f=frame(v)
        for key,value in dict(intent='new',revision=9,build='new',session='new',
            map_key='new',dataset='new',layer='bridge',lanes=(LaneId(99,1,0),),
            gps_pairs=((2,3),(3,4),(4,5))).items():
            with self.subTest(key=key):
                bad=replace(f,identity=replace(identity(),**{key:value}))
                self.assertEqual(evaluate(v,(f,bad),surface(),identity()).failure_reason,
                                 'STALE_FRAME_IDENTITY')

    def test_rolling_prefix_and_repeated_uid_preserve_order(self):
        v=vehicle(); ident=replace(identity(),gps_pairs=((1,2),(2,1),(1,2)))
        f=replace(frame(v),identity=ident); area=replace(surface(),identity=ident)
        self.assertTrue(evaluate(v,(f,),area,ident).accepted)
        shortened=replace(ident,lanes=ident.lanes[1:],gps_pairs=ident.gps_pairs[1:])
        self.assertEqual(evaluate(v,(f,),area,shortened).failure_reason,'STALE_SURFACE_IDENTITY')

    def test_bridge_and_tilt_not_flattened(self):
        v=vehicle()
        for p in (Pose(0.,8.,0.,0.),Pose(0.,0.,0.,0.,.1,0.),Pose(0.,0.,0.,0.,0.,.1)):
            f=Frame(0.,identity(),(p,))
            self.assertFalse(evaluate(v,(f,),surface(),identity()).accepted)
        f=frame(v,p=Pose(0.,8.,0.,0.))
        self.assertTrue(evaluate(v,(f,),surface(y=8.),identity()).accepted)

    def test_invalid_time_pose_jump_hitch_and_nan(self):
        v=vehicle(1); f=frame(v)
        for bad in (replace(f,time_s=-1.),replace(f,time_s=float('nan')),
                    replace(f,time_s=.3),replace(f,time_s=.1,poses=(Pose(100.,0.,0.,0.),f.poses[1])),
                    replace(f,time_s=.1,poses=(Pose(float('nan'),0.,0.,0.),f.poses[1]))):
            self.assertFalse(evaluate(v,(f,bad),surface(),identity()).accepted)
        self.assertFalse(evaluate(v,(f,f),surface(),identity()).accepted)

    def test_malformed_and_self_intersecting_boundaries(self):
        v=vehicle()
        for area in (replace(surface(),exterior=((0.,0.),(2.,2.),(0.,2.),(2.,0.))),
                     replace(surface(),holes=(rect(90.,90.,110.,110.),)),
                     replace(surface(),holes=(rect(-2.,-2.,2.,2.),rect(-1.,-1.,1.,1.)))):
            self.assertFalse(evaluate(v,(frame(v),),area,identity()).accepted)
        self.assertFalse(evaluate(None,(),None,identity()).accepted)

    def test_low_speed_domain_is_explicit(self):
        v=vehicle()
        for speed,angle,dt in ((-1.,0.,.02),(6.,0.,.02),(1.,1.,.02),(1.,0.,.1)):
            with self.assertRaises(EnvelopeError):
                propagate(v,frame(v),speed,angle,dt)

    def test_propagation_rejects_tilt_and_disconnected_input_instead_of_repair(self):
        v=vehicle(1); f=frame(v)
        for bad in (replace(f,poses=(replace(f.poses[0],pitch=.1),f.poses[1])),
                    replace(f,poses=(f.poses[0],replace(f.poses[1],x=2.))),
                    replace(f,poses=(f.poses[0],replace(f.poses[1],x=.001))),
                    replace(f,poses=(f.poses[0],replace(f.poses[1],y=1.)))):
            with self.assertRaises(EnvelopeError):
                propagate(v,bad,2.,.2,.02)

    def test_boundary_winding_and_world_offset_do_not_change_clearance(self):
        v=vehicle(); f=frame(v); area=surface(rect(-3.,-10.,3.,10.))
        expected=evaluate(v,(f,),area,identity()).minimum_clearance_m
        for dx,dz in ((0.,0.),(80000.,-60000.)):
            poly=tuple((x+dx,z+dz) for x,z in area.exterior[::-1])
            moved=frame(v,p=Pose(dx,0.,dz,0.))
            result=evaluate(v,(moved,),replace(area,exterior=poly),identity())
            self.assertTrue(result.accepted)
            self.assertAlmostEqual(result.minimum_clearance_m,expected,places=8)

    def test_integration_convergence_with_off_axle_multi_trailer(self):
        v=vehicle(2)
        v=replace(v,bodies=(replace(v.bodies[0],hitch_rear_m=-1.),)+v.bodies[1:])
        a=b=frame(v)
        for i in range(200):
            delta=.3*math.sin(i*.01)
            a=propagate(v,a,2.,delta,.02)
            for _ in range(4):
                b=propagate(v,b,2.,delta,.005)
        for p,q in zip(a.poses,b.poses):
            self.assertLess(math.hypot(p.x-q.x,p.z-q.z),1e-7)
            self.assertLess(abs(p.heading-q.heading),1e-8)

    def test_front_overhang_collision_even_when_axle_and_hitch_clear(self):
        v=vehicle()
        area=surface(holes=(rect(.9,-4.9,1.1,-4.7),))
        f=frame(v)
        self.assertTrue(inside((f.poses[0].x,f.poses[0].z),area.exterior))
        self.assertFalse(evaluate(v,(f,),area,identity()).accepted)

    def test_observation_is_not_permission_for_opposite_lane_or_traffic(self):
        # 5A never alters LaneId/intent or constructs a controller command.
        v=vehicle(); result=evaluate(v,(frame(v),),surface(),identity())
        self.assertEqual(result.identity,identity())
        self.assertFalse(hasattr(result,'steering'))
        self.assertFalse(hasattr(result,'target_speed'))

    def test_both_sides_and_size_changes_have_expected_straight_clearance(self):
        for width in (2.,2.5,2.8):
            for count in (0,1,2,4):
                v=vehicle(count)
                v=replace(v,bodies=tuple(replace(b,width_m=width) for b in v.bodies))
                area=surface(rect(-3.,-20.,3.,100.))
                result=evaluate(v,(frame(v),),area,identity())
                self.assertTrue(result.accepted)
                self.assertAlmostEqual(result.minimum_clearance_m,3-width/2-.17,places=8)

    def test_speed_domain_scales_sweep_but_not_static_body(self):
        v=vehicle(); a=frame(v); b=propagate(v,a,1.,.2,.02)
        slow=evaluate(v,(a,b),surface(),identity())
        fast=evaluate(replace(v,max_speed_mps=5.),(a,b),surface(),identity())
        self.assertEqual(slow.steps[0].footprints,fast.steps[0].footprints)
        self.assertGreater(fast.steps[-1].corner_motion_bounds_m[0],
                           slow.steps[-1].corner_motion_bounds_m[0])
        self.assertLess(fast.minimum_clearance_m,slow.minimum_clearance_m)

    def test_numeric_extremes_cannot_turn_overflow_into_clearance(self):
        v=vehicle()
        bad=Frame(0.,identity(),(Pose(1e308,0.,1e308,0.),))
        huge=surface(rect(-1e308,-1e308,1e308,1e308))
        self.assertFalse(evaluate(v,(bad,),huge,identity()).accepted)
        self.assertFalse(evaluate(v,(bad,),surface(),identity()).accepted)

    def test_s_curve_forward_heading_wrap_and_jitter_remain_continuous(self):
        v=vehicle(1)
        a=frame(v,p=Pose(0.,0.,0.,math.pi-.01),headings=(math.pi-.01,))
        samples=[a]
        for i in range(120):
            dt=.015 if i%2 else .02
            a=propagate(v,a,1.5,-.3 if i<60 else .3,dt)
            samples.append(a)
        self.assertGreater(max(f.poses[0].heading for f in samples),math.pi)
        result=evaluate(v,tuple(samples),surface(),identity())
        self.assertTrue(result.accepted)
        for a,b in zip(samples,samples[1:]):
            self.assertLess(abs(a.poses[0].heading-b.poses[0].heading),.01)


if __name__ == '__main__':
    unittest.main()
