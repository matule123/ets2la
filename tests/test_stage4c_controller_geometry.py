"""Phase 4C: the regulator must receive an orthogonal, current Frenet frame."""
import copy
import math
import unittest

from core.navigation.route import Route


REFERENCE_GEOMETRY = dict(valid=True, reference_ahead_m=2.1, wheelbase_m=3.8)


def circle(radius, direction=1, spacing=2.):
    return [(-direction*radius*(1-math.cos(s/radius)),
             radius*math.sin(s/radius))
            for s in (i*spacing for i in range(int(100/spacing)+1))]


def centred_circle_commands(radius, direction=1, route_factory=Route, gps=False):
    points = circle(radius, direction)
    authority = ((1, 1, 0, '', -1, ()), 50)
    route = (route_factory(points, point_authorities=[authority]*len(points))
             if gps else route_factory(points))
    rows = []
    for index in range(201):
        s = 30.+index*.01
        position = (-direction*radius*(1-math.cos(s/radius)),
                    radius*math.sin(s/radius))
        heading = math.pi-direction*s/radius+math.asin(direction*2.1/radius)
        raw = route.steering(position, heading, 6., cross_track_error_m=0.,
                             reference_geometry=REFERENCE_GEOMETRY,
                             control_authority=(dict(lane_identity=authority[0],
                                 elevation_layer=50, revision=8) if gps else None),
                             curvature_preview_s=.134)
        rows.append((raw, route.last_steering_debug.copy()))
    return rows


class Stage4CProjectionTests(unittest.TestCase):
    def test_real_dense_endpoint_bias_without_and_with_trailer(self):
        # Exact existing edges reconstructed independently from recorded
        # projection/fraction pairs; max fit residual <1e-5 m. Same road UID
        # 5962819262254292140, direction 1, lane 0, layer 70, revision 8.
        # No inferred link or map point is added by this reproduction.
        points = [(37152.53672876489, 58734.46106865852),
                  (37154.43894752514, 58734.42191782091),
                  (37156.334176791024, 58734.600562968415)]
        samples = (
            # Sept 10 15:18:45.764, SDK 205408450, no trailer.
            ((37154.79797363281, 58736.232666015625),
             -1.5230051104074764, .2770340339414877),
            # Sept 10 15:25:21.562, SDK 568877244, with trailer.
            ((37154.81399536133, 58736.251052856445),
             -1.5228882642146484, .28631972681985424),
        )
        authority = ((5962819262254292140, 1, 0, '', -1, ()), 70)
        route = Route([(x, 70., z) for x, z in points],
                      point_authorities=[authority]*3)
        for position, heading, fraction in samples:
            with self.subTest(position=position):
                projection = route._authority_projection(authority, position, heading)
                self.assertEqual(projection[2], 1,
                                 'body heading must not pin projection to previous endpoint')
                self.assertAlmostEqual(projection[3], fraction, places=8)

    def test_centred_circle_cannot_manufacture_periodic_heading_error(self):
        for direction in (-1, 1):
            for radius in (18., 35., 83., 250.):
                with self.subTest(direction=direction, radius=radius):
                    rows = centred_circle_commands(radius, direction)
                    raw = [r[0] for r in rows]
                    # 0.005 input corresponds to 0.20 degrees at the tyre;
                    # existing 4B produces a 0.048 input sawtooth on R18.
                    self.assertLess(max(raw)-min(raw), .005)
                    progress = [r[1]['tracking_progress_m'] for r in rows]
                    self.assertTrue(all(b > a for a, b in zip(progress, progress[1:])))
                    self.assertTrue(all(value*direction > 0 for value in raw))

    def test_branch_choice_cannot_cross_unavailable_or_opposite_edges(self):
        route = Route([(0., 0.), (0., 2.), (0., 4.), (0., 2.), (0., 0.)])
        # Only these indices are owned by a confirmed live LaneId. An omitted
        # intervening edge is not permission to refine onto a different run.
        projection = route._best_projection([0, 3], (0., 2.4), math.pi)
        self.assertEqual(projection[2], 0)
        # The U-turn's reverse arm must not win from physical proximity.
        projection = route._best_projection(range(4), (.01, 1.), math.pi)
        self.assertLess(projection[2], 2)

    def test_authoritative_circle_has_no_chord_frequency_feedback_impulse(self):
        for direction in (-1, 1):
            for radius in (18., 35., 83., 250.):
                with self.subTest(direction=direction, radius=radius):
                    rows = centred_circle_commands(radius, direction, gps=True)
                    raw = [r[0] for r in rows]
                    self.assertLess(max(raw)-min(raw), .001)
                    for _, debug in rows:
                        self.assertAlmostEqual(debug['projection_longitudinal_residual_m'],
                                               0., places=8)
                        self.assertAlmostEqual(debug['raw'], debug['feed_forward']
                            + debug['heading_feedback']+debug['cte_feedback'], places=12)
                        self.assertEqual(debug['curvature_observation_weight'], 0.)

    def test_refinement_does_not_mutate_geometry_or_create_temporal_state(self):
        route = Route(circle(35.))
        before = copy.deepcopy((route.points, route.world_points))
        position = (-35*(1-math.cos(30.5/35)), 35*math.sin(30.5/35))
        heading = math.pi-30.5/35+math.asin(2.1/35)
        expected = route._best_projection(range(len(route.points)-1), position, heading)
        for other_heading in (heading-.1, heading+.1, heading):
            route._best_projection(range(len(route.points)-1), position, other_heading)
        self.assertEqual(expected, route._best_projection(
            range(len(route.points)-1), position, heading))
        self.assertEqual(before, (route.points, route.world_points))

    def test_sparse_roundabout_oracle_preserves_physical_occurrence(self):
        from tests.steering_bench import path, run
        from tools.run_stage4c_bench import gps_factory
        data = path([(0., 55.), (-1/25, 25*math.tau), (0., 100.)], step=2.)
        result, rows = run(data, speed=6., lock_rad=.70,
            controller_lock_rad=.70, controller_preview_s=.134,
            observation_ahead_m=2.1, controller_ahead_m=2.1,
            lag=.01, transport=.067, route_factory=gps_factory(Route),
            oracle_window_m=(5., 45.))
        self.assertLess(max(b['s']-a['s'] for a, b in zip(rows, rows[1:])), 1.)
        self.assertTrue(result['completed'])
        self.assertLess(result['max_cte'], .70)


class Stage4CClosedLoopTests(unittest.TestCase):
    def test_sparse_gps_curves_cab_trailer_and_speed_profiles(self):
        from tests.steering_bench import run
        from tools.run_stage4c_bench import cases, gps_factory
        for name, data, options in cases():
            for trailer in (False, True):
                with self.subTest(name=name, trailer=trailer):
                    result, rows = run(data, lock_rad=.70,
                        controller_lock_rad=.70, controller_preview_s=.134,
                        observation_ahead_m=2.1, controller_ahead_m=2.1,
                        lag=.01, transport=.067, noisy=True, jitter=True,
                        trailer=trailer, route_factory=gps_factory(Route),
                        oracle_window_m=(5., 45.), **options)
                    self.assertTrue(result['completed'])
                    self.assertFalse(result['lost_lane'])
                    self.assertTrue(all(r['authority_valid'] for r in rows))
                    self.assertLess(result['max_cte'], .70)
                    self.assertEqual(result['monotone_opposite_samples'], 0)
                    self.assertLessEqual(result['max_rate'], .60+1e-8)
                    self.assertLessEqual(result['max_accel'], 14.+1e-8)
                    # Confirm the entire settled tail, not one final sample.
                    exit_t = next(r['t'] for r in rows
                                  if r['s'] >= data['boundaries'][-2]+8.)
                    tail = [r for r in rows if r['t'] >= exit_t+4.]
                    self.assertTrue(tail)
                    self.assertLess(max(abs(r['cte']) for r in tail), .25)

    def test_both_signed_feedback_terms_are_locally_restoring_at_cab_reference(self):
        from core.lateral_controller import solve
        for radius, speed in ((18., 5.4), (35., 7.4), (83., 12.), (250., 25.)):
            for direction in (-1., 1.):
                with self.subTest(radius=radius, direction=direction):
                    k = direction/radius
                    a, wb = 2.1, 3.8
                    equilibrium = math.asin(a*k)
                    def motion(e, h):
                        raw, debug = solve(k, k, e, h, speed,
                            reference_ahead_m=a, wheelbase_m=wb)
                        self.assertTrue(debug['valid'])
                        vehicle_k = math.tan(.70*raw)/wb
                        return (speed*(math.sin(h)-a*vehicle_k*math.cos(h)),
                                speed*(k*(math.cos(h)+a*vehicle_k*math.sin(h))
                                       /(1+k*e)-vehicle_k))
                    self.assertAlmostEqual(motion(0., equilibrium)[0], 0., places=12)
                    self.assertAlmostEqual(motion(0., equilibrium)[1], 0., places=12)
                    eps = 1e-5
                    e_plus, e_minus = motion(eps, equilibrium), motion(-eps, equilibrium)
                    h_plus, h_minus = motion(0., equilibrium+eps), motion(0., equilibrium-eps)
                    j00, j10 = [(p-m)/(2*eps) for p, m in zip(e_plus, e_minus)]
                    j01, j11 = [(p-m)/(2*eps) for p, m in zip(h_plus, h_minus)]
                    # Hurwitz condition for the independent local 2x2
                    # kinematics. Actuator delays are tested closed-loop above.
                    self.assertLess(j00+j11, 0.)
                    self.assertGreater(j00*j11-j01*j10, 0.)


if __name__ == '__main__':
    unittest.main()
