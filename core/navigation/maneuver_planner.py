"""5B3 offline, bounded local path search for the 5A articulated model.

No runtime publisher/controller: an accepted result is a geometric candidate,
not permission to enter opposing traffic. The supplied 5B2 surface is the local
maneuver zone. Entry/exit are the exact first/last fixed-axle points of the
supplied local LanePath. GPS segment boundaries are mandatory directed gates.

Search uses septic Bezier boundary-value curves, never offsets a map lane.
Cab position/tangent/curvature are analytic; trailer headings use spatial RK4.
Bernstein interval bounds over the entire curve constrain tyre angle, rate,
acceleration and lateral acceleration. Final 5A sweeps include intersample
motion, body/surface uncertainty and an explicit numerical integration reserve.
The finite family is incomplete: search failure does NOT prove impossibility.
"""
from dataclasses import asdict, dataclass, replace
from itertools import product
import math

import numpy as np

from core.navigation.drivable_surface import (
    DrivableSurfaceSnapshot, digest, evaluate_on_confirmed_surface,
    lane_path_fingerprint, validate_path_identity,
)
from core.navigation.lane_model import LaneId, LanePath, wrap_angle
from core.swept_envelope import (
    EnvelopeError, EnvelopeResult, Frame, Identity, Pose, Vehicle,
    articulated_poses, clearance, expand, footprint, require,
)


@dataclass(frozen=True)
class PlannerLimits:
    speed_mps: float = 2.
    min_speed_mps: float = .25
    tyre_rate_rad_s: float = .12
    tyre_accel_rad_s2: float = .20
    lateral_accel_mps2: float = .8
    max_articulation_rad: float = math.radians(55)
    sample_dt_s: float = .02
    integration_reserve_m: float = .005
    endpoint_position_m: float = .001
    endpoint_heading_rad: float = math.radians(1)
    exit_article_heading_rad: float = math.radians(5)
    max_candidates: int = 81

    def validate(self):
        values = asdict(self)
        require(all(type(v) in (float, int) and math.isfinite(v)
                    for v in values.values()), 'INVALID_PLANNER_LIMITS')
        require(.1 <= self.min_speed_mps <= self.speed_mps <= 5
                and 0 < self.tyre_rate_rad_s <= 1
                and 0 < self.tyre_accel_rad_s2 <= 2
                and 0 < self.lateral_accel_mps2 <= 2
                and 0 < self.max_articulation_rad <= math.radians(65)
                and .002 <= self.sample_dt_s <= .02
                and .001 <= self.integration_reserve_m <= .02
                and 0 < self.endpoint_position_m <= .05
                and 0 < self.endpoint_heading_rad <= math.radians(2)
                and 0 < self.exit_article_heading_rad <= math.radians(5)
                and type(self.max_candidates) is int
                and 1 <= self.max_candidates <= 625,
                'UNSUPPORTED_PLANNER_LIMITS')


@dataclass(frozen=True)
class ManeuverSample:
    frame: Frame
    s_m: float
    tyre_rad: float
    curvature_right_m_inv: float
    source_segment_index: int
    source_lane_id: LaneId
    gps_pair: tuple[int, int]


@dataclass(frozen=True)
class ManeuverPlan:
    accepted: bool
    failure_reason: str
    identity: Identity
    token: str = ''
    request_fingerprint: str = ''
    surface_token: str = ''
    source_path_fingerprint: str = ''
    controls_xz: tuple = ()
    samples: tuple[ManeuverSample, ...] = ()
    envelope: EnvelopeResult | None = None
    speed_mps: float = 0.
    requires_entry_speed_reduction: bool = False
    diagnostics: tuple = ()
    # Even an accepted geometric plan has no runtime or traffic permission.
    runtime_authorized: bool = False


def _forward(h):
    return np.array((-math.sin(h), -math.cos(h)))


def _curve(ctrl, u):
    """Bezier value and four derivatives in the dimensionless parameter u."""
    u = np.asarray(u)
    ctrl = np.asarray(ctrl,dtype=float)
    if ctrl.ndim == 3:
        scaled = u*len(ctrl)
        index = np.minimum(scaled.astype(int),len(ctrl)-1)
        local = scaled-index
        out = [np.zeros(u.shape+(2,)) for _ in range(5)]
        for j,piece in enumerate(ctrl):
            mask = index == j
            if np.any(mask):
                values = _curve(piece,local[mask])
                for k in range(5):
                    out[k][mask] = values[k]*len(ctrl)**k
        return out
    out, points = [], np.asarray(ctrl, dtype=float)
    for _ in range(5):
        degree = len(points)-1
        weights = np.array([math.comb(degree, i)*u**i*(1-u)**(degree-i)
                            for i in range(degree+1)]).T
        out.append(weights @ points)
        points = degree*np.diff(points, axis=0)
    return out


def _geometry(ctrl, u):
    p, d, dd, _, _ = _curve(ctrl, u)
    q = np.linalg.norm(d, axis=-1)
    require(bool(np.all(q > 1e-6)), 'CUSP_OR_REVERSED_CURVE')
    cross = d[..., 0]*dd[..., 1]-d[..., 1]*dd[..., 0]
    return p, np.arctan2(-d[..., 0], -d[..., 1]), cross/q**3, q


# Interval arithmetic over Bernstein convex hulls, including unsampled points.
def _add(a, b): return a[0]+b[0], a[1]+b[1]
def _neg(a): return -a[1], -a[0]
def _mul(a, b):
    all_values = np.array([a[i]*b[j] for i in (0, 1) for j in (0, 1)])
    return all_values.min(axis=0), all_values.max(axis=0)
def _scale(a, k): return _mul(a, (k, k))
def _absmax(a): return np.maximum(abs(a[0]), abs(a[1]))
def _square(a):
    return np.where((a[0] <= 0) & (a[1] >= 0), 0.,
                    np.minimum(a[0]**2, a[1]**2)), _absmax(a)**2
def _dot(a, b): return _add(_mul(a[0], b[0]), _mul(a[1], b[1]))
def _cross(a, b): return _add(_mul(a[0], b[1]), _neg(_mul(a[1], b[0])))


def _limits(ctrl, wheelbase):
    """Conservative whole-curve bounds, not maxima of sampled derivatives.

    k=A/Q^(3/2), k_s=A'/Q^2-3AB/Q^3, where Q=r'·r',
    A=cross(r',r''), B=r'·r''. All derivatives are in world metres.
    """
    if np.asarray(ctrl).ndim == 3:
        # A positive derivative along one common axis proves injectivity of
        # the ENTIRE path: no loop, self-crossing or reverse branch between
        # sampled points. This sufficient test deliberately rejects some
        # feasible, more complicated maneuvers outside the single-turn family.
        axis=(ctrl[0,1]-ctrl[0,0])+(ctrl[-1,-1]-ctrl[-1,-2])
        require(float(np.linalg.norm(axis))>1e-6,'UNSUPPORTED_MANEUVER_TURN')
        axis=axis/np.linalg.norm(axis)
        for piece in ctrl:
            cells=np.array([piece])
            for _ in range(5):
                temp,left,right=cells,[cells[:,0]],[cells[:,-1]]
                for _ in range(len(piece)-1):
                    temp=(temp[:,:-1]+temp[:,1:])*.5
                    left.append(temp[:,0]);right.append(temp[:,-1])
                cells=np.concatenate((np.stack(left,axis=1),np.stack(right[::-1],axis=1)))
            require(bool(np.all(np.diff(cells,axis=1)@axis > 1e-10)),
                    'NONMONOTONE_MANEUVER_PROGRESS')
        bounds = [_limits(piece,wheelbase) for piece in ctrl]
        return tuple(max(row[j] for row in bounds) for j in range(3))
    degree = len(ctrl)-1
    cells = np.array([ctrl], dtype=float)
    for _ in range(7):  # 128 exact de Casteljau subcurves
        temp, left, right = cells, [cells[:, 0]], [cells[:, -1]]
        for _ in range(degree):
            temp = (temp[:, :-1]+temp[:, 1:])*.5
            left.append(temp[:, 0]); right.append(temp[:, -1])
        cells = np.concatenate((np.stack(left, axis=1),
                                np.stack(right[::-1], axis=1)))
    ds, cur = [], cells
    for n in range(degree, degree-4, -1):
        cur = n*np.diff(cur, axis=1)
        ds.append(tuple((cur[:, :, j].min(axis=1),
                         cur[:, :, j].max(axis=1)) for j in (0, 1)))
    a, b, c, d = ds
    Q = _add(_square(a[0]), _square(a[1]))
    require(bool(np.all(Q[0] > 1e-16)), 'UNPROVEN_REGULAR_CURVE')
    inv = lambda power: (Q[1]**-power, Q[0]**-power)
    A, B, Ap = _cross(a, b), _dot(a, b), _cross(a, c)
    App, Bp = _add(_cross(b, c), _cross(a, d)), _add(_dot(b, b), _dot(a, c))
    k = _mul(A, inv(1.5))
    ks = _add(_mul(Ap, inv(2)), _scale(_mul(_mul(A, B), inv(3)), -3))
    kss = _mul(_add(_add(_mul(App, inv(2)),
        _scale(_mul(_mul(Ap, B), inv(3)), -7)),
        _add(_scale(_mul(_mul(A, Bp), inv(3)), -3),
             _scale(_mul(_mul(A, _square(B)), inv(4)), 18))), inv(.5))
    denominator = _add((1., 1.), _scale(_square(k), wheelbase**2))
    reciprocal = (1/denominator[1], 1/denominator[0])
    delta_s = _scale(_mul(ks, reciprocal), wheelbase)
    delta_ss = _add(_scale(_mul(kss, reciprocal), wheelbase),
        _scale(_mul(_mul(_mul(k, _square(ks)), _square(reciprocal)),
                    (1., 1.)), -2*wheelbase**3))
    return tuple(float(np.max(_absmax(v))) for v in (k, delta_s, delta_ss))


def _controls(start, goal, tyre_start, tyre_end, wb, handles):
    p, g = np.array((start.x, start.z)), np.array((goal.x, goal.z))
    f, e = _forward(start.heading), _forward(goal.heading)
    require(abs(tyre_start)<1e-9 and abs(tyre_end)<1e-9,'ENTRY_NOT_STRAIGHT')
    a,b,c,d,e2,f2 = handles
    # First/last FOUR control points are collinear: k=0 AND dk/ds=0
    # at straight/curve joins. No step in required physical tyre velocity.
    return np.array((p,p+a*f,p+b*f,p+c*f,g-d*e,g-e2*e,g-f2*e,g))


def _family(path, start, goal, delta, wb, budget):
    distance = math.hypot(goal.x-start.x, goal.z-start.z)
    require(5 <= distance <= 200, 'UNSUPPORTED_MANEUVER_EXTENT')
    require(len(path.segments)>=3,'MANEUVER_REQUIRES_ENTRY_TURN_EXIT')
    # Keep a straight approach and settling tail. The control net only shapes
    # the turn and eight metres on either side; it cannot move map centrelines.
    incoming,outgoing = path.segments[1].centerline[0],path.segments[-1].centerline[0]
    f,e = _forward(start.heading),_forward(goal.heading)
    entry_length = float(np.array((incoming.x-start.x,incoming.z-start.z)) @ f)
    exit_length = float(np.array((goal.x-outgoing.x,goal.z-outgoing.z)) @ e)
    require(entry_length >= 10 and exit_length >= 10,'INSUFFICIENT_APPROACH_OR_EXIT')
    for segment,reference,direction in ((path.segments[0],start,f),(path.segments[-1],goal,e)):
        require(all(abs(float(np.array((point.x-reference.x,point.z-reference.z)) @
                    np.array((-direction[1],direction[0]))))<.01
                    for point in segment.centerline), 'NONSTRAIGHT_ENTRY_OR_EXIT')
    launch = replace(start,x=start.x+(entry_length-8)*f[0],
                     z=start.z+(entry_length-8)*f[1])
    settle = Pose(goal.x-(exit_length-8)*e[0],start.y,
                  goal.z-(exit_length-8)*e[1],goal.heading)
    points = tuple(point for point in path.points
                   if float(np.array((point.x-launch.x,point.z-launch.z))@f)>=-.001
                   and float(np.array((point.x-settle.x,point.z-settle.z))@e)<=.001)
    require(len(points)>=3,'MISSING_LOCAL_TURN_GEOMETRY')
    s = np.array([0.]+list(np.cumsum([math.hypot(b.x-a.x, b.z-a.z)
                            for a, b in zip(points, points[1:])])))
    u = s/s[-1]
    distance = math.hypot(settle.x-launch.x,settle.z-launch.z)
    base = np.array([.10,.25,.50,.50,.25,.10])*distance
    if abs(delta) < 1e-9:
        zero = _controls(launch, settle, 0., 0., wb, (0.,)*6)
        original = _curve(zero, u)[0]
        columns = []
        for j in range(6):
            unit = np.zeros(6); unit[j] = 1
            columns.append((_curve(_controls(launch, settle, 0., 0., wb, unit), u)[0]
                            - original).ravel())
        target = np.array([(p.x, p.z) for p in points])
        fit = np.linalg.lstsq(np.array(columns).T,
                             (target-original).ravel(), rcond=None)[0]
        if np.all(np.isfinite(fit)):
            base = np.clip(fit, .04*distance, 1.4*distance)
    steps = sorted(product(range(-2, 3), repeat=4),
                   key=lambda x: (sum(v*v for v in x), x))
    for step in steps[:budget]:
        perturb = np.array((0,step[0],step[1],step[2],step[3],0))
        handles = base + perturb*(.10*distance)
        if np.any(handles <= .01*distance):
            continue
        line1 = np.linspace((start.x,start.z),(launch.x,launch.z),8)
        line2 = np.linspace((settle.x,settle.z),(goal.x,goal.z),8)
        yield np.array((line1,_controls(launch,settle,delta,0.,wb,handles),line2))


def _trailer_derivative(vehicle, d, dd, headings):
    q2 = float(d @ d)
    heading = math.atan2(-d[0], -d[1])
    yaw = -(d[0]*dd[1]-d[1]*dd[0])/q2
    vx, vz = map(float, d)
    out = []
    for i, h in enumerate(headings, 1):
        hitch = vehicle.bodies[i-1].hitch_rear_m
        hx, hz = vx-hitch*yaw*math.cos(heading), vz+hitch*yaw*math.sin(heading)
        length = vehicle.bodies[i].hitch_front_m
        yaw = (-hx*math.cos(h)+hz*math.sin(h))/length
        vx, vz = hx+length*yaw*math.cos(h), hz-length*yaw*math.sin(h)
        out.append(yaw); heading = h
    return np.asarray(out)


def _frames(vehicle, start, ctrl, us, speed):
    positions, headings, ks, qs = _geometry(ctrl, us)
    # One-sided derivatives at piece joins. The arbitrary Bezier parameter
    # speed may jump there although physical ds/dt, heading, k and k_s do not.
    mids = (us[:-1]+us[1:])/2
    ders,seconds,eders,eseconds = (np.zeros((len(mids),2)) for _ in range(4))
    for j,piece in enumerate(ctrl):
        mask = np.minimum((mids*len(ctrl)).astype(int),len(ctrl)-1)==j
        # Select the interval's piece explicitly. nextafter(u)*3 can round
        # back onto a join and accidentally use the other piece's derivative.
        for values,points in (((ders,seconds),us[:-1]),((eders,eseconds),us[1:])):
            _,d,dd,_,_ = _curve(piece,np.clip(points[mask]*len(ctrl)-j,0.,1.))
            values[0][mask],values[1][mask] = d*len(ctrl),dd*len(ctrl)**2
    _,mders,mseconds,_,_ = _curve(ctrl,mids)
    dus = np.diff(us)
    nodes = us[:-1,None]+dus[:,None]*np.array((
        .5-math.sqrt(3/5)/2,.5,.5+math.sqrt(3/5)/2))
    quadrature = _geometry(ctrl,nodes.ravel())[3].reshape((-1,3))
    lengths = dus*(quadrature @ np.array((5/18,4/9,5/18)))
    headings = np.unwrap(headings)
    headings += round((start.poses[0].heading-headings[0])/math.tau)*math.tau
    trailer_h = np.array([p.heading for p in start.poses[1:]])
    frames, ss, travelled = [start], [0.], 0.
    for i in range(1, len(us)):
        du = us[i]-us[i-1]
        if len(trailer_h):
            k1 = _trailer_derivative(vehicle,ders[i-1],seconds[i-1],trailer_h)
            k2 = _trailer_derivative(vehicle,mders[i-1],mseconds[i-1],trailer_h+du*k1/2)
            k3 = _trailer_derivative(vehicle,mders[i-1],mseconds[i-1],trailer_h+du*k2/2)
            k4 = _trailer_derivative(vehicle,eders[i-1],eseconds[i-1],trailer_h+du*k3)
            trailer_h += du*(k1+2*k2+2*k3+k4)/6
        # Gauss-Legendre quadrature for travelled metres (not polyline chords).
        travelled += float(lengths[i-1])
        x, z = positions[i]
        p = Pose(float(x), start.poses[0].y, float(z), float(headings[i]))
        frames.append(Frame(start.time_s+travelled/speed, start.identity,
                            articulated_poses(vehicle, p, tuple(map(float,trailer_h)))))
        ss.append(travelled)
    return tuple(frames), tuple(ss), tuple(map(float, ks))


def _sample_parameters(ctrl, speed, dt):
    """Bound ds/dt per piece without oversampling every straight by turn speed.

    The Bernstein derivative hull bounds ds/du everywhere in that piece.
    Every join is an explicit sample and no integration step crosses it.
    This changes resource allocation, not the 20 ms or 10k-sample limits.
    """
    counts = [max(1,math.ceil(float(np.max(np.linalg.norm(
        7*np.diff(piece,axis=0),axis=1)))/(speed*dt))) for piece in ctrl]
    require(sum(counts) <= 9999, 'MANEUVER_SAMPLE_BUDGET_EXCEEDED')
    return np.concatenate([np.linspace(j/len(ctrl),(j+1)/len(ctrl),count+1)[:-1]
                           for j,count in enumerate(counts)]+[np.array([1.])])


def _gate_indices(path, frames):
    """Cross every ordered directed segment gate, never jump to another arm."""
    indices, next_gate = [], 1
    gates = [seg.centerline[0] for seg in path.segments[1:]]
    signs = []
    p = frames[0].poses[0]
    for gate in gates:
        f = _forward(gate.heading)
        signs.append(float(np.array((p.x-gate.x, p.z-gate.z)) @ f))
    require(all(v < -.01 for v in signs), 'AMBIGUOUS_OR_LOOPING_MANEUVER_GATES')
    for frame in frames:
        p = frame.poses[0]
        for j, gate in enumerate(gates):
            f = _forward(gate.heading)
            current = float(np.array((p.x-gate.x, p.z-gate.z)) @ f)
            if signs[j] < 0 <= current:
                require(j == next_gate-1, 'MANEUVER_GPS_GATE_ORDER')
                require(float(_forward(p.heading) @ f) > .5,
                        'MANEUVER_GATE_WRONG_DIRECTION')
                next_gate += 1
            require(not (signs[j] >= 0 > current), 'MANEUVER_GATE_BACKTRACK')
            signs[j] = current
        indices.append(next_gate-1)
    require(next_gate == len(path.segments), 'MANEUVER_MISSING_GPS_GATE')
    return tuple(indices)


def _screen(vehicle, frames, surface, limits):
    margin = vehicle.safety_margin_m+vehicle.uncertainty_m
    minimum = math.inf
    for frame in frames:
        for a, b in zip(frame.poses, frame.poses[1:]):
            require(abs(wrap_angle(b.heading-a.heading)) <= limits.max_articulation_rad,
                    'ARTICULATION_LIMIT')
        for body, pose in zip(vehicle.bodies, frame.poses):
            c = clearance(expand(footprint(body, pose), margin), surface)
            require(c > 0, 'SWEPT_ENVELOPE_RESERVE_VIOLATION')
            minimum = min(minimum, c)
    return minimum


def _exit(vehicle, frame, goal, path, limits):
    p = frame.poses[0]
    require(math.hypot(p.x-goal.x, p.z-goal.z) <= limits.endpoint_position_m,
            'EXIT_POSITION_MISMATCH')
    require(abs(wrap_angle(p.heading-goal.heading)) <= limits.endpoint_heading_rad,
            'EXIT_HEADING_MISMATCH')
    right = np.array((math.cos(goal.heading), -math.sin(goal.heading)))
    gate = path.segments[-1].centerline[0]
    forward = _forward(goal.heading)
    for body, pose in zip(vehicle.bodies, frame.poses):
        require(abs(wrap_angle(pose.heading-goal.heading)) <=
                limits.exit_article_heading_rad, 'ARTICLES_NOT_SETTLED_AT_EXIT')
        for x, z in footprint(body, pose):
            require(float(np.array((x-gate.x,z-gate.z)) @ forward) >
                    vehicle.safety_margin_m+vehicle.uncertainty_m,
                    'ARTICLE_NOT_CLEAR_OF_JUNCTION')
            lateral = abs(float(np.array((x-goal.x, z-goal.z)) @ right))
            # Derived width is an ADDITIONAL restriction at exit, not evidence
            # permitting any footprint outside the independently proven surface.
            require(lateral+vehicle.safety_margin_m+vehicle.uncertainty_m <
                    path.segments[-1].width_m/2, 'ARTICLE_OUTSIDE_EXIT_LANE')


def _request_hash(vehicle, start, path, surface, limits, entry_speed, tyre):
    return digest({'vehicle': asdict(vehicle), 'start': asdict(start),
        'path': lane_path_fingerprint(path), 'surface': asdict(surface),
        'limits': asdict(limits), 'entry_speed': entry_speed, 'entry_tyre': tyre})


def _plan_hash(request, controls, speed, samples, envelope):
    return digest({'request':request,'controls':controls,'speed':speed,
                   'samples':[asdict(sample) for sample in samples],
                   'envelope':asdict(envelope)})


def plan_maneuver(vehicle, start, path, surface, expected_identity,
                  limits=None, entry_speed_mps=0., entry_tyre_rad=0.):
    """Pure offline bounded search. No inferred boundary, traffic or body data."""
    diagnostics = []
    try:
        limits = limits or PlannerLimits()
        limits.validate(); vehicle.validate(); start.validate(vehicle)
        validate_path_identity(path, start.identity, expected_identity)
        require(isinstance(surface, DrivableSurfaceSnapshot), 'MISSING_CONFIRMED_DRIVABLE_BOUNDARY')
        initial = evaluate_on_confirmed_surface(vehicle, (start,), surface, path,
                                                expected_identity)
        require(initial.accepted, initial.failure_reason)
        require(type(entry_speed_mps) in (int, float) and math.isfinite(entry_speed_mps)
                and 0 <= entry_speed_mps <= 30
                and type(entry_tyre_rad) in (int, float) and math.isfinite(entry_tyre_rad)
                and abs(entry_tyre_rad) <= vehicle.max_tyre_rad, 'INVALID_ENTRY_STATE')
        first, goal = path.points[0], path.points[-1]
        p = start.poses[0]
        require(math.dist((p.x, p.y, p.z), (first.x, first.y, first.z)) <=
                limits.endpoint_position_m and
                abs(wrap_angle(p.heading-first.heading)) <= limits.endpoint_heading_rad,
                'START_NOT_AT_CONFIRMED_ENTRY')
        require(abs(goal.curvature) <= 1e-5, 'EXIT_NOT_STRAIGHT')
        # The local family covers a single turn, not route loops/S curves.
        turn = wrap_angle(goal.heading-p.heading)
        require(math.radians(20) <= abs(turn) <= math.radians(120),
                'UNSUPPORTED_MANEUVER_TURN')
        budget_vehicle = replace(vehicle, uncertainty_m=vehicle.uncertainty_m+
            surface.boundary_uncertainty_m+limits.integration_reserve_m)
        start_heads = tuple(s.heading for s in start.poses[1:])
        exact = articulated_poses(vehicle, p, start_heads)
        require(all(math.dist((a.x,a.y,a.z),(b.x,b.y,b.z)) < 1e-8
                    for a,b in zip(start.poses,exact)), 'START_HITCH_NOT_CLOSED')
        candidates = []
        for candidate, ctrl in enumerate(_family(path, p, goal, entry_tyre_rad,
                vehicle.wheelbase_m, limits.max_candidates)):
            try:
                k, delta_s, delta_ss = _limits(ctrl, vehicle.wheelbase_m)
                require(math.atan(vehicle.wheelbase_m*k) <= vehicle.max_tyre_rad,
                        'TYRE_ANGLE_LIMIT')
                speed = min(limits.speed_mps, vehicle.max_speed_mps,
                    math.sqrt(limits.lateral_accel_mps2/max(k,1e-12)),
                    limits.tyre_rate_rad_s/max(delta_s,1e-12),
                    math.sqrt(limits.tyre_accel_rad_s2/max(delta_ss,1e-12)))
                require(speed >= limits.min_speed_mps, 'MANEUVER_SPEED_BELOW_DOMAIN')
                frames, ss, _ = _frames(vehicle,start,ctrl,np.linspace(0,1,97),speed)
                require(ss[-1] >= .9*path.distance_m and ss[-1] <= 1.5*path.distance_m,
                        'MANEUVER_CORRIDOR_LENGTH_MISMATCH')
                _gate_indices(path,frames)
                _exit(budget_vehicle,frames[-1],goal,path,limits)
                minimum = _screen(budget_vehicle, frames, surface.surface, limits)
                # Among valid coarse candidates prefer clearance; length is a
                # small tie-breaker. Final acceptance always uses dense sweeps.
                candidates.append((-minimum+.001*ss[-1],candidate,ctrl,speed,
                                   (k,delta_s,delta_ss)))
                diagnostics.append((candidate,'COARSE_CANDIDATE',minimum))
            except EnvelopeError as error:
                diagnostics.append((candidate,str(error),None))
        for _, candidate, ctrl, speed, bounds in sorted(candidates,key=lambda x:(x[0],x[1])):
            try:
                us = _sample_parameters(ctrl,speed,limits.sample_dt_s)
                frames, ss, ks = _frames(vehicle,start,ctrl,us,speed)
                refined = np.empty(2*len(us)-1)
                refined[::2],refined[1::2] = us,(us[:-1]+us[1:])/2
                fine, _, _ = _frames(vehicle,start,ctrl,refined,speed)
                error = max(math.hypot(a.x-b.x,a.z-b.z)+
                    max(body.front_m,body.rear_m,body.width_m)*abs(a.heading-b.heading)
                    for a_frame,b_frame in zip(frames,fine[::2])
                    for body,a,b in zip(vehicle.bodies,a_frame.poses,b_frame.poses))
                require(error <= limits.integration_reserve_m/4,
                        'UNCONVERGED_ARTICULATED_INTEGRATION')
                indices = _gate_indices(path,frames)
                require(all(abs(wrap_angle(b.heading-a.heading)) <=
                    limits.max_articulation_rad for f in frames
                    for a,b in zip(f.poses,f.poses[1:])), 'ARTICULATION_LIMIT')
                _exit(budget_vehicle,frames[-1],goal,path,limits)
                numerical_vehicle = replace(vehicle, uncertainty_m=
                    vehicle.uncertainty_m+limits.integration_reserve_m)
                envelope = evaluate_on_confirmed_surface(numerical_vehicle,frames,
                    surface,path,expected_identity)
                require(envelope.accepted,envelope.failure_reason)
                samples = tuple(ManeuverSample(f,s,math.atan(vehicle.wheelbase_m*k),k,i,
                    path.segments[i].lane_id,expected_identity.gps_pairs[i])
                    for f,s,k,i in zip(frames,ss,ks,indices))
                request_hash = _request_hash(vehicle,start,path,surface,limits,
                                            entry_speed_mps,entry_tyre_rad)
                controls = tuple(tuple(tuple(map(float,row)) for row in piece) for piece in ctrl)
                token = _plan_hash(request_hash,controls,speed,samples,envelope)
                diagnostics.append((candidate,'ACCEPTED',{
                    'integration_difference_m':error, 'length_m':ss[-1],
                    'curvature_bound_m_inv':bounds[0],
                    'tyre_rate_bound_rad_s':bounds[1]*speed,
                    'tyre_accel_bound_rad_s2':bounds[2]*speed**2,
                    'minimum_clearance_m':envelope.minimum_clearance_m}))
                return ManeuverPlan(True,'',expected_identity,token,request_hash,
                    surface.token,lane_path_fingerprint(path),controls,samples,
                    envelope,speed,entry_speed_mps>speed,tuple(diagnostics))
            except EnvelopeError as error:
                diagnostics.append((candidate,'DENSE_'+str(error),None))
        return ManeuverPlan(False,'NO_VALIDATED_CANDIDATE_IN_SEARCH_BUDGET',
                            expected_identity,diagnostics=tuple(diagnostics))
    except EnvelopeError as error:
        return ManeuverPlan(False,str(error),expected_identity,diagnostics=tuple(diagnostics))
    except (AttributeError,TypeError,ValueError,OverflowError,IndexError):
        return ManeuverPlan(False,'MALFORMED_MANEUVER_INPUT',expected_identity,
                            diagnostics=tuple(diagnostics))


def validate_plan_request(plan, vehicle, start, path, surface, expected_identity,
                          limits, entry_speed_mps=0., entry_tyre_rad=0.):
    """Offline callback guard. A future runtime must ALSO check live pose/time.

    This deliberately grants no use-on-moving-vehicle permission; start is the
    original request state, never a nearest-point substitution from a new tick.
    """
    try:
        require(plan.accepted and not plan.runtime_authorized, 'INVALID_OFFLINE_PLAN')
        require(plan.identity == expected_identity, 'STALE_MANEUVER_IDENTITY')
        validate_path_identity(path,start.identity,expected_identity)
        current = _request_hash(vehicle,start,path,surface,limits,
                                 entry_speed_mps,entry_tyre_rad)
        require(current == plan.request_fingerprint, 'STALE_MANEUVER_REQUEST')
        require(plan.surface_token == surface.token and
                plan.source_path_fingerprint == lane_path_fingerprint(path) and
                plan.requires_entry_speed_reduction == (entry_speed_mps > plan.speed_mps),
                'MANEUVER_RESULT_METADATA_MISMATCH')
        require(plan.envelope is not None and plan.envelope.accepted and
                plan.token == _plan_hash(current,plan.controls_xz,plan.speed_mps,
                                          plan.samples,plan.envelope),
                'MANEUVER_RESULT_INTEGRITY_MISMATCH')
        return ''
    except EnvelopeError as error:
        return str(error)
    except (AttributeError,TypeError,ValueError,OverflowError):
        return 'MALFORMED_MANEUVER_REQUEST'
