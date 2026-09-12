"""Phase 5A: observational low-speed articulated footprint model.

No controller, planner, runtime imports, telemetry defaults or map mutations.
World coordinates are ETS2 X/Y/Z, Y is up. Heading is SDK rotationX in radians:
forward=(-sin(h), 0, -cos(h)); right=(cos(h), 0, -sin(h)). Positive physical
tyre angle is right, so heading_dot=-v*tan(delta)/wheelbase.

Bodies are conservative rectangular footprints relative to their fixed axle.
All dimensions must include mirrors/body overhangs. Track width is NOT body
width. Hitch offsets are longitudinal metres forward of that same axle.
Only a confirmed horizontal surface and fixed, non-steered trailer axles are
supported. Grade, roll, tyre slip, rear steer and unproven tandem equivalents
must be resolved upstream; they cannot acquire clearance authority here.

Sweeps use endpoint convex hulls expanded by a conservative intersample bound:
every body point travels <= corner_speed_bound*dt/2 to its nearest endpoint.
A square Minkowski expansion encloses that Euclidean ball, including rotation.
This covers continuous bounded motion, not merely the two endpoint polygons.
Clearance is a conservative lower bound under the supplied geometry/motion
evidence, not a declaration that a real map corridor has been verified.
"""
from dataclasses import dataclass
import math

from core.navigation.lane_model import LaneId

Point = tuple[float, float]  # X/Z
Ring = tuple[Point, ...]
EPS = 1e-9
COORDINATE_LIMIT_M = 1_000_000.  # explicit numerical domain of polygon predicates


class EnvelopeError(ValueError):
    """Stable machine-readable refusal; never silently substitute geometry."""


def require(ok, reason):
    if not ok:
        raise EnvelopeError(reason)


def finite(*values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


def cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def edges(ring):
    return zip(ring, ring[1:]+ring[:1])


def point_segment(p, a, b):
    dx, dz = b[0]-a[0], b[1]-a[1]
    den = dx*dx+dz*dz
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dz)/den)) if den else 0.
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dz)


def intersects(a, b, c, d):
    if (max(a[0],b[0])+EPS < min(c[0],d[0]) or
            max(c[0],d[0])+EPS < min(a[0],b[0]) or
            max(a[1],b[1])+EPS < min(c[1],d[1]) or
            max(c[1],d[1])+EPS < min(a[1],b[1])):
        return False
    if any(point_segment(p, u, v) <= EPS for p, u, v in
           ((a,c,d), (b,c,d), (c,a,b), (d,a,b))):
        return True
    return cross(a,b,c)*cross(a,b,d) < 0 and cross(c,d,a)*cross(c,d,b) < 0


def inside(p, ring):
    if any(point_segment(p, a, b) <= EPS for a, b in edges(ring)):
        return True
    hit = False
    for a, b in edges(ring):
        if (a[1] > p[1]) != (b[1] > p[1]):
            if p[0] < a[0]+(p[1]-a[1])*(b[0]-a[0])/(b[1]-a[1]):
                hit = not hit
    return hit


def hull(points):
    points = sorted(set(points))
    def half(seq):
        out = []
        for p in seq:
            while len(out) > 1 and cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out
    return tuple(half(points)[:-1]+half(points[::-1])[:-1])


def expand(poly, amount):
    # Circumscribed square: never approximate a circle by an inscribed polygon.
    return hull((x+dx, z+dz) for x,z in poly
                for dx,dz in ((-amount,-amount),(-amount,amount),
                              (amount,-amount),(amount,amount)))


def validate_ring(ring):
    require(isinstance(ring, tuple) and 3 <= len(ring) <= 2048,
            'INVALID_BOUNDARY_RING')
    require(all(isinstance(p, tuple) and len(p) == 2 and finite(*p) for p in ring),
            'NONFINITE_BOUNDARY')
    require(all(abs(v) <= COORDINATE_LIMIT_M for p in ring for v in p),
            'BOUNDARY_OUTSIDE_NUMERICAL_DOMAIN')
    require(len(set(ring)) == len(ring), 'DUPLICATE_BOUNDARY_VERTEX')
    require(abs(sum(cross(ring[0],a,b) for a,b in edges(ring))) > EPS,
            'DEGENERATE_BOUNDARY')
    for i,b in enumerate(ring):
        a,c = ring[i-1],ring[(i+1)%len(ring)]
        require(math.dist(a,b) > EPS, 'DEGENERATE_BOUNDARY_EDGE')
        require(not (abs(cross(a,b,c)) <= EPS and
                     (a[0]-b[0])*(c[0]-b[0])+(a[1]-b[1])*(c[1]-b[1]) > 0),
                'BACKTRACKING_BOUNDARY_EDGE')
    es = list(edges(ring))
    for i,(a,b) in enumerate(es):
        for j in range(i+1,len(es)):
            if j == i+1 or (i == 0 and j == len(es)-1):
                continue
            require(not intersects(a,b,*es[j]), 'SELF_INTERSECTING_BOUNDARY')


@dataclass(frozen=True)
class Identity:
    intent: str
    revision: int
    build: str
    session: str
    map_key: str
    dataset: str
    layer: str
    lanes: tuple[LaneId, ...]
    gps_pairs: tuple[tuple[int, int], ...]

    def validate(self):
        require(all(isinstance(v,str) and v.strip() for v in
                    (self.intent,self.build,self.session,self.map_key,self.dataset,self.layer)),
                'MISSING_IDENTITY')
        require(type(self.revision) is int and self.revision >= 0, 'INVALID_REVISION')
        require(isinstance(self.lanes,tuple) and bool(self.lanes)
                and all(isinstance(l,LaneId) for l in self.lanes), 'MISSING_LANE_SEQUENCE')
        require(all(type(l.road_uid) is int and l.road_uid >= 0
                    and type(l.direction) is int and l.direction in (-1,1)
                    and type(l.lane_index) is int and l.lane_index >= 0
                    and isinstance(l.connector_path,tuple)
                    and all(type(i) is int and i >= 0 for i in l.connector_path)
                    for l in self.lanes), 'INVALID_LANE_IDENTITY')
        require(isinstance(self.gps_pairs,tuple) and len(self.gps_pairs)==len(self.lanes)
                and all(isinstance(p,tuple) and len(p)==2
                        and all(type(v) is int and v >= 0 for v in p)
                        for p in self.gps_pairs), 'INVALID_GPS_PAIRS')


@dataclass(frozen=True)
class Body:
    name: str
    width_m: float
    front_m: float  # axle to foremost body extent (tractor includes wheelbase)
    rear_m: float   # axle to rearmost body extent
    hitch_front_m: float  # axle to incoming hitch; tractor uses 0
    hitch_rear_m: float   # outgoing hitch, signed FORWARD of axle
    source: str
    confirmed: bool
    axle_model: str  # explicit evidence, not a default inferred from wheel positions

    def validate(self, trailer=False):
        require(self.axle_model == 'fixed_axle', 'UNSUPPORTED_AXLE_MODEL')
        require(self.confirmed is True and isinstance(self.source,str)
                and bool(self.source.strip()), 'UNPROVEN_BODY_DIMENSIONS')
        require(isinstance(self.name,str) and bool(self.name.strip())
                and finite(self.width_m,self.front_m,self.rear_m,
                    self.hitch_front_m,self.hitch_rear_m), 'INVALID_BODY_DIMENSIONS')
        require(0 < self.width_m <= 5 and 0 < self.front_m <= 25
                and 0 <= self.rear_m <= 15, 'UNSUPPORTED_BODY_DIMENSIONS')
        require(-self.rear_m <= self.hitch_rear_m <= self.front_m,
                'HITCH_OUTSIDE_BODY')
        require((0 < self.hitch_front_m <= self.front_m) if trailer
                else self.hitch_front_m == 0, 'INVALID_INCOMING_HITCH')


@dataclass(frozen=True)
class Vehicle:
    bodies: tuple[Body, ...]
    wheelbase_m: float
    max_tyre_rad: float
    max_speed_mps: float
    safety_margin_m: float
    uncertainty_m: float  # bound on ALL footprint-point errors, including yaw/integration

    def validate(self):
        require(isinstance(self.bodies,tuple) and 1 <= len(self.bodies) <= 5,
                'UNSUPPORTED_ARTICLE_COUNT')
        for i,b in enumerate(self.bodies):
            b.validate(i>0)
        require(len({b.name for b in self.bodies}) == len(self.bodies), 'DUPLICATE_BODY_NAME')
        require(finite(self.wheelbase_m,self.max_tyre_rad,self.max_speed_mps,
                       self.safety_margin_m,self.uncertainty_m), 'INVALID_MODEL_LIMITS')
        require(2 <= self.wheelbase_m <= self.bodies[0].front_m
                and 0 < self.max_tyre_rad < 1.2 and 0 < self.max_speed_mps <= 5,
                'OUTSIDE_LOW_SPEED_MODEL')
        require(self.safety_margin_m > 0 and self.uncertainty_m > 0,
                'MISSING_POSITIVE_RESERVE')
        require(all(finite(*bound) for bound in self.motion_bounds()),
                'UNBOUNDED_MOTION_MODEL')

    def motion_bounds(self):
        """Global axle speed/yaw/corner speed bounds, including off-axle hitches."""
        speed = self.max_speed_mps
        yaw = speed*math.tan(self.max_tyre_rad)/self.wheelbase_m
        bounds = []
        for i,b in enumerate(self.bodies):
            if i:
                speed += abs(self.bodies[i-1].hitch_rear_m)*yaw
                yaw = speed/b.hitch_front_m
            radius = math.hypot(max(b.front_m,b.rear_m), b.width_m/2)
            bounds.append((speed,yaw,speed+radius*yaw))
        return tuple(bounds)


@dataclass(frozen=True)
class Pose:
    """Fixed axle ground projection; y is the confirmed support plane elevation.

    An SDK chassis/axle height is not this ground-reference pose. Conversion
    requires measured static geometry and support-plane evidence upstream.
    """
    x: float
    y: float
    z: float
    heading: float
    pitch: float = 0.
    roll: float = 0.


def offset(p, forward):
    return Pose(p.x-forward*math.sin(p.heading), p.y,
                p.z-forward*math.cos(p.heading), p.heading, p.pitch, p.roll)


def footprint(body, pose):
    s,c = math.sin(pose.heading), math.cos(pose.heading)
    return tuple((pose.x-f*s+r*c, pose.z-f*c-r*s)
                 for f,r in ((body.front_m,-body.width_m/2),
                             (body.front_m,body.width_m/2),
                             (-body.rear_m,body.width_m/2),
                             (-body.rear_m,-body.width_m/2)))


@dataclass(frozen=True)
class Frame:
    time_s: float  # monotonic, not wall clock
    identity: Identity
    poses: tuple[Pose, ...]  # fixed axle poses, NOT SDK chassis origins

    def validate(self, vehicle):
        self.identity.validate()
        require(finite(self.time_s) and self.time_s >= 0, 'INVALID_MONOTONIC_TIME')
        require(isinstance(self.poses,tuple) and len(self.poses)==len(vehicle.bodies),
                'MISSING_ARTICLE_POSE')
        for p in self.poses:
            require(finite(p.x,p.y,p.z,p.heading,p.pitch,p.roll), 'NONFINITE_POSE')
            require(max(abs(p.x),abs(p.y),abs(p.z)) <= COORDINATE_LIMIT_M
                    and abs(p.heading) <= 1000*math.tau, 'POSE_OUTSIDE_NUMERICAL_DOMAIN')
            require(p.pitch == 0 and p.roll == 0, 'UNSUPPORTED_BODY_TILT')
            require(abs(p.y-self.poses[0].y) <= EPS, 'WRONG_ELEVATION_OR_GRADE')
        for i in range(1,len(self.poses)):
            a = offset(self.poses[i-1],vehicle.bodies[i-1].hitch_rear_m)
            b = offset(self.poses[i],vehicle.bodies[i].hitch_front_m)
            require(math.hypot(a.x-b.x,a.z-b.z) <= vehicle.uncertainty_m,
                    'DISCONNECTED_HITCH')


@dataclass(frozen=True)
class Surface:
    identity: Identity
    exterior: Ring
    holes: tuple[Ring, ...]
    y_m: float
    source: str
    confirmed: bool
    horizontal: bool

    def validate(self):
        self.identity.validate()
        require(self.confirmed is True and isinstance(self.source,str)
                and bool(self.source.strip()), 'UNPROVEN_DRIVABLE_BOUNDARY')
        require(self.horizontal is True and finite(self.y_m), 'UNSUPPORTED_SURFACE_GRADE')
        require(isinstance(self.holes,tuple) and len(self.holes) <= 32, 'INVALID_HOLES')
        require(sum(len(r) for r in (self.exterior,)+self.holes) <= 4096,
                'BOUNDARY_VERTEX_BUDGET_EXCEEDED')
        validate_ring(self.exterior)
        for i,hole in enumerate(self.holes):
            validate_ring(hole)
            require(all(inside(p,self.exterior) for p in hole), 'HOLE_OUTSIDE_SURFACE')
            for other in (self.exterior,)+self.holes[:i]:
                require(not any(intersects(a,b,c,d) for a,b in edges(hole)
                                for c,d in edges(other)), 'INTERSECTING_BOUNDARIES')
            for other in self.holes[:i]:
                require(not inside(hole[0],other) and not inside(other[0],hole),
                        'NESTED_HOLES')


def clearance(poly, surface):
    """Distance of an entire convex polygon to a concave area including holes.

    Zero means touching/overlapping/outside (not an invented penetration depth).
    Callers distinguish this from a positive distance insufficient for reserve.
    """
    if not all(inside(p,surface.exterior) for p in poly):
        return 0.
    for hole in surface.holes:
        if any(inside(p,hole) for p in poly) or inside(hole[0],poly):
            return 0.
    distance = math.inf
    for ring in (surface.exterior,)+surface.holes:
        for a,b in edges(poly):
            for c,d in edges(ring):
                if intersects(a,b,c,d):
                    return 0.
                dx = max(0.,min(a[0],b[0])-max(c[0],d[0]),
                         min(c[0],d[0])-max(a[0],b[0]))
                dz = max(0.,min(a[1],b[1])-max(c[1],d[1]),
                         min(c[1],d[1])-max(a[1],b[1]))
                if dx*dx+dz*dz > distance*distance:
                    continue
                distance = min(distance,point_segment(a,c,d),point_segment(b,c,d),
                               point_segment(c,a,b),point_segment(d,a,b))
    return distance


@dataclass(frozen=True)
class EnvelopeStep:
    start_s: float
    end_s: float
    footprints: tuple[Ring, ...]
    swept_polygons: tuple[Ring, ...]
    corner_motion_bounds_m: tuple[float, ...]
    clearance_m: tuple[float, ...]  # after safety+uncertainty+motion expansion
    y_m: float


@dataclass(frozen=True)
class EnvelopeResult:
    accepted: bool
    failure_reason: str
    identity: Identity
    steps: tuple[EnvelopeStep, ...] = ()
    minimum_clearance_m: float | None = None
    limiting_body: str | None = None


def evaluate(vehicle, frames, surface, expected_identity):
    """Pure offline assessment; never publishes a navigation/control packet.

    Acceptance is conditional on confirmed dimensions/surface and bounded
    low-speed fixed-axle motion. It grants no traffic or driving permission.
    """
    try:
        expected_identity.validate()
        vehicle.validate()
        require(surface is not None, 'MISSING_DRIVABLE_BOUNDARY')
        surface.validate()
        require(surface.identity == expected_identity, 'STALE_SURFACE_IDENTITY')
        require(isinstance(frames,tuple) and 1 <= len(frames) <= 20000, 'INVALID_FRAME_SEQUENCE')
        bounds = vehicle.motion_bounds()
        steps = []
        minimum, limiting = math.inf, None
        previous = None
        for frame in frames:
            require(frame.identity == expected_identity, 'STALE_FRAME_IDENTITY')
            frame.validate(vehicle)
            for p in frame.poses:
                require(abs(p.y-surface.y_m) <= EPS, 'WRONG_ELEVATION_OR_GRADE')
            dt = frame.time_s-previous.time_s if previous else 0.
            if previous:
                require(0 < dt <= .25, 'INVALID_OR_UNBOUNDED_SAMPLE_GAP')
                for a,b,(v,w,_) in zip(previous.poses,frame.poses,bounds):
                    require(math.hypot(a.x-b.x,a.z-b.z) <= v*dt+2*vehicle.uncertainty_m,
                            'MOTION_EXCEEDS_MODEL')
                    # Unwrapped headings are intentional; do not hide a full revolution.
                    require(abs(a.heading-b.heading) <= w*dt+EPS, 'YAW_EXCEEDS_MODEL')
            polys = tuple(footprint(b,p) for b,p in zip(vehicle.bodies,frame.poses))
            swept, pads, distances = [], [], []
            for i,(body,poly) in enumerate(zip(vehicle.bodies,polys)):
                pad = bounds[i][2]*dt/2
                base = hull(poly+footprint(body,previous.poses[i])) if previous else poly
                expanded = expand(base,pad+vehicle.safety_margin_m+vehicle.uncertainty_m)
                require(all(finite(*p) for p in expanded), 'UNBOUNDED_SWEPT_POLYGON')
                dist = clearance(expanded,surface)
                require(math.isfinite(dist), 'UNBOUNDED_CLEARANCE')
                swept.append(expanded); pads.append(pad); distances.append(dist)
                if dist < minimum:
                    minimum,limiting = dist,body.name
            steps.append(EnvelopeStep(previous.time_s if previous else frame.time_s,
                         frame.time_s,polys,tuple(swept),tuple(pads),tuple(distances),surface.y_m))
            previous = frame
        return EnvelopeResult(minimum > EPS,
            '' if minimum > EPS else 'SWEPT_ENVELOPE_RESERVE_VIOLATION',
            expected_identity,tuple(steps),minimum,limiting)
    except EnvelopeError as error:
        return EnvelopeResult(False,str(error),expected_identity)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return EnvelopeResult(False,'MALFORMED_ENVELOPE_INPUT',expected_identity)


def articulated_poses(vehicle, tractor, trailer_headings):
    """Reconstruct axle placements from explicit articulation, preserving hitches."""
    vehicle.validate()
    require(finite(tractor.x,tractor.y,tractor.z,tractor.heading,
                   tractor.pitch,tractor.roll,*trailer_headings), 'NONFINITE_POSE')
    require(tractor.pitch == 0 and tractor.roll == 0, 'UNSUPPORTED_BODY_TILT')
    require(len(trailer_headings) == len(vehicle.bodies)-1, 'MISSING_ARTICULATION')
    poses = [tractor]
    for i,h in enumerate(trailer_headings,1):
        hitch = offset(poses[-1],vehicle.bodies[i-1].hitch_rear_m)
        poses.append(offset(Pose(hitch.x,hitch.y,hitch.z,h),-vehicle.bodies[i].hitch_front_m))
    return tuple(poses)


def propagate(vehicle, frame, speed_mps, tyre_rad, dt_s):
    """Diagnostic RK4 no-slip model for a CONSTANT physical input <= 20 ms.

    This is not a planner or an actuator model. No steering gain/lag is invented.
    For upstream hitch velocity H and trailer heading h:
      omega = dot(H,(-cos(h),sin(h)))/hitch_length
      axle_velocity = H - hitch_length*omega*(-cos(h),sin(h)).
    Off-axle outgoing hitch velocity includes its parent's angular velocity.
    """
    vehicle.validate()
    frame.validate(vehicle)
    for i in range(1,len(frame.poses)):
        a=offset(frame.poses[i-1],vehicle.bodies[i-1].hitch_rear_m)
        b=offset(frame.poses[i],vehicle.bodies[i].hitch_front_m)
        require(math.hypot(a.x-b.x,a.z-b.z) <= EPS,
                'PROPAGATION_REQUIRES_CLOSED_HITCH')
    require(finite(speed_mps,tyre_rad,dt_s) and 0 <= speed_mps <= vehicle.max_speed_mps
            and abs(tyre_rad) <= vehicle.max_tyre_rad and 0 < dt_s <= .02,
            'UNSUPPORTED_MOTION_INPUT')
    require(max(w for _,w,_ in vehicle.motion_bounds())*dt_s <= .05,
            'INTEGRATION_STEP_TOO_LARGE')
    require(len(frame.poses)==len(vehicle.bodies), 'MISSING_ARTICLE_POSE')
    p = frame.poses[0]
    q = (p.x,p.z)+tuple(pose.heading for pose in frame.poses)
    require(finite(*q,p.y,frame.time_s), 'NONFINITE_POSE')
    def derivative(state):
        hs = state[2:]
        vx,vz = -speed_mps*math.sin(hs[0]),-speed_mps*math.cos(hs[0])
        w = -speed_mps*math.tan(tyre_rad)/vehicle.wheelbase_m
        out = [vx,vz,w]
        for i,h in enumerate(hs[1:],1):
            prev = hs[i-1]
            offset_m = vehicle.bodies[i-1].hitch_rear_m
            hx,hz = vx-offset_m*w*math.cos(prev),vz+offset_m*w*math.sin(prev)
            length = vehicle.bodies[i].hitch_front_m
            w = (-hx*math.cos(h)+hz*math.sin(h))/length
            vx,vz = hx+length*w*math.cos(h),hz-length*w*math.sin(h)
            out.append(w)
        return tuple(out)
    def shifted(k,scale):
        return tuple(a+scale*b for a,b in zip(q,k))
    k1=derivative(q); k2=derivative(shifted(k1,dt_s/2))
    k3=derivative(shifted(k2,dt_s/2)); k4=derivative(shifted(k3,dt_s))
    out=tuple(a+dt_s*(b+2*c+2*d+e)/6 for a,b,c,d,e in zip(q,k1,k2,k3,k4))
    poses=articulated_poses(vehicle,Pose(out[0],p.y,out[1],out[2]),out[3:])
    return Frame(frame.time_s+dt_s,frame.identity,poses)
