"""
Coordinate-based route navigation for UltraPilot.

A :class:`Route` is a polyline of world ``(x, z)`` waypoints captured from SCS
telemetry. Given the truck's current world pose it produces a steering value in
``[-1, 1]`` from one signed Frenet bicycle controller on the validated lane
geometry. Curvature, heading and cross-track feedback are expressed in 1/m
and converted to the game input only once.

This drives the truck along a previously-recorded path with no game-map data or
vision — purely from world coordinates.  Sign convention: positive steering =
steer right; the world uses ETS2's heading where ``forward = (-sin h, -cos h)``.
"""

import bisect
import json
import math
import os
from typing import List, Optional, Sequence, Tuple

from core.lateral_controller import (
    ACTUATION_PREVIEW_S, MAX_STEERING_LOCK_RAD, MIN_STEERING_LOCK_RAD,
    REFERENCE_LOCK_RAD, WHEELBASE_M,
    offset_frame, solve as solve_lateral,
)

Point = Tuple[float, float]


def iter_path_xz(points):
    """Yield finite ETS2 world ``(X, Z)`` coordinates from 2D/3D paths.

    Lane trajectories contain ``[X, Y, Z]`` while legacy recorded routes use
    ``[X, Z]``. Ground-plane consumers must not unpack these formats directly.
    """
    for point in points or ():
        if not isinstance(point, (list, tuple)):
            continue
        try:
            if len(point) >= 3:
                x, z = float(point[0]), float(point[2])
            elif len(point) >= 2:
                x, z = float(point[0]), float(point[1])
            else:
                continue
        except (TypeError, ValueError, OverflowError):
            continue
        if math.isfinite(x) and math.isfinite(z):
            yield x, z

# Legacy Stanley calibration values remain public for compatibility and audit
# diagnostics. Authoritative lane steering below does not add these independent
# heading/CTE regulators. They are not the gains of the active Frenet law.
K_HEADING = 2.60
K_CTE = 1.05              # broad-road/straight lateral correction [1/s]
K_CTE_CURVE = 1.70        # proven-bend lateral correction [1/s]
K_SOFT = 1.0              # speed softening [m/s] keeps CTE finite at v=0
# Historical diagnostic category only; no separate low-speed controller.
LOW_SPEED_CAPTURE_MAX_MS = 6.0
# Reference chassis and equivalent game-input calibration. These are model
# parameters, not geometric safety limits; see core.lateral_controller.
TRUCK_WHEELBASE_M = WHEELBASE_M
NORMALIZED_STEERING_ANGLE_RAD = REFERENCE_LOCK_RAD
STEERING_CURVATURE_SPAN_M = 8.0
# The lane trajectory is resampled near two metres and therefore contains a
# small deterministic curvature quantisation.  Steering represents the
# tractor wheelbase footprint with a symmetric spatial quadrature around the
# reference progress.  This is a calculation over immutable road geometry,
# not a time-domain filter of commands or telemetry.
STEERING_CURVATURE_SAMPLE_OFFSETS_M = (-4.0, -2.0, 0.0, 2.0, 4.0)
STEERING_CURVATURE_SAMPLE_WEIGHTS = (0.10, 0.20, 0.40, 0.20, 0.10)
# Only the historical _pursuit_geometry_solution test utility uses these.
# Route.steering() never calls it; neither value is a runtime control gain.
STEERING_PURSUIT_MAX_M = 28.0
# LaneTrajectory is sampled at roughly two metres.  A single target can land
# on one quantised lateral sample and turn harmless centimetre-scale map noise
# into a visible wheel pulse.  These are simultaneous distances on the same
# immutable geometry (not past/future command samples).  On a true circular
# lane every horizon yields the same bicycle curvature; averaging therefore
# removes sample quantisation without flattening a real bend.
STEERING_PURSUIT_OFFSETS_M = (
    (-4.0, 0.20),
    (-2.0, 0.20),
    (0.0, 0.20),
    (2.0, 0.20),
    (4.0, 0.20),
)
# Estimate the Frenet tangent across a physical tractor-length.  A 1.5 m
# secant was shorter than the map's two-metre resampling interval and turned
# harmless lane-point quantisation into alternating heading error.
STEERING_REFERENCE_TANGENT_M = 6.0
# A live LaneId is already accepted by LaneLocator with its 2.4 m lateral
# gate. Route must not project that identity onto a more distant future run
# merely because it occurs later in the same trajectory.
AUTHORITY_PROJECTION_MAX_DISTANCE_M = 2.4
# Elevation layers are quantised from each segment's midpoint.  A continuous
# uphill road can therefore change layer at a perfectly valid boundary.  Only
# derivative sampling (never lane identity or projection) may cross such a
# change, and only over one adjacent validated trajectory sample with a small
# real 3-D residual.  A bridge/road crossing remains many metres apart in Y and
# cannot satisfy these bounds.
AUTHORITY_DERIVATIVE_MAX_SAMPLE_GAP_M = 3.25
AUTHORITY_DERIVATIVE_MAX_VERTICAL_STEP_M = 0.75
# Road lanes encode direction against the road item while prefab connectors
# encode it against the connector definition.  Their raw +/- flags can differ
# on one correctly directed road->prefab edge.  In that heterogeneous case we
# require the incoming, boundary and outgoing X/Z tangents to agree within
# 70 degrees before a derivative may cross the already validated edge.
AUTHORITY_DERIVATIVE_MAX_HEADING_JUMP_RAD = math.radians(70.0)
# A lane-centred tractor does not imply a lane-contained semi-trailer.  The
# trailer axle follows a smaller radius.  These dimensions are deliberately
# conservative and the resulting tractor offset is always capped by the
# confirmed lane width; no map point or LaneId is moved.
TRAILER_EFFECTIVE_AXLE_DISTANCE_M = 8.0
TRACTOR_BODY_WIDTH_M = 2.55
VEHICLE_ENVELOPE_MARGIN_M = 0.15
TRAILER_POSE_MIN_DISTANCE_M = 1.5
TRAILER_POSE_MAX_DISTANCE_M = 24.0
TRAILER_PROGRESS_BEHIND_MAX_M = 32.0
TRAILER_VERTICAL_TOLERANCE_M = 4.0
# On a constant-radius bend the trailer axle cuts inward by
# sqrt(R^2 + Ltr^2) - R.  Half is the analytic min-max centre: tractor and
# trailer axle then use equal portions of the confirmed lane envelope.  One
# percent of the same spatial prediction covers polyline/chassis discretising
# error in the R18 articulated replay; unlike the former 60% reserve it does
# not deliberately hold the tractor far outside the lane centre through the
# whole bend. Live trailer CTE never changes this fraction or its side.
TRAILER_BALANCED_REFERENCE_FRACTION = 0.51
# A longer window is retained for anticipatory curve braking; steering uses
# the shorter local window above so it cannot cut across a bend.
CURV_WINDOW_M = 60.0
CURVE_PROFILE_STEP_M = 4.0
TIGHT_CURVE_RADIUS = 60.0
# These values now drive audit fields only. They report whether LaneLocator and
# Route projection agree and whether a tight curve is approaching; neither
# replaces or clamps the geometric steering command.
CURVE_DIRECTION_HOLD_CTE_AGREEMENT_M = 0.45
ARRIVAL_RADIUS = 12.0     # metres from the last point counts as "arrived"


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def speed_gain(speed_ms: float) -> float:
    """Gentler steering at speed, sharper when crawling (like ETS2LA's schedule)."""
    speed_kmh = abs(speed_ms) * 3.6
    # 1.3 at standstill → ~0.5 at 90 km/h, floored.
    return _clamp(1.3 - (speed_kmh / 90.0) * 0.8, 0.45, 1.3)


def curve_cte_gain(radius_m: float, lateral_error_m: float = 0.0) -> float:
    """Return the legacy radius gain for diagnostics/calibration tooling.

    Authoritative Route steering no longer multiplies CTE by this independent
    gain; preview pursuit provides the lateral response in the same geometry
    as its curve and heading terms.  Keeping this pure helper preserves the
    existing diagnostics and historical calibration tests.
    """
    radius = float(radius_m)
    curve_weight = _clamp(
        (500.0 - radius) / (500.0 - TIGHT_CURVE_RADIUS), 0.0, 1.0)
    # A transient localisation error is not permission for extra steering
    # authority. The removed error-dependent gain reached 8.0 and turned the
    # captured 0.093 command into full 0.700 lock within one second.
    return K_CTE + (K_CTE_CURVE - K_CTE) * curve_weight


def curve_speed_limit_ms(radius_m: float, distance_m: float,
                         lateral_accel_ms2: float = 1.6,
                         approach_decel_ms2: float = 1.6) -> float:
    """Return a speed envelope that reaches a bend at safe apex speed."""
    try:
        radius = float(radius_m)
        distance = max(0.0, float(distance_m))
    except (TypeError, ValueError, OverflowError):
        return float("inf")
    if not math.isfinite(radius) or radius <= 0.0 or radius >= 2000.0:
        return float("inf")
    # Validated junctions and roundabouts legitimately have radii below 30 m.
    # Clamp only the numerical floor; never discard the sharpest bends.
    radius = max(6.0, radius)
    apex_speed2 = max(0.0, float(lateral_accel_ms2)) * radius
    return math.sqrt(apex_speed2 + 2.0 * max(
        0.0, float(approach_decel_ms2)) * distance)


class Route:
    def __init__(self, points: Optional[Sequence[Point]] = None, name: str = "route",
                 point_authorities: Optional[Sequence[object]] = None):
        self.world_points = [tuple(p) for p in (points or [])]
        # Lane trajectories are world (X,Y,Z); steering remains strictly X/Z.
        self.points: List[Point] = [
            (float(p[0]), float(p[2])) if len(p) >= 3
            else (float(p[0]), float(p[1]))
            for p in self.world_points]
        self.name = name
        # Arc-length metadata and a progress-aware projection cache.  A global
        # nearest-segment search is ambiguous where a route crosses itself or
        # passes another arm of a roundabout.  Once acquired, tracking must
        # advance along the confirmed polyline instead of jumping to whichever
        # geometrically-near arm happens to win by a few centimetres.
        self._segment_lengths: List[float] = []
        self._cumulative_m: List[float] = [0.0]
        for first, second in zip(self.points, self.points[1:]):
            length = math.dist(first, second)
            self._segment_lengths.append(length)
            self._cumulative_m.append(self._cumulative_m[-1] + length)
        self._tracking_state = None
        self._point_authorities = list(point_authorities or ())
        if len(self._point_authorities) != len(self.points):
            self._point_authorities = []
        self._authority_segments = {}
        self._authority_projection_segments = {}
        self._authority_runs = {}
        if self._point_authorities:
            for index in range(len(self.points) - 1):
                first = self._point_authorities[index]
                second = self._point_authorities[index + 1]
                if first == second:
                    self._authority_segments.setdefault(first, []).append(index)
                    self._authority_projection_segments.setdefault(
                        first, []).append(index)
            run_start = 0
            for index in range(1, len(self._point_authorities) + 1):
                if (index == len(self._point_authorities)
                        or self._point_authorities[index]
                            != self._point_authorities[run_start]):
                    authority = self._point_authorities[run_start]
                    if index - run_start >= 2:
                        self._authority_runs.setdefault(authority, []).append(
                            (run_start, index - 1))
                    run_start = index
            # A validated trajectory changes point ownership at the directed
            # segment joining two adjacent LaneIds.  Omitting that segment
            # left a longitudinal hole: LaneLocator could already confirm the
            # destination identity while Route projection was clamped to its
            # first *same-identity* segment one sample farther ahead.  Give the
            # immutable transition edge to both endpoint identities, but only
            # when the existing 3-D/direction proof accepts it.  This does not
            # add a chord or connectivity; the segment already exists in the
            # revision-bound LanePath in exactly this order.
            for index in range(len(self.points) - 1):
                first = self._point_authorities[index]
                second = self._point_authorities[index + 1]
                if (first != second and self._authority_geometry_compatible(
                        first, second, index, index + 1)):
                    for authority in (first, second):
                        segments = self._authority_projection_segments.setdefault(
                            authority, [])
                        if index not in segments:
                            segments.append(index)
            for segments in self._authority_projection_segments.values():
                segments.sort()
        self.last_steering_debug = {
            "feed_forward": 0.0, "feedback": 0.0,
            "local_curvature": 0.0, "raw": 0.0, "output": 0.0,
            "curve_direction_hold": False,
            "curve_direction_hold_eligible": False,
            "curve_direction_hold_approach": False,
        }

    def _compose_curve_steering(
            self, feed_forward: float, heading_feedback: float,
            cte_feedback: float) -> Tuple[float, dict]:
        """Return one stateless geometric steering command.

        All three inputs use the same normalized road-wheel unit.  The caller
        obtains them as an exact counterfactual decomposition of *one* preview
        pursuit solution, not as three independently tuned regulators.  Their
        sum is consequently the geometric solution itself.

        The former sign projection treated every non-zero floating-point
        curvature -- including a logged ``-0.000`` on a straight -- as a
        monotonic left bend.  It then forced a necessary +0.250 .. +0.327 lane
        recovery to zero until CTE grew beyond localisation authority.  A real
        truck may also need a small steering angle opposite the local curve
        while it is already crossing back toward centre.  Clipping that valid
        geometry is not a safety proof.

        ``SteeringDynamics`` is deliberately the only temporal/rate-limiting
        stage.  This pure function has no proof timers, hysteresis, filtering
        or hidden correction authority.
        """
        feedback = float(heading_feedback) + float(cte_feedback)
        candidate = float(feed_forward) + feedback
        curve_sign = (1 if feed_forward > 1e-12
                      else -1 if feed_forward < -1e-12 else 0)
        return float(candidate), {
            "curve_foundation_active": bool(curve_sign),
            "curve_foundation_sign": int(curve_sign),
            # Kept in the diagnostic schema so old log tooling remains able to
            # parse the record.  Phase 4F never projects a valid pursuit angle.
            "curve_sign_projection_active": False,
            "raw_feedback": float(feedback),
            "feedback_applied": float(feedback),
        }

    @staticmethod
    def _pursuit_steering_angle(origin: Point, heading: float,
                                target: Point) -> Tuple[float, float, float]:
        """Road-wheel angle for one immutable preview target.

        ``heading - bearing`` follows UltraPilot's positive-right convention.
        The nonlinear bicycle/pure-pursuit equation is exact on a circle and
        naturally blends curve entry, lateral capture and yaw correction into
        one target.  No previous command or telemetry sample is consulted.
        """
        dx = float(target[0]) - float(origin[0])
        dz = float(target[1]) - float(origin[1])
        distance = math.hypot(dx, dz)
        if distance < 0.25:
            return 0.0, 0.0, float(distance)
        bearing = math.atan2(-dx, -dz)
        bearing_error = (
            (float(heading) - bearing + math.pi)
            % (2.0 * math.pi) - math.pi)
        angle = math.atan2(
            2.0 * TRUCK_WHEELBASE_M * math.sin(bearing_error),
            distance)
        return float(angle), float(bearing_error), float(distance)

    def _pursuit_geometry_solution(
            self, origin: Point, heading: float, progress: float,
            lookahead_m: float, maximum_progress: float
            ) -> Tuple[float, float, float, float, Point]:
        """Fit one pursuit demand to several simultaneous path horizons.

        All targets belong to the same validated authority bounds.  The
        weighted road-wheel angles are identical on an ideal circle, while
        alternating two-metre resampling error cancels spatially.  No prior
        Route or steering state is read or written.
        """
        weighted_angle = 0.0
        weighted_bearing = 0.0
        weighted_distance = 0.0
        weighted_progress = 0.0
        total_weight = 0.0
        representative_target = self._point_at_progress(progress)
        used_progresses = []
        for offset_m, weight in STEERING_PURSUIT_OFFSETS_M:
            target_progress = min(
                float(maximum_progress),
                float(progress) + min(
                    STEERING_PURSUIT_MAX_M,
                    max(0.5, float(lookahead_m) + float(offset_m))))
            # At an authority endpoint several requested horizons can clamp
            # to the same map point. Counting that one endpoint repeatedly
            # changes the geometric weighting solely because a LaneId run
            # ended, producing an otherwise unexplained steering pulse.  A
            # physical target position participates exactly once.
            if any(abs(target_progress - used) <= 1e-6
                   for used in used_progresses):
                continue
            used_progresses.append(target_progress)
            target = self._point_at_progress(target_progress)
            angle, bearing, distance = self._pursuit_steering_angle(
                origin, heading, target)
            if distance < 0.25:
                continue
            weighted_angle += float(weight) * angle
            weighted_bearing += float(weight) * bearing
            weighted_distance += float(weight) * distance
            weighted_progress += float(weight) * target_progress
            total_weight += float(weight)
            representative_target = target
        if total_weight <= 1e-12:
            return 0.0, 0.0, 0.0, float(progress), representative_target
        target_progress = weighted_progress / total_weight
        return (
            weighted_angle / total_weight,
            weighted_bearing / total_weight,
            weighted_distance / total_weight,
            target_progress,
            self._point_at_progress(target_progress),
        )

    # --- Construction / persistence ------------------------------------------
    def add_point(self, x: float, z: float, min_spacing: float = 10.0) -> bool:
        """Append a breadcrumb if it is at least ``min_spacing`` m from the last."""
        p = (float(x), float(z))
        if not self.points:
            self.points.append(p)
            self.world_points.append(p)
            self._tracking_state = None
            return True
        lx, lz = self.points[-1]
        if math.hypot(p[0] - lx, p[1] - lz) >= min_spacing:
            length = math.hypot(p[0] - lx, p[1] - lz)
            self.points.append(p)
            self.world_points.append(p)
            self._segment_lengths.append(length)
            self._cumulative_m.append(self._cumulative_m[-1] + length)
            self._tracking_state = None
            return True
        return False

    def save(self, path: str):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            json.dump({"name": self.name, "points": self.world_points}, f)

    @classmethod
    def load(cls, path: str) -> "Route":
        with open(path, "r") as f:
            data = json.load(f)
        return cls(points=data.get("points", []),
                   name=data.get("name", os.path.splitext(os.path.basename(path))[0]))

    def __len__(self) -> int:
        return len(self.points)

    # --- Geometry -------------------------------------------------------------
    def closest_index(self, pos: Point) -> int:
        """Index of the nearest waypoint to ``pos``."""
        if not self.points:
            return 0
        px, pz = pos
        best_i, best_d = 0, float("inf")
        for i, (x, z) in enumerate(self.points):
            d = (x - px) ** 2 + (z - pz) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _project_segment(self, index: int, pos: Point, heading: float):
        """Return ``(score, distance2, index, t, progress_m)`` for one edge."""
        px, pz = pos
        ax, az = self.points[index]
        bx, bz = self.points[index + 1]
        dx, dz = bx - ax, bz - az
        length2 = dx*dx + dz*dz
        if length2 < 1e-8:
            return None
        t = _clamp(((px-ax)*dx + (pz-az)*dz) / length2, 0.0, 1.0)
        qx, qz = ax + t*dx, az + t*dz
        distance2 = (px-qx)**2 + (pz-qz)**2
        length = math.sqrt(length2)
        fx, fz = -math.sin(heading), -math.cos(heading)
        alignment = (dx*fx + dz*fz) / length
        # Heading disagreement is deliberately expensive.  Opposite-facing
        # edges remain a fallback only if the local window contains no forward
        # edge (for example immediately after a telemetry teleport).
        score = distance2 + (1.0 - alignment) * 36.0
        progress = self._cumulative_m[index] + t*length
        return score, distance2, index, t, progress, alignment

    def _best_projection(self, indices, pos: Point, heading: float):
        best = None
        fallback = None
        for index in indices:
            candidate = self._project_segment(index, pos, heading)
            if candidate is None:
                continue
            if fallback is None or candidate[1] < fallback[1]:
                fallback = candidate
            if candidate[5] < -0.15:
                continue
            if best is None or candidate[0] < best[0]:
                best = candidate
        return best if best is not None else fallback

    def _authority_projection(self, authority, pos: Point, heading: float):
        """Project only onto points owned by one proven lane/deck identity."""
        if not self._point_authorities or authority is None:
            return None
        indices = self._authority_projection_segments.get(authority, ())
        if not indices:
            return None
        if self._tracking_state is not None:
            last_progress = float(self._tracking_state[4])
            local_indices = [index for index in indices
                             if last_progress - 8.0
                             <= self._cumulative_m[index]
                             <= last_progress + 40.0]
            # Never fall back to a global run after progress has been
            # acquired. On loops, roundabouts and vertically separated roads
            # a future occurrence of this LaneId may overlap the truck in X/Z
            # while being tens of metres later in the authoritative order.
            # Absence from this physically reachable window is therefore a
            # fail-closed mismatch, not permission to jump route progress.
            if not local_indices:
                return None
            indices = local_indices
        return self._best_projection(indices, pos, heading)

    def _authority_run(self, authority, progress: float):
        """Return the contiguous point run of ``authority`` nearest progress."""
        if not self._point_authorities or authority is None:
            return None
        runs = self._authority_runs.get(authority, ())
        if not runs:
            return None
        progress = float(progress)

        def distance_to_run(run):
            minimum = self._cumulative_m[run[0]]
            maximum = self._cumulative_m[run[1]]
            if minimum <= progress <= maximum:
                return 0.0
            return min(abs(minimum - progress), abs(maximum - progress))

        return min(runs, key=distance_to_run)

    def _authority_geometry_compatible(
            self, first, second, first_index: Optional[int] = None,
            second_index: Optional[int] = None) -> bool:
        """Whether adjacent validated runs may share local path derivatives.

        Point order in the validated lane trajectory proves the directed
        topological edge.  Equal elevation layers retain the original rule.
        Quantised layers may differ on a continuous grade, so derivatives may
        also cross one *immediate* sample when its real 3-D step proves
        continuity.  Projection and live LaneId authority remain exact.
        """
        try:
            first_lane, first_elevation = first
            second_lane, second_elevation = second
            first_direction = first_lane[1]
            second_direction = second_lane[1]
        except (TypeError, IndexError):
            return False
        if (first_lane is None or second_lane is None
                or first_elevation is None or second_elevation is None):
            return False
        same_direction_flag = first_direction == second_direction
        try:
            first_is_prefab = bool(first_lane[3])
            second_is_prefab = bool(second_lane[3])
        except (TypeError, IndexError):
            return False
        heterogeneous_boundary = first_is_prefab != second_is_prefab
        if not same_direction_flag and not heterogeneous_boundary:
            return False
        # Equal quantised elevation and equal direction are necessary labels,
        # not geometric proof.  Returning here used to accept any X/Y/Z gap
        # between different LaneIds on the same nominal layer.  Every identity
        # transition must also pass the immediate ordered-sample check below;
        # otherwise derivative/projection code could manufacture a chord.
        if (first_index is None or second_index is None
                or int(second_index) != int(first_index) + 1
                or not (0 <= int(first_index) < len(self.world_points))
                or not (0 <= int(second_index) < len(self.world_points))):
            return False
        first_point = self.world_points[int(first_index)]
        second_point = self.world_points[int(second_index)]
        try:
            if len(first_point) >= 3 and len(second_point) >= 3:
                dx = float(second_point[0]) - float(first_point[0])
                dy = float(second_point[1]) - float(first_point[1])
                dz = float(second_point[2]) - float(first_point[2])
            elif len(first_point) >= 2 and len(second_point) >= 2:
                # Legacy/recorded and deterministic test routes are X/Z.  They
                # still provide exact ground-plane gap proof; only an actual
                # 3-D LanePath may prove a vertical-layer transition.
                dx = float(second_point[0]) - float(first_point[0])
                dy = 0.0
                dz = float(second_point[1]) - float(first_point[1])
                if first_elevation != second_elevation:
                    return False
            else:
                return False
        except (TypeError, ValueError, OverflowError):
            return False
        spatially_continuous = bool(
            all(math.isfinite(value) for value in (dx, dy, dz))
            and math.sqrt(dx * dx + dy * dy + dz * dz)
                <= AUTHORITY_DERIVATIVE_MAX_SAMPLE_GAP_M
            and abs(dy) <= AUTHORITY_DERIVATIVE_MAX_VERTICAL_STEP_M)
        if not spatially_continuous:
            return False
        if same_direction_flag:
            return True

        # A differing road/prefab flag is not accepted on label semantics
        # alone.  Prove that the ordered trajectory actually crosses the
        # boundary in one forward direction.  This check is derivative-only:
        # it cannot create connectivity, change LaneId, or expand projection
        # authority.
        first_index = int(first_index)
        second_index = int(second_index)
        if first_index < 1 or second_index + 1 >= len(self.world_points):
            return False
        previous = self.points[first_index - 1]
        following = self.points[second_index + 1]
        vectors = (
            (float(self.points[first_index][0]) - float(previous[0]),
             float(self.points[first_index][1]) - float(previous[1])),
            (dx, dz),
            (float(following[0]) - float(self.points[second_index][0]),
             float(following[1]) - float(self.points[second_index][1])),
        )
        minimum_dot = math.cos(AUTHORITY_DERIVATIVE_MAX_HEADING_JUMP_RAD)
        for left, right in zip(vectors, vectors[1:]):
            left_length = math.hypot(*left)
            right_length = math.hypot(*right)
            if left_length < 0.5 or right_length < 0.5:
                return False
            alignment = ((left[0] * right[0] + left[1] * right[1])
                         / (left_length * right_length))
            if not math.isfinite(alignment) or alignment < minimum_dot:
                return False
        return True

    def _authority_geometry_bounds(self, authority, progress: float,
                                   reach_m: float):
        """Bounds for derivatives across proven adjacent lane transitions.

        Position projection remains scoped to the exact live LaneId. Only
        tangent and curvature may inspect immutable neighbouring samples, and
        only through immediate directed runs on the same deck and direction.
        This keeps road->prefab->road derivatives centred at the boundary.
        """
        run = self._authority_run(authority, progress)
        if run is None:
            return None
        reach = max(0.0, float(reach_m))
        desired_minimum = max(0.0, float(progress) - reach)
        desired_maximum = min(self._cumulative_m[-1], float(progress) + reach)
        first, last = run
        minimum = self._cumulative_m[first]
        maximum = self._cumulative_m[last]

        current = authority
        while minimum > desired_minimum and first > 0:
            adjacent = self._point_authorities[first - 1]
            if not self._authority_geometry_compatible(
                    adjacent, current, first - 1, first):
                break
            adjacent_run = self._authority_run(
                adjacent, self._cumulative_m[first - 1])
            if adjacent_run is None or adjacent_run[1] != first - 1:
                break
            first = adjacent_run[0]
            minimum = max(desired_minimum, self._cumulative_m[first])
            current = adjacent

        current = authority
        while maximum < desired_maximum and last + 1 < len(self.points):
            adjacent = self._point_authorities[last + 1]
            if not self._authority_geometry_compatible(
                    current, adjacent, last, last + 1):
                break
            adjacent_run = self._authority_run(
                adjacent, self._cumulative_m[last + 1])
            if adjacent_run is None or adjacent_run[0] != last + 1:
                break
            last = adjacent_run[1]
            maximum = min(desired_maximum, self._cumulative_m[last])
            current = adjacent
        # Callers ask for a local derivative window, not the complete current
        # LaneId run.  Returning an entire long run makes a spatial regression
        # depend on hundreds of metres of unrelated geometry.  The loops above
        # merely prove how far the requested window may cross adjacent runs.
        return (max(desired_minimum, minimum),
                min(desired_maximum, maximum))

    def _authority_bounds(self, authority, progress: float):
        """Return the contiguous progress interval for one lane/deck identity."""
        run = self._authority_run(authority, progress)
        if run is None:
            return None
        first, last = run
        minimum = self._cumulative_m[first]
        maximum = self._cumulative_m[last]
        return minimum, maximum

    def _authority_tangent(self, authority, progress: float):
        """Return a centred tangent on proven same-direction/same-deck runs."""
        bounds = self._authority_geometry_bounds(
            authority, progress, STEERING_REFERENCE_TANGENT_M)
        if bounds is None:
            return None
        minimum, maximum = bounds
        before_progress = max(minimum, progress - STEERING_REFERENCE_TANGENT_M)
        after_progress = min(maximum, progress + STEERING_REFERENCE_TANGENT_M)
        # Incompatible boundaries remain one-sided and fail-contained. A
        # proven compatible transition retains a centred derivative on both
        # sides and cannot inject a heading-feedback impulse.
        if after_progress - before_progress < 0.5:
            before_progress, after_progress = minimum, maximum
        if after_progress - before_progress < 0.5:
            return None
        return (self._point_at_progress(before_progress),
                self._point_at_progress(after_progress))

    def _tracking_projection(self, pos: Point, heading: float):
        if len(self.points) < 2:
            return (0, 0.0, 0.0, 0.0)

        px, pz = float(pos[0]), float(pos[1])
        heading = float(heading)
        state = self._tracking_state
        if state is not None:
            last_pos, last_heading, last_index, last_t, last_progress = state
            movement = math.hypot(px-last_pos[0], pz-last_pos[1])
            heading_delta = abs((heading-last_heading+math.pi) % (2*math.pi)-math.pi)
            # Map, steering, curvature and distance consumers query the same
            # route during one tick. Reuse the exact projection so those reads
            # cannot move route progress independently of the truck.
            if movement < 1e-4 and heading_delta < 1e-5:
                distance2 = self._project_segment(last_index, (px, pz), heading)[1]
                return last_index, last_t, last_progress, distance2
        else:
            movement = float("inf")

        segment_count = len(self.points) - 1
        reacquire = state is None or movement > 35.0
        if reacquire:
            candidate = self._best_projection(range(segment_count), (px, pz), heading)
        else:
            last_progress = state[4]
            # Normal telemetry may skip several frames, but it cannot move the
            # truck dozens of route metres without comparable world movement.
            # A small backward tolerance handles GPS noise while preventing a
            # crossing/roundabout arm from becoming the new target.
            min_progress = max(0.0, last_progress - 5.0)
            max_progress = min(self._cumulative_m[-1],
                               last_progress + max(18.0, movement*2.5 + 8.0))
            first = max(0, bisect.bisect_right(
                self._cumulative_m, min_progress) - 2)
            last = min(segment_count, bisect.bisect_left(
                self._cumulative_m, max_progress) + 1)
            candidate = self._best_projection(range(first, last),
                                              (px, pz), heading)
            # Lost map/telemetry position: permit a global reacquisition only
            # when the entire progress window is clearly nowhere near the
            # truck.  At a crossing the local arm is at zero distance and wins.
            if candidate is None or candidate[1] > 18.0**2:
                candidate = self._best_projection(range(segment_count),
                                                  (px, pz), heading)

        if candidate is None:
            return (0, 0.0, 0.0, float("inf"))
        _, distance2, index, t, progress, _ = candidate
        self._tracking_state = ((px, pz), heading, index, t, progress)
        return index, t, progress, distance2

    def tracking_index(self, pos: Point, heading: float) -> int:
        """Closest route segment that also agrees with the truck heading.

        A pure nearest-point lookup is ambiguous on roundabouts, crossings and
        parallel carriageways. It can jump to another arm and command a random
        left/right turn even though the truck is driving straight.
        """
        return self._tracking_projection(pos, heading)[0]

    def lookahead_point(self, idx: int, pos: Point, distance: float) -> Point:
        """Walk ``distance`` metres from the projection of ``pos`` on edge ``idx``."""
        if not self.points:
            return pos
        i = min(max(int(idx), 0), len(self.points)-1)
        if i >= len(self.points)-1:
            return self.points[-1]
        ax, az = self.points[i]
        bx, bz = self.points[i+1]
        dx, dz = bx-ax, bz-az
        length2 = dx*dx + dz*dz
        t = (0.0 if length2 < 1e-9 else
             _clamp(((pos[0]-ax)*dx + (pos[1]-az)*dz) / length2, 0.0, 1.0))
        qx, qz = ax+t*dx, az+t*dz
        remaining = max(0.0, float(distance))
        first_remaining = math.hypot(bx-qx, bz-qz)
        if remaining <= first_remaining and first_remaining > 1e-9:
            fraction = remaining / first_remaining
            return (qx+(bx-qx)*fraction, qz+(bz-qz)*fraction)
        remaining -= first_remaining
        i += 1
        while i < len(self.points) - 1:
            ax, az = self.points[i]
            bx, bz = self.points[i + 1]
            seg = math.hypot(bx - ax, bz - az)
            if seg >= remaining:
                t = remaining / seg if seg > 1e-6 else 1.0
                return (ax + (bx - ax) * t, az + (bz - az) * t)
            remaining -= seg
            i += 1
        return self.points[-1]

    def _point_at_progress(self, progress_m: float) -> Point:
        """Interpolate the immutable route at one arc-length position."""
        if not self.points:
            return (0.0, 0.0)
        if len(self.points) == 1:
            return self.points[0]
        progress = _clamp(float(progress_m), 0.0, self._cumulative_m[-1])
        index = min(len(self._segment_lengths) - 1, max(
            0, bisect.bisect_right(self._cumulative_m, progress) - 1))
        length = self._segment_lengths[index]
        fraction = (0.0 if length < 1e-9 else
                    (progress - self._cumulative_m[index]) / length)
        first, second = self.points[index], self.points[index + 1]
        return (first[0] + (second[0] - first[0]) * fraction,
                first[1] + (second[1] - first[1]) * fraction)

    def tracking_progress(self, pos: Point, heading: float) -> float:
        """Current progress shared by steering, speed and turn semantics."""
        return float(self._tracking_projection(pos, heading)[2])

    def _curvature_at_progress(
            self, progress_m: float,
            span_m: float = STEERING_CURVATURE_SPAN_M,
            progress_bounds: Optional[Tuple[float, float]] = None) -> float:
        """Signed Menger curvature around one route-progress sample."""
        if len(self.points) < 3 or self._cumulative_m[-1] < 2.0:
            return 0.0
        total = self._cumulative_m[-1]
        minimum, maximum = (0.0, total)
        if progress_bounds is not None:
            minimum = _clamp(float(progress_bounds[0]), 0.0, total)
            maximum = _clamp(float(progress_bounds[1]), minimum, total)
            if maximum - minimum < 2.0:
                return 0.0
        span = max(2.0, float(span_m))
        centre = _clamp(float(progress_m), minimum, maximum)
        before = max(minimum, centre - span)
        after = min(maximum, centre + span)
        # Near either trajectory end retain a full, one-sided sample rather
        # than collapsing two points and manufacturing infinite curvature.
        if centre - before < 1.0:
            before = centre
            centre = min(maximum, before + span)
            after = min(maximum, before + span * 2.0)
        elif after - centre < 1.0:
            after = centre
            centre = max(minimum, after - span)
            before = max(minimum, after - span * 2.0)
        first = self._point_at_progress(before)
        middle = self._point_at_progress(centre)
        last = self._point_at_progress(after)
        one = (middle[0] - first[0], middle[1] - first[1])
        two = (last[0] - middle[0], last[1] - middle[1])
        cross = one[0] * two[1] - one[1] * two[0]
        a, b, c = (math.dist(first, middle), math.dist(first, last),
                   math.dist(middle, last))
        product = a * b * c
        return 0.0 if product < 1e-6 else 2.0 * cross / product

    def _steering_curvature_at_progress(
            self, progress_m: float, authority=None) -> float:
        """Curvature represented across the tractor's spatial footprint."""
        # Two 2 m trajectory intervals represent the 3.8 m reference chassis.
        # Sampling at fractional chord lengths (3.8 m) introduces a periodic
        # curvature error even on an exact circle sampled every 2 m.
        span = 4.0
        reach = max(map(abs, STEERING_CURVATURE_SAMPLE_OFFSETS_M)) + span
        bounds = self._authority_geometry_bounds(
            authority, progress_m, reach)
        return sum(
            weight * (self._curvature_at_progress(progress_m + offset, span)
                      if bounds is None else self._curvature_at_progress(
                          progress_m + offset, span,
                          progress_bounds=bounds))
            for offset, weight in zip(
                STEERING_CURVATURE_SAMPLE_OFFSETS_M,
                STEERING_CURVATURE_SAMPLE_WEIGHTS))

    def curve_profile_ahead(self, pos: Point, heading: float,
                            horizon_m: float = CURV_WINDOW_M) -> dict:
        """Sharpest validated local curve in the forward driving horizon."""
        progress = self.tracking_progress(pos, heading)
        remaining = max(0.0, self._cumulative_m[-1] - progress)
        horizon = min(max(0.0, float(horizon_m)), remaining)
        offsets = [0.0]
        sample = CURVE_PROFILE_STEP_M
        while sample < horizon:
            offsets.append(sample)
            sample += CURVE_PROFILE_STEP_M
        if horizon > 0.0 and offsets[-1] != horizon:
            offsets.append(horizon)
        ranked = [(abs(curvature), offset, curvature)
                  for offset in offsets
                  for curvature in (self._curvature_at_progress(
                      progress + offset),)]
        magnitude, distance, signed = max(ranked, default=(0.0, 0.0, 0.0))
        radius = 1e6 if magnitude < 1e-9 else 1.0 / magnitude
        return {
            "radius_m": float(radius),
            "distance_m": float(distance),
            "signed_curvature": float(signed),
            "horizon_m": float(horizon),
        }

    def cross_track_error(self, idx: int, pos: Point) -> float:
        """Signed perpendicular distance from ``pos`` to the segment at ``idx``.

        Positive when the truck is to the *left* of the path direction.
        """
        if len(self.points) < 2:
            return 0.0
        j = min(idx, len(self.points) - 2)
        ax, az = self.points[j]
        bx, bz = self.points[j + 1]
        dx, dz = bx - ax, bz - az
        seg = math.hypot(dx, dz)
        if seg < 1e-6:
            return 0.0
        # 2D cross product of segment dir and (pos - a), normalised.
        return ((pos[0] - ax) * dz - (pos[1] - az) * dx) / seg

    def _trailer_envelope_offset(self, progress: float, pos: Point,
                                 heading: float, speed_ms: float,
                                 tractor_cte: float,
                                 envelope: Optional[dict],
                                 preview_curvature_per_m: float = 0.0,
                                 active_authority=None,
                                 ) -> Tuple[float, dict]:
        """Return a proven outward tractor offset for an attached trailer.

        The immutable lane remains the reference.  We independently project
        the live trailer pose onto a small, directed window *behind* the
        tractor only to prove physical coupling and the correct deck.  Offset
        side and magnitude come solely from the Ackermann swept path of the
        upcoming vehicle-length of immutable geometry.  Invalid, opposite,
        other-deck or ambiguous poses are fail-neutral and do not create an
        offset; measured trailer CTE is diagnostic and has no steering
        authority.
        """
        debug = {
            "accepted": False, "reason": "trailer is not attached",
            "trailer_cte_m": 0.0, "measured_offtrack_m": 0.0,
            "predicted_offtrack_m": 0.0, "required_offset_m": 0.0,
            "available_offset_m": 0.0, "applied_offset_m": 0.0,
            "trailer_progress_m": 0.0,
            "curve_side_proven": False,
            "preview_curvature_per_m": float(
                preview_curvature_per_m),
            "curvature_source": "local_frenet_path",
            "balanced_reference_fraction": float(
                TRAILER_BALANCED_REFERENCE_FRACTION),
        }
        if not isinstance(envelope, dict) or not envelope.get("attached", False):
            return 0.0, debug
        try:
            trailer_pos = tuple(map(float, envelope["position"][:2]))
            trailer_heading = float(envelope["heading"])
            lane_width = float(envelope["lane_width_m"])
            tractor_altitude = float(envelope["tractor_altitude_m"])
            trailer_altitude = float(envelope["trailer_altitude_m"])
            values = (*trailer_pos, trailer_heading, lane_width,
                      tractor_altitude, trailer_altitude, float(tractor_cte),
                      float(preview_curvature_per_m))
            if not all(math.isfinite(value) for value in values):
                raise ValueError("non-finite trailer envelope metadata")
        except (KeyError, TypeError, ValueError, IndexError, OverflowError):
            debug["reason"] = "trailer envelope metadata is malformed"
            return 0.0, debug
        if not 2.4 <= lane_width <= 12.0:
            debug["reason"] = "confirmed lane width is unavailable"
            return 0.0, debug
        if abs(trailer_altitude - tractor_altitude) > TRAILER_VERTICAL_TOLERANCE_M:
            debug["reason"] = "trailer is on a different elevation layer"
            return 0.0, debug
        pose_distance = math.dist(pos, trailer_pos)
        if not TRAILER_POSE_MIN_DISTANCE_M <= pose_distance <= TRAILER_POSE_MAX_DISTANCE_M:
            debug["reason"] = "trailer pose is not physically coupled to the tractor"
            return 0.0, debug
        articulation = abs((heading - trailer_heading + math.pi)
                           % (2.0 * math.pi) - math.pi)
        if articulation > math.radians(70.0):
            debug["reason"] = "trailer direction is incompatible with the tractor"
            return 0.0, debug

        segment_count = len(self.points) - 1
        minimum = max(0.0, progress - TRAILER_PROGRESS_BEHIND_MAX_M)
        maximum = min(self._cumulative_m[-1], progress + 2.0)
        first = max(0, bisect.bisect_right(
            self._cumulative_m, minimum) - 2)
        last = min(segment_count, bisect.bisect_left(
            self._cumulative_m, maximum) + 1)
        candidate = self._best_projection(
            range(first, last), trailer_pos, trailer_heading)
        if candidate is None:
            debug["reason"] = "trailer has no directed projection on the active lane"
            return 0.0, debug
        _, distance2, index, _fraction, trailer_progress, alignment = candidate
        if (alignment < 0.15 or trailer_progress > progress + 2.0
                or progress - trailer_progress > TRAILER_PROGRESS_BEHIND_MAX_M
                or distance2 > max(8.0, lane_width * 1.5) ** 2):
            debug["reason"] = "trailer projection is outside the active directed lane"
            return 0.0, debug

        trailer_cte = self.cross_track_error(index, trailer_pos)
        measured = trailer_cte - tractor_cte
        # Trailer placement and actuator scheduling must not own competing
        # estimates of the same bend. ``preview_curvature_per_m`` is the one
        # canonical local path curvature inside the same
        # validated authority bounds. The old trailer-only 12--24 m estimator
        # changed side/magnitude at different map positions and moved the
        # virtual tractor origin while the steering target itself was stable.
        swept_curvature = float(preview_curvature_per_m)
        dominant_sign = (0 if abs(swept_curvature) < 1.0 / 1000.0
                         else (1 if swept_curvature > 0.0 else -1))
        curve_side_proven = bool(dominant_sign)
        predicted = 0.0
        if curve_side_proven:
            radius = 1.0 / abs(swept_curvature)
            magnitude = (math.sqrt(
                radius * radius + TRAILER_EFFECTIVE_AXLE_DISTANCE_M ** 2)
                         - radius)
            predicted = -math.copysign(magnitude, swept_curvature)

        # Immutable curvature owns both side and magnitude.  The live trailer
        # projection above validates physical coupling and remains diagnostic,
        # but it is never a fast steering input.  The calibrated fraction is
        # the steady min-max reference plus its spatial entry reserve described
        # above; the confirmed lane envelope still caps the tractor target.
        required = predicted * TRAILER_BALANCED_REFERENCE_FRACTION
        measurement_conflict = bool(
            abs(measured) > 0.10 and predicted * measured < 0.0)
        available = max(
            0.0, (lane_width - TRACTOR_BODY_WIDTH_M) * 0.5
            - VEHICLE_ENVELOPE_MARGIN_M)
        applied = _clamp(required, -available, available)
        debug.update({
            "accepted": True,
            "reason": (
                "accepted; measured side conflicts but is diagnostic only"
                if measurement_conflict else
                ("accepted" if abs(required) <= available + 1e-6
                 else "accepted but constrained by confirmed lane width")),
            "trailer_cte_m": float(trailer_cte),
            "measured_offtrack_m": float(measured),
            "predicted_offtrack_m": float(predicted),
            "required_offset_m": float(required),
            "available_offset_m": float(available),
            "applied_offset_m": float(applied),
            "trailer_progress_m": float(trailer_progress),
            "curve_side_proven": bool(curve_side_proven),
        })
        return float(applied), debug

    def distance_to_end(self, pos: Point, heading: float = None) -> float:
        """Path-length distance from ``pos`` (snapped to nearest waypoint) to the end."""
        if not self.points:
            return 0.0
        # Heading-aware matching avoids selecting the wrong arm of a
        # roundabout. Recorded routes without a heading retain nearest-point
        # behaviour. The old code referenced an undefined ``heading`` variable
        # here and crashed the whole map plugin on every calculation.
        if len(self.points) == 1:
            return math.dist(pos, self.points[0])
        if heading is not None:
            idx = self.tracking_index(pos, heading)
        else:
            # Find the nearest segment, not merely the nearest waypoint.
            def segment_distance2(i):
                ax, az = self.points[i]
                bx, bz = self.points[i + 1]
                dx, dz = bx - ax, bz - az
                length2 = dx*dx + dz*dz
                t = (0.0 if length2 < 1e-9 else
                     _clamp(((pos[0]-ax)*dx + (pos[1]-az)*dz) / length2, 0.0, 1.0))
                return (pos[0] - (ax+t*dx))**2 + (pos[1] - (az+t*dz))**2
            idx = min(range(len(self.points) - 1), key=segment_distance2)
        ax, az = self.points[idx]
        bx, bz = self.points[idx + 1]
        dx, dz = bx - ax, bz - az
        length2 = dx*dx + dz*dz
        t = (0.0 if length2 < 1e-9 else
             _clamp(((pos[0]-ax)*dx + (pos[1]-az)*dz) / length2, 0.0, 1.0))
        total = math.hypot(dx, dz) * (1.0 - t)
        for i in range(idx + 1, len(self.points) - 1):
            ax, az = self.points[i]
            bx, bz = self.points[i + 1]
            total += math.hypot(bx - ax, bz - az)
        return total

    def is_finished(self, pos: Point) -> bool:
        if not self.points:
            return True
        ex, ez = self.points[-1]
        near_end = math.hypot(ex - pos[0], ez - pos[1]) < ARRIVAL_RADIUS
        # Also require being close to the final segment (not just the last point's circle).
        return near_end and self.closest_index(pos) >= len(self.points) - 2

    # --- Steering -------------------------------------------------------------
    def signed_curvature_ahead(self, pos: Point, heading: float,
                               window_m: float = CURV_WINDOW_M) -> float:
        """Return signed path curvature; positive follows positive steering."""
        if len(self.points) < 3:
            return 0.0
        idx, projection_t, _progress, _distance2 = self._tracking_projection(
            pos, heading)
        ax0, az0 = self.points[idx]
        bx0, bz0 = self.points[min(idx + 1, len(self.points) - 1)]
        p0 = (ax0 + (bx0-ax0) * projection_t,
              az0 + (bz0-az0) * projection_t)
        p1 = self.lookahead_point(idx, pos, window_m * 0.5)
        p2 = self.lookahead_point(idx, pos, window_m)
        first = (p1[0] - p0[0], p1[1] - p0[1])
        second = (p2[0] - p1[0], p2[1] - p1[1])
        cross = first[0] * second[1] - first[1] * second[0]
        a = math.dist(p0, p1)
        b = math.dist(p0, p2)
        c = math.dist(p1, p2)
        if a < 1e-3 or b < 1e-3 or c < 1e-3 or abs(cross) < 1e-6:
            return 0.0
        product = a * b * c
        if product < 1e-6:
            return 0.0
        return 2.0 * cross / product

    def curvature_ahead(self, pos: Point, heading: float,
                        window_m: float = CURV_WINDOW_M) -> float:
        """Radius (m) of the sharpest bend in the next ``window_m`` of path.

        Returns a large number (≈straight) when the road is straight or there
        isn't enough path. Used two ways: (1) to shrink the steering lookahead
        into tight curves so the truck tracks the apex instead of cutting it,
        and (2) by the autopilot to brake *before* a sharp bend rather than
        mid-corner. The estimate is the discrete Menger curvature (circle
        through three points: the truck, a near point, a far point)."""
        return float(self.curve_profile_ahead(
            pos, heading, window_m)["radius_m"])

    def steering(self, pos: Point, heading: float, speed_ms: float = 0.0,
                 lane_offset_m: float = 0.0,
                 cross_track_error_m: Optional[float] = None,
                 vehicle_envelope: Optional[dict] = None,
                 control_authority: Optional[dict] = None,
                 control_dt_s: Optional[float] = None,
                 vehicle_curvature_per_m: Optional[float] = None,
                 actuator_response_s: float = ACTUATION_PREVIEW_S,
                 steering_lock_rad: float = REFERENCE_LOCK_RAD) -> float:
        """Steering command in ``[-1, 1]`` (positive = right) to follow the route.

        GPS callers pass lane_offset_m=0: LanePath already is the confirmed
        lane, never an unsnapped road centreline. A positive legacy offset
        targets the right side. Trailer envelope offsets are separate virtual
        tractor references; the authoritative map points remain immutable.
        """
        self.last_steering_debug = {
            "feed_forward": 0.0, "feedback": 0.0,
            "local_curvature": 0.0, "raw": 0.0, "output": 0.0,
            "curve_direction_hold": False,
            "curve_direction_hold_eligible": False,
            "curve_direction_hold_approach": False,
            "curve_direction_hold_fraction": 0.0,
            "curve_sign_projection_active": False,
            "curve_direction_hold_error_proven": False,
            "straight_recovery_active": False,
            "low_speed_capture_active": False,
            "cte_steer": 0.0,
            "cte_gain": 0.0,
            "lane_recovery_multiplier": 1.0,
            "guidance_lookahead_m": 0.0,
            "guidance_target_distance_m": 0.0,
            "guidance_heading_error_rad": 0.0,
            "guidance_curvature": 0.0,
            "trailer_envelope": {},
            "authority_valid": True,
            "authority_lane_id": None,
            "authority_revision": None,
        }
        if len(self.points) < 2:
            self.last_steering_debug.update(
                authority_valid=False,
                control_failure="route has fewer than two points")
            return 0.0

        try:
            inputs = (pos[0], pos[1], heading, speed_ms, lane_offset_m,
                      actuator_response_s)
            if cross_track_error_m is not None:
                inputs += (cross_track_error_m,)
            if not all(math.isfinite(float(value)) for value in inputs):
                raise ValueError("non-finite pose/control input")
        except (TypeError, ValueError, IndexError, OverflowError):
            self.last_steering_debug.update(
                authority_valid=False, control_failure="invalid pose/control input")
            return 0.0
        try:
            steering_lock_rad = float(steering_lock_rad)
            calibration_valid = bool(
                math.isfinite(steering_lock_rad)
                and MIN_STEERING_LOCK_RAD <= steering_lock_rad
                    <= MAX_STEERING_LOCK_RAD)
        except (TypeError, ValueError, OverflowError):
            steering_lock_rad = float("nan")
            calibration_valid = False
        if not calibration_valid:
            self.last_steering_debug.update(
                authority_valid=False,
                control_failure="invalid actuator calibration",
                steering_lock_rad=float(steering_lock_rad),
                steering_lock_valid=False)
            return 0.0

        authority_lane = None
        authority_revision = None
        authority_elevation = None
        lane_width_m = 4.0
        if isinstance(control_authority, dict):
            authority_lane = control_authority.get("lane_identity")
            authority_revision = control_authority.get("revision")
            authority_elevation = control_authority.get("elevation_layer")
            try:
                lane_width_m = float(control_authority.get(
                    "lane_width_m", lane_width_m))
            except (TypeError, ValueError, OverflowError):
                lane_width_m = 4.0
        route_authority = ((authority_lane, authority_elevation)
                           if authority_lane is not None else None)
        # A plain nearest-waypoint lookup is ambiguous on divided motorways,
        # roundabouts and junctions.  Use the heading-aware segment selected by
        # the same geometry used for localisation, otherwise steering can jump
        # onto a neighbouring arm and immediately pull across the median.
        authority_projection = self._authority_projection(
            route_authority, pos, heading)
        if self._point_authorities and route_authority is not None:
            projection_distance_m = (
                float("inf") if authority_projection is None
                else math.sqrt(max(0.0, float(authority_projection[1]))))
            if (authority_projection is None
                    or projection_distance_m
                    > AUTHORITY_PROJECTION_MAX_DISTANCE_M):
                self.last_steering_debug.update({
                    "authority_valid": False,
                    "control_failure": (
                        "live LaneId has no reachable projection on the "
                        "validated trajectory"),
                    "authority_lane_id": authority_lane,
                    "authority_revision": authority_revision,
                    "authority_projection_distance_m": float(
                        projection_distance_m),
                })
                return 0.0
            _score, _distance2, idx, _fraction, progress, _alignment = (
                authority_projection)
            self._tracking_state = (
                (float(pos[0]), float(pos[1])), float(heading),
                idx, _fraction, progress)
        else:
            projection_distance_m = None
            idx, _fraction, progress, _distance2 = self._tracking_projection(
                pos, heading)

        # This short tangent selects and verifies the forward route segment; it
        # is not a second steering target. The geometric guidance target below
        # is interpolated by arc-length on the same confirmed trajectory.
        projection = self._point_at_progress(progress)
        tangent_window = _clamp(3.0 + abs(speed_ms) * 0.15, 3.0, 6.0)
        authority_tangent = self._authority_tangent(
            route_authority, progress)
        if authority_tangent is not None:
            tangent_first, tangent_target = authority_tangent
            path_dx = tangent_target[0] - tangent_first[0]
            path_dz = tangent_target[1] - tangent_first[1]
        else:
            # A forward-only secant is not the lane tangent on a curve: its
            # heading is already rotated by roughly half the sampled arc.  It
            # therefore cancelled about half of pure-pursuit's curve bearing
            # and made the truck drive straight into a bend before applying a
            # late, sharp correction.  Use the same centred spatial derivative
            # as authority-scoped routes even when a legacy/recorded path has
            # no point-authority metadata.
            tangent_before = self._point_at_progress(
                progress - tangent_window)
            tangent_target = self._point_at_progress(
                progress + tangent_window)
            path_dx = tangent_target[0] - tangent_before[0]
            path_dz = tangent_target[1] - tangent_before[1]
        path_length = math.hypot(path_dx, path_dz)
        if path_length < 0.5:
            self.last_steering_debug.update(
                authority_valid=False,
                control_failure="validated local lane tangent is unavailable")
            return 0.0
        path_heading = math.atan2(-path_dx, -path_dz)
        heading_error = ((heading - path_heading + math.pi)
                         % (2.0 * math.pi) - math.pi)
        fx, fz = -math.sin(heading), -math.cos(heading)
        alignment = (fx * path_dx + fz * path_dz) / path_length
        if alignment <= 0.10 or abs(heading_error) > math.radians(82.0):
            self.last_steering_debug.update(
                authority_valid=False,
                control_failure="vehicle heading is incompatible with local lane tangent")
            return 0.0

        # Cross-track error, measured to the lane-offset line so it pulls us
        # into our lane, not the centre. CLAMPED to ±5 m: when the truck is far
        # from the road (e.g. a wrong map dataset is loaded, or we're on a ferry
        # / car park) the raw CTE can be 30+ m. Capping it prevents an invalid
        # position from turning the geometric intercept into immediate full
        # lock; the runtime authority gate still handles the fail-closed stop.
        has_confirmed_lane_error = cross_track_error_m is not None
        geometric_cte = self.cross_track_error(idx, pos) + lane_offset_m
        cte = (geometric_cte if cross_track_error_m is None
               else float(cross_track_error_m) + lane_offset_m)
        geometric_cte = max(-5.0, min(5.0, geometric_cte))
        cte = max(-5.0, min(5.0, cte))
        cte_geometry_residual = abs(cte - geometric_cte)
        cte_error_geometrically_proven = bool(
            not has_confirmed_lane_error
            or cte_geometry_residual <= CURVE_DIRECTION_HOLD_CTE_AGREEMENT_M)

        # One local Frenet frame for CTE, heading and curvature. Preview moves
        # only the feed-forward to where it will reach the tyres; it never
        # substitutes a distant target bearing for the current heading error.
        v = abs(float(speed_ms))
        local_curvature = self._steering_curvature_at_progress(
            progress, authority=route_authority)
        preview_distance = v * actuator_response_s
        preview_bounds = self._authority_geometry_bounds(
            route_authority, progress, preview_distance + 12.0)
        target_progress = min(self._cumulative_m[-1],
                              progress + preview_distance)
        if preview_bounds is not None:
            target_progress = min(target_progress, preview_bounds[1])
        preview_curvature = self._steering_curvature_at_progress(
            target_progress, authority=route_authority)
        trailer_offset, trailer_debug = self._trailer_envelope_offset(
            progress, pos, heading, v, cte - lane_offset_m, vehicle_envelope,
            preview_curvature_per_m=local_curvature,
            active_authority=route_authority)
        target_heading = 0.0
        control_curvature = local_curvature
        control_preview_curvature = preview_curvature
        if trailer_debug.get("accepted", False):
            # All three differential terms belong to the SAME immutable
            # spatial trailer reference. Never feed measured trailer CTE into
            # the cab steering or leave a stored offset after a curve/revision.
            available = trailer_debug["available_offset_m"]

            def target_frame(at_progress):
                span = TRUCK_WHEELBASE_M
                ks = [self._steering_curvature_at_progress(
                    at_progress + delta, authority=route_authority)
                    for delta in (-span, 0.0, span)]
                targets = []
                for k in ks:
                    magnitude = (0.0 if abs(k) < 1e-12 else
                        TRAILER_EFFECTIVE_AXLE_DISTANCE_M ** 2 * abs(k)
                        / (math.sqrt(1.0 + (
                            TRAILER_EFFECTIVE_AXLE_DISTANCE_M * k) ** 2) + 1.0))
                    targets.append(_clamp(math.copysign(
                        magnitude * TRAILER_BALANCED_REFERENCE_FRACTION, k),
                        -available, available))
                return offset_frame(
                    ks[1], (ks[2]-ks[0])/(2*span), targets[1],
                    (targets[2]-targets[0])/(2*span),
                    (targets[2]-2*targets[1]+targets[0])/(span*span))

            target_heading, control_curvature = target_frame(progress)
            _, control_preview_curvature = target_frame(target_progress)
        steer, control = solve_lateral(
            control_curvature, control_preview_curvature,
            (cte + trailer_offset) * math.cos(target_heading),
            heading_error - target_heading, v,
            vehicle_curvature_per_m=vehicle_curvature_per_m,
            response_s=actuator_response_s, steering_lock_rad=steering_lock_rad)
        if not control["valid"]:
            self.last_steering_debug.update(
                authority_valid=False, control_failure=control["reason"])
            return 0.0
        # Output execution and its physical bounds belong to SteeringDynamics.
        # Only the normalized API range is applied here. No curve hold, CTE
        # threshold, standstill clamp or parallel recovery controller.
        raw_steer = steer
        steer = _clamp(raw_steer, -1.0, 1.0)
        feed_forward = control["feed_forward"]
        heading_feedback = control["heading_feedback"]
        cte_feedback = control["cte_feedback"]
        feedback = heading_feedback + cte_feedback
        control_cte = cte + trailer_offset
        trailer_debug["curvature_source"] = "local_frenet_path"
        self.last_steering_debug = {
            **control,
            "controller": "frenet_bicycle",
            "curve_sign_projection_active": False,
            "trailer_target_heading_rad": target_heading,
            "trailer_target_m": -trailer_offset,
            "target_curvature_per_m": control_curvature,
            "feed_forward": feed_forward, "feedback": feedback,
            "feedback_applied": feedback,
            "heading_feedback": heading_feedback,
            "cte_feedback": cte_feedback,
            "curve_reference": feed_forward,
            "guidance_delta": feedback,
            "guidance_applied": feedback,
            "feedback_angle_rad": feedback * steering_lock_rad,
            "local_curvature": local_curvature,
            "preview_curvature": preview_curvature,
            "raw": raw_steer, "output": steer,
            "curve_direction_hold": False,
            "curve_direction_hold_eligible": False,
            "curve_direction_hold_approach": False,
            "curve_direction_hold_fraction": 0.0,
            "curve_direction_hold_error_proven": cte_error_geometrically_proven,
            "geometric_cte": geometric_cte, "control_cte": control_cte,
            "cte_geometry_residual": cte_geometry_residual,
            "straight_recovery_active": False,
            "low_speed_capture_active": v < LOW_SPEED_CAPTURE_MAX_MS,
            "cte_steer": control["cte_feedback_angle_rad"],
            "cte_gain": 1.0 / control["feedback_length_m"] ** 2,
            "lane_recovery_multiplier": 1.0,
            "guidance_lookahead_m": target_progress - progress,
            "guidance_target_distance_m": target_progress - progress,
            "guidance_heading_error_rad": heading_error,
            "guidance_curvature": control["curvature_demand_per_m"],
            "pursuit_target_progress_m": target_progress,
            "pursuit_target_xz": self._point_at_progress(target_progress),
            "steering_limit": 1.0,
            "steering_lock_rad": steering_lock_rad,
            "steering_lock_valid": True,
            "saturated": abs(raw_steer) > 1.0,
            "trailer_envelope": trailer_debug,
            "authority_valid": True, "authority_lane_id": authority_lane,
            "authority_revision": authority_revision,
            "authority_projection_distance_m": projection_distance_m,
            "tracking_progress_m": progress,
            "tracking_segment_index": idx,
            "tracking_segment_fraction": _fraction,
            "tracking_projection_xz": projection,
            "local_tangent_heading_rad": path_heading,
        }
        return steer
