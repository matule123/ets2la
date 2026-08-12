"""
Coordinate-based route navigation for UltraPilot.

A :class:`Route` is a polyline of world ``(x, z)`` waypoints captured from SCS
telemetry. Given the truck's current world pose it produces a steering value in
``[-1, 1]`` from local lane curvature, local-tangent heading error and
**cross-track error** (perpendicular distance to the path).

This drives the truck along a previously-recorded path with no game-map data or
vision — purely from world coordinates.  Sign convention: positive steering =
steer right; the world uses ETS2's heading where ``forward = (-sin h, -cos h)``.
"""

import bisect
import json
import math
import os
from typing import List, Optional, Sequence, Tuple

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

# Stanley calibration helpers remain public for compatibility and isolated
# calibration tests. Authoritative GPS steering evaluates them in the same
# Frenet frame as the Ackermann curvature below.
K_HEADING = 2.60          # yaw damping in the common Frenet steering frame
# Closed-loop 20 Hz bicycle calibration.  The former 1.80 bend-lateral gain
# was tuned against open-loop samples and duplicated too much of the heading
# correction once the game's 320 ms wheel response was included.  A measured
# +/-0.45 m LaneMatch disturbance could therefore command a new half-cycle
# before the previous wheel request reached the road.  The broad-road gain is
# retained while bend CTE authority is reduced and geometric yaw damping is
# strengthened.  No temporal filter or hidden state is involved.
K_CTE = 1.05              # broad-road/straight lateral correction [1/s]
K_CTE_CURVE = 1.70        # proven-bend lateral correction [1/s]
K_SOFT = 1.0              # speed softening [m/s] keeps CTE finite at v=0
# Below walking speed, expand the same target so engagement follows one
# shallow intercept rather than demanding a steep correction at standstill.
LOW_SPEED_CAPTURE_MAX_MS = 5.0
# Coherent tractor bicycle model.  The former 5.0 m / 0.18 rad pair had a
# minimum possible turning radius of 27.3 m, yet the validated ProMods trace
# contains an 18 m roundabout/prefab lane.  It therefore saturated by design,
# ran off the lane and corrected only after localisation had degraded.  A
# typical tractor axle spacing and a deliberately conservative usable tyre
# angle cover a 13.3 m radius without inventing geometry or raising any gate.
TRUCK_WHEELBASE_M = 3.8
NORMALIZED_STEERING_ANGLE_RAD = 0.28
STEERING_CURVATURE_SPAN_M = 8.0
# The lane trajectory is resampled near two metres and therefore contains a
# small deterministic curvature quantisation.  Steering represents the
# tractor wheelbase footprint with a symmetric spatial quadrature around the
# reference progress.  This is a calculation over immutable road geometry,
# not a time-domain filter of commands or telemetry.
STEERING_CURVATURE_SAMPLE_OFFSETS_M = (-4.0, -2.0, 0.0, 2.0, 4.0)
STEERING_CURVATURE_SAMPLE_WEIGHTS = (0.10, 0.20, 0.40, 0.20, 0.10)
# These legacy lookahead constants remain public for recorded-route callers.
# Authoritative lane steering uses the short spatial Frenet reference below;
# low-speed capture still uses the 20 m value as its geometric correction cap.
GUIDANCE_LOOKAHEAD_MIN_M = 8.0
GUIDANCE_LOOKAHEAD_MAX_M = 30.0
GUIDANCE_LOW_SPEED_LOOKAHEAD_M = 20.0
# State feedback is converted to a real steering angle before it is combined
# with the Ackermann feed-forward angle.  This is the calibrated usable SCS
# response of the tractor, not a temporal signal filter.
FEEDBACK_STEERING_RESPONSE = 0.40
STEERING_REFERENCE_PREVIEW_MIN_M = 0.5
STEERING_REFERENCE_PREVIEW_MAX_M = 4.0
# Estimate the Frenet tangent across a physical tractor-length.  A 1.5 m
# secant was shorter than the map's two-metre resampling interval and turned
# harmless lane-point quantisation into alternating heading error.
STEERING_REFERENCE_TANGENT_M = 6.0
# A live LaneId is already accepted by LaneLocator with its 2.4 m lateral
# gate. Route must not project that identity onto a more distant future run
# merely because it occurs later in the same trajectory.
AUTHORITY_PROJECTION_MAX_DISTANCE_M = 2.4
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
TRAILER_CURVATURE_PROOF_FRACTION = 0.70
# On a constant-radius bend the trailer axle cuts inward by
# sqrt(R^2 + Ltr^2) - R.  Half is the steady-state min-max solution.  The
# deterministic articulated 20 Hz entry replay needs another 10% of that same
# immutable spatial prediction while the trailer articulation is still
# building, after which the confirmed lane-envelope cap remains authoritative.
# Live trailer CTE never changes this fraction or its side.
TRAILER_BALANCED_REFERENCE_FRACTION = 0.60
# A longer window is retained for anticipatory curve braking; steering uses
# the shorter local window above so it cannot cut across a bend.
CURV_WINDOW_M = 60.0
CURVE_PROFILE_STEP_M = 4.0
TIGHT_CURVE_RADIUS = 60.0
# These values now drive audit fields only. They report whether LaneLocator and
# Route projection agree and whether a tight curve is approaching; neither
# replaces or clamps the geometric steering command.
CURVE_DIRECTION_HOLD_CTE_AGREEMENT_M = 0.45
CURVE_DIRECTION_HOLD_APPROACH_RADIUS_M = 30.0
CURVE_DIRECTION_HOLD_APPROACH_DISTANCE_M = 35.0
ARRIVAL_RADIUS = 12.0     # metres from the last point counts as "arrived"


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def speed_gain(speed_ms: float) -> float:
    """Gentler steering at speed, sharper when crawling (like ETS2LA's schedule)."""
    speed_kmh = abs(speed_ms) * 3.6
    # 1.3 at standstill → ~0.5 at 90 km/h, floored.
    return _clamp(1.3 - (speed_kmh / 90.0) * 0.8, 0.45, 1.3)


def curve_cte_gain(radius_m: float, lateral_error_m: float = 0.0) -> float:
    """Strengthen cross-track recovery only inside a proven map bend.

    A lookahead point lies on a chord of the lane curve. At road speed the old
    fixed gain was too weak to cancel that inward bias, so the truck could
    settle near the centre divider although the trajectory was lane-centred.
    Straights retain the calm gain to avoid right/left hunting.
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
        self._authority_runs = {}
        if self._point_authorities:
            for index in range(len(self.points) - 1):
                first = self._point_authorities[index]
                second = self._point_authorities[index + 1]
                if first == second:
                    self._authority_segments.setdefault(first, []).append(index)
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

        All three inputs use the same normalized road-wheel unit.  In a
        confirmed bend the signed curvature is immutable spatial authority:
        feedback may continuously reduce the angle to zero, but cannot turn
        the tractor against that bend.  Following the tangent at zero steering
        already crosses back toward the outside of a curve; an opposite wheel
        command is what caused the 22:59 positive-feedback excursion.

        ``SteeringDynamics`` is deliberately the only temporal/rate-limiting
        stage.  This pure function has no proof timers, hysteresis, filtering
        or hidden correction authority.
        """
        feedback = float(heading_feedback) + float(cte_feedback)
        candidate = float(feed_forward) + feedback
        curve_sign = (1 if feed_forward > 0.0
                      else -1 if feed_forward < 0.0 else 0)
        sign_projected = bool(curve_sign and candidate * curve_sign < 0.0)
        if sign_projected:
            candidate = 0.0
        applied_feedback = candidate - float(feed_forward)
        return float(candidate), {
            "curve_foundation_active": bool(curve_sign),
            "curve_foundation_sign": int(curve_sign),
            "curve_sign_projection_active": bool(sign_projected),
            "raw_feedback": float(feedback),
            "feedback_applied": float(applied_feedback),
        }

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
        indices = self._authority_segments.get(authority, ())
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

    @staticmethod
    def _authority_geometry_compatible(first, second) -> bool:
        """Whether adjacent validated runs may share local path derivatives.

        Point order in the validated lane trajectory proves the directed
        topological edge. A derivative may cross that exact edge only when
        travel direction and elevation also agree; no point or connection is
        synthesized.
        """
        try:
            first_lane, first_elevation = first
            second_lane, second_elevation = second
            first_direction = first_lane[1]
            second_direction = second_lane[1]
        except (TypeError, IndexError):
            return False
        return bool(
            first_lane is not None and second_lane is not None
            and first_direction == second_direction
            and first_elevation is not None
            and first_elevation == second_elevation)

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
            if not self._authority_geometry_compatible(adjacent, current):
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
            if not self._authority_geometry_compatible(current, adjacent):
                break
            adjacent_run = self._authority_run(
                adjacent, self._cumulative_m[last + 1])
            if adjacent_run is None or adjacent_run[0] != last + 1:
                break
            last = adjacent_run[1]
            maximum = min(desired_maximum, self._cumulative_m[last])
            current = adjacent
        return minimum, maximum

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
        reach = (max(map(abs, STEERING_CURVATURE_SAMPLE_OFFSETS_M))
                 + STEERING_CURVATURE_SPAN_M)
        bounds = self._authority_geometry_bounds(
            authority, progress_m, reach)
        return sum(
            weight * (self._curvature_at_progress(progress_m + offset)
                      if bounds is None else self._curvature_at_progress(
                          progress_m + offset, progress_bounds=bounds))
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
                      tractor_altitude, trailer_altitude, float(tractor_cte))
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
        # Integrate curvature over one moving vehicle-length ahead.  This is a
        # spatial swept-path calculation, not a time-domain steering filter.
        horizon = _clamp(12.0 + abs(speed_ms), 12.0, 24.0)
        offsets = (0.0, horizon * 0.25, horizon * 0.50,
                   horizon * 0.75, horizon)
        weights = (0.10, 0.20, 0.30, 0.25, 0.15)
        active_bounds = self._authority_geometry_bounds(
            active_authority, progress,
            horizon + STEERING_CURVATURE_SPAN_M)
        curvature_samples = [
            (self._curvature_at_progress(progress + offset)
             if active_bounds is None else self._curvature_at_progress(
                 progress + offset, progress_bounds=active_bounds))
            for offset in offsets]
        swept_curvature = sum(weight * curvature for weight, curvature in zip(
            weights, curvature_samples))
        dominant_sign = (0 if abs(swept_curvature) < 1.0 / 1000.0
                         else (1 if swept_curvature > 0.0 else -1))
        coherent_weight = sum(
            weight for weight, curvature in zip(weights, curvature_samples)
            if dominant_sign and curvature * dominant_sign > 1.0 / 1500.0)
        curve_side_proven = bool(
            dominant_sign
            and coherent_weight >= TRAILER_CURVATURE_PROOF_FRACTION)
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
                 control_dt_s: Optional[float] = None) -> float:
        """Steering command in ``[-1, 1]`` (positive = right) to follow the route.

        ``lane_offset_m`` shifts the target line sideways: positive = keep to the
        RIGHT of the path centre (the driving lane on right-hand-traffic maps like
        ETS2), negative = left. Without this the truck drives the road centreline
        — which on a two-way road is the oncoming lane. A ~2.7 m offset keeps us
        firmly in our own lane, the main fix for "jazdí protismerom".
        """
        self.last_steering_debug = {
            "feed_forward": 0.0, "feedback": 0.0,
            "local_curvature": 0.0, "raw": 0.0, "output": 0.0,
            "curve_direction_hold": False,
            "curve_direction_hold_eligible": False,
            "curve_direction_hold_approach": False,
            "curve_direction_hold_fraction": 0.0,
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
            tangent_target = self._point_at_progress(progress + tangent_window)
            path_dx = tangent_target[0] - projection[0]
            path_dz = tangent_target[1] - projection[1]
        path_length = math.hypot(path_dx, path_dz)
        if path_length < 0.5:
            return 0.0
        path_heading = math.atan2(-path_dx, -path_dz)
        heading_error = ((heading - path_heading + math.pi)
                         % (2.0 * math.pi) - math.pi)
        fx, fz = -math.sin(heading), -math.cos(heading)
        alignment = (fx * path_dx + fz * path_dz) / path_length
        if alignment <= 0.10 or abs(heading_error) > math.radians(82.0):
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

        # --- Coherent Frenet/Ackermann guidance -----------------------------
        #
        # The 08:20 and 08:24 video replays proved a structural defect in the
        # former single-chord controller: on R12--R18 bends its chord demand
        # fell to 0.02--0.30 while the same authoritative trajectory required
        # 0.46--1.01 Ackermann feed-forward.  CTE consequently grew to 2.407 m.
        # A path chord is a useful intercept, but it cannot replace the road's
        # curvature when the vehicle is already displaced.
        #
        # Use one Frenet frame at one projected reference progress.  Curvature,
        # tangent heading and CTE are converted to steering *angles* in that
        # frame, then summed once. There is no temporal average of telemetry or
        # commands and no Route-level correction state.  LaneId/revision select
        # the projection authority only; SteeringDynamics is the sole temporal
        # execution stage.
        v = max(abs(speed_ms), 0.0)
        reference_preview = _clamp(
            v * 0.35, STEERING_REFERENCE_PREVIEW_MIN_M,
            STEERING_REFERENCE_PREVIEW_MAX_M)
        reference_progress = progress + reference_preview
        # Heading belongs to the current Frenet frame. Moving this tangent to
        # the curvature preview double-counts a constant-radius bend as both
        # feed-forward and a fictitious heading error.
        if authority_tangent is not None:
            before, after = authority_tangent
        else:
            before = self._point_at_progress(max(
                0.0, progress - STEERING_REFERENCE_TANGENT_M))
            after = self._point_at_progress(
                progress + STEERING_REFERENCE_TANGENT_M)
        reference_dx, reference_dz = after[0]-before[0], after[1]-before[1]
        if math.hypot(reference_dx, reference_dz) < 0.5:
            return 0.0
        reference_heading = math.atan2(-reference_dx, -reference_dz)
        guidance_heading_error = (
            (heading - reference_heading + math.pi)
            % (2.0 * math.pi) - math.pi)
        if abs(guidance_heading_error) > math.radians(82.0):
            return 0.0

        trailer_offset, trailer_debug = self._trailer_envelope_offset(
            progress, pos, heading, v, cte - lane_offset_m,
            vehicle_envelope, active_authority=route_authority)
        control_cte = _clamp(cte + trailer_offset, -5.0, 5.0)
        local_curvature = self._steering_curvature_at_progress(
            reference_progress, authority=route_authority)
        local_radius = (1e6 if abs(local_curvature) < 1e-9
                        else 1.0 / abs(local_curvature))
        cte_gain = curve_cte_gain(local_radius, control_cte)
        cte_steer = math.atan(
            (cte_gain * control_cte) / (K_SOFT + v))
        low_speed_capture_active = False
        if v < LOW_SPEED_CAPTURE_MAX_MS and abs(cte_steer) > 1e-9:
            geometric_limit = math.atan2(
                abs(control_cte), GUIDANCE_LOW_SPEED_LOOKAHEAD_M)
            blend = _clamp(v / LOW_SPEED_CAPTURE_MAX_MS, 0.0, 1.0)
            capture_limit = (geometric_limit
                             + (abs(cte_steer) - geometric_limit) * blend)
            if abs(cte_steer) > capture_limit:
                cte_steer = math.copysign(capture_limit, cte_steer)
                low_speed_capture_active = True

        # One controller, one unit: combine real road-wheel angles in radians
        # and normalize exactly once.  Sign convention is positive=right:
        #   delta_ff = atan(L * kappa)
        #   delta_fb = response * (K_heading * e_heading
        #                          + atan(K_cte * e_y / (v + K_soft)))
        # LaneLocator's signed error is negated once by the map plugin before
        # it reaches ``cte``; no other sign inversion occurs in this equation.
        feed_forward_angle = math.atan(
            TRUCK_WHEELBASE_M * local_curvature)
        heading_feedback_angle = (
            FEEDBACK_STEERING_RESPONSE * K_HEADING
            * guidance_heading_error)
        cte_feedback_angle = FEEDBACK_STEERING_RESPONSE * cte_steer
        feed_forward = feed_forward_angle / NORMALIZED_STEERING_ANGLE_RAD
        heading_feedback = (
            heading_feedback_angle / NORMALIZED_STEERING_ANGLE_RAD)
        cte_feedback = cte_feedback_angle / NORMALIZED_STEERING_ANGLE_RAD
        scaled_feedback = heading_feedback + cte_feedback
        steer, composition_debug = self._compose_curve_steering(
            feed_forward, heading_feedback, cte_feedback)
        steering_angle = steer * NORMALIZED_STEERING_ANGLE_RAD
        guidance_curvature = (math.tan(_clamp(
            steering_angle, -1.20, 1.20)) / TRUCK_WHEELBASE_M)
        straight_recovery_active = bool(
            abs(local_curvature) < 1.0 / 500.0
            and abs(control_cte) > 0.35)
        lane_recovery_multiplier = 1.0
        guidance_lookahead = reference_preview
        target_distance = reference_preview
        curve_direction_hold = False
        approach_profile = self.curve_profile_ahead(
            pos, heading, CURVE_DIRECTION_HOLD_APPROACH_DISTANCE_M)
        approach_signed = float(approach_profile["signed_curvature"])
        curve_direction_hold_approach = bool(
            float(approach_profile["radius_m"])
                <= CURVE_DIRECTION_HOLD_APPROACH_RADIUS_M
            and float(approach_profile["distance_m"])
                <= CURVE_DIRECTION_HOLD_APPROACH_DISTANCE_M
            and local_curvature * approach_signed > 0.0)
        # No direction-hold state is needed: noisy LaneMatch CTE is never a
        # separate steering input.  Keep the proof fields for audit/log schema
        # compatibility and to show whether locator and route still agree.
        curve_direction_hold_fraction = 0.0
        curve_direction_hold_eligible = False
        raw_steer = steer
        # A fixed ±0.70 limit physically cannot follow a proven 25–40 m prefab
        # bend in the truck model (it bottoms out near a 40 m radius), which is
        # why the real exit replay ran wide into the verge. Grant additional
        # authority only in a validated local curve; straights and broad bends
        # retain the old limit and all output remains inside the controller's
        # real [-1, 1] range.
        curvature_magnitude = abs(local_curvature)
        curve_authority = _clamp(
            (curvature_magnitude - 1.0 / 80.0)
            / (1.0 / 25.0 - 1.0 / 80.0), 0.0, 1.0)
        steering_limit = 0.70 + 0.30 * curve_authority
        steer = _clamp(steer, -steering_limit, steering_limit)
        if v < 5.0:
            standstill_limit = 0.22 + (v / 5.0) * 0.48
            steer = _clamp(steer, -standstill_limit, standstill_limit)
        # Straight geometry retains the conservative guard. A proven curve may
        # use the physical steering required to hold its lane centre.
        if abs(local_curvature) < 1.0 / 500.0:
            # A small centred correction retains the calm historical guard.
            # A confirmed LaneMatch more than 0.35 m from centre may recover
            # with bounded additional authority; otherwise ±0.16 can never
            # unwind the residual error left by a tight S-bend.
            straight_limit = 0.16
            if has_confirmed_lane_error:
                straight_limit += 0.34 * _clamp(
                    (abs(control_cte) - 0.35) / 1.15, 0.0, 1.0)
            steer = _clamp(steer, -straight_limit, straight_limit)
        steer = _clamp(steer, -1.0, 1.0)
        self.last_steering_debug = {
            "feed_forward": float(feed_forward),
            "feedback": float(scaled_feedback),
            "feedback_applied": float(
                composition_debug.get(
                    "feedback_applied", scaled_feedback)),
            "heading_feedback": float(heading_feedback),
            "cte_feedback": float(cte_feedback),
            "feed_forward_angle_rad": float(
                feed_forward_angle),
            "feedback_angle_rad": float(
                scaled_feedback * NORMALIZED_STEERING_ANGLE_RAD),
            "heading_feedback_angle_rad": float(
                heading_feedback_angle),
            "cte_feedback_angle_rad": float(cte_feedback_angle),
            "local_curvature": float(local_curvature),
            "raw": float(raw_steer),
            "output": float(steer),
            "curve_direction_hold": bool(curve_direction_hold),
            "curve_direction_hold_eligible": bool(
                curve_direction_hold_eligible),
            "curve_direction_hold_approach": bool(
                curve_direction_hold_approach),
            "curve_direction_hold_fraction": float(
                curve_direction_hold_fraction),
            "curve_direction_hold_error_proven": bool(
                cte_error_geometrically_proven),
            "geometric_cte": float(geometric_cte),
            "control_cte": float(control_cte),
            "cte_geometry_residual": float(cte_geometry_residual),
            "straight_recovery_active": bool(straight_recovery_active),
            "low_speed_capture_active": bool(low_speed_capture_active),
            "cte_steer": float(cte_steer),
            "cte_gain": float(cte_gain),
            "lane_recovery_multiplier": float(lane_recovery_multiplier),
            "guidance_lookahead_m": float(guidance_lookahead),
            "guidance_target_distance_m": float(target_distance),
            "guidance_heading_error_rad": float(guidance_heading_error),
            "guidance_curvature": float(guidance_curvature),
            "steering_limit": float(steering_limit),
            "saturated": bool(abs(raw_steer) > steering_limit + 1e-9),
            "trailer_envelope": dict(trailer_debug),
            "authority_valid": True,
            "authority_lane_id": authority_lane,
            "authority_revision": authority_revision,
            "authority_projection_distance_m": projection_distance_m,
            **composition_debug,
        }
        return steer
