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

# Tuning: gentle + far lookahead so the truck anticipates curves smoothly
# instead of jerking late into them (which caused it to crash on bends).
#
# The lateral controller is now a **Stanley law** (Hoffmann/Stanford, the
# standard for kinematic lane-keeping) instead of two hand-tuned gains:
#     δ = heading_error + atan( k_cte · cte / (k_soft + speed) )
# This couples the heading correction and the cross-track correction in a
# physically meaningful way: at speed the CTE term is damped (no twitchy
# over-correction), at crawl it's strong (precise low-speed placement). It
# tracks curves far better than the old ANGLE_GAIN·h + CTE_GAIN·cte sum,
# which oscillated in S-bends because the two terms fought each other.
K_HEADING = 1.0           # heading-error weight (Stanley keeps this at 1.0)
K_CTE = 0.80              # damped lane-centre recovery; avoids edge tracking
K_CTE_CURVE = 1.80        # hold the mapped lane centre against curve cutting
K_SOFT = 1.0              # softening constant → CTE term never explodes at v=0
TRUCK_WHEELBASE_M = 5.0
NORMALIZED_STEERING_ANGLE_RAD = 0.18
# Calibrated conversion of Stanley's feedback angle to the normalized SCS
# steering input. The former implicit factor was 0.18 (feedback radians were
# effectively treated as controller units), which the real trace proves too
# weak; a full 1.0 physical conversion hunts on sampled S-curves. Closed-loop
# sweeps across 0--90 km/h select this bounded response.
FEEDBACK_STEERING_RESPONSE = 0.21
STEERING_CURVATURE_SPAN_M = 6.0
STEERING_PREVIEW_MIN_M = 0.5
STEERING_PREVIEW_MAX_M = 4.0
# A longer window is retained for anticipatory curve braking; steering uses
# the shorter local window above so it cannot cut across a bend.
CURV_WINDOW_M = 60.0
CURVE_PROFILE_STEP_M = 4.0
TIGHT_CURVE_RADIUS = 60.0
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
    def __init__(self, points: Optional[Sequence[Point]] = None, name: str = "route"):
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

    def _curvature_at_progress(self, progress_m: float,
                               span_m: float = STEERING_CURVATURE_SPAN_M) -> float:
        """Signed Menger curvature around one route-progress sample."""
        if len(self.points) < 3 or self._cumulative_m[-1] < 2.0:
            return 0.0
        total = self._cumulative_m[-1]
        span = max(2.0, float(span_m))
        centre = _clamp(float(progress_m), 0.0, total)
        before = max(0.0, centre - span)
        after = min(total, centre + span)
        # Near either trajectory end retain a full, one-sided sample rather
        # than collapsing two points and manufacturing infinite curvature.
        if centre - before < 1.0:
            before = centre
            centre = min(total, before + span)
            after = min(total, before + span * 2.0)
        elif after - centre < 1.0:
            after = centre
            centre = max(0.0, after - span)
            before = max(0.0, after - span * 2.0)
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
                 cross_track_error_m: Optional[float] = None) -> float:
        """Steering command in ``[-1, 1]`` (positive = right) to follow the route.

        ``lane_offset_m`` shifts the target line sideways: positive = keep to the
        RIGHT of the path centre (the driving lane on right-hand-traffic maps like
        ETS2), negative = left. Without this the truck drives the road centreline
        — which on a two-way road is the oncoming lane. A ~2.7 m offset keeps us
        firmly in our own lane, the main fix for "jazdí protismerom".
        """
        if len(self.points) < 2:
            return 0.0

        # A plain nearest-waypoint lookup is ambiguous on divided motorways,
        # roundabouts and junctions.  Use the heading-aware segment selected by
        # the same geometry used for localisation, otherwise steering can jump
        # onto a neighbouring arm and immediately pull across the median.
        idx = self.tracking_index(pos, heading)

        # Use the proven stable preview from the pre-intent controller. A very
        # short 2–4 m tangent and 4–8 m curvature window amplified normal
        # two-metre LaneTrajectory sampling noise into alternating full-lock
        # commands at segment boundaries.
        # Stanley uses the local path tangent. A direction to a 70 m chord
        # cuts one bend toward the median and the opposite bend toward grass.
        projection = self.lookahead_point(idx, pos, 0.0)
        tangent_window = _clamp(3.0 + abs(speed_ms) * 0.15, 3.0, 6.0)
        tangent_target = self.lookahead_point(idx, pos, tangent_window)
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
        # / car park) the raw CTE can be 30+ m, which saturates the Stanley law
        # to full-lock — that's the „truck yanks hard left the moment autopilot
        # engages" bug. Capping it keeps the steering reasonable while still
        # pulling back toward the lane.
        has_confirmed_lane_error = cross_track_error_m is not None
        cte = (self.cross_track_error(idx, pos)
               if cross_track_error_m is None
               else float(cross_track_error_m)) + lane_offset_m
        cte = max(-5.0, min(5.0, cte))

        # --- Stanley lateral-control law (Fáza 3a) -------------------------
        #   δ = K_HEADING · heading_error + atan( K_CTE · cte / (K_SOFT + v) )
        # The CTE term is a steering ANGLE (not a velocity), so it's damped at
        # speed (K_SOFT + v in the denominator) and strong at crawl. Combined
        # with the heading error it tracks the lane without the oscillation the
        # old pure-gain sum produced in S-bends. The speed_gain schedule scales
        # the whole command down with speed (gentle inputs at 90 km/h).
        v = max(abs(speed_ms), 0.0)
        progress = self.tracking_progress(pos, heading)
        # Feed-forward remains local. The long curve horizon belongs only to
        # speed/braking; moving this preview farther ahead makes the truck cut
        # the straight before a junction rather than follow the lane entry.
        preview = _clamp(v * 0.35, STEERING_PREVIEW_MIN_M,
                         STEERING_PREVIEW_MAX_M)
        local_curvature = self._curvature_at_progress(progress + preview)
        local_radius = (1e6 if abs(local_curvature) < 1e-9
                        else 1.0 / abs(local_curvature))
        cte_gain = curve_cte_gain(local_radius, cte)
        if (has_confirmed_lane_error
                and abs(local_curvature) < 1.0 / 100.0):
            # On the straight immediately after a bend the old ±0.16 guard and
            # base Stanley gain could preserve a 1–1.7 m residual offset for
            # tens of metres. Scale recovery by *measured* displacement while
            # leaving sub-35 cm localisation noise and tight curves untouched.
            recovery_weight = _clamp(
                (abs(cte) - 0.35) / 1.15, 0.0, 1.0)
            cte_gain *= 1.0 + 1.75 * recovery_weight
        cte_steer = math.atan(
            (cte_gain * cte) / (K_SOFT + v))
        feed_forward = (math.atan(TRUCK_WHEELBASE_M * local_curvature)
                        / NORMALIZED_STEERING_ANGLE_RAD)
        # Heading/CTE feedback stays deliberately damped: amplifying every
        # sampled tangent or CTE through the physical Ackermann scale makes an
        # S-bend hunt.  The confirmed-lane recovery gain above fixes the real
        # broad-curve drift without changing tight prefab steering.
        feedback = (FEEDBACK_STEERING_RESPONSE
                    * (K_HEADING * heading_error + cte_steer)
                    / NORMALIZED_STEERING_ANGLE_RAD)
        steer = feed_forward + speed_gain(speed_ms) * feedback
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
                    (abs(cte) - 0.35) / 1.15, 0.0, 1.0)
            steer = _clamp(steer, -straight_limit, straight_limit)
        return _clamp(steer, -1.0, 1.0)
