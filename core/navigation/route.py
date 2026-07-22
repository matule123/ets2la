"""
Coordinate-based route navigation for UltraPilot.

A :class:`Route` is a polyline of world ``(x, z)`` waypoints captured from SCS
telemetry.  Given the truck's current world pose it produces a steering value in
``[-1, 1]`` using the same idea as ETS2LA's ``GetSteering`` — a blend of
**heading error** to a lookahead point (pure-pursuit) and **cross-track error**
(perpendicular distance to the path), scaled down with speed.

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
K_CTE = 0.70              # damped lane-centre recovery; avoids right/left hunting
K_SOFT = 1.0              # softening constant → CTE term never explodes at v=0
MIN_LOOKAHEAD = 22.0
MAX_LOOKAHEAD = 75.0
# Curvature-aware lookahead: look FAR ahead on straights (anticipate), but
# TIGHTEN the lookahead in sharp curves (react precisely to the apex). The
# path's local curvature is measured over CURV_WINDOW_M of road ahead; a
# tight radius shrinks the lookahead so we track the apex instead of cutting
# across the oncoming lane / kerb.
CURV_WINDOW_M = 40.0
STRAIGHT_LOOKAHEAD = 70.0
TIGHT_CURVE_LOOKAHEAD = 18.0
TIGHT_CURVE_RADIUS = 60.0   # radius below this = "tight" (shrinks lookahead)
ARRIVAL_RADIUS = 12.0     # metres from the last point counts as "arrived"


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def speed_gain(speed_ms: float) -> float:
    """Gentler steering at speed, sharper when crawling (like ETS2LA's schedule)."""
    speed_kmh = abs(speed_ms) * 3.6
    # 1.3 at standstill → ~0.5 at 90 km/h, floored.
    return _clamp(1.3 - (speed_kmh / 90.0) * 0.8, 0.45, 1.3)


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
    def curvature_ahead(self, pos: Point, heading: float,
                        window_m: float = CURV_WINDOW_M) -> float:
        """Radius (m) of the sharpest bend in the next ``window_m`` of path.

        Returns a large number (≈straight) when the road is straight or there
        isn't enough path. Used two ways: (1) to shrink the steering lookahead
        into tight curves so the truck tracks the apex instead of cutting it,
        and (2) by the autopilot to brake *before* a sharp bend rather than
        mid-corner. The estimate is the discrete Menger curvature (circle
        through three points: the truck, a near point, a far point)."""
        if len(self.points) < 3:
            return 1e6
        idx = self.tracking_index(pos, heading)
        # Sample the path at three positions along the upcoming window.
        p0 = (pos[0], pos[1])
        p1 = self.lookahead_point(idx, pos, window_m * 0.5)
        p2 = self.lookahead_point(idx, pos, window_m)
        # Menger curvature: k = 4·area / (|a||b||c|), radius = 1/|k|.
        ax, ay = p1[0] - p0[0], p1[1] - p0[1]
        bx, by = p2[0] - p0[0], p2[1] - p0[1]
        cx, cy = p2[0] - p1[0], p2[1] - p1[1]
        area = abs(ax * by - ay * bx) * 0.5   # triangle area
        a = math.hypot(ax, ay)
        b = math.hypot(bx, by)
        c = math.hypot(cx, cy)
        # Degenerate triangle (collinear → straight road, or coincident points)
        # means "no curvature"; return a huge radius. area→0 with non-zero side
        # lengths is the straight-line case and WOULD divide by zero without
        # this guard, so we check both the sides and the area.
        if a < 1e-3 or b < 1e-3 or c < 1e-3 or area < 1e-6:
            return 1e6
        prod = a * b * c
        if prod < 1e-6:
            return 1e6
        return prod / (4.0 * area)            # circumradius (m)

    def steering(self, pos: Point, heading: float, speed_ms: float = 0.0,
                 lane_offset_m: float = 0.0) -> float:
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

        # --- Curvature-aware lookahead (Fáza 3b) ---------------------------
        # Look far ahead on straights (so we anticipate the next bend early),
        # but tighten the lookahead inside a sharp curve (so we track the apex
        # instead of cutting across it). Radius → 0 shrinks toward the tight
        # value; radius → ∞ relaxes toward the straight value. Speed still
        # nudges the lookahead up a little so a fast truck sees further.
        radius = self.curvature_ahead(pos, heading)
        # 0 at straight (radius≥200), 1 at tight (radius≤TIGHT_CURVE_RADIUS).
        tight = _clamp((200.0 - radius) / (200.0 - TIGHT_CURVE_RADIUS), 0.0, 1.0)
        speed_look = abs(speed_ms) * 1.2
        lookahead = _clamp(
            STRAIGHT_LOOKAHEAD - tight * (STRAIGHT_LOOKAHEAD - TIGHT_CURVE_LOOKAHEAD)
            + speed_look * (1.0 - tight),
            TIGHT_CURVE_LOOKAHEAD, MAX_LOOKAHEAD,
        )
        # Keep the target before a junction corner until the cab reaches the
        # confirmed connector. A long lookahead otherwise cuts across islands.
        walked = 0.0
        base_heading = None
        for i in range(idx, min(len(self.points) - 1, idx + 80)):
            ax, az = self.points[i]
            bx, bz = self.points[i + 1]
            seg = math.hypot(bx-ax, bz-az)
            if seg < 1e-5:
                continue
            tangent = math.atan2(-(bx-ax), -(bz-az))
            if base_heading is None:
                base_heading = tangent
            change = abs((tangent-base_heading+math.pi) % (2*math.pi)-math.pi)
            # Only treat a real junction/corner as a gate. Gentle continuous
            # bends must keep their normal preview or steering changes late.
            if change > math.radians(25.0) and walked > 4.0:
                lookahead = min(lookahead, max(MIN_LOOKAHEAD, walked - 3.0))
                break
            walked += seg
        tx, tz = self.lookahead_point(idx, pos, lookahead)

        # Shift the lookahead + the reference line sideways by lane_offset_m, so
        # we aim for our lane (right of centre) instead of the oncoming lane.
        if abs(lane_offset_m) > 1e-3:
            j = min(idx, len(self.points) - 2)
            ax, az = self.points[j]
            bx, bz = self.points[j + 1]
            sdx, sdz = bx - ax, bz - az
            sl = math.hypot(sdx, sdz) or 1.0
            # right-of-travel offset vector in ETS2's X/Z plane
            ox, oz = (-sdz / sl) * lane_offset_m, (sdx / sl) * lane_offset_m
            tx += ox
            tz += oz

        # Desired direction (truck → lookahead point).
        dx, dz = tx - pos[0], tz - pos[1]
        # Truck forward vector in ETS2 world space.
        fx, fz = -math.sin(heading), -math.cos(heading)
        # Signed heading error: +angle means the target is to the right.
        # Standard 2-D cross(target, forward): positive means target is on the
        # truck's right in ETS2's x/z coordinate system.
        cross = fx * dz - fz * dx
        dot = fx * dx + fz * dz
        heading_error = math.atan2(cross, dot)

        # Never chase a target behind the cab. This is a stale/wrong branch,
        # not a valid steering request.
        if dot <= 1.0 or abs(heading_error) > math.radians(82.0):
            return 0.0

        # Cross-track error, measured to the lane-offset line so it pulls us
        # into our lane, not the centre. CLAMPED to ±5 m: when the truck is far
        # from the road (e.g. a wrong map dataset is loaded, or we're on a ferry
        # / car park) the raw CTE can be 30+ m, which saturates the Stanley law
        # to full-lock — that's the „truck yanks hard left the moment autopilot
        # engages" bug. Capping it keeps the steering reasonable while still
        # pulling back toward the lane.
        cte = self.cross_track_error(idx, pos) + lane_offset_m
        cte = max(-5.0, min(5.0, cte))

        # --- Stanley lateral-control law (Fáza 3a) -------------------------
        #   δ = K_HEADING · heading_error + atan( K_CTE · cte / (K_SOFT + v) )
        # The CTE term is a steering ANGLE (not a velocity), so it's damped at
        # speed (K_SOFT + v in the denominator) and strong at crawl. Combined
        # with the heading error it tracks the lane without the oscillation the
        # old pure-gain sum produced in S-bends. The speed_gain schedule scales
        # the whole command down with speed (gentle inputs at 90 km/h).
        v = max(abs(speed_ms), 0.0)
        cte_steer = math.atan((K_CTE * cte) / (K_SOFT + v))
        steer = K_HEADING * heading_error + cte_steer
        # Clamp the *angle* before the speed gain — without this a 90° heading
        # error + maxed CTE produced steer values > 2.0, which then became ±1.0
        # after _clamp and looked like „always full lock one way".
        steer = max(-0.7, min(0.7, steer))
        steer *= speed_gain(speed_ms)
        # On a genuinely straight road only small lane-centering corrections
        # are valid. This prevents a bad waypoint from winding the wheel until
        # the truck leaves its lane, while tight roundabouts remain unrestricted.
        if radius > 300.0:
            steer = _clamp(steer, -0.16, 0.16)
        elif radius > 100.0:
            # A broad road bend must not wind on 40% steering at launch merely
            # because the speed-dependent Stanley term is strongest at zero
            # speed. Tight prefab turns remain unrestricted below 100 m.
            steer = _clamp(steer, -0.22, 0.22)
        return _clamp(steer, -1.0, 1.0)
