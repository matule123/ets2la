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

import json
import math
import os
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]

# Tuning (kept conservative; the engine/autopilot can further damp the output).
ANGLE_GAIN = 1.6          # rad of heading error → steering (≈0.6 rad ⇒ full lock)
CTE_GAIN = 0.06           # per-metre lateral correction
MIN_LOOKAHEAD = 8.0       # metres
MAX_LOOKAHEAD = 35.0
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
        self.points: List[Point] = [tuple(p) for p in (points or [])]
        self.name = name

    # --- Construction / persistence ------------------------------------------
    def add_point(self, x: float, z: float, min_spacing: float = 10.0) -> bool:
        """Append a breadcrumb if it is at least ``min_spacing`` m from the last."""
        p = (float(x), float(z))
        if not self.points:
            self.points.append(p)
            return True
        lx, lz = self.points[-1]
        if math.hypot(p[0] - lx, p[1] - lz) >= min_spacing:
            self.points.append(p)
            return True
        return False

    def save(self, path: str):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            json.dump({"name": self.name, "points": self.points}, f)

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

    def lookahead_point(self, idx: int, pos: Point, distance: float) -> Point:
        """Walk forward along the polyline ``distance`` metres from waypoint ``idx``."""
        if not self.points:
            return pos
        remaining = distance
        i = idx
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

    def distance_to_end(self, pos: Point) -> float:
        """Path-length distance from ``pos`` (snapped to nearest waypoint) to the end."""
        if not self.points:
            return 0.0
        idx = self.closest_index(pos)
        total = math.hypot(self.points[idx][0] - pos[0], self.points[idx][1] - pos[1])
        for i in range(idx, len(self.points) - 1):
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
    def steering(self, pos: Point, heading: float, speed_ms: float = 0.0) -> float:
        """Steering command in ``[-1, 1]`` (positive = right) to follow the route."""
        if len(self.points) < 2:
            return 0.0

        idx = self.closest_index(pos)
        lookahead = _clamp(MIN_LOOKAHEAD + abs(speed_ms) * 0.9, MIN_LOOKAHEAD, MAX_LOOKAHEAD)
        tx, tz = self.lookahead_point(idx, pos, lookahead)

        # Desired direction (truck → lookahead point).
        dx, dz = tx - pos[0], tz - pos[1]
        # Truck forward vector in ETS2 world space.
        fx, fz = -math.sin(heading), -math.cos(heading)
        # Signed heading error: +angle means the target is to the right.
        cross = fx * dz - fz * dx
        dot = fx * dx + fz * dz
        heading_error = math.atan2(cross, dot)

        # Cross-track error reinforces the pure-pursuit heading error: both share
        # the same sign for a given side of the path, so they add.
        cte = self.cross_track_error(idx, pos)

        steer = heading_error * ANGLE_GAIN + cte * CTE_GAIN
        return _clamp(steer * speed_gain(speed_ms), -1.0, 1.0)
