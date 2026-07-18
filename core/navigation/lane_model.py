"""Lane-level map primitives and stable vehicle localisation.

The extracted ETS2 map does not provide a ready-made lane graph for ordinary
roads.  It does provide road-look lane lists, accurate 3-D road splines and
lane-level prefab curves.  This module keeps measured and derived values
explicit so downstream safety decisions can distinguish them.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Literal, Optional, Sequence


DataSource = Literal["dataset", "derived"]


def wrap_angle(value: float) -> float:
    """Return an angle in the closed-open interval [-pi, pi)."""
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True, slots=True)
class LaneId:
    road_uid: int
    direction: int
    lane_index: int
    prefab_token: Optional[str] = None
    connector_index: Optional[int] = None
    connector_path: tuple[int, ...] = ()

    def sort_key(self):
        return (self.road_uid, self.direction, self.lane_index,
                self.prefab_token or "", self.connector_index or -1,
                self.connector_path)


@dataclass(frozen=True, slots=True)
class LanePoint:
    x: float
    y: float
    z: float
    s: float = 0.0
    heading: float = 0.0
    curvature: float = 0.0
    lane_id: Optional[LaneId] = None
    segment_index: int = -1


@dataclass(frozen=True, slots=True)
class LaneConnection:
    target: LaneId
    kind: Literal["road", "merge", "split", "prefab", "roundabout"]
    curve_indices: tuple[int, ...] = ()
    gps_exit_uid: Optional[int] = None


@dataclass(frozen=True, slots=True)
class LaneSegment:
    lane_id: LaneId
    start_uid: int
    end_uid: int
    direction: int
    lane_index: int
    lane_count: int
    width_m: float
    width_source: DataSource
    elevation_layer: int
    road_look_token: Optional[str]
    lane_type: str
    centerline: tuple[LanePoint, ...]
    left_neighbor: Optional[LaneId] = None
    right_neighbor: Optional[LaneId] = None
    successors: tuple[LaneConnection, ...] = ()
    connector_curve_indices: tuple[int, ...] = ()
    gps_uids: frozenset[int] = field(default_factory=frozenset)


@dataclass(frozen=True, slots=True)
class LanePath:
    segments: tuple[LaneSegment, ...]
    points: tuple[LanePoint, ...]
    source_gps_uids: tuple[int, ...] = ()
    distance_m: float = 0.0
    confidence: float = 0.0
    valid: bool = False
    failure_reason: str = ""
    revision: int = 0


@dataclass(frozen=True, slots=True)
class GpsCorridorEdge:
    start_uid: int
    end_uid: int
    kind: Literal["road", "prefab", "graph"]
    gps_pair_index: int
    segment_index: Optional[int] = None
    prefab_instance: Any = None


@dataclass(frozen=True, slots=True)
class GpsCorridor:
    gps_uids: tuple[int, ...]
    edges: tuple[GpsCorridorEdge, ...]
    valid: bool
    failure_reason: str = ""


@dataclass(frozen=True, slots=True)
class LaneMatch:
    lane_id: LaneId
    point: LanePoint
    segment_index: int
    point_index: int
    lateral_error_m: float
    vertical_error_m: float
    heading_error_rad: float
    score: float
    confidence: float
    switch_reason: str


@dataclass(frozen=True, slots=True)
class LaneLocatorConfig:
    search_radius_m: float = 28.0
    max_lateral_m: float = 11.0
    max_vertical_m: float = 4.0
    max_heading_rad: float = math.radians(100.0)
    heading_weight: float = 5.5
    vertical_weight: float = 3.0
    off_route_penalty: float = 7.0
    discontinuity_penalty: float = 4.0
    derived_width_penalty: float = 0.6
    switch_margin: float = 1.5
    ambiguity_margin: float = 0.25


class LaneLocator:
    """Heading/elevation/topology-aware locator with score hysteresis."""

    def __init__(self, network, config: Optional[LaneLocatorConfig] = None):
        self.network = network
        self.config = config or LaneLocatorConfig()
        self.previous: Optional[LaneMatch] = None

    @staticmethod
    def _project(position, lane: LaneSegment):
        px, py, pz = position
        best = None
        points = lane.centerline
        for index, (a, b) in enumerate(zip(points, points[1:])):
            dx, dz = b.x - a.x, b.z - a.z
            length2 = dx * dx + dz * dz
            if length2 < 1e-8:
                continue
            t = max(0.0, min(1.0,
                ((px - a.x) * dx + (pz - a.z) * dz) / length2))
            qx, qz = a.x + dx * t, a.z + dz * t
            qy = a.y + (b.y - a.y) * t
            distance = math.hypot(px - qx, pz - qz)
            if best is None or distance < best[0]:
                heading = math.atan2(-dx, -dz)
                signed = (((px - qx) * (-dz) + (pz - qz) * dx)
                          / math.sqrt(length2))
                best = (distance, LanePoint(qx, qy, qz,
                            a.s + (b.s - a.s) * t, heading), index,
                        index + (1 if t >= 0.5 else 0), signed, abs(py - qy))
        return best

    def locate(self, position: Sequence[float], heading: float,
               gps_uids: Sequence[int] = (),
               previous: Optional[LaneMatch] = None) -> Optional[LaneMatch]:
        if len(position) == 2:
            px, pz = position
            py = float(self.network.altitude_near((px, pz)) or 0.0)
        else:
            px, py, pz = map(float, position[:3])
        previous = previous if previous is not None else self.previous
        gps = frozenset(int(uid) for uid in gps_uids)
        candidates = self.network.lane_segments_near(
            (px, pz), self.config.search_radius_m)
        ranked = []
        for lane in candidates:
            projected = self._project((px, py, pz), lane)
            if projected is None:
                continue
            distance, point, segment_index, point_index, signed, vertical = projected
            heading_error = abs(wrap_angle(heading - point.heading))
            if (distance > self.config.max_lateral_m
                    or vertical > self.config.max_vertical_m
                    or heading_error > self.config.max_heading_rad):
                continue
            on_route = not gps or bool(lane.gps_uids & gps)
            continuous = (previous is None
                          or lane.lane_id == previous.lane_id
                          or self.network.lanes_connected(
                              previous.lane_id, lane.lane_id))
            if previous is not None and not continuous:
                # Hysteresis is not permission to teleport to a nearby road.
                # A transition must be topologically confirmed by the network.
                continue
            score = (distance
                     + heading_error * self.config.heading_weight
                     + vertical * self.config.vertical_weight
                     + (0.0 if on_route else self.config.off_route_penalty)
                     + (self.config.derived_width_penalty
                        if lane.width_source == "derived" else 0.0))
            confidence = max(0.0, min(1.0, 1.0 - score / 18.0))
            ranked.append((score, lane, point, segment_index, point_index, signed,
                           vertical, heading_error, confidence))
        if not ranked:
            self.previous = None
            return None
        ranked.sort(key=lambda item: (item[0], item[1].lane_id.sort_key()))
        # An initial exact/near tie is not a reliable lane match. Silently
        # breaking it by LaneId can select a parallel road or carriageway.
        if (previous is None and len(ranked) > 1
                and ranked[1][0] - ranked[0][0]
                    <= self.config.ambiguity_margin):
            self.previous = None
            return None
        chosen = ranked[0]
        reason = "best_score" if previous is None else "better_lane"
        if previous is not None:
            old = next((item for item in ranked
                        if item[1].lane_id == previous.lane_id), None)
            if old is not None and old[0] <= chosen[0] + self.config.switch_margin:
                chosen = old
                reason = "hysteresis_hold"
            elif chosen[1].lane_id == previous.lane_id:
                reason = "same_lane"
            elif self.network.lanes_connected(previous.lane_id,
                                              chosen[1].lane_id):
                reason = "topology_transition"
        score, lane, point, segment_index, point_index, signed, vertical, error, confidence = chosen
        match = LaneMatch(lane.lane_id, point, segment_index, point_index,
                          signed, vertical, wrap_angle(heading - point.heading),
                          score, confidence, reason)
        self.previous = match
        return match
