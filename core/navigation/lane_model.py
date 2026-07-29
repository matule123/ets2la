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
    # Index in the unfiltered road-look lane array. This differs from
    # ``lane_index`` when SCS inserts rail/no_vehicles lanes and is essential
    # for preserving the physical lane across adjacent road items.
    raw_lane_index: int = -1


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
    score_components: tuple[tuple[str, float], ...] = ()


@dataclass(frozen=True, slots=True)
class LaneLocatorConfig:
    search_radius_m: float = 28.0
    # A different carriageway at a junction can be only 6-10 m away.  It is
    # not a valid fallback for the truck's lane and must never become steering
    # authority merely because it is geometrically nearby.
    max_lateral_m: float = 2.4
    # Acquisition remains strict.  Retention is lane-width-aware only for the
    # exact previously confirmed LaneId, so a curved road sample or delayed
    # telemetry tick cannot erase a valid match before score hysteresis runs.
    # It is never available to a neighbouring/opposing/new candidate.
    same_lane_retention_width_fraction: float = 0.75
    same_lane_retention_max_distance_m: float = 3.4
    same_lane_retention_max_progress_m: float = 35.0
    max_vertical_m: float = 4.0
    # More than 65 degrees means an intersecting/opposing arm, not the lane the
    # truck is currently travelling on.  The former 100 degree allowance could
    # initialise navigation on the oncoming carriageway.
    max_heading_rad: float = math.radians(35.0)
    heading_weight: float = 7.0
    vertical_weight: float = 3.0
    # SCS telemetry reports the vehicle reference point while map nodes retain
    # the road surface/reference height. Real ProMods 1.59 samples show a
    # stable ~0.9-1.5 m same-deck bias. Keep the strict 4 m hard gate so a
    # bridge cannot match the road below, but do not score that expected bias
    # as uncertainty.
    vertical_score_deadband_m: float = 1.5
    off_route_penalty: float = 7.0
    discontinuity_penalty: float = 4.0
    derived_width_penalty: float = 0.6
    switch_margin: float = 1.5
    ambiguity_margin: float = 0.25
    # A neighbouring road lane is not a normal topology hop. Confirm it from
    # at least two progressive localization samples so one noisy frame cannot
    # rebuild the route on another LaneId. These bounds are lane-change-only;
    # they do not widen acquisition, heading, elevation or ambiguity gates.
    lane_change_min_samples: int = 2
    lane_change_min_lateral_progress_m: float = 0.6
    lane_change_max_longitudinal_step_m: float = 35.0
    lane_change_progress_tolerance_m: float = 0.2
    lane_change_max_lane_heading_delta_rad: float = math.radians(15.0)


@dataclass(slots=True)
class _LaneChangeEvidence:
    source: LaneId
    target: LaneId
    source_lateral_start_m: float
    target_lateral_start_m: float
    source_lateral_last_m: float
    target_lateral_last_m: float
    source_lateral_sign: int
    last_position: tuple[float, float, float]
    samples: int = 1


class LaneLocator:
    """Heading/elevation/topology-aware locator with score hysteresis."""

    def __init__(self, network, config: Optional[LaneLocatorConfig] = None):
        self.network = network
        self.config = config or LaneLocatorConfig()
        self.previous: Optional[LaneMatch] = None
        self._lane_change_evidence: Optional[_LaneChangeEvidence] = None

    def _adjacent_road_lane_change(self, source: LaneId, target: LaneId):
        """Return lane segments only for a topology-proven adjacent change."""
        if (source.direction != target.direction
                or abs(source.lane_index - target.lane_index) != 1
                or source.prefab_token is not None
                or target.prefab_token is not None):
            return None
        index = getattr(self.network, "_lane_id_index", {})
        first, second = index.get(source), index.get(target)
        if first is None or second is None:
            # Synthetic/test networks need not expose the production index.
            lanes = getattr(self.network, "lanes", ())
            first = next((lane for lane in lanes if lane.lane_id == source), None)
            second = next((lane for lane in lanes if lane.lane_id == target), None)
        if first is None or second is None:
            return None
        # A road-edge change is handled by its explicit successor/merge/split
        # topology. Lane indices can legitimately be renumbered there, so it
        # must not be mistaken for a lateral manoeuvre. Progressive adjacent
        # switching is confirmed only between the parallel lanes generated
        # from one road item.
        if source.road_uid != target.road_uid:
            return None
        if (first.elevation_layer != second.elevation_layer
                or abs(wrap_angle(
                    first.centerline[len(first.centerline) // 2].heading
                    - second.centerline[len(second.centerline) // 2].heading))
                    > self.config.lane_change_max_lane_heading_delta_rad):
            return None
        return first, second

    def _confirm_lane_change(self, previous, target_item, raw_projections,
                             position, diagnostic_mode):
        """Observe a real adjacent-lane crossing without mutating diagnostics."""
        target_lane = target_item[1]
        pair = self._adjacent_road_lane_change(
            previous.lane_id, target_lane.lane_id)
        if pair is None:
            if not diagnostic_mode:
                self._lane_change_evidence = None
            return False
        source_raw = raw_projections.get(previous.lane_id)
        target_raw = raw_projections.get(target_lane.lane_id)
        if source_raw is None or target_raw is None:
            if not diagnostic_mode:
                self._lane_change_evidence = None
            return False
        source_projection = source_raw[1]
        target_projection = target_raw[1]
        source_lateral = abs(source_projection[4])
        target_lateral = abs(target_projection[4])
        source_sign = (1 if source_projection[4] > 0.0 else
                       -1 if source_projection[4] < 0.0 else 0)
        # The truck must already have crossed the midpoint toward the adjacent
        # lane. A score change caused by route penalty or a nearby parallel arm
        # cannot start lane-change evidence.
        if source_lateral < target_lateral + 0.25 or source_sign == 0:
            if not diagnostic_mode:
                self._lane_change_evidence = None
            return False
        if (source_projection[5] > self.config.max_vertical_m
                or source_raw[2] > self.config.max_heading_rad):
            if not diagnostic_mode:
                self._lane_change_evidence = None
            return False
        if diagnostic_mode:
            # Diagnostic queries are observational. Production already made
            # the stateful decision; expose the raw adjacent candidate without
            # consuming a sample or changing the confirmation history.
            return True
        evidence = self._lane_change_evidence
        if (evidence is None or evidence.source != previous.lane_id
                or evidence.target != target_lane.lane_id):
            self._lane_change_evidence = _LaneChangeEvidence(
                previous.lane_id, target_lane.lane_id,
                source_lateral, target_lateral,
                source_lateral, target_lateral, source_sign,
                tuple(map(float, position)),
            )
            return False
        dx = float(position[0]) - evidence.last_position[0]
        dz = float(position[2]) - evidence.last_position[2]
        lane_heading = target_projection[1].heading
        longitudinal = dx * -math.sin(lane_heading) + dz * -math.cos(lane_heading)
        tolerance = self.config.lane_change_progress_tolerance_m
        progressive = bool(
            source_sign == evidence.source_lateral_sign
            and source_lateral + tolerance >= evidence.source_lateral_last_m
            and target_lateral <= evidence.target_lateral_last_m + tolerance
            and -tolerance <= longitudinal
                <= self.config.lane_change_max_longitudinal_step_m)
        if not progressive:
            self._lane_change_evidence = None
            return False
        evidence.samples += 1
        evidence.source_lateral_last_m = source_lateral
        evidence.target_lateral_last_m = target_lateral
        evidence.last_position = tuple(map(float, position))
        confirmed = bool(
            evidence.samples >= self.config.lane_change_min_samples
            and source_lateral - evidence.source_lateral_start_m
                >= self.config.lane_change_min_lateral_progress_m
            and evidence.target_lateral_start_m - target_lateral
                >= self.config.lane_change_min_lateral_progress_m)
        if confirmed:
            self._lane_change_evidence = None
        return confirmed

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
               previous: Optional[LaneMatch] = None,
               diagnostics: Optional[dict] = None,
               diagnostic_mode: bool = False) -> Optional[LaneMatch]:
        """Locate the lane; diagnostic mode never mutates locator history."""
        if len(position) == 2:
            px, pz = position
            py = float(self.network.altitude_near((px, pz)) or 0.0)
        else:
            px, py, pz = map(float, position[:3])
        previous = previous if previous is not None else self.previous
        if previous is None and not diagnostic_mode:
            self._lane_change_evidence = None
        gps_order = tuple(int(uid) for uid in gps_uids)
        gps = frozenset(gps_order)
        directed_gps_edges = frozenset(zip(gps_order, gps_order[1:]))
        candidates = self.network.lane_segments_near(
            (px, pz), self.config.search_radius_m)
        prefix_candidates = getattr(
            self.network, "route_prefix_lane_segments_near", None)
        if gps_order and callable(prefix_candidates):
            candidates = list(candidates) + list(prefix_candidates(
                (px, py, pz), gps_order, self.config.search_radius_m,
                register=not diagnostic_mode))
        route_candidates = getattr(
            self.network, "gps_prefab_lane_segments_near", None)
        if len(gps_order) >= 2 and callable(route_candidates):
            candidates = list(candidates) + list(route_candidates(
                (px, py, pz), gps_order, self.config.search_radius_m,
                register=not diagnostic_mode))
        # The same terminal navCurve can be both the route-prefix candidate
        # and a directly GPS-proven candidate. It is one lane, not an
        # ambiguity merely because two conservative queries returned it.
        unique_candidates = {}
        candidate_signatures = {}
        colliding_lane_ids = set()
        for candidate in candidates:
            signature = (
                candidate.start_uid, candidate.end_uid, candidate.direction,
                candidate.lane_index, candidate.lane_count,
                candidate.width_m, candidate.width_source,
                candidate.elevation_layer, candidate.road_look_token,
                candidate.lane_type, candidate.connector_curve_indices,
                candidate.raw_lane_index, candidate.centerline,
            )
            previous_signature = candidate_signatures.get(candidate.lane_id)
            if previous_signature is None:
                unique_candidates[candidate.lane_id] = candidate
                candidate_signatures[candidate.lane_id] = signature
            elif previous_signature != signature:
                # One LaneId cannot identify two directions, geometries or
                # elevation layers. Keeping either candidate would make query
                # order decide navigation authority, so reject the collision.
                colliding_lane_ids.add(candidate.lane_id)
        candidates = [candidate for lane_id, candidate
                      in unique_candidates.items()
                      if lane_id not in colliding_lane_ids]
        if diagnostics is not None:
            diagnostics.clear()
            diagnostics.update({
                "world": {"x": px, "y": py, "z": pz},
                "truck_heading_rad": float(heading),
                "candidate_lanes": [],
                "outcome": "pending",
            })
        ranked = []
        raw_projections = {}
        for lane in candidates:
            projected = self._project((px, py, pz), lane)
            if projected is None:
                continue
            distance, point, segment_index, point_index, signed, vertical = projected
            lateral = abs(signed)
            longitudinal = math.sqrt(max(
                0.0, distance * distance - lateral * lateral))
            heading_error = abs(wrap_angle(heading - point.heading))
            raw_projections[lane.lane_id] = (lane, projected, heading_error)
            same_previous_lane = bool(
                previous is not None and lane.lane_id == previous.lane_id)
            previous_progress = (
                math.dist(
                    (point.x, point.y, point.z),
                    (previous.point.x, previous.point.y, previous.point.z))
                if same_previous_lane else float("inf"))
            retention_distance_limit = max(
                self.config.max_lateral_m,
                min(self.config.same_lane_retention_max_distance_m,
                    lane.width_m
                    * self.config.same_lane_retention_width_fraction))
            retention_allowed = bool(
                same_previous_lane
                and previous_progress
                    <= self.config.same_lane_retention_max_progress_m)
            distance_limit = (
                retention_distance_limit if retention_allowed
                else self.config.max_lateral_m)
            candidate_diagnostic = None
            if diagnostics is not None:
                candidate_diagnostic = {
                    "lane_id": {
                        "road_uid": int(lane.lane_id.road_uid),
                        "direction": int(lane.lane_id.direction),
                        "lane_index": int(lane.lane_id.lane_index),
                        "prefab_token": lane.lane_id.prefab_token,
                        "connector_index": lane.lane_id.connector_index,
                        "connector_path": list(lane.lane_id.connector_path),
                    },
                    "road_token": lane.road_look_token,
                    "prefab_token": lane.lane_id.prefab_token,
                    "distance_m": float(distance),
                    "signed_lateral_m": float(signed),
                    "longitudinal_overrun_m": float(longitudinal),
                    "previous_progress_m": (
                        float(previous_progress)
                        if math.isfinite(previous_progress) else None),
                    "same_lane_retention": bool(retention_allowed),
                    "distance_limit_m": float(distance_limit),
                    "nearest_world": {
                        "x": float(point.x), "y": float(point.y),
                        "z": float(point.z),
                    },
                    "lane_heading_rad": float(point.heading),
                    "lane_heading_deg": math.degrees(point.heading),
                    "heading_error_rad": float(heading_error),
                    "heading_error_deg": math.degrees(heading_error),
                    "elevation_difference_m": float(vertical),
                    "accepted": False,
                    "rejection": None,
                    "score": None,
                    "confidence": None,
                    "score_components": {},
                }
                diagnostics["candidate_lanes"].append(candidate_diagnostic)
            if (distance > distance_limit
                    or vertical > self.config.max_vertical_m
                    or heading_error > self.config.max_heading_rad):
                if candidate_diagnostic is not None:
                    rejected = []
                    if distance > distance_limit:
                        rejected.append("lateral")
                    if vertical > self.config.max_vertical_m:
                        rejected.append("elevation")
                    if heading_error > self.config.max_heading_rad:
                        rejected.append("heading")
                    candidate_diagnostic["rejection"] = "+".join(rejected)
                continue
            exact_route_edge = ((lane.start_uid, lane.end_uid)
                                in directed_gps_edges)
            on_route = not gps or bool(lane.gps_uids & gps)
            invalid_adjacent_change = bool(
                previous is not None
                and previous.lane_id.road_uid == lane.lane_id.road_uid
                and previous.lane_id.direction == lane.lane_id.direction
                and abs(previous.lane_id.lane_index
                        - lane.lane_id.lane_index) == 1
                and self._adjacent_road_lane_change(
                    previous.lane_id, lane.lane_id) is None)
            continuous = (previous is None
                          or lane.lane_id == previous.lane_id
                          or (not invalid_adjacent_change
                              and self.network.lanes_connected(
                                  previous.lane_id, lane.lane_id)))
            if previous is not None and not continuous:
                # Hysteresis is not permission to teleport to a nearby road.
                # A transition must be topologically confirmed by the network.
                if candidate_diagnostic is not None:
                    candidate_diagnostic["rejection"] = "topology"
                continue
            components = (
                ("lateral", float(distance)),
                ("heading", float(heading_error * self.config.heading_weight)),
                ("vertical", float(max(
                    0.0, vertical - self.config.vertical_score_deadband_m)
                    * self.config.vertical_weight)),
                ("off_route", float(
                    0.0 if exact_route_edge or not gps else
                    min(2.0, self.config.off_route_penalty) if on_route else
                    self.config.off_route_penalty)),
                ("derived_width", float(
                    self.config.derived_width_penalty
                    if lane.width_source == "derived" else 0.0)),
            )
            score = sum(value for _, value in components)
            confidence = max(0.0, min(1.0, 1.0 - score / 18.0))
            if candidate_diagnostic is not None:
                candidate_diagnostic.update({
                    "accepted": True,
                    "score": float(score),
                    "confidence": float(confidence),
                    "score_components": dict(components),
                })
            ranked.append((score, lane, point, segment_index, point_index, signed,
                           vertical, heading_error, confidence, components))
        if not ranked:
            if diagnostics is not None:
                diagnostics["outcome"] = "no_match"
            if not diagnostic_mode:
                self.previous = None
                self._lane_change_evidence = None
            return None
        ranked.sort(key=lambda item: (item[0], item[1].lane_id.sort_key()))
        # An initial exact/near tie is not a reliable lane match. Silently
        # breaking it by LaneId can select a parallel road or carriageway.
        if (previous is None and len(ranked) > 1
                and ranked[1][0] - ranked[0][0]
                    <= self.config.ambiguity_margin):
            if diagnostics is not None:
                diagnostics["outcome"] = "ambiguous"
                diagnostics["ambiguity_margin"] = float(
                    ranked[1][0] - ranked[0][0])
            if not diagnostic_mode:
                self.previous = None
                self._lane_change_evidence = None
            return None
        best = ranked[0]
        chosen = best
        reason = "best_score" if previous is None else "better_lane"
        if previous is not None:
            old = next((item for item in ranked
                        if item[1].lane_id == previous.lane_id), None)
            adjacent_change = bool(
                best[1].lane_id != previous.lane_id
                and self._adjacent_road_lane_change(
                    previous.lane_id, best[1].lane_id) is not None)
            lane_change_confirmed = bool(
                adjacent_change and self._confirm_lane_change(
                    previous, best, raw_projections, (px, py, pz),
                    diagnostic_mode))
            if adjacent_change and not lane_change_confirmed:
                if old is None:
                    if diagnostics is not None:
                        diagnostics["outcome"] = "lane_change_pending"
                    # Preserve the last confirmed match internally so the next
                    # sample can prove or reject the crossing. Consumers see no
                    # match and therefore publish steering=0/nav_active=False.
                    return None
                chosen = old
                reason = "lane_change_pending"
            elif lane_change_confirmed:
                chosen = best
                reason = "lane_change_confirmed"
            elif old is not None and old[0] <= chosen[0] + self.config.switch_margin:
                chosen = old
                reason = "hysteresis_hold"
            elif chosen[1].lane_id == previous.lane_id:
                reason = "same_lane"
            elif self.network.lanes_connected(previous.lane_id,
                                              chosen[1].lane_id):
                reason = "topology_transition"
            if not adjacent_change and not diagnostic_mode:
                self._lane_change_evidence = None
        score, lane, point, segment_index, point_index, signed, vertical, _, confidence, components = chosen
        match = LaneMatch(lane.lane_id, point, segment_index, point_index,
                          signed, vertical, wrap_angle(heading - point.heading),
                          score, confidence, reason, components)
        if diagnostics is not None:
            diagnostics["outcome"] = "matched"
            diagnostics["selected_lane_id"] = {
                "road_uid": int(lane.lane_id.road_uid),
                "direction": int(lane.lane_id.direction),
                "lane_index": int(lane.lane_id.lane_index),
                "prefab_token": lane.lane_id.prefab_token,
                "connector_index": lane.lane_id.connector_index,
                "connector_path": list(lane.lane_id.connector_path),
            }
        if not diagnostic_mode:
            self.previous = match
        return match
