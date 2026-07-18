"""Safe segment-wise lane trajectory construction and validation.

This module never invents topology. It only resamples a valid ``LanePath`` and
applies a small corridor-bounded fairing inside ordinary road segments. Prefab
curves and every segment boundary remain authoritative.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Optional, Sequence

from core.navigation.lane_model import LanePath, LanePoint, LaneSegment, wrap_angle


@dataclass(frozen=True, slots=True)
class TrajectoryValidation:
    valid: bool
    failure_reason: str = ""
    input_points: int = 0
    output_points: int = 0
    original_length_m: float = 0.0
    result_length_m: float = 0.0
    min_spacing_m: float = 0.0
    average_spacing_m: float = 0.0
    max_spacing_m: float = 0.0
    max_heading_jump_deg: float = 0.0
    max_curvature: float = 0.0
    max_curvature_jump: float = 0.0
    max_corridor_deviation_m: float = 0.0
    max_height_jump_m: float = 0.0
    self_intersections: int = 0


MAX_CONTROL_GAP_M = 3.25
MIN_CONTROL_SPACING_M = 0.20
MAX_HEADING_JUMP_DEG = 38.0
MAX_CURVATURE = 0.55
MAX_CURVATURE_JUMP = 0.35
MAX_HEIGHT_JUMP_M = 1.50
MAX_LENGTH_CHANGE_RATIO = 0.035


def _xyz(point):
    return (point.x, point.y, point.z)


def _finite_point(point):
    return all(math.isfinite(float(value)) for value in
               (point.x, point.y, point.z, point.s,
                point.heading, point.curvature))


def _polyline_length(points):
    return sum(math.dist(_xyz(a), _xyz(b))
               for a, b in zip(points, points[1:]))


def _point_segment_distance(point, first, second):
    vx, vy, vz = (second.x-first.x, second.y-first.y, second.z-first.z)
    length2 = vx*vx + vy*vy + vz*vz
    if length2 < 1e-10:
        return math.dist(_xyz(point), _xyz(first))
    t = max(0.0, min(1.0,
        ((point.x-first.x)*vx + (point.y-first.y)*vy
         + (point.z-first.z)*vz) / length2))
    projected = (first.x+vx*t, first.y+vy*t, first.z+vz*t)
    return math.dist(_xyz(point), projected)


def _distance_to_centerline(point, segment):
    return min((_point_segment_distance(point, first, second)
                for first, second in zip(segment.centerline,
                                         segment.centerline[1:])),
               default=float("inf"))


def _interpolate(first, second, fraction, lane_id=None, segment_index=-1):
    return LanePoint(
        first.x + (second.x-first.x)*fraction,
        first.y + (second.y-first.y)*fraction,
        first.z + (second.z-first.z)*fraction,
        lane_id=lane_id if lane_id is not None else first.lane_id,
        segment_index=segment_index if segment_index >= 0 else first.segment_index,
    )


def _resample_polyline(points, spacing_m, lane_id=None, segment_index=-1):
    points = tuple(points)
    if len(points) < 2:
        return points
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.dist(_xyz(first), _xyz(second)))
    total = cumulative[-1]
    if total < 1e-8:
        return (replace(points[0], lane_id=lane_id,
                        segment_index=segment_index),)
    interval_count = max(1, int(math.ceil(total / spacing_m)))
    targets = [total * index / interval_count
               for index in range(interval_count + 1)]
    result, edge = [], 0
    for target in targets:
        while edge + 1 < len(cumulative) and cumulative[edge+1] < target - 1e-9:
            edge += 1
        if edge + 1 >= len(points):
            point = points[-1]
        else:
            length = cumulative[edge+1] - cumulative[edge]
            fraction = 0.0 if length < 1e-10 else (
                (target-cumulative[edge]) / length)
            point = _interpolate(points[edge], points[edge+1], fraction,
                                 lane_id, segment_index)
        result.append(replace(point, lane_id=lane_id or point.lane_id,
                              segment_index=(segment_index if segment_index >= 0
                                             else point.segment_index)))
    # Endpoint preservation is an acceptance requirement, not an approximation.
    result[0] = replace(points[0], lane_id=lane_id or points[0].lane_id,
                        segment_index=(segment_index if segment_index >= 0
                                       else points[0].segment_index))
    result[-1] = replace(points[-1], lane_id=lane_id or points[-1].lane_id,
                         segment_index=(segment_index if segment_index >= 0
                                        else points[-1].segment_index))
    return tuple(result)


def _fair_ordinary_segment(segment):
    """Very small local fairing, accepted only inside the lane corridor."""
    points = tuple(segment.centerline)
    if (len(points) < 4 or segment.lane_type in
            ("prefab", "roundabout", "graph")):
        return points
    limit = min(0.75, segment.width_m * 0.20)
    result = [points[0]]
    for index in range(1, len(points)-1):
        previous, current, following = points[index-1:index+2]
        candidate = LanePoint(
            (previous.x + 6*current.x + following.x) / 8.0,
            (previous.y + 6*current.y + following.y) / 8.0,
            (previous.z + 6*current.z + following.z) / 8.0,
        )
        if _distance_to_centerline(candidate, segment) <= limit:
            result.append(candidate)
        else:
            result.append(current)
    result.append(points[-1])
    return tuple(result)


def _with_kinematics(points):
    points = tuple(points)
    if not points:
        return ()
    headings, cumulative = [], [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.dist(_xyz(first), _xyz(second)))
    for index, point in enumerate(points):
        before = points[max(0, index-1)]
        after = points[min(len(points)-1, index+1)]
        dx, dz = after.x-before.x, after.z-before.z
        heading = (math.atan2(-dx, -dz) if math.hypot(dx, dz) > 1e-9
                   else (headings[-1] if headings else point.heading))
        headings.append(heading)
    curvatures = []
    for index in range(len(points)):
        if index == 0 or index == len(points)-1:
            curvatures.append(0.0)
            continue
        distance = cumulative[index+1] - cumulative[index-1]
        change = wrap_angle(headings[index+1] - headings[index-1])
        curvatures.append(change / distance if distance > 1e-8 else 0.0)
    return tuple(replace(point, s=cumulative[index], heading=headings[index],
                         curvature=curvatures[index])
                 for index, point in enumerate(points))


def _segments_intersect_2d(a, b, c, d):
    def orient(p, q, r):
        return ((q.x-p.x)*(r.z-p.z) - (q.z-p.z)*(r.x-p.x))
    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    epsilon = 1e-7
    return ((o1 > epsilon and o2 < -epsilon or o1 < -epsilon and o2 > epsilon)
            and (o3 > epsilon and o4 < -epsilon or o3 < -epsilon and o4 > epsilon))


def _count_self_intersections(points):
    count = 0
    edges = list(zip(points, points[1:]))
    for first_index, (a, b) in enumerate(edges):
        for second_index in range(first_index + 3, len(edges)):
            c, d = edges[second_index]
            # A geometric crossing on another bridge deck is not a route
            # self-intersection in 3-D.
            if min(abs(a.y-c.y), abs(a.y-d.y),
                   abs(b.y-c.y), abs(b.y-d.y)) > 3.0:
                continue
            if _segments_intersect_2d(a, b, c, d):
                count += 1
    return count


def _invalid(path, reason):
    return LanePath(path.segments, (), path.source_gps_uids,
                    valid=False, failure_reason=reason,
                    revision=path.revision)


def _source_length(segments):
    total = sum(_polyline_length(segment.centerline) for segment in segments)
    for first, second in zip(segments, segments[1:]):
        total += math.dist(_xyz(first.centerline[-1]),
                           _xyz(second.centerline[0]))
    return total


def build_lane_trajectory(lane_path: LanePath, spacing_m: float = 2.0) -> LanePath:
    """Build a uniformly sampled, lane-bounded control trajectory."""
    if not isinstance(lane_path, LanePath) or not lane_path.valid:
        reason = getattr(lane_path, "failure_reason", "") or "input LanePath is invalid"
        return _invalid(lane_path, reason) if isinstance(lane_path, LanePath) else LanePath(
            (), (), valid=False, failure_reason=reason)
    if not (0.75 <= float(spacing_m) <= 3.0):
        return _invalid(lane_path,
                        f"control spacing {spacing_m!r} m is outside 0.75..3.0 m")
    if not lane_path.segments:
        return _invalid(lane_path, "input LanePath has no LaneSegments")
    if any(not _finite_point(point)
           for segment in lane_path.segments for point in segment.centerline):
        return _invalid(lane_path, "input LanePath contains non-finite geometry")
    for segment in lane_path.segments:
        if any(math.dist(_xyz(first), _xyz(second)) <= 1e-6
               for first, second in zip(segment.centerline,
                                        segment.centerline[1:])):
            return _invalid(lane_path,
                            f"LaneSegment {segment.lane_id} has duplicate points")
        vectors = [(second.x-first.x, second.z-first.z)
                   for first, second in zip(segment.centerline,
                                            segment.centerline[1:])]
        for first, second in zip(vectors, vectors[1:]):
            first_len, second_len = math.hypot(*first), math.hypot(*second)
            if (first_len > 1e-6 and second_len > 1e-6
                    and (first[0]*second[0] + first[1]*second[1])
                        / (first_len*second_len) < -0.5):
                return _invalid(lane_path,
                    f"LaneSegment {segment.lane_id} reverses direction")

    sampled_segments = []
    for segment_index, segment in enumerate(lane_path.segments):
        if len(segment.centerline) < 2:
            return _invalid(lane_path,
                            f"LaneSegment {segment.lane_id} has fewer than two points")
        fair = _fair_ordinary_segment(segment)
        sampled = _resample_polyline(fair, spacing_m, segment.lane_id,
                                     segment_index)
        sampled_segments.append(sampled)

    flattened = []
    for segment_index, (segment, sampled) in enumerate(
            zip(lane_path.segments, sampled_segments)):
        if segment_index:
            previous = lane_path.segments[segment_index-1]
            if not any(connection.target == segment.lane_id
                       for connection in previous.successors):
                return _invalid(lane_path,
                    f"unconfirmed topology {previous.lane_id} -> {segment.lane_id}")
            gap = math.dist(_xyz(flattened[-1]), _xyz(sampled[0]))
            if gap > 6.0:
                return _invalid(lane_path,
                    f"confirmed segment boundary has {gap:.2f} m gap at "
                    f"UID {segment.start_uid}")
            if gap > 0.25:
                interval_count = max(1, int(math.ceil(gap / spacing_m)))
                start, end = flattened[-1], sampled[0]
                for interval in range(1, interval_count):
                    fraction = interval/interval_count
                    owner = previous if fraction <= 0.5 else segment
                    owner_index = segment_index-1 if fraction <= 0.5 else segment_index
                    flattened.append(_interpolate(
                        start, end, fraction, owner.lane_id, owner_index))
        if flattened and math.dist(_xyz(flattened[-1]), _xyz(sampled[0])) <= 0.25:
            flattened.extend(sampled[1:])
        else:
            flattened.extend(sampled)

    points = _with_kinematics(flattened)
    result = LanePath(lane_path.segments, points, lane_path.source_gps_uids,
                      _polyline_length(points), lane_path.confidence,
                      True, "", lane_path.revision)
    source_length = _source_length(lane_path.segments)
    if source_length > 1e-6:
        length_ratio = abs(result.distance_m-source_length) / source_length
        if length_ratio > MAX_LENGTH_CHANGE_RATIO:
            return _invalid(result,
                f"trajectory length changed by {length_ratio*100:.2f}%")
    validation = validate_lane_trajectory(result)
    if not validation.valid:
        return _invalid(result, validation.failure_reason)
    return result


def validate_lane_trajectory(lane_path: LanePath) -> TrajectoryValidation:
    """Numerically validate geometry, topology, density and lane containment."""
    if not isinstance(lane_path, LanePath) or not lane_path.valid:
        return TrajectoryValidation(False,
            getattr(lane_path, "failure_reason", "") or "LanePath is invalid")
    points, segments = tuple(lane_path.points), tuple(lane_path.segments)
    if len(points) < 2 or not segments:
        return TrajectoryValidation(False, "trajectory has fewer than two points",
                                    output_points=len(points))
    for segment in segments:
        if any(not _finite_point(point) for point in segment.centerline):
            return TrajectoryValidation(False,
                f"LaneSegment {segment.lane_id} contains non-finite geometry")
    if any(not _finite_point(point) for point in points):
        return TrajectoryValidation(False, "trajectory contains non-finite geometry",
                                    output_points=len(points))

    for index, (first, second) in enumerate(zip(segments, segments[1:])):
        if first.end_uid != second.start_uid:
            return TrajectoryValidation(False,
                f"LaneSegment topology UID mismatch {first.end_uid} -> {second.start_uid}")
        if not any(connection.target == second.lane_id
                   for connection in first.successors):
            return TrajectoryValidation(False,
                f"missing LaneConnection {first.lane_id} -> {second.lane_id}")

    first_source, last_source = segments[0].centerline[0], segments[-1].centerline[-1]
    if math.dist(_xyz(points[0]), _xyz(first_source)) > 1e-6:
        return TrajectoryValidation(False, "trajectory changed the first point")
    if math.dist(_xyz(points[-1]), _xyz(last_source)) > 1e-6:
        return TrajectoryValidation(False, "trajectory changed the last point")

    spacings = [math.dist(_xyz(a), _xyz(b))
                for a, b in zip(points, points[1:])]
    min_spacing, max_spacing = min(spacings), max(spacings)
    average_spacing = sum(spacings) / len(spacings)
    heading_jumps = [abs(math.degrees(wrap_angle(b.heading-a.heading)))
                     for a, b in zip(points, points[1:])]
    curvature = [abs(point.curvature) for point in points]
    curvature_jumps = [abs(b.curvature-a.curvature)
                       for a, b in zip(points, points[1:])]
    height_jumps = [abs(b.y-a.y) for a, b in zip(points, points[1:])]
    deviations = []
    previous_segment_index = -1
    seen_segment_indices = set()
    for point in points:
        if not (0 <= point.segment_index < len(segments)):
            return TrajectoryValidation(False,
                f"point at s={point.s:.2f} m lost LaneSegment identity")
        segment = segments[point.segment_index]
        if (point.segment_index < previous_segment_index
                or point.segment_index > previous_segment_index + 1
                and previous_segment_index >= 0):
            return TrajectoryValidation(False,
                f"LaneSegment identity order jumps from {previous_segment_index} "
                f"to {point.segment_index}")
        previous_segment_index = point.segment_index
        seen_segment_indices.add(point.segment_index)
        if point.lane_id != segment.lane_id:
            return TrajectoryValidation(False,
                f"point at s={point.s:.2f} m has wrong lane identity")
        deviation = _distance_to_centerline(point, segment)
        deviations.append(deviation)
        if deviation > segment.width_m * 0.5 + 1e-6:
            return TrajectoryValidation(False,
                f"trajectory leaves lane {segment.lane_id} corridor by "
                f"{deviation:.2f} m")
    expected_indices = set(range(len(segments)))
    if seen_segment_indices != expected_indices:
        return TrajectoryValidation(False,
            f"trajectory omitted LaneSegment indices "
            f"{sorted(expected_indices-seen_segment_indices)}")
    max_deviation = max(deviations, default=0.0)
    self_intersections = _count_self_intersections(points)
    original_length = _source_length(segments)
    result_length = _polyline_length(points)

    metrics = dict(
        input_points=sum(len(segment.centerline) for segment in segments),
        output_points=len(points), original_length_m=original_length,
        result_length_m=result_length, min_spacing_m=min_spacing,
        average_spacing_m=average_spacing, max_spacing_m=max_spacing,
        max_heading_jump_deg=max(heading_jumps, default=0.0),
        max_curvature=max(curvature, default=0.0),
        max_curvature_jump=max(curvature_jumps, default=0.0),
        max_corridor_deviation_m=max_deviation,
        max_height_jump_m=max(height_jumps, default=0.0),
        self_intersections=self_intersections,
    )
    if max_spacing > MAX_CONTROL_GAP_M:
        return TrajectoryValidation(False,
            f"trajectory point gap {max_spacing:.2f} m exceeds {MAX_CONTROL_GAP_M:.2f} m",
            **metrics)
    dense = sum(spacing < MIN_CONTROL_SPACING_M for spacing in spacings)
    if dense:
        return TrajectoryValidation(False,
            f"trajectory contains {dense} over-dense gaps below "
            f"{MIN_CONTROL_SPACING_M:.2f} m", **metrics)
    if metrics["max_heading_jump_deg"] > MAX_HEADING_JUMP_DEG:
        return TrajectoryValidation(False,
            f"heading jump {metrics['max_heading_jump_deg']:.1f} deg exceeds "
            f"{MAX_HEADING_JUMP_DEG:.1f} deg", **metrics)
    if metrics["max_curvature"] > MAX_CURVATURE:
        return TrajectoryValidation(False,
            f"curvature {metrics['max_curvature']:.3f} 1/m exceeds "
            f"{MAX_CURVATURE:.3f} 1/m", **metrics)
    if metrics["max_curvature_jump"] > MAX_CURVATURE_JUMP:
        return TrajectoryValidation(False,
            f"curvature jump {metrics['max_curvature_jump']:.3f} 1/m exceeds "
            f"{MAX_CURVATURE_JUMP:.3f} 1/m", **metrics)
    if metrics["max_height_jump_m"] > MAX_HEIGHT_JUMP_M:
        return TrajectoryValidation(False,
            f"height jump {metrics['max_height_jump_m']:.2f} m exceeds "
            f"{MAX_HEIGHT_JUMP_M:.2f} m", **metrics)
    if original_length > 1e-6:
        ratio = abs(result_length-original_length) / original_length
        if ratio > MAX_LENGTH_CHANGE_RATIO:
            return TrajectoryValidation(False,
                f"trajectory length changed by {ratio*100:.2f}%", **metrics)
    if self_intersections:
        return TrajectoryValidation(False,
            f"trajectory has {self_intersections} self-intersection(s)", **metrics)
    return TrajectoryValidation(True, "", **metrics)


def derive_display_points(lane_path: LanePath,
                          spacing_m: float = 4.0) -> tuple[LanePoint, ...]:
    """Return a sparser representation of the identical authoritative polyline."""
    if not isinstance(lane_path, LanePath) or not lane_path.valid:
        return ()
    trajectory = lane_path
    if not trajectory.points or any(point.lane_id is None
                                    for point in trajectory.points):
        trajectory = build_lane_trajectory(lane_path)
    if not trajectory.valid or len(trajectory.points) < 2:
        return ()
    if float(spacing_m) < 2.0:
        return tuple(trajectory.points)
    # Preserve every authoritative LaneSegment boundary while thinning each
    # contiguous segment independently. No display chord can skip a junction.
    groups, current = [], []
    for point in trajectory.points:
        if current and point.segment_index != current[-1].segment_index:
            groups.append(tuple(current))
            current = []
        current.append(point)
    if current:
        groups.append(tuple(current))
    sampled = []
    for group in groups:
        part = _resample_polyline(group, float(spacing_m))
        if sampled and part and math.dist(_xyz(sampled[-1]), _xyz(part[0])) < 1e-8:
            sampled.extend(part[1:])
        else:
            sampled.extend(part)
    return _with_kinematics(sampled)
