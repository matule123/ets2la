"""Read-only projection audit of frame-bound steering replays.

Two recorded (fraction, projection) observations identify an existing straight
LanePath edge exactly. No missing vertices/frames are interpolated. Edges with
inconsistent reconstructions are discarded. This is evidence extraction, not
a substitute map or a closed-loop replay of unrecorded geometry.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def identity(row):
    return tuple(row.get(key) for key in (
        "navigation_intent_id", "route_build_id", "revision",
        "source_game_session_id", "source_map_key",
        "source_dataset_fingerprint")) + (
            json.dumps(row.get("lane_id"), sort_keys=True),
            row.get("elevation_layer"))


def unique_frames(document):
    seen = set()
    rows = []
    for row in document["samples"]:
        key = identity(row), row.get("calculation_sdk_frame_us")
        if (key in seen or key[1] is None
                or not row.get("autopilot_active")
                or not row.get("packet_binding_valid")
                or row.get("tracking_segment_fraction") is None):
            continue
        seen.add(key)
        rows.append(row)
    return rows


def reconstruct_edges(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[identity(row), row["tracking_segment_index"]].append(row)
    edges = {}
    for key, group in groups.items():
        low = min(group, key=lambda row: row["tracking_segment_fraction"])
        high = max(group, key=lambda row: row["tracking_segment_fraction"])
        f0, f1 = (row["tracking_segment_fraction"] for row in (low, high))
        if f1 - f0 < 0.2:
            continue
        p0, p1 = (row["tracking_projection_xz"] for row in (low, high))
        delta = tuple((y-x)/(f1-f0) for x, y in zip(p0, p1))
        start = tuple(x-f0*d for x, d in zip(p0, delta))
        end = tuple(x+d for x, d in zip(start, delta))
        residual = max(math.dist(row["tracking_projection_xz"], tuple(
            x+row["tracking_segment_fraction"]*d
            for x, d in zip(start, delta))) for row in group)
        if residual > 1e-5:
            continue
        edges[key] = (start, end)
    return edges


def project(edge, position):
    start, end = edge
    delta = tuple(y-x for x, y in zip(start, end))
    length2 = sum(d*d for d in delta)
    fraction = max(0., min(1., sum((p-x)*d for p, x, d in zip(
        position, start, delta))/length2))
    point = tuple(x+fraction*d for x, d in zip(start, delta))
    return fraction, math.dist(position, point), point


def audit(document):
    rows = unique_frames(document)
    edges = reconstruct_edges(rows)
    events = []
    for row in rows:
        index = row["tracking_segment_index"]
        key = identity(row)
        fraction = row["tracking_segment_fraction"]
        if not (fraction < 1e-9 or fraction > 1-1e-9):
            continue
        step = 1 if fraction > .5 else -1
        selected, adjacent = edges.get((key, index)), edges.get((key, index+step))
        if selected is None or adjacent is None:
            continue
        shared = (selected[1], adjacent[0]) if step == 1 else (
            selected[0], adjacent[1])
        if math.dist(*shared) > 1e-5:
            continue
        position = row["observation_xz"]
        other_fraction, other_distance, other_point = project(adjacent, position)
        old_distance = math.dist(position, row["tracking_projection_xz"])
        if old_distance-other_distance <= 0.01 or not 0 < other_fraction < 1:
            continue
        events.append({
            "row": row, "selected_edge_xz": selected,
            "adjacent_edge_xz": adjacent, "adjacent_index": index+step,
            "adjacent_fraction": other_fraction,
            "selected_distance_m": old_distance,
            "adjacent_distance_m": other_distance,
            "projection_displacement_m": math.dist(
                row["tracking_projection_xz"], other_point),
        })
    events.sort(key=lambda event: event["projection_displacement_m"], reverse=True)
    return {
        "identity": document["identity"], "unique_calculation_frames": len(rows),
        "reconstructed_edges": len(edges), "proven_endpoint_bias_samples": len(events),
        "max_projection_displacement_m": max((e["projection_displacement_m"]
                                                for e in events), default=0.),
        "events": events,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replays", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--verify-ref", default="7f3d365")
    args = parser.parse_args()
    if not args.output.resolve().is_relative_to(ROOT):
        parser.error("output must stay in the repository")
    from core.navigation.route import Route
    from tools.run_steering_bench import factories_for_ref
    baseline, _, _ = factories_for_ref(args.verify_ref)
    results = []
    for source in args.replays:
        raw = source.read_bytes()
        result = audit(json.loads(raw))
        reproduced = corrected = 0
        for event in result['events']:
            row = event['row']
            forward = event['adjacent_index'] > row['tracking_segment_index']
            one, two = ((event['selected_edge_xz'], event['adjacent_edge_xz'])
                        if forward else
                        (event['adjacent_edge_xz'], event['selected_edge_xz']))
            points = [one[0], one[1], two[1]]
            old = baseline(points)._best_projection(range(2),
                row['observation_xz'], row['observation_heading_rad'])
            new = Route(points)._best_projection(range(2),
                row['observation_xz'], row['observation_heading_rad'])
            expected_old, expected_new = ((0, 1) if forward else (1, 0))
            original_reproduced = (old[2] == expected_old
                                  and abs(old[3]-row['tracking_segment_fraction']) < 1e-7)
            fixed = (new[2] == expected_new
                     and abs(new[3]-event['adjacent_fraction']) < 1e-7)
            event.update(baseline_reproduced=original_reproduced, corrected=fixed)
            reproduced += original_reproduced
            corrected += original_reproduced and fixed
        result.update(baseline_ref=args.verify_ref,
                      exact_baseline_reproductions=reproduced,
                      corrected_projections=corrected)
        result.update(source=str(source), sha256=hashlib.sha256(raw).hexdigest())
        results.append(result)
        print(json.dumps({key: value for key, value in result.items()
                          if key != "events"}, ensure_ascii=False))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2)+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
