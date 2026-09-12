"""Read-only Phase 5B2 audit of map sources that might resemble boundaries.

The output deliberately distinguishes display/lane-placement observations from
independently confirmed drivable/collision geometry. It never emits an
authoritative surface catalog and never mutates map data.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path


DISPLAY_ONLY_KEYS = {
    'mapPoints': 'PPD world-map/GPS visualization points',
    'laneOffset': 'road-look lane placement offset',
    'shoulderSpaces': 'road-look shoulder placement/rendering data',
}
BOUNDARY_KEY_WORDS = ('collision', 'boundary', 'drivable', 'driveable')


def load(path):
    with path.open('r', encoding='utf-8') as stream:
        return json.load(stream)


def nested_boundary_keys(value, found=None):
    found = found if found is not None else Counter()
    if isinstance(value, dict):
        for key, child in value.items():
            if any(word in str(key).lower() for word in BOUNDARY_KEY_WORDS):
                found[str(key)] += 1
            nested_boundary_keys(child, found)
    elif isinstance(value, list):
        for child in value:
            nested_boundary_keys(child, found)
    return found


def audit(dataset):
    prefabs_path = dataset / 'europe-prefabDescriptions.json'
    looks_path = dataset / 'europe-roadLooks.json'
    prefabs, looks = load(prefabs_path), load(looks_path)

    descriptions = prefabs.values() if isinstance(prefabs, dict) else prefabs
    descriptions = list(descriptions)
    road_looks = looks.values() if isinstance(looks, dict) else looks
    road_looks = list(road_looks)
    point_types, polygon_colours = Counter(), Counter()
    no_points = road_only = with_polygon = 0
    mod63 = []
    for description in descriptions:
        points = description.get('mapPoints', ()) or ()
        if not points:
            no_points += 1
        types = [str(point.get('type', '')) for point in points
                 if isinstance(point, dict)]
        point_types.update(types)
        road_only += bool(types) and set(types) == {'road'}
        with_polygon += 'polygon' in types
        polygon_colours.update(str(point.get('color', 'missing')) for point in points
            if isinstance(point, dict) and point.get('type') == 'polygon')
        if description.get('token') == 'mod_ger_63':
            mod63.append({
                'token': description.get('token'),
                'path': description.get('path'),
                'map_point_count': len(points),
                'map_point_types': types,
                'polygon_count': types.count('polygon'),
            })

    explicit_width_keys = Counter()
    for look in road_looks:
        for key in look:
            if 'width' in str(key).lower():
                explicit_width_keys[str(key)] += 1

    boundary_keys = nested_boundary_keys(descriptions)
    boundary_keys.update(nested_boundary_keys(road_looks))
    return {
        'schema_version': 1,
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'dataset': str(dataset.resolve()),
        'input_files': {
            prefabs_path.name: prefabs_path.stat().st_mtime_ns,
            looks_path.name: looks_path.stat().st_mtime_ns,
        },
        'prefab_descriptions': len(descriptions),
        'road_looks': len(road_looks),
        'map_points': sum(point_types.values()),
        'map_point_types': dict(sorted(point_types.items())),
        'polygon_colours': dict(sorted(polygon_colours.items())),
        'prefabs_without_map_points': no_points,
        'prefabs_with_road_points_only': road_only,
        'prefabs_with_polygon_points': with_polygon,
        'road_look_explicit_width_keys': dict(explicit_width_keys),
        'candidate_boundary_named_keys': dict(boundary_keys),
        'mod_ger_63': mod63,
        'classifications': {
            'ppd_map_points': 'DISPLAY_ONLY_NOT_CONTROL_AUTHORITY',
            'road_look_lane_offset': 'LANE_PLACEMENT_NOT_BOUNDARY',
            'road_look_shoulder_spaces': 'PLACEMENT_NOT_CONFIRMED_BOUNDARY',
            'derived_lane_width': 'DERIVED_NOT_BOUNDARY',
        },
        'authoritative_boundary_available': False,
        'failure_reason': 'MISSING_CONFIRMED_DRIVABLE_BOUNDARY',
        'required_next_evidence': [
            'SCS collision boundary export with documented coordinate transform',
            'or independent drivable-surface survey with bounded uncertainty',
            'exact intent/revision/build/session/map/dataset/LanePath binding',
        ],
        'official_semantics': [
            'https://modding.scssoft.com/wiki/Documentation/Tools/'
            'SCS_Blender_Tools/Locators/Prefab_Locators',
            'https://modding.scssoft.com/wiki/Documentation/Engine/Game_data',
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=Path,
                        default=Path('map-cache/promods-1.59'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = audit(args.dataset)
    rendered = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + '\n', encoding='utf-8')
    print(rendered)


if __name__ == '__main__':
    main()
