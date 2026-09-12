"""Phase 5B2 provenance, identity and fail-closed surface regressions."""
from dataclasses import replace
import copy
import hashlib
import json
import math
from pathlib import Path
import unittest
from unittest import mock

import core.navigation.road_network as road_network_module
from core.navigation.drivable_surface import (
    COORDINATE_FRAME, DrivableSurfaceCatalog, audit_lane_path_sources,
    build_confirmed_surface, evaluate_on_confirmed_surface,
    identity_fingerprint, lane_path_fingerprint,
)
from core.navigation.lane_model import (
    LaneConnection, LaneId, LanePath, LanePoint, LaneSegment,
)
from core.navigation.lane_trajectory import build_lane_trajectory
from core.navigation.road_network import RoadNetwork
from core.settings.manager import SettingsManager
from core.swept_envelope import Body, Frame, Identity, Pose, Vehicle, evaluate
from tools.run_stage5b2_surface_audit import audit as audit_dataset


def identity():
    lanes = (
        LaneId(10, 1, 0),
        LaneId(20, 1, 0, 'mod_ger_63', 13, (13, 29, 17, 25)),
        LaneId(30, 1, 0),
    )
    return Identity('intent', 8, 'build', 'session', 'promods-1.59',
                    'fingerprint', 'elevation:6', lanes,
                    ((1, 2), (2, 3), (3, 4)))


def lane_path(x=0., y=18.):
    ident = identity()
    segments = []
    for index, lane_id in enumerate(ident.lanes):
        z0, z1 = index * 10., (index + 1) * 10.
        points = tuple(LanePoint(x, y, z, heading=math.pi)
                       for z in (z0, z0 + 5., z1))
        successor = (() if index == len(ident.lanes)-1 else
                     (LaneConnection(ident.lanes[index+1], 'prefab'),))
        segments.append(LaneSegment(
            lane_id, index+1, index+2, 1, 0, 1, 4.5, 'derived', 6,
            'road-look' if index != 1 else None,
            'prefab' if index == 1 else 'road', points,
            successors=successor, gps_uids=frozenset((index+1, index+2)),
            raw_lane_index=0, gps_pair_index=index,
            connector_curve_indices=((13, 29, 17, 25) if index == 1 else ())))
    source = LanePath(tuple(segments), (), (1, 2, 3, 4), 30., .99, True,
                      revision=8)
    result = build_lane_trajectory(source)
    if not result.valid:
        raise AssertionError(result.failure_reason)
    return result


def record(path=None, ident=None, half_width=4., y=18., uncertainty=.05):
    path, ident = path or lane_path(), ident or identity()
    value = {
        'schema_version': 1,
        'method': 'independent_drivable_surface_survey_v1',
        'source': 'survey fixture with traceable control points',
        'evidence_sha256': hashlib.sha256(b'fixture evidence').hexdigest(),
        'confirmed': True,
        'coordinate_frame': COORDINATE_FRAME,
        'horizontal': True,
        'elevation_layer': 6,
        'identity_fingerprint': identity_fingerprint(ident),
        'lane_path_fingerprint': lane_path_fingerprint(path),
        'boundary_uncertainty_m': uncertainty,
        'exterior_xyz': [
            [-half_width, y, -8.], [half_width, y, -8.],
            [half_width, y, 38.], [-half_width, y, 38.],
        ],
        'holes_xyz': [],
    }
    return value


def vehicle():
    body = Body('tractor', 2.5, 5., 1.2, 0., 0., 'fixture', True,
                'fixed_axle')
    return Vehicle((body,), 3.8, .70, 2., .15, .02)


def frame(ident=None, x=0., y=18., z=10.):
    return Frame(0., ident or identity(), (Pose(x, y, z, math.pi),))


class NetworkFixture:
    def __init__(self):
        self._prefab_lane_data = {
            'mod_ger_63': {
                'surface_source': 'ppd_map_points_visual_only',
                'surface_failure_reason': 'PREFAB_MAP_POINTS_ARE_DISPLAY_ONLY',
                'map_point_count': 5,
            },
        }
        self._prefab_map_polygons = {'mod_ger_63': ()}
        self.road_looks = {'road-look': {'lanesLeft': (), 'lanesRight': ()}}


class Stage5B2DrivableSurfaceTests(unittest.TestCase):
    def assert_rejected(self, changed, reason):
        path, ident = lane_path(), identity()
        raw = record(path, ident)
        changed(raw)
        result = build_confirmed_surface(raw, path, ident, ident)
        self.assertEqual(result.failure_reason, reason)

    def test_current_map_sources_are_observations_not_authority(self):
        path, ident = lane_path(), identity()
        audit = audit_lane_path_sources(NetworkFixture(), path, ident, ident)
        self.assertFalse(audit.available)
        self.assertEqual([row.observation_source for row in audit.segments], [
            'road_look_lane_placement_only',
            'ppd_map_points_visual_only',
            'road_look_lane_placement_only',
        ])
        self.assertEqual(audit.segments[1].failure_reason,
                         'PREFAB_MAP_POINTS_ARE_DISPLAY_ONLY')

    def test_road_network_labels_loaded_ppd_points_display_only(self):
        description = {
            'token': 'visual-prefab', 'path': 'fixture.ppd',
            'nodes': [], 'navCurves': [], 'navNodes': [],
            'mapPoints': [
                {'x': 0., 'y': 0., 'z': 2., 'type': 'road',
                 'neighbors': []},
                {'x': 1., 'y': 0., 'z': 2., 'type': 'polygon',
                 'neighbors': []},
            ],
        }
        network = RoadNetwork()
        with (mock.patch.object(road_network_module, '_find_json',
                                side_effect=lambda _directory, name: name),
              mock.patch.object(road_network_module, '_loadf',
                                side_effect=lambda path: ([description] if
                                    path == 'prefabDescriptions' else []))):
            network._load_prefabs('unused')
        data = network._prefab_lane_data['visual-prefab']
        self.assertEqual(data['surface_source'],
                         'ppd_map_points_visual_only')
        self.assertEqual(data['surface_failure_reason'],
                         'PREFAB_MAP_POINTS_ARE_DISPLAY_ONLY')
        self.assertEqual(data['road_map_point_count'], 1)
        self.assertEqual(data['polygon_map_point_count'], 1)

    def test_default_surface_catalog_is_empty_and_fail_closed(self):
        document = SettingsManager.__new__(SettingsManager)._get_defaults()[
            'drivable_surfaces']
        path, ident = lane_path(), identity()
        self.assertEqual(DrivableSurfaceCatalog(document).resolve(
            path, ident, ident).failure_reason,
            'MISSING_CONFIRMED_DRIVABLE_BOUNDARY')

    def test_promods_159_and_mod_ger_63_do_not_claim_boundaries(self):
        dataset = Path(__file__).parents[1] / 'map-cache' / 'promods-1.59'
        if not dataset.exists():
            self.skipTest('promods-1.59 fixture is unavailable')
        result = audit_dataset(dataset)
        self.assertFalse(result['authoritative_boundary_available'])
        self.assertEqual(result['candidate_boundary_named_keys'], {})
        self.assertEqual(result['road_look_explicit_width_keys'], {})
        self.assertEqual(len(result['mod_ger_63']), 1)
        self.assertEqual(result['mod_ger_63'][0]['polygon_count'], 0)
        self.assertEqual(result['mod_ger_63'][0]['map_point_types'],
                         ['road'] * 5)

    def test_confirmed_road_prefab_road_surface_is_compiled(self):
        path, ident = lane_path(), identity()
        result = build_confirmed_surface(record(path, ident), path, ident, ident)
        self.assertEqual(result.failure_reason, '')
        self.assertIsNotNone(result.surface)
        self.assertEqual(result.surface.identity.lanes, ident.lanes)
        self.assertEqual(result.surface.identity.gps_pairs, ident.gps_pairs)
        self.assertEqual(result.lane_path_fingerprint,
                         lane_path_fingerprint(path))

    def test_missing_and_ambiguous_catalog_records_fail_closed(self):
        path, ident = lane_path(), identity()
        empty = DrivableSurfaceCatalog({'schema_version': 1, 'surfaces': []})
        self.assertEqual(empty.resolve(path, ident, ident).failure_reason,
                         'MISSING_CONFIRMED_DRIVABLE_BOUNDARY')
        raw = record(path, ident)
        duplicate = DrivableSurfaceCatalog({
            'schema_version': 1, 'surfaces': [raw, copy.deepcopy(raw)]})
        self.assertEqual(duplicate.resolve(path, ident, ident).failure_reason,
                         'AMBIGUOUS_DRIVABLE_SURFACE')

    def test_catalog_copies_records_and_resolves_only_exact_key(self):
        path, ident = lane_path(), identity()
        raw = record(path, ident)
        catalog = DrivableSurfaceCatalog({'schema_version': 1,
                                          'surfaces': [raw]})
        raw['exterior_xyz'][0][0] = 999.
        self.assertEqual(catalog.resolve(path, ident, ident).failure_reason, '')
        moved = lane_path(x=.01)
        self.assertEqual(catalog.resolve(moved, ident, ident).failure_reason,
                         'MISSING_CONFIRMED_DRIVABLE_BOUNDARY')

    def test_visual_and_derived_sources_are_never_promoted(self):
        for method in ('ppd_map_points', 'centerline_buffer', 'lane_offset',
                       'road_look_width', 'hud_ribbon', 'live_map_polygon',
                       'visual_mesh', 'free_interpolation'):
            with self.subTest(method=method):
                self.assert_rejected(
                    lambda raw, method=method: raw.__setitem__('method', method),
                    'UNPROVEN_DRIVABLE_BOUNDARY_SOURCE')
        self.assert_rejected(
            lambda raw: raw.__setitem__(
                'source', 'independent survey copied from PPD map points'),
            'UNPROVEN_DRIVABLE_BOUNDARY_SOURCE')

    def test_unconfirmed_unknown_and_bad_evidence_rejected(self):
        cases = (
            (lambda raw: raw.__setitem__('confirmed', False),
             'UNPROVEN_DRIVABLE_BOUNDARY'),
            (lambda raw: raw.__setitem__('method', 'some_future_guess'),
             'UNSUPPORTED_DRIVABLE_BOUNDARY_SOURCE'),
            (lambda raw: raw.__setitem__('evidence_sha256', 'bad'),
             'UNPROVEN_DRIVABLE_BOUNDARY'),
            (lambda raw: raw.__setitem__('source', ''),
             'UNPROVEN_DRIVABLE_BOUNDARY'),
        )
        for change, reason in cases:
            with self.subTest(reason=reason):
                self.assert_rejected(change, reason)

    def test_coordinate_frame_grade_and_elevation_layer_fail_closed(self):
        cases = (
            (lambda raw: raw.__setitem__('coordinate_frame', 'screen'),
             'UNSUPPORTED_SURFACE_COORDINATE_FRAME'),
            (lambda raw: raw.__setitem__('horizontal', False),
             'UNSUPPORTED_SURFACE_GRADE'),
            (lambda raw: raw.__setitem__('elevation_layer', 0),
             'SURFACE_ELEVATION_LAYER_MISMATCH'),
            (lambda raw: raw['exterior_xyz'][0].__setitem__(1, 18.01),
             'UNSUPPORTED_SURFACE_GRADE'),
        )
        for change, reason in cases:
            with self.subTest(reason=reason):
                self.assert_rejected(change, reason)

    def test_identity_revision_lane_gps_build_map_and_dataset_are_exact(self):
        path, ident = lane_path(), identity()
        for key, value in {
            'intent': 'other', 'revision': 9, 'build': 'other',
            'session': 'other', 'map_key': 'europe', 'dataset': 'other',
            'layer': 'elevation:0', 'lanes': tuple(reversed(ident.lanes)),
            'gps_pairs': ((1, 2), (3, 2), (3, 4)),
        }.items():
            with self.subTest(key=key):
                stale = replace(ident, **{key: value})
                result = build_confirmed_surface(record(path, ident), path,
                                                 ident, stale)
                self.assertEqual(result.failure_reason,
                                 'STALE_SURFACE_IDENTITY')

    def test_record_fingerprints_cannot_be_relabelled(self):
        cases = (
            (lambda raw: raw.__setitem__('identity_fingerprint', '0'*64),
             'SURFACE_IDENTITY_FINGERPRINT_MISMATCH'),
            (lambda raw: raw.__setitem__('lane_path_fingerprint', '0'*64),
             'SURFACE_LANE_PATH_FINGERPRINT_MISMATCH'),
        )
        for change, reason in cases:
            with self.subTest(reason=reason):
                self.assert_rejected(change, reason)

    def test_bridge_and_road_below_cannot_share_surface(self):
        path, ident = lane_path(), identity()
        raw = record(path, ident, y=0.)
        result = build_confirmed_surface(raw, path, ident, ident)
        self.assertEqual(result.failure_reason, 'SURFACE_ELEVATION_MISMATCH')

    def test_boundary_must_cover_complete_first_and_last_gps_segments(self):
        path, ident = lane_path(), identity()
        raw = record(path, ident)
        raw['exterior_xyz'][2][2] = 23.
        raw['exterior_xyz'][3][2] = 23.
        self.assertEqual(build_confirmed_surface(
            raw, path, ident, ident).failure_reason,
            'SURFACE_DOES_NOT_COVER_LANE_PATH')

    def test_hole_and_boundary_uncertainty_reject_centerline(self):
        path, ident = lane_path(), identity()
        raw = record(path, ident)
        raw['holes_xyz'] = [[[-1., 18., 14.], [1., 18., 14.],
                             [1., 18., 16.], [-1., 18., 16.]]]
        self.assertEqual(build_confirmed_surface(
            raw, path, ident, ident).failure_reason,
            'SURFACE_DOES_NOT_COVER_LANE_PATH')
        raw = record(path, ident, half_width=.04, uncertainty=.05)
        self.assertEqual(build_confirmed_surface(
            raw, path, ident, ident).failure_reason,
            'LANE_PATH_INSIDE_BOUNDARY_UNCERTAINTY')

    def test_malformed_self_intersecting_and_nonfinite_boundaries_rejected(self):
        cases = (
            lambda raw: raw.__setitem__('exterior_xyz', [[0., 18., 0.]]),
            lambda raw: raw.__setitem__('exterior_xyz', [
                [-4., 18., -8.], [4., 18., 38.],
                [-4., 18., 38.], [4., 18., -8.]]),
            lambda raw: raw['exterior_xyz'][0].__setitem__(0, float('nan')),
        )
        for change in cases:
            with self.subTest(change=change):
                path, ident = lane_path(), identity()
                raw = record(path, ident); change(raw)
                self.assertTrue(build_confirmed_surface(
                    raw, path, ident, ident).failure_reason)

    def test_uncertainty_range_rejects_bool_zero_and_unbounded(self):
        for value in (True, 0., .0009, 2.001, float('nan')):
            with self.subTest(value=value):
                self.assert_rejected(
                    lambda raw, value=value: raw.__setitem__(
                        'boundary_uncertainty_m', value),
                    'INVALID_BOUNDARY_UNCERTAINTY')

    def test_surface_uncertainty_is_added_once_to_complete_envelope(self):
        path, ident, v = lane_path(), identity(), vehicle()
        raw = record(path, ident, half_width=1.5, uncertainty=.10)
        snapshot = build_confirmed_surface(raw, path, ident, ident)
        self.assertFalse(snapshot.failure_reason)
        direct = evaluate(v, (frame(),), snapshot.surface, ident)
        combined = evaluate_on_confirmed_surface(
            v, (frame(),), snapshot, path, ident)
        self.assertTrue(direct.accepted)
        self.assertFalse(combined.accepted)
        self.assertEqual(combined.failure_reason,
                         'SWEPT_ENVELOPE_RESERVE_VIOLATION')

    def test_stale_geometry_is_rechecked_when_surface_is_consumed(self):
        path, ident, v = lane_path(), identity(), vehicle()
        snapshot = build_confirmed_surface(record(path, ident), path,
                                           ident, ident)
        stale_path = lane_path(x=.01)
        result = evaluate_on_confirmed_surface(
            v, (frame(),), snapshot, stale_path, ident)
        self.assertEqual(result.failure_reason, 'STALE_SURFACE_LANE_PATH')

    def test_trajectory_only_change_is_also_stale(self):
        path, ident, v = lane_path(), identity(), vehicle()
        snapshot = build_confirmed_surface(record(path, ident), path,
                                           ident, ident)
        points = list(path.points)
        points[len(points)//2] = replace(points[len(points)//2], x=.01)
        changed = replace(path, points=tuple(points))
        result = evaluate_on_confirmed_surface(
            v, (frame(),), snapshot, changed, ident)
        self.assertEqual(result.failure_reason, 'STALE_SURFACE_LANE_PATH')

    def test_stale_frame_cannot_use_current_surface(self):
        path, ident, v = lane_path(), identity(), vehicle()
        snapshot = build_confirmed_surface(record(path, ident), path,
                                           ident, ident)
        stale_frame = replace(frame(), identity=replace(ident, build='old'))
        result = evaluate_on_confirmed_surface(
            v, (stale_frame,), snapshot, path, ident)
        self.assertEqual(result.failure_reason, 'STALE_FRAME_IDENTITY')

    def test_valid_surface_and_frame_pass_to_5a_without_mutation(self):
        path, ident, v = lane_path(), identity(), vehicle()
        snapshot = build_confirmed_surface(record(path, ident), path,
                                           ident, ident)
        before = json.dumps(record(path, ident), sort_keys=True)
        result = evaluate_on_confirmed_surface(
            v, (frame(),), snapshot, path, ident)
        self.assertTrue(result.accepted, result.failure_reason)
        self.assertEqual(v.uncertainty_m, .02)
        self.assertEqual(before, json.dumps(record(path, ident), sort_keys=True))


if __name__ == '__main__':
    unittest.main()
