"""Phase 5B2: fail-closed provenance for low-speed drivable surfaces.

Lane centrelines, derived 4.5 m widths, road-look shoulders and PPD Map Points
are useful navigation/rendering observations. They are not collision boundaries.
Only an independently confirmed, identity-bound XYZ boundary record can become
a 5A ``Surface``. This module neither plans a path nor publishes control output.

The 5A model is intentionally horizontal. Graded/banked surfaces remain
unsupported rather than being flattened or projected into an invented plane.
"""
from dataclasses import dataclass, replace
import hashlib
import json
import math

from core.navigation.lane_model import LaneId, LanePath
from core.navigation.lane_trajectory import validate_lane_trajectory
from core.swept_envelope import (
    EnvelopeError, EnvelopeResult, Identity, Ring, Surface, Vehicle,
    edges, evaluate, inside, point_segment,
)

COORDINATE_FRAME = "ETS2_WORLD_XYZ"
SUPPORTED_METHODS = frozenset((
    "scs_collision_boundary_export_v1",
    "independent_drivable_surface_survey_v1",
))
FORBIDDEN_METHODS = frozenset((
    "centerline_buffer", "lane_offset", "road_look_width", "hud_ribbon",
    "live_map_polygon", "ppd_map_points", "visual_mesh", "free_interpolation",
))
FORBIDDEN_SOURCE_TERMS = (
    'ppd map point', 'ppd_map_point', 'centerline buffer',
    'centreline buffer', 'lane offset', 'road look width', 'road-look width',
    'hud ribbon', 'live map', 'visual mesh', 'free interpolation',
)


def require(condition, reason):
    if not condition:
        raise EnvelopeError(reason)


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def digest(value):
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(',', ':'), allow_nan=False,
    ).encode('utf-8')).hexdigest()


def lane_id_value(lane_id):
    require(isinstance(lane_id, LaneId), 'INVALID_SURFACE_LANE_IDENTITY')
    return (lane_id.road_uid, lane_id.direction, lane_id.lane_index,
            lane_id.prefab_token, lane_id.connector_index,
            tuple(lane_id.connector_path))


def identity_fingerprint(identity):
    identity.validate()
    return digest({
        'intent': identity.intent, 'revision': identity.revision,
        'build': identity.build, 'session': identity.session,
        'map_key': identity.map_key, 'dataset': identity.dataset,
        'layer': identity.layer,
        'lanes': [lane_id_value(value) for value in identity.lanes],
        'gps_pairs': identity.gps_pairs,
    })


def lane_path_fingerprint(lane_path):
    require(isinstance(lane_path, LanePath) and lane_path.valid,
            'INVALID_LANE_PATH_FOR_SURFACE')
    return digest({
        'revision': lane_path.revision,
        'source_gps_uids': tuple(lane_path.source_gps_uids),
        'expected_first_gps_pair_index': lane_path.expected_first_gps_pair_index,
        'trajectory_points': [
            (point.x, point.y, point.z, point.s, point.heading,
             point.curvature, lane_id_value(point.lane_id),
             point.segment_index)
            for point in lane_path.points
        ],
        'segments': [{
            'lane_id': lane_id_value(segment.lane_id),
            'start_uid': segment.start_uid, 'end_uid': segment.end_uid,
            'direction': segment.direction,
            'gps_pair_index': segment.gps_pair_index,
            'elevation_layer': segment.elevation_layer,
            'lane_type': segment.lane_type,
            'width_m': segment.width_m, 'width_source': segment.width_source,
            'points': [(p.x, p.y, p.z, p.heading) for p in segment.centerline],
        } for segment in lane_path.segments],
    })


def validate_path_identity(lane_path, identity, expected_identity):
    require(isinstance(identity, Identity) and identity == expected_identity,
            'STALE_SURFACE_IDENTITY')
    identity.validate()
    validation = validate_lane_trajectory(lane_path)
    require(validation.valid, 'INVALID_LANE_PATH_FOR_SURFACE')
    require(lane_path.revision == identity.revision,
            'SURFACE_REVISION_MISMATCH')
    require(tuple(segment.lane_id for segment in lane_path.segments)
            == identity.lanes, 'SURFACE_LANE_SEQUENCE_MISMATCH')
    require(tuple((segment.start_uid, segment.end_uid)
                  for segment in lane_path.segments) == identity.gps_pairs,
            'SURFACE_GPS_PAIR_SEQUENCE_MISMATCH')


@dataclass(frozen=True)
class SegmentBoundaryObservation:
    lane_id: LaneId
    gps_pair: tuple[int, int]
    gps_pair_index: int
    elevation_layer: int
    lane_type: str
    observation_source: str
    failure_reason: str
    display_polygon_count: int = 0
    map_point_count: int = 0


@dataclass(frozen=True)
class SurfaceSourceAudit:
    schema_version: int
    identity_fingerprint: str
    lane_path_fingerprint: str
    available: bool
    failure_reason: str
    segments: tuple[SegmentBoundaryObservation, ...]


def audit_lane_path_sources(network, lane_path, identity, expected_identity):
    """Classify current map sources without promoting display geometry."""
    try:
        validate_path_identity(lane_path, identity, expected_identity)
        rows = []
        for segment in lane_path.segments:
            token = segment.lane_id.prefab_token
            if token:
                data = network._prefab_lane_data.get(token, {})
                display_count = len(network._prefab_map_polygons.get(token, ()))
                source = str(data.get('surface_source',
                                     'ppd_map_points_visual_only'))
                reason = str(data.get('surface_failure_reason') or
                             ('PREFAB_MAP_POINTS_ARE_DISPLAY_ONLY'
                              if display_count else
                              'PREFAB_HAS_NO_BOUNDARY_GEOMETRY'))
                count = int(data.get('map_point_count', 0) or 0)
            elif segment.lane_type == 'graph':
                source, reason, display_count, count = (
                    'graph_topology_only', 'GRAPH_EDGE_HAS_NO_BOUNDARY_GEOMETRY',
                    0, 0)
            else:
                look = network.road_looks.get(segment.road_look_token, {})
                # Shoulder spacing and lane counts describe placement/rendering;
                # width remains explicitly "derived" in LaneSegment.
                source = ('road_look_lane_placement_only' if look
                          else 'road_centerline_only')
                reason = ('ROAD_LOOK_HAS_NO_EXPLICIT_DRIVABLE_BOUNDARY'
                          if look else 'ROAD_HAS_NO_BOUNDARY_GEOMETRY')
                display_count = count = 0
            rows.append(SegmentBoundaryObservation(
                segment.lane_id, (segment.start_uid, segment.end_uid),
                segment.gps_pair_index, segment.elevation_layer,
                segment.lane_type, source, reason, display_count, count))
        reason = next((row.failure_reason for row in rows if row.failure_reason),
                      'MISSING_CONFIRMED_DRIVABLE_BOUNDARY')
        return SurfaceSourceAudit(1, identity_fingerprint(identity),
            lane_path_fingerprint(lane_path), False, reason, tuple(rows))
    except EnvelopeError as error:
        return SurfaceSourceAudit(1, '', '', False, str(error), ())
    except (AttributeError, TypeError, ValueError, OverflowError):
        return SurfaceSourceAudit(1, '', '', False,
                                  'MALFORMED_SURFACE_SOURCE_INPUT', ())


@dataclass(frozen=True)
class DrivableSurfaceSnapshot:
    schema_version: int
    token: str
    identity_fingerprint: str
    lane_path_fingerprint: str
    boundary_uncertainty_m: float
    evidence_method: str
    evidence_sha256: str
    source: str
    surface: Surface | None
    failure_reason: str


def failed_snapshot(reason):
    return DrivableSurfaceSnapshot(1, '', '', '', 0., '', '', '', None,
                                   reason)


class DrivableSurfaceCatalog:
    """Immutable resolver for independently produced surface evidence.

    Loading a record does not grant driving authority. ``resolve`` compiles it
    through all identity, path, coordinate-frame and coverage checks on every
    requested LanePath. Duplicate records are ambiguous and fail closed.
    """

    def __init__(self, document):
        self._records = {}
        self.failure_reason = ''
        try:
            require(type(document) is dict
                    and type(document.get('schema_version')) is int
                    and document['schema_version'] == 1,
                    'INVALID_DRIVABLE_SURFACE_CATALOG_SCHEMA')
            records = document.get('surfaces')
            require(type(records) is list and len(records) <= 10_000,
                    'INVALID_DRIVABLE_SURFACE_CATALOG')
            for record in records:
                require(type(record) is dict,
                        'MALFORMED_DRIVABLE_SURFACE_RECORD')
                identity_hash = record.get('identity_fingerprint')
                path_hash = record.get('lane_path_fingerprint')
                require(_sha256(identity_hash) and _sha256(path_hash),
                        'INVALID_SURFACE_CATALOG_KEY')
                key = (identity_hash, path_hash)
                require(key not in self._records,
                        'AMBIGUOUS_DRIVABLE_SURFACE')
                # JSON round-trip prevents the caller mutating nested rings
                # after catalog validation.
                self._records[key] = json.loads(json.dumps(
                    record, allow_nan=False))
        except EnvelopeError as error:
            self._records = {}
            self.failure_reason = str(error)
        except (TypeError, ValueError, OverflowError, KeyError):
            self._records = {}
            self.failure_reason = 'MALFORMED_DRIVABLE_SURFACE_CATALOG'

    def resolve(self, lane_path, identity, expected_identity):
        if self.failure_reason:
            return failed_snapshot(self.failure_reason)
        try:
            validate_path_identity(lane_path, identity, expected_identity)
            key = (identity_fingerprint(identity),
                   lane_path_fingerprint(lane_path))
        except EnvelopeError as error:
            return failed_snapshot(str(error))
        except (TypeError, ValueError, OverflowError):
            return failed_snapshot('MALFORMED_SURFACE_SOURCE_INPUT')
        record = self._records.get(key)
        if record is None:
            return failed_snapshot('MISSING_CONFIRMED_DRIVABLE_BOUNDARY')
        return build_confirmed_surface(record, lane_path, identity,
                                       expected_identity)


def _sha256(value):
    return (type(value) is str and len(value) == 64
            and all(char in '0123456789abcdef' for char in value))


def _xyz_ring(raw, reason):
    require(type(raw) is list and 3 <= len(raw) <= 2048, reason)
    ring = []
    for point in raw:
        require(type(point) in (list, tuple) and len(point) == 3
                and all(number(value) for value in point), reason)
        x, y, z = map(float, point)
        require(max(abs(x), abs(y), abs(z)) <= 1_000_000, reason)
        ring.append((x, y, z))
    return tuple(ring)


def build_confirmed_surface(record, lane_path, identity, expected_identity):
    """Compile one externally evidenced local maneuver surface.

    A record is exact to this LanePath geometry and identity. Merely setting
    ``confirmed`` cannot make a known display/offset derivation acceptable.
    """
    try:
        validate_path_identity(lane_path, identity, expected_identity)
        require(type(record) is dict and type(record.get('schema_version')) is int
                and record['schema_version'] == 1,
                'INVALID_DRIVABLE_SURFACE_SCHEMA')
        method = record.get('method')
        require(method not in FORBIDDEN_METHODS, 'UNPROVEN_DRIVABLE_BOUNDARY_SOURCE')
        require(method in SUPPORTED_METHODS, 'UNSUPPORTED_DRIVABLE_BOUNDARY_SOURCE')
        source, evidence = record.get('source'), record.get('evidence_sha256')
        require(type(source) is str and bool(source.strip())
                and type(evidence) is str and len(evidence) == 64
                and all(value in '0123456789abcdef' for value in evidence)
                and record.get('confirmed') is True,
                'UNPROVEN_DRIVABLE_BOUNDARY')
        require(not any(term in source.casefold()
                        for term in FORBIDDEN_SOURCE_TERMS),
                'UNPROVEN_DRIVABLE_BOUNDARY_SOURCE')
        require(record.get('coordinate_frame') == COORDINATE_FRAME,
                'UNSUPPORTED_SURFACE_COORDINATE_FRAME')
        require(record.get('horizontal') is True,
                'UNSUPPORTED_SURFACE_GRADE')
        layers = {segment.elevation_layer for segment in lane_path.segments}
        require(len(layers) == 1, 'SURFACE_SPANS_MULTIPLE_ELEVATION_LAYERS')
        record_layer = record.get('elevation_layer')
        require(type(record_layer) is int and record_layer == next(iter(layers)),
                'SURFACE_ELEVATION_LAYER_MISMATCH')
        identity_hash = identity_fingerprint(identity)
        path_hash = lane_path_fingerprint(lane_path)
        require(record.get('identity_fingerprint') == identity_hash,
                'SURFACE_IDENTITY_FINGERPRINT_MISMATCH')
        require(record.get('lane_path_fingerprint') == path_hash,
                'SURFACE_LANE_PATH_FINGERPRINT_MISMATCH')
        uncertainty = record.get('boundary_uncertainty_m')
        require(number(uncertainty) and .001 <= uncertainty <= 2.,
                'INVALID_BOUNDARY_UNCERTAINTY')
        exterior_xyz = _xyz_ring(record.get('exterior_xyz'),
                                 'INVALID_EXTERIOR_BOUNDARY')
        raw_holes = record.get('holes_xyz', [])
        require(type(raw_holes) is list and len(raw_holes) <= 32,
                'INVALID_HOLE_BOUNDARIES')
        holes_xyz = tuple(_xyz_ring(raw, 'INVALID_HOLE_BOUNDARY')
                          for raw in raw_holes)
        heights = tuple(point[1] for ring in (exterior_xyz,)+holes_xyz
                        for point in ring)
        require(max(heights)-min(heights) <= .001,
                'UNSUPPORTED_SURFACE_GRADE')
        height = sum(heights)/len(heights)
        surface = Surface(identity,
            tuple((x, z) for x, _y, z in exterior_xyz),
            tuple(tuple((x, z) for x, _y, z in ring) for ring in holes_xyz),
            height, f'{source}:{evidence}', True, True)
        surface.validate()

        # Every authoritative source point, including first/final GPS pieces,
        # must be on this same deck and strictly inside the evidenced boundary.
        # This verifies coverage, not enough clearance for a vehicle; evaluate()
        # performs the latter using the complete swept envelope.
        all_points = (tuple(lane_path.points)
                      + tuple(point for segment in lane_path.segments
                              for point in segment.centerline))
        require(bool(all_points), 'EMPTY_SURFACE_CORRIDOR')
        require(all(abs(point.y-height) <= uncertainty for point in all_points),
                'SURFACE_ELEVATION_MISMATCH')
        for point in all_points:
            position = (point.x, point.z)
            require(inside(position, surface.exterior)
                    and not any(inside(position, hole) for hole in surface.holes),
                    'SURFACE_DOES_NOT_COVER_LANE_PATH')
            distance = min(point_segment(position, a, b)
                           for ring in (surface.exterior,)+surface.holes
                           for a, b in edges(ring))
            require(distance > uncertainty,
                    'LANE_PATH_INSIDE_BOUNDARY_UNCERTAINTY')
        canonical = dict(record)
        token = digest({'record': canonical, 'identity': identity_hash,
                        'lane_path': path_hash})
        return DrivableSurfaceSnapshot(1, token, identity_hash, path_hash,
            float(uncertainty), method, evidence, source, surface, '')
    except EnvelopeError as error:
        return failed_snapshot(str(error))
    except (AttributeError, TypeError, ValueError, OverflowError, KeyError):
        return failed_snapshot('MALFORMED_DRIVABLE_SURFACE_RECORD')


def evaluate_on_confirmed_surface(vehicle, frames, snapshot, lane_path,
                                  expected_identity):
    """Evaluate only against the still-current, exactly bound LanePath.

    The current path is deliberately supplied again at consumption time.  A
    revision token alone cannot prove that an asynchronous callback did not
    carry geometry from an older build bearing the same revision number.
    Boundary uncertainty is then applied exactly once before the 5A model.
    """
    if not isinstance(snapshot, DrivableSurfaceSnapshot):
        return EnvelopeResult(False, 'MISSING_CONFIRMED_DRIVABLE_BOUNDARY',
                              expected_identity)
    if snapshot.failure_reason or snapshot.surface is None:
        return EnvelopeResult(False, snapshot.failure_reason or
                              'MISSING_CONFIRMED_DRIVABLE_BOUNDARY',
                              expected_identity)
    if snapshot.surface.identity != expected_identity:
        return EnvelopeResult(False, 'STALE_SURFACE_IDENTITY', expected_identity)
    try:
        validate_path_identity(lane_path, snapshot.surface.identity,
                               expected_identity)
        require(snapshot.identity_fingerprint ==
                identity_fingerprint(expected_identity),
                'STALE_SURFACE_IDENTITY_FINGERPRINT')
        require(snapshot.lane_path_fingerprint ==
                lane_path_fingerprint(lane_path),
                'STALE_SURFACE_LANE_PATH')
        require(isinstance(vehicle, Vehicle), 'INVALID_VEHICLE_MODEL')
        adjusted = replace(vehicle,
            uncertainty_m=vehicle.uncertainty_m+snapshot.boundary_uncertainty_m)
        adjusted.validate()
    except EnvelopeError as error:
        return EnvelopeResult(False, str(error), expected_identity)
    except (TypeError, ValueError, OverflowError):
        return EnvelopeResult(False, 'INVALID_COMBINED_GEOMETRY_UNCERTAINTY',
                              expected_identity)
    return evaluate(adjusted, frames, snapshot.surface, expected_identity)
