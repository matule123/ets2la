"""Phase 5B1: one observational vehicle-profile owner, no driving authority.

SDK geometry is not body geometry. An exact *observable* configuration hash
selects a documented external profile, never a guessed truck-name default.
SDK lacks full accessory identity, channel validity and an atomic frame marker;
therefore a matched catalog entry alone never authorizes a live maneuver.
No consumer currently replaces steering/ACC/traffic geometry with this model.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import uuid

from core.sdk.vehicle_observation import VehicleObservation
from core.swept_envelope import Body, Vehicle, Identity, EnvelopeError


def require(condition, reason):
    if not condition:
        raise EnvelopeError(reason)


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode('utf-8')).hexdigest()


def configuration_fingerprint(observation):
    """Only SDK-observable configuration; NOT a checksum of installed mods."""
    articles = []
    for a in observation.articles:
        if not a.attached:
            continue
        articles.append({
            'slot': a.slot, 'id': a.vehicle_id, 'brand_id': a.brand_id,
            'body_type': a.body_type, 'chain_type': a.chain_type,
            'cargo_accessory_id': a.cargo_accessory_id, 'hook_m': a.hook_local_m,
            'wheels': [(w.index, w.position_m, w.radius_m, w.steerable,
                        w.simulated, w.powered, w.liftable) for w in a.wheels]})
    return digest({'schema': 1, 'source': observation.source,
                   'game_version': observation.game_version, 'articles': articles})


@dataclass(frozen=True)
class FixedAxleGeometry:
    axle_local_m: tuple[float, float, float]
    wheelbase_m: float | None
    hook_forward_m: float
    track_m: float  # observation ONLY, never body width


def fixed_axle_geometry(article):
    """Prove the narrow single-axle model, not an averaged tandem surrogate."""
    wheels = article.wheels
    require(wheels and all(w.simulated for w in wheels), 'UNPROVEN_SIMULATED_WHEELS')
    require(all(w.on_ground and w.lift == 0 and w.lift_offset_m == 0 for w in wheels),
            'UNSUPPORTED_LIFTED_OR_AIRBORNE_AXLE')
    rear = [w for w in wheels if not w.steerable]
    front = [w for w in wheels if w.steerable]
    require(bool(rear), 'MISSING_FIXED_AXLE')
    require(article.slot == -1 or not front, 'UNSUPPORTED_STEERED_TRAILER')

    def axle(group):
        require(len(group) >= 2, 'UNPROVEN_AXLE_PAIR')
        # 0.1 mm is float/config-coordinate tolerance, not a tandem grouping
        # distance. Real separate planes always require a proven new model.
        require(max(w.position_m[2] for w in group)-min(w.position_m[2] for w in group)
                <= .0001, 'UNPROVEN_MULTI_AXLE_EQUIVALENT')
        require(max(w.position_m[1] for w in group)-min(w.position_m[1] for w in group)
                <= .0001, 'NONLEVEL_AXLE_CONFIGURATION')
        left, right = min(w.position_m[0] for w in group), max(w.position_m[0] for w in group)
        require(left < 0 < right and right-left <= 5, 'INVALID_AXLE_PAIR')
        require(abs(left+right) <= .0001, 'UNSUPPORTED_ASYMMETRIC_AXLE')
        return ((left+right)/2, group[0].position_m[1], group[0].position_m[2]), right-left

    centre, track = axle(rear)
    wheelbase = None
    if article.slot == -1:
        front_centre, _ = axle(front)
        wheelbase = centre[2]-front_centre[2]
        require(2 <= wheelbase <= 8, 'UNSUPPORTED_WHEELBASE')
    require(abs(article.hook_local_m[0]-centre[0]) <= .0001,
            'UNSUPPORTED_OFF_CENTRE_HITCH')
    return FixedAxleGeometry(centre, wheelbase, centre[2]-article.hook_local_m[2], track)


@dataclass(frozen=True)
class ProfileToken:
    producer_session: str
    generation: int
    configuration: str
    evidence: str


@dataclass(frozen=True)
class VehicleProfile:
    schema_version: int
    token: ProfileToken
    observation: VehicleObservation | None
    observed_at: float  # first arrival of this SDK frame, not each repeated poll
    observation_failure: str
    model_failure: str
    model: Vehicle | None
    axle_geometry: tuple[FixedAxleGeometry | None, ...]
    geometry_failures: tuple[str, ...]
    # These are explicit missing prerequisites, not driving permission flags.
    live_maneuver_ready: bool = False
    live_blockers: tuple[str, ...] = (
        'UNPROVEN_FULL_ACCESSORY_CONFIGURATION',
        'SDK_FRAME_NOT_ATOMIC_OR_CHANNEL_VALIDATED',
        'UNPROVEN_GROUND_REFERENCE_AND_SURFACE',
    )

    def diagnostic(self):
        return {'schema_version': self.schema_version, 'token': asdict(self.token),
                'sdk_frame_us': self.observation.sdk_frame_us if self.observation else 0,
                'observed_at': self.observed_at,
                'observation_failure': self.observation_failure,
                'model_failure': self.model_failure,
                'model_available': self.model is not None,
                'live_maneuver_ready': False, 'live_blockers': self.live_blockers,
                'axle_geometry': [asdict(a) if a else None for a in self.axle_geometry],
                'geometry_failures': self.geometry_failures,
                'attached_articles': sum(a.attached for a in self.observation.articles)
                                     if self.observation else 0}


def compile_catalog_entry(entry, observation):
    """Strict offline adapter. Every Body dimension comes from external evidence."""
    require(isinstance(entry, dict) and type(entry.get('schema_version')) is int
            and entry['schema_version'] == 1,
            'INVALID_VEHICLE_PROFILE_SCHEMA')
    source = entry.get('source')
    evidence = entry.get('evidence_sha256')
    require(type(source) is str and bool(source.strip()) and type(evidence) is str
            and len(evidence) == 64 and all(c in '0123456789abcdef' for c in evidence)
            and entry.get('confirmed') is True, 'UNPROVEN_BODY_DIMENSIONS')
    require(entry.get('configuration_fingerprint') == configuration_fingerprint(observation),
            'VEHICLE_PROFILE_CONFIGURATION_MISMATCH')
    articles = tuple(a for a in observation.articles if a.attached)
    require(1 <= len(articles) <= 5, 'UNSUPPORTED_ARTICLE_COUNT')
    definitions = entry.get('bodies')
    require(type(definitions) is list and len(definitions) == len(articles),
            'MISSING_ARTICLE_BODY_PROFILE')
    axles = tuple(fixed_axle_geometry(a) for a in articles)
    bodies = []
    for a, axle, definition in zip(articles, axles, definitions):
        require(isinstance(definition, dict) and type(definition.get('slot')) is int
                and definition['slot'] == a.slot,
                'BODY_PROFILE_SLOT_MISMATCH')
        require(definition.get('axle_model') == 'fixed_axle', 'UNSUPPORTED_AXLE_MODEL')
        dims = [definition.get(k) for k in ('width_m', 'front_m', 'rear_m',
                                           'hitch_front_m', 'hitch_rear_m')]
        require(all(number(v) for v in dims), 'INVALID_BODY_DIMENSIONS')
        require(dims[0] >= axle.track_m, 'BODY_PROFILE_NARROWER_THAN_WHEEL_CENTRES')
        # Incoming hook exists for each trailer; truck SDK hook is outgoing.
        measured = dims[4] if a.slot == -1 else dims[3]
        require(abs(measured-axle.hook_forward_m) <= .0001, 'BODY_PROFILE_HITCH_MISMATCH')
        # A trailer's outgoing hitch is NOT present in this ABI. Its value
        # remains explicitly sourced from the confirmed external body profile.
        body = Body('tractor' if a.slot == -1 else f'trailer{a.slot}', *dims,
                    source + ':' + evidence, True, 'fixed_axle')
        body.validate(a.slot != -1)
        bodies.append(body)
    limits = entry.get('limits', {})
    require(isinstance(limits, dict), 'INVALID_MODEL_LIMITS')
    names = ('max_tyre_rad', 'max_speed_mps', 'safety_margin_m', 'uncertainty_m')
    params = [limits.get(name) for name in names]
    require(all(number(v) for v in params), 'INVALID_MODEL_LIMITS')
    require(params[3] >= .0001, 'UNCERTAINTY_BELOW_CONFIGURATION_PRECISION')
    # This is a documented physical tyre bound, NOT normalized steering gain.
    model = Vehicle(tuple(bodies), axles[0].wheelbase_m, *params)
    model.validate()
    return model, axles


class VehicleProfileProvider:
    """One owner per Telemetry process. Invalid input never holds a valid model."""
    def __init__(self, catalog=None):
        self.session = uuid.uuid4().hex
        self.generation = 0
        self._key = None
        self._last_frame = None
        self._frame_seen_at = 0.
        self._last_now = None
        self._compiled_key = None
        self._compiled = (None, (), 'MISSING_CONFIRMED_BODY_PROFILE', ())
        self.catalog_error = ''
        self.entries = {}
        if catalog is None:
            catalog = {'schema_version': 1, 'profiles': []}
        try:
            require(isinstance(catalog, dict) and type(catalog.get('schema_version')) is int
                    and catalog['schema_version'] == 1 and type(catalog.get('profiles')) is list
                    and len(catalog['profiles']) <= 128, 'INVALID_VEHICLE_PROFILE_CATALOG')
            for entry in catalog['profiles']:
                require(isinstance(entry, dict), 'INVALID_VEHICLE_PROFILE_CATALOG')
                key = entry.get('configuration_fingerprint')
                require(type(key) is str and len(key) == 64 and
                        all(c in '0123456789abcdef' for c in key), 'INVALID_PROFILE_FINGERPRINT')
                require(key not in self.entries, 'AMBIGUOUS_VEHICLE_PROFILE')
                # Own the data; a settings/UI mutation cannot change a live token.
                self.entries[key] = json.loads(json.dumps(entry, allow_nan=False))
        except (EnvelopeError, TypeError, ValueError, RecursionError, OverflowError) as exc:
            self.entries = {}
            self.catalog_error = str(exc) if isinstance(exc, EnvelopeError) else 'INVALID_VEHICLE_PROFILE_CATALOG'

    def update(self, observation, now):
        require(number(now) and now >= 0, 'INVALID_MONOTONIC_TIME')
        failure = ''
        fingerprint = evidence = ''
        o = observation if isinstance(observation, VehicleObservation) else None
        frame = o.sdk_frame_us if o else None
        reset = self._last_frame is not None and frame is not None and frame < self._last_frame
        clock_reset = self._last_now is not None and now < self._last_now
        if o is None:
            failure = 'VEHICLE_OBSERVATION_UNAVAILABLE'
        elif type(o.schema_version) is not int or o.schema_version != 1 or o.source != 'scs_shared_memory_revision_12':
            failure = 'UNSUPPORTED_VEHICLE_OBSERVATION_SCHEMA'
        elif o.failure_reason:
            failure = o.failure_reason
        elif not o.active or o.paused:
            failure = 'SDK_PAUSED_OR_INACTIVE'
        elif not o.stable_read or not number(o.captured_at) or o.captured_at > now:
            failure = 'INVALID_VEHICLE_CAPTURE'
        elif type(frame) is not int or frame <= 0:
            failure = 'INVALID_VEHICLE_FRAME_CLOCK'
        elif reset or clock_reset:
            failure = 'VEHICLE_CLOCK_REGRESSED'
        elif len(o.articles) != 11 or tuple(a.slot for a in o.articles) != tuple(range(-1,10)):
            failure = 'INCOMPLETE_VEHICLE_SLOT_OBSERVATION'
        elif not o.articles[0].attached:
            failure = 'MISSING_TRACTOR_OBSERVATION'
        else:
            attached = [a.slot for a in o.articles if a.attached]
            if attached != list(range(-1, len(attached)-1)):
                failure = 'NONCONTIGUOUS_TRAILER_CHAIN'
            else:
                fingerprint = configuration_fingerprint(o)
        if frame != self._last_frame or reset or clock_reset:
            self._frame_seen_at = o.captured_at if o and number(o.captured_at) else now
        if not failure and now-self._frame_seen_at > .5:
            failure = 'STALE_VEHICLE_OBSERVATION'
        entry = self.entries.get(fingerprint)
        evidence = digest(entry) if entry else ''
        # Dynamic axle support affects model validity, not static dimensions.
        support = tuple((w.on_ground, w.lift == 0, w.lift_offset_m == 0)
                        for a in o.articles if a.attached for w in a.wheels) if o else ()
        key = (fingerprint, evidence, failure, support)
        if key != self._key or reset or clock_reset:
            self.generation += 1
            self._key = key
        self._last_frame, self._last_now = frame, now
        if key != self._compiled_key:
            model, axles, geometry_failures = None, (), ()
            model_failure = failure or self.catalog_error or 'MISSING_CONFIRMED_BODY_PROFILE'
            if not failure:
                # Useful observed geometry is published even with no body
                # catalog, but unsupported axles remain explicitly unknown.
                geometry, reasons = [], []
                for article in o.articles:
                    if article.attached:
                        try:
                            geometry.append(fixed_axle_geometry(article))
                            reasons.append('')
                        except EnvelopeError as exc:
                            geometry.append(None); reasons.append(str(exc))
                axles, geometry_failures = tuple(geometry), tuple(reasons)
            if not failure and not self.catalog_error and entry:
                try:
                    model, axles = compile_catalog_entry(entry, o)
                    model_failure = ''
                except (EnvelopeError, TypeError, ValueError, KeyError) as exc:
                    model_failure = str(exc) if isinstance(exc, EnvelopeError) else 'INVALID_BODY_PROFILE'
            self._compiled = model, axles, model_failure, geometry_failures
            self._compiled_key = key
        model, axles, model_failure, geometry_failures = self._compiled
        return VehicleProfile(1, ProfileToken(self.session, self.generation, fingerprint, evidence),
                              o, self._frame_seen_at, failure, model_failure, model, axles, geometry_failures)


@dataclass(frozen=True)
class EnvelopeVehicleBinding:
    """Offline model bound to one route/context; NOT a maneuver or pose proof."""
    vehicle: Vehicle
    identity: Identity
    profile_token: ProfileToken
    configuration_confirmation_source: str


def bind_envelope_vehicle(profile, current, identity, expected_identity, now,
                          confirmed_token=None, confirmation_source=''):
    """External confirmation must attest the full actual accessory configuration.

    No runtime code automatically supplies it from the partial SDK fingerprint.
    A caller must recheck the binding against the current profile/route before
    each future use. Frame/ground/surface/traffic validation remains mandatory.
    """
    require(isinstance(profile, VehicleProfile) and isinstance(current, VehicleProfile),
            'MISSING_CURRENT_VEHICLE_PROFILE')
    require(profile.token == current.token, 'STALE_VEHICLE_PROFILE')
    require(number(now) and 0 <= now-current.observed_at <= .5,
            'STALE_VEHICLE_OBSERVATION')
    require(not current.observation_failure, current.observation_failure)
    require(current.model is not None, current.model_failure)
    require(isinstance(identity, Identity) and isinstance(expected_identity, Identity), 'MISSING_IDENTITY')
    identity.validate(); expected_identity.validate()
    require(identity == expected_identity, 'STALE_ENVELOPE_IDENTITY')
    require(confirmed_token == current.token and isinstance(confirmation_source, str)
            and bool(confirmation_source.strip()), 'UNPROVEN_FULL_ACCESSORY_CONFIGURATION')
    current.model.validate()
    return EnvelopeVehicleBinding(current.model, identity, current.token, confirmation_source)
