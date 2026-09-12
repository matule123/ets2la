"""Read-only vehicle observations, RenCloud shared-memory ABI revision 12.

Offsets are interface facts from scs-telemetry-common.hpp, not copied reader
code. Metres, seconds and radians are explicit. SCS placement headings are
counterclockwise turns; physical steering exposed here is positive RIGHT.
No dimensions, equivalent tandem axle, or ground plane are fabricated.

Two equal copies detect an overlapping write, NOT an atomic SDK transaction:
the upstream ABI has no sequence lock / per-field validity bits. Consumers
must not promote these observations to certified multi-body collision poses.
"""
from dataclasses import dataclass
import math
import struct
import time

ABI_REVISION = 12
TRAILER_START = 6000
TRAILER_SIZE = 1560
TRAILER_COUNT = 10
CAPTURE_SIZE = TRAILER_START + TRAILER_SIZE * TRAILER_COUNT
SOURCE = 'scs_shared_memory_revision_12'


@dataclass(frozen=True)
class WheelObservation:
    index: int
    position_m: tuple[float, float, float]
    radius_m: float
    steerable: bool
    simulated: bool
    powered: bool
    liftable: bool
    on_ground: bool
    lift: float
    lift_offset_m: float
    steering_right_rad: float


@dataclass(frozen=True)
class ArticleObservation:
    slot: int  # -1 tractor, 0..9 trailers, including detached slots
    attached: bool
    vehicle_id: str
    brand_id: str
    name: str
    body_type: str
    chain_type: str
    cargo_accessory_id: str
    hook_local_m: tuple[float, float, float]
    wheels: tuple[WheelObservation, ...]
    position_m: tuple[float, float, float]  # SDK chassis origin, NOT ground axle
    rotation_rad: tuple[float, float, float]  # heading, pitch, roll


@dataclass(frozen=True)
class VehicleObservation:
    schema_version: int
    source: str
    failure_reason: str
    captured_at: float
    sdk_frame_us: int = 0
    simulated_us: int = 0
    render_us: int = 0
    active: bool = False
    paused: bool = False
    game_version: tuple[int, ...] = ()
    articles: tuple[ArticleObservation, ...] = ()
    stable_read: bool = False
    atomic: bool = False


class ObservationError(ValueError):
    pass


def decode_vehicle_observation(blob, captured_at):
    """Pure binary decoder; bounds/ABI/finite checks precede publication."""
    def fail(reason):
        raise ObservationError(reason)

    def values(fmt, offset, count=1):
        result = struct.unpack_from('<' + str(count) + fmt, blob, offset)
        if fmt in ('f', 'd') and not all(math.isfinite(v) for v in result):
            fail('NONFINITE_SDK_FIELD')
        return result

    def scalar(fmt, offset):
        return values(fmt, offset)[0]

    def flag(offset):
        value = scalar('B', offset)
        if value not in (0, 1):
            fail('INVALID_SDK_BOOLEAN')
        return bool(value)

    def string(offset):
        try:
            return blob[offset:offset+64].split(b'\0', 1)[0].decode('utf-8')
        except UnicodeError:
            fail('INVALID_SDK_STRING')

    if len(blob) != CAPTURE_SIZE:
        fail('TRUNCATED_SDK_VEHICLE_BLOCK')
    if scalar('I', 40) != ABI_REVISION:
        fail('UNSUPPORTED_SDK_ABI')
    game = values('I', 44, 5)
    if game[2] not in (1, 2):
        fail('UNKNOWN_SDK_GAME')
    active, paused = flag(0), flag(4)
    if not active:
        return VehicleObservation(1, SOURCE, 'SDK_PAUSED_OR_INACTIVE', captured_at,
            scalar('Q', 8), scalar('Q', 16), scalar('Q', 24), active, paused, game,
            stable_read=True)
    articles = []
    for slot in range(-1, TRAILER_COUNT):
        base = TRAILER_START + slot * TRAILER_SIZE
        attached = active if slot == -1 else flag(base+80)
        # Detached blocks may retain stale configuration/pose bytes. Read only
        # their attachment state; never let a parked trailer enter the chain.
        if slot >= 0 and not attached:
            articles.append(ArticleObservation(slot, False, '', '', '', '', '', '',
                            (0., 0., 0.), (), (0., 0., 0.), (0., 0., 0.)))
            continue
        count = scalar('I', 80 if slot == -1 else base+148)
        # ABI reserves 16 elements but the producer registers only 14 wheels.
        if not 2 <= count <= 14:
            fail('UNSUPPORTED_SDK_WHEEL_COUNT')
        if slot == -1:
            xyz, radius, flags, ground, lift, lift_offset, steering = (
                1676, 752, 1500, 1590, 1328, 1392, 1200)
            hook, placement = 1664, 2200
            ids = (string(2428), string(2300), string(2492), '', '', '')
        else:
            xyz, radius, flags, ground, lift, lift_offset, steering = (
                base+676, base+552, base, base+64, base+424, base+488, base+296)
            hook, placement = base+664, base+872
            ids = (string(base+920), string(base+1112), string(base+1240),
                   string(base+1048), string(base+1304), string(base+984))
        if not ids[0]:
            fail('MISSING_SDK_VEHICLE_ID')
        wheels = tuple(WheelObservation(i,
            tuple(scalar('f', xyz+axis*64+4*i) for axis in range(3)),
            scalar('f', radius+4*i), *(flag(flags+16*j+i) for j in range(4)),
            flag(ground+i), scalar('f', lift+4*i), scalar('f', lift_offset+4*i),
            -math.tau*scalar('f', steering+4*i)) for i in range(count))
        if any(not 0 < w.radius_m <= 2 or not 0 <= w.lift <= 1 or
               any(abs(v) > 50 for v in w.position_m) for w in wheels):
            fail('INVALID_SDK_WHEEL_GEOMETRY')
        position = values('d', placement, 3)
        rotation = tuple(v*math.tau for v in values('d', placement+24, 3))
        hook_xyz = values('f', hook, 3)
        if any(abs(v) > 1_000_000 for v in position) or any(abs(v) > 50 for v in hook_xyz):
            fail('INVALID_SDK_PLACEMENT')
        articles.append(ArticleObservation(slot, attached, *ids, hook_xyz,
                                          wheels, position, rotation))
    return VehicleObservation(1, SOURCE, '', captured_at, scalar('Q', 8),
        scalar('Q', 16), scalar('Q', 24), active, paused, game, tuple(articles), True)


def capture_vehicle_observation(mm, now=None):
    """Bounded read, at most three copies; no sleeps/retries on the control path."""
    captured = time.monotonic() if now is None else now
    try:
        first = bytes(mm[:CAPTURE_SIZE])
        for _ in range(2):
            second = bytes(mm[:CAPTURE_SIZE])
            if first == second:
                return decode_vehicle_observation(second, captured)
            first = second
        reason = 'SDK_VEHICLE_READ_CHANGED'
    except ObservationError as exc:
        reason = str(exc)
    except (TypeError, ValueError, OSError, BufferError, struct.error):
        reason = 'SDK_VEHICLE_READ_UNAVAILABLE'
    return VehicleObservation(1, SOURCE, reason, captured)
