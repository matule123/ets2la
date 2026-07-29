"""Stable navigation intent identity for the moving ETS2LA SDK UID window.

The native buffer is an observation of a route window, not destination
identity.  This module compares ordered, occurrence-sensitive UID sequences and
their remaining-distance samples.  It deliberately does not build geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import uuid


class NavigationBufferClass(str, Enum):
    SAME_EXACT = "SAME_EXACT"
    ADVANCED_PREFIX = "ADVANCED_PREFIX"
    EXTENDED_HORIZON = "EXTENDED_HORIZON"
    OVERLAPPING_CONTINUATION = "OVERLAPPING_CONTINUATION"
    TEMPORARILY_UNAVAILABLE = "TEMPORARILY_UNAVAILABLE"
    TRUE_REROUTE = "TRUE_REROUTE"
    DESTINATION_REMOVED = "DESTINATION_REMOVED"
    SESSION_OR_DATASET_CHANGED = "SESSION_OR_DATASET_CHANGED"


CONTINUATION_CLASSES = frozenset({
    NavigationBufferClass.SAME_EXACT,
    NavigationBufferClass.ADVANCED_PREFIX,
    NavigationBufferClass.EXTENDED_HORIZON,
    NavigationBufferClass.OVERLAPPING_CONTINUATION,
    NavigationBufferClass.TEMPORARILY_UNAVAILABLE,
})


@dataclass(frozen=True, slots=True)
class RouteItem:
    uid: int
    distance_m: float | None = None


@dataclass(frozen=True, slots=True)
class NavigationBufferDecision:
    classification: NavigationBufferClass
    intent_id: str | None
    previous_intent_id: str | None
    old_count: int
    new_count: int
    overlap: int = 0
    trimmed: int = 0
    extended: int = 0
    reason: str = ""
    intent_changed: bool = False


def normalize_route_items(values):
    items = []
    for value in values or ():
        try:
            if isinstance(value, RouteItem):
                uid = value.uid
                raw_distance = value.distance_m
            elif isinstance(value, dict):
                uid = int(value.get("uid", 0) or 0)
                raw_distance = value.get("distance")
            else:
                uid = int(value)
                raw_distance = None
        except (TypeError, ValueError, OverflowError):
            continue
        if not uid:
            continue
        try:
            distance = float(raw_distance) if raw_distance is not None else None
            if distance is not None and (not math.isfinite(distance)
                                         or distance < 0.0):
                distance = None
        except (TypeError, ValueError, OverflowError):
            distance = None
        items.append(RouteItem(uid, distance))
    return tuple(items)


def route_uids(items):
    return tuple(item.uid for item in normalize_route_items(items))


def _distance_evidence_matches(old_items, new_items):
    compared = 0
    for old, new in zip(old_items, new_items):
        if old.distance_m is None or new.distance_m is None:
            continue
        compared += 1
        tolerance = max(80.0, 0.015 * max(old.distance_m, new.distance_m))
        if abs(old.distance_m - new.distance_m) > tolerance:
            return False
    # UID order alone remains valid for exact/prefix relations. For a partial
    # overlap containing repeated UIDs, any available distance evidence must
    # agree so a different lap cannot masquerade as forward progress.
    return compared >= 0


def ordered_suffix_prefix_overlap(old_values, new_values):
    """Longest trustworthy old-suffix/new-prefix overlap, preserving repeats."""
    old = normalize_route_items(old_values)
    new = normalize_route_items(new_values)
    for count in range(min(len(old), len(new)), 0, -1):
        old_part, new_part = old[-count:], new[:count]
        if ([item.uid for item in old_part] == [item.uid for item in new_part]
                and _distance_evidence_matches(old_part, new_part)):
            return count
    return 0


def destination_identity(dest_city, items, explicit_request=None):
    """Return only a destination identity proven independently of window size."""
    if explicit_request not in (None, ""):
        return ("request", str(explicit_request))
    city = str(dest_city or "").strip().casefold()
    if city:
        return ("city", city)
    normalized = normalize_route_items(items)
    # A terminal SDK item near zero remaining distance is an actual endpoint.
    # A non-zero last distance is a truncated horizon and must not become a
    # destination identity merely because another UID is appended later.
    if (normalized and normalized[-1].distance_m is not None
            and normalized[-1].distance_m <= 100.0):
        return ("terminal_uid", normalized[-1].uid)
    return None


def classify_navigation_buffer(old_values, new_values, *,
                               destination_present=True,
                               old_destination=None, new_destination=None,
                               old_context=None, new_context=None):
    old = normalize_route_items(old_values)
    new = normalize_route_items(new_values)
    old_uids = tuple(item.uid for item in old)
    new_uids = tuple(item.uid for item in new)

    if not destination_present:
        return (NavigationBufferClass.DESTINATION_REMOVED, 0, 0, 0,
                "the game reports that the destination was removed")
    if (old_context is not None and new_context is not None
            and tuple(old_context) != tuple(new_context)):
        return (NavigationBufferClass.SESSION_OR_DATASET_CHANGED, 0, 0, 0,
                "game session, map key or dataset fingerprint changed")
    if (old_destination is not None and new_destination is not None
            and old_destination != new_destination):
        return (NavigationBufferClass.TRUE_REROUTE, 0, 0, 0,
                "proven waypoint or terminal destination identity changed")
    if not new:
        return (NavigationBufferClass.TEMPORARILY_UNAVAILABLE, 0, 0, 0,
                "destination still exists but the SDK UID buffer is unavailable")
    if not old:
        return (NavigationBufferClass.TRUE_REROUTE, 0, 0, len(new),
                "first usable SDK route window for this destination")
    if old_uids == new_uids:
        return (NavigationBufferClass.SAME_EXACT, len(old), 0, 0,
                "ordered UID window is byte-for-byte identical")

    for trimmed in range(1, len(old)):
        if old_uids[trimmed:] == new_uids and _distance_evidence_matches(
                old[trimmed:], new):
            return (NavigationBufferClass.ADVANCED_PREFIX, len(new), trimmed, 0,
                    f"new window is the old ordered suffix after {trimmed} passed UIDs")
    if (len(new) > len(old) and new_uids[:len(old)] == old_uids
            and _distance_evidence_matches(old, new[:len(old)])):
        return (NavigationBufferClass.EXTENDED_HORIZON, len(old), 0,
                len(new)-len(old),
                "the existing ordered window is unchanged and only its horizon grew")

    overlap = ordered_suffix_prefix_overlap(old, new)
    # Two consecutive ordered edges are the minimum topological proof. UID
    # occurrences are not collapsed into a set, and remaining-distance samples
    # above disambiguate repeated sequences on loops/roundabouts.
    if overlap >= 2:
        trimmed = len(old) - overlap
        extended = len(new) - overlap
        if trimmed > 0 and extended > 0:
            return (NavigationBufferClass.OVERLAPPING_CONTINUATION,
                    overlap, trimmed, extended,
                    "ordered suffix/prefix overlap proves a rolling continuation")

    return (NavigationBufferClass.TRUE_REROUTE, overlap, 0, 0,
            "ordered windows lack a trustworthy forward overlap")


class NavigationIntentTracker:
    """State machine that keeps one ID across compatible SDK window changes."""

    def __init__(self, intent_id=None):
        self.intent_id = intent_id
        self.items = ()
        self.destination = None
        self.context = None
        self._awaiting_first_window = False

    def update(self, values, *, destination_present, destination=None,
               context=None):
        new_items = normalize_route_items(values)
        old_count = len(self.items)
        classification, overlap, trimmed, extended, reason = (
            classify_navigation_buffer(
                self.items, new_items,
                destination_present=destination_present,
                old_destination=self.destination,
                new_destination=destination,
                old_context=self.context,
                new_context=context))
        if (self._awaiting_first_window and new_items
                and classification == NavigationBufferClass.TRUE_REROUTE
                and destination == self.destination
                and (context is None or tuple(context) == self.context)):
            classification = NavigationBufferClass.EXTENDED_HORIZON
            overlap, trimmed, extended = 0, 0, len(new_items)
            reason = "first SDK window for an already established navigation intent"
        previous_intent = self.intent_id
        if classification == NavigationBufferClass.DESTINATION_REMOVED:
            self.intent_id = None
        elif classification in {
                NavigationBufferClass.TRUE_REROUTE,
                NavigationBufferClass.SESSION_OR_DATASET_CHANGED}:
            self.intent_id = uuid.uuid4().hex

        if classification != NavigationBufferClass.TEMPORARILY_UNAVAILABLE:
            self.items = new_items
        if destination is not None:
            self.destination = destination
        if context is not None:
            self.context = tuple(context)
        if classification == NavigationBufferClass.DESTINATION_REMOVED:
            self.items = ()
            self.destination = None
            self._awaiting_first_window = False
        elif classification in {
                NavigationBufferClass.TRUE_REROUTE,
                NavigationBufferClass.SESSION_OR_DATASET_CHANGED}:
            self._awaiting_first_window = not bool(new_items)
        elif new_items:
            self._awaiting_first_window = False

        return NavigationBufferDecision(
            classification, self.intent_id, previous_intent,
            old_count,
            len(new_items), overlap, trimmed, extended, reason,
            previous_intent != self.intent_id)


@dataclass(frozen=True, slots=True)
class BuildToken:
    intent_id: str
    input_key: tuple
    build_id: str
    generation: int


class NavigationBuildGuard:
    """One active build per intent; completed/stale callbacks cannot republish."""

    def __init__(self):
        self._active = {}
        self._completed = set()
        self._generation = 0

    def begin(self, intent_id, input_key, build_id):
        intent_id = str(intent_id or "")
        key = (intent_id, tuple(input_key))
        if not intent_id or intent_id in self._active or key in self._completed:
            return None
        self._generation += 1
        token = BuildToken(intent_id, tuple(input_key), str(build_id),
                           self._generation)
        self._active[intent_id] = token
        return token

    def may_publish(self, token):
        return bool(token is not None
                    and self._active.get(token.intent_id) == token)

    def finish(self, token):
        if token is None:
            return
        if self._active.get(token.intent_id) == token:
            self._active.pop(token.intent_id, None)
            self._completed.add((token.intent_id, token.input_key))

    def input_completed(self, intent_id, input_key):
        return (str(intent_id or ""), tuple(input_key)) in self._completed

    def reset_intent(self, intent_id):
        intent_id = str(intent_id or "")
        self._active.pop(intent_id, None)
        self._completed = {key for key in self._completed if key[0] != intent_id}


def snapshot_matches_navigation_intent(state, snapshot):
    """Compatibility predicate shared by HUD, AR, map and autopilot."""
    if not isinstance(snapshot, dict):
        return False
    intent_id = state.get("navigation_intent_id")
    snapshot_intent = snapshot.get("navigation_intent_id")
    if intent_id is not None or snapshot_intent is not None:
        return bool(intent_id and snapshot_intent == intent_id
                    and snapshot.get("request_id") == intent_id)
    # Safe compatibility for snapshots produced before intent IDs existed.
    try:
        return (tuple(int(uid) for uid in
                      (snapshot.get("source_gps_uids", ()) or ()))
                == tuple(int(uid) for uid in
                         (state.get("game_route_node_uids", ()) or ()))
                and snapshot.get("request_id")
                    == state.get("nav_recalc_request"))
    except (TypeError, ValueError, OverflowError):
        return False
