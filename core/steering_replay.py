"""Bounded, observational high-rate steering telemetry.

The normal console diagnostic is intentionally human-readable and throttled to
roughly one row per second.  That cadence cannot prove or reject sub-second
steering oscillation.  ``SteeringReplayBuffer`` retains the latest control
ticks in memory and writes them only on an explicit safety/shutdown event.  It
does not publish shared state, alter timing inputs, or participate in control.
"""

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import json
import math
import os
import re
import threading
import time
import uuid


SCHEMA_VERSION = 3
CALCULATION_PACKET_SCHEMA_VERSION = 1
DEFAULT_CAPACITY = 3600
EXECUTION_CAPACITY_MULTIPLIER = 3
MAX_EXECUTION_CAPACITY = 20000

_TRAJECTORY_IDENTITY_FIELDS = (
    "navigation_intent_id",
    "route_build_id",
    "revision",
    "source_game_session_id",
    "source_map_key",
    "source_dataset_fingerprint",
)


def _json_value(value):
    """Return a deterministic JSON-safe copy; non-finite numbers are absent."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return str(value)
    return number if math.isfinite(number) else None


def diagnostic_copy(value):
    """Return a detached JSON-safe diagnostic value.

    Shared state and the lightweight test doubles used by the plugins may
    return mutable nested dictionaries.  A shallow ``dict(...)`` copy can
    therefore pair a command with lane/geometry values changed by a later map
    tick.  This copy is deliberately observational and is never passed back to
    the control path.
    """
    return _json_value(value)


def bind_steering_calculation(debug, lane_match, trajectory, *, sequence):
    """Freeze every input/output identity belonging to one Route calculation.

    The full LanePath is intentionally not duplicated at control frequency.
    Its revision-bound identity is sufficient to locate the immutable
    trajectory export, while the local projection/tangent/curvature already
    present in ``debug`` records the geometry actually used by the controller.
    """
    packet = diagnostic_copy(debug or {})
    identity = {
        key: (trajectory or {}).get(key) for key in _TRAJECTORY_IDENTITY_FIELDS
    }
    packet.update({
        "calculation_packet_schema_version": CALCULATION_PACKET_SCHEMA_VERSION,
        "calculation_sequence": int(sequence),
        "lane_match_snapshot": diagnostic_copy(lane_match or {}),
        "trajectory_identity": diagnostic_copy(identity),
    })
    return packet


def steering_packet_binding(packet, application_trajectory):
    """Validate replay causality without accepting or rejecting control.

    Returns ``(valid, reasons)``.  It is diagnostic only: runtime authority is
    still decided by ``navigation_command`` and the existing fail-closed
    checks.  Keeping this separate prevents telemetry instrumentation from
    becoming a second steering authority.
    """
    packet = packet or {}
    reasons = []
    if packet.get("calculation_packet_schema_version") != (
            CALCULATION_PACKET_SCHEMA_VERSION):
        reasons.append("missing_or_unknown_calculation_packet_schema")
    try:
        if int(packet.get("calculation_sequence", 0)) <= 0:
            reasons.append("missing_calculation_sequence")
    except (TypeError, ValueError, OverflowError):
        reasons.append("invalid_calculation_sequence")

    required = (
        "computed_at", "observation_timestamp", "sdk_frame_us",
        "controller", "raw", "output", "local_curvature",
        "preview_curvature", "tracking_progress_m",
        "tracking_segment_index", "tracking_segment_fraction",
        "tracking_projection_xz", "local_tangent_heading_rad",
        "observation_xz", "observation_heading_rad",
    )
    for key in required:
        if packet.get(key) is None:
            reasons.append(f"missing_{key}")

    match = packet.get("lane_match_snapshot")
    if not isinstance(match, dict) or not match:
        reasons.append("missing_lane_match_snapshot")
    identity = packet.get("trajectory_identity")
    if not isinstance(identity, dict):
        reasons.append("missing_trajectory_identity")
        identity = {}
    for key in _TRAJECTORY_IDENTITY_FIELDS:
        packet_value = identity.get(key)
        application_value = (application_trajectory or {}).get(key)
        if packet_value is None:
            reasons.append(f"missing_identity_{key}")
        elif application_value is not None and packet_value != application_value:
            reasons.append(f"application_mismatch_{key}")
    if isinstance(match, dict) and identity.get("revision") is not None:
        try:
            if int(match.get("revision")) != int(identity["revision"]):
                reasons.append("lane_match_revision_mismatch")
        except (TypeError, ValueError, OverflowError):
            reasons.append("invalid_lane_match_revision")
    return not reasons, tuple(reasons)


class SteeringReplayBuffer:
    """Keep a fixed number of telemetry ticks and atomically export a replay."""

    def __init__(self, capacity=DEFAULT_CAPACITY, *, monotonic=time.monotonic,
                 wall_time=time.time):
        capacity = int(capacity)
        if capacity < 2 or capacity > 20000:
            raise ValueError("steering replay capacity must be within 2..20000")
        self._samples = deque(maxlen=capacity)
        # Normal samples follow the plugin/application cadence, while the
        # execution stream runs at 60 Hz.  Equal item counts retained only
        # about one third of the same drive in schema 3 and discarded the
        # beginning of ordinary three-minute reproductions.
        execution_capacity = min(
            MAX_EXECUTION_CAPACITY,
            capacity * EXECUTION_CAPACITY_MULTIPLIER)
        self._execution_samples = deque(maxlen=execution_capacity)
        self._monotonic = monotonic
        self._wall_time = wall_time
        self._sequence = 0
        self._execution_sequence = 0
        self._lock = threading.Lock()

    @property
    def capacity(self):
        return int(self._samples.maxlen)

    @property
    def execution_capacity(self):
        return int(self._execution_samples.maxlen)

    def __len__(self):
        with self._lock:
            return len(self._samples)

    def append(self, sample):
        """Copy one tick without feeding anything back to the control path."""
        with self._lock:
            self._sequence += 1
            row = dict(sample or {})
            row.update({
                "sequence": self._sequence,
                "monotonic_s": float(self._monotonic()),
                "wall_time_s": float(self._wall_time()),
            })
            self._samples.append(_json_value(row))

    def append_execution(self, sample):
        """Record one physical dynamics step independently of plugin cadence."""
        with self._lock:
            self._execution_sequence += 1
            row = dict(sample or {})
            row.update({
                "sequence": self._execution_sequence,
                "monotonic_s": float(self._monotonic()),
                "wall_time_s": float(self._wall_time()),
            })
            self._execution_samples.append(_json_value(row))

    def snapshot(self):
        # Do not let an analyser mutate nested evidence retained by the ring.
        with self._lock:
            return tuple(diagnostic_copy(row) for row in self._samples)

    def execution_snapshot(self):
        with self._lock:
            return tuple(diagnostic_copy(row)
                         for row in self._execution_samples)

    def export(self, directory, *, reason, identity=None):
        """Write the current immutable snapshot using an atomic replacement."""
        # Copy both streams and their counters under one lock.  An execution
        # tick must not land between the copied rows and the dropped-row
        # counters recorded in the same evidence document.
        with self._lock:
            samples = tuple(diagnostic_copy(row) for row in self._samples)
            execution_samples = tuple(
                diagnostic_copy(row) for row in self._execution_samples)
            sample_sequence = self._sequence
            execution_sequence = self._execution_sequence
        if not samples:
            return None
        directory = os.path.abspath(os.fspath(directory))
        os.makedirs(directory, exist_ok=True)
        now = datetime.now(timezone.utc)
        stamp = now.strftime("%Y%m%dT%H%M%S.%fZ")
        safe_reason = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(reason or "event"))
        safe_reason = safe_reason.strip("-.")[:48] or "event"
        path = os.path.join(
            directory, f"steering-replay-{stamp}-{safe_reason}.json")
        temporary = path + ".tmp-" + uuid.uuid4().hex
        payload = {
            "schema_version": SCHEMA_VERSION,
            "created_at": now.isoformat(),
            "reason": str(reason or "event"),
            "capacity": self.capacity,
            "execution_capacity": self.execution_capacity,
            "sample_count": len(samples),
            "dropped_sample_count": max(0, sample_sequence-len(samples)),
            "execution_sample_count": len(execution_samples),
            "dropped_execution_sample_count": max(
                0, execution_sequence-len(execution_samples)),
            "identity": _json_value(identity or {}),
            "samples": samples,
            "execution_samples": execution_samples,
        }
        try:
            with open(temporary, "x", encoding="utf-8") as stream:
                json.dump(payload, stream, ensure_ascii=False,
                          separators=(",", ":"), sort_keys=True)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        except Exception:
            try:
                os.remove(temporary)
            except OSError:
                pass
            raise
        return path
