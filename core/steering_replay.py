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
import time
import uuid


SCHEMA_VERSION = 1
DEFAULT_CAPACITY = 3600


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


class SteeringReplayBuffer:
    """Keep a fixed number of telemetry ticks and atomically export a replay."""

    def __init__(self, capacity=DEFAULT_CAPACITY, *, monotonic=time.monotonic,
                 wall_time=time.time):
        capacity = int(capacity)
        if capacity < 2 or capacity > 20000:
            raise ValueError("steering replay capacity must be within 2..20000")
        self._samples = deque(maxlen=capacity)
        self._monotonic = monotonic
        self._wall_time = wall_time
        self._sequence = 0

    @property
    def capacity(self):
        return int(self._samples.maxlen)

    def __len__(self):
        return len(self._samples)

    def append(self, sample):
        """Copy one tick without feeding anything back to the control path."""
        self._sequence += 1
        row = dict(sample or {})
        row.update({
            "sequence": self._sequence,
            "monotonic_s": float(self._monotonic()),
            "wall_time_s": float(self._wall_time()),
        })
        self._samples.append(_json_value(row))

    def snapshot(self):
        return tuple(dict(row) for row in self._samples)

    def export(self, directory, *, reason, identity=None):
        """Write the current immutable snapshot using an atomic replacement."""
        samples = self.snapshot()
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
            "sample_count": len(samples),
            "dropped_sample_count": max(0, self._sequence-len(samples)),
            "identity": _json_value(identity or {}),
            "samples": samples,
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
