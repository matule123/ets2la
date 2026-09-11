"""Monotonic timing primitives for the lateral-control data path.

The navigation geometry and controller stay stateless in time.  These helpers
only ensure that observations and already-computed command packets move
forward exactly once; they never alter a steering value or invent a sample.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Hashable, Optional


@dataclass(frozen=True)
class SequenceDecision:
    accepted: bool
    is_new: bool
    reason: str = ""


class MonotonicSequenceGate:
    """Accept new or identical packets and reject ordering regressions.

    ``identity`` binds the sequence to a game/navigation generation.  A real
    identity change resets the ordering state; a delayed callback from the old
    identity is still rejected by the caller's revision/intent validation.
    """

    def __init__(self):
        self._identity: Optional[Hashable] = None
        self._sequence: Optional[int] = None
        self._frame_us: Optional[int] = None
        self._lock = threading.Lock()

    def reset(self) -> None:
        with self._lock:
            self._identity = None
            self._sequence = None
            self._frame_us = None

    def observe(self, identity: Hashable, sequence: int,
                frame_us: Optional[int] = None) -> SequenceDecision:
        try:
            sequence = int(sequence)
            frame = None if frame_us is None else int(frame_us)
        except (TypeError, ValueError, OverflowError):
            return SequenceDecision(False, False, "packet ordering is malformed")
        if sequence <= 0 or (frame is not None and frame < 0):
            return SequenceDecision(False, False, "packet ordering is malformed")

        with self._lock:
            if identity != self._identity:
                self._identity = identity
                self._sequence = sequence
                self._frame_us = frame
                return SequenceDecision(True, True)
            assert self._sequence is not None
            if sequence < self._sequence:
                return SequenceDecision(
                    False, False,
                    f"calculation sequence regressed from {self._sequence} to {sequence}")
            if sequence == self._sequence:
                if frame != self._frame_us:
                    return SequenceDecision(
                        False, False,
                        "one calculation sequence was reused for a different SDK frame")
                return SequenceDecision(True, False)
            if (frame is not None and self._frame_us is not None
                    and frame <= self._frame_us):
                return SequenceDecision(
                    False, False,
                    f"SDK frame regressed from {self._frame_us} to {frame}")
            self._sequence = sequence
            self._frame_us = frame
            return SequenceDecision(True, True)


class FrameGate:
    """Deduplicate SDK observations without estimating missing frames."""

    def __init__(self):
        self._identity: Optional[Hashable] = None
        self._frame_us: Optional[int] = None
        self._lock = threading.Lock()

    def reset(self) -> None:
        with self._lock:
            self._identity = None
            self._frame_us = None

    def observe(self, identity: Hashable,
                frame_us: Optional[int]) -> SequenceDecision:
        try:
            frame = int(frame_us or 0)
        except (TypeError, ValueError, OverflowError):
            return SequenceDecision(False, False, "SDK frame is malformed")
        if frame < 0:
            return SequenceDecision(False, False, "SDK frame is malformed")
        # HTTP/legacy fixtures have no authoritative frame counter.  They must
        # retain their established every-tick behaviour.
        if frame == 0:
            return SequenceDecision(True, True)
        with self._lock:
            if identity != self._identity:
                self._identity = identity
                self._frame_us = frame
                return SequenceDecision(True, True)
            assert self._frame_us is not None
            if frame < self._frame_us:
                return SequenceDecision(
                    False, False,
                    f"SDK frame regressed from {self._frame_us} to {frame}")
            if frame == self._frame_us:
                return SequenceDecision(True, False)
            self._frame_us = frame
            return SequenceDecision(True, True)


class CadenceMonitor:
    """Bounded diagnostic statistics for one real-time loop."""

    def __init__(self, target_hz: float):
        target_hz = float(target_hz)
        if not math.isfinite(target_hz) or target_hz <= 0.0:
            raise ValueError("target_hz must be finite and positive")
        self.target_hz = target_hz
        self.started_at = time.monotonic()
        self.last_tick = None
        self.last_report = self.started_at
        self.tick_count = 0
        self.overrun_count = 0
        self.maximum_interval_s = 0.0
        self.interval_sum_s = 0.0

    def tick(self, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else float(now)
        if self.last_tick is not None:
            interval = max(0.0, now - self.last_tick)
            self.maximum_interval_s = max(self.maximum_interval_s, interval)
            self.interval_sum_s += interval
            if interval > (1.5 / self.target_hz):
                self.overrun_count += 1
        self.last_tick = now
        self.tick_count += 1

    def snapshot(self, now: Optional[float] = None, *, reset_window=False) -> dict:
        now = time.monotonic() if now is None else float(now)
        intervals = max(0, self.tick_count - 1)
        elapsed = max(0.0, now - self.started_at)
        result = {
            "target_hz": self.target_hz,
            "observed_hz": (intervals / elapsed if elapsed > 0.0 else 0.0),
            "tick_count": self.tick_count,
            "overrun_count": self.overrun_count,
            "maximum_interval_s": self.maximum_interval_s,
            "mean_interval_s": (
                self.interval_sum_s / intervals if intervals else 0.0),
            "timestamp": now,
        }
        if reset_window:
            self.started_at = now
            self.last_report = now
            self.tick_count = 0
            self.overrun_count = 0
            self.maximum_interval_s = 0.0
            self.interval_sum_s = 0.0
            self.last_tick = None
        return result
