"""Fixed-cadence execution of the one physical steering trajectory.

Route owns geometric steering and :class:`SteeringDynamics` owns the one
angle/rate/acceleration trajectory.  ``SteeringExecutor`` merely clocks that
existing dynamics at a deterministic cadence so unrelated longitudinal,
logging or IPC work cannot make the wheel advance in visible 50--300 ms steps.
It does not filter, interpolate or retain a history of target values.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

from core.control_timing import CadenceMonitor, wait_for_next_tick
from core.steering_dynamics import SteeringDynamics


STEERING_EXECUTION_HZ = 60.0
STEERING_TARGET_MAX_AGE_S = 0.5


class SteeringExecutor:
    def __init__(self, dynamics: Optional[SteeringDynamics] = None, *,
                 writer: Optional[Callable[[float], None]] = None,
                 observer: Optional[Callable[[dict], None]] = None,
                 clock=time.monotonic):
        self.dynamics = dynamics or SteeringDynamics()
        self._writer = writer
        self._observer = observer
        self._clock = clock
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._running = False
        self._active = False
        self._target = 0.0
        self._speed_ms = 0.0
        self._curvature_per_m = 0.0
        self._submitted_at = 0.0
        self._submission_sequence = 0
        self._output = float(self.dynamics.command)
        self._last_debug = dict(self.dynamics.last_debug)
        self._cadence = CadenceMonitor(STEERING_EXECUTION_HZ)

    @staticmethod
    def _finite(value, default=0.0):
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError):
            return float(default)
        return value if math.isfinite(value) else float(default)

    @property
    def running(self) -> bool:
        return bool(self._running)

    @property
    def output(self) -> float:
        with self._lock:
            return float(self._output)

    @property
    def last_debug(self) -> dict:
        with self._lock:
            return dict(self._last_debug)

    def reset(self, command=0.0, *, active=False) -> float:
        command = max(-1.0, min(1.0, self._finite(command)))
        now = self._clock()
        with self._lock:
            self._output = self.dynamics.reset(command)
            self._last_debug = dict(self.dynamics.last_debug)
            self._active = bool(active)
            self._target = command if active else 0.0
            self._speed_ms = 0.0
            self._curvature_per_m = 0.0
            self._submitted_at = now
            self._submission_sequence += 1
            return float(self._output)

    def submit(self, target, *, speed_ms=0.0, curvature_per_m=0.0,
               active=True, submitted_at=None) -> int:
        now = self._clock() if submitted_at is None else float(submitted_at)
        target = max(-1.0, min(1.0, self._finite(target)))
        speed = abs(self._finite(speed_ms))
        curvature = self._finite(curvature_per_m)
        with self._lock:
            self._target = target
            self._speed_ms = speed
            self._curvature_per_m = curvature
            self._active = bool(active)
            self._submitted_at = now
            self._submission_sequence += 1
            return self._submission_sequence

    def step(self, dt, *, now=None) -> float:
        now = self._clock() if now is None else float(now)
        with self._lock:
            age = max(0.0, now - self._submitted_at)
            active = bool(self._active)
            target_fresh = bool(age <= STEERING_TARGET_MAX_AGE_S)
            target = self._target if active and target_fresh else 0.0
            output = self.dynamics.update(
                target, dt, speed_ms=self._speed_ms,
                curvature_per_m=self._curvature_per_m)
            debug = dict(self.dynamics.last_debug)
            debug.update({
                "executor_active": active,
                "target_fresh": target_fresh,
                "target_age_s": age,
                "submission_sequence": self._submission_sequence,
                "execution_monotonic_s": now,
            })
            self._output = float(output)
            self._last_debug = debug
        if self._writer is not None:
            self._writer(float(output))
        if self._observer is not None:
            self._observer(dict(debug))
        return float(output)

    def start(self) -> None:
        if self._running:
            return
        self._stop.clear()
        self._cadence = CadenceMonitor(STEERING_EXECUTION_HZ)
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="UltraPilot-SteeringExecutor", daemon=True)
        self._thread.start()

    def stop(self, timeout=1.0) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, float(timeout)))
        self._running = False
        self._thread = None

    def _run(self) -> None:
        period = 1.0 / STEERING_EXECUTION_HZ
        previous = self._clock()
        deadline = previous
        while not self._stop.is_set():
            now = self._clock()
            dt = max(1e-3, now - previous)
            previous = now
            self._cadence.tick(now)
            self.step(dt, now=now)
            deadline, stopped = wait_for_next_tick(
                self._stop, now, period, clock=self._clock)
            if stopped:
                break
        self._running = False

    def cadence_snapshot(self, now=None) -> dict:
        return self._cadence.snapshot(now)
