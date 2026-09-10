"""Explicit, validated steering actuator calibration.

The SCS controller accepts a normalized command, while Route works in road-
wheel radians. These values describe only that boundary and its observed
timing. They are deliberately static: telemetry is diagnostic evidence, not
an online adaptation signal or a second steering controller.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


CALIBRATION_SCHEMA_VERSION = 1

# Identified from the two frame-bound 2026-09-10 replays for the tested truck
# and input setup. This is a provisional default, not an ETS2 universal value.
DEFAULT_TYRE_ANGLE_PER_INPUT_RAD = 0.70
MIN_TYRE_ANGLE_PER_INPUT_RAD = 0.60
MAX_TYRE_ANGLE_PER_INPUT_RAD = 0.95

# The accepted calculation is normally one 15 Hz SDK frame behind the current
# application observation and the written command appears in road-wheel data
# one further frame later. Keep the two causal terms explicit.
DEFAULT_COMMAND_DELAY_S = 0.067
DEFAULT_OBSERVATION_DELAY_S = 0.067
MIN_COMMAND_DELAY_S = 0.0
MAX_COMMAND_DELAY_S = 0.20
MIN_OBSERVATION_DELAY_S = 0.0
MAX_OBSERVATION_DELAY_S = 0.25
MIN_PREVIEW_HORIZON_S = 0.02
MAX_PREVIEW_HORIZON_S = 0.35
DEFAULT_CALIBRATION_SOURCE = "dense_replay_20260910_provisional"


@dataclass(frozen=True)
class SteeringActuatorCalibration:
    """Validated command/tyre conversion and causal observation timing."""

    schema_version: int
    tyre_angle_per_input_rad: float
    command_delay_s: float
    observation_delay_s: float
    source: str
    valid: bool = True
    failure_reason: str = ""
    legacy_lock_setting: bool = False

    @property
    def preview_horizon_s(self) -> float:
        return self.command_delay_s + self.observation_delay_s

    def as_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "tyre_angle_per_input_rad": self.tyre_angle_per_input_rad,
            # Compatibility/diagnostic alias; there is still one authority.
            "steering_lock_rad": self.tyre_angle_per_input_rad,
            "command_delay_s": self.command_delay_s,
            "observation_delay_s": self.observation_delay_s,
            "preview_horizon_s": self.preview_horizon_s,
            "source": self.source,
            "valid": self.valid,
            "failure_reason": self.failure_reason,
            "legacy_lock_setting": self.legacy_lock_setting,
            "online_adaptation": False,
        }


def _number(value: Any) -> float:
    if isinstance(value, bool):
        raise ValueError("boolean is not a numeric calibration")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("calibration value is not finite")
    return result


def _invalid(reason: str, *, lock: float = float("nan"),
             command_delay: float = float("nan"),
             observation_delay: float = float("nan"), source: str = "",
             legacy: bool = False) -> SteeringActuatorCalibration:
    return SteeringActuatorCalibration(
        schema_version=CALIBRATION_SCHEMA_VERSION,
        tyre_angle_per_input_rad=lock,
        command_delay_s=command_delay,
        observation_delay_s=observation_delay,
        source=source,
        valid=False,
        failure_reason=reason,
        legacy_lock_setting=legacy,
    )


def steering_calibration_from_settings(
        autopilot_settings: Mapping[str, Any] | None,
        ) -> SteeringActuatorCalibration:
    """Resolve one static calibration, failing closed on malformed overrides.

    Existing installations used scalar ``steering_lock_rad``. It remains a
    migration input only when the new object is absent and never competes with
    the structured calibration.
    """

    settings = autopilot_settings if isinstance(
        autopilot_settings, Mapping) else {}
    raw = settings.get("steering_actuator_calibration")
    legacy = raw is None and "steering_lock_rad" in settings
    if raw is None:
        raw = {
            "schema_version": CALIBRATION_SCHEMA_VERSION,
            "tyre_angle_per_input_rad": settings.get(
                "steering_lock_rad", DEFAULT_TYRE_ANGLE_PER_INPUT_RAD),
            "command_delay_s": DEFAULT_COMMAND_DELAY_S,
            "observation_delay_s": DEFAULT_OBSERVATION_DELAY_S,
            "source": ("legacy_lock_plus_phase4b_timing" if legacy else
                       DEFAULT_CALIBRATION_SOURCE),
        }
    if not isinstance(raw, Mapping):
        return _invalid("steering actuator calibration must be an object")
    try:
        schema_value = raw.get("schema_version", -1)
        if isinstance(schema_value, bool):
            raise ValueError("boolean schema")
        schema = int(schema_value)
        if float(schema_value) != float(schema):
            raise ValueError("fractional schema")
    except (TypeError, ValueError, OverflowError):
        schema = -1
    if schema != CALIBRATION_SCHEMA_VERSION:
        return _invalid(
            f"unsupported steering actuator calibration schema {schema}")
    source = str(raw.get("source", "") or "").strip()
    if not source:
        return _invalid("steering actuator calibration source is missing")
    try:
        lock = _number(raw.get("tyre_angle_per_input_rad"))
        command_delay = _number(raw.get("command_delay_s"))
        observation_delay = _number(raw.get("observation_delay_s"))
    except (TypeError, ValueError, OverflowError) as error:
        return _invalid(
            f"malformed steering actuator calibration: {error}",
            source=source, legacy=legacy)
    if not (MIN_TYRE_ANGLE_PER_INPUT_RAD <= lock
            <= MAX_TYRE_ANGLE_PER_INPUT_RAD):
        return _invalid(
            "tyre angle per normalized input is outside 0.60..0.95 rad",
            lock=lock, command_delay=command_delay,
            observation_delay=observation_delay, source=source,
            legacy=legacy)
    if not (MIN_COMMAND_DELAY_S <= command_delay <= MAX_COMMAND_DELAY_S):
        return _invalid(
            "steering command delay is outside 0.00..0.20 s",
            lock=lock, command_delay=command_delay,
            observation_delay=observation_delay, source=source,
            legacy=legacy)
    if not (MIN_OBSERVATION_DELAY_S <= observation_delay
            <= MAX_OBSERVATION_DELAY_S):
        return _invalid(
            "steering observation delay is outside 0.00..0.25 s",
            lock=lock, command_delay=command_delay,
            observation_delay=observation_delay, source=source,
            legacy=legacy)
    preview = command_delay + observation_delay
    if not (MIN_PREVIEW_HORIZON_S <= preview <= MAX_PREVIEW_HORIZON_S):
        return _invalid(
            "combined steering preview is outside 0.02..0.35 s",
            lock=lock, command_delay=command_delay,
            observation_delay=observation_delay, source=source,
            legacy=legacy)
    return SteeringActuatorCalibration(
        schema_version=schema,
        tyre_angle_per_input_rad=lock,
        command_delay_s=command_delay,
        observation_delay_s=observation_delay,
        source=source,
        legacy_lock_setting=legacy,
    )
