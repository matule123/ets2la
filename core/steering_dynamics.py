"""Physical output dynamics for an already validated steering command.

This module does not choose a lane, alter a trajectory or calculate geometric
feedback.  It turns the normalized steering-angle request produced by
``Route.steering`` into a command that a real steering actuator can follow.
All limits are expressed per second and integrated with measured monotonic
``dt``; no moving average or stored target history is used.
"""

from __future__ import annotations

import math


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# Conservative estimated actuator parameters.  ``1.0`` normalized command is
# the usable 0.28 rad road-wheel angle defined by the geometric truck model.
STEERING_DYNAMICS_MIN_DT_S = 0.001
# A normal 60/100 Hz frame uses its real dt. Up to 100 ms is still integrated
# as real elapsed actuator time; a longer scheduler stall is never converted
# into one proportionally large steering jump.
STEERING_DYNAMICS_MAX_DT_S = 0.100
STEERING_MAX_COMMAND_LOW_SPEED = 1.00
STEERING_MAX_COMMAND_HIGH_SPEED = 0.55
STEERING_HIGH_SPEED_MS = 25.0
STEERING_LOW_SPEED_FULL_AUTHORITY_MS = 5.0
STEERING_RATE_LOW_SPEED_PER_S = 0.60
STEERING_RATE_HIGH_SPEED_PER_S = 0.38
# The first R18 replay used 4.0/s2 here.  That was physically smooth, but it
# added about 300 ms of command phase lag on top of the game's measured
# ~320 ms wheel response.  The truck consequently entered the prefab before
# the requested road-wheel rate had been established.  These bounds remain
# more than an order of magnitude below the old instantaneous 120/s2 reversal,
# while allowing a confirmed tight curve to build the existing 0.60/s rate in
# roughly one 50 ms simulation frame instead of modelling the game's actuator
# lag twice.  On a straight the separate 0.35 authority below keeps the same
# change deliberately softer.
STEERING_ACCEL_LOW_SPEED_PER_S2 = 14.0
STEERING_ACCEL_HIGH_SPEED_PER_S2 = 7.0
STEERING_STRAIGHT_DAMPING = 0.72
STEERING_STRAIGHT_ACCEL_DAMPING = 0.35
STEERING_CURVE_FULL_AUTHORITY_PER_M = 1.0 / 80.0
STEERING_DEMAND_FULL_AUTHORITY = 0.25
# Estimated 2% settling envelope of the commanded steering actuator.  The
# angle/rate/acceleration clamps remain the hard physical bounds; this
# critically damped target law only decides when to accelerate and brake.
STEERING_SETTLING_LOW_SPEED_S = 0.33
STEERING_SETTLING_HIGH_SPEED_S = 0.50
STEERING_CURVE_SETTLING_LOW_SPEED_S = 0.17
STEERING_CURVE_SETTLING_HIGH_SPEED_S = 0.30
STEERING_NOISE_DEADBAND = 0.004
STEERING_SETTLE_EPSILON = 1e-5


class SteeringDynamics:
    """Second-order, speed-scheduled normalized steering actuator.

    ``command`` is normalized steering angle and ``rate`` is normalized angle
    per second. Acceleration limits the first derivative's change. A critically
    damped current-target law prevents oscillatory settling without a sample-
    history average or a delayed copy of old geometry.
    """

    def __init__(self, initial_command: float = 0.0):
        self.command = _clamp(float(initial_command), -1.0, 1.0)
        self.rate = 0.0
        self.last_debug = self._empty_debug(self.command)

    @staticmethod
    def _empty_debug(command: float = 0.0) -> dict:
        return {
            "raw_target": float(command),
            "bounded_target": float(command),
            "output": float(command),
            "rate_per_s": 0.0,
            "acceleration_per_s2": 0.0,
            "dt_s": 0.0,
            "dt_used_s": 0.0,
            "speed_ms": 0.0,
            "max_command": 1.0,
            "max_rate_per_s": STEERING_RATE_LOW_SPEED_PER_S,
            "max_acceleration_per_s2": STEERING_ACCEL_LOW_SPEED_PER_S2,
            "natural_frequency_rad_s": 0.0,
            "settling_time_s": STEERING_SETTLING_LOW_SPEED_S,
            "target_saturated": False,
            "rate_limited": False,
            "acceleration_limited": False,
            "dt_limited": False,
            "deadband_active": False,
        }

    def reset(self, command: float = 0.0, rate: float = 0.0) -> float:
        """Synchronize with a proven current wheel command during handover."""
        self.command = _clamp(float(command), -1.0, 1.0)
        self.rate = float(rate) if math.isfinite(float(rate)) else 0.0
        self.last_debug = self._empty_debug(self.command)
        self.last_debug["rate_per_s"] = self.rate
        return self.command

    @staticmethod
    def _authority_fraction(curvature_per_m: float,
                            command_demand: float) -> float:
        curve_fraction = _clamp(
            abs(float(curvature_per_m))
            / STEERING_CURVE_FULL_AUTHORITY_PER_M, 0.0, 1.0)
        demand_fraction = _clamp(
            abs(float(command_demand)) / STEERING_DEMAND_FULL_AUTHORITY,
            0.0, 1.0)
        return max(curve_fraction, demand_fraction)

    @staticmethod
    def limits(speed_ms: float, curvature_per_m: float,
               command_demand: float = 0.0) -> tuple[float, float, float]:
        speed = _clamp(abs(float(speed_ms)), 0.0, STEERING_HIGH_SPEED_MS)
        # Parking/compact-prefab speeds retain the full low-speed actuator
        # rate.  Above 5 m/s the authority falls continuously with speed.
        speed_fraction = _clamp(
            (speed - STEERING_LOW_SPEED_FULL_AUTHORITY_MS)
            / (STEERING_HIGH_SPEED_MS
               - STEERING_LOW_SPEED_FULL_AUTHORITY_MS), 0.0, 1.0)
        max_command = (STEERING_MAX_COMMAND_LOW_SPEED
                       + (STEERING_MAX_COMMAND_HIGH_SPEED
                          - STEERING_MAX_COMMAND_LOW_SPEED) * speed_fraction)
        base_rate = (STEERING_RATE_LOW_SPEED_PER_S
                     + (STEERING_RATE_HIGH_SPEED_PER_S
                        - STEERING_RATE_LOW_SPEED_PER_S) * speed_fraction)
        base_accel = (STEERING_ACCEL_LOW_SPEED_PER_S2
                      + (STEERING_ACCEL_HIGH_SPEED_PER_S2
                         - STEERING_ACCEL_LOW_SPEED_PER_S2) * speed_fraction)
        # A large current geometric request is also evidence that damping must
        # not delay the beginning of a curve merely because the sampled local
        # curvature is still on the approach segment.  This uses only the
        # current request; it does not predict, average or rewrite geometry.
        authority_fraction = SteeringDynamics._authority_fraction(
            curvature_per_m, command_demand)
        rate_damping = (STEERING_STRAIGHT_DAMPING
                        + (1.0 - STEERING_STRAIGHT_DAMPING)
                        * authority_fraction)
        accel_damping = (STEERING_STRAIGHT_ACCEL_DAMPING
                         + (1.0 - STEERING_STRAIGHT_ACCEL_DAMPING)
                         * authority_fraction)
        return (max_command, base_rate * rate_damping,
                base_accel * accel_damping)

    @staticmethod
    def response(speed_ms: float,
                 authority_fraction: float = 0.0) -> tuple[float, float]:
        """Return estimated critical settling time and natural frequency."""
        speed = _clamp(abs(float(speed_ms)), 0.0, STEERING_HIGH_SPEED_MS)
        speed_fraction = _clamp(
            (speed - STEERING_LOW_SPEED_FULL_AUTHORITY_MS)
            / (STEERING_HIGH_SPEED_MS
               - STEERING_LOW_SPEED_FULL_AUTHORITY_MS), 0.0, 1.0)
        straight_settling = (STEERING_SETTLING_LOW_SPEED_S
                             + (STEERING_SETTLING_HIGH_SPEED_S
                                - STEERING_SETTLING_LOW_SPEED_S)
                             * speed_fraction)
        curve_settling = (STEERING_CURVE_SETTLING_LOW_SPEED_S
                          + (STEERING_CURVE_SETTLING_HIGH_SPEED_S
                             - STEERING_CURVE_SETTLING_LOW_SPEED_S)
                          * speed_fraction)
        authority = _clamp(float(authority_fraction), 0.0, 1.0)
        settling_time = (straight_settling
                         + (curve_settling - straight_settling) * authority)
        # A critically damped second-order system settles to about 2% in 4/w.
        return settling_time, 4.0 / settling_time

    def update(self, target: float, dt: float, *, speed_ms: float = 0.0,
               curvature_per_m: float = 0.0) -> float:
        try:
            raw_target = float(target)
            raw_dt = float(dt)
            speed = abs(float(speed_ms))
            curvature = float(curvature_per_m)
        except (TypeError, ValueError, OverflowError):
            raw_target, raw_dt, speed, curvature = 0.0, 0.0, 0.0, 0.0
        if not all(math.isfinite(value) for value in
                   (raw_target, raw_dt, speed, curvature)):
            raw_target, raw_dt, speed, curvature = 0.0, 0.0, 0.0, 0.0

        used_dt = _clamp(raw_dt, STEERING_DYNAMICS_MIN_DT_S,
                         STEERING_DYNAMICS_MAX_DT_S)
        # Releasing an established lock needs the same authority as acquiring
        # it; otherwise the wheel would enter a curve faster than it unwinds.
        command_demand = max(abs(raw_target), abs(self.command))
        authority_fraction = self._authority_fraction(
            curvature, command_demand)
        max_command, max_rate, max_accel = self.limits(
            speed, curvature, command_demand)
        settling_time, natural_frequency = self.response(
            speed, authority_fraction)
        bounded_target = _clamp(raw_target, -max_command, max_command)
        deadband_active = bool(
            abs(curvature) < 1.0 / 500.0
            and abs(bounded_target) <= STEERING_NOISE_DEADBAND
            and abs(self.command) <= STEERING_NOISE_DEADBAND * 2.0)
        if deadband_active:
            bounded_target = 0.0

        error = bounded_target - self.command
        if abs(error) <= STEERING_SETTLE_EPSILON and abs(self.rate) <= (
                max_accel * used_dt):
            previous_rate = self.rate
            self.command = bounded_target
            self.rate = 0.0
            acceleration = -previous_rate / used_dt
            rate_limited = acceleration_limited = False
        else:
            # Critically damped second-order target law (zeta = 1):
            #   command'' = w^2 * error - 2*w * command'
            # Hard physical rate/acceleration limits below remain authoritative.
            # Unlike a sample-history average this acts on the current target,
            # has explicit units and cannot retain an old steering direction.
            desired_acceleration = (
                natural_frequency * natural_frequency * error
                - 2.0 * natural_frequency * self.rate)
            acceleration = _clamp(
                desired_acceleration, -max_accel, max_accel)
            acceleration_limited = abs(
                desired_acceleration) > max_accel + 1e-9
            previous_rate = self.rate
            unrestricted_rate = previous_rate + acceleration * used_dt
            new_rate = _clamp(unrestricted_rate, -max_rate, max_rate)
            rate_limited = abs(unrestricted_rate) > max_rate + 1e-9
            # Report the acceleration actually integrated after a rate clamp.
            acceleration = (new_rate - previous_rate) / used_dt
            # Constant-acceleration kinematics. Semi-implicit Euler used the
            # end-of-frame rate over the whole tick, then the former crossing
            # guard snapped rate to zero at the target. Both made the measured
            # second derivative exceed its advertised bound. Trapezoidal
            # integration lets the critically damped law pass a tiny amount
            # through a target and reverse continuously on following ticks.
            new_command = (self.command
                           + 0.5 * (previous_rate + new_rate) * used_dt)
            self.command = _clamp(new_command, -max_command, max_command)
            self.rate = new_rate

        self.last_debug = {
            "raw_target": raw_target,
            "bounded_target": bounded_target,
            "output": self.command,
            "rate_per_s": self.rate,
            "acceleration_per_s2": acceleration,
            "dt_s": raw_dt,
            "dt_used_s": used_dt,
            "speed_ms": speed,
            "max_command": max_command,
            "max_rate_per_s": max_rate,
            "max_acceleration_per_s2": max_accel,
            "natural_frequency_rad_s": natural_frequency,
            "settling_time_s": settling_time,
            "target_saturated": abs(raw_target) > max_command + 1e-9,
            "rate_limited": bool(rate_limited),
            "acceleration_limited": bool(acceleration_limited),
            "dt_limited": abs(raw_dt - used_dt) > 1e-9,
            "deadband_active": deadband_active,
        }
        return float(self.command)
