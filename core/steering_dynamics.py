"""Physical output trajectory for an already validated steering command.

This module does not choose a lane, alter a trajectory or calculate geometric
feedback.  It turns the normalized steering-angle request produced by
``Route.steering`` into a command that a real steering actuator can follow.
All limits are expressed per second and integrated with measured monotonic
``dt``; no moving average, low-pass target or stored target history is used.

The game already contains the truck's steering response.  Modelling another
settling response here adds phase lag to the closed control loop: after a bend
the requested direction can reverse while the command is still following the
previous half-cycle.  This module therefore plans the quickest trajectory that
obeys the proven angle, rate and acceleration limits.  It starts braking the
angular rate at the physical stopping distance instead of delaying the current
geometric target through a second actuator model.
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
STEERING_NOISE_DEADBAND = 0.004
STEERING_SETTLE_EPSILON = 1e-5
# At the final few thousandths of normalized angle, the continuous
# sqrt(2*a*distance) stopping envelope can still cross the target inside one
# discrete control frame.  This 25 ms terminal horizon caps only that final
# velocity so a final from-rest acceleration does not create a correction
# larger than the current error. It acts on the current target and stores no
# samples; an already moving actuator still obeys its acceleration bound.
STEERING_TERMINAL_APPROACH_S = 0.025


class SteeringDynamics:
    """Speed-scheduled normalized angle/rate/acceleration trajectory.

    ``command`` is normalized steering angle and ``rate`` is normalized angle
    per second. Acceleration limits the first derivative's change.  The target
    itself is never filtered: a stopping-distance velocity profile reaches the
    newest target with the least phase delay permitted by the physical bounds.
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
            "settling_time_s": 0.0,
            "target_error": 0.0,
            "stopping_distance": 0.0,
            "safe_rate_per_s": 0.0,
            "terminal_rate_per_s": 0.0,
            "trajectory_phase": "settled",
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
        max_command, max_rate, max_accel = self.limits(
            speed, curvature, command_demand)
        bounded_target = _clamp(raw_target, -max_command, max_command)
        deadband_active = bool(
            abs(curvature) < 1.0 / 500.0
            and abs(bounded_target) <= STEERING_NOISE_DEADBAND
            and abs(self.command) <= STEERING_NOISE_DEADBAND * 2.0)
        if deadband_active:
            bounded_target = 0.0

        error = bounded_target - self.command
        previous_rate = self.rate
        stopping_distance = (
            previous_rate * previous_rate / (2.0 * max_accel)
            if max_accel > 1e-9 else float("inf"))
        continuous_safe_rate = math.sqrt(max(
            0.0, 2.0 * max_accel * abs(error)))
        terminal_rate = abs(error) / STEERING_TERMINAL_APPROACH_S
        safe_rate = min(continuous_safe_rate, terminal_rate)
        target_eta = (
            abs(error) / max(max_rate, 1e-9)
            + abs(previous_rate) / max(max_accel, 1e-9))
        if abs(error) <= STEERING_SETTLE_EPSILON and abs(self.rate) <= (
                max_accel * used_dt):
            self.command = bounded_target
            self.rate = 0.0
            acceleration = -previous_rate / used_dt
            rate_limited = acceleration_limited = False
            trajectory_phase = "settled"
        else:
            direction = 1.0 if error > 0.0 else -1.0
            desired_rate = direction * min(max_rate, safe_rate)
            requested_rate_delta = desired_rate - previous_rate
            allowed_rate_delta = max_accel * used_dt
            actual_rate_delta = _clamp(
                requested_rate_delta, -allowed_rate_delta,
                allowed_rate_delta)
            new_rate = _clamp(
                previous_rate + actual_rate_delta, -max_rate, max_rate)
            acceleration = (new_rate - previous_rate) / used_dt
            acceleration_limited = bool(
                abs(requested_rate_delta) > allowed_rate_delta + 1e-9)
            rate_limited = bool(
                safe_rate > max_rate + 1e-9
                and abs(new_rate) >= max_rate - 1e-9)
            signed_previous_rate = previous_rate * direction
            signed_new_rate = new_rate * direction
            if signed_previous_rate < -1e-9:
                trajectory_phase = "reverse"
            elif signed_new_rate + 1e-9 < signed_previous_rate:
                trajectory_phase = "brake"
            elif safe_rate <= max_rate + 1e-9:
                trajectory_phase = "approach"
            elif abs(new_rate) >= max_rate - 1e-9:
                trajectory_phase = "cruise"
            else:
                trajectory_phase = "accelerate"
            # Report the acceleration actually integrated after a rate clamp.
            # Constant-acceleration kinematics keep the command and its first
            # two derivatives inside the advertised physical envelope.  A
            # tiny target crossing is corrected on the next tick with the same
            # bounds; it is never hidden by snapping or temporal averaging.
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
            "natural_frequency_rad_s": 0.0,
            "settling_time_s": target_eta,
            "target_error": bounded_target - self.command,
            "stopping_distance": stopping_distance,
            "safe_rate_per_s": safe_rate,
            "terminal_rate_per_s": terminal_rate,
            "trajectory_phase": trajectory_phase,
            "target_saturated": abs(raw_target) > max_command + 1e-9,
            "rate_limited": bool(rate_limited),
            "acceleration_limited": bool(acceleration_limited),
            "dt_limited": abs(raw_dt - used_dt) > 1e-9,
            "deadband_active": deadband_active,
        }
        return float(self.command)
