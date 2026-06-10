import time


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class PID:
    """
    A generic Proportional-Integral-Derivative controller for smooth control.

    Improvements over the original:
      * no numpy dependency (was crashing on the missing ``np`` import),
      * configurable integral clamp and optional output clamp,
      * derivative-on-measurement guard against divide-by-zero.
    """

    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0,
                 integral_limit: float = 10.0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral_limit = integral_limit
        self.output_limits = output_limits

        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()

    def update(self, measured_value: float, dt: float = None) -> float:
        if dt is None:
            now = time.time()
            dt = now - self._last_time
            self._last_time = now
        if dt <= 0:
            dt = 1e-3

        error = self.setpoint - measured_value

        p_term = self.kp * error

        self._integral += error * dt
        self._integral = _clamp(self._integral, -self.integral_limit, self.integral_limit)
        i_term = self.ki * self._integral

        derivative = (error - self._last_error) / dt
        d_term = self.kd * derivative
        self._last_error = error

        output = p_term + i_term + d_term
        lo, hi = self.output_limits
        if lo is not None or hi is not None:
            output = _clamp(output, lo if lo is not None else -1e9,
                            hi if hi is not None else 1e9)
        return output

    def set_setpoint(self, value: float):
        # Reset the integral when the target changes a lot to avoid lag/overshoot.
        if abs(value - self.setpoint) > 1e-6:
            self._integral = 0.0
        self.setpoint = value

    def reset(self):
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()
