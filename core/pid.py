import time

class PID:
    """
    A generic Proportional-Integral-Derivative controller for smooth control.
    """
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.setpoint = setpoint

        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()

    def update(self, measured_value: float, dt: float = None) -> float:
        """
        Calculates the control output based on the current measured value.
        """
        if dt is None:
            now = time.time()
            dt = now - self._last_time
            self._last_time = now

        error = self.setpoint - measured_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self._integral += error * dt
        i_term = self.ki * self._integral

        # Derivative term
        derivative = (error - self._last_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative

        self._last_error = error

        return p_term + i_term + d_term

    def set_setpoint(self, value: float):
        self.setpoint = value

    def reset(self):
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()
