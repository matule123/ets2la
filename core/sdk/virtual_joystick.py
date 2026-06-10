import vgamepad as vg
import logging


class VirtualJoystick:
    """
    Emulates a virtual Xbox 360 controller for analog steering/throttle/brake
    when the SCS SDK DLL is not available.

    Uses vgamepad's float API (the previous code called ``VX3Gamepad`` and
    ``axis_left_rx`` which don't exist, so analog control silently never worked
    and everything fell back to jerky keyboard input).
    """

    def __init__(self):
        try:
            self.gamepad = vg.VX360Gamepad()
            self._steer = 0.0
            self._rt = 0.0
            self._lt = 0.0
            logging.info("Virtual Joystick (vgamepad VX360) initialized.")
        except Exception as e:
            logging.error(f"Failed to initialize vgamepad: {str(e)}")
            self.gamepad = None

    def _flush(self):
        # Steering on the left-stick X axis; triggers for throttle/brake.
        self.gamepad.left_joystick_float(x_value_float=self._steer, y_value_float=0.0)
        self.gamepad.right_trigger_float(value_float=self._rt)
        self.gamepad.left_trigger_float(value_float=self._lt)
        self.gamepad.update()

    def set_steering(self, value: float):
        """value: -1.0 .. 1.0 (left .. right)."""
        if self.gamepad:
            self._steer = max(-1.0, min(1.0, value))
            self._flush()

    def set_throttle(self, value: float):
        """value: 0.0 .. 1.0 → right trigger."""
        if self.gamepad:
            self._rt = max(0.0, min(1.0, value))
            self._flush()

    def set_brake(self, value: float):
        """value: 0.0 .. 1.0 → left trigger."""
        if self.gamepad:
            self._lt = max(0.0, min(1.0, value))
            self._flush()
