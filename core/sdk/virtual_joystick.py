import vgamepad as vg
import logging

class VirtualJoystick:
    """
    Emulates a virtual Xbox 360 controller.
    Provides analog precision when the SDK DLL is not available.
    """
    def __init__(self):
        try:
            self.gamepad = vg.VX3Gamepad()
            logging.info("Virtual Joystick (vgamepad) initialized.")
        except Exception as e:
            logging.error(f"Failed to initialize vgamepad: {str(e)}")
            self.gamepad = None

    def set_steering(self, value: float):
        """
        Maps -1.0...1.0 to Xbox Axis range (-32768 to 32767)
        """
        if self.gamepad:
            # Left stick X axis for steering
            axis_value = int(value * 32767)
            self.gamepad.axis_left_rx(axis_value)
            self.gamepad.update()

    def set_throttle(self, value: float):
        """
        Maps 0.0...1.0 to Right Trigger (0 to 255)
        """
        if self.gamepad:
            trigger_value = int(value * 255)
            self.gamepad.right_trigger(trigger_value)
            self.gamepad.update()

    def set_brake(self, value: float):
        """
        Maps 0.0...1.0 to Left Trigger (0 to 255)
        """
        if self.gamepad:
            trigger_value = int(value * 255)
            self.gamepad.left_trigger(trigger_value)
            self.gamepad.update()
