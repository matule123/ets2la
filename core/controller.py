import pydirectinput
import logging
from core.sdk.precision_driver import SCSController
from core.sdk.virtual_joystick import VirtualJoystick

class Controller:
    """
    Advanced Input Controller for ETS2.
    Prioritizes: SCS SDK DLL -> Virtual Joystick (vgamepad) -> Digital Keys (pydirectinput)
    """
    def __init__(self):
        logging.info("Initializing Professional Controller...")
        pydirectinput.FAILSAFE = False

        # 1. Try SDK DLL (Absolute Precision)
        self.sdk_driver = SCSController()

        # 2. Try Virtual Joystick (Analog Precision)
        self.vjoy = VirtualJoystick()

        # Determine current precision level
        if self.sdk_driver.dll:
            self.mode = "SDK"
            logging.info("Control Mode: [PRECISION SDK]")
        elif self.vjoy.gamepad:
            self.mode = "VJOY"
            logging.info("Control Mode: [ANALOG VIRTUAL JOYSTICK]")
        else:
            self.mode = "DIGITAL"
            logging.warning("Control Mode: [DIGITAL FALLBACK] - Steering will be jerky")

    def set_steering(self, value: float):
        """Value: -1.0 (full left) to 1.0 (full right)"""
        if self.mode == "SDK":
            self.sdk_driver.set_steering(value)
        elif self.mode == "VJOY":
            self.vjoy.set_steering(value)
        else:
            # Digital Fallback
            if value < -0.1:
                pydirectinput.keyDown('a')
                pydirectinput.keyUp('d')
            elif value > 0.1:
                pydirectinput.keyDown('d')
                pydirectinput.keyUp('a')
            else:
                pydirectinput.keyUp('a')
                pydirectinput.keyUp('d')

    def set_throttle(self, value: float):
        """Value: 0.0 to 1.0"""
        if self.mode == "SDK":
            self.sdk_driver.set_throttle(value)
        elif self.mode == "VJOY":
            self.vjoy.set_throttle(value)
        else:
            # Digital Fallback
            if value > 0.1:
                pydirectinput.keyDown('w')
            else:
                pydirectinput.keyUp('w')

    def set_brake(self, value: float):
        """Value: 0.0 to 1.0"""
        if self.mode == "SDK":
            self.sdk_driver.set_brake(value)
        elif self.mode == "VJOY":
            self.vjoy.set_brake(value)
        else:
            # Digital Fallback
            if value > 0.1:
                pydirectinput.keyDown('s')
            else:
                pydirectinput.keyUp('s')

    def stop_completely(self):
        """Brakes and stops the truck."""
        self.set_throttle(0)
        self.set_brake(1.0)

    def pay_toll(self):
        """Simulates paying a toll by pressing the interaction key."""
        logging.info("Paying toll...")
        pydirectinput.press('e')
