import ctypes
import os
import logging
from typing import Optional

class SCSController:
    """
    Interface for the SCS SDK Controller DLL.
    This allows direct memory writing/input simulation for ETS2.
    """
    #: ctypes exports a usable control DLL must provide.
    REQUIRED_EXPORTS = ("SetSteering", "SetThrottle", "SetBrake")

    def __init__(self, dll_path: str = None):
        if dll_path is None:
            from core.paths import resource
            dll_path = resource("assets", "scs_sdk_controller.dll")
        self.dll_path = dll_path
        self.dll = None
        self._load_dll()

    def _load_dll(self):
        if not os.path.exists(self.dll_path):
            logging.info(f"SCS control DLL not found at {self.dll_path}; using virtual joystick.")
            return

        try:
            dll = ctypes.CDLL(self.dll_path)
        except Exception as e:
            logging.error(f"Failed to load SCS control DLL: {e}")
            return

        # The SCS *plugin* DLL drives the game via Local\\SCSControls shared
        # memory, not ctypes calls — it has no SetSteering/SetThrottle exports.
        # Only claim this backend if the DLL actually exposes them, otherwise we
        # would silently send no input. The Controller then falls back to vgamepad.
        if all(hasattr(dll, name) for name in self.REQUIRED_EXPORTS):
            self.dll = dll
            logging.info("SCS SDK Controller DLL loaded (ctypes control available).")
        else:
            logging.info("SCS control DLL has no ctypes control exports; "
                         "using virtual joystick instead.")

    def set_steering(self, value: float):
        """Value: -1.0 to 1.0"""
        if self.dll and hasattr(self.dll, 'SetSteering'):
            self.dll.SetSteering(ctypes.c_float(value))
        else:
            # Fallback is handled by the main Controller class
            pass

    def set_throttle(self, value: float):
        """Value: 0.0 to 1.0"""
        if self.dll and hasattr(self.dll, 'SetThrottle'):
            self.dll.SetThrottle(ctypes.c_float(value))

    def set_brake(self, value: float):
        """Value: 0.0 to 1.0"""
        if self.dll and hasattr(self.dll, 'SetBrake'):
            self.dll.SetBrake(ctypes.c_float(value))
