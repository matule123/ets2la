import ctypes
import os
import logging
from typing import Optional

class SCSController:
    """
    Interface for the SCS SDK Controller DLL.
    This allows direct memory writing/input simulation for ETS2.
    """
    def __init__(self, dll_path: str = "sdk/scs_sdk_controller.dll"):
        self.dll_path = dll_path
        self.dll = None
        self._load_dll()

    def _load_dll(self):
        if not os.path.exists(self.dll_path):
            logging.warning(f"SCS SDK DLL not found at {self.dll_path}. Precise control disabled.")
            return

        try:
            self.dll = ctypes.CDLL(self.dll_path)
            logging.info("SCS SDK Controller DLL loaded successfully.")
        except Exception as e:
            logging.error(f"Failed to load SCS SDK DLL: {str(e)}")

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
