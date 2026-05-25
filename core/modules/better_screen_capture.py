import numpy as np
import cv2
import mss
import logging
from core.modules.base_module import BaseModule

class BetterScreenCapture(BaseModule):
    """
    High-performance screen capture module.
    Provides optimized methods to capture the game screen.
    """
    def __init__(self, engine):
        super().__init__(engine)
        self.sct = mss.mss()
        self.monitor = self.sct.monitors[1] if len(self.sct.monitors) > 1 else self.sct.monitors[0]
        self.capture_area = None # (left, top, width, height)
        logging.info("BetterScreenCapture module initialized.")

    def on_start(self):
        logging.info("BetterScreenCapture module started.")

    def on_stop(self):
        logging.info("BetterScreenCapture module stopped.")

    def set_capture_area(self, left, top, width, height):
        """Set the specific area of the screen to capture."""
        self.capture_area = {"left": left, "top": top, "width": width, "height": height}
        logging.info(f"Capture area set to: {self.capture_area}")

    def capture(self, cropped=True):
        """
        Capture the screen.
        :param cropped: If True, return only the capture_area. Otherwise, return full screen.
        :return: numpy array of the captured frame.
        """
        try:
            if cropped and self.capture_area:
                screenshot = self.sct.grab(self.capture_area)
            else:
                screenshot = self.sct.grab(self.monitor)

            # Convert to numpy array and BGR (OpenCV format)
            frame = np.array(screenshot)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            return frame
        except Exception as e:
            logging.error(f"Error capturing screen: {e}")
            return None

    def update(self, delta_time: float):
        pass
