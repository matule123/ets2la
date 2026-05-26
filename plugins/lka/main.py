import logging
from sdk.base_plugin import BasePlugin
import numpy as np

class Plugin(BasePlugin):
    """
    Lane Keep Assist (LKA) Plugin.
    Uses perception data to keep the truck centered in the lane.
    """
    def __init__(self, sdk_proxy):
        super().__init__(sdk_proxy)
        self.enabled = True
        self.centering_strength = 0.4
        logging.info("LKA Plugin initialized.")

    def on_start(self):
        logging.info("LKA Plugin started.")

    def on_stop(self):
        logging.info("LKA Plugin stopped.")

    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        # 1. Get Lane Deviation (from core/modules/perception)
        # In a real system, this would come from a CV module analyzing the screen
        deviation = self.sdk.shared_state.get("lane_deviation", 0.0) # -1.0 (left) to 1.0 (right)

        if abs(deviation) < 0.05:
            return # We are centered

        # 2. Calculate Steering Correction
        # If deviation is positive (right), we need to steer left (negative value)
        correction = -deviation * self.centering_strength

        # 3. Apply Correction to Shared State
        current_steering = self.sdk.shared_state.get("nav_steering", 0.0)
        new_steering = np.clip(current_steering + correction, -1.0, 1.0)

        self.sdk.shared_state.set("nav_steering", new_steering)
        self.tags.lka_active = True if abs(deviation) > 0.1 else False
        self.tags.lka_deviation = deviation
