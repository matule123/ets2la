from sdk.base_plugin import BasePlugin
import logging
import numpy as np

class Plugin(BasePlugin):
    """
    EcoDrive Plugin: Optimizes fuel consumption by smoothing throttle inputs
    and managing acceleration curves to prevent wasteful fuel consumption.
    """
    def on_start(self):
        logging.info("EcoDrive Plugin started - Optimizing fuel efficiency.")
        self.enabled = True
        self.last_throttle = 0.0
        self.smoothing_factor = 0.15 # Lower = smoother, more fuel efficient

    def on_stop(self):
        logging.info("EcoDrive Plugin stopped.")
        self.enabled = False

    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        # Get current target throttle from shared state or other plugins
        # Since other plugins call sdk.controller.set_throttle,
        # we intercept it by reading the 'target_throttle' tag if available.
        target_throttle = self.sdk.shared_state.get("target_throttle", 0.0)

        # Apply Exponential Smoothing (Low-pass filter)
        # current = (alpha * target) + ((1 - alpha) * previous)
        smoothed_throttle = (self.smoothing_factor * target_throttle) + ((1 - self.smoothing_factor) * self.last_throttle)

        # Prevent sudden bursts of acceleration
        self.last_throttle = smoothed_throttle

        # Apply the smoothed throttle to the controller
        self.sdk.controller.set_throttle(smoothed_throttle)

        # Log efficiency occasionally
        if np.random.random() < 0.01:
            logging.debug(f"EcoDrive: Smoothing throttle {target_throttle:.2f} -> {smoothed_throttle:.2f}")
