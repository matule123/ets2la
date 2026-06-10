import logging
from sdk.base_plugin import BasePlugin


class Plugin(BasePlugin):
    """
    EcoDrive Plugin: improves fuel efficiency by telling the Autopilot to apply
    low-pass smoothing to throttle changes (prevents wasteful acceleration
    bursts).

    Design note: EcoDrive does NOT drive the device directly — that would fight
    the Autopilot over the throttle.  Instead it publishes ``eco_active`` and
    ``eco_smoothing`` which the Autopilot consumes when producing the final
    throttle intent.  Lower smoothing = gentler = more economical.
    """

    NAME = "ecodrive"
    DEFAULT_ENABLED = False  # opt-in

    def on_start(self):
        logging.info("EcoDrive Plugin started - Optimizing fuel efficiency.")
        self.smoothing_factor = 0.15  # Lower = smoother / more economical

    def on_stop(self):
        logging.info("EcoDrive Plugin stopped.")
        self.sdk.shared_state.set("eco_active", False)

    def on_tick(self, delta_time: float):
        # Publish eco parameters for the Autopilot to apply.
        self.sdk.shared_state.set("eco_active", True)
        self.sdk.shared_state.set("eco_smoothing", self.smoothing_factor)
        if self.tags is not None:
            self.tags.eco_active = True
            self.tags.eco_smoothing = self.smoothing_factor
