import logging
from core.modules.base_module import BaseModule
from core.sdk import scs_sdk

class TelemetryModule(BaseModule):
    """
    Handles real-time telemetry data from the game.
    Equivalent to the TruckSimAPI in ets2la.
    """
    def __init__(self, engine):
        super().__init__(engine)
        self.sdk = scs_sdk.SCSSDK()
        self.current_data = {}

    def on_start(self):
        logging.info("TelemetryModule: Connecting to game telemetry...")

    def on_stop(self):
        logging.info("TelemetryModule: Stopping data acquisition.")

    def run(self, delta_time: float = 0.0):
        if not self.enabled:
            return

        # Update internal state from SDK
        self.current_data = self.sdk.get_telemetry()
        # Push to shared state for plugins to use
        self.engine.shared_state.set("telemetry", self.current_data)
        return self.current_data
