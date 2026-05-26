import logging
from core.modules.base_module import BaseModule
from core.sdk import scs_sdk

class ControlModule(BaseModule):
    """
    Handles sending control inputs back to the game.
    Equivalent to SDKController in ets2la.
    """
    def __init__(self, engine):
        super().__init__(engine)
        self.controller = scs_sdk.SCSController()

    def on_start(self):
        logging.info("ControlModule: Ready to send inputs.")

    def on_stop(self):
        logging.info("ControlModule: Releasing control.")

    def run(self, delta_time: float = 0.0):
        if not self.enabled:
            return

        # Read desired inputs from shared state
        throttle = self.engine.shared_state.get("acc_throttle", 0.0)
        brake = self.engine.shared_state.get("acc_brake", 0.0)
        steering = self.engine.shared_state.get("nav_steering", 0.0)

        # Send to game
        self.controller.set_throttle(throttle)
        self.controller.set_brake(brake)
        self.controller.set_steering(steering)
