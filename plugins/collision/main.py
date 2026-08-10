import logging
import time
from sdk.base_plugin import BasePlugin


class Plugin(BasePlugin):
    """
    Collision Avoidance plugin.

    Watches perception danger and publishes a *brake request* that the Autopilot
    combines (via max) with the ACC brake.  It never writes ``acc_brake`` or the
    final controls directly — that used to fight the ACC plugin every tick.

    Cross-process note: the old version used an in-process ``event_bus`` to talk
    to the engine, which never worked across process boundaries.  All coordination
    now goes through shared state, consistent with the rest of the system.
    """

    NAME = "collision"

    def on_start(self):
        logging.info("Collision Avoidance plugin started.")
        self.enabled = True

    def on_stop(self):
        logging.info("Collision Avoidance plugin stopped.")
        self.sdk.set("collision_brake_request", 0.0)

    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        # Only the lane-aligned SCS traffic calculation has actuator
        # authority. Raw screenshot perception remains available under
        # ``vision_obstacle_diagnostic`` but cannot prove depth or lane and
        # therefore cannot stop the truck.
        try:
            traffic_age = time.monotonic() - float(
                self.sdk.get("traffic_snapshot_timestamp", 0.0) or 0.0)
        except (TypeError, ValueError, OverflowError):
            traffic_age = float("inf")
        if (not self.sdk.get("traffic_snapshot_valid", False)
                or not 0.0 <= traffic_age <= 0.5):
            brake = 0.0
        else:
            try:
                brake = max(0.0, min(1.0, float(
                    self.sdk.get("traffic_brake", 0.0) or 0.0)))
            except (TypeError, ValueError, OverflowError):
                brake = 0.0
        if brake > 0.01:
            self.sdk.set("collision_brake_request", brake)
            self.tags.collision_status = f"SCS TRAFFIC {brake:.2f}"
        else:
            self.sdk.set("collision_brake_request", 0.0)
            self.tags.collision_status = "Clear"
