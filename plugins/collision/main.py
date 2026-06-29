import logging
import numpy as np
from sdk.base_plugin import BasePlugin
from plugins.collision.settings import settings


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

        system_state = self.sdk.get("system_state")
        obstacle = self.sdk.get("obstacle", {"level": 0, "position": "center"}) or {}
        danger_level = obstacle.get("level", 0) or 0

        # Engine stores system_state as the plain enum name (e.g. "EMERGENCY"),
        # not "SystemState.EMERGENCY" — match accordingly.
        if system_state == "EMERGENCY" or danger_level > settings.emergency_threshold:
            self.sdk.set("collision_brake_request", 1.0)
            self.tags.collision_status = "EMERGENCY BRAKE"
            return

        # Proportional braking as danger ramps up; 0 when the road is clear.
        # Combine the vision danger with the real lead-vehicle distance from
        # the ETS2LA traffic data — the vision signal alone often misses a car
        # right in front, so the real gap backs it up.
        vision_brake = 0.0
        if danger_level > 0.3:
            vision_brake = float(np.clip(0.3 + danger_level * 0.7, 0.0, 0.9))
        lead_dist = float(self.sdk.get("lead_distance", 0.0) or 0.0)
        lead_brake = 0.0
        if 0 < lead_dist < 15.0:
            lead_brake = float(np.clip((15.0 - lead_dist) / 15.0, 0.0, 0.9))
        brake = max(vision_brake, lead_brake)
        if brake > 0.01:
            self.sdk.set("collision_brake_request", brake)
            self.tags.collision_status = f"BRAKING {brake:.2f}"
        else:
            self.sdk.set("collision_brake_request", 0.0)
            self.tags.collision_status = "Clear"
