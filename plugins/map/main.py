import logging
import math
import numpy as np
from sdk.base_plugin import BasePlugin

class Plugin(BasePlugin):
    """
    Map and Navigation plugin.
    Provides navigation cues and target offsets for the autopilot.
    """

    def on_start(self):
        logging.info("Map Plugin started.")
        self.enabled = True
        self.current_waypoint = None
        self.nav_direction = 0.0 # -1.0 to 1.0

    def on_stop(self):
        logging.info("Map Plugin stopped.")
        self.enabled = False

    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        # 1. Telemetry data
        truck = self.sdk.telemetry.get("truck", {})
        if not truck:
            return

        # In a full implementation, this would read from a map file or API.
        # For now, we implement a "virtual waypoint" that moves toward a target.
        truck_pos = self.sdk.shared_state.get("truck_pos", (0, 0))

        # Simple simulation of navigation:
        # If we are in 'NAVIGATING' state, we calculate direction to target.
        system_state = self.sdk.shared_state.get("system_state")
        if system_state == "NAVIGATING":
            target_pos = self.sdk.shared_state.get("target_pos", (1000, 1000))

            # Calculate angle to target
            dx = target_pos[0] - truck_pos[0]
            dz = target_pos[1] - truck_pos[1]
            angle_to_target = math.atan2(dx, dz)

            # Get truck rotation
            truck_rot = truck.get("rotation", 0)

            # Difference between truck rotation and target angle
            diff = angle_to_target - truck_rot
            # Normalize angle to -pi to pi
            diff = (diff + math.pi) % (2 * math.pi) - math.pi

            # Normalize diff to -1.0 to 1.0 for the autopilot
            self.nav_direction = np.clip(diff / math.pi, -1.0, 1.0)
            self.sdk.shared_state.set("nav_direction", self.nav_direction)
        else:
            self.nav_direction = 0.0
            self.sdk.shared_state.set("nav_direction", 0.0)

    def set_destination(self, x: float, z: float):
        """Set a new destination waypoint."""
        self.sdk.shared_state.set("target_pos", (x, z))
        self.sdk.shared_state.set("system_state", "NAVIGATING")
        logging.info(f"New destination set to: {x}, {z}")
