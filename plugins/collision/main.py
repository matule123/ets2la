import logging
from sdk.base_plugin import BasePlugin
from core.events import bus as event_bus

class Plugin(BasePlugin):
    """
    Collision Avoidance & Bypass Plugin.
    Monitors the environment and handles emergency braking and bypass steering.
    """
    def __init__(self, sdk_proxy):
        super().__init__(sdk_proxy)
        self.enabled = True
        logging.info("Collision Avoidance & Bypass Plugin initialized.")

    def on_start(self):
        logging.info("Collision Avoidance & Bypass Plugin started.")
        event_bus.subscribe("emergency_brake", self.on_emergency_brake)

    def on_stop(self):
        logging.info("Collision Avoidance & Bypass Plugin stopped.")

    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        system_state = self.sdk.get("system_state")
        obstacle = self.sdk.get("obstacle", {"level": 0, "position": "center"})
        danger_level = obstacle.get("level", 0)
        obstacle_pos = obstacle.get("position", "center")

        # Handle EMERGENCY state (Extreme danger)
        if system_state == "SystemState.EMERGENCY" or danger_level > 0.8:
            logging.warning("Collision Avoidance: EMERGENCY BRAKE!")
            self.trigger_emergency_stop()
            event_bus.publish("collision_alert", {"level": "CRITICAL", "action": "BRAKING"})
            return

        # Handle OVERTAKING / BYPASS state
        # If the planner decided to overtake, this plugin helps execute the steering
        if system_state == "SystemState.OVERTAKING":
            # If obstacle is center or left, we want to steer right
            steering_value = 0.3 if obstacle_pos in ["center", "left"] else -0.3

            # Log bypass maneuver
            logging.info(f"Collision Avoidance: Executing bypass steering {steering_value}")
            self.sdk.set("bypass_steering", steering_value)

            # Slightly reduce speed during the maneuver for safety
            self.sdk.set("acc_brake", 0.1)
        else:
            # Reset bypass steering when not in overtaking mode
            self.sdk.set("bypass_steering", 0.0)
            self.sdk.set("acc_brake", 0.0)

    def trigger_emergency_stop(self):
        self.sdk.set("emergency_brake", True)
        self.sdk.set("acc_brake", 1.0)

    def on_emergency_brake(self, data):
        logging.info(f"Collision Avoidance received emergency brake event: {data}")
        self.trigger_emergency_stop()
