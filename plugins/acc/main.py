import logging
import numpy as np
from sdk.base_plugin import BasePlugin
from core.pid import PID

class Plugin(BasePlugin):
    """
    Adaptive Cruise Control (ACC) Plugin.
    Maintains a safe distance from the vehicle in front by controlling speed.
    """

    def on_start(self):
        logging.info("ACC Plugin started with PID control.")
        self.enabled = True
        self.target_speed = 80.0  # km/h
        self.safe_distance = 50.0 # meters

        # Speed PID: kp, ki, kd
        self.speed_pid = PID(kp=0.5, ki=0.05, kd=0.1)
        self.speed_pid.set_setpoint(self.target_speed)

    def on_stop(self):
        logging.info("ACC Plugin stopped.")
        self.enabled = False

    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        # 1. Telemetry & State
        truck = self.sdk.telemetry.get("truck", {})
        speed = truck.get("speed", 0)
        speed_kmh = speed * 3.6 if speed < 200 else speed

        danger_level = self.sdk.shared_state.get("danger_level", 0)

        # 2. Dynamic Target Speed based on danger level
        effective_target_speed = self.target_speed
        if danger_level > 0.05:
            # Quadratic reduction for smoother deceleration
            reduction_factor = max(0.2, 1.0 - (danger_level ** 2 * 4))
            effective_target_speed = max(15.0, self.target_speed * reduction_factor)
            logging.debug(f"ACC: Adjusting target speed to {effective_target_speed:.1f} km/h")

        self.speed_pid.set_setpoint(effective_target_speed)
        throttle_output = self.speed_pid.update(speed_kmh, delta_time)
        throttle_val = np.clip(throttle_output, 0.0, 1.0)

        # 3. Output to shared state for the Controller to use
        if speed_kmh > effective_target_speed + 3:
            self.sdk.shared_state.set("acc_throttle", 0.0)
            self.sdk.shared_state.set("acc_brake", 0.2 + (0.1 * danger_level))
        else:
            self.sdk.shared_state.set("acc_throttle", throttle_val)
            self.sdk.shared_state.set("acc_brake", 0.0)

        # Also update a tag for UI
        self.tags.acc_speed = effective_target_speed
