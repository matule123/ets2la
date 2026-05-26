import logging
import numpy as np
from sdk.base_plugin import BasePlugin
from core.pid import PID
from plugins.acc.settings import settings

class Plugin(BasePlugin):
    """
    Adaptive Cruise Control (ACC) Plugin.
    Maintains a safe distance from the vehicle in front by controlling speed.
    """

    def on_start(self):
        logging.info("ACC Plugin started with professional PID control.")
        self.enabled = True

        # Initialize PID for speed maintenance
        # Kp: Proportional (fast response), Ki: Integral (eliminates steady-state error), Kd: Derivative (damps oscillations)
        self.speed_pid = PID(kp=0.6, ki=0.1, kd=0.05)
        self.speed_pid.set_setpoint(settings.target_speed)

    def on_stop(self):
        logging.info("ACC Plugin stopped.")
        self.enabled = False

    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        # 1. Telemetry & State
        truck = self.sdk.telemetry.get("truck", {})
        speed = truck.get("speed", 0)
        # Handle both m/s and km/h inputs
        speed_kmh = speed * 3.6 if speed < 200 else speed

        # Get danger level from perception/traffic analysis
        danger_level = self.sdk.shared_state.get("danger_level", 0)

        # 2. Emergency Collision Avoidance
        if danger_level > settings.emergency_brake_threshold:
            logging.warning("ACC: EMERGENCY BRAKING ACTIVE!")
            self.sdk.shared_state.set("acc_throttle", 0.0)
            self.sdk.shared_state.set("acc_brake", 1.0)
            self.sdk.shared_state.set("tts_message", "Collision alert! Emergency braking.")
            return

        # 3. Dynamic Target Speed Calculation
        # Blend target speed with a "safe" speed based on distance/danger
        effective_target_speed = settings.target_speed
        if danger_level > 0.05:
            # Non-linear reduction for smoother approach: speed drops faster as danger increases
            reduction_factor = max(0.3, 1.0 - (danger_level ** 1.5 * 3))
            effective_target_speed = max(20.0, settings.target_speed * reduction_factor)
            logging.debug(f"ACC: Adjusting target speed to {effective_target_speed:.1f} km/h due to traffic")

        self.speed_pid.set_setpoint(effective_target_speed)
        throttle_output = self.speed_pid.update(speed_kmh, delta_time)

        # 4. Control Output Mapping
        # Map PID output to throttle (0 to 1) and brake (0 to 1)
        throttle_val = np.clip(throttle_output, 0.0, 1.0)

        if speed_kmh > effective_target_speed + 2:
            # Gradual braking based on overspeed
            brake_power = np.clip((speed_kmh - effective_target_speed) / 15.0, 0.1, 0.6)
            self.sdk.shared_state.set("acc_throttle", 0.0)
            self.sdk.shared_state.set("acc_brake", brake_power)
        else:
            self.sdk.shared_state.set("acc_throttle", throttle_val)
            self.sdk.shared_state.set("acc_brake", 0.0)

        # Update UI tags
        self.tags.acc_speed = effective_target_speed
        self.tags.acc_status = "Active" if self.enabled else "Disabled"
