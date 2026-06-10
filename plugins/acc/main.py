import logging
import numpy as np
from sdk.base_plugin import BasePlugin
from core.pid import PID
from plugins.acc.settings import settings

class Plugin(BasePlugin):
    """
    Adaptive Cruise Control (ACC) Plugin.
    Maintains a safe distance from the vehicle in front by controlling speed,
    and (improvement over the original) automatically respects the in-game
    posted speed limit when ``obey_speed_limit`` is enabled.
    """

    NAME = "acc"

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
        truck = self.sdk.telemetry.get("truck", {}) or {}
        speed = truck.get("speed", 0) or 0
        # Handle both m/s and km/h inputs
        speed_kmh = abs(speed) * 3.6 if abs(speed) < 200 else abs(speed)

        # Get danger level from perception/traffic analysis
        danger_level = self.sdk.shared_state.get("danger_level", 0) or 0

        # 2. Emergency Collision Avoidance
        if danger_level > settings.emergency_brake_threshold:
            logging.warning("ACC: EMERGENCY BRAKING ACTIVE!")
            self.sdk.shared_state.set("acc_throttle", 0.0)
            self.sdk.shared_state.set("acc_brake", 1.0)
            self.sdk.shared_state.set("tts_message", "Collision alert! Emergency braking.")
            return

        # 3. Dynamic Target Speed Calculation
        # Start from the user's target (live override from the UI settings page,
        # falling back to the persisted default), never exceeding the posted limit.
        user_target = self.sdk.shared_state.get("acc_target_speed", None)
        base_target_speed = float(user_target) if user_target is not None else settings.target_speed
        effective_target_speed = base_target_speed
        obey_limit = self.sdk.shared_state.get("acc_obey_limit", None)
        obey_limit = bool(obey_limit) if obey_limit is not None else getattr(settings, "obey_speed_limit", True)
        if obey_limit:
            speed_limit_ms = truck.get("speedLimit", 0) or 0
            speed_limit_kmh = speed_limit_ms * 3.6 if speed_limit_ms < 200 else speed_limit_ms
            if speed_limit_kmh > 5:  # 0 means "unknown / no limit"
                effective_target_speed = min(effective_target_speed, speed_limit_kmh)

        if danger_level > 0.05:
            # Non-linear reduction for smoother approach: speed drops faster as danger increases
            reduction_factor = max(0.3, 1.0 - (danger_level ** 1.5 * 3))
            effective_target_speed = max(20.0, base_target_speed * reduction_factor)
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
