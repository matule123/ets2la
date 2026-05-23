from sdk.base_plugin import BasePlugin
from core.pid import PID
import logging

class Plugin(BasePlugin):
    """Autopilot plugin for lane keeping and speed control using PID."""

    def on_start(self):
        logging.info("Autopilot Plugin started with PID control.")
        self.enabled = True
        self.target_speed = 80.0 # km/h

        # PID for Steering: kp, ki, kd
        self.steering_pid = PID(kp=0.3, ki=0.01, kd=0.1)
        self.steering_pid.set_setpoint(0.0)

        # PID for Throttle/Speed
        self.speed_pid = PID(kp=0.5, ki=0.05, kd=0.1)
        self.speed_pid.set_setpoint(self.target_speed)

    def on_stop(self):
        logging.info("Autopilot Plugin stopped.")
        self.enabled = False

    def on_tick(self, delta_time: float):
        # 1. Telemetry & State
        speed = self.sdk.telemetry.get("truck", {}).get("speed", 0)
        speed_kmh = speed * 3.6 if speed < 200 else speed
        system_state = self.sdk.shared_state.get("system_state")
        danger_level = self.sdk.shared_state.get("danger_level", 0)

        # 2. Handle States
        if system_state == "EMERGENCY":
            self.sdk.controller.stop_completely()
            logging.critical("EMERGENCY STOP TRIGGERED!")
            return

        if system_state == "PAY_TOLL":
            if speed_kmh > 2.0:
                self.sdk.controller.set_throttle(0)
                self.sdk.controller.set_brake(0.6)
                return
            else:
                self.sdk.controller.pay_toll()
                logging.info("Toll payment sequence executed.")
                return

        if system_state == "AVOID_OBSTACLE":
            # Dynamic braking based on danger level
            self.sdk.controller.set_throttle(0)
            self.sdk.controller.set_brake(0.8 * danger_level)
            return

        # 3. ACC 2.0: Smart Distance Keeping
        # Adjust target speed based on danger level (distance to vehicle in front)
        # Danger level 0.0 = clear, 0.3 = approaching vehicle
        effective_target_speed = self.target_speed
        if danger_level > 0.1:
            # Reduce speed linearly as danger level increases
            reduction_factor = 1.0 - (danger_level * 2) # 0.1 danger -> 80% speed, 0.3 danger -> 40% speed
            effective_target_speed = max(20.0, self.target_speed * max(0.2, reduction_factor))
            logging.debug(f"ACC 2.0: Reducing target speed to {effective_target_speed:.1f} km/h")

        throttle_output = self.speed_pid.update(speed_kmh, delta_time)
        # Use a dynamic setpoint for the PID to achieve the effective target speed
        self.speed_pid.set_setpoint(effective_target_speed)

        throttle_val = max(0.0, min(1.0, throttle_output))

        if speed_kmh > effective_target_speed + 5:
            self.sdk.controller.set_throttle(0)
            self.sdk.controller.set_brake(0.3)
        else:
            self.sdk.controller.set_throttle(throttle_val)
            self.sdk.controller.set_brake(0)

        # 4. Normal Operation: Steering Control (PID)
        nav_direction = self.sdk.shared_state.get("nav_direction", 0)
        lane_offset = self.sdk.shared_state.get("lane_offset", 0)

        target_offset = 0.0
        if abs(nav_direction) > 0.2:
            target_offset = nav_direction * 0.6

        self.steering_pid.set_setpoint(target_offset)
        steering_output = self.steering_pid.update(lane_offset, delta_time)

        steering_val = max(-1.0, min(1.0, steering_output))
        self.sdk.controller.set_steering(steering_val)
