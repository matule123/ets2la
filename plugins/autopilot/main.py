from sdk.base_plugin import BasePlugin
from core.pid import PID
import logging

class Plugin(BasePlugin):
    """Autopilot plugin for lane keeping and speed control using PID."""

    def on_start(self):
        logging.info("Autopilot Plugin started with PID control.")
        self.enabled = True


        # PID for Steering: kp, ki, kd
        self.steering_pid = PID(kp=0.3, ki=0.01, kd=0.1)
        self.steering_pid.set_setpoint(0.0)


    def on_stop(self):
        logging.info("Autopilot Plugin stopped.")
        self.enabled = False

    def on_tick(self, delta_time: float):
        # 1. Telemetry & State
        speed = self.sdk.telemetry.get("truck", {}).get("speed", 0)
        speed_kmh = speed * 3.6 if speed < 200 else speed
        system_state = self.sdk.shared_state.get("system_state")
        danger_level = self.sdk.shared_state.get("danger_level", 0)
        lane_offset = self.sdk.shared_state.get("lane_offset", 0)

        # 2. Handle States with Hysteresis and Safety
        if system_state == "EMERGENCY":
            self.sdk.controller.stop_completely()
            logging.critical("EMERGENCY STOP TRIGGERED!")
            self.sdk.shared_state.set("tts_message", "Emergency stop triggered!")
            return

        if system_state == "PAY_TOLL":
            # Enhanced Toll Logic: Ensure complete stop before payment
            if speed_kmh > 0.5:
                self.sdk.controller.set_throttle(0)
                self.sdk.controller.set_brake(0.7) # Stronger braking for toll
                return
            else:
                self.sdk.controller.pay_toll()
                logging.info("Toll payment sequence executed - Truck Stopped.")
                return

        if system_state == "AVOID_OBSTACLE":
            # Dynamic braking based on danger level with a safety floor
            self.sdk.controller.set_throttle(0)
            brake_power = np.clip(0.5 + (0.5 * danger_level), 0.5, 1.0)
            self.sdk.controller.set_brake(brake_power)
            return

        # 3. ACC integration: Use outputs from the ACC plugin
        acc_throttle = self.sdk.shared_state.get("acc_throttle", 0.0)
        acc_brake = self.sdk.shared_state.get("acc_brake", 0.0)

        if acc_throttle is not None and acc_brake is not None:
            self.sdk.controller.set_throttle(acc_throttle)
            self.sdk.controller.set_brake(acc_brake)
            logging.debug(f"Autopilot using ACC outputs: T={acc_throttle:.2f}, B={acc_brake:.2f}")
        else:
            # Fallback if ACC plugin is disabled or not running
            self.sdk.controller.set_throttle(0.1)
            self.sdk.controller.set_brake(0)


        # 4. Steering Control with Recovery Mode
        nav_direction = self.sdk.shared_state.get("nav_direction", 0)

        target_offset = 0.0
        if abs(nav_direction) > 0.2:
            target_offset = nav_direction * 0.6

        self.steering_pid.set_setpoint(target_offset)
        steering_output = self.steering_pid.update(lane_offset, delta_time)

        # RECOVERY MODE: If the truck is too far from the center,
        # multiply the PID output to get back to the lane more aggressively.
        recovery_factor = 1.0
        if abs(lane_offset) > 0.4:
            recovery_factor = 1.5 + (abs(lane_offset) * 0.5)
            logging.debug(f"Recovery Mode Active: Factor {recovery_factor:.2f}")

        steering_val = np.clip(steering_output * recovery_factor, -1.0, 1.0)
        self.sdk.controller.set_steering(steering_val)
