import logging
import numpy as np
from sdk.base_plugin import BasePlugin
from core.pid import PID


class Plugin(BasePlugin):
    """
    Autopilot plugin — the single authority that turns perception + ACC outputs
    into the final control intents (steering / throttle / brake).

    It integrates:
      * ACC plugin throttle/brake targets,
      * EcoDrive throttle smoothing (when active),
      * lane offset + navigation direction into a PID-driven steering signal,
      * planner states (EMERGENCY / PAY_TOLL / AVOID_OBSTACLE) for safety.
    """

    NAME = "autopilot"

    def on_start(self):
        logging.info("Autopilot Plugin started with PID control.")
        self.enabled = True
        self.steering_pid = PID(kp=0.3, ki=0.01, kd=0.1)
        self.steering_pid.set_setpoint(0.0)
        self._last_throttle = 0.0
        self._last_steering = 0.0

    def on_stop(self):
        logging.info("Autopilot Plugin stopped.")
        self.enabled = False

    def _apply_throttle(self, throttle: float):
        """Apply EcoDrive low-pass smoothing to throttle if eco is active."""
        if self.sdk.shared_state.get("eco_active", False):
            alpha = float(self.sdk.shared_state.get("eco_smoothing", 0.15))
            throttle = (alpha * throttle) + ((1 - alpha) * self._last_throttle)
        self._last_throttle = throttle
        self.sdk.controller.set_throttle(throttle)

    def on_tick(self, delta_time: float):
        # 1. Telemetry & state
        speed = self.sdk.telemetry.get("truck", {}).get("speed", 0) or 0
        speed_kmh = speed * 3.6 if speed < 200 else speed
        system_state = self.sdk.shared_state.get("system_state")
        danger_level = self.sdk.shared_state.get("danger_level", 0) or 0
        lane_offset = self.sdk.shared_state.get("lane_offset", 0) or 0

        # 2. Safety states
        if system_state == "EMERGENCY":
            self.sdk.controller.stop_completely()
            self.sdk.shared_state.set("tts_message", "Emergency stop triggered!")
            return

        if system_state == "PAY_TOLL":
            if speed_kmh > 0.5:
                self.sdk.controller.set_throttle(0)
                self.sdk.controller.set_brake(0.7)
            else:
                self.sdk.controller.pay_toll()
            return

        # Brake requests: collision plugin (CV) + traffic-following (real lead car).
        collision_brake = float(self.sdk.shared_state.get("collision_brake_request", 0.0) or 0.0)
        traffic_brake = float(self.sdk.shared_state.get("traffic_brake", 0.0) or 0.0)
        light_brake = float(self.sdk.shared_state.get("light_brake", 0.0) or 0.0)
        collision_brake = max(collision_brake, traffic_brake, light_brake)

        if system_state == "AVOID_OBSTACLE":
            self.sdk.controller.set_throttle(0)
            brake_power = float(np.clip(0.5 + (0.5 * danger_level), 0.5, 1.0))
            self.sdk.controller.set_brake(max(brake_power, collision_brake))
            return

        # 3. Longitudinal control from ACC outputs
        acc_throttle = self.sdk.shared_state.get("acc_throttle", None)
        acc_brake = self.sdk.shared_state.get("acc_brake", None)
        if acc_throttle is not None and acc_brake is not None:
            brake = max(float(acc_brake), collision_brake)
            # Don't accelerate while any brake is requested.
            self._apply_throttle(0.0 if brake > 0.01 else float(acc_throttle))
            self.sdk.controller.set_brake(brake)
        else:
            # Fallback if ACC plugin is disabled / not running yet: hold a gentle
            # cruise so the truck keeps rolling (0.1 was too low — it stalled).
            self._apply_throttle(0.0 if collision_brake > 0.01 else 0.35)
            self.sdk.controller.set_brake(collision_brake)

        # 4. Lateral control.
        #    When the navigation plugin is following a recorded route, its
        #    coordinate-based steering (cross-track + heading error) is the
        #    primary lateral signal — far more reliable than screen CV.  Smooth
        #    it across frames to avoid jerky wheel motion.
        nav_active = bool(self.sdk.shared_state.get("nav_active", False))
        if nav_active:
            nav_steering = float(self.sdk.shared_state.get("nav_steering", 0.0) or 0.0)
            # Heavier low-pass + per-tick rate limit so the wheel never snaps.
            target = (0.18 * nav_steering) + (0.82 * self._last_steering)
            max_step = 0.05  # max steering change per tick
            delta = float(np.clip(target - self._last_steering, -max_step, max_step))
            self._last_steering = float(np.clip(self._last_steering + delta, -1.0, 1.0))
            steering_val = self._last_steering
        else:
            # No reliable lateral source (no route, no lane detection).
            # Decay smoothly toward straight instead of reacting to CV noise —
            # that random jitter is what made the wheel twitch with no route.
            nav_direction = self.sdk.shared_state.get("nav_direction", 0) or 0
            have_lane = abs(lane_offset) > 0.02 or abs(nav_direction) > 0.05
            if have_lane:
                target_offset = nav_direction * 0.6 if abs(nav_direction) > 0.2 else 0.0
                self.steering_pid.set_setpoint(target_offset)
                steering_output = self.steering_pid.update(lane_offset, delta_time)
                speed_damping = float(np.clip(1.0 - (speed_kmh / 120.0) * 0.3, 0.7, 1.0))
                center_force = -lane_offset * 0.05
                recovery = 1.5 + (abs(lane_offset) * 0.5) if abs(lane_offset) > 0.4 else 1.0
                raw = float(np.clip((steering_output * speed_damping) + center_force * recovery, -1.0, 1.0))
            else:
                raw = 0.0  # hold straight
            # Rate-limit this branch too, so it can never snap the wheel.
            delta = float(np.clip(raw - self._last_steering, -0.05, 0.05))
            self._last_steering = float(np.clip(self._last_steering + delta, -1.0, 1.0))
            steering_val = self._last_steering

        self.sdk.controller.set_steering(steering_val)

        # Publish UI tags.
        self.tags.steering = round(steering_val, 3)
        self.tags.speed_kmh = round(speed_kmh, 1)
        self.tags.nav_active = nav_active
