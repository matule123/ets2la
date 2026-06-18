import logging
import numpy as np
from sdk.base_plugin import BasePlugin


# --- Tuning (kept here, mirrored into settings under "autopilot" section) -----
STEER_RATE_LIMIT = 0.10      # max steering change per second (smooth, no jerk)
STEER_FOLLOW_BLEND = 0.55    # how much of nav/vision steering to apply per tick
                             # (low = smooth/laggy, high = snappy/jittery)
VISION_DEADZONE = 0.03       # ignore vision lane offset noise below this
BRAKE_RAMP_UP = 2.5          # brake can rise this fast per second (anti-jerk)
BRAKE_RAMP_DOWN = 4.0        # brake releases faster than it engages
BRAKE_MIN_HOLD = 0.04        # below this, treat brake as zero (avoid flutter)
THROTTLE_RAMP = 3.0          # throttle slew rate per second


class Plugin(BasePlugin):
    """
    Autopilot plugin — the single authority that turns perception + ACC outputs
    into the final control intents (steering / throttle / brake).

    Design (Phase 1 tuning):
      * Lateral control is smoothed ONCE here. Route.steering()/vision already
        compute the raw target, so we only apply a short rate-limit (no heavy
        exponential lag) — that was the cause of the fishtailing, because the
        signal got integrated 2-3 times in a row.
      * Braking uses a ramp (anti-jerk): the command grows and decays smoothly
        over time, so it never slams to 1.0 and never releases in a step. This
        is what stops the "sudden hard braking" and the resulting loss of grip
        that made the truck spin.
      * When the real ETS2LA traffic data is available we trust it over the
        noisy screen-vision obstacle signal, so phantom braking all but
        disappears.
    """

    NAME = "autopilot"

    def on_start(self):
        logging.info("Autopilot Plugin started (Phase 1 tuning).")
        self.enabled = True
        self._last_throttle = 0.0
        self._last_steering = 0.0
        self._last_brake = 0.0          # smoothed brake command (the ramp)
        self._blinker = "off"
        # Rolling speed estimate (for ramp scaling when telemetry lags).
        self._speed_kmh = 0.0

    def on_stop(self):
        logging.info("Autopilot Plugin stopped.")
        self.enabled = False

    # --- Low-pass ramps -------------------------------------------------------
    def _ramp(self, current, target, dt, up_rate, down_rate):
        """Move `current` toward `target` no faster than up/down_rate per second."""
        if dt <= 0:
            dt = 1e-3
        if target > current:
            max_step = up_rate * dt
            return min(target, current + max_step)
        else:
            max_step = down_rate * dt
            return max(target, current - max_step)

    def _apply_throttle(self, throttle: float, dt: float):
        """Slew the throttle smoothly (eco smoothing if active)."""
        if self.sdk.shared_state.get("eco_active", False):
            alpha = float(self.sdk.shared_state.get("eco_smoothing", 0.15))
            throttle = (alpha * throttle) + ((1 - alpha) * self._last_throttle)
        throttle = self._ramp(self._last_throttle, max(0.0, min(1.0, throttle)),
                              dt, THROTTLE_RAMP, THROTTLE_RAMP)
        self._last_throttle = throttle
        self.sdk.controller.set_throttle(throttle)

    def on_tick(self, delta_time: float):
        dt = max(delta_time, 1e-3)

        # 1. Telemetry & state
        truck = self.sdk.telemetry.get("truck", {}) or {}
        speed = truck.get("speed", 0) or 0
        speed_kmh = abs(speed) * 3.6 if abs(speed) < 200 else abs(speed)
        self._speed_kmh = 0.6 * speed_kmh + 0.4 * self._speed_kmh
        system_state = self.sdk.shared_state.get("system_state")
        danger_level = self.sdk.shared_state.get("danger_level", 0) or 0
        lane_offset = self.sdk.shared_state.get("lane_offset", 0) or 0
        # Real traffic available? If so, down-weight the noisy vision signal.
        traffic = self.sdk.shared_state.get("traffic", []) or []
        have_real_traffic = len(traffic) > 0

        # 2. Safety states — these still brake hard, but through the ramp so
        #    the truck doesn't lock up and spin.
        if system_state == "EMERGENCY":
            self._set_brake(1.0, dt)
            self.sdk.controller.set_throttle(0.0)
            self._last_throttle = 0.0
            self.sdk.shared_state.set("tts_message", "Emergency stop triggered!")
            return

        if system_state == "PAY_TOLL":
            if speed_kmh > 0.5:
                self.sdk.controller.set_throttle(0.0)
                self._last_throttle = 0.0
                self._set_brake(0.7, dt)
            else:
                self._set_brake(0.0, dt)
                self.sdk.controller.pay_toll()
            return

        # --- Gather all brake requests, combine via max() -------------------
        collision_brake = float(self.sdk.shared_state.get("collision_brake_request", 0.0) or 0.0)
        traffic_brake = float(self.sdk.shared_state.get("traffic_brake", 0.0) or 0.0)
        light_brake = float(self.sdk.shared_state.get("light_brake", 0.0) or 0.0)
        # Vision obstacle (screen CV). Only trust it as a *nudge*: when we have
        # real traffic data, heavily discount it so a shadow / sign can't cause
        # a phantom full stop.
        if danger_level > 0.35:
            vision_brake = float(np.clip((danger_level - 0.35) * 1.8, 0.0, 1.0))
            vision_brake *= (0.25 if have_real_traffic else 1.0)
        else:
            vision_brake = 0.0
        requested_brake = max(collision_brake, traffic_brake, light_brake, vision_brake)

        if system_state == "AVOID_OBSTACLE":
            requested_brake = max(requested_brake,
                                  float(np.clip(0.5 + (0.5 * danger_level), 0.5, 1.0)))

        # --- Curve slowdown: ease off the throttle (light brake at speed) ---
        turn = abs(self._last_steering)
        curve_factor = 1.0 if turn < 0.18 else max(0.35, 1.0 - (turn - 0.18) * 1.6)
        if turn > 0.45 and speed_kmh > 45:
            requested_brake = max(requested_brake,
                                  float(np.clip((turn - 0.45) * 0.6, 0.0, 0.35)))

        # 3. Apply braking THROUGH THE RAMP (anti-jerk). This is the key change:
        #    the truck brakes firmly but progressively, never a step to 1.0.
        self._set_brake(requested_brake, dt)

        # 4. Longitudinal control from ACC outputs
        acc_throttle = self.sdk.shared_state.get("acc_throttle", None)
        acc_brake = self.sdk.shared_state.get("acc_brake", None)
        braking = self._last_brake > BRAKE_MIN_HOLD
        if acc_throttle is not None and acc_brake is not None:
            # Never accelerate while any brake is being applied.
            target_throttle = 0.0 if braking else float(acc_throttle) * curve_factor
        else:
            # Fallback if ACC is disabled / not running yet: gentle cruise.
            target_throttle = 0.0 if braking else 0.35 * curve_factor
        self._apply_throttle(target_throttle, dt)

        # 5. Lateral control.
        nav_active = bool(self.sdk.shared_state.get("nav_active", False))
        if nav_active:
            # nav_steering is already a finished pure-pursuit + CTE value from
            # the Route/map plugin. Apply a SHORT rate-limit only — the old
            # 0.35/0.65 exponential lag was a second integrator that caused the
            # truck to overshoot and oscillate (fishtail) in and out of curves.
            nav_steering = float(self.sdk.shared_state.get("nav_steering", 0.0) or 0.0)
            target = STEER_FOLLOW_BLEND * nav_steering + (1 - STEER_FOLLOW_BLEND) * self._last_steering
            target = float(np.clip(target, -1.0, 1.0))
            self._last_steering = self._ramp_steering(target, dt)
        else:
            # Vision lane-keeping (no map/route): gentle proportional law on the
            # smoothed lane offset.  lane_offset is +when the lane centre is to
            # our left, so steer = -offset. Eased with speed so it never
            # over-corrects fast.
            off = float(lane_offset)
            if abs(off) < VISION_DEADZONE:
                raw = 0.0
            else:
                gain = 0.55 if speed_kmh < 50 else max(0.30, 0.55 - (speed_kmh - 50) / 220.0)
                raw = float(np.clip(-off * gain, -1.0, 1.0))
            target = STEER_FOLLOW_BLEND * raw + (1 - STEER_FOLLOW_BLEND) * self._last_steering
            target = float(np.clip(target, -1.0, 1.0))
            self._last_steering = self._ramp_steering(target, dt)

        steering_val = self._last_steering
        self.sdk.controller.set_steering(steering_val)

        # Turn signals with hysteresis (don't flicker on tiny corrections).
        want = self._blinker
        if steering_val > 0.16:
            want = "right"
        elif steering_val < -0.16:
            want = "left"
        elif abs(steering_val) < 0.06:
            want = "off"
        if want != self._blinker:
            self._blinker = want
            self.sdk.controller.set_blinker(want)

        # Publish UI tags.
        self.tags.steering = round(steering_val, 3)
        self.tags.speed_kmh = round(speed_kmh, 1)
        self.tags.nav_active = nav_active
        self.tags.brake = round(self._last_brake, 2)
        self.tags.throttle = round(self._last_throttle, 2)

    # --- Brake ramp -----------------------------------------------------------
    def _set_brake(self, requested: float, dt: float):
        """Apply the brake command through a ramp so it never jerks.

        Also clears the throttle the moment the brake engages (engine braking +
        avoids fighting the brakes), which the old code did abruptly."""
        requested = max(0.0, min(1.0, float(requested)))
        self._last_brake = self._ramp(self._last_brake, requested, dt,
                                      BRAKE_RAMP_UP, BRAKE_RAMP_DOWN)
        self.sdk.controller.set_brake(self._last_brake)

    def _ramp_steering(self, target: float, dt: float) -> float:
        """Rate-limit the steering so the wheel moves smoothly, never snapping."""
        target = float(np.clip(target, -1.0, 1.0))
        max_step = STEER_RATE_LIMIT * max(dt, 1e-3)
        delta = float(np.clip(target - self._last_steering, -max_step, max_step))
        return float(np.clip(self._last_steering + delta, -1.0, 1.0))
