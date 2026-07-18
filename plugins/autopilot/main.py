import logging
import math
import time
import numpy as np
from sdk.base_plugin import BasePlugin
from core.navigation.runtime_preflight import CONFIDENCE_THRESHOLD


# --- Tuning (kept here, mirrored into settings under "autopilot" section) -----
STEER_RATE_LIMIT = 0.075     # max steering change per second (smooth, no jerk)
MIN_LANE_TRAJECTORY_CONFIDENCE = CONFIDENCE_THRESHOLD
# 0.72 rejects ambiguous/off-route matches while retaining a wide margin below
# ProMods-1.59 centre samples (min 0.895, p05 0.950, median 0.966) and
# validated built trajectories (0.970-0.980). Exactly 0.72 is accepted.
STEER_FOLLOW_BLEND = 0.38    # suppress single-frame route/vision target jumps
                             # (low = smooth/laggy, high = snappy/jittery)
VISION_DEADZONE = 0.03       # ignore vision lane offset noise below this
BRAKE_RAMP_UP = 2.5          # brake can rise this fast per second (anti-jerk)
BRAKE_RAMP_DOWN = 4.0        # brake releases faster than it engages
BRAKE_MIN_HOLD = 0.04        # below this, treat brake as zero (avoid flutter)
THROTTLE_RAMP = 3.0          # throttle slew rate per second

# Anticipatory curve braking (Fáza 3c). The lateral acceleration a truck can
# hold comfortably is ~2.5 m/s²; the safe speed for a bend of radius R is
# v_safe = sqrt(A_LAT_MAX · R). We brake proactively when the MAP's measured
# curvature radius ahead would put us over that, so we slow BEFORE the apex —
# the old code only reacted once the steering was already wound in (too late,
# the truck understeered wide / fish-tailed on corner entry).
A_LAT_MAX = 2.5             # comfortable lateral accel (m/s²)
CURVE_BRAKE_MAX = 0.4       # never brake harder than this for a curve alone
CURVE_BRAKE_MARGIN_MS = 0.5 # start braking this much before v_safe (hysteresis)


def lane_authority_rejection_reason(state, snapshot, now=None):
    """Explain why a lane snapshot may not drive; empty means accepted."""
    now = time.monotonic() if now is None else float(now)
    if not isinstance(snapshot, dict) or not snapshot.get("valid", False):
        return str((snapshot or {}).get("failure_reason")
                   or "lane trajectory is invalid")
    try:
        confidence = float(snapshot.get("confidence", 0.0) or 0.0)
        if not math.isfinite(confidence):
            return "lane trajectory confidence is non-finite"
        if confidence < MIN_LANE_TRAJECTORY_CONFIDENCE:
            return (f"lane trajectory confidence {confidence:.6f} is below "
                    f"{MIN_LANE_TRAJECTORY_CONFIDENCE:.2f}")
        snapshot_revision = int(snapshot.get("revision", -1) or -1)
        current_revision = int(state.get("lane_trajectory_revision", -2) or -2)
        if snapshot_revision != current_revision:
            return (f"lane trajectory revision {snapshot_revision} is stale; "
                    f"current revision is {current_revision}")
        snapshot_uids = tuple(int(uid) for uid in
                              (snapshot.get("source_gps_uids", ()) or ()))
        game_uids = tuple(int(uid) for uid in
                          (state.get("game_route_node_uids", []) or []))
        if snapshot_uids != game_uids:
            return "lane trajectory belongs to a different GPS target"
        if snapshot.get("request_id") != state.get("nav_recalc_request"):
            return "lane trajectory calculation request is stale"
        heartbeat = float(state.get("lane_trajectory_heartbeat", 0.0) or 0.0)
        if heartbeat <= 0.0 or now - heartbeat > 0.5:
            return "map plugin heartbeat is stale"
        if state.get("telemetry_valid", True) is False:
            return "vehicle telemetry is invalid"
        if state.get("navigation_recalculating", False):
            return "navigation is recalculating"
        points = snapshot.get("points", ()) or ()
        if len(points) < 2:
            return "lane trajectory has fewer than two control points"
        for point in points:
            if (not isinstance(point, (list, tuple)) or len(point) < 3
                    or not all(math.isfinite(float(value)) for value in point[:3])):
                return "lane trajectory contains malformed or non-finite 3D points"
        # Use the live localisation for this revision, not only the match that
        # existed when the immutable geometry snapshot was built.  This blocks
        # initial full-left pulls and driving an otherwise valid route in the
        # opposite direction after the truck changes arm at a junction.
        live_match = state.get("lane_match") or snapshot.get("lane_match") or {}
        match_revision = int(live_match.get("revision", snapshot_revision)
                             or snapshot_revision)
        if match_revision != snapshot_revision:
            return "live lane localisation belongs to a stale trajectory"
        lateral = abs(float(live_match.get("lateral_error_m", 0.0) or 0.0))
        heading = abs(float(live_match.get("heading_error_rad", 0.0) or 0.0))
        if not math.isfinite(lateral) or lateral > 3.25:
            return f"truck is {lateral:.2f} m outside the confirmed GPS lane"
        if not math.isfinite(heading) or heading > math.radians(42.0):
            return (f"truck heading differs from the GPS lane by "
                    f"{math.degrees(heading):.1f} degrees")
    except (TypeError, ValueError, OverflowError):
        return "lane trajectory metadata is malformed"
    return ""


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
        # Rolling speed estimate (for ramp scaling when telemetry lags).
        self._speed_kmh = 0.0
        # Soft-start: when the autopilot is first engaged the steering ramps in
        # from zero over ~1.2 s. Without this the first tick slams ~55% of the
        # target steering, which is the visible „jerk to one side on enable“.
        self._engage_blend = 0.0
        self._was_active = False
        self._diag_t = 0.0              # throttle for diagnostic logging
        self._reverse_recovery = False
        self._drive_request_t = 0.0

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
        self.sdk.shared_state.set("autopilot_control_heartbeat", time.monotonic())

        # 1. Telemetry & state
        truck = self.sdk.telemetry.get("truck", {}) or {}
        speed = truck.get("speed", 0) or 0
        speed_kmh = abs(speed) * 3.6 if abs(speed) < 200 else abs(speed)
        try:
            gear = int(truck.get("gear", 0) or 0)
        except (TypeError, ValueError, OverflowError):
            gear = 0
        self._speed_kmh = 0.6 * speed_kmh + 0.4 * self._speed_kmh
        system_state = self.sdk.shared_state.get("system_state")
        danger_level = self.sdk.shared_state.get("danger_level", 0) or 0
        lane_offset = self.sdk.shared_state.get("lane_offset", 0) or 0
        # Real traffic available? If so, down-weight the noisy vision signal.
        traffic = self.sdk.shared_state.get("traffic", []) or []
        have_real_traffic = len(traffic) > 0
        snapshot = self.sdk.shared_state.get("lane_trajectory", {}) or {}
        try:
            snapshot_revision = int(snapshot.get("revision", -1) or -1)
            snapshot_confidence = float(snapshot.get("confidence", 0.0) or 0.0)
            game_route_distance = float(self.sdk.shared_state.get(
                "game_route_distance", 0.0) or 0.0)
        except (TypeError, ValueError, OverflowError):
            snapshot_revision, snapshot_confidence, game_route_distance = -1, 0.0, 0.0
        gps_navigation_present = bool(
            len(snapshot.get("source_gps_uids", ()) or ()) >= 2
            or game_route_distance > 25.0)
        authority_reason = lane_authority_rejection_reason(
            self.sdk.shared_state, snapshot)
        lane_authority_safe = not authority_reason
        self.sdk.shared_state.set(
            "autopilot_lane_revision",
            snapshot_revision if lane_authority_safe else -1)
        self.sdk.shared_state.set("autopilot_navigation_readiness", {
            "ready": lane_authority_safe,
            "reason": authority_reason,
            "revision": snapshot_revision if lane_authority_safe else -1,
            "confidence": snapshot_confidence,
            "threshold": MIN_LANE_TRAJECTORY_CONFIDENCE,
            "timestamp": time.monotonic(),
        })
        navigation_unreliable = bool(
            self.sdk.shared_state.get("navigation_unreliable", False)
            or (gps_navigation_present and not lane_authority_safe))

        # Never feed throttle to a reversing truck.  In ETS2's automatic
        # gearbox a brake held after stopping can select reverse; the old
        # controller then applied cruise throttle on the next tick.  Disengage
        # immediately and return all automatic commands to a safe neutral.
        autopilot_engaged = bool(self.sdk.shared_state.get(
            "autopilot_active", False))
        reversing = bool(autopilot_engaged and
                         (float(speed) < -0.10 or gear < 0
                          or self._reverse_recovery))
        if reversing:
            self._reverse_recovery = True
            self.sdk.controller.set_throttle(0.0)
            self._last_throttle = 0.0
            self._last_steering = self._ramp_steering(0.0, dt)
            self.sdk.controller.set_steering(self._last_steering)
            now = time.monotonic()
            if float(speed) < -0.10 or speed_kmh > 0.5:
                self._set_brake(0.62, dt)
                self.sdk.shared_state.set(
                    "navigation_status", "Zastavujem pred zaradením jazdy dopredu")
            elif gear < 0:
                self._set_brake(0.18, dt)
                if now - self._drive_request_t >= 0.7:
                    self.sdk.controller.select_drive(True)
                    self._drive_request_t = now
                self.sdk.shared_state.set(
                    "navigation_status", "Zaraďujem jazdu dopredu")
            else:
                self.sdk.controller.select_drive(False)
                self.sdk.controller.set_brake(0.0)
                self._last_brake = 0.0
                self._reverse_recovery = False
                self.sdk.shared_state.set(
                    "navigation_status", "Jazda dopredu pripravená")
            return

        # Prevention comes before recovery: if the autopilot is engaged while
        # the automatic gearbox is in N, select Drive before any throttle is
        # allowed. This also covers a fresh engagement after loading the game.
        if autopilot_engaged and speed_kmh < 0.5 and gear == 0:
            self.sdk.controller.set_throttle(0.0)
            self._last_throttle = 0.0
            self.sdk.controller.set_brake(0.0)
            self._last_brake = 0.0
            self.sdk.controller.select_drive(True)
            self._drive_request_t = time.monotonic()
            self.sdk.shared_state.set(
                "navigation_status", "Pripravujem jazdu dopredu")
            return
        if gear > 0:
            # geardrive is a momentary SDK button, never leave it latched.
            self.sdk.controller.select_drive(False)

        arrival_pending = bool(self.sdk.shared_state.get(
            "navigation_arrival_pending", False))
        if arrival_pending and autopilot_engaged:
            self.sdk.controller.set_throttle(0.0)
            self._last_throttle = 0.0
            self._last_steering = self._ramp_steering(0.0, dt)
            self.sdk.controller.set_steering(self._last_steering)
            if speed_kmh > 1.0:
                self._set_brake(0.72, dt)
                self.sdk.shared_state.set(
                    "navigation_status", "Prichádzam do cieľa – zastavujem")
            else:
                # Release before disengaging so an automatic gearbox cannot
                # interpret a held brake as a request to reverse.
                self.sdk.controller.set_brake(0.0)
                self._last_brake = 0.0
                self.sdk.shared_state.set("autopilot_active", False)
                self.sdk.shared_state.set("nav_active", False)
                self.sdk.shared_state.set("navigation_arrival_pending", False)
                self.sdk.shared_state.set("navigation_status", "Cieľ dosiahnutý")
                self.sdk.shared_state.set("tts_message", "Cieľ dosiahnutý.")
                logging.info("Navigation: destination reached; vehicle stopped and autopilot disengaged.")
            return

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
        aux_brake = float(self.sdk.shared_state.get("aux_brake_request", 0.0) or 0.0)
        # Vision obstacle (screen CV). Only trust it as a *nudge*: when we have
        # real traffic data, heavily discount it so a shadow / sign can't cause
        # a phantom full stop.
        if danger_level > 0.35:
            vision_brake = float(np.clip((danger_level - 0.35) * 1.8, 0.0, 1.0))
            vision_brake *= (0.25 if have_real_traffic else 1.0)
        else:
            vision_brake = 0.0
        requested_brake = max(collision_brake, traffic_brake, light_brake,
                              aux_brake, vision_brake)
        if navigation_unreliable:
            # A GPS route with a mismatched map must never fall through to
            # camera lane detection at an intersection. Stop predictably.
            requested_brake = max(requested_brake, 0.70)

        if system_state == "AVOID_OBSTACLE":
            requested_brake = max(requested_brake,
                                  float(np.clip(0.5 + (0.5 * danger_level), 0.5, 1.0)))

        # --- Anticipatory curve braking (Fáza 3c) -------------------------
        # Slow BEFORE a sharp bend, using the MAP's measured path curvature
        # ahead (path_curvature_radius), not the steering we're already turning
        # (that was too late — the truck understeered into corners). The safe
        # speed for radius R at comfortable lateral accel A_LAT_MAX is
        # v_safe = sqrt(A_LAT_MAX · R); if our speed exceeds it, brake.
        radius = self.sdk.shared_state.get("path_curvature_radius", None)
        curve_factor = 1.0          # throttle multiplier (set below)
        if radius is not None:
            try:
                R = float(radius)
            except (TypeError, ValueError):
                R = 1e6
            if 30.0 < R < 2000.0:   # ignore straight / garbage radii
                v_safe = math.sqrt(A_LAT_MAX * R)         # m/s
                v_now = abs(speed)                        # m/s
                if v_now > v_safe + CURVE_BRAKE_MARGIN_MS:
                    over = (v_now - v_safe) / max(v_safe, 1.0)  # 0..1+ excess
                    curve_brake = float(np.clip(over * 0.8, 0.0, CURVE_BRAKE_MAX))
                    requested_brake = max(requested_brake, curve_brake)
                    # Also ease the throttle so we don't fight the brake.
                    curve_factor = max(0.3, 1.0 - over)

        # --- Reactive curve slowdown: ease off the throttle (light brake at
        # speed) — a back-up to the proactive brake above, in case the map
        # curvature isn't published yet (e.g. no map loaded, vision only). ---
        turn = abs(self._last_steering)
        curve_factor = min(curve_factor,
                           1.0 if turn < 0.18 else max(0.35, 1.0 - (turn - 0.18) * 1.6))
        if turn > 0.45 and speed_kmh > 45:
            requested_brake = max(requested_brake,
                                  float(np.clip((turn - 0.45) * 0.6, 0.0, 0.35)))

        # 3. Apply braking THROUGH THE RAMP (anti-jerk). This is the key change:
        #    the truck brakes firmly but progressively, never a step to 1.0.
        self._set_brake(requested_brake, dt)

        # While stopped for traffic keep the gearbox in Drive. Holding the
        # brake must never become ETS2's automatic brake-to-reverse gesture.
        if autopilot_engaged and speed_kmh < 1.0 and requested_brake > 0.0:
            self.sdk.controller.select_drive(True)

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
        nav_active = bool(self.sdk.shared_state.get("nav_active", False)
                          and lane_authority_safe)

        # Soft-start: detect the rising edge of autopilot_active and fade the
        # steering authority in from 0 → 1 over ~1.2 s. This kills the jerk that
        # happens the instant the user toggles the autopilot on (the first tick
        # would otherwise apply 55% of whatever target was computed).
        active = bool(self.sdk.shared_state.get("autopilot_active", False))
        if active and not self._was_active:
            self._engage_blend = 0.0
        self._was_active = active
        engage = min(1.0, self._engage_blend + dt / 1.2)
        self._engage_blend = engage if active else 0.0

        if navigation_unreliable:
            self._last_steering = self._ramp_steering(0.0, dt)
        elif nav_active:
            # nav_steering is already a finished pure-pursuit + CTE value from
            # the Route/map plugin. Apply a SHORT rate-limit only — the old
            # 0.35/0.65 exponential lag was a second integrator that caused the
            # truck to overshoot and oscillate (fishtail) in and out of curves.
            nav_steering = float(self.sdk.shared_state.get("nav_steering", 0.0) or 0.0)
            target = STEER_FOLLOW_BLEND * nav_steering + (1 - STEER_FOLLOW_BLEND) * self._last_steering
            target = float(np.clip(target, -1.0, 1.0))
            self._last_steering = self._ramp_steering(target, dt)
        elif not gps_navigation_present:
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
        else:
            # Never substitute camera lane keeping for an invalid GPS lane at
            # an intersection. Steering returns to zero through the existing
            # rate limiter; the brake/throttle safety path above handles stop.
            self._last_steering = self._ramp_steering(0.0, dt)

        # Apply the soft-start engagement ramp so we never slam the wheel over
        # the moment the autopilot is switched on.
        steering_val = self._last_steering * self._engage_blend

        # Diagnostic: log the lateral-control state once per second so we can see
        # exactly why the truck turns the way it does (the sign of lane_offset /
        # nav_steering vs the resulting steering_val is what tells us whether
        # the convention is correct).
        self._diag_t += dt
        if self._diag_t >= 1.0:
            self._diag_t = 0.0
            logging.info(
                "autopilot: active=%s nav=%s engage=%.2f lane_off=%.3f "
                "nav_steer=%.3f target=%.3f steer_out=%.3f speed=%.0f "
                "lane_revision=%s confidence=%.3f reject=%s",
                active, nav_active, self._engage_blend,
                float(lane_offset),
                float(self.sdk.shared_state.get("nav_steering", 0.0) or 0.0),
                float(self._last_steering), steering_val, speed_kmh,
                snapshot_revision,
                snapshot_confidence,
                authority_reason)

        self.sdk.controller.set_steering(steering_val)

        # NOTE: turn signals are NOT driven from steering here anymore. Tying the
        # blinkers to the steering value made them flicker on every curve and —
        # worse — toggle a "lane change" during obstacle avoidance, which is
        # exactly the "pruhy sa menia pri obchádzaní" bug. Indicator control now
        # lives in the dedicated turn-signals logic (see plugins/turnsignals),
        # which only signals a real lane change / turn when the route actually
        # requires one. We still publish the steering so that logic can use it.
        self.tags.steering = round(steering_val, 3)

        # Publish UI tags.
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
