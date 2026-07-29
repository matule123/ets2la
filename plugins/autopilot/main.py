import logging
import math
import time
import numpy as np
from sdk.base_plugin import BasePlugin
from core.navigation.runtime_preflight import CONFIDENCE_THRESHOLD
from core.navigation.navigation_intent import snapshot_matches_navigation_intent
from core.navigation.route import curve_speed_limit_ms


# --- Tuning (kept here, mirrored into settings under "autopilot" section) -----
STEER_RATE_LIMIT = 0.60      # acquire confirmed curve steering before lane drift
STEER_UNWIND_RATE = 1.80     # release confirmed lock faster than it is acquired
MIN_LANE_TRAJECTORY_CONFIDENCE = CONFIDENCE_THRESHOLD
# 0.72 rejects ambiguous/off-route matches while retaining a wide margin below
# ProMods-1.59 centre samples (min 0.895, p05 0.950, median 0.966) and
# validated built trajectories (0.970-0.980). Exactly 0.72 is accepted.
STEER_FOLLOW_BLEND = 0.72    # follow lane authority without a second slow integrator
                             # (low = smooth/laggy, high = snappy/jittery)
VISION_DEADZONE = 0.03       # ignore vision lane offset noise below this
BRAKE_RAMP_UP = 2.5          # brake can rise this fast per second (anti-jerk)
BRAKE_RAMP_DOWN = 4.0        # brake releases faster than it engages
BRAKE_MIN_HOLD = 0.04        # below this, treat brake as zero (avoid flutter)
THROTTLE_RAMP = 3.0          # throttle slew rate per second
DRIVE_ENGAGE_SETTLE_S = 0.45 # allow the selector pulse to reach the gearbox
DRIVE_RETRY_S = 1.50         # retry D without blocking throttle indefinitely
ENGAGEMENT_DEFAULT_LATERAL_M = 1.10
ENGAGEMENT_MAX_LATERAL_M = 1.50
ENGAGEMENT_MAX_HEADING_RAD = math.radians(18.0)
AUTHORITY_DEFAULT_LATERAL_M = 1.80
AUTHORITY_RETENTION_WIDTH_FRACTION = 0.75
AUTHORITY_RETENTION_MAX_M = 3.40

# Anticipatory curve braking (Fáza 3c). The lateral acceleration a truck can
# hold comfortably is ~2.5 m/s²; the safe speed for a bend of radius R is
# v_safe = sqrt(A_LAT_MAX · R). We brake proactively when the MAP's measured
# curvature radius ahead would put us over that, so we slow BEFORE the apex —
# the old code only reacted once the steering was already wound in (too late,
# the truck understeered wide / fish-tailed on corner entry).
A_LAT_MAX = 1.8             # stable loaded-truck lateral acceleration (m/s²)
CURVE_BRAKE_MAX = 0.55      # bounded proactive brake for proven sharp curves
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
        if not snapshot_matches_navigation_intent(state, snapshot):
            return "lane trajectory belongs to a different navigation intent"
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
        if live_match.get("valid") is False:
            return str(live_match.get("failure_reason")
                       or "live lane localisation is temporarily unavailable")
        live_lane_id = live_match.get("active_lane_id")
        corridor = snapshot.get("lane_corridor", ()) or ()
        corridor_entry = next((entry for entry in corridor
                               if entry.get("lane_id") == live_lane_id), None)
        if corridor and live_lane_id is not None and corridor_entry is None:
            return "live localisation belongs to a lane outside the GPS corridor"
        if (not corridor and snapshot.get("active_lane_id") is not None
                and live_lane_id is not None
                and live_lane_id != snapshot.get("active_lane_id")):
            return "live localisation belongs to a different GPS lane"
        live_layer = live_match.get("elevation_layer")
        if (corridor_entry is not None and live_layer is not None
                and int(live_layer) != int(
                    corridor_entry.get("elevation_layer"))):
            return "live localisation belongs to a different elevation layer"
        snapshot_layer = (snapshot.get("lane_match") or {}).get(
            "elevation_layer")
        if (not corridor and snapshot_layer is not None
                and live_layer is not None
                and int(live_layer) != int(snapshot_layer)):
            return "live localisation belongs to a different elevation layer"
        lateral = abs(float(live_match.get("lateral_error_m", 0.0) or 0.0))
        heading = abs(float(live_match.get("heading_error_rad", 0.0) or 0.0))
        if (not math.isfinite(lateral)
                or lateral > authority_retention_lateral_limit(live_match)):
            return f"truck is {lateral:.2f} m outside the confirmed GPS lane"
        if not math.isfinite(heading) or heading > math.radians(28.0):
            return (f"truck heading differs from the GPS lane by "
                    f"{math.degrees(heading):.1f} degrees")
    except (TypeError, ValueError, OverflowError):
        return "lane trajectory metadata is malformed"
    return ""


def game_gps_navigation_present(state, snapshot=None):
    """Detect game-GPS ownership even when its native UID buffer is invalid."""
    snapshot = snapshot or {}
    if bool(state.get("game_gps_navigation_active", False)):
        return True
    if bool(state.get("navigation_arrival_pending", False)):
        return True
    if state.get("dest_city"):
        return True
    try:
        if float(state.get("game_route_distance", 0.0) or 0.0) > 0.0:
            return True
    except (TypeError, ValueError, OverflowError):
        return True
    return bool(len(state.get("game_route_node_uids", []) or []) >= 2
                or len(snapshot.get("source_gps_uids", []) or []) >= 2)


def _authority_reason_key(reason):
    """Stable log category for failure text containing live measurements."""
    text = str(reason or "")
    if text.startswith("truck is ") and text.endswith(
            " outside the confirmed GPS lane"):
        return "outside_confirmed_gps_lane"
    if text.startswith("truck heading differs from the GPS lane"):
        return "gps_lane_heading_mismatch"
    return text


def recorded_route_rejection_reason(state):
    """Validate the explicitly activated, GPS-exclusive replay authority."""
    if state.get("navigation_source") != "recorded_route":
        return "recorded route is not the selected navigation source"
    if not state.get("recorded_route_active", False):
        return "recorded route was not explicitly activated"
    points = state.get("nav_path", []) or []
    if len(points) < 2:
        return "recorded route has fewer than two points"
    try:
        for point in points:
            if (not isinstance(point, (list, tuple)) or len(point) < 2
                    or not all(math.isfinite(float(value)) for value in point[:3])):
                return "recorded route contains malformed or non-finite points"
    except (TypeError, ValueError, OverflowError):
        return "recorded route metadata is malformed"
    return ""


def engagement_lateral_limit(live_match):
    """Return a lane-width-aware initial localisation gate in metres.

    The central two thirds of the measured lane are eligible for engagement.
    A missing width (old shared state) retains the former conservative 1.10 m
    gate, while unusually wide lanes can never relax beyond 1.50 m.
    """
    try:
        width = float((live_match or {}).get("lane_width_m"))
    except (TypeError, ValueError, OverflowError):
        return ENGAGEMENT_DEFAULT_LATERAL_M
    if not math.isfinite(width) or width < 2.4 or width > 12.0:
        return ENGAGEMENT_DEFAULT_LATERAL_M
    return min(ENGAGEMENT_MAX_LATERAL_M,
               max(0.60, width / 3.0))


def authority_retention_lateral_limit(live_match):
    """Continue correcting only inside LaneLocator's proven lane retention.

    Initial engagement remains governed by the much stricter gate above. Once
    the exact LaneId, direction, elevation and revision are locked, stopping
    steering at the former 1.80 m boundary made a recoverable curve deviation
    grow until the vehicle left the lane. Match the locator's width-aware
    same-lane retention, but never grant this distance to a new lane candidate.
    """
    try:
        width = float((live_match or {}).get("lane_width_m"))
    except (TypeError, ValueError, OverflowError):
        return AUTHORITY_DEFAULT_LATERAL_M
    if not math.isfinite(width) or width < 2.4 or width > 12.0:
        return AUTHORITY_DEFAULT_LATERAL_M
    return max(AUTHORITY_DEFAULT_LATERAL_M, min(
        AUTHORITY_RETENTION_MAX_M,
        width * AUTHORITY_RETENTION_WIDTH_FRACTION))


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
        self._drive_engage_started = 0.0
        self._lane_lock_acquired = False
        self._last_authority_stop_reason = None

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

    def _publish_control_tags(self, speed_kmh, nav_active=False):
        """Keep HUD pedal/steering indicators present on safety early-returns."""
        self.tags.speed_kmh = round(float(speed_kmh), 1)
        self.tags.nav_active = bool(nav_active)
        self.tags.steering = round(float(self._last_steering), 3)
        self.tags.brake = round(float(self._last_brake), 2)
        self.tags.throttle = round(float(self._last_throttle), 2)

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
        except (TypeError, ValueError, OverflowError):
            snapshot_revision, snapshot_confidence = -1, 0.0
        gps_navigation_present = game_gps_navigation_present(
            self.sdk.shared_state, snapshot)
        recorded_route_requested = bool(
            self.sdk.shared_state.get("navigation_source") == "recorded_route"
            or self.sdk.shared_state.get("recorded_route_active", False))
        if gps_navigation_present:
            authority_reason = lane_authority_rejection_reason(
                self.sdk.shared_state, snapshot)
            authority_revision = snapshot_revision
            authority_source = "gps_lane"
        elif recorded_route_requested:
            authority_reason = recorded_route_rejection_reason(
                self.sdk.shared_state)
            authority_revision = -1
            authority_source = "recorded_route"
        else:
            # No route is not navigation authority.  Preserve the existing
            # fail-closed engagement rule; vision may assist perception but
            # cannot by itself authorize the autopilot.
            authority_reason = lane_authority_rejection_reason(
                self.sdk.shared_state, snapshot)
            authority_revision = -1
            authority_source = "none"
        active_requested = bool(self.sdk.shared_state.get(
            "autopilot_active", False))
        # Starting steering on a boundary or while facing a neighbouring arm
        # caused the initial pull into the oncoming carriageway. Engagement is
        # therefore stricter than continued driving; afterwards the normal
        # fail-closed lane corridor above is checked on every tick.
        if not active_requested:
            self._lane_lock_acquired = False
            self._drive_engage_started = 0.0
        if (gps_navigation_present and not authority_reason
                and not self._lane_lock_acquired):
            live_match = (self.sdk.shared_state.get("lane_match")
                          or snapshot.get("lane_match"))
            # Legacy/offline consumers can exercise confidence handling
            # without runtime localisation. In the game, map always publishes
            # lane_match and the strict engagement gate below is mandatory.
            if not live_match:
                engage_lateral = engage_heading = 0.0
            else:
                try:
                    engage_lateral = abs(float(
                        live_match.get("lateral_error_m", float("inf"))))
                    engage_heading = abs(float(
                        live_match.get("heading_error_rad", float("inf"))))
                except (TypeError, ValueError, OverflowError):
                    engage_lateral = engage_heading = float("inf")
            engage_lateral_limit = engagement_lateral_limit(live_match)
            if (not self._lane_lock_acquired
                    and (engage_lateral > engage_lateral_limit
                         or engage_heading > ENGAGEMENT_MAX_HEADING_RAD)):
                authority_reason = (
                    "truck is not centred and aligned in the confirmed GPS lane")
            elif active_requested and not self._lane_lock_acquired:
                self._lane_lock_acquired = True
        navigation_authority_safe = not authority_reason
        self.sdk.shared_state.set(
            "autopilot_lane_revision",
            (authority_revision if navigation_authority_safe
             and authority_source == "gps_lane" else -1))
        self.sdk.shared_state.set("autopilot_navigation_readiness", {
            "ready": navigation_authority_safe,
            "reason": authority_reason,
            "revision": (authority_revision
                         if navigation_authority_safe else -1),
            "source": authority_source,
            "confidence": snapshot_confidence,
            "threshold": MIN_LANE_TRAJECTORY_CONFIDENCE,
            "timestamp": time.monotonic(),
        })
        navigation_unreliable = bool(
            (gps_navigation_present
             and (self.sdk.shared_state.get("navigation_unreliable", False)
                  or not navigation_authority_safe))
            or (recorded_route_requested and not navigation_authority_safe))

        # Never feed throttle to a reversing truck.  In ETS2's automatic
        # gearbox a brake held after stopping can select reverse; the old
        # controller then applied cruise throttle on the next tick.  Disengage
        # immediately and return all automatic commands to a safe neutral.
        autopilot_engaged = bool(self.sdk.shared_state.get(
            "autopilot_active", False))

        # Arrival is terminal and must run before reverse recovery or the
        # automatic-D handshake. With gear 0/-1 those branches used to return
        # first, so the autopilot stayed enabled and could start reversing.
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
                # Release brake and any momentary D selector request before
                # disabling authority. Never emit a new D/R pulse at arrival.
                self.sdk.controller.set_brake(0.0)
                self.sdk.controller.select_drive(False)
                self._last_brake = 0.0
                self._reverse_recovery = False
                self._drive_engage_started = 0.0
                self.sdk.shared_state.set("autopilot_active", False)
                self.sdk.shared_state.set("nav_active", False)
                self.sdk.shared_state.set("nav_steering", 0.0)
                self.sdk.shared_state.set("navigation_arrival_pending", False)
                self.sdk.shared_state.set("navigation_status", "Cieľ dosiahnutý")
                self.sdk.shared_state.set("tts_message", "Cieľ dosiahnutý.")
                logging.info("Navigation: destination reached; vehicle stopped and autopilot disengaged.")
            self._publish_control_tags(speed_kmh, False)
            return

        # Fail closed before touching the gearbox or throttle. A missing,
        # stale, low-confidence or off-lane GPS trajectory is not permission to
        # fall back to vision driving. While moving we perform a controlled
        # stop; once stationary we release all automation and disengage.
        if autopilot_engaged and not navigation_authority_safe:
            authority_reason_key = _authority_reason_key(authority_reason)
            if authority_reason_key != self._last_authority_stop_reason:
                logging.warning(
                    "Autopilot lost navigation authority; controlled stop: %s",
                    authority_reason)
                self._last_authority_stop_reason = authority_reason_key
            self.sdk.controller.set_throttle(0.0)
            self._last_throttle = 0.0
            self._last_steering = self._ramp_steering(0.0, dt)
            self.sdk.controller.set_steering(self._last_steering)
            self.sdk.shared_state.set(
                "navigation_status", f"Autopilot zablokovany: {authority_reason}")
            if speed_kmh > 1.0:
                self._set_brake(0.70, dt)
            else:
                self.sdk.controller.set_brake(0.0)
                self.sdk.controller.select_drive(False)
                self._last_brake = 0.0
                self._drive_engage_started = 0.0
                self._reverse_recovery = False
                self.sdk.shared_state.set("autopilot_active", False)
                self.sdk.shared_state.set("nav_active", False)
                self.sdk.shared_state.set("nav_steering", 0.0)
                self.sdk.shared_state.set(
                    "autopilot_disable_reason", authority_reason)
                logging.warning(
                    "Autopilot automatically disengaged after safety stop: %s",
                    authority_reason)
            self._publish_control_tags(speed_kmh, False)
            return

        if navigation_authority_safe:
            self._last_authority_stop_reason = None

        reversing = bool(autopilot_engaged and
                         (float(speed) < -0.10 or gear < 0
                          or self._reverse_recovery))
        if reversing:
            self._reverse_recovery = True
            self.sdk.controller.set_throttle(0.0)
            self._last_throttle = 0.0
            self._last_steering = self._ramp_steering(0.0, dt)
            self.sdk.controller.set_steering(self._last_steering)
            if float(speed) < -0.10 or speed_kmh > 0.5:
                self._set_brake(0.62, dt)
                self.sdk.shared_state.set(
                    "navigation_status", "Zastavujem neočakávanú spiatočku")
            else:
                self.sdk.controller.set_brake(0.0)
                self.sdk.controller.select_drive(False)
                self._last_brake = 0.0
                self._reverse_recovery = False
                self._drive_engage_started = 0.0
                self.sdk.shared_state.set("autopilot_active", False)
                self.sdk.shared_state.set("nav_active", False)
                self.sdk.shared_state.set("nav_steering", 0.0)
                self.sdk.shared_state.set(
                    "autopilot_disable_reason", "unexpected reverse gear")
                logging.warning(
                    "Autopilot automatically disengaged: unexpected reverse gear")
                self.sdk.shared_state.set(
                    "navigation_status", "Autopilot vypnutý po spiatočke")
            self._publish_control_tags(speed_kmh, False)
            return

        # ``gear`` is the currently engaged ratio, not a reliable automatic
        # selector mode. Several ETS2 automatic transmissions report gear 0
        # while stationary in D and engage first gear only after throttle is
        # applied. Waiting for gear > 0 before allowing any throttle therefore
        # deadlocks forever: D waits for throttle and autopilot waits for D.
        # Send one proven momentary D pulse, wait briefly, then continue with
        # the normal ramped throttle even if the ratio still reads zero. Retry
        # the selector periodically without re-entering the blocking phase.
        if autopilot_engaged and speed_kmh < 0.5 and gear == 0:
            now = time.monotonic()
            if self._drive_engage_started <= 0.0:
                self._drive_engage_started = now
                self.sdk.controller.select_drive(True)
                self._drive_request_t = now
            elif now - self._drive_request_t >= DRIVE_RETRY_S:
                self.sdk.controller.select_drive(True)
                self._drive_request_t = now

            if now - self._drive_engage_started < DRIVE_ENGAGE_SETTLE_S:
                self.sdk.controller.set_throttle(0.0)
                self._last_throttle = 0.0
                self.sdk.controller.set_brake(0.0)
                self._last_brake = 0.0
                self.sdk.shared_state.set(
                    "navigation_status", "Pripravujem jazdu dopredu")
                # Keep the HUD controls visible during this intentional wait.
                self.tags.speed_kmh = round(speed_kmh, 1)
                self.tags.nav_active = bool(
                    self.sdk.shared_state.get("nav_active", False)
                    and navigation_authority_safe)
                self.tags.brake = 0.0
                self.tags.throttle = 0.0
                return
            self.sdk.shared_state.set(
                "navigation_status", "Jazda dopredu pripravená")
        if gear > 0:
            # The engine owns the physical release half of every momentary
            # selector pulse. Publishing False here can overwrite an unconsumed
            # True in shared state before the engine process observes it.
            self._drive_engage_started = 0.0

        # 2. Safety states — these still brake hard, but through the ramp so
        #    the truck doesn't lock up and spin.
        if system_state == "EMERGENCY":
            self._set_brake(1.0, dt)
            self.sdk.controller.set_throttle(0.0)
            self._last_throttle = 0.0
            # Never leave the previous steering value latched while stopping
            # in a queue. That stale command kept winding the truck out of its
            # lane until the authority guard disengaged it. A valid GPS lane
            # remains lateral authority during braking; without it unwind.
            emergency_nav_active = bool(
                navigation_authority_safe
                and self.sdk.shared_state.get("nav_active", False))
            if emergency_nav_active:
                nav_steering = float(self.sdk.shared_state.get(
                    "nav_steering", 0.0) or 0.0)
                target = (STEER_FOLLOW_BLEND * nav_steering
                          + (1.0 - STEER_FOLLOW_BLEND)
                          * self._last_steering)
            else:
                target = 0.0
            self._last_steering = self._ramp_steering(target, dt)
            self.sdk.controller.set_steering(
                self._last_steering * self._engage_blend)
            self.sdk.shared_state.set("tts_message", "Emergency stop triggered!")
            self._publish_control_tags(speed_kmh, emergency_nav_active)
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
        curve_distance = self.sdk.shared_state.get(
            "path_curve_distance_m", 0.0)
        curve_factor = 1.0          # throttle multiplier (set below)
        curve_limit_ms = float("inf")
        if radius is not None:
            try:
                R = float(radius)
                distance_to_curve = float(curve_distance or 0.0)
            except (TypeError, ValueError, OverflowError):
                R = 1e6
                distance_to_curve = 0.0
            if 0.0 < R < 2000.0:
                curve_limit_ms = curve_speed_limit_ms(
                    R, distance_to_curve, A_LAT_MAX)
                v_now = abs(speed)                        # m/s
                if v_now > curve_limit_ms + CURVE_BRAKE_MARGIN_MS:
                    excess = v_now - curve_limit_ms
                    curve_brake = float(np.clip(
                        excess / 6.0, 0.0, CURVE_BRAKE_MAX))
                    requested_brake = max(requested_brake, curve_brake)
                    curve_factor = max(
                        0.0, min(1.0, curve_limit_ms / max(v_now, 1.0)))
                elif v_now > curve_limit_ms * 0.90:
                    # Coast into the speed envelope instead of accelerating
                    # until the brake threshold and then oscillating around it.
                    curve_factor = max(0.15, min(
                        1.0, (curve_limit_ms - v_now)
                        / max(curve_limit_ms * 0.10, 0.5)))
        self.sdk.shared_state.set(
            "path_curve_speed_limit_ms",
            (None if not math.isfinite(curve_limit_ms)
             else float(curve_limit_ms)))

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
                          and navigation_authority_safe)

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
            # The map plugin already computes Stanley cross-track feedback
            # from this revision's signed LaneMatch. Do not add that same
            # lateral error a second time here: the former adaptive trim was
            # a duplicate integrator and could carry a bend correction onto
            # the following straight.
            target = (STEER_FOLLOW_BLEND * nav_steering
                      + (1 - STEER_FOLLOW_BLEND) * self._last_steering)
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
            diagnostic_match = (self.sdk.shared_state.get("lane_match")
                                or snapshot.get("lane_match") or {})
            try:
                live_lateral = float(diagnostic_match.get(
                    "lateral_error_m", 0.0) or 0.0)
                live_heading = math.degrees(float(diagnostic_match.get(
                    "heading_error_rad", 0.0) or 0.0))
                diagnostic_radius = (None if radius is None else float(radius))
                diagnostic_curve_distance = float(curve_distance or 0.0)
            except (TypeError, ValueError, OverflowError):
                live_lateral = live_heading = float("nan")
                diagnostic_radius = None
                diagnostic_curve_distance = float("nan")
            logging.info(
                "autopilot: active=%s nav=%s engage=%.2f lane_cte=%.3f "
                "lane_heading=%.1fdeg vision_off=%.3f "
                "nav_steer=%.3f target=%.3f steer_out=%.3f speed=%.0f "
                "curve_r=%s curve_d=%.1f curve_limit=%s "
                "lane_revision=%s confidence=%.3f reject=%s",
                active, nav_active, self._engage_blend,
                live_lateral, live_heading, float(lane_offset),
                float(self.sdk.shared_state.get("nav_steering", 0.0) or 0.0),
                float(self._last_steering), steering_val, speed_kmh,
                ("-" if diagnostic_radius is None
                 else f"{diagnostic_radius:.1f}"),
                diagnostic_curve_distance,
                ("-" if not math.isfinite(curve_limit_ms)
                 else f"{curve_limit_ms * 3.6:.1f}kmh"),
                snapshot_revision,
                snapshot_confidence,
                authority_reason)

        self.sdk.controller.set_steering(steering_val)

        # Confirm engagement only after this plugin has accepted the exact
        # navigation authority and initialized/applied a safe control output.
        # The engine may acknowledge the user's request earlier, but must not
        # claim that the autopilot is enabled before this handshake exists.
        engagement_request = self.sdk.shared_state.get(
            "autopilot_engagement_request")
        engagement_confirmed = self.sdk.shared_state.get(
            "autopilot_engagement_confirmed")
        lane_authority_confirmed = bool(
            authority_source == "recorded_route" or self._lane_lock_acquired)
        if (active and nav_active and lane_authority_confirmed
                and engagement_request is not None
                and engagement_request != engagement_confirmed):
            self.sdk.shared_state.update_batch({
                "autopilot_engagement_confirmed": engagement_request,
                "navigation_status": "Autopilot zapnutý",
                "tts_message": "Autopilot enabled.",
            })
            logging.info(
                "Autopilot enabled after navigation authority and control "
                "initialization (request %s).", engagement_request)

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
        """Rate-limit steering without retaining stale lock after a bend.

        Applying and releasing the wheel at the same slow rate left the old
        command active for several seconds after the route target had crossed
        zero. That delay is a control integrator: the truck first ran wide,
        then crossed the centre and kept turning into the opposite lane. Keep
        engagement gentle, but unwind/reverse toward zero three times faster.
        """
        target = float(np.clip(target, -1.0, 1.0))
        current = float(self._last_steering)
        reversing = current * target < 0.0
        unwinding = abs(target) < abs(current) or reversing
        rate = STEER_UNWIND_RATE if unwinding else STEER_RATE_LIMIT
        max_step = rate * max(dt, 1e-3)
        if reversing:
            # Release the old lock first; do not cross zero at the fast unwind
            # rate and apply an opposite command in the same control frame.
            return float(max(0.0, current - max_step)
                         if current > 0.0 else min(0.0, current + max_step))
        delta = float(np.clip(target - self._last_steering, -max_step, max_step))
        return float(np.clip(self._last_steering + delta, -1.0, 1.0))
