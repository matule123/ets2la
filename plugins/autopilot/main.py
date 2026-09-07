import logging
import math
import os
import time
import numpy as np
from sdk.base_plugin import BasePlugin
from core.navigation.runtime_preflight import (
    CONFIDENCE_THRESHOLD, effective_lane_confidence,
)
from core.navigation.navigation_intent import snapshot_matches_navigation_intent
from core.navigation.route import curve_speed_limit_ms
from core.steering_dynamics import SteeringDynamics
from core.steering_replay import SteeringReplayBuffer
from core.paths import app_dir


# --- Tuning (kept here, mirrored into settings under "autopilot" section) -----
MIN_LANE_TRAJECTORY_CONFIDENCE = CONFIDENCE_THRESHOLD
# 0.72 rejects ambiguous/off-route matches while retaining a wide margin below
# ProMods-1.59 centre samples (min 0.895, p05 0.950, median 0.966) and
# validated built trajectories (0.970-0.980). Exactly 0.72 is accepted.
VISION_STEER_FOLLOW_BLEND = 0.72  # used only without authoritative GPS geometry
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
CURVE_BRAKE_MARGIN_MS = 0.15
CURVE_APPROACH_DECEL_MS2 = 1.0
CURVE_BRAKE_RESPONSE_S = 1.0
# A compact prefab can contain right-left-right curvature within a few truck
# lengths.  Reserve distance for the steering axis and trailer to settle; the
# old point-mass envelope reached the apex speed mathematically but entered
# the first lobe too fast to follow the proven lane centre.
CURVE_STEERING_SETUP_M = 20.0


def planned_curve_speed_limit_ms(radius_m, distance_m, speed_ms):
    """Return the curve-entry envelope and its proven usable distance.

    The former point-mass envelope spent every metre on an ideal 1.6 m/s2
    stop. It ignored brake ramp and loaded-truck response, so the captured R18
    hairpin was still approached at 43 km/h. Reserve one current-speed second
    plus steering setup, then plan at a comfortable 1.0 m/s2. This changes
    only longitudinal control; map geometry and steering authority are intact.
    """
    radius = float(radius_m)
    distance = max(0.0, float(distance_m))
    speed = max(0.0, abs(float(speed_ms)))
    setup = CURVE_STEERING_SETUP_M if radius < 45.0 else 0.0
    response = speed * CURVE_BRAKE_RESPONSE_S
    usable_distance = max(0.0, distance - setup - response)
    return (curve_speed_limit_ms(
        radius, usable_distance, A_LAT_MAX, CURVE_APPROACH_DECEL_MS2),
        usable_distance)


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
        snapshot_revision = int(snapshot.get("revision", -1) or -1)
        current_revision = int(state.get("lane_trajectory_revision", -2) or -2)
        if snapshot_revision != current_revision:
            return (f"lane trajectory revision {snapshot_revision} is stale; "
                    f"current revision is {current_revision}")
        steering_revision = int(state.get(
            "nav_trajectory_revision", snapshot_revision) or -1)
        if (state.get("nav_active", False)
                and steering_revision != snapshot_revision):
            return (f"steering revision {steering_revision} is stale; "
                    f"lane trajectory revision is {snapshot_revision}")
        if not snapshot_matches_navigation_intent(state, snapshot):
            return "lane trajectory belongs to a different navigation intent"
        context_checks = (
            ("source_game_session_id", "game_session_id", "game session"),
            ("source_map_key", "active_map_key", "map dataset"),
            ("source_dataset_fingerprint", "active_dataset_fingerprint",
             "map dataset fingerprint"),
        )
        for snapshot_key, state_key, label in context_checks:
            source_value = snapshot.get(snapshot_key)
            if source_value is not None and source_value != state.get(state_key):
                return f"lane trajectory belongs to a stale {label}"
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
        confidence = effective_lane_confidence(snapshot, live_match)
        if not math.isfinite(confidence):
            return "lane authority confidence is non-finite"
        if confidence < MIN_LANE_TRAJECTORY_CONFIDENCE:
            return (f"lane authority confidence {confidence:.6f} is below "
                    f"{MIN_LANE_TRAJECTORY_CONFIDENCE:.2f}")
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


def navigation_command(state, snapshot, *, gps_active, now=None, packet=None):
    """Read one finished command, with its existing trajectory identity.

    A fresh map heartbeat is not evidence that an older steering calculation
    used the new pose/revision. Check the packet, not independent scalar keys.
    Legacy/recorded producers retain their existing scalar contract.
    """
    now = time.monotonic() if now is None else float(now)
    # The caller may provide the exact immutable packet it observed.  This is
    # important for dense diagnostics: the map plugin can publish a newer
    # packet between command acceptance and end-of-tick recording, which used
    # to pair the accepted steer_raw with the *next* packet's geometry.
    packet = dict(packet if packet is not None
                  else (state.get("nav_steering_debug", {}) or {}))
    try:
        if gps_active and packet.get("controller") == "frenet_bicycle":
            if (not packet.get("authority_valid", False)
                    or int(packet["authority_revision"]) != int(snapshot["revision"])):
                return 0.0, 0.0, "steering command belongs to an invalid/stale revision"
            if packet.get("navigation_intent_id") != snapshot.get("navigation_intent_id"):
                return 0.0, 0.0, "steering command belongs to a different navigation intent"
            identity_fields = (
                "route_build_id", "source_game_session_id", "source_map_key",
                "source_dataset_fingerprint")
            for key in identity_fields:
                if not snapshot.get(key) or packet.get(key) != snapshot.get(key):
                    return 0.0, 0.0, f"steering command {key} is stale"
            for key in ("computed_at", "observation_timestamp"):
                age = now - float(packet[key])
                if not math.isfinite(age) or not 0.0 <= age <= 0.5:
                    return 0.0, 0.0, f"steering command {key} is stale"
            target, curvature = float(packet["output"]), float(packet["local_curvature"])
        else:
            target = float(state.get("nav_steering", 0.0) or 0.0)
            curvature = float(state.get("path_curve_signed_curvature", 0.0) or 0.0)
        if not math.isfinite(target) or not math.isfinite(curvature):
            return 0.0, 0.0, "steering command is non-finite"
    except (TypeError, ValueError, OverflowError, KeyError, AttributeError):
        return 0.0, 0.0, "steering command metadata is malformed"
    return target, curvature, ""


def _authority_reason_key(reason):
    """Stable log category for failure text containing live measurements."""
    text = str(reason or "")
    if text.startswith("truck is ") and text.endswith(
            " outside the confirmed GPS lane"):
        return "outside_confirmed_gps_lane"
    if text.startswith("truck heading differs from the GPS lane"):
        return "gps_lane_heading_mismatch"
    if (text.startswith("lane authority confidence ")
            and text.endswith(f" is below {MIN_LANE_TRAJECTORY_CONFIDENCE:.2f}")):
        return "lane_authority_confidence"
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
      * GPS lateral control is one geometric Route target followed by one
        physical slew limit. No temporal steering average or second feedback
        loop can carry an old bend into the next segment.
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
        self._steering_dynamics = SteeringDynamics()
        self._steering_dynamics_debug = dict(
            self._steering_dynamics.last_debug)
        self._last_steering_event = (False, False)
        self._last_control_dt = 0.0
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
        self._reverse_recovery_owned = False
        self._automatic_brake_stop = False
        self._drive_request_t = 0.0
        self._drive_engage_started = 0.0
        self._lane_lock_acquired = False
        self._last_authority_stop_reason = None
        # Observational only: about 55 seconds at the measured ~65 Hz runtime.
        # Nothing in the controller reads this buffer.
        self._steering_replay = SteeringReplayBuffer()
        self._accepted_navigation_command = {}

    def on_stop(self):
        self._export_steering_replay("plugin_stop")
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

    def _publish_automatic_disable(self, reason):
        """Publish one atomic disable event for UI and main-console relay."""
        reason = str(reason or "unknown safety reason")
        seq = time.monotonic_ns()
        self.sdk.shared_state.update_batch({
            "autopilot_active": False,
            "nav_active": False,
            "nav_steering": 0.0,
            "safety_hazard_active": True,
            "autopilot_disable_reason": reason,
            "autopilot_log_event": {
                "seq": seq,
                "level": "WARNING",
                "message": f"Autopilot automatically disabled: {reason}",
            },
        })
        # File I/O is deliberately after the atomic safety transition. A slow
        # or failed diagnostic export must never delay actuator authority loss.
        self._export_steering_replay("automatic_disable", reason)

    def _steering_replay_identity(self):
        snapshot = self.sdk.shared_state.get("lane_trajectory", {}) or {}
        return {
            "navigation_intent_id": snapshot.get(
                "navigation_intent_id",
                self.sdk.shared_state.get("navigation_intent_id")),
            "route_build_id": snapshot.get("route_build_id"),
            "revision": snapshot.get("revision"),
            "source_game_session_id": snapshot.get(
                "source_game_session_id",
                self.sdk.shared_state.get("game_session_id")),
            "source_map_key": snapshot.get(
                "source_map_key",
                self.sdk.shared_state.get("active_map_key")),
            "source_dataset_fingerprint": snapshot.get(
                "source_dataset_fingerprint",
                self.sdk.shared_state.get("active_dataset_fingerprint")),
        }

    def _export_steering_replay(self, event, detail=""):
        replay = getattr(self, "_steering_replay", None)
        if replay is None or len(replay) == 0:
            return None
        try:
            directory = os.path.join(app_dir(), "route-diagnostics")
            path = replay.export(
                directory, reason=event,
                identity={**self._steering_replay_identity(),
                          "detail": str(detail or "")})
            logging.info(
                "Dense steering replay exported: samples=%d reason=%s path=%s",
                len(replay), event, path)
            return path
        except Exception as error:
            # Diagnostics must never change control or prevent shutdown.
            logging.warning("Dense steering replay export failed: %s", error)
            return None

    def _record_steering_replay_tick(self, truck, snapshot, steering_val,
                                     observed_game_steering, speed_kmh,
                                     authority_reason):
        replay = getattr(self, "_steering_replay", None)
        if replay is None:
            return
        steering_debug = self.sdk.shared_state.get(
            "nav_steering_debug", {}) or {}
        accepted = dict(getattr(
            self, "_accepted_navigation_command", {}) or {})
        dynamics = dict(getattr(
            self, "_steering_dynamics_debug", {}) or {})
        match = (self.sdk.shared_state.get("lane_match")
                 or snapshot.get("lane_match") or {})
        trailer = steering_debug.get("trailer_envelope", {}) or {}
        lane_id = (match.get("active_lane_id")
                   or snapshot.get("active_lane_id") or {})
        replay.append({
            "control_dt_s": getattr(self, "_last_control_dt", None),
            "sdk_frame_us": truck.get("sdkFrameTimeUs"),
            "observation_timestamp": steering_debug.get(
                "observation_timestamp"),
            "computed_at": steering_debug.get("computed_at"),
            "accepted_packet_computed_at": accepted.get("computed_at"),
            "accepted_packet_observation_timestamp": accepted.get(
                "observation_timestamp"),
            "accepted_packet_sdk_frame_us": accepted.get("sdk_frame_us"),
            "accepted_packet_output": accepted.get("output"),
            "accepted_nav_command": accepted.get("command"),
            "accepted_nav_curvature_per_m": accepted.get("curvature_per_m"),
            "accepted_command_rejection": accepted.get("rejection"),
            "accepted_packet_controller": accepted.get("controller"),
            "accepted_packet_revision": accepted.get("authority_revision"),
            "accepted_packet_navigation_intent_id": accepted.get(
                "navigation_intent_id"),
            "accepted_packet_route_build_id": accepted.get("route_build_id"),
            "autopilot_active": bool(self.sdk.shared_state.get(
                "autopilot_active", False)),
            "nav_active": bool(self.sdk.shared_state.get("nav_active", False)),
            "speed_kmh": speed_kmh,
            # Preserve the established console/replay field names and also
            # carry explicit-unit aliases for downstream analysis.
            "lane_cte": match.get("lateral_error_m"),
            "lane_cte_m": match.get("lateral_error_m"),
            "lane_heading": match.get("heading_error_rad"),
            "lane_heading_error_rad": match.get("heading_error_rad"),
            "lane_id": lane_id,
            "elevation_layer": match.get("elevation_layer"),
            "preview_k": steering_debug.get("preview_curvature"),
            "preview_k_per_m": steering_debug.get("preview_curvature"),
            "local_k_per_m": steering_debug.get("local_curvature"),
            "tracking_progress_m": steering_debug.get("tracking_progress_m"),
            "tracking_segment_index": steering_debug.get(
                "tracking_segment_index"),
            "tracking_segment_fraction": steering_debug.get(
                "tracking_segment_fraction"),
            "tracking_projection_xz": steering_debug.get(
                "tracking_projection_xz"),
            "local_tangent_heading_rad": steering_debug.get(
                "local_tangent_heading_rad"),
            "observation_xz": steering_debug.get("observation_xz"),
            "observation_heading_rad": steering_debug.get(
                "observation_heading_rad"),
            "curve_reference": steering_debug.get("feed_forward"),
            "heading_feedback": steering_debug.get("heading_feedback"),
            "cte_feedback": steering_debug.get("cte_feedback"),
            "guidance_delta": steering_debug.get("feedback"),
            "guidance_applied": steering_debug.get("feedback_applied"),
            "steer_raw": dynamics.get("raw_target"),
            "steer_bounded": dynamics.get("bounded_target"),
            "steer_out": steering_val,
            "engine_applied_steering": self.sdk.shared_state.get(
                "engine_applied_steering"),
            "game_steer": observed_game_steering,
            "game_steer_right": observed_game_steering,
            "tyre_angles_rad": truck.get("roadWheelAnglesRad", []),
            "yaw_right_rad_s": truck.get("yawRateRadS"),
            "vehicle_curvature_source": steering_debug.get(
                "vehicle_curvature_source"),
            "curvature_observation_weight": steering_debug.get(
                "curvature_observation_weight"),
            "predicted_frenet_cte_m": steering_debug.get(
                "predicted_frenet_cte_m"),
            "predicted_frenet_heading_error_rad": steering_debug.get(
                "predicted_frenet_heading_error_rad"),
            "steering_rate_per_s": dynamics.get("rate_per_s"),
            "steering_acceleration_per_s2": dynamics.get(
                "acceleration_per_s2"),
            "trailer_cte_m": trailer.get("trailer_cte_m"),
            "trailer_required_offset_m": trailer.get("required_offset_m"),
            "trailer_applied_offset_m": trailer.get("applied_offset_m"),
            "trailer_curvature_per_m": trailer.get(
                "preview_curvature_per_m"),
            "authority_rejection": str(authority_reason or ""),
            **self._steering_replay_identity(),
        })

    def on_tick(self, delta_time: float):
        self._last_control_dt = float(delta_time)
        # Longitudinal ramps also reject a scheduler-sized jump. Steering uses
        # its own 100 ms physical integration bound internally.
        dt = min(max(float(delta_time), 1e-3), 0.10)
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
        snapshot = self.sdk.shared_state.get("lane_trajectory", {}) or {}
        try:
            snapshot_revision = int(snapshot.get("revision", -1) or -1)
            snapshot_confidence = float(snapshot.get("confidence", 0.0) or 0.0)
        except (TypeError, ValueError, OverflowError):
            snapshot_revision, snapshot_confidence = -1, 0.0
        gps_navigation_present = game_gps_navigation_present(
            self.sdk.shared_state, snapshot)
        accepted_packet = dict(self.sdk.shared_state.get(
            "nav_steering_debug", {}) or {})
        nav_command, nav_command_curvature, command_reason = navigation_command(
            self.sdk.shared_state, snapshot, gps_active=gps_navigation_present,
            packet=accepted_packet)
        self._accepted_navigation_command = {
            **accepted_packet,
            "command": float(nav_command),
            "curvature_per_m": float(nav_command_curvature),
            "rejection": str(command_reason or ""),
        }
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
        if (not authority_reason and command_reason
                and self.sdk.shared_state.get("nav_active", False)):
            authority_reason = command_reason
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

        # Handover also applies on braking/toll early returns, not only cruise.
        # SCS gameSteer is left-positive; the one actuator is right-positive.
        try:
            observed_game_steering = -float(truck.get("gameSteer", 0.0) or 0.0)
            if not math.isfinite(observed_game_steering):
                observed_game_steering = 0.0
            observed_game_steering = float(np.clip(observed_game_steering, -1., 1.))
        except (TypeError, ValueError, OverflowError):
            observed_game_steering = 0.0
        if not autopilot_engaged or not self._was_active:
            self._reset_steering_dynamics(observed_game_steering)
        self._was_active = autopilot_engaged
        self._engage_blend = 1.0 if autopilot_engaged else 0.0

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
                self.sdk.shared_state.set("safety_hazard_active", True)
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
                self._publish_automatic_disable(authority_reason)
                logging.warning(
                    "Autopilot automatically disengaged after safety stop: %s",
                    authority_reason)
            self._publish_control_tags(speed_kmh, False)
            return

        if navigation_authority_safe:
            self._last_authority_stop_reason = None

        reverse_signal = bool(float(speed) < -0.10 or gear < 0)
        if autopilot_engaged and reverse_signal and not self._reverse_recovery:
            self._reverse_recovery = True
            # A reverse ratio reached while UltraPilot itself was holding the
            # service brake is ETS2's automatic brake-to-reverse gesture, not
            # an unexplained driver selection.
            self._reverse_recovery_owned = bool(self._automatic_brake_stop)
        reversing = bool(autopilot_engaged and
                         (reverse_signal or self._reverse_recovery))
        if reversing:
            self.sdk.controller.set_throttle(0.0)
            self._last_throttle = 0.0
            self._last_steering = self._ramp_steering(0.0, dt)
            self.sdk.controller.set_steering(self._last_steering)
            if float(speed) < -0.10 or speed_kmh > 0.5:
                self._set_brake(0.62, dt)
                self.sdk.shared_state.set(
                    "navigation_status", "Zastavujem neočakávanú spiatočku")
                self._publish_control_tags(speed_kmh, False)
                return
            if gear < 0 and self._reverse_recovery_owned:
                # Release the brake gesture before requesting D. Keep
                # throttle at zero until telemetry proves reverse is gone.
                self.sdk.controller.set_brake(0.0)
                self._last_brake = 0.0
                now = time.monotonic()
                if (self._drive_request_t <= 0.0
                        or now - self._drive_request_t >= DRIVE_RETRY_S):
                    self.sdk.controller.select_drive(True)
                    self._drive_request_t = now
                self.sdk.shared_state.set(
                    "navigation_status",
                    "Obnovujem jazdu dopredu po automatickej spiatočke")
                self._publish_control_tags(speed_kmh, False)
                return
            if gear < 0:
                # A reverse selection not caused by our own brake remains a
                # genuine fail-closed event.
                self.sdk.controller.set_brake(0.0)
                self.sdk.controller.select_drive(False)
                self._last_brake = 0.0
                self._publish_automatic_disable("unexpected reverse gear")
                logging.warning(
                    "Autopilot automatically disengaged: unexpected reverse gear")
                self.sdk.shared_state.set(
                    "navigation_status", "Autopilot vypnutý po spiatočke")
                self._reverse_recovery = False
                self._reverse_recovery_owned = False
                self._automatic_brake_stop = False
                self._drive_engage_started = 0.0
                self._publish_control_tags(speed_kmh, False)
                return
            # D/neutral is proven again; continue through the normal drive
            # handshake without needlessly disabling the autopilot.
            self._reverse_recovery = False
            self._reverse_recovery_owned = False
            self._automatic_brake_stop = False

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
                # The selector handshake is longitudinal only. It must not
                # leave the previous CTL_STEERING value latched for up to
                # 450 ms: execute this tick's already validated packet through
                # the same sole physical dynamics used by normal driving.
                settle_nav_active = bool(
                    navigation_authority_safe
                    and self.sdk.shared_state.get("nav_active", False))
                self._last_steering = self._ramp_steering(
                    nav_command if settle_nav_active else 0.0, dt,
                    speed_ms=abs(speed),
                    curvature_per_m=nav_command_curvature)
                self.sdk.controller.set_steering(self._last_steering)
                self.sdk.shared_state.set(
                    "navigation_status", "Pripravujem jazdu dopredu")
                self._publish_control_tags(speed_kmh, settle_nav_active)
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
            self._automatic_brake_stop = bool(
                autopilot_engaged and speed_kmh < 1.0
                and self._last_brake > BRAKE_MIN_HOLD)
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
                target = float(np.clip(nav_command, -1.0, 1.0))
            else:
                target = 0.0
            self._last_steering = self._ramp_steering(
                target, dt, speed_ms=abs(speed), curvature_per_m=nav_command_curvature)
            self.sdk.controller.set_steering(self._last_steering)
            self.sdk.shared_state.set("tts_message", "Emergency stop triggered!")
            self._publish_control_tags(speed_kmh, emergency_nav_active)
            return

        if system_state == "PAY_TOLL":
            toll_nav_active = bool(navigation_authority_safe
                and self.sdk.shared_state.get("nav_active", False))
            self._last_steering = self._ramp_steering(
                nav_command if toll_nav_active else 0.0, dt,
                speed_ms=abs(speed), curvature_per_m=nav_command_curvature)
            self.sdk.controller.set_steering(self._last_steering)
            if speed_kmh > 0.5:
                self.sdk.controller.set_throttle(0.0)
                self._last_throttle = 0.0
                self._set_brake(0.7, dt)
            else:
                self._set_brake(0.0, dt)
                self.sdk.controller.pay_toll()
            self._publish_control_tags(speed_kmh, toll_nav_active)
            return

        # --- Gather all brake requests, combine via max() -------------------
        collision_brake = float(self.sdk.shared_state.get("collision_brake_request", 0.0) or 0.0)
        try:
            traffic_age = time.monotonic() - float(
                self.sdk.shared_state.get(
                    "traffic_snapshot_timestamp", 0.0) or 0.0)
        except (TypeError, ValueError, OverflowError):
            traffic_age = float("inf")
        traffic_brake = (float(self.sdk.shared_state.get(
            "traffic_brake", 0.0) or 0.0)
            if (self.sdk.shared_state.get("traffic_snapshot_valid", False)
                and 0.0 <= traffic_age <= 0.5) else 0.0)
        light_brake = float(self.sdk.shared_state.get("light_brake", 0.0) or 0.0)
        aux_brake = float(self.sdk.shared_state.get("aux_brake_request", 0.0) or 0.0)
        # Raw screenshot danger is diagnostic only. ``danger_level`` now
        # represents lane-aligned SCS traffic, already covered by
        # traffic_brake/collision_brake, so it must not become a duplicate
        # braking channel.
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
        curve_brake = 0.0
        curve_geometry_available = False
        if radius is not None:
            try:
                R = float(radius)
                distance_to_curve = float(curve_distance or 0.0)
            except (TypeError, ValueError, OverflowError):
                R = 1e6
                distance_to_curve = 0.0
            curve_geometry_available = bool(
                math.isfinite(R) and R > 0.0)
            if 0.0 < R < 2000.0:
                v_now = abs(speed)                        # m/s
                curve_limit_ms, _usable_curve_distance = \
                    planned_curve_speed_limit_ms(
                        R, distance_to_curve, v_now)
                if v_now > curve_limit_ms + CURVE_BRAKE_MARGIN_MS:
                    excess = v_now - curve_limit_ms
                    curve_brake = float(np.clip(
                        excess / 5.0, 0.0, CURVE_BRAKE_MAX))
                    requested_brake = max(requested_brake, curve_brake)
                    curve_factor = max(
                        0.0, min(1.0, curve_limit_ms / max(v_now, 1.0)))
                elif v_now > curve_limit_ms * 0.85:
                    # Coast into the speed envelope instead of accelerating
                    # until the brake threshold and then oscillating around it.
                    curve_factor = max(0.15, min(
                        1.0, (curve_limit_ms - v_now)
                        / max(curve_limit_ms * 0.15, 0.5)))
        self.sdk.shared_state.set(
            "path_curve_speed_limit_ms",
            (None if not math.isfinite(curve_limit_ms)
             else float(curve_limit_ms)))

        # --- Reactive curve slowdown: ease off the throttle (light brake at
        # speed) — a back-up to the proactive brake above, in case the map
        # curvature isn't published yet (e.g. no map loaded, vision only). ---
        # Steering magnitude is not road curvature: it also contains lane
        # recovery and the trailer swept-path correction. Treating it as a
        # second curve sensor caused needless braking while merely
        # re-centring and could override the proven map-speed envelope. Keep
        # this fallback only when no finite map curvature exists.
        if not curve_geometry_available:
            turn = abs(self._last_steering)
            curve_factor = min(
                curve_factor,
                1.0 if turn < 0.18
                else max(0.35, 1.0 - (turn - 0.18) * 1.6))
            if turn > 0.45 and speed_kmh > 45:
                requested_brake = max(
                    requested_brake,
                    float(np.clip(
                        (turn - 0.45) * 0.6, 0.0, 0.35)))

        # 3. Apply braking THROUGH THE RAMP (anti-jerk). This is the key change:
        #    the truck brakes firmly but progressively, never a step to 1.0.
        self._set_brake(requested_brake, dt)
        self._automatic_brake_stop = bool(
            autopilot_engaged and speed_kmh < 1.0
            and self._last_brake > BRAKE_MIN_HOLD)

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

        # Handover was synchronized before every early-return braking branch.
        active = bool(self.sdk.shared_state.get("autopilot_active", False))
        if not active:
            self._reset_steering_dynamics(observed_game_steering)
        elif navigation_unreliable:
            self._last_steering = self._ramp_steering(0.0, dt)
        elif nav_active:
            # Route publishes one finished geometric curvature command.  Do
            # not average it with an older direction: the 23:10 replay proved
            # that doing so briefly emitted the opposite sign at an S-bend and
            # then carried excess lock into the recovery. The physical slew
            # limiter below is the one and only output-shaping stage.
            nav_steering = nav_command
            self.sdk.shared_state.set(
                "nav_steering_filtered", nav_steering)
            target = float(np.clip(nav_steering, -1.0, 1.0))
            self._last_steering = self._ramp_steering(
                target, dt, speed_ms=abs(speed), curvature_per_m=nav_command_curvature)
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
            target = (VISION_STEER_FOLLOW_BLEND * raw
                      + (1 - VISION_STEER_FOLLOW_BLEND)
                      * self._last_steering)
            target = float(np.clip(target, -1.0, 1.0))
            self._last_steering = self._ramp_steering(target, dt)
        else:
            # Never substitute camera lane keeping for an invalid GPS lane at
            # an intersection. Steering returns to zero through the existing
            # rate limiter; the brake/throttle safety path above handles stop.
            self._last_steering = self._ramp_steering(0.0, dt)

        # Inactive control relinquishes the device. Active control is already
        # continuous because its actuator was initialized from gameSteer.
        steering_val = self._last_steering if active else 0.0

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
                steering_debug = (self.sdk.shared_state.get(
                    "nav_steering_debug", {}) or {})
                diagnostic_feed_forward = float(
                    steering_debug.get("feed_forward", 0.0) or 0.0)
                diagnostic_feedback = float(
                    steering_debug.get("feedback", 0.0) or 0.0)
                diagnostic_feedback_applied = float(
                    steering_debug.get(
                        "feedback_applied", diagnostic_feedback) or 0.0)
                diagnostic_heading_feedback = float(
                    steering_debug.get("heading_feedback", 0.0) or 0.0)
                diagnostic_cte_feedback = float(
                    steering_debug.get("cte_feedback", 0.0) or 0.0)
                diagnostic_sign_projected = bool(
                    steering_debug.get(
                        "curve_sign_projection_active", False))
                diagnostic_direction_hold = bool(
                    steering_debug.get("curve_direction_hold", False))
                diagnostic_hold_fraction = float(
                    steering_debug.get(
                        "curve_direction_hold_fraction", 0.0) or 0.0)
                diagnostic_cte_residual = float(
                    steering_debug.get("cte_geometry_residual", 0.0) or 0.0)
                diagnostic_cte_proven = bool(
                    steering_debug.get(
                        "curve_direction_hold_error_proven", False))
                diagnostic_low_speed_capture = bool(
                    steering_debug.get("low_speed_capture_active", False))
                diagnostic_guidance_lookahead = float(
                    steering_debug.get("guidance_lookahead_m", 0.0) or 0.0)
                diagnostic_guidance_heading = math.degrees(float(
                    steering_debug.get(
                        "guidance_heading_error_rad", 0.0) or 0.0))
                diagnostic_guidance_curvature = float(
                    steering_debug.get("guidance_curvature", 0.0) or 0.0)
                diagnostic_preview_curvature = float(
                    steering_debug.get("preview_curvature", 0.0) or 0.0)
                trailer_debug = steering_debug.get(
                    "trailer_envelope", {}) or {}
                diagnostic_trailer_cte = float(
                    trailer_debug.get("trailer_cte_m", 0.0) or 0.0)
                diagnostic_trailer_required = float(
                    trailer_debug.get("required_offset_m", 0.0) or 0.0)
                diagnostic_trailer_offset = float(
                    trailer_debug.get("applied_offset_m", 0.0) or 0.0)
                diagnostic_trailer_reason = str(
                    trailer_debug.get("reason", "") or "")
                diagnostic_trailer_side_proven = bool(
                    trailer_debug.get("curve_side_proven", False))
                diagnostic_trailer_curvature = float(
                    trailer_debug.get(
                        "preview_curvature_per_m", 0.0) or 0.0)
                diagnostic_trailer_fraction = float(
                    trailer_debug.get(
                        "balanced_reference_fraction", 0.0) or 0.0)
                diagnostic_trailer_curve_source = str(
                    trailer_debug.get("curvature_source", "none") or "none")
                lane_id_payload = (diagnostic_match.get("active_lane_id")
                                   or snapshot.get("active_lane_id") or {})
                diagnostic_lane_id = (
                    f"{lane_id_payload.get('road_uid', '-')}:"
                    f"{lane_id_payload.get('direction', '-')}:"
                    f"{lane_id_payload.get('lane_index', '-')}:"
                    f"{lane_id_payload.get('prefab_token') or '-'}:"
                    f"{lane_id_payload.get('connector_index') if lane_id_payload.get('connector_index') is not None else '-'}")
                diagnostic_elevation_layer = diagnostic_match.get(
                    "elevation_layer", "-")
                dynamics_debug = dict(getattr(
                    self, "_steering_dynamics_debug", {}) or {})
                diagnostic_raw_target = float(
                    dynamics_debug.get("raw_target", 0.0) or 0.0)
                diagnostic_bounded_target = float(
                    dynamics_debug.get("bounded_target", 0.0) or 0.0)
                diagnostic_steering_rate = float(
                    dynamics_debug.get("rate_per_s", 0.0) or 0.0)
                diagnostic_steering_accel = float(
                    dynamics_debug.get("acceleration_per_s2", 0.0) or 0.0)
                diagnostic_steering_error = float(
                    dynamics_debug.get("target_error", 0.0) or 0.0)
                diagnostic_stopping_distance = float(
                    dynamics_debug.get("stopping_distance", 0.0) or 0.0)
                diagnostic_safe_rate = float(
                    dynamics_debug.get("safe_rate_per_s", 0.0) or 0.0)
                diagnostic_trajectory_phase = str(
                    dynamics_debug.get("trajectory_phase", "unknown")
                    or "unknown")
                diagnostic_dt = float(getattr(
                    self, "_last_control_dt", 0.0) or 0.0)
                diagnostic_dt_used = float(
                    dynamics_debug.get("dt_used_s", 0.0) or 0.0)
                diagnostic_game_steering = float(observed_game_steering)
                diagnostic_game_tracking = (
                    diagnostic_game_steering - float(steering_val))
                diagnostic_lock_rad = float(
                    steering_debug.get("steering_lock_rad", float("nan")))
                tyre_angles = [float(value) for value in
                               (truck.get("roadWheelAnglesRad", []) or [])
                               if math.isfinite(float(value))]
                diagnostic_tyre_angle = (
                    sum(tyre_angles) / len(tyre_angles)
                    if tyre_angles else float("nan"))
                diagnostic_command_angle = (
                    float(steering_val) * diagnostic_lock_rad)
                diagnostic_game_to_command = (
                    diagnostic_game_steering / float(steering_val)
                    if abs(float(steering_val)) >= 0.05 else float("nan"))
                diagnostic_tyre_per_game = (
                    diagnostic_tyre_angle / diagnostic_game_steering
                    if abs(diagnostic_game_steering) >= 0.05
                    else float("nan"))
                yaw_rate = float(truck.get("yawRateRadS", float("nan")))
                diagnostic_yaw_curvature = (
                    yaw_rate / abs(float(speed))
                    if (truck.get("yawRateValid", False)
                        and abs(float(speed)) > 1.0
                        and math.isfinite(yaw_rate)) else float("nan"))
                diagnostic_dynamics_flags = ",".join(name for name in (
                    "target_saturated", "rate_limited",
                    "acceleration_limited", "dt_limited",
                    "deadband_active") if dynamics_debug.get(name, False)) or "none"
            except (TypeError, ValueError, OverflowError):
                live_lateral = live_heading = float("nan")
                diagnostic_radius = None
                diagnostic_curve_distance = float("nan")
                diagnostic_feed_forward = diagnostic_feedback = float("nan")
                diagnostic_feedback_applied = float("nan")
                diagnostic_heading_feedback = diagnostic_cte_feedback = float("nan")
                diagnostic_sign_projected = False
                diagnostic_direction_hold = False
                diagnostic_hold_fraction = diagnostic_cte_residual = float("nan")
                diagnostic_cte_proven = False
                diagnostic_low_speed_capture = False
                diagnostic_guidance_lookahead = float("nan")
                diagnostic_guidance_heading = float("nan")
                diagnostic_guidance_curvature = float("nan")
                diagnostic_preview_curvature = float("nan")
                diagnostic_trailer_cte = float("nan")
                diagnostic_trailer_required = float("nan")
                diagnostic_trailer_offset = float("nan")
                diagnostic_trailer_reason = "malformed"
                diagnostic_trailer_side_proven = False
                diagnostic_trailer_curvature = float("nan")
                diagnostic_trailer_fraction = float("nan")
                diagnostic_trailer_curve_source = "malformed"
                diagnostic_lane_id = "malformed"
                diagnostic_elevation_layer = "-"
                diagnostic_raw_target = diagnostic_bounded_target = float("nan")
                diagnostic_steering_rate = diagnostic_steering_accel = float("nan")
                diagnostic_steering_error = float("nan")
                diagnostic_stopping_distance = diagnostic_safe_rate = float("nan")
                diagnostic_trajectory_phase = "malformed"
                diagnostic_dt = diagnostic_dt_used = float("nan")
                diagnostic_game_steering = diagnostic_game_tracking = float("nan")
                diagnostic_lock_rad = diagnostic_tyre_angle = float("nan")
                diagnostic_command_angle = diagnostic_game_to_command = float("nan")
                diagnostic_tyre_per_game = diagnostic_yaw_curvature = float("nan")
                diagnostic_dynamics_flags = "malformed"
            self.sdk.shared_state.set("steering_dynamics_diagnostic", {
                **dict(getattr(self, "_steering_dynamics_debug", {}) or {}),
                "feed_forward": diagnostic_feed_forward,
                "feedback": diagnostic_feedback,
                "feedback_applied": diagnostic_feedback_applied,
                "heading_feedback": diagnostic_heading_feedback,
                "cte_feedback": diagnostic_cte_feedback,
                "curve_sign_projection_active": diagnostic_sign_projected,
                "lane_cte_m": live_lateral,
                "lane_heading_deg": live_heading,
                "curvature_per_m": diagnostic_guidance_curvature,
                "preview_curvature_per_m": diagnostic_preview_curvature,
                "trailer_curvature_per_m": diagnostic_trailer_curvature,
                "trailer_reference_fraction": diagnostic_trailer_fraction,
                "trailer_curvature_source": diagnostic_trailer_curve_source,
                "lookahead_m": diagnostic_guidance_lookahead,
                "observed_game_steering": diagnostic_game_steering,
                "game_steer_tracking_error": diagnostic_game_tracking,
                "configured_lock_rad": diagnostic_lock_rad,
                "commanded_tyre_angle_rad": diagnostic_command_angle,
                "measured_tyre_angle_rad": diagnostic_tyre_angle,
                "game_to_command_ratio": diagnostic_game_to_command,
                "tyre_rad_per_game_input": diagnostic_tyre_per_game,
                "yaw_curvature_per_m": diagnostic_yaw_curvature,
                "navigation_intent_id": self.sdk.shared_state.get(
                    "navigation_intent_id"),
                "revision": snapshot_revision,
                "authority_rejection": authority_reason,
                "timestamp": time.monotonic(),
            })
            logging.info(
                "autopilot: active=%s nav=%s engage=%.2f lane_cte=%.3f "
                "lane_heading=%.1fdeg vision_off=%.3f "
                "nav_steer=%.3f nav_target=%.3f "
                "steer_out=%.3f speed=%.0f "
                "curve_reference=%.3f guidance_delta=%.3f "
                "guidance_applied=%.3f curve_hold=%s "
                "hold_fraction=%.2f sign_projected=%s "
                "cte_residual=%.3f cte_proven=%s "
                "low_speed_capture=%s guidance_L=%.1fm "
                "guidance_heading=%.1fdeg guidance_k=%.5f preview_k=%.5f "
                "curve_r=%s curve_d=%.1f curve_limit=%s "
                "brake=%.3f brake_req=%.3f curve_brake=%.3f "
                "collision_brake=%.3f traffic_brake=%.3f light_brake=%.3f "
                "aux_brake=%.3f vision_brake=%.3f "
                "trailer_cte=%.3f trailer_required=%.3f trailer_offset=%.3f "
                "trailer_k=%.5f trailer_fraction=%.2f "
                "trailer_side_proven=%s trailer_curve_source=%s "
                "trailer_reason=%s "
                "engine_steer=%.3f articulation_guard=%s "
                "steer_raw=%.3f steer_bounded=%.3f steer_rate=%.3f/s "
                "steer_accel=%.3f/s2 dt=%.4f used_dt=%.4f "
                "steer_phase=%s steer_error=%.3f stop_angle=%.3f "
                "safe_rate=%.3f/s game_steer=%.3f game_tracking=%.3f "
                "heading_fb=%.3f cte_fb=%.3f steer_flags=%s "
                "intent=%s lane_revision=%s LaneId=%s elevation=%s "
                "confidence=%.3f reject=%s lock_rad=%.3f "
                "command_tyre_rad=%.4f game_to_command=%s "
                "tyre_rad_per_game=%s yaw_k=%s "
                "tyre_angles_rad=%s yaw_right_rad_s=%s sdk_frame_us=%s",
                active, nav_active, self._engage_blend,
                live_lateral, live_heading, float(lane_offset),
                float(self.sdk.shared_state.get("nav_steering", 0.0) or 0.0),
                float(self._last_steering), steering_val, speed_kmh,
                diagnostic_feed_forward, diagnostic_feedback,
                diagnostic_feedback_applied,
                diagnostic_direction_hold,
                diagnostic_hold_fraction, diagnostic_sign_projected,
                diagnostic_cte_residual,
                diagnostic_cte_proven,
                diagnostic_low_speed_capture,
                diagnostic_guidance_lookahead,
                diagnostic_guidance_heading,
                diagnostic_guidance_curvature,
                diagnostic_preview_curvature,
                ("-" if diagnostic_radius is None
                 else f"{diagnostic_radius:.1f}"),
                diagnostic_curve_distance,
                ("-" if not math.isfinite(curve_limit_ms)
                 else f"{curve_limit_ms * 3.6:.1f}kmh"),
                self._last_brake, requested_brake, curve_brake,
                collision_brake, traffic_brake, light_brake,
                aux_brake, vision_brake,
                diagnostic_trailer_cte, diagnostic_trailer_required,
                diagnostic_trailer_offset,
                diagnostic_trailer_curvature,
                diagnostic_trailer_fraction,
                diagnostic_trailer_side_proven,
                diagnostic_trailer_curve_source,
                diagnostic_trailer_reason.replace(" ", "_"),
                float(self.sdk.shared_state.get(
                    "engine_applied_steering", steering_val) or 0.0),
                bool(self.sdk.shared_state.get(
                    "trailer_articulation_guarded", False)),
                diagnostic_raw_target, diagnostic_bounded_target,
                diagnostic_steering_rate, diagnostic_steering_accel,
                diagnostic_dt, diagnostic_dt_used,
                diagnostic_trajectory_phase, diagnostic_steering_error,
                diagnostic_stopping_distance, diagnostic_safe_rate,
                diagnostic_game_steering, diagnostic_game_tracking,
                diagnostic_heading_feedback, diagnostic_cte_feedback,
                diagnostic_dynamics_flags,
                self.sdk.shared_state.get("navigation_intent_id"),
                snapshot_revision,
                diagnostic_lane_id, diagnostic_elevation_layer,
                snapshot_confidence, authority_reason, diagnostic_lock_rad,
                diagnostic_command_angle,
                ("-" if not math.isfinite(diagnostic_game_to_command)
                 else f"{diagnostic_game_to_command:.3f}"),
                ("-" if not math.isfinite(diagnostic_tyre_per_game)
                 else f"{diagnostic_tyre_per_game:.3f}"),
                ("-" if not math.isfinite(diagnostic_yaw_curvature)
                 else f"{diagnostic_yaw_curvature:.5f}"),
                truck.get("roadWheelAnglesRad", []),
                truck.get("yawRateRadS"), truck.get("sdkFrameTimeUs"))

        self.sdk.controller.set_steering(steering_val)
        self._record_steering_replay_tick(
            truck, snapshot, steering_val, observed_game_steering,
            speed_kmh, authority_reason)

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

    def _reset_steering_dynamics(self, command: float = 0.0) -> float:
        dynamics = getattr(self, "_steering_dynamics", None)
        if dynamics is None:
            dynamics = SteeringDynamics(command)
            self._steering_dynamics = dynamics
        self._last_steering = dynamics.reset(command)
        self._steering_dynamics_debug = dict(dynamics.last_debug)
        return self._last_steering

    def _ramp_steering(self, target: float, dt: float, *,
                       speed_ms=None, curvature_per_m=None) -> float:
        """Apply the sole physical steering-angle/rate/acceleration model.

        This is a state-space actuator, not a moving-average filter. Geometry
        remains current while normalized angle, angular rate and angular
        acceleration obey speed-scheduled, real-time bounds.
        """
        dynamics = getattr(self, "_steering_dynamics", None)
        if dynamics is None:
            dynamics = SteeringDynamics(getattr(self, "_last_steering", 0.0))
            self._steering_dynamics = dynamics
        # Tests and safety transitions may explicitly synchronize the public
        # command. Never let a hidden actuator state retain an older lock.
        current = float(getattr(self, "_last_steering", 0.0) or 0.0)
        if abs(dynamics.command - current) > 1e-9:
            dynamics.reset(current)
        try:
            if speed_ms is None:
                speed_ms = (abs(float(getattr(
                    self, "_speed_kmh", 0.0) or 0.0)) / 3.6)
            steering_debug = self.sdk.shared_state.get(
                "nav_steering_debug", {}) or {}
            if curvature_per_m is None:
                curvature_per_m = float(steering_debug.get(
                    "local_curvature",
                    self.sdk.shared_state.get(
                        "path_curve_signed_curvature", 0.0)) or 0.0)
        except (AttributeError, TypeError, ValueError, OverflowError):
            speed_ms = 0.0 if speed_ms is None else float(speed_ms)
            curvature_per_m = (0.0 if curvature_per_m is None
                               else float(curvature_per_m))
        output = dynamics.update(
            target, dt, speed_ms=speed_ms,
            curvature_per_m=curvature_per_m or 0.0)
        self._steering_dynamics_debug = dict(dynamics.last_debug)
        event = (bool(dynamics.last_debug["target_saturated"]),
                 bool(dynamics.last_debug["dt_limited"]))
        previous_event = getattr(self, "_last_steering_event", (False, False))
        if event != previous_event and (any(event) or any(previous_event)):
            logging.info(
                "Steering dynamics event: target_saturated=%s dt_limited=%s "
                "raw=%.3f bounded=%.3f dt=%.4f used_dt=%.4f speed=%.2fm/s",
                event[0], event[1], dynamics.last_debug["raw_target"],
                dynamics.last_debug["bounded_target"],
                dynamics.last_debug["dt_s"],
                dynamics.last_debug["dt_used_s"],
                dynamics.last_debug["speed_ms"])
        self._last_steering_event = event
        return output
