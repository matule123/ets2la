import logging
import math
from sdk.base_plugin import BasePlugin


# === Tuning =================================================================
# --- Speed policy -----------------------------------------------------------
# Lateral acceleration a loaded ETS2 truck holds comfortably (m/s^2). Drives the
# curve-safe-speed law: v_safe = sqrt(A_LAT * radius).
A_LAT_MAX = 2.5
AUX_BRAKE_MAX = 0.35      # aux brake nudge when over the plan
AUX_OVERSHOOT_MS = 1.0    # how far over plan before aux brake kicks in

# --- Lane / trailer-offset policy ------------------------------------------
# ETS2 is right-hand traffic: we hold the right lane by default.
BASE_LANE_OFFSET_M = 2.7      # right of the road centreline
LANE_HALF_WIDTH_M = 1.75      # half a 3.5 m lane — how far we can shift before
                              # we'd cross into the next lane
# Trailer geometry we can't read directly (the SDK doesn't expose trailer
# length). We estimate it: a coupled trailer is flagged by `trailer_attached`,
# and we assume a typical semi length. The articulation angle lets us refine
# how much of that length is currently swung offline.
ASSUMED_TRAILER_LEN_M = 10.0
# How much of the trailer's tail we allow to cross toward the lane edge before
# we MUST swing wide. Bigger = more tolerance = less nudging.
TRAILER_EDGE_TOLERANCE_M = 0.4
# Scale back the nudge when there's oncoming traffic close by — we'd rather
# kiss the kerb than meet a truck head-on. Below this gap we cut the outward
# nudge entirely.
ONCOMING_SUPPRESS_GAP_M = 40.0

# --- Curve-preview (look-ahead profile) ------------------------------------
# We sample the path's lateral position at several distances ahead and turn the
# set of samples into a curve-profile: a bend angle at each band. The nudge is
# driven by the TIGHTEST band we can still reach in time, so a gentle bend that
# barely curves is ignored (no nudge), and we start moving BEFORE a sharp bend
# arrives — not inside it.
# Sample bands (metres ahead). Near → far; each becomes one curvature sample.
PREVIEW_BANDS_M = (10.0, 20.0, 35.0, 55.0, 80.0)
# A band below this bend angle (radians, ~tangent) counts as "straight" → no
# nudge from it. Set so motorway sweeps (very gentle) don't trigger anything.
MIN_BEND_ANGLE = 0.10
# Begin the nudge this many metres BEFORE the tightest band's centre, so by the
# time the bend is on us we're already wide. Lead scales with speed below.
PREVIEW_LEAD_BASE_M = 12.0
# How far ahead (seconds) we start anticipating. At 90 km/h this is ~50 m.
PREVIEW_LEAD_TIME_S = 2.0
# Smooth blend between the current bend and the upcoming one: when a sharper
# bend is approaching within the lead time we ramp toward its nudge early.
PREVIEW_BLEND_WINDOW_S = 1.5


class Plugin(BasePlugin):
    """Drive policy — the single strategic brain for HOW we drive.

    Other plugins are reactive (each brakes for its own trigger, lanecontrol
    nudges the lane). This plugin is the one place that looks at everything at
    once and publishes two coherent, combined plans the autopilot just follows:

      * ``planned_speed_ms`` — the lowest safe speed implied by the road right
        now (curvature, road class, posted limit, lead vehicle, red light).
        ACC clamps its target to this.
      * ``drive_lane_offset`` — where laterally we should sit (m, +right of the
        road centreline). Starts at the right-lane baseline, then adds a
        **dynamically computed** trailer-aware nudge: in a bend it swings the
        tractor outward just enough that the trailer's projected tail stays
        inside our lane, scaled DOWN when the bend is gentle / the lane is wide
        / we're slow, and scaled to ZERO when oncoming traffic is too close to
        risk crossing the centreline. The map plugin reads this for steering.

    It never writes controls directly — it only advises. That keeps the
    autopilot the single authority while giving it one coherent plan instead of
    a pile of independent, sometimes-contradictory requests."""

    NAME = "drivepolicy"

    def on_start(self):
        logging.info("DrivePolicy plugin started.")
        self.enabled = True
        # Smoothed plan values so a single noisy sample can't yank the target.
        self._planned = None
        self._lane = BASE_LANE_OFFSET_M

    def on_stop(self):
        self.sdk.set("planned_speed_ms", None)
        self.sdk.set("aux_brake_request", 0.0)
        self.sdk.set("drive_lane_offset", None)

    # ====================================================================
    #  SPEED POLICY
    # ====================================================================
    def _compute_planned_speed(self, dt):
        """Lowest safe speed (m/s) implied by all the limits right now."""
        truck = self.sdk.telemetry.get("truck", {}) or {}
        limits = []

        # 1. Road-class cap (km/h → m/s), published by the map plugin.
        road_cap = self.sdk.get("road_speed_cap")
        if road_cap:
            try:
                limits.append(float(road_cap) / 3.6)
            except (TypeError, ValueError):
                pass

        # 2. Posted in-game speed limit (m/s already, 0 = unknown).
        sl = truck.get("speedLimit", 0.0) or 0.0
        if 0.5 < sl < 200:
            limits.append(float(sl))

        # 3. Curve-safe speed from the measured path curvature.
        radius = self.sdk.get("path_curvature_radius")
        if radius:
            try:
                R = float(radius)
                if 30.0 < R < 2000.0:
                    limits.append(math.sqrt(A_LAT_MAX * R))
            except (TypeError, ValueError):
                pass

        # 4. Lead vehicle: hold a ~3 s time-gap.
        lead = self.sdk.get("lead_distance")
        if lead and lead > 0:
            try:
                limits.append(float(lead) / 3.0)
            except (TypeError, ValueError):
                pass

        # 5. Red/yellow light ahead: ramp to a near-stop as we approach.
        light = self.sdk.get("traffic_light")
        if light:
            color = light.get("color")
            ldist = float(light.get("distance", 999.0) or 999.0)
            if color in ("red", "yellow") and ldist < 70.0:
                limits.append(max(0.0, ldist / 4.0))

        plan = min(limits) if limits else 25.0
        plan = max(0.0, min(plan, 40.0))   # clamp 0..144 km/h

        # Asymmetric lag: drop fast (safe), rise slow (don't lunge into trouble).
        if self._planned is None:
            self._planned = plan
        else:
            rise = min(3.0 * dt, 1.0)
            fall = min(12.0 * dt, 1.0)
            if plan < self._planned:
                self._planned += (plan - self._planned) * fall
            else:
                self._planned += (plan - self._planned) * rise
        return float(self._planned)

    def _compute_aux_brake(self, speed_ms, planned):
        """0..1 aux-brake nudge when we're carrying more speed than the plan."""
        over = speed_ms - planned - AUX_OVERSHOOT_MS
        if over > 0 and planned > 1.0:
            return min(AUX_BRAKE_MAX, over / 8.0 * AUX_BRAKE_MAX)
        return 0.0

    # ====================================================================
    #  LANE + ADAPTIVE TRAILER POLICY
    # ====================================================================
    def _compute_lane_offset(self):
        """Right-of-centre offset (m) with a dynamic trailer-aware nudge.

        The nudge is computed from a **curve preview** of the road ahead: we
        sample the path's lateral position at several distances, build a
        bend-angle at each band, and pick the tightest band we can still reach
        in time. The nudge then ramps in BEFORE that band (lead time), so by the
        time the bend is on us we're already wide — and on gentle bends / empty
        straights the nudge is zero, so we never wander out of our lane for
        nothing."""
        base = BASE_LANE_OFFSET_M
        nudge = self._adaptive_trailer_nudge()
        return float(base + nudge)

    def _adaptive_trailer_nudge(self):
        """DYNAMIC outward nudge (m, + = swing right / out of a right bend).

        Returns 0 when no trailer is coupled, or when there's no bend worth
        correcting. Otherwise it blends the CURRENT bend (where we are now) with
        the UPCOMING bend (the tightest band inside our lead time), so the
        tractor starts moving wide before the corner arrives."""
        # --- Gate: no trailer → no nudge. ---
        if not self.sdk.get("trailer_attached", False):
            return 0.0

        speed_ms = abs(float(self.sdk.telemetry.get("truck", {}).get("speed", 0.0) or 0.0))
        art = float(self.sdk.get("trailer_articulation", 0.0) or 0.0)
        lanes = self.sdk.get("road_lanes") or 1
        try:
            lanes = int(lanes)
        except (TypeError, ValueError):
            lanes = 1

        # --- Curve preview: a bend-angle sample at each distance band. ---
        # Each entry: (distance_m, direction(+1/-1/0), strength). strength is a
        # tangent-of-bend-angle proxy between adjacent bands; bands below
        # MIN_BEND_ANGLE are treated as straight.
        profile = self._curve_profile()
        if not profile:
            return 0.0

        # --- Off-track the trailer's tail currently experiences. ---
        off_track = ASSUMED_TRAILER_LEN_M * (1.0 - math.cos(abs(art)))

        # Room to nudge outward without crossing into the neighbouring lane.
        our_lanes = max(1, lanes // 2)
        room = max(0.0, LANE_HALF_WIDTH_M * our_lanes - TRAILER_EDGE_TOLERANCE_M)
        desired = min(off_track, room)
        if desired <= 0.0:
            return 0.0

        # Speed factor: crawl → no nudge (jerky + unnecessary), cruise → full.
        speed_f = max(0.0, min(1.0, (speed_ms - 5.0) / 10.0)) if speed_ms > 0 else 0.0

        # Oncoming-traffic suppression (don't cross the centreline into a truck).
        oncoming = self._nearest_oncoming_gap()
        if oncoming is not None and oncoming < ONCOMING_SUPPRESS_GAP_M:
            suppress = max(0.0, (oncoming - ONCOMING_SUPPRESS_GAP_M * 0.5)
                           / (ONCOMING_SUPPRESS_GAP_M * 0.5))
        else:
            suppress = 1.0

        # --- Blend the current bend with the upcoming tightest bend. ---
        # `current` = bend at the nearest band (where we are). `upcoming` =
        # tightest band within our lead distance. We ramp toward `upcoming`
        # early so the nudge is in place before the corner.
        lead_dist = PREVIEW_LEAD_BASE_M + speed_ms * PREVIEW_LEAD_TIME_S
        current = self._bend_at(profile, 0.0, lead_dist * 0.4)      # near band
        upcoming = self._tightest_within(profile, lead_dist)

        # Convert each bend sample into a nudge magnitude (0..desired).
        cur_mag = desired * self._bend_factor(current[2]) * speed_f * suppress
        up_mag = desired * self._bend_factor(upcoming[2]) * speed_f * suppress

        # Weighted blend: at speed, weight shifts toward the upcoming bend so we
        # anticipate; at crawl we mostly follow the current one.
        if speed_ms > 1e-3:
            up_weight = max(0.0, min(1.0, (speed_ms - 5.0) / 20.0)) * 0.6
        else:
            up_weight = 0.0
        # If the current and upcoming bends disagree in direction (an S-curve),
        # don't anticipate — swing with the corner we're actually in.
        if current[1] != 0.0 and upcoming[1] != 0.0 and current[1] != upcoming[1]:
            up_weight = 0.0
            up_mag = 0.0

        # Pick the dominant direction; magnitude is the blended value.
        if current[1] != 0.0:
            sign = current[1]
        elif upcoming[1] != 0.0:
            sign = upcoming[1]
        else:
            return 0.0
        magnitude = cur_mag * (1.0 - up_weight) + up_mag * up_weight
        return sign * magnitude

    @staticmethod
    def _bend_factor(strength):
        """Map a bend strength (tangent proxy) to a 0..1 nudge factor.

        Gentle bends (below the curve) get ~0; tight bends saturate to 1. This
        is the 'sometimes you don't need to swing wide at all' gate."""
        if strength <= MIN_BEND_ANGLE:
            return 0.0
        return max(0.0, min(1.0, (strength - MIN_BEND_ANGLE) / 0.4))

    def _curve_profile(self):
        """Sample the path's lateral position at each preview band and return a
        list of ``(distance, direction, strength)`` tuples.

        ``direction`` is +1 (right), -1 (left) or 0 (straight), derived from the
        signed lateral drift between consecutive bands. ``strength`` is the
        absolute drift per metre (a bend-angle tangent proxy). The first band
        carries direction/strength 0 (it's the reference)."""
        pos = self.sdk.get("truck_world_pos")
        heading = self.sdk.get("truck_heading", 0.0) or 0.0
        path = (self.sdk.get("nav_path", []) or self.sdk.get("map_path", []) or [])
        if not pos or len(path) < 2:
            return []

        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)

        # Lateral offset of the path at a given target distance ahead.
        def lateral_at(target):
            best = None
            best_d = 1e18
            for wx, wz in path:
                dx, dz = wx - px, wz - pz
                a = dx * (-sin_h) + dz * (-cos_h)
                if a < 2.0 or a > 120.0:
                    continue
                d = abs(a - target)
                if d < best_d:
                    best_d = d
                    best = (a, dx * cos_h - dz * sin_h)
            return best

        samples = []
        for band in PREVIEW_BANDS_M:
            s = lateral_at(band)
            if s is None:
                # Hole in the path at this band — skip (keeps indices aligned).
                samples.append(None)
            else:
                samples.append(s)

        profile = []
        prev = None
        for band, s in zip(PREVIEW_BANDS_M, samples):
            if s is None:
                profile.append((band, 0.0, 0.0))
                continue
            if prev is None:
                profile.append((band, 0.0, 0.0))
            else:
                drift = s[1] - prev[1]
                gap = abs(s[0] - prev[0]) or 1.0
                strength = abs(drift) / gap
                if strength < 0.06:
                    direction = 0.0
                else:
                    direction = 1.0 if drift > 0 else -1.0
                profile.append((band, direction, strength))
            prev = s
        return profile

    def _bend_at(self, profile, lo_m, hi_m):
        """Bend sample inside ``[lo_m, hi_m]`` (closest band). Fallback straight."""
        for dist, direction, strength in profile:
            if lo_m <= dist <= hi_m:
                return (dist, direction, strength)
        return (0.0, 0.0, 0.0)

    def _tightest_within(self, profile, max_dist):
        """Tightest bend band within ``max_dist`` ahead (max strength). The
        upcoming corner we want to anticipate."""
        best = (0.0, 0.0, 0.0)
        for dist, direction, strength in profile:
            if dist > max_dist:
                continue
            if strength > best[2]:
                best = (dist, direction, strength)
        return best

    def _nearest_oncoming_gap(self):
        """Gap (m) to the closest oncoming vehicle ahead, or None if none.

        Uses the real traffic list; a vehicle is oncoming when it's ahead of us
        but facing the opposite way (yaw differs by ~π)."""
        pos = self.sdk.get("truck_world_pos")
        heading = self.sdk.get("truck_heading", 0.0) or 0.0
        traffic = self.sdk.get("traffic", []) or []
        if not pos or not traffic:
            return None
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)
        best = None
        for v in traffic:
            dx, dz = v["x"] - px, v["z"] - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            if not (5.0 < ahead < 150.0):
                continue
            vyaw = v.get("yaw", heading)
            if math.cos(vyaw - heading) < -0.3:   # facing the opposite way
                if best is None or ahead < best:
                    best = ahead
        return best

    # ====================================================================
    #  TICK
    # ====================================================================
    def on_tick(self, delta_time: float):
        dt = max(delta_time, 1e-3)
        truck = self.sdk.telemetry.get("truck", {}) or {}
        speed_ms = abs(float(truck.get("speed", 0.0) or 0.0))

        planned = self._compute_planned_speed(dt)
        self.sdk.set("planned_speed_ms", planned)
        self.sdk.set("aux_brake_request", self._compute_aux_brake(speed_ms, planned))

        # Smooth the lane offset too so the trailer nudge doesn't twitch.
        target_lane = self._compute_lane_offset()
        # Fast fall-in, slow drift to avoid lane wandering.
        self._lane += (target_lane - self._lane) * min(1.0, 4.0 * dt)
        self.sdk.set("drive_lane_offset", float(self._lane))

        self.tags.planned_speed_kmh = round(planned * 3.6, 1)
        self.tags.drive_lane_offset = round(self._lane, 2)
        self.tags.aux_brake = round(self._compute_aux_brake(speed_ms, planned), 2)
