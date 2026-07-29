import logging
import math
from sdk.base_plugin import BasePlugin
from core.navigation.navigation_intent import snapshot_matches_navigation_intent
from core.navigation.route import Route


# --- Tuning -----------------------------------------------------------------
APPROACH_M = 65.0       # begin signalling before a proven junction event
SUSTAIN_S = 0.8         # bridge only brief progress/telemetry jitter


class Plugin(BasePlugin):
    """Automatic turn signals.

    Consumes topology-proven turn events from the exact lane snapshot and
    cancels the indicator once the truck passes the event. Ordinary road
    curvature, obstacle avoidance and steering corrections are not turns.

    The request is written through ``route_blinker``. The engine remains the
    only owner of the physical momentary control and mirrors its result to HUD.
    """

    NAME = "turnsignals"

    def on_start(self):
        logging.info("Turn-signals plugin started.")
        self.enabled = True
        self._current = "off"
        self._sustain = 0.0    # seconds left to hold the signal after the bend
        self._route_key = None
        self._route = None
        self.sdk.set("route_blinker", "off")

    def on_stop(self):
        self.sdk.set("ctl_blinker", "off")
        self.sdk.set("route_blinker", "off")
        self.sdk.set("active_blinker", "off")

    def on_tick(self, delta_time: float):
        dt = max(delta_time, 1e-3)

        # Never override the driver: if the autopilot is off, leave blinkers
        # alone entirely (the player controls them).
        if not self.sdk.shared_state.get("autopilot_active", False):
            if self._current != "off":
                self._set("off")
            self.sdk.set("route_blinker", "off")
            self.sdk.set("lane_change_safe", True)
            return

        pos = self.sdk.shared_state.get("truck_world_pos")
        heading = self.sdk.shared_state.get("truck_heading", 0.0) or 0.0
        system_state = self.sdk.shared_state.get("system_state", "IDLE")
        # Do NOT signal while avoiding an obstacle — that's a swerve, not a turn,
        # and signalling it is exactly the unwanted "lane change during bypass".
        avoiding = system_state in ("AVOID_OBSTACLE", "EMERGENCY")

        snapshot = self.sdk.shared_state.get("lane_trajectory", {}) or {}
        current_revision = self.sdk.shared_state.get(
            "lane_trajectory_revision", -1)
        authoritative = bool(
            snapshot.get("valid", False)
            and snapshot.get("revision") == current_revision
            and snapshot_matches_navigation_intent(
                self.sdk.shared_state, snapshot))
        path = (snapshot.get("display_points", ()) or ()) if authoritative else ()
        events = (snapshot.get("turn_events", ()) or ()) if authoritative else ()

        target = "off"
        # Keep the route-requested signal active while waiting at a red light.
        if pos and not avoiding and len(path) >= 3 and events:
            target = self._signal_for_events(
                pos, heading, path, events, snapshot)

        # Sustain: keep the signal briefly after the bend so it doesn't strobe
        # on/off as the lookahead wobbles right at the turn threshold.
        if target == "off" and self._current != "off" and self._sustain > 0:
            self._sustain -= dt
            target = self._current
        elif target != "off":
            self._sustain = SUSTAIN_S
        else:
            self._sustain = 0.0

        if target != self._current:
            self._set(target)

        self.tags.turn_signal = target
        # Dedicated route request. The engine arbitrates it with the legacy
        # planner and mirrors the one physical result to active_blinker/HUD.
        self.sdk.set("route_blinker", target)

        # --- Blind-spot check: is it safe to actually move into the signalled
        # lane? When a signal is on we scan the adjacent lane beside+behind us;
        # if a car is there the autopilot must NOT change lanes yet. Off = safe.
        if target in ("left", "right"):
            self.sdk.set("lane_change_safe",
                         self._lane_change_safe(pos, heading, target))
        else:
            self.sdk.set("lane_change_safe", True)

    def _lane_change_safe(self, pos, heading, side):
        """True if no vehicle occupies the target lane in our blind spot.

        Checks the lane we'd move into (~3.5 m to the signalled side) from a few
        metres behind us to ~15 m ahead. Uses the real ETS2LA traffic list; if
        empty, assume safe."""
        traffic = self.sdk.shared_state.get("traffic", []) or []
        if not traffic or not pos:
            return True
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)
        side_sign = 1.0 if side == "right" else -1.0
        target_lat = side_sign * 3.5
        for v in traffic:
            dx, dz = v["x"] - px, v["z"] - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lat = dx * cos_h - dz * sin_h
            if -5.0 < ahead < 15.0 and abs(lat - target_lat) < 2.2:
                return False
        return True

    # --- Geometry -------------------------------------------------------------
    def _signal_for_events(self, pos, heading, path, events, snapshot):
        """Signal only a topology-proven upcoming prefab/roundabout turn."""
        key = (snapshot.get("navigation_intent_id"),
               snapshot.get("revision"), snapshot.get("route_build_id"))
        if key != self._route_key:
            self._route_key = key
            self._route = Route(path, name="turn-signal-authority")
        if self._route is None or len(self._route) < 2:
            return "off"
        progress = self._route.tracking_progress(pos, heading)
        candidates = []
        for event in events:
            try:
                start = float(event["start_s_m"])
                end = float(event["end_s_m"])
                direction = str(event["direction"])
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            if direction not in ("left", "right"):
                continue
            distance = start - progress
            if -8.0 <= end - progress and distance <= APPROACH_M:
                candidates.append((max(distance, 0.0), start, direction))
        return min(candidates)[2] if candidates else "off"

    # --- Output ---------------------------------------------------------------
    def _set(self, side):
        self._current = side
        if side != "off":
            logging.info("Turn signal: %s", side)
