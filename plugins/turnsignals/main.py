import logging
import math
from sdk.base_plugin import BasePlugin


# --- Tuning -----------------------------------------------------------------
NEAR_M = 15.0           # measure the path's lateral position this close ahead
FAR_M = 60.0            # ...and this far ahead; the bend is the difference
LATERAL_TURN_M = 6.0    # path drifts sideways by this many metres → it's a turn
APPROACH_M = 90.0       # only look at points within this distance ahead
SUSTAIN_S = 3.0         # keep the signal on this long after the bend fades
MIN_SPEED_MS = 1.5      # don't fiddle with blinkers while parked / crawling


class Plugin(BasePlugin):
    """Automatic turn signals.

    Looks at the route ahead (``nav_path`` / ``map_path``) and switches the
    indicator on when a real turn is approaching, then cancels it once we're
    past it. Crucially, it does NOT signal during obstacle avoidance or minor
    steering corrections — those are not turns and would cause the "pruhy sa
    menia pri obchádzaní" chaos the old steering-driven blinkers produced.

    The signal is written through ``ctl_blinker`` (the cross-process channel the
    engine flushes each tick), and ``active_blinker`` is mirrored so the HUD's
    rear-view camera knows when to pop up.
    """

    NAME = "turnsignals"

    def on_start(self):
        logging.info("Turn-signals plugin started.")
        self.enabled = True
        self._current = "off"
        self._sustain = 0.0    # seconds left to hold the signal after the bend

    def on_stop(self):
        self.sdk.set("ctl_blinker", "off")
        self.sdk.set("active_blinker", "off")

    def on_tick(self, delta_time: float):
        dt = max(delta_time, 1e-3)

        # Never override the driver: if the autopilot is off, leave blinkers
        # alone entirely (the player controls them).
        if not self.sdk.shared_state.get("autopilot_active", False):
            if self._current != "off":
                self._set("off")
            return

        speed = float(self.sdk.shared_state.get("truck_speed_ms", 0.0) or 0.0)
        pos = self.sdk.shared_state.get("truck_world_pos")
        heading = self.sdk.shared_state.get("truck_heading", 0.0) or 0.0
        system_state = self.sdk.shared_state.get("system_state", "IDLE")
        # Do NOT signal while avoiding an obstacle — that's a swerve, not a turn,
        # and signalling it is exactly the unwanted "lane change during bypass".
        avoiding = system_state in ("AVOID_OBSTACLE", "EMERGENCY")

        path = (self.sdk.shared_state.get("nav_path", [])
                or self.sdk.shared_state.get("map_path", []) or [])

        target = "off"
        if pos and not avoiding and len(path) >= 3 and abs(speed) >= MIN_SPEED_MS:
            target = self._signal_for_path(pos, heading, path)

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
        # Mirror for the HUD rear-cam (so it pops up on a real signal).
        self.sdk.set("active_blinker", target)

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
    def _signal_for_path(self, pos, heading, path):
        """Return 'left' / 'right' / 'off' based on the bend ahead.

        Measures how far the route drifts sideways between NEAR_M and FAR_M ahead
        of the truck. A big lateral drift in one direction = a turn that way.
        This is far more robust than a single-segment angle on a noisy polyline,
        which rarely produced a usable signal."""
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)

        def lateral_at(target_a):
            """Lateral offset (m, +right) of the path at ~target_a metres ahead."""
            best = None
            best_d = 1e18
            for wx, wz in path:
                dx, dz = wx - px, wz - pz
                a = dx * (-sin_h) + dz * (-cos_h)
                if a < 2.0 or a > APPROACH_M:
                    continue
                d = abs(a - target_a)
                if d < best_d:
                    best_d = d
                    l = dx * cos_h - dz * sin_h
                    best = (a, l)
            return best

        near = lateral_at(NEAR_M)
        far = lateral_at(FAR_M)
        if near is None or far is None:
            return "off"

        # Lateral drift of the path between near and far. The truck-frame
        # lateral sign here is +left/−right for the heading convention used, so
        # a negative drift means the road bends to our right.
        drift = far[1] - near[1]
        if drift <= -LATERAL_TURN_M:
            return "right"
        if drift >= LATERAL_TURN_M:
            return "left"
        return "off"

    # --- Output ---------------------------------------------------------------
    def _set(self, side):
        self._current = side
        self.sdk.controller.set_blinker(side)
        if side != "off":
            logging.info("Turn signal: %s", side)
