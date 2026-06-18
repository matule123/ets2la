import logging
import math
import numpy as np
from sdk.base_plugin import BasePlugin


# --- Tuning -----------------------------------------------------------------
LOOKAHEAD_M = 45.0      # how far ahead we look to detect an upcoming turn
TURN_ANGLE_RAD = 0.26   # ~15°: bend sharper than this counts as a turn
CANCEL_ANGLE_RAD = 0.10  # once we're past the turn, cancel below this bend
APPROACH_M = 55.0       # start signalling when the turn is within this distance
SUSTAIN_S = 2.0         # keep the signal on this long after the bend fades
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
            target = self._signal_for_path(pos, heading, path, dt)

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

    # --- Geometry -------------------------------------------------------------
    def _signal_for_path(self, pos, heading, path, dt):
        """Return 'left' / 'right' / 'off' based on the bend ahead."""
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)

        def to_truck(wx, wz):
            dx, dz = wx - px, wz - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lateral = dx * cos_h - dz * sin_h
            return ahead, lateral

        # Walk the path and find the lateral offset of the point ~LOOKAHEAD_M
        # ahead of us — that's where we're heading. Sign tells left/right.
        al = [to_truck(wx, wz) for wx, wz in path]
        # Only consider points in front of us and within the approach window.
        upcoming = [(a, l) for a, l in al if 3.0 < a < APPROACH_M]
        if len(upcoming) < 2:
            return "off"

        # Bend = change in heading between the near segment and the far segment.
        near = upcoming[0]
        far = min(upcoming, key=lambda p: abs(p[0] - LOOKAHEAD_M))
        a0, l0 = near
        a1, l1 = far
        if a1 - a0 < 2.0:
            return "off"
        # Heading of the path segment, relative to the truck's heading (0).
        seg_dir = math.atan2(l1 - l0, a1 - a0)
        if abs(seg_dir) >= TURN_ANGLE_RAD:
            # positive lateral drift → path goes right → right indicator
            return "right" if seg_dir > 0 else "left"
        return "off"

    # --- Output ---------------------------------------------------------------
    def _set(self, side):
        self._current = side
        self.sdk.controller.set_blinker(side)
        if side != "off":
            logging.info("Turn signal: %s", side)
