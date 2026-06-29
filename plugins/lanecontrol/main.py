import logging
import math
from sdk.base_plugin import BasePlugin


# --- Tuning -----------------------------------------------------------------
LANE_WIDTH = 3.5        # metres between lane centres (right lane = +LANE_WIDTH)
OVERTAKE_GAP = 60.0     # if a slower car is within this ahead, consider overtaking
OVERTAKE_DELTA = 3.0    # ...and it's at least this much slower than us (m/s)
MERGE_BACK_GAP = 25.0   # once the overtaken car is this far behind, merge back right
MERGE_RATE = 1.5        # how fast we slide between lanes (m/s of offset change)
RAMP_LOOKAHEAD = 70.0   # how far ahead to look for a merging/exit ramp


class Plugin(BasePlugin):
    """Dynamic lane control.

    Decides which lane the truck should be in and publishes ``lane_offset_m`` so
    the map/autopilot steering aims there instead of a fixed right lane. Handles:
      • merging onto a motorway / exiting — move toward the lane the route joins
      • overtaking a slower vehicle — move left while passing, then back right
      • blind-spot safety — never move into a lane the turn-signal check flagged
        unsafe (``lane_change_safe``); wait instead.

    The base offset is the right lane (+LANE_WIDTH). We compute a desired offset
    each tick and slew toward it at MERGE_RATE so the truck glides over, never
    snapping. The map plugin reads ``lane_offset_m`` directly.
    """

    NAME = "lanecontrol"

    def on_start(self):
        logging.info("Lane-control plugin started.")
        self.enabled = True
        self._offset = LANE_WIDTH   # current desired lane offset (right lane)
        self._overtaking = False    # are we currently passing someone on the left?

    def on_stop(self):
        # Hand control back to the default right-lane offset.
        self.sdk.set("lane_offset_m", LANE_WIDTH)

    def on_tick(self, delta_time: float):
        dt = max(delta_time, 1e-3)
        if not self.sdk.shared_state.get("autopilot_active", False):
            return

        desired = self._desired_offset()
        # Slew toward the desired offset smoothly (no snap lane changes).
        step = MERGE_RATE * dt
        if self._offset < desired:
            self._offset = min(desired, self._offset + step)
        else:
            self._offset = max(desired, self._offset - step)
        self.sdk.set("lane_offset_m", round(self._offset, 2))
        self.tags.lane_offset = round(self._offset, 2)
        self.tags.overtaking = self._overtaking
        # Publish a road-hazard marker when a lane is blocked by a slow/stopped
        # vehicle ahead — the HUD draws cones there and we already steer around
        # it via the overtake logic. Cleared when the road is free.
        self._publish_hazard()

    def _publish_hazard(self):
        """Publish ``road_hazard`` (distance + lane) when a lane is blocked.

        A near-stationary vehicle (<2 m/s) in our lane within ~70 m counts as a
        blocked lane — the HUD marks it with cones and the overtake logic moves
        us out of it. When nothing is blocking, the key is cleared so the cones
        disappear."""
        traffic = self.sdk.shared_state.get("traffic", []) or []
        pos = self.sdk.shared_state.get("truck_world_pos")
        heading = self.sdk.shared_state.get("truck_heading", 0.0) or 0.0
        if not traffic or not pos:
            self.sdk.set("road_hazard", None)
            return
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)
        hazard = None
        for v in traffic:
            dx, dz = v["x"] - px, v["z"] - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lat = dx * cos_h - dz * sin_h
            vspeed = float(v.get("speed", 0.0) or 0.0)
            if 10.0 < ahead < 70.0 and abs(lat) < 3.0 and vspeed < 2.0:
                hazard = {"distance": ahead, "lane_offset": lat}
                break
        self.sdk.set("road_hazard", hazard)

    # --- Decision ------------------------------------------------------------
    def _desired_offset(self):
        """Return the lane offset (m, +right) we want to be at right now."""
        # Default: stay in the right lane.
        target = LANE_WIDTH
        safe = bool(self.sdk.shared_state.get("lane_change_safe", True))
        speed = float(self.sdk.shared_state.get("truck_speed_ms", 0.0) or 0.0)

        # --- Overtake logic: a slower car ahead + left lane free + safe. ---
        slow = self._slow_vehicle_ahead()
        if slow is not None:
            self._overtaking = True
            if safe:
                target = -LANE_WIDTH   # left lane (pass)
            else:
                # Want to pass but it's not safe yet — stay put and wait.
                target = LANE_WIDTH
        elif self._overtaking:
            # We were overtaking; merge back right once the passed car is behind
            # us and clear. If not safe, hold the left lane a moment longer.
            if self._passed_clear() and safe:
                self._overtaking = False
                target = LANE_WIDTH
            else:
                target = -LANE_WIDTH

        # --- Ramp/merge: nudge toward the lane the route is joining. We only
        # do this when NOT overtaking so it never fights a pass manoeuvre. ---
        if not self._overtaking:
            join = self._route_lateral_hint()
            if join is not None:
                # Bias toward the route's lateral direction, clamped to a real lane.
                target = max(-LANE_WIDTH, min(LANE_WIDTH, join))

        return target

    def _slow_vehicle_ahead(self):
        """Return the nearest slower vehicle ahead in our lane, or None."""
        traffic = self.sdk.shared_state.get("traffic", []) or []
        pos = self.sdk.shared_state.get("truck_world_pos")
        heading = self.sdk.shared_state.get("truck_heading", 0.0) or 0.0
        my_speed = float(self.sdk.shared_state.get("truck_speed_ms", 0.0) or 0.0)
        if not traffic or not pos:
            return None
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)
        best = None
        for v in traffic:
            dx, dz = v["x"] - px, v["z"] - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lat = dx * cos_h - dz * sin_h
            # In our lane (within ~2 m) and ahead but within overtaking range.
            if 8.0 < ahead < OVERTAKE_GAP and abs(lat) < 2.2:
                v_speed = float(v.get("speed", 0.0) or 0.0)
                if my_speed - v_speed > OVERTAKE_DELTA:
                    if best is None or ahead < best[0]:
                        best = (ahead, v)
        return best

    def _passed_clear(self):
        """True once the vehicle we overtook is well behind us (merge back)."""
        traffic = self.sdk.shared_state.get("traffic", []) or []
        pos = self.sdk.shared_state.get("truck_world_pos")
        heading = self.sdk.shared_state.get("truck_heading", 0.0) or 0.0
        if not traffic or not pos:
            return True
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)
        for v in traffic:
            dx, dz = v["x"] - px, v["z"] - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lat = dx * cos_h - dz * sin_h
            # The car we passed is now in the right lane (lat ~ +LANE_WIDTH) and
            # behind us. If nothing is close behind in our current (left) lane,
            # it's safe to merge back.
            if -MERGE_BACK_GAP < ahead < -5.0 and abs(lat - LANE_WIDTH) < 2.5:
                return True
        return True  # nothing behind — clear to merge

    def _route_lateral_hint(self):
        """Return a desired lateral offset if the route is merging/exiting soon.

        Looks ~RAMP_LOOKAHEAD ahead in the planned path; if it drifts strongly
        to one side (joining a motorway from a slip road, or taking an exit),
        we bias the lane offset that way so the truck lines up to merge. Returns
        None when the route is just going straight."""
        path = (self.sdk.shared_state.get("nav_path", [])
                or self.sdk.shared_state.get("map_path", []) or [])
        pos = self.sdk.shared_state.get("truck_world_pos")
        heading = self.sdk.shared_state.get("truck_heading", 0.0) or 0.0
        if not pos or len(path) < 3:
            return None
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)
        far_lat = None
        for wx, wz in path:
            dx, dz = wx - px, wz - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            if 20.0 < ahead < RAMP_LOOKAHEAD:
                far_lat = dx * cos_h - dz * sin_h
                break
        if far_lat is None:
            return None
        # Only act on a strong drift (a real merge/exit), not a gentle curve.
        if abs(far_lat) < 6.0:
            return None
        # Drift right → aim for right lane; drift left → left lane. Sign matches
        # our offset convention (+right).
        return LANE_WIDTH if far_lat > 0 else -LANE_WIDTH
