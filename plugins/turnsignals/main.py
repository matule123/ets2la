import logging
import math

from sdk.base_plugin import BasePlugin
from core.navigation.navigation_intent import snapshot_matches_navigation_intent
from core.navigation.route import Route


# The indication point is topology-owned (junction entry, lane-change start or
# roundabout exit).  Distance only determines how early that proven instruction
# is announced; it never creates a turn from steering or visual curvature.
MIN_APPROACH_M = 28.0
MAX_APPROACH_M = 80.0
APPROACH_TIME_S = 4.0
EVENT_PASS_MARGIN_M = 8.0


def indication_approach_m(speed_ms):
    """Distance giving roughly four seconds of warning at the current speed."""
    try:
        speed = max(0.0, abs(float(speed_ms)))
    except (TypeError, ValueError, OverflowError):
        speed = 0.0
    return max(MIN_APPROACH_M, min(MAX_APPROACH_M,
                                   speed * APPROACH_TIME_S))


class Plugin(BasePlugin):
    """Topology-driven automatic turn signals.

    One immutable LaneTrajectory revision may nominate one upcoming manoeuvre.
    Once selected, its side is locked until the proven endpoint is passed, the
    navigation authority changes, or ETS2 reports that the driver/game has
    cancelled it.  Steering angle, camera lanes and obstacle avoidance never
    choose an indicator direction.
    """

    NAME = "turnsignals"

    def on_start(self):
        logging.info("Turn-signals plugin started (topology state machine).")
        self.enabled = True
        self._current = "off"
        self._active_event_id = None
        self._active_event = None
        self._route_key = None
        self._route = None
        self._observed_active = False
        self._consumed_event_ids = set()
        self.sdk.set("route_blinker", "off")
        self.sdk.set("turn_signal_event", None)

    def on_stop(self):
        self._clear("plugin stopped")
        self.sdk.set("ctl_blinker", "off")
        self.sdk.set("active_blinker", "off")

    @staticmethod
    def _event_id(event):
        value = event.get("event_id")
        if value:
            return str(value)
        # Compatibility for an older, already-built snapshot.  All fields are
        # from that exact snapshot; there is no geometry-derived fallback.
        return repr((
            event.get("kind"), event.get("direction"),
            event.get("segment_index"), event.get("start_s_m"),
            event.get("end_s_m"), event.get("prefab_token"),
        ))

    @staticmethod
    def _observed_side(state):
        truck = ((state.get("telemetry", {}) or {}).get("truck", {}) or {})
        left = bool(truck.get("blinkerLeft", False))
        right = bool(truck.get("blinkerRight", False))
        if left == right:
            return "off" if not left else "hazard"
        return "left" if left else "right"

    def on_tick(self, delta_time: float):
        del delta_time  # progress and telemetry, not wall-clock timers, own state

        if not self.sdk.shared_state.get("autopilot_active", False):
            self._clear("autopilot inactive")
            self.sdk.set("lane_change_safe", True)
            return

        system_state = self.sdk.shared_state.get("system_state", "IDLE")
        avoiding = system_state in ("AVOID_OBSTACLE", "EMERGENCY")
        snapshot = self.sdk.shared_state.get("lane_trajectory", {}) or {}
        current_revision = self.sdk.shared_state.get(
            "lane_trajectory_revision", -1)
        authoritative = bool(
            snapshot.get("valid", False)
            and snapshot.get("revision") == current_revision
            and snapshot_matches_navigation_intent(
                self.sdk.shared_state, snapshot))
        if not authoritative or avoiding:
            self._clear("navigation authority unavailable" if not authoritative
                        else "safety manoeuvre active")
            self.sdk.set("lane_change_safe", True)
            return

        pos = self.sdk.shared_state.get("truck_world_pos")
        heading = self.sdk.shared_state.get("truck_heading", 0.0) or 0.0
        path = snapshot.get("display_points", ()) or ()
        events = snapshot.get("turn_events", ()) or ()
        if not pos or len(path) < 2:
            self._clear("route pose unavailable")
            self.sdk.set("lane_change_safe", True)
            return

        geometry_key = (
            snapshot.get("navigation_intent_id"), snapshot.get("revision"),
            snapshot.get("route_build_id"),
        )
        if geometry_key != self._route_key:
            self._route_key = geometry_key
            self._route = Route(path, name="turn-signal-authority")
        if self._route is None or len(self._route) < 2:
            self._clear("route geometry unavailable")
            return

        progress = self._route.tracking_progress(pos, heading)
        event_by_id = {
            self._event_id(event): event for event in events
            if isinstance(event, dict)
        }

        # Never fight ETS2's self-cancel or a driver's deliberate cancellation.
        # First observe that the requested side really became active; only a
        # subsequent active->off edge completes the manoeuvre.
        observed = self._observed_side(self.sdk.shared_state)
        if self._active_event_id is not None:
            if observed == self._current:
                self._observed_active = True
            elif self._observed_active and observed == "off":
                self._consumed_event_ids.add(self._active_event_id)
                self._clear("indicator cancelled by game or driver")

        if self._active_event_id is not None:
            active = event_by_id.get(self._active_event_id)
            if active is None:
                self._clear("active manoeuvre absent from current revision")
            else:
                try:
                    cancel_s = float(active["end_s_m"]) + EVENT_PASS_MARGIN_M
                except (KeyError, TypeError, ValueError, OverflowError):
                    self._clear("active manoeuvre is malformed")
                else:
                    if progress > cancel_s:
                        self._consumed_event_ids.add(self._active_event_id)
                        self._clear("manoeuvre endpoint passed")
                    else:
                        self._active_event = active

        if self._active_event_id is None:
            selected = self._next_event(
                events, progress,
                indication_approach_m(self.sdk.shared_state.get(
                    "truck_speed_ms", 0.0)))
            if selected is not None:
                event_id = self._event_id(selected)
                if event_id not in self._consumed_event_ids:
                    self._activate(event_id, selected, progress)

        target = self._current if self._active_event_id is not None else "off"
        self.tags.turn_signal = target
        self.sdk.set("route_blinker", target)
        self.sdk.set("turn_signal_event", self._diagnostic_payload(
            self._active_event, progress) if self._active_event else None)

        if target in ("left", "right"):
            self.sdk.set("lane_change_safe",
                         self._lane_change_safe(pos, heading, target))
        else:
            self.sdk.set("lane_change_safe", True)

    def _next_event(self, events, progress, approach_m):
        candidates = []
        for event in events:
            if not isinstance(event, dict):
                continue
            try:
                direction = str(event["direction"])
                signal_s = float(event.get(
                    "signal_s_m", event["start_s_m"]))
                end_s = float(event["end_s_m"])
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            event_id = self._event_id(event)
            if direction not in ("left", "right"):
                continue
            if event_id in getattr(self, "_consumed_event_ids", set()):
                continue
            distance = signal_s - progress
            if (-EVENT_PASS_MARGIN_M <= end_s - progress
                    and -EVENT_PASS_MARGIN_M <= distance <= approach_m):
                candidates.append((max(distance, 0.0), signal_s,
                                   event_id, event))
        return min(candidates, default=(None, None, None, None))[3]

    def _activate(self, event_id, event, progress):
        side = str(event.get("direction"))
        if side not in ("left", "right"):
            return
        self._active_event_id = event_id
        self._active_event = event
        self._observed_active = False
        self._set(side)
        logging.info(
            "Turn signal accepted: side=%s kind=%s distance=%.1fm "
            "gps_pair=%s..%s prefab=%s event=%s",
            side, event.get("kind", "unknown"),
            float(event.get("signal_s_m", event.get("start_s_m", progress)))
                - progress,
            event.get("gps_pair_start"), event.get("gps_pair_end"),
            event.get("prefab_token"), event_id)

    def _clear(self, reason):
        was_active = self._active_event_id
        if self._current != "off":
            logging.info("Turn signal cancelled: side=%s reason=%s event=%s",
                         self._current, reason, was_active)
        self._active_event_id = None
        self._active_event = None
        self._observed_active = False
        self._set("off")
        self.sdk.set("route_blinker", "off")
        self.sdk.set("turn_signal_event", None)

    @staticmethod
    def _diagnostic_payload(event, progress):
        return {
            "event_id": event.get("event_id"),
            "direction": event.get("direction"),
            "kind": event.get("kind"),
            "distance_m": float(event.get(
                "signal_s_m", event.get("start_s_m", progress))) - progress,
            "end_distance_m": float(event.get("end_s_m", progress)) - progress,
            "gps_pair_start": event.get("gps_pair_start"),
            "gps_pair_end": event.get("gps_pair_end"),
            "prefab_token": event.get("prefab_token"),
            "source_lane_id": event.get("source_lane_id"),
            "target_lane_id": event.get("target_lane_id"),
        }

    def _lane_change_safe(self, pos, heading, side):
        """True if no vehicle occupies the adjacent lane blind spot."""
        traffic = self.sdk.shared_state.get("traffic", []) or []
        if not traffic or not pos:
            return True
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)
        target_lat = (1.0 if side == "right" else -1.0) * 3.5
        for vehicle in traffic:
            try:
                dx, dz = vehicle["x"] - px, vehicle["z"] - pz
            except (KeyError, TypeError):
                continue
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lateral = dx * cos_h - dz * sin_h
            if -5.0 < ahead < 15.0 and abs(lateral - target_lat) < 2.2:
                return False
        return True

    def _set(self, side):
        if side == self._current:
            return
        self._current = side
