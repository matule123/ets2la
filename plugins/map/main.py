import logging
import os
import math
import time
from sdk.base_plugin import BasePlugin
from core.navigation.route import Route
from core.navigation.lane_trajectory import build_lane_trajectory
from core.navigation.route_diagnostics import (
    RouteBuildDiagnostics, classify_failure, dataset_fingerprint,
    export_anonymized_failure, friendly_failure_message,
    lane_change_payload, safe_diagnostic_call,
)
from core.navigation.runtime_preflight import CONFIDENCE_THRESHOLD
from core.navigation.navigation_intent import (
    CONTINUATION_CLASSES, NavigationBufferClass, NavigationBuildGuard,
    classify_navigation_buffer, ordered_suffix_prefix_overlap,
    snapshot_matches_navigation_intent,
)
from core.paths import app_dir

# routes/ lives next to the app (works both from source and when frozen).
ROUTES_DIR = os.path.join(app_dir(), "routes")

# Controller/HUD/AR need a safe rolling horizon, not 50 000 control samples
# for an entire 100 km trip. The complete native GPS UID tuple remains the
# authoritative target signature; only live forward geometry is bounded.
RUNTIME_ROUTE_HORIZON_M = 8_000.0
RUNTIME_ROUTE_MAX_UIDS = 768
LANE_MATCH_GRACE_SECONDS = 0.30
LANE_MATCH_GRACE_FRAMES = 5
LANE_MATCH_MAX_DISPLACEMENT_M = 35.0


class Plugin(BasePlugin):
    """
    Coordinate-based navigation plugin.

    Replaces the old stub (which checked a never-set ``NAVIGATING`` state and a
    never-set ``truck_pos`` key, so it did nothing).  This version follows a
    polyline of world waypoints recorded straight from telemetry:

      * **Record** — breadcrumb ``truck_world_pos`` into a :class:`Route`.
      * **Replay** — steer along a loaded route via cross-track + heading error,
        publishing ``nav_steering`` / ``nav_active`` for the Autopilot to use.

    Commands arrive through shared state (set by the UI): ``nav_cmd`` (one of
    ``record`` / ``stop_record`` / ``load`` / ``clear`` / ``stop``) with an
    optional ``nav_arg`` (the route name).  The plugin consumes each command and
    clears ``nav_cmd`` back to ``None``.
    """

    NAME = "map"

    def on_start(self):
        logging.info("Map (navigation) plugin started.")
        self.enabled = True
        self.recording = None        # Route being recorded, or None
        self.active_route = None     # Route being followed, or None
        self.road_net = None         # RoadNetwork loaded from a downloaded map
        self._net_attempted = False  # tried to load the road network this run?
        self._net_loading = False    # background load in progress (don't re-enter)
        self._map_load_generation = 0
        self._diag_t = 0.0           # throttle for localization diagnostics
        self._roads_t = 0.0          # throttle nearby-road HUD publishing
        self._roads_pos = None
        self._roads_revision = int(self.sdk.get(
            "map_road_segments_revision", 0) or 0)
        # Perspective-HUD geometry is presentation-only.  Its prefab scan can
        # be expensive and must never own the navigation heartbeat tick.
        self._roads_loading = False
        self._roads_job_id = 0
        self._live_map_t = 0.0
        self._live_map_pos = None
        self._live_map_revision = int(self.sdk.get(
            "live_map_scene_revision", 0) or 0)
        # Wide live-map scene queries are presentation-only and can take
        # several seconds while a new region populates its lane caches.  They
        # must never run on the navigation tick that owns the 500 ms safety
        # heartbeat.
        self._live_map_loading = False
        self._live_map_job_id = 0
        self._lane_signature = None
        self._rolling_route_refresh_needed = False
        self._lane_path = None
        self._lane_route = None
        self._lane_match = None
        self._lane_localization_current = False
        self._lane_loss_started_at = None
        self._lane_loss_frames = 0
        self._lane_authority_identity = None
        self._lane_revision = int(self.sdk.get(
            "lane_trajectory_revision", 0) or 0)
        self._navigation_log_seq = int(self.sdk.get(
            "navigation_log_seq", 0) or 0)
        self._lane_failure_signature = None
        self._last_logged_lane_failure = None
        self._lane_retry_at = 0.0
        self._last_failed_route_diagnostic = None
        self._build_guard = NavigationBuildGuard()
        self._build_tokens = {}
        if not isinstance(self.sdk.get("map_load_progress"), dict):
            self.sdk.set("map_load_progress", {
                "active": False,
                "percent": 0,
                "phase": "Čakám na mapu",
                "message": "Čakám na načítanie mapového datasetu.",
                "map_key": None,
                "generation": self._map_load_generation,
            })
        legacy_gps_evidence = bool(
            self.sdk.get("game_gps_navigation_active", False)
            or self.sdk.get("dest_city")
            or len(self._normalise_gps_uids(
                self.sdk.get("game_route_node_uids", ()) or ())) >= 2)
        if not self.sdk.get("navigation_intent_id") and legacy_gps_evidence:
            # Safe compatibility with an older engine/shared state. The modern
            # engine publishes this before the map worker needs it.
            legacy_intent = (self.sdk.get("nav_recalc_request")
                             or f"legacy-{time.time_ns()}")
            self.sdk.shared_state.update_batch({
                "navigation_intent_id": legacy_intent,
                "nav_recalc_request": legacy_intent,
            })
        # A plugin/process restart is not an explicit replay activation.
        # Manager-backed shared state can outlive this object, so discard only
        # stale recorded-route ownership while leaving a GPS snapshot intact.
        if self.sdk.get("navigation_source") == "recorded_route":
            self.sdk.shared_state.update_batch({
                "navigation_source": "none",
                "recorded_route_active": False,
                "nav_path": [], "nav_active": False,
                "nav_steering": 0.0, "nav_trajectory_revision": -1,
            })
        else:
            self.sdk.set("recorded_route_active", False)
        os.makedirs(ROUTES_DIR, exist_ok=True)
        self._publish_route_list()

    def on_stop(self):
        logging.info("Map (navigation) plugin stopped.")
        self._roads_job_id += 1
        self._roads_loading = False
        self._live_map_job_id += 1
        self._live_map_loading = False
        self._deactivate_recorded_route(clear_outputs=True)

    @staticmethod
    def _lane_id_payload(lane_id):
        if lane_id is None:
            return None
        return {
            "road_uid": int(lane_id.road_uid),
            "direction": int(lane_id.direction),
            "lane_index": int(lane_id.lane_index),
            "prefab_token": lane_id.prefab_token,
            "connector_index": lane_id.connector_index,
            "connector_path": list(lane_id.connector_path),
        }

    def _lane_runtime_metadata(self, lane_id):
        """Return immutable control metadata for the exact localized lane."""
        index = getattr(self.road_net, "_lane_id_index", {}) or {}
        segment = index.get(lane_id)
        if segment is None:
            return {"lane_width_m": None, "elevation_layer": None}
        try:
            width = float(segment.width_m)
            if not math.isfinite(width) or width <= 0.0:
                width = None
        except (TypeError, ValueError, OverflowError):
            width = None
        return {
            "lane_width_m": width,
            "elevation_layer": int(segment.elevation_layer),
        }

    def _lane_match_payload(self, match, revision):
        metadata = self._lane_runtime_metadata(match.lane_id)
        components = dict(match.score_components)
        # Lateral/heading displacement have explicit, lane-width-aware safety
        # gates in the autopilot. ``off_route`` can also be non-zero solely
        # because the game trimmed the passed UID prefix. Neither is evidence
        # that this same-revision, topology-confirmed LaneId became ambiguous.
        # Preserve the original confidence for ranking/diagnostics and publish
        # a separate identity-quality value for runtime authority.
        identity_penalty = sum(max(0.0, float(components.get(name, 0.0)))
                               for name in ("vertical", "derived_width"))
        authority_confidence = max(
            0.0, min(1.0, 1.0 - identity_penalty / 18.0))
        return {
            "revision": int(revision),
            "valid": True,
            "active_lane_id": self._lane_id_payload(match.lane_id),
            "point": [float(match.point.x), float(match.point.y),
                      float(match.point.z)],
            "lateral_error_m": float(match.lateral_error_m),
            "heading_error_rad": float(match.heading_error_rad),
            "vertical_error_m": float(match.vertical_error_m),
            "score": float(match.score),
            "confidence": float(match.confidence),
            "authority_confidence": float(authority_confidence),
            "score_components": components,
            "switch_reason": match.switch_reason,
            **metadata,
        }

    def _lane_corridor_payload(self, lane_path):
        """Ordered lane authority carried by one validated LanePath."""
        result, seen = [], set()
        for segment in getattr(lane_path, "segments", ()) or ():
            if segment.lane_id in seen:
                continue
            seen.add(segment.lane_id)
            result.append({
                "lane_id": self._lane_id_payload(segment.lane_id),
                "direction": int(segment.direction),
                "lane_index": int(segment.lane_index),
                "lane_type": str(segment.lane_type),
                "successor_kind": (str(segment.successors[0].kind)
                                   if segment.successors else None),
                "elevation_layer": int(segment.elevation_layer),
                "lane_width_m": float(segment.width_m),
                "start_uid": int(segment.start_uid),
                "end_uid": int(segment.end_uid),
                "lane_change": lane_change_payload(segment.lane_change),
            })
        return result

    def _turn_events_payload(self, lane_path):
        """Semantic turn events derived from this exact validated LanePath."""
        points = tuple(getattr(lane_path, "points", ()) or ())
        segments = tuple(getattr(lane_path, "segments", ()) or ())
        events = []
        for segment_index, segment in enumerate(segments):
            if (segment.lane_id.prefab_token in (None, "graph")
                    and segment.lane_type not in ("prefab", "roundabout")):
                continue
            owned = [point for point in points
                     if int(point.segment_index) == segment_index]
            if len(owned) < 3:
                continue
            heading_change = sum(
                (second.heading - first.heading + math.pi)
                % (2.0 * math.pi) - math.pi
                for first, second in zip(owned, owned[1:]))
            if abs(heading_change) < math.radians(22.0):
                continue
            events.append({
                "segment_index": int(segment_index),
                "start_s_m": float(owned[0].s),
                "end_s_m": float(owned[-1].s),
                # ETS heading decreases for a right turn.
                "direction": "right" if heading_change < 0.0 else "left",
                "angle_deg": float(math.degrees(heading_change)),
                "kind": str(segment.lane_type),
            })
        return events

    def _next_lane_revision(self):
        shared_revision = int(self.sdk.get(
            "lane_trajectory_revision", 0) or 0)
        self._lane_revision = max(self._lane_revision, shared_revision) + 1
        return self._lane_revision

    def _next_route_build_revision(self):
        """Return the next publishable revision without reserving runtime state."""
        shared_revision = int(self.sdk.get(
            "lane_trajectory_revision", 0) or 0)
        return max(self._lane_revision, shared_revision) + 1

    def _route_build_environment(self):
        try:
            route_distance_m = float(
                self.sdk.get("game_route_distance", 0.0) or 0.0)
        except (TypeError, ValueError, OverflowError):
            route_distance_m = 0.0
        return {
            "active_map_key": self.sdk.get("active_map_key"),
            "active_map_name": self.sdk.get("active_map_name"),
            "game_version": self.sdk.get("installed_game_version"),
            "dataset_fingerprint": self.sdk.get(
                "active_dataset_fingerprint", "unavailable") or "unavailable",
            "game_session_id": self.sdk.get("game_session_id"),
            "navigation_intent_id": self.sdk.get("navigation_intent_id"),
            "navigation_request_id": self.sdk.get("nav_recalc_request"),
            "navigation_buffer_classification": self.sdk.get(
                "navigation_buffer_classification"),
            "route_distance_m": route_distance_m,
            "truck_speed_mps": float(abs(
                self.sdk.get("truck_speed_ms", 0.0) or 0.0)),
            "road_speed_cap_kmh": float(
                self.sdk.get("road_speed_cap", 0.0) or 0.0),
        }

    def _route_build_input_key(self, build_uids, match):
        lane_key = (match.lane_id.sort_key() if match is not None else None)
        return (
            tuple(build_uids), lane_key,
            self.sdk.get("game_session_id"),
            self.sdk.get("active_map_key"),
            self.sdk.get("active_dataset_fingerprint"),
        )

    def _new_route_diagnostics(self, uids, pos, altitude, heading,
                               started_monotonic=None, input_key=None):
        diagnostic = RouteBuildDiagnostics(
            self._next_route_build_revision(), uids,
            (float(pos[0]), float(altitude), float(pos[1])), float(heading),
            self._route_build_environment(),
            started_monotonic=started_monotonic)
        if input_key is not None:
            token = self._build_guard.begin(
                self.sdk.get("navigation_intent_id"), input_key,
                diagnostic.build_id)
            if token is None:
                return None
            self._build_tokens[diagnostic.build_id] = token
            logging.info(
                "Navigation buffer: classification=%s intent=%s build=%s "
                "revision=%s old_count=%d new_count=%d overlap=%d "
                "trimmed=%d extended=%d reason=validated route build scheduled",
                self.sdk.get("navigation_buffer_classification", "UNKNOWN"),
                self.sdk.get("navigation_intent_id"), diagnostic.build_id,
                diagnostic.revision,
                len((self.sdk.get("lane_trajectory", {}) or {}).get(
                    "source_gps_uids", ()) or ()), len(tuple(uids)),
                int(self.sdk.get("navigation_buffer_overlap", 0) or 0),
                int(self.sdk.get("navigation_buffer_trimmed", 0) or 0),
                int(self.sdk.get("navigation_buffer_extended", 0) or 0))
        return diagnostic

    def _remember_route_diagnostic(self, diagnostic, status,
                                   complete_input=True):
        record = (safe_diagnostic_call(diagnostic, "finish", status)
                  or getattr(diagnostic, "record", {}) or {})
        try:
            failure = record.get("failure") or {}
            summary = {
                "route_build_id": record.get("route_build_id"),
                "revision": record.get("revision"),
                "status": record.get("status", status),
                "failure_code": failure.get("code"),
                "failure_phase": failure.get("phase"),
                "message": failure.get("friendly_message"),
                "duration_ms": record.get("duration_ms"),
                "phases": [{
                    "name": phase.get("name"),
                    "status": phase.get("status"),
                    "duration_ms": phase.get("duration_ms"),
                } for phase in record.get("phases", ())],
            }
            self.sdk.set("route_diagnostic_last_result", summary)
            if status != "success":
                self._last_failed_route_diagnostic = record
        except Exception:
            # Diagnostic publication is never part of navigation authority.
            pass
        token = self._build_tokens.pop(
            getattr(diagnostic, "build_id", None), None)
        if complete_input:
            self._build_guard.finish(token)
        else:
            # A stale callback did not evaluate the current GPS input and may
            # not suppress its replacement build on the following tick.
            self._build_guard.abandon(token)
        return record

    def _fail_route_build(self, diagnostic, reason, uids, status=None):
        record = getattr(diagnostic, "record", {}) or {}
        failure = record.get("failure") or {}
        if failure.get("code") is None:
            safe_diagnostic_call(diagnostic, "fail_phase", "route_build", reason)
            failure = (getattr(diagnostic, "record", {}) or {}).get(
                "failure") or {}
        failure_code = failure.get("code") or classify_failure(
            "route_build", reason)
        friendly = (failure.get("friendly_message")
                    or friendly_failure_message(failure_code))
        safe_diagnostic_call(diagnostic, "start_phase", "publish_snapshot", {
            "valid": False, "failure_code": failure_code,
        })
        current = self.sdk.get("lane_trajectory", {}) or {}
        if (self._rolling_route_refresh_needed
                and current.get("valid", False)
                and current.get("navigation_intent_id")
                    == self.sdk.get("navigation_intent_id")):
            # A failed horizon refresh cannot erase the last validated
            # trajectory for the same intent. Control still depends on the
            # live LaneMatch; no stale steering is manufactured here.
            safe_diagnostic_call(diagnostic, "finish_phase", "publish_snapshot",
                                 details={
                "published_revision": current.get("revision"),
                "valid": True, "preserved_previous_snapshot": True,
                "point_count": len(current.get("points", ()) or ()),
            })
            self.sdk.shared_state.update_batch({
                "navigation_recalculating": False,
                "navigation_status": friendly,
                "navigation_failure_reason": str(reason),
            })
            self._publish_preserved_snapshot_liveness(
                current, self._lane_match)
            try:
                logging.error(
                    "Navigation horizon refresh failed; preserving revision %s: %s",
                    current.get("revision"), reason)
            except Exception:
                pass
            self._remember_route_diagnostic(diagnostic, "failed")
            return current
        snapshot = self._publish_invalid_lane_trajectory(
            reason, uids, status or friendly,
            revision=diagnostic.revision,
            route_build_id=diagnostic.build_id,
            failure_code=failure_code)
        safe_diagnostic_call(diagnostic, "finish_phase", "publish_snapshot",
                             details={
            "published_revision": snapshot["revision"], "valid": False,
            "point_count": 0,
        })
        self._remember_route_diagnostic(diagnostic, "failed")
        return snapshot

    def _publish_preserved_snapshot_liveness(self, snapshot, match):
        """Refresh a preserved revision only for its proven live LaneId.

        A rolling-horizon rebuild may fail while the already published prefix
        remains valid.  The old code preserved that snapshot but stopped
        updating its heartbeat on duplicate failed inputs, causing a false
        ``map plugin heartbeat is stale`` shutdown.  Liveness is renewed only
        when the current LaneMatch is part of the immutable corridor; a match
        outside it remains fail-closed with an identity-specific reason.
        """
        try:
            revision = int(snapshot.get("revision", -1) or -1)
            current_revision = int(self.sdk.get(
                "lane_trajectory_revision", -2) or -2)
            lane_payload = self._lane_id_payload(
                match.lane_id if match is not None else None)
            in_corridor = any(
                entry.get("lane_id") == lane_payload
                for entry in (snapshot.get("lane_corridor", ()) or ()))
            accepted = bool(
                snapshot.get("valid", False)
                and revision == current_revision
                and snapshot_matches_navigation_intent(
                    self.sdk.shared_state, snapshot)
                and match is not None
                and in_corridor)
        except (TypeError, ValueError, OverflowError):
            accepted = False
            revision = int(self.sdk.get(
                "lane_trajectory_revision", -1) or -1)

        heartbeat = time.monotonic()
        if accepted:
            self._lane_match = match
            self._lane_localization_current = True
            self.sdk.shared_state.update_batch({
                "lane_match": self._lane_match_payload(match, revision),
                "lane_trajectory_heartbeat": heartbeat,
                "navigation_unreliable": False,
            })
            self._reset_lane_loss()
            return True

        reason = "live lane identity is outside the preserved GPS corridor"
        self._lane_localization_current = False
        self.sdk.shared_state.update_batch({
            "lane_match": {
                "revision": revision,
                "valid": False,
                "lateral_error_m": 0.0,
                "heading_error_rad": 0.0,
                "failure_reason": reason,
            },
            # The plugin is alive; authority is rejected by the invalid live
            # match above, not misreported as a dead heartbeat.
            "lane_trajectory_heartbeat": heartbeat,
            "nav_active": False,
            "nav_steering": 0.0,
            "navigation_unreliable": True,
            "navigation_failure_reason": reason,
        })
        return False

    def _finish_stale_route_build(self, diagnostic, reason,
                                  source_revision=None):
        safe_diagnostic_call(diagnostic, "fail_phase", "stale_revision", reason, {
            "source_revision": source_revision,
            "target_revision": diagnostic.revision,
            "current_revision": int(self.sdk.get(
                "lane_trajectory_revision", -1) or -1),
        })
        self._remember_route_diagnostic(
            diagnostic, "stale", complete_input=False)

    def _handle_diagnostic_export(self):
        request = self.sdk.get("route_diagnostic_export_request")
        if not request:
            return
        record = self._last_failed_route_diagnostic
        requested_id = None if request is True else str(request)
        if record is None or (requested_id and requested_id != str(
                record.get("route_build_id"))):
            result = {
                "ok": False,
                "message": "Požadovaný neúspešný výpočet už nie je dostupný.",
                "route_build_id": requested_id,
                "path": None,
            }
        else:
            try:
                path = export_anonymized_failure(record)
                result = {
                    "ok": True,
                    "message": "Anonymizovaná diagnostika bola uložená.",
                    "route_build_id": record["route_build_id"],
                    "path": path,
                }
                try:
                    logging.info(
                        "route-build diagnostic exported id=%s path=%s",
                        record["route_build_id"], path)
                except Exception:
                    pass
            except Exception as exc:
                try:
                    logging.exception(
                        "route-build diagnostic export failed: %s", exc)
                except Exception:
                    pass
                result = {
                    "ok": False,
                    "message": "Diagnostiku sa nepodarilo uložiť.",
                    "route_build_id": record.get("route_build_id"),
                    "path": None,
                }
        try:
            self.sdk.shared_state.update_batch({
                "route_diagnostic_export_request": None,
                "route_diagnostic_export_result": result,
            })
        except Exception:
            # Export reporting is optional and cannot gate route calculation.
            pass

    @staticmethod
    def _normalise_gps_uids(raw_uids):
        try:
            from core.navigation.road_network import _uid
            return tuple(_uid(uid) for uid in (raw_uids or ()) if _uid(uid))
        except Exception:
            return ()

    def _rebase_rolling_snapshot(self, snapshot, uids, request_id):
        """Rebind unchanged geometry to any proven continuation window."""
        old_uids = self._normalise_gps_uids(
            snapshot.get("source_gps_uids", ()) or ())
        uids = tuple(uids or ())
        intent_id = self.sdk.get("navigation_intent_id")
        classification, overlap, _trimmed, _extended, _reason = (
            classify_navigation_buffer(
                old_uids, uids, destination_present=True,
                old_destination=intent_id, new_destination=intent_id,
                old_context=(snapshot.get("source_game_session_id"),
                             snapshot.get("source_map_key"),
                             snapshot.get("source_dataset_fingerprint")),
                new_context=(self.sdk.get("game_session_id"),
                             self.sdk.get("active_map_key"),
                             self.sdk.get("active_dataset_fingerprint"))))
        if (not snapshot.get("valid", False)
                or classification not in CONTINUATION_CLASSES
                or classification == NavigationBufferClass.TEMPORARILY_UNAVAILABLE
                or snapshot.get("request_id") != request_id
                or snapshot.get("navigation_intent_id", request_id) != intent_id
                or not snapshot.get("route_build_id")
                or snapshot.get("source_game_session_id")
                    != self.sdk.get("game_session_id")
                or snapshot.get("source_map_key")
                    != self.sdk.get("active_map_key")
                or snapshot.get("source_dataset_fingerprint")
                    != self.sdk.get("active_dataset_fingerprint")):
            return None, False
        covered = self._normalise_gps_uids(
            snapshot.get("covered_gps_uids", ()) or ())
        covered_overlap = ordered_suffix_prefix_overlap(covered, uids)
        if covered_overlap < min(2, len(covered), len(uids)):
            # The truck advanced beyond the geometry horizon. Old points no
            # longer prove the current route and must not be retained.
            return None, False
        remaining_covered = covered[-covered_overlap:]
        rebased = dict(snapshot)
        rebased["source_gps_uids"] = [int(uid) for uid in uids]
        rebased["covered_gps_uids"] = [int(uid) for uid in remaining_covered]
        rebased["navigation_intent_id"] = intent_id
        capacity = max(2, int(snapshot.get(
            "covered_gps_uid_capacity", len(covered)) or len(covered)))
        rebased["covered_gps_uid_capacity"] = capacity
        horizon_complete = bool(
            snapshot.get("route_horizon_complete", False)
            and covered_overlap == len(uids))
        rebased["route_horizon_complete"] = horizon_complete
        refresh_needed = bool(
            not horizon_complete
            and len(remaining_covered) <= max(2, capacity // 3))
        return rebased, refresh_needed

    def _horizon_continuity_reason(self, snapshot, previous_path,
                                   new_path, build_uids, match):
        """Empty means a replacement horizon is topologically proven."""
        if not snapshot.get("valid", False):
            return ""
        if snapshot.get("navigation_intent_id") != self.sdk.get(
                "navigation_intent_id"):
            return "navigation intent changed during horizon refresh"
        covered = self._normalise_gps_uids(
            snapshot.get("covered_gps_uids", ()) or ())
        overlap_count = ordered_suffix_prefix_overlap(covered, build_uids)
        if overlap_count < 2:
            return "horizon refresh has no ordered common UID edge"
        shared_sequence = covered[-overlap_count:]
        shared_uid_edges = set(zip(shared_sequence, shared_sequence[1:]))
        old_segments = tuple(getattr(previous_path, "segments", ()) or ())
        new_segments = tuple(getattr(new_path, "segments", ()) or ())
        if not old_segments or not new_segments:
            return "horizon refresh lacks LaneSegment identity"
        old_by_id = {segment.lane_id: segment for segment in old_segments}
        new_by_id = {segment.lane_id: segment for segment in new_segments}
        common = [(old_by_id[segment.lane_id], segment)
                  for segment in new_segments if segment.lane_id in old_by_id]
        if not common:
            # A progressively confirmed adjacent-lane change can legitimately
            # replace every remaining LaneId on a long parallel road. It is
            # proven without a chord only when old and new segments describe
            # the same directed GPS edge, road item and elevation layer.
            new_live = new_by_id.get(
                match.lane_id if match is not None else None)
            lateral_anchor = next((old for old in old_segments
                if (match is not None
                    and match.switch_reason == "lane_change_confirmed"
                    and new_live is not None
                    and old.lane_id.prefab_token is None
                    and new_live.lane_id.prefab_token is None
                    and old.lane_id.road_uid == new_live.lane_id.road_uid
                    and old.direction == new_live.direction
                    and old.elevation_layer == new_live.elevation_layer
                    and abs(old.lane_index - new_live.lane_index) == 1
                    and (old.start_uid, old.end_uid) in shared_uid_edges
                    and (new_live.start_uid, new_live.end_uid)
                        == (old.start_uid, old.end_uid))), None)
            if lateral_anchor is None:
                return "horizon refresh has no common LaneId"
            return ""
        proven_common = [(old, new) for old, new in common
                         if (old.start_uid, old.end_uid) in shared_uid_edges
                         and (new.start_uid, new.end_uid)
                             == (old.start_uid, old.end_uid)]
        if not proven_common:
            return "horizon refresh has no common LaneId on the shared UID edge"
        if match is not None and match.lane_id not in new_by_id:
            return "live LaneId is outside the newly validated horizon"
        for old, new in proven_common:
            if (old.direction != new.direction
                    or old.elevation_layer != new.elevation_layer
                    or old.lane_index != new.lane_index):
                return "common LaneId changed direction, lane or elevation layer"
        return ""

    def _game_gps_navigation_present(self, *, include_snapshot=True):
        """Return whether the game owns navigation, independent of UID health.

        UID count alone is insufficient: the native route buffer can be empty
        or rejected while a destination still exists.  During that interval a
        recorded route must remain disarmed rather than becoming a fallback.
        The redundant compatibility checks also keep fail-closed behaviour
        with older engine state and across inter-process update boundaries.
        """
        if bool(self.sdk.get("game_gps_navigation_active", False)):
            return True
        if bool(self.sdk.get("navigation_arrival_pending", False)):
            return True
        if self.sdk.get("dest_city"):
            return True
        try:
            if float(self.sdk.get("game_route_distance", 0.0) or 0.0) > 0.0:
                return True
        except (TypeError, ValueError, OverflowError):
            return True
        if len(self._normalise_gps_uids(
                self.sdk.get("game_route_node_uids", []) or [])) >= 2:
            return True
        if include_snapshot:
            snapshot = self.sdk.get("lane_trajectory", {}) or {}
            if len(self._normalise_gps_uids(
                    snapshot.get("source_gps_uids", []) or [])) >= 2:
                return True
        return False

    def _deactivate_recorded_route(self, *, clear_outputs=False, reason=None):
        """Disarm replay so it cannot resume after a GPS ownership interval."""
        was_active = getattr(self, "active_route", None) is not None
        self.active_route = None
        payload = {"recorded_route_active": False}
        if self.sdk.get("navigation_source") == "recorded_route":
            payload["navigation_source"] = "none"
        if clear_outputs:
            payload.update({
                "nav_path": [], "nav_active": False,
                "nav_steering": 0.0, "nav_trajectory_revision": -1,
                "path_curvature_radius": None,
                "path_curve_distance_m": None,
                "path_curve_signed_curvature": 0.0,
            })
        shared_state = getattr(self.sdk, "shared_state", None)
        if shared_state is not None and hasattr(shared_state, "update_batch"):
            shared_state.update_batch(payload)
        else:
            for key, value in payload.items():
                self.sdk.set(key, value)
        if was_active and reason:
            logging.info("Navigation: recorded route disarmed: %s", reason)

    def _runtime_gps_window(self, uids):
        """Return an ordered, bounded prefix for live control geometry.

        Missing nodes are retained so the lane builder reports the precise
        topology error instead of silently skipping an authoritative GPS UID.
        """
        uids = tuple(uids or ())
        if len(uids) <= 2 or self.road_net is None:
            return uids
        selected = [uids[0]]
        distance = 0.0
        for start_uid, end_uid in zip(uids, uids[1:]):
            selected.append(end_uid)
            start = self.road_net.nodes.get(start_uid)
            end = self.road_net.nodes.get(end_uid)
            if start is None or end is None:
                break
            distance += math.hypot(end[0] - start[0], end[1] - start[1])
            if (distance >= RUNTIME_ROUTE_HORIZON_M
                    or len(selected) >= RUNTIME_ROUTE_MAX_UIDS):
                break
        return tuple(selected)

    def _build_is_current(self, uids, revision, request_id=None,
                          diagnostic=None):
        token = (self._build_tokens.get(diagnostic.build_id)
                 if diagnostic is not None else None)
        return bool(
            self._normalise_gps_uids(
                self.sdk.get("game_route_node_uids", []) or []) == tuple(uids)
            and int(self.sdk.get("lane_trajectory_revision", -1) or -1)
                == int(revision)
            and self.sdk.get("nav_recalc_request") == request_id
            and (diagnostic is None or self._build_guard.may_publish(token)))

    def _reset_lane_loss(self):
        self._lane_loss_started_at = None
        self._lane_loss_frames = 0

    def _snapshot_identity_mismatch(self, snapshot, uids, request_id):
        """Return why a valid snapshot no longer belongs to this authority."""
        if not snapshot.get("valid", False):
            return ""
        checks = (
            (snapshot.get("request_id") == request_id,
             "GPS calculation request changed"),
            (snapshot.get("navigation_intent_id", request_id)
                == self.sdk.get("navigation_intent_id"),
             "navigation intent changed"),
            (bool(snapshot.get("route_build_id")),
             "route build identity is missing"),
            (snapshot.get("source_game_session_id")
                == self.sdk.get("game_session_id"),
             "game session changed"),
            (snapshot.get("source_map_key")
                == self.sdk.get("active_map_key"),
             "map dataset changed"),
            (snapshot.get("source_dataset_fingerprint")
                == self.sdk.get("active_dataset_fingerprint"),
             "map dataset fingerprint changed"),
        )
        mismatch = next((reason for valid, reason in checks if not valid), "")
        if mismatch:
            return mismatch
        identity = (
            snapshot.get("navigation_intent_id", snapshot.get("request_id")),
            snapshot.get("request_id"), snapshot.get("route_build_id"),
            snapshot.get("source_game_session_id"),
            snapshot.get("source_map_key"),
            snapshot.get("source_dataset_fingerprint"),
        )
        if (self._lane_authority_identity is not None
                and identity != self._lane_authority_identity):
            return "route snapshot identity changed"
        return ""

    def _lane_match_displacement(self, pos, altitude):
        if self._lane_match is None:
            return float("inf")
        point = self._lane_match.point
        return math.dist(
            (float(pos[0]), float(altitude), float(pos[1])),
            (float(point.x), float(point.y), float(point.z)))

    def _publish_invalid_lane_trajectory(self, reason, uids=(), status=None,
                                         log_failure=True, revision=None,
                                         route_build_id=None,
                                         failure_code=None):
        if route_build_id is None:
            # An external authority/session/teleport invalidation supersedes a
            # previously completed input. A subsequent identical UID window
            # must be allowed one fresh validated build. Failed build results
            # pass their build ID and remain deduplicated by the guard.
            self._build_guard.reset_intent(
                self.sdk.get("navigation_intent_id"))
        revision = (self._next_lane_revision() if revision is None
                    else int(revision))
        self._lane_revision = max(self._lane_revision, revision)
        snapshot = {
            "revision": revision, "valid": False, "confidence": 0.0,
            "active_lane_id": None, "lane_match": None,
            "points": [], "display_points": [], "distance_m": 0.0,
            "failure_reason": str(reason or "Navigačná trajektória nie je platná"),
            "source_gps_uids": [int(uid) for uid in uids],
            "request_id": self.sdk.get("nav_recalc_request"),
            "navigation_intent_id": self.sdk.get("navigation_intent_id"),
            "route_build_id": route_build_id,
            "failure_code": failure_code,
        }
        gps_navigation_present = self._game_gps_navigation_present(
            include_snapshot=False)
        navigation_source = (
            "gps_lane" if gps_navigation_present
            else "recorded_route" if getattr(self, "active_route", None) is not None
            else "none")
        self.sdk.shared_state.update_batch({
            "lane_trajectory_revision": revision,
            "lane_trajectory": snapshot,
            "nav_path": [], "map_path": [],
            "nav_active": False, "nav_steering": 0.0,
            "nav_trajectory_revision": -1,
            "navigation_unreliable": True,
            "navigation_failure_reason": snapshot["failure_reason"],
            "navigation_source": navigation_source,
        })
        if status:
            technical = str(status)
            friendly = (friendly_failure_message(failure_code)
                        if failure_code else
                        "Trasu sa nepodarilo bezpečne zostaviť"
                        if any(word in technical.lower() for word in
                               ("geometry gap", "lane transition", "laneconnection",
                                "topology", "corridor edge"))
                        else technical)
            self.sdk.set("navigation_status", friendly)
        technical_reason = snapshot["failure_reason"]
        if log_failure:
            failure_signature = (tuple(int(uid) for uid in uids), technical_reason)
            if getattr(self, "_last_logged_lane_failure", None) != failure_signature:
                self._last_logged_lane_failure = failure_signature
                try:
                    logging.error(
                        "Navigation calculation failed: %s "
                        "(GPS UID count=%d, revision=%d)",
                        technical_reason, len(tuple(uids)), revision)
                except Exception:
                    pass
                self._navigation_log_seq += 1
                self.sdk.shared_state.update_batch({
                    "navigation_log_seq": self._navigation_log_seq,
                    "navigation_log_event": {
                        "seq": self._navigation_log_seq,
                        "level": "ERROR",
                        "message": (friendly_failure_message(failure_code)
                                    if failure_code else
                                    "Výpočet navigácie zlyhal. Podrobnosti sú v logu."),
                    },
                })
        self._lane_path = None
        self._lane_route = None
        self._lane_authority_identity = None
        return snapshot

    def _update_lane_trajectory(self, pos, heading):
        """Build and atomically publish the sole GPS lane trajectory snapshot."""
        raw_uids = self.sdk.get("game_route_node_uids", []) or []
        uids = self._normalise_gps_uids(raw_uids)
        signature = uids
        if signature != self._lane_signature:
            current_snapshot = self.sdk.get("lane_trajectory", {}) or {}
            old_window = self._normalise_gps_uids(
                current_snapshot.get("source_gps_uids", ()) or ())
            observed_class, _observed_overlap, _, _, _ = (
                classify_navigation_buffer(
                    old_window, uids, destination_present=True,
                    old_destination=current_snapshot.get(
                        "navigation_intent_id"),
                    new_destination=self.sdk.get("navigation_intent_id"),
                    old_context=(
                        current_snapshot.get("source_game_session_id"),
                        current_snapshot.get("source_map_key"),
                        current_snapshot.get(
                            "source_dataset_fingerprint")),
                    new_context=(self.sdk.get("game_session_id"),
                                 self.sdk.get("active_map_key"),
                                 self.sdk.get(
                                     "active_dataset_fingerprint"))))
            rebased, refresh_needed = self._rebase_rolling_snapshot(
                current_snapshot, uids, self.sdk.get("nav_recalc_request"))
            if rebased is not None:
                self._lane_signature = signature
                self._rolling_route_refresh_needed = bool(refresh_needed)
                self.sdk.shared_state.update_batch({
                    "lane_trajectory": rebased,
                    "nav_path": list(rebased.get("display_points", ()) or ()),
                    "map_path": list(rebased.get("points", ()) or ()),
                    "nav_trajectory_revision": rebased.get("revision", -1),
                })
                self._lane_authority_identity = (
                    rebased.get("navigation_intent_id"),
                    rebased["request_id"], rebased["route_build_id"],
                    rebased["source_game_session_id"],
                    rebased["source_map_key"],
                    rebased["source_dataset_fingerprint"],
                )
                return self._lane_path
            self._lane_signature = signature
            self._rolling_route_refresh_needed = False
            self._lane_match = None
            self._lane_localization_current = False
            self._reset_lane_loss()
            self._lane_failure_signature = None
            self._lane_retry_at = 0.0
            locator = getattr(self.road_net, "_runtime_lane_locator", None)
            if locator is not None:
                locator.previous = None
            declared_class = self.sdk.get(
                "navigation_buffer_classification")
            incompatible_change = bool(
                observed_class in {
                    NavigationBufferClass.TRUE_REROUTE,
                    NavigationBufferClass.SESSION_OR_DATASET_CHANGED,
                }
                or declared_class in {
                    NavigationBufferClass.TRUE_REROUTE.value,
                    NavigationBufferClass.SESSION_OR_DATASET_CHANGED.value,
                    NavigationBufferClass.DESTINATION_REMOVED.value,
                })
            if current_snapshot.get("valid", False) and not incompatible_change:
                # A compatible window must never erase a usable trajectory.
                # A horizon refresh below can replace it only after validation.
                self._rolling_route_refresh_needed = True
            elif current_snapshot.get("valid", False):
                self._publish_invalid_lane_trajectory(
                    "GPS navigation intent changed", uids,
                    "Načítavam GPS trasu", log_failure=False)
            elif current_snapshot.get("navigation_intent_id") != self.sdk.get(
                    "navigation_intent_id"):
                self._publish_invalid_lane_trajectory(
                    "Načítavam GPS trasu", uids, "Načítavam GPS trasu",
                    log_failure=False)
            self.sdk.set("navigation_recalculating", bool(len(uids) >= 2))
        if len(uids) < 2:
            return None
        if self.road_net is None or not self.road_net.loaded:
            self.sdk.set("navigation_status", "Načítavam GPS trasu")
            return None

        build_uids = self._runtime_gps_window(uids)
        altitude = float(self.sdk.get("truck_altitude", 0.0) or 0.0)
        current = self.sdk.get("lane_trajectory", {}) or {}
        previous_lane_path = self._lane_path
        build_revision = int(self.sdk.get(
            "lane_trajectory_revision", -1) or -1)
        build_request = self.sdk.get("nav_recalc_request")
        identity_mismatch = self._snapshot_identity_mismatch(
            current, uids, build_request)
        if identity_mismatch:
            self._lane_match = None
            self._lane_localization_current = False
            self._reset_lane_loss()
            locator = getattr(self.road_net, "_runtime_lane_locator", None)
            if locator is not None:
                locator.previous = None
            self._publish_invalid_lane_trajectory(
                identity_mismatch, uids, identity_mismatch,
                log_failure=False)
            return None
        if (current.get("valid", False) and self._lane_match is not None
                and self._lane_match_displacement(pos, altitude)
                    > LANE_MATCH_MAX_DISPLACEMENT_M):
            # A normal tick, including the longest measured route build, stays
            # below this spatial bound. Crossing it is a teleport/session jump,
            # not hysteresis: remove authority before acquiring a fresh lane.
            self._lane_match = None
            self._lane_localization_current = False
            self._reset_lane_loss()
            locator = getattr(self.road_net, "_runtime_lane_locator", None)
            if locator is not None:
                locator.previous = None
            self._publish_invalid_lane_trajectory(
                "Vehicle position changed discontinuously", uids,
                "GPS route is being recalculated", log_failure=False)
            return None
        needs_build = bool(
            self._rolling_route_refresh_needed
            or not current.get("valid", False))
        failure_signature = (uids, str(current.get("failure_reason", "")))
        # Re-localise on the authoritative lane each tick. Moving between the
        # already validated LaneSegments updates only live localisation; it is
        # not a reason to rebuild the same trajectory.
        locator = getattr(self.road_net, "_runtime_lane_locator", None)
        if locator is None:
            from core.navigation.lane_model import LaneLocator
            locator = self.road_net._runtime_lane_locator = LaneLocator(self.road_net)
        locator_started = time.monotonic()
        diagnostic_requested = needs_build
        locator_capture = {} if diagnostic_requested else None
        authoritative_segments = (
            tuple(getattr(previous_lane_path, "segments", ()) or ())
            if current.get("valid", False) else ())
        match = locator.locate((pos[0], altitude, pos[1]), heading, build_uids,
                               self._lane_match, diagnostics=locator_capture,
                               authoritative_segments=authoritative_segments)
        if not self._build_is_current(uids, build_revision, build_request):
            if diagnostic_requested:
                diagnostic = self._new_route_diagnostics(
                    uids, pos, altitude, heading, locator_started)
                safe_diagnostic_call(diagnostic, "start_phase", "LaneLocator")
                safe_diagnostic_call(diagnostic, "observe_locator",
                                     locator_capture, match)
                self._finish_stale_route_build(
                    diagnostic, "GPS revision changed during LaneLocator",
                    build_revision)
            return None
        # A confirmed move to an adjacent lane (or a proven topology
        # transition) changes lane-centre geometry. Create a validated
        # same-intent snapshot instead of steering against the old lane or
        # accepting a live LaneId outside its immutable corridor.
        match_lane_payload = self._lane_id_payload(
            match.lane_id if match is not None else None)
        match_in_corridor = any(
            entry.get("lane_id") == match_lane_payload
            for entry in (current.get("lane_corridor", ()) or ()))
        lane_authority_refresh = bool(
            match is not None and current.get("valid", False)
            and not match_in_corridor
            and match.switch_reason in {
                "lane_change_confirmed", "topology_transition"})
        if lane_authority_refresh:
            needs_build = True
            self._rolling_route_refresh_needed = True
            if locator_capture is None:
                locator_capture = {}
                locator.locate(
                    (pos[0], altitude, pos[1]), heading, build_uids,
                    self._lane_match, diagnostics=locator_capture,
                    diagnostic_mode=True,
                    authoritative_segments=authoritative_segments)
            diagnostic_requested = True
        diagnostic = None
        if diagnostic_requested:
            input_key = self._route_build_input_key(build_uids, match)
            if lane_authority_refresh:
                # Returning to a lane used earlier in the same intent is a
                # different replacement of the current corridor, not a
                # duplicate callback from that earlier completed build.
                input_key += (
                    "lane-authority-refresh",
                    current.get("route_build_id"),
                )
            if self._build_guard.input_completed(
                    self.sdk.get("navigation_intent_id"), input_key):
                if current.get("valid", False):
                    self._publish_preserved_snapshot_liveness(current, match)
                return self._lane_path if current.get("valid", False) else None
            diagnostic = self._new_route_diagnostics(
                uids, pos, altitude, heading, locator_started,
                input_key=input_key)
            if diagnostic is None:
                if current.get("valid", False):
                    self._publish_preserved_snapshot_liveness(current, match)
                return self._lane_path if current.get("valid", False) else None
            safe_diagnostic_call(diagnostic, "start_phase", "LaneLocator")
        if match is None:
            self._lane_localization_current = False
            if not needs_build:
                # The GPS target and its validated geometry are unchanged.
                # A single noisy/off-centre localisation sample must stop
                # steering, but must not erase the route from HUD/AR/map or
                # manufacture a new trajectory revision.  The explicit
                # out-of-gate errors keep every control consumer fail-closed
                # until LaneLocator confirms the lane again.
                now = time.monotonic()
                if self._lane_loss_started_at is None:
                    self._lane_loss_started_at = now
                    self._lane_loss_frames = 0
                self._lane_loss_frames += 1
                # Geometry and destination identity remain valid. Loss of the
                # live LaneMatch removes control authority atomically, but it
                # is not permission to rebuild the same LanePath or invent a
                # new intent. Keep previous-match hysteresis for reacquisition.
                self.sdk.shared_state.update_batch({
                    "lane_match": {
                        "revision": self._lane_revision,
                        "valid": False,
                        "lateral_error_m": 0.0,
                        "heading_error_rad": 0.0,
                        "failure_reason": "live lane localization unavailable",
                    },
                    "lane_trajectory_heartbeat": time.monotonic(),
                    "nav_active": False,
                    "nav_steering": 0.0,
                    "navigation_unreliable": True,
                    "navigation_failure_reason": (
                        "Live lane localization is temporarily unavailable"),
                })
                return self._lane_path
            if self._lane_failure_signature == failure_signature:
                # Same intent/window and same failed input: observe localisation
                # on later ticks, but do not schedule another identical build.
                return None
            if diagnostic is None:
                diagnostic = self._new_route_diagnostics(
                    uids, pos, altitude, heading, locator_started)
                safe_diagnostic_call(diagnostic, "start_phase", "LaneLocator")
                locator_capture = {}
                locator.locate(
                    (pos[0], altitude, pos[1]), heading, build_uids,
                    self._lane_match, diagnostics=locator_capture,
                    diagnostic_mode=True,
                    authoritative_segments=authoritative_segments)
            outcome = locator_capture.get("outcome", "no_match")
            safe_diagnostic_call(diagnostic, "observe_locator",
                                 locator_capture, None)
            technical_reason = (
                "LaneLocator found ambiguous candidate lanes"
                if outcome == "ambiguous" else
                "LaneLocator found no candidate satisfying route, distance, "
                "heading, elevation and topology gates")
            safe_diagnostic_call(
                diagnostic, "fail_phase", "LaneLocator", technical_reason,
                locator_capture,
                duration_ms=(time.monotonic() - locator_started) * 1000.0)
            failure_time = time.monotonic()
            snapshot = self._fail_route_build(
                diagnostic, technical_reason, uids)
            self._lane_failure_signature = (
                uids, str(snapshot.get("failure_reason", "")))
            self._lane_retry_at = failure_time + 1.0
            return None
        if diagnostic is not None:
            safe_diagnostic_call(diagnostic, "observe_locator",
                                 locator_capture, match)
            safe_diagnostic_call(diagnostic, "finish_phase", "LaneLocator",
                                 details={
                "outcome": "matched",
                "lane_id": self._lane_id_payload(match.lane_id),
                "confidence": float(match.confidence),
                "score_components": dict(match.score_components),
            })
        self._lane_match = match
        self._lane_localization_current = True
        if not needs_build and self._lane_path is not None:
            # Keep the geometry snapshot immutable. Runtime localization and
            # liveness are published separately under the same revision.
            self.sdk.set("lane_match", self._lane_match_payload(
                match, self._lane_revision))
            self.sdk.shared_state.update_batch({
                "lane_trajectory_heartbeat": time.monotonic(),
                "navigation_unreliable": False,
                "navigation_failure_reason": "",
            })
            self._reset_lane_loss()
            return self._lane_path

        # A manager-backed valid snapshot can survive a plugin restart while
        # the in-process Route/LanePath objects do not. Rebuilding that missing
        # runtime object is still one real calculation and therefore needs its
        # own build id and target revision.
        if diagnostic is None:
            input_key = self._route_build_input_key(build_uids, match)
            if self._build_guard.input_completed(
                    self.sdk.get("navigation_intent_id"), input_key):
                if current.get("valid", False):
                    self._publish_preserved_snapshot_liveness(current, match)
                return self._lane_path if current.get("valid", False) else None
            diagnostic = self._new_route_diagnostics(
                uids, pos, altitude, heading, locator_started,
                input_key=input_key)
            if diagnostic is None:
                if current.get("valid", False):
                    self._publish_preserved_snapshot_liveness(current, match)
                return self._lane_path if current.get("valid", False) else None
            safe_diagnostic_call(diagnostic, "start_phase", "LaneLocator")
            locator_capture = {}
            locator.locate(
                (pos[0], altitude, pos[1]), heading, build_uids,
                self._lane_match, diagnostics=locator_capture,
                diagnostic_mode=True,
                authoritative_segments=authoritative_segments)
            safe_diagnostic_call(diagnostic, "observe_locator",
                                 locator_capture, match)
            safe_diagnostic_call(
                diagnostic, "finish_phase", "LaneLocator", details={
                "outcome": "matched_runtime_rebuild",
                "lane_id": self._lane_id_payload(match.lane_id),
                "confidence": float(match.confidence),
                "score_components": dict(match.score_components),
            }, duration_ms=(time.monotonic()-locator_started)*1000.0)

        self.sdk.set("navigation_status", "Vyberám jazdné pruhy")
        try:
            design_speed_mps = max(
                abs(float(self.sdk.get("truck_speed_ms", 0.0) or 0.0)),
                float(self.sdk.get("road_speed_cap", 0.0) or 0.0) / 3.6,
            )
            lane_path, _ = self.road_net.build_lane_path(
                build_uids, pos, heading, altitude=altitude, start_match=match,
                diagnostics=diagnostic, speed_mps=design_speed_mps)
        except Exception as exc:
            failure_time = time.monotonic()
            technical_reason = (
                f"{type(exc).__name__} while building LanePath: {exc}")
            try:
                logging.exception("route-build failed unexpectedly: %s",
                                  technical_reason)
            except Exception:
                pass
            safe_diagnostic_call(diagnostic, "fail_phase", "route_build",
                                 technical_reason, {
                "failure_code": "INTERNAL_ERROR",
            })
            snapshot = self._fail_route_build(
                diagnostic, technical_reason, uids)
            self._lane_failure_signature = (
                uids, str(snapshot.get("failure_reason", "")))
            self._lane_retry_at = failure_time + 1.0
            return None
        if not self._build_is_current(
                uids, build_revision, build_request, diagnostic):
            self._finish_stale_route_build(
                diagnostic, "GPS revision changed while building LanePath",
                build_revision)
            return None
        if not lane_path.valid:
            failure_time = time.monotonic()
            snapshot = self._fail_route_build(
                diagnostic, lane_path.failure_reason, uids)
            self._lane_failure_signature = (
                uids, str(snapshot.get("failure_reason", "")))
            self._lane_retry_at = failure_time + 1.0
            return None
        self.sdk.set("navigation_status", "Vytváram trajektóriu")
        try:
            trajectory = build_lane_trajectory(
                lane_path, spacing_m=2.0, diagnostics=diagnostic)
        except Exception as exc:
            failure_time = time.monotonic()
            technical_reason = (
                f"{type(exc).__name__} while building trajectory: {exc}")
            try:
                logging.exception("route-build failed unexpectedly: %s",
                                  technical_reason)
            except Exception:
                pass
            safe_diagnostic_call(
                diagnostic, "fail_phase", "build_lane_trajectory",
                technical_reason, {
                "failure_code": "INTERNAL_ERROR",
            })
            snapshot = self._fail_route_build(
                diagnostic, technical_reason, uids)
            self._lane_failure_signature = (
                uids, str(snapshot.get("failure_reason", "")))
            self._lane_retry_at = failure_time + 1.0
            return None
        if not trajectory.valid:
            failure_time = time.monotonic()
            snapshot = self._fail_route_build(
                diagnostic, trajectory.failure_reason, uids)
            self._lane_failure_signature = (
                uids, str(snapshot.get("failure_reason", "")))
            self._lane_retry_at = failure_time + 1.0
            return None
        continuity_reason = self._horizon_continuity_reason(
            current, previous_lane_path, trajectory, build_uids, match)
        if continuity_reason:
            safe_diagnostic_call(
                diagnostic, "fail_phase", "connect_lane_sequence",
                continuity_reason, {"failure_code": "TOPOLOGY_NO_CONNECTION"})
            snapshot = self._fail_route_build(
                diagnostic, continuity_reason, uids)
            self._lane_failure_signature = (
                uids, str(snapshot.get("failure_reason", "")))
            return previous_lane_path
        if not self._build_is_current(
                uids, build_revision, build_request, diagnostic):
            self._finish_stale_route_build(
                diagnostic, "GPS revision changed after trajectory validation",
                build_revision)
            return None
        revision = diagnostic.revision
        live_match_payload = self._lane_match_payload(match, revision)
        lane_corridor = self._lane_corridor_payload(trajectory)
        turn_events = self._turn_events_payload(trajectory)
        control_points = [[float(p.x), float(p.y), float(p.z)]
                          for p in trajectory.points]
        # Phase 4 requires controller, HUD and AR to consume geometrically
        # identical authoritative points. A redrawn 4 m chord can deviate from
        # a curved 2 m polyline, so publish the validated control samples to all
        # three consumers. Display resampling remains an offline/API facility,
        # not a second runtime geometry.
        display_points = [list(point) for point in control_points]
        snapshot = {
            "revision": revision, "valid": True,
            "confidence": float(min(trajectory.confidence, match.confidence)),
            "confidence_components": {
                "locator": float(match.confidence),
                "trajectory": float(trajectory.confidence),
                "locator_score": float(match.score),
                "locator_score_components": dict(match.score_components),
                "threshold": CONFIDENCE_THRESHOLD,
            },
            "active_lane_id": self._lane_id_payload(match.lane_id),
            "lane_corridor": lane_corridor,
            "turn_events": turn_events,
            "lane_match": dict(live_match_payload),
            "points": control_points, "display_points": display_points,
            "distance_m": float(trajectory.distance_m), "failure_reason": "",
            "source_gps_uids": [int(uid) for uid in uids],
            "covered_gps_uids": [int(uid) for uid in build_uids],
            "covered_gps_uid_capacity": len(build_uids),
            "route_horizon_complete": len(build_uids) == len(uids),
            "request_id": build_request,
            "navigation_intent_id": self.sdk.get("navigation_intent_id"),
            "route_build_id": diagnostic.build_id,
            "source_game_session_id": self.sdk.get("game_session_id"),
            "source_map_key": self.sdk.get("active_map_key"),
            "source_dataset_fingerprint": self.sdk.get(
                "active_dataset_fingerprint"),
            "failure_code": None,
        }
        # One shared-state assignment publishes one coherent geometry revision.
        safe_diagnostic_call(diagnostic, "start_phase", "publish_snapshot", {
            "valid": True, "point_count": len(control_points),
        })
        self.sdk.shared_state.update_batch({
            "lane_trajectory_revision": revision,
            "lane_trajectory": snapshot,
            "nav_path": display_points,
            "map_path": control_points,
            "nav_trajectory_revision": revision,
            "lane_trajectory_heartbeat": time.monotonic(),
            "lane_match": live_match_payload,
            "navigation_source": "gps_lane",
            "recorded_route_active": False,
            # A new revision must never coexist with steering derived from the
            # previous geometry. The control branch publishes the new command
            # only after constructing Route from these exact points.
            "nav_active": False,
            "nav_steering": 0.0,
        })
        self._lane_revision = max(self._lane_revision, revision)
        self._lane_authority_identity = (
            snapshot["navigation_intent_id"], snapshot["request_id"],
            snapshot["route_build_id"], snapshot["source_game_session_id"],
            snapshot["source_map_key"],
            snapshot["source_dataset_fingerprint"],
        )
        self.sdk.set("navigation_unreliable", False)
        self.sdk.set("navigation_failure_reason", "")
        self.sdk.set("navigation_recalculating", False)
        self.sdk.set("navigation_status", "Navigácia pripravená")
        self._navigation_log_seq += 1
        self.sdk.shared_state.update_batch({
            "navigation_log_seq": self._navigation_log_seq,
            "navigation_log_event": {
                "seq": self._navigation_log_seq,
                "level": "INFO",
                "message": (
                    f"Navigácia vypočítaná: {len(control_points)} bodov, "
                    f"{trajectory.distance_m:.1f} m, "
                    f"spoľahlivosť {snapshot['confidence']:.3f}."),
            },
        })
        self._lane_path = trajectory
        self._lane_route = Route(control_points, name="gps-lane-trajectory")
        self._lane_failure_signature = None
        self._rolling_route_refresh_needed = False
        self._last_logged_lane_failure = None
        self._lane_retry_at = 0.0
        self._reset_lane_loss()
        safe_diagnostic_call(diagnostic, "finish_phase", "publish_snapshot",
                             details={
            "published_revision": revision, "valid": True,
            "point_count": len(control_points),
            "nav_path_point_count": len(display_points),
        })
        self._remember_route_diagnostic(diagnostic, "success")
        return trajectory

    # --- Helpers --------------------------------------------------------------
    def _publish_route_list(self):
        try:
            names = sorted(f[:-5] for f in os.listdir(ROUTES_DIR) if f.endswith(".json"))
        except Exception:
            names = []
        self.sdk.set("nav_routes", names)

    @staticmethod
    def _driving_line(points, offset):
        """Return the visible lane-centre line used by the controller.

        HUD and AR must show the same right-of-centre target that steering
        follows, not the raw road centre through a median.
        """
        points = [tuple(p[:2]) for p in points]
        if len(points) < 2 or abs(offset) < 0.05:
            return points
        shifted = []
        for index, point in enumerate(points):
            a = points[max(0, index - 1)]
            b = points[min(len(points) - 1, index + 1)]
            dx, dz = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dz)
            if length < 0.1:
                shifted.append(point)
            else:
                shifted.append((point[0] - dz / length * offset,
                                point[1] + dx / length * offset))
        return shifted

    @staticmethod
    def _distance_window(points, metres=220.0):
        """Keep a physical look-ahead distance, independent of point density."""
        points = list(points)
        if len(points) < 2:
            return points
        result, travelled = [points[0]], 0.0
        for point in points[1:]:
            travelled += math.dist(tuple(result[-1][:2]), tuple(point[:2]))
            result.append(point)
            if travelled >= metres:
                break
        return result

    def _handle_command(self, pos):
        cmd = self.sdk.get("nav_cmd")
        if not cmd:
            return
        arg = self.sdk.get("nav_arg") or "route"
        self.sdk.set("nav_cmd", None)

        if cmd == "record":
            self.recording = Route(name=arg)
            if pos:
                self.recording.add_point(pos[0], pos[1])
            logging.info("Navigation: started recording '%s'.", arg)

        elif cmd == "stop_record":
            if self.recording and len(self.recording) >= 2:
                path = os.path.join(ROUTES_DIR, f"{self.recording.name}.json")
                self.recording.save(path)
                logging.info("Navigation: saved route '%s' (%d points).",
                             self.recording.name, len(self.recording))
                self._publish_route_list()
            self.recording = None

        elif cmd == "load":
            if self._game_gps_navigation_present():
                self._deactivate_recorded_route(clear_outputs=False)
                message = "Recorded route cannot start while game GPS is active"
                self.sdk.shared_state.update_batch({
                    "navigation_status": message,
                    "nav_command_result": {
                        "command": "load", "route": arg,
                        "ok": False, "message": message,
                    },
                })
                logging.info(
                    "Navigation: ignored recorded route '%s'; game GPS owns navigation.",
                    arg)
                return
            path = os.path.join(ROUTES_DIR, f"{arg}.json")
            try:
                self.active_route = Route.load(path)
                self.sdk.set("nav_command_result", {
                    "command": "load", "route": arg, "ok": True,
                    "message": f"Recorded route '{arg}' loaded.",
                })
                logging.info("Navigation: loaded route '%s' (%d points).",
                             arg, len(self.active_route))
                self.sdk.set("tts_message", f"Route {arg} loaded. Navigation active.")
            except Exception as e:
                logging.error("Navigation: failed to load '%s': %s", arg, e)
                self._deactivate_recorded_route(clear_outputs=True)
                self.sdk.set("nav_command_result", {
                    "command": "load", "route": arg, "ok": False,
                    "message": f"Recorded route '{arg}' could not be loaded.",
                })

        elif cmd in ("clear", "stop"):
            self._deactivate_recorded_route(clear_outputs=True)
            self.sdk.set("nav_command_result", {
                "command": cmd, "ok": True,
                "message": "Recorded navigation stopped.",
            })
            logging.info("Navigation: stopped.")

        elif cmd == "switch_map":
            self._deactivate_recorded_route(clear_outputs=True)
            self._map_load_generation += 1
            self.road_net = None
            self._net_attempted = False
            self._net_loading = False
            self._lane_signature = None
            self._rolling_route_refresh_needed = False
            self._lane_path = None
            self._lane_route = None
            self._lane_match = None
            self._lane_localization_current = False
            self._reset_lane_loss()
            self._lane_failure_signature = None
            self._last_logged_lane_failure = None
            self._lane_retry_at = 0.0
            self._build_guard = NavigationBuildGuard()
            self._build_tokens = {}
            self._roads_pos = None
            self._roads_job_id += 1
            self._roads_loading = False
            self._live_map_pos = None
            self._live_map_job_id += 1
            self._live_map_loading = False
            # Invalidate control geometry before publishing any map metadata;
            # another process must never observe a switched dataset alongside
            # steering from the previous one.
            self._publish_invalid_lane_trajectory(
                "Map dataset is changing", (),
                "Načítavam zvolenú mapu", log_failure=False)
            self._roads_revision += 1
            self.sdk.shared_state.update_batch({
                "navigation_intent_id": None,
                "nav_recalc_request": None,
                "navigation_buffer_classification":
                    NavigationBufferClass.SESSION_OR_DATASET_CHANGED.value,
                "navigation_buffer_available": False,
                "game_gps_navigation_active": False,
                "game_route_node_uids": [],
                "game_route_points": [],
                "game_route_meta": [],
                "active_map_key": None,
                "active_map_name": None,
                "active_dataset_fingerprint": "unavailable",
                "map_path": [],
                "map_road_segments": [],
                "map_road_segments_revision": self._roads_revision,
                "live_map_road_segments": [],
                "live_map_scene_polygons": [],
                "live_map_scene_features": [],
                "live_map_scene_revision": self._live_map_revision + 1,
                "lane_match": None,
                "nav_command_result": None,
                "map_load_progress": {
                    "active": True,
                    "percent": 0,
                    "phase": "Pripravujem mapový dataset",
                    "message": "Pripravujem mapový dataset — 0 %",
                    "map_key": arg,
                    "generation": self._map_load_generation,
                },
            })
            self._live_map_revision += 1
            self.sdk.set("map_status", f"Loading map dataset {arg}...")
            logging.info("Navigation: switching map dataset to %s.", arg)

    def _schedule_live_map_scene(self, pos, altitude):
        """Build broad presentation geometry without blocking navigation.

        The worker captures both the network and map generation.  A result
        from an old dataset or a superseded job is discarded instead of being
        published into the current live-map revision.
        """
        if getattr(self, "_live_map_loading", False):
            return False
        net = self.road_net
        if net is None or not getattr(net, "loaded", False):
            return False

        import threading
        request_pos = (float(pos[0]), float(pos[1]))
        request_altitude = float(altitude)
        generation = self._map_load_generation
        self._live_map_job_id = getattr(self, "_live_map_job_id", 0) + 1
        job_id = self._live_map_job_id
        self._live_map_loading = True

        def _worker():
            try:
                roads = net.live_map_segments_3d_near(
                    request_pos, radius=1200.0, limit=8500,
                    altitude=request_altitude)
                road_payload = []
                for (a, b, kind, lanes, divided, dash_on, pillar,
                     rail_post, half_width, suppress_markings, path_key,
                     path_index) in roads:
                    road_payload.append([
                        list(a), list(b), kind, lanes, divided, dash_on,
                        pillar, rail_post, half_width, suppress_markings,
                        path_key, path_index,
                        net.live_map_road_type(
                            path_key, lanes=lanes, divided=divided),
                    ])
                polygon_payload = [
                    [[list(point) for point in points], colour, z_index]
                    for points, colour, z_index in
                    net.live_map_polygons_near(
                        request_pos, radius=1200.0, limit=1800)
                ]
                feature_payload = [list(feature) for feature in
                                   net.map_features_near(
                                       request_pos, radius=1200.0, limit=1000)]

                if (job_id != self._live_map_job_id
                        or generation != self._map_load_generation
                        or self.road_net is not net):
                    return
                self._live_map_revision += 1
                self.sdk.shared_state.update_batch({
                    "live_map_road_segments": road_payload,
                    "live_map_scene_polygons": polygon_payload,
                    "live_map_scene_features": feature_payload,
                    "live_map_scene_revision": self._live_map_revision,
                })
                self._live_map_pos = request_pos
            except Exception as e:
                logging.debug("Live-map scene error: %s", e)
            finally:
                if job_id == self._live_map_job_id:
                    self._live_map_loading = False

        threading.Thread(target=_worker, name="LiveMapScene", daemon=True).start()
        return True

    def _schedule_hud_road_scene(self, pos, altitude, anchor_lane_id=None):
        """Build the perspective road/prefab mesh off the navigation tick."""
        if getattr(self, "_roads_loading", False):
            return False
        net = self.road_net
        if net is None or not getattr(net, "loaded", False):
            return False

        import threading
        request_pos = (float(pos[0]), float(pos[1]))
        request_altitude = float(altitude)
        generation = self._map_load_generation
        self._roads_job_id = getattr(self, "_roads_job_id", 0) + 1
        job_id = self._roads_job_id
        self._roads_loading = True

        def _worker():
            try:
                roads = net.hud_segments_3d_near(
                    # The camera clips at 210 m. A much wider 950-chord scene
                    # made dense roundabout prefabs expensive to repaint and
                    # could starve visible HUD updates as a route appeared.
                    request_pos, radius=230.0, limit=600,
                    altitude=request_altitude,
                    anchor_lane_id=anchor_lane_id)
                payload = [[list(a), list(b), kind, lanes, divided,
                            dash_on, pillar, rail_post, half_width,
                            suppress_markings, path_key, path_index]
                           for a, b, kind, lanes, divided, dash_on,
                           pillar, rail_post, half_width, suppress_markings,
                           path_key, path_index in roads]
                if (job_id != self._roads_job_id
                        or generation != self._map_load_generation
                        or self.road_net is not net):
                    return
                self._roads_revision += 1
                self.sdk.shared_state.update_batch({
                    "map_road_segments": payload,
                    "map_road_segments_revision": self._roads_revision,
                })
                self._roads_pos = request_pos
            except Exception as exc:
                logging.debug("HUD road geometry error: %s", exc)
            finally:
                if job_id == self._roads_job_id:
                    self._roads_loading = False

        threading.Thread(target=_worker, name="HudRoadScene",
                         daemon=True).start()
        return True

    def _load_road_net(self):
        """Load the downloaded road network once, in the background (non-blocking).

        The full ETS2 map is ~1.1 M nodes / 250 k segments and takes ~20 s to
        parse, so we must NOT do it on the engine tick thread (that would freeze
        the whole autopilot).  Instead we kick off a worker thread once; while it
        runs the truck keeps its current safe state. The network resolves the
        node UIDs supplied by the in-game GPS; it never invents a route.
        """
        if self.road_net is not None and self.road_net.loaded:
            return
        if self._net_attempted or self._net_loading:
            return
        self._net_attempted = True
        self._net_loading = True
        try:
            import threading
            generation = self._map_load_generation

            def _worker():
                try:
                    from core.navigation import map_data
                    from core.navigation.road_network import RoadNetwork
                    from core.settings.manager import SettingsManager
                    datasets = map_data.list_datasets()
                    downloaded = [d for d in datasets if d["downloaded"]]
                    if not downloaded:
                        self.sdk.set("map_status", "No map downloaded yet.")
                        return
                    # Choose the map: prefer the user's last selection (settings),
                    # otherwise fall back to the first downloaded dataset.
                    sm = SettingsManager()
                    wanted = (sm.get("selected_map") or "").strip()
                    _game_path, installed_version = map_data.installed_ets2()
                    chosen = map_data.choose_downloaded_for_game(
                        datasets, installed_version, wanted)
                    if chosen is None:
                        reason = (f"Selected map {wanted} is not ready. "
                                  f"Create or download a dataset for ETS2 "
                                  f"{installed_version} first.")
                        self.sdk.set("navigation_unreliable", True)
                        self.sdk.set("map_status", reason)
                        self._publish_invalid_lane_trajectory(
                            reason, (), reason, log_failure=False,
                            failure_code="DATASET_VERSION_MISMATCH")
                        logging.error("Navigation: %s", reason)
                        return
                    if chosen["key"] != wanted:
                        logging.info(
                            "Navigation: ETS2 changed to %s; selected exact "
                            "compatible dataset %s instead of %s.",
                            installed_version, chosen["key"], wanted or "none")
                        sm.set("selected_map", chosen["key"])
                        self.sdk.set("selected_map", chosen["key"])
                    if generation != self._map_load_generation:
                        return
                    compatible, installed_version, reason = \
                        map_data.compatible_with_installed_game(chosen["key"])
                    self.sdk.set("installed_game_version", installed_version)
                    if not compatible:
                        self.sdk.set("navigation_unreliable", True)
                        self.sdk.set("map_status", reason)
                        self._publish_invalid_lane_trajectory(
                            reason, (), reason, log_failure=False,
                            failure_code="DATASET_VERSION_MISMATCH")
                        logging.error("Navigation: %s", reason)
                        return
                    self.sdk.set("active_map_key", chosen["key"])
                    self.sdk.set("active_map_name",
                                 chosen.get("name") or chosen["key"])
                    data_dir = map_data.dataset_dir(chosen["key"])
                    self.sdk.set("active_dataset_fingerprint",
                                 dataset_fingerprint(data_dir))
                    self.sdk.set("map_status",
                                 f"Loading road network ({chosen['key']})…")
                    def _load_progress(fraction, phase):
                        if generation != self._map_load_generation:
                            return
                        percent = max(0, min(100, int(round(
                            float(fraction) * 100.0))))
                        message = f"{phase} — {percent} %"
                        self.sdk.shared_state.update_batch({
                            "map_status": message,
                            "map_load_progress": {
                                "active": percent < 100,
                                "percent": percent,
                                "phase": str(phase),
                                "message": message,
                                "map_key": chosen["key"],
                                "generation": generation,
                            },
                        })

                    _load_progress(0.0, "Pripravujem mapový dataset")
                    net = RoadNetwork()
                    if net.load(data_dir, progress_cb=_load_progress):
                        if generation != self._map_load_generation:
                            logging.info(
                                "Navigation: discarded stale map load for %s.",
                                chosen["key"])
                            return
                        self.road_net = net
                        stats = net.load_statistics()
                        ready = ("Mapa je pripravená ("
                                 f"{stats['nodes']:,} uzlov, "
                                 f"{stats['roads']:,} ciest, "
                                 f"{stats['prefabs']:,} prefabov). "
                                 "Čakám na hernú GPS trasu.").replace(",", " ")
                        self.sdk.shared_state.update_batch({
                            "map_status": ready,
                            "map_load_progress": {
                                "active": False,
                                "percent": 100,
                                "phase": "Mapa je pripravená",
                                "message": ready,
                                "map_key": chosen["key"],
                                "generation": generation,
                            },
                        })
                        logging.info("Navigation: road network loaded engine-side "
                                     "(%d segments, key=%s).", len(net.segments), chosen["key"])
                    else:
                        # Allow a retry on the next run, not this one.
                        self._net_attempted = False
                        self.sdk.set("navigation_unreliable", True)
                        self.sdk.shared_state.update_batch({
                            "map_status": "Mapové dáta sa nedajú načítať.",
                            "map_load_progress": {
                                "active": False, "percent": 0,
                                "phase": "Načítanie zlyhalo",
                                "message": "Mapové dáta sa nedajú načítať.",
                                "map_key": chosen["key"],
                                "generation": generation,
                            },
                        })
                        self._publish_invalid_lane_trajectory(
                            "Map data is unreadable", (),
                            "Map data is unreadable", log_failure=False,
                            failure_code="INTERNAL_ERROR")
                except Exception as e:
                    logging.error("Navigation: engine-side road network load failed: %s", e)
                    self.sdk.set("navigation_unreliable", True)
                    self.sdk.set("map_status", f"Map load error: {e}")
                    self.sdk.set("map_load_progress", {
                        "active": False, "percent": 0,
                        "phase": "Načítanie zlyhalo",
                        "message": f"Chyba načítania mapy: {e}",
                        "generation": generation,
                    })
                    self._publish_invalid_lane_trajectory(
                        f"Map load error: {e}", (),
                        f"Map load error: {e}", log_failure=False,
                        failure_code="INTERNAL_ERROR")
                finally:
                    if generation == self._map_load_generation:
                        self._net_loading = False

            threading.Thread(target=_worker, name="RoadNetLoader", daemon=True).start()
        except Exception as e:
            logging.error("Navigation: could not start road network loader: %s", e)
            self._net_loading = False

    def _lane_offset(self):
        """How far (metres) to drive to the RIGHT of the road centreline.

        ETS2 is right-hand traffic, so the autopilot must hold the right lane —
        driving the bare centreline put it in the oncoming lane („protismer").

        The full lateral strategy — right-lane baseline, lane-change requests,
        AND the adaptive trailer-aware swing-wide nudge — is owned by the
        **drivepolicy** plugin, which publishes ``drive_lane_offset``. We prefer
        that when present (it's the coherent combined plan). Fallbacks, in order:
        a manual ``lane_offset_m`` override, then the 2.7 m right-lane default.
        This keeps the map plugin a geometry follower, not a strategist."""
        drv = self.sdk.get("drive_lane_offset", None)
        if drv is not None:
            try:
                # The map graph is commonly the centre line of the whole road.
                # Keep the truck in the right-hand lane, but never accept a
                # large transient lane-change offset while following GPS.
                return max(-2.2, min(2.2, float(drv)))
            except (TypeError, ValueError):
                pass
        v = self.sdk.get("lane_offset_m", None)
        if v is not None:
            try:
                return float(v)
            except (TypeError, ValueError):
                pass
        # 1.8 m is one half-lane: enough to stay out of the centre/median while
        # remaining safe on narrow roads and exact prefab lane curves.
        return 1.8

    def _publish_road_type(self, pos):
        """Classify the road under the truck and publish a speed cap.

        Slows the autopilot on narrow/local/dirt sectors (the „poľné / úzke
        cesty" behaviour) while leaving motorways at full speed. ACC reads
        ``road_speed_cap`` (km/h) and never exceeds it. Cheap no-op when the
        road network isn't loaded yet."""
        net = self.road_net
        if net is None or not getattr(net, "loaded", False) or not pos:
            return
        rt = net.road_type_at(pos)
        if not rt:
            return
        rtype = rt.get("type", "local")
        lanes = rt.get("lanes", 1)
        # Speed caps (km/h) per road class — tuned for a truck. Narrow/dirt
        # sectors cap much lower than the posted limit would, because a truck
        # physically can't take a single-lane dirt road at 90.
        caps = {
            "motorway": 90,
            "expressway": 80,
            "local": 60 if lanes >= 2 else 50,
            "dirt": 35,
        }
        cap = caps.get(rtype, 70)
        prev = self.sdk.get("road_speed_cap", None)
        # Only publish when it changes, to avoid spamming shared state every tick.
        if prev != cap:
            self.sdk.set("road_speed_cap", cap)
            self.sdk.set("road_type", rtype)
            self.sdk.set("road_lanes", lanes)
            logging.info("Road type: %s (%d lanes) -> speed cap %d km/h", rtype, lanes, cap)

    # --- Tick -----------------------------------------------------------------
    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        pos = self.sdk.get("truck_world_pos")
        heading = self.sdk.get("truck_heading", 0.0) or 0.0
        speed = self.sdk.get("truck_speed_ms", 0.0) or 0.0

        self._handle_command(pos)
        try:
            self._handle_diagnostic_export()
        except Exception:
            # A diagnostics/export defect cannot suppress this navigation tick.
            pass

        # GPS and replay are mutually exclusive states.  Disarming (rather
        # than merely suspending) is intentional: after a GPS target is
        # removed an old recorded route may restart only after a new explicit
        # UI ``load`` command.
        gps_navigation_present = self._game_gps_navigation_present()
        if (gps_navigation_present
                and (getattr(self, "active_route", None) is not None
                     or self.sdk.get("navigation_source") == "recorded_route")):
            self._deactivate_recorded_route(
                clear_outputs=(
                    self.sdk.get("navigation_source") == "recorded_route"),
                reason="game GPS became authoritative")

        if self.sdk.get("telemetry_valid", True) is False:
            self._deactivate_recorded_route(clear_outputs=True)
            # A failed SDK read is not evidence that the waypoint was removed.
            # Keep the immutable GPS snapshot/UID window, but revoke live
            # control authority atomically. GameWatcher performs the immediate
            # hard invalidation for a real process/session shutdown.
            self.sdk.shared_state.update_batch({
                "lane_trajectory_heartbeat": 0.0,
                "lane_match": {
                    "revision": self.sdk.get(
                        "lane_trajectory_revision", -1),
                    "valid": False,
                    "failure_reason": "telemetry temporarily unavailable",
                },
                "nav_active": False, "nav_steering": 0.0,
                "navigation_unreliable": True,
                "navigation_failure_reason":
                    "Telemetria vozidla nie je dostupná",
                "recorded_route_active": False,
            })
            return

        if not pos:
            return

        # Lazily load the downloaded road network (engine process) the first
        # time we have a position. Cheap no-op once attempted.
        self._load_road_net()
        self._update_lane_trajectory(pos, heading)

        # Display-only local road geometry. It is deliberately separate from
        # nav_path and therefore cannot influence autopilot steering.
        self._roads_t += delta_time
        moved = (self._roads_pos is None or math.hypot(
            float(pos[0]) - self._roads_pos[0],
            float(pos[1]) - self._roads_pos[1]) >= 12.0)
        if (self._roads_t >= 0.75 and moved
                and self.road_net is not None and self.road_net.loaded):
            altitude = float(self.sdk.get("truck_altitude", 0.0) or 0.0)
            if self._schedule_hud_road_scene(
                    pos, altitude,
                    anchor_lane_id=(self._lane_match.lane_id
                                    if self._lane_localization_current
                                    and self._lane_match is not None else None)):
                self._roads_t = 0.0

        # The top-down map needs a wider scene than the perspective HUD.  Keep
        # this presentation snapshot separate so nearby parallel streets and
        # POIs can be shown without entering HUD, LaneLocator or route inputs.
        self._live_map_t += delta_time
        live_map_moved = (self._live_map_pos is None or math.hypot(
            float(pos[0]) - self._live_map_pos[0],
            float(pos[1]) - self._live_map_pos[1]) >= 18.0)
        if (self._live_map_t >= 1.0 and live_map_moved
                and self.road_net is not None and self.road_net.loaded):
            altitude = float(self.sdk.get("truck_altitude", 0.0) or 0.0)
            if self._schedule_live_map_scene(pos, altitude):
                self._live_map_t = 0.0

        # Localization diagnostics: every ~2 s, log where the truck is and where
        # the map thinks the nearest road is. If the distance is huge (hundreds
        # of metres), the chosen map dataset doesn't match the game/mod and the
        # autopilot will chase a road that's nowhere near us.
        self._diag_t += delta_time
        if self._diag_t >= 2.0 and self.road_net is not None and self.road_net.loaded:
            self._diag_t = 0.0
            try:
                seg_idx = self.road_net._nearest_segment_index(pos)
                if seg_idx is not None:
                    (ax, az), (bx, bz) = self.road_net.segments[seg_idx]
                    sdx, sdz = bx - ax, bz - az
                    L2 = sdx * sdx + sdz * sdz
                    if L2 > 1e-9:
                        t = max(0.0, min(1.0, ((pos[0] - ax) * sdx + (pos[1] - az) * sdz) / L2))
                        qx, qz = ax + t * sdx, az + t * sdz
                    else:
                        qx, qz = ax, az
                    dist = math.hypot(pos[0] - qx, pos[1] - qz)
                    logging.debug(
                        "map: truck=(%.0f, %.0f) nearest_seg=(%.0f, %.0f) dist=%.1fm "
                        "heading=%.3f rad (%.0f°)",
                        pos[0], pos[1], qx, qz, dist, heading, math.degrees(heading))
            except Exception as e:
                logging.debug("map diag error: %s", e)

        # Classify the road we're on + publish a speed cap so the autopilot
        # slows down on narrow/local/dirt sectors and keeps full speed on
        # motorways/expressways. Drives the "nech ide pomalšie na poľných /
        # úzkych cestách" behaviour.
        self._publish_road_type(pos)

        # Recording: drop a breadcrumb every ~10 m.
        if self.recording is not None:
            if self.recording.add_point(pos[0], pos[1]):
                self.tags.nav_recording_points = len(self.recording)

        # A live game-GPS route always wins over legacy recorded-route replay.
        # Otherwise replay could overwrite nav_steering/nav_path while HUD and
        # AR still displayed a valid lane snapshot from a different route.
        gps_navigation_present = self._game_gps_navigation_present()
        if (not gps_navigation_present and self.active_route is not None
                and len(self.active_route) >= 2):
            if self.active_route.is_finished(pos):
                self.sdk.set("tts_message", "Destination reached.")
                logging.info("Navigation: destination reached.")
                self._deactivate_recorded_route(clear_outputs=True)
                return

            steer = self.active_route.steering(pos, heading, speed,
                                               lane_offset_m=self._lane_offset())
            curve_profile = self.active_route.curve_profile_ahead(pos, heading)
            idx = self.active_route.closest_index(pos)
            upcoming = self._distance_window(
                self.active_route.points[idx:], 220.0)
            visible = [list(p) for p in self._driving_line(
                upcoming, self._lane_offset())]
            self.sdk.shared_state.update_batch({
                "nav_steering": float(steer),
                "nav_active": True,
                "nav_path": visible,
                "navigation_source": "recorded_route",
                "recorded_route_active": True,
                "nav_trajectory_revision": -1,
                "navigation_unreliable": False,
                "navigation_failure_reason": "",
                "path_curvature_radius": curve_profile["radius_m"],
                "path_curve_distance_m": curve_profile["distance_m"],
                "path_curve_signed_curvature": curve_profile["signed_curvature"],
            })
            self.sdk.set("distance_to_dest", self.active_route.distance_to_end(pos, heading))
            # Publish the upcoming path curvature so the autopilot can brake
            # BEFORE a sharp bend (anticipatory) instead of reacting to its own
            # steering mid-corner. Radius in metres; large = straight.
            self.tags.nav_steering = round(steer, 3)

        else:
            # No recorded route: drive by the downloaded MAP. This is automatic
            # map-based driving — no recording needed.
            snapshot = self.sdk.get("lane_trajectory", {}) or {}
            route = self._lane_route
            if (route is not None and len(route) >= 2
                    and self._lane_localization_current
                    and bool(snapshot.get("valid", False))
                    and int(snapshot.get("revision", -1)) == self._lane_revision
                    and snapshot_matches_navigation_intent(
                        self.sdk.shared_state, snapshot)):
                # The native planned-route buffer is lane-specific geometry.
                # Applying the generic road-centre offset once more moved the
                # target towards the median (and made HUD and steering disagree
                # with the truck's actual lane).
                # LaneLocator and steering consume the same authoritative lane
                # identity. Its signed error is opposite Route's CTE sign.
                live_match = self.sdk.get("lane_match", {}) or {}
                lane_payload = self._lane_id_payload(
                    self._lane_match.lane_id if self._lane_match else None)
                metadata = self._lane_runtime_metadata(
                    self._lane_match.lane_id if self._lane_match else None)
                corridor_entry = next((entry for entry in
                    (snapshot.get("lane_corridor", ()) or ())
                    if entry.get("lane_id") == lane_payload), None)
                same_lane_authority = bool(
                    self._lane_match is not None
                    and metadata["lane_width_m"] is not None
                    and metadata["elevation_layer"] is not None
                    and live_match.get("valid", False)
                    and int(live_match.get("revision", -1) or -1)
                        == int(snapshot.get("revision", -2) or -2)
                    and live_match.get("active_lane_id") == lane_payload
                    and corridor_entry is not None
                    and live_match.get("elevation_layer")
                        == metadata["elevation_layer"]
                    and corridor_entry.get("elevation_layer")
                        == metadata["elevation_layer"]
                    and live_match.get("lane_width_m")
                        == metadata["lane_width_m"]
                    and corridor_entry.get("lane_width_m")
                        == metadata["lane_width_m"])
                if not same_lane_authority:
                    self.sdk.shared_state.update_batch({
                        "nav_active": False, "nav_steering": 0.0,
                        "path_curvature_radius": None,
                        "path_curve_distance_m": None,
                        "path_curve_signed_curvature": 0.0,
                    })
                    self.tags.nav_steering = 0.0
                    return
                live_cte = -float(live_match["lateral_error_m"])
                if not math.isfinite(live_cte):
                    self.sdk.shared_state.update_batch({
                        "nav_active": False, "nav_steering": 0.0,
                        "path_curvature_radius": None,
                        "path_curve_distance_m": None,
                        "path_curve_signed_curvature": 0.0,
                    })
                    self.tags.nav_steering = 0.0
                    return
                steer = route.steering(pos, heading, speed,
                                       lane_offset_m=0.0,
                                       cross_track_error_m=live_cte)
                curve_profile = route.curve_profile_ahead(pos, heading)
                # Safety: if the truck is far from the snapped path (wrong map
                # dataset, or we're off-road on a ferry / car park), the CTE is
                # huge and Stanley saturates to full-lock. Detect that and
                # disable nav steering instead of yanking the wheel. The
                # autopilot treats the lost lane authority as a safe stop; it
                # never substitutes vision steering inside GPS navigation.
                idx = route.tracking_index(pos, heading)
                nearest = route.points[min(idx, len(route.points)-1)]
                off_dist = math.hypot(pos[0] - nearest[0], pos[1] - nearest[1])
                if off_dist > 50.0:
                    self.sdk.shared_state.update_batch({
                        "nav_active": False, "nav_steering": 0.0,
                        "path_curvature_radius": None,
                        "path_curve_distance_m": None,
                        "path_curve_signed_curvature": 0.0,
                    })
                    self.sdk.set("map_status",
                                 f"Truck is {off_dist:.0f}m from the nearest road — "
                                 "map dataset may not match the game. Switch maps on the Map page.")
                    self.tags.nav_steering = 0.0
                else:
                    steering_debug = dict(getattr(
                        route, "last_steering_debug", {}) or {})
                    self.sdk.shared_state.update_batch({
                        "nav_steering": float(steer), "nav_active": True,
                        "nav_steering_debug": steering_debug,
                        "path_curvature_radius": curve_profile["radius_m"],
                        "path_curve_distance_m": curve_profile["distance_m"],
                        "path_curve_signed_curvature": (
                            curve_profile["signed_curvature"]),
                    })
                # Curvature radius (m) of the road ahead — lets the autopilot
                # anticipate bends (brake before, not during).
                self.tags.nav_steering = round(steer, 3)
            else:
                self.sdk.shared_state.update_batch({
                    "nav_active": False,
                    "nav_steering": 0.0,
                    "path_curvature_radius": None,
                    "path_curve_distance_m": None,
                    "path_curve_signed_curvature": 0.0,
                })
