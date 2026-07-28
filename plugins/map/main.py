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
    safe_diagnostic_call,
)
from core.navigation.runtime_preflight import CONFIDENCE_THRESHOLD
from core.paths import app_dir

# routes/ lives next to the app (works both from source and when frozen).
ROUTES_DIR = os.path.join(app_dir(), "routes")

# Controller/HUD/AR need a safe rolling horizon, not 50 000 control samples
# for an entire 100 km trip. The complete native GPS UID tuple remains the
# authoritative target signature; only live forward geometry is bounded.
RUNTIME_ROUTE_HORIZON_M = 8_000.0
RUNTIME_ROUTE_MAX_UIDS = 768


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
        self._lane_signature = None
        self._lane_path = None
        self._lane_route = None
        self._lane_match = None
        self._lane_revision = int(self.sdk.get(
            "lane_trajectory_revision", 0) or 0)
        self._navigation_log_seq = int(self.sdk.get(
            "navigation_log_seq", 0) or 0)
        self._lane_failure_signature = None
        self._last_logged_lane_failure = None
        self._lane_retry_at = 0.0
        self._last_failed_route_diagnostic = None
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
        return {
            "active_map_key": self.sdk.get("active_map_key"),
            "active_map_name": self.sdk.get("active_map_name"),
            "game_version": self.sdk.get("installed_game_version"),
            "dataset_fingerprint": self.sdk.get(
                "active_dataset_fingerprint", "unavailable") or "unavailable",
        }

    def _new_route_diagnostics(self, uids, pos, altitude, heading,
                               started_monotonic=None):
        return RouteBuildDiagnostics(
            self._next_route_build_revision(), uids,
            (float(pos[0]), float(altitude), float(pos[1])), float(heading),
            self._route_build_environment(),
            started_monotonic=started_monotonic)

    def _remember_route_diagnostic(self, diagnostic, status):
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

    def _finish_stale_route_build(self, diagnostic, reason,
                                  source_revision=None):
        safe_diagnostic_call(diagnostic, "fail_phase", "stale_revision", reason, {
            "source_revision": source_revision,
            "target_revision": diagnostic.revision,
            "current_revision": int(self.sdk.get(
                "lane_trajectory_revision", -1) or -1),
        })
        self._remember_route_diagnostic(diagnostic, "stale")

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

    def _build_is_current(self, uids, revision, request_id=None):
        return bool(
            self._normalise_gps_uids(
                self.sdk.get("game_route_node_uids", []) or []) == tuple(uids)
            and int(self.sdk.get("lane_trajectory_revision", -1) or -1)
                == int(revision)
            and self.sdk.get("nav_recalc_request") == request_id)

    def _publish_invalid_lane_trajectory(self, reason, uids=(), status=None,
                                         log_failure=True, revision=None,
                                         route_build_id=None,
                                         failure_code=None):
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
        return snapshot

    def _update_lane_trajectory(self, pos, heading):
        """Build and atomically publish the sole GPS lane trajectory snapshot."""
        raw_uids = self.sdk.get("game_route_node_uids", []) or []
        uids = self._normalise_gps_uids(raw_uids)
        signature = uids
        if signature != self._lane_signature:
            self._lane_signature = signature
            self._lane_match = None
            self._lane_failure_signature = None
            self._lane_retry_at = 0.0
            locator = getattr(self.road_net, "_runtime_lane_locator", None)
            if locator is not None:
                locator.previous = None
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
        current = self.sdk.get("lane_trajectory", {}) or {}
        build_revision = int(self.sdk.get(
            "lane_trajectory_revision", -1) or -1)
        build_request = self.sdk.get("nav_recalc_request")
        needs_build = not bool(current.get("valid", False))
        failure_signature = (uids, str(current.get("failure_reason", "")))
        if (needs_build and self._lane_failure_signature == failure_signature
                and time.monotonic() < self._lane_retry_at):
            return None
        # Re-localise on the authoritative lane each tick. A confirmed lane
        # transition triggers a fresh trajectory revision, never a shifted copy.
        altitude = float(self.sdk.get("truck_altitude", 0.0) or 0.0)
        locator = getattr(self.road_net, "_runtime_lane_locator", None)
        if locator is None:
            from core.navigation.lane_model import LaneLocator
            locator = self.road_net._runtime_lane_locator = LaneLocator(self.road_net)
        locator_started = time.monotonic()
        diagnostic_requested = needs_build
        locator_capture = {} if diagnostic_requested else None
        match = locator.locate((pos[0], altitude, pos[1]), heading, build_uids,
                               self._lane_match, diagnostics=locator_capture)
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
        diagnostic = None
        if diagnostic_requested:
            diagnostic = self._new_route_diagnostics(
                uids, pos, altitude, heading, locator_started)
            safe_diagnostic_call(diagnostic, "start_phase", "LaneLocator")
        if match is None:
            if diagnostic is None:
                diagnostic = self._new_route_diagnostics(
                    uids, pos, altitude, heading, locator_started)
                safe_diagnostic_call(diagnostic, "start_phase", "LaneLocator")
                locator_capture = {}
                locator.locate(
                    (pos[0], altitude, pos[1]), heading, build_uids,
                    self._lane_match, diagnostics=locator_capture,
                    diagnostic_mode=True)
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
            self._fail_route_build(diagnostic, technical_reason, uids)
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
        if (self._lane_match is not None
                and match.lane_id != self._lane_match.lane_id):
            needs_build = True
            if diagnostic is None:
                diagnostic = self._new_route_diagnostics(
                    uids, pos, altitude, heading, locator_started)
                safe_diagnostic_call(diagnostic, "start_phase", "LaneLocator")
                locator_capture = {}
                locator.locate(
                    (pos[0], altitude, pos[1]), heading, build_uids,
                    self._lane_match, diagnostics=locator_capture,
                    diagnostic_mode=True)
                safe_diagnostic_call(diagnostic, "observe_locator",
                                     locator_capture, match)
                safe_diagnostic_call(
                    diagnostic, "finish_phase", "LaneLocator", details={
                    "outcome": "lane_transition",
                    "lane_id": self._lane_id_payload(match.lane_id),
                    "confidence": float(match.confidence),
                    "score_components": dict(match.score_components),
                }, duration_ms=(time.monotonic()-locator_started)*1000.0)
        self._lane_match = match
        if not needs_build and self._lane_path is not None:
            # Keep the geometry snapshot immutable. Runtime localization and
            # liveness are published separately under the same revision.
            self.sdk.set("lane_match", {
                "revision": self._lane_revision,
                "active_lane_id": self._lane_id_payload(match.lane_id),
                "point": [float(match.point.x), float(match.point.y),
                          float(match.point.z)],
                "lateral_error_m": float(match.lateral_error_m),
                "heading_error_rad": float(match.heading_error_rad),
                "vertical_error_m": float(match.vertical_error_m),
                "score": float(match.score),
                "confidence": float(match.confidence),
                "score_components": dict(match.score_components),
                "switch_reason": match.switch_reason,
            })
            self.sdk.set("lane_trajectory_heartbeat", time.monotonic())
            return self._lane_path

        # A manager-backed valid snapshot can survive a plugin restart while
        # the in-process Route/LanePath objects do not. Rebuilding that missing
        # runtime object is still one real calculation and therefore needs its
        # own build id and target revision.
        if diagnostic is None:
            diagnostic = self._new_route_diagnostics(
                uids, pos, altitude, heading, locator_started)
            safe_diagnostic_call(diagnostic, "start_phase", "LaneLocator")
            locator_capture = {}
            locator.locate(
                (pos[0], altitude, pos[1]), heading, build_uids,
                self._lane_match, diagnostics=locator_capture,
                diagnostic_mode=True)
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
            lane_path, _ = self.road_net.build_lane_path(
                build_uids, pos, heading, altitude=altitude, start_match=match,
                diagnostics=diagnostic)
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
        if not self._build_is_current(uids, build_revision, build_request):
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
        if not self._build_is_current(uids, build_revision, build_request):
            self._finish_stale_route_build(
                diagnostic, "GPS revision changed after trajectory validation",
                build_revision)
            return None
        revision = diagnostic.revision
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
            "lane_match": {
                "point": [float(match.point.x), float(match.point.y),
                          float(match.point.z)],
                "lateral_error_m": float(match.lateral_error_m),
                "heading_error_rad": float(match.heading_error_rad),
                "vertical_error_m": float(match.vertical_error_m),
                "score": float(match.score),
                "confidence": float(match.confidence),
                "score_components": dict(match.score_components),
                "switch_reason": match.switch_reason,
            },
            "points": control_points, "display_points": display_points,
            "distance_m": float(trajectory.distance_m), "failure_reason": "",
            "source_gps_uids": [int(uid) for uid in uids],
            "covered_gps_uids": [int(uid) for uid in build_uids],
            "route_horizon_complete": len(build_uids) == len(uids),
            "request_id": build_request,
            "route_build_id": diagnostic.build_id,
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
            "navigation_source": "gps_lane",
            "recorded_route_active": False,
            # A new revision must never coexist with steering derived from the
            # previous geometry. The control branch publishes the new command
            # only after constructing Route from these exact points.
            "nav_active": False,
            "nav_steering": 0.0,
        })
        self._lane_revision = max(self._lane_revision, revision)
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
        self._last_logged_lane_failure = None
        self._lane_retry_at = 0.0
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
            self._lane_path = None
            self._lane_route = None
            self._lane_match = None
            self._lane_failure_signature = None
            self._last_logged_lane_failure = None
            self._lane_retry_at = 0.0
            self._roads_pos = None
            # Invalidate control geometry before publishing any map metadata;
            # another process must never observe a switched dataset alongside
            # steering from the previous one.
            self._publish_invalid_lane_trajectory(
                "Map dataset is changing", (),
                "Načítavam zvolenú mapu", log_failure=False)
            self._roads_revision += 1
            self.sdk.shared_state.update_batch({
                "active_map_key": None,
                "active_map_name": None,
                "active_dataset_fingerprint": "unavailable",
                "map_path": [],
                "map_road_segments": [],
                "map_road_segments_revision": self._roads_revision,
                "lane_match": None,
                "nav_command_result": None,
            })
            self.sdk.set("map_status", f"Loading map dataset {arg}...")
            logging.info("Navigation: switching map dataset to %s.", arg)

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
                    net = RoadNetwork()
                    if net.load(data_dir):
                        if generation != self._map_load_generation:
                            logging.info(
                                "Navigation: discarded stale map load for %s.",
                                chosen["key"])
                            return
                        self.road_net = net
                        self.sdk.set("map_status",
                                     f"Map ready ({len(net.segments)} segments). "
                                     "Waiting for the in-game GPS route.")
                        logging.info("Navigation: road network loaded engine-side "
                                     "(%d segments, key=%s).", len(net.segments), chosen["key"])
                    else:
                        # Allow a retry on the next run, not this one.
                        self._net_attempted = False
                        self.sdk.set("navigation_unreliable", True)
                        self.sdk.set("map_status",
                                     "Map data unreadable — will retry.")
                        self._publish_invalid_lane_trajectory(
                            "Map data is unreadable", (),
                            "Map data is unreadable", log_failure=False,
                            failure_code="INTERNAL_ERROR")
                except Exception as e:
                    logging.error("Navigation: engine-side road network load failed: %s", e)
                    self.sdk.set("navigation_unreliable", True)
                    self.sdk.set("map_status", f"Map load error: {e}")
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
            # Backward-compatible safety for an older engine that does not yet
            # publish the explicit GPS-presence flag in the same telemetry-loss
            # batch. Clearing the target first makes every old snapshot stale.
            self.sdk.shared_state.update_batch({
                "game_gps_navigation_active": False,
                "game_route_node_uids": [],
                "game_route_points": [], "game_route_meta": [],
                "navigation_arrival_pending": False,
            })
            current = self.sdk.get("lane_trajectory", {}) or {}
            if (current.get("valid", False)
                    or current.get("failure_reason")
                        != "Telemetria vozidla nie je dostupná"
                    or int(current.get("revision", -1) or -1)
                        != int(self.sdk.get(
                            "lane_trajectory_revision", -2) or -2)):
                self._publish_invalid_lane_trajectory(
                    "Telemetria vozidla nie je dostupná",
                    self.sdk.get("game_route_node_uids", []) or (),
                    "Telemetria vozidla nie je dostupná")
            else:
                self.sdk.shared_state.update_batch({
                    "nav_path": [], "map_path": [],
                    "nav_active": False, "nav_steering": 0.0,
                    "nav_trajectory_revision": -1,
                    "navigation_source": "none",
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
            self._roads_t = 0.0
            try:
                altitude = float(self.sdk.get("truck_altitude", 0.0) or 0.0)
                roads = self.road_net.hud_segments_3d_near(
                    pos, radius=280.0, limit=950, altitude=altitude)
                payload = [[list(a), list(b), kind, lanes, divided, dash_on,
                            pillar, rail_post, half_width, suppress_markings]
                           for a, b, kind, lanes, divided, dash_on, pillar,
                           rail_post, half_width, suppress_markings in roads]
                self._roads_revision += 1
                self.sdk.shared_state.update_batch({
                    "map_road_segments": payload,
                    "map_road_segments_revision": self._roads_revision,
                })
                self._roads_pos = (float(pos[0]), float(pos[1]))
            except Exception as e:
                logging.debug("HUD road geometry error: %s", e)

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
            })
            self.sdk.set("distance_to_dest", self.active_route.distance_to_end(pos, heading))
            # Publish the upcoming path curvature so the autopilot can brake
            # BEFORE a sharp bend (anticipatory) instead of reacting to its own
            # steering mid-corner. Radius in metres; large = straight.
            self.sdk.set("path_curvature_radius",
                         self.active_route.curvature_ahead(pos, heading))
            self.tags.nav_steering = round(steer, 3)

        else:
            # No recorded route: drive by the downloaded MAP. This is automatic
            # map-based driving — no recording needed.
            snapshot = self.sdk.get("lane_trajectory", {}) or {}
            route = self._lane_route
            if (route is not None and len(route) >= 2
                    and bool(snapshot.get("valid", False))
                    and int(snapshot.get("revision", -1)) == self._lane_revision
                    and tuple(snapshot.get("source_gps_uids", ()) or ())
                        == self._normalise_gps_uids(
                            self.sdk.get("game_route_node_uids", []) or [])):
                # The native planned-route buffer is lane-specific geometry.
                # Applying the generic road-centre offset once more moved the
                # target towards the median (and made HUD and steering disagree
                # with the truck's actual lane).
                steer = route.steering(pos, heading, speed,
                                       lane_offset_m=0.0)
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
                    })
                    self.sdk.set("map_status",
                                 f"Truck is {off_dist:.0f}m from the nearest road — "
                                 "map dataset may not match the game. Switch maps on the Map page.")
                    self.tags.nav_steering = 0.0
                else:
                    self.sdk.shared_state.update_batch({
                        "nav_steering": float(steer), "nav_active": True,
                    })
                # Curvature radius (m) of the road ahead — lets the autopilot
                # anticipate bends (brake before, not during).
                self.sdk.set("path_curvature_radius",
                             route.curvature_ahead(pos, heading))
                self.tags.nav_steering = round(steer, 3)
            else:
                self.sdk.set("nav_active", False)
                self.sdk.set("nav_steering", 0.0)
