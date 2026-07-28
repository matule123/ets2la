"""Event-scoped diagnostics for the authoritative GPS lane pipeline.

This module is deliberately observational.  It records inputs and outputs of
the existing route builder without changing lane selection, topology,
geometry, validation limits, or publication authority.
"""

from __future__ import annotations

import copy
import hashlib
import json
import logging
import math
import os
import re
import time
import uuid
from dataclasses import asdict
from datetime import datetime, timezone


SCHEMA_VERSION = 1

FAILURE_CODES = (
    "DATASET_MISSING_PREFAB",
    "DATASET_VERSION_MISMATCH",
    "DATASET_MISSING_UID",
    "LOCALIZATION_NO_MATCH",
    "LOCALIZATION_AMBIGUOUS",
    "TOPOLOGY_NO_CONNECTION",
    "TOPOLOGY_AMBIGUOUS",
    "GEOMETRY_GAP",
    "GEOMETRY_HEADING_JUMP",
    "GEOMETRY_ELEVATION_JUMP",
    "TRAJECTORY_VALIDATION_FAILED",
    "STALE_REVISION",
    "INTERNAL_ERROR",
)

FRIENDLY_FAILURE_MESSAGES = {
    "DATASET_MISSING_PREFAB": "Mapové dáta pre túto križovatku nie sú úplné.",
    "DATASET_VERSION_MISMATCH": "Zvolená mapa nezodpovedá verzii hry.",
    "DATASET_MISSING_UID": "GPS trasa nie je v zvolenej mape úplná.",
    "LOCALIZATION_NO_MATCH": "Kamión sa nepodarilo nájsť na GPS pruhu.",
    "LOCALIZATION_AMBIGUOUS": "Polohu kamióna nemožno jednoznačne priradiť k pruhu.",
    "TOPOLOGY_NO_CONNECTION": "Mapové pruhy nemajú potvrdené spojenie.",
    "TOPOLOGY_AMBIGUOUS": "Pokračovanie GPS trasy nie je jednoznačné.",
    "GEOMETRY_GAP": "V geometrii GPS trasy je medzera.",
    "GEOMETRY_HEADING_JUMP": "Smer GPS trasy sa mení neplatným spôsobom.",
    "GEOMETRY_ELEVATION_JUMP": "Výškový priebeh GPS trasy nie je platný.",
    "TRAJECTORY_VALIDATION_FAILED": "GPS trasu sa nepodarilo bezpečne overiť.",
    "STALE_REVISION": "GPS cieľ sa počas výpočtu zmenil.",
    "INTERNAL_ERROR": "Výpočet GPS trasy sa nepodarilo dokončiť.",
}


def _safe_log(level, message, *args):
    """Diagnostics must remain non-fatal even with a broken log handler."""
    try:
        logging.log(level, message, *args)
    except Exception:
        pass


def safe_diagnostic_call(diagnostic, method, *args, **kwargs):
    """Invoke an optional observer without allowing it to affect navigation."""
    if diagnostic is None:
        return None
    try:
        callback = getattr(diagnostic, method, None)
        return callback(*args, **kwargs) if callback is not None else None
    except Exception:
        return None


def classify_failure(phase: str, reason: str, details=None) -> str:
    """Map a technical failure to one stable, user-independent code."""
    phase = str(phase or "").lower()
    reason = str(reason or "").lower()
    details = details or {}
    outcome = str(details.get("outcome", "")).lower()

    if phase == "stale_revision" or "stale revision" in reason:
        return "STALE_REVISION"
    if "version" in phase or ("dataset" in reason and "version" in reason):
        return "DATASET_VERSION_MISMATCH"
    if "absent from the active map" in reason or "gps uid" in reason and any(
            word in reason for word in ("missing", "absent", "not found")):
        return "DATASET_MISSING_UID"
    if "locator" in phase or "localization" in phase:
        if outcome == "ambiguous" or "ambiguous" in reason:
            return "LOCALIZATION_AMBIGUOUS"
        return "LOCALIZATION_NO_MATCH"
    if "ambiguous" in reason:
        return "TOPOLOGY_AMBIGUOUS"
    if any(word in reason for word in (
            "height jump", "elevation jump", "vertical jump")):
        return "GEOMETRY_ELEVATION_JUMP"
    if any(word in reason for word in (
            "heading jump", "direction jump", "reverses direction",
            "points away", "heading differs")):
        return "GEOMETRY_HEADING_JUMP"
    if "gap" in reason or "offset" in reason:
        return "GEOMETRY_GAP"
    if "prefab" in reason and any(word in reason for word in (
            "missing", "absent", "unavailable", "no lane", "no connector",
            "not found", "lacks")):
        return "DATASET_MISSING_PREFAB"
    if any(word in reason for word in (
            "no laneconnection", "missing laneconnection", "no connection",
            "does not connect", "unconfirmed topology", "unconfirmed lane",
            "no directed topological path", "unproven edge", "graph-only edge")):
        return "TOPOLOGY_NO_CONNECTION"
    if phase in ("build_lane_trajectory", "validate_lane_trajectory"):
        return "TRAJECTORY_VALIDATION_FAILED"
    if "corridor" in phase or "lane_sequence" in phase or "lane_path" in phase:
        return "TOPOLOGY_NO_CONNECTION"
    return "INTERNAL_ERROR"


def friendly_failure_message(code: str) -> str:
    return FRIENDLY_FAILURE_MESSAGES.get(
        str(code or ""), FRIENDLY_FAILURE_MESSAGES["INTERNAL_ERROR"])


def lane_id_payload(lane_id):
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


def dataset_fingerprint(data_dir: str) -> str:
    """Return a cheap stable fingerprint without hashing multi-GB datasets."""
    digest = hashlib.sha256()
    found = False
    suffixes = (
        "nodes.json", "roads.json", "prefabs.json", "roadlooks.json",
        "prefabdescriptions.json", "graph.json", "config.json",
    )
    try:
        candidates = []
        for root, _dirs, files in os.walk(data_dir):
            for name in files:
                if name.lower().endswith(suffixes):
                    candidates.append(os.path.join(root, name))
        for path in sorted(candidates, key=lambda item: os.path.relpath(
                item, data_dir).replace("\\", "/").lower()):
            found = True
            relative = os.path.relpath(path, data_dir).replace("\\", "/")
            size = os.path.getsize(path)
            digest.update(relative.lower().encode("utf-8", "replace"))
            digest.update(str(size).encode("ascii"))
            with open(path, "rb") as stream:
                digest.update(stream.read(65536))
                if size > 65536:
                    stream.seek(max(0, size - 65536))
                    digest.update(stream.read(65536))
    except OSError as exc:
        _safe_log(logging.WARNING,
                  "route-build: dataset fingerprint unavailable: %s", exc)
        return "unavailable"
    return digest.hexdigest()[:24] if found else "unavailable"


class RouteBuildDiagnostics:
    """One diagnostic record spanning exactly one route calculation."""

    def __init__(self, revision, gps_uids, world, truck_heading,
                 environment=None, route_build_id=None, started_monotonic=None):
        now = time.monotonic()
        self._started = now if started_monotonic is None else float(started_monotonic)
        self._phase_started = {}
        self._finished = False
        self.record = {
            "schema_version": SCHEMA_VERSION,
            "route_build_id": route_build_id or uuid.uuid4().hex,
            "revision": int(revision),
            "started_at": datetime.now(timezone.utc).isoformat(),
            "duration_ms": None,
            "status": "running",
            "failure": {
                "code": None,
                "phase": None,
                "friendly_message": None,
                "technical_reason": None,
            },
            "context": {
                "gps_uids": [int(uid) for uid in gps_uids],
                "gps_uid": None,
                "gps_uid_index": None,
                "road_token": None,
                "prefab_token": None,
                "lane_id_before": None,
                "lane_id_after": None,
                "world": {
                    "x": float(world[0]), "y": float(world[1]),
                    "z": float(world[2]),
                },
                "truck_heading_rad": float(truck_heading),
                "truck_heading_deg": math.degrees(float(truck_heading)),
                "lane_heading_rad": None,
                "lane_heading_deg": None,
                "elevation_difference_m": None,
                "candidate_lanes": [],
                "planned_lane_connection": {
                    "type": None, "source": None, "target": None,
                },
                "geometry": {
                    "gap_m": None,
                    "heading_jump_deg": None,
                    "elevation_jump_m": None,
                },
                "confidence": {"overall": None, "components": {}},
                "environment": {
                    "active_map_key": None,
                    "active_map_name": None,
                    "game_version": None,
                    "dataset_fingerprint": "unavailable",
                    **dict(environment or {}),
                },
            },
            "phases": [],
        }

    @property
    def build_id(self):
        return self.record["route_build_id"]

    @property
    def revision(self):
        return self.record["revision"]

    def start_phase(self, name, details=None):
        name = str(name)
        self._phase_started[name] = time.monotonic()

    def finish_phase(self, name, status="ok", details=None, duration_ms=None):
        name = str(name)
        started = self._phase_started.pop(name, time.monotonic())
        elapsed = ((time.monotonic() - started) * 1000.0
                   if duration_ms is None else float(duration_ms))
        phase = {
            "name": name, "status": str(status),
            "duration_ms": round(max(0.0, elapsed), 3),
            "details": copy.deepcopy(details or {}),
        }
        self.record["phases"].append(phase)
        return phase

    def fail_phase(self, phase, reason, details=None, duration_ms=None):
        details = copy.deepcopy(details or {})
        code = (details.get("failure_code")
                or classify_failure(phase, reason, details))
        details.setdefault("failure_code", code)
        details.setdefault("technical_reason", str(reason or ""))
        self.finish_phase(phase, "failed", details, duration_ms=duration_ms)
        if self.record["failure"]["code"] is None:
            self.record["failure"] = {
                "code": code,
                "phase": str(phase),
                "friendly_message": friendly_failure_message(code),
                "technical_reason": str(reason or ""),
            }
        self._merge_failure_details(details)
        return code

    def _merge_failure_details(self, details):
        context = self.record["context"]
        for key in ("gps_uid", "gps_uid_index", "road_token", "prefab_token",
                    "lane_id_before", "lane_id_after", "lane_heading_rad",
                    "lane_heading_deg", "elevation_difference_m"):
            if details.get(key) is not None:
                context[key] = copy.deepcopy(details[key])
        geometry = details.get("geometry") or {}
        for key in context["geometry"]:
            if geometry.get(key) is not None:
                context["geometry"][key] = float(geometry[key])
        connection = details.get("planned_lane_connection") or {}
        for key in context["planned_lane_connection"]:
            if connection.get(key) is not None:
                context["planned_lane_connection"][key] = copy.deepcopy(
                    connection[key])

    def observe_locator(self, capture, match=None):
        capture = capture or {}
        context = self.record["context"]
        context["candidate_lanes"] = copy.deepcopy(
            capture.get("candidate_lanes", []))
        candidates = capture.get("candidate_lanes", []) or []
        if candidates:
            best = min(candidates, key=lambda item: (
                item.get("score") is None,
                float(item.get("score") if item.get("score") is not None
                      else item.get("distance_m", float("inf")))))
            context["road_token"] = best.get("road_token")
            context["prefab_token"] = best.get("prefab_token")
            context["lane_id_after"] = copy.deepcopy(best.get("lane_id"))
            context["lane_heading_rad"] = best.get("lane_heading_rad")
            context["lane_heading_deg"] = best.get("lane_heading_deg")
            context["elevation_difference_m"] = best.get(
                "elevation_difference_m")
            if best.get("confidence") is not None:
                context["confidence"] = {
                    "overall": float(best["confidence"]),
                    "components": {
                        "locator_score": float(best["score"]),
                        "locator_score_components": copy.deepcopy(
                            best.get("score_components", {})),
                    },
                }
        if match is not None:
            lane = lane_id_payload(match.lane_id)
            context["lane_id_after"] = lane
            context["lane_heading_rad"] = float(match.point.heading)
            context["lane_heading_deg"] = math.degrees(match.point.heading)
            context["elevation_difference_m"] = float(match.vertical_error_m)
            context["confidence"] = {
                "overall": float(match.confidence),
                "components": {
                    "locator_score": float(match.score),
                    "locator_score_components": dict(match.score_components),
                },
            }

    def observe_lane_path(self, lane_path):
        context = self.record["context"]
        segments = tuple(getattr(lane_path, "segments", ()) or ())
        if segments:
            boundaries = []
            for first, second in zip(segments, segments[1:]):
                a, b = first.centerline[-1], second.centerline[0]
                boundaries.append((
                    math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)),
                    abs(math.degrees(
                        (b.heading-a.heading+math.pi) % (2*math.pi)-math.pi)),
                    abs(b.y-a.y), first, second,
                ))
            if boundaries:
                _gap, _heading, _elevation, before, after = max(
                    boundaries, key=lambda item: item[0])
            else:
                before = after = segments[-1]
            context["lane_id_before"] = lane_id_payload(before.lane_id)
            context["lane_id_after"] = lane_id_payload(after.lane_id)
            context["road_token"] = after.road_look_token
            context["prefab_token"] = (
                after.lane_id.prefab_token or before.lane_id.prefab_token)
            connection = next((item for item in before.successors
                               if item.target == after.lane_id), None)
            if connection is not None:
                context["planned_lane_connection"] = {
                    "type": connection.kind,
                    "source": lane_id_payload(before.lane_id),
                    "target": lane_id_payload(connection.target),
                }
            context["geometry"] = {
                "gap_m": max((item[0] for item in boundaries), default=None),
                "heading_jump_deg": max(
                    (item[1] for item in boundaries), default=None),
                "elevation_jump_m": max(
                    (item[2] for item in boundaries), default=None),
            }
            uid_match = re.search(
                r"\bUID\s+(-?\d+)",
                str(getattr(lane_path, "failure_reason", "")), re.IGNORECASE)
            if uid_match:
                uid = int(uid_match.group(1))
                context["gps_uid"] = uid
                try:
                    context["gps_uid_index"] = context["gps_uids"].index(uid)
                except ValueError:
                    pass
        confidence = getattr(lane_path, "confidence", None)
        if confidence is not None:
            context["confidence"]["components"]["lane_path"] = float(confidence)

    def observe_validation(self, validation):
        metrics = {
            key: value for key, value in asdict(validation).items()
            if key not in ("valid", "failure_reason")
        }
        geometry = self.record["context"]["geometry"]
        geometry["gap_m"] = metrics.get("max_spacing_m", geometry["gap_m"])
        geometry["heading_jump_deg"] = metrics.get(
            "max_heading_jump_deg", geometry["heading_jump_deg"])
        geometry["elevation_jump_m"] = metrics.get(
            "max_height_jump_m", geometry["elevation_jump_m"])
        self.record["context"]["confidence"]["components"][
            "validation_metrics"] = metrics
        return metrics

    def finish(self, status):
        if self._finished:
            return self.record
        self._finished = True
        self.record["status"] = str(status)
        self.record["duration_ms"] = round(
            max(0.0, (time.monotonic() - self._started) * 1000.0), 3)
        # Defer potentially slow handler I/O until the navigation result has
        # already been published. The phase callbacks above only collect data.
        _safe_log(
            logging.INFO,
            "route-build start id=%s revision=%d gps_uids=%d environment=%s",
            self.build_id, self.revision,
            len(self.record["context"].get("gps_uids", ())),
            json.dumps(self.record["context"]["environment"],
                       sort_keys=True, default=str))
        for phase in self.record["phases"]:
            level = (logging.ERROR if phase["status"] == "failed"
                     else logging.INFO)
            _safe_log(
                level,
                "route-build phase-result id=%s revision=%d phase=%s "
                "status=%s duration_ms=%.3f details=%s",
                self.build_id, self.revision, phase["name"], phase["status"],
                phase["duration_ms"],
                json.dumps(phase.get("details") or {}, sort_keys=True,
                           default=str))
        if status == "success":
            _safe_log(logging.INFO,
                "route-build result id=%s revision=%d status=success "
                "duration_ms=%.3f phases=%s",
                self.build_id, self.revision, self.record["duration_ms"],
                json.dumps([{
                    "name": phase["name"], "status": phase["status"],
                    "duration_ms": phase["duration_ms"],
                } for phase in self.record["phases"]],
                    separators=(",", ":")))
        else:
            payload = json.dumps(self.record, sort_keys=True, default=str,
                                 separators=(",", ":"))
            _safe_log(logging.ERROR, "route-build result %s", payload)
        return self.record


def _hash_identifier(value):
    if value is None:
        return None
    digest = hashlib.sha256(str(value).encode("utf-8", "replace")).hexdigest()
    return "anon-" + digest[:16]


def anonymize_failure_record(record):
    """Remove absolute positions and stable game/map identifiers."""
    source = copy.deepcopy(record)
    world = ((source.get("context") or {}).get("world") or {})
    origin = tuple(float(world.get(axis, 0.0) or 0.0) for axis in ("x", "y", "z"))
    identifier_keys = {
        "gps_uid", "road_uid", "start_uid", "end_uid", "road_token",
        "prefab_token", "connector_index", "connector_path",
        "lane_id", "lane_ids",
    }
    reason_keys = {"technical_reason", "failure_reason", "reason"}

    sensitive_text = set()
    absolute_coordinate_text = set()

    def identifier_key(key):
        key = str(key or "").lower()
        return (key in identifier_keys or key == "uid"
                or key.endswith("_uid") or key.endswith("_token")
                or key in ("laneid", "lane_identifier"))

    def collect_identifiers(value, parent_key=None):
        key = str(parent_key or "").lower()
        if isinstance(value, dict):
            for child_key, item in value.items():
                collect_identifiers(item, child_key)
        elif isinstance(value, (list, tuple)):
            for item in value:
                collect_identifiers(item, parent_key)
        elif (identifier_key(key) or key in (
                "gps_uids", "source_gps_uids", "covered_gps_uids")):
            text = str(value)
            if text:
                sensitive_text.add(text)

    collect_identifiers(source)

    def collect_coordinates(value):
        if isinstance(value, dict):
            lowered = {str(key).lower(): item for key, item in value.items()}
            if {"x", "y", "z"}.issubset(lowered):
                for axis in ("x", "y", "z"):
                    try:
                        number = float(lowered[axis])
                    except (TypeError, ValueError):
                        continue
                    absolute_coordinate_text.add(str(lowered[axis]))
                    absolute_coordinate_text.add(str(number))
            for item in value.values():
                collect_coordinates(item)
        elif isinstance(value, (list, tuple)):
            for item in value:
                collect_coordinates(item)

    collect_coordinates(source)

    def coordinate_dict(value):
        if not isinstance(value, dict):
            return False
        keys = {str(key).lower() for key in value}
        if not {"x", "y", "z"}.issubset(keys):
            return False
        try:
            for axis in ("x", "y", "z"):
                float(next(item for key, item in value.items()
                           if str(key).lower() == axis))
        except (StopIteration, TypeError, ValueError):
            return False
        return True

    def relative_coordinates(value):
        values = {
            str(key).lower(): item for key, item in value.items()
            if str(key).lower() in ("x", "y", "z")
        }
        return {
            "relative_x": round(float(values["x"]) - origin[0], 2),
            "relative_y": round(float(values["y"]) - origin[1], 2),
            "relative_z": round(float(values["z"]) - origin[2], 2),
        }

    def redact_text(value):
        text = str(value)
        if re.search(r"\bUID\s+[-+]?\d+", text, re.IGNORECASE):
            return "redacted identifier text"
        if re.search(r"\bLaneId\s*\(", text):
            return "redacted identifier text"
        for identifier in sensitive_text:
            if identifier and identifier in text:
                return "redacted identifier text"
        for coordinate in absolute_coordinate_text:
            if coordinate and coordinate in text:
                return "redacted coordinate text"
        return value

    def clean(value, parent_key=None):
        key = str(parent_key or "").lower()
        if coordinate_dict(value):
            return relative_coordinates(value)
        if isinstance(value, dict):
            result = {}
            for child_key, item in value.items():
                normalized_key = str(child_key).lower()
                if normalized_key in reason_keys and item:
                    result[child_key] = "redacted; use failure code and phase"
                elif identifier_key(normalized_key) and item is not None:
                    if isinstance(item, (list, tuple)):
                        result[child_key] = [
                            _hash_identifier(entry) for entry in item]
                    else:
                        result[child_key] = _hash_identifier(item)
                elif normalized_key in (
                        "gps_uids", "source_gps_uids", "covered_gps_uids"):
                    result[child_key] = [
                        _hash_identifier(entry) for entry in (item or [])]
                else:
                    result[child_key] = clean(item, child_key)
            return result
        if isinstance(value, list):
            if (any(word in key for word in ("world", "point", "position"))
                    and len(value) >= 3
                    and all(isinstance(item, (int, float))
                            for item in value[:3])):
                return [round(float(value[index]) - origin[index], 2)
                        for index in range(3)] + list(value[3:])
            return [clean(item, parent_key) for item in value]
        if isinstance(value, str):
            return redact_text(value)
        return value

    anonymized = clean(source)
    anonymized["anonymized"] = True
    return anonymized


def export_anonymized_failure(record, directory=None):
    if (record or {}).get("status") == "success":
        raise ValueError("only a failed route calculation can be exported")
    if directory is None:
        from core.paths import app_dir
        directory = os.path.join(app_dir(), "route-diagnostics")
    directory = os.path.abspath(os.fspath(directory))
    os.makedirs(directory, exist_ok=True)
    build_id = re.sub(r"[^A-Za-z0-9_.-]", "_", str(
        (record or {}).get("route_build_id") or "unknown"))
    path = os.path.join(directory, f"route-failure-{build_id}.json")
    temporary = path + ".tmp-" + uuid.uuid4().hex
    try:
        with open(temporary, "x", encoding="utf-8") as stream:
            json.dump(anonymize_failure_record(record), stream,
                      ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.remove(temporary)
        except OSError:
            pass
        raise
    return path
