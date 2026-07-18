"""Runtime navigation readiness audit shared by diagnostics and tests."""

import math
import time

from core.camera import camera_snapshot_reason


CONFIDENCE_THRESHOLD = 0.72


def _check(ok, reason=""):
    return {"ready": bool(ok), "reason": "" if ok else str(reason)}


def build_runtime_preflight(state, now=None):
    """Return fail-closed readiness for the currently published runtime state.

    This does not create geometry or provide fallbacks.  It makes route-target,
    revision, height, camera, consumer and control blockers visible in one
    atomic diagnostic payload.
    """
    now = time.monotonic() if now is None else float(now)
    telemetry_ok = state.get("telemetry_valid", False) is True
    game_uids = tuple(state.get("game_route_node_uids", ()) or ())
    gps_ok = len(game_uids) >= 2
    snapshot = state.get("lane_trajectory", {}) or {}
    try:
        revision = int(snapshot.get("revision", -1) or -1)
        current_revision = int(state.get("lane_trajectory_revision", -2) or -2)
        snapshot_uids = tuple(snapshot.get("source_gps_uids", ()) or ())
        confidence = float(snapshot.get("confidence", 0.0) or 0.0)
        points = snapshot.get("points", ()) or ()
        xyz_ok = bool(len(points) >= 2 and all(
            isinstance(point, (list, tuple)) and len(point) >= 3
            and all(math.isfinite(float(value)) for value in point[:3])
            for point in points))
    except (TypeError, ValueError, OverflowError):
        revision, current_revision, snapshot_uids = -1, -2, ()
        confidence, xyz_ok = 0.0, False
    revision_ok = bool(revision == current_revision
                       and snapshot_uids == game_uids
                       and snapshot.get("request_id")
                           == state.get("nav_recalc_request"))
    try:
        heartbeat = float(state.get("lane_trajectory_heartbeat", 0.0) or 0.0)
        telemetry_timestamp = float(state.get(
            "telemetry_timestamp", 0.0) or 0.0)
    except (TypeError, ValueError, OverflowError):
        heartbeat, telemetry_timestamp = 0.0, 0.0
    map_ok = bool(snapshot.get("valid", False) and revision_ok and xyz_ok
                  and heartbeat > 0.0 and now - heartbeat <= 0.5
                  and not state.get("navigation_recalculating", False))
    camera = state.get("camera_snapshot", {}) or {}
    camera_reason = camera_snapshot_reason(
        camera, now=now,
        telemetry_timestamp=telemetry_timestamp)
    hud = state.get("hud_navigation_readiness", {}) or {}
    ar = state.get("ar_navigation_readiness", {}) or {}
    autopilot = state.get("autopilot_navigation_readiness", {}) or {}

    checks = {
        "telemetry": _check(telemetry_ok, "vehicle telemetry is unavailable"),
        "gps_target": _check(gps_ok, "GPS has no target or fewer than two UIDs"),
        "lane_trajectory": _check(
            map_ok, snapshot.get("failure_reason")
            or ("stale target/revision" if not revision_ok
                else "lane trajectory is unavailable or stale")),
        "xyz_preserved": _check(xyz_ok, "trajectory does not contain finite X/Y/Z"),
        "camera": _check(not camera_reason, camera_reason),
        "hud": _check(hud.get("ready", False),
                      hud.get("reason", "HUD has not acknowledged the revision")),
        "ar": _check(ar.get("ready", False),
                     ar.get("reason", "AR has not acknowledged the revision")),
        "confidence": _check(
            math.isfinite(confidence) and confidence >= CONFIDENCE_THRESHOLD,
            f"confidence {confidence:.6f} is below {CONFIDENCE_THRESHOLD:.2f}"),
        "autopilot": _check(
            autopilot.get("ready", False),
            autopilot.get("reason", "autopilot has not accepted the revision")),
    }
    return {
        "timestamp": now, "revision": current_revision,
        "gps_uid_count": len(game_uids), "confidence": confidence,
        "confidence_threshold": CONFIDENCE_THRESHOLD,
        "manual_autopilot_off_policy": "immediate control release",
        "automatic_navigation_failure_policy": "smooth steering release, throttle zero, safe brake",
        "checks": checks,
        "display_ready": all(checks[name]["ready"]
                             for name in ("telemetry", "gps_target",
                                          "lane_trajectory", "hud")),
        "ar_ready": all(checks[name]["ready"]
                        for name in ("telemetry", "gps_target",
                                     "lane_trajectory", "camera", "ar")),
        "autopilot_ready": all(checks[name]["ready"]
                               for name in ("telemetry", "gps_target",
                                            "lane_trajectory", "confidence",
                                            "autopilot")),
    }
