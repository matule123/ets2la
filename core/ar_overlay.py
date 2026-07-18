"""Click-through AR renderer for the authoritative lane trajectory.

The active path has no approximate projection, screen offset, height offset or
geometry repair.  It renders only current ``display_points`` through the exact
``camera_snapshot`` produced from ``Local\\ETS2LACameraProps``.
"""

import sys
import math
import time

from PyQt6.QtCore import QPointF, QTimer, Qt
from PyQt6.QtGui import QColor, QPainter, QPen
from PyQt6.QtWidgets import QApplication, QWidget

from core.camera import project_world_point, project_world_points


def _perspective_route_widths(depth_m, camera_snapshot=None):
    """Return halo/core pixel widths for a road-bound perspective trace.

    A constant screen-space pen stays equally thick at the horizon and reads
    as a vertical cable.  Scale it by camera depth: deliberately substantial
    near the cab, but narrow in the distance like a marking painted on the
    road.  This changes presentation only; world X/Y/Z remain authoritative.
    """
    try:
        depth = max(0.1, float(depth_m))
    except (TypeError, ValueError, OverflowError):
        depth = 1000.0
    snapshot = camera_snapshot or {}
    try:
        viewport_width = float((snapshot.get("viewport") or {})["width"])
        hfov = math.radians(float(snapshot["fov_horizontal_deg"]))
        focal_px = viewport_width / (2.0 * math.tan(hfov * 0.5))
        # A normal ETS2 lane is about 3.5 m wide.  The blue core is a 1.75 m
        # road-bound ribbon: exactly half a lane in world scale.  Clamp only
        # the extreme near/horizon cases so it remains legible and stable.
        core = max(7.0, min(150.0, focal_px * 1.75 / depth))
        return core + max(5.0, core * 0.16), core
    except (KeyError, TypeError, ValueError, OverflowError, ZeroDivisionError):
        scale = max(0.16, min(1.0, 12.0 / depth))
        return 7.0 + 36.0 * scale, 4.0 + 25.0 * scale


def _traffic_occluders(camera_snapshot, traffic, telemetry_timestamp=0.0):
    """Return screen rectangles occupied by nearer game vehicles.

    Qt overlays cannot read the game's depth buffer.  We reconstruct a
    conservative depth mask from the authoritative ETS2LA traffic cuboids so
    the route is not painted through cars and trucks.
    """
    occluders = []
    for vehicle in traffic or ():
        try:
            x, y, z = (float(vehicle[key]) for key in ("x", "y", "z"))
            width = max(0.8, float(vehicle.get("width", 2.0) or 2.0))
            height = max(1.0, float(vehicle.get("height", 1.7) or 1.7))
            length = max(1.5, float(vehicle.get("length", 4.5) or 4.5))
            yaw = float(vehicle.get("yaw", 0.0) or 0.0)
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
        forward = (-math.sin(yaw), -math.cos(yaw))
        right = (math.cos(yaw), -math.sin(yaw))
        corners = []
        for longitudinal in (-length * 0.5, length * 0.5):
            for lateral in (-width * 0.5, width * 0.5):
                wx = x + forward[0] * longitudinal + right[0] * lateral
                wz = z + forward[1] * longitudinal + right[1] * lateral
                for wy in (y, y + height):
                    corners.append((wx, wy, wz))
        projected, reason = project_world_points(
            camera_snapshot, corners,
            telemetry_timestamp=float(telemetry_timestamp or 0.0))
        visible = [point for point in projected if point is not None]
        if reason or len(visible) < 2:
            continue
        xs, ys = [p[0] for p in visible], [p[1] for p in visible]
        left, right_px = min(xs) - 3.0, max(xs) + 3.0
        top, bottom = min(ys) - 3.0, max(ys) + 3.0
        if right_px - left < 3.0 or bottom - top < 3.0:
            continue
        occluders.append((left, top, right_px, bottom,
                          min(float(p[2]) for p in visible)))
    return occluders


def _segment_is_occluded(first, second, depth, occluders):
    midpoint_x = (first.x() + second.x()) * 0.5
    midpoint_y = (first.y() + second.y()) * 0.5
    for left, top, right, bottom, vehicle_depth in occluders:
        if (left <= midpoint_x <= right and top <= midpoint_y <= bottom
                and depth > vehicle_depth + 0.25):
            return True
    return False


class AROverlay(QWidget):
    def __init__(self, shared_state):
        super().__init__()
        self.state = shared_state
        self._last_status = None
        self._last_status_at = 0.0
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint
            | Qt.WindowType.WindowStaysOnTopHint
            | Qt.WindowType.Tool
            | Qt.WindowType.WindowTransparentForInput)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setAttribute(Qt.WidgetAttribute.WA_ShowWithoutActivating)
        screen = QApplication.primaryScreen()
        if screen is not None:
            self.setGeometry(screen.geometry())
        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(16)

    def _tick(self):
        self._sync_viewport()
        self.update()

    def _sync_viewport(self):
        """Follow the actual ETS2 client rectangle, including monitor moves."""
        snapshot = self.state.get("camera_snapshot", {}) or {}
        viewport = snapshot.get("viewport") or {}
        try:
            geometry = (int(viewport["x"]), int(viewport["y"]),
                        int(viewport["width"]), int(viewport["height"]))
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        if geometry[2] < 64 or geometry[3] < 64:
            return
        current = (self.x(), self.y(), self.width(), self.height())
        if geometry != current:
            self.setGeometry(*geometry)

    def _project_world(self, point):
        snapshot = self.state.get("camera_snapshot", {}) or {}
        projected = project_world_point(
            snapshot, point,
            telemetry_timestamp=float(self.state.get(
                "telemetry_timestamp", 0.0) or 0.0))
        if projected is None:
            return None
        return QPointF(projected[0], projected[1])

    def _publish_status(self, ready, reason, lane_revision=-1):
        now = time.monotonic()
        payload = {
            "ready": bool(ready), "reason": str(reason or ""),
            "lane_revision": int(lane_revision),
            "camera_revision": int((self.state.get(
                "camera_snapshot", {}) or {}).get("revision", -1) or -1),
            "timestamp": now,
        }
        signature = (payload["ready"], payload["reason"],
                     payload["lane_revision"], payload["camera_revision"])
        if signature != self._last_status or now - self._last_status_at >= 1.0:
            self.state.set("ar_navigation_readiness", payload)
            self._last_status = signature
            self._last_status_at = now

    def paintEvent(self, event):
        if not self.state.get("ar_enabled", True):
            self.state.set("ar_lane_revision", -1)
            self._publish_status(False, "AR is disabled")
            return
        if not self.state.get("game_in_truck", False):
            self.state.set("ar_lane_revision", -1)
            self._publish_status(False, "game telemetry is unavailable")
            return
        if self.state.get("navigation_recalculating", False):
            self.state.set("ar_lane_revision", -1)
            self._publish_status(False, "navigation is recalculating")
            return

        current_revision, world, route_reason = self._current_display_points_with_reason()
        if len(world) < 2:
            self.state.set("ar_lane_revision", -1)
            self._publish_status(False, route_reason or "lane trajectory is unavailable")
            return
        camera_snapshot = self.state.get("camera_snapshot", {}) or {}
        telemetry_timestamp = float(self.state.get(
            "telemetry_timestamp", 0.0) or 0.0)
        projected_values, camera_reason = project_world_points(
            camera_snapshot, world,
            telemetry_timestamp=telemetry_timestamp)
        if camera_reason:
            self.state.set("ar_lane_revision", -1)
            self._publish_status(False, camera_reason, current_revision)
            return

        # The overlay has no access to the game's depth buffer.  Suppress the
        # first metres that are physically hidden by the cab/dashboard rather
        # than painting the road trace over the interior.
        projected = [None if point is None or float(point[2]) < 8.0 else
                     (QPointF(point[0], point[1]), float(point[2]))
                     for point in projected_values]
        strips, current = [], []
        for point in projected:
            if point is None:
                if len(current) >= 2:
                    strips.append(current)
                current = []
            else:
                current.append(point)
        if len(current) >= 2:
            strips.append(current)
        if not strips:
            self.state.set("ar_lane_revision", -1)
            self._publish_status(False, "all trajectory points are outside the camera frustum",
                                 current_revision)
            return

        self.state.set("ar_lane_revision", current_revision)
        self._publish_status(True, "", current_revision)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        # Paint far segments first, then the near ones.  Per-segment depth
        # scaling makes the trace lie visually on the road while round caps
        # keep adjacent samples continuous.
        segments = []
        for strip in strips:
            for first, second in zip(strip, strip[1:]):
                depth = (first[1] + second[1]) * 0.5
                segments.append((depth, first[0], second[0]))
        occluders = _traffic_occluders(
            camera_snapshot, self.state.get("traffic", []) or [],
            telemetry_timestamp)
        segments.sort(key=lambda item: item[0], reverse=True)
        for halo in (True, False):
            for depth, first, second in segments:
                if _segment_is_occluded(first, second, depth, occluders):
                    continue
                halo_width, core_width = _perspective_route_widths(
                    depth, camera_snapshot)
                painter.setPen(QPen(
                    QColor(45, 142, 255, 95 if halo else 240),
                    halo_width if halo else core_width,
                    Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap,
                    Qt.PenJoinStyle.RoundJoin))
                painter.drawLine(first, second)

    def _current_display_points(self):
        """Backward-compatible two-value reader used by integration checks."""
        revision, points, _ = AROverlay._current_display_points_with_reason(self)
        return revision, points

    def _current_display_points_with_reason(self):
        """Return the unmodified current-revision display path and reason."""
        snapshot = self.state.get("lane_trajectory", {}) or {}
        try:
            current_revision = int(self.state.get(
                "lane_trajectory_revision", -1) or -1)
            snapshot_revision = int(snapshot.get("revision", -2) or -2)
            heartbeat = float(self.state.get(
                "lane_trajectory_heartbeat", 0.0) or 0.0)
            snapshot_uids = tuple(int(uid) for uid in
                                  (snapshot.get("source_gps_uids", ()) or ()))
            game_uids = tuple(int(uid) for uid in
                              (self.state.get("game_route_node_uids", []) or []))
        except (TypeError, ValueError, OverflowError):
            return -1, [], "lane trajectory metadata is malformed"
        if not snapshot.get("valid", False):
            return -1, [], str(snapshot.get("failure_reason")
                               or "lane trajectory is invalid")
        if snapshot_revision != current_revision:
            return -1, [], "lane trajectory revision is stale"
        if snapshot_uids != game_uids:
            return -1, [], "lane trajectory belongs to a different GPS target"
        if snapshot.get("request_id") != self.state.get("nav_recalc_request"):
            return -1, [], "lane trajectory request is stale"
        if heartbeat <= 0.0 or time.monotonic() - heartbeat > 0.5:
            return -1, [], "map plugin heartbeat is stale"
        if self.state.get("telemetry_valid", True) is False:
            return -1, [], "vehicle telemetry is invalid"
        if self.state.get("navigation_recalculating", False):
            return -1, [], "navigation is recalculating"
        points = snapshot.get("display_points", []) or []
        if len(points) < 2:
            return -1, [], "display trajectory has fewer than two points"
        try:
            if any(not isinstance(point, (list, tuple)) or len(point) < 3
                   or not all(math.isfinite(float(value))
                              for value in point[:3]) for point in points):
                return -1, [], "display trajectory contains malformed or non-finite 3D points"
        except (TypeError, ValueError, OverflowError):
            return -1, [], "display trajectory metadata is malformed"
        return current_revision, points, ""


def run_ar(shared_state):
    existing = QApplication.instance()
    app = existing or QApplication(sys.argv)
    overlay = AROverlay(shared_state)
    overlay.show()
    if existing is not None:
        return overlay
    sys.exit(app.exec())
