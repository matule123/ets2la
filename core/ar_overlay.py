"""Click-through AR renderer for the authoritative lane trajectory.

The active path has no approximate projection, screen offset, height offset or
geometry repair.  It renders only current ``display_points`` through the exact
``camera_snapshot`` produced from ``Local\\ETS2LACameraProps``.
"""

import sys
import math
import time

from PyQt6.QtCore import QPointF, QTimer, Qt
from PyQt6.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import QApplication, QWidget

from core.camera import camera_snapshot_reason, project_world_point


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

    def _camera_reason(self):
        snapshot = self.state.get("camera_snapshot", {}) or {}
        return camera_snapshot_reason(
            snapshot, telemetry_timestamp=float(self.state.get(
                "telemetry_timestamp", 0.0) or 0.0))

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
        camera_reason = self._camera_reason()
        if camera_reason:
            self.state.set("ar_lane_revision", -1)
            self._publish_status(False, camera_reason, current_revision)
            return

        projected = [self._project_world(point) for point in world]
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
        for points in strips:
            painter.setPen(QPen(QColor(45, 142, 255, 90), 18,
                                Qt.PenStyle.SolidLine,
                                Qt.PenCapStyle.RoundCap,
                                Qt.PenJoinStyle.RoundJoin))
            painter.drawPolyline(QPolygonF(points))
            painter.setPen(QPen(QColor(45, 142, 255, 235), 7,
                                Qt.PenStyle.SolidLine,
                                Qt.PenCapStyle.RoundCap,
                                Qt.PenJoinStyle.RoundJoin))
            painter.drawPolyline(QPolygonF(points))

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
