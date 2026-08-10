"""Click-through AR renderer for the authoritative lane trajectory.

The active path has no approximate projection, screen offset, height offset or
geometry repair.  It renders only current ``display_points`` through the exact
``camera_snapshot`` produced from ``Local\\ETS2LACameraProps``.
"""

import sys
import math
import time
from core.navigation.navigation_intent import snapshot_matches_navigation_intent

from PyQt6.QtCore import QPointF, Qt
from PyQt6.QtGui import QColor, QPainter, QPen
from PyQt6.QtWidgets import QApplication, QWidget

from core.camera import (
    CameraSnapshotProducer, project_world_point, project_world_points,
)
from core.navigation.route import Route, iter_path_xz
from core.sdk.scs_sdk import SCSTelemetry


AR_MIN_ROAD_DEPTH_M = 8.0
AR_FADE_IN_END_M = 20.0
AR_FADE_OUT_START_M = 105.0
AR_MAX_ROAD_DEPTH_M = 130.0
AR_TOP_VISIBILITY_FRACTION = 0.12
AR_PROFILE_OCCLUSION_RAD = math.radians(0.30)
AR_PROFILE_BEARING_RAD = math.radians(2.0)


def _road_profile_visible_prefix(world_points, camera_snapshot):
    """Hide the route after a crest proven by its own X/Y/Z profile.

    The overlay has no access to ETS2's depth buffer.  A farther centreline
    sample whose elevation ray drops behind the highest nearer ray is hidden
    by the nearer road profile.  This never moves or fills trajectory points;
    it only returns the first line-of-sight prefix.
    """
    try:
        cx, cy, cz = map(float, camera_snapshot["position"][:3])
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return []
    visible = []
    sight_lines = []
    for point in world_points:
        try:
            x, y, z = map(float, point[:3])
            horizontal = math.hypot(x - cx, z - cz)
            if horizontal < 0.5:
                continue
            angle = math.atan2(y - cy, horizontal)
            bearing = math.atan2(x - cx, z - cz)
        except (TypeError, ValueError, IndexError, OverflowError):
            break
        same_ray_angles = [
            previous_angle for previous_bearing, previous_angle in sight_lines
            if abs(math.atan2(math.sin(bearing - previous_bearing),
                              math.cos(bearing - previous_bearing)))
            <= AR_PROFILE_BEARING_RAD
        ]
        if (len(visible) >= 2 and same_ray_angles
                and angle < max(same_ray_angles) - AR_PROFILE_OCCLUSION_RAD):
            break
        visible.append(point)
        sight_lines.append((bearing, angle))
    return visible


def _route_alpha(depth_m):
    """Distance fade used by the AR render loop, in the ETS2LA style."""
    try:
        depth = float(depth_m)
    except (TypeError, ValueError, OverflowError):
        return 0.0
    if depth <= AR_MIN_ROAD_DEPTH_M or depth >= AR_MAX_ROAD_DEPTH_M:
        return 0.0
    if depth < AR_FADE_IN_END_M:
        return (depth - AR_MIN_ROAD_DEPTH_M) / (
            AR_FADE_IN_END_M - AR_MIN_ROAD_DEPTH_M)
    if depth > AR_FADE_OUT_START_M:
        return (AR_MAX_ROAD_DEPTH_M - depth) / (
            AR_MAX_ROAD_DEPTH_M - AR_FADE_OUT_START_M)
    return 1.0


def _forward_route_suffix(world_points, camera_snapshot):
    """Select original trajectory samples at/after the atomic truck pose.

    HUD intentionally retains a little road behind the truck.  AR must not:
    when the player looks sideways/backward that tail can project onto a
    different visible road.  This function only slices existing samples; it
    never moves, joins or invents trajectory geometry.
    """
    try:
        vehicle = camera_snapshot["vehicle_position"]
        position = (float(vehicle[0]), float(vehicle[2]))
        heading = float(camera_snapshot["vehicle_heading"])
        if (len(world_points) < 2
                or not all(math.isfinite(value)
                           for value in (*position, heading))):
            return []
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return []
    xz = list(iter_path_xz(world_points))
    if len(xz) != len(world_points) or len(xz) < 2:
        return []
    route = Route(xz)
    index, fraction, _progress, distance2 = route._tracking_projection(
        position, heading)
    if not math.isfinite(distance2) or distance2 > 12.0 ** 2:
        return []
    # If the axle is already inside a segment, its first endpoint is behind
    # the vehicle and must not be rendered when the camera is turned around.
    # Start at the following *existing* sample; no interpolated point is
    # invented. At an exact endpoint that sample itself remains valid.
    start = int(index) if fraction <= 1e-6 else int(index) + 1
    start = max(0, min(start, len(world_points) - 1))
    return list(world_points[start:])


def _first_visible_road_strip(projected_values, viewport):
    """Keep only the first continuous, conservatively visible road trace.

    The overlay cannot access ETS2's depth buffer.  It must therefore never
    let a far route reappear after leaving the camera frustum (for example
    behind a crest, building or interchange).  Points are not moved or
    densified: this function only hides samples outside the road visibility
    envelope and stops at its first gap.
    """
    try:
        height = float((viewport or {})["height"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return []
    top = height * AR_TOP_VISIBILITY_FRACTION
    current = []
    started = False
    for point in projected_values:
        valid = False
        if point is not None:
            try:
                x, y, depth = map(float, point[:3])
                valid = (math.isfinite(x) and math.isfinite(y)
                         and math.isfinite(depth)
                         and AR_MIN_ROAD_DEPTH_M <= depth
                         <= AR_MAX_ROAD_DEPTH_M
                         and top <= y <= height)
            except (TypeError, ValueError, IndexError, OverflowError):
                valid = False
        if not valid:
            if started:
                break
            current = []
            continue
        started = True
        current.append((QPointF(x, y), depth))
    return current if len(current) >= 2 else []


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
    scale = max(0.16, min(1.0, 12.0 / depth))
    return 4.0 + 24.0 * scale, 2.0 + 11.0 * scale


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
    return not _visible_segment_parts(first, second, depth, occluders)


def _line_rect_interval(first, second, bounds):
    """Liang-Barsky interval of a screen segment inside one rectangle."""
    left, top, right, bottom = bounds
    dx, dy = second.x() - first.x(), second.y() - first.y()
    enter, leave = 0.0, 1.0
    for p, q in ((-dx, first.x()-left), (dx, right-first.x()),
                 (-dy, first.y()-top), (dy, bottom-first.y())):
        if abs(p) < 1e-9:
            if q < 0.0:
                return None
            continue
        ratio = q / p
        if p < 0.0:
            enter = max(enter, ratio)
        else:
            leave = min(leave, ratio)
        if enter >= leave:
            return None
    return enter, leave


def _visible_segment_parts(first, second, depth, occluders):
    """Clip an AR chord around nearer vehicles instead of midpoint popping.

    The old midpoint test either painted a line through half a vehicle or
    removed a complete two-metre road sample.  Exact interval subtraction
    creates stable cut-outs at the projected cuboid edges without moving or
    smoothing any authoritative world point.
    """
    visible = [(0.0, 1.0)]
    for left, top, right, bottom, vehicle_depth in occluders:
        if depth <= vehicle_depth + 0.25:
            continue
        covered = _line_rect_interval(
            first, second, (left, top, right, bottom))
        if covered is None:
            continue
        cut_start, cut_end = covered
        remaining = []
        for start, end in visible:
            if cut_end <= start or cut_start >= end:
                remaining.append((start, end))
                continue
            if cut_start > start + 1e-4:
                remaining.append((start, min(cut_start, end)))
            if cut_end < end - 1e-4:
                remaining.append((max(cut_end, start), end))
        visible = remaining
        if not visible:
            break

    dx, dy = second.x() - first.x(), second.y() - first.y()
    return [
        (QPointF(first.x()+dx*start, first.y()+dy*start),
         QPointF(first.x()+dx*end, first.y()+dy*end))
        for start, end in visible if end-start > 1e-4
    ]


class AROverlay(QWidget):
    def __init__(self, shared_state):
        super().__init__()
        self.state = shared_state
        self._last_status = None
        self._last_status_at = 0.0
        # Like ETS2LA's AR loop, sample CameraProps *and* SCS telemetry in the
        # renderer process. A CameraProps-only refresh is invalid: it mixes a
        # new view matrix with an older renderTime/truck pose and visibly
        # slides the line off the road during quick camera movement.
        self._render_camera_producer = CameraSnapshotProducer()
        self._render_telemetry = SCSTelemetry()
        self._render_telemetry_connected = self._render_telemetry.connect()
        self._render_telemetry_retry_at = 0.0
        self._render_camera_snapshot = None
        self._last_presented_render_time = -1
        self._published_lane_revision = None
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
    def render_once(self):
        """Acquire and synchronously present at most one SCS render frame.

        This is intentionally driven by ``run_ar`` rather than a Qt timer.
        Camera acquisition, route projection and ``repaint`` therefore remain
        on one thread and in one call stack, matching ETS2LA's immediate AR
        loop without copying its GPLv3 implementation.
        """
        if self.state.get("app_shutdown_requested", False):
            return False

        snapshot = self._fresh_render_camera_snapshot()
        if not snapshot.get("valid", False):
            # A frame boundary is a transient acquisition race. Keep the last
            # correctly projected frame instead of flashing an empty/old one.
            if "frame boundary" in str(snapshot.get("failure_reason", "")):
                return True
            self._render_camera_snapshot = snapshot
            self.repaint()
            return True

        render_time = int(snapshot.get("render_time_us", 0) or 0)
        if (render_time > 0
                and render_time == self._last_presented_render_time):
            return True
        self._last_presented_render_time = render_time
        self._render_camera_snapshot = snapshot
        self._sync_viewport(snapshot)
        self.repaint()
        return True

    def _fresh_render_camera_snapshot(self):
        shared = self.state.get("camera_snapshot", {}) or {}
        if not getattr(self, "_render_telemetry_connected", False):
            now = time.monotonic()
            if now - getattr(self, "_render_telemetry_retry_at", 0.0) < 1.0:
                return shared
            self._render_telemetry_retry_at = now
            try:
                self._render_telemetry_connected = bool(
                    self._render_telemetry.connect())
            except Exception:
                self._render_telemetry_connected = False
            if not self._render_telemetry_connected:
                return shared
        try:
            raw = self._render_telemetry.update() or {}
            render_time = int(raw.get("renderTime", 0) or 0)
            # A zero render clock cannot prove that CameraProps and telemetry
            # belong to one frame. The engine's already-atomic snapshot is the
            # safe fallback for older telemetry plugins.
            if render_time <= 0:
                return shared
            sampled_at = time.monotonic()
            fresh = self._render_camera_producer.read(
                render_time, sampled_at, now=sampled_at)
            # Detect a frame boundary that occurred while CameraProps was
            # copied. Never publish a hybrid of adjacent game frames.
            end_render_time = int(
                self._render_telemetry.read_long_long(24)[0] or 0)
            if end_render_time != render_time:
                return {
                    "valid": False,
                    "failure_reason": "camera crossed an SCS render-frame boundary",
                    "revision": int(shared.get("revision", -1) or -1),
                    "viewport": shared.get("viewport"),
                }
            placement = raw.get("truckPlacement", {}) or {}
            x = float(placement["coordinateX"])
            y = float(placement["coordinateY"])
            z = float(placement["coordinateZ"])
            turns = float(placement["rotationX"])
            heading = (turns * math.tau + math.pi) % math.tau - math.pi
        except Exception as error:
            return {
                "valid": False,
                "failure_reason": f"atomic AR frame could not be read: {error}",
                "revision": int(shared.get("revision", -1) or -1),
                "viewport": shared.get("viewport"),
            }
        if not fresh.get("valid", False):
            return fresh
        # Camera, render clock and vehicle pose were sampled inside one proven
        # render tick. This private frame is used only for AR projection; the
        # shared navigation snapshot and its revision remain untouched.
        fresh = dict(fresh)
        fresh.update({
            "vehicle_position": [x, y, z],
            "vehicle_heading": heading,
            "telemetry_valid": True,
            "ar_frame_atomic": True,
        })
        return fresh

    def _active_camera_snapshot(self):
        private = getattr(self, "_render_camera_snapshot", None)
        return private or self.state.get("camera_snapshot", {}) or {}

    def _sync_viewport(self, snapshot=None):
        """Follow the actual ETS2 client rectangle, including monitor moves."""
        snapshot = snapshot or self.state.get("camera_snapshot", {}) or {}
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
        snapshot = AROverlay._active_camera_snapshot(self)
        projected = project_world_point(
            snapshot, point,
            telemetry_timestamp=float(snapshot.get(
                "telemetry_timestamp", 0.0) or 0.0))
        if projected is None:
            return None
        return QPointF(projected[0], projected[1])

    def _publish_status(self, ready, reason, lane_revision=-1):
        now = time.monotonic()
        payload = {
            "ready": bool(ready), "reason": str(reason or ""),
            "lane_revision": int(lane_revision),
            "camera_revision": int(AROverlay._active_camera_snapshot(
                self).get("revision", -1) or -1),
            "timestamp": now,
        }
        # Camera revision advances every render frame. Including it in the
        # signature turned a diagnostic Manager write into a 60/120/144 Hz IPC
        # stream and delayed the renderer we were trying to synchronize.
        signature = (payload["ready"], payload["reason"],
                     payload["lane_revision"])
        if signature != self._last_status or now - self._last_status_at >= 1.0:
            self.state.set("ar_navigation_readiness", payload)
            self._last_status = signature
            self._last_status_at = now

    def _publish_lane_revision(self, revision):
        """Publish revision changes, not one Manager write per render frame."""
        revision = int(revision)
        if revision != self._published_lane_revision:
            self.state.set("ar_lane_revision", revision)
            self._published_lane_revision = revision

    def paintEvent(self, event):
        if not self.state.get("ar_enabled", True):
            self._publish_lane_revision(-1)
            self._publish_status(False, "AR is disabled")
            return
        if not self.state.get("game_in_truck", False):
            self._publish_lane_revision(-1)
            self._publish_status(False, "game telemetry is unavailable")
            return
        if self.state.get("navigation_recalculating", False):
            self._publish_lane_revision(-1)
            self._publish_status(False, "navigation is recalculating")
            return

        current_revision, world, route_reason = self._current_display_points_with_reason()
        if len(world) < 2:
            self._publish_lane_revision(-1)
            self._publish_status(False, route_reason or "lane trajectory is unavailable")
            return
        camera_snapshot = self._active_camera_snapshot()
        if not camera_snapshot.get("valid", False):
            self._publish_lane_revision(-1)
            self._publish_status(False, str(camera_snapshot.get(
                "failure_reason") or "atomic AR camera frame is unavailable"),
                current_revision)
            return
        telemetry_timestamp = float(camera_snapshot.get(
            "telemetry_timestamp", 0.0) or 0.0)
        world = _forward_route_suffix(world, camera_snapshot)
        if len(world) < 2:
            self._publish_lane_revision(-1)
            self._publish_status(
                False, "trajectory has no forward samples at the atomic vehicle pose",
                current_revision)
            return
        world = _road_profile_visible_prefix(world, camera_snapshot)
        if len(world) < 2:
            self._publish_lane_revision(-1)
            self._publish_status(
                False, "trajectory is hidden by the proven road elevation profile",
                current_revision)
            return
        projected_values, camera_reason = project_world_points(
            camera_snapshot, world,
            telemetry_timestamp=telemetry_timestamp)
        if camera_reason:
            self._publish_lane_revision(-1)
            self._publish_status(False, camera_reason, current_revision)
            return

        # Qt has no access to the game's depth buffer. Suppress the cab-hidden
        # start, cap the conservative road-visible distance and never render a
        # later strip after the route first leaves that envelope.
        strip = _first_visible_road_strip(
            projected_values, camera_snapshot.get("viewport") or {})
        if not strip:
            self._publish_lane_revision(-1)
            self._publish_status(False, "all trajectory points are outside the camera frustum",
                                 current_revision)
            return

        self._publish_lane_revision(current_revision)
        self._publish_status(True, "", current_revision)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        # Paint far segments first, then the near ones.  Per-segment depth
        # scaling makes the trace lie visually on the road while round caps
        # keep adjacent samples continuous.
        segments = []
        for first, second in zip(strip, strip[1:]):
            depth = (first[1] + second[1]) * 0.5
            alpha = _route_alpha(depth)
            if alpha > 0.0:
                # Do not punch rectangular holes around traffic. Without the
                # game depth buffer those cut-outs are not physically proven
                # and visibly pop as vehicles move.
                segments.append((depth, alpha, first[0], second[0]))
        segments.sort(key=lambda item: item[0], reverse=True)
        for halo in (True, False):
            for depth, alpha, first, second in segments:
                halo_width, core_width = _perspective_route_widths(
                    depth, camera_snapshot)
                painter.setPen(QPen(
                    QColor(45, 142, 255, round(
                        (95 if halo else 240) * alpha)),
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
        except (TypeError, ValueError, OverflowError):
            return -1, [], "lane trajectory metadata is malformed"
        if not snapshot.get("valid", False):
            return -1, [], str(snapshot.get("failure_reason")
                               or "lane trajectory is invalid")
        if snapshot_revision != current_revision:
            return -1, [], "lane trajectory revision is stale"
        if not snapshot_matches_navigation_intent(self.state, snapshot):
            return -1, [], "lane trajectory belongs to a different navigation intent"
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
    """Run an immediate SCS-render-clock-driven AR loop.

    Qt owns only the transparent native window. It does not schedule camera
    frames: every accepted SCS render tick is acquired and synchronously
    painted before this loop processes the next one.
    """
    existing = QApplication.instance()
    app = existing or QApplication(sys.argv)
    overlay = AROverlay(shared_state)
    overlay.show()
    while True:
        app.processEvents()
        if not overlay.render_once():
            break
        # Yield to ETS2 and the other UltraPilot processes while still polling
        # well above normal 60/120/144 Hz game render rates.
        time.sleep(0.001)
    overlay.close()
    app.processEvents()
    return overlay
