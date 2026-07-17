"""
AR overlay (best-effort): a transparent, click-through, always-on-top window
drawn over the game that projects the anticipated route onto the road.

Honest limitation: SCS telemetry does not expose the full game camera matrix,
so this uses an *approximate* chase-camera model with calibration parameters
(FOV, height, pitch, behind).  The first alignment will be off — tune the
ar_* values in shared state (or settings) until the blue line sits on the road.
The overlay is click-through, so it never blocks the game.
"""

import sys
import math
import time
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QPainter, QColor, QPen, QPolygonF
from PyQt6.QtCore import Qt, QTimer, QPointF


class AROverlay(QWidget):
    def __init__(self, shared_state):
        super().__init__()
        self.state = shared_state
        self._last_path = []
        self._last_path_at = 0.0
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint |
            Qt.WindowType.WindowStaysOnTopHint |
            Qt.WindowType.Tool |
            Qt.WindowType.WindowTransparentForInput)   # click-through
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setAttribute(Qt.WidgetAttribute.WA_ShowWithoutActivating)
        screen = QApplication.primaryScreen().geometry()
        self.setGeometry(screen)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(60)

    def _cfg(self, key, default):
        v = self.state.get(key, None)
        return float(v) if v is not None else default

    def _project(self, ahead, lateral, road_height=0.0):
        """Approx world→screen for a chase camera. Returns QPointF or None."""
        if not self.state.get("ar_enabled", True):
            return None
        w, h = self.width(), self.height()
        # Defaults tuned for ETS2's standard interior/chase camera so the route
        # roughly lands on the road out-of-the-box (fine-tune in Settings → AR).
        fov = self._cfg("ar_fov", 75.0)                 # degrees (ETS2 default ~75)
        cam_h = self._cfg("ar_height", 2.6)             # camera height (m)
        pitch = math.radians(self._cfg("ar_pitch", 4.0))
        behind = self._cfg("ar_behind", 5.0)
        d = ahead + behind
        if d < 1.5:
            return None
        f = (w / 2) / math.tan(math.radians(fov) / 2)
        # camera looks slightly down by `pitch`
        # A road above the truck must project higher on the windscreen and a
        # road below it lower. The old fixed ground plane made the route float
        # in the air at ramps and bridges.
        y_world = cam_h - road_height
        sx = w / 2 + (lateral / d) * f
        sy = h / 2 + ((y_world / d) * f) + math.tan(pitch) * f
        return QPointF(sx, sy)

    @staticmethod
    def _road_height_at(px, pz, road_segments, truck_altitude):
        """Return the nearby road surface height relative to the truck.

        Only accept a route point when it lies on published map geometry. This
        prevents stale/incorrect AR points from being painted through the sky.
        """
        best = None
        for segment in road_segments:
            try:
                a, b = segment[0], segment[1]
                ax, az = float(a[0]), float(a[1])
                bx, bz = float(b[0]), float(b[1])
                ah = float(a[2]) if len(a) > 2 else truck_altitude
                bh = float(b[2]) if len(b) > 2 else ah
            except (TypeError, ValueError, IndexError):
                continue
            vx, vz = bx - ax, bz - az
            length2 = vx * vx + vz * vz
            if length2 < 1e-6:
                continue
            t = max(0.0, min(1.0,
                    ((px - ax) * vx + (pz - az) * vz) / length2))
            qx, qz = ax + vx * t, az + vz * t
            distance2 = (px - qx) ** 2 + (pz - qz) ** 2
            relative_height = ah + (bh - ah) * t - truck_altitude
            # At an overpass two road segments can occupy the same X/Z. Pick
            # the deck closest to the truck's current level instead of an
            # arbitrary upper/lower road, while horizontal distance remains
            # the hard on-road acceptance condition below.
            score = distance2 + relative_height * relative_height * 2.0
            if best is None or score < best[0]:
                best = (score, distance2, relative_height)
        # The planned driving line can be laterally offset from the map road
        # centre, but it must still be within a normal carriageway width.
        return best[2] if best is not None and best[1] <= 10.0 ** 2 else None

    def paintEvent(self, event):
        if (not self.state.get("ar_enabled", True)
                or not self.state.get("game_in_truck", False)):
            return
        if (self.state.get("navigation_unreliable", False)
                or self.state.get("navigation_recalculating", False)):
            self._last_path = []
            self._last_path_at = 0.0
            return
        pos = self.state.get("truck_world_pos")
        # Use the same real GPS path as the HUD. Keep the last valid path through
        # short shared-memory refresh gaps so the AR ribbon cannot flicker out.
        # AR is safety-sensitive: draw only the route that the navigation
        # plugin has localized and published for steering. Never fall back to
        # raw/stale route buffers or an arbitrary road-ahead path.
        path = self.state.get("nav_path", []) or []
        now = time.monotonic()
        if len(path) >= 2:
            self._last_path = [tuple(point[:2]) for point in path]
            self._last_path_at = now
        elif self._last_path and now - self._last_path_at < 4.0:
            path = self._last_path
        if not pos or len(path) < 2:
            return
        h = self.state.get("truck_heading", 0.0) or 0.0
        tx, tz = pos

        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Densify sparse GPS nodes before projection. This keeps the ribbon on
        # the road and prevents long diagonal chords across curved junctions.
        world = [tuple(point[:2]) for point in path]
        if not world:
            return
        closest = min(range(len(world)),
                      key=lambda i: math.dist(tuple(pos), world[i]))
        closest_distance = math.dist(tuple(pos), world[closest])
        # A route that cannot be localized at the truck must not be projected:
        # it creates the horizontal screen-wide stroke seen in the old overlay.
        if closest_distance > 12.0:
            return
        world = world[closest:]
        # Connect the ribbon to the truck only when the first valid GPS point
        # is genuinely nearby. A stale/different map node must never produce a
        # screen-wide line.
        if world and math.dist(tuple(pos), world[0]) <= 8.0:
            world.insert(0, tuple(pos))
        dense = []
        previous_direction = None
        for a, b in zip(world, world[1:]):
            vx, vz = b[0] - a[0], b[1] - a[1]
            distance = math.hypot(vx, vz)
            if distance > 40.0:
                break
            if distance > .8:
                direction = (vx / distance, vz / distance)
                if previous_direction is not None:
                    dot = max(-1.0, min(1.0,
                              direction[0] * previous_direction[0]
                              + direction[1] * previous_direction[1]))
                    if math.degrees(math.acos(dot)) > 105.0:
                        break
                previous_direction = direction
            samples = max(1, min(24, int(distance / 3.0)))
            for index in range(samples):
                t = index / samples
                dense.append((a[0] + (b[0] - a[0]) * t,
                              a[1] + (b[1] - a[1]) * t))
        if world:
            dense.append(world[-1])

        road_segments = self.state.get("map_road_segments", []) or []
        truck_altitude = float(self.state.get("truck_altitude", 0.0) or 0.0)
        if not road_segments:
            return
        strips = []
        pts = []
        for px, pz in dense:
            dx, dz = px - tx, pz - tz
            ahead = dx * (-math.sin(h)) + dz * (-math.cos(h))
            lateral = dx * math.cos(h) - dz * math.sin(h)
            # The near-camera projection is numerically explosive and was the
            # source of screen-wide blue diagonals. Only render a plausible,
            # forward road corridor and let the line begin a few metres ahead.
            corridor = max(5.0, 3.8 + ahead * 0.16)
            road_height = self._road_height_at(
                px, pz, road_segments, truck_altitude)
            p = (self._project(ahead, lateral, road_height)
                 if (road_height is not None and 7.0 < ahead < 145.0
                     and abs(lateral) < corridor) else None)
            if p:
                if not (-20.0 <= p.x() <= self.width() + 20.0
                        and self.height() * .40 <= p.y() <= self.height() * .98):
                    p = None
            if p:
                if (pts and math.hypot(p.x() - pts[-1].x(),
                                      p.y() - pts[-1].y()) > self.width() * 0.08):
                    if len(pts) >= 2:
                        strips.append(pts)
                    pts = []
                pts.append(p)
            elif len(pts) >= 2:
                strips.append(pts)
                pts = []
        if len(pts) >= 2:
            strips.append(pts)
        for pts in strips:
            # Glow + core line, like ETS2LA's painted route.
            glow = QPen(QColor(45, 142, 255, 90), 18,
                       Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap,
                       Qt.PenJoinStyle.RoundJoin)
            qp.setPen(glow)
            qp.drawPolyline(QPolygonF(pts))
            core = QPen(QColor(45, 142, 255, 235), 7,
                       Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap,
                       Qt.PenJoinStyle.RoundJoin)
            qp.setPen(core)
            qp.drawPolyline(QPolygonF(pts))


def run_ar(shared_state):
    app = QApplication.instance() or QApplication(sys.argv)
    ov = AROverlay(shared_state)
    ov.show()
    if not QApplication.instance().startingUp():
        return ov
    sys.exit(app.exec())
