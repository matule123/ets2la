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
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QPainter, QColor, QPen, QPolygonF
from PyQt6.QtCore import Qt, QTimer, QPointF


class AROverlay(QWidget):
    def __init__(self, shared_state):
        super().__init__()
        self.state = shared_state
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

    def _project(self, ahead, lateral):
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
        y_world = cam_h
        sx = w / 2 + (lateral / d) * f
        sy = h / 2 + ((y_world / d) * f) + math.tan(pitch) * f
        return QPointF(sx, sy)

    def paintEvent(self, event):
        if not self.state.get("ar_enabled", True):
            return
        pos = self.state.get("truck_world_pos")
        # Draw the active navigation path; fall back to the map-computed road
        # ahead so the line shows during map-based driving (no recorded route).
        path = self.state.get("nav_path", []) or self.state.get("map_path", []) or []
        if not pos or len(path) < 2:
            return
        h = self.state.get("truck_heading", 0.0) or 0.0
        tx, tz = pos

        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)

        pts = []
        for px, pz in path:
            dx, dz = px - tx, pz - tz
            ahead = dx * (-math.sin(h)) + dz * (-math.cos(h))
            lateral = dx * math.cos(h) - dz * math.sin(h)
            p = self._project(ahead, lateral)
            if p:
                pts.append(p)
        if len(pts) >= 2:
            # Glow + core line, like ETS2LA's painted route.
            qp.setPen(QPen(QColor(59, 130, 246, 90), 16))
            qp.drawPolyline(QPolygonF(pts))
            qp.setPen(QPen(QColor(59, 130, 246, 230), 6))
            qp.drawPolyline(QPolygonF(pts))


def run_ar(shared_state):
    app = QApplication.instance() or QApplication(sys.argv)
    ov = AROverlay(shared_state)
    ov.show()
    if not QApplication.instance().startingUp():
        return ov
    sys.exit(app.exec())
