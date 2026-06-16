import sys
import math
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QPainter, QColor, QFont, QPen, QPolygonF, QBrush
from PyQt6.QtCore import Qt, QTimer, QRectF, QPointF

_STATE_COLORS = {
    "EMERGENCY": "#EF4444", "AVOID_OBSTACLE": "#F59E0B", "OVERTAKING": "#F59E0B",
    "PAY_TOLL": "#EAB308", "FOLLOW_LANE": "#10B981", "CRUISE": "#10B981",
    "IDLE": "#9CA3AF",
}
_VEH_COLORS = {  # surrounding vehicle colours by type
    "car": "#3B82F6", "van": "#8B5CF6", "bus": "#F59E0B", "truck": "#EF4444",
}


def _gear_text(gear):
    if not gear:
        return "N"
    if gear > 0:
        return str(int(gear))
    return "R"


class UltraPilotHUD(QWidget):
    """Animated driving-view HUD: ego truck, surrounding traffic, route, lights."""

    W, H = 460, 300       # wider driving view, anchored bottom-centre
    VIEW_M = 90.0          # metres shown ahead in the driving view

    def __init__(self, shared_state):
        super().__init__()
        self.shared_state = shared_state
        self._blink = True
        self.init_ui()

    def init_ui(self):
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint |
                            Qt.WindowType.WindowStaysOnTopHint | Qt.WindowType.Tool)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.resize(self.W, self.H)
        screen = QApplication.primaryScreen().geometry()
        # Bottom-centre of the screen (like the ETS2LA layout).
        self.move((screen.width() - self.W) // 2, screen.height() - self.H - 60)
        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(80)   # ~12 fps animation

    def _tick(self):
        self._blink = not self._blink
        self.update()

    # --- Data -----------------------------------------------------------------
    def _read(self):
        s = self.shared_state
        state = s.get("system_state", "IDLE")
        state = state.name if hasattr(state, "name") else str(state)
        speed = s.get("speed", 0) or 0
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            speed = 0.0
        truck = (s.get("telemetry", {}) or {}).get("truck", {}) or {}
        return {
            "state": state,
            "speed_kmh": abs(speed) * 3.6 if abs(speed) < 200 else abs(speed),
            "gear": truck.get("gear", 0),
            "active": bool(s.get("autopilot_active", False)),
            "throttle": float(s.get("ctl_throttle", 0.0) or 0.0),
            "brake": float(s.get("ctl_brake", 0.0) or 0.0),
            "pos": s.get("truck_world_pos"),
            "heading": s.get("truck_heading", 0.0) or 0.0,
            "traffic": s.get("traffic", []) or [],
            "light": s.get("traffic_light"),
            "nav_path": s.get("nav_path", []) or [],
            "limit_ms": truck.get("speedLimit", 0.0) or 0.0,
        }

    # --- Painting -------------------------------------------------------------
    def paintEvent(self, event):
        d = self._read()
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        accent = QColor(_STATE_COLORS.get(d["state"], "#10B981"))

        # Card.
        qp.setBrush(QColor(255, 255, 255, 238))
        qp.setPen(QPen(QColor(0, 0, 0, 28), 1))
        qp.drawRoundedRect(QRectF(1, 1, self.W - 2, self.H - 2), 16, 16)

        # ---- Top bar: speed + autopilot pill ----
        qp.setPen(QColor("#111827"))
        qp.setFont(QFont("Segoe UI", 40, QFont.Weight.Bold))
        qp.drawText(QRectF(18, 12, 150, 56), Qt.AlignmentFlag.AlignLeft, f"{d['speed_kmh']:.0f}")
        qp.setPen(QColor("#6B7280"))
        qp.setFont(QFont("Segoe UI", 10))
        qp.drawText(QRectF(20, 64, 80, 16), Qt.AlignmentFlag.AlignLeft, "km/h")

        on = d["active"]
        pill = QRectF(self.W - 116, 16, 96, 22)
        qp.setBrush(QColor("#10B981") if on else QColor("#9CA3AF"))
        qp.setPen(Qt.PenStyle.NoPen)
        qp.drawRoundedRect(pill, 11, 11)
        qp.setPen(QColor("#FFFFFF"))
        qp.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        qp.drawText(pill, Qt.AlignmentFlag.AlignCenter, "AUTOPILOT" if on else "MANUAL")

        # gear + speed-limit chips
        qp.setPen(accent)
        qp.setFont(QFont("Segoe UI", 13, QFont.Weight.Bold))
        qp.drawText(QRectF(118, 24, 40, 30), Qt.AlignmentFlag.AlignLeft, _gear_text(d["gear"]))
        if d["limit_ms"] and d["limit_ms"] > 1:
            self._draw_limit_sign(qp, self.W - 116, 44, d["limit_ms"] * 3.6)

        # ---- Traffic light widget (top, under pill) ----
        if d["light"]:
            self._draw_traffic_light(qp, self.W - 60, 70, d["light"])

        # ---- Driving view ----
        view = QRectF(14, 96, self.W - 70, self.H - 112)
        self._draw_driving_view(qp, view, d, accent)

        # ---- Throttle / brake vertical bar (right side) ----
        self._draw_pedal_bar(qp, QRectF(self.W - 46, 96, 30, self.H - 112), d)

    # --- Sub-widgets ----------------------------------------------------------
    def _draw_limit_sign(self, qp, x, y, kmh):
        qp.setBrush(QColor("#FFFFFF")); qp.setPen(QPen(QColor("#EF4444"), 3))
        qp.drawEllipse(QRectF(x, y, 34, 34))
        qp.setPen(QColor("#111827")); qp.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        qp.drawText(QRectF(x, y, 34, 34), Qt.AlignmentFlag.AlignCenter, f"{kmh:.0f}")

    def _draw_traffic_light(self, qp, x, y, light):
        col = {"red": "#EF4444", "green": "#22C55E", "yellow": "#F59E0B"}.get(light.get("color"), "#9CA3AF")
        qp.setBrush(QColor("#1F2937")); qp.setPen(Qt.PenStyle.NoPen)
        qp.drawRoundedRect(QRectF(x, y, 22, 50), 5, 5)
        for i, c in enumerate(("#EF4444", "#F59E0B", "#22C55E")):
            active = (light.get("color") == {0: "red", 1: "yellow", 2: "green"}[i])
            qp.setBrush(QColor(c) if active else QColor(60, 60, 64))
            qp.drawEllipse(QRectF(x + 5, y + 4 + i * 15, 12, 12))
        tl = light.get("time_left", 0) or 0
        if tl > 0:
            qp.setPen(QColor(col)); qp.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
            qp.drawText(QRectF(x - 8, y + 52, 38, 14), Qt.AlignmentFlag.AlignCenter, f"{tl:.0f}s")

    def _project(self, ahead, lateral, view, height=0.0):
        """Ground-plane perspective projection (chase-cam looking forward)."""
        H = 6.0          # camera height above road
        cam_back = 8.0   # camera distance behind the truck
        f = view.height() * 1.15
        horizon = view.top() + view.height() * 0.30
        d = ahead + cam_back
        if d < 1.6:
            return None
        s = f / d
        return QPointF(view.center().x() + lateral * s, horizon + (H - height) * s)

    def _draw_driving_view(self, qp, view, d, accent):
        # Dark 3D scene (sky + ground), like the ETS2LA visualization.
        qp.setBrush(QColor("#0F1318")); qp.setPen(Qt.PenStyle.NoPen)
        qp.drawRoundedRect(view, 10, 10)
        qp.save(); qp.setClipRect(view)
        horizon_y = view.top() + view.height() * 0.30
        qp.setBrush(QColor("#1A2027"))
        qp.drawRect(QRectF(view.left(), horizon_y, view.width(), view.bottom() - horizon_y))

        pos, h = d["pos"], d["heading"]

        def to_truck(wx, wz):
            dx, dz = wx - pos[0], wz - pos[1]
            ahead = dx * (-math.sin(h)) + dz * (-math.cos(h))
            lateral = dx * math.cos(h) - dz * math.sin(h)
            return ahead, lateral

        if pos:
            # Convert the route to truck-frame (ahead, lateral) points once.
            path = d["nav_path"]
            al = []
            for px, pz in path:
                al.append(to_truck(px, pz))

            if len(al) >= 2:
                # Lane markings FOLLOW the road curve: offset the path sideways.
                for off in (-6.0, -2.0, 2.0, 6.0):
                    pts = []
                    for i, (a, l) in enumerate(al):
                        # perpendicular direction from the local path heading
                        j = min(i + 1, len(al) - 1)
                        da, dl = al[j][0] - a, al[j][1] - l
                        n = math.hypot(da, dl) or 1.0
                        px_, lateral_ = l + (-da / n) * off, a  # offset lateral, keep ahead
                        p = self._project(a, l + (-da / n) * off, view)
                        if p:
                            pts.append(p)
                    if len(pts) >= 2:
                        qp.setPen(QPen(QColor(255, 255, 255, 55), 1, Qt.PenStyle.DashLine))
                        qp.drawPolyline(QPolygonF(pts))

                # Anticipated route (blue) painted along the curved road.
                pts = [self._project(a, l, view) for a, l in al]
                pts = [p for p in pts if p is not None]
                if len(pts) >= 2:
                    qp.setPen(QPen(QColor(59, 130, 246, 90), 13))
                    qp.drawPolyline(QPolygonF(pts))
                    qp.setPen(QPen(QColor("#3B82F6"), 6))
                    qp.drawPolyline(QPolygonF(pts))
            else:
                # No route yet: straight guide lines.
                for lat in (-6, -2, 2, 6):
                    pts = [self._project(a, lat, view) for a in range(2, 90, 4)]
                    pts = [p for p in pts if p is not None]
                    if len(pts) >= 2:
                        qp.setPen(QPen(QColor(255, 255, 255, 45), 1, Qt.PenStyle.DashLine))
                        qp.drawPolyline(QPolygonF(pts))

            # Surrounding vehicles as grey 3D boxes (far → near for overlap).
            vehs = []
            for v in d["traffic"]:
                a, l = to_truck(v["x"], v["z"])
                if -6 < a < 84 and abs(l) < 22:
                    vehs.append((a, l, v))
            vehs.sort(key=lambda t: -t[0])
            for a, l, v in vehs:
                self._draw_box(qp, view, a, l, v)

        # Ego truck marker at the bottom.
        ex, ey = view.center().x(), view.bottom() - 24
        qp.setBrush(QColor(accent)); qp.setPen(QPen(QColor("#065F46"), 1))
        qp.drawRoundedRect(QRectF(ex - 16, ey - 26, 32, 44), 6, 6)
        qp.restore()

    def _draw_box(self, qp, view, ahead, lateral, v):
        hgt = {"car": 1.5, "van": 2.3, "bus": 3.0, "truck": 3.2}.get(v["type"], 1.6)
        hw = max(0.9, v["width"] / 2)
        n, fr = ahead - v["length"] / 2, ahead + v["length"] / 2
        c = [self._project(n, lateral - hw, view), self._project(n, lateral + hw, view),
             self._project(fr, lateral - hw, view), self._project(fr, lateral + hw, view),
             self._project(n, lateral - hw, view, hgt), self._project(n, lateral + hw, view, hgt),
             self._project(fr, lateral - hw, view, hgt), self._project(fr, lateral + hw, view, hgt)]
        if any(p is None for p in c):
            return
        bl, br, fl, fr_, blt, brt, flt, frt = c
        qp.setPen(QPen(QColor("#3A4049"), 1))
        qp.setBrush(QColor("#6B7280")); qp.drawPolygon(QPolygonF([bl, br, brt, blt]))   # back
        qp.setBrush(QColor("#9CA3AF")); qp.drawPolygon(QPolygonF([bl, fl, flt, blt]))   # left
        qp.drawPolygon(QPolygonF([br, fr_, frt, brt]))                                  # right
        qp.setBrush(QColor("#7C828B")); qp.drawPolygon(QPolygonF([fl, fr_, frt, flt]))  # front
        qp.setBrush(QColor("#B6BCC4")); qp.drawPolygon(QPolygonF([blt, brt, frt, flt])) # top

    def _draw_vehicle(self, qp, center, v, ego_h, scale):
        # All grey; the silhouette tells car / van / bus / truck apart.
        ln = max(7, v["length"] * scale)
        wd = max(5, v["width"] * scale)
        rel = v["yaw"] - ego_h
        body = QColor("#9CA3AF")
        dark = QColor("#6B7280")
        win = QColor("#D1D5DB")
        qp.save()
        qp.translate(center)
        qp.rotate(-math.degrees(rel))
        qp.setPen(QPen(QColor("#4B5563"), 1))
        t = v["type"]
        if t == "truck":
            qp.setBrush(dark)
            qp.drawRoundedRect(QRectF(-wd / 2, ln / 2 - wd, wd, wd), 2, 2)        # cab
            qp.setBrush(body)
            qp.drawRoundedRect(QRectF(-wd / 2, -ln / 2, wd, ln - wd - 1), 2, 2)   # trailer
        elif t == "bus":
            qp.setBrush(body)
            qp.drawRoundedRect(QRectF(-wd / 2, -ln / 2, wd, ln), 3, 3)
            qp.setBrush(win)
            qp.drawRoundedRect(QRectF(-wd / 2 + 1.5, -ln / 2 + 2, wd - 3, ln - 4), 2, 2)
        elif t == "van":
            qp.setBrush(body)
            qp.drawRoundedRect(QRectF(-wd / 2, -ln / 2, wd, ln), 3, 3)
            qp.setBrush(dark)
            qp.drawRoundedRect(QRectF(-wd / 2, ln / 2 - wd * 0.8, wd, wd * 0.8), 2, 2)  # cab
        else:  # car
            qp.setBrush(body)
            qp.drawRoundedRect(QRectF(-wd / 2, -ln / 2, wd, ln), 4, 4)
            qp.setBrush(dark)
            qp.drawRoundedRect(QRectF(-wd / 2 + 1.5, -ln / 4, wd - 3, ln / 2), 2, 2)  # cabin
        qp.restore()

    def _draw_pedal_bar(self, qp, r, d):
        mid = r.center().y()
        qp.setBrush(QColor("#F3F4F6")); qp.setPen(QPen(QColor("#E5E7EB"), 1))
        qp.drawRoundedRect(r, 6, 6)
        qp.setPen(QPen(QColor("#9CA3AF"), 1))
        qp.drawLine(QPointF(r.left(), mid), QPointF(r.right(), mid))
        half = (mid - r.top())
        # throttle up (green), brake down (red)
        t = max(0.0, min(1.0, d["throttle"]))
        b = max(0.0, min(1.0, d["brake"]))
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor("#22C55E"))
        qp.drawRoundedRect(QRectF(r.left() + 4, mid - t * half, r.width() - 8, t * half), 3, 3)
        qp.setBrush(QColor("#EF4444"))
        qp.drawRoundedRect(QRectF(r.left() + 4, mid, r.width() - 8, b * half), 3, 3)
        qp.setPen(QColor("#6B7280")); qp.setFont(QFont("Segoe UI", 7, QFont.Weight.Bold))
        qp.drawText(QRectF(r.left() - 2, r.top() - 14, r.width() + 4, 12),
                    Qt.AlignmentFlag.AlignCenter, "GAS")
        qp.drawText(QRectF(r.left() - 2, r.bottom() + 2, r.width() + 4, 12),
                    Qt.AlignmentFlag.AlignCenter, "BRK")

    # --- Dragging -------------------------------------------------------------
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.old_pos = event.globalPosition().toPoint()

    def mouseMoveEvent(self, event):
        if hasattr(self, "old_pos"):
            delta = event.globalPosition().toPoint() - self.old_pos
            self.move(self.x() + delta.x(), self.y() + delta.y())
            self.old_pos = event.globalPosition().toPoint()


def run_hud(shared_state):
    app = QApplication(sys.argv)
    hud = UltraPilotHUD(shared_state)
    hud.show()
    sys.exit(app.exec())
