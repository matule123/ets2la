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

    W, H = 860, 520       # large immersive scene, anchored bottom-centre
    VIEW_M = 110.0         # metres shown ahead in the driving view

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
            # The route to draw: active navigation, else the map road ahead.
            "nav_path": (s.get("nav_path", []) or s.get("map_path", []) or []),
            "limit_ms": truck.get("speedLimit", 0.0) or 0.0,
            "acc_speed": s.get("tags.acc.acc_speed"),
            "nav_dist": s.get("distance_to_dest"),
        }

    # --- Painting -------------------------------------------------------------
    def paintEvent(self, event):
        # In PyQt6 an exception escaping paintEvent aborts the whole process
        # (exit code 0xC0000409). Catch + log so the HUD can never crash-loop.
        try:
            self._do_paint(event)
        except Exception as e:
            import logging
            logging.error("HUD paint error: %s", e)

    def _do_paint(self, event):
        d = self._read()
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        accent = QColor(_STATE_COLORS.get(d["state"], "#10B981"))

        # 1) Full-bleed 3D driving scene as the whole background (like ETS2LA).
        scene = QRectF(0, 0, self.W, self.H)
        self._draw_driving_view(qp, scene, d, accent)

        # 1b) Dark top panel so the overlaid numbers stay fully readable
        #     (fixes "half the numbers cut off").
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(8, 11, 15, 190))
        qp.drawRect(QRectF(0, 0, self.W, 132))

        # 2) Throttle (green) + brake (red) strips on the far-left edge.
        t = max(0.0, min(1.0, d["throttle"]))
        b = max(0.0, min(1.0, d["brake"]))
        midy = self.H / 2
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(34, 197, 94, 230))
        qp.drawRect(QRectF(0, midy - t * (midy - 60), 7, t * (midy - 60)))
        qp.setBrush(QColor(239, 68, 68, 230))
        qp.drawRect(QRectF(0, midy, 7, b * (midy - 60)))

        # 3) Current speed (big white, top-left) + KM/H — full height panel.
        qp.setPen(QColor("#FFFFFF"))
        qp.setFont(QFont("Segoe UI", 60, QFont.Weight.Bold))
        qp.drawText(QRectF(18, 14, 240, 86), Qt.AlignmentFlag.AlignVCenter, f"{d['speed_kmh']:.0f}")
        qp.setPen(QColor(255, 255, 255, 160))
        qp.setFont(QFont("Segoe UI", 12, QFont.Weight.Bold))
        qp.drawText(QRectF(22, 100, 120, 18), Qt.AlignmentFlag.AlignLeft, "KM/H")

        # 4) Speed-limit sign (red circle, top-centre).
        if d["limit_ms"] and d["limit_ms"] > 1:
            self._draw_limit_sign(qp, self.W / 2 - 26, 14, d["limit_ms"] * 3.6, big=True)

        # 5) Cruise-control target (teal, under the sign).
        if d["acc_speed"] is not None:
            try:
                qp.setPen(QColor("#22D3EE"))
                qp.setFont(QFont("Segoe UI", 26, QFont.Weight.Bold))
                qp.drawText(QRectF(self.W / 2 + 18, 70, 110, 34),
                            Qt.AlignmentFlag.AlignLeft, f"{float(d['acc_speed']):.0f}")
                qp.setPen(QColor(34, 211, 238, 170))
                qp.setFont(QFont("Segoe UI", 9))
                qp.drawText(QRectF(self.W / 2 + 20, 100, 80, 16),
                            Qt.AlignmentFlag.AlignLeft, "auto" if d["active"] else "set")
            except (TypeError, ValueError):
                pass

        # 6) Direction / gear chip (top-right).
        qp.setBrush(QColor(40, 44, 52, 220)); qp.setPen(QPen(QColor(90, 96, 104, 200), 1))
        qp.drawRoundedRect(QRectF(self.W - 64, 14, 48, 48), 8, 8)
        qp.setPen(QColor("#FFFFFF")); qp.setFont(QFont("Segoe UI", 20, QFont.Weight.Bold))
        qp.drawText(QRectF(self.W - 64, 16, 48, 44), Qt.AlignmentFlag.AlignCenter, _gear_text(d["gear"]))

        # 7) Autopilot status pill (top-right, under the chip).
        on = d["active"]
        pill = QRectF(self.W - 116, 70, 100, 22)
        qp.setBrush(QColor("#10B981") if on else QColor(120, 120, 128, 220))
        qp.setPen(Qt.PenStyle.NoPen)
        qp.drawRoundedRect(pill, 11, 11)
        qp.setPen(QColor("#FFFFFF")); qp.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        qp.drawText(pill, Qt.AlignmentFlag.AlignCenter, "AUTOPILOT" if on else "MANUAL")

        # 8) Traffic light: a 3-bulb signal + countdown in the scene.
        lt = d["light"]
        if lt:
            color = lt.get("color", "off")
            cx, cy = self.W / 2 + 70, self.H * 0.34
            # housing
            qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor(20, 24, 30, 230))
            qp.drawRoundedRect(QRectF(cx, cy, 26, 64), 6, 6)
            for i, (cname, on_c) in enumerate((("red", "#EF4444"),
                                               ("yellow", "#FBBF24"),
                                               ("green", "#22C55E"))):
                lit = (color == cname)
                qp.setBrush(QColor(on_c) if lit else QColor(55, 60, 66))
                qp.drawEllipse(QRectF(cx + 5, cy + 5 + i * 19, 16, 16))
            tl = lt.get("time_left", 0) or 0
            if tl > 0:
                col = {"red": "#F87171", "green": "#4ADE80", "yellow": "#FBBF24"}.get(color, "#D1D5DB")
                qp.setPen(QColor("#FFFFFF")); qp.setFont(QFont("Segoe UI", 16, QFont.Weight.Bold))
                qp.drawText(QRectF(cx + 32, cy + 8, 90, 24), Qt.AlignmentFlag.AlignLeft, f"{tl:.1f}s")
                qp.setPen(QColor(col)); qp.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
                nxt = {"red": "RED → GREEN", "green": "GREEN → YELLOW",
                       "yellow": "YELLOW"}.get(color, "")
                qp.drawText(QRectF(cx + 32, cy + 32, 110, 14), Qt.AlignmentFlag.AlignLeft, nxt)

    # --- Sub-widgets ----------------------------------------------------------
    def _draw_limit_sign(self, qp, x, y, kmh, big=False):
        sz = 52 if big else 34
        ring = 5 if big else 3
        qp.setBrush(QColor("#FFFFFF")); qp.setPen(QPen(QColor("#EF4444"), ring))
        qp.drawEllipse(QRectF(x, y, sz, sz))
        qp.setPen(QColor("#111827"))
        qp.setFont(QFont("Segoe UI", 17 if big else 11, QFont.Weight.Bold))
        qp.drawText(QRectF(x, y, sz, sz), Qt.AlignmentFlag.AlignCenter, f"{kmh:.0f}")

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
        H = 8.0           # camera height above road
        cam_back = 16.0   # camera further behind the truck (wider view)
        f = view.height() * 1.15
        horizon = view.top() + view.height() * 0.30
        d = ahead + cam_back
        if d < 1.6:
            return None
        s = f / d
        return QPointF(view.center().x() + lateral * s, horizon + (H - height) * s)

    def _draw_driving_view(self, qp, view, d, accent):
        # Semi-transparent 3D scene so the game shows through a little.
        qp.setBrush(QColor(15, 19, 24, 165)); qp.setPen(Qt.PenStyle.NoPen)
        qp.drawRoundedRect(view, 14, 14)
        qp.save(); qp.setClipRect(view)
        horizon_y = view.top() + view.height() * 0.30
        qp.setBrush(QColor(26, 32, 39, 150))
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

            def offset_pt(i, off):
                """Path point i shifted sideways by `off` metres (follows curve)."""
                a, l = al[i]
                j = min(i + 1, len(al) - 1)
                da, dl = al[j][0] - a, al[j][1] - l
                n = math.hypot(da, dl) or 1.0
                return self._project(a, l + (-da / n) * off, view)

            if len(al) >= 2:
                HALF = 7.0   # half road width (m)
                # 1) Filled asphalt ribbon (left edge → right edge), curving.
                left = [offset_pt(i, -HALF) for i in range(len(al))]
                right = [offset_pt(i, HALF) for i in range(len(al))]
                ribbon = [p for p in left if p] + [p for p in reversed(right) if p]
                if len(ribbon) >= 3:
                    qp.setPen(Qt.PenStyle.NoPen)
                    qp.setBrush(QColor(38, 42, 48, 230))
                    qp.drawPolygon(QPolygonF(ribbon))
                # 2) Solid white edge lines + dashed centre line.
                for off, dash in ((-HALF, False), (HALF, False), (0.0, True)):
                    pts = [offset_pt(i, off) for i in range(len(al))]
                    pts = [p for p in pts if p]
                    if len(pts) >= 2:
                        style = Qt.PenStyle.DashLine if dash else Qt.PenStyle.SolidLine
                        qp.setPen(QPen(QColor(240, 240, 245, 200), 2, style))
                        qp.drawPolyline(QPolygonF(pts))
                # 3) Anticipated route (blue) glow on the road.
                pts = [self._project(a, l, view) for a, l in al]
                pts = [p for p in pts if p is not None]
                if len(pts) >= 2:
                    qp.setPen(QPen(QColor(59, 130, 246, 90), 12))
                    qp.drawPolyline(QPolygonF(pts))
                    qp.setPen(QPen(QColor("#3B82F6"), 5))
                    qp.drawPolyline(QPolygonF(pts))
            else:
                # No route: a straight filled ribbon ahead so the road is visible.
                left = [self._project(a, -7, view) for a in range(2, 95, 6)]
                right = [self._project(a, 7, view) for a in range(2, 95, 6)]
                ribbon = [p for p in left if p] + [p for p in reversed(right) if p]
                if len(ribbon) >= 3:
                    qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor(38, 42, 48, 220))
                    qp.drawPolygon(QPolygonF(ribbon))
                for lat, dash in ((-7, False), (7, False), (0, True)):
                    pts = [self._project(a, lat, view) for a in range(2, 95, 6)]
                    pts = [p for p in pts if p is not None]
                    if len(pts) >= 2:
                        st = Qt.PenStyle.DashLine if dash else Qt.PenStyle.SolidLine
                        qp.setPen(QPen(QColor(240, 240, 245, 190), 2, st))
                        qp.drawPolyline(QPolygonF(pts))

            # Surrounding vehicles as solid 3D models (far → near for overlap).
            vehs = []
            for v in d["traffic"]:
                a, l = to_truck(v["x"], v["z"])
                if -6 < a < 84 and abs(l) < 22:
                    vehs.append((a, l, v))
            vehs.sort(key=lambda t: -t[0])
            for a, l, v in vehs:
                self._draw_box(qp, view, a, l, v)

        # Ego truck as a 3D model at the bottom centre (cab + trailer).
        self._draw_box(qp, view, 7.0, 0.0,
                       {"type": "truck", "width": 2.6, "length": 14.0, "yaw": d["heading"]})
        qp.restore()

    def _box3d(self, qp, n, fr, hw, lateral, z0, z1, view, faces):
        """Draw one shaded cuboid between heights z0..z1. faces=(side,top) colors."""
        c = [self._project(n, lateral - hw, view, z0), self._project(n, lateral + hw, view, z0),
             self._project(fr, lateral - hw, view, z0), self._project(fr, lateral + hw, view, z0),
             self._project(n, lateral - hw, view, z1), self._project(n, lateral + hw, view, z1),
             self._project(fr, lateral - hw, view, z1), self._project(fr, lateral + hw, view, z1)]
        if any(p is None for p in c):
            return False
        bl, br, fl, fr_, blt, brt, flt, frt = c
        side, top = faces
        qp.setPen(QPen(QColor("#34393F"), 1))
        qp.setBrush(QColor(side).darker(115)); qp.drawPolygon(QPolygonF([bl, br, brt, blt]))   # back
        qp.setBrush(QColor(side)); qp.drawPolygon(QPolygonF([bl, fl, flt, blt]))               # left
        qp.drawPolygon(QPolygonF([br, fr_, frt, brt]))                                         # right
        qp.setBrush(QColor(side).darker(108)); qp.drawPolygon(QPolygonF([fl, fr_, frt, flt]))  # front
        qp.setBrush(QColor(top)); qp.drawPolygon(QPolygonF([blt, brt, frt, flt]))              # top
        return True

    def _draw_box(self, qp, view, ahead, lateral, v):
        # Vehicle = lower body + a smaller cabin on top → reads as a real model.
        t = v.get("type", "car")
        body_h = {"car": 1.1, "van": 1.7, "bus": 2.8, "truck": 2.6}.get(t, 1.2)
        hw = max(0.9, v["width"] / 2)
        ln = max(3.5, v["length"])
        n, fr = ahead - ln / 2, ahead + ln / 2
        if not self._box3d(qp, n, fr, hw, lateral, 0.0, body_h, view,
                           ("#8A9099", "#AEB4BC")):
            return
        # Cabin / cab on top (cars & vans: middle; trucks: front; bus: full).
        if t == "bus":
            self._box3d(qp, n + 0.3, fr - 0.3, hw * 0.92, lateral, body_h, body_h + 0.7, view,
                        ("#9AA0A8", "#C2C8D0"))
        elif t == "truck":
            self._box3d(qp, fr - ln * 0.28, fr - 0.2, hw * 0.95, lateral, body_h, body_h + 1.0, view,
                        ("#9AA0A8", "#C2C8D0"))
        else:  # car / van cabin in the middle
            self._box3d(qp, n + ln * 0.28, fr - ln * 0.22, hw * 0.9, lateral, body_h, body_h + 0.8, view,
                        ("#9AA0A8", "#C2C8D0"))

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
