import sys
import math
import logging
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QPainter, QColor, QFont, QPen, QPolygonF
from PyQt6.QtCore import Qt, QTimer, QRectF, QPointF

# State → accent colour (left as-is; the whole HUD now lives on the left panel).
_STATE_COLORS = {
    "EMERGENCY": "#EF4444", "AVOID_OBSTACLE": "#F59E0B", "OVERTAKING": "#F59E0B",
    "PAY_TOLL": "#EAB308", "FOLLOW_LANE": "#10B981", "CRUISE": "#10B981",
    "IDLE": "#9CA3AF",
}


def _gear_text(gear):
    if not gear:
        return "N"
    return str(int(gear)) if gear > 0 else "R"


class UltraPilotHUD(QWidget):
    """Left-side driving-view HUD (per the reference photo).

    A single compact panel anchored to the bottom-left of the screen containing:
      • a 3D perspective driving scene — road ribbon, edge/centre lines, the
        anticipated route, and the surrounding traffic rendered as solid 3D models,
      • a traffic-light with a live colour countdown,
      • a compact speed / limit / gear readout along the top,
      • a thin throttle/brake strip on the panel's left edge.

    The whole thing is transparent and click-draggable so it sits over the game
    without blocking the view of the road ahead.
    """

    # Compact left-panel size, matching the reference photo (left side only).
    W, H = 360, 470

    def __init__(self, shared_state):
        super().__init__()
        self.shared_state = shared_state
        self._blink = True
        self._drag = None
        self.init_ui()

    def init_ui(self):
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint |
                            Qt.WindowType.WindowStaysOnTopHint | Qt.WindowType.Tool)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.resize(self.W, self.H)
        screen = QApplication.primaryScreen().geometry()
        # Bottom-left of the screen (the reference HUD is on the left side).
        self.move(24, screen.height() - self.H - 24)
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
            # Rear-view camera: shown when a turn signal is active (left/right)
            # OR when the dedicated rear_cam flag is set. Gives a glance behind
            # during lane changes, like a real side/rear mirror inset.
            "blinker": (s.get("active_blinker") or "off"),
            "rear_cam": bool(s.get("rear_cam", False)),
        }

    # --- Painting -------------------------------------------------------------
    def paintEvent(self, event):
        # In PyQt6 an exception escaping paintEvent aborts the whole process
        # (exit code 0xC0000409). Catch + log so the HUD can never crash-loop.
        try:
            self._do_paint(event)
        except Exception as e:
            logging.error("HUD paint error: %s", e)

    def _do_paint(self, event):
        d = self._read()
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 1) Panel background: a dark rounded card (the left HUD from the photo).
        panel = QRectF(0, 0, self.W, self.H)
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(10, 13, 18, 205))
        qp.drawRoundedRect(panel, 16, 16)
        # Subtle accent border tinted by the current driving state.
        accent = QColor(_STATE_COLORS.get(d["state"], "#10B981"))
        qp.setPen(QPen(QColor(accent.red(), accent.green(), accent.blue(), 90), 1))
        qp.setBrush(Qt.BrushStyle.NoBrush)
        qp.drawRoundedRect(panel.adjusted(0.5, 0.5, -0.5, -0.5), 16, 16)

        # 2) Compact top readout: speed (big) + KM/H, limit sign, gear chip.
        self._draw_top_bar(qp, d)

        # 3) The 3D driving scene fills the rest of the panel.
        scene = QRectF(8, 86, self.W - 16, self.H - 94)
        qp.save(); qp.setClipRect(scene)
        self._draw_driving_view(qp, scene, d)
        qp.restore()

        # 4) Traffic light + countdown inside the scene (top-right of the scene).
        if d["light"]:
            self._draw_light(qp, scene, d["light"])

        # 5) Throttle/brake strip on the panel's far-left edge.
        self._draw_pedals(qp, d)

        # 6) Autopilot status pill at the bottom of the panel.
        self._draw_status_pill(qp, d)

        # 7) Rear-view camera inset (bottom-right of the panel). Shows a glance
        #    behind whenever a turn signal is on or the rear_cam flag is set.
        self._draw_rear_cam(qp, d)

    # --- Top bar --------------------------------------------------------------
    def _draw_top_bar(self, qp, d):
        # Current speed (big white).
        qp.setPen(QColor("#FFFFFF"))
        qp.setFont(QFont("Segoe UI", 46, QFont.Weight.Bold))
        qp.drawText(QRectF(16, 10, 170, 64), Qt.AlignmentFlag.AlignVCenter,
                    f"{d['speed_kmh']:.0f}")
        qp.setPen(QColor(255, 255, 255, 150))
        qp.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        qp.drawText(QRectF(18, 66, 120, 16), Qt.AlignmentFlag.AlignLeft, "KM/H")

        # Speed-limit sign (top-centre).
        if d["limit_ms"] and d["limit_ms"] > 1:
            self._draw_limit_sign(qp, self.W - 110, 16, d["limit_ms"] * 3.6)

        # Gear chip (top-right).
        qp.setBrush(QColor(40, 44, 52, 220))
        qp.setPen(QPen(QColor(90, 96, 104, 200), 1))
        qp.drawRoundedRect(QRectF(self.W - 52, 16, 36, 36), 8, 8)
        qp.setPen(QColor("#FFFFFF"))
        qp.setFont(QFont("Segoe UI", 16, QFont.Weight.Bold))
        qp.drawText(QRectF(self.W - 52, 17, 36, 34), Qt.AlignmentFlag.AlignCenter,
                    _gear_text(d["gear"]))

    def _draw_limit_sign(self, qp, x, y, kmh):
        sz = 40
        qp.setBrush(QColor("#FFFFFF")); qp.setPen(QPen(QColor("#EF4444"), 4))
        qp.drawEllipse(QRectF(x, y, sz, sz))
        qp.setPen(QColor("#111827"))
        qp.setFont(QFont("Segoe UI", 13, QFont.Weight.Bold))
        qp.drawText(QRectF(x, y, sz, sz), Qt.AlignmentFlag.AlignCenter, f"{kmh:.0f}")

    # --- Driving scene --------------------------------------------------------
    def _project(self, ahead, lateral, view, height=0.0):
        """Ground-plane perspective projection (chase-cam looking forward)."""
        H = 8.0           # camera height above road
        cam_back = 14.0   # camera behind the truck
        f = view.height() * 1.05
        horizon = view.top() + view.height() * 0.26
        d = ahead + cam_back
        if d < 1.6:
            return None
        s = f / d
        return QPointF(view.center().x() + lateral * s, horizon + (H - height) * s)

    def _draw_driving_view(self, qp, view, d):
        # Slightly darker ground band below the horizon for depth.
        horizon_y = view.top() + view.height() * 0.26
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(20, 25, 32, 160))
        qp.drawRect(QRectF(view.left(), horizon_y, view.width(), view.bottom() - horizon_y))

        pos, h = d["pos"], d["heading"]

        def to_truck(wx, wz):
            dx, dz = wx - pos[0], wz - pos[1]
            ahead = dx * (-math.sin(h)) + dz * (-math.cos(h))
            lateral = dx * math.cos(h) - dz * math.sin(h)
            return ahead, lateral

        if pos:
            path = d["nav_path"]
            al = [to_truck(px, pz) for px, pz in path]

            def offset_pt(i, off):
                a, l = al[i]
                j = min(i + 1, len(al) - 1)
                da, dl = al[j][0] - a, al[j][1] - l
                n = math.hypot(da, dl) or 1.0
                return self._project(a, l + (-da / n) * off, view)

            if len(al) >= 2:
                HALF = 6.5  # half road width (m)
                # 1) Filled asphalt ribbon (left edge → right edge), curving.
                left = [offset_pt(i, -HALF) for i in range(len(al))]
                right = [offset_pt(i, HALF) for i in range(len(al))]
                ribbon = [p for p in left if p] + [p for p in reversed(right) if p]
                if len(ribbon) >= 3:
                    qp.setPen(Qt.PenStyle.NoPen)
                    qp.setBrush(QColor(36, 40, 46, 235))
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
                    qp.setPen(QPen(QColor(59, 130, 246, 80), 10))
                    qp.drawPolyline(QPolygonF(pts))
                    qp.setPen(QPen(QColor("#3B82F6"), 4))
                    qp.drawPolyline(QPolygonF(pts))
            else:
                # No route: a straight filled ribbon ahead so the road is visible.
                left = [self._project(a, -6.5, view) for a in range(2, 80, 6)]
                right = [self._project(a, 6.5, view) for a in range(2, 80, 6)]
                ribbon = [p for p in left if p] + [p for p in reversed(right) if p]
                if len(ribbon) >= 3:
                    qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor(36, 40, 46, 220))
                    qp.drawPolygon(QPolygonF(ribbon))
                for lat, dash in ((-6.5, False), (6.5, False), (0.0, True)):
                    pts = [self._project(a, lat, view) for a in range(2, 80, 6)]
                    pts = [p for p in pts if p is not None]
                    if len(pts) >= 2:
                        st = Qt.PenStyle.DashLine if dash else Qt.PenStyle.SolidLine
                        qp.setPen(QPen(QColor(240, 240, 245, 190), 2, st))
                        qp.drawPolyline(QPolygonF(pts))

            # Surrounding vehicles as solid 3D models (far → near for overlap).
            vehs = []
            for v in d["traffic"]:
                a, l = to_truck(v["x"], v["z"])
                if -6 < a < 70 and abs(l) < 18:
                    vehs.append((a, l, v))
            vehs.sort(key=lambda t: -t[0])
            for a, l, v in vehs:
                self._draw_box(qp, view, a, l, v)

        # Ego truck as a 3D model at the bottom centre (cab + trailer).
        self._draw_box(qp, view, 6.0, 0.0,
                       {"type": "truck", "width": 2.6, "length": 14.0, "yaw": d["heading"]})

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
        hw = max(0.9, v.get("width", 2.0) / 2)
        ln = max(3.5, v.get("length", 4.5))
        n, fr = ahead - ln / 2, ahead + ln / 2
        if not self._box3d(qp, n, fr, hw, lateral, 0.0, body_h, view, ("#8A9099", "#AEB4BC")):
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

    # --- Traffic light --------------------------------------------------------
    def _draw_light(self, qp, view, light):
        """3-bulb signal + countdown, anchored top-right of the scene."""
        color = light.get("color", "off")
        cx, cy = view.right() - 70, view.top() + 14
        # Housing.
        qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor(20, 24, 30, 235))
        qp.drawRoundedRect(QRectF(cx, cy, 24, 58), 6, 6)
        for i, (cname, on_c) in enumerate((("red", "#EF4444"),
                                           ("yellow", "#FBBF24"),
                                           ("green", "#22C55E"))):
            lit = (color == cname)
            qp.setBrush(QColor(on_c) if lit else QColor(55, 60, 66))
            qp.drawEllipse(QRectF(cx + 4, cy + 4 + i * 17, 16, 16))
        # Countdown + next-state hint to the right of the housing.
        tl = light.get("time_left", 0) or 0
        if tl > 0:
            qp.setPen(QColor("#FFFFFF")); qp.setFont(QFont("Segoe UI", 15, QFont.Weight.Bold))
            qp.drawText(QRectF(cx + 28, cy + 4, 60, 22), Qt.AlignmentFlag.AlignLeft,
                        f"{tl:.1f}s")
            nxt_col = {"red": "#F87171", "green": "#4ADE80", "yellow": "#FBBF24"}.get(color, "#D1D5DB")
            nxt_txt = {"red": "→ ZELENÁ", "green": "→ ŽLTÁ", "yellow": "→ ČERVENÁ"}.get(color, "")
            qp.setPen(QColor(nxt_col)); qp.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
            qp.drawText(QRectF(cx + 28, cy + 26, 80, 14), Qt.AlignmentFlag.AlignLeft, nxt_txt)

    # --- Pedals + status ------------------------------------------------------
    def _draw_pedals(self, qp, d):
        """Thin throttle (green) / brake (red) strip on the panel's left edge."""
        t = max(0.0, min(1.0, d["throttle"]))
        b = max(0.0, min(1.0, d["brake"]))
        top = 90; bot = self.H - 40
        midy = (top + bot) / 2
        half = (bot - top) / 2 - 6
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(34, 197, 94, 230))
        qp.drawRect(QRectF(3, midy - t * half, 5, t * half))
        qp.setBrush(QColor(239, 68, 68, 230))
        qp.drawRect(QRectF(3, midy, 5, b * half))

    def _draw_status_pill(self, qp, d):
        on = d["active"]
        pill = QRectF(self.W / 2 - 60, self.H - 30, 120, 22)
        qp.setBrush(QColor("#10B981") if on else QColor(120, 120, 128, 220))
        qp.setPen(Qt.PenStyle.NoPen)
        qp.drawRoundedRect(pill, 11, 11)
        qp.setPen(QColor("#FFFFFF")); qp.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        qp.drawText(pill, Qt.AlignmentFlag.AlignCenter, "AUTOPILOT" if on else "MANUÁL")

    # --- Rear-view camera inset ----------------------------------------------
    def _draw_rear_cam(self, qp, d):
        """A small rearward driving-scene inset in the bottom-right corner.

        Shows only when a turn signal is active (or the explicit rear_cam flag
        is set), so the driver gets a glance behind during lane changes / turns
        — like a side mirror lighting up with the indicator. Renders the road
        behind the truck and any following vehicles as 3D models, mirrored so it
        reads naturally (the truck is at the TOP of the inset, the road recedes
        downward)."""
        show = d.get("rear_cam") or d.get("blinker") in ("left", "right")
        if not show:
            return
        pos, h = d["pos"], d["heading"]
        if not pos:
            return

        cw, ch = 168, 120
        cam = QRectF(self.W - cw - 12, self.H - ch - 12, cw, ch)
        # Housing with a subtle border in the active indicator colour.
        qp.setPen(QPen(QColor("#10B981"), 1))
        qp.setBrush(QColor(6, 9, 13, 235))
        qp.drawRoundedRect(cam, 10, 10)
        qp.save(); qp.setClipRect(cam)

        # Rearward projection: invert "ahead" so +behind maps toward the bottom.
        # We reuse the forward projection but flip the sign of the ahead axis.
        horizon = cam.top() + cam.height() * 0.20

        def proj_back(behind, lateral, height=0.0):
            cam_h = 7.0; cam_back = 10.0
            f = cam.height() * 0.95
            dist = behind + cam_back
            if dist < 1.5:
                return None
            s = f / dist
            return QPointF(cam.center().x() + lateral * s,
                           cam.bottom() - (cam_h - height) * s + 6)

        # Road ribbon behind: straight, narrowing toward the top of the inset.
        HALF = 6.5
        behinds = list(range(2, 70, 6))
        left = [p for p in [proj_back(a, -HALF) for a in behinds] if p]
        right = [p for p in [proj_back(a, HALF) for a in behinds] if p]
        ribbon = left + list(reversed(right))
        if len(ribbon) >= 3:
            qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor(32, 36, 42, 240))
            qp.drawPolygon(QPolygonF(ribbon))
        for lat, dash in ((-HALF, False), (HALF, False), (0.0, True)):
            pts = [p for p in [proj_back(a, lat) for a in behinds] if p]
            if len(pts) >= 2:
                st = Qt.PenStyle.DashLine if dash else Qt.PenStyle.SolidLine
                qp.setPen(QPen(QColor(240, 240, 245, 180), 1, st))
                qp.drawPolyline(QPolygonF(pts))

        # Following vehicles: those BEHIND us (negative ahead in truck frame),
        # drawn as small models receding toward the top of the inset.
        def to_truck(wx, wz):
            dx, dz = wx - pos[0], wz - pos[1]
            ahead = dx * (-math.sin(h)) + dz * (-math.cos(h))
            lateral = dx * math.cos(h) - dz * math.sin(h)
            return ahead, lateral

        vehs = []
        for v in d["traffic"]:
            a, l = to_truck(v["x"], v["z"])
            behind = -a                       # behind the truck = positive
            if 2 < behind < 60 and abs(l) < 16:
                vehs.append((behind, l, v))
        vehs.sort(key=lambda t: t[0])         # nearest-behind first (drawn last)
        for behind, l, v in vehs:
            self._draw_box_back(qp, cam, behind, l, v, proj_back)

        # "REAR" label + the active indicator side, so it's obvious what's lit.
        qp.setPen(QColor("#10B981")); qp.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        side_txt = {"left": "◀ ĽAVÁ", "right": "PRAVÁ ▶"}.get(d.get("blinker"), "")
        qp.drawText(QRectF(cam.left() + 6, cam.top() + 3, cw - 12, 14),
                    Qt.AlignmentFlag.AlignLeft, f"◉ ZADNÁ KAMERA {side_txt}")
        qp.restore()

    def _draw_box_back(self, qp, view, behind, lateral, v, proj):
        """A simplified vehicle model for the rear-cam inset (just a shaded box)."""
        t = v.get("type", "car")
        body_h = {"car": 1.1, "van": 1.7, "bus": 2.8, "truck": 2.6}.get(t, 1.2)
        hw = max(0.9, v.get("width", 2.0) / 2)
        ln = max(3.5, v.get("length", 4.5))
        n, fr = behind - ln / 2, behind + ln / 2
        # 8 corners of the cuboid in rear-cam space.
        c = [proj(n, lateral - hw, 0.0), proj(n, lateral + hw, 0.0),
             proj(fr, lateral - hw, 0.0), proj(fr, lateral + hw, 0.0),
             proj(n, lateral - hw, body_h), proj(n, lateral + hw, body_h),
             proj(fr, lateral - hw, body_h), proj(fr, lateral + hw, body_h)]
        if any(p is None for p in c):
            return
        bl, br, fl, fr_, blt, brt, flt, frt = c
        qp.setPen(QPen(QColor("#34393F"), 1))
        qp.setBrush(QColor("#8A9099").darker(110)); qp.drawPolygon(QPolygonF([bl, br, brt, blt]))
        qp.setBrush(QColor("#AEB4BC")); qp.drawPolygon(QPolygonF([bl, fl, flt, blt]))
        qp.drawPolygon(QPolygonF([br, fr_, frt, brt]))

    # --- Dragging -------------------------------------------------------------
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag = event.globalPosition().toPoint()

    def mouseMoveEvent(self, event):
        if self._drag is not None:
            delta = event.globalPosition().toPoint() - self._drag
            self.move(self.x() + delta.x(), self.y() + delta.y())
            self._drag = event.globalPosition().toPoint()

    def mouseReleaseEvent(self, event):
        self._drag = None


def run_hud(shared_state):
    app = QApplication(sys.argv)
    hud = UltraPilotHUD(shared_state)
    hud.show()
    sys.exit(app.exec())
