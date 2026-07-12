import time
import math
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PyQt6.QtGui import QPainter, QColor, QFont, QPen, QPolygonF
from PyQt6.QtCore import Qt, QTimer, QRectF, QPointF


class _TopDown(QWidget):
    """Top-down (map-style) view: road ahead + traffic around the truck."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        self.setMinimumSize(280, 280)

    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        # Background matches the app's dark palette (#161B22).
        qp.setBrush(QColor(22, 27, 34)); qp.setPen(Qt.PenStyle.NoPen)
        qp.drawRoundedRect(QRectF(0, 0, w, h), 12, 12)
        # Subtle inner border for depth.
        qp.setPen(QPen(QColor("#30363D"), 1)); qp.setBrush(Qt.BrushStyle.NoBrush)
        qp.drawRoundedRect(QRectF(1, 1, w - 2, h - 2), 11, 11)

        pos = self.state.get("truck_world_pos")
        if not pos:
            qp.setPen(QColor("#8B95A5"))
            qp.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Top-down mapa\n(potrebuje telemetriu)")
            return
        head = self.state.get("truck_heading", 0.0) or 0.0
        scale = (min(w, h) - 40) / 200.0   # show ~200 m around the truck
        cx, cy = w / 2, h * 0.62           # truck a bit below centre (see ahead)
        sin_h, cos_h = math.sin(head), math.cos(head)

        def to_screen(wx, wz):
            dx, dz = wx - pos[0], wz - pos[1]
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lat = dx * cos_h - dz * sin_h
            return QPointF(cx + lat * scale, cy - ahead * scale)

        # Road ahead: wide faint glow under a brighter line for a neon feel.
        path = self.state.get("nav_path", []) or self.state.get("map_path", []) or []
        if len(path) >= 2:
            pts = [to_screen(px, pz) for px, pz in path]
            road_w = max(6, int(7 * scale))
            qp.setPen(QPen(QColor(37, 99, 235, 70), road_w + 6, Qt.PenStyle.SolidLine,
                           Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            qp.drawPolyline(QPolygonF(pts))
            qp.setPen(QPen(QColor("#3B82F6"), road_w, Qt.PenStyle.SolidLine,
                           Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            qp.drawPolyline(QPolygonF(pts))

        # Surrounding traffic (amber dots, a touch bigger).
        qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor("#F59E0B"))
        for v in (self.state.get("traffic", []) or []):
            p = to_screen(v["x"], v["z"])
            if 0 <= p.x() <= w and 0 <= p.y() <= h:
                qp.drawEllipse(p, 5, 5)

        # Ego truck (green arrow, always pointing up).
        qp.setBrush(QColor("#34D399")); qp.setPen(QPen(QColor("#10B981"), 1))
        qp.drawPolygon(QPolygonF([QPointF(cx, cy - 13), QPointF(cx - 9, cy + 9),
                                  QPointF(cx + 9, cy + 9)]))


class _GlassIsland(QWidget):
    """Frosted 'liquid glass' island showing ETA + remaining distance."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        from core.theme import palette
        self._pal = palette(state.get("ui_theme", "light") or "light")
        self.setMinimumHeight(170)

    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        pal = self._pal

        # Pull live data.
        dist_m = self.state.get("distance_to_dest")
        speed = self.state.get("truck_speed_ms", 0.0) or 0.0
        nav = bool(self.state.get("nav_active"))

        # Layered translucent rounded rects = frosted-glass look. Use the
        # palette's card colour as the base so the island matches the theme
        # (was hardcoded white — wrong in dark mode).
        island = QRectF(w / 2 - 230, h / 2 - 70, 460, 140)
        base = QColor(pal['card'])
        for i, a in enumerate((60, 110, 235)):
            c = QColor(base.red(), base.green(), base.blue(), a)
            qp.setBrush(c)
            qp.setPen(QPen(QColor(base.red(), base.green(), base.blue(), 90), 1))
            qp.drawRoundedRect(island.adjusted(-i * 4, -i * 4, i * 4, i * 4), 34, 34)
        accent = QColor(pal['title'])
        accent.setAlpha(140)
        qp.setPen(QPen(accent, 2))
        qp.setBrush(Qt.BrushStyle.NoBrush)
        qp.drawRoundedRect(island, 34, 34)

        if not nav or dist_m is None:
            qp.setPen(QColor(pal['muted']))
            qp.setFont(QFont("Segoe UI", 14))
            qp.drawText(island, Qt.AlignmentFlag.AlignCenter,
                        "No active navigation.\nLoad a route or a map.")
            return

        dist_km = float(dist_m) / 1000.0
        # Remaining time + ETA from current speed.
        if speed > 1.0:
            secs = float(dist_m) / speed
            eta = time.localtime(time.time() + secs)
            eta_txt = time.strftime("%H:%M", eta)
            mins = int(secs / 60)
            rem_txt = f"{mins // 60} h {mins % 60} min" if mins >= 60 else f"{mins} min"
        else:
            eta_txt, rem_txt = "—", "—"

        # ETA (big, left).
        qp.setPen(QColor(pal['text']))
        qp.setFont(QFont("Segoe UI", 34, QFont.Weight.Bold))
        qp.drawText(QRectF(island.left() + 30, island.top() + 26, 200, 50),
                    Qt.AlignmentFlag.AlignLeft, eta_txt)
        qp.setPen(QColor(pal['muted']))
        qp.setFont(QFont("Segoe UI", 11))
        qp.drawText(QRectF(island.left() + 32, island.top() + 78, 200, 20),
                    Qt.AlignmentFlag.AlignLeft, "predpokladaný príchod")

        # Distance + remaining time (right).
        qp.setPen(QColor(pal['title']))
        qp.setFont(QFont("Segoe UI", 26, QFont.Weight.Bold))
        qp.drawText(QRectF(island.right() - 220, island.top() + 28, 190, 40),
                    Qt.AlignmentFlag.AlignRight, f"{dist_km:.1f} km")
        qp.setPen(QColor(pal['text']))
        qp.setFont(QFont("Segoe UI", 13))
        qp.drawText(QRectF(island.right() - 220, island.top() + 72, 190, 24),
                    Qt.AlignmentFlag.AlignRight, f"⏱ {rem_txt}")


class _HUDPreview(QWidget):
    """In-app preview of the left-side driving HUD (the same 3D scene the
    transparent overlay draws over the game).

    Mirrors core/hud.py's driving-scene rendering so what you see here is what
    you get on the windshield — road ribbon, edge/centre lines, the blue
    anticipated route, surrounding vehicles as 3D models, and the traffic light
    + countdown.
    """

    def __init__(self, state):
        super().__init__()
        self.state = state
        self.setMinimumHeight(420)

    def paintEvent(self, event):
        import math
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        # Card background (same dark panel as the HUD).
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(10, 13, 18, 235))
        qp.drawRoundedRect(QRectF(0, 0, w, h), 16, 16)

        d = self._read()
        # Compact top readout (speed / KM/H / limit / gear).
        qp.setPen(QColor("#FFFFFF"))
        qp.setFont(QFont("Segoe UI", 40, QFont.Weight.Bold))
        qp.drawText(QRectF(16, 8, 160, 56), Qt.AlignmentFlag.AlignVCenter,
                    f"{d['speed_kmh']:.0f}")
        qp.setPen(QColor(255, 255, 255, 150))
        qp.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        qp.drawText(QRectF(18, 60, 120, 16), Qt.AlignmentFlag.AlignLeft, "KM/H")
        if d["limit_ms"] > 1:
            self._limit(qp, w - 100, 14, d["limit_ms"] * 3.6)
        qp.setBrush(QColor(40, 44, 52, 220))
        qp.setPen(QPen(QColor(90, 96, 104, 200), 1))
        qp.drawRoundedRect(QRectF(w - 48, 14, 34, 34), 8, 8)
        qp.setPen(QColor("#FFFFFF"))
        qp.setFont(QFont("Segoe UI", 15, QFont.Weight.Bold))
        g = d["gear"]
        gt = "N" if not g else (str(int(g)) if g > 0 else "R")
        qp.drawText(QRectF(w - 48, 15, 34, 32), Qt.AlignmentFlag.AlignCenter, gt)

        # 3D driving scene.
        scene = QRectF(8, 78, w - 16, h - 86)
        qp.save(); qp.setClipRect(scene)
        self._scene(qp, scene, d)
        qp.restore()

        # Status pill.
        on = d["active"]
        pill = QRectF(w / 2 - 60, h - 28, 120, 20)
        qp.setBrush(QColor("#10B981") if on else QColor(120, 120, 128, 220))
        qp.setPen(Qt.PenStyle.NoPen)
        qp.drawRoundedRect(pill, 10, 10)
        qp.setPen(QColor("#FFFFFF")); qp.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        qp.drawText(pill, Qt.AlignmentFlag.AlignCenter, "AUTOPILOT" if on else "MANUÁL")

    def _read(self):
        s = self.state
        speed = s.get("speed", 0) or 0
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            speed = 0.0
        truck = (s.get("telemetry", {}) or {}).get("truck", {}) or {}
        return {
            "speed_kmh": abs(speed) * 3.6 if abs(speed) < 200 else abs(speed),
            "gear": truck.get("gear", 0),
            "active": bool(s.get("autopilot_active", False)),
            "pos": s.get("truck_world_pos"),
            "heading": s.get("truck_heading", 0.0) or 0.0,
            "traffic": s.get("traffic", []) or [],
            "light": s.get("traffic_light"),
            "nav_path": (s.get("nav_path", []) or s.get("map_path", []) or []),
            "limit_ms": truck.get("speedLimit", 0.0) or 0.0,
        }

    def _limit(self, qp, x, y, kmh):
        qp.setBrush(QColor("#FFFFFF")); qp.setPen(QPen(QColor("#EF4444"), 4))
        qp.drawEllipse(QRectF(x, y, 36, 36))
        qp.setPen(QColor("#111827"))
        qp.setFont(QFont("Segoe UI", 12, QFont.Weight.Bold))
        qp.drawText(QRectF(x, y, 36, 36), Qt.AlignmentFlag.AlignCenter, f"{kmh:.0f}")

    def _proj(self, ahead, lateral, view, height=0.0):
        cam_h = 8.0; cam_back = 14.0
        f = view.height() * 1.05
        horizon = view.top() + view.height() * 0.26
        dist = ahead + cam_back
        if dist < 1.6:
            return None
        s = f / dist
        return QPointF(view.center().x() + lateral * s, horizon + (cam_h - height) * s)

    def _scene(self, qp, view, d):
        import math
        horizon_y = view.top() + view.height() * 0.26
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(20, 25, 32, 160))
        qp.drawRect(QRectF(view.left(), horizon_y, view.width(), view.bottom() - horizon_y))

        pos, h = d["pos"], d["heading"]

        def to_truck(wx, wz):
            dx, dz = wx - pos[0], wz - pos[1]
            ahead = dx * (-math.sin(h)) + dz * (-math.cos(h))
            lat = dx * math.cos(h) - dz * math.sin(h)
            return ahead, lat

        if pos:
            al = [to_truck(px, pz) for px, pz in d["nav_path"]]

            def offset_pt(i, off):
                a, l = al[i]
                j = min(i + 1, len(al) - 1)
                da, dl = al[j][0] - a, al[j][1] - l
                n = math.hypot(da, dl) or 1.0
                return self._proj(a, l + (-da / n) * off, view)

            if len(al) >= 2:
                HALF = 6.5
                left = [offset_pt(i, -HALF) for i in range(len(al))]
                right = [offset_pt(i, HALF) for i in range(len(al))]
                ribbon = [p for p in left if p] + [p for p in reversed(right) if p]
                if len(ribbon) >= 3:
                    qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor(36, 40, 46, 235))
                    qp.drawPolygon(QPolygonF(ribbon))
                for off, dash in ((-HALF, False), (HALF, False), (0.0, True)):
                    pts = [p for p in [offset_pt(i, off) for i in range(len(al))] if p]
                    if len(pts) >= 2:
                        st = Qt.PenStyle.DashLine if dash else Qt.PenStyle.SolidLine
                        qp.setPen(QPen(QColor(240, 240, 245, 200), 2, st))
                        qp.drawPolyline(QPolygonF(pts))
                pts = [self._proj(a, l, view) for a, l in al]
                pts = [p for p in pts if p is not None]
                if len(pts) >= 2:
                    qp.setPen(QPen(QColor(59, 130, 246, 80), 10))
                    qp.drawPolyline(QPolygonF(pts))
                    qp.setPen(QPen(QColor("#3B82F6"), 4))
                    qp.drawPolyline(QPolygonF(pts))
            else:
                left = [self._proj(a, -6.5, view) for a in range(2, 80, 6)]
                right = [self._proj(a, 6.5, view) for a in range(2, 80, 6)]
                ribbon = [p for p in left if p] + [p for p in reversed(right) if p]
                if len(ribbon) >= 3:
                    qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor(36, 40, 46, 220))
                    qp.drawPolygon(QPolygonF(ribbon))

            # Surrounding vehicles as 3D models (far → near for overlap).
            vehs = []
            for v in d["traffic"]:
                a, l = to_truck(v["x"], v["z"])
                if -6 < a < 70 and abs(l) < 18:
                    vehs.append((a, l, v))
            vehs.sort(key=lambda t: -t[0])
            for a, l, v in vehs:
                self._box(qp, view, a, l, v)
            self._box(qp, view, 6.0, 0.0,
                      {"type": "truck", "width": 2.6, "length": 14.0})

        # Traffic light + countdown.
        if d["light"]:
            color = d["light"].get("color", "off")
            cx, cy = view.right() - 66, view.top() + 12
            qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor(20, 24, 30, 235))
            qp.drawRoundedRect(QRectF(cx, cy, 22, 52), 6, 6)
            for i, (cn, cc) in enumerate((("red", "#EF4444"), ("yellow", "#FBBF24"),
                                          ("green", "#22C55E"))):
                qp.setBrush(QColor(cc) if color == cn else QColor(55, 60, 66))
                qp.drawEllipse(QRectF(cx + 3, cy + 3 + i * 15, 16, 16))
            tl = d["light"].get("time_left", 0) or 0
            if tl > 0:
                qp.setPen(QColor("#FFFFFF")); qp.setFont(QFont("Segoe UI", 14, QFont.Weight.Bold))
                qp.drawText(QRectF(cx + 26, cy + 2, 56, 20), Qt.AlignmentFlag.AlignLeft,
                            f"{tl:.1f}s")

    def _box(self, qp, view, ahead, lateral, v):
        t = v.get("type", "car")
        body_h = {"car": 1.1, "van": 1.7, "bus": 2.8, "truck": 2.6}.get(t, 1.2)
        hw = max(0.9, v.get("width", 2.0) / 2)
        ln = max(3.5, v.get("length", 4.5))
        n, fr = ahead - ln / 2, ahead + ln / 2
        self._box3d(qp, n, fr, hw, lateral, 0.0, body_h, view, ("#8A9099", "#AEB4BC"))
        if t == "truck":
            self._box3d(qp, fr - ln * 0.28, fr - 0.2, hw * 0.95, lateral, body_h, body_h + 1.0, view,
                        ("#9AA0A8", "#C2C8D0"))

    def _box3d(self, qp, n, fr, hw, lateral, z0, z1, view, faces):
        c = [self._proj(n, lateral - hw, view, z0), self._proj(n, lateral + hw, view, z0),
             self._proj(fr, lateral - hw, view, z0), self._proj(fr, lateral + hw, view, z0),
             self._proj(n, lateral - hw, view, z1), self._proj(n, lateral + hw, view, z1),
             self._proj(fr, lateral - hw, view, z1), self._proj(fr, lateral + hw, view, z1)]
        if any(p is None for p in c):
            return
        bl, br, fl, fr_, blt, brt, flt, frt = c
        side, top = faces
        qp.setPen(QPen(QColor("#34393F"), 1))
        qp.setBrush(QColor(side).darker(115)); qp.drawPolygon(QPolygonF([bl, br, brt, blt]))
        qp.setBrush(QColor(side)); qp.drawPolygon(QPolygonF([bl, fl, flt, blt]))
        qp.drawPolygon(QPolygonF([br, fr_, frt, brt]))
        qp.setBrush(QColor(side).darker(108)); qp.drawPolygon(QPolygonF([fl, fr_, frt, flt]))
        qp.setBrush(QColor(top)); qp.drawPolygon(QPolygonF([blt, brt, frt, flt]))


class VisualizationPage(QWidget):
    """Visualization tab: left HUD preview + glass island with ETA/distance."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        from core.theme import palette
        self._pal = palette(state.get("ui_theme", "light") or "light")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(30, 30, 30, 30)
        self.title = QLabel("🛰️ Visualization")
        self.title.setStyleSheet("font-size: 24px; font-weight: bold; color: " + self._pal['title'] + ";")
        lay.addWidget(self.title)

        self.sub = QLabel("Ľavý HUD panel — rovnaká 3D scéna, ktorá sa vykresľuje cez hru. "
                     "Modrá čiara = plánovaná trasa, sivé modely = okolité vozidlá, "
                     "semafor s odpočtom.")
        self.sub.setStyleSheet("color: " + self._pal['muted'] + "; font-size:13px;")
        self.sub.setWordWrap(True)
        lay.addWidget(self.sub)

        # Live preview of the left-side driving HUD. Prefer the real GPU 3D
        # renderer; fall back to the 2D QPainter preview if OpenGL is missing.
        try:
            from ui.driving_scene import DrivingScene
            self.scene = DrivingScene(state)
            if getattr(self.scene, "has_gl", False):
                self.hud_preview = self.scene
            else:
                self.hud_preview = _HUDPreview(state)
        except Exception:
            self.hud_preview = _HUDPreview(state)
        lay.addWidget(self.hud_preview, stretch=1)

        self.island = _GlassIsland(state)
        self.island._pal = self._pal
        lay.addWidget(self.island)
        lay.addStretch()
        self.timer = QTimer()
        # OpenGL widget updates itself internally; the 2D fallback needs update().
        if isinstance(self.hud_preview, _HUDPreview):
            self.timer.timeout.connect(self.hud_preview.update)
        self.timer.timeout.connect(self.island.update)
        self.timer.start(120)

    def restyle(self, theme):
        """Re-apply palette colours when the theme switches (dark ↔ light)."""
        from core.theme import palette
        self._pal = palette(theme)
        self.title.setStyleSheet("font-size: 24px; font-weight: bold; color: " + self._pal['title'] + ";")
        self.sub.setStyleSheet("color: " + self._pal['muted'] + "; font-size:13px;")
        self.island._pal = self._pal
