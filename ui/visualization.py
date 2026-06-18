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
        qp.setBrush(QColor(18, 22, 28)); qp.setPen(Qt.PenStyle.NoPen)
        qp.drawRoundedRect(QRectF(0, 0, w, h), 12, 12)

        pos = self.state.get("truck_world_pos")
        if not pos:
            qp.setPen(QColor("#6B7280"))
            qp.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Top-down map\n(needs telemetry)")
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

        # Road ahead (filled ribbon from the published path).
        path = self.state.get("nav_path", []) or self.state.get("map_path", []) or []
        if len(path) >= 2:
            pts = [to_screen(px, pz) for px, pz in path]
            qp.setPen(QPen(QColor("#2563EB"), max(6, int(7 * scale)), Qt.PenStyle.SolidLine,
                           Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            qp.drawPolyline(QPolygonF(pts))

        # Surrounding traffic (grey dots).
        qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor("#9CA3AF"))
        for v in (self.state.get("traffic", []) or []):
            p = to_screen(v["x"], v["z"])
            if 0 <= p.x() <= w and 0 <= p.y() <= h:
                qp.drawEllipse(p, 4, 4)

        # Ego truck (green arrow, always pointing up).
        qp.setBrush(QColor("#10B981")); qp.setPen(QPen(QColor("#065F46"), 1))
        qp.drawPolygon(QPolygonF([QPointF(cx, cy - 12), QPointF(cx - 8, cy + 8),
                                  QPointF(cx + 8, cy + 8)]))


class _GlassIsland(QWidget):
    """Frosted 'liquid glass' island showing ETA + remaining distance."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        self.setMinimumHeight(170)

    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        # Pull live data.
        dist_m = self.state.get("distance_to_dest")
        speed = self.state.get("truck_speed_ms", 0.0) or 0.0
        nav = bool(self.state.get("nav_active"))

        # Layered translucent rounded rects = frosted-glass look.
        island = QRectF(w / 2 - 230, h / 2 - 70, 460, 140)
        for i, a in enumerate((40, 70, 235)):
            qp.setBrush(QColor(255, 255, 255, a))
            qp.setPen(QPen(QColor(255, 255, 255, 90), 1))
            qp.drawRoundedRect(island.adjusted(-i * 4, -i * 4, i * 4, i * 4), 34, 34)
        qp.setPen(QPen(QColor(16, 185, 129, 120), 2))
        qp.setBrush(Qt.BrushStyle.NoBrush)
        qp.drawRoundedRect(island, 34, 34)

        if not nav or dist_m is None:
            qp.setPen(QColor("#6B7280"))
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
        qp.setPen(QColor("#111827"))
        qp.setFont(QFont("Segoe UI", 34, QFont.Weight.Bold))
        qp.drawText(QRectF(island.left() + 30, island.top() + 26, 200, 50),
                    Qt.AlignmentFlag.AlignLeft, eta_txt)
        qp.setPen(QColor("#6B7280"))
        qp.setFont(QFont("Segoe UI", 11))
        qp.drawText(QRectF(island.left() + 32, island.top() + 78, 200, 20),
                    Qt.AlignmentFlag.AlignLeft, "predpokladaný príchod")

        # Distance + remaining time (right).
        qp.setPen(QColor("#10B981"))
        qp.setFont(QFont("Segoe UI", 26, QFont.Weight.Bold))
        qp.drawText(QRectF(island.right() - 220, island.top() + 28, 190, 40),
                    Qt.AlignmentFlag.AlignRight, f"{dist_km:.1f} km")
        qp.setPen(QColor("#374151"))
        qp.setFont(QFont("Segoe UI", 13))
        qp.drawText(QRectF(island.right() - 220, island.top() + 72, 190, 24),
                    Qt.AlignmentFlag.AlignRight, f"⏱ {rem_txt}")


class VisualizationPage(QWidget):
    """Visualization tab: a glass island with ETA + remaining distance."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        lay = QVBoxLayout(self)
        lay.setContentsMargins(30, 30, 30, 30)
        title = QLabel("🛰️ Visualization")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #065F46;")
        lay.addWidget(title)

        # Real GPU 3D driving scene (degrades to a hint if libs are missing).
        try:
            from ui.gpu_view import GpuView
            lay.addWidget(GpuView(state), stretch=1)
        except Exception as e:
            msg = QLabel(f"3D view unavailable:\n{e}\n\n"
                        "Install the GPU libraries:\npip install pyqtgraph PyOpenGL")
            msg.setStyleSheet("color:#6B7280; font-size:13px;")
            lay.addWidget(msg, stretch=1)

        self.island = _GlassIsland(state)
        lay.addWidget(self.island)
        lay.addStretch()
        self.timer = QTimer()
        self.timer.timeout.connect(self.island.update)
        self.timer.start(500)
