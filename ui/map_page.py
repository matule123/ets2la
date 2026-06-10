import os
import json
import math

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit, QComboBox,
)
from PyQt6.QtGui import QPainter, QColor, QPen, QPolygonF
from PyQt6.QtCore import Qt, QTimer, QPointF
from core.paths import app_dir

ROUTES_DIR = os.path.join(app_dir(), "routes")


class MapView(QWidget):
    """Top-down 2D view of the active route polyline and the truck pose."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        self.route_points = []   # [(x, z), ...] drawn route (loaded for display)
        self.setMinimumHeight(300)
        self.setStyleSheet("background-color: #0E0E0E; border-radius: 8px;")

    def set_route(self, points):
        self.route_points = points or []
        self.update()

    def _bounds(self, pts):
        xs = [p[0] for p in pts]
        zs = [p[1] for p in pts]
        return min(xs), max(xs), min(zs), max(zs)

    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        truck = self.state.get("truck_world_pos")
        heading = self.state.get("truck_heading", 0.0) or 0.0

        pts = list(self.route_points)
        all_pts = pts + ([truck] if truck else [])
        if not all_pts:
            qp.setPen(QColor("#555"))
            qp.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                        "No route loaded.\nRecord or load a route below.")
            return

        minx, maxx, minz, maxz = self._bounds(all_pts)
        span = max(maxx - minx, maxz - minz, 50.0)
        pad = 30
        scale = (min(w, h) - 2 * pad) / span

        def to_screen(p):
            sx = pad + (p[0] - minx) * scale
            # Flip Z so "north" (smaller z) is up on screen.
            sy = pad + (maxz - p[1]) * scale
            return QPointF(sx, sy)

        # Route polyline.
        if len(pts) >= 2:
            qp.setPen(QPen(QColor("#00FF7F"), 3))
            poly = QPolygonF([to_screen(p) for p in pts])
            qp.drawPolyline(poly)
            # Start (green) and end (red) markers.
            qp.setBrush(QColor("#34C759"))
            qp.drawEllipse(to_screen(pts[0]), 5, 5)
            qp.setBrush(QColor("#FF453A"))
            qp.drawEllipse(to_screen(pts[-1]), 5, 5)

        # Truck as a heading arrow.
        if truck:
            c = to_screen(truck)
            fx, fz = -math.sin(heading), -math.cos(heading)
            tip = QPointF(c.x() + fx * 14, c.y() - fz * 14)
            left = QPointF(c.x() - fz * 7 + fx * -6, c.y() - fx * 7 - fz * -6)
            right = QPointF(c.x() + fz * 7 + fx * -6, c.y() + fx * 7 - fz * -6)
            qp.setBrush(QColor("#FFD60A"))
            qp.setPen(QPen(QColor("#FFD60A"), 1))
            qp.drawPolygon(QPolygonF([tip, left, right]))


class MapPage(QWidget):
    """Navigation page: record / replay routes and watch the truck follow them."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(15)

        title = QLabel("🗺️ Navigation")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #00FF7F;")
        layout.addWidget(title)

        self.view = MapView(state)
        layout.addWidget(self.view)

        self.status = QLabel("Idle.")
        self.status.setStyleSheet("color: #8E8E93;")
        layout.addWidget(self.status)

        # --- Record row ---
        rec_row = QHBoxLayout()
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("route name")
        self.name_edit.setStyleSheet("background:#1E1E1E; border:1px solid #333; padding:6px;")
        btn_rec = QPushButton("● Record")
        btn_stop_rec = QPushButton("■ Stop & Save")
        btn_rec.clicked.connect(self.start_record)
        btn_stop_rec.clicked.connect(self.stop_record)
        rec_row.addWidget(self.name_edit)
        rec_row.addWidget(btn_rec)
        rec_row.addWidget(btn_stop_rec)
        layout.addLayout(rec_row)

        # --- Replay row ---
        play_row = QHBoxLayout()
        self.route_combo = QComboBox()
        self.route_combo.setStyleSheet("background:#1E1E1E; border:1px solid #333; padding:6px;")
        btn_load = QPushButton("▶ Load & Navigate")
        btn_clear = QPushButton("⏹ Stop Nav")
        btn_load.clicked.connect(self.load_route)
        btn_clear.clicked.connect(self.stop_nav)
        play_row.addWidget(self.route_combo)
        play_row.addWidget(btn_load)
        play_row.addWidget(btn_clear)
        layout.addLayout(play_row)
        layout.addStretch()

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(150)
        self._last_routes = None

    # --- Actions --------------------------------------------------------------
    def start_record(self):
        name = (self.name_edit.text().strip() or "route").replace(" ", "_")
        self.state.set("nav_arg", name)
        self.state.set("nav_cmd", "record")
        self.status.setText(f"Recording '{name}'… drive the route, then Stop & Save.")

    def stop_record(self):
        self.state.set("nav_cmd", "stop_record")
        self.status.setText("Route saved.")

    def load_route(self):
        name = self.route_combo.currentText()
        if not name:
            return
        self.state.set("nav_arg", name)
        self.state.set("nav_cmd", "load")
        # Load the polyline into the view for display.
        try:
            with open(os.path.join(ROUTES_DIR, f"{name}.json")) as f:
                self.view.set_route(json.load(f).get("points", []))
        except Exception:
            self.view.set_route([])
        self.status.setText(f"Navigating route '{name}'.")

    def stop_nav(self):
        self.state.set("nav_cmd", "stop")
        self.view.set_route([])
        self.status.setText("Navigation stopped.")

    def refresh(self):
        # Keep the route dropdown in sync with what the map plugin published.
        routes = self.state.get("nav_routes", []) or []
        if routes != self._last_routes:
            self._last_routes = list(routes)
            current = self.route_combo.currentText()
            self.route_combo.clear()
            self.route_combo.addItems(routes)
            if current in routes:
                self.route_combo.setCurrentText(current)

        if self.state.get("nav_active"):
            dist = self.state.get("distance_to_dest")
            if dist is not None:
                self.status.setText(f"Navigating — {float(dist) / 1000:.2f} km to destination.")
        self.view.update()
