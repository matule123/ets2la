import os
import json
import math

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit, QComboBox,
    QProgressBar,
)
from PyQt6.QtGui import QPainter, QColor, QPen, QPolygonF
from PyQt6.QtCore import Qt, QTimer, QPointF, QThread, pyqtSignal
from core.paths import app_dir

ROUTES_DIR = os.path.join(app_dir(), "routes")


class MapDownloadWorker(QThread):
    """Downloads + extracts a map dataset in the background."""
    progress = pyqtSignal(float, str)
    done = pyqtSignal(bool)

    def __init__(self, key):
        super().__init__()
        self.key = key

    def run(self):
        try:
            from core.navigation import map_data
            ok = map_data.download(self.key, progress_cb=lambda f, t: self.progress.emit(f, t))
            self.done.emit(bool(ok))
        except Exception:
            self.done.emit(False)


class RoadNetLoadWorker(QThread):
    """Loads the downloaded road network in the background (can be large)."""
    # (network, reason) — reason is "" on success, a human hint otherwise.
    done = pyqtSignal(object, str)

    def run(self):
        try:
            from core.navigation import map_data
            from core.navigation.road_network import RoadNetwork
            downloaded = [d for d in map_data.list_datasets() if d["downloaded"]]
            if not downloaded:
                self.done.emit(None, "no_map")
                return
            net = RoadNetwork()
            if net.load(map_data.dataset_dir(downloaded[0]["key"])):
                self.done.emit(net, "")
            else:
                # Files present but couldn't be parsed — usually a corrupt or
                # half-finished download. Suggesting a re-download fixes it.
                self.done.emit(None, "corrupt")
        except Exception as e:
            self.done.emit(None, f"error:{e}")


class MapView(QWidget):
    """Top-down 2D view of the active route polyline and the truck pose."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        self.route_points = []   # [(x, z), ...] drawn route (loaded for display)
        self.road_net = None     # RoadNetwork (when a map is downloaded + loaded)
        self.setMinimumHeight(300)
        self.setStyleSheet("background-color: #FFFFFF; border: 1px solid #E5E7EB; border-radius: 10px;")

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

        # Truck-centered map view when the road network is loaded.
        if self.road_net is not None and self.road_net.loaded and truck:
            self._paint_map(qp, w, h, truck, heading)
            return

        pts = list(self.route_points)
        all_pts = pts + ([truck] if truck else [])
        if not all_pts:
            qp.setPen(QColor("#9CA3AF"))
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
            qp.setPen(QPen(QColor("#10B981"), 3))
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

    def _paint_map(self, qp, w, h, truck, heading):
        """Truck-centered road-network view (fixed zoom, ~radius metres around)."""
        radius = 700.0                     # metres shown around the truck
        scale = (min(w, h) - 20) / (2 * radius)
        cx, cz = truck

        def to_screen(p):
            sx = w / 2 + (p[0] - cx) * scale
            sy = h / 2 - (cz - p[1]) * scale   # flip Z so north is up
            return QPointF(sx, sy)

        # Nearby roads (grey).
        qp.setPen(QPen(QColor("#B7BDC6"), 2))
        for a, b in self.road_net.segments_near(truck, radius):
            qp.drawLine(to_screen(a), to_screen(b))

        # Road ahead from the map graph (blue) — stage-3 path generation.
        try:
            ahead = self.road_net.path_ahead(truck, heading)
            if len(ahead) >= 2:
                qp.setPen(QPen(QColor("#2563EB"), 4))
                qp.drawPolyline(QPolygonF([to_screen(p) for p in ahead]))
        except Exception:
            pass

        # Recorded/loaded route on top (green).
        pts = list(self.route_points)
        if len(pts) >= 2:
            qp.setPen(QPen(QColor("#10B981"), 3))
            qp.drawPolyline(QPolygonF([to_screen(p) for p in pts]))

        # Truck arrow at centre.
        c = to_screen(truck)
        fx, fz = -math.sin(heading), -math.cos(heading)
        tip = QPointF(c.x() + fx * 16, c.y() - fz * 16)
        left = QPointF(c.x() - fz * 8 + fx * -7, c.y() - fx * 8 - fz * -7)
        right = QPointF(c.x() + fz * 8 + fx * -7, c.y() + fx * 8 - fz * -7)
        qp.setBrush(QColor("#F59E0B"))
        qp.setPen(QPen(QColor("#B45309"), 1))
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
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #065F46;")
        layout.addWidget(title)

        self.view = MapView(state)
        layout.addWidget(self.view)

        self.status = QLabel("Idle.")
        self.status.setStyleSheet("color: #6B7280;")
        layout.addWidget(self.status)

        # --- Record row ---
        rec_row = QHBoxLayout()
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("route name")
        self.name_edit.setStyleSheet("background:#FFFFFF; border:1px solid #DfE3E8; border-radius:8px; padding:6px;")
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
        self.route_combo.setStyleSheet("background:#FFFFFF; border:1px solid #DfE3E8; border-radius:8px; padding:6px;")
        btn_load = QPushButton("▶ Load & Navigate")
        btn_clear = QPushButton("⏹ Stop Nav")
        btn_load.clicked.connect(self.load_route)
        btn_clear.clicked.connect(self.stop_nav)
        play_row.addWidget(self.route_combo)
        play_row.addWidget(btn_load)
        play_row.addWidget(btn_clear)
        layout.addLayout(play_row)

        # --- Map data (ETS2 / ProMods / versions) ---
        map_title = QLabel("Map data")
        map_title.setStyleSheet("font-size: 15px; font-weight: bold; color: #0F766E; margin-top: 8px;")
        layout.addWidget(map_title)
        map_row = QHBoxLayout()
        self.map_combo = QComboBox()
        self.map_combo.setStyleSheet("background:#FFFFFF; border:1px solid #DfE3E8; border-radius:8px; padding:6px;")
        self.map_combo.currentIndexChanged.connect(self._on_map_selected)
        self.btn_dl = QPushButton("⬇ Download map")
        self.btn_dl.clicked.connect(self.download_map)
        map_row.addWidget(self.map_combo)
        map_row.addWidget(self.btn_dl)
        layout.addLayout(map_row)
        self.active_map_lbl = QLabel("Aktívna mapa: —")
        self.active_map_lbl.setStyleSheet("color: #0F766E; font-size: 13px; font-weight: 700;")
        layout.addWidget(self.active_map_lbl)
        self.dl_bar = QProgressBar()
        self.dl_bar.setVisible(False)
        layout.addWidget(self.dl_bar)
        self.dl_status = QLabel("")
        self.dl_status.setStyleSheet("color: #6B7280; font-size: 12px;")
        layout.addWidget(self.dl_status)

        layout.addStretch()

        self._dl_worker = None
        self._net_worker = None
        self._populate_maps()
        self._load_road_net()   # if a map is already downloaded, load it for display

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(150)
        self._last_routes = None

    # --- Map data -------------------------------------------------------------
    def _populate_maps(self):
        try:
            from core.navigation import map_data
            datasets = map_data.list_datasets()
        except Exception as e:
            self.dl_status.setText(f"Could not reach map index: {e}")
            return
        self.map_combo.blockSignals(True)
        self.map_combo.clear()
        # Read the user's last selection so we pre-select it (saved map).
        try:
            from core.settings.manager import SettingsManager
            wanted = (SettingsManager().get("selected_map") or "").strip()
        except Exception:
            wanted = ""
        sel_idx = 0
        for i, d in enumerate(datasets):
            mark = "✓ " if d["downloaded"] else ""
            self.map_combo.addItem(f"{mark}{d['key']}  ({d['game']} {d['version']})", d["key"])
            if d["key"] == wanted:
                sel_idx = i
        if datasets:
            self.map_combo.setCurrentIndex(sel_idx)
            self.dl_status.setText("Pick your game version (or ProMods) and download once.")
        self.map_combo.blockSignals(False)
        self._update_active_map_label()

    def _on_map_selected(self, _idx):
        """User picked a dataset in the combo — remember it as the active map."""
        key = self.map_combo.currentData()
        if not key:
            return
        try:
            from core.settings.manager import SettingsManager
            SettingsManager().set("selected_map", key)
        except Exception:
            pass
        # Mirror to shared state so the engine/map plugin can switch without restart.
        self.state.set("selected_map", key)
        self._update_active_map_label()

    def _update_active_map_label(self):
        """Show which map the autopilot is actually using."""
        name = self.state.get("active_map_name") or self.state.get("active_map_key")
        sel = self.state.get("selected_map")
        if name:
            self.active_map_lbl.setText(f"Aktívna mapa: {name}")
        elif sel:
            self.active_map_lbl.setText(f"Vybraná mapa: {sel}")
        else:
            self.active_map_lbl.setText("Aktívna mapa: —")

    def download_map(self):
        if self._dl_worker is not None:
            return
        key = self.map_combo.currentData()
        if not key:
            return
        self.btn_dl.setEnabled(False)
        self.dl_bar.setVisible(True)
        self._dl_worker = MapDownloadWorker(key)
        self._dl_worker.progress.connect(self._on_dl_progress)
        self._dl_worker.done.connect(self._on_dl_done)
        self._dl_worker.start()

    def _on_dl_progress(self, frac, text):
        self.dl_bar.setValue(int(frac * 100))
        self.dl_status.setText(text)

    def _on_dl_done(self, ok):
        self.btn_dl.setEnabled(True)
        self.dl_bar.setVisible(False)
        self._dl_worker = None
        self.dl_status.setText("✓ Map downloaded — loading road network…"
                               if ok else "✗ Download failed (check internet).")
        self._populate_maps()
        if ok:
            self._load_road_net()

    def _load_road_net(self):
        """Load the downloaded road network in the background (for the map view)."""
        if self.view.road_net is not None or self._net_worker is not None:
            return
        try:
            from core.navigation import map_data
            if not any(d["downloaded"] for d in map_data.list_datasets()):
                return
        except Exception:
            return
        self.dl_status.setText("Loading road network…")
        self._net_worker = RoadNetLoadWorker()
        self._net_worker.done.connect(self._on_net_loaded)
        self._net_worker.start()

    def _on_net_loaded(self, net, reason=""):
        self._net_worker = None
        if net is not None:
            self.view.road_net = net
            self.dl_status.setText(f"✓ Map loaded ({len(net.segments)} road segments). "
                                   "Roads around the truck are shown above.")
            self.view.update()
            return

        # Tell the user *why* it failed and what to do, instead of a bare message.
        if reason == "no_map":
            self.dl_status.setText("No map downloaded yet. Pick your game version and download.")
        elif reason == "corrupt":
            self.dl_status.setText("Map files look incomplete or corrupt. "
                                   "Please download the map again.")
        elif reason.startswith("error:"):
            self.dl_status.setText(f"Could not load map: {reason[6:]}. "
                                   "Try downloading it again.")
        else:
            self.dl_status.setText("Map data present but could not be loaded.")

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

        # Publish the road ahead so the autopilot can steer by map (no recording).
        net = self.view.road_net
        truck = self.state.get("truck_world_pos")
        if net is not None and net.loaded and truck:
            try:
                path = net.path_ahead(truck, self.state.get("truck_heading", 0.0) or 0.0)
                self.state.set("map_path", [list(p) for p in path] if len(path) >= 2 else [])
            except Exception:
                self.state.set("map_path", [])

        if self.state.get("nav_active"):
            dist = self.state.get("distance_to_dest")
            if dist is not None:
                self.status.setText(f"Navigating — {float(dist) / 1000:.2f} km to destination.")
            else:
                # Map-based driving (no recorded route) — show a clear status so
                # the user knows the autopilot is steering by the map.
                self.status.setText("Map steering active — following the road ahead.")
        else:
            # Surface the map-loading status the engine publishes (loading /
            # ready / error) so the user is never left guessing why nav is off.
            ms = self.state.get("map_status")
            if ms:
                self.status.setText(str(ms))
        self.view.update()
