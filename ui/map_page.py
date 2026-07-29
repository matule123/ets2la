import math
import time

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QProgressBar, QFrame, QSizePolicy,
)
from PyQt6.QtGui import QPainter, QColor, QPen, QPolygonF
from PyQt6.QtCore import Qt, QTimer, QPointF, QPoint, QThread, pyqtSignal
from core.navigation.navigation_intent import snapshot_matches_navigation_intent


def live_map_navigation_points(state, now=None):
    """Return the sole geometry that the live map may draw in this state."""
    snapshot = state.get("lane_trajectory", {}) or {}
    try:
        game_uids = tuple(int(uid) for uid in
                          (state.get("game_route_node_uids", []) or []))
        snapshot_uids = tuple(int(uid) for uid in
                              (snapshot.get("source_gps_uids", []) or []))
        route_distance = float(state.get("game_route_distance", 0.0) or 0.0)
        gps_present = bool(
            state.get("game_gps_navigation_active", False)
            or state.get("navigation_arrival_pending", False)
            or state.get("dest_city") or route_distance > 0.0
            or len(game_uids) >= 2 or len(snapshot_uids) >= 2)
    except (TypeError, ValueError, OverflowError):
        return []

    if gps_present:
        try:
            revision = int(snapshot.get("revision", -2) or -2)
            current_revision = int(state.get(
                "lane_trajectory_revision", -1) or -1)
            heartbeat = float(state.get(
                "lane_trajectory_heartbeat", 0.0) or 0.0)
            now = time.monotonic() if now is None else float(now)
            points = snapshot.get("display_points", []) or []
            if (not snapshot.get("valid", False)
                    or revision != current_revision
                    or not snapshot_matches_navigation_intent(state, snapshot)
                    or heartbeat <= 0.0 or now - heartbeat > 0.5
                    or state.get("telemetry_valid", True) is False
                    or state.get("navigation_recalculating", False)
                    or len(points) < 2):
                return []
            if any(not isinstance(point, (list, tuple)) or len(point) < 3
                   or not all(math.isfinite(float(value)) for value in point[:3])
                   for point in points):
                return []
            return points
        except (TypeError, ValueError, OverflowError):
            return []

    if (state.get("navigation_source") == "recorded_route"
            and state.get("recorded_route_active", False)):
        points = state.get("nav_path", []) or []
        try:
            if len(points) >= 2 and all(
                    isinstance(point, (list, tuple)) and len(point) >= 2
                    and all(math.isfinite(float(value)) for value in point[:3])
                    for point in points):
                return points
        except (TypeError, ValueError, OverflowError):
            pass
    return []


def rejected_navigation_command_message(state):
    """Return a persistent user-facing reason for a rejected command."""
    result = state.get("nav_command_result") or {}
    if result and not result.get("ok", False):
        return str(result.get("message") or "Navigation command was rejected.")
    return ""


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


class MapView(QWidget):
    """Top-down 2D view of the active route polyline and the truck pose."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        # Bounded display-only snapshot from the map plugin. Loading another
        # complete RoadNetwork here doubles memory and can terminate the UI.
        self.road_segments = []
        self._pal = None         # set by the page (or a default below)
        self.zoom_radius = 280.0
        self.pan_world = [0.0, 0.0]
        self._drag_at = None
        self.setMinimumHeight(300)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.live_badge = QLabel("●  LIVE MAP", self)
        self.live_badge.setObjectName("LiveMapBadge")
        self.live_badge.setStyleSheet(
            "background:rgba(19,24,28,225);color:#34D399;"
            "border:1px solid #2F3A3E;border-radius:10px;padding:6px 10px;"
            "font-size:11px;font-weight:750;")
        self.live_badge.adjustSize()
        self.empty_state = QLabel(
            "<div style='font-size:17px;font-weight:700;color:#F3F4F6;'>"
            "Čakám na živú mapu</div>"
            "<div style='margin-top:7px;color:#9CA3AF;'>"
            "Spusti hru a nastav GPS cieľ.<br>"
            "Potvrdená trasa sa zobrazí automaticky.</div>", self)
        self.empty_state.setObjectName("LiveMapEmptyState")
        self.empty_state.setTextFormat(Qt.TextFormat.RichText)
        self.empty_state.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.empty_state.setWordWrap(True)
        self.empty_state.setFixedSize(360, 108)
        self.empty_state.setStyleSheet(
            "background:rgba(29,32,35,235);border:1px solid #353A40;"
            "border-radius:14px;padding:12px;")
        self.apply_theme()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.live_badge.move(14, 14)
        self.live_badge.raise_()
        self.empty_state.move(max(14, (self.width() - self.empty_state.width()) // 2),
                              max(52, (self.height() - self.empty_state.height()) // 2))
        self.empty_state.raise_()

    def reset_view(self):
        self.zoom_radius = 280.0
        self.pan_world[:] = [0.0, 0.0]
        self.update()

    def wheelEvent(self, event):
        # Wheel up zooms in, wheel down zooms out to a broad regional view.
        factor = 0.78 if event.angleDelta().y() > 0 else 1.28
        self.zoom_radius = max(90.0, min(500.0, self.zoom_radius * factor))
        self.update()
        event.accept()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_at = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._drag_at is None or not (event.buttons() & Qt.MouseButton.LeftButton):
            return
        delta = event.position() - self._drag_at
        scale = max(1e-6, (min(self.width(), self.height()) - 20) / (2 * self.zoom_radius))
        self.pan_world[0] -= delta.x() / scale
        self.pan_world[1] -= delta.y() / scale
        self._drag_at = event.position()
        self.update()

    def mouseReleaseEvent(self, event):
        self._drag_at = None
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def mouseDoubleClickEvent(self, event):
        self.reset_view()

    def apply_theme(self):
        """Apply the palette background/border. Called on init + theme switch."""
        from core.theme import palette
        if self._pal is None:
            self._pal = palette(self.state.get("ui_theme", "light") or "light")
        self.setStyleSheet(
            "background-color:#151515;border:1px solid #303238;border-radius:10px;")

    def set_road_segments(self, payload):
        """Keep only finite X/Z endpoints from the lightweight live snapshot."""
        segments = []
        for item in (payload or [])[:1200]:
            try:
                a, b = item[0], item[1]
                values = float(a[0]), float(a[1]), float(b[0]), float(b[1])
                if not all(math.isfinite(value) for value in values):
                    continue
                segments.append((values[:2], values[2:]))
            except (TypeError, ValueError, IndexError):
                continue
        self.road_segments = segments

    def _bounds(self, pts):
        xs = [p[0] for p in pts]
        zs = [p[1] for p in pts]
        return min(xs), max(xs), min(zs), max(zs)

    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor("#151515"))
        qp.drawRoundedRect(self.rect(), 10, 10)
        qp.setPen(QPen(QColor("#202326"), 1))
        for x in range(0, w, 42):
            qp.drawLine(x, 0, x, h)
        for y in range(0, h, 42):
            qp.drawLine(0, y, w, y)

        truck = self.state.get("truck_world_pos")
        heading = self.state.get("truck_heading", 0.0) or 0.0

        # Truck-centered view from the engine-published local map snapshot.
        if self.road_segments and truck:
            self.empty_state.hide()
            self._paint_map(qp, w, h, truck, heading)
            return

        pts = [point for point in (
            self._to_xz(point)
            for point in live_map_navigation_points(self.state))
            if point is not None]
        all_pts = pts + ([truck] if truck else [])
        if not all_pts:
            self.empty_state.show()
            self.empty_state.raise_()
            return
        self.empty_state.hide()

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
            qp.setPen(QPen(QColor("#1597F5"), 5, Qt.PenStyle.SolidLine,
                           Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
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
        """Continuous road-network view with wheel zoom and mouse panning."""
        radius = self.zoom_radius
        scale = (min(w, h) - 20) / (2 * radius)
        cx = truck[0] + self.pan_world[0]
        cz = truck[1] + self.pan_world[1]

        def to_screen(p):
            sx = w / 2 + (p[0] - cx) * scale
            sy = h / 2 - (cz - p[1]) * scale   # flip Z so north is up
            return QPointF(sx, sy)

        # Nearby roads (grey).
        qp.setPen(QPen(QColor("#555B63"), 2, Qt.PenStyle.SolidLine,
                       Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
        for a, b in self.road_segments:
            qp.drawLine(to_screen(a), to_screen(b))

        # GPS uses only current-revision snapshot geometry. Recorded replay is
        # admitted only by live_map_navigation_points() in its exclusive mode.
        ahead = live_map_navigation_points(self.state)
        ahead = [point for point in (self._to_xz(point) for point in ahead)
                 if point is not None]
        if len(ahead) >= 2:
            qp.setPen(QPen(QColor("#1597F5"), 6, Qt.PenStyle.SolidLine,
                           Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            qp.drawPolyline(QPolygonF([to_screen(p) for p in ahead]))

        # Truck arrow at centre.
        c = to_screen(truck)
        fx, fz = -math.sin(heading), -math.cos(heading)
        tip = QPointF(c.x() + fx * 16, c.y() - fz * 16)
        left = QPointF(c.x() - fz * 8 + fx * -7, c.y() - fx * 8 - fz * -7)
        right = QPointF(c.x() + fz * 8 + fx * -7, c.y() + fx * 8 - fz * -7)
        qp.setBrush(QColor("#1597F5"))
        qp.setPen(QPen(QColor("#E8F4FF"), 2))
        qp.drawPolygon(QPolygonF([tip, left, right]))

        # Small unobtrusive interaction hint; no extra toolbar is needed.
        qp.setPen(QColor(185, 190, 198, 155))
        qp.drawText(14, h - 12, "koliesko: zoom  •  potiahnuť: posun  •  dvojklik: kamión")

    @staticmethod
    def _to_xz(point):
        """Accept legacy X/Z and authoritative lane X/Y/Z points."""
        if not isinstance(point, (list, tuple)):
            return None
        try:
            if len(point) >= 3:
                return float(point[0]), float(point[2])
            if len(point) >= 2:
                return float(point[0]), float(point[1])
        except (TypeError, ValueError, OverflowError):
            return None
        return None

class MapPage(QWidget):
    """Game-GPS navigation, active dataset and authoritative live map."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        from core.theme import palette
        self._pal = palette(state.get("ui_theme", "light") or "light")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(10)

        head = QHBoxLayout()
        self.title = QLabel("Navigácia")
        self.title.setStyleSheet("font-size:24px;font-weight:800;color:" + self._pal['text'] + ";")
        head.addWidget(self.title)
        self.title_copy = QLabel("Herná GPS, mapové dáta a živý priebeh trasy")
        self.title_copy.setStyleSheet("color:" + self._pal['muted'] + ";font-size:12px;margin-left:8px;")
        head.addWidget(self.title_copy)
        head.addStretch()
        layout.addLayout(head)

        stats = QHBoxLayout()
        stats.setSpacing(10)
        self.nav_stats = {}
        for key, caption in (("gps", "HERNÁ GPS"), ("map", "AKTÍVNA MAPA"),
                             ("trajectory", "TRAJEKTÓRIA")):
            card = QFrame()
            card.setObjectName("NavigationStat")
            card.setStyleSheet(
                "QFrame#NavigationStat{background:" + self._pal['card']
                + ";border:1px solid " + self._pal['border']
                + ";border-radius:11px;}")
            box = QVBoxLayout(card)
            box.setContentsMargins(14, 10, 14, 10)
            box.setSpacing(2)
            cap = QLabel(caption)
            cap.setStyleSheet("font-size:9px;font-weight:750;color:"
                              + self._pal['muted'] + ";")
            value = QLabel("—")
            value.setStyleSheet("font-size:13px;font-weight:700;color:"
                                + self._pal['text'] + ";")
            box.addWidget(cap)
            box.addWidget(value)
            stats.addWidget(card, 1)
            self.nav_stats[key] = (card, cap, value)
        layout.addLayout(stats)

        content = QHBoxLayout()
        content.setSpacing(12)
        self.controls = QFrame()
        controls = self.controls
        controls.setObjectName("NavigationControls")
        controls.setFixedWidth(300)
        controls.setStyleSheet(
            "#NavigationControls{background:" + self._pal['card']
            + ";border:1px solid " + self._pal['border'] + ";border-radius:12px;}"
        )
        ctl = QVBoxLayout(controls)
        ctl.setContentsMargins(14, 14, 14, 14)
        ctl.setSpacing(9)

        self.source_cap = QLabel("AKTÍVNA NAVIGÁCIA")
        self.source_cap.setStyleSheet(
            "color:#7B818A!important;font-size:10px;font-weight:750;letter-spacing:1px;")
        ctl.addWidget(self.source_cap)
        self.status = QLabel("Čakám na hernú GPS trasu")
        self.status.setWordWrap(True)
        self.status.setStyleSheet(
            "background:#F5F6F7;color:#59616C;border-radius:7px;padding:9px;font-size:12px;")
        ctl.addWidget(self.status)

        self.gps_hint = QLabel(
            "Cieľ nastav priamo v navigácii hry. UltraPilot automaticky "
            "použije potvrdenú trasu a rovnakú revíziu zobrazí na HUD, AR aj mape.")
        self.gps_hint.setWordWrap(True)
        self.gps_hint.setStyleSheet(
            "color:#6B7280!important;font-size:11px;line-height:1.3;")
        ctl.addWidget(self.gps_hint)

        self.map_title = QLabel("MAPOVÉ DÁTA")
        self.map_title.setStyleSheet("color:#7B818A;font-size:10px;font-weight:700;margin-top:8px;")
        ctl.addWidget(self.map_title)
        self.map_combo = QComboBox()
        self.map_combo.currentIndexChanged.connect(self._on_map_selected)
        ctl.addWidget(self.map_combo)
        self.btn_dl = QPushButton("↓  Download map data")
        self.btn_dl.clicked.connect(self.download_map)
        ctl.addWidget(self.btn_dl)
        self.btn_use = QPushButton("Use & load selected map")
        self.btn_use.setStyleSheet(
            "QPushButton{background:#159957;color:white;border:none;border-radius:7px;padding:9px;font-weight:700;}"
            "QPushButton:hover{background:#118249;}"
        )
        self.btn_use.clicked.connect(self.use_selected_map)
        ctl.addWidget(self.btn_use)
        self.active_map_lbl = QLabel("Active map: —")
        self.active_map_lbl.setStyleSheet(
            "color:#FFFFFF!important;background:#159957;font-size:12px;font-weight:700;"
            "border-radius:7px;padding:9px 10px;")
        ctl.addWidget(self.active_map_lbl)
        self.dl_bar = QProgressBar()
        self.dl_bar.setVisible(False)
        ctl.addWidget(self.dl_bar)
        self.dl_status = QLabel("")
        self.dl_status.setWordWrap(True)
        self.dl_status.setStyleSheet("color:#7B818A!important;font-size:11px;")
        ctl.addWidget(self.dl_status)
        self.btn_diag = QPushButton("Uložiť poslednú diagnostiku trasy")
        self.btn_diag.clicked.connect(self.save_route_diagnostic)
        self.btn_diag.setEnabled(False)
        ctl.addWidget(self.btn_diag)
        ctl.addStretch()
        content.addWidget(controls)

        self.view = MapView(state)
        self.view._pal = self._pal
        self.view.apply_theme()
        self.view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        content.addWidget(self.view, 1)
        layout.addLayout(content, 1)

        self._dl_worker = None
        self._last_active_map_key = None
        self._last_road_segments_revision = None
        self._last_pose_signature = None
        self._populate_maps()
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(250)
        self._last_diag_export_result = None

    def restyle(self, theme):
        """Re-apply palette colours when the theme switches (dark ↔ light)."""
        from core.theme import palette
        self._pal = palette(theme)
        self.title.setStyleSheet("font-size:24px;font-weight:800;color:" + self._pal['text'] + ";")
        self.title_copy.setStyleSheet(
            "color:" + self._pal['muted'] + ";font-size:12px;margin-left:8px;")
        self.controls.setStyleSheet(
            "#NavigationControls{background:" + self._pal['card']
            + ";border:1px solid " + self._pal['border'] + ";border-radius:12px;}")
        self.gps_hint.setStyleSheet(
            "color:" + self._pal['muted'] + ";font-size:11px;")
        for card, cap, value in self.nav_stats.values():
            card.setStyleSheet(
                "QFrame#NavigationStat{background:" + self._pal['card']
                + ";border:1px solid " + self._pal['border']
                + ";border-radius:11px;}")
            cap.setStyleSheet("font-size:9px;font-weight:750;color:"
                              + self._pal['muted'] + ";")
            value.setStyleSheet("font-size:13px;font-weight:700;color:"
                                + self._pal['text'] + ";")
        self.status.setStyleSheet(
            "background:" + self._pal['card2'] + ";color:" + self._pal['muted']
            + ";border-radius:8px;padding:10px;font-size:12px;")
        self.map_title.setStyleSheet("color:#7B818A;font-size:10px;font-weight:700;margin-top:8px;")
        self.active_map_lbl.setStyleSheet(
            "color:#FFFFFF;background:#159957;font-size:12px;font-weight:700;"
            "border-radius:7px;padding:9px 10px;")
        self.dl_status.setStyleSheet("color:#7B818A;font-size:11px;")
        self.view._pal = self._pal
        self.view.apply_theme()

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
            game_version = d.get("game_version") or d["version"]
            if d.get("mod"):
                detail = f"{d['mod']} {d.get('mod_version') or d['version']} · {d['game']} {game_version}"
            else:
                detail = f"{d['game']} {game_version}"
                if d.get("content"):
                    detail += f" · {d['content']}"
            self.map_combo.addItem(f"{mark}{d['key']}  ({detail})", d["key"])
            if d["key"] == wanted:
                sel_idx = i
        if datasets:
            self.map_combo.setCurrentIndex(sel_idx)
            self.dl_status.setText("Pick your game version (or ProMods) and download once.")
        self.map_combo.blockSignals(False)
        self._update_active_map_label()
        self._update_map_actions()

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
        self._update_map_actions()

    def _update_map_actions(self):
        key = self.map_combo.currentData()
        downloaded = False
        try:
            from core.navigation import map_data
            downloaded = bool(key and map_data.is_downloaded(key))
        except Exception:
            pass
        self.btn_dl.setVisible(not downloaded)
        self.btn_use.setEnabled(downloaded)
        local = False
        trucklib_required = False
        if not downloaded:
            try:
                entry = next((item for item in map_data.list_datasets()
                              if item["key"] == key), {})
                local = entry.get("source") == "local-game"
                trucklib_required = entry.get("source") == "trucklib-required"
            except Exception:
                pass
        self.btn_dl.setEnabled(not trucklib_required)
        self.btn_use.setText(
            "Pouzit a nacitat vybranu mapu" if downloaded else
            ("Pre ETS2 1.60 chyba TruckLib exporter" if trucklib_required
             else "Najprv vytvor mapu z nainstalovanej hry" if local
             else "Najprv stiahni mapu"))
        self.btn_dl.setText("Podpora 1.60 sa pripravuje" if trucklib_required
                            else "Vytvorit z nainstalovanej hry" if local
                            else "Stiahnut mapu")

    def use_selected_map(self):
        key = self.map_combo.currentData()
        if not key:
            return
        self._on_map_selected(self.map_combo.currentIndex())
        self.view.set_road_segments([])
        self.state.set("nav_arg", key)
        self.state.set("nav_cmd", "switch_map")
        self.dl_status.setText(f"Loading roads, prefabs and cities from {key}...")
        self.btn_use.setEnabled(False)

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
        if ok:
            self.dl_status.setText("Map data ready - loading road network...")
        else:
            try:
                from core.navigation import map_data
                reason = map_data.last_error()
            except Exception:
                reason = ""
            self.dl_status.setText(
                reason or "Map preparation failed; see the log for details.")
        self._populate_maps()

    def save_route_diagnostic(self):
        result = self.state.get("route_diagnostic_last_result") or {}
        build_id = result.get("route_build_id")
        if not build_id or result.get("status") == "success":
            self.status.setText("No failed route calculation is available.")
            return
        self.state.set("route_diagnostic_export_result", None)
        self.state.set("route_diagnostic_export_request", build_id)
        self.status.setText("Saving anonymized route diagnostic…")

    def refresh(self):
        # The engine may auto-select a compatible dataset after comparing live
        # GPS node UIDs. Reload that same network in the UI process as well.
        active_key = self.state.get("active_map_key")
        repaint = False
        if active_key and active_key != self._last_active_map_key:
            self._last_active_map_key = active_key
            index = self.map_combo.findData(active_key)
            if index >= 0:
                self.map_combo.blockSignals(True)
                self.map_combo.setCurrentIndex(index)
                self.map_combo.blockSignals(False)
            self.view.set_road_segments([])
            repaint = True

        # Fetch the relatively large segment list only when the producer
        # publishes a new atomic revision, rather than on every UI refresh.
        road_revision = self.state.get("map_road_segments_revision", 0)
        if road_revision != self._last_road_segments_revision:
            self._last_road_segments_revision = road_revision
            self.view.set_road_segments(
                self.state.get("map_road_segments", []) or [])
            repaint = True

        truck = self.state.get("truck_world_pos")
        heading = float(self.state.get("truck_heading", 0.0) or 0.0)
        try:
            pose_signature = (round(float(truck[0]), 1),
                              round(float(truck[1]), 1), round(heading, 3))
        except (TypeError, ValueError, IndexError):
            pose_signature = None
        if pose_signature != self._last_pose_signature:
            self._last_pose_signature = pose_signature
            repaint = True

        command_error = rejected_navigation_command_message(self.state)
        if command_error:
            self.status.setText(command_error)
        elif self.state.get("nav_active"):
            dist = self.state.get("distance_to_dest")
            if dist is not None:
                self.status.setText(f"Navigating — {float(dist) / 1000:.2f} km to destination.")
            else:
                self.status.setText("GPS navigation active — following the route selected in game.")
        else:
            # Surface the map-loading status the engine publishes (loading /
            # ready / error) so the user is never left guessing why nav is off.
            ms = self.state.get("map_status")
            if ms:
                self.status.setText(str(ms))
        diagnostic = self.state.get("route_diagnostic_last_result") or {}
        self.btn_diag.setEnabled(bool(
            diagnostic.get("route_build_id")
            and diagnostic.get("status") != "success"))
        export_result = self.state.get("route_diagnostic_export_result")
        if export_result and export_result != self._last_diag_export_result:
            self._last_diag_export_result = dict(export_result)
            if export_result.get("ok"):
                self.status.setText(
                    f"Diagnostic saved: {export_result.get('path', '')}")
            else:
                self.status.setText(str(export_result.get(
                    "message", "Diagnostic could not be saved.")))
        # Keep the active-map badge in sync with whatever the map plugin
        # published (so the user sees the real running map, not just the
        # last selection from the combo).
        self._update_active_map_label()
        gps_active = bool(self.state.get("game_gps_navigation_active", False))
        self.nav_stats["gps"][2].setText("Aktívna" if gps_active else "Čakám na cieľ")
        self.nav_stats["map"][2].setText(str(
            self.state.get("active_map_name")
            or self.state.get("active_map_key") or "Nezvolená"))
        trajectory = self.state.get("lane_trajectory", {}) or {}
        if trajectory.get("valid", False):
            self.nav_stats["trajectory"][2].setText(
                f"Platná  •  rev. {trajectory.get('revision', 0)}")
        else:
            self.nav_stats["trajectory"][2].setText("Čakám na výpočet")
        if repaint or self.state.get("nav_active"):
            self.view.update()
