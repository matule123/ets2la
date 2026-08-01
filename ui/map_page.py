import math
import time

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QProgressBar, QFrame, QSizePolicy,
)
from PyQt6.QtGui import QPainter, QColor, QPen, QPolygonF, QPainterPath
from PyQt6.QtCore import (Qt, QTimer, QPointF, QPoint, QRectF, QThread,
                          pyqtSignal)
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
        self.scene_polygons = []
        self.scene_features = []
        self._pal = None         # set by the page (or a default below)
        # A slightly wider initial field matches the navigation-map reference
        # and uses the broad, display-only 1.2 km scene from the map process.
        self.zoom_radius = 650.0
        self.pan_world = [0.0, 0.0]
        self._drag_at = None
        self.setMinimumHeight(300)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.live_badge = QLabel("●  LIVE", self)
        self.live_badge.setObjectName("LiveMapBadge")
        self.live_badge.setStyleSheet(
            "background:rgba(19,24,28,232);color:#35C779;"
            "border:1px solid #34383C;border-radius:9px;padding:5px 9px;"
            "font-size:11px;font-weight:750;")
        self.live_badge.adjustSize()
        self.turn_banner = QLabel("", self)
        self.turn_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.turn_banner.setStyleSheet(
            "background:#119B5B;color:white;border:none;border-radius:12px;"
            "padding:12px 22px;font-size:20px;font-weight:800;")
        self.turn_banner.hide()
        self.map_credit = QLabel("UltraPilot Maps", self)
        self.map_credit.setStyleSheet(
            "background:transparent;color:rgba(220,223,226,115);"
            "border:none;font-size:9px;")
        self.map_credit.adjustSize()
        self.map_controls = []
        for caption, tooltip, callback in (
                ("+", "Priblížiť", self.zoom_in),
                ("−", "Oddialiť", self.zoom_out),
                ("⌖", "Vycentrovať na kamión", self.reset_view)):
            button = QPushButton(caption, self)
            button.setToolTip(tooltip)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
            button.setFixedSize(32, 30)
            button.setStyleSheet(
                "QPushButton{background:rgba(31,34,37,235);color:#E7E9EB;"
                "border:1px solid #484C50;border-radius:7px;font-size:17px;"
                "font-weight:700;padding:0;}"
                "QPushButton:hover{background:#3B4045;border-color:#6A7076;}")
            button.clicked.connect(callback)
            self.map_controls.append(button)
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
        for index, button in enumerate(self.map_controls):
            button.move(max(8, self.width() - 44), 14 + index * 35)
            button.raise_()
        self.map_credit.move(
            max(8, self.width() - self.map_credit.width() - 12),
            max(8, self.height() - self.map_credit.height() - 8))
        self.map_credit.raise_()
        self.turn_banner.move(
            max(54, (self.width() - self.turn_banner.width()) // 2), 14)
        self.turn_banner.raise_()
        self.empty_state.move(max(14, (self.width() - self.empty_state.width()) // 2),
                              max(52, (self.height() - self.empty_state.height()) // 2))
        self.empty_state.raise_()

    def reset_view(self):
        self.zoom_radius = 650.0
        self.pan_world[:] = [0.0, 0.0]
        self.update()

    def zoom_in(self):
        self.zoom_radius = max(70.0, self.zoom_radius * 0.78)
        self.update()

    def zoom_out(self):
        self.zoom_radius = min(1400.0, self.zoom_radius * 1.28)
        self.update()

    def wheelEvent(self, event):
        # Wheel up zooms in, wheel down zooms out to a broad regional view.
        factor = 0.78 if event.angleDelta().y() > 0 else 1.28
        self.zoom_radius = max(70.0, min(1400.0, self.zoom_radius * factor))
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
        """Keep finite display geometry and all road-style metadata.

        These fields are display-only.  Preserving them fixes the former live
        map which reduced every motorway and prefab to the same two-pixel line.
        """
        segments = []
        for item in (payload or [])[:6500]:
            try:
                a, b = item[0], item[1]
                values = float(a[0]), float(a[1]), float(b[0]), float(b[1])
                if not all(math.isfinite(value) for value in values):
                    continue
                segments.append({
                    "a": values[:2], "b": values[2:],
                    "kind": str(item[2]) if len(item) > 2 else "road",
                    "lanes": max(1, int(item[3])) if len(item) > 3 else 2,
                    "divided": bool(item[4]) if len(item) > 4 else False,
                    "dash_on": bool(item[5]) if len(item) > 5 else True,
                    "half_width": max(1.5, float(item[8]))
                    if len(item) > 8 and item[8] is not None else 4.8,
                    "suppress_markings": bool(item[9])
                    if len(item) > 9 else False,
                    "path_key": str(item[10]) if len(item) > 10 else "",
                    "path_index": int(item[11]) if len(item) > 11 else 0,
                    "road_type": str(item[12]) if len(item) > 12 else (
                        "divided" if len(item) > 4 and bool(item[4])
                        else "local"),
                })
            except (TypeError, ValueError, IndexError):
                continue
        self.road_segments = segments

    def set_scene_polygons(self, payload):
        polygons = []
        for item in (payload or [])[:1400]:
            try:
                points = []
                for point in item[0]:
                    x, z = float(point[0]), float(point[1])
                    if not math.isfinite(x) or not math.isfinite(z):
                        raise ValueError("non-finite polygon point")
                    points.append((x, z))
                if len(points) >= 3:
                    polygons.append({
                        "points": tuple(points),
                        "colour": max(0, min(8, int(item[1]))),
                        "z_index": int(item[2]),
                    })
            except (TypeError, ValueError, IndexError, OverflowError):
                continue
        self.scene_polygons = sorted(
            polygons, key=lambda polygon: polygon["z_index"])

    def set_scene_features(self, payload):
        features = []
        for item in (payload or [])[:800]:
            try:
                x, z = float(item[0]), float(item[1])
                if not math.isfinite(x) or not math.isfinite(z):
                    continue
                features.append({
                    "pos": (x, z), "kind": str(item[2]),
                    "icon": str(item[3]), "label": str(item[4]),
                })
            except (TypeError, ValueError, IndexError, OverflowError):
                continue
        self.scene_features = features

    def _bounds(self, pts):
        xs = [p[0] for p in pts]
        zs = [p[1] for p in pts]
        return min(xs), max(xs), min(zs), max(zs)

    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor("#1A1A1A"))
        qp.drawRoundedRect(self.rect(), 10, 10)

        truck = self.state.get("truck_world_pos")
        heading = self.state.get("truck_heading", 0.0) or 0.0

        # Truck-centered view from the engine-published local map snapshot.
        if self.road_segments and truck:
            self.empty_state.hide()
            self._paint_map(qp, w, h, truck, heading)
            return

        self.turn_banner.hide()
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
        """Layered live map ported from truckermudgeon/maps.

        The palette and ordering mirror ``GameMapStyle.tsx``: prefab map-area
        polygons, road casing, road fill, route and finally POI symbols.
        """
        radius = self.zoom_radius
        scale = (min(w, h) - 20) / (2 * radius)
        cx = truck[0] + self.pan_world[0]
        cz = truck[1] + self.pan_world[1]

        def to_screen(p):
            sx = w / 2 + (p[0] - cx) * scale
            sy = h / 2 - (cz - p[1]) * scale   # flip Z so north is up
            return QPointF(sx, sy)

        # Exact dark MapArea palette from the downloaded maps renderer.  The
        # geometry is made of real placed-prefab neighbour loops.
        map_area_colours = {
            0: QColor.fromHsl(200, 5, 92),
            1: QColor.fromHsl(38, 64, 89),
            2: QColor.fromHsl(38, 64, 64),
            3: QColor.fromHsl(143, 51, 64),
            4: QColor.fromHsl(0, 255, 64),
            5: QColor.fromHsl(107, 130, 64),
            6: QColor.fromHsl(201, 135, 64),
            7: QColor.fromHsl(53, 214, 64),
            8: QColor.fromHsl(267, 117, 64),
        }
        qp.setPen(Qt.PenStyle.NoPen)
        for polygon in self.scene_polygons:
            points = polygon["points"]
            if not any(cx-radius*1.45 <= point[0] <= cx+radius*1.45
                       and cz-radius*1.45 <= point[1] <= cz+radius*1.45
                       for point in points):
                continue
            qp.setBrush(map_area_colours.get(
                polygon["colour"], QColor("#30302F")))
            qp.drawPolygon(QPolygonF([to_screen(point) for point in points]))

        visible = []
        for segment in self.road_segments:
            a, b = segment["a"], segment["b"]
            if (max(a[0], b[0]) < cx - radius * 1.35
                    or min(a[0], b[0]) > cx + radius * 1.35
                    or max(a[1], b[1]) < cz - radius * 1.35
                    or min(a[1], b[1]) > cz + radius * 1.35):
                continue
            visible.append((segment, to_screen(a), to_screen(b)))

        # Exact dark road palette from GameMapStyle.tsx: [fill, casing].
        road_colours = {
            "freeway": (QColor("#95813E"), QColor("#372F21")),
            "divided": (QColor("#3C4043"), QColor("#4C5043")),
            "no_vehicles": (QColor("#606166"), QColor("#888888")),
            "local": (QColor("#606166"), QColor("#333333")),
        }
        for segment, a, b in visible:
            width = max(2.4, min(44.0,
                2.0 * float(segment["half_width"]) * scale))
            _surface, casing = road_colours.get(
                segment["road_type"], road_colours["local"])
            qp.setPen(QPen(casing, width + 2.4, Qt.PenStyle.SolidLine,
                           Qt.PenCapStyle.RoundCap,
                           Qt.PenJoinStyle.RoundJoin))
            qp.drawLine(a, b)
        for segment, a, b in visible:
            width = max(1.8, min(42.0,
                2.0 * float(segment["half_width"]) * scale))
            surface, _casing = road_colours.get(
                segment["road_type"], road_colours["local"])
            qp.setPen(QPen(surface, width, Qt.PenStyle.SolidLine,
                           Qt.PenCapStyle.RoundCap,
                           Qt.PenJoinStyle.RoundJoin))
            qp.drawLine(a, b)

        # GPS uses only current-revision snapshot geometry. Recorded replay is
        # admitted only by live_map_navigation_points() in its exclusive mode.
        ahead = live_map_navigation_points(self.state)
        ahead = [point for point in (self._to_xz(point) for point in ahead)
                 if point is not None]
        if len(ahead) >= 2:
            polyline = QPolygonF([to_screen(p) for p in ahead])
            qp.setPen(QPen(QColor("#075F9C"), 10, Qt.PenStyle.SolidLine,
                           Qt.PenCapStyle.RoundCap,
                           Qt.PenJoinStyle.RoundJoin))
            qp.drawPolyline(polyline)
            qp.setPen(QPen(QColor("#079AF0"), 6, Qt.PenStyle.SolidLine,
                           Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            qp.drawPolyline(polyline)

        self._paint_scene_features(qp, to_screen, cx, cz, radius, scale)

        snapshot = self.state.get("lane_trajectory", {}) or {}
        events = snapshot.get("turn_events", []) or []
        if events and snapshot.get("valid", False):
            event = events[0]
            try:
                distance = max(
                    0.0, float(event.get("start_s_m", 0.0) or 0.0))
            except (TypeError, ValueError, OverflowError):
                distance = 0.0
            arrow = "↱" if event.get("direction") == "right" else "↰"
            shown = (f"{distance / 1000:.1f} km" if distance >= 1000
                     else f"{int(round(distance / 10.0) * 10)} m")
            self.turn_banner.setText(f"{arrow}   {shown}")
            self.turn_banner.adjustSize()
            self.turn_banner.move(
                max(54, (self.width() - self.turn_banner.width()) // 2), 14)
            self.turn_banner.show()
            self.turn_banner.raise_()
        else:
            self.turn_banner.hide()

        # Crisp maps-style position marker at the exact telemetry position.
        c = to_screen(truck)
        fx, fz = -math.sin(heading), -math.cos(heading)
        tip = QPointF(c.x() + fx * 16, c.y() - fz * 16)
        left = QPointF(c.x() - fz * 8 + fx * -7, c.y() - fx * 8 - fz * -7)
        right = QPointF(c.x() + fz * 8 + fx * -7, c.y() + fx * 8 - fz * -7)
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(0, 0, 0, 75))
        qp.drawEllipse(QPointF(c.x()+1.5, c.y()+2.0), 15, 15)
        qp.setBrush(QColor("#FFFFFF"))
        qp.drawEllipse(c, 14, 14)
        qp.setBrush(QColor("#1597F5"))
        qp.drawEllipse(c, 11.5, 11.5)
        qp.setPen(QPen(QColor("#FFFFFF"), 1.2,
                       Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap,
                       Qt.PenJoinStyle.RoundJoin))
        qp.setBrush(QColor("#FFFFFF"))
        qp.drawPolygon(QPolygonF([tip, left, right]))

        # Small unobtrusive interaction hint; no extra toolbar is needed.
        qp.setPen(QColor(185, 190, 198, 145))
        qp.drawText(14, h - 12,
                    "koliesko: zoom  •  potiahnuť: posun  •  dvojklik: kamión")

    def _paint_scene_features(self, qp, to_screen, cx, cz, radius, scale):
        """Paint maps-style POI symbols and labels from exported SCS data."""
        icon_colours = {
            "gas": QColor("#00A84F"), "fuel": QColor("#00A84F"),
            "parking": QColor("#1675C1"), "service": QColor("#D29B13"),
            "dealer": QColor("#D29B13"), "garage": QColor("#1675C1"),
            "train": QColor("#1675C1"), "ferry": QColor("#1675C1"),
            "toll": QColor("#00A84F"), "weigh": QColor("#7D8B95"),
        }
        occupied = []
        for feature in self.scene_features:
            x, z = feature["pos"]
            if abs(x-cx) > radius*1.35 or abs(z-cz) > radius*1.35:
                continue
            point = to_screen((x, z))
            kind = feature["kind"].lower()
            icon = feature["icon"].lower()
            label = feature["label"]
            if kind == "city":
                if label and radius >= 320:
                    qp.setPen(QColor("#DCD9D2"))
                    font = qp.font()
                    font.setBold(True)
                    font.setPointSize(9)
                    qp.setFont(font)
                    qp.drawText(QPointF(point.x()+7, point.y()-5), label)
                continue

            if kind == "company":
                colour = QColor("#B99A43")
            else:
                key = next((name for name in icon_colours if name in icon), "")
                colour = icon_colours.get(key, QColor("#65717B"))
            if any(abs(point.x()-other.x()) < 17 and
                   abs(point.y()-other.y()) < 17 for other in occupied):
                continue
            occupied.append(point)
            size = 13 if scale < .55 else 16
            rect_x, rect_y = point.x()-size/2, point.y()-size/2
            qp.setPen(QPen(QColor("#F3F5F7"), 1.0))
            qp.setBrush(colour)
            qp.drawRoundedRect(int(rect_x), int(rect_y), size, size, 2, 2)
            self._paint_feature_symbol(
                qp, QRectF(rect_x, rect_y, size, size), icon, kind)
            if kind == "company" and label and radius <= 650:
                qp.setPen(QColor("#DCD9D2"))
                font = qp.font()
                font.setBold(True)
                font.setPointSize(7)
                qp.setFont(font)
                qp.drawText(QPointF(point.x()+size/2+3, point.y()+3), label)

    @staticmethod
    def _paint_feature_symbol(qp, rect, icon, kind):
        """Draw compact vector POI pictograms; never raw placeholder letters."""
        qp.save()
        qp.setBrush(Qt.BrushStyle.NoBrush)
        pen = QPen(QColor("#FFFFFF"), max(1.0, rect.width() / 11.0),
                   Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap,
                   Qt.PenJoinStyle.RoundJoin)
        qp.setPen(pen)
        x, y, w, h = rect.x(), rect.y(), rect.width(), rect.height()
        key = f"{kind} {icon}".lower()
        if any(value in key for value in ("gas", "fuel")):
            qp.drawRoundedRect(QRectF(x+w*.25, y+h*.20, w*.38, h*.60),
                               w*.04, w*.04)
            qp.drawLine(QPointF(x+w*.32, y+h*.38),
                        QPointF(x+w*.56, y+h*.38))
            hose = QPainterPath(QPointF(x+w*.63, y+h*.31))
            hose.cubicTo(x+w*.86, y+h*.32, x+w*.79, y+h*.72,
                         x+w*.72, y+h*.72)
            qp.drawPath(hose)
        elif any(value in key for value in ("service", "garage", "dealer")):
            qp.drawLine(QPointF(x+w*.27, y+h*.72),
                        QPointF(x+w*.73, y+h*.26))
            qp.drawEllipse(QPointF(x+w*.27, y+h*.72), w*.09, w*.09)
            qp.drawLine(QPointF(x+w*.64, y+h*.22),
                        QPointF(x+w*.79, y+h*.37))
        elif "train" in key:
            qp.drawRoundedRect(QRectF(x+w*.24, y+h*.18, w*.52, h*.54),
                               w*.08, w*.08)
            qp.drawLine(QPointF(x+w*.34, y+h*.36),
                        QPointF(x+w*.66, y+h*.36))
            qp.drawEllipse(QPointF(x+w*.34, y+h*.76), w*.06, w*.06)
            qp.drawEllipse(QPointF(x+w*.66, y+h*.76), w*.06, w*.06)
        elif "ferry" in key:
            boat = QPainterPath(QPointF(x+w*.18, y+h*.58))
            boat.lineTo(x+w*.82, y+h*.58)
            boat.lineTo(x+w*.69, y+h*.76)
            boat.lineTo(x+w*.31, y+h*.76)
            boat.closeSubpath()
            qp.drawPath(boat)
            qp.drawLine(QPointF(x+w*.38, y+h*.58),
                        QPointF(x+w*.38, y+h*.34))
            qp.drawLine(QPointF(x+w*.38, y+h*.34),
                        QPointF(x+w*.63, y+h*.45))
        elif any(value in key for value in ("toll", "weigh")):
            qp.drawLine(QPointF(x+w*.24, y+h*.72),
                        QPointF(x+w*.24, y+h*.27))
            qp.drawLine(QPointF(x+w*.76, y+h*.72),
                        QPointF(x+w*.76, y+h*.27))
            qp.drawLine(QPointF(x+w*.20, y+h*.29),
                        QPointF(x+w*.80, y+h*.29))
            qp.drawLine(QPointF(x+w*.36, y+h*.29),
                        QPointF(x+w*.36, y+h*.72))
        elif kind == "company":
            roof = QPainterPath(QPointF(x+w*.18, y+h*.43))
            roof.lineTo(x+w*.50, y+h*.20)
            roof.lineTo(x+w*.82, y+h*.43)
            qp.drawPath(roof)
            qp.drawRect(QRectF(x+w*.24, y+h*.43, w*.52, h*.36))
            qp.drawRect(QRectF(x+w*.43, y+h*.58, w*.16, h*.21))
        else:
            # Parking/rest-area symbol: a car silhouette instead of a raw P.
            car = QPainterPath(QPointF(x+w*.20, y+h*.61))
            car.lineTo(x+w*.30, y+h*.40)
            car.lineTo(x+w*.68, y+h*.40)
            car.lineTo(x+w*.80, y+h*.61)
            qp.drawPath(car)
            qp.drawLine(QPointF(x+w*.20, y+h*.61),
                        QPointF(x+w*.80, y+h*.61))
            qp.drawEllipse(QPointF(x+w*.31, y+h*.69), w*.06, w*.06)
            qp.drawEllipse(QPointF(x+w*.69, y+h*.69), w*.06, w*.06)
        qp.restore()

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
        self.dl_bar.setRange(0, 100)
        self.dl_bar.setFormat("%p %")
        self.dl_bar.setTextVisible(True)
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
        self._last_live_map_scene_revision = None
        self._last_pose_signature = None
        self._populate_maps()
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(250)
        self._last_diag_export_result = None
        self._last_map_load_generation = None

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
            "Použiť a načítať vybranú mapu" if downloaded else
            ("Pre ETS2 1.60 chýba TruckLib exportér" if trucklib_required
             else "Najprv vytvor mapu z nainštalovanej hry" if local
             else "Najprv stiahni mapu"))
        self.btn_dl.setText("Podpora 1.60 sa pripravuje" if trucklib_required
                            else "Vytvoriť z nainštalovanej hry" if local
                            else "Stiahnuť mapu")

    def use_selected_map(self):
        key = self.map_combo.currentData()
        if not key:
            return
        self._on_map_selected(self.map_combo.currentIndex())
        self.view.set_road_segments([])
        self.view.set_scene_polygons([])
        self.view.set_scene_features([])
        self.state.set("nav_arg", key)
        self.state.set("nav_cmd", "switch_map")
        self.dl_status.clear()
        self.dl_bar.hide()
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
            self.dl_status.setText("Mapové dáta sú pripravené — načítavam cesty…")
        else:
            try:
                from core.navigation import map_data
                reason = map_data.last_error()
            except Exception:
                reason = ""
            self.dl_status.setText(
                reason or "Príprava mapy zlyhala; podrobnosti sú v logu.")
        self._populate_maps()

    def save_route_diagnostic(self):
        result = self.state.get("route_diagnostic_last_result") or {}
        build_id = result.get("route_build_id")
        if not build_id or result.get("status") == "success":
            self.status.setText("Nie je dostupný žiadny neúspešný výpočet trasy.")
            return
        self.state.set("route_diagnostic_export_result", None)
        self.state.set("route_diagnostic_export_request", build_id)
        self.status.setText("Ukladám anonymizovanú diagnostiku trasy…")

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
            self.view.set_scene_polygons([])
            self.view.set_scene_features([])
            repaint = True

        # Roads, true prefab map polygons and POIs are one atomic display
        # scene.  Older shared state falls back to the historical HUD roads.
        scene_revision = self.state.get("live_map_scene_revision")
        if scene_revision is not None:
            if scene_revision != self._last_live_map_scene_revision:
                self._last_live_map_scene_revision = scene_revision
                self.view.set_road_segments(
                    self.state.get("live_map_road_segments", []) or [])
                self.view.set_scene_polygons(
                    self.state.get("live_map_scene_polygons", []) or [])
                self.view.set_scene_features(
                    self.state.get("live_map_scene_features", []) or [])
                repaint = True
        else:
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

        # Downloads keep their local progress here. Engine-side parsing is
        # shown exclusively by Dynamic Island so the navigation card cannot
        # retain a stale 2% bar after a cache load or plugin restart.
        load_progress = self.state.get("map_load_progress", {}) or {}
        if self._dl_worker is None and load_progress:
            active = bool(load_progress.get("active", False))
            generation = load_progress.get("generation")
            self.dl_bar.hide()
            self.dl_status.clear()
            if generation is not None:
                self._last_map_load_generation = generation
            self.btn_use.setEnabled(not active)

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
