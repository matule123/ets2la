import sys
import math
import logging
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QPainter, QColor, QFont, QPen, QPolygonF, QRadialGradient, QLinearGradient, QBrush
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


# Body colours cycled by vehicle id, so traffic isn't all grey. Each is a
# (body, roof) pair; the roof (cabin/glass) is rendered darker for contrast.
_CAR_PALETTE = [
    ("#3B82F6", "#1E40AF"),  # blue
    ("#EF4444", "#991B1B"),  # red
    ("#F59E0B", "#B45309"),  # amber
    ("#10B981", "#047857"),  # green
    ("#8B5CF6", "#5B21B6"),  # purple
    ("#E5E7EB", "#9CA3AF"),  # white
    ("#374151", "#111827"),  # dark grey
    ("#EC4899", "#9D174D"),  # pink
]


def _car_colour(v):
    """Stable (body, roof) colour pair for a vehicle, keyed on its id."""
    vid = v.get("id", 0) or 0
    return _CAR_PALETTE[int(vid) % len(_CAR_PALETTE)]


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

    # Left-panel size — larger so the 3D scene + truck models read clearly.
    W, H = 560, 640

    def __init__(self, shared_state):
        super().__init__()
        self.shared_state = shared_state
        self._blink = True
        self._drag = None
        self._t = 0.0          # animation clock (seconds) for moving lane dashes
        self._shown = False   # becomes True once the UI process signals ready
        self.init_ui()

    def init_ui(self):
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint |
                            Qt.WindowType.WindowStaysOnTopHint | Qt.WindowType.Tool)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.resize(self.W, self.H)
        screen = QApplication.primaryScreen().geometry()
        # Bottom-left of the screen (the reference HUD is on the left side).
        self.move(24, screen.height() - self.H - 24)
        # Start hidden: the HUD only appears once the main app window is up
        # (``ui_ready`` flag in shared state). This avoids the HUD flashing on
        # screen during the onboarding wizard / before the dashboard is visible.
        self.hide()
        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(80)   # ~12 fps animation

    def _tick(self):
        self._blink = not self._blink
        self._t += 0.08   # advance the animation clock (timer fires every 80 ms)
        # Wait for the UI process to flag itself ready before we show the HUD.
        try:
            if not self._shown and self.shared_state.get("ui_ready", False):
                self._shown = True
                self.show()
        except Exception:
            pass
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
            # Total lane count on the road under the truck (drives the road
            # width in the 3D scene — 2 lanes default when unknown).
            "lanes": int(s.get("road_lanes", 2) or 2),
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

    @staticmethod
    def _smooth_path(pts, samples=6):
        """Catmull-Rom spline through ``pts`` (list of (ahead, lateral)).

        Produces a much denser, smoothly-curving polyline so bends read as
        real curves instead of a kinked zig-zag of a few points. ``samples``
        is the number of interpolated points inserted between each pair."""
        n = len(pts)
        if n < 2:
            return list(pts)
        out = []
        for i in range(n - 1):
            p0 = pts[max(0, i - 1)]
            p1 = pts[i]
            p2 = pts[i + 1]
            p3 = pts[min(n - 1, i + 2)]
            for t in range(samples + 1):
                f = t / samples
                f2 = f * f
                f3 = f2 * f
                a = 0.5 * (2 * p1[0] + (-p0[0] + p2[0]) * f +
                           (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * f2 +
                           (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * f3)
                b = 0.5 * (2 * p1[1] + (-p0[1] + p2[1]) * f +
                           (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * f2 +
                           (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * f3)
                out.append((a, b))
        return out

    def _draw_driving_view(self, qp, view, d):
        # Slightly darker ground band below the horizon for depth.
        horizon_y = view.top() + view.height() * 0.26
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(9, 11, 14, 235))
        qp.drawRect(QRectF(view.left(), horizon_y, view.width(), view.bottom() - horizon_y))

        pos, h = d["pos"], d["heading"]

        def to_truck(wx, wz):
            dx, dz = wx - pos[0], wz - pos[1]
            ahead = dx * (-math.sin(h)) + dz * (-math.cos(h))
            lateral = dx * math.cos(h) - dz * math.sin(h)
            return ahead, lateral

        if pos:
            path = d["nav_path"]
            raw = [pt for pt in (to_truck(px, pz) for px, pz in path)
                   if 0.5 <= pt[0] <= 140.0]
            # Smooth the path so curves are continuous (no hard kinks).
            al = self._smooth_path(raw) if len(raw) >= 2 else raw

            # Dynamic half-width from the live lane count (not a fixed 2 lanes).
            # ~3.6 m per lane + 1.5 m shoulder on each side, clamped sanely.
            lanes = max(1, d.get("lanes", 2))
            HALF = max(4.0, min(16.0, lanes * 3.6 / 2 + 1.5))

            def offset_pt(i, off):
                a, l = al[i]
                j = min(i + 1, len(al) - 1)
                da, dl = al[j][0] - a, al[j][1] - l
                n = math.hypot(da, dl) or 1.0
                return self._project(a, l + (-da / n) * off, view)

            def polyline_at(off):
                return [p for p in (offset_pt(i, off) for i in range(len(al))) if p]

            if len(al) >= 2:
                # 1) Filled asphalt ribbon (left edge → right edge), curving.
                left = polyline_at(-HALF)
                right = polyline_at(HALF)
                ribbon = left + list(reversed(right))
                if len(ribbon) >= 3:
                    qp.setPen(Qt.PenStyle.NoPen)
                    qp.setBrush(QColor(24, 27, 31, 250))
                    qp.drawPolygon(QPolygonF(ribbon))
                # 2) Solid white edge lines.
                for off in (-HALF, HALF):
                    pts = polyline_at(off)
                    if len(pts) >= 2:
                        qp.setPen(QPen(QColor(240, 240, 245, 210), 2, Qt.PenStyle.SolidLine))
                        qp.drawPolyline(QPolygonF(pts))
                # 3) Dashed lane dividers — one fewer line than the lane count,
                #    placed symmetrically so a 4-lane road shows 3 dividers etc.
                #    The dashes ANIMATE (scroll toward the truck) so the road
                #    reads as moving, driven by the _tick animation clock.
                if lanes >= 2:
                    spacing = (2 * HALF) / lanes
                    first = -HALF + spacing
                    # Move the dashes faster the quicker we go (visual speed cue).
                    spd = max(0.4, min(4.0, d.get("speed_kmh", 0) / 25.0))
                    dash_off = -(self._t * spd * 6) % 12
                    pen = QPen(QColor(225, 228, 233, 205), 2.2, Qt.PenStyle.DashLine)
                    pen.setDashPattern([3, 3])
                    pen.setDashOffset(dash_off)
                    for k in range(lanes - 1):
                        off = first + k * spacing
                        pts = polyline_at(off)
                        if len(pts) >= 2:
                            qp.setPen(pen)
                            qp.drawPolyline(QPolygonF(pts))
                # 4) Anticipated route (blue) glow on the road.
                pts = [self._project(a, l, view) for a, l in al]
                pts = [p for p in pts if p is not None]
                if len(pts) >= 2:
                    qp.setPen(QPen(QColor(59, 130, 246, 80), 10))
                    qp.drawPolyline(QPolygonF(pts))
                    qp.setPen(QPen(QColor("#3B82F6"), 4))
                    qp.drawPolyline(QPolygonF(pts))
            else:
                # No route: a straight filled ribbon ahead so the road is visible.
                left = [self._project(a, -HALF, view) for a in range(2, 90, 6)]
                right = [self._project(a, HALF, view) for a in range(2, 90, 6)]
                ribbon = [p for p in left if p] + [p for p in reversed(right) if p]
                if len(ribbon) >= 3:
                    qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor(36, 40, 46, 220))
                    qp.drawPolygon(QPolygonF(ribbon))
                for lat, dash in ((-HALF, False), (HALF, False), (0.0, True)):
                    pts = [self._project(a, lat, view) for a in range(2, 90, 6)]
                    pts = [p for p in pts if p is not None]
                    if len(pts) >= 2:
                        st = Qt.PenStyle.DashLine if dash else Qt.PenStyle.SolidLine
                        qp.setPen(QPen(QColor(240, 240, 245, 190), 2, st))
                        qp.drawPolyline(QPolygonF(pts))

            # Surrounding vehicles as solid 3D models (far → near for overlap).
            vehs = []
            for v in d["traffic"]:
                a, l = to_truck(v["x"], v["z"])
                if -6 < a < 80 and abs(l) < (HALF + 6):
                    vehs.append((a, l, v))
            vehs.sort(key=lambda t: -t[0])
            for a, l, v in vehs:
                self._draw_box(qp, view, a, l, v)

        # Ego truck as a 3D model at the bottom centre (cab + trailer).
        self._draw_ego_truck(qp, view, d["heading"])

    def _draw_ego_truck(self, qp, view, heading):
        """The player's articulated truck: a detailed cab + a long box trailer.

        Drawn fixed at the bottom centre of the scene (chase-cam). Richer than
        the generic traffic model: chassis with wheel shadows, a tinted-glass
        cab, exhaust stack, and a tall trailer box with a visible roof line."""
        # Geometry (metres, truck-space): cab at the front, trailer behind.
        cab_h = 2.9
        trailer_h = 3.7
        hw = 1.25            # half truck width
        # Keep the complete rig in front of the chase camera. The former
        # negative coordinates put the trailer almost behind the projection
        # plane and expanded it into a huge distorted trapezoid.
        tr_n, tr_f = 3.5, 13.8
        cab_n, cab_f = 13.4, 18.2

        body = "#B8BCC4"        # cab body — slightly cooler silver
        body_dark = "#6E747C"
        cab_glass = "#102833"   # deep tinted windshield
        chassis = "#2E3239"
        trailer_body = "#C8CCD2"  # trailer — a touch lighter than the cab
        trailer_dark = "#838A93"

        # --- Chassis slab under everything (ground presence + wheel shadow). ---
        self._box3d(qp, tr_n - 0.3, cab_f + 0.3, hw + 0.2, 0.0, 0.0, 0.45, view,
                    (chassis, chassis))

        # --- Wheels: dark cylinders peeking below the chassis (axle pairs). ---
        wheel_h = 0.95
        for ax in (cab_n + 0.9, cab_f - 0.7, tr_n + 1.2, tr_f - 1.2):
            for side in (-1, 1):
                self._box3d(qp, ax - 0.35, ax + 0.35, 0.22,
                            side * (hw + 0.16), 0.0, wheel_h, view,
                            ("#101216", "#2A2E35"))

        # --- Trailer box (long, tall) — the big rectangular cargo body. ---
        self._box3d(qp, tr_n, tr_f, hw, 0.0, 0.45, trailer_h, view, (trailer_body, "#E5E7EB"))
        # Trailer roof trim (slightly raised rail along the top).
        self._box3d(qp, tr_n + 0.2, tr_f - 0.2, hw * 0.9, 0.0, trailer_h, trailer_h + 0.18,
                    view, (trailer_dark, trailer_dark))

        # --- Fifth-wheel gap (the hitch between cab and trailer). ---
        self._box3d(qp, tr_f - 0.3, cab_n + 0.3, hw * 0.5, 0.0, 0.45, 0.9, view,
                    (chassis, chassis))

        # --- Cab: taller, with a tinted windshield + side windows. ---
        self._box3d(qp, cab_n, cab_f, hw, 0.0, 0.45, cab_h, view, (body, "#E5E7EB"))
        # Greenhouse: tinted glass band wrapping the upper cab.
        self._box3d(qp, cab_n + 0.15, cab_f - 0.15, hw * 0.92, 0.0, cab_h * 0.55, cab_h + 0.35,
                    view, (cab_glass, "#1B3A44"))
        # Exhaust stack (thin dark column behind the cab, right side).
        self._box3d(qp, cab_n - 0.1, cab_n + 0.2, 0.18, hw - 0.25, 0.5, cab_h + 0.6,
                    view, ("#2A2E33", "#3A3F47"))

        # --- Head lamps (warm) at the front bumper + tail lamps (red) at trailer back. ---
        self._draw_lights(qp, view, cab_n, cab_f, hw, 0.0, cab_h)
        # Trailer rear lights.
        self._draw_lights(qp, view, tr_n, tr_f, hw, 0.0, trailer_h)

    def _box3d(self, qp, n, fr, hw, lateral, z0, z1, view, faces):
        """Draw one shaded cuboid between heights z0..z1. faces=(side,top) colors.

        Stronger per-face shading than before (back/front much darker, top
        lighter) so the volume reads as a solid 3D object instead of a flat /
        "deravý" outline."""
        c = [self._project(n, lateral - hw, view, z0), self._project(n, lateral + hw, view, z0),
             self._project(fr, lateral - hw, view, z0), self._project(fr, lateral + hw, view, z0),
             self._project(n, lateral - hw, view, z1), self._project(n, lateral + hw, view, z1),
             self._project(fr, lateral - hw, view, z1), self._project(fr, lateral + hw, view, z1)]
        if any(p is None for p in c):
            return False
        bl, br, fl, fr_, blt, brt, flt, frt = c
        side, top = faces
        # Thin dark seam between volumes — reads as a crisp edge, not a hole.
        qp.setPen(QPen(QColor("#15181C"), 1))
        # Faces ordered back→front with increasing brightness for depth.
        qp.setBrush(QColor(side).darker(150)); qp.drawPolygon(QPolygonF([bl, br, brt, blt]))   # back (darkest)
        qp.setBrush(QColor(side).darker(118)); qp.drawPolygon(QPolygonF([bl, fl, flt, blt]))   # left
        qp.drawPolygon(QPolygonF([br, fr_, frt, brt]))                                         # right
        qp.setBrush(QColor(side).darker(132)); qp.drawPolygon(QPolygonF([fl, fr_, frt, flt]))  # front
        qp.setBrush(QColor(top).lighter(112)); qp.drawPolygon(QPolygonF([blt, brt, frt, flt])) # top (lightest)
        return True

    def _draw_box(self, qp, view, ahead, lateral, v):
        """A fuller, single-piece vehicle model.

        Instead of two stacked cubes (body + cabin) that left visible seams, this
        sculpts the car from overlapping volumes so it reads as one solid shape:
          • a low floor/underbody slab (gives it ground presence, no floating)
          • the main body, full-length and solid
          • a green-tinted cabin/greenhouse (windows) set on top, smaller
          • head lamps (warm) at the front, tail lamps (red) at the back
        Drawn far→near so closer cars overlap correctly. All solid, no gaps.
        """
        t = v.get("type", "car")
        body_h = {"car": 1.2, "van": 1.8, "bus": 2.8, "truck": 2.6}.get(t, 1.3)
        hw = max(0.9, v.get("width", 2.0) / 2)
        ln = max(3.5, v.get("length", 4.5))
        n, fr = ahead - ln / 2, ahead + ln / 2
        body, body_dark = _car_colour(v)
        glass = "#0F2A33"     # dark tinted glass

        # Floor slab: wider/longer than the body so it pokes out a touch = wheels
        # region, grounding the model so it never looks like it floats/has a hole.
        self._box3d(qp, n - 0.2, fr + 0.2, hw + 0.15, lateral, 0.0, 0.35, view,
                    (body_dark, body_dark))

        if t == "truck":
            # Tractor unit (front) + cargo box (back), as one connected shape.
            self._box3d(qp, n, fr - ln * 0.35, hw, lateral, 0.35, body_h, view, (body, "#E5E7EB"))
            self._box3d(qp, fr - ln * 0.40, fr - 0.2, hw * 0.96, lateral, 0.35, body_h + 1.0,
                        view, (body_dark, body_dark))
            # Cab on the tractor.
            self._box3d(qp, fr - ln * 0.40, fr - 0.3, hw * 0.9, lateral, body_h, body_h + 0.8,
                        view, (glass, "#1B3A44"))
        elif t == "bus":
            self._box3d(qp, n, fr, hw, lateral, 0.35, body_h, view, (body, "#E5E7EB"))
            # Full-length tinted window band.
            self._box3d(qp, n + 0.3, fr - 0.3, hw * 0.92, lateral, body_h * 0.45, body_h + 0.5,
                        view, (glass, "#1B3A44"))
        else:
            # Car / van: one solid body + a cabin greenhouse set into the middle.
            self._box3d(qp, n, fr, hw, lateral, 0.35, body_h, view, (body, "#E5E7EB"))
            # Cabin narrower and shorter than the body, so the bonnet + boot read
            # as solid extensions of the same shape (no gap between them).
            cab_n = n + ln * 0.22
            cab_f = fr - ln * 0.20
            self._box3d(qp, cab_n, cab_f, hw * 0.86, lateral, body_h, body_h + 0.55,
                        view, (glass, "#1B3A44"))

        # Lights: small lamps at the front (warm white) and back (red). These are
        # what make the silhouette read as a real vehicle facing a direction.
        self._draw_lights(qp, view, n, fr, hw, lateral, body_h)

        # Four separate dark wheels instead of a single full-width floor block.
        for axle in (n + ln * 0.22, fr - ln * 0.22):
            for side in (-1, 1):
                self._box3d(qp, axle - 0.18, axle + 0.18, 0.13,
                            lateral + side * (hw + 0.08), 0.0, 0.62, view,
                            ("#0D0F12", "#30343A"))

    def _draw_lights(self, qp, view, n, fr, hw, lateral, body_h):
        """Head/tail lamps as small bright quads projected onto the body's ends."""
        head_y = body_h * 0.55   # roughly bumper-height
        tail_y = body_h * 0.55
        # Front (warm) — two lamps, left & right of centre, at the front face.
        for off in (-hw * 0.6, hw * 0.6):
            a = self._project(fr + 0.02, lateral + off, view, head_y)
            b = self._project(fr + 0.02, lateral + off + 0.35, view, head_y)
            c = self._project(fr + 0.02, lateral + off + 0.35, view, head_y + 0.25)
            d = self._project(fr + 0.02, lateral + off, view, head_y + 0.25)
            if None not in (a, b, c, d):
                qp.setPen(Qt.PenStyle.NoPen)
                qp.setBrush(QColor("#FFF7CC"))
                qp.drawPolygon(QPolygonF([a, b, c, d]))
        # Rear (red) — two lamps at the back face.
        for off in (-hw * 0.6, hw * 0.6):
            a = self._project(n - 0.02, lateral + off, view, tail_y)
            b = self._project(n - 0.02, lateral + off + 0.35, view, tail_y)
            c = self._project(n - 0.02, lateral + off + 0.35, view, tail_y + 0.25)
            d = self._project(n - 0.02, lateral + off, view, tail_y + 0.25)
            if None not in (a, b, c, d):
                qp.setPen(Qt.PenStyle.NoPen)
                qp.setBrush(QColor("#EF4444"))
                qp.drawPolygon(QPolygonF([a, b, c, d]))

    # --- Traffic light --------------------------------------------------------
    def _draw_light(self, qp, view, light):
        """Traffic-light with a glowing active bulb, housing + countdown.

        The lit bulb gets a radial-gradient halo so it actually looks lit (the
        flat-fill version read as three identical grey dots). Anchor: top-right
        of the scene, like a real signal visible through the windshield."""
        color = light.get("color", "off")
        cx, cy = view.right() - 64, view.top() + 12
        # Outer frame + inner housing (two rounded rects = bevelled look).
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(8, 10, 14, 240))
        qp.drawRoundedRect(QRectF(cx, cy, 26, 66), 7, 7)
        qp.setBrush(QColor(22, 26, 32, 255))
        qp.drawRoundedRect(QRectF(cx + 2, cy + 2, 22, 62), 5, 5)

        on_col = {"red": "#EF4444", "yellow": "#FBBF24", "green": "#22C55E"}.get(color)
        for i, cname in enumerate(("red", "yellow", "green")):
            by = cy + 4 + i * 19
            rect = QRectF(cx + 4, by, 18, 18)
            lit = (color == cname)
            if lit and on_col:
                # Glow halo: radial gradient fading from the lit colour out.
                grad = QRadialGradient(rect.center(), 22)
                c = QColor(on_col)
                grad.setColorAt(0.0, QColor(c.red(), c.green(), c.blue(), 235))
                grad.setColorAt(0.4, QColor(c.red(), c.green(), c.blue(), 120))
                grad.setColorAt(1.0, QColor(c.red(), c.green(), c.blue(), 0))
                qp.setBrush(QBrush(grad))
                qp.drawEllipse(QRectF(cx - 10, by - 10, 38, 38))
                # Bright core.
                qp.setBrush(QColor(255, 255, 255, 230))
                qp.drawEllipse(rect.adjusted(5, 5, -5, -5))
                qp.setBrush(on_col)
                qp.drawEllipse(rect)
            else:
                off = QColor({"red": "#3A1414", "yellow": "#3A2E08",
                              "green": "#0E3A1A"}.get(cname, "#2C3138"))
                qp.setBrush(off)
                qp.drawEllipse(rect)

        # Countdown + next-state hint beside the housing.
        tl = light.get("time_left", 0) or 0
        if tl > 0:
            qp.setPen(QColor("#FFFFFF")); qp.setFont(QFont("Segoe UI", 16, QFont.Weight.Bold))
            qp.drawText(QRectF(cx + 30, cy + 2, 64, 24), Qt.AlignmentFlag.AlignLeft,
                        f"{tl:.1f}s")
            nxt_col = {"red": "#F87171", "green": "#4ADE80",
                       "yellow": "#FBBF24"}.get(color, "#D1D5DB")
            nxt_txt = {"red": "→ ZELENÁ", "green": "→ ŽLTÁ",
                       "yellow": "→ ČERVENÁ"}.get(color, "")
            qp.setPen(QColor(nxt_col)); qp.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
            qp.drawText(QRectF(cx + 30, cy + 26, 86, 14), Qt.AlignmentFlag.AlignLeft, nxt_txt)

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
        """A full, solid vehicle model for the rear-cam inset.

        Same single-piece look as the main scene (floor + body + tinted cabin +
        lamps), built from the rear-cam projection so it reads at small size
        without looking like a hollow box."""
        t = v.get("type", "car")
        body_h = {"car": 1.2, "van": 1.8, "bus": 2.8, "truck": 2.6}.get(t, 1.3)
        hw = max(0.9, v.get("width", 2.0) / 2)
        ln = max(3.5, v.get("length", 4.5))
        n, fr = behind - ln / 2, behind + ln / 2

        def box(z0, z1, n_off=0.0, fr_off=0.0, hwf=1.0, faces=("#C8CCD2", "#E5E7EB")):
            """Draw one solid cuboid; all 5 faces so it's never see-through."""
            nn, ff = n + n_off, fr + fr_off
            hh = hw * hwf
            c = [proj(nn, lateral - hh, z0), proj(nn, lateral + hh, z0),
                 proj(ff, lateral - hh, z0), proj(ff, lateral + hh, z0),
                 proj(nn, lateral - hh, z1), proj(nn, lateral + hh, z1),
                 proj(ff, lateral - hh, z1), proj(ff, lateral + hh, z1)]
            if any(p is None for p in c):
                return False
            bl, br, fl, fr_, blt, brt, flt, frt = c
            side, top = faces
            qp.setPen(QPen(QColor("#34393F"), 1))
            qp.setBrush(QColor(side).darker(115)); qp.drawPolygon(QPolygonF([bl, br, brt, blt]))
            qp.setBrush(QColor(side)); qp.drawPolygon(QPolygonF([bl, fl, flt, blt]))
            qp.drawPolygon(QPolygonF([br, fr_, frt, brt]))
            qp.setBrush(QColor(side).darker(108)); qp.drawPolygon(QPolygonF([fl, fr_, frt, flt]))
            qp.setBrush(QColor(top)); qp.drawPolygon(QPolygonF([blt, brt, frt, flt]))
            return True

        # Floor slab + solid body + tinted cabin, so the silhouette is one shape.
        if not box(0.0, 0.35, n_off=-0.2, fr_off=0.2, hwf=1.07,
                   faces=("#9AA0A8", "#9AA0A8")):
            return
        box(0.35, body_h, faces=("#C8CCD2", "#E5E7EB"))
        if t == "truck":
            box(body_h, body_h + 1.0, n_off=0.0, fr_off=-ln * 0.35,
                hwf=0.96, faces=("#9AA0A8", "#9AA0A8"))
            box(body_h, body_h + 0.8, n_off=0.0, fr_off=-ln * 0.35,
                hwf=0.9, faces=("#0F2A33", "#1B3A44"))
        else:
            box(body_h, body_h + 0.55, n_off=ln * 0.22, fr_off=-ln * 0.20,
                hwf=0.86, faces=("#0F2A33", "#1B3A44"))

        # Lamps: we see the FRONT of these cars (they're behind us, facing our
        # direction), so warm headlamps are the bright cue; tail lamps are hidden.
        for off in (-hw * 0.6, hw * 0.6):
            a = proj(fr + 0.02, lateral + off, body_h * 0.55)
            b = proj(fr + 0.02, lateral + off + 0.35, body_h * 0.55)
            c = proj(fr + 0.02, lateral + off + 0.35, body_h * 0.55 + 0.25)
            d = proj(fr + 0.02, lateral + off, body_h * 0.55 + 0.25)
            if None not in (a, b, c, d):
                qp.setPen(Qt.PenStyle.NoPen); qp.setBrush(QColor("#FFF7CC"))
                qp.drawPolygon(QPolygonF([a, b, c, d]))

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
    # NOTE: do NOT call hud.show() here. The HUD starts hidden and the _tick()
    # poll waits for the ``ui_ready`` shared-state flag (set by the main UI's
    # showEvent) before showing itself. This guarantees the order
    # splash → main app → HUD instead of the HUD flashing on first.
    sys.exit(app.exec())
