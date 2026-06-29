"""
Real 3D driving-scene renderer (QOpenGLWidget + PyOpenGL).

Draws the road ahead, surrounding vehicles (as solid 3D models with windows +
lamps) and the traffic light with actual depth — the GPU gives us proper
perspective and shading that QPainter 2D simply cannot. Used both by the
transparent HUD overlay and the Visualization page card.

Falls back to nothing (no crash) if OpenGL isn't available.
"""

import math

try:
    from OpenGL.GL import *
    import numpy as np
    _HAS_GL = True
except Exception:
    _HAS_GL = False

from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from PyQt6.QtGui import QSurfaceFormat
from PyQt6.QtCore import Qt, QTimer


def _box_verts(w, l, h, cx=0.0, cy=0.0, cz=0.0):
    """24 vertices (4 per face) of an axis-aligned box, centred at (cx,cy,cz)."""
    x, y, z = w / 2, l / 2, h / 2
    bx, by, bz = cx, cy, cz + z   # top
    tx, ty, tz = cx, cy, cz - z   # bottom
    return np.array([
        # bottom
        tx-x, ty-y, tz,  tx+x, ty-y, tz,  tx+x, ty+y, tz,  tx-x, ty+y, tz,
        # top
        bx-x, by-y, bz,  bx+x, by-y, bz,  bx+x, by+y, bz,  bx-x, by+y, bz,
        # front (+y)
        tx-x, ty+y, tz,  tx+x, ty+y, tz,  bx+x, by+y, bz,  bx-x, by+y, bz,
        # back (-y)
        tx+x, ty-y, tz,  tx-x, ty-y, tz,  bx-x, by-y, bz,  bx+x, by-y, bz,
        # left (-x)
        tx-x, ty-y, tz,  tx-x, ty+y, tz,  bx-x, by+y, bz,  bx-x, by-y, bz,
        # right (+x)
        tx+x, ty+y, tz,  tx+x, ty-y, tz,  bx+x, by-y, bz,  bx+x, by+y, bz,
    ], dtype=np.float32)


def _draw_box(w, l, h, r, g, b, cx=0.0, cy=0.0, cz=0.0, shade=1.0):
    """Draw a shaded box: each face a quad, faces darker/lighter for depth."""
    v = _box_verts(w, l, h, cx, cy, cz)
    cols = [0.55, 0.45, 0.85, 0.7, 0.6, 0.8]   # per-face brightness multipliers
    glBegin(GL_QUADS)
    for fi in range(6):
        s = shade * cols[fi]
        glColor3f(r * s, g * s, b * s)
        base = fi * 12
        for vi in range(4):
            glVertex3f(v[base + vi*3], v[base + vi*3 + 1], v[base + vi*3 + 2])
    glEnd()


class DrivingScene(QOpenGLWidget):
    """GPU 3D driving scene: road, vehicles, traffic light.

    Reads position/heading/traffic/route from a shared-state-like object
    (``state``) and renders a chase-cam perspective every frame."""

    def __init__(self, state, parent=None):
        if not _HAS_GL:
            # Without OpenGL we can't render; parent should check has_gl.
            super().__init__(parent)
            self.has_gl = False
            return
        fmt = QSurfaceFormat()
        fmt.setDepthBufferSize(24)
        fmt.setSamples(4)   # 4x MSAA for smooth edges
        QSurfaceFormat.setDefaultFormat(fmt)
        super().__init__(parent)
        self.state = state
        self.has_gl = True
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(80)

    def initializeGL(self):
        glClearColor(0.04, 0.05, 0.07, 0.0)   # dark, transparent-clear
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_MULTISAMPLE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glShadeModel(GL_SMOOTH)
        # Distance fog: distant geometry fades into the haze, giving real depth
        # (a flat-clear horizon makes the road look like it's floating). Linear
        # fog between 60 m and 320 m — close geometry stays crisp, the far road
        # melts into the sky. Fog colour matches the sky gradient's horizon.
        glEnable(GL_FOG)
        glFogi(GL_FOG_MODE, GL_LINEAR)
        glFogf(GL_FOG_START, 60.0)
        glFogf(GL_FOG_END, 320.0)
        glFogf(GL_FOG_DENSITY, 0.6)
        fog_col = (0.42, 0.50, 0.58, 1.0)
        glFogfv(GL_FOG_COLOR, (GLfloat * 4)(*fog_col))
        glHint(GL_FOG_HINT, GL_DONTCARE)

    def resizeGL(self, w, h):
        if h <= 0:
            h = 1
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60.0, w / h, 0.5, 500.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        self._frame = getattr(self, "_frame", 0) + 1
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        # Chase camera: 14m behind, 7m above, looking forward.
        gluLookAt(0, -14, 7,  0, 30, 2,  0, 0, 1)
        self._draw_sky()           # gradient sky dome (drawn first, behind all)
        self._draw_ground()
        self._draw_road()
        self._draw_nav_line()       # blue route line painted on the road
        self._draw_road_signs()     # gantry/roadside signs by road type
        self._draw_signage()        # overhead board with the destination city
        self._draw_vehicles()
        self._draw_emergency()      # police/fire with flashing roof lights
        self._draw_traffic_light()  # 3-bulb signal ahead
        self._draw_cones()          # construction cones / road closures
        self._draw_construction()   # excavators / barriers at worksites
        self._draw_pedestrians()    # walking figures on the sidewalks
        self._draw_ego()
        # 2D overlays (ACC box, rear-cam inset) drawn after the 3D scene.
        self._draw_overlays()

    # --- Scene elements ------------------------------------------------------
    def _draw_sky(self):
        """Sky dome: a vertical gradient from a pale horizon up to a deeper
        zenith, drawn as big quads behind everything. Gives the road a sky to
        sit under instead of a flat dark void — a major cheap visual upgrade.

        Depth test + fog are briefly disabled so the sky always sits behind the
        scene and isn't greyed into the haze."""
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_FOG)
        glBegin(GL_QUADS)
        # Zenith band (top) — deeper blue-grey.
        glColor3f(0.30, 0.40, 0.52)
        glVertex3f(-400, -120, 180); glVertex3f(400, -120, 180)
        glVertex3f(400, 420, 180);   glVertex3f(-400, 420, 180)
        # Horizon band (lower) — lighter, softens the transition.
        glColor3f(0.55, 0.62, 0.68)
        glVertex3f(-400, -120, 60); glVertex3f(400, -120, 60)
        glVertex3f(400, 420, 60);   glVertex3f(-400, 420, 60)
        glEnd()
        glEnable(GL_FOG)
        glEnable(GL_DEPTH_TEST)

    def _draw_ground(self):
        # A flat dark-green ground plane gives the road something to sit on.
        glBegin(GL_QUADS)
        glColor3f(0.07, 0.10, 0.08)
        glVertex3f(-60, -20, 0); glVertex3f(60, -20, 0)
        glVertex3f(60, 120, 0);  glVertex3f(-60, 120, 0)
        glEnd()

    def _draw_road(self):
        d = self._read()
        path = d.get("path", [])
        # Fallback: a straight two-lane road if no route yet.
        if len(path) < 2:
            self._draw_road_segment([(-3.5, 0), (-3.5, 90),
                                     (3.5, 90), (3.5, 0)])
            self._draw_dashed_centre(0)
            return
        # Convert route (world x,z) to truck-relative (ahead, lateral) points.
        pos, h = d.get("pos"), d.get("heading", 0.0)
        al = []
        if pos:
            for wx, wz in path:
                dx, dz = wx - pos[0], wz - pos[1]
                ahead = dx * (-math.sin(h)) + dz * (-math.cos(h))
                lat = dx * math.cos(h) - dz * math.sin(h)
                al.append((ahead, lat))
        else:
            al = [(i * 6.0, 0.0) for i in range(15)]
        # Build the asphalt ribbon as a triangle strip following the path.
        HALF = 3.5
        self._draw_road_ribbon(al, HALF)
        # Centre dashed line.
        self._draw_path_polyline(al, 0.0, dashed=True)

    def _draw_road_ribbon(self, al, half):
        # Triangles for the asphalt; build left/right edges from the path.
        left = [(a, l - half) for a, l in al if 0 < a < 90]
        right = [(a, l + half) for a, l in al if 0 < a < 90]
        if len(left) < 2 or len(right) < 2:
            return
        glBegin(GL_TRIANGLE_STRIP)
        glColor3f(0.13, 0.14, 0.16)
        for (la, ll), (ra, rl) in zip(left, right):
            glVertex3f(ll, la, 0.02); glVertex3f(rl, ra, 0.02)
        glEnd()
        # Solid white edge lines.
        for edge in (left, right):
            glBegin(GL_LINE_STRIP)
            glColor3f(0.85, 0.86, 0.88)
            glLineWidth(2.0)
            for a, l in edge:
                glVertex3f(l, a, 0.04)
            glEnd()

    def _draw_path_polyline(self, al, lateral_offset, dashed=False):
        pts = [(a, l + lateral_offset) for a, l in al if 0 < a < 90]
        if len(pts) < 2:
            return
        if dashed:
            glBegin(GL_LINES)
        else:
            glBegin(GL_LINE_STRIP)
        glColor3f(1.0, 1.0, 0.95)
        glLineWidth(1.8)
        prev = None
        for i, (a, l) in enumerate(pts):
            if dashed:
                if i % 2 == 0 and prev is not None:
                    glVertex3f(prev[1], prev[0], 0.05)
                    glVertex3f(l, a, 0.05)
                prev = (a, l)
            else:
                glVertex3f(l, a, 0.05)
        glEnd()

    def _draw_road_segment(self, corners):
        glBegin(GL_QUADS)
        glColor3f(0.13, 0.14, 0.16)
        for x, y in corners:
            glVertex3f(x, y, 0.02)
        glEnd()

    def _draw_dashed_centre(self, lateral):
        glBegin(GL_LINES)
        glColor3f(1.0, 1.0, 0.9)
        glLineWidth(1.8)
        for i in range(0, 85, 8):
            glVertex3f(lateral, i + 2, 0.05)
            glVertex3f(lateral, i + 5, 0.05)
        glEnd()

    def _draw_vehicles(self):
        d = self._read()
        pos, h = d.get("pos"), d.get("heading", 0.0)
        traffic = d.get("traffic", [])
        if not pos:
            return
        vehs = []
        for v in traffic:
            dx, dz = v["x"] - pos[0], v["z"] - pos[1]
            ahead = dx * (-math.sin(h)) + dz * (-math.cos(h))
            lat = dx * math.cos(h) - dz * math.sin(h)
            if 2 < ahead < 80 and abs(lat) < 8:
                vehs.append((ahead, lat, v))
        vehs.sort(key=lambda t: -t[0])   # far → near
        for ahead, lat, v in vehs:
            self._draw_vehicle(ahead, lat, v)

    def _draw_vehicle(self, ahead, lat, v):
        t = v.get("type", "car")
        ln = max(3.5, v.get("length", 4.5))
        wd = max(1.6, v.get("width", 2.0))
        body_h = {"car": 1.2, "van": 1.9, "bus": 2.8, "truck": 2.6}.get(t, 1.3)
        # Body (silver-grey).
        self._draw_box(wd, ln, body_h, 0.78, 0.80, 0.84, cy=ahead, cx=lat, cz=body_h / 2)
        # Cabin / greenhouse (dark tinted glass), smaller and on top.
        cab_h = 0.6 if t != "truck" else 0.9
        cab_l = ln * 0.45
        self._draw_box(wd * 0.82, cab_l, cab_h, 0.12, 0.20, 0.26,
                       cx=lat, cy=ahead + ln * 0.05, cz=body_h + cab_h / 2)
        # Head lamps (warm) at the front face, tail lamps (red) at the back.
        self._draw_lamps(ahead, lat, wd, ln, body_h, t)
        # Turn signals (orange), flashing on the signalled side. The frame
        # counter toggles them ~3 Hz like real indicators.
        blink = v.get("blink")
        if blink in ("left", "right"):
            self._draw_blinker(ahead, lat, wd, ln, body_h, blink)

    def _draw_blinker(self, ahead, lat, wd, ln, body_h, side):
        on = (getattr(self, "_frame", 0) // 4) % 2 == 0
        if not on:
            return
        ox = -wd * 0.45 if side == "left" else wd * 0.45
        for oy in (ln * 0.45, -ln * 0.45):
            self._draw_box(0.22, 0.18, 0.14, 1.0, 0.6, 0.05,
                           cx=lat + ox, cy=ahead + oy, cz=body_h * 0.55, shade=1.0)

    def _draw_lamps(self, ahead, lat, wd, ln, body_h, t):
        ly = body_h * 0.45
        # Head (front = +y of the vehicle, which is the far end).
        for ox in (-wd * 0.32, wd * 0.32):
            self._draw_box(0.35, 0.15, 0.22, 1.0, 0.95, 0.7,
                           cx=lat + ox, cy=ahead + ln / 2 - 0.05, cz=ly)
        # Tail (red, back).
        for ox in (-wd * 0.32, wd * 0.32):
            self._draw_box(0.35, 0.15, 0.22, 0.95, 0.1, 0.1,
                           cx=lat + ox, cy=ahead - ln / 2 + 0.05, cz=ly)

    def _draw_ego(self):
        """Draw the player's tractor, and — if a trailer is coupled — the
        semi-trailer hinged behind it.

        The tractor sits at the bottom centre (``ahead=8``). When the telemetry
        reports an attached trailer we also draw the trailer, rotated about the
        fifth-wheel pivot by the live **articulation angle** (tractor vs trailer
        heading). That angle grows in tight bends, so the trailer visibly cuts
        the corner like a real semi — the whole point of "natáčanie návesu".
        When no trailer is coupled we only draw the cab (the old behaviour)."""
        blink = (self.state or {}).get("active_blinker", "off")
        self._draw_tractor(8.0, 0.0, blink)

        d = self._read()
        if not d.get("trailer_attached"):
            return
        # Articulation angle (rad) between tractor and trailer headings.
        # +angle = trailer tail swung LEFT of the tractor; in our scene the
        # camera looks down +y with +x to the right, so a positive articulation
        # rotates the trailer counter-clockwise (its tail moves to -x = left).
        art = float(d.get("trailer_articulation", 0.0) or 0.0)
        self._draw_semi_trailer(8.0, 0.0, art, blink)

    def _draw_tractor(self, ahead, lat, blink):
        """The cab/tractor: a short, tall truck body + cab + lamps + blinkers.

        Drawn shorter than the old 14 m box because that 14 m was really a
        whole tractor+trailer combo; now the trailer is modelled separately."""
        ego = {"type": "truck", "width": 2.5, "length": 6.5}
        if blink in ("left", "right"):
            ego["blink"] = blink
        self._draw_vehicle(ahead, lat, ego)

    def _draw_semi_trailer(self, tractor_ahead, tractor_lat, articulation, blink):
        """A box trailer hinged at the tractor's rear (the fifth wheel).

        The pivot is at the back face of the tractor; the trailer extends
        backwards from there and is rotated by ``articulation`` (radians) about
        that pivot, so in a curve the trailer's tail swings inboard — the
        visible "nacýľaný náves" effect. We use glRotate so the whole trailer
        body, lamps and blinkers rotate together as a rigid unit."""
        # Pivot = back of the tractor (tractor length 6.5 → rear at -3.25 from
        # its centre, plus a small gap for the hitch).
        pivot_ahead = tractor_ahead - 3.25 - 0.4
        pivot_lat = tractor_lat

        glPushMatrix()
        # Move to the pivot, rotate about Z (the vertical axis), then draw the
        # trailer extending further back from the pivot.
        glTranslatef(pivot_lat, pivot_ahead, 0.0)
        # Positive articulation = tail to the left = rotate CCW = -Z in GL
        # (right-handed). glRotate takes degrees; sign chosen so a right-hand
        # bend (art>0) visibly tucks the trailer's tail to the LEFT.
        glRotatef(math.degrees(-articulation), 0.0, 0.0, 1.0)
        glTranslatef(-pivot_lat, -pivot_ahead, 0.0)

        # Trailer body: long, tall box. Centre sits half its length behind the
        # pivot so it trails away from the tractor.
        tr_len = 11.0
        tr_w = 2.5
        tr_h = 3.0
        tr_cy = pivot_ahead - tr_len / 2
        # Body (white/grey box body).
        self._draw_box(tr_w, tr_len, tr_h, 0.9, 0.9, 0.92,
                       cx=pivot_lat, cy=tr_cy, cz=tr_h / 2)
        # Rear lamps (red) across the back face.
        for ox in (-tr_w * 0.35, tr_w * 0.35):
            self._draw_box(0.35, 0.15, 0.45, 0.95, 0.1, 0.1,
                           cx=pivot_lat + ox, cy=tr_cy - tr_len / 2 + 0.05,
                           cz=tr_h * 0.4, shade=1.0)
        # Trailer blinkers mirror the tractor's, flashing on the signalled side.
        if blink in ("left", "right"):
            on = (getattr(self, "_frame", 0) // 4) % 2 == 0
            if on:
                ox = -tr_w * 0.45 if blink == "left" else tr_w * 0.45
                self._draw_box(0.22, 0.18, 0.18, 1.0, 0.6, 0.05,
                               cx=pivot_lat + ox,
                               cy=tr_cy - tr_len / 2 + 0.1,
                               cz=tr_h * 0.5, shade=1.0)
        glPopMatrix()

    # --- Traffic light (3D pole + glowing bulb) ------------------------------
    def _draw_traffic_light(self):
        d = self._read()
        light = d.get("light")
        if not light:
            return
        dist = light.get("distance", 999.0)
        if dist > 90.0 or dist < 2.0:
            return
        x = 5.5
        y = min(dist, 85.0)
        color = light.get("color", "off")
        self._draw_box(0.18, 0.18, 5.0, 0.18, 0.19, 0.21, cx=x, cy=y, cz=2.5)
        self._draw_box(0.5, 0.4, 1.2, 0.08, 0.09, 0.10, cx=x, cy=y, cz=4.6)
        bulb_colors = (("red", 0.95, 0.1, 0.1), ("yellow", 0.95, 0.75, 0.1),
                       ("green", 0.1, 0.85, 0.2))
        for i, (cname, r, g, b) in enumerate(bulb_colors):
            lit = (color == cname)
            br = 1.0 if lit else 0.18
            cz = 5.15 - i * 0.38
            self._draw_box(0.3, 0.3, 0.3, r * br, g * br, b * br,
                           cx=x, cy=y - 0.15, cz=cz, shade=1.0)
            # Glow halo on the active bulb: a larger, semi-transparent face that
            # bleeds the colour outward, like a real lit signal at dusk.
            if lit:
                self._draw_glow(x, y - 0.15, cz, r, g, b)

    def _draw_glow(self, cx, cy, cz, r, g, b, radius=1.6, alpha=0.32):
        """Soft additive halo around a light source (traffic bulb, lamp).

        A few concentric fading quads built up with additive blending give the
        impression of bloom without a real post-processing pass. Depth write is
        left on (it's behind the bulb itself) so distant geometry still sorts."""
        # Additive blending: stacked layers brighten into a glow, never darken.
        glBlendFunc(GL_SRC_ALPHA, GL_ONE)
        layers = 3
        for k in range(layers, 0, -1):
            rad = radius * k / layers
            a = alpha * (1.0 - (k - 1) / layers)
            glColor4f(r, g, b, a)
            glBegin(GL_QUADS)
            glVertex3f(cx - rad, cy - rad, cz)
            glVertex3f(cx + rad, cy - rad, cz)
            glVertex3f(cx + rad, cy + rad, cz)
            glVertex3f(cx - rad, cy + rad, cz)
            glEnd()
        # Restore normal alpha blending for the rest of the scene.
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    # --- Roadside / gantry signs by road type --------------------------------
    def _draw_road_signs(self):
        """Draw a gantry or roadside information board depending on road type.

        Motorways get a tall green gantry overhead (like ETS2's route boards);
        local/city roads get a small roadside speed-limit-ish post. This is
        visual dressing driven by the classified road type — no game data
        needed beyond what the map plugin already publishes."""
        d = self._read()
        rtype = d.get("road_type", "local")
        # Place one sign ~40 m ahead, refreshed each frame.
        y = 40.0
        if rtype in ("motorway", "expressway"):
            # Green gantry: two posts + a wide board across the road overhead.
            for sx in (-6.0, 6.0):
                self._draw_box(0.25, 0.25, 6.0, 0.5, 0.5, 0.52, cx=sx, cy=y, cz=3.0)
            self._draw_box(13.0, 0.4, 1.8, 0.10, 0.45, 0.18, cy=y, cz=6.2)
            # Three white "destination" slits on the board.
            for i in range(3):
                self._draw_box(8.0, 0.05, 0.35, 0.9, 0.92, 0.95,
                               cy=y - 0.05, cz=5.7 + i * 0.45)
        else:
            # Roadside post with a small board (speed-limit / town sign feel).
            sx = 6.5
            self._draw_box(0.12, 0.12, 3.0, 0.5, 0.5, 0.52, cx=sx, cy=y, cz=1.5)
            r, g, b = (0.95, 0.95, 0.95) if rtype == "local" else (0.95, 0.8, 0.1)
            self._draw_box(0.7, 0.12, 0.7, r, g, b, cx=sx, cy=y, cz=3.1)

    # --- Destination sign on the gantry (real job target) --------------------
    def _draw_signage(self):
        """Draw the destination text on a green overhead gantry board.

        Complements :meth:`_draw_road_signs` (which draws the bare gantry
        shape) by printing the **real job destination** (``dest_city`` from
        telemetry's Zone 9 ``cityDst``) plus the remaining distance to it.
        ETS2's own signage data (europe-signs.json) isn't shipped, so this is
        the closest thing to a "real" destination board: the city the current
        job is going to. Only shown when there's an active job AND we're on a
        motorway/expressway (gantries don't appear over local roads); a local-
        road variant shows the destination on a roadside board instead."""
        d = self._read()
        dest = (d.get("dest_city") or "").strip()
        if not dest:
            return   # no active job → nothing meaningful to print
        rtype = d.get("road_type", "local")
        dist = d.get("distance_to_dest")
        # Format the distance (km when far, m when close) for the sign line.
        if dist is None:
            dist_txt = ""
        else:
            try:
                dk = float(dist) / 1000.0
                dist_txt = ("%.0f km" % dk) if dk >= 1.0 else ("%d m" % int(dist))
            except (TypeError, ValueError):
                dist_txt = ""

        if rtype in ("motorway", "expressway"):
            # Overhead gantry board (matches _draw_road_signs' motorway gantry
            # at y=40). A green panel + white text lines.
            y = 40.0
            # The text board sits just in front of the bare gantry board.
            self._draw_box(12.0, 0.3, 1.5, 0.07, 0.33, 0.13, cy=y - 0.3, cz=6.2)
            # Project the board centre to screen space to place the text.
            sx, sy = self._world_to_screen(0.0, y - 0.3, 6.2)
            if sx is not None:
                self._text_label(sx - 70, sy - 6, dest.upper(), (1, 1, 1))
                if dist_txt:
                    self._text_label(sx - 18, sy + 16, dist_txt, (0.95, 0.95, 0.4))
        else:
            # Local road: roadside destination board (right side of the road).
            y = 30.0
            sx_world = 6.5
            self._draw_box(0.12, 0.12, 3.5, 0.5, 0.5, 0.52,
                           cx=sx_world, cy=y, cz=1.75)
            self._draw_box(2.0, 0.12, 1.0, 0.07, 0.33, 0.13,
                           cx=sx_world, cy=y, cz=3.6)
            sx, sy = self._world_to_screen(sx_world, y, 3.6)
            if sx is not None:
                self._text_label(sx - 30, sy - 4, dest, (1, 1, 1))

    def _world_to_screen(self, x, y, z):
        """Project a world point to widget pixel coords for text placement.

        Uses the current GL projection/modelview (chase cam). Returns
        ``(None, None)`` if the point is behind the camera or unprojections
        fail — callers should skip drawing text in that case. Lightweight read
        of the matrices via glGetDoublev; no external deps."""
        try:
            from OpenGL.GL import glGetDoublev, GL_PROJECTION_MATRIX, GL_MODELVIEW_MATRIX
            from OpenGL.GLU import gluProject
            import numpy as np
            proj = glGetDoublev(GL_PROJECTION_MATRIX)
            model = glGetDoublev(GL_MODELVIEW_MATRIX)
            view = glGetIntegerv(GL_VIEWPORT)
            ok, winx, winy, winz = gluProject(x, y, z, model, proj, view)
            if not ok:
                return None, None
            # Qt widget origin is top-left; GL viewport origin is bottom-left.
            return float(winx), self.height() - float(winy)
        except Exception:
            return None, None

    # --- Construction cones / road-closure markers ---------------------------
    def _draw_cones(self):
        """Draw orange cones where a road hazard is reported.

        Reads ``road_hazard`` (a dict with ``distance`` and ``lane_offset``)
        published by the lane-control plugin when a lane is blocked. If none,
        nothing is drawn. A double row of cones marks the closed lane."""
        d = self._read()
        hz = d.get("road_hazard")
        if not hz:
            return
        dist = float(hz.get("distance", 0.0) or 0.0)
        lat = float(hz.get("lane_offset", 3.5) or 3.5)
        if dist <= 5 or dist > 80:
            return
        # A row of small cones along the hazard, tapering the closure.
        for i in range(8):
            cy = dist - 12 + i * 3.0
            if cy < 2:
                continue
            taper = max(0.0, 1.0 - i / 8.0)
            cx = lat + (3.0 * taper if lat > 0 else -3.0 * taper)
            self._draw_box(0.25, 0.25, 0.6, 0.95, 0.45, 0.05,
                           cx=cx, cy=cy, cz=0.3, shade=1.0)
            # White reflective ring on the cone.
            self._draw_box(0.27, 0.27, 0.08, 0.95, 0.95, 0.95,
                           cx=cx, cy=cy, cz=0.4, shade=1.0)

    # --- Emergency vehicles with roof light bar ------------------------------
    def _draw_emergency(self):
        """Police / fire / ambulance: a vehicle with a flashing red+blue bar.

        ETS2LA's traffic buffer doesn't tag emergency types, so we treat any
        vehicle whose id is in a reserved range as emergency (lets the user see
        the effect). Real emergency detection would need a game-side mod."""
        d = self._read()
        pos, h = d.get("pos"), d.get("heading", 0.0)
        traffic = d.get("traffic", [])
        if not pos:
            return
        px, pz = pos
        sin_h, cos_h = math.sin(h), math.cos(h)
        frame = getattr(self, "_frame", 0)
        for v in traffic:
            # Reserved id range = emergency (visual demo).
            try:
                vid = int(v.get("id", 0) or 0)
            except (TypeError, ValueError):
                continue
            if vid % 100 != 7:    # ~1% of vehicles → demo
                continue
            dx, dz = v["x"] - px, v["z"] - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lat = dx * cos_h - dz * sin_h
            if not (5 < ahead < 80 and abs(lat) < 8):
                continue
            ln = max(5.0, float(v.get("length", 5.0) or 5.0))
            wd = max(2.0, float(v.get("width", 2.0) or 2.0))
            body_h = 1.8
            # Body (white).
            self._draw_box(wd, ln, body_h, 0.92, 0.92, 0.93,
                           cx=lat, cy=ahead, cz=body_h / 2)
            # Cabin (dark glass).
            self._draw_box(wd * 0.85, ln * 0.4, 0.7, 0.1, 0.15, 0.2,
                           cx=lat, cy=ahead + ln * 0.1, cz=body_h + 0.35)
            # Roof light bar: two halves, flashing red/blue at ~2 Hz.
            on = (frame // 8) % 2 == 0
            r = 1.0 if on else 0.1
            b = 0.1 if on else 1.0
            self._draw_box(wd * 0.25, 0.25, 0.12, r, 0.05, 0.05,
                           cx=lat - wd * 0.2, cy=ahead, cz=body_h + 0.12, shade=1.0)
            self._draw_box(wd * 0.25, 0.25, 0.12, 0.05, 0.05, b,
                           cx=lat + wd * 0.2, cy=ahead, cz=body_h + 0.12, shade=1.0)

    # --- Construction vehicles / excavators at worksites ---------------------
    def _draw_construction(self):
        """Yellow heavy machinery parked beside a hazard (excavator demo).

        Drawn near any reported road hazard, as if a worksite is closing the
        lane. Real worksite data isn't exposed by the game, so we dress the
        scene around hazards the autopilot already detects."""
        d = self._read()
        hz = d.get("road_hazard")
        if not hz:
            return
        dist = float(hz.get("distance", 0.0) or 0.0)
        if dist <= 10 or dist > 80:
            return
        # An excavator-ish shape to the right of the hazard: yellow body +
        # dark cab + a long arm reaching over.
        x = 7.0
        y = dist - 4.0
        self._draw_box(2.6, 4.0, 1.6, 0.9, 0.72, 0.05, cx=x, cy=y, cz=0.8)
        self._draw_box(2.2, 1.6, 0.8, 0.12, 0.13, 0.14, cx=x, cy=y + 1.2, cz=2.0)
        # Arm (two segments) reaching toward the road.
        self._draw_box(0.3, 0.3, 3.0, 0.85, 0.68, 0.05, cx=x - 1.5, cy=y, cz=2.6)
        self._draw_box(0.25, 2.5, 0.25, 0.85, 0.68, 0.05,
                       cx=x - 2.6, cy=y, cz=2.0, shade=1.0)
        # Tracks (dark).
        for ox in (-1.1, 1.1):
            self._draw_box(0.5, 4.0, 0.5, 0.12, 0.12, 0.13,
                           cx=x + ox, cy=y, cz=0.25, shade=1.0)

    # --- Pedestrians on sidewalks (simple walking figures) -------------------
    def _draw_pedestrians(self):
        """A few walking figures on the sidewalks in city/local sectors.

        ETS2 pedestrians aren't in the traffic buffer, so these are ambient
        dressing synced to the road type (only on local/city roads). Each figure
        is a simple capsule + head; a frame-based sway animates the walk."""
        d = self._read()
        rtype = d.get("road_type", "local")
        if rtype not in ("local", "expressway"):
            return
        # Stable positions on both sidewalks, walking forward slowly.
        frame = getattr(self, "_frame", 0)
        sidewalks = (-7.5, 7.5)
        for side, sx in (("L", -7.5), ("R", 7.5)):
            for i in range(3):
                # Each pedestrian at a different distance, walking toward us.
                speed = 1.4   # m/s walking speed
                base = (i * 18.0 + (hash(side + str(i)) % 7) * 3.0)
                py = (base - frame * 0.066 * speed) % 60.0 + 8.0
                sway = math.sin((py + frame * 0.1) * 0.6) * 0.05
                self._draw_pedestrian(sx, py, sway)

    def _draw_pedestrian(self, x, y, sway):
        """One walking figure: legs (sway), torso, head."""
        # Legs (two thin boxes, swaying as if mid-stride).
        self._draw_box(0.18, 0.18 + sway, 0.9, 0.2, 0.22, 0.28,
                       cx=x - 0.12, cy=y, cz=0.45)
        self._draw_box(0.18, 0.18 - sway, 0.9, 0.2, 0.22, 0.28,
                       cx=x + 0.12, cy=y, cz=0.45)
        # Torso.
        self._draw_box(0.42, 0.28, 0.85, 0.55, 0.25, 0.25, cx=x, cy=y, cz=1.35)
        # Head.
        self._draw_box(0.26, 0.26, 0.26, 0.9, 0.72, 0.6,
                       cx=x, cy=y, cz=1.92, shade=1.0)

    # --- Navigation line painted on the road (blue) -------------------------
    def _draw_nav_line(self):
        d = self._read()
        path = d.get("path", [])
        pos, h = d.get("pos"), d.get("heading", 0.0)
        if not pos or len(path) < 2:
            return
        pts = []
        for wx, wz in path:
            dx, dz = wx - pos[0], wz - pos[1]
            ahead = dx * (-math.sin(h)) + dz * (-math.cos(h))
            lat = dx * math.cos(h) - dz * math.sin(h)
            if 3.0 < ahead < 90.0:
                pts.append((ahead, lat))
        if len(pts) < 2:
            return
        glLineWidth(5.0)
        glBegin(GL_LINE_STRIP)
        glColor4f(0.25, 0.45, 0.95, 0.85)
        for a, l in pts:
            glVertex3f(l, a, 0.08)
        glEnd()

    # --- 2D overlays: ACC box + rear-cam inset -------------------------------
    def _draw_overlays(self):
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.width(), self.height(), 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        glDisable(GL_DEPTH_TEST)
        self._draw_acc_box()
        self._draw_rear_cam_inset()
        glEnable(GL_DEPTH_TEST)
        glPopMatrix()
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)

    def _draw_acc_box(self):
        """Small card showing the ACC target vehicle distance."""
        d = self._read()
        lead = d.get("lead_distance")
        if not lead or lead <= 0 or lead > 120:
            return
        w = self.width()
        x, y, bw, bh = w - 170, 70, 150, 48
        glBegin(GL_QUADS)
        glColor4f(0.05, 0.07, 0.10, 0.82)
        glVertex2f(x, y); glVertex2f(x + bw, y)
        glVertex2f(x + bw, y + bh); glVertex2f(x, y + bh)
        glEnd()
        norm = max(0.0, min(1.0, float(lead) / 80.0))
        bar_w = bw - 20
        r = 0.95 - 0.7 * norm
        g = 0.2 + 0.6 * norm
        glBegin(GL_QUADS)
        glColor3f(r, g, 0.2)
        glVertex2f(x + 10, y + 10); glVertex2f(x + 10 + bar_w * norm, y + 10)
        glVertex2f(x + 10 + bar_w * norm, y + 18); glVertex2f(x + 10, y + 18)
        glEnd()
        self._text_label(x + 10, y + 22, "ACC  %.0f m" % float(lead), (1, 1, 1))

    def _draw_rear_cam_inset(self):
        """Mirror-style inset showing traffic behind us (bottom-right)."""
        d = self._read()
        blink = d.get("blinker", "off")
        show = blink in ("left", "right") or bool(d.get("rear_cam"))
        if not show:
            return
        w, h = self.width(), self.height()
        x, y, bw, bh = w - 175, h - 135, 160, 115
        glBegin(GL_QUADS)
        glColor4f(0.03, 0.05, 0.07, 0.9)
        glVertex2f(x, y); glVertex2f(x + bw, y)
        glVertex2f(x + bw, y + bh); glVertex2f(x, y + bh)
        glEnd()
        traffic = d.get("traffic", [])
        pos, hdg = d.get("pos"), d.get("heading", 0.0)
        if not pos:
            return
        px, pz = pos
        sin_h, cos_h = math.sin(hdg), math.cos(hdg)
        for v in traffic:
            dx, dz = v["x"] - px, v["z"] - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lat = dx * cos_h - dz * sin_h
            behind = -ahead
            if 2 < behind < 60 and abs(lat) < 12:
                nx = x + bw / 2 + (lat / 12.0) * (bw / 2 - 8)
                ny = y + bh - 12 - (behind / 60.0) * (bh - 20)
                self._dot(nx, ny, 4, 0.85, 0.87, 0.9)
        self._text_label(x + 8, y + 4, "ZADNÁ KAMERA", (0.2, 0.85, 0.4))

    # --- Low-level 2D helpers ------------------------------------------------
    def _dot(self, cx, cy, r, red, g, b):
        glBegin(GL_TRIANGLE_FAN)
        glColor3f(red, g, b)
        glVertex2f(cx, cy)
        for k in range(13):
            a = 2 * 3.14159 * k / 12
            glVertex2f(cx + r * math.cos(a), cy + r * math.sin(a))
        glEnd()

    def _text_label(self, x, y, text, rgb):
        """Render a small text label via QPainter onto the GL widget."""
        try:
            from PyQt6.QtGui import QPainter, QFont, QColor
            p = QPainter(self)
            p.setRenderHint(QPainter.RenderHint.Antialiasing)
            p.setPen(QColor(int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255)))
            p.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
            p.drawText(int(x), int(y + 12), text)
            p.end()
        except Exception:
            pass

    # --- Data ----------------------------------------------------------------
    def _read(self):
        s = self.state or {}
        return {
            "pos": s.get("truck_world_pos"),
            "heading": s.get("truck_heading", 0.0) or 0.0,
            "traffic": s.get("traffic", []) or [],
            "path": (s.get("nav_path", []) or s.get("map_path", []) or []),
            "light": s.get("traffic_light"),
            "blinker": (s.get("active_blinker") or "off"),
            "rear_cam": bool(s.get("rear_cam", False)),
            "lead_distance": s.get("lead_distance"),
            "speed_ms": s.get("truck_speed_ms", 0.0),
            "road_type": (s.get("road_type") or "local"),
            "road_hazard": s.get("road_hazard"),
            # Destination city of the current job (Zone 9 string) + remaining
            # path distance to it. Used by the overhead gantry sign text.
            "dest_city": (s.get("dest_city") or ""),
            "distance_to_dest": s.get("distance_to_dest"),
            # Articulated trailer (Zone 14). When attached we get the trailer's
            # world pose + a signed articulation angle (tractor vs trailer
            # heading). None / False → draw the cab only.
            "trailer_attached": bool(s.get("trailer_attached", False)),
            "trailer_pos": s.get("trailer_world_pos"),
            "trailer_heading": s.get("trailer_heading"),
            "trailer_articulation": float(s.get("trailer_articulation", 0.0) or 0.0),
        }


# Provide gluPerspective/gluLookAt even if GLU isn't bound (rare).
try:
    from OpenGL.GLU import gluPerspective, gluLookAt
except Exception:
    def gluPerspective(fovy, aspect, near, far):
        import numpy as np
        f = 1.0 / math.tan(math.radians(fovy) / 2)
        M = np.zeros((4, 4), dtype=np.float32)
        M[0, 0] = f / aspect
        M[1, 1] = f
        M[2, 2] = (far + near) / (near - far)
        M[2, 3] = (2 * far * near) / (near - far)
        M[3, 2] = -1
        glMatrixMode(GL_PROJECTION)
        glLoadMatrixf(M)
        glMatrixMode(GL_MODELVIEW)

    def gluLookAt(ex, ey, ez, cx, cy, cz, ux, uy, uz):
        import numpy as np
        def norm(v):
            n = math.sqrt(sum(i * i for i in v)); return [i / n for i in v]
        f = norm([cx - ex, cy - ey, cz - ez])
        s = norm([f[1] * uz - f[2] * uy, f[2] * ux - f[0] * uz, f[0] * uy - f[1] * ux])
        u = [s[1] * f[2] - s[2] * f[1], s[2] * f[0] - s[0] * f[2], s[0] * f[1] - s[1] * f[0]]
        M = np.identity(4, dtype=np.float32)
        M[0, 0:3] = s; M[1, 0:3] = u; M[2, 0:3] = [-i for i in f]
        M[0, 3] = -sum(s[i] * [ex, ey, ez][i] for i in range(3))
        M[1, 3] = -sum(u[i] * [ex, ey, ez][i] for i in range(3))
        M[2, 3] = sum(f[i] * [ex, ey, ez][i] for i in range(3))
        glMultMatrixf(M)
