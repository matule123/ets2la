"""
Real GPU-accelerated 3D driving view (pyqtgraph + OpenGL).

This renders the ego truck, surrounding vehicles and the route as actual 3D
meshes on the GPU — unlike the HUD's software (QPainter) perspective.  It needs
``pyqtgraph`` and ``PyOpenGL`` installed:

    pip install pyqtgraph PyOpenGL

If they're missing the widget degrades to a hint label (the app never crashes).
"""

import math

try:
    import numpy as np
    import pyqtgraph.opengl as gl
    _HAS_GL = True
except Exception:
    _HAS_GL = False

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import QTimer

_VEH_HEIGHT = {"car": 1.5, "van": 2.3, "bus": 3.0, "truck": 3.4}


def _box(w, l, h):
    """Solid box mesh (x=width, y=length, z=up), centred on the ground."""
    x, y = w / 2.0, l / 2.0
    verts = np.array([
        [-x, -y, 0], [x, -y, 0], [x, y, 0], [-x, y, 0],
        [-x, -y, h], [x, -y, h], [x, y, h], [-x, y, h]], dtype=float)
    faces = np.array([
        [0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7],
        [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
        [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]], dtype=int)
    return gl.MeshData(vertexes=verts, faces=faces)


class GpuView(QWidget):
    def __init__(self, state):
        super().__init__()
        self.state = state
        self.ok = _HAS_GL
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        if not _HAS_GL:
            lbl = QLabel("Real 3D view needs the GPU libraries:\n\n"
                         "pip install pyqtgraph PyOpenGL\n\n"
                         "(then restart UltraPilot)")
            lbl.setStyleSheet("color:#6B7280; font-size:14px;")
            lay.addWidget(lbl)
            return

        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor(15, 19, 24)
        self.view.opts["distance"] = 38
        self.view.opts["elevation"] = 16
        self.view.opts["azimuth"] = 90      # look forward (+y)
        self.view.setMinimumHeight(320)
        lay.addWidget(self.view)

        grid = gl.GLGridItem()
        grid.scale(5, 5, 1)
        self.view.addItem(grid)

        self.ego = gl.GLMeshItem(meshdata=_box(2.6, 6.5, 3.2),
                                 color=(0.06, 0.7, 0.45, 1), smooth=False,
                                 drawEdges=True, edgeColor=(0, 0.4, 0.3, 1))
        self.view.addItem(self.ego)

        self.route = gl.GLLinePlotItem(color=(0.23, 0.51, 0.96, 1), width=5, antialias=True)
        self.view.addItem(self.route)

        # Traffic light marker (a small box on a pole), recoloured each frame.
        self.light = gl.GLMeshItem(meshdata=_box(0.8, 0.8, 1.2), smooth=False,
                                   color=(0.4, 0.4, 0.4, 1))
        self.light.setVisible(False)
        self.view.addItem(self.light)

        self._veh_items = []
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(80)

    def refresh(self):
        if not _HAS_GL:
            return
        pos = self.state.get("truck_world_pos")
        if not pos:
            return
        h = self.state.get("truck_heading", 0.0) or 0.0
        sin_h, cos_h = math.sin(h), math.cos(h)

        def to_truck(wx, wz):
            dx, dz = wx - pos[0], wz - pos[1]
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lateral = dx * cos_h - dz * sin_h
            return ahead, lateral

        # Route polyline (x=lateral, y=ahead, slightly above ground).
        path = self.state.get("nav_path", []) or []
        if len(path) >= 2:
            pts = []
            for px, pz in path:
                a, l = to_truck(px, pz)
                pts.append([l, a, 0.1])
            try:
                self.route.setData(pos=np.array(pts, dtype=float))
            except Exception:
                pass

        # Surrounding vehicles as solid 3D boxes.
        for it in self._veh_items:
            self.view.removeItem(it)
        self._veh_items = []
        for v in (self.state.get("traffic", []) or [])[:30]:
            a, l = to_truck(v["x"], v["z"])
            if a < -10 or a > 95 or abs(l) > 26:
                continue
            hgt = _VEH_HEIGHT.get(v.get("type"), 1.6)
            item = gl.GLMeshItem(meshdata=_box(v.get("width", 2.2) or 2.2,
                                               v.get("length", 4.5) or 4.5, hgt),
                                 color=(0.62, 0.66, 0.72, 1), smooth=False,
                                 drawEdges=True, edgeColor=(0.3, 0.33, 0.38, 1))
            # rotate by relative yaw, then place at (lateral, ahead)
            rel = math.degrees((v.get("yaw", 0.0) or 0.0) - h)
            item.rotate(rel, 0, 0, 1)
            item.translate(l, a, 0)
            self.view.addItem(item)
            self._veh_items.append(item)

        # Traffic light (nearest, ahead) — recoloured by state.
        lt = self.state.get("traffic_light")
        if lt:
            a, l = to_truck(lt["x"], lt["z"])
            if 0 < a < 95:
                col = {"red": (0.9, 0.2, 0.2, 1), "green": (0.2, 0.85, 0.3, 1),
                       "yellow": (0.95, 0.75, 0.2, 1)}.get(lt.get("color"), (0.4, 0.4, 0.4, 1))
                self.light.setColor(col)
                self.light.resetTransform()
                self.light.translate(l, a, 2.5)
                self.light.setVisible(True)
            else:
                self.light.setVisible(False)
        else:
            self.light.setVisible(False)
