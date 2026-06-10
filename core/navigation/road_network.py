"""
Road-network parser (stage 2 of map-based navigation).

Loads the ETS2LA map dataset downloaded by map_data.py and exposes the road
geometry as simple ``(x, z)`` segments, with a grid spatial index so we can
quickly fetch the roads around the truck.

This stage is intentionally geometry-light and *read only*: it powers the map
view (drawing the road network around the truck) and will later feed lane points
to the steering controller.  It does NOT touch vehicle control.

Node and road formats follow ETS2LA's extracted data:
  nodes.json : [{uid, x, y, z, ...}]
  roads.json : [{uid, startNodeUid, endNodeUid, ...}]
"""

import os
import json
import math
import logging

try:
    import orjson
    def _loadf(path):
        with open(path, "rb") as f:
            return orjson.loads(f.read())
except Exception:
    def _loadf(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)


def _find_json(data_dir: str, category: str):
    """Find a <category>*.json file anywhere inside the dataset folder."""
    for root, _dirs, files in os.walk(data_dir):
        for f in files:
            if f.startswith(category) and f.endswith(".json"):
                return os.path.join(root, f)
    return None


class RoadNetwork:
    """In-memory road graph: node positions + segments, with a grid index."""

    GRID = 500.0  # metres per spatial-index cell

    def __init__(self):
        self.nodes = {}          # uid -> (x, z)
        self.segments = []       # [((x1,z1),(x2,z2)), ...]
        self._grid = {}          # (cx,cz) -> [segment_index, ...]
        self.loaded = False

    # --- Loading --------------------------------------------------------------
    def load(self, data_dir: str) -> bool:
        nodes_path = _find_json(data_dir, "nodes")
        roads_path = _find_json(data_dir, "roads")
        if not nodes_path or not roads_path:
            logging.error("road_network: nodes/roads json not found in %s", data_dir)
            return False

        try:
            raw_nodes = _loadf(nodes_path)
            for n in raw_nodes:
                self.nodes[n["uid"]] = (float(n["x"]), float(n["z"]))
        except Exception as e:
            logging.exception("road_network: failed to load nodes: %s", e)
            return False

        try:
            raw_roads = _loadf(roads_path)
            for r in raw_roads:
                a = self.nodes.get(r.get("startNodeUid"))
                b = self.nodes.get(r.get("endNodeUid"))
                if a and b:
                    self._add_segment(a, b)
        except Exception as e:
            logging.exception("road_network: failed to load roads: %s", e)
            return False

        self.loaded = True
        logging.info("road_network: loaded %d nodes, %d segments",
                     len(self.nodes), len(self.segments))
        return True

    def _cell(self, x, z):
        return (int(x // self.GRID), int(z // self.GRID))

    def _add_segment(self, a, b):
        idx = len(self.segments)
        self.segments.append((a, b))
        # Register in every grid cell the endpoints fall into.
        for p in (a, b):
            self._grid.setdefault(self._cell(*p), []).append(idx)

    # --- Queries --------------------------------------------------------------
    def segments_near(self, pos, radius: float = 800.0):
        """Return road segments with an endpoint within ``radius`` of ``pos``."""
        if not self.loaded or not pos:
            return []
        px, pz = pos
        cells = range(-(int(radius // self.GRID) + 1), int(radius // self.GRID) + 2)
        seen = set()
        out = []
        cx0, cz0 = self._cell(px, pz)
        r2 = radius * radius
        for dx in cells:
            for dz in cells:
                for idx in self._grid.get((cx0 + dx, cz0 + dz), ()):
                    if idx in seen:
                        continue
                    seen.add(idx)
                    a, b = self.segments[idx]
                    if (a[0] - px) ** 2 + (a[1] - pz) ** 2 <= r2 or \
                       (b[0] - px) ** 2 + (b[1] - pz) ** 2 <= r2:
                        out.append((a, b))
        return out

    def nearest_segment(self, pos):
        """Nearest road segment to ``pos`` (for localization). Returns seg or None."""
        near = self.segments_near(pos, 300.0) or self.segments_near(pos, 1500.0)
        if not near:
            return None
        px, pz = pos

        def dist2_to_seg(seg):
            (ax, az), (bx, bz) = seg
            dx, dz = bx - ax, bz - az
            L2 = dx * dx + dz * dz
            if L2 < 1e-9:
                return (ax - px) ** 2 + (az - pz) ** 2
            t = max(0.0, min(1.0, ((px - ax) * dx + (pz - az) * dz) / L2))
            qx, qz = ax + t * dx, az + t * dz
            return (qx - px) ** 2 + (qz - pz) ** 2

        return min(near, key=dist2_to_seg)
