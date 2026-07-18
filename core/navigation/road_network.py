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
import heapq
import time
from dataclasses import replace

from core.navigation.lane_model import (
    GpsCorridor, GpsCorridorEdge, LaneConnection, LaneId, LaneLocator,
    LanePath, LanePoint, LaneSegment,
)

CACHE_VERSION = 7  # lane metadata + prefab lane graph/path identity


def _uid(value):
    """Canonical ETS2 UID. JSON stores hexadecimal strings; SDK sends int64."""
    if isinstance(value, int):
        return value - (1 << 64) if value >= (1 << 63) else value
    if value in (None, "", "0"):
        return 0
    number = int(str(value), 16)
    return number - (1 << 64) if number >= (1 << 63) else number


def _forward_vector(transform):
    """ETS2 horizontal forward vector from a node/prefab quaternion."""
    quat = transform.get("rotationQuat") or transform.get("quaternion")
    if isinstance(quat, (list, tuple)) and len(quat) == 4:
        qw, qx, qy, qz = (float(value) for value in quat)
        magnitude = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if magnitude > 1e-9:
            qw, qx, qy, qz = (value / magnitude for value in (qw, qx, qy, qz))
            return (-2.0 * (qx*qz + qw*qy),
                    2.0 * (qx*qx + qy*qy) - 1.0)
    rotation = float(transform.get("rotation", 0.0) or 0.0)
    return (-math.sin(rotation), -math.cos(rotation))

try:
    import orjson
    def _loadf(path):
        with open(path, "rb") as f:
            return orjson.loads(f.read())
except Exception:
    def _loadf(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)


def _smooth(points, per_seg=6):
    """Catmull-Rom spline through the polyline → smooth, drivable curve."""
    if len(points) < 3:
        return points
    pts = [points[0]] + list(points) + [points[-1]]
    out = []
    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1], pts[i + 2]
        for s in range(per_seg):
            t = s / per_seg
            t2, t3 = t * t, t * t * t
            x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t +
                       (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
            z = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t +
                       (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
            out.append((x, z))
    out.append(points[-1])
    return out


def _find_json(data_dir: str, category: str):
    """Find a ``<category>.json`` file anywhere inside the dataset folder.

    ETS2LA names the files with a region prefix (``europe-nodes.json``,
    ``europe-roads.json``, ``promods-nodes.json`` ...) rather than the bare
    category, so we match the *suffix* of the stem instead of a prefix.
    Accepts ``nodes.json``, ``europe-nodes.json`` and ``promods_roads.json``.
    """
    cat = category.lower()
    for root, _dirs, files in os.walk(data_dir):
        for f in files:
            low = f.lower()
            if not low.endswith(".json"):
                continue
            stem = low[:-5]                      # filename without ".json"
            if stem == cat or stem.endswith("-" + cat) or stem.endswith("_" + cat):
                return os.path.join(root, f)
    return None


class RoadNetwork:
    """In-memory road graph: node positions + segments, with a grid index."""

    GRID = 500.0  # metres per spatial-index cell

    def __init__(self):
        self.nodes = {}          # uid -> (x, z)
        self.node_rot = {}       # uid -> map yaw, needed to place prefab curves
        self.node_alt = {}       # uid -> world elevation
        self.node_forward = {}   # uid -> quaternion-accurate horizontal tangent
        self.adj = {}            # uid -> [connected uid, ...]  (road graph, from roads.json)
        self.fwd = {}            # uid -> [uid, ...]  forward neighbours (graph.json)
        self.bwd = {}            # uid -> [uid, ...]  backward neighbours (graph.json)
        self._ngrid = {}         # (cx,cz) -> [uid, ...]  (node spatial index)
        self.segments = []       # [((x1,z1),(x2,z2)), ...]
        self._seg_uids = []      # [(start_uid, end_uid), ...]  parallel to segments
        self._seg_road_uids = [] # road item uid parallel to segments
        self._seg_look_tokens = []  # exact road-look token parallel to segments
        self._grid = {}          # (cx,cz) -> [segment_index, ...]  (legacy, endpoint-based)
        self._seg_grid = {}      # (cx,cz) -> [segment_index, ...]  (both endpoints)
        self.road_looks = {}     # token -> type, lane counts and direction split
        self._road_look_token = {}  # node_uid -> roadLookToken (nearest road's type)
        self._road_length = {}   # directed endpoint pair -> spline tangent length
        self._prefab_desc = {}   # token -> compact detailed prefab description
        self._prefab_grid = {}   # spatial index of placed prefab instances
        self._prefab_pairs = {}  # unordered endpoint UID pair -> prefab instances
        self._prefab_lane_data = {}  # token -> dataset lane/curve connectivity
        self._lane_cache = {}    # segment index -> tuple[LaneSegment, ...]
        self._lane_id_index = {} # LaneId -> LaneSegment (populated lazily)
        self._lane_path_revision = 0
        self.loaded = False

    # --- Loading --------------------------------------------------------------
    def load(self, data_dir: str) -> bool:
        # Fast path: a pickled cache of the parsed network, keyed on the mtimes
        # of the source JSON files. Loading 1.1M nodes from JSON takes ~5-7s;
        # unpickling the ready object takes ~1s. Rebuilds automatically when the
        # dataset is updated or its version changes.
        if self._try_load_cache(data_dir):
            return True

        nodes_path = _find_json(data_dir, "nodes")
        roads_path = _find_json(data_dir, "roads")
        if not nodes_path or not roads_path:
            logging.error("road_network: nodes/roads json not found in %s", data_dir)
            return False

        try:
            raw_nodes = _loadf(nodes_path)
            for n in raw_nodes:
                uid = _uid(n["uid"])
                # IMPORTANT coordinate mapping:
                #   ETS2 SDK reports the truck as (coordinateX, coordinateY, coordinateZ)
                #   where coordinateX/coordinateZ are the *horizontal* plane and
                #   coordinateY is altitude. The extracted nodes.json stores these
                #   as (x, y, z) with x/y horizontal and z = altitude — i.e. the
                #   axes are swapped vs. the SDK. We index by (x, y) so that a
                #   truck position (coordinateX, coordinateZ) from telemetry lands
                #   on the right road. Using z (altitude) here was the root cause
                #   of "navigation never works" — every node sat at altitude ~50.
                x, y = float(n["x"]), float(n["y"])
                self.nodes[uid] = (x, y)
                self.node_rot[uid] = float(n.get("rotation", 0.0) or 0.0)
                self.node_alt[uid] = float(n.get("z", 0.0) or 0.0)
                self.node_forward[uid] = _forward_vector(n)
                self._ngrid.setdefault(self._cell(x, y), []).append(uid)
        except Exception as e:
            logging.exception("road_network: failed to load nodes: %s", e)
            return False

        try:
            raw_roads = raw_roads  # noqa
        except Exception:
            pass
        try:
            raw_roads = _loadf(roads_path)
            for r in raw_roads:
                su, eu = _uid(r.get("startNodeUid")), _uid(r.get("endNodeUid"))
                a, b = self.nodes.get(su), self.nodes.get(eu)
                if a and b:
                    self._road_length[(su, eu)] = float(
                        r.get("length", math.dist(a, b)) or math.dist(a, b))
                    # Remember the endpoint uids alongside the geometry so that
                    # segment-snapping can recover graph nodes to walk from.
                    self._seg_uids.append((su, eu))
                    self._seg_road_uids.append(_uid(r.get("uid")))
                    self._seg_look_tokens.append(str(r.get("roadLookToken") or ""))
                    self._seg_grid.setdefault(self._cell(*a), []).append(len(self.segments))
                    if self._cell(*a) != self._cell(*b):
                        self._seg_grid.setdefault(self._cell(*b), []).append(len(self.segments))
                    self.segments.append((a, b))
                    self.adj.setdefault(su, []).append(eu)
                    self.adj.setdefault(eu, []).append(su)
                    # Remember the road-look token on both endpoints so
                    # road_type_at can classify the road we're driving on.
                    tok = r.get("roadLookToken")
                    if tok:
                        self._road_look_token[su] = tok
                        self._road_look_token[eu] = tok
        except Exception as e:
            logging.exception("road_network: failed to load roads: %s", e)
            return False

        self.loaded = True
        # Prefer the dense navigation graph (graph.json) when available — it's
        # the same graph ETS2LA's Map plugin pathfinds on, so it's far more
        # complete than the roads-only adjacency. Falls back silently to `adj`.
        self._load_nav_graph(data_dir)
        self._load_prefabs(data_dir)
        # Road-look table: classifies each road segment (motorway / expressway /
        # local / dirt) + lane count, used by the autopilot to slow down on
        # narrow/local roads and cap speed in city sectors.
        self._load_road_looks(data_dir)
        logging.info("road_network: loaded %d nodes, %d segments, nav-graph nodes=%d",
                     len(self.nodes), len(self.segments),
                     len(self.fwd) if self.fwd else len(self.adj))
        # Persist the parsed network so the next launch is fast (~1s vs ~6s).
        self._save_cache(data_dir)
        return True

    # --- Pickle cache ---------------------------------------------------------
    def _cache_path(self, data_dir: str) -> str:
        return os.path.join(data_dir, ".roadnet.cache")

    def _source_signature(self, data_dir: str):
        """(name, mtime, size) list of every source JSON the cache must honour.

        The cache is invalidated whenever any of these change — i.e. when the
        user re-downloads or switches the map dataset."""
        sig = []
        try:
            for root, _dirs, files in os.walk(data_dir):
                for f in files:
                    if f.endswith(".json"):
                        p = os.path.join(root, f)
                        st = os.stat(p)
                        sig.append((f, int(st.st_mtime), st.st_size))
        except Exception:
            return []
        sig.sort()
        return sig

    def _try_load_cache(self, data_dir: str) -> bool:
        """Load the pickled network if it matches the current sources."""
        path = self._cache_path(data_dir)
        if not os.path.exists(path):
            return False
        try:
            import pickle
            with open(path, "rb") as f:
                payload = pickle.load(f)
            # Invalidate if the source files changed since the cache was built.
            if (payload.get("version") != CACHE_VERSION
                    or payload.get("sig") != self._source_signature(data_dir)):
                logging.info("road_network: cache stale — rebuilding.")
                return False
            data = payload["data"]
            for k in ("nodes", "node_rot", "node_alt", "node_forward",
                      "adj", "fwd", "bwd", "_ngrid", "segments",
                      "_seg_uids", "_seg_road_uids", "_seg_look_tokens",
                      "_grid", "_seg_grid", "_road_look_token",
                      "_road_length", "road_looks", "_prefab_desc", "_prefab_grid",
                      "_prefab_pairs", "_prefab_lane_data", "loaded"):
                setattr(self, k, data.get(k, getattr(self, k)))
            self.loaded = bool(data.get("loaded", True))
            logging.info("road_network: loaded from cache (%d nodes, %d fwd).",
                         len(self.nodes), len(self.fwd))
            return True
        except Exception as e:
            logging.debug("road_network: cache read failed (%s) — rebuilding.", e)
            return False

    def _save_cache(self, data_dir: str):
        try:
            import pickle
            payload = {
                "version": CACHE_VERSION,
                "sig": self._source_signature(data_dir),
                "data": {
                    "nodes": self.nodes, "node_rot": self.node_rot,
                    "node_alt": self.node_alt, "node_forward": self.node_forward,
                    "adj": self.adj, "fwd": self.fwd,
                    "bwd": self.bwd, "_ngrid": self._ngrid, "segments": self.segments,
                    "_seg_uids": self._seg_uids,
                    "_seg_road_uids": self._seg_road_uids,
                    "_seg_look_tokens": self._seg_look_tokens,
                    "_grid": self._grid,
                    "_seg_grid": self._seg_grid, "_road_look_token": self._road_look_token,
                    "_road_length": self._road_length,
                    "road_looks": self.road_looks,
                    "_prefab_desc": self._prefab_desc,
                    "_prefab_grid": self._prefab_grid,
                    "_prefab_pairs": self._prefab_pairs,
                    "_prefab_lane_data": self._prefab_lane_data,
                    "loaded": self.loaded,
                },
            }
            with open(self._cache_path(data_dir), "wb") as f:
                pickle.dump(payload, f, protocol=pickle.HIGHEST_PROTOCOL)
            logging.info("road_network: wrote cache (%d nodes).", len(self.nodes))
        except Exception as e:
            logging.debug("road_network: cache write failed (%s).", e)

    def _load_nav_graph(self, data_dir: str):
        """Load the precomputed navigation graph (``graph.json``).

        ETS2LA ships this: a list of ``[uid, {"forward": [...], "backward": [...]}]``
        where each entry lists the connected node uids with distances/directions.
        It's denser and more correct than rebuilding adjacency from roads.json,
        which is what we need so ``path_ahead`` actually traces a long road
        instead of dying after one segment.
        """
        path = _find_json(data_dir, "graph")
        if not path:
            return
        try:
            raw = _loadf(path)
            nf = nb = 0
            for uid, data in raw:
                uid = _uid(uid)
                fw = [_uid(e["nodeId"]) for e in (data.get("forward") or []) if e.get("nodeId")]
                bw = [_uid(e["nodeId"]) for e in (data.get("backward") or []) if e.get("nodeId")]
                if fw:
                    self.fwd[uid] = fw
                    nf += 1
                if bw:
                    self.bwd[uid] = bw
                    nb += 1
            logging.info("road_network: nav-graph loaded (%d fwd / %d bwd nodes).", nf, nb)
        except Exception as e:
            logging.warning("road_network: nav-graph load failed (%s) — using roads.json graph.", e)

    def _load_prefabs(self, data_dir: str):
        """Load compact prefab navigation curves and placed instances.

        Roads end at prefab entrances.  Roundabouts/intersections live in
        ``prefabDescriptions.json`` as cubic nav curves; drawing or steering a
        straight graph chord between entrances cuts directly through the middle.
        """
        desc_path = _find_json(data_dir, "prefabDescriptions")
        inst_path = _find_json(data_dir, "prefabs")
        if not desc_path or not inst_path:
            return
        try:
            def forward(transform):
                """Horizontal forward vector used by ETS2LA's Hermite3D.

                Prefab rotations are not a conventional 2D yaw.  The source
                map includes the exact quaternion and ETS2LA rotates the local
                ``(0, 0, -1)`` vector with it.  Using ``cos(rotation)`` here
                turned every tangent by roughly 90 degrees, which produced the
                star-shaped roads at roundabouts.
                """
                return _forward_vector(transform)

            for raw in _loadf(desc_path):
                curves = []
                lane_curves = []
                for curve in raw.get("navCurves", ()):
                    start, end = curve.get("start", {}), curve.get("end", {})
                    start_forward = forward(start)
                    end_forward = forward(end)
                    curves.append((
                        float(start.get("x", 0)), float(start.get("y", 0)),
                        float(end.get("x", 0)), float(end.get("y", 0)),
                        start_forward[0], start_forward[1],
                        end_forward[0], end_forward[1],
                    ))
                    lane_curves.append({
                        "nav_node_index": int(curve.get("navNodeIndex", -1)),
                        "next_lines": tuple(int(i) for i in curve.get("nextLines", ())),
                        "prev_lines": tuple(int(i) for i in curve.get("prevLines", ())),
                        "start_y": float(start.get("z", 0.0) or 0.0),
                        "end_y": float(end.get("z", 0.0) or 0.0),
                    })
                nodes = tuple((float(node.get("x", 0)),
                               float(node.get("y", 0)),
                               float(node.get("rotation", 0)))
                              for node in raw.get("nodes", ()))
                nav_nodes = []
                for node in raw.get("navNodes", ()):
                    connections = tuple(
                        (int(connection.get("targetNavNodeIndex", -1)),
                         tuple(int(i) for i in connection.get("curveIndices", ())))
                        for connection in node.get("connections", ()))
                    nav_nodes.append((str(node.get("type", "")),
                                      int(node.get("endIndex", -1)), connections))
                self._prefab_desc[str(raw.get("token", ""))] = (
                    nodes, tuple(curves), tuple(nav_nodes))
                self._prefab_lane_data[str(raw.get("token", ""))] = {
                    "path": str(raw.get("path", "")),
                    "nodes": tuple({
                        "input_lanes": tuple(int(i) for i in node.get("inputLanes", ())),
                        "output_lanes": tuple(int(i) for i in node.get("outputLanes", ())),
                        "y": float(node.get("z", 0.0) or 0.0),
                    } for node in raw.get("nodes", ())),
                    "curves": tuple(lane_curves),
                }

            for raw in _loadf(inst_path):
                token = str(raw.get("token", ""))
                if token not in self._prefab_desc:
                    continue
                uids = tuple(_uid(value) for value in raw.get("nodeUids", ())
                             if _uid(value))
                if not uids:
                    continue
                instance = (token, uids, int(raw.get("originNodeIndex", 0)))
                x, z = float(raw.get("x", 0)), float(raw.get("y", 0))
                self._prefab_grid.setdefault(self._cell(x, z), []).append(instance)
                for i in range(len(uids)):
                    for j in range(i + 1, len(uids)):
                        pair = (min(uids[i], uids[j]), max(uids[i], uids[j]))
                        self._prefab_pairs.setdefault(pair, []).append(instance)
            logging.info("road_network: loaded %d prefab types / %d endpoint pairs",
                         len(self._prefab_desc), len(self._prefab_pairs))
        except Exception as error:
            logging.warning("road_network: detailed prefab load failed: %s", error)
            self._prefab_desc.clear()
            self._prefab_grid.clear()
            self._prefab_pairs.clear()
            self._prefab_lane_data.clear()

    @staticmethod
    def _hermite_curve(curve, spacing=2.25):
        """Sample one local 2D prefab curve with endpoint tangents."""
        sx, sy, ex, ey, sdx, sdy, edx, edy = curve
        length = math.hypot(ex - sx, ey - sy)
        count = max(4, min(80, int(length / spacing) + 1))
        m0 = (sdx * length, sdy * length)
        m1 = (edx * length, edy * length)
        points = []
        for index in range(count):
            t = index / (count - 1)
            t2, t3 = t * t, t * t * t
            h00, h10 = 2*t3 - 3*t2 + 1, t3 - 2*t2 + t
            h01, h11 = -2*t3 + 3*t2, t3 - t2
            points.append((h00*sx + h10*m0[0] + h01*ex + h11*m1[0],
                           h00*sy + h10*m0[1] + h01*ey + h11*m1[1]))
        return points

    def _transform_prefab_points(self, instance, points):
        token, uids, origin_index = instance
        desc = self._prefab_desc.get(token)
        anchor = self.nodes.get(uids[0]) if uids else None
        if not desc or anchor is None or not desc[0]:
            return []
        origin_index = max(0, min(origin_index, len(desc[0]) - 1))
        ox, oz, local_rot = desc[0][origin_index]
        rotation = self.node_rot.get(uids[0], 0.0) - local_rot
        c, s = math.cos(rotation), math.sin(rotation)
        ax, az = anchor
        return [(ax + (x - ox) * c - (z - oz) * s,
                 az + (x - ox) * s + (z - oz) * c) for x, z in points]

    def _prefab_curve_segments(self, instance, curve_indices=None):
        desc = self._prefab_desc.get(instance[0])
        if not desc:
            return []
        curves = desc[1]
        indices = curve_indices if curve_indices is not None else range(len(curves))
        result = []
        for index in indices:
            if not (0 <= index < len(curves)):
                continue
            points = self._transform_prefab_points(
                instance, self._hermite_curve(curves[index]))
            result.extend(zip(points, points[1:]))
        return result

    def _connected_prefab_points(self, instance, curve_indices, start_point):
        """Join a prefab route's lane curves without drawing chords between them.

        Curve indices are not guaranteed to be stored in spatial order and an
        individual curve can point either way.  Concatenating them verbatim was
        the source of the enormous blue zig-zags visible over the game.
        """
        desc = self._prefab_desc.get(instance[0])
        if not desc:
            return []
        pieces = []
        for index in curve_indices:
            if 0 <= index < len(desc[1]):
                points = self._transform_prefab_points(
                    instance, self._hermite_curve(desc[1][index]))
                if len(points) >= 2:
                    pieces.append(points)
        if not pieces:
            return []

        result = []
        cursor = tuple(start_point)
        # Greedily take the curve endpoint nearest to the preceding endpoint.
        # Real prefab lane pieces touch, so a generous 12 m guard still rejects
        # accidental connections to another arm of a large junction.
        while pieces:
            best_index = best_reverse = None
            best_distance = float("inf")
            for index, points in enumerate(pieces):
                for reverse, endpoint in ((False, points[0]), (True, points[-1])):
                    distance = math.dist(cursor, endpoint)
                    if distance < best_distance:
                        best_distance = distance
                        best_index, best_reverse = index, reverse
            if best_index is None or (result and best_distance > 12.0):
                break
            points = pieces.pop(best_index)
            if best_reverse:
                points.reverse()
            if not result:
                result.extend(points)
            else:
                result.extend(points[1:] if best_distance < 1.0 else points)
            cursor = result[-1]
        return result

    def prefab_segments_near(self, pos, radius=800.0, limit=10000):
        if not pos or not self._prefab_grid:
            return []
        px, pz = pos
        cx, cz = self._cell(px, pz)
        rings = int(radius // self.GRID) + 1
        seen, result = set(), []
        for dx in range(-rings, rings + 1):
            for dz in range(-rings, rings + 1):
                for instance in self._prefab_grid.get((cx + dx, cz + dz), ()):
                    marker = (instance[0], instance[1])
                    if marker in seen:
                        continue
                    seen.add(marker)
                    for segment in self._prefab_curve_segments(instance):
                        a, b = segment
                        if min((a[0]-px)**2 + (a[1]-pz)**2,
                               (b[0]-px)**2 + (b[1]-pz)**2) <= radius*radius:
                            result.append(segment)
                            if len(result) >= limit:
                                return result
        return result

    def _road_curve_3d(self, first, second, spacing=2.5):
        """Exact-ish Hermite centreline for a normal road, including height."""
        reverse = False
        tangent_length = self._road_length.get((first, second))
        if tangent_length is None:
            tangent_length = self._road_length.get((second, first))
            reverse = tangent_length is not None
        if tangent_length is None or first not in self.nodes or second not in self.nodes:
            return []
        if reverse:
            first, second = second, first
        sx, sz = self.nodes[first]
        ex, ez = self.nodes[second]
        sh, eh = self.node_alt.get(first, 0.0), self.node_alt.get(second, 0.0)
        sdx, sdz = self.node_forward.get(first, (0.0, 0.0))
        edx, edz = self.node_forward.get(second, (0.0, 0.0))
        chord = math.hypot(ex-sx, ez-sz)
        tangent_length = max(chord * 0.45, min(float(tangent_length), chord * 2.5))
        count = max(4, min(100, int(max(chord, tangent_length) / spacing) + 1))
        points = []
        for index in range(count):
            t = index / (count - 1)
            t2, t3 = t*t, t*t*t
            h00, h10 = 2*t3-3*t2+1, t3-2*t2+t
            h01, h11 = -2*t3+3*t2, t3-t2
            x = h00*sx + h10*sdx*tangent_length + h01*ex + h11*edx*tangent_length
            z = h00*sz + h10*sdz*tangent_length + h01*ez + h11*edz*tangent_length
            height = sh + (eh-sh)*t
            points.append((x, z, height))
        if reverse:
            points.reverse()
        return points

    def hud_segments_3d_near(self, pos, radius: float = 280.0, limit: int = 950,
                             altitude=None):
        """Curved road segments with elevation for the perspective HUD."""
        if not self.loaded or not pos:
            return []
        px, pz = pos
        cx, cz = self._cell(px, pz)
        rings = int(radius // self.GRID) + 1
        seen, ranked = set(), []
        for dx in range(-rings, rings + 1):
            for dz in range(-rings, rings + 1):
                for index in self._seg_grid.get((cx+dx, cz+dz), ()):
                    if index in seen:
                        continue
                    seen.add(index)
                    first, second = self._seg_uids[index]
                    token = (self._road_look_token.get(first)
                             or self._road_look_token.get(second))
                    lanes = int((self.road_looks.get(token) or {}).get("lanes", 2))
                    curve = self._road_curve_3d(first, second)
                    for curve_index, (a, b) in enumerate(zip(curve, curve[1:])):
                        distance2 = min((a[0]-px)**2+(a[1]-pz)**2,
                                        (b[0]-px)**2+(b[1]-pz)**2)
                        if distance2 <= radius*radius:
                            look = self.road_looks.get(token) or {}
                            divided = bool((look.get("lanes_left", 0)
                                            and look.get("lanes_right", 0))
                                           or (lanes >= 4 and look.get("type")
                                               in ("motorway", "expressway")))
                            # Fixed 7.5 m dash / 5 m gap in world space. Qt's
                            # screen-space DashLine restarted on every sampled
                            # curve and produced differently-sized markings.
                            dash_on = (curve_index % 5) < 3
                            pillar = (curve_index % 12) == 0
                            rail_post = (curve_index % 4) == 0
                            ranked.append((distance2, a, b, "road",
                                           max(1, lanes), divided, dash_on,
                                           pillar, rail_post))
        # Spatial index for the overlap check below. Scanning every ordinary
        # chord for every prefab chord made HUD publication unnecessarily
        # expensive in large interchanges.
        normal_bins = {}
        overlap_cell = 12.0
        for item in ranked:
            _, ra, rb, rkind, *_rest = item
            if rkind != "road":
                continue
            key = (math.floor(((ra[0] + rb[0]) * .5) / overlap_cell),
                   math.floor(((ra[1] + rb[1]) * .5) / overlap_cell))
            normal_bins.setdefault(key, []).append((ra, rb))

        # Prefab curves fill the otherwise missing geometry between ordinary
        # road objects at junctions. The HUD renders them only as an unmarked
        # asphalt underlay, never as independent outlined lanes.
        for prefab_index, (a, b) in enumerate(
                self.prefab_segments_near(pos, radius, limit * 2)):
            uid_a = self._nearest_node(a, max_ring=1)
            uid_b = self._nearest_node(b, max_ring=1)
            ah = self.node_alt.get(uid_a, 0.0)
            bh = self.node_alt.get(uid_b, ah)
            distance2 = min((a[0]-px)**2+(a[1]-pz)**2,
                            (b[0]-px)**2+(b[1]-pz)**2)
            # At an overpass, a nearest-X/Z lookup can borrow the bridge's node
            # height for the prefab road below. Close to the truck, suppress a
            # prefab assigned to another deck; the ordinary 3-D road segments
            # still render the real bridge at its own altitude.
            if (altitude is not None and distance2 < 90.0 ** 2
                    and min(abs(ah - altitude), abs(bh - altitude)) > 3.2):
                continue

            # Prefab curves bridge gaps at junctions, but some datasets repeat
            # an exit that is already represented by a normal road curve. Do
            # not publish a second parallel asphalt ribbon on top/beside it.
            pmx, pmz = (a[0] + b[0]) * .5, (a[1] + b[1]) * .5
            pvx, pvz = b[0] - a[0], b[1] - a[1]
            plen = math.hypot(pvx, pvz)
            duplicate = False
            if plen > .2:
                cell_x = math.floor(pmx / overlap_cell)
                cell_z = math.floor(pmz / overlap_cell)
                candidates = []
                for cell_dx in (-1, 0, 1):
                    for cell_dz in (-1, 0, 1):
                        candidates.extend(normal_bins.get(
                            (cell_x + cell_dx, cell_z + cell_dz), ()))
                for ra, rb in candidates:
                    rvx, rvz = rb[0] - ra[0], rb[1] - ra[1]
                    rlen2 = rvx * rvx + rvz * rvz
                    if rlen2 < .04:
                        continue
                    t = max(0.0, min(1.0,
                        ((pmx - ra[0]) * rvx + (pmz - ra[1]) * rvz) / rlen2))
                    qx, qz = ra[0] + rvx * t, ra[1] + rvz * t
                    if (pmx - qx) ** 2 + (pmz - qz) ** 2 > 3.2 ** 2:
                        continue
                    alignment = abs((pvx * rvx + pvz * rvz)
                                    / (plen * math.sqrt(rlen2)))
                    if alignment > .94:
                        duplicate = True
                        break
            if duplicate:
                continue
            ranked.append((distance2, (a[0], a[1], ah),
                           (b[0], b[1], bh), "lane", 1, False,
                           (prefab_index % 5) < 3,
                           False, False))
        ranked.sort(key=lambda item: item[0])
        return [(a, b, kind, lanes, divided, dash_on, pillar, rail_post)
                for _, a, b, kind, lanes, divided, dash_on, pillar, rail_post
                in ranked[:limit]]

    # --- Authoritative lane-level GPS route ---------------------------------
    def _road_pair_index(self):
        index = getattr(self, "_road_pair_index_cache", None)
        if index is None:
            index = {}
            for segment_index, (start, end) in enumerate(self._seg_uids):
                index.setdefault((min(start, end), max(start, end)), []).append(
                    segment_index)
            self._road_pair_index_cache = index
        return index

    def _classify_corridor_edge(self, start, end, gps_pair_index):
        pair = (min(start, end), max(start, end))
        prefab_instances = tuple(self._prefab_pairs.get(pair, ()))
        if prefab_instances:
            return GpsCorridorEdge(start, end, "prefab", gps_pair_index,
                                   prefab_instance=prefab_instances)
        road_indices = self._road_pair_index().get(pair, ())
        if len(road_indices) == 1:
            return GpsCorridorEdge(start, end, "road", gps_pair_index,
                                   segment_index=road_indices[0])
        if len(road_indices) > 1:
            return None
        # Only the directed extracted graph can prove an otherwise geometry-
        # less edge. It may be densified later, but never invented by distance.
        if (end in self.fwd.get(start, ())
                or end in self.bwd.get(start, ())):
            return GpsCorridorEdge(start, end, "graph", gps_pair_index)
        return None

    def resolve_gps_corridor(self, gps_uids):
        """Resolve sparse SDK UIDs without changing their authoritative order."""
        uids = tuple(_uid(value) for value in gps_uids if _uid(value))
        if len(uids) < 2:
            return GpsCorridor(uids, (), False,
                               "GPS corridor requires at least two non-zero UIDs")
        missing = [uid for uid in uids if uid not in self.nodes]
        if missing:
            return GpsCorridor(uids, (), False,
                               f"GPS UID {missing[0]} is absent from the active map")
        edges = []
        for pair_index, (start, goal) in enumerate(zip(uids, uids[1:])):
            if start == goal:
                continue
            direct = self._classify_corridor_edge(start, goal, pair_index)
            if direct is not None:
                edges.append(direct)
                continue
            bridge = self._route_bridge(start, goal)
            if len(bridge) < 2:
                return GpsCorridor(
                    uids, tuple(edges), False,
                    f"no directed topological path for GPS UID pair "
                    f"{start} -> {goal} at index {pair_index}")
            if bridge[0] != start or bridge[-1] != goal:
                return GpsCorridor(
                    uids, tuple(edges), False,
                    f"topological bridge changed authoritative GPS UID order "
                    f"{start} -> {goal}")
            for edge_start, edge_end in zip(bridge, bridge[1:]):
                edge = self._classify_corridor_edge(
                    edge_start, edge_end, pair_index)
                if edge is None:
                    return GpsCorridor(
                        uids, tuple(edges), False,
                        f"directed graph path contains unproven edge "
                        f"{edge_start} -> {edge_end}")
                edges.append(edge)
        if not edges:
            return GpsCorridor(uids, (), False, "GPS corridor contains no edges")
        return GpsCorridor(uids, tuple(edges), True)

    @staticmethod
    def _curve_chain_is_valid(lane_data, indices):
        curves = lane_data.get("curves", ())
        if not indices or any(not (0 <= index < len(curves)) for index in indices):
            return False
        for first, second in zip(indices, indices[1:]):
            a, b = curves[first], curves[second]
            if (second not in a["next_lines"]
                    or first not in b["prev_lines"]):
                return False
            if a["nav_node_index"] < 0 or b["nav_node_index"] < 0:
                return False
        return curves[indices[0]]["nav_node_index"] >= 0

    def _prefab_connector_options(self, instance, start_uid, end_uid):
        token, uids, _origin = instance
        try:
            start_item, end_item = uids.index(start_uid), uids.index(end_uid)
        except ValueError:
            return []
        desc = self._prefab_desc.get(token)
        lane_data = self._prefab_lane_data.get(token)
        if not desc or not lane_data:
            return []
        nav_nodes = desc[2]
        start_nav = next((i for i, node in enumerate(nav_nodes)
                          if node[0] == "physical" and node[1] == start_item), None)
        end_nav = next((i for i, node in enumerate(nav_nodes)
                        if node[0] == "physical" and node[1] == end_item), None)
        if start_nav is None or end_nav is None:
            return []
        options = []

        def walk(nav_index, curve_indices, visited):
            if len(visited) > len(nav_nodes) + 1:
                return
            if nav_index == end_nav:
                if self._curve_chain_is_valid(lane_data, curve_indices):
                    options.append(tuple(curve_indices))
                return
            for target, indices in nav_nodes[nav_index][2]:
                if target in visited or not indices:
                    continue
                combined = tuple(curve_indices) + tuple(indices)
                if curve_indices:
                    curves = lane_data["curves"]
                    if (indices[0] not in curves[curve_indices[-1]]["next_lines"]
                            or curve_indices[-1] not in curves[indices[0]]["prev_lines"]):
                        continue
                walk(target, combined, visited | {target})

        walk(start_nav, (), {start_nav})
        start_lanes = tuple(lane_data["nodes"][start_item]["input_lanes"])
        end_lanes = tuple(lane_data["nodes"][end_item]["output_lanes"])
        filtered = [indices for indices in options
                    if (not start_lanes or indices[0] in start_lanes)
                    and (not end_lanes or indices[-1] in end_lanes)]
        return sorted(set(filtered))

    def _prefab_curve_chain_3d(self, instance, indices):
        token, uids, origin_index = instance
        desc = self._prefab_desc[token]
        lane_data = self._prefab_lane_data[token]
        if not uids or not desc[0]:
            return ()
        origin_index = max(0, min(origin_index, len(desc[0]) - 1))
        if origin_index >= len(uids):
            return ()
        origin_uid = uids[origin_index]
        anchor = self.nodes.get(origin_uid)
        if anchor is None:
            return ()
        ox, oz, local_rotation = desc[0][origin_index]
        origin_y = lane_data["nodes"][origin_index]["y"]
        rotation = self.node_rot.get(origin_uid, 0.0) - local_rotation
        c, s = math.cos(rotation), math.sin(rotation)
        anchor_y = self.node_alt.get(origin_uid, 0.0)
        result = []
        for curve_index in indices:
            curve = desc[1][curve_index]
            local_points = self._hermite_curve(curve, spacing=2.25)
            curve_meta = lane_data["curves"][curve_index]
            count = max(1, len(local_points) - 1)
            piece = []
            for point_index, (x, z) in enumerate(local_points):
                fraction = point_index / count
                local_y = (curve_meta["start_y"]
                           + (curve_meta["end_y"] - curve_meta["start_y"])
                           * fraction)
                piece.append((
                    anchor[0] + (x - ox) * c - (z - oz) * s,
                    anchor_y + local_y - origin_y,
                    anchor[1] + (x - ox) * s + (z - oz) * c,
                ))
            if result and piece:
                if math.dist(result[-1], piece[0]) > 0.5:
                    return ()
                result.extend(piece[1:])
            else:
                result.extend(piece)
        lane_points, travelled = [], 0.0
        for index, point in enumerate(result):
            before = result[max(0, index - 1)]
            after = result[min(len(result) - 1, index + 1)]
            dx, dz = after[0] - before[0], after[2] - before[2]
            if lane_points:
                travelled += math.dist(point, result[index - 1])
            heading = (math.atan2(-dx, -dz)
                       if math.hypot(dx, dz) > 1e-8
                       else (lane_points[-1].heading if lane_points else 0.0))
            lane_points.append(LanePoint(point[0], point[1], point[2],
                                         travelled, heading))
        return tuple(lane_points)

    def _prefab_lane_segment(self, edge, lane_index):
        candidates = []
        for instance in edge.prefab_instance or ():
            token = instance[0]
            lane_data = self._prefab_lane_data.get(token) or {}
            try:
                start_item = instance[1].index(edge.start_uid)
            except ValueError:
                continue
            input_lanes = tuple((lane_data.get("nodes") or ())[start_item]
                                ["input_lanes"])
            try:
                end_item = instance[1].index(edge.end_uid)
                output_lanes = tuple((lane_data.get("nodes") or ())[end_item]
                                     ["output_lanes"])
            except (ValueError, IndexError):
                output_lanes = ()
            options = self._prefab_connector_options(
                instance, edge.start_uid, edge.end_uid)
            preferred_curve = (input_lanes[min(lane_index, len(input_lanes)-1)]
                               if input_lanes else None)
            preferred = [option for option in options
                         if preferred_curve is not None
                         and option[0] == preferred_curve]
            chosen_options = preferred or (options if len(options) == 1 else [])
            for indices in chosen_options:
                points = self._prefab_curve_chain_3d(instance, indices)
                if len(points) >= 2:
                    exit_lane_index = (output_lanes.index(indices[-1])
                                       if indices[-1] in output_lanes
                                       else lane_index)
                    candidates.append((instance, indices, points,
                                       exit_lane_index,
                                       max(1, len(output_lanes))))
        if len(candidates) != 1:
            return None, ("ambiguous prefab lane connector"
                          if candidates else "missing prefab lane connector")
        instance, indices, points, exit_lane_index, exit_lane_count = candidates[0]
        lane_data = self._prefab_lane_data.get(instance[0]) or {}
        lane_id = LaneId(min(instance[1]), 1, exit_lane_index,
                         instance[0], indices[0], tuple(indices))
        prefab_path = str(lane_data.get("path", "")).lower()
        segment = LaneSegment(
            lane_id, edge.start_uid, edge.end_uid, 1, exit_lane_index,
            exit_lane_count,
            4.5, "derived", int(round(points[len(points)//2].y / 3.0)),
            None, ("roundabout" if "roundabout" in prefab_path else "prefab"),
            points, connector_curve_indices=tuple(indices),
            gps_uids=frozenset((edge.start_uid, edge.end_uid)))
        self._lane_id_index[lane_id] = segment
        return segment, ""

    def _graph_lane_segment(self, edge, previous):
        if previous is None:
            return None
        start = self.nodes[edge.start_uid]
        end = self.nodes[edge.end_uid]
        start_y = self.node_alt.get(edge.start_uid, 0.0)
        end_y = self.node_alt.get(edge.end_uid, start_y)
        anchor = previous.centerline[-1]
        offset_x, offset_z = anchor.x - start[0], anchor.z - start[1]
        distance = math.dist(start, end)
        steps = max(2, int(math.ceil(distance / 3.0)) + 1)
        raw = []
        for index in range(steps):
            fraction = index / (steps - 1)
            raw.append((start[0] + (end[0]-start[0])*fraction + offset_x,
                        start_y + (end_y-start_y)*fraction,
                        start[1] + (end[1]-start[1])*fraction + offset_z))
        points, travelled = [], 0.0
        for index, point in enumerate(raw):
            if points:
                travelled += math.dist(raw[index-1], point)
            before, after = raw[max(0, index-1)], raw[min(len(raw)-1, index+1)]
            heading = math.atan2(-(after[0]-before[0]), -(after[2]-before[2]))
            points.append(LanePoint(*point, travelled, heading))
        lane_id = LaneId(edge.start_uid ^ edge.end_uid, 1,
                         previous.lane_index, "graph", edge.gps_pair_index)
        segment = LaneSegment(
            lane_id, edge.start_uid, edge.end_uid, 1, previous.lane_index, 1,
            previous.width_m, "derived",
            int(round(points[len(points)//2].y / 3.0)), None, "graph",
            tuple(points), gps_uids=frozenset((edge.start_uid, edge.end_uid)))
        self._lane_id_index[lane_id] = segment
        return segment

    @staticmethod
    def _lane_connection(first, second):
        if first.end_uid != second.start_uid:
            return None
        if first.lane_type == "roundabout" or second.lane_type == "roundabout":
            kind = "roundabout"
        elif first.lane_id.prefab_token or second.lane_id.prefab_token:
            kind = "prefab"
        elif second.lane_count > first.lane_count:
            kind = "split"
        elif second.lane_count < first.lane_count:
            kind = "merge"
        else:
            kind = "road"
        curves = (second.connector_curve_indices
                  if kind in ("prefab", "roundabout") else ())
        return LaneConnection(second.lane_id, kind, curves,
                              gps_exit_uid=second.end_uid)

    def select_lane_sequence(self, corridor, start_match):
        """Select one continuous lane for every authoritative corridor edge."""
        if not isinstance(corridor, GpsCorridor) or not corridor.valid:
            return (), (getattr(corridor, "failure_reason", "invalid corridor")
                        or "invalid corridor")
        if start_match is None:
            return (), "LaneLocator did not confirm a starting lane"
        selected = []
        lane_index = start_match.lane_id.lane_index
        for edge_number, edge in enumerate(corridor.edges):
            current = None
            if edge.kind == "road":
                lanes = [lane for lane in self._build_lane_segments(edge.segment_index)
                         if lane.start_uid == edge.start_uid
                         and lane.end_uid == edge.end_uid]
                if not lanes:
                    return tuple(selected), (
                        f"no lane geometry for directed road edge "
                        f"{edge.start_uid} -> {edge.end_uid}")
                by_index = {lane.lane_index: lane for lane in lanes}
                if lane_index in by_index:
                    current = by_index[lane_index]
                elif selected and lane_index >= len(lanes):
                    # A disappearing outer lane is a confirmed merge at the
                    # shared road node. Clamp only to the adjacent edge lane.
                    lane_index = max(by_index)
                    current = by_index[lane_index]
                else:
                    return tuple(selected), (
                        f"starting lane {lane_index} is unavailable on road edge "
                        f"{edge.start_uid} -> {edge.end_uid}")
            elif edge.kind == "prefab":
                current, reason = self._prefab_lane_segment(edge, lane_index)
                if current is None:
                    return tuple(selected), (
                        f"{reason} for prefab {edge.start_uid} -> {edge.end_uid}")
            else:
                # A directed graph edge proves node reachability, but it has no
                # concrete lane centre, width or elevation. Do not invent one.
                return tuple(selected), (
                    f"graph-only edge {edge.start_uid} -> {edge.end_uid} "
                    "has no lane-confirmed geometry")
            if selected:
                connection = self._lane_connection(selected[-1], current)
                if connection is None:
                    return tuple(selected), (
                        f"no LaneConnection from {selected[-1].end_uid} to "
                        f"{current.start_uid} at corridor edge {edge_number}")
                selected[-1] = replace(selected[-1],
                                       successors=(connection,))
            selected.append(current)
            lane_index = current.lane_index
        return tuple(selected), ""

    def connect_lane_sequence(self, segments, gps_uids):
        """Join only confirmed lane connections into an unsmoothed 3-D path."""
        segments = list(segments)
        uids = tuple(_uid(value) for value in gps_uids if _uid(value))
        if not segments:
            return LanePath((), (), uids, valid=False,
                            failure_reason="lane sequence is empty")
        # Placed SCS prefab graph anchors can lie outside the compact local
        # nav-curve footprint. Fit only an already confirmed prefab transition
        # to its adjacent lane centres; never bridge an unconfirmed graph gap.
        for index, segment in enumerate(tuple(segments)):
            if segment.lane_id.prefab_token in (None, "graph"):
                continue
            previous = segments[index - 1] if index else None
            following = segments[index + 1] if index + 1 < len(segments) else None
            original_start, original_end = segment.centerline[0], segment.centerline[-1]
            start = previous.centerline[-1] if previous is not None else original_start
            end = following.centerline[0] if following is not None else original_end
            start_gap = math.dist((original_start.x, original_start.y, original_start.z),
                                  (start.x, start.y, start.z))
            end_gap = math.dist((original_end.x, original_end.y, original_end.z),
                                (end.x, end.y, end.z))
            if max(start_gap, end_gap) <= 6.0:
                continue
            # Never replace a curved prefab/roundabout connector with a single
            # endpoint fit.  That construction cuts across the island even
            # though the prefab nav-curves correctly travel around it.  A
            # misplaced curved connector must fail closed at the geometry-gap
            # check below rather than becoming an unsafe shortcut.
            source_length = sum(math.dist(
                (a.x, a.y, a.z), (b.x, b.y, b.z))
                for a, b in zip(segment.centerline, segment.centerline[1:]))
            source_chord = math.dist(
                (original_start.x, original_start.y, original_start.z),
                (original_end.x, original_end.y, original_end.z))
            source_turn = sum(abs(
                (b.heading - a.heading + math.pi) % (2.0 * math.pi) - math.pi)
                              for a, b in zip(segment.centerline,
                                              segment.centerline[1:]))
            curved_connector = bool(
                segment.lane_type == "roundabout"
                or source_turn > math.radians(35.0)
                or (source_chord > 1.0
                    and source_length / source_chord > 1.10))
            if curved_connector:
                target_dx, target_dz = end.x - start.x, end.z - start.z
                target_chord = math.hypot(target_dx, target_dz)
                source_dx = original_end.x - original_start.x
                source_dz = original_end.z - original_start.z
                source_chord_xz = math.hypot(source_dx, source_dz)
                if source_chord_xz < 1.0 or target_chord < 1.0:
                    continue
                scale = target_chord / source_chord_xz
                # A large similarity scale turns a compact roundabout arc into
                # a shortcut across its island. Roundabout nav-curves must be
                # close to their placed prefab scale or fail closed.
                scale_valid = (0.72 <= scale <= 1.38
                               if segment.lane_type == "roundabout"
                               else 0.35 <= scale <= 5.0)
                if not scale_valid:
                    continue
                source_angle = math.atan2(source_dz, source_dx)
                target_angle = math.atan2(target_dz, target_dx)
                rotation = target_angle - source_angle
                cosine, sine = math.cos(rotation), math.sin(rotation)
                fitted = []
                count = max(1, len(segment.centerline) - 1)
                for point_index, point in enumerate(segment.centerline):
                    local_x = (point.x - original_start.x) * scale
                    local_z = (point.z - original_start.z) * scale
                    fraction = point_index / count
                    # Preserve the complete prefab curve instead of replacing
                    # it with an endpoint chord. Endpoint correction in Y is
                    # linear; X/Z undergo one shape-preserving similarity fit.
                    fitted.append(LanePoint(
                        start.x + local_x * cosine - local_z * sine,
                        point.y + (start.y - original_start.y) * (1.0-fraction)
                        + (end.y - original_end.y) * fraction,
                        start.z + local_x * sine + local_z * cosine,
                        lane_id=segment.lane_id, segment_index=index,
                    ))
                dense = [fitted[0]]
                for first_point, second_point in zip(fitted, fitted[1:]):
                    gap = math.dist(
                        (first_point.x, first_point.y, first_point.z),
                        (second_point.x, second_point.y, second_point.z))
                    steps = max(1, int(math.ceil(gap / 2.25)))
                    for step in range(1, steps + 1):
                        fraction = step / steps
                        dense.append(LanePoint(
                            first_point.x + (second_point.x-first_point.x)*fraction,
                            first_point.y + (second_point.y-first_point.y)*fraction,
                            first_point.z + (second_point.z-first_point.z)*fraction,
                            lane_id=segment.lane_id, segment_index=index,
                        ))
                segments[index] = replace(segment, centerline=tuple(dense))
                continue
            if (max(start_gap, end_gap) > 140.0
                    or abs(start.y - end.y) > 6.0
                    or (previous is None and following is None)):
                continue
            start_heading = (previous.centerline[-1].heading if previous is not None
                             else following.centerline[0].heading)
            end_heading = (following.centerline[0].heading if following is not None
                           else previous.centerline[-1].heading)
            distance = math.dist((start.x, start.y, start.z),
                                 (end.x, end.y, end.z))
            if distance < 1.0 or distance > 150.0:
                continue
            tangent = min(18.0, distance * 0.38)
            count = max(5, int(math.ceil(distance / 2.0)) + 1)
            fitted = []
            for point_index in range(count):
                t = point_index / (count - 1)
                t2, t3 = t*t, t*t*t
                h00, h10 = 2*t3 - 3*t2 + 1, t3 - 2*t2 + t
                h01, h11 = -2*t3 + 3*t2, t3 - t2
                sdx, sdz = -math.sin(start_heading), -math.cos(start_heading)
                edx, edz = -math.sin(end_heading), -math.cos(end_heading)
                fitted.append(LanePoint(
                    h00*start.x + h10*tangent*sdx
                    + h01*end.x + h11*tangent*edx,
                    start.y + (end.y-start.y)*t,
                    h00*start.z + h10*tangent*sdz
                    + h01*end.z + h11*tangent*edz,
                    lane_id=segment.lane_id, segment_index=index,
                ))
            segments[index] = replace(segment, centerline=tuple(fitted))

        segments = tuple(segments)
        # Topology alone is not permission to steer through a reversed prefab
        # arm. Confirm that its entry and exit tangents agree with the adjacent
        # lanes before publishing the blue line or steering authority.
        for index, segment in enumerate(segments):
            if segment.lane_id.prefab_token in (None, "graph"):
                continue
            points = segment.centerline
            checks = []
            if index and len(points) >= 2:
                previous = segments[index - 1].centerline
                if len(previous) >= 2:
                    checks.append((previous[-2], previous[-1],
                                   points[0], points[1], "entry"))
            if index + 1 < len(segments) and len(points) >= 2:
                following = segments[index + 1].centerline
                if len(following) >= 2:
                    checks.append((points[-2], points[-1],
                                   following[0], following[1], "exit"))
            for a, b, c, d, boundary in checks:
                first = math.atan2(-(b.x-a.x), -(b.z-a.z))
                second = math.atan2(-(d.x-c.x), -(d.z-c.z))
                jump = abs((second-first+math.pi) % (2*math.pi)-math.pi)
                # Prefab boundary samples can legitimately turn sharply at a
                # compact city junction.  More than 75 degrees, however, is a
                # reversed/crossing arm and must never become lane authority.
                if jump > math.radians(75.0):
                    return LanePath(
                        segments, (), uids, valid=False,
                        failure_reason=(
                            f"prefab {boundary} direction jump is "
                            f"{math.degrees(jump):.1f} degrees at UID "
                            f"{segment.start_uid}"))
        points = []
        for index, segment in enumerate(segments):
            if len(segment.centerline) < 2:
                return LanePath(segments, tuple(points), uids, valid=False,
                    failure_reason=f"LaneSegment {segment.lane_id} has no geometry")
            if index:
                previous = segments[index - 1]
                if not any(connection.target == segment.lane_id
                           for connection in previous.successors):
                    return LanePath(segments, tuple(points), uids, valid=False,
                        failure_reason=(f"unconfirmed lane transition "
                                        f"{previous.lane_id} -> {segment.lane_id}"))
                gap = math.dist((points[-1].x, points[-1].y, points[-1].z),
                                (segment.centerline[0].x,
                                 segment.centerline[0].y,
                                 segment.centerline[0].z))
                if gap > 6.0:
                    return LanePath(segments, tuple(points), uids, valid=False,
                        failure_reason=(f"confirmed lane transition has {gap:.1f} m "
                                        f"geometry gap at UID {segment.start_uid}"))
                if gap > 0.35:
                    # This only densifies an already confirmed LaneConnection.
                    start, end = points[-1], segment.centerline[0]
                    steps = max(2, int(math.ceil(gap / 2.0)))
                    for step in range(1, steps):
                        fraction = step / steps
                        points.append(LanePoint(
                            start.x + (end.x-start.x)*fraction,
                            start.y + (end.y-start.y)*fraction,
                            start.z + (end.z-start.z)*fraction))
            points.extend(segment.centerline[1:] if points and
                          math.dist((points[-1].x, points[-1].y, points[-1].z),
                                    (segment.centerline[0].x,
                                     segment.centerline[0].y,
                                     segment.centerline[0].z)) <= 0.35
                          else segment.centerline)
        rebuilt, distance = [], 0.0
        for index, point in enumerate(points):
            if rebuilt:
                distance += math.dist((rebuilt[-1].x, rebuilt[-1].y, rebuilt[-1].z),
                                      (point.x, point.y, point.z))
            before = points[max(0, index-1)]
            after = points[min(len(points)-1, index+1)]
            dx, dz = after.x-before.x, after.z-before.z
            heading = (math.atan2(-dx, -dz) if math.hypot(dx, dz) > 1e-8
                       else (rebuilt[-1].heading if rebuilt else point.heading))
            rebuilt.append(LanePoint(point.x, point.y, point.z,
                                     distance, heading, point.curvature))
        prefab_count = sum(segment.lane_id.prefab_token not in (None, "graph")
                           for segment in segments)
        graph_count = sum(segment.lane_id.prefab_token == "graph"
                          for segment in segments)
        confidence = max(0.0, 0.98 - prefab_count * 0.01 - graph_count * 0.08)
        self._lane_path_revision += 1
        return LanePath(segments, tuple(rebuilt), uids, distance, confidence,
                        True, "", self._lane_path_revision)

    def build_lane_path(self, gps_uids, position, heading, altitude=None,
                        previous_match=None, start_match=None):
        """Convenience pipeline used by tests and the future map-plugin switch."""
        corridor = self.resolve_gps_corridor(gps_uids)
        if not corridor.valid:
            return LanePath((), (), corridor.gps_uids, valid=False,
                            failure_reason=corridor.failure_reason), None
        if altitude is None:
            locator_position = tuple(position[:2])
        else:
            locator_position = (float(position[0]), float(altitude),
                                float(position[1]))
        match = start_match
        if match is None:
            match = LaneLocator(self).locate(locator_position, heading,
                                             corridor.gps_uids, previous_match)
        segments, reason = self.select_lane_sequence(corridor, match)
        if reason:
            return LanePath(segments, (), corridor.gps_uids, valid=False,
                            failure_reason=reason), match
        # The rolling SDK route begins at the next GPS anchor. The truck may
        # still be on the confirmed incoming lane leading to that anchor. Add
        # this real lane segment before the first corridor edge so HUD, AR and
        # steering start at the truck instead of 10+ metres across the prefab.
        active = self._lane_id_index.get(match.lane_id) if match else None
        if (active is not None and segments
                and active.lane_id != segments[0].lane_id
                and active.end_uid == segments[0].start_uid):
            connection = self._lane_connection(active, segments[0])
            if connection is not None:
                active = replace(active, successors=(connection,))
                segments = (active,) + tuple(segments)

        # Trim the actual first LaneSegment as well as the flattened LanePath.
        # build_lane_trajectory() deliberately rebuilds its control geometry
        # from segments, so trimming only LanePath.points resurrected the part
        # of the incoming road behind the truck and produced a screen-wide AR
        # chord.  This keeps every consumer on the same forward-only geometry.
        if match is not None and segments and segments[0].lane_id == match.lane_id:
            first = segments[0]
            line = first.centerline
            if len(line) >= 2:
                nearest = min(range(len(line)), key=lambda index: math.dist(
                    (line[index].x, line[index].y, line[index].z),
                    (match.point.x, match.point.y, match.point.z)))
                if nearest > 0:
                    trimmed = tuple(line[nearest:])
                    if len(trimmed) >= 2:
                        first = replace(first, centerline=trimmed)
                        segments = (first,) + tuple(segments[1:])
        path = self.connect_lane_sequence(segments, corridor.gps_uids)
        if not path.valid or match is None or len(path.points) < 2:
            return path, match

        # The native GPS buffer can begin at the entrance of a prefab while
        # the truck is already part-way through it. Publishing those points
        # behind the camera made AR draw a giant line across the whole screen.
        # Trim only to the nearest confirmed point and rebuild arc distance;
        # never translate or laterally offset the authoritative geometry.
        nearest = min(range(len(path.points)), key=lambda index: math.dist(
            (path.points[index].x, path.points[index].y, path.points[index].z),
            (match.point.x, match.point.y, match.point.z)))
        nearest_distance = math.dist(
            (path.points[nearest].x, path.points[nearest].y, path.points[nearest].z),
            (match.point.x, match.point.y, match.point.z))
        if nearest > 0 and nearest_distance <= 8.0:
            source = path.points[nearest:]
            rebuilt, distance = [], 0.0
            for index, point in enumerate(source):
                if rebuilt:
                    distance += math.dist(
                        (rebuilt[-1].x, rebuilt[-1].y, rebuilt[-1].z),
                        (point.x, point.y, point.z))
                rebuilt.append(replace(point, s=distance))
            path = replace(path, points=tuple(rebuilt), distance_m=distance,
                           confidence=min(path.confidence, match.confidence))
        return path, match

    def refine_route(self, uids, progress=None):
        """Replace prefab entrance chords in a GPS UID route with nav curves."""
        self._last_refine_complete = True
        self._last_refine_error = ""
        deadline = time.monotonic() + 25.0
        uids = [_uid(value) for value in uids]
        if not uids:
            return []
        result = [self.nodes[uids[0]]] if uids[0] in self.nodes else []
        pairs = list(zip(uids, uids[1:]))
        for pair_index, (first, second) in enumerate(pairs, 1):
            if time.monotonic() >= deadline:
                self._last_refine_complete = False
                self._last_refine_error = f"časový limit pri úseku {pair_index}/{len(pairs)}"
                logging.warning("road_network: GPS route refinement timed out at %d/%d sections",
                                pair_index, len(pairs))
                break
            if progress:
                progress(pair_index, len(pairs), 0)
            target = self.nodes.get(second)
            pair = (min(first, second), max(first, second))
            detailed = None
            for instance in self._prefab_pairs.get(pair, ()):
                try:
                    start_item = instance[1].index(first)
                    end_item = instance[1].index(second)
                except ValueError:
                    continue
                desc = self._prefab_desc.get(instance[0])
                if not desc:
                    continue
                nav_nodes = desc[2]
                start_nav = next((i for i, node in enumerate(nav_nodes)
                                  if node[0] == "physical" and node[1] == start_item), None)
                end_nav = next((i for i, node in enumerate(nav_nodes)
                                if node[0] == "physical" and node[1] == end_item), None)
                indices = None
                if start_nav is not None and end_nav is not None:
                    indices = next((conn[1] for conn in nav_nodes[start_nav][2]
                                    if conn[0] == end_nav), None)
                    if indices is None:
                        indices = next((conn[1] for conn in nav_nodes[end_nav][2]
                                        if conn[0] == start_nav), None)
                if indices:
                    detailed = self._connected_prefab_points(
                        instance, indices, self.nodes[first])
                    break
            if detailed:
                start_gap = math.dist(result[-1], detailed[0]) if result else 0.0
                end_gap = math.dist(detailed[-1], target) if target is not None else 0.0
                # A prefab transform can be offset from the SDK endpoint. Do
                # not abort the whole route; fall through to graph bridging.
                if start_gap > 12.0 or end_gap > 12.0:
                    detailed = None
            if detailed:
                gap = math.dist(result[-1], detailed[0]) if result else 0.0
                result.extend(detailed[1:] if gap < 1.0 else detailed)
                if target is not None:
                    result.append(target)
            elif ((first, second) in self._road_length
                  or (second, first) in self._road_length):
                curve = self._road_curve_3d(first, second)
                points = [(point[0], point[1]) for point in curve]
                if points:
                    if result and math.dist(result[-1], points[-1]) < math.dist(result[-1], points[0]):
                        points.reverse()
                    gap = math.dist(result[-1], points[0]) if result else 0.0
                    if gap > 12.0:
                        self._last_refine_complete = False
                        self._last_refine_error = (
                            f"nesúvislá cestná krivka {first} → {second}, medzera {gap:.0f} m")
                        break
                    result.extend(points[1:] if gap < 1.0 else points)
            elif target is not None:
                # SDK route nodes are deliberately sparse (dozens of nodes can
                # represent 100+ km). Fill a non-adjacent pair through the map
                # graph instead of appending a kilometre-long straight chord.
                gap = math.dist(result[-1], target) if result else 0.0
                if gap > 40.0:
                    bridge = self._route_bridge(
                        first, second,
                        progress=(lambda expanded, pi=pair_index:
                                  progress(pi, len(pairs), expanded))
                        if progress else None)
                    if len(bridge) < 2:
                        bridge = self._route_bridge_nearby(
                            first, second, max_offset=38.0,
                            progress=(lambda expanded, pi=pair_index:
                                      progress(pi, len(pairs), expanded))
                            if progress else None)
                    if len(bridge) < 2:
                        self._last_refine_complete = False
                        self._last_refine_error = (
                            f"cestný graf nespojil uzly {first} → {second}, medzera {gap:.0f} m")
                        logging.warning(
                            "road_network: cannot connect sparse GPS nodes %s -> %s (%.0f m)",
                            first, second, gap)
                        break
                    for bridge_a, bridge_b in zip(bridge, bridge[1:]):
                        curve = self._road_curve_3d(bridge_a, bridge_b)
                        points = [(point[0], point[1]) for point in curve]
                        if points:
                            if (result and math.dist(result[-1], points[-1])
                                    < math.dist(result[-1], points[0])):
                                points.reverse()
                            result.extend(points[1:] if result and
                                          math.dist(result[-1], points[0]) < 1.0
                                          else points)
                        else:
                            # The directed graph can contain a valid edge even
                            # when the decorative road-curve record is absent.
                            # Densify that authoritative edge so downstream
                            # continuity checks do not see one 67 m chord.
                            target_point = self.nodes[bridge_b]
                            source_point = result[-1]
                            edge_length = math.dist(source_point, target_point)
                            steps = max(1, int(math.ceil(edge_length / 8.0)))
                            for step in range(1, steps + 1):
                                fraction = step / steps
                                result.append((
                                    source_point[0] + (target_point[0] - source_point[0]) * fraction,
                                    source_point[1] + (target_point[1] - source_point[1]) * fraction,
                                ))
                else:
                    result.append(target)
        return result

    def _route_bridge_nearby(self, start, goal, max_offset=38.0,
                             progress=None):
        """Bridge sparse SDK nodes through compatible nearby graph nodes.

        Map versions occasionally rename/remove one endpoint while the road
        around it remains connected.  A direct chord creates shortcuts; this
        bounded fallback instead relocates each endpoint by at most one road
        segment and still requires a real directed graph path between them.
        """
        if start not in self.nodes or goal not in self.nodes:
            return []

        def candidates(uid):
            px, pz = self.nodes[uid]
            cx, cz = self._cell(px, pz)
            rings = max(1, int(math.ceil(max_offset / self.GRID)))
            found = [(0.0, uid)]
            seen = {uid}
            for dx in range(-rings, rings + 1):
                for dz in range(-rings, rings + 1):
                    for other in self._ngrid.get((cx + dx, cz + dz), ()):
                        if other in seen:
                            continue
                        distance = math.dist((px, pz), self.nodes[other])
                        if distance <= max_offset:
                            seen.add(other)
                            found.append((distance, other))
            found.sort()
            return found[:5]

        best = None
        for start_gap, candidate_start in candidates(start):
            for goal_gap, candidate_goal in candidates(goal):
                if candidate_start == start and candidate_goal == goal:
                    continue
                path = self._route_bridge(candidate_start, candidate_goal,
                                          max_expanded=6000,
                                          progress=progress)
                if len(path) < 2:
                    continue
                graph_length = sum(
                    math.dist(self.nodes[a], self.nodes[b])
                    for a, b in zip(path, path[1:]))
                score = start_gap + graph_length + goal_gap
                if best is None or score < best[0]:
                    best = (score, candidate_start, candidate_goal, path)
        if best is None:
            return []
        score, candidate_start, candidate_goal, path = best
        result = ([start] if candidate_start != start else []) + path
        if candidate_goal != goal:
            result.append(goal)
        logging.info(
            "road_network: recovered sparse GPS gap %s -> %s via nearby graph "
            "nodes %s -> %s (%.1f m).",
            start, goal, candidate_start, candidate_goal, score)
        return result

    def _route_bridge(self, start, goal, max_expanded=12000, progress=None):
        """A* bridge between two sparse SDK GPS nodes on the loaded graph."""
        if start == goal:
            return [start]
        cache = getattr(self, "_route_bridge_cache", None)
        if cache is None:
            cache = self._route_bridge_cache = {}
        key = (start, goal)
        if key in cache:
            return cache[key]
        if start not in self.nodes or goal not in self.nodes:
            return []

        # An explicit directed edge is authoritative. Some extracted graph
        # nodes also expose a long path in the opposite-direction table; the
        # old ambiguity check then rejected even this exact 67 m connection.
        # Route order already tells us start -> goal, so never replace a direct
        # edge with (or reject it because of) an unrelated reverse detour.
        if goal in self.fwd.get(start, ()):
            path = [start, goal]
            cache[key] = path
            return path
        if goal in self.bwd.get(start, ()) and not self.fwd.get(start):
            path = [start, goal]
            cache[key] = path
            return path

        gx, gz = self.nodes[goal]

        def directed_search(graph):
            queue = [(math.dist(self.nodes[start], self.nodes[goal]), 0.0, start)]
            cost, previous, expanded = {start: 0.0}, {}, 0
            while queue and expanded < max_expanded:
                _score, current_cost, current = heapq.heappop(queue)
                if current_cost != cost.get(current):
                    continue
                if current == goal:
                    path = [goal]
                    while path[-1] != start:
                        path.append(previous[path[-1]])
                    path.reverse()
                    return path
                expanded += 1
                if progress and expanded % 250 == 0:
                    progress(expanded)
                cx, cz = self.nodes[current]
                for neighbour in graph.get(current, ()):
                    point = self.nodes.get(neighbour)
                    if point is None:
                        continue
                    new_cost = current_cost + math.hypot(point[0] - cx,
                                                         point[1] - cz)
                    if new_cost >= cost.get(neighbour, float("inf")):
                        continue
                    cost[neighbour], previous[neighbour] = new_cost, current
                    heuristic = math.hypot(point[0] - gx, point[1] - gz)
                    heapq.heappush(queue, (new_cost + heuristic,
                                           new_cost, neighbour))
            return []

        # Never mix forward and reverse edges in one A* search. That allowed
        # illegal U-turns and shortcuts across roundabout islands/medians.
        forward = directed_search(self.fwd)
        backward = directed_search(self.bwd)
        if forward and backward and forward != backward:
            logging.warning("road_network: ambiguous directed bridge %s -> %s", start, goal)
            cache[key] = []
            return []
        path = forward or backward
        if path:
            cache[key] = path
            return path
        # Older datasets may only contain undirected road adjacency. It is safe
        # as a fallback only when no directed graph exists at either endpoint.
        if not self.fwd.get(start) and not self.bwd.get(start):
            path = directed_search(self.adj)
            if path:
                cache[key] = path
                return path
        # Do not cache a failed limited search: a later recovery pass may use a
        # larger expansion budget after the map graph has finished loading.
        return []

    def _load_road_looks(self, data_dir: str):
        """Load the road-look table (``roadLooks.json``).

        Classifies each road-look token into a coarse type + lane count, used by
        ``road_type_at`` so the autopilot can slow down on local/narrow roads and
        keep full speed on motorways. Built from the ``name`` and the
        ``lanesLeft/Right`` lists ETS2LA ships in the dataset."""
        path = _find_json(data_dir, "roadLooks")
        if not path:
            return
        try:
            raw = _loadf(path)
            for r in raw:
                tok = r.get("token")
                if not tok:
                    continue
                name = (r.get("name", "") or "").lower()
                lanes_l = r.get("lanesLeft", []) or []
                lanes_r = r.get("lanesRight", []) or []
                lanes = len(lanes_l) + len(lanes_r)
                lane_str = " ".join(lanes_l + lanes_r).lower()
                if "motorway" in lane_str or "highway" in name:
                    rtype = "motorway"
                elif "expressway" in lane_str or "express" in name:
                    rtype = "expressway"
                elif "dirt" in name or "minim" in name or "ground" in name:
                    rtype = "dirt"
                elif "local" in lane_str or "old road" in name:
                    rtype = "local"
                else:
                    rtype = "local" if lanes <= 2 else "expressway"
                self.road_looks[tok] = {
                    "type": rtype, "lanes": max(1, lanes),
                    "lanes_left": len(lanes_l), "lanes_right": len(lanes_r),
                    # These are the lane facts actually present in the map.
                    # Width is absent and is therefore deliberately not stored
                    # as a dataset value (lane construction marks it derived).
                    "lane_types_left": tuple(str(value) for value in lanes_l),
                    "lane_types_right": tuple(str(value) for value in lanes_r),
                    "offset_m": float(r.get("offset", 0.0) or 0.0),
                    "lane_offset_m": (float(r["laneOffset"])
                                      if r.get("laneOffset") is not None else None),
                    "shoulder_left_m": float(r.get("shoulderSpaceLeft", 0.0) or 0.0),
                    "shoulder_right_m": float(r.get("shoulderSpaceRight", 0.0) or 0.0),
                }
            logging.info("road_network: %d road-looks classified.", len(self.road_looks))
        except Exception as e:
            logging.debug("road_network: road-looks load failed (%s).", e)

    def road_type_at(self, pos):
        """Return the road classification at ``pos``: a dict
        ``{"type": "motorway"|"expressway"|"local"|"dirt", "lanes": int}``, or
        ``None`` if no road is nearby / no look table loaded. The autopilot uses
        this to cap speed on narrow/local sectors."""
        if not self.road_looks or not self.loaded or not pos:
            return None
        # Find the nearest graph segment, then look up its roadLook token.
        # We don't store tokens per segment (heavy), so match by reading the
        # roads.json token for the segment's uid pair if available; fall back to
        # a width-based guess from lane count = unknown.
        seg_idx = self._nearest_segment_index(pos)
        if seg_idx is None:
            return None
        su, eu = self._seg_uids[seg_idx] if seg_idx < len(self._seg_uids) else (None, None)
        tok = self._road_look_token.get(su) or self._road_look_token.get(eu)
        if tok and tok in self.road_looks:
            return self.road_looks[tok]
        # Fallback: guess from the segment length (short = city/local, long = highway).
        (ax, az), (bx, bz) = self.segments[seg_idx]
        seg_len = math.hypot(bx - ax, bz - az)
        if seg_len > 60:
            return {"type": "motorway", "lanes": 2}
        if seg_len > 20:
            return {"type": "expressway", "lanes": 2}
        return {"type": "local", "lanes": 1}

    def _forward_neighbours(self, uid, going_forward):
        """Connected node uids in the travel direction.

        Uses the dense nav-graph when available (forward/backward lists),
        otherwise falls back to the roads.json adjacency. ``going_forward`` picks
        forward vs backward neighbours (a two-way road has both; we follow the
        one matching our travel direction)."""
        if self.fwd:
            return (self.fwd.get(uid, []) if going_forward
                    else self.bwd.get(uid, []))
        return self.adj.get(uid, [])

    def _cell(self, x, z):
        return (int(x // self.GRID), int(z // self.GRID))

    def _add_segment(self, a, b):
        idx = len(self.segments)
        self.segments.append((a, b))
        # Register in every grid cell the endpoints fall into.
        for p in (a, b):
            self._grid.setdefault(self._cell(*p), []).append(idx)

    # --- Lane-level geometry -------------------------------------------------
    @staticmethod
    def _drivable_lane_type(lane_type):
        value = str(lane_type or "").lower()
        return ("road" in value
                and "no_vehicles" not in value
                and ".rail." not in value
                and not value.startswith("traffic_lane.rail"))

    @staticmethod
    def _offset_curve(curve, lateral_m, reverse=False):
        """Offset a sampled road spline and return direction-correct 3-D points."""
        if reverse:
            curve = list(reversed(curve))
            lateral_m = -lateral_m
        result, travelled = [], 0.0
        for index, point in enumerate(curve):
            before = curve[max(0, index - 1)]
            after = curve[min(len(curve) - 1, index + 1)]
            dx, dz = after[0] - before[0], after[1] - before[1]
            length = math.hypot(dx, dz)
            if length < 1e-8:
                ox = oz = 0.0
                heading = result[-1].heading if result else 0.0
            else:
                # Right normal of the direction of travel.
                ox, oz = -dz / length * lateral_m, dx / length * lateral_m
                heading = math.atan2(-dx, -dz)
            if result:
                travelled += math.hypot(
                    point[0] + ox - result[-1].x,
                    point[1] + oz - result[-1].z)
            result.append(LanePoint(point[0] + ox, point[2], point[1] + oz,
                                    travelled, heading))
        return tuple(result)

    def _build_lane_segments(self, segment_index):
        """Lazily derive lane centres for one ordinary road map item.

        The dataset supplies lane order/type but no width.  SCS prefab lane
        centres are spaced 4.5 m apart, so 4.5 m is used as an explicitly
        derived width. ``roadLook.offset`` is the gap between direction groups.
        """
        if segment_index in self._lane_cache:
            return self._lane_cache[segment_index]
        if not (0 <= segment_index < len(self._seg_uids)):
            return ()
        start_uid, end_uid = self._seg_uids[segment_index]
        road_uid = (self._seg_road_uids[segment_index]
                    if segment_index < len(self._seg_road_uids) else 0)
        token = (self._seg_look_tokens[segment_index]
                 if segment_index < len(self._seg_look_tokens) else
                 self._road_look_token.get(start_uid, ""))
        look = self.road_looks.get(token) or {}
        curve = self._road_curve_3d(start_uid, end_uid, spacing=3.0)
        if len(curve) < 2:
            self._lane_cache[segment_index] = ()
            return ()
        width = 4.5
        separation = max(0.0, float(look.get("offset_m", 0.0) or 0.0))
        groups = ((1, tuple(look.get("lane_types_right", ()))),
                  (-1, tuple(look.get("lane_types_left", ()))))
        built = []
        for direction, lane_types in groups:
            drivable = [(raw_index, lane_type)
                        for raw_index, lane_type in enumerate(lane_types)
                        if self._drivable_lane_type(lane_type)]
            for lane_index, (raw_index, lane_type) in enumerate(drivable):
                fixed_side_offset = separation * 0.5 + width * (raw_index + 0.5)
                # Right lanes lie right of start->end. Left lanes lie left and
                # travel end->start; _offset_curve handles the reversal sign.
                lateral = fixed_side_offset if direction > 0 else -fixed_side_offset
                lane_id = LaneId(road_uid, direction, lane_index)
                # Road-look arrays are ordered from the centre outwards.
                # Therefore index-1 is the physical left neighbour and
                # index+1 the physical right neighbour in both directions.
                left = (LaneId(road_uid, direction, lane_index - 1)
                        if lane_index > 0 else None)
                right = (LaneId(road_uid, direction, lane_index + 1)
                         if lane_index + 1 < len(drivable) else None)
                centerline = self._offset_curve(curve, lateral,
                                                reverse=direction < 0)
                mid_y = centerline[len(centerline) // 2].y
                lane = LaneSegment(
                    lane_id=lane_id,
                    start_uid=start_uid if direction > 0 else end_uid,
                    end_uid=end_uid if direction > 0 else start_uid,
                    direction=direction,
                    lane_index=lane_index,
                    lane_count=len(drivable),
                    width_m=width,
                    width_source="derived",
                    elevation_layer=int(round(mid_y / 3.0)),
                    road_look_token=token or None,
                    lane_type=lane_type,
                    centerline=centerline,
                    left_neighbor=left,
                    right_neighbor=right,
                    gps_uids=frozenset((start_uid, end_uid)),
                )
                built.append(lane)
                self._lane_id_index[lane_id] = lane
        result = tuple(built)
        self._lane_cache[segment_index] = result
        return result

    def lane_segments_near(self, pos, radius=28.0):
        """Return lazily built lane centres whose road items are near ``pos``."""
        if not self.loaded or not pos:
            return []
        px, pz = pos
        cx, cz = self._cell(px, pz)
        rings = int(radius // self.GRID) + 1
        seen, result = set(), []
        generous = radius + 18.0
        for dx in range(-rings, rings + 1):
            for dz in range(-rings, rings + 1):
                for index in self._seg_grid.get((cx + dx, cz + dz), ()):
                    if index in seen:
                        continue
                    seen.add(index)
                    (ax, az), (bx, bz) = self.segments[index]
                    vx, vz = bx - ax, bz - az
                    length2 = vx * vx + vz * vz
                    t = (0.0 if length2 < 1e-8 else max(0.0, min(1.0,
                         ((px - ax) * vx + (pz - az) * vz) / length2)))
                    qx, qz = ax + vx * t, az + vz * t
                    if math.hypot(px - qx, pz - qz) <= generous:
                        result.extend(self._build_lane_segments(index))
        return result

    def altitude_near(self, pos):
        index = self._nearest_segment_index(pos, radius=80.0)
        if index is None:
            return None
        first, second = self._seg_uids[index]
        return (self.node_alt.get(first, 0.0)
                + self.node_alt.get(second, 0.0)) * 0.5

    def lanes_connected(self, first, second):
        """Conservative ordinary-road topology check used by hysteresis."""
        if first == second:
            return True
        if (first.road_uid == second.road_uid
                and first.direction == second.direction
                and abs(first.lane_index - second.lane_index) == 1):
            return True
        a = self._lane_id_index.get(first)
        b = self._lane_id_index.get(second)
        if a is None or b is None or a.direction != b.direction:
            return False
        if (any(connection.target == second for connection in a.successors)
                or any(connection.target == first for connection in b.successors)):
            return True
        if a.end_uid != b.start_uid:
            pair = (min(a.end_uid, b.start_uid),
                    max(a.end_uid, b.start_uid))
            for instance in self._prefab_pairs.get(pair, ()):
                options = self._prefab_connector_options(
                    instance, a.end_uid, b.start_uid)
                lane_data = self._prefab_lane_data.get(instance[0]) or {}
                try:
                    start_item = instance[1].index(a.end_uid)
                    end_item = instance[1].index(b.start_uid)
                    inputs = lane_data["nodes"][start_item]["input_lanes"]
                    outputs = lane_data["nodes"][end_item]["output_lanes"]
                except (ValueError, IndexError, KeyError):
                    continue
                for option in options:
                    input_ok = (not inputs or
                                inputs[min(a.lane_index, len(inputs)-1)]
                                == option[0])
                    output_ok = (not outputs or
                                 (option[-1] in outputs and
                                  outputs.index(option[-1]) == b.lane_index))
                    if input_ok and output_ok:
                        return True
            return False
        graph = self.fwd if a.direction > 0 else self.bwd
        return (b.end_uid in graph.get(a.end_uid, ())
                or b.start_uid == a.end_uid)

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
                cell = (cx0 + dx, cz0 + dz)
                indices = list(self._seg_grid.get(cell, ()))
                indices.extend(self._grid.get(cell, ()))
                for idx in indices:
                    if idx in seen:
                        continue
                    seen.add(idx)
                    a, b = self.segments[idx]
                    if (a[0] - px) ** 2 + (a[1] - pz) ** 2 <= r2 or \
                       (b[0] - px) ** 2 + (b[1] - pz) ** 2 <= r2:
                        out.append((a, b))
        return out

    def visual_segments_near(self, pos, radius: float = 800.0, limit: int = 12000):
        """Curved roads and true prefab geometry for the live map."""
        return [((a[0], a[1]), (b[0], b[1]))
                for a, b, _kind, _lanes, _divided, _dash_on, _pillar, _rail_post
                in self.hud_segments_3d_near(pos, radius, limit)]

    def hud_segments_near(self, pos, radius: float = 170.0, limit: int = 320):
        """Return bounded nearby road geometry for the perspective HUD."""
        px, pz = pos
        ranked = []
        for a, b in self.visual_segments_near(pos, radius, limit=max(limit * 3, 960)):
            ax, az = a
            bx, bz = b
            dx, dz = bx - ax, bz - az
            length2 = dx * dx + dz * dz
            t = 0.0 if length2 < 1e-9 else max(
                0.0, min(1.0, ((px - ax) * dx + (pz - az) * dz) / length2))
            qx, qz = ax + t * dx, az + t * dz
            distance2 = (px - qx) ** 2 + (pz - qz) ** 2
            if distance2 <= radius * radius:
                ranked.append((distance2, a, b))
        ranked.sort(key=lambda item: item[0])
        return [(a, b) for _, a, b in ranked[:limit]]

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

    def _nearest_node(self, pos, max_ring=6):
        """uid of the node closest to ``pos`` (via the node grid).

        Expands the search ring by ring up to ``max_ring`` cells (~max_ring*GRID
        metres) so a truck that's a few hundred metres off any node is still
        localized. Returns the uid or None if nothing is in range at all.
        """
        px, pz = pos
        cx0, cz0 = self._cell(px, pz)
        best, best_d = None, float("inf")
        for r in range(max_ring + 1):  # expand search rings
            ring_best_d = best_d
            for dx in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    if max(abs(dx), abs(dz)) != r and r > 0:
                        continue  # only the outer ring (avoids re-scanning inner)
                    for uid in self._ngrid.get((cx0 + dx, cz0 + dz), ()):
                        x, z = self.nodes[uid]
                        d = (x - px) ** 2 + (z - pz) ** 2
                        if d < best_d:
                            best_d, best = d, uid
            if best is not None:
                # Once we've found candidates in a ring, we can stop — inner
                # rings are always closer, so the first ring with hits wins.
                break
        return best

    def _locate_on_road(self, pos, heading):
        """Snap ``pos`` to the road graph and seed the forward walk.

        Returns ``(start_point, start_uid, dirx, dirz)`` where start_point is the
        snapped position to begin the path from, start_uid the graph node we walk
        from, and (dirx, dirz) the unit travel direction. Works while driving
        between nodes (snaps to the nearest graph segment), so the truck is
        always localized on an actual road it can be steered along.
        """
        seg_idx = self._nearest_segment_index(pos)
        if seg_idx is not None:
            (ax, az), (bx, bz) = self.segments[seg_idx]
            su, eu = self._seg_uids[seg_idx]
            dx, dz = bx - ax, bz - az
            L2 = dx * dx + dz * dz
            if L2 > 1e-9:
                t = max(0.0, min(1.0, ((pos[0] - ax) * dx + (pos[1] - az) * dz) / L2))
                sx, sz = ax + t * dx, az + t * dz
            else:
                sx, sz = ax, az
            # Travel direction along this segment, oriented to the truck heading.
            seg_L = math.hypot(dx, dz) or 1.0
            sdx, sdz = dx / seg_L, dz / seg_L
            fwdx, fwdz = -math.sin(heading), -math.cos(heading)
            forward = (sdx * fwdx + sdz * fwdz >= 0)
            if not forward:
                sdx, sdz = -sdx, -sdz
            # Walk from the graph node at the forward end of this segment — that
            # node is guaranteed to be in `adj` (the segment came from roads.json).
            start_uid = eu if forward else su
            return (sx, sz), start_uid, sdx, sdz

        # Fallback: nearest node, heading as-is.
        cur_uid = self._nearest_node(pos)
        if cur_uid is None:
            return None, None, -math.sin(heading), -math.cos(heading)
        return tuple(self.nodes[cur_uid]), cur_uid, -math.sin(heading), -math.cos(heading)

    def _nearest_segment_index(self, pos, radius=400.0):
        """Index into self.segments of the closest segment to ``pos``.

        Uses the endpoint grid for a fast first filter, then the exact
        point-to-segment distance to pick the true nearest. ``None`` if no road
        is within ``radius`` metres.
        """
        if not self.loaded or not pos:
            return None
        px, pz = pos
        cx0, cz0 = self._cell(px, pz)
        seen = set()
        cands = []
        rings = int(radius // self.GRID) + 1
        r2 = radius * radius
        for dx in range(-rings, rings + 1):
            for dz in range(-rings, rings + 1):
                for idx in self._seg_grid.get((cx0 + dx, cz0 + dz), ()):
                    if idx in seen:
                        continue
                    seen.add(idx)
                    a, b = self.segments[idx]
                    # quick endpoint reject
                    if (a[0]-px)**2 + (a[1]-pz)**2 <= r2 or (b[0]-px)**2 + (b[1]-pz)**2 <= r2:
                        cands.append(idx)
        if not cands:
            return None
        best_i, best_d = None, float("inf")
        for idx in cands:
            (ax, az), (bx, bz) = self.segments[idx]
            sdx, sdz = bx - ax, bz - az
            L2 = sdx * sdx + sdz * sdz
            if L2 < 1e-9:
                d = (ax - px) ** 2 + (az - pz) ** 2
            else:
                t = max(0.0, min(1.0, ((px - ax) * sdx + (pz - az) * sdz) / L2))
                qx, qz = ax + t * sdx, az + t * sdz
                d = (qx - px) ** 2 + (qz - pz) ** 2
            if d < best_d:
                best_d, best_i = d, idx
        return best_i

    def path_ahead(self, pos, heading, length=260.0, max_steps=80):
        """
        Follow the road graph forward from ``pos`` in the travel direction,
        returning a polyline ``[(x, z), ...]`` of the road ahead starting AT the
        truck's snapped position.

        The truck heading is the authority for "forward": at each node we pick the
        neighbour whose direction best matches where the truck is actually heading
        (not the segment direction, which can point the wrong way on two-way
        roads). We seed the walk from a node snapped onto the nearest road
        segment, and if that node is a dead end we fall back to the nearest node
        that actually has a forward continuation.
        """
        if not self.loaded or not pos:
            return []
        fwdx, fwdz = -math.sin(heading), -math.cos(heading)
        start, cur, _sdx, _sdz = self._locate_on_road(pos, heading)

        def neighbours(uid, going_forward):
            # With the dense nav-graph, forward/backward lists are directional
            # relative to the road's stored orientation — NOT the truck's travel
            # direction. The caller decides which list to read based on which way
            # the truck is actually going. With the roads.json fallback, every
            # neighbour is a candidate and the dot test does all the work.
            return self._forward_neighbours(uid, going_forward)

        def travel_direction_at(uid, tx, tz):
            """At a given node, which nav-graph list (forward/backward) best
            matches the current travel direction (tx,tz)? Road orientation in
            the data is unrelated to our driving direction, so we must re-pick
            the list at EVERY node — deciding it only once at the seed was the
            reason the walk died on two-way roads (orientation flips between
            segments)."""
            cx, cz = self.nodes[uid]
            f = b = -2.0
            for nb in self._forward_neighbours(uid, True):
                if nb in self.nodes:
                    nx, nz = self.nodes[nb]
                    L = math.hypot(nx - cx, nz - cz) or 1.0
                    f = max(f, ((nx - cx) * tx + (nz - cz) * tz) / L)
            for nb in self._forward_neighbours(uid, False):
                if nb in self.nodes:
                    nx, nz = self.nodes[nb]
                    L = math.hypot(nx - cx, nz - cz) or 1.0
                    b = max(b, ((nx - cx) * tx + (nz - cz) * tz) / L)
            return f >= b

        def walk_from(seed_uid, start_pt):
            path = [start_pt]
            visited = {seed_uid}
            total = 0.0
            first = True
            tx, tz = fwdx, fwdz
            c = seed_uid
            # Travel direction starts from the truck heading, then updates to
            # follow the last segment we drove along — so the path keeps tracing
            # a curving road instead of dying the moment it bends away from the
            # original heading.
            while total < length and len(path) < max_steps:
                cx, cz = self.nodes[c]
                # Re-decide forward vs backward at THIS node from the current
                # travel direction (road orientation can flip between segments).
                going_forward = travel_direction_at(c, tx, tz)
                # First step lenient (a new road may leave the node at a wide
                # angle relative to the heading); later steps want continuity.
                best, best_dot = None, (-0.30 if first else 0.0)
                for nb in neighbours(c, going_forward):
                    if nb in visited or nb not in self.nodes:
                        continue
                    nx, nz = self.nodes[nb]
                    vx, vz = nx - cx, nz - cz
                    L = math.hypot(vx, vz)
                    if L < 1e-3:
                        continue
                    dot = (vx * tx + vz * tz) / L
                    if dot > best_dot:
                        best_dot, best = dot, nb
                if best is None:
                    break
                nx, nz = self.nodes[best]
                seg = math.hypot(nx - cx, nz - cz)
                # Update travel direction to this segment so curves keep tracing.
                tx, tz = (nx - cx) / seg, (nz - cz) / seg
                path.append((nx, nz))
                total += seg
                visited.add(best)
                c = best
                first = False
            return path

        # Try the snapped node first; if it's a dead end for our heading, fall
        # back to the nearest node that has a forward-ish neighbour.
        path = walk_from(cur, start) if cur is not None else []
        if len(path) < 2:
            alt = self._nearest_forward_node(pos, fwdx, fwdz)
            if alt is not None and alt != cur:
                path = walk_from(alt, self.nodes[alt])
        return _smooth(path) if len(path) >= 2 else path

    def _nearest_forward_node(self, pos, fwdx, fwdz, max_ring=8):
        """Nearest node (by ring search) that has a neighbour roughly ahead.

        Used as a recovery when the segment-snapped node is a dead end for our
        heading — we widen the search until we find a node we can actually walk
        forward from.
        """
        px, pz = pos
        cx0, cz0 = self._cell(px, pz)
        best, best_d = None, float("inf")
        for r in range(max_ring + 1):
            ring_hit = False
            for dx in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    if max(abs(dx), abs(dz)) != r and r > 0:
                        continue
                    for uid in self._ngrid.get((cx0 + dx, cz0 + dz), ()):
                        x, z = self.nodes[uid]
                        d = (x - px) ** 2 + (z - pz) ** 2
                        if d >= best_d:
                            continue
                        # Must have at least one forward-ish neighbour (try both
                        # directions of the nav-graph since we don't know which
                        # side we approached from).
                        cands = (self._forward_neighbours(uid, True) +
                                 self._forward_neighbours(uid, False))
                        has_fwd = False
                        for nb in cands:
                            if nb not in self.nodes:
                                continue
                            nx, nz = self.nodes[nb]
                            vx, vz = nx - x, nz - z
                            L = math.hypot(vx, vz)
                            if L > 1e-3 and (vx * fwdx + vz * fwdz) / L > -0.2:
                                has_fwd = True
                                break
                        if has_fwd:
                            best_d, best, ring_hit = d, uid, True
            if ring_hit:
                break
        return best
