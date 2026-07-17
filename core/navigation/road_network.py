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

CACHE_VERSION = 5  # exact road curves + elevation-aware HUD geometry


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
        self._grid = {}          # (cx,cz) -> [segment_index, ...]  (legacy, endpoint-based)
        self._seg_grid = {}      # (cx,cz) -> [segment_index, ...]  (both endpoints)
        self.road_looks = {}     # token -> type, lane counts and direction split
        self._road_look_token = {}  # node_uid -> roadLookToken (nearest road's type)
        self._road_length = {}   # directed endpoint pair -> spline tangent length
        self._prefab_desc = {}   # token -> compact detailed prefab description
        self._prefab_grid = {}   # spatial index of placed prefab instances
        self._prefab_pairs = {}  # unordered endpoint UID pair -> prefab instances
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
                      "_seg_uids", "_grid", "_seg_grid", "_road_look_token",
                      "_road_length", "road_looks", "_prefab_desc", "_prefab_grid",
                      "_prefab_pairs", "loaded"):
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
                    "_seg_uids": self._seg_uids, "_grid": self._grid,
                    "_seg_grid": self._seg_grid, "_road_look_token": self._road_look_token,
                    "_road_length": self._road_length,
                    "road_looks": self.road_looks,
                    "_prefab_desc": self._prefab_desc,
                    "_prefab_grid": self._prefab_grid,
                    "_prefab_pairs": self._prefab_pairs,
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
            ranked.append((distance2, (a[0], a[1], ah),
                           (b[0], b[1], bh), "lane", 1, False,
                           (prefab_index % 5) < 3,
                           False, False))
        ranked.sort(key=lambda item: item[0])
        return [(a, b, kind, lanes, divided, dash_on, pillar, rail_post)
                for _, a, b, kind, lanes, divided, dash_on, pillar, rail_post
                in ranked[:limit]]

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
                            result.append(self.nodes[bridge_b])
                else:
                    result.append(target)
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
        cache[key] = []
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
