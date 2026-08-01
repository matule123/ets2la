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
from core.navigation.route_diagnostics import (
    lane_id_payload, safe_diagnostic_call,
)
from core.navigation.road_look_offsets_159 import LANE_OFFSETS_159

CACHE_VERSION = 11  # adds Truckermudgeon prefab map polygons for the live map


def _uid(value):
    """Canonical ETS2 UID. JSON stores hexadecimal strings; SDK sends int64."""
    if isinstance(value, int):
        return value - (1 << 64) if value >= (1 << 63) else value
    if value in (None, "", "0"):
        return 0
    number = int(str(value), 16)
    return number - (1 << 64) if number >= (1 << 63) else number


def _rotate_right(values, count):
    """Match TruckLib's descriptor-node ordering for placed prefabs."""
    values = tuple(values)
    if not values:
        return values
    count %= len(values)
    if count == 0:
        return values
    return values[-count:] + values[:-count]


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
        self.node_forward_item = {}   # node uid -> exact SCS map item after node
        self.node_backward_item = {}  # node uid -> exact SCS map item before node
        self.adj = {}            # uid -> [connected uid, ...]  (road graph, from roads.json)
        self.fwd = {}            # uid -> [uid, ...]  forward neighbours (graph.json)
        self.bwd = {}            # uid -> [uid, ...]  backward neighbours (graph.json)
        self._ngrid = {}         # (cx,cz) -> [uid, ...]  (node spatial index)
        self.segments = []       # [((x1,z1),(x2,z2)), ...]
        self._seg_uids = []      # [(start_uid, end_uid), ...]  parallel to segments
        self._seg_road_uids = [] # road item uid parallel to segments
        self._seg_look_tokens = []  # exact road-look token parallel to segments
        self._road_segment_by_uid = {}  # road map-item uid -> segment index
        self._grid = {}          # (cx,cz) -> [segment_index, ...]  (legacy, endpoint-based)
        self._seg_grid = {}      # (cx,cz) -> [segment_index, ...]  (both endpoints)
        self.road_looks = {}     # token -> type, lane counts and direction split
        self._road_look_token = {}  # node_uid -> roadLookToken (nearest road's type)
        self._road_length = {}   # directed endpoint pair -> spline tangent length
        self._prefab_desc = {}   # token -> compact detailed prefab description
        self._prefab_grid = {}   # spatial index of placed prefab instances
        self._prefab_pairs = {}  # unordered endpoint UID pair -> prefab instances
        self._prefab_lane_data = {}  # token -> dataset lane/curve connectivity
        self._prefab_map_polygons = {}  # token -> Truckermudgeon map polygons
        # Display-only map landmarks.  They are deliberately not part of the
        # route cache, so stale UI data cannot influence lane localisation.
        self._map_feature_grid = {}  # (cx,cz) -> compact feature tuples
        self._map_feature_count = 0
        self._lane_cache = {}    # segment index -> tuple[LaneSegment, ...]
        self._lane_id_index = {} # LaneId -> LaneSegment (populated lazily)
        self._lane_path_revision = 0
        self.loaded = False

    @staticmethod
    def _display_count(value):
        return f"{int(value):,}".replace(",", " ")

    def load_statistics(self):
        """Return read-only counts for the loading UI and diagnostics."""
        placed_prefabs = {
            (instance[0], tuple(instance[1]))
            for instances in self._prefab_grid.values()
            for instance in instances
        }
        return {
            "nodes": len(self.nodes),
            "roads": len(self.segments),
            "prefabs": len(placed_prefabs),
        }

    def _loaded_counts_text(self):
        stats = self.load_statistics()
        return (f"Uzly {self._display_count(stats['nodes'])}  •  "
                f"cesty {self._display_count(stats['roads'])}  •  "
                f"prefaby {self._display_count(stats['prefabs'])}")

    # --- Loading --------------------------------------------------------------
    def load(self, data_dir: str, progress_cb=None) -> bool:
        """Load one dataset and optionally report coarse, read-only progress.

        ``progress_cb`` receives ``(fraction, phase)`` only at phase
        boundaries. Callback failures are observational and cannot alter the
        loader result.
        """
        def report(fraction, phase):
            if progress_cb is None:
                return
            try:
                progress_cb(float(fraction), str(phase))
            except Exception:
                logging.debug("road_network: progress callback failed",
                              exc_info=True)

        # Fast path: a pickled cache of the parsed network, keyed on the mtimes
        # of the source JSON files. Loading 1.1M nodes from JSON takes ~5-7s;
        # unpickling the ready object takes ~1s. Rebuilds automatically when the
        # dataset is updated or its version changes.
        report(0.02, "Načítavam cesty a prefaby z cache")
        if self._try_load_cache(data_dir, progress_cb=report):
            report(0.90, self._loaded_counts_text())
            report(0.92, "Načítavam mestá, firmy a služby")
            self._load_map_features(data_dir)
            report(1.0, f"Mapa je pripravená  •  {self._loaded_counts_text()}")
            return True

        nodes_path = _find_json(data_dir, "nodes")
        roads_path = _find_json(data_dir, "roads")
        if not nodes_path or not roads_path:
            logging.error("road_network: nodes/roads json not found in %s", data_dir)
            report(1.0, "Chýbajú mapové dáta")
            return False

        try:
            report(0.10, "Načítavam uzly")
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
                self.node_forward_item[uid] = _uid(n.get("forwardItemUid"))
                self.node_backward_item[uid] = _uid(n.get("backwardItemUid"))
                self._ngrid.setdefault(self._cell(x, y), []).append(uid)
            report(0.38, f"Uzly: {self._display_count(len(self.nodes))} načítaných")
        except Exception as e:
            logging.exception("road_network: failed to load nodes: %s", e)
            return False

        try:
            raw_roads = raw_roads  # noqa
        except Exception:
            pass
        try:
            report(0.42, "Načítavam cesty")
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
                    self._road_segment_by_uid[self._seg_road_uids[-1]] = len(self.segments)
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
            report(0.64, f"Cesty: {self._display_count(len(self.segments))} načítaných")
        except Exception as e:
            logging.exception("road_network: failed to load roads: %s", e)
            return False

        self.loaded = True
        # Prefer the dense navigation graph (graph.json) when available — it's
        # the same graph ETS2LA's Map plugin pathfinds on, so it's far more
        # complete than the roads-only adjacency. Falls back silently to `adj`.
        report(0.69, "Načítavam GPS konektivitu")
        self._load_nav_graph(data_dir)
        report(0.77, "Načítavam prefaby a križovatky")
        self._load_prefabs(data_dir)
        report(0.84, ("Prefaby: "
                      f"{self._display_count(self.load_statistics()['prefabs'])} "
                      "načítaných"))
        # Road-look table: classifies each road segment (motorway / expressway /
        # local / dirt) + lane count, used by the autopilot to slow down on
        # narrow/local roads and cap speed in city sectors.
        report(0.88, "Načítavam typy ciest a pruhy")
        self._load_road_looks(data_dir)
        report(0.92, "Načítavam mestá, firmy a služby")
        self._load_map_features(data_dir)
        logging.info("road_network: loaded %d nodes, %d segments, nav-graph nodes=%d",
                     len(self.nodes), len(self.segments),
                     len(self.fwd) if self.fwd else len(self.adj))
        # Persist the parsed network so the next launch is fast (~1s vs ~6s).
        report(0.95, "Ukladám zrýchľovaciu cache")
        self._save_cache(data_dir)
        report(1.0, f"Mapa je pripravená  •  {self._loaded_counts_text()}")
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

    def _try_load_cache(self, data_dir: str, progress_cb=None) -> bool:
        """Load the pickled network if it matches the current sources."""
        path = self._cache_path(data_dir)
        if not os.path.exists(path):
            return False
        try:
            import pickle
            try:
                total_bytes = max(0, int(os.path.getsize(path)))
            except OSError:
                total_bytes = 0
            last_fraction = 0.0
            last_report_at = 0.0

            def report_cache(position):
                nonlocal last_fraction, last_report_at
                if progress_cb is None or total_bytes <= 0:
                    return
                relative = max(0.0, min(1.0,
                    float(position) / float(total_bytes)))
                fraction = 0.02 + relative * 0.86
                now = time.monotonic()
                if (fraction < 0.88
                        and fraction-last_fraction < 0.03
                        and now-last_report_at < 0.40):
                    return
                if relative < 0.35:
                    phase = "Načítavam uzly a priestorové indexy z cache"
                elif relative < 0.58:
                    phase = "Načítavam cesty a pruhy z cache"
                elif relative < 0.75:
                    phase = "Načítavam GPS konektivitu z cache"
                else:
                    phase = "Načítavam prefaby a križovatky z cache"
                try:
                    progress_cb(fraction, phase)
                except Exception:
                    logging.debug("road_network: cache progress callback failed",
                                  exc_info=True)
                last_fraction = fraction
                last_report_at = now

            class ProgressReader:
                def __init__(self, stream):
                    self.stream = stream

                def read(self, size=-1):
                    value = self.stream.read(size)
                    report_cache(self.stream.tell())
                    return value

                def readline(self, size=-1):
                    value = self.stream.readline(size)
                    report_cache(self.stream.tell())
                    return value

                def readinto(self, buffer):
                    count = self.stream.readinto(buffer)
                    report_cache(self.stream.tell())
                    return count

                def __getattr__(self, name):
                    return getattr(self.stream, name)

            with open(path, "rb") as f:
                reader = ProgressReader(f) if total_bytes > 0 else f
                payload = pickle.load(reader)
                report_cache(total_bytes)
            # Invalidate if the source files changed since the cache was built.
            if (payload.get("version") != CACHE_VERSION
                    or payload.get("sig") != self._source_signature(data_dir)):
                logging.info("road_network: cache stale — rebuilding.")
                return False
            data = payload["data"]
            for k in ("nodes", "node_rot", "node_alt", "node_forward",
                      "node_forward_item", "node_backward_item",
                      "adj", "fwd", "bwd", "_ngrid", "segments",
                      "_seg_uids", "_seg_road_uids", "_seg_look_tokens",
                      "_road_segment_by_uid",
                      "_grid", "_seg_grid", "_road_look_token",
                      "_road_length", "road_looks", "_prefab_desc", "_prefab_grid",
                      "_prefab_pairs", "_prefab_lane_data",
                      "_prefab_map_polygons", "loaded"):
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
                    "node_forward_item": self.node_forward_item,
                    "node_backward_item": self.node_backward_item,
                    "adj": self.adj, "fwd": self.fwd,
                    "bwd": self.bwd, "_ngrid": self._ngrid, "segments": self.segments,
                    "_seg_uids": self._seg_uids,
                    "_seg_road_uids": self._seg_road_uids,
                    "_seg_look_tokens": self._seg_look_tokens,
                    "_road_segment_by_uid": self._road_segment_by_uid,
                    "_grid": self._grid,
                    "_seg_grid": self._seg_grid, "_road_look_token": self._road_look_token,
                    "_road_length": self._road_length,
                    "road_looks": self.road_looks,
                    "_prefab_desc": self._prefab_desc,
                    "_prefab_grid": self._prefab_grid,
                    "_prefab_pairs": self._prefab_pairs,
                    "_prefab_lane_data": self._prefab_lane_data,
                    "_prefab_map_polygons": self._prefab_map_polygons,
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
                # Port of truckermudgeon/maps
                # packages/libs/map/prefabs.ts::toRoadStringsAndPolygons.
                # Map points are already authored as ordered neighbour loops;
                # following that topology preserves real prefab islands,
                # depots and junction surfaces without inventing geometry.
                map_points = tuple(raw.get("mapPoints", ()) or ())
                polygon_indices = {
                    index for index, point in enumerate(map_points)
                    if isinstance(point, dict) and point.get("type") == "polygon"
                }
                visited_polygon_indices = set()
                polygons = []
                colour_z = {0: 3, 1: 0, 2: 2, 3: 1,
                            4: 99, 5: 98, 6: 97, 7: 96, 8: 95}
                for start_index in sorted(polygon_indices):
                    if start_index in visited_polygon_indices:
                        continue
                    ordered = []
                    current = start_index
                    while current not in visited_polygon_indices:
                        if current not in polygon_indices:
                            ordered = []
                            break
                        point = map_points[current]
                        ordered.append((float(point.get("x", 0.0) or 0.0),
                                        float(point.get("y", 0.0) or 0.0)))
                        visited_polygon_indices.add(current)
                        neighbours = [
                            int(value) for value in point.get("neighbors", ())
                            if int(value) in polygon_indices
                        ]
                        unvisited = [value for value in neighbours
                                     if value not in visited_polygon_indices]
                        if unvisited:
                            current = unvisited[0]
                        elif start_index in neighbours:
                            current = start_index
                            break
                        else:
                            ordered = []
                            break
                    if len(ordered) >= 3 and current == start_index:
                        first = map_points[start_index]
                        colour = int(first.get("color", 0) or 0)
                        z_index = ((10 if first.get("roadOver") else 0)
                                   + colour_z.get(colour, 0))
                        polygons.append((tuple(ordered), colour, z_index))
                self._prefab_desc[str(raw.get("token", ""))] = (
                    nodes, tuple(curves), tuple(nav_nodes))
                self._prefab_map_polygons[str(raw.get("token", ""))] = tuple(
                    sorted(polygons, key=lambda item: item[2]))
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
                raw_uids = tuple(_uid(value) for value in raw.get("nodeUids", ())
                                 if _uid(value))
                if not raw_uids:
                    continue
                origin_index = int(raw.get("originNodeIndex", 0))
                if not (0 <= origin_index < len(raw_uids)):
                    raise ValueError(
                        f"invalid originNodeIndex for prefab {token}")
                # TruckLib's placed nodeUids are in map-item order. PPD
                # inputLanes, outputLanes and navNodes are in descriptor order,
                # which is rotateRight(raw_uids, originNodeIndex). New datasets
                # publish that companion array explicitly; legacy datasets do
                # not, but the same deterministic ordering contract still
                # applies. Never infer it from proximity or connector geometry.
                expected_uids = _rotate_right(raw_uids, origin_index)
                descriptor_values = raw.get("descriptorNodeUids")
                if descriptor_values is None:
                    uids = expected_uids
                else:
                    uids = tuple(_uid(value) for value in descriptor_values
                                 if _uid(value))
                    if uids != expected_uids:
                        raise ValueError(
                            f"invalid descriptorNodeUids for prefab {token}")
                instance = (token, uids, origin_index, True)
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
            self._prefab_map_polygons.clear()

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
        token, uids, origin_index = instance[:3]
        descriptor_order = bool(instance[3]) if len(instance) > 3 else False
        desc = self._prefab_desc.get(token)
        anchor_index = origin_index if descriptor_order else 0
        anchor = (self.nodes.get(uids[anchor_index])
                  if uids and 0 <= anchor_index < len(uids) else None)
        if not desc or anchor is None or not desc[0]:
            return []
        origin_index = max(0, min(origin_index, len(desc[0]) - 1))
        descriptor_anchor = origin_index if descriptor_order else 0
        ox, oz, local_rot = desc[0][descriptor_anchor]
        rotation_uid = uids[anchor_index] if descriptor_order else uids[0]
        rotation = self.node_rot.get(rotation_uid, 0.0) - local_rot
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

    def prefab_segments_3d_near(self, pos, radius=800.0, limit=10000,
                                allowed_node_uids=None,
                                include_path_metadata=False):
        """Return prefab navCurve chords with their real local elevations.

        A nearest-node X/Z lookup is not a valid source of prefab height: at
        an overpass it can select a node from the other deck and turn a flat
        junction lane into a near-vertical HUD ribbon.  The PPD navCurve
        stores start/end Y and ``_prefab_curve_chain_3d`` applies the same
        placed-prefab transform as lane navigation, so use that geometry
        directly for the display mesh as well.
        """
        if not pos or not self._prefab_grid:
            return []
        px, pz = pos
        cx, cz = self._cell(px, pz)
        rings = int(radius // self.GRID) + 1
        seen, result = set(), []
        display_instance_index = 0
        for dx in range(-rings, rings + 1):
            for dz in range(-rings, rings + 1):
                for instance in self._prefab_grid.get((cx + dx, cz + dz), ()):
                    marker = (instance[0], instance[1])
                    if marker in seen:
                        continue
                    seen.add(marker)
                    if (allowed_node_uids is not None
                            and not any(uid in allowed_node_uids
                                        for uid in instance[1])):
                        continue
                    desc = self._prefab_desc.get(instance[0])
                    if not desc:
                        continue
                    instance_path_index = display_instance_index
                    display_instance_index += 1
                    for curve_index in range(len(desc[1])):
                        try:
                            points = self._prefab_curve_chain_3d(
                                instance, (curve_index,))
                        except (IndexError, KeyError, TypeError, ValueError):
                            continue
                        path_key = f"p{instance_path_index}:{curve_index}"
                        for segment_index, (first, second) in enumerate(
                                zip(points, points[1:])):
                            a = (first.x, first.z, first.y)
                            b = (second.x, second.z, second.y)
                            if min((a[0]-px)**2 + (a[1]-pz)**2,
                                   (b[0]-px)**2 + (b[1]-pz)**2) > radius*radius:
                                continue
                            if include_path_metadata:
                                result.append((a, b, path_key,
                                               segment_index))
                            else:
                                result.append((a, b))
                            if len(result) >= limit:
                                return result
        return result

    @staticmethod
    def _hud_chord_is_sane(a, b, altitude=None, distance2=float("inf")):
        """Reject malformed display chords without changing map topology."""
        if not all(math.isfinite(float(value)) for value in (*a, *b)):
            return False
        horizontal = math.hypot(b[0] - a[0], b[1] - a[1])
        if horizontal < 0.05:
            return False
        # Sampled map chords are roughly 2.5 m long. Even a very steep ramp
        # cannot legitimately gain several metres in one sample; that is a
        # wrong-deck assignment and was rendered as a road into the sky.
        if abs(b[2] - a[2]) > max(1.5, horizontal * 0.35):
            return False
        if altitude is not None and distance2 < 90.0 ** 2:
            if min(abs(a[2] - altitude), abs(b[2] - altitude)) > 3.2:
                return False
        return True

    def _road_curve_3d(self, first, second, spacing=2.5,
                       with_tangents=False):
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
            if with_tangents:
                dh00, dh10 = 6*t2-6*t, 3*t2-4*t+1
                dh01, dh11 = -6*t2+6*t, 3*t2-2*t
                tx = (dh00*sx + dh10*sdx*tangent_length
                      + dh01*ex + dh11*edx*tangent_length)
                tz = (dh00*sz + dh10*sdz*tangent_length
                      + dh01*ez + dh11*edz*tangent_length)
                points.append((x, z, height, tx, tz))
            else:
                points.append((x, z, height))
        if reverse:
            points.reverse()
            if with_tangents:
                points = [(x, z, y, -tx, -tz)
                          for x, z, y, tx, tz in points]
        return points

    def hud_segments_3d_near(self, pos, radius: float = 280.0, limit: int = 950,
                             altitude=None, connected_only: bool = True,
                             anchor_lane_id=None):
        """Curved road segments with elevation for the perspective HUD."""
        if not self.loaded or not pos:
            return []
        px, pz = pos
        cx, cz = self._cell(px, pz)
        rings = int(radius // self.GRID) + 1
        candidate_indices = set()
        for dx in range(-rings, rings + 1):
            for dz in range(-rings, rings + 1):
                candidate_indices.update(
                    self._seg_grid.get((cx+dx, cz+dz), ()))

        # The old HUD painted every road inside a 280 m circle. Parallel
        # streets, depot lanes and overpasses then floated as unrelated islands
        # beside the driving view. Keep the topological component containing
        # the truck; junction arms remain because placed prefab endpoints link
        # their real road objects, while an unrelated nearby road cannot enter.
        connected_indices = set(candidate_indices)
        prefab_links = {}
        prefab_seen = set()
        for dx in range(-rings, rings + 1):
            for dz in range(-rings, rings + 1):
                for instance in self._prefab_grid.get((cx+dx, cz+dz), ()):
                    marker = (instance[0], instance[1])
                    if marker in prefab_seen:
                        continue
                    prefab_seen.add(marker)
                    endpoint_set = set(instance[1])
                    for uid in endpoint_set:
                        prefab_links.setdefault(uid, set()).update(endpoint_set)
        # Anchor the component to the live, revision-matched LaneId whenever
        # it is an ordinary road lane. Pure 2-D nearest-road selection can
        # choose an unconnected parallel road or the deck below an overpass;
        # the real carriageway then disappears around/behind the truck in the
        # HUD. Prefab LaneIds intentionally fall back to proximity because
        # their ``road_uid`` identifies a placed prefab, not a road item.
        anchor_road_uid = getattr(anchor_lane_id, "road_uid", None)
        start_index = self._road_segment_by_uid.get(anchor_road_uid)
        if start_index not in candidate_indices:
            start_index = self._nearest_segment_index(pos)
        if connected_only and start_index in candidate_indices:
            node_roads = {}
            for index in candidate_indices:
                for uid in self._seg_uids[index]:
                    node_roads.setdefault(uid, set()).add(index)
            connected_indices, queue = {start_index}, [start_index]
            while queue:
                index = queue.pop()
                linked_nodes = set(self._seg_uids[index])
                for uid in tuple(linked_nodes):
                    linked_nodes.update(prefab_links.get(uid, ()))
                for uid in linked_nodes:
                    for neighbour in node_roads.get(uid, ()):
                        if neighbour not in connected_indices:
                            connected_indices.add(neighbour)
                            queue.append(neighbour)

        ranked = []
        for index in connected_indices:
            first, second = self._seg_uids[index]
            # Use the exact road item's look. Node-level compatibility lookup
            # is ambiguous at junctions and can select a neighbouring arm's
            # lane count/offset, which misplaces the entire HUD carriageway.
            token = (self._seg_look_tokens[index]
                     if index < len(self._seg_look_tokens) else
                     self._road_look_token.get(first)
                     or self._road_look_token.get(second))
            look = self.road_looks.get(token) or {}
            lane_segments = tuple(self._build_lane_segments(index))
            groups = []
            if lane_segments:
                by_direction = {
                    direction: tuple(lane for lane in lane_segments
                                     if lane.direction == direction)
                    for direction in (-1, 1)
                }
                by_direction = {key: value for key, value in by_direction.items()
                                if value}
                # ``roadLook.offset`` separates carriageways around a median.
                # Painting one ribbon around the raw map-item centre ignored
                # that offset (5.75 m on the reported blkw2c road), clipped the
                # truck's carriageway away and made the HUD rig appear outside
                # the road. Keep separated direction groups as real ribbons.
                split = (len(by_direction) == 2
                         and abs(float(look.get("offset_m", 0.0) or 0.0)) > .75)
                groups = (list(by_direction.values()) if split
                          else [lane_segments])

            ribbons = []
            for group in groups:
                oriented = [tuple(lane.centerline
                                  if lane.direction > 0
                                  else reversed(lane.centerline))
                            for lane in group]
                count = min((len(points) for points in oriented), default=0)
                if count < 2:
                    continue
                curve, half_widths = [], []
                shoulder = max(.5, float(look.get("shoulder_left_m", .5) or .5),
                               float(look.get("shoulder_right_m", .5) or .5))
                for point_index in range(count):
                    sample = [points[point_index] for points in oriented]
                    if len(sample) == 1:
                        left = right = sample[0]
                    else:
                        left, right = max(
                            ((a, b) for item_index, a in enumerate(sample)
                             for b in sample[item_index + 1:]),
                            key=lambda pair: math.hypot(
                                pair[1].x-pair[0].x,
                                pair[1].z-pair[0].z))
                    span = math.hypot(right.x-left.x, right.z-left.z)
                    curve.append(((left.x+right.x)*.5,
                                  (left.z+right.z)*.5,
                                  (left.y+right.y)*.5))
                    half_widths.append(span*.5 + 2.25 + shoulder)
                ribbons.append((curve, len(group), not split,
                                tuple(half_widths)))

            if not ribbons:
                lanes = max(1, int(look.get("lanes", 2)))
                curve = self._road_curve_3d(first, second)
                ribbons = [(curve, lanes, bool(
                    (look.get("lanes_left", 0) and look.get("lanes_right", 0))
                    or (lanes >= 4 and look.get("type")
                        in ("motorway", "expressway"))),
                    tuple(lanes * 4.5 / 2.0 + .5 for _ in curve))]

            for ribbon_index, (curve, lanes, divided,
                               half_widths) in enumerate(ribbons):
                for curve_index, (a, b) in enumerate(zip(curve, curve[1:])):
                    distance2 = min((a[0]-px)**2+(a[1]-pz)**2,
                                    (b[0]-px)**2+(b[1]-pz)**2)
                    if distance2 > radius*radius:
                        continue
                    if not self._hud_chord_is_sane(a, b, altitude, distance2):
                        continue
                    # Fixed 7.5 m dash / 5 m gap in world space. Qt's
                    # screen-space DashLine restarted on every sampled curve.
                    dash_on = (curve_index % 5) < 3
                    pillar = (curve_index % 12) == 0
                    rail_post = (curve_index % 4) == 0
                    half_width = ((half_widths[curve_index]
                                   + half_widths[curve_index + 1]) * .5)
                    near_prefab_boundary = (
                        (first in prefab_links and curve_index < 5)
                        or (second in prefab_links
                            and curve_index >= len(curve) - 6))
                    ranked.append((distance2, a, b, "road",
                                   max(1, lanes), divided, dash_on,
                                   pillar, rail_post, half_width,
                                   near_prefab_boundary,
                                   f"r{index}:{ribbon_index}", curve_index))
        # Road items stop at prefab boundaries. Without any prefab surface the
        # HUD therefore showed a literal black hole where a junction or
        # roundabout should be. PPD navCurves are authoritative lane-centre
        # geometry, although the compact dataset does not expose the original
        # paint material. Publish each sampled curve as a distinct display
        # path so the HUD can draw a smooth, topology-safe lane envelope rather
        # than either hiding the lane or joining unrelated junction arms.
        # Restrict them to prefabs attached to the current road component and
        # preserve their real Y coordinates.
        connected_nodes = ({
            uid for index in connected_indices for uid in self._seg_uids[index]
        } if connected_only else None)
        prefab_limit = max(limit, min(limit * 3, 3000))
        for a, b, path_key, path_index in self.prefab_segments_3d_near(
                pos, radius=radius, limit=prefab_limit,
                allowed_node_uids=connected_nodes,
                include_path_metadata=True):
            distance2 = min((a[0]-px)**2 + (a[1]-pz)**2,
                            (b[0]-px)**2 + (b[1]-pz)**2)
            if not self._hud_chord_is_sane(a, b, altitude, distance2):
                continue
            ranked.append((distance2, a, b, "lane", 1, False, False,
                           False, False, 3.05, True,
                           path_key, path_index))
        ranked.sort(key=lambda item: item[0])
        return [(a, b, kind, lanes, divided, dash_on, pillar, rail_post,
                 half_width, suppress_markings, path_key, path_index)
                for _, a, b, kind, lanes, divided, dash_on, pillar, rail_post,
                half_width, suppress_markings, path_key, path_index
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
        # navNodeIndex did not exist in older PPD revisions. The chain reaching
        # this function was already selected through physical navNodes and is
        # still bounded by inputLanes/outputLanes, so -1 is missing data rather
        # than permission to invent a connector.
        return True

    def _prefab_connector_options(self, instance, start_uid, end_uid):
        token, uids, _origin = instance[:3]
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
                curves = lane_data.get("curves", ())
                # SCS stores the target nav-node identity on the first curve
                # of each navNode connection.  nextLines/prevLines alone can
                # describe a geometrically continuous ring but do not prove
                # that it is the GPS-selected exit.  Reject a damaged or
                # mismatched connector instead of taking the other side of a
                # roundabout.
                if not (0 <= indices[0] < len(curves)):
                    continue
                nav_node_index = int(curves[indices[0]].get(
                    "nav_node_index", -1))
                if nav_node_index != -1:
                    # A PPD can duplicate an AI navNode for different entry
                    # histories around a roundabout. In that case the curve
                    # refers to the canonical duplicate rather than the exact
                    # target index, but both carry the same endIndex. This is
                    # the only valid alias; a different endIndex is another
                    # lane/exit and is rejected.
                    if not (0 <= nav_node_index < len(nav_nodes)):
                        continue
                    if (nav_node_index != target
                            and nav_nodes[nav_node_index][1]
                                != nav_nodes[target][1]):
                        continue
                combined = tuple(curve_indices) + tuple(indices)
                if curve_indices:
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

    def _prefab_parallel_lane_options(self, instance, start_uid, end_uid):
        """Return PPD-proven parallel lanes for one physical GPS connection.

        ``navNodes.connections`` stores a representative AI path. Parallel
        lanes are encoded by inputLanes/outputLanes and reciprocal curve
        nextLines/prevLines. This enumerator is used only when the
        representative path is physically offset from confirmed incoming lane
        geometry, never as a general alternative-route search. Consecutive
        prefabs may continue the same parallel lane only through one exact
        endpoint/heading match on the next GPS-proven physical connection.
        """
        token, uids, _origin = instance[:3]
        try:
            start_item, end_item = (uids.index(start_uid),
                                    uids.index(end_uid))
        except ValueError:
            return []
        lane_data = self._prefab_lane_data.get(token) or {}
        desc = self._prefab_desc.get(token)
        if not desc:
            return []
        nav_nodes = desc[2]
        end_nav = next((index for index, node in enumerate(nav_nodes)
                        if node[0] == "physical" and node[1] == end_item), None)
        if end_nav is None:
            return []
        nodes = lane_data.get("nodes", ())
        curves = tuple(lane_data.get("curves", ()))
        if not (0 <= start_item < len(nodes) and 0 <= end_item < len(nodes)):
            return []
        starts = tuple(int(value) for value in nodes[start_item]["input_lanes"])
        ends = tuple(int(value) for value in nodes[end_item]["output_lanes"])
        all_outputs = {int(value) for node in nodes
                       for value in node.get("output_lanes", ())}
        options = []

        def walk_lane(curve_index, chain, visited):
            if curve_index in ends:
                nav_index = int(curves[curve_index].get(
                    "nav_node_index", -1))
                if (nav_index == -1
                        or (0 <= nav_index < len(nav_nodes)
                            and (nav_index == end_nav
                                 or nav_nodes[nav_index][1]
                                    == nav_nodes[end_nav][1]))):
                    candidate = tuple(chain)
                    if self._curve_chain_is_valid(lane_data, candidate):
                        options.append(candidate)
                return
            for next_index in curves[curve_index].get("next_lines", ()):
                next_index = int(next_index)
                if (next_index in visited
                        or not (0 <= next_index < len(curves))
                        or (next_index in all_outputs
                            and next_index not in ends)
                        or curve_index not in curves[next_index].get(
                            "prev_lines", ())):
                    continue
                walk_lane(next_index, chain + (next_index,),
                          visited | {next_index})

        for start_curve in starts:
            if 0 <= start_curve < len(curves):
                walk_lane(start_curve, (start_curve,), {start_curve})
        return sorted(set(options))

    def _segment_uses_parallel_prefab_sibling(self, segment):
        """Whether ``segment`` is a PPD sibling, not its navNode representative."""
        token = segment.lane_id.prefab_token
        path = tuple(segment.connector_curve_indices or ())
        if token in (None, "graph") or not path:
            return False
        pair = (min(segment.start_uid, segment.end_uid),
                max(segment.start_uid, segment.end_uid))
        for instance in self._prefab_pairs.get(pair, ()):
            if instance[0] != token:
                continue
            representatives = self._prefab_connector_options(
                instance, segment.start_uid, segment.end_uid)
            siblings = self._prefab_parallel_lane_options(
                instance, segment.start_uid, segment.end_uid)
            if path in siblings:
                return path not in representatives
        return False

    def _prefab_curve_chain_3d(self, instance, indices):
        token, uids, origin_index = instance[:3]
        descriptor_order = bool(instance[3]) if len(instance) > 3 else False
        desc = self._prefab_desc[token]
        lane_data = self._prefab_lane_data[token]
        if not uids or not desc[0]:
            return ()
        origin_index = max(0, min(origin_index, len(desc[0]) - 1))
        # Loaded datasets are normalised into descriptor order. The three-item
        # instance fallback remains only for old in-memory callers/tests.
        # Translation and rotation must match _transform_prefab_points().
        anchor_index = origin_index if descriptor_order else 0
        if anchor_index >= len(uids):
            return ()
        origin_uid = uids[anchor_index]
        anchor = self.nodes.get(origin_uid)
        if anchor is None:
            return ()
        descriptor_anchor = origin_index if descriptor_order else 0
        ox, oz, local_rotation = desc[0][descriptor_anchor]
        origin_y = lane_data["nodes"][descriptor_anchor]["y"]
        rotation = self.node_rot.get(origin_uid, 0.0) - local_rotation
        c, s = math.cos(rotation), math.sin(rotation)
        anchor_y = self.node_alt.get(origin_uid, 0.0)
        # A placed prefab may be pitched/rolled even though its horizontal
        # placement only exposes yaw.  Using the origin node's altitude as a
        # constant offset made every navCurve flat and left a vertical step at
        # the other descriptor nodes (1.83 m and 2.21 m in the captured
        # ProMods lane-change failures).  Descriptor-ordered endpoint UIDs are
        # authoritative placement constraints, so recover the missing rigid
        # elevation plane from them.  This changes Y only; it cannot invent a
        # horizontal chord or a connection outside the GPS-proven prefab.
        elevation_plane = None
        if descriptor_order:
            samples = []
            lane_nodes = lane_data.get("nodes") or ()
            for node_index, descriptor_node in enumerate(desc[0]):
                if node_index >= len(uids) or node_index >= len(lane_nodes):
                    continue
                uid = uids[node_index]
                if uid not in self.node_alt:
                    continue
                nx, nz = descriptor_node[:2]
                local_node_y = float(lane_nodes[node_index].get("y", 0.0))
                samples.append((
                    node_index, float(nx), float(nz),
                    float(self.node_alt[uid]) - local_node_y,
                ))
            if samples:
                _base_index, base_x, base_z, base_delta = next(
                    (sample for sample in samples
                     if sample[0] == descriptor_anchor), samples[0])
                vectors = [
                    (x - base_x, z - base_z, delta - base_delta)
                    for node_index, x, z, delta in samples
                    if node_index != descriptor_anchor
                ]
                gradient_x = gradient_z = 0.0
                best_pair = None
                for first_index, first in enumerate(vectors):
                    for second in vectors[first_index + 1:]:
                        determinant = first[0] * second[1] - first[1] * second[0]
                        if (best_pair is None
                                or abs(determinant) > abs(best_pair[0])):
                            best_pair = (determinant, first, second)
                if best_pair is not None and abs(best_pair[0]) > 1e-6:
                    determinant, first, second = best_pair
                    gradient_x = (
                        first[2] * second[1] - first[1] * second[2]
                    ) / determinant
                    gradient_z = (
                        first[0] * second[2] - first[2] * second[0]
                    ) / determinant
                elif vectors:
                    longest = max(
                        vectors, key=lambda value: value[0] ** 2 + value[1] ** 2)
                    length2 = longest[0] ** 2 + longest[1] ** 2
                    if length2 > 1e-8:
                        gradient_x = longest[2] * longest[0] / length2
                        gradient_z = longest[2] * longest[1] / length2
                # A rigid placement must put every descriptor node on the
                # same plane.  If the dataset contradicts that contract, keep
                # the old origin-only transform so downstream continuity
                # validation remains fail-closed.
                residual = max(abs(
                    base_delta + gradient_x * (x - base_x)
                    + gradient_z * (z - base_z) - delta
                ) for _node_index, x, z, delta in samples)
                if residual <= 0.35:
                    elevation_plane = (
                        base_x, base_z, base_delta, gradient_x, gradient_z)
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
                if elevation_plane is None:
                    world_y = anchor_y + local_y - origin_y
                else:
                    base_x, base_z, base_delta, gradient_x, gradient_z = \
                        elevation_plane
                    world_y = (
                        local_y + base_delta
                        + gradient_x * (x - base_x)
                        + gradient_z * (z - base_z)
                    )
                piece.append((
                    anchor[0] + (x - ox) * c - (z - oz) * s,
                    world_y,
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

    def _make_prefab_lane_segment(self, edge, instance, indices, points,
                                  fallback_lane_index=0):
        """Build one lane segment from an already proven PPD curve chain."""
        lane_data = self._prefab_lane_data.get(instance[0]) or {}
        try:
            end_item = instance[1].index(edge.end_uid)
            output_lanes = tuple((lane_data.get("nodes") or ())[end_item]
                                 ["output_lanes"])
        except (ValueError, IndexError, KeyError, TypeError):
            output_lanes = ()
        exit_lane_index = (output_lanes.index(indices[-1])
                           if indices[-1] in output_lanes
                           else fallback_lane_index)
        lane_id = LaneId(min(instance[1]), 1, exit_lane_index,
                         instance[0], indices[0], tuple(indices))
        prefab_path = str(lane_data.get("path", "")).lower()
        return LaneSegment(
            lane_id, edge.start_uid, edge.end_uid, 1, exit_lane_index,
            max(1, len(output_lanes)), 4.5, "derived",
            int(round(points[len(points)//2].y / 3.0)), None,
            ("roundabout" if "roundabout" in prefab_path else "prefab"),
            points, connector_curve_indices=tuple(indices),
            gps_uids=frozenset((edge.start_uid, edge.end_uid)))

    def _prefab_lane_segment(self, edge, lane_index, start_position=None,
                             register=True, allow_parallel_sibling=False):
        """Resolve a GPS-selected prefab edge to one proven navCurve chain.

        ETS2LA does not assume that road lane index N equals prefab input N.
        It first enumerates topologically valid input/output paths, then picks
        the entry navCurve nearest the actual incoming lane position.  Geometry
        is used only to rank already-confirmed paths; it never creates an edge.
        """
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
            if (allow_parallel_sibling
                    and start_position is not None and options):
                sx = float(start_position.x if hasattr(start_position, "x")
                           else start_position[0])
                sy = float(start_position.y if hasattr(start_position, "y")
                           else start_position[1])
                sz = float(start_position.z if hasattr(start_position, "z")
                           else start_position[2])
                representative_gaps = []
                for option in options:
                    option_points = self._prefab_curve_chain_3d(
                        instance, option)
                    if option_points:
                        representative_gaps.append(math.dist(
                            (sx, sy, sz),
                            (option_points[0].x, option_points[0].y,
                             option_points[0].z)))
                # The representative navNode connector can describe only one
                # of several parallel lanes. Consult reciprocal PPD lane links
                # only when that representative is demonstrably not the live
                # incoming lane. This keeps roundabouts and already continuous
                # prefabs on their former deterministic path.
                if (representative_gaps
                        and min(representative_gaps) > 1.0):
                    aligned_siblings = []
                    for sibling in self._prefab_parallel_lane_options(
                            instance, edge.start_uid, edge.end_uid):
                        sibling_points = self._prefab_curve_chain_3d(
                            instance, sibling)
                        if not sibling_points:
                            continue
                        entry = sibling_points[0]
                        gap = math.dist((sx, sy, sz),
                                        (entry.x, entry.y, entry.z))
                        incoming_heading = getattr(start_position,
                                                   "heading", None)
                        heading_error = (abs((
                            entry.heading - float(incoming_heading) + math.pi
                        ) % (2.0 * math.pi) - math.pi)
                            if incoming_heading is not None else 0.0)
                        # Use the same 0.35 m continuity contract enforced
                        # between consecutive prefab segments below.  This is
                        # proof of one shared lane endpoint, not a wider gap
                        # allowance or a synthetic connecting chord.
                        if (gap <= 0.35
                                and heading_error <= math.radians(10.0)):
                            aligned_siblings.append(sibling)
                    # One exact lane-centre match is evidence. Zero or several
                    # matches are not; retain the navNode representative so
                    # an unrelated fork/loop cannot become a hidden choice.
                    if len(aligned_siblings) == 1:
                        options = aligned_siblings
            preferred_curve = (input_lanes[min(lane_index, len(input_lanes)-1)]
                               if input_lanes else None)
            if start_position is not None:
                # Match ETS2LA's GetClosestCurve: consider every proven entry
                # curve and rank it against the live incoming lane endpoint.
                chosen_options = options
            else:
                preferred = [option for option in options
                             if preferred_curve is not None
                             and option[0] == preferred_curve]
                chosen_options = preferred or (options if len(options) == 1 else [])
            for indices in chosen_options:
                points = self._prefab_curve_chain_3d(instance, indices)
                if len(points) >= 2:
                    candidates.append((instance, indices, points))
        if len(candidates) > 1 and start_position is not None:
            sx = float(start_position.x if hasattr(start_position, "x")
                       else start_position[0])
            sy = float(start_position.y if hasattr(start_position, "y")
                       else start_position[1])
            sz = float(start_position.z if hasattr(start_position, "z")
                       else start_position[2])
            ranked = sorted(candidates, key=lambda item: math.dist(
                (sx, sy, sz),
                (item[2][0].x, item[2][0].y, item[2][0].z)))
            best_entry = ranked[0][1][0]
            # Once the physical entry curve is known, GPS's end UID and the
            # nav graph determine the exit. Paths beginning elsewhere belong
            # to another incoming lane/arm and must not remain ambiguous.
            candidates = [item for item in ranked if item[1][0] == best_entry]
        if len(candidates) > 1:
            # Some roundabout PPDs enumerate both the direct confirmed exit
            # and a full extra lap that returns to the same output curve. GPS
            # supplies only entry/exit UIDs, so an extra lap is not an
            # independent route choice. When every option has the identical
            # instance, entry curve and exit curve, accept only a uniquely
            # shortest navCurve chain; otherwise retain fail-closed ambiguity.
            signatures = {(item[0][0], item[1][0], item[1][-1])
                          for item in candidates}
            if len(signatures) == 1:
                ranked = sorted(candidates, key=lambda item: sum(
                    math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                    for a, b in zip(item[2], item[2][1:])))
                shortest = sum(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                               for a, b in zip(ranked[0][2], ranked[0][2][1:]))
                next_length = sum(
                    math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                    for a, b in zip(ranked[1][2], ranked[1][2][1:]))
                if shortest < next_length * 0.75:
                    candidates = [ranked[0]]
        if len(candidates) != 1:
            return None, ("ambiguous prefab lane connector"
                          if candidates else "missing prefab lane connector")
        instance, indices, points = candidates[0]
        segment = self._make_prefab_lane_segment(
            edge, instance, indices, points, lane_index)
        if register:
            self._lane_id_index[segment.lane_id] = segment
        return segment, ""

    def route_prefix_lane_segments_near(self, position, gps_uids,
                                        radius=28.0, register=True):
        """Return nearby proven prefab lanes entering the first GPS UID.

        The rolling game GPS can drop the junction entrance before the truck
        has physically left the preceding prefab. Only directed PPD connector
        chains ending at the first authoritative UID are eligible. If several
        already-passed histories have converged onto the same terminal curve,
        that current curve is one lane and is deduplicated accordingly.
        """
        gps_uids = tuple(_uid(value) for value in gps_uids if _uid(value))
        if (not self.loaded or len(position) < 3 or len(gps_uids) < 2
                or any(uid not in self.nodes for uid in gps_uids)):
            return ()
        route_start_uid = gps_uids[0]
        first_distinct_pair = next((
            (pair_index, start_uid, end_uid)
            for pair_index, (start_uid, end_uid) in enumerate(
                zip(gps_uids, gps_uids[1:]))
            if start_uid != end_uid
        ), None)
        if first_distinct_pair is None:
            return ()
        pair_index, start_uid, end_uid = first_distinct_pair
        if self._classify_corridor_edge(
                start_uid, end_uid, pair_index) is None:
            # A prefix entering the first UID belongs to this GPS route only
            # when the route's first outgoing connectivity is also proven.
            return ()
        px, _py, pz = position
        cx, cz = self._cell(px, pz)
        rings = int(radius // self.GRID) + 1
        instances, seen_instances = [], set()
        for dx in range(-rings, rings + 1):
            for dz in range(-rings, rings + 1):
                for instance in self._prefab_grid.get((cx + dx, cz + dz), ()):
                    marker = (instance[0], instance[1])
                    if (marker in seen_instances
                            or route_start_uid not in instance[1]):
                        continue
                    seen_instances.add(marker)
                    instances.append(instance)

        result, seen_lanes = [], set()
        for instance in instances:
            token, uids = instance[:2]
            lane_data = self._prefab_lane_data.get(token) or {}
            try:
                end_item = uids.index(route_start_uid)
                node_data = lane_data["nodes"][end_item]
            except (ValueError, IndexError, KeyError, TypeError):
                continue
            output_lanes = tuple(node_data.get("output_lanes", ()))
            prefab_path = str(lane_data.get("path", "")).lower()

            def make_segment(start_uid, indices, points):
                exit_curve = indices[-1]
                lane_index = (output_lanes.index(exit_curve)
                              if exit_curve in output_lanes else 0)
                lane_id = LaneId(
                    min(uids), 1, lane_index, token, indices[0], indices)
                return LaneSegment(
                    lane_id, start_uid, route_start_uid, 1, lane_index,
                    max(1, len(output_lanes)), 4.5, "derived",
                    int(round(points[len(points)//2].y / 3.0)), None,
                    ("roundabout" if "roundabout" in prefab_path
                     else "prefab"),
                    points, connector_curve_indices=indices,
                    gps_uids=frozenset((route_start_uid,)))

            for start_uid in uids:
                if start_uid == route_start_uid:
                    continue
                for option in self._prefab_connector_options(
                        instance, start_uid, route_start_uid):
                    points = self._prefab_curve_chain_3d(instance, option)
                    if len(points) < 2:
                        continue
                    segment = make_segment(start_uid, option, points)
                    projected = LaneLocator._project(position, segment)
                    if projected is None or projected[0] > radius:
                        continue
                    # At a converged output the entrance history is behind the
                    # truck and cannot identify a different current lane. Use
                    # the exact terminal navCurve as its canonical identity.
                    terminal = (option[-1],)
                    terminal_points = self._prefab_curve_chain_3d(
                        instance, terminal)
                    if len(terminal_points) >= 2:
                        terminal_segment = make_segment(
                            route_start_uid, terminal, terminal_points)
                        terminal_projection = LaneLocator._project(
                            position, terminal_segment)
                        if (terminal_projection is not None
                                and terminal_projection[0]
                                    <= projected[0] + 1e-6):
                            segment = terminal_segment
                    lane_id = segment.lane_id
                    if lane_id in seen_lanes:
                        continue
                    seen_lanes.add(lane_id)
                    if register:
                        self._lane_id_index[lane_id] = segment
                    result.append(segment)
        return tuple(result)

    def gps_prefab_lane_segments_near(self, position, gps_uids, radius=28.0,
                                      register=True):
        """Expose only directly GPS-proven prefab lanes to LaneLocator."""
        if not self.loaded or len(position) < 3:
            return ()
        gps_uids = tuple(_uid(value) for value in gps_uids if _uid(value))
        if any(uid not in self.nodes for uid in gps_uids):
            return ()
        result, seen = [], set()
        for pair_index, (start_uid, end_uid) in enumerate(
                zip(gps_uids, gps_uids[1:])):
            if start_uid == end_uid:
                continue
            edge = self._classify_corridor_edge(
                start_uid, end_uid, pair_index)
            if edge is None:
                # Never jump ahead to a geometrically nearby later arm after
                # the current GPS prefix has already lost topology.
                break
            if edge.kind != "prefab":
                continue
            lane_count = 0
            for instance in edge.prefab_instance or ():
                lane_data = self._prefab_lane_data.get(instance[0]) or {}
                try:
                    start_item = instance[1].index(start_uid)
                    lane_count = max(lane_count, len(
                        lane_data["nodes"][start_item]["input_lanes"]))
                except (ValueError, IndexError, KeyError, TypeError):
                    continue
            for lane_index in range(max(1, lane_count)):
                segment, _reason = self._prefab_lane_segment(
                    edge, lane_index, position, register=register,
                    allow_parallel_sibling=True)
                if segment is None or segment.lane_id in seen:
                    continue
                projected = LaneLocator._project(position, segment)
                if projected is None or projected[0] > radius:
                    continue
                seen.add(segment.lane_id)
                result.append(segment)
        return tuple(result)

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

    @staticmethod
    def _retarget_road_end_to_prefab(road, prefab):
        """Taper a confirmed road lane into the selected prefab input lane.

        A PPD can expose different input navCurves for different GPS exits.
        When the requested exit starts one lane beside the continuing lane,
        the lane change belongs on the preceding road—not in a 4.5 m chord at
        the physical prefab boundary. This helper is intentionally narrow: it
        requires shared topology, equal height, matching direction and enough
        road distance for a gradual transition to the exact PPD endpoint.
        """
        if (road.lane_id.prefab_token is not None
                or prefab.lane_id.prefab_token in (None, "graph")
                or road.end_uid != prefab.start_uid
                or len(road.centerline) < 3 or len(prefab.centerline) < 2):
            return road
        source = road.centerline[-1]
        target = prefab.centerline[0]
        dx, dy, dz = (target.x-source.x, target.y-source.y,
                      target.z-source.z)
        gap = math.sqrt(dx*dx + dy*dy + dz*dz)
        if gap <= 0.35:
            return road
        first_heading = road.centerline[-1].heading
        second_heading = prefab.centerline[0].heading
        heading_jump = abs((second_heading-first_heading+math.pi)
                           % (2.0*math.pi)-math.pi)
        forward_x, forward_z = -math.sin(first_heading), -math.cos(first_heading)
        longitudinal = dx*forward_x + dz*forward_z
        lateral = dx*math.cos(first_heading) - dz*math.sin(first_heading)
        total = road.centerline[-1].s
        if (gap > road.width_m * 1.10 or abs(dy) > 1.0
                or heading_jump > math.radians(15.0)
                or abs(longitudinal) > 1.5
                or abs(lateral) > road.width_m * 1.05
                or total < max(24.0, abs(lateral) * 7.0)):
            return road

        transition = min(total, max(36.0, abs(lateral) * 12.0))
        shifted = []
        for point in road.centerline:
            progress = max(0.0, min(1.0,
                (point.s - (total-transition)) / transition))
            smooth = progress*progress*(3.0-2.0*progress)
            shifted.append((point.x + dx*smooth,
                            point.y + dy*smooth,
                            point.z + dz*smooth))
        # Preserve the exact authoritative PPD boundary and rebuild arc length
        # and headings after the bounded lane transition.
        shifted[-1] = (target.x, target.y, target.z)
        rebuilt, travelled = [], 0.0
        for index, point in enumerate(shifted):
            if rebuilt:
                travelled += math.dist(shifted[index-1], point)
            before = shifted[max(0, index-1)]
            after = shifted[min(len(shifted)-1, index+1)]
            tx, tz = after[0]-before[0], after[2]-before[2]
            heading = (math.atan2(-tx, -tz) if math.hypot(tx, tz) > 1e-8
                       else (rebuilt[-1].heading if rebuilt else first_heading))
            rebuilt.append(LanePoint(
                point[0], point[1], point[2], travelled, heading,
                lane_id=road.lane_id))
        return replace(road, centerline=tuple(rebuilt))

    @staticmethod
    def _retarget_road_start_from_prefab(prefab, road):
        """Taper a confirmed prefab output into the following road lane.

        This is the exit-side counterpart of ``_retarget_road_end_to_prefab``.
        It removes a one-sample lateral kink only when the two real segments
        share a UID, height and direction and the correction stays inside one
        lane width over a sufficiently long road segment.
        """
        if (prefab.lane_id.prefab_token in (None, "graph")
                or road.lane_id.prefab_token is not None
                or prefab.end_uid != road.start_uid
                or len(prefab.centerline) < 2 or len(road.centerline) < 3):
            return road
        source = road.centerline[0]
        target = prefab.centerline[-1]
        dx, dy, dz = (target.x-source.x, target.y-source.y,
                      target.z-source.z)
        gap = math.sqrt(dx*dx + dy*dy + dz*dz)
        if gap <= 0.35:
            return road
        first_heading = prefab.centerline[-1].heading
        second_heading = road.centerline[0].heading
        heading_jump = abs((second_heading-first_heading+math.pi)
                           % (2.0*math.pi)-math.pi)
        forward_x, forward_z = -math.sin(second_heading), -math.cos(second_heading)
        longitudinal = dx*forward_x + dz*forward_z
        lateral = dx*math.cos(second_heading) - dz*math.sin(second_heading)
        total = road.centerline[-1].s
        if (gap > road.width_m * 1.10 or abs(dy) > 1.0
                or heading_jump > math.radians(15.0)
                or abs(longitudinal) > 1.5
                or abs(lateral) > road.width_m * 1.05
                or total < max(24.0, abs(lateral) * 7.0)):
            return road

        transition = min(total, max(36.0, abs(lateral) * 12.0))
        shifted = []
        for point in road.centerline:
            progress = max(0.0, min(1.0, point.s / transition))
            smooth = 1.0 - progress*progress*(3.0-2.0*progress)
            shifted.append((point.x + dx*smooth,
                            point.y + dy*smooth,
                            point.z + dz*smooth))
        shifted[0] = (target.x, target.y, target.z)
        rebuilt, travelled = [], 0.0
        for index, point in enumerate(shifted):
            if rebuilt:
                travelled += math.dist(shifted[index-1], point)
            before = shifted[max(0, index-1)]
            after = shifted[min(len(shifted)-1, index+1)]
            tx, tz = after[0]-before[0], after[2]-before[2]
            heading = (math.atan2(-tx, -tz) if math.hypot(tx, tz) > 1e-8
                       else (rebuilt[-1].heading if rebuilt else second_heading))
            rebuilt.append(LanePoint(
                point[0], point[1], point[2], travelled, heading,
                lane_id=road.lane_id))
        return replace(road, centerline=tuple(rebuilt))

    @staticmethod
    def _lane_boundary_is_continuous(first, second):
        """Require one shared lane endpoint, direction and elevation layer."""
        if (first.end_uid != second.start_uid
                or first.direction != second.direction
                or not first.centerline or not second.centerline):
            return False
        source = first.centerline[-1]
        target = second.centerline[0]
        gap = math.dist((source.x, source.y, source.z),
                        (target.x, target.y, target.z))
        heading = abs((target.heading-source.heading+math.pi)
                      % (2.0*math.pi)-math.pi)
        return gap <= 0.35 and heading <= math.radians(15.0)

    def _parallel_segments_for_edge(self, edge, original):
        """Enumerate only real adjacent road lanes or PPD-proven siblings."""
        if edge.kind == "road":
            result = []
            for lane in self._build_lane_segments(edge.segment_index):
                if (lane.start_uid != edge.start_uid
                        or lane.end_uid != edge.end_uid):
                    continue
                if (original.raw_lane_index >= 0
                        and lane.raw_lane_index >= 0
                        and abs(lane.raw_lane_index
                                - original.raw_lane_index) > 1):
                    continue
                result.append(lane)
            return tuple(result)
        if edge.kind != "prefab":
            return ()
        result, seen = [], set()
        for instance in edge.prefab_instance or ():
            options = set(self._prefab_connector_options(
                instance, edge.start_uid, edge.end_uid))
            options.update(self._prefab_parallel_lane_options(
                instance, edge.start_uid, edge.end_uid))
            for indices in sorted(options):
                points = self._prefab_curve_chain_3d(instance, indices)
                if len(points) < 2:
                    continue
                segment = self._make_prefab_lane_segment(
                    edge, instance, indices, points,
                    original.lane_index)
                if segment.lane_id in seen:
                    continue
                seen.add(segment.lane_id)
                result.append(segment)
        return tuple(result)

    def _backtrack_confirmed_lane_change(self, corridor, selected, current,
                                         edge_number):
        """Move a required adjacent-lane transition onto a safe earlier road.

        The normal selector follows the currently occupied lane greedily. A
        chain of short road/prefab pieces can then reach a junction whose only
        GPS-confirmed input starts on the adjacent lane. Never bridge that
        boundary. Walk the already selected GPS edges backwards, using only
        real directed road lanes and reciprocal PPD input/output chains, until
        one sufficiently long road can contain the transition. Every
        intermediate boundary must remain an exact endpoint match.
        """
        if (not selected or edge_number <= 0
                or current.lane_id.prefab_token in (None, "graph")
                or selected[-1].lane_id.prefab_token is not None
                or selected[-1].end_uid != current.start_uid):
            return None
        source = selected[-1].centerline[-1]
        target = current.centerline[0]
        dx, dy, dz = target.x-source.x, target.y-source.y, target.z-source.z
        gap = math.sqrt(dx*dx + dy*dy + dz*dz)
        heading = abs((target.heading-source.heading+math.pi)
                      % (2.0*math.pi)-math.pi)
        forward_x = -math.sin(source.heading)
        forward_z = -math.cos(source.heading)
        longitudinal = dx*forward_x + dz*forward_z
        lateral = dx*math.cos(source.heading) - dz*math.sin(source.heading)
        width = min(selected[-1].width_m, current.width_m)
        if (gap <= 0.35 or gap > width*1.10 or abs(dy) > 1.0
                or heading > math.radians(10.0)
                or abs(longitudinal) > 1.5
                or abs(lateral) > width*1.05):
            return None

        explored = 0
        search_truncated = False

        def search(index, downstream):
            nonlocal explored, search_truncated
            if index < 0:
                return []
            if explored >= 128:
                search_truncated = True
                return []
            edge = corridor.edges[index]
            original = selected[index]
            matches = [candidate for candidate in
                       self._parallel_segments_for_edge(edge, original)
                       if self._lane_boundary_is_continuous(
                           candidate, downstream)]
            solutions = []
            for candidate in matches:
                explored += 1
                previous = selected[index-1] if index else None
                anchored = False
                if previous is not None and self._lane_boundary_is_continuous(
                        previous, candidate):
                    solutions.append((index, {index: candidate}))
                    anchored = True
                elif (previous is not None
                        and previous.lane_id.prefab_token
                            not in (None, "graph")
                        and candidate.lane_id.prefab_token is None):
                    tapered = self._retarget_road_start_from_prefab(
                        previous, candidate)
                    if (self._lane_boundary_is_continuous(previous, tapered)
                            and self._lane_boundary_is_continuous(
                                tapered, downstream)):
                        solutions.append((index, {index: tapered}))
                        anchored = True
                if not anchored:
                    for start, upstream in search(index-1, candidate):
                        combined = dict(upstream)
                        combined[index] = candidate
                        solutions.append((start, combined))
                        if len(solutions) >= 16:
                            search_truncated = True
                            return solutions
                if len(solutions) >= 16:
                    search_truncated = True
                    return solutions
            return solutions

        solutions = search(edge_number-1, current)
        if search_truncated or not solutions:
            return None
        # Preserve the occupied lane for as long as topology permits. A path
        # that anchors later is a proven merge/split or a shorter safe lane
        # change, while an earlier alternative would move the truck without
        # need. Equal latest anchors are still genuinely ambiguous.
        latest_start = max(solution[0] for solution in solutions)
        latest = [solution for solution in solutions
                  if solution[0] == latest_start]
        if len(latest) != 1:
            return None
        start_index, replacements = latest[0]

        rebuilt = list(selected)
        for index, segment in replacements.items():
            rebuilt[index] = segment
        chain = rebuilt + [current]
        boundary_start = max(0, start_index-1)
        for index in range(boundary_start, len(chain)-1):
            if not self._lane_boundary_is_continuous(
                    chain[index], chain[index+1]):
                return None
        for index in range(boundary_start, len(rebuilt)):
            connection = self._lane_connection(
                chain[index], chain[index+1])
            if connection is None:
                return None
            rebuilt[index] = replace(rebuilt[index],
                                     successors=(connection,))
        for index in replacements:
            segment = rebuilt[index]
            self._lane_id_index[segment.lane_id] = segment
        return rebuilt

    def select_lane_sequence(self, corridor, start_match):
        """Select one continuous lane for every authoritative corridor edge."""
        if not isinstance(corridor, GpsCorridor) or not corridor.valid:
            return (), (getattr(corridor, "failure_reason", "invalid corridor")
                        or "invalid corridor")
        if start_match is None:
            return (), "LaneLocator did not confirm a starting lane"
        selected = []
        lane_index = start_match.lane_id.lane_index
        matched_lane = self._lane_id_index.get(start_match.lane_id)
        raw_lane_index = (matched_lane.raw_lane_index
                          if matched_lane is not None
                          and matched_lane.raw_lane_index >= 0
                          else lane_index)
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
                by_raw_index = {lane.raw_lane_index: lane for lane in lanes}
                if (selected and selected[-1].lane_id.prefab_token
                        not in (None, "graph")):
                    # PPD outputLanes are navCurve identifiers, not road-lane
                    # ordinals.  Their list order is reversed in some SCS
                    # prefabs (for example dlc_blkw_81).  The road edge is
                    # already proven by the authoritative GPS corridor, so
                    # choose only among its real directed lanes by continuity
                    # with the selected output navCurve.  Treating the
                    # outputLanes array index as raw_lane_index moved the path
                    # across a lane at the prefab boundary and produced a
                    # sharp artificial bump.
                    exit_point = selected[-1].centerline[-1]
                    ranked_lanes = sorted(lanes, key=lambda lane: math.dist(
                        (exit_point.x, exit_point.y, exit_point.z),
                        (lane.centerline[0].x, lane.centerline[0].y,
                         lane.centerline[0].z)))
                    best_gap = math.dist(
                        (exit_point.x, exit_point.y, exit_point.z),
                        (ranked_lanes[0].centerline[0].x,
                         ranked_lanes[0].centerline[0].y,
                         ranked_lanes[0].centerline[0].z))
                    next_gap = (math.dist(
                        (exit_point.x, exit_point.y, exit_point.z),
                        (ranked_lanes[1].centerline[0].x,
                         ranked_lanes[1].centerline[0].y,
                         ranked_lanes[1].centerline[0].z))
                        if len(ranked_lanes) > 1 else float("inf"))
                    if best_gap > 6.0:
                        return tuple(selected), (
                            f"prefab output has no continuous lane on road edge "
                            f"{edge.start_uid} -> {edge.end_uid} "
                            f"(nearest gap {best_gap:.2f} m)")
                    if next_gap - best_gap < 0.35:
                        return tuple(selected), (
                            f"prefab output is ambiguous between road lanes at "
                            f"{edge.start_uid} (gaps {best_gap:.2f} m and "
                            f"{next_gap:.2f} m)")
                    current = ranked_lanes[0]
                    lane_index = current.lane_index
                    raw_lane_index = current.raw_lane_index
                elif raw_lane_index in by_raw_index:
                    current = by_raw_index[raw_lane_index]
                    lane_index = current.lane_index
                elif lane_index in by_index and not selected:
                    current = by_index[lane_index]
                    raw_lane_index = current.raw_lane_index
                elif selected and raw_lane_index > max(by_raw_index):
                    # A disappearing outer lane is a confirmed merge at the
                    # shared road node. Clamp only to the adjacent edge lane.
                    raw_lane_index = max(by_raw_index)
                    current = by_raw_index[raw_lane_index]
                    lane_index = current.lane_index
                else:
                    return tuple(selected), (
                        f"starting lane {lane_index} (raw {raw_lane_index}) is "
                        f"unavailable on road edge "
                        f"{edge.start_uid} -> {edge.end_uid}")
            elif edge.kind == "prefab":
                incoming_point = (selected[-1].centerline[-1] if selected
                                  else start_match.point)
                current, reason = self._prefab_lane_segment(
                    edge, lane_index, incoming_point,
                    allow_parallel_sibling=(
                        not selected
                        or selected[-1].lane_id.prefab_token is None
                        or (selected[-1].end_uid == edge.start_uid
                            and self._segment_uses_parallel_prefab_sibling(
                                selected[-1]))))
                if current is None:
                    return tuple(selected), (
                        f"{reason} for prefab {edge.start_uid} -> {edge.end_uid}")
                if selected:
                    selected[-1] = self._retarget_road_end_to_prefab(
                        selected[-1], current)
                    source = selected[-1].centerline[-1]
                    target = current.centerline[0]
                    boundary_gap = math.dist(
                        (source.x, source.y, source.z),
                        (target.x, target.y, target.z))
                    if boundary_gap > 0.35:
                        recovered = self._backtrack_confirmed_lane_change(
                            corridor, selected, current, edge_number)
                        if recovered is None:
                            return tuple(selected), (
                                "road-to-prefab lane identity mismatch at UID "
                                f"{current.start_uid}: geometry gap "
                                f"{boundary_gap:.2f} m; "
                                "no GPS-proven adjacent-lane approach")
                        selected = recovered
            else:
                # A directed graph edge proves node reachability, but it has no
                # concrete lane centre, width or elevation. Do not invent one.
                return tuple(selected), (
                    f"graph-only edge {edge.start_uid} -> {edge.end_uid} "
                    "has no lane-confirmed geometry")
            if (selected and selected[-1].lane_id.prefab_token
                    not in (None, "graph")
                    and current.lane_id.prefab_token is None):
                current = self._retarget_road_start_from_prefab(
                    selected[-1], current)
            if selected:
                if (selected[-1].lane_id.prefab_token not in (None, "graph")
                        and current.lane_id.prefab_token not in (None, "graph")):
                    previous_end = selected[-1].centerline[-1]
                    current_start = current.centerline[0]
                    heading = selected[-1].centerline[-1].heading
                    right_x, right_z = -math.cos(heading), math.sin(heading)
                    lateral = ((current_start.x - previous_end.x) * right_x
                               + (current_start.z - previous_end.z) * right_z)
                    elevation = current_start.y - previous_end.y
                    gap = math.dist(
                        (previous_end.x, previous_end.y, previous_end.z),
                        (current_start.x, current_start.y, current_start.z))
                    if gap > 0.35:
                        return tuple(selected), (
                            "prefab lane identity mismatch at UID "
                            f"{current.start_uid}: geometry gap {gap:.2f} m "
                            f"(lateral offset {abs(lateral):.2f} m, "
                            f"elevation {abs(elevation):.2f} m)")
                connection = self._lane_connection(selected[-1], current)
                if connection is None:
                    return tuple(selected), (
                        f"no LaneConnection from {selected[-1].end_uid} to "
                        f"{current.start_uid} at corridor edge {edge_number}")
                selected[-1] = replace(selected[-1],
                                       successors=(connection,))
            selected.append(current)
            lane_index = current.lane_index
            raw_lane_index = (current.raw_lane_index
                              if current.raw_lane_index >= 0 else lane_index)
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
                if (gap > 0.35
                        and previous.lane_id.prefab_token not in (None, "graph")
                        and segment.lane_id.prefab_token not in (None, "graph")):
                    return LanePath(
                        segments, tuple(points), uids, valid=False,
                        failure_reason=(
                            "unproven prefab geometry chord of "
                            f"{gap:.2f} m at UID {segment.start_uid}"))
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
                        previous_match=None, start_match=None,
                        diagnostics=None):
        """Convenience pipeline used by tests and the future map-plugin switch."""
        if diagnostics is not None:
            safe_diagnostic_call(diagnostics, "start_phase",
                                 "resolve_gps_corridor", {
                "gps_uid_count": len(tuple(gps_uids or ())),
            })
        corridor = self.resolve_gps_corridor(gps_uids)
        if not corridor.valid:
            if diagnostics is not None:
                missing = next((
                    (index, uid) for index, uid in enumerate(corridor.gps_uids)
                    if uid not in self.nodes), (None, None))
                safe_diagnostic_call(
                    diagnostics, "fail_phase", "resolve_gps_corridor",
                    corridor.failure_reason, {
                        "gps_uid_index": missing[0],
                        "gps_uid": missing[1],
                        "resolved_edge_count": len(corridor.edges),
                    })
            return LanePath((), (), corridor.gps_uids, valid=False,
                            failure_reason=corridor.failure_reason), None
        if diagnostics is not None:
            safe_diagnostic_call(diagnostics, "finish_phase",
                                 "resolve_gps_corridor", details={
                "gps_uid_count": len(corridor.gps_uids),
                "edge_count": len(corridor.edges),
                "edge_kinds": [edge.kind for edge in corridor.edges],
            })
        if altitude is None:
            locator_position = tuple(position[:2])
        else:
            locator_position = (float(position[0]), float(altitude),
                                float(position[1]))
        match = start_match
        if match is None:
            locator_capture = {} if diagnostics is not None else None
            if diagnostics is not None:
                safe_diagnostic_call(diagnostics, "start_phase", "LaneLocator")
            match = LaneLocator(self).locate(locator_position, heading,
                                             corridor.gps_uids, previous_match,
                                             diagnostics=locator_capture)
            if diagnostics is not None:
                safe_diagnostic_call(diagnostics, "observe_locator",
                                     locator_capture, match)
                if match is None:
                    reason = ("LaneLocator result is ambiguous" if
                              locator_capture.get("outcome") == "ambiguous" else
                              "LaneLocator did not confirm a starting lane")
                    safe_diagnostic_call(diagnostics, "fail_phase",
                                         "LaneLocator", reason,
                                         locator_capture)
                else:
                    safe_diagnostic_call(diagnostics, "finish_phase",
                                         "LaneLocator", details={
                        "outcome": "matched",
                        "lane_id": lane_id_payload(match.lane_id),
                        "confidence": float(match.confidence),
                    })
        if diagnostics is not None:
            safe_diagnostic_call(diagnostics, "start_phase",
                                 "select_lane_sequence")
        segments, reason = self.select_lane_sequence(corridor, match)
        if reason:
            if diagnostics is not None:
                edge_index = min(len(segments), max(0, len(corridor.edges) - 1))
                edge = corridor.edges[edge_index] if corridor.edges else None
                last = segments[-1] if segments else None
                safe_diagnostic_call(diagnostics, "fail_phase",
                                     "select_lane_sequence", reason, {
                    "gps_uid_index": (edge.gps_pair_index if edge else None),
                    "gps_uid": (edge.start_uid if edge else None),
                    "road_token": (last.road_look_token if last else None),
                    "prefab_token": (
                        edge.prefab_instance[0][0]
                        if edge and edge.prefab_instance else
                        last.lane_id.prefab_token if last else None),
                    "lane_id_after": ({
                        "road_uid": int(last.lane_id.road_uid),
                        "direction": int(last.lane_id.direction),
                        "lane_index": int(last.lane_id.lane_index),
                        "prefab_token": last.lane_id.prefab_token,
                        "connector_index": last.lane_id.connector_index,
                        "connector_path": list(last.lane_id.connector_path),
                    } if last else None),
                    "selected_segment_count": len(segments),
                })
            return LanePath(segments, (), corridor.gps_uids, valid=False,
                            failure_reason=reason), match
        if diagnostics is not None:
            safe_diagnostic_call(diagnostics, "finish_phase",
                                 "select_lane_sequence", details={
                "segment_count": len(segments),
                "lane_ids": [str(segment.lane_id) for segment in segments],
            })
        # The rolling SDK route begins at the next GPS anchor. The truck may
        # still be on the confirmed incoming lane leading to that anchor. Add
        # this real lane segment before the first corridor edge so HUD, AR and
        # steering start at the truck instead of 10+ metres across the prefab.
        active = self._lane_id_index.get(match.lane_id) if match else None
        if active is not None and segments and active.lane_id != segments[0].lane_id:
            active_index = next(
                (index for index, segment in enumerate(segments)
                 if segment.lane_id == active.lane_id), None)
            if active_index is not None:
                # The SCS GPS list is a rolling look-ahead and can retain one
                # or more anchors already passed by the truck.  Once the
                # locator confirms a later lane from that exact authoritative
                # sequence, discard only the passed prefix.  Prepending the
                # later lane before edge zero reverses the proven topology and
                # caused navigation to fail a few metres after setting off.
                segments = tuple(segments[active_index:])
                active = segments[0]
            else:
                prefix = None
                if active.end_uid != segments[0].start_uid:
                    prefix_edge = self._classify_corridor_edge(
                        active.end_uid, segments[0].start_uid, -1)
                    if prefix_edge is not None and prefix_edge.kind == "prefab":
                        prefix, _reason = self._prefab_lane_segment(
                            prefix_edge, active.lane_index,
                            active.centerline[-1],
                            allow_parallel_sibling=True)
                if prefix is not None:
                    active = self._retarget_road_end_to_prefab(active, prefix)
                    first_connection = self._lane_connection(active, prefix)
                    next_connection = self._lane_connection(prefix, segments[0])
                    if first_connection is not None and next_connection is not None:
                        active = replace(active, successors=(first_connection,))
                        prefix = replace(prefix, successors=(next_connection,))
                        segments = (active, prefix) + tuple(segments)
                        connection = first_connection
                    else:
                        connection = None
                else:
                    # A rolling GPS window can start exactly at the prefab
                    # connected to the truck's current road lane. This direct
                    # prefix path used to add only LaneConnection metadata and
                    # skipped the same bounded road->prefab taper used by the
                    # normal corridor loop. On the captured ProMods junction
                    # that joined road lane 1 to prefab lane 0 in one sample
                    # (52.16 degree heading jump). Retarget only when the
                    # existing topology, direction, height and one-lane bounds
                    # prove the transition; otherwise validation stays closed.
                    if (active.end_uid == segments[0].start_uid
                            and segments[0].lane_id.prefab_token
                                not in (None, "graph")):
                        active = self._retarget_road_end_to_prefab(
                            active, segments[0])
                        active_end = active.centerline[-1]
                        prefab_start = segments[0].centerline[0]
                        entry_gap = math.dist(
                            (active_end.x, active_end.y, active_end.z),
                            (prefab_start.x, prefab_start.y, prefab_start.z))
                        heading = active_end.heading
                        entry_dx = prefab_start.x - active_end.x
                        entry_dz = prefab_start.z - active_end.z
                        lateral_gap = abs(
                            entry_dx * math.cos(heading)
                            - entry_dz * math.sin(heading))
                        # Do not turn harmless sub-metre source-coordinate or
                        # height noise into a new route rejection. This early
                        # gate is only for the demonstrated adjacent-lane
                        # chord; normal residuals still reach the existing
                        # trajectory validator unchanged.
                        if entry_gap > 0.35 and lateral_gap > 1.0:
                            # The captured ProMods failure had 4.50 m of
                            # lateral displacement but only 11.95 m of road
                            # remaining. A one-sample join hit 52.16 degrees
                            # and aimed the truck at the roadside pole. If the
                            # bounded taper cannot prove a safe lane change,
                            # report the actionable lane error before geometry
                            # construction instead of emitting a chord.
                            failed = LanePath(
                                tuple(segments), (), corridor.gps_uids,
                                valid=False,
                                failure_reason=(
                                    "GPS turn begins in an adjacent lane; "
                                    f"{lateral_gap:.2f} m lateral transition "
                                    "cannot be completed safely before the "
                                    "junction"))
                            if diagnostics is not None:
                                safe_diagnostic_call(
                                    diagnostics, "observe_lane_path", failed)
                                safe_diagnostic_call(
                                    diagnostics, "fail_phase", "LanePath",
                                    failed.failure_reason, {
                                        "geometry": {
                                            "gap_m": float(entry_gap),
                                            "lateral_gap_m": float(
                                                lateral_gap),
                                        },
                                    })
                            return failed, match
                    connection = (self._lane_connection(active, segments[0])
                                  if active.end_uid == segments[0].start_uid
                                  else None)
                if connection is None:
                    # The GPS buffer may start at the next anchor, but it may
                    # not start on an unrelated parallel arm.
                    failed = LanePath(
                        tuple(segments), (), corridor.gps_uids, valid=False,
                        failure_reason=(
                            "current truck lane does not connect to the first GPS "
                            f"lane ({active.lane_id} -> {segments[0].lane_id})"))
                    if diagnostics is not None:
                        safe_diagnostic_call(diagnostics, "observe_lane_path",
                                             failed)
                        safe_diagnostic_call(diagnostics, "fail_phase",
                                             "LanePath", failed.failure_reason)
                    return failed, match
                if prefix is None:
                    active = replace(active, successors=(connection,))
                    segments = (active,) + tuple(segments)

        # Trim the actual first LaneSegment as well as the flattened LanePath.
        # build_lane_trajectory() deliberately rebuilds its control geometry
        # from segments, so trimming only LanePath.points resurrected the part
        # of the incoming road behind the truck and produced a screen-wide AR
        # chord.  Start at the exact, lane-confirmed projection: choosing the
        # nearest sampled centreline point can choose the sample behind the
        # truck and still draw a sharp chord through the camera.
        if match is not None and segments and segments[0].lane_id == match.lane_id:
            first = segments[0]
            line = first.centerline
            if len(line) >= 2:
                projected = replace(match.point, lane_id=first.lane_id,
                                    segment_index=0)
                # LaneLocator.segment_index identifies the source edge on
                # which ``projected`` lies. Only its end and later samples are
                # forward along this directed LaneSegment.
                following = line[max(0, match.segment_index + 1):]
                trimmed = [projected]
                trimmed.extend(point for point in following if math.dist(
                    (point.x, point.y, point.z),
                    (trimmed[-1].x, trimmed[-1].y, trimmed[-1].z)) > 1e-6)
                if len(trimmed) < 2:
                    failed = LanePath(
                        tuple(segments), (), corridor.gps_uids, valid=False,
                        failure_reason=(
                            "confirmed truck position leaves no forward "
                            "geometry on the first GPS lane"))
                    if diagnostics is not None:
                        safe_diagnostic_call(diagnostics, "observe_lane_path",
                                             failed)
                        safe_diagnostic_call(diagnostics, "fail_phase",
                                             "LanePath", failed.failure_reason)
                    return failed, match
                first = replace(first, centerline=tuple(trimmed))
                segments = (first,) + tuple(segments[1:])
        if diagnostics is not None:
            safe_diagnostic_call(diagnostics, "start_phase",
                                 "connect_lane_sequence")
        path = self.connect_lane_sequence(segments, corridor.gps_uids)
        if diagnostics is not None:
            safe_diagnostic_call(diagnostics, "observe_lane_path", path)
            if path.valid:
                safe_diagnostic_call(diagnostics, "finish_phase",
                                     "connect_lane_sequence", details={
                    "segment_count": len(path.segments),
                    "point_count": len(path.points),
                    "distance_m": float(path.distance_m),
                    "confidence": float(path.confidence),
                })
            else:
                safe_diagnostic_call(diagnostics, "fail_phase",
                                     "connect_lane_sequence",
                                     path.failure_reason)
        if not path.valid or match is None or len(path.points) < 2:
            return path, match

        if diagnostics is not None:
            safe_diagnostic_call(diagnostics, "start_phase", "LanePath")

        # A valid topology is still not enough for runtime steering: the
        # published polyline must begin at the lane actually occupied by the
        # truck and must point in the same direction.  Fail closed instead of
        # publishing a sideways connector or an opposing carriageway.
        nearest = min(range(len(path.points)), key=lambda index: math.dist(
            (path.points[index].x, path.points[index].y, path.points[index].z),
            (match.point.x, match.point.y, match.point.z)))
        nearest_distance = math.dist(
            (path.points[nearest].x, path.points[nearest].y, path.points[nearest].z),
            (match.point.x, match.point.y, match.point.z))
        if nearest_distance > 3.0:
            failed = replace(
                path, valid=False,
                failure_reason=(
                    f"first GPS lane is offset {nearest_distance:.1f} m from "
                    "the confirmed truck lane"))
            if diagnostics is not None:
                safe_diagnostic_call(diagnostics, "fail_phase",
                                     "LanePath", failed.failure_reason, {
                    "geometry": {"gap_m": float(nearest_distance)},
                })
            return failed, match
        probe = min(len(path.points) - 1, nearest + 2)
        if probe > nearest:
            dx = path.points[probe].x - path.points[nearest].x
            dz = path.points[probe].z - path.points[nearest].z
            if math.hypot(dx, dz) > 0.5:
                path_heading = math.atan2(-dx, -dz)
                heading_error = abs((path_heading - match.point.heading + math.pi)
                                    % (2.0 * math.pi) - math.pi)
                if heading_error > math.radians(35.0):
                    failed = replace(
                        path, valid=False,
                        failure_reason=(
                            "first GPS lane points away from the confirmed "
                            f"truck lane by {math.degrees(heading_error):.1f} "
                            "degrees"))
                    if diagnostics is not None:
                        safe_diagnostic_call(
                            diagnostics, "fail_phase", "LanePath",
                            failed.failure_reason, {
                            "geometry": {
                                "heading_jump_deg": math.degrees(heading_error),
                            },
                        })
                    return failed, match

        # The native GPS buffer can begin at the entrance of a prefab while
        # the truck is already part-way through it. Publishing those points
        # behind the camera made AR draw a giant line across the whole screen.
        # Trim only to the nearest confirmed point and rebuild arc distance;
        # never translate or laterally offset the authoritative geometry.
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
        if diagnostics is not None:
            safe_diagnostic_call(diagnostics, "finish_phase",
                                 "LanePath", details={
                "valid": True,
                "point_count": len(path.points),
                "nearest_truck_distance_m": float(nearest_distance),
            })
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
            legacy_159 = False
            config_path = os.path.join(data_dir, "config.json")
            if os.path.isfile(config_path):
                config = _loadf(config_path)
                version = str(config.get(
                    "game_version_major_minor", config.get("version", "")))
                legacy_159 = version == "1.59" or version.startswith("1.59.")

            def lane_offsets(record, camel, snake):
                raw = record.get(camel, record.get(snake, ())) or ()
                values = []
                for item in raw:
                    value = item[0] if isinstance(item, (list, tuple)) else item
                    value = float(value)
                    if not math.isfinite(value):
                        raise ValueError(f"non-finite {camel} in {record.get('token')}")
                    values.append(value)
                return tuple(values)

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
                left_offsets = lane_offsets(
                    r, "laneOffsetsLeft", "lane_offsets_left")
                right_offsets = lane_offsets(
                    r, "laneOffsetsRight", "lane_offsets_right")
                has_lane_offset_fields = any(key in r for key in (
                    "laneOffsetsLeft", "laneOffsetsRight",
                    "lane_offsets_left", "lane_offsets_right"))
                # The official legacy 1.59 JSON intentionally omitted these
                # arrays. Restore only facts verified from ETS2 1.59 defs; a
                # new TruckLib dataset always remains authoritative.
                if legacy_159 and not has_lane_offset_fields:
                    left_offsets, right_offsets = LANE_OFFSETS_159.get(
                        str(tok).removeprefix("road."), ((), ()))
                self.road_looks[tok] = {
                    "type": rtype, "lanes": max(1, lanes),
                    "lanes_left": len(lanes_l), "lanes_right": len(lanes_r),
                    # These are the lane facts actually present in the map.
                    # Width is absent and is therefore deliberately not stored
                    # as a dataset value (lane construction marks it derived).
                    "lane_types_left": tuple(str(value) for value in lanes_l),
                    "lane_types_right": tuple(str(value) for value in lanes_r),
                    "offset_m": float(r.get("offset", 0.0) or 0.0),
                    "lane_offsets_left_m": left_offsets,
                    "lane_offsets_right_m": right_offsets,
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
    def _offset_curve(curve, lateral_start_m, lateral_end_m=None, reverse=False,
                      start_forward=None, end_forward=None):
        """Offset a sampled road spline with ETS2's road-look transition.

        SCS stores a road's lane offsets at its *end*.  At the start, the
        preceding road item's offsets apply and are interpolated along this
        road.  ETS2LA's ``ParsedRoad.InterpolateLane`` follows the same rule.
        Keeping one fixed offset made adjacent lane centrelines miss at every
        road-look/lane-count transition; the path joiner then exposed that miss
        as the short sideways spikes visible near exits.
        """
        if lateral_end_m is None:
            lateral_end_m = lateral_start_m
        if reverse:
            curve = list(reversed(curve))
            lateral_start_m, lateral_end_m = -lateral_end_m, -lateral_start_m
            start_forward, end_forward = (
                ((-end_forward[0], -end_forward[1])
                 if end_forward is not None else None),
                ((-start_forward[0], -start_forward[1])
                 if start_forward is not None else None),
            )
        result, travelled = [], 0.0
        for index, point in enumerate(curve):
            fraction = index / max(1, len(curve) - 1)
            lateral_m = (lateral_start_m
                         + (lateral_end_m - lateral_start_m) * fraction)
            before = curve[max(0, index - 1)]
            after = curve[min(len(curve) - 1, index + 1)]
            if index == 0 and start_forward is not None:
                dx, dz = start_forward
            elif index == len(curve) - 1 and end_forward is not None:
                dx, dz = end_forward
            elif len(point) >= 5:
                dx, dz = point[3], point[4]
                if reverse:
                    dx, dz = -dx, -dz
            else:
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

    @staticmethod
    def _lane_center_offsets(look):
        """Return signed raw-lane centres using ETS2LA/TruckLib's rules.

        Positive values are right of the map item's start->end centreline;
        negative values are left.  The 1.59 extractor does not retain
        New TruckLib datasets preserve ``lane_offsets_left/right``. Legacy
        1.59 datasets receive the same facts from the versioned compatibility
        table loaded by ``_load_road_looks``.
        """
        width, half = 4.5, 2.25
        left = tuple(look.get("lane_types_left", ()))
        right = tuple(look.get("lane_types_right", ()))
        has_left, has_right = bool(left), bool(right)
        road_center = 0.0
        if has_left and has_right and (len(left) + len(right)) % 2 == 1:
            road_center = (len(left) - len(right)) * width

        right_centers = []
        if has_right:
            if has_left or len(right) == 1:
                first = road_center + half
            elif len(right) % 2 == 1:
                first = road_center - half
            else:
                first = road_center - math.ceil(len(right) / 2.0) * width + half
            right_centers = [first + index * width
                             for index in range(len(right))]

        left_centers = []
        if has_left:
            if has_right:
                first = road_center - half
            elif len(left) == 1:
                first = road_center - half
            elif len(left) % 2 == 1:
                first = road_center + half
            else:
                first = road_center + math.ceil(len(left) / 2.0) * width - half
            left_centers = [first - index * width
                            for index in range(len(left))]

        # roadLooks.json's ``offset`` is the SII road_offset. ETS2LA applies
        # the full value to each side (not half of it).
        road_offset = float(look.get("offset_m", 0.0) or 0.0)
        right_centers = [value + road_offset for value in right_centers]
        left_centers = [value - road_offset for value in left_centers]

        # Match ETS2LA RoadUtils.CalculateRoadLaneCenters exactly: right lane
        # offsets add their X component, left lane offsets subtract it.
        right_offsets = tuple(look.get("lane_offsets_right_m", ()))
        left_offsets = tuple(look.get("lane_offsets_left_m", ()))
        right_centers = [
            value + (float(right_offsets[index])
                     if index < len(right_offsets) else 0.0)
            for index, value in enumerate(right_centers)
        ]
        left_centers = [
            value - (float(left_offsets[index])
                     if index < len(left_offsets) else 0.0)
            for index, value in enumerate(left_centers)
        ]
        return tuple(left_centers), tuple(right_centers)

    def _previous_road_look(self, segment_index):
        """Exact preceding road look, or ``None`` for prefab/ambiguous data."""
        if not (0 <= segment_index < len(self._seg_uids)):
            return None
        start_uid = self._seg_uids[segment_index][0]
        previous_item = self.node_backward_item.get(start_uid, 0)
        previous_index = self._road_segment_by_uid.get(previous_item)
        if previous_index is None or previous_index == segment_index:
            return None
        previous_uids = self._seg_uids[previous_index]
        # The item link must really terminate at this node. This prevents an
        # arbitrary nearby or graph-only branch from influencing lane geometry.
        if previous_uids[1] != start_uid:
            return None
        token = self._seg_look_tokens[previous_index]
        return self.road_looks.get(token)

    def _prefab_boundary_lane_offsets(self, node_uid, road_curve, at_start):
        """Derive road-boundary lane centres from the adjacent prefab PPD.

        SCS road-look offsets describe the road's end. Its start inherits lane
        positions from the preceding map item. For road-to-road transitions
        ``_previous_road_look`` supplies those values; when the preceding item
        is a prefab, the authoritative positions are the input/output navCurve
        endpoints at the shared physical node. ``at_start`` selects whether
        the road leaves or enters that prefab in stored road direction.
        """
        if node_uid not in self.nodes or not road_curve:
            return (), ()
        adjacent_item = (self.node_backward_item.get(node_uid, 0)
                         if at_start else
                         self.node_forward_item.get(node_uid, 0))
        if (not adjacent_item
                or adjacent_item in self._road_segment_by_uid):
            return (), ()
        try:
            tangent = road_curve[0] if at_start else road_curve[-1]
            fx, fz = float(tangent[3]), float(tangent[4])
        except (IndexError, TypeError, ValueError):
            return (), ()
        forward_length = math.hypot(fx, fz)
        if forward_length < 1e-8:
            return (), ()
        fx, fz = fx / forward_length, fz / forward_length
        cache = getattr(self, "_prefab_boundary_offset_cache", None)
        if cache is None:
            cache = self._prefab_boundary_offset_cache = {}
        cache_key = (int(node_uid), bool(at_start),
                     round(fx, 5), round(fz, 5))
        if cache_key in cache:
            return cache[cache_key]
        right_x, right_z = -fz, fx
        nx, nz = self.nodes[node_uid]
        node_y = self.node_alt.get(node_uid, 0.0)
        cx, cz = self._cell(nx, nz)
        instances, seen = [], set()
        for dx in (-1, 0, 1):
            for dz in (-1, 0, 1):
                for instance in self._prefab_grid.get((cx + dx, cz + dz), ()):
                    marker = (instance[0], instance[1])
                    if marker in seen or node_uid not in instance[1]:
                        continue
                    seen.add(marker)
                    instances.append(instance)

        forward_offsets, reverse_offsets = [], []
        for instance in instances:
            lane_data = self._prefab_lane_data.get(instance[0]) or {}
            try:
                node_index = instance[1].index(node_uid)
                prefab_node = (lane_data.get("nodes") or ())[node_index]
            except (ValueError, IndexError, TypeError):
                continue
            # Leaving a prefab into the stored road start uses output curves;
            # entering one at the stored road end uses input curves. The
            # opposite travel direction uses the complementary set.
            if at_start:
                groups = ((prefab_node.get("output_lanes", ()), True, -1),
                          (prefab_node.get("input_lanes", ()), False, 0))
            else:
                groups = ((prefab_node.get("input_lanes", ()), True, 0),
                          (prefab_node.get("output_lanes", ()), False, -1))
            for curve_indices, travels_forward, endpoint_index in groups:
                for curve_index in curve_indices:
                    points = self._prefab_curve_chain_3d(
                        instance, (int(curve_index),))
                    if len(points) < 2:
                        continue
                    point = points[endpoint_index]
                    direction_x = -math.sin(point.heading)
                    direction_z = -math.cos(point.heading)
                    alignment = direction_x * fx + direction_z * fz
                    if ((travels_forward and alignment < 0.85)
                            or (not travels_forward and alignment > -0.85)):
                        continue
                    dxp, dzp = point.x - nx, point.z - nz
                    planar_gap = math.hypot(dxp, dzp)
                    if planar_gap > 24.0 or abs(point.y - node_y) > 2.0:
                        continue
                    offset = dxp * right_x + dzp * right_z
                    target = forward_offsets if travels_forward else reverse_offsets
                    if not any(abs(offset - existing) < 0.20
                               for existing in target):
                        target.append(offset)

        # right lane arrays run from the centre outwards in increasing signed
        # offset; left arrays use the mirrored (decreasing) order.
        forward_offsets.sort()
        reverse_offsets.sort(reverse=True)
        result = tuple(reverse_offsets), tuple(forward_offsets)
        cache[cache_key] = result
        return result

    @staticmethod
    def _match_prefab_offsets(expected, candidates, width=4.5):
        """Map physical prefab offsets to the road's raw-lane prefix."""
        available = list(candidates)
        result = []
        for target in expected:
            if not available:
                break
            best = min(range(len(available)),
                       key=lambda index: abs(available[index] - target))
            if abs(available[best] - target) > width * 1.5:
                break
            result.append(available.pop(best))
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
        curve = self._road_curve_3d(start_uid, end_uid, spacing=3.0,
                                    with_tangents=True)
        if len(curve) < 2:
            self._lane_cache[segment_index] = ()
            return ()
        width = 4.5
        left_offsets, right_offsets = self._lane_center_offsets(look)
        prefab_end_left, prefab_end_right = (
            self._prefab_boundary_lane_offsets(end_uid, curve, False))
        matched_end_left = self._match_prefab_offsets(
            left_offsets, prefab_end_left, width)
        matched_end_right = self._match_prefab_offsets(
            right_offsets, prefab_end_right, width)
        if matched_end_left:
            left_offsets = matched_end_left + left_offsets[len(matched_end_left):]
        if matched_end_right:
            right_offsets = (matched_end_right
                             + right_offsets[len(matched_end_right):])
        previous_look = self._previous_road_look(segment_index)
        if previous_look:
            previous_left, previous_right = self._lane_center_offsets(previous_look)
        else:
            prefab_start_left, prefab_start_right = (
                self._prefab_boundary_lane_offsets(start_uid, curve, True))
            previous_left = self._match_prefab_offsets(
                left_offsets, prefab_start_left, width)
            previous_right = self._match_prefab_offsets(
                right_offsets, prefab_start_right, width)
        groups = ((1, tuple(look.get("lane_types_right", ()))),
                  (-1, tuple(look.get("lane_types_left", ()))))
        built = []
        for direction, lane_types in groups:
            drivable = [(raw_index, lane_type)
                        for raw_index, lane_type in enumerate(lane_types)
                        if self._drivable_lane_type(lane_type)]
            for lane_index, (raw_index, lane_type) in enumerate(drivable):
                end_offsets = right_offsets if direction > 0 else left_offsets
                start_offsets = previous_right if direction > 0 else previous_left
                lateral_end = end_offsets[raw_index]
                lateral_start = (start_offsets[raw_index]
                                 if raw_index < len(start_offsets)
                                 else lateral_end)
                lane_id = LaneId(road_uid, direction, lane_index)
                # Road-look arrays are ordered from the centre outwards.
                # Therefore index-1 is the physical left neighbour and
                # index+1 the physical right neighbour in both directions.
                left = (LaneId(road_uid, direction, lane_index - 1)
                        if lane_index > 0 else None)
                right = (LaneId(road_uid, direction, lane_index + 1)
                         if lane_index + 1 < len(drivable) else None)
                centerline = self._offset_curve(
                    curve, lateral_start, lateral_end,
                    reverse=direction < 0,
                    start_forward=self.node_forward.get(start_uid),
                    end_forward=self.node_forward.get(end_uid))
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
                    raw_lane_index=raw_index,
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
        return [((segment[0][0], segment[0][1]),
                 (segment[1][0], segment[1][1]))
                for segment in self.hud_segments_3d_near(
                    pos, radius, limit, connected_only=False)]

    def live_map_segments_3d_near(self, pos, radius: float = 900.0,
                                  limit: int = 6000, altitude=None):
        """Return the broad, display-only road scene around the truck.

        HUD deliberately keeps only the road component carrying the truck.
        The top-down map must also show nearby streets, ramps and junction
        arms.  This query disables only that visual component filter and
        still uses real road and prefab geometry; it creates no route or
        localisation candidates.
        """
        return self.hud_segments_3d_near(
            pos, radius=radius, limit=limit, altitude=altitude,
            connected_only=False)

    def live_map_road_type(self, path_key, lanes=2, divided=False):
        """Translate the exact road look to truckermudgeon/maps categories."""
        if str(path_key).startswith("p"):
            return "divided" if divided else "local"
        try:
            segment_index = int(str(path_key).split(":", 1)[0][1:])
            token = self._seg_look_tokens[segment_index]
            source_type = str((self.road_looks.get(token) or {}).get(
                "type", "local"))
        except (ValueError, IndexError, TypeError):
            source_type = "local"
        if source_type in ("motorway", "expressway"):
            return "freeway"
        if source_type == "dirt":
            return "no_vehicles"
        if divided or int(lanes or 0) >= 4:
            return "divided"
        return "local"

    def live_map_polygons_near(self, pos, radius: float = 900.0,
                               limit: int = 1200):
        """Return real placed-prefab polygons using maps' neighbour loops."""
        if not pos or not self._prefab_grid or not self._prefab_map_polygons:
            return []
        px, pz = float(pos[0]), float(pos[1])
        cx, cz = self._cell(px, pz)
        rings = int(radius // self.GRID) + 1
        seen = set()
        ranked = []
        for dx in range(-rings, rings + 1):
            for dz in range(-rings, rings + 1):
                for instance in self._prefab_grid.get((cx+dx, cz+dz), ()):
                    marker = (instance[0], instance[1])
                    if marker in seen:
                        continue
                    seen.add(marker)
                    for local_points, colour, z_index in \
                            self._prefab_map_polygons.get(instance[0], ()):
                        points = self._transform_prefab_points(instance, local_points)
                        if len(points) < 3:
                            continue
                        min_distance2 = min(
                            (point[0]-px) ** 2 + (point[1]-pz) ** 2
                            for point in points)
                        inside = (min(point[0] for point in points) <= px <=
                                  max(point[0] for point in points)
                                  and min(point[1] for point in points) <= pz <=
                                  max(point[1] for point in points))
                        if min_distance2 <= radius * radius or inside:
                            ranked.append((min_distance2, tuple(points),
                                           int(colour), int(z_index)))
        ranked.sort(key=lambda item: (item[3], item[0]))
        return [(points, colour, z_index)
                for _distance, points, colour, z_index in ranked[:limit]]

    def _load_map_features(self, data_dir: str):
        """Load compact display-only city, company and POI landmarks."""
        self._map_feature_grid = {}
        self._map_feature_count = 0
        seen_features = set()

        def add(x, z, kind, icon="", label=""):
            try:
                x, z = float(x), float(z)
                if not math.isfinite(x) or not math.isfinite(z):
                    return
            except (TypeError, ValueError, OverflowError):
                return
            kind = str(kind)
            icon = str(icon or "")
            marker = (round(x / 3.0), round(z / 3.0), kind, icon)
            if marker in seen_features:
                return
            seen_features.add(marker)
            feature = (x, z, kind, icon, str(label or ""))
            self._map_feature_grid.setdefault(self._cell(x, z), []).append(feature)
            self._map_feature_count += 1

        try:
            company_names = {}
            defs_path = _find_json(data_dir, "companyDefs")
            if defs_path:
                company_names = {
                    str(item.get("token") or ""): str(item.get("name") or "")
                    for item in _loadf(defs_path)
                    if isinstance(item, dict)
                }

            companies_path = _find_json(data_dir, "companies")
            if companies_path:
                for item in _loadf(companies_path):
                    if not isinstance(item, dict):
                        continue
                    token = str(item.get("token") or "")
                    add(item.get("x"), item.get("y"), "company", token,
                        company_names.get(token) or token.upper())

            pois_path = _find_json(data_dir, "pois")
            if pois_path:
                for item in _loadf(pois_path):
                    if not isinstance(item, dict):
                        continue
                    add(item.get("x"), item.get("y"),
                        str(item.get("type") or "poi"),
                        str(item.get("icon") or ""),
                        str(item.get("label") or ""))

            cities_path = _find_json(data_dir, "cities")
            if cities_path:
                for item in _loadf(cities_path):
                    if not isinstance(item, dict) or item.get("hidden"):
                        continue
                    add(item.get("x"), item.get("y"), "city",
                        str(item.get("token") or ""),
                        str(item.get("name") or item.get("token") or ""))
            logging.info("road_network: loaded %d display map features.",
                         self._map_feature_count)
        except Exception:
            # Landmarks are presentation only.  A malformed optional JSON
            # must never prevent the authoritative network from loading.
            self._map_feature_grid = {}
            self._map_feature_count = 0
            logging.warning("road_network: display map features unavailable",
                            exc_info=True)

    def map_features_near(self, pos, radius: float = 900.0,
                          limit: int = 700):
        """Return bounded display features near ``pos``."""
        if not pos or not self._map_feature_grid:
            return []
        px, pz = float(pos[0]), float(pos[1])
        cx, cz = self._cell(px, pz)
        rings = int(radius // self.GRID) + 1
        radius2 = radius * radius
        ranked = []
        for dx in range(-rings, rings + 1):
            for dz in range(-rings, rings + 1):
                for feature in self._map_feature_grid.get((cx+dx, cz+dz), ()):
                    distance2 = ((feature[0]-px) ** 2
                                 + (feature[1]-pz) ** 2)
                    if distance2 <= radius2:
                        ranked.append((distance2, feature))
        priority = {"city": 0, "company": 1, "facility": 2,
                    "landmark": 3, "viewpoint": 4}
        ranked.sort(key=lambda item: (
            priority.get(item[1][2], 8), item[0]))
        return [feature for _distance, feature in ranked[:limit]]

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
