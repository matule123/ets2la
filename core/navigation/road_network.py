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
        self.adj = {}            # uid -> [connected uid, ...]  (road graph, from roads.json)
        self.fwd = {}            # uid -> [uid, ...]  forward neighbours (graph.json)
        self.bwd = {}            # uid -> [uid, ...]  backward neighbours (graph.json)
        self._ngrid = {}         # (cx,cz) -> [uid, ...]  (node spatial index)
        self.segments = []       # [((x1,z1),(x2,z2)), ...]
        self._seg_uids = []      # [(start_uid, end_uid), ...]  parallel to segments
        self._grid = {}          # (cx,cz) -> [segment_index, ...]  (legacy, endpoint-based)
        self._seg_grid = {}      # (cx,cz) -> [segment_index, ...]  (both endpoints)
        self.road_looks = {}     # token -> {"type": str, "lanes": int}  (road classification)
        self._road_look_token = {}  # node_uid -> roadLookToken (nearest road's type)
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
                uid = n["uid"]
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
                su, eu = r.get("startNodeUid"), r.get("endNodeUid")
                a, b = self.nodes.get(su), self.nodes.get(eu)
                if a and b:
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
            if payload.get("sig") != self._source_signature(data_dir):
                logging.info("road_network: cache stale — rebuilding.")
                return False
            data = payload["data"]
            for k in ("nodes", "adj", "fwd", "bwd", "_ngrid", "segments",
                      "_seg_uids", "_grid", "_seg_grid", "_road_look_token",
                      "road_looks", "loaded"):
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
                "sig": self._source_signature(data_dir),
                "data": {
                    "nodes": self.nodes, "adj": self.adj, "fwd": self.fwd,
                    "bwd": self.bwd, "_ngrid": self._ngrid, "segments": self.segments,
                    "_seg_uids": self._seg_uids, "_grid": self._grid,
                    "_seg_grid": self._seg_grid, "_road_look_token": self._road_look_token,
                    "road_looks": self.road_looks, "loaded": self.loaded,
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
                fw = [e["nodeId"] for e in (data.get("forward") or []) if e.get("nodeId")]
                bw = [e["nodeId"] for e in (data.get("backward") or []) if e.get("nodeId")]
                if fw:
                    self.fwd[uid] = fw
                    nf += 1
                if bw:
                    self.bwd[uid] = bw
                    nb += 1
            logging.info("road_network: nav-graph loaded (%d fwd / %d bwd nodes).", nf, nb)
        except Exception as e:
            logging.warning("road_network: nav-graph load failed (%s) — using roads.json graph.", e)

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
                self.road_looks[tok] = {"type": rtype, "lanes": max(1, lanes)}
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
