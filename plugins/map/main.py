import logging
import os
import math
import time
from sdk.base_plugin import BasePlugin
from core.navigation.route import Route
from core.navigation.lane_trajectory import (
    build_lane_trajectory, derive_display_points,
)
from core.paths import app_dir

# routes/ lives next to the app (works both from source and when frozen).
ROUTES_DIR = os.path.join(app_dir(), "routes")


class Plugin(BasePlugin):
    """
    Coordinate-based navigation plugin.

    Replaces the old stub (which checked a never-set ``NAVIGATING`` state and a
    never-set ``truck_pos`` key, so it did nothing).  This version follows a
    polyline of world waypoints recorded straight from telemetry:

      * **Record** — breadcrumb ``truck_world_pos`` into a :class:`Route`.
      * **Replay** — steer along a loaded route via cross-track + heading error,
        publishing ``nav_steering`` / ``nav_active`` for the Autopilot to use.

    Commands arrive through shared state (set by the UI): ``nav_cmd`` (one of
    ``record`` / ``stop_record`` / ``load`` / ``clear`` / ``stop``) with an
    optional ``nav_arg`` (the route name).  The plugin consumes each command and
    clears ``nav_cmd`` back to ``None``.
    """

    NAME = "map"

    def on_start(self):
        logging.info("Map (navigation) plugin started.")
        self.enabled = True
        self.recording = None        # Route being recorded, or None
        self.active_route = None     # Route being followed, or None
        self.road_net = None         # RoadNetwork loaded from a downloaded map
        self._net_attempted = False  # tried to load the road network this run?
        self._net_loading = False    # background load in progress (don't re-enter)
        self._diag_t = 0.0           # throttle for localization diagnostics
        self._roads_t = 0.0          # throttle nearby-road HUD publishing
        self._last_recalc_request = None
        self._recalc_started = 0.0
        self._last_recalc_stage = None
        self._auto_map_signature = None
        self._auto_map_loading = False
        self._route_orientation_signature = None
        self._route_reversed = False
        self._last_route_progress_log = 0.0
        self._last_navigation_status_log = 0.0
        self._last_geometry_log = 0.0
        self._last_route_join_log = 0.0
        self._resolved_route_cache_signature = None
        self._resolved_route_cache = []
        self._failed_route_signature = None
        self._vision_lane_m = 0.0
        self._lane_signature = None
        self._lane_path = None
        self._lane_route = None
        self._lane_match = None
        self._lane_revision = int(self.sdk.get(
            "lane_trajectory_revision", 0) or 0)
        self._lane_diag_t = 0.0
        os.makedirs(ROUTES_DIR, exist_ok=True)
        self._publish_route_list()

    def on_stop(self):
        logging.info("Map (navigation) plugin stopped.")
        self.sdk.set("nav_active", False)
        self.sdk.set("nav_steering", 0.0)
        self.sdk.set("nav_trajectory_revision", -1)

    @staticmethod
    def _lane_id_payload(lane_id):
        if lane_id is None:
            return None
        return {
            "road_uid": int(lane_id.road_uid),
            "direction": int(lane_id.direction),
            "lane_index": int(lane_id.lane_index),
            "prefab_token": lane_id.prefab_token,
            "connector_index": lane_id.connector_index,
            "connector_path": list(lane_id.connector_path),
        }

    def _next_lane_revision(self):
        shared_revision = int(self.sdk.get(
            "lane_trajectory_revision", 0) or 0)
        self._lane_revision = max(self._lane_revision, shared_revision) + 1
        return self._lane_revision

    @staticmethod
    def _normalise_gps_uids(raw_uids):
        try:
            from core.navigation.road_network import _uid
            return tuple(_uid(uid) for uid in (raw_uids or ()) if _uid(uid))
        except Exception:
            return ()

    def _build_is_current(self, uids, revision, request_id=None):
        return bool(
            self._normalise_gps_uids(
                self.sdk.get("game_route_node_uids", []) or []) == tuple(uids)
            and int(self.sdk.get("lane_trajectory_revision", -1) or -1)
                == int(revision)
            and self.sdk.get("nav_recalc_request") == request_id)

    def _publish_invalid_lane_trajectory(self, reason, uids=(), status=None):
        revision = self._next_lane_revision()
        snapshot = {
            "revision": revision, "valid": False, "confidence": 0.0,
            "active_lane_id": None, "lane_match": None,
            "points": [], "display_points": [], "distance_m": 0.0,
            "failure_reason": str(reason or "Navigačná trajektória nie je platná"),
            "source_gps_uids": [int(uid) for uid in uids],
        }
        self.sdk.shared_state.update_batch({
            "lane_trajectory_revision": revision,
            "lane_trajectory": snapshot,
        })
        self.sdk.set("nav_path", [])
        self.sdk.set("map_path", [])
        self.sdk.set("nav_active", False)
        self.sdk.set("nav_steering", 0.0)
        self.sdk.set("nav_trajectory_revision", -1)
        self.sdk.set("navigation_unreliable", True)
        self.sdk.set("navigation_failure_reason", snapshot["failure_reason"])
        if status:
            self.sdk.set("navigation_status", status)
        self._lane_path = None
        self._lane_route = None
        return snapshot

    def _update_lane_trajectory(self, pos, heading):
        """Build and atomically publish the sole GPS lane trajectory snapshot."""
        raw_uids = self.sdk.get("game_route_node_uids", []) or []
        uids = self._normalise_gps_uids(raw_uids)
        signature = uids
        if signature != self._lane_signature:
            self._lane_signature = signature
            self._lane_match = None
            locator = getattr(self.road_net, "_runtime_lane_locator", None)
            if locator is not None:
                locator.previous = None
            self._publish_invalid_lane_trajectory(
                "Načítavam GPS trasu", uids, "Načítavam GPS trasu")
            self.sdk.set("navigation_recalculating", bool(len(uids) >= 2))
        if len(uids) < 2:
            return None
        if self.road_net is None or not self.road_net.loaded:
            self.sdk.set("navigation_status", "Načítavam GPS trasu")
            return None

        current = self.sdk.get("lane_trajectory", {}) or {}
        build_revision = int(self.sdk.get(
            "lane_trajectory_revision", -1) or -1)
        build_request = self.sdk.get("nav_recalc_request")
        needs_build = not bool(current.get("valid", False))
        # Re-localise on the authoritative lane each tick. A confirmed lane
        # transition triggers a fresh trajectory revision, never a shifted copy.
        altitude = float(self.sdk.get("truck_altitude", 0.0) or 0.0)
        locator = getattr(self.road_net, "_runtime_lane_locator", None)
        if locator is None:
            from core.navigation.lane_model import LaneLocator
            locator = self.road_net._runtime_lane_locator = LaneLocator(self.road_net)
        match = locator.locate((pos[0], altitude, pos[1]), heading, uids,
                               self._lane_match)
        if not self._build_is_current(uids, build_revision, build_request):
            return None
        if match is None:
            self._publish_invalid_lane_trajectory(
                "Kamión sa nepodarilo spoľahlivo lokalizovať na GPS pruhu",
                uids, "Kamión nie je na potvrdenom jazdnom pruhu")
            return None
        if (self._lane_match is not None
                and match.lane_id != self._lane_match.lane_id):
            needs_build = True
        self._lane_match = match
        if not needs_build and self._lane_path is not None:
            # Keep the geometry snapshot immutable. Runtime localization and
            # liveness are published separately under the same revision.
            self.sdk.set("lane_match", {
                "revision": self._lane_revision,
                "active_lane_id": self._lane_id_payload(match.lane_id),
                "point": [float(match.point.x), float(match.point.y),
                          float(match.point.z)],
                "lateral_error_m": float(match.lateral_error_m),
                "heading_error_rad": float(match.heading_error_rad),
                "vertical_error_m": float(match.vertical_error_m),
                "switch_reason": match.switch_reason,
            })
            self.sdk.set("lane_trajectory_heartbeat", time.monotonic())
            return self._lane_path

        self.sdk.set("navigation_status", "Vyberám jazdné pruhy")
        lane_path, _ = self.road_net.build_lane_path(
            uids, pos, heading, altitude=altitude, start_match=match)
        if not self._build_is_current(uids, build_revision, build_request):
            return None
        if not lane_path.valid:
            self._publish_invalid_lane_trajectory(
                lane_path.failure_reason, uids, lane_path.failure_reason)
            return None
        self.sdk.set("navigation_status", "Vytváram trajektóriu")
        trajectory = build_lane_trajectory(lane_path, spacing_m=2.0)
        if not trajectory.valid:
            self._publish_invalid_lane_trajectory(
                trajectory.failure_reason, uids, trajectory.failure_reason)
            return None
        display = derive_display_points(trajectory, spacing_m=4.0)
        if len(display) < 2:
            self._publish_invalid_lane_trajectory(
                "Trajektória nemá bezpečné display body", uids,
                "Trajektória nemá bezpečné display body")
            return None
        if not self._build_is_current(uids, build_revision, build_request):
            return None
        revision = self._next_lane_revision()
        control_points = [[float(p.x), float(p.y), float(p.z)]
                          for p in trajectory.points]
        # Phase 4 requires controller, HUD and AR to consume geometrically
        # identical authoritative points. A redrawn 4 m chord can deviate from
        # a curved 2 m polyline, so publish the validated control samples to all
        # three consumers. ``derive_display_points`` remains an offline/API
        # facility but is not a second runtime geometry.
        display_points = [list(point) for point in control_points]
        snapshot = {
            "revision": revision, "valid": True,
            "confidence": float(min(trajectory.confidence, match.confidence)),
            "active_lane_id": self._lane_id_payload(match.lane_id),
            "lane_match": {
                "point": [float(match.point.x), float(match.point.y),
                          float(match.point.z)],
                "lateral_error_m": float(match.lateral_error_m),
                "heading_error_rad": float(match.heading_error_rad),
                "vertical_error_m": float(match.vertical_error_m),
                "switch_reason": match.switch_reason,
            },
            "points": control_points, "display_points": display_points,
            "distance_m": float(trajectory.distance_m), "failure_reason": "",
            "source_gps_uids": [int(uid) for uid in uids],
            "request_id": build_request,
        }
        # One shared-state assignment publishes one coherent geometry revision.
        self.sdk.shared_state.update_batch({
            "lane_trajectory_revision": revision,
            "lane_trajectory": snapshot,
            "nav_path": display_points,
            "map_path": control_points,
            "nav_trajectory_revision": revision,
            "lane_trajectory_heartbeat": time.monotonic(),
        })
        self.sdk.set("navigation_unreliable", False)
        self.sdk.set("navigation_failure_reason", "")
        self.sdk.set("navigation_recalculating", False)
        self.sdk.set("navigation_status", "Navigácia pripravená")
        self._lane_path = trajectory
        self._lane_route = Route(control_points, name="gps-lane-trajectory")
        return trajectory

    # --- Helpers --------------------------------------------------------------
    def _publish_route_list(self):
        try:
            names = sorted(f[:-5] for f in os.listdir(ROUTES_DIR) if f.endswith(".json"))
        except Exception:
            names = []
        self.sdk.set("nav_routes", names)

    @staticmethod
    def _driving_line(points, offset):
        """Return the visible lane-centre line used by the controller.

        HUD and AR must show the same right-of-centre target that steering
        follows, not the raw road centre through a median.
        """
        points = [tuple(p[:2]) for p in points]
        if len(points) < 2 or abs(offset) < 0.05:
            return points
        shifted = []
        for index, point in enumerate(points):
            a = points[max(0, index - 1)]
            b = points[min(len(points) - 1, index + 1)]
            dx, dz = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dz)
            if length < 0.1:
                shifted.append(point)
            else:
                shifted.append((point[0] - dz / length * offset,
                                point[1] + dx / length * offset))
        return shifted

    @staticmethod
    def _distance_window(points, metres=220.0):
        """Keep a physical look-ahead distance, independent of point density."""
        points = list(points)
        if len(points) < 2:
            return points
        result, travelled = [points[0]], 0.0
        for point in points[1:]:
            travelled += math.dist(tuple(result[-1][:2]), tuple(point[:2]))
            result.append(point)
            if travelled >= metres:
                break
        return result

    def _handle_command(self, pos):
        cmd = self.sdk.get("nav_cmd")
        if not cmd:
            return
        arg = self.sdk.get("nav_arg") or "route"
        self.sdk.set("nav_cmd", None)

        if cmd == "record":
            self.recording = Route(name=arg)
            if pos:
                self.recording.add_point(pos[0], pos[1])
            logging.info("Navigation: started recording '%s'.", arg)

        elif cmd == "stop_record":
            if self.recording and len(self.recording) >= 2:
                path = os.path.join(ROUTES_DIR, f"{self.recording.name}.json")
                self.recording.save(path)
                logging.info("Navigation: saved route '%s' (%d points).",
                             self.recording.name, len(self.recording))
                self._publish_route_list()
            self.recording = None

        elif cmd == "load":
            path = os.path.join(ROUTES_DIR, f"{arg}.json")
            try:
                self.active_route = Route.load(path)
                logging.info("Navigation: loaded route '%s' (%d points).",
                             arg, len(self.active_route))
                self.sdk.set("tts_message", f"Route {arg} loaded. Navigation active.")
            except Exception as e:
                logging.error("Navigation: failed to load '%s': %s", arg, e)
                self.active_route = None

        elif cmd in ("clear", "stop"):
            self.active_route = None
            self.sdk.set("nav_active", False)
            self.sdk.set("nav_steering", 0.0)
            logging.info("Navigation: stopped.")

        elif cmd == "switch_map":
            self.road_net = None
            self._net_attempted = False
            self._net_loading = False
            self.sdk.set("active_map_key", None)
            self.sdk.set("active_map_name", None)
            self.sdk.set("map_path", [])
            self.sdk.set("nav_active", False)
            self.sdk.set("map_status", f"Loading map dataset {arg}...")
            logging.info("Navigation: switching map dataset to %s.", arg)

    def _load_road_net(self):
        """Load the downloaded road network once, in the background (non-blocking).

        The full ETS2 map is ~1.1 M nodes / 250 k segments and takes ~20 s to
        parse, so we must NOT do it on the engine tick thread (that would freeze
        the whole autopilot).  Instead we kick off a worker thread once; while it
        runs the truck keeps its current safe state. The network resolves the
        node UIDs supplied by the in-game GPS; it never invents a route.
        """
        if self.road_net is not None and self.road_net.loaded:
            return
        if self._net_attempted or self._net_loading:
            return
        self._net_attempted = True
        self._net_loading = True
        try:
            import threading

            def _worker():
                try:
                    from core.navigation import map_data
                    from core.navigation.road_network import RoadNetwork
                    from core.settings.manager import SettingsManager
                    datasets = map_data.list_datasets()
                    downloaded = [d for d in datasets if d["downloaded"]]
                    if not downloaded:
                        self.sdk.set("map_status", "No map downloaded yet.")
                        return
                    # Choose the map: prefer the user's last selection (settings),
                    # otherwise fall back to the first downloaded dataset.
                    sm = SettingsManager()
                    wanted = (sm.get("selected_map") or "").strip()
                    chosen = next((d for d in downloaded if d["key"] == wanted), None)
                    if chosen is None:
                        chosen = downloaded[0]
                    self.sdk.set("active_map_key", chosen["key"])
                    self.sdk.set("active_map_name",
                                 chosen.get("name") or chosen["key"])
                    self.sdk.set("map_status",
                                 f"Loading road network ({chosen['key']})…")
                    net = RoadNetwork()
                    if net.load(map_data.dataset_dir(chosen["key"])):
                        self.road_net = net
                        self.sdk.set("map_status",
                                     f"Map ready ({len(net.segments)} segments). "
                                     "Waiting for the in-game GPS route.")
                        logging.info("Navigation: road network loaded engine-side "
                                     "(%d segments, key=%s).", len(net.segments), chosen["key"])
                    else:
                        # Allow a retry on the next run, not this one.
                        self._net_attempted = False
                        self.sdk.set("map_status",
                                     "Map data unreadable — will retry.")
                except Exception as e:
                    logging.error("Navigation: engine-side road network load failed: %s", e)
                    self.sdk.set("map_status", f"Map load error: {e}")
                finally:
                    self._net_loading = False

            threading.Thread(target=_worker, name="RoadNetLoader", daemon=True).start()
        except Exception as e:
            logging.error("Navigation: could not start road network loader: %s", e)
            self._net_loading = False

    def _lane_offset(self):
        """How far (metres) to drive to the RIGHT of the road centreline.

        ETS2 is right-hand traffic, so the autopilot must hold the right lane —
        driving the bare centreline put it in the oncoming lane („protismer").

        The full lateral strategy — right-lane baseline, lane-change requests,
        AND the adaptive trailer-aware swing-wide nudge — is owned by the
        **drivepolicy** plugin, which publishes ``drive_lane_offset``. We prefer
        that when present (it's the coherent combined plan). Fallbacks, in order:
        a manual ``lane_offset_m`` override, then the 2.7 m right-lane default.
        This keeps the map plugin a geometry follower, not a strategist."""
        drv = self.sdk.get("drive_lane_offset", None)
        if drv is not None:
            try:
                # The map graph is commonly the centre line of the whole road.
                # Keep the truck in the right-hand lane, but never accept a
                # large transient lane-change offset while following GPS.
                return max(-2.2, min(2.2, float(drv)))
            except (TypeError, ValueError):
                pass
        v = self.sdk.get("lane_offset_m", None)
        if v is not None:
            try:
                return float(v)
            except (TypeError, ValueError):
                pass
        # 1.8 m is one half-lane: enough to stay out of the centre/median while
        # remaining safe on narrow roads and exact prefab lane curves.
        return 1.8

    def _publish_road_type(self, pos):
        """Classify the road under the truck and publish a speed cap.

        Slows the autopilot on narrow/local/dirt sectors (the „poľné / úzke
        cesty" behaviour) while leaving motorways at full speed. ACC reads
        ``road_speed_cap`` (km/h) and never exceeds it. Cheap no-op when the
        road network isn't loaded yet."""
        net = self.road_net
        if net is None or not getattr(net, "loaded", False) or not pos:
            return
        rt = net.road_type_at(pos)
        if not rt:
            return
        rtype = rt.get("type", "local")
        lanes = rt.get("lanes", 1)
        # Speed caps (km/h) per road class — tuned for a truck. Narrow/dirt
        # sectors cap much lower than the posted limit would, because a truck
        # physically can't take a single-lane dirt road at 90.
        caps = {
            "motorway": 90,
            "expressway": 80,
            "local": 60 if lanes >= 2 else 50,
            "dirt": 35,
        }
        cap = caps.get(rtype, 70)
        prev = self.sdk.get("road_speed_cap", None)
        # Only publish when it changes, to avoid spamming shared state every tick.
        if prev != cap:
            self.sdk.set("road_speed_cap", cap)
            self.sdk.set("road_type", rtype)
            self.sdk.set("road_lanes", lanes)
            logging.info("Road type: %s (%d lanes) -> speed cap %d km/h", rtype, lanes, cap)

    def _resolved_game_route(self, pos, heading):
        """Resolve the in-game GPS node UIDs through the active map dataset."""
        uids = self.sdk.get("game_route_node_uids", []) or []
        if len(uids) < 2 or self.road_net is None or not self.road_net.loaded:
            return []
        from core.navigation.road_network import _uid
        normalized_uids = [_uid(uid) for uid in uids]
        meta = self.sdk.get("game_route_meta", []) or []
        valid_entries = []
        for item_index, uid in enumerate(normalized_uids):
            if uid not in self.road_net.nodes:
                continue
            distance = 0.0
            if item_index < len(meta) and isinstance(meta[item_index], dict):
                try:
                    distance = float(meta[item_index].get("distance", 0.0) or 0.0)
                except (TypeError, ValueError):
                    distance = 0.0
            valid_entries.append((uid, distance))
        all_valid = [entry[0] for entry in valid_entries]
        valid_distances = [entry[1] for entry in valid_entries]
        # Missing IDs must split the route. Compressing them out connected the
        # preceding and following roads with a fake, kilometres-long chord.
        runs, current = [], []
        for uid in normalized_uids:
            if uid in self.road_net.nodes:
                current.append(uid)
            else:
                if current:
                    runs.append(current)
                current = []
        if current:
            runs.append(current)
        viable = [run for run in runs if len(run) >= 2]
        # Keep every UID that exists in the selected dataset, in native GPS
        # order. Choosing only the run nearest the truck could retain a 0.5 km
        # fragment from a 9.8 km route. refine_route() safely fills gaps
        # between surviving UIDs through the directed road graph.
        valid_uids = all_valid
        matched = [tuple(self.road_net.nodes[uid]) for uid in valid_uids]
        match_ratio = len(all_valid) / max(1, len(uids))
        self.sdk.set("game_route_match", match_ratio)
        if len(matched) < 2 or match_ratio < 0.55:
            self.sdk.set("navigation_unreliable", True)
            self.sdk.set(
                "map_status",
                "Vybraná mapa nezodpovedá trase v hre. Vyber správny ETS2/ATS dataset.")
            self.sdk.set("game_route_points", [])
            self._auto_select_matching_map(uids)
            return []
        self.sdk.set("navigation_unreliable", False)

        # ETS2LA route metadata is ordered by decreasing remaining distance:
        # first item is near the truck, last item is the destination. Prefer
        # that authoritative order; heading-only guessing selected the wrong
        # direction on roundabouts and parallel roads.
        route = Route(matched)
        idx = route.tracking_index(pos, heading)
        fx, fz = -math.sin(heading), -math.cos(heading)

        def direction_score(target_index):
            if not (0 <= target_index < len(matched)):
                return -float("inf")
            dx = matched[target_index][0] - matched[idx][0]
            dz = matched[target_index][1] - matched[idx][1]
            length = math.hypot(dx, dz) or 1.0
            return (dx * fx + dz * fz) / length

        orientation_signature = (len(uids), int(uids[0]), int(uids[-1]))
        if orientation_signature != self._route_orientation_signature:
            self._route_orientation_signature = orientation_signature
            distances = [float(item.get("distance", 0.0) or 0.0)
                         for item in meta if isinstance(item, dict)]
            if len(distances) >= 2 and abs(distances[0] - distances[-1]) > 1.0:
                self._route_reversed = distances[0] < distances[-1]
            else:
                self._route_reversed = direction_score(idx - 1) > direction_score(idx + 1)
        if self._route_reversed:
            matched.reverse()
            valid_uids.reverse()
            valid_distances.reverse()
            route = Route(matched)
            idx = route.tracking_index(pos, heading)
        # Route metadata distances remain tied to their SDK nodes even while
        # the buffer itself updates only in sparse steps. Use the live SCS GPS
        # distance to select our progress along that ordered list. This avoids
        # snapping to a geometrically nearby occurrence near the destination
        # on loops/parallel roads (the source of 0.6 km vs 11 km routes).
        try:
            game_distance = float(self.sdk.get("game_route_distance", 0.0) or 0.0)
        except (TypeError, ValueError):
            game_distance = 0.0
        usable_distance_indices = [
            item_index for item_index, distance in enumerate(valid_distances)
            if distance > 0.0]
        if game_distance > 100.0 and len(usable_distance_indices) >= 2:
            metadata_idx = min(
                usable_distance_indices,
                key=lambda item_index: abs(valid_distances[item_index]
                                           - game_distance))
            metadata_error = abs(valid_distances[metadata_idx] - game_distance)
            metadata_span = max(valid_distances) - min(valid_distances)
            if (metadata_span > 500.0
                    and metadata_error <= max(5000.0, game_distance * .35)):
                if metadata_idx != idx:
                    logging.info(
                        "Navigation: GPS distance selected route node %d instead of geometric node %d "
                        "(SDK %.1f km, game %.1f km).",
                        metadata_idx, idx, valid_distances[metadata_idx] / 1000.0,
                        game_distance / 1000.0)
                idx = metadata_idx
        # GPS UIDs are intentionally sparse. The truck will commonly be
        # between the nearest UID and its predecessor; starting exactly at
        # ``idx`` then made the resolved geometry begin tens of metres ahead
        # (for example 56 m) and the safety localisation rejected it. Keep the
        # preceding SDK section, refine it into its real road curve, and let
        # the segment projection below trim the result at the truck position.
        route_start = max(0, idx - 1)
        remaining_uids = valid_uids[route_start:]
        route_snap_point = None
        # Join the truck's exact position to the SDK route through the real
        # directed road graph. Merely retaining an earlier sparse GPS UID is
        # insufficient on long road sectors, where its curve can still start
        # 50+ metres away from the truck.
        snap_point, start_uid, _dir_x, _dir_z = self.road_net._locate_on_road(
            pos, heading)
        if snap_point is not None and start_uid is not None:
            best_join = None
            start_candidates = [start_uid]
            # At divided roads the heading-selected endpoint can have no legal
            # directed connection to the sparse SDK route. Try both endpoints
            # of the actual segment occupied by the truck.
            seg_index = self.road_net._nearest_segment_index(pos)
            if seg_index is not None:
                for endpoint in self.road_net._seg_uids[seg_index]:
                    if endpoint not in start_candidates:
                        start_candidates.append(endpoint)
            # Never search for the geometrically closest one among several
            # future GPS UIDs.  On loops, roundabouts and parallel motorway
            # carriageways a UID close to the truck can actually be many
            # kilometres farther along the route.  Selecting it used to cut a
            # 15 km game route down to about 0.8 km.  ``route_start`` was
            # already selected from the SDK remaining-distance metadata, so
            # only join to that authoritative route position.
            target_index = route_start
            target_uid = valid_uids[target_index]
            for candidate_start in start_candidates:
                bridge = self.road_net._route_bridge(
                    candidate_start, target_uid, max_expanded=24000)
                if not bridge:
                    continue
                bridge_length = sum(
                    math.dist(self.road_net.nodes[a], self.road_net.nodes[b])
                    for a, b in zip(bridge, bridge[1:]))
                approach = math.dist(
                    snap_point, self.road_net.nodes[candidate_start])
                total_length = approach + bridge_length
                if best_join is None or total_length < best_join[0]:
                    best_join = (total_length, target_index, bridge)
            if best_join is not None:
                _join_length, target_index, bridge = best_join
                remaining_uids = bridge + valid_uids[target_index + 1:]
                route_snap_point = tuple(snap_point)
                now = time.monotonic()
                if now - getattr(self, "_last_route_join_log", 0.0) >= 5.0:
                    self._last_route_join_log = now
                    logging.info(
                        "Navigation: truck snapped to road graph; "
                        "joining GPS route over %.1f m.", _join_length)
        cache_signature = tuple(remaining_uids)
        if cache_signature == self._failed_route_signature:
            return []

        def route_progress(done, total, expanded):
            if (not self.sdk.get("navigation_recalculating", False)
                    or self._last_recalc_request is None
                    or self._recalc_started <= 0.0):
                return
            fraction = done / max(1, total)
            self.sdk.set("navigation_progress", 0.72 + 0.24 * fraction)
            detail = (f" · kontrolujem {expanded} uzlov"
                      if expanded else "")
            self.sdk.set(
                "navigation_status",
                f"Spájam cestný úsek {done}/{total}{detail}")
            now = time.monotonic()
            if now - self._last_route_progress_log >= 1.0:
                self._last_route_progress_log = now
                message = (f"Výpočet trasy: úsek {done}/{total}, "
                           f"prehľadaných {expanded} uzlov, "
                           f"čas {now - self._recalc_started:.1f} s")
                logging.info(message)
                self.sdk.set("navigation_log_event", {
                    "seq": time.time_ns(), "level": "INFO", "message": message})

        if cache_signature == self._resolved_route_cache_signature:
            remaining = list(self._resolved_route_cache)
        else:
            remaining = self.road_net.refine_route(
                remaining_uids, progress=route_progress)
            if not getattr(self.road_net, "_last_refine_complete", True):
                self.sdk.set("game_route_resolved_points", len(remaining))
                self.sdk.set("game_route_points", [])
                reason = (getattr(self.road_net, "_last_refine_error", "")
                          or "neznáma chyba spojenia cestnej siete")
                self.sdk.set("navigation_failure_reason", reason)
                logging.error("Navigation geometry failed: %s (%d partial points).",
                              reason, len(remaining))
                self._failed_route_signature = cache_signature
                return []
            self._resolved_route_cache_signature = cache_signature
            self._resolved_route_cache = list(remaining)
            self._failed_route_signature = None
            self.sdk.set("navigation_failure_reason", "")
        if len(remaining) < 2:
            remaining = matched[idx:]
        if (route_snap_point is not None and remaining
                and math.dist(route_snap_point, remaining[0]) <= 180.0):
            # The snap point and start_uid lie on the same physical road
            # segment, so this is a valid short partial segment, not an
            # off-road straight-line shortcut.
            first = remaining[0]
            gap = math.dist(route_snap_point, first)
            steps = max(1, int(math.ceil(gap / 8.0)))
            partial = [(
                route_snap_point[0]
                + (first[0] - route_snap_point[0]) * step / steps,
                route_snap_point[1]
                + (first[1] - route_snap_point[1]) * step / steps,
            ) for step in range(steps)]
            remaining = partial + remaining
        # Final safety net for both rendering and steering: never publish a
        # discontinuous route across unrelated map sectors.
        continuous = [remaining[0]] if remaining else []
        for point in remaining[1:]:
            if math.dist(continuous[-1], point) > 350.0:
                break
            continuous.append(point)
        remaining = continuous
        # Localise the truck on an ACTUAL route segment. The former path_ahead
        # join selected an arbitrary nearby branch and then drew/steered across
        # medians and roundabout centres. A route farther than one road width,
        # pointing backwards, or containing a long chord is unsafe and is not
        # published at all.
        fx, fz = -math.sin(heading), -math.cos(heading)
        best = None
        for ri, (a, b) in enumerate(zip(remaining, remaining[1:])):
            vx, vz = b[0] - a[0], b[1] - a[1]
            length2 = vx * vx + vz * vz
            if length2 < 0.04:
                continue
            t = max(0.0, min(1.0,
                    ((pos[0] - a[0]) * vx + (pos[1] - a[1]) * vz) / length2))
            q = (a[0] + vx * t, a[1] + vz * t)
            distance = math.dist(pos, q)
            alignment = (vx * fx + vz * fz) / math.sqrt(length2)
            # Strongly reject the opposite carriageway/roundabout arm.
            if alignment < -0.15:
                continue
            score = distance + max(0.0, 0.35 - alignment) * 8.0
            if best is None or score < best[0]:
                best = (score, distance, ri, q, alignment)
        if best is None or best[1] > 12.0:
            distance = best[1] if best else float("inf")
            reason = ("GPS trasa nie je na ceste pri kamione"
                      if not math.isfinite(distance)
                      else f"GPS trasa je {distance:.1f} m od kamiona")
            self.sdk.set("navigation_failure_reason", reason)
            self.sdk.set("game_route_points", [])
            self.sdk.set("nav_path", [])
            self.sdk.set("nav_active", False)
            self.sdk.set("navigation_unreliable", True)
            logging.error("Navigation rejected for safety: %s.", reason)
            return []
        _score, _distance, ri, projected, _alignment = best
        remaining = [projected] + remaining[ri + 1:]
        # Remove a single short backtracking node from otherwise continuous
        # GPS geometry. Prefab joins occasionally contain A->B->C where B is a
        # duplicate/reversed connector; rejecting the complete route produced
        # false 153-degree errors on a visually straight road.
        cleaned = list(remaining)
        cleaned_changed = True
        while cleaned_changed and len(cleaned) >= 3:
            cleaned_changed = False
            for point_index in range(1, len(cleaned) - 1):
                a, b, c = (cleaned[point_index - 1], cleaned[point_index],
                           cleaned[point_index + 1])
                ab = (b[0] - a[0], b[1] - a[1])
                bc = (c[0] - b[0], c[1] - b[1])
                ab_len, bc_len = math.hypot(*ab), math.hypot(*bc)
                if ab_len < 0.2 or bc_len < 0.2:
                    del cleaned[point_index]
                    cleaned_changed = True
                    break
                cosine = max(-1.0, min(1.0,
                    (ab[0] * bc[0] + ab[1] * bc[1]) / (ab_len * bc_len)))
                turn = math.degrees(math.acos(cosine))
                shortcut = math.dist(a, c)
                if turn > 120.0 and min(ab_len, bc_len) <= 24.0 and shortcut <= 40.0:
                    logging.info(
                        "Navigation: removed reversed prefab connector (%.0f degrees).",
                        turn)
                    del cleaned[point_index]
                    cleaned_changed = True
                    break
        remaining = cleaned
        # Native GPS/map nodes describe the carriageway reference line, which
        # is often the median/road centre rather than the centre of the lane
        # occupied by the truck.  Measure the truck's real lateral displacement
        # from the selected route segment and carry that lane centre through
        # the remaining path.  HUD, AR and steering then share one line that
        # starts underneath the truck instead of several metres to its left.
        if len(remaining) >= 2:
            vx = remaining[1][0] - remaining[0][0]
            vz = remaining[1][1] - remaining[0][1]
            segment_length = math.hypot(vx, vz)
            if segment_length > 0.2:
                normal_x, normal_z = -vz / segment_length, vx / segment_length
                lane_shift = ((pos[0] - projected[0]) * normal_x
                              + (pos[1] - projected[1]) * normal_z)
                # More than one normal lane width means localisation selected
                # the wrong carriageway; do not hide that with a huge shift.
                lane_shift = max(-5.25, min(5.25, lane_shift))
                # Vision supplies the missing within-lane position. Positive
                # lane_offset means the lane centre is to the truck's left.
                # Convert it to metres and blend toward that centre farther
                # ahead, while keeping the first point exactly under the cab.
                try:
                    vision_offset = float(self.sdk.get("lane_offset", 0.0) or 0.0)
                except (TypeError, ValueError):
                    vision_offset = 0.0
                target_lateral = max(-2.2, min(2.2, -vision_offset * 8.0))
                # Lane perception contains frame-to-frame pixel noise. Never
                # feed it directly into world geometry: hold tiny changes and
                # slew larger real movement slowly so a stationary-in-lane
                # truck cannot oscillate across the HUD/controller path.
                lane_error = target_lateral - self._vision_lane_m
                vehicle_speed = abs(float(
                    self.sdk.get("truck_speed_ms", 0.0) or 0.0))
                if vehicle_speed >= 0.4 and abs(lane_error) >= 0.14:
                    self._vision_lane_m += max(-0.025, min(0.025, lane_error))
                desired_lateral = self._vision_lane_m
                truck_right_x, truck_right_z = math.cos(heading), -math.sin(heading)
                normal_screen_sign = (normal_x * truck_right_x
                                      + normal_z * truck_right_z)
                vision_shift = (desired_lateral / normal_screen_sign
                                if abs(normal_screen_sign) > 0.35 else 0.0)
                aligned = []
                travelled = 0.0
                for point_index, point in enumerate(remaining):
                    before = remaining[max(0, point_index - 1)]
                    after = remaining[min(len(remaining) - 1, point_index + 1)]
                    local_x = after[0] - before[0]
                    local_z = after[1] - before[1]
                    local_length = math.hypot(local_x, local_z)
                    if local_length <= 0.2:
                        aligned.append(tuple(point))
                        continue
                    # Recompute the normal at every point. A single world-space
                    # translation is only correct on a straight; through a bend
                    # it cuts across lanes and eventually reaches the median.
                    local_normal_x = -local_z / local_length
                    local_normal_z = local_x / local_length
                    if point_index:
                        travelled += math.dist(remaining[point_index - 1], point)
                    centre_blend = min(1.0, travelled / 32.0)
                    total_shift = lane_shift + vision_shift * centre_blend
                    aligned.append((
                        point[0] + local_normal_x * total_shift,
                        point[1] + local_normal_z * total_shift,
                    ))
                remaining = aligned
                # Remove the tiny residual created when the closest point was
                # an endpoint.  This also guarantees zero initial CTE when the
                # autopilot is enabled on a correctly occupied lane.
                if math.dist(pos, remaining[0]) <= 2.5:
                    remaining[0] = tuple(pos)
                self.sdk.set("navigation_lane_alignment_m", lane_shift)
        previous_vector = None
        previous_length = None
        route_length = 0.0
        for a, b in zip(remaining, remaining[1:]):
            vx, vz = b[0] - a[0], b[1] - a[1]
            segment_length = math.hypot(vx, vz)
            route_length += segment_length
            if math.dist(a, b) > 40.0:
                reason = f"nespojity usek GPS trasy {math.dist(a, b):.1f} m"
                self.sdk.set("navigation_failure_reason", reason)
                self.sdk.set("game_route_points", [])
                self.sdk.set("nav_path", [])
                self.sdk.set("nav_active", False)
                self.sdk.set("navigation_unreliable", True)
                logging.error("Navigation rejected for safety: %s.", reason)
                return []
            if segment_length > 0.8:
                vector = (vx / segment_length, vz / segment_length)
                if previous_vector is not None:
                    dot = max(-1.0, min(1.0,
                              previous_vector[0] * vector[0]
                              + previous_vector[1] * vector[1]))
                    turn_degrees = math.degrees(math.acos(dot))
                    # A connected map graph can legitimately contain an
                    # approximately 90-degree turn at a junction. Reject it
                    # only when both arms are long (a fake chord between
                    # unrelated roads), or when the reversal is extreme.
                    long_corner = (previous_length is not None
                                   and min(previous_length, segment_length) > 24.0)
                    if turn_degrees > 125.0 or (turn_degrees > 72.0
                                                and long_corner):
                        reason = f"neplatny skok smeru GPS trasy {turn_degrees:.0f} stupnov"
                        self.sdk.set("navigation_failure_reason", reason)
                        self.sdk.set("game_route_points", [])
                        self.sdk.set("nav_path", [])
                        self.sdk.set("nav_active", False)
                        self.sdk.set("navigation_unreliable", True)
                        logging.error("Navigation rejected for safety: %s.", reason)
                        return []
                previous_vector = vector
                previous_length = segment_length
        game_distance = float(self.sdk.get("game_route_distance", 0.0) or 0.0)
        # SCS reports routeDistance in real-world metres, while the map node
        # coordinates use ETS2/ATS' compressed world.  Outside cities the
        # scale is roughly 1:19 (and differs in cities), so comparing these
        # values directly produced the persistent false error
        # "map 0.8 km, game GPS 15 km".  The SDK value is authoritative for
        # the UI; the coordinate length is only useful for detecting a wildly
        # implausible/incomplete geometry.
        world_scale = (game_distance / route_length
                       if game_distance > 100.0 and route_length > 1.0 else 0.0)
        self.sdk.set("game_route_world_scale", world_scale)
        if game_distance > 100.0 and (world_scale < 0.45 or world_scale > 35.0):
            reason = (f"neuplna geometria GPS trasy "
                      f"(mierka 1:{world_scale:.1f})")
            self.sdk.set("navigation_failure_reason", reason)
            self.sdk.set("game_route_points", [])
            self.sdk.set("nav_path", [])
            self.sdk.set("nav_active", False)
            self.sdk.set("navigation_unreliable", True)
            logging.error("Navigation rejected for safety: %s.", reason)
            return []
        now = time.monotonic()
        if (game_distance > 100.0
                and now - getattr(self, "_last_geometry_log", 0.0) >= 5.0):
            self._last_geometry_log = now
            logging.info(
                "Navigation geometry ready: %.3f map km = %.3f GPS km "
                "(world scale 1:%.2f).",
                route_length / 1000.0, game_distance / 1000.0, world_scale)
        # Publish a consistently dense route. SDK UIDs can be kilometres
        # apart, while HUD rendering and steering need regular local samples.
        dense_remaining = []
        for start, end in zip(remaining, remaining[1:]):
            distance = math.dist(start, end)
            samples = max(1, int(math.ceil(distance / 4.0)))
            for sample in range(samples):
                fraction = sample / samples
                point = (start[0] + (end[0] - start[0]) * fraction,
                         start[1] + (end[1] - start[1]) * fraction)
                if not dense_remaining or math.dist(dense_remaining[-1], point) > .15:
                    dense_remaining.append(point)
        if remaining:
            dense_remaining.append(remaining[-1])
        remaining = dense_remaining
        self.sdk.set("navigation_unreliable", False)
        self.sdk.set("game_route_resolved_points", len(remaining))
        self.sdk.set("game_route_points", [list(p) for p in remaining])
        return remaining

    def _auto_select_matching_map(self, uids):
        """Background-select a downloaded dataset matching the live GPS UIDs."""
        signature = (len(uids), tuple(uids[:3]), tuple(uids[-3:]))
        if self._auto_map_loading or signature == self._auto_map_signature:
            return
        self._auto_map_signature = signature
        self._auto_map_loading = True

        def worker():
            try:
                from core.navigation import map_data
                from core.navigation.road_network import RoadNetwork
                from core.settings.manager import SettingsManager
                active = self.sdk.get("active_map_key")
                best = (0.0, None, None)
                for dataset in map_data.list_datasets():
                    if not dataset.get("downloaded") or dataset.get("key") == active:
                        continue
                    candidate = RoadNetwork()
                    if not candidate.load(map_data.dataset_dir(dataset["key"])):
                        continue
                    from core.navigation.road_network import _uid
                    ratio = sum(1 for uid in uids if _uid(uid) in candidate.nodes) / len(uids)
                    if ratio > best[0]:
                        best = (ratio, dataset, candidate)
                    if ratio >= 0.92:
                        break
                ratio, dataset, candidate = best
                if dataset is not None and ratio >= 0.55:
                    self.road_net = candidate
                    key = dataset["key"]
                    SettingsManager().set("selected_map", key)
                    self.sdk.set("active_map_key", key)
                    self.sdk.set("active_map_name", dataset.get("name") or key)
                    self.sdk.set("map_status", f"Automaticky zvolená kompatibilná mapa: {key}")
                    logging.info("Navigation: auto-selected %s (GPS UID match %.0f%%).",
                                 key, ratio * 100)
                else:
                    self.sdk.set(
                        "map_status",
                        "Žiadna stiahnutá mapa nezodpovedá GPS trase v hre.")
            except Exception as error:
                logging.warning("Navigation: automatic map matching failed: %s", error)
            finally:
                self._auto_map_loading = False

        import threading
        threading.Thread(target=worker, name="MapAutoMatcher", daemon=True).start()

    def _ensure_map_path(self, pos, heading):
        """Compute and publish the road-ahead polyline from the downloaded map.

        Falls back to whatever the UI process publishes as ``map_path`` if the
        engine-side network isn't available yet.
        """
        # Prefer the actual route selected in ETS2's world map, exported by the
        # ETS2LA route buffer. This is the planned route, not merely the nearest
        # road segment in front of the truck.
        game_route = self._resolved_game_route(pos, heading)
        if len(game_route) >= 2:
            route = Route([tuple(p) for p in game_route])
            # _resolved_game_route() already projects the truck onto the
            # correct GPS segment and returns only the remaining route.  A
            # second nearest-segment lookup here could jump to a parallel
            # carriageway or another arm of a junction and shifted the whole
            # HUD/controller reference sideways.
            remaining = [list(p) for p in route.points]
            self.sdk.set("map_path", remaining)
            # Display exactly the distance visible in ETS2 GPS. The geometric
            # polyline length is only a validation aid, never the UI authority.
            game_distance = float(self.sdk.get("game_route_distance", 0.0) or 0.0)
            self.sdk.set("distance_to_dest",
                         game_distance if game_distance > 0
                         else route.distance_to_end(pos, heading))
            return self._distance_window(remaining, 260.0)

        # Do not invent a route from whichever road edge happens to be ahead.
        # Steering is allowed only from the in-game GPS (or a recorded route).
        self.sdk.set("map_path", [])
        return []

    def _update_recalculation(self, pos, heading):
        def publish(stage, progress, status):
            self.sdk.set("navigation_progress", progress)
            self.sdk.set("navigation_status", status)
            now = time.monotonic()
            if (stage != self._last_recalc_stage
                    or now - self._last_navigation_status_log >= 1.0):
                self._last_recalc_stage = stage
                self._last_navigation_status_log = now
                logging.info("Navigation [%d%%]: %s", int(progress * 100), status)
                self.sdk.set("navigation_log_event", {
                    "seq": time.time_ns(), "level": "INFO",
                    "message": f"Navigation [{int(progress * 100)}%]: {status}"})

        request = self.sdk.get("nav_recalc_request")
        if request and request != self._last_recalc_request:
            self._last_recalc_request = request
            self._recalc_started = time.monotonic()
            self.sdk.set("navigation_recalculating", True)
            publish("request", 0.08, "Nový cieľ · čítam body trasy z ETS2…")
            self.sdk.set("nav_path", [])
            logging.info("Navigation: recalculating route for new in-game destination.")
        if not self.sdk.get("navigation_recalculating", False):
            return
        elapsed = time.monotonic() - self._recalc_started
        points = self.sdk.get("game_route_node_uids", []) or []
        if elapsed < 0.25:
            publish("validate", 0.25, "Kontrolujem cieľ, mapu a súvislosť trasy…")
            return
        if len(points) < 2:
            stale = bool(self.sdk.get("route_buffer_stale", False))
            if stale:
                game_km = float(self.sdk.get("game_route_distance", 0.0) or 0.0) / 1000.0
                old_km = float(self.sdk.get("route_buffer_distance", 0.0) or 0.0) / 1000.0
                publish("wait-new", 0.36,
                        f"Čakám na novú trasu · stará {old_km:.0f} km, cieľ {game_km:.0f} km")
                # Keep polling while the player closes the world map and ETS2
                # republishes its route; never fall back to the stale UIDs.
                return
            if elapsed > 15.0:
                fallback = []
                self.sdk.set("nav_path", [list(p) for p in fallback[:60]])
                self.sdk.set("navigation_progress", 0.0)
                self.sdk.set("navigation_status", "Herné GPS neposkytlo naplánovanú trasu")
                self.sdk.set("navigation_recalculating", False)
                return
            publish("wait-points", 0.42,
                    f"Čakám na body GPS z ETS2 · {elapsed:.0f} s")
            return
        if self._net_loading or self.road_net is None or not self.road_net.loaded:
            publish("load-map", 0.55,
                    f"Načítavam mapové dáta · prijatých {len(points)} bodov · {elapsed:.0f} s")
            # Loading a full map can legitimately take longer than route
            # matching. Do not report a false route failure after 15 seconds.
            return
        match = float(self.sdk.get("game_route_match", 0.0) or 0.0)
        publish("match", 0.72,
                f"Párujem {len(points)} GPS bodov s mapou · zhoda {match * 100:.0f} %")
        path = self._ensure_map_path(pos, heading)
        if len(path) >= 2 and elapsed >= 0.55:
            self.sdk.set("nav_path", path[:60])
            self.sdk.set("navigation_progress", 1.0)
            self.sdk.set("navigation_status", "Trasa prepočítaná · navigácia pripravená")
            self.sdk.set("navigation_recalculating", False)
            self._last_recalc_stage = "ready"
            resolved = int(self.sdk.get("game_route_resolved_points", 0) or 0)
            distance_km = float(self.sdk.get("distance_to_dest", 0.0) or 0.0) / 1000.0
            logging.info(
                "Navigation calculation complete: %d GPS nodes -> %d detailed map points, "
                "match %.1f%%, remaining %.2f km, elapsed %.2fs.",
                len(points), resolved, match * 100.0, distance_km,
                time.monotonic() - self._recalc_started)
            self.sdk.set("navigation_log_event", {
                "seq": time.time_ns(), "level": "INFO",
                "message": (f"Navigation hotová: {len(points)} GPS uzlov, "
                            f"{resolved} mapových bodov, {distance_km:.2f} km")})
            logging.info("Navigation route published to HUD, live map and AR overlay (%d visible points).",
                         len(path[:60]))
        elif elapsed > 15.0:
            resolved = int(self.sdk.get("game_route_resolved_points", 0) or 0)
            reason = (self.sdk.get("navigation_failure_reason", "")
                      or "mapové body nevytvorili súvislú trasu")
            self.sdk.set("navigation_progress", 0.0)
            self.sdk.set(
                "navigation_status",
                f"Chyba trasy: {reason} · {len(points)} GPS / {resolved} mapových bodov")
            self.sdk.set("navigation_recalculating", False)
            self._last_recalc_stage = "failed"
            logging.error(
                "Navigation calculation failed: %d GPS nodes, %d resolved map points, "
                "match %.1f%%, elapsed %.2fs.",
                len(points), resolved, match * 100.0,
                time.monotonic() - self._recalc_started)
            self.sdk.set("navigation_log_event", {
                "seq": time.time_ns(), "level": "ERROR",
                "message": f"Chyba výpočtu navigácie: {reason}"})
            logging.error("Navigation route was not published: %s. HUD/live map remain empty for safety.",
                          reason)

    # --- Tick -----------------------------------------------------------------
    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        pos = self.sdk.get("truck_world_pos")
        heading = self.sdk.get("truck_heading", 0.0) or 0.0
        speed = self.sdk.get("truck_speed_ms", 0.0) or 0.0

        self._handle_command(pos)

        if self.sdk.get("telemetry_valid", True) is False:
            current = self.sdk.get("lane_trajectory", {}) or {}
            if current.get("valid", False):
                self._publish_invalid_lane_trajectory(
                    "Telemetria vozidla nie je dostupná",
                    self.sdk.get("game_route_node_uids", []) or (),
                    "Telemetria vozidla nie je dostupná")
            else:
                self.sdk.set("nav_active", False)
                self.sdk.set("nav_steering", 0.0)
            return

        if not pos:
            return

        # Lazily load the downloaded road network (engine process) the first
        # time we have a position. Cheap no-op once attempted.
        self._load_road_net()
        self._update_lane_trajectory(pos, heading)

        # Display-only local road geometry. It is deliberately separate from
        # nav_path and therefore cannot influence autopilot steering.
        self._roads_t += delta_time
        if self._roads_t >= 0.35 and self.road_net is not None and self.road_net.loaded:
            self._roads_t = 0.0
            try:
                altitude = float(self.sdk.get("truck_altitude", 0.0) or 0.0)
                roads = self.road_net.hud_segments_3d_near(
                    pos, radius=280.0, limit=950, altitude=altitude)
                self.sdk.set("map_road_segments",
                             [[list(a), list(b), kind, lanes, divided, dash_on,
                               pillar, rail_post]
                              for a, b, kind, lanes, divided, dash_on,
                              pillar, rail_post in roads])
            except Exception as e:
                logging.debug("HUD road geometry error: %s", e)

        # Localization diagnostics: every ~2 s, log where the truck is and where
        # the map thinks the nearest road is. If the distance is huge (hundreds
        # of metres), the chosen map dataset doesn't match the game/mod and the
        # autopilot will chase a road that's nowhere near us.
        self._diag_t += delta_time
        if self._diag_t >= 2.0 and self.road_net is not None and self.road_net.loaded:
            self._diag_t = 0.0
            try:
                seg_idx = self.road_net._nearest_segment_index(pos)
                if seg_idx is not None:
                    (ax, az), (bx, bz) = self.road_net.segments[seg_idx]
                    sdx, sdz = bx - ax, bz - az
                    L2 = sdx * sdx + sdz * sdz
                    if L2 > 1e-9:
                        t = max(0.0, min(1.0, ((pos[0] - ax) * sdx + (pos[1] - az) * sdz) / L2))
                        qx, qz = ax + t * sdx, az + t * sdz
                    else:
                        qx, qz = ax, az
                    dist = math.hypot(pos[0] - qx, pos[1] - qz)
                    logging.debug(
                        "map: truck=(%.0f, %.0f) nearest_seg=(%.0f, %.0f) dist=%.1fm "
                        "heading=%.3f rad (%.0f°)",
                        pos[0], pos[1], qx, qz, dist, heading, math.degrees(heading))
            except Exception as e:
                logging.debug("map diag error: %s", e)

        # Classify the road we're on + publish a speed cap so the autopilot
        # slows down on narrow/local/dirt sectors and keeps full speed on
        # motorways/expressways. Drives the "nech ide pomalšie na poľných /
        # úzkych cestách" behaviour.
        self._publish_road_type(pos)

        # Recording: drop a breadcrumb every ~10 m.
        if self.recording is not None:
            if self.recording.add_point(pos[0], pos[1]):
                self.tags.nav_recording_points = len(self.recording)

        # A live game-GPS route always wins over legacy recorded-route replay.
        # Otherwise replay could overwrite nav_steering/nav_path while HUD and
        # AR still displayed a valid lane snapshot from a different route.
        gps_lane_requested = len(self._normalise_gps_uids(
            self.sdk.get("game_route_node_uids", []) or [])) >= 2
        if (not gps_lane_requested and self.active_route is not None
                and len(self.active_route) >= 2):
            if self.active_route.is_finished(pos):
                self.sdk.set("nav_active", False)
                self.sdk.set("nav_steering", 0.0)
                self.sdk.set("tts_message", "Destination reached.")
                logging.info("Navigation: destination reached.")
                self.active_route = None
                return

            steer = self.active_route.steering(pos, heading, speed,
                                               lane_offset_m=self._lane_offset())
            self.sdk.set("nav_steering", float(steer))
            self.sdk.set("nav_active", True)
            self.sdk.set("distance_to_dest", self.active_route.distance_to_end(pos, heading))
            # Publish the upcoming path curvature so the autopilot can brake
            # BEFORE a sharp bend (anticipatory) instead of reacting to its own
            # steering mid-corner. Radius in metres; large = straight.
            self.sdk.set("path_curvature_radius",
                         self.active_route.curvature_ahead(pos, heading))
            self.tags.nav_steering = round(steer, 3)

            # Publish the upcoming path points so the HUD can draw "where to go".
            idx = self.active_route.closest_index(pos)
            upcoming = self._distance_window(self.active_route.points[idx:], 220.0)
            visible = self._driving_line(upcoming, self._lane_offset())
            self.sdk.set("nav_path", [list(p) for p in visible])
        else:
            # No recorded route: drive by the downloaded MAP. This is automatic
            # map-based driving — no recording needed.
            snapshot = self.sdk.get("lane_trajectory", {}) or {}
            route = self._lane_route
            if (route is not None and len(route) >= 2
                    and bool(snapshot.get("valid", False))
                    and int(snapshot.get("revision", -1)) == self._lane_revision
                    and tuple(snapshot.get("source_gps_uids", ()) or ())
                        == self._normalise_gps_uids(
                            self.sdk.get("game_route_node_uids", []) or [])):
                # The native planned-route buffer is lane-specific geometry.
                # Applying the generic road-centre offset once more moved the
                # target towards the median (and made HUD and steering disagree
                # with the truck's actual lane).
                steer = route.steering(pos, heading, speed,
                                       lane_offset_m=0.0)
                # Safety: if the truck is far from the snapped path (wrong map
                # dataset, or we're off-road on a ferry / car park), the CTE is
                # huge and Stanley saturates to full-lock. Detect that and
                # disable nav steering instead of yanking the wheel — the
                # autopilot then falls back to vision lane-keeping.
                idx = route.tracking_index(pos, heading)
                nearest = route.points[min(idx, len(route.points)-1)]
                off_dist = math.hypot(pos[0] - nearest[0], pos[1] - nearest[1])
                if off_dist > 50.0:
                    self.sdk.set("nav_active", False)
                    self.sdk.set("nav_steering", 0.0)
                    self.sdk.set("map_status",
                                 f"Truck is {off_dist:.0f}m from the nearest road — "
                                 "map dataset may not match the game. Switch maps on the Map page.")
                    self.tags.nav_steering = 0.0
                else:
                    self.sdk.set("nav_steering", float(steer))
                    self.sdk.set("nav_active", True)
                # Curvature radius (m) of the road ahead — lets the autopilot
                # anticipate bends (brake before, not during).
                self.sdk.set("path_curvature_radius",
                             route.curvature_ahead(pos, heading))
                self.tags.nav_steering = round(steer, 3)
            else:
                self.sdk.set("nav_active", False)
                self.sdk.set("nav_steering", 0.0)

        self._lane_diag_t += delta_time
        if self._lane_diag_t >= 1.0:
            self._lane_diag_t = 0.0
            snapshot = self.sdk.get("lane_trajectory", {}) or {}
            match = snapshot.get("lane_match") or {}
            logging.info(
                "lane-trajectory: revision=%s valid=%s confidence=%.3f lane=%s "
                "lateral=%.3f heading=%.3f vertical=%.3f control=%d display=%d "
                "gps_distance=%.1f lane_distance=%.1f switch=%s failure=%s",
                snapshot.get("revision", 0), bool(snapshot.get("valid", False)),
                float(snapshot.get("confidence", 0.0) or 0.0),
                snapshot.get("active_lane_id"),
                float(match.get("lateral_error_m", 0.0) or 0.0),
                float(match.get("heading_error_rad", 0.0) or 0.0),
                float(match.get("vertical_error_m", 0.0) or 0.0),
                len(snapshot.get("points", ()) or ()),
                len(snapshot.get("display_points", ()) or ()),
                float(self.sdk.get("game_route_distance", 0.0) or 0.0),
                float(snapshot.get("distance_m", 0.0) or 0.0),
                match.get("switch_reason", ""),
                snapshot.get("failure_reason", ""))
