import logging
import os
import math
import time
from sdk.base_plugin import BasePlugin
from core.navigation.route import Route
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
        self._resolved_route_cache_signature = None
        self._resolved_route_cache = []
        self._failed_route_signature = None
        os.makedirs(ROUTES_DIR, exist_ok=True)
        self._publish_route_list()

    def on_stop(self):
        logging.info("Map (navigation) plugin stopped.")
        self.sdk.set("nav_active", False)
        self.sdk.set("nav_steering", 0.0)

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
        all_valid = [uid for uid in normalized_uids if uid in self.road_net.nodes]
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
        valid_uids = (min(viable,
                          key=lambda run: min(math.dist(pos, self.road_net.nodes[uid])
                                              for uid in run))
                      if viable else all_valid)
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
            meta = self.sdk.get("game_route_meta", []) or []
            distances = [float(item.get("distance", 0.0) or 0.0)
                         for item in meta if isinstance(item, dict)]
            if len(distances) >= 2 and abs(distances[0] - distances[-1]) > 1.0:
                self._route_reversed = distances[0] < distances[-1]
            else:
                self._route_reversed = direction_score(idx - 1) > direction_score(idx + 1)
        if self._route_reversed:
            matched.reverse()
            valid_uids.reverse()
            route = Route(matched)
            idx = route.tracking_index(pos, heading)
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
            for target_index in range(route_start,
                                      min(len(valid_uids), route_start + 5)):
                bridge = self.road_net._route_bridge(
                    start_uid, valid_uids[target_index], max_expanded=16000)
                if not bridge:
                    continue
                bridge_length = sum(
                    math.dist(self.road_net.nodes[a], self.road_net.nodes[b])
                    for a, b in zip(bridge, bridge[1:]))
                if best_join is None or bridge_length < best_join[0]:
                    best_join = (bridge_length, target_index, bridge)
            if best_join is not None:
                _join_length, target_index, bridge = best_join
                remaining_uids = bridge + valid_uids[target_index + 1:]
                route_snap_point = tuple(snap_point)
                logging.info(
                    "Navigation: truck snapped to road graph; joining GPS route over %.1f m.",
                    _join_length)
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
        previous_vector = None
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
                    # Detailed prefab/road curves cannot make an instantaneous
                    # right angle. Such a corner is a fake bridge between two
                    # unrelated branches and caused the blue stair-step line.
                    if turn_degrees > 52.0:
                        reason = f"neplatny skok smeru GPS trasy {turn_degrees:.0f} stupnov"
                        self.sdk.set("navigation_failure_reason", reason)
                        self.sdk.set("game_route_points", [])
                        self.sdk.set("nav_path", [])
                        self.sdk.set("nav_active", False)
                        self.sdk.set("navigation_unreliable", True)
                        logging.error("Navigation rejected for safety: %s.", reason)
                        return []
                previous_vector = vector
        game_distance = float(self.sdk.get("game_route_distance", 0.0) or 0.0)
        if (game_distance > 100.0
                and abs(route_length - game_distance)
                > max(2000.0, game_distance * 0.20)):
            reason = (f"mapa vypocitala {route_length / 1000.0:.1f} km, "
                      f"ale herne GPS ukazuje {game_distance / 1000.0:.1f} km")
            self.sdk.set("navigation_failure_reason", reason)
            self.sdk.set("game_route_points", [])
            self.sdk.set("nav_path", [])
            self.sdk.set("nav_active", False)
            self.sdk.set("navigation_unreliable", True)
            logging.error("Navigation rejected for safety: %s.", reason)
            return []
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
            idx = route.tracking_index(pos, heading)
            remaining = [list(p) for p in route.points[idx:]]
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

        if not pos:
            return

        self._update_recalculation(pos, heading)

        # Lazily load the downloaded road network (engine process) the first
        # time we have a position. Cheap no-op once attempted.
        self._load_road_net()

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

        # Replay: follow the active route.
        if self.active_route is not None and len(self.active_route) >= 2:
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
            map_path = self._ensure_map_path(pos, heading)
            if len(map_path) >= 2:
                route = Route([tuple(p) for p in map_path])
                steer = route.steering(pos, heading, speed,
                                       lane_offset_m=self._lane_offset())
                # Safety: if the truck is far from the snapped path (wrong map
                # dataset, or we're off-road on a ferry / car park), the CTE is
                # huge and Stanley saturates to full-lock. Detect that and
                # disable nav steering instead of yanking the wheel — the
                # autopilot then falls back to vision lane-keeping.
                off_dist = math.hypot(pos[0] - map_path[0][0], pos[1] - map_path[0][1])
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
                    upcoming = self._distance_window(map_path, 220.0)
                    visible = self._driving_line(upcoming, self._lane_offset())
                    self.sdk.set("nav_path", [list(p) for p in visible])
                # Curvature radius (m) of the road ahead — lets the autopilot
                # anticipate bends (brake before, not during).
                self.sdk.set("path_curvature_radius",
                             route.curvature_ahead(pos, heading))
                self.tags.nav_steering = round(steer, 3)
            else:
                self.sdk.set("nav_active", False)
                self.sdk.set("nav_path", [])
