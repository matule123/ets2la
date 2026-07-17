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
        runs the truck keeps driving by whatever path is already available, and
        map-based steering switches on the moment the network is ready.
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
                                     "Map-based steering active.")
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
                return float(drv)
            except (TypeError, ValueError):
                pass
        v = self.sdk.get("lane_offset_m", None)
        if v is not None:
            try:
                return float(v)
            except (TypeError, ValueError):
                pass
        return 2.7

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

    def _ensure_map_path(self, pos, heading):
        """Compute and publish the road-ahead polyline from the downloaded map.

        Falls back to whatever the UI process publishes as ``map_path`` if the
        engine-side network isn't available yet.
        """
        # Prefer the actual route selected in ETS2's world map, exported by the
        # ETS2LA route buffer. This is the planned route, not merely the nearest
        # road segment in front of the truck.
        game_route = self.sdk.get("game_route_points", []) or []
        if len(game_route) >= 2:
            route = Route([tuple(p) for p in game_route])
            idx = route.closest_index(pos)
            remaining = [list(p) for p in route.points[idx:]]
            self.sdk.set("map_path", remaining)
            self.sdk.set("distance_to_dest", route.distance_to_end(pos))
            return remaining[:60]

        # Do not invent a route from whichever road edge happens to be ahead.
        # Steering is allowed only from the in-game GPS (or a recorded route).
        self.sdk.set("map_path", [])
        return []

    def _update_recalculation(self, pos, heading):
        request = self.sdk.get("nav_recalc_request")
        if request and request != self._last_recalc_request:
            self._last_recalc_request = request
            self._recalc_started = time.monotonic()
            self.sdk.set("navigation_recalculating", True)
            self.sdk.set("navigation_progress", 0.08)
            self.sdk.set("navigation_status", "Nový cieľ · načítavam trasu z hry…")
            self.sdk.set("nav_path", [])
            logging.info("Navigation: recalculating route for new in-game destination.")
        if not self.sdk.get("navigation_recalculating", False):
            return
        elapsed = time.monotonic() - self._recalc_started
        points = self.sdk.get("game_route_points", []) or []
        if elapsed < 0.25:
            self.sdk.set("navigation_progress", 0.25)
            self.sdk.set("navigation_status", "Kontrolujem cieľ a mapové dáta…")
            return
        if len(points) < 2:
            if elapsed > 6.0:
                fallback = []
                self.sdk.set("nav_path", [list(p) for p in fallback[:60]])
                self.sdk.set("navigation_progress", 0.0)
                self.sdk.set("navigation_status", "Herné GPS neposkytlo naplánovanú trasu")
                self.sdk.set("navigation_recalculating", False)
                return
            self.sdk.set("navigation_progress", 0.42)
            self.sdk.set("navigation_status", "Čakám na naplánovanú trasu z ETS2…")
            return
        self.sdk.set("navigation_progress", 0.72)
        self.sdk.set("navigation_status", f"Spracúvam {len(points)} bodov trasy…")
        path = self._ensure_map_path(pos, heading)
        if len(path) >= 2 and elapsed >= 0.55:
            self.sdk.set("nav_path", path[:60])
            self.sdk.set("navigation_progress", 1.0)
            self.sdk.set("navigation_status", "Trasa prepočítaná · navigácia pripravená")
            self.sdk.set("navigation_recalculating", False)
            logging.info("Navigation: route recalculated (%d planned points).", len(points))

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
                roads = self.road_net.hud_segments_near(pos)
                self.sdk.set("map_road_segments", [[list(a), list(b)] for a, b in roads])
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
            self.sdk.set("distance_to_dest", self.active_route.distance_to_end(pos))
            # Publish the upcoming path curvature so the autopilot can brake
            # BEFORE a sharp bend (anticipatory) instead of reacting to its own
            # steering mid-corner. Radius in metres; large = straight.
            self.sdk.set("path_curvature_radius",
                         self.active_route.curvature_ahead(pos, heading))
            self.tags.nav_steering = round(steer, 3)

            # Publish the upcoming path points so the HUD can draw "where to go".
            idx = self.active_route.closest_index(pos)
            self.sdk.set("nav_path", [list(p) for p in self.active_route.points[idx:idx + 25]])
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
                    self.sdk.set("nav_path", [list(p) for p in map_path[:25]])
                # Curvature radius (m) of the road ahead — lets the autopilot
                # anticipate bends (brake before, not during).
                self.sdk.set("path_curvature_radius",
                             route.curvature_ahead(pos, heading))
                self.tags.nav_steering = round(steer, 3)
            else:
                self.sdk.set("nav_active", False)
                self.sdk.set("nav_path", [])
