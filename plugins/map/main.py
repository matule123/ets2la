import logging
import os
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
                    downloaded = [d for d in map_data.list_datasets() if d["downloaded"]]
                    if not downloaded:
                        self.sdk.set("map_status", "No map downloaded yet.")
                        return
                    self.sdk.set("map_status",
                                 f"Loading road network ({downloaded[0]['key']})…")
                    net = RoadNetwork()
                    if net.load(map_data.dataset_dir(downloaded[0]["key"])):
                        self.road_net = net
                        self.sdk.set("map_status",
                                     f"Map ready ({len(net.segments)} segments). "
                                     "Map-based steering active.")
                        logging.info("Navigation: road network loaded engine-side "
                                     "(%d segments).", len(net.segments))
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

    def _ensure_map_path(self, pos, heading):
        """Compute and publish the road-ahead polyline from the downloaded map.

        Falls back to whatever the UI process publishes as ``map_path`` if the
        engine-side network isn't available yet.
        """
        # Prefer the engine-side network (works regardless of which UI page is open).
        if self.road_net is not None and self.road_net.loaded:
            try:
                path = self.road_net.path_ahead(pos, heading)
            except Exception:
                path = []
            if len(path) >= 2:
                self.sdk.set("map_path", [list(p) for p in path])
                return [list(p) for p in path[:25]]
            self.sdk.set("map_path", [])
            return []
        # Fallback: reuse a path the UI process may have published.
        return self.sdk.get("map_path", []) or []

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

        # Lazily load the downloaded road network (engine process) the first
        # time we have a position. Cheap no-op once attempted.
        self._load_road_net()

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

            steer = self.active_route.steering(pos, heading, speed)
            self.sdk.set("nav_steering", float(steer))
            self.sdk.set("nav_active", True)
            self.sdk.set("distance_to_dest", self.active_route.distance_to_end(pos))
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
                steer = route.steering(pos, heading, speed)
                self.sdk.set("nav_steering", float(steer))
                self.sdk.set("nav_active", True)
                self.sdk.set("nav_path", [list(p) for p in map_path[:25]])
                self.tags.nav_steering = round(steer, 3)
            else:
                self.sdk.set("nav_active", False)
                self.sdk.set("nav_path", [])
