import time
import logging

from core.telemetry import Telemetry
from core.controller import Controller
from core.plugin_manager import PluginManager
from core.perception import Perception
from core.module_manager import ModuleManager
from core.settings.manager import SettingsManager
from core.ipc.shared_state import SharedState
from core.voice.assistant import VoiceAssistant
from core.modules.game_watcher import GameWatcher
from core.modules.better_screen_capture import BetterScreenCapture
from core.modules.traffic_analysis import TrafficAnalysis
from core.planner import UltraPilotPlanner
from sdk.plugin_sdk import (
    CTL_STEERING, CTL_THROTTLE, CTL_BRAKE, CTL_BLINKER, CTL_PAY_TOLL,
)


class UltraPilotEngine:
    """
    The main engine for ETS2-UltraPilot.

    Owns the single physical Controller.  Plugins never drive the device
    directly — they write *control intents* into shared state and the engine
    flushes them here, gated by a master ``autopilot_active`` switch for safety.
    """

    def __init__(self, shared_dict=None):
        try:
            from core.logger import setup as _ls
            _ls()
        except Exception:
            logging.basicConfig(level=logging.INFO)
        logging.info("Starting ETS2-UltraPilot Engine...")

        self.settings = SettingsManager()
        # Wrap the shared dict handed down by the bootloader (or create one).
        self.shared_state = SharedState(shared_dict)
        # Route voice alerts through shared state to the single tts plugin speaker.
        self.voice = VoiceAssistant(self.shared_state)

        self.telemetry = Telemetry()
        self.controller = Controller()
        # Surrounding traffic + traffic lights from the ETS2LA game plugin (if installed).
        from core.sdk.ets2la_data import ETS2LAData
        self.ets2la = ETS2LAData()
        from core.sdk.ets2la_route import ETS2LARouteReader
        self.ets2la_route = ETS2LARouteReader()
        self.perception = Perception(self.shared_state)
        self.planner = UltraPilotPlanner()

        self.module_manager = ModuleManager(self)
        self.plugin_manager = PluginManager(self)

        # Register Core Modules
        self.module_manager.register_module(GameWatcher)
        self.module_manager.register_module(BetterScreenCapture)
        self.module_manager.register_module(TrafficAnalysis)

        self.running = False
        self.fps = self.settings.get("general", {}).get("fps", 60)

        # Global hotkey ('N' by default) to toggle the autopilot from inside the
        # game without alt-tabbing.  Uses GetAsyncKeyState (works app-wide).
        self._hotkey_vk = 0x4E  # 'N'
        self._hotkey_was_down = False
        self._cranking = False
        self._engine_off_samples = 0
        self._engine_start_attempt = 0
        self._last_engine_start = 0.0
        self._last_game_route_distance = None
        self._last_game_destination = ""
        self._last_route_signature = None
        # Track autopilot on/off edges so we release controls only once on disable.
        self._was_active = False
        try:
            import win32api  # noqa: F401
            self._has_win32 = True
        except Exception:
            self._has_win32 = False

        # Publish current settings so plugins (other processes) can read them.
        self.shared_state.set("settings", self.settings.settings)
        # Master safety switch: nothing is sent to the game until enabled.
        if self.shared_state.get("autopilot_active") is None:
            self.shared_state.set("autopilot_active", False)

    def start(self):
        self.running = True
        self.plugin_manager.discover_and_load()
        self.run_loop()

    def stop(self):
        self.running = False
        self.controller.release_all()
        self.module_manager.stop_all()
        self.plugin_manager.stop_all()
        self.voice.stop()
        logging.info("ETS2-UltraPilot Engine stopped.")

    def _autostart_truck(self, truck):
        """Recover a stalled engine using telemetry-controlled ignition steps."""
        if not self.shared_state.get("autopilot_active", False):
            self._engine_off_samples = 0
            return
        running = bool(truck.get("engineEnabled", False)) or float(
            truck.get("engineRpm", 0.0) or 0.0) > 150.0
        if running:
            self._engine_off_samples = 0
            self._engine_start_attempt = 0
            return

        # Ignore short false telemetry frames. Previously one false boolean was
        # enough to press E, which could turn a healthy engine off.
        self._engine_off_samples += 1
        if self._engine_off_samples < max(8, int(self.fps * 0.35)):
            return
        if self._cranking:
            return
        now = time.time()
        if now - self._last_engine_start < 5.0:
            return
        self._last_engine_start = now
        self._engine_start_attempt += 1
        self._cranking = True

        def _running_now():
            current = ((self.shared_state.get("telemetry", {}) or {})
                       .get("truck", {}) or {})
            return (bool(current.get("engineEnabled", False)) or
                    float(current.get("engineRpm", 0.0) or 0.0) > 150.0)

        def _start_sequence():
            import pydirectinput
            try:
                # ETS2 configurations differ: a tap may enable electrics, a
                # second tap may select ignition, and realistic ignition needs
                # a hold. Check telemetry after every stage and stop instantly
                # when the engine catches so E can never toggle it back off.
                for _ in range(2):
                    if _running_now():
                        return
                    pydirectinput.press('e')
                    time.sleep(0.45)
                if _running_now():
                    return
                hold_s = min(2.4, 1.25 + 0.25 * (self._engine_start_attempt - 1))
                pydirectinput.keyDown('e')
                deadline = time.time() + hold_s
                while time.time() < deadline and not _running_now():
                    time.sleep(0.08)
            except Exception as e:
                logging.warning("Engine start sequence failed: %s", e)
            finally:
                try:
                    pydirectinput.keyUp('e')
                except Exception:
                    pass
                self._cranking = False

        try:
            import threading
            threading.Thread(target=_start_sequence, daemon=True).start()
            logging.info("Truck engine off — adaptive ignition sequence started.")
            self.shared_state.set("tts_message", "Štartujem motor.")
        except Exception as e:
            self._cranking = False
            logging.warning("Could not start ignition worker: %s", e)

    # --- Traffic following ----------------------------------------------------
    def _lead_brake(self, traffic, pos, heading):
        """Brake (0..1) for the closest vehicle ahead in our lane, else 0.

        Phase 1 tuning — uses **time-to-collision** instead of a bare distance
        ramp, so it brakes early when we are closing fast on a car (even one
        that's far away) but holds a short gap when we're already matched in
        speed.  This is what makes it actually stop *before* the car instead of
        rear-ending it: the closing speed, not the distance, drives the brake.
        """
        import math
        if not traffic or not pos:
            return 0.0
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)
        # Our forward speed (m/s) for relative-velocity math.
        my_speed = float(self.shared_state.get("truck_speed_ms", 0.0) or 0.0)
        best = None  # (ahead, closing_speed)
        for v in traffic:
            dx, dz = v["x"] - px, v["z"] - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lateral = dx * cos_h - dz * sin_h
            if not (2.0 < ahead < 120.0 and abs(lateral) < 2.6):  # in our lane, ahead
                continue
            # Skip ONCOMING vehicles (facing roughly opposite to us) — only brake
            # for cars going the same way, so we don't stop for the other lane.
            vyaw = v.get("yaw", heading)
            facing = math.cos(vyaw - heading)   # ~1 same dir, ~-1 oncoming
            if facing < -0.3:
                continue
            # Closing speed = how fast the gap is shrinking (m/s). Lead's speed
            # projected onto our forward direction.
            lead_speed = float(v.get("speed", 0.0) or 0.0)
            closing = max(0.0, my_speed - lead_speed)
            if best is None or ahead < best[0]:
                best = (ahead, closing)
        if best is None:
            return 0.0
        ahead, closing = best
        self.shared_state.set("lead_distance", ahead)

        # Safety gap we always want to keep (scales a little with our speed).
        safe_gap = 6.0 + 0.4 * abs(my_speed)          # ~6 m at rest, ~30 m at 60 km/h
        # Effective gap once the car's length is accounted for.
        gap = max(0.0, ahead - safe_gap)

        # If we're not closing, a pure distance brake is enough (gentle).
        if closing < 0.5:
            if gap <= 0.0:
                return 1.0
            if gap >= 40.0:
                return 0.0
            return float((40.0 - gap) / 40.0)

        # Closing: time-to-collision to the safe gap. Small TTC → strong brake.
        ttc = gap / closing if closing > 1e-3 else 0.0
        if ttc <= 1.0:        # <1s to gap — brake fully
            return 1.0
        if ttc >= 5.0:        # >5s — no action yet
            return 0.0
        # Smooth 1→0 between TTC 1s..5s (square for a firmer near-range response).
        return float(((5.0 - ttc) / 4.0) ** 2)

    def _light_brake(self, light):
        """Brake (0..1) to stop smoothly at a red light ahead; 0 on green/none.

        Phase 1 tuning — yellow now also eases the speed down (the old code kept
        full throttle through yellow), and the red stop is approached with a
        deceleration ramp instead of a hard 1.0 at the line, so the truck glides
        to the stop line instead of lunging at it.
        """
        if not light:
            return 0.0
        color = light.get("color")
        dist = light.get("distance", 999.0)
        my_speed = float(self.shared_state.get("truck_speed_ms", 0.0) or 0.0)

        if color == "red":
            # STOP-HOLD: once we're basically stopped near the line, clamp to a
            # full hold so the truck can't creep forward through the red (the
            # proportional ramp alone fades as speed drops and the ACC throttle
            # then nudges us into the junction).
            #
            # Stop-and-go exception (Fáza 3e): in a queue at a red light the car
            # ahead often pulls forward a few metres as the queue shuffles. We
            # must FOLLOW it (creep), not hold dead still — otherwise we leave a
            # growing gap and the cars behind lay on the horn. So if a lead
            # vehicle is detected ahead AND it has moved well clear of us, relax
            # the hard hold to a softer brake that lets the truck crawl up to it.
            lead_dist = None
            try:
                lead_dist = self.shared_state.get("lead_distance")
                if lead_dist is not None:
                    lead_dist = float(lead_dist)
            except (TypeError, ValueError):
                lead_dist = None
            queue_creep = (lead_dist is not None and lead_dist > 8.0)
            if my_speed < 0.5 and dist <= 12.0 and not queue_creep:
                return 1.0
            # Begin braking early; full as we reach the line.
            if dist <= 6.0 and not queue_creep:
                return 1.0
            if dist >= 70.0:
                return 0.0
            # Steeper ramp than the old 50m so a fast truck still stops in time.
            return float(max(0.0, min(1.0, (70.0 - dist) / 50.0)))

        if color == "yellow":
            # Only ease off for yellow if we can still stop comfortably; don't
            # panic-brake a fast truck that's basically at the line.
            stop_dist = (my_speed ** 2) / (2.0 * 4.0)   # ~4 m/s^2 comfortable
            if dist > stop_dist + 6.0 and dist < 60.0:
                return float(max(0.0, min(0.5, (60.0 - dist) / 80.0)))
            return 0.0

        return 0.0          # green / off → keep going

    # --- Articulated trailer -------------------------------------------------
    def _articulation_angle(self, truck_heading: float, trailer_heading: float) -> float:
        """Signed angle (radians) between the tractor and the semi-trailer.

        Positive = the trailer's tail is swung to the LEFT of the tractor's
        heading (a right-hand bend pushes it left and vice-versa as the combo
        pivots about the fifth wheel). Wrapped to ``[-π, π]``. Used only for
        the HUD drawing of the hinged trailer — it does NOT affect steering."""
        import math
        diff = float(truck_heading or 0.0) - float(trailer_heading or 0.0)
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff

    # --- Hotkey ---------------------------------------------------------------
    @staticmethod
    def _game_window_active():
        """True only when ETS2/ATS owns the foreground Windows window."""
        try:
            import psutil
            import win32gui
            import win32process
            hwnd = win32gui.GetForegroundWindow()
            if not hwnd:
                return False
            _thread_id, process_id = win32process.GetWindowThreadProcessId(hwnd)
            executable = psutil.Process(process_id).name().lower()
            return executable in {"eurotrucks2.exe", "amtrucks.exe"}
        except Exception:
            # Process-name lookup can be denied by Windows security software;
            # the official game window titles are a safe secondary check.
            try:
                import win32gui
                title = win32gui.GetWindowText(win32gui.GetForegroundWindow()).lower()
                return ("euro truck simulator 2" in title
                        or "american truck simulator" in title)
            except Exception:
                return False

    def _check_hotkey(self):
        """Toggle on an N-key rising edge, but only inside the game window."""
        if not self._has_win32:
            return
        try:
            import win32api
            down = bool(win32api.GetAsyncKeyState(self._hotkey_vk) & 0x8000)
        except Exception:
            return
        if down and not self._hotkey_was_down and self._game_window_active():
            new_state = not bool(self.shared_state.get("autopilot_active", False))
            self.shared_state.set("autopilot_active", new_state)
            msg = "Autopilot enabled." if new_state else "Autopilot disabled."
            logging.info("Hotkey N -> %s", msg)
            self.shared_state.set("tts_message", msg)
            if not new_state:
                self.controller.release_all()
        self._hotkey_was_down = down

    def _process_autopilot_command(self):
        """Apply and acknowledge the UI's explicit master-switch command."""
        command = self.shared_state.get("autopilot_command")
        if not isinstance(command, dict):
            return
        seq = command.get("seq")
        if not seq or seq == getattr(self, "_last_autopilot_command", None):
            return
        desired = bool(command.get("enabled", False))
        self._last_autopilot_command = seq
        self.shared_state.set("autopilot_active", desired)
        if not desired:
            self.controller.release_all()
            self._was_active = False
            self.shared_state.update_batch({
                CTL_STEERING: 0.0, CTL_THROTTLE: 0.0, CTL_BRAKE: 0.0,
            })
        self.shared_state.set("autopilot_command_ack", seq)
        self.shared_state.set("autopilot_command_pending", None)
        logging.info("Autopilot %s (command acknowledged).",
                     "enabled" if desired else "disabled")

    # --- Control flush --------------------------------------------------------
    def _flush_controls(self):
        """Apply the latest control intents to the physical device.

        Gated by the master switch.  When the autopilot is NOT active we release
        everything once and then leave the controls untouched — so the driver
        keeps full manual control of a real wheel (writing 0 every frame would
        fight the player's steering through the SCS SDK input)."""
        if not self.shared_state.get("autopilot_active", False):
            if self._was_active:
                self.controller.release_all()
                self._was_active = False
            return
        self._was_active = True

        steering = self.shared_state.get(CTL_STEERING, 0.0)
        throttle = self.shared_state.get(CTL_THROTTLE, 0.0)
        brake = self.shared_state.get(CTL_BRAKE, 0.0)

        # Live steering tuning from the Settings page: sensitivity + invert.
        sens = self.shared_state.get("steering_sensitivity", 1.0) or 1.0
        steering = max(-1.0, min(1.0, float(steering) * float(sens)))
        if self.shared_state.get("steering_invert", False):
            steering = -steering

        # Speed-dependent steering clamp: the faster we go, the less the wheel
        # may turn — this stops the truck from yanking into a barrier in curves.
        spd_kmh = abs(float(self.shared_state.get("truck_speed_ms", 0.0) or 0.0)) * 3.6
        max_steer = 1.0 if spd_kmh < 30 else max(0.25, 1.0 - (spd_kmh - 30) / 110.0)

        # Jackknife / trailer-swing protection (Fáza 3d). When a semi-trailer is
        # coupled and its articulation angle is already large, winding the wheel
        # further into the SAME direction the trailer is swinging would fold the
        # combo (jackknife at low speed, trailer-swing at speed). We clamp the
        # steering away from the dangerous direction, scaled by how folded the
        # combo already is — full clamp at 35° articulation. This is a safety
        # limit only; it never increases the steering command.
        if self.shared_state.get("trailer_attached", False):
            import math as _m
            try:
                art = float(self.shared_state.get("trailer_articulation", 0.0) or 0.0)
            except (TypeError, ValueError):
                art = 0.0
            fold = abs(art) / _m.radians(35.0)         # 0..1+ at 35° articulation
            if fold > 0.5:                             # only intervene past ~17°
                # Limit how much MORE we can steer into the swing direction.
                # art>0 → trailer tail left → swinging right → clamp +steering.
                sign = 1.0 if art > 0 else -1.0
                # Reserve shrinks from full toward ~0.3 as we approach a fold.
                reserve = max(0.3, 1.0 - fold)
                # Asymmetric cap: allow the safe direction fully, the dangerous
                # one only up to (current steering scaled by reserve).
                if (steering * sign) > 0:
                    steering = sign * min(abs(steering),
                                          max_steer * reserve, max_steer)
        steering = max(-max_steer, min(max_steer, steering))

        self.controller.set_steering(steering)
        self.controller.set_throttle(throttle)
        self.controller.set_brake(brake)

        # Blinker: a plugin may force one via ctl_blinker, otherwise follow the planner.
        blinker = self.shared_state.get(CTL_BLINKER) or self.shared_state.get("active_blinker", "off")
        self.controller.set_blinker(blinker)
        if self.shared_state.get(CTL_BLINKER):
            self.shared_state.set(CTL_BLINKER, None)

        if self.shared_state.get(CTL_PAY_TOLL):
            self.controller.pay_toll()
            self.shared_state.set(CTL_PAY_TOLL, False)

    def run_loop(self):
        last_time = time.time()
        while self.running:
            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time

            # 0. Global hotkey + explicit UI command. Process the UI command
            # last so a click always wins over a coincident N-key edge.
            self._check_hotkey()
            self._process_autopilot_command()

            # 1. Telemetry
            if self.telemetry.update():
                truck = self.telemetry.get("truck", {}) or {}
                raw = self.telemetry.get("raw", {}) or {}
                self.shared_state.set(
                    "game_in_truck", bool(raw.get("sdkActive", bool(truck))))
                self._autostart_truck(truck)
                dest_city = self.telemetry.get("dest_city", "") or ""
                try:
                    route_distance = float(truck.get("routeDistance", 0.0) or 0.0)
                except (TypeError, ValueError):
                    route_distance = 0.0
                prev_distance = self._last_game_route_distance
                destination_changed = bool(
                    dest_city and dest_city != self._last_game_destination)
                route_changed = bool(
                    route_distance > 0 and prev_distance is not None and prev_distance > 0
                    and abs(route_distance - prev_distance) > 1000.0)
                first_route = bool(route_distance > 0 and prev_distance in (None, 0))
                planned_items = self.ets2la_route.read()
                planned_uids = [int(item["uid"]) for item in planned_items
                                if isinstance(item, dict) and item.get("uid")]
                route_signature = None
                if len(planned_uids) >= 2:
                    route_signature = (len(planned_uids), planned_uids[-1])
                    if route_signature != self._last_route_signature:
                        self.shared_state.set("game_route_points", [])
                    self.shared_state.set("game_route_node_uids", planned_uids)
                    self.shared_state.set("game_route_meta", planned_items)
                else:
                    self.shared_state.set("game_route_node_uids", [])
                    self.shared_state.set("game_route_points", [])
                planned_route_changed = bool(
                    route_signature and route_signature != self._last_route_signature)
                if destination_changed or route_changed or first_route or planned_route_changed:
                    request = f"{time.time():.3f}:{dest_city}:{route_distance:.0f}"
                    self.shared_state.set("nav_recalc_request", request)
                    self.shared_state.set("nav_destination", dest_city or "nový cieľ")
                    logging.info("Navigation: new in-game destination detected (%s, %.1f km).",
                                 dest_city or "map waypoint", route_distance / 1000.0)
                if route_signature:
                    self._last_route_signature = route_signature
                if route_distance > 0:
                    self._last_game_route_distance = route_distance
                if dest_city:
                    self._last_game_destination = dest_city
                self.shared_state.update_batch({
                    "telemetry": self.telemetry.data,
                    "speed": truck.get("speed", 0),
                    # World pose for coordinate-based navigation (map plugin).
                    "truck_world_pos": (truck.get("x", 0.0), truck.get("z", 0.0)),
                    "truck_heading": truck.get("rotation", 0.0),
                    "truck_speed_ms": truck.get("speed", 0.0),
                    # Destination city of the current job (for the gantry sign).
                    "dest_city": dest_city,
                    "game_route_distance": route_distance,
                    "game_route_time": float(truck.get("routeTime", 0.0) or 0.0),
                })

                # Trailer (articulated semi-trailer, Zone 14). We publish its
                # world pose + the articulation angle (signed heading difference
                # between tractor and trailer) so the HUD can draw the trailer
                # hinged behind the cab. When no trailer is attached we publish
                # empty values, which the HUD treats as "cab only".
                trailer = self.telemetry.get("trailer", {}) or {}
                if trailer.get("attached"):
                    tr_pos = (trailer.get("x", 0.0), trailer.get("z", 0.0))
                    articulation = self._articulation_angle(
                        truck.get("rotation", 0.0), trailer.get("rotation", 0.0))
                    self.shared_state.update_batch({
                        "trailer_attached": True,
                        "trailer_world_pos": tr_pos,
                        "trailer_heading": trailer.get("rotation", 0.0),
                        "trailer_articulation": articulation,
                    })
                else:
                    # Clear stale trailer state when the trailer is uncoupled.
                    if self.shared_state.get("trailer_attached", False):
                        self.shared_state.update_batch({
                            "trailer_attached": False,
                            "trailer_world_pos": None,
                            "trailer_heading": None,
                            "trailer_articulation": 0.0,
                        })

                # Surrounding traffic + the traffic light controlling us (ETS2LA plugin).
                try:
                    from core.sdk.ets2la_data import nearest_light_ahead
                    traffic = self.ets2la.read_traffic()
                    lights = self.ets2la.read_traffic_lights()
                    pos = (truck.get("x", 0.0), truck.get("z", 0.0))
                    hdg = truck.get("rotation", 0.0)
                    self.shared_state.set("traffic", traffic)
                    light = nearest_light_ahead(lights, pos, hdg)
                    self.shared_state.set("traffic_light", light)
                    # Lead-vehicle following: brake for the nearest car ahead in our lane.
                    self.shared_state.set("traffic_brake", self._lead_brake(traffic, pos, hdg))
                    # Stop on red / go on green.
                    self.shared_state.set("light_brake", self._light_brake(light))
                except Exception:
                    pass
            else:
                self.shared_state.set("game_in_truck", False)

            # A transient error in any one frame must NOT kill the engine — log
            # it and keep looping (self-healing). The bootloader also restarts
            # the whole Engine process if it ever does die.
            try:
                # 2. Perception
                obstacle_data = self.perception.detect_obstacles()
                self.shared_state.set("obstacle", obstacle_data)
                self.shared_state.set("nav_direction", self.perception.detect_navigation_arrow())
                self.shared_state.set("lane_offset", self.perception.detect_lanes())
                self.shared_state.set("toll_detected", self.perception.detect_toll())
                self.shared_state.set("danger_level", obstacle_data.get("level", 0))

                # 3. Planning
                perception_data = {
                    "lane_offset": self.shared_state.get("lane_offset"),
                    "nav_direction": self.shared_state.get("nav_direction"),
                    "obstacle": obstacle_data,
                    "danger_level": obstacle_data.get("level", 0),
                    "toll_detected": self.shared_state.get("toll_detected"),
                }
                telemetry_data = self.shared_state.get("telemetry", {})

                current_state, voice_alert = self.planner.update(
                    perception_data, telemetry_data, delta_time)
                self.shared_state.set("system_state",
                                      getattr(current_state, "name", str(current_state)))
                self.shared_state.set("active_blinker", self.planner.active_blinker)
                if voice_alert:
                    self.voice.say(voice_alert)

                # 4. Core modules + plugin supervision
                self.module_manager.update_all(delta_time)
                self.plugin_manager.tick(delta_time)

                # 5. Apply control intents to the device (safety-gated)
                self._flush_controls()
            except Exception as e:
                logging.error("Engine frame error (recovered): %s", e)

            # 6. Maintain target FPS
            sleep_time = (1.0 / self.fps) - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    engine = UltraPilotEngine()
    try:
        engine.start()
    except KeyboardInterrupt:
        engine.stop()
