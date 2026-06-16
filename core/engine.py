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

    # --- Traffic following ----------------------------------------------------
    def _lead_brake(self, traffic, pos, heading):
        """Brake (0..1) for the closest vehicle ahead in our lane, else 0."""
        import math
        if not traffic or not pos:
            return 0.0
        px, pz = pos
        sin_h, cos_h = math.sin(heading), math.cos(heading)
        nearest = None
        for v in traffic:
            dx, dz = v["x"] - px, v["z"] - pz
            ahead = dx * (-sin_h) + dz * (-cos_h)
            lateral = dx * cos_h - dz * sin_h
            if 2.0 < ahead < 60.0 and abs(lateral) < 2.5:   # in our lane, in front
                if nearest is None or ahead < nearest:
                    nearest = ahead
        if nearest is None:
            return 0.0
        self.shared_state.set("lead_distance", nearest)
        # Gentle far away, firm when close: full brake under ~8 m.
        if nearest <= 8.0:
            return 1.0
        if nearest >= 45.0:
            return 0.0
        return float(max(0.0, min(1.0, (45.0 - nearest) / 37.0)))

    # --- Hotkey ---------------------------------------------------------------
    def _check_hotkey(self):
        """Toggle autopilot_active on a rising edge of the 'N' key (app-wide)."""
        if not self._has_win32:
            return
        try:
            import win32api
            down = bool(win32api.GetAsyncKeyState(self._hotkey_vk) & 0x8000)
        except Exception:
            return
        if down and not self._hotkey_was_down:
            new_state = not bool(self.shared_state.get("autopilot_active", False))
            self.shared_state.set("autopilot_active", new_state)
            msg = "Autopilot enabled." if new_state else "Autopilot disabled."
            logging.info("Hotkey N -> %s", msg)
            self.shared_state.set("tts_message", msg)
            if not new_state:
                self.controller.release_all()
        self._hotkey_was_down = down

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

            # 0. Global hotkey (toggle autopilot from inside the game)
            self._check_hotkey()

            # 1. Telemetry
            if self.telemetry.update():
                truck = self.telemetry.get("truck", {}) or {}
                self.shared_state.update_batch({
                    "telemetry": self.telemetry.data,
                    "speed": truck.get("speed", 0),
                    # World pose for coordinate-based navigation (map plugin).
                    "truck_world_pos": (truck.get("x", 0.0), truck.get("z", 0.0)),
                    "truck_heading": truck.get("rotation", 0.0),
                    "truck_speed_ms": truck.get("speed", 0.0),
                })

                # Surrounding traffic + the traffic light controlling us (ETS2LA plugin).
                try:
                    from core.sdk.ets2la_data import nearest_light_ahead
                    traffic = self.ets2la.read_traffic()
                    lights = self.ets2la.read_traffic_lights()
                    pos = (truck.get("x", 0.0), truck.get("z", 0.0))
                    hdg = truck.get("rotation", 0.0)
                    self.shared_state.set("traffic", traffic)
                    self.shared_state.set("traffic_light", nearest_light_ahead(lights, pos, hdg))
                    # Lead-vehicle following: brake for the nearest car ahead in our lane.
                    self.shared_state.set("traffic_brake", self._lead_brake(traffic, pos, hdg))
                except Exception:
                    pass

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
            # Store the state name (string) so other processes read it cleanly.
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
