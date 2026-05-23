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
from core.planner import UltraPilotPlanner

class UltraPilotEngine:
    """The main engine for ETS2-UltraPilot."""

    def __init__(self):
        logging.basicConfig(level=logging.INFO)
        logging.info("Starting ETS2-UltraPilot Engine...")

        self.settings = SettingsManager()
        self.shared_state = SharedState()
        self.voice = VoiceAssistant()

        self.telemetry = Telemetry()
        self.controller = Controller()
        self.perception = Perception()
        self.planner = UltraPilotPlanner()

        self.module_manager = ModuleManager(self)
        self.plugin_manager = PluginManager(self)

        # Register Core Modules
        self.module_manager.register_module(GameWatcher)

        self.running = False
        self.fps = self.settings.get("general", {}).get("fps", 60)

    def start(self):
        self.running = True
        self.plugin_manager.discover_and_load()

        self.run_loop()

    def stop(self):
        self.running = False
        self.module_manager.stop_all()
        self.plugin_manager.stop_all()
        self.voice.stop()
        logging.info("ETS2-UltraPilot Engine stopped.")

    def run_loop(self):
        last_time = time.time()
        while self.running:
            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time

            # 1. Update Telemetry and push to Shared State
            if self.telemetry.update():
                self.shared_state.update_batch({
                    "telemetry": self.telemetry.data,
                    "speed": self.telemetry.get("truck", {}).get("speed", 0)
                })

            # 2. Update Perception and push to Shared State
            self.shared_state.set("nav_direction", self.perception.detect_navigation_arrow())
            self.shared_state.set("lane_offset", self.perception.detect_lanes())
            self.shared_state.set("danger_level", self.perception.detect_obstacles())
            self.shared_state.set("toll_detected", self.perception.detect_toll())

            # 3. Planning Layer
            perception_data = {
                "lane_offset": self.shared_state.get("lane_offset"),
                "nav_direction": self.shared_state.get("nav_direction"),
                "danger_level": self.shared_state.get("danger_level"),
                "toll_detected": self.shared_state.get("toll_detected")
            }
            telemetry_data = self.shared_state.get("telemetry", {})

            current_state, voice_alert = self.planner.update(perception_data, telemetry_data, delta_time)
            self.shared_state.set("system_state", current_state)

            if voice_alert:
                self.voice.say(voice_alert)

            # 4. Update Core Modules (includes GameWatcher)
            self.module_manager.update_all(delta_time)

            # 4. Plugin Manager tick
            self.plugin_manager.tick(delta_time)

            # 5. Maintain FPS
            sleep_time = (1.0 / self.fps) - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

if __name__ == "__main__":
    engine = UltraPilotEngine()
    try:
        engine.start()
    except KeyboardInterrupt:
        engine.stop()
