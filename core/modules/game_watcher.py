import psutil
import logging
import time
from core.modules.base_module import BaseModule

class GameWatcher(BaseModule):
    """
    Module that monitors if Euro Truck Simulator 2 is running.
    Can be used to auto-start or auto-stop the autopilot.
    """
    def __init__(self, engine):
        super().__init__(engine)
        self.process_name = "eurotrucks2.exe"
        self.is_game_running = False

    def on_start(self):
        logging.info("Game Watcher started. Monitoring for eurotrucks2.exe...")

    def on_stop(self):
        logging.info("Game Watcher stopped.")

    def update(self, delta_time: float):
        # Check if the game process is active
        was_running = self.is_game_running
        self.is_game_running = self._check_process()

        if self.is_game_running and not was_running:
            logging.info("ETS2 detected! Preparing autopilot...")
            self.engine.voice.speak("Euro Truck Simulator 2 detected. UltraPilot is ready.")
            # We could potentially auto-start the engine here
        elif not self.is_game_running and was_running:
            logging.info("ETS2 closed. Putting autopilot to sleep...")
            self.engine.voice.speak("Game closed. UltraPilot entering standby.")

    def _check_process(self) -> bool:
        for proc in psutil.process_iter(['name']):
            try:
                if proc.info['name'].lower() == self.process_name:
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return False
