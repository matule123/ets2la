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
        self.session_id = 0

    def on_start(self):
        logging.info("Game Watcher started. Monitoring for eurotrucks2.exe...")

    def on_stop(self):
        logging.info("Game Watcher stopped.")

    def update(self, delta_time: float):
        # Check if the game process is active
        was_running = self.is_game_running
        self.is_game_running = self._check_process()

        if self.is_game_running and not was_running:
            self.session_id += 1
            self._reset_session(starting=True)
            logging.info("ETS2 detected! Preparing autopilot...")
            self.engine.voice.speak("Euro Truck Simulator 2 detected. UltraPilot is ready.")
            # We could potentially auto-start the engine here
        elif not self.is_game_running and was_running:
            self._reset_session(starting=False)
            logging.info("ETS2 closed. Putting autopilot to sleep...")
            self.engine.voice.speak("Game closed. UltraPilot entering standby.")

    def _reset_session(self, starting: bool):
        """Invalidate controls and routes across an ETS2 process restart.

        Steam can replace the executable when the beta branch is changed.  The
        old implementation kept ``autopilot_active`` and the previous map
        snapshot alive, producing a green button with no valid controller after
        returning from 1.60 to 1.59.
        """
        state = self.engine.shared_state
        self.engine.controller.release_all()
        reason = "game session restarted" if starting else "game closed"
        state.update_batch({
            "autopilot_active": False,
            "autopilot_disable_reason": reason,
            "nav_active": False,
            "nav_steering": 0.0,
            "map_path": [],
            "nav_path": [],
            "game_route_node_uids": [],
            "game_route_points": [],
            "game_route_meta": [],
            "navigation_unreliable": True,
            "game_session_id": self.session_id,
        })
        if not starting:
            state.set("navigation_status", "Cakam na spustenie hry")
            return

        version = "Unknown"
        try:
            from core.sdk.game_utils import find_scs_games, get_version_for_game
            for game_path in find_scs_games():
                if "Euro Truck Simulator 2" in game_path:
                    version = get_version_for_game(game_path)
                    break
        except Exception as exc:
            logging.warning("Game Watcher: version detection failed: %s", exc)
        state.set("installed_game_version", version)

        # Force the map plugin to revalidate/reload the selected dataset for
        # the executable that has just started.  It will fail closed on a
        # version mismatch instead of silently using whichever cache is first.
        try:
            from core.settings.manager import SettingsManager
            selected = (SettingsManager().get("selected_map") or "").strip()
        except Exception:
            selected = ""
        state.set("nav_arg", selected)
        state.set("nav_cmd", "switch_map")
        state.set("navigation_status", f"Overujem mapu pre ETS2 {version}")
        logging.info("Game Watcher: new session %d, ETS2 %s, selected map=%s; "
                     "autopilot reset and map revalidation requested.",
                     self.session_id, version, selected or "none")

    def _check_process(self) -> bool:
        for proc in psutil.process_iter(['name']):
            try:
                if proc.info['name'].lower() == self.process_name:
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return False
