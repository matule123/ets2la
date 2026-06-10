import logging
import time
from sdk.base_plugin import BasePlugin


class Plugin(BasePlugin):
    """
    Discord Rich Presence plugin for ETS2-UltraPilot.
    Shows current status, speed and autopilot state on Discord.
    """

    NAME = "discord"
    DEFAULT_ENABLED = False  # opt-in (needs a client id)
    CLIENT_ID = "YOUR_CLIENT_ID_HERE"
    UPDATE_INTERVAL = 15.0   # Discord rate-limits presence updates to ~5/min

    def on_start(self):
        logging.info("Discord Rich Presence Plugin started.")
        self.presence = None
        self._last_update = 0.0
        self._start_ts = time.time()
        try:
            from pypresence import Presence
            self.presence = Presence(self.CLIENT_ID)
            self.presence.connect()
        except Exception as e:
            logging.error(f"Failed to initialize Discord Rich Presence: {e}")
            self.enabled = False

    def on_stop(self):
        if self.presence is not None:
            try:
                self.presence.close()
            except Exception:
                pass
        logging.info("Discord Rich Presence Plugin stopped.")

    def on_tick(self, delta_time: float):
        if self.presence is None:
            return
        now = time.time()
        if now - self._last_update < self.UPDATE_INTERVAL:
            return
        self._last_update = now

        speed = self.sdk.telemetry.get("truck", {}).get("speed", 0) or 0
        speed_kmh = speed * 3.6 if speed < 200 else speed
        state = self.sdk.shared_state.get("system_state", "IDLE")
        state_text = state.name if hasattr(state, 'name') else str(state)

        try:
            self.presence.update(
                state=f"Speed: {speed_kmh:.0f} km/h",
                details=f"UltraPilot: {state_text}",
                start=self._start_ts,
                large_image="ultra_pilot_logo",
            )
        except Exception as e:
            logging.debug(f"Discord update failed: {e}")
