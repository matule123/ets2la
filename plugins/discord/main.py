from sdk.base_plugin import BasePlugin
import logging
import time

class DiscordPlugin(BasePlugin):
    """
    Discord Rich Presence plugin for ETS2-UltraPilot.
    Shows current status, speed, and autopilot state on Discord.
    """
    def on_start(self):
        logging.info("Discord Rich Presence Plugin started.")
        try:
            from pypresence import Presence
            self.presence = Presence("YOUR_CLIENT_ID_HERE") # User will need to provide this
            self.presence.connect()
            self.enabled = True
        except Exception as e:
            logging.error(f"Failed to initialize Discord Rich Presence: {str(e)}")
            self.enabled = False

    def on_stop(self):
        if hasattr(self, 'presence'):
            self.presence.close()
        logging.info("Discord Rich Presence Plugin stopped.")

    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        # Fetch data from SDK
        speed = self.sdk.telemetry.get("truck", {}).get("speed", 0)
        speed_kmh = speed * 3.6 if speed < 200 else speed

        system_state = self.sdk.shared_state.get("system_state", "IDLE")
        state_text = system_state.name if hasattr(system_state, 'name') else str(system_state)

        try:
            self.presence.update(
                set_status=True,
                state=f"Speed: {speed_kmh:.1f} km/h",
                details=f"SCS: {state_text}",
                start=time.time(),
                assets_large="ultra_pilot_logo", # Assumes asset exists in Discord Dev Portal
                large_image="ultra_pilot_logo"
            )
        except Exception as e:
            logging.debug(f"Discord update failed: {e}")
