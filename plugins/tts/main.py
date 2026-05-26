import logging
import pyttsx3
import threading
from sdk.base_plugin import BasePlugin

class Plugin(BasePlugin):
    """TTS plugin for voiced announcements and accessibility."""

    def on_start(self):
        logging.info("TTS Plugin started.")
        try:
            self.engine = pyttsx3.init()
            self.enabled = True
            self.last_speed_limit = 0
            self.last_fuel_notification = 0
            self.last_damage_notification = 0
            logging.info("TTS engine initialized successfully.")
        except Exception as e:
            logging.error(f"Failed to initialize TTS engine: {e}")
            self.enabled = False

    def on_stop(self):
        logging.info("TTS Plugin stopped.")
        self.enabled = False

    def speak(self, text: str):
        """Speak the given text using pyttsx3 in a separate thread to avoid blocking."""
        if not self.enabled:
            return
        logging.info(f"TTS Speaking: {text}")
        def _say():
            try:
                local_engine = pyttsx3.init()
                local_engine.say(text)
                local_engine.runAndWait()
            except Exception as e:
                logging.error(f"TTS speaking error: {e}")

        threading.Thread(target=_say, daemon=True).start()

    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        # 1. Monitor Shared State for Messages
        msg = self.sdk.shared_state.get("tts_message")
        if msg:
            self.speak(msg)
            self.sdk.shared_state.set("tts_message", None)

        # 2. Telemetry Data
        truck = self.sdk.telemetry.get("truck", {})
        if not truck:
            return

        speed_limit = truck.get("speedLimit", 0)
        fuel_range = truck.get("fuelRange", 0)

        # 3. Speed Limit Notifications
        if abs(speed_limit - self.last_speed_limit) > 1:
            self.last_speed_limit = speed_limit
            self.speak(f"Speed limit updated to {round(speed_limit * 3.6)} kilometers per hour.")

        # 4. Fuel Notifications
        if fuel_range < 50:
            current_time = self.sdk.shared_state.get("system_time", 0)
            if current_time - self.last_fuel_notification > 600:
                self.speak(f"Warning: Critical fuel level. {round(fuel_range)} kilometers remaining.")
                self.last_fuel_notification = current_time

    def announce(self, text: str):
        """Method for other plugins to trigger a voice announcement."""
        self.speak(text)
