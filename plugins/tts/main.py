import logging
import pyttsx3
import queue
import threading
from sdk.base_plugin import BasePlugin


class _SpeechDispatcher:
    """One process-wide pyttsx3 loop; every utterance is serialized."""

    def __init__(self):
        self._queue = queue.Queue(maxsize=32)
        self._thread = threading.Thread(
            target=self._run, name="UltraPilot-TTS", daemon=True)
        self._thread.start()

    def submit(self, text):
        text = str(text).strip()
        if not text:
            return
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            logging.warning("TTS queue full; dropping duplicate/late message")

    def _run(self):
        try:
            engine = pyttsx3.init()
        except Exception as exc:
            logging.error("Failed to initialize TTS engine: %s", exc)
            return
        while True:
            text = self._queue.get()
            try:
                engine.say(text)
                engine.runAndWait()
            except Exception as exc:
                # A failed utterance must not kill the single dispatcher; the
                # next queued message can still be spoken.
                logging.error("TTS speaking error: %s", exc)
            finally:
                self._queue.task_done()


_dispatcher = None
_dispatcher_lock = threading.Lock()


def _get_dispatcher():
    global _dispatcher
    with _dispatcher_lock:
        if _dispatcher is None or not _dispatcher._thread.is_alive():
            _dispatcher = _SpeechDispatcher()
        return _dispatcher

class Plugin(BasePlugin):
    """TTS plugin for voiced announcements and accessibility."""

    def on_start(self):
        logging.info("TTS Plugin started.")
        try:
            self._dispatcher = _get_dispatcher()
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
        """Queue text without starting another pyttsx3 run loop."""
        if not self.enabled:
            return
        logging.info(f"TTS Speaking: {text}")
        _get_dispatcher().submit(text)

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
        # Zero/negative limits are transient SDK sentinels, not real signs.
        # The captured run announced -4 km/h and started a second utterance at
        # exactly the moment another voice message was active.
        if speed_limit > 0.5 and abs(speed_limit - self.last_speed_limit) > 1:
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
