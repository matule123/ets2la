import threading
import logging
from queue import Queue

try:
    import pyttsx3
except Exception:
    pyttsx3 = None


class VoiceAssistant:
    """
    Voice alert router for the Engine process.

    To avoid two competing pyttsx3 engines (this one *and* the ``tts`` plugin
    fighting over the audio device), the Engine no longer speaks directly.
    Instead, when given the shared state, it publishes alerts to ``tts_message``
    and the ``tts`` plugin — the single speaker — voices them.

    If no shared state is provided (e.g. standalone use) it falls back to an
    in-process pyttsx3 thread so callers still get audio.
    """

    def __init__(self, shared_state=None):
        self.shared_state = shared_state
        self.queue = Queue()
        self.running = True
        self.engine = None

        # Only spin up a local speech engine when we have nowhere to publish to.
        if shared_state is None and pyttsx3 is not None:
            try:
                self.engine = pyttsx3.init()
                self.engine.setProperty('rate', 170)
                self.engine.setProperty('volume', 0.9)
            except Exception as e:
                logging.error(f"Voice Assistant init failed, running silent: {e}")
                self.engine = None
            self.thread = threading.Thread(target=self._process_queue, daemon=True)
            self.thread.start()

        logging.info("Voice Assistant initialized (%s).",
                     "shared tts_message" if shared_state is not None else "local pyttsx3")

    def speak(self, text: str):
        """Publish an alert. The tts plugin (or local fallback) voices it."""
        if not text:
            return
        logging.info(f"Voice: {text}")
        if self.shared_state is not None:
            self.shared_state.set("tts_message", text)
        else:
            self.queue.put(text)

    # Alias used by the engine/planner.
    def say(self, text: str):
        self.speak(text)

    def _process_queue(self):
        while self.running:
            try:
                text = self.queue.get(timeout=1)
                if self.engine is not None:
                    self.engine.say(text)
                    self.engine.runAndWait()
                self.queue.task_done()
            except Exception:
                pass

    def stop(self):
        self.running = False
