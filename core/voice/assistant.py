import pyttsx3
import threading
import logging
from queue import Queue

class VoiceAssistant:
    """
    Professional TTS system for real-time driver alerts.
    Runs in a separate thread to prevent UI/Engine blocking.
    """
    def __init__(self):
        self.engine = pyttsx3.init()
        self.queue = Queue()
        self.running = True

        # Voice configuration
        self.engine.setProperty('rate', 170)
        self.engine.setProperty('volume', 0.9)

        self.thread = threading.Thread(target=self._process_queue, daemon=True)
        self.thread.start()
        logging.info("Voice Assistant initialized.")

    def speak(self, text: str):
        """Enqueue a message to be spoken."""
        logging.info(f"Voice: {text}")
        self.queue.put(text)

    def _process_queue(self):
        while self.running:
            try:
                text = self.queue.get(timeout=1)
                self.engine.say(text)
                self.engine.runAndWait()
                self.queue.task_done()
            except Exception:
                pass

    def stop(self):
        self.running = False
