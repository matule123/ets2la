import json
import os
import logging
from typing import Any, Dict

class SettingsManager:
    """Handles persistence and access to application settings."""

    def __init__(self, filename: str = "settings.json"):
        # Resolve relative to the app dir so it works frozen and from source.
        if not os.path.isabs(filename):
            from core.paths import app_dir
            filename = os.path.join(app_dir(), filename)
        self.filename = filename
        self.settings: Dict[str, Any] = {}
        self.load()

    def load(self):
        """Load settings from disk."""
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r') as f:
                    self.settings = json.load(f)
                logging.info("Settings loaded successfully.")
            except Exception as e:
                logging.error(f"Error loading settings: {e}")
                self.settings = {}
        else:
            self.settings = self._get_defaults()
            self.save()

    def save(self):
        """Save current settings to disk."""
        try:
            with open(self.filename, 'w') as f:
                json.dump(self.settings, f, indent=4)
            logging.info("Settings saved to disk.")
        except Exception as e:
            logging.error(f"Error saving settings: {e}")

    def get(self, key: str, default: Any = None) -> Any:
        """Retrieve a setting value."""
        return self.settings.get(key, default)

    def set(self, key: str, value: Any):
        """Set a setting value and save to disk."""
        self.settings[key] = value
        self.save()

    def _get_defaults(self) -> Dict[str, Any]:
        """Default settings for the first run."""
        return {
            "general": {
                "target_speed": 80.0,
                "fps": 60,
                "dark_mode": True
            },
            "autopilot": {
                "enabled": False,
                "kp": 0.3,
                "ki": 0.01,
                "kd": 0.1
            },
            "hud": {
                "enabled": True,
                "color": "lime",
                "position": [100, 100]
            },
            # Per-plugin enable map (folder name -> bool). Missing = enabled.
            "plugins": {
                "autopilot": True,
                "acc": True,
                "collision": True,
                "map": True,
                "tts": True,
                # The AR-style HUD plugin emits overlay data with no renderer; the
                # on-screen HUD is core/hud.py instead, so keep this plugin off.
                "hud": False,
                "ecodrive": False,
                "discord": False
            },
            # Onboarding / first-run state. ``onboarded`` is false until the
            # setup wizard finishes; ``ui_language_code`` is the ISO code of the
            # selected UI language (sk, en, …); ``selected_map`` is the dataset
            # key (e.g. "ets2-1.59") chosen in the wizard.
            "onboarded": False,
            "ui_language_code": "sk",
            "selected_map": "",
            # Startup chime (plays assets/sounds/boot.mp3 if present).
            "startup_sound": True,
        }
