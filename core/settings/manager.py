import json
import os
import logging
import tempfile
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
        """Save current settings atomically.

        The UI and the map worker both update settings.  Replacing a complete
        temporary file prevents a process interruption from leaving a partial
        JSON document that would forget the user's enabled plugins.
        """
        temporary = None
        try:
            directory = os.path.dirname(self.filename) or "."
            os.makedirs(directory, exist_ok=True)
            fd, temporary = tempfile.mkstemp(
                prefix=".settings-", suffix=".tmp", dir=directory)
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                json.dump(self.settings, f, indent=4)
                f.flush()
                os.fsync(f.fileno())
            os.replace(temporary, self.filename)
            temporary = None
            logging.info("Settings saved to disk.")
            return True
        except Exception as e:
            logging.error(f"Error saving settings: {e}")
            return False
        finally:
            if temporary:
                try:
                    os.unlink(temporary)
                except OSError:
                    pass

    def get(self, key: str, default: Any = None) -> Any:
        """Retrieve a setting value."""
        return self.settings.get(key, default)

    def set(self, key: str, value: Any):
        """Set a setting value and save to disk."""
        self.settings[key] = value
        self.save()

    def plugin_enabled(self, name: str, default: bool = True) -> bool:
        """Return the persisted state for one plugin."""
        plugins = self.settings.get("plugins", {}) or {}
        return bool(plugins.get(str(name), default))

    def set_plugin_enabled(self, name: str, enabled: bool):
        """Persist one plugin without discarding the other plugin choices."""
        previous = dict(self.settings.get("plugins", {}) or {})
        plugins = dict(previous)
        plugins[str(name)] = bool(enabled)
        self.settings["plugins"] = plugins
        if not self.save():
            self.settings["plugins"] = previous
            raise OSError("plugin setting could not be saved")

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
