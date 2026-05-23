from abc import ABC, abstractmethod
from typing import Any, Dict
import multiprocessing as mp

class BasePlugin(ABC):
    """
    Base class for all UltraPilot plugins.
    Now designed to run in a separate Process for stability and performance.
    """

    def __init__(self, sdk_proxy: Any):
        self.sdk = sdk_proxy # Proxy to the engine's shared state and controllers
        self.enabled = False
        self.config: Dict[str, Any] = {}

    @abstractmethod
    def on_start(self):
        """Called when the plugin process starts."""
        pass

    @abstractmethod
    def on_stop(self):
        """Called when the plugin process is terminated."""
        pass

    @abstractmethod
    def on_tick(self, delta_time: float):
        """The main execution loop for the plugin."""
        pass

    def update_config(self, new_config: Dict[str, Any]):
        self.config.update(new_config)
