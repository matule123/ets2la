from abc import ABC, abstractmethod
import logging

class BaseModule(ABC):
    """
    Base class for core system modules.
    Unlike plugins, modules are critical components of the UltraPilot core.
    """

    def __init__(self, engine):
        self.engine = engine
        self.enabled = True

    @abstractmethod
    def on_start(self):
        """Initialize the module."""
        pass

    @abstractmethod
    def on_stop(self):
        """Clean up the module."""
        pass

    def update(self, delta_time: float):
        """Optional update loop."""
        pass
