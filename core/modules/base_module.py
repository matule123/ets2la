from abc import ABC, abstractmethod
import logging

class BaseModule(ABC):
    """
    Base class for all UltraPilot core modules.
    Ensures a consistent lifecycle for data acquisition and processing.
    """
    def __init__(self, engine):
        self.engine = engine
        self.enabled = True
        self.name = self.__class__.__name__
        logging.info(f"Module {self.name} initialized.")

    @abstractmethod
    def on_start(self):
        """Called when the module is started."""
        pass

    @abstractmethod
    def on_stop(self):
        """Called when the module is stopped."""
        pass

    @abstractmethod
    def run(self, delta_time: float = 0.0):
        """Main execution loop for the module."""
        pass

    def toggle(self):
        self.enabled = not self.enabled
        logging.info(f"Module {self.name} {'enabled' if self.enabled else 'disabled'}.")
