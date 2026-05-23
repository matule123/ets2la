import logging
from typing import List
from core.modules.base_module import BaseModule

class ModuleManager:
    """Manages core system modules."""

    def __init__(self, engine):
        self.engine = engine
        self.modules: List[BaseModule] = []

    def register_module(self, module_class):
        """Instantiate and register a module."""
        try:
            module = module_class(self.engine)
            self.modules.append(module)
            module.on_start()
            logging.info(f"Module {module_class.__name__} registered and started.")
        except Exception as e:
            logging.error(f"Failed to register module {module_class.__name__}: {e}")

    def stop_all(self):
        for module in self.modules:
            module.on_stop()

    def update_all(self, delta_time: float):
        for module in self.modules:
            if module.enabled:
                module.update(delta_time)
