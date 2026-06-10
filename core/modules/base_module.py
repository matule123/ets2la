import logging


class BaseModule:
    """
    Base class for all UltraPilot core modules.

    Modules run inside the Engine process and are ticked every frame via
    ``update(delta_time)``.  Hooks are optional (not abstract) so a module only
    needs to override what it uses.  ``run`` is kept as an alias of ``update``
    for backwards compatibility with older modules.
    """

    def __init__(self, engine):
        self.engine = engine
        self.enabled = True
        self.name = self.__class__.__name__
        logging.info(f"Module {self.name} initialized.")

    def on_start(self):
        """Called when the module is started."""

    def on_stop(self):
        """Called when the module is stopped."""

    def update(self, delta_time: float = 0.0):
        """Main execution loop for the module (called each frame)."""

    # Backwards-compatible alias.
    def run(self, delta_time: float = 0.0):
        return self.update(delta_time)

    def toggle(self):
        self.enabled = not self.enabled
        logging.info(f"Module {self.name} {'enabled' if self.enabled else 'disabled'}.")
