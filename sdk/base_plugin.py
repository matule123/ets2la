from typing import Any, Dict


class BasePlugin:
    """
    Base class for all UltraPilot plugins.

    Each plugin runs in its own OS process for stability (a crash in one plugin
    cannot take down the engine).  The plugin receives a :class:`PluginSDK`
    instance which exposes telemetry, the control proxy, shared state, tags and
    settings through one consistent API.

    Lifecycle hooks (``on_start`` / ``on_tick`` / ``on_stop``) are optional —
    override only the ones you need.  This is intentionally *not* an ABC so a
    plugin that only implements ``on_tick`` still loads cleanly.
    """

    #: Override in a subclass to give the plugin a friendly name / default state.
    NAME: str = "plugin"
    DEFAULT_ENABLED: bool = True

    def __init__(self, sdk: Any):
        self.sdk = sdk                       # unified PluginSDK proxy
        self.tags = getattr(sdk, "tags", None)
        self.settings = getattr(sdk, "settings", None)
        self.enabled = self.DEFAULT_ENABLED
        self.config: Dict[str, Any] = {}

    # --- Lifecycle (override as needed) --------------------------------------
    def on_start(self):
        """Called once when the plugin process starts."""

    def on_stop(self):
        """Called when the plugin process is shutting down."""

    def on_tick(self, delta_time: float):
        """Main loop, called every frame while the plugin is enabled."""

    # --- Helpers --------------------------------------------------------------
    def update_config(self, new_config: Dict[str, Any]):
        self.config.update(new_config)

    def toggle(self):
        self.enabled = not self.enabled
        return self.enabled
