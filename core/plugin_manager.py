import multiprocessing as mp
import os
import logging
import time
from typing import List, Type
from sdk.base_plugin import BasePlugin

def plugin_worker(plugin_class: Type[BasePlugin], sdk_proxy: Any, stop_event: mp.Event):
    """
    The entrypoint for a plugin running in its own process.
    """
    try:
        # Create the plugin instance inside the new process
        plugin = plugin_class(sdk_proxy)
        plugin.on_start()

        last_time = time.time()
        while not stop_event.is_set():
            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time

            if plugin.enabled:
                plugin.on_tick(delta_time)

            time.sleep(0.01) # High-frequency tick

        plugin.on_stop()
    except Exception as e:
        logging.error(f"Plugin process crashed: {e}")

class PluginManager:
    """
    Advanced Plugin Manager using Multiprocessing.
    Each plugin runs in its own OS process to prevent the main app from crashing.
    """
    def __init__(self, engine):
        self.engine = engine
        self.processes: List[mp.Process] = []
        self.stop_events: List[mp.Event] = []
        self.plugin_configs: List[Type[BasePlugin]] = []
        self.plugin_dir = "plugins"

    def discover_and_load(self):
        """Finds plugins and spawns processes."""
        logging.info("Spawning plugin processes...")

        # For this implementation, we'll map the folder structure to classes
        # In a real system, we'd dynamically import the module.
        # Here, we'll manually register the known plugins for stability.
        from plugins.autopilot.main import Plugin as AutopilotPlugin
        from plugins.hud.main import Plugin as HudPlugin

        self.plugin_configs = [AutopilotPlugin, HudPlugin]

        for plugin_class in self..plugin_configs:
            stop_event = mp.Event()
            # Pass the shared state as the SDK proxy
            proc = mp.Process(
                target=plugin_worker,
                args=(plugin_class, self.engine.shared_state, stop_event),
                daemon=True
            )
            proc.start()
            self.processes.append(proc)
            self.stop_events.append(stop_event)
            logging.info(f"Process started for {plugin_class.__name__}")

    def tick(self, delta_time: float):
        """Core loop no longer ticks plugins, they tick themselves in their own processes."""
        pass

    def stop_all(self):
        for event in self.stop_events:
            event.set()
        for proc in self.processes:
            proc.join(timeout=1)
            proc.terminate()
