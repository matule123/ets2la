import multiprocessing as mp
import os
import logging
import time
import importlib
import inspect
from typing import Any, Type, Dict

from sdk.base_plugin import BasePlugin
from sdk.plugin_sdk import PluginSDK


def _shutdown_transport_error(error):
    """Return whether Manager/pipe IPC disappeared during normal shutdown."""
    text = f"{type(error).__name__}: {error}".lower()
    return any(marker in text for marker in (
        "brokenpipe", "broken pipe", "eoferror", "pipe has been ended",
        "pipe is being closed", "winerror 109", "winerror 232",
        "multiprocessing.managers.remoteerror", "keyerror",
    ))


def plugin_worker(plugin_class: Type[BasePlugin], plugin_name: str,
                  shared_dict: Dict[str, Any], stop_event):
    """
    Entrypoint for a plugin running in its own process.

    The PluginSDK is built *inside* the new process from the shared managed
    dict, so the plugin gets a fully wired SDK (telemetry / controller proxy /
    tags / settings) without anything unpicklable crossing the process boundary.
    """
    # Each plugin process is a fresh Python interpreter, so it has NO logging
    # setup inherited from the parent — without this, every ``logging.info`` call
    # inside a plugin (e.g. the autopilot diagnostics) is silently dropped and
    # never reaches ultrapilot.log. Configure a file handler that appends to the
    # same log the engine uses, tagged with the plugin name.
    try:
        from core.paths import app_dir
        log_path = os.path.join(app_dir(), "ultrapilot.log")
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s,%(msecs)03d %(levelname)-8s " + plugin_name + " %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
            filename=log_path,
            filemode="a",
        )
    except Exception:
        logging.basicConfig(level=logging.INFO)

    try:
        sdk = PluginSDK(shared_dict, plugin_name)
        plugin = plugin_class(sdk)
        plugin.on_start()

        tick_dt = 0.01  # 100 Hz
        last_time = time.time()
        while not stop_event.is_set():
            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time

            # Allow the UI to enable/disable plugins live via shared state.
            live_enabled = shared_dict.get(f"plugin_enabled.{plugin_name}", None)
            if live_enabled is not None:
                plugin.enabled = bool(live_enabled)

            if plugin.enabled:
                try:
                    plugin.on_tick(delta_time)
                except Exception as e:
                    if stop_event.is_set() or _shutdown_transport_error(e):
                        break
                    # Include the failing source line. A message-only exception
                    # made route failures look like a frozen progress indicator.
                    logging.exception(f"[plugin:{plugin_name}] on_tick error: {e}")

            time.sleep(tick_dt)

        plugin.on_stop()
    except Exception as e:
        import traceback
        if stop_event.is_set() or _shutdown_transport_error(e):
            logging.info("[plugin:%s] stopped during application shutdown.",
                         plugin_name)
        else:
            logging.error(f"[plugin:{plugin_name}] process crashed: {e}\n{traceback.format_exc()}")


class PluginManager:
    """
    Multiprocessing plugin manager with crash supervision.

    Each plugin runs in its own process.  If a plugin process dies unexpectedly
    while the engine is still running, it is automatically restarted, so a
    single misbehaving plugin can never permanently break the autopilot.
    """

    def __init__(self, engine):
        self.engine = engine
        from core.paths import app_dir
        self.plugin_dir = os.path.join(app_dir(), "plugins")
        # folder -> {class, process, stop_event}
        self.plugins: Dict[str, Dict[str, Any]] = {}
        self._published_processes = None

    # --- Discovery ------------------------------------------------------------
    def _find_plugin_class(self, folder: str) -> Type[BasePlugin]:
        """Import plugins.<folder>.main and return its plugin class.

        Accepts a class literally named ``Plugin`` or any BasePlugin subclass
        (so e.g. ``DiscordPlugin`` is found too)."""
        module = importlib.import_module(f"plugins.{folder}.main")
        if hasattr(module, "Plugin"):
            return getattr(module, "Plugin")
        for _, obj in inspect.getmembers(module, inspect.isclass):
            if issubclass(obj, BasePlugin) and obj is not BasePlugin:
                return obj
        raise ImportError(f"No BasePlugin subclass found in plugins.{folder}.main")

    def _enabled_in_settings(self, folder: str) -> bool:
        plugins_cfg = self.engine.settings.get("plugins", {}) or {}
        # Default: enabled unless explicitly turned off in settings.json.
        return bool(plugins_cfg.get(folder, True))

    def discover_and_load(self):
        logging.info("Discovering plugins...")
        if not os.path.isdir(self.plugin_dir):
            logging.error(f"Plugin directory not found: {self.plugin_dir}")
            return

        for folder in sorted(os.listdir(self.plugin_dir)):
            full = os.path.join(self.plugin_dir, folder)
            if not os.path.isdir(full) or folder.startswith("__"):
                continue
            if not os.path.exists(os.path.join(full, "main.py")):
                continue
            if not self._enabled_in_settings(folder):
                logging.info(f"Plugin '{folder}' disabled in settings, skipping.")
                continue
            try:
                plugin_class = self._find_plugin_class(folder)
                self.plugins[folder] = {"class": plugin_class, "process": None, "stop_event": None}
                self._spawn(folder)
                logging.info(f"Loaded plugin: {folder} ({plugin_class.__name__})")
            except Exception as e:
                logging.error(f"Failed to load plugin '{folder}': {e}")

    # --- Process management ----------------------------------------------------
    def _spawn(self, folder: str):
        entry = self.plugins[folder]
        stop_event = mp.Event()
        proc = mp.Process(
            target=plugin_worker,
            args=(entry["class"], folder, self.engine.shared_state.raw, stop_event),
            name=f"Plugin-{folder}",
            daemon=True,
        )
        proc.start()
        entry["process"] = proc
        entry["stop_event"] = stop_event
        self._publish_processes()

    def _publish_processes(self):
        """Expose exact plugin PIDs for the Performance UI on Windows."""
        processes = {}
        for name, entry in self.plugins.items():
            proc = entry.get("process")
            if proc is not None and proc.pid and proc.is_alive():
                processes[name] = int(proc.pid)
        if processes != self._published_processes:
            self._published_processes = dict(processes)
            self.engine.shared_state.set("plugin_processes", processes)

    def tick(self, delta_time: float):
        """Supervise plugin processes; restart any that died unexpectedly."""
        if not self.engine.running:
            return
        for folder, entry in self.plugins.items():
            proc = entry["process"]
            stop_event = entry["stop_event"]
            if proc is not None and not proc.is_alive() and stop_event is not None \
                    and not stop_event.is_set():
                logging.warning(f"Plugin '{folder}' died — restarting.")
                self._spawn(folder)
        self._publish_processes()

    def stop_all(self):
        for entry in self.plugins.values():
            if entry["stop_event"]:
                entry["stop_event"].set()
        for entry in self.plugins.values():
            proc = entry["process"]
            if proc:
                proc.join(timeout=1)
                if proc.is_alive():
                    proc.terminate()
        self.engine.shared_state.set("plugin_processes", {})
