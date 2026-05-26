import logging
import importlib
import os
from typing import List, Dict, Any
from sdk.base_plugin import BasePlugin

class HUDElement:
    """Base class for all HUD elements."""
    def __init__(self, plugin):
        self.plugin = plugin
        self.data = [] # List of AR primitives (Rectangle, Text, etc.)
        self.enabled = True
        self.fps = 30

    def draw(self, offset_x: float, width: float, height: float):
        """Override this to define the visual output."""
        pass

class HUDManager(BasePlugin):
    """
    Professional HUD Manager.
    Discovers HUD elements and aggregates their data for the AR rendering engine.
    """
    def __init__(self, sdk_proxy):
        super().__init__(sdk_proxy)
        self.elements: List[HUDElement] = []
        self.enabled = True
        self.widget_scaling = 1.0
        self.padding = 10

    def on_start(self):
        logging.info("HUD Manager started. Discovering elements...")
        self.discover_elements()

    def on_stop(self):
        logging.info("HUD Manager stopped.")

    def discover_elements(self):
        """Dynamically loads elements from the plugins/hud/elements directory."""
        elements_path = os.path.join(os.path.dirname(__file__), "elements")
        if not os.path.exists(elements_path):
            return

        for filename in os.listdir(elements_path):
            if filename.endswith(".py") and filename != "__init__.py":
                module_name = f"plugins.hud.elements.{filename[:-3]}"
                try:
                    module = importlib.import_module(module_name)
                    # Look for a class named 'Widget' in the module
                    if hasattr(module, "Widget"):
                        element_class = getattr(module, "Widget")
                        self.elements.append(element_class(self))
                        logging.info(f"HUD: Loaded element {module_name}")
                except Exception as e:
                    logging.error(f"HUD: Failed to load element {module_name}: {e}")

    def calculate_layout(self):
        """Calculates the positions and sizes of all active widgets."""
        active_elements = [e for e in self.elements if e.enabled]
        if not active_elements:
            return []

        total_width = 0
        element_configs = []

        for e in active_elements:
            # Default width of 100px per widget
            w = 100 * self.widget_scaling
            total_width += w + self.padding
            element_configs.append({"element": e, "width": w})

        # Center the HUD on the screen (hypothetical screen width 1920)
        start_x = (1920 - total_width) / 2

        final_layout = []
        current_x = start_x
        for config in element_configs:
            final_layout.append({
                "element": config["element"],
                "offset_x": current_x,
                "width": config["width"],
                "height": 50 * self.widget_scaling
            })
            current_x += config["width"] + self.padding

        return final_layout

    def on_tick(self, delta_time: float):
        if not self.enabled:
            return

        layout = self.calculate_layout()
        aggregated_ar_data = []

        for item in layout:
            element = item["element"]
            element.draw(item["offset_x"], item["width"], item["height"])
            aggregated_ar_data.extend(element.data)
            element.data = [] # Clear for next frame

        # Push everything to the AR system
        self.tags.AR = aggregated_ar_data
        self.tags.hud_status = "Active"
