from plugins.hud.main import HUDElement

class Widget(HUDElement):
    """Fuel Gauge HUD Element."""
    def draw(self, offset_x, width, height):
        # Fetch telemetry
        truck = self.plugin.sdk.telemetry.get("truck", {})
        fuel = truck.get("fuel", 0) # Percent 0-1

        # Render background
        self.data.append({
            "type": "Rectangle",
            "rect": [offset_x, 100, offset_x + width, 100 + height],
            "color": (0, 0, 0, 150)
        })

        # Render fuel bar
        bar_width = width * fuel
        self.data.append({
            "type": "Rectangle",
            "rect": [offset_x + 5, 110, offset_x + 5 + bar_width, 130],
            "color": (0, 255, 0, 255) if fuel > 0.2 else (255, 0, 0, 255)
        })

        # Render text
        self.data.append({
            "type": "Text",
            "pos": [offset_x + 10, 135],
            "text": f"Fuel: {int(fuel * 100)}%",
            "size": 16,
            "color": (255, 255, 255, 255)
        })
