from plugins.hud.main import HUDElement

class Widget(HUDElement):
    """Speedometer HUD Element."""
    def draw(self, offset_x, width, height):
        # Fetch telemetry
        truck = self.plugin.sdk.telemetry.get("truck", {})
        speed = truck.get("speed", 0)
        speed_kmh = speed * 3.6 if speed < 200 else speed

        # Render background
        self.data.append({
            "type": "Rectangle",
            "rect": [offset_x, 100, offset_x + width, 100 + height],
            "color": (0, 0, 0, 150)
        })

        # Render text
        self.data.append({
            "type": "Text",
            "pos": [offset_x + 10, 110],
            "text": f"{int(speed_kmh)} km/h",
            "size": 20,
            "color": (0, 255, 0, 255)
        })
