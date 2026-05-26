from plugins.hud.main import HUDElement

class Widget(HUDElement):
    """Gear Indicator HUD Element."""
    def draw(self, offset_x, width, height):
        # Fetch telemetry
        truck = self.plugin.sdk.telemetry.get("truck", {})
        gear = truck.get("gear", 0)

        # Render background
        self.data.append({
            "type": "Rectangle",
            "rect": [offset_x, 100, offset_x + width, 100 + height],
            "color": (0, 0, 0, 150)
        })

        # Render text
        gear_text = f"G: {gear}" if gear != 0 else "N"
        self.data.append({
            "type": "Text",
            "pos": [offset_x + 20, 115],
            "text": gear_text,
            "size": 24,
            "color": (255, 255, 0, 255)
        })
