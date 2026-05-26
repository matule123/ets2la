from plugins.hud.main import HUDElement
from core.sdk.projection import ARProjection
import math

class Widget(HUDElement):
    """Traffic Light AR Element."""
    def __init__(self, plugin):
        super().__init__(plugin)
        self.projector = ARProjection()

    def draw(self, offset_x, width, height):
        # Get traffic lights from shared state (simulated here)
        lights = self.plugin.sdk.shared_state.get("traffic_lights", [])
        if not lights:
            return

        truck_pos = self.plugin.sdk.telemetry.get("truck", {}).get("pos", (0, 0, 0))
        truck_rot = self.plugin.sdk.shared_state.get("truck_rot", (0, 0, 0))

        for light in lights:
            # light = {"pos": (x, y, z), "state": "red", "time_left": 10}
            screen_pos = self.projector.project(truck_pos, truck_rot, light["pos"])

            if screen_pos:
                sx, sy = screen_pos

                # Render Light Circle
                color = (255, 0, 0, 255) if light["state"] == "red" else \
                         (255, 255, 0, 255) if light["state"] == "yellow" else \
                         (0, 255, 0, 255)

                self.data.append({
                    "type": "Circle",
                    "pos": [sx, sy],
                    "radius": 10,
                    "color": color
                })

                # Render Countdown
                self.data.append({
                    "type": "Text",
                    "pos": [sx - 10, sy - 20],
                    "text": f"{int(light['time_left'])}s",
                    "size": 14,
                    "color": (255, 255, 255, 255)
                })
