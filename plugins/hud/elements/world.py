from plugins.hud.main import HUDElement
from core.sdk.projection import ARProjection

class Widget(HUDElement):
    """World-to-HUD Road and Vehicle Rendering."""
    def __init__(self, plugin):
        super().__init__(plugin)
        self.projector = ARProjection()

    def draw(self, offset_x, width, height):
        # Get data from shared state
        vehicles = self.plugin.sdk.shared_state.get("nearby_vehicles", [])
        roads = self.plugin.sdk.shared_state.get("road_geometry", [])
        truck_pos = self.plugin.sdk.telemetry.get("truck", {}).get("pos", (0, 0, 0))
        truck_rot = self.plugin.sdk.shared_state.get("truck_rot", (0, 0, 0))

        # 1. Draw Roads / Lane Lines
        for road_segment in roads:
            # segment = {"start": (x,y,z), "end": (x,y,z), "color": (r,g,b,a)}
            p1 = self.projector.project(truck_pos, truck_rot, road_segment["start"])
            p2 = self.projector.project(truck_pos, truck_rot, road_segment["end"])

            if p1 and p2:
                self.data.append({
                    "type": "Line",
                    "start": p1,
                    "end": p2,
                    "color": road_segment["color"],
                    "width": 2
                })

        # 2. Draw Vehicles
        for veh in vehicles:
            # veh = {"pos": (x,y,z), "color": (r,g,b,a)}
            screen_pos = self.projector.project(truck_pos, truck_rot, veh["pos"])
            if screen_pos:
                sx, sy = screen_pos
                self.data.append({
                    "type": "Rectangle",
                    "rect": [sx - 10, sy - 10, sx + 10, sy + 10],
                    "color": veh["color"]
                })
                self.data.append({
                    "type": "Text",
                    "pos": [sx - 10, sy - 20],
                    "text": "VEHICLE",
                    "size": 12,
                    "color": (255, 255, 255, 255)
                })
