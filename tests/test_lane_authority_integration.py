import unittest
import time

from core.ar_overlay import AROverlay
from core.hud import UltraPilotHUD
from core.navigation.route import Route
from plugins.autopilot.main import Plugin as AutopilotPlugin
from plugins.map.main import Plugin as MapPlugin
from tests.test_lane_route_builder import SyntheticMap


class State:
    def __init__(self, values=None):
        self.data = dict(values or {})

    def get(self, key, default=None):
        return self.data.get(key, default)

    def set(self, key, value):
        self.data[key] = value

    def update_batch(self, values):
        self.data.update(values)


class MapSDK:
    def __init__(self, values=None):
        self.shared_state = State(values)

    def get(self, key, default=None):
        return self.shared_state.get(key, default)

    def set(self, key, value):
        self.shared_state.set(key, value)


class Controller:
    def __init__(self):
        self.steering = self.throttle = self.brake = 0.0

    def set_steering(self, value): self.steering = value
    def set_throttle(self, value): self.throttle = value
    def set_brake(self, value): self.brake = value
    def set_blinker(self, value): pass
    def pay_toll(self): pass


class Tags:
    pass


class Telemetry:
    def get(self, key, default=None):
        return {"speed": 15.0} if key == "truck" else default


def build_map_plugin(y=3.0):
    synthetic = SyntheticMap()
    synthetic.node(1, 0, 0, y)
    synthetic.node(2, 0, 40, y)
    synthetic.node(3, 0, 80, y)
    first = synthetic.road(1, 2, 2)
    synthetic.road(2, 3, 2)
    lane = synthetic.net._build_lane_segments(first)[0]
    point = lane.centerline[2]
    sdk = MapSDK({
        "game_route_node_uids": [1, 2, 3],
        "truck_altitude": y,
        "lane_trajectory_revision": 0,
    })
    plugin = MapPlugin(sdk)
    plugin.on_start()
    plugin.road_net = synthetic.net
    plugin._net_attempted = True
    plugin._update_lane_trajectory((point.x, point.z), point.heading)
    return plugin, sdk, point


class LaneAuthorityIntegrationTests(unittest.TestCase):
    def test_one_snapshot_drives_controller_hud_ar_and_compatibility(self):
        plugin, sdk, _ = build_map_plugin()
        snapshot = sdk.get("lane_trajectory")
        self.assertTrue(snapshot["valid"], snapshot["failure_reason"])
        self.assertEqual(plugin._lane_route.world_points,
                         [tuple(point) for point in snapshot["points"]])
        self.assertEqual(snapshot["display_points"], snapshot["points"])
        self.assertEqual(sdk.get("nav_path"), snapshot["display_points"])
        self.assertEqual(sdk.get("nav_trajectory_revision"), snapshot["revision"])

        hud = type("HUDReader", (), {
            "shared_state": sdk.shared_state,
            "_rear_cam_side": "off", "_rear_cam_until": 0.0,
        })()
        hud_data = UltraPilotHUD._read(hud)
        ar = type("ARReader", (), {"state": sdk.shared_state})()
        ar_revision, ar_points = AROverlay._current_display_points(ar)
        self.assertIs(hud_data["nav_path"], snapshot["display_points"])
        self.assertIs(ar_points, snapshot["display_points"])
        self.assertEqual(hud_data["lane_revision"], snapshot["revision"])
        self.assertEqual(ar_revision, snapshot["revision"])

    def test_destination_change_removes_old_revision_and_unproven_route(self):
        plugin, sdk, point = build_map_plugin()
        old_revision = sdk.get("lane_trajectory")["revision"]
        sdk.set("game_route_node_uids", [1, 99])
        plugin._update_lane_trajectory((point.x, point.z), point.heading)
        snapshot = sdk.get("lane_trajectory")
        self.assertGreater(snapshot["revision"], old_revision)
        self.assertFalse(snapshot["valid"])
        self.assertEqual(snapshot["points"], [])
        self.assertEqual(sdk.get("nav_path"), [])
        self.assertFalse(sdk.get("nav_active"))

    def test_xyz_and_vertical_layers_are_preserved(self):
        plugin, sdk, _ = build_map_plugin(y=12.0)
        snapshot = sdk.get("lane_trajectory")
        self.assertTrue(all(len(point) == 3 for point in snapshot["points"]))
        self.assertTrue(all(abs(point[1] - 12.0) < 1e-6
                            for point in snapshot["points"]))
        route = Route(snapshot["points"])
        self.assertEqual(route.world_points[0][1], 12.0)
        self.assertEqual(route.points[0],
                         (snapshot["points"][0][0], snapshot["points"][0][2]))

    def test_legacy_recorded_route_cannot_override_live_gps_snapshot(self):
        plugin, sdk, point = build_map_plugin()
        snapshot = sdk.get("lane_trajectory")
        plugin.tags = Tags()
        plugin.active_route = Route([[100, 0, 0], [100, 0, 100]], "legacy")
        plugin.on_tick(0.02)
        self.assertEqual(sdk.get("nav_path"), snapshot["display_points"])
        self.assertEqual(sdk.get("nav_trajectory_revision"), snapshot["revision"])

    def test_stale_or_invalid_snapshot_hides_hud_and_ar(self):
        plugin, sdk, _ = build_map_plugin()
        sdk.set("lane_trajectory_revision",
                sdk.get("lane_trajectory_revision") + 1)
        hud = type("HUDReader", (), {
            "shared_state": sdk.shared_state,
            "_rear_cam_side": "off", "_rear_cam_until": 0.0,
        })()
        ar = type("ARReader", (), {"state": sdk.shared_state})()
        self.assertEqual(UltraPilotHUD._read(hud)["nav_path"], [])
        self.assertEqual(AROverlay._current_display_points(ar), (-1, []))

    def test_low_confidence_and_invalid_route_brake_and_center_smoothly(self):
        for confidence, valid in ((0.50, True), (0.95, False)):
            state = State({
                "system_state": "CRUISE", "danger_level": 0.0,
                "lane_offset": 0.8, "traffic": [], "nav_active": True,
                "nav_steering": 0.7, "acc_throttle": 0.6,
                "acc_brake": 0.0, "autopilot_active": True,
                "game_route_distance": 100.0,
                "game_route_node_uids": [1, 2],
                "lane_trajectory_heartbeat": time.monotonic(),
                "lane_trajectory_revision": 7,
                "lane_trajectory": {
                    "revision": 7, "valid": valid, "confidence": confidence,
                    "source_gps_uids": [1, 2], "points": [[0, 0, 0], [0, 0, 10]],
                    "display_points": [[0, 0, 0], [0, 0, 10]],
                },
            })
            sdk = type("SDK", (), {})()
            sdk.shared_state, sdk.controller, sdk.telemetry = state, Controller(), Telemetry()
            plugin = AutopilotPlugin(sdk)
            plugin.tags = Tags()
            plugin.on_start()
            plugin._last_steering = 0.5
            plugin._last_throttle = 0.5
            plugin._engage_blend = 1.0
            plugin._was_active = True
            plugin.on_tick(0.1)
            self.assertGreater(plugin._last_steering, 0.0)
            self.assertLess(plugin._last_steering, 0.5)
            self.assertGreater(plugin._last_brake, 0.0)
            self.assertLess(plugin._last_brake, 0.70)
            self.assertEqual(state.get("autopilot_lane_revision"), -1)


if __name__ == "__main__":
    unittest.main()
