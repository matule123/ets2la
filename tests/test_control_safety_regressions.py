import os
import sys
import unittest

from PyQt6.QtCore import QPointF

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from core.ar_overlay import _perspective_route_widths, _segment_is_occluded
from core.hud import (
    HUD_CAMERA_BACK_M, HUD_EGO_AHEAD_M, HUD_ROAD_BEHIND_M, UltraPilotHUD,
    _clip_truck_road_segment,
)
from plugins.autopilot.main import Plugin as AutopilotPlugin
from plugins.lanecontrol.main import Plugin as LaneControlPlugin
from sdk.plugin_sdk import _ControllerProxy, CTL_SELECT_DRIVE


class State:
    def __init__(self, values=None):
        self.values = dict(values or {})

    def get(self, key, default=None):
        return self.values.get(key, default)

    def set(self, key, value):
        self.values[key] = value


class Controller:
    def __init__(self):
        self.steering = self.throttle = self.brake = 0.0

    def set_steering(self, value): self.steering = value
    def set_throttle(self, value): self.throttle = value
    def set_brake(self, value): self.brake = value
    def select_drive(self, pressed=True): self.drive = pressed; return True


class Telemetry:
    def __init__(self, truck): self.truck = truck
    def get(self, key, default=None):
        return self.truck if key == "truck" else default


class Tags:
    pass


def autopilot(truck, state):
    plugin = AutopilotPlugin.__new__(AutopilotPlugin)
    plugin.sdk = type("SDK", (), {})()
    plugin.sdk.shared_state = state
    plugin.sdk.controller = Controller()
    plugin.sdk.telemetry = Telemetry(truck)
    plugin.tags = Tags()
    plugin.on_start()
    return plugin


class ControlSafetyRegressionTests(unittest.TestCase):
    def test_plugin_controller_proxy_supports_drive_selector(self):
        state = {}
        proxy = _ControllerProxy(state)
        self.assertTrue(proxy.select_drive(True))
        self.assertIs(state[CTL_SELECT_DRIVE], True)
        self.assertTrue(proxy.select_drive(False))
        self.assertIs(state[CTL_SELECT_DRIVE], False)

    def test_lanecontrol_accepts_authoritative_xyz_path(self):
        plugin = LaneControlPlugin.__new__(LaneControlPlugin)
        plugin.sdk = type("SDK", (), {})()
        plugin.sdk.shared_state = State({
            "nav_path": [(0.0, 12.0, 0.0), (8.0, 12.0, -35.0)],
            "truck_world_pos": (0.0, 0.0), "truck_heading": 0.0,
        })
        # Regression for "too many values to unpack (expected 2)".
        self.assertIsInstance(plugin._route_lateral_hint(), (float, type(None)))

    def test_reverse_gear_is_recovered_without_disengaging(self):
        state = State({"autopilot_active": True})
        truck = {"speed": -0.5, "gear": -1}
        plugin = autopilot(truck, state)
        plugin.on_tick(0.05)
        self.assertTrue(state.get("autopilot_active"))
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        truck["speed"] = 0.0
        plugin.sdk.telemetry.truck = truck
        plugin._drive_request_t = -1.0
        plugin.on_tick(0.05)
        self.assertTrue(plugin.sdk.controller.drive)
        truck["gear"] = 1
        plugin.on_tick(0.05)
        self.assertFalse(plugin._reverse_recovery)
        self.assertFalse(plugin.sdk.controller.drive)

    def test_neutral_selects_drive_before_throttle(self):
        state = State({"autopilot_active": True})
        plugin = autopilot({"speed": 0.0, "gear": 0}, state)
        plugin.on_tick(0.05)
        self.assertTrue(state.get("autopilot_active"))
        self.assertTrue(plugin.sdk.controller.drive)
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        self.assertEqual(plugin.sdk.controller.brake, 0.0)

    def test_arrival_stops_and_disengages(self):
        state = State({
            "autopilot_active": True, "navigation_arrival_pending": True,
            "game_route_distance": 3.0,
        })
        plugin = autopilot({"speed": 0.1, "gear": 1}, state)
        plugin.on_tick(0.05)
        self.assertFalse(state.get("autopilot_active"))
        self.assertFalse(state.get("nav_active"))
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        self.assertEqual(plugin.sdk.controller.brake, 0.0)
        self.assertEqual(state.get("navigation_status"), "Cieľ dosiahnutý")

    def test_ar_width_is_compact_original_size(self):
        snapshot = {
            "viewport": {"width": 1920}, "fov_horizontal_deg": 75.0,
        }
        halo, core = _perspective_route_widths(50.0, snapshot)
        self.assertGreater(core, 4.0)
        self.assertLess(core, 8.0)
        self.assertGreater(halo, core)

    def test_nearer_vehicle_occludes_route_segment(self):
        rects = [(40.0, 40.0, 60.0, 60.0, 10.0)]
        self.assertTrue(_segment_is_occluded(
            QPointF(45.0, 50.0), QPointF(55.0, 50.0), 20.0, rects))
        self.assertFalse(_segment_is_occluded(
            QPointF(45.0, 50.0), QPointF(55.0, 50.0), 5.0, rects))

    def test_hud_ego_and_road_share_the_telemetry_origin(self):
        self.assertEqual(HUD_EGO_AHEAD_M, 0.0)
        self.assertGreater(HUD_CAMERA_BACK_M, 40.0)
        hud = UltraPilotHUD.__new__(UltraPilotHUD)
        hud._view_yaw = 0.0

        class View:
            def height(self): return 500.0
            def top(self): return 0.0
            def center(self): return QPointF(400.0, 250.0)

        road_origin = UltraPilotHUD._project(hud, 0.0, 0.0, View())
        ego_origin = UltraPilotHUD._project(
            hud, HUD_EGO_AHEAD_M, 0.0, View())
        self.assertEqual(road_origin, ego_origin)

    def test_hud_road_continues_behind_complete_tractor_trailer(self):
        self.assertGreaterEqual(HUD_ROAD_BEHIND_M, 40.0)
        clipped = _clip_truck_road_segment((-60.0, 0.0), (-10.0, 0.0))
        self.assertIsNotNone(clipped)
        first, second, _t0, _t1 = clipped
        self.assertAlmostEqual(first[0], -HUD_ROAD_BEHIND_M)
        self.assertEqual(second, (-10.0, 0.0))


if __name__ == "__main__":
    unittest.main()
