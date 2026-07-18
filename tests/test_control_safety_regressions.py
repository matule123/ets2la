import os
import sys
import unittest

from PyQt6.QtCore import QPointF

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from core.ar_overlay import _perspective_route_widths, _segment_is_occluded
from plugins.autopilot.main import Plugin as AutopilotPlugin
from plugins.lanecontrol.main import Plugin as LaneControlPlugin


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
    def test_lanecontrol_accepts_authoritative_xyz_path(self):
        plugin = LaneControlPlugin.__new__(LaneControlPlugin)
        plugin.sdk = type("SDK", (), {})()
        plugin.sdk.shared_state = State({
            "nav_path": [(0.0, 12.0, 0.0), (8.0, 12.0, -35.0)],
            "truck_world_pos": (0.0, 0.0), "truck_heading": 0.0,
        })
        # Regression for "too many values to unpack (expected 2)".
        self.assertIsInstance(plugin._route_lateral_hint(), (float, type(None)))

    def test_reverse_gear_disengages_without_throttle(self):
        state = State({"autopilot_active": True})
        plugin = autopilot({"speed": -0.5, "gear": -1}, state)
        plugin.on_tick(0.05)
        self.assertFalse(state.get("autopilot_active"))
        self.assertEqual(plugin.sdk.controller.throttle, 0.0)
        self.assertFalse(state.get("nav_active"))

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

    def test_ar_width_is_half_lane_in_perspective(self):
        snapshot = {
            "viewport": {"width": 1920}, "fov_horizontal_deg": 75.0,
        }
        halo, core = _perspective_route_widths(50.0, snapshot)
        self.assertGreater(core, 35.0)
        self.assertGreater(halo, core)

    def test_nearer_vehicle_occludes_route_segment(self):
        rects = [(40.0, 40.0, 60.0, 60.0, 10.0)]
        self.assertTrue(_segment_is_occluded(
            QPointF(45.0, 50.0), QPointF(55.0, 50.0), 20.0, rects))
        self.assertFalse(_segment_is_occluded(
            QPointF(45.0, 50.0), QPointF(55.0, 50.0), 5.0, rects))


if __name__ == "__main__":
    unittest.main()
