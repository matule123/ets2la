import math
import struct
import time
import unittest

from core.engine import UltraPilotEngine
from core.sdk.ets2la_data import (
    ETS2LAData, _SEM_FMT, _SEM_SIZE, nearest_light_ahead,
)
from plugins.drivepolicy.main import Plugin as DrivePolicyPlugin
from plugins.turnsignals.main import Plugin as TurnSignalsPlugin


class _State:
    def __init__(self, values=None):
        self.values = dict(values or {})

    def get(self, key, default=None):
        return self.values.get(key, default)

    def set(self, key, value):
        self.values[key] = value


class _BlinkerController:
    def __init__(self):
        self.events = []

    def set_blinker(self, side):
        self.events.append(side)


class PolicySignalLightRegressionTests(unittest.TestCase):
    def test_drivepolicy_accepts_authoritative_xyz_path(self):
        values = {
            "truck_world_pos": (0.0, 0.0), "truck_heading": 0.0,
            "nav_path": [[0.0, 4.0, -10.0], [1.0, 4.1, -20.0],
                         [5.0, 4.2, -35.0], [12.0, 4.3, -55.0],
                         [24.0, 4.4, -80.0]],
        }
        plugin = DrivePolicyPlugin.__new__(DrivePolicyPlugin)
        plugin.sdk = type("SDK", (), {
            "get": lambda _self, key, default=None: values.get(key, default),
        })()
        profile = plugin._curve_profile()
        self.assertEqual(len(profile), 5)
        self.assertTrue(any(strength > 0.0 for _, _, strength in profile))

    def test_turnsignals_accept_xyz_and_use_ets2_right_hand_sign(self):
        plugin = TurnSignalsPlugin.__new__(TurnSignalsPlugin)
        right = [[0.0, 5.0, -15.0], [3.0, 5.0, -30.0],
                 [12.0, 5.0, -60.0]]
        left = [[0.0, 5.0, -15.0], [-3.0, 5.0, -30.0],
                [-12.0, 5.0, -60.0]]
        self.assertEqual(plugin._signal_for_path((0.0, 0.0), 0.0, right),
                         "right")
        self.assertEqual(plugin._signal_for_path((0.0, 0.0), 0.0, left),
                         "left")

    def test_turnsignal_remains_available_while_stopped_before_turn(self):
        points = [[0.0, 5.0, -15.0], [3.0, 5.0, -30.0],
                  [12.0, 5.0, -60.0]]
        state = _State({
            "autopilot_active": True, "truck_speed_ms": 0.0,
            "truck_world_pos": (0.0, 0.0), "truck_heading": 0.0,
            "system_state": "DRIVING", "lane_trajectory_revision": 4,
            "lane_trajectory": {
                "valid": True, "revision": 4, "display_points": points,
            },
        })
        controller = _BlinkerController()
        plugin = TurnSignalsPlugin.__new__(TurnSignalsPlugin)
        plugin.sdk = type("SDK", (), {
            "shared_state": state, "controller": controller,
            "set": lambda _self, key, value: state.set(key, value),
        })()
        plugin.tags = type("Tags", (), {})()
        plugin.on_start()
        plugin.on_tick(0.1)
        self.assertEqual(controller.events[-1], "right")
        self.assertEqual(state.get("route_blinker"), "right")

    def test_semaphore_tile_coordinates_become_absolute_world_coordinates(self):
        entries = [0] * (13 * 40)
        entries[:13] = [
            6.5, 59.25, 8.0, 83, 121,
            1.0, 0.0, 0.0, 0.0, 1, 7.5, 2, 99,
        ]
        payload = struct.pack(_SEM_FMT, *entries)
        self.assertEqual(len(payload), _SEM_SIZE)
        reader = ETS2LAData()
        reader._traffic_buf = object()
        reader._parked_buf = object()
        reader._sem_buf = bytearray(payload)
        lights = reader.read_traffic_lights()
        self.assertEqual(len(lights), 1)
        self.assertAlmostEqual(lights[0]["x"], 42502.5)
        self.assertAlmostEqual(lights[0]["z"], 61960.0)
        self.assertAlmostEqual(lights[0]["y"], 59.25)
        self.assertEqual(lights[0]["color"], "red")

    def test_nearest_light_rejects_closer_crossing_arm(self):
        lights = [
            {"x": 0.0, "z": -10.0, "yaw": math.pi / 2,
             "color": "green"},
            {"x": 4.0, "z": -30.0, "yaw": math.pi,
             "color": "red"},
        ]
        selected = nearest_light_ahead(lights, (0.0, 0.0), 0.0)
        self.assertIsNotNone(selected)
        self.assertEqual(selected["color"], "red")
        self.assertAlmostEqual(selected["distance"], 30.0)

    def test_absolute_red_light_produces_brake_request(self):
        selected = nearest_light_ahead([
            {"x": 42506.0, "z": 61930.0, "yaw": math.pi,
             "color": "red"},
        ], (42502.0, 61960.0), 0.0)
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = _State({"truck_speed_ms": 12.0})
        self.assertGreater(engine._light_brake(selected), 0.0)

    def test_red_hold_cannot_be_bypassed_by_stale_or_distant_lead(self):
        state = _State({"truck_speed_ms": 0.0, "lead_distance": 40.0})
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = state
        self.assertEqual(engine._light_brake(
            {"color": "red", "distance": 8.0}), 1.0)
        self.assertEqual(engine._lead_brake([], (0.0, 0.0), 0.0), 0.0)
        self.assertIsNone(state.get("lead_distance"))


if __name__ == "__main__":
    unittest.main()
