import math
import struct
import time
import unittest
from unittest import mock

from core.engine import UltraPilotEngine
from core.navigation.lane_model import LaneId, LanePath, LanePoint, LaneSegment
from core.sdk.ets2la_data import (
    ETS2LAData, _SEM_FMT, _SEM_SIZE, nearest_light_ahead,
)
from plugins.drivepolicy.main import Plugin as DrivePolicyPlugin
from plugins.map.main import Plugin as MapPlugin
from plugins.turnsignals.main import Plugin as TurnSignalsPlugin
import plugins.tts.main as tts_module


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

    def test_turnsignals_use_only_proven_turn_events(self):
        plugin = TurnSignalsPlugin.__new__(TurnSignalsPlugin)
        plugin._route_key = None
        plugin._route = None
        path = [[0.0, 5.0, 0.0], [0.0, 5.0, -15.0],
                [3.0, 5.0, -30.0], [12.0, 5.0, -60.0]]
        snapshot = {"revision": 4, "route_build_id": "build"}
        right = [{"start_s_m": 15.0, "end_s_m": 70.0,
                  "direction": "right"}]
        self.assertEqual(plugin._signal_for_events(
            (0.0, 0.0), 0.0, path, right, snapshot), "right")
        # A geometrically curved road without a semantic junction event is not
        # an instruction to signal.
        self.assertEqual(plugin._signal_for_events(
            (0.0, 0.0), 0.0, path, [], snapshot), "off")

    def test_map_publishes_turn_event_only_for_prefab_topology(self):
        lane_id = LaneId(10, 1, 0, prefab_token="junction",
                         connector_index=0, connector_path=(1,))
        headings = (0.0, -0.20, -0.40, -0.60)
        points = tuple(LanePoint(
            float(index * 3), 4.0, float(-index * 8), float(index * 9),
            heading, lane_id=lane_id, segment_index=0)
            for index, heading in enumerate(headings))
        segment = LaneSegment(
            lane_id, 1, 2, 1, 0, 1, 4.0, "dataset", 0, "look",
            "prefab", points)
        lane_path = LanePath((segment,), points, (1, 2), 27.0,
                             0.95, True, "", 4)
        plugin = MapPlugin.__new__(MapPlugin)
        events = plugin._turn_events_payload(lane_path)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["direction"], "right")
        self.assertAlmostEqual(events[0]["start_s_m"], 0.0)
        self.assertAlmostEqual(events[0]["end_s_m"], 27.0)

        road = LaneSegment(
            LaneId(10, 1, 0), 1, 2, 1, 0, 1, 4.0, "dataset", 0,
            "look", "road", points)
        self.assertEqual(plugin._turn_events_payload(
            LanePath((road,), points, (1, 2), 27.0, 0.95, True)), [])

    def test_turnsignal_remains_available_while_stopped_before_turn(self):
        points = [[0.0, 5.0, 0.0], [0.0, 5.0, -15.0],
                  [3.0, 5.0, -30.0], [12.0, 5.0, -60.0]]
        state = _State({
            "autopilot_active": True, "truck_speed_ms": 0.0,
            "truck_world_pos": (0.0, 0.0), "truck_heading": 0.0,
            "system_state": "DRIVING", "lane_trajectory_revision": 4,
            "lane_trajectory": {
                "valid": True, "revision": 4, "display_points": points,
                "turn_events": [{"start_s_m": 15.0, "end_s_m": 70.0,
                                 "direction": "right"}],
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
        # The plugin publishes one request; only Engine owns the physical
        # momentary blinker button and therefore cannot double-toggle it.
        self.assertEqual(controller.events, [])
        self.assertEqual(state.get("route_blinker"), "right")

        state.set("autopilot_active", False)
        plugin.on_tick(0.1)
        self.assertEqual(state.get("route_blinker"), "off")
        self.assertTrue(state.get("lane_change_safe"))

    def test_tts_dispatcher_serializes_rapid_messages_on_one_run_loop(self):
        class Engine:
            def __init__(self):
                self.messages = []
                self.running = False
                self.overlap = False

            def say(self, text):
                self.messages.append(text)

            def runAndWait(self):
                if self.running:
                    self.overlap = True
                self.running = True
                time.sleep(0.005)
                self.running = False

        engine = Engine()
        with mock.patch.object(tts_module.pyttsx3, "init",
                               return_value=engine) as initialize:
            dispatcher = tts_module._SpeechDispatcher()
            dispatcher.submit("first")
            dispatcher.submit("second")
            dispatcher._queue.join()
        initialize.assert_called_once_with()
        self.assertEqual(engine.messages, ["first", "second"])
        self.assertFalse(engine.overlap)

    def test_tts_ignores_negative_speed_limit_sentinel(self):
        values = {"tts_message": None}
        plugin = tts_module.Plugin.__new__(tts_module.Plugin)
        plugin.enabled = True
        plugin.last_speed_limit = 16.7
        plugin.last_fuel_notification = 0
        plugin.sdk = type("SDK", (), {
            "shared_state": _State(values),
            "telemetry": type("Telemetry", (), {
                "get": lambda _self, key, default=None: {
                    "speedLimit": -1.0, "fuelRange": 500.0,
                } if key == "truck" else default,
            })(),
        })()
        with mock.patch.object(plugin, "speak") as speak:
            plugin.on_tick(0.1)
        speak.assert_not_called()
        self.assertAlmostEqual(plugin.last_speed_limit, 16.7)

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
