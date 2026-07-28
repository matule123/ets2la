import unittest

from core.engine import _live_route_suffix, _telemetry_loss_navigation_payload
from unittest import mock

from core.modules.game_watcher import GameWatcher


class _State:
    def __init__(self):
        self.data = {
            "autopilot_active": True, "nav_active": True,
            "game_gps_navigation_active": True,
            "recorded_route_active": True,
            "navigation_source": "recorded_route",
            "lane_trajectory_revision": 6,
            "lane_trajectory": {
                "revision": 6, "valid": True,
                "source_gps_uids": [10, 11],
                "points": [[0, 0, 0], [0, 0, 10]],
                "display_points": [[0, 0, 0], [0, 0, 10]],
            },
        }

    def set(self, key, value):
        self.data[key] = value

    def get(self, key, default=None):
        return self.data.get(key, default)

    def update_batch(self, values):
        self.data.update(values)


class _Controller:
    def __init__(self):
        self.released = 0

    def release_all(self):
        self.released += 1


class _Engine:
    def __init__(self):
        self.shared_state = _State()
        self.controller = _Controller()


class GameSessionResetTests(unittest.TestCase):
    def test_live_route_suffix_does_not_keep_passed_fork_arm(self):
        items = [
            {"uid": 10, "distance": 900.0},
            {"uid": 11, "distance": 600.0},
            {"uid": 12, "distance": 200.0},
            {"uid": 13, "distance": 0.0},
        ]
        suffix, passed, matched = _live_route_suffix(items, 210.0)
        self.assertEqual(passed, 2)
        self.assertEqual(matched, 200.0)
        self.assertEqual([item["uid"] for item in suffix], [12, 13])

    def test_game_close_disables_master_and_clears_route(self):
        engine = _Engine()
        watcher = GameWatcher(engine)
        watcher._reset_session(starting=False)
        self.assertFalse(engine.shared_state.data["autopilot_active"])
        self.assertFalse(engine.shared_state.data["nav_active"])
        self.assertEqual(engine.shared_state.data["game_route_node_uids"], [])
        self.assertFalse(engine.shared_state.data["game_gps_navigation_active"])
        self.assertFalse(engine.shared_state.data["recorded_route_active"])
        self.assertEqual(engine.shared_state.data["navigation_source"], "none")
        snapshot = engine.shared_state.data["lane_trajectory"]
        self.assertFalse(snapshot["valid"])
        self.assertEqual(snapshot["points"], [])
        self.assertEqual(snapshot["revision"], 7)
        self.assertEqual(engine.shared_state.data["lane_trajectory_revision"], 7)
        self.assertEqual(engine.controller.released, 1)

    def test_telemetry_loss_payload_atomically_invalidates_all_route_authority(self):
        state = _State()
        state.data.update({
            "game_route_node_uids": [10, 11],
            "nav_path": [[0, 0, 0], [0, 0, 10]],
            "nav_steering": 0.6,
        })

        payload = _telemetry_loss_navigation_payload(state)

        self.assertFalse(payload["game_gps_navigation_active"])
        self.assertFalse(payload["recorded_route_active"])
        self.assertEqual(payload["navigation_source"], "none")
        self.assertFalse(payload["lane_trajectory"]["valid"])
        self.assertEqual(payload["lane_trajectory"]["points"], [])
        self.assertEqual(payload["lane_trajectory_revision"], 7)
        self.assertEqual(payload["nav_path"], [])
        self.assertFalse(payload["nav_active"])
        self.assertEqual(payload["nav_steering"], 0.0)
        state.update_batch(payload)
        repeated = _telemetry_loss_navigation_payload(state)
        self.assertEqual(repeated["lane_trajectory_revision"], 7)

        # A malformed mixed state must be repaired, not reused as a new
        # revision paired with an older snapshot body.
        state.data["lane_trajectory_revision"] = 8
        repaired = _telemetry_loss_navigation_payload(state)
        self.assertEqual(repaired["lane_trajectory_revision"], 9)
        self.assertEqual(repaired["lane_trajectory"]["revision"], 9)

    @mock.patch("core.sdk.game_utils.get_version_for_game", return_value="1.59")
    @mock.patch("core.sdk.game_utils.find_scs_games",
                return_value=[r"C:\\Steam\\Euro Truck Simulator 2"])
    @mock.patch("core.settings.manager.SettingsManager")
    def test_game_start_forces_selected_map_revalidation(
            self, manager, _find, _version):
        manager.return_value.get.return_value = "promods-1.59"
        engine = _Engine()
        watcher = GameWatcher(engine)
        watcher.session_id = 4
        watcher._reset_session(starting=True)
        self.assertEqual(engine.shared_state.data["installed_game_version"],
                         "1.59")
        self.assertEqual(engine.shared_state.data["nav_cmd"], "switch_map")
        self.assertEqual(engine.shared_state.data["nav_arg"], "promods-1.59")
        self.assertFalse(engine.shared_state.data["autopilot_active"])


if __name__ == "__main__":
    unittest.main()
