import unittest
from unittest import mock

from core.modules.game_watcher import GameWatcher


class _State:
    def __init__(self):
        self.data = {"autopilot_active": True, "nav_active": True}

    def set(self, key, value):
        self.data[key] = value

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
    def test_game_close_disables_master_and_clears_route(self):
        engine = _Engine()
        watcher = GameWatcher(engine)
        watcher._reset_session(starting=False)
        self.assertFalse(engine.shared_state.data["autopilot_active"])
        self.assertFalse(engine.shared_state.data["nav_active"])
        self.assertEqual(engine.shared_state.data["game_route_node_uids"], [])
        self.assertEqual(engine.controller.released, 1)

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
