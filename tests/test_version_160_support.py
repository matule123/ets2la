import unittest

from core.navigation import map_data
from core.sdk import sdk_downloader


class Version160SupportTests(unittest.TestCase):
    def setUp(self):
        self.old_index = map_data._index_cache
        map_data._index_cache = {
            key: dict(value) for key, value in
            map_data.COMPATIBILITY_DATASETS.items()
        }

    def tearDown(self):
        map_data._index_cache = self.old_index

    def test_ets2_160_and_promods_283_are_distinct_datasets(self):
        datasets = {item["key"]: item for item in map_data.list_datasets()}
        self.assertEqual(datasets["ets2-1.60"]["game_version"], "1.60")
        self.assertEqual(datasets["promods-2.83"]["game_version"], "1.60")
        self.assertEqual(datasets["promods-2.83"]["mod_version"], "2.83")
        self.assertEqual(map_data.suggest_key("1.60"), "ets2-1.60")
        self.assertEqual(map_data.suggest_key("1.60", prefer_promods=True),
                         "promods-2.83")

    def test_sdk_160_accepts_full_patch_version_and_route_plugin(self):
        self.assertTrue(sdk_downloader.is_supported("1.60"))
        self.assertTrue(sdk_downloader.is_supported("1.60.2.0"))
        self.assertIn("1.60", sdk_downloader.supported_versions())
        self.assertIn("ets2la_plugin.dll", sdk_downloader.SDK_FILES)


if __name__ == "__main__":
    unittest.main()
