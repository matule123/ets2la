import importlib
import sys
import unittest
from unittest import mock

from PyQt6.QtCore import QPoint, Qt
from PyQt6.QtWidgets import (QApplication, QWidget, QFrame, QLabel, QStatusBar,
                             QPushButton)

from core.theme import palette, stylesheet

# The installed build exposes the historical lowercase ``ui`` package name;
# the source checkout directory is ``UI`` and may be case-sensitive in CI.
sys.modules.setdefault("ui", importlib.import_module("UI"))
from UI import app as app_module
from UI.app import (MacTitleBar, UltraPilotApp, rounded_window_region,
                    window_control_notch_path, DashboardPage, AboutPage,
                    PluginsPage)
from UI.icons import line_icon
from UI.map_page import MapPage
from UI.dynamic_island import DynamicIsland
from UI.perf_overlay import PerfOverlay
from UI.settings_menu import SettingsMenu
from UI.onboarding import OnboardingWizard, _LangRow


class State:
    def __init__(self, values=None):
        self.values = dict(values or {})

    def get(self, key, default=None):
        return self.values.get(key, default)

    def set(self, key, value):
        self.values[key] = value

    def update_batch(self, values):
        self.values.update(values)


class UiChromeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_window_controls_match_reference_order_and_have_real_hitboxes(self):
        host = QWidget()
        bar = MacTitleBar(host, palette("light"))
        self.assertEqual(list(bar.controls), ["maximize", "minimize", "close"])
        controls = list(bar.controls.values())
        self.assertTrue(all(button.width() == 9 and button.height() == 9
                            for button in controls))
        self.assertEqual([button._color.name().upper() for button in controls],
                         ["#00CA4E", "#FFBD44", "#FF5F57"])
        self.assertTrue(all("border:none" in button.styleSheet()
                            for button in controls))
        self.assertTrue(all(not hasattr(button, "_glyph")
                            for button in controls))
        host.show()
        bar.show()
        self.app.processEvents()
        centres = [button.geometry().center().x() for button in controls]
        self.assertEqual([b-a for a, b in zip(centres, centres[1:])], [15, 15])
        bar.close()
        host.close()
        self.assertEqual([button.accessibleName() for button in controls],
                         ["Maximalizovať", "Minimalizovať", "Zavrieť"])

    def test_window_controls_are_seated_in_a_curved_notch(self):
        path = window_control_notch_path(68, 31)
        bounds = path.boundingRect()
        self.assertEqual((bounds.width(), bounds.height()), (68.0, 31.0))
        # The lower-left cutout is outside while all three control centres are
        # inside the painted surface.
        self.assertFalse(path.contains(bounds.bottomLeft()))
        for x in (21.5, 35.5, 49.5):
            self.assertTrue(path.contains(type(bounds.center())(x, 12.5)))

    def test_sidebar_uses_card_navigation_styles_and_original_line_icons(self):
        css = stylesheet("light")
        for selector in ("QPushButton#NavButton:checked",
                         "QFrame#SidebarUpdateCard",
                         "QFrame#SidebarStatusCard",
                         "QPushButton#SidebarPerformance"):
            self.assertIn(selector, css)
        for name in ("dashboard", "navigation", "visualization", "plugins",
                     "settings", "about", "performance", "autopilot",
                     "steering"):
            self.assertFalse(line_icon(name).isNull())

    def test_main_window_is_larger_rounded_and_has_no_bottom_status_bar(self):
        state = State({"ui_theme": "light", "ui_language_code": "sk"})
        with (mock.patch.object(app_module, "MapPage",
                                side_effect=lambda _state: QWidget()),
              mock.patch("UI.update_widget.UpdateCheckerWidget.check",
                         autospec=True)):
            window = UltraPilotApp(state)
        self.assertEqual((window.width(), window.height()), (1220, 760))
        self.assertGreaterEqual(window.minimumWidth(), 980)
        self.assertFalse(window.findChildren(QStatusBar))
        self.assertTrue(window.sidebar.isAncestorOf(window.start_btn))
        self.assertEqual(window.start_btn.height(), 48)
        self.assertEqual(window.start_btn.text(), "Zapnúť autopilota")
        self.assertEqual(window.centralWidget().objectName(), "WindowSurface")
        self.assertIn("border: 1px solid #AEB5BE", stylesheet("light"))
        region = rounded_window_region(1220, 760)
        self.assertFalse(region.contains(QPoint(0, 0)))
        self.assertTrue(region.contains(QPoint(610, 380)))
        window.close()

    def test_settings_use_responsive_cards_and_vector_header_icon(self):
        settings = SettingsMenu(State({"ui_theme": "light"}))
        cards = settings.findChildren(QFrame, "SettingsCard")
        self.assertEqual(len(cards), 4)
        labels = [label.text() for label in settings.findChildren(QLabel)]
        self.assertIn("Nastavenia", labels)
        self.assertNotIn("AR zobrazenie", labels)
        self.assertFalse(hasattr(settings, "ar_toggle"))
        self.assertFalse(any(text.startswith("⚙") for text in labels))
        self.assertIn("QWidget#SettingsPage", settings.styleSheet())
        settings.close()

    def test_onboarding_language_page_uses_two_column_status_cards(self):
        state = State({"ui_language_code": "sk"})
        wizard = OnboardingWizard(state)
        try:
            wizard._go_step(1)
            self.app.processEvents()
            self.assertGreaterEqual(len(wizard.lang_rows), 2)
            self.assertTrue(all(isinstance(row, _LangRow)
                                for row in wizard.lang_rows))
            self.assertEqual(wizard.lang_rows_grid.columnCount(), 2)
            self.assertEqual(wizard.lang_section.text(), "DOSTUPNÉ JAZYKY")
            for index, row in enumerate(wizard.lang_rows):
                self.assertEqual(
                    wizard.lang_rows_grid.getItemPosition(index)[1], index % 2)
                self.assertEqual(row.code_badge.text(), row.code.upper())
                self.assertGreaterEqual(row.minimumHeight(), 116)
        finally:
            wizard.close()

    def test_performance_popover_has_transparent_rounded_surface(self):
        overlay = PerfOverlay(State({"ui_theme": "light"}))
        self.assertTrue(overlay.testAttribute(
            Qt.WidgetAttribute.WA_TranslucentBackground))
        self.assertEqual(overlay.width(), 330)
        self.assertEqual(overlay.surface.objectName(), "PerfSurface")
        self.assertIsNotNone(overlay.surface.graphicsEffect())
        self.assertIn("QFrame#PerfSurface", overlay.styleSheet())
        labels = [label.text() for label in overlay.findChildren(QLabel)]
        self.assertIn("┌ Plugins", labels)
        self.assertTrue(any(text.startswith("└ Total:") for text in labels))
        overlay.close()

    def test_performance_rows_have_enough_height_and_are_not_clipped(self):
        plugins = [(f"Plugin {index}", 10.0 + index, 0.0)
                   for index in range(10)]
        with mock.patch("UI.perf_overlay._collect",
                        return_value=(512.0, 5.0, plugins)):
            overlay = PerfOverlay(State({"ui_theme": "light"}))
            overlay.refresh()
        self.assertEqual(overlay.rows_box.count(), 10)
        self.assertEqual(overlay.height(), 338)
        self.assertTrue(all(overlay.rows_box.itemAt(i).widget().height() == 23
                            for i in range(10)))
        overlay.close()

    def test_navigation_is_game_gps_first_and_live_map_badge_is_visible(self):
        with mock.patch.object(MapPage, "_populate_maps", autospec=True):
            page = MapPage(State({"ui_theme": "light"}))
        labels = [label.text() for label in page.findChildren(QLabel)]
        buttons = [button.text() for button in page.findChildren(QPushButton)]
        self.assertIn("●  LIVE", labels)
        self.assertFalse(any("Record" in text or "Recorded" in text
                             for text in labels + buttons))
        self.assertFalse(hasattr(page, "route_combo"))
        self.assertFalse(hasattr(page, "name_edit"))
        self.assertEqual(set(page.nav_stats), {"gps", "map", "trajectory"})
        self.assertTrue(page.view.empty_state.isVisibleTo(page.view))
        page.close()

    def test_live_map_preserves_road_style_metadata_and_load_progress(self):
        state = State({
            "ui_theme": "light",
            "map_load_progress": {
                "active": True, "percent": 77,
                "phase": "Načítavam prefaby a križovatky",
                "message": "Načítavam prefaby a križovatky — 77 %",
                "generation": 4,
            },
        })
        with mock.patch.object(MapPage, "_populate_maps", autospec=True):
            page = MapPage(state)
        page.view.set_road_segments([[
            [1.0, 2.0, 3.0], [4.0, 5.0, 3.0], "road", 4, True,
            True, False, False, 8.25, False, "r10:0", 3,
        ]])
        segment = page.view.road_segments[0]
        self.assertEqual(segment["lanes"], 4)
        self.assertTrue(segment["divided"])
        self.assertEqual(segment["half_width"], 8.25)
        self.assertEqual(segment["path_key"], "r10:0")
        page.refresh()
        self.assertFalse(page.dl_bar.isVisibleTo(page))
        self.assertEqual(page.dl_status.text(), "")
        self.assertEqual(len(page.view.map_controls), 3)
        page.close()

    def test_engine_map_loading_is_presented_by_dynamic_island_only(self):
        host = QWidget()
        host.state = State({
            "navigation_recalculating": True,
            "navigation_progress": 0.0,
            "navigation_status": "NaÄŤĂ­tavam GPS trasu",
            "map_load_progress": {
                "active": True, "percent": 77,
                "phase": "Načítavam prefaby a križovatky",
                "message": "Načítavam prefaby a križovatky — 77 %",
                "generation": 4,
            },
        })
        island = DynamicIsland(host)
        island._poll_log()
        self.assertEqual(island.time_lbl.text(), "MAPA")
        self.assertEqual(island.src_lbl.text(), "77%")
        self.assertIn("prefaby", island.msg_lbl.text())
        self.assertEqual(island.msg_lbl.alignment(), Qt.AlignmentFlag.AlignCenter)
        self.assertEqual(island.progress.value(), 77)
        host.state.set("map_load_progress", {
            "active": True, "percent": 100,
            "phase": "Mapa je pripravená", "generation": 4,
        })
        self.assertTrue(island._poll_map_load())
        self.assertEqual(island.src_lbl.text(), "100%")
        island.close()
        host.close()

    def test_failed_lane_localisation_has_no_misleading_percentage(self):
        host = QWidget()
        host.state = State({
            "navigation_recalculating": True,
            "navigation_progress": 0.72,
            "navigation_status": "Kamión nie je na potvrdenom GPS pruhu",
        })
        island = DynamicIsland(host)
        self.assertTrue(island._poll_navigation())
        self.assertEqual(island.src_lbl.text(), "")
        self.assertFalse(island.progress.isVisible())
        island.close()
        host.close()

    def test_live_map_scene_is_one_atomic_revision_with_maps_layers(self):
        state = State({
            "ui_theme": "light", "active_map_key": "promods-1.59",
            "truck_world_pos": (10.0, 20.0),
            "live_map_scene_revision": 9,
            "live_map_road_segments": [[
                [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], "road", 4,
                True, True, False, False, 8.0, False, "r1:0", 0,
                "freeway",
            ]],
            "live_map_scene_polygons": [[
                [[0.0, 0.0], [20.0, 0.0], [20.0, 12.0], [0.0, 12.0]],
                2, 2,
            ]],
            "live_map_scene_features": [[12.0, 4.0, "facility", "gas_ico", ""]],
        })
        with mock.patch.object(MapPage, "_populate_maps", autospec=True):
            page = MapPage(state)
        page.refresh()
        self.assertEqual(page.view.road_segments[0]["road_type"], "freeway")
        self.assertEqual(page.view.scene_polygons[0]["colour"], 2)
        self.assertEqual(page.view.scene_features[0]["icon"], "gas_ico")
        self.assertEqual(page._last_live_map_scene_revision, 9)
        self.assertEqual(page.view.zoom_radius, 650.0)
        page.close()

    def test_plugin_toggle_is_persisted_for_the_next_run(self):
        state = State({"ui_theme": "light", "plugin_enabled.acc": True})
        plugins = PluginsPage(state)
        action = next(
            button for button in plugins.findChildren(QPushButton)
            if button.objectName() == "PluginAction"
            and button.text() == "Vypnúť")
        manager = mock.Mock()
        with mock.patch("core.settings.manager.SettingsManager",
                        return_value=manager):
            action.click()
        self.app.processEvents()
        manager.set_plugin_enabled.assert_called_once_with("acc", False)
        self.assertFalse(state.get("plugin_enabled.acc"))
        plugins.close()

    def test_plugin_toggle_does_not_change_live_state_when_save_fails(self):
        state = State({"ui_theme": "light", "plugin_enabled.acc": True})
        plugins = PluginsPage(state)
        action = next(
            button for button in plugins.findChildren(QPushButton)
            if button.objectName() == "PluginAction"
            and button.text() == "Vypnúť")
        manager = mock.Mock()
        manager.set_plugin_enabled.side_effect = OSError("disk full")
        with (mock.patch("core.settings.manager.SettingsManager",
                         return_value=manager),
              mock.patch.object(app_module.logging, "error") as log_error):
            action.click()
        self.assertTrue(state.get("plugin_enabled.acc"))
        log_error.assert_called_once()
        plugins.close()

    def test_dashboard_and_about_use_clean_non_emoji_headers(self):
        state = State({"ui_theme": "light"})
        dashboard = DashboardPage(state)
        about = AboutPage(state)
        self.assertEqual(dashboard.title.text(), "Prehľad")
        self.assertNotIn("🚀", dashboard.title.text())
        self.assertEqual(about.title.text(), "O aplikácii")
        self.assertEqual(len(about.findChildren(QFrame, "AboutFeature")), 3)
        self.assertTrue(hasattr(dashboard, "route_status"))
        self.assertTrue(hasattr(dashboard, "safety_status"))
        dashboard.close()
        about.close()

    def test_plugin_page_header_has_no_emoji_and_slider_handles_fit(self):
        with mock.patch("core.paths.app_dir", return_value="Z:/missing"):
            plugins = PluginsPage(State({"ui_theme": "light"}))
        self.assertEqual(plugins.title.text(), "Pluginy")
        self.assertNotIn("🧩", plugins.title.text())
        css = stylesheet("light")
        self.assertIn("QSlider { min-height: 26px; }", css)
        self.assertIn("margin: -6px 0", css)
        plugins.close()

    def test_plugin_manager_uses_search_and_two_column_cards(self):
        state = State({"ui_theme": "light", "plugin_enabled.hud": False})
        with (mock.patch.object(app_module.os.path, "isdir", return_value=True),
              mock.patch.object(app_module.os, "listdir",
                                return_value=["acc", "hud", "tts"]),
              mock.patch.object(app_module.os.path, "exists", return_value=True)):
            plugins = PluginsPage(state)
            self.assertEqual(len(plugins.findChildren(QFrame, "PluginCard")), 3)
            self.assertEqual(plugins.search.placeholderText(), "Hľadať plugin…")
            labels = [label.text() for label in plugins.findChildren(QLabel)]
            self.assertTrue(any(text.startswith("Aktívne pluginy") for text in labels))
            self.assertTrue(any(text.startswith("Dostupné pluginy") for text in labels))
            plugins.search.setText("hud")
            self.app.processEvents()
            self.assertEqual(len(plugins.findChildren(QFrame, "PluginCard")), 1)
        plugins.close()


if __name__ == "__main__":
    unittest.main()
