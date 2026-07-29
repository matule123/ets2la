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
                    window_control_notch_path, DashboardPage, AboutPage)
from UI.icons import line_icon
from UI.map_page import MapPage
from UI.perf_overlay import PerfOverlay
from UI.settings_menu import SettingsMenu


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
        self.assertTrue(all(button.width() == 13 and button.height() == 13
                            for button in controls))
        self.assertEqual([button.accessibleName() for button in controls],
                         ["Maximalizovať", "Minimalizovať", "Zavrieť"])

    def test_window_controls_are_seated_in_a_curved_notch(self):
        path = window_control_notch_path(82, 34)
        bounds = path.boundingRect()
        self.assertEqual((bounds.width(), bounds.height()), (82.0, 34.0))
        # The lower-left cutout is outside while all three control centres are
        # inside the painted surface.
        self.assertFalse(path.contains(bounds.bottomLeft()))
        for x in (18.5, 37.5, 56.5):
            self.assertTrue(path.contains(type(bounds.center())(x, 13.5)))

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
        self.assertEqual(window.centralWidget().objectName(), "WindowSurface")
        region = rounded_window_region(1220, 760)
        self.assertFalse(region.contains(QPoint(0, 0)))
        self.assertTrue(region.contains(QPoint(610, 380)))
        window.close()

    def test_settings_use_responsive_cards_and_vector_header_icon(self):
        settings = SettingsMenu(State({"ui_theme": "light"}))
        cards = settings.findChildren(QFrame, "SettingsCard")
        self.assertEqual(len(cards), 5)
        labels = [label.text() for label in settings.findChildren(QLabel)]
        self.assertIn("Nastavenia", labels)
        self.assertFalse(any(text.startswith("⚙") for text in labels))
        self.assertIn("QWidget#SettingsPage", settings.styleSheet())
        settings.close()

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

    def test_navigation_is_game_gps_first_and_live_map_badge_is_visible(self):
        with mock.patch.object(MapPage, "_populate_maps", autospec=True):
            page = MapPage(State({"ui_theme": "light"}))
        labels = [label.text() for label in page.findChildren(QLabel)]
        buttons = [button.text() for button in page.findChildren(QPushButton)]
        self.assertIn("●  LIVE MAP", labels)
        self.assertFalse(any("Record" in text or "Recorded" in text
                             for text in labels + buttons))
        self.assertFalse(hasattr(page, "route_combo"))
        self.assertFalse(hasattr(page, "name_edit"))
        page.close()

    def test_dashboard_and_about_use_clean_non_emoji_headers(self):
        state = State({"ui_theme": "light"})
        dashboard = DashboardPage(state)
        about = AboutPage(state)
        self.assertEqual(dashboard.title.text(), "Prehľad")
        self.assertNotIn("🚀", dashboard.title.text())
        self.assertEqual(about.title.text(), "O aplikácii")
        self.assertEqual(len(about.findChildren(QFrame, "AboutFeature")), 3)
        dashboard.close()
        about.close()


if __name__ == "__main__":
    unittest.main()
