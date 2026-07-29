import importlib
import sys
import unittest

from PyQt6.QtWidgets import QApplication, QWidget

from core.theme import palette, stylesheet

# The installed build exposes the historical lowercase ``ui`` package name;
# the source checkout directory is ``UI`` and may be case-sensitive in CI.
sys.modules.setdefault("ui", importlib.import_module("UI"))
from UI.app import MacTitleBar, window_control_notch_path
from UI.icons import line_icon


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
                     "settings", "about"):
            self.assertFalse(line_icon(name).isNull())


if __name__ == "__main__":
    unittest.main()
