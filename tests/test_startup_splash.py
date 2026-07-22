import unittest

from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication, QLabel, QWidget

from UI.splash import BootSplash


class StartupSplashTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_black_grey_splash_has_no_first_load_hint(self):
        splash = BootSplash()
        card = splash.findChild(QWidget, "Card")
        self.assertIsNotNone(card)
        self.assertIn("background: #000000", card.styleSheet())
        labels = splash.findChildren(QLabel)
        texts = [label.text() for label in labels]
        self.assertIn("Initializing...", texts)
        self.assertFalse(any("Prvé načítanie" in text for text in texts))
        self.assertEqual(splash.spinner._arc_color, "#A3A3A3")
        self.assertEqual(splash.spinner._track_color, "#303030")
        self.assertIn("#A3A3A3", splash.status_lbl.styleSheet())
        splash.close()

    def test_spinner_angle_advances_while_its_event_loop_runs(self):
        splash = BootSplash()
        splash.show()
        self.app.processEvents()
        start = splash.spinner._angle
        QTest.qWait(80)
        self.app.processEvents()
        self.assertTrue(splash.spinner._timer.isActive())
        self.assertNotEqual(splash.spinner._angle, start)
        splash.close()


if __name__ == "__main__":
    unittest.main()
