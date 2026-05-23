from sdk.base_plugin import BasePlugin
from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QFont
import logging

class HUDWindow(QWidget):
    def __init__(self, sdk):
        super().__init__()
        self.sdk = sdk
        self.init_ui()

    def init_ui(self):
        # Transparent and Always on Top
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint |
            Qt.WindowType.WindowStaysOnTopHint |
            Qt.WindowType.WindowTransparentForInput
        )
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForInput)

        self.layout = QVBoxLayout()
        self.speed_label = QLabel("Speed: 0 km/h")
        self.speed_label.setStyleSheet("color: lime; font-size: 24px; font-weight: bold; font-family: 'Segoe UI';")
        self.layout.addWidget(self.speed_label)

        self.status_label = QLabel("Status: Idle")
        self.status_label.setStyleSheet("color: yellow; font-size: 18px; font-family: 'Segoe UI';")
        self.layout.addWidget(self.status_label)

        self.setLayout(self.layout)
        self.setGeometry(100, 100, 300, 200)

    def update_data(self):
        speed = self.sdk.telemetry.get("truck", {}).get("speed", 0)
        self.speed_label.setText(f"Speed: {speed * 3.6:.1f} km/h")

        # Get safety/obstacle info
        danger = self.sdk.perception.detect_obstacles()
        if danger > 0.3:
            self.status_label.setText(f"STATUS: WARNING - OBSTACLE! {danger:.2f}")
            self.status_label.setStyleSheet("color: red; font-size: 18px; font-weight: bold; font-family: 'Segoe UI';")
        else:
            active_plugins = [p.description.name for p in self.sdk.plugin_manager.plugins if p.enabled]
            self.status_label.setText(f"Active: {', '.join(active_plugins) if active_plugins else 'None'}")
            self.status_label.setStyleSheet("color: yellow; font-size: 18px; font-family: 'Segoe UI';")

class Plugin(BasePlugin):
    """HUD Overlay plugin for real-time info."""

    def on_start(self):
        logging.info("HUD Plugin started.")
        self.enabled = True
        # We need a separate QApplication for the HUD if not already running,
        # but since the main app is a QApplication, we can just create a window.
        # However, the HUD needs to be on the main thread or handled carefully.
        self.window = HUDWindow(self.sdk)
        self.window.show()

        self.timer = QTimer()
        self.timer.timeout.connect(self.window.update_data)
        self.timer.start(100)

    def on_stop(self):
        logging.info("HUD Plugin stopped.")
        self.enabled = False
        if hasattr(self, 'window'):
            self.window.close()
        if hasattr(self, 'timer'):
            self.timer.stop()

    def on_tick(self, delta_time: float):
        pass
