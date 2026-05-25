import sys
import os
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QWidget, QStackedWidget, QFrame, QSlider
from PyQt6.QtCore import QTimer, Qt
from core.engine import UltraPilotEngine

# Gaming Dark Theme Stylesheet
DARK_THEME = """
QMainWindow {
    background-color: #121212;
}
QWidget {
    background-color: #121212;
    color: #E0E0E0;
    font-family: 'Segoe UI', sans-serif;
}
QPushButton {
    background-color: #1E1E1E;
    border: 1px solid #333;
    border-radius: 5px;
    padding: 10px;
    color: #B0B0B0;
}
QPushButton:hover {
    background-color: #2A2A2A;
    border-color: #00FF7F;
    color: #FFFFFF;
}
QPushButton:pressed {
    background-color: #00FF7F;
    color: #000000;
}
QLabel {
    color: #E0E0E0;
}
QFrame#Sidebar {
    background-color: #1A1A1A;
    border-right: 1px solid #333;
}
QSlider::handle:horizontal {
    background: #00FF7F;
    border: 1px solid #000;
    width: 18px;
    margin: -2px 0;
    border-radius: 9px;
}
QSlider::groove:horizontal {
    border: 1px solid #333;
    height: 8px;
    background: #222;
    margin: 2px 0;
    border-radius: 4px;
}
"""

class AboutPage(Page):
    def __init__(self, engine):
        super().__init__(engine)
        self.label = QLabel("ℹ️ About UltraPilot")
        self.label.setStyleSheet("font-size: 24px; font-weight: bold; color: #00FF7F; margin-bottom: 20px;")
        self.layout.addWidget(self.label)

        text = QLabel(
            "ETS2 UltraPilot Pro Edition\n\n"
            "A professional-grade autopilot system for Euro Truck Simulator 2.\n"
            "Featuring Advanced Lane Assist, Adaptive Cruise Control, "
            "and Voice Navigation.\n\n"
            "Built for simulation enthusiasts."
        )
        text.setWordWrap(True)
        text.setStyleSheet("font-size: 16px; line-height: 150%;")
        self.layout.addWidget(text)
        self.layout.addStretch()

class PluginsPage(Page):
    def __init__(self, engine):
        super().__init__(engine)
        self.engine = engine
        self.label = QLabel("🧩 Plugin Management")
        self.label.setStyleSheet("font-size: 24px; font-weight: bold; color: #00FF7F; margin-bottom: 20px;")
        self.layout.addWidget(self.label)

        self.plugin_list = QVBoxLayout()
        self.layout.addWidget(self.plugin_list)
        self.layout.addStretch()
        self.refresh_plugins()

    def refresh_plugins(self):
        # Clear current list
        for i in reversed(range(self.plugin_list.count())):
            self.plugin_list.itemAt(i).widget().setParent(None)

        # In this architecture, we can't easily get the Plugin instances
        # because they are in separate processes.
        # We'll list the folders in the plugins directory.
        import os
        plugin_dir = "plugins"
        if os.path.exists(plugin_dir):
            for folder in os.listdir(plugin_dir):
                if os.path.isdir(os.path.join(plugin_dir, folder)):
                    self.add_plugin_row(folder)

    def add_plugin_row(self, name):
        row = QFrame()
        row.setStyleSheet("background-color: #1A1A1A; border-radius: 5px; margin-bottom: 5px;")
        l = QHBoxLayout(row)

        lbl = QLabel(name.capitalize())
        lbl.setFixedWidth(150)
        l.addWidget(lbl)

        btn = QPushButton("Toggle")
        btn.setFixedWidth(100)

        def toggle():
            # This is tricky because plugins are in separate processes.
            # We'll use the shared state to signal a toggle.
            current = self.engine.shared_state.get(f"plugin_{name}_enabled", True)
            self.engine.shared_state.set(f"plugin_{name}_enabled", not current)
            logging.info(f"Requested toggle for plugin: {name}")

        btn.clicked.connect(toggle)
        l.addWidget(btn)
        self.plugin_list.addWidget(row)

class DashboardPage(Page):
    def __init__(self, engine):
        super().__init__(engine)
        self.label = QLabel("🚀 UltraPilot Telemetry")
        self.label.setStyleSheet("font-size: 24px; font-weight: bold; color: #00FF7F; margin-bottom: 20px;")
        self.layout.addWidget(self.label)

        self.speed_container = QFrame()
        self.speed_container.setStyleSheet("background-color: #1A1A1A; border-radius: 10px; padding: 20px;")
        self.speed_layout = QVBoxLayout(self.speed_container)

        self.speed_title = QLabel("CURRENT SPEED")
        self.speed_title.setStyleSheet("color: #888; font-size: 12px; font-weight: bold;")
        self.speed_layout.addWidget(self.speed_title)

        self.speed_val = QLabel("0 km/h")
        self.speed_val.setStyleSheet("font-size: 48px; font-weight: bold; color: #FFFFFF;")
        self.speed_layout.addWidget(self.speed_val)

        self.layout.addWidget(self.speed_container)
        self.layout.addStretch()

class SettingsPage(Page):
    def __init__(self, engine):
        super().__init__(engine)
        self.label = QLabel("⚙️ System Configuration")
        self.label.setStyleSheet("font-size: 24px; font-weight: bold; color: #00FF7F; margin-bottom: 20px;")
        self.layout.addWidget(self.label)

        self.add_setting_slider("Cruise Speed (km/h)", "target_speed", 40, 120, 80)
        self.add_setting_slider("Steering Sensitivity (Kp)", "steering_kp", 0.1, 1.0, 0.3, step=0.01)
        self.add_setting_slider("Steering Stability (Kd)", "steering_kd", 0.0, 0.5, 0.1, step=0.01)
        self.add_setting_slider("Acceleration Power", "throttle_power", 0.1, 1.0, 0.5, step=0.01)

        self.layout.addStretch()

    def add_setting_slider(self, name, key, min_val, max_val, default_val, step=1):
        container = QFrame()
        container.setStyleSheet("background-color: #1A1A1A; border-radius: 5px; margin-bottom: 10px;")
        l = QHBoxLayout(container)

        lbl = QLabel(name)
        lbl.setFixedWidth(200)
        l.addWidget(lbl)

        slider = QSlider(Qt.Orientation.Horizontal)
        multiplier = 100 if step < 1 else 1
        slider.setRange(int(min_val * multiplier), int(max_val * multiplier))
        slider.setValue(int(default_val * multiplier))

        val_lbl = QLabel(f"{default_val}")
        val_lbl.setFixedWidth(50)

        def update_val(val):
            real_val = val / multiplier
            val_lbl.setText(f"{real_val:.2f}")
            self.engine.shared_state.set(key, real_val)

        slider.valueChanged.connect(update_val)
        l.addWidget(slider)
        l.addWidget(val_lbl)
        self.layout.addWidget(container)

class UltraPilotApp(QMainWindow):
    def __init__(self, engine):
        super().__init__()
        self.engine = engine
        self.setWindowTitle("ETS2 UltraPilot Pro Edition")
        self.setFixedSize(800, 500)
        self.setStyleSheet(DARK_THEME)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        self.sidebar = QFrame()
        self.sidebar.setObjectName("Sidebar")
        self.sidebar.setFixedWidth(200)
        sidebar_layout = QVBoxLayout(self.sidebar)

        self.btn_dash = QPushButton("Dashboard")
        self.btn_settings = QPushButton("Settings")
        self.btn_plugins = QPushButton("Plugins")
        self.btn_about = QPushButton("About")

        self.btn_dash.clicked.connect(lambda: self.pages.setCurrentIndex(0))
        self.btn_settings.clicked.connect(lambda: self.pages.setCurrentIndex(1))
        self.btn_plugins.clicked.connect(lambda: self.pages.setCurrentIndex(2))
        self.btn_about.clicked.connect(lambda: self.pages.setCurrentIndex(3))

        sidebar_layout.addWidget(self.btn_dash)
        sidebar_layout.addWidget(self.btn_settings)
        sidebar_layout.addWidget(self.btn_plugins)
        sidebar_layout.addWidget(self.btn_about)
        sidebar_layout.addStretch()
        main_layout.addWidget(self.sidebar)

        self.pages = QStackedWidget()
        self.pages.addWidget(DashboardPage(engine))
        self.pages.addWidget(SettingsPage(engine))
        self.pages.addWidget(PluginsPage(engine))
        self.pages.addWidget(AboutPage(engine))
        main_layout.addWidget(self.pages)

        self.start_btn = QPushButton("START ENGINE")
        self.start_btn.setStyleSheet("background-color: #00FF7F; color: #000; font-weight: bold;")
        self.start_btn.clicked.connect(self.toggle_engine)
        self.statusBar().addWidget(self.start_btn)
        self.statusBar().setStyleSheet("background-color: #1A1A1A; color: #888;")

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

    def toggle_engine(self):
        if self.engine.running:
            self.engine.stop()
            self.start_btn.setText("START ENGINE")
            self.start_btn.setStyleSheet("background-color: #00FF7F; color: #000; font-weight: bold;")
        else:
            import threading
            threading.Thread(target=self.engine.start, daemon=True).start()
            self.start_btn.setText("STOP ENGINE")
            self.start_btn.setStyleSheet("background-color: #FF4444; color: #FFF; font-weight: bold;")

    def update_ui(self):
        dash = self.pages.widget(0)
        if isinstance(dash, DashboardPage):
            speed = self.engine.telemetry.get("truck", {}).get("speed", 0)
            dash.speed_val.setText(f"{speed * 3.6:.1f} km/h")

if __name__ == "__main__":
    engine = UltraPilotEngine()
    engine.plugin_manager.discover_and_load()

    app = QApplication(sys.argv)
    window = UltraPilotApp(engine)
    window.show()
    sys.exit(app.exec())
