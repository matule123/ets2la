import sys
import os
import logging
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QWidget, QStackedWidget, QFrame,
)
from PyQt6.QtCore import QTimer, Qt

from ui.settings_menu import SettingsMenu
from ui.map_page import MapPage

# Clean light theme (ETS2LA-style: white surfaces, green accent).
ACCENT = "#10B981"
LIGHT_THEME = """
QMainWindow { background-color: #F4F6F8; }
QWidget { background-color: #F4F6F8; color: #1A1D21; font-family: 'Segoe UI', sans-serif; }
QPushButton { background-color: #FFFFFF; border: 1px solid #DfE3E8; border-radius: 8px; padding: 10px; color: #374151; }
QPushButton:hover { border-color: #10B981; color: #065F46; }
QPushButton:pressed { background-color: #10B981; color: #FFFFFF; }
QLabel { color: #1A1D21; }
QFrame#Sidebar { background-color: #FFFFFF; border-right: 1px solid #E5E7EB; }
QComboBox, QLineEdit { background-color: #FFFFFF; border: 1px solid #DfE3E8; border-radius: 8px; padding: 7px; }
QCheckBox { spacing: 8px; }
QSlider::groove:horizontal { height: 6px; background: #E5E7EB; border-radius: 3px; }
QSlider::handle:horizontal { background: #10B981; width: 16px; margin: -6px 0; border-radius: 8px; }
QStatusBar { background-color: #FFFFFF; border-top: 1px solid #E5E7EB; }
"""
DARK_THEME = LIGHT_THEME  # kept for backwards-compatible references


class Page(QWidget):
    def __init__(self, state):
        super().__init__()
        self.state = state
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(30, 30, 30, 30)
        self.layout.setSpacing(15)


class AboutPage(Page):
    def __init__(self, state):
        super().__init__(state)
        title = QLabel("ℹ️ About UltraPilot")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #065F46; margin-bottom: 20px;")
        self.layout.addWidget(title)
        text = QLabel(
            "ETS2 UltraPilot Pro Edition\n\n"
            "A professional-grade autopilot for Euro Truck Simulator 2.\n"
            "Lane Assist, Adaptive Cruise Control, Collision Avoidance, "
            "Navigation, HUD and Voice — each plugin isolated in its own process."
        )
        text.setWordWrap(True)
        text.setStyleSheet("font-size: 16px;")
        self.layout.addWidget(text)
        self.layout.addStretch()


class PluginsPage(Page):
    def __init__(self, state):
        super().__init__(state)
        title = QLabel("🧩 Plugin Management")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #065F46; margin-bottom: 20px;")
        self.layout.addWidget(title)

        self.plugin_list = QVBoxLayout()
        self.layout.addLayout(self.plugin_list)
        self.layout.addStretch()
        self.refresh_plugins()

    def refresh_plugins(self):
        for i in reversed(range(self.plugin_list.count())):
            w = self.plugin_list.itemAt(i).widget()
            if w:
                w.setParent(None)

        from core.paths import app_dir
        plugin_dir = os.path.join(app_dir(), "plugins")
        if not os.path.isdir(plugin_dir):
            return
        for folder in sorted(os.listdir(plugin_dir)):
            full = os.path.join(plugin_dir, folder)
            if os.path.isdir(full) and os.path.exists(os.path.join(full, "main.py")):
                self.add_plugin_row(folder)

    def add_plugin_row(self, name):
        row = QFrame()
        row.setStyleSheet("background-color: #FFFFFF; border: 1px solid #E5E7EB; border-radius: 8px; padding: 4px;")
        l = QHBoxLayout(row)

        lbl = QLabel(name.capitalize())
        lbl.setFixedWidth(150)
        l.addWidget(lbl)

        status = QLabel()
        l.addWidget(status)
        l.addStretch()

        btn = QPushButton("Toggle")
        btn.setFixedWidth(100)

        def render():
            enabled = self.state.get(f"plugin_enabled.{name}", True)
            status.setText("● ON" if enabled else "○ OFF")
            status.setStyleSheet(f"color: {'#34C759' if enabled else '#FF453A'}; font-weight: bold;")

        def toggle():
            current = self.state.get(f"plugin_enabled.{name}", True)
            self.state.set(f"plugin_enabled.{name}", not current)
            logging.info(f"Toggled plugin '{name}' -> {not current}")
            render()

        btn.clicked.connect(toggle)
        l.addWidget(btn)
        self.plugin_list.addWidget(row)
        render()


class DashboardPage(Page):
    def __init__(self, state):
        super().__init__(state)
        title = QLabel("🚀 UltraPilot Telemetry")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #065F46; margin-bottom: 10px;")
        self.layout.addWidget(title)

        container = QFrame()
        container.setStyleSheet("background-color: #FFFFFF; border: 1px solid #E5E7EB; border-radius: 12px; padding: 20px;")
        cl = QVBoxLayout(container)

        st = QLabel("CURRENT SPEED")
        st.setStyleSheet("color: #6B7280; font-size: 12px; font-weight: bold;")
        cl.addWidget(st)
        self.speed_val = QLabel("0 km/h")
        self.speed_val.setStyleSheet("font-size: 48px; font-weight: bold; color: #111827;")
        cl.addWidget(self.speed_val)

        self.state_val = QLabel("SYSTEM: IDLE")
        self.state_val.setStyleSheet("color: #10B981; font-size: 16px; font-weight: bold;")
        cl.addWidget(self.state_val)

        self.layout.addWidget(container)

        # Live telemetry grid (gear / rpm / fuel / limit / position / nav).
        self.metrics = {}
        grid_frame = QFrame()
        grid_frame.setStyleSheet("background-color: #FFFFFF; border: 1px solid #E5E7EB; border-radius: 12px; padding: 15px;")
        grid = QHBoxLayout(grid_frame)
        for key, label in [("gear", "GEAR"), ("rpm", "RPM"), ("fuel", "FUEL"),
                           ("limit", "LIMIT"), ("nav", "NAV")]:
            col = QVBoxLayout()
            cap = QLabel(label)
            cap.setStyleSheet("color: #6B7280; font-size: 11px; font-weight: bold;")
            val = QLabel("—")
            val.setStyleSheet("color: #111827; font-size: 20px; font-weight: bold;")
            col.addWidget(cap)
            col.addWidget(val)
            grid.addLayout(col)
            self.metrics[key] = val
        self.layout.addWidget(grid_frame)

        self.conn_val = QLabel("● Waiting for game telemetry…")
        self.conn_val.setStyleSheet("color: #9CA3AF; font-size: 12px; margin-top: 6px;")
        self.layout.addWidget(self.conn_val)

        self.layout.addStretch()

    def refresh(self):
        speed = self.state.get("speed", 0) or 0
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            speed = 0.0
        speed_kmh = speed * 3.6 if abs(speed) < 200 else speed
        self.speed_val.setText(f"{abs(speed_kmh):.1f} km/h")
        sysstate = self.state.get("system_state", "IDLE")
        self.state_val.setText(f"SYSTEM: {sysstate}")

        truck = (self.state.get("telemetry", {}) or {}).get("truck", {}) or {}
        gear = truck.get("gear", 0)
        gear_txt = str(int(gear)) if gear and gear > 0 else ("R" if gear and gear < 0 else "N")
        self.metrics["gear"].setText(gear_txt)
        self.metrics["rpm"].setText(f"{truck.get('engineRpm', 0) or 0:.0f}")
        self.metrics["fuel"].setText(f"{truck.get('fuel', 0) or 0:.0f}L")
        limit_ms = truck.get("speedLimit", 0) or 0
        self.metrics["limit"].setText(f"{limit_ms * 3.6:.0f}" if limit_ms > 1 else "—")
        if self.state.get("nav_active"):
            dist = self.state.get("distance_to_dest")
            self.metrics["nav"].setText(f"{float(dist) / 1000:.1f}km" if dist else "ON")
        else:
            self.metrics["nav"].setText("off")

        # Connection indicator: sdkActive in the latest telemetry snapshot.
        raw = (self.state.get("telemetry", {}) or {}).get("raw", {}) or {}
        if raw.get("sdkActive"):
            self.conn_val.setText("● Telemetry connected")
            self.conn_val.setStyleSheet("color: #34C759; font-size: 12px; margin-top: 6px;")
        else:
            self.conn_val.setText("● Waiting for game telemetry…")
            self.conn_val.setStyleSheet("color: #8E8E93; font-size: 12px; margin-top: 6px;")


class UltraPilotApp(QMainWindow):
    """Control panel. Runs in its own process and talks to the engine purely
    through shared state — START/STOP flips the ``autopilot_active`` master
    switch rather than starting/stopping the engine object directly."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        self.setWindowTitle("ETS2 UltraPilot Pro Edition")
        self.setFixedSize(800, 500)
        from core.theme import stylesheet
        self._theme = (state.get("ui_theme", "light") or "light")
        self.setStyleSheet(stylesheet(self._theme))
        # Window/taskbar icon.
        from PyQt6.QtGui import QIcon
        from core.paths import resource
        _ico = resource("assets", "favicon.ico")
        if os.path.exists(_ico):
            self.setWindowIcon(QIcon(_ico))

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        self.sidebar = QFrame()
        self.sidebar.setObjectName("Sidebar")
        self.sidebar.setFixedWidth(200)
        sb = QVBoxLayout(self.sidebar)
        # Logo at the top of the sidebar.
        from PyQt6.QtGui import QPixmap
        from core.paths import resource as _res
        logo = QLabel()
        _pm = QPixmap(_res("assets", "logo.png"))
        if not _pm.isNull():
            logo.setPixmap(_pm.scaledToWidth(150, 1))  # 1 = SmoothTransformation
        logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sb.addWidget(logo)
        sb.addSpacing(10)
        btn_dash = QPushButton("Dashboard")
        btn_map = QPushButton("Navigation")
        btn_plugins = QPushButton("Plugins")
        btn_settings = QPushButton("Settings")
        btn_about = QPushButton("About")
        btn_dash.clicked.connect(lambda: self.pages.setCurrentIndex(0))
        btn_map.clicked.connect(lambda: self.pages.setCurrentIndex(1))
        btn_plugins.clicked.connect(lambda: self.pages.setCurrentIndex(2))
        btn_settings.clicked.connect(lambda: self.pages.setCurrentIndex(3))
        btn_about.clicked.connect(lambda: self.pages.setCurrentIndex(4))
        for b in (btn_dash, btn_map, btn_plugins, btn_settings, btn_about):
            sb.addWidget(b)
        sb.addStretch()
        main_layout.addWidget(self.sidebar)

        self.pages = QStackedWidget()
        self.pages.addWidget(DashboardPage(state))
        self.pages.addWidget(MapPage(state))
        self.pages.addWidget(PluginsPage(state))
        self.pages.addWidget(SettingsMenu(state))
        self.pages.addWidget(AboutPage(state))
        main_layout.addWidget(self.pages)

        self.start_btn = QPushButton("ENABLE AUTOPILOT")
        self.start_btn.clicked.connect(self.toggle_autopilot)
        self.statusBar().addWidget(self.start_btn)
        self.statusBar().setStyleSheet("background-color: #FFFFFF; color: #6B7280;")
        self._render_start_btn()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

    def _render_start_btn(self):
        active = self.state.get("autopilot_active", False)
        if active:
            self.start_btn.setText("DISABLE AUTOPILOT")
            self.start_btn.setStyleSheet("background-color: #EF4444; color: #FFFFFF; font-weight: bold; padding: 8px 18px; border-radius: 8px;")
        else:
            self.start_btn.setText("ENABLE AUTOPILOT")
            self.start_btn.setStyleSheet("background-color: #10B981; color: #FFFFFF; font-weight: bold; padding: 8px 18px; border-radius: 8px;")

    def toggle_autopilot(self):
        current = self.state.get("autopilot_active", False)
        self.state.set("autopilot_active", not current)
        logging.info(f"Autopilot master switch -> {not current}")
        self._render_start_btn()

    def update_ui(self):
        dash = self.pages.widget(0)
        if isinstance(dash, DashboardPage):
            dash.refresh()
        self._render_start_btn()


if __name__ == "__main__":
    # Standalone preview (no engine) — uses a plain dict as shared state.
    from core.ipc.shared_state import SharedState
    app = QApplication(sys.argv)
    window = UltraPilotApp(SharedState())
    window.show()
    sys.exit(app.exec())
