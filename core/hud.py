from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt, QTimer, QPoint
from PyQt6.QtGui import QColor, QPalette, QFont
import sys

class UltraPilotHUD(QWidget):
    """
    Transparent, always-on-top HUD overlay for ETS2.
    Displays the current system state and key telemetry.
    """
    def __init__(self, shared_state):
        super().__init__()
        self.shared_state = shared_state
        self.init_ui()

    def init_ui(self):
        # Window setup: Transparent, Always on Top, No Frame
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint |
            Qt.WindowType.WindowStaysOnTopHint |
            Qt.WindowType.Tool
        )
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setAttribute(Qt.WidgetAttribute.WA_ShowRequests StatefulWidget)

        # Layout and Style
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.layout.setSpacing(5)

        self.state_label = QLabel("INITIALIZING...")
        self.state_label.setStyleSheet("""
            color: #00FFCC;
            font-weight: bold;
            font-size: 18px;
            font-family: 'Segoe UI', sans-serif;
            background-color: rgba(0, 0, 0, 150);
            border-radius: 5px;
            padding: 5px;
        """)
        self.layout.addWidget(self.state_label)

        self.telemetry_label = QLabel("S: 0 km/h | L: 0.00")
        self.telemetry_label.setStyleSheet("""
            color: #FFFFFF;
            font-size: 14px;
            font-family: 'Consolas', monospace;
            background-color: rgba(0, 0, 0, 120);
            border-radius: 5px;
            padding: 5px;
        """)
        self.layout.addWidget(self.telemetry_label)

        self.setLayout(self.layout)

        # Position the HUD in the top-right corner
        self.setGeometry(100, 100, 250, 100)
        self.move_to_corner()

        # Update Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_hud)
        self.timer.start(100) # 10 FPS is enough for a HUD

    def move_to_corner(self):
        screen = QApplication.primaryScreen().geometry()
        self.move(screen.width() - 260, 50)

    def update_hud(self):
        # Fetch data from Shared State
        state = self.shared_state.get("system_state", "IDLE")

        # Format state name (removes 'SystemState.' prefix if present)
        state_text = state.name if hasattr(state, 'name') else str(state)
        self.state_label.setText(f"SYSTEM: {state_text}")

        # Telemetry
        speed = self.shared_state.get("speed", 0)
        lane = self.shared_state.get("lane_offset", 0.0)
        self.telemetry_label.setText(f"S: {speed:.1f} km/h | L: {lane:.2f}")

        # Dynamic color based on state
        if state_text == "EMERGENCY":
            self.state_label.setStyleSheet(self.state_label.styleSheet().replace("#00FFCC", "#FF0000"))
        elif state_text == "PAY_TOLL":
            self.state_label.setStyleSheet(self.state_label.styleSheet().replace("#00FFCC", "#FFFF00"))
        else:
            self.state_label.setStyleSheet(self.state_label.styleSheet().replace("#FF0000", "#00FFCC").replace("#FFFF00", "#00FFCC"))

    def mousePressEvent(self, event):
        # Allow moving the HUD manually if needed
        if event.button() == Qt.MouseButton.LeftButton:
            self.old_pos = event.globalPosition().toPoint()

    def mouseMoveEvent(self, event):
        if hasattr(self, 'old_pos'):
            delta = event.globalPosition().toPoint() - self.old_pos
            self.move(self.x() + delta.x(), self.y() + delta.y())
            self.old_pos = event.globalPosition().toPoint()

def run_hud(shared_state):
    app = QApplication(sys.argv)
    hud = UltraPilotHUD(shared_state)
    hud.show()
    sys.exit(app.exec())
