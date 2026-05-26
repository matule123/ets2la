import logging
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QCheckBox, QComboBox, QFrame
from PyQt6.QtCore import Qt

class SettingsMenu(QWidget):
    """
    Graphical Settings Menu for UltraPilot.
    Allows users to adjust plugin parameters in real-time.
    """
    def __init__(self, sdk):
        super().__init__()
        self.sdk = sdk
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("UltraPilot Pro - Settings")
        self.setFixedSize(400, 500)
        self.setStyleSheet("background-color: #121212; color: white; font-family: 'Segoe UI';")

        layout = QVBoxLayout()
        layout.setSpacing(20)
        layout.setContentsMargins(20, 20, 20, 20)

        # --- ACC Section ---
        acc_frame = QFrame()
        acc_frame.setFrameShape(QFrame.Shape.StyledPanel)
        acc_frame.setStyleSheet("background-color: #1e1e1e; border-radius: 10px; padding: 10px;")
        acc_layout = QVBoxLayout(acc_frame)

        acc_title = QLabel("Adaptive Cruise Control")
        acc_title.setStyleSheet("font-size: 18px; font-weight: bold; color: #00ffcc;")
        acc_layout.addWidget(acc_title)

        # Target Speed Slider
        speed_layout = QHBoxLayout()
        self.speed_label = QLabel("Target Speed: 80 km/h")
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(30, 140)
        self.speed_slider.setValue(80)
        self.speed_slider.valueChanged.connect(self.update_acc_speed)
        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_slider)
        acc_layout.addLayout(speed_layout)

        # Distance Slider
        dist_layout = QHBoxLayout()
        self.dist_label = QLabel("Safe Distance: 2.0s")
        self.dist_slider = QSlider(Qt.Orientation.Horizontal)
        self.dist_slider.setRange(5, 40) # 0.5 to 4.0s
        self.dist_slider.setValue(20)
        self.dist_slider.valueChanged.connect(self.update_acc_dist)
        dist_layout.addWidget(self.dist_label)
        dist_layout.addWidget(self.dist_slider)
        acc_layout.addLayout(dist_layout)

        layout.addWidget(acc_frame)

        # --- LKA Section ---
        lka_frame = QFrame()
        lka_frame.setFrameShape(QFrame.Shape.StyledPanel)
        lka_frame.setStyleSheet("background-color: #1e1e1e; border-radius: 10px; padding: 10px;")
        lka_layout = QVBoxLayout(lka_frame)

        lka_title = QLabel("Lane Keep Assist")
        lka_title.setStyleSheet("font-size: 18px; font-weight: bold; color: #00ffcc;")
        lka_layout.addWidget(lka_title)

        self.lka_toggle = QCheckBox("Enable LKA")
        self.lka_toggle.setChecked(True)
        self.lka_toggle.toggled.connect(self.update_lka_state)
        lka_layout.addWidget(self.lka_toggle)

        layout.addWidget(lka_frame)

        layout.addStretch()
        self.setLayout(layout)

    def update_acc_speed(self, val):
        self.speed_label.setText(f"Target Speed: {val} km/h")
        self.sdk.shared_state.set("acc_target_speed", float(val))

    def update_acc_dist(self, val):
        dist = val / 10.0
        self.dist_label.setText(f"Safe Distance: {dist:.1f}s")
        self.sdk.shared_state.set("acc_safe_distance", dist)

    def update_lka_state(self, state):
        self.sdk.shared_state.set("lka_enabled", state)
