import logging
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QCheckBox, QFrame
from PyQt6.QtCore import Qt


class SettingsMenu(QWidget):
    """
    Live settings panel for UltraPilot.

    Writes directly into the shared state (the same managed dict every process
    sees), so the values it sets are the ones the plugins actually read:
      * ``acc_target_speed``   — consumed by plugins/acc/main.py
      * ``acc_safe_distance``  — consumed by plugins/acc (time-gap)
      * ``acc_obey_limit``     — consumed by plugins/acc
    """

    def __init__(self, state):
        super().__init__()
        self.state = state
        self.init_ui()

    def init_ui(self):
        self.setStyleSheet("background-color: #121212; color: white; font-family: 'Segoe UI';")

        layout = QVBoxLayout()
        layout.setSpacing(20)
        layout.setContentsMargins(30, 30, 30, 30)

        title = QLabel("⚙️ Settings")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #00FF7F; margin-bottom: 10px;")
        layout.addWidget(title)

        # --- ACC Section ---
        acc_frame = QFrame()
        acc_frame.setStyleSheet("background-color: #1e1e1e; border-radius: 10px; padding: 10px;")
        acc_layout = QVBoxLayout(acc_frame)

        acc_title = QLabel("Adaptive Cruise Control")
        acc_title.setStyleSheet("font-size: 18px; font-weight: bold; color: #00ffcc;")
        acc_layout.addWidget(acc_title)

        # Target Speed Slider
        init_speed = int(self.state.get("acc_target_speed", 80) or 80)
        speed_layout = QHBoxLayout()
        self.speed_label = QLabel(f"Target Speed: {init_speed} km/h")
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(30, 140)
        self.speed_slider.setValue(init_speed)
        self.speed_slider.valueChanged.connect(self.update_acc_speed)
        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_slider)
        acc_layout.addLayout(speed_layout)

        # Follow-distance (time gap) Slider
        init_gap = int((self.state.get("acc_safe_distance", 2.0) or 2.0) * 10)
        dist_layout = QHBoxLayout()
        self.dist_label = QLabel(f"Safe Distance: {init_gap / 10.0:.1f}s")
        self.dist_slider = QSlider(Qt.Orientation.Horizontal)
        self.dist_slider.setRange(5, 40)  # 0.5 to 4.0s
        self.dist_slider.setValue(init_gap)
        self.dist_slider.valueChanged.connect(self.update_acc_dist)
        dist_layout.addWidget(self.dist_label)
        dist_layout.addWidget(self.dist_slider)
        acc_layout.addLayout(dist_layout)

        # Obey posted speed limit
        self.limit_toggle = QCheckBox("Obey posted speed limit")
        self.limit_toggle.setChecked(bool(self.state.get("acc_obey_limit", True)))
        self.limit_toggle.toggled.connect(self.update_obey_limit)
        acc_layout.addWidget(self.limit_toggle)

        layout.addWidget(acc_frame)
        layout.addStretch()
        self.setLayout(layout)

        # Publish initial values so plugins pick them up immediately.
        self.update_acc_speed(init_speed)
        self.update_acc_dist(init_gap)
        self.update_obey_limit(self.limit_toggle.isChecked())

    def update_acc_speed(self, val):
        self.speed_label.setText(f"Target Speed: {val} km/h")
        self.state.set("acc_target_speed", float(val))

    def update_acc_dist(self, val):
        dist = val / 10.0
        self.dist_label.setText(f"Safe Distance: {dist:.1f}s")
        self.state.set("acc_safe_distance", dist)

    def update_obey_limit(self, checked):
        self.state.set("acc_obey_limit", bool(checked))
