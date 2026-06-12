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
        self.setStyleSheet("background-color: #F4F6F8; color: #1A1D21; font-family: 'Segoe UI';")

        layout = QVBoxLayout()
        layout.setSpacing(20)
        layout.setContentsMargins(30, 30, 30, 30)

        title = QLabel("⚙️ Settings")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #065F46; margin-bottom: 10px;")
        layout.addWidget(title)

        # --- ACC Section ---
        acc_frame = QFrame()
        acc_frame.setStyleSheet("background-color: #FFFFFF; border: 1px solid #E5E7EB; border-radius: 12px; padding: 12px;")
        acc_layout = QVBoxLayout(acc_frame)

        acc_title = QLabel("Adaptive Cruise Control")
        acc_title.setStyleSheet("font-size: 18px; font-weight: bold; color: #0F766E;")
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

        # --- Steering Section ---
        steer_frame = QFrame()
        steer_frame.setStyleSheet("background-color: #FFFFFF; border: 1px solid #E5E7EB; border-radius: 12px; padding: 12px;")
        steer_layout = QVBoxLayout(steer_frame)
        steer_title = QLabel("Steering")
        steer_title.setStyleSheet("font-size: 18px; font-weight: bold; color: #0F766E;")
        steer_layout.addWidget(steer_title)

        self.invert_toggle = QCheckBox("Invert steering (flip if the truck turns the wrong way)")
        self.invert_toggle.setChecked(bool(self.state.get("steering_invert", False)))
        self.invert_toggle.toggled.connect(self.update_invert)
        steer_layout.addWidget(self.invert_toggle)

        sens_layout = QHBoxLayout()
        init_sens = int((self.state.get("steering_sensitivity", 1.0) or 1.0) * 100)
        self.sens_label = QLabel(f"Sensitivity: {init_sens / 100:.2f}×")
        self.sens_slider = QSlider(Qt.Orientation.Horizontal)
        self.sens_slider.setRange(30, 200)  # 0.3× .. 2.0×
        self.sens_slider.setValue(init_sens)
        self.sens_slider.valueChanged.connect(self.update_sensitivity)
        sens_layout.addWidget(self.sens_label)
        sens_layout.addWidget(self.sens_slider)
        steer_layout.addLayout(sens_layout)

        layout.addWidget(steer_frame)

        # --- Appearance Section (theme + language) ---
        from PyQt6.QtWidgets import QComboBox, QPushButton
        from core.i18n import LANGUAGES, coverage
        app_frame = QFrame()
        app_frame.setStyleSheet("background-color: #FFFFFF; border: 1px solid #E5E7EB; border-radius: 12px; padding: 12px;")
        app_layout = QVBoxLayout(app_frame)
        app_title = QLabel("Appearance")
        app_title.setStyleSheet("font-size: 18px; font-weight: bold; color: #0F766E;")
        app_layout.addWidget(app_title)

        theme_row = QHBoxLayout()
        theme_row.addWidget(QLabel("Theme:"))
        self.theme_combo = QComboBox()
        self.theme_combo.addItems(["Light", "Dark", "System"])
        cur = (self.state.get("ui_theme", "light") or "light").capitalize()
        self.theme_combo.setCurrentText(cur)
        self.theme_combo.currentTextChanged.connect(self.update_theme)
        theme_row.addWidget(self.theme_combo)
        theme_row.addStretch()
        app_layout.addLayout(theme_row)

        lang_row = QHBoxLayout()
        lang_row.addWidget(QLabel("Language:"))
        self.lang_combo = QComboBox()
        self.lang_combo.addItems(LANGUAGES.keys())
        self.lang_combo.setCurrentText(self.state.get("ui_language", "Slovenčina") or "Slovenčina")
        self.lang_combo.currentTextChanged.connect(self.update_language)
        lang_row.addWidget(self.lang_combo)
        lang_row.addStretch()
        app_layout.addLayout(lang_row)

        self.cov_label = QLabel("")
        self.cov_label.setStyleSheet("color: #6B7280; font-size: 12px;")
        app_layout.addWidget(self.cov_label)

        ets_btn = QPushButton("🌐 Open ETS2LA web app")
        ets_btn.clicked.connect(self._open_ets2la)
        app_layout.addWidget(ets_btn)

        layout.addWidget(app_frame)
        layout.addStretch()
        self.setLayout(layout)

        # Publish initial values so plugins pick them up immediately.
        self.update_acc_speed(init_speed)
        self.update_acc_dist(init_gap)
        self.update_obey_limit(self.limit_toggle.isChecked())
        self.update_invert(self.invert_toggle.isChecked())
        self.update_sensitivity(init_sens)
        self.update_language(self.lang_combo.currentText())

    def update_theme(self, name):
        self.state.set("ui_theme", name.lower())

    def update_language(self, lang):
        from core.i18n import coverage
        self.state.set("ui_language", lang)
        self.cov_label.setText(f"{lang} — {coverage(lang)}% translated")

    def _open_ets2la(self):
        import webbrowser
        webbrowser.open("https://app.ets2la.com/onboarding")

    def update_acc_speed(self, val):
        self.speed_label.setText(f"Target Speed: {val} km/h")
        self.state.set("acc_target_speed", float(val))

    def update_acc_dist(self, val):
        dist = val / 10.0
        self.dist_label.setText(f"Safe Distance: {dist:.1f}s")
        self.state.set("acc_safe_distance", dist)

    def update_obey_limit(self, checked):
        self.state.set("acc_obey_limit", bool(checked))

    def update_invert(self, checked):
        self.state.set("steering_invert", bool(checked))

    def update_sensitivity(self, val):
        s = val / 100.0
        self.sens_label.setText(f"Sensitivity: {s:.2f}×")
        self.state.set("steering_sensitivity", s)
