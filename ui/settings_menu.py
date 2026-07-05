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
        from core import i18n
        # Show every available language (bundled + downloaded) with its coverage
        # percentage. The combo stores the language code as item data. Languages
        # that aren't downloaded yet are shown greyed — there's a separate
        # „Download language“ button below to fetch them from GitHub.
        self._lang_codes = []
        for info in i18n.available():
            self._lang_codes.append(info["code"])
            label = f"{info['name']}  ·  {info['coverage']}%" if info["downloaded"] else f"{info['name']}  ·  (stiahnuteľné)"
            self.lang_combo.addItem(label, info["code"])
        cur_code = self.state.get("ui_language_code") or "sk"
        for i in range(self.lang_combo.count()):
            if self.lang_combo.itemData(i) == cur_code:
                self.lang_combo.setCurrentIndex(i)
                break
        self.lang_combo.currentIndexChanged.connect(self.update_language)
        lang_row.addWidget(self.lang_combo)

        self.dl_lang_btn = QPushButton("Stiahnuť jazyk")
        self.dl_lang_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.dl_lang_btn.setStyleSheet(
            "QPushButton{background:#F3F4F6;color:#111827;border:1px solid #E5E7EB;"
            "border-radius:8px;padding:6px 12px;font-size:12px;font-weight:600;}"
            "QPushButton:hover{border-color:#10B981;color:#10B981;}")
        self.dl_lang_btn.clicked.connect(self.download_language)
        lang_row.addWidget(self.dl_lang_btn)
        lang_row.addStretch()
        app_layout.addLayout(lang_row)

        self.cov_label = QLabel("")
        self.cov_label.setStyleSheet("color: #6B7280; font-size: 12px;")
        app_layout.addWidget(self.cov_label)

        layout.addWidget(app_frame)

        # --- AR overlay (calibration) ---
        ar_frame = QFrame()
        ar_frame.setStyleSheet("background-color: #FFFFFF; border: 1px solid #E5E7EB; border-radius: 12px; padding: 12px;")
        ar_lay = QVBoxLayout(ar_frame)
        ar_title = QLabel("AR overlay (experimental)")
        ar_title.setStyleSheet("font-size: 18px; font-weight: bold; color: #0F766E;")
        ar_lay.addWidget(ar_title)
        self.ar_toggle = QCheckBox("Draw the route on the road over the game")
        self.ar_toggle.setChecked(bool(self.state.get("ar_enabled", True)))
        self.ar_toggle.toggled.connect(lambda v: self.state.set("ar_enabled", bool(v)))
        ar_lay.addWidget(self.ar_toggle)

        def ar_slider(label, key, lo, hi, default, scale=1.0):
            row = QHBoxLayout()
            cap = QLabel(label)
            cur = self.state.get(key, default)
            cur = float(cur) if cur is not None else default
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(lo, hi); sl.setValue(int(cur * scale))
            sl.valueChanged.connect(lambda v: self.state.set(key, v / scale))
            row.addWidget(cap); row.addWidget(sl)
            ar_lay.addLayout(row)

        ar_slider("FOV", "ar_fov", 40, 100, 60.0)
        ar_slider("Height", "ar_height", 5, 60, 2.5, scale=10.0)   # 0.5–6.0 m
        ar_slider("Pitch", "ar_pitch", -20, 30, 8.0)
        layout.addWidget(ar_frame)

        # --- Performance sub-card (plugin RAM usage) ---
        try:
            from ui.performance import PerformancePage
            perf_frame = QFrame()
            perf_frame.setStyleSheet("background-color: #FFFFFF; border: 1px solid #E5E7EB; border-radius: 12px; padding: 6px;")
            pf_lay = QVBoxLayout(perf_frame)
            pf_lay.addWidget(PerformancePage(self.state))
            layout.addWidget(perf_frame)
        except Exception:
            pass

        layout.addStretch()
        self.setLayout(layout)

        # Publish initial values so plugins pick them up immediately.
        self.update_acc_speed(init_speed)
        self.update_acc_dist(init_gap)
        self.update_obey_limit(self.limit_toggle.isChecked())
        self.update_invert(self.invert_toggle.isChecked())
        self.update_sensitivity(init_sens)
        self.update_language(self.lang_combo.currentIndex())

    def update_theme(self, name):
        self.state.set("ui_theme", name.lower())

    def update_language(self, idx):
        """Language combo changed — ``idx`` is the row; data holds the code."""
        from core import i18n
        code = self.lang_combo.itemData(idx) if isinstance(idx, int) and idx >= 0 else "sk"
        if not code:
            return
        self.state.set("ui_language_code", code)
        cov = i18n.coverage(code)
        name = next((i["name"] for i in i18n.available() if i["code"] == code), code)
        self.cov_label.setText(f"{name} — {cov}% translated")

    def download_language(self):
        """Offer to download a language that isn't bundled/downloaded yet."""
        from PyQt6.QtWidgets import QInputDialog
        from core import i18n
        # Build a list of languages that aren't downloaded yet.
        choices = [i for i in i18n.available() if not i["downloaded"]]
        # Also include anything declared in index.json even if available() hasn't
        # surfaced it (defensive: usually they're already listed).
        if not choices:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.information(self, "UltraPilot",
                "Všetky dostupné jazyky sú už stiahnuté.")
            return
        labels = [f"{c['name']} ({c['english_name']})" for c in choices]
        choice, ok = QInputDialog.getItem(
            self, "Stiahnuť jazyk", "Vyber jazyk na stiahnutie:", labels, 0, False)
        if not ok or not choice:
            return
        info = choices[labels.index(choice)]
        self.dl_lang_btn.setEnabled(False)
        self.dl_lang_btn.setText("Sťahujem…")
        # Run the download in a worker thread so the UI doesn't freeze.
        from PyQt6.QtCore import QThread, pyqtSignal

        class _DL(QThread):
            done = pyqtSignal(bool, str)
            def __init__(self, code):
                super().__init__()
                self.code = code
            def run(self):
                ok = i18n.install_from_github(self.code)
                self.done.emit(bool(ok), self.code)

        self._dl_worker = _DL(info["code"])
        self._dl_worker.done.connect(self._on_lang_downloaded)
        self._dl_worker.start()

    def _on_lang_downloaded(self, ok, code):
        from PyQt6.QtWidgets import QMessageBox
        from core import i18n
        self.dl_lang_btn.setEnabled(True)
        self.dl_lang_btn.setText("Stiahnuť jazyk")
        if ok:
            i18n.reload()
            QMessageBox.information(self, "UltraPilot",
                f"Jazyk '{code}' bol stiahnutý.")
            # Refresh the combo with the newly available language.
            cur = self.lang_combo.currentData()
            self.lang_combo.blockSignals(True)
            self.lang_combo.clear()
            self._lang_codes = []
            for info in i18n.available():
                self._lang_codes.append(info["code"])
                label = f"{info['name']}  ·  {info['coverage']}%" if info["downloaded"] else f"{info['name']}  ·  (stiahnuteľné)"
                self.lang_combo.addItem(label, info["code"])
            for i in range(self.lang_combo.count()):
                if self.lang_combo.itemData(i) == cur:
                    self.lang_combo.setCurrentIndex(i)
                    break
            self.lang_combo.blockSignals(False)
            self.update_language(self.lang_combo.currentIndex())
        else:
            QMessageBox.warning(self, "UltraPilot",
                "Nepodarilo sa stiahnuť jazyk. Skontroluj internetové pripojenie "
                "alebo nastav GITHUB_TOKEN (repozitár môže byť súkromný).")

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
