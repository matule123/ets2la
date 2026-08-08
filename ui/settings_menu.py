import logging
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QSlider, QCheckBox, QFrame, QComboBox,
                             QPushButton)
from PyQt6.QtCore import Qt
from core.theme import palette


def _frame_qss(pal):
    """Card frame style derived from the active palette (theme-aware)."""
    return ("QFrame#SettingsCard{background-color:" + pal['card']
            + ";border:1px solid " + pal['border']
            + ";border-radius:16px;}")


def _title_qss(pal):
    return "font-size:15px;font-weight:750;color:" + pal['text'] + ";"


def _caption_qss(pal):
    return "color: " + pal['muted'] + ";"


def _icon_qss(pal, radius=9):
    dark = pal["bg"] == "#0D1117"
    background = "#172554" if dark else "#EFF6FF"
    border = "#1E3A8A" if dark else "#DBEAFE"
    return ("background:" + background + ";border:1px solid " + border
            + ";border-radius:" + str(radius) + "px;")


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
        self.setObjectName("SettingsPage")
        self.state = state
        self._pal = palette(state.get("ui_theme", "light") or "light")
        # Track widgets whose styles depend on the theme so we can recolour them
        # when the user flips dark/light without rebuilding the page.
        self._themed_frames = []
        self._themed_titles = []
        self._themed_captions = []
        self._themed_icons = []
        self.init_ui()

    def _section_card(self, icon_name, title, subtitle):
        from ui.icons import line_icon
        card = QFrame()
        card.setObjectName("SettingsCard")
        card.setStyleSheet(_frame_qss(self._pal))
        self._themed_frames.append(card)
        body = QVBoxLayout(card)
        body.setContentsMargins(18, 16, 18, 17)
        body.setSpacing(12)
        header = QHBoxLayout()
        header.setSpacing(10)
        icon = QLabel()
        icon.setFixedSize(34, 34)
        icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        icon.setPixmap(line_icon(icon_name, self._pal['title'], 24).pixmap(24, 24))
        icon.setStyleSheet(_icon_qss(self._pal))
        self._themed_icons.append((icon, icon_name, 24))
        header.addWidget(icon)
        texts = QVBoxLayout()
        texts.setSpacing(1)
        heading = QLabel(title)
        heading.setStyleSheet(_title_qss(self._pal))
        self._themed_titles.append(heading)
        detail = QLabel(subtitle)
        detail.setWordWrap(True)
        detail.setStyleSheet("font-size:11px;color:" + self._pal['muted'] + ";")
        self._themed_captions.append(detail)
        texts.addWidget(heading)
        texts.addWidget(detail)
        header.addLayout(texts, 1)
        body.addLayout(header)
        return card, body

    def _caption(self, text):
        label = QLabel(text)
        label.setStyleSheet("font-size:12px;font-weight:650;color:"
                            + self._pal['muted'] + ";")
        self._themed_captions.append(label)
        return label

    def init_ui(self):
        self.setStyleSheet(
            "QWidget#SettingsPage{background-color:" + self._pal['bg']
            + ";color:" + self._pal['text'] + ";font-family:'Segoe UI';}")

        layout = QVBoxLayout(self)
        layout.setSpacing(14)
        layout.setContentsMargins(22, 22, 22, 24)

        page_title = QLabel("Nastavenia")
        page_title.setObjectName("SettingsPageTitle")
        page_title.setStyleSheet("font-size:24px;font-weight:800;color:"
                                 + self._pal['text'] + ";")
        page_subtitle = QLabel(
            "Globálne nastavenia, ovládanie a doplnky UltraPilotu")
        page_subtitle.setObjectName("SettingsPageSubtitle")
        page_subtitle.setStyleSheet("font-size:12px;color:"
                                    + self._pal['muted'] + ";")
        self._page_title = page_title
        self._page_subtitle = page_subtitle
        layout.addWidget(page_title)
        layout.addWidget(page_subtitle)

        workspace = QFrame()
        workspace.setObjectName("SettingsWorkspace")
        workspace.setStyleSheet(
            "QFrame#SettingsWorkspace{background:" + self._pal['card']
            + ";border:1px solid " + self._pal['border']
            + ";border-radius:16px;}")
        self._workspace = workspace
        workspace_layout = QHBoxLayout(workspace)
        workspace_layout.setContentsMargins(0, 0, 0, 0)
        workspace_layout.setSpacing(0)

        nav_panel = QFrame()
        nav_panel.setObjectName("SettingsCategoryRail")
        nav_panel.setFixedWidth(205)
        nav_panel.setStyleSheet(
            "QFrame#SettingsCategoryRail{background:" + self._pal['card2']
            + ";border:none;border-right:1px solid " + self._pal['border']
            + ";border-top-left-radius:16px;border-bottom-left-radius:16px;}")
        self._nav_panel = nav_panel
        nav_layout = QVBoxLayout(nav_panel)
        nav_layout.setContentsMargins(12, 16, 12, 16)
        nav_layout.setSpacing(5)
        global_label = QLabel("APLIKÁCIA")
        global_label.setObjectName("SettingsCategoryLabel")
        nav_layout.addWidget(global_label)
        self._settings_nav_buttons = []

        def add_nav(text, section):
            button = QPushButton(text)
            button.setObjectName("SettingsCategoryButton")
            button.setCheckable(True)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
            button.clicked.connect(
                lambda _checked=False, key=section, target=button:
                self._show_settings_section(key, target))
            nav_layout.addWidget(button)
            self._settings_nav_buttons.append(button)
            return button

        default_nav = add_nav("Globálne", "global")
        add_nav("Ovládanie", "controls")
        add_nav("SDK", "sdk")
        plugins_label = QLabel("PLUGINY")
        plugins_label.setObjectName("SettingsCategoryLabel")
        nav_layout.addSpacing(13)
        nav_layout.addWidget(plugins_label)
        add_nav("Adaptívny tempomat", "controls")
        add_nav("Lane Control", "controls")
        nav_layout.addStretch()
        self._settings_category_labels = (global_label, plugins_label)
        workspace_layout.addWidget(nav_panel)

        content_panel = QFrame()
        content_panel.setObjectName("SettingsContentPanel")
        content_layout = QVBoxLayout(content_panel)
        content_layout.setContentsMargins(20, 18, 20, 20)
        content_layout.setSpacing(15)
        self._content_panel = content_panel

        hero = QFrame()
        hero.setObjectName("SettingsHero")
        hero.setStyleSheet(
            "QFrame#SettingsHero{background:" + self._pal['card']
            + ";border:1px solid " + self._pal['border']
            + ";border-radius:16px;}")
        self._hero = hero
        hero_layout = QHBoxLayout(hero)
        hero_layout.setContentsMargins(20, 16, 20, 16)
        hero_layout.setSpacing(14)
        from ui.icons import line_icon
        hero_icon = QLabel()
        hero_icon.setFixedSize(46, 46)
        hero_icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        hero_icon.setPixmap(
            line_icon("settings", self._pal['title'], 30).pixmap(30, 30))
        hero_icon.setStyleSheet(_icon_qss(self._pal, 13))
        self._hero_icon = hero_icon
        hero_layout.addWidget(hero_icon)
        hero_text = QVBoxLayout()
        hero_text.setSpacing(2)
        self._hero_title = QLabel("Nastavenia")
        self._hero_title.setStyleSheet("font-size:22px;font-weight:800;color:"
                                       + self._pal['text'] + ";")
        self._hero_subtitle = QLabel(
            "Jazda, riadenie a vzhľad aplikácie na jednom mieste")
        self._hero_subtitle.setStyleSheet("font-size:12px;color:"
                                          + self._pal['muted'] + ";")
        hero_text.addWidget(self._hero_title)
        hero_text.addWidget(self._hero_subtitle)
        hero_layout.addLayout(hero_text, 1)
        content_layout.addWidget(hero)

        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(16)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        # --- ACC Section ---
        acc_frame, acc_layout = self._section_card(
            "dashboard", "Adaptívny tempomat",
            "Rýchlosť a bezpečný odstup od vozidla pred tebou")

        # Target Speed Slider
        init_speed = int(self.state.get("acc_target_speed", 80) or 80)
        self.speed_label = self._caption(f"Cieľová rýchlosť  ·  {init_speed} km/h")
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(30, 140)
        self.speed_slider.setValue(init_speed)
        self.speed_slider.valueChanged.connect(self.update_acc_speed)
        acc_layout.addWidget(self.speed_label)
        acc_layout.addWidget(self.speed_slider)

        # Follow-distance (time gap) Slider
        init_gap = int((self.state.get("acc_safe_distance", 2.0) or 2.0) * 10)
        self.dist_label = self._caption(
            f"Bezpečný odstup  ·  {init_gap / 10.0:.1f} s")
        self.dist_slider = QSlider(Qt.Orientation.Horizontal)
        self.dist_slider.setRange(5, 40)  # 0.5 to 4.0s
        self.dist_slider.setValue(init_gap)
        self.dist_slider.valueChanged.connect(self.update_acc_dist)
        acc_layout.addWidget(self.dist_label)
        acc_layout.addWidget(self.dist_slider)

        # Obey posted speed limit
        self.limit_toggle = QCheckBox("Dodržiavať rýchlostné obmedzenia")
        self.limit_toggle.setChecked(bool(self.state.get("acc_obey_limit", True)))
        self.limit_toggle.toggled.connect(self.update_obey_limit)
        acc_layout.addWidget(self.limit_toggle)

        grid.addWidget(acc_frame, 0, 0)
        self._acc_frame = acc_frame

        # --- Steering Section ---
        steer_frame, steer_layout = self._section_card(
            "steering", "Riadenie",
            "Smer a citlivosť výstupu pre volant")

        self.invert_toggle = QCheckBox("Obrátiť smer riadenia")
        self.invert_toggle.setChecked(bool(self.state.get("steering_invert", False)))
        self.invert_toggle.toggled.connect(self.update_invert)
        steer_layout.addWidget(self.invert_toggle)

        init_sens = int((self.state.get("steering_sensitivity", 1.0) or 1.0) * 100)
        self.sens_label = self._caption(
            f"Citlivosť  ·  {init_sens / 100:.2f}×")
        self.sens_slider = QSlider(Qt.Orientation.Horizontal)
        self.sens_slider.setRange(30, 200)  # 0.3× .. 2.0×
        self.sens_slider.setValue(init_sens)
        self.sens_slider.valueChanged.connect(self.update_sensitivity)
        steer_layout.addWidget(self.sens_label)
        steer_layout.addWidget(self.sens_slider)
        steer_layout.addStretch()
        grid.addWidget(steer_frame, 0, 1)
        self._steer_frame = steer_frame

        # --- Appearance Section (theme + language) ---
        app_frame, app_layout = self._section_card(
            "visualization", "Vzhľad a jazyk",
            "Farebný režim a jazyk používateľského rozhrania")

        theme_row = QHBoxLayout()
        theme_lbl = self._caption("Téma")
        theme_row.addWidget(theme_lbl)
        self.theme_combo = QComboBox()
        self.theme_combo.addItems(["Light", "Dark", "System"])
        cur = (self.state.get("ui_theme", "light") or "light").capitalize()
        self.theme_combo.setCurrentText(cur)
        self.theme_combo.currentTextChanged.connect(self.update_theme)
        theme_row.addWidget(self.theme_combo)
        theme_row.addStretch()
        app_layout.addLayout(theme_row)

        lang_row = QHBoxLayout()
        lang_lbl = self._caption("Jazyk")
        lang_row.addWidget(lang_lbl)
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
        self.dl_lang_btn.setStyleSheet(self._btn_qss())
        self.dl_lang_btn.clicked.connect(self.download_language)
        lang_row.addWidget(self.dl_lang_btn)
        lang_row.addStretch()
        app_layout.addLayout(lang_row)

        self.cov_label = QLabel("")
        self.cov_label.setStyleSheet("color: " + self._pal['muted'] + "; font-size: 12px;")
        app_layout.addWidget(self.cov_label)

        grid.addWidget(app_frame, 1, 0)
        self._app_frame = app_frame

        # --- Sound (startup chime) ---
        sound_frame, snd_lay = self._section_card(
            "about", "Zvuk aplikácie",
            "Jemné zvukové potvrdenie pri spustení UltraPilotu")
        self.sound_toggle = QCheckBox("Prehrať uvítací zvuk pri štarte")
        self.sound_toggle.setChecked(bool(self.state.get("startup_sound", True)))
        self.sound_toggle.toggled.connect(lambda v: self.state.set("startup_sound", bool(v)))
        snd_lay.addWidget(self.sound_toggle)
        performance_hint = self._caption(
            "Podrobné využitie RAM, CPU a pluginov otvoríš tlačidlom „Výkon aplikácie“ v bočnom paneli.")
        performance_hint.setWordWrap(True)
        snd_lay.addWidget(performance_hint)
        grid.addWidget(sound_frame, 1, 1)
        self._sound_frame = sound_frame

        sdk_frame, sdk_layout = self._section_card(
            "plugins", "SCS SDK a runtime",
            "Pripojenie telemetrie, virtuálneho ovládača a herných pluginov")
        sdk_info = QLabel(
            "UltraPilot kontroluje potrebné SDK knižnice pri štarte. Ak niektorá "
            "súčasť chýba, aplikácia zostane bezpečne vypnutá a uvedie presný dôvod.")
        sdk_info.setWordWrap(True)
        sdk_info.setStyleSheet("font-size:12px;color:" + self._pal['muted'] + ";")
        self._themed_captions.append(sdk_info)
        sdk_layout.addWidget(sdk_info)
        grid.addWidget(sdk_frame, 2, 0, 1, 2)
        self._sdk_frame = sdk_frame

        self._perf_page = None
        content_layout.addLayout(grid)
        content_layout.addStretch()
        workspace_layout.addWidget(content_panel, 1)
        layout.addWidget(workspace)
        layout.addStretch()
        default_nav.setChecked(True)
        self._show_settings_section("global", default_nav)
        self._restyle_workspace()

        # Publish initial values so plugins pick them up immediately.
        self.update_acc_speed(init_speed)
        self.update_acc_dist(init_gap)
        self.update_obey_limit(self.limit_toggle.isChecked())
        self.update_invert(self.invert_toggle.isChecked())
        self.update_sensitivity(init_sens)
        self.update_language(self.lang_combo.currentIndex())

    def _show_settings_section(self, section, selected_button=None):
        """Switch the nested settings board without rebuilding live controls."""
        for button in getattr(self, "_settings_nav_buttons", []):
            button.setChecked(button is selected_button)
        groups = {
            "global": (self._app_frame, self._sound_frame),
            "controls": (self._acc_frame, self._steer_frame),
            "sdk": (self._sdk_frame,),
        }
        visible = set(groups.get(section, groups["global"]))
        for frame in (self._acc_frame, self._steer_frame, self._app_frame,
                      self._sound_frame, self._sdk_frame):
            frame.setVisible(frame in visible)
        headings = {
            "global": ("Globálne nastavenia",
                       "Vzhľad, jazyk a správanie celej aplikácie."),
            "controls": ("Nastavenia ovládania",
                         "Adaptívny tempomat a výstup riadenia kamióna."),
            "sdk": ("SDK a runtime",
                    "Stav rozhraní potrebných na komunikáciu s ETS2."),
        }
        title, subtitle = headings.get(section, headings["global"])
        self._hero_title.setText(title)
        self._hero_subtitle.setText(subtitle)

    def _restyle_workspace(self):
        p = self._pal
        self._workspace.setStyleSheet(
            "QFrame#SettingsWorkspace{background:" + p['card']
            + ";border:1px solid " + p['border'] + ";border-radius:16px;}"
            "QLabel#SettingsCategoryLabel{color:" + p['muted']
            + ";font-size:10px;font-weight:700;padding:5px 8px;border:none;}"
            "QPushButton#SettingsCategoryButton{background:transparent;color:"
            + p['text'] + ";border:1px solid transparent;border-radius:8px;"
            "padding:8px 10px;text-align:left;font-size:12px;font-weight:550;}"
            "QPushButton#SettingsCategoryButton:hover{background:" + p['card']
            + ";border-color:" + p['border'] + ";}"
            "QPushButton#SettingsCategoryButton:checked{background:" + p['card']
            + ";border-color:" + p['border'] + ";font-weight:700;color:"
            + p['text'] + ";}")
        self._nav_panel.setStyleSheet(
            "QFrame#SettingsCategoryRail{background:" + p['card2']
            + ";border:none;border-right:1px solid " + p['border']
            + ";border-top-left-radius:16px;border-bottom-left-radius:16px;}")

    def update_theme(self, name):
        self.state.set("ui_theme", name.lower())

    def _btn_qss(self):
        p = self._pal
        return ("QPushButton{background:" + p['card2'] + ";color:" + p['text'] +
                ";border:1px solid " + p['border'] +
                ";border-radius:8px;padding:6px 12px;font-size:12px;font-weight:600;}"
                "QPushButton:hover{border-color:" + p['title'] + ";color:" + p['title'] + ";}")

    def restyle(self, theme):
        """Re-apply colours when the theme changes (called by UltraPilotApp)."""
        self._pal = palette(theme)
        p = self._pal
        self.setStyleSheet(
            "QWidget#SettingsPage{background-color:" + p['bg']
            + ";color:" + p['text'] + ";font-family:'Segoe UI';}")
        self._page_title.setStyleSheet(
            "font-size:24px;font-weight:800;color:" + p['text'] + ";")
        self._page_subtitle.setStyleSheet(
            "font-size:12px;color:" + p['muted'] + ";")
        self._restyle_workspace()
        for fr in getattr(self, "_themed_frames", []):
            fr.setStyleSheet(_frame_qss(p))
        for ttl in getattr(self, "_themed_titles", []):
            ttl.setStyleSheet(_title_qss(p))
        for caption in getattr(self, "_themed_captions", []):
            caption.setStyleSheet("font-size:12px;color:" + p['muted'] + ";")
        from ui.icons import line_icon
        for icon, icon_name, size in getattr(self, "_themed_icons", []):
            icon.setPixmap(line_icon(icon_name, p['title'], size).pixmap(
                size, size))
            icon.setStyleSheet(_icon_qss(p))
        if hasattr(self, "_hero"):
            self._hero.setStyleSheet(
                "QFrame#SettingsHero{background:" + p['card']
                + ";border:1px solid " + p['border']
                + ";border-radius:16px;}")
            self._hero_title.setStyleSheet(
                "font-size:22px;font-weight:800;color:" + p['text'] + ";")
            self._hero_subtitle.setStyleSheet(
                "font-size:12px;color:" + p['muted'] + ";")
            self._hero_icon.setPixmap(
                line_icon("settings", p['title'], 30).pixmap(30, 30))
            self._hero_icon.setStyleSheet(_icon_qss(p, 13))
        if hasattr(self, "cov_label"):
            self.cov_label.setStyleSheet("color: " + p['muted'] + "; font-size: 12px;")
        if hasattr(self, "dl_lang_btn"):
            self.dl_lang_btn.setStyleSheet(self._btn_qss())
        if getattr(self, "_perf_page", None) is not None and hasattr(self._perf_page, "restyle"):
            self._perf_page.restyle(theme)

    def update_language(self, idx):
        """Language combo changed — ``idx`` is the row; data holds the code."""
        from core import i18n
        code = self.lang_combo.itemData(idx) if isinstance(idx, int) and idx >= 0 else "sk"
        if not code:
            return
        self.state.set("ui_language_code", code)
        try:
            from core.settings.manager import SettingsManager
            SettingsManager().set("ui_language_code", code)
        except Exception:
            pass
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
        self.speed_label.setText(f"Cieľová rýchlosť  ·  {val} km/h")
        self.state.set("acc_target_speed", float(val))

    def update_acc_dist(self, val):
        dist = val / 10.0
        self.dist_label.setText(f"Bezpečný odstup  ·  {dist:.1f} s")
        self.state.set("acc_safe_distance", dist)

    def update_obey_limit(self, checked):
        self.state.set("acc_obey_limit", bool(checked))

    def update_invert(self, checked):
        self.state.set("steering_invert", bool(checked))

    def update_sensitivity(self, val):
        s = val / 100.0
        self.sens_label.setText(f"Citlivosť  ·  {s:.2f}×")
        self.state.set("steering_sensitivity", s)
