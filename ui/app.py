import sys
import os
import logging
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QWidget, QStackedWidget, QFrame, QScrollArea,
)
from PyQt6.QtCore import QTimer, Qt, QSize

from ui.settings_menu import SettingsMenu
from ui.map_page import MapPage

# All theming comes from core.theme.stylesheet() applied in UltraPilotApp; the
# old inline LIGHT_THEME/DARK_THEME strings here were dead code (never applied).


class MacTitleBar(QFrame):
    """Compact draggable title bar with Apple-style window controls."""

    def __init__(self, window, palette):
        super().__init__()
        self.window = window
        self._drag_offset = None
        self.setFixedHeight(38)
        self.setObjectName("MacTitleBar")
        self.setStyleSheet(
            "#MacTitleBar{background:" + palette['surface'] + ";"
            "border:none;border-bottom:1px solid " + palette['border'] + ";}")
        row = QHBoxLayout(self)
        row.setContentsMargins(13, 0, 13, 0)
        row.setSpacing(8)
        for color, tip, action in (
                ("#FF5F57", "Zavrieť", window.close),
                ("#FEBC2E", "Minimalizovať", window.showMinimized),
                ("#28C840", "Maximalizovať", self._toggle_maximize)):
            dot = QPushButton("")
            dot.setToolTip(tip)
            dot.setFixedSize(14, 14)
            dot.setStyleSheet(
                f"QPushButton{{background:{color};border:1px solid rgba(0,0,0,0.18);"
                "border-radius:7px;padding:0;margin:0;}"
                "QPushButton:hover{border:2px solid rgba(0,0,0,0.32);}")
            dot.clicked.connect(action)
            row.addWidget(dot)
        row.addSpacing(6)
        title = QLabel("UltraPilot")
        title.setStyleSheet("font-size:12px;font-weight:700;color:" + palette['muted'] + ";border:none;")
        row.addWidget(title)
        row.addStretch()

    def _toggle_maximize(self):
        self.window.showNormal() if self.window.isMaximized() else self.window.showMaximized()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_offset = event.globalPosition().toPoint() - self.window.frameGeometry().topLeft()

    def mouseMoveEvent(self, event):
        if self._drag_offset is not None and event.buttons() & Qt.MouseButton.LeftButton:
            if self.window.isMaximized():
                self.window.showNormal()
            self.window.move(event.globalPosition().toPoint() - self._drag_offset)

    def mouseReleaseEvent(self, event):
        self._drag_offset = None

    def mouseDoubleClickEvent(self, event):
        self._toggle_maximize()


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
        from core.theme import palette
        self._pal = palette(state.get("ui_theme", "light") or "light")
        self.title = QLabel("ℹ️ About UltraPilot")
        self.title.setStyleSheet("font-size: 24px; font-weight: bold; color: " + self._pal['title'] + "; margin-bottom: 20px;")
        self.layout.addWidget(self.title)
        self.text = QLabel(
            "UltraPilot\n\n"
            "A professional-grade autopilot for Euro Truck Simulator 2.\n"
            "Lane Assist, Adaptive Cruise Control, Collision Avoidance, "
            "Navigation, HUD and Voice — each plugin isolated in its own process."
        )
        self.text.setWordWrap(True)
        self.text.setStyleSheet("font-size: 16px; color: " + self._pal['text'] + ";")
        self.layout.addWidget(self.text)
        self.layout.addStretch()

    def restyle(self, theme):
        from core.theme import palette
        self._pal = palette(theme)
        self.title.setStyleSheet("font-size: 24px; font-weight: bold; color: " + self._pal['title'] + "; margin-bottom: 20px;")
        self.text.setStyleSheet("font-size: 16px; color: " + self._pal['text'] + ";")


class PluginsPage(Page):
    def __init__(self, state):
        super().__init__(state)
        from core.theme import palette
        self._pal = palette(state.get("ui_theme", "light") or "light")
        self._themed_rows = []
        self.title = QLabel("🧩 Plugin Management")
        self.title.setStyleSheet("font-size: 24px; font-weight: bold; color: " + self._pal['title'] + "; margin-bottom: 20px;")
        self.layout.addWidget(self.title)

        self.plugin_list = QVBoxLayout()
        self.layout.addLayout(self.plugin_list)
        self.layout.addStretch()
        self.refresh_plugins()

    def restyle(self, theme):
        from core.theme import palette
        self._pal = palette(theme)
        self.title.setStyleSheet("font-size: 24px; font-weight: bold; color: " + self._pal['title'] + "; margin-bottom: 20px;")
        # Re-render rows so the new palette applies.
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
        names = [f for f in sorted(os.listdir(plugin_dir))
                 if os.path.isdir(os.path.join(plugin_dir, f))
                 and os.path.exists(os.path.join(plugin_dir, f, "main.py"))]
        # Enabled plugins on top, disabled below.
        names.sort(key=lambda n: (not self.state.get(f"plugin_enabled.{n}", True), n))
        enabled = [n for n in names if self.state.get(f"plugin_enabled.{n}", True)]
        disabled = [n for n in names if not self.state.get(f"plugin_enabled.{n}", True)]
        if enabled:
            self._section("● ACTIVE")
            for n in enabled:
                self.add_plugin_row(n)
        if disabled:
            self._section("○ DISABLED")
            for n in disabled:
                self.add_plugin_row(n)

    def _section(self, text):
        lbl = QLabel(text)
        lbl.setStyleSheet("color:" + self._pal['muted'] + "; font-size:12px; font-weight:700; margin-top:8px;")
        self.plugin_list.addWidget(lbl)

    _DESC = {
        "autopilot": "Steering + throttle/brake control",
        "acc": "Adaptive cruise control",
        "collision": "Emergency braking & collision avoidance",
        "map": "Coordinate / map navigation",
        "tts": "Voice announcements",
        "discord": "Discord rich presence",
        "ecodrive": "Fuel-saving throttle smoothing",
        "hud": "On-screen HUD elements",
    }

    def add_plugin_row(self, name):
        row = QFrame()
        row.setStyleSheet("background-color:" + self._pal['card'] + "; border:1px solid " + self._pal['border'] + "; border-radius:10px;")
        l = QHBoxLayout(row)
        l.setContentsMargins(14, 10, 14, 10)

        info = QVBoxLayout()
        lbl = QLabel(name.capitalize())
        lbl.setStyleSheet("color:" + self._pal['text'] + "; font-size:15px; font-weight:700; border:none;")
        desc = QLabel(self._DESC.get(name, ""))
        desc.setStyleSheet("color:" + self._pal['muted'] + "; font-size:12px; border:none;")
        info.addWidget(lbl); info.addWidget(desc)
        l.addLayout(info)
        l.addStretch()

        btn = QPushButton()
        btn.setFixedWidth(120)

        def render():
            enabled = self.state.get(f"plugin_enabled.{name}", True)
            btn.setText("● ENABLED" if enabled else "○ DISABLED")
            bg = self._pal['success'] if enabled else self._pal['muted']
            btn.setStyleSheet(
                f"background-color:{bg}; color:#FFFFFF;"
                "border:none; border-radius:8px; padding:8px; font-weight:700;")

        def toggle():
            current = self.state.get(f"plugin_enabled.{name}", True)
            self.state.set(f"plugin_enabled.{name}", not current)
            logging.info(f"Toggled plugin '{name}' -> {not current}")
            self.refresh_plugins()   # re-sort: active up, disabled down

        btn.clicked.connect(toggle)
        l.addWidget(btn)
        self.plugin_list.addWidget(row)
        render()


class DashboardPage(Page):
    def __init__(self, state):
        super().__init__(state)
        from core.theme import palette
        self._pal = palette(state.get("ui_theme", "light") or "light")
        title = QLabel("🚀 UltraPilot Telemetry")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: " + self._pal['title'] + "; margin-bottom: 10px;")
        self.layout.addWidget(title)

        # --- Prominent autopilot status card (the eye-catcher of the page) ---
        self.ap_card = QFrame()
        self.ap_card.setObjectName("ApCard")
        self.ap_card.setStyleSheet(
            "#ApCard { background-color: " + self._pal['card'] + "; border: 1px solid " + self._pal['border'] + "; "
            "border-radius: 16px; }")
        ap_l = QVBoxLayout(self.ap_card)
        ap_l.setContentsMargins(24, 20, 24, 20)
        ap_l.setSpacing(6)

        ap_head = QHBoxLayout()
        self.ap_dot = QLabel("●")
        self.ap_dot.setStyleSheet("font-size: 22px; color: " + self._pal['muted'] + "; border:none;")
        ap_head.addWidget(self.ap_dot)
        self.ap_title = QLabel("Autopilot vypnutý")
        self.ap_title.setStyleSheet("font-size: 20px; font-weight: bold; color: " + self._pal['text'] + "; border:none;")
        ap_head.addWidget(self.ap_title)
        ap_head.addStretch()
        self.ap_state = QLabel("MANUÁL")
        self.ap_state.setStyleSheet("color: " + self._pal['muted'] + "; font-size: 12px; font-weight: 700; border:none;")
        ap_head.addWidget(self.ap_state)
        ap_l.addLayout(ap_head)

        # Big speed readout beside the system state.
        speed_row = QHBoxLayout()
        self.speed_val = QLabel("0")
        self.speed_val.setStyleSheet("font-size: 56px; font-weight: bold; color: " + self._pal['title'] + "; border:none;")
        speed_row.addWidget(self.speed_val)
        sp_unit = QVBoxLayout()
        sp_lbl = QLabel("Aktuálna rýchlosť")
        sp_lbl.setStyleSheet("color: " + self._pal['muted'] + "; font-size: 11px; font-weight: 600; border:none;")
        self.speed_unit = QLabel("km/h")
        self.speed_unit.setStyleSheet("color: " + self._pal['text'] + "; font-size: 16px; font-weight: 700; border:none;")
        sp_unit.addWidget(sp_lbl); sp_unit.addWidget(self.speed_unit)
        sp_unit.addStretch()
        speed_row.addLayout(sp_unit)
        speed_row.addStretch()
        ap_l.addLayout(speed_row)
        self.layout.addWidget(self.ap_card)

        # --- Live telemetry grid (gear / rpm / fuel / limit / nav) ---
        self.metrics = {}
        grid_frame = QFrame()
        grid_frame.setObjectName("Card")
        grid = QHBoxLayout(grid_frame)
        grid.setContentsMargins(8, 12, 8, 12)
        for key, icon, label in [("gear", "⚙️", "PREVOD"), ("rpm", "🔧", "OTÁČKY"),
                                 ("fuel", "⛽", "PALIVO"), ("limit", "🚦", "LIMIT"),
                                 ("nav", "🧭", "NAVIGÁCIA")]:
            col = QVBoxLayout()
            col.setSpacing(2)
            cap = QLabel(f"{icon}  {label}")
            cap.setStyleSheet("color: " + self._pal['muted'] + "; font-size: 11px; font-weight: bold; border:none;")
            val = QLabel("—")
            val.setStyleSheet("color: " + self._pal['text'] + "; font-size: 22px; font-weight: bold; border:none;")
            col.addWidget(cap)
            col.addWidget(val)
            grid.addLayout(col)
            self.metrics[key] = val
        self.layout.addWidget(grid_frame)

        self.conn_val = QLabel("● Čakám na telemetriu z hry…")
        self.conn_val.setStyleSheet("color: " + self._pal['muted'] + "; font-size: 12px; margin-top: 6px;")
        self.layout.addWidget(self.conn_val)

        self.layout.addStretch()

    def restyle(self, theme):
        """Re-apply palette colours when the theme switches (dark ↔ light)."""
        from core.theme import palette
        self._pal = palette(theme)
        # refresh() re-sets every card/label style from self._pal.
        self.refresh()

    def refresh(self):
        speed = self.state.get("speed", 0) or 0
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            speed = 0.0
        speed_kmh = speed * 3.6 if abs(speed) < 200 else speed
        self.speed_val.setText(f"{abs(speed_kmh):.0f}")

        sysstate = self.state.get("system_state", "IDLE")
        active = self.state.get("autopilot_active", False)
        # The autopilot card reflects the master switch: green when driving,
        # grey when manual. The system state (CRUISE / FOLLOW_LANE / …) is the
        # fine-grained sub-state shown as the chip.
        if active:
            self.ap_dot.setStyleSheet("font-size: 22px; color: " + self._pal['success'] + "; border:none;")
            self.ap_title.setText("Autopilot aktívny")
            self.ap_title.setStyleSheet("font-size: 20px; font-weight: bold; color: " + self._pal['title'] + "; border:none;")
            self.ap_state.setText(str(sysstate))
            self.ap_state.setStyleSheet("color: " + self._pal['success'] + "; font-size: 12px; font-weight: 700; border:none;")
            # Active = ETS2LA hero gradient (green-tinted) with an accent border.
            self.ap_card.setStyleSheet(
                "#ApCard { background: qlineargradient(x1:0,y1:0,x2:1,y2:1,"
                " stop:0 " + self._pal['card2'] + ", stop:1 " + self._pal['hero_b'] + ");"
                " border: 1px solid " + self._pal['accent2'] + "; border-radius: 16px; }")
            self.speed_val.setStyleSheet("font-size: 56px; font-weight: bold; color: " + self._pal['title'] + "; border:none;")
        else:
            self.ap_dot.setStyleSheet("font-size: 22px; color: " + self._pal['muted'] + "; border:none;")
            self.ap_title.setText("Autopilot vypnutý")
            self.ap_title.setStyleSheet("font-size: 20px; font-weight: bold; color: " + self._pal['text'] + "; border:none;")
            self.ap_state.setText("MANUÁL")
            self.ap_state.setStyleSheet("color: " + self._pal['muted'] + "; font-size: 12px; font-weight: 700; border:none;")
            # Inactive = calm flat card with a soft border.
            self.ap_card.setStyleSheet(
                "#ApCard { background-color: " + self._pal['card'] + "; border: 1px solid " + self._pal['border'] + "; "
                "border-radius: 16px; }")
            self.speed_val.setStyleSheet("font-size: 56px; font-weight: bold; color: " + self._pal['text'] + "; border:none;")

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
            self.conn_val.setText("● Telemetria pripojená")
            self.conn_val.setStyleSheet("color: " + self._pal['success'] + "; font-size: 12px; margin-top: 6px;")
        else:
            self.conn_val.setText("● Čakám na telemetriu z hry…")
            self.conn_val.setStyleSheet("color: " + self._pal['muted'] + "; font-size: 12px; margin-top: 6px;")


class UltraPilotApp(QMainWindow):
    """Control panel. Runs in its own process and talks to the engine purely
    through shared state — START/STOP flips the ``autopilot_active`` master
    switch rather than starting/stopping the engine object directly."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        self.setWindowTitle("UltraPilot")
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.Window)
        self.resize(1000, 640)
        self.setMinimumSize(880, 560)
        from core.theme import stylesheet, palette
        self._theme = (state.get("ui_theme", "light") or "light")
        self._pal = palette(self._theme)
        self.setStyleSheet(stylesheet(self._theme))
        # Window + taskbar icon. On Windows the taskbar icon only shows if we
        # set an explicit AppUserModelID before any window appears.
        from PyQt6.QtGui import QIcon
        from core.paths import resource
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("UltraPilot.App")
        except Exception:
            pass
        _ico = resource("assets", "favicon.ico")
        if os.path.exists(_ico):
            icon = QIcon(_ico)
            self.setWindowIcon(icon)
            app = QApplication.instance()
            if app is not None:
                app.setWindowIcon(icon)

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        self.title_bar = MacTitleBar(self, self._pal)
        root_layout.addWidget(self.title_bar)
        content = QWidget()
        main_layout = QHBoxLayout(content)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        root_layout.addWidget(content, 1)

        self.sidebar = QFrame()
        self.sidebar.setObjectName("Sidebar")
        self.sidebar.setFixedWidth(224)
        sb = QVBoxLayout(self.sidebar)
        sb.setContentsMargins(0, 16, 0, 12)
        sb.setSpacing(0)

        # Brand block at the top: logo + wordmark + version.
        from PyQt6.QtGui import QPixmap, QIcon
        from core.paths import resource as _res
        brand_row = QHBoxLayout()
        brand_row.setContentsMargins(18, 0, 12, 0)
        brand_row.setSpacing(10)
        logo = QLabel()
        _pm = QIcon(_res("assets", "favicon.ico")).pixmap(40, 40)
        if _pm.isNull():
            _pm = QPixmap(_res("assets", "logo.png")).scaledToWidth(
                40, Qt.TransformationMode.SmoothTransformation)
        if not _pm.isNull():
            logo.setPixmap(_pm)
        logo.setStyleSheet("border:none;")
        brand_row.addWidget(logo)
        brand_txt = QVBoxLayout()
        brand_txt.setSpacing(0)
        word = QLabel("UltraPilot")
        word.setStyleSheet("font-size: 18px; font-weight: 800; color: " + self._pal['title'] + "; border:none;")
        brand_txt.addWidget(word)
        # The update checker widget lives where „Pro Edition“ used to — it shows
        # the current version and an Update button when a newer release exists.
        from ui.update_widget import UpdateCheckerWidget
        self.update_checker = UpdateCheckerWidget(self.state)
        brand_txt.addWidget(self.update_checker)
        brand_row.addLayout(brand_txt)
        brand_row.addStretch()
        brand_w = QWidget()
        brand_w.setLayout(brand_row)
        brand_w.setStyleSheet("border:none;")
        sb.addWidget(brand_w)
        sb.addSpacing(18)

        # ETS2LA-style navigation. The glyphs come from Windows' monochrome
        # Segoe MDL2 icon font (not emoji), so they stay crisp at every DPI.
        nav = [
            ("Main", None, None),
            ("dashboard", "Dashboard", 0),
            ("navigation", "Navigation", 1),
            ("visualization", "Visualization", 2),
            ("Plugins", None, None),
            ("plugins", "Manager", 3),
            ("Application", None, None),
            ("settings", "Settings", 4),
            ("about", "About", 5),
        ]
        self._nav_btns = []
        for icon_text, text, idx in nav:
            if idx is None:
                section = QLabel(icon_text)
                section.setObjectName("NavSection")
                section.setStyleSheet(
                    "color:#7B818A;font-size:11px;font-weight:500;"
                    "padding:14px 18px 5px 18px;border:none;")
                sb.addWidget(section)
                continue
            from ui.icons import line_icon
            b = QPushButton(text)
            b.setIcon(line_icon(icon_text))
            b.setIconSize(QSize(20, 20))
            b.setObjectName("NavButton")
            b.setProperty("navIndex", idx)
            b.setProperty("navKey", "plugins" if text == "Manager" else text.lower())
            b.setStyleSheet(
                "QPushButton{font-family:'Segoe UI';"
                "font-size:13px;text-align:left;background:transparent;color:#30343B;"
                "border:none;border-radius:7px;padding:8px 12px;margin:1px 10px;}"
                "QPushButton:hover{background:#F5F5F6;color:#111827;}"
                "QPushButton:checked{background:#EEEEF0;color:#111827;font-weight:600;}"
            )
            b.setCheckable(True)
            b.clicked.connect(lambda _=False, i=idx: self._goto(i))
            sb.addWidget(b)
            self._nav_btns.append(b)
        self._nav_btns[0].setChecked(True)
        sb.addStretch()

        # Sidebar footer: live connection + autopilot indicator.
        self.side_conn = QLabel("● Čakám na hru")
        self.side_conn.setStyleSheet(
            "color: " + self._pal['muted'] + "; font-size: 11px; font-weight: 600; border:none; "
            "padding: 10px 18px;")
        sb.addWidget(self.side_conn)

        # Hamburger button: toggles the small floating performance overlay.
        self.perf_overlay = None
        self.perf_btn = QPushButton("◫  Performance")
        self.perf_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.perf_btn.setFixedHeight(34)
        self.perf_btn.setToolTip("Performance")
        self.perf_btn.setStyleSheet(
            "QPushButton{background:transparent;border:1px solid " + self._pal['border'] + ";"
            "border-radius:8px;color:" + self._pal['muted'] + ";font-size:12px;font-weight:700;}"
            "QPushButton:hover{border-color:" + self._pal['title'] + ";color:" + self._pal['title'] + ";}")
        # Black/white style toggle kept minimal — colour flips with state below.
        self.perf_btn.clicked.connect(self.toggle_perf_overlay)
        sb.addWidget(self.perf_btn)
        main_layout.addWidget(self.sidebar)

        self.pages = QStackedWidget()
        # Build each page defensively so one broken page can't stop the app.
        def _add(factory, name):
            try:
                page = factory()
                # Wrap every page in a scroll area so tall content (Plugins,
                # Settings) is reachable instead of clipped at the window edge.
                scroll = QScrollArea()
                scroll.setWidgetResizable(True)
                scroll.setFrameShape(QFrame.Shape.NoFrame)
                scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
                scroll.setWidget(page)
                self.pages.addWidget(scroll)
            except Exception as e:
                logging.error("UI page '%s' failed: %s", name, e)
                err = QLabel(f"{name} unavailable:\n{e}")
                err.setWordWrap(True)
                self.pages.addWidget(err)

        from ui.visualization import VisualizationPage
        _add(lambda: DashboardPage(state), "Dashboard")
        _add(lambda: MapPage(state), "Navigation")
        _add(lambda: VisualizationPage(state), "Visualization")
        _add(lambda: PluginsPage(state), "Plugins")
        _add(lambda: SettingsMenu(state), "Settings")
        _add(lambda: AboutPage(state), "About")
        main_layout.addWidget(self.pages)

        self.start_btn = QPushButton("▶ ZAPNÚŤ AUTOPILOT")
        self.start_btn.clicked.connect(self.toggle_autopilot)
        self.statusBar().addWidget(self.start_btn)
        self._render_start_btn()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)
        self._language = state.get("ui_language_code", "sk") or "sk"
        self._apply_language(self._language)

        # Dynamic Island: a floating pill at the top that shows live log output
        # (INFO green / WARNING amber / ERROR red + grey timestamp + source).
        try:
            from ui.dynamic_island import DynamicIsland
            self.island = DynamicIsland.install(self)
        except Exception as e:
            logging.debug("Dynamic Island unavailable: %s", e)

    def _render_start_btn(self):
        from core.i18n import t
        lang = self.state.get("ui_language_code", "sk") or "sk"
        active = self.state.get("autopilot_active", False)
        if active:
            self.start_btn.setText("■  " + t(lang, "app", "disable_ap"))
            self.start_btn.setStyleSheet("background-color: " + self._pal['danger'] + "; color: #FFFFFF; font-weight: bold; padding: 8px 18px; border-radius: 8px;")
        else:
            self.start_btn.setText("▶  " + t(lang, "app", "enable_ap"))
            self.start_btn.setStyleSheet("background-color: " + self._pal['success'] + "; color: #FFFFFF; font-weight: bold; padding: 8px 18px; border-radius: 8px;")

    def toggle_autopilot(self):
        import time
        current = bool(self.state.get("autopilot_active", False))
        desired = not current
        seq = time.time_ns()
        # Apply immediately for responsive UI, then let Engine authoritatively
        # acknowledge the explicit desired state (not a second toggle).
        self.state.set("autopilot_active", desired)
        self.state.set("autopilot_command", {"seq": seq, "enabled": desired})
        self.state.set("autopilot_command_pending", seq)
        if not desired:
            # Clear stale intents immediately; Engine also releases the device.
            self.state.set("ctl_steering", 0.0)
            self.state.set("ctl_throttle", 0.0)
            self.state.set("ctl_brake", 0.0)
        logging.info("Autopilot requested -> %s (command %s)", desired, seq)
        self._render_start_btn()

    def toggle_perf_overlay(self):
        """Show/hide the small floating performance panel."""
        try:
            if self.perf_overlay is None:
                from ui.perf_overlay import PerfOverlay
                self.perf_overlay = PerfOverlay(self.state, self)
            if self.perf_overlay.isVisible():
                self.perf_overlay.hide()
                self.perf_btn.setStyleSheet(
                    "QPushButton{background:transparent;border:1px solid " + self._pal['border'] + ";"
                    "border-radius:8px;color:" + self._pal['muted'] + ";font-size:16px;font-weight:700;}"
                    "QPushButton:hover{border-color:" + self._pal['title'] + ";color:" + self._pal['title'] + ";}")
            else:
                self.perf_overlay.show_above(self.perf_btn)
                self.perf_overlay.refresh()
                # Active state: filled accent chip.
                self.perf_btn.setStyleSheet(
                    "QPushButton{background:" + self._pal['title'] + ";color:#FFFFFF;"
                    "border:1px solid " + self._pal['title'] + ";border-radius:8px;font-size:12px;font-weight:700;}"
                    "QPushButton:hover{border-color:" + self._pal['title'] + ";}")
        except Exception as e:
            logging.warning("perf overlay toggle failed: %s", e)

    def _goto(self, index):
        self.pages.setCurrentIndex(index)
        for i, b in enumerate(getattr(self, "_nav_btns", [])):
            b.setChecked(i == index)

    def _apply_language(self, code):
        from core.i18n import t
        for button in self._nav_btns:
            key = button.property("navKey")
            label = "Visualization" if key == "visualization" else t(code, "app", key)
            button.setText(label)
        self._render_start_btn()

    def showEvent(self, event):
        """The main window is up — let the HUD process know it can appear now."""
        try:
            self.state.set("ui_ready", True)
        except Exception:
            pass
        super().showEvent(event)

    def update_ui(self):
        new_language = self.state.get("ui_language_code", "sk") or "sk"
        if new_language != getattr(self, "_language", None):
            self._language = new_language
            self._apply_language(new_language)
        # Live theme switching from the Settings page.
        new_theme = self.state.get("ui_theme", "light") or "light"
        if new_theme != getattr(self, "_theme", None):
            self._theme = new_theme
            from core.theme import stylesheet, palette
            self._pal = palette(new_theme)
            self.setStyleSheet(stylesheet(new_theme))
            # Re-render the chrome widgets that cache colours from the palette
            # (brand wordmark, hamburger, sidebar footer, start button).
            self._render_start_btn()
            if hasattr(self, "side_conn"):
                # refresh() will re-apply the right footer state colours.
                pass
            # Re-style every page that keeps its own colour cache so dark/light
            # actually applies to their cards and labels (not just the window).
            # Index-agnostic: any page exposing restyle(theme) gets refreshed.
            for idx in range(self.pages.count()):
                pg = self.pages.widget(idx)
                # Pages are now wrapped in a QScrollArea; reach the inner widget.
                if isinstance(pg, QScrollArea):
                    pg = pg.widget()
                if pg is not None and hasattr(pg, "restyle"):
                    try:
                        pg.restyle(new_theme)
                    except Exception:
                        pass
        dash = self.pages.widget(0)
        if isinstance(dash, QScrollArea):
            dash = dash.widget()
        if isinstance(dash, DashboardPage):
            dash.refresh()
        self._render_start_btn()
        # Sidebar footer: reflects telemetry connection + autopilot state.
        raw = (self.state.get("telemetry", {}) or {}).get("raw", {}) or {}
        connected = bool(raw.get("sdkActive"))
        active = bool(self.state.get("autopilot_active", False))
        if active:
            self.side_conn.setText("● Autopilot aktívny")
            self.side_conn.setStyleSheet(
                "color: " + self._pal['title'] + "; font-size: 11px; font-weight: 700; border:none; padding: 10px 18px;")
        elif connected:
            self.side_conn.setText("● Hra pripojená")
            self.side_conn.setStyleSheet(
                "color: " + self._pal['success'] + "; font-size: 11px; font-weight: 600; border:none; padding: 10px 18px;")
        else:
            self.side_conn.setText("● Čakám na hru")
            self.side_conn.setStyleSheet(
                "color: " + self._pal['muted'] + "; font-size: 11px; font-weight: 600; border:none; padding: 10px 18px;")


if __name__ == "__main__":
    # Standalone preview (no engine) — uses a plain dict as shared state.
    from core.ipc.shared_state import SharedState
    app = QApplication(sys.argv)
    window = UltraPilotApp(SharedState())
    window.show()
    sys.exit(app.exec())
