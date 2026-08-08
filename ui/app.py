import sys
import os
import logging
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QWidget, QStackedWidget, QFrame, QScrollArea, QLineEdit, QGridLayout,
)
from PyQt6.QtCore import QTimer, Qt, QSize, QRectF
from PyQt6.QtGui import (QColor, QPainter, QPainterPath, QPen,
                         QRegion)

from ui.settings_menu import SettingsMenu
from ui.map_page import MapPage
from ui.icons import line_icon

# All theming comes from core.theme.stylesheet() applied in UltraPilotApp; the
# old inline LIGHT_THEME/DARK_THEME strings here were dead code (never applied).


def window_control_notch_path(width, height):
    """Reference-shaped top-right notch for the three window controls."""
    width, height = float(width), float(height)
    path = QPainterPath()
    path.moveTo(0.0, 0.0)
    path.lineTo(width, 0.0)
    path.lineTo(width, height - 9.0)
    path.quadTo(width, height, width - 10.0, height)
    path.lineTo(15.0, height)
    path.cubicTo(7.0, height, 7.0, height - 7.0, 0.0, height - 9.0)
    path.closeSubpath()
    return path


def content_surface_path(width, height, notch_width=59.0, notch_height=24.0,
                         radius=15.0):
    """Rounded inner application surface with a top-right control cutout."""
    width, height = float(width), float(height)
    notch_width = min(float(notch_width), max(0.0, width - radius * 2.0))
    notch_x = width - notch_width
    path = QPainterPath()
    path.moveTo(radius, 0.5)
    path.lineTo(max(radius, notch_x - 10.0), 0.5)
    path.cubicTo(notch_x - 3.0, 0.5, notch_x - 3.0, 8.0,
                 notch_x + 4.0, 10.0)
    path.lineTo(notch_x + 4.0, max(10.0, notch_height - 8.0))
    path.quadTo(notch_x + 4.0, notch_height,
                notch_x + 13.0, notch_height)
    path.lineTo(width - radius, notch_height)
    path.quadTo(width - 0.5, notch_height, width - 0.5,
                notch_height + radius)
    path.lineTo(width - 0.5, height - radius)
    path.quadTo(width - 0.5, height - 0.5, width - radius, height - 0.5)
    path.lineTo(radius, height - 0.5)
    path.quadTo(0.5, height - 0.5, 0.5, height - radius)
    path.lineTo(0.5, radius)
    path.quadTo(0.5, 0.5, radius, 0.5)
    path.closeSubpath()
    return path


def rounded_window_region(width, height, radius=15.0):
    """Antialiased-looking top-level mask for the frameless window."""
    path = QPainterPath()
    path.addRoundedRect(QRectF(0.0, 0.0, max(1.0, float(width) - 1.0),
                               max(1.0, float(height) - 1.0)),
                        float(radius), float(radius))
    return QRegion(path.toFillPolygon().toPolygon())


class WindowControlDot(QPushButton):
    """Small, solid traffic-light control matching the ETS2LA chrome."""

    def __init__(self, color, glyph, name, tooltip, action, parent=None):
        super().__init__("", parent)
        self._color = QColor(color)
        self.setObjectName(name)
        self.setAccessibleName(tooltip)
        self.setToolTip(tooltip)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        # ETS2LA frontend uses three identical 11 px circles.
        self.setFixedSize(11, 11)
        self.setStyleSheet("QPushButton{background:transparent;border:none;"
                           "padding:0;margin:0;}")
        self.clicked.connect(action)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(self._color)
        painter.drawEllipse(QRectF(0.5, 0.5, 10.0, 10.0))


class ContentHost(QWidget):
    """Anchors the floating controls to the content surface on every resize."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._title_bar = None

    def attach_title_bar(self, title_bar):
        self._title_bar = title_bar
        title_bar.setParent(self)
        self.position_title_bar()

    def position_title_bar(self):
        if self._title_bar is None:
            return
        self._title_bar.move(self.width() - 8 - self._title_bar.width(), 8)
        self._title_bar.raise_()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.position_title_bar()


class ContentSurface(QFrame):
    """The one shared rounded content window used by every sidebar page."""

    def __init__(self, palette, parent=None):
        super().__init__(parent)
        self._palette = palette
        self.setObjectName("ContentSurface")
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setStyleSheet("QFrame#ContentSurface{background:transparent;border:none;}")

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(QPen(QColor(self._palette["border"]), 1.0))
        painter.setBrush(QColor(self._palette["surface"]))
        painter.drawPath(content_surface_path(self.width(), self.height()))
        painter.end()

    def set_palette(self, palette):
        self._palette = palette
        self.update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        path = content_surface_path(self.width(), self.height())
        self.setMask(QRegion(path.toFillPolygon().toPolygon()))


class MacTitleBar(QFrame):
    """Three window controls seated in the content card's curved notch."""

    def __init__(self, window, palette):
        super().__init__()
        self.window = window
        self._palette = palette
        # Reference geometry: 59x24, 11 px dots and a 4 px gap.
        self.setFixedSize(59, 24)
        self.setObjectName("MacTitleBar")
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setStyleSheet("#MacTitleBar{background:transparent;border:none;}")
        row = QHBoxLayout(self)
        row.setContentsMargins(9, 6, 9, 7)
        row.setSpacing(4)
        self.controls = {}
        for key, color, glyph, tip, action in (
                ("maximize", "#22C55E", "+", "Maximalizovať",
                 self._toggle_maximize),
                ("minimize", "#EAB308", "−", "Minimalizovať",
                 window.showMinimized),
                ("close", "#EF4444", "×", "Zavrieť", window.close)):
            dot = WindowControlDot(color, glyph, f"WindowControl-{key}",
                                   tip, action, self)
            row.addWidget(dot)
            self.controls[key] = dot

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        path = window_control_notch_path(self.width(), self.height())
        painter.setPen(QPen(QColor(self._palette["border"]), 1.0))
        painter.setBrush(QColor(self._palette["sidebar"]))
        painter.drawPath(path)
        painter.end()

    def set_palette(self, palette):
        self._palette = palette
        self.update()

    def _toggle_maximize(self):
        self.window.showNormal() if self.window.isMaximized() else self.window.showMaximized()


class WindowDragArea(QFrame):
    """Invisible top strip used to move the frameless main window."""

    def __init__(self, window):
        super().__init__(window.centralWidget())
        self.window = window
        self._offset = None
        self.setStyleSheet("background:transparent;border:none;")
        self.setCursor(Qt.CursorShape.SizeAllCursor)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            # Let Windows perform the move.  Manual QWidget.move() is unreliable
            # for frameless windows on high-DPI/multi-monitor desktops.
            handle = self.window.windowHandle()
            if handle is not None and handle.startSystemMove():
                self._offset = None
                event.accept()
                return
            self._offset = (event.globalPosition().toPoint()
                            - self.window.frameGeometry().topLeft())
            event.accept()

    def mouseMoveEvent(self, event):
        if (self._offset is not None
                and event.buttons() & Qt.MouseButton.LeftButton
                and not self.window.isMaximized()):
            self.window.move(event.globalPosition().toPoint() - self._offset)
            event.accept()

    def mouseReleaseEvent(self, event):
        self._offset = None
        event.accept()

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.window.showNormal() if self.window.isMaximized() else self.window.showMaximized()
            event.accept()

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
        from core.update_check import current_version
        self._pal = palette(state.get("ui_theme", "light") or "light")
        self.setObjectName("AboutPage")
        self.title = QLabel("O aplikácii")
        self.title.setObjectName("PageTitle")
        self.layout.addWidget(self.title)

        self.hero = QFrame()
        self.hero.setObjectName("AboutHero")
        hero_lay = QHBoxLayout(self.hero)
        hero_lay.setContentsMargins(24, 22, 24, 22)
        hero_lay.setSpacing(16)
        logo = QLabel()
        logo.setFixedSize(58, 58)
        logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        logo.setPixmap(line_icon("autopilot", "#FFFFFF", 38).pixmap(38, 38))
        logo.setStyleSheet("background:#0E9F6E;border-radius:16px;border:none;")
        hero_lay.addWidget(logo)
        hero_text = QVBoxLayout()
        hero_text.setSpacing(3)
        product = QLabel("UltraPilot")
        product.setObjectName("AboutProduct")
        version = QLabel(f"Verzia {current_version()}  •  Asistent jazdy pre ETS2")
        version.setObjectName("AboutMeta")
        summary = QLabel(
            "Navigácia podľa hernej GPS, udržiavanie v pruhu, adaptívny "
            "tempomat a bezpečnostné systémy v jednej aplikácii.")
        summary.setWordWrap(True)
        summary.setObjectName("AboutSummary")
        hero_text.addWidget(product)
        hero_text.addWidget(version)
        hero_text.addSpacing(5)
        hero_text.addWidget(summary)
        hero_lay.addLayout(hero_text, 1)
        self.layout.addWidget(self.hero)

        cards = QHBoxLayout()
        cards.setSpacing(12)
        self._about_cards = []
        for icon_name, heading, detail in (
                ("navigation", "Navigácia", "Trasa z hernej GPS a presná strednica jazdného pruhu."),
                ("autopilot", "Asistencia", "Riadenie, odstup a bezpečné zastavenie pod jednou autoritou."),
                ("visualization", "Zobrazenie", "HUD, AR a live mapa používajú rovnakú revíziu trasy.")):
            card = QFrame()
            card.setObjectName("AboutFeature")
            row = QVBoxLayout(card)
            row.setContentsMargins(17, 16, 17, 16)
            icon = QLabel()
            icon.setPixmap(line_icon(icon_name, "#0E9F6E", 24).pixmap(24, 24))
            icon.setFixedHeight(28)
            name = QLabel(heading)
            name.setObjectName("AboutFeatureTitle")
            copy = QLabel(detail)
            copy.setWordWrap(True)
            copy.setObjectName("AboutFeatureText")
            row.addWidget(icon)
            row.addWidget(name)
            row.addWidget(copy)
            row.addStretch()
            cards.addWidget(card, 1)
            self._about_cards.append(card)
        self.layout.addLayout(cards)

        self.note = QLabel(
            "UltraPilot spracúva hernú telemetriu lokálne. Ide o pomocný systém "
            "pre videohru — jazdu maj vždy pod dohľadom.")
        self.note.setWordWrap(True)
        self.note.setObjectName("AboutNotice")
        self.layout.addWidget(self.note)
        self.layout.addStretch()
        self.restyle(state.get("ui_theme", "light") or "light")

    def restyle(self, theme):
        from core.theme import palette
        self._pal = palette(theme)
        p = self._pal
        self.setStyleSheet(
            "QWidget#AboutPage{background:" + p['bg'] + ";}"
            "QLabel#PageTitle{font-size:24px;font-weight:800;color:" + p['text'] + ";}"
            "QFrame#AboutHero{background:qlineargradient(x1:0,y1:0,x2:1,y2:1,"
            "stop:0 " + p['hero_a'] + ",stop:1 " + p['hero_b'] + ");"
            "border:1px solid " + p['border'] + ";border-radius:17px;}"
            "QLabel#AboutProduct{font-size:27px;font-weight:850;color:#FFFFFF;}"
            "QLabel#AboutMeta{font-size:11px;font-weight:650;color:#A7F3D0;}"
            "QLabel#AboutSummary{font-size:13px;color:#E5E7EB;}"
            "QFrame#AboutFeature{background:" + p['card'] + ";border:1px solid "
            + p['border'] + ";border-radius:14px;}"
            "QLabel#AboutFeatureTitle{font-size:15px;font-weight:750;color:" + p['text'] + ";}"
            "QLabel#AboutFeatureText{font-size:12px;color:" + p['muted'] + ";}"
            "QLabel#AboutNotice{background:" + p['card2'] + ";border:1px solid "
            + p['border'] + ";border-radius:11px;padding:13px;color:" + p['muted'] + ";}")


class PluginsPage(Page):
    def __init__(self, state):
        super().__init__(state)
        from core.theme import palette
        self._pal = palette(state.get("ui_theme", "light") or "light")
        self.title = QLabel("Pluginy")
        self.title.setStyleSheet(
            "font-size:24px;font-weight:800;color:" + self._pal['text'] + ";")
        self.subtitle = QLabel("Správa asistenčných funkcií UltraPilotu")
        self.subtitle.setStyleSheet(
            "font-size:12px;color:" + self._pal['muted'] + ";margin-bottom:5px;")
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.subtitle)

        toolbar = QHBoxLayout()
        self.search = QLineEdit()
        self.search.setObjectName("PluginSearch")
        self.search.setPlaceholderText("Hľadať plugin…")
        self.search.setClearButtonEnabled(True)
        self.search.setMinimumHeight(38)
        self.search.textChanged.connect(self.refresh_plugins)
        toolbar.addWidget(self.search, 1)
        self.summary = QLabel("0 aktívnych")
        self.summary.setObjectName("PluginSummary")
        self.summary.setAlignment(Qt.AlignmentFlag.AlignCenter)
        toolbar.addWidget(self.summary)

        self.catalog_shell = QFrame()
        self.catalog_shell.setObjectName("PluginCatalogue")
        self.catalog_shell.setStyleSheet(
            "QFrame#PluginCatalogue{background:" + self._pal['card']
            + ";border:1px solid " + self._pal['border']
            + ";border-radius:16px;}")
        self.cards_layout = QVBoxLayout(self.catalog_shell)
        self.cards_layout.setContentsMargins(18, 17, 18, 19)
        self.cards_layout.setSpacing(14)
        self.cards_layout.addLayout(toolbar)

        self.cards_host = QWidget()
        self.cards_host.setObjectName("PluginCardsHost")
        self.cards_layout.addWidget(self.cards_host)
        self.layout.addWidget(self.catalog_shell)
        self.layout.addStretch()
        self.refresh_plugins()

    def restyle(self, theme):
        from core.theme import palette
        self._pal = palette(theme)
        self.title.setStyleSheet(
            "font-size:24px;font-weight:800;color:" + self._pal['text'] + ";")
        self.subtitle.setStyleSheet(
            "font-size:12px;color:" + self._pal['muted'] + ";margin-bottom:5px;")
        self.catalog_shell.setStyleSheet(
            "QFrame#PluginCatalogue{background:" + self._pal['card']
            + ";border:1px solid " + self._pal['border']
            + ";border-radius:16px;}")
        self.refresh_plugins()

    def refresh_plugins(self):
        from core.paths import app_dir
        plugin_dir = os.path.join(app_dir(), "plugins")
        names = []
        if os.path.isdir(plugin_dir):
            names = [f for f in sorted(os.listdir(plugin_dir))
                     if os.path.isdir(os.path.join(plugin_dir, f))
                     and os.path.exists(os.path.join(plugin_dir, f, "main.py"))]
        query = self.search.text().strip().lower()
        if query:
            names = [name for name in names if query in name.lower()
                     or query in self._DESC.get(name, "").lower()]
        enabled = [n for n in names if self.state.get(f"plugin_enabled.{n}", True)]
        disabled = [n for n in names if not self.state.get(f"plugin_enabled.{n}", True)]
        self.summary.setText(f"{len(enabled)} aktívnych  •  {len(names)} spolu")
        self.summary.setStyleSheet(
            "background:" + self._pal['card'] + ";border:1px solid "
            + self._pal['border'] + ";border-radius:9px;padding:8px 12px;"
            "font-size:11px;font-weight:650;color:" + self._pal['muted'] + ";")

        host = QWidget()
        host.setObjectName("PluginCardsHost")
        listing = QVBoxLayout(host)
        listing.setContentsMargins(0, 4, 0, 0)
        listing.setSpacing(14)
        if enabled:
            listing.addWidget(self._section("Aktívne pluginy", len(enabled)))
            grid = QGridLayout()
            grid.setHorizontalSpacing(12)
            grid.setVerticalSpacing(12)
            for index, name in enumerate(enabled):
                grid.addWidget(self._plugin_card(name, True), index // 2, index % 2)
            grid.setColumnStretch(0, 1)
            grid.setColumnStretch(1, 1)
            listing.addLayout(grid)
        if disabled:
            listing.addWidget(self._section("Dostupné pluginy", len(disabled)))
            grid = QGridLayout()
            grid.setHorizontalSpacing(12)
            grid.setVerticalSpacing(12)
            for index, name in enumerate(disabled):
                grid.addWidget(self._plugin_card(name, False), index // 2, index % 2)
            grid.setColumnStretch(0, 1)
            grid.setColumnStretch(1, 1)
            listing.addLayout(grid)
        if not names:
            empty = QLabel("Nenašli sa žiadne pluginy.")
            empty.setAlignment(Qt.AlignmentFlag.AlignCenter)
            empty.setStyleSheet("padding:30px;color:" + self._pal['muted'] + ";")
            listing.addWidget(empty)
        old_host = self.cards_host
        self.cards_layout.replaceWidget(old_host, host)
        self.cards_host = host
        old_host.setParent(None)
        old_host.deleteLater()

    def _section(self, text, count):
        lbl = QLabel(f"{text}   {count}")
        lbl.setStyleSheet("color:" + self._pal['text']
                          + ";font-size:13px;font-weight:750;margin-top:5px;")
        return lbl

    _PLUGIN_META = {
        "acc": ("Adaptívny tempomat", "Automaticky prispôsobuje rýchlosť a bezpečný odstup od vozidla pred kamiónom.", "dashboard"),
        "autopilot": ("Autopilot", "Spája navigačnú trajektóriu s riadením, plynom a brzdou počas asistovanej jazdy.", "navigation"),
        "collision": ("Ochrana pred kolíziou", "Sleduje riziká pred vozidlom a pri bezprostrednom nebezpečenstve vyžiada núdzové brzdenie.", "about"),
        "discord": ("Discord Rich Presence", "Zobrazuje aktuálny stav UltraPilotu a jazdy v používateľskom profile Discord.", "visualization"),
        "drivepolicy": ("Jazdná politika", "Vyhodnocuje dopravnú situáciu a zjednocuje bezpečnostné rozhodnutia asistenčných systémov.", "plugins"),
        "ecodrive": ("Eco Drive", "Vyhladzuje požiadavky na plyn a pomáha obmedziť zbytočnú spotrebu paliva.", "performance"),
        "hud": ("HUD", "Zobrazuje stav jazdy, navigáciu a upozornenia priamo nad herným obrazom.", "visualization"),
        "lanecontrol": ("Lane Control", "Udržiava kamión v potvrdenom jazdnom pruhu podľa autoritatívnej trajektórie.", "steering"),
        "map": ("Mapová navigácia", "Načítava mapové dáta, lokalizuje pruh a publikuje spoločný navigačný snapshot.", "navigation"),
        "toll": ("Mýto a brány", "Pomáha pri bezpečnom prejazde mýtnymi bránami a súvisiacimi cestnými obmedzeniami.", "dashboard"),
        "tts": ("Hlasové upozornenia", "Číta dôležité navigačné a bezpečnostné oznámenia bez odvádzania pozornosti od jazdy.", "about"),
        "turnsignals": ("Smerovky", "Ovláda smerovky podľa potvrdeného smeru trasy a rešpektuje aktívne výstražné svetlá.", "plugins"),
    }
    _DESC = {name: values[1] for name, values in _PLUGIN_META.items()}

    def _plugin_card(self, name, enabled):
        card = QFrame()
        card.setObjectName("PluginCard")
        card.setMinimumHeight(126)
        card.setStyleSheet(
            "QFrame#PluginCard{background:" + self._pal['card']
            + ";border:1px solid " + self._pal['border']
            + ";border-radius:12px;}QFrame#PluginCard:hover{border-color:#A7B0BA;}")
        box = QVBoxLayout(card)
        box.setContentsMargins(15, 14, 15, 14)
        box.setSpacing(8)
        head = QHBoxLayout()
        meta = self._PLUGIN_META.get(
            name, (name.replace("_", " ").title(),
                   "Doplnková funkcia UltraPilotu.", "plugins"))
        from ui.icons import line_icon
        icon = QLabel()
        icon.setObjectName("PluginIcon")
        icon.setFixedSize(34, 34)
        icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        icon.setPixmap(line_icon(meta[2], self._pal['title'], 21).pixmap(21, 21))
        icon.setStyleSheet(
            "background:" + self._pal['card2'] + ";border:1px solid "
            + self._pal['border'] + ";border-radius:9px;")
        head.addWidget(icon)
        title = QLabel(meta[0])
        title.setStyleSheet("font-size:14px;font-weight:750;color:"
                            + self._pal['text'] + ";")
        head.addWidget(title)
        head.addStretch()
        author = QLabel("matu_le33")
        author.setObjectName("PluginAuthor")
        author.setStyleSheet(
            "background:" + self._pal['card2'] + ";color:" + self._pal['muted']
            + ";border:1px solid " + self._pal['border']
            + ";border-radius:7px;padding:4px 8px;font-size:10px;font-weight:650;")
        head.addWidget(author)
        action = QPushButton("Vypnúť" if enabled else "Zapnúť")
        action.setObjectName("PluginAction")
        action.setFixedHeight(28)
        action.setCursor(Qt.CursorShape.PointingHandCursor)
        if enabled:
            action.setStyleSheet(
                "QPushButton{background:#ECFDF5;color:#047857;border:1px solid #A7F3D0;"
                "border-radius:7px;padding:4px 10px;font-size:10px;font-weight:700;}"
                "QPushButton:hover{background:#D1FAE5;}")
        else:
            action.setStyleSheet(
                "QPushButton{background:" + self._pal['card2'] + ";color:"
                + self._pal['text'] + ";border:1px solid " + self._pal['border']
                + ";border-radius:7px;padding:4px 10px;font-size:10px;font-weight:700;}"
                "QPushButton:hover{border-color:#10B981;color:#059669;}")
        head.addWidget(action)
        box.addLayout(head)
        desc = QLabel(meta[1])
        desc.setWordWrap(True)
        desc.setStyleSheet("font-size:11px;color:" + self._pal['muted'] + ";")
        box.addWidget(desc)
        box.addStretch()

        def toggle():
            new_value = not bool(self.state.get(f"plugin_enabled.{name}", True))
            try:
                from core.settings.manager import SettingsManager
                SettingsManager().set_plugin_enabled(name, new_value)
            except Exception as error:
                logging.error("Could not persist plugin '%s': %s", name, error)
                return
            self.state.set(f"plugin_enabled.{name}", new_value)
            logging.info("Toggled and saved plugin '%s' -> %s",
                         name, new_value)
            self.refresh_plugins()

        action.clicked.connect(toggle)
        return card


class DashboardPage(Page):
    def __init__(self, state):
        super().__init__(state)
        from core.theme import palette
        self._pal = palette(state.get("ui_theme", "light") or "light")
        self.title = QLabel("Prehľad")
        self.title.setStyleSheet("font-size:24px;font-weight:800;color:" + self._pal['text'] + ";")
        self.subtitle = QLabel("Aktuálny stav jazdy a asistenčných systémov")
        self.subtitle.setStyleSheet("font-size:12px;color:" + self._pal['muted'] + ";margin-bottom:6px;")
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.subtitle)

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
        for key, label in [("gear", "PREVOD"), ("rpm", "OTÁČKY"),
                           ("fuel", "PALIVO"), ("limit", "LIMIT"),
                           ("nav", "NAVIGÁCIA")]:
            col = QVBoxLayout()
            col.setSpacing(2)
            cap = QLabel(label)
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

        detail_row = QHBoxLayout()
        detail_row.setSpacing(12)
        self._dashboard_primary = []
        self._dashboard_muted = []
        self._dashboard_badges = []

        def info_card(caption):
            card = QFrame()
            card.setObjectName("Card")
            box = QVBoxLayout(card)
            box.setContentsMargins(18, 15, 18, 15)
            box.setSpacing(3)
            cap = QLabel(caption)
            cap.setStyleSheet("font-size:10px;font-weight:750;color:"
                              + self._pal['muted'] + ";")
            value = QLabel("—")
            value.setStyleSheet("font-size:16px;font-weight:750;color:"
                                + self._pal['text'] + ";")
            detail = QLabel("—")
            detail.setWordWrap(True)
            detail.setStyleSheet("font-size:11px;color:" + self._pal['muted'] + ";")
            self._dashboard_muted.extend((cap, detail))
            self._dashboard_primary.append(value)
            box.addWidget(cap)
            box.addWidget(value)
            box.addWidget(detail)
            detail_row.addWidget(card, 1)
            return value, detail

        self.route_status, self.route_detail = info_card("TRASA A MAPA")
        self.safety_status, self.safety_detail = info_card("BEZPEČNOSTNÉ SYSTÉMY")
        self.layout.addLayout(detail_row)

        systems = QFrame()
        systems.setObjectName("Card")
        systems_lay = QHBoxLayout(systems)
        systems_lay.setContentsMargins(18, 12, 18, 12)
        systems_lay.setSpacing(8)
        systems_title = QLabel("Systémy")
        systems_title.setStyleSheet("font-size:12px;font-weight:750;color:"
                                    + self._pal['text'] + ";")
        self._dashboard_primary.append(systems_title)
        systems_lay.addWidget(systems_title)
        for text in ("Lane Assist", "ACC", "Collision", "HUD + AR"):
            badge = QLabel(text)
            badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            badge.setStyleSheet(
                "background:" + self._pal['card2'] + ";border:1px solid "
                + self._pal['border'] + ";border-radius:8px;padding:5px 9px;"
                "font-size:10px;font-weight:650;color:" + self._pal['muted'] + ";")
            systems_lay.addWidget(badge)
            self._dashboard_badges.append(badge)
        systems_lay.addStretch()
        self.layout.addWidget(systems)

        self.layout.addStretch()

    def restyle(self, theme):
        """Re-apply palette colours when the theme switches (dark ↔ light)."""
        from core.theme import palette
        self._pal = palette(theme)
        self.title.setStyleSheet(
            "font-size:24px;font-weight:800;color:" + self._pal['text'] + ";")
        self.subtitle.setStyleSheet(
            "font-size:12px;color:" + self._pal['muted'] + ";margin-bottom:6px;")
        for label in getattr(self, "_dashboard_primary", []):
            label.setStyleSheet("font-size:13px;font-weight:750;color:"
                                + self._pal['text'] + ";")
        for label in getattr(self, "_dashboard_muted", []):
            label.setStyleSheet("font-size:11px;color:"
                                + self._pal['muted'] + ";")
        for badge in getattr(self, "_dashboard_badges", []):
            badge.setStyleSheet(
                "background:" + self._pal['card2'] + ";border:1px solid "
                + self._pal['border'] + ";border-radius:8px;padding:5px 9px;"
                "font-size:10px;font-weight:650;color:" + self._pal['muted'] + ";")
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

        map_name = (self.state.get("active_map_name")
                    or self.state.get("active_map_key") or "žiadna")
        revision = self.state.get("lane_trajectory_revision", 0) or 0
        if self.state.get("nav_active"):
            self.route_status.setText("GPS trasa aktívna")
            self.route_detail.setText(f"Mapa: {map_name}  •  revízia {revision}")
        else:
            self.route_status.setText("Čakám na GPS cieľ")
            self.route_detail.setText(f"Pripravená mapa: {map_name}")
        if self.state.get("telemetry_valid", False):
            self.safety_status.setText("Systémy pripravené")
            self.safety_detail.setText("Lane Assist, ACC a ochranné brzdenie sú online")
        else:
            self.safety_status.setText("Čakám na hru")
            self.safety_detail.setText("Bez telemetrie sa nevydávajú riadiace výstupy")

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
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.resize(1220, 760)
        self.setMinimumSize(980, 640)
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

        central = QFrame()
        central.setObjectName("WindowSurface")
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        content = QWidget()
        main_layout = QHBoxLayout(content)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        root_layout.addWidget(content, 1)
        # No title strip or title text: the dots float directly in the corner.
        self.drag_area = WindowDragArea(self)
        self.drag_area.setGeometry(0, 0, central.width(), 34)
        self.drag_area.raise_()
        self.sidebar = QFrame()
        self.sidebar.setObjectName("Sidebar")
        self.sidebar.setFixedWidth(232)
        sb = QVBoxLayout(self.sidebar)
        sb.setContentsMargins(10, 17, 10, 12)
        sb.setSpacing(0)

        # Brand block at the top: logo + wordmark + version.
        from PyQt6.QtGui import QPixmap, QIcon
        from core.paths import resource as _res
        brand_row = QGridLayout()
        brand_row.setContentsMargins(8, 0, 8, 0)
        brand_row.setHorizontalSpacing(0)
        logo = QLabel()
        logo.setFixedSize(34, 34)
        logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        _pm = QIcon(_res("assets", "favicon.ico")).pixmap(34, 34)
        if _pm.isNull():
            _pm = QPixmap(_res("assets", "logo.png")).scaledToWidth(
                34, Qt.TransformationMode.SmoothTransformation)
        if not _pm.isNull():
            logo.setPixmap(_pm)
        logo.setStyleSheet("border:none;")
        brand_row.addWidget(logo, 0, 0)
        word = QLabel("UltraPilot")
        self.brand_word = word
        word.setObjectName("BrandWordmark")
        word.setAlignment(Qt.AlignmentFlag.AlignCenter)
        word.setStyleSheet(
            "font-size:21px;font-weight:750;color:" + self._pal["text"]
            + ";border:none;")
        brand_row.addWidget(word, 0, 1)
        brand_balance = QWidget()
        brand_balance.setFixedSize(34, 34)
        brand_balance.setStyleSheet("border:none;background:transparent;")
        brand_row.addWidget(brand_balance, 0, 2)
        brand_row.setColumnMinimumWidth(0, 34)
        brand_row.setColumnStretch(1, 1)
        brand_row.setColumnMinimumWidth(2, 34)
        brand_w = QWidget()
        self.brand_container = brand_w
        brand_w.setLayout(brand_row)
        brand_w.setStyleSheet("border:none;")
        sb.addWidget(brand_w)
        sb.addSpacing(11)

        # Version/update is a full-width card, matching the reference layout;
        # the existing update workflow remains entirely unchanged.
        from ui.update_widget import UpdateCheckerWidget
        update_card = QFrame()
        update_card.setObjectName("SidebarUpdateCard")
        update_layout = QVBoxLayout(update_card)
        update_layout.setContentsMargins(10, 7, 10, 7)
        self.update_checker = UpdateCheckerWidget(self.state)
        update_layout.addWidget(self.update_checker)
        sb.addWidget(update_card)
        sb.addSpacing(8)

        # ETS2LA-style navigation with crisp monochrome Lucide-like line icons.
        nav = [
            ("HLAVNÉ", None, None),
            ("dashboard", "Dashboard", 0),
            ("navigation", "Navigation", 1),
            ("visualization", "Visualization", 2),
            ("ROZŠÍRENIA", None, None),
            ("plugins", "Manager", 3),
            ("POMOC", None, None),
            ("about", "About", 5),
            ("__stretch__", None, None),
            ("settings", "Settings", 4),
        ]
        self._nav_btns = []
        for icon_text, text, idx in nav:
            if idx is None:
                if icon_text == "__stretch__":
                    sb.addStretch()
                    continue
                section = QLabel(icon_text)
                section.setObjectName("NavSection")
                sb.addWidget(section)
                continue
            b = QPushButton(text)
            b.setIcon(line_icon(icon_text))
            b.setIconSize(QSize(20, 20))
            b.setObjectName("NavButton")
            b.setFixedHeight(38)
            b.setProperty("navIndex", idx)
            b.setProperty("navKey", "plugins" if text == "Manager" else text.lower())
            b.setCheckable(True)
            b.clicked.connect(lambda _=False, i=idx: self._goto(i))
            sb.addWidget(b)
            self._nav_btns.append(b)
        self._nav_btns[0].setChecked(True)
        # Sidebar footer: connection and performance controls share one compact
        # card instead of floating as unrelated text/buttons.
        footer_card = QFrame()
        footer_card.setObjectName("SidebarStatusCard")
        footer_layout = QVBoxLayout(footer_card)
        footer_layout.setContentsMargins(9, 7, 9, 8)
        footer_layout.setSpacing(4)
        self.side_conn = QLabel("●  Čakám na hru")
        self.side_conn.setObjectName("SidebarConnection")
        self.side_conn.setProperty("connectionState", "waiting")
        footer_layout.addWidget(self.side_conn)

        # Hamburger button: toggles the small floating performance overlay.
        self.perf_overlay = None
        self.perf_btn = QPushButton("Performance")
        self.perf_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.perf_btn.setFixedHeight(32)
        self.perf_btn.setToolTip("Performance")
        self.perf_btn.setObjectName("SidebarPerformance")
        self.perf_btn.setIcon(line_icon("performance"))
        self.perf_btn.setIconSize(QSize(18, 18))
        self.perf_btn.setText("Výkon aplikácie")
        # Black/white style toggle kept minimal — colour flips with state below.
        self.perf_btn.clicked.connect(self.toggle_perf_overlay)
        footer_layout.addWidget(self.perf_btn)
        sb.addWidget(footer_card)
        main_layout.addWidget(self.sidebar)

        # Every sidebar destination is rendered inside the same rounded inner
        # window.  Its top-right path leaves a real shaped seat for the dots.
        content_host = ContentHost()
        self.content_host = content_host
        content_host.setObjectName("ContentHost")
        content_host.setStyleSheet("QWidget#ContentHost{background:transparent;border:none;}")
        content_host_layout = QVBoxLayout(content_host)
        content_host_layout.setContentsMargins(8, 8, 8, 8)
        content_host_layout.setSpacing(0)
        self.content_surface = ContentSurface(self._pal)
        surface_layout = QVBoxLayout(self.content_surface)
        surface_layout.setContentsMargins(1, 1, 1, 1)
        surface_layout.setSpacing(0)
        content_host_layout.addWidget(self.content_surface)
        main_layout.addWidget(content_host, 1)

        self.title_bar = MacTitleBar(self, self._pal)
        self.content_host.attach_title_bar(self.title_bar)

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
        surface_layout.addWidget(self.pages)

        self.start_btn = QPushButton("ZAPNÚŤ AUTOPILOT")
        self.start_btn.setObjectName("SidebarAutopilot")
        self.start_btn.setFixedHeight(48)
        self.start_btn.setIcon(line_icon("autopilot", "#FFFFFF"))
        self.start_btn.setIconSize(QSize(20, 20))
        self.start_btn.clicked.connect(self.toggle_autopilot)
        # The old QMainWindow status bar painted the unexplained full-width
        # white rectangle at the bottom. Keep the action inside the sidebar.
        sb.insertSpacing(max(0, sb.count() - 1), 8)
        sb.insertWidget(max(0, sb.count() - 1), self.start_btn)
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
            self.start_btn.setText(t(lang, "app", "disable_ap").capitalize())
        else:
            self.start_btn.setText(t(lang, "app", "enable_ap").capitalize())
        if self.start_btn.property("active") != bool(active):
            self.start_btn.setProperty("active", bool(active))
            self.start_btn.style().unpolish(self.start_btn)
            self.start_btn.style().polish(self.start_btn)

    def toggle_autopilot(self):
        import time
        current = bool(self.state.get("autopilot_active", False))
        desired = not current
        seq = time.time_ns()
        # Engine owns the master state. Publishing it directly here allowed the
        # worker plugin to observe an unvalidated enable before the command was
        # acknowledged and before an engagement request id existed.
        self.state.set("autopilot_command", {"seq": seq, "enabled": desired})
        self.state.set("autopilot_command_pending", seq)
        if not desired:
            # Clear stale intents immediately; Engine also releases the device.
            self.state.set("ctl_steering", 0.0)
            self.state.set("ctl_throttle", 0.0)
            self.state.set("ctl_brake", 0.0)
        logging.info("Autopilot requested -> %s (command %s)", desired, seq)

    def toggle_perf_overlay(self):
        """Show/hide the small floating performance panel."""
        try:
            if self.perf_overlay is None:
                from ui.perf_overlay import PerfOverlay
                self.perf_overlay = PerfOverlay(self.state, self)
            if self.perf_overlay.isVisible():
                self.perf_overlay.hide()
                self.perf_btn.setProperty("active", False)
            else:
                self.perf_overlay.show_above(self.perf_btn)
                self.perf_overlay.refresh()
                self.perf_btn.setProperty("active", True)
            self.perf_btn.style().unpolish(self.perf_btn)
            self.perf_btn.style().polish(self.perf_btn)
        except Exception as e:
            logging.warning("perf overlay toggle failed: %s", e)

    def _goto(self, index):
        self.pages.setCurrentIndex(index)
        for button in getattr(self, "_nav_btns", []):
            button.setChecked(int(button.property("navIndex")) == index)

    def _apply_language(self, code):
        from core.i18n import t
        for button in self._nav_btns:
            key = button.property("navKey")
            label = "Visualization" if key == "visualization" else t(code, "app", key)
            button.setText(label)
        self._render_start_btn()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.isMaximized():
            self.clearMask()
        else:
            self.setMask(rounded_window_region(self.width(), self.height()))
        if hasattr(self, "drag_area"):
            self.drag_area.setGeometry(0, 0, self.centralWidget().width(), 34)
            self.drag_area.raise_()
        if hasattr(self, "title_bar"):
            self.content_host.position_title_bar()

    def showEvent(self, event):
        """The main window is up — let the HUD process know it can appear now."""
        self._set_native_windows_icon()
        self._apply_native_rounded_corners()
        if not self.isMaximized():
            self.setMask(rounded_window_region(self.width(), self.height()))
        try:
            self.state.set("ui_ready", True)
        except Exception:
            pass
        super().showEvent(event)

    def _apply_native_rounded_corners(self):
        """Ask Windows 11 DWM for the native rounded-corner treatment."""
        if os.name != "nt":
            return
        try:
            import ctypes
            preference = ctypes.c_int(2)  # DWMWCP_ROUND
            ctypes.windll.dwmapi.DwmSetWindowAttribute(
                int(self.winId()), 33, ctypes.byref(preference),
                ctypes.sizeof(preference))
        except Exception:
            pass

    def _set_native_windows_icon(self):
        """Set WM/class icons as well as QIcon for the Windows taskbar.

        The app is launched through ``py.exe`` by the installer. Some Windows
        builds then ignore Qt's icon and retain the generic Python icon unless
        the native HWND/class icons are explicitly replaced.
        """
        if os.name != "nt":
            return
        try:
            import ctypes
            from ctypes import wintypes
            from core.paths import resource
            path = resource("assets", "favicon.ico")
            if not path or not os.path.exists(path):
                return
            user32 = ctypes.windll.user32
            load = user32.LoadImageW
            load.argtypes = [wintypes.HINSTANCE, wintypes.LPCWSTR,
                             wintypes.UINT, ctypes.c_int, ctypes.c_int,
                             wintypes.UINT]
            load.restype = ctypes.c_void_p
            send = user32.SendMessageW
            send.argtypes = [wintypes.HWND, wintypes.UINT,
                             ctypes.c_size_t, ctypes.c_void_p]
            set_class_icon = user32.SetClassLongPtrW
            set_class_icon.argtypes = [wintypes.HWND, ctypes.c_int,
                                       ctypes.c_void_p]
            set_class_icon.restype = ctypes.c_void_p
            IMAGE_ICON, LR_LOADFROMFILE = 1, 0x0010
            big = load(None, path, IMAGE_ICON, 32, 32, LR_LOADFROMFILE)
            small = load(None, path, IMAGE_ICON, 16, 16, LR_LOADFROMFILE)
            hwnd = int(self.winId())
            WM_SETICON = 0x0080
            if big:
                send(hwnd, WM_SETICON, 1, big)
                set_class_icon(hwnd, -14, big)  # GCLP_HICON
                self._native_big_icon = big
            if small:
                send(hwnd, WM_SETICON, 0, small)
                set_class_icon(hwnd, -34, small)  # GCLP_HICONSM
                self._native_small_icon = small
        except Exception as exc:
            logging.debug("native taskbar icon could not be set: %s", exc)

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
            self.title_bar.set_palette(self._pal)
            self.content_surface.set_palette(self._pal)
            self.brand_word.setStyleSheet(
                "font-size:21px;font-weight:750;color:"
                + self._pal["text"] + ";border:none;")
            self.update_checker.restyle(new_theme)
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
            connection_state = "autopilot"
        elif connected:
            self.side_conn.setText("● Hra pripojená")
            connection_state = "connected"
        else:
            self.side_conn.setText("● Čakám na hru")
            connection_state = "waiting"
        if self.side_conn.property("connectionState") != connection_state:
            self.side_conn.setProperty("connectionState", connection_state)
            self.side_conn.style().unpolish(self.side_conn)
            self.side_conn.style().polish(self.side_conn)


if __name__ == "__main__":
    # Standalone preview (no engine) — uses a plain dict as shared state.
    from core.ipc.shared_state import SharedState
    app = QApplication(sys.argv)
    window = UltraPilotApp(SharedState())
    window.show()
    sys.exit(app.exec())
