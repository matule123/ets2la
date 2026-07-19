"""
UltraPilot first-run onboarding wizard.

Shown in place of the main window the first time UltraPilot starts (when the
``onboarded`` flag in settings is missing / false). Five quick steps:

  0. Welcome
  1. Language   — Slovak & English are bundled; others can be downloaded.
  2. Game & SDK — auto-detect installed ETS2/ATS, install the SDK DLLs.
  3. Map pack   — pick a dataset matching the detected game, download it.
  4. Done       — summary + launch.

After completion the wizard writes ``onboarded = true`` (plus ``ui_language``
and ``selected_map``) to settings.json and emits ``finished`` so the caller can
open the main window. The wizard can also be re-run from the Settings page.
"""

import logging
import os

from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer, QPropertyAnimation, QEasingCurve
from PyQt6.QtGui import QIcon, QPixmap
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QStackedWidget, QFrame, QProgressBar, QButtonGroup, QRadioButton,
    QScrollArea, QMessageBox, QGraphicsOpacityEffect,
)

from core import i18n
from core.paths import resource
from core.theme import stylesheet


# ----------------------------------------------------------------- helpers
ACCENT = "#10B981"
ACCENT_HI = "#34D399"
SUCCESS = "#22C55E"
DANGER = "#EF4444"
WARN = "#F59E0B"


def _icon_path():
    try:
        return resource("assets", "favicon.ico")
    except Exception:
        return ""


def _btn_primary(text):
    b = QPushButton(text)
    b.setObjectName("Primary")
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    b.setStyleSheet(
        "QPushButton#Primary{background: qlineargradient(x1:0,y1:0,x2:0,y2:1,"
        "stop:0 " + ACCENT_HI + ", stop:1 #059669); color:#fff; border:none;"
        "border-radius:10px; padding:11px 24px; font-size:14px; font-weight:700;}"
        "QPushButton#Primary:hover{background: qlineargradient(x1:0,y1:0,x2:0,y2:1,"
        "stop:0 #3DEDA6, stop:1 #06A977);}"
        "QPushButton#Primary:disabled{background:#E5E7EB; color:#9CA3AF;}")
    return b


def _btn_ghost(text):
    b = QPushButton(text)
    b.setObjectName("Ghost")
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    b.setStyleSheet(
        "QPushButton#Ghost{background:transparent; color:#111827;"
        "border:1px solid #E5E7EB; border-radius:10px; padding:11px 18px;"
        "font-size:14px; font-weight:600;}"
        "QPushButton#Ghost:hover{border-color:" + ACCENT + "; color:" + ACCENT + ";}"
        "QPushButton:disabled{color:#9CA3AF; border-color:#E5E7EB;}")
    return b


# ----------------------------------------------------------------- workers
class _LangDownloadWorker(QThread):
    done = pyqtSignal(bool, str)   # (ok, code)
    def __init__(self, code):
        super().__init__()
        self.code = code
    def run(self):
        try:
            ok = i18n.install_from_github(self.code)
            self.done.emit(bool(ok), self.code)
        except Exception as e:
            logging.error("onboarding lang dl: %s", e)
            self.done.emit(False, self.code)


class _MapDownloadWorker(QThread):
    progress = pyqtSignal(float, str)
    done = pyqtSignal(bool, str)   # (ok, key)
    def __init__(self, key):
        super().__init__()
        self.key = key
    def run(self):
        try:
            from core.navigation import map_data
            ok = map_data.download(self.key, progress_cb=lambda f, t: self.progress.emit(f, t))
            self.done.emit(bool(ok), self.key)
        except Exception as e:
            logging.error("onboarding map dl: %s", e)
            self.done.emit(False, self.key)


class _SDKInstallWorker(QThread):
    done = pyqtSignal(bool, str, str)   # (ok, game_path, message)
    def __init__(self, game_path, version):
        super().__init__()
        self.game_path = game_path
        self.version = version
    def run(self):
        try:
            from core.sdk import sdk_downloader
            ok, msg = sdk_downloader.ensure_installed(self.game_path, self.version)
            self.done.emit(ok, self.game_path, msg)
        except Exception as e:
            logging.error("onboarding sdk: %s", e)
            self.done.emit(False, self.game_path, "failed:" + str(e))


class _SDKMaintenanceWorker(QThread):
    done = pyqtSignal(bool, str, str, str)
    def __init__(self, game_path, version, action):
        super().__init__()
        self.game_path, self.version, self.action = game_path, version, action
    def run(self):
        try:
            from core.sdk import sdk_downloader
            if self.action == "repair":
                ok, msg = sdk_downloader.repair(self.game_path, self.version)
            else:
                ok, msg = sdk_downloader.uninstall(self.game_path)
            self.done.emit(ok, self.game_path, msg, self.action)
        except Exception as e:
            logging.error("onboarding sdk %s: %s", self.action, e)
            self.done.emit(False, self.game_path, "failed:" + str(e), self.action)


# ----------------------------------------------------------------- card widgets
class _LangRow(QWidget):
    """One selectable language row in the language step."""
    def __init__(self, info, parent=None):
        super().__init__(parent)
        self.code = info["code"]
        self.info = info
        self._build()
        self._refresh(info)

    def _build(self):
        self.setObjectName("Card")
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(10)
        left = QVBoxLayout()
        left.setSpacing(2)
        self.name = QLabel()
        self.name.setStyleSheet("font-size:14px; font-weight:700; color:#111827;")
        self.sub = QLabel()
        self.sub.setStyleSheet("font-size:12px; color:#6B7280;")
        left.addWidget(self.name)
        left.addWidget(self.sub)
        lay.addLayout(left)
        lay.addStretch()
        self.cov = QLabel()
        self.cov.setStyleSheet("font-size:12px; color:#6B7280;")
        lay.addWidget(self.cov)
        self.action = QPushButton()
        self.action.setCursor(Qt.CursorShape.PointingHandCursor)
        self.action.setStyleSheet(
            "QPushButton{background:#F3F4F6; color:#111827; border:1px solid #E5E7EB;"
            "border-radius:8px; padding:6px 14px; font-size:12px; font-weight:600;}"
            "QPushButton:hover{border-color:" + ACCENT + "; color:" + ACCENT + ";}")
        self.action.clicked.connect(self._on_action)
        lay.addWidget(self.action)

    def _refresh(self, info):
        self.info = info
        self.name.setText(info["name"] + ("  ·  " + info["english_name"] if info["english_name"] != info["name"] else ""))
        cov = info.get("coverage", 0)
        self.cov.setText(str(cov) + "%")
        self.cov.setStyleSheet("font-size:12px; font-weight:600; color:" + (SUCCESS if cov >= 80 else (WARN if cov >= 40 else DANGER)))
        if info.get("bundled"):
            self.sub.setText(_("lang_bundled"))
            self.action.setText(_("lang_select"))
            self.action.setEnabled(True)
        elif info.get("downloaded"):
            self.sub.setText(_("lang_downloaded"))
            self.action.setText(_("lang_select"))
            self.action.setEnabled(True)
        else:
            self.action.setText(_("lang_download"))
            self.action.setEnabled(True)

    def mark_selected(self, selected):
        if selected:
            self.setStyleSheet("#Card{background:#ECFDF5; border:2px solid " + ACCENT + "; border-radius:12px;}")
            self.action.setText(_("lang_selected"))
            self.action.setStyleSheet("QPushButton{background:#10B981;color:#FFFFFF;border:none;border-radius:8px;padding:6px 14px;font-size:12px;font-weight:700;}")
        else:
            self.setStyleSheet("#Card{background:#FFFFFF; border:1px solid #E5E7EB; border-radius:12px;}")
            if self.info.get("downloaded") or self.info.get("bundled"):
                self.action.setText(_("lang_select"))
            self.action.setStyleSheet(
                "QPushButton{background:#F3F4F6;color:#111827;border:1px solid #E5E7EB;"
                "border-radius:8px;padding:6px 14px;font-size:12px;font-weight:600;}"
                "QPushButton:hover{border-color:" + ACCENT + ";color:" + ACCENT + ";}")

    def _on_action(self):
        win = self.window()
        if isinstance(win, OnboardingWizard):
            if self.info.get("downloaded") or self.info.get("bundled"):
                win.select_language(self.code)
            else:
                win.download_language(self.code, self)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._on_action()
            event.accept()
            return
        super().mousePressEvent(event)


# ----------------------------------------------------------------- translation shortcut
_tr = {}  # filled by wizard on language change; module-level helper used by widgets
_lang_code = "sk"


def _(key, **fmt):
    return i18n.t(_lang_code, "onboarding", key, **fmt) or i18n.t(_lang_code, "common", key, **fmt)


# ----------------------------------------------------------------- the wizard
class OnboardingWizard(QWidget):
    """Multi-step first-run setup window."""

    finished = pyqtSignal()   # emit when user clicks „Launch UltraPilot“

    def __init__(self, state):
        super().__init__()
        self.state = state
        global _lang_code
        _lang_code = (state.get("ui_language_code") or "sk")
        self.setObjectName("Onboarding")
        self.setWindowTitle(_("win"))
        self.resize(720, 600)
        self.setMinimumSize(680, 560)
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("UltraPilot.Onboarding")
        except Exception:
            pass
        ico = _icon_path()
        if ico and os.path.exists(ico):
            self.setWindowIcon(QIcon(ico))

        # Selected values.
        self.selected_lang_code = _lang_code
        self.selected_map = state.get("selected_map") or ""
        self._lang_workers = []
        self._map_worker = None
        self._sdk_workers = []
        self._sdk_status = {}    # game_path -> "installed"/"unsupported"/"failed"
        self._games = []         # [{path, version, kind}]

        self._build()
        self.setStyleSheet(stylesheet("light"))
        self._go_step(0)

    # --------------------------------------------------------------- chrome
    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_hero())
        root.addWidget(self._build_step_rail())
        self.stack = QStackedWidget()
        self._build_welcome()
        self._build_language()
        self._build_sdk()
        self._build_map()
        self._build_done()
        root.addWidget(self.stack, stretch=1)
        root.addWidget(self._build_footer())

    def _build_hero(self):
        hero = QFrame()
        hero.setFixedHeight(64)
        hero.setStyleSheet("background:#FFFFFF; border-bottom:1px solid #E5E7EB;")
        h = QHBoxLayout(hero)
        h.setContentsMargins(24, 12, 24, 12)
        logo = QLabel()
        ico = _icon_path()
        pm = QIcon(ico).pixmap(34, 34) if ico and os.path.exists(ico) else QPixmap()
        if not pm.isNull():
            logo.setPixmap(pm)
        logo.setStyleSheet("border:none;")
        h.addWidget(logo)
        brand = QLabel("UltraPilot")
        brand.setStyleSheet("font-size:18px; font-weight:800; color:#065F46;")
        h.addWidget(brand)
        h.addStretch()
        return hero

    def _build_step_rail(self):
        rail = QFrame()
        rail.setFixedHeight(46)
        rail.setStyleSheet("background:#FFFFFF; border-bottom:1px solid #F3F4F6;")
        h = QHBoxLayout(rail)
        h.setContentsMargins(24, 8, 24, 8)
        h.setSpacing(8)
        self._step_labels = []
        names = [_("step_welcome"), _("step_language"), _("step_sdk"), _("step_map"), _("step_done")]
        for i, name in enumerate(names):
            badge = QLabel(str(i + 1))
            badge.setFixedSize(22, 22)
            badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            badge.setStyleSheet("background:#F3F4F6; color:#6B7280; border-radius:11px; font-weight:700; font-size:11px;")
            lbl = QLabel(name)
            lbl.setStyleSheet("font-size:12px; color:#6B7280; font-weight:600;")
            cell = QHBoxLayout()
            cell.setSpacing(6)
            cell.addWidget(badge)
            cell.addWidget(lbl)
            wrap = QWidget()
            wrap.setLayout(cell)
            wrap.setStyleSheet("border:none;")
            h.addWidget(wrap)
            self._step_labels.append((badge, lbl, wrap))
            if i < 4:
                sep = QLabel("·")
                sep.setStyleSheet("color:#D1D5DB; border:none;")
                h.addWidget(sep)
        h.addStretch()
        return rail

    def _build_footer(self):
        foot = QFrame()
        foot.setFixedHeight(64)
        foot.setStyleSheet("background:#FFFFFF; border-top:1px solid #E5E7EB;")
        fh = QHBoxLayout(foot)
        fh.setContentsMargins(24, 12, 24, 12)
        fh.setSpacing(10)
        self.back_btn = _btn_ghost(_("back"))
        self.back_btn.clicked.connect(self._back)
        self.next_btn = _btn_primary(_("start"))
        self.next_btn.clicked.connect(self._next)
        fh.addStretch()
        fh.addWidget(self.back_btn)
        fh.addWidget(self.next_btn)
        return foot

    def _fade_in(self, w):
        try:
            eff = QGraphicsOpacityEffect(w)
            w.setGraphicsEffect(eff)
            a = QPropertyAnimation(eff, b"opacity", w)
            a.setDuration(150)
            a.setStartValue(0.0)
            a.setEndValue(1.0)
            a.setEasingCurve(QEasingCurve.Type.OutCubic)
            a.start(QPropertyAnimation.DeletionPolicy.DeleteWhenStopped)
        except Exception:
            pass

    # --------------------------------------------------------------- pages
    def _page(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(36, 22, 36, 22)
        lay.setSpacing(12)
        scroll.setWidget(inner)
        return scroll, lay

    def _build_welcome(self):
        scroll, lay = self._page()
        title = QLabel(_("welcome_t"))
        title.setStyleSheet("font-size:30px; font-weight:800; color:#111827; letter-spacing:-0.5px;")
        lay.addWidget(title)
        desc = QLabel(_("welcome_d"))
        desc.setWordWrap(True)
        desc.setStyleSheet("font-size:15px; color:#374151;")
        lay.addWidget(desc)
        lay.addStretch()
        self.stack.addWidget(scroll)

    def _build_language(self):
        scroll, lay = self._page()
        self.lang_title = QLabel(_("lang_t"))
        self.lang_title.setStyleSheet("font-size:30px; font-weight:800; color:#111827; letter-spacing:-0.5px;")
        lay.addWidget(self.lang_title)
        self.lang_desc = QLabel(_("lang_d"))
        self.lang_desc.setWordWrap(True)
        self.lang_desc.setStyleSheet("font-size:14px; color:#374151;")
        lay.addWidget(self.lang_desc)
        lay.addSpacing(6)
        self.lang_rows_wrap = QVBoxLayout()
        self.lang_rows_wrap.setSpacing(8)
        lay.addLayout(self.lang_rows_wrap)
        self.lang_rows = []
        self._populate_languages()
        lay.addStretch()
        self.stack.addWidget(scroll)

    def _populate_languages(self):
        # Clear old rows.
        for row in self.lang_rows:
            row.setParent(None)
            row.deleteLater()
        self.lang_rows = []
        langs = i18n.available()
        # Bundled first, then by coverage desc.
        langs.sort(key=lambda l: (not l["bundled"], -l["coverage"]))
        for info in langs:
            row = _LangRow(info)
            row.mark_selected(info["code"] == self.selected_lang_code)
            self.lang_rows_wrap.addWidget(row)
            self.lang_rows.append(row)

    def _build_sdk(self):
        scroll, lay = self._page()
        self.sdk_title = QLabel(_("sdk_t"))
        self.sdk_title.setStyleSheet("font-size:30px; font-weight:800; color:#111827; letter-spacing:-0.5px;")
        lay.addWidget(self.sdk_title)
        self.sdk_desc = QLabel(_("sdk_d"))
        self.sdk_desc.setWordWrap(True)
        self.sdk_desc.setStyleSheet("font-size:14px; color:#374151;")
        lay.addWidget(self.sdk_desc)
        lay.addSpacing(6)
        self.sdk_status_label = QLabel(_("sdk_detecting"))
        self.sdk_status_label.setStyleSheet("font-size:13px; color:#6B7280;")
        lay.addWidget(self.sdk_status_label)
        self.sdk_games_wrap = QVBoxLayout()
        self.sdk_games_wrap.setSpacing(8)
        lay.addLayout(self.sdk_games_wrap)
        lay.addStretch()
        self.stack.addWidget(scroll)
        # Defer detection until the page is shown.

    def _build_map(self):
        scroll, lay = self._page()
        self.map_title = QLabel(_("map_t"))
        self.map_title.setStyleSheet("font-size:30px; font-weight:800; color:#111827; letter-spacing:-0.5px;")
        lay.addWidget(self.map_title)
        self.map_desc = QLabel(_("map_d"))
        self.map_desc.setWordWrap(True)
        self.map_desc.setStyleSheet("font-size:14px; color:#374151;")
        lay.addWidget(self.map_desc)
        lay.addSpacing(6)
        self.map_status_label = QLabel(_("map_detecting"))
        self.map_status_label.setStyleSheet("font-size:13px; color:#6B7280;")
        lay.addWidget(self.map_status_label)
        self.map_list_wrap = QVBoxLayout()
        self.map_list_wrap.setSpacing(8)
        lay.addLayout(self.map_list_wrap)
        lay.addStretch()
        self.stack.addWidget(scroll)

    def _build_done(self):
        scroll, lay = self._page()
        lay.addStretch()
        self.done_icon = QLabel("✔")
        self.done_icon.setStyleSheet("font-size:64px; color:" + SUCCESS + ";")
        self.done_icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self.done_icon)
        self.done_title = QLabel(_("done_t"))
        self.done_title.setStyleSheet("font-size:30px; font-weight:800; color:#111827;")
        self.done_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self.done_title)
        self.done_desc = QLabel(_("done_d"))
        self.done_desc.setStyleSheet("font-size:14px; color:#374151;")
        self.done_desc.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self.done_desc)
        lay.addSpacing(8)
        self.done_summary = QLabel("")
        self.done_summary.setStyleSheet("font-size:13px; color:#6B7280;")
        self.done_summary.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.done_summary.setWordWrap(True)
        lay.addWidget(self.done_summary)
        lay.addStretch()
        self.stack.addWidget(scroll)

    # --------------------------------------------------------------- navigation
    def _go_step(self, idx):
        idx = max(0, min(idx, self.stack.count() - 1))
        self.stack.setCurrentIndex(idx)
        self._cur = idx
        for i, (badge, lbl, wrap) in enumerate(self._step_labels):
            active = (i == idx)
            done = (i < idx)
            if active:
                badge.setStyleSheet("background:" + ACCENT + "; color:#fff; border-radius:11px; font-weight:700; font-size:11px;")
                lbl.setStyleSheet("font-size:12px; color:" + ACCENT + "; font-weight:700;")
            elif done:
                badge.setText("✓")
                badge.setStyleSheet("background:" + SUCCESS + "; color:#fff; border-radius:11px; font-weight:700; font-size:11px;")
                lbl.setStyleSheet("font-size:12px; color:#6B7280; font-weight:600;")
            else:
                badge.setText(str(i + 1))
                badge.setStyleSheet("background:#F3F4F6; color:#6B7280; border-radius:11px; font-weight:700; font-size:11px;")
                lbl.setStyleSheet("font-size:12px; color:#6B7280; font-weight:600;")
        self._fade_in(self.stack.currentWidget())
        self._update_nav()
        # Trigger page-specific work.
        if idx == 2:
            QTimer.singleShot(50, self.detect_games)
        elif idx == 3:
            QTimer.singleShot(50, self.load_maps)
        elif idx == 4:
            self._build_done_summary()

    def _update_nav(self):
        from core.i18n import t as _t
        i = self._cur
        self.back_btn.setVisible(i > 0)
        self.back_btn.setText(_t(_lang_code, "common", "back"))
        if i == 0:
            self.next_btn.setText(_("start"))
            self.next_btn.setEnabled(True)
        elif i == 1:
            self.next_btn.setText(_t(_lang_code, "common", "next"))
            self.next_btn.setEnabled(True)
        elif i == 2:
            self.next_btn.setText(_t(_lang_code, "common", "next"))
            self.next_btn.setEnabled(True)
        elif i == 3:
            self.next_btn.setText(_t(_lang_code, "common", "next"))
            self.next_btn.setEnabled(True)
        elif i == 4:
            self.next_btn.setText(_("launch"))
            self.next_btn.setEnabled(True)

    def _next(self):
        if self._cur < self.stack.count() - 1:
            self._go_step(self._cur + 1)
        else:
            self._finalize()

    def _back(self):
        if self._cur > 0:
            self._go_step(self._cur - 1)

    # --------------------------------------------------------------- language step
    def select_language(self, code):
        global _lang_code
        self.selected_lang_code = code
        _lang_code = code
        self.state.set("ui_language_code", code)
        # Refresh row highlight.
        for row in self.lang_rows:
            row.mark_selected(row.code == code)
        # Re-translate visible chrome.
        self._retranslate()

    def _retranslate(self):
        # Rebuild step labels + page titles with the new language.
        names = [_("step_welcome"), _("step_language"), _("step_sdk"), _("step_map"), _("step_done")]
        for (badge, lbl, wrap), name in zip(self._step_labels, names):
            lbl.setText(name)
        self.setWindowTitle(_("win"))
        # Welcome page.
        w_scroll = self.stack.widget(0)
        lays = w_scroll.widget().layout()
        if lays.count() >= 2:
            lays.itemAt(0).widget().setText(_("welcome_t"))
            lays.itemAt(1).widget().setText(_("welcome_d"))
        # Language page.
        self.lang_title.setText(_("lang_t"))
        self.lang_desc.setText(_("lang_d"))
        self._populate_languages()
        # SDK page.
        self.sdk_title.setText(_("sdk_t"))
        self.sdk_desc.setText(_("sdk_d"))
        # Map page.
        self.map_title.setText(_("map_t"))
        self.map_desc.setText(_("map_d"))
        # Done page.
        self.done_title.setText(_("done_t"))
        self.done_desc.setText(_("done_d"))
        self._update_nav()

    def download_language(self, code, row):
        if not row:
            return
        row.action.setEnabled(False)
        row.action.setText(_("lang_downloading"))
        w = _LangDownloadWorker(code)
        w.done.connect(lambda ok, c: self._on_lang_downloaded(ok, c, row))
        self._lang_workers.append(w)
        w.start()

    def _on_lang_downloaded(self, ok, code, row):
        if ok:
            i18n.reload()
            info = next((l for l in i18n.available() if l["code"] == code), None)
            if info:
                row._refresh(info)
        else:
            row.action.setEnabled(True)
            row.action.setText(_("lang_failed"))

    # --------------------------------------------------------------- SDK step
    def detect_games(self):
        # Run detection in a worker thread so the UI stays responsive.
        class _Detect(QThread):
            done = pyqtSignal(list)
            def run(self):
                try:
                    from core.sdk import game_utils
                    paths = game_utils.find_scs_games()
                    out = []
                    for p in paths:
                        kind = "ats" if "American Truck Simulator" in p else "ets2"
                        ver = game_utils.get_version_for_game(p)
                        out.append({"path": p, "version": ver, "kind": kind})
                    self.done.emit(out)
                except Exception as e:
                    logging.error("onboarding detect: %s", e)
                    self.done.emit([])
        self._detect_worker = _Detect()
        self._detect_worker.done.connect(self._on_games_detected)
        self._detect_worker.start()

    def _on_games_detected(self, games):
        self._games = games
        # Clear old game cards.
        for i in reversed(range(self.sdk_games_wrap.count())):
            w = self.sdk_games_wrap.itemAt(i).widget()
            if w:
                w.setParent(None)
                w.deleteLater()
        if not games:
            self.sdk_status_label.setText(_("sdk_none"))
            return
        self.sdk_status_label.setText("")
        from core.sdk import sdk_downloader
        for g in games:
            card = self._build_game_card(g, sdk_downloader)
            self.sdk_games_wrap.addWidget(card)

    def _build_game_card(self, game, sdk_downloader):
        card = QFrame()
        card.setStyleSheet("QFrame{background:#FFFFFF; border:1px solid #E5E7EB; border-radius:12px;} "
                           "QLabel{background:transparent; border:none;}")
        lay = QHBoxLayout(card)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(10)
        left = QVBoxLayout()
        left.setSpacing(2)
        kind_name = _("sdk_game_ats") if game["kind"] == "ats" else _("sdk_game_ets2")
        name = QLabel(kind_name + "  ·  " + _("sdk_version", ver=game["version"]))
        name.setStyleSheet("font-size:14px; font-weight:700; color:#111827; border:none;")
        left.addWidget(name)
        status_key = "path:" + game["path"]
        installed = sdk_downloader.is_sdk_installed(game["path"])
        self._sdk_status[status_key] = "installed" if installed else ("unsupported" if not sdk_downloader.is_supported(game["version"]) else "missing")
        self._refresh_game_status(card, game, status_key)
        status_lbl = QLabel()
        status_lbl.setObjectName("status")
        left.addWidget(status_lbl)
        self._set_game_status_label(status_lbl, status_key)
        lay.addLayout(left)
        lay.addStretch()
        actions = QHBoxLayout()
        actions.setSpacing(7)
        btn = QPushButton()
        btn.setObjectName("action")
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._set_game_button(btn, status_key, game)
        btn.clicked.connect(lambda _, g=game, c=card: self._primary_sdk_action(g, c))
        actions.addWidget(btn)
        uninstall_btn = QPushButton(_("sdk_uninstall"))
        uninstall_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        uninstall_btn.clicked.connect(lambda _, g=game, c=card: self._uninstall_sdk(g, c))
        uninstall_btn.setStyleSheet("QPushButton{background:#FFF; color:#DC2626; border:1px solid #FCA5A5; border-radius:8px; padding:6px 12px; font-size:12px; font-weight:600;} QPushButton:hover{background:#FEF2F2; border-color:#EF4444;} QPushButton:disabled{color:#9CA3AF; border-color:#E5E7EB;}")
        actions.addWidget(uninstall_btn)
        lay.addLayout(actions)
        # Keep references for status updates.
        card._status_key = status_key
        card._status_lbl = status_lbl
        card._btn = btn
        card._uninstall_btn = uninstall_btn
        self._refresh_game_status(card, game, status_key)
        return card

    def _set_game_status_label(self, lbl, status_key):
        st = self._sdk_status.get(status_key, "missing")
        if st == "installed":
            lbl.setText(_("sdk_installed"))
            lbl.setStyleSheet("font-size:12px; color:" + SUCCESS + "; font-weight:600; border:none;")
        elif st == "unsupported":
            lbl.setText(_("sdk_unsupported", ver=st.replace("unsupported", "").strip(":") or ""))
            lbl.setStyleSheet("font-size:12px; color:" + WARN + "; font-weight:600; border:none;")
        else:
            lbl.setText(_("sdk_not_installed"))
            lbl.setStyleSheet("font-size:12px; color:#6B7280; border:none;")

    def _set_game_button(self, btn, status_key, game):
        from core.i18n import t as _t
        st = self._sdk_status.get(status_key, "missing")
        if st == "installed":
            btn.setText(_("sdk_repair"))
            btn.setEnabled(True)
            btn.setStyleSheet("QPushButton{background:#ECFDF5; color:#065F46; border:1px solid #6EE7B7; border-radius:8px; padding:6px 14px; font-size:12px; font-weight:600;} QPushButton:hover{background:#D1FAE5; border-color:#10B981;}")
        elif st == "unsupported":
            btn.setText(_t(_lang_code, "common", "skip"))
            btn.setEnabled(False)
            btn.setStyleSheet("QPushButton{background:#F9FAFB; color:#9CA3AF; border:1px solid #E5E7EB; border-radius:8px; padding:6px 14px; font-size:12px; font-weight:600;} QPushButton:disabled{color:#9CA3AF;}")
        else:
            btn.setText(_("sdk_install"))
            btn.setEnabled(True)
            btn.setStyleSheet("QPushButton{background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 " + ACCENT_HI + ", stop:1 #059669); color:#fff; border:none; border-radius:8px; padding:6px 14px; font-size:12px; font-weight:600;} QPushButton:hover{background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #3DEDA6, stop:1 #06A977);} QPushButton:disabled{background:#E5E7EB; color:#9CA3AF;}")

    def _refresh_game_status(self, card, game, status_key):
        if hasattr(card, "_status_lbl"):
            self._set_game_status_label(card._status_lbl, status_key)
            self._set_game_button(card._btn, status_key, game)
            card._uninstall_btn.setVisible(self._sdk_status.get(status_key) == "installed")

    def _primary_sdk_action(self, game, card):
        status_key = "path:" + game["path"]
        if self._sdk_status.get(status_key) == "installed":
            self._repair_sdk(game, card)
        else:
            self._install_sdk(game, card)

    def _install_sdk(self, game, card):
        status_key = "path:" + game["path"]
        self._sdk_status[status_key] = "installing"
        card._btn.setEnabled(False)
        card._btn.setText(_("sdk_installing"))
        w = _SDKInstallWorker(game["path"], game["version"])
        w.done.connect(lambda ok, gp, msg: self._on_sdk_installed(ok, gp, msg, card, game))
        self._sdk_workers.append(w)
        w.start()

    def _on_sdk_installed(self, ok, game_path, msg, card, game):
        status_key = "path:" + game_path
        if ok and msg in ("installed", "already"):
            self._sdk_status[status_key] = "installed"
        elif msg.startswith("unsupported"):
            self._sdk_status[status_key] = "unsupported"
        else:
            self._sdk_status[status_key] = "failed"
        self._refresh_game_status(card, game, status_key)
        if not ok and not msg.startswith("unsupported"):
            QMessageBox.warning(self, "UltraPilot", _("sdk_install_fail", err=msg))

    def _repair_sdk(self, game, card):
        card._btn.setEnabled(False)
        card._uninstall_btn.setEnabled(False)
        card._btn.setText(_("sdk_repairing"))
        w = _SDKMaintenanceWorker(game["path"], game["version"], "repair")
        w.done.connect(lambda ok, gp, msg, action: self._on_sdk_maintained(ok, gp, msg, action, card, game))
        self._sdk_workers.append(w)
        w.start()

    def _uninstall_sdk(self, game, card):
        answer = QMessageBox.question(
            self, "UltraPilot", _("sdk_uninstall_confirm"),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        card._btn.setEnabled(False)
        card._uninstall_btn.setEnabled(False)
        card._uninstall_btn.setText(_("sdk_uninstalling"))
        w = _SDKMaintenanceWorker(game["path"], game["version"], "uninstall")
        w.done.connect(lambda ok, gp, msg, action: self._on_sdk_maintained(ok, gp, msg, action, card, game))
        self._sdk_workers.append(w)
        w.start()

    def _on_sdk_maintained(self, ok, game_path, msg, action, card, game):
        from core.sdk import sdk_downloader
        status_key = "path:" + game_path
        installed = sdk_downloader.is_sdk_installed(game_path)
        self._sdk_status[status_key] = "installed" if installed else "missing"
        card._uninstall_btn.setText(_("sdk_uninstall"))
        card._uninstall_btn.setEnabled(True)
        self._refresh_game_status(card, game, status_key)
        if not ok:
            key = "sdk_repair_fail" if action == "repair" else "sdk_uninstall_fail"
            QMessageBox.warning(self, "UltraPilot", _(key, err=msg))

    # --------------------------------------------------------------- map step
    def load_maps(self):
        # Run in a worker to keep UI responsive (network + yaml parse).
        class _Load(QThread):
            done = pyqtSignal(list, str)   # (datasets, suggested_key)
            def __init__(self, games):
                super().__init__()
                self.games = games
            def run(self):
                try:
                    from core.navigation import map_data
                    idx = map_data.list_datasets()
                    # Pick a suggested key from the first detected game version.
                    suggested = ""
                    if self.games:
                        prefer_promods = False  # ETS2 vanilla is the safe default
                        suggested = map_data.suggest_key(self.games[0]["version"], prefer_promods=prefer_promods)
                    self.done.emit(idx, suggested)
                except Exception as e:
                    logging.error("onboarding maps: %s", e)
                    self.done.emit([], "")
        self._map_load_worker = _Load(self._games)
        self._map_load_worker.done.connect(self._on_maps_loaded)
        self._map_load_worker.start()

    def _on_maps_loaded(self, datasets, suggested):
        # Clear old entries.
        for i in reversed(range(self.map_list_wrap.count())):
            w = self.map_list_wrap.itemAt(i).widget()
            if w:
                w.setParent(None)
                w.deleteLater()
        if not datasets:
            self.map_status_label.setText(_("map_none"))
            return
        self.map_status_label.setText("")
        # Filter by detected game kind.
        kinds = {g["kind"] for g in self._games}
        if kinds:
            def matches(key):
                k = key.lower()
                if "ats" in kinds:
                    return k.startswith("ats-")
                # ETS2 default: ets2-* and promods-* (and a few common mods).
                return k.startswith("ets2-") or k.startswith("promods-") or k.startswith("tmp-")
            datasets = [d for d in datasets if matches(d["key"])]
        # Sort: suggested first, then version desc.
        datasets.sort(key=lambda d: (d["key"] != suggested, _ver_tuple(d["version"]), d["key"]), reverse=False)
        datasets.sort(key=lambda d: (d["key"] != suggested))  # stable: suggested on top
        # Default-select the suggested (or first already-downloaded) entry.
        if not self.selected_map:
            self.selected_map = suggested or (next((d["key"] for d in datasets if d["downloaded"]), ""))
        for d in datasets:
            self.map_list_wrap.addWidget(self._build_map_card(d, suggested))

    def _build_map_card(self, d, suggested):
        from core.navigation import map_data
        card = QFrame()
        card.setCursor(Qt.CursorShape.PointingHandCursor)
        card._key = d["key"]
        lay = QHBoxLayout(card)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(10)
        left = QVBoxLayout()
        left.setSpacing(2)
        # Label: name (from config if cached) + version suffix.
        name = self._map_label(d, map_data)
        title = QLabel(name)
        title.setStyleSheet("font-size:14px; font-weight:700; color:#111827; border:none;")
        left.addWidget(title)
        status = QLabel()
        status.setObjectName("status")
        left.addWidget(status)
        lay.addLayout(left)
        lay.addStretch()
        btn = QPushButton()
        btn.setObjectName("action")
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        lay.addWidget(btn)
        self._refresh_map_card(card, d, status, btn)
        # Click on the card body (not the button) selects it.
        def select(_=None, c=card, key=d["key"]):
            self._select_map(key)
        for w in (card, title):
            w.mousePressEvent = select
        card._status = status
        card._btn = btn
        card._data = d
        return card

    def _map_label(self, d, map_data):
        # Try reading the config name (only present if already downloaded).
        cfg_name = ""
        try:
            cd = map_data.dataset_dir(d["key"])
            cfg = os.path.join(cd, "config.json")
            if os.path.exists(cfg):
                import json
                with open(cfg, encoding="utf-8") as f:
                    cfg_name = (json.load(f) or {}).get("name", "")
        except Exception:
            pass
        base = cfg_name or d["key"]
        game_version = d.get("game_version") or d["version"]
        ver = _("map_for_game", game=d["game"], ver=game_version)
        if d.get("mod"):
            base += f" · {d['mod']} {d.get('mod_version') or d['version']}"
        elif d.get("content"):
            base += f" · {d['content']}"
        return base + "  ·  " + ver

    def _refresh_map_card(self, card, d, status_lbl, btn):
        from core.navigation import map_data
        is_down = map_data.is_downloaded(d["key"])
        selected = (self.selected_map == d["key"])
        # Card border.
        if selected:
            card.setStyleSheet("QFrame{background:#ECFDF5; border:2px solid " + ACCENT + "; border-radius:12px;} QLabel{background:transparent;border:none;}")
        else:
            card.setStyleSheet("QFrame{background:#FFFFFF; border:1px solid #E5E7EB; border-radius:12px;} QLabel{background:transparent;border:none;}")
        if is_down:
            status_lbl.setText(_("map_ready"))
            status_lbl.setStyleSheet("font-size:12px; color:" + SUCCESS + "; font-weight:600; border:none;")
            btn.setText(_("map_selected") if selected else _("map_select"))
            btn.setStyleSheet("QPushButton{background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 " + ACCENT_HI + ", stop:1 #059669); color:#fff; border:none; border-radius:8px; padding:6px 14px; font-size:12px; font-weight:600;} QPushButton:hover{background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #3DEDA6, stop:1 #06A977);}")
            btn.setEnabled(True)
            btn.clicked.disconnect() if btn.receivers(btn.clicked) else None
            btn.clicked.connect(lambda _, key=d["key"]: self._select_map(key))
        else:
            # Estimate packed size from index/config if available (not loaded yet).
            btn.setText(_("map_download", size="~95 MB"))
            btn.setStyleSheet("QPushButton{background:#F3F4F6; color:#111827; border:1px solid #E5E7EB; border-radius:8px; padding:6px 14px; font-size:12px; font-weight:600;} QPushButton:hover{border-color:" + ACCENT + "; color:" + ACCENT + ";}")
            status_lbl.setText("")
            btn.setEnabled(True)
            try:
                btn.clicked.disconnect()
            except Exception:
                pass
            btn.clicked.connect(lambda _, key=d["key"], c=card: self._download_map(key, c))

    def _select_map(self, key):
        self.selected_map = key
        self.state.set("selected_map", key)
        # Refresh all cards.
        for i in range(self.map_list_wrap.count()):
            card = self.map_list_wrap.itemAt(i).widget()
            if card is not None and hasattr(card, "_data"):
                self._refresh_map_card(card, card._data, card._status, card._btn)

    def _download_map(self, key, card):
        if self._map_worker is not None and self._map_worker.isRunning():
            return
        btn = card._btn
        status = card._status
        btn.setEnabled(False)
        btn.setText(_("map_downloading", pct=0))
        self._map_worker = _MapDownloadWorker(key)
        self._map_worker.progress.connect(
            lambda frac, txt, b=btn, s=status: self._on_map_progress(frac, txt, b, s))
        self._map_worker.done.connect(lambda ok, k: self._on_map_downloaded(ok, k))
        self._map_worker.start()

    def _on_map_progress(self, frac, txt, btn, status):
        from core.i18n import t as _t
        pct = int(frac * 100)
        btn.setText(_t(_lang_code, "onboarding", "map_downloading", pct=pct))
        status.setText(txt)
        status.setStyleSheet("font-size:12px;color:#4B5563;border:none;")

    def _on_map_downloaded(self, ok, key):
        # Refresh the matching card.
        for i in range(self.map_list_wrap.count()):
            card = self.map_list_wrap.itemAt(i).widget()
            if card is not None and getattr(card, "_key", None) == key:
                from core.navigation import map_data
                d = next((x for x in map_data.list_datasets() if x["key"] == key), card._data)
                card._data = d
                self._refresh_map_card(card, d, card._status, card._btn)
                if ok:
                    self._select_map(key)
                break

    # --------------------------------------------------------------- done
    def _build_done_summary(self):
        from core.i18n import t as _t
        # Language line.
        lang_name = next((l["name"] for l in i18n.available() if l["code"] == self.selected_lang_code), self.selected_lang_code)
        # SDK line.
        sdk_installed = any(self._sdk_status.get("path:" + g["path"]) == "installed" for g in self._games)
        sdk_line = _("sdk_installed") if sdk_installed else (_("sdk_none") if not self._games else _("sdk_not_installed"))
        # Map line.
        map_line = self.selected_map or _("done_map_none")
        self.done_summary.setText(
            _t(_lang_code, "onboarding", "done_lang") + " " + lang_name + "    ·    " +
            _t(_lang_code, "onboarding", "done_sdk") + " " + sdk_line + "    ·    " +
            _t(_lang_code, "onboarding", "done_map") + " " + map_line)

    # --------------------------------------------------------------- finalize
    def _finalize(self):
        # Persist the chosen language + map + onboarded flag to settings.json.
        try:
            from core.settings.manager import SettingsManager
            sm = SettingsManager()
            sm.set("onboarded", True)
            sm.set("ui_language_code", self.selected_lang_code)
            if self.selected_map:
                sm.set("selected_map", self.selected_map)
            # Also publish to shared state for the live app.
            self.state.set("ui_language_code", self.selected_lang_code)
            self.state.set("selected_map", self.selected_map)
            self.state.set("onboarded", True)
        except Exception as e:
            logging.error("onboarding finalize: %s", e)
        self.finished.emit()
        self.close()


def _ver_tuple(v):
    """Turn '1.59' into (1, 59) for sorting; unknown → (0,)."""
    try:
        return tuple(int(p) for p in str(v).split("."))
    except Exception:
        return (0,)
