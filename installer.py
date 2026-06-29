"""
UltraPilot — modern installer (PyQt6).

A bespoke dark/light setup window (logo hero, step cards, smooth navigation)
that installs the pre-built application, copies the SCS SDK plugin DLLs into the
game, installs the ViGEmBus driver, and creates Start-menu / desktop shortcuts.

Build it into a single UltraPilot_Installer.exe with build_installer.py.
"""

import os
import sys
import json
import shutil
import subprocess

from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QStackedWidget, QProgressBar, QTextEdit, QFileDialog, QComboBox, QCheckBox,
    QLineEdit, QScrollArea, QFrame, QMessageBox,
)
from PyQt6.QtGui import QPixmap, QIcon

APP_NAME = "UltraPilot"


def _res(*parts):
    """Resource path: _MEIPASS first (PyInstaller onefile), then exe folder, then source."""
    roots = []
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        roots.append(meipass)
    if getattr(sys, "frozen", False):
        roots.append(os.path.dirname(sys.executable))
    roots.append(os.path.dirname(os.path.abspath(__file__)))
    for r in roots:
        cand = os.path.join(r, *parts)
        if os.path.exists(cand):
            return cand
    return os.path.join(roots[-1], *parts)


ICON_PATH = _res("assets", "favicon.ico")
LOGO_PATH = _res("assets", "logo.png")
RECORD_PATH = os.path.join(os.environ.get("LOCALAPPDATA", os.path.expanduser("~")),
                           "UltraPilot", "install.json")
ACCENT = "#10B981"

DARK = {"bg": "#0E1116", "bg2": "#151A21", "card": "#1A2029", "card2": "#222A35",
        "text": "#E6E8EB", "muted": "#8A93A0", "border": "#272F3A",
        "title": "#34D399", "field": "#11161D"}
LIGHT = {"bg": "#F2F4F7", "bg2": "#FFFFFF", "card": "#FFFFFF", "card2": "#F7F9FB",
         "text": "#111827", "muted": "#6B7280", "border": "#E5E8EC",
         "title": "#065F46", "field": "#FFFFFF"}


def _qss(theme):
    c = DARK if theme == "dark" else LIGHT
    return (
        "#Window { background: " + c['bg'] + "; }"
        " #Hero { background: " + c['bg2'] + "; }"
        " #StepBadge { background: " + c['card2'] + "; border: 1px solid " + c['border'] + "; border-radius: 11px; }"
        " QLabel { color: " + c['text'] + "; }"
        " QLabel#Title { font-size: 30px; font-weight: 800; }"
        " QLabel#Subtitle { font-size: 15px; color: " + c['muted'] + "; }"
        " QLabel#Brand { font-size: 22px; font-weight: 800; color: " + c['title'] + "; }"
        " QLabel#BrandSub { font-size: 11px; font-weight: 600; color: " + c['muted'] + "; }"
        " QLabel#StepLabel { font-size: 13px; font-weight: 600; color: " + c['muted'] + "; }"
        " QLabel#StepLabelActive { font-size: 13px; font-weight: 700; color: " + c['title'] + "; }"
        " QLabel#Caption { font-size: 12px; color: " + c['muted'] + "; }"
        " QLabel#Desc { font-size: 14px; color: " + c['text'] + "; }"
        " QPushButton#Primary { background: " + ACCENT + "; color: #FFFFFF; border: none;"
        " border-radius: 10px; padding: 11px 22px; font-size: 14px; font-weight: 700; }"
        " QPushButton#Primary:hover { background: #059669; }"
        " QPushButton#Primary:disabled { background: " + c['card2'] + "; color: " + c['muted'] + "; }"
        " QPushButton#Ghost { background: transparent; color: " + c['text'] + "; border: 1px solid " + c['border'] + ";"
        " border-radius: 10px; padding: 11px 18px; font-size: 14px; font-weight: 600; }"
        " QPushButton#Ghost:hover { border-color: " + ACCENT + "; color: " + ACCENT + "; }"
        " QPushButton#Icon { background: " + c['card2'] + "; color: " + c['text'] + "; border: 1px solid " + c['border'] + ";"
        " border-radius: 9px; padding: 7px 12px; font-size: 14px; }"
        " QPushButton#Icon:hover { border-color: " + ACCENT + "; }"
        " QComboBox, QLineEdit { background: " + c['field'] + "; color: " + c['text'] + ";"
        " border: 1px solid " + c['border'] + "; border-radius: 9px; padding: 9px 12px; font-size: 14px; }"
        " QComboBox:hover, QLineEdit:focus { border-color: " + ACCENT + "; }"
        " QComboBox::drop-down { border: none; width: 24px; }"
        " QComboBox QAbstractItemView { background: " + c['card'] + "; color: " + c['text'] + ";"
        " border: 1px solid " + c['border'] + "; border-radius: 8px; outline: none;"
        " selection-background-color: " + ACCENT + "; selection-color: #FFFFFF; }"
        " QCheckBox { spacing: 9px; color: " + c['text'] + "; font-size: 14px; }"
        " QCheckBox::indicator { width: 19px; height: 19px; border: 1px solid " + c['border'] + ";"
        " border-radius: 5px; background: " + c['field'] + "; }"
        " QCheckBox::indicator:checked { background: " + ACCENT + "; border-color: " + ACCENT + "; }"
        " QProgressBar { background: " + c['card2'] + "; border: none; border-radius: 7px;"
        " height: 10px; text-align: center; color: transparent; }"
        " QProgressBar::chunk { background: " + ACCENT + "; border-radius: 7px; }"
        " QTextEdit { background: " + c['field'] + "; color: " + c['text'] + ";"
        " border: 1px solid " + c['border'] + "; border-radius: 9px;"
        " font-family: 'Consolas','Cascadia Mono',monospace; font-size: 12px; }"
        " QScrollArea { border: none; background: transparent; }"
        " QScrollBar:vertical { background: transparent; width: 10px; margin: 2px; }"
        " QScrollBar::handle:vertical { background: " + c['border'] + "; border-radius: 5px; min-height: 30px; }"
        " QScrollBar::add-line, QScrollBar::sub-line { height: 0; }"
    )


TR = {
    "Slovensky": {
        "win": "UltraPilot — Inštalácia",
        "brand": "UltraPilot", "brand_sub": "Autopilot pre Euro Truck Simulator 2",
        "welcome_t": "Vitaj v UltraPilot",
        "welcome_d": "Pokročilý autopilot pre ETS2: udržiavanie pruhu, adaptívny tempomat, vyhýbanie sa prekážkam, navigácia podľa mapy, HUD a hlasové oznámenia. Sprievodca ťa prevedie inštaláciou krok za krokom.",
        "lang": "Jazyk",
        "lic_t": "Licenčné podmienky", "lic_s": "Odsúhlas podmienky pre pokračovanie.",
        "lic_accept": "Čítal(a) som a súhlasím s podmienkami",
        "path_t": "Cesta inštalácie", "path_s": "Vyber, kam sa má UltraPilot nainštalovať.",
        "path_lbl": "Priečinok inštalácie:", "browse": "Prehľadávať…",
        "inst_t": "Inštalujem UltraPilot", "inst_s": "Počkaj, kým sa súčasti nainštalujú.",
        "fin_t": "Inštalácia dokončená", "fin_s": "UltraPilot je pripravený na použitie.",
        "fin_launch": "Spustiť UltraPilot teraz",
        "install_btn": "Nainštalovať",
        "steps": ("Úvod", "Licencia", "Cesta", "Inštalácia", "Dokončenie"),
        "next": "Ďalej", "back": "Späť", "finish": "Dokončiť",
        "s_prep": "Pripravujem inštaláciu…", "s_dll": "Inštalujem SCS pluginy do hry…",
        "s_dll_ok": "  ✓ {} → {}", "s_dll_none": "Hra zatiaľ nenájdená — DLL sa nainštalujú pri prvom spustení.",
        "s_vigem": "Nastavujem ViGEmBus ovládač…", "s_short": "Vytváram skratky…",
        "s_done": "Hotovo! UltraPilot je nainštalovaný.", "s_err": "Chyba: {}",
        "lic_text": (
            "ULTRAPILOT — LICENČNÉ PODMIENKY\n\n"
            "1. ÚČEL. UltraPilot je nástroj asistencie vodiča určený výhradne na "
            "vzdelávacie a zábavné účely v rámci videohry Euro Truck Simulator 2. "
            "Nie je určený pre žiadne reálne vozidlá.\n\n"
            "2. ZODPOVEDNÉ POUŽÍVANIE. Úplnú zodpovednosť za dohľad nad softvérom "
            "nesieš ty. Softvér sa môže správať nepredvídateľne.\n\n"
            "3. BEZ ZÁRUKY. Softvér je poskytovaný „taký aký je\", bez akejkoľvek záruky.\n\n"
            "4. OBMEDZENIE ZODPOVEDNOSTI. Autori nenesú zodpovednosť za žiadne škody.\n\n"
            "5. KOMPONENTY TRETÍCH STRÁN. Inštalátor môže nainštalovať ovládače "
            "(ViGEmBus) a herné SDK pluginy podliehajúce ich vlastným licenciám.\n\n"
            "Inštaláciou UltraPilotu vyjadruješ, že si si tieto podmienky prečítal a súhlasíš."
        ),
    },
}


def tr_get(lang, key):
    return TR.get(lang, {}).get(key, TR["Slovensky"].get(key, key))


class InstallWorker(QThread):
    log = pyqtSignal(str)
    progress = pyqtSignal(int)
    finished_ok = pyqtSignal(bool, str)

    def __init__(self, install_path, lang):
        super().__init__()
        self.install_path = install_path
        self.lang = lang
        self.t = TR.get(lang, TR["Slovensky"])

    def _real_python(self):
        py = shutil.which("py") or shutil.which("py.exe")
        if py:
            return [py, "-3"]
        for name in ("python", "python.exe", "python3", "python3.exe"):
            found = shutil.which(name)
            if found:
                return [found]
        return []

    def _copy_bundled(self):
        here = getattr(sys, "_MEIPASS", None) or \
            (os.path.dirname(sys.executable) if getattr(sys, "frozen", False)
             else os.path.dirname(os.path.abspath(__file__)))
        for item in ("core", "plugins", "sdk", "ui", "assets",
                     "main.py", "bootloader.py", "requirements.txt"):
            s = os.path.join(here, item)
            if not os.path.exists(s):
                continue
            d = os.path.join(self.install_path, item)
            try:
                if os.path.isdir(s):
                    shutil.copytree(s, d, dirs_exist_ok=True,
                                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
                else:
                    shutil.copy2(s, d)
            except Exception:
                pass

    def _fetch_repo(self):
        try:
            tmp = self.install_path + "_clone"
            if os.path.isdir(tmp):
                shutil.rmtree(tmp, ignore_errors=True)
            r = subprocess.run(["git", "clone", "--depth", "1",
                                "https://github.com/matule123/ets2la.git", tmp],
                               capture_output=True, text=True, timeout=600)
            if r.returncode == 0 and os.path.exists(os.path.join(tmp, "main.py")):
                for item in os.listdir(tmp):
                    s = os.path.join(tmp, item)
                    d = os.path.join(self.install_path, item)
                    if os.path.isdir(s):
                        shutil.copytree(s, d, dirs_exist_ok=True)
                    else:
                        shutil.copy2(s, d)
                shutil.rmtree(tmp, ignore_errors=True)
                self.log.emit("  Stiahnuté z GitHub.")
                return True
        except Exception as e:
            self.log.emit("  git clone nedostupný (" + str(e) + ").")
        try:
            import requests, zipfile, io
            self.log.emit("  Sťahujem zdrojový zip…")
            resp = requests.get("https://github.com/matule123/ets2la/archive/refs/heads/main.zip",
                                timeout=120)
            if resp.status_code == 200:
                zf = zipfile.ZipFile(io.BytesIO(resp.content))
                zf.extractall(self.install_path)
                root = os.path.join(self.install_path, "ets2la-main")
                if os.path.isdir(root):
                    for item in os.listdir(root):
                        shutil.move(os.path.join(root, item),
                                    os.path.join(self.install_path, item))
                    shutil.rmtree(root, ignore_errors=True)
                self.log.emit("  Stiahnuté a rozbalené.")
                return True
        except Exception as e:
            self.log.emit("  Stiahnutie zlyhalo (" + str(e) + ").")
        return False

    def _pip_install(self):
        req = os.path.join(self.install_path, "requirements.txt")
        py = self._real_python()
        if not py:
            self.log.emit("  Python nebol nájdený na PATH — nainštaluj Python 3.")
            return
        try:
            self.log.emit("  Používam Python: " + py[0])
            if os.path.exists(req):
                subprocess.run([*py, "-m", "pip", "install", "-r", req],
                               capture_output=True, timeout=1800)
            subprocess.run([*py, "-m", "pip", "install", "pyqtgraph", "PyOpenGL"],
                           capture_output=True, timeout=600)
            self.log.emit("  Závislosti nainštalované.")
        except Exception as e:
            self.log.emit("  problém s pip (" + str(e) + ") — nainštaluj manuálne.")

    def _make_shortcuts(self, exe_path, mode):
        """Create a robust launcher (.bat) + Desktop/Start-menu shortcuts to it.

        Directly targeting ``pythonw.exe "main.py"`` from a .lnk broke on many
        machines (the Microsoft Store python stub refuses to spawn the app's
        multiprocessing children, so the shortcut silently does nothing). A
        small launcher .bat in the install folder runs ``py -3 main.py`` with
        the correct working dir and keeps a window open on error — the shortcut
        points at that, which always works."""
        icon = os.path.join(self.install_path, "assets", "favicon.ico")
        main_py = os.path.basename(exe_path)
        bat_path = os.path.join(self.install_path, "UltraPilot.bat")
        try:
            with open(bat_path, "w", encoding="utf-8") as f:
                f.write("@echo off\r\n")
                f.write("cd /d \"" + self.install_path + "\"\r\n")
                f.write("start \"\" /b py -3 " + main_py + "\r\n")
                f.write("exit\r\n")
        except Exception as e:
            self.log.emit("  launcher: " + str(e))
            bat_path = exe_path  # fall back to the script directly

        for folder in (os.path.join(os.environ.get("USERPROFILE", ""), "Desktop"),
                       os.path.join(os.environ.get("APPDATA", ""),
                                    "Microsoft\\Windows\\Start Menu\\Programs")):
            try:
                if not folder:
                    continue
                os.makedirs(folder, exist_ok=True)
                lnk = os.path.join(folder, APP_NAME + ".lnk")
                ps = (
                    '$s=(New-Object -ComObject WScript.Shell).CreateShortcut("' + lnk + '");'
                    '$s.TargetPath="' + bat_path + '";'
                    '$s.WorkingDirectory="' + self.install_path + '";'
                    '$s.WindowStyle=7;'
                    + ('$s.IconLocation="' + icon + '";' if os.path.exists(icon) else "")
                    + '$s.Save()'
                )
                subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                               capture_output=True)
            except Exception as e:
                self.log.emit("  skratka: " + str(e))

    def run(self):
        try:
            mode = "source"
            self.log.emit(self.t["s_prep"])
            # --- Pre-flight: check every prerequisite BEFORE touching disk ---
            missing = []
            py = self._real_python()
            if not py:
                missing.append("Python 3 — stiahni z https://python.org "
                               "(začiarkni „Add Python to PATH“)")
            else:
                # Verify pip works in that Python.
                try:
                    subprocess.run([*py, "-m", "pip", "--version"],
                                   capture_output=True, timeout=30)
                    self.log.emit("✓ Python: " + py[0])
                except Exception:
                    missing.append("pip pre Python — obyčajne sa inštaluje s Pythonom")
            # git is optional (we fall back to a zip download), just warn.
            if not (shutil.which("git")):
                self.log.emit("  (git nie je nainštalovaný — použije sa zip stiahnutie)")
            if missing:
                self.log.emit("")
                self.log.emit("✗ Chýbajú požiadavky:")
                for m in missing:
                    self.log.emit("   • " + m)
                self.log.emit("Nainštaluj ich a spusti inštalátor znova.")
                self.finished_ok.emit(False, "")
                return
            os.makedirs(self.install_path, exist_ok=True)
            self.progress.emit(3)
            self.log.emit("Kopírujem súbory UltraPilot…")
            self._copy_bundled()
            if not os.path.exists(os.path.join(self.install_path, "main.py")):
                self.log.emit("  Balík neúplný — sťahujem z GitHub…")
                if not self._fetch_repo():
                    raise RuntimeError("Nepodarilo sa získať súbory UltraPilot.")
            self.progress.emit(45)
            self.log.emit("Inštalujem Python závislosti (môže trvať pár minút)…")
            self._pip_install()
            exe_path = os.path.join(self.install_path, "main.py")
            self.progress.emit(75)
            self.log.emit(self.t["s_dll"])
            try:
                from core.sdk.game_utils import install_game_dlls
                folders = install_game_dlls(os.path.join(self.install_path, "assets"))
                if folders:
                    for fld in folders:
                        self.log.emit(self.t["s_dll_ok"].format("SCS pluginy", fld))
                else:
                    self.log.emit(self.t["s_dll_none"])
            except Exception as e:
                self.log.emit("  (" + str(e) + ")")
            self.progress.emit(80)
            self.log.emit(self.t["s_vigem"])
            try:
                from core.sdk.vigembus import ensure_vigembus
                ensure_vigembus(os.path.join(self.install_path, "assets"),
                                log=self.log.emit)
            except Exception as e:
                self.log.emit("  (" + str(e) + ")")
            self.progress.emit(90)
            self.log.emit(self.t["s_short"])
            self._make_shortcuts(exe_path, mode)
            try:
                rec = {"install_path": self.install_path, "exe_path": exe_path, "mode": mode}
                os.makedirs(os.path.dirname(RECORD_PATH), exist_ok=True)
                with open(RECORD_PATH, "w") as f:
                    json.dump(rec, f)
            except Exception:
                pass
            self.progress.emit(100)
            self.log.emit("")
            self.log.emit("✔ " + self.t["s_done"])
            self.finished_ok.emit(True, exe_path)
        except Exception as e:
            self.log.emit(self.t["s_err"].format(e))
            self.finished_ok.emit(False, "")
def _primary_btn(text):
    b = QPushButton(text)
    b.setObjectName("Primary")
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    return b


def _ghost_btn(text):
    b = QPushButton(text)
    b.setObjectName("Ghost")
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    return b


class InstallerWindow(QWidget):
    """A bespoke multi-step installer window (no QWizard).

    Steps live in a QStackedWidget; the hero header + step rail stay fixed. The
    theme toggle is a normal button (the QWizard custom-button approach never
    re-applied the stylesheet reliably)."""

    def __init__(self):
        super().__init__()
        self.setObjectName("Window")
        self.lang = "Slovensky"
        self.theme = "dark"
        self.exe_path = ""
        self._worker = None
        self._cur = 0
        self.setWindowTitle(TR[self.lang]["win"])
        self.setFixedSize(760, 600)
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("UltraPilot.Installer")
        except Exception:
            pass
        if os.path.exists(ICON_PATH):
            self.setWindowIcon(QIcon(ICON_PATH))
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_hero())
        self._build_step_rail_widget(root)
        self.stack = QStackedWidget()
        self._build_welcome()
        self._build_license()
        self._build_path()
        self._build_install()
        self._build_finish()
        root.addWidget(self.stack, stretch=1)
        self._build_footer(root)
        self._apply_theme()
        self._go_step(0)

    def _build_hero(self):
        hero = QFrame()
        hero.setObjectName("Hero")
        hero.setFixedHeight(76)
        h = QHBoxLayout(hero)
        h.setContentsMargins(28, 14, 20, 14)
        logo = QLabel()
        pm = QIcon(ICON_PATH).pixmap(44, 44)
        if pm.isNull():
            pm = QPixmap(LOGO_PATH).scaledToWidth(44, Qt.TransformationMode.SmoothTransformation)
        if not pm.isNull():
            logo.setPixmap(pm)
        logo.setStyleSheet("border:none;")
        h.addWidget(logo)
        brand_col = QVBoxLayout()
        brand_col.setSpacing(0)
        brand = QLabel(TR[self.lang]["brand"])
        brand.setObjectName("Brand")
        sub = QLabel(TR[self.lang]["brand_sub"])
        sub.setObjectName("BrandSub")
        brand_col.addWidget(brand)
        brand_col.addWidget(sub)
        h.addLayout(brand_col)
        h.addStretch()
        self.theme_btn = QPushButton("light")
        self.theme_btn.setObjectName("Icon")
        self.theme_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.theme_btn.setFixedWidth(64)
        self.theme_btn.clicked.connect(self._toggle_theme)
        h.addWidget(self.theme_btn)
        return hero

    def _build_step_rail_widget(self, parent_layout):
        rail = QWidget()
        rail.setFixedHeight(46)
        h = QHBoxLayout(rail)
        h.setContentsMargins(28, 8, 28, 8)
        h.setSpacing(10)
        self._step_labels = []
        steps = TR[self.lang]["steps"]
        for i, name in enumerate(steps):
            badge = QLabel(str(i + 1))
            badge.setObjectName("StepBadge")
            badge.setFixedSize(22, 22)
            badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl = QLabel(name)
            lbl.setObjectName("StepLabel")
            cell = QHBoxLayout()
            cell.setSpacing(7)
            cell.addWidget(badge)
            cell.addWidget(lbl)
            wrap = QWidget()
            wrap.setLayout(cell)
            wrap.setStyleSheet("border:none;")
            h.addWidget(wrap)
            self._step_labels.append((badge, lbl, wrap))
            if i < len(steps) - 1:
                sep = QLabel("·")
                sep.setStyleSheet("color:#3A4250; border:none;")
                h.addWidget(sep)
        h.addStretch()
        parent_layout.addWidget(rail)

    def _page_frame(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(36, 24, 36, 24)
        lay.setSpacing(14)
        scroll.setWidget(inner)
        return scroll, lay

    def _build_welcome(self):
        scroll, lay = self._page_frame()
        title = QLabel(TR[self.lang]["welcome_t"])
        title.setObjectName("Title")
        lay.addWidget(title)
        desc = QLabel(TR[self.lang]["welcome_d"])
        desc.setObjectName("Desc")
        desc.setWordWrap(True)
        lay.addWidget(desc)
        lay.addSpacing(8)
        chips = QHBoxLayout()
        chips.setSpacing(8)
        for icon, txt in (("P", "Pruh"), ("T", "Tempomat"), ("S", "Semafor"),
                          ("N", "Navigácia"), ("H", "HUD")):
            c = QLabel(icon + "  " + txt)
            c.setStyleSheet("background:#1A2029; color:#E6E8EB; border:1px solid #272F3A;"
                            "border-radius:14px; padding:6px 12px; font-size:12px; font-weight:600;")
            chips.addWidget(c)
        chips.addStretch()
        lay.addLayout(chips)
        row = QHBoxLayout()
        row.setSpacing(10)
        cap = QLabel(TR[self.lang]["lang"])
        cap.setObjectName("Caption")
        self.lang_combo = QComboBox()
        self.lang_combo.addItems(list(TR.keys()))
        self.lang_combo.setCurrentText(self.lang)
        self.lang_combo.currentTextChanged.connect(self._on_lang)
        row.addWidget(cap)
        row.addWidget(self.lang_combo)
        row.addStretch()
        lay.addLayout(row)
        lay.addStretch()
        self.stack.addWidget(scroll)

    def _build_license(self):
        scroll, lay = self._page_frame()
        title = QLabel(TR[self.lang]["lic_t"])
        title.setObjectName("Title")
        sub = QLabel(TR[self.lang]["lic_s"])
        sub.setObjectName("Subtitle")
        self.lic_text = QTextEdit()
        self.lic_text.setReadOnly(True)
        self.lic_text.setText(TR[self.lang]["lic_text"])
        self.lic_chk = QCheckBox(TR[self.lang]["lic_accept"])
        self.lic_chk.toggled.connect(self._update_nav)
        lay.addWidget(title)
        lay.addWidget(sub)
        lay.addWidget(self.lic_text)
        lay.addWidget(self.lic_chk)
        self.stack.addWidget(scroll)

    def _build_path(self):
        scroll, lay = self._page_frame()
        title = QLabel(TR[self.lang]["path_t"])
        title.setObjectName("Title")
        sub = QLabel(TR[self.lang]["path_s"])
        sub.setObjectName("Subtitle")
        lay.addWidget(title)
        lay.addWidget(sub)
        lay.addSpacing(8)
        lbl = QLabel(TR[self.lang]["path_lbl"])
        lbl.setObjectName("Caption")
        row = QHBoxLayout()
        row.setSpacing(8)
        default = os.path.join(os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "UltraPilot")
        self.path_edit = QLineEdit(default)
        browse = _ghost_btn(TR[self.lang]["browse"])
        browse.clicked.connect(self._browse)
        row.addWidget(self.path_edit, stretch=1)
        row.addWidget(browse)
        lay.addWidget(lbl)
        lay.addLayout(row)
        lay.addStretch()
        self.stack.addWidget(scroll)

    def _build_install(self):
        scroll, lay = self._page_frame()
        title = QLabel(TR[self.lang]["inst_t"])
        title.setObjectName("Title")
        sub = QLabel(TR[self.lang]["inst_s"])
        sub.setObjectName("Subtitle")
        lay.addWidget(title)
        lay.addWidget(sub)
        lay.addSpacing(8)
        self.progress = QProgressBar()
        self.progress.setValue(0)
        lay.addWidget(self.progress)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        lay.addWidget(self.log_view)
        self.stack.addWidget(scroll)

    def _build_finish(self):
        scroll, lay = self._page_frame()
        self.fin_title = QLabel(TR[self.lang]["fin_t"])
        self.fin_title.setObjectName("Title")
        self.fin_sub = QLabel(TR[self.lang]["fin_s"])
        self.fin_sub.setObjectName("Subtitle")
        icon = QLabel("✔")
        icon.setStyleSheet("font-size:54px; border:none; color:#10B981;")
        icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.launch_chk = QCheckBox(TR[self.lang]["fin_launch"])
        self.launch_chk.setChecked(True)
        lay.addStretch()
        lay.addWidget(icon)
        lay.addWidget(self.fin_title)
        lay.addWidget(self.fin_sub)
        lay.addSpacing(10)
        lay.addWidget(self.launch_chk)
        lay.addStretch()
        self.stack.addWidget(scroll)

    def _build_footer(self, parent_layout):
        foot = QFrame()
        foot.setFixedHeight(64)
        fh = QHBoxLayout(foot)
        fh.setContentsMargins(28, 12, 28, 12)
        fh.setSpacing(10)
        self.back_btn = _ghost_btn(TR[self.lang]["back"])
        self.back_btn.clicked.connect(self._back)
        self.next_btn = _primary_btn(TR[self.lang]["next"])
        self.next_btn.clicked.connect(self._next)
        fh.addStretch()
        fh.addWidget(self.back_btn)
        fh.addWidget(self.next_btn)
        parent_layout.addWidget(foot)

    def _apply_theme(self):
        self.setStyleSheet(_qss(self.theme))
        self.theme_btn.setText("TMA" if self.theme == "dark" else "SVETLÁ")

    def _toggle_theme(self):
        self.theme = "light" if self.theme == "dark" else "dark"
        self._apply_theme()

    def _on_lang(self, lang):
        self.lang = lang

    def _go_step(self, idx):
        idx = max(0, min(idx, self.stack.count() - 1))
        self.stack.setCurrentIndex(idx)
        self._cur = idx
        for i, (badge, lbl, wrap) in enumerate(self._step_labels):
            active = (i == idx)
            done = (i < idx)
            col = "#10B981" if (active or done) else "#222A35"
            fg = "#FFFFFF" if (active or done) else "#8A93A0"
            bd = "#10B981" if (active or done) else "#272F3A"
            badge.setStyleSheet(
                "background:" + col + "; color:" + fg + "; border:1px solid " + bd + "; border-radius:11px;")
            lbl.setObjectName("StepLabelActive" if active else "StepLabel")
        self.setStyleSheet(self.styleSheet())
        self._update_nav()

    def _update_nav(self):
        i = self._cur
        self.back_btn.setVisible(i > 0 and i < 4)
        if i == 0:
            self.next_btn.setText(TR[self.lang]["next"])
            self.next_btn.setEnabled(True)
        elif i == 1:
            self.next_btn.setText(TR[self.lang]["next"])
            self.next_btn.setEnabled(self.lic_chk.isChecked())
        elif i == 2:
            self.next_btn.setText(TR[self.lang]["install_btn"])
            self.next_btn.setEnabled(True)
        elif i == 3:
            self.next_btn.setText(TR[self.lang]["next"])
            self.next_btn.setEnabled(self._worker is None or not self._worker.isRunning())
        elif i == 4:
            self.next_btn.setText(TR[self.lang]["finish"])
            self.next_btn.setEnabled(True)

    def _next(self):
        i = self._cur
        if i == 2:
            self._start_install()
        if i < 4:
            self._go_step(i + 1)
        else:
            self.close()

    def _back(self):
        if self._cur > 0 and self._cur < 4:
            self._go_step(self._cur - 1)

    def _browse(self):
        d = QFileDialog.getExistingDirectory(self, TR[self.lang]["path_t"],
                                             self.path_edit.text())
        if d:
            self.path_edit.setText(d)

    def _start_install(self):
        if self._worker is not None and self._worker.isRunning():
            return
        path = self.path_edit.text().strip() or os.path.join(
            os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "UltraPilot")
        self.progress.setValue(0)
        self.log_view.clear()
        self._worker = InstallWorker(path, self.lang)
        self._worker.log.connect(self.log_view.append)
        self._worker.progress.connect(self.progress.setValue)
        self._worker.finished_ok.connect(self._on_done)
        self._worker.start()

    def _on_done(self, ok, exe_path):
        self.exe_path = exe_path
        if ok:
            self._go_step(4)
        else:
            QMessageBox.warning(self, APP_NAME, "Inštalácia zlyhala. Pozri log vyššie.")

    def closeEvent(self, event):
        if hasattr(self, "launch_chk") and self.launch_chk.isChecked() and self.exe_path:
            try:
                if sys.platform == "win32":
                    os.startfile(self.exe_path)
                else:
                    subprocess.Popen([sys.executable, self.exe_path])
            except Exception:
                pass
        super().closeEvent(event)


def _read_record():
    try:
        if os.path.exists(RECORD_PATH):
            with open(RECORD_PATH) as f:
                rec = json.load(f)
            if rec.get("install_path") and os.path.isdir(rec["install_path"]):
                return rec
    except Exception:
        pass
    return None


def _uninstall(rec):
    try:
        shutil.rmtree(rec["install_path"], ignore_errors=True)
    except Exception:
        pass
    for folder in (os.path.join(os.environ.get("USERPROFILE", ""), "Desktop"),
                   os.path.join(os.environ.get("APPDATA", ""),
                                "Microsoft\\Windows\\Start Menu\\Programs")):
        lnk = os.path.join(folder, "UltraPilot.lnk")
        try:
            if os.path.exists(lnk):
                os.remove(lnk)
        except Exception:
            pass
    try:
        os.remove(RECORD_PATH)
    except Exception:
        pass


def _maintenance_dialog(rec):
    box = QMessageBox()
    box.setWindowTitle("UltraPilot")
    if os.path.exists(ICON_PATH):
        box.setWindowIcon(QIcon(ICON_PATH))
    box.setStyleSheet(_qss("dark"))
    box.setText("UltraPilot je už nainštalovaný.\nČo chceš spraviť?")
    box.addButton("Opraviť", QMessageBox.ButtonRole.AcceptRole)
    uninstall = box.addButton("Odinštalovať", QMessageBox.ButtonRole.DestructiveRole)
    box.addButton("Zrušiť", QMessageBox.ButtonRole.RejectRole)
    box.exec()
    clicked = box.clickedButton()
    if clicked == uninstall:
        return "uninstall"
    if box.buttonRole(clicked) == QMessageBox.ButtonRole.AcceptRole:
        return "repair"
    return "cancel"


def main():
    app = QApplication(sys.argv)
    rec = _read_record()
    if rec is not None:
        action = _maintenance_dialog(rec)
        if action == "uninstall":
            _uninstall(rec)
            QMessageBox.information(None, APP_NAME, "UltraPilot bol odinštalovaný.")
            return
        elif action == "cancel":
            return
    w = InstallerWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
