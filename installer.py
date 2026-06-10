"""
UltraPilot — pretty installer (PyQt6).

A dark, branded setup wizard (logo, language picker, live status log) that
installs the pre-built application, copies the SCS SDK plugin DLLs into the game,
installs the ViGEmBus driver, and creates Start-menu / desktop shortcuts.

Build it into a single UltraPilot_Installer.exe with build_installer.py.
"""

import os
import sys
import shutil
import subprocess
from pathlib import Path

from PyQt6.QtWidgets import (
    QApplication, QWizard, QWizardPage, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QProgressBar, QTextEdit, QFileDialog, QComboBox, QCheckBox,
    QLineEdit,
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtGui import QPixmap, QIcon, QColor

APP_NAME = "UltraPilot"


def _res(*parts):
    """Resource path that works from source and when frozen next to the exe."""
    base = os.path.dirname(sys.executable) if getattr(sys, "frozen", False) \
        else os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base, *parts)


ICON_PATH = _res("assets", "favicon.ico")
LOGO_PATH = _res("assets", "logo.png")

# --- Theme ------------------------------------------------------------------
DARK_QSS = """
QWizard, QWizardPage, QWidget { background-color: #0E0F13; color: #E6E6E6;
    font-family: 'Segoe UI', sans-serif; font-size: 14px; }
QLabel#Title { font-size: 26px; font-weight: 800; color: #00FF7F; }
QLabel#Subtitle { font-size: 14px; color: #9AA0A6; }
QLabel#Step { font-size: 16px; font-weight: 700; color: #00FF7F; }
QPushButton { background-color: #1B1D22; border: 1px solid #2C2F36; border-radius: 6px;
    padding: 8px 16px; color: #E6E6E6; }
QPushButton:hover { border-color: #00FF7F; color: #FFFFFF; }
QPushButton:disabled { color: #5A5F66; border-color: #1B1D22; }
QComboBox, QLineEdit { background-color: #16181D; border: 1px solid #2C2F36;
    border-radius: 6px; padding: 7px; }
QComboBox QAbstractItemView { background-color: #16181D; selection-background-color: #00663A; }
QTextEdit { background-color: #0A0B0E; color: #C8F7D6; border: 1px solid #2C2F36;
    border-radius: 6px; font-family: 'Consolas', monospace; font-size: 12px; }
QCheckBox { spacing: 8px; }
QProgressBar { background-color: #16181D; border: 1px solid #2C2F36; border-radius: 6px;
    height: 18px; text-align: center; color: #0E0F13; }
QProgressBar::chunk { background-color: #00FF7F; border-radius: 5px; }
"""

# --- Translations -----------------------------------------------------------
TR = {
    "English": {
        "win": "UltraPilot Setup",
        "welcome_t": "Welcome to UltraPilot",
        "welcome_s": "The advanced autopilot for Euro Truck Simulator 2.",
        "welcome_d": "This wizard will install UltraPilot, set up the SCS SDK plugins in your "
                     "game and create shortcuts. Lane keeping, adaptive cruise control, "
                     "collision avoidance, coordinate navigation, HUD and voice — included.",
        "lang": "Language / Jazyk:",
        "lic_t": "License Agreement",
        "lic_s": "Please accept the terms to continue.",
        "lic_text": "UltraPilot is provided for educational and entertainment use with "
                    "Euro Truck Simulator 2. Use it responsibly — keep your hands ready to "
                    "take over. The authors are not liable for any in-game incidents.",
        "lic_accept": "I accept the terms and conditions",
        "path_t": "Choose Install Location",
        "path_s": "Select where UltraPilot will be installed.",
        "path_lbl": "Installation folder:",
        "browse": "Browse…",
        "inst_t": "Installing UltraPilot",
        "inst_s": "Please wait while the components are set up.",
        "fin_t": "Installation Complete",
        "fin_s": "UltraPilot is ready to drive.",
        "fin_launch": "Launch UltraPilot now",
        "install_btn": "Install",
        # status lines
        "s_prep": "Preparing installation folder…",
        "s_copy": "Copying application files…",
        "s_copying": "Copying: {0}",
        "s_dll": "Installing SCS telemetry & controller plugins into the game…",
        "s_dll_ok": "Installed {0} → {1}",
        "s_dll_none": "No game found yet — DLLs will be installed on first launch.",
        "s_vigem": "Checking / installing ViGEmBus driver…",
        "s_short": "Creating Start-menu and desktop shortcuts…",
        "s_done": "All done! UltraPilot has been installed successfully.",
        "s_err": "Something went wrong: {0}",
    },
    "Slovenský": {
        "win": "Inštalácia UltraPilot",
        "welcome_t": "Vitajte v UltraPilot",
        "welcome_s": "Pokročilý autopilot pre Euro Truck Simulator 2.",
        "welcome_d": "Tento sprievodca nainštaluje UltraPilot, nastaví SCS SDK pluginy v hre a "
                     "vytvorí skratky. Udržiavanie pruhu, adaptívny tempomat, vyhýbanie sa "
                     "kolíziám, súradnicová navigácia, HUD a hlas — všetko v balíku.",
        "lang": "Jazyk / Language:",
        "lic_t": "Licenčná dohoda",
        "lic_s": "Pre pokračovanie prosím prijmite podmienky.",
        "lic_text": "UltraPilot je určený na vzdelávacie a zábavné použitie s hrou "
                    "Euro Truck Simulator 2. Používaj zodpovedne — maj ruky pripravené prevziať "
                    "riadenie. Autori nezodpovedajú za žiadne incidenty v hre.",
        "lic_accept": "Súhlasím s podmienkami",
        "path_t": "Vyber miesto inštalácie",
        "path_s": "Zvoľ, kam sa UltraPilot nainštaluje.",
        "path_lbl": "Inštalačný priečinok:",
        "browse": "Prehľadávať…",
        "inst_t": "Inštalujem UltraPilot",
        "inst_s": "Počkaj, prosím, kým sa komponenty nastavia.",
        "fin_t": "Inštalácia dokončená",
        "fin_s": "UltraPilot je pripravený jazdiť.",
        "fin_launch": "Spustiť UltraPilot teraz",
        "install_btn": "Inštalovať",
        "s_prep": "Pripravujem inštalačný priečinok…",
        "s_copy": "Kopírujem súbory aplikácie…",
        "s_copying": "Kopírujem: {0}",
        "s_dll": "Inštalujem SCS telemetriu a controller plugin do hry…",
        "s_dll_ok": "Nainštalované {0} → {1}",
        "s_dll_none": "Hra zatiaľ nenájdená — DLL sa nainštalujú pri prvom spustení.",
        "s_vigem": "Kontrolujem / inštalujem ViGEmBus ovládač…",
        "s_short": "Vytváram skratky v Štart menu a na ploche…",
        "s_done": "Hotovo! UltraPilot bol úspešne nainštalovaný.",
        "s_err": "Niečo sa pokazilo: {0}",
    },
}


# --- Install worker ---------------------------------------------------------
class InstallWorker(QThread):
    log = pyqtSignal(str)
    progress = pyqtSignal(int)
    finished_ok = pyqtSignal(bool, str)  # success, exe_path

    def __init__(self, install_path: str, lang: str):
        super().__init__()
        self.install_path = install_path
        self.t = TR[lang]

    # locate the app payload to install
    def _payload(self):
        """Return (source_dir, mode). mode: 'frozen' (UltraPilot.exe folder) or 'source'."""
        here = os.path.dirname(os.path.abspath(__file__)) if not getattr(sys, "frozen", False) \
            else os.path.dirname(sys.executable)
        candidates = []
        # 0) PyInstaller one-file: payload extracted next to the bundle.
        meipass = getattr(sys, "_MEIPASS", None)
        if meipass:
            candidates.append(os.path.join(meipass, "payload"))
        # 1) payload/ shipped next to the installer
        candidates.append(os.path.join(here, "payload"))
        # 2) a cx_Freeze build output (dev machine)
        candidates += [os.path.join(here, d) for d in (
            "build/exe.win-amd64-3.14", "build/exe.win-amd64-3.13",
            "build/exe.win-amd64-3.12")]
        for cand in candidates:
            if os.path.exists(os.path.join(cand, "UltraPilot.exe")):
                return cand, "frozen"
        # 3) source tree (run installed app with Python)
        return here, "source"

    def run(self):
        try:
            src, mode = self._payload()
            self.log.emit(self.t["s_prep"])
            os.makedirs(self.install_path, exist_ok=True)
            self.progress.emit(5)

            # --- copy files ---
            self.log.emit(self.t["s_copy"])
            if mode == "frozen":
                files = []
                for root, _dirs, fnames in os.walk(src):
                    for fn in fnames:
                        files.append(os.path.join(root, fn))
                total = max(1, len(files))
                for i, f in enumerate(files):
                    rel = os.path.relpath(f, src)
                    dst = os.path.join(self.install_path, rel)
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    shutil.copy2(f, dst)
                    if i % 25 == 0:
                        self.log.emit(self.t["s_copying"].format(rel))
                    self.progress.emit(5 + int(60 * (i + 1) / total))
                exe_path = os.path.join(self.install_path, "UltraPilot.exe")
            else:
                # Copy the source tree needed to run with Python.
                for item in ("core", "plugins", "sdk", "ui", "assets",
                             "main.py", "bootloader.py", "requirements.txt"):
                    s = os.path.join(src, item)
                    if not os.path.exists(s):
                        continue
                    self.log.emit(self.t["s_copying"].format(item))
                    d = os.path.join(self.install_path, item)
                    if os.path.isdir(s):
                        shutil.copytree(s, d, dirs_exist_ok=True,
                                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
                    else:
                        shutil.copy2(s, d)
                exe_path = os.path.join(self.install_path, "main.py")
                self.progress.emit(65)

            # --- DLLs into the game ---
            self.log.emit(self.t["s_dll"])
            try:
                # Imported from the installer's own bundled modules; the DLL
                # source is the freshly-copied install_path/assets folder.
                from core.sdk.game_utils import install_game_dlls
                assets = os.path.join(self.install_path, "assets")
                folders = install_game_dlls(assets)
                if folders:
                    for fld in folders:
                        self.log.emit(self.t["s_dll_ok"].format("SCS plugins", fld))
                else:
                    self.log.emit(self.t["s_dll_none"])
            except Exception as e:
                self.log.emit(f"  ({e})")
            self.progress.emit(80)

            # --- ViGEmBus ---
            self.log.emit(self.t["s_vigem"])
            try:
                from core.sdk.vigembus import ensure_vigembus
                ensure_vigembus(os.path.join(self.install_path, "assets"),
                                log=self.log.emit)
            except Exception as e:
                self.log.emit(f"  ({e})")
            self.progress.emit(90)

            # --- shortcuts ---
            self.log.emit(self.t["s_short"])
            self._make_shortcuts(exe_path, mode)
            self.progress.emit(100)

            self.log.emit("")
            self.log.emit("✔ " + self.t["s_done"])
            self.finished_ok.emit(True, exe_path)
        except Exception as e:
            self.log.emit(self.t["s_err"].format(e))
            self.finished_ok.emit(False, "")

    def _make_shortcuts(self, exe_path, mode):
        icon = os.path.join(self.install_path, "assets", "favicon.ico")
        if mode == "frozen":
            target, args, workdir = exe_path, "", self.install_path
        else:
            pyw = os.path.join(os.path.dirname(sys.executable), "pythonw.exe")
            target = pyw if os.path.exists(pyw) else "pythonw.exe"
            args = f'"{exe_path}"'
            workdir = self.install_path

        locations = {
            "Desktop": os.path.join(os.environ.get("USERPROFILE", ""), "Desktop"),
            "StartMenu": os.path.join(os.environ.get("APPDATA", ""),
                                      r"Microsoft\Windows\Start Menu\Programs"),
        }
        for _name, folder in locations.items():
            try:
                if not folder:
                    continue
                os.makedirs(folder, exist_ok=True)
                lnk = os.path.join(folder, f"{APP_NAME}.lnk")
                ps = (
                    f'$s=(New-Object -ComObject WScript.Shell).CreateShortcut("{lnk}");'
                    f'$s.TargetPath="{target}";'
                    + (f'$s.Arguments=\'{args}\';' if args else "")
                    + f'$s.WorkingDirectory="{workdir}";'
                    + (f'$s.IconLocation="{icon}";' if os.path.exists(icon) else "")
                    + '$s.Save()'
                )
                subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                               capture_output=True)
            except Exception as e:
                self.log.emit(f"  shortcut: {e}")


# --- Pages ------------------------------------------------------------------
class WelcomePage(QWizardPage):
    def __init__(self, wizard):
        super().__init__()
        self.wizard_ref = wizard
        lay = QVBoxLayout(self)
        lay.setContentsMargins(40, 30, 40, 30)
        lay.setSpacing(14)

        logo = QLabel()
        pm = QPixmap(LOGO_PATH)
        if pm.isNull():
            pm = QPixmap(ICON_PATH)
        if not pm.isNull():
            logo.setPixmap(pm.scaled(150, 150, Qt.AspectRatioMode.KeepAspectRatio,
                                     Qt.TransformationMode.SmoothTransformation))
        logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(logo)

        self.title_lbl = QLabel(); self.title_lbl.setObjectName("Title")
        self.title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.sub_lbl = QLabel(); self.sub_lbl.setObjectName("Subtitle")
        self.sub_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.desc_lbl = QLabel(); self.desc_lbl.setWordWrap(True)
        self.desc_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self.title_lbl)
        lay.addWidget(self.sub_lbl)
        lay.addWidget(self.desc_lbl)
        lay.addSpacing(10)

        row = QHBoxLayout()
        self.lang_lbl = QLabel()
        self.lang_combo = QComboBox()
        self.lang_combo.addItems(TR.keys())
        self.lang_combo.setCurrentText("Slovenský")
        self.lang_combo.currentTextChanged.connect(self.wizard_ref.set_language)
        row.addStretch(); row.addWidget(self.lang_lbl); row.addWidget(self.lang_combo); row.addStretch()
        lay.addLayout(row)
        lay.addStretch()

    def retranslate(self, t):
        self.title_lbl.setText(t["welcome_t"])
        self.sub_lbl.setText(t["welcome_s"])
        self.desc_lbl.setText(t["welcome_d"])
        self.lang_lbl.setText(t["lang"])


class LicensePage(QWizardPage):
    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(40, 30, 40, 30)
        self.step = QLabel(); self.step.setObjectName("Step")
        self.sub = QLabel(); self.sub.setObjectName("Subtitle")
        self.text = QTextEdit(); self.text.setReadOnly(True)
        self.chk = QCheckBox()
        self.chk.toggled.connect(self.completeChanged)
        lay.addWidget(self.step); lay.addWidget(self.sub)
        lay.addWidget(self.text); lay.addWidget(self.chk)

    def retranslate(self, t):
        self.step.setText(t["lic_t"]); self.sub.setText(t["lic_s"])
        self.text.setText(t["lic_text"]); self.chk.setText(t["lic_accept"])

    def isComplete(self):
        return self.chk.isChecked()


class PathPage(QWizardPage):
    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(40, 30, 40, 30)
        self.step = QLabel(); self.step.setObjectName("Step")
        self.sub = QLabel(); self.sub.setObjectName("Subtitle")
        self.lbl = QLabel()
        row = QHBoxLayout()
        self.edit = QLineEdit(os.path.join(os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "UltraPilot"))
        self.browse = QPushButton()
        self.browse.clicked.connect(self._browse)
        row.addWidget(self.edit); row.addWidget(self.browse)
        lay.addWidget(self.step); lay.addWidget(self.sub); lay.addSpacing(10)
        lay.addWidget(self.lbl); lay.addLayout(row); lay.addStretch()

    def _browse(self):
        p = QFileDialog.getExistingDirectory(self, "Select folder")
        if p:
            self.edit.setText(os.path.join(p, "UltraPilot"))

    def retranslate(self, t):
        self.step.setText(t["path_t"]); self.sub.setText(t["path_s"])
        self.lbl.setText(t["path_lbl"]); self.browse.setText(t["browse"])

    def path(self):
        return self.edit.text().strip()


class InstallPage(QWizardPage):
    def __init__(self, wizard):
        super().__init__()
        self.wizard_ref = wizard
        self._done = False
        self._started = False
        lay = QVBoxLayout(self)
        lay.setContentsMargins(40, 30, 40, 30)
        self.step = QLabel(); self.step.setObjectName("Step")
        self.sub = QLabel(); self.sub.setObjectName("Subtitle")
        self.bar = QProgressBar()
        self.logbox = QTextEdit(); self.logbox.setReadOnly(True)
        lay.addWidget(self.step); lay.addWidget(self.sub)
        lay.addWidget(self.bar); lay.addWidget(self.logbox)

    def retranslate(self, t):
        self.step.setText(t["inst_t"]); self.sub.setText(t["inst_s"])

    def initializePage(self):
        if self._started:
            return
        self._started = True
        t = self.wizard_ref.tr()
        self.wizard_ref.button(QWizard.WizardButton.BackButton).setEnabled(False)
        self.wizard_ref.button(QWizard.WizardButton.NextButton).setEnabled(False)
        self.worker = InstallWorker(self.wizard_ref.path_page.path(),
                                    self.wizard_ref.lang)
        self.worker.log.connect(self._log)
        self.worker.progress.connect(self.bar.setValue)
        self.worker.finished_ok.connect(self._finished)
        self.worker.start()

    def _log(self, msg):
        self.logbox.append(msg)
        self.logbox.ensureCursorVisible()

    def _finished(self, ok, exe_path):
        self._done = ok
        self.wizard_ref.exe_path = exe_path
        self.wizard_ref.button(QWizard.WizardButton.NextButton).setEnabled(True)
        self.completeChanged.emit()

    def isComplete(self):
        return self._done


class FinishPage(QWizardPage):
    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(40, 40, 40, 40)
        lay.setSpacing(16)
        self.icon = QLabel()
        pm = QPixmap(LOGO_PATH)
        if pm.isNull():
            pm = QPixmap(ICON_PATH)
        if not pm.isNull():
            self.icon.setPixmap(pm.scaled(110, 110, Qt.AspectRatioMode.KeepAspectRatio,
                                          Qt.TransformationMode.SmoothTransformation))
        self.icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_lbl = QLabel(); self.title_lbl.setObjectName("Title")
        self.title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.sub = QLabel(); self.sub.setObjectName("Subtitle")
        self.sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.launch = QCheckBox(); self.launch.setChecked(True)
        lay.addStretch(); lay.addWidget(self.icon); lay.addWidget(self.title_lbl)
        lay.addWidget(self.sub); lay.addWidget(self.launch, alignment=Qt.AlignmentFlag.AlignCenter)
        lay.addStretch()

    def retranslate(self, t):
        self.title_lbl.setText(t["fin_t"]); self.sub.setText(t["fin_s"])
        self.launch.setText(t["fin_launch"])


# --- Wizard -----------------------------------------------------------------
class Installer(QWizard):
    def __init__(self):
        super().__init__()
        self.lang = "Slovenský"
        self.exe_path = ""
        self.setWizardStyle(QWizard.WizardStyle.ModernStyle)
        self.setOption(QWizard.WizardOption.NoBackButtonOnStartPage, True)
        self.setStyleSheet(DARK_QSS)
        if os.path.exists(ICON_PATH):
            self.setWindowIcon(QIcon(ICON_PATH))
        self.setFixedSize(640, 560)

        self.welcome = WelcomePage(self)
        self.license = LicensePage()
        self.path_page = PathPage()
        self.install_page = InstallPage(self)
        self.finish = FinishPage()
        for p in (self.welcome, self.license, self.path_page, self.install_page, self.finish):
            self.addPage(p)

        self.button(QWizard.WizardButton.CommitButton).setText(TR[self.lang]["install_btn"])
        self.set_language(self.lang)

    def tr(self):
        return TR[self.lang]

    def set_language(self, lang):
        self.lang = lang
        t = TR[lang]
        self.setWindowTitle(t["win"])
        for p in (self.welcome, self.license, self.path_page, self.install_page, self.finish):
            p.retranslate(t)
        self.setButtonText(QWizard.WizardButton.NextButton,
                           {"English": "Next ›", "Slovenský": "Ďalej ›"}[lang])
        self.setButtonText(QWizard.WizardButton.BackButton,
                           {"English": "‹ Back", "Slovenský": "‹ Späť"}[lang])
        self.setButtonText(QWizard.WizardButton.FinishButton,
                           {"English": "Finish", "Slovenský": "Dokončiť"}[lang])
        self.setButtonText(QWizard.WizardButton.CancelButton,
                           {"English": "Cancel", "Slovenský": "Zrušiť"}[lang])

    def done(self, result):
        # Launch the app if requested on the finish page.
        if result == QWizard.DialogCode.Accepted and self.finish.launch.isChecked() and self.exe_path:
            try:
                if self.exe_path.endswith(".py"):
                    pyw = os.path.join(os.path.dirname(sys.executable), "pythonw.exe")
                    subprocess.Popen([pyw if os.path.exists(pyw) else "pythonw", self.exe_path],
                                     cwd=os.path.dirname(self.exe_path))
                else:
                    subprocess.Popen([self.exe_path], cwd=os.path.dirname(self.exe_path))
            except Exception:
                pass
        super().done(result)


def main():
    app = QApplication(sys.argv)
    w = Installer()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
