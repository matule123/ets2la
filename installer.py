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

# --- Themes (light is the default) ------------------------------------------
LIGHT_QSS = """
QWizard, QWizardPage, QWidget { background-color: #FFFFFF; color: #1A1D21;
    font-family: 'Segoe UI', sans-serif; font-size: 14px; }
QLabel#Title { font-size: 26px; font-weight: 800; color: #065F46; }
QLabel#Subtitle { font-size: 14px; color: #6B7280; }
QLabel#Step { font-size: 16px; font-weight: 700; color: #065F46; }
QPushButton { background-color: #F3F4F6; border: 1px solid #DfE3E8; border-radius: 8px;
    padding: 8px 16px; color: #1A1D21; }
QPushButton:hover { border-color: #10B981; color: #065F46; }
QPushButton:disabled { color: #9CA3AF; border-color: #EEF1F4; }
QComboBox, QLineEdit { background-color: #FFFFFF; border: 1px solid #DfE3E8;
    border-radius: 8px; padding: 7px; }
QComboBox QAbstractItemView { background-color: #FFFFFF; selection-background-color: #10B981; }
QTextEdit { background-color: #0A0B0E; color: #C8F7D6; border: 1px solid #2C2F36;
    border-radius: 8px; font-family: 'Consolas', monospace; font-size: 12px; }
QCheckBox { spacing: 8px; }
QProgressBar { background-color: #EEF1F4; border: 1px solid #DfE3E8; border-radius: 6px;
    height: 18px; text-align: center; color: #1A1D21; }
QProgressBar::chunk { background-color: #10B981; border-radius: 5px; }
"""

DARK_QSS = """
QWizard, QWizardPage, QWidget { background-color: #0E0F13; color: #E6E6E6;
    font-family: 'Segoe UI', sans-serif; font-size: 14px; }
QLabel#Title { font-size: 26px; font-weight: 800; color: #34D399; }
QLabel#Subtitle { font-size: 14px; color: #9AA0A6; }
QLabel#Step { font-size: 16px; font-weight: 700; color: #34D399; }
QPushButton { background-color: #1B1D22; border: 1px solid #2C2F36; border-radius: 8px;
    padding: 8px 16px; color: #E6E6E6; }
QPushButton:hover { border-color: #10B981; color: #FFFFFF; }
QPushButton:disabled { color: #5A5F66; border-color: #1B1D22; }
QComboBox, QLineEdit { background-color: #16181D; border: 1px solid #2C2F36;
    border-radius: 8px; padding: 7px; }
QComboBox QAbstractItemView { background-color: #16181D; selection-background-color: #00663A; }
QTextEdit { background-color: #0A0B0E; color: #C8F7D6; border: 1px solid #2C2F36;
    border-radius: 8px; font-family: 'Consolas', monospace; font-size: 12px; }
QCheckBox { spacing: 8px; }
QProgressBar { background-color: #16181D; border: 1px solid #2C2F36; border-radius: 6px;
    height: 18px; text-align: center; color: #0E0F13; }
QProgressBar::chunk { background-color: #10B981; border-radius: 5px; }
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
        "lic_text": (
            "ULTRAPILOT — END USER LICENSE AGREEMENT\n\n"
            "1. PURPOSE. UltraPilot is a driver-assistance and automation tool intended "
            "solely for educational and entertainment use within the video game Euro "
            "Truck Simulator 2 / American Truck Simulator. It is not intended for, and "
            "must not be used in, any real-world vehicle or safety-critical system.\n\n"
            "2. RESPONSIBLE USE. You remain fully responsible for supervising the "
            "software at all times. Keep your hands ready to take over and disable the "
            "autopilot whenever necessary. The software may behave unpredictably.\n\n"
            "3. NO WARRANTY. The software is provided \"AS IS\", without warranty of any "
            "kind, express or implied, including but not limited to the warranties of "
            "merchantability and fitness for a particular purpose.\n\n"
            "4. LIMITATION OF LIABILITY. In no event shall the authors be liable for any "
            "in-game incidents, data loss, or any direct, indirect or consequential "
            "damages arising from the use of this software.\n\n"
            "5. THIRD-PARTY COMPONENTS. This installer may set up third-party drivers "
            "(e.g. ViGEmBus) and game SDK plugins, each subject to their own licenses.\n\n"
            "By installing UltraPilot you acknowledge that you have read and agree to "
            "these terms."
        ),
        "lic_accept": "I have read and accept the terms and conditions",
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
        "lic_text": (
            "ULTRAPILOT — LICENČNÁ ZMLUVA S KONCOVÝM POUŽÍVATEĽOM\n\n"
            "1. ÚČEL. UltraPilot je nástroj na asistenciu a automatizáciu jazdy určený "
            "výhradne na vzdelávacie a zábavné použitie v hre Euro Truck Simulator 2 / "
            "American Truck Simulator. Nie je určený a nesmie byť použitý v žiadnom "
            "skutočnom vozidle ani bezpečnostne kritickom systéme.\n\n"
            "2. ZODPOVEDNÉ POUŽÍVANIE. Za dohľad nad softvérom nesieš plnú zodpovednosť "
            "po celý čas. Maj ruky pripravené prevziať riadenie a autopilota kedykoľvek "
            "vypni, ak je to potrebné. Softvér sa môže správať nepredvídateľne.\n\n"
            "3. BEZ ZÁRUKY. Softvér je poskytovaný „TAK AKO JE\", bez akýchkoľvek záruk, "
            "výslovných či implicitných, vrátane záruk predajnosti a vhodnosti na "
            "konkrétny účel.\n\n"
            "4. OBMEDZENIE ZODPOVEDNOSTI. Autori v žiadnom prípade nezodpovedajú za "
            "žiadne incidenty v hre, stratu dát ani žiadne priame, nepriame či následné "
            "škody vyplývajúce z používania tohto softvéru.\n\n"
            "5. KOMPONENTY TRETÍCH STRÁN. Tento inštalátor môže nainštalovať ovládače "
            "tretích strán (napr. ViGEmBus) a herné SDK pluginy s vlastnými licenciami.\n\n"
            "Inštaláciou UltraPilot potvrdzuješ, že si si tieto podmienky prečítal a "
            "súhlasíš s nimi."
        ),
        "lic_accept": "Prečítal som si podmienky a súhlasím s nimi",
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
    "Čeština": {
        "win": "Instalace UltraPilot", "welcome_t": "Vítejte v UltraPilot",
        "welcome_s": "Pokročilý autopilot pro Euro Truck Simulator 2.",
        "welcome_d": "Průvodce nainstaluje UltraPilot, nastaví SDK pluginy ve hře a vytvoří zástupce.",
        "lang": "Jazyk / Language:", "lic_t": "Licenční smlouva",
        "lic_s": "Pro pokračování přijměte podmínky.",
        "lic_accept": "Přečetl jsem si podmínky a souhlasím",
        "path_t": "Vyber místo instalace", "path_s": "Zvol, kam se UltraPilot nainstaluje.",
        "path_lbl": "Instalační složka:", "browse": "Procházet…",
        "inst_t": "Instaluji UltraPilot", "inst_s": "Počkej, než se komponenty nastaví.",
        "fin_t": "Instalace dokončena", "fin_s": "UltraPilot je připraven.",
        "fin_launch": "Spustit UltraPilot", "install_btn": "Instalovat",
    },
    "Deutsch": {
        "win": "UltraPilot Setup", "welcome_t": "Willkommen bei UltraPilot",
        "welcome_s": "Der fortschrittliche Autopilot für Euro Truck Simulator 2.",
        "welcome_d": "Dieser Assistent installiert UltraPilot, richtet die SDK-Plugins im Spiel ein und erstellt Verknüpfungen.",
        "lang": "Sprache / Language:", "lic_t": "Lizenzvereinbarung",
        "lic_s": "Bitte akzeptieren Sie die Bedingungen.",
        "lic_accept": "Ich habe die Bedingungen gelesen und akzeptiere sie",
        "path_t": "Installationsort wählen", "path_s": "Wählen Sie, wohin UltraPilot installiert wird.",
        "path_lbl": "Installationsordner:", "browse": "Durchsuchen…",
        "inst_t": "UltraPilot wird installiert", "inst_s": "Bitte warten…",
        "fin_t": "Installation abgeschlossen", "fin_s": "UltraPilot ist bereit.",
        "fin_launch": "UltraPilot jetzt starten", "install_btn": "Installieren",
    },
    "Polski": {
        "win": "Instalator UltraPilot", "welcome_t": "Witaj w UltraPilot",
        "welcome_s": "Zaawansowany autopilot do Euro Truck Simulator 2.",
        "welcome_d": "Kreator zainstaluje UltraPilot, skonfiguruje wtyczki SDK w grze i utworzy skróty.",
        "lang": "Język / Language:", "lic_t": "Umowa licencyjna",
        "lic_s": "Zaakceptuj warunki, aby kontynuować.",
        "lic_accept": "Przeczytałem i akceptuję warunki",
        "path_t": "Wybierz lokalizację", "path_s": "Wybierz, gdzie zainstalować UltraPilot.",
        "path_lbl": "Folder instalacji:", "browse": "Przeglądaj…",
        "inst_t": "Instalowanie UltraPilot", "inst_s": "Proszę czekać…",
        "fin_t": "Instalacja zakończona", "fin_s": "UltraPilot jest gotowy.",
        "fin_launch": "Uruchom UltraPilot", "install_btn": "Instaluj",
    },
}

# Translation coverage (% of English keys present).
_EN_KEYS = set(TR["English"].keys())
def tr_coverage(lang):
    keys = set(TR.get(lang, {}).keys())
    return round(100 * len(keys & _EN_KEYS) / len(_EN_KEYS)) if _EN_KEYS else 100

def tr_get(lang, key):
    return TR.get(lang, {}).get(key) or TR["English"].get(key, key)


# --- Install worker ---------------------------------------------------------
class InstallWorker(QThread):
    log = pyqtSignal(str)
    progress = pyqtSignal(int)
    finished_ok = pyqtSignal(bool, str)  # success, exe_path

    def __init__(self, install_path: str, lang: str):
        super().__init__()
        self.install_path = install_path
        self.t = {**TR["English"], **TR.get(lang, {})}  # English fallback for missing keys

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
            mode = "source"   # installed app runs from source via Python
            self.log.emit(self.t["s_prep"])
            os.makedirs(self.install_path, exist_ok=True)
            self.progress.emit(3)

            # --- get the app ---
            # Prefer the files bundled inside the installer (offline, fast,
            # always available). Only fall back to a GitHub download if the
            # bundle is somehow incomplete (e.g. a slim build).
            self.log.emit("Copying UltraPilot files…")
            self._copy_bundled()
            if not os.path.exists(os.path.join(self.install_path, "main.py")):
                self.log.emit("  Bundled files incomplete — downloading from GitHub…")
                if not self._fetch_repo():
                    raise RuntimeError("Could not obtain UltraPilot files "
                                       "(no bundle and GitHub unreachable).")
            self.progress.emit(45)

            # --- install Python dependencies (incl. the 3D libraries) ---
            self.log.emit("Installing Python dependencies (this can take a few minutes)…")
            self._pip_install()
            exe_path = os.path.join(self.install_path, "main.py")
            self.progress.emit(75)

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
            _write_record(self.install_path, exe_path, mode)
            self.progress.emit(100)

            self.log.emit("")
            self.log.emit("✔ " + self.t["s_done"])
            self.finished_ok.emit(True, exe_path)
        except Exception as e:
            self.log.emit(self.t["s_err"].format(e))
            self.finished_ok.emit(False, "")

    # --- fetching / dependencies ---------------------------------------------
    REPO_GIT = "https://github.com/matule123/ets2la.git"
    REPO_ZIP = "https://github.com/matule123/ets2la/archive/refs/heads/main.zip"

    def _fetch_repo(self):
        """Clone the repo (or download the zip) into install_path. True on success."""
        # 1) git clone if git is available
        try:
            tmp = self.install_path + "_clone"
            if os.path.isdir(tmp):
                shutil.rmtree(tmp, ignore_errors=True)
            r = subprocess.run(["git", "clone", "--depth", "1", self.REPO_GIT, tmp],
                               capture_output=True, text=True, timeout=600)
            if r.returncode == 0 and os.path.exists(os.path.join(tmp, "main.py")):
                for item in os.listdir(tmp):
                    s, d = os.path.join(tmp, item), os.path.join(self.install_path, item)
                    if os.path.isdir(s):
                        shutil.copytree(s, d, dirs_exist_ok=True)
                    else:
                        shutil.copy2(s, d)
                shutil.rmtree(tmp, ignore_errors=True)
                self.log.emit("  Cloned from GitHub.")
                return True
        except Exception as e:
            self.log.emit(f"  git clone unavailable ({e}).")

        # 2) download the zip
        try:
            import requests, zipfile, io
            self.log.emit("  Downloading source zip…")
            resp = requests.get(self.REPO_ZIP, timeout=120)
            if resp.status_code == 200:
                zf = zipfile.ZipFile(io.BytesIO(resp.content))
                zf.extractall(self.install_path)
                # the zip extracts to a 'ets2la-main' subfolder — flatten it
                root = os.path.join(self.install_path, "ets2la-main")
                if os.path.isdir(root):
                    for item in os.listdir(root):
                        shutil.move(os.path.join(root, item),
                                    os.path.join(self.install_path, item))
                    shutil.rmtree(root, ignore_errors=True)
                self.log.emit("  Downloaded and extracted.")
                return True
        except Exception as e:
            self.log.emit(f"  Zip download failed ({e}).")
        return False

    def _copy_bundled(self):
        """Fallback: copy whatever ships next to the installer (at least assets)."""
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

    def _real_python(self):
        """Find the real Python interpreter to run pip with.

        In a frozen installer ``sys.executable`` is UltraPilot_Installer.exe
        itself, so `pip install` would re-launch the installer. We look up the
        real interpreter on PATH (python / py launcher) instead. Returns the
        path, or '' if none is found.
        """
        # Prefer the version-agnostic `py` launcher (ships with official Python).
        py = shutil.which("py") or shutil.which("py.exe")
        if py:
            return [py, "-3"]
        # Otherwise a plain python on PATH.
        for name in ("python", "python.exe", "python3", "python3.exe"):
            found = shutil.which(name)
            if found:
                return [found]
        return []

    def _pip_install(self):
        req = os.path.join(self.install_path, "requirements.txt")
        pkgs = ["pyqtgraph", "PyOpenGL"]
        py = self._real_python()
        if not py:
            self.log.emit("  Python not found on PATH — install Python 3, "
                          "then run: pip install -r requirements.txt")
            return
        try:
            self.log.emit(f"  Using Python: {py[0]}")
            if os.path.exists(req):
                subprocess.run([*py, "-m", "pip", "install", "-r", req],
                               capture_output=True, timeout=1800)
            subprocess.run([*py, "-m", "pip", "install", *pkgs],
                           capture_output=True, timeout=600)
            self.log.emit("  Dependencies installed.")
        except Exception as e:
            self.log.emit(f"  pip install issue ({e}) — install manually if needed.")

    def _make_shortcuts(self, exe_path, mode):
        icon = os.path.join(self.install_path, "assets", "favicon.ico")
        if mode == "frozen":
            target, args, workdir = exe_path, "", self.install_path
        else:
            # The installer is itself a frozen exe, so sys.executable is NOT the
            # Python interpreter — find the real pythonw on PATH (or a launcher).
            pyw = (shutil.which("pythonw") or shutil.which("pythonw.exe")
                   or shutil.which("python") or "pythonw.exe")
            target = pyw
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
        pm = QPixmap(ICON_PATH)          # symbol only (no wordmark)
        if pm.isNull():
            pm = QPixmap(LOGO_PATH)
        if not pm.isNull():
            logo.setPixmap(pm.scaled(120, 120, Qt.AspectRatioMode.KeepAspectRatio,
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
        pm = QPixmap(ICON_PATH)          # symbol only (no wordmark)
        if pm.isNull():
            pm = QPixmap(LOGO_PATH)
        if not pm.isNull():
            self.icon.setPixmap(pm.scaled(96, 96, Qt.AspectRatioMode.KeepAspectRatio,
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
        self.theme = "dark"    # default black background
        self.setWizardStyle(QWizard.WizardStyle.ModernStyle)
        self.setOption(QWizard.WizardOption.NoBackButtonOnStartPage, True)
        self.apply_theme()
        if os.path.exists(ICON_PATH):
            self.setWindowIcon(QIcon(ICON_PATH))
        self.setFixedSize(640, 560)

        # Sun / Moon theme toggle as a real wizard button (always visible in the
        # button row; a free-floating child gets hidden behind the page).
        self.setOption(QWizard.WizardOption.HaveCustomButton1, True)
        self.setButtonText(QWizard.WizardButton.CustomButton1, "☀️")
        self.customButtonClicked.connect(self._on_custom)

    def _on_custom(self, which):
        if which == QWizard.WizardButton.CustomButton1:
            self.toggle_theme()

        self.welcome = WelcomePage(self)
        self.license = LicensePage()
        self.path_page = PathPage()
        self.install_page = InstallPage(self)
        self.finish = FinishPage()
        for p in (self.welcome, self.license, self.path_page, self.install_page, self.finish):
            self.addPage(p)

        self.button(QWizard.WizardButton.CommitButton).setText(self.tr()["install_btn"])
        self.set_language(self.lang)

    def apply_theme(self):
        self.setStyleSheet(LIGHT_QSS if self.theme == "light" else DARK_QSS)

    def toggle_theme(self):
        self.theme = "dark" if self.theme == "light" else "light"
        self.apply_theme()
        self.setButtonText(QWizard.WizardButton.CustomButton1,
                           "☀️" if self.theme == "dark" else "🌙")

    def tr(self):
        return {**TR["English"], **TR.get(self.lang, {})}

    def set_language(self, lang):
        self.lang = lang
        t = {**TR["English"], **TR.get(lang, {})}  # English fallback for missing keys
        self.setWindowTitle(t["win"])
        for p in (self.welcome, self.license, self.path_page, self.install_page, self.finish):
            p.retranslate(t)
        nav = {
            "English": ("Next ›", "‹ Back", "Finish", "Cancel"),
            "Slovenský": ("Ďalej ›", "‹ Späť", "Dokončiť", "Zrušiť"),
            "Čeština": ("Další ›", "‹ Zpět", "Dokončit", "Zrušit"),
            "Deutsch": ("Weiter ›", "‹ Zurück", "Fertig", "Abbrechen"),
            "Polski": ("Dalej ›", "‹ Wstecz", "Zakończ", "Anuluj"),
        }.get(lang, ("Next ›", "‹ Back", "Finish", "Cancel"))
        self.setButtonText(QWizard.WizardButton.NextButton, nav[0])
        self.setButtonText(QWizard.WizardButton.BackButton, nav[1])
        self.setButtonText(QWizard.WizardButton.FinishButton, nav[2])
        self.setButtonText(QWizard.WizardButton.CancelButton, nav[3])

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


RECORD_PATH = os.path.join(os.environ.get("LOCALAPPDATA", os.path.expanduser("~")),
                           "UltraPilot", ".install.json")


def _write_record(install_path, exe_path, mode):
    import json
    try:
        os.makedirs(os.path.dirname(RECORD_PATH), exist_ok=True)
        with open(RECORD_PATH, "w") as f:
            json.dump({"install_path": install_path, "exe_path": exe_path, "mode": mode}, f)
    except Exception:
        pass


def _read_record():
    import json
    try:
        with open(RECORD_PATH) as f:
            rec = json.load(f)
        if rec.get("install_path") and os.path.isdir(rec["install_path"]):
            return rec
    except Exception:
        pass
    return None


def _uninstall(rec):
    """Remove the installed app folder and shortcuts."""
    import shutil
    try:
        shutil.rmtree(rec["install_path"], ignore_errors=True)
    except Exception:
        pass
    for folder in (os.path.join(os.environ.get("USERPROFILE", ""), "Desktop"),
                   os.path.join(os.environ.get("APPDATA", ""),
                                r"Microsoft\Windows\Start Menu\Programs")):
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
    """Already installed -> offer Repair / Uninstall / Cancel. Returns action."""
    from PyQt6.QtWidgets import QMessageBox
    box = QMessageBox()
    box.setWindowTitle("UltraPilot")
    if os.path.exists(ICON_PATH):
        box.setWindowIcon(QIcon(ICON_PATH))
    box.setStyleSheet(DARK_QSS)
    box.setText("UltraPilot je už nainštalovaný.\nČo chceš spraviť?")
    repair = box.addButton("Opraviť", QMessageBox.ButtonRole.AcceptRole)
    uninstall = box.addButton("Odinštalovať", QMessageBox.ButtonRole.DestructiveRole)
    box.addButton("Zrušiť", QMessageBox.ButtonRole.RejectRole)
    box.exec()
    clicked = box.clickedButton()
    if clicked == uninstall:
        return "uninstall"
    if clicked == repair:
        return "repair"
    return "cancel"


def main():
    app = QApplication(sys.argv)
    # If already installed, show the maintenance options first.
    rec = _read_record()
    if rec is not None:
        action = _maintenance_dialog(rec)
        if action == "uninstall":
            _uninstall(rec)
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.information(None, "UltraPilot", "UltraPilot bol odinštalovaný.")
            return
        elif action == "cancel":
            return
        # "repair" falls through to a normal (re)install over the same folder.
    w = Installer()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
