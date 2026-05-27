import os
import sys
import shutil
import subprocess
from pathlib import Path
from PyQt6.QtWidgets import (
    QApplication, QWizard, QWizardPage, QVBoxLayout, QLabel,
    QPushButton, QProgressBar, QTextEdit, QFileDialog, QComboBox, QCheckBox
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtGui import QPixmap, QIcon

# Try to import game_utils from the bundled core
try:
    from core.sdk import game_utils
except ImportError:
    # Fallback if we are running from source and haven't installed dependencies
    game_utils = None

# Configuration
APP_NAME = "ETS2 Lane Assist"
ICON_PATH = "assets/favicon.ico"

TRANSLATIONS = {
    "English": {
        "welcome_title": "Welcome to ETS2LA Setup",
        "welcome_subtitle": "This wizard will guide you through the installation of the Lane Assist for Euro Truck Simulator 2.",
        "welcome_desc": "ETS2LA provides advanced lane assist and a professional HUD for a more realistic trucking experience.",
        "lang_title": "Select Language",
        "lang_subtitle": "Please choose your preferred language for the installation.",
        "path_title": "Installation Folder",
        "path_subtitle": "Choose where you want to install ETS2LA.",
        "path_label": "Installation directory:",
        "terms_title": "License Agreement",
        "terms_subtitle": "Please accept the terms to continue.",
        "terms_text": "By installing ETS2LA, you agree to use this software for educational and entertainment purposes. We are not responsible for any virtual crashes.",
        "terms_checkbox": "I accept the terms and conditions",
        "setup_title": "Extracting Files",
        "setup_sub": "Unpacking application files to the target folder...",
        "sdk_title": "Game SDK Setup",
        "sdk_sub": "Installing required plugins to your ETS2/ATS directory...",
        "shortcut_title": "Finalizing",
        "shortcut_sub": "Creating desktop shortcut...",
        "success_msg": "\n[SUCCESS] Step completed successfully!",
        "error_msg": "\n[ERROR] Step failed. Please check the logs above.",
    },
    "Slovenský": {
        "welcome_title": "Vitajte v ETS2LA Setup",
        "welcome_subtitle": "Tento sprievodca vás prevedie inštaláciou Lane Assist pre Euro Truck Simulator 2.",
        "welcome_desc": "ETS2LA poskytuje pokročilú asistenciu pri udržiavaní jazdných pruhov a profesionálne HUD pre realistickejší zážitok.",
        "lang_title": "Vyberte jazyk",
        "lang_subtitle": "Prosím, vyberte preferovaný jazyk pre inštaláciu.",
        "path_title": "Priečinok inštalácie",
        "path_subtitle": "Vyberte, kam chcete nainštalovať ETS2LA.",
        "path_label": "Inštalačný priečinok:",
        "terms_title": "Licenčná dohoda",
        "terms_subtitle": "Pre pokračovanie prosím prijmite podmienky.",
        "terms_text": "Inštaláciou ETS2LA súhlasíte, že budete používať tento softvér na vzdelávacie a zábavné účely. Nie sme zodpovedné za žiadne virtuálne nehody.",
        "terms_checkbox": "Prijímam podmienky používania",
        "setup_title": "Extrakcia súborov",
        "setup_sub": "Rozbalovanie súborov aplikácie do cieľového priečinka...",
        "sdk_title": "Nastavenie Game SDK",
        "sdk_sub": "Inštalácia potrebných pluginov do priečinka ETS2/ATS...",
        "shortcut_title": "Finalizácia",
        "shortcut_sub": "Vytváranie ikony na ploche...",
        "success_msg": "\n[ÚCPECH] Krok bol úspešne dokončený!",
        "error_msg": "\n[CHYBA] Krok zlyhal. Pozrite si prosím logy vyššie.",
    }
}

class Worker(QThread):
    progress = pyqtSignal(int)
    log = pyqtSignal(str)
    finished = pyqtSignal(bool)

    def __init__(self, func, *args, **kwargs):
        super().__init__()
        self.func = func
        self.args = args
        self.kwargs = kwargs

    def run(self):
        try:
            result = self.func(*self.args, **self.kwargs, worker=self)
            self.finished.emit(result)
        except Exception as e:
            self.log.emit(f"Critical error: {str(e)}")
            self.finished.emit(False)

class InstallWorker:
    def __init__(self, worker, install_path):
        self.worker = worker
        self.install_path = install_path

    def run_command(self, command, cwd=None):
        try:
            result = subprocess.run(command, shell=True, capture_output=True, text=True, cwd=cwd)
            if result.returncode != 0:
                self.worker.log.emit(f"Command failed: {command}\nError: {result.stderr}")
                return False
            return True
        except Exception as e:
            self.worker.log.emit(f"Execution error: {str(e)}")
            return False

    def extract_files(self, worker):
        worker.log.emit("Extracting application files...")
        try:
            # PyInstaller extracts bundled data to _MEIPASS
            if hasattr(sys, '_MEIPASS'):
                src_path = sys._MEIPASS
            else:
                src_path = os.path.abspath(os.path.dirname(__file__))

            # We want to copy everything EXCEPT the installer itself and build artifacts
            exclude_dirs = {'build', 'dist', '__pycache__'}
            exclude_files = {'installer_pro.py', 'build_installer_pro.py', 'ets2la_installer.exe'}

            if not os.path.exists(self.install_path):
                os.makedirs(self.install_path, exist_ok=True)

            for item in os.listdir(src_path):
                if item in exclude_files or item in exclude_dirs:
                    continue

                s = os.path.join(src_path, item)
                d = os.path.join(self.install_path, item)

                if os.path.isdir(s):
                    if os.path.exists(d):
                        shutil.rmtree(d)
                    shutil.copytree(s, d)
                else:
                    shutil.copy2(s, d)
                worker.log.emit(f"Copied {item}...")

            return True
        except Exception as e:
            worker.log.emit(f"Extraction error: {str(e)}")
            return False

    def install_game_sdk(self, worker):
        if game_utils is None:
            worker.log.emit("Game utils not found. Skipping SDK installation.")
            return False

        worker.log.emit("Searching for installed SCS games...")
        games = game_utils.find_scs_games()
        if not games:
            worker.log.emit("No SCS games found. Please install ETS2 or ATS.")
            return False

        for game_path in games:
            version = game_utils.get_version_for_game(game_path)
            worker.log.emit(f"Found game at {game_path} (Version: {version})")

            # Look for SDK files in the extracted installation path
            sdk_version_path = os.path.join(self.install_path, "sdk", version)
            if not os.path.exists(sdk_version_path):
                worker.log.emit(f"No SDK files found for version {version} in {sdk_version_path}. Skipping...")
                continue

            target_path = os.path.join(game_path, "bin", "win_x64", "plugins")
            os.makedirs(target_path, exist_ok=True)
            try:
                for file in os.listdir(sdk_version_path):
                    src = os.path.join(sdk_version_path, file)
                    dst = os.path.join(target_path, file)
                    shutil.copy2(src, dst)
                    worker.log.emit(f"Installed {file} to {target_path}")
            except Exception as e:
                worker.log.emit(f"Error copying files to {game_path}: {e}")
                return False
        return True

    def create_desktop_shortcut(self, worker):
        worker.log.emit("Creating desktop shortcut...")
        try:
            # The app executable will be built by PyInstaller in the final package
            # For the installer, we assume the main app is called APP_NAME.exe in the installation folder
            target_exe = os.path.join(self.install_path, f"{APP_NAME}.exe")
            if not os.path.exists(target_exe):
                # If not already built, we might need to warn, but usually the app is already packaged
                worker.log.emit("Main executable not found in installation path. Shortcut might not work.")

            desktop = Path(os.path.join(os.environ['USERPROFILE'], 'Desktop'))
            shortcut_path = os.path.join(desktop, f"{APP_NAME}.lnk")
            ps_cmd = f'$s=(New-Object -ComObject WScript.Shell).CreateShortcut("{shortcut_path}");$s.TargetPath="{target_exe}";$s.Save()'
            return self.run_command(f"powershell -Command {ps_cmd}")
        except Exception as e:
            worker.log.emit(f"Shortcut error: {e}")
            return False

class LanguagePage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Select Language")
        self.setSubTitle("Please choose your preferred language for the installation.")
        layout = QVBoxLayout()
        self.lang_combo = QComboBox()
        self.lang_combo.addItems(TRANSLATIONS.keys())
        self.lang_combo.setCurrentText("Slovenský")
        layout.addWidget(QLabel("Language / Jazyk:"))
        layout.addWidget(self.lang_combo)
        layout.addStretch()
        self.setLayout(layout)
    def get_language(self):
        return self.lang_combo.currentText()

class PathPage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Installation Folder")
        self.setSubTitle("Choose where you want to install ETS2LA.")
        layout = QVBoxLayout()
        self.path_edit = QTextEdit()
        self.path_edit.setReadOnly(True)
        self.path_edit.setMaximumHeight(30)
        self.path_edit.setText(os.path.join(os.environ['USERPROFILE'], 'Documents', 'ETS2LA'))
        self.browse_btn = QPushButton("Browse...")
        self.browse_btn.clicked.connect(self.browse)
        layout.addWidget(QLabel("Installation directory:"))
        layout.addWidget(self.path_edit)
        layout.addWidget(self.browse_btn)
        layout.addStretch()
        self.setLayout(layout)
    def browse(self):
        path = QFileDialog.getExistingDirectory(self, "Select Installation Folder")
        if path:
            self.path_edit.setText(path)
    def get_path(self):
        return self.path_edit.toPlainText().strip()

class TermsPage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("License Agreement")
        self.setSubTitle("Please accept the terms to continue.")
        layout = QVBoxLayout()
        self.terms_text = QTextEdit()
        self.terms_text.setReadOnly(True)
        self.terms_text.setText("By installing ETS2LA, you agree to use this software for educational and entertainment purposes. We are not responsible for any virtual crashes.")
        layout.addWidget(self.terms_text)
        self.accept_checkbox = QCheckBox("I accept the terms and conditions")
        layout.addWidget(self.accept_checkbox)
        self.setLayout(layout)
    def isComplete(self):
        return self.accept_checkbox.isChecked()

class WelcomePage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout()
        self.logo_label = QLabel()
        pixmap = QPixmap(ICON_PATH)
        if not pixmap.isNull():
            self.logo_label.setPixmap(pixmap.scaled(128, 128, Qt.AspectRatioMode.KeepAspectRatio))
        self.logo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.logo_label)
        self.desc = QLabel("")
        self.desc.setWordWrap(True)
        self.desc.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.desc)
        self.setLayout(layout)
    def update_text(self, lang):
        t = TRANSLATIONS[lang]
        self.setTitle(t["welcome_title"])
        self.setSubTitle(t["welcome_subtitle"])
        self.desc.setText(t["welcome_desc"])

class ProcessPage(QWizardPage):
    def __init__(self, title, subtext, func_name, parent=None):
        super().__init__(parent)
        self.setTitle(title)
        self.setSubTitle(subtext)
        self.func_name = func_name
        layout = QVBoxLayout()
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: Consolas, monospace;")
        layout.addWidget(self.log_area)
        self.progress_bar = QProgressBar()
        layout.addWidget(self.progress_bar)
        self.setLayout(layout)
    def update_log(self, text):
        self.log_area.append(text)
        self.log_area.ensureCursorVisible()
    def update_progress(self, val):
        self.progress_bar.setValue(val)

class ETS2LAInstaller(QWizard):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME} Setup")
        self.setFixedSize(600, 500)
        self.setWindowIcon(QIcon(ICON_PATH))
        self.lang_page = LanguagePage()
        self.path_page = PathPage()
        self.terms_page = TermsPage()
        self.welcome_page = WelcomePage()
        self.addPage(self.welcome_page)
        self.addPage(self.lang_page)
        self.addPage(self.path_page)
        self.addPage(self.terms_page)
        self.logic = None
        self.setOption(QWizard.WizardOption.HaveHelpButton, False)
        self.setOption(QWizard.WizardOption.HaveCustomButton1, False)
        self.setOption(QWizard.WizardOption.DisabledUpperRightButton, False)

    def initializePage(self, page):
        if page == self.welcome_page:
            self.welcome_page.update_text("Slovenský")

    def validatePage(self, page):
        if page == self.lang_page:
            lang = self.lang_page.get_language()
            self.welcome_page.update_text(lang)
            self.current_lang = lang
            t = TRANSLATIONS[lang]
            self.addPage(self.create_process_page(t["setup_title"], t["setup_sub"], "extract_files"))
            self.addPage(self.create_process_page(t["sdk_title"], t["sdk_sub"], "install_game_sdk"))
            self.addPage(self.create_process_page(t["shortcut_title"], t["shortcut_sub"], "create_desktop_shortcut"))
            self.logic = InstallWorker(self, self.path_page.get_path())
        return True

    def create_process_page(self, title, subtext, func_name):
        page = ProcessPage(title, subtext, func_name)
        page.button(QWizard.WizardButton.NextButton).clicked.connect(lambda: self.start_process(page))
        return page

    def start_process(self, page):
        self.button(QWizard.WizardButton.NextButton).setEnabled(False)
        self.button(QWizard.WizardButton.BackButton).setEnabled(False)
        func = getattr(self.logic, page.func_name)
        self.worker = Worker(func, self)
        self.worker.log.connect(page.update_log)
        self.worker.progress.connect(page.update_progress)
        self.worker.finished.connect(lambda success: self.finish_process(page, success))
        self.worker.start()

    def finish_process(self, page, success):
        self.button(QWizard.WizardButton.NextButton).setEnabled(True)
        self.button(QWizard.WizardButton.BackButton).setEnabled(True)
        t = TRANSLATIONS[self.current_lang]
        if success:
            page.update_log(t["success_msg"])
        else:
            page.update_log(t["error_msg"])

def main():
    app = QApplication(sys.argv)
    wizard = ETS2LAInstaller()
    wizard.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
