"""
UltraPilot — modern installer (PyQt6).

A bespoke dark/light setup window (logo hero, step rail, smooth navigation)
that **always** downloads the latest sources from GitHub, makes sure a usable
Python (>= 3.10, with pip) is present (auto-installing from python.org if not),
installs the Python dependencies, copies the SCS SDK plugin DLLs into the game,
installs the ViGEmBus driver, and creates Start-menu / desktop shortcuts.

Build it into a single UltraPilot_Installer.exe with build_installer.py.

NOTE: the source files are no longer read from the PyInstaller bundle
(_MEIPASS / payload). They are always fetched from the GitHub repository
``matule123/ets2la``. If the repo is private, set the ``GITHUB_TOKEN``
environment variable before launching the installer — without it, all three
download strategies (git clone / zip archive / raw file-by-file) will fail with
404. The permanent fix is to make the repository public (see Task 2).
"""

import os
import sys
import json
import shutil
import logging
import subprocess

from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer, QPropertyAnimation, QEasingCurve, QByteArray, pyqtProperty
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QStackedWidget, QProgressBar, QTextEdit, QFileDialog, QComboBox, QCheckBox,
    QLineEdit, QScrollArea, QFrame, QMessageBox,
)
from PyQt6.QtGui import QPixmap, QIcon, QColor, QPainter, QFont, QPen
from PyQt6.QtWidgets import QGraphicsOpacityEffect

APP_NAME = "UltraPilot"

# GitHub source — files are ALWAYS fetched from here.
REPO = "matule123/ets2la"
REPO_URL = "https://github.com/" + REPO + ".git"
ARCHIVE_URL = "https://github.com/" + REPO + "/archive/refs/heads/main.zip"
CONTENTS_API = "https://api.github.com/repos/" + REPO + "/git/trees/main?recursive=1"
RAW_BASE = "https://raw.githubusercontent.com/" + REPO + "/main/"

# Python auto-install (see Task 1). 3.12 is stable and ships working pip;
# 3.14 embeddable has no pip, so we use the official installer.
PY_VERSION = "3.12.9"
PY_INSTALLER_URL = "https://www.python.org/ftp/python/" + PY_VERSION + \
                   "/python-" + PY_VERSION + "-amd64.exe"


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

ACCENT = "#10B981"          # primary green
ACCENT_HI = "#34D399"       # lighter green (gradients / hover)
ACCENT_LO = "#059669"       # darker green (pressed / gradient end)
SUCCESS = "#22C55E"
DANGER = "#EF4444"
WARN = "#F59E0B"

DARK = {"bg": "#0E1116", "bg2": "#151A21", "card": "#1A2029", "card2": "#222A35",
        "text": "#E6E8EB", "muted": "#8A93A0", "border": "#272F3A",
        "title": "#34D399", "field": "#11161D", "glow": "rgba(16,185,129,0.35)"}
LIGHT = {"bg": "#F4F6F9", "bg2": "#FFFFFF", "card": "#FFFFFF", "card2": "#EEF2F6",
         "text": "#0F172A", "muted": "#64748B", "border": "#E2E8F0",
         "title": "#047857", "field": "#FFFFFF", "glow": "rgba(16,185,129,0.20)"}


def _qss(theme):
    c = DARK if theme == "dark" else LIGHT
    return (
        "#Window { background: " + c['bg'] + "; }"
        " #Hero { background: " + c['bg2'] + ";"
        " border-bottom: 1px solid " + c['border'] + "; }"
        " #StepBadge { background: " + c['card2'] + "; border: 1px solid " + c['border'] + ";"
        " border-radius: 11px; }"
        " QLabel { color: " + c['text'] + "; }"
        " QLabel#Title { font-size: 32px; font-weight: 800; letter-spacing: -0.5px; }"
        " QLabel#Subtitle { font-size: 15px; color: " + c['muted'] + "; }"
        " QLabel#SectionTitle { font-size: 13px; font-weight: 700; color: " + c['muted'] + ";"
        " text-transform: uppercase; letter-spacing: 1px; }"
        " QLabel#Brand { font-size: 22px; font-weight: 800; color: " + c['title'] + "; }"
        " QLabel#BrandSub { font-size: 11px; font-weight: 600; color: " + c['muted'] + "; }"
        " QLabel#StepLabel { font-size: 13px; font-weight: 600; color: " + c['muted'] + "; }"
        " QLabel#StepLabelActive { font-size: 13px; font-weight: 700; color: " + c['title'] + "; }"
        " QLabel#Caption { font-size: 12px; color: " + c['muted'] + "; }"
        " QLabel#Desc { font-size: 14px; color: " + c['text'] + "; }"
        " QLabel#Success { font-size: 64px; color: " + SUCCESS + "; }"
        " QLabel#Error { font-size: 64px; color: " + DANGER + "; }"
        " QLabel#StatusLine { font-size: 14px; font-weight: 600; color: " + c['title'] + "; }"
        " QLabel#FeatIcon { font-size: 26px; }"
        " QLabel#FeatName { font-size: 14px; font-weight: 700; color: " + c['text'] + "; }"
        " QLabel#FeatDesc { font-size: 12px; color: " + c['muted'] + "; }"
        " QLabel#DiskOk { font-size: 12px; color: " + SUCCESS + "; font-weight: 600; }"
        " QLabel#DiskWarn { font-size: 12px; color: " + WARN + "; font-weight: 600; }"
        " #Card, #FeatCard { background: " + c['card'] + "; border: 1px solid " + c['border'] + ";"
        " border-radius: 12px; }"
        " #FeatCard:hover { border-color: " + ACCENT + "; }"
        " QPushButton#Primary {"
        " background: qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 " + ACCENT_HI + ", stop:1 " + ACCENT_LO + ");"
        " color: #FFFFFF; border: none; border-radius: 10px; padding: 11px 24px;"
        " font-size: 14px; font-weight: 700; }"
        " QPushButton#Primary:hover { background: qlineargradient(x1:0,y1:0,x2:0,y2:1,"
        " stop:0 #3DEDA6, stop:1 #06A977); }"
        " QPushButton#Primary:disabled { background: " + c['card2'] + "; color: " + c['muted'] + ";"
        " border: 1px solid " + c['border'] + "; }"
        " QPushButton#Ghost { background: transparent; color: " + c['text'] + ";"
        " border: 1px solid " + c['border'] + "; border-radius: 10px; padding: 11px 18px;"
        " font-size: 14px; font-weight: 600; }"
        " QPushButton#Ghost:hover { border-color: " + ACCENT + "; color: " + ACCENT + "; }"
        " QPushButton#Icon { background: " + c['card2'] + "; color: " + c['text'] + ";"
        " border: 1px solid " + c['border'] + "; border-radius: 9px; padding: 7px 12px; font-size: 13px; }"
        " QPushButton#Icon:hover { border-color: " + ACCENT + "; color: " + ACCENT + "; }"
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
        " QProgressBar { background: " + c['card2'] + "; border: 1px solid " + c['border'] + ";"
        " border-radius: 8px; height: 14px; text-align: center; color: transparent; }"
        " QProgressBar::chunk {"
        " background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 " + ACCENT + ", stop:1 " + ACCENT_HI + ");"
        " border-radius: 7px; }"
        " QTextEdit { background: " + c['field'] + "; color: " + c['text'] + ";"
        " border: 1px solid " + c['border'] + "; border-radius: 9px;"
        " font-family: 'Consolas','Cascadia Mono',monospace; font-size: 12px; }"
        " QScrollArea { border: none; background: transparent; }"
        " QScrollBar:vertical { background: transparent; width: 10px; margin: 2px; }"
        " QScrollBar::handle:vertical { background: " + c['border'] + "; border-radius: 5px; min-height: 30px; }"
        " QScrollBar::handle:vertical:hover { background: " + c['muted'] + "; }"
        " QScrollBar::add-line, QScrollBar::sub-line { height: 0; }"
    )


# ----------------------------------------------------------------- i18n (JSON)
# Installer translation tables live as JSON files in ``languages/`` (shipped
# inside the bundle via ``--add-data``).  We read them with ``_res`` so it works
# under PyInstaller (_MEIPASS), next to the exe, and from source.  The installer
# only ships ``sk`` and ``en`` — extra languages can be downloaded later from
# the in-app onboarding.  Each JSON file has namespaces (``common``, ``installer``);
# we flatten them into one dict so the existing ``self.t["key"]`` code keeps working.
_LANG_CACHE = {}
_FEATS_FALLBACK = {
    "sk": (
        ("🛣️", "Udržiavanie pruhu", "Sleduje vozovku a drží kamión v pruhu."),
        ("🎯", "Adaptívny tempomat", "Udržiava rýchlosť a brzdí pred pomalšími."),
        ("🚦", "Semafor a prekážky", "Reaguje na zastavenia a prekážky v ceste."),
        ("🗺️", "Navigácia podľa mapy", "Jazdi po svete ETS2 podľa súradníc."),
        ("🖥️", "HUD a hlas", "Priehľadný prekryv a hlasové oznámenia."),
    ),
    "en": (
        ("🛣️", "Lane keeping", "Watches the road and keeps the truck in lane."),
        ("🎯", "Adaptive cruise", "Holds speed and brakes for slower traffic."),
        ("🚦", "Traffic & obstacles", "Reacts to stops and obstacles ahead."),
        ("🗺️", "Map navigation", "Drive the ETS2 world by coordinates."),
        ("🖥️", "HUD & voice", "Transparent overlay and voice announcements."),
    ),
}


def _lang_dir():
    """Where the bundled languages/ folder lives."""
    for r in (_res("languages"), os.path.join(os.path.dirname(os.path.abspath(__file__)), "languages")):
        if r and os.path.isdir(r):
            return r
    return _res("languages")


def _available_langs():
    """List of language codes available in the bundled languages/ folder."""
    d = _lang_dir()
    out = []
    try:
        for f in sorted(os.listdir(d)):
            if f.endswith(".json") and f != "index.json":
                out.append(f[:-5].lower())
    except Exception:
        pass
    if "sk" not in out:
        out.insert(0, "sk")
    if "en" not in out:
        out.append("en")
    return out


def _lang_name(code):
    """Display name for a language code (from _meta.name, with fallbacks)."""
    tbl = _load_lang(code)
    meta = tbl.get("_meta") if isinstance(tbl, dict) else {}
    if isinstance(meta, dict) and meta.get("name"):
        return meta["name"]
    return {"sk": "Slovenčina", "en": "English",
            "cs": "Čeština", "de": "Deutsch", "pl": "Polski",
            "fr": "Français", "es": "Español"}.get(code, code)


def _load_lang(code):
    """Load one language file, flattened (common + installer merged). Cached."""
    code = (code or "sk").lower()
    if code in _LANG_CACHE:
        return _LANG_CACHE[code]
    path = _res("languages", code + ".json")
    if not os.path.exists(path):
        # Fall back to Slovak, then English.
        for c in ("sk", "en"):
            p = _res("languages", c + ".json")
            if os.path.exists(p):
                path = p
                code = c
                break
    try:
        import json as _json
        with open(path, encoding="utf-8") as f:
            data = _json.load(f)
        # Flatten: merge common.* + installer.* into one dict.
        flat = {}
        flat.update(data.get("common", {}))
        flat.update(data.get("installer", {}))
        flat["_meta"] = data.get("_meta", {})
        _LANG_CACHE[code] = flat
        return flat
    except Exception:
        return {}


def _lang_coverage(code):
    """Percent of English keys present in ``code`` (flattened view)."""
    en = _load_lang("en")
    if not en:
        return 100
    tbl = _load_lang(code)
    ref = {k for k in en if not k.startswith("_") and not isinstance(en[k], (list, tuple))}
    have = {k for k in ref if k in tbl}
    return round(100 * len(have) / len(ref)) if ref else 100


# Backward-compatible names used elsewhere in this file.
# TR maps BOTH display names (legacy) and language codes to the flat dict.
TR = {}
TR["sk"] = TR["Slovenčina"] = _load_lang("sk")
TR["en"] = TR["English"] = _load_lang("en")


def _ensure_lang_loaded(code):
    """Make sure ``code`` is loaded into TR under both its code and display name."""
    code = (code or "").lower()
    if code and code not in TR:
        flat = _load_lang(code)
        if flat:
            TR[code] = flat
            name = (flat.get("_meta") or {}).get("name")
            if name:
                TR[name] = flat
    return TR.get(code)


def tr_get(lang, key):
    """Translate ``key`` for ``lang`` (a display name OR a code)."""
    if lang in TR:
        return TR[lang].get(key, TR["Slovensky"].get(key, key))
    # Treat as a code.
    tbl = _load_lang(lang) or _load_lang("sk")
    return tbl.get(key, _load_lang("sk").get(key, key))


# Paths/entries that must never be copied from the GitHub tree.
_FETCH_BLACKLIST_DIRS = ("__pycache__", ".git", ".github", ".claude", ".vscode",
                         ".idea", "build", "dist", "map-cache", "model-cache",
                         "routes", "UltraPilot.egg-info", "node_modules")
_FETCH_BLACKLIST_SUFFIX = (".pyc", ".pyo", ".log", ".msi", ".exe", ".spec", ".egg-info")
_FETCH_BLACKLIST_FILES = {"settings.json", ".gitignore", ".ds_store", "thumbs.db"}


def _github_headers():
    """Auth headers for GitHub requests (token optional, enables private repos)."""
    h = {"Accept": "application/vnd.github+json"}
    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if token:
        h["Authorization"] = "Bearer " + token
    return h


class InstallWorker(QThread):
    log = pyqtSignal(str)
    progress = pyqtSignal(int)
    status = pyqtSignal(str)        # human-readable name of the current stage
    finished_ok = pyqtSignal(bool, str)

    def __init__(self, install_path, lang):
        super().__init__()
        self.install_path = install_path
        # ``lang`` may be a code (sk/en) or a legacy display name. Resolve to
        # a flat translation dict (common + installer namespaces merged).
        self.lang = lang
        if lang in TR:                  # legacy display name path
            self.t = TR[lang]
        else:
            self.t = _load_lang(lang) or _load_lang("sk")

    # ---------------------------------------------------------------- Python
    def _real_python(self):
        """Find a usable Python (>= 3.10, with pip) on PATH. Returns [args] or []."""
        candidates = []
        py = shutil.which("py") or shutil.which("py.exe")
        if py:
            candidates.append([py, "-3"])
        for name in ("python", "python.exe", "python3", "python3.exe"):
            found = shutil.which(name)
            if found:
                candidates.append([found])

        for c in candidates:
            # Version check.
            try:
                r = subprocess.run([*c, "--version"], capture_output=True, text=True, timeout=10)
                out = (r.stdout or r.stderr).strip()  # 'Python 3.12.9'
                parts = out.lower().replace("python", "").strip().split(".")
                major = int(parts[0]) if parts and parts[0].isdigit() else 0
                minor = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
                if (major, minor) < (3, 10):
                    continue
            except Exception:
                continue
            # pip check.
            try:
                rp = subprocess.run([*c, "-m", "pip", "--version"],
                                    capture_output=True, timeout=30)
                if rp.returncode == 0:
                    return c
            except Exception:
                continue
        return []

    def _refresh_path_from_registry(self):
        """After installing Python, re-read PATH from registry so we can use it now."""
        try:
            import winreg
            extra = []
            for hive, path, flag in (
                (winreg.HKEY_CURRENT_USER, r"Environment", winreg.KEY_READ),
                (winreg.HKEY_LOCAL_MACHINE,
                 r"System\CurrentControlSet\Control\Session Manager\Environment", winreg.KEY_READ),
            ):
                try:
                    with winreg.OpenKey(hive, path, 0, flag) as k:
                        val, _ = winreg.QueryValueEx(k, "PATH")
                        extra.append(val)
                except Exception:
                    pass
            if extra:
                merged = os.environ.get("PATH", "") + os.pathsep + os.pathsep.join(extra)
                os.environ["PATH"] = merged
        except Exception:
            pass

    def _install_python_from_web(self):
        """Download + run the official Python installer (/passive, per-user, PATH on)."""
        try:
            import requests
        except Exception as e:
            self.log.emit(self.t["py_fail"].format(err=str(e)))
            return False
        tmp = os.path.join(os.environ.get("TEMP", os.path.expanduser("~")),
                           "UltraPilot_python_installer.exe")
        try:
            self.log.emit(self.t["py_download"].format(ver=PY_VERSION))
            r = requests.get(PY_INSTALLER_URL, timeout=120, stream=True)
            if r.status_code != 200:
                self.log.emit(self.t["py_fail"].format(err="HTTP " + str(r.status_code)))
                return False
            with open(tmp, "wb") as f:
                for chunk in r.iter_content(chunk_size=65536):
                    if chunk:
                        f.write(chunk)
            self.log.emit(self.t["py_install"])
            # /passive: progress bar, no user clicks. InstallAllUsers=0: per-user
            # (no admin prompt). PrependPath=1: puts python on PATH. Include_pip=1.
            proc = subprocess.run(
                [tmp, "/passive", "InstallAllUsers=0", "PrependPath=1",
                 "Include_pip=1", "Include_test=0", "InstallLauncherAllUsers=0"],
                timeout=600)
            ok = proc.returncode == 0
            if ok:
                self.log.emit(self.t["py_done"])
            else:
                self.log.emit(self.t["py_fail"].format(err="kód " + str(proc.returncode)))
            self._refresh_path_from_registry()
            return ok
        except Exception as e:
            self.log.emit(self.t["py_fail"].format(err=str(e)))
            return False
        finally:
            try:
                if os.path.exists(tmp):
                    os.remove(tmp)
            except Exception:
                pass

    def _ensure_python(self):
        """Make sure a usable Python is available. Auto-install if missing."""
        self.status.emit(self.t["py_check"])
        py = self._real_python()
        if py:
            self.log.emit(self.t["py_found"].format(py=py[0]))
            return True
        self.log.emit(self.t["py_missing"])
        if self._install_python_from_web():
            py = self._real_python()
            if py:
                self.log.emit(self.t["py_found"].format(py=py[0]))
                return True
        self.log.emit(self.t["py_manual"])
        return False

    # ---------------------------------------------------------------- Sources
    def _try_git_clone(self):
        try:
            tmp = self.install_path + "_clone"
            if os.path.isdir(tmp):
                shutil.rmtree(tmp, ignore_errors=True)
            r = subprocess.run(["git", "clone", "--depth", "1", REPO_URL, tmp],
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
                self.log.emit(self.t["src_git_ok"])
                return True
        except Exception as e:
            self.log.emit(self.t["src_err"].format(err=str(e)))
        return False

    def _try_zip_archive(self):
        try:
            import requests, zipfile, io
            resp = requests.get(ARCHIVE_URL, headers=_github_headers(), timeout=120)
            if resp.status_code == 200:
                zf = zipfile.ZipFile(io.BytesIO(resp.content))
                zf.extractall(self.install_path)
                root = os.path.join(self.install_path, "ets2la-main")
                if os.path.isdir(root):
                    for item in os.listdir(root):
                        shutil.move(os.path.join(root, item),
                                    os.path.join(self.install_path, item))
                    shutil.rmtree(root, ignore_errors=True)
                self.log.emit(self.t["src_zip_ok"])
                return True
        except Exception as e:
            self.log.emit(self.t["src_err"].format(err=str(e)))
        return False

    def _try_raw_file_by_file(self):
        """Last-resort: list the tree via Contents API and fetch each blob raw."""
        try:
            import requests
            r = requests.get(CONTENTS_API, headers=_github_headers(), timeout=30)
            if r.status_code != 200:
                self.log.emit(self.t["src_err"].format(err="API HTTP " + str(r.status_code)))
                return False
            tree = r.json().get("tree", [])
            blobs = [e for e in tree if e.get("type") == "blob"]

            def allowed(path):
                lower = path.lower().replace("/", os.sep)
                parts = lower.split(os.sep)
                if any(p in _FETCH_BLACKLIST_DIRS for p in parts):
                    return False
                if os.path.basename(lower) in _FETCH_BLACKLIST_FILES:
                    return False
                if any(lower.endswith(suf) for suf in _FETCH_BLACKLIST_SUFFIX):
                    return False
                return True

            count = 0
            for entry in blobs:
                path = entry["path"]
                if not allowed(path):
                    continue
                dest = os.path.join(self.install_path, path)
                os.makedirs(os.path.dirname(dest), exist_ok=True)
                rr = requests.get(RAW_BASE + path, headers=_github_headers(), timeout=60)
                if rr.status_code == 200:
                    with open(dest, "wb") as f:
                        f.write(rr.content)
                    count += 1
            if count > 0:
                self.log.emit(self.t["src_raw_ok"].format(n=count))
                return os.path.exists(os.path.join(self.install_path, "main.py"))
        except Exception as e:
            self.log.emit(self.t["src_err"].format(err=str(e)))
        return False

    def _fetch_repo(self):
        """Always fetch the latest sources from GitHub. Three fallback strategies."""
        self.status.emit(self.t["src_try_git"])
        if self._try_git_clone():
            return True
        self.status.emit(self.t["src_try_zip"])
        if self._try_zip_archive():
            return True
        self.status.emit(self.t["src_try_raw"])
        if self._try_raw_file_by_file():
            return True
        self.log.emit(self.t["src_fail"])
        return False

    # ---------------------------------------------------------------- Deps
    def _pip_install(self):
        req = os.path.join(self.install_path, "requirements.txt")
        py = self._real_python()
        if not py:
            self.log.emit("  Python nebol nájdený — závislosti preskočené.")
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

    # ---------------------------------------------------------------- Shortcuts
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

    # ---------------------------------------------------------------- Main run
    def run(self):
        try:
            mode = "source"
            self.log.emit(self.t["s_prep"])
            self.progress.emit(2)

            # 1) Make sure a usable Python exists (auto-install from python.org).
            if not self._ensure_python():
                self.finished_ok.emit(False, "")
                return
            self.progress.emit(10)

            # 2) Always download the latest sources from GitHub.
            os.makedirs(self.install_path, exist_ok=True)
            if not self._fetch_repo():
                raise RuntimeError("Nepodarilo sa získať súbory UltraPilot z GitHubu.")
            self.progress.emit(50)

            # 3) Python dependencies.
            self.status.emit(self.t["s_deps"])
            self._pip_install()
            exe_path = os.path.join(self.install_path, "main.py")
            self.progress.emit(75)

            # 4) SCS plugin DLLs into the game.
            self.status.emit(self.t["s_dll"])
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
            self.progress.emit(82)

            # 5) ViGEmBus driver.
            self.status.emit(self.t["s_vigem"])
            try:
                from core.sdk.vigembus import ensure_vigembus
                ensure_vigembus(os.path.join(self.install_path, "assets"),
                                log=self.log.emit)
            except Exception as e:
                self.log.emit("  (" + str(e) + ")")
            self.progress.emit(90)

            # 6) Shortcuts + install record.
            self.status.emit(self.t["s_short"])
            self._make_shortcuts(exe_path, mode)
            try:
                rec = {"install_path": self.install_path, "exe_path": exe_path, "mode": mode}
                os.makedirs(os.path.dirname(RECORD_PATH), exist_ok=True)
                with open(RECORD_PATH, "w", encoding="utf-8") as f:
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


class ThemeToggle(QWidget):
    """Animated pill-shaped dark/light switch with a sun (light) / moon (dark).

    Clicking it slides the knob from one side to the other with a 220 ms eased
    animation and emits ``toggled(bool dark)``. Paint is fully custom so it
    looks identical in every palette and stays legible in both themes."""

    toggled = pyqtSignal(bool)

    def __init__(self, dark: bool = False, parent=None):
        super().__init__(parent)
        self._dark = bool(dark)
        self._knob = 1.0 if self._dark else 0.0   # 0 = sun (light), 1 = moon (dark)
        self.setFixedSize(58, 30)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self._anim = None

    def is_dark(self) -> bool:
        return self._dark

    @pyqtProperty(float)
    def knob(self) -> float:
        return self._knob

    @knob.setter
    def knob(self, v: float):
        self._knob = float(v)
        self.update()

    def set_dark(self, dark: bool, animate: bool = True):
        dark = bool(dark)
        if dark == self._dark and self._anim is None:
            return
        self._dark = dark
        target = 1.0 if dark else 0.0
        if self._anim is not None:
            self._anim.stop()
        if animate:
            self._anim = QPropertyAnimation(self, b"knob", self)
            self._anim.setDuration(220)
            self._anim.setStartValue(self._knob)
            self._anim.setEndValue(target)
            self._anim.setEasingCurve(QEasingCurve.Type.InOutCubic)
            self._anim.start(QPropertyAnimation.DeletionPolicy.DeleteWhenStopped)
        else:
            self._knob = target
            self.update()
        self.toggled.emit(self._dark)

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            self.set_dark(not self._dark)
        super().mouseReleaseEvent(e)

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        w, h = self.width(), self.height()
        # Track colours blend between sun (light theme) and moon (dark theme).
        # k=0 → light theme, k=1 → dark theme.
        k = self._knob
        sun_bg = QColor("#FBBF24")     # warm amber track when light
        moon_bg = QColor("#1F2937")    # deep slate track when dark
        bg = QColor(
            int(sun_bg.red()   + (moon_bg.red()   - sun_bg.red())   * k),
            int(sun_bg.green() + (moon_bg.green() - sun_bg.green()) * k),
            int(sun_bg.blue()  + (moon_bg.blue()  - sun_bg.blue())  * k),
        )
        p.setBrush(bg)
        p.setPen(QPen(QColor(255, 255, 255, 40), 1))
        p.drawRoundedRect(1, 1, w - 2, h - 2, h / 2 - 1, h / 2 - 1)

        # Knob travels between the two ends with a little inset margin.
        margin = 4
        knob_d = h - margin * 2
        x = margin + k * (w - margin * 2 - knob_d)
        knob = QColor("#FFFFFF")
        p.setBrush(knob)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(int(x), margin, knob_d, knob_d)

        # Icon inside the knob: a sun (rays) when light, a crescent moon when dark.
        cx = x + knob_d / 2
        cy = h / 2
        p.setPen(QPen(QColor("#F59E0B").darker(120), 1))
        # Sun rays (drawn faintly so they fade out as it gets dark).
        ray_color = QColor("#F59E0B")
        ray_color.setAlphaF(max(0.0, 1.0 - k))
        p.setPen(QPen(ray_color, 1.4))
        import math
        for ang in range(0, 360, 45):
            a = math.radians(ang)
            r1 = knob_d * 0.30
            r2 = knob_d * 0.42
            p.drawLine(
                int(cx + math.cos(a) * r1), int(cy + math.sin(a) * r1),
                int(cx + math.cos(a) * r2), int(cy + math.sin(a) * r2),
            )
        # Crescent moon (fades in as it gets dark).
        moon_color = QColor("#FBBF24")
        moon_color.setAlphaF(max(0.0, k))
        p.setBrush(moon_color)
        p.setPen(Qt.PenStyle.NoPen)
        mr = knob_d * 0.34
        p.drawEllipse(int(cx - mr * 0.15), int(cy), int(mr * 2), int(mr * 2))
        # Bite a crescent out of the moon using the track colour.
        p.setBrush(bg)
        p.drawEllipse(int(cx + mr * 0.55), int(cy - mr * 0.25), int(mr * 1.8), int(mr * 1.8))


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

    Steps live in a QStackedWidget; the hero header + step rail stay fixed.
    Switching pages applies a short fade so the transition feels smooth."""

    def __init__(self):
        super().__init__()
        self.setObjectName("Window")
        self.lang = "sk"
        self.theme = "dark"
        self.exe_path = ""
        self._worker = None
        self._cur = 0
        self.setWindowTitle(TR[self.lang]["win"])
        self.setFixedSize(820, 640)
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

    # ----------------------------------------------------------------- chrome
    def _build_hero(self):
        hero = QFrame()
        hero.setObjectName("Hero")
        hero.setFixedHeight(80)
        h = QHBoxLayout(hero)
        h.setContentsMargins(30, 16, 22, 16)
        logo = QLabel()
        pm = QIcon(ICON_PATH).pixmap(46, 46)
        if pm.isNull():
            pm = QPixmap(LOGO_PATH).scaledToWidth(46, Qt.TransformationMode.SmoothTransformation)
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
        self.theme_btn = ThemeToggle(dark=(self.theme == "dark"))
        self.theme_btn.toggled.connect(self._on_theme_toggle)
        h.addWidget(self.theme_btn)
        return hero

    def _build_step_rail_widget(self, parent_layout):
        rail = QWidget()
        rail.setFixedHeight(50)
        h = QHBoxLayout(rail)
        h.setContentsMargins(30, 10, 30, 10)
        h.setSpacing(8)
        self._step_labels = []
        steps = TR[self.lang]["steps"]
        for i, name in enumerate(steps):
            badge = QLabel(str(i + 1))
            badge.setObjectName("StepBadge")
            badge.setFixedSize(24, 24)
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
        lay.setContentsMargins(38, 24, 38, 24)
        lay.setSpacing(14)
        scroll.setWidget(inner)
        return scroll, lay

    # ----------------------------------------------------------------- pages
    def _build_welcome(self):
        scroll, lay = self._page_frame()
        title = QLabel(TR[self.lang]["welcome_t"])
        title.setObjectName("Title")
        lay.addWidget(title)
        desc = QLabel(TR[self.lang]["welcome_d"])
        desc.setObjectName("Desc")
        desc.setWordWrap(True)
        lay.addWidget(desc)
        lay.addSpacing(6)

        # Feature grid (2 columns).
        feat_title = QLabel(TR[self.lang]["feat_t"])
        feat_title.setObjectName("SectionTitle")
        lay.addWidget(feat_title)
        grid = QVBoxLayout()
        grid.setSpacing(8)
        row = None
        feats = TR[self.lang]["feats"]
        for i, (icon, name, fd) in enumerate(feats):
            if i % 2 == 0:
                row = QHBoxLayout()
                row.setSpacing(8)
                grid.addLayout(row)
            card = QFrame()
            card.setObjectName("FeatCard")
            cl = QHBoxLayout(card)
            cl.setContentsMargins(14, 12, 14, 12)
            cl.setSpacing(12)
            ic = QLabel(icon)
            ic.setObjectName("FeatIcon")
            col = QVBoxLayout()
            col.setSpacing(2)
            nm = QLabel(name)
            nm.setObjectName("FeatName")
            ds = QLabel(fd)
            ds.setObjectName("FeatDesc")
            ds.setWordWrap(True)
            col.addWidget(nm)
            col.addWidget(ds)
            cl.addWidget(ic)
            cl.addLayout(col, stretch=1)
            row.addWidget(card)
        if row is not None:
            row.addStretch()
        lay.addLayout(grid)

        # Requirements box.
        req_title = QLabel(TR[self.lang]["req_t"])
        req_title.setObjectName("SectionTitle")
        lay.addWidget(req_title)
        req_card = QFrame()
        req_card.setObjectName("Card")
        rl = QVBoxLayout(req_card)
        rl.setContentsMargins(14, 12, 14, 12)
        rl.setSpacing(6)
        for it in TR[self.lang]["req_items"]:
            lab = QLabel("•  " + it)
            lab.setStyleSheet("font-size:13px;")
            rl.addWidget(lab)
        lay.addWidget(req_card)

        # Language row. The installer ships only sk + en (others are downloadable
        # from the in-app onboarding); each entry shows the display name and the
        # translation coverage percentage.
        row = QHBoxLayout()
        row.setSpacing(10)
        cap = QLabel(TR[self.lang].get("language", TR[self.lang].get("lang", "Language")))
        cap.setObjectName("Caption")
        self.lang_combo = QComboBox()
        # The installer ships only Slovak + English (others are downloadable
        # later from the in-app onboarding). Each entry shows the display name
        # and the translation coverage percentage.
        for code in ("sk", "en"):
            _ensure_lang_loaded(code)
            name = _lang_name(code)
            cov = _lang_coverage(code)
            self.lang_combo.addItem(f"{name}  ·  {cov}%", code)
        # Select the current code by data.
        for i in range(self.lang_combo.count()):
            if self.lang_combo.itemData(i) == self.lang:
                self.lang_combo.setCurrentIndex(i)
                break
        self.lang_combo.currentIndexChanged.connect(self._on_lang_idx)
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
        self.path_edit.textChanged.connect(self._update_path_status)
        browse = _ghost_btn(TR[self.lang]["browse"])
        browse.clicked.connect(self._browse)
        row.addWidget(self.path_edit, stretch=1)
        row.addWidget(browse)
        lay.addWidget(lbl)
        lay.addLayout(row)
        # Disk-free + non-empty indicator.
        self.path_status = QLabel("")
        self.path_status.setObjectName("DiskOk")
        lay.addWidget(self.path_status)
        lay.addStretch()
        self.stack.addWidget(scroll)
        self._update_path_status()

    def _build_install(self):
        scroll, lay = self._page_frame()
        title = QLabel(TR[self.lang]["inst_t"])
        title.setObjectName("Title")
        sub = QLabel(TR[self.lang]["inst_s"])
        sub.setObjectName("Subtitle")
        lay.addWidget(title)
        lay.addWidget(sub)
        lay.addSpacing(8)
        self.status_line = QLabel(TR[self.lang]["status_wait"])
        self.status_line.setObjectName("StatusLine")
        lay.addWidget(self.status_line)
        self.progress = QProgressBar()
        self.progress.setValue(0)
        lay.addWidget(self.progress)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        lay.addWidget(self.log_view)
        self.stack.addWidget(scroll)

    def _build_finish(self):
        scroll, lay = self._page_frame()
        icon = QLabel("✔")
        icon.setObjectName("Success")
        icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.fin_title = QLabel(TR[self.lang]["fin_t"])
        self.fin_title.setObjectName("Title")
        self.fin_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.fin_sub = QLabel(TR[self.lang]["fin_s"])
        self.fin_sub.setObjectName("Subtitle")
        self.fin_sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.fin_summary = QLabel("")
        self.fin_summary.setObjectName("Desc")
        self.fin_summary.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.fin_summary.setWordWrap(True)
        self.launch_chk = QCheckBox(TR[self.lang]["fin_launch"])
        self.launch_chk.setChecked(True)
        lay.addStretch()
        lay.addWidget(icon)
        lay.addWidget(self.fin_title)
        lay.addWidget(self.fin_sub)
        lay.addSpacing(4)
        lay.addWidget(self.fin_summary)
        lay.addSpacing(10)
        self._centered = QHBoxLayout()
        self._centered.addStretch()
        self._centered.addWidget(self.launch_chk)
        self._centered.addStretch()
        lay.addLayout(self._centered)
        lay.addStretch()
        self.stack.addWidget(scroll)

    def _build_footer(self, parent_layout):
        foot = QFrame()
        foot.setFixedHeight(66)
        foot.setObjectName("Hero")  # reuse the hero background so it reads as a footer bar
        fh = QHBoxLayout(foot)
        fh.setContentsMargins(30, 12, 30, 12)
        fh.setSpacing(10)
        self.back_btn = _ghost_btn(TR[self.lang]["back"])
        self.back_btn.clicked.connect(self._back)
        self.next_btn = _primary_btn(TR[self.lang]["next"])
        self.next_btn.clicked.connect(self._next)
        fh.addStretch()
        fh.addWidget(self.back_btn)
        fh.addWidget(self.next_btn)
        parent_layout.addWidget(foot)

    # ----------------------------------------------------------------- behavior
    def _apply_theme(self):
        self.setStyleSheet(_qss(self.theme))
        self._sync_theme_widgets()

    def _on_theme_toggle(self, dark):
        """ThemeToggle flipped — ``dark`` is the new state."""
        self.theme = "dark" if dark else "light"
        self._apply_theme()

    def _toggle_theme(self):
        # Kept for completeness (e.g. keyboard shortcuts); the toggle widget is
        # the primary UI now.
        self.theme = "light" if self.theme == "dark" else "dark"
        if hasattr(self, "theme_btn") and isinstance(self.theme_btn, ThemeToggle):
            self.theme_btn.set_dark(self.theme == "dark", animate=True)
        self._apply_theme()

    def _sync_theme_widgets(self):
        """Re-apply theme-dependent inline styles (step rail, path status).

        These widgets use inline palettes derived from DARK/LIGHT so they must
        be refreshed whenever the theme changes — otherwise stale colours leave
        them invisible (the root cause of the dark-mode „nothing shows up“ bug)."""
        # Step rail badges + labels.
        c = DARK if self.theme == "dark" else LIGHT
        if hasattr(self, "_step_labels"):
            idx = getattr(self, "_cur", 0)
            for i, (badge, lbl, wrap) in enumerate(self._step_labels):
                active = (i == idx)
                done = (i < idx)
                if active:
                    bg, fg, bd = ACCENT, "#FFFFFF", ACCENT
                    txt = "✓" if done else str(i + 1)
                elif done:
                    bg, fg, bd = c['card2'], SUCCESS, SUCCESS
                    txt = "✓"
                else:
                    bg, fg, bd = c['card2'], c['muted'], c['border']
                    txt = str(i + 1)
                badge.setText(txt)
                badge.setStyleSheet(
                    "color:" + fg + "; background:" + bg + "; border:1px solid " + bd + ";"
                    " border-radius:12px; font-weight:700;")
                lbl.setStyleSheet("color:" + (c['title'] if active else c['muted']) +
                                  "; font-size:13px; font-weight:" + ("700" if active else "600") + ";")
        # Path status colour (objectName drives QSS, but re-apply to be safe).
        if hasattr(self, "path_status"):
            ok = self.path_status.objectName() == "DiskOk"
            col = SUCCESS if ok else WARN
            self.path_status.setStyleSheet("color:" + col + "; font-size:12px; font-weight:600;")

    def _on_lang_idx(self, idx):
        """Language combo changed — ``idx`` is the row; data holds the code."""
        code = self.lang_combo.itemData(idx) if idx >= 0 else "sk"
        if code:
            self.lang = code
            _ensure_lang_loaded(code)

    def _update_path_status(self):
        if not hasattr(self, "path_status"):
            return
        p = self.path_edit.text().strip()
        if not p:
            self.path_status.setText("")
            return
        try:
            usage = shutil.disk_usage(p)
            gb = usage.free / (1024 ** 3)
            ok = gb >= 0.5
            self.path_status.setText(
                (self.t_disk_free() if ok else self.t_disk_low()).format(
                    "{:.1f} GB".format(gb)))
            self.path_status.setObjectName("DiskOk" if ok else "DiskWarn")
        except Exception:
            # Drive not reachable yet (e.g. user is typing). Quiet.
            non_empty = os.path.isdir(p) and len(os.listdir(p)) > 0 if os.path.isdir(p) else False
            if non_empty:
                self.path_status.setText(self.t_disk_warn())
                self.path_status.setObjectName("DiskWarn")
            else:
                self.path_status.setText("")
            return
        # Non-empty check.
        try:
            if os.path.isdir(p) and os.listdir(p):
                self.path_status.setText(self.path_status.text() + "   " + self.t_disk_warn())
                self.path_status.setObjectName("DiskWarn")
        except Exception:
            pass
        # Re-apply object style.
        self.path_status.setStyleSheet(self.styleSheet())

    def t_disk_free(self):
        return TR[self.lang]["disk_free"]

    def t_disk_low(self):
        return TR[self.lang]["disk_low"]

    def t_disk_warn(self):
        return TR[self.lang]["disk_warn"]

    def _go_step(self, idx):
        idx = max(0, min(idx, self.stack.count() - 1))
        # Tear down any opacity effect left on the page we're leaving — a
        # lingering QGraphicsOpacityEffect on a QScrollArea is what made content
        # from the previous step bleed through / overlap the new one.
        prev = self.stack.currentWidget()
        if prev is not None:
            try:
                prev.setGraphicsEffect(None)
            except Exception:
                pass
        self.stack.setCurrentIndex(idx)
        self._cur = idx
        self._sync_theme_widgets()
        self._fade_in(self.stack.currentWidget())
        self._update_nav()

    def _fade_in(self, widget):
        # A short opacity fade makes the step transition feel smooth. We animate
        # the inner content widget (not the QScrollArea itself — effects on
        # scroll areas cause rendering glitches) and clear the effect when done
        # so nothing leaks into later repaints.
        if widget is None:
            return
        try:
            target = widget.widget() if hasattr(widget, "widget") else widget
            eff = QGraphicsOpacityEffect(target)
            target.setGraphicsEffect(eff)
            anim = QPropertyAnimation(eff, b"opacity", target)
            anim.setDuration(150)
            anim.setStartValue(0.0)
            anim.setEndValue(1.0)
            anim.setEasingCurve(QEasingCurve.Type.OutCubic)

            def _cleanup(*_):
                try:
                    target.setGraphicsEffect(None)
                except Exception:
                    pass
            anim.finished.connect(_cleanup)
            anim.start(QPropertyAnimation.DeletionPolicy.DeleteWhenStopped)
        except Exception:
            pass

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

    def _append_log(self, line):
        """Colorize log lines by their prefix marker."""
        prefix, rest = "", line
        if line.startswith("✓"):
            prefix, rest = "✓", line[1:]
            color = SUCCESS
        elif line.startswith("✗"):
            prefix, rest = "✗", line[1:]
            color = DANGER
        elif line.startswith("⚠"):
            prefix, rest = "⚠", line[1:]
            color = WARN
        elif line.startswith("✔"):
            prefix, rest = "✔", line[1:]
            color = SUCCESS
        else:
            self.log_view.append(line)
            return
        if prefix:
            html = ('<span style="color:{}; font-weight:700;">{}</span>'
                    '<span style="color:inherit;">{}</span>').format(color, prefix, rest)
            self.log_view.append(html)

    def _start_install(self):
        if self._worker is not None and self._worker.isRunning():
            return
        path = self.path_edit.text().strip() or os.path.join(
            os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "UltraPilot")
        self.progress.setValue(0)
        self.log_view.clear()
        self._worker = InstallWorker(path, self.lang)
        self._worker.log.connect(self._append_log)
        self._worker.progress.connect(self.progress.setValue)
        self._worker.status.connect(self._on_status)
        self._worker.finished_ok.connect(self._on_done)
        self._worker.start()

    def _on_status(self, text):
        self.status_line.setText(text)

    def _on_done(self, ok, exe_path):
        self.exe_path = exe_path
        if ok:
            try:
                self.fin_summary.setText(
                    TR[self.lang]["fin_summary"].format(path=exe_path or ""))
            except Exception:
                self.fin_summary.setText("")
            self._go_step(4)
        else:
            QMessageBox.warning(self, APP_NAME, "Inštalácia zlyhala. Pozri log vyššie.")
            self._update_nav()

    def closeEvent(self, event):
        if hasattr(self, "launch_chk") and self.launch_chk.isChecked() and self.exe_path:
            self._launch_app()
        super().closeEvent(event)

    def _launch_app(self):
        """Launch the freshly installed UltraPilot.

        We must NOT ``os.startfile(main.py)`` — that opens whatever is associated
        with the ``.py`` extension (commonly VS Code). Instead run the launcher
        ``UltraPilot.bat`` that ``_make_shortcuts`` wrote next to ``main.py``; it
        invokes ``py -3 main.py`` from the install dir so the app actually starts.
        Failing that, run ``py -3 main.py`` directly via subprocess."""
        install_dir = os.path.dirname(self.exe_path) if self.exe_path else ""
        if not install_dir or not os.path.isdir(install_dir):
            return
        bat = os.path.join(install_dir, "UltraPilot.bat")
        try:
            if sys.platform == "win32":
                if os.path.exists(bat):
                    # Use the launcher .bat — it sets cwd and runs py -3 main.py.
                    os.startfile(bat)
                else:
                    # No launcher (shortcuts failed) — run py directly with cwd.
                    subprocess.Popen(["py", "-3", "main.py"],
                                     cwd=install_dir,
                                     creationflags=subprocess.DETACHED_PROCESS)
            else:
                subprocess.Popen([sys.executable, "main.py"], cwd=install_dir)
        except Exception as e:
            logging.debug("launch failed: %s", e)


def _read_record():
    try:
        if os.path.exists(RECORD_PATH):
            with open(RECORD_PATH, encoding="utf-8") as f:
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
