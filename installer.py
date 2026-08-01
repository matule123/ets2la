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
import math
import re
import shutil
import logging
import subprocess

from PyQt6.QtCore import (Qt, QThread, pyqtSignal, QTimer, QPropertyAnimation,
                          QEasingCurve, QByteArray, pyqtProperty, QPointF, QRectF)
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QStackedWidget, QProgressBar, QTextEdit, QFileDialog, QComboBox, QCheckBox,
    QLineEdit, QScrollArea, QFrame, QMessageBox, QDialog, QGridLayout,
)
from PyQt6.QtGui import QPixmap, QIcon, QColor, QPainter, QFont, QPen
from PyQt6.QtWidgets import QGraphicsOpacityEffect

APP_NAME = "UltraPilot"
APP_VERSION = "0.4.1"
INSTALLER_VERSION = "1.3"

# On Windows, hide the black CMD consoles that subprocess.run would otherwise
# flash up (git, pip, powershell). 0x08000000 = CREATE_NO_WINDOW.
_NO_WIN = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0

# GitHub source — files are ALWAYS fetched from here.
REPO = "matule123/ets2la"
REPO_URL = "https://github.com/" + REPO + ".git"
ARCHIVE_URL = "https://github.com/" + REPO + "/archive/refs/heads/main.zip"
CODELOAD_URL = "https://codeload.github.com/" + REPO + "/zip/refs/heads/main"
CONTENTS_API = "https://api.github.com/repos/" + REPO + "/git/trees/main?recursive=1"
RAW_BASE = "https://raw.githubusercontent.com/" + REPO + "/main/"
COMMIT_API = "https://api.github.com/repos/" + REPO + "/commits/main"

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
                           "Programs", "UltraPilot", "install.json")

ACCENT = "#10B981"          # primary green
ACCENT_HI = "#34D399"       # lighter green (gradients / hover)
ACCENT_LO = "#059669"       # darker green (pressed / gradient end)
SUCCESS = "#22C55E"
SUCCESS_DARK = "#16A34A"    # darker green for done step badges (white text contrast)
DANGER = "#EF4444"
WARN = "#F59E0B"

# GitHub-style black + neutral grey dark palette (no blue tint).
DARK = {"bg": "#0D1117", "bg2": "#161B22", "card": "#161B22", "card2": "#21262D",
        "text": "#E6EDF3", "muted": "#8B949E", "border": "#30363D",
        "title": "#2EA043", "field": "#0D1117", "glow": "rgba(46,160,67,0.35)"}
LIGHT = {"bg": "#F4F6F9", "bg2": "#FFFFFF", "card": "#FFFFFF", "card2": "#EEF2F6",
         "text": "#0F172A", "muted": "#64748B", "border": "#E2E8F0",
         "title": "#047857", "field": "#FFFFFF", "glow": "rgba(46,160,67,0.20)"}


def _qss(theme):
    c = DARK if theme == "dark" else LIGHT
    return (
        "#Window { background: " + c['bg'] + "; }"
        " #Hero { background: " + c['bg2'] + ";"
        " border-bottom: 1px solid " + c['border'] + "; }"
        " #StepBadge { background: " + c['card2'] + "; border: 1px solid " + c['border'] + ";"
        " border-radius: 11px; }"
        # The page content + scroll viewport MUST have an explicit dark/light
        # background — otherwise a bare QWidget paints the platform default
        # (white on Windows) and you get „white parts“ in dark mode.
        " QWidget#Page, QScrollArea#PageScroll, QScrollArea#PageScroll > QWidget > QWidget {"
        " background: " + c['bg'] + "; }"
        " QScrollArea#PageScroll { border: none; background: " + c['bg'] + "; }"
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
        " QLabel#VerBadge { font-size: 12px; font-weight: 700; color: " + c['title'] + ";"
        " background: " + c['card2'] + "; border: 1px solid " + c['border'] + ";"
        " border-radius: 10px; padding: 6px 14px; max-width: 380px; }"
        " #Card, #FeatCard { background: qlineargradient(x1:0,y1:0,x2:0,y2:1,"
        " stop:0 " + c['card'] + ", stop:1 " + c['bg2'] + ");"
        " border: 1px solid " + c['border'] + "; border-radius: 12px; }"
        " #FeatCard:hover { border-color: " + ACCENT + ";"
        " background: qlineargradient(x1:0,y1:0,x2:0,y2:1,"
        " stop:0 " + c['card2'] + ", stop:1 " + c['card'] + "); }"
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


# The installed application is a runtime payload, not a source checkout.  Keep
# this as an allow-list so a newly added developer directory can never silently
# start shipping to users.  Matching is case-insensitive because the archive
# says ``ui`` while an existing Windows checkout may expose it as ``UI``.
_RUNTIME_ROOT_DIRS = frozenset({"assets", "core", "languages", "plugins", "sdk", "ui"})
_RUNTIME_ROOT_FILES = frozenset({
    "main.py", "bootloader.py", "requirements.txt", "readme.md", "license",
    "license.md", "copying",
})
_RUNTIME_BLOCKED_PARTS = frozenset({
    "__pycache__", ".git", ".github", ".agents", ".claude", ".codex",
    ".idea", ".pytest_cache", ".vscode", ".zcode", "build", "dist",
    "docs", "node_modules", "tests", "tools", "ultrapilot.egg-info",
})
_RUNTIME_BLOCKED_SUFFIXES = (".pyc", ".pyo", ".log", ".msi", ".exe", ".spec", ".egg-info")
_SDK_DLL_NAMES = ("scs-telemetry.dll", "scs_sdk_controller.dll", "ets2la_plugin.dll")


def _normalise_repo_path(path):
    """Return a safe repository-relative POSIX path, or ``""`` if unsafe."""
    value = str(path or "").replace("\\", "/").strip("/")
    parts = [part for part in value.split("/") if part not in ("", ".")]
    if not parts or any(part == ".." for part in parts):
        return ""
    return "/".join(parts)


def _is_runtime_payload_path(path):
    """Whether a GitHub tree entry belongs in an installed UltraPilot copy."""
    rel = _normalise_repo_path(path)
    if not rel:
        return False
    parts = rel.lower().split("/")
    if any(part in _RUNTIME_BLOCKED_PARTS for part in parts):
        return False
    if rel.lower().endswith(_RUNTIME_BLOCKED_SUFFIXES):
        return False
    if len(parts) == 1:
        return parts[0] in _RUNTIME_ROOT_FILES or parts[0] in _RUNTIME_ROOT_DIRS
    return parts[0] in _RUNTIME_ROOT_DIRS


def _copy_runtime_tree(src_root, dst_root, on_file=None):
    """Copy only the allow-listed runtime payload from a repository checkout."""
    copied = 0
    for root, dirs, files in os.walk(src_root):
        rel_root = os.path.relpath(root, src_root)
        rel_root = "" if rel_root == "." else rel_root.replace("\\", "/")
        # Prune known non-runtime subtrees before os.walk descends into them.
        dirs[:] = [d for d in dirs if _is_runtime_payload_path(
            (rel_root + "/" + d).strip("/"))]
        for name in files:
            rel = (rel_root + "/" + name).strip("/")
            if not _is_runtime_payload_path(rel):
                continue
            src = os.path.join(root, name)
            dest = os.path.join(dst_root, *rel.split("/"))
            os.makedirs(os.path.dirname(_long_path(dest)), exist_ok=True)
            shutil.copy2(_long_path(src), _long_path(dest))
            copied += 1
            if on_file is not None:
                on_file(copied, rel)
    return copied


def _remove_legacy_development_payload(install_path):
    """Remove developer-only entries left by installers older than this filter."""
    root = os.path.abspath(str(install_path or ""))
    source_root = os.path.abspath(os.path.dirname(__file__))
    if (not root or root == os.path.abspath(os.path.sep)
            or os.path.normcase(root) == os.path.normcase(source_root)
            or not os.path.isfile(os.path.join(root, "main.py"))):
        return []
    names = (
        ".agents", ".claude", ".codex", ".git", ".github", ".idea",
        ".pytest_cache", ".vscode", ".zcode", "build", "dist", "docs",
        "tests", "tools", "UltraPilot.egg-info", "__pycache__",
        "installer.py", "build_installer.py", "freeze_app.py",
        "UltraPilot_Installer.spec", ".gitignore", "instbuild.log", "msi.log",
    )
    removed = []
    for name in names:
        target = os.path.abspath(os.path.join(root, name))
        if os.path.dirname(target) != root:
            continue
        try:
            if os.path.isdir(target):
                shutil.rmtree(target)
                removed.append(name)
            elif os.path.isfile(target):
                os.remove(target)
                removed.append(name)
        except OSError:
            # A locked developer artifact is harmless; repair will report what
            # it could clean without failing the usable runtime installation.
            continue
    return removed


def _sdk_plugins_dir(target):
    """Resolve current and legacy SDK target formats to a plugins directory.

    New records contain the game root. Older 0.4.1 records accidentally stored
    ``<game>/bin`` and a few development builds stored the plugins directory
    itself. Supporting all three formats makes repair/uninstall deterministic.
    """
    value = os.path.normpath(str(target or "").strip())
    if not value or value == ".":
        return ""
    name = os.path.basename(value).lower()
    parent = os.path.dirname(value)
    if name == "plugins" and os.path.basename(parent).lower() == "win_x64":
        return value
    if name == "win_x64" and os.path.basename(parent).lower() == "bin":
        return os.path.join(value, "plugins")
    if name == "bin":  # legacy installer record
        return os.path.join(value, "win_x64", "plugins")
    return os.path.join(value, "bin", "win_x64", "plugins")


def _sdk_game_root(target):
    """Return a canonical game root for any supported SDK target format."""
    plugins = _sdk_plugins_dir(target)
    if not plugins:
        return ""
    return os.path.dirname(os.path.dirname(os.path.dirname(plugins)))


def _sdk_plugin_dirs(rec=None, include_detected=True):
    """Collect de-duplicated SDK plugin directories from record and detection."""
    candidates = list((rec or {}).get("sdk_targets") or [])
    if include_detected:
        try:
            from core.sdk.game_utils import find_scs_games
            candidates.extend(find_scs_games())
        except Exception:
            pass
    result = []
    seen = set()
    for target in candidates:
        plugins = _sdk_plugins_dir(target)
        key = os.path.normcase(os.path.abspath(plugins)) if plugins else ""
        if key and key not in seen:
            seen.add(key)
            result.append(plugins)
    return result


def _write_install_record(rec, path=None):
    """Persist an install record atomically so repair cannot leave partial JSON."""
    target = path or RECORD_PATH
    parent = os.path.dirname(target)
    if parent:
        os.makedirs(parent, exist_ok=True)
    tmp = target + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(rec, handle, indent=2)
        handle.flush()
        try:
            os.fsync(handle.fileno())
        except OSError:
            pass
    os.replace(tmp, target)


def _update_sdk_targets(rec, installed_plugin_dirs, path=None):
    """Store canonical game roots returned by ``install_game_dlls``."""
    roots = []
    seen = set()
    for target in installed_plugin_dirs or []:
        root = _sdk_game_root(target)
        key = os.path.normcase(os.path.abspath(root)) if root else ""
        if key and key not in seen:
            seen.add(key)
            roots.append(root)
    rec["sdk_targets"] = roots
    _write_install_record(rec, path=path)
    return roots


def _long_path(path):
    """Add the ``\\\\?\\`` prefix on Windows so paths over MAX_PATH (260) work.

    Without it deeply nested files from the GitHub zip fail to write with a
    cryptic „cannot unpack file“ / WinError 3,206. No-op on non-Windows."""
    if sys.platform == "win32":
        p = os.path.abspath(path)
        if not p.startswith("\\\\?\\") and (len(p) >= 260 or " " in p):
            return "\\\\?\\" + p
    return path


def _dir_size_mb(path):
    """Total size in MB of every file under ``path`` (for install log stats)."""
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total / (1024 * 1024)


def _count_files(path):
    """Number of files under ``path`` (for install log stats)."""
    n = 0
    for _root, _dirs, files in os.walk(path):
        n += len(files)
    return n


def _installer_commit():
    """Short git commit SHA of the installer's own folder, or '' if not a git
    checkout (e.g. a built exe). Used for the version badge on the welcome page."""
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        out = subprocess.run(["git", "-C", here, "rev-parse", "--short=7", "HEAD"],
                             capture_output=True, text=True, timeout=6)
        if out.returncode == 0:
            match = re.search(r"(?i)\b[0-9a-f]{7,40}\b", out.stdout)
            return match.group(0)[:7].lower() if match else ""
    except Exception:
        pass
    return ""


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
        # Tracked for the install record so the uninstaller knows what it can
        # safely remove (Python auto-installed by us, SDK target game folders).
        self.python_installed_by_installer = False
        self.sdk_targets = []
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
                r = subprocess.run([*c, "--version"], capture_output=True, text=True, timeout=10, creationflags=_NO_WIN)
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
                                    capture_output=True, text=True, timeout=15, creationflags=_NO_WIN)
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
                self.python_installed_by_installer = True
                return True
        self.log.emit(self.t["py_manual"])
        return False

    # ---------------------------------------------------------------- Sources
    def _try_git_clone(self):
        import time
        try:
            tmp = self.install_path + "_clone"
            if os.path.isdir(tmp):
                shutil.rmtree(tmp, ignore_errors=True)
            self.log.emit("  ▸ git clone --depth 1 " + REPO_URL)
            t0 = time.time()
            r = subprocess.run(["git", "clone", "--depth", "1", "--progress",
                                REPO_URL, tmp],
                               capture_output=True, text=True, timeout=600,
                               creationflags=_NO_WIN)
            dt = time.time() - t0
            if r.returncode != 0:
                # Surface git's own error text (auth, network, repo not found).
                err = (r.stderr or "").strip().splitlines()
                msg = err[-1] if err else "git returncode " + str(r.returncode)
                self.log.emit(self.t["src_err"].format(err=msg))
                return False
            if not os.path.exists(os.path.join(tmp, "main.py")):
                self.log.emit(self.t["src_err"].format(err="clone succeeded but main.py missing"))
                return False
            # A clone contains repository metadata, tests and developer tools.
            # Copy only the explicit runtime allow-list into the application.
            nfiles = _copy_runtime_tree(
                tmp, self.install_path,
                on_file=lambda count, rel: self.log.emit(
                    "    [{:>4}] {}".format(count, rel)),
            )
            self.log.emit("    ✓ nakopírovaných {} súborov".format(nfiles))
            shutil.rmtree(tmp, ignore_errors=True)
            mb = _dir_size_mb(self.install_path)
            speed = (mb / dt) if dt > 0 else 0.0
            self.log.emit("  ✓ git clone — {} súborov, {:.1f} MB ({:.1f} MB/s, {:.0f}s)".format(
                _count_files(self.install_path), mb, speed, dt))
            self.log.emit(self.t["src_git_ok"])
            return True
        except Exception as e:
            self.log.emit(self.t["src_err"].format(err=str(e)))
        return False

    def _try_zip_archive(self):
        import zipfile, io, time, traceback
        try:
            t0 = time.time()
            self.log.emit("  ▸ Pripájam sa ku GitHubu…")
            data = None
            errors = []

            # Try both GitHub endpoints. The regular archive URL may be blocked
            # by a redirect/proxy while codeload works directly (and vice versa).
            for url in (CODELOAD_URL, ARCHIVE_URL):
                self.log.emit("  [INF] Zdroj: " + url)
                try:
                    chunks = bytearray()
                    try:
                        import requests
                        resp = requests.get(url, headers=_github_headers(), timeout=180,
                                            stream=True, allow_redirects=True)
                        try:
                            if resp.status_code != 200:
                                raise RuntimeError("HTTP " + str(resp.status_code))
                            total = int(resp.headers.get("Content-Length") or 0)
                            for chunk in resp.iter_content(chunk_size=65536):
                                if chunk:
                                    chunks.extend(chunk)
                                    pct = len(chunks) * 100 // total if total else 0
                                    self.status.emit(
                                        "Sťahujem z GitHubu… {}% ({:.1f} MB)".format(
                                            pct, len(chunks) / (1024 * 1024)))
                            if total and len(chunks) != total:
                                raise RuntimeError(
                                    "neúplný prenos: {} z {} bajtov".format(
                                        len(chunks), total))
                        finally:
                            close = getattr(resp, "close", None)
                            if callable(close):
                                close()
                    except Exception as request_error:
                        # Standard-library fallback is bundled with every Python
                        # and therefore also works in the one-file installer.
                        self.log.emit("  [WRN] requests transport zlyhal, skúšam urllib: "
                                      + str(request_error))
                        # Discard every byte from the failed transport. Appending
                        # a fresh ZIP to a partial ZIP creates an archive that can
                        # download to 100% but can never be opened.
                        chunks = bytearray()
                        from urllib.request import Request, urlopen
                        req = Request(url, headers={**_github_headers(),
                                      "User-Agent": "UltraPilot-Installer/" + APP_VERSION})
                        with urlopen(req, timeout=180) as resp:
                            status_code = getattr(resp, "status", None)
                            if status_code not in (None, 200):
                                raise RuntimeError("HTTP " + str(status_code))
                            total = int(resp.headers.get("Content-Length") or 0)
                            while True:
                                chunk = resp.read(65536)
                                if not chunk:
                                    break
                                chunks.extend(chunk)
                                pct = len(chunks) * 100 // total if total else 0
                                self.status.emit("Sťahujem z GitHubu… {}% ({:.1f} MB)".format(
                                    pct, len(chunks) / (1024 * 1024)))
                            if total and len(chunks) != total:
                                raise RuntimeError(
                                    "neúplný prenos: {} z {} bajtov".format(
                                        len(chunks), total))
                    if len(chunks) < 1024:
                        raise RuntimeError("GitHub vrátil prázdny alebo neúplný archív")
                    candidate = bytes(chunks)
                    if not candidate.startswith(b"PK"):
                        raise RuntimeError("odpoveď nie je ZIP archív")
                    # Validate each endpoint before accepting it. A corrupt
                    # codeload response must fall through to the regular GitHub
                    # archive endpoint instead of aborting the whole ZIP method.
                    with zipfile.ZipFile(io.BytesIO(candidate)) as candidate_zip:
                        bad = candidate_zip.testzip()
                        if bad is not None:
                            raise RuntimeError("poškodený zip pri " + str(bad))
                        candidate_files = [
                            name for name in candidate_zip.namelist()
                            if not name.endswith("/")]
                        if not any(name.replace("\\", "/").endswith("/main.py")
                                   or name == "main.py"
                                   for name in candidate_files):
                            raise RuntimeError("ZIP neobsahuje main.py")
                    data = candidate
                    break
                except Exception as de:
                    errors.append(url + ": " + str(de))
                    self.log.emit("  [WRN] Endpoint zlyhal: " + str(de))
            if data is None:
                raise RuntimeError("; ".join(errors) or "GitHub download failed")
            dt = time.time() - t0
            mb = len(data) / (1024 * 1024)
            speed = (mb / dt) if dt > 0 else 0.0
            self.log.emit("  ✓ Stiahnuté: {:.1f} MB ({:.1f} MB/s, {:.0f}s)".format(
                mb, speed, dt))
            # Validate the zip is intact before extracting (catches truncated
            # downloads that would raise „cannot unpack file“ mid-extract).
            zf = zipfile.ZipFile(io.BytesIO(data))
            bad = zf.testzip()
            if bad is not None:
                self.log.emit(self.t["src_err"].format(
                    err="poškodený zip pri " + str(bad)))
                return False
            # Extract file-by-file with a long-path prefix and per-file error
            # isolation: one locked/colliding file must not abort the whole
            # install. Track failures and report them at the end.
            prefix = ""
            archive_names = [n for n in zf.namelist() if not n.endswith("/")]
            if archive_names:
                prefix = archive_names[0].split("/")[0] if "/" in archive_names[0] else ""
            names = []
            for archive_name in archive_names:
                rel = (archive_name[len(prefix) + 1:]
                       if prefix and archive_name.startswith(prefix + "/")
                       else archive_name)
                rel = _normalise_repo_path(rel)
                if rel and _is_runtime_payload_path(rel):
                    names.append((archive_name, rel))
            self.log.emit("  ▸ Runtime balík: {} súborov (vývojové súbory sa neinštalujú).".format(
                len(names)))
            failed = []
            extracted = 0
            for n, rel in names:
                dest = os.path.join(self.install_path, rel)
                # \\?\ opts out of the 260-char MAX_PATH limit on Windows so
                # deeply nested files don't fail with „cannot unpack file“.
                dest_long = _long_path(dest)
                try:
                    os.makedirs(os.path.dirname(dest_long) or _long_path(self.install_path),
                                exist_ok=True)
                    with zf.open(n) as src, open(dest_long, "wb") as out:
                        out.write(src.read())
                    extracted += 1
                    # Explicitly log every downloaded/extracted file. This is
                    # intentionally verbose so the user can see exactly what
                    # the installer placed on disk.
                    self.log.emit("    [{:>4}/{:>4}] {}".format(
                        extracted, len(names), rel.replace("\\", "/")))
                    self.status.emit("Rozbaľujem súbory… {}/{}".format(extracted, len(names)))
                except Exception as fe:
                    failed.append(rel + " (" + str(fe) + ")")
            # Flatten the "<repo>-main/" wrapper if the zip was nested.
            root = os.path.join(self.install_path, "ets2la-main")
            if os.path.isdir(root):
                for item in os.listdir(root):
                    shutil.move(os.path.join(root, item),
                                os.path.join(self.install_path, item))
                shutil.rmtree(root, ignore_errors=True)
            if failed:
                self.log.emit("  ⚠ {} súborov sa nepodarilo rozbaliť:".format(len(failed)))
                for f in failed[:10]:
                    self.log.emit("     – " + f)
                if len(failed) > 10:
                    self.log.emit("     … a ďalších {}".format(len(failed) - 10))
            self.log.emit("  ✓ Rozbalených {} súborov.".format(extracted))
            required = ("main.py", "bootloader.py", "requirements.txt")
            missing_required = [name for name in required if not os.path.isfile(
                os.path.join(self.install_path, name))]
            if failed or extracted != len(names) or missing_required:
                details = "neúplný runtime ZIP"
                if missing_required:
                    details += "; chýba " + ", ".join(missing_required)
                self.log.emit(self.t["src_err"].format(err=details))
                return False
            self.log.emit(self.t["src_zip_ok"])
            return True
        except Exception as e:
            # Full traceback in debug so „cannot unpack file“/WinError has a
            # clear root cause in the log instead of a bare message.
            logging.debug("zip install failed:\n%s", traceback.format_exc())
            self.log.emit(self.t["src_err"].format(err=str(e)))
        return False

    def _try_raw_file_by_file(self):
        """Last-resort: list the tree via Contents API and fetch each blob raw."""
        import requests, time
        try:
            self.log.emit("  ▸ Získavam zoznam súborov z GitHub API…")
            r = requests.get(CONTENTS_API, headers=_github_headers(), timeout=30)
            if r.status_code != 200:
                self.log.emit(self.t["src_err"].format(err="API HTTP " + str(r.status_code)))
                return False
            tree = r.json().get("tree", [])
            blobs = [e for e in tree if e.get("type") == "blob"]

            todo = [e for e in blobs if _is_runtime_payload_path(e["path"])]
            total = len(todo)
            self.log.emit("  ▸ Stahujem {} súborov jeden po druhom…".format(total))
            count = 0
            failed = []
            total_bytes = 0
            t0 = time.time()
            for i, entry in enumerate(todo, 1):
                path = entry["path"]
                dest = os.path.join(self.install_path, path)
                os.makedirs(os.path.dirname(_long_path(dest)) or _long_path(self.install_path),
                            exist_ok=True)
                rr = requests.get(RAW_BASE + path, headers=_github_headers(), timeout=60)
                if rr.status_code == 200:
                    with open(_long_path(dest), "wb") as f:
                        f.write(rr.content)
                    total_bytes += len(rr.content)
                    count += 1
                    self.log.emit("    [{:>4}/{:>4}] {}".format(count, total, path))
                else:
                    failed.append("{} (HTTP {})".format(path, rr.status_code))
                if i % 25 == 0 or i == total:
                    self.status.emit("Sťahujem súbory… {}/{} ({:.1f} MB)".format(
                        i, total, total_bytes / (1024 * 1024)))
            dt = time.time() - t0
            mb = total_bytes / (1024 * 1024)
            speed = (mb / dt) if dt > 0 else 0.0
            self.log.emit("  ✓ Stiahnutých {} súborov, {:.1f} MB ({:.1f} MB/s)".format(
                count, mb, speed))
            if count == total and total > 0:
                self.log.emit(self.t["src_raw_ok"].format(n=count))
                return os.path.exists(os.path.join(self.install_path, "main.py"))
            if failed:
                self.log.emit("  ⚠ Neúplný runtime prenos: {} z {} súborov zlyhalo.".format(
                    len(failed), total))
                for item in failed[:5]:
                    self.log.emit("     – " + item)
        except Exception as e:
            self.log.emit(self.t["src_err"].format(err=str(e)))
        return False

    def _finalise_runtime_payload(self):
        removed = _remove_legacy_development_payload(self.install_path)
        if removed:
            self.log.emit("  ✓ Odstránené staré vývojové položky: " + ", ".join(removed))
        # Installed copies have no .git directory. Persist the exact source
        # revision so the application never has to display a bare "build".
        commit = ""
        try:
            import requests
            response = requests.get(COMMIT_API, headers=_github_headers(), timeout=20)
            if response.status_code == 200:
                match = re.match(r"(?i)^[0-9a-f]{7,40}$",
                                 str(response.json().get("sha", "")).strip())
                commit = match.group(0)[:7].lower() if match else ""
            else:
                self.log.emit("  [WRN] Commit API HTTP {}.".format(response.status_code))
        except Exception as exc:
            self.log.emit("  [WRN] Nepodarilo sa zistiť commit: " + str(exc))
        if commit:
            marker = os.path.join(self.install_path, "commit.txt")
            temporary = marker + ".tmp"
            with open(temporary, "w", encoding="utf-8") as stream:
                stream.write(commit)
            os.replace(temporary, marker)
            self.log.emit("  ✓ Revízia aplikácie: " + commit)
        self.log.emit("  ✓ Runtime balík pripravený: {} súborov, {:.1f} MB.".format(
            _count_files(self.install_path), _dir_size_mb(self.install_path)))

    def _fetch_repo(self):
        """Fetch the runtime subset, falling back to clone/archive transports."""
        # The tree + raw transport is intentionally first: unlike a repository
        # archive it downloads only allow-listed runtime blobs. Git and ZIP stay
        # as resilient fallbacks for API limits or restrictive proxies.
        self.status.emit(self.t["src_try_raw"])
        if self._try_raw_file_by_file():
            self._finalise_runtime_payload()
            return True
        if shutil.which("git"):
            self.status.emit(self.t["src_try_git"])
            if self._try_git_clone():
                self._finalise_runtime_payload()
                return True
        else:
            self.log.emit("  [INF] Git nie je potrebný — používam priamy GitHub prenos.")
        self.status.emit(self.t["src_try_zip"])
        if self._try_zip_archive():
            self._finalise_runtime_payload()
            return True
        self.log.emit(self.t["src_fail"])
        return False

    # ---------------------------------------------------------------- Deps
    def _pip_install(self):
        req = os.path.join(self.install_path, "requirements.txt")
        py = self._real_python()
        if not py:
            self.log.emit("  Python nebol nájdený — závislosti preskočené.")
            return False
        try:
            self.log.emit("  ▸ Používam Python: " + py[0])
            # Parse requirements.txt and install each package individually so
            # the user sees detailed progress (one log line per package)
            # instead of a single silent pip run.
            pkgs = []
            if os.path.exists(req):
                with open(req, encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if not line or line.startswith("#"):
                            continue
                        if ";" in line:  # env marker (e.g. pywin32 for Windows)
                            line = line.split(";")[0]
                        pkgs.append(line)
            # Ensure the 3D-view extras are present too.
            for extra in ("pyqtgraph", "PyOpenGL"):
                if extra.lower() not in " ".join(pkgs).lower():
                    pkgs.append(extra)
            self.log.emit("  ▸ Nainštalujem {} balíkov…".format(len(pkgs)))
            failed = []
            for i, pkg in enumerate(pkgs, 1):
                self.status.emit("pip install {}/{}: {}".format(i, len(pkgs), pkg.split(">")[0].split("=")[0]))
                self.log.emit("    [{:>2}/{:>2}] {} …".format(i, len(pkgs), pkg))
                try:
                    result = subprocess.run(
                        [*py, "-m", "pip", "install", pkg],
                        capture_output=True, text=True, timeout=900,
                        creationflags=_NO_WIN)
                    if result.returncode != 0:
                        detail = (result.stderr or result.stdout or
                                  "pip exit code {}".format(result.returncode))
                        detail = " ".join(detail.strip().split())[-700:]
                        failed.append((pkg, detail))
                        self.log.emit("      [CHYBA] {}: {}".format(pkg, detail))
                except Exception as pe:
                    failed.append((pkg, str(pe)))
                    self.log.emit("      ⚠ {}".format(pe))
            if failed:
                self.log.emit("  [CHYBA] Nenainštalované povinné balíky: " +
                              ", ".join(pkg for pkg, _ in failed))
                return False
            self.log.emit("  ✓ Závislosti nainštalované ({} balíkov).".format(len(pkgs)))
            return True
        except Exception as e:
            self.log.emit("  problém s pip (" + str(e) + ") — nainštaluj manuálne.")
            return False

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
                               capture_output=True, creationflags=_NO_WIN)
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
            if not self._pip_install():
                raise RuntimeError(
                    "Inštalácia povinných Python balíkov zlyhala. "
                    "Presný balík a odpoveď pipu sú uvedené vyššie.")
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
                    # Remember the game roots so the uninstaller can offer to
                    # remove the SDK DLLs later. folders are .../plugins dirs.
                    for plugins_dir in folders:
                        game_root = _sdk_game_root(plugins_dir)
                        if game_root and game_root not in self.sdk_targets:
                            self.sdk_targets.append(game_root)
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
                rec = {
                    "install_path": self.install_path,
                    "exe_path": exe_path,
                    "mode": mode,
                    "version": APP_VERSION,
                    "python_installed_by_installer": self.python_installed_by_installer,
                    "sdk_targets": self.sdk_targets,
                }
                _write_install_record(rec)
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
        # Guard against no-op toggles (covers both the idle and animating case).
        if dark == self._dark:
            return
        self._dark = dark
        target = 1.0 if dark else 0.0
        if self._anim is not None:
            try:
                self._anim.stop()
            except Exception:
                pass
            self._anim = None
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
        # Defer the signal to the next event-loop cycle. Emitting synchronously
        # here would trigger setStyleSheet on the parent window from inside our
        # own mouseReleaseEvent, which Qt can take badly (the toggle widget may
        # be re-laid-out / reparented mid-event → crash). QTimer.singleShot(0)
        # guarantees we return from this call before the theme switch runs.
        dark_now = self._dark
        QTimer.singleShot(0, lambda: self.toggled.emit(dark_now))

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            self.set_dark(not self._dark)
            e.accept()
            return
        super().mouseReleaseEvent(e)

    def mousePressEvent(self, e):
        # Accept the press as well as the release.  The old draggable custom
        # header treated an ignored press as a window move and swallowed the
        # matching release before the theme could change.
        if e.button() == Qt.MouseButton.LeftButton:
            e.accept()
            return
        super().mousePressEvent(e)

    def paintEvent(self, _e):
        # Always pair QPainter creation with end() in a finally block — an open
        # painter left behind by an exception in the middle of paint crashes the
        # next repaint with „A paint device can only be painted by one painter“.
        p = QPainter(self)
        try:
            self._draw(p)
        except Exception:
            pass
        finally:
            p.end()

    def _draw(self, p: QPainter):
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        w, h = self.width(), self.height()
        # k = 0 → fully light (sun), k = 1 → fully dark (moon).
        k = self._knob

        # --- Track (the pill background) ------------------------------------
        sun_bg = QColor("#FBBF24")     # warm amber when light
        moon_bg = QColor("#334155")    # slate when dark (matches the grey theme)
        bg = QColor(
            int(sun_bg.red()   + (moon_bg.red()   - sun_bg.red())   * k),
            int(sun_bg.green() + (moon_bg.green() - sun_bg.green()) * k),
            int(sun_bg.blue()  + (moon_bg.blue()  - sun_bg.blue())  * k),
        )
        p.setBrush(bg)
        p.setPen(QPen(QColor(255, 255, 255, 40), 1))
        p.drawRoundedRect(1, 1, w - 2, h - 2, h / 2 - 1, h / 2 - 1)

        # --- Knob (the white circle that slides) ----------------------------
        margin = 4
        knob_d = max(8, h - margin * 2)
        x = margin + k * (w - margin * 2 - knob_d)
        cx = x + knob_d / 2
        cy = h / 2
        # Subtle drop shadow under the knob for depth.
        p.setBrush(QColor(0, 0, 0, 38))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(int(x + 1), int(margin + 1), knob_d, knob_d)
        p.setBrush(QColor("#FFFFFF"))
        p.drawEllipse(int(x), margin, knob_d, knob_d)

        # --- Icons (cross-fade inside the knob) -----------------------------
        # SUN (fades out as k → 1). A small yellow disc + 8 short rays.
        sun_alpha = max(0.0, 1.0 - k)
        if sun_alpha > 0.01:
            yellow = QColor("#F59E0B")
            yellow.setAlphaF(sun_alpha)
            p.setBrush(yellow)
            p.setPen(Qt.PenStyle.NoPen)
            disc_r = knob_d * 0.22
            p.drawEllipse(QPointF(cx, cy), disc_r, disc_r)
            ray = QPen(yellow, max(1.0, knob_d * 0.06), Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
            p.setPen(ray)
            r1 = knob_d * 0.30
            r2 = knob_d * 0.42
            for ang in range(0, 360, 45):
                a = math.radians(ang)
                p.drawLine(
                    QPointF(cx + math.cos(a) * r1, cy + math.sin(a) * r1),
                    QPointF(cx + math.cos(a) * r2, cy + math.sin(a) * r2),
                )

        # MOON (fades in as k → 1). Drawn as a yellow disc, then a disc in the
        # knob's white colour offset to one side bites out a crescent.
        moon_alpha = max(0.0, k)
        if moon_alpha > 0.01:
            moon = QColor("#FBBF24")
            moon.setAlphaF(moon_alpha)
            p.setBrush(moon)
            p.setPen(Qt.PenStyle.NoPen)
            mr = knob_d * 0.30
            p.drawEllipse(QPointF(cx - mr * 0.15, cy), mr, mr)
            # The bite: same colour as the knob so the crescent reads cleanly.
            p.setBrush(QColor("#FFFFFF"))
            p.drawEllipse(QPointF(cx + mr * 0.55, cy - mr * 0.20), mr * 0.95, mr * 0.95)


class InstallerLanguageIcon(QWidget):
    """Crisp custom language mark used on the installer welcome page."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(38, 38)
        self.setAccessibleName("Language")
        self.setToolTip("Jazyk / Language")

    def paintEvent(self, _event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor("#10B981"))
            painter.drawEllipse(QRectF(1, 1, 36, 36))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.setPen(QPen(QColor("#FFFFFF"), 1.55,
                                Qt.PenStyle.SolidLine,
                                Qt.PenCapStyle.RoundCap))

            # Globe: outer meridian, latitude lines and the narrow longitude.
            globe = QRectF(8.5, 8.5, 18, 18)
            painter.drawEllipse(globe)
            painter.drawEllipse(QRectF(14, 8.5, 7, 18))
            painter.drawArc(QRectF(8.5, 12, 18, 7), 0, 180 * 16)
            painter.drawArc(QRectF(8.5, 16, 18, 7), 180 * 16, 180 * 16)

            # Small overlapping conversation tile makes the symbol explicitly
            # about languages rather than a generic world/map icon.
            bubble = QRectF(20, 20, 12, 9)
            painter.setPen(QPen(QColor("#D9FFF2"), 1.0))
            painter.setBrush(QColor("#087D60"))
            painter.drawRoundedRect(bubble, 3, 3)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor("#FFFFFF"))
            painter.drawEllipse(QPointF(24, 24.5), 1.0, 1.0)
            painter.drawEllipse(QPointF(28, 24.5), 1.0, 1.0)
        finally:
            painter.end()


class InstallerStepBadge(QWidget):
    """Painted badge whose circle cannot be clipped by stylesheet metrics."""

    def __init__(self, text, parent=None):
        super().__init__(parent)
        self._text = str(text)
        self._state = "pending"
        self._theme = "light"
        self.setFixedSize(32, 32)

    def set_state(self, state, theme, text=None):
        self._state = str(state)
        self._theme = str(theme)
        if text is not None:
            self._text = str(text)
        self.update()

    def text(self):
        return self._text

    def paintEvent(self, _event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            if self._state == "active":
                background, foreground, border = ACCENT, "#FFFFFF", ACCENT
            elif self._state == "done":
                background, foreground, border = SUCCESS_DARK, "#FFFFFF", SUCCESS_DARK
            else:
                pal = DARK if self._theme == "dark" else LIGHT
                background, foreground, border = pal["card2"], pal["muted"], pal["border"]
            painter.setPen(QPen(QColor(border), 1.2))
            painter.setBrush(QColor(background))
            painter.drawEllipse(QRectF(1.5, 1.5, 29.0, 29.0))
            painter.setPen(QColor(foreground))
            painter.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self._text)
        finally:
            painter.end()


class InstallerStepCell(QWidget):
    """Clickable step-rail cell used to revisit installation status safely."""

    clicked = pyqtSignal(int)

    def __init__(self, index, parent=None):
        super().__init__(parent)
        self.index = int(index)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self.index)
            event.accept()
            return
        super().mouseReleaseEvent(event)


def _esc(text: str) -> str:
    """HTML-escape a string so log output can't inject markup."""
    return (str(text)
            .replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;"))


def _append_log_preserving_scroll(view, html):
    """Append while following only when the reader is already at the bottom."""
    bar = view.verticalScrollBar()
    old_value = bar.value()
    follow = old_value >= max(0, bar.maximum() - 2)
    view.append(html)
    if follow:
        bar.setValue(bar.maximum())
    else:
        # QTextEdit.append() moves its cursor to the end; restore the reader's
        # viewport so inspecting even one older line is never interrupted.
        bar.setValue(min(old_value, bar.maximum()))


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

    def __init__(self, lang="sk", theme="light"):
        super().__init__()
        self.setObjectName("Window")
        self.lang = lang
        self.theme = theme
        self.exe_path = ""
        self._worker = None
        self._cur = 0
        self.setWindowTitle(TR[self.lang]["win"])
        # Use the real Windows title bar and its native minimise, maximise and
        # close controls.  The former frameless window lost all three when the
        # decorative coloured dots were removed.
        self.setWindowFlags(
            Qt.WindowType.Window
            | Qt.WindowType.WindowTitleHint
            | Qt.WindowType.WindowSystemMenuHint
            | Qt.WindowType.WindowMinMaxButtonsHint
            | Qt.WindowType.WindowCloseButtonHint)
        self.resize(920, 700)
        self.setMinimumSize(820, 640)
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
        hero = QFrame(self)
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
        rail.setFixedHeight(64)
        h = QHBoxLayout(rail)
        h.setContentsMargins(28, 12, 28, 12)
        h.setSpacing(8)
        self._step_labels = []
        steps = TR[self.lang]["steps"]
        for i, name in enumerate(steps):
            badge = InstallerStepBadge(str(i + 1))
            lbl = QLabel(name)
            lbl.setObjectName("StepLabel")
            cell = QHBoxLayout()
            cell.setSpacing(8)
            cell.addWidget(badge)
            cell.addWidget(lbl)
            wrap = InstallerStepCell(i)
            wrap.setLayout(cell)
            wrap.setStyleSheet("border:none;")
            wrap.clicked.connect(self._on_step_clicked)
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
        # Give the page an objectName + autofill so it NEVER falls back to the
        # platform's default WHITE window colour (the root cause of „white
        # parts“ in dark mode — a bare QWidget in a scroll area ignores QSS bg).
        scroll.setObjectName("PageScroll")
        inner = QWidget()
        inner.setObjectName("Page")
        inner.setAutoFillBackground(True)
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(38, 24, 38, 24)
        lay.setSpacing(14)
        scroll.setWidget(inner)
        scroll.viewport().setAutoFillBackground(True)
        return scroll, lay

    # ----------------------------------------------------------------- pages
    def _build_welcome(self):
        scroll, lay = self._page_frame()
        lay.setContentsMargins(30, 22, 30, 20)
        lay.setSpacing(12)

        # A calm, centred introduction keeps the first page focused and avoids
        # decorative motion while the user decides whether to continue.
        hero = QFrame()
        hero.setObjectName("Card")
        hero.setMinimumHeight(220)
        hcol = QVBoxLayout(hero)
        hcol.setContentsMargins(54, 28, 54, 28)
        hcol.setSpacing(9)
        hcol.addStretch()
        eyebrow = QLabel("ULTRAPILOT  ·  SETUP")
        eyebrow.setObjectName("SectionTitle")
        eyebrow.setAlignment(Qt.AlignmentFlag.AlignCenter)
        hcol.addWidget(eyebrow)
        self.welcome_title = QLabel(TR[self.lang]["welcome_t"])
        self.welcome_title.setObjectName("Title")
        self.welcome_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.welcome_title.setWordWrap(True)
        hcol.addWidget(self.welcome_title)
        self.welcome_description = QLabel(TR[self.lang]["welcome_d"])
        self.welcome_description.setObjectName("Desc")
        self.welcome_description.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.welcome_description.setWordWrap(True)
        hcol.addWidget(self.welcome_description)
        hcol.addSpacing(5)
        chips = QHBoxLayout()
        chips.setSpacing(7)
        chips.addStretch()
        for text_value in (("Windows 10/11"), ("ETS2 / ATS"), ("x64")):
            chip = QLabel(text_value)
            chip.setObjectName("VerBadge")
            chip.setAlignment(Qt.AlignmentFlag.AlignCenter)
            chips.addWidget(chip)
        chips.addStretch()
        hcol.addLayout(chips)
        hcol.addStretch()
        lay.addWidget(hero)

        middle = QHBoxLayout()
        middle.setSpacing(12)

        feature_wrap = QFrame()
        feature_wrap.setObjectName("Card")
        feature_layout = QVBoxLayout(feature_wrap)
        feature_layout.setContentsMargins(16, 14, 16, 14)
        feature_layout.setSpacing(8)
        feat_title = QLabel(TR[self.lang]["feat_t"])
        feat_title.setObjectName("SectionTitle")
        feature_layout.addWidget(feat_title)
        feats = TR[self.lang]["feats"]
        for icon, name, fd in feats[:3]:
            row = QHBoxLayout()
            row.setSpacing(10)
            ic = QLabel(icon if icon else "•")
            ic.setFixedWidth(28)
            ic.setAlignment(Qt.AlignmentFlag.AlignCenter)
            nm = QLabel(name)
            nm.setObjectName("FeatName")
            row.addWidget(ic)
            row.addWidget(nm, stretch=1)
            ok = QLabel("✓")
            ok.setStyleSheet("color:" + SUCCESS + ";font-weight:800;")
            row.addWidget(ok)
            feature_layout.addLayout(row)
        feature_layout.addStretch()
        middle.addWidget(feature_wrap, stretch=1)

        language_card = QFrame()
        language_card.setObjectName("Card")
        language_layout = QVBoxLayout(language_card)
        language_layout.setContentsMargins(18, 14, 18, 14)
        language_layout.setSpacing(7)
        language_head = QHBoxLayout()
        self.language_mark = InstallerLanguageIcon(language_card)
        language_head.addWidget(self.language_mark)
        language_text = QVBoxLayout()
        language_text.setSpacing(0)
        cap = QLabel(TR[self.lang].get("language", TR[self.lang].get("lang", "Language")))
        cap.setObjectName("FeatName")
        language_text.addWidget(cap)
        lang_hint = QLabel("Jazyk tohto inštalátora" if self.lang == "sk"
                           else "Installer display language")
        lang_hint.setObjectName("Caption")
        lang_hint.setWordWrap(True)
        lang_hint.setMaximumWidth(280)
        language_text.addWidget(lang_hint)
        language_head.addLayout(language_text, stretch=1)
        language_layout.addLayout(language_head)
        self.lang_combo = QComboBox()
        self.lang_combo.setMinimumWidth(245)
        for code in ("sk", "en"):
            _ensure_lang_loaded(code)
            self.lang_combo.addItem(
                "{}  ·  {}%".format(_lang_name(code), _lang_coverage(code)), code)
        for i in range(self.lang_combo.count()):
            if self.lang_combo.itemData(i) == self.lang:
                self.lang_combo.setCurrentIndex(i)
                break
        self.lang_combo.currentIndexChanged.connect(self._on_lang_idx)
        language_layout.addWidget(self.lang_combo)
        language_layout.addStretch()
        middle.addWidget(language_card, stretch=1)
        lay.addLayout(middle)

        req_card = QFrame()
        req_card.setObjectName("Card")
        req_layout = QVBoxLayout(req_card)
        req_layout.setContentsMargins(16, 11, 16, 11)
        req_layout.setSpacing(7)
        req_title = QLabel(TR[self.lang]["req_t"])
        req_title.setObjectName("SectionTitle")
        req_layout.addWidget(req_title)
        req_grid = QGridLayout()
        req_grid.setHorizontalSpacing(18)
        req_grid.setVerticalSpacing(5)
        for index, item in enumerate(TR[self.lang]["req_items"]):
            lab = QLabel("✓  " + item)
            lab.setObjectName("Caption")
            lab.setWordWrap(True)
            lab.setMaximumWidth(370)
            req_grid.addWidget(lab, index // 2, index % 2)
        req_layout.addLayout(req_grid)
        lay.addWidget(req_card)

        lay.addStretch()
        installer_label = "Inštalátor" if self.lang == "sk" else "Installer"
        ver_text = "UltraPilot {}  ·  {} {}".format(
            APP_VERSION, installer_label, INSTALLER_VERSION)
        self.ver_lbl = QLabel(ver_text)
        self.ver_lbl.setObjectName("VerBadge")
        self.ver_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        version_row = QHBoxLayout()
        version_row.addStretch()
        version_row.addWidget(self.ver_lbl)
        version_row.addStretch()
        lay.addLayout(version_row)
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
        default = os.path.join(os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "Programs", "UltraPilot")
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
        self.log_view.setStyleSheet(
            "QTextEdit{background:#FFFFFF;color:#1F2937;border:1px solid #D5DAE1;"
            "border-radius:8px;padding:10px;font-family:'Cascadia Mono','Consolas';font-size:12px;}"
        )
        # Cap the buffer so a long install (lots of pip output) can't grow the
        # document unbounded and lag the UI; older lines drop off the top.
        try:
            self.log_view.document().setMaximumBlockCount(2000)
        except Exception:
            pass
        self.log_view.setPlaceholderText(TR[self.lang].get("log_placeholder",
            "Inštalácia zatiaľ nezačala — klikni „Nainštalovať“."))
        self.log_view.setMinimumHeight(180)
        lay.addWidget(self.log_view, stretch=1)
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
                    state = "active"
                elif done:
                    state = "done"
                else:
                    state = "pending"
                badge.set_state(state, self.theme, "✓" if done else str(i + 1))
                lbl.setStyleSheet("color:" + (c['title'] if active else c['muted']) +
                                  "; font-size:13px; font-weight:" + ("700" if active else "600") +
                                  "; padding:0; margin:0;")
        # Path status colour (objectName drives QSS, but re-apply to be safe).
        if hasattr(self, "path_status"):
            ok = self.path_status.objectName() == "DiskOk"
            col = SUCCESS if ok else WARN
            self.path_status.setStyleSheet("color:" + col + "; font-size:12px; font-weight:600;")

    def _on_step_clicked(self, index):
        """Allow safe review of the install log and completed summary.

        Forward steps remain locked until their work has happened. Once an
        installation worker exists, step 4 is always the authoritative log;
        after success, step 5 can be selected again without starting work.
        """
        index = int(index)
        if index == 3 and self._worker is not None:
            self._go_step(3)
        elif (index == 4 and self.exe_path
              and self._worker is not None and not self._worker.isRunning()):
            self._go_step(4)
        elif 0 <= index < self._cur and self._cur < 3:
            self._go_step(index)

    def _on_lang_idx(self, idx):
        """Language combo changed — ``idx`` is the row; data holds the code."""
        code = self.lang_combo.itemData(idx) if idx >= 0 else "sk"
        if code:
            self.lang = code
            _ensure_lang_loaded(code)
            # Rebuild the installer immediately so every page, the step rail
            # and footer use the selected translation.
            fresh = InstallerWindow(lang=code, theme=self.theme)
            fresh.move(self.pos())
            fresh.show()
            self._language_replacement = fresh
            fresh._language_previous = self
            self.close()

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
            # Finish clicked — launch the app (if requested) BEFORE closing so
            # a startfile/DETACHED_PROCESS hiccup can't take the installer down
            # with it. closeEvent no longer launches anything.
            if hasattr(self, "launch_chk") and self.launch_chk.isChecked() and self.exe_path:
                try:
                    self._launch_app()
                except Exception as e:
                    logging.debug("launch on finish failed: %s", e)
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
        """Append one line to the install log with a timestamp and colour coding.

        Categories (by leading marker / shape):
          ✓ ✔ → green (success)
          ✗   → red   (error)
          ⚠   → amber (warning)
          ─── section ───  → muted, full-width divider
          ' ' (leading space) → muted (subprocess / sub-output)
          other → default text
        Auto-scrolls to the bottom and caps the buffer so very long installs
        can't slow the UI down."""
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        # Section dividers: lines wrapped in ── … ── render as a centred muted bar.
        s = line.strip()
        if s.startswith("──") and s.endswith("──") and len(s) > 6:
            body = s.strip("─").strip()
            html = ('<div style="color:{muted}; font-size:11px; font-weight:700;'
                    ' letter-spacing:1px; text-transform:uppercase; margin:6px 0;'
                    ' border-bottom:1px solid {border}; padding-bottom:3px;">'
                    '{body}</div>').format(muted=self._log_muted(), border=self._log_border(), body=_esc(body))
            self._append_html(html)
            return

        if line.startswith("✓") or line.startswith("✔"):
            color, sym, rest = SUCCESS, "[INF]", line[1:]
        elif line.startswith("✗"):
            color, sym, rest = DANGER, "[ERR]", line[1:]
        elif line.startswith("⚠"):
            color, sym, rest = WARN, "[WRN]", line[1:]
        elif line.lstrip().startswith("[INF]"):
            color, sym, rest = SUCCESS, "[INF]", line.lstrip()[5:]
        elif line.startswith(" "):
            # Indented sub-output (pip, git) — render dimmer.
            color, sym, rest = self._log_muted(), "", line
        else:
            color, sym, rest = self._log_text(), "", line

        ts_html = '<span style="color:#6B7280; font-size:11px;">{ts}</span> '.format(
            ts=ts)
        if sym:
            mark = '<span style="color:{c}; font-weight:700;">{s}</span> '.format(c=color, s=_esc(sym))
            body = '<span style="color:#1F2937;">{r}</span>'.format(r=_esc(rest))
        else:
            mark = ""
            mark = '<span style="color:#22C55E;font-weight:700;">[INF]</span> '
            body = '<span style="color:{c};">{r}</span>'.format(c=color, r=_esc(rest))
        self._append_html(ts_html + mark + body)

    def _append_html(self, html):
        _append_log_preserving_scroll(self.log_view, html)

    def _log_text(self):
        return DARK["text"] if self.theme == "dark" else LIGHT["text"]

    def _log_muted(self):
        return DARK["muted"] if self.theme == "dark" else LIGHT["muted"]

    def _log_dim(self):
        return "#5B6573" if self.theme == "dark" else "#9AA4B2"

    def _log_border(self):
        return DARK["border"] if self.theme == "dark" else LIGHT["border"]

    def _start_install(self):
        if self._worker is not None and self._worker.isRunning():
            return
        path = self.path_edit.text().strip() or os.path.join(
            os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "Programs", "UltraPilot")
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
        # Launch-on-finish now happens in _next() (the „Dokončiť“ click) so the
        # X button / Alt+F4 no longer silently spawns the app and a launch
        # failure can't crash the installer here.
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


def _do_uninstall_app(rec, log=None):
    """Remove the app folder, shortcuts and the install record."""
    install_path = rec.get("install_path", "")
    if install_path and os.path.isdir(install_path):
        if log:
            log("Odstraňujem priečinok " + install_path)
        shutil.rmtree(install_path, ignore_errors=True)
    for folder in (os.path.join(os.environ.get("USERPROFILE", ""), "Desktop"),
                   os.path.join(os.environ.get("APPDATA", ""),
                                "Microsoft\\Windows\\Start Menu\\Programs")):
        lnk = os.path.join(folder, "UltraPilot.lnk")
        try:
            if os.path.exists(lnk):
                if log:
                    log("Odstraňujem skratku " + lnk)
                os.remove(lnk)
        except Exception:
            pass
    try:
        if os.path.exists(RECORD_PATH):
            if log:
                log("Odstraňujem záznam inštalácie")
            os.remove(RECORD_PATH)
    except Exception:
        pass


def _do_uninstall_sdk(rec, log=None):
    """Remove the SDK DLLs from each recorded game's plugins folder."""
    plugin_dirs = _sdk_plugin_dirs(rec, include_detected=True)
    if not plugin_dirs:
        if log:
            log("Žiadna hra ani SDK priečinok nebol nájdený.")
        return {"removed": 0, "failed": 0, "found": 0}
    removed = 0
    failed = 0
    found = 0
    for plugins_dir in plugin_dirs:
        for name in _SDK_DLL_NAMES:
            p = os.path.join(plugins_dir, name)
            try:
                if os.path.exists(p):
                    found += 1
                    if log:
                        log("Odstraňujem " + name + " z " + plugins_dir)
                    os.remove(p)
                    removed += 1
            except Exception as e:
                failed += 1
                if log:
                    log("⚠ " + name + ": " + str(e))
    if log:
        if failed:
            log("⚠ SDK: odstránené {}, zlyhalo {} (hra môže používať DLL).".format(
                removed, failed))
        elif removed:
            log("✓ SDK pluginy odstránené: {} súborov.".format(removed))
        else:
            log("SDK pluginy už v nájdených priečinkoch nie sú.")
    return {"removed": removed, "failed": failed, "found": found}


def _do_uninstall_python(rec, log=None):
    """Silently uninstall the Python the installer downloaded, if any.

    Only fires when ``python_installed_by_installer`` is true in the record — we
    never touch a Python the user installed themselves."""
    if not rec.get("python_installed_by_installer"):
        if log:
            log("Python nebol nainštalovaný inštalátorom — preskakujem.")
        return
    tmp = os.path.join(os.environ.get("TEMP", os.path.expanduser("~")),
                       "UltraPilot_python_uninstaller.exe")
    try:
        import requests
        if log:
            log("Sťahujem odinštalátor Pythonu…")
        r = requests.get(PY_INSTALLER_URL, timeout=120, stream=True)
        if r.status_code != 200:
            if log:
                log("⚠ Nepodarilo sa stiahnuť odinštalátor (HTTP " + str(r.status_code) + ").")
            return
        with open(tmp, "wb") as f:
            for chunk in r.iter_content(65536):
                if chunk:
                    f.write(chunk)
        if log:
            log("Odinštalovávam Python (ticho)…")
        subprocess.run([tmp, "/uninstall", "/quiet"], timeout=600)
    except Exception as e:
        if log:
            log("⚠ Odinštalovanie Pythonu zlyhalo: " + str(e))
    finally:
        try:
            if os.path.exists(tmp):
                os.remove(tmp)
        except Exception:
            pass


class _MaintenanceDialog(QDialog):
    """Custom repair / uninstall / cancel picker (replaces the old QMessageBox).

    Drawn with the installer's own dark palette so the text is always legible —
    the previous QMessageBox picked up light system colours and became invisible
    against the dark QSS."""

    def __init__(self, rec, parent=None):
        super().__init__(parent)
        self.rec = rec
        self.action = "cancel"
        self.setWindowTitle(APP_NAME)
        self.setFixedSize(460, 280)
        self.setObjectName("Window")
        self.setStyleSheet(_qss("light"))
        if os.path.exists(ICON_PATH):
            self.setWindowIcon(QIcon(ICON_PATH))
        lay = QVBoxLayout(self)
        lay.setContentsMargins(28, 24, 28, 20)
        lay.setSpacing(10)
        title = QLabel("UltraPilot — údržba")
        title.setStyleSheet("font-size:22px;font-weight:800;color:#047857;")
        lay.addWidget(title)
        sub = QLabel("UltraPilot je už nainštalovaný.\nČo chceš spraviť?")
        sub.setStyleSheet("font-size:14px;color:#64748B;")
        sub.setWordWrap(True)
        lay.addWidget(sub)
        lay.addStretch()
        row = QHBoxLayout()
        row.setSpacing(10)
        for label, role, primary in (
            ("Opraviť", "repair", True),
            ("Odinštalovať", "uninstall", False),
            ("Zrušiť", "cancel", False),
        ):
            btn = _primary_btn(label) if primary else _ghost_btn(label)
            btn.clicked.connect(lambda _, r=role: self._choose(r))
            row.addWidget(btn)
        lay.addLayout(row)

    def _choose(self, role):
        self.action = role
        self.accept()


class _UninstallDialog(QDialog):
    """Pick what to remove, then run the uninstall with a live log + progress."""

    def __init__(self, rec, parent=None):
        super().__init__(parent)
        self.rec = rec
        self.setWindowTitle(APP_NAME + " — Odinštalovanie")
        self.setObjectName("Window")
        self.setStyleSheet(_qss("light"))
        self.resize(560, 480)
        if os.path.exists(ICON_PATH):
            self.setWindowIcon(QIcon(ICON_PATH))
        lay = QVBoxLayout(self)
        lay.setContentsMargins(24, 20, 24, 20)
        lay.setSpacing(10)
        title = QLabel("Odinštalovanie UltraPilot")
        title.setStyleSheet("font-size:20px;font-weight:800;color:#047857;")
        lay.addWidget(title)
        sub = QLabel("Vyber, čo chceš odinštalovať:")
        sub.setStyleSheet("font-size:13px;color:#64748B;")
        lay.addWidget(sub)

        # Shared checkbox style: bright text + a visible check indicator on the
        # dark palette (the default QCheckBox colours were nearly invisible).
        _chk_qss = ("QCheckBox{color:#0F172A; font-size:14px; spacing:10px;"
                    " padding:4px 0;} QCheckBox::indicator{width:18px; height:18px;"
                    " border:2px solid #CBD5E1; border-radius:4px; background:#FFFFFF;}"
                    "QCheckBox::indicator:checked{background:#2EA043; border-color:#2EA043;}"
                    "QCheckBox::indicator:hover{border-color:#2EA043;}")

        self.chk_app = QCheckBox("Aplikácia UltraPilot (priečinok, skratky, záznam)")
        self.chk_app.setChecked(True)
        self.chk_app.setStyleSheet(_chk_qss)
        lay.addWidget(self.chk_app)

        self.chk_sdk = QCheckBox("SDK pluginy (DLL z hry)")
        # Do not rely only on the install record: older versions did not always
        # save sdk_targets. Detect the actual DLLs in every installed game.
        sdk_plugin_dirs = _sdk_plugin_dirs(rec, include_detected=True)
        has_sdk = any(os.path.exists(os.path.join(folder, dll))
                      for folder in sdk_plugin_dirs for dll in _SDK_DLL_NAMES)
        if sdk_plugin_dirs:
            self.rec["sdk_targets"] = [_sdk_game_root(folder) for folder in sdk_plugin_dirs]
        self.chk_sdk.setChecked(has_sdk)
        self.chk_sdk.setEnabled(has_sdk)
        self.chk_sdk.setStyleSheet(_chk_qss)
        lay.addWidget(self.chk_sdk)

        # Python: always selectable. If the installer didn't install it we warn
        # in the label that it's the user's own Python — but we let them opt in
        # (the installer just won't run the silent uninstaller in that case,
        # see _do_uninstall_python).
        py_by_installer = bool(rec.get("python_installed_by_installer"))
        if py_by_installer:
            py_text = "Python (nainštalovaný inštalátorom)"
        else:
            py_text = ("Python  ·  pozor: inštalátor ho nenainštaloval "
                       "(pravdepodobne tvoj vlastný) — odinštaluje sa ticho len ak ho poznáme")
        self.chk_python = QCheckBox(py_text)
        self.chk_python.setChecked(py_by_installer)
        self.chk_python.setEnabled(True)
        self.chk_python.setStyleSheet(_chk_qss)
        lay.addWidget(self.chk_python)

        lay.addSpacing(6)
        self.progress = QProgressBar()
        self.progress.setValue(0)
        lay.addWidget(self.progress)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("QTextEdit{background:#0A0A0A;color:#E5E7EB;border:1px solid #D5DAE1;border-radius:8px;padding:8px;font-family:'Cascadia Mono','Consolas';}")
        self.log_view.document().setMaximumBlockCount(2000)
        self.log_view.setMinimumHeight(140)
        lay.addWidget(self.log_view, stretch=1)

        row = QHBoxLayout()
        row.addStretch()
        self.run_btn = _primary_btn("Odinštalovať")
        self.run_btn.clicked.connect(self._run)
        self.close_btn = _ghost_btn("Zavrieť")
        self.close_btn.clicked.connect(self.accept)
        row.addWidget(self.run_btn)
        row.addWidget(self.close_btn)
        lay.addLayout(row)
        self._worker = None

    def _log(self, msg):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        _append_log_preserving_scroll(self.log_view,
            '<span style="color:#5B6573; font-size:11px;">[' + ts + ']</span> '
            '<span style="color:#E6EDF3;">' + _esc(msg) + '</span>')

    def _run(self):
        if self._worker is not None and self._worker.isRunning():
            return
        self.run_btn.setEnabled(False)
        self.run_btn.setText("Odinštalujem…")
        self._worker = _UninstallWorker(
            self.rec,
            remove_app=self.chk_app.isChecked(),
            remove_sdk=self.chk_sdk.isChecked(),
            remove_python=self.chk_python.isChecked(),
        )
        self._worker.log.connect(self._log)
        self._worker.progress.connect(self.progress.setValue)
        self._worker.done.connect(self._on_done)
        self._worker.start()

    def _on_done(self):
        self.run_btn.setText("Hotovo")
        self._log("✔ Odinštalovanie dokončené.")


class _UninstallWorker(QThread):
    log = pyqtSignal(str)
    progress = pyqtSignal(int)
    done = pyqtSignal()

    def __init__(self, rec, remove_app, remove_sdk, remove_python):
        super().__init__()
        self.rec = rec
        self.remove_app = remove_app
        self.remove_sdk = remove_sdk
        self.remove_python = remove_python

    def run(self):
        steps = sum([self.remove_app, self.remove_sdk, self.remove_python])
        pct = 0
        try:
            if self.remove_app:
                self.log.emit("─── Aplikácia ───")
                _do_uninstall_app(self.rec, log=self.log.emit)
                pct += int(100 / max(1, steps))
                self.progress.emit(pct)
            if self.remove_sdk:
                self.log.emit("─── SDK pluginy ───")
                _do_uninstall_sdk(self.rec, log=self.log.emit)
                pct += int(100 / max(1, steps))
                self.progress.emit(pct)
            if self.remove_python:
                self.log.emit("─── Python ───")
                _do_uninstall_python(self.rec, log=self.log.emit)
                pct += 100 - pct
                self.progress.emit(pct)
            self.progress.emit(100)
        except Exception as e:
            self.log.emit("✗ Chyba: " + str(e))
        self.done.emit()


class _RepairDialog(QDialog):
    """Re-pull changed/missing files from GitHub, re-apply SDK + deps.

    Compares the recorded install against the live repository: anything missing
    or obviously stale is overwritten from the latest ``main`` branch, then the
    SDK DLLs are re-copied into the game and pip dependencies reinstalled."""

    def __init__(self, rec, parent=None):
        super().__init__(parent)
        self.rec = rec
        self.setWindowTitle(APP_NAME + " — Oprava")
        self.setObjectName("Window")
        self.setStyleSheet(_qss("light"))
        self.resize(560, 480)
        if os.path.exists(ICON_PATH):
            self.setWindowIcon(QIcon(ICON_PATH))
        lay = QVBoxLayout(self)
        lay.setContentsMargins(24, 20, 24, 20)
        lay.setSpacing(10)
        title = QLabel("Oprava UltraPilot")
        title.setStyleSheet("font-size:20px;font-weight:800;color:#047857;")
        lay.addWidget(title)
        sub = QLabel("Skontrolujem súbory oproti GitHubu, doplním chýbajúce\na znova nainštalujem SDK a Python kniňnice.")
        sub.setStyleSheet("font-size:13px;color:#64748B;")
        sub.setWordWrap(True)
        lay.addWidget(sub)
        lay.addSpacing(6)
        self.progress = QProgressBar()
        self.progress.setValue(0)
        lay.addWidget(self.progress)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet("QTextEdit{background:#0A0A0A;color:#E5E7EB;border:1px solid #D5DAE1;border-radius:8px;padding:8px;font-family:'Cascadia Mono','Consolas';}")
        self.log_view.document().setMaximumBlockCount(2000)
        self.log_view.setMinimumHeight(160)
        lay.addWidget(self.log_view, stretch=1)
        row = QHBoxLayout()
        row.addStretch()
        self.run_btn = _primary_btn("Spustiť opravu")
        self.run_btn.clicked.connect(self._run)
        self.close_btn = _ghost_btn("Zavrieť")
        self.close_btn.clicked.connect(self.accept)
        row.addWidget(self.run_btn)
        row.addWidget(self.close_btn)
        lay.addLayout(row)
        self._worker = None

    def _log(self, msg):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        _append_log_preserving_scroll(self.log_view,
            '<span style="color:#5B6573; font-size:11px;">[' + ts + ']</span> '
            '<span style="color:#E6EDF3;">' + _esc(msg) + '</span>')

    def _run(self):
        if self._worker is not None and self._worker.isRunning():
            return
        self.run_btn.setEnabled(False)
        self.run_btn.setText("Opravujem…")
        self._worker = _RepairWorker(self.rec)
        self._worker.log.connect(self._log)
        self._worker.progress.connect(self.progress.setValue)
        self._worker.done.connect(self._on_done)
        self._worker.start()

    def _on_done(self, ok):
        self.run_btn.setText("Hotovo" if ok else "Oprava zlyhala")
        self._log("✔ Oprava dokončená." if ok else
                  "✗ Oprava nebola dokončená; pozri presnú chybu vyššie.")


class _RepairWorker(QThread):
    log = pyqtSignal(str)
    progress = pyqtSignal(int)
    done = pyqtSignal(bool)

    def __init__(self, rec):
        super().__init__()
        self.rec = rec

    def run(self):
        install_path = self.rec.get("install_path", "")
        try:
            # 1) Re-fetch sources from GitHub into the install folder (overwrite).
            self.log.emit("─── Kontrola súborov ───")
            self.log.emit("Sťahujem aktuálne súbory z GitHubu…")
            self.progress.emit(15)
            worker = InstallWorker(install_path, "sk")
            worker.log = self.log
            worker.status = type("S", (), {"emit": staticmethod(lambda *a: None)})()
            worker.progress = type("S", (), {"emit": staticmethod(lambda *a: None)})()
            ok_repo = worker._fetch_repo()
            if ok_repo:
                self.log.emit("✓ Súbory synchronizované.")
            else:
                self.log.emit("✗ Zdrojové súbory sa nepodarilo stiahnuť; "
                              "presné chyby jednotlivých prenosov sú vyššie.")
                self.done.emit(False)
                return

            # 2) Re-install Python deps.
            self.progress.emit(45)
            self.log.emit("─── Python knižnice ───")
            if not worker._pip_install():
                self.log.emit("✗ Povinné Python knižnice sa nepodarilo nainštalovať.")
                self.done.emit(False)
                return
            self.log.emit("✓ Knižnice nastavené.")

            # 3) Re-apply SDK DLLs into the game.
            self.progress.emit(75)
            self.log.emit("─── SDK pluginy ───")
            try:
                from core.sdk.game_utils import install_game_dlls
                folders = install_game_dlls(os.path.join(install_path, "assets"))
                if folders:
                    for fld in folders:
                        self.log.emit("✓ SDK → " + fld)
                    roots = _update_sdk_targets(self.rec, folders)
                    self.log.emit("✓ SDK ciele uložené: {}".format(len(roots)))
                else:
                    self.log.emit("Hra zatiaľ nenájdená — DLL sa nainštalujú pri prvom spustení.")
            except Exception as e:
                self.log.emit("⚠ SDK: " + str(e))

            self.progress.emit(100)
            self.done.emit(True)
        except Exception as e:
            self.log.emit("✗ Chyba: " + str(e))
            self.done.emit(False)


def _maintenance_dialog(rec):
    """Show the maintenance picker; returns one of 'repair', 'uninstall', 'cancel'."""
    dlg = _MaintenanceDialog(rec)
    dlg.exec()
    return dlg.action


def main():
    app = QApplication(sys.argv)
    rec = _read_record()
    if rec is not None:
        action = _maintenance_dialog(rec)
        if action == "uninstall":
            ud = _UninstallDialog(rec)
            ud.exec()
            return
        elif action == "repair":
            rd = _RepairDialog(rec)
            rd.exec()
            return
        elif action == "cancel":
            return
    w = InstallerWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
