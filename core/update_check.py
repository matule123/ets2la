"""
Update checking and applying for UltraPilot.

The old blocking splash window is gone — the UI now drives updates itself
(``ui/update_widget.py`` shows a spinner + status bar inside the sidebar). This
module is the pure logic layer the UI calls into:

* ``VERSION``           — the current app version (kept in sync with installer).
* ``current_version()`` — same, as a function.
* ``latest_release()``  — the newest GitHub release tag, or None.
* ``check_for_update()``— ``(available: bool, latest_tag: str)``.
* ``perform_update(progress_cb)``— hybrid update: ``git pull`` if the install is a
  git checkout, otherwise download the latest release zip and overwrite files
  (settings.json / routes / map-cache are preserved).
* ``git_commit()``      — short commit hash for the about/update UI.

All network calls are bounded with timeouts and never raise — on failure they
return a benign result (``False`` / ``""`` / ``None``) so the caller can show a
clear status instead of crashing.
"""

import logging
import os
import subprocess
import sys

VERSION = "0.4.0"
REPO = "matule123/ets2la"
API_URL = f"https://api.github.com/repos/{REPO}/releases/latest"
ARCHIVE_URL = f"https://github.com/{REPO}/archive/refs/heads/main.zip"


def _app_dir():
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def current_version() -> str:
    return VERSION


def git_commit() -> str:
    """Short HEAD commit hash, or '' if not a git checkout / git missing."""
    try:
        out = subprocess.run(
            ["git", "-C", _app_dir(), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=8)
        if out.returncode == 0:
            return out.stdout.strip()
    except Exception:
        pass
    return ""


def latest_release() -> str | None:
    """Latest release tag (without leading v/V) from GitHub, or None."""
    try:
        import requests
        r = requests.get(API_URL, timeout=6)
        if r.status_code == 200:
            return (r.json().get("tag_name") or "").lstrip("vV") or None
    except Exception:
        pass
    return None


def _is_newer(remote: str, local: str) -> bool:
    def parts(v):
        out = []
        for p in str(v).split("."):
            try:
                out.append(int("".join(c for c in p if c.isdigit()) or 0))
            except Exception:
                out.append(0)
        return out
    try:
        return parts(remote) > parts(local)
    except Exception:
        return False


def check_for_update() -> tuple:
    """Return ``(available: bool, latest_tag: str|None)``."""
    remote = latest_release()
    if remote and _is_newer(remote, VERSION):
        return True, remote
    return False, remote


def _git_pull(progress_cb=None) -> bool:
    """Fast-forward pull when the install is a git checkout. Returns success."""
    if not os.path.isdir(os.path.join(_app_dir(), ".git")):
        return False
    try:
        if progress_cb:
            progress_cb(0.2, "git pull…")
        r = subprocess.run(["git", "-C", _app_dir(), "pull", "--ff-only"],
                           capture_output=True, text=True, timeout=120)
        ok = r.returncode == 0
        if progress_cb:
            progress_cb(1.0, "git pull " + ("OK" if ok else "failed"))
        return ok
    except Exception as e:
        logging.warning("update git pull failed: %s", e)
        if progress_cb:
            progress_cb(1.0, "git pull error")
        return False


# Files/folders that must never be overwritten by an update (user data + caches).
_PROTECTED = {
    "settings.json", "routes", "map-cache", "model-cache", "logs",
    "UltraPilot_Installer.exe", "install.json",
}


def _zip_update(progress_cb=None) -> bool:
    """Fallback: download the main-branch zip and overwrite non-protected files."""
    try:
        import requests, zipfile, io, shutil
        if progress_cb:
            progress_cb(0.1, "Sťahujem balík aktualizácie…")
        r = requests.get(ARCHIVE_URL, timeout=180, stream=True)
        if r.status_code != 200:
            if progress_cb:
                progress_cb(1.0, "download HTTP " + str(r.status_code))
            return False
        data = r.content
        if progress_cb:
            progress_cb(0.5, "Rozbaľujem…")
        zf = zipfile.ZipFile(io.BytesIO(data))
        names = zf.namelist()
        # GitHub zips nest under "<repo>-main/".
        prefix = names[0].split("/")[0] if names else ""
        replaced = 0
        for n in names:
            if n.endswith("/"):
                continue
            rel = n[len(prefix) + 1:] if prefix and n.startswith(prefix + "/") else n
            if not rel or rel in _PROTECTED or rel.split(os.sep)[0] in _PROTECTED:
                continue
            dest = os.path.join(_app_dir(), rel)
            os.makedirs(os.path.dirname(dest) or _app_dir(), exist_ok=True)
            with open(dest, "wb") as f:
                f.write(zf.read(n))
            replaced += 1
        if progress_cb:
            progress_cb(1.0, f"Aktualizované ({replaced} súborov)")
        return True
    except Exception as e:
        logging.warning("update zip failed: %s", e)
        if progress_cb:
            progress_cb(1.0, "chyba: " + str(e))
        return False


def perform_update(progress_cb=None) -> bool:
    """Apply the latest code: git pull first, zip fallback otherwise."""
    if _git_pull(progress_cb):
        return True
    return _zip_update(progress_cb)
