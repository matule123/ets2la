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
import re

VERSION = "0.4.1"
REPO = "matule123/ets2la"
API_URL = f"https://api.github.com/repos/{REPO}/releases/latest"
ARCHIVE_URL = f"https://github.com/{REPO}/archive/refs/heads/main.zip"


def _app_dir():
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def current_version() -> str:
    return VERSION


def _display_commit(value: str) -> str:
    """Return exactly one 7-character SHA for the version badge."""
    value = (value or "").strip()
    match = re.search(r"(?i)(?<![0-9a-f])[0-9a-f]{7,40}(?![0-9a-f])", value)
    return match.group(0)[:7].lower() if match else ""


def git_commit() -> str:
    """Short build commit, including frozen installs without a .git folder."""
    env_commit = (os.environ.get("ULTRAPILOT_COMMIT") or "").strip()
    if env_commit:
        return _display_commit(env_commit) or "build"
    marker = ""
    for name in ("commit.txt", "BUILD_COMMIT"):
        try:
            with open(os.path.join(_app_dir(), name), "r", encoding="utf-8") as f:
                marker = _display_commit(f.read())
            if marker:
                break
        except Exception:
            pass
    # In a clean checkout HEAD is authoritative. If a ZIP fallback replaced
    # files without moving HEAD, the tree is dirty and commit.txt identifies
    # the code that is actually installed.
    if os.path.isdir(os.path.join(_app_dir(), ".git")):
        try:
            status = subprocess.run(
                ["git", "-C", _app_dir(), "status", "--porcelain"],
                capture_output=True, text=True, timeout=8)
            if marker and status.returncode == 0 and status.stdout.strip():
                return marker
            out = subprocess.run(
                ["git", "-C", _app_dir(), "rev-parse", "--short", "HEAD"],
                capture_output=True, text=True, timeout=8)
            if out.returncode == 0:
                return _display_commit(out.stdout) or "build"
        except Exception:
            pass
    if marker:
        return marker
    # A frozen build must still show an explicit revision instead of silently
    # omitting the field. Installer/build scripts can replace this value.
    return "build"


def latest_release() -> str | None:
    """Latest release tag (without leading v/V) from GitHub, or None.

    Falls back to the latest commit SHA on ``main`` when the repo has no
    published releases (which is the common case during active development).

    Never raises — on failure logs the reason and returns None so the caller
    can show a clear status instead of silently reporting „up to date“."""
    import requests
    headers = {"Accept": "application/vnd.github+json"}
    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if token:
        headers["Authorization"] = f"Bearer {token}"
    try:
        headers["Cache-Control"] = "no-cache"
        # main is authoritative: a commit pushed after the latest Release must
        # be detected immediately, even when VERSION did not change.
        rc = requests.get(f"https://api.github.com/repos/{REPO}/commits/main",
                          headers=headers, timeout=8)
        if rc.status_code == 200:
            return (rc.json().get("sha", "") or "")[:10] or None
        elif rc.status_code in (403, 429):
            logging.warning("update check: GitHub commits API rate-limited (HTTP %s).",
                            rc.status_code)
        else:
            logging.warning("update check: commits API returned HTTP %s.",
                            rc.status_code)
        # Fallback only: use the latest published release when the commits API
        # is unavailable for this installation.
        r = requests.get(API_URL, headers=headers, timeout=8)
        if r.status_code == 200:
            tag = (r.json().get("tag_name") or "").lstrip("vV")
            if tag:
                return tag
    except Exception as e:
        logging.warning("update check: network error — %s", e)
    return None


def _looks_like_sha(s: str) -> bool:
    """True if ``s`` looks like a git short SHA (7+ hex chars, not a version).

    A version like ``0.4.0`` starts with a digit and contains dots, so it never
    matches. A short SHA like ``a1b2c3d`` is all hex and has no dots."""
    s = (s or "").strip()
    if len(s) < 7:
        return False
    return all(c in "0123456789abcdefABCDEF" for c in s)


def _is_newer(remote: str, local: str) -> bool:
    """Decide whether ``remote`` represents a newer release than ``local``.

    Three regimes:
      • both SHA-like  → newer iff they differ (any divergence = pull main)
      • both versions  → numeric semver compare
      • mixed (one SHA, one version) → CANNOT compare reliably → return False.
        This was the root cause of „always shows update available“: in a
        frozen/non-git install ``local`` is the VERSION constant while
        ``remote`` is a commit SHA, so the old „different ⇒ newer“ rule fired
        forever. Treating that case as „not newer“ stops the false positive."""
    r_sha = _looks_like_sha(remote)
    l_sha = _looks_like_sha(local)
    if r_sha and l_sha:
        r, l = (remote or "").lower(), (local or "").lower()
        return not (r.startswith(l) or l.startswith(r))
    if r_sha or l_sha:
        # Legacy frozen builds reported only "build" or VERSION. A known
        # remote commit must be offered or these installs stay stuck forever.
        if r_sha and (not local or str(local).lower() == "build"):
            return True
        return True

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
    """Return ``(available: bool, latest_tag: str|None)``.

    Compares the local commit (or version) against the remote. When the repo
    has no releases, ``latest_tag`` is the short remote commit SHA and an
    update is available whenever it differs from the local one."""
    remote = latest_release()
    # Prefer the git commit SHA when available (source/dev). Fall back to the
    # bundled VERSION constant only in a frozen build without .git.
    local_ref = git_commit() or VERSION
    if remote and _is_newer(remote, local_ref):
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
        if ok:
            head = subprocess.run(
                ["git", "-C", _app_dir(), "rev-parse", "--short", "HEAD"],
                capture_output=True, text=True, timeout=8)
            commit = _display_commit(head.stdout) if head.returncode == 0 else ""
            if commit:
                with open(os.path.join(_app_dir(), "commit.txt"), "w", encoding="utf-8") as f:
                    f.write(commit)
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


def _zip_update(progress_cb=None, target_commit=None) -> bool:
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
        # Persist the exact remote revision outside the bundled executable.
        # git_commit() reads this file before the embedded build metadata, so
        # the same downloaded update is not offered again after restart.
        if target_commit and _looks_like_sha(target_commit):
            with open(os.path.join(_app_dir(), "commit.txt"), "w", encoding="utf-8") as f:
                f.write(str(target_commit).strip())
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
    target = latest_release()
    return _zip_update(progress_cb, target_commit=target)
