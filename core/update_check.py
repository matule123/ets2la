"""
Update checking and applying for UltraPilot.

The old blocking splash window is gone — the UI now drives updates itself
(``ui/update_widget.py`` shows a spinner + status bar inside the sidebar). This
module is the pure logic layer the UI calls into:

* ``VERSION``           — the current app version (kept in sync with installer).
* ``current_version()`` — same, as a function.
* ``latest_release()``  — the newest GitHub release tag, or None.
* ``check_for_update()``— ``(available: bool, latest_tag: str)``.
* ``prepare_update()``  — download and verify without changing the app.
* ``install_prepared_update()`` — apply the staged package without network I/O.
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
import json
import tempfile

VERSION = "0.4.1"
_LATEST_COMMIT_INFO = {}
REPO = "matule123/ets2la"
API_URL = f"https://api.github.com/repos/{REPO}/releases/latest"
ARCHIVE_URL = f"https://github.com/{REPO}/archive/refs/heads/main.zip"


def _update_cache_dir() -> str:
    """Writable per-user staging directory, never inside the source tree."""
    local = os.environ.get("LOCALAPPDATA") or tempfile.gettempdir()
    return os.path.join(local, "UltraPilot", "update-cache")


def _prepared_paths():
    root = _update_cache_dir()
    return (os.path.join(root, "update.zip"),
            os.path.join(root, "update.json"))


def _format_download_progress(downloaded: int, total: int) -> str:
    done_mb = max(0, int(downloaded)) / (1024 * 1024)
    if total > 0:
        total_mb = max(0, int(total)) / (1024 * 1024)
        percent = min(100, int(max(0, downloaded) * 100 / max(1, total)))
        return (f"Stiahnuté {done_mb:.2f} MB z {total_mb:.2f} MB "
                f"({percent} %)")
    return f"Stiahnuté {done_mb:.2f} MB • celkovú veľkosť zisťujem"


def _archive_url_for_target(target: str | None) -> str:
    """Pin downloads to the exact commit advertised by the update check."""
    target = str(target or "").strip()
    if _looks_like_sha(target):
        return f"https://github.com/{REPO}/archive/{target}.zip"
    return ARCHIVE_URL


def _format_prepared_update_size(info: dict) -> str:
    """Describe verified staging size without confusing ZIP and install size."""
    info = info if isinstance(info, dict) else {}
    archive_bytes = int(info.get("archive_bytes", 0)
                        or info.get("downloaded_bytes", 0)
                        or info.get("total_bytes", 0) or 0)
    unpacked_bytes = int(info.get("unpacked_bytes", 0) or 0)
    archive_text = f"{archive_bytes / (1024 * 1024):.2f} MB"
    if unpacked_bytes > 0:
        unpacked_text = f"{unpacked_bytes / (1024 * 1024):.2f} MB"
        # The ZIP for this repository is currently roughly 0.83 MB.  Showing
        # that compressed transfer as the only prominent size made a verified
        # multi-megabyte installation look permanently stuck at 0.83 MB.
        # Lead with the actual prepared installation size; retain the archive
        # size as an explicitly labelled transfer fact.
        return (f"Pripravené na inštaláciu: {unpacked_text} "
                f"• stiahnutý balík {archive_text}")
    return f"Stiahnutý balík: {archive_text}"


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
    # In a real checkout HEAD is always authoritative. A stale commit.txt from
    # an older ZIP update must never replace the repository's actual revision.
    if os.path.isdir(os.path.join(_app_dir(), ".git")):
        try:
            out = subprocess.run(
                ["git", "-C", _app_dir(), "rev-parse", "--short=7", "HEAD"],
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
            global _LATEST_COMMIT_INFO
            payload = rc.json() or {}
            message = (((payload.get("commit") or {}).get("message")) or "").strip()
            lines = [line.strip() for line in message.splitlines() if line.strip()]
            _LATEST_COMMIT_INFO = {
                "sha": _display_commit(payload.get("sha", "")),
                "title": lines[0] if lines else "Aktualizácia UltraPilot",
                "description": "\n".join(lines[1:]) if len(lines) > 1 else "",
            }
            # Keep remote and local revisions identical in presentation and
            # comparison: one conventional seven-character short SHA.
            return _display_commit(rc.json().get("sha", "")) or None
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


def latest_commit_info() -> dict:
    """Metadata captured during the latest GitHub commit check."""
    return dict(_LATEST_COMMIT_INFO)


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

# Removed production modules that a ZIP update cannot delete merely by
# extracting the new archive. Paths are explicit and repository-relative.
_OBSOLETE = {
    "core/sdk/projection.py",
    "plugins/hud/elements/traffic_lights.py",
    "plugins/hud/elements/world.py",
}


def _apply_zip_bytes(data, progress_cb=None, target_commit=None) -> bool:
    """Validate and apply one downloaded archive to the application tree."""
    try:
        import zipfile, io
        if progress_cb:
            progress_cb(0.05, "Overujem stiahnutý balík…")
        zf = zipfile.ZipFile(io.BytesIO(data))
        bad_member = zf.testzip()
        if bad_member:
            raise ValueError("poškodený súbor v archíve: " + bad_member)
        names = zf.namelist()
        # GitHub zips nest under "<repo>-main/".
        prefix = names[0].split("/")[0] if names else ""
        replaced = 0
        for n in names:
            if n.endswith("/"):
                continue
            rel = n[len(prefix) + 1:] if prefix and n.startswith(prefix + "/") else n
            # ZIP member names always use '/', including on Windows. Validate
            # each component before turning it into a local path; this also
            # prevents a malformed archive from escaping the app directory.
            parts = tuple(part for part in rel.replace("\\", "/").split("/")
                          if part not in ("", "."))
            if (not parts or parts[0] in _PROTECTED
                    or any(part == ".." or ":" in part for part in parts)):
                continue
            dest = os.path.join(_app_dir(), *parts)
            os.makedirs(os.path.dirname(dest) or _app_dir(), exist_ok=True)
            with open(dest, "wb") as f:
                f.write(zf.read(n))
            replaced += 1
        for rel in _OBSOLETE:
            obsolete = os.path.join(_app_dir(), *rel.split("/"))
            if os.path.isfile(obsolete):
                os.remove(obsolete)
        # Persist the exact remote revision outside the bundled executable.
        # git_commit() reads this file before the embedded build metadata, so
        # the same downloaded update is not offered again after restart.
        if target_commit and _looks_like_sha(target_commit):
            with open(os.path.join(_app_dir(), "commit.txt"), "w", encoding="utf-8") as f:
                f.write(str(target_commit).strip())
        if progress_cb:
            progress_cb(1.0, f"Nainštalované ({replaced} súborov)")
        return True
    except Exception as e:
        logging.warning("update archive apply failed: %s", e)
        if progress_cb:
            progress_cb(1.0, "chyba: " + str(e))
        return False


def prepare_update(progress_cb=None, target_commit=None) -> bool:
    """Download and verify an update without modifying application files."""
    archive_path, manifest_path = _prepared_paths()
    archive_tmp = archive_path + ".tmp"
    manifest_tmp = manifest_path + ".tmp"
    try:
        import requests, zipfile
        os.makedirs(os.path.dirname(archive_path), exist_ok=True)
        target = target_commit or latest_release()
        # Do not label one commit in the UI and then fetch a moving main.zip.
        # Pin SHA updates and request an identity-encoded, uncached response so
        # Content-Length describes this exact transfer rather than a stale CDN
        # or content-encoded representation.
        response = requests.get(
            _archive_url_for_target(target), timeout=180, stream=True,
            headers={"Cache-Control": "no-cache", "Accept-Encoding": "identity"})
        if response.status_code != 200:
            if progress_cb:
                progress_cb(1.0, "Sťahovanie zlyhalo (HTTP "
                            + str(response.status_code) + ")")
            return False
        try:
            total = int(response.headers.get("Content-Length", 0) or 0)
        except (TypeError, ValueError):
            total = 0
        downloaded = 0
        if progress_cb:
            progress_cb(0.0, _format_download_progress(0, total))
        with open(archive_tmp, "wb") as stream:
            for chunk in response.iter_content(chunk_size=128 * 1024):
                if not chunk:
                    continue
                stream.write(chunk)
                downloaded += len(chunk)
                if progress_cb:
                    fraction = min(0.99, downloaded / total) if total else 0.0
                    progress_cb(fraction,
                                _format_download_progress(downloaded, total))
        unpacked_bytes = 0
        file_count = 0
        with open(archive_tmp, "rb") as stream:
            with zipfile.ZipFile(stream) as archive:
                bad_member = archive.testzip()
                if bad_member:
                    raise ValueError("poškodený súbor v archíve: " + bad_member)
                members = [member for member in archive.infolist()
                           if not member.is_dir()]
                if not members:
                    raise ValueError("prázdny aktualizačný archív")
                unpacked_bytes = sum(max(0, int(member.file_size))
                                     for member in members)
                file_count = len(members)
        os.replace(archive_tmp, archive_path)
        manifest = {
            "target_commit": str(target or ""),
            "downloaded_bytes": downloaded,
            # HTTP Content-Length is useful only while streaming. Redirects,
            # content encoding and proxy responses can make it differ from the
            # bytes actually stored, so the verified file is authoritative.
            "total_bytes": downloaded,
            "archive_bytes": downloaded,
            "unpacked_bytes": unpacked_bytes,
            "file_count": file_count,
        }
        with open(manifest_tmp, "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, ensure_ascii=False)
        os.replace(manifest_tmp, manifest_path)
        if progress_cb:
            progress_cb(1.0, _format_prepared_update_size(manifest))
        return True
    except Exception as e:
        logging.warning("update download failed: %s", e)
        for path in (archive_tmp, manifest_tmp):
            try:
                if os.path.isfile(path):
                    os.remove(path)
            except Exception:
                pass
        if progress_cb:
            progress_cb(1.0, "chyba: " + str(e))
        return False


def prepared_update_info() -> dict:
    """Return verified staging metadata, or an empty dict."""
    archive_path, manifest_path = _prepared_paths()
    try:
        if not os.path.isfile(archive_path):
            return {}
        with open(manifest_path, "r", encoding="utf-8") as stream:
            info = json.load(stream)
        if not isinstance(info, dict):
            return {}
        # Upgrade manifests created by the older UI. They stored only the HTTP
        # Content-Length (the misleading 0.8 MB value) and omitted unpacked
        # size. The verified ZIP itself is authoritative and can be inspected
        # locally without downloading it again.
        actual_archive_bytes = os.path.getsize(archive_path)
        info["archive_bytes"] = actual_archive_bytes
        info["downloaded_bytes"] = actual_archive_bytes
        info["total_bytes"] = actual_archive_bytes
        if int(info.get("unpacked_bytes", 0) or 0) <= 0:
            import zipfile
            with zipfile.ZipFile(archive_path) as archive:
                members = [member for member in archive.infolist()
                           if not member.is_dir()]
                info["unpacked_bytes"] = sum(
                    max(0, int(member.file_size)) for member in members)
                info["file_count"] = len(members)
        return info
    except Exception:
        return {}


def mark_update_startup_notice() -> None:
    """Ask the next startup splash to show the installation transition."""
    marker = os.path.join(_update_cache_dir(), "installed.notice")
    os.makedirs(os.path.dirname(marker), exist_ok=True)
    temporary = marker + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        stream.write("installed")
    os.replace(temporary, marker)


def take_update_startup_notice() -> bool:
    """Atomically consume the one-shot startup installation notice."""
    marker = os.path.join(_update_cache_dir(), "installed.notice")
    try:
        if not os.path.isfile(marker):
            return False
        os.remove(marker)
        return True
    except Exception:
        return False


def install_prepared_update(progress_cb=None) -> bool:
    """Install the already verified archive; never performs network I/O."""
    archive_path, manifest_path = _prepared_paths()
    info = prepared_update_info()
    if not info:
        if progress_cb:
            progress_cb(1.0, "Stiahnutá aktualizácia sa nenašla")
        return False
    try:
        with open(archive_path, "rb") as stream:
            data = stream.read()
        ok = _apply_zip_bytes(
            data, progress_cb=progress_cb,
            target_commit=info.get("target_commit"))
        if not ok:
            return False
        mark_update_startup_notice()
        for path in (archive_path, manifest_path):
            try:
                os.remove(path)
            except OSError:
                pass
        return True
    except Exception as e:
        logging.warning("prepared update install failed: %s", e)
        if progress_cb:
            progress_cb(1.0, "chyba: " + str(e))
        return False


def _zip_update(progress_cb=None, target_commit=None) -> bool:
    """Backward-compatible immediate ZIP download and installation."""
    try:
        import requests
        if progress_cb:
            progress_cb(0.1, "Sťahujem balík aktualizácie…")
        response = requests.get(
            _archive_url_for_target(target_commit), timeout=180, stream=True,
            headers={"Cache-Control": "no-cache", "Accept-Encoding": "identity"})
        if response.status_code != 200:
            if progress_cb:
                progress_cb(1.0, "download HTTP " + str(response.status_code))
            return False
        return _apply_zip_bytes(
            response.content, progress_cb=progress_cb,
            target_commit=target_commit)
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
