"""
SDK plugin installation for UltraPilot.

The SCS SDK ships as two DLLs that go into the game's ``bin/win_x64/plugins/``
folder so the game exposes telemetry and accepts control input:

* ``scs-telemetry.dll``    — telemetry out of the game (truck state, position…).
* ``scs_sdk_controller.dll`` — control input back into the game (steering…).

The ETS2LA project keeps versioned copies of these DLLs in its repository, but
that repo is currently private, so we can't fetch new builds from it at runtime.
UltraPilot therefore ships DLLs for the most common game version (1.59) inside
``assets/`` and uses them directly.  Other game versions fall back to a clear
„install manually“ message.  The data structure below is ready to point at
remote URLs once a public source exists.
"""

import logging
import os
import shutil

try:
    import requests
except Exception:  # pragma: no cover
    requests = None

from core.paths import app_dir, resource

# The two SDK DLLs every supported game version needs.
SDK_FILES = ("scs-telemetry.dll", "scs_sdk_controller.dll")

# Per-version sources.  ``bundled`` means the DLL is shipped in ``assets/`` and
# is used directly (no network).  ``urls`` (optional) would let us download a
# build for a version we don't bundle — left as a hook for the future.
SOURCES = {
    "1.59": {
        "bundled": True,        # present in assets/scs-telemetry.dll etc.
        "urls": {},             # could be filled later with release URLs
    },
    # 1.58/1.57 ship the same SDK ABI in practice, so we reuse 1.59's DLLs.
    "1.58": {"bundled": True, "alias": "1.59"},
    "1.57": {"bundled": True, "alias": "1.59"},
}

DEFAULT_ALIAS = "1.59"


def supported_versions() -> list:
    """Game versions we have a SDK source for (sorted, newest first)."""
    return sorted(SOURCES.keys(), reverse=True)


def _resolve(version: str) -> dict:
    """Follow ``alias`` chains to a concrete source entry."""
    version = (version or "").strip()
    seen = set()
    cur = version
    while cur and cur in SOURCES and cur not in seen:
        seen.add(cur)
        entry = SOURCES[cur]
        nxt = entry.get("alias")
        if nxt and nxt != cur:
            cur = nxt
        else:
            return entry
    # Unknown version: best-effort fallback to the default alias.
    return SOURCES.get(DEFAULT_ALIAS, {"bundled": True})


def is_supported(version: str) -> bool:
    return bool(version and version in SOURCES)


def _dll_source_dir() -> str:
    """Where bundled DLLs live (assets/ next to the app)."""
    try:
        return resource("assets")
    except Exception:
        return os.path.join(app_dir(), "assets")


def _ensure_dlls_local(version: str, dest_dir: str, log=None) -> bool:
    """Get the SDK DLLs for ``version`` into ``dest_dir`` (no game install).

    Returns True if all required DLLs are present in ``dest_dir`` afterwards.
    """
    entry = _resolve(version)
    os.makedirs(dest_dir, exist_ok=True)

    if entry.get("bundled"):
        src_dir = _dll_source_dir()
        ok = True
        for name in SDK_FILES:
            s = os.path.join(src_dir, name)
            d = os.path.join(dest_dir, name)
            if os.path.exists(d):
                continue
            if os.path.exists(s):
                try:
                    shutil.copy2(s, d)
                except Exception as e:
                    if log:
                        log("  " + name + ": " + str(e))
                    ok = False
            else:
                if log:
                    log("  chýba " + name + " v " + src_dir)
                ok = False
        return ok

    urls = entry.get("urls") or {}
    if not requests or not urls:
        return False
    ok = True
    for name in SDK_FILES:
        d = os.path.join(dest_dir, name)
        if os.path.exists(d):
            continue
        url = urls.get(name)
        if not url:
            ok = False
            continue
        try:
            r = requests.get(url, timeout=120, stream=True)
            if r.status_code != 200:
                ok = False
                continue
            with open(d, "wb") as f:
                for chunk in r.iter_content(65536):
                    if chunk:
                        f.write(chunk)
        except Exception as e:
            if log:
                log("  " + name + ": " + str(e))
            ok = False
    return ok


def is_sdk_installed(game_path: str) -> bool:
    """True if all required DLLs are already in the game's plugins folder."""
    plugins_dir = os.path.join(game_path, "bin", "win_x64", "plugins")
    return all(os.path.exists(os.path.join(plugins_dir, n)) for n in SDK_FILES)


def ensure_installed(game_path: str, version: str, log=None, progress_cb=None) -> tuple:
    """Make sure the SDK DLLs are installed into ``game_path`` for ``version``.

    Returns ``(ok: bool, message: str)`` where ``message`` is a short status:
    ``"installed"``, ``"already"``, ``"unsupported:<version>"`` or
    ``"failed:<detail>"``.
    """
    if not game_path or not os.path.isdir(game_path):
        return False, "failed:no_game_path"

    # Already there? Nothing to do.
    if is_sdk_installed(game_path):
        return True, "already"

    if not is_supported(version):
        return False, "unsupported:" + str(version)

    # Stage DLLs into the game's plugins folder.
    plugins_dir = os.path.join(game_path, "bin", "win_x64", "plugins")
    try:
        os.makedirs(plugins_dir, exist_ok=True)
    except Exception as e:
        logging.error("sdk_downloader: cannot create %s: %s", plugins_dir, e)
        return False, "failed:" + str(e)

    if progress_cb:
        progress_cb(0.2)
    if not _ensure_dlls_local(version, plugins_dir, log=log):
        return False, "failed:dll_missing"
    if progress_cb:
        progress_cb(1.0)

    # Remove the legacy single-file plugin if present (ETS2LA used to ship it).
    legacy = os.path.join(plugins_dir, "ets2la_plugin.dll")
    if os.path.exists(legacy):
        try:
            os.remove(legacy)
        except Exception:
            pass

    return is_sdk_installed(game_path), "installed" if is_sdk_installed(game_path) else "failed:verify"
