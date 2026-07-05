"""
Internationalisation for UltraPilot.

Translation tables live as JSON files in ``<app_dir>/languages`` — one file per
language (``en.json``, ``sk.json``, …).  English (``en``) is the reference; the
other files may be partial, in which case missing keys fall back to English.
``coverage(code)`` reports how complete a language is so the UI can show
„Slovenčina — 100% translated“.

Two language codes are always available because they ship inside the repo:
``sk`` (Slovenčina) and ``en`` (English).  Any other ``<code>.json`` placed in
``languages/`` (typically downloaded from GitHub by the onboarding wizard or the
settings page) becomes available too.  ``available()`` returns the full list,
marking which ones are downloaded.

The onboarding wizard and the installer share this same module so the language
choice is consistent across both.
"""

import json
import logging
import os

try:
    import requests
except Exception:  # pragma: no cover
    requests = None

from core.paths import app_dir

BUNDLED_CODES = ("sk", "en")          # always present, ship with the app
DEFAULT_LANG = "sk"

REPO = "matule123/ets2la"
RAW_BASE = "https://raw.githubusercontent.com/" + REPO + "/main/languages/"


# --------------------------------------------------------------------------- paths
def _bundled_dir():
    """Where bundled language files live (next to the executable / source)."""
    base = app_dir()
    # When frozen by PyInstaller the bundled files are under _MEIPASS; otherwise
    # they sit in <app_dir>/languages.  Check both, prefer _MEIPASS.
    meipass = getattr(__import__("sys"), "_MEIPASS", None)
    for cand in (os.path.join(meipass, "languages") if meipass else None,
                 os.path.join(base, "languages"),
                 os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "languages")):
        if cand and os.path.isdir(cand):
            return cand
    return os.path.join(base, "languages")


def languages_dir() -> str:
    """Where downloaded language files are stored (user-writable)."""
    d = os.path.join(app_dir(), "languages")
    os.makedirs(d, exist_ok=True)
    return d


def _path_for(code, prefer_bundled=True):
    """Find the JSON file for ``code``. Bundled copy first if ``prefer_bundled``."""
    code = (code or "").lower()
    fname = code + ".json"
    dirs = []
    if prefer_bundled:
        dirs.append(_bundled_dir())
    dirs.append(languages_dir())
    for d in dirs:
        p = os.path.join(d, fname)
        if os.path.exists(p):
            return p
    return ""


# --------------------------------------------------------------------------- loading
_cache = {}


def load(code: str) -> dict:
    """Load a language table (cached). Falls back to English, then empty."""
    code = (code or "").lower() or DEFAULT_LANG
    if code in _cache:
        return _cache[code]
    path = _path_for(code)
    if not path and code != "en":
        path = _path_for("en")
    if not path:
        return {}
    try:
        with open(path, encoding="utf-8") as f:
            tbl = json.load(f)
        _cache[code] = tbl
        return tbl
    except Exception as e:
        logging.error("i18n: failed to load %s: %s", path, e)
        return {}


def reload(code: str = None):
    """Drop the cache (optionually just for one language) so the next load re-reads disk."""
    if code:
        _cache.pop((code or "").lower(), None)
    else:
        _cache.clear()


# --------------------------------------------------------------------------- translate
def t(lang_code: str, namespace: str, key: str, **fmt) -> str:
    """Translate ``key`` under ``namespace`` for ``lang_code``.

    Falls back to English, then to the key itself.  Supports ``str.format``
    substitution via ``**fmt``.
    """
    lang = (lang_code or "").lower() or DEFAULT_LANG
    for code in (lang, "en"):
        val = load(code).get(namespace, {}).get(key)
        if val:
            try:
                return val.format(**fmt) if fmt else val
            except Exception:
                return val
    return key


def coverage(code: str) -> int:
    """Percent of English keys present in ``code`` across all namespaces."""
    en = load("en")
    if not en:
        return 100
    tbl = load(code)
    if code == "en" or not tbl:
        return 100 if code == "en" else 0
    total = 0
    have = 0
    for ns, kv in en.items():
        if ns.startswith("_"):
            continue
        if not isinstance(kv, dict):
            continue
        for k in kv:
            total += 1
            tns = tbl.get(ns)
            if isinstance(tns, dict) and k in tns:
                have += 1
    return round(100 * have / total) if total else 100


# --------------------------------------------------------------------------- listing
def _meta(code: str) -> dict:
    tbl = load(code)
    meta = tbl.get("_meta") if isinstance(tbl, dict) else None
    if isinstance(meta, dict):
        return meta
    # Fallback display names for known codes.
    names = {
        "sk": ("Slovenčina", "Slovak"), "en": ("English", "English"),
        "cs": ("Čeština", "Czech"), "de": ("Deutsch", "German"),
        "pl": ("Polski", "Polish"), "fr": ("Français", "French"),
        "es": ("Español", "Spanish"),
    }
    n = names.get(code, (code, code))
    return {"code": code, "name": n[0], "english_name": n[1]}


def _bundled_codes_available() -> list:
    """Codes that are physically bundled (in case sk/en ship loose)."""
    d = _bundled_dir()
    out = set(BUNDLED_CODES)
    try:
        for f in os.listdir(d):
            if f.endswith(".json") and f != "index.json":
                out.add(f[:-5].lower())
    except Exception:
        pass
    return sorted(out)


def available() -> list:
    """Return ``[{code, name, english_name, coverage, downloaded, bundled}]``.

    Bundled languages come first (sk, en), then any others present on disk,
    finally everything declared in ``index.json`` that is not yet downloaded.
    """
    bundled = _bundled_codes_available()
    downloaded = set()
    try:
        for f in os.listdir(languages_dir()):
            if f.endswith(".json") and f != "index.json":
                downloaded.add(f[:-5].lower())
    except Exception:
        pass

    seen = set()
    out = []

    def add(code, force_bundled=False, force_downloaded=False):
        code = (code or "").lower()
        if not code or code in seen:
            return
        seen.add(code)
        meta = _meta(code)
        is_bundled = force_bundled or code in bundled
        is_down = force_downloaded or code in downloaded or is_bundled
        out.append({
            "code": code,
            "name": meta.get("name", code),
            "english_name": meta.get("english_name", code),
            "coverage": _orig_coverage(code) if is_down else 0,
            "downloaded": is_down,
            "bundled": is_bundled,
        })

    # Bundled first (sk, en), then the rest of bundled, then downloaded, then index entries.
    for c in BUNDLED_CODES:
        add(c, force_bundled=True)
    for c in bundled:
        add(c, force_bundled=True)
    for c in sorted(downloaded):
        add(c, force_downloaded=True)
    for entry in _index_languages():
        add(entry.get("code"))
    return out


def _index_languages() -> list:
    """Read ``index.json`` (bundled or in the languages dir) for the catalog."""
    for d in (_bundled_dir(), languages_dir()):
        p = os.path.join(d, "index.json")
        if os.path.exists(p):
            try:
                with open(p, encoding="utf-8") as f:
                    return json.load(f).get("languages", [])
            except Exception:
                pass
    return []


# --------------------------------------------------------------------------- install/uninstall
def _github_headers():
    h = {}
    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if token:
        h["Authorization"] = "Bearer " + token
    return h


def install_from_github(code: str, progress_cb=None) -> bool:
    """Download ``languages/<code>.json`` from GitHub into the user languages dir."""
    code = (code or "").lower()
    if code in BUNDLED_CODES:
        return True  # nothing to do
    if requests is None:
        logging.error("i18n: requests not available — cannot download %s", code)
        return False
    try:
        url = RAW_BASE + code + ".json"
        if progress_cb:
            progress_cb(0.1, code)
        r = requests.get(url, headers=_github_headers(), timeout=30)
        if r.status_code != 200:
            logging.error("i18n: download %s failed (HTTP %s)", code, r.status_code)
            return False
        data = r.json()
        if not isinstance(data, dict):
            return False
        os.makedirs(languages_dir(), exist_ok=True)
        dest = os.path.join(languages_dir(), code + ".json")
        with open(dest, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        reload(code)
        if progress_cb:
            progress_cb(1.0, code)
        return True
    except Exception as e:
        logging.error("i18n: install %s failed: %s", code, e)
        return False


def uninstall(code: str) -> bool:
    """Remove a downloaded language. Bundled languages cannot be removed."""
    code = (code or "").lower()
    if code in BUNDLED_CODES:
        return False
    try:
        p = os.path.join(languages_dir(), code + ".json")
        if os.path.exists(p):
            os.remove(p)
        reload(code)
        return True
    except Exception as e:
        logging.error("i18n: uninstall %s failed: %s", code, e)
        return False


# --------------------------------------------------------------------------- backward-compat
# The legacy API (used by ui/settings_menu.py) exposed a ``LANGUAGES`` dict
# keyed by *display name* and a ``coverage(lang_name)`` that took that display
# name.  Keep a thin shim so existing callers keep working while we migrate.

def _legacy_name_to_code():
    out = {}
    for lang in available():
        out[lang["name"]] = lang["code"]
    return out


# A lazy proxy that behaves like the old ``LANGUAGES`` dict (name -> table).
class _LanguagesDict(dict):
    def __missing__(self, key):
        return {}

    def __iter__(self):
        return iter(_legacy_name_to_code())

    def keys(self):
        return _legacy_name_to_code().keys()

    def values(self):
        return [load(c) for c in _legacy_name_to_code().values()]

    def items(self):
        nm = _legacy_name_to_code()
        return [(name, load(code)) for name, code in nm.items()]

    def __contains__(self, key):
        return key in _legacy_name_to_code()

    def __getitem__(self, key):
        code = _legacy_name_to_code().get(key)
        return load(code) if code else {}


LANGUAGES = _LanguagesDict()


def legacy_coverage(lang_name: str) -> int:
    """Old signature: takes a display name (e.g. „Slovenčina“), returns %."""
    code = _legacy_name_to_code().get(lang_name)
    return coverage(code) if code else 0


# Module-level ``coverage`` kept compatible: accept either a code or a display name.
_orig_coverage = coverage


def coverage(lang):  # noqa: F811 — intentionally overriding the strict version
    if lang and lang in _legacy_name_to_code():
        return _orig_coverage(_legacy_name_to_code()[lang])
    return _orig_coverage(lang)


# Old-style ``t(lang_name, key)`` for callers that still use the flat app namespace.
def legacy_t(lang_name: str, key: str) -> str:
    code = _legacy_name_to_code().get(lang_name, DEFAULT_LANG)
    return t(code, "app", key)
