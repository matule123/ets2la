"""
Map data acquisition for UltraPilot (stage 1 of map-based navigation).

Legacy ETS2LA releases publish pre-extracted road-network data on GitLab.  The
TypeScript ETS2LA/maps parser only supports the 1.59 map format.  ETS2 1.60
uses map format 907 and therefore requires a TruckLib based exporter; a 1.59
parser output must never be relabelled as 1.60.

Data source: https://gitlab.com/ETS2LA/data  (index.yaml -> per-version zips).
A full ETS2 map is ~86 MB packed / ~944 MB unpacked, so it is downloaded once
and cached under <app>/map-cache/<key>/.
"""

import os
import json
import logging
import re
import zipfile

try:
    import requests
except Exception:
    requests = None

try:
    import yaml
except Exception:
    yaml = None

from core.paths import app_dir

INDEX_URL = "https://gitlab.com/ETS2LA/data/-/raw/main/index.yaml"
_BASE = INDEX_URL.rsplit("/", 1)[0]  # .../raw/main

# Keep the new choices visible, but generate them only in the independent
# TruckLib application.  This is intentionally not called ``local-game``:
# that previously invoked ETS2LA/maps and produced invalid 1.60 data from a
# parser whose newest supported format is 1.59.
COMPATIBILITY_DATASETS = {
    "ets2-1.60": {
        "game": "ETS2", "version": "1.60", "game_version": "1.60",
        "content": "Base game + official map DLC",
        "source": "trucklib-required", "parser": "TruckLib (format 907)",
    },
    "promods-2.83": {
        "game": "ETS2", "version": "1.60", "game_version": "1.60",
        "mod": "ProMods", "mod_version": "2.83",
        "source": "trucklib-required", "parser": "TruckLib (format 907)",
    },
}

_last_error = ""


def last_error() -> str:
    return _last_error


def _set_error(message: str):
    global _last_error
    _last_error = str(message or "")
    if _last_error:
        logging.error("map_data: %s", _last_error)


def installed_ets2() -> tuple:
    """Return ``(path, major.minor)`` for the actually installed ETS2."""
    try:
        from core.sdk.game_utils import find_scs_games, get_version_for_game
        for path in find_scs_games():
            if "Euro Truck Simulator 2" in path:
                return path, get_version_for_game(path)
    except Exception as exc:
        logging.warning("map_data: could not detect installed ETS2: %s", exc)
    return "", "Unknown"


def dataset_game_version(key: str) -> str:
    entry = (get_index() or {}).get(key, {}) or {}
    return str(entry.get("game_version", entry.get("version", ""))).strip()


def compatible_with_installed_game(key: str) -> tuple:
    """Return ``(compatible, installed_version, explanation)``.

    This deliberately compares major.minor versions.  A 1.59 dataset must
    never be loaded or relabelled as 1.60, even if its JSON schema still parses.
    """
    _path, installed = installed_ets2()
    required = dataset_game_version(key)
    if not required or installed in ("", "Unknown", "0.0"):
        return True, installed, ""
    ok = installed == required
    reason = "" if ok else (
        f"Mapa {key} je urcena pre ETS2 {required}, ale nainstalovana hra je "
        f"ETS2 {installed}.")
    return ok, installed, reason


def cache_dir() -> str:
    d = os.path.join(app_dir(), "map-cache")
    os.makedirs(d, exist_ok=True)
    return d


def dataset_dir(key: str) -> str:
    return os.path.join(cache_dir(), key)


def is_downloaded(key: str) -> bool:
    """Return true only for a complete dataset, never for a partial staging copy."""
    d = dataset_dir(key)
    if not os.path.isdir(d):
        return False
    if key not in COMPATIBILITY_DATASETS:
        # Preserve the exact legacy 1.59 readiness contract.  Those datasets
        # are downloaded and managed by the pre-existing code path.
        for _root, _dirs, files in os.walk(d):
            if any(name == "config.json" or name.endswith(".json")
                   for name in files):
                return True
        return False
    required = {
        "nodes", "roads", "prefabs", "roadlooks",
        "prefabdescriptions", "graph",
    }
    found = set()
    for root, _dirs, files in os.walk(d):
        for f in files:
            low = f.casefold()
            if not low.endswith(".json"):
                continue
            stem = low[:-5]
            for category in required:
                if (stem == category or stem.endswith("-" + category)
                        or stem.endswith("_" + category)):
                    found.add(category)
    if found != required:
        return False
    # New 1.60 datasets are trusted only after the standalone generator's
    # transactional validation marker is present.
    try:
        with open(os.path.join(d, "config.json"), "r", encoding="utf-8") as stream:
            config = json.load(stream)
        generator = config.get("generator") or {}
        validation = config.get("validation") or {}
        packages = config.get("packages")
        valid_packages = (isinstance(packages, list) and bool(packages)
                          and all(isinstance(package, dict)
                                  and re.fullmatch(
                                      r"[0-9a-f]{64}",
                                      str(package.get("sha256", "")))
                                  for package in packages))
        return bool(
            config.get("dataset_key") == key
            and config.get("map_format") == 907
            and str(config.get("game_version_major_minor")) == "1.60"
            and generator.get("library") == "TruckLib"
            and generator.get("trucklib_version") == "0.5.1"
            and config.get("generation_complete") is True
            and validation.get("valid") is True
            and valid_packages
            and (key != "promods-2.83"
                 or str(config.get("promods_version")) == "2.83")
        )
    except (OSError, ValueError, TypeError):
        return False


_index_cache = None


def get_index(force: bool = False) -> dict:
    """Fetch the dataset index (key -> {game, version, path, config})."""
    global _index_cache
    if _index_cache is not None and not force:
        return _index_cache
    if requests is None or yaml is None:
        logging.error("map_data: requests/pyyaml not available.")
        return {}
    try:
        r = requests.get(INDEX_URL, timeout=20)
        if r.status_code == 200:
            _index_cache = yaml.safe_load(r.text) or {}
            for key, entry in COMPATIBILITY_DATASETS.items():
                _index_cache.setdefault(key, dict(entry))
            return _index_cache
    except Exception as e:
        logging.error("map_data: failed to fetch index: %s", e)
    # Keep the known compatibility choices visible while offline. Downloading
    # still performs a real HTTP check, so these entries cannot masquerade as
    # locally available data.
    _index_cache = {
        key: dict(entry) for key, entry in COMPATIBILITY_DATASETS.items()
    }
    return _index_cache


def list_datasets() -> list:
    """Return [{key, game, version, downloaded}] sorted newest-version first."""
    idx = get_index()
    out = []
    for key, v in idx.items():
        out.append({
            "key": key,
            "game": v.get("game", "?"),
            "version": v.get("version", "?"),
            "game_version": v.get("game_version", v.get("version", "?")),
            "mod": v.get("mod"),
            "mod_version": v.get("mod_version"),
            "content": v.get("content"),
            "source": v.get("source", "published-archive"),
            "downloaded": is_downloaded(key),
        })
    out.sort(key=lambda d: (str(d["game"]), str(d["version"])), reverse=True)
    return out


def choose_downloaded_for_game(datasets: list, installed_version: str,
                               wanted: str = ""):
    """Choose a downloaded dataset for exactly the running game version.

    This is used after Steam changes branches.  It prefers the user's selected
    dataset when compatible, then preserves the Base/ProMods preference, but
    never crosses a game-version boundary.
    """
    downloaded = [item for item in datasets if item.get("downloaded")]
    selected = next((item for item in downloaded
                     if item.get("key") == wanted), None)
    if installed_version in ("", "Unknown", "0.0"):
        return selected or (downloaded[0] if downloaded else None)
    exact = [item for item in downloaded
             if str(item.get("game_version", item.get("version", "")))
             == installed_version]
    if selected in exact:
        return selected
    prefer_promods = "promods" in (wanted or "").lower()
    preferred = [item for item in exact
                 if bool(item.get("mod")) == prefer_promods]
    return (preferred or exact or [None])[0]


def suggest_key(game_version: str = None, prefer_promods: bool = False) -> str:
    """Best-guess dataset key for the installed game (e.g. '1.59' -> 'ets2-1.59')."""
    idx = get_index()
    if not idx:
        return ""
    keys = list(idx.keys())
    if game_version:
        vt = game_version.strip()
        variant = "promods" if prefer_promods else "ets2"
        for k in keys:
            entry = idx.get(k, {}) or {}
            compatible = str(entry.get("game_version",
                              entry.get("version", ""))).strip() == vt
            if k.startswith(variant) and (vt in k or compatible):
                return k
        for k in keys:  # any variant matching the version
            if vt in k:
                return k
    # fall back to the first plain ETS2 entry
    for k in keys:
        if k.startswith("ets2-"):
            return k
    return keys[0] if keys else ""


def download(key: str, progress_cb=None) -> bool:
    """
    Download + extract dataset ``key`` into the cache.

    ``progress_cb(fraction, text)`` is called during the download (optional).
    Returns True on success.
    """
    idx = get_index()
    if key not in idx:
        logging.error("map_data: unknown dataset '%s'", key)
        return False
    entry = idx[key]
    if entry.get("source") == "trucklib-required":
        _set_error(
            f"Mapa {key} vyzaduje parser TruckLib pre format 907. "
            "Stary parser ETS2LA/maps podporuje iba ETS2 1.59 a jeho data "
            "sa nesmu pouzit ako mapa 1.60. Pouzite samostatny program "
            "UltraPilot Map Generator; tlacidlo v UltraPilote ho zamerne "
            "nespusta.")
        return False
    if requests is None:
        return False
    out_dir = dataset_dir(key)
    zip_path = out_dir + ".zip"
    os.makedirs(cache_dir(), exist_ok=True)
    last_logged = {"phase": None, "download": -1, "extract": -1}

    def report(frac, text):
        if progress_cb:
            try:
                progress_cb(frac, text)
            except Exception:
                pass
        # Mirror meaningful progress into the main ETS2LA-style runtime log.
        # Throttle byte/chunk updates to 10% steps so the log is informative
        # without producing hundreds of nearly identical records.
        if text.startswith("Sťahujem mapu"):
            bucket = min(10, int(max(0.0, frac - 0.03) / 0.067))
            if bucket != last_logged["download"]:
                last_logged["download"] = bucket
                logging.info("Mapa [%s]: %s", key, text)
        elif text.startswith("Rozbaľujem mapu"):
            bucket = min(10, int(max(0.0, frac - 0.75) / 0.019))
            if bucket != last_logged["extract"]:
                last_logged["extract"] = bucket
                logging.info("Mapa [%s]: %s", key, text)
        elif text != last_logged["phase"]:
            last_logged["phase"] = text
            logging.info("Mapa [%s]: %s", key, text)

    try:
        report(0.0, f"Pripájam sa k serveru… ({key})")
        archive_paths = (entry["path"],) + tuple(entry.get("path_candidates", ()))
        r, attempted = None, []
        for archive_path in archive_paths:
            url = _BASE + archive_path
            attempted.append(url)
            candidate = requests.get(url, stream=True, timeout=30)
            if candidate.status_code == 200:
                r = candidate
                break
            candidate.close()
        if r is None:
            logging.error("map_data: no published package for %s; tried %s",
                          key, ", ".join(attempted))
            return False
        total = int(r.headers.get("content-length", 0)) or 1
        report(0.03, f"Spojenie nadviazané · sťahujem archív {total/1e6:.0f} MB")
        done = 0
        import time as _t
        t0 = _t.time()
        last = 0.0
        with open(zip_path, "wb") as f:
            for chunk in r.iter_content(1024 * 256):
                f.write(chunk)
                done += len(chunk)
                now = _t.time()
                if now - last >= 0.2:          # throttle UI updates
                    last = now
                    el = max(0.1, now - t0)
                    spd = done / el / 1e6      # MB/s
                    pct = done / total * 100
                    report(0.03 + min(0.67, (done / total) * 0.67),
                           f"Sťahujem mapu · {pct:.0f}% · {done/1e6:.0f}/{total/1e6:.0f} MB · {spd:.1f} MB/s")

        report(0.71, "Kontrolujem stiahnutý ZIP archív…")
        with zipfile.ZipFile(zip_path, "r") as z:
            bad = z.testzip()
            if bad:
                raise RuntimeError("Poškodený súbor v archíve: " + bad)

        report(0.74, "Pripravujem priečinok mapy…")
        if os.path.isdir(out_dir):
            import shutil
            shutil.rmtree(out_dir)
        os.makedirs(out_dir, exist_ok=True)
        with zipfile.ZipFile(zip_path, "r") as z:
            members = [m for m in z.infolist() if not m.is_dir()]
            count = max(1, len(members))
            for i, member in enumerate(members, 1):
                z.extract(member, out_dir)
                if i == 1 or i == count or i % max(1, count // 100) == 0:
                    report(0.75 + 0.19 * (i / count),
                           f"Rozbaľujem mapu · {i}/{count} súborov · {i/count*100:.0f}%")
        os.remove(zip_path)

        # Save the config alongside for reference.
        report(0.95, "Sťahujem a kontrolujem konfiguráciu mapy…")
        try:
            config_paths = ((entry.get("config"),)
                            + tuple(entry.get("config_candidates", ())))
            for config_path in filter(None, config_paths):
                cfg = requests.get(_BASE + config_path, timeout=20)
                if cfg.status_code == 200 and yaml is not None:
                    with open(os.path.join(out_dir, "config.json"), "w") as f:
                        config = yaml.safe_load(cfg.text) or {}
                        config.update({
                            "dataset_key": key,
                            "game_version": entry.get("game_version",
                                                      entry.get("version")),
                            "mod": entry.get("mod"),
                            "mod_version": entry.get("mod_version"),
                        })
                        json.dump(config, f, indent=2)
                    break
        except Exception:
            pass

        report(0.98, "Overujem rozbalené mapové súbory…")
        if not is_downloaded(key):
            raise RuntimeError("Mapa je neúplná alebo neobsahuje požadované súbory")
        report(1.0, f"Mapa {key} je pripravená a overená")
        logging.info("map_data: downloaded dataset %s", key)
        return is_downloaded(key)
    except Exception as e:
        logging.exception("map_data: failed to download %s: %s", key, e)
        return False
    finally:
        if os.path.exists(zip_path):
            try:
                os.remove(zip_path)
            except Exception:
                pass
