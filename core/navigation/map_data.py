"""
Map data acquisition for UltraPilot (stage 1 of map-based navigation).

ETS2LA publishes pre-extracted road-network data for every game version and mod
(ETS2 1.59/1.58/1.57, ProMods, ATS, TruckersMP, …) on GitLab.  This module lists
those datasets, downloads the one matching the player's game and caches it
locally so later stages can parse nodes/roads/prefabs and steer along the map.

Data source: https://gitlab.com/ETS2LA/data  (index.yaml -> per-version zips).
A full ETS2 map is ~86 MB packed / ~944 MB unpacked, so it is downloaded once
and cached under <app>/map-cache/<key>/.
"""

import os
import json
import logging
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


def cache_dir() -> str:
    d = os.path.join(app_dir(), "map-cache")
    os.makedirs(d, exist_ok=True)
    return d


def dataset_dir(key: str) -> str:
    return os.path.join(cache_dir(), key)


def is_downloaded(key: str) -> bool:
    """A dataset is ready if its folder exists and holds extracted json data."""
    d = dataset_dir(key)
    if not os.path.isdir(d):
        return False
    # Mark complete if our download wrote the config, or any json data exists.
    for root, _dirs, files in os.walk(d):
        for f in files:
            if f == "config.json" or f.endswith(".json"):
                return True
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
            return _index_cache
    except Exception as e:
        logging.error("map_data: failed to fetch index: %s", e)
    return {}


def list_datasets() -> list:
    """Return [{key, game, version, downloaded}] sorted newest-version first."""
    idx = get_index()
    out = []
    for key, v in idx.items():
        out.append({
            "key": key,
            "game": v.get("game", "?"),
            "version": v.get("version", "?"),
            "downloaded": is_downloaded(key),
        })
    out.sort(key=lambda d: (str(d["game"]), str(d["version"])), reverse=True)
    return out


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
            if k.startswith(variant) and vt in k:
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
    if requests is None:
        return False

    entry = idx[key]
    url = _BASE + entry["path"]
    out_dir = dataset_dir(key)
    zip_path = out_dir + ".zip"
    os.makedirs(cache_dir(), exist_ok=True)

    def report(frac, text):
        if progress_cb:
            try:
                progress_cb(frac, text)
            except Exception:
                pass

    try:
        report(0.0, f"Pripájam sa k serveru… ({key})")
        r = requests.get(url, stream=True, timeout=30)
        if r.status_code != 200:
            logging.error("map_data: download HTTP %s for %s", r.status_code, key)
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
            cfg = requests.get(_BASE + entry["config"], timeout=20)
            if cfg.status_code == 200 and yaml is not None:
                with open(os.path.join(out_dir, "config.json"), "w") as f:
                    json.dump(yaml.safe_load(cfg.text), f, indent=2)
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
