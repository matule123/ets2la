import os
import json
import subprocess
import logging
from pathlib import Path

try:
    import winreg
except ImportError:
    winreg = None

try:
    import vdf
except ImportError:
    vdf = None

try:
    from win32api import GetFileVersionInfo, LOWORD, HIWORD
except ImportError:
    GetFileVersionInfo = LOWORD = HIWORD = None

def get_steam_install_folder():
    if os.name == "nt" and winreg:
        try:
            return winreg.QueryValueEx(
                winreg.OpenKey(winreg.HKEY_CURRENT_USER, "SOFTWARE\\Valve\\Steam"), "SteamPath"
            )[0]
        except Exception:
            pass
    return r"C:\Program Files (x86)\Steam" if os.name == "nt" else os.path.expanduser("~/.steam/steam")

def read_steam_library_folders():
    steam_folder = get_steam_install_folder()
    library_file = os.path.join(steam_folder, "steamapps", "libraryfolders.vdf")

    # Fallback for some installations
    if not os.path.exists(library_file):
        library_file = r"D:\Program Files (x86)\Steam\steamapps\libraryfolders.vdf"

    if not os.path.exists(library_file):
        return []

    try:
        with open(library_file, "r") as f:
            if vdf:
                data = vdf.load(f)
                libraries = []
                for key in data.get("libraryfolders", {}):
                    if key.isnumeric():
                        libraries.append(data["libraryfolders"][key]["path"])
                return libraries
            else:
                # Basic fallback if vdf is not installed yet during initial setup
                # This is a crude way to find paths if vdf is missing
                return []
    except Exception as e:
        logging.error(f"Error reading library folders: {e}")
        return []

def find_scs_games():
    libraries = read_steam_library_folders()
    if not libraries:
        libraries = [r"C:\Games", r"C:\Program Files (x86)\Steam"] # Common defaults

    found_games = []
    for library in libraries:
        # ETS2
        ets2_path = os.path.join(library, "steamapps", "common", "Euro Truck Simulator 2")
        if os.path.exists(os.path.join(ets2_path, "base.scs")):
            found_games.append(ets2_path)

        # ATS
        ats_path = os.path.join(library, "steamapps", "common", "American Truck Simulator")
        if os.path.exists(os.path.join(ats_path, "base.scs")):
            found_games.append(ats_path)

    return found_games

def get_version_number(filename):
    if os.name == "nt" and GetFileVersionInfo:
        try:
            info = GetFileVersionInfo(filename, os.sep)
            ms = info["FileVersionMS"]
            ls = info["FileVersionLS"]
            return HIWORD(ms), LOWORD(ms), HIWORD(ls), LOWORD(ls)
        except Exception:
            return (0, 0, 0, 0)
    return (0, 0, 0, 0)

def get_version_for_game(game_path):
    if "Euro Truck Simulator" in game_path:
        exe_path = os.path.join(game_path, "bin", "win_x64", "eurotrucks2.exe")
        version = get_version_number(exe_path)
        return ".".join([str(i) for i in version[:2]])
    elif "American Truck Simulator" in game_path:
        exe_path = os.path.join(game_path, "bin", "win_x64", "amtrucks.exe")
        version = get_version_number(exe_path)
        return ".".join([str(i) for i in version[:2]])
    return "Unknown"


def install_game_dlls(assets_dir: str, names=None) -> list:
    """
    Copy the SCS SDK plugin DLLs from ``assets_dir`` into every detected game's
    ``bin/win_x64/plugins/`` folder so the game shares telemetry AND accepts
    control input.

    ``names`` defaults to both the telemetry and controller plugins.  Returns the
    list of plugin folders written to.  Files already locked by a running game
    are skipped quietly (they're usually already the right version).
    """
    import shutil

    if names is None:
        names = ["scs-telemetry.dll", "scs_sdk_controller.dll"]

    installed = []
    for game_path in find_scs_games():
        plugins_dir = os.path.join(game_path, "bin", "win_x64", "plugins")
        try:
            os.makedirs(plugins_dir, exist_ok=True)
        except Exception as e:
            logging.error("Could not create %s: %s", plugins_dir, e)
            continue
        for name in names:
            src = os.path.join(assets_dir, name)
            if not os.path.exists(src):
                logging.info("DLL %s not found in assets — skipping.", name)
                continue
            dst = os.path.join(plugins_dir, name)
            try:
                shutil.copy2(src, dst)
                logging.info("Installed %s into %s", name, plugins_dir)
            except Exception as e:
                # Most common cause: the game is running and has the DLL locked.
                logging.info("Skipped %s (in use / locked?): %s", name, e)
        if plugins_dir not in installed:
            installed.append(plugins_dir)
    return installed


def install_telemetry_dll(dll_source: str) -> list:
    """
    Backwards-compatible helper: install just the telemetry DLL given its path.
    Prefer :func:`install_game_dlls` to install the whole SDK at once.
    """
    import shutil

    if not dll_source or not os.path.exists(dll_source):
        logging.warning("Telemetry DLL not found at %s — skipping game install. "
                        "Place 'scs-telemetry.dll' in assets/ to enable telemetry.",
                        dll_source)
        return []

    installed = []
    for game_path in find_scs_games():
        plugins_dir = os.path.join(game_path, "bin", "win_x64", "plugins")
        try:
            os.makedirs(plugins_dir, exist_ok=True)
            shutil.copy2(dll_source, os.path.join(plugins_dir, "scs-telemetry.dll"))
            installed.append(plugins_dir)
            logging.info("Installed telemetry DLL into %s", plugins_dir)
        except Exception as e:
            logging.info("Skipped telemetry DLL (in use / locked?): %s", e)
    return installed
