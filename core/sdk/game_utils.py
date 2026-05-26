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
