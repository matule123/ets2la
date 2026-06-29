"""
Build the pretty UltraPilot installer.

Produces a clean setup folder:

    dist/UltraPilot-Setup/
        UltraPilot_Installer.exe   <- small, branded PyQt wizard (with icon)
        payload/                   <- the frozen application it installs

The user runs UltraPilot_Installer.exe; it copies payload/ to the chosen
location, installs the SCS plugins into the game, sets up ViGEmBus and makes
shortcuts.  Zip the UltraPilot-Setup folder to distribute it.

Why a folder and not a single .exe: the frozen app is a few hundred MB, so a
one-file installer would re-extract all of it to temp on every launch (slow).
Shipping the payload beside a small installer is the standard pattern for big
apps and starts instantly.

Usage:
    pip install cx_Freeze pyinstaller
    python build_installer.py
"""

import os
import sys
import shutil
import subprocess

# Force UTF-8 on stdout/stderr so tick / arrow glyphs print fine everywhere
# (a bare print of \u2714 crashed the build under Windows-1250 consoles).
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

# Enable ANSI VT-100 colour codes on Windows consoles. The old ``os.system("")``
# trick only worked on some terminals; calling SetConsoleMode with
# ENABLE_VIRTUAL_TERMINAL_PROCESSING is the reliable way, so the green/red/cyan
# build messages actually show up coloured everywhere.
_ANSI_OK = False
try:
    import ctypes
    k32 = ctypes.windll.kernel32
    _h = k32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
    _mode = ctypes.c_uint32()
    if k32.GetConsoleMode(_h, ctypes.byref(_mode)):
        _ANSI_OK = bool(k32.SetConsoleMode(_h, _mode.value | 0x0004))
except Exception:
    _ANSI_OK = False

_C = {"g": "\033[92m", "y": "\033[93m", "r": "\033[91m", "c": "\033[96m",
      "b": "\033[1m", "x": "\033[0m"}


def cprint(color, msg):
    """Coloured print: green/yellow/red/cyan/bold. Falls back to plain text if
    the console can't render ANSI colours."""
    code = _C.get(color, "")
    if code and _ANSI_OK:
        try:
            print(code + msg + _C["x"])
            return
        except UnicodeEncodeError:
            pass
    try:
        print(msg)
    except UnicodeEncodeError:
        print(msg.encode("ascii", "replace").decode("ascii"))


def step(n, total, msg):
    cprint("c", f"\n{_C['b']}[{n}/{total}]{_C['x']}{_C['c']} {msg}")


ICON = os.path.join("assets", "favicon.ico")
SETUP_DIR = os.path.join("dist", "UltraPilot-Setup")
PAYLOAD_DIR = os.path.join(SETUP_DIR, "payload")


def _ensure(pkg, import_name=None):
    try:
        __import__(import_name or pkg)
        return True
    except ImportError:
        print(f"Installing {pkg}...")
        subprocess.run([sys.executable, "-m", "pip", "install", pkg], check=False)
        try:
            __import__(import_name or pkg)
            return True
        except ImportError:
            print(f"Could not install {pkg}.")
            return False


def build_payload():
    """Assemble the offline payload (the source tree the installer falls back to
    when GitHub is unreachable)."""
    step(1, 3, "Assembling the offline payload...")
    if os.path.exists(PAYLOAD_DIR):
        shutil.rmtree(PAYLOAD_DIR, ignore_errors=True)
    os.makedirs(PAYLOAD_DIR, exist_ok=True)
    # Skip caches / build artefacts / huge caches so the payload stays small
    # (the real map cache is downloaded at runtime).
    skip = {"__pycache__", ".git", "build", "dist", ".claude",
            "map-cache", "model-cache", "routes", "UltraPilot.egg-info"}

    def _ignore(_d, names):
        return [n for n in names if n in skip]

    for item in ("core", "plugins", "sdk", "ui", "assets"):
        src = os.path.abspath(item)
        if os.path.isdir(src):
            shutil.copytree(src, os.path.join(PAYLOAD_DIR, item),
                            dirs_exist_ok=True, ignore=_ignore)
    for f in ("main.py", "bootloader.py", "requirements.txt", "README.md"):
        if os.path.exists(f):
            shutil.copy2(f, os.path.join(PAYLOAD_DIR, f))
    cprint("g", "  Payload assembled.")
    return PAYLOAD_DIR


def build_installer_exe():
    """Build the single branded installer exe (PyInstaller, onefile/windowed)."""
    step(2, 3, "Building UltraPilot_Installer.exe (PyInstaller)...")
    if not _ensure("pyinstaller", "PyInstaller"):
        return None
    if not os.path.exists(ICON):
        cprint("r", f"  ERROR: icon not found at {ICON}")
        return None
    sep = ";" if os.name == "nt" else ":"
    data = [f"--add-data=assets{sep}assets"]
    for item in ("core", "plugins", "sdk", "ui"):
        data.append(f"--add-data={item}{sep}{item}")
    for f in ("main.py", "bootloader.py", "requirements.txt"):
        data.append(f"--add-data={f}{sep}.")
    cmd = [
        sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean",
        "--onefile", "--windowed", "--name", "UltraPilot_Installer",
        f"--icon={ICON}", *data,
        "--hidden-import=core.sdk.game_utils",
        "--hidden-import=core.sdk.vigembus",
        "installer.py",
    ]
    cprint("y", "  Running PyInstaller...")
    r = subprocess.run(cmd)
    if r.returncode != 0:
        cprint("r", "  ERROR: PyInstaller failed.")
        return None
    exe = os.path.join("dist", "UltraPilot_Installer.exe")
    if not os.path.exists(exe):
        cprint("r", "  ERROR: installer exe not produced.")
        return None
    cprint("g", f"  Installer built: {exe}")
    return exe


def assemble(installer_exe):
    """Move the built exe into the setup folder (payload/ is already there)."""
    step(3, 3, "Assembling the setup folder...")
    os.makedirs(SETUP_DIR, exist_ok=True)
    shutil.copy2(installer_exe, os.path.join(SETUP_DIR, "UltraPilot_Installer.exe"))
    cprint("g", f"\nDone!  Setup folder: {os.path.abspath(SETUP_DIR)}")
    cprint("c", "  Run UltraPilot_Installer.exe inside it, or zip the folder to share.")


def main():
    # Build in the right order: payload first (lives inside SETUP_DIR), then the
    # installer exe (lives in dist/), then move the exe into the setup folder.
    build_payload()
    installer_exe = build_installer_exe()
    if not installer_exe:
        return 1
    assemble(installer_exe)
    return 0


if __name__ == "__main__":
    sys.exit(main())
