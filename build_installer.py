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
import glob
import shutil
import subprocess

ICON = os.path.join("assets", "favicon.ico")
SETUP_DIR = os.path.join("dist", "UltraPilot-Setup")


def _ensure(pkg, import_name=None):
    try:
        __import__(import_name or pkg)
        return True
    except ImportError:
        print(f"Installing {pkg}…")
        subprocess.run([sys.executable, "-m", "pip", "install", pkg], check=False)
        try:
            __import__(import_name or pkg)
            return True
        except ImportError:
            print(f"Could not install {pkg}.")
            return False


def build_app():
    """Freeze the application with cx_Freeze (creates build/exe.win-amd64-*)."""
    print("=== Step 1/3: freezing the application (cx_Freeze) ===")
    if not _ensure("cx_Freeze", "cx_Freeze"):
        return None
    subprocess.run([sys.executable, "setup_msi.py", "build"], check=True)
    builds = [b for b in glob.glob(os.path.join("build", "exe.win-amd64-*"))
              if os.path.exists(os.path.join(b, "UltraPilot.exe"))]
    if not builds:
        print("ERROR: frozen app not found after build.")
        return None
    return builds[0]


def build_installer_exe():
    """Build a small branded installer exe (no payload bundled)."""
    print("=== Step 2/3: building UltraPilot_Installer.exe (PyInstaller) ===")
    if not _ensure("pyinstaller", "PyInstaller"):
        return None
    sep = ";" if os.name == "nt" else ":"
    cmd = [
        sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean",
        "--onefile", "--windowed",
        "--name", "UltraPilot_Installer",
        f"--icon={ICON}",
        f"--add-data=assets{sep}assets",
        "--hidden-import=core.sdk.game_utils",
        "--hidden-import=core.sdk.vigembus",
        "installer.py",
    ]
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)
    exe = os.path.join("dist", "UltraPilot_Installer.exe")
    return exe if os.path.exists(exe) else None


def assemble(payload_dir, installer_exe):
    """Assemble dist/UltraPilot-Setup/{UltraPilot_Installer.exe, payload/}."""
    print("=== Step 3/3: assembling the setup folder ===")
    if os.path.exists(SETUP_DIR):
        shutil.rmtree(SETUP_DIR)
    os.makedirs(SETUP_DIR, exist_ok=True)
    shutil.copy2(installer_exe, os.path.join(SETUP_DIR, "UltraPilot_Installer.exe"))
    shutil.copytree(payload_dir, os.path.join(SETUP_DIR, "payload"))
    print(f"\n[OK] Done!  Setup folder: {SETUP_DIR}")
    print("  Run UltraPilot_Installer.exe inside it, or zip the folder to share.")


def main():
    payload = build_app()
    if not payload:
        return 1
    installer_exe = build_installer_exe()
    if not installer_exe:
        return 1
    assemble(payload, installer_exe)
    return 0


if __name__ == "__main__":
    sys.exit(main())
