"""
Build a real Windows ``.msi`` installer for UltraPilot with cx_Freeze.

Usage:
    pip install cx_Freeze
    python setup_msi.py bdist_msi
    # -> dist/UltraPilot-<version>-win64.msi

The MSI installs UltraPilot into ``Program Files\\UltraPilot``, freezes
``main.py`` into ``UltraPilot.exe``, and creates Start-menu + desktop shortcuts.

Notes
-----
* Plugins are discovered at runtime by listing the ``plugins/`` folder, so the
  whole source tree (core/ plugins/ sdk/ ui/ assets/) is shipped as data next to
  the executable and kept importable.
* Place the third-party SCS telemetry binary at ``assets/scs-telemetry.dll`` to
  have it installed into the game (see core/sdk/game_utils.install_telemetry_dll).
"""

import sys
from cx_Freeze import setup, Executable

# cx_Freeze's dependency walker can recurse deeply on large packages
# (numpy/cv2/PyQt6) — lift the limit so analysis doesn't hit RecursionError.
sys.setrecursionlimit(10000)

VERSION = "0.3.0"

# Pull in everything the engine + plugins + UI import, plus dynamically-loaded
# plugin packages (cx_Freeze can't see importlib-loaded modules on its own).
build_exe_options = {
    "packages": [
        "os", "sys", "multiprocessing", "logging", "json", "math", "time",
        "numpy", "cv2", "mss", "PyQt6", "psutil", "requests",
        "core", "core.sdk", "core.ipc", "core.modules", "core.settings",
        "core.voice", "core.navigation",
        "sdk", "ui",
        "plugins", "plugins.autopilot", "plugins.acc", "plugins.collision",
        "plugins.map", "plugins.tts", "plugins.discord", "plugins.ecodrive",
        "plugins.hud",
    ],
    # Only ship genuine data here. core/, sdk/, ui/, plugins/* are frozen into
    # library.zip via `packages`; listing them again in include_files makes
    # cx_Freeze's MSI generator emit an empty File table (0 files packaged).
    # plugins/ stays as on-disk data only so the plugin manager can list folders.
    "include_files": [
        ("assets/", "assets/"),
        ("plugins/", "plugins/"),
        ("README.md", "README.md"),
    ],
    # torch/torchvision are optional (the lane AI model degrades to OpenCV when
    # absent).  The whole ML stack pulls in ~1.2 GB / tens of thousands of files
    # which overflow the MSI packager and bloat the installer, so exclude it and
    # other large transitive deps not needed at runtime.
    "excludes": [
        "tkinter", "test", "unittest",
        # AI/ML stack (optional lane model only) and its heavy transitive deps:
        "torch", "torchvision", "torchaudio", "sympy", "scipy", "pandas",
        "matplotlib", "IPython", "notebook", "pytest", "gradio", "gradio_client",
        "transformers", "huggingface_hub", "safetensors", "networkx", "sklearn",
        "scikit-learn", "fastapi", "uvicorn", "starlette", "pyarrow", "numba",
    ],
    "include_msvcr": True,
}

# Per-user shortcuts (Start menu + Desktop).
shortcut_table = [
    ("StartMenuShortcut", "ProgramMenuFolder", "UltraPilot",
     "TARGETDIR", "[TARGETDIR]UltraPilot.exe", None, None, None, None, None,
     None, "TARGETDIR"),
    ("DesktopShortcut", "DesktopFolder", "UltraPilot",
     "TARGETDIR", "[TARGETDIR]UltraPilot.exe", None, None, None, None, None,
     None, "TARGETDIR"),
]

bdist_msi_options = {
    # Stable upgrade code so future versions upgrade in place.
    "upgrade_code": "{B6F4A2E1-7C3D-4A9B-9E21-6D0E5A1C8F33}",
    "add_to_path": False,
    "initial_target_dir": r"[ProgramFiles64Folder]\UltraPilot",
    "install_icon": "assets/favicon.ico",
    "data": {"Shortcut": shortcut_table},
}

# cx_Freeze 8.x renamed the Windows GUI base from "Win32GUI" to "gui".
base = "gui" if sys.platform == "win32" else None

executables = [
    Executable(
        "main.py",
        base=base,
        target_name="UltraPilot.exe",
        icon="assets/favicon.ico",
        shortcut_name="UltraPilot",
        shortcut_dir="ProgramMenuFolder",
    )
]

setup(
    name="UltraPilot",
    version=VERSION,
    description="UltraPilot — Autopilot for Euro Truck Simulator 2",
    author="matule123",
    options={
        "build_exe": build_exe_options,
        "bdist_msi": bdist_msi_options,
    },
    executables=executables,
)
