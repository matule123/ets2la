"""
Internal build step: freeze the UltraPilot application with cx_Freeze.

This is NOT the installer — it only turns the source into a runnable
``build/exe.win-amd64-*/UltraPilot.exe`` folder.  The single installer
(build_installer.py) wraps that frozen app into UltraPilot_Installer.exe.

    python freeze_app.py build      # -> build/exe.win-amd64-*/UltraPilot.exe
"""

import sys
from cx_Freeze import setup, Executable

# cx_Freeze's dependency walker can recurse deeply on large packages
# (numpy/cv2/PyQt6) — lift the limit so analysis doesn't hit RecursionError.
sys.setrecursionlimit(10000)

VERSION = "0.4.0"

build_exe_options = {
    "packages": [
        "os", "sys", "multiprocessing", "logging", "json", "math", "time",
        "numpy", "cv2", "mss", "PyQt6", "psutil", "requests", "yaml",
        # pyqtgraph / OpenGL are optional (3D view); cx_Freeze auto-includes them
        # only if installed. Listing them as required packages broke the build
        # when PyOpenGL wasn't installed, so they're intentionally NOT here.
        "core", "core.sdk", "core.ipc", "core.modules", "core.settings",
        "core.voice", "core.navigation",
        "sdk", "ui",
        "plugins", "plugins.autopilot", "plugins.acc", "plugins.collision",
        "plugins.map", "plugins.tts", "plugins.discord", "plugins.ecodrive",
        "plugins.hud",
    ],
    # plugins/ stays as on-disk data so the plugin manager can list folders;
    # assets/ holds the icon, logo and the SCS DLLs.
    "include_files": [
        ("assets/", "assets/"),
        ("plugins/", "plugins/"),
        ("README.md", "README.md"),
    ],
    # The optional AI/ML stack (~1.2 GB) is excluded — the lane model degrades to
    # OpenCV when absent, and bundling it bloats the build enormously.
    "excludes": [
        "tkinter", "test", "unittest",
        "torch", "torchvision", "torchaudio", "sympy", "scipy", "pandas",
        "matplotlib", "IPython", "notebook", "pytest", "gradio", "gradio_client",
        "transformers", "huggingface_hub", "safetensors", "networkx", "sklearn",
        "scikit-learn", "fastapi", "uvicorn", "starlette", "pyarrow", "numba",
    ],
    "include_msvcr": True,
}

# Bundle the optional 3D libs only if they're installed (avoids build failure).
for _opt_pkg, _opt_import in (("pyqtgraph", "pyqtgraph"), ("OpenGL", "OpenGL")):
    try:
        __import__(_opt_import)
        build_exe_options["packages"].append(_opt_pkg)
    except Exception:
        pass

base = "gui" if sys.platform == "win32" else None

executables = [
    Executable(
        "main.py",
        base=base,
        target_name="UltraPilot.exe",
        icon="assets/favicon.ico",
    )
]

setup(
    name="UltraPilot",
    version=VERSION,
    description="UltraPilot — Autopilot for Euro Truck Simulator 2",
    author="matule123",
    options={"build_exe": build_exe_options},
    executables=executables,
)
