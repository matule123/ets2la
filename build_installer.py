"""
Build the pretty UltraPilot installer.

Produces a SINGLE self-contained exe:

    dist/UltraPilot_Installer.exe   <- small, branded PyQt wizard (with icon)

The installer downloads the latest application sources directly from GitHub
(git clone → zip archive → raw file-by-file fallback) at install time, so there
is no bundled payload — the build output is one exe you can ship on its own.
Only ``assets/`` (DLLs, logo, icon) and ``languages/`` (bundled sk + en) are
packed inside the exe; everything else comes from the repository on install.

Note: the repository must be PUBLIC (or a ``GITHUB_TOKEN`` must be set at install
time) for the download to succeed — see the installer module docs.

Usage:
    pip install pyinstaller
    python build_installer.py
"""

import os
import sys
import shutil
import subprocess
import time

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
    cprint("c", f"\n[{n}/{total}] {msg}")


def info(msg):
    cprint("c", "      • " + msg)


def ok(msg):
    cprint("g", "      ✓ " + msg)


def warn(msg):
    cprint("y", "      ! " + msg)


def fail(msg, hint=None):
    cprint("r", "      ✗ " + msg)
    if hint:
        cprint("y", "        Riešenie: " + hint)


ICON = os.path.join("assets", "favicon.ico")


def _build_commit():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], text=True,
            stderr=subprocess.DEVNULL, timeout=8).strip()
    except Exception:
        return "nezistený"


def _ensure(pkg, import_name=None):
    try:
        __import__(import_name or pkg)
        return True
    except ImportError:
        info(f"Balík {pkg} chýba — skúšam ho doinštalovať…")
        subprocess.run([sys.executable, "-m", "pip", "install", pkg], check=False)
        try:
            __import__(import_name or pkg)
            return True
        except ImportError:
            fail(f"Balík {pkg} sa nepodarilo nainštalovať.",
                 f"Spusti: {sys.executable} -m pip install {pkg}")
            return False


def build_installer_exe():
    """Build the single branded installer exe (PyInstaller, onefile/windowed).

    Output: ``dist/UltraPilot_Installer.exe`` — one self-contained file.

    The exe bundles only what the wizard itself needs to run and to install the
    app: ``assets/`` (SCS DLLs, icon, logo), ``languages/`` (bundled sk + en)
    and the Python modules it lazy-imports (``core.sdk.game_utils`` /
    ``core.sdk.vigembus``). The actual application source is NOT bundled — the
    installer always pulls the latest from GitHub, so the build stays tiny and
    users always get a fresh copy."""
    started = time.monotonic()
    cprint("c", "\n============================================================")
    cprint("c", "  UltraPilot · zostavenie inštalátora")
    cprint("c", "============================================================")

    step(1, 5, "Kontrolujem buildovacie prostredie")
    info("Python: " + sys.version.split()[0])
    info("Commit zdrojov: " + _build_commit())
    info("Pracovný priečinok: " + os.path.abspath(os.curdir))
    if not _ensure("pyinstaller", "PyInstaller"):
        return None
    ok("PyInstaller je pripravený")

    step(2, 5, "Kontrolujem súbory inštalátora")
    if not os.path.exists(ICON):
        fail("Chýba ikona: " + ICON,
             "obnov súbor assets/favicon.ico a spusti build znova")
        return None
    for folder in ("assets", "languages"):
        if not os.path.isdir(folder):
            fail("Chýba priečinok: " + folder)
            return None
        info(f"{folder}/ · {sum(len(files) for _, _, files in os.walk(folder))} súborov")
    ok("Ikona, assety a jazyky sú dostupné")

    step(3, 5, "Pripravujem konfiguráciu balíka")
    sep = ";" if os.name == "nt" else ":"
    # Bundle only the installer's own runtime data. App sources come from GitHub.
    data = [
        f"--add-data=assets{sep}assets",
        f"--add-data=languages{sep}languages",
    ]
    cmd = [
        sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean",
        "--log-level", "WARN",
        "--onefile", "--windowed", "--name", "UltraPilot_Installer",
        f"--icon={ICON}", *data,
        # These are lazy-imported inside InstallWorker.run(); tell PyInstaller
        # to collect them (and their ``core/sdk`` package) into the bundle.
        "--hidden-import=core.sdk.game_utils",
        "--hidden-import=core.sdk.vigembus",
        "--collect-submodules=core.sdk",
        "installer.py",
    ]
    info("Režim: jeden samostatný EXE súbor bez konzolového okna")
    info("Aplikácia sa pri inštalácii stiahne z najnovšieho commitu na GitHube")
    ok("Konfigurácia je pripravená")

    step(4, 5, "Zostavujem UltraPilot_Installer.exe")
    warn("Táto fáza môže trvať niekoľko minút. Okno nezatváraj.")
    r = subprocess.run(cmd)
    if r.returncode != 0:
        fail(f"PyInstaller skončil s chybovým kódom {r.returncode}.",
             "pozri posledné červené riadky vyššie")
        return None
    ok("Balenie programu bolo dokončené")

    step(5, 5, "Overujem výsledný inštalátor")
    exe = os.path.join("dist", "UltraPilot_Installer.exe")
    if not os.path.exists(exe):
        fail("Build skončil bez výsledného EXE súboru.",
             "skontroluj priečinok build/ a výpis PyInstalleru")
        return None
    size_mb = os.path.getsize(exe) / (1024 * 1024)
    elapsed = time.monotonic() - started
    ok(f"Súbor existuje a má {size_mb:.1f} MB")
    ok(f"Build dokončený za {elapsed:.1f} sekundy")
    return exe


def main():
    exe = build_installer_exe()
    if not exe:
        return 1
    cprint("g", "\n============================================================")
    cprint("g", "  HOTOVO · inštalátor je pripravený")
    cprint("g", "============================================================")
    cprint("c", "  Umiestnenie: " + os.path.abspath(exe))
    cprint("c", "  Na zdieľanie stačí tento jediný EXE súbor.")
    cprint("c", "  Počas inštalácie si stiahne aktuálne súbory z GitHubu.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
