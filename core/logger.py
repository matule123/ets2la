"""
Rich console logging for UltraPilot (ETS2LA-style startup log).

Uses the ``rich`` library to print colourised, panel-style log lines in the
terminal — like the ETS2LA reference dashboard. Falls back to a plain ANSI
formatter if ``rich`` is not installed so logging never breaks.

    12:01:03 | INFO     | Engine started
    12:01:03 | WARNING  | Telemetry not found
    12:01:04 | ERROR    | Plugin crashed
"""
import logging
import os
import sys

# Enable ANSI colours on Windows terminals (best effort).
try:
    os.system("")
except Exception:
    pass


def _ensure_windows_console():
    """Create the one main runtime console for a frozen GUI build."""
    # Installed source builds launch through pythonw.exe (stdout=None), while
    # frozen builds use a GUI executable. Both need an allocated console. A
    # developer running from an existing terminal already has stdout, so no
    # second window is created there.
    needs_console = getattr(sys, "frozen", False) or sys.stdout is None
    if os.name != "nt" or not needs_console:
        return
    try:
        import ctypes
        from multiprocessing import current_process
        k32 = ctypes.windll.kernel32
        # Only the parent owns the console. Spawned processes inherit it.
        if current_process().name == "MainProcess" and not k32.GetConsoleWindow():
            k32.AllocConsole()
            k32.SetConsoleTitleW("UltraPilot · Runtime log")
        if k32.GetConsoleWindow():
            # GUI executables and their multiprocessing children start with
            # sys.stdout/sys.stderr=None even though the console is inherited.
            # Bind every process to that same console explicitly.
            sys.stdout = open("CONOUT$", "w", encoding="utf-8", buffering=1)
            sys.stderr = open("CONOUT$", "w", encoding="utf-8", buffering=1)
            if current_process().name == "MainProcess":
                sys.stdin = open("CONIN$", "r", encoding="utf-8")
        # Enable ANSI colours in the inherited/new console.
        handle = k32.GetStdHandle(-11)
        mode = ctypes.c_uint32()
        if k32.GetConsoleMode(handle, ctypes.byref(mode)):
            k32.SetConsoleMode(handle, mode.value | 0x0004)
    except Exception:
        pass


class _ETS2LAFormatter(logging.Formatter):
    RESET = "\033[0m"
    GREY = "\033[90m"
    WHITE = "\033[97m"
    COLORS = {
        logging.DEBUG: "\033[96m", logging.INFO: "\033[92m",
        logging.WARNING: "\033[93m", logging.ERROR: "\033[91m",
        logging.CRITICAL: "\033[97;41m",
    }
    TAGS = {
        logging.DEBUG: "DBG", logging.INFO: "INF", logging.WARNING: "WRN",
        logging.ERROR: "ERR", logging.CRITICAL: "CRT",
    }

    def format(self, record):
        color = self.COLORS.get(record.levelno, self.WHITE)
        tag = self.TAGS.get(record.levelno, "LOG")
        ts = self.formatTime(record, "%H:%M:%S")
        msg = record.getMessage()
        source = f"{record.filename}:{record.lineno}"
        # Aim the source column at 96, while allowing long messages to remain
        # intact instead of truncating useful diagnostics.
        visible = 6 + 9 + len(msg)
        gap = " " * max(2, 96 - visible)
        line = (f"{color}[{tag}]{self.RESET} {self.GREY}{ts}{self.RESET}  "
                f"{self.WHITE}{msg}{self.RESET}{gap}{self.GREY}{source}{self.RESET}")
        if record.exc_info:
            line += "\n" + self.formatException(record.exc_info)
        return line


def _make_console_handler():
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(_ETS2LAFormatter())
    return handler


def setup(level=logging.INFO):
    """Install the rich console handler on the root logger + a shared log file."""
    _ensure_windows_console()
    root = logging.getLogger()
    root.setLevel(level)
    for h in list(root.handlers):
        root.removeHandler(h)

    # Rich colourised console output.
    root.addHandler(_make_console_handler())

    # Plain log FILE so errors from every process are captured and can be shared.
    try:
        from core.paths import app_dir
        path = os.path.join(app_dir(), "ultrapilot.log")
        fh = logging.FileHandler(path, mode="a", encoding="utf-8")
        fh.setFormatter(logging.Formatter(
            "%(asctime)s %(levelname)-8s %(processName)s %(message)s"))
        root.addHandler(fh)
        root.info("Logging to %s", path)
    except Exception:
        pass
