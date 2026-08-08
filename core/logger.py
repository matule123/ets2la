"""Deterministic ETS2LA-style colour console logging for UltraPilot.

    [INF] 12:01:03  Engine started
    [WRN] 12:01:03  Telemetry not found
    [ERR] 12:01:04  Plugin crashed
"""
import logging
import os
import re
import sys

# Enable ANSI colours on Windows terminals (best effort).
try:
    os.system("")
except Exception:
    pass


def _configure_windows_console(kernel32, hwnd):
    """Apply the larger, dark ETS2LA-like console geometry on Windows."""
    try:
        import ctypes
        from ctypes import wintypes

        class COORD(ctypes.Structure):
            _fields_ = [("X", ctypes.c_short), ("Y", ctypes.c_short)]

        class SMALL_RECT(ctypes.Structure):
            _fields_ = [("Left", ctypes.c_short), ("Top", ctypes.c_short),
                        ("Right", ctypes.c_short), ("Bottom", ctypes.c_short)]

        class CONSOLE_FONT_INFOEX(ctypes.Structure):
            _fields_ = [("cbSize", wintypes.ULONG), ("nFont", wintypes.DWORD),
                        ("dwFontSize", COORD), ("FontFamily", wintypes.UINT),
                        ("FontWeight", wintypes.UINT),
                        ("FaceName", wintypes.WCHAR * 32)]

        output = kernel32.GetStdHandle(-11)
        # A larger monospace font and a wider/taller window match the visual
        # density of ETS2LA while retaining a generous scrollback buffer.
        font = CONSOLE_FONT_INFOEX()
        font.cbSize = ctypes.sizeof(CONSOLE_FONT_INFOEX)
        font.dwFontSize = COORD(0, 18)
        font.FontFamily = 54
        font.FontWeight = 400
        font.FaceName = "Consolas"
        kernel32.SetCurrentConsoleFontEx(output, False, ctypes.byref(font))
        kernel32.SetConsoleScreenBufferSize(output, COORD(132, 2000))
        rect = SMALL_RECT(0, 0, 119, 30)
        kernel32.SetConsoleWindowInfo(output, True, ctypes.byref(rect))
        kernel32.SetConsoleTextAttribute(output, 0x0007)

        # Give classic conhost the same dark frame as Windows Terminal.
        dark = ctypes.c_int(1)
        for attribute in (20, 19):  # Win11, then older Win10 fallback
            try:
                if ctypes.windll.dwmapi.DwmSetWindowAttribute(
                        hwnd, attribute, ctypes.byref(dark),
                        ctypes.sizeof(dark)) == 0:
                    break
            except Exception:
                continue
    except Exception:
        pass


def ensure_windows_console():
    """Create/configure the one main runtime console."""
    # Installed source builds launch through pythonw.exe (stdout=None), while
    # frozen builds use a GUI executable. Both need an allocated console. A
    # developer running from an existing terminal already has stdout, so no
    # second window is created there.
    needs_console = getattr(sys, "frozen", False) or sys.stdout is None
    if os.name != "nt":
        return
    try:
        import ctypes
        from multiprocessing import current_process
        k32 = ctypes.windll.kernel32
        # Only the parent owns the console. Spawned processes inherit it.
        if (needs_console and current_process().name == "MainProcess"
                and not k32.GetConsoleWindow()):
            k32.AllocConsole()
            k32.SetConsoleTitleW("UltraPilot · Runtime log")
        if needs_console and k32.GetConsoleWindow():
            # GUI executables and their multiprocessing children start with
            # sys.stdout/sys.stderr=None even though the console is inherited.
            # Bind every process to that same console explicitly.
            sys.stdout = open("CONOUT$", "w", encoding="utf-8", buffering=1)
            sys.stderr = open("CONOUT$", "w", encoding="utf-8", buffering=1)
            if current_process().name == "MainProcess":
                sys.stdin = open("CONIN$", "r", encoding="utf-8")
        if k32.GetConsoleWindow():
            k32.SetConsoleOutputCP(65001)
            k32.SetConsoleCP(65001)
            for stream in (sys.stdout, sys.stderr):
                try:
                    stream.reconfigure(encoding="utf-8", errors="replace")
                except (AttributeError, OSError):
                    pass
        # Enable ANSI colours in the inherited/new console.
        handle = k32.GetStdHandle(-11)
        mode = ctypes.c_uint32()
        if k32.GetConsoleMode(handle, ctypes.byref(mode)):
            k32.SetConsoleMode(handle, mode.value | 0x0004)
        hwnd = k32.GetConsoleWindow()
        if (hwnd and current_process().name == "MainProcess"
                and getattr(sys.stdout, "isatty", lambda: False)()):
            k32.SetConsoleTitleW("UltraPilot")
            _configure_windows_console(k32, hwnd)
    except Exception:
        pass


def is_elevated_windows_process():
    """Return True only when the current Windows token is elevated."""
    if os.name != "nt":
        return False
    try:
        import ctypes
        return bool(ctypes.windll.shell32.IsUserAnAdmin())
    except Exception:
        return False


def print_elevated_runtime_warning(input_fn=None):
    """Explain why an elevated runtime is refused and wait for acknowledgement."""
    ensure_windows_console()
    stream = sys.stdout
    print("\n\033[91m┌─ UltraPilot sa nespustí ako administrátor ───────────────┐\033[0m",
          file=stream)
    print("\033[91m│\033[0m Aplikácia ani jej log nepotrebujú zvýšené oprávnenia.", file=stream)
    print("\033[91m│\033[0m Spusť UltraPilot normálne cez jeho odkaz v ponuke Štart.", file=stream)
    print("\033[91m└───────────────────────────────────────────────────────────┘\033[0m",
          file=stream)
    prompt = "\nStlač Enter pre zatvorenie…"
    try:
        (input_fn or input)(prompt)
    except (EOFError, OSError):
        pass


def print_startup_banner(version="", commit=""):
    """Render the human startup header outside the timestamped log stream."""
    label = "UltraPilot"
    if version:
        label += "  v" + str(version).lstrip("v")
    if commit:
        label += "  ·  " + str(commit)[:7]
    print("\n\033[94m╭──────────────────────────────────────────────────────────╮\033[0m")
    print("\033[94m│\033[0m  " + label.ljust(56) + "\033[94m│\033[0m")
    print("\033[94m│\033[0m  Pripravujem bezpečné jazdné systémy a pluginy…".ljust(58)
          + "\033[94m│\033[0m")
    print("\033[94m╰──────────────────────────────────────────────────────────╯\033[0m\n")


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
        # ETS2LA keeps a clear two-space gutter between timestamp and message.
        # Source/line details remain in ultrapilot.log instead of crowding the
        # live console.
        line = (f"{color}[{tag}]{self.RESET} {self.GREY}{ts}{self.RESET}  "
                f"{self.WHITE}{msg}{self.RESET}")
        if record.exc_info:
            line += "\n" + self.formatException(record.exc_info)
        return line


def _make_console_handler():
    # A deterministic formatter matches ETS2LA's [INF] / [WRN] / [ERR]
    # alignment exactly. Rich's default columns differ by terminal/version.
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(_ETS2LAFormatter())
    return handler


def session_log_offset():
    """Capture a byte boundary so shutdown reports only this application run."""
    try:
        from core.paths import app_dir
        path = os.path.join(app_dir(), "ultrapilot.log")
        return os.path.getsize(path) if os.path.isfile(path) else 0
    except OSError:
        return 0


_LEVEL_LINE = re.compile(
    r"\s(?P<level>WARNING|ERROR|CRITICAL)\s+"
    r"(?P<source>\S+)\s+(?P<message>.*)$")
_LOADED_PLUGIN = re.compile(r"Loaded plugin:\s*([^\s(]+)", re.IGNORECASE)
_MESSAGE_PLUGIN = re.compile(
    r"\[plugin:([^\]]+)\]|plugin\s+['\"]?([A-Za-z0-9_.-]+)",
    re.IGNORECASE)


def _session_log_lines(offset):
    for handler in logging.getLogger().handlers:
        try:
            handler.flush()
        except Exception:
            pass
    try:
        from core.paths import app_dir
        path = os.path.join(app_dir(), "ultrapilot.log")
        with open(path, "rb") as stream:
            stream.seek(max(0, int(offset)))
            return stream.read().decode("utf-8", errors="replace").splitlines()
    except (OSError, ValueError):
        return []


def collect_plugin_issues(offset=0, lines=None):
    """Collect per-plugin warning/error counts from only the current session."""
    lines = list(_session_log_lines(offset) if lines is None else lines)
    known_plugins = set()
    for line in lines:
        match = _LOADED_PLUGIN.search(line)
        if match:
            known_plugins.add(match.group(1).strip().lower())

    issues = {}
    for line in lines:
        match = _LEVEL_LINE.search(line)
        if not match:
            continue
        level = match.group("level")
        source = match.group("source").strip()
        message = match.group("message").strip()
        plugin = None
        named = _MESSAGE_PLUGIN.search(message)
        if named:
            plugin = (named.group(1) or named.group(2) or "").strip().lower()
        elif source.lower().startswith("plugin-"):
            plugin = source[7:].strip().lower()
        elif source.lower() in known_plugins:
            plugin = source.lower()
        if not plugin:
            continue
        bucket = issues.setdefault(plugin, {
            "warnings": 0, "errors": 0,
            "warning_messages": [], "error_messages": [],
        })
        is_error = level in ("ERROR", "CRITICAL")
        key = "errors" if is_error else "warnings"
        messages_key = "error_messages" if is_error else "warning_messages"
        bucket[key] += 1
        if message not in bucket[messages_key] and len(bucket[messages_key]) < 2:
            bucket[messages_key].append(message)
    return issues


def _issue_frame(plugin, item, colour=True):
    """Render one compact ETS2LA log-file box for a plugin."""
    width = 34
    reset = "\033[0m" if colour else ""
    grey = "\033[90m" if colour else ""
    red = "\033[91m" if colour else ""
    yellow = "\033[93m" if colour else ""
    title = str(plugin)[:width - 2]
    title_pad = max(0, width - len(title))
    left = title_pad // 2
    right = title_pad - left
    output = [grey + "┌" + "─" * width + "┐" + reset,
              grey + "│" + " " * left + title + " " * right + "│" + reset]

    def count_row(label, value, color):
        text = f"{label}: {int(value or 0)}"
        return (grey + "│ " + reset + color + text
                + reset + " " * max(0, width - len(text) - 1)
                + grey + "│" + reset)

    output.append(count_row("Errors", item.get("errors", 0), red))
    output.append(count_row("Warnings", item.get("warnings", 0), yellow))
    output.append(grey + "└" + "─" * width + "┘" + reset)
    return "\n".join(output)


def format_plugin_issue_summary(issues, colour=True):
    """Return the same compact per-log count boxes used by ETS2LA."""
    return "\n".join(_issue_frame(plugin, issues[plugin], colour=colour)
                     for plugin in sorted(issues))


def finish_session_log(offset=0, input_fn=None, colour=True):
    """Print the plugin summary and hold the console only when issues exist."""
    issues = collect_plugin_issues(offset)
    summary = format_plugin_issue_summary(issues, colour=colour)
    if not summary:
        return issues
    print("\nErrors and warnings in the log files:\n")
    print(summary)
    try:
        (input_fn or input)(
            "\nLog zostáva otvorený kvôli problémom. Stlač Enter pre zatvorenie…")
    except (EOFError, OSError):
        pass
    return issues


def setup(level=logging.INFO):
    """Install the ETS2LA console handler and shared plain log file."""
    ensure_windows_console()
    root = logging.getLogger()
    root.setLevel(level)
    for h in list(root.handlers):
        root.removeHandler(h)

    # Deterministic colourised console output.
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
