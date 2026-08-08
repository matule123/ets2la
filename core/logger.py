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
import re
import sys

# Enable ANSI colours on Windows terminals (best effort).
try:
    os.system("")
except Exception:
    pass


def ensure_windows_console():
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
    try:
        from rich.console import Console
        from rich.logging import RichHandler
        handler = RichHandler(
            console=Console(), rich_tracebacks=True, markup=False,
            show_time=True, show_level=True, show_path=True,
            log_time_format="%H:%M:%S",
        )
        handler.setFormatter(logging.Formatter("%(message)s"))
        return handler
    except ImportError:
        pass
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


def _issue_frame(title, rows, colour=""):
    reset = "\033[0m" if colour else ""
    width = 64
    output = [colour + "┌─ " + title + " " + "─" * max(
        1, width-len(title)-4) + "┐" + reset]
    for plugin, count, sample in rows:
        count_text = f"{count}×"
        output.append(colour + "│ " + (plugin + "  " + count_text)[:width-2].ljust(
            width-2) + " │" + reset)
        if sample:
            clean = " ".join(sample.split())
            output.append(colour + "│   " + clean[:width-5].ljust(width-5)
                          + " │" + reset)
    output.append(colour + "└" + "─" * width + "┘" + reset)
    return "\n".join(output)


def format_plugin_issue_summary(issues, colour=True):
    """Return ETS2LA-style amber/red frames for affected plugins."""
    warning_rows, error_rows = [], []
    for plugin in sorted(issues):
        item = issues[plugin]
        if item.get("warnings"):
            warning_rows.append((plugin, item["warnings"],
                                 (item.get("warning_messages") or [""])[0]))
        if item.get("errors"):
            error_rows.append((plugin, item["errors"],
                               (item.get("error_messages") or [""])[0]))
    frames = []
    if warning_rows:
        frames.append(_issue_frame(
            "PLUGINY S UPOZORNENÍM", warning_rows,
            "\033[38;5;214m" if colour else ""))
    if error_rows:
        frames.append(_issue_frame(
            "PLUGINY S CHYBOU", error_rows,
            "\033[91m" if colour else ""))
    return "\n\n".join(frames)


def finish_session_log(offset=0, input_fn=None, colour=True):
    """Print the plugin summary and hold the console only when issues exist."""
    issues = collect_plugin_issues(offset)
    summary = format_plugin_issue_summary(issues, colour=colour)
    if not summary:
        return issues
    print("\nKontrola pluginov za toto spustenie:\n")
    print(summary)
    try:
        (input_fn or input)(
            "\nLog zostáva otvorený kvôli problémom. Stlač Enter pre zatvorenie…")
    except (EOFError, OSError):
        pass
    return issues


def setup(level=logging.INFO):
    """Install the rich console handler on the root logger + a shared log file."""
    ensure_windows_console()
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
