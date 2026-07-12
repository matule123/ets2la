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

# Enable ANSI colours on Windows terminals (best effort).
try:
    os.system("")
except Exception:
    pass


def _make_console_handler():
    """Return a rich-based console handler, or a plain ANSI fallback."""
    try:
        from rich.console import Console
        from rich.logging import RichHandler
        # force_terminal so colours show even when stdout is piped/redirected.
        console = Console(force_terminal=True, soft_wrap=False)
        handler = RichHandler(
            console=console,
            show_time=True,
            show_level=True,
            show_path=False,
            markup=True,
            rich_tracebacks=True,
            tracebacks_show_locals=False,
            log_time_format="[%H:%M:%S]",
        )
        # RichHandler already adds time + level; the message carries the rest.
        handler.setFormatter(logging.Formatter("%(message)s", datefmt="[%H:%M:%S]"))
        return handler
    except Exception:
        # Fallback: simple ANSI colour formatter.
        _RESET = "\033[0m"
        _COLORS = {
            "DEBUG": "\033[90m", "INFO": "\033[92m",
            "WARNING": "\033[93m", "ERROR": "\033[91m", "CRITICAL": "\033[97;41m",
        }
        _TAG = "\033[96mUltraPilot\033[0m"

        class _ColorFormatter(logging.Formatter):
            def format(self, record):
                color = _COLORS.get(record.levelname, "")
                ts = self.formatTime(record, "%H:%M:%S")
                level = f"{color}{record.levelname:<8}{_RESET}"
                return f"\033[90m[{ts}]{_RESET} {_TAG} {level} {record.getMessage()}"

        handler = logging.StreamHandler()
        handler.setFormatter(_ColorFormatter())
        return handler


def setup(level=logging.INFO):
    """Install the rich console handler on the root logger + a shared log file."""
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
