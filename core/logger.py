"""
Colourful console logging for UltraPilot (ETS2LA-style startup log).

Prints level-coloured lines like:
    [12:01:03] INFO     Engine started
    [12:01:03] WARNING  Telemetry not found
    [12:01:04] ERROR    Plugin crashed
Use setup() once at process start.
"""

import logging
import os

# Enable ANSI colours on Windows terminals.
try:
    os.system("")
except Exception:
    pass

_RESET = "\033[0m"
_COLORS = {
    "DEBUG": "\033[90m",     # grey
    "INFO": "\033[92m",      # green
    "WARNING": "\033[93m",   # yellow
    "ERROR": "\033[91m",     # red
    "CRITICAL": "\033[97;41m",
}
_TAG = "\033[96mUltraPilot\033[0m"  # cyan brand tag


class _ColorFormatter(logging.Formatter):
    def format(self, record):
        color = _COLORS.get(record.levelname, "")
        ts = self.formatTime(record, "%H:%M:%S")
        level = f"{color}{record.levelname:<8}{_RESET}"
        return f"\033[90m[{ts}]{_RESET} {_TAG} {level} {record.getMessage()}"


def setup(level=logging.INFO):
    """Install the colour formatter on the root logger (idempotent)."""
    root = logging.getLogger()
    root.setLevel(level)
    for h in list(root.handlers):
        root.removeHandler(h)
    handler = logging.StreamHandler()
    handler.setFormatter(_ColorFormatter())
    root.addHandler(handler)
