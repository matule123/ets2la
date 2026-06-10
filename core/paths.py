"""
Filesystem path resolution that works both from source and when frozen.

When running from source, data lives relative to the project root.  When frozen
by cx_Freeze, modules live inside ``lib/library.zip`` so ``__file__``-relative
paths break — data files (plugins/, assets/, routes/, settings.json) instead sit
next to the executable.  Use :func:`app_dir` to get the correct base directory.
"""

import os
import sys


def is_frozen() -> bool:
    return bool(getattr(sys, "frozen", False))


def app_dir() -> str:
    """Base directory containing assets/, plugins/, etc."""
    if is_frozen():
        # cx_Freeze: data files are copied next to the executable.
        return os.path.dirname(sys.executable)
    # Source: project root is the parent of this file's package (core/).
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def resource(*parts) -> str:
    """Absolute path to a bundled resource, e.g. resource('assets', 'icon.ico')."""
    return os.path.join(app_dir(), *parts)
