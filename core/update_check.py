"""
Startup update check: compares the local version against the latest GitHub
release and, if a newer one exists and this is a git checkout, runs `git pull`.

Shows a small splash window with a status bar.  Never blocks startup on error —
if GitHub is unreachable or anything fails, it just continues to the app.
"""

import os
import sys
import subprocess

VERSION = "0.3.0"
# Public GitHub repo to check (owner/name).
REPO = "matule123/ets2la"
API_URL = f"https://api.github.com/repos/{REPO}/releases/latest"


def _app_dir():
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _latest_tag():
    """Return the latest release tag from GitHub, or None on any failure."""
    try:
        import requests
        r = requests.get(API_URL, timeout=6)
        if r.status_code == 200:
            return (r.json().get("tag_name") or "").lstrip("vV") or None
    except Exception:
        pass
    return None


def _is_newer(remote, local):
    def parts(v):
        out = []
        for p in str(v).split("."):
            try:
                out.append(int("".join(c for c in p if c.isdigit()) or 0))
            except Exception:
                out.append(0)
        return out
    return parts(remote) > parts(local)


def run_with_splash():
    """Show a splash + status bar while checking/updating. Returns when done."""
    try:
        from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QProgressBar
        from PyQt6.QtGui import QPixmap, QIcon
        from PyQt6.QtCore import Qt, QTimer
    except Exception:
        return  # no Qt → skip silently

    app = QApplication.instance() or QApplication(sys.argv)
    w = QWidget()
    w.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
    w.setFixedSize(420, 220)
    w.setStyleSheet("background:#0E0F13; color:#E6E6E6; font-family:'Segoe UI';")
    lay = QVBoxLayout(w); lay.setContentsMargins(30, 26, 30, 26); lay.setSpacing(12)

    logo = QLabel(); logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
    pm = QPixmap(os.path.join(_app_dir(), "assets", "logo.png"))
    if not pm.isNull():
        logo.setPixmap(pm.scaledToWidth(150, Qt.TransformationMode.SmoothTransformation))
    lay.addWidget(logo)

    status = QLabel("Checking for updates…")
    status.setAlignment(Qt.AlignmentFlag.AlignCenter)
    status.setStyleSheet("color:#9AA0A6;")
    lay.addWidget(status)

    bar = QProgressBar(); bar.setRange(0, 0)  # indeterminate
    bar.setStyleSheet("QProgressBar{background:#16181D;border:1px solid #2C2F36;"
                      "border-radius:6px;height:16px;}"
                      "QProgressBar::chunk{background:#10B981;border-radius:5px;}")
    lay.addWidget(bar)
    w.show()

    def work():
        try:
            remote = _latest_tag()
            if remote and _is_newer(remote, VERSION):
                status.setText(f"New version {remote} found — updating…")
                app.processEvents()
                if os.path.isdir(os.path.join(_app_dir(), ".git")):
                    try:
                        subprocess.run(["git", "-C", _app_dir(), "pull", "--ff-only"],
                                       capture_output=True, timeout=60)
                        status.setText(f"Updated to {remote}. Starting…")
                    except Exception:
                        status.setText("Update failed — starting current version…")
                else:
                    status.setText(f"Update {remote} available (download manually). Starting…")
            else:
                status.setText("Up to date. Starting…")
        except Exception:
            status.setText("Starting…")
        app.processEvents()
        QTimer.singleShot(700, w.close)

    QTimer.singleShot(150, work)
    # Run the splash event loop until it closes.
    while w.isVisible():
        app.processEvents()
        import time
        time.sleep(0.02)
