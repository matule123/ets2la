"""Boot splash window shown while the main UltraPilot window is initializing.

An app-sized frameless card centered on screen with the UltraPilot icon, a
large rotating spinner and an „Initializing“ label. Shown the moment the UI
process starts and closed once the main window's ``showEvent`` flips the
``ui_ready`` shared-state flag — so the user sees a clear loading state
instead of the HUD or an empty desktop flashing before the dashboard appears.
"""

import os
import sys

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPixmap, QIcon
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel

from ui.update_widget import Spinner


def _icon_path():
    """Resolve the icon asset whether running from source or frozen.

    Prefers favicon.ico (the ETS2LA-style icon) over logo.png."""
    here = os.path.dirname(os.path.abspath(__file__))
    project = os.path.dirname(here)
    roots = []
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        roots.append(meipass)
    roots.append(os.path.dirname(sys.executable) if getattr(sys, "frozen", False) else project)
    roots.append(here)
    for r in roots:
        for name in ("assets/favicon.ico", "assets/logo.png"):
            cand = os.path.join(r, name)
            if os.path.exists(cand):
                return cand
    return os.path.join(project, "assets", "favicon.ico")


class BootSplash(QWidget):
    """An app-sized centered splash card with icon + big spinner + status text."""

    # Match the main app's default window size so the splash feels like the
    # app is already „there“, just loading.
    W, H = 640, 520

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint |
            Qt.WindowType.WindowStaysOnTopHint |
            Qt.WindowType.Tool
        )
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self._build()
        self._center()

    def _build(self):
        # GitHub-style black palette, consistent with the app + installer.
        self.setStyleSheet("BootSplash { background: transparent; }")
        wrap = QVBoxLayout(self)
        wrap.setContentsMargins(0, 0, 0, 0)

        card = QWidget()
        card.setObjectName("Card")
        card.setStyleSheet(
            "#Card { background: #0D1117; border: 1px solid #30363D;"
            " border-radius: 18px; }")
        cl = QVBoxLayout(card)
        cl.setContentsMargins(56, 56, 56, 48)
        cl.setSpacing(28)
        cl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Icon (favicon.ico preferred).
        icon_lbl = QLabel()
        icon_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pm = QPixmap(_icon_path())
        if not pm.isNull():
            icon_lbl.setPixmap(pm.scaledToWidth(
                96, Qt.TransformationMode.SmoothTransformation))
        else:
            ic = QIcon(_icon_path())
            if not ic.isNull():
                icon_lbl.setPixmap(ic.pixmap(96, 96))
            else:
                icon_lbl.setText("UltraPilot")
                icon_lbl.setStyleSheet("color:#2EA043; font-size:32px; font-weight:800;")
        cl.addWidget(icon_lbl)

        # Brand wordmark.
        brand = QLabel("UltraPilot")
        brand.setAlignment(Qt.AlignmentFlag.AlignCenter)
        brand.setStyleSheet("color:#2EA043; font-size:30px; font-weight:800; letter-spacing:0.5px;")
        cl.addWidget(brand)

        # Subtitle.
        sub = QLabel("Autopilot pre Euro Truck Simulator 2")
        sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sub.setStyleSheet("color:#8B949E; font-size:13px; font-weight:500;")
        cl.addWidget(sub)

        cl.addSpacing(8)

        # Big spinning wheel.
        self.spinner = Spinner(size=56)
        cl.addWidget(self.spinner, alignment=Qt.AlignmentFlag.AlignCenter)

        # Status text.
        self.status_lbl = QLabel("Initializing…")
        self.status_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_lbl.setStyleSheet("color:#8B949E; font-size:14px; font-weight:600;")
        cl.addWidget(self.status_lbl)

        wrap.addWidget(card)
        self.setFixedSize(self.W, self.H)

    def set_status(self, text: str):
        self.status_lbl.setText(text)

    def _center(self):
        from PyQt6.QtWidgets import QApplication
        screen = QApplication.primaryScreen().geometry()
        self.move(screen.width() // 2 - self.width() // 2,
                  screen.height() // 2 - self.height() // 2)
