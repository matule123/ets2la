"""Boot splash window shown while the main UltraPilot window is initializing.

A small frameless, translucent card centered on screen with the logo, a
rotating spinner (reused from ``ui.update_widget``) and an „Initializing“
label. It is shown the moment the UI process starts and closed once the main
window's ``showEvent`` flips the ``ui_ready`` shared-state flag — so the user
sees a clear loading state instead of the HUD or an empty desktop flashing
before the dashboard appears.
"""

import os
import sys

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QIcon
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel

from ui.update_widget import Spinner


def _logo_path():
    """Resolve the logo asset whether running from source or frozen."""
    here = os.path.dirname(os.path.abspath(__file__))
    project = os.path.dirname(here)
    roots = []
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        roots.append(meipass)
    roots.append(os.path.dirname(sys.executable) if getattr(sys, "frozen", False) else project)
    roots.append(here)
    for r in roots:
        for name in ("assets/logo.png", "assets/favicon.ico"):
            cand = os.path.join(r, name)
            if os.path.exists(cand):
                return cand
    return os.path.join(project, "assets", "logo.png")


class BootSplash(QWidget):
    """A compact centered splash card with logo + spinner + status text."""

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
        # Outer translucent widget; the visible card is styled via QSS.
        self.setStyleSheet("BootSplash { background: transparent; }")
        wrap = QVBoxLayout(self)
        wrap.setContentsMargins(0, 0, 0, 0)

        card = QWidget()
        card.setObjectName("Card")
        card.setStyleSheet(
            "#Card { background: #1E232B; border: 1px solid #3D4654;"
            " border-radius: 18px; }")
        cl = QVBoxLayout(card)
        cl.setContentsMargins(40, 36, 40, 32)
        cl.setSpacing(16)
        cl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Logo.
        logo = QLabel()
        logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pm = QPixmap(_logo_path())
        if not pm.isNull():
            logo.setPixmap(pm.scaledToWidth(
                84, Qt.TransformationMode.SmoothTransformation))
        else:
            # Fallback to the window icon if the PNG is missing.
            ic = QIcon(_logo_path())
            if not ic.isNull():
                logo.setPixmap(ic.pixmap(84, 84))
            else:
                logo.setText("UltraPilot")
                logo.setStyleSheet("color:#34D399; font-size:28px; font-weight:800;")
        cl.addWidget(logo)

        # Brand wordmark.
        brand = QLabel("UltraPilot")
        brand.setAlignment(Qt.AlignmentFlag.AlignCenter)
        brand.setStyleSheet("color:#34D399; font-size:22px; font-weight:800;")
        cl.addWidget(brand)

        # Spinner + status row.
        row = QVBoxLayout()
        row.setSpacing(8)
        row.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.spinner = Spinner(size=26)
        row.addWidget(self.spinner, alignment=Qt.AlignmentFlag.AlignCenter)
        self.status_lbl = QLabel("Initializing…")
        self.status_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_lbl.setStyleSheet("color:#9AA4B2; font-size:13px; font-weight:600;")
        row.addWidget(self.status_lbl)
        cl.addLayout(row)

        wrap.addWidget(card)
        self.adjustSize()

    def set_status(self, text: str):
        self.status_lbl.setText(text)

    def _center(self):
        from PyQt6.QtWidgets import QApplication
        screen = QApplication.primaryScreen().geometry()
        self.move(screen.width() // 2 - self.width() // 2,
                  screen.height() // 2 - self.height() // 2)
