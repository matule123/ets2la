"""Lightweight startup window displayed while UltraPilot initializes."""

import os
import sys

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QIcon, QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QGraphicsDropShadowEffect,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
)

try:
    from ui.update_widget import Spinner
except ModuleNotFoundError:
    # Source checkouts may preserve the directory's original upper-case name.
    from UI.update_widget import Spinner


def _icon_path():
    """Resolve the application icon both from source and a frozen build."""
    here = os.path.dirname(os.path.abspath(__file__))
    project = os.path.dirname(here)
    roots = []
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        roots.append(meipass)
    roots.append(
        os.path.dirname(sys.executable)
        if getattr(sys, "frozen", False)
        else project
    )
    roots.append(here)
    for root in roots:
        for name in ("assets/favicon.ico", "assets/logo.png"):
            candidate = os.path.join(root, name)
            if os.path.exists(candidate):
                return candidate
    return os.path.join(project, "assets", "favicon.ico")


class BootSplash(QWidget):
    """Compact ETS2LA-style startup card with an animated progress ring."""

    W, H = 680, 500

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint
            | Qt.WindowType.WindowStaysOnTopHint
            | Qt.WindowType.Tool
        )
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self._build()
        self._center()

    def _build(self):
        self.setObjectName("BootSplash")
        self.setStyleSheet("#BootSplash { background: transparent; }")

        outer = QVBoxLayout(self)
        outer.setContentsMargins(18, 18, 18, 18)

        card = QWidget(self)
        card.setObjectName("Card")
        card.setStyleSheet(
            "#Card { background: #FFFFFF; border: 1px solid #E5E7EB;"
            " border-radius: 20px; }"
        )
        shadow = QGraphicsDropShadowEffect(card)
        shadow.setBlurRadius(36)
        shadow.setOffset(0, 10)
        shadow.setColor(QColor(15, 23, 42, 55))
        card.setGraphicsEffect(shadow)

        layout = QVBoxLayout(card)
        layout.setContentsMargins(42, 34, 42, 30)
        layout.setSpacing(0)

        brand_row = QHBoxLayout()
        brand_row.setSpacing(14)
        brand_row.setAlignment(Qt.AlignmentFlag.AlignCenter)

        icon_label = QLabel()
        icon_label.setFixedSize(58, 58)
        icon_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pixmap = QPixmap(_icon_path())
        if pixmap.isNull():
            icon = QIcon(_icon_path())
            if not icon.isNull():
                pixmap = icon.pixmap(58, 58)
        if not pixmap.isNull():
            icon_label.setPixmap(
                pixmap.scaled(
                    58,
                    58,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            )
        brand_row.addWidget(icon_label)

        wordmark = QLabel("UltraPilot")
        wordmark.setStyleSheet(
            "color: #065F46; font-size: 27px; font-weight: 800;"
            " background: transparent;"
        )
        brand_row.addWidget(wordmark)
        layout.addLayout(brand_row)

        subtitle = QLabel("Autopilot pre Euro Truck Simulator 2")
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        subtitle.setStyleSheet(
            "color: #64748B; font-size: 13px; font-weight: 500;"
            " background: transparent;"
        )
        layout.addSpacing(10)
        layout.addWidget(subtitle)
        layout.addStretch(1)

        self.spinner = Spinner(size=48)
        layout.addWidget(self.spinner, alignment=Qt.AlignmentFlag.AlignCenter)

        self.status_lbl = QLabel("Initializing…")
        self.status_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_lbl.setStyleSheet(
            "color: #0F172A; font-size: 14px; font-weight: 700;"
            " background: transparent;"
        )
        layout.addSpacing(16)
        layout.addWidget(self.status_lbl)

        hint = QLabel("Prvé načítanie mapy môže chvíľu trvať")
        hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        hint.setStyleSheet(
            "color: #94A3B8; font-size: 11px; background: transparent;"
        )
        layout.addSpacing(6)
        layout.addWidget(hint)

        outer.addWidget(card)
        self.setFixedSize(self.W, self.H)

    def set_status(self, text: str):
        self.status_lbl.setText(text)

    def showEvent(self, event):
        super().showEvent(event)
        self.spinner.start()

    def _center(self):
        screen = QApplication.primaryScreen()
        if screen is None:
            return
        area = screen.availableGeometry()
        frame = self.frameGeometry()
        frame.moveCenter(area.center())
        self.move(frame.topLeft())
