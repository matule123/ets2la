"""
Small always-on-top performance overlay (the „hamburger“ panel).

A compact frameless window anchored to the bottom-left of the screen that shows
the app's total RAM use and a bar per running plugin, refreshing twice a second.
It mirrors the data ``ui/performance.py`` collects (process tree RSS, plugin
worker names) so the two stay consistent.

The overlay is opened from the hamburger (≡) button in the main window's
sidebar footer and can be dragged by its title bar.
"""

import os
from PyQt6.QtCore import Qt, QTimer, QPoint
from PyQt6.QtGui import QPainter, QColor
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QProgressBar, QPushButton,
)

try:
    import psutil
except Exception:
    psutil = None


def _collect():
    """Return (app_rss_mb, [(plugin_name, rss_mb), ...]) for our process tree."""
    if psutil is None:
        return 0.0, []
    me = psutil.Process(os.getpid())
    try:
        root = me.parent() or me
    except Exception:
        root = me
    procs = [root] + root.children(recursive=True)
    seen, app_rss, plugins = set(), 0, []
    for p in procs:
        try:
            if p.pid in seen:
                continue
            seen.add(p.pid)
            rss = p.memory_info().rss
            app_rss += rss
            try:
                mp_name = next((a for a in p.cmdline() if a.startswith("Plugin-")), None)
            except Exception:
                mp_name = None
            label = mp_name or p.name()
            if label.startswith("Plugin-"):
                plugins.append((label.replace("Plugin-", "").capitalize(), rss))
        except Exception:
            continue
    return app_rss / 1e6, [(n, r / 1e6) for n, r in plugins]


class PerfOverlay(QWidget):
    """Frameless, always-on-top, draggable mini performance panel."""

    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint |
            Qt.WindowType.WindowStaysOnTopHint |
            Qt.WindowType.Tool)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, False)
        self.setFixedSize(260, 230)
        self._drag = None
        # Resolve the palette BEFORE _build() so the labels can read _pal.
        from core.theme import palette
        self._pal = palette(state.get("ui_theme", "light") or "light")
        self._build()
        self.setStyleSheet("background-color: " + self._pal['card'] + "; border: 1px solid " + self._pal['border'] + "; border-radius: 14px;")
        # Anchor bottom-left.
        screen = self.screen().geometry() if self.screen() else None
        if screen is not None:
            self.move(24, screen.height() - self.height() - 24)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)

    def restyle(self, theme):
        """Re-apply colours when the theme changes."""
        from core.theme import palette
        self._pal = palette(theme)
        p = self._pal
        self.setStyleSheet("background-color: " + p['card'] + "; border: 1px solid " + p['border'] + "; border-radius: 14px;")
        # Rebuild the labels so the new palette's text colours apply.
        # Simplest reliable path: clear and re-add via a fresh _build-like refresh.
        self.refresh()

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 10, 14, 12)
        root.setSpacing(6)
        head = QHBoxLayout()
        head.setSpacing(6)
        title = QLabel("⚡ Performance")
        title.setStyleSheet("font-size: 13px; font-weight: 800; color: " + self._pal['title'] + ";")
        head.addWidget(title)
        head.addStretch()
        close = QPushButton("✕")
        close.setFixedSize(22, 22)
        close.setCursor(Qt.CursorShape.PointingHandCursor)
        close.setStyleSheet("QPushButton{background:transparent;border:none;color:" + self._pal['muted'] + ";font-size:14px;} QPushButton:hover{color:#EF4444;}")
        close.clicked.connect(self.hide)
        head.addWidget(close)
        root.addLayout(head)

        self.total_lbl = QLabel("UltraPilot: — MB")
        self.total_lbl.setStyleSheet("font-size: 12px; font-weight: 700; color: " + self._pal['text'] + ";")
        root.addWidget(self.total_lbl)

        bar_wrap = QFrame()
        bar_wrap.setStyleSheet("background: transparent; border: none;")
        bw = QVBoxLayout(bar_wrap)
        bw.setContentsMargins(0, 0, 0, 0)
        bw.setSpacing(2)
        self.total_bar = QProgressBar()
        self.total_bar.setFixedHeight(8)
        self.total_bar.setRange(0, 100)
        bw.addWidget(self.total_bar)
        root.addWidget(bar_wrap)

        hint = QLabel("Per plugin:")
        hint.setStyleSheet("font-size: 11px; color: " + self._pal['muted'] + ";")
        root.addWidget(hint)
        self.rows_box = QVBoxLayout()
        self.rows_box.setSpacing(3)
        root.addLayout(self.rows_box)
        root.addStretch()

    def _clear_rows(self):
        while self.rows_box.count():
            w = self.rows_box.takeAt(0).widget()
            if w:
                w.setParent(None)
                w.deleteLater()

    def refresh(self):
        app_mb, plugins = _collect()
        self.total_lbl.setText(f"UltraPilot: {app_mb:.0f} MB")
        # The total bar is relative to a 1 GB soft cap for a quick visual feel.
        self.total_bar.setValue(min(100, int(app_mb / 1024 * 100)))
        self._clear_rows()
        plug_total = sum(r for _, r in plugins) or 1.0
        if not plugins:
            lbl = QLabel("žiadne pluginy")
            lbl.setStyleSheet("font-size: 11px; color: " + self._pal['muted'] + ";")
            self.rows_box.addWidget(lbl)
            return
        for name, mb in sorted(plugins, key=lambda r: -r[1]):
            row = QHBoxLayout()
            row.setSpacing(6)
            n = QLabel(name)
            n.setFixedWidth(90)
            n.setStyleSheet("font-size: 11px; color: " + self._pal['text'] + ";")
            bar = QProgressBar()
            bar.setFixedHeight(7)
            bar.setRange(0, 100)
            bar.setValue(int(100 * mb / plug_total))
            bar.setTextVisible(False)
            val = QLabel(f"{mb:.0f} MB")
            val.setFixedWidth(48)
            val.setStyleSheet("font-size: 11px; color: " + self._pal['muted'] + ";")
            row.addWidget(n)
            row.addWidget(bar, stretch=1)
            row.addWidget(val)
            wrap = QWidget()
            wrap.setLayout(row)
            wrap.setStyleSheet("background: transparent; border: none;")
            self.rows_box.addWidget(wrap)

    # --- drag by the title area ---
    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton and e.position().y() <= 30:
            self._drag = e.globalPosition().toPoint() - self.frameGeometry().topLeft()
            e.accept()

    def mouseMoveEvent(self, e):
        if self._drag is not None and (e.buttons() & Qt.MouseButton.LeftButton):
            self.move(e.globalPosition().toPoint() - self._drag)
            e.accept()

    def mouseReleaseEvent(self, e):
        self._drag = None
