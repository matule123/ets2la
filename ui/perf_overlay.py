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
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QGraphicsDropShadowEffect,
)

try:
    import psutil
except Exception:
    psutil = None


def _collect(state=None):
    """Return (app_rss_mb, app_cpu_pct, [(plugin_name, rss_mb, cpu_pct), ...])."""
    if psutil is None:
        return 0.0, 0.0, []
    me = psutil.Process(os.getpid())
    try:
        root = me.parent() or me
    except Exception:
        root = me
    procs = [root] + root.children(recursive=True)
    seen, app_rss, app_cpu, plugins = set(), 0, 0.0, []
    for p in procs:
        try:
            if p.pid in seen:
                continue
            seen.add(p.pid)
            rss = p.memory_info().rss
            app_rss += rss
            try:
                cpu = p.cpu_percent(interval=None)
            except Exception:
                cpu = 0.0
            app_cpu += cpu
            # Plugin names cannot be recovered reliably from a spawned
            # python.exe command line on Windows. Exact PIDs are published by
            # PluginManager and resolved below.
        except Exception:
            continue
    published = state.get("plugin_processes", {}) if state is not None else {}
    for name, pid in dict(published or {}).items():
        try:
            proc = psutil.Process(int(pid))
            plugins.append((str(name).replace("_", " ").title(),
                            proc.memory_info().rss, proc.cpu_percent(interval=None)))
        except Exception:
            continue
    return app_rss / 1e6, app_cpu, [(n, r / 1e6, c) for n, r, c in plugins]


class PerfOverlay(QWidget):
    """Frameless, always-on-top, draggable mini performance panel."""

    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint |
            Qt.WindowType.WindowStaysOnTopHint |
            Qt.WindowType.Tool)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setFixedSize(330, 390)
        self._drag = None
        # Resolve the palette BEFORE _build() so the labels can read _pal.
        from core.theme import palette
        self._pal = palette(state.get("ui_theme", "light") or "light")
        self._build()
        self._apply_window_style()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)

    def restyle(self, theme):
        """The reference performance console intentionally stays dark."""
        from core.theme import palette
        self._pal = palette(theme)
        self._apply_window_style()
        self.refresh()

    def _apply_window_style(self):
        self.setStyleSheet(
            "PerfOverlay{background:transparent;}"
            "QFrame#PerfSurface{background:#1D1D1F;"
            "border:1px solid #D6D6D8;border-radius:5px;}"
            "QFrame#PerfSurface QWidget{background:transparent;border:none;}"
            "QLabel{background:transparent;border:none;}"
            "QLabel#PerfTitle{font-family:Consolas;font-size:12px;"
            "font-weight:700;color:#F3F4F6;}"
            "QLabel#PerfBranch{font-family:Consolas;font-size:11px;"
            "font-weight:650;color:#7DE3F4;}"
            "QLabel#PerfValue{font-family:Consolas;font-size:11px;"
            "color:#F3F4F6;}"
            "QLabel#PerfMuted{font-family:Consolas;font-size:10px;"
            "color:#9CA3AF;}")

    def show_above(self, anchor):
        """Open as a compact popover directly above the performance button."""
        point = anchor.mapToGlobal(QPoint(0, 0))
        screen = anchor.screen().availableGeometry() if anchor.screen() else None
        x = point.x()
        y = point.y() - self.height() - 10
        if screen is not None:
            x = max(screen.left() + 8, min(x, screen.right() - self.width() - 8))
            y = max(screen.top() + 8, min(y, screen.bottom() - self.height() - 8))
        self.move(x, y)
        self.show()
        self.raise_()

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(0)
        self.surface = QFrame()
        self.surface.setObjectName("PerfSurface")
        shadow = QGraphicsDropShadowEffect(self.surface)
        shadow.setBlurRadius(22)
        shadow.setOffset(0, 6)
        shadow.setColor(QColor(0, 0, 0, 105))
        self.surface.setGraphicsEffect(shadow)
        root.addWidget(self.surface)
        body = QVBoxLayout(self.surface)
        body.setContentsMargins(13, 12, 13, 13)
        body.setSpacing(4)
        title = QLabel("┌ Plugins")
        title.setObjectName("PerfTitle")
        body.addWidget(title)
        self.rows_box = QVBoxLayout()
        self.rows_box.setSpacing(2)
        body.addLayout(self.rows_box)
        self.total_lbl = QLabel("└ Total:  — MB")
        self.total_lbl.setObjectName("PerfTitle")
        body.addWidget(self.total_lbl)
        body.addStretch(1)

    def _clear_rows(self):
        while self.rows_box.count():
            w = self.rows_box.takeAt(0).widget()
            if w:
                w.setParent(None)
                w.deleteLater()

    def refresh(self):
        app_mb, _app_cpu, plugins = _collect(self.state)
        desired = max(128, min(570, 88 + max(1, len(plugins)) * 25))
        if desired != self.height():
            old_bottom = self.y() + self.height()
            self.setFixedHeight(desired)
            screen = self.screen().availableGeometry() if self.screen() else None
            y = old_bottom - desired
            if screen is not None:
                y = max(screen.top() + 8, min(y, screen.bottom() - desired - 8))
            self.move(self.x(), y)
        total = f"{app_mb / 1024:.2f} GB" if app_mb >= 1024 else f"{app_mb:.0f} MB"
        self.total_lbl.setText(f"└ Total:  {total}")
        self._clear_rows()
        if not plugins:
            lbl = QLabel("│  čakám na procesy pluginov")
            lbl.setObjectName("PerfMuted")
            self.rows_box.addWidget(lbl)
            return
        ordered = sorted(plugins, key=lambda r: -r[1])
        for index, (name, mb, cpu) in enumerate(ordered):
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(3)
            branch = QLabel("│")
            branch.setObjectName("PerfBranch")
            branch.setFixedWidth(11)
            n = QLabel(name)
            n.setObjectName("PerfValue")
            value = f"{mb / 1024:.2f} GB" if mb >= 1024 else f"{mb:.1f} MB"
            val = QLabel(value)
            val.setObjectName("PerfValue")
            val.setFixedWidth(70)
            val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            row.addWidget(branch)
            row.addWidget(val)
            dash = QLabel("-")
            dash.setObjectName("PerfMuted")
            row.addWidget(dash)
            row.addWidget(n, 1)
            wrap = QWidget()
            wrap.setFixedHeight(23)
            wrap.setLayout(row)
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
