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


def _bar_color(frac, pal):
    """Colour a plugin bar by its share: green (small) → amber → red (heavy)."""
    if frac > 0.5:
        return pal['danger']
    if frac > 0.25:
        return pal['warn']
    return pal['success']


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
        self.setFixedSize(390, 460)
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
        """Re-apply colours when the theme changes."""
        from core.theme import palette
        self._pal = palette(theme)
        p = self._pal
        self._apply_window_style()
        self._style_total_bar()
        # Rebuild the labels so the new palette's text colours apply.
        # Simplest reliable path: clear and re-add via a fresh _build-like refresh.
        self.refresh()

    def _apply_window_style(self):
        p = self._pal
        self.setStyleSheet(
            "PerfOverlay{background:" + p['card'] + ";border:1px solid " + p['border'] + ";border-radius:16px;}"
            "QLabel{background:transparent;border:none;}"
            "QWidget{font-family:'Segoe UI';}")

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
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(10)
        head = QHBoxLayout()
        head.setSpacing(6)
        title = QLabel("Performance")
        title.setStyleSheet("font-size:17px;font-weight:800;color:" + self._pal['title'] + ";")
        head.addWidget(title)
        head.addStretch()
        close = QPushButton("×")
        close.setToolTip("Zavrieť Performance")
        close.setFixedSize(28, 28)
        close.setCursor(Qt.CursorShape.PointingHandCursor)
        close.setStyleSheet("QPushButton{background:#FEE2E2;border:1px solid #FCA5A5;"
                            "border-radius:14px;color:#B91C1C;font-size:18px;font-weight:800;}"
                            "QPushButton:hover{background:#EF4444;color:#FFFFFF;}")
        close.clicked.connect(self.hide)
        head.addWidget(close)
        root.addLayout(head)

        summary = QFrame()
        summary.setStyleSheet("QFrame{background:" + self._pal['field'] + ";border:1px solid " + self._pal['border'] + ";border-radius:12px;}")
        summary_lay = QHBoxLayout(summary)
        summary_lay.setContentsMargins(13, 10, 13, 10)
        self.total_lbl = QLabel("RAM\n— MB")
        self.total_lbl.setStyleSheet("font-size:12px;font-weight:700;color:" + self._pal['text'] + ";")
        self.cpu_lbl = QLabel("CPU\n— %")
        self.cpu_lbl.setStyleSheet("font-size:12px;font-weight:700;color:" + self._pal['text'] + ";")
        summary_lay.addWidget(self.total_lbl)
        summary_lay.addStretch()
        summary_lay.addWidget(self.cpu_lbl)
        root.addWidget(summary)

        bar_wrap = QFrame()
        bar_wrap.setStyleSheet("background: transparent; border: none;")
        bw = QVBoxLayout(bar_wrap)
        bw.setContentsMargins(0, 0, 0, 0)
        bw.setSpacing(2)
        self.total_bar = QProgressBar()
        self.total_bar.setFixedHeight(8)
        self.total_bar.setRange(0, 100)
        self.total_bar.setTextVisible(False)
        self._style_total_bar()
        bw.addWidget(self.total_bar)
        root.addWidget(bar_wrap)

        hint = QLabel("PROCESY A PLUGINY")
        hint.setStyleSheet("font-size:10px;font-weight:700;letter-spacing:1px;color:" + self._pal['muted'] + ";")
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

    def _style_total_bar(self):
        """Style the total-RAM bar's chunk so it matches the themed plugin bars
        (was the native platform blue because no QSS was set on it)."""
        p = self._pal
        self.total_bar.setStyleSheet(
            "QProgressBar{background:" + p['field'] + "; border:none; border-radius:4px;}"
            "QProgressBar::chunk{border-radius:4px;"
            "background: qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            "stop:0 " + p['title'] + ", stop:1 " + p['accent2'] + ");}")

    def refresh(self):
        app_mb, app_cpu, plugins = _collect(self.state)
        # Grow with the real plugin count so the final rows are not clipped.
        # Keep the bottom edge anchored above the sidebar button.
        desired = max(390, min(650, 245 + (len(plugins) + 1) * 29))
        if desired != self.height():
            old_bottom = self.y() + self.height()
            self.setFixedHeight(desired)
            screen = self.screen().availableGeometry() if self.screen() else None
            y = old_bottom - desired
            if screen is not None:
                y = max(screen.top() + 8, min(y, screen.bottom() - desired - 8))
            self.move(self.x(), y)
        self.total_lbl.setText(f"RAM\n{app_mb:.0f} MB")
        self.cpu_lbl.setText(f"CPU\n{app_cpu:.0f} %")
        # The total bar is relative to a 1 GB soft cap for a quick visual feel.
        self.total_bar.setValue(min(100, int(app_mb / 1024 * 100)))
        self._clear_rows()
        plug_total = sum(r for _, r, _ in plugins) or 1.0
        root_row = QHBoxLayout()
        root_icon = QLabel("●")
        root_icon.setFixedWidth(24)
        root_icon.setStyleSheet("color:" + self._pal['title'] + ";font-size:13px;")
        root_name = QLabel("UltraPilot")
        root_name.setStyleSheet("font-size:12px;font-weight:800;color:" + self._pal['text'] + ";")
        root_ram = QLabel(f"{app_mb:.0f} MB")
        root_ram.setStyleSheet("font-size:11px;font-weight:700;color:" + self._pal['muted'] + ";")
        root_row.addWidget(root_icon); root_row.addWidget(root_name); root_row.addStretch(); root_row.addWidget(root_ram)
        root_wrap = QWidget(); root_wrap.setLayout(root_row)
        root_wrap.setStyleSheet("background:transparent;border:none;")
        self.rows_box.addWidget(root_wrap)
        if not plugins:
            lbl = QLabel("žiadne pluginy")
            lbl.setStyleSheet("font-size: 11px; color: " + self._pal['muted'] + ";")
            self.rows_box.addWidget(lbl)
            return
        ordered = sorted(plugins, key=lambda r: -r[1])
        for index, (name, mb, cpu) in enumerate(ordered):
            frac = mb / plug_total
            row = QHBoxLayout()
            row.setSpacing(6)
            branch = QLabel("└─" if index == len(ordered) - 1 else "├─")
            branch.setFixedWidth(24)
            branch.setStyleSheet("font-family:Consolas;font-size:14px;color:" + self._pal['muted'] + ";")
            n = QLabel(name)
            n.setFixedWidth(105)
            n.setStyleSheet("font-size: 11px; color: " + self._pal['text'] + ";")
            bar = QProgressBar()
            bar.setFixedHeight(7)
            bar.setRange(0, 100)
            bar.setValue(int(100 * frac))
            bar.setTextVisible(False)
            # Colour the chunk by the plugin's memory share.
            col = _bar_color(frac, self._pal)
            bar.setStyleSheet(
                "QProgressBar{background:" + self._pal['field'] + "; border:none; border-radius:3px;}"
                "QProgressBar::chunk{background:" + col + "; border-radius:3px;}")
            val = QLabel(f"{mb:.1f} MB")
            val.setFixedWidth(65)
            val.setStyleSheet("font-size: 11px; color: " + self._pal['muted'] + ";")
            row.addWidget(branch)
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
