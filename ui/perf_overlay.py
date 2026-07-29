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
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setFixedSize(420, 430)
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
            "PerfOverlay{background:transparent;}"
            "QFrame#PerfSurface{background:" + p['card']
            + ";border:1px solid " + p['border'] + ";border-radius:18px;}"
            "QFrame#PerfMetric{background:" + p['card2']
            + ";border:1px solid " + p['border'] + ";border-radius:11px;}"
            "QLabel{background:transparent;border:none;}"
            "QLabel#PerfTitle{font-size:17px;font-weight:800;color:"
            + p['text'] + ";}"
            "QLabel#PerfSubtitle{font-size:11px;color:" + p['muted'] + ";}"
            "QPushButton#PerfClose{background:" + p['card2']
            + ";border:1px solid " + p['border']
            + ";border-radius:13px;color:" + p['muted']
            + ";font-size:17px;font-weight:700;padding:0;}"
            "QPushButton#PerfClose:hover{background:#FEE2E2;"
            "border-color:#FCA5A5;color:#B91C1C;}"
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
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(0)
        self.surface = QFrame()
        self.surface.setObjectName("PerfSurface")
        shadow = QGraphicsDropShadowEffect(self.surface)
        shadow.setBlurRadius(28)
        shadow.setOffset(0, 7)
        shadow.setColor(QColor(0, 0, 0, 70))
        self.surface.setGraphicsEffect(shadow)
        root.addWidget(self.surface)
        body = QVBoxLayout(self.surface)
        body.setContentsMargins(18, 16, 18, 18)
        body.setSpacing(11)
        head = QHBoxLayout()
        head.setSpacing(10)
        from ui.icons import line_icon
        icon = QLabel()
        icon.setFixedSize(36, 36)
        icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        icon.setPixmap(line_icon("performance", "#047857", 24).pixmap(24, 24))
        icon.setStyleSheet("background:#D1FAE5;border:1px solid #A7F3D0;"
                           "border-radius:10px;")
        head.addWidget(icon)
        title_column = QVBoxLayout()
        title_column.setSpacing(0)
        title = QLabel("Výkon aplikácie")
        title.setObjectName("PerfTitle")
        subtitle = QLabel("RAM, CPU a aktívne pluginy")
        subtitle.setObjectName("PerfSubtitle")
        title_column.addWidget(title)
        title_column.addWidget(subtitle)
        head.addLayout(title_column)
        head.addStretch()
        close = QPushButton("×")
        close.setObjectName("PerfClose")
        close.setToolTip("Zavrieť okno výkonu")
        close.setFixedSize(27, 27)
        close.setCursor(Qt.CursorShape.PointingHandCursor)
        close.clicked.connect(self.hide)
        head.addWidget(close)
        body.addLayout(head)

        summary_lay = QHBoxLayout()
        summary_lay.setSpacing(8)

        def metric(text):
            frame = QFrame()
            frame.setObjectName("PerfMetric")
            frame.setMinimumHeight(62)
            lay = QVBoxLayout(frame)
            lay.setContentsMargins(10, 8, 10, 8)
            label = QLabel(text)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet("font-size:11px;font-weight:750;color:"
                                + self._pal['text'] + ";")
            lay.addWidget(label)
            summary_lay.addWidget(frame, 1)
            return label

        self.total_lbl = metric("RAM\n— MB")
        self.cpu_lbl = metric("CPU\n— %")
        self.plugin_count_lbl = metric("PLUGINY\n—")
        body.addLayout(summary_lay)

        memory_label = QLabel("Pamäť aplikácie")
        memory_label.setObjectName("PerfSubtitle")
        body.addWidget(memory_label)
        self.total_bar = QProgressBar()
        self.total_bar.setFixedHeight(8)
        self.total_bar.setRange(0, 100)
        self.total_bar.setTextVisible(False)
        self._style_total_bar()
        body.addWidget(self.total_bar)

        hint = QLabel("PROCESY A PLUGINY")
        hint.setStyleSheet("font-size:10px;font-weight:700;letter-spacing:1px;color:" + self._pal['muted'] + ";")
        body.addWidget(hint)
        self.rows_box = QVBoxLayout()
        self.rows_box.setSpacing(5)
        body.addLayout(self.rows_box)
        body.addStretch()

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
        desired = max(350, min(620, 270 + (len(plugins) + 1) * 34))
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
        self.plugin_count_lbl.setText(f"PLUGINY\n{len(plugins)}")
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
        root_wrap.setStyleSheet(
            "background:" + self._pal['card2'] + ";border:1px solid "
            + self._pal['border'] + ";border-radius:8px;")
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
            wrap.setStyleSheet("background:" + self._pal['card2']
                               + ";border:1px solid " + self._pal['border']
                               + ";border-radius:8px;")
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
