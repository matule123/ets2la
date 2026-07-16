"""Compact live activity banner fed by UltraPilot's multi-process log."""

import os
import re

from PyQt6.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, QPoint, QEvent
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QFrame


_LEVEL_COLOR = {
    "DEBUG": "#8B949E", "INFO": "#2EA043", "WARNING": "#D29922",
    "ERROR": "#F85149", "CRITICAL": "#F85149",
}
_LOG_RE = re.compile(
    r"^(?P<date>\d{4}-\d{2}-\d{2}) (?P<time>\d{2}:\d{2}:\d{2}),\d+\s+"
    r"(?P<level>DEBUG|INFO|WARNING|ERROR|CRITICAL)\s+(?P<src>\S+)\s+(?P<msg>.*)$"
)


class DynamicIsland(QWidget):
    """Top-centred pill that appears only while the app is doing something."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setFixedHeight(40)
        self._visible = False
        self._animation = None
        self._log_file = None
        self._log_pos = 0
        self._build()
        self.hide()

        self._hide_timer = QTimer(self)
        self._hide_timer.setSingleShot(True)
        self._hide_timer.timeout.connect(self._slide_out)
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll_log)
        self._poll_timer.start(250)

    def _build(self):
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        self.frame = QFrame()
        self.frame.setObjectName("DynamicIsland")
        self.frame.setStyleSheet(
            "#DynamicIsland{background:rgba(18,22,29,0.96);"
            "border:1px solid #3B4554;border-radius:16px;}"
        )
        row = QHBoxLayout(self.frame)
        row.setContentsMargins(14, 6, 14, 6)
        row.setSpacing(9)
        self.time_lbl = QLabel()
        self.time_lbl.setStyleSheet("color:#8B949E;font-size:11px;font-weight:600;border:none;")
        self.msg_lbl = QLabel()
        self.msg_lbl.setStyleSheet("color:#2EA043;font-size:12px;font-weight:700;border:none;")
        self.src_lbl = QLabel()
        self.src_lbl.setStyleSheet("color:#687384;font-size:10px;border:none;")
        row.addWidget(self.time_lbl)
        row.addWidget(self.msg_lbl, 1)
        row.addWidget(self.src_lbl, 0, Qt.AlignmentFlag.AlignRight)
        outer.addWidget(self.frame)

    @staticmethod
    def install(parent_window):
        island = DynamicIsland(parent_window)
        from core.paths import app_dir
        island._log_file = os.path.join(app_dir(), "ultrapilot.log")
        try:
            island._log_pos = os.path.getsize(island._log_file)
        except OSError:
            pass
        parent_window.installEventFilter(island)
        return island

    def _poll_log(self):
        """Read new records written by the engine, UI, HUD and all plugins."""
        if not self._log_file:
            return
        try:
            size = os.path.getsize(self._log_file)
            if size < self._log_pos:
                self._log_pos = 0
            if size == self._log_pos:
                return
            with open(self._log_file, "r", encoding="utf-8", errors="replace") as stream:
                stream.seek(self._log_pos)
                lines = stream.readlines()
                self._log_pos = stream.tell()
        except (OSError, ValueError):
            return

        # A burst can contain many progress messages. Showing the newest one
        # keeps the banner calm while still surfacing warnings and errors.
        for line in reversed(lines):
            match = _LOG_RE.match(line.rstrip())
            if not match:
                continue
            item = match.groupdict()
            if item["msg"].startswith("Logging to ") or "OpenGL_accelerate" in item["msg"]:
                continue
            self.show_record(item["msg"], item["level"], item["time"], item["src"])
            break

    def show_record(self, msg, level, ts, src):
        color = _LEVEL_COLOR.get(level, "#8B949E")
        short = msg if len(msg) <= 96 else msg[:93] + "..."
        self.time_lbl.setText(ts)
        self.msg_lbl.setText(short)
        self.msg_lbl.setStyleSheet(
            f"color:{color};font-size:12px;font-weight:700;border:none;")
        self.src_lbl.setText((src or "UltraPilot").rsplit(".", 1)[-1])
        self._slide_in()
        self._hide_timer.start(4500 if level in ("WARNING", "ERROR", "CRITICAL") else 3000)

    def _anchor(self):
        parent = self.parentWidget()
        if not parent:
            return
        self.adjustSize()
        width = min(max(310, self.frame.sizeHint().width() + 4), max(310, parent.width() - 28))
        self.setFixedWidth(width)
        self.move(parent.width() // 2 - width // 2, 10)

    def _animate(self, start, end, duration, easing, finished=None):
        if self._animation:
            self._animation.stop()
        self._animation = QPropertyAnimation(self, b"pos", self)
        self._animation.setStartValue(start)
        self._animation.setEndValue(end)
        self._animation.setDuration(duration)
        self._animation.setEasingCurve(easing)
        if finished:
            self._animation.finished.connect(finished)
        self._animation.start()

    def _slide_in(self):
        self._anchor()
        end = self.pos()
        if self._visible:
            self.raise_()
            return
        self._visible = True
        start = QPoint(end.x(), -self.height())
        self.move(start)
        self.show()
        self.raise_()
        self._animate(start, end, 180, QEasingCurve.Type.OutCubic)

    def _slide_out(self):
        if not self._visible:
            return
        self._visible = False
        start = self.pos()
        self._animate(start, QPoint(start.x(), -self.height()), 170,
                      QEasingCurve.Type.InCubic, self.hide)

    def reposition(self):
        if self._visible:
            self._anchor()
            self.raise_()

    def eventFilter(self, watched, event):
        if watched is self.parentWidget() and event.type() in (
                QEvent.Type.Resize, QEvent.Type.Show, QEvent.Type.WindowStateChange):
            QTimer.singleShot(0, self.reposition)
        return super().eventFilter(watched, event)
