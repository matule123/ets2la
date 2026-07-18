"""Compact live activity banner fed by UltraPilot's multi-process log."""

import os
import re

from PyQt6.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, QPoint, QEvent
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QFrame, QProgressBar


_LEVEL_COLOR = {
    "DEBUG": "#8B949E", "INFO": "#2EA043", "WARNING": "#D29922",
    "ERROR": "#F85149", "CRITICAL": "#F85149",
}
_LOG_RE = re.compile(
    r"^(?P<date>\d{4}-\d{2}-\d{2}) (?P<time>\d{2}:\d{2}:\d{2}),\d+\s+"
    r"(?P<level>DEBUG|INFO|WARNING|ERROR|CRITICAL)\s+(?P<src>\S+)\s+(?P<msg>.*)$"
)
_ACTIVITY = (
    "loading", "loaded", "download", "unpack", "extract", "initializ",
    "starting", "started", "ready", "map", "dataset", "road network",
    "plugin", "connected", "install", "repair", "update", "error", "failed",
    "route", "navigation", "gps", "calculation",
)
_TECHNICAL_LOG_MARKERS = (
    "map: truck=", "nearest_seg=", "truckposition", "truck position",
    "truck_world_pos", "coordinatex", "coordinatez", "heading=",
    "system transitioning", "follow_lane", "follow lane", "cruise ->",
    "camera: revision=", "rendertime=", "viewport=", "hfov=",
    "active_lane_id", "lane_match", "locator_score", "trajectory_score",
)


def _friendly_activity_message(message):
    """Return concise user-facing activity text, or ``None`` for diagnostics."""
    low = message.lower()
    if any(marker in low for marker in _TECHNICAL_LOG_MARKERS):
        return None
    # Reject coordinate dumps even when their exact prefix changes.
    if (re.search(r"\b[xyz]=[-+]?\d", low)
            or re.search(r"\b(position|truck|camera)\s*[=:].*[-+]?\d", low)
            or "failure_reason=" in low):
        return None
    friendly = (
        ("new in-game destination detected", "Načítavam nový cieľ z hernej navigácie"),
        ("in-game destination cleared", "Cieľ navigácie bol odstránený"),
        ("road network: loaded", "Mapa ciest je pripravená"),
        ("road network loaded", "Mapa ciest je pripravená"),
        ("loading road network", "Načítavam mapu ciest"),
        ("connected to scs telemetry", "Hra bola pripojená"),
        ("camera shared memory", "Čakám na údaje z hernej kamery"),
    )
    for marker, text in friendly:
        if marker in low:
            return text
    return message


class DynamicIsland(QWidget):
    """Top-centred pill that appears only while the app is doing something."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setFixedHeight(72)
        self._visible = False
        self._animation = None
        self._log_file = None
        self._log_pos = 0
        self._navigation_was_active = False
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
            "#DynamicIsland{background:#FFFFFF;"
            "border:1px solid #D9DCE1;border-radius:15px;}"
        )
        content = QVBoxLayout(self.frame)
        content.setContentsMargins(14, 5, 14, 5)
        content.setSpacing(3)
        row = QHBoxLayout()
        row.setSpacing(9)
        self.time_lbl = QLabel()
        self.time_lbl.setStyleSheet("color:#9CA3AF;font-size:10px;font-weight:600;border:none;")
        self.msg_lbl = QLabel()
        self.msg_lbl.setWordWrap(True)
        self.msg_lbl.setMinimumWidth(310)
        self.msg_lbl.setStyleSheet("color:#2EA043;font-size:12px;font-weight:700;border:none;")
        self.src_lbl = QLabel()
        self.src_lbl.setStyleSheet("color:#9CA3AF;font-size:10px;border:none;")
        row.addWidget(self.time_lbl)
        row.addWidget(self.msg_lbl, 1)
        row.addWidget(self.src_lbl, 0, Qt.AlignmentFlag.AlignRight)
        content.addLayout(row)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(4)
        self.progress.setStyleSheet(
            "QProgressBar{background:#E5E7EB;border:none;border-radius:2px;}"
            "QProgressBar::chunk{background:#10B981;border-radius:2px;}")
        self.progress.hide()
        content.addWidget(self.progress)
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
        if self._poll_navigation():
            return
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
            message = item["msg"]
            low = message.lower()
            if (message.startswith("Logging to ") or "OpenGL_accelerate" in message
                    or not any(word in low for word in _ACTIVITY)):
                continue
            message = _friendly_activity_message(message)
            if not message:
                continue
            self.show_record(message, item["level"], item["time"], item["src"])
            break

    def _poll_navigation(self):
        parent = self.parentWidget()
        state = getattr(parent, "state", None)
        if state is None:
            return False
        active = bool(state.get("navigation_recalculating", False))
        progress = float(state.get("navigation_progress", 0.0) or 0.0)
        status = state.get("navigation_status", "") or "Prepočítavam navigáciu…"
        if active:
            self._navigation_was_active = True
            self.time_lbl.setText("NAV")
            self.msg_lbl.setText(status)
            self.msg_lbl.setStyleSheet("color:#047857;font-size:12px;font-weight:700;border:none;")
            self.src_lbl.setText(f"{int(progress * 100)}%")
            self.progress.setValue(int(progress * 100))
            self.progress.show()
            self._hide_timer.stop()
            self._slide_in()
            return True
        if self._navigation_was_active:
            self._navigation_was_active = False
            self.time_lbl.setText("NAV")
            self.msg_lbl.setText(status or "Trasa je pripravená")
            succeeded = progress >= 0.99 or "pripraven" in status.lower()
            self.src_lbl.setText("100%" if succeeded else "NEÚSPEŠNÉ")
            self.progress.setValue(100 if succeeded else max(0, int(progress * 100)))
            self.msg_lbl.setStyleSheet(
                ("color:#047857;" if succeeded else "color:#B42318;")
                + "font-size:12px;font-weight:700;border:none;")
            self.progress.show()
            self._slide_in()
            # Keep the outcome readable. Previously the calculation vanished
            # before the user could see whether it succeeded or failed.
            self._hide_timer.start(6000 if succeeded else 10000)
            return True
        return False

    def show_record(self, msg, level, ts, src):
        color = _LEVEL_COLOR.get(level, "#8B949E")
        # The wider, wrapped island can show the actual diagnostic. Truncating
        # at 62 characters previously left only "Chyba:" with no explanation.
        short = msg if len(msg) <= 180 else msg[:177] + "..."
        self.time_lbl.setText(ts)
        self.msg_lbl.setText(short)
        self.msg_lbl.setStyleSheet(
            f"color:{color};font-size:12px;font-weight:700;border:none;")
        self.src_lbl.setText((src or "UltraPilot").rsplit(".", 1)[-1])
        self.progress.hide()
        self._slide_in()
        self._hide_timer.start(4500 if level in ("WARNING", "ERROR", "CRITICAL") else 3000)

    def _anchor(self):
        parent = self.parentWidget()
        if not parent:
            return
        self.adjustSize()
        width = min(max(380, self.frame.sizeHint().width() + 4), min(720, parent.width() - 28))
        self.setFixedWidth(width)
        self.move(parent.width() // 2 - width // 2, parent.height() - self.height() - 48)

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
        start = QPoint(end.x(), self.parentWidget().height() + self.height())
        self.move(start)
        self.show()
        self.raise_()
        self._animate(start, end, 180, QEasingCurve.Type.OutCubic)

    def _slide_out(self):
        if not self._visible:
            return
        self._visible = False
        start = self.pos()
        self._animate(start, QPoint(start.x(), self.parentWidget().height() + self.height()), 170,
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
