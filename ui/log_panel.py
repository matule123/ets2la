"""Live log panel — shows the app's own log output in real time.

A dark, console-style panel (timestamp + coloured level tag + message) that
mirrors what gets written to ``ultrapilot.log``. A custom ``logging.Handler``
feeds every record into a ring buffer (capped so a long session can't grow the
document unbounded) and renders it with colour coding:

    INFO    → grey text
    WARNING → amber
    ERROR   → red
    DEBUG   → dim
    (plugin/section prefixes like ``map:``, ``autopilot:`` keep their colour)

It is added as a top-level page („Log“) in the main window's sidebar.
"""

import logging
from datetime import datetime

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QTextCharFormat, QTextCursor, QFont, QColor
from PyQt6.QtWidgets import QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt6.QtWidgets import QTextEdit

from ui.app import Page


# Pre-formatted HTML colours per level.
_LEVEL_COLOR = {
    logging.DEBUG:    "#5B6573",
    logging.INFO:     "#9AA4B2",
    logging.WARNING:  "#F59E0B",
    logging.ERROR:    "#EF4444",
    logging.CRITICAL: "#EF4444",
}
_LEVEL_TAG = {
    logging.DEBUG:    "DEBUG",
    logging.INFO:     "INFO ",
    logging.WARNING:  "WARN ",
    logging.ERROR:    "ERROR",
    logging.CRITICAL: "CRIT ",
}


class _BufferHandler(logging.Handler):
    """A logging handler that appends formatted HTML lines to a QTextEdit.

    Created once and attached to the root logger; lives for the whole session.
    Keeps at most ``max_blocks`` lines so memory stays bounded."""

    def __init__(self, target, max_blocks=2000):
        super().__init__()
        self._target = target
        self._max = max_blocks

    def emit(self, record):
        try:
            from core.theme import palette
            pal = palette("dark")  # the log panel is always dark for contrast
            ts = datetime.fromtimestamp(record.created).strftime("%H:%M:%S")
            lvl = record.levelno
            col = _LEVEL_COLOR.get(lvl, "#9AA4B2")
            tag = _LEVEL_TAG.get(lvl, "LOG  ")
            # Escape the message so log text can't inject HTML.
            msg = (record.getMessage() or "").replace("&", "&amp;")\
                                             .replace("<", "&lt;")\
                                             .replace(">", "&gt;")
            line = (
                f'<span style="color:#5B6573;">[{ts}]</span> '
                f'<span style="color:{col};font-weight:700;">{tag}</span> '
                f'<span style="color:{pal["text"]};">{msg}</span>'
            )
            cursor = self._target.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            cursor.insertHtml(line + "<br>")
            # Trim old blocks if the document grew too large.
            doc = self._target.document()
            if doc.blockCount() > self._max:
                cursor.movePosition(QTextCursor.MoveOperation.Start)
                cursor.movePosition(QTextCursor.MoveOperation.Down,
                                    QTextCursor.MoveMode.KeepAnchor,
                                    doc.blockCount() - self._max)
                cursor.removeSelectedText()
            self._target.setTextCursor(cursor)
            self._target.ensureCursorVisible()
        except Exception:
            # A logging handler must never raise.
            pass


class LogPage(Page):
    """Sidebar page: a live, colour-coded console of the app's log output."""

    # Class-level so we attach the handler exactly once even if the page is
    # rebuilt on theme switch.
    _handler = None

    def __init__(self, state):
        super().__init__(state)
        title = QLabel("📋 Žurnál (Log)")
        title.setStyleSheet("font-size: 22px; font-weight: 800; color: #34D399; margin-bottom: 8px;")
        self.layout.addWidget(title)

        sub = QLabel("Živý výpis logov aplikácie — rovnaké ako v ultrapilot.log.")
        sub.setStyleSheet("color: #9AA4B2; font-size: 13px; margin-bottom: 12px;")
        sub.setWordWrap(True)
        self.layout.addWidget(sub)

        # Toolbar: clear + auto-scroll toggle + level filter (cosmetic).
        bar = QHBoxLayout()
        bar.setSpacing(8)
        self.clear_btn = QPushButton("🗑 Vyčistiť")
        self.clear_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.clear_btn.clicked.connect(self._clear)
        bar.addWidget(self.clear_btn)
        self.scroll_btn = QPushButton("⬇ Auto-scroll: ZAP")
        self.scroll_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.scroll_btn.setCheckable(True)
        self.scroll_btn.setChecked(True)
        self.scroll_btn.clicked.connect(self._toggle_scroll)
        bar.addWidget(self.scroll_btn)
        bar.addStretch()
        wrap = self._row_wrap(bar)
        self.layout.addWidget(wrap)

        # The console itself — always dark for readability regardless of theme.
        self.view = QTextEdit()
        self.view.setReadOnly(True)
        self.view.setFont(QFont("Consolas", 9))
        self.view.document().setMaximumBlockCount(2000)
        self.view.setStyleSheet(
            "QTextEdit{background:#0F1419; color:#E6E8EB; border:1px solid #30363D;"
            " border-radius:10px; padding:8px;}")
        self.layout.addWidget(self.view, stretch=1)

        # Attach the handler once.
        if LogPage._handler is None:
            LogPage._handler = _BufferHandler(self.view)
            LogPage._handler.setLevel(logging.INFO)
            logging.getLogger().addHandler(LogPage._handler)
        else:
            # Re-target the existing handler at this fresh widget.
            LogPage._handler._target = self.view
        self._auto_scroll = True

    def _row_wrap(self, layout):
        from PyQt6.QtWidgets import QWidget
        w = QWidget()
        w.setObjectName("Panel")
        w.setStyleSheet("background: transparent;")
        w.setLayout(layout)
        return w

    def _clear(self):
        self.view.clear()

    def _toggle_scroll(self):
        self._auto_scroll = self.scroll_btn.isChecked()
        self.scroll_btn.setText("⬇ Auto-scroll: ZAP" if self._auto_scroll
                                else "⬇ Auto-scroll: VYP")
