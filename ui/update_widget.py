"""
Update checker widget for the UltraPilot sidebar + a reusable spinner.

Replaces the old pre-launch splash window: the app opens immediately and this
widget shows the current version. Pressing „Skontrolovať“ spins the ring while
it asks GitHub whether a newer release exists; if so, an „Aktualizovať“ button
appears. Confirming it runs the hybrid update (git pull → zip fallback) with a
progress bar and then restarts the app.

The spinner is a plain ring (the requested „obvod kolieska“ style): a partial
arc that rotates every frame, drawn entirely in ``paintEvent`` so it looks the
same in every theme.
"""

import logging
import os
import sys

from PyQt6.QtCore import Qt, QTimer, QRectF, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton, QProgressBar, QMessageBox
from PyQt6.QtCore import QThread, pyqtSignal


ACCENT = "#10B981"


class Spinner(QWidget):
    """A small circular spinner (rotating arc, a.k.a. obvod kolieska)."""

    def __init__(self, size=18, parent=None):
        super().__init__(parent)
        self._angle = 0
        self._size = size
        self.setFixedSize(size, size)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(60)

    def _tick(self):
        self._angle = (self._angle + 18) % 360
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        try:
            self._draw(p)
        finally:
            p.end()

    def _draw(self, p):
        w, h = self.width(), self.height()
        margin = 2
        rect = QRectF(margin, margin, w - 2 * margin, h - 2 * margin)
        # Faint full ring as the track.
        track = QPen(QColor("#9AA4B2"))
        track.setWidthF(max(1.5, w * 0.10))
        p.setPen(track)
        p.drawArc(rect, 0, 360 * 16)
        # Bright rotating arc (about 100°).
        arc = QPen(QColor(ACCENT))
        arc.setWidthF(max(1.5, w * 0.10))
        arc.setCapStyle(Qt.PenCapStyle.RoundCap)
        p.setPen(arc)
        p.drawArc(rect, int(-self._angle * 16), int(100 * 16))


class _CheckWorker(QThread):
    """Calls check_for_update off the UI thread."""
    done = pyqtSignal(bool, object)  # (available, latest_tag_or_None)

    def run(self):
        try:
            from core.update_check import check_for_update
            self.done.emit(*check_for_update())
        except Exception:
            self.done.emit(False, None)


class _UpdateWorker(QThread):
    """Runs perform_update; emits (fraction, text) progress and a final bool."""
    progress = pyqtSignal(float, str)
    done = pyqtSignal(bool)

    def run(self):
        try:
            from core.update_check import perform_update
            ok = perform_update(progress_cb=lambda f, t: self.progress.emit(f, t))
            self.done.emit(bool(ok))
        except Exception as e:
            logging.error("update failed: %s", e)
            self.progress.emit(1.0, "chyba: " + str(e))
            self.done.emit(False)


class UpdateCheckerWidget(QWidget):
    """Compact sidebar widget: version label + check/update button + spinner."""

    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        from core.update_check import VERSION, git_commit
        self._version = VERSION
        self._commit = git_commit()
        self._check_worker = None
        self._update_worker = None
        self._build()
        # Auto-check once shortly after launch (non-blocking).
        QTimer.singleShot(2500, self.check)

    def _build(self):
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 2, 0, 4)
        lay.setSpacing(6)
        self.version_lbl = QLabel(self._version_text())
        self.version_lbl.setStyleSheet("font-size: 10px; font-weight: 600; color: #9CA3AF; border:none;")
        lay.addWidget(self.version_lbl)
        lay.addStretch()
        self.spinner = Spinner(size=14)
        self.spinner.hide()
        lay.addWidget(self.spinner)
        self.btn = QPushButton("Skontrolovať")
        self.btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn.setStyleSheet(
            "QPushButton{background:transparent;color:#9CA3AF;border:1px solid #3D4654;"
            "border-radius:6px;padding:2px 8px;font-size:10px;font-weight:600;border:none;}"
            "QPushButton:hover{color:" + ACCENT + ";}")
        self.btn.clicked.connect(self.check)
        lay.addWidget(self.btn)
        self.progress = QProgressBar()
        self.progress.setFixedHeight(8)
        self.progress.setVisible(False)
        lay.addWidget(self.progress)

    def _version_text(self):
        t = "v" + self._version
        if self._commit:
            t += "  ·  " + self._commit
        return t

    def check(self):
        if self._check_worker is not None and self._check_worker.isRunning():
            return
        self.btn.hide()
        self.spinner.show()
        self.version_lbl.setText("Kontrolujem aktualizácie…")
        self._check_worker = _CheckWorker()
        self._check_worker.done.connect(self._on_checked)
        self._check_worker.start()

    def _on_checked(self, available, latest):
        self.spinner.hide()
        self.btn.show()
        if available and latest:
            self.version_lbl.setText("Dostupná v" + str(latest))
            self.btn.setText("Aktualizovať")
            try:
                self.btn.clicked.disconnect()
            except Exception:
                pass
            self.btn.clicked.connect(self._confirm_update)
        else:
            self.version_lbl.setText(self._version_text() + "  ·  aktuálna")
            self.btn.setText("Skontrolovať")
            try:
                self.btn.clicked.disconnect()
            except Exception:
                pass
            self.btn.clicked.connect(self.check)

    def _confirm_update(self):
        ret = QMessageBox.question(
            self, "Aktualizovať UltraPilot",
            "Naozaj aktualizovať? Aplikácia sa po dokončení reštartuje.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if ret != QMessageBox.StandardButton.Yes:
            return
        self._do_update()

    def _do_update(self):
        if self._update_worker is not None and self._update_worker.isRunning():
            return
        self.btn.hide()
        self.spinner.show()
        self.version_lbl.setText("Aktualizujem…")
        self.progress.setVisible(True)
        self.progress.setRange(0, 100)
        self._update_worker = _UpdateWorker()
        self._update_worker.progress.connect(self._on_progress)
        self._update_worker.done.connect(self._on_updated)
        self._update_worker.start()

    def _on_progress(self, fraction, text):
        self.progress.setValue(int(fraction * 100))
        self.version_lbl.setText("Aktualizujem… " + text)

    def _on_updated(self, ok):
        self.spinner.hide()
        self.progress.setVisible(False)
        if ok:
            self.version_lbl.setText("✔ Aktualizované — reštartujem…")
            QTimer.singleShot(800, self._restart)
        else:
            self.version_lbl.setText("Aktualizácia zlyhala — skús znova.")
            self.btn.show()
            self.btn.setText("Skontrolovať")
            try:
                self.btn.clicked.disconnect()
            except Exception:
                pass
            self.btn.clicked.connect(self.check)

    def _restart(self):
        """Re-launch the app (bootloader / main.py) and exit this process."""
        try:
            from PyQt6.QtCore import QProcess
            QProcess.startDetached(sys.executable, [os.path.join(_app_base(), "main.py")])
        except Exception as e:
            logging.error("restart failed: %s", e)
        QApplication_exit()


def _app_base():
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def QApplication_exit():
    """Quit the whole application (alias to keep the import local)."""
    from PyQt6.QtWidgets import QApplication
    QApplication.quit()
