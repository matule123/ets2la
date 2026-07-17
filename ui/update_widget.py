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
from PyQt6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel,
                             QPushButton, QProgressBar, QDialog)
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


class UpdateConfirmDialog(QDialog):
    """A clean light ETS2LA-style confirmation dialog (replaces the drab
    native QMessageBox). Shows the new version, a short note that the app will
    restart, and green/grey Yes–No buttons."""

    def __init__(self, latest_tag, title="", description="", parent=None):
        super().__init__(parent)
        from core.update_check import _display_commit
        latest_tag = _display_commit(str(latest_tag)) or str(latest_tag)
        self.setWindowTitle("Aktualizovať UltraPilot")
        self.setModal(True)
        self.setFixedSize(470, 310)
        # Match the application's default white ETS2LA-style surfaces.
        self.setStyleSheet(
            "UpdateConfirmDialog{background:#FFFFFF;}"
            "QLabel{color:#111827;background:transparent;}")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(28, 24, 28, 20)
        lay.setSpacing(12)

        head = QHBoxLayout()
        head.setSpacing(14)
        icon = QLabel("↓")
        icon.setFixedSize(48, 48)
        icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        icon.setStyleSheet("font-size:28px;font-weight:800;color:#047857;background:#ECFDF5;border:1px solid #A7F3D0;border-radius:14px;")
        head.addWidget(icon)
        col = QVBoxLayout()
        col.setSpacing(2)
        title = QLabel("Dostupná aktualizácia")
        title.setStyleSheet("font-size:18px;font-weight:800;color:#111827;")
        col.addWidget(title)
        ver = QLabel("Commit: " + str(latest_tag))
        ver.setStyleSheet("font-size:12px;color:#047857;font-weight:700;")
        col.addWidget(ver)
        head.addLayout(col, stretch=1)
        lay.addLayout(head)

        commit_title = QLabel(title or "Aktualizácia UltraPilot")
        commit_title.setWordWrap(True)
        commit_title.setStyleSheet("font-size:14px;font-weight:800;color:#111827;")
        lay.addWidget(commit_title)
        note_text = description or "Táto verzia obsahuje najnovšie opravy a vylepšenia."
        note = QLabel(note_text + "\n\nAplikácia sa po dokončení reštartuje. "
                      "Nastavenia, trasy a mapy zostanú zachované.")
        note.setWordWrap(True)
        note.setStyleSheet("font-size:13px;color:#4B5563;background:#F9FAFB;border:1px solid #E5E7EB;border-radius:10px;padding:12px;")
        lay.addWidget(note)
        lay.addStretch()

        row = QHBoxLayout()
        row.addStretch()
        no = QPushButton("Zrušiť")
        no.setCursor(Qt.CursorShape.PointingHandCursor)
        no.setFixedWidth(110)
        no.setStyleSheet(
            "QPushButton{background:#FFFFFF;color:#374151;border:1px solid #D1D5DB;"
            "border-radius:8px;padding:9px;font-weight:600;}"
            "QPushButton:hover{background:#F9FAFB;border-color:#9CA3AF;}")
        no.clicked.connect(self.reject)
        row.addWidget(no)
        yes = QPushButton("Aktualizovať")
        yes.setCursor(Qt.CursorShape.PointingHandCursor)
        yes.setFixedWidth(130)
        yes.setStyleSheet(
            "QPushButton{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            "stop:0 #10B981, stop:1 #059669);color:#FFFFFF;border:none;"
            "border-radius:8px;padding:9px;font-weight:700;}"
            "QPushButton:hover{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            "stop:0 #34D399, stop:1 #059669);}")
        yes.clicked.connect(self.accept)
        yes.setDefault(True)
        row.addWidget(yes)
        lay.addLayout(row)


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
        # Vertical layout so each element gets its own row and nothing is
        # squeezed by the 210px sidebar (button full-width, progress below).
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 2, 0, 4)
        lay.setSpacing(4)
        # Version + spinner row.
        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        top.setSpacing(6)
        self.version_lbl = QLabel(self._version_text())
        self.version_lbl.setStyleSheet("font-size:11px;font-weight:700;color:#6B7280;border:none;")
        self.version_lbl.setWordWrap(True)
        top.addWidget(self.version_lbl)
        # A separate status line (check result / progress) so the version is
        # ALWAYS visible above it and never overwritten.
        top.addStretch()
        self.spinner = Spinner(size=14)
        self.spinner.hide()
        top.addWidget(self.spinner)
        lay.addLayout(top)
        self.status_lbl = QLabel("")
        self.status_lbl.setStyleSheet("font-size:10px;color:#6B7280;border:none;")
        self.status_lbl.setWordWrap(True)
        lay.addWidget(self.status_lbl)
        # Button (full width, short label so it fits the narrow sidebar).
        self.btn = QPushButton("Aktualizácia")
        self.btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._apply_btn_style()
        self.btn.clicked.connect(self.check)
        lay.addWidget(self.btn)
        # Progress bar on its own row below the button.
        self.progress = QProgressBar()
        self.progress.setFixedHeight(8)
        self.progress.setVisible(False)
        lay.addWidget(self.progress)

    def _apply_btn_style(self, update_available=False):
        """Neutral 'check' look, or green 'update' look when one is available."""
        if update_available:
            self.btn.setStyleSheet(
                "QPushButton{background:" + ACCENT + ";color:#FFFFFF;border:none;"
                "border-radius:6px;padding:4px 10px;font-size:11px;font-weight:700;}"
                "QPushButton:hover{background:#059669;}")
        else:
            self.btn.setStyleSheet(
                "QPushButton{background:#FFFFFF;color:#4B5563;border:1px solid #D1D5DB;"
                "border-radius:6px;padding:4px 10px;font-size:11px;font-weight:600;}"
                "QPushButton:hover{background:#F0FDF4;color:" + ACCENT + ";border-color:" + ACCENT + ";}")

    def _version_text(self):
        t = "v" + self._version
        if self._commit:
            # The badge contains only the short SHA; never append build counts,
            # line numbers or other metadata from an older commit file.
            from core.update_check import _display_commit
            commit = _display_commit(self._commit)
            t += "  ·  " + (commit or "build")
        return t

    def check(self):
        if self._check_worker is not None and self._check_worker.isRunning():
            return
        self.btn.hide()
        self.spinner.show()
        self.status_lbl.setText("Kontrolujem aktualizácie…")
        self._check_worker = _CheckWorker()
        self._check_worker.done.connect(self._on_checked)
        self._check_worker.start()

    def _on_checked(self, available, latest):
        self.spinner.hide()
        self.btn.show()
        if latest is None:
            self.status_lbl.setText("Kontrola zlyhala")
            self.btn.setText("Skúsiť znova")
            self._apply_btn_style(update_available=False)
            try:
                self.btn.clicked.disconnect()
            except Exception:
                pass
            self.btn.clicked.connect(self.check)
            return
        if available and latest:
            from core.update_check import _display_commit
            latest = _display_commit(str(latest)) or str(latest)
            from core.update_check import latest_commit_info
            info = latest_commit_info()
            self._latest_title = info.get("title", "")
            self._latest_description = info.get("description", "")
            # Remember the tag/SHA so the confirm dialog can show it.
            self._latest_tag = str(latest)
            summary = self._latest_title or "Nová verzia UltraPilot"
            self.status_lbl.setText("Commit " + str(latest) + "\n" + summary)
            self.btn.setText("Stiahnuť " + str(latest))
            self._apply_btn_style(update_available=True)
            try:
                self.btn.clicked.disconnect()
            except Exception:
                pass
            self.btn.clicked.connect(self._confirm_update)
        else:
            self.status_lbl.setText("aktuálna")
            self.btn.setText("Aktualizácia")
            self._apply_btn_style(update_available=False)
            try:
                self.btn.clicked.disconnect()
            except Exception:
                pass
            self.btn.clicked.connect(self.check)

    def _confirm_update(self):
        latest = getattr(self, "_latest_tag", None) or ""
        dlg = UpdateConfirmDialog(
            latest,
            title=getattr(self, "_latest_title", ""),
            description=getattr(self, "_latest_description", ""),
            parent=self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        self._do_update()

    def _do_update(self):
        if self._update_worker is not None and self._update_worker.isRunning():
            return
        self.btn.hide()
        self.spinner.show()
        self.status_lbl.setText("Aktualizujem…")
        self.progress.setVisible(True)
        self.progress.setRange(0, 100)
        self._update_worker = _UpdateWorker()
        self._update_worker.progress.connect(self._on_progress)
        self._update_worker.done.connect(self._on_updated)
        self._update_worker.start()

    def _on_progress(self, fraction, text):
        self.progress.setValue(int(fraction * 100))
        self.status_lbl.setText("Aktualizujem… " + text)

    def _on_updated(self, ok):
        self.spinner.hide()
        self.progress.setVisible(False)
        if ok:
            self.status_lbl.setText("✔ Aktualizované — reštartujem…")
            QTimer.singleShot(800, self._restart)
        else:
            self.status_lbl.setText("Aktualizácia zlyhala — skús znova.")
            self.btn.show()
            self.btn.setText("Aktualizácia")
            try:
                self.btn.clicked.disconnect()
            except Exception:
                pass
            self.btn.clicked.connect(self.check)

    def _restart(self):
        """Re-launch the app (bootloader / main.py) and exit this process.

        In a frozen (PyInstaller) build ``sys.executable`` IS the app exe and
        must be launched without a script argument — the bootloader ignores
        extra args and would just re-run the old bundled code. From source we
        launch ``python main.py``."""
        try:
            from PyQt6.QtCore import QProcess
            if getattr(sys, "frozen", False):
                # Frozen: re-launch the exe itself (updated files are on disk).
                QProcess.startDetached(sys.executable, [])
            else:
                # Source: run python with main.py from the project base.
                main_py = os.path.join(_app_base(), "main.py")
                QProcess.startDetached(sys.executable, [main_py])
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
