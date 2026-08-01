"""
Update checker widget for the UltraPilot sidebar + a reusable spinner.

Replaces the old pre-launch splash window: the app opens immediately and this
widget shows the current version. Pressing „Skontrolovať“ spins the ring while
it asks GitHub whether a newer release exists; if so, an „Aktualizovať“ button
appears. Its dialog downloads and verifies the package first, displays byte
progress, and applies it only after „Inštalovať a reštartovať“ is confirmed.

The spinner is a plain ring (the requested „obvod kolieska“ style): a partial
arc that rotates every frame, drawn entirely in ``paintEvent`` so it looks the
same in every theme.
"""

import logging
import os
import sys

from PyQt6.QtCore import Qt, QTimer, QRectF, QPointF, QPoint, QElapsedTimer
from PyQt6.QtGui import QPainter, QPen, QColor, QIcon
from PyQt6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel,
                             QPushButton, QProgressBar, QDialog, QFrame)
from PyQt6.QtCore import QThread, pyqtSignal


ACCENT = "#10B981"


class Spinner(QWidget):
    """A small circular spinner (rotating arc, a.k.a. obvod kolieska)."""

    def __init__(self, size=18, parent=None, *, arc_color=ACCENT,
                 track_color="#9AA4B2"):
        super().__init__(parent)
        self._angle = 0
        self._size = size
        self._arc_color = str(arc_color)
        self._track_color = str(track_color)
        self.setFixedSize(size, size)
        self._clock = QElapsedTimer()
        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)

    def _tick(self):
        # Derive the angle from elapsed time so a busy UI cannot make the
        # animation appear frozen or permanently slow after delayed frames.
        self._angle = (self._clock.elapsed() * 0.30) % 360
        self.update()

    def start(self):
        if not self._clock.isValid():
            self._clock.start()
        elif not self._timer.isActive():
            self._clock.restart()
        if not self._timer.isActive():
            self._timer.start()

    def stop(self):
        self._timer.stop()

    def showEvent(self, event):
        super().showEvent(event)
        self.start()

    def hideEvent(self, event):
        self.stop()
        super().hideEvent(event)

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
        track = QPen(QColor(self._track_color))
        track.setWidthF(max(1.5, w * 0.10))
        p.setPen(track)
        p.drawArc(rect, 0, 360 * 16)
        # Bright rotating arc (about 100°).
        arc = QPen(QColor(self._arc_color))
        arc.setWidthF(max(1.5, w * 0.10))
        arc.setCapStyle(Qt.PenCapStyle.RoundCap)
        p.setPen(arc)
        p.drawArc(rect, int(-self._angle * 16), int(100 * 16))


class HoverUpdateButton(QPushButton):
    """Update button that exposes enter/leave without changing click behavior."""

    hovered = pyqtSignal(bool)

    def enterEvent(self, event):
        self.hovered.emit(True)
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.hovered.emit(False)
        super().leaveEvent(event)


class UpdateChangesPopover(QFrame):
    """Non-activating release-notes card positioned below the update button."""

    def __init__(self, parent=None):
        super().__init__(parent, Qt.WindowType.ToolTip | Qt.WindowType.FramelessWindowHint)
        self.setAttribute(Qt.WidgetAttribute.WA_ShowWithoutActivating, True)
        self.setObjectName("UpdateChangesPopover")
        self.setFixedWidth(310)
        self.setStyleSheet(
            "QFrame#UpdateChangesPopover{background:#FFFFFF;border:1px solid #D1D5DB;"
            "border-radius:10px;}"
            "QLabel{background:transparent;border:none;color:#374151;}")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 12, 14, 12)
        lay.setSpacing(5)
        heading = QLabel("Čo je nové")
        heading.setStyleSheet("font-size:11px;font-weight:800;color:#047857;")
        lay.addWidget(heading)
        self.release_title = QLabel("")
        self.release_title.setTextFormat(Qt.TextFormat.PlainText)
        self.release_title.setWordWrap(True)
        self.release_title.setStyleSheet(
            "font-size:13px;font-weight:800;color:#111827;")
        lay.addWidget(self.release_title)
        self.release_description = QLabel("")
        self.release_description.setTextFormat(Qt.TextFormat.PlainText)
        self.release_description.setWordWrap(True)
        self.release_description.setStyleSheet(
            "font-size:12px;color:#4B5563;line-height:1.25;")
        lay.addWidget(self.release_description)
        self.hide()

    def set_changes(self, title, description):
        title = str(title or "Nová verzia UltraPilot").strip()
        description = str(description or "Najnovšie opravy a vylepšenia.").strip()
        if len(description) > 800:
            description = description[:797].rstrip() + "…"
        self.release_title.setText(title)
        self.release_description.setText(description)
        self.adjustSize()

    def show_below(self, button):
        self.adjustSize()
        anchor = button.mapToGlobal(QPoint(0, button.height() + 6))
        screen = button.screen().availableGeometry()
        x = min(max(anchor.x(), screen.left() + 8),
                screen.right() - self.width() - 8)
        y = anchor.y()
        if y + self.height() > screen.bottom() - 8:
            y = button.mapToGlobal(QPoint(0, -self.height() - 6)).y()
        self.move(x, y)
        self.show()
        self.raise_()


class _CheckWorker(QThread):
    """Calls check_for_update off the UI thread."""
    done = pyqtSignal(bool, object)  # (available, latest_tag_or_None)

    def run(self):
        try:
            from core.update_check import check_for_update
            self.done.emit(*check_for_update())
        except Exception:
            self.done.emit(False, None)


class _DownloadWorker(QThread):
    """Downloads and verifies an update without changing the application."""
    progress = pyqtSignal(float, str)
    done = pyqtSignal(bool)

    def __init__(self, target_commit="", parent=None):
        super().__init__(parent)
        self.target_commit = target_commit

    def run(self):
        try:
            from core.update_check import prepare_update
            ok = prepare_update(
                progress_cb=lambda f, t: self.progress.emit(f, t),
                target_commit=self.target_commit)
            self.done.emit(bool(ok))
        except Exception as e:
            logging.error("update download failed: %s", e)
            self.progress.emit(1.0, "chyba: " + str(e))
            self.done.emit(False)


class _InstallWorker(QThread):
    """Applies only the already downloaded and verified update."""
    progress = pyqtSignal(float, str)
    done = pyqtSignal(bool)

    def run(self):
        try:
            from core.update_check import install_prepared_update
            ok = install_prepared_update(
                progress_cb=lambda f, t: self.progress.emit(f, t))
            self.done.emit(bool(ok))
        except Exception as e:
            logging.error("update install failed: %s", e)
            self.progress.emit(1.0, "chyba: " + str(e))
            self.done.emit(False)


class UpdateConfirmDialog(QDialog):
    """Staged update dialog: download, ready, then install and restart."""

    download_requested = pyqtSignal()
    install_requested = pyqtSignal()

    def __init__(self, latest_tag, title="", description="", parent=None):
        super().__init__(parent)
        from core.update_check import _display_commit
        latest_tag = _display_commit(str(latest_tag)) or str(latest_tag)
        self.setWindowTitle("Aktualizovať UltraPilot")
        self.setModal(True)
        self.setFixedSize(500, 340)
        # Match the application's default white ETS2LA-style surfaces.
        self.setStyleSheet(
            "UpdateConfirmDialog{background:#FFFFFF;}"
            "QLabel{color:#111827;background:transparent;}")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(28, 24, 28, 20)
        lay.setSpacing(12)

        head = QHBoxLayout()
        head.setSpacing(14)
        self.brand_logo = QLabel()
        self.brand_logo.setFixedSize(52, 52)
        self.brand_logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.brand_logo.setStyleSheet(
            "background:#F9FAFB;border:1px solid #E5E7EB;border-radius:14px;")
        try:
            from core.paths import resource
            logo_path = resource("assets", "favicon.ico")
            pixmap = QIcon(logo_path).pixmap(40, 40)
            if not pixmap.isNull():
                self.brand_logo.setPixmap(pixmap)
        except Exception:
            pass
        head.addWidget(self.brand_logo)
        col = QVBoxLayout()
        col.setSpacing(2)
        self.title_lbl = QLabel("Dostupná aktualizácia")
        self.title_lbl.setStyleSheet("font-size:18px;font-weight:800;color:#111827;")
        col.addWidget(self.title_lbl)
        ver = QLabel("Commit: " + str(latest_tag))
        ver.setStyleSheet("font-size:12px;color:#047857;font-weight:700;")
        col.addWidget(ver)
        head.addLayout(col, stretch=1)
        lay.addLayout(head)

        # Release notes live in the hover card below the sidebar update button.
        # This dialog is deliberately limited to download/install state.
        self.note = QLabel(
            "Najprv sa aktualizácia bezpečne stiahne a overí. Aplikácia sa "
            "zmení až po potvrdení inštalácie.")
        self.note.setWordWrap(True)
        self.note.setStyleSheet("font-size:13px;color:#4B5563;background:#F9FAFB;border:1px solid #E5E7EB;border-radius:10px;padding:12px;")
        lay.addWidget(self.note)

        self.progress_text = QLabel("")
        self.progress_text.setStyleSheet(
            "font-size:12px;color:#374151;font-weight:700;")
        self.progress_text.setVisible(False)
        lay.addWidget(self.progress_text)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setFixedHeight(14)
        self.progress.setTextVisible(False)
        self.progress.setStyleSheet(
            "QProgressBar{background:#E5E7EB;border:none;border-radius:7px;}"
            "QProgressBar::chunk{background:#10B981;border-radius:7px;}")
        self.progress.setVisible(False)
        lay.addWidget(self.progress)
        lay.addStretch()

        row = QHBoxLayout()
        row.addStretch()
        self.cancel_btn = QPushButton("Zrušiť")
        self.cancel_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.cancel_btn.setFixedWidth(110)
        self.cancel_btn.setStyleSheet(
            "QPushButton{background:#FFFFFF;color:#374151;border:1px solid #D1D5DB;"
            "border-radius:8px;padding:9px;font-weight:600;}"
            "QPushButton:hover{background:#F9FAFB;border-color:#9CA3AF;}")
        self.cancel_btn.clicked.connect(self.reject)
        row.addWidget(self.cancel_btn)
        self.primary_btn = QPushButton("Stiahnuť")
        self.primary_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.primary_btn.setMinimumWidth(150)
        self.primary_btn.setStyleSheet(
            "QPushButton{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            "stop:0 #10B981, stop:1 #059669);color:#FFFFFF;border:none;"
            "border-radius:8px;padding:9px;font-weight:700;}"
            "QPushButton:disabled{background:#A7F3D0;color:#ECFDF5;}"
            "QPushButton:hover{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            "stop:0 #34D399, stop:1 #059669);}")
        self.primary_btn.clicked.connect(self.download_requested.emit)
        self.primary_btn.setDefault(True)
        row.addWidget(self.primary_btn)
        lay.addLayout(row)

    def set_downloading(self):
        self.title_lbl.setText("Sťahujem aktualizáciu")
        self.progress.setVisible(True)
        self.progress_text.setVisible(True)
        self.progress.setValue(0)
        self.progress_text.setText(
            "Stiahnuté 0.00 MB • celkovú veľkosť zisťujem")
        self.primary_btn.setEnabled(False)
        self.primary_btn.setText("Sťahujem…")
        self.cancel_btn.setEnabled(False)

    def set_progress(self, fraction, text):
        self.progress.setValue(max(0, min(100, int(float(fraction) * 100))))
        self.progress_text.setText(str(text))

    def set_ready(self, size_text=None):
        self.title_lbl.setText("Aktualizácia je pripravená na inštaláciu")
        self.note.setText(
            "Balík bol úplne stiahnutý a overený. Kliknutím na tlačidlo "
            "sa aktualizácia nainštaluje a UltraPilot sa reštartuje.")
        self.progress.setValue(100)
        if size_text:
            self.progress_text.setVisible(True)
            self.progress_text.setText(str(size_text))
        self.primary_btn.setText("Inštalovať a reštartovať")
        self.primary_btn.setEnabled(True)
        self.cancel_btn.setEnabled(True)
        try:
            self.primary_btn.clicked.disconnect()
        except Exception:
            pass
        self.primary_btn.clicked.connect(self.install_requested.emit)

    def set_installing(self):
        self.title_lbl.setText("Inštalácia aktualizácie")
        self.progress_text.setText("Inštalujem pripravené súbory…")
        self.primary_btn.setText("Inštalujem…")
        self.primary_btn.setEnabled(False)
        self.cancel_btn.setEnabled(False)

    def set_failed(self, text="Aktualizácia zlyhala — skús znova."):
        self.title_lbl.setText("Aktualizácia sa nepodarila")
        self.progress_text.setVisible(True)
        self.progress_text.setText(text)
        self.primary_btn.setText("Skúsiť znova")
        self.primary_btn.setEnabled(True)
        self.cancel_btn.setEnabled(True)
        try:
            self.primary_btn.clicked.disconnect()
        except Exception:
            pass
        self.primary_btn.clicked.connect(self.download_requested.emit)

    def set_install_failed(self):
        self.set_failed(
            "Inštalácia zlyhala. Stiahnutý balík zostal pripravený.")
        self.primary_btn.setText("Skúsiť inštaláciu znova")
        try:
            self.primary_btn.clicked.disconnect()
        except Exception:
            pass
        self.primary_btn.clicked.connect(self.install_requested.emit)


class UpdateCheckerWidget(QWidget):
    """Compact sidebar widget: version label + check/update button + spinner."""

    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        from core.update_check import VERSION, git_commit
        self._version = VERSION
        self._commit = git_commit()
        self._check_worker = None
        self._download_worker = None
        self._install_worker = None
        self._update_dialog = None
        self._update_available = False
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
        self.btn = HoverUpdateButton("Aktualizácia")
        self.btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._apply_btn_style()
        self.btn.clicked.connect(self.check)
        self.changes_popover = UpdateChangesPopover(self)
        self.btn.hovered.connect(self._on_update_hover)
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
        self._update_available = False
        self.changes_popover.hide()
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
            self._update_available = False
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
            self._update_available = True
            self.changes_popover.set_changes(
                self._latest_title, self._latest_description)
            # Remember the tag/SHA so the confirm dialog can show it.
            self._latest_tag = str(latest)
            self.status_lbl.setText("Dostupná nová verzia")
            self.btn.setText("Aktualizovať")
            self._apply_btn_style(update_available=True)
            try:
                self.btn.clicked.disconnect()
            except Exception:
                pass
            self.btn.clicked.connect(self._confirm_update)
        else:
            self._update_available = False
            self.changes_popover.hide()
            self.status_lbl.setText("aktuálna")
            self.btn.setText("Aktualizácia")
            self._apply_btn_style(update_available=False)
            try:
                self.btn.clicked.disconnect()
            except Exception:
                pass
            self.btn.clicked.connect(self.check)

    def _on_update_hover(self, entered):
        if entered and self._update_available and self.btn.isVisible():
            self.changes_popover.show_below(self.btn)
        else:
            self.changes_popover.hide()

    def _confirm_update(self):
        self.changes_popover.hide()
        latest = getattr(self, "_latest_tag", None) or ""
        dlg = UpdateConfirmDialog(
            latest,
            title=getattr(self, "_latest_title", ""),
            description=getattr(self, "_latest_description", ""),
            parent=self)
        self._update_dialog = dlg
        dlg.download_requested.connect(self._start_download)
        dlg.install_requested.connect(self._start_install)
        try:
            from core.update_check import prepared_update_info
            staged = prepared_update_info()
            if (staged and (not latest
                    or str(staged.get("target_commit", "")) == str(latest))):
                dlg.progress.setVisible(True)
                dlg.progress_text.setVisible(True)
                from core.update_check import _format_prepared_update_size
                dlg.set_ready(_format_prepared_update_size(staged))
        except Exception:
            pass
        dlg.exec()
        self._update_dialog = None

    def closeEvent(self, event):
        self.changes_popover.close()
        super().closeEvent(event)

    def _start_download(self):
        if (self._download_worker is not None
                and self._download_worker.isRunning()):
            return
        dlg = self._update_dialog
        if dlg is None:
            return
        self.btn.hide()
        self.spinner.show()
        self.status_lbl.setText("Sťahujem aktualizáciu…")
        dlg.set_downloading()
        self._download_worker = _DownloadWorker(
            getattr(self, "_latest_tag", ""), self)
        self._download_worker.progress.connect(self._on_download_progress)
        self._download_worker.done.connect(self._on_downloaded)
        self._download_worker.start()

    def _on_download_progress(self, fraction, text):
        if self._update_dialog is not None:
            self._update_dialog.set_progress(fraction, text)
        self.status_lbl.setText("Sťahujem aktualizáciu…")

    def _on_downloaded(self, ok):
        self.spinner.hide()
        self.btn.show()
        if ok:
            self.status_lbl.setText("Aktualizácia je pripravená")
            self.btn.setText("Aktualizovať")
            if self._update_dialog is not None:
                try:
                    from core.update_check import (prepared_update_info,
                                                   _format_prepared_update_size)
                    size_text = _format_prepared_update_size(
                        prepared_update_info())
                except Exception:
                    size_text = None
                self._update_dialog.set_ready(size_text)
        else:
            self.status_lbl.setText("Sťahovanie aktualizácie zlyhalo")
            if self._update_dialog is not None:
                self._update_dialog.set_failed(
                    "Sťahovanie zlyhalo — skontroluj pripojenie a skús znova.")

    def _start_install(self):
        if (self._install_worker is not None
                and self._install_worker.isRunning()):
            return
        if self._update_dialog is None:
            return
        self.btn.hide()
        self.spinner.show()
        self.status_lbl.setText("Inštalácia aktualizácie…")
        self._update_dialog.set_installing()
        self._install_worker = _InstallWorker(self)
        self._install_worker.progress.connect(self._on_install_progress)
        self._install_worker.done.connect(self._on_installed)
        self._install_worker.start()

    def _on_install_progress(self, fraction, text):
        if self._update_dialog is not None:
            self._update_dialog.set_progress(fraction, text)

    def _on_installed(self, ok):
        self.spinner.hide()
        if ok:
            self.status_lbl.setText("Aktualizácia nainštalovaná — reštartujem…")
            if self._update_dialog is not None:
                self._update_dialog.accept()
            QTimer.singleShot(400, self._restart)
        else:
            self.btn.show()
            self.status_lbl.setText("Inštalácia aktualizácie zlyhala")
            if self._update_dialog is not None:
                self._update_dialog.set_install_failed()

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
