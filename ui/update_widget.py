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


ACCENT = "#2563EB"
ACCENT_HOVER = "#1D4ED8"
ACCENT_SOFT = "#DBEAFE"
ACCENT_LIGHT = "#60A5FA"


def _ui_palette(source=None):
    """Resolve the live app palette without requiring a specific state type."""
    state = getattr(source, "state", source)
    mode = "light"
    getter = getattr(state, "get", None)
    if callable(getter):
        try:
            mode = getter("ui_theme", "light") or "light"
        except Exception:
            pass
    from core.theme import palette
    return palette(mode)


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
        self._labels = []
        p = _ui_palette(parent)
        self.setStyleSheet(
            "QFrame#UpdateChangesPopover{background:" + p["surface"]
            + ";border:1px solid " + p["border"] + ";border-radius:12px;}"
            "QLabel{background:transparent;border:none;color:" + p["text"] + ";}")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 12, 14, 12)
        lay.setSpacing(5)
        heading = QLabel("Čo je nové")
        heading.setObjectName("UpdatePopoverHeading")
        heading.setStyleSheet("font-size:11px;font-weight:800;color:" + ACCENT + ";")
        lay.addWidget(heading)
        self.release_title = QLabel("")
        self.release_title.setObjectName("UpdatePopoverTitle")
        self.release_title.setTextFormat(Qt.TextFormat.PlainText)
        self.release_title.setWordWrap(True)
        self.release_title.setStyleSheet(
            "font-size:13px;font-weight:800;color:" + p["text"] + ";")
        lay.addWidget(self.release_title)
        self.release_description = QLabel("")
        self.release_description.setObjectName("UpdatePopoverDescription")
        self.release_description.setTextFormat(Qt.TextFormat.PlainText)
        self.release_description.setWordWrap(True)
        self.release_description.setStyleSheet(
            "font-size:12px;color:" + p["muted"] + ";line-height:1.25;")
        lay.addWidget(self.release_description)
        self.hide()

    def restyle(self, theme):
        p = _ui_palette({"ui_theme": theme})
        self.setStyleSheet(
            "QFrame#UpdateChangesPopover{background:" + p["surface"]
            + ";border:1px solid " + p["border"] + ";border-radius:12px;}"
            "QLabel{background:transparent;border:none;color:" + p["text"] + ";}")
        self.release_title.setStyleSheet(
            "font-size:13px;font-weight:800;color:" + p["text"] + ";")
        self.release_description.setStyleSheet(
            "font-size:12px;color:" + p["muted"] + ";line-height:1.25;")

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
        p = _ui_palette(parent)
        self._palette = p
        from core.update_check import _display_commit
        latest_tag = _display_commit(str(latest_tag)) or str(latest_tag)
        self.setWindowTitle("Aktualizovať UltraPilot")
        self.setModal(True)
        self.setFixedSize(520, 370)
        self.setStyleSheet(
            "UpdateConfirmDialog{background:" + p["bg"] + ";}"
            "QLabel{color:" + p["text"] + ";background:transparent;}")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(28, 24, 28, 20)
        lay.setSpacing(12)

        head = QHBoxLayout()
        head.setSpacing(14)
        self.brand_logo = QLabel()
        self.brand_logo.setFixedSize(52, 52)
        self.brand_logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.brand_logo.setStyleSheet(
            "background:" + p["card"] + ";border:1px solid "
            + p["border"] + ";border-radius:14px;")
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
        self.title_lbl.setStyleSheet(
            "font-size:18px;font-weight:800;color:" + p["text"] + ";")
        col.addWidget(self.title_lbl)
        ver = QLabel("Commit: " + str(latest_tag))
        ver.setStyleSheet("font-size:12px;color:" + ACCENT + ";font-weight:700;")
        col.addWidget(ver)
        self.phase_badge = QLabel("Bezpečná aktualizácia")
        self.phase_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._set_phase_badge("Bezpečná aktualizácia", "active")
        col.addWidget(self.phase_badge, alignment=Qt.AlignmentFlag.AlignLeft)
        head.addLayout(col, stretch=1)
        lay.addLayout(head)

        # Release notes live in the hover card below the sidebar update button.
        # This dialog is deliberately limited to download/install state.
        self.note = QLabel(
            "Najprv sa aktualizácia bezpečne stiahne a overí. Aplikácia sa "
            "zmení až po potvrdení inštalácie.")
        self.note.setWordWrap(True)
        self.note.setStyleSheet(
            "font-size:13px;color:" + p["muted"] + ";background:"
            + p["card"] + ";border:1px solid " + p["border"]
            + ";border-radius:12px;padding:12px;")
        lay.addWidget(self.note)

        self.progress_text = QLabel("")
        self.progress_text.setStyleSheet(
            "font-size:12px;color:" + p["text"] + ";font-weight:700;")
        self.progress_text.setVisible(False)
        progress_header = QHBoxLayout()
        progress_header.setContentsMargins(0, 0, 0, 0)
        progress_header.addWidget(self.progress_text, 1)
        self.progress_percent = QLabel("0 %")
        self.progress_percent.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.progress_percent.setStyleSheet(
            "font-size:11px;color:" + ACCENT + ";font-weight:800;")
        self.progress_percent.setVisible(False)
        progress_header.addWidget(self.progress_percent)
        lay.addLayout(progress_header)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setFixedHeight(12)
        self.progress.setTextVisible(False)
        self.progress.setStyleSheet(
            "QProgressBar{background:" + p["border"]
            + ";border:none;border-radius:6px;}"
            "QProgressBar::chunk{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            "stop:0 #2563EB,stop:0.58 #3B82F6,stop:1 #60A5FA);"
            "border-radius:6px;}")
        self.progress.setVisible(False)
        lay.addWidget(self.progress)
        lay.addStretch()

        row = QHBoxLayout()
        row.addStretch()
        self.cancel_btn = QPushButton("Zrušiť")
        self.cancel_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.cancel_btn.setFixedWidth(110)
        self.cancel_btn.setStyleSheet(
            "QPushButton{background:" + p["surface"] + ";color:" + p["text"]
            + ";border:1px solid " + p["border"]
            + ";border-radius:9px;padding:9px;font-weight:600;}"
            "QPushButton:hover{background:" + p["card2"]
            + ";border-color:#93C5FD;color:" + ACCENT + ";}")
        self.cancel_btn.clicked.connect(self.reject)
        row.addWidget(self.cancel_btn)
        self.primary_btn = QPushButton("Stiahnuť")
        self.primary_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self.primary_btn.setMinimumWidth(150)
        self.primary_btn.setStyleSheet(
            "QPushButton{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            "stop:0 #3B82F6,stop:1 #2563EB);color:#FFFFFF;border:1px solid #1D4ED8;"
            "border-radius:9px;padding:9px;font-weight:700;}"
            "QPushButton:disabled{background:#BFDBFE;color:#EFF6FF;border-color:#BFDBFE;}"
            "QPushButton:hover{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
            "stop:0 #60A5FA,stop:1 #2563EB);}")
        self.primary_btn.clicked.connect(self.download_requested.emit)
        self.primary_btn.setDefault(True)
        row.addWidget(self.primary_btn)
        lay.addLayout(row)

    def _set_phase_badge(self, text, state):
        dark = self._palette["bg"] == "#0D1117"
        colours = {
            "active": (("#BFDBFE", "#172554", "#1E3A8A") if dark else
                       ("#1D4ED8", "#EFF6FF", "#DBEAFE")),
            "success": (("#86EFAC", "#052E16", "#166534") if dark else
                        ("#047857", "#ECFDF5", "#A7F3D0")),
            "error": (("#FDA4AF", "#3F0A0A", "#7F1D1D") if dark else
                      ("#B42318", "#FEF3F2", "#FECDCA")),
        }
        foreground, background, border = colours[state]
        self.phase_badge.setText(text)
        self.phase_badge.setStyleSheet(
            "font-size:10px;font-weight:750;color:" + foreground
            + ";background:" + background + ";border:1px solid " + border
            + ";border-radius:7px;padding:3px 8px;")

    def set_downloading(self):
        self.title_lbl.setText("Sťahujem aktualizáciu")
        self._set_phase_badge("Sťahovanie a overenie", "active")
        self.progress.setVisible(True)
        self.progress_text.setVisible(True)
        self.progress_percent.setVisible(True)
        self.progress.setValue(0)
        self.progress_percent.setText("0 %")
        self.progress_text.setText(
            "Stiahnuté 0.00 MB • celkovú veľkosť zisťujem")
        self.primary_btn.setEnabled(False)
        self.primary_btn.setText("Sťahujem…")
        self.cancel_btn.setEnabled(False)

    def set_progress(self, fraction, text):
        value = max(0, min(100, int(float(fraction) * 100)))
        self.progress.setValue(value)
        self.progress_percent.setVisible(True)
        self.progress_percent.setText(f"{value} %")
        self.progress_text.setText(str(text))

    def set_ready(self, size_text=None):
        self.title_lbl.setText("Aktualizácia je pripravená na inštaláciu")
        self._set_phase_badge("Overené a pripravené", "success")
        self.note.setText(
            "Balík bol úplne stiahnutý a overený. Kliknutím na tlačidlo "
            "sa aktualizácia nainštaluje a UltraPilot sa reštartuje.")
        self.progress.setValue(100)
        self.progress_percent.setVisible(True)
        self.progress_percent.setText("100 %")
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
        self._set_phase_badge("Inštalácia", "active")
        self.progress_text.setText("Inštalujem pripravené súbory…")
        self.primary_btn.setText("Inštalujem…")
        self.primary_btn.setEnabled(False)
        self.cancel_btn.setEnabled(False)

    def set_failed(self, text="Aktualizácia zlyhala — skús znova."):
        self.title_lbl.setText("Aktualizácia sa nepodarila")
        self._set_phase_badge("Vyžaduje pozornosť", "error")
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
        self.progress.setTextVisible(False)
        p = _ui_palette(self.state)
        self.progress.setStyleSheet(
            "QProgressBar{background:" + p["border"]
            + ";border:none;border-radius:4px;}"
            "QProgressBar::chunk{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            "stop:0 #2563EB,stop:1 #60A5FA);border-radius:4px;}")
        self.progress.setVisible(False)
        lay.addWidget(self.progress)
        self.restyle(getattr(self.state, "get", lambda *_: "light")(
            "ui_theme", "light") if self.state is not None else "light")

    def restyle(self, theme):
        p = _ui_palette({"ui_theme": theme})
        self.version_lbl.setStyleSheet(
            "font-size:11px;font-weight:700;color:" + p["muted"]
            + ";border:none;")
        self.status_lbl.setStyleSheet(
            "font-size:10px;color:" + p["muted"] + ";border:none;")
        self.progress.setStyleSheet(
            "QProgressBar{background:" + p["border"]
            + ";border:none;border-radius:4px;}"
            "QProgressBar::chunk{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            "stop:0 #2563EB,stop:1 #60A5FA);border-radius:4px;}")
        self._apply_btn_style(self._update_available)
        self.changes_popover.restyle(theme)

    def _apply_btn_style(self, update_available=False):
        """Neutral check look, or blue primary action when one is available."""
        p = _ui_palette(self.state)
        if update_available:
            self.btn.setStyleSheet(
                "QPushButton{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
                "stop:0 #2563EB,stop:1 #3B82F6);color:#FFFFFF;border:1px solid #1D4ED8;"
                "border-radius:7px;padding:5px 10px;font-size:11px;font-weight:700;}"
                "QPushButton:hover{background:#1D4ED8;}")
        else:
            self.btn.setStyleSheet(
                "QPushButton{background:" + p["surface"] + ";color:" + p["muted"]
                + ";border:1px solid " + p["border"]
                + ";border-radius:7px;padding:5px 10px;font-size:11px;font-weight:600;}"
                "QPushButton:hover{background:" + p["card2"] + ";color:"
                + ACCENT + ";border-color:#93C5FD;}")

    def _version_text(self):
        t = "v" + self._version
        if self._commit:
            # The badge contains only the short SHA; never append build counts,
            # line numbers or other metadata from an older commit file.
            from core.update_check import _display_commit
            commit = _display_commit(self._commit)
            t += "  ·  " + (commit or "neznáma revízia")
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
