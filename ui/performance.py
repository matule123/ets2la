import os
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QProgressBar
from PyQt6.QtCore import QTimer

try:
    import psutil
except Exception:
    psutil = None


class PerformancePage(QWidget):
    """Performance card: total RAM, app RAM, and per-process (plugin) usage."""

    def __init__(self, state):
        super().__init__()
        self.state = state
        lay = QVBoxLayout(self)
        lay.setContentsMargins(30, 30, 30, 30)
        lay.setSpacing(12)

        title = QLabel("📊 Performance")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #065F46;")
        lay.addWidget(title)

        # Total system RAM bar.
        self.total_lbl = QLabel("System RAM")
        self.total_lbl.setStyleSheet("color:#6B7280; font-weight:bold;")
        lay.addWidget(self.total_lbl)
        self.total_bar = QProgressBar()
        lay.addWidget(self.total_bar)

        self.app_lbl = QLabel("UltraPilot RAM: —")
        self.app_lbl.setStyleSheet("font-weight:bold; margin-top:6px;")
        lay.addWidget(self.app_lbl)

        # Per-process rows container.
        self.rows_frame = QFrame()
        self.rows_frame.setStyleSheet("background:#FFFFFF; border:1px solid #E5E7EB; border-radius:12px; padding:10px;")
        self.rows = QVBoxLayout(self.rows_frame)
        lay.addWidget(self.rows_frame)
        lay.addStretch()

        if psutil is None:
            self.app_lbl.setText("psutil not installed — performance unavailable.")

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(1500)

    def _clear_rows(self):
        while self.rows.count():
            w = self.rows.takeAt(0).widget()
            if w:
                w.setParent(None)

    def refresh(self):
        if psutil is None:
            return
        vm = psutil.virtual_memory()
        self.total_bar.setValue(int(vm.percent))
        self.total_bar.setFormat(f"{vm.percent:.0f}%  ({(vm.total-vm.available)/1e9:.1f} / {vm.total/1e9:.1f} GB)")

        # Find our process tree (this UI process + parent + siblings).
        me = psutil.Process(os.getpid())
        try:
            root = me.parent() or me
        except Exception:
            root = me
        procs = [root] + root.children(recursive=True)
        seen, app_rss = set(), 0
        plugins = []
        for p in procs:
            try:
                if p.pid in seen:
                    continue
                seen.add(p.pid)
                rss = p.memory_info().rss
                app_rss += rss
                # multiprocessing names plugin workers "Plugin-<folder>".
                pname = p.name()
                try:
                    mp_name = next((a for a in p.cmdline() if a.startswith("Plugin-")), None)
                except Exception:
                    mp_name = None
                label = mp_name or pname
                if label.startswith("Plugin-"):
                    plugins.append((label.replace("Plugin-", "").capitalize(), rss))
            except Exception:
                continue

        plug_rss = sum(r for _, r in plugins) or 1
        self.app_lbl.setText(f"UltraPilot RAM: {app_rss/1e6:.0f} MB total · "
                             f"plugins {plug_rss/1e6:.0f} MB")

        self._clear_rows()
        if not plugins:
            lbl = QLabel("No plugin processes running yet (start the app with the game).")
            lbl.setStyleSheet("color:#9CA3AF;")
            self.rows.addWidget(lbl)
        for label, rss in sorted(plugins, key=lambda r: -r[1]):
            pct = 100 * rss / plug_rss
            row = QHBoxLayout()
            n = QLabel(label); n.setFixedWidth(150)
            bar = QProgressBar(); bar.setValue(int(pct))
            bar.setFormat(f"{rss/1e6:.0f} MB  ({pct:.0f}%)")
            row.addWidget(n); row.addWidget(bar)
            holder = QWidget(); holder.setLayout(row)
            self.rows.addWidget(holder)
