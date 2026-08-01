import multiprocessing as mp
import sys
import os
import time
import logging
from collections import deque

# Ensure the project root is in path before importing project modules.
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class RestartGuard:
    """Stops a broken child import from becoming an endless restart loop."""
    def __init__(self, max_crashes=3, window_seconds=20.0):
        self.max_crashes = int(max_crashes)
        self.window_seconds = float(window_seconds)
        self._crashes = {}

    def allow_restart(self, name, now=None):
        now = time.monotonic() if now is None else float(now)
        history = self._crashes.setdefault(name, deque())
        while history and now - history[0] > self.window_seconds:
            history.popleft()
        history.append(now)
        return len(history) <= self.max_crashes


def run_engine(shared_dict):
    """Process for the Autopilot Engine."""
    from core.logger import setup as _log_setup
    _log_setup()
    logging.info("Launching Engine Process...")
    from core.engine import UltraPilotEngine
    engine = UltraPilotEngine(shared_dict)
    engine.start()


def _play_boot_sound(state):
    """Play the startup chime if the user has it enabled and a file exists."""
    try:
        if not state.get("startup_sound", True):
            return
        from core import sound
        sound.play("boot")
    except Exception:
        pass


def _set_app_id():
    """Set a stable Windows AppUserModelID so all windows group under one
    taskbar button with our icon. Must run BEFORE any window is created."""
    try:
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("UltraPilot.App")
    except Exception:
        pass


def _app_icon():
    """Resolve the app icon (QIcon) from assets/favicon.ico."""
    from PyQt6.QtGui import QIcon
    from core.paths import resource
    p = resource("assets", "favicon.ico")
    return QIcon(p) if p and os.path.exists(p) else QIcon()


def _ensure_ui_package_name():
    """Alias a case-mismatched source checkout without affecting frozen UI."""
    try:
        __import__("ui")
    except ModuleNotFoundError:
        # Git records the package as ``ui``. A historical Windows checkout can
        # retain ``UI`` on disk, and Python 3.14 now rejects that case mismatch.
        # Keep one canonical import identity so app.py's internal imports do
        # not load a second copy of the package.
        import UI as ui_package
        sys.modules.setdefault("ui", ui_package)


def run_splash(shared_dict):
    """Animate startup independently while the main UI builds its pages."""
    from core.logger import setup as _log_setup
    _log_setup()
    _set_app_id()
    from PyQt6.QtWidgets import QApplication
    from PyQt6.QtCore import QTimer
    _ensure_ui_package_name()
    try:
        from ui.splash import BootSplash
    except ModuleNotFoundError:
        from UI.splash import BootSplash
    from core.ipc.shared_state import SharedState

    app = QApplication([])
    app.setWindowIcon(_app_icon())
    state = SharedState(shared_dict)
    splash = BootSplash()
    splash.setWindowIcon(_app_icon())
    update_notice = False
    try:
        from core.update_check import take_update_startup_notice
        update_notice = take_update_startup_notice()
    except Exception:
        pass
    if update_notice:
        splash.show_update_installation()
    splash.show()

    def finish_when_ready():
        if (state.get("ui_ready", False)
                or state.get("splash_close_requested", False)
                or state.get("app_shutdown_requested", False)):
            splash.close()
            app.quit()

    poll = QTimer()
    poll.timeout.connect(finish_when_ready)
    poll.start(50)
    sys.exit(app.exec())


def run_ui(shared_dict):
    """Process for the Main Control Panel UI."""
    from core.logger import setup as _log_setup
    _log_setup()
    logging.info("Launching UI Process...")
    _set_app_id()
    _ensure_ui_package_name()
    from PyQt6.QtWidgets import QApplication
    from ui.app import UltraPilotApp
    from core.ipc.shared_state import SharedState
    from core.settings.manager import SettingsManager

    app = QApplication(sys.argv)
    app.setWindowIcon(_app_icon())
    state = SharedState(shared_dict)

    # First-run onboarding: if the user hasn't completed setup yet, show the
    # wizard before the main window. When the wizard finishes it writes
    # ``onboarded = true`` to settings and we open the dashboard.
    try:
        sm = SettingsManager()
        if not sm.get("onboarded", False):
            from ui.onboarding import OnboardingWizard
            # Onboarding is itself the first visible app window.
            state.set("splash_close_requested", True)
            wizard = OnboardingWizard(state)
            wizard.show()
            main_window = {"w": None}

            def launch_main():
                main_window["w"] = UltraPilotApp(state)
                main_window["w"].show()
                _play_boot_sound(state)

            wizard.finished.connect(launch_main)
            sys.exit(app.exec())
            return
    except Exception as e:
        logging.warning("Onboarding skipped (%s) — opening main window.", e)

    window = UltraPilotApp(state)
    window.show()
    _play_boot_sound(state)
    sys.exit(app.exec())


def run_hud(shared_dict):
    """Process for the transparent HUD overlay."""
    from core.logger import setup as _log_setup
    _log_setup()
    _set_app_id()
    from core.hud import run_hud as _run_hud
    from core.ipc.shared_state import SharedState
    _run_hud(SharedState(shared_dict))


def run_ar(shared_dict):
    """Process for the click-through AR overlay drawn over the game."""
    from core.logger import setup as _log_setup
    _log_setup()
    _set_app_id()
    from PyQt6.QtWidgets import QApplication
    from core.ar_overlay import AROverlay
    from core.ipc.shared_state import SharedState
    app = QApplication(sys.argv)
    ov = AROverlay(SharedState(shared_dict))
    ov.show()
    sys.exit(app.exec())


def _ensure_game_dlls():
    """Best-effort: install the SCS telemetry + controller DLLs into the game.

    The DLLs are third-party binaries shipped in assets/; if a file is missing
    or locked by a running game this is a quiet no-op.  Safe to run every launch."""
    try:
        from core.sdk.game_utils import install_game_dlls
        from core.paths import resource
        install_game_dlls(resource("assets"))
    except Exception as e:
        logging.debug(f"Game DLL install skipped: {e}")


def _ensure_vigembus():
    """Best-effort: install the ViGEmBus driver (vgamepad fallback) on startup."""
    try:
        from core.sdk.vigembus import ensure_vigembus
        from core.paths import resource
        ensure_vigembus(resource("assets"))
    except Exception as e:
        logging.debug(f"ViGEmBus check skipped: {e}")


def main():
    try:
        from core.logger import setup as _log_setup
        _log_setup()
    except Exception:
        logging.basicConfig(level=logging.INFO)
    logging.info("UltraPilot Bootloader starting...")
    _ensure_game_dlls()
    _ensure_vigembus()

    # ONE shared manager dict, handed to every process.
    manager = mp.Manager()
    shared_dict = manager.dict()

    targets = {
        "Splash": run_splash,
        "Engine": run_engine,
        "UI": run_ui,
        "HUD": run_hud,
        "AR": run_ar,   # draws nothing until enabled in Settings (ar_enabled)
    }

    def spawn(name):
        p = mp.Process(target=targets[name], args=(shared_dict,), name=name)
        p.start()
        logging.info(f"Process {name} started (PID: {p.pid})")
        return p

    processes = {name: spawn(name) for name in targets}
    restart_guard = RestartGuard()

    def shutdown():
        logging.info("Shutting down UltraPilot…")
        # Keep the Manager alive while the Engine asks every plugin to stop.
        # Otherwise a normal close produces BrokenPipe/EOF crash reports.
        try:
            shared_dict["app_shutdown_requested"] = True
        except Exception:
            pass
        engine = processes.get("Engine")
        if engine is not None and engine.is_alive():
            engine.join(timeout=4)
        for proc in processes.values():
            if proc.is_alive():
                proc.terminate()
        for proc in processes.values():
            proc.join(timeout=3)

    try:
        # Supervise. Closing the UI window quits the whole app (it does NOT get
        # respawned — that caused the "won't stay closed / keeps reopening" bug).
        # Engine/HUD are restarted only if they crash unexpectedly.
        while True:
            time.sleep(1.0)
            if not processes["UI"].is_alive():
                logging.info("UI closed — exiting UltraPilot.")
                shutdown()
                break
            for name in [n for n in ("Engine", "HUD", "AR") if n in processes]:
                p = processes[name]
                if not p.is_alive():
                    if restart_guard.allow_restart(name):
                        logging.warning(
                            f"Process {name} crashed (code {p.exitcode}) — restarting.")
                        processes[name] = spawn(name)
                    else:
                        message = (
                            f"Process {name} repeatedly crashed (code {p.exitcode}); "
                            "automatic restarts stopped. Repair the installation "
                            "and inspect ultrapilot.log.")
                        logging.critical(message)
                        shared_dict["process_failure"] = {
                            "process": name, "exit_code": p.exitcode,
                            "message": message,
                        }
                        processes.pop(name, None)
    except KeyboardInterrupt:
        shutdown()


if __name__ == "__main__":
    mp.freeze_support()
    main()
