import multiprocessing as mp
import sys
import os
import time
import logging

# Ensure the project root is in path before importing project modules.
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


def run_engine(shared_dict):
    """Process for the Autopilot Engine."""
    logging.basicConfig(level=logging.INFO)
    logging.info("Launching Engine Process...")
    from core.engine import UltraPilotEngine
    engine = UltraPilotEngine(shared_dict)
    engine.start()


def run_ui(shared_dict):
    """Process for the Main Control Panel UI."""
    logging.basicConfig(level=logging.INFO)
    logging.info("Launching UI Process...")
    from PyQt6.QtWidgets import QApplication
    from ui.app import UltraPilotApp
    from core.ipc.shared_state import SharedState

    app = QApplication(sys.argv)
    window = UltraPilotApp(SharedState(shared_dict))
    window.show()
    sys.exit(app.exec())


def run_hud(shared_dict):
    """Process for the transparent HUD overlay."""
    logging.basicConfig(level=logging.INFO)
    from core.hud import run_hud as _run_hud
    from core.ipc.shared_state import SharedState
    _run_hud(SharedState(shared_dict))


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
        "Engine": run_engine,
        "UI": run_ui,
        "HUD": run_hud,
    }

    def spawn(name):
        p = mp.Process(target=targets[name], args=(shared_dict,), name=name)
        p.start()
        logging.info(f"Process {name} started (PID: {p.pid})")
        return p

    processes = {name: spawn(name) for name in targets}

    def shutdown():
        logging.info("Shutting down UltraPilot…")
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
            for name in ("Engine", "HUD"):
                p = processes[name]
                if not p.is_alive():
                    logging.warning(f"Process {name} crashed (code {p.exitcode}) — restarting.")
                    processes[name] = spawn(name)
    except KeyboardInterrupt:
        shutdown()


if __name__ == "__main__":
    mp.freeze_support()
    main()
