import multiprocessing as mp
import sys
import os
import logging
from core.engine import UltraPilotEngine
from ui.app import UltraPilotApp
from core.hud import run_hud
from PyQt6.QtWidgets import QApplication

# Ensure the project root is in path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def run_engine(shared_state):
    """Process for the Autopilot Engine."""
    logging.info("Launching Engine Process...")
    engine = UltraPilotEngine()
    # Override the engine's shared state with the one from the bootloader
    engine.shared_state = shared_state
    engine.start()

def run_ui(shared_state):
    """Process for the Main Control Panel UI."""
    logging.info("Launching UI Process...")
    app = QApplication(sys.argv)
    window = UltraPilotApp(None) # Engine is not needed here, we use shared_state
    # Inject shared_state into the window
    window.shared_state = shared_state
    window.show()
    sys.exit(app.exec())

def main():
    logging.basicConfig(level=logging.INFO)
    logging.info("UltraPilot Bootloader starting...")

    # 1. Create a Shared Manager for IPC
    manager = mp.Manager()
    shared_state = manager.dict()

    # 2. Define Processes
    processes = [
        mp.Process(target=run_engine, args=(shared_state,), name="Engine"),
        mp.Process(target=run_ui, args=(shared_state,), name="UI"),
        mp.Process(target=run_hud, args=(shared_state,), name="HUD"),
    ]

    # 3. Start all processes
    for p in processes:
        p.start()
        logging.info(f"Process {p.name} started (PID: {p.pid})")

    try:
        # Keep the bootloader alive while processes are running
        for p in processes:
            p.join()
    except KeyboardInterrupt:
        logging.info("Bootloader shutting down...")
        for p in processes:
            p.terminate()
            p.join()

if __name__ == "__main__":
    main()
