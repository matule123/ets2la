import sys
import os

# Add the project root to sys.path so imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from ui.app import UltraPilotApp
from PyQt6.QtWidgets import QApplication
from core.engine import UltraPilotEngine

if __name__ == "__main__":
    import bootloader
    bootloader.main()
