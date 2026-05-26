import os
import sys
import subprocess
from pathlib import Path

# This script is used to compile the installer.py into a professional .exe
# It ensures the correct icon and name are used.

def build_installer():
    print("--- Building UltraPilot Installer (.exe) ---")

    # Configuration
    script_path = "installer.py"
    icon_path = "assets/favicon.ico"
    output_name = "ets2la_installer"

    if not os.path.exists(script_path):
        print(f"Error: {script_path} not found!")
        return

    if not os.path.exists(icon_path):
        print(f"Warning: Icon not found at {icon_path}, building without icon.")
        icon_arg = ""
    else:
        icon_arg = f"--icon={icon_path}"

    cmd = [
        "pyinstaller",
        "--noconsole",
        "--onefile",
        f"--name={output_name}",
        icon_arg,
        script_path
    ]

    print(f"Running: {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=True)
        print(f"\nSuccess! Installer built as dist/{output_name}.exe")
        print("You can now move this file to your desired location.")
    except subprocess.CalledProcessError as e:
        print(f"Build failed: {e}")

if __name__ == "__main__":
    build_installer()
