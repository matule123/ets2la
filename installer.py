import os
import sys
import subprocess
import requests
import zipfile
import shutil
from pathlib import Path

# Configuration
REPO_URL = "https://github.com/your-repo/ETS2-UltraPilot.git" # User will replace this
ASSETS_URL = "https://github.com/your-repo/ETS2-UltraPilot/releases/latest/download/assets.zip" # User will replace this
REQUIRED_PACKAGES = ["pyqt6", "opencv-python", "numpy", "mss", "pydirectinput", "textual", "requests", "pyautogui", "vgamepad"]

def log(message, level="INFO"):
    print(f"[{level}] {message}")

def run_command(command, shell=True):
    try:
        result = subprocess.run(command, shell=shell, capture_output=True, text=True)
        if result.returncode != 0:
            log(f"Command failed: {command}\nError: {result.stderr}", "ERROR")
            return False
        return True
    except Exception as e:
        log(f"Execution error: {str(e)}", "ERROR")
        return False

def install_dependencies():
    log("Installing required Python packages...")
    for package in REQUIRED_PACKAGES:
        log(f"Installing {package}...")
        if not run_command(f"pip install {package}"):
            log(f"Failed to install {package}", "ERROR")
            return False
    return True

def setup_git_repo():
    log("Setting up project repository...")
    if os.path.exists(".git"):
        log("Repository already exists. Updating...")
        run_command("git pull")
    else:
        log("Cloning repository...")
        if not run_command(f"git clone {REPO_URL} ."):
            log("Failed to clone repository", "ERROR")
            return False
    return True

def download_assets():
    log("Downloading required binary assets (DLLs, SDKs)...")
    try:
        response = requests.get(ASSETS_URL)
        if response.status_code == 200:
            with open("assets_temp.zip", "wb") as f:
                f.write(response.content)

            log("Extracting assets...")
            with zipfile.ZipFile("assets_temp.zip", "r") as zip_ref:
                zip_ref.extractall("sdk") # Extract to sdk folder

            os.remove("assets_temp.zip")
            log("Assets installed successfully.")
            return True
        else:
            log(f"Failed to download assets. HTTP {response.status_code}", "ERROR")
            return False
    except Exception as e:
        log(f"Asset download error: {str(e)}", "ERROR")
        return False

def main():
    print("==========================================")
    print("   ETS2-UltraPilot Professional Installer  ")
    print("==========================================\n")

    if not setup_git_repo():
        print("\n[!] Repository setup failed. Please check your internet connection.")
        input("Press Enter to exit...")
        sys.exit(1)

    if not install_dependencies():
        print("\n[!] Dependency installation failed. Some features may not work.")
        # Continue anyway, let the updater handle it later

    if not download_assets():
        print("\n[!] Binary assets could not be downloaded. SDK features will be disabled.")
        # Continue anyway

    print("\n==========================================")
    print("   Installation Complete!              ")
    print("   You can now run main.py               ")
    print("==========================================\n")
    input("Press Enter to finish...")

if __name__ == "__main__":
    main()
