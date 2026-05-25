import os
import sys
import subprocess
import requests
import zipfile
import shutil
from pathlib import Path

# Configuration
REPO_URL = "https://github.com/matule123/ets2la.git"
ASSETS_URL = "https://github.com/matule123/ets2la/releases/latest/download/assets.zip"
REQUIRED_PACKAGES = ["pyqt6", "opencv-python", "numpy", "mss", "pydirectinput", "textual", "requests", "pyautogui", "vgamepad", "pyinstaller", "torch", "torchvision", "beautifulsoup4"]

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
        # Clone into a temporary directory to avoid "directory not empty" errors
        try:
            if not run_command(f"git clone {REPO_URL} temp_repo"):
                log("Failed to clone repository", "ERROR")
                return False

            # Move contents from temp_repo to current directory
            for item in os.listdir("temp_repo"):
                shutil.move(os.path.join("temp_repo", item), ".")

            shutil.rmtree("temp_repo")
            log("Repository cloned and files moved successfully.")
        except Exception as e:
            log(f"Unexpected error during repository setup: {str(e)}", "ERROR")
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

def build_executable():
    log("Building executable (.exe) with PyInstaller...")
    try:
        # We use --onefile for a single exe, --noconsole to hide the terminal,
        # and --collect-all to make sure torch and other complex packages are bundled.
        # Note: torch is huge, so we might need specific hooks.
        cmd = [
            "pyinstaller",
            "--noconsole",
            "--onefile",
            "--name", "ETS2_UltraPilot",
            "--collect-all", "torch",
            "--collect-all", "torchvision",
            "main.py"
        ]

        if not run_command(" ".join(cmd)):
            log("PyInstaller failed to build the executable.", "ERROR")
            return False

        # Copy assets and sdk to the dist folder so the exe can find them
        dist_folder = "dist"
        if os.path.exists(dist_folder):
            if os.path.exists("sdk"):
                shutil.copytree("sdk", os.path.join(dist_folder, "sdk"), dirs_exist_ok=True)
            if os.path.exists("assets"):
                shutil.copytree("assets", os.path.join(dist_folder, "assets"), dirs_exist_ok=True)
            log("Assets copied to distribution folder.")

        log("Executable created successfully in the 'dist' folder!")
        return True
    except Exception as e:
        log(f"Build error: {str(e)}", "ERROR")
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

    if not build_executable():
        print("\n[!] Failed to create executable. You can still run the project via main.py")

    print("\n==========================================")
    print("   Installation Complete!              ")
    print("   Your app is ready in the 'dist' folder ")
    print("==========================================\n")
    input("Press Enter to finish...")

if __name__ == "__main__":
    main()
