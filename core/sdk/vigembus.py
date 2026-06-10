"""
ViGEmBus driver detection / installation.

ViGEmBus is the kernel driver that ``vgamepad`` needs to create a virtual
controller.  With the SCS SDK control backend it's only required for the
vgamepad *fallback*, but we still offer to install it.

``ensure_vigembus()`` checks whether the driver is present and, if not, runs the
bundled installer from ``assets/`` silently.  It is a best-effort no-op when the
driver is already present or the installer binary isn't shipped.
"""

import os
import logging
import subprocess


def is_installed() -> bool:
    """True if the ViGEmBus driver service is registered, or vgamepad can init."""
    # 1) Registry service check (fast, no side effects).
    try:
        import winreg
        try:
            winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                           r"SYSTEM\CurrentControlSet\Services\ViGEmBus")
            return True
        except FileNotFoundError:
            pass
    except Exception:
        pass

    # 2) Functional check: can vgamepad actually create a device?
    try:
        import vgamepad as vg
        g = vg.VX360Gamepad()
        del g
        return True
    except Exception:
        return False


def _find_installer(assets_dir: str):
    """Locate a bundled ViGEmBus installer in assets/ (exe or msi)."""
    if not assets_dir or not os.path.isdir(assets_dir):
        return None
    for name in os.listdir(assets_dir):
        low = name.lower()
        if "vigembus" in low and low.endswith((".exe", ".msi")):
            return os.path.join(assets_dir, name)
    return None


def ensure_vigembus(assets_dir: str, log=logging.info) -> bool:
    """
    Make sure ViGEmBus is installed.  Returns True if present (or installed now).

    Runs the bundled installer silently if found.  ``.msi`` is installed via
    ``msiexec /i ... /qn``; ``.exe`` via ``/S`` (the ViGEmBus NSIS silent flag).
    """
    if os.name != "nt":
        return False
    if is_installed():
        log("ViGEmBus driver already present.")
        return True

    installer = _find_installer(assets_dir)
    if not installer:
        log("ViGEmBus not installed and no bundled installer found in assets/. "
            "Virtual-gamepad fallback will be unavailable (SCS SDK control still works). "
            "Download it from https://github.com/ViGEm/ViGEmBus/releases")
        return False

    try:
        if installer.lower().endswith(".msi"):
            cmd = ["msiexec", "/i", installer, "/qn", "/norestart"]
        else:
            cmd = [installer, "/S"]  # NSIS silent install
        log(f"Installing ViGEmBus from {os.path.basename(installer)}…")
        subprocess.run(cmd, check=True)
        log("ViGEmBus installed.")
        return True
    except Exception as e:
        logging.error(f"ViGEmBus install failed: {e}")
        return False
