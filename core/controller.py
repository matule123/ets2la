import logging

try:
    import pydirectinput
    pydirectinput.FAILSAFE = False
    _HAS_PDI = True
except Exception:  # not installed / non-Windows
    pydirectinput = None
    _HAS_PDI = False

from core.sdk.scs_controller_writer import SCSControlsWriter
from core.sdk.virtual_joystick import VirtualJoystick


def _scs_dll_installed() -> bool:
    """True if the SCS controller plugin DLL is installed in any detected game."""
    try:
        import os
        from core.sdk.game_utils import find_scs_games
        for g in find_scs_games():
            if os.path.exists(os.path.join(g, "bin", "win_x64", "plugins",
                                            "scs_sdk_controller.dll")):
                return True
    except Exception:
        pass
    return False


class Controller:
    """
    Advanced input controller for ETS2.

    Priority: SCS SDK plugin (Local\\SCSControls) -> virtual joystick (vgamepad)
    -> digital keys.

    The SCS SDK path writes steering/throttle/brake straight into the game, so it
    works *alongside a real wheel (e.g. G29)* and turns the in-game wheel, without
    creating a virtual Xbox controller.  It only attaches once the game is
    running (the DLL creates the shared memory), so the writer reconnects lazily.
    Only the Engine process owns a Controller; plugins write intents instead.
    """

    def __init__(self):
        logging.info("Initializing Professional Controller...")

        self.scs = SCSControlsWriter()
        self.vjoy = None

        # Prefer SCS SDK when its plugin is connected now OR installed (it will
        # attach as soon as the game starts).  This avoids spawning a virtual
        # Xbox controller when the user drives a real wheel.
        if self.scs.connected or _scs_dll_installed():
            self.mode = "SCS_SDK"
            logging.info("Control Mode: [SCS SDK] (writes to the game; real wheel stays usable)")
        else:
            self.vjoy = VirtualJoystick()
            if self.vjoy.gamepad:
                self.mode = "VJOY"
                logging.info("Control Mode: [ANALOG VIRTUAL JOYSTICK]")
            elif _HAS_PDI:
                self.mode = "DIGITAL"
                logging.warning("Control Mode: [DIGITAL FALLBACK] - Steering will be jerky")
            else:
                self.mode = "NONE"
                logging.error("No control backend available (no SCS DLL, vgamepad or pydirectinput).")

        # Track digital key state so we don't spam keyDown/keyUp.
        self._keys_down = set()
        self.current_blinker = "off"

    # --- Digital helpers ------------------------------------------------------
    def _key(self, key: str, down: bool):
        if not _HAS_PDI:
            return
        if down and key not in self._keys_down:
            pydirectinput.keyDown(key)
            self._keys_down.add(key)
        elif not down and key in self._keys_down:
            pydirectinput.keyUp(key)
            self._keys_down.discard(key)

    # --- Analog/precise control ----------------------------------------------
    def set_steering(self, value: float):
        value = max(-1.0, min(1.0, value))
        if self.mode == "SCS_SDK":
            self.scs.set_steering(value)
        elif self.mode == "VJOY":
            self.vjoy.set_steering(value)
        elif self.mode == "DIGITAL":
            self._key('a', value < -0.1)
            self._key('d', value > 0.1)

    def set_throttle(self, value: float):
        value = max(0.0, min(1.0, value))
        if self.mode == "SCS_SDK":
            self.scs.set_throttle(value)
        elif self.mode == "VJOY":
            self.vjoy.set_throttle(value)
        elif self.mode == "DIGITAL":
            self._key('w', value > 0.1)

    def set_brake(self, value: float):
        value = max(0.0, min(1.0, value))
        if self.mode == "SCS_SDK":
            self.scs.set_brake(value)
        elif self.mode == "VJOY":
            self.vjoy.set_brake(value)
        elif self.mode == "DIGITAL":
            self._key('s', value > 0.1)

    def set_blinker(self, side: str):
        """side: 'left', 'right' or 'off'. Tracks state so 'off' actually cancels."""
        if side == self.current_blinker:
            return
        logging.info(f"Blinker: {side}")
        if not _HAS_PDI:
            self.current_blinker = side
            return
        if side == "left":
            pydirectinput.press('[')
        elif side == "right":
            pydirectinput.press(']')
        else:  # off -> press the currently-active side again to cancel
            if self.current_blinker == "left":
                pydirectinput.press('[')
            elif self.current_blinker == "right":
                pydirectinput.press(']')
        self.current_blinker = side

    def stop_completely(self):
        self.set_throttle(0.0)
        self.set_brake(1.0)

    def pay_toll(self):
        logging.info("Paying toll...")
        if _HAS_PDI:
            pydirectinput.press('e')

    def release_all(self):
        """Release every input — used on shutdown / when autopilot turns off."""
        if self.mode in ("SCS_SDK", "VJOY"):
            self.set_steering(0.0)
            self.set_throttle(0.0)
            self.set_brake(0.0)
        elif self.mode == "DIGITAL" and _HAS_PDI:
            for key in list(self._keys_down):
                pydirectinput.keyUp(key)
            self._keys_down.clear()
