import logging
import os
import re

try:
    import pydirectinput
    pydirectinput.FAILSAFE = False
    _HAS_PDI = True
except Exception:  # not installed / non-Windows
    pydirectinput = None
    _HAS_PDI = False

from core.sdk.scs_controller_writer import SCSControlsWriter
from core.sdk.virtual_joystick import VirtualJoystick


_SCS_TO_PDI_KEY = {
    "lbracket": "[", "rbracket": "]", "semicolon": ";",
    "apostrophe": "'", "comma": ",", "period": ".", "slash": "/",
    "backslash": "\\", "minus": "-", "equals": "=", "space": "space",
}


def _discover_blinker_keys(documents_dir=None):
    """Return the active ETS2 profile's physical left/right indicator keys.

    ETS2 stores the selected profile name in ``game.log.txt`` and its control
    expressions below a UTF-8-hex profile directory.  Reading the expressions
    avoids assuming the default ``[``/``]`` keys; wheel users commonly bind
    them to completely different keyboard keys as a secondary input.

    This is deliberately read-only.  Missing or non-keyboard bindings return
    an empty mapping so the caller can use the semantic SCS input instead.
    """
    root = documents_dir or os.path.join(
        os.path.expanduser("~"), "Documents", "Euro Truck Simulator 2")
    log_path = os.path.join(root, "game.log.txt")
    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as stream:
            log_text = stream.read()
    except OSError:
        return {}
    matches = re.findall(r"Set profile finished:\s*'([^']+)'", log_text)
    if not matches:
        return {}
    encoded = matches[-1].encode("utf-8").hex().upper()
    controls_path = None
    for directory in ("steam_profiles", "profiles"):
        candidate = os.path.join(root, directory, encoded, "controls.sii")
        if os.path.isfile(candidate):
            controls_path = candidate
            break
    if not controls_path:
        return {}
    try:
        with open(controls_path, "r", encoding="utf-8", errors="replace") as stream:
            controls = stream.read()
    except OSError:
        return {}

    result = {}
    for side, mix_name in (("left", "lblinker"), ("right", "rblinker")):
        expression = re.search(
            rf'"mix\s+{mix_name}\s+`([^`]*)`', controls, re.IGNORECASE)
        if not expression:
            continue
        key = re.search(r"keyboard\.([A-Za-z0-9_]+)\?", expression.group(1))
        if key:
            token = key.group(1).lower()
            result[side] = _SCS_TO_PDI_KEY.get(token, token)
    return result


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
        self._scs_blinker_button = None
        self._blinker_keys = _discover_blinker_keys()
        if self._blinker_keys:
            logging.info("ETS2 profile blinker keys: left=%s right=%s",
                         self._blinker_keys.get("left", "unbound"),
                         self._blinker_keys.get("right", "unbound"))

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
        if side not in ("left", "right", "off"):
            side = "off"

        # Prefer the key actually configured by the active ETS2 profile. The
        # controller DLL exposes semantic indicator fields, but unlike its
        # analog axes they are not consumed by every live game/input setup.
        # A profile-resolved key works with keyboard and wheel profiles alike.
        button = side if side in ("left", "right") else self.current_blinker
        key = getattr(self, "_blinker_keys", {}).get(button)
        if _HAS_PDI and key:
            if side == self.current_blinker:
                return
            logging.info("Blinker: %s", side)
            pydirectinput.press(key)
            self.current_blinker = side
            return

        if self.mode == "SCS_SDK":
            # A blinker control is a button edge, not a persistent state. Hold
            # True for one engine frame, release it on the next, and only send
            # another edge when the requested side changes.
            if self._scs_blinker_button is not None:
                if self._scs_blinker_button == "left":
                    self.scs.set_left_blinker(False)
                else:
                    self.scs.set_right_blinker(False)
                self._scs_blinker_button = None
            if side == self.current_blinker:
                return
            logging.info("Blinker: %s", side)
            if button == "left":
                self.scs.set_left_blinker(True)
                self._scs_blinker_button = "left"
            elif button == "right":
                self.scs.set_right_blinker(True)
                self._scs_blinker_button = "right"
            self.current_blinker = side
            return
        if side == self.current_blinker:
            return
        logging.info(f"Blinker: {side}")
        if not _HAS_PDI:
            self.current_blinker = side
            return
        left_key = getattr(self, "_blinker_keys", {}).get("left", "[")
        right_key = getattr(self, "_blinker_keys", {}).get("right", "]")
        if side == "left":
            pydirectinput.press(left_key)
        elif side == "right":
            pydirectinput.press(right_key)
        else:  # off -> press the currently-active side again to cancel
            if self.current_blinker == "left":
                pydirectinput.press(left_key)
            elif self.current_blinker == "right":
                pydirectinput.press(right_key)
        self.current_blinker = side

    def select_drive(self, pressed: bool = True):
        """Select Drive explicitly instead of using the brake/reverse gesture."""
        if self.mode == "SCS_SDK":
            return (self.scs.select_drive() if pressed
                    else self.scs.release_drive())
        return False

    def stop_completely(self):
        self.set_throttle(0.0)
        self.set_brake(1.0)

    def pay_toll(self):
        logging.info("Paying toll...")
        if _HAS_PDI:
            # Keep the interaction key separate from ETS2's E ignition key.
            # Using E here was able to switch off a running engine at a false
            # toll detection; Enter confirms the toll/menu interaction safely.
            pydirectinput.press('enter')

    def release_all(self):
        """Release every input — used on shutdown / when autopilot turns off."""
        if self.mode in ("SCS_SDK", "VJOY"):
            self.set_steering(0.0)
            self.set_throttle(0.0)
            self.set_brake(0.0)
            if self.mode == "SCS_SDK":
                # Do not leave the momentary gearbox button latched when the
                # master switch changes between its press/release frames.
                self.scs.release_drive()
        elif self.mode == "DIGITAL" and _HAS_PDI:
            for key in list(self._keys_down):
                pydirectinput.keyUp(key)
            self._keys_down.clear()
