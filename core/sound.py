"""
Tiny sound helper for UltraPilot.

Looks for ``assets/sounds/<name>.mp3`` (or .wav) and plays it. If no file is
present — which is the default, since we ship no audio assets — it does nothing.
Playback runs on a background thread so it can never stall the UI.

Backend preference on Windows: ``winsound`` for .wav (built-in, instant), then
``playsimple``/``pygame.mixer`` for .mp3 if available. Anything missing is a
silent no-op, never an error.

Drop a file at ``assets/sounds/boot.mp3`` to hear a chime when the app starts
(``bootloader.py`` calls ``play("boot")`` after the UI is up).
"""

import logging
import os
import threading

from core.paths import resource

_EXTS = (".mp3", ".wav")


def _find(name: str) -> str:
    """Resolve a sound name to a file path, or '' if none exists."""
    for ext in _EXTS:
        try:
            p = resource("assets", "sounds", name + ext)
        except Exception:
            p = os.path.join(resource("assets"), "sounds", name + ext)
        if p and os.path.exists(p):
            return p
    return ""


def _play_wav(path: str):
    try:
        import winsound
        winsound.PlaySound(path, winsound.SND_FILENAME | winsound.SND_ASYNC)
        return True
    except Exception:
        return False


def _play_mp3(path: str):
    # Try pygame first (common in this project's deps), then playsound.
    try:
        import pygame
        if not getattr(pygame, "_snd_init", False):
            try:
                pygame.mixer.init()
            except Exception:
                pass
            pygame._snd_init = True
        pygame.mixer.music.load(path)
        pygame.mixer.music.play()
        return True
    except Exception:
        pass
    try:
        from playsound import playsound
        playsound(path, block=False)
        return True
    except Exception:
        return False


def play(name: str, volume: float = 1.0) -> bool:
    """Play ``assets/sounds/<name>.{mp3,wav}`` asynchronously.

    Returns True if playback started, False if the file is missing or no backend
    is available. Never raises."""
    path = _find(name)
    if not path:
        return False

    def _run():
        try:
            ok = False
            if path.lower().endswith(".wav"):
                ok = _play_wav(path)
            if not ok:
                ok = _play_mp3(path)
            if not ok:
                logging.debug("sound: no backend for %s", path)
        except Exception as e:
            logging.debug("sound: %s failed: %s", name, e)

    threading.Thread(target=_run, daemon=True).start()
    return True
