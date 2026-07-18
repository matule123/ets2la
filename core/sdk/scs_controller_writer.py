"""
Direct game control via the SCS SDK controller plugin (scs_sdk_controller.dll).

The plugin maps a named shared-memory block ``Local\\SCSControls`` onto the
game's input.  We write the truck's steering / throttle / brake into that block
and the game applies them — *while a real wheel (e.g. G29) stays connected* and
the in-game wheel turns with the autopilot.  No virtual Xbox controller is
created (unlike the vgamepad backend).

The field layout below is the exact ordered struct the plugin expects: each
bool is 1 byte, each float 4 bytes, laid out sequentially.  Offsets are computed
from this order, so it MUST match the DLL — it is copied verbatim from the
reference plugin's control schema.  Writing to the wrong offset would send the
wrong input, so the order is intentionally fixed.
"""

import logging
import struct

# (name, 'f' for float / '?' for bool) in the exact order the plugin lays them
# out in Local\SCSControls.  Only steering/aforward/abackward/clutch are used by
# UltraPilot, but every preceding field must be present so their offsets line up.
_FIELDS = [
    ("j_left", "f"), ("j_right", "f"), ("j_up", "f"), ("j_down", "f"),
    ("selectfcs", "?"), ("back", "?"), ("skip", "?"), ("scrol_up", "?"),
    ("scrol_dwn", "?"), ("mapzoom_in", "?"), ("mapzoom_out", "?"),
    ("trs_zoom_in", "?"), ("trs_zoom_out", "?"), ("joy_nav_prv", "?"),
    ("joy_nav_nxt", "?"), ("joy_sec_prv", "?"), ("joy_sec_nxt", "?"),
    ("scroll_j_x", "f"), ("scroll_j_y", "f"),
    ("shortcut_1", "?"), ("shortcut_2", "?"), ("shortcut_3", "?"),
    ("shortcut_4", "?"), ("pause", "?"), ("screenshot", "?"),
    ("cam1", "?"), ("cam2", "?"), ("cam3", "?"), ("cam4", "?"), ("cam5", "?"),
    ("cam6", "?"), ("cam7", "?"), ("cam8", "?"), ("camcycle", "?"),
    ("camreset", "?"), ("camrotate", "?"), ("camzoomin", "?"),
    ("camzoomout", "?"), ("camzoom", "?"), ("camfwd", "?"), ("camback", "?"),
    ("camleft", "?"), ("camright", "?"), ("camup", "?"), ("camdown", "?"),
    ("lookleft", "?"), ("lookright", "?"), ("camlr", "?"), ("camud", "?"),
    ("j_cam_lk_lr", "f"), ("j_cam_lk_ud", "f"), ("j_cam_mv_lr", "f"),
    ("j_cam_mv_ud", "f"), ("j_trzoom_in", "f"), ("j_trzoom_out", "f"),
    ("j_mappan_x", "f"), ("j_mappan_y", "f"), ("j_mapzom_in", "f"),
    ("j_mapzom_out", "f"),
    ("lookpos1", "?"), ("lookpos2", "?"), ("lookpos3", "?"), ("lookpos4", "?"),
    ("lookpos5", "?"), ("lookpos6", "?"), ("lookpos7", "?"), ("lookpos8", "?"),
    ("lookpos9", "?"), ("looksteer", "?"), ("lookblink", "?"),
    ("steering", "f"), ("aforward", "f"), ("abackward", "f"), ("clutch", "f"),
    ("activate", "?"), ("menu", "?"), ("ignitionoff", "?"), ("ignitionon", "?"),
    ("ignitionstrt", "?"), ("attach", "?"), ("frontsuspup", "?"),
    ("frontsuspdwn", "?"), ("rearsuspup", "?"), ("rearsuspdwn", "?"),
    ("suspreset", "?"), ("horn", "?"), ("airhorn", "?"), ("lighthorn", "?"),
    ("beacon", "?"), ("motorbrake", "?"), ("engbraketog", "?"),
    ("engbrakeup", "?"), ("engbrakedwn", "?"), ("trailerbrake", "?"),
    ("retarderup", "?"), ("retarderdown", "?"), ("retarder0", "?"),
    ("retarder1", "?"), ("retarder2", "?"), ("retarder3", "?"),
    ("retarder4", "?"), ("retarder5", "?"), ("liftaxle", "?"),
    ("liftaxlet", "?"), ("slideaxlefwd", "?"), ("slideaxlebwd", "?"),
    ("slideaxleman", "?"), ("diflock", "?"), ("rwinopen", "?"),
    ("rwinclose", "?"), ("lwinopen", "?"), ("lwinclose", "?"),
    ("engbrakeauto", "?"), ("retarderauto", "?"), ("embrake", "?"),
    ("laneassmode", "?"), ("tranpwrmode", "?"), ("parkingbrake", "?"),
    ("wipers", "?"), ("wipersback", "?"), ("wipers0", "?"), ("wipers1", "?"),
    ("wipers2", "?"), ("wipers3", "?"), ("wipers4", "?"), ("cruiectrl", "?"),
    ("cruiectrlinc", "?"), ("cruiectrldec", "?"), ("cruiectrlres", "?"),
    ("accmode", "?"), ("laneassist", "?"), ("light", "?"), ("lightoff", "?"),
    ("lightpark", "?"), ("lighton", "?"), ("hblight", "?"), ("lblinker", "?"),
    ("lblinkerh", "?"), ("rblinker", "?"), ("rblinkerh", "?"),
    ("flasher4way", "?"), ("showmirrors", "?"), ("showhud", "?"), ("navmap", "?"),
    ("photo_mode", "?"), ("quicksave", "?"), ("quickload", "?"), ("radio", "?"),
    ("radionext", "?"), ("radioprev", "?"), ("radioup", "?"), ("radiodown", "?"),
    ("display", "?"), ("quickpark", "?"), ("dashmapzoom", "?"), ("tripreset", "?"),
    ("sb_activate", "?"), ("sb_swap", "?"), ("infotainment", "?"),
    ("photores", "?"), ("photomove", "?"), ("phototake", "?"), ("photofwd", "?"),
    ("photobwd", "?"), ("photoleft", "?"), ("photoright", "?"), ("photoup", "?"),
    ("photodown", "?"), ("photorolll", "?"), ("photorollr", "?"),
    ("photosman", "?"), ("photo_opts", "?"), ("photosnap", "?"),
    ("photo_hctrl", "?"), ("photonames", "?"), ("photozoomout", "?"),
    ("photozoomin", "?"), ("phot_z_j_out", "f"), ("phot_z_j_in", "f"),
    ("album_pgup", "?"), ("album_pgdn", "?"), ("album_itup", "?"),
    ("album_itdn", "?"), ("album_itlf", "?"), ("album_itrg", "?"),
    ("album_ithm", "?"), ("album_iten", "?"), ("album_itac", "?"),
    ("album_itop", "?"), ("album_itdl", "?"), ("camwalk_for", "?"),
    ("camwalk_back", "?"), ("camwalk_righ", "?"), ("camwalk_left", "?"),
    ("camwalk_run", "?"), ("camwalk_jump", "?"), ("camwalk_crou", "?"),
    ("camwalk_lr", "?"), ("camwalk_ud", "?"), ("gearup", "?"), ("geardown", "?"),
    ("gear0", "?"), ("geardrive", "?"), ("gearreverse", "?"), ("gearuphint", "?"),
    ("geardownhint", "?"), ("transemi", "?"), ("drive", "?"), ("reverse", "?"),
    ("cmirrorsel", "?"), ("fmirrorsel", "?"), ("mirroryawl", "?"),
    ("mirroryawr", "?"), ("mirrorpitu", "?"), ("mirrorpitl", "?"),
    ("mirrorreset", "?"), ("quicksel1", "?"), ("quicksel2", "?"),
    ("quicksel3", "?"), ("quicksel4", "?"), ("quicksel5", "?"), ("quicksel6", "?"),
    ("quicksel7", "?"), ("quicksel8", "?"), ("mpptt", "?"), ("replayhidec", "?"),
    ("gearsel1on", "?"), ("gearsel1off", "?"), ("gearsel1tgl", "?"),
    ("gearsel2on", "?"), ("gearsel2off", "?"), ("gearsel2tgl", "?"),
    ("gear1", "?"), ("gear2", "?"), ("gear3", "?"), ("gear4", "?"), ("gear5", "?"),
    ("gear6", "?"), ("gear7", "?"), ("gear8", "?"), ("gear9", "?"), ("gear10", "?"),
    ("gear11", "?"), ("gear12", "?"), ("gear13", "?"), ("gear14", "?"),
    ("gear15", "?"), ("gear16", "?"), ("adjuster", "?"), ("advpage0", "?"),
    ("advpage1", "?"), ("advpage2", "?"), ("advpage3", "?"), ("advpage4", "?"),
    ("advpagen", "?"), ("advpagep", "?"), ("advmouse", "?"), ("advetamode", "?"),
    ("gar_man", "?"), ("advzoomin", "?"), ("advzoomout", "?"), ("assistact1", "?"),
    ("assistact2", "?"), ("assistact3", "?"), ("assistact4", "?"),
    ("assistact5", "?"), ("adj_seats", "?"), ("adj_mirrors", "?"),
    ("adj_lights", "?"), ("adj_uimirror", "?"), ("chat_act", "?"),
    ("quick_chat", "?"), ("cycl_zoom", "?"), ("name_tags", "?"),
    ("headreset", "?"), ("menustereo", "?"),
]

_SIZE = {"f": struct.calcsize("f"), "?": struct.calcsize("?")}


class SCSControlsWriter:
    """Writes steering/throttle/brake into Local\\SCSControls (Windows)."""

    MEM_NAME = r"Local\SCSControls"

    def __init__(self, invert_steering: bool = False):
        self.invert_steering = invert_steering
        self.connected = False
        self._buf = None

        # Compute byte offset of every field from the ordered layout.
        self._offsets = {}
        total = 0
        for name, t in _FIELDS:
            self._offsets[name] = total
            total += _SIZE[t]
        self._total = total
        self._retry = 0  # throttle reconnect attempts when the game isn't up yet

        self._connect()

    def _connect(self):
        try:
            import mmap
            # tagname-only mmap maps the existing named block created by the DLL.
            self._buf = mmap.mmap(0, self._total, self.MEM_NAME)
            self.connected = True
            logging.info("SCS SDK controller: connected to %s (%d bytes).",
                         self.MEM_NAME, self._total)
        except Exception as e:
            self.connected = False
            self._buf = None
            logging.info("SCS SDK controller unavailable (%s). "
                         "Is the game running with scs_sdk_controller.dll installed?", e)

    def _maybe_reconnect(self):
        """Retry connecting roughly every ~2s of calls until the game is up."""
        if self.connected:
            return
        self._retry += 1
        if self._retry % 120 == 0:
            self._connect()

    def _write_float(self, name: str, value: float):
        if not self.connected:
            self._maybe_reconnect()
            return
        try:
            self._buf.seek(self._offsets[name])
            self._buf.write(struct.pack("f", float(value)))
            self._buf.flush()
        except Exception as e:
            logging.error("SCS SDK write %s failed: %s", name, e)
            self.connected = False

    def _write_bool(self, name: str, value: bool):
        if not self.connected:
            self._maybe_reconnect()
            return False
        try:
            self._buf.seek(self._offsets[name])
            self._buf.write(struct.pack("?", bool(value)))
            self._buf.flush()
            return True
        except Exception as e:
            logging.error("SCS SDK write %s failed: %s", name, e)
            self.connected = False
            return False

    def select_drive(self):
        return self._write_bool("geardrive", True)

    def release_drive(self):
        return self._write_bool("geardrive", False)

    # --- Public API (mirrors the other control backends) ---------------------
    def set_steering(self, value: float):
        v = max(-1.0, min(1.0, value))
        if self.invert_steering:
            v = -v
        self._write_float("steering", v)

    def set_throttle(self, value: float):
        self._write_float("aforward", max(0.0, min(1.0, value)))

    def set_brake(self, value: float):
        self._write_float("abackward", max(0.0, min(1.0, value)))

    def reset(self):
        for name in ("steering", "aforward", "abackward", "clutch"):
            self._write_float(name, 0.0)

    def close(self):
        try:
            if self._buf is not None:
                self.reset()
                self._buf.close()
        except Exception:
            pass
