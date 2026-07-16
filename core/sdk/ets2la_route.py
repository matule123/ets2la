"""Read the route exported by ETS2LA's ``Local\\ETS2LARoute`` mapping."""

import logging
import mmap
import struct
import time


class ETS2LARouteReader:
    SIZE = 96_000
    ITEM = struct.Struct("=qff")  # node uid, world x, world z

    def __init__(self):
        self._mm = None
        self._last_read = 0.0
        self._cached = []

    def _connect(self):
        if self._mm is not None:
            return True
        try:
            self._mm = mmap.mmap(0, self.SIZE, r"Local\ETS2LARoute")
            logging.info("Navigation: connected to ETS2LA planned-route buffer.")
            return True
        except Exception:
            return False

    def read(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_read < 0.5:
            return self._cached
        self._last_read = now
        if not self._connect():
            return []
        try:
            raw = self._mm[:self.SIZE]
            points = []
            for uid, x, z in self.ITEM.iter_unpack(raw):
                if uid == 0:
                    break
                points.append((float(x), float(z)))
            self._cached = points
            return points
        except Exception as e:
            logging.debug("Navigation route buffer read failed: %s", e)
            try:
                self._mm.close()
            except Exception:
                pass
            self._mm = None
            return []
