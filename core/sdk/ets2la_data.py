"""
Reader for the ETS2LA game plugin's shared memory (ets2la_plugin.dll).

When the ETS2LA plugin is installed in the game it publishes two extra buffers
that the standard SCS telemetry plugin does not:

  * ``Local\\ETS2LATraffic``   — up to 40 surrounding vehicles (pos/rot/size/speed)
  * ``Local\\ETS2LASemaphore`` — up to 40 traffic lights / gates (pos/state/timer)

We read those so the HUD can draw the surrounding traffic and show the next
traffic light + countdown.  Everything degrades gracefully (returns []) when the
plugin or game isn't present.
"""

import math
import struct
import logging

try:
    import mmap
except Exception:
    mmap = None

# --- struct layouts (must match the plugin, copied from ETS2LA) ---------------
_VEH = "ffffffffffffhhbb"          # 12 floats + 2 shorts + 2 bytes
_TRL = "ffffffffff"                # one trailer (10 floats)
_VEH_OBJ = _VEH + _TRL * 3         # vehicle + 3 trailers = 46 values
_TRAFFIC_FMT = "=" + _VEH_OBJ * 40
_TRAFFIC_SIZE = 6960

_PARKED = "ffffffffffhb"         # pose + dimensions + id + trailer flag
_PARKED_FMT = "=" + _PARKED * 40
_PARKED_SIZE = 1720

_SEM = "fffhhffffifii"             # one semaphore (13 values)
_SEM_FMT = "=" + _SEM * 40
_SEM_SIZE = 1920

# Traffic-light state codes (from ETS2LA).
ST_OFF, ST_Y2R, ST_RED, ST_Y2G, ST_GREEN, ST_SLEEP = 0, 1, 2, 4, 8, 32


def _yaw(q0, q1, q2, q3):
    """Yaw from ETS2LA's quaternion memory order (w, y, x, z)."""
    try:
        w, x, y, z = q0, q2, q1, q3
        return math.atan2(2.0 * (y * z + w * x),
                          w * w - x * x - y * y + z * z)
    except Exception:
        return 0.0


def _veh_type(length):
    if length < 6.0:
        return "car"
    if length < 9.5:
        return "van"
    if length < 14.0:
        return "bus"
    return "truck"


class ETS2LAData:
    """Lazily-connecting reader for the ETS2LA traffic + semaphore buffers."""

    def __init__(self):
        self._traffic_buf = None
        self._parked_buf = None
        self._sem_buf = None
        self._retry = 0

    def _connect(self):
        if mmap is None:
            return
        try:
            if self._traffic_buf is None:
                self._traffic_buf = mmap.mmap(0, _TRAFFIC_SIZE, r"Local\ETS2LATraffic")
        except Exception:
            self._traffic_buf = None
        try:
            if self._parked_buf is None:
                self._parked_buf = mmap.mmap(
                    0, _PARKED_SIZE, r"Local\ETS2LAParkedVehicles")
        except Exception:
            self._parked_buf = None
        try:
            if self._sem_buf is None:
                self._sem_buf = mmap.mmap(0, _SEM_SIZE, r"Local\ETS2LASemaphore")
        except Exception:
            self._sem_buf = None

    def _ensure(self):
        if (self._traffic_buf is not None and self._sem_buf is not None
                and self._parked_buf is not None):
            return
        self._retry += 1
        if self._retry % 30 == 1:   # retry roughly every ~0.5s of calls
            self._connect()

    # --- Public reads ---------------------------------------------------------
    def read_traffic(self) -> list:
        """List of {x, z, yaw, length, width, speed, type, id} for nearby vehicles."""
        self._ensure()
        if self._traffic_buf is None:
            return []
        try:
            data = struct.unpack(_TRAFFIC_FMT, self._traffic_buf[:_TRAFFIC_SIZE])
        except Exception:
            self._traffic_buf = None
            return []
        out = []
        for i in range(40):
            b = i * 46
            px, py, pz = data[b], data[b + 1], data[b + 2]
            rx, ry, rz, rw = data[b + 3], data[b + 4], data[b + 5], data[b + 6]
            width, height, length = data[b + 7], data[b + 8], data[b + 9]
            speed = data[b + 10]
            vid = data[b + 13]
            if px == 0 and pz == 0:
                continue
            out.append({
                "x": px, "y": py, "z": pz, "yaw": _yaw(rx, ry, rz, rw),
                "length": length or 4.5, "width": width or 2.0,
                "speed": speed, "type": _veh_type(length or 4.5), "id": vid,
            })
        # The companion buffer contains stationary/parked traffic. Without it,
        # cars stopped around junctions disappear from the visualization.
        if self._parked_buf is not None:
            try:
                parked = struct.unpack(_PARKED_FMT, self._parked_buf[:_PARKED_SIZE])
                seen_ids = {vehicle["id"] for vehicle in out}
                for i in range(40):
                    b = i * 12
                    px, py, pz = parked[b], parked[b + 1], parked[b + 2]
                    q0, q1, q2, q3 = parked[b + 3:b + 7]
                    width, height, length = parked[b + 7:b + 10]
                    vid = parked[b + 10]
                    if ((px == 0 and pz == 0) or vid in seen_ids
                            or not any((q0, q1, q2, q3))):
                        continue
                    out.append({
                        "x": px, "y": py, "z": pz, "yaw": _yaw(q0, q1, q2, q3),
                        "length": length or 4.5, "width": width or 2.0,
                        "speed": 0.0, "type": _veh_type(length or 4.5),
                        "id": vid, "parked": True,
                    })
                    seen_ids.add(vid)
            except Exception:
                self._parked_buf = None
        return out

    def read_traffic_lights(self) -> list:
        """List of {x, z, state, color, time_left} for active traffic lights."""
        self._ensure()
        if self._sem_buf is None:
            return []
        try:
            data = struct.unpack(_SEM_FMT, self._sem_buf[:_SEM_SIZE])
        except Exception:
            self._sem_buf = None
            return []
        out = []
        for i in range(40):
            b = i * 13
            px, py, pz = data[b], data[b + 1], data[b + 2]
            kind = data[b + 9]          # 1 = traffic light
            time_left = data[b + 10]
            state = data[b + 11]
            if kind != 1 or (px == 0 and pz == 0):
                continue
            out.append({"x": px, "z": pz, "state": state,
                        "color": _state_color(state), "time_left": time_left})
        return out


def _state_color(state):
    if state == ST_RED:
        return "red"
    if state == ST_GREEN:
        return "green"
    if state in (ST_Y2R, ST_Y2G):
        return "yellow"
    return "off"


def nearest_light_ahead(lights, pos, heading, max_dist=120.0):
    """Pick the traffic light most likely controlling us (ahead, within range)."""
    if not lights or not pos:
        return None
    px, pz = pos
    fx, fz = -math.sin(heading), -math.cos(heading)
    best, best_d = None, max_dist
    for lt in lights:
        dx, dz = lt["x"] - px, lt["z"] - pz
        dist = math.hypot(dx, dz)
        if dist < 3 or dist > max_dist:
            continue
        # in front of us (dot of forward and direction-to-light > 0)
        if (fx * dx + fz * dz) <= 0:
            continue
        if dist < best_d:
            best_d, best = dist, dict(lt, distance=dist)
    return best
