"""Verified ETS2 camera snapshots and world-to-screen projection.

The ETS2LA game plugin publishes ``Local\\ETS2LACameraProps`` with the
layout ``=ffffhhffff`` (FOV, local X/Y/Z, map tile X/Z, quaternion W/X/Y/Z).
Tile coordinates are 512 metre sectors, therefore absolute world X/Z are
``local + tile * 512``.  The quaternion axes follow ETS2LA's historical
Camera module: the two middle components are swapped before use.

Matrices in snapshots are row-major and multiply column vectors.  World
coordinates use ETS2 X/Y/Z (Y up); camera space is right-handed with +X to
the right, +Y up and -Z forward.  Projection uses horizontal FOV and OpenGL
clip Z in [-1, 1].  No approximate camera offset is used anywhere here.
"""

from __future__ import annotations

import ctypes
import math
import mmap
import os
import struct
import time
from typing import Callable, Optional, Sequence


CAMERA_MAPPING = r"Local\ETS2LACameraProps"
CAMERA_FORMAT = "=ffffhhffff"
CAMERA_SIZE = struct.calcsize(CAMERA_FORMAT)
CAMERA_MAX_AGE_S = 0.50
TELEMETRY_SYNC_TOLERANCE_S = 0.25


def _finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _matmul4(left: Sequence[float], right: Sequence[float]) -> list[float]:
    return [
        sum(float(left[row * 4 + index]) * float(right[index * 4 + column])
            for index in range(4))
        for row in range(4) for column in range(4)
    ]


def normalize_camera_quaternion(raw: Sequence[float]) -> tuple[float, float, float, float]:
    """Return ETS2LA camera quaternion as normalized ``(w, x, y, z)``.

    The producer stores W/X/Y/Z, but the original ETS2LA Camera class maps
    them to W/Y/X/Z.  Keeping that explicit avoids silently rotating around
    the wrong axes.
    """
    if len(raw) != 4 or not _finite(raw):
        raise ValueError("camera quaternion is missing or non-finite")
    w, stored_x, stored_y, z = map(float, raw)
    x, y = stored_y, stored_x
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-6 or not math.isfinite(norm):
        raise ValueError("camera quaternion has zero length")
    if not 0.5 <= norm <= 1.5:
        raise ValueError(f"camera quaternion norm {norm:.3f} is implausible")
    return w / norm, x / norm, y / norm, z / norm


def quaternion_to_euler(quaternion: Sequence[float]) -> tuple[float, float, float]:
    """Return ``(pitch, yaw, roll)`` radians in ETS2LA camera convention."""
    w, x, y, z = map(float, quaternion)
    yaw = math.atan2(2.0 * (y * z + w * x),
                     w * w - x * x - y * y + z * z)
    pitch_term = max(-1.0, min(1.0, -2.0 * (x * z - w * y)))
    pitch = math.asin(pitch_term)
    roll = math.atan2(2.0 * (x * y + w * z),
                      w * w + x * x - y * y - z * z)
    return pitch, yaw, roll


def build_camera_matrices(position: Sequence[float],
                          quaternion: Sequence[float],
                          horizontal_fov_deg: float, aspect: float,
                          near_m: float = 0.10,
                          far_m: float = 2000.0) -> tuple[list[float], list[float], list[float]]:
    """Build row-major view, projection and view-projection matrices."""
    if len(position) != 3 or not _finite(position):
        raise ValueError("camera position is missing or non-finite")
    if not (10.0 <= float(horizontal_fov_deg) <= 170.0):
        raise ValueError("camera horizontal FOV is outside 10..170 degrees")
    if not math.isfinite(float(aspect)) or not 0.2 <= float(aspect) <= 8.0:
        raise ValueError("camera viewport aspect ratio is invalid")
    if not (0.0 < near_m < far_m):
        raise ValueError("camera clipping planes are invalid")

    pitch, yaw, roll = quaternion_to_euler(quaternion)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    # Exact inverse yaw -> pitch -> roll sequence used by the original AR
    # ConvertToScreenCoordinate implementation, expressed as 4x4 matrices.
    inverse_yaw = [
        cy, 0.0, -sy, 0.0,
        0.0, 1.0, 0.0, 0.0,
        sy, 0.0, cy, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]
    inverse_pitch = [
        1.0, 0.0, 0.0, 0.0,
        0.0, cp, sp, 0.0,
        0.0, -sp, cp, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]
    inverse_roll = [
        cr, sr, 0.0, 0.0,
        -sr, cr, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]
    rotation = _matmul4(inverse_roll,
                        _matmul4(inverse_pitch, inverse_yaw))
    px, py, pz = map(float, position)
    view = list(rotation)
    for row in range(3):
        view[row * 4 + 3] = -(rotation[row * 4] * px
                              + rotation[row * 4 + 1] * py
                              + rotation[row * 4 + 2] * pz)

    tangent = math.tan(math.radians(float(horizontal_fov_deg)) * 0.5)
    fx = 1.0 / tangent
    fy = float(aspect) / tangent
    projection = [
        fx, 0.0, 0.0, 0.0,
        0.0, fy, 0.0, 0.0,
        0.0, 0.0, -(far_m + near_m) / (far_m - near_m),
        -(2.0 * far_m * near_m) / (far_m - near_m),
        0.0, 0.0, -1.0, 0.0,
    ]
    view_projection = _matmul4(projection, view)
    return view, projection, view_projection


def invalid_camera_snapshot(reason: str, *, revision: int = 0,
                            timestamp: Optional[float] = None,
                            render_time: int = 0) -> dict:
    return {
        "revision": int(revision), "valid": False,
        "failure_reason": str(reason), "source": CAMERA_MAPPING,
        "camera_mode": "unknown", "timestamp": float(
            time.monotonic() if timestamp is None else timestamp),
        "telemetry_timestamp": 0.0, "render_time_us": int(render_time or 0),
        "position": None, "quaternion": None, "viewport": None,
        "view_matrix": [], "projection_matrix": [],
        "view_projection": [],
    }


class GameViewportProvider:
    """Locate the visible ETS2/ATS client rectangle without fixed offsets."""

    TITLES = ("Euro Truck Simulator 2", "American Truck Simulator")

    def __init__(self, cache_s: float = 0.25):
        self.cache_s = float(cache_s)
        self._last_at = 0.0
        self._last = None

    def __call__(self) -> Optional[dict]:
        now = time.monotonic()
        if now - self._last_at < self.cache_s:
            return self._last
        self._last_at = now
        self._last = self._find_windows_viewport() if os.name == "nt" else None
        return self._last

    @classmethod
    def _find_windows_viewport(cls) -> Optional[dict]:
        user32 = ctypes.windll.user32

        class RECT(ctypes.Structure):
            _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long),
                        ("right", ctypes.c_long), ("bottom", ctypes.c_long)]

        class POINT(ctypes.Structure):
            _fields_ = [("x", ctypes.c_long), ("y", ctypes.c_long)]

        matches = []
        callback_type = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p,
                                           ctypes.c_void_p)

        @callback_type
        def enumerate_window(hwnd, _):
            if not user32.IsWindowVisible(hwnd):
                return True
            length = user32.GetWindowTextLengthW(hwnd)
            if length <= 0:
                return True
            title_buffer = ctypes.create_unicode_buffer(length + 1)
            user32.GetWindowTextW(hwnd, title_buffer, length + 1)
            title = title_buffer.value
            if any(name.lower() in title.lower() for name in cls.TITLES):
                matches.append((hwnd, title))
            return True

        user32.EnumWindows(enumerate_window, 0)
        for hwnd, title in matches:
            rect = RECT()
            origin = POINT(0, 0)
            if not user32.GetClientRect(hwnd, ctypes.byref(rect)):
                continue
            if not user32.ClientToScreen(hwnd, ctypes.byref(origin)):
                continue
            width, height = rect.right - rect.left, rect.bottom - rect.top
            if width >= 64 and height >= 64:
                return {
                    "x": int(origin.x), "y": int(origin.y),
                    "width": int(width), "height": int(height),
                    "aspect": float(width / height), "hwnd": int(hwnd),
                    "title": title,
                }
        return None


class CameraPropsReader:
    """Non-blocking reader for the game plugin's CameraProps mapping."""

    def __init__(self):
        self._mapping = None
        self._last_attempt = 0.0

    def _connect(self):
        now = time.monotonic()
        if self._mapping is not None or now - self._last_attempt < 0.5:
            return
        self._last_attempt = now
        try:
            self._mapping = mmap.mmap(0, CAMERA_SIZE, CAMERA_MAPPING)
        except Exception:
            self._mapping = None

    def __call__(self) -> Optional[tuple]:
        self._connect()
        if self._mapping is None:
            return None
        try:
            # Two equal copies reject a frame boundary/torn struct. Camera data
            # normally changes at render cadence, far slower than these copies.
            previous = self._mapping[:CAMERA_SIZE]
            for _ in range(3):
                current = self._mapping[:CAMERA_SIZE]
                if current == previous:
                    return struct.unpack(CAMERA_FORMAT, current)
                previous = current
        except Exception:
            self._mapping = None
        return None


class CameraSnapshotProducer:
    """Create immutable, time-synchronised camera snapshots for shared state."""

    def __init__(self, read_raw: Optional[Callable[[], Optional[tuple]]] = None,
                 viewport_provider: Optional[Callable[[], Optional[dict]]] = None):
        self.read_raw = read_raw or CameraPropsReader()
        self.viewport_provider = viewport_provider or GameViewportProvider()
        self.revision = 0
        self._last_render_time = 0
        self._last_render_change_at = 0.0

    def _invalid(self, reason: str, now: float, render_time: int) -> dict:
        self.revision += 1
        return invalid_camera_snapshot(reason, revision=self.revision,
                                       timestamp=now, render_time=render_time)

    def read(self, render_time: int, telemetry_timestamp: float,
             now: Optional[float] = None) -> dict:
        now = float(time.monotonic() if now is None else now)
        try:
            render_time = int(render_time or 0)
            telemetry_timestamp = float(telemetry_timestamp or 0.0)
        except (TypeError, ValueError):
            return self._invalid("camera timing metadata is invalid", now, 0)
        if render_time <= 0:
            return self._invalid("SCS renderTime is unavailable", now, render_time)
        if render_time != self._last_render_time:
            self._last_render_time = render_time
            self._last_render_change_at = now
        elif (self._last_render_change_at <= 0.0
              or now - self._last_render_change_at > CAMERA_MAX_AGE_S):
            return self._invalid("SCS renderTime is stale or the game is paused",
                                 now, render_time)
        if (telemetry_timestamp <= 0.0
                or abs(now - telemetry_timestamp) > TELEMETRY_SYNC_TOLERANCE_S):
            return self._invalid("camera and telemetry timestamps are not synchronized",
                                 now, render_time)

        raw = self.read_raw()
        if raw is None:
            return self._invalid(
                f"camera shared memory {CAMERA_MAPPING} is unavailable or unstable",
                now, render_time)
        viewport = self.viewport_provider()
        if not isinstance(viewport, dict):
            return self._invalid("ETS2/ATS game client viewport is unavailable",
                                 now, render_time)
        try:
            fov, local_x, y, local_z, tile_x, tile_z, qw, qx, qy, qz = raw
            scalars = (fov, local_x, y, local_z, qw, qx, qy, qz)
            if not _finite(scalars):
                raise ValueError("camera properties contain NaN or Infinity")
            width = int(viewport["width"])
            height = int(viewport["height"])
            if width < 64 or height < 64:
                raise ValueError("game viewport is too small")
            aspect = float(width / height)
            position = (float(local_x) + int(tile_x) * 512.0,
                        float(y), float(local_z) + int(tile_z) * 512.0)
            quaternion = normalize_camera_quaternion((qw, qx, qy, qz))
            view, projection, view_projection = build_camera_matrices(
                position, quaternion, float(fov), aspect)
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            return self._invalid(str(error), now, render_time)

        self.revision += 1
        return {
            "revision": self.revision, "valid": True, "failure_reason": "",
            "source": CAMERA_MAPPING,
            "camera_mode": "game-camera-manager",
            "position": list(position), "local_position": [float(local_x), float(y), float(local_z)],
            "tile": [int(tile_x), int(tile_z)],
            "quaternion": list(quaternion),
            "quaternion_raw": [float(qw), float(qx), float(qy), float(qz)],
            "fov_horizontal_deg": float(fov),
            "fov_convention": "horizontal-degrees",
            "viewport": dict(viewport, width=width, height=height, aspect=aspect),
            "aspect": aspect,
            "timestamp": now, "telemetry_timestamp": telemetry_timestamp,
            "render_time_us": render_time,
            "synchronized_to": "SCS renderTime sampled with telemetry",
            "matrix_layout": "row-major",
            "vector_convention": "column-vector",
            "world_axes": ("ETS2 +X east, +Y up, +Z south; "
                           "heading 0 faces -Z north"),
            "world_handedness": "right-handed",
            "camera_axes": "+X right, +Y up, -Z forward",
            "handedness": "right-handed",
            "clip_space": "opengl-negative-one-to-one",
            "view_matrix": view, "projection_matrix": projection,
            "view_projection": view_projection,
        }


def camera_snapshot_reason(snapshot: object, *, now: Optional[float] = None,
                           telemetry_timestamp: Optional[float] = None) -> str:
    """Return an exact rejection reason, or ``""`` for a current snapshot."""
    if not isinstance(snapshot, dict):
        return "camera snapshot is missing"
    if not snapshot.get("valid", False):
        return str(snapshot.get("failure_reason") or "camera snapshot is invalid")
    now = float(time.monotonic() if now is None else now)
    try:
        timestamp = float(snapshot.get("timestamp", 0.0) or 0.0)
        sampled_telemetry = float(snapshot.get("telemetry_timestamp", 0.0) or 0.0)
        matrix = snapshot.get("view_projection")
        viewport = snapshot.get("viewport") or {}
        quaternion = snapshot.get("quaternion")
        if timestamp <= 0.0 or now - timestamp > CAMERA_MAX_AGE_S:
            return "camera snapshot is stale"
        if telemetry_timestamp is not None:
            telemetry_timestamp = float(telemetry_timestamp or 0.0)
            if (telemetry_timestamp <= 0.0
                    or abs(sampled_telemetry - telemetry_timestamp)
                        > TELEMETRY_SYNC_TOLERANCE_S):
                return "camera snapshot belongs to stale telemetry"
        if not isinstance(matrix, (list, tuple)) or len(matrix) != 16 or not _finite(matrix):
            return "camera view-projection matrix is missing or non-finite"
        if not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4 or not _finite(quaternion):
            return "camera quaternion is missing or non-finite"
        if int(viewport.get("width", 0)) < 64 or int(viewport.get("height", 0)) < 64:
            return "camera viewport is invalid"
        if snapshot.get("matrix_layout") != "row-major":
            return "camera matrix layout is unsupported"
        if snapshot.get("clip_space") != "opengl-negative-one-to-one":
            return "camera clip-space convention is unsupported"
    except (TypeError, ValueError, OverflowError):
        return "camera snapshot metadata is malformed"
    return ""


def project_world_point(snapshot: dict, point: Sequence[float], *,
                        now: Optional[float] = None,
                        telemetry_timestamp: Optional[float] = None) -> Optional[tuple[float, float, float]]:
    """Project ETS2 X/Y/Z to viewport-local pixel X/Y and positive depth."""
    if camera_snapshot_reason(snapshot, now=now,
                              telemetry_timestamp=telemetry_timestamp):
        return None
    try:
        x, y, z = map(float, point[:3])
        if not _finite((x, y, z)):
            return None
        matrix = [float(value) for value in snapshot["view_projection"]]
        vector = (x, y, z, 1.0)
        clip = [sum(matrix[row * 4 + column] * vector[column]
                    for column in range(4)) for row in range(4)]
        if not _finite(clip) or clip[3] <= 1e-5:
            return None
        ndc_x, ndc_y, ndc_z = (clip[index] / clip[3] for index in range(3))
        if not (-1.0 <= ndc_x <= 1.0 and -1.0 <= ndc_y <= 1.0
                and -1.0 <= ndc_z <= 1.0):
            return None
        viewport = snapshot["viewport"]
        width, height = float(viewport["width"]), float(viewport["height"])
        return ((ndc_x * 0.5 + 0.5) * width,
                (1.0 - (ndc_y * 0.5 + 0.5)) * height,
                float(clip[3]))
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return None
