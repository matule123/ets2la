import mmap
import struct
import logging
import os
from typing import Any, Dict, Tuple, List

class SCSTelemetry:
    """
    High-performance telemetry reader using shared memory (mmap).
    Based on the SCS SDK Telemetry common headers.
    """
    def __init__(self):
        if os.name != "nt":
            self.mmap_name = "/dev/shm/SCS/SCSTelemetry"
        else:
            self.mmap_name = "Local\\SCSTelemetry"

        self.mmap_size = 32 * 1024
        self.string_size = 64
        self.mm = None

    def connect(self):
        try:
            self.mm = mmap.mmap(0, self.mmap_size, self.mmap_name)
            logging.info("Connected to SCS Telemetry shared memory.")
            return True
        except Exception as e:
            logging.error(f"Could not connect to SCS Telemetry: {e}")
            return False

    def read_bool(self, offset: int, count: int = 1) -> Tuple[Any, int]:
        if count == 1:
            val = struct.unpack("?", self.mm[offset : offset + 1])[0]
            return val, offset + 1
        bools = [struct.unpack("?", self.mm[offset + i : offset + i + 1])[0] for i in range(count)]
        return bools, offset + count

    def read_int(self, offset: int, count: int = 1) -> Tuple[Any, int]:
        if count == 1:
            val = struct.unpack("i", self.mm[offset : offset + 4])[0]
            return val, offset + 4
        ints = [struct.unpack("i", self.mm[offset + i * 4 : offset + i * 4 + 4])[0] for i in range(count)]
        return ints, offset + count * 4

    def read_float(self, offset: int, count: int = 1) -> Tuple[Any, int]:
        if count == 1:
            val = struct.unpack("f", self.mm[offset : offset + 4])[0]
            return val, offset + 4
        floats = [struct.unpack("f", self.mm[offset + i * 4 : offset + i * 4 + 4])[0] for i in range(count)]
        return floats, offset + count * 4

    def read_double(self, offset: int, count: int = 1) -> Tuple[Any, int]:
        if count == 1:
            val = struct.unpack("d", self.mm[offset : offset + 8])[0]
            return val, offset + 8
        doubles = [struct.unpack("d", self.mm[offset + i * 8 : offset + i * 8 + 8])[0] for i in range(count)]
        return doubles, offset + count * 8

    def read_long_long(self, offset: int, count: int = 1) -> Tuple[Any, int]:
        if count == 1:
            val = struct.unpack("Q", self.mm[offset : offset + 8])[0]
            return val, offset + 8
        longs = [struct.unpack("Q", self.mm[offset + i * 8 : offset + i * 8 + 8])[0] for i in range(count)]
        return longs, offset + count * 8

    def read_char(self, offset: int, count: int) -> Tuple[str, int]:
        char_data = self.mm[offset : offset + count]
        try:
            decoded = char_data.decode("utf-8").rstrip("\x00")
        except UnicodeDecodeError:
            decoded = ""
        return decoded, offset + count

    # --- Trailer (Zone 14) -------------------------------------------------
    # The scs-sdk-plugin stores up to 10 trailer structs back-to-back starting
    # at absolute offset 6000 (Zone 14). Each trailer struct is 1560 bytes; we
    # only need a handful of fields from the first attached trailer. The byte
    # offsets below are derived from scs-telemetry-common.hpp / the reference
    # ETS2LA reader (Modules/TruckSimAPI/api.py:readTrailer):
    #   * attached  — bool at block-relative +81   (Zone 1 wheel flags + 1)
    #   * worldX/Y/Z — double at block-relative +872 (Zone 5 placement)
    #   * rotationX/Y/Z — double at +896/+904/+912  (heading/pitch/roll)
    # rotationX is a 0..1 fraction of a full turn (same convention as the
    # truck), so the caller multiplies by 2π to get a heading in radians.
    TRAILER_BLOCK_START = 6000
    TRAILER_BLOCK_SIZE = 1560
    TRAILER_MAX = 10

    def read_trailer(self, index: int = 0) -> Dict[str, Any]:
        """Read the placement + attached flag of trailer ``index`` (0-based).

        Returns an empty dict if the SDK isn't connected or the offset would
        exceed the mapped region. Callers treat ``not trailer`` as "no trailer
        / unsupported", so a missing or disconnected trailer degrades to the
        cab-only rendering without raising.
        """
        if not self.mm:
            return {}
        base = self.TRAILER_BLOCK_START + index * self.TRAILER_BLOCK_SIZE
        # The whole first-trailer struct must fit inside the 32 KB mapping.
        if base + self.TRAILER_BLOCK_SIZE > self.mmap_size:
            return {}
        try:
            attached = self.read_bool(base + 81)[0]
            world_x = self.read_double(base + 872)[0]
            world_y = self.read_double(base + 880)[0]
            world_z = self.read_double(base + 888)[0]
            rot_x = self.read_double(base + 896)[0]
            rot_y = self.read_double(base + 904)[0]
            rot_z = self.read_double(base + 912)[0]
        except Exception as e:
            logging.error(f"Error reading trailer {index}: {e}")
            return {}
        return {
            "attached": bool(attached),
            "worldX": float(world_x),
            "worldY": float(world_y),
            "worldZ": float(world_z),
            "rotationX": float(rot_x),
            "rotationY": float(rot_y),
            "rotationZ": float(rot_z),
        }

    # --- Job destination (Zone 9 config strings) --------------------------
    # Zone 9 starts at offset 2300 and is a row of fixed 64-byte strings. The
    # 8th string is ``cityDst`` — the human-readable destination city of the
    # current job (e.g. "Berlin"). Used for the overhead gantry sign text.
    # Layout (each string 64 bytes): truckBrandId(2300) truckBrand(2364)
    # truckId(2428) truckName(2492) cargoId(2556) cargo(2620) cityDstId(2684)
    # cityDst(2748) compDstId(2812) ...
    CITY_DST_OFFSET = 2748
    CITY_DST_LEN = 64

    def read_job_destination(self) -> str:
        """Destination city name of the current job, or ``""`` if none / error.

        Empty string means "no active job" (the game writes blanks into the
        slot). Callers treat falsy as "no destination to show on a sign"."""
        if not self.mm:
            return ""
        try:
            name, _ = self.read_char(self.CITY_DST_OFFSET, self.CITY_DST_LEN)
            return name.strip()
        except Exception as e:
            logging.error(f"Error reading job destination: {e}")
            return ""

    def update(self) -> Dict[str, Any]:
        """Read the telemetry fields we use from shared memory.

        Uses the **fixed absolute zone offsets** of the scs-sdk-plugin shared
        memory layout (the same plugin/struct ETS2LA targets), derived from
        scs-telemetry-common.hpp:

            Zone 4 (floats)  starts at 700
            Zone 5 (bools)   starts at 1500
            Zone 8 (doubles) starts at 2200   <- truck world placement

        The previous version hand-guessed these offsets, so position/heading
        were wrong, which broke any coordinate-based navigation.
        """
        if not self.mm:
            return {}

        data: Dict[str, Any] = {}
        try:
            # --- Zone 3: truck ints (gear) ---
            data["truckInt"] = {"gear": self.read_int(504)[0]}

            # --- Zone 4: truck floats (absolute byte offsets) ---
            tf = {}
            tf["speed"] = self.read_float(948)[0]               # m/s
            tf["engineRpm"] = self.read_float(952)[0]
            tf["cruiseControlSpeed"] = self.read_float(988)[0]
            tf["fuel"] = self.read_float(1000)[0]               # liters
            tf["fuelRange"] = self.read_float(1008)[0]          # km
            tf["speedLimit"] = self.read_float(1068)[0]         # m/s
            data["truckFloat"] = tf

            # --- Zone 5: truck bools ---
            tb = {}
            tb["parkBrake"] = self.read_bool(1566)[0]
            tb["engineEnabled"] = self.read_bool(1576)[0]
            tb["blinkerLeftActive"] = self.read_bool(1578)[0]
            tb["blinkerRightActive"] = self.read_bool(1579)[0]
            data["truckBool"] = tb

            # --- Zone 8: truck world placement (doubles) ---
            tp = {}
            tp["coordinateX"] = self.read_double(2200)[0]
            tp["coordinateY"] = self.read_double(2208)[0]
            tp["coordinateZ"] = self.read_double(2216)[0]
            # rotationX is a 0..1 fraction of a full turn (heading); Y/Z are pitch/roll.
            tp["rotationX"] = self.read_double(2224)[0]
            tp["rotationY"] = self.read_double(2232)[0]
            tp["rotationZ"] = self.read_double(2240)[0]
            data["truckPlacement"] = tp

            # sdkActive lives at offset 0 — useful to know the game is feeding data.
            data["sdkActive"] = self.read_bool(0)[0]

        except Exception as e:
            logging.error(f"Error reading SCS telemetry: {e}")

        return data
