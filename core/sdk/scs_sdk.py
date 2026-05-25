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

    def update(self) -> Dict[str, Any]:
        """Read all telemetry data from shared memory."""
        if not self.mm:
            return {}

        data = {}
        offset = 0

        try:
            # Zone 1: Basic info
            data["sdkActive"], offset = self.read_bool(offset)
            offset += 3 # placeholder
            data["pause"], offset = self.read_bool(offset)
            offset += 3 # placeholder
            data["time"], offset = self.read_long_long(offset)
            data["simulatedTime"], offset = self.read_long_long(offset)
            data["renderTime"], offset = self.read_long_long(offset)
            data["multiplayerTimeOffset"], offset = self.read_long_long(offset)

            # Zone 2: UI and Config (simplified)
            # Skipping some buffers to get to key values
            offset = 40
            data["scsValues"] = {}
            data["scsValues"]["versionMajor"], offset = self.read_int(offset)
            data["scsValues"]["versionMinor"], offset = self.read_int(offset)

            # Fast forward to Truck Float data (Zone 4 starts at 700)
            offset = 700
            data["truckFloat"] = {}
            data["truckFloat"]["speed"], offset = self.read_float(offset)
            data["truckFloat"]["engineRpm"], offset = self.read_float(offset)
            data["truckFloat"]["userSteer"], offset = self.read_float(offset)
            data["truckFloat"]["userThrottle"], offset = self.read_float(offset)
            data["truckFloat"]["userBrake"], offset = self.read_float(offset)
            data["truckFloat"]["gameSteer"], offset = self.read_float(offset)
            data["truckFloat"]["gameThrottle"], offset = self.read_float(offset)
            data["truckFloat"]["gameBrake"], offset = self.read_float(offset)
            data["truckFloat"]["cruiseControlSpeed"], offset = self.read_float(offset)
            data["truckFloat"]["fuel"], offset = self.read_float(offset)
            data["truckFloat"]["speedLimit"], offset = self.read_float(offset)

            # Zone 5: Bools (Start at 1500)
            offset = 1500
            data["truckBool"] = {}
            data["truckBool"]["parkBrake"], offset = self.read_bool(offset)
            data["truckBool"]["engineEnabled"], offset = self.read_bool(offset + 550) # approximated

            # Zone 8: Placement (Start at 2200)
            offset = 2200
            data["truckPlacement"] = {}
            data["truckPlacement"]["coordinateX"], offset = self.read_double(offset)
            data["truckPlacement"]["coordinateY"], offset = self.read_double(offset)
            data["truckPlacement"]["coordinateZ"], offset = self.read_double(offset)
            data["truckPlacement"]["rotationX"], offset = self.read_double(offset)
            data["truckPlacement"]["rotationY"], offset = self.read_double(offset)
            data["truckPlacement"]["rotationZ"], offset = self.read_double(offset)

        except Exception as e:
            logging.error(f"Error reading SCS telemetry: {e}")

        return data
