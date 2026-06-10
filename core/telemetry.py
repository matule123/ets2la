import logging
from typing import Dict, Any

try:
    import requests
except Exception:
    requests = None

from core.sdk.scs_sdk import SCSTelemetry


class Telemetry:
    """
    Reads telemetry from the SCS shared-memory SDK (preferred) or the HTTP
    telemetry server, and normalizes it into a consistent ``truck`` dict so
    every plugin can rely on ``telemetry.get("truck", {})["speed"]`` regardless
    of the source.
    """

    def __init__(self, url: str = "http://localhost:25555/api/ets2/telemetry"):
        self.url = url
        self.data: Dict[str, Any] = {}
        self.sdk_reader = SCSTelemetry()
        self.use_sdk = self.sdk_reader.connect()
        logging.info(f"Using {'Shared Memory' if self.use_sdk else 'HTTP'} for telemetry.")

    def update(self) -> bool:
        if self.use_sdk:
            try:
                raw = self.sdk_reader.update()
                self.data = self._normalize_sdk(raw)
                return True
            except Exception as e:
                logging.error(f"SDK Telemetry error: {e}. Falling back to HTTP.")
                self.use_sdk = False

        if requests is None:
            return False
        try:
            response = requests.get(self.url, timeout=0.2)
            if response.status_code == 200:
                self.data = self._normalize_http(response.json())
                return True
        except Exception as e:
            logging.debug(f"Telemetry HTTP error: {e}")
        return False

    # --- Normalization --------------------------------------------------------
    def _normalize_sdk(self, raw: Dict[str, Any]) -> Dict[str, Any]:
        import math
        tf = raw.get("truckFloat", {}) or {}
        tb = raw.get("truckBool", {}) or {}
        tp = raw.get("truckPlacement", {}) or {}
        ti = raw.get("truckInt", {}) or {}
        speed_ms = tf.get("speed", 0.0) or 0.0
        # rotationX is a 0..1 fraction of a full turn → heading in radians.
        heading = (tp.get("rotationX", 0.0) or 0.0) * 2.0 * math.pi
        truck = {
            "speed": speed_ms,                       # m/s (plugins convert)
            "speed_kmh": abs(speed_ms) * 3.6,
            "engineRpm": tf.get("engineRpm", 0.0),
            "gear": ti.get("gear", 0),
            "fuel": tf.get("fuel", 0.0),
            "fuelRange": tf.get("fuelRange", 0.0),
            "speedLimit": tf.get("speedLimit", 0.0),
            "cruiseControlSpeed": tf.get("cruiseControlSpeed", 0.0),
            "parkBrake": tb.get("parkBrake", False),
            "engineEnabled": tb.get("engineEnabled", False),
            "blinkerLeft": tb.get("blinkerLeftActive", False),
            "blinkerRight": tb.get("blinkerRightActive", False),
            "rotation": heading,                     # radians (heading)
            "x": tp.get("coordinateX", 0.0),
            "z": tp.get("coordinateZ", 0.0),
        }
        pos = (tp.get("coordinateX", 0.0), tp.get("coordinateZ", 0.0))
        return {"raw": raw, "truck": truck, "position": pos, "heading": heading}

    def _normalize_http(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        truck = payload.get("truck", payload) or {}
        norm = {
            "speed": truck.get("speed", 0.0),
            "speed_kmh": truck.get("speed", 0.0),
            "engineRpm": truck.get("engineRpm", 0.0),
            "fuel": truck.get("fuel", 0.0),
            "speedLimit": truck.get("speedLimit", 0.0),
            "cruiseControlSpeed": truck.get("cruiseControlSpeed", 0.0),
        }
        return {"raw": payload, "truck": norm}

    def get(self, key: str, default: Any = None) -> Any:
        return self.data.get(key, default)
