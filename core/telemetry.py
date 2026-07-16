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
        # SCS stores rotationX in turns (ETS2LA likewise multiplies it by 360
        # for degrees). Convert once here and expose radians to navigation.
        rot_x = float(tp.get("rotationX", 0.0) or 0.0)
        heading = (rot_x * math.tau + math.pi) % math.tau - math.pi
        if not getattr(self, "_logged_rot_convention", False):
            logging.info("telemetry: rotationX=%.4f turns -> heading %.4f rad (%.1f deg)",
                         rot_x, heading, math.degrees(heading))
            self._logged_rot_convention = True
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

        # --- Trailer placement (Zone 14). Only the first trailer is used —
        # the ETS2 tractor+semi-trailer combo is articulated at a single hitch.
        # If none is attached (or the SDK doesn't expose it) we publish an
        # empty trailer dict so downstream code can detect "no trailer".
        trailer = {}
        try:
            raw_tr = self.sdk_reader.read_trailer(0)
            if raw_tr and raw_tr.get("attached"):
                raw_tr_heading = float(raw_tr.get("rotationX", 0.0) or 0.0)
                tr_heading = (raw_tr_heading * math.tau + math.pi) % math.tau - math.pi
                trailer = {
                    "attached": True,
                    "speed": speed_ms,                 # approximated by the truck's
                    "x": raw_tr.get("worldX", 0.0),
                    "z": raw_tr.get("worldZ", 0.0),
                    "rotation": tr_heading,            # radians (heading)
                    "rotationX": raw_tr.get("rotationX", 0.0),
                }
        except Exception as e:
            logging.debug(f"Trailer telemetry unavailable: {e}")

        # --- Job destination city (Zone 9 string). Empty when no job active.
        dest_city = ""
        try:
            dest_city = self.sdk_reader.read_job_destination()
        except Exception as e:
            logging.debug(f"Job destination unavailable: {e}")

        return {"raw": raw, "truck": truck, "trailer": trailer,
                "dest_city": dest_city,
                "position": pos, "heading": heading}

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
