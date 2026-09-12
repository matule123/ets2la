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

    def __init__(self, url: str = "http://localhost:25555/api/ets2/telemetry", *,
                 vehicle_profile_catalog=None):
        self.url = url
        self.data: Dict[str, Any] = {}
        from core.vehicle_profile import VehicleProfileProvider
        self.vehicle_profiles = VehicleProfileProvider(vehicle_profile_catalog)
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
    def _vehicle_profile(self, observation):
        import time
        import json
        from dataclasses import asdict
        from core.vehicle_profile import VehicleProfileProvider
        if not hasattr(self, "vehicle_profiles"):
            self.vehicle_profiles = VehicleProfileProvider()
        now = time.monotonic()
        profile = self.vehicle_profiles.update(observation, now)
        # Diagnostics only: bounded log rate, no disk work or controller changes.
        signature = (profile.token.configuration, profile.observation_failure, profile.model_failure)
        if (signature != getattr(self, '_profile_log_signature', None) and
                now-getattr(self, '_profile_logged_at', -10.) >= 10.):
            diagnostic = profile.diagnostic()
            if profile.observation and not profile.observation_failure:
                diagnostic['observation'] = asdict(profile.observation)
            logging.info("Vehicle profile: %s", json.dumps(diagnostic, ensure_ascii=True))
            self._profile_log_signature, self._profile_logged_at = signature, now
        return profile

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
            "userSteer": tf.get("userSteer", 0.0),
            "gameSteer": tf.get("gameSteer", 0.0),
            "gear": ti.get("gear", 0),
            "fuel": tf.get("fuel", 0.0),
            "fuelRange": tf.get("fuelRange", 0.0),
            "routeDistance": tf.get("routeDistance", 0.0),
            "routeTime": tf.get("routeTime", 0.0),
            "speedLimit": tf.get("speedLimit", 0.0),
            "cruiseControlSpeed": tf.get("cruiseControlSpeed", 0.0),
            "parkBrake": tb.get("parkBrake", False),
            "engineEnabled": tb.get("engineEnabled", False),
            "blinkerLeft": tb.get("blinkerLeftActive", False),
            "blinkerRight": tb.get("blinkerRightActive", False),
            "rotation": heading,                     # radians (heading)
            "x": tp.get("coordinateX", 0.0),
            "y": tp.get("coordinateY", 0.0),
            "z": tp.get("coordinateZ", 0.0),
            "pose_valid": all(key in tp for key in
                              ("coordinateX", "coordinateY", "coordinateZ",
                               "rotationX")),
        }
        # Tyre angles are independent calibration diagnostics; yaw is the
        # vehicle-motion input to the predictor. Neither rewrites LaneMatch.
        wheel_turns = tf.get("wheelSteeringTurns", ()) or ()
        steerable = raw.get("wheelSteerable", ()) or ()
        truck["roadWheelAnglesRad"] = [
            -math.tau * float(turns)
            for turns, enabled in zip(wheel_turns, steerable)
            if enabled and math.isfinite(float(turns))]
        truck["yawRateRadS"] = -math.tau * float(
            raw.get("yawRateTurnsPerSecond", 0.0) or 0.0)
        truck["yawRateValid"] = ("yawRateTurnsPerSecond" in raw
                                and math.isfinite(truck["yawRateRadS"]))
        truck["sdkFrameTimeUs"] = raw.get("time", 0)
        from core.vehicle_geometry import sdk_reference_geometry
        truck["referenceGeometry"] = sdk_reference_geometry(raw)
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
                raw_wheel_count = int(raw_tr.get("wheelCount", 0) or 0)
                wheel_count = (raw_wheel_count
                               if 0 <= raw_wheel_count <= 16 else 0)
                wheel_x = list(raw_tr.get("wheelPositionX", ()) or ())[:wheel_count]
                wheel_y = list(raw_tr.get("wheelPositionY", ()) or ())[:wheel_count]
                wheel_z = list(raw_tr.get("wheelPositionZ", ()) or ())[:wheel_count]
                hook = (
                    float(raw_tr.get("hookPositionX", 0.0) or 0.0),
                    float(raw_tr.get("hookPositionY", 0.0) or 0.0),
                    float(raw_tr.get("hookPositionZ", 0.0) or 0.0),
                )
                effective_axle_distance = None
                wheel_track = None
                if (wheel_count > 0
                        and len(wheel_x) == len(wheel_y) == len(wheel_z)
                        and all(math.isfinite(float(value)) for value in
                                (*hook, *wheel_x, *wheel_y, *wheel_z))):
                    axle_centre = (
                        sum(map(float, wheel_x)) / wheel_count,
                        sum(map(float, wheel_y)) / wheel_count,
                        sum(map(float, wheel_z)) / wheel_count,
                    )
                    effective_axle_distance = math.hypot(
                        axle_centre[0] - hook[0],
                        axle_centre[2] - hook[2])
                    wheel_track = max(map(float, wheel_x)) - min(
                        map(float, wheel_x))
                trailer = {
                    "attached": True,
                    "speed": speed_ms,                 # approximated by the truck's
                    "x": raw_tr.get("worldX", 0.0),
                    "y": raw_tr.get("worldY", 0.0),
                    "z": raw_tr.get("worldZ", 0.0),
                    "rotation": tr_heading,            # radians (heading)
                    "rotationX": raw_tr.get("rotationX", 0.0),
                    # Static local SDK geometry. Consumers validate its
                    # physical range before using it; malformed geometry is
                    # never silently promoted to steering authority.
                    "wheelCount": wheel_count,
                    "effectiveAxleDistanceM": effective_axle_distance,
                    "wheelTrackM": wheel_track,
                }
        except Exception as e:
            logging.debug(f"Trailer telemetry unavailable: {e}")

        # --- Job destination city (Zone 9 string). Empty when no job active.
        dest_city = ""
        try:
            dest_city = self.sdk_reader.read_job_destination()
        except Exception as e:
            logging.debug(f"Job destination unavailable: {e}")

        from dataclasses import replace
        from core.sdk.vehicle_observation import VehicleObservation
        observation = raw.get("vehicleObservation")
        if (isinstance(observation, VehicleObservation) and not observation.failure_reason
                and observation.sdk_frame_us != raw.get("time")):
            observation = replace(observation, failure_reason='SDK_PROFILE_FRAME_MISMATCH')
        profile = self._vehicle_profile(observation)
        return {"raw": raw, "truck": truck, "trailer": trailer,
                "vehicle_profile": profile,
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
            "routeDistance": truck.get("routeDistance", 0.0),
            "routeTime": truck.get("routeTime", 0.0),
            # The legacy HTTP endpoint used by this project does not expose a
            # verified world-pose contract. Navigation must fail closed rather
            # than interpreting absent coordinates as world origin.
            "pose_valid": False,
        }
        return {"raw": payload, "truck": norm,
                "vehicle_profile": self._vehicle_profile(None)}

    def get(self, key: str, default: Any = None) -> Any:
        return self.data.get(key, default)
