import requests
import logging
from typing import Dict, Any
from core.sdk.scs_sdk import SCSTelemetry

class Telemetry:
    """Reads telemetry data from the ETS2 Telemetry Server or Shared Memory."""

    def __init__(self, url: str = "http://localhost:2302/api/"):
        self.url = url
        self.data: Dict[str, Any] = {}
        self.sdk_reader = SCSTelemetry()
        self.use_sdk = self.sdk_reader.connect()
        logging.info(f"Using {'Shared Memory' if self.use_sdk else 'HTTP'} for telemetry.")

    def update(self) -> bool:
        """Fetch latest telemetry data."""
        if self.use_sdk:
            try:
                self.data = self.sdk_reader.update()
                return True
            except Exception as e:
                logging.error(f"SDK Telemetry error: {e}. Falling back to HTTP.")
                self.use_sdk = False

        try:
            response = requests.get(f"{self.url}truck", timeout=0.1)
            if response.status_code == 200:
                self.data = response.json()
                return True
        except Exception as e:
            logging.error(f"Telemetry error: {e}")
        return False

    def get(self, key: str, default: Any = None) -> Any:
        """Get a specific value from telemetry data."""
        return self.data.get(key, default)
