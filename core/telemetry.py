import requests
import logging
from typing import Dict, Any

class Telemetry:
    """Reads telemetry data from the ETS2 Telemetry Server."""

    def __init__(self, url: str = "http://localhost:2302/api/"):
        self.url = url
        self.data: Dict[str, Any] = {}

    def update(self) -> bool:
        """Fetch latest telemetry data."""
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
