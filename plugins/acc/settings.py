from dataclasses import dataclass

@dataclass
class ACCSettings:
    target_speed: float = 80.0       # km/h
    safe_distance: float = 50.0      # meters
    aggressiveness: str = "Normal"   # "Eco", "Normal", "Aggressive"
    follow_distance_seconds: float = 2.0
    emergency_brake_threshold: float = 0.8
    obey_speed_limit: bool = True    # auto-cap target speed to posted limit

    def __post_init__(self):
        # Ensure values are within reasonable ranges
        self.target_speed = max(10.0, min(160.0, self.target_speed))
        self.safe_distance = max(5.0, min(200.0, self.safe_distance))

# Global settings instance
settings = ACCSettings()
