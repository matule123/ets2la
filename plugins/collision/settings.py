from dataclasses import dataclass

@dataclass
class CollisionSettings:
    emergency_threshold: float = 0.8
    bypass_steering_intensity: float = 0.3
    brake_during_bypass: float = 0.1

    def __post_init__(self):
        self.emergency_threshold = max(0.1, min(1.0, self.emergency_threshold))
        self.bypass_steering_intensity = max(0.0, min(1.0, self.bypass_steering_intensity))

# Global settings instance
settings = CollisionSettings()
