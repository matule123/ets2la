import logging
import time
from typing import Dict, List
from core.modules.base_module import BaseModule

class Vehicle:
    """Represents a detected vehicle in the traffic."""
    def __init__(self, id: int, position: float, speed: float):
        self.id = id
        self.position = position # Relative to truck
        self.speed = speed
        self.last_seen = time.time()

class TrafficAnalysis(BaseModule):
    """
    Core module for tracking traffic patterns and vehicle behavior.
    Processes raw perception data into a structured traffic model.
    """
    def __init__(self, engine):
        super().__init__(engine)
        self.tracked_vehicles: Dict[int, Vehicle] = {}
        self.next_id = 0
        logging.info("TrafficAnalysis module initialized.")

    def on_start(self):
        logging.info("TrafficAnalysis module started.")

    def on_stop(self):
        logging.info("TrafficAnalysis module stopped.")

    def update(self, delta_time: float):
        # 1. Get the latest obstacle data from shared state
        obstacle = self.engine.shared_state.get("obstacle", {"level": 0, "position": "center"})
        level = obstacle.get("level", 0)
        pos = obstacle.get("position", "center")

        # 2. Simple tracking logic
        # In a real system, this would use a Kalman filter or similar tracking
        if level > 0.1:
            # We found something. For simplicity, we track the main obstacle as vehicle 0
            veh = self.tracked_vehicles.get(0)
            if veh:
                veh.position = pos
                veh.last_seen = time.time()
            else:
                self.tracked_vehicles[0] = Vehicle(0, pos, 0.0)
                logging.info("TrafficAnalysis: New vehicle tracked.")

        # 3. Clean up stale vehicles
        now = time.time()
        self.tracked_vehicles = {k: v for k, v in self.tracked_vehicles.items()
                                 if now - v.last_seen < 2.0}

        # 4. Push traffic summary back to shared state
        traffic_summary = {
            "vehicle_count": len(self.tracked_vehicles),
            "congestion_level": 0.0 if not self.tracked_vehicles else (len(self.tracked_vehicles) * 0.2),
            "primary_obstacle": self.tracked_vehicles[0].position if 0 in self.tracked_vehicles else "none"
        }
        self.engine.shared_state.set("traffic_analysis", traffic_summary)
