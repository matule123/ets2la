from enum import Enum, auto
import logging

class SystemState(Enum):
    IDLE = auto()
    CRUISE = auto()
    FOLLOW_LANE = auto()
    AVOID_OBSTACLE = auto()
    PAY_TOLL = auto()
    EMERGENCY = auto()

class UltraPilotPlanner:
    """
    The Planning Layer of ETS2-UltraPilot.
    Decides the high-level state based on perception and telemetry.
    """

    def __init__(self):
        self.current_state = SystemState.IDLE
        self.state_duration = 0.0
        logging.info("UltraPilot Planner initialized.")

    def update(self, perception_data, telemetry_data, delta_time):
        """
        Determines the best state to be in.
        """
        self.state_duration += delta_time
        voice_alert = None

        # 1. Emergency First
        if perception_data.get('danger_level', 0) > 0.7:
            if self.current_state != SystemState.EMERGENCY:
                voice_alert = "Emergency stop! Collision imminent!"
            self.set_state(SystemState.EMERGENCY)
            return self.current_state, voice_alert

        # 2. Toll Payment
        if perception_data.get('toll_detected', False):
            if self.current_state != SystemState.PAY_TOLL:
                voice_alert = "Toll booth detected. Stopping to pay."
            self.set_state(SystemState.PAY_TOLL)
            return self.current_state, voice_alert

        # 3. Obstacle Avoidance
        if perception_data.get('danger_level', 0) > 0.3:
            if self.current_state != SystemState.AVOID_OBSTACLE:
                voice_alert = "Caution. Slowing down for obstacle."
            self.set_state(SystemState.AVOID_OBSTACLE)
            return self.current_state, voice_alert

        # 4. Lane Following / Navigation
        if abs(perception_data.get('lane_offset', 0)) > 0.1 or abs(perception_data.get('nav_direction', 0)) > 0.2:
            if self.current_state == SystemState.CRUISE:
                voice_alert = "Adjusting steering for navigation."
            self.set_state(SystemState.FOLLOW_LANE)
        else:
            if self.current_state != SystemState.CRUISE:
                voice_alert = "Lanes centered. Cruising."
            self.set_state(SystemState.CRUISE)

        return self.current_state, voice_alert

    def set_state(self, new_state):
        if self.current_state != new_state:
            logging.info(f"System transitioning: {self.current_state.name} -> {new_state.name}")
            self.current_state = new_state
            self.state_duration = 0.0
