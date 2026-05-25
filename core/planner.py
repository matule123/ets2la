from enum import Enum, auto
import logging

class SystemState(Enum):
    IDLE = auto()
    CRUISE = auto()
    FOLLOW_LANE = auto()
    AVOID_OBSTACLE = auto()
    OVERTAKING = auto()
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
        self.active_blinker = "off"
        logging.info("UltraPilot Planner initialized.")

    def update(self, perception_data, telemetry_data, delta_time):
        """
        Determines the best state to be in.
        """
        self.state_duration += delta_time
        voice_alert = None

        obstacle = perception_data.get('obstacle', {'level': 0, 'position': 'center'})
        danger_level = obstacle.get('level', 0)
        obstacle_pos = obstacle.get('position', 'center')

        # 1. Emergency First
        if danger_level > 0.7:
            if self.current_state != SystemState.EMERGENCY:
                voice_alert = "Emergency stop! Collision imminent!"
            self.set_state(SystemState.EMERGENCY)
            self.set_blinker("off")
            return self.current_state, voice_alert

        # 2. Toll Payment
        if perception_data.get('toll_detected', False):
            if self.current_state != SystemState.PAY_TOLL:
                voice_alert = "Toll booth detected. Stopping to pay."
            self.set_state(SystemState.PAY_TOLL)
            self.set_blinker("off")
            return self.current_state, voice_alert

        # 3. Overtaking / Bypass Logic
        if danger_level > 0.3:
            if self.current_state != SystemState.OVERTAKING:
                voice_alert = "Obstacle detected. Initiating bypass maneuver."
                self.set_state(SystemState.OVERTAKING)

            # Set blinker based on where we need to move
            # If obstacle is center or left, we move right. If right, move left.
            target_blinker = "right" if obstacle_pos in ["center", "left"] else "left"
            self.set_blinker(target_blinker)

            return self.current_state, voice_alert

        # 4. Lane Following / Navigation
        if abs(perception_data.get('lane_offset', 0)) > 0.1 or abs(perception_data.get('nav_direction', 0)) > 0.2:
            if self.current_state == SystemState.CRUISE:
                voice_alert = "Adjusting steering for navigation."
            self.set_state(SystemState.FOLLOW_LANE)

            # Signal if navigation direction is strong
            nav_dir = perception_data.get('nav_direction', 0)
            if abs(nav_dir) > 0.6:
                self.set_blinker("left" if nav_dir < 0 else "right")
            else:
                self.set_blinker("off")
        else:
            if self.current_state != SystemState.CRUISE:
                voice_alert = "Lanes centered. Cruising."
            self.set_state(SystemState.CRUISE)
            self.set_blinker("off")

        return self.current_state, voice_alert

    def set_state(self, new_state):
        if self.current_state != new_state:
            logging.info(f"System transitioning: {self.current_state.name} -> {new_state.name}")
            self.current_state = new_state
            self.state_duration = 0.0

    def set_blinker(self, side):
        if self.active_blinker != side:
            self.active_blinker = side
            # We don't call the controller here, the engine will pick up the state
            # or we can push it to shared state.
