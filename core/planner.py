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

        # 2. Toll Payment — DISABLED by default.
        #    Vision-based toll detection (yellow pixels) was firing constantly and
        #    made the truck stop every second ("Stopping to pay" flicker), which
        #    ruined normal driving. Only act on it when explicitly enabled.
        if perception_data.get('toll_detected', False) and perception_data.get('enable_toll', False):
            if self.current_state != SystemState.PAY_TOLL:
                voice_alert = "Toll booth detected. Stopping to pay."
            self.set_state(SystemState.PAY_TOLL)
            self.set_blinker("off")
            return self.current_state, voice_alert

        # 3. Obstacle ahead — slow down / brake (no blind vision-based lane change).
        #    Auto-overtaking via screen CV is unreliable, so mid-level danger maps to
        #    a controlled AVOID_OBSTACLE (brake), which the Autopilot plugin handles.
        if danger_level > 0.3:
            if self.current_state != SystemState.AVOID_OBSTACLE:
                voice_alert = "Obstacle ahead. Slowing down."
            self.set_state(SystemState.AVOID_OBSTACLE)
            self.set_blinker("off")
            return self.current_state, voice_alert

        # 4. Lane Following / Navigation — with HYSTERESIS so the state doesn't
        #    flicker CRUISE<->FOLLOW_LANE on tiny offset noise (that flicker also
        #    spammed the voice and jittered the wheel).
        lane = abs(perception_data.get('lane_offset', 0))
        nav = abs(perception_data.get('nav_direction', 0))
        if self.current_state == SystemState.FOLLOW_LANE:
            following = lane > 0.06 or nav > 0.12   # stay following until well centred
        else:
            following = lane > 0.18 or nav > 0.28   # only start following on a real deviation

        if following:
            self.set_state(SystemState.FOLLOW_LANE)
            nav_dir = perception_data.get('nav_direction', 0)
            if abs(nav_dir) > 0.6:
                self.set_blinker("left" if nav_dir < 0 else "right")
            else:
                self.set_blinker("off")
        else:
            self.set_state(SystemState.CRUISE)
            self.set_blinker("off")

        # No voice on routine lane adjustments (it was talking non-stop).
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
