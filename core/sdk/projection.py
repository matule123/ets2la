import math
from typing import Tuple, List, Optional

class ARProjection:
    """
    Handles 3D to 2D coordinate projection for the AR HUD.
    Based on the perspective projection used in professional simulation overlays.
    """
    def __init__(self, screen_width: int = 1920, screen_height: int = 1080, fov: float = 90.0):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.fov_rad = math.radians(fov)
        # Calculate window distance based on FOV
        self.window_distance = ((screen_height * (4/3) / 2) / math.tan(self.fov_rad / 2))

    def project(self,
                head_pos: Tuple[float, float, float],
                head_rot: Tuple[float, float, float],
                target_pos: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        """
        Projects a 3D world coordinate to 2D screen coordinates.
        head_pos: (x, y, z) of the truck/camera
        head_rot: (yaw, pitch, roll) in radians
        target_pos: (x, y, z) of the object to project
        """
        # 1. Relative positioning
        rel_x = target_pos[0] - head_pos[0]
        rel_y = target_pos[1] - head_pos[1]
        rel_z = target_pos[2] - head_pos[2]

        # 2. Rotation transformation (Simple Yaw only for basic implementation)
        # In a full system, this would be a 3x3 rotation matrix
        yaw = head_rot[0]
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # Rotate around Y axis (Up)
        final_x = rel_x * cos_y - rel_z * sin_y
        final_z = rel_x * sin_y + rel_z * cos_y
        final_y = rel_y # Keep Y as is for simple projection

        # 3. Z-Clipping (Only project objects in front of the camera)
        if final_z <= 0:
            return None

        # 4. Perspective Projection
        screen_x = (final_x / final_z) * self.window_distance + (self.screen_width / 2)
        screen_y = (final_y / final_z) * self.window_distance + (self.screen_height / 2)

        # 5. Screen Clipping
        if 0 <= screen_x <= self.screen_width and 0 <= screen_y <= self.screen_height:
            return (screen_x, screen_y)

        return None
