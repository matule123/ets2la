import cv2
import numpy as np
from mss import mss
import logging
from typing import Dict, Any, Tuple

class Perception:
    """Computer Vision module for ETS2."""

    def __init__(self):
        self.sct = mss.mss()
        self.monitor = self.sct.monitor
        # Define the region of the screen to capture (GPS area)
        self.capture_region = {
            "top": int(self.monitor["height"] * 0.7),
            "left": int(self.monitor["width"] * 0.3),
            "width": int(self.monitor["width"] * 0.4),
            "height": int(self.monitor["height"] * 0.3)
        }
        # Define the region for obstacle detection (center of the screen, horizon)
        self.obstacle_region = {
            "top": int(self.monitor["height"] * 0.4),
            "left": int(self.monitor["width"] * 0.4),
            "width": int(self.monitor["width"] * 0.2),
            "height": int(self.monitor["height"] * 0.3)
        }
        # Load templates for navigation arrows
        self.templates = {
            "left": None,
            "right": None,
            "straight": None
        }

    def get_frame(self, region=None):
        """Captures a region of the screen."""
        target_region = region if region else self.capture_region
        img = self.sct.grab(target_region)
        frame = np.array(img)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame

    def detect_navigation_arrow(self) -> float:
        """Detects the navigation arrow direction."""
        frame = self.get_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.templates["left"] is not None:
            res_l = cv2.matchTemplate(gray, self.templates["left"], cv2.TM_CCOEFF_NORMED)
            res_r = cv2.matchTemplate(gray, self.templates["right"], cv2.TM_CCOEFF_NORMED)
            res_s = cv2.matchTemplate(gray, self.templates["straight"], cv2.TM_CCOEFF_NORMED)

            val_l = cv2.minMaxLoc(res_l)[1]
            val_r = cv2.minMaxLoc(res_r)[1]
            val_s = cv2.minMaxLoc(res_s)[1]

            if val_l > 0.8 and val_l > val_r and val_l > val_s: return -1.0
            if val_r > 0.8 and val_r > val_l and val_r > val_s: return 1.0
            if val_s > 0.8 and val_s > val_l and val_s > val_r: return 0.0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        M = cv2.moments(mask)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            center = self.capture_region["width"] // 2
            return np.clip((cx - center) / center, -1.0, 1.0)

        return 0.0

    def detect_lanes(self) -> float:
        """Detects lane offset."""
        frame = self.get_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([
            (0, height), (width, height),
            (width // 2 + 150, height // 2), (width // 2 - 150, height // 2),
        ], np.int32)
        cv2.fillPoly(mask, [polygon], 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        lines = cv2.HoughLinesP(cropped_edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)
        if lines is not None:
            avg_x = np.mean([line[0][0] for line in lines])
            return (avg_x - (width // 2)) / (width // 2)
        return 0.0

    def detect_toll(self) -> bool:
        """
        Detects if the truck is at a toll booth.
        Looks for specific color signatures of toll signs.
        """
        frame = self.get_frame()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Toll booths often have bright yellow or blue signs.
        # This is a simplified detection for the "toll" signature.
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Check if the yellow area is significant enough in the upper half of the screen
        height, width = mask.shape
        roi = mask[0:height//2, 0:width]
        if cv2.countNonZero(roi) > 500: # Threshold for detection
            return True
        return False

    def detect_obstacles(self) -> float:
        """
        Detects obstacles in front (e.g., red brake lights).
        Returns: Danger level 0.0 (clear) to 1.0 (critical).
        """
        frame = self.get_frame(self.obstacle_region)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Look for red colors (brake lights)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        full_mask = cv2.bitwise_or(mask1, mask2)

        red_pixel_count = cv2.countNonZero(full_mask)
        total_pixels = full_mask.shape[0] * full_mask.shape[1]
        danger_level = red_pixel_count / total_pixels

        return np.clip(danger_level * 10, 0.0, 1.0)
