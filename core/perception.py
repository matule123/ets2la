import logging
import numpy as np
import cv2
import mss


class Perception:
    """Computer Vision module for ETS2 (screen-capture based)."""

    def __init__(self, shared_state=None):
        # Shared state so detections can publish e.g. ai_confidence.
        self.shared_state = shared_state

        self.sct = mss.mss()
        # monitors[0] is the virtual "all monitors" entry; [1] is the primary.
        self.monitor = self.sct.monitors[1] if len(self.sct.monitors) > 1 else self.sct.monitors[0]

        # Lane detection uses our own OpenCV pipeline (detect_lanes).
        # The old ETS2LA HuggingFace model no longer exists and 404'd on every
        # start, so we don't try to download it anymore.
        self.model = None

        # Temporal smoothing for danger level
        self._last_danger_level = 0.0

        # Define the region of the screen to capture (GPS area)
        self.capture_region = {
            "top": int(self.monitor["height"] * 0.7),
            "left": int(self.monitor["width"] * 0.3),
            "width": int(self.monitor["width"] * 0.4),
            "height": int(self.monitor["height"] * 0.3)
        }
        # Road ahead — used for vision lane-keeping when driving WITHOUT a map.
        self.lane_region = {
            "top": int(self.monitor["height"] * 0.55),
            "left": int(self.monitor["width"] * 0.20),
            "width": int(self.monitor["width"] * 0.60),
            "height": int(self.monitor["height"] * 0.32),
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

    def _process_ai_output(self, predictions: list, width: int) -> float:
        """
        Converts raw AI model predictions into a normalized lane offset.
        Expected output: A list of floats or a single float representing offset.
        """
        if not predictions:
            return 0.0

        try:
            # If the model returns a list of predictions, average them
            if isinstance(predictions, list):
                val = sum(predictions) / len(predictions)
            else:
                val = predictions

            return np.clip(float(val), -1.0, 1.0)
        except Exception:
            return 0.0

    def detect_lanes(self) -> float:
        """
        Vision lane-keeping (used when driving WITHOUT a map).

        Splits detected edges into left/right lane lines by slope, estimates the
        lane centre, and returns the truck's offset from it in ``[-1, 1]``
        (negative = truck left of centre, positive = right).  Temporally smoothed.
        """
        frame = self.get_frame(self.lane_region)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 60, 160)
        h, w = edges.shape

        # Trapezoid ROI focused on the road ahead.
        mask = np.zeros_like(edges)
        poly = np.array([[(int(0.02 * w), h), (int(0.98 * w), h),
                          (int(0.62 * w), int(0.30 * h)), (int(0.38 * w), int(0.30 * h))]], np.int32)
        cv2.fillPoly(mask, poly, 255)
        edges = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 40, minLineLength=40, maxLineGap=60)
        cx = w / 2.0
        left_x, right_x = [], []
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < 0.4:      # ignore near-horizontal edges
                    continue
                mx = (x1 + x2) / 2.0
                (left_x if slope < 0 else right_x).append(mx)

        offset = None
        if left_x and right_x:
            lane_center = (np.mean(left_x) + np.mean(right_x)) / 2.0
            offset = (cx - lane_center) / cx
        elif left_x:                       # only left line → bias right a bit
            offset = (cx - (np.mean(left_x) + w * 0.18)) / cx
        elif right_x:
            offset = (cx - (np.mean(right_x) - w * 0.18)) / cx

        if offset is None:
            self._publish("ai_confidence", 0.0)
            # decay toward 0 (no lines → hold straight)
            self._last_lane = getattr(self, "_last_lane", 0.0) * 0.7
            return self._last_lane

        self._publish("ai_confidence", 0.5)
        offset = float(np.clip(offset, -1.0, 1.0))
        prev = getattr(self, "_last_lane", 0.0)
        self._last_lane = 0.4 * offset + 0.6 * prev   # temporal smoothing
        return self._last_lane

    def _publish(self, key, value):
        """Safely write to shared state (no-op if running standalone)."""
        if self.shared_state is not None:
            self.shared_state.set(key, value)

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

    def detect_obstacles(self) -> dict:
        """
        Detects obstacles ahead and returns ``{'level': 0..1, 'position': left/center/right}``.

        Two signals, combined conservatively to avoid phantom braking:
          * **Brake lights** (red mass) — the strongest cue a vehicle ahead is
            slowing; weighted heavily.
          * **Large object ahead** — a big contiguous contour in the lane region
            (a vehicle blocking the view); weighted lightly as a nudge.

        Note: screen vision is inherently limited.  Reliable traffic tracking
        ultimately needs the game's traffic shared-memory data.
        """
        frame = self.get_frame(self.obstacle_region)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        total_pixels = max(1, h * w)

        # --- 1) Brake lights (red) ---
        mask1 = cv2.inRange(hsv, np.array([0, 90, 90]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 90, 90]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(mask1, mask2)
        red_count = cv2.countNonZero(red_mask)
        red_danger = np.clip((red_count / total_pixels) * 12.0, 0.0, 1.0)

        # --- 2) Large object ahead (big dark/contrasting contour) ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 60, 160)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obj_danger = 0.0
        obj_cx = None
        if contours:
            biggest = max(contours, key=cv2.contourArea)
            area_frac = cv2.contourArea(biggest) / total_pixels
            # Only count genuinely large masses (a close vehicle), low weight.
            if area_frac > 0.18:
                obj_danger = np.clip((area_frac - 0.18) * 1.2, 0.0, 0.5)
                M = cv2.moments(biggest)
                if M["m00"] > 0:
                    obj_cx = int(M["m10"] / M["m00"])

        current_danger = float(np.clip(max(red_danger, red_danger * 0.6 + obj_danger),
                                       0.0, 1.0))

        # Temporal smoothing to steady the signal.
        alpha = 0.3
        self._last_danger_level = (alpha * current_danger) + ((1 - alpha) * self._last_danger_level)

        # Position from whichever signal is present (prefer red mass).
        position = "center"
        cx = None
        if red_count > 80:
            M = cv2.moments(red_mask)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
        elif obj_cx is not None:
            cx = obj_cx
        if cx is not None:
            center_x = w // 2
            if cx < center_x - 20:
                position = "left"
            elif cx > center_x + 20:
                position = "right"

        return {"level": self._last_danger_level, "position": position}
