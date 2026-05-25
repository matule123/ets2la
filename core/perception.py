from core.ai_model import Model, MODEL_CONFIG

class Perception:
    """Computer Vision module for ETS2."""

    def __init__(self):
        self.sct = mss.mss()
        self.monitor = self.sct.monitor

        # Initialize AI Model for Lane Detection
        self.model = Model(
            HF_owner=MODEL_CONFIG["HF_OWNER"],
            HF_repository=MODEL_CONFIG["HF_REPOSITORY"],
            HF_model_folder=MODEL_CONFIG["HF_FOLDER"]
        )
        self.model.load_model()

        # Temporal smoothing for danger level
        self._last_danger_level = 0.0

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
        """Detects lane offset using AI with traditional CV fallback.
        Updates 'ai_confidence' in shared state.
        """
        frame = self.get_frame()

        # 1. Try AI Model
        if self.model.loaded:
            try:
                predictions = self.model.detect(frame)
                if predictions:
                    # We can estimate confidence based on prediction variance or specific model output
                    # For now, we simulate confidence based on whether the model returned a value
                    confidence = 0.95 if predictions else 0.0
                    self.shared_state.set("ai_confidence", confidence)
                    return self._process_ai_output(predictions, frame.shape[1])
            except Exception as e:
                logging.error(f"AI Lane Detection failed: {e}")
                self.shared_state.set("ai_confidence", 0.0)

        # 2. Fallback to Traditional CV
        self.shared_state.set("ai_confidence", 0.3) # Lower confidence for traditional CV
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

        self.shared_state.set("ai_confidence", 0.0)
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

    def detect_obstacles(self) -> dict:
        """
        Detects obstacles in front (e.g., red brake lights).
        Returns: Dictionary with 'level' (0.0 to 1.0) and 'position' ('left', 'right', 'center').
        Includes temporal smoothing.
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
        current_danger = np.clip((red_pixel_count / total_pixels) * 10, 0.0, 1.0)

        # Temporal smoothing
        alpha = 0.3
        self._last_danger_level = (alpha * current_danger) + ((1 - alpha) * self._last_danger_level)

        # Determine position of the red mass
        position = "center"
        if red_pixel_count > 100:
            M = cv2.moments(full_mask)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                center_x = frame.shape[1] // 2
                if cx < center_x - 20:
                    position = "left"
                elif cx > center_x + 20:
                    position = "right"

        return {"level": self._last_danger_level, "position": position}
