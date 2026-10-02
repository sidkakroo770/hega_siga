import cv2
import numpy as np

class HybridBannerDetector:
    def __init__(self, panel_only=False):
        # Tighter green mask is working perfectly!
        self.lower_green = np.array([45, 100, 80])
        self.upper_green = np.array([75, 255, 255])
        
        # Minimum size requirement
        self.min_area = 700
        self.panel_only = panel_only
        self.tracked_bbox = None
        self.tracked_area = None

    def reset_track(self):
        self.tracked_bbox = None
        self.tracked_area = None

    def lock_target(self, result):
        """Seed approach tracking from the target actually centered by camera."""
        if not result["detected"]:
            raise ValueError("Cannot lock an undetected target")
        self.tracked_bbox = result["bbox"]
        self.tracked_area = result["area"]
        self.panel_only = True

    def matches_panel(self, bbox, area, frame_shape):
        x, y, w, h = bbox
        height, width = frame_shape[:2]
        # Use the observed target shape, not an assumed camera projection.
        # Clipped targets have unreliable centers during forward approach.
        if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
            return False
        if self.tracked_bbox is None:
            return True
        px, py, pw, ph = self.tracked_bbox
        intersection = max(0, min(x+w, px+pw)-max(x, px)) * max(
            0, min(y+h, py+ph)-max(y, py))
        overlap = intersection / min(w*h, pw*ph)
        return (overlap >= 0.65
                and 0.75 <= area / self.tracked_area <= 1.30
                and 0.80 <= (w/h) / (pw/ph) <= 1.25)

    def detect(self, frame, relaxed_approach=False):
        # 0. Get screen dimensions to calculate errors
        height, width = frame.shape[:2]
        screen_center_x = width // 2
        screen_center_y = height // 2

        result = {
            "detected": False,
            "bbox": None,
            "center": None,
            "area": 0,
            "largest_area": 0.0,
            "rejection_reason": "no_green_contours",
            "clipped": False,
            "error_x": 0, # Added for velocity control
            "error_y": 0, # Added for velocity control
            "debug_frame": frame.copy()
        }
        
        # 1. COLOR DETECTION
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # 2. MORPHOLOGICAL CLEANUP
        kernel = np.ones((5,5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        
        # --- PICTURE IN PICTURE DEBUG ---
        mask_small = cv2.resize(green_mask, (160, 120))
        mask_bgr = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
        result["debug_frame"][0:120, 0:160] = mask_bgr
        cv2.rectangle(result["debug_frame"], (0, 0), (160, 120), (255, 255, 255), 1)
        cv2.putText(result["debug_frame"], "MASK", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        # --------------------------------
        
        # Find contours of the green blobs
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = []
        
        rejection_counts = {}
        for cnt in contours:
            area = cv2.contourArea(cnt)
            result["largest_area"] = max(result["largest_area"], area)
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = w / float(h)
            extent = area / (w * h)
            reason = None
            if area <= self.min_area:
                reason = "area_below_minimum"
            elif not relaxed_approach and not 0.8 < aspect_ratio < 5.0:
                reason = "aspect_ratio"
            elif extent <= 0.15:
                reason = "low_extent"
            elif not relaxed_approach:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.05 * peri, True)
                if not 4 <= len(approx) <= 16:
                    reason = "polygon_shape"
                elif self.panel_only and not self.matches_panel(
                        (x, y, w, h), area, frame.shape):
                    reason = "target_changed_or_clipped"
            if reason is None:
                valid_contours.append(cnt)
            else:
                rejection_counts[reason] = rejection_counts.get(reason, 0) + 1
                if area > self.min_area:
                    cv2.putText(result["debug_frame"], "REJ: " + reason,
                                (x, max(15, y-5)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.4, (0, 0, 255), 1)
        if rejection_counts:
            result["rejection_reason"] = ",".join(
                f"{reason}:{count}" for reason, count in rejection_counts.items())

        if not valid_contours:
            # Draw camera center crosshairs even when nothing is detected
            cv2.line(result["debug_frame"], (screen_center_x, 0), (screen_center_x, height), (255, 255, 255), 1)
            cv2.line(result["debug_frame"], (0, screen_center_y), (width, screen_center_y), (255, 255, 255), 1)
            return result
            
        best_contour = max(valid_contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(best_contour)
        
        center_x = x + (w // 2)
        center_y = y + (h // 2)
        
        result["detected"] = True
        result["rejection_reason"] = "none"
        result["clipped"] = (x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1)
        result["bbox"] = (x, y, w, h)
        result["center"] = (center_x, center_y)
        result["area"] = cv2.contourArea(best_contour)
        if self.panel_only:
            self.tracked_bbox = result["bbox"]
            self.tracked_area = result["area"]
        
        # Calculate the crucial error offset values for the mission runner
        result["error_x"] = center_x - screen_center_x
        result["error_y"] = center_y - screen_center_y
        
        # Draw bounding box and target center
        cv2.rectangle(result["debug_frame"], (x, y), (x+w, y+h), (0, 255, 0), 3)
        cv2.circle(result["debug_frame"], (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.putText(result["debug_frame"], "BANNER LOCKED", (x, y-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw absolute camera crosshairs
        cv2.line(result["debug_frame"], (screen_center_x, 0), (screen_center_x, height), (255, 255, 255), 1)
        cv2.line(result["debug_frame"], (0, screen_center_y), (width, screen_center_y), (255, 255, 255), 1)
        
        return result
