import cv2
import math
import numpy as np

class HybridBannerDetector:
    def __init__(self, panel_only=False, lower_green=(45, 100, 80),
                 upper_green=(75, 255, 255), min_area_at_640x480=700,
                 morph_kernel_size=5, aspect_ratio_bounds=(0.8, 5.0),
                 min_extent=0.15):
        # Gazebo baseline. Real IMX296/lens/lighting values require calibration.
        lower = tuple(int(v) for v in lower_green)
        upper = tuple(int(v) for v in upper_green)
        if (len(lower) != 3 or len(upper) != 3
                or not all(0 <= lower[i] <= upper[i] <= (179 if i == 0 else 255)
                           for i in range(3))):
            raise ValueError("invalid OpenCV HSV bounds")
        if not math.isfinite(min_area_at_640x480) or min_area_at_640x480 <= 0:
            raise ValueError("minimum banner area must be positive")
        if not isinstance(morph_kernel_size, int) or morph_kernel_size < 1 or morph_kernel_size % 2 == 0:
            raise ValueError("morphology kernel size must be a positive odd integer")
        aspect_low, aspect_high = (float(v) for v in aspect_ratio_bounds)
        if (not all(math.isfinite(v) for v in (aspect_low, aspect_high, min_extent))
                or not 0 < aspect_low < aspect_high
                or not 0 < min_extent < 1):
            raise ValueError("invalid banner shape bounds")
        self.lower_green = np.array(lower, dtype=np.uint8)
        self.upper_green = np.array(upper, dtype=np.uint8)
        self.min_area = float(min_area_at_640x480)
        self.morph_kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
        self.aspect_ratio_bounds = (aspect_low, aspect_high)
        self.min_extent = float(min_extent)
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

    def detect(self, frame, relaxed_approach=False, debug=True):
        # 0. Get screen dimensions to calculate errors
        height, width = frame.shape[:2]
        min_area = self.min_area * width * height / (640 * 480)
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
            "debug_frame": frame.copy() if debug else None
        }
        
        # 1. COLOR DETECTION
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # 2. MORPHOLOGICAL CLEANUP
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, self.morph_kernel)
        
        # --- PICTURE IN PICTURE DEBUG ---
        if debug:
            preview_w, preview_h = min(160, width), min(120, height)
            mask_small = cv2.resize(green_mask, (preview_w, preview_h))
            mask_bgr = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
            result["debug_frame"][0:preview_h, 0:preview_w] = mask_bgr
            cv2.rectangle(result["debug_frame"], (0, 0), (preview_w-1, preview_h-1), (255, 255, 255), 1)
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
            if area <= min_area:
                reason = "area_below_minimum"
            elif not relaxed_approach and not self.aspect_ratio_bounds[0] < aspect_ratio < self.aspect_ratio_bounds[1]:
                reason = "aspect_ratio"
            elif extent <= self.min_extent:
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
                if debug and area > min_area:
                    cv2.putText(result["debug_frame"], "REJ: " + reason,
                                (x, max(15, y-5)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.4, (0, 0, 255), 1)
        if rejection_counts:
            result["rejection_reason"] = ",".join(
                f"{reason}:{count}" for reason, count in rejection_counts.items())

        if not valid_contours:
            # Draw camera center crosshairs even when nothing is detected
            if debug:
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
        if debug:
            cv2.rectangle(result["debug_frame"], (x, y), (x+w, y+h), (0, 255, 0), 3)
            cv2.circle(result["debug_frame"], (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.putText(result["debug_frame"], "BANNER LOCKED", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.line(result["debug_frame"], (screen_center_x, 0), (screen_center_x, height), (255, 255, 255), 1)
            cv2.line(result["debug_frame"], (0, screen_center_y), (width, screen_center_y), (255, 255, 255), 1)
        
        return result
