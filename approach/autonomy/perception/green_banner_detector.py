import cv2
import numpy as np

class GreenBannerDetector:
    def __init__(self):
        # 1. Green limits (for the banner background)
        self.lower_green = np.array([40, 50, 50])
        self.upper_green = np.array([90, 255, 255])
        
        # 2. White limits (for the AEROTHON text)
        # Low saturation (0-50), High value/brightness (200-255)
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 50, 255])
        
        self.min_area = 500

    def detect(self, frame):
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Mask 1: Find everything green
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # Find contours of the green areas
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        result = {
            "detected": False,
            "bbox": None,
            "center": None,
            "area": 0,
            "debug_frame": frame.copy()
        }
        
        valid_contours = []
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(cnt)
                
                # THE NEW LOGIC: Isolate this specific green blob's region
                roi_hsv = hsv[y:y+h, x:x+w]
                
                # Mask 2: Search for white text INSIDE this green region
                white_mask = cv2.inRange(roi_hsv, self.lower_white, self.upper_white)
                
                # Count how many white pixels exist inside this green bounding box
                white_pixel_count = cv2.countNonZero(white_mask)
                
                # If there are white pixels (the text), it's the banner!
                # Grass will have ~0 white pixels.
                if white_pixel_count > 15:  
                    valid_contours.append(cnt)
                else:
                    # Draw rejected grass blobs in red for debugging
                    cv2.rectangle(result["debug_frame"], (x, y), (x + w, y + h), (0, 0, 255), 1)
                    cv2.putText(result["debug_frame"], "Grass (No Text)", (x, y - 5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        if not valid_contours:
            return result
            
        # Out of the blobs that have both Green AND White, pick the largest
        best_contour = max(valid_contours, key=cv2.contourArea)
        area = cv2.contourArea(best_contour)
        
        x, y, w, h = cv2.boundingRect(best_contour)
        center_x = x + (w // 2)
        center_y = y + (h // 2)
        
        result["detected"] = True
        result["bbox"] = (x, y, w, h)
        result["center"] = (center_x, center_y)
        result["area"] = area
        
        # Draw the successfully verified banner in thick green
        cv2.rectangle(result["debug_frame"], (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.circle(result["debug_frame"], (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.putText(result["debug_frame"], "TARGET VERIFIED", (x, y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
        return result
