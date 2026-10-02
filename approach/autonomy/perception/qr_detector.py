import cv2
import numpy as np

class QRDetector:
    def __init__(self):
        # Standard OpenCV QR detector
        self.detector = cv2.QRCodeDetector()

    def detect(self, frame):
        height, width = frame.shape[:2]
        screen_center_x = width // 2
        screen_center_y = height // 2

        result = {
            "detected": False,
            "qr_data": "",
            "center": None,
            "error_x": 0,
            "error_y": 0,
            "debug_frame": frame.copy()
        }

        # --- ATTEMPT 1: Strict QR Decoding ---
        data, points = "", None
        try:
            # OpenCV 4.6.0 bug workaround: catch C++ crash on partial QR codes
            data, points, _ = self.detector.detectAndDecode(frame)
        except cv2.error:
            points = None

        if points is not None and data:
            points = points[0]
            center_x = int(points[:, 0].mean())
            center_y = int(points[:, 1].mean())

            result.update({
                "detected": True,
                "qr_data": data,
                "center": (center_x, center_y),
                "error_x": center_x - screen_center_x,
                "error_y": center_y - screen_center_y
            })

            # Draw exact bounds if fully decoded
            for i in range(4):
                p1 = tuple(points[i].astype(int))
                p2 = tuple(points[(i + 1) % 4].astype(int))
                cv2.line(result["debug_frame"], p1, p2, (0, 255, 0), 3)

            cv2.circle(result["debug_frame"], (center_x, center_y), 6, (0, 0, 255), -1)
            cv2.putText(result["debug_frame"], f"FULL QR: {data}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
        else:
            # --- ATTEMPT 2: Color-Agnostic Edge Density Tracking ---
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Find all sharp edges in the image
            edges = cv2.Canny(gray, 100, 200)
            
            # Use a massive morphological kernel to smudge all the dense QR edges into one solid blob
            kernel = np.ones((25, 25), np.uint8)
            dense_mask = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
            
            # Find the outlines of these dense areas
            contours, _ = cv2.findContours(dense_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Assume the largest block of high-density edges is the QR code
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Filter out small background noise
                if cv2.contourArea(largest_contour) > 5000:
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    
                    center_x = x + (w // 2)
                    center_y = y + (h // 2)
                    
                    result.update({
                        "detected": True,
                        "qr_data": "PARTIAL_LOCK",
                        "center": (center_x, center_y),
                        "error_x": center_x - screen_center_x,
                        "error_y": center_y - screen_center_y
                    })
                    
                    # Draw a yellow warning box to indicate a partial/fallback lock
                    cv2.rectangle(result["debug_frame"], (x, y), (x+w, y+h), (0, 255, 255), 3)
                    cv2.circle(result["debug_frame"], (center_x, center_y), 6, (0, 0, 255), -1)
                    cv2.putText(result["debug_frame"], "EDGE-DENSITY LOCK", (20, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            # Picture-in-Picture Debug: Show the computer's edge-density map
            mask_small = cv2.resize(dense_mask if 'dense_mask' in locals() else np.zeros((120, 160), dtype=np.uint8), (160, 120))
            mask_bgr = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
            result["debug_frame"][0:120, width-160:width] = mask_bgr
            cv2.rectangle(result["debug_frame"], (width-160, 0), (width, 120), (255, 255, 255), 1)

        if not result["detected"]:
            cv2.putText(result["debug_frame"], 'SEARCHING FOR QR...', (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Draw absolute camera crosshairs
        cv2.line(result["debug_frame"], (screen_center_x, 0), (screen_center_x, height), (255, 255, 255), 1)
        cv2.line(result["debug_frame"], (0, screen_center_y), (width, screen_center_y), (255, 255, 255), 1)

        return result
