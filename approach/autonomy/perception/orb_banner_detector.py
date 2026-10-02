import cv2
import numpy as np
import os

class ORBBannerDetector:
    def __init__(self):
        # Load the template image
        template_path = os.path.join(os.path.dirname(__file__), 'banner_template.png')
        self.template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
        
        if self.template is None:
            raise FileNotFoundError(f"Could not find template image at {template_path}")
            
        self.h_template, self.w_template = self.template.shape
        
        # Initialize ORB detector with hyper-sensitive thresholds for synthetic Gazebo textures
        self.orb = cv2.ORB_create(
            nfeatures=2000, 
            fastThreshold=0, 
            edgeThreshold=0
        )
        
        # Pre-compute keypoints and descriptors for the template
        self.kp_template, self.des_template = self.orb.detectAndCompute(self.template, None)
        
        # Initialize the Brute Force Matcher
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # Minimum number of good matches required to confidently calculate the center
        self.min_match_count = 5

    def detect(self, frame):
        result = {
            "detected": False,
            "bbox": None,
            "center": None,
            "area": 0,
            "debug_frame": frame.copy()
        }
        
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find keypoints and descriptors in the current live frame
        kp_frame, des_frame = self.orb.detectAndCompute(gray_frame, None)
        
        if des_frame is None or len(kp_frame) < self.min_match_count:
            if des_frame is None:
                print(f"Template Features: {len(self.kp_template)} | Live Frame Features: 0 | Good Matches Found: 0")
            return result
            
        # Match descriptors using KNN (K-Nearest Neighbors)
        matches = self.bf.knnMatch(self.des_template, des_frame, k=2)
        
        # Apply Lowe's ratio test to filter out false positives
        # --- CHANGED TO 0.85 TO RELAX MATCHING STRICTNESS ---
        good_matches = []
        for m, n in matches:
            if m.distance < 0.85 * n.distance:
                good_matches.append(m)
                
        # --- DEBUG PRINT STATEMENT ---
        print(f"Template Features: {len(self.kp_template)} | "
              f"Live Frame Features: {len(kp_frame)} | "
              f"Good Matches Found: {len(good_matches)}")
        # -----------------------------
                
        # Draw the matches on the debug frame for visualization
        result["debug_frame"] = cv2.drawMatches(
            self.template, self.kp_template, 
            frame, kp_frame, 
            good_matches, None, 
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
        
        # If we have enough good matches, calculate the homography (perspective projection)
        if len(good_matches) >= self.min_match_count:
            src_pts = np.float32([self.kp_template[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            
            # Find the perspective transformation matrix
            matrix, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            
            if matrix is not None:
                # Map the 4 corners of the template to the live frame
                pts = np.float32([
                    [0, 0], 
                    [0, self.h_template - 1], 
                    [self.w_template - 1, self.h_template - 1], 
                    [self.w_template - 1, 0]
                ]).reshape(-1, 1, 2)
                
                dst = cv2.perspectiveTransform(pts, matrix)
                
                # Because cv2.drawMatches puts the template next to the frame, 
                # we need to shift the drawing coordinates for the debug view
                dst_draw = np.float32(dst) + np.float32([self.w_template, 0])
                
                # Draw the projected bounding polygon
                result["debug_frame"] = cv2.polylines(result["debug_frame"], [np.int32(dst_draw)], True, (0, 255, 0), 3)
                
                # Calculate the center of the projected polygon
                M = cv2.moments(dst)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    
                    # Calculate bounding box from the polygon points
                    x, y, w, h = cv2.boundingRect(dst)
                    
                    result["detected"] = True
                    result["bbox"] = (x, y, w, h)
                    result["center"] = (center_x, center_y)
                    result["area"] = w * h
                    
                    # Draw center point (shifted for debug view)
                    cv2.circle(result["debug_frame"], (center_x + self.w_template, center_y), 5, (0, 0, 255), -1)
                    
        return result
