import cv2
import numpy as np
import sys
import os
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from autonomy.perception.hybrid_banner_detector import HybridBannerDetector

latest_frame = None

def on_image(msg):
    global latest_frame
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    latest_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def send_velocity_cmd(master, vy, vz):
    """
    Commands velocity in meters per second.
    vy: right (positive) / left (negative)
    vz: down (positive) / up (negative) - NED frame standard
    """
    type_mask = int(0b0000111111000111) 
    master.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, type_mask,
            0, 0, 0,    
            0, vy, vz,  
            0, 0, 0,    
            0, 0)       
    )

def main():
    global latest_frame
    
    print("Connecting to ArduPilot via MAVLink...")
    master = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    master.wait_heartbeat()
    print("Heartbeat received! Ready for 2-Axis autonomous tracking.")

    detector = HybridBannerDetector()
    node = Node()
    topic = "/iris/camera_forward/image_raw"
    node.subscribe(Image, topic, on_image)

    # Controller tuning
    Kp_y = 0.003  # Aggressiveness for left/right
    Kp_z = 0.003  # Aggressiveness for up/down
    deadzone = 30 # Pixel tolerance to prevent jittering

    try:
        while True:
            if latest_frame is not None:
                frame = latest_frame.copy()
                frame_height, frame_width = frame.shape[:2]
                
                # Calculate absolute center of the camera FOV
                screen_center_x = frame_width // 2
                screen_center_y = frame_height // 2
                
                result = detector.detect(frame)
                
                if result["detected"]:
                    banner_center_x, banner_center_y = result["center"]
                    
                    # Calculate pixel error for both axes
                    error_x = banner_center_x - screen_center_x
                    error_y = banner_center_y - screen_center_y
                    
                    vy = 0.0
                    vz = 0.0
                    
                    # Horizontal Correction (Left/Right)
                    if abs(error_x) > deadzone:
                        vy = max(-1.0, min(1.0, error_x * Kp_y))
                        
                    # Vertical Correction (Up/Down)
                    if abs(error_y) > deadzone:
                        vz = max(-1.0, min(1.0, error_y * Kp_z))
                        
                    # Send commands
                    if vy != 0.0 or vz != 0.0:
                        send_velocity_cmd(master, vy, vz)
                        cv2.putText(result["debug_frame"], f"TRACKING | vy: {vy:.2f}, vz: {vz:.2f}", 
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    else:
                        send_velocity_cmd(master, 0, 0)
                        cv2.putText(result["debug_frame"], "TARGET CENTERED (2D)", 
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                else:
                    # Target lost, hold position
                    send_velocity_cmd(master, 0, 0)
                    cv2.putText(result["debug_frame"], "SEARCHING...", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # Draw the crosshairs of the camera FOV
                cv2.line(result["debug_frame"], (screen_center_x, 0), 
                         (screen_center_x, frame_height), (255, 255, 255), 1)
                cv2.line(result["debug_frame"], (0, screen_center_y), 
                         (frame_width, screen_center_y), (255, 255, 255), 1)

                cv2.imshow("Hybrid Autonomous Tracking", result["debug_frame"])
                
            if cv2.waitKey(30) & 0xFF == 27: # Press ESC to stop
                break
                
    except KeyboardInterrupt:
        pass
        
    send_velocity_cmd(master, 0, 0) # Failsafe stop
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
